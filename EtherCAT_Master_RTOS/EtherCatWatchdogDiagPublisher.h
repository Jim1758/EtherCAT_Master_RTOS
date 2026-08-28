#pragma once

#include "EtherCatMaster.h"
#include "SHMManager.h"
#include "GlobalConfig.h"

#include <cstdint>
#include <cstring>

// ============================================================================
// Stage 12E.7D - OSCARMAX_ECAT_WATCHDOG_DIAG Priority-50 Publisher
// ============================================================================
//
// Ownership:
//
//     Priority-64
//         Reads physical ESC registers and owns EtherCAT NIC access.
//         Publishes only an owner-safe RT shadow.
//
//     Priority-50
//         Calls PublishEtherCatWatchdogDiagHealth() from the existing
//         approximately-10ms supervisory loop.
//
//     Windows ENI Tool
//         Read-only consumer of OSCARMAX_ECAT_WATCHDOG_DIAG.
//
// IMPORTANT:
//
//     This publisher performs ZERO EtherCAT transactions.
//
//     It only calls OSCARMAX_ECAT_WatchdogDiagRt_Read(), which copies an
//     already-existing Priority-64 RT shadow through a bounded seqlock seam.
//
// ESC source values already captured by Stage 12E.7A:
//
//     0x0400..0x0401  Watchdog Divider
//     0x0420..0x0421  Watchdog Time Process Data
//     0x0440..0x0441  Watchdog Status Process Data
//     0x0442          Watchdog Counter Process Data
//
// EnabledSmMask:
//     Bit N = live SyncManager N has ControlByte bit 6
//             (Watchdog Trigger Enable) set.
//
// Source fingerprint:
//
//     OSCARMAX_ECAT_WATCHDOG_DIAG_P50_12E7D_20260827
//
// ============================================================================


// Defined by EtherCatMaster_DC_Runtime.cpp (Stage 12E.7A).
//
// Lock-free / wait-free reader.
// A concurrent P64 shadow update simply causes false to be returned.
// P50 keeps the previous coherent SHM value and tries again on the next pass.
extern "C" bool OSCARMAX_ECAT_WatchdogDiagRt_Read(
    uint16_t slavePosition,
    uint32_t * valid,
    uint16_t * watchdogDivider,
    uint16_t * watchdogTimeProcessData,
    uint16_t * watchdogStatusProcessData,
    uint8_t * watchdogCounterProcessData,
    uint16_t * enabledSmMask,
    int32_t * lastTransportResult,
    uint64_t * lastUpdateTick);


// ============================================================================
// Priority-50 Publisher
// ============================================================================

inline void PublishEtherCatWatchdogDiagHealth(
    EtherCatMaster* masterCore)
{
    if (masterCore == nullptr)
    {
        return;
    }

    SHM_ECAT_WatchdogDiagData* watchdogDiag =
        GetEtherCatWatchdogDiagSharedMemoryData();

    if (watchdogDiag == nullptr)
    {
        return;
    }


    // ------------------------------------------------------------------------
    // Resolve current Runtime slave count.
    // ------------------------------------------------------------------------

    const auto* runtimeSlaves =
        (masterCore->m_pEni != nullptr)
        ? &masterCore->m_pEni->GetSlaves()
        : nullptr;

    uint32_t slaveCount = 0u;

    if (runtimeSlaves != nullptr)
    {
        slaveCount =
            static_cast<uint32_t>(
                runtimeSlaves->size());
    }

    if (slaveCount >
        SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES)
    {
        slaveCount =
            SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES;
    }


    // ------------------------------------------------------------------------
    // P50-local publisher observability.
    // ------------------------------------------------------------------------

    static SHM_ECAT_WatchdogDiagData*
        lastMapping =
        nullptr;

    static uint32_t
        lastSlaveCount =
        0u;

    static uint64_t
        shadowReadSuccess =
        0ULL;

    static uint64_t
        shadowReadMiss =
        0ULL;

    static bool
        publisherPrinted =
        false;


    const bool mappingChanged =
        lastMapping != watchdogDiag;

    const uint32_t previousSlaveCount =
        mappingChanged
        ? 0u
        : lastSlaveCount;


    SHM_ECAT_WatchdogDiagHeader& header =
        watchdogDiag->Header;


    // ------------------------------------------------------------------------
    // Shared Memory seqlock.
    //
    // odd  = Priority-50 writer active
    // even = stable coherent snapshot
    // ------------------------------------------------------------------------

    InterlockedIncrement(
        reinterpret_cast<volatile LONG*>(
            &header.Sequence));

    MemoryBarrier();


    // ------------------------------------------------------------------------
    // Refresh fixed ABI header.
    // ------------------------------------------------------------------------

    header.Magic =
        SHM_ECAT_WATCHDOG_DIAG_MAGIC;

    header.VersionMajor =
        SHM_ECAT_WATCHDOG_DIAG_VERSION_MAJOR;

    header.VersionMinor =
        SHM_ECAT_WATCHDOG_DIAG_VERSION_MINOR;

    header.StructSize =
        static_cast<uint32_t>(
            sizeof(
                SHM_ECAT_WatchdogDiagData));

    header.Heartbeat++;

    header.PublishCount++;

    header.SlaveCount =
        static_cast<uint16_t>(
            slaveCount);

    header.MaxSlaves =
        static_cast<uint16_t>(
            SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES);


    // ------------------------------------------------------------------------
    // Publish one fixed Watchdog slot per Runtime slave.
    // ------------------------------------------------------------------------

    for (
        uint32_t slave = 0u;
        slave < slaveCount;
        ++slave)
    {
        SHM_ECAT_WatchdogDiagEntry& target =
            watchdogDiag->Entries[slave];


        // Fixed slot identity is refreshed defensively in case the mapping
        // was recreated while the RT process remained alive.
        target.SlavePosition =
            static_cast<uint16_t>(
                slave);


        uint32_t valid = 0u;

        uint16_t watchdogDivider = 0u;

        uint16_t watchdogTimeProcessData = 0u;

        uint16_t watchdogStatusProcessData = 0u;

        uint8_t watchdogCounterProcessData = 0u;

        uint16_t enabledSmMask = 0u;

        int32_t lastTransportResult = 0;

        uint64_t lastUpdateTick = 0ULL;


        const bool readOk =
            OSCARMAX_ECAT_WatchdogDiagRt_Read(
                static_cast<uint16_t>(
                    slave),
                &valid,
                &watchdogDivider,
                &watchdogTimeProcessData,
                &watchdogStatusProcessData,
                &watchdogCounterProcessData,
                &enabledSmMask,
                &lastTransportResult,
                &lastUpdateTick);


        if (!readOk)
        {
            // Benign seqlock collision with Priority-64.
            //
            // Keep the previous coherent Shared Memory value. No retry,
            // spin, wait, Sleep or EtherCAT access is allowed here.
            shadowReadMiss++;

            continue;
        }


        shadowReadSuccess++;


        // --------------------------------------------------------------------
        // Runtime explicitly reports whether the latest Watchdog register
        // snapshot is valid.
        //
        // When Valid==0, the P64 shadow intentionally retains the last-known
        // raw register bytes together with the latest transport status.
        // Windows can therefore display STALE / INVALID while preserving
        // valuable post-event evidence.
        // --------------------------------------------------------------------

        target.Valid =
            valid != 0u
            ? 1u
            : 0u;

        target.WatchdogDivider =
            watchdogDivider;

        target.WatchdogTimeProcessData =
            watchdogTimeProcessData;

        target.WatchdogStatusProcessData =
            watchdogStatusProcessData;

        target.WatchdogCounterProcessData =
            watchdogCounterProcessData;

        target.EnabledSmMask =
            enabledSmMask;

        target.LastTransportResult =
            lastTransportResult;

        target.LastUpdateTick =
            lastUpdateTick;
    }


    // ------------------------------------------------------------------------
    // Defensive cleanup if a future Runtime topology is reloaded with fewer
    // slaves while the same Shared Memory mapping remains open.
    // ------------------------------------------------------------------------

    if (previousSlaveCount > slaveCount)
    {
        uint32_t clearEnd =
            previousSlaveCount;

        if (clearEnd >
            SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES)
        {
            clearEnd =
                SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES;
        }

        for (
            uint32_t slave = slaveCount;
            slave < clearEnd;
            ++slave)
        {
            SHM_ECAT_WatchdogDiagEntry& target =
                watchdogDiag->Entries[slave];

            std::memset(
                &target,
                0,
                sizeof(target));

            target.SlavePosition =
                static_cast<uint16_t>(
                    slave);
        }
    }


    // ------------------------------------------------------------------------
    // ABI-safe publisher observability.
    //
    // Reserved32[0] = successful P64 shadow reads, saturated to U32
    // Reserved32[1] = P64 shadow read collisions/misses, saturated to U32
    // Reserved32[2] = Stage 12E.7D marker
    // ------------------------------------------------------------------------

    header.Reserved32[0] =
        shadowReadSuccess > 0xFFFFFFFFULL
        ? 0xFFFFFFFFu
        : static_cast<uint32_t>(
            shadowReadSuccess);

    header.Reserved32[1] =
        shadowReadMiss > 0xFFFFFFFFULL
        ? 0xFFFFFFFFu
        : static_cast<uint32_t>(
            shadowReadMiss);

    header.Reserved32[2] =
        0x12E0070Du;


    MemoryBarrier();


    // even = coherent Shared Memory snapshot
    InterlockedIncrement(
        reinterpret_cast<volatile LONG*>(
            &header.Sequence));


    lastMapping =
        watchdogDiag;

    lastSlaveCount =
        slaveCount;


    if (!publisherPrinted)
    {
        publisherPrinted =
            true;

        DEBUG_PRINT(
            "[ECAT-WATCHDOG-DIAG-PUBLISHER] ACTIVE | "
            "Name:OSCARMAX_ECAT_WATCHDOG_DIAG | "
            "Cadence:10ms | "
            "Source:P64_RT_SHADOW | "
            "SHMWriter:P50 | "
            "ESC:0400/0420/0440/0442 | "
            "EtherCATTrafficAdded:NO | "
            "ControlWrite:NO\n");
    }
}
