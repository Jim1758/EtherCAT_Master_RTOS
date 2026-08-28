#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 12E.7B - OSCARMAX EtherCAT Process Data Watchdog Diagnostic Contract
// ============================================================================
//
// Shared Memory name:
//     OSCARMAX_ECAT_WATCHDOG_DIAG
//
// Purpose:
//     Publish the live ESC Process Data Watchdog diagnostic snapshot captured
//     by the Priority-64 owner-safe ESC diagnostic sweep to the Windows
//     EtherCAT ENI Tool.
//
// Data path:
//
//     ESC registers
//       -> Priority-64 EtherCatMaster_DC_Runtime.cpp RT shadow
//       -> OSCARMAX_ECAT_WatchdogDiagRt_Read()
//       -> Priority-50 System_EDM_SINKER_MODE.cpp publisher
//       -> OSCARMAX_ECAT_WATCHDOG_DIAG
//       -> Windows ENI Tool (read-only)
//
// Raw ESC sources:
//
//     0x0400..0x0401
//         Watchdog Divider
//
//     0x0420..0x0421
//         Watchdog Time Process Data
//
//     0x0440..0x0441
//         Watchdog Status Process Data
//
//     0x0442
//         Watchdog Counter Process Data
//
// EnabledSmMask:
//     Derived from the most recent live SyncManager Control bytes.
//
//     Bit N = SyncManager N has ControlByte bit 6
//             (Watchdog Trigger Enable) set.
//
// Design rules:
//
//   1. This is a NEW independent diagnostic Shared Memory channel.
//
//      It does NOT modify:
//          EDM_SINKER_MODE
//          OSCARMAX_ECAT_DIAG
//          OSCARMAX_ECAT_SM_DIAG
//          OSCARMAX_ECAT_FMMU_DIAG
//          OSCARMAX_ECAT_SERVICE
//
//   2. Windows is strictly read-only.
//
//   3. No EtherCAT register write path exists in this contract.
//
//   4. Priority-64 never writes Shared Memory directly.
//
//   5. Priority-50 is the single Shared Memory writer.
//
//   6. Fixed-width scalar fields only:
//          no bool
//          no pointer
//          no STL type
//
//   7. Pack=1 gives deterministic C++ / C# layout.
//
//   8. One global Sequence protects the whole Shared Memory snapshot:
//
//          writer:
//              Sequence++ -> odd
//              write fields
//              MemoryBarrier()
//              Sequence++ -> even
//
//          reader:
//              accept only identical even Sequence
//              before and after copy.
//
//   9. Entries are fixed by SlavePosition:
//
//          Entries[SlavePosition]
//
//      No variable-length parsing is required.
//
//  10. Valid means the latest owner-safe Watchdog register snapshot has
//      successfully completed at least once.
//
//      When a later transport attempt fails, Runtime may preserve the last
//      known-good register values while LastTransportResult / LastUpdateTick
//      continue to describe the latest diagnostic observation.
//
// Source fingerprint:
//
//     OSCARMAX_ECAT_WATCHDOG_DIAG_CONTRACT_12E7B_20260827
//
// ============================================================================


// ============================================================================
// Contract constants
// ============================================================================

// Little-endian memory bytes:
//     45 43 57 44
//     E  C  W  D
//
// "ECWD" = EtherCAT Watchdog Diagnostic
static constexpr uint32_t SHM_ECAT_WATCHDOG_DIAG_MAGIC =
0x44574345u;


static constexpr uint16_t
SHM_ECAT_WATCHDOG_DIAG_VERSION_MAJOR =
1u;


static constexpr uint16_t
SHM_ECAT_WATCHDOG_DIAG_VERSION_MINOR =
0u;


static constexpr uint32_t
SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES =
128u;


// ============================================================================
// 64-byte global header
// ============================================================================

struct SHM_ECAT_WatchdogDiagHeader
{
    uint32_t Magic;
    // SHM_ECAT_WATCHDOG_DIAG_MAGIC

    uint16_t VersionMajor;
    // ABI major

    uint16_t VersionMinor;
    // ABI minor

    uint32_t StructSize;
    // sizeof(SHM_ECAT_WatchdogDiagData)


    uint32_t Sequence;
    // seqlock:
    // even = stable
    // odd  = Priority-50 writer active


    uint32_t Heartbeat;
    // Incremented by Priority-50 every publish pass.


    uint64_t PublishCount;
    // Number of completed P50 publication passes.


    uint16_t SlaveCount;
    // Current Runtime slave count.


    uint16_t MaxSlaves;
    // Always SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES
    // for ABI v1.0.


    // ------------------------------------------------------------
    // Reserved global diagnostic observability.
    //
    // Stage 12E.7B initial publisher may use:
    //
    // [0] Shadow read success
    // [1] Shadow read miss
    // [2] Stage marker
    //
    // Remaining words stay reserved.
    // ------------------------------------------------------------
    uint32_t Reserved32[8];
};


// ============================================================================
// One fixed 32-byte Watchdog diagnostic slot per slave
// ============================================================================

struct SHM_ECAT_WatchdogDiagEntry
{
    // ------------------------------------------------------------
    // Identity / validity
    // ------------------------------------------------------------

    uint16_t SlavePosition;
    // Zero-based Runtime topology position.


    uint8_t Valid;
    // 1 = Watchdog register snapshot available.
    // 0 = no valid live snapshot yet.


    uint8_t Reserved8A;


    // ------------------------------------------------------------
    // ESC Watchdog registers
    // ------------------------------------------------------------

    uint16_t WatchdogDivider;
    // ESC 0x0400..0x0401


    uint16_t WatchdogTimeProcessData;
    // ESC 0x0420..0x0421


    uint16_t WatchdogStatusProcessData;
    // ESC 0x0440..0x0441


    uint8_t WatchdogCounterProcessData;
    // ESC 0x0442


    uint8_t Reserved8B;


    // ------------------------------------------------------------
    // SyncManager Watchdog Trigger source
    // ------------------------------------------------------------

    uint16_t EnabledSmMask;
    // Bit N = physical SyncManager N has
    //         ControlByte bit 6 enabled.


    uint16_t Reserved16;


    // ------------------------------------------------------------
    // Latest diagnostic transport observation
    // ------------------------------------------------------------

    int32_t LastTransportResult;
    // > 0 : successful EtherCAT WKC
    //   0 : WKC == 0
    // < 0 : bounded transport/deadline failure


    uint64_t LastUpdateTick;
    // Priority-64 PDO tick associated with latest Watchdog observation.


    uint32_t Reserved32;
};


// ============================================================================
// Complete Shared Memory mapping
// ============================================================================

struct SHM_ECAT_WatchdogDiagData
{
    SHM_ECAT_WatchdogDiagHeader Header;

    SHM_ECAT_WatchdogDiagEntry
        Entries[
            SHM_ECAT_WATCHDOG_DIAG_MAX_SLAVES
        ];
};


// ============================================================================
// ABI guards
// ============================================================================

#if !defined(__INTELLISENSE__)

static_assert(
    sizeof(SHM_ECAT_WatchdogDiagHeader) == 64,
    "SHM_ECAT_WatchdogDiagHeader ABI changed; "
    "update ENI Tool C# contract.");


static_assert(
    sizeof(SHM_ECAT_WatchdogDiagEntry) == 32,
    "SHM_ECAT_WatchdogDiagEntry ABI changed; "
    "update ENI Tool C# contract.");


static_assert(
    sizeof(SHM_ECAT_WatchdogDiagData) == 4160,
    "SHM_ECAT_WatchdogDiagData ABI changed; "
    "update ENI Tool C# contract.");

#endif


#pragma pack(pop)