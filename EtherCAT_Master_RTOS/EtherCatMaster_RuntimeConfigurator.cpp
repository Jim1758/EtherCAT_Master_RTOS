#include "EtherCatMaster.h"

#include <rtapi.h>
#include <rtssapi.h>

#include <string.h>

#include "GlobalConfig.h"


// ============================================================================
// Stage 5D - Complete Runtime Data-Driven Cutover
//
// This is the complete Stage 5D startup configuration path.
//
// Runtime XML now drives:
//
// INIT
// - SyncManager register values
// - A3E DC / Sync0 ESC configuration
//
// PRE-OP
// - Configurable PDO Assignment / Mapping
// - DC Sync Mode / Cycle (1C32 / 1C33)
// - Process Data Watchdog timeout
// - PRE_OP executable Runtime InitCommands
//
// SAFE-OP
// - SAFE_OP executable Runtime InitCommands
//
// The existing C++ per-ProductCode configuration methods remain untouched
// and are used automatically when the Stage 5C Equivalence Gate is CLOSED.
//
// NOT moved in Stage 5D:
// - FMMU configuration
// - DC propagation-delay AUTO measurement / 0x0928
// - PDO runtime / 250 us cyclic path
//
// All functions in this file run only during Startup.
// ============================================================================

namespace
{
    // Current verified ESC watchdog divider.
    //
    // 40 ns * (2498 + 2) = 100000 ns = 100 us / tick.
    //
    // Runtime XML owns the timeout in milliseconds.
    const uint16_t RUNTIME_WATCHDOG_DIVIDER =
        2498U;


    bool IsDcModeEnabled(
        const EtherCatSlave& slave)
    {
        return
            slave.runtimeDc.present &&
            strcmp(
                slave.runtimeDc.mode,
                "DC") ==
            0;
    }


    bool CheckSdoResult(
        int slaveIdx,
        const char* stage,
        uint16_t index,
        uint8_t subIndex,
        int wkc)
    {
        RtPrintf(
            "[RUNTIME-SDO] "
            "S%d | Stage:%s | "
            "0x%04X:%02X | WKC:%d | Result:%s\n",

            slaveIdx,
            stage,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            wkc,

            wkc >= 1
            ? "PASS"
            : "FAIL");


        return
            wkc >= 1;
    }
}


// ============================================================================
// ShouldUseRuntimeConfiguration
// ============================================================================

bool EtherCatMaster::ShouldUseRuntimeConfiguration(
    int slaveIdx) const
{
    if (!m_runtimeEquivalenceVerified)
    {
        return false;
    }


    if (m_pEni == nullptr)
    {
        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    if (slaveIdx < 0 ||
        slaveIdx >=
        (int)slaves.size())
    {
        return false;
    }


    // Full Stage5D cutover has already been proven against this exact
    // Runtime topology by AuditRuntimeConfigurationEquivalence().
    //
    // No ProductCode allow-list is needed here.
    return
        true;
}


// ============================================================================
// Stage5D.1 compatibility wrappers
// ============================================================================

bool EtherCatMaster::ShouldUseRuntimeSyncManagerConfiguration(
    int slaveIdx) const
{
    if (!ShouldUseRuntimeConfiguration(
        slaveIdx))
    {
        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    return
        !slaves[(size_t)slaveIdx]
        .runtimeSyncManagers
        .empty();
}


// ============================================================================
// ConfigureRuntimeSyncManagers
// ============================================================================

bool EtherCatMaster::ConfigureRuntimeSyncManagers(
    int slaveIdx)
{
    if (!ShouldUseRuntimeSyncManagerConfiguration(
        slaveIdx))
    {
        return
            true;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const EtherCatSlave& slave =
        slaves[(size_t)slaveIdx];


    RtPrintf(
        "[RUNTIME-SM-CUTOVER] "
        "S%d | Source:RUNTIME_XML | "
        "Profile:%s | MapMode:%s | SMCount:%u | BEGIN\n",

        slaveIdx,

        slave.runtimeProfile.name[0] != '\0'
        ? slave.runtimeProfile.name
        : "N/A",

        slave.runtimeProfile.pdoMappingMode[0] != '\0'
        ? slave.runtimeProfile.pdoMappingMode
        : "N/A",

        (unsigned int)
        slave.runtimeSyncManagers.size());


    LARGE_INTEGER wait;

    wait.QuadPart =
        5 *
        10000;


    int appliedCount =
        0;


    for (const auto& sm :
        slave.runtimeSyncManagers)
    {
        if (sm.index < 0 ||
            sm.index >
            15)
        {
            RtPrintf(
                "[RUNTIME-SM-CUTOVER] "
                "S%d | Invalid SM index:%d | Result:FAIL\n",

                slaveIdx,
                sm.index);


            return
                false;
        }


        uint8_t raw[8] =
        {
            0
        };


        raw[0] =
            (uint8_t)(
                sm.startAddress &
                0x00FFU);


        raw[1] =
            (uint8_t)(
                (
                    sm.startAddress >>
                    8
                    ) &
                0x00FFU);


        raw[2] =
            (uint8_t)(
                sm.length &
                0x00FFU);


        raw[3] =
            (uint8_t)(
                (
                    sm.length >>
                    8
                    ) &
                0x00FFU);


        raw[4] =
            sm.controlByte;


        raw[5] =
            0x00;


        raw[6] =
            sm.enabled
            ? 0x01
            : 0x00;


        raw[7] =
            0x00;


        const uint16_t registerAddress =
            (uint16_t)(
                0x0800 +
                (
                    sm.index *
                    8
                    ));


        const int wkc =
            ecx_APWR(
                m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                registerAddress,
                8,
                raw,
                20);


        RtPrintf(
            "[RUNTIME-SM-APPLY] "
            "S%d SM%d | "
            "Reg:0x%04X | "
            "Start:0x%04X | "
            "Len:%u | "
            "Ctrl:0x%02X | "
            "Enable:%d | "
            "WKC:%d | "
            "Result:%s\n",

            slaveIdx,
            sm.index,

            (unsigned int)
            registerAddress,

            (unsigned int)
            sm.startAddress,

            (unsigned int)
            sm.length,

            (unsigned int)
            sm.controlByte,

            sm.enabled
            ? 1
            : 0,

            wkc,

            wkc >= 1
            ? "PASS"
            : "FAIL");


        if (wkc < 1)
        {
            RtPrintf(
                "[RUNTIME-SM-CUTOVER-RESULT] "
                "S%d | Applied:%d/%u | Result:FAIL\n",

                slaveIdx,

                appliedCount,

                (unsigned int)
                slave.runtimeSyncManagers.size());


            return
                false;
        }


        appliedCount++;


        RtSleepFt(
            &wait);
    }


    RtPrintf(
        "[RUNTIME-SM-CUTOVER-RESULT] "
        "S%d | Applied:%d/%u | Result:PASS\n",

        slaveIdx,

        appliedCount,

        (unsigned int)
        slave.runtimeSyncManagers.size());


    return
        true;
}


// ============================================================================
// Runtime DC ESC configuration - INIT
// ============================================================================

static bool ConfigureRuntimeDcEscInit(
    EtherCatMaster* master,
    int slaveIdx,
    const EtherCatSlave& slave)
{
    if (!IsDcModeEnabled(
        slave))
    {
        return
            true;
    }


    if (!slave.runtimeDc.hasCycleTimeNs ||
        slave.runtimeDc.cycleTimeNs ==
        0U ||
        !slave.runtimeDc.hasShiftTimeNs ||
        slave.runtimeDc.shiftTimeNs <
        0)
    {
        RtPrintf(
            "[RUNTIME-DC-INIT] "
            "S%d | Invalid Runtime DC Cycle/Shift | Result:FAIL\n",

            slaveIdx);


        return
            false;
    }


    const uint32_t cycleTimeNs =
        slave.runtimeDc.cycleTimeNs;


    const uint64_t shiftTimeNs =
        (uint64_t)
        slave.runtimeDc.shiftTimeNs;


    if (shiftTimeNs >=
        (uint64_t)cycleTimeNs)
    {
        RtPrintf(
            "[RUNTIME-DC-INIT] "
            "S%d | Shift:%llu must be < Cycle:%u | Result:FAIL\n",

            slaveIdx,

            (unsigned long long)
            shiftTimeNs,

            (unsigned int)
            cycleTimeNs);


        return
            false;
    }


    const uint16_t configAddr =
        m_slaveInfo[slaveIdx]
        .configAddr;


    uint64_t zeroTime =
        0;


    uint8_t disableSync =
        0x00;


    uint8_t cyclicControl =
        0x00;


    uint16_t speedCounterStart =
        0x1000;


    uint8_t enableSync =
        0x03;


    uint64_t slaveTimeNs =
        0;


    RtPrintf(
        "[RUNTIME-DC-INIT] "
        "S%d | Mode:DC | Cycle:%u | Shift:%llu | Ref:%d | BEGIN\n",

        slaveIdx,

        (unsigned int)
        cycleTimeNs,

        (unsigned long long)
        shiftTimeNs,

        slave.runtimeDc.referenceClock
        ? 1
        : 0);


    // Preserve the exact verified A3E startup sequence.

    if (master->ecx_FPWR(
        configAddr,
        0x0920,
        &zeroTime,
        8,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x0910,
        &zeroTime,
        8,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x0981,
        &disableSync,
        1,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x0980,
        &cyclicControl,
        1,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPRD(
        configAddr,
        0x0910,
        &slaveTimeNs,
        8,
        20) <
        1)
    {
        return
            false;
    }


    const uint64_t masterTimeNs =
        master->GetCurrentMasterTimeNs();


    uint64_t timeOffset =
        masterTimeNs -
        slaveTimeNs;


    if (master->ecx_FPWR(
        configAddr,
        0x0920,
        &timeOffset,
        8,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x0930,
        &speedCounterStart,
        2,
        20) <
        1)
    {
        return
            false;
    }


    slaveTimeNs =
        0;


    if (master->ecx_FPRD(
        configAddr,
        0x0910,
        &slaveTimeNs,
        8,
        20) <
        1)
    {
        return
            false;
    }


    uint64_t startTime =
        slaveTimeNs +
        100000000ULL;


    startTime =
        (
            (
                startTime /
                (uint64_t)cycleTimeNs
                ) +
            1ULL
            ) *
        (uint64_t)cycleTimeNs;


    startTime +=
        shiftTimeNs;


    uint32_t cycleTimeWrite =
        cycleTimeNs;


    if (master->ecx_FPWR(
        configAddr,
        0x0990,
        &startTime,
        8,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x09A0,
        &cycleTimeWrite,
        4,
        20) <
        1)
    {
        return
            false;
    }


    if (master->ecx_FPWR(
        configAddr,
        0x0981,
        &enableSync,
        1,
        20) <
        1)
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-DC-INIT-RESULT] "
        "S%d | Start:%llu | Cycle:%u | Shift:%llu | Result:PASS\n",

        slaveIdx,

        (unsigned long long)
        startTime,

        (unsigned int)
        cycleTimeNs,

        (unsigned long long)
        shiftTimeNs);


    return
        true;
}


// ============================================================================
// Runtime InitCommand executor
// ============================================================================

static bool ExecuteRuntimeInitCommands(
    EtherCatMaster* master,
    int slaveIdx,
    const EtherCatSlave& slave,
    const char* transition)
{
    int applicable =
        0;


    int applied =
        0;


    LARGE_INTEGER wait;

    wait.QuadPart =
        300 *
        10000;


    for (const auto& command :
        slave.runtimeInitCommands)
    {
        if (!command.apply ||
            strcmp(
                command.transition,
                transition) !=
            0)
        {
            continue;
        }


        applicable++;


        if (command.data.empty())
        {
            RtPrintf(
                "[RUNTIME-INITCMD] "
                "S%d | Transition:%s | "
                "0x%04X:%02X | Data:EMPTY | Result:FAIL\n",

                slaveIdx,
                transition,

                (unsigned int)
                command.index,

                (unsigned int)
                command.subIndex);


            return
                false;
        }


        const int wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                command.index,
                command.subIndex,
                FALSE,
                (int)command.data.size(),
                const_cast<uint8_t*>(
                    command.data.data()),
                20);


        RtPrintf(
            "[RUNTIME-INITCMD] "
            "S%d | Transition:%s | "
            "Source:%s | Apply:1 | "
            "0x%04X:%02X | Bytes:%u | "
            "WKC:%d | Result:%s\n",

            slaveIdx,
            transition,

            command.source[0] != '\0'
            ? command.source
            : "N/A",

            (unsigned int)
            command.index,

            (unsigned int)
            command.subIndex,

            (unsigned int)
            command.data.size(),

            wkc,

            wkc >= 1
            ? "PASS"
            : "FAIL");


        if (wkc < 1)
        {
            return
                false;
        }


        applied++;


        RtSleepFt(
            &wait);
    }


    RtPrintf(
        "[RUNTIME-INITCMD-RESULT] "
        "S%d | Transition:%s | "
        "Applied:%d/%d | Result:PASS\n",

        slaveIdx,
        transition,
        applied,
        applicable);


    return
        true;
}


// ============================================================================
// Runtime PDO Mapping - PRE-OP
// ============================================================================

static bool ConfigureRuntimePdoMapping(
    EtherCatMaster* master,
    int slaveIdx,
    const EtherCatSlave& slave)
{
    if (!slave.runtimeProfile.hasPdoMappingMode ||
        strcmp(
            slave.runtimeProfile.pdoMappingMode,
            "Configurable") !=
        0)
    {
        RtPrintf(
            "[RUNTIME-PDO-CUTOVER] "
            "S%d | MapMode:%s | Action:FIXED_NO_REMAP | Result:PASS\n",

            slaveIdx,

            slave.runtimeProfile.pdoMappingMode[0] != '\0'
            ? slave.runtimeProfile.pdoMappingMode
            : "N/A");


        return
            true;
    }


    if (!slave.runtimeProfile.hasRxPdoAssignIndex ||
        !slave.runtimeProfile.hasTxPdoAssignIndex)
    {
        RtPrintf(
            "[RUNTIME-PDO-CUTOVER] "
            "S%d | Assignment Index missing | Result:FAIL\n",

            slaveIdx);


        return
            false;
    }


    if (slave.runtimeRxPdos.empty() ||
        slave.runtimeTxPdos.empty())
    {
        RtPrintf(
            "[RUNTIME-PDO-CUTOVER] "
            "S%d | Configurable profile has empty PDO list | Result:FAIL\n",

            slaveIdx);


        return
            false;
    }


    RtPrintf(
        "[RUNTIME-PDO-CUTOVER] "
        "S%d | Source:RUNTIME_XML | "
        "RxAssign:0x%04X RxPDO:%u | "
        "TxAssign:0x%04X TxPDO:%u | BEGIN\n",

        slaveIdx,

        (unsigned int)
        slave.runtimeProfile.rxPdoAssignIndex,

        (unsigned int)
        slave.runtimeRxPdos.size(),

        (unsigned int)
        slave.runtimeProfile.txPdoAssignIndex,

        (unsigned int)
        slave.runtimeTxPdos.size());


    uint8_t zero8 =
        0;


    // Disable assignments first.
    int wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            slave.runtimeProfile.rxPdoAssignIndex,
            0x00,
            FALSE,
            1,
            &zero8,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "PDO_DISABLE_RX_ASSIGN",
        slave.runtimeProfile.rxPdoAssignIndex,
        0x00,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            slave.runtimeProfile.txPdoAssignIndex,
            0x00,
            FALSE,
            1,
            &zero8,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "PDO_DISABLE_TX_ASSIGN",
        slave.runtimeProfile.txPdoAssignIndex,
        0x00,
        wkc))
    {
        return
            false;
    }


    // ------------------------------------------------------------------------
    // RxPDO mapping objects
    // ------------------------------------------------------------------------

    for (const auto& pdo :
        slave.runtimeRxPdos)
    {
        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                pdo.index,
                0x00,
                FALSE,
                1,
                &zero8,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "RX_PDO_DISABLE",
            pdo.index,
            0x00,
            wkc))
        {
            return
                false;
        }


        if (pdo.entries.size() >
            255U)
        {
            return
                false;
        }


        for (size_t entryIndex = 0;
            entryIndex <
            pdo.entries.size();
            entryIndex++)
        {
            uint32_t mapping =
                pdo.entries[entryIndex]
                .mappingValue;


            const uint8_t subIndex =
                (uint8_t)(
                    entryIndex +
                    1U);


            wkc =
                master->ecx_SDOwrite(
                    slaveIdx,
                    pdo.index,
                    subIndex,
                    FALSE,
                    4,
                    &mapping,
                    20);


            if (!CheckSdoResult(
                slaveIdx,
                "RX_PDO_ENTRY",
                pdo.index,
                subIndex,
                wkc))
            {
                return
                    false;
            }
        }


        uint8_t count =
            (uint8_t)
            pdo.entries.size();


        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                pdo.index,
                0x00,
                FALSE,
                1,
                &count,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "RX_PDO_ENABLE",
            pdo.index,
            0x00,
            wkc))
        {
            return
                false;
        }
    }


    // ------------------------------------------------------------------------
    // TxPDO mapping objects
    // ------------------------------------------------------------------------

    for (const auto& pdo :
        slave.runtimeTxPdos)
    {
        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                pdo.index,
                0x00,
                FALSE,
                1,
                &zero8,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "TX_PDO_DISABLE",
            pdo.index,
            0x00,
            wkc))
        {
            return
                false;
        }


        if (pdo.entries.size() >
            255U)
        {
            return
                false;
        }


        for (size_t entryIndex = 0;
            entryIndex <
            pdo.entries.size();
            entryIndex++)
        {
            uint32_t mapping =
                pdo.entries[entryIndex]
                .mappingValue;


            const uint8_t subIndex =
                (uint8_t)(
                    entryIndex +
                    1U);


            wkc =
                master->ecx_SDOwrite(
                    slaveIdx,
                    pdo.index,
                    subIndex,
                    FALSE,
                    4,
                    &mapping,
                    20);


            if (!CheckSdoResult(
                slaveIdx,
                "TX_PDO_ENTRY",
                pdo.index,
                subIndex,
                wkc))
            {
                return
                    false;
            }
        }


        uint8_t count =
            (uint8_t)
            pdo.entries.size();


        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                pdo.index,
                0x00,
                FALSE,
                1,
                &count,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "TX_PDO_ENABLE",
            pdo.index,
            0x00,
            wkc))
        {
            return
                false;
        }
    }


    // ------------------------------------------------------------------------
    // Rx Assignment
    // ------------------------------------------------------------------------

    if (slave.runtimeRxPdos.size() >
        255U ||
        slave.runtimeTxPdos.size() >
        255U)
    {
        return
            false;
    }


    for (size_t pdoIndex = 0;
        pdoIndex <
        slave.runtimeRxPdos.size();
        pdoIndex++)
    {
        uint16_t assignedPdo =
            slave.runtimeRxPdos[pdoIndex]
            .index;


        const uint8_t subIndex =
            (uint8_t)(
                pdoIndex +
                1U);


        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                slave.runtimeProfile.rxPdoAssignIndex,
                subIndex,
                FALSE,
                2,
                &assignedPdo,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "RX_ASSIGN_ENTRY",
            slave.runtimeProfile.rxPdoAssignIndex,
            subIndex,
            wkc))
        {
            return
                false;
        }
    }


    uint8_t rxCount =
        (uint8_t)
        slave.runtimeRxPdos.size();


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            slave.runtimeProfile.rxPdoAssignIndex,
            0x00,
            FALSE,
            1,
            &rxCount,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "RX_ASSIGN_ENABLE",
        slave.runtimeProfile.rxPdoAssignIndex,
        0x00,
        wkc))
    {
        return
            false;
    }


    // ------------------------------------------------------------------------
    // Tx Assignment
    // ------------------------------------------------------------------------

    for (size_t pdoIndex = 0;
        pdoIndex <
        slave.runtimeTxPdos.size();
        pdoIndex++)
    {
        uint16_t assignedPdo =
            slave.runtimeTxPdos[pdoIndex]
            .index;


        const uint8_t subIndex =
            (uint8_t)(
                pdoIndex +
                1U);


        wkc =
            master->ecx_SDOwrite(
                slaveIdx,
                slave.runtimeProfile.txPdoAssignIndex,
                subIndex,
                FALSE,
                2,
                &assignedPdo,
                20);


        if (!CheckSdoResult(
            slaveIdx,
            "TX_ASSIGN_ENTRY",
            slave.runtimeProfile.txPdoAssignIndex,
            subIndex,
            wkc))
        {
            return
                false;
        }
    }


    uint8_t txCount =
        (uint8_t)
        slave.runtimeTxPdos.size();


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            slave.runtimeProfile.txPdoAssignIndex,
            0x00,
            FALSE,
            1,
            &txCount,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "TX_ASSIGN_ENABLE",
        slave.runtimeProfile.txPdoAssignIndex,
        0x00,
        wkc))
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-PDO-CUTOVER-RESULT] "
        "S%d | RxPDO:%u | TxPDO:%u | Result:PASS\n",

        slaveIdx,

        (unsigned int)
        slave.runtimeRxPdos.size(),

        (unsigned int)
        slave.runtimeTxPdos.size());


    return
        true;
}


// ============================================================================
// Runtime DC CoE Sync configuration - PRE-OP
// ============================================================================

static bool ConfigureRuntimeDcCoePreOp(
    EtherCatMaster* master,
    int slaveIdx,
    const EtherCatSlave& slave)
{
    if (!IsDcModeEnabled(
        slave))
    {
        return
            true;
    }


    if (!slave.runtimeDc.hasCycleTimeNs ||
        slave.runtimeDc.cycleTimeNs ==
        0U)
    {
        return
            false;
    }


    uint16_t freeRunMode =
        0;


    uint16_t sync0Mode =
        2;


    uint32_t cycleTimeNs =
        slave.runtimeDc.cycleTimeNs;


    int wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C32,
            0x01,
            FALSE,
            2,
            &freeRunMode,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_FREE_1C32",
        0x1C32,
        0x01,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C33,
            0x01,
            FALSE,
            2,
            &freeRunMode,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_FREE_1C33",
        0x1C33,
        0x01,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C32,
            0x01,
            FALSE,
            2,
            &sync0Mode,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_SYNC_1C32",
        0x1C32,
        0x01,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C32,
            0x0A,
            FALSE,
            4,
            &cycleTimeNs,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_CYCLE_1C32",
        0x1C32,
        0x0A,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C33,
            0x01,
            FALSE,
            2,
            &sync0Mode,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_SYNC_1C33",
        0x1C33,
        0x01,
        wkc))
    {
        return
            false;
    }


    wkc =
        master->ecx_SDOwrite(
            slaveIdx,
            0x1C33,
            0x0A,
            FALSE,
            4,
            &cycleTimeNs,
            20);


    if (!CheckSdoResult(
        slaveIdx,
        "DC_CYCLE_1C33",
        0x1C33,
        0x0A,
        wkc))
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-DC-PREOP-RESULT] "
        "S%d | Mode:DC | Cycle:%u | Result:PASS\n",

        slaveIdx,

        (unsigned int)
        cycleTimeNs);


    return
        true;
}


// ============================================================================
// Runtime Watchdog - PRE-OP
// ============================================================================

static bool ConfigureRuntimeWatchdog(
    EtherCatMaster* master,
    int slaveIdx,
    const EtherCatSlave& slave)
{
    if (!slave.runtimeWatchdog.present ||
        !slave.runtimeWatchdog.hasProcessDataTimeoutMs ||
        slave.runtimeWatchdog.processDataTimeoutMs ==
        0U)
    {
        RtPrintf(
            "[RUNTIME-WATCHDOG] "
            "S%d | Runtime timeout missing | Result:FAIL\n",

            slaveIdx);


        return
            false;
    }


    const uint64_t tickNs =
        40ULL *
        (
            (uint64_t)RUNTIME_WATCHDOG_DIVIDER +
            2ULL
            );


    const uint64_t timeoutNs =
        (uint64_t)
        slave.runtimeWatchdog.processDataTimeoutMs *
        1000000ULL;


    const uint64_t ticks64 =
        timeoutNs /
        tickNs;


    if (ticks64 == 0ULL ||
        ticks64 >
        0xFFFFULL ||
        (
            ticks64 *
            tickNs
            ) !=
        timeoutNs)
    {
        RtPrintf(
            "[RUNTIME-WATCHDOG] "
            "S%d | Timeout:%u ms cannot be represented exactly | "
            "TickNs:%llu | Result:FAIL\n",

            slaveIdx,

            (unsigned int)
            slave.runtimeWatchdog.processDataTimeoutMs,

            (unsigned long long)
            tickNs);


        return
            false;
    }


    uint16_t divider =
        RUNTIME_WATCHDOG_DIVIDER;


    uint16_t processDataTicks =
        (uint16_t)
        ticks64;


    int wkc =
        master->ecx_APWR(
            m_slaveInfo[slaveIdx].APRDAPWR_Addr,
            0x0400,
            2,
            &divider,
            20);


    if (wkc < 1)
    {
        return
            false;
    }


    wkc =
        master->ecx_APWR(
            m_slaveInfo[slaveIdx].APRDAPWR_Addr,
            0x0420,
            2,
            &processDataTicks,
            20);


    if (wkc < 1)
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-WATCHDOG-RESULT] "
        "S%d | Timeout:%u ms | Divider:%u | "
        "Ticks:%u | TickNs:%llu | Result:PASS\n",

        slaveIdx,

        (unsigned int)
        slave.runtimeWatchdog.processDataTimeoutMs,

        (unsigned int)
        divider,

        (unsigned int)
        processDataTicks,

        (unsigned long long)
        tickNs);


    return
        true;
}


// ============================================================================
// ConfigureRuntimeInitStage
// ============================================================================

bool EtherCatMaster::ConfigureRuntimeInitStage(
    int slaveIdx,
    uint64_t unifiedStartTime)
{
    (void)
        unifiedStartTime;


    if (!ShouldUseRuntimeConfiguration(
        slaveIdx))
    {
        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const EtherCatSlave& slave =
        slaves[(size_t)slaveIdx];


    RtPrintf(
        "[RUNTIME-CONFIG-INIT] "
        "S%d | BEGIN\n",

        slaveIdx);


    if (!ConfigureRuntimeSyncManagers(
        slaveIdx))
    {
        return
            false;
    }


    if (!ConfigureRuntimeDcEscInit(
        this,
        slaveIdx,
        slave))
    {
        return
            false;
    }


    if (!ExecuteRuntimeInitCommands(
        this,
        slaveIdx,
        slave,
        "INIT"))
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-CONFIG-INIT-RESULT] "
        "S%d | Result:PASS\n",

        slaveIdx);


    return
        true;
}


// ============================================================================
// ConfigureRuntimePreOpStage
// ============================================================================

bool EtherCatMaster::ConfigureRuntimePreOpStage(
    int slaveIdx)
{
    if (!ShouldUseRuntimeConfiguration(
        slaveIdx))
    {
        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const EtherCatSlave& slave =
        slaves[(size_t)slaveIdx];


    RtPrintf(
        "[RUNTIME-CONFIG-PREOP] "
        "S%d | BEGIN\n",

        slaveIdx);


    if (!ConfigureRuntimePdoMapping(
        this,
        slaveIdx,
        slave))
    {
        return
            false;
    }


    if (!ConfigureRuntimeDcCoePreOp(
        this,
        slaveIdx,
        slave))
    {
        return
            false;
    }


    if (!ConfigureRuntimeWatchdog(
        this,
        slaveIdx,
        slave))
    {
        return
            false;
    }


    if (!ExecuteRuntimeInitCommands(
        this,
        slaveIdx,
        slave,
        "PRE_OP"))
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-CONFIG-PREOP-RESULT] "
        "S%d | Result:PASS\n",

        slaveIdx);


    return
        true;
}


// ============================================================================
// ConfigureRuntimeSafeOpStage
// ============================================================================

bool EtherCatMaster::ConfigureRuntimeSafeOpStage(
    int slaveIdx)
{
    if (!ShouldUseRuntimeConfiguration(
        slaveIdx))
    {
        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const EtherCatSlave& slave =
        slaves[(size_t)slaveIdx];


    RtPrintf(
        "[RUNTIME-CONFIG-SAFEOP] "
        "S%d | BEGIN\n",

        slaveIdx);


    if (!ExecuteRuntimeInitCommands(
        this,
        slaveIdx,
        slave,
        "SAFE_OP"))
    {
        return
            false;
    }


    RtPrintf(
        "[RUNTIME-CONFIG-SAFEOP-RESULT] "
        "S%d | Result:PASS\n",

        slaveIdx);


    return
        true;
}
