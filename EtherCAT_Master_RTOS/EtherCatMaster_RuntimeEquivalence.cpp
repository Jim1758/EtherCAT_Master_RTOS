#include "EtherCatMaster.h"

#include <rtapi.h>
#include <stdio.h>
#include <string.h>

#include "GlobalConfig.h"


// ============================================================================
// Stage 5C - Runtime Configuration Equivalence Audit
//
// Runtime XML is now able to carry:
//
// - SyncManagers
// - RxPDO / TxPDO + Entry
// - InitCommands
// - DC
// - Watchdog
//
// The actual EtherCAT hardware writes are still performed by the current:
//
// ConfigureSlaveGeneric_INIT()
// ConfigureSlaveGeneric_PRE_OP()
// ConfigureSlaveGeneric_SAFE_OP()
//
// Stage 5C only compares both sides.
//
// It does NOT:
//
// - call APWR / FPWR / SDOwrite
// - change AL State
// - change DC
// - change PDO
// - change Watchdog
//
// If differences are found, Startup continues with the current verified
// hard-code configuration. Stage 5D will not switch the hardware source
// until the differences have been resolved.
// ============================================================================

namespace
{
    const uint16_t CURRENT_WATCHDOG_DIVIDER =
        2498U;


    const uint16_t CURRENT_WATCHDOG_PD_TICKS =
        20000U;


    const uint32_t CURRENT_A3E_DC_CYCLE_NS =
        250000U;


    const int64_t CURRENT_A3E_SYNC0_SHIFT_NS =
        125000LL;


    uint32_t CurrentHardcodeWatchdogTimeoutMs()
    {
        // EtherCAT ESC watchdog tick:
        // 40 ns * (Divider + 2)
        //
        // Divider 2498 -> 100000 ns = 100 us
        // 20000 ticks   -> 2000 ms
        const uint64_t tickNs =
            40ULL *
            (
                (uint64_t)CURRENT_WATCHDOG_DIVIDER +
                2ULL
                );


        const uint64_t timeoutNs =
            tickNs *
            (uint64_t)CURRENT_WATCHDOG_PD_TICKS;


        return
            (uint32_t)(
                timeoutNs /
                1000000ULL);
    }


    const EtherCatRuntimeSyncManagerConfig*
        FindRuntimeSm(
            const EtherCatSlave& slave,
            int smIndex)
    {
        for (const auto& sm :
            slave.runtimeSyncManagers)
        {
            if (sm.index ==
                smIndex)
            {
                return
                    &sm;
            }
        }


        return
            nullptr;
    }


    const EtherCatRuntimePdoConfig*
        FindRuntimePdo(
            const std::vector<EtherCatRuntimePdoConfig>& pdos,
            uint16_t pdoIndex)
    {
        for (const auto& pdo :
            pdos)
        {
            if (pdo.index ==
                pdoIndex)
            {
                return
                    &pdo;
            }
        }


        return
            nullptr;
    }


    bool CompareSm(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        int smIndex,
        uint16_t expectedStart,
        uint16_t expectedLength,
        uint8_t expectedControl,
        bool expectedEnable,
        int& coreErrors)
    {
        const EtherCatRuntimeSyncManagerConfig* sm =
            FindRuntimeSm(
                runtimeSlave,
                smIndex);


        if (sm == nullptr)
        {
            coreErrors++;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-ERROR] "
                "S%d SM%d | Runtime:NOT_FOUND | "
                "CPP Start:0x%04X Len:%u Ctrl:0x%02X Enable:%d\n",

                slaveIndex,
                smIndex,

                (unsigned int)
                expectedStart,

                (unsigned int)
                expectedLength,

                (unsigned int)
                expectedControl,

                expectedEnable
                ? 1
                : 0);


            return
                false;
        }


        const bool startMatch =
            sm->startAddress ==
            expectedStart;


        const bool lengthMatch =
            sm->length ==
            expectedLength;


        const bool controlMatch =
            sm->controlByte ==
            expectedControl;


        const bool enableMatch =
            sm->enabled ==
            expectedEnable;


        const bool pass =
            startMatch &&
            lengthMatch &&
            controlMatch &&
            enableMatch;


        if (!pass)
        {
            coreErrors++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-SM] "
            "S%d SM%d | "
            "Start XML:0x%04X CPP:0x%04X %s | "
            "Len XML:%u CPP:%u %s | "
            "Ctrl XML:0x%02X CPP:0x%02X %s | "
            "Enable XML:%d CPP:%d %s | "
            "Result:%s\n",

            slaveIndex,
            smIndex,

            (unsigned int)
            sm->startAddress,

            (unsigned int)
            expectedStart,

            startMatch
            ? "MATCH"
            : "DIFF",

            (unsigned int)
            sm->length,

            (unsigned int)
            expectedLength,

            lengthMatch
            ? "MATCH"
            : "DIFF",

            (unsigned int)
            sm->controlByte,

            (unsigned int)
            expectedControl,

            controlMatch
            ? "MATCH"
            : "DIFF",

            sm->enabled
            ? 1
            : 0,

            expectedEnable
            ? 1
            : 0,

            enableMatch
            ? "MATCH"
            : "DIFF",

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    }


    bool ComparePdoEntries(
        int slaveIndex,
        const char* direction,
        const EtherCatRuntimePdoConfig* runtimePdo,
        uint16_t expectedPdoIndex,
        const uint32_t* expectedMappings,
        int expectedEntryCount,
        int& coreErrors)
    {
        if (runtimePdo == nullptr)
        {
            coreErrors++;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-ERROR] "
                "S%d %sPDO 0x%04X | Runtime:NOT_FOUND\n",

                slaveIndex,
                direction,
                (unsigned int)
                expectedPdoIndex);


            return
                false;
        }


        bool pass =
            true;


        if ((int)runtimePdo->entries.size() !=
            expectedEntryCount)
        {
            pass =
                false;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-ERROR] "
                "S%d %sPDO 0x%04X | "
                "EntryCount XML:%u CPP:%d | DIFF\n",

                slaveIndex,
                direction,
                (unsigned int)
                expectedPdoIndex,

                (unsigned int)
                runtimePdo->entries.size(),

                expectedEntryCount);
        }


        const int compareCount =
            (int)runtimePdo->entries.size() <
            expectedEntryCount
            ?
            (int)runtimePdo->entries.size()
            :
            expectedEntryCount;


        for (int i = 0;
            i < compareCount;
            i++)
        {
            const uint32_t actual =
                runtimePdo->entries[(size_t)i].mappingValue;


            const uint32_t expected =
                expectedMappings[i];


            const bool entryMatch =
                actual ==
                expected;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-PDO] "
                "S%d %sPDO 0x%04X Entry:%d | "
                "XML:0x%08X CPP:0x%08X | %s\n",

                slaveIndex,
                direction,
                (unsigned int)
                expectedPdoIndex,
                i + 1,

                (unsigned int)
                actual,

                (unsigned int)
                expected,

                entryMatch
                ? "MATCH"
                : "DIFF");


            if (!entryMatch)
            {
                pass =
                    false;
            }
        }


        if (!pass)
        {
            coreErrors++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-PDO-RESULT] "
            "S%d %sPDO 0x%04X | Result:%s\n",

            slaveIndex,
            direction,
            (unsigned int)
            expectedPdoIndex,

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    }


    void CompareWatchdogPolicy(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        int& policyDifferences)
    {
        const uint32_t hardcodeMs =
            CurrentHardcodeWatchdogTimeoutMs();


        if (!runtimeSlave.runtimeWatchdog.present ||
            !runtimeSlave.runtimeWatchdog.hasProcessDataTimeoutMs)
        {
            policyDifferences++;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-POLICY] "
                "S%d Watchdog | Runtime:NOT_SPECIFIED | "
                "CPP Divider:%u Ticks:%u ~= %u ms | Result:DIFF\n",

                slaveIndex,

                (unsigned int)
                CURRENT_WATCHDOG_DIVIDER,

                (unsigned int)
                CURRENT_WATCHDOG_PD_TICKS,

                (unsigned int)
                hardcodeMs);


            return;
        }


        const uint32_t runtimeMs =
            runtimeSlave.runtimeWatchdog.processDataTimeoutMs;


        const bool match =
            runtimeMs ==
            hardcodeMs;


        if (!match)
        {
            policyDifferences++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-POLICY] "
            "S%d Watchdog | "
            "XML:%u ms | "
            "CPP Divider:%u Ticks:%u ~= %u ms | "
            "Result:%s\n",

            slaveIndex,

            (unsigned int)
            runtimeMs,

            (unsigned int)
            CURRENT_WATCHDOG_DIVIDER,

            (unsigned int)
            CURRENT_WATCHDOG_PD_TICKS,

            (unsigned int)
            hardcodeMs,

            match
            ? "MATCH"
            : "DIFF");
    }


    void CompareA3EDcPolicy(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        bool expectedReference,
        int& policyDifferences)
    {
        const bool modeMatch =
            runtimeSlave.runtimeDc.present &&
            strcmp(
                runtimeSlave.runtimeDc.mode,
                "DC") ==
            0;


        const bool cycleMatch =
            runtimeSlave.runtimeDc.present &&
            runtimeSlave.runtimeDc.hasCycleTimeNs &&
            runtimeSlave.runtimeDc.cycleTimeNs ==
            CURRENT_A3E_DC_CYCLE_NS;


        const bool shiftMatch =
            runtimeSlave.runtimeDc.present &&
            runtimeSlave.runtimeDc.hasShiftTimeNs &&
            runtimeSlave.runtimeDc.shiftTimeNs ==
            CURRENT_A3E_SYNC0_SHIFT_NS;


        const bool referenceMatch =
            runtimeSlave.runtimeDc.present &&
            runtimeSlave.runtimeDc.hasReferenceClock &&
            runtimeSlave.runtimeDc.referenceClock ==
            expectedReference;


        if (!modeMatch)
        {
            policyDifferences++;
        }


        if (!cycleMatch)
        {
            policyDifferences++;
        }


        if (!shiftMatch)
        {
            policyDifferences++;
        }


        if (!referenceMatch)
        {
            policyDifferences++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-DC] "
            "S%d | "
            "Mode XML:%s CPP:DC %s | "
            "Cycle XML:%u CPP:%u %s | "
            "Shift XML:%lld CPP:%lld %s | "
            "Ref XML:%d CPP:%d %s\n",

            slaveIndex,

            runtimeSlave.runtimeDc.mode[0] != '\0'
            ? runtimeSlave.runtimeDc.mode
            : "N/A",

            modeMatch
            ? "MATCH"
            : "DIFF",

            (unsigned int)
            runtimeSlave.runtimeDc.cycleTimeNs,

            (unsigned int)
            CURRENT_A3E_DC_CYCLE_NS,

            cycleMatch
            ? "MATCH"
            : "DIFF",

            (long long)
            runtimeSlave.runtimeDc.shiftTimeNs,

            (long long)
            CURRENT_A3E_SYNC0_SHIFT_NS,

            shiftMatch
            ? "MATCH"
            : "DIFF",

            runtimeSlave.runtimeDc.referenceClock
            ? 1
            : 0,

            expectedReference
            ? 1
            : 0,

            referenceMatch
            ? "MATCH"
            : "DIFF");
    }
    bool ComparePdoMappingMode(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        const char* expectedMode,
        int& policyDifferences)
    {
        const bool match =
            runtimeSlave.runtimeProfile.hasPdoMappingMode &&
            strcmp(
                runtimeSlave.runtimeProfile.pdoMappingMode,
                expectedMode) ==
            0;


        if (!match)
        {
            policyDifferences++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-PDO-MODE] "
            "S%d | XML:%s | CPP:%s | Result:%s\n",

            slaveIndex,

            runtimeSlave.runtimeProfile.pdoMappingMode[0] != '\0'
            ? runtimeSlave.runtimeProfile.pdoMappingMode
            : "N/A",

            expectedMode,

            match
            ? "MATCH"
            : "DIFF");


        return
            match;
    }


    void CompareA3EInformationalInitCommands(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        int& policyDifferences)
    {
        bool pass =
            true;


        for (const auto& command :
            runtimeSlave.runtimeInitCommands)
        {
            if (strcmp(
                command.source,
                "ESI") ==
                0 &&
                command.apply)
            {
                pass =
                    false;
            }
        }


        if (!pass)
        {
            policyDifferences++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-INITCMD] "
            "S%d A3E | "
            "ESI commands must remain Apply:0 | "
            "Result:%s\n",

            slaveIndex,

            pass
            ? "MATCH"
            : "DIFF");
    }


    void Compare8124SafeOpInitCommands(
        int slaveIndex,
        const EtherCatSlave& runtimeSlave,
        int& coverageWarnings)
    {
        bool found[4] =
        {
            false,
            false,
            false,
            false
        };


        for (const auto& command :
            runtimeSlave.runtimeInitCommands)
        {
            if (!command.apply ||
                strcmp(
                    command.source,
                    "OSCARMAX_RUNTIME") !=
                0 ||
                strcmp(
                    command.transition,
                    "SAFE_OP") !=
                0 ||
                command.index !=
                0x2002U ||
                command.subIndex <
                1U ||
                command.subIndex >
                4U ||
                command.data.size() !=
                2U ||
                command.data[0] !=
                0x01U ||
                command.data[1] !=
                0x00U)
            {
                continue;
            }


            found[
                command.subIndex -
                    1U] =
                true;
        }


        bool pass =
            true;


        for (int i = 0;
            i < 4;
            i++)
        {
            if (!found[i])
            {
                pass =
                    false;
            }
        }


        if (!pass)
        {
            coverageWarnings++;
        }


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-INITCMD] "
            "S%d R1-EC8124D0 | "
            "SAFE_OP 0x2002:01..04 UINT16(1) | "
            "Result:%s\n",

            slaveIndex,

            pass
            ? "MATCH"
            : "NOT_REPRESENTED");
    }
}


// ============================================================================
// AuditRuntimeConfigurationEquivalence
// ============================================================================

bool EtherCatMaster::AuditRuntimeConfigurationEquivalence()
{
    // Fail-safe default:
    // Runtime XML hardware cutover is closed until this complete audit passes.
    m_runtimeEquivalenceVerified =
        false;


    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[RUNTIME-EQUIVALENCE] FAILED | "
            "Reason:Runtime Config is not attached | "
            "HardwareWrite:NO\n");


        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int comparedSlaves =
        0;


    int coreErrors =
        0;


    int policyDifferences =
        0;


    int coverageWarnings =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-EQUIVALENCE] BEGIN | "
        "Stage:5C | Mode:AUDIT_ONLY | HardwareWrite:NO\n"
        "============================================================\n");


    const uint32_t a3eRxMappings[4] =
    {
        0x60400010U,
        0x60FF0020U,
        0x60B80010U,
        0x60600008U
    };


    const uint32_t a3eTxMappings[8] =
    {
        0x60410010U,
        0x60640020U,
        0x606C0020U,
        0x60770010U,
        0x60B90010U,
        0x60BA0020U,
        0x60610008U,
        0x25100020U
    };


    int firstServoIndex =
        -1;


    for (int i = 0;
        i < (int)slaves.size();
        i++)
    {
        if (strcmp(
            slaves[(size_t)i].type,
            "Servo") ==
            0)
        {
            firstServoIndex =
                i;

            break;
        }
    }


    for (int i = 0;
        i < (int)slaves.size();
        i++)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)i];


        comparedSlaves++;


        const int slaveCoreErrorsBefore =
            coreErrors;


        const int slavePolicyBefore =
            policyDifferences;


        const int slaveWarningsBefore =
            coverageWarnings;


        CompareWatchdogPolicy(
            i,
            slave,
            policyDifferences);


        if (slave.vendorId ==
            0x000001DDU)
        {
            switch (slave.productCode)
            {
            case 0x00005500U:
                if (!slave.runtimeSyncManagers.empty())
                {
                    coreErrors++;


                    RtPrintf(
                        "[RUNTIME-EQUIVALENCE-ERROR] "
                        "S%d R1-EC5500 | "
                        "XML SM:%u | CPP SM:0 | Result:FAIL\n",

                        i,

                        (unsigned int)
                        slave.runtimeSyncManagers.size());
                }


                ComparePdoMappingMode(
                    i,
                    slave,
                    "Fixed",
                    policyDifferences);

                break;


            case 0x00006002U:
                CompareSm(
                    i,
                    slave,
                    0,
                    0x1000,
                    2,
                    0x00,
                    true,
                    coreErrors);


                ComparePdoMappingMode(
                    i,
                    slave,
                    "Fixed",
                    policyDifferences);

                break;


            case 0x00007062U:
                CompareSm(
                    i,
                    slave,
                    0,
                    0x0F00,
                    1,
                    0x44,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    1,
                    0x0F01,
                    1,
                    0x44,
                    true,
                    coreErrors);


                ComparePdoMappingMode(
                    i,
                    slave,
                    "Fixed",
                    policyDifferences);

                break;


            case 0x00008124U:
                CompareSm(
                    i,
                    slave,
                    0,
                    0x1000,
                    128,
                    0x26,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    1,
                    0x1080,
                    128,
                    0x22,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    2,
                    0x1100,
                    0,
                    0x24,
                    false,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    3,
                    0x11C0,
                    8,
                    0x20,
                    true,
                    coreErrors);


                ComparePdoMappingMode(
                    i,
                    slave,
                    "Fixed",
                    policyDifferences);


                Compare8124SafeOpInitCommands(
                    i,
                    slave,
                    coverageWarnings);

                break;


            case 0x00006010U:
            {
                CompareSm(
                    i,
                    slave,
                    0,
                    0x1000,
                    128,
                    0x36,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    1,
                    0x10C0,
                    128,
                    0x32,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    2,
                    0x1180,
                    9,
                    0x24,
                    true,
                    coreErrors);


                CompareSm(
                    i,
                    slave,
                    3,
                    0x1480,
                    23,
                    0x00,
                    true,
                    coreErrors);


                const bool rxAssignMatch =
                    slave.runtimeProfile.hasRxPdoAssignIndex &&
                    slave.runtimeProfile.rxPdoAssignIndex ==
                    0x1C12U;


                const bool txAssignMatch =
                    slave.runtimeProfile.hasTxPdoAssignIndex &&
                    slave.runtimeProfile.txPdoAssignIndex ==
                    0x1C13U;


                if (!rxAssignMatch)
                {
                    coreErrors++;
                }


                if (!txAssignMatch)
                {
                    coreErrors++;
                }


                RtPrintf(
                    "[RUNTIME-EQUIVALENCE-PDO-ASSIGN] "
                    "S%d A3E | "
                    "Rx XML:0x%04X CPP:0x1C12 %s | "
                    "Tx XML:0x%04X CPP:0x1C13 %s\n",

                    i,

                    (unsigned int)
                    slave.runtimeProfile.rxPdoAssignIndex,

                    rxAssignMatch
                    ? "MATCH"
                    : "DIFF",

                    (unsigned int)
                    slave.runtimeProfile.txPdoAssignIndex,

                    txAssignMatch
                    ? "MATCH"
                    : "DIFF");


                ComparePdoEntries(
                    i,
                    "Rx",
                    FindRuntimePdo(
                        slave.runtimeRxPdos,
                        0x1601),
                    0x1601,
                    a3eRxMappings,
                    4,
                    coreErrors);


                ComparePdoEntries(
                    i,
                    "Tx",
                    FindRuntimePdo(
                        slave.runtimeTxPdos,
                        0x1A01),
                    0x1A01,
                    a3eTxMappings,
                    8,
                    coreErrors);


                CompareA3EDcPolicy(
                    i,
                    slave,
                    i ==
                    firstServoIndex,
                    policyDifferences);


                ComparePdoMappingMode(
                    i,
                    slave,
                    "Configurable",
                    policyDifferences);


                CompareA3EInformationalInitCommands(
                    i,
                    slave,
                    policyDifferences);

                break;
            }


            default:
                coverageWarnings++;


                RtPrintf(
                    "[RUNTIME-EQUIVALENCE-COVERAGE] "
                    "S%d | Delta Product:0x%08X has no Stage5C baseline.\n",

                    i,

                    (unsigned int)
                    slave.productCode);

                break;
            }
        }
        else
        {
            coverageWarnings++;


            RtPrintf(
                "[RUNTIME-EQUIVALENCE-COVERAGE] "
                "S%d | Vendor:0x%08X Product:0x%08X "
                "has no Stage5C baseline.\n",

                i,

                (unsigned int)
                slave.vendorId,

                (unsigned int)
                slave.productCode);
        }


        const int slaveCoreErrors =
            coreErrors -
            slaveCoreErrorsBefore;


        const int slavePolicyDiffs =
            policyDifferences -
            slavePolicyBefore;


        const int slaveCoverageWarnings =
            coverageWarnings -
            slaveWarningsBefore;


        RtPrintf(
            "[RUNTIME-EQUIVALENCE-SLAVE] "
            "S%d | CoreErrors:%d | "
            "PolicyDiffs:%d | CoverageWarnings:%d | "
            "Result:%s\n",

            i,

            slaveCoreErrors,

            slavePolicyDiffs,

            slaveCoverageWarnings,

            slaveCoreErrors == 0 &&
            slavePolicyDiffs == 0
            ? "EQUIVALENT"
            : "DIFFERENT");
    }


    const bool equivalent =
        coreErrors == 0 &&
        policyDifferences == 0;


    m_runtimeEquivalenceVerified =
        equivalent;


    RtPrintf(
        "[RUNTIME-EQUIVALENCE-RESULT] "
        "Compared:%d | "
        "CoreErrors:%d | "
        "PolicyDiffs:%d | "
        "CoverageWarnings:%d | "
        "Result:%s | "
        "StartupAction:CONTINUE | "
        "AuditHardwareWrite:NO | "
        "CutoverGate:%s\n",

        comparedSlaves,

        coreErrors,

        policyDifferences,

        coverageWarnings,

        equivalent
        ? "EQUIVALENT"
        : "NOT-EQUIVALENT",

        equivalent
        ? "OPEN"
        : "CLOSED");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-EQUIVALENCE] END | "
        "Stage:5C | Result:%s | AuditOnly:YES\n"
        "============================================================\n\n",

        equivalent
        ? "EQUIVALENT"
        : "NOT-EQUIVALENT");


    return
        equivalent;
}
