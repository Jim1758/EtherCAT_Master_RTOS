#include "EtherCatMaster.h"

#include <rtapi.h>

#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 8B - Generic Runtime PRE-OP Configuration Safety Gate
//
// Purpose:
//
// The existing Stage-2 verifier is still ProductCode-specific.
// That means a future ESI-supported device could be configured correctly by
// Runtime XML but still be rejected because the C++ verifier has no hardcoded
// case.
//
// Stage 8B promotes the proven generic verifier driven only by Runtime XML:
//
// - runtimeSyncManagers
// - runtimeProfile.pdoMappingMode
// - runtimeProfile.rxPdoAssignIndex / txPdoAssignIndex
// - runtimeRxPdos / runtimeTxPdos
//
// IMPORTANT:
//
// - Readback only.
// - No APWR / FPWR / SDO write.
// - Existing VerifyPreOpSmAndPdoConfiguration() remains the real gate.
// - Stage 8A failure is logged but does not alter Startup.
// ============================================================================

namespace
{
    uint16_t ReadLe16(
        const uint8_t* data)
    {
        return
            (uint16_t)(
                (uint16_t)data[0] |
                (
                    (uint16_t)data[1] <<
                    8
                    ));
    }


    bool StringEquals(
        const char* a,
        const char* b)
    {
        return
            a != nullptr &&
            b != nullptr &&
            strcmp(
                a,
                b) ==
            0;
    }
}


// ============================================================================
// AuditRuntimePreOpConfigurationGeneric
// ============================================================================

bool EtherCatMaster::AuditRuntimePreOpConfigurationGeneric()
{
    if (m_pEni ==
        nullptr)
    {
        RtPrintf(
            "[RUNTIME-PREOP-VERIFY] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int smCheckCount =
        0;


    int pdoCheckCount =
        0;


    int skipCount =
        0;


    int coverageErrors =
        0;


    int errorCount =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-PREOP-VERIFY] BEGIN | "
        "Stage:8B | Mode:SAFETY_GATE_READ_ONLY | "
        "ExpectedSource:RUNTIME_XML | HardwareWrite:NO\n"
        "============================================================\n");


    // ========================================================================
    // Generic helpers
    // ========================================================================

    auto verifySm =
        [this,
        &smCheckCount,
        &errorCount]
    (
        int slaveIdx,
        const EtherCatRuntimeSyncManagerConfig& sm
        ) -> bool
    {
        smCheckCount++;


        if (sm.index <
            0 ||
            sm.index >
            15)
        {
            errorCount++;


            RtPrintf(
                "[RUNTIME-SM-VERIFY] "
                "S%d SM%d | Invalid Runtime index | Result:FAIL\n",

                slaveIdx,
                sm.index);


            return
                false;
        }


        uint8_t raw[8] =
        {
            0
        };


        const uint16_t registerAddress =
            (uint16_t)(
                0x0800 +
                (
                    sm.index *
                    8
                    ));


        const int wkc =
            ecx_APRD(
                m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                registerAddress,
                8,
                raw,
                20);


        const uint16_t actualStart =
            ReadLe16(
                &raw[0]);


        const uint16_t actualLength =
            ReadLe16(
                &raw[2]);


        const uint8_t actualControl =
            raw[4];


        const bool actualEnable =
            (
                raw[6] &
                0x01
                ) !=
            0;


        const bool pass =
            wkc >=
            1 &&
            actualStart ==
            sm.startAddress &&
            actualLength ==
            sm.length &&
            actualControl ==
            sm.controlByte &&
            actualEnable ==
            sm.enabled;


        if (!pass)
        {
            errorCount++;
        }


        RtPrintf(
            "[RUNTIME-SM-VERIFY] "
            "S%d SM%d | "
            "Start XML:0x%04X ACT:0x%04X %s | "
            "Len XML:%u ACT:%u %s | "
            "Ctrl XML:0x%02X ACT:0x%02X %s | "
            "Enable XML:%d ACT:%d %s | "
            "WKC:%d | Result:%s\n",

            slaveIdx,
            sm.index,

            (unsigned int)
            sm.startAddress,

            (unsigned int)
            actualStart,

            actualStart ==
            sm.startAddress
            ? "MATCH"
            : "DIFF",

            (unsigned int)
            sm.length,

            (unsigned int)
            actualLength,

            actualLength ==
            sm.length
            ? "MATCH"
            : "DIFF",

            (unsigned int)
            sm.controlByte,

            (unsigned int)
            actualControl,

            actualControl ==
            sm.controlByte
            ? "MATCH"
            : "DIFF",

            sm.enabled
            ? 1
            : 0,

            actualEnable
            ? 1
            : 0,

            actualEnable ==
            sm.enabled
            ? "MATCH"
            : "DIFF",

            wkc,

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    };


    auto verifySdoU8 =
        [this,
        &pdoCheckCount,
        &errorCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint8_t expected,
        const char* stage
        ) -> bool
    {
        pdoCheckCount++;


        uint8_t actual =
            0;


        int size =
            1;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result >
            0 &&
            size ==
            1;


        const bool pass =
            readOk &&
            actual ==
            expected;


        if (!pass)
        {
            errorCount++;
        }


        RtPrintf(
            "[RUNTIME-PDO-VERIFY] "
            "S%d | %s | %04X:%02X | "
            "U8 XML:0x%02X ACT:0x%02X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            stage,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    };


    auto verifySdoU16 =
        [this,
        &pdoCheckCount,
        &errorCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint16_t expected,
        const char* stage
        ) -> bool
    {
        pdoCheckCount++;


        uint16_t actual =
            0;


        int size =
            2;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result >
            0 &&
            size ==
            2;


        const bool pass =
            readOk &&
            actual ==
            expected;


        if (!pass)
        {
            errorCount++;
        }


        RtPrintf(
            "[RUNTIME-PDO-VERIFY] "
            "S%d | %s | %04X:%02X | "
            "U16 XML:0x%04X ACT:0x%04X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            stage,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    };


    auto verifySdoU32 =
        [this,
        &pdoCheckCount,
        &errorCount]
    (
        int slaveIdx,
        uint16_t index,
        uint8_t subIndex,
        uint32_t expected,
        const char* stage
        ) -> bool
    {
        pdoCheckCount++;


        uint32_t actual =
            0;


        int size =
            4;


        const int result =
            ecx_SDOread(
                slaveIdx,
                index,
                subIndex,
                FALSE,
                &size,
                &actual,
                20);


        const bool readOk =
            result >
            0 &&
            size ==
            4;


        const bool pass =
            readOk &&
            actual ==
            expected;


        if (!pass)
        {
            errorCount++;
        }


        RtPrintf(
            "[RUNTIME-PDO-VERIFY] "
            "S%d | %s | %04X:%02X | "
            "U32 XML:0x%08X ACT:0x%08X | "
            "Read:%s | Result:%s\n",

            slaveIdx,

            stage,

            (unsigned int)
            index,

            (unsigned int)
            subIndex,

            (unsigned int)
            expected,

            (unsigned int)
            actual,

            readOk
            ? "OK"
            : "FAIL",

            pass
            ? "PASS"
            : "FAIL");


        return
            pass;
    };


    auto verifyPdoDirection =
        [&verifySdoU8,
        &verifySdoU16,
        &verifySdoU32,
        &coverageErrors,
        &errorCount]
    (
        int slaveIdx,
        const char* direction,
        uint16_t assignIndex,
        bool hasAssignIndex,
        const std::vector<EtherCatRuntimePdoConfig>& pdos
        ) -> void
    {
        if (pdos.empty())
        {
            return;
        }


        if (!hasAssignIndex ||
            assignIndex ==
            0)
        {
            coverageErrors++;
            errorCount++;


            RtPrintf(
                "[RUNTIME-PDO-VERIFY-COVERAGE] "
                "S%d | %s | Runtime PDO exists but assignment index missing | "
                "Result:FAIL\n",

                slaveIdx,
                direction);


            return;
        }


        verifySdoU8(
            slaveIdx,
            assignIndex,
            0x00,
            (uint8_t)
            pdos.size(),
            direction);


        for (size_t pdoIndex = 0;
            pdoIndex <
            pdos.size();
            pdoIndex++)
        {
            const EtherCatRuntimePdoConfig& pdo =
                pdos[pdoIndex];


            verifySdoU16(
                slaveIdx,
                assignIndex,
                (uint8_t)(
                    pdoIndex +
                    1U),
                pdo.index,
                direction);


            verifySdoU8(
                slaveIdx,
                pdo.index,
                0x00,
                (uint8_t)
                pdo.entries.size(),
                direction);


            for (size_t entryIndex = 0;
                entryIndex <
                pdo.entries.size();
                entryIndex++)
            {
                const EtherCatRuntimePdoEntryConfig& entry =
                    pdo.entries[entryIndex];


                verifySdoU32(
                    slaveIdx,
                    pdo.index,
                    (uint8_t)(
                        entryIndex +
                        1U),
                    entry.mappingValue,
                    direction);
            }
        }
    };


    // ========================================================================
    // Runtime-driven verification
    // ========================================================================

    for (int slaveIdx = 0;
        slaveIdx <
        (int)slaves.size();
        slaveIdx++)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)slaveIdx];


        const bool hasProcessData =
            slave.outputBitLength >
            0U ||
            slave.inputBitLength >
            0U;


        const bool hasMailbox =
            slave.runtimeMailbox.present;


        // --------------------------------------------------------------------
        // SyncManager
        // --------------------------------------------------------------------

        if (slave.runtimeSyncManagers.empty())
        {
            if (hasProcessData ||
                hasMailbox)
            {
                coverageErrors++;
                errorCount++;


                RtPrintf(
                    "[RUNTIME-SM-VERIFY-COVERAGE] "
                    "S%d | Runtime SM schema missing while transport exists | "
                    "Result:FAIL\n",

                    slaveIdx);
            }
            else
            {
                skipCount++;


                RtPrintf(
                    "[RUNTIME-SM-VERIFY] "
                    "S%d | Runtime SM:0 | ProcessData:NO Mailbox:NO | "
                    "Result:N/A\n",

                    slaveIdx);
            }
        }
        else
        {
            for (const auto& sm :
                slave.runtimeSyncManagers)
            {
                verifySm(
                    slaveIdx,
                    sm);
            }
        }


        // --------------------------------------------------------------------
        // PDO
        //
        // Fixed:
        //     no SDO remap/readback is required.
        //
        // Configurable:
        //     verify complete assignment + mapping from Runtime XML.
        // --------------------------------------------------------------------

        if (!slave.runtimeProfile.present ||
            !slave.runtimeProfile.hasPdoMappingMode)
        {
            if (!slave.runtimeRxPdos.empty() ||
                !slave.runtimeTxPdos.empty())
            {
                coverageErrors++;
                errorCount++;


                RtPrintf(
                    "[RUNTIME-PDO-VERIFY-COVERAGE] "
                    "S%d | Runtime PDO exists but PdoMappingMode missing | "
                    "Result:FAIL\n",

                    slaveIdx);
            }


            continue;
        }


        if (StringEquals(
            slave.runtimeProfile.pdoMappingMode,
            "Fixed"))
        {
            RtPrintf(
                "[RUNTIME-PDO-VERIFY] "
                "S%d | MapMode:Fixed | "
                "Action:NO_SDO_MAPPING_READBACK | Result:N/A\n",

                slaveIdx);


            continue;
        }


        if (!StringEquals(
            slave.runtimeProfile.pdoMappingMode,
            "Configurable"))
        {
            coverageErrors++;
            errorCount++;


            RtPrintf(
                "[RUNTIME-PDO-VERIFY-COVERAGE] "
                "S%d | Unknown MapMode:%s | Result:FAIL\n",

                slaveIdx,

                slave.runtimeProfile.pdoMappingMode);


            continue;
        }


        verifyPdoDirection(
            slaveIdx,
            "RxPDO",
            slave.runtimeProfile.rxPdoAssignIndex,
            slave.runtimeProfile.hasRxPdoAssignIndex,
            slave.runtimeRxPdos);


        verifyPdoDirection(
            slaveIdx,
            "TxPDO",
            slave.runtimeProfile.txPdoAssignIndex,
            slave.runtimeProfile.hasTxPdoAssignIndex,
            slave.runtimeTxPdos);
    }


    const bool pass =
        errorCount ==
        0;


    RtPrintf(
        "[RUNTIME-PREOP-VERIFY-RESULT] "
        "Slaves:%u | "
        "SMChecks:%d | "
        "PDOChecks:%d | "
        "Skips:%d | "
        "CoverageErrors:%d | "
        "Errors:%d | "
        "Result:%s | "
        "ExpectedSource:RUNTIME_XML | "
        "HardwareWrite:NO | "
        "SafetyGate:RUNTIME_STAGE8B\n",

        (unsigned int)
        slaves.size(),

        smCheckCount,
        pdoCheckCount,
        skipCount,
        coverageErrors,
        errorCount,

        pass
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-PREOP-VERIFY] END | "
        "Stage:8B | Result:%s | ShadowOnly:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
