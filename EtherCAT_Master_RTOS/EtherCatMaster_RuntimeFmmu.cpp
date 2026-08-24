#include "EtherCatMaster.h"

#include <rtapi.h>
#include <rtssapi.h>

#include <algorithm>
#include <string.h>
#include <vector>

#include "GlobalConfig.h"


// ============================================================================
// Stage 6C - Runtime FMMU Cutover
//
// Runtime XML is now the FMMU configuration source.
//
// This file runs ONLY during Startup / PRE-OP.
//
// Safety model:
//
// 1. Stage 5 Runtime Equivalence Gate must be OPEN.
// 2. New Runtime XML must explicitly contain <Fmmus>.
// 3. Before ANY Runtime FMMU write, the complete plan is checked against
//    the exact Process Image allocation rule used by BuildIoMap():
//
//       slave order
//         Output bytes first
//         Input bytes second
//
// 4. The Runtime FMMU physical address must also match the ProcessData
//    PhysAddr already parsed from Runtime XML.
// 5. Every APWR is followed immediately by APRD readback.
// 6. Any failure aborts Startup in PRE-OP.
// 7. Old XML with no <Fmmus> keeps legacy Config_Slave_FMMU() fallback.
//
// No code here runs inside the 250 us cyclic PDO path.
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


    uint32_t ReadLe32(
        const uint8_t* data)
    {
        return
            (uint32_t)data[0] |
            (
                (uint32_t)data[1] <<
                8
                ) |
            (
                (uint32_t)data[2] <<
                16
                ) |
            (
                (uint32_t)data[3] <<
                24
                );
    }


    bool IsOutputFmmu(
        const EtherCatRuntimeFmmuConfig& fmmu)
    {
        return
            strcmp(
                fmmu.direction,
                "Output") ==
            0;
    }


    bool IsInputFmmu(
        const EtherCatRuntimeFmmuConfig& fmmu)
    {
        return
            strcmp(
                fmmu.direction,
                "Input") ==
            0;
    }


    bool RuntimeFmmuReadbackMatches(
        const EtherCatRuntimeFmmuConfig& expected,
        const uint8_t actual[16])
    {
        const uint32_t actualLogicalAddress =
            ReadLe32(
                &actual[0]);


        const uint16_t actualLogicalLength =
            ReadLe16(
                &actual[4]);


        const uint8_t actualLogicalStartBit =
            actual[6];


        const uint8_t actualLogicalEndBit =
            actual[7];


        const uint16_t actualPhysicalAddress =
            ReadLe16(
                &actual[8]);


        const uint8_t actualPhysicalStartBit =
            actual[10];


        const uint8_t actualType =
            actual[11];


        const bool actualEnabled =
            (
                actual[12] &
                0x01
                ) !=
            0;


        return
            actualLogicalAddress ==
            expected.logicalStartAddress &&
            actualLogicalLength ==
            expected.logicalLength &&
            actualLogicalStartBit ==
            expected.logicalStartBit &&
            actualLogicalEndBit ==
            expected.logicalEndBit &&
            actualPhysicalAddress ==
            expected.physicalStartAddress &&
            actualPhysicalStartBit ==
            expected.physicalStartBit &&
            actualType ==
            expected.type &&
            actualEnabled ==
            expected.enabled;
    }
}


// ============================================================================
// ShouldUseRuntimeFmmuConfiguration
// ============================================================================

bool EtherCatMaster::ShouldUseRuntimeFmmuConfiguration(
    int slaveIdx) const
{
    if (!m_runtimeEquivalenceVerified)
    {
        return
            false;
    }


    if (m_pEni == nullptr)
    {
        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    if (slaveIdx < 0 ||
        slaveIdx >=
        (int)slaves.size())
    {
        return
            false;
    }


    return
        slaves[(size_t)slaveIdx]
        .runtimeFmmuSchemaPresent;
}


// ============================================================================
// PreflightRuntimeFmmuConfiguration
// ============================================================================

bool EtherCatMaster::PreflightRuntimeFmmuConfiguration()
{
    if (!m_runtimeEquivalenceVerified)
    {
        RtPrintf(
            "[RUNTIME-FMMU-PREFLIGHT] "
            "Stage5 CutoverGate:CLOSED | "
            "Action:LEGACY_ALL | Result:PASS\n");


        return
            true;
    }


    if (m_pEni == nullptr ||
        m_IoMapSize < 0 ||
        m_IoMapSize >
        (int)sizeof(m_IoMap))
    {
        RtPrintf(
            "[RUNTIME-FMMU-PREFLIGHT] "
            "Invalid Master Process Image state | Result:FAIL\n");


        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    uint32_t expectedLogicalOffsetBytes =
        0;


    int runtimeSlaveCount =
        0;


    int legacySlaveCount =
        0;


    int runtimeFmmuCount =
        0;


    int errors =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-FMMU-PREFLIGHT] BEGIN | "
        "Stage:6C | Source:RUNTIME_XML | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIdx = 0;
        slaveIdx <
        (int)slaves.size();
        slaveIdx++)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)slaveIdx];


        const uint32_t outputBytes =
            (
                slave.outputBitLength +
                7U
                ) /
            8U;


        const uint32_t inputBytes =
            (
                slave.inputBitLength +
                7U
                ) /
            8U;


        const uint32_t expectedOutputOffset =
            expectedLogicalOffsetBytes;


        expectedLogicalOffsetBytes +=
            outputBytes;


        const uint32_t expectedInputOffset =
            expectedLogicalOffsetBytes;


        expectedLogicalOffsetBytes +=
            inputBytes;


        const bool hasProcessData =
            slave.outputBitLength >
            0U ||
            slave.inputBitLength >
            0U;


        if (!slave.runtimeFmmuSchemaPresent)
        {
            if (hasProcessData)
            {
                legacySlaveCount++;


                RtPrintf(
                    "[RUNTIME-FMMU-PREFLIGHT] "
                    "S%d | FmmuSchema:NO | "
                    "ProcessData:YES | Action:LEGACY_FALLBACK | Result:PASS\n",

                    slaveIdx);
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-FMMU-PREFLIGHT] "
                    "S%d | FmmuSchema:NO | ProcessData:NO | Result:N/A\n",

                    slaveIdx);
            }


            continue;
        }


        runtimeSlaveCount++;


        if (!hasProcessData)
        {
            if (!slave.runtimeFmmus.empty())
            {
                errors++;


                RtPrintf(
                    "[RUNTIME-FMMU-PREFLIGHT] "
                    "S%d | ProcessData:NO but RuntimeFMMU:%u | Result:FAIL\n",

                    slaveIdx,

                    (unsigned int)
                    slave.runtimeFmmus.size());
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-FMMU-PREFLIGHT] "
                    "S%d | ProcessData:NO | RuntimeFMMU:0 | Result:PASS\n",

                    slaveIdx);
            }


            continue;
        }


        if (slave.runtimeFmmus.empty())
        {
            errors++;


            RtPrintf(
                "[RUNTIME-FMMU-PREFLIGHT] "
                "S%d | FmmuSchema:YES but RuntimeFMMU:0 | "
                "ProcessData exists | Result:FAIL\n",

                slaveIdx);


            continue;
        }


        int outputCount =
            0;


        int inputCount =
            0;


        bool usedIndex[16] =
        {
            false
        };


        for (const auto& fmmu :
            slave.runtimeFmmus)
        {
            runtimeFmmuCount++;


            bool pass =
                true;


            if (fmmu.index < 0 ||
                fmmu.index >
                15)
            {
                pass =
                    false;
            }
            else
            {
                if (usedIndex[fmmu.index])
                {
                    pass =
                        false;
                }


                usedIndex[fmmu.index] =
                    true;
            }


            if (fmmu.logicalLength ==
                0U ||
                fmmu.logicalStartBit >
                7U ||
                fmmu.logicalEndBit >
                7U ||
                fmmu.physicalStartBit >
                7U ||
                !fmmu.enabled)
            {
                pass =
                    false;
            }


            if (IsOutputFmmu(
                fmmu))
            {
                outputCount++;


                const uint8_t expectedEndBit =
                    slave.outputBitLength >
                    0U
                    ? (uint8_t)(
                        (
                            slave.outputBitLength -
                            1U
                            ) %
                        8U)
                    : 0U;


                if (fmmu.type !=
                    2U ||
                    slave.outputBitLength ==
                    0U ||
                    outputCount >
                    1 ||
                    fmmu.logicalStartAddress !=
                    expectedOutputOffset ||
                    fmmu.logicalLength !=
                    outputBytes ||
                    fmmu.logicalStartBit !=
                    0U ||
                    fmmu.logicalEndBit !=
                    expectedEndBit ||
                    fmmu.physicalStartAddress !=
                    slave.configAddrOut ||
                    fmmu.physicalStartBit !=
                    0U)
                {
                    pass =
                        false;
                }
            }
            else if (IsInputFmmu(
                fmmu))
            {
                inputCount++;


                const uint8_t expectedEndBit =
                    slave.inputBitLength >
                    0U
                    ? (uint8_t)(
                        (
                            slave.inputBitLength -
                            1U
                            ) %
                        8U)
                    : 0U;


                if (fmmu.type !=
                    1U ||
                    slave.inputBitLength ==
                    0U ||
                    inputCount >
                    1 ||
                    fmmu.logicalStartAddress !=
                    expectedInputOffset ||
                    fmmu.logicalLength !=
                    inputBytes ||
                    fmmu.logicalStartBit !=
                    0U ||
                    fmmu.logicalEndBit !=
                    expectedEndBit ||
                    fmmu.physicalStartAddress !=
                    slave.configAddrIn ||
                    fmmu.physicalStartBit !=
                    0U)
                {
                    pass =
                        false;
                }
            }
            else
            {
                pass =
                    false;
            }


            const uint64_t logicalEnd =
                (uint64_t)
                fmmu.logicalStartAddress +
                (uint64_t)
                fmmu.logicalLength;


            if (logicalEnd >
                (uint64_t)m_IoMapSize)
            {
                pass =
                    false;
            }


            if (!pass)
            {
                errors++;
            }


            RtPrintf(
                "[RUNTIME-FMMU-PREFLIGHT] "
                "S%d FMMU%d %s | "
                "Logical:0x%08X/%u [%u..%u] | "
                "Physical:0x%04X.%u | "
                "Type:%u Enable:%d | "
                "ExpectedOutOff:%u ExpectedInOff:%u | "
                "Result:%s\n",

                slaveIdx,
                fmmu.index,

                fmmu.direction[0] != '\0'
                ? fmmu.direction
                : "N/A",

                (unsigned int)
                fmmu.logicalStartAddress,

                (unsigned int)
                fmmu.logicalLength,

                (unsigned int)
                fmmu.logicalStartBit,

                (unsigned int)
                fmmu.logicalEndBit,

                (unsigned int)
                fmmu.physicalStartAddress,

                (unsigned int)
                fmmu.physicalStartBit,

                (unsigned int)
                fmmu.type,

                fmmu.enabled
                ? 1
                : 0,

                (unsigned int)
                expectedOutputOffset,

                (unsigned int)
                expectedInputOffset,

                pass
                ? "PASS"
                : "FAIL");
        }


        const int expectedOutputCount =
            slave.outputBitLength >
            0U
            ? 1
            : 0;


        const int expectedInputCount =
            slave.inputBitLength >
            0U
            ? 1
            : 0;


        if (outputCount !=
            expectedOutputCount ||
            inputCount !=
            expectedInputCount)
        {
            errors++;


            RtPrintf(
                "[RUNTIME-FMMU-PREFLIGHT] "
                "S%d | DirectionCount "
                "Out:%d/%d In:%d/%d | Result:FAIL\n",

                slaveIdx,

                outputCount,
                expectedOutputCount,

                inputCount,
                expectedInputCount);
        }
    }


    if (expectedLogicalOffsetBytes !=
        (uint32_t)m_IoMapSize)
    {
        errors++;


        RtPrintf(
            "[RUNTIME-FMMU-PREFLIGHT] "
            "ProcessImage layout mismatch | "
            "ExpectedFromSlaves:%u ActualMap:%d | Result:FAIL\n",

            (unsigned int)
            expectedLogicalOffsetBytes,

            m_IoMapSize);
    }


    const bool pass =
        errors ==
        0;


    RtPrintf(
        "[RUNTIME-FMMU-PREFLIGHT-RESULT] "
        "RuntimeSlaves:%d | LegacySlaves:%d | "
        "FmmuEntries:%d | "
        "ProcessImage:%u/%d B | "
        "Errors:%d | Result:%s | HardwareWrite:NO\n",

        runtimeSlaveCount,
        legacySlaveCount,
        runtimeFmmuCount,

        (unsigned int)
        expectedLogicalOffsetBytes,

        m_IoMapSize,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-FMMU-PREFLIGHT] END | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// ConfigureRuntimeFmmus
// ============================================================================

bool EtherCatMaster::ConfigureRuntimeFmmus(
    int slaveIdx)
{
    if (!ShouldUseRuntimeFmmuConfiguration(
        slaveIdx))
    {
        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const EtherCatSlave& slave =
        slaves[(size_t)slaveIdx];


    if (slave.runtimeFmmus.empty())
    {
        // Explicit schema + no ProcessData is valid.
        if (slave.outputBitLength ==
            0U &&
            slave.inputBitLength ==
            0U)
        {
            RtPrintf(
                "[RUNTIME-FMMU-CUTOVER] "
                "S%d | ProcessData:NO | FMMU:0 | Result:PASS\n",

                slaveIdx);


            return
                true;
        }


        RtPrintf(
            "[RUNTIME-FMMU-CUTOVER] "
            "S%d | Explicit schema but FMMU list is empty | Result:FAIL\n",

            slaveIdx);


        return
            false;
    }


    RtPrintf(
        "[RUNTIME-FMMU-CUTOVER] "
        "S%d | Source:RUNTIME_XML | FMMU:%u | BEGIN\n",

        slaveIdx,

        (unsigned int)
        slave.runtimeFmmus.size());


    LARGE_INTEGER retryWait;

    retryWait.QuadPart =
        1 *
        10000;


    int applied =
        0;


    for (const auto& fmmu :
        slave.runtimeFmmus)
    {
        uint8_t raw[16] =
        {
            0
        };


        raw[0] =
            (uint8_t)(
                fmmu.logicalStartAddress &
                0xFFU);


        raw[1] =
            (uint8_t)(
                (
                    fmmu.logicalStartAddress >>
                    8
                    ) &
                0xFFU);


        raw[2] =
            (uint8_t)(
                (
                    fmmu.logicalStartAddress >>
                    16
                    ) &
                0xFFU);


        raw[3] =
            (uint8_t)(
                (
                    fmmu.logicalStartAddress >>
                    24
                    ) &
                0xFFU);


        raw[4] =
            (uint8_t)(
                fmmu.logicalLength &
                0xFFU);


        raw[5] =
            (uint8_t)(
                (
                    fmmu.logicalLength >>
                    8
                    ) &
                0xFFU);


        raw[6] =
            fmmu.logicalStartBit;


        raw[7] =
            fmmu.logicalEndBit;


        raw[8] =
            (uint8_t)(
                fmmu.physicalStartAddress &
                0xFFU);


        raw[9] =
            (uint8_t)(
                (
                    fmmu.physicalStartAddress >>
                    8
                    ) &
                0xFFU);


        raw[10] =
            fmmu.physicalStartBit;


        raw[11] =
            fmmu.type;


        raw[12] =
            fmmu.enabled
            ? 0x01
            : 0x00;


        const uint16_t registerAddress =
            (uint16_t)(
                0x0600 +
                (
                    fmmu.index *
                    16
                    ));


        int writeWkc =
            0;


        int retry =
            0;


        while (retry <
            5)
        {
            writeWkc =
                ecx_APWR(
                    m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                    registerAddress,
                    16,
                    raw,
                    20);


            if (writeWkc >=
                1)
            {
                break;
            }


            retry++;


            RtSleepFt(
                &retryWait);
        }


        if (writeWkc <
            1)
        {
            RtPrintf(
                "[RUNTIME-FMMU-APPLY] "
                "S%d FMMU%d | Reg:0x%04X | "
                "WriteWKC:%d Retry:%d | Result:FAIL\n",

                slaveIdx,
                fmmu.index,

                (unsigned int)
                registerAddress,

                writeWkc,
                retry);


            return
                false;
        }


        uint8_t readback[16] =
        {
            0
        };


        const int readWkc =
            ecx_APRD(
                m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                registerAddress,
                16,
                readback,
                20);


        const bool match =
            readWkc >=
            1 &&
            RuntimeFmmuReadbackMatches(
                fmmu,
                readback);


        RtPrintf(
            "[RUNTIME-FMMU-APPLY] "
            "S%d FMMU%d %s | "
            "Logical:0x%08X/%u [%u..%u] | "
            "Physical:0x%04X.%u | "
            "Type:%u Enable:%d | "
            "WriteWKC:%d ReadWKC:%d | "
            "Readback:%s | Result:%s\n",

            slaveIdx,
            fmmu.index,

            fmmu.direction[0] != '\0'
            ? fmmu.direction
            : "N/A",

            (unsigned int)
            fmmu.logicalStartAddress,

            (unsigned int)
            fmmu.logicalLength,

            (unsigned int)
            fmmu.logicalStartBit,

            (unsigned int)
            fmmu.logicalEndBit,

            (unsigned int)
            fmmu.physicalStartAddress,

            (unsigned int)
            fmmu.physicalStartBit,

            (unsigned int)
            fmmu.type,

            fmmu.enabled
            ? 1
            : 0,

            writeWkc,
            readWkc,

            match
            ? "MATCH"
            : "DIFF",

            match
            ? "PASS"
            : "FAIL");


        if (!match)
        {
            return
                false;
        }


        applied++;
    }


    RtPrintf(
        "[RUNTIME-FMMU-CUTOVER-RESULT] "
        "S%d | Applied:%d/%u | Result:PASS\n",

        slaveIdx,
        applied,

        (unsigned int)
        slave.runtimeFmmus.size());


    return
        true;
}
