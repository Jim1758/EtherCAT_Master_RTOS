#include "EtherCatMaster.h"

#include <rtapi.h>
#include <string.h>

#include "GlobalConfig.h"


// ============================================================================
// Stage 6A - Runtime Transport Shadow Audit
//
// Runtime XML now describes Mailbox and FMMU explicitly.
//
// This file is READ / COMPARE ONLY.
//
// Existing verified paths remain authoritative:
// - InitSlaveMailboxInfo()
// - Config_Slave_FMMU()
//
// HardwareWrite:NO
// StartupAction:CONTINUE
// ============================================================================

namespace
{
    uint16_t ReadLe16(const uint8_t* data)
    {
        return
            (uint16_t)(
                (uint16_t)data[0] |
                ((uint16_t)data[1] << 8));
    }


    uint32_t ReadLe32(const uint8_t* data)
    {
        return
            (uint32_t)data[0] |
            ((uint32_t)data[1] << 8) |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[3] << 24);
    }
}


bool EtherCatMaster::AuditRuntimeTransportConfiguration()
{
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[RUNTIME-TRANSPORT-AUDIT] "
            "Runtime Config is not attached | Result:FAIL\n");

        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int mailboxChecks = 0;
    int fmmuChecks = 0;
    int schemaMissing = 0;
    int errorCount = 0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-TRANSPORT-AUDIT] BEGIN | "
        "Stage:6A | Mode:SHADOW_READ_ONLY | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIdx = 0;
        slaveIdx < (int)slaves.size();
        ++slaveIdx)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)slaveIdx];


        // ====================================================================
        // Mailbox
        // ====================================================================

        const bool softwareMailboxPresent =
            m_slaveInfo[slaveIdx].mbxOutLength > 0 ||
            m_slaveInfo[slaveIdx].mbxInLength > 0;


        if (!slave.runtimeMailbox.present)
        {
            if (softwareMailboxPresent)
            {
                ++schemaMissing;
                ++errorCount;

                RtPrintf(
                    "[RUNTIME-TRANSPORT-MAILBOX] "
                    "S%d | Runtime:NOT_PRESENT | "
                    "Software Out:0x%04X/%u In:0x%04X/%u | Result:FAIL\n",
                    slaveIdx,
                    (unsigned int)m_slaveInfo[slaveIdx].mbxOutAddr,
                    (unsigned int)m_slaveInfo[slaveIdx].mbxOutLength,
                    (unsigned int)m_slaveInfo[slaveIdx].mbxInAddr,
                    (unsigned int)m_slaveInfo[slaveIdx].mbxInLength);
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-TRANSPORT-MAILBOX] "
                    "S%d | Runtime:NO | Software:NO | Result:N/A\n",
                    slaveIdx);
            }
        }
        else
        {
            const EtherCatRuntimeMailboxDirectionConfig* directions[2] =
            {
                &slave.runtimeMailbox.out,
                &slave.runtimeMailbox.in
            };

            const char* labels[2] =
            {
                "OUT",
                "IN"
            };


            for (int directionIndex = 0;
                directionIndex < 2;
                ++directionIndex)
            {
                const EtherCatRuntimeMailboxDirectionConfig& expected =
                    *directions[directionIndex];

                if (!expected.present)
                {
                    continue;
                }


                ++mailboxChecks;


                const uint16_t softwareAddress =
                    directionIndex == 0
                    ? m_slaveInfo[slaveIdx].mbxOutAddr
                    : m_slaveInfo[slaveIdx].mbxInAddr;

                const uint16_t softwareLength =
                    directionIndex == 0
                    ? m_slaveInfo[slaveIdx].mbxOutLength
                    : m_slaveInfo[slaveIdx].mbxInLength;


                const bool softwareMatch =
                    softwareAddress == expected.startAddress &&
                    softwareLength == expected.length;


                uint8_t raw[8] = { 0 };
                int readWkc = 0;


                if (expected.smIndex >= 0 &&
                    expected.smIndex <= 15)
                {
                    const uint16_t smRegister =
                        (uint16_t)(
                            0x0800 +
                            (expected.smIndex * 8));

                    readWkc =
                        ecx_APRD(
                            m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                            smRegister,
                            8,
                            raw,
                            20);
                }


                const uint16_t actualAddress =
                    ReadLe16(&raw[0]);

                const uint16_t actualLength =
                    ReadLe16(&raw[2]);

                const uint8_t actualControl =
                    raw[4];

                const bool actualEnabled =
                    (raw[6] & 0x01) != 0;


                const bool escMatch =
                    readWkc >= 1 &&
                    actualAddress == expected.startAddress &&
                    actualLength == expected.length &&
                    actualControl == expected.controlByte &&
                    actualEnabled == expected.enabled;


                const bool pass =
                    softwareMatch && escMatch;


                if (!pass)
                {
                    ++errorCount;
                }


                RtPrintf(
                    "[RUNTIME-TRANSPORT-MAILBOX] "
                    "S%d %s | "
                    "SW XML:0x%04X/%u CPP:0x%04X/%u %s | "
                    "ESC SM%d XML:0x%04X/%u Ctrl:0x%02X En:%d | "
                    "ACT:0x%04X/%u Ctrl:0x%02X En:%d WKC:%d %s | "
                    "Result:%s\n",
                    slaveIdx,
                    labels[directionIndex],
                    (unsigned int)expected.startAddress,
                    (unsigned int)expected.length,
                    (unsigned int)softwareAddress,
                    (unsigned int)softwareLength,
                    softwareMatch ? "MATCH" : "DIFF",
                    expected.smIndex,
                    (unsigned int)expected.startAddress,
                    (unsigned int)expected.length,
                    (unsigned int)expected.controlByte,
                    expected.enabled ? 1 : 0,
                    (unsigned int)actualAddress,
                    (unsigned int)actualLength,
                    (unsigned int)actualControl,
                    actualEnabled ? 1 : 0,
                    readWkc,
                    escMatch ? "MATCH" : "DIFF",
                    pass ? "PASS" : "FAIL");
            }
        }


        // ====================================================================
        // FMMU
        // ====================================================================

        const bool hasProcessData =
            slave.outputBitLength > 0 ||
            slave.inputBitLength > 0;


        if (slave.runtimeFmmus.empty())
        {
            if (hasProcessData)
            {
                ++schemaMissing;
                ++errorCount;

                RtPrintf(
                    "[RUNTIME-TRANSPORT-FMMU] "
                    "S%d | Runtime FMMU plan missing while ProcessData exists | "
                    "Result:FAIL\n",
                    slaveIdx);
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-TRANSPORT-FMMU] "
                    "S%d | ProcessData:NO | RuntimeFMMU:0 | Result:N/A\n",
                    slaveIdx);
            }

            continue;
        }


        for (const auto& expected :
            slave.runtimeFmmus)
        {
            ++fmmuChecks;


            if (expected.index < 0 ||
                expected.index > 15)
            {
                ++errorCount;

                RtPrintf(
                    "[RUNTIME-TRANSPORT-FMMU] "
                    "S%d FMMU%d | Invalid Runtime index | Result:FAIL\n",
                    slaveIdx,
                    expected.index);

                continue;
            }


            uint8_t raw[16] = { 0 };

            const uint16_t registerAddress =
                (uint16_t)(
                    0x0600 +
                    (expected.index * 16));


            const int readWkc =
                ecx_APRD(
                    m_slaveInfo[slaveIdx].APRDAPWR_Addr,
                    registerAddress,
                    16,
                    raw,
                    20);


            const uint32_t actualLogicalAddress =
                ReadLe32(&raw[0]);

            const uint16_t actualLogicalLength =
                ReadLe16(&raw[4]);

            const uint8_t actualLogicalStartBit =
                raw[6];

            const uint8_t actualLogicalEndBit =
                raw[7];

            const uint16_t actualPhysicalAddress =
                ReadLe16(&raw[8]);

            const uint8_t actualPhysicalStartBit =
                raw[10];

            const uint8_t actualType =
                raw[11];

            const bool actualEnabled =
                (raw[12] & 0x01) != 0;


            const bool pass =
                readWkc >= 1 &&
                actualLogicalAddress == expected.logicalStartAddress &&
                actualLogicalLength == expected.logicalLength &&
                actualLogicalStartBit == expected.logicalStartBit &&
                actualLogicalEndBit == expected.logicalEndBit &&
                actualPhysicalAddress == expected.physicalStartAddress &&
                actualPhysicalStartBit == expected.physicalStartBit &&
                actualType == expected.type &&
                actualEnabled == expected.enabled;


            if (!pass)
            {
                ++errorCount;
            }


            RtPrintf(
                "[RUNTIME-TRANSPORT-FMMU] "
                "S%d FMMU%d %s | "
                "Logical XML:0x%08X/%u [%u..%u] "
                "ACT:0x%08X/%u [%u..%u] | "
                "Physical XML:0x%04X.%u ACT:0x%04X.%u | "
                "Type XML:%u ACT:%u | "
                "Enable XML:%d ACT:%d | "
                "WKC:%d | Result:%s\n",
                slaveIdx,
                expected.index,
                expected.direction[0] != '\0'
                ? expected.direction
                : "N/A",
                (unsigned int)expected.logicalStartAddress,
                (unsigned int)expected.logicalLength,
                (unsigned int)expected.logicalStartBit,
                (unsigned int)expected.logicalEndBit,
                (unsigned int)actualLogicalAddress,
                (unsigned int)actualLogicalLength,
                (unsigned int)actualLogicalStartBit,
                (unsigned int)actualLogicalEndBit,
                (unsigned int)expected.physicalStartAddress,
                (unsigned int)expected.physicalStartBit,
                (unsigned int)actualPhysicalAddress,
                (unsigned int)actualPhysicalStartBit,
                (unsigned int)expected.type,
                (unsigned int)actualType,
                expected.enabled ? 1 : 0,
                actualEnabled ? 1 : 0,
                readWkc,
                pass ? "PASS" : "FAIL");
        }
    }


    const bool pass =
        errorCount == 0;


    RtPrintf(
        "[RUNTIME-TRANSPORT-AUDIT-RESULT] "
        "MailboxChecks:%d | "
        "FmmuChecks:%d | "
        "SchemaMissing:%d | "
        "Errors:%d | "
        "Result:%s | "
        "HardwareWrite:NO | "
        "StartupAction:CONTINUE\n",
        mailboxChecks,
        fmmuChecks,
        schemaMissing,
        errorCount,
        pass ? "PASS" : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-TRANSPORT-AUDIT] END | "
        "Stage:6A | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",
        pass ? "PASS" : "FAIL");


    return pass;
}
