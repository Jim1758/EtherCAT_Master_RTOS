#include "EtherCatMaster.h"

#include <rtapi.h>

#include <string.h>

#include "GlobalConfig.h"


// ============================================================================
// Stage 7A - Runtime Process Image Binding Shadow Audit
//
// This audit compares the Runtime XML application binding with the CURRENT
// BuildIoMap() result.
//
// IMPORTANT:
//
// - The currently built application map is read-only during this audit.
// - No list is rebuilt here.
// - No pointer is changed here.
// - No EtherCAT hardware register is written here.
// - No Motion / PLC binding is changed here.
// - Startup continues even when the audit reports a difference.
//
// Stage 7 cutover is allowed only after this audit is proven on hardware.
// ============================================================================

namespace
{
    bool PointerOffsetFromIoMap(
        const char* ioMap,
        int ioMapSize,
        const void* pointerValue,
        int32_t& offsetOut)
    {
        offsetOut =
            -1;


        if (pointerValue == nullptr)
        {
            return
                true;
        }


        if (ioMap == nullptr ||
            ioMapSize < 0)
        {
            return
                false;
        }


        // EtherCatMaster::m_IoMap is declared as char[4096].
        //
        // Keep this helper byte-oriented while matching the actual
        // project type exactly. Pointer subtraction remains a byte offset
        // because sizeof(char) is always 1.
        const char* pointer =
            reinterpret_cast<const char*>(
                pointerValue);


        const char* end =
            ioMap +
            ioMapSize;


        if (pointer < ioMap ||
            pointer >= end)
        {
            return
                false;
        }


        offsetOut =
            static_cast<int32_t>(
                pointer -
                ioMap);


        return
            true;
    }


    int CountGenericIoBySlave(
        const std::vector<ENI_GenericIO>& list,
        int slaveIdx)
    {
        int count =
            0;


        for (const auto& item :
            list)
        {
            if (item.slaveIndex ==
                slaveIdx)
            {
                count++;
            }
        }


        return
            count;
    }


    int CountAnalogBySlave(
        const std::vector<ENI_AnalogModule>& list,
        int slaveIdx)
    {
        int count =
            0;


        for (const auto& item :
            list)
        {
            if (item.slaveIndex ==
                slaveIdx)
            {
                count++;
            }
        }


        return
            count;
    }


    int CountServoBySlave(
        const std::vector<ENI_ServoDrive>& list,
        int slaveIdx)
    {
        int count =
            0;


        for (const auto& item :
            list)
        {
            if (item.slaveIndex ==
                slaveIdx)
            {
                count++;
            }
        }


        return
            count;
    }


    bool BindingKindEquals(
        const EtherCatRuntimeProcessImageBindingConfig& binding,
        const char* expected)
    {
        return
            binding.kind[0] != '\0' &&
            strcmp(
                binding.kind,
                expected) ==
            0;
    }


    bool AbiEquals(
        const EtherCatRuntimeProcessImageBindingConfig& binding,
        const char* expected)
    {
        return
            binding.abi[0] != '\0' &&
            strcmp(
                binding.abi,
                expected) ==
            0;
    }
}


// ============================================================================
// AuditRuntimeProcessImageBindings
// ============================================================================

bool EtherCatMaster::AuditRuntimeProcessImageBindings()
{
    if (m_pEni == nullptr)
    {
        RtPrintf(
            "[RUNTIME-BINDING-AUDIT] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int expectedGenericIndex =
        0;


    int expectedAnalogIndex =
        0;


    int expectedServoIndex =
        0;


    int bindingChecks =
        0;


    int pointerChecks =
        0;


    int errors =
        0;


    uint32_t maxRuntimeEnd =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-BINDING-AUDIT] BEGIN | "
        "Stage:7A | Mode:SHADOW_READ_ONLY | "
        "AuditWrite:NO | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIdx = 0;
        slaveIdx <
        (int)slaves.size();
        slaveIdx++)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)slaveIdx];


        const EtherCatRuntimeProcessImageBindingConfig& binding =
            slave.runtimeProcessImageBinding;


        bindingChecks++;


        if (!binding.present)
        {
            errors++;


            RtPrintf(
                "[RUNTIME-BINDING] "
                "S%d | Runtime ProcessImageBinding missing | Result:FAIL\n",

                slaveIdx);


            continue;
        }


        const uint32_t actualOutputBytes =
            (
                slave.outputBitLength +
                7U
                ) /
            8U;


        const uint32_t actualInputBytes =
            (
                slave.inputBitLength +
                7U
                ) /
            8U;


        bool basePass =
            true;


        if (binding.outputBytes !=
            actualOutputBytes ||
            binding.inputBytes !=
            actualInputBytes)
        {
            basePass =
                false;
        }


        if (binding.outputBytes ==
            0U)
        {
            if (binding.outputOffset !=
                -1)
            {
                basePass =
                    false;
            }
        }
        else
        {
            if (binding.outputOffset <
                0)
            {
                basePass =
                    false;
            }
            else
            {
                const uint32_t end =
                    (uint32_t)
                    binding.outputOffset +
                    binding.outputBytes;


                if (end >
                    maxRuntimeEnd)
                {
                    maxRuntimeEnd =
                        end;
                }
            }
        }


        if (binding.inputBytes ==
            0U)
        {
            if (binding.inputOffset !=
                -1)
            {
                basePass =
                    false;
            }
        }
        else
        {
            if (binding.inputOffset <
                0)
            {
                basePass =
                    false;
            }
            else
            {
                const uint32_t end =
                    (uint32_t)
                    binding.inputOffset +
                    binding.inputBytes;


                if (end >
                    maxRuntimeEnd)
                {
                    maxRuntimeEnd =
                        end;
                }
            }
        }


        const int genericOccurrences =
            CountGenericIoBySlave(
                m_IoList,
                slaveIdx);


        const int analogOccurrences =
            CountAnalogBySlave(
                m_AdList,
                slaveIdx);


        const int servoOccurrences =
            CountServoBySlave(
                m_ServoList,
                slaveIdx);


        const int totalOccurrences =
            genericOccurrences +
            analogOccurrences +
            servoOccurrences;


        if (totalOccurrences !=
            1)
        {
            basePass =
                false;
        }


        bool classPass =
            false;


        bool pointerPass =
            true;


        int32_t actualOutputOffset =
            -1;


        int32_t actualInputOffset =
            -1;


        // ====================================================================
        // GenericIO
        // ====================================================================

        if (BindingKindEquals(
            binding,
            "GenericIO"))
        {
            if (expectedGenericIndex <
                (int)m_IoList.size())
            {
                const ENI_GenericIO& item =
                    m_IoList[
                        (size_t)
                            expectedGenericIndex];


                classPass =
                    item.slaveIndex ==
                    slaveIdx &&
                    genericOccurrences ==
                    1 &&
                    analogOccurrences ==
                    0 &&
                    servoOccurrences ==
                    0 &&
                    item.vendorId ==
                    slave.vendorId &&
                    item.productCode ==
                    slave.productCode &&
                    AbiEquals(
                        binding,
                        "ByteArray");


                if (!PointerOffsetFromIoMap(
                    m_IoMap,
                    m_IoMapSize,
                    item.pOutputLoc,
                    actualOutputOffset))
                {
                    pointerPass =
                        false;
                }


                if (!PointerOffsetFromIoMap(
                    m_IoMap,
                    m_IoMapSize,
                    item.pInputLoc,
                    actualInputOffset))
                {
                    pointerPass =
                        false;
                }


                if (binding.outputBytes >
                    0U)
                {
                    pointerChecks++;


                    if (actualOutputOffset !=
                        binding.outputOffset ||
                        item.outBuffer.size() !=
                        binding.outputBytes)
                    {
                        pointerPass =
                            false;
                    }
                }
                else
                {
                    if (item.pOutputLoc !=
                        nullptr ||
                        !item.outBuffer.empty())
                    {
                        pointerPass =
                            false;
                    }
                }


                if (binding.inputBytes >
                    0U)
                {
                    pointerChecks++;


                    if (actualInputOffset !=
                        binding.inputOffset ||
                        item.inBuffer.size() !=
                        binding.inputBytes)
                    {
                        pointerPass =
                            false;
                    }
                }
                else
                {
                    if (item.pInputLoc !=
                        nullptr ||
                        !item.inBuffer.empty())
                    {
                        pointerPass =
                            false;
                    }
                }
            }


            expectedGenericIndex++;
        }


        // ====================================================================
        // AnalogInput
        // ====================================================================

        else if (BindingKindEquals(
            binding,
            "AnalogInput"))
        {
            if (expectedAnalogIndex <
                (int)m_AdList.size())
            {
                const ENI_AnalogModule& item =
                    m_AdList[
                        (size_t)
                            expectedAnalogIndex];


                classPass =
                    item.slaveIndex ==
                    slaveIdx &&
                    genericOccurrences ==
                    0 &&
                    analogOccurrences ==
                    1 &&
                    servoOccurrences ==
                    0 &&
                    item.vendorId ==
                    slave.vendorId &&
                    item.productCode ==
                    slave.productCode &&
                    AbiEquals(
                        binding,
                        "Int16Channels");


                if (!PointerOffsetFromIoMap(
                    m_IoMap,
                    m_IoMapSize,
                    item.pInputLoc,
                    actualInputOffset))
                {
                    pointerPass =
                        false;
                }


                if (binding.outputBytes !=
                    0U ||
                    binding.outputOffset !=
                    -1 ||
                    binding.elementBytes !=
                    2U ||
                    binding.inputBytes ==
                    0U ||
                    (
                        binding.inputBytes %
                        binding.elementBytes
                        ) !=
                    0U ||
                    binding.channelCount !=
                    (
                        binding.inputBytes /
                        binding.elementBytes
                        ))
                {
                    pointerPass =
                        false;
                }


                pointerChecks++;


                if (actualInputOffset !=
                    binding.inputOffset ||
                    item.channelValues.size() !=
                    binding.channelCount ||
                    (
                        item.channelValues.size() *
                        sizeof(int16_t)
                        ) !=
                    binding.inputBytes)
                {
                    pointerPass =
                        false;
                }
            }


            expectedAnalogIndex++;
        }


        // ====================================================================
        // Servo
        // ====================================================================

        else if (BindingKindEquals(
            binding,
            "Servo"))
        {
            if (expectedServoIndex <
                (int)m_ServoList.size())
            {
                const ENI_ServoDrive& item =
                    m_ServoList[
                        (size_t)
                            expectedServoIndex];


                classPass =
                    item.slaveIndex ==
                    slaveIdx &&
                    genericOccurrences ==
                    0 &&
                    analogOccurrences ==
                    0 &&
                    servoOccurrences ==
                    1 &&
                    item.vendorId ==
                    slave.vendorId &&
                    item.productCode ==
                    slave.productCode &&
                    AbiEquals(
                        binding,
                        "ServoPDO_9_23") &&
                    binding.outputBytes ==
                    sizeof(ServoOutput) &&
                    binding.inputBytes ==
                    sizeof(ServoInput);


                if (!PointerOffsetFromIoMap(
                    m_IoMap,
                    m_IoMapSize,
                    item.pOutput,
                    actualOutputOffset))
                {
                    pointerPass =
                        false;
                }


                if (!PointerOffsetFromIoMap(
                    m_IoMap,
                    m_IoMapSize,
                    item.pInput,
                    actualInputOffset))
                {
                    pointerPass =
                        false;
                }


                pointerChecks +=
                    2;


                if (item.pOutput ==
                    nullptr ||
                    item.pInput ==
                    nullptr ||
                    actualOutputOffset !=
                    binding.outputOffset ||
                    actualInputOffset !=
                    binding.inputOffset)
                {
                    pointerPass =
                        false;
                }
            }


            expectedServoIndex++;
        }


        // ====================================================================
        // Unknown
        // ====================================================================

        else
        {
            classPass =
                false;


            pointerPass =
                false;
        }


        const bool pass =
            basePass &&
            classPass &&
            pointerPass;


        if (!pass)
        {
            errors++;
        }


        RtPrintf(
            "[RUNTIME-BINDING] "
            "S%d | Kind:%s Abi:%s | "
            "Runtime Out:%d/%u In:%d/%u | "
            "Actual Out:%d In:%d | "
            "Occurrences IO:%d AD:%d Servo:%d | "
            "Class:%s Pointer:%s Base:%s | Result:%s\n",

            slaveIdx,

            binding.kind[0] != '\0'
            ? binding.kind
            : "N/A",

            binding.abi[0] != '\0'
            ? binding.abi
            : "N/A",

            binding.outputOffset,

            (unsigned int)
            binding.outputBytes,

            binding.inputOffset,

            (unsigned int)
            binding.inputBytes,

            actualOutputOffset,
            actualInputOffset,

            genericOccurrences,
            analogOccurrences,
            servoOccurrences,

            classPass
            ? "PASS"
            : "FAIL",

            pointerPass
            ? "PASS"
            : "FAIL",

            basePass
            ? "PASS"
            : "FAIL",

            pass
            ? "PASS"
            : "FAIL");
    }


    // ========================================================================
    // Global List Order / Count / Process Image consistency.
    // ========================================================================

    if (expectedGenericIndex !=
        (int)m_IoList.size())
    {
        errors++;


        RtPrintf(
            "[RUNTIME-BINDING-LIST] "
            "GenericIO Count Runtime:%d Actual:%u | Result:FAIL\n",

            expectedGenericIndex,

            (unsigned int)
            m_IoList.size());
    }


    if (expectedAnalogIndex !=
        (int)m_AdList.size())
    {
        errors++;


        RtPrintf(
            "[RUNTIME-BINDING-LIST] "
            "Analog Count Runtime:%d Actual:%u | Result:FAIL\n",

            expectedAnalogIndex,

            (unsigned int)
            m_AdList.size());
    }


    if (expectedServoIndex !=
        (int)m_ServoList.size())
    {
        errors++;


        RtPrintf(
            "[RUNTIME-BINDING-LIST] "
            "Servo Count Runtime:%d Actual:%u | Result:FAIL\n",

            expectedServoIndex,

            (unsigned int)
            m_ServoList.size());
    }


    if (maxRuntimeEnd !=
        (uint32_t)m_IoMapSize)
    {
        errors++;


        RtPrintf(
            "[RUNTIME-BINDING-LIST] "
            "ProcessImageEnd Runtime:%u Actual:%d | Result:FAIL\n",

            (unsigned int)
            maxRuntimeEnd,

            m_IoMapSize);
    }


    const bool pass =
        errors ==
        0;


    RtPrintf(
        "[RUNTIME-BINDING-AUDIT-RESULT] "
        "Slaves:%u | "
        "GenericIO:%d/%u | "
        "Analog:%d/%u | "
        "Servo:%d/%u | "
        "PointerChecks:%d | "
        "ProcessImage:%u/%d B | "
        "Errors:%d | "
        "Result:%s | "
        "AuditWrite:NO | "
        "HardwareWrite:NO | "
        "StartupAction:CONTINUE\n",

        (unsigned int)
        slaves.size(),

        expectedGenericIndex,

        (unsigned int)
        m_IoList.size(),

        expectedAnalogIndex,

        (unsigned int)
        m_AdList.size(),

        expectedServoIndex,

        (unsigned int)
        m_ServoList.size(),

        pointerChecks,

        (unsigned int)
        maxRuntimeEnd,

        m_IoMapSize,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-BINDING-AUDIT] END | "
        "Stage:7A | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
