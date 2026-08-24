#include "EtherCatMaster.h"

#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 7B - Runtime Process Image Binding Cutover
//
// Runtime <ProcessImageBinding> becomes the application-side binding source.
//
// This file builds the SAME public/runtime objects that existing code already
// consumes:
//
// - m_IoList
// - m_AdList
// - m_ServoList
// - pointers into m_IoMap
//
// It does NOT change:
// - Process Image byte layout
// - m_IoMap storage
// - PLC AutoMap API
// - Motion Link API
// - EtherCAT SM/PDO/FMMU/DC configuration
// - 250 us cyclic PDO runtime
//
// Safety:
// - Every slave binding must be explicit before this path is selected.
// - Every offset/size is validated before a pointer is created.
// - Binding order must remain exact slave order.
// - Unknown Kind / ABI is a hard startup failure.
// ============================================================================

namespace
{
    bool IsRangeValid(
        int32_t offset,
        uint32_t bytes,
        size_t capacity)
    {
        if (bytes ==
            0U)
        {
            return
                offset ==
                -1;
        }


        if (offset <
            0)
        {
            return
                false;
        }


        const uint64_t end =
            (uint64_t)
            (uint32_t)
            offset +
            (uint64_t)
            bytes;


        return
            end <=
            (uint64_t)
            capacity;
    }


    bool StringEquals(
        const char* actual,
        const char* expected)
    {
        return
            actual !=
            nullptr &&
            expected !=
            nullptr &&
            strcmp(
                actual,
                expected) ==
            0;
    }
}


// ============================================================================
// BuildRuntimeProcessImageBindings
// ============================================================================

int EtherCatMaster::BuildRuntimeProcessImageBindings()
{
    if (m_pEni ==
        nullptr)
    {
        DEBUG_PRINT(
            "[RUNTIME-BINDING-CUTOVER] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            -1;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    // ========================================================================
    // Preflight the COMPLETE Runtime binding before mutating current lists.
    // ========================================================================

    uint32_t expectedSequentialOffset =
        0;


    int expectedGenericCount =
        0;


    int expectedAnalogCount =
        0;


    int expectedServoCount =
        0;


    int preflightErrors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[RUNTIME-BINDING-CUTOVER-PREFLIGHT] BEGIN | "
        "Stage:7B | Source:RUNTIME_XML | BuildWrite:NO\n"
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


        bool pass =
            binding.present;


        const uint32_t processOutputBytes =
            (
                slave.outputBitLength +
                7U
                ) /
            8U;


        const uint32_t processInputBytes =
            (
                slave.inputBitLength +
                7U
                ) /
            8U;


        const int32_t expectedOutputOffset =
            processOutputBytes >
            0U
            ? (int32_t)
            expectedSequentialOffset
            : -1;


        expectedSequentialOffset +=
            processOutputBytes;


        const int32_t expectedInputOffset =
            processInputBytes >
            0U
            ? (int32_t)
            expectedSequentialOffset
            : -1;


        expectedSequentialOffset +=
            processInputBytes;


        if (binding.outputBytes !=
            processOutputBytes ||
            binding.inputBytes !=
            processInputBytes ||
            binding.outputOffset !=
            expectedOutputOffset ||
            binding.inputOffset !=
            expectedInputOffset)
        {
            pass =
                false;
        }


        if (!IsRangeValid(
            binding.outputOffset,
            binding.outputBytes,
            sizeof(m_IoMap)) ||
            !IsRangeValid(
                binding.inputOffset,
                binding.inputBytes,
                sizeof(m_IoMap)))
        {
            pass =
                false;
        }


        if (StringEquals(
            binding.kind,
            "GenericIO"))
        {
            expectedGenericCount++;


            if (!StringEquals(
                binding.abi,
                "ByteArray") ||
                binding.elementBytes !=
                1U ||
                binding.channelCount !=
                0U)
            {
                pass =
                    false;
            }
        }
        else if (StringEquals(
            binding.kind,
            "AnalogInput"))
        {
            expectedAnalogCount++;


            if (!StringEquals(
                binding.abi,
                "Int16Channels") ||
                binding.outputBytes !=
                0U ||
                binding.outputOffset !=
                -1 ||
                binding.inputBytes ==
                0U ||
                binding.elementBytes !=
                2U ||
                (
                    binding.inputBytes %
                    2U
                    ) !=
                0U ||
                binding.channelCount !=
                (
                    binding.inputBytes /
                    2U
                    ))
            {
                pass =
                    false;
            }
        }
        else if (StringEquals(
            binding.kind,
            "Servo"))
        {
            expectedServoCount++;


            if (!StringEquals(
                binding.abi,
                "ServoPDO_9_23") ||
                binding.outputBytes !=
                sizeof(ServoOutput) ||
                binding.inputBytes !=
                sizeof(ServoInput) ||
                binding.outputOffset <
                0 ||
                binding.inputOffset <
                0)
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


        if (!pass)
        {
            preflightErrors++;
        }


        DEBUG_PRINT(
            "[RUNTIME-BINDING-CUTOVER-PREFLIGHT] "
            "S%d | Kind:%s Abi:%s | "
            "Runtime Out:%d/%u In:%d/%u | "
            "Expected Out:%d/%u In:%d/%u | "
            "Result:%s\n",

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

            expectedOutputOffset,

            (unsigned int)
            processOutputBytes,

            expectedInputOffset,

            (unsigned int)
            processInputBytes,

            pass
            ? "PASS"
            : "FAIL");
    }


    if (expectedSequentialOffset >
        sizeof(m_IoMap))
    {
        preflightErrors++;
    }


    const bool preflightPass =
        preflightErrors ==
        0;


    DEBUG_PRINT(
        "[RUNTIME-BINDING-CUTOVER-PREFLIGHT-RESULT] "
        "Slaves:%u | GenericIO:%d | Analog:%d | Servo:%d | "
        "ProcessImage:%u/%u B | Errors:%d | Result:%s | BuildWrite:NO\n",

        (unsigned int)
        slaves.size(),

        expectedGenericCount,
        expectedAnalogCount,
        expectedServoCount,

        (unsigned int)
        expectedSequentialOffset,

        (unsigned int)
        sizeof(m_IoMap),

        preflightErrors,

        preflightPass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[RUNTIME-BINDING-CUTOVER-PREFLIGHT] END | Result:%s\n"
        "============================================================\n\n",

        preflightPass
        ? "PASS"
        : "FAIL");


    if (!preflightPass)
    {
        return
            -1;
    }


    // ========================================================================
    // Cutover build.
    //
    // Preflight is complete, so list/map mutation starts only now.
    // ========================================================================

    m_IoList.clear();
    m_AdList.clear();
    m_ServoList.clear();


    memset(
        m_IoMap,
        0,
        sizeof(m_IoMap));


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[RUNTIME-BINDING-CUTOVER] BEGIN | "
        "Stage:7B | Source:RUNTIME_XML\n"
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


        // ====================================================================
        // GenericIO
        // ====================================================================

        if (StringEquals(
            binding.kind,
            "GenericIO"))
        {
            ENI_GenericIO mod;


            mod.slaveIndex =
                slaveIdx;


            mod.vendorId =
                slave.vendorId;


            mod.productCode =
                slave.productCode;


            mod.pOutputLoc =
                nullptr;


            mod.pInputLoc =
                nullptr;


            if (binding.outputBytes >
                0U)
            {
                mod.pOutputLoc =
                    static_cast<void*>(
                        &m_IoMap[
                            binding.outputOffset]);


                mod.outBuffer.resize(
                    binding.outputBytes,
                    0);
            }


            if (binding.inputBytes >
                0U)
            {
                mod.pInputLoc =
                    static_cast<void*>(
                        &m_IoMap[
                            binding.inputOffset]);


                mod.inBuffer.resize(
                    binding.inputBytes,
                    0);
            }


            m_IoList.push_back(
                mod);


            DEBUG_PRINT(
                "[RUNTIME-BINDING-CUTOVER] "
                "S%d | Kind:GenericIO | ListIndex:%u | "
                "Out:%d/%u In:%d/%u | Result:PASS\n",

                slaveIdx,

                (unsigned int)
                (
                    m_IoList.size() -
                    1U
                    ),

                binding.outputOffset,

                (unsigned int)
                binding.outputBytes,

                binding.inputOffset,

                (unsigned int)
                binding.inputBytes);
        }


        // ====================================================================
        // AnalogInput
        // ====================================================================

        else if (StringEquals(
            binding.kind,
            "AnalogInput"))
        {
            ENI_AnalogModule ad;


            ad.slaveIndex =
                slaveIdx;


            ad.vendorId =
                slave.vendorId;


            ad.productCode =
                slave.productCode;


            ad.pInputLoc =
                nullptr;


            if (binding.inputBytes >
                0U)
            {
                ad.pInputLoc =
                    static_cast<void*>(
                        &m_IoMap[
                            binding.inputOffset]);


                ad.channelValues.resize(
                    binding.channelCount,
                    0);
            }


            m_AdList.push_back(
                ad);


            DEBUG_PRINT(
                "[RUNTIME-BINDING-CUTOVER] "
                "S%d | Kind:AnalogInput | ListIndex:%u | "
                "In:%d/%u | Element:%u Channels:%u | Result:PASS\n",

                slaveIdx,

                (unsigned int)
                (
                    m_AdList.size() -
                    1U
                    ),

                binding.inputOffset,

                (unsigned int)
                binding.inputBytes,

                (unsigned int)
                binding.elementBytes,

                (unsigned int)
                binding.channelCount);
        }


        // ====================================================================
        // Servo
        // ====================================================================

        else if (StringEquals(
            binding.kind,
            "Servo"))
        {
            ENI_ServoDrive axis;


            axis.slaveIndex =
                slaveIdx;


            axis.vendorId =
                slave.vendorId;


            axis.productCode =
                slave.productCode;


            axis.pOutput =
                reinterpret_cast<ServoOutput*>(
                    &m_IoMap[
                        binding.outputOffset]);


            axis.pInput =
                reinterpret_cast<ServoInput*>(
                    &m_IoMap[
                        binding.inputOffset]);


            m_ServoList.push_back(
                axis);


            DEBUG_PRINT(
                "[RUNTIME-BINDING-CUTOVER] "
                "S%d | Kind:Servo | ListIndex:%u | "
                "Out:%d/%u In:%d/%u | Abi:%s | Result:PASS\n",

                slaveIdx,

                (unsigned int)
                (
                    m_ServoList.size() -
                    1U
                    ),

                binding.outputOffset,

                (unsigned int)
                binding.outputBytes,

                binding.inputOffset,

                (unsigned int)
                binding.inputBytes,

                binding.abi);
        }


        else
        {
            // Should be unreachable after complete preflight.
            DEBUG_PRINT(
                "[RUNTIME-BINDING-CUTOVER] "
                "S%d | Unknown Kind:%s | Result:FAIL\n",

                slaveIdx,

                binding.kind);


            m_IoList.clear();
            m_AdList.clear();
            m_ServoList.clear();


            memset(
                m_IoMap,
                0,
                sizeof(m_IoMap));


            m_IoMapSize =
                0;


            return
                -1;
        }
    }


    m_IoMapSize =
        (int)
        expectedSequentialOffset;


    if ((int)m_IoList.size() !=
        expectedGenericCount ||
        (int)m_AdList.size() !=
        expectedAnalogCount ||
        (int)m_ServoList.size() !=
        expectedServoCount)
    {
        DEBUG_PRINT(
            "[RUNTIME-BINDING-CUTOVER] "
            "Post-build list count mismatch | Result:FAIL\n");


        m_IoList.clear();
        m_AdList.clear();
        m_ServoList.clear();


        memset(
            m_IoMap,
            0,
            sizeof(m_IoMap));


        m_IoMapSize =
            0;


        return
            -1;
    }


    // Preserve the existing consumer API exactly.
    m_Plc.SetIoLists(
        &m_IoList,
        &m_AdList);


    m_Plc.AutoMapIO();


    DEBUG_PRINT(
        "[RUNTIME-BINDING-CUTOVER-RESULT] "
        "Slaves:%u | GenericIO:%u | Analog:%u | Servo:%u | "
        "ProcessImage:%d B | Result:PASS\n",

        (unsigned int)
        slaves.size(),

        (unsigned int)
        m_IoList.size(),

        (unsigned int)
        m_AdList.size(),

        (unsigned int)
        m_ServoList.size(),

        m_IoMapSize);


    DEBUG_PRINT(
        "============================================================\n"
        "[RUNTIME-BINDING-CUTOVER] END | Result:PASS\n"
        "============================================================\n\n");


    DEBUG_PRINT(
        ">>> Mapping Done. Total Map Size: %d Bytes | Source:RUNTIME_XML\n",

        m_IoMapSize);


    return
        0;
}
