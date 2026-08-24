#include "EtherCatMaster.h"

#include <cstdint>
#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 11A - Composite Multi-Binding Shadow Audit
//
// Current active application binding:
//     <ProcessImageBinding>
//     Stage7B
//
// New future-capable shadow schema:
//     <ApplicationBindings>
//     0..N functional regions in one Slave.
//
// Stage 11A proves the schema/parser/audit pipeline while current devices are
// still represented as one "SimpleProjection" binding.
//
// NO hardware access.
// NO m_IoList / m_AdList / m_ServoList mutation.
// NO Stage9B / Stage8B gate change.
// ============================================================================

namespace
{
    bool TextEquals(
        const char* a,
        const char* b)
    {
        return
            a !=
            nullptr &&
            b !=
            nullptr &&
            strcmp(
                a,
                b) ==
            0;
    }


    bool IsValidBitRange(
        int32_t bitOffset,
        uint32_t bitLength,
        uint32_t processImageBits)
    {
        if (bitLength ==
            0U)
        {
            return
                bitOffset ==
                -1;
        }


        if (bitOffset <
            0)
        {
            return
                false;
        }


        const uint64_t end =
            static_cast<uint64_t>(
                static_cast<uint32_t>(
                    bitOffset)) +
            static_cast<uint64_t>(
                bitLength);


        return
            end <=
            static_cast<uint64_t>(
                processImageBits);
    }


    bool RangesOverlap(
        int32_t leftOffset,
        uint32_t leftLength,
        int32_t rightOffset,
        uint32_t rightLength)
    {
        if (leftLength ==
            0U ||
            rightLength ==
            0U)
        {
            return
                false;
        }


        if (leftOffset <
            0 ||
            rightOffset <
            0)
        {
            return
                true;
        }


        const uint64_t leftBegin =
            static_cast<uint32_t>(
                leftOffset);


        const uint64_t leftEnd =
            leftBegin +
            leftLength;


        const uint64_t rightBegin =
            static_cast<uint32_t>(
                rightOffset);


        const uint64_t rightEnd =
            rightBegin +
            rightLength;


        return
            leftBegin <
            rightEnd&&
            rightBegin <
            leftEnd;
    }


    const char* ExpectedFunctionKind(
        const EtherCatRuntimeProcessImageBindingConfig& simple)
    {
        if (TextEquals(simple.kind, "AnalogInput"))
        {
            return "AnalogInput";
        }

        if (TextEquals(simple.kind, "Servo"))
        {
            return "ServoDrive";
        }

        if (TextEquals(simple.kind, "GenericIO"))
        {
            if (simple.inputBytes > 0U &&
                simple.outputBytes == 0U)
            {
                return "DigitalInput";
            }

            if (simple.outputBytes > 0U &&
                simple.inputBytes == 0U)
            {
                return "DigitalOutput";
            }

            if (simple.outputBytes > 0U &&
                simple.inputBytes > 0U)
            {
                return "DigitalIO";
            }
        }

        return "GenericData";
    }


    const char* ExpectedInterface(
        const EtherCatRuntimeProcessImageBindingConfig& simple)
    {
        if (TextEquals(simple.kind, "AnalogInput"))
        {
            return "Analog";
        }

        if (TextEquals(simple.kind, "Servo"))
        {
            return "CiA402";
        }

        if (TextEquals(simple.kind, "GenericIO") &&
            (simple.inputBytes > 0U ||
                simple.outputBytes > 0U))
        {
            return "DiscreteIO";
        }

        return "RawProcessData";
    }


    const char* ExpectedDataType(
        const EtherCatRuntimeProcessImageBindingConfig& simple)
    {
        if (TextEquals(simple.kind, "AnalogInput"))
        {
            return "Int16";
        }

        if (TextEquals(simple.kind, "Servo"))
        {
            return "Composite";
        }

        if (TextEquals(simple.kind, "GenericIO") &&
            (simple.inputBytes > 0U ||
                simple.outputBytes > 0U))
        {
            return "BitField";
        }

        return "UInt8";
    }


    uint16_t ExpectedElementBits(
        const EtherCatRuntimeProcessImageBindingConfig& simple)
    {
        if (TextEquals(simple.kind, "AnalogInput"))
        {
            return 16U;
        }

        if (TextEquals(simple.kind, "GenericIO") &&
            (simple.inputBytes > 0U ||
                simple.outputBytes > 0U))
        {
            return 1U;
        }

        return 0U;
    }


    uint16_t ExpectedChannelCount(
        const EtherCatRuntimeProcessImageBindingConfig& simple)
    {
        if (TextEquals(simple.kind, "AnalogInput"))
        {
            return simple.channelCount;
        }

        if (TextEquals(simple.kind, "GenericIO"))
        {
            return static_cast<uint16_t>(
                (simple.inputBytes + simple.outputBytes) * 8U);
        }

        return 0U;
    }
}


// ============================================================================
// AuditRuntimeCompositeApplicationBindings
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeApplicationBindings()
{
    if (m_pEni ==
        nullptr)
    {
        DEBUG_PRINT(
            "[COMPOSITE-BINDING-AUDIT] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    const uint32_t processImageBits =
        m_IoMapSize >
        0
        ? static_cast<uint32_t>(
            m_IoMapSize) *
        8U
        : 0U;


    int schemaSlaves =
        0;


    int simpleProjectionSlaves =
        0;


    int bindingCount =
        0;


    int rangeChecks =
        0;


    int coverageChecks =
        0;


    int overlapChecks =
        0;


    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-BINDING-AUDIT] BEGIN | "
        "Stage:11A | Mode:SHADOW_READ_ONLY | "
        "ActiveBinding:SIMPLE_STAGE7B | "
        "HardwareRead:NO | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIndex = 0;
        slaveIndex <
        static_cast<int>(
            slaves.size());
        slaveIndex++)
    {
        const EtherCatSlave& slave =
            slaves[
                static_cast<size_t>(
                    slaveIndex)];


        const auto& simple =
            slave.runtimeProcessImageBinding;


        bool pass =
            true;


        if (slave.runtimeApplicationBindingsSchemaPresent)
        {
            schemaSlaves++;
        }
        else
        {
            pass =
                false;
        }


        const bool simpleProjection =
            TextEquals(
                slave.runtimeApplicationBindingsMode,
                "SimpleProjection");


        if (simpleProjection)
        {
            simpleProjectionSlaves++;
        }
        else
        {
            pass =
                false;
        }


        bindingCount +=
            static_cast<int>(
                slave.runtimeApplicationBindings.size());


        // ====================================================================
        // Generic range validation.
        // ====================================================================

        uint32_t outputCoveredBits =
            0;


        uint32_t inputCoveredBits =
            0;


        for (size_t bindingIndex = 0;
            bindingIndex <
            slave.runtimeApplicationBindings.size();
            bindingIndex++)
        {
            const auto& binding =
                slave.runtimeApplicationBindings[
                    bindingIndex];


            rangeChecks +=
                2;


            if (!IsValidBitRange(
                binding.outputBitOffset,
                binding.outputBitLength,
                processImageBits) ||
                !IsValidBitRange(
                    binding.inputBitOffset,
                    binding.inputBitLength,
                    processImageBits))
            {
                pass =
                    false;
            }


            outputCoveredBits +=
                binding.outputBitLength;


            inputCoveredBits +=
                binding.inputBitLength;


            for (size_t otherIndex =
                bindingIndex +
                1U;
                otherIndex <
                slave.runtimeApplicationBindings.size();
                otherIndex++)
            {
                const auto& other =
                    slave.runtimeApplicationBindings[
                        otherIndex];


                overlapChecks +=
                    2;


                if (RangesOverlap(
                    binding.outputBitOffset,
                    binding.outputBitLength,
                    other.outputBitOffset,
                    other.outputBitLength) ||
                    RangesOverlap(
                        binding.inputBitOffset,
                        binding.inputBitLength,
                        other.inputBitOffset,
                        other.inputBitLength))
                {
                    pass =
                        false;
                }
            }
        }


        coverageChecks +=
            2;


        if (outputCoveredBits !=
            slave.outputBitLength ||
            inputCoveredBits !=
            slave.inputBitLength)
        {
            pass =
                false;
        }


        // ====================================================================
        // Stage11A current-machine projection equivalence.
        //
        // The current simple device must be exactly one composite binding
        // matching the Stage7B ProcessImageBinding envelope.
        // ====================================================================

        bool projectionPass =
            simple.present &&
            slave.runtimeApplicationBindings.size() ==
            1U;


        if (projectionPass)
        {
            const auto& binding =
                slave.runtimeApplicationBindings[0];


            const int32_t expectedOutputBitOffset =
                simple.outputBytes >
                0U
                ? simple.outputOffset *
                8
                : -1;


            const uint32_t expectedOutputBitLength =
                simple.outputBytes *
                8U;


            const int32_t expectedInputBitOffset =
                simple.inputBytes >
                0U
                ? simple.inputOffset *
                8
                : -1;


            const uint32_t expectedInputBitLength =
                simple.inputBytes *
                8U;


            projectionPass =
                TextEquals(
                    binding.id,
                    "Primary") &&
                TextEquals(
                    binding.kind,
                    ExpectedFunctionKind(
                        simple)) &&
                TextEquals(
                    binding.legacyKind,
                    simple.kind) &&
                TextEquals(
                    binding.abi,
                    simple.abi) &&
                TextEquals(
                    binding.interfaceType,
                    ExpectedInterface(
                        simple)) &&
                TextEquals(
                    binding.dataType,
                    ExpectedDataType(
                        simple)) &&
                TextEquals(
                    binding.sampleMode,
                    "Cyclic") &&
                TextEquals(
                    binding.unit,
                    "Raw") &&
                binding.outputBitOffset ==
                expectedOutputBitOffset &&
                binding.outputBitLength ==
                expectedOutputBitLength &&
                binding.inputBitOffset ==
                expectedInputBitOffset &&
                binding.inputBitLength ==
                expectedInputBitLength &&
                binding.elementBits ==
                ExpectedElementBits(
                    simple) &&
                binding.channelCount ==
                ExpectedChannelCount(
                    simple);
        }


        if (!projectionPass)
        {
            pass =
                false;
        }


        if (!pass)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-BINDING-SLAVE] "
            "S%d | Schema:%s Mode:%s | "
            "Bindings:%u | OutCoverage:%u/%u bits | "
            "InCoverage:%u/%u bits | "
            "SimpleProjection:%s | Result:%s\n",

            slaveIndex,

            slave.runtimeApplicationBindingsSchemaPresent
            ? "YES"
            : "NO",

            slave.runtimeApplicationBindingsMode[0] != '\0'
            ? slave.runtimeApplicationBindingsMode
            : "N/A",

            (unsigned int)
            slave.runtimeApplicationBindings.size(),

            (unsigned int)
            outputCoveredBits,

            (unsigned int)
            slave.outputBitLength,

            (unsigned int)
            inputCoveredBits,

            (unsigned int)
            slave.inputBitLength,

            projectionPass
            ? "MATCH"
            : "DIFF",

            pass
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-BINDING-AUDIT-RESULT] "
        "Slaves:%u | "
        "SchemaSlaves:%d/%u | "
        "SimpleProjection:%d/%u | "
        "Bindings:%d | "
        "RangeChecks:%d | CoverageChecks:%d | OverlapChecks:%d | "
        "ProcessImage:%u bits | "
        "Errors:%d | Result:%s | "
        "ActiveBinding:SIMPLE_STAGE7B | "
        "HardwareRead:NO | HardwareWrite:NO | "
        "StartupAction:CONTINUE\n",

        (unsigned int)
        slaves.size(),

        schemaSlaves,

        (unsigned int)
        slaves.size(),

        simpleProjectionSlaves,

        (unsigned int)
        slaves.size(),

        bindingCount,

        rangeChecks,
        coverageChecks,
        overlapChecks,

        (unsigned int)
        processImageBits,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-BINDING-AUDIT] END | "
        "Stage:11A | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
