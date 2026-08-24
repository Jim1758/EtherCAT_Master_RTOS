#include "EtherCatMaster.h"

#include <cstdint>
#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 11C.1 - Persisted Composite Runtime Shadow Audit
//
// Active Runtime remains:
//
//     Stage7B ProcessImageBinding
//     +
//     Stage11A SimpleProjection ApplicationBindings
//
// New shadow:
//
//     Project v1.3 persisted ApplicationBindings
//       Relative Bit Offset
//             +
//       Slave Process Image Base
//             =
//       Absolute Runtime Process Image Bit Offset
//
// This stage verifies the translation only.
//
// NO hardware read.
// NO hardware write.
// NO BuildIoMap mutation.
// NO PLC/Motion mutation.
// NO gate cutover.
// ============================================================================

namespace
{
    bool TextEquals(
        const char* left,
        const char* right)
    {
        return
            left !=
            nullptr &&
            right !=
            nullptr &&
            strcmp(
                left,
                right) ==
            0;
    }


    bool IsSimpleProjectionMode(
        const char* mode)
    {
        return
            TextEquals(
                mode,
                "SimpleProjection");
    }


    bool IsCompositeMode(
        const char* mode)
    {
        return
            TextEquals(
                mode,
                "Composite");
    }


    bool IsAbsoluteRangeValid(
        int32_t absoluteOffset,
        uint32_t length,
        uint32_t processImageBits)
    {
        if (length ==
            0U)
        {
            return
                absoluteOffset ==
                -1;
        }


        if (absoluteOffset <
            0)
        {
            return
                false;
        }


        const uint64_t end =
            static_cast<uint64_t>(
                static_cast<uint32_t>(
                    absoluteOffset)) +
            static_cast<uint64_t>(
                length);


        return
            end <=
            static_cast<uint64_t>(
                processImageBits);
    }


    bool IsRelativeRangeValid(
        int32_t relativeOffset,
        uint32_t length,
        uint32_t slaveLength)
    {
        if (length ==
            0U)
        {
            return
                relativeOffset ==
                -1;
        }


        if (relativeOffset <
            0)
        {
            return
                false;
        }


        const uint64_t end =
            static_cast<uint64_t>(
                static_cast<uint32_t>(
                    relativeOffset)) +
            static_cast<uint64_t>(
                length);


        return
            end <=
            static_cast<uint64_t>(
                slaveLength);
    }


    bool RelativeToAbsoluteMatches(
        int32_t baseBit,
        int32_t relativeOffset,
        int32_t absoluteOffset,
        uint32_t length)
    {
        if (length ==
            0U)
        {
            return
                baseBit ==
                -1
                ? relativeOffset ==
                -1 &&
                absoluteOffset ==
                -1
                : relativeOffset ==
                -1 &&
                absoluteOffset ==
                -1;
        }


        if (baseBit <
            0 ||
            relativeOffset <
            0)
        {
            return
                false;
        }


        const int64_t expected =
            static_cast<int64_t>(
                baseBit) +
            static_cast<int64_t>(
                relativeOffset);


        return
            expected ==
            static_cast<int64_t>(
                absoluteOffset);
    }


    bool RangeInsideEnvelope(
        int32_t envelopeOffset,
        uint32_t envelopeLength,
        int32_t bindingOffset,
        uint32_t bindingLength)
    {
        if (bindingLength ==
            0U)
        {
            return
                bindingOffset ==
                -1;
        }


        if (envelopeLength ==
            0U ||
            envelopeOffset <
            0 ||
            bindingOffset <
            0)
        {
            return
                false;
        }


        const uint64_t envelopeBegin =
            static_cast<uint32_t>(
                envelopeOffset);


        const uint64_t envelopeEnd =
            envelopeBegin +
            envelopeLength;


        const uint64_t bindingBegin =
            static_cast<uint32_t>(
                bindingOffset);


        const uint64_t bindingEnd =
            bindingBegin +
            bindingLength;


        return
            bindingBegin >=
            envelopeBegin &&
            bindingEnd <=
            envelopeEnd;
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


    bool SimpleProjectionEquivalent(
        const EtherCatRuntimeApplicationBindingConfig& active,
        const EtherCatRuntimeApplicationBindingConfig& shadow)
    {
        return
            TextEquals(
                active.id,
                shadow.id) &&
            TextEquals(
                active.kind,
                shadow.kind) &&
            TextEquals(
                active.legacyKind,
                shadow.legacyKind) &&
            TextEquals(
                active.abi,
                shadow.abi) &&
            TextEquals(
                active.interfaceType,
                shadow.interfaceType) &&
            TextEquals(
                active.dataType,
                shadow.dataType) &&
            TextEquals(
                active.sampleMode,
                shadow.sampleMode) &&
            TextEquals(
                active.unit,
                shadow.unit) &&
            TextEquals(
                active.axisRef,
                shadow.axisRef) &&
            active.outputBitOffset ==
            shadow.outputBitOffset &&
            active.outputBitLength ==
            shadow.outputBitLength &&
            active.inputBitOffset ==
            shadow.inputBitOffset &&
            active.inputBitLength ==
            shadow.inputBitLength &&
            active.elementBits ==
            shadow.elementBits &&
            active.channelCount ==
            shadow.channelCount;
    }
}


// ============================================================================
// AuditRuntimeCompositeProjectShadowBindings
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeProjectShadowBindings()
{
    if (m_pEni ==
        nullptr)
    {
        DEBUG_PRINT(
            "[COMPOSITE-RUNTIME-SHADOW-AUDIT] "
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


    int simpleSlaves =
        0;


    int compositeSlaves =
        0;


    int bindingCount =
        0;


    int rangeChecks =
        0;


    int relativeChecks =
        0;


    int envelopeChecks =
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
        "[COMPOSITE-RUNTIME-SHADOW-AUDIT] BEGIN | "
        "Stage:11C.1 | Mode:SHADOW_READ_ONLY | "
        "Source:PROJECT_V1_3 | "
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


        bool pass =
            true;


        bool relativePass =
            true;


        bool envelopePass =
            true;


        bool overlapPass =
            true;


        bool simpleEquivalent =
            true;


        if (slave.runtimeCompositeBindingsShadowSchemaPresent)
        {
            schemaSlaves++;
        }
        else
        {
            pass =
                false;
        }


        const bool simpleMode =
            IsSimpleProjectionMode(
                slave.runtimeCompositeBindingsShadowMode);


        const bool compositeMode =
            IsCompositeMode(
                slave.runtimeCompositeBindingsShadowMode);


        if (simpleMode)
        {
            simpleSlaves++;
        }
        else if (compositeMode)
        {
            compositeSlaves++;
        }
        else
        {
            pass =
                false;
        }


        bindingCount +=
            static_cast<int>(
                slave.runtimeCompositeBindingsShadow.size());


        // --------------------------------------------------------------------
        // Verify shadow base bits against active Stage11A simple envelope.
        // --------------------------------------------------------------------

        int32_t expectedOutputBaseBit =
            -1;


        int32_t expectedInputBaseBit =
            -1;


        if (!slave.runtimeApplicationBindings.empty())
        {
            const auto& active =
                slave.runtimeApplicationBindings[0];


            if (active.outputBitLength >
                0U)
            {
                expectedOutputBaseBit =
                    active.outputBitOffset;
            }


            if (active.inputBitLength >
                0U)
            {
                expectedInputBaseBit =
                    active.inputBitOffset;
            }
        }


        if (slave.runtimeCompositeBindingsShadowOutputBaseBit !=
            expectedOutputBaseBit ||
            slave.runtimeCompositeBindingsShadowInputBaseBit !=
            expectedInputBaseBit)
        {
            relativePass =
                false;

            pass =
                false;
        }


        uint32_t outputCoverage =
            0U;


        uint32_t inputCoverage =
            0U;


        for (size_t bindingIndex = 0;
            bindingIndex <
            slave.runtimeCompositeBindingsShadow.size();
            bindingIndex++)
        {
            const auto& binding =
                slave.runtimeCompositeBindingsShadow[
                    bindingIndex];


            rangeChecks +=
                2;


            relativeChecks +=
                2;


            envelopeChecks +=
                2;


            if (binding.id[0] ==
                '\0' ||
                binding.kind[0] ==
                '\0' ||
                binding.dataType[0] ==
                '\0' ||
                binding.sampleMode[0] ==
                '\0')
            {
                pass =
                    false;
            }


            if (!IsAbsoluteRangeValid(
                binding.outputBitOffset,
                binding.outputBitLength,
                processImageBits) ||
                !IsAbsoluteRangeValid(
                    binding.inputBitOffset,
                    binding.inputBitLength,
                    processImageBits))
            {
                pass =
                    false;
            }


            if (!IsRelativeRangeValid(
                binding.outputRelativeBitOffset,
                binding.outputBitLength,
                slave.outputBitLength) ||
                !IsRelativeRangeValid(
                    binding.inputRelativeBitOffset,
                    binding.inputBitLength,
                    slave.inputBitLength))
            {
                relativePass =
                    false;

                pass =
                    false;
            }


            if (!RelativeToAbsoluteMatches(
                slave.runtimeCompositeBindingsShadowOutputBaseBit,
                binding.outputRelativeBitOffset,
                binding.outputBitOffset,
                binding.outputBitLength) ||
                !RelativeToAbsoluteMatches(
                    slave.runtimeCompositeBindingsShadowInputBaseBit,
                    binding.inputRelativeBitOffset,
                    binding.inputBitOffset,
                    binding.inputBitLength))
            {
                relativePass =
                    false;

                pass =
                    false;
            }


            if (!slave.runtimeApplicationBindings.empty())
            {
                const auto& active =
                    slave.runtimeApplicationBindings[0];


                if (!RangeInsideEnvelope(
                    active.outputBitOffset,
                    active.outputBitLength,
                    binding.outputBitOffset,
                    binding.outputBitLength) ||
                    !RangeInsideEnvelope(
                        active.inputBitOffset,
                        active.inputBitLength,
                        binding.inputBitOffset,
                        binding.inputBitLength))
                {
                    envelopePass =
                        false;

                    pass =
                        false;
                }
            }
            else if (binding.outputBitLength >
                0U ||
                binding.inputBitLength >
                0U)
            {
                envelopePass =
                    false;

                pass =
                    false;
            }


            outputCoverage +=
                binding.outputBitLength;


            inputCoverage +=
                binding.inputBitLength;


            for (size_t otherIndex =
                bindingIndex +
                1U;
                otherIndex <
                slave.runtimeCompositeBindingsShadow.size();
                otherIndex++)
            {
                const auto& other =
                    slave.runtimeCompositeBindingsShadow[
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
                    overlapPass =
                        false;

                    pass =
                        false;
                }
            }
        }


        coverageChecks +=
            2;


        // Project Editor intentionally allows gaps.
        // Therefore coverage may be <= Slave process-data envelope,
        // but never larger.
        if (outputCoverage >
            slave.outputBitLength ||
            inputCoverage >
            slave.inputBitLength)
        {
            pass =
                false;
        }


        // --------------------------------------------------------------------
        // SimpleProjection must still exactly match active Stage11A schema.
        // Composite mode is a decomposition/subset inside that envelope.
        // --------------------------------------------------------------------

        if (simpleMode)
        {
            simpleEquivalent =
                slave.runtimeApplicationBindings.size() ==
                1U &&
                slave.runtimeCompositeBindingsShadow.size() ==
                1U;


            if (simpleEquivalent)
            {
                simpleEquivalent =
                    SimpleProjectionEquivalent(
                        slave.runtimeApplicationBindings[0],
                        slave.runtimeCompositeBindingsShadow[0]);
            }


            if (!simpleEquivalent)
            {
                pass =
                    false;
            }
        }


        if (!pass)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-RUNTIME-SHADOW-SLAVE] "
            "S%d | Mode:%s | Bindings:%u | "
            "OutCoverage:%u/%u bits | InCoverage:%u/%u bits | "
            "OutBase:%d InBase:%d | "
            "RelativeToAbsolute:%s | Envelope:%s | Overlap:%s | "
            "SimpleEq:%s | Result:%s\n",

            slaveIndex,

            slave.runtimeCompositeBindingsShadowMode[0] != '\0'
            ? slave.runtimeCompositeBindingsShadowMode
            : "N/A",

            (unsigned int)
            slave.runtimeCompositeBindingsShadow.size(),

            (unsigned int)
            outputCoverage,

            (unsigned int)
            slave.outputBitLength,

            (unsigned int)
            inputCoverage,

            (unsigned int)
            slave.inputBitLength,

            slave.runtimeCompositeBindingsShadowOutputBaseBit,

            slave.runtimeCompositeBindingsShadowInputBaseBit,

            relativePass
            ? "PASS"
            : "FAIL",

            envelopePass
            ? "PASS"
            : "FAIL",

            overlapPass
            ? "PASS"
            : "FAIL",

            simpleMode
            ? (
                simpleEquivalent
                ? "MATCH"
                : "DIFF"
                )
            : "N/A",

            pass
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-RUNTIME-SHADOW-AUDIT-RESULT] "
        "Slaves:%u | SchemaSlaves:%d/%u | "
        "SimpleSlaves:%d | CompositeSlaves:%d | "
        "Bindings:%d | "
        "RangeChecks:%d | RelativeChecks:%d | "
        "EnvelopeChecks:%d | CoverageChecks:%d | OverlapChecks:%d | "
        "ProcessImage:%u bits | "
        "Errors:%d | Result:%s | "
        "ActiveBinding:SIMPLE_STAGE7B | "
        "ShadowAction:NO_CUTOVER | "
        "HardwareRead:NO | HardwareWrite:NO | StartupAction:CONTINUE\n",

        (unsigned int)
        slaves.size(),

        schemaSlaves,

        (unsigned int)
        slaves.size(),

        simpleSlaves,

        compositeSlaves,

        bindingCount,

        rangeChecks,

        relativeChecks,

        envelopeChecks,

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
        "[COMPOSITE-RUNTIME-SHADOW-AUDIT] END | "
        "Stage:11C.1 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
