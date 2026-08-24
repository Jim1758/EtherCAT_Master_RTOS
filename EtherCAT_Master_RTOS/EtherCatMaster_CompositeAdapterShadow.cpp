#include "EtherCatMaster.h"

#include <cstdint>
#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 11C.2 - Composite Runtime Adapter Shadow Build
//
// Builds a generic application-descriptor layer from
// runtimeCompositeBindingsShadow.
//
// NO active adapter cutover.
// NO EtherCAT hardware access.
// ============================================================================

namespace
{
    bool TextEquals(
        const char* left,
        const char* right)
    {
        return
            left != nullptr &&
            right != nullptr &&
            strcmp(left, right) == 0;
    }


    uint32_t CalculateByteSpan(
        uint8_t bitShift,
        uint32_t bitLength)
    {
        if (bitLength == 0U)
        {
            return 0U;
        }

        return
            (static_cast<uint32_t>(bitShift) +
                bitLength +
                7U) /
            8U;
    }


    bool ConfigureRange(
        char* ioMap,
        int ioMapSize,
        int32_t bitOffset,
        uint32_t bitLength,
        int32_t& byteOffset,
        uint8_t& bitShift,
        uint32_t& byteSpan,
        bool& byteAligned,
        uint8_t*& pointer)
    {
        byteOffset = -1;
        bitShift = 0U;
        byteSpan = 0U;
        byteAligned = true;
        pointer = nullptr;

        if (bitLength == 0U)
        {
            return bitOffset == -1;
        }

        if (bitOffset < 0 || ioMap == nullptr || ioMapSize <= 0)
        {
            return false;
        }

        const uint64_t endBit =
            static_cast<uint64_t>(
                static_cast<uint32_t>(bitOffset)) +
            static_cast<uint64_t>(bitLength);

        const uint64_t ioMapBits =
            static_cast<uint64_t>(ioMapSize) *
            8ULL;

        if (endBit > ioMapBits)
        {
            return false;
        }

        byteOffset =
            bitOffset /
            8;

        bitShift =
            static_cast<uint8_t>(
                bitOffset %
                8);

        byteSpan =
            CalculateByteSpan(
                bitShift,
                bitLength);

        byteAligned =
            bitShift == 0U &&
            (bitLength % 8U) == 0U;

        if (byteOffset < 0 ||
            static_cast<uint64_t>(
                static_cast<uint32_t>(byteOffset)) +
            static_cast<uint64_t>(byteSpan) >
            static_cast<uint64_t>(ioMapSize))
        {
            return false;
        }

        pointer =
            reinterpret_cast<uint8_t*>(ioMap) +
            byteOffset;

        return true;
    }


    const ENI_GenericIO* FindIoBySlave(
        const std::vector<ENI_GenericIO>& list,
        int slaveIndex)
    {
        for (const auto& item : list)
        {
            if (item.slaveIndex == slaveIndex)
            {
                return &item;
            }
        }

        return nullptr;
    }


    const ENI_AnalogModule* FindAnalogBySlave(
        const std::vector<ENI_AnalogModule>& list,
        int slaveIndex)
    {
        for (const auto& item : list)
        {
            if (item.slaveIndex == slaveIndex)
            {
                return &item;
            }
        }

        return nullptr;
    }


    const ENI_ServoDrive* FindServoBySlave(
        const std::vector<ENI_ServoDrive>& list,
        int slaveIndex)
    {
        for (const auto& item : list)
        {
            if (item.slaveIndex == slaveIndex)
            {
                return &item;
            }
        }

        return nullptr;
    }


    int PointerOffset(
        const void* pointer,
        const char* base)
    {
        if (pointer == nullptr)
        {
            return -1;
        }

        return
            static_cast<int>(
                reinterpret_cast<const char*>(pointer) -
                base);
    }


    // ========================================================================
    // Stage 11D.1 ServoPDO_9_23 field recipes.
    //
    // These are ABI recipes, not ProductCode recipes.
    // ========================================================================

    struct StructuredServoFieldRecipe
    {
        const char* fieldId;
        const char* semanticKind;
        const char* direction;
        const char* dataType;
        const char* unit;

        uint32_t relativeBitOffset;
        uint32_t bitLength;
    };


    static const StructuredServoFieldRecipe
        kServoOutputFieldRecipes[] =
    {
        {
            "ControlWord",
            "DriveCommand",
            "Output",
            "UInt16",
            "Raw",
            0U,
            16U
        },
        {
            "TargetVelocity",
            "DriveCommand",
            "Output",
            "Int32",
            "Raw",
            16U,
            32U
        },
        {
            "TouchProbeFunction",
            "TouchProbe",
            "Output",
            "UInt16",
            "Raw",
            48U,
            16U
        },
        {
            "ModesOfOperation",
            "DriveCommand",
            "Output",
            "Int8",
            "Raw",
            64U,
            8U
        }
    };


    static const StructuredServoFieldRecipe
        kServoInputFieldRecipes[] =
    {
        {
            "StatusWord",
            "DriveStatus",
            "Input",
            "UInt16",
            "Raw",
            0U,
            16U
        },
        {
            "ActualPosition",
            "PositionFeedback",
            "Input",
            "Int32",
            "Count",
            16U,
            32U
        },
        {
            "ActualVelocity",
            "VelocityFeedback",
            "Input",
            "Int32",
            "Raw",
            48U,
            32U
        },
        {
            "ActualTorque",
            "TorqueFeedback",
            "Input",
            "Int16",
            "Raw",
            80U,
            16U
        },
        {
            "TouchProbeStatus",
            "TouchProbe",
            "Input",
            "UInt16",
            "Raw",
            96U,
            16U
        },
        {
            "TouchProbePosition",
            "TouchProbe",
            "Input",
            "Int32",
            "Count",
            112U,
            32U
        },
        {
            "ModesOfOperationDisplay",
            "DriveStatus",
            "Input",
            "Int8",
            "Raw",
            144U,
            8U
        },
        {
            "Object2510",
            "Diagnostic",
            "Input",
            "UInt32",
            "Raw",
            152U,
            32U
        }
    };


    bool IsOutputRecipe(
        const StructuredServoFieldRecipe& recipe)
    {
        return
            TextEquals(
                recipe.direction,
                "Output");
    }


    const uint8_t* GetLegacyServoFieldPointer(
        const ENI_ServoDrive& servo,
        const EtherCatStructuredServoFieldDescriptor& field)
    {
        const uint8_t* base =
            nullptr;


        if (TextEquals(
            field.direction,
            "Output"))
        {
            if (servo.pOutput == nullptr)
            {
                return nullptr;
            }

            base =
                reinterpret_cast<const uint8_t*>(
                    servo.pOutput);
        }
        else if (TextEquals(
            field.direction,
            "Input"))
        {
            if (servo.pInput == nullptr)
            {
                return nullptr;
            }

            base =
                reinterpret_cast<const uint8_t*>(
                    servo.pInput);
        }
        else
        {
            return nullptr;
        }


        return
            base +
            (
                field.relativeBitOffset /
                8U
                );
    }
}


// ============================================================================
// BuildRuntimeCompositeApplicationDescriptorsShadow
// ============================================================================

bool EtherCatMaster::BuildRuntimeCompositeApplicationDescriptorsShadow()
{
    m_CompositeApplicationDescriptorsShadow.clear();

    if (m_pEni == nullptr)
    {
        DEBUG_PRINT(
            "[COMPOSITE-ADAPTER-SHADOW-BUILD] "
            "Runtime Config missing | Result:FAIL\n");

        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int expectedBindings = 0;
    int rangeEndpoints = 0;
    int byteAlignedRanges = 0;

    int genericDataCount = 0;
    int digitalInputCount = 0;
    int digitalOutputCount = 0;
    int analogInputCount = 0;
    int servoDriveCount = 0;
    int otherKindCount = 0;

    int errors = 0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-ADAPTER-SHADOW-BUILD] BEGIN | "
        "Stage:11C.2 | Source:COMPOSITE_RUNTIME_SHADOW | "
        "ActiveAdapters:LEGACY_STAGE7B | "
        "HardwareRead:NO | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIndex = 0;
        slaveIndex < static_cast<int>(slaves.size());
        ++slaveIndex)
    {
        const EtherCatSlave& slave =
            slaves[static_cast<size_t>(slaveIndex)];

        expectedBindings +=
            static_cast<int>(
                slave.runtimeCompositeBindingsShadow.size());


        for (size_t bindingIndex = 0;
            bindingIndex <
            slave.runtimeCompositeBindingsShadow.size();
            ++bindingIndex)
        {
            const auto& binding =
                slave.runtimeCompositeBindingsShadow[bindingIndex];

            EtherCatCompositeApplicationDescriptor descriptor;

            descriptor.slaveIndex = slaveIndex;
            descriptor.bindingIndex =
                static_cast<int32_t>(bindingIndex);
            descriptor.sortOrder = binding.sortOrder;

            strncpy_s(
                descriptor.id,
                sizeof(descriptor.id),
                binding.id,
                _TRUNCATE);

            strncpy_s(
                descriptor.name,
                sizeof(descriptor.name),
                binding.name,
                _TRUNCATE);

            strncpy_s(
                descriptor.kind,
                sizeof(descriptor.kind),
                binding.kind,
                _TRUNCATE);

            strncpy_s(
                descriptor.legacyKind,
                sizeof(descriptor.legacyKind),
                binding.legacyKind,
                _TRUNCATE);

            strncpy_s(
                descriptor.abi,
                sizeof(descriptor.abi),
                binding.abi,
                _TRUNCATE);

            strncpy_s(
                descriptor.interfaceType,
                sizeof(descriptor.interfaceType),
                binding.interfaceType,
                _TRUNCATE);

            strncpy_s(
                descriptor.dataType,
                sizeof(descriptor.dataType),
                binding.dataType,
                _TRUNCATE);

            strncpy_s(
                descriptor.sampleMode,
                sizeof(descriptor.sampleMode),
                binding.sampleMode,
                _TRUNCATE);

            strncpy_s(
                descriptor.unit,
                sizeof(descriptor.unit),
                binding.unit,
                _TRUNCATE);

            strncpy_s(
                descriptor.axisRef,
                sizeof(descriptor.axisRef),
                binding.axisRef,
                _TRUNCATE);

            descriptor.outputBitOffset =
                binding.outputBitOffset;
            descriptor.outputBitLength =
                binding.outputBitLength;

            descriptor.inputBitOffset =
                binding.inputBitOffset;
            descriptor.inputBitLength =
                binding.inputBitLength;

            descriptor.elementBits =
                binding.elementBits;
            descriptor.channelCount =
                binding.channelCount;


            const bool outputPass =
                ConfigureRange(
                    m_IoMap,
                    m_IoMapSize,
                    descriptor.outputBitOffset,
                    descriptor.outputBitLength,
                    descriptor.outputByteOffset,
                    descriptor.outputBitShift,
                    descriptor.outputByteSpan,
                    descriptor.outputByteAligned,
                    descriptor.pOutputByteBase);


            const bool inputPass =
                ConfigureRange(
                    m_IoMap,
                    m_IoMapSize,
                    descriptor.inputBitOffset,
                    descriptor.inputBitLength,
                    descriptor.inputByteOffset,
                    descriptor.inputBitShift,
                    descriptor.inputByteSpan,
                    descriptor.inputByteAligned,
                    descriptor.pInputByteBase);


            if (!outputPass || !inputPass)
            {
                errors++;
            }


            if (descriptor.outputBitLength > 0U)
            {
                rangeEndpoints++;

                if (descriptor.outputByteAligned)
                {
                    byteAlignedRanges++;
                }
            }


            if (descriptor.inputBitLength > 0U)
            {
                rangeEndpoints++;

                if (descriptor.inputByteAligned)
                {
                    byteAlignedRanges++;
                }
            }


            if (TextEquals(descriptor.kind, "GenericData"))
            {
                genericDataCount++;
            }
            else if (TextEquals(descriptor.kind, "DigitalInput"))
            {
                digitalInputCount++;
            }
            else if (TextEquals(descriptor.kind, "DigitalOutput"))
            {
                digitalOutputCount++;
            }
            else if (TextEquals(descriptor.kind, "AnalogInput"))
            {
                analogInputCount++;
            }
            else if (TextEquals(descriptor.kind, "ServoDrive"))
            {
                servoDriveCount++;
            }
            else
            {
                otherKindCount++;
            }


            m_CompositeApplicationDescriptorsShadow.push_back(
                descriptor);


            DEBUG_PRINT(
                "[COMPOSITE-ADAPTER-SHADOW-DESC] "
                "S%d B%u | Id:%s Kind:%s Type:%s Axis:%s | "
                "Out:%d/%u Byte:%d Shift:%u Span:%u Align:%s | "
                "In:%d/%u Byte:%d Shift:%u Span:%u Align:%s | "
                "Elem:%u Ch:%u | Result:%s\n",

                slaveIndex,
                (unsigned int)bindingIndex,

                descriptor.id[0] != '\0'
                ? descriptor.id
                : "N/A",

                descriptor.kind[0] != '\0'
                ? descriptor.kind
                : "N/A",

                descriptor.dataType[0] != '\0'
                ? descriptor.dataType
                : "N/A",

                descriptor.axisRef[0] != '\0'
                ? descriptor.axisRef
                : "-",

                descriptor.outputBitOffset,
                (unsigned int)descriptor.outputBitLength,
                descriptor.outputByteOffset,
                (unsigned int)descriptor.outputBitShift,
                (unsigned int)descriptor.outputByteSpan,
                descriptor.outputByteAligned ? "YES" : "NO",

                descriptor.inputBitOffset,
                (unsigned int)descriptor.inputBitLength,
                descriptor.inputByteOffset,
                (unsigned int)descriptor.inputBitShift,
                (unsigned int)descriptor.inputByteSpan,
                descriptor.inputByteAligned ? "YES" : "NO",

                (unsigned int)descriptor.elementBits,
                (unsigned int)descriptor.channelCount,

                outputPass && inputPass
                ? "PASS"
                : "FAIL");
        }
    }


    const bool pass =
        errors == 0 &&
        static_cast<int>(
            m_CompositeApplicationDescriptorsShadow.size()) ==
        expectedBindings;


    DEBUG_PRINT(
        "[COMPOSITE-ADAPTER-SHADOW-BUILD-RESULT] "
        "Slaves:%u | Bindings:%d | Descriptors:%u | "
        "RangeEndpoints:%d | ByteAlignedRanges:%d | "
        "Kinds GenericData:%d DI:%d DO:%d AI:%d Servo:%d Other:%d | "
        "Errors:%d | Result:%s | "
        "ActiveAdapters:LEGACY_STAGE7B | "
        "ShadowAction:NO_CUTOVER | "
        "HardwareRead:NO | HardwareWrite:NO\n",

        (unsigned int)slaves.size(),
        expectedBindings,
        (unsigned int)
        m_CompositeApplicationDescriptorsShadow.size(),
        rangeEndpoints,
        byteAlignedRanges,
        genericDataCount,
        digitalInputCount,
        digitalOutputCount,
        analogInputCount,
        servoDriveCount,
        otherKindCount,
        errors,
        pass ? "PASS" : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-ADAPTER-SHADOW-BUILD] END | "
        "Stage:11C.2 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass ? "PASS" : "FAIL");


    return pass;
}


// ============================================================================
// AuditRuntimeCompositeApplicationDescriptorsShadow
//
// Compare generic descriptors against CURRENT Stage7B adapters using
// compatibility Binding Kind, not VendorId/ProductCode.
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeApplicationDescriptorsShadow()
{
    if (m_pEni == nullptr)
    {
        DEBUG_PRINT(
            "[COMPOSITE-ADAPTER-SHADOW-AUDIT] "
            "Runtime Config missing | Result:FAIL\n");

        return false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int pointerChecks = 0;
    int envelopeChecks = 0;
    int legacyIoChecks = 0;
    int legacyAnalogChecks = 0;
    int legacyServoChecks = 0;
    int errors = 0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-ADAPTER-SHADOW-AUDIT] BEGIN | "
        "Stage:11C.2 | Mode:SHADOW_READ_ONLY | "
        "CompareTo:STAGE7B_ACTIVE_ADAPTERS | "
        "ProductCodeBaseline:NO | "
        "HardwareRead:NO | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIndex = 0;
        slaveIndex < static_cast<int>(slaves.size());
        ++slaveIndex)
    {
        const EtherCatSlave& slave =
            slaves[static_cast<size_t>(slaveIndex)];


        std::vector<const EtherCatCompositeApplicationDescriptor*>
            descriptors;


        for (const auto& descriptor :
            m_CompositeApplicationDescriptorsShadow)
        {
            if (descriptor.slaveIndex == slaveIndex)
            {
                descriptors.push_back(&descriptor);
            }
        }


        bool pass =
            descriptors.size() ==
            slave.runtimeCompositeBindingsShadow.size();

        bool pointerPass = true;
        bool legacyPass = true;


        for (const auto* descriptor : descriptors)
        {
            if (descriptor == nullptr)
            {
                pointerPass = false;
                pass = false;
                continue;
            }


            if (descriptor->outputBitLength > 0U)
            {
                pointerChecks++;

                if (descriptor->pOutputByteBase == nullptr ||
                    PointerOffset(
                        descriptor->pOutputByteBase,
                        m_IoMap) !=
                    descriptor->outputByteOffset)
                {
                    pointerPass = false;
                    pass = false;
                }
            }


            if (descriptor->inputBitLength > 0U)
            {
                pointerChecks++;

                if (descriptor->pInputByteBase == nullptr ||
                    PointerOffset(
                        descriptor->pInputByteBase,
                        m_IoMap) !=
                    descriptor->inputByteOffset)
                {
                    pointerPass = false;
                    pass = false;
                }
            }
        }


        const auto& active =
            slave.runtimeProcessImageBinding;


        if (TextEquals(active.kind, "GenericIO"))
        {
            legacyIoChecks++;

            const ENI_GenericIO* io =
                FindIoBySlave(
                    m_IoList,
                    slaveIndex);


            if (io == nullptr)
            {
                legacyPass = false;
                pass = false;
            }
            else
            {
                const int expectedOutByte =
                    active.outputBytes > 0U
                    ? active.outputOffset
                    : -1;

                const int expectedInByte =
                    active.inputBytes > 0U
                    ? active.inputOffset
                    : -1;


                if (PointerOffset(io->pOutputLoc, m_IoMap) != expectedOutByte ||
                    PointerOffset(io->pInputLoc, m_IoMap) != expectedInByte)
                {
                    legacyPass = false;
                    pass = false;
                }


                for (const auto* descriptor : descriptors)
                {
                    if (descriptor == nullptr)
                    {
                        continue;
                    }


                    if (descriptor->outputBitLength > 0U)
                    {
                        envelopeChecks++;

                        const int32_t begin =
                            active.outputOffset * 8;

                        const int32_t end =
                            begin +
                            static_cast<int32_t>(
                                active.outputBytes * 8U);

                        const int32_t descriptorEnd =
                            descriptor->outputBitOffset +
                            static_cast<int32_t>(
                                descriptor->outputBitLength);

                        if (descriptor->outputBitOffset < begin ||
                            descriptorEnd > end)
                        {
                            legacyPass = false;
                            pass = false;
                        }
                    }


                    if (descriptor->inputBitLength > 0U)
                    {
                        envelopeChecks++;

                        const int32_t begin =
                            active.inputOffset * 8;

                        const int32_t end =
                            begin +
                            static_cast<int32_t>(
                                active.inputBytes * 8U);

                        const int32_t descriptorEnd =
                            descriptor->inputBitOffset +
                            static_cast<int32_t>(
                                descriptor->inputBitLength);

                        if (descriptor->inputBitOffset < begin ||
                            descriptorEnd > end)
                        {
                            legacyPass = false;
                            pass = false;
                        }
                    }
                }
            }
        }
        else if (TextEquals(active.kind, "AnalogInput"))
        {
            legacyAnalogChecks++;

            const ENI_AnalogModule* analog =
                FindAnalogBySlave(
                    m_AdList,
                    slaveIndex);


            if (analog == nullptr)
            {
                legacyPass = false;
                pass = false;
            }
            else
            {
                int descriptorChannels = 0;
                uint32_t descriptorInputBits = 0U;
                int32_t minInputBit = -1;
                bool allAnalog = true;


                for (const auto* descriptor : descriptors)
                {
                    if (descriptor == nullptr)
                    {
                        continue;
                    }

                    if (!TextEquals(
                        descriptor->kind,
                        "AnalogInput"))
                    {
                        allAnalog = false;
                    }

                    descriptorChannels +=
                        descriptor->channelCount;

                    descriptorInputBits +=
                        descriptor->inputBitLength;

                    if (descriptor->inputBitLength > 0U &&
                        (minInputBit < 0 ||
                            descriptor->inputBitOffset < minInputBit))
                    {
                        minInputBit =
                            descriptor->inputBitOffset;
                    }

                    if (descriptor->inputBitLength > 0U)
                    {
                        envelopeChecks++;
                    }
                }


                // =========================================================
                // Stage 11C.2 Fix1
                //
                // Do NOT use ENI_AnalogModule::channelCount here.
                //
                // Current proven Stage7B construction resizes
                // channelValues to binding.channelCount, but does not assign
                // the separate channelCount member.  Therefore the
                // authoritative initialized legacy state is:
                //
                //     channelValues.size()
                //
                // This is also what the proven Stage7A/7B audit validates.
                // =========================================================

                const int legacyChannels =
                    static_cast<int>(
                        analog->channelValues.size());


                const uint32_t legacyInputBits =
                    static_cast<uint32_t>(
                        analog->channelValues.size() *
                        sizeof(int16_t) *
                        8U);


                DEBUG_PRINT(
                    "[COMPOSITE-ADAPTER-SHADOW-ANALOG] "
                    "S%d | DescriptorChannels:%d LegacyChannels:%d | "
                    "DescriptorBits:%u LegacyBits:%u ActiveBits:%u | "
                    "FirstBit:%d ActiveFirstBit:%d | "
                    "LegacyPtrByte:%d ActivePtrByte:%d\n",

                    slaveIndex,

                    descriptorChannels,

                    legacyChannels,

                    (unsigned int)
                    descriptorInputBits,

                    (unsigned int)
                    legacyInputBits,

                    (unsigned int)
                    (active.inputBytes * 8U),

                    minInputBit,

                    active.inputOffset * 8,

                    PointerOffset(
                        analog->pInputLoc,
                        m_IoMap),

                    active.inputOffset);


                if (!allAnalog ||
                    descriptorChannels != legacyChannels ||
                    descriptorInputBits != legacyInputBits ||
                    descriptorInputBits != active.inputBytes * 8U ||
                    minInputBit != active.inputOffset * 8 ||
                    PointerOffset(
                        analog->pInputLoc,
                        m_IoMap) !=
                    active.inputOffset)
                {
                    legacyPass = false;
                    pass = false;
                }
            }
        }
        else if (TextEquals(active.kind, "Servo"))
        {
            legacyServoChecks++;

            const ENI_ServoDrive* servo =
                FindServoBySlave(
                    m_ServoList,
                    slaveIndex);


            if (servo == nullptr ||
                descriptors.size() != 1U ||
                !TextEquals(
                    descriptors[0]->kind,
                    "ServoDrive"))
            {
                legacyPass = false;
                pass = false;
            }
            else
            {
                const auto* descriptor =
                    descriptors[0];

                envelopeChecks += 2;

                if (PointerOffset(servo->pOutput, m_IoMap) != active.outputOffset ||
                    PointerOffset(servo->pInput, m_IoMap) != active.inputOffset ||
                    descriptor->outputBitOffset != active.outputOffset * 8 ||
                    descriptor->inputBitOffset != active.inputOffset * 8 ||
                    descriptor->outputBitLength != active.outputBytes * 8U ||
                    descriptor->inputBitLength != active.inputBytes * 8U)
                {
                    legacyPass = false;
                    pass = false;
                }
            }
        }
        else
        {
            legacyPass = false;
            pass = false;
        }


        if (!pass)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-ADAPTER-SHADOW-SLAVE] "
            "S%d | CompatKind:%s | Descriptors:%u | "
            "Pointer:%s | LegacyEnvelope:%s | Result:%s\n",

            slaveIndex,

            active.kind[0] != '\0'
            ? active.kind
            : "N/A",

            (unsigned int)descriptors.size(),

            pointerPass ? "PASS" : "FAIL",
            legacyPass ? "PASS" : "FAIL",
            pass ? "PASS" : "FAIL");
    }


    const bool pass =
        errors == 0;


    DEBUG_PRINT(
        "[COMPOSITE-ADAPTER-SHADOW-AUDIT-RESULT] "
        "Slaves:%u | Descriptors:%u | "
        "PointerChecks:%d | EnvelopeChecks:%d | "
        "LegacyIO:%d | LegacyAnalog:%d | LegacyServo:%d | "
        "Errors:%d | Result:%s | "
        "ProductCodeBaseline:NO | "
        "ActiveAdapters:LEGACY_STAGE7B | "
        "ShadowAction:NO_CUTOVER | "
        "HardwareRead:NO | HardwareWrite:NO | StartupAction:CONTINUE\n",

        (unsigned int)slaves.size(),
        (unsigned int)
        m_CompositeApplicationDescriptorsShadow.size(),
        pointerChecks,
        envelopeChecks,
        legacyIoChecks,
        legacyAnalogChecks,
        legacyServoChecks,
        errors,
        pass ? "PASS" : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-ADAPTER-SHADOW-AUDIT] END | "
        "Stage:11C.2 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass ? "PASS" : "FAIL");


    return pass;
}


// ============================================================================
// Stage 11D.1 - Structured ServoDrive Adapter Shadow
// ============================================================================

bool EtherCatMaster::BuildStructuredServoDriveFieldDescriptorsShadow()
{
    m_StructuredServoDriveFieldsShadow.clear();


    int servoParents =
        0;

    int outputFields =
        0;

    int inputFields =
        0;

    int outputCoverageBits =
        0;

    int inputCoverageBits =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-STRUCTURED-SHADOW-BUILD] BEGIN | "
        "Stage:11D.1 | "
        "Parent:SERVO_DRIVE_COMPOSITE_DESCRIPTOR | "
        "ABI:ServoPDO_9_23 | "
        "ProductCodeBaseline:NO | "
        "MotionConsumer:LEGACY | "
        "ServoCommand:LEGACY | "
        "LiveWrite:NO\n"
        "============================================================\n");


    for (const auto& parent :
        m_CompositeApplicationDescriptorsShadow)
    {
        if (!TextEquals(
            parent.kind,
            "ServoDrive"))
        {
            continue;
        }


        servoParents++;


        const ENI_ServoDrive* servo =
            FindServoBySlave(
                m_ServoList,
                parent.slaveIndex);


        bool parentPass =
            servo != nullptr &&
            servo->pOutput != nullptr &&
            servo->pInput != nullptr &&
            TextEquals(
                parent.abi,
                "ServoPDO_9_23") &&
            parent.outputBitLength ==
            72U &&
            parent.inputBitLength ==
            184U &&
            parent.outputByteAligned &&
            parent.inputByteAligned &&
            parent.pOutputByteBase !=
            nullptr &&
            parent.pInputByteBase !=
            nullptr;


        int servoOutputFields =
            0;

        int servoInputFields =
            0;

        int servoOutputBits =
            0;

        int servoInputBits =
            0;


        if (parentPass)
        {
            for (const auto& recipe :
                kServoOutputFieldRecipes)
            {
                EtherCatStructuredServoFieldDescriptor
                    field;


                field.slaveIndex =
                    parent.slaveIndex;

                field.parentBindingIndex =
                    parent.bindingIndex;


                strncpy_s(
                    field.parentBindingId,
                    sizeof(field.parentBindingId),
                    parent.id,
                    _TRUNCATE);

                strncpy_s(
                    field.parentAbi,
                    sizeof(field.parentAbi),
                    parent.abi,
                    _TRUNCATE);

                strncpy_s(
                    field.fieldId,
                    sizeof(field.fieldId),
                    recipe.fieldId,
                    _TRUNCATE);

                strncpy_s(
                    field.semanticKind,
                    sizeof(field.semanticKind),
                    recipe.semanticKind,
                    _TRUNCATE);

                strncpy_s(
                    field.direction,
                    sizeof(field.direction),
                    recipe.direction,
                    _TRUNCATE);

                strncpy_s(
                    field.interfaceType,
                    sizeof(field.interfaceType),
                    parent.interfaceType,
                    _TRUNCATE);

                strncpy_s(
                    field.dataType,
                    sizeof(field.dataType),
                    recipe.dataType,
                    _TRUNCATE);

                strncpy_s(
                    field.unit,
                    sizeof(field.unit),
                    recipe.unit,
                    _TRUNCATE);

                strncpy_s(
                    field.axisRef,
                    sizeof(field.axisRef),
                    parent.axisRef,
                    _TRUNCATE);


                field.relativeBitOffset =
                    recipe.relativeBitOffset;

                field.absoluteBitOffset =
                    parent.outputBitOffset +
                    static_cast<int32_t>(
                        recipe.relativeBitOffset);

                field.bitLength =
                    recipe.bitLength;

                field.byteOffset =
                    field.absoluteBitOffset /
                    8;

                field.byteSpan =
                    recipe.bitLength /
                    8U;

                field.byteAligned =
                    (
                        field.absoluteBitOffset %
                        8
                        ) ==
                    0 &&
                    (
                        field.bitLength %
                        8U
                        ) ==
                    0U;


                const uint64_t relativeEnd =
                    static_cast<uint64_t>(
                        recipe.relativeBitOffset) +
                    static_cast<uint64_t>(
                        recipe.bitLength);


                if (!field.byteAligned ||
                    relativeEnd >
                    static_cast<uint64_t>(
                        parent.outputBitLength) ||
                    field.byteOffset <
                    0 ||
                    field.byteOffset +
                    static_cast<int32_t>(
                        field.byteSpan) >
                    m_IoMapSize)
                {
                    parentPass =
                        false;

                    errors++;

                    continue;
                }


                field.pByteBase =
                    reinterpret_cast<uint8_t*>(
                        m_IoMap) +
                    field.byteOffset;


                m_StructuredServoDriveFieldsShadow.push_back(
                    field);


                outputFields++;
                servoOutputFields++;
                outputCoverageBits +=
                    static_cast<int>(
                        recipe.bitLength);
                servoOutputBits +=
                    static_cast<int>(
                        recipe.bitLength);
            }


            for (const auto& recipe :
                kServoInputFieldRecipes)
            {
                EtherCatStructuredServoFieldDescriptor
                    field;


                field.slaveIndex =
                    parent.slaveIndex;

                field.parentBindingIndex =
                    parent.bindingIndex;


                strncpy_s(
                    field.parentBindingId,
                    sizeof(field.parentBindingId),
                    parent.id,
                    _TRUNCATE);

                strncpy_s(
                    field.parentAbi,
                    sizeof(field.parentAbi),
                    parent.abi,
                    _TRUNCATE);

                strncpy_s(
                    field.fieldId,
                    sizeof(field.fieldId),
                    recipe.fieldId,
                    _TRUNCATE);

                strncpy_s(
                    field.semanticKind,
                    sizeof(field.semanticKind),
                    recipe.semanticKind,
                    _TRUNCATE);

                strncpy_s(
                    field.direction,
                    sizeof(field.direction),
                    recipe.direction,
                    _TRUNCATE);

                strncpy_s(
                    field.interfaceType,
                    sizeof(field.interfaceType),
                    parent.interfaceType,
                    _TRUNCATE);

                strncpy_s(
                    field.dataType,
                    sizeof(field.dataType),
                    recipe.dataType,
                    _TRUNCATE);

                strncpy_s(
                    field.unit,
                    sizeof(field.unit),
                    recipe.unit,
                    _TRUNCATE);

                strncpy_s(
                    field.axisRef,
                    sizeof(field.axisRef),
                    parent.axisRef,
                    _TRUNCATE);


                field.relativeBitOffset =
                    recipe.relativeBitOffset;

                field.absoluteBitOffset =
                    parent.inputBitOffset +
                    static_cast<int32_t>(
                        recipe.relativeBitOffset);

                field.bitLength =
                    recipe.bitLength;

                field.byteOffset =
                    field.absoluteBitOffset /
                    8;

                field.byteSpan =
                    recipe.bitLength /
                    8U;

                field.byteAligned =
                    (
                        field.absoluteBitOffset %
                        8
                        ) ==
                    0 &&
                    (
                        field.bitLength %
                        8U
                        ) ==
                    0U;


                const uint64_t relativeEnd =
                    static_cast<uint64_t>(
                        recipe.relativeBitOffset) +
                    static_cast<uint64_t>(
                        recipe.bitLength);


                if (!field.byteAligned ||
                    relativeEnd >
                    static_cast<uint64_t>(
                        parent.inputBitLength) ||
                    field.byteOffset <
                    0 ||
                    field.byteOffset +
                    static_cast<int32_t>(
                        field.byteSpan) >
                    m_IoMapSize)
                {
                    parentPass =
                        false;

                    errors++;

                    continue;
                }


                field.pByteBase =
                    reinterpret_cast<uint8_t*>(
                        m_IoMap) +
                    field.byteOffset;


                m_StructuredServoDriveFieldsShadow.push_back(
                    field);


                inputFields++;
                servoInputFields++;
                inputCoverageBits +=
                    static_cast<int>(
                        recipe.bitLength);
                servoInputBits +=
                    static_cast<int>(
                        recipe.bitLength);
            }
        }


        if (servoOutputFields !=
            4 ||
            servoInputFields !=
            8 ||
            servoOutputBits !=
            72 ||
            servoInputBits !=
            184)
        {
            parentPass =
                false;
        }


        if (!parentPass)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[SERVO-STRUCTURED-SHADOW-SERVO] "
            "S%d | Parent:%s | ABI:%s | "
            "Fields:%d Out:%d/%d bits In:%d/%d bits | "
            "Axis:%s | Result:%s\n",

            parent.slaveIndex,

            parent.id[0] != '\0'
            ? parent.id
            : "N/A",

            parent.abi[0] != '\0'
            ? parent.abi
            : "N/A",

            servoOutputFields +
            servoInputFields,

            servoOutputBits,

            72,

            servoInputBits,

            184,

            parent.axisRef[0] != '\0'
            ? parent.axisRef
            : "-",

            parentPass
            ? "PASS"
            : "FAIL");
    }


    const int expectedFields =
        servoParents *
        12;


    const bool pass =
        errors ==
        0 &&
        servoParents ==
        static_cast<int>(
            m_ServoList.size()) &&
        static_cast<int>(
            m_StructuredServoDriveFieldsShadow.size()) ==
        expectedFields;


    DEBUG_PRINT(
        "[SERVO-STRUCTURED-SHADOW-BUILD-RESULT] "
        "ServoParents:%d | Fields:%u/%d | "
        "OutputFields:%d OutputBits:%d | "
        "InputFields:%d InputBits:%d | "
        "Errors:%d | Result:%s | "
        "ABI:ServoPDO_9_23 | "
        "ProductCodeBaseline:NO | "
        "MotionConsumer:LEGACY | "
        "ServoCommand:LEGACY | "
        "LiveWrite:NO | "
        "ShadowOnly:YES\n",

        servoParents,

        (unsigned int)
        m_StructuredServoDriveFieldsShadow.size(),

        expectedFields,

        outputFields,

        outputCoverageBits,

        inputFields,

        inputCoverageBits,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-STRUCTURED-SHADOW-BUILD] END | "
        "Stage:11D.1 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


const EtherCatStructuredServoFieldDescriptor*
EtherCatMaster::FindStructuredServoDriveFieldShadow(
    int slaveIndex,
    const char* fieldId) const
{
    if (fieldId ==
        nullptr)
    {
        return
            nullptr;
    }


    for (const auto& field :
        m_StructuredServoDriveFieldsShadow)
    {
        if (field.slaveIndex ==
            slaveIndex &&
            TextEquals(
                field.fieldId,
                fieldId))
        {
            return
                &field;
        }
    }


    return
        nullptr;
}


bool EtherCatMaster::AuditStructuredServoDriveFieldDescriptorsShadow()
{
    int servoChecks =
        0;

    int fieldChecks =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int valueChecks =
        0;

    int valueMatches =
        0;

    int coverageChecks =
        0;

    int coverageMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-STRUCTURED-SHADOW-AUDIT] BEGIN | "
        "Stage:11D.1 | "
        "CompareTo:ENI_SERVO_DRIVE_LEGACY | "
        "ABI:ServoPDO_9_23 | "
        "ProductCodeBaseline:NO | "
        "HardwareRead:NO | HardwareWrite:NO | "
        "MotionConsumer:LEGACY | "
        "ServoCommand:LEGACY\n"
        "============================================================\n");


    for (const auto& servo :
        m_ServoList)
    {
        servoChecks++;


        bool outputCoverage[
            72] =
            {
                false
            };


            bool inputCoverage[
                184] =
                {
                    false
                };


                int servoFields =
                    0;

                int servoPointerChecks =
                    0;

                int servoPointerMatches =
                    0;

                int servoValueChecks =
                    0;

                int servoValueMatches =
                    0;

                bool servoPass =
                    servo.pOutput !=
                    nullptr &&
                    servo.pInput !=
                    nullptr;


                for (const auto& field :
                    m_StructuredServoDriveFieldsShadow)
                {
                    if (field.slaveIndex !=
                        servo.slaveIndex)
                    {
                        continue;
                    }


                    servoFields++;
                    fieldChecks++;


                    const uint8_t* legacyPointer =
                        GetLegacyServoFieldPointer(
                            servo,
                            field);


                    pointerChecks++;
                    servoPointerChecks++;


                    const bool pointerMatch =
                        legacyPointer !=
                        nullptr &&
                        field.pByteBase !=
                        nullptr &&
                        reinterpret_cast<const uint8_t*>(
                            field.pByteBase) ==
                        legacyPointer;


                    if (pointerMatch)
                    {
                        pointerMatches++;
                        servoPointerMatches++;
                    }
                    else
                    {
                        servoPass =
                            false;

                        errors++;
                    }


                    valueChecks++;
                    servoValueChecks++;


                    const bool valueMatch =
                        pointerMatch &&
                        field.byteSpan >
                        0U &&
                        memcmp(
                            field.pByteBase,
                            legacyPointer,
                            field.byteSpan) ==
                        0;


                    if (valueMatch)
                    {
                        valueMatches++;
                        servoValueMatches++;
                    }
                    else
                    {
                        servoPass =
                            false;

                        errors++;
                    }


                    const uint32_t startBit =
                        field.relativeBitOffset;

                    const uint32_t endBit =
                        startBit +
                        field.bitLength;


                    if (TextEquals(
                        field.direction,
                        "Output"))
                    {
                        if (endBit >
                            72U)
                        {
                            servoPass =
                                false;

                            errors++;
                        }
                        else
                        {
                            for (uint32_t bit =
                                startBit;
                                bit <
                                endBit;
                                ++bit)
                            {
                                if (outputCoverage[
                                    bit])
                                {
                                    servoPass =
                                        false;

                                    errors++;
                                }

                                    outputCoverage[
                                        bit] =
                                        true;
                            }
                        }
                    }
                    else if (TextEquals(
                        field.direction,
                        "Input"))
                    {
                        if (endBit >
                            184U)
                        {
                            servoPass =
                                false;

                            errors++;
                        }
                        else
                        {
                            for (uint32_t bit =
                                startBit;
                                bit <
                                endBit;
                                ++bit)
                            {
                                if (inputCoverage[
                                    bit])
                                {
                                    servoPass =
                                        false;

                                    errors++;
                                }

                                    inputCoverage[
                                        bit] =
                                        true;
                            }
                        }
                    }
                    else
                    {
                        servoPass =
                            false;

                        errors++;
                    }
                }


                bool outputComplete =
                    true;

                for (bool bit :
                outputCoverage)
                {
                    if (!bit)
                    {
                        outputComplete =
                            false;

                        break;
                    }
                }


                bool inputComplete =
                    true;

                for (bool bit :
                inputCoverage)
                {
                    if (!bit)
                    {
                        inputComplete =
                            false;

                        break;
                    }
                }


                coverageChecks +=
                    2;


                if (outputComplete)
                {
                    coverageMatches++;
                }
                else
                {
                    servoPass =
                        false;

                    errors++;
                }


                if (inputComplete)
                {
                    coverageMatches++;
                }
                else
                {
                    servoPass =
                        false;

                    errors++;
                }


                if (servoFields !=
                    12)
                {
                    servoPass =
                        false;

                    errors++;
                }


                DEBUG_PRINT(
                    "[SERVO-STRUCTURED-SHADOW-AUDIT-SERVO] "
                    "S%d | Fields:%d/12 | "
                    "Pointer:%d/%d | "
                    "Values:%d/%d | "
                    "OutCoverage:%s | InCoverage:%s | "
                    "Result:%s\n",

                    servo.slaveIndex,

                    servoFields,

                    servoPointerMatches,

                    servoPointerChecks,

                    servoValueMatches,

                    servoValueChecks,

                    outputComplete
                    ? "72/72"
                    : "FAIL",

                    inputComplete
                    ? "184/184"
                    : "FAIL",

                    servoPass
                    ? "PASS"
                    : "FAIL");
    }


    const bool pass =
        errors ==
        0 &&
        servoChecks ==
        3 &&
        fieldChecks ==
        36 &&
        pointerChecks ==
        36 &&
        pointerMatches ==
        36 &&
        valueChecks ==
        36 &&
        valueMatches ==
        36 &&
        coverageChecks ==
        6 &&
        coverageMatches ==
        6;


    DEBUG_PRINT(
        "[SERVO-STRUCTURED-SHADOW-AUDIT-RESULT] "
        "Servos:%d | Fields:%d | "
        "PointerChecks:%d PointerMatches:%d | "
        "ValueChecks:%d ValueMatches:%d | "
        "CoverageChecks:%d CoverageMatches:%d | "
        "Errors:%d | Result:%s | "
        "ABI:ServoPDO_9_23 | "
        "StructuredFields:ACTIVE_SHADOW | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | "
        "ProductCodeBaseline:NO | "
        "GateAction:%s | StartupAction:CONTINUE\n",

        servoChecks,

        fieldChecks,

        pointerChecks,

        pointerMatches,

        valueChecks,

        valueMatches,

        coverageChecks,

        coverageMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        pass
        ? "ALLOW_STAGE11D2"
        : "BLOCK_SERVO_GENERIC_CUTOVER");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-STRUCTURED-SHADOW-AUDIT] END | "
        "Stage:11D.1 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
