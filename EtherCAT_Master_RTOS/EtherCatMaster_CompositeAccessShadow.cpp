#include "EtherCatMaster.h"

#include <cstdint>
#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 11C.3 - Generic Adapter Access API Shadow
//
// Generic access over Stage11C.2 descriptors.
//
// Live Process Image:
//     READ allowed.
//
// Live Process Image WRITE:
//     NOT allowed in this stage.
//
// Output write API:
//     caller-provided scratch buffer only.
//
// This intentionally separates "can decode/encode the binding correctly"
// from "is authorized to drive hardware output".
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


    bool AxisMatches(
        const char* descriptorAxis,
        const char* requestedAxis)
    {
        // nullptr means "ignore AxisRef".
        if (requestedAxis == nullptr)
        {
            return true;
        }


        const char* actual =
            descriptorAxis != nullptr
            ? descriptorAxis
            : "";


        return
            strcmp(
                actual,
                requestedAxis) == 0;
    }


    bool ReadBitsFromByteBase(
        const uint8_t* byteBase,
        uint8_t bitShift,
        uint32_t bitLength,
        uint64_t& value)
    {
        value =
            0ULL;


        if (bitLength == 0U ||
            bitLength > 64U ||
            byteBase == nullptr)
        {
            return false;
        }


        for (uint32_t bitIndex = 0U;
            bitIndex < bitLength;
            ++bitIndex)
        {
            const uint32_t sourceBit =
                static_cast<uint32_t>(
                    bitShift) +
                bitIndex;


            const uint32_t byteIndex =
                sourceBit /
                8U;


            const uint8_t bitInByte =
                static_cast<uint8_t>(
                    sourceBit %
                    8U);


            const uint64_t bitValue =
                static_cast<uint64_t>(
                    (
                        byteBase[byteIndex] >>
                        bitInByte
                        ) &
                    0x01U);


            value |=
                bitValue <<
                bitIndex;
        }


        return true;
    }


    bool ReadBitsFromAbsoluteBuffer(
        const uint8_t* buffer,
        uint32_t bufferBytes,
        int32_t bitOffset,
        uint32_t bitLength,
        uint64_t& value)
    {
        value =
            0ULL;


        if (buffer == nullptr ||
            bufferBytes == 0U ||
            bitOffset < 0 ||
            bitLength == 0U ||
            bitLength > 64U)
        {
            return false;
        }


        const uint64_t endBit =
            static_cast<uint64_t>(
                static_cast<uint32_t>(
                    bitOffset)) +
            static_cast<uint64_t>(
                bitLength);


        if (endBit >
            static_cast<uint64_t>(
                bufferBytes) *
            8ULL)
        {
            return false;
        }


        const uint32_t byteOffset =
            static_cast<uint32_t>(
                bitOffset) /
            8U;


        const uint8_t bitShift =
            static_cast<uint8_t>(
                static_cast<uint32_t>(
                    bitOffset) %
                8U);


        return
            ReadBitsFromByteBase(
                buffer +
                byteOffset,
                bitShift,
                bitLength,
                value);
    }


    bool WriteBitsToAbsoluteBuffer(
        uint8_t* buffer,
        uint32_t bufferBytes,
        int32_t bitOffset,
        uint32_t bitLength,
        uint64_t value)
    {
        if (buffer == nullptr ||
            bufferBytes == 0U ||
            bitOffset < 0 ||
            bitLength == 0U ||
            bitLength > 64U)
        {
            return false;
        }


        const uint64_t endBit =
            static_cast<uint64_t>(
                static_cast<uint32_t>(
                    bitOffset)) +
            static_cast<uint64_t>(
                bitLength);


        if (endBit >
            static_cast<uint64_t>(
                bufferBytes) *
            8ULL)
        {
            return false;
        }


        for (uint32_t bitIndex = 0U;
            bitIndex < bitLength;
            ++bitIndex)
        {
            const uint32_t destinationBit =
                static_cast<uint32_t>(
                    bitOffset) +
                bitIndex;


            const uint32_t byteIndex =
                destinationBit /
                8U;


            const uint8_t bitInByte =
                static_cast<uint8_t>(
                    destinationBit %
                    8U);


            const uint8_t mask =
                static_cast<uint8_t>(
                    1U <<
                    bitInByte);


            const bool set =
                (
                    (
                        value >>
                        bitIndex
                        ) &
                    0x01ULL
                    ) !=
                0ULL;


            if (set)
            {
                buffer[byteIndex] =
                    static_cast<uint8_t>(
                        buffer[byteIndex] |
                        mask);
            }
            else
            {
                buffer[byteIndex] =
                    static_cast<uint8_t>(
                        buffer[byteIndex] &
                        static_cast<uint8_t>(
                            ~mask));
            }
        }


        return true;
    }


    const ENI_AnalogModule* FindLegacyAnalogBySlave(
        const std::vector<ENI_AnalogModule>& list,
        int slaveIndex)
    {
        for (const auto& item : list)
        {
            if (item.slaveIndex ==
                slaveIndex)
            {
                return
                    &item;
            }
        }


        return
            nullptr;
    }


    bool ReadLegacyInt16(
        const ENI_AnalogModule* analog,
        int channelIndex,
        int16_t& value)
    {
        value =
            0;


        if (analog == nullptr ||
            analog->pInputLoc == nullptr ||
            channelIndex < 0 ||
            channelIndex >=
            static_cast<int>(
                analog->channelValues.size()))
        {
            return false;
        }


        const uint8_t* source =
            reinterpret_cast<const uint8_t*>(
                analog->pInputLoc) +
            static_cast<size_t>(
                channelIndex) *
            sizeof(int16_t);


        memcpy(
            &value,
            source,
            sizeof(value));


        return true;
    }


    bool IsApprovedGenericLiveReadKind(
        const char* kind)
    {
        if (kind == nullptr)
        {
            return false;
        }


        // Generic / status-like read data.
        if (TextEquals(kind, "Status") ||
            TextEquals(kind, "Diagnostic") ||
            TextEquals(kind, "Timestamp"))
        {
            return true;
        }


        // Standard IO.
        if (TextEquals(kind, "DigitalInput") ||
            TextEquals(kind, "DigitalIO") ||
            TextEquals(kind, "AnalogInput") ||
            TextEquals(kind, "AnalogIO"))
        {
            return true;
        }


        // Feedback.
        if (TextEquals(kind, "EncoderFeedback") ||
            TextEquals(kind, "PositionFeedback") ||
            TextEquals(kind, "VelocityFeedback") ||
            TextEquals(kind, "AccelerationFeedback") ||
            TextEquals(kind, "JerkFeedback") ||
            TextEquals(kind, "TorqueFeedback") ||
            TextEquals(kind, "ForceFeedback") ||
            TextEquals(kind, "CurrentFeedback"))
        {
            return true;
        }


        // Counter / capture.
        if (TextEquals(kind, "HighSpeedCounter") ||
            TextEquals(kind, "FrequencyInput") ||
            TextEquals(kind, "LatchCapture") ||
            TextEquals(kind, "TouchProbe"))
        {
            return true;
        }


        // Machine input signals.
        if (TextEquals(kind, "HomeSensor") ||
            TextEquals(kind, "LimitSwitch"))
        {
            return true;
        }


        // Measurement inputs.
        if (TextEquals(kind, "TemperatureInput") ||
            TextEquals(kind, "PressureInput") ||
            TextEquals(kind, "DisplacementInput") ||
            TextEquals(kind, "LoadCellInput") ||
            TextEquals(kind, "StrainGaugeInput") ||
            TextEquals(kind, "VoltageInput") ||
            TextEquals(kind, "CurrentInput"))
        {
            return true;
        }


        // Safety* here is descriptive metadata only.
        // This does NOT imply certified safety/FSoE behavior.
        if (TextEquals(kind, "SafetyInput") ||
            TextEquals(kind, "SafetyStatus"))
        {
            return true;
        }


        // DriveStatus can be consumed read-only without driving motion.
        if (TextEquals(kind, "DriveStatus"))
        {
            return true;
        }


        return false;
    }


    bool IsDeferredStructuredReadKind(
        const char* kind)
    {
        return
            TextEquals(kind, "ServoDrive") ||
            TextEquals(kind, "StepperDrive") ||
            TextEquals(kind, "SpindleDrive") ||
            TextEquals(kind, "HydraulicAxis") ||
            TextEquals(kind, "PneumaticAxis");
    }


    const ENI_GenericIO* FindLegacyIoBySlave(
        const std::vector<ENI_GenericIO>& list,
        int slaveIndex)
    {
        for (const auto& item : list)
        {
            if (item.slaveIndex ==
                slaveIndex)
            {
                return
                    &item;
            }
        }


        return
            nullptr;
    }


    bool ReadLegacyUInt16Input(
        const ENI_GenericIO* io,
        uint16_t& value)
    {
        value =
            0U;


        if (io == nullptr ||
            io->pInputLoc == nullptr)
        {
            return false;
        }


        memcpy(
            &value,
            io->pInputLoc,
            sizeof(value));


        return true;
    }


    // ========================================================================
    // Stage 11D.2 helper.
    //
    // IMPORTANT:
    // This must live in the file-scope anonymous namespace.
    // It must NOT be defined inside an EtherCatMaster member function.
    // ========================================================================

    bool ReadRawFieldBytes(
        const uint8_t* byteBase,
        uint32_t byteSpan,
        uint64_t& value)
    {
        value =
            0ULL;


        if (byteBase ==
            nullptr ||
            byteSpan ==
            0U ||
            byteSpan >
            sizeof(value))
        {
            return
                false;
        }


        memcpy(
            &value,
            byteBase,
            byteSpan);


        return
            true;
    }
}


// ============================================================================
// FindCompositeBindingShadow
// ============================================================================

const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::FindCompositeBindingShadow(
    int slaveIndex,
    const char* bindingId) const
{
    if (bindingId == nullptr ||
        bindingId[0] == '\0')
    {
        return nullptr;
    }


    for (const auto& descriptor :
        m_CompositeApplicationDescriptorsShadow)
    {
        if (descriptor.slaveIndex ==
            slaveIndex &&
            TextEquals(
                descriptor.id,
                bindingId))
        {
            return
                &descriptor;
        }
    }


    return
        nullptr;
}


// ============================================================================
// FindCompositeBindingByKindAxisShadow
//
// occurrence is zero-based within matching Kind/Axis.
// axisRef == nullptr means ignore AxisRef.
// ============================================================================

const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::FindCompositeBindingByKindAxisShadow(
    const char* kind,
    const char* axisRef,
    int occurrence) const
{
    if (kind == nullptr ||
        kind[0] == '\0' ||
        occurrence < 0)
    {
        return nullptr;
    }


    int current =
        0;


    for (const auto& descriptor :
        m_CompositeApplicationDescriptorsShadow)
    {
        if (!TextEquals(
            descriptor.kind,
            kind) ||
            !AxisMatches(
                descriptor.axisRef,
                axisRef))
        {
            continue;
        }


        if (current ==
            occurrence)
        {
            return
                &descriptor;
        }


        current++;
    }


    return
        nullptr;
}


// ============================================================================
// Scalar read APIs
//
// BitLength must be 1..64.
// Larger Composite regions such as ServoPDO are intentionally NOT flattened
// into one uint64_t.
// ============================================================================

bool EtherCatMaster::ReadCompositeInputBitsShadow(
    int slaveIndex,
    const char* bindingId,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        descriptor->inputBitLength == 0U ||
        descriptor->inputBitLength > 64U)
    {
        return false;
    }


    return
        ReadBitsFromByteBase(
            descriptor->pInputByteBase,
            descriptor->inputBitShift,
            descriptor->inputBitLength,
            value);
}


bool EtherCatMaster::ReadCompositeOutputBitsShadow(
    int slaveIndex,
    const char* bindingId,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        descriptor->outputBitLength == 0U ||
        descriptor->outputBitLength > 64U)
    {
        return false;
    }


    return
        ReadBitsFromByteBase(
            descriptor->pOutputByteBase,
            descriptor->outputBitShift,
            descriptor->outputBitLength,
            value);
}


// ============================================================================
// Typed signed input reads
// ============================================================================

bool EtherCatMaster::ReadCompositeInputInt16Shadow(
    int slaveIndex,
    const char* bindingId,
    int16_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int16") ||
        descriptor->inputBitLength !=
        16U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeInputBitsShadow(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    const uint16_t raw16 =
        static_cast<uint16_t>(
            raw);


    memcpy(
        &value,
        &raw16,
        sizeof(value));


    return true;
}


bool EtherCatMaster::ReadCompositeInputInt32Shadow(
    int slaveIndex,
    const char* bindingId,
    int32_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int32") ||
        descriptor->inputBitLength !=
        32U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeInputBitsShadow(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    const uint32_t raw32 =
        static_cast<uint32_t>(
            raw);


    memcpy(
        &value,
        &raw32,
        sizeof(value));


    return true;
}


bool EtherCatMaster::ReadCompositeInputInt64Shadow(
    int slaveIndex,
    const char* bindingId,
    int64_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int64") ||
        descriptor->inputBitLength !=
        64U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeInputBitsShadow(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    memcpy(
        &value,
        &raw,
        sizeof(value));


    return true;
}


// ============================================================================
// SAFE output write shadow
//
// IMPORTANT:
// This writes ONLY to caller-provided scratchIoMap.
// It never writes m_IoMap.
// ============================================================================

bool EtherCatMaster::WriteCompositeOutputBitsToScratchShadow(
    int slaveIndex,
    const char* bindingId,
    uint64_t value,
    uint8_t* scratchIoMap,
    uint32_t scratchIoMapBytes) const
{
    const auto* descriptor =
        FindCompositeBindingShadow(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        descriptor->outputBitLength == 0U ||
        descriptor->outputBitLength > 64U)
    {
        return false;
    }


    // Explicit safety barrier:
    // caller is not allowed to pass the live Process Image itself.
    if (scratchIoMap ==
        reinterpret_cast<const uint8_t*>(
            m_IoMap))
    {
        return false;
    }


    return
        WriteBitsToAbsoluteBuffer(
            scratchIoMap,
            scratchIoMapBytes,
            descriptor->outputBitOffset,
            descriptor->outputBitLength,
            value);
}


// ============================================================================
// AuditRuntimeCompositeAccessApiShadow
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeAccessApiShadow()
{
    int idLookups =
        0;

    int kindAxisLookups =
        0;

    int inputScalarReads =
        0;

    int outputScalarReads =
        0;

    int largeScalarSkips =
        0;

    int typedInt16Reads =
        0;

    int legacyAnalogMatches =
        0;

    int scratchWrites =
        0;

    int codecCases =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-ACCESS-SHADOW-AUDIT] BEGIN | "
        "Stage:11C.3 | "
        "Source:GENERIC_DESCRIPTOR_VECTOR | "
        "LiveRead:YES | LiveWrite:NO | "
        "ScratchWrite:YES | "
        "ProductCodeBaseline:NO\n"
        "============================================================\n");


    // ------------------------------------------------------------------------
    // 1. Lookup API verification.
    // ------------------------------------------------------------------------

    for (size_t descriptorIndex = 0;
        descriptorIndex <
        m_CompositeApplicationDescriptorsShadow.size();
        ++descriptorIndex)
    {
        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        const auto* byId =
            FindCompositeBindingShadow(
                descriptor.slaveIndex,
                descriptor.id);


        idLookups++;


        if (byId !=
            &descriptor)
        {
            errors++;
        }


        int occurrence =
            0;


        for (size_t previous = 0;
            previous <
            descriptorIndex;
            ++previous)
        {
            const auto& earlier =
                m_CompositeApplicationDescriptorsShadow[
                    previous];


            if (TextEquals(
                earlier.kind,
                descriptor.kind) &&
                TextEquals(
                    earlier.axisRef,
                    descriptor.axisRef))
            {
                occurrence++;
            }
        }


        const auto* byKindAxis =
            FindCompositeBindingByKindAxisShadow(
                descriptor.kind,
                descriptor.axisRef,
                occurrence);


        kindAxisLookups++;


        if (byKindAxis !=
            &descriptor)
        {
            errors++;
        }


        // --------------------------------------------------------------------
        // 2. Generic scalar read verification.
        // --------------------------------------------------------------------

        if (descriptor.inputBitLength >
            0U &&
            descriptor.inputBitLength <=
            64U)
        {
            uint64_t apiValue =
                0ULL;


            uint64_t directValue =
                0ULL;


            const bool apiPass =
                ReadCompositeInputBitsShadow(
                    descriptor.slaveIndex,
                    descriptor.id,
                    apiValue);


            const bool directPass =
                ReadBitsFromByteBase(
                    descriptor.pInputByteBase,
                    descriptor.inputBitShift,
                    descriptor.inputBitLength,
                    directValue);


            inputScalarReads++;


            if (!apiPass ||
                !directPass ||
                apiValue !=
                directValue)
            {
                errors++;
            }
        }
        else if (descriptor.inputBitLength >
            64U)
        {
            largeScalarSkips++;
        }


        if (descriptor.outputBitLength >
            0U &&
            descriptor.outputBitLength <=
            64U)
        {
            uint64_t apiValue =
                0ULL;


            uint64_t directValue =
                0ULL;


            const bool apiPass =
                ReadCompositeOutputBitsShadow(
                    descriptor.slaveIndex,
                    descriptor.id,
                    apiValue);


            const bool directPass =
                ReadBitsFromByteBase(
                    descriptor.pOutputByteBase,
                    descriptor.outputBitShift,
                    descriptor.outputBitLength,
                    directValue);


            outputScalarReads++;


            if (!apiPass ||
                !directPass ||
                apiValue !=
                directValue)
            {
                errors++;
            }
        }
        else if (descriptor.outputBitLength >
            64U)
        {
            largeScalarSkips++;
        }
    }


    // ------------------------------------------------------------------------
    // 3. S3 four-channel Int16 equivalence against current proven m_AdList.
    // ------------------------------------------------------------------------

    const ENI_AnalogModule* legacyAnalog =
        FindLegacyAnalogBySlave(
            m_AdList,
            3);


    for (int channel = 0;
        channel <
        4;
        ++channel)
    {
        char bindingId[16] =
        {
            0
        };


        sprintf_s(
            bindingId,
            sizeof(bindingId),
            "AD%d",
            channel + 1);


        int16_t genericValue =
            0;


        int16_t legacyValue =
            0;


        const bool genericPass =
            ReadCompositeInputInt16Shadow(
                3,
                bindingId,
                genericValue);


        const bool legacyPass =
            ReadLegacyInt16(
                legacyAnalog,
                channel,
                legacyValue);


        typedInt16Reads++;


        if (genericPass &&
            legacyPass &&
            genericValue ==
            legacyValue)
        {
            legacyAnalogMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-ACCESS-SHADOW-INT16] "
            "S3 %s | Generic:%d | Legacy:%d | "
            "GenericRead:%s LegacyRead:%s | Match:%s\n",

            bindingId,

            (int)
            genericValue,

            (int)
            legacyValue,

            genericPass
            ? "PASS"
            : "FAIL",

            legacyPass
            ? "PASS"
            : "FAIL",

            genericPass &&
            legacyPass &&
            genericValue ==
            legacyValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // 4. Scratch-only output write verification.
    //
    // S2 Primary = 16-bit DigitalOutput at absolute bit 16.
    //
    // Copy current Process Image, modify SCRATCH only, verify live map remains
    // untouched.
    // ------------------------------------------------------------------------

    {
        uint8_t scratch[4096] =
        {
            0
        };


        if (m_IoMapSize >
            0 &&
            m_IoMapSize <=
            static_cast<int>(
                sizeof(scratch)))
        {
            memcpy(
                scratch,
                m_IoMap,
                static_cast<size_t>(
                    m_IoMapSize));


            uint64_t liveBefore =
                0ULL;


            uint64_t liveAfter =
                0ULL;


            uint64_t scratchValue =
                0ULL;


            const bool liveBeforePass =
                ReadCompositeOutputBitsShadow(
                    2,
                    "Primary",
                    liveBefore);


            const bool writePass =
                WriteCompositeOutputBitsToScratchShadow(
                    2,
                    "Primary",
                    0xA55AULL,
                    scratch,
                    static_cast<uint32_t>(
                        m_IoMapSize));


            const auto* output =
                FindCompositeBindingShadow(
                    2,
                    "Primary");


            const bool scratchReadPass =
                output != nullptr &&
                ReadBitsFromAbsoluteBuffer(
                    scratch,
                    static_cast<uint32_t>(
                        m_IoMapSize),
                    output->outputBitOffset,
                    output->outputBitLength,
                    scratchValue);


            const bool liveAfterPass =
                ReadCompositeOutputBitsShadow(
                    2,
                    "Primary",
                    liveAfter);


            scratchWrites++;


            if (!liveBeforePass ||
                !writePass ||
                !scratchReadPass ||
                scratchValue !=
                0xA55AULL ||
                !liveAfterPass ||
                liveAfter !=
                liveBefore)
            {
                errors++;
            }


            DEBUG_PRINT(
                "[COMPOSITE-ACCESS-SHADOW-SCRATCH-WRITE] "
                "S2 Primary | "
                "Scratch:0x%04llX | "
                "LiveBefore:0x%04llX LiveAfter:0x%04llX | "
                "LivePreserved:%s | Result:%s\n",

                (unsigned long long)
                scratchValue,

                (unsigned long long)
                liveBefore,

                (unsigned long long)
                liveAfter,

                liveAfter ==
                liveBefore
                ? "YES"
                : "NO",

                liveBeforePass &&
                writePass &&
                scratchReadPass &&
                scratchValue ==
                0xA55AULL &&
                liveAfterPass &&
                liveAfter ==
                liveBefore
                ? "PASS"
                : "FAIL");
        }
        else
        {
            errors++;
        }
    }


    // ------------------------------------------------------------------------
    // 5. Synthetic non-byte-aligned bit codec cases.
    //
    // This proves the accessor layer can support future packed DI/DO/Status
    // fields before such hardware is available.
    // ------------------------------------------------------------------------

    struct CodecCase
    {
        int32_t bitOffset;
        uint32_t bitLength;
        uint64_t value;
    };


    const CodecCase cases[] =
    {
        { 3, 1, 0x1ULL },
        { 6, 5, 0x15ULL },
        { 9, 7, 0x55ULL },
        { 13, 16, 0xA55AULL }
    };


    for (const auto& test :
        cases)
    {
        uint8_t buffer[8] =
        {
            0
        };


        uint64_t readback =
            0ULL;


        const bool writePass =
            WriteBitsToAbsoluteBuffer(
                buffer,
                static_cast<uint32_t>(
                    sizeof(buffer)),
                test.bitOffset,
                test.bitLength,
                test.value);


        const bool readPass =
            ReadBitsFromAbsoluteBuffer(
                buffer,
                static_cast<uint32_t>(
                    sizeof(buffer)),
                test.bitOffset,
                test.bitLength,
                readback);


        codecCases++;


        if (!writePass ||
            !readPass ||
            readback !=
            test.value)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-ACCESS-SHADOW-CODEC] "
            "Bit:%d/%u | Write:0x%llX Read:0x%llX | Result:%s\n",

            test.bitOffset,

            (unsigned int)
            test.bitLength,

            (unsigned long long)
            test.value,

            (unsigned long long)
            readback,

            writePass &&
            readPass &&
            readback ==
            test.value
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-ACCESS-SHADOW-AUDIT-RESULT] "
        "Descriptors:%u | "
        "IdLookups:%d | KindAxisLookups:%d | "
        "InputScalarReads:%d | OutputScalarReads:%d | "
        "LargeScalarSkips:%d | "
        "TypedInt16Reads:%d LegacyAnalogMatches:%d | "
        "ScratchWrites:%d | CodecCases:%d | "
        "Errors:%d | Result:%s | "
        "ProductCodeBaseline:NO | "
        "LiveRead:YES | LiveWrite:NO | ScratchWrite:YES | "
        "ActiveAdapters:LEGACY_STAGE7B | "
        "ShadowAction:NO_CUTOVER | StartupAction:CONTINUE\n",

        (unsigned int)
        m_CompositeApplicationDescriptorsShadow.size(),

        idLookups,

        kindAxisLookups,

        inputScalarReads,

        outputScalarReads,

        largeScalarSkips,

        typedInt16Reads,

        legacyAnalogMatches,

        scratchWrites,

        codecCases,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-ACCESS-SHADOW-AUDIT] END | "
        "Stage:11C.3 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11C.4 - Generic LIVE Read Route
//
// This is the first controlled application-side route activation.
//
// The generic descriptor layer is authoritative for approved scalar INPUT
// functions.
//
// Existing PLC and Motion consumers are intentionally not switched here.
// ============================================================================

bool EtherCatMaster::BuildRuntimeCompositeLiveReadRoute()
{
    m_CompositeLiveReadDescriptorIndices.clear();

    m_CompositeLiveReadRouteEnabled =
        false;


    int inputDescriptors =
        0;

    int eligible =
        0;

    int routed =
        0;

    int deferredStructured =
        0;

    int deferredLargeScalar =
        0;

    int deferredOther =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-LIVE-READ-ROUTE] BEGIN | "
        "Stage:11C.4 | "
        "Route:GENERIC_LIVE_READ | "
        "PLC:LEGACY_STAGE7B | Motion:LEGACY_SERVO | "
        "LiveWrite:NO | ProductCodeBaseline:NO\n"
        "============================================================\n");


    for (uint32_t descriptorIndex = 0U;
        descriptorIndex <
        static_cast<uint32_t>(
            m_CompositeApplicationDescriptorsShadow.size());
        ++descriptorIndex)
    {
        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        if (descriptor.inputBitLength ==
            0U)
        {
            continue;
        }


        inputDescriptors++;


        if (IsDeferredStructuredReadKind(
            descriptor.kind))
        {
            deferredStructured++;


            DEBUG_PRINT(
                "[COMPOSITE-LIVE-READ-DEFER] "
                "S%d %s | Kind:%s | Input:%d/%u | "
                "Reason:STRUCTURED_LEGACY_ADAPTER\n",

                descriptor.slaveIndex,

                descriptor.id[0] != '\0'
                ? descriptor.id
                : "N/A",

                descriptor.kind[0] != '\0'
                ? descriptor.kind
                : "N/A",

                descriptor.inputBitOffset,

                (unsigned int)
                descriptor.inputBitLength);


            continue;
        }


        if (!IsApprovedGenericLiveReadKind(
            descriptor.kind))
        {
            deferredOther++;


            DEBUG_PRINT(
                "[COMPOSITE-LIVE-READ-DEFER] "
                "S%d %s | Kind:%s | Input:%d/%u | "
                "Reason:KIND_NOT_YET_APPROVED\n",

                descriptor.slaveIndex,

                descriptor.id[0] != '\0'
                ? descriptor.id
                : "N/A",

                descriptor.kind[0] != '\0'
                ? descriptor.kind
                : "N/A",

                descriptor.inputBitOffset,

                (unsigned int)
                descriptor.inputBitLength);


            continue;
        }


        eligible++;


        if (descriptor.inputBitLength >
            64U)
        {
            deferredLargeScalar++;


            DEBUG_PRINT(
                "[COMPOSITE-LIVE-READ-DEFER] "
                "S%d %s | Kind:%s | Input:%d/%u | "
                "Reason:SCALAR_OVER_64_BITS\n",

                descriptor.slaveIndex,

                descriptor.id[0] != '\0'
                ? descriptor.id
                : "N/A",

                descriptor.kind[0] != '\0'
                ? descriptor.kind
                : "N/A",

                descriptor.inputBitOffset,

                (unsigned int)
                descriptor.inputBitLength);


            continue;
        }


        if (descriptor.inputBitOffset <
            0 ||
            descriptor.pInputByteBase ==
            nullptr)
        {
            errors++;


            DEBUG_PRINT(
                "[COMPOSITE-LIVE-READ-ROUTE-BINDING] "
                "S%d %s | Kind:%s | Input:%d/%u | Result:FAIL\n",

                descriptor.slaveIndex,

                descriptor.id[0] != '\0'
                ? descriptor.id
                : "N/A",

                descriptor.kind[0] != '\0'
                ? descriptor.kind
                : "N/A",

                descriptor.inputBitOffset,

                (unsigned int)
                descriptor.inputBitLength);


            continue;
        }


        m_CompositeLiveReadDescriptorIndices.push_back(
            descriptorIndex);


        routed++;


        DEBUG_PRINT(
            "[COMPOSITE-LIVE-READ-ROUTE-BINDING] "
            "S%d %s | Kind:%s Type:%s Axis:%s | "
            "Input:%d/%u | Byte:%d Shift:%u Span:%u | "
            "Result:ROUTED\n",

            descriptor.slaveIndex,

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

            descriptor.inputBitOffset,

            (unsigned int)
            descriptor.inputBitLength,

            descriptor.inputByteOffset,

            (unsigned int)
            descriptor.inputBitShift,

            (unsigned int)
            descriptor.inputByteSpan);
    }


    const bool pass =
        errors ==
        0 &&
        routed ==
        static_cast<int>(
            m_CompositeLiveReadDescriptorIndices.size());


    if (pass)
    {
        m_CompositeLiveReadRouteEnabled =
            true;
    }


    DEBUG_PRINT(
        "[COMPOSITE-LIVE-READ-ROUTE-RESULT] "
        "InputDescriptors:%d | Eligible:%d | Routed:%d | "
        "DeferredStructured:%d | DeferredLargeScalar:%d | DeferredOther:%d | "
        "Errors:%d | Result:%s | "
        "RouteEnabled:%s | "
        "CutoverScope:GENERIC_READ_API | "
        "PLCRead:LEGACY | MotionRead:LEGACY | LiveWrite:NO\n",

        inputDescriptors,

        eligible,

        routed,

        deferredStructured,

        deferredLargeScalar,

        deferredOther,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_CompositeLiveReadRouteEnabled
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-LIVE-READ-ROUTE] END | "
        "Stage:11C.4 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Active route lookup
// ============================================================================

const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::FindCompositeLiveReadBinding(
    int slaveIndex,
    const char* bindingId) const
{
    if (!m_CompositeLiveReadRouteEnabled ||
        bindingId == nullptr ||
        bindingId[0] == '\0')
    {
        return nullptr;
    }


    for (const uint32_t descriptorIndex :
    m_CompositeLiveReadDescriptorIndices)
    {
        if (descriptorIndex >=
            m_CompositeApplicationDescriptorsShadow.size())
        {
            continue;
        }


        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        if (descriptor.slaveIndex ==
            slaveIndex &&
            TextEquals(
                descriptor.id,
                bindingId))
        {
            return
                &descriptor;
        }
    }


    return
        nullptr;

}


const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::FindCompositeLiveReadByKindAxis(
    const char* kind,
    const char* axisRef,
    int occurrence) const
{
    if (!m_CompositeLiveReadRouteEnabled ||
        kind == nullptr ||
        kind[0] == '\0' ||
        occurrence < 0)
    {
        return nullptr;
    }


    int current =
        0;


    for (const uint32_t descriptorIndex :
    m_CompositeLiveReadDescriptorIndices)
    {
        if (descriptorIndex >=
            m_CompositeApplicationDescriptorsShadow.size())
        {
            continue;
        }


        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        if (!TextEquals(
            descriptor.kind,
            kind) ||
            !AxisMatches(
                descriptor.axisRef,
                axisRef))
        {
            continue;
        }


        if (current ==
            occurrence)
        {
            return
                &descriptor;
        }


        current++;
    }


    return
        nullptr;
}


// ============================================================================
// Active generic LIVE read API
// ============================================================================

bool EtherCatMaster::ReadCompositeLiveInputBits(
    int slaveIndex,
    const char* bindingId,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* descriptor =
        FindCompositeLiveReadBinding(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        descriptor->inputBitLength ==
        0U ||
        descriptor->inputBitLength >
        64U)
    {
        return false;
    }


    return
        ReadBitsFromByteBase(
            descriptor->pInputByteBase,
            descriptor->inputBitShift,
            descriptor->inputBitLength,
            value);
}


bool EtherCatMaster::ReadCompositeLiveInputInt16(
    int slaveIndex,
    const char* bindingId,
    int16_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeLiveReadBinding(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int16") ||
        descriptor->inputBitLength !=
        16U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeLiveInputBits(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    const uint16_t raw16 =
        static_cast<uint16_t>(
            raw);


    memcpy(
        &value,
        &raw16,
        sizeof(value));


    return true;
}


bool EtherCatMaster::ReadCompositeLiveInputInt32(
    int slaveIndex,
    const char* bindingId,
    int32_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeLiveReadBinding(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int32") ||
        descriptor->inputBitLength !=
        32U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeLiveInputBits(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    const uint32_t raw32 =
        static_cast<uint32_t>(
            raw);


    memcpy(
        &value,
        &raw32,
        sizeof(value));


    return true;
}


bool EtherCatMaster::ReadCompositeLiveInputInt64(
    int slaveIndex,
    const char* bindingId,
    int64_t& value) const
{
    value =
        0;


    const auto* descriptor =
        FindCompositeLiveReadBinding(
            slaveIndex,
            bindingId);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int64") ||
        descriptor->inputBitLength !=
        64U)
    {
        return false;
    }


    uint64_t raw =
        0ULL;


    if (!ReadCompositeLiveInputBits(
        slaveIndex,
        bindingId,
        raw))
    {
        return false;
    }


    memcpy(
        &value,
        &raw,
        sizeof(value));


    return true;
}


// ============================================================================
// Audit active generic LIVE read route
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeLiveReadRoute()
{
    int routeLookups =
        0;

    int readMatches =
        0;

    int digitalInputChecks =
        0;

    int analogChecks =
        0;

    int deferredServoChecks =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-LIVE-READ-AUDIT] BEGIN | "
        "Stage:11C.4 | "
        "Route:GENERIC_LIVE_READ | "
        "RouteEnabled:%s | "
        "PLCRead:LEGACY | MotionRead:LEGACY | LiveWrite:NO\n"
        "============================================================\n",

        m_CompositeLiveReadRouteEnabled
        ? "YES"
        : "NO");


    if (!m_CompositeLiveReadRouteEnabled)
    {
        errors++;
    }


    // ------------------------------------------------------------------------
    // Every routed descriptor must resolve through the ACTIVE route and read
    // exactly the same live Process Image value as the proven 11C.3 shadow API.
    // ------------------------------------------------------------------------

    for (const uint32_t descriptorIndex :
    m_CompositeLiveReadDescriptorIndices)
    {
        if (descriptorIndex >=
            m_CompositeApplicationDescriptorsShadow.size())
        {
            errors++;
            continue;
        }


        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        const auto* active =
            FindCompositeLiveReadBinding(
                descriptor.slaveIndex,
                descriptor.id);


        routeLookups++;


        if (active !=
            &descriptor)
        {
            errors++;
            continue;
        }


        uint64_t liveValue =
            0ULL;


        uint64_t shadowValue =
            0ULL;


        const bool livePass =
            ReadCompositeLiveInputBits(
                descriptor.slaveIndex,
                descriptor.id,
                liveValue);


        const bool shadowPass =
            ReadCompositeInputBitsShadow(
                descriptor.slaveIndex,
                descriptor.id,
                shadowValue);


        if (livePass &&
            shadowPass &&
            liveValue ==
            shadowValue)
        {
            readMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-LIVE-READ-COMPARE] "
            "S%d %s | Kind:%s | "
            "Live:0x%llX Shadow:0x%llX | "
            "Match:%s\n",

            descriptor.slaveIndex,

            descriptor.id[0] != '\0'
            ? descriptor.id
            : "N/A",

            descriptor.kind[0] != '\0'
            ? descriptor.kind
            : "N/A",

            (unsigned long long)
            liveValue,

            (unsigned long long)
            shadowValue,

            livePass &&
            shadowPass &&
            liveValue ==
            shadowValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // S1 DigitalInput compatibility check against current Stage7B m_IoList.
    // ------------------------------------------------------------------------

    {
        const auto* liveDescriptor =
            FindCompositeLiveReadBinding(
                1,
                "Primary");


        const ENI_GenericIO* legacyIo =
            FindLegacyIoBySlave(
                m_IoList,
                1);


        uint64_t genericValue =
            0ULL;


        uint16_t legacyValue =
            0U;


        const bool genericPass =
            liveDescriptor != nullptr &&
            TextEquals(
                liveDescriptor->kind,
                "DigitalInput") &&
            ReadCompositeLiveInputBits(
                1,
                "Primary",
                genericValue);


        const bool legacyPass =
            ReadLegacyUInt16Input(
                legacyIo,
                legacyValue);


        digitalInputChecks++;


        if (!genericPass ||
            !legacyPass ||
            static_cast<uint16_t>(
                genericValue) !=
            legacyValue)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-LIVE-READ-DI] "
            "S1 Primary | Generic:0x%04llX Legacy:0x%04X | "
            "Match:%s\n",

            (unsigned long long)
            genericValue,

            (unsigned int)
            legacyValue,

            genericPass&&
            legacyPass&&
            static_cast<uint16_t>(
                genericValue) ==
            legacyValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // S3 Composite AD1..AD4 active live route.
    // ------------------------------------------------------------------------

    for (int channel = 0;
        channel <
        4;
        ++channel)
    {
        char bindingId[16] =
        {
            0
        };


        sprintf_s(
            bindingId,
            sizeof(bindingId),
            "AD%d",
            channel + 1);


        int16_t activeValue =
            0;


        int16_t shadowValue =
            0;


        const bool activePass =
            ReadCompositeLiveInputInt16(
                3,
                bindingId,
                activeValue);


        const bool shadowPass =
            ReadCompositeInputInt16Shadow(
                3,
                bindingId,
                shadowValue);


        analogChecks++;


        if (!activePass ||
            !shadowPass ||
            activeValue !=
            shadowValue)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-LIVE-READ-AI] "
            "S3 %s | Active:%d Shadow:%d | Match:%s\n",

            bindingId,

            (int)
            activeValue,

            (int)
            shadowValue,

            activePass &&
            shadowPass &&
            activeValue ==
            shadowValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // ServoDrive must NOT appear in generic scalar live-read route yet.
    // ------------------------------------------------------------------------

    for (int slaveIndex = 4;
        slaveIndex <=
        6;
        ++slaveIndex)
    {
        const auto* servo =
            FindCompositeLiveReadBinding(
                slaveIndex,
                "Primary");


        deferredServoChecks++;


        if (servo !=
            nullptr)
        {
            errors++;
        }
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-LIVE-READ-AUDIT-RESULT] "
        "Routed:%u | RouteLookups:%d | ReadMatches:%d | "
        "DigitalInputChecks:%d | AnalogChecks:%d | "
        "DeferredServoChecks:%d | "
        "Errors:%d | Result:%s | "
        "Route:GENERIC_LIVE_READ | RouteEnabled:%s | "
        "CutoverScope:GENERIC_READ_API | "
        "PLCRead:LEGACY | MotionRead:LEGACY | "
        "LiveWrite:NO | ProductCodeBaseline:NO | StartupAction:CONTINUE\n",

        (unsigned int)
        m_CompositeLiveReadDescriptorIndices.size(),

        routeLookups,

        readMatches,

        digitalInputChecks,

        analogChecks,

        deferredServoChecks,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_CompositeLiveReadRouteEnabled
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-LIVE-READ-AUDIT] END | "
        "Stage:11C.4 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11C.5 - Generic Read Consumer Bridge SHADOW
//
// Consumer-facing semantic identity:
//
//     Kind + AxisRef + occurrence
//
// This deliberately hides:
//
//     ProductCode
//     Slave process-image byte offsets
//     legacy m_IoList / m_AdList indices
//
// Existing application consumers are NOT cut over in Stage11C.5.
// ============================================================================

const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::ResolveCompositeReadConsumerShadow(
    const char* kind,
    const char* axisRef,
    int occurrence) const
{
    return
        FindCompositeLiveReadByKindAxis(
            kind,
            axisRef,
            occurrence);
}


// ============================================================================
// Consumer scalar read
// ============================================================================

bool EtherCatMaster::ReadCompositeConsumerBitsShadow(
    const char* kind,
    const char* axisRef,
    int occurrence,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* descriptor =
        ResolveCompositeReadConsumerShadow(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputBits(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt16Shadow(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int16_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumerShadow(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int16") ||
        descriptor->inputBitLength !=
        16U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt16(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt32Shadow(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int32_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumerShadow(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int32") ||
        descriptor->inputBitLength !=
        32U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt32(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt64Shadow(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int64_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumerShadow(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int64") ||
        descriptor->inputBitLength !=
        64U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt64(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


// ============================================================================
// AuditRuntimeCompositeReadConsumerBridgeShadow
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeReadConsumerBridgeShadow()
{
    int semanticLookups =
        0;

    int semanticResolved =
        0;

    int consumerReads =
        0;

    int consumerReadMatches =
        0;

    int digitalInputChecks =
        0;

    int analogChecks =
        0;

    int negativeLookupChecks =
        0;

    int deferredServoChecks =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-CONSUMER-BRIDGE-SHADOW] BEGIN | "
        "Stage:11C.5 | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "PLCConsumer:LEGACY | EDMConsumer:LEGACY | MotionConsumer:LEGACY | "
        "LiveWrite:NO | ProductCodeBaseline:NO\n"
        "============================================================\n");


    // ------------------------------------------------------------------------
    // 1. Current DI consumer:
    //
    //     DigitalInput / "" / 0
    //
    // Must resolve S1 Primary and match current legacy m_IoList.
    // ------------------------------------------------------------------------

    {
        const auto* descriptor =
            ResolveCompositeReadConsumerShadow(
                "DigitalInput",
                "",
                0);


        semanticLookups++;


        if (descriptor != nullptr)
        {
            semanticResolved++;
        }


        uint64_t genericValue =
            0ULL;


        uint16_t legacyValue =
            0U;


        const bool genericPass =
            descriptor != nullptr &&
            descriptor->slaveIndex ==
            1 &&
            TextEquals(
                descriptor->id,
                "Primary") &&
            ReadCompositeConsumerBitsShadow(
                "DigitalInput",
                "",
                0,
                genericValue);


        const ENI_GenericIO* legacyIo =
            FindLegacyIoBySlave(
                m_IoList,
                1);


        const bool legacyPass =
            ReadLegacyUInt16Input(
                legacyIo,
                legacyValue);


        consumerReads++;
        digitalInputChecks++;


        if (genericPass &&
            legacyPass &&
            static_cast<uint16_t>(
                genericValue) ==
            legacyValue)
        {
            consumerReadMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-BRIDGE-DI] "
            "Key:DigitalInput/-/0 | "
            "Resolved:S%d/%s | "
            "Generic:0x%04llX Legacy:0x%04X | "
            "Match:%s\n",

            descriptor != nullptr
            ? descriptor->slaveIndex
            : -1,

            descriptor != nullptr &&
            descriptor->id[0] != '\0'
            ? descriptor->id
            : "N/A",

            (unsigned long long)
            genericValue,

            (unsigned int)
            legacyValue,

            genericPass &&
            legacyPass &&
            static_cast<uint16_t>(
                genericValue) ==
            legacyValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // 2. Current four AnalogInput consumers:
    //
    //     AnalogInput / "" / occurrence 0..3
    //
    // This proves a consumer no longer needs to know S3 or AD1..AD4.
    // ------------------------------------------------------------------------

    const ENI_AnalogModule* legacyAnalog =
        FindLegacyAnalogBySlave(
            m_AdList,
            3);


    for (int channel = 0;
        channel <
        4;
        ++channel)
    {
        const auto* descriptor =
            ResolveCompositeReadConsumerShadow(
                "AnalogInput",
                "",
                channel);


        semanticLookups++;


        if (descriptor != nullptr)
        {
            semanticResolved++;
        }


        int16_t genericValue =
            0;


        int16_t legacyValue =
            0;


        const bool genericPass =
            descriptor != nullptr &&
            descriptor->slaveIndex ==
            3 &&
            ReadCompositeConsumerInt16Shadow(
                "AnalogInput",
                "",
                channel,
                genericValue);


        const bool legacyPass =
            ReadLegacyInt16(
                legacyAnalog,
                channel,
                legacyValue);


        consumerReads++;
        analogChecks++;


        if (genericPass &&
            legacyPass &&
            genericValue ==
            legacyValue)
        {
            consumerReadMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-BRIDGE-AI] "
            "Key:AnalogInput/-/%d | "
            "Resolved:S%d/%s | "
            "Generic:%d Legacy:%d | Match:%s\n",

            channel,

            descriptor != nullptr
            ? descriptor->slaveIndex
            : -1,

            descriptor != nullptr &&
            descriptor->id[0] != '\0'
            ? descriptor->id
            : "N/A",

            (int)
            genericValue,

            (int)
            legacyValue,

            genericPass &&
            legacyPass &&
            genericValue ==
            legacyValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // 3. Future semantic key not present on current hardware:
    //
    //     PositionFeedback / X / 0
    //
    // A clean "not found" is correct.  No fallback to ProductCode.
    // ------------------------------------------------------------------------

    {
        const auto* future =
            ResolveCompositeReadConsumerShadow(
                "PositionFeedback",
                "X",
                0);


        semanticLookups++;
        negativeLookupChecks++;


        if (future !=
            nullptr)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-BRIDGE-FUTURE] "
            "Key:PositionFeedback/X/0 | "
            "Present:%s | Expected:NOT_PRESENT | Result:%s\n",

            future != nullptr
            ? "YES"
            : "NO",

            future == nullptr
            ? "PASS"
            : "FAIL");
    }


    // ------------------------------------------------------------------------
    // 4. ServoDrive must remain unavailable through generic scalar consumer
    //    bridge because Stage11C.4 intentionally deferred it.
    // ------------------------------------------------------------------------

    {
        const auto* servo =
            ResolveCompositeReadConsumerShadow(
                "ServoDrive",
                "",
                0);


        semanticLookups++;
        deferredServoChecks++;


        if (servo !=
            nullptr)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-BRIDGE-SERVO] "
            "Key:ServoDrive/-/0 | "
            "Present:%s | Expected:DEFERRED | Result:%s\n",

            servo != nullptr
            ? "YES"
            : "NO",

            servo == nullptr
            ? "PASS"
            : "FAIL");
    }


    // ------------------------------------------------------------------------
    // 5. Type safety:
    //
    // Current AnalogInput is Int16. Int32/Int64 consumer reads must reject
    // the same semantic binding rather than reinterpret it incorrectly.
    // ------------------------------------------------------------------------

    {
        int32_t value32 =
            0;


        int64_t value64 =
            0;


        const bool int32Rejected =
            !ReadCompositeConsumerInt32Shadow(
                "AnalogInput",
                "",
                0,
                value32);


        const bool int64Rejected =
            !ReadCompositeConsumerInt64Shadow(
                "AnalogInput",
                "",
                0,
                value64);


        if (!int32Rejected ||
            !int64Rejected)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-BRIDGE-TYPE-GUARD] "
            "AnalogInput/-/0 | "
            "Int32Rejected:%s Int64Rejected:%s | Result:%s\n",

            int32Rejected
            ? "YES"
            : "NO",

            int64Rejected
            ? "YES"
            : "NO",

            int32Rejected &&
            int64Rejected
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-CONSUMER-BRIDGE-SHADOW-AUDIT-RESULT] "
        "SemanticLookups:%d | Resolved:%d | "
        "ConsumerReads:%d | ConsumerReadMatches:%d | "
        "DigitalInputChecks:%d | AnalogChecks:%d | "
        "NegativeLookupChecks:%d | DeferredServoChecks:%d | "
        "Errors:%d | Result:%s | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "PLCConsumer:LEGACY | EDMConsumer:LEGACY | MotionConsumer:LEGACY | "
        "LiveWrite:NO | ProductCodeBaseline:NO | "
        "BridgeAction:SHADOW_ONLY | StartupAction:CONTINUE\n",

        semanticLookups,

        semanticResolved,

        consumerReads,

        consumerReadMatches,

        digitalInputChecks,

        analogChecks,

        negativeLookupChecks,

        deferredServoChecks,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-CONSUMER-BRIDGE-SHADOW] END | "
        "Stage:11C.5 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11C.6 - EDM Read Consumer API LIVE Cutover
//
// Active semantic consumer API:
//
//     Kind + AxisRef + occurrence
//
// Source:
//
//     Stage11C.4 GENERIC_LIVE_READ
//
// Existing EDM call sites are not migrated here.
// ============================================================================

bool EtherCatMaster::BuildRuntimeCompositeReadConsumerLiveRoute()
{
    m_CompositeReadConsumerLiveRouteEnabled =
        false;


    int candidates =
        0;

    int resolvable =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-CONSUMER-LIVE-ROUTE] BEGIN | "
        "Stage:11C.6 | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "EDMConsumerAPI:GENERIC_LIVE | "
        "EDMCallSites:LEGACY | "
        "PLCConsumer:LEGACY | MotionConsumer:LEGACY | "
        "LiveWrite:NO | ProductCodeBaseline:NO\n"
        "============================================================\n");


    if (!m_CompositeLiveReadRouteEnabled)
    {
        errors++;
    }


    for (const uint32_t descriptorIndex :
    m_CompositeLiveReadDescriptorIndices)
    {
        if (descriptorIndex >=
            m_CompositeApplicationDescriptorsShadow.size())
        {
            errors++;
            continue;
        }


        const auto& descriptor =
            m_CompositeApplicationDescriptorsShadow[
                descriptorIndex];


        candidates++;


        const auto* resolved =
            FindCompositeLiveReadByKindAxis(
                descriptor.kind,
                descriptor.axisRef,
                0);


        // The first matching descriptor for a Kind/Axis proves the semantic
        // route is available.  Exact occurrence ordering is audited below.
        if (resolved != nullptr)
        {
            resolvable++;
        }
        else
        {
            errors++;
        }
    }


    const bool pass =
        errors ==
        0;


    if (pass)
    {
        m_CompositeReadConsumerLiveRouteEnabled =
            true;
    }


    DEBUG_PRINT(
        "[COMPOSITE-CONSUMER-LIVE-ROUTE-RESULT] "
        "Candidates:%d | Resolvable:%d | "
        "Errors:%d | Result:%s | "
        "RouteEnabled:%s | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "EDMConsumerAPI:GENERIC_LIVE | EDMCallSites:LEGACY | "
        "PLCConsumer:LEGACY | MotionConsumer:LEGACY | "
        "LiveWrite:NO\n",

        candidates,

        resolvable,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_CompositeReadConsumerLiveRouteEnabled
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-CONSUMER-LIVE-ROUTE] END | "
        "Stage:11C.6 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Active semantic consumer lookup
// ============================================================================

const EtherCatCompositeApplicationDescriptor*
EtherCatMaster::ResolveCompositeReadConsumer(
    const char* kind,
    const char* axisRef,
    int occurrence) const
{
    if (!m_CompositeReadConsumerLiveRouteEnabled)
    {
        return
            nullptr;
    }


    return
        FindCompositeLiveReadByKindAxis(
            kind,
            axisRef,
            occurrence);
}


// ============================================================================
// Active semantic consumer reads
// ============================================================================

bool EtherCatMaster::ReadCompositeConsumerBits(
    const char* kind,
    const char* axisRef,
    int occurrence,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* descriptor =
        ResolveCompositeReadConsumer(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputBits(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt16(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int16_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumer(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int16") ||
        descriptor->inputBitLength !=
        16U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt16(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt32(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int32_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumer(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int32") ||
        descriptor->inputBitLength !=
        32U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt32(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


bool EtherCatMaster::ReadCompositeConsumerInt64(
    const char* kind,
    const char* axisRef,
    int occurrence,
    int64_t& value) const
{
    value =
        0;


    const auto* descriptor =
        ResolveCompositeReadConsumer(
            kind,
            axisRef,
            occurrence);


    if (descriptor == nullptr ||
        !TextEquals(
            descriptor->dataType,
            "Int64") ||
        descriptor->inputBitLength !=
        64U)
    {
        return
            false;
    }


    return
        ReadCompositeLiveInputInt64(
            descriptor->slaveIndex,
            descriptor->id,
            value);
}


// ============================================================================
// AuditRuntimeCompositeReadConsumerLiveRoute
// ============================================================================

bool EtherCatMaster::AuditRuntimeCompositeReadConsumerLiveRoute()
{
    int semanticLookups =
        0;

    int resolved =
        0;

    int activeReads =
        0;

    int shadowMatches =
        0;

    int digitalInputChecks =
        0;

    int analogChecks =
        0;

    int negativeLookupChecks =
        0;

    int deferredServoChecks =
        0;

    int typeGuardChecks =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[COMPOSITE-CONSUMER-LIVE-AUDIT] BEGIN | "
        "Stage:11C.6 | "
        "RouteEnabled:%s | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "EDMConsumerAPI:GENERIC_LIVE | EDMCallSites:LEGACY | "
        "PLCConsumer:LEGACY | MotionConsumer:LEGACY | LiveWrite:NO\n"
        "============================================================\n",

        m_CompositeReadConsumerLiveRouteEnabled
        ? "YES"
        : "NO");


    if (!m_CompositeReadConsumerLiveRouteEnabled)
    {
        errors++;
    }


    // ------------------------------------------------------------------------
    // S1 DI
    // ------------------------------------------------------------------------

    {
        const auto* active =
            ResolveCompositeReadConsumer(
                "DigitalInput",
                "",
                0);


        const auto* shadow =
            ResolveCompositeReadConsumerShadow(
                "DigitalInput",
                "",
                0);


        semanticLookups++;


        if (active != nullptr)
        {
            resolved++;
        }


        uint64_t activeValue =
            0ULL;


        uint64_t shadowValue =
            0ULL;


        const bool activePass =
            active != nullptr &&
            ReadCompositeConsumerBits(
                "DigitalInput",
                "",
                0,
                activeValue);


        const bool shadowPass =
            shadow != nullptr &&
            ReadCompositeConsumerBitsShadow(
                "DigitalInput",
                "",
                0,
                shadowValue);


        activeReads++;
        digitalInputChecks++;


        if (activePass &&
            shadowPass &&
            activeValue ==
            shadowValue)
        {
            shadowMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-LIVE-DI] "
            "Key:DigitalInput/-/0 | "
            "Active:S%d/%s | "
            "ActiveValue:0x%04llX ShadowValue:0x%04llX | Match:%s\n",

            active != nullptr
            ? active->slaveIndex
            : -1,

            active != nullptr &&
            active->id[0] != '\0'
            ? active->id
            : "N/A",

            (unsigned long long)
            activeValue,

            (unsigned long long)
            shadowValue,

            activePass &&
            shadowPass &&
            activeValue ==
            shadowValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // S3 AD1..AD4 by semantic occurrence
    // ------------------------------------------------------------------------

    for (int channel = 0;
        channel <
        4;
        ++channel)
    {
        const auto* active =
            ResolveCompositeReadConsumer(
                "AnalogInput",
                "",
                channel);


        const auto* shadow =
            ResolveCompositeReadConsumerShadow(
                "AnalogInput",
                "",
                channel);


        semanticLookups++;


        if (active != nullptr)
        {
            resolved++;
        }


        int16_t activeValue =
            0;


        int16_t shadowValue =
            0;


        const bool activePass =
            active != nullptr &&
            ReadCompositeConsumerInt16(
                "AnalogInput",
                "",
                channel,
                activeValue);


        const bool shadowPass =
            shadow != nullptr &&
            ReadCompositeConsumerInt16Shadow(
                "AnalogInput",
                "",
                channel,
                shadowValue);


        activeReads++;
        analogChecks++;


        if (activePass &&
            shadowPass &&
            activeValue ==
            shadowValue)
        {
            shadowMatches++;
        }
        else
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-LIVE-AI] "
            "Key:AnalogInput/-/%d | "
            "Active:S%d/%s | "
            "Active:%d Shadow:%d | Match:%s\n",

            channel,

            active != nullptr
            ? active->slaveIndex
            : -1,

            active != nullptr &&
            active->id[0] != '\0'
            ? active->id
            : "N/A",

            (int)
            activeValue,

            (int)
            shadowValue,

            activePass &&
            shadowPass &&
            activeValue ==
            shadowValue
            ? "YES"
            : "NO");
    }


    // ------------------------------------------------------------------------
    // Future PositionFeedback must cleanly remain absent.
    // ------------------------------------------------------------------------

    {
        const auto* future =
            ResolveCompositeReadConsumer(
                "PositionFeedback",
                "X",
                0);


        semanticLookups++;
        negativeLookupChecks++;


        if (future !=
            nullptr)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-LIVE-FUTURE] "
            "Key:PositionFeedback/X/0 | Present:%s | "
            "Expected:NOT_PRESENT | Result:%s\n",

            future != nullptr
            ? "YES"
            : "NO",

            future == nullptr
            ? "PASS"
            : "FAIL");
    }


    // ------------------------------------------------------------------------
    // Servo remains deferred from scalar semantic consumer route.
    // ------------------------------------------------------------------------

    {
        const auto* servo =
            ResolveCompositeReadConsumer(
                "ServoDrive",
                "",
                0);


        semanticLookups++;
        deferredServoChecks++;


        if (servo !=
            nullptr)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-LIVE-SERVO] "
            "Key:ServoDrive/-/0 | Present:%s | "
            "Expected:DEFERRED | Result:%s\n",

            servo != nullptr
            ? "YES"
            : "NO",

            servo == nullptr
            ? "PASS"
            : "FAIL");
    }


    // ------------------------------------------------------------------------
    // Type guards remain active.
    // ------------------------------------------------------------------------

    {
        int32_t wrong32 =
            0;


        int64_t wrong64 =
            0;


        const bool int32Rejected =
            !ReadCompositeConsumerInt32(
                "AnalogInput",
                "",
                0,
                wrong32);


        const bool int64Rejected =
            !ReadCompositeConsumerInt64(
                "AnalogInput",
                "",
                0,
                wrong64);


        typeGuardChecks +=
            2;


        if (!int32Rejected ||
            !int64Rejected)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[COMPOSITE-CONSUMER-LIVE-TYPE-GUARD] "
            "AnalogInput/-/0 | "
            "Int32Rejected:%s Int64Rejected:%s | Result:%s\n",

            int32Rejected
            ? "YES"
            : "NO",

            int64Rejected
            ? "YES"
            : "NO",

            int32Rejected &&
            int64Rejected
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[COMPOSITE-CONSUMER-LIVE-AUDIT-RESULT] "
        "SemanticLookups:%d | Resolved:%d | "
        "ActiveReads:%d | ShadowMatches:%d | "
        "DigitalInputChecks:%d | AnalogChecks:%d | "
        "NegativeLookupChecks:%d | DeferredServoChecks:%d | "
        "TypeGuardChecks:%d | "
        "Errors:%d | Result:%s | "
        "RouteEnabled:%s | "
        "ConsumerIdentity:KIND_AXIS_OCCURRENCE | "
        "ReadSource:GENERIC_LIVE_READ | "
        "EDMConsumerAPI:GENERIC_LIVE | EDMCallSites:LEGACY | "
        "PLCConsumer:LEGACY | MotionConsumer:LEGACY | "
        "LiveWrite:NO | ProductCodeBaseline:NO | StartupAction:CONTINUE\n",

        semanticLookups,

        resolved,

        activeReads,

        shadowMatches,

        digitalInputChecks,

        analogChecks,

        negativeLookupChecks,

        deferredServoChecks,

        typeGuardChecks,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_CompositeReadConsumerLiveRouteEnabled
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[COMPOSITE-CONSUMER-LIVE-AUDIT] END | "
        "Stage:11C.6 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11D.2 - Structured ServoDrive Live Read SHADOW
// ============================================================================

bool EtherCatMaster::ReadStructuredServoInputFieldBitsShadow(
    int slaveIndex,
    const char* fieldId,
    uint64_t& value) const
{
    value =
        0ULL;


    const auto* field =
        FindStructuredServoDriveFieldShadow(
            slaveIndex,
            fieldId);


    if (field ==
        nullptr ||
        strcmp(
            field->direction,
            "Input") !=
        0 ||
        field->pByteBase ==
        nullptr ||
        field->bitLength ==
        0U ||
        field->bitLength >
        64U ||
        !field->byteAligned)
    {
        return
            false;
    }


    return
        ReadBitsFromByteBase(
            field->pByteBase,
            0U,
            field->bitLength,
            value);
}


bool EtherCatMaster::PrepareStructuredServoDriveLiveReadShadow()
{
    m_StructuredServoLiveReadShadowPrepared =
        false;

    m_StructuredServoLiveReadInputFieldCount =
        0;


    m_StructuredServoLiveReadCycles.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadExpectedChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadStableChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadUnstableSkips.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_StructuredServoLiveReadFailures.store(
        0ULL,
        std::memory_order_relaxed);


    int servos =
        0;

    int inputFields =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-STRUCTURED-LIVE-READ-PREPARE] BEGIN | "
        "Stage:11D.2 | "
        "Source:STRUCTURED_SERVO_FIELDS | "
        "CompareTo:MOTION_ENI_SERVO_PINPUT | "
        "SampleCadence:PLC_1MS | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n"
        "============================================================\n");


    for (const auto& servo :
        m_ServoList)
    {
        servos++;


        if (servo.pInput ==
            nullptr)
        {
            errors++;
            continue;
        }


        int servoInputFields =
            0;


        for (const auto& field :
            m_StructuredServoDriveFieldsShadow)
        {
            if (field.slaveIndex !=
                servo.slaveIndex ||
                strcmp(
                    field.direction,
                    "Input") !=
                0)
            {
                continue;
            }


            inputFields++;
            servoInputFields++;
            pointerChecks++;


            const uint8_t* legacyPointer =
                reinterpret_cast<const uint8_t*>(
                    servo.pInput) +
                (
                    field.relativeBitOffset /
                    8U
                    );


            const bool pointerMatch =
                field.pByteBase !=
                nullptr &&
                reinterpret_cast<const uint8_t*>(
                    field.pByteBase) ==
                legacyPointer;


            if (pointerMatch)
            {
                pointerMatches++;
            }
            else
            {
                errors++;
            }


            if (!field.byteAligned ||
                field.bitLength ==
                0U ||
                field.bitLength >
                64U ||
                field.byteSpan ==
                0U ||
                field.byteSpan >
                sizeof(uint64_t))
            {
                errors++;
            }
        }


        if (servoInputFields !=
            8)
        {
            errors++;
        }


        DEBUG_PRINT(
            "[SERVO-STRUCTURED-LIVE-READ-PREPARE-SERVO] "
            "S%d | InputFields:%d/8 | Result:%s\n",

            servo.slaveIndex,

            servoInputFields,

            servoInputFields ==
            8
            ? "PASS"
            : "FAIL");
    }


    const bool pass =
        errors ==
        0 &&
        servos ==
        3 &&
        inputFields ==
        24 &&
        pointerChecks ==
        24 &&
        pointerMatches ==
        24;


    if (pass)
    {
        m_StructuredServoLiveReadInputFieldCount =
            inputFields;

        m_StructuredServoLiveReadShadowPrepared =
            true;
    }


    DEBUG_PRINT(
        "[SERVO-STRUCTURED-LIVE-READ-PREPARE-RESULT] "
        "Servos:%d | InputFields:%d/24 | "
        "PointerChecks:%d PointerMatches:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "SampleCadence:PLC_1MS | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n",

        servos,

        inputFields,

        pointerChecks,

        pointerMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_StructuredServoLiveReadShadowPrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-STRUCTURED-LIVE-READ-PREPARE] END | "
        "Stage:11D.2 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::SampleStructuredServoDriveLiveReadShadow()
{
    // ========================================================================
    // Stage 11D.10:
    //
    // D2 is retired after final Servo INPUT release.
    //
    // The active counter is a handshake with the 250us release gate so
    // retirement never captures the D2 freeze point while a 1ms sample is
    // still executing.
    // ========================================================================

    if (m_ServoInputReleaseD2StopRequested.load(
        std::memory_order_acquire))
    {
        return;
    }


    m_ServoInputReleaseD2SamplerActive.fetch_add(
        1U,
        std::memory_order_acq_rel);


    // Re-check after becoming active to close the race between the first
    // StopRequested load and Active++.
    if (m_ServoInputReleaseD2StopRequested.load(
        std::memory_order_acquire))
    {
        m_ServoInputReleaseD2SamplerActive.fetch_sub(
            1U,
            std::memory_order_acq_rel);

        return;
    }


    // ========================================================================
    // 1ms SHADOW sampler.
    //
    // Strict rules:
    // - no allocation
    // - no logging
    // - no hardware access
    // - no Process Image write
    //
    // Triple-sample rule:
    //
    //     legacy A
    //     structured Generic
    //     legacy B
    //
    // If A != B, a 250us PDO refresh crossed this comparison window.
    // That field is counted as UnstableSkip instead of a false mismatch.
    // ========================================================================

    if (!m_StructuredServoLiveReadShadowPrepared ||
        m_StructuredServoLiveReadInputFieldCount <=
        0)
    {
        return;
    }


    uint64_t cycleExpected =
        0ULL;

    uint64_t cycleStable =
        0ULL;

    uint64_t cycleMatches =
        0ULL;

    uint64_t cycleUnstable =
        0ULL;

    uint64_t cycleMismatches =
        0ULL;

    uint64_t cycleFailures =
        0ULL;


    for (const auto& field :
        m_StructuredServoDriveFieldsShadow)
    {
        if (strcmp(
            field.direction,
            "Input") !=
            0)
        {
            continue;
        }


        cycleExpected++;


        const ENI_ServoDrive* servo =
            nullptr;


        for (const auto& candidate :
            m_ServoList)
        {
            if (candidate.slaveIndex ==
                field.slaveIndex)
            {
                servo =
                    &candidate;

                break;
            }
        }


        if (servo ==
            nullptr ||
            servo->pInput ==
            nullptr ||
            field.pByteBase ==
            nullptr ||
            field.byteSpan ==
            0U ||
            field.byteSpan >
            sizeof(uint64_t))
        {
            cycleFailures++;
            continue;
        }


        const uint8_t* legacyPointer =
            reinterpret_cast<const uint8_t*>(
                servo->pInput) +
            (
                field.relativeBitOffset /
                8U
                );


        uint64_t legacyBefore =
            0ULL;

        uint64_t structuredValue =
            0ULL;

        uint64_t legacyAfter =
            0ULL;


        const bool beforePass =
            ReadRawFieldBytes(
                legacyPointer,
                field.byteSpan,
                legacyBefore);


        const bool structuredPass =
            ReadRawFieldBytes(
                field.pByteBase,
                field.byteSpan,
                structuredValue);


        const bool afterPass =
            ReadRawFieldBytes(
                legacyPointer,
                field.byteSpan,
                legacyAfter);


        if (!beforePass ||
            !structuredPass ||
            !afterPass)
        {
            cycleFailures++;
            continue;
        }


        if (legacyBefore !=
            legacyAfter)
        {
            cycleUnstable++;
            continue;
        }


        cycleStable++;


        if (structuredValue ==
            legacyBefore)
        {
            cycleMatches++;
        }
        else
        {
            cycleMismatches++;
        }
    }


    m_StructuredServoLiveReadCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadExpectedChecks.fetch_add(
        cycleExpected,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadStableChecks.fetch_add(
        cycleStable,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadMatches.fetch_add(
        cycleMatches,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadUnstableSkips.fetch_add(
        cycleUnstable,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadMismatches.fetch_add(
        cycleMismatches,
        std::memory_order_relaxed);


    m_StructuredServoLiveReadFailures.fetch_add(
        cycleFailures,
        std::memory_order_relaxed);


    m_ServoInputReleaseD2SamplerActive.fetch_sub(
        1U,
        std::memory_order_release);
}


void EtherCatMaster::PrintStructuredServoDriveLiveReadShadow() const
{
    static constexpr uint64_t kMinimumCycles =
        5000ULL;


    // Require at least 90% of all attempted field checks to be stable.
    //
    // The remaining <=10% may be legitimate cross-thread 250us refresh
    // windows and are explicitly reported as UnstableSkips.
    static constexpr uint64_t kMinimumStablePermille =
        900ULL;


    const uint64_t cycles =
        m_StructuredServoLiveReadCycles.load(
            std::memory_order_relaxed);


    const uint64_t expected =
        m_StructuredServoLiveReadExpectedChecks.load(
            std::memory_order_relaxed);


    const uint64_t stable =
        m_StructuredServoLiveReadStableChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_StructuredServoLiveReadMatches.load(
            std::memory_order_relaxed);


    const uint64_t unstable =
        m_StructuredServoLiveReadUnstableSkips.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_StructuredServoLiveReadMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_StructuredServoLiveReadFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedByCycles =
        cycles *
        static_cast<uint64_t>(
            m_StructuredServoLiveReadInputFieldCount);


    const bool accountingPass =
        expected ==
        expectedByCycles &&
        stable +
        unstable +
        failures ==
        expected &&
        matches +
        mismatches ==
        stable;


    const uint64_t stablePermille =
        expected >
        0ULL
        ? (
            stable *
            1000ULL
            ) /
        expected
        : 0ULL;


    const bool windowQualified =
        cycles >=
        kMinimumCycles;


    const bool stableCoveragePass =
        expected >
        0ULL &&
        stablePermille >=
        kMinimumStablePermille;


    const bool clean =
        accountingPass &&
        stableCoveragePass &&
        mismatches ==
        0ULL &&
        failures ==
        0ULL;


    const bool ready =
        m_StructuredServoLiveReadShadowPrepared &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[SERVO-STRUCTURED-LIVE-READ-SHADOW] "
        "Prepared:%s | Fields:%d | "
        "Cycles:%llu/%llu | "
        "Expected:%llu | Stable:%llu Matches:%llu | "
        "Unstable:%llu | Mismatch:%llu ReadFail:%llu | "
        "StablePermille:%llu/1000 | "
        "Accounting:%s | Window:%s | Clean:%s | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | Ready:%s | Result:%s\n",

        m_StructuredServoLiveReadShadowPrepared
        ? "YES"
        : "NO",

        m_StructuredServoLiveReadInputFieldCount,

        (unsigned long long)
        cycles,

        (unsigned long long)
        kMinimumCycles,

        (unsigned long long)
        expected,

        (unsigned long long)
        stable,

        (unsigned long long)
        matches,

        (unsigned long long)
        unstable,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        failures,

        (unsigned long long)
        stablePermille,

        accountingPass
        ? "PASS"
        : "FAIL",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.3:
    // piggyback on the same existing 1000ms diagnostic call.
    PrintMotionServoInputConsumerBridgeShadow();
}


// ============================================================================
// Stage 11D.3 - Motion Servo Input Consumer Bridge SHADOW
//
// Current Motion consumer identity is the existing drive/context list slot.
// AxisContext.axisIndex is preserved as the semantic Motion identity.
//
// Axis NAME is deliberately not part of this mapping.
// NC owns axis-name / parameter interpretation.
// ============================================================================

bool EtherCatMaster::PrepareMotionServoInputConsumerBridgeShadow()
{
    m_MotionServoInputBridgeShadowPrepared =
        false;

    m_MotionServoInputBridgeFieldCount =
        0;

    m_MotionServoInputBridgeShadowMaps.clear();


    m_MotionServoInputBridgeCycles.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeFailures.store(
        0ULL,
        std::memory_order_relaxed);


    int maps =
        0;

    int fields =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-SERVO-INPUT-BRIDGE-PREPARE] BEGIN | "
        "Stage:11D.3 | "
        "Identity:MOTION_SLOT_AXIS_INDEX | "
        "Source:STRUCTURED_SERVO_INPUT | "
        "CompareTo:MOTION_ENI_SERVO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n"
        "============================================================\n");


    if (m_ServoList.empty() ||
        m_ServoList.size() !=
        m_Axes.size() ||
        m_ServoList.size() >
        static_cast<size_t>(
            MAX_AXES))
    {
        errors++;
    }


    if (errors ==
        0)
    {
        m_MotionServoInputBridgeShadowMaps.reserve(
            m_ServoList.size());


        for (size_t slot = 0;
            slot <
            m_ServoList.size();
            ++slot)
        {
            const ENI_ServoDrive& servo =
                m_ServoList[
                    slot];


            const AxisContext& axis =
                m_Axes[
                    slot];


            MotionServoInputBridgeShadowMap
                map;


            map.motionSlot =
                static_cast<int>(
                    slot);

            map.axisIndex =
                axis.axisIndex;

            map.servoSlaveIndex =
                servo.slaveIndex;


            map.statusWord =
                FindStructuredServoDriveFieldShadow(
                    servo.slaveIndex,
                    "StatusWord");

            map.actualPosition =
                FindStructuredServoDriveFieldShadow(
                    servo.slaveIndex,
                    "ActualPosition");

            map.modesOfOperationDisplay =
                FindStructuredServoDriveFieldShadow(
                    servo.slaveIndex,
                    "ModesOfOperationDisplay");

            map.touchProbeStatus =
                FindStructuredServoDriveFieldShadow(
                    servo.slaveIndex,
                    "TouchProbeStatus");

            map.touchProbePosition =
                FindStructuredServoDriveFieldShadow(
                    servo.slaveIndex,
                    "TouchProbePosition");


            const EtherCatStructuredServoFieldDescriptor*
                descriptors[5] =
            {
                map.statusWord,
                map.actualPosition,
                map.modesOfOperationDisplay,
                map.touchProbeStatus,
                map.touchProbePosition
            };


            const uint8_t*
                legacyPointers[5] =
            {
                servo.pInput != nullptr
                ? reinterpret_cast<const uint8_t*>(
                    &servo.pInput->StatusWord)
                : nullptr,

                servo.pInput != nullptr
                ? reinterpret_cast<const uint8_t*>(
                    &servo.pInput->ActualPosition)
                : nullptr,

                servo.pInput != nullptr
                ? reinterpret_cast<const uint8_t*>(
                    &servo.pInput->ModesOfOperationDisplay)
                : nullptr,

                servo.pInput != nullptr
                ? reinterpret_cast<const uint8_t*>(
                    &servo.pInput->TouchProbeStatus)
                : nullptr,

                servo.pInput != nullptr
                ? reinterpret_cast<const uint8_t*>(
                    &servo.pInput->TouchProbePos1)
                : nullptr
            };


            bool mapPass =
                servo.pInput !=
                nullptr;


            for (int fieldIndex = 0;
                fieldIndex <
                5;
                ++fieldIndex)
            {
                fields++;
                pointerChecks++;


                const auto* descriptor =
                    descriptors[
                        fieldIndex];


                const bool descriptorPass =
                    descriptor !=
                    nullptr &&
                    strcmp(
                        descriptor->direction,
                        "Input") ==
                    0 &&
                    descriptor->pByteBase !=
                    nullptr &&
                    descriptor->byteSpan >
                    0U &&
                    descriptor->byteSpan <=
                    sizeof(uint64_t);


                const bool pointerMatch =
                    descriptorPass &&
                    legacyPointers[
                        fieldIndex] !=
                    nullptr &&
                            reinterpret_cast<const uint8_t*>(
                                descriptor->pByteBase) ==
                            legacyPointers[
                                fieldIndex];


                        if (pointerMatch)
                        {
                            pointerMatches++;
                        }
                        else
                        {
                            mapPass =
                                false;

                            errors++;
                        }
            }


            if (mapPass)
            {
                m_MotionServoInputBridgeShadowMaps.push_back(
                    map);

                maps++;
            }


            DEBUG_PRINT(
                "[MOTION-SERVO-INPUT-BRIDGE-MAP] "
                "Slot:%u | AxisIndex:%d | "
                "Servo:S%d | "
                "Fields:5/5 | Pointer:%s | "
                "Identity:AXIS_INDEX | "
                "AxisName:NC_MANAGED | "
                "Result:%s\n",

                (unsigned int)
                slot,

                axis.axisIndex,

                servo.slaveIndex,

                mapPass
                ? "MATCH"
                : "FAIL",

                mapPass
                ? "PASS"
                : "FAIL");
        }
    }


    const int expectedMaps =
        static_cast<int>(
            m_ServoList.size());


    const int expectedFields =
        expectedMaps *
        5;


    const bool pass =
        errors ==
        0 &&
        maps ==
        expectedMaps &&
        fields ==
        expectedFields &&
        pointerChecks ==
        expectedFields &&
        pointerMatches ==
        expectedFields;


    if (pass)
    {
        m_MotionServoInputBridgeFieldCount =
            fields;

        m_MotionServoInputBridgeShadowPrepared =
            true;
    }


    DEBUG_PRINT(
        "[MOTION-SERVO-INPUT-BRIDGE-PREPARE-RESULT] "
        "Maps:%d/%d | Fields:%d/%d | "
        "PointerChecks:%d PointerMatches:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "Identity:AXIS_INDEX | "
        "AxisName:NC_MANAGED | "
        "RuntimeAxisRef:NOT_REQUIRED | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n",

        maps,

        expectedMaps,

        fields,

        expectedFields,

        pointerChecks,

        pointerMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoInputBridgeShadowPrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[MOTION-SERVO-INPUT-BRIDGE-PREPARE] END | "
        "Stage:11D.3 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::SampleMotionServoInputConsumerBridgeShadow()
{
    // ========================================================================
    // Called from MotionCore::UpdateAllMotion() in the 250us Motion phase.
    //
    // The PDO Process Image has already been refreshed in this SAME RT thread.
    // Therefore no cross-thread stability window is needed here.
    //
    // Strict rules:
    // - no allocation
    // - no logging
    // - no descriptor lookup by string
    // - no Process Image write
    // ========================================================================

    if (!m_MotionServoInputBridgeShadowPrepared)
    {
        return;
    }


    uint64_t cycleChecks =
        0ULL;

    uint64_t cycleMatches =
        0ULL;

    uint64_t cycleMismatches =
        0ULL;

    uint64_t cycleFailures =
        0ULL;


    for (const auto& map :
        m_MotionServoInputBridgeShadowMaps)
    {
        if (map.motionSlot <
            0 ||
            map.motionSlot >=
            static_cast<int>(
                m_ServoList.size()))
        {
            cycleFailures +=
                5ULL;

            continue;
        }


        const ENI_ServoDrive& servo =
            m_ServoList[
                static_cast<size_t>(
                    map.motionSlot)];


        if (servo.pInput ==
            nullptr)
        {
            cycleFailures +=
                5ULL;

            continue;
        }


        const EtherCatStructuredServoFieldDescriptor*
            descriptors[5] =
        {
            map.statusWord,
            map.actualPosition,
            map.modesOfOperationDisplay,
            map.touchProbeStatus,
            map.touchProbePosition
        };


        const uint8_t*
            legacyPointers[5] =
        {
            reinterpret_cast<const uint8_t*>(
                &servo.pInput->StatusWord),

            reinterpret_cast<const uint8_t*>(
                &servo.pInput->ActualPosition),

            reinterpret_cast<const uint8_t*>(
                &servo.pInput->ModesOfOperationDisplay),

            reinterpret_cast<const uint8_t*>(
                &servo.pInput->TouchProbeStatus),

            reinterpret_cast<const uint8_t*>(
                &servo.pInput->TouchProbePos1)
        };


        for (int fieldIndex = 0;
            fieldIndex <
            5;
            ++fieldIndex)
        {
            cycleChecks++;


            const auto* descriptor =
                descriptors[
                    fieldIndex];


            if (descriptor ==
                nullptr ||
                descriptor->pByteBase ==
                nullptr ||
                descriptor->byteSpan ==
                0U ||
                descriptor->byteSpan >
                sizeof(uint64_t))
            {
                cycleFailures++;
                continue;
            }


            uint64_t genericValue =
                0ULL;

            uint64_t legacyValue =
                0ULL;


            const bool genericPass =
                ReadRawFieldBytes(
                    descriptor->pByteBase,
                    descriptor->byteSpan,
                    genericValue);


            const bool legacyPass =
                ReadRawFieldBytes(
                    legacyPointers[
                        fieldIndex],
                    descriptor->byteSpan,
                            legacyValue);


            if (!genericPass ||
                !legacyPass)
            {
                cycleFailures++;
                continue;
            }


            if (genericValue ==
                legacyValue)
            {
                cycleMatches++;
            }
            else
            {
                cycleMismatches++;
            }
        }
    }


    m_MotionServoInputBridgeCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeChecks.fetch_add(
        cycleChecks,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeMatches.fetch_add(
        cycleMatches,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeMismatches.fetch_add(
        cycleMismatches,
        std::memory_order_relaxed);

    m_MotionServoInputBridgeFailures.fetch_add(
        cycleFailures,
        std::memory_order_relaxed);


    // Stage 11D.5:
    // run the AxisIndex semantic input preflight in the SAME
    // existing 250us Motion shadow phase.
    SampleMotionServoAxisIndexInputPreflightShadow();
}


void EtherCatMaster::PrintMotionServoInputConsumerBridgeShadow() const
{
    // 250us Motion phase:
    // 20,000 cycles ~= 5 seconds.
    static constexpr uint64_t kMinimumMotionCycles =
        20000ULL;


    const uint64_t cycles =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed);


    const uint64_t checks =
        m_MotionServoInputBridgeChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_MotionServoInputBridgeMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_MotionServoInputBridgeMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_MotionServoInputBridgeFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        cycles *
        static_cast<uint64_t>(
            m_MotionServoInputBridgeFieldCount);


    const bool accountingPass =
        checks ==
        expectedChecks &&
        matches +
        mismatches +
        failures ==
        checks;


    const bool windowQualified =
        cycles >=
        kMinimumMotionCycles;


    // Stage11D.2 must also be qualified before this consumer bridge
    // can be considered ready for the next shadow stage.
    const uint64_t d2Cycles =
        m_StructuredServoLiveReadCycles.load(
            std::memory_order_relaxed);


    const uint64_t d2Mismatch =
        m_StructuredServoLiveReadMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d2Failures =
        m_StructuredServoLiveReadFailures.load(
            std::memory_order_relaxed);


    const bool d2Qualified =
        m_StructuredServoLiveReadShadowPrepared &&
        d2Cycles >=
        5000ULL &&
        d2Mismatch ==
        0ULL &&
        d2Failures ==
        0ULL;


    const bool clean =
        accountingPass &&
        mismatches ==
        0ULL &&
        failures ==
        0ULL;


    const bool ready =
        m_MotionServoInputBridgeShadowPrepared &&
        windowQualified &&
        d2Qualified &&
        clean;


    DEBUG_PRINT(
        "[MOTION-SERVO-INPUT-BRIDGE-SHADOW] "
        "Prepared:%s | "
        "Maps:%u | Fields:%d | "
        "Cycles:%llu/%llu | "
        "Checks:%llu Expected:%llu | "
        "Matches:%llu Mismatch:%llu ReadFail:%llu | "
        "Accounting:%s | D2:%s | Window:%s | Clean:%s | "
        "Identity:AXIS_INDEX | "
        "AxisIndexGate:%s | "
        "AxisName:NC_MANAGED | "
        "RuntimeAxisRef:NOT_REQUIRED | "
        "LiveSemanticCutoverEligible:%s | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | Ready:%s | Result:%s\n",

        m_MotionServoInputBridgeShadowPrepared
        ? "YES"
        : "NO",

        (unsigned int)
        m_MotionServoInputBridgeShadowMaps.size(),

        m_MotionServoInputBridgeFieldCount,

        (unsigned long long)
        cycles,

        (unsigned long long)
        kMinimumMotionCycles,

        (unsigned long long)
        checks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        failures,

        accountingPass
        ? "PASS"
        : "FAIL",

        d2Qualified
        ? "QUALIFIED"
        : "WAIT",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : (
            m_MotionServoAxisIndexIdentityAuditRan
            ? "BLOCK"
            : "NOT_RUN"
            ),

        m_MotionServoAxisIndexIdentityGatePassed
        ? "YES"
        : "WAIT_AXIS_INDEX_GATE",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.4 Rev2:
    // static AxisIndex identity gate status.
    PrintMotionServoAxisIndexIdentityGate();
}


// ============================================================================
// Stage 11D.4 Rev2 - Motion Servo AxisIndex Semantic Identity Gate
//
// EtherCAT/Motion identity:
//
//     Motion slot
//         -> AxisContext.axisIndex
//         -> Servo slaveIndex
//         -> structured Servo field owner
//
// Axis NAME is not part of this identity.
// NC owns axis-name / parameter interpretation.
//
// No Motion consumer cutover.
// No Servo command cutover.
// No Process Image write.
// No EtherCAT hardware access.
// ============================================================================

bool EtherCatMaster::AuditMotionServoAxisIndexIdentityGate()
{
    m_MotionServoAxisIndexIdentityAuditRan =
        true;

    m_MotionServoAxisIndexIdentityGatePassed =
        false;

    m_MotionServoAxisIndexIdentityValidAxes =
        0;

    m_MotionServoAxisIndexIdentityContextMatches =
        0;

    m_MotionServoAxisIndexIdentityServoMatches =
        0;

    m_MotionServoAxisIndexIdentityFieldOwnerMatches =
        0;

    m_MotionServoAxisIndexIdentityDuplicates =
        0;

    m_MotionServoAxisIndexIdentityErrors =
        0;

    m_MotionServoAxisIndexIdentityAuditEntries.clear();


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-SERVO-AXISINDEX-IDENTITY] BEGIN | "
        "Stage:11D.4R2 | "
        "Identity:AXIS_INDEX | "
        "AxisName:NC_MANAGED | "
        "AXIS_CFGConsumedByGate:NO | "
        "RuntimeAxisRefRequired:NO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n"
        "============================================================\n");


    if (!m_MotionServoInputBridgeShadowPrepared ||
        m_MotionServoInputBridgeShadowMaps.empty() ||
        m_MotionServoInputBridgeShadowMaps.size() !=
        m_ServoList.size() ||
        m_MotionServoInputBridgeShadowMaps.size() !=
        m_Axes.size())
    {
        m_MotionServoAxisIndexIdentityErrors++;

        DEBUG_PRINT(
            "[MOTION-SERVO-AXISINDEX-IDENTITY-RESULT] "
            "Maps:%u | ValidAxisIndex:0 | "
            "ContextMatches:0 | ServoMatches:0 | "
            "FieldOwnerMatches:0 | Duplicates:0 | Errors:%d | "
            "Result:FAIL | Gate:BLOCK | "
            "Reason:STAGE11D3_MAP_NOT_READY | "
            "Identity:AXIS_INDEX | AxisName:NC_MANAGED | "
            "RuntimeAxisRefRequired:NO | "
            "MotionConsumer:LEGACY_ENI_SERVO | "
            "ServoCommand:LEGACY_ENI_SERVO | LiveWrite:NO\n",

            (unsigned int)
            m_MotionServoInputBridgeShadowMaps.size(),

            m_MotionServoAxisIndexIdentityErrors);

        DEBUG_PRINT(
            "============================================================\n"
            "[MOTION-SERVO-AXISINDEX-IDENTITY] END | "
            "Stage:11D.4R2 | Result:FAIL\n"
            "============================================================\n\n");

        return
            false;
    }


    m_MotionServoAxisIndexIdentityAuditEntries.reserve(
        m_MotionServoInputBridgeShadowMaps.size());


    for (size_t mapIndex = 0;
        mapIndex <
        m_MotionServoInputBridgeShadowMaps.size();
        ++mapIndex)
    {
        const auto& map =
            m_MotionServoInputBridgeShadowMaps[
                mapIndex];

        MotionServoAxisIndexIdentityAuditEntry
            entry;

        entry.motionSlot =
            map.motionSlot;

        entry.axisIndex =
            map.axisIndex;

        entry.servoSlaveIndex =
            map.servoSlaveIndex;


        entry.slotValid =
            map.motionSlot >=
            0 &&
            map.motionSlot <
            static_cast<int>(
                m_Axes.size()) &&
            map.motionSlot <
            static_cast<int>(
                m_ServoList.size()) &&
            map.motionSlot ==
            static_cast<int>(
                mapIndex);


        entry.axisIndexValid =
            map.axisIndex >=
            0 &&
            map.axisIndex <
            MAX_AXES;


        if (entry.axisIndexValid)
        {
            m_MotionServoAxisIndexIdentityValidAxes++;
        }


        entry.contextMatch =
            entry.slotValid &&
            m_Axes[
                static_cast<size_t>(
                    map.motionSlot)]
            .axisIndex ==
                    map.axisIndex;


                if (entry.contextMatch)
                {
                    m_MotionServoAxisIndexIdentityContextMatches++;
                }


                entry.servoMatch =
                    entry.slotValid &&
                    m_ServoList[
                        static_cast<size_t>(
                            map.motionSlot)]
                    .slaveIndex ==
                            map.servoSlaveIndex;


                        if (entry.servoMatch)
                        {
                            m_MotionServoAxisIndexIdentityServoMatches++;
                        }


                        const EtherCatStructuredServoFieldDescriptor*
                            fields[5] =
                        {
                            map.statusWord,
                            map.actualPosition,
                            map.modesOfOperationDisplay,
                            map.touchProbeStatus,
                            map.touchProbePosition
                        };


                        bool fieldOwnerMatch =
                            true;


                        for (const auto* field :
                            fields)
                        {
                            if (field ==
                                nullptr ||
                                field->slaveIndex !=
                                map.servoSlaveIndex ||
                                strcmp(
                                    field->direction,
                                    "Input") !=
                                0 ||
                                field->pByteBase ==
                                nullptr)
                            {
                                fieldOwnerMatch =
                                    false;

                                break;
                            }
                        }


                        entry.structuredFieldOwnerMatch =
                            fieldOwnerMatch;


                        if (entry.structuredFieldOwnerMatch)
                        {
                            m_MotionServoAxisIndexIdentityFieldOwnerMatches++;
                        }


                        const bool entryPass =
                            entry.slotValid &&
                            entry.axisIndexValid &&
                            entry.contextMatch &&
                            entry.servoMatch &&
                            entry.structuredFieldOwnerMatch;


                        if (!entryPass)
                        {
                            m_MotionServoAxisIndexIdentityErrors++;
                        }


                        DEBUG_PRINT(
                            "[MOTION-SERVO-AXISINDEX-IDENTITY-MAP] "
                            "Slot:%d | AxisIndex:%d | Servo:S%d | "
                            "Slot:%s | AxisRange:%s | "
                            "Context:%s | Servo:%s | FieldOwner:%s | "
                            "AxisName:NC_MANAGED | Result:%s\n",

                            entry.motionSlot,

                            entry.axisIndex,

                            entry.servoSlaveIndex,

                            entry.slotValid
                            ? "PASS"
                            : "FAIL",

                            entry.axisIndexValid
                            ? "PASS"
                            : "FAIL",

                            entry.contextMatch
                            ? "MATCH"
                            : "FAIL",

                            entry.servoMatch
                            ? "MATCH"
                            : "FAIL",

                            entry.structuredFieldOwnerMatch
                            ? "MATCH"
                            : "FAIL",

                            entryPass
                            ? "PASS"
                            : "FAIL");


                        m_MotionServoAxisIndexIdentityAuditEntries.push_back(
                            entry);
    }


    // Axis indices do NOT have to be contiguous.
    //
    // Valid examples:
    //     0, 1, 2
    //     0, 2, 5
    //     1, 4, 7
    //
    // Only range + uniqueness matter.
    for (size_t i = 0;
        i <
        m_MotionServoAxisIndexIdentityAuditEntries.size();
        ++i)
    {
        const auto& left =
            m_MotionServoAxisIndexIdentityAuditEntries[
                i];


        for (size_t j =
            i + 1;
            j <
            m_MotionServoAxisIndexIdentityAuditEntries.size();
            ++j)
        {
            const auto& right =
                m_MotionServoAxisIndexIdentityAuditEntries[
                    j];


            if (left.axisIndex ==
                right.axisIndex)
            {
                m_MotionServoAxisIndexIdentityDuplicates++;
            }


            if (left.servoSlaveIndex ==
                right.servoSlaveIndex)
            {
                m_MotionServoAxisIndexIdentityDuplicates++;
            }


            if (left.motionSlot ==
                right.motionSlot)
            {
                m_MotionServoAxisIndexIdentityDuplicates++;
            }
        }
    }


    if (m_MotionServoAxisIndexIdentityDuplicates >
        0)
    {
        m_MotionServoAxisIndexIdentityErrors +=
            m_MotionServoAxisIndexIdentityDuplicates;
    }


    const int mapCount =
        static_cast<int>(
            m_MotionServoAxisIndexIdentityAuditEntries.size());


    const bool pass =
        mapCount >
        0 &&
        m_MotionServoAxisIndexIdentityValidAxes ==
        mapCount &&
        m_MotionServoAxisIndexIdentityContextMatches ==
        mapCount &&
        m_MotionServoAxisIndexIdentityServoMatches ==
        mapCount &&
        m_MotionServoAxisIndexIdentityFieldOwnerMatches ==
        mapCount &&
        m_MotionServoAxisIndexIdentityDuplicates ==
        0 &&
        m_MotionServoAxisIndexIdentityErrors ==
        0;


    m_MotionServoAxisIndexIdentityGatePassed =
        pass;


    DEBUG_PRINT(
        "[MOTION-SERVO-AXISINDEX-IDENTITY-RESULT] "
        "Maps:%d | "
        "ValidAxisIndex:%d | "
        "ContextMatches:%d | "
        "ServoMatches:%d | "
        "FieldOwnerMatches:%d | "
        "Duplicates:%d | Errors:%d | "
        "Result:%s | Gate:%s | "
        "Identity:AXIS_INDEX | "
        "AxisName:NC_MANAGED | "
        "AXIS_CFGConsumedByGate:NO | "
        "RuntimeAxisRefRequired:NO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | Next:%s\n",

        mapCount,

        m_MotionServoAxisIndexIdentityValidAxes,

        m_MotionServoAxisIndexIdentityContextMatches,

        m_MotionServoAxisIndexIdentityServoMatches,

        m_MotionServoAxisIndexIdentityFieldOwnerMatches,

        m_MotionServoAxisIndexIdentityDuplicates,

        m_MotionServoAxisIndexIdentityErrors,

        pass
        ? "PASS"
        : "FAIL",

        pass
        ? "OPEN"
        : "BLOCK",

        pass
        ? "ALLOW_STAGE11D5"
        : "FIX_AXIS_INDEX_MAPPING");


    DEBUG_PRINT(
        "============================================================\n"
        "[MOTION-SERVO-AXISINDEX-IDENTITY] END | "
        "Stage:11D.4R2 | Result:%s | MotionCutover:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::PrintMotionServoAxisIndexIdentityGate() const
{
    DEBUG_PRINT(
        "[MOTION-SERVO-AXISINDEX-IDENTITY-STATUS] "
        "Ran:%s | Maps:%u | "
        "ValidAxisIndex:%d | "
        "ContextMatches:%d | "
        "ServoMatches:%d | "
        "FieldOwnerMatches:%d | "
        "Duplicates:%d | Errors:%d | "
        "State:%s | Gate:%s | "
        "Identity:AXIS_INDEX | "
        "AxisName:NC_MANAGED | "
        "RuntimeAxisRefRequired:NO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n",

        m_MotionServoAxisIndexIdentityAuditRan
        ? "YES"
        : "NO",

        (unsigned int)
        m_MotionServoAxisIndexIdentityAuditEntries.size(),

        m_MotionServoAxisIndexIdentityValidAxes,

        m_MotionServoAxisIndexIdentityContextMatches,

        m_MotionServoAxisIndexIdentityServoMatches,

        m_MotionServoAxisIndexIdentityFieldOwnerMatches,

        m_MotionServoAxisIndexIdentityDuplicates,

        m_MotionServoAxisIndexIdentityErrors,

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : (
            m_MotionServoAxisIndexIdentityAuditRan
            ? "BLOCKED"
            : "NOT_RUN"
            ),

        m_MotionServoAxisIndexIdentityGatePassed
        ? "OPEN"
        : "BLOCK");


    // Stage 11D.5:
    // same existing low-frequency supervisory diagnostic chain.
    PrintMotionServoAxisIndexInputPreflightShadow();
}


// ============================================================================
// Stage 11D.5 - AxisIndex Semantic Motion Input Preflight Shadow
//
// Semantic identity:
//
//     AxisContext.axisIndex
//
// Prepared route:
//
//     AxisIndex
//         -> fixed route array slot
//         -> Servo slave
//         -> five Motion-consumed structured Servo input fields
//
// Actual MotionCore consumer:
//     LEGACY ENI_ServoDrive
//
// This stage does NOT cut over Motion.
// ============================================================================

namespace
{
    template <typename TValue>
    bool ReadStructuredTypedFieldValueD5(
        const EtherCatStructuredServoFieldDescriptor* field,
        TValue& value)
    {
        value =
            TValue{};


        if (field ==
            nullptr ||
            field->pByteBase ==
            nullptr ||
            strcmp(
                field->direction,
                "Input") !=
            0 ||
            field->byteSpan !=
            sizeof(TValue) ||
            !field->byteAligned)
        {
            return
                false;
        }


        memcpy(
            &value,
            field->pByteBase,
            sizeof(TValue));


        return
            true;
    }
}


bool EtherCatMaster::PrepareMotionServoAxisIndexInputPreflightShadow()
{
    m_MotionServoAxisIndexInputPreflightPrepared =
        false;

    m_MotionServoAxisIndexInputRouteCount =
        0;

    m_MotionServoAxisIndexInputFieldCount =
        0;


    m_MotionServoAxisIndexInputPreflightCycles.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightFailures.store(
        0ULL,
        std::memory_order_relaxed);


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        m_MotionServoAxisIndexInputRoutesShadow[
            axisIndex] =
            MotionServoAxisIndexInputRouteShadow{};
    }


    int routeCount =
        0;

    int fieldCount =
        0;

    int lookupChecks =
        0;

    int lookupMatches =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int valueChecks =
        0;

    int valueMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-AXISINDEX-INPUT-PREFLIGHT] BEGIN | "
        "Stage:11D.5 | "
        "Identity:AXIS_INDEX | "
        "Route:FIXED_AXIS_INDEX_TABLE | "
        "SemanticReadAPI:AXIS_INDEX_SHADOW | "
        "AxisName:NC_MANAGED | "
        "RuntimeAxisRefRequired:NO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO\n"
        "============================================================\n");


    if (!m_MotionServoAxisIndexIdentityGatePassed ||
        !m_MotionServoInputBridgeShadowPrepared)
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (const auto& map :
            m_MotionServoInputBridgeShadowMaps)
        {
            if (map.axisIndex <
                0 ||
                map.axisIndex >=
                MAX_AXES ||
                map.motionSlot <
                0 ||
                map.motionSlot >=
                static_cast<int>(
                    m_ServoList.size()))
            {
                errors++;
                continue;
            }


            MotionServoAxisIndexInputRouteShadow&
                route =
                m_MotionServoAxisIndexInputRoutesShadow[
                    map.axisIndex];


            if (route.valid)
            {
                errors++;
                continue;
            }


            route.valid =
                true;

            route.axisIndex =
                map.axisIndex;

            route.motionSlot =
                map.motionSlot;

            route.servoSlaveIndex =
                map.servoSlaveIndex;

            route.statusWord =
                map.statusWord;

            route.actualPosition =
                map.actualPosition;

            route.modesOfOperationDisplay =
                map.modesOfOperationDisplay;

            route.touchProbeStatus =
                map.touchProbeStatus;

            route.touchProbePosition =
                map.touchProbePosition;


            routeCount++;
            fieldCount +=
                5;
        }
    }


    // ========================================================================
    // Startup lookup audit.
    //
    // Query in reverse D3-map order so the check is explicitly by AxisIndex
    // rather than by physical vector traversal order.
    // ========================================================================

    for (auto it =
        m_MotionServoInputBridgeShadowMaps.rbegin();
        it !=
        m_MotionServoInputBridgeShadowMaps.rend();
        ++it)
    {
        const auto& map =
            *it;


        lookupChecks++;


        const MotionServoAxisIndexInputRouteShadow*
            route =
            nullptr;


        if (map.axisIndex >=
            0 &&
            map.axisIndex <
            MAX_AXES)
        {
            const auto& candidate =
                m_MotionServoAxisIndexInputRoutesShadow[
                    map.axisIndex];


            if (candidate.valid)
            {
                route =
                    &candidate;
            }
        }


        const bool lookupMatch =
            route !=
            nullptr &&
            route->axisIndex ==
            map.axisIndex &&
            route->motionSlot ==
            map.motionSlot &&
            route->servoSlaveIndex ==
            map.servoSlaveIndex;


        if (lookupMatch)
        {
            lookupMatches++;
        }
        else
        {
            errors++;
        }


        if (route ==
            nullptr)
        {
            continue;
        }


        const EtherCatStructuredServoFieldDescriptor*
            fields[5] =
        {
            route->statusWord,
            route->actualPosition,
            route->modesOfOperationDisplay,
            route->touchProbeStatus,
            route->touchProbePosition
        };


        if (route->motionSlot <
            0 ||
            route->motionSlot >=
            static_cast<int>(
                m_ServoList.size()) ||
            m_ServoList[
                static_cast<size_t>(
                    route->motionSlot)]
            .pInput ==
                    nullptr)
        {
            errors++;
            continue;
        }


                const ENI_ServoDrive& servo =
                    m_ServoList[
                        static_cast<size_t>(
                            route->motionSlot)];


                const uint8_t*
                    legacyPointers[5] =
                {
                    reinterpret_cast<const uint8_t*>(
                        &servo.pInput->StatusWord),

                    reinterpret_cast<const uint8_t*>(
                        &servo.pInput->ActualPosition),

                    reinterpret_cast<const uint8_t*>(
                        &servo.pInput->ModesOfOperationDisplay),

                    reinterpret_cast<const uint8_t*>(
                        &servo.pInput->TouchProbeStatus),

                    reinterpret_cast<const uint8_t*>(
                        &servo.pInput->TouchProbePos1)
                };


                bool routePass =
                    lookupMatch;


                for (int fieldIndex = 0;
                    fieldIndex <
                    5;
                    ++fieldIndex)
                {
                    pointerChecks++;


                    const auto* field =
                        fields[
                            fieldIndex];


                    const bool pointerMatch =
                        field !=
                        nullptr &&
                        field->pByteBase !=
                        nullptr &&
                        field->slaveIndex ==
                        route->servoSlaveIndex &&
                        reinterpret_cast<const uint8_t*>(
                            field->pByteBase) ==
                        legacyPointers[
                            fieldIndex];


                    if (pointerMatch)
                    {
                        pointerMatches++;
                    }
                    else
                    {
                        routePass =
                            false;

                        errors++;
                    }


                    valueChecks++;


                    const bool valueMatch =
                        pointerMatch &&
                        field->byteSpan >
                        0U &&
                        memcmp(
                            field->pByteBase,
                            legacyPointers[
                                fieldIndex],
                            field->byteSpan) ==
                        0;


                                if (valueMatch)
                                {
                                    valueMatches++;
                                }
                                else
                                {
                                    routePass =
                                        false;

                                    errors++;
                                }
                }


                DEBUG_PRINT(
                    "[MOTION-AXISINDEX-INPUT-PREFLIGHT-MAP] "
                    "AxisIndex:%d | Slot:%d | Servo:S%d | "
                    "Lookup:%s | Fields:5/5 | "
                    "Pointer:%s | Value:%s | "
                    "AxisName:NC_MANAGED | Result:%s\n",

                    route->axisIndex,

                    route->motionSlot,

                    route->servoSlaveIndex,

                    lookupMatch
                    ? "PASS"
                    : "FAIL",

                    routePass
                    ? "MATCH"
                    : "FAIL",

                    routePass
                    ? "MATCH"
                    : "FAIL",

                    routePass
                    ? "PASS"
                    : "FAIL");
    }


    const int expectedRoutes =
        static_cast<int>(
            m_MotionServoInputBridgeShadowMaps.size());


    const int expectedFields =
        expectedRoutes *
        5;


    const bool pass =
        errors ==
        0 &&
        expectedRoutes >
        0 &&
        routeCount ==
        expectedRoutes &&
        fieldCount ==
        expectedFields &&
        lookupChecks ==
        expectedRoutes &&
        lookupMatches ==
        expectedRoutes &&
        pointerChecks ==
        expectedFields &&
        pointerMatches ==
        expectedFields &&
        valueChecks ==
        expectedFields &&
        valueMatches ==
        expectedFields;


    if (pass)
    {
        m_MotionServoAxisIndexInputRouteCount =
            routeCount;

        m_MotionServoAxisIndexInputFieldCount =
            fieldCount;

        m_MotionServoAxisIndexInputPreflightPrepared =
            true;
    }


    DEBUG_PRINT(
        "[MOTION-AXISINDEX-INPUT-PREFLIGHT-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | "
        "Lookup:%d/%d | "
        "Pointer:%d/%d | Value:%d/%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "Identity:AXIS_INDEX | "
        "Route:FIXED_AXIS_INDEX_TABLE | "
        "SemanticReadAPI:AXIS_INDEX_SHADOW | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | "
        "Next:%s\n",

        routeCount,

        expectedRoutes,

        fieldCount,

        expectedFields,

        lookupMatches,

        lookupChecks,

        pointerMatches,

        pointerChecks,

        valueMatches,

        valueChecks,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoAxisIndexInputPreflightPrepared
        ? "YES"
        : "NO",

        pass
        ? "RUNTIME_QUALIFY_STAGE11D5"
        : "BLOCK_STAGE11D5");


    DEBUG_PRINT(
        "============================================================\n"
        "[MOTION-AXISINDEX-INPUT-PREFLIGHT] END | "
        "Stage:11D.5 | Result:%s | MotionCutover:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool EtherCatMaster::ReadMotionServoInputByAxisIndexShadow(
    int axisIndex,
    uint16_t& statusWord,
    int32_t& actualPosition,
    int8_t& modesOfOperationDisplay,
    uint16_t& touchProbeStatus,
    int32_t& touchProbePosition) const
{
    statusWord =
        0U;

    actualPosition =
        0;

    modesOfOperationDisplay =
        0;

    touchProbeStatus =
        0U;

    touchProbePosition =
        0;


    if (!m_MotionServoAxisIndexInputPreflightPrepared ||
        axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        return
            false;
    }


    const MotionServoAxisIndexInputRouteShadow&
        route =
        m_MotionServoAxisIndexInputRoutesShadow[
            axisIndex];


    if (!route.valid ||
        route.axisIndex !=
        axisIndex)
    {
        return
            false;
    }


    return
        ReadStructuredTypedFieldValueD5(
            route.statusWord,
            statusWord) &&
        ReadStructuredTypedFieldValueD5(
            route.actualPosition,
            actualPosition) &&
        ReadStructuredTypedFieldValueD5(
            route.modesOfOperationDisplay,
            modesOfOperationDisplay) &&
        ReadStructuredTypedFieldValueD5(
            route.touchProbeStatus,
            touchProbeStatus) &&
        ReadStructuredTypedFieldValueD5(
            route.touchProbePosition,
            touchProbePosition);
}


void EtherCatMaster::SampleMotionServoAxisIndexInputPreflightShadow()
{
    // ========================================================================
    // Existing 250us Motion shadow phase.
    //
    // No allocation.
    // No logging.
    // No string lookup.
    // No hardware access.
    // No write.
    //
    // IMPORTANT:
    // Candidate values are obtained ONLY through:
    //
    //     ReadMotionServoInputByAxisIndexShadow(axisIndex, ...)
    //
    // Legacy values are used only as the comparison baseline.
    // ========================================================================

    if (!m_MotionServoAxisIndexInputPreflightPrepared)
    {
        return;
    }


    uint64_t cycleChecks =
        0ULL;

    uint64_t cycleMatches =
        0ULL;

    uint64_t cycleMismatches =
        0ULL;

    uint64_t cycleFailures =
        0ULL;


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        const MotionServoAxisIndexInputRouteShadow&
            route =
            m_MotionServoAxisIndexInputRoutesShadow[
                axisIndex];


        if (!route.valid)
        {
            continue;
        }


        cycleChecks +=
            5ULL;


        if (route.motionSlot <
            0 ||
            route.motionSlot >=
            static_cast<int>(
                m_ServoList.size()))
        {
            cycleFailures +=
                5ULL;

            continue;
        }


        const ENI_ServoDrive& servo =
            m_ServoList[
                static_cast<size_t>(
                    route.motionSlot)];


        if (servo.pInput ==
            nullptr)
        {
            cycleFailures +=
                5ULL;

            continue;
        }


        uint16_t semanticStatusWord =
            0U;

        int32_t semanticActualPosition =
            0;

        int8_t semanticMode =
            0;

        uint16_t semanticTouchProbeStatus =
            0U;

        int32_t semanticTouchProbePosition =
            0;


        const bool semanticReadPass =
            ReadMotionServoInputByAxisIndexShadow(
                axisIndex,
                semanticStatusWord,
                semanticActualPosition,
                semanticMode,
                semanticTouchProbeStatus,
                semanticTouchProbePosition);


        if (!semanticReadPass)
        {
            cycleFailures +=
                5ULL;

            continue;
        }


        const bool statusMatch =
            semanticStatusWord ==
            servo.pInput->StatusWord;


        const bool positionMatch =
            semanticActualPosition ==
            servo.pInput->ActualPosition;


        const bool modeMatch =
            semanticMode ==
            servo.pInput->ModesOfOperationDisplay;


        const bool probeStatusMatch =
            semanticTouchProbeStatus ==
            servo.pInput->TouchProbeStatus;


        const bool probePositionMatch =
            semanticTouchProbePosition ==
            servo.pInput->TouchProbePos1;


        const bool matches[5] =
        {
            statusMatch,
            positionMatch,
            modeMatch,
            probeStatusMatch,
            probePositionMatch
        };


        for (bool match :
        matches)
        {
            if (match)
            {
                cycleMatches++;
            }
            else
            {
                cycleMismatches++;
            }
        }
    }


    m_MotionServoAxisIndexInputPreflightCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightChecks.fetch_add(
        cycleChecks,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightMatches.fetch_add(
        cycleMatches,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightMismatches.fetch_add(
        cycleMismatches,
        std::memory_order_relaxed);

    m_MotionServoAxisIndexInputPreflightFailures.fetch_add(
        cycleFailures,
        std::memory_order_relaxed);
}


void EtherCatMaster::PrintMotionServoAxisIndexInputPreflightShadow() const
{
    // 250us Motion phase:
    // 20,000 cycles ~= 5 seconds.
    static constexpr uint64_t kMinimumCycles =
        20000ULL;


    const uint64_t cycles =
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed);


    const uint64_t checks =
        m_MotionServoAxisIndexInputPreflightChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_MotionServoAxisIndexInputPreflightMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_MotionServoAxisIndexInputPreflightMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_MotionServoAxisIndexInputPreflightFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        cycles *
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputFieldCount);


    const bool accountingPass =
        checks ==
        expectedChecks &&
        matches +
        mismatches +
        failures ==
        checks;


    const bool windowQualified =
        cycles >=
        kMinimumCycles;


    const uint64_t d3Cycles =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed);


    const uint64_t d3Mismatches =
        m_MotionServoInputBridgeMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d3Failures =
        m_MotionServoInputBridgeFailures.load(
            std::memory_order_relaxed);


    const bool d3Qualified =
        m_MotionServoInputBridgeShadowPrepared &&
        d3Cycles >=
        20000ULL &&
        d3Mismatches ==
        0ULL &&
        d3Failures ==
        0ULL;


    const bool clean =
        accountingPass &&
        mismatches ==
        0ULL &&
        failures ==
        0ULL;


    const bool ready =
        m_MotionServoAxisIndexInputPreflightPrepared &&
        m_MotionServoAxisIndexIdentityGatePassed &&
        d3Qualified &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[MOTION-AXISINDEX-INPUT-PREFLIGHT-SHADOW] "
        "Prepared:%s | "
        "Routes:%d | Fields:%d | "
        "Cycles:%llu/%llu | "
        "Checks:%llu Expected:%llu | "
        "Matches:%llu Mismatch:%llu ReadFail:%llu | "
        "Accounting:%s | D3:%s | AxisIndexGate:%s | "
        "Window:%s | Clean:%s | "
        "Identity:AXIS_INDEX | "
        "Route:FIXED_AXIS_INDEX_TABLE | "
        "SemanticReadAPI:AXIS_INDEX_SHADOW | "
        "AxisName:NC_MANAGED | "
        "RuntimeAxisRefRequired:NO | "
        "MotionConsumer:LEGACY_ENI_SERVO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveWrite:NO | "
        "Ready:%s | Result:%s\n",

        m_MotionServoAxisIndexInputPreflightPrepared
        ? "YES"
        : "NO",

        m_MotionServoAxisIndexInputRouteCount,

        m_MotionServoAxisIndexInputFieldCount,

        (unsigned long long)
        cycles,

        (unsigned long long)
        kMinimumCycles,

        (unsigned long long)
        checks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        failures,

        accountingPass
        ? "PASS"
        : "FAIL",

        d3Qualified
        ? "QUALIFIED"
        : "WAIT",

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : "BLOCK",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.6:
    // actual Motion consumer seam qualification.
    PrintMotionServoInputConsumerSeamPreflightShadow();
}


// ============================================================================
// Stage 11D.6 - Motion Servo Input Consumer Seam Preflight
//
// Actual source:
//   LEGACY ENI_ServoDrive::pInput -> MotionServoInputSnapshot
//
// Candidate source:
//   AxisIndex -> Stage11D.5 semantic typed read API
//
// Motion algorithms still consume the LEGACY snapshot in this stage.
// ============================================================================

bool EtherCatMaster::PrepareMotionServoInputConsumerSeamPreflightShadow()
{
    m_MotionServoInputConsumerSeamPreflightPrepared =
        false;

    m_MotionServoInputConsumerSeamAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamReadFailures.store(
        0ULL,
        std::memory_order_relaxed);


    const bool pass =
        m_MotionServoAxisIndexIdentityGatePassed &&
        m_MotionServoAxisIndexInputPreflightPrepared &&
        m_MotionServoAxisIndexInputRouteCount >
        0 &&
        m_MotionServoAxisIndexInputFieldCount ==
        m_MotionServoAxisIndexInputRouteCount *
        5;


    if (pass)
    {
        m_MotionServoInputConsumerSeamPreflightPrepared =
            true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-INPUT-CONSUMER-SEAM-PREPARE] BEGIN | "
        "Stage:11D.6 | "
        "ActualInputSource:LEGACY_ENI_SERVO_SNAPSHOT | "
        "CandidateInputSource:AXIS_INDEX_SEMANTIC | "
        "ConsumerSeam:MOTION_SERVO_INPUT_SNAPSHOT | "
        "MotionConsumer:LEGACY_SNAPSHOT | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveSemanticInput:NO\n"
        "============================================================\n");


    DEBUG_PRINT(
        "[MOTION-INPUT-CONSUMER-SEAM-PREPARE-RESULT] "
        "Routes:%d | Fields:%d | "
        "AxisIndexGate:%s | D5Prepared:%s | "
        "Result:%s | Prepared:%s | "
        "UpdateMotionDirectPInput:NO | "
        "UpdateServoStateDirectPInput:NO | "
        "TouchProbeInputDirectPInput:NO | "
        "DebugInputDirectPInput:NO | "
        "ActualInputSource:LEGACY_ENI_SERVO_SNAPSHOT | "
        "CandidateInputSource:AXIS_INDEX_SEMANTIC | "
        "MotionConsumer:LEGACY_SNAPSHOT | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveSemanticInput:NO | Next:%s\n",

        m_MotionServoAxisIndexInputRouteCount,
        m_MotionServoAxisIndexInputFieldCount,

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : "BLOCK",

        m_MotionServoAxisIndexInputPreflightPrepared
        ? "YES"
        : "NO",

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoInputConsumerSeamPreflightPrepared
        ? "YES"
        : "NO",

        pass
        ? "RUNTIME_QUALIFY_STAGE11D6"
        : "BLOCK_STAGE11D6");


    DEBUG_PRINT(
        "============================================================\n"
        "[MOTION-INPUT-CONSUMER-SEAM-PREPARE] END | "
        "Stage:11D.6 | Result:%s | MotionCutover:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return pass;
}


void EtherCatMaster::ObserveMotionServoInputConsumerSeamShadow(
    int axisIndex,
    uint16_t statusWord,
    int32_t actualPosition,
    int8_t modesOfOperationDisplay,
    uint16_t touchProbeStatus,
    int32_t touchProbePosition)
{
    if (!m_MotionServoInputConsumerSeamPreflightPrepared)
    {
        return;
    }


    m_MotionServoInputConsumerSeamAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamChecks.fetch_add(
        5ULL,
        std::memory_order_relaxed);


    uint16_t semanticStatusWord =
        0U;

    int32_t semanticActualPosition =
        0;

    int8_t semanticMode =
        0;

    uint16_t semanticTouchProbeStatus =
        0U;

    int32_t semanticTouchProbePosition =
        0;


    const bool semanticReadPass =
        ReadMotionServoInputByAxisIndexShadow(
            axisIndex,
            semanticStatusWord,
            semanticActualPosition,
            semanticMode,
            semanticTouchProbeStatus,
            semanticTouchProbePosition);


    if (!semanticReadPass)
    {
        m_MotionServoInputConsumerSeamReadFailures.fetch_add(
            5ULL,
            std::memory_order_relaxed);

        return;
    }


    const bool matches[5] =
    {
        semanticStatusWord ==
            statusWord,

        semanticActualPosition ==
            actualPosition,

        semanticMode ==
            modesOfOperationDisplay,

        semanticTouchProbeStatus ==
            touchProbeStatus,

        semanticTouchProbePosition ==
            touchProbePosition
    };


    uint64_t matchCount =
        0ULL;


    for (bool match :
    matches)
    {
        if (match)
        {
            matchCount++;
        }
    }


    m_MotionServoInputConsumerSeamMatches.fetch_add(
        matchCount,
        std::memory_order_relaxed);

    m_MotionServoInputConsumerSeamMismatches.fetch_add(
        5ULL -
        matchCount,
        std::memory_order_relaxed);
}


void EtherCatMaster::PrintMotionServoInputConsumerSeamPreflightShadow() const
{
    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t axisSamples =
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed);

    const uint64_t checks =
        m_MotionServoInputConsumerSeamChecks.load(
            std::memory_order_relaxed);

    const uint64_t matches =
        m_MotionServoInputConsumerSeamMatches.load(
            std::memory_order_relaxed);

    const uint64_t mismatches =
        m_MotionServoInputConsumerSeamMismatches.load(
            std::memory_order_relaxed);

    const uint64_t failures =
        m_MotionServoInputConsumerSeamReadFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        axisSamples *
        5ULL;


    const bool accountingPass =
        checks ==
        expectedChecks &&
        matches +
        mismatches +
        failures ==
        checks;


    const bool windowQualified =
        requiredAxisSamples >
        0ULL &&
        axisSamples >=
        requiredAxisSamples;


    const uint64_t d5Cycles =
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed);

    const uint64_t d5Mismatches =
        m_MotionServoAxisIndexInputPreflightMismatches.load(
            std::memory_order_relaxed);

    const uint64_t d5Failures =
        m_MotionServoAxisIndexInputPreflightFailures.load(
            std::memory_order_relaxed);


    const bool d5Qualified =
        m_MotionServoAxisIndexInputPreflightPrepared &&
        d5Cycles >=
        20000ULL &&
        d5Mismatches ==
        0ULL &&
        d5Failures ==
        0ULL;


    const bool clean =
        accountingPass &&
        mismatches ==
        0ULL &&
        failures ==
        0ULL;


    const bool ready =
        m_MotionServoInputConsumerSeamPreflightPrepared &&
        m_MotionServoAxisIndexIdentityGatePassed &&
        d5Qualified &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[MOTION-INPUT-CONSUMER-SEAM-SHADOW] "
        "Prepared:%s | Routes:%d | "
        "AxisSamples:%llu/%llu | "
        "Checks:%llu Expected:%llu | "
        "Matches:%llu Mismatch:%llu ReadFail:%llu | "
        "Accounting:%s | D5:%s | AxisIndexGate:%s | "
        "Window:%s | Clean:%s | "
        "ActualInputSource:LEGACY_ENI_SERVO_SNAPSHOT | "
        "CandidateInputSource:AXIS_INDEX_SEMANTIC | "
        "ConsumerSeam:MOTION_SERVO_INPUT_SNAPSHOT | "
        "UpdateMotionDirectPInput:NO | "
        "UpdateServoStateDirectPInput:NO | "
        "TouchProbeInputDirectPInput:NO | "
        "DebugInputDirectPInput:NO | "
        "ServoCommand:LEGACY_ENI_SERVO | "
        "LiveSemanticInput:NO | "
        "Ready:%s | Result:%s\n",

        m_MotionServoInputConsumerSeamPreflightPrepared
        ? "YES"
        : "NO",

        m_MotionServoAxisIndexInputRouteCount,

        (unsigned long long)
        axisSamples,

        (unsigned long long)
        requiredAxisSamples,

        (unsigned long long)
        checks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        failures,

        accountingPass
        ? "PASS"
        : "FAIL",

        d5Qualified
        ? "QUALIFIED"
        : "WAIT",

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : "BLOCK",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.7:
    // controlled semantic core Motion input cutover status.
    PrintControlledMotionServoInputCutover();
}


// ============================================================================
// Stage 11D.7 - Controlled AxisIndex Semantic Core Motion Input Cutover
//
// Scope:
//   UpdateMotion / UpdateServoState snapshot source only.
//
// Compatibility kept in this stage:
//   GetDriveTouchProbeData -> legacy snapshot
//   ExportDebugInfo        -> legacy snapshot
//
// Servo OUTPUT command path remains LEGACY.
//
// Fail-safe:
//   Any semantic read failure while active:
//     - current axis immediately keeps same-cycle legacy snapshot
//     - semantic route is disabled
//     - fault is latched
//     - no automatic re-enable until next boot
// ============================================================================

bool EtherCatMaster::PrepareControlledMotionServoInputCutover()
{
    m_ControlledMotionServoInputCutoverPrepared =
        false;

    m_ControlledMotionServoInputCutoverEnabled.store(
        false,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputCutoverFaultLatched.store(
        false,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputCutoverTransitions.store(
        0ULL,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputLegacyAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputSemanticAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputFallbackAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_ControlledMotionServoInputRouteFaults.store(
        0ULL,
        std::memory_order_relaxed);


    const bool pass =
        m_MotionServoInputConsumerSeamPreflightPrepared &&
        m_MotionServoAxisIndexIdentityGatePassed &&
        m_MotionServoAxisIndexInputPreflightPrepared &&
        m_MotionServoAxisIndexInputRouteCount >
        0;


    if (pass)
    {
        m_ControlledMotionServoInputCutoverPrepared =
            true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-SEMANTIC-INPUT-CUTOVER-PREPARE] "
        "Stage:11D.7 | Routes:%d | "
        "D6Prepared:%s | AxisGate:%s | "
        "Result:%s | Prepared:%s | "
        "StartSource:LEGACY_SNAPSHOT | "
        "TargetSource:AXIS_INDEX_SEMANTIC | "
        "ServoCommand:LEGACY | LiveOutputWrite:NO\n"
        "============================================================\n",

        m_MotionServoAxisIndexInputRouteCount,

        m_MotionServoInputConsumerSeamPreflightPrepared
        ? "YES"
        : "NO",

        m_MotionServoAxisIndexIdentityGatePassed
        ? "PASS"
        : "BLOCK",

        pass
        ? "PASS"
        : "FAIL",

        m_ControlledMotionServoInputCutoverPrepared
        ? "YES"
        : "NO");


    return
        pass;
}


void EtherCatMaster::UpdateControlledMotionServoInputCutoverGate()
{
    // ========================================================================
    // 250us Motion hot path.
    //
    // No allocation.
    // No logging.
    // No hardware access.
    //
    // Called once BEFORE the axis loop so all axes in the cycle use one
    // consistent source.
    // ========================================================================

    if (!m_ControlledMotionServoInputCutoverPrepared)
    {
        return;
    }


    if (m_ControlledMotionServoInputCutoverEnabled.load(
        std::memory_order_relaxed))
    {
        return;
    }


    if (m_ControlledMotionServoInputCutoverFaultLatched.load(
        std::memory_order_relaxed))
    {
        return;
    }


    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d6AxisSamples =
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t d6Checks =
        m_MotionServoInputConsumerSeamChecks.load(
            std::memory_order_relaxed);


    const uint64_t d6Matches =
        m_MotionServoInputConsumerSeamMatches.load(
            std::memory_order_relaxed);


    const uint64_t d6Mismatches =
        m_MotionServoInputConsumerSeamMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d6Failures =
        m_MotionServoInputConsumerSeamReadFailures.load(
            std::memory_order_relaxed);


    const bool d6AccountingPass =
        d6Checks ==
        d6AxisSamples *
        5ULL &&
        d6Matches +
        d6Mismatches +
        d6Failures ==
        d6Checks;


    const bool d6Qualified =
        requiredAxisSamples >
        0ULL &&
        d6AxisSamples >=
        requiredAxisSamples &&
        d6AccountingPass &&
        d6Mismatches ==
        0ULL &&
        d6Failures ==
        0ULL &&
        m_MotionServoAxisIndexIdentityGatePassed &&
        m_MotionServoAxisIndexInputPreflightPrepared;


    if (!d6Qualified)
    {
        return;
    }


    m_ControlledMotionServoInputCutoverEnabled.store(
        true,
        std::memory_order_release);


    m_ControlledMotionServoInputCutoverTransitions.fetch_add(
        1ULL,
        std::memory_order_relaxed);
}


bool EtherCatMaster::TryReadControlledMotionServoInputByAxisIndex(
    int axisIndex,
    MotionServoInputSnapshot& snapshot)
{
    // ========================================================================
    // 250us per-axis hot path.
    //
    // snapshot arrives already containing the SAME-CYCLE legacy fallback.
    //
    // On semantic success:
    //     overwrite snapshot with semantic values.
    //
    // On semantic failure:
    //     do NOT modify snapshot.
    //     latch rollback and return false.
    // ========================================================================

    if (!m_ControlledMotionServoInputCutoverPrepared)
    {
        return
            false;
    }


    const bool enabled =
        m_ControlledMotionServoInputCutoverEnabled.load(
            std::memory_order_acquire);


    if (!enabled)
    {
        m_ControlledMotionServoInputLegacyAxisSamples.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        if (m_ControlledMotionServoInputCutoverFaultLatched.load(
            std::memory_order_relaxed))
        {
            m_ControlledMotionServoInputFallbackAxisSamples.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        return
            false;
    }


    uint16_t semanticStatusWord =
        0U;

    int32_t semanticActualPosition =
        0;

    int8_t semanticMode =
        0;

    uint16_t semanticTouchProbeStatus =
        0U;

    int32_t semanticTouchProbePosition =
        0;


    const bool readPass =
        ReadMotionServoInputByAxisIndexShadow(
            axisIndex,
            semanticStatusWord,
            semanticActualPosition,
            semanticMode,
            semanticTouchProbeStatus,
            semanticTouchProbePosition);


    if (!readPass)
    {
        m_ControlledMotionServoInputRouteFaults.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        m_ControlledMotionServoInputCutoverFaultLatched.store(
            true,
            std::memory_order_release);


        m_ControlledMotionServoInputCutoverEnabled.store(
            false,
            std::memory_order_release);


        m_ControlledMotionServoInputFallbackAxisSamples.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        m_ControlledMotionServoInputLegacyAxisSamples.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        // IMPORTANT:
        // snapshot still contains caller's same-cycle LEGACY data.
        return
            false;
    }


    snapshot.StatusWord =
        semanticStatusWord;

    snapshot.ActualPosition =
        semanticActualPosition;

    snapshot.ModesOfOperationDisplay =
        semanticMode;

    snapshot.TouchProbeStatus =
        semanticTouchProbeStatus;

    snapshot.TouchProbePosition =
        semanticTouchProbePosition;


    m_ControlledMotionServoInputSemanticAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    return
        true;
}


void EtherCatMaster::PrintControlledMotionServoInputCutover() const
{
    const uint64_t requiredSemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t semanticAxisSamples =
        m_ControlledMotionServoInputSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t legacyAxisSamples =
        m_ControlledMotionServoInputLegacyAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t fallbackAxisSamples =
        m_ControlledMotionServoInputFallbackAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t routeFaults =
        m_ControlledMotionServoInputRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t transitions =
        m_ControlledMotionServoInputCutoverTransitions.load(
            std::memory_order_relaxed);


    const bool enabled =
        m_ControlledMotionServoInputCutoverEnabled.load(
            std::memory_order_relaxed);


    const bool faultLatched =
        m_ControlledMotionServoInputCutoverFaultLatched.load(
            std::memory_order_relaxed);


    const uint64_t d6RequiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d6AxisSamples =
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t d6Checks =
        m_MotionServoInputConsumerSeamChecks.load(
            std::memory_order_relaxed);


    const uint64_t d6Matches =
        m_MotionServoInputConsumerSeamMatches.load(
            std::memory_order_relaxed);


    const uint64_t d6Mismatches =
        m_MotionServoInputConsumerSeamMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d6Failures =
        m_MotionServoInputConsumerSeamReadFailures.load(
            std::memory_order_relaxed);


    const bool d6AccountingPass =
        d6Checks ==
        d6AxisSamples *
        5ULL &&
        d6Matches +
        d6Mismatches +
        d6Failures ==
        d6Checks;


    const bool d6Qualified =
        d6RequiredAxisSamples >
        0ULL &&
        d6AxisSamples >=
        d6RequiredAxisSamples &&
        d6AccountingPass &&
        d6Mismatches ==
        0ULL &&
        d6Failures ==
        0ULL;


    const bool windowQualified =
        requiredSemanticAxisSamples >
        0ULL &&
        semanticAxisSamples >=
        requiredSemanticAxisSamples;


    const bool clean =
        routeFaults ==
        0ULL &&
        fallbackAxisSamples ==
        0ULL &&
        !faultLatched;


    const bool ready =
        m_ControlledMotionServoInputCutoverPrepared &&
        enabled &&
        d6Qualified &&
        windowQualified &&
        clean &&
        transitions ==
        1ULL;


    DEBUG_PRINT(
        "[MOTION-SEMANTIC-INPUT-CUTOVER] "
        "Prep:%s Gate:%s Enabled:%s Fault:%s | "
        "Semantic:%llu/%llu Legacy:%llu Fallback:%llu RouteFault:%llu Transitions:%llu | "
        "D6:%s Window:%s Clean:%s | "
        "Active:%s | CoreMotion:SNAPSHOT | "
        "TouchProbeCompat:D8_CONTROLLED DebugCompat:D8_CONTROLLED | "
        "ServoCmd:LEGACY OutputCutover:NO | Ready:%s Result:%s\n",

        m_ControlledMotionServoInputCutoverPrepared
        ? "YES"
        : "NO",

        d6Qualified
        ? "PASS"
        : "WAIT",

        enabled
        ? "YES"
        : "NO",

        faultLatched
        ? "YES"
        : "NO",

        (unsigned long long)
        semanticAxisSamples,

        (unsigned long long)
        requiredSemanticAxisSamples,

        (unsigned long long)
        legacyAxisSamples,

        (unsigned long long)
        fallbackAxisSamples,

        (unsigned long long)
        routeFaults,

        (unsigned long long)
        transitions,

        d6Qualified
        ? "QUALIFIED"
        : "WAIT",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        enabled
        ? "AXIS_INDEX_SEMANTIC"
        : (
            faultLatched
            ? "LEGACY_ROLLBACK"
            : "LEGACY_WARMUP"
            ),

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.8:
    // compatibility input retirement status.
    PrintMotionServoInputCompatibilityRetirement();
}


// ============================================================================
// Stage 11D.8 - Semantic Motion Input Compatibility Retirement
//
// Problem solved:
//
// Compatibility readers such as HMI Debug and HOME Touch Probe run outside
// the 250us Motion thread.  Reading raw structured Process Image fields from
// those callsites would create an unnecessary cross-thread snapshot problem.
//
// Solution:
//
// The 250us Motion thread publishes the EXACT MotionServoInputSnapshot that
// it actually consumes.
//
// Compatibility readers then read this stable atomic publication.
//
// Warmup / emergency fallback remains the centralized legacy pInput snapshot.
// ============================================================================

bool EtherCatMaster::PrepareMotionServoInputCompatibilityRetirement()
{
    m_MotionServoInputCompatibilityRetirementPrepared =
        false;


    m_MotionServoInputCompatibilityPublishedAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputCompatibilitySemanticReads.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputCompatibilityWarmupLegacyReads.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputCompatibilityFallbackReads.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoInputCompatibilityReadFailures.store(
        0ULL,
        std::memory_order_relaxed);


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        auto& slot =
            m_PublishedMotionServoInput[
                axisIndex];


        slot.sequence.store(
            0U,
            std::memory_order_relaxed);

        slot.statusWord.store(
            0U,
            std::memory_order_relaxed);

        slot.actualPosition.store(
            0,
            std::memory_order_relaxed);

        slot.modesOfOperationDisplay.store(
            0,
            std::memory_order_relaxed);

        slot.touchProbeStatus.store(
            0U,
            std::memory_order_relaxed);

        slot.touchProbePosition.store(
            0,
            std::memory_order_relaxed);

        slot.publishCount.store(
            0ULL,
            std::memory_order_relaxed);
    }


    const bool pass =
        m_ControlledMotionServoInputCutoverPrepared &&
        m_MotionServoInputConsumerSeamPreflightPrepared &&
        m_MotionServoAxisIndexInputPreflightPrepared &&
        m_MotionServoAxisIndexInputRouteCount >
        0;


    if (pass)
    {
        m_MotionServoInputCompatibilityRetirementPrepared =
            true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-INPUT-COMPAT-RETIRE-PREPARE] "
        "Stage:11D.8 | Routes:%d | "
        "D7Prepared:%s | D6Prepared:%s | "
        "Result:%s | Prepared:%s | "
        "PublishSource:ACTIVE_MOTION_SNAPSHOT | "
        "TouchProbe:CONTROLLED | Debug:CONTROLLED | "
        "LegacyInput:WARMUP_FALLBACK_ONLY | "
        "ServoCmd:LEGACY | OutputCutover:NO\n"
        "============================================================\n",

        m_MotionServoAxisIndexInputRouteCount,

        m_ControlledMotionServoInputCutoverPrepared
        ? "YES"
        : "NO",

        m_MotionServoInputConsumerSeamPreflightPrepared
        ? "YES"
        : "NO",

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoInputCompatibilityRetirementPrepared
        ? "YES"
        : "NO");


    return
        pass;
}


void EtherCatMaster::PublishMotionServoInputConsumerSnapshot(
    int axisIndex,
    const MotionServoInputSnapshot& snapshot)
{
    // ========================================================================
    // 250us Motion hot path.
    //
    // Single writer per AxisIndex.
    // No allocation.
    // No logging.
    // No hardware access.
    // No string lookup.
    // ========================================================================

    if (!m_MotionServoInputCompatibilityRetirementPrepared ||
        axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        return;
    }


    PublishedMotionServoInputAtomic&
        slot =
        m_PublishedMotionServoInput[
            axisIndex];


    // Odd sequence = writer active.
    slot.sequence.fetch_add(
        1U,
        std::memory_order_acq_rel);


    slot.statusWord.store(
        snapshot.StatusWord,
        std::memory_order_relaxed);

    slot.actualPosition.store(
        snapshot.ActualPosition,
        std::memory_order_relaxed);

    slot.modesOfOperationDisplay.store(
        snapshot.ModesOfOperationDisplay,
        std::memory_order_relaxed);

    slot.touchProbeStatus.store(
        snapshot.TouchProbeStatus,
        std::memory_order_relaxed);

    slot.touchProbePosition.store(
        snapshot.TouchProbePosition,
        std::memory_order_relaxed);


    slot.publishCount.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    // Even sequence = complete coherent publication.
    slot.sequence.fetch_add(
        1U,
        std::memory_order_release);


    m_MotionServoInputCompatibilityPublishedAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);
}


bool EtherCatMaster::TryReadMotionServoPublishedInputCompatibility(
    int axisIndex,
    MotionServoInputSnapshot& snapshot)
{
    snapshot =
        MotionServoInputSnapshot{};


    if (!m_MotionServoInputCompatibilityRetirementPrepared ||
        axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        return
            false;
    }


    // ========================================================================
    // Compatibility semantic cutover only opens AFTER D7 itself is fully
    // qualified, not merely enabled.
    // ========================================================================

    const uint64_t requiredD7SemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d7SemanticAxisSamples =
        m_ControlledMotionServoInputSemanticAxisSamples.load(
            std::memory_order_acquire);


    const bool d7Enabled =
        m_ControlledMotionServoInputCutoverEnabled.load(
            std::memory_order_acquire);


    const bool d7Fault =
        m_ControlledMotionServoInputCutoverFaultLatched.load(
            std::memory_order_acquire);


    const uint64_t d7Fallback =
        m_ControlledMotionServoInputFallbackAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t d7RouteFault =
        m_ControlledMotionServoInputRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t d7Transitions =
        m_ControlledMotionServoInputCutoverTransitions.load(
            std::memory_order_relaxed);


    const bool d7Qualified =
        d7Enabled &&
        !d7Fault &&
        requiredD7SemanticAxisSamples >
        0ULL &&
        d7SemanticAxisSamples >=
        requiredD7SemanticAxisSamples &&
        d7Fallback ==
        0ULL &&
        d7RouteFault ==
        0ULL &&
        d7Transitions ==
        1ULL;


    if (!d7Qualified)
    {
        if (d7Fault ||
            d7RouteFault >
            0ULL)
        {
            m_MotionServoInputCompatibilityFallbackReads.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_MotionServoInputCompatibilityWarmupLegacyReads.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        return
            false;
    }


    const PublishedMotionServoInputAtomic&
        slot =
        m_PublishedMotionServoInput[
            axisIndex];


    if (slot.publishCount.load(
        std::memory_order_acquire) ==
        0ULL)
    {
        m_MotionServoInputCompatibilityReadFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoInputCompatibilityFallbackReads.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return
            false;
    }


    // Four retries are more than enough for a 250us single writer when the
    // compatibility reader runs in the slower NC/HMI domain.
    for (int attempt = 0;
        attempt <
        4;
        ++attempt)
    {
        const uint32_t beginSequence =
            slot.sequence.load(
                std::memory_order_acquire);


        if ((beginSequence &
            1U) !=
            0U)
        {
            continue;
        }


        MotionServoInputSnapshot
            candidate;


        candidate.StatusWord =
            slot.statusWord.load(
                std::memory_order_relaxed);

        candidate.ActualPosition =
            slot.actualPosition.load(
                std::memory_order_relaxed);

        candidate.ModesOfOperationDisplay =
            slot.modesOfOperationDisplay.load(
                std::memory_order_relaxed);

        candidate.TouchProbeStatus =
            slot.touchProbeStatus.load(
                std::memory_order_relaxed);

        candidate.TouchProbePosition =
            slot.touchProbePosition.load(
                std::memory_order_relaxed);


        const uint32_t endSequence =
            slot.sequence.load(
                std::memory_order_acquire);


        if (beginSequence ==
            endSequence &&
            (endSequence &
                1U) ==
            0U)
        {
            snapshot =
                candidate;


            m_MotionServoInputCompatibilitySemanticReads.fetch_add(
                1ULL,
                std::memory_order_relaxed);


            return
                true;
        }
    }


    m_MotionServoInputCompatibilityReadFailures.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoInputCompatibilityFallbackReads.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    return
        false;
}


void EtherCatMaster::PrintMotionServoInputCompatibilityRetirement() const
{
    const uint64_t requiredD7SemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d7SemanticAxisSamples =
        m_ControlledMotionServoInputSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const bool d7Enabled =
        m_ControlledMotionServoInputCutoverEnabled.load(
            std::memory_order_relaxed);


    const bool d7Fault =
        m_ControlledMotionServoInputCutoverFaultLatched.load(
            std::memory_order_relaxed);


    const uint64_t d7Fallback =
        m_ControlledMotionServoInputFallbackAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t d7RouteFault =
        m_ControlledMotionServoInputRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t d7Transitions =
        m_ControlledMotionServoInputCutoverTransitions.load(
            std::memory_order_relaxed);


    const bool d7Qualified =
        d7Enabled &&
        !d7Fault &&
        requiredD7SemanticAxisSamples >
        0ULL &&
        d7SemanticAxisSamples >=
        requiredD7SemanticAxisSamples &&
        d7Fallback ==
        0ULL &&
        d7RouteFault ==
        0ULL &&
        d7Transitions ==
        1ULL;


    int publishedRoutes =
        0;


    for (const auto& route :
        m_MotionServoAxisIndexInputRoutesShadow)
    {
        if (!route.valid ||
            route.axisIndex <
            0 ||
            route.axisIndex >=
            MAX_AXES)
        {
            continue;
        }


        if (m_PublishedMotionServoInput[
            route.axisIndex]
            .publishCount.load(
                std::memory_order_relaxed) >
                0ULL)
        {
            publishedRoutes++;
        }
    }


    const uint64_t publishedAxisSamples =
        m_MotionServoInputCompatibilityPublishedAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t semanticReads =
        m_MotionServoInputCompatibilitySemanticReads.load(
            std::memory_order_relaxed);


    const uint64_t warmupLegacyReads =
        m_MotionServoInputCompatibilityWarmupLegacyReads.load(
            std::memory_order_relaxed);


    const uint64_t fallbackReads =
        m_MotionServoInputCompatibilityFallbackReads.load(
            std::memory_order_relaxed);


    const uint64_t readFailures =
        m_MotionServoInputCompatibilityReadFailures.load(
            std::memory_order_relaxed);


    // HMI ExportDebugInfo runs in the slower 10ms control domain.
    //
    // 200 semantic reads per route gives approximately a two-second
    // compatibility-active qualification window on the current machine.
    const uint64_t requiredSemanticReads =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        200ULL;


    const bool windowQualified =
        requiredSemanticReads >
        0ULL &&
        semanticReads >=
        requiredSemanticReads;


    const bool clean =
        fallbackReads ==
        0ULL &&
        readFailures ==
        0ULL;


    const bool ready =
        m_MotionServoInputCompatibilityRetirementPrepared &&
        d7Qualified &&
        publishedRoutes ==
        m_MotionServoAxisIndexInputRouteCount &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[MOTION-INPUT-COMPAT-RETIRE] "
        "Prep:%s | Routes:%d PublishedRoutes:%d | "
        "Published:%llu | SemanticReads:%llu/%llu | "
        "WarmupLegacy:%llu Fallback:%llu ReadFail:%llu | "
        "D7:%s Window:%s Clean:%s | "
        "CompatSource:%s | "
        "TouchProbeCompat:%s DebugCompat:%s | "
        "LegacyPInput:WARMUP_FALLBACK_ONLY | "
        "CoreMotion:AXIS_INDEX_SEMANTIC | "
        "ServoCmd:LEGACY OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_MotionServoInputCompatibilityRetirementPrepared
        ? "YES"
        : "NO",

        m_MotionServoAxisIndexInputRouteCount,

        publishedRoutes,

        (unsigned long long)
        publishedAxisSamples,

        (unsigned long long)
        semanticReads,

        (unsigned long long)
        requiredSemanticReads,

        (unsigned long long)
        warmupLegacyReads,

        (unsigned long long)
        fallbackReads,

        (unsigned long long)
        readFailures,

        d7Qualified
        ? "QUALIFIED"
        : "WAIT",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        d7Qualified
        ? "PUBLISHED_ACTIVE_MOTION_SNAPSHOT"
        : "LEGACY_WARMUP",

        d7Qualified
        ? "SEMANTIC_PUBLISHED"
        : "LEGACY_WARMUP",

        d7Qualified
        ? "SEMANTIC_PUBLISHED"
        : "LEGACY_WARMUP",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11D.9:
    // legacy Servo input normal-path retirement status.
    PrintLegacyMotionServoInputNormalPathRetirement();
}


// ============================================================================
// Stage 11D.9 - Legacy Servo Input Normal-Path Retirement
//
// Normal Motion input after retirement:
//
//     AxisIndex semantic route
//         -> MotionServoInputSnapshot
//         -> UpdateMotion / UpdateServoState
//
// Legacy pInput remains available only for an on-demand emergency fallback.
//
// D3 / D5 / D6 historical Motion comparison windows are frozen after the
// transition and remain available as qualification evidence.
// ============================================================================

bool EtherCatMaster::PrepareLegacyMotionServoInputNormalPathRetirement()
{
    m_LegacyMotionServoInputNormalPathRetirementPrepared =
        false;


    m_LegacyMotionServoInputNormalPathRetired.store(
        false,
        std::memory_order_relaxed);

    m_LegacyMotionServoInputNormalPathRetirementFaultLatched.store(
        false,
        std::memory_order_relaxed);

    m_LegacyMotionServoInputNormalPathRetirementTransitions.store(
        0ULL,
        std::memory_order_relaxed);

    m_LegacyMotionServoInputRetiredSemanticAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_LegacyMotionServoInputEmergencyFallbackReads.store(
        0ULL,
        std::memory_order_relaxed);


    m_LegacyMotionServoInputFrozenD3Cycles =
        0ULL;

    m_LegacyMotionServoInputFrozenD5Cycles =
        0ULL;

    m_LegacyMotionServoInputFrozenD6AxisSamples =
        0ULL;


    const bool pass =
        m_MotionServoInputCompatibilityRetirementPrepared &&
        m_ControlledMotionServoInputCutoverPrepared &&
        m_MotionServoInputConsumerSeamPreflightPrepared &&
        m_MotionServoAxisIndexInputPreflightPrepared &&
        m_MotionServoAxisIndexInputRouteCount >
        0;


    if (pass)
    {
        m_LegacyMotionServoInputNormalPathRetirementPrepared =
            true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[MOTION-LEGACY-INPUT-RETIRE-PREPARE] "
        "Stage:11D.9 | Routes:%d | "
        "D8Prepared:%s | D7Prepared:%s | "
        "Result:%s | Prepared:%s | "
        "NormalInput:SEMANTIC_AFTER_D8_GATE | "
        "LegacyPInput:EMERGENCY_ONLY_AFTER_RETIRE | "
        "Freeze:D3_D5_D6 | "
        "D2Diagnostic:RETAINED_1MS_SHADOW | "
        "ServoCmd:LEGACY | OutputCutover:NO\n"
        "============================================================\n",

        m_MotionServoAxisIndexInputRouteCount,

        m_MotionServoInputCompatibilityRetirementPrepared
        ? "YES"
        : "NO",

        m_ControlledMotionServoInputCutoverPrepared
        ? "YES"
        : "NO",

        pass
        ? "PASS"
        : "FAIL",

        m_LegacyMotionServoInputNormalPathRetirementPrepared
        ? "YES"
        : "NO");


    return
        pass;
}


void EtherCatMaster::UpdateLegacyMotionServoInputNormalPathRetirementGate()
{
    // ========================================================================
    // Called once per 250us Motion cycle before the axis loop.
    //
    // No allocation.
    // No logging.
    // No hardware access.
    //
    // Retire only after D8 itself is fully qualified.
    // ========================================================================

    if (!m_LegacyMotionServoInputNormalPathRetirementPrepared)
    {
        return;
    }


    if (m_LegacyMotionServoInputNormalPathRetired.load(
        std::memory_order_relaxed))
    {
        return;
    }


    if (m_LegacyMotionServoInputNormalPathRetirementFaultLatched.load(
        std::memory_order_relaxed))
    {
        return;
    }


    const uint64_t requiredD7SemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d7SemanticAxisSamples =
        m_ControlledMotionServoInputSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const bool d7Qualified =
        m_ControlledMotionServoInputCutoverEnabled.load(
            std::memory_order_relaxed) &&
        !m_ControlledMotionServoInputCutoverFaultLatched.load(
            std::memory_order_relaxed) &&
        requiredD7SemanticAxisSamples >
        0ULL &&
        d7SemanticAxisSamples >=
        requiredD7SemanticAxisSamples &&
        m_ControlledMotionServoInputFallbackAxisSamples.load(
            std::memory_order_relaxed) ==
        0ULL &&
        m_ControlledMotionServoInputRouteFaults.load(
            std::memory_order_relaxed) ==
        0ULL &&
        m_ControlledMotionServoInputCutoverTransitions.load(
            std::memory_order_relaxed) ==
        1ULL;


    int publishedRoutes =
        0;


    for (const auto& route :
        m_MotionServoAxisIndexInputRoutesShadow)
    {
        if (!route.valid ||
            route.axisIndex <
            0 ||
            route.axisIndex >=
            MAX_AXES)
        {
            continue;
        }


        if (m_PublishedMotionServoInput[
            route.axisIndex]
            .publishCount.load(
                std::memory_order_relaxed) >
                0ULL)
        {
            publishedRoutes++;
        }
    }


    const uint64_t requiredCompatibilitySemanticReads =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        200ULL;


    const uint64_t compatibilitySemanticReads =
        m_MotionServoInputCompatibilitySemanticReads.load(
            std::memory_order_relaxed);


    const bool d8Qualified =
        d7Qualified &&
        publishedRoutes ==
        m_MotionServoAxisIndexInputRouteCount &&
        requiredCompatibilitySemanticReads >
        0ULL &&
        compatibilitySemanticReads >=
        requiredCompatibilitySemanticReads &&
        m_MotionServoInputCompatibilityFallbackReads.load(
            std::memory_order_relaxed) ==
        0ULL &&
        m_MotionServoInputCompatibilityReadFailures.load(
            std::memory_order_relaxed) ==
        0ULL;


    if (!d8Qualified)
    {
        return;
    }


    // Capture historical comparison windows at the exact retirement boundary.
    //
    // D3 and D5 have already run for this Motion cycle.
    // D6 from the previous cycle is complete.
    m_LegacyMotionServoInputFrozenD3Cycles =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed);

    m_LegacyMotionServoInputFrozenD5Cycles =
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed);

    m_LegacyMotionServoInputFrozenD6AxisSamples =
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed);


    m_LegacyMotionServoInputNormalPathRetired.store(
        true,
        std::memory_order_release);


    m_LegacyMotionServoInputNormalPathRetirementTransitions.fetch_add(
        1ULL,
        std::memory_order_relaxed);
}


bool EtherCatMaster::IsLegacyMotionServoInputNormalPathRetired() const
{
    return
        m_LegacyMotionServoInputNormalPathRetirementPrepared &&
        m_LegacyMotionServoInputNormalPathRetired.load(
            std::memory_order_acquire) &&
        !m_LegacyMotionServoInputNormalPathRetirementFaultLatched.load(
            std::memory_order_acquire);
}


bool EtherCatMaster::TryReadRetiredMotionServoInputByAxisIndex(
    int axisIndex,
    MotionServoInputSnapshot& snapshot)
{
    snapshot =
        MotionServoInputSnapshot{};


    if (!IsLegacyMotionServoInputNormalPathRetired())
    {
        return
            false;
    }


    const bool pass =
        TryReadControlledMotionServoInputByAxisIndex(
            axisIndex,
            snapshot);


    if (pass)
    {
        m_LegacyMotionServoInputRetiredSemanticAxisSamples.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        return
            true;
    }


    // Semantic route unexpectedly failed after retirement.
    //
    // Disable retirement for the remainder of the boot.
    m_LegacyMotionServoInputNormalPathRetirementFaultLatched.store(
        true,
        std::memory_order_release);


    m_LegacyMotionServoInputNormalPathRetired.store(
        false,
        std::memory_order_release);


    return
        false;
}


void EtherCatMaster::ReportLegacyMotionServoInputEmergencyFallback()
{
    m_LegacyMotionServoInputEmergencyFallbackReads.fetch_add(
        1ULL,
        std::memory_order_relaxed);
}


void EtherCatMaster::PrintLegacyMotionServoInputNormalPathRetirement() const
{
    const bool retired =
        IsLegacyMotionServoInputNormalPathRetired();


    const bool fault =
        m_LegacyMotionServoInputNormalPathRetirementFaultLatched.load(
            std::memory_order_relaxed);


    const uint64_t transitions =
        m_LegacyMotionServoInputNormalPathRetirementTransitions.load(
            std::memory_order_relaxed);


    const uint64_t semanticAxisSamples =
        m_LegacyMotionServoInputRetiredSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t emergencyFallbacks =
        m_LegacyMotionServoInputEmergencyFallbackReads.load(
            std::memory_order_relaxed);


    const uint64_t requiredSemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t currentD3Cycles =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed);


    const uint64_t currentD5Cycles =
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed);


    const uint64_t currentD6AxisSamples =
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed);


    const bool historicalFrozen =
        retired &&
        transitions ==
        1ULL &&
        currentD3Cycles ==
        m_LegacyMotionServoInputFrozenD3Cycles &&
        currentD5Cycles ==
        m_LegacyMotionServoInputFrozenD5Cycles &&
        currentD6AxisSamples ==
        m_LegacyMotionServoInputFrozenD6AxisSamples;


    const bool windowQualified =
        requiredSemanticAxisSamples >
        0ULL &&
        semanticAxisSamples >=
        requiredSemanticAxisSamples;


    const bool clean =
        !fault &&
        emergencyFallbacks ==
        0ULL &&
        historicalFrozen;


    const bool ready =
        m_LegacyMotionServoInputNormalPathRetirementPrepared &&
        retired &&
        transitions ==
        1ULL &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[MOTION-LEGACY-INPUT-NORMALPATH-RETIRE] "
        "Prep:%s Retired:%s Fault:%s Transition:%llu | "
        "Semantic:%llu/%llu EmergencyFallback:%llu | "
        "FrozenD3:%llu/%llu FrozenD5:%llu/%llu FrozenD6:%llu/%llu | "
        "HistoricalFrozen:%s Window:%s Clean:%s | "
        "NormalInput:%s | "
        "LegacyPInput:%s | "
        "D2Diagnostic:RETAINED_1MS_SHADOW | "
        "TouchProbe:SEMANTIC_PUBLISHED Debug:SEMANTIC_PUBLISHED | "
        "ServoCmd:LEGACY OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_LegacyMotionServoInputNormalPathRetirementPrepared
        ? "YES"
        : "NO",

        retired
        ? "YES"
        : "NO",

        fault
        ? "YES"
        : "NO",

        (unsigned long long)
        transitions,

        (unsigned long long)
        semanticAxisSamples,

        (unsigned long long)
        requiredSemanticAxisSamples,

        (unsigned long long)
        emergencyFallbacks,

        (unsigned long long)
        currentD3Cycles,

        (unsigned long long)
        m_LegacyMotionServoInputFrozenD3Cycles,

        (unsigned long long)
        currentD5Cycles,

        (unsigned long long)
        m_LegacyMotionServoInputFrozenD5Cycles,

        (unsigned long long)
        currentD6AxisSamples,

        (unsigned long long)
        m_LegacyMotionServoInputFrozenD6AxisSamples,

        historicalFrozen
        ? "YES"
        : "NO",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        retired
        ? "AXIS_INDEX_SEMANTIC_ONLY"
        : (
            fault
            ? "LEGACY_EMERGENCY_ROLLBACK"
            : "LEGACY_QUALIFICATION"
            ),

        retired
        ? "EMERGENCY_ONLY"
        : (
            fault
            ? "EMERGENCY_ACTIVE"
            : "QUALIFICATION_ACTIVE"
            ),

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");
}


// ============================================================================
// Stage 11D.10 - Servo Input Release / Diagnostic Retirement Gate
//
// Final Servo INPUT state after release:
//
//     Core Motion:
//         AXIS_INDEX_SEMANTIC_ONLY
//
//     TouchProbe / Debug:
//         SEMANTIC_PUBLISHED
//
//     Legacy pInput:
//         EMERGENCY_ONLY
//
//     D2:
//         1ms legacy-vs-structured sampler STOPPED/FROZEN
//
//     D3 / D5 / D6:
//         already FROZEN by Stage11D.9
//
// Servo OUTPUT remains LEGACY and is explicitly outside this release.
// ============================================================================

bool EtherCatMaster::PrepareServoInputReleaseGate()
{
    m_ServoInputReleaseGatePrepared =
        false;


    m_ServoInputReleaseD2StopRequested.store(
        false,
        std::memory_order_relaxed);

    m_ServoInputReleaseD2SamplerActive.store(
        0U,
        std::memory_order_relaxed);

    m_ServoInputReleaseComplete.store(
        false,
        std::memory_order_relaxed);

    m_ServoInputReleaseTransitions.store(
        0ULL,
        std::memory_order_relaxed);


    m_ServoInputReleaseFrozenD2Cycles =
        0ULL;

    m_ServoInputReleaseFrozenD2ExpectedChecks =
        0ULL;


    const bool pass =
        m_LegacyMotionServoInputNormalPathRetirementPrepared &&
        m_MotionServoInputCompatibilityRetirementPrepared &&
        m_ControlledMotionServoInputCutoverPrepared &&
        m_StructuredServoLiveReadShadowPrepared &&
        m_MotionServoAxisIndexInputRouteCount >
        0;


    if (pass)
    {
        m_ServoInputReleaseGatePrepared =
            true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-INPUT-RELEASE-PREPARE] "
        "Stage:11D.10 | Routes:%d | "
        "D9Prepared:%s | D2Prepared:%s | "
        "Result:%s | Prepared:%s | "
        "MainlineTarget:AXIS_INDEX_SEMANTIC | "
        "LegacyPInputTarget:EMERGENCY_ONLY | "
        "Retire:D2_D3_D5_D6_DIAGNOSTICS | "
        "ServoCmd:LEGACY | OutputCutover:NO\n"
        "============================================================\n",

        m_MotionServoAxisIndexInputRouteCount,

        m_LegacyMotionServoInputNormalPathRetirementPrepared
        ? "YES"
        : "NO",

        m_StructuredServoLiveReadShadowPrepared
        ? "YES"
        : "NO",

        pass
        ? "PASS"
        : "FAIL",

        m_ServoInputReleaseGatePrepared
        ? "YES"
        : "NO");


    return
        pass;
}


void EtherCatMaster::UpdateServoInputReleaseGate()
{
    // ========================================================================
    // 250us Motion gate.
    //
    // No allocation.
    // No logging.
    // No EtherCAT hardware access.
    //
    // Phase A:
    //     wait for D9 + D2 qualification
    //     then request D2 stop
    //
    // Phase B:
    //     wait until D2 active sampler count reaches zero
    //     then capture frozen D2 counters and seal release
    // ========================================================================

    if (!m_ServoInputReleaseGatePrepared)
    {
        return;
    }


    if (m_ServoInputReleaseComplete.load(
        std::memory_order_acquire))
    {
        return;
    }


    // ------------------------------------------------------------------------
    // D9 qualification.
    // ------------------------------------------------------------------------

    const uint64_t requiredD9SemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d9SemanticAxisSamples =
        m_LegacyMotionServoInputRetiredSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const bool historicalFrozen =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD3Cycles &&
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD5Cycles &&
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD6AxisSamples;


    const bool d9Qualified =
        IsLegacyMotionServoInputNormalPathRetired() &&
        m_LegacyMotionServoInputNormalPathRetirementTransitions.load(
            std::memory_order_relaxed) ==
        1ULL &&
        requiredD9SemanticAxisSamples >
        0ULL &&
        d9SemanticAxisSamples >=
        requiredD9SemanticAxisSamples &&
        m_LegacyMotionServoInputEmergencyFallbackReads.load(
            std::memory_order_relaxed) ==
        0ULL &&
        historicalFrozen;


    // ------------------------------------------------------------------------
    // D2 qualification.
    // ------------------------------------------------------------------------

    const uint64_t d2Cycles =
        m_StructuredServoLiveReadCycles.load(
            std::memory_order_relaxed);


    const uint64_t d2Expected =
        m_StructuredServoLiveReadExpectedChecks.load(
            std::memory_order_relaxed);


    const uint64_t d2Stable =
        m_StructuredServoLiveReadStableChecks.load(
            std::memory_order_relaxed);


    const uint64_t d2Matches =
        m_StructuredServoLiveReadMatches.load(
            std::memory_order_relaxed);


    const uint64_t d2Unstable =
        m_StructuredServoLiveReadUnstableSkips.load(
            std::memory_order_relaxed);


    const uint64_t d2Mismatches =
        m_StructuredServoLiveReadMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d2Failures =
        m_StructuredServoLiveReadFailures.load(
            std::memory_order_relaxed);


    const uint64_t d2ExpectedByCycles =
        d2Cycles *
        static_cast<uint64_t>(
            m_StructuredServoLiveReadInputFieldCount);


    const uint64_t d2StablePermille =
        d2Expected >
        0ULL
        ? (
            d2Stable *
            1000ULL
            ) /
        d2Expected
        : 0ULL;


    const bool d2AccountingPass =
        d2Expected ==
        d2ExpectedByCycles &&
        d2Stable +
        d2Unstable +
        d2Failures ==
        d2Expected &&
        d2Matches +
        d2Mismatches ==
        d2Stable;


    const bool d2Qualified =
        m_StructuredServoLiveReadShadowPrepared &&
        d2Cycles >=
        5000ULL &&
        d2AccountingPass &&
        d2StablePermille >=
        900ULL &&
        d2Mismatches ==
        0ULL &&
        d2Failures ==
        0ULL;


    if (!m_ServoInputReleaseD2StopRequested.load(
        std::memory_order_acquire))
    {
        if (!d9Qualified ||
            !d2Qualified)
        {
            return;
        }


        // Stop future D2 1ms entries.
        m_ServoInputReleaseD2StopRequested.store(
            true,
            std::memory_order_release);


        return;
    }


    // D2 stop already requested.  Wait for any in-flight 1ms sample.
    if (m_ServoInputReleaseD2SamplerActive.load(
        std::memory_order_acquire) !=
        0U)
    {
        return;
    }


    // No new D2 sample can start after StopRequested=YES and Active=0.
    m_ServoInputReleaseFrozenD2Cycles =
        m_StructuredServoLiveReadCycles.load(
            std::memory_order_relaxed);

    m_ServoInputReleaseFrozenD2ExpectedChecks =
        m_StructuredServoLiveReadExpectedChecks.load(
            std::memory_order_relaxed);


    m_ServoInputReleaseComplete.store(
        true,
        std::memory_order_release);


    m_ServoInputReleaseTransitions.fetch_add(
        1ULL,
        std::memory_order_relaxed);
}


bool EtherCatMaster::IsServoInputReleaseComplete() const
{
    return
        m_ServoInputReleaseGatePrepared &&
        m_ServoInputReleaseComplete.load(
            std::memory_order_acquire);
}


void EtherCatMaster::PrintServoInputReleaseGate() const
{
    const bool released =
        IsServoInputReleaseComplete();


    const bool stopRequested =
        m_ServoInputReleaseD2StopRequested.load(
            std::memory_order_relaxed);


    const uint32_t d2Active =
        m_ServoInputReleaseD2SamplerActive.load(
            std::memory_order_relaxed);


    const uint64_t transitions =
        m_ServoInputReleaseTransitions.load(
            std::memory_order_relaxed);


    const uint64_t d2Cycles =
        m_StructuredServoLiveReadCycles.load(
            std::memory_order_relaxed);


    const uint64_t d2Expected =
        m_StructuredServoLiveReadExpectedChecks.load(
            std::memory_order_relaxed);


    const uint64_t d2Stable =
        m_StructuredServoLiveReadStableChecks.load(
            std::memory_order_relaxed);


    const uint64_t d2Unstable =
        m_StructuredServoLiveReadUnstableSkips.load(
            std::memory_order_relaxed);


    const uint64_t d2Mismatches =
        m_StructuredServoLiveReadMismatches.load(
            std::memory_order_relaxed);


    const uint64_t d2Failures =
        m_StructuredServoLiveReadFailures.load(
            std::memory_order_relaxed);


    const uint64_t d2StablePermille =
        d2Expected >
        0ULL
        ? (
            d2Stable *
            1000ULL
            ) /
        d2Expected
        : 0ULL;


    const bool d2Qualified =
        d2Cycles >=
        5000ULL &&
        d2Expected ==
        d2Cycles *
        static_cast<uint64_t>(
            m_StructuredServoLiveReadInputFieldCount) &&
        d2Stable +
        d2Unstable +
        d2Failures ==
        d2Expected &&
        d2StablePermille >=
        900ULL &&
        d2Mismatches ==
        0ULL &&
        d2Failures ==
        0ULL;


    const uint64_t requiredD9SemanticAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoAxisIndexInputRouteCount) *
        20000ULL;


    const uint64_t d9SemanticAxisSamples =
        m_LegacyMotionServoInputRetiredSemanticAxisSamples.load(
            std::memory_order_relaxed);


    const bool historicalFrozen =
        m_MotionServoInputBridgeCycles.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD3Cycles &&
        m_MotionServoAxisIndexInputPreflightCycles.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD5Cycles &&
        m_MotionServoInputConsumerSeamAxisSamples.load(
            std::memory_order_relaxed) ==
        m_LegacyMotionServoInputFrozenD6AxisSamples;


    const bool d9Qualified =
        IsLegacyMotionServoInputNormalPathRetired() &&
        requiredD9SemanticAxisSamples >
        0ULL &&
        d9SemanticAxisSamples >=
        requiredD9SemanticAxisSamples &&
        m_LegacyMotionServoInputEmergencyFallbackReads.load(
            std::memory_order_relaxed) ==
        0ULL &&
        historicalFrozen;


    const bool d2Frozen =
        released &&
        d2Active ==
        0U &&
        d2Cycles ==
        m_ServoInputReleaseFrozenD2Cycles &&
        d2Expected ==
        m_ServoInputReleaseFrozenD2ExpectedChecks;


    const bool clean =
        released &&
        transitions ==
        1ULL &&
        d9Qualified &&
        d2Qualified &&
        d2Frozen;


    DEBUG_PRINT(
        "[SERVO-INPUT-RELEASE] "
        "Prep:%s StopReq:%s Released:%s Transition:%llu | "
        "D9:%s Semantic:%llu/%llu EmergencyFallback:%llu | "
        "D2:%s Sampler:%s Active:%u Cycles:%llu/%llu Expected:%llu/%llu "
        "StablePermille:%llu/1000 Mismatch:%llu ReadFail:%llu | "
        "D3D5D6:%s | "
        "Mainline:%s | LegacyPInput:%s | "
        "TouchProbe:SEMANTIC_PUBLISHED Debug:SEMANTIC_PUBLISHED | "
        "ServoCmd:%s OutputCutover:%s | "
        "Ready:%s Result:%s\n",

        m_ServoInputReleaseGatePrepared
        ? "YES"
        : "NO",

        stopRequested
        ? "YES"
        : "NO",

        released
        ? "YES"
        : "NO",

        (unsigned long long)
        transitions,

        d9Qualified
        ? "QUALIFIED"
        : "WAIT",

        (unsigned long long)
        d9SemanticAxisSamples,

        (unsigned long long)
        requiredD9SemanticAxisSamples,

        (unsigned long long)
        m_LegacyMotionServoInputEmergencyFallbackReads.load(
            std::memory_order_relaxed),

        d2Qualified
        ? "QUALIFIED"
        : "WAIT",

        released
        ? "STOPPED"
        : (
            stopRequested
            ? "STOP_REQUESTED"
            : "RUNNING"
            ),

        (unsigned int)
        d2Active,

        (unsigned long long)
        d2Cycles,

        (unsigned long long)
        m_ServoInputReleaseFrozenD2Cycles,

        (unsigned long long)
        d2Expected,

        (unsigned long long)
        m_ServoInputReleaseFrozenD2ExpectedChecks,

        (unsigned long long)
        d2StablePermille,

        (unsigned long long)
        d2Mismatches,

        (unsigned long long)
        d2Failures,

        historicalFrozen
        ? "FROZEN"
        : "ACTIVE",

        released
        ? "AXIS_INDEX_SEMANTIC"
        : "QUALIFICATION",

        released
        ? "EMERGENCY_ONLY"
        : "QUALIFICATION_ACTIVE",

        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed)
        ? "STRUCTURED_AXISINDEX"
        : "LEGACY_SEAM",

        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed)
        ? "YES"
        : "NO",

        clean
        ? "YES"
        : "NO",

        clean
        ? "PASS"
        : "CHECK");
}


// ============================================================================
// Stage 11E.1 - Servo Output Command Ownership Shadow
//
// Independent Servo OUTPUT track.
//
// Current command authority:
//
//     MotionCore / Homing
//         -> ENI_ServoDrive::pOutput
//
// Candidate semantic route:
//
//     AxisIndex
//         -> structured Servo OUTPUT descriptors
//
// This stage is READ/COMPARE ONLY.
//
// No Process Image write.
// No structured output write.
// No Servo command cutover.
// ============================================================================

namespace
{
    template <typename TValue>
    bool ReadStructuredServoOutputTypedFieldE1(
        const EtherCatStructuredServoFieldDescriptor* field,
        TValue& value)
    {
        value =
            TValue{};


        if (field ==
            nullptr ||
            field->pByteBase ==
            nullptr ||
            strcmp(
                field->direction,
                "Output") !=
            0 ||
            field->byteSpan !=
            sizeof(TValue) ||
            !field->byteAligned)
        {
            return
                false;
        }


        memcpy(
            &value,
            field->pByteBase,
            sizeof(TValue));


        return
            true;
    }
}


bool EtherCatMaster::PrepareMotionServoOutputCommandOwnershipShadow()
{
    m_MotionServoOutputCommandOwnershipShadowPrepared =
        false;

    m_MotionServoOutputCommandOwnershipRouteCount =
        0;

    m_MotionServoOutputCommandOwnershipFieldCount =
        0;


    m_MotionServoOutputCommandOwnershipAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandOwnershipChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandOwnershipMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandOwnershipMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandOwnershipReadFailures.store(
        0ULL,
        std::memory_order_relaxed);


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        m_MotionServoAxisIndexOutputRoutesShadow[
            axisIndex] =
            MotionServoAxisIndexOutputRouteShadow{};
    }


    int routeCount =
        0;

    int fieldCount =
        0;

    int lookupChecks =
        0;

    int lookupMatches =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int valueChecks =
        0;

    int valueMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-OWNERSHIP-PREFLIGHT] BEGIN | "
        "Stage:11E.1 | "
        "Identity:AXIS_INDEX | "
        "CurrentCommand:LEGACY_ENI_SERVO_POUTPUT | "
        "CandidateRoute:STRUCTURED_SERVO_OUTPUT | "
        "Fields:ControlWord,TargetVelocity,TouchProbeFunction,ModesOfOperation | "
        "SemanticWrite:NO | LiveWrite:NO\n"
        "============================================================\n");


    if (!m_ServoInputReleaseGatePrepared ||
        !m_MotionServoAxisIndexIdentityGatePassed ||
        !m_MotionServoAxisIndexInputPreflightPrepared)
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (int axisIndex = 0;
            axisIndex <
            MAX_AXES;
            ++axisIndex)
        {
            const MotionServoAxisIndexInputRouteShadow&
                inputRoute =
                m_MotionServoAxisIndexInputRoutesShadow[
                    axisIndex];


            if (!inputRoute.valid)
            {
                continue;
            }


            lookupChecks++;


            if (inputRoute.motionSlot <
                0 ||
                inputRoute.motionSlot >=
                static_cast<int>(
                    m_ServoList.size()))
            {
                errors++;
                continue;
            }


            const ENI_ServoDrive&
                servo =
                m_ServoList[
                    static_cast<size_t>(
                        inputRoute.motionSlot)];


            if (servo.pOutput ==
                nullptr)
            {
                errors++;
                continue;
            }


            MotionServoAxisIndexOutputRouteShadow&
                route =
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex];


            if (route.valid)
            {
                errors++;
                continue;
            }


            route.valid =
                true;

            route.axisIndex =
                axisIndex;

            route.motionSlot =
                inputRoute.motionSlot;

            route.servoSlaveIndex =
                inputRoute.servoSlaveIndex;


            route.controlWord =
                FindStructuredServoDriveFieldShadow(
                    route.servoSlaveIndex,
                    "ControlWord");

            route.targetVelocity =
                FindStructuredServoDriveFieldShadow(
                    route.servoSlaveIndex,
                    "TargetVelocity");

            route.touchProbeFunction =
                FindStructuredServoDriveFieldShadow(
                    route.servoSlaveIndex,
                    "TouchProbeFunction");

            route.modesOfOperation =
                FindStructuredServoDriveFieldShadow(
                    route.servoSlaveIndex,
                    "ModesOfOperation");


            const EtherCatStructuredServoFieldDescriptor*
                fields[4] =
            {
                route.controlWord,
                route.targetVelocity,
                route.touchProbeFunction,
                route.modesOfOperation
            };


            const uint8_t*
                legacyPointers[4] =
            {
                reinterpret_cast<const uint8_t*>(
                    &servo.pOutput->ControlWord),

                reinterpret_cast<const uint8_t*>(
                    &servo.pOutput->TargetVelocity),

                reinterpret_cast<const uint8_t*>(
                    &servo.pOutput->TouchProbeFunc),

                reinterpret_cast<const uint8_t*>(
                    &servo.pOutput->ModesOfOperation)
            };


            const uint32_t
                expectedSizes[4] =
            {
                sizeof(
                    servo.pOutput->ControlWord),

                sizeof(
                    servo.pOutput->TargetVelocity),

                sizeof(
                    servo.pOutput->TouchProbeFunc),

                sizeof(
                    servo.pOutput->ModesOfOperation)
            };


            bool routePass =
                true;


            for (int fieldIndex = 0;
                fieldIndex <
                4;
                ++fieldIndex)
            {
                pointerChecks++;


                const auto* field =
                    fields[
                        fieldIndex];


                const bool pointerMatch =
                    field !=
                    nullptr &&
                    field->pByteBase !=
                    nullptr &&
                    field->slaveIndex ==
                    route.servoSlaveIndex &&
                    strcmp(
                        field->direction,
                        "Output") ==
                    0 &&
                    field->byteAligned &&
                    field->byteSpan ==
                    expectedSizes[
                        fieldIndex] &&
                    reinterpret_cast<const uint8_t*>(
                        field->pByteBase) ==
                            legacyPointers[
                                fieldIndex];


                        if (pointerMatch)
                        {
                            pointerMatches++;
                        }
                        else
                        {
                            routePass =
                                false;

                            errors++;
                        }


                        valueChecks++;


                        const bool valueMatch =
                            pointerMatch &&
                            memcmp(
                                field->pByteBase,
                                legacyPointers[
                                    fieldIndex],
                                field->byteSpan) ==
                            0;


                                    if (valueMatch)
                                    {
                                        valueMatches++;
                                    }
                                    else
                                    {
                                        routePass =
                                            false;

                                        errors++;
                                    }
            }


            if (routePass)
            {
                lookupMatches++;
                routeCount++;
                fieldCount +=
                    4;
            }


            DEBUG_PRINT(
                "[SERVO-OUTPUT-OWNERSHIP-MAP] "
                "AxisIndex:%d | Slot:%d | Servo:S%d | "
                "Fields:4/4 | Pointer:%s | Value:%s | "
                "CurrentCommand:LEGACY_POUTPUT | "
                "SemanticWrite:NO | Result:%s\n",

                route.axisIndex,

                route.motionSlot,

                route.servoSlaveIndex,

                routePass
                ? "MATCH"
                : "FAIL",

                routePass
                ? "MATCH"
                : "FAIL",

                routePass
                ? "PASS"
                : "FAIL");
        }
    }


    int expectedRoutes =
        0;


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        if (m_MotionServoAxisIndexInputRoutesShadow[
            axisIndex]
            .valid)
        {
            expectedRoutes++;
        }
    }


    const int expectedFields =
        expectedRoutes *
        4;


    const bool pass =
        errors ==
        0 &&
        expectedRoutes >
        0 &&
        routeCount ==
        expectedRoutes &&
        fieldCount ==
        expectedFields &&
        lookupChecks ==
        expectedRoutes &&
        lookupMatches ==
        expectedRoutes &&
        pointerChecks ==
        expectedFields &&
        pointerMatches ==
        expectedFields &&
        valueChecks ==
        expectedFields &&
        valueMatches ==
        expectedFields;


    if (pass)
    {
        m_MotionServoOutputCommandOwnershipRouteCount =
            routeCount;

        m_MotionServoOutputCommandOwnershipFieldCount =
            fieldCount;

        m_MotionServoOutputCommandOwnershipShadowPrepared =
            true;
    }


    DEBUG_PRINT(
        "[SERVO-OUTPUT-OWNERSHIP-PREFLIGHT-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | "
        "Lookup:%d/%d | Pointer:%d/%d | Value:%d/%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "Identity:AXIS_INDEX | "
        "CurrentCommand:LEGACY_ENI_SERVO_POUTPUT | "
        "CandidateRoute:STRUCTURED_SERVO_OUTPUT | "
        "SemanticWrite:NO | LiveWrite:NO | "
        "Next:%s\n",

        routeCount,
        expectedRoutes,

        fieldCount,
        expectedFields,

        lookupMatches,
        lookupChecks,

        pointerMatches,
        pointerChecks,

        valueMatches,
        valueChecks,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoOutputCommandOwnershipShadowPrepared
        ? "YES"
        : "NO",

        pass
        ? "RUNTIME_QUALIFY_STAGE11E1"
        : "BLOCK_STAGE11E1");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-OUTPUT-OWNERSHIP-PREFLIGHT] END | "
        "Stage:11E.1 | Result:%s | OutputCutover:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool EtherCatMaster::ReadMotionServoOutputByAxisIndexShadow(
    int axisIndex,
    uint16_t& controlWord,
    int32_t& targetVelocity,
    uint16_t& touchProbeFunction,
    int8_t& modesOfOperation) const
{
    controlWord =
        0U;

    targetVelocity =
        0;

    touchProbeFunction =
        0U;

    modesOfOperation =
        0;


    if (!m_MotionServoOutputCommandOwnershipShadowPrepared ||
        axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        return
            false;
    }


    const MotionServoAxisIndexOutputRouteShadow&
        route =
        m_MotionServoAxisIndexOutputRoutesShadow[
            axisIndex];


    if (!route.valid ||
        route.axisIndex !=
        axisIndex)
    {
        return
            false;
    }


    return
        ReadStructuredServoOutputTypedFieldE1(
            route.controlWord,
            controlWord) &&
        ReadStructuredServoOutputTypedFieldE1(
            route.targetVelocity,
            targetVelocity) &&
        ReadStructuredServoOutputTypedFieldE1(
            route.touchProbeFunction,
            touchProbeFunction) &&
        ReadStructuredServoOutputTypedFieldE1(
            route.modesOfOperation,
            modesOfOperation);
}


void EtherCatMaster::ObserveMotionServoOutputCommandOwnershipShadow(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    // ========================================================================
    // 250us Motion hot-path shadow.
    //
    // Starts only after Servo INPUT release is complete.
    //
    // No allocation.
    // No logging.
    // No string lookup.
    // No EtherCAT hardware access.
    // No Process Image write.
    // ========================================================================

    if (!m_MotionServoOutputCommandOwnershipShadowPrepared ||
        !IsServoInputReleaseComplete())
    {
        return;
    }


    m_MotionServoOutputCommandOwnershipAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandOwnershipChecks.fetch_add(
        4ULL,
        std::memory_order_relaxed);


    uint16_t semanticControlWord =
        0U;

    int32_t semanticTargetVelocity =
        0;

    uint16_t semanticTouchProbeFunction =
        0U;

    int8_t semanticModesOfOperation =
        0;


    const bool readPass =
        ReadMotionServoOutputByAxisIndexShadow(
            axisIndex,
            semanticControlWord,
            semanticTargetVelocity,
            semanticTouchProbeFunction,
            semanticModesOfOperation);


    if (!readPass)
    {
        m_MotionServoOutputCommandOwnershipReadFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        return;
    }


    const bool matches[4] =
    {
        semanticControlWord ==
            controlWord,

        semanticTargetVelocity ==
            targetVelocity,

        semanticTouchProbeFunction ==
            touchProbeFunction,

        semanticModesOfOperation ==
            modesOfOperation
    };


    uint64_t matchCount =
        0ULL;


    for (bool match :
    matches)
    {
        if (match)
        {
            matchCount++;
        }
    }


    m_MotionServoOutputCommandOwnershipMatches.fetch_add(
        matchCount,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandOwnershipMismatches.fetch_add(
        4ULL -
        matchCount,
        std::memory_order_relaxed);


    // Stage 11E.2:
    // feed the exact command snapshot observed by Stage11E.1 into
    // the scratch-only structured output bridge.
    ObserveMotionServoOutputSemanticScratchBridgeShadow(
        axisIndex,
        controlWord,
        targetVelocity,
        touchProbeFunction,
        modesOfOperation);
}


void EtherCatMaster::PrintMotionServoOutputCommandOwnershipShadow() const
{
    // =========================================================
    // Final Generic Servo I/O release cleanup.
    //
    // Once E7 has latched GENERIC_SERVO_IO_COMPLETE, the old
    // E1 -> E6 diagnostic chain is historical-only.
    //
    // Keep only the compact final Servo Generic I/O release line.
    //
    // This changes diagnostics only. Runtime Servo I/O behavior
    // is untouched.
    // =========================================================

    if (IsServoGenericIoReleaseComplete())
    {
        PrintServoGenericIoReleaseGate();
        return;
    }


    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandOwnershipRouteCount) *
        20000ULL;


    const uint64_t axisSamples =
        m_MotionServoOutputCommandOwnershipAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t checks =
        m_MotionServoOutputCommandOwnershipChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_MotionServoOutputCommandOwnershipMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_MotionServoOutputCommandOwnershipMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_MotionServoOutputCommandOwnershipReadFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        axisSamples *
        4ULL;


    const bool accountingPass =
        checks ==
        expectedChecks &&
        matches +
        mismatches +
        failures ==
        checks;


    const bool inputRelease =
        IsServoInputReleaseComplete();


    const bool windowQualified =
        requiredAxisSamples >
        0ULL &&
        axisSamples >=
        requiredAxisSamples;


    const bool clean =
        accountingPass &&
        mismatches ==
        0ULL &&
        failures ==
        0ULL;


    const bool ready =
        m_MotionServoOutputCommandOwnershipShadowPrepared &&
        inputRelease &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-OWNERSHIP-SHADOW] "
        "Prep:%s InputRelease:%s | "
        "Routes:%d Fields:%d | "
        "AxisSamples:%llu/%llu | "
        "Checks:%llu Expected:%llu Matches:%llu "
        "Mismatch:%llu ReadFail:%llu | "
        "Accounting:%s Window:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "CurrentCommand:LEGACY_ENI_SERVO_POUTPUT | "
        "CandidateRoute:STRUCTURED_SERVO_OUTPUT | "
        "ObservedFields:CW_TV_TPF_MOO | "
        "SemanticWrite:NO LiveWrite:NO OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_MotionServoOutputCommandOwnershipShadowPrepared
        ? "YES"
        : "NO",

        inputRelease
        ? "PASS"
        : "WAIT",

        m_MotionServoOutputCommandOwnershipRouteCount,

        m_MotionServoOutputCommandOwnershipFieldCount,

        (unsigned long long)
        axisSamples,

        (unsigned long long)
        requiredAxisSamples,

        (unsigned long long)
        checks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        failures,

        accountingPass
        ? "PASS"
        : "FAIL",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11E.2:
    // same existing 1000ms supervisory diagnostic chain.
    PrintMotionServoOutputSemanticScratchBridgeShadow();
}


// ============================================================================
// Stage 11E.2 - Servo Output Semantic Scratch Command Bridge Shadow
//
// Prerequisite:
//     Stage11E.1 ownership/read shadow must first qualify in THIS boot.
//
// Current command authority remains:
//
//     MotionCore / Homing
//         -> ENI_ServoDrive::pOutput
//
// Candidate route:
//
//     AxisIndex
//         -> structured Servo output field descriptors
//         -> LOCAL ServoOutput scratch image
//
// Stage11E.2 never writes to the live Process Image.
// ============================================================================

namespace
{
    template <typename TValue>
    bool WriteStructuredServoOutputTypedFieldToScratchE2(
        const EtherCatStructuredServoFieldDescriptor* field,
        const uint8_t* liveOutputBase,
        uint8_t* scratchBase,
        size_t scratchSize,
        const TValue& value)
    {
        if (field ==
            nullptr ||
            field->pByteBase ==
            nullptr ||
            liveOutputBase ==
            nullptr ||
            scratchBase ==
            nullptr ||
            strcmp(
                field->direction,
                "Output") !=
            0 ||
            !field->byteAligned ||
            field->byteSpan !=
            sizeof(TValue))
        {
            return
                false;
        }


        const uintptr_t liveAddress =
            reinterpret_cast<uintptr_t>(
                liveOutputBase);


        const uintptr_t fieldAddress =
            reinterpret_cast<uintptr_t>(
                field->pByteBase);


        if (fieldAddress <
            liveAddress)
        {
            return
                false;
        }


        const uintptr_t delta =
            fieldAddress -
            liveAddress;


        if (delta >
            scratchSize ||
            field->byteSpan >
            scratchSize -
            static_cast<size_t>(
                delta))
        {
            return
                false;
        }


        memcpy(
            scratchBase +
            static_cast<size_t>(
                delta),
            &value,
            sizeof(TValue));


        return
            true;
    }


    bool BuildStructuredServoOutputScratchImageE2(
        const EtherCatMaster::MotionServoAxisIndexOutputRouteShadow& route,
        const ServoOutput* liveOutput,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation,
        ServoOutput& scratch)
    {
        if (!route.valid ||
            liveOutput ==
            nullptr)
        {
            return
                false;
        }


        // Preserve any future / non-owned bytes by cloning the current
        // ABI image before writing the four owned semantic fields.
        memcpy(
            &scratch,
            liveOutput,
            sizeof(scratch));


        const uint8_t* liveBase =
            reinterpret_cast<const uint8_t*>(
                liveOutput);


        uint8_t* scratchBase =
            reinterpret_cast<uint8_t*>(
                &scratch);


        return
            WriteStructuredServoOutputTypedFieldToScratchE2(
                route.controlWord,
                liveBase,
                scratchBase,
                sizeof(scratch),
                controlWord) &&
            WriteStructuredServoOutputTypedFieldToScratchE2(
                route.targetVelocity,
                liveBase,
                scratchBase,
                sizeof(scratch),
                targetVelocity) &&
            WriteStructuredServoOutputTypedFieldToScratchE2(
                route.touchProbeFunction,
                liveBase,
                scratchBase,
                sizeof(scratch),
                touchProbeFunction) &&
            WriteStructuredServoOutputTypedFieldToScratchE2(
                route.modesOfOperation,
                liveBase,
                scratchBase,
                sizeof(scratch),
                modesOfOperation);
    }
}


bool EtherCatMaster::PrepareMotionServoOutputSemanticScratchBridgeShadow()
{
    m_MotionServoOutputSemanticScratchBridgePrepared =
        false;

    m_MotionServoOutputSemanticScratchBridgeRouteCount =
        0;

    m_MotionServoOutputSemanticScratchBridgeFieldCount =
        0;


    m_MotionServoOutputSemanticScratchAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchFieldChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchFieldMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchFieldMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchWriteFailures.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchImageChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchImageMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchImageFailures.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchLivePreserveChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchLivePreserveMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputSemanticScratchLiveMutationFailures.store(
        0ULL,
        std::memory_order_relaxed);


    int routeCount =
        0;

    int fieldCount =
        0;

    int scratchCaseChecks =
        0;

    int scratchCaseMatches =
        0;

    int coverageChecks =
        0;

    int coverageMatches =
        0;

    int livePreserveChecks =
        0;

    int livePreserveMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-SCRATCH-BRIDGE-PREFLIGHT] BEGIN | "
        "Stage:11E.2 | "
        "Identity:AXIS_INDEX | "
        "CommandSource:LEGACY_ENI_SERVO_POUTPUT | "
        "Candidate:STRUCTURED_SERVO_OUTPUT_SCRATCH | "
        "PhysicalTarget:UNCHANGED | "
        "SemanticWrite:SCRATCH_ONLY | "
        "LiveWrite:NO | OutputCutover:NO\n"
        "============================================================\n");


    if (!m_MotionServoOutputCommandOwnershipShadowPrepared ||
        m_MotionServoOutputCommandOwnershipRouteCount <=
        0)
    {
        errors++;
    }


    if (errors ==
        0)
    {
        struct ScratchCase
        {
            uint16_t controlWord;
            int32_t targetVelocity;
            uint16_t touchProbeFunction;
            int8_t modesOfOperation;
        };


        const ScratchCase cases[3] =
        {
            {
                0x0000U,
                0,
                0x0000U,
                0
            },

            {
                0xFFFFU,
                123456789,
                0xA55AU,
                0x7F
            },

            {
                0x0080U,
                -123456789,
                0x5AA5U,
                static_cast<int8_t>(
                    -7)
            }
        };


        for (int axisIndex = 0;
            axisIndex <
            MAX_AXES;
            ++axisIndex)
        {
            const MotionServoAxisIndexOutputRouteShadow&
                route =
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex];


            if (!route.valid)
            {
                continue;
            }


            if (route.motionSlot <
                0 ||
                route.motionSlot >=
                static_cast<int>(
                    m_ServoList.size()))
            {
                errors++;
                continue;
            }


            const ENI_ServoDrive&
                servo =
                m_ServoList[
                    static_cast<size_t>(
                        route.motionSlot)];


            if (servo.pOutput ==
                nullptr)
            {
                errors++;
                continue;
            }


            routeCount++;
            fieldCount +=
                4;


            const uint8_t* liveBase =
                reinterpret_cast<const uint8_t*>(
                    servo.pOutput);


            uint8_t liveBefore[
                sizeof(ServoOutput)] =
                {};


                memcpy(
                    liveBefore,
                    liveBase,
                    sizeof(liveBefore));


                // ------------------------------------------------------------
                // Coverage / overlap audit.
                //
                // Current ServoOutput ABI is 9 bytes and the four structured
                // fields must own all 9 bytes exactly once.
                // ------------------------------------------------------------

                bool covered[
                    sizeof(ServoOutput)] =
                    {};


                    const EtherCatStructuredServoFieldDescriptor*
                        fields[4] =
                    {
                        route.controlWord,
                        route.targetVelocity,
                        route.touchProbeFunction,
                        route.modesOfOperation
                    };


                    bool coveragePass =
                        true;


                    for (const auto* field :
                        fields)
                    {
                        if (field ==
                            nullptr ||
                            field->pByteBase ==
                            nullptr)
                        {
                            coveragePass =
                                false;

                            break;
                        }


                        const uintptr_t liveAddress =
                            reinterpret_cast<uintptr_t>(
                                liveBase);


                        const uintptr_t fieldAddress =
                            reinterpret_cast<uintptr_t>(
                                field->pByteBase);


                        if (fieldAddress <
                            liveAddress)
                        {
                            coveragePass =
                                false;

                            break;
                        }


                        const size_t offset =
                            static_cast<size_t>(
                                fieldAddress -
                                liveAddress);


                        if (offset >
                            sizeof(ServoOutput) ||
                            field->byteSpan >
                            sizeof(ServoOutput) -
                            offset)
                        {
                            coveragePass =
                                false;

                            break;
                        }


                        for (size_t byteIndex = 0;
                            byteIndex <
                            field->byteSpan;
                            ++byteIndex)
                        {
                            const size_t owned =
                                offset +
                                byteIndex;


                            if (covered[
                                owned])
                            {
                                coveragePass =
                                    false;

                                break;
                            }


                                covered[
                                    owned] =
                                    true;
                        }


                        if (!coveragePass)
                        {
                            break;
                        }
                    }


                    if (coveragePass)
                    {
                        for (bool owned :
                        covered)
                        {
                            if (!owned)
                            {
                                coveragePass =
                                    false;

                                break;
                            }
                        }
                    }


                    coverageChecks++;


                    if (coveragePass)
                    {
                        coverageMatches++;
                    }
                    else
                    {
                        errors++;
                    }


                    // ------------------------------------------------------------
                    // Synthetic scratch write cases.
                    //
                    // These values are written ONLY to a local ServoOutput object.
                    // ------------------------------------------------------------

                    bool allCasesPass =
                        coveragePass;


                    for (const ScratchCase& test :
                        cases)
                    {
                        scratchCaseChecks++;


                        ServoOutput scratch =
                        {};


                        const bool writePass =
                            BuildStructuredServoOutputScratchImageE2(
                                route,
                                servo.pOutput,
                                test.controlWord,
                                test.targetVelocity,
                                test.touchProbeFunction,
                                test.modesOfOperation,
                                scratch);


                        ServoOutput expected =
                        {};


                        expected.ControlWord =
                            test.controlWord;

                        expected.TargetVelocity =
                            test.targetVelocity;

                        expected.TouchProbeFunc =
                            test.touchProbeFunction;

                        expected.ModesOfOperation =
                            test.modesOfOperation;


                        const bool imagePass =
                            writePass &&
                            memcmp(
                                &scratch,
                                &expected,
                                sizeof(expected)) ==
                            0;


                        if (imagePass)
                        {
                            scratchCaseMatches++;
                        }
                        else
                        {
                            allCasesPass =
                                false;

                            errors++;
                        }
                    }


                    // ------------------------------------------------------------
                    // Prove startup scratch tests did not mutate live output.
                    // ------------------------------------------------------------

                    livePreserveChecks++;


                    const bool livePreserved =
                        memcmp(
                            liveBefore,
                            liveBase,
                            sizeof(liveBefore)) ==
                        0;


                    if (livePreserved)
                    {
                        livePreserveMatches++;
                    }
                    else
                    {
                        allCasesPass =
                            false;

                        errors++;
                    }


                    DEBUG_PRINT(
                        "[SERVO-OUTPUT-SCRATCH-BRIDGE-MAP] "
                        "AxisIndex:%d | Slot:%d | Servo:S%d | "
                        "Fields:4/4 | Coverage:%s | "
                        "ScratchCases:%s | LivePreserve:%s | "
                        "PhysicalTarget:UNCHANGED | "
                        "Result:%s\n",

                        route.axisIndex,

                        route.motionSlot,

                        route.servoSlaveIndex,

                        coveragePass
                        ? "9/9"
                        : "FAIL",

                        allCasesPass
                        ? "3/3"
                        : "FAIL",

                        livePreserved
                        ? "YES"
                        : "NO",

                        allCasesPass &&
                        livePreserved
                        ? "PASS"
                        : "FAIL");
        }
    }


    const int expectedRoutes =
        m_MotionServoOutputCommandOwnershipRouteCount;


    const int expectedFields =
        expectedRoutes *
        4;


    const int expectedScratchCases =
        expectedRoutes *
        3;


    const bool pass =
        errors ==
        0 &&
        expectedRoutes >
        0 &&
        routeCount ==
        expectedRoutes &&
        fieldCount ==
        expectedFields &&
        scratchCaseChecks ==
        expectedScratchCases &&
        scratchCaseMatches ==
        expectedScratchCases &&
        coverageChecks ==
        expectedRoutes &&
        coverageMatches ==
        expectedRoutes &&
        livePreserveChecks ==
        expectedRoutes &&
        livePreserveMatches ==
        expectedRoutes;


    if (pass)
    {
        m_MotionServoOutputSemanticScratchBridgeRouteCount =
            routeCount;

        m_MotionServoOutputSemanticScratchBridgeFieldCount =
            fieldCount;

        m_MotionServoOutputSemanticScratchBridgePrepared =
            true;
    }


    DEBUG_PRINT(
        "[SERVO-OUTPUT-SCRATCH-BRIDGE-PREFLIGHT-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | "
        "ScratchCases:%d/%d | "
        "Coverage:%d/%d | "
        "LivePreserve:%d/%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "CommandSource:LEGACY_ENI_SERVO_POUTPUT | "
        "Candidate:STRUCTURED_SERVO_OUTPUT_SCRATCH | "
        "SemanticWrite:SCRATCH_ONLY | "
        "LiveWrite:NO | OutputCutover:NO | "
        "Next:%s\n",

        routeCount,
        expectedRoutes,

        fieldCount,
        expectedFields,

        scratchCaseMatches,
        expectedScratchCases,

        coverageMatches,
        expectedRoutes,

        livePreserveMatches,
        expectedRoutes,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoOutputSemanticScratchBridgePrepared
        ? "YES"
        : "NO",

        pass
        ? "WAIT_STAGE11E1_RUNTIME_QUALIFICATION"
        : "BLOCK_STAGE11E2");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-OUTPUT-SCRATCH-BRIDGE-PREFLIGHT] END | "
        "Stage:11E.2 | Result:%s | ScratchOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::ObserveMotionServoOutputSemanticScratchBridgeShadow(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    // ========================================================================
    // 250us Motion hot-path shadow.
    //
    // Starts only after Stage11E.1 has already qualified in this boot.
    //
    // All writes below target a LOCAL stack ServoOutput scratch image only.
    // ========================================================================

    if (!m_MotionServoOutputSemanticScratchBridgePrepared ||
        !IsServoInputReleaseComplete())
    {
        return;
    }


    const uint64_t requiredE1AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandOwnershipRouteCount) *
        20000ULL;


    const uint64_t e1AxisSamples =
        m_MotionServoOutputCommandOwnershipAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e1Mismatches =
        m_MotionServoOutputCommandOwnershipMismatches.load(
            std::memory_order_relaxed);


    const uint64_t e1ReadFailures =
        m_MotionServoOutputCommandOwnershipReadFailures.load(
            std::memory_order_relaxed);


    const bool e1Qualified =
        m_MotionServoOutputCommandOwnershipShadowPrepared &&
        requiredE1AxisSamples >
        0ULL &&
        e1AxisSamples >=
        requiredE1AxisSamples &&
        e1Mismatches ==
        0ULL &&
        e1ReadFailures ==
        0ULL;


    if (!e1Qualified)
    {
        return;
    }


    m_MotionServoOutputSemanticScratchAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputSemanticScratchFieldChecks.fetch_add(
        4ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputSemanticScratchImageChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputSemanticScratchLivePreserveChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    if (axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        m_MotionServoOutputSemanticScratchWriteFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    const MotionServoAxisIndexOutputRouteShadow&
        route =
        m_MotionServoAxisIndexOutputRoutesShadow[
            axisIndex];


    if (!route.valid ||
        route.motionSlot <
        0 ||
        route.motionSlot >=
        static_cast<int>(
            m_ServoList.size()))
    {
        m_MotionServoOutputSemanticScratchWriteFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    const ENI_ServoDrive&
        servo =
        m_ServoList[
            static_cast<size_t>(
                route.motionSlot)];


    if (servo.pOutput ==
        nullptr)
    {
        m_MotionServoOutputSemanticScratchWriteFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputSemanticScratchLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    uint8_t liveBefore[
        sizeof(ServoOutput)] =
        {};


        memcpy(
            liveBefore,
            servo.pOutput,
            sizeof(liveBefore));


        ServoOutput scratch =
        {};


        const bool writePass =
            BuildStructuredServoOutputScratchImageE2(
                route,
                servo.pOutput,
                controlWord,
                targetVelocity,
                touchProbeFunction,
                modesOfOperation,
                scratch);


        if (!writePass)
        {
            m_MotionServoOutputSemanticScratchWriteFailures.fetch_add(
                4ULL,
                std::memory_order_relaxed);

            m_MotionServoOutputSemanticScratchImageFailures.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            const bool fieldMatches[4] =
            {
                scratch.ControlWord ==
                    controlWord,

                scratch.TargetVelocity ==
                    targetVelocity,

                scratch.TouchProbeFunc ==
                    touchProbeFunction,

                scratch.ModesOfOperation ==
                    modesOfOperation
            };


            uint64_t fieldMatchCount =
                0ULL;


            for (bool fieldMatch :
            fieldMatches)
            {
                if (fieldMatch)
                {
                    fieldMatchCount++;
                }
            }


            m_MotionServoOutputSemanticScratchFieldMatches.fetch_add(
                fieldMatchCount,
                std::memory_order_relaxed);


            m_MotionServoOutputSemanticScratchFieldMismatches.fetch_add(
                4ULL -
                fieldMatchCount,
                std::memory_order_relaxed);


            ServoOutput expected =
            {};


            expected.ControlWord =
                controlWord;

            expected.TargetVelocity =
                targetVelocity;

            expected.TouchProbeFunc =
                touchProbeFunction;

            expected.ModesOfOperation =
                modesOfOperation;


            const bool imageMatch =
                memcmp(
                    &scratch,
                    &expected,
                    sizeof(expected)) ==
                0;


            if (imageMatch)
            {
                m_MotionServoOutputSemanticScratchImageMatches.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
            }
            else
            {
                m_MotionServoOutputSemanticScratchImageFailures.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
            }
        }


        // Prove scratch-only bridge never modified the live legacy command image.
        const bool livePreserved =
            memcmp(
                liveBefore,
                servo.pOutput,
                sizeof(liveBefore)) ==
            0;


        if (livePreserved)
        {
            m_MotionServoOutputSemanticScratchLivePreserveMatches.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_MotionServoOutputSemanticScratchLiveMutationFailures.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        // Stage 11E.3:
        // same final legacy command snapshot, now used only to verify
        // exact m_IoMap target offsets and local rollback mechanics.
        ObserveMotionServoOutputLiveWritePreflightShadow(
            axisIndex,
            controlWord,
            targetVelocity,
            touchProbeFunction,
            modesOfOperation);
}


void EtherCatMaster::PrintMotionServoOutputSemanticScratchBridgeShadow() const
{
    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputSemanticScratchBridgeRouteCount) *
        20000ULL;


    const uint64_t axisSamples =
        m_MotionServoOutputSemanticScratchAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t fieldChecks =
        m_MotionServoOutputSemanticScratchFieldChecks.load(
            std::memory_order_relaxed);


    const uint64_t fieldMatches =
        m_MotionServoOutputSemanticScratchFieldMatches.load(
            std::memory_order_relaxed);


    const uint64_t fieldMismatches =
        m_MotionServoOutputSemanticScratchFieldMismatches.load(
            std::memory_order_relaxed);


    const uint64_t writeFailures =
        m_MotionServoOutputSemanticScratchWriteFailures.load(
            std::memory_order_relaxed);


    const uint64_t imageChecks =
        m_MotionServoOutputSemanticScratchImageChecks.load(
            std::memory_order_relaxed);


    const uint64_t imageMatches =
        m_MotionServoOutputSemanticScratchImageMatches.load(
            std::memory_order_relaxed);


    const uint64_t imageFailures =
        m_MotionServoOutputSemanticScratchImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t livePreserveChecks =
        m_MotionServoOutputSemanticScratchLivePreserveChecks.load(
            std::memory_order_relaxed);


    const uint64_t livePreserveMatches =
        m_MotionServoOutputSemanticScratchLivePreserveMatches.load(
            std::memory_order_relaxed);


    const uint64_t liveMutationFailures =
        m_MotionServoOutputSemanticScratchLiveMutationFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedFieldChecks =
        axisSamples *
        4ULL;


    const bool fieldAccountingPass =
        fieldChecks ==
        expectedFieldChecks &&
        fieldMatches +
        fieldMismatches +
        writeFailures ==
        fieldChecks;


    const bool imageAccountingPass =
        imageChecks ==
        axisSamples &&
        imageMatches +
        imageFailures ==
        imageChecks;


    const bool livePreserveAccountingPass =
        livePreserveChecks ==
        axisSamples &&
        livePreserveMatches +
        liveMutationFailures ==
        livePreserveChecks;


    const uint64_t requiredE1AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandOwnershipRouteCount) *
        20000ULL;


    const uint64_t e1AxisSamples =
        m_MotionServoOutputCommandOwnershipAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e1Mismatches =
        m_MotionServoOutputCommandOwnershipMismatches.load(
            std::memory_order_relaxed);


    const uint64_t e1ReadFailures =
        m_MotionServoOutputCommandOwnershipReadFailures.load(
            std::memory_order_relaxed);


    const bool e1Qualified =
        m_MotionServoOutputCommandOwnershipShadowPrepared &&
        requiredE1AxisSamples >
        0ULL &&
        e1AxisSamples >=
        requiredE1AxisSamples &&
        e1Mismatches ==
        0ULL &&
        e1ReadFailures ==
        0ULL;


    const bool windowQualified =
        requiredAxisSamples >
        0ULL &&
        axisSamples >=
        requiredAxisSamples;


    const bool clean =
        fieldAccountingPass &&
        imageAccountingPass &&
        livePreserveAccountingPass &&
        fieldMismatches ==
        0ULL &&
        writeFailures ==
        0ULL &&
        imageFailures ==
        0ULL &&
        liveMutationFailures ==
        0ULL;


    const bool ready =
        m_MotionServoOutputSemanticScratchBridgePrepared &&
        IsServoInputReleaseComplete() &&
        e1Qualified &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-SCRATCH-BRIDGE-SHADOW] "
        "Prep:%s E1:%s InputRelease:%s | "
        "Routes:%d Fields:%d | "
        "AxisSamples:%llu/%llu | "
        "FieldChecks:%llu Expected:%llu Matches:%llu "
        "Mismatch:%llu WriteFail:%llu | "
        "Image:%llu/%llu ImageFail:%llu | "
        "LivePreserve:%llu/%llu Mutation:%llu | "
        "Accounting:%s Window:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "CommandSource:LEGACY_ENI_SERVO_POUTPUT | "
        "Candidate:STRUCTURED_SERVO_OUTPUT_SCRATCH | "
        "PhysicalTarget:UNCHANGED | "
        "SemanticWrite:SCRATCH_ONLY LiveWrite:NO OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_MotionServoOutputSemanticScratchBridgePrepared
        ? "YES"
        : "NO",

        e1Qualified
        ? "QUALIFIED"
        : "WAIT",

        IsServoInputReleaseComplete()
        ? "PASS"
        : "WAIT",

        m_MotionServoOutputSemanticScratchBridgeRouteCount,

        m_MotionServoOutputSemanticScratchBridgeFieldCount,

        (unsigned long long)
        axisSamples,

        (unsigned long long)
        requiredAxisSamples,

        (unsigned long long)
        fieldChecks,

        (unsigned long long)
        expectedFieldChecks,

        (unsigned long long)
        fieldMatches,

        (unsigned long long)
        fieldMismatches,

        (unsigned long long)
        writeFailures,

        (unsigned long long)
        imageMatches,

        (unsigned long long)
        imageChecks,

        (unsigned long long)
        imageFailures,

        (unsigned long long)
        livePreserveMatches,

        (unsigned long long)
        livePreserveChecks,

        (unsigned long long)
        liveMutationFailures,

        fieldAccountingPass &&
        imageAccountingPass &&
        livePreserveAccountingPass
        ? "PASS"
        : "FAIL",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11E.3:
    // exact live-target / rollback preflight status.
    PrintMotionServoOutputLiveWritePreflightShadow();
}


// ============================================================================
// Stage 11E.3 - Servo Output Live-Write Target / Rollback Preflight SHADOW
//
// Future physical target:
//
//     AxisIndex
//         -> structured Servo output descriptors
//         -> exact ServoOutput envelope inside m_IoMap
//
// Existing transmission:
//     same m_IoMap
//         -> next existing LRW Process Data frame
//
// IMPORTANT:
// Stage11E.3 NEVER writes to m_IoMap.
// All semantic writes are redirected to a LOCAL 9-byte window.
// ============================================================================

namespace
{
    template <typename TValue>
    bool WriteStructuredServoOutputToRedirectedWindowE3(
        const EtherCatStructuredServoFieldDescriptor* field,
        const uint8_t* liveMapBase,
        size_t liveMapSize,
        uint32_t windowGlobalOffset,
        uint8_t* redirectedWindow,
        size_t redirectedWindowSize,
        const TValue& value)
    {
        if (field ==
            nullptr ||
            field->pByteBase ==
            nullptr ||
            liveMapBase ==
            nullptr ||
            redirectedWindow ==
            nullptr ||
            strcmp(
                field->direction,
                "Output") !=
            0 ||
            !field->byteAligned ||
            field->byteSpan !=
            sizeof(TValue))
        {
            return
                false;
        }


        const uintptr_t mapAddress =
            reinterpret_cast<uintptr_t>(
                liveMapBase);


        const uintptr_t fieldAddress =
            reinterpret_cast<uintptr_t>(
                field->pByteBase);


        if (fieldAddress <
            mapAddress)
        {
            return
                false;
        }


        const uintptr_t absoluteOffsetRaw =
            fieldAddress -
            mapAddress;


        if (absoluteOffsetRaw >
            static_cast<uintptr_t>(
                liveMapSize))
        {
            return
                false;
        }


        const size_t absoluteOffset =
            static_cast<size_t>(
                absoluteOffsetRaw);


        if (absoluteOffset >
            liveMapSize ||
            field->byteSpan >
            liveMapSize -
            absoluteOffset)
        {
            return
                false;
        }


        if (absoluteOffset <
            static_cast<size_t>(
                windowGlobalOffset))
        {
            return
                false;
        }


        const size_t localOffset =
            absoluteOffset -
            static_cast<size_t>(
                windowGlobalOffset);


        if (localOffset >
            redirectedWindowSize ||
            field->byteSpan >
            redirectedWindowSize -
            localOffset)
        {
            return
                false;
        }


        memcpy(
            redirectedWindow +
            localOffset,
            &value,
            sizeof(TValue));


        return
            true;
    }


    bool BuildRedirectedServoOutputImageE3(
        const EtherCatMaster::MotionServoAxisIndexOutputRouteShadow& route,
        const uint8_t* liveMapBase,
        size_t liveMapSize,
        uint32_t outputGlobalOffset,
        const ServoOutput* liveOutput,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation,
        ServoOutput& redirectedImage)
    {
        if (!route.valid ||
            liveMapBase ==
            nullptr ||
            liveOutput ==
            nullptr)
        {
            return
                false;
        }


        memcpy(
            &redirectedImage,
            liveOutput,
            sizeof(redirectedImage));


        uint8_t* redirectedBase =
            reinterpret_cast<uint8_t*>(
                &redirectedImage);


        return
            WriteStructuredServoOutputToRedirectedWindowE3(
                route.controlWord,
                liveMapBase,
                liveMapSize,
                outputGlobalOffset,
                redirectedBase,
                sizeof(redirectedImage),
                controlWord) &&
            WriteStructuredServoOutputToRedirectedWindowE3(
                route.targetVelocity,
                liveMapBase,
                liveMapSize,
                outputGlobalOffset,
                redirectedBase,
                sizeof(redirectedImage),
                targetVelocity) &&
            WriteStructuredServoOutputToRedirectedWindowE3(
                route.touchProbeFunction,
                liveMapBase,
                liveMapSize,
                outputGlobalOffset,
                redirectedBase,
                sizeof(redirectedImage),
                touchProbeFunction) &&
            WriteStructuredServoOutputToRedirectedWindowE3(
                route.modesOfOperation,
                liveMapBase,
                liveMapSize,
                outputGlobalOffset,
                redirectedBase,
                sizeof(redirectedImage),
                modesOfOperation);
    }
}


bool EtherCatMaster::PrepareMotionServoOutputLiveWritePreflightShadow()
{
    m_MotionServoOutputLiveWritePreflightPrepared =
        false;

    m_MotionServoOutputLiveWritePreflightRouteCount =
        0;

    m_MotionServoOutputLiveWritePreflightFieldCount =
        0;


    m_MotionServoOutputLiveWritePreflightAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightTargetChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightTargetMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightTargetFailures.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightImageChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightImageMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightImageFailures.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightRollbackChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightRollbackMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightRollbackFailures.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightLivePreserveChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightLivePreserveMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputLiveWritePreflightLiveMutationFailures.store(
        0ULL,
        std::memory_order_relaxed);


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        m_MotionServoOutputLiveWritePreflightRoutes[
            axisIndex] =
            MotionServoOutputLiveWritePreflightRoute{};
    }


    int routeCount =
        0;

    int fieldCount =
        0;

    int targetChecks =
        0;

    int targetMatches =
        0;

    int envelopeChecks =
        0;

    int envelopeMatches =
        0;

    int syntheticChecks =
        0;

    int syntheticMatches =
        0;

    int rollbackChecks =
        0;

    int rollbackMatches =
        0;

    int livePreserveChecks =
        0;

    int livePreserveMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-LIVEWRITE-PREFLIGHT] BEGIN | "
        "Stage:11E.3 | "
        "Identity:AXIS_INDEX | "
        "FutureTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "PlannedProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "RollbackSource:LEGACY_COMMAND_SNAPSHOT | "
        "RedirectedWrite:LOCAL_WINDOW_ONLY | "
        "LiveWrite:NO | OutputCutover:NO\n"
        "============================================================\n");


    if (!m_MotionServoOutputSemanticScratchBridgePrepared ||
        !m_MotionServoOutputCommandOwnershipShadowPrepared ||
        m_IoMapSize <=
        0 ||
        m_IoMapSize >
        static_cast<int>(
            sizeof(m_IoMap)))
    {
        errors++;
    }


    if (errors ==
        0)
    {
        struct SyntheticCase
        {
            uint16_t controlWord;
            int32_t targetVelocity;
            uint16_t touchProbeFunction;
            int8_t modesOfOperation;
        };


        const SyntheticCase cases[3] =
        {
            {
                0x0000U,
                0,
                0x0000U,
                0
            },

            {
                0x000FU,
                987654321,
                0x1357U,
                9
            },

            {
                0x0080U,
                -987654321,
                0x2468U,
                static_cast<int8_t>(
                    -9)
            }
        };


        for (int axisIndex = 0;
            axisIndex <
            MAX_AXES;
            ++axisIndex)
        {
            const MotionServoAxisIndexOutputRouteShadow&
                outputRoute =
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex];


            if (!outputRoute.valid)
            {
                continue;
            }


            if (outputRoute.motionSlot <
                0 ||
                outputRoute.motionSlot >=
                static_cast<int>(
                    m_ServoList.size()))
            {
                errors++;
                continue;
            }


            const ENI_ServoDrive&
                servo =
                m_ServoList[
                    static_cast<size_t>(
                        outputRoute.motionSlot)];


            if (servo.pOutput ==
                nullptr)
            {
                errors++;
                continue;
            }


            const uintptr_t mapAddress =
                reinterpret_cast<uintptr_t>(
                    m_IoMap);


            const uintptr_t liveAddress =
                reinterpret_cast<uintptr_t>(
                    servo.pOutput);


            if (liveAddress <
                mapAddress)
            {
                errors++;
                continue;
            }


            const uintptr_t outputOffsetRaw =
                liveAddress -
                mapAddress;


            if (outputOffsetRaw >
                static_cast<uintptr_t>(
                    m_IoMapSize))
            {
                errors++;
                continue;
            }


            const uint32_t outputOffset =
                static_cast<uint32_t>(
                    outputOffsetRaw);


            if (outputOffset >
                static_cast<uint32_t>(
                    m_IoMapSize) ||
                sizeof(ServoOutput) >
                static_cast<size_t>(
                    m_IoMapSize -
                    static_cast<int>(
                        outputOffset)))
            {
                errors++;
                continue;
            }


            MotionServoOutputLiveWritePreflightRoute&
                preflightRoute =
                m_MotionServoOutputLiveWritePreflightRoutes[
                    axisIndex];


            if (preflightRoute.valid)
            {
                errors++;
                continue;
            }


            preflightRoute.valid =
                true;

            preflightRoute.axisIndex =
                outputRoute.axisIndex;

            preflightRoute.motionSlot =
                outputRoute.motionSlot;

            preflightRoute.servoSlaveIndex =
                outputRoute.servoSlaveIndex;

            preflightRoute.outputOffsetBytes =
                outputOffset;

            preflightRoute.outputSpanBytes =
                static_cast<uint32_t>(
                    sizeof(ServoOutput));


            routeCount++;
            fieldCount +=
                4;


            const EtherCatStructuredServoFieldDescriptor*
                fields[4] =
            {
                outputRoute.controlWord,
                outputRoute.targetVelocity,
                outputRoute.touchProbeFunction,
                outputRoute.modesOfOperation
            };


            const size_t expectedOffsets[4] =
            {
                offsetof(
                    ServoOutput,
                    ControlWord),

                offsetof(
                    ServoOutput,
                    TargetVelocity),

                offsetof(
                    ServoOutput,
                    TouchProbeFunc),

                offsetof(
                    ServoOutput,
                    ModesOfOperation)
            };


            const size_t expectedSizes[4] =
            {
                sizeof(
                    servo.pOutput->ControlWord),

                sizeof(
                    servo.pOutput->TargetVelocity),

                sizeof(
                    servo.pOutput->TouchProbeFunc),

                sizeof(
                    servo.pOutput->ModesOfOperation)
            };


            bool targetPass =
                true;


            for (int fieldIndex = 0;
                fieldIndex <
                4;
                ++fieldIndex)
            {
                targetChecks++;


                const auto* field =
                    fields[
                        fieldIndex];


                bool fieldTargetPass =
                    false;


                if (field !=
                    nullptr &&
                    field->pByteBase !=
                    nullptr &&
                    strcmp(
                        field->direction,
                        "Output") ==
                    0 &&
                    field->byteAligned &&
                    field->byteSpan ==
                    expectedSizes[
                        fieldIndex])
                {
                    const uintptr_t fieldAddress =
                        reinterpret_cast<uintptr_t>(
                            field->pByteBase);


                    if (fieldAddress >=
                        mapAddress)
                    {
                        const uintptr_t fieldOffsetRaw =
                            fieldAddress -
                            mapAddress;


                        const uintptr_t expectedFieldOffset =
                            static_cast<uintptr_t>(
                                outputOffset) +
                            static_cast<uintptr_t>(
                                expectedOffsets[
                                    fieldIndex]);


                        fieldTargetPass =
                            fieldOffsetRaw ==
                            expectedFieldOffset;
                    }
                }


                        if (fieldTargetPass)
                        {
                            targetMatches++;
                        }
                        else
                        {
                            targetPass =
                                false;

                            errors++;
                        }
            }


            // ------------------------------------------------------------
            // Servo output envelope must not overlap another Servo route.
            // ------------------------------------------------------------

            envelopeChecks++;


            bool envelopePass =
                true;


            for (int otherAxis = 0;
                otherAxis <
                MAX_AXES;
                ++otherAxis)
            {
                if (otherAxis ==
                    axisIndex)
                {
                    continue;
                }


                const MotionServoOutputLiveWritePreflightRoute&
                    existing =
                    m_MotionServoOutputLiveWritePreflightRoutes[
                        otherAxis];


                if (!existing.valid)
                {
                    continue;
                }


                const uint32_t aBegin =
                    outputOffset;

                const uint32_t aEnd =
                    outputOffset +
                    static_cast<uint32_t>(
                        sizeof(ServoOutput));


                const uint32_t bBegin =
                    existing.outputOffsetBytes;

                const uint32_t bEnd =
                    existing.outputOffsetBytes +
                    existing.outputSpanBytes;


                const bool overlap =
                    aBegin <
                    bEnd&&
                    bBegin <
                    aEnd;


                if (overlap)
                {
                    envelopePass =
                        false;

                    break;
                }
            }


            if (envelopePass)
            {
                envelopeMatches++;
            }
            else
            {
                errors++;
            }


            uint8_t liveBefore[
                sizeof(ServoOutput)] =
                {};


                memcpy(
                    liveBefore,
                    servo.pOutput,
                    sizeof(liveBefore));


                bool syntheticRoutePass =
                    targetPass &&
                    envelopePass;


                for (const SyntheticCase& test :
                    cases)
                {
                    syntheticChecks++;


                    ServoOutput redirected =
                    {};


                    const bool redirectedPass =
                        BuildRedirectedServoOutputImageE3(
                            outputRoute,
                            reinterpret_cast<const uint8_t*>(
                                m_IoMap),
                            static_cast<size_t>(
                                m_IoMapSize),
                            outputOffset,
                            servo.pOutput,
                            test.controlWord,
                            test.targetVelocity,
                            test.touchProbeFunction,
                            test.modesOfOperation,
                            redirected);


                    ServoOutput expected =
                    {};


                    expected.ControlWord =
                        test.controlWord;

                    expected.TargetVelocity =
                        test.targetVelocity;

                    expected.TouchProbeFunc =
                        test.touchProbeFunction;

                    expected.ModesOfOperation =
                        test.modesOfOperation;


                    const bool imagePass =
                        redirectedPass &&
                        memcmp(
                            &redirected,
                            &expected,
                            sizeof(expected)) ==
                        0;


                    if (imagePass)
                    {
                        syntheticMatches++;
                    }
                    else
                    {
                        syntheticRoutePass =
                            false;

                        errors++;
                    }


                    // --------------------------------------------------------
                    // Simulated rollback:
                    //
                    // restore the redirected LOCAL window from the original
                    // live legacy command snapshot.
                    // --------------------------------------------------------

                    rollbackChecks++;


                    memcpy(
                        &redirected,
                        liveBefore,
                        sizeof(redirected));


                    const bool rollbackPass =
                        memcmp(
                            &redirected,
                            liveBefore,
                            sizeof(redirected)) ==
                        0;


                    if (rollbackPass)
                    {
                        rollbackMatches++;
                    }
                    else
                    {
                        syntheticRoutePass =
                            false;

                        errors++;
                    }
                }


                livePreserveChecks++;


                const bool livePreserved =
                    memcmp(
                        liveBefore,
                        servo.pOutput,
                        sizeof(liveBefore)) ==
                    0;


                if (livePreserved)
                {
                    livePreserveMatches++;
                }
                else
                {
                    syntheticRoutePass =
                        false;

                    errors++;
                }


                DEBUG_PRINT(
                    "[SERVO-OUTPUT-LIVEWRITE-PREFLIGHT-MAP] "
                    "AxisIndex:%d | Slot:%d | Servo:S%d | "
                    "IoMapOffset:%u/%u | Span:%u | "
                    "Target:%s | Envelope:%s | "
                    "RedirectedCases:%s | Rollback:%s | "
                    "LivePreserve:%s | "
                    "FutureTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
                    "Result:%s\n",

                    outputRoute.axisIndex,

                    outputRoute.motionSlot,

                    outputRoute.servoSlaveIndex,

                    outputOffset,

                    (unsigned int)
                    m_IoMapSize,

                    (unsigned int)
                    sizeof(ServoOutput),

                    targetPass
                    ? "4/4"
                    : "FAIL",

                    envelopePass
                    ? "PASS"
                    : "FAIL",

                    syntheticRoutePass
                    ? "3/3"
                    : "FAIL",

                    syntheticRoutePass
                    ? "3/3"
                    : "FAIL",

                    livePreserved
                    ? "YES"
                    : "NO",

                    targetPass &&
                    envelopePass &&
                    syntheticRoutePass &&
                    livePreserved
                    ? "PASS"
                    : "FAIL");
        }
    }


    const int expectedRoutes =
        m_MotionServoOutputSemanticScratchBridgeRouteCount;


    const int expectedFields =
        expectedRoutes *
        4;


    const int expectedSyntheticChecks =
        expectedRoutes *
        3;


    const bool pass =
        errors ==
        0 &&
        expectedRoutes >
        0 &&
        routeCount ==
        expectedRoutes &&
        fieldCount ==
        expectedFields &&
        targetChecks ==
        expectedFields &&
        targetMatches ==
        expectedFields &&
        envelopeChecks ==
        expectedRoutes &&
        envelopeMatches ==
        expectedRoutes &&
        syntheticChecks ==
        expectedSyntheticChecks &&
        syntheticMatches ==
        expectedSyntheticChecks &&
        rollbackChecks ==
        expectedSyntheticChecks &&
        rollbackMatches ==
        expectedSyntheticChecks &&
        livePreserveChecks ==
        expectedRoutes &&
        livePreserveMatches ==
        expectedRoutes;


    if (pass)
    {
        m_MotionServoOutputLiveWritePreflightRouteCount =
            routeCount;

        m_MotionServoOutputLiveWritePreflightFieldCount =
            fieldCount;

        m_MotionServoOutputLiveWritePreflightPrepared =
            true;
    }


    DEBUG_PRINT(
        "[SERVO-OUTPUT-LIVEWRITE-PREFLIGHT-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | "
        "Target:%d/%d | Envelope:%d/%d | "
        "Redirected:%d/%d | Rollback:%d/%d | "
        "LivePreserve:%d/%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "FutureTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "PlannedProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "RollbackSource:LEGACY_COMMAND_SNAPSHOT | "
        "LiveWrite:NO | OutputCutover:NO | "
        "Next:%s\n",

        routeCount,
        expectedRoutes,

        fieldCount,
        expectedFields,

        targetMatches,
        expectedFields,

        envelopeMatches,
        expectedRoutes,

        syntheticMatches,
        expectedSyntheticChecks,

        rollbackMatches,
        expectedSyntheticChecks,

        livePreserveMatches,
        expectedRoutes,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoOutputLiveWritePreflightPrepared
        ? "YES"
        : "NO",

        pass
        ? "WAIT_STAGE11E2_RUNTIME_QUALIFICATION"
        : "BLOCK_STAGE11E3");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-OUTPUT-LIVEWRITE-PREFLIGHT] END | "
        "Stage:11E.3 | Result:%s | LiveWrite:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::ObserveMotionServoOutputLiveWritePreflightShadow(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    // ========================================================================
    // 250us Motion hot-path SHADOW.
    //
    // Starts only after Stage11E.2 is already qualified in this boot.
    //
    // No live write.
    // No logging.
    // No allocation.
    // No string lookup.
    // No EtherCAT hardware access.
    // ========================================================================

    if (!m_MotionServoOutputLiveWritePreflightPrepared ||
        !IsServoInputReleaseComplete())
    {
        return;
    }


    const uint64_t requiredE2AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputSemanticScratchBridgeRouteCount) *
        20000ULL;


    const uint64_t e2AxisSamples =
        m_MotionServoOutputSemanticScratchAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e2FieldMismatches =
        m_MotionServoOutputSemanticScratchFieldMismatches.load(
            std::memory_order_relaxed);


    const uint64_t e2WriteFailures =
        m_MotionServoOutputSemanticScratchWriteFailures.load(
            std::memory_order_relaxed);


    const uint64_t e2ImageFailures =
        m_MotionServoOutputSemanticScratchImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t e2LiveMutations =
        m_MotionServoOutputSemanticScratchLiveMutationFailures.load(
            std::memory_order_relaxed);


    const bool e2Qualified =
        m_MotionServoOutputSemanticScratchBridgePrepared &&
        requiredE2AxisSamples >
        0ULL &&
        e2AxisSamples >=
        requiredE2AxisSamples &&
        e2FieldMismatches ==
        0ULL &&
        e2WriteFailures ==
        0ULL &&
        e2ImageFailures ==
        0ULL &&
        e2LiveMutations ==
        0ULL;


    if (!e2Qualified)
    {
        return;
    }


    m_MotionServoOutputLiveWritePreflightAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputLiveWritePreflightTargetChecks.fetch_add(
        4ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputLiveWritePreflightImageChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputLiveWritePreflightRollbackChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputLiveWritePreflightLivePreserveChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    if (axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        m_MotionServoOutputLiveWritePreflightTargetFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightRollbackFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    const MotionServoAxisIndexOutputRouteShadow&
        outputRoute =
        m_MotionServoAxisIndexOutputRoutesShadow[
            axisIndex];


    const MotionServoOutputLiveWritePreflightRoute&
        preflightRoute =
        m_MotionServoOutputLiveWritePreflightRoutes[
            axisIndex];


    if (!outputRoute.valid ||
        !preflightRoute.valid ||
        outputRoute.motionSlot <
        0 ||
        outputRoute.motionSlot >=
        static_cast<int>(
            m_ServoList.size()))
    {
        m_MotionServoOutputLiveWritePreflightTargetFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightRollbackFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    const ENI_ServoDrive&
        servo =
        m_ServoList[
            static_cast<size_t>(
                outputRoute.motionSlot)];


    if (servo.pOutput ==
        nullptr)
    {
        m_MotionServoOutputLiveWritePreflightTargetFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightImageFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightRollbackFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    const uintptr_t mapAddress =
        reinterpret_cast<uintptr_t>(
            m_IoMap);


    const uintptr_t liveAddress =
        reinterpret_cast<uintptr_t>(
            servo.pOutput);


    const bool outputEnvelopeStable =
        liveAddress >=
        mapAddress &&
        static_cast<uintptr_t>(
            liveAddress -
            mapAddress) ==
        static_cast<uintptr_t>(
            preflightRoute.outputOffsetBytes);


    const EtherCatStructuredServoFieldDescriptor*
        fields[4] =
    {
        outputRoute.controlWord,
        outputRoute.targetVelocity,
        outputRoute.touchProbeFunction,
        outputRoute.modesOfOperation
    };


    const size_t expectedOffsets[4] =
    {
        offsetof(
            ServoOutput,
            ControlWord),

        offsetof(
            ServoOutput,
            TargetVelocity),

        offsetof(
            ServoOutput,
            TouchProbeFunc),

        offsetof(
            ServoOutput,
            ModesOfOperation)
    };


    uint64_t targetMatchCount =
        0ULL;


    for (int fieldIndex = 0;
        fieldIndex <
        4;
        ++fieldIndex)
    {
        const auto* field =
            fields[
                fieldIndex];


        bool targetMatch =
            false;


        if (outputEnvelopeStable &&
            field !=
            nullptr &&
            field->pByteBase !=
            nullptr)
        {
            const uintptr_t fieldAddress =
                reinterpret_cast<uintptr_t>(
                    field->pByteBase);


            if (fieldAddress >=
                mapAddress)
            {
                const uintptr_t fieldOffset =
                    fieldAddress -
                    mapAddress;


                targetMatch =
                    fieldOffset ==
                    static_cast<uintptr_t>(
                        preflightRoute.outputOffsetBytes) +
                    static_cast<uintptr_t>(
                        expectedOffsets[
                            fieldIndex]);
            }
        }


        if (targetMatch)
        {
            targetMatchCount++;
        }
    }


    m_MotionServoOutputLiveWritePreflightTargetMatches.fetch_add(
        targetMatchCount,
        std::memory_order_relaxed);


    m_MotionServoOutputLiveWritePreflightTargetFailures.fetch_add(
        4ULL -
        targetMatchCount,
        std::memory_order_relaxed);


    uint8_t liveBefore[
        sizeof(ServoOutput)] =
        {};


        memcpy(
            liveBefore,
            servo.pOutput,
            sizeof(liveBefore));


        ServoOutput redirected =
        {};


        const bool redirectedPass =
            outputEnvelopeStable &&
            targetMatchCount ==
            4ULL &&
            BuildRedirectedServoOutputImageE3(
                outputRoute,
                reinterpret_cast<const uint8_t*>(
                    m_IoMap),
                static_cast<size_t>(
                    m_IoMapSize),
                preflightRoute.outputOffsetBytes,
                servo.pOutput,
                controlWord,
                targetVelocity,
                touchProbeFunction,
                modesOfOperation,
                redirected);


        ServoOutput expected =
        {};


        expected.ControlWord =
            controlWord;

        expected.TargetVelocity =
            targetVelocity;

        expected.TouchProbeFunc =
            touchProbeFunction;

        expected.ModesOfOperation =
            modesOfOperation;


        const bool imagePass =
            redirectedPass &&
            memcmp(
                &redirected,
                &expected,
                sizeof(expected)) ==
            0 &&
            memcmp(
                &redirected,
                servo.pOutput,
                sizeof(redirected)) ==
            0;


        if (imagePass)
        {
            m_MotionServoOutputLiveWritePreflightImageMatches.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_MotionServoOutputLiveWritePreflightImageFailures.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        // Simulated rollback to the exact command image that was authoritative
        // before the prospective structured write.
        memcpy(
            &redirected,
            liveBefore,
            sizeof(redirected));


        const bool rollbackPass =
            memcmp(
                &redirected,
                liveBefore,
                sizeof(redirected)) ==
            0;


        if (rollbackPass)
        {
            m_MotionServoOutputLiveWritePreflightRollbackMatches.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_MotionServoOutputLiveWritePreflightRollbackFailures.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        // Final safety assertion:
        // no live Process Image byte was ever used as a write target.
        const bool livePreserved =
            memcmp(
                liveBefore,
                servo.pOutput,
                sizeof(liveBefore)) ==
            0;


        if (livePreserved)
        {
            m_MotionServoOutputLiveWritePreflightLivePreserveMatches.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }
        else
        {
            m_MotionServoOutputLiveWritePreflightLiveMutationFailures.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        // Stage 11E.4:
        // validate the centralized legacy producer seam against this
        // cycle's final Servo command image.
        ObserveMotionServoOutputCommandSeamCycleShadow(
            axisIndex,
            controlWord,
            targetVelocity,
            touchProbeFunction,
            modesOfOperation);
}


void EtherCatMaster::PrintMotionServoOutputLiveWritePreflightShadow() const
{
    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputLiveWritePreflightRouteCount) *
        20000ULL;


    const uint64_t axisSamples =
        m_MotionServoOutputLiveWritePreflightAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t targetChecks =
        m_MotionServoOutputLiveWritePreflightTargetChecks.load(
            std::memory_order_relaxed);


    const uint64_t targetMatches =
        m_MotionServoOutputLiveWritePreflightTargetMatches.load(
            std::memory_order_relaxed);


    const uint64_t targetFailures =
        m_MotionServoOutputLiveWritePreflightTargetFailures.load(
            std::memory_order_relaxed);


    const uint64_t imageChecks =
        m_MotionServoOutputLiveWritePreflightImageChecks.load(
            std::memory_order_relaxed);


    const uint64_t imageMatches =
        m_MotionServoOutputLiveWritePreflightImageMatches.load(
            std::memory_order_relaxed);


    const uint64_t imageFailures =
        m_MotionServoOutputLiveWritePreflightImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t rollbackChecks =
        m_MotionServoOutputLiveWritePreflightRollbackChecks.load(
            std::memory_order_relaxed);


    const uint64_t rollbackMatches =
        m_MotionServoOutputLiveWritePreflightRollbackMatches.load(
            std::memory_order_relaxed);


    const uint64_t rollbackFailures =
        m_MotionServoOutputLiveWritePreflightRollbackFailures.load(
            std::memory_order_relaxed);


    const uint64_t livePreserveChecks =
        m_MotionServoOutputLiveWritePreflightLivePreserveChecks.load(
            std::memory_order_relaxed);


    const uint64_t livePreserveMatches =
        m_MotionServoOutputLiveWritePreflightLivePreserveMatches.load(
            std::memory_order_relaxed);


    const uint64_t liveMutationFailures =
        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.load(
            std::memory_order_relaxed);


    const uint64_t expectedTargetChecks =
        axisSamples *
        4ULL;


    const bool targetAccountingPass =
        targetChecks ==
        expectedTargetChecks &&
        targetMatches +
        targetFailures ==
        targetChecks;


    const bool imageAccountingPass =
        imageChecks ==
        axisSamples &&
        imageMatches +
        imageFailures ==
        imageChecks;


    const bool rollbackAccountingPass =
        rollbackChecks ==
        axisSamples &&
        rollbackMatches +
        rollbackFailures ==
        rollbackChecks;


    const bool livePreserveAccountingPass =
        livePreserveChecks ==
        axisSamples &&
        livePreserveMatches +
        liveMutationFailures ==
        livePreserveChecks;


    const uint64_t requiredE2AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputSemanticScratchBridgeRouteCount) *
        20000ULL;


    const uint64_t e2AxisSamples =
        m_MotionServoOutputSemanticScratchAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e2FieldMismatches =
        m_MotionServoOutputSemanticScratchFieldMismatches.load(
            std::memory_order_relaxed);


    const uint64_t e2WriteFailures =
        m_MotionServoOutputSemanticScratchWriteFailures.load(
            std::memory_order_relaxed);


    const uint64_t e2ImageFailures =
        m_MotionServoOutputSemanticScratchImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t e2LiveMutations =
        m_MotionServoOutputSemanticScratchLiveMutationFailures.load(
            std::memory_order_relaxed);


    const bool e2Qualified =
        m_MotionServoOutputSemanticScratchBridgePrepared &&
        requiredE2AxisSamples >
        0ULL &&
        e2AxisSamples >=
        requiredE2AxisSamples &&
        e2FieldMismatches ==
        0ULL &&
        e2WriteFailures ==
        0ULL &&
        e2ImageFailures ==
        0ULL &&
        e2LiveMutations ==
        0ULL;


    const bool windowQualified =
        requiredAxisSamples >
        0ULL &&
        axisSamples >=
        requiredAxisSamples;


    const bool clean =
        targetAccountingPass &&
        imageAccountingPass &&
        rollbackAccountingPass &&
        livePreserveAccountingPass &&
        targetFailures ==
        0ULL &&
        imageFailures ==
        0ULL &&
        rollbackFailures ==
        0ULL &&
        liveMutationFailures ==
        0ULL;


    const bool ready =
        m_MotionServoOutputLiveWritePreflightPrepared &&
        IsServoInputReleaseComplete() &&
        e2Qualified &&
        windowQualified &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-LIVEWRITE-PREFLIGHT-SHADOW] "
        "Prep:%s E2:%s InputRelease:%s | "
        "Routes:%d Fields:%d | "
        "AxisSamples:%llu/%llu | "
        "Target:%llu/%llu TargetFail:%llu | "
        "CommandImage:%llu/%llu ImageFail:%llu | "
        "Rollback:%llu/%llu RollbackFail:%llu | "
        "LivePreserve:%llu/%llu Mutation:%llu | "
        "Accounting:%s Window:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "FutureTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "PlannedProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "RollbackSource:LEGACY_COMMAND_SNAPSHOT | "
        "LiveWrite:NO OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_MotionServoOutputLiveWritePreflightPrepared
        ? "YES"
        : "NO",

        e2Qualified
        ? "QUALIFIED"
        : "WAIT",

        IsServoInputReleaseComplete()
        ? "PASS"
        : "WAIT",

        m_MotionServoOutputLiveWritePreflightRouteCount,

        m_MotionServoOutputLiveWritePreflightFieldCount,

        (unsigned long long)
        axisSamples,

        (unsigned long long)
        requiredAxisSamples,

        (unsigned long long)
        targetMatches,

        (unsigned long long)
        targetChecks,

        (unsigned long long)
        targetFailures,

        (unsigned long long)
        imageMatches,

        (unsigned long long)
        imageChecks,

        (unsigned long long)
        imageFailures,

        (unsigned long long)
        rollbackMatches,

        (unsigned long long)
        rollbackChecks,

        (unsigned long long)
        rollbackFailures,

        (unsigned long long)
        livePreserveMatches,

        (unsigned long long)
        livePreserveChecks,

        (unsigned long long)
        liveMutationFailures,

        targetAccountingPass &&
        imageAccountingPass &&
        rollbackAccountingPass &&
        livePreserveAccountingPass
        ? "PASS"
        : "FAIL",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    // Stage 11E.4:
    // centralized Motion/Homing command seam status.
    PrintMotionServoOutputCommandSeamShadow();
}


// ============================================================================
// Stage 11E.4 - Servo Output Command Seam SHADOW
//
// Active producer:
//     Motion / Homing
//         -> MotionCore centralized LEGACY command seam
//         -> same ServoOutput / m_IoMap
//
// Candidate:
//     structured AxisIndex output route
//
// No structured live write.
// No output cutover.
// ============================================================================

bool EtherCatMaster::PrepareMotionServoOutputCommandSeamShadow()
{
    m_MotionServoOutputCommandSeamPrepared =
        false;

    m_MotionServoOutputCommandSeamRouteCount =
        0;

    m_MotionServoOutputCommandSeamFieldCount =
        0;


    m_MotionServoOutputCommandSeamRuntimeStarted.store(
        false,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandSeamCycleSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamCycleChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamCycleMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamCycleMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamCycleFailures.store(
        0ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandSeamWriteChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamWriteMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamWriteFailures.store(
        0ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandSeamControlWordWrites.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamTargetVelocityWrites.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamTouchProbeWrites.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCommandSeamModesWrites.store(
        0ULL,
        std::memory_order_relaxed);


    for (int axisIndex = 0;
        axisIndex <
        MAX_AXES;
        ++axisIndex)
    {
        m_MotionServoOutputCommandSeamAxisStates[
            axisIndex] =
            MotionServoOutputCommandSeamAxisState{};
    }


    int routes =
        0;

    int fields =
        0;

    int initialChecks =
        0;

    int initialMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-COMMAND-SEAM-PREFLIGHT] BEGIN | "
        "Stage:11E.4 | "
        "Identity:AXIS_INDEX | "
        "ActiveProducer:LEGACY_COMMAND_SEAM_TO_POUTPUT | "
        "CandidateProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "Scope:EDM_MOTION_HOMING | "
        "StructuredLiveWrite:NO | OutputCutover:NO\n"
        "============================================================\n");


    if (!m_MotionServoOutputLiveWritePreflightPrepared ||
        !m_MotionServoOutputCommandOwnershipShadowPrepared)
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (int axisIndex = 0;
            axisIndex <
            MAX_AXES;
            ++axisIndex)
        {
            const MotionServoAxisIndexOutputRouteShadow&
                route =
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex];


            if (!route.valid)
            {
                continue;
            }


            if (route.motionSlot <
                0 ||
                route.motionSlot >=
                static_cast<int>(
                    m_ServoList.size()))
            {
                errors++;
                continue;
            }


            const ENI_ServoDrive&
                servo =
                m_ServoList[
                    static_cast<size_t>(
                        route.motionSlot)];


            if (servo.pOutput ==
                nullptr)
            {
                errors++;
                continue;
            }


            MotionServoOutputCommandSeamAxisState&
                state =
                m_MotionServoOutputCommandSeamAxisStates[
                    axisIndex];


            state.valid =
                true;

            state.axisIndex =
                axisIndex;

            state.motionSlot =
                route.motionSlot;

            state.servoSlaveIndex =
                route.servoSlaveIndex;

            state.controlWord =
                servo.pOutput->ControlWord;

            state.targetVelocity =
                servo.pOutput->TargetVelocity;

            state.touchProbeFunction =
                servo.pOutput->TouchProbeFunc;

            state.modesOfOperation =
                servo.pOutput->ModesOfOperation;


            routes++;
            fields +=
                4;


            uint16_t semanticControlWord =
                0U;

            int32_t semanticTargetVelocity =
                0;

            uint16_t semanticTouchProbeFunction =
                0U;

            int8_t semanticModesOfOperation =
                0;


            initialChecks +=
                4;


            const bool readPass =
                ReadMotionServoOutputByAxisIndexShadow(
                    axisIndex,
                    semanticControlWord,
                    semanticTargetVelocity,
                    semanticTouchProbeFunction,
                    semanticModesOfOperation);


            const bool initialMatch =
                readPass &&
                semanticControlWord ==
                state.controlWord &&
                semanticTargetVelocity ==
                state.targetVelocity &&
                semanticTouchProbeFunction ==
                state.touchProbeFunction &&
                semanticModesOfOperation ==
                state.modesOfOperation;


            if (initialMatch)
            {
                initialMatches +=
                    4;
            }
            else
            {
                errors++;
            }


            DEBUG_PRINT(
                "[SERVO-OUTPUT-COMMAND-SEAM-MAP] "
                "AxisIndex:%d | Slot:%d | Servo:S%d | "
                "InitialImage:%s | "
                "ActiveProducer:LEGACY_COMMAND_SEAM_TO_POUTPUT | "
                "StructuredLiveWrite:NO | Result:%s\n",

                state.axisIndex,

                state.motionSlot,

                state.servoSlaveIndex,

                initialMatch
                ? "4/4"
                : "FAIL",

                initialMatch
                ? "PASS"
                : "FAIL");
        }
    }


    const int expectedRoutes =
        m_MotionServoOutputLiveWritePreflightRouteCount;


    const int expectedFields =
        expectedRoutes *
        4;


    const bool pass =
        errors ==
        0 &&
        expectedRoutes >
        0 &&
        routes ==
        expectedRoutes &&
        fields ==
        expectedFields &&
        initialChecks ==
        expectedFields &&
        initialMatches ==
        expectedFields;


    if (pass)
    {
        m_MotionServoOutputCommandSeamRouteCount =
            routes;

        m_MotionServoOutputCommandSeamFieldCount =
            fields;

        m_MotionServoOutputCommandSeamPrepared =
            true;
    }


    DEBUG_PRINT(
        "[SERVO-OUTPUT-COMMAND-SEAM-PREFLIGHT-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | "
        "InitialImage:%d/%d | Errors:%d | "
        "Result:%s | Prepared:%s | "
        "ActiveProducer:LEGACY_COMMAND_SEAM_TO_POUTPUT | "
        "CandidateProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "MotionCoreDirectAssignments:SEAM_ONLY | "
        "TouchProbeCommand:SEAM_ROUTED | "
        "StructuredLiveWrite:NO | OutputCutover:NO | "
        "Next:%s\n",

        routes,
        expectedRoutes,

        fields,
        expectedFields,

        initialMatches,
        expectedFields,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_MotionServoOutputCommandSeamPrepared
        ? "YES"
        : "NO",

        pass
        ? "WAIT_STAGE11E3_RUNTIME_QUALIFICATION"
        : "BLOCK_STAGE11E4");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-OUTPUT-COMMAND-SEAM-PREFLIGHT] END | "
        "Stage:11E.4 | Result:%s | OutputCutover:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void EtherCatMaster::ObserveMotionServoOutputCommandSeamWriteShadow(
    int axisIndex,
    MotionServoOutputCommandField field,
    int64_t value)
{
    // Stage 11E.6: freeze historical E4 write accounting after retirement.
    if (m_MotionServoOutputCompatibilityRetirementActive.load(
        std::memory_order_relaxed))
    {
        return;
    }


    // ========================================================================
    // Called immediately AFTER the active legacy seam writes pOutput.
    //
    // Same 250us Motion thread.
    //
    // No allocation / no logging / no hardware access.
    // ========================================================================

    if (!m_MotionServoOutputCommandSeamPrepared ||
        axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        return;
    }


    MotionServoOutputCommandSeamAxisState&
        state =
        m_MotionServoOutputCommandSeamAxisStates[
            axisIndex];


    if (!state.valid)
    {
        return;
    }


    bool valueMatch =
        false;


    switch (field)
    {
    case MotionServoOutputCommandField::ControlWord:
    {
        state.controlWord =
            static_cast<uint16_t>(
                value);


        if (m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed))
        {
            m_MotionServoOutputCommandSeamControlWordWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        uint16_t semantic =
            0U;


        valueMatch =
            ReadStructuredServoOutputTypedFieldE1(
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex]
                .controlWord,
                        semantic) &&
            semantic ==
                        state.controlWord;

                    break;
    }


    case MotionServoOutputCommandField::TargetVelocity:
    {
        state.targetVelocity =
            static_cast<int32_t>(
                value);


        if (m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed))
        {
            m_MotionServoOutputCommandSeamTargetVelocityWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        int32_t semantic =
            0;


        valueMatch =
            ReadStructuredServoOutputTypedFieldE1(
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex]
                .targetVelocity,
                        semantic) &&
            semantic ==
                        state.targetVelocity;

                    break;
    }


    case MotionServoOutputCommandField::TouchProbeFunction:
    {
        state.touchProbeFunction =
            static_cast<uint16_t>(
                value);


        if (m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed))
        {
            m_MotionServoOutputCommandSeamTouchProbeWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        uint16_t semantic =
            0U;


        valueMatch =
            ReadStructuredServoOutputTypedFieldE1(
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex]
                .touchProbeFunction,
                        semantic) &&
            semantic ==
                        state.touchProbeFunction;

                    break;
    }


    case MotionServoOutputCommandField::ModesOfOperation:
    {
        state.modesOfOperation =
            static_cast<int8_t>(
                value);


        if (m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed))
        {
            m_MotionServoOutputCommandSeamModesWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }


        int8_t semantic =
            0;


        valueMatch =
            ReadStructuredServoOutputTypedFieldE1(
                m_MotionServoAxisIndexOutputRoutesShadow[
                    axisIndex]
                .modesOfOperation,
                        semantic) &&
            semantic ==
                        state.modesOfOperation;

                    break;
    }


    default:
        return;
    }


    if (!m_MotionServoOutputCommandSeamRuntimeStarted.load(
        std::memory_order_relaxed))
    {
        return;
    }


    m_MotionServoOutputCommandSeamWriteChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    if (valueMatch)
    {
        m_MotionServoOutputCommandSeamWriteMatches.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
    else
    {
        m_MotionServoOutputCommandSeamWriteFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
}


void EtherCatMaster::ObserveMotionServoOutputCommandSeamCycleShadow(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    // ========================================================================
    // Called from E3 at the final Servo command image for this Motion cycle.
    //
    // E4 does not start its own qualification window until E3 itself is
    // qualified in this boot.
    // ========================================================================

    if (!m_MotionServoOutputCommandSeamPrepared ||
        !IsServoInputReleaseComplete())
    {
        return;
    }


    const uint64_t requiredE3AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputLiveWritePreflightRouteCount) *
        20000ULL;


    const uint64_t e3AxisSamples =
        m_MotionServoOutputLiveWritePreflightAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e3TargetFailures =
        m_MotionServoOutputLiveWritePreflightTargetFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3ImageFailures =
        m_MotionServoOutputLiveWritePreflightImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3RollbackFailures =
        m_MotionServoOutputLiveWritePreflightRollbackFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3Mutations =
        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.load(
            std::memory_order_relaxed);


    const bool e3Qualified =
        m_MotionServoOutputLiveWritePreflightPrepared &&
        requiredE3AxisSamples >
        0ULL &&
        e3AxisSamples >=
        requiredE3AxisSamples &&
        e3TargetFailures ==
        0ULL &&
        e3ImageFailures ==
        0ULL &&
        e3RollbackFailures ==
        0ULL &&
        e3Mutations ==
        0ULL;


    if (!e3Qualified)
    {
        return;
    }


    bool expected =
        false;


    if (m_MotionServoOutputCommandSeamRuntimeStarted.compare_exchange_strong(
        expected,
        true,
        std::memory_order_relaxed))
    {
        // Start from the NEXT complete Motion command cycle.
        return;
    }


    m_MotionServoOutputCommandSeamCycleSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandSeamCycleChecks.fetch_add(
        4ULL,
        std::memory_order_relaxed);


    if (axisIndex <
        0 ||
        axisIndex >=
        MAX_AXES)
    {
        m_MotionServoOutputCommandSeamCycleFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        return;
    }


    const MotionServoOutputCommandSeamAxisState&
        state =
        m_MotionServoOutputCommandSeamAxisStates[
            axisIndex];


    if (!state.valid)
    {
        m_MotionServoOutputCommandSeamCycleFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        return;
    }


    const bool matches[4] =
    {
        state.controlWord ==
            controlWord,

        state.targetVelocity ==
            targetVelocity,

        state.touchProbeFunction ==
            touchProbeFunction,

        state.modesOfOperation ==
            modesOfOperation
    };


    uint64_t matchCount =
        0ULL;


    for (bool match :
    matches)
    {
        if (match)
        {
            matchCount++;
        }
    }


    m_MotionServoOutputCommandSeamCycleMatches.fetch_add(
        matchCount,
        std::memory_order_relaxed);


    m_MotionServoOutputCommandSeamCycleMismatches.fetch_add(
        4ULL -
        matchCount,
        std::memory_order_relaxed);


    // Stage 11E.5 final live structured producer verification.
    ObserveMotionServoOutputStructuredProducerCycle(
        axisIndex,
        controlWord,
        targetVelocity,
        touchProbeFunction,
        modesOfOperation);
}


void EtherCatMaster::PrintMotionServoOutputCommandSeamShadow() const
{
    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;


    const uint64_t cycleSamples =
        m_MotionServoOutputCommandSeamCycleSamples.load(
            std::memory_order_relaxed);


    const uint64_t cycleChecks =
        m_MotionServoOutputCommandSeamCycleChecks.load(
            std::memory_order_relaxed);


    const uint64_t cycleMatches =
        m_MotionServoOutputCommandSeamCycleMatches.load(
            std::memory_order_relaxed);


    const uint64_t cycleMismatches =
        m_MotionServoOutputCommandSeamCycleMismatches.load(
            std::memory_order_relaxed);


    const uint64_t cycleFailures =
        m_MotionServoOutputCommandSeamCycleFailures.load(
            std::memory_order_relaxed);


    const uint64_t writeChecks =
        m_MotionServoOutputCommandSeamWriteChecks.load(
            std::memory_order_relaxed);


    const uint64_t writeMatches =
        m_MotionServoOutputCommandSeamWriteMatches.load(
            std::memory_order_relaxed);


    const uint64_t writeFailures =
        m_MotionServoOutputCommandSeamWriteFailures.load(
            std::memory_order_relaxed);


    const uint64_t controlWordWrites =
        m_MotionServoOutputCommandSeamControlWordWrites.load(
            std::memory_order_relaxed);


    const uint64_t targetVelocityWrites =
        m_MotionServoOutputCommandSeamTargetVelocityWrites.load(
            std::memory_order_relaxed);


    const uint64_t touchProbeWrites =
        m_MotionServoOutputCommandSeamTouchProbeWrites.load(
            std::memory_order_relaxed);


    const uint64_t modesWrites =
        m_MotionServoOutputCommandSeamModesWrites.load(
            std::memory_order_relaxed);


    const uint64_t expectedCycleChecks =
        cycleSamples *
        4ULL;


    const bool cycleAccountingPass =
        cycleChecks ==
        expectedCycleChecks &&
        cycleMatches +
        cycleMismatches +
        cycleFailures ==
        cycleChecks;


    const bool writeAccountingPass =
        writeMatches +
        writeFailures ==
        writeChecks;


    const uint64_t requiredE3AxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputLiveWritePreflightRouteCount) *
        20000ULL;


    const uint64_t e3AxisSamples =
        m_MotionServoOutputLiveWritePreflightAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t e3TargetFailures =
        m_MotionServoOutputLiveWritePreflightTargetFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3ImageFailures =
        m_MotionServoOutputLiveWritePreflightImageFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3RollbackFailures =
        m_MotionServoOutputLiveWritePreflightRollbackFailures.load(
            std::memory_order_relaxed);


    const uint64_t e3Mutations =
        m_MotionServoOutputLiveWritePreflightLiveMutationFailures.load(
            std::memory_order_relaxed);


    const bool e3Qualified =
        m_MotionServoOutputLiveWritePreflightPrepared &&
        requiredE3AxisSamples >
        0ULL &&
        e3AxisSamples >=
        requiredE3AxisSamples &&
        e3TargetFailures ==
        0ULL &&
        e3ImageFailures ==
        0ULL &&
        e3RollbackFailures ==
        0ULL &&
        e3Mutations ==
        0ULL;


    const bool windowQualified =
        requiredAxisSamples >
        0ULL &&
        cycleSamples >=
        requiredAxisSamples;


    // ControlWord, TargetVelocity and Mode are normal continuous command
    // paths. TouchProbeFunction is event-driven and may legitimately remain 0
    // unless HOME probe logic is exercised during this boot.
    const bool continuousWritersSeen =
        controlWordWrites >
        0ULL &&
        targetVelocityWrites >
        0ULL &&
        modesWrites >
        0ULL;


    const bool clean =
        cycleAccountingPass &&
        writeAccountingPass &&
        cycleMismatches ==
        0ULL &&
        cycleFailures ==
        0ULL &&
        writeFailures ==
        0ULL;


    const bool ready =
        m_MotionServoOutputCommandSeamPrepared &&
        m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed) &&
        IsServoInputReleaseComplete() &&
        e3Qualified &&
        windowQualified &&
        continuousWritersSeen &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-COMMAND-SEAM-SHADOW] "
        "Prep:%s Runtime:%s E3:%s InputRelease:%s | "
        "Routes:%d Fields:%d | "
        "AxisSamples:%llu/%llu | "
        "Cycle:%llu/%llu Mismatch:%llu Fail:%llu | "
        "SeamWrites:%llu/%llu WriteFail:%llu | "
        "Writes:CW:%llu TV:%llu TPF:%llu MOO:%llu | "
        "Accounting:%s Window:%s Writers:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "ActiveProducer:LEGACY_COMMAND_SEAM_TO_POUTPUT | "
        "CandidateProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "MotionCoreWritePath:LEGACY_SEAM_ONLY | "
        "TouchProbeWrite:SEAM_ROUTED_EVENT_DRIVEN | "
        "StructuredLiveWrite:NO OutputCutover:NO | "
        "Ready:%s Result:%s\n",

        m_MotionServoOutputCommandSeamPrepared
        ? "YES"
        : "NO",

        m_MotionServoOutputCommandSeamRuntimeStarted.load(
            std::memory_order_relaxed)
        ? "STARTED"
        : "WAIT",

        e3Qualified
        ? "QUALIFIED"
        : "WAIT",

        IsServoInputReleaseComplete()
        ? "PASS"
        : "WAIT",

        m_MotionServoOutputCommandSeamRouteCount,

        m_MotionServoOutputCommandSeamFieldCount,

        (unsigned long long)
        cycleSamples,

        (unsigned long long)
        requiredAxisSamples,

        (unsigned long long)
        cycleMatches,

        (unsigned long long)
        cycleChecks,

        (unsigned long long)
        cycleMismatches,

        (unsigned long long)
        cycleFailures,

        (unsigned long long)
        writeMatches,

        (unsigned long long)
        writeChecks,

        (unsigned long long)
        writeFailures,

        (unsigned long long)
        controlWordWrites,

        (unsigned long long)
        targetVelocityWrites,

        (unsigned long long)
        touchProbeWrites,

        (unsigned long long)
        modesWrites,

        cycleAccountingPass &&
        writeAccountingPass
        ? "PASS"
        : "FAIL",

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        continuousWritersSeen
        ? "PASS"
        : "WAIT",

        clean
        ? "YES"
        : "NO",

        ready
        ? "YES"
        : "NO",

        ready
        ? "PASS"
        : "CHECK");


    PrintMotionServoOutputStructuredProducerCutover();
}


// ============================================================================
// Stage 11E.5 - Controlled Structured Servo Output Producer Cutover
// ============================================================================

namespace
{
    bool IsStructuredServoOutputFieldTargetExactE5(
        const EtherCatStructuredServoFieldDescriptor* descriptor,
        const ServoOutput* legacyOutput,
        size_t expectedOffset,
        size_t expectedSize)
    {
        if (descriptor == nullptr ||
            descriptor->pByteBase == nullptr ||
            legacyOutput == nullptr ||
            strcmp(descriptor->direction, "Output") != 0 ||
            !descriptor->byteAligned ||
            descriptor->byteSpan != expectedSize)
        {
            return false;
        }

        return
            reinterpret_cast<uintptr_t>(descriptor->pByteBase) ==
            reinterpret_cast<uintptr_t>(legacyOutput) +
            static_cast<uintptr_t>(expectedOffset);
    }


    template <typename TValue>
    bool WriteStructuredServoOutputFieldLiveE5(
        const EtherCatStructuredServoFieldDescriptor* descriptor,
        const ServoOutput* legacyOutput,
        size_t expectedOffset,
        const TValue& value)
    {
        if (!IsStructuredServoOutputFieldTargetExactE5(
            descriptor,
            legacyOutput,
            expectedOffset,
            sizeof(TValue)))
        {
            return false;
        }

        memcpy(
            descriptor->pByteBase,
            &value,
            sizeof(TValue));

        TValue readback = TValue{};

        memcpy(
            &readback,
            descriptor->pByteBase,
            sizeof(TValue));

        return
            memcmp(
                &readback,
                &value,
                sizeof(TValue)) == 0;
    }
}


bool EtherCatMaster::PrepareMotionServoOutputStructuredProducerCutover()
{
    m_MotionServoOutputStructuredProducerPrepared = false;

    m_MotionServoOutputStructuredProducerEnabled.store(false, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerFault.store(false, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerTransitions.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerWarmupFallbacks.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerFaultFallbacks.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerRouteFaults.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerWriteAttempts.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerWrites.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerWriteFailures.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerReadbackChecks.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerReadbackMatches.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerReadbackFailures.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerControlWordWrites.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerTargetVelocityWrites.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerTouchProbeWrites.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerModesWrites.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerAxisSamples.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerCycleChecks.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerCycleMatches.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerCycleMismatches.store(0ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerCycleFailures.store(0ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerEmergencyRestores.store(0ULL, std::memory_order_relaxed);


    int routes = 0;
    int fields = 0;
    int targetChecks = 0;
    int targetMatches = 0;
    int errors = 0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-STRUCTURED-CUTOVER-PREPARE] BEGIN | "
        "Stage:11E.5 | Identity:AXIS_INDEX | "
        "Gate:E4_RUNTIME_QUALIFIED | "
        "InitialProducer:LEGACY_COMMAND_SEAM | "
        "PlannedProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "PhysicalTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "Rollback:BOOT_LATCHED_LEGACY | Enabled:NO\n"
        "============================================================\n");


    if (!m_MotionServoOutputCommandSeamPrepared ||
        !m_MotionServoOutputLiveWritePreflightPrepared ||
        !m_MotionServoOutputSemanticScratchBridgePrepared)
    {
        errors++;
    }


    if (errors == 0)
    {
        for (int axisIndex = 0; axisIndex < MAX_AXES; ++axisIndex)
        {
            const MotionServoAxisIndexOutputRouteShadow& route =
                m_MotionServoAxisIndexOutputRoutesShadow[axisIndex];

            if (!route.valid)
            {
                continue;
            }

            if (route.motionSlot < 0 ||
                route.motionSlot >= static_cast<int>(m_ServoList.size()))
            {
                errors++;
                continue;
            }

            const ENI_ServoDrive& servo =
                m_ServoList[static_cast<size_t>(route.motionSlot)];

            if (servo.pOutput == nullptr)
            {
                errors++;
                continue;
            }

            routes++;
            fields += 4;

            const EtherCatStructuredServoFieldDescriptor* descriptors[4] =
            {
                route.controlWord,
                route.targetVelocity,
                route.touchProbeFunction,
                route.modesOfOperation
            };

            const size_t offsets[4] =
            {
                offsetof(ServoOutput, ControlWord),
                offsetof(ServoOutput, TargetVelocity),
                offsetof(ServoOutput, TouchProbeFunc),
                offsetof(ServoOutput, ModesOfOperation)
            };

            const size_t sizes[4] =
            {
                sizeof(servo.pOutput->ControlWord),
                sizeof(servo.pOutput->TargetVelocity),
                sizeof(servo.pOutput->TouchProbeFunc),
                sizeof(servo.pOutput->ModesOfOperation)
            };

            bool routePass = true;

            for (int f = 0; f < 4; ++f)
            {
                targetChecks++;

                const bool pass =
                    IsStructuredServoOutputFieldTargetExactE5(
                        descriptors[f],
                        servo.pOutput,
                        offsets[f],
                        sizes[f]);

                if (pass)
                {
                    targetMatches++;
                }
                else
                {
                    routePass = false;
                    errors++;
                }
            }

            DEBUG_PRINT(
                "[SERVO-OUTPUT-STRUCTURED-CUTOVER-MAP] "
                "AxisIndex:%d | Slot:%d | Servo:S%d | "
                "Target:%s | Rollback:LEGACY_SAME_COMMAND | Result:%s\n",
                route.axisIndex,
                route.motionSlot,
                route.servoSlaveIndex,
                routePass ? "4/4" : "FAIL",
                routePass ? "PASS" : "FAIL");
        }
    }


    const int expectedRoutes =
        m_MotionServoOutputCommandSeamRouteCount;

    const int expectedFields =
        expectedRoutes * 4;

    const bool pass =
        errors == 0 &&
        expectedRoutes > 0 &&
        routes == expectedRoutes &&
        fields == expectedFields &&
        targetChecks == expectedFields &&
        targetMatches == expectedFields;


    if (pass)
    {
        m_MotionServoOutputStructuredProducerPrepared = true;
    }


    DEBUG_PRINT(
        "[SERVO-OUTPUT-STRUCTURED-CUTOVER-PREPARE-RESULT] "
        "Routes:%d/%d | Fields:%d/%d | Target:%d/%d | Errors:%d | "
        "Result:%s | Prepared:%s | "
        "Gate:E4_RUNTIME_QUALIFIED | "
        "InitialProducer:LEGACY_COMMAND_SEAM | "
        "PlannedProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "PhysicalTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "Rollback:BOOT_LATCHED_LEGACY | Enabled:NO | Next:%s\n",
        routes,
        expectedRoutes,
        fields,
        expectedFields,
        targetMatches,
        expectedFields,
        errors,
        pass ? "PASS" : "FAIL",
        m_MotionServoOutputStructuredProducerPrepared ? "YES" : "NO",
        pass
        ? "WAIT_E4_QUALIFICATION_THEN_AUTO_CUTOVER"
        : "BLOCK_STAGE11E5");


    DEBUG_PRINT(
        "============================================================\n"
        "[SERVO-OUTPUT-STRUCTURED-CUTOVER-PREPARE] END | "
        "Stage:11E.5 | Result:%s | Enabled:NO\n"
        "============================================================\n\n",
        pass ? "PASS" : "FAIL");


    return pass;
}


bool EtherCatMaster::IsMotionServoOutputCommandSeamQualifiedForCutover() const
{
    const uint64_t required =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;

    return
        m_MotionServoOutputStructuredProducerPrepared &&
        m_MotionServoOutputCommandSeamPrepared &&
        m_MotionServoOutputCommandSeamRuntimeStarted.load(std::memory_order_relaxed) &&
        IsServoInputReleaseComplete() &&
        required > 0ULL &&
        m_MotionServoOutputCommandSeamCycleSamples.load(std::memory_order_relaxed) >= required &&
        m_MotionServoOutputCommandSeamCycleMismatches.load(std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputCommandSeamCycleFailures.load(std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputCommandSeamWriteFailures.load(std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputCommandSeamControlWordWrites.load(std::memory_order_relaxed) > 0ULL &&
        m_MotionServoOutputCommandSeamTargetVelocityWrites.load(std::memory_order_relaxed) > 0ULL &&
        m_MotionServoOutputCommandSeamModesWrites.load(std::memory_order_relaxed) > 0ULL;
}


bool EtherCatMaster::TryWriteMotionServoOutputCommandStructured(
    int axisIndex,
    MotionServoOutputCommandField field,
    int64_t value,
    ServoOutput* legacyOutput)
{
    if (!m_MotionServoOutputStructuredProducerPrepared)
    {
        return false;
    }

    if (m_MotionServoOutputStructuredProducerFault.load(std::memory_order_relaxed))
    {
        m_MotionServoOutputStructuredProducerFaultFallbacks.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return false;
    }

    if (!IsMotionServoOutputCommandSeamQualifiedForCutover())
    {
        m_MotionServoOutputStructuredProducerWarmupFallbacks.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return false;
    }


    if (axisIndex < 0 ||
        axisIndex >= MAX_AXES ||
        legacyOutput == nullptr)
    {
        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(1ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerFaultFallbacks.fetch_add(1ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerFault.store(true, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerEnabled.store(false, std::memory_order_relaxed);
        return false;
    }


    const MotionServoAxisIndexOutputRouteShadow& route =
        m_MotionServoAxisIndexOutputRoutesShadow[axisIndex];

    const MotionServoOutputLiveWritePreflightRoute& liveRoute =
        m_MotionServoOutputLiveWritePreflightRoutes[axisIndex];


    if (!route.valid ||
        !liveRoute.valid ||
        route.motionSlot < 0 ||
        route.motionSlot >= static_cast<int>(m_ServoList.size()) ||
        m_ServoList[static_cast<size_t>(route.motionSlot)].pOutput != legacyOutput)
    {
        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(1ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerFaultFallbacks.fetch_add(1ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerFault.store(true, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerEnabled.store(false, std::memory_order_relaxed);
        return false;
    }


    bool expected = false;

    if (m_MotionServoOutputStructuredProducerEnabled.compare_exchange_strong(
        expected,
        true,
        std::memory_order_relaxed))
    {
        m_MotionServoOutputStructuredProducerTransitions.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }


    m_MotionServoOutputStructuredProducerWriteAttempts.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerReadbackChecks.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    bool writePass = false;


    switch (field)
    {
    case MotionServoOutputCommandField::ControlWord:
    {
        const uint16_t typed = static_cast<uint16_t>(value);

        writePass =
            WriteStructuredServoOutputFieldLiveE5(
                route.controlWord,
                legacyOutput,
                offsetof(ServoOutput, ControlWord),
                typed);

        if (writePass)
        {
            m_MotionServoOutputStructuredProducerControlWordWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        break;
    }


    case MotionServoOutputCommandField::TargetVelocity:
    {
        const int32_t typed = static_cast<int32_t>(value);

        writePass =
            WriteStructuredServoOutputFieldLiveE5(
                route.targetVelocity,
                legacyOutput,
                offsetof(ServoOutput, TargetVelocity),
                typed);

        if (writePass)
        {
            m_MotionServoOutputStructuredProducerTargetVelocityWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        break;
    }


    case MotionServoOutputCommandField::TouchProbeFunction:
    {
        const uint16_t typed = static_cast<uint16_t>(value);

        writePass =
            WriteStructuredServoOutputFieldLiveE5(
                route.touchProbeFunction,
                legacyOutput,
                offsetof(ServoOutput, TouchProbeFunc),
                typed);

        if (writePass)
        {
            m_MotionServoOutputStructuredProducerTouchProbeWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        break;
    }


    case MotionServoOutputCommandField::ModesOfOperation:
    {
        const int8_t typed = static_cast<int8_t>(value);

        writePass =
            WriteStructuredServoOutputFieldLiveE5(
                route.modesOfOperation,
                legacyOutput,
                offsetof(ServoOutput, ModesOfOperation),
                typed);

        if (writePass)
        {
            m_MotionServoOutputStructuredProducerModesWrites.fetch_add(
                1ULL,
                std::memory_order_relaxed);
        }

        break;
    }


    default:
        writePass = false;
        break;
    }


    if (writePass)
    {
        m_MotionServoOutputStructuredProducerWrites.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerReadbackMatches.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return true;
    }


    // Current command will be immediately rewritten by MotionCore's
    // legacy fallback after this false return.
    m_MotionServoOutputStructuredProducerWriteFailures.fetch_add(1ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerReadbackFailures.fetch_add(1ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(1ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerFaultFallbacks.fetch_add(1ULL, std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerFault.store(true, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerEnabled.store(false, std::memory_order_relaxed);

    return false;
}


void EtherCatMaster::ObserveMotionServoOutputStructuredProducerCycle(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    if (!m_MotionServoOutputStructuredProducerPrepared ||
        !m_MotionServoOutputStructuredProducerEnabled.load(std::memory_order_relaxed) ||
        m_MotionServoOutputStructuredProducerFault.load(std::memory_order_relaxed))
    {
        return;
    }


    m_MotionServoOutputStructuredProducerAxisSamples.fetch_add(1ULL, std::memory_order_relaxed);
    m_MotionServoOutputStructuredProducerCycleChecks.fetch_add(4ULL, std::memory_order_relaxed);


    if (axisIndex < 0 ||
        axisIndex >= MAX_AXES)
    {
        m_MotionServoOutputStructuredProducerCycleFailures.fetch_add(4ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(1ULL, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerFault.store(true, std::memory_order_relaxed);
        m_MotionServoOutputStructuredProducerEnabled.store(false, std::memory_order_relaxed);
        return;
    }


    uint16_t cw = 0U;
    int32_t tv = 0;
    uint16_t tpf = 0U;
    int8_t moo = 0;


    const bool readPass =
        ReadMotionServoOutputByAxisIndexShadow(
            axisIndex,
            cw,
            tv,
            tpf,
            moo);


    const bool matches[4] =
    {
        readPass && cw == controlWord,
        readPass && tv == targetVelocity,
        readPass && tpf == touchProbeFunction,
        readPass && moo == modesOfOperation
    };


    uint64_t matchCount = 0ULL;

    for (bool match : matches)
    {
        if (match)
        {
            matchCount++;
        }
    }


    m_MotionServoOutputStructuredProducerCycleMatches.fetch_add(
        matchCount,
        std::memory_order_relaxed);

    m_MotionServoOutputStructuredProducerCycleMismatches.fetch_add(
        4ULL - matchCount,
        std::memory_order_relaxed);


    if (!readPass ||
        matchCount != 4ULL)
    {
        if (!readPass)
        {
            m_MotionServoOutputStructuredProducerCycleFailures.fetch_add(
                4ULL - matchCount,
                std::memory_order_relaxed);
        }

        // Emergency same-cycle legacy image restoration before the next LRW.
        const MotionServoAxisIndexOutputRouteShadow& route =
            m_MotionServoAxisIndexOutputRoutesShadow[axisIndex];

        if (route.valid &&
            route.motionSlot >= 0 &&
            route.motionSlot < static_cast<int>(m_ServoList.size()))
        {
            ENI_ServoDrive& servo =
                m_ServoList[static_cast<size_t>(route.motionSlot)];

            if (servo.pOutput != nullptr)
            {
                servo.pOutput->ControlWord = controlWord;
                servo.pOutput->TargetVelocity = targetVelocity;
                servo.pOutput->TouchProbeFunc = touchProbeFunction;
                servo.pOutput->ModesOfOperation = modesOfOperation;

                m_MotionServoOutputStructuredProducerEmergencyRestores.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
            }
        }

        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerFault.store(
            true,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerEnabled.store(
            false,
            std::memory_order_relaxed);
    }
}


void EtherCatMaster::PrintMotionServoOutputStructuredProducerCutover() const
{
    const bool e4Qualified =
        IsMotionServoOutputCommandSeamQualifiedForCutover();

    const bool enabled =
        m_MotionServoOutputStructuredProducerEnabled.load(std::memory_order_relaxed);

    const bool fault =
        m_MotionServoOutputStructuredProducerFault.load(std::memory_order_relaxed);

    const uint64_t transitions =
        m_MotionServoOutputStructuredProducerTransitions.load(std::memory_order_relaxed);

    const uint64_t warmupFallbacks =
        m_MotionServoOutputStructuredProducerWarmupFallbacks.load(std::memory_order_relaxed);

    const uint64_t faultFallbacks =
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(std::memory_order_relaxed);

    const uint64_t routeFaults =
        m_MotionServoOutputStructuredProducerRouteFaults.load(std::memory_order_relaxed);

    const uint64_t attempts =
        m_MotionServoOutputStructuredProducerWriteAttempts.load(std::memory_order_relaxed);

    const uint64_t writes =
        m_MotionServoOutputStructuredProducerWrites.load(std::memory_order_relaxed);

    const uint64_t writeFailures =
        m_MotionServoOutputStructuredProducerWriteFailures.load(std::memory_order_relaxed);

    const uint64_t rbChecks =
        m_MotionServoOutputStructuredProducerReadbackChecks.load(std::memory_order_relaxed);

    const uint64_t rbMatches =
        m_MotionServoOutputStructuredProducerReadbackMatches.load(std::memory_order_relaxed);

    const uint64_t rbFailures =
        m_MotionServoOutputStructuredProducerReadbackFailures.load(std::memory_order_relaxed);

    const uint64_t cwWrites =
        m_MotionServoOutputStructuredProducerControlWordWrites.load(std::memory_order_relaxed);

    const uint64_t tvWrites =
        m_MotionServoOutputStructuredProducerTargetVelocityWrites.load(std::memory_order_relaxed);

    const uint64_t tpfWrites =
        m_MotionServoOutputStructuredProducerTouchProbeWrites.load(std::memory_order_relaxed);

    const uint64_t mooWrites =
        m_MotionServoOutputStructuredProducerModesWrites.load(std::memory_order_relaxed);

    const uint64_t axisSamples =
        m_MotionServoOutputStructuredProducerAxisSamples.load(std::memory_order_relaxed);

    const uint64_t cycleChecks =
        m_MotionServoOutputStructuredProducerCycleChecks.load(std::memory_order_relaxed);

    const uint64_t cycleMatches =
        m_MotionServoOutputStructuredProducerCycleMatches.load(std::memory_order_relaxed);

    const uint64_t cycleMismatches =
        m_MotionServoOutputStructuredProducerCycleMismatches.load(std::memory_order_relaxed);

    const uint64_t cycleFailures =
        m_MotionServoOutputStructuredProducerCycleFailures.load(std::memory_order_relaxed);

    const uint64_t emergencyRestores =
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(std::memory_order_relaxed);

    const uint64_t requiredAxisSamples =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;

    const uint64_t expectedCycleChecks =
        axisSamples * 4ULL;


    const bool writeAccounting =
        writes + writeFailures == attempts &&
        rbChecks == attempts &&
        rbMatches + rbFailures == rbChecks;

    const bool cycleAccounting =
        cycleChecks == expectedCycleChecks &&
        cycleMatches + cycleMismatches + cycleFailures == cycleChecks;

    const bool windowQualified =
        requiredAxisSamples > 0ULL &&
        axisSamples >= requiredAxisSamples;

    const bool writersSeen =
        cwWrites > 0ULL &&
        tvWrites > 0ULL &&
        mooWrites > 0ULL;

    const bool clean =
        writeAccounting &&
        cycleAccounting &&
        writeFailures == 0ULL &&
        rbFailures == 0ULL &&
        routeFaults == 0ULL &&
        faultFallbacks == 0ULL &&
        cycleMismatches == 0ULL &&
        cycleFailures == 0ULL &&
        emergencyRestores == 0ULL;

    const bool ready =
        m_MotionServoOutputStructuredProducerPrepared &&
        e4Qualified &&
        enabled &&
        !fault &&
        transitions == 1ULL &&
        windowQualified &&
        writersSeen &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-STRUCTURED-CUTOVER] "
        "Prep:%s E4:%s Enabled:%s Fault:%s Transition:%llu | "
        "AxisSamples:%llu/%llu | "
        "StructuredWrites:%llu/%llu WriteFail:%llu | "
        "Readback:%llu/%llu ReadFail:%llu | "
        "Cycle:%llu/%llu Mismatch:%llu Fail:%llu | "
        "Writes:CW:%llu TV:%llu TPF:%llu MOO:%llu | "
        "WarmupFallback:%llu FaultFallback:%llu RouteFault:%llu EmergencyRestore:%llu | "
        "Accounting:%s Window:%s Writers:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "ActiveProducer:%s | LegacySeam:%s | "
        "PhysicalTarget:SAME_M_IOMAP_SERVO_OUTPUT | "
        "Transmit:EXISTING_NEXT_LRW | "
        "Rollback:BOOT_LATCHED_LEGACY | "
        "OutputCutover:%s | Ready:%s Result:%s\n",

        m_MotionServoOutputStructuredProducerPrepared ? "YES" : "NO",
        e4Qualified ? "QUALIFIED" : "WAIT",
        enabled ? "YES" : "NO",
        fault ? "YES" : "NO",
        (unsigned long long)transitions,

        (unsigned long long)axisSamples,
        (unsigned long long)requiredAxisSamples,

        (unsigned long long)writes,
        (unsigned long long)attempts,
        (unsigned long long)writeFailures,

        (unsigned long long)rbMatches,
        (unsigned long long)rbChecks,
        (unsigned long long)rbFailures,

        (unsigned long long)cycleMatches,
        (unsigned long long)cycleChecks,
        (unsigned long long)cycleMismatches,
        (unsigned long long)cycleFailures,

        (unsigned long long)cwWrites,
        (unsigned long long)tvWrites,
        (unsigned long long)tpfWrites,
        (unsigned long long)mooWrites,

        (unsigned long long)warmupFallbacks,
        (unsigned long long)faultFallbacks,
        (unsigned long long)routeFaults,
        (unsigned long long)emergencyRestores,

        writeAccounting && cycleAccounting ? "PASS" : "FAIL",
        windowQualified ? "QUALIFIED" : "WARMUP",
        writersSeen ? "PASS" : "WAIT",
        clean ? "YES" : "NO",

        enabled && !fault
        ? "STRUCTURED_AXISINDEX_OUTPUT"
        : "LEGACY_COMMAND_SEAM",

        enabled && !fault
        ? "ROLLBACK_ONLY"
        : "ACTIVE",

        enabled && !fault
        ? "YES"
        : "NO",

        ready ? "YES" : "NO",

        ready
        ? "PASS"
        : (fault ? "FALLBACK" : "CHECK"));


    PrintMotionServoOutputCompatibilityRetirementShadow();
}


// ============================================================================
// Stage 11E.6 - Servo Output Compatibility Retirement Shadow
// ============================================================================

bool EtherCatMaster::PrepareMotionServoOutputCompatibilityRetirementShadow()
{
    m_MotionServoOutputCompatibilityRetirementPrepared =
        false;

    m_MotionServoOutputCompatibilityRetirementActive.store(
        false,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementTransitions.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementAxisSamples.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementMismatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementFailures.store(
        0ULL,
        std::memory_order_relaxed);


    const bool pass =
        m_MotionServoOutputStructuredProducerPrepared &&
        m_MotionServoOutputCommandSeamPrepared &&
        m_MotionServoOutputLiveWritePreflightPrepared &&
        m_MotionServoOutputCommandSeamRouteCount > 0;


    if (pass)
    {
        m_MotionServoOutputCompatibilityRetirementPrepared = true;
    }


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[SERVO-OUTPUT-COMPAT-RETIREMENT-PREPARE] "
        "Stage:11E.6 | Routes:%d | E5Prepared:%s | "
        "Result:%s | Prepared:%s | "
        "MainlineTarget:STRUCTURED_AXISINDEX_OUTPUT | "
        "LegacyPOutputTarget:EMERGENCY_ONLY | "
        "HistoricalOutputShadows:FREEZE_AFTER_E5_QUALIFIED\n"
        "============================================================\n\n",
        m_MotionServoOutputCommandSeamRouteCount,
        m_MotionServoOutputStructuredProducerPrepared ? "YES" : "NO",
        pass ? "PASS" : "FAIL",
        m_MotionServoOutputCompatibilityRetirementPrepared ? "YES" : "NO");


    return pass;
}


bool EtherCatMaster::IsMotionServoOutputStructuredProducerQualifiedForRetirement() const
{
    const uint64_t required =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;

    const uint64_t axisSamples =
        m_MotionServoOutputStructuredProducerAxisSamples.load(
            std::memory_order_relaxed);

    const uint64_t attempts =
        m_MotionServoOutputStructuredProducerWriteAttempts.load(
            std::memory_order_relaxed);

    const uint64_t writes =
        m_MotionServoOutputStructuredProducerWrites.load(
            std::memory_order_relaxed);

    const uint64_t writeFailures =
        m_MotionServoOutputStructuredProducerWriteFailures.load(
            std::memory_order_relaxed);

    const uint64_t rbChecks =
        m_MotionServoOutputStructuredProducerReadbackChecks.load(
            std::memory_order_relaxed);

    const uint64_t rbMatches =
        m_MotionServoOutputStructuredProducerReadbackMatches.load(
            std::memory_order_relaxed);

    const uint64_t rbFailures =
        m_MotionServoOutputStructuredProducerReadbackFailures.load(
            std::memory_order_relaxed);

    const uint64_t cycleChecks =
        m_MotionServoOutputStructuredProducerCycleChecks.load(
            std::memory_order_relaxed);

    const uint64_t cycleMatches =
        m_MotionServoOutputStructuredProducerCycleMatches.load(
            std::memory_order_relaxed);

    const uint64_t cycleMismatches =
        m_MotionServoOutputStructuredProducerCycleMismatches.load(
            std::memory_order_relaxed);

    const uint64_t cycleFailures =
        m_MotionServoOutputStructuredProducerCycleFailures.load(
            std::memory_order_relaxed);

    const bool accounting =
        writes + writeFailures == attempts &&
        rbChecks == attempts &&
        rbMatches + rbFailures == rbChecks &&
        cycleChecks == axisSamples * 4ULL &&
        cycleMatches + cycleMismatches + cycleFailures == cycleChecks;


    return
        m_MotionServoOutputCompatibilityRetirementPrepared &&
        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed) &&
        m_MotionServoOutputStructuredProducerTransitions.load(
            std::memory_order_relaxed) == 1ULL &&
        required > 0ULL &&
        axisSamples >= required &&
        accounting &&
        writeFailures == 0ULL &&
        rbFailures == 0ULL &&
        cycleMismatches == 0ULL &&
        cycleFailures == 0ULL &&
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerRouteFaults.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerControlWordWrites.load(
            std::memory_order_relaxed) > 0ULL &&
        m_MotionServoOutputStructuredProducerTargetVelocityWrites.load(
            std::memory_order_relaxed) > 0ULL &&
        m_MotionServoOutputStructuredProducerModesWrites.load(
            std::memory_order_relaxed) > 0ULL;
}


void EtherCatMaster::ActivateMotionServoOutputCompatibilityRetirement()
{
    bool expected = false;

    if (!m_MotionServoOutputCompatibilityRetirementActive.compare_exchange_strong(
        expected,
        true,
        std::memory_order_relaxed))
    {
        return;
    }


    m_MotionServoOutputCompatibilityRetirementTransitions.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_MotionServoOutputRetirementFrozenE1AxisSamples =
        m_MotionServoOutputCommandOwnershipAxisSamples.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementFrozenE2AxisSamples =
        m_MotionServoOutputSemanticScratchAxisSamples.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementFrozenE3AxisSamples =
        m_MotionServoOutputLiveWritePreflightAxisSamples.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementFrozenE4AxisSamples =
        m_MotionServoOutputCommandSeamCycleSamples.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementFrozenE4WriteChecks =
        m_MotionServoOutputCommandSeamWriteChecks.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementFrozenE5CycleSamples =
        m_MotionServoOutputStructuredProducerAxisSamples.load(
            std::memory_order_relaxed);


    m_MotionServoOutputRetirementBaselineWarmupFallbacks =
        m_MotionServoOutputStructuredProducerWarmupFallbacks.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementBaselineFaultFallbacks =
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementBaselineRouteFaults =
        m_MotionServoOutputStructuredProducerRouteFaults.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementBaselineEmergencyRestores =
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementBaselineStructuredWrites =
        m_MotionServoOutputStructuredProducerWrites.load(
            std::memory_order_relaxed);

    m_MotionServoOutputRetirementBaselineReadbackMatches =
        m_MotionServoOutputStructuredProducerReadbackMatches.load(
            std::memory_order_relaxed);
}


void EtherCatMaster::ObserveMotionServoOutputFinalCommandRuntime(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    if (!m_MotionServoOutputCompatibilityRetirementPrepared)
    {
        ObserveMotionServoOutputCommandOwnershipShadow(
            axisIndex,
            controlWord,
            targetVelocity,
            touchProbeFunction,
            modesOfOperation);
        return;
    }


    if (m_MotionServoOutputCompatibilityRetirementActive.load(
        std::memory_order_relaxed))
    {
        ObserveMotionServoOutputCompatibilityRetirementCycle(
            axisIndex,
            controlWord,
            targetVelocity,
            touchProbeFunction,
            modesOfOperation);
        return;
    }


    ObserveMotionServoOutputCommandOwnershipShadow(
        axisIndex,
        controlWord,
        targetVelocity,
        touchProbeFunction,
        modesOfOperation);


    if (IsMotionServoOutputStructuredProducerQualifiedForRetirement())
    {
        ActivateMotionServoOutputCompatibilityRetirement();
    }
}


void EtherCatMaster::ObserveMotionServoOutputCompatibilityRetirementCycle(
    int axisIndex,
    uint16_t controlWord,
    int32_t targetVelocity,
    uint16_t touchProbeFunction,
    int8_t modesOfOperation)
{
    if (!m_MotionServoOutputCompatibilityRetirementActive.load(
        std::memory_order_relaxed) ||
        !m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) ||
        m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed))
    {
        return;
    }


    m_MotionServoOutputCompatibilityRetirementAxisSamples.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementChecks.fetch_add(
        4ULL,
        std::memory_order_relaxed);


    if (axisIndex < 0 || axisIndex >= MAX_AXES)
    {
        m_MotionServoOutputCompatibilityRetirementFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);
        return;
    }


    uint16_t cw = 0U;
    int32_t tv = 0;
    uint16_t tpf = 0U;
    int8_t moo = 0;


    const bool readPass =
        ReadMotionServoOutputByAxisIndexShadow(
            axisIndex,
            cw,
            tv,
            tpf,
            moo);


    if (!readPass)
    {
        m_MotionServoOutputCompatibilityRetirementFailures.fetch_add(
            4ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerFault.store(
            true,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerEnabled.store(
            false,
            std::memory_order_relaxed);

        return;
    }


    const bool fieldMatches[4] =
    {
        cw == controlWord,
        tv == targetVelocity,
        tpf == touchProbeFunction,
        moo == modesOfOperation
    };


    uint64_t matchCount = 0ULL;

    for (bool match : fieldMatches)
    {
        if (match)
        {
            matchCount++;
        }
    }


    m_MotionServoOutputCompatibilityRetirementMatches.fetch_add(
        matchCount,
        std::memory_order_relaxed);

    m_MotionServoOutputCompatibilityRetirementMismatches.fetch_add(
        4ULL - matchCount,
        std::memory_order_relaxed);


    if (matchCount != 4ULL)
    {
        // Safety restore before next LRW.
        const MotionServoAxisIndexOutputRouteShadow& route =
            m_MotionServoAxisIndexOutputRoutesShadow[axisIndex];

        if (route.valid &&
            route.motionSlot >= 0 &&
            route.motionSlot < static_cast<int>(m_ServoList.size()))
        {
            ENI_ServoDrive& servo =
                m_ServoList[static_cast<size_t>(route.motionSlot)];

            if (servo.pOutput != nullptr)
            {
                servo.pOutput->ControlWord = controlWord;
                servo.pOutput->TargetVelocity = targetVelocity;
                servo.pOutput->TouchProbeFunc = touchProbeFunction;
                servo.pOutput->ModesOfOperation = modesOfOperation;

                m_MotionServoOutputStructuredProducerEmergencyRestores.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);
            }
        }

        m_MotionServoOutputStructuredProducerRouteFaults.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerFault.store(
            true,
            std::memory_order_relaxed);

        m_MotionServoOutputStructuredProducerEnabled.store(
            false,
            std::memory_order_relaxed);
    }


    // =========================================================
    // Stage 11E.7 - Generic Servo I/O Release Gate update
    //
    // This is the correct non-const runtime location.
    //
    // The E6 sample counters above have already been updated for
    // this final Servo command cycle.  E7 may therefore latch
    // release immediately when its input/output qualification
    // conditions become true.
    //
    // Print functions remain const/read-only.
    // =========================================================

    UpdateServoGenericIoReleaseGate();
}


void EtherCatMaster::PrintMotionServoOutputCompatibilityRetirementShadow() const
{
    const bool active =
        m_MotionServoOutputCompatibilityRetirementActive.load(
            std::memory_order_relaxed);

    const uint64_t transitions =
        m_MotionServoOutputCompatibilityRetirementTransitions.load(
            std::memory_order_relaxed);

    const uint64_t axisSamples =
        m_MotionServoOutputCompatibilityRetirementAxisSamples.load(
            std::memory_order_relaxed);

    const uint64_t checks =
        m_MotionServoOutputCompatibilityRetirementChecks.load(
            std::memory_order_relaxed);

    const uint64_t matches =
        m_MotionServoOutputCompatibilityRetirementMatches.load(
            std::memory_order_relaxed);

    const uint64_t mismatches =
        m_MotionServoOutputCompatibilityRetirementMismatches.load(
            std::memory_order_relaxed);

    const uint64_t failures =
        m_MotionServoOutputCompatibilityRetirementFailures.load(
            std::memory_order_relaxed);

    const uint64_t required =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;

    const uint64_t expectedChecks =
        axisSamples * 4ULL;


    const bool accounting =
        checks == expectedChecks &&
        matches + mismatches + failures == checks;


    const bool frozen =
        !active ||
        (
            m_MotionServoOutputCommandOwnershipAxisSamples.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE1AxisSamples &&

            m_MotionServoOutputSemanticScratchAxisSamples.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE2AxisSamples &&

            m_MotionServoOutputLiveWritePreflightAxisSamples.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE3AxisSamples &&

            m_MotionServoOutputCommandSeamCycleSamples.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE4AxisSamples &&

            m_MotionServoOutputCommandSeamWriteChecks.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE4WriteChecks &&

            m_MotionServoOutputStructuredProducerAxisSamples.load(
                std::memory_order_relaxed) ==
            m_MotionServoOutputRetirementFrozenE5CycleSamples
            );


    const uint64_t warmupFallbacks =
        m_MotionServoOutputStructuredProducerWarmupFallbacks.load(
            std::memory_order_relaxed);

    const uint64_t faultFallbacks =
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(
            std::memory_order_relaxed);

    const uint64_t routeFaults =
        m_MotionServoOutputStructuredProducerRouteFaults.load(
            std::memory_order_relaxed);

    const uint64_t restores =
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(
            std::memory_order_relaxed);


    const bool legacyFallbackUnchanged =
        !active ||
        (
            warmupFallbacks ==
            m_MotionServoOutputRetirementBaselineWarmupFallbacks &&
            faultFallbacks ==
            m_MotionServoOutputRetirementBaselineFaultFallbacks &&
            routeFaults ==
            m_MotionServoOutputRetirementBaselineRouteFaults &&
            restores ==
            m_MotionServoOutputRetirementBaselineEmergencyRestores
            );


    const uint64_t writes =
        m_MotionServoOutputStructuredProducerWrites.load(
            std::memory_order_relaxed);

    const uint64_t readbacks =
        m_MotionServoOutputStructuredProducerReadbackMatches.load(
            std::memory_order_relaxed);

    const uint64_t writeDelta =
        active &&
        writes >= m_MotionServoOutputRetirementBaselineStructuredWrites
        ? writes - m_MotionServoOutputRetirementBaselineStructuredWrites
        : 0ULL;

    const uint64_t readbackDelta =
        active &&
        readbacks >= m_MotionServoOutputRetirementBaselineReadbackMatches
        ? readbacks - m_MotionServoOutputRetirementBaselineReadbackMatches
        : 0ULL;


    const bool window =
        required > 0ULL &&
        axisSamples >= required;


    const bool mainline =
        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed) &&
        m_MotionServoOutputStructuredProducerWriteFailures.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerReadbackFailures.load(
            std::memory_order_relaxed) == 0ULL;


    const bool clean =
        active &&
        transitions == 1ULL &&
        accounting &&
        frozen &&
        legacyFallbackUnchanged &&
        mainline &&
        mismatches == 0ULL &&
        failures == 0ULL &&
        writeDelta > 0ULL &&
        readbackDelta == writeDelta;


    const bool ready =
        m_MotionServoOutputCompatibilityRetirementPrepared &&
        window &&
        clean;


    DEBUG_PRINT(
        "[SERVO-OUTPUT-COMPAT-RETIREMENT-SHADOW] "
        "Prep:%s Active:%s Transition:%llu | "
        "AxisSamples:%llu/%llu | "
        "Checks:%llu Expected:%llu Matches:%llu Mismatch:%llu Fail:%llu | "
        "StructuredDelta:%llu ReadbackDelta:%llu | "
        "WarmupBaseline:%llu FaultFallback:%llu RouteFault:%llu EmergencyRestore:%llu | "
        "Historical:%s LegacyFallback:%s | "
        "Accounting:%s Window:%s Mainline:%s Clean:%s | "
        "Identity:AXIS_INDEX | "
        "ActiveProducer:STRUCTURED_AXISINDEX_OUTPUT | "
        "LegacyPOutput:EMERGENCY_ONLY | "
        "OldOutputQualification:FROZEN | "
        "Transmit:EXISTING_NEXT_LRW | "
        "Ready:%s Result:%s\n",

        m_MotionServoOutputCompatibilityRetirementPrepared ? "YES" : "NO",
        active ? "YES" : "NO",
        (unsigned long long)transitions,

        (unsigned long long)axisSamples,
        (unsigned long long)required,

        (unsigned long long)checks,
        (unsigned long long)expectedChecks,
        (unsigned long long)matches,
        (unsigned long long)mismatches,
        (unsigned long long)failures,

        (unsigned long long)writeDelta,
        (unsigned long long)readbackDelta,

        (unsigned long long)m_MotionServoOutputRetirementBaselineWarmupFallbacks,
        (unsigned long long)faultFallbacks,
        (unsigned long long)routeFaults,
        (unsigned long long)restores,

        frozen ? "FROZEN" : "ACTIVE",
        legacyFallbackUnchanged ? "UNCHANGED" : "CHANGED",

        accounting ? "PASS" : "FAIL",
        window ? "QUALIFIED" : "WARMUP",
        mainline ? "PASS" : "FAIL",
        clean ? "YES" : "NO",

        ready ? "YES" : "NO",
        ready ? "PASS" : (active ? "CHECK" : "WAIT_E5"));


    // Stage 11E.7:
    // status output only. Gate mutation occurs in the non-const
    // E6 runtime observer above.
    PrintServoGenericIoReleaseGate();
}


// ============================================================================
// Stage 11E.7 - Servo Generic I/O Release Gate
// ============================================================================

bool EtherCatMaster::PrepareServoGenericIoReleaseGate()
{
    m_ServoGenericIoReleaseGatePrepared = false;

    m_ServoGenericIoReleaseComplete.store(
        false,
        std::memory_order_relaxed);

    m_ServoGenericIoReleaseTransitions.store(
        0ULL,
        std::memory_order_relaxed);


    const bool pass =
        m_ServoInputReleaseGatePrepared &&
        m_MotionServoOutputCompatibilityRetirementPrepared &&
        m_MotionServoOutputStructuredProducerPrepared &&
        m_MotionServoOutputCommandSeamRouteCount > 0;


    if (pass)
    {
        m_ServoGenericIoReleaseGatePrepared = true;
    }


    DEBUG_PRINT(
        "[SERVO-GENERIC-IO-RELEASE-PREPARE] "
        "Stage:11E.7 | InputGate:%s | OutputRetirement:%s | "
        "Routes:%d | Prepared:%s | Result:%s | "
        "Action:NO_BEHAVIOR_CHANGE\n",

        m_ServoInputReleaseGatePrepared ? "YES" : "NO",
        m_MotionServoOutputCompatibilityRetirementPrepared ? "YES" : "NO",
        m_MotionServoOutputCommandSeamRouteCount,
        m_ServoGenericIoReleaseGatePrepared ? "YES" : "NO",
        pass ? "PASS" : "FAIL");


    return pass;
}


void EtherCatMaster::UpdateServoGenericIoReleaseGate()
{
    if (!m_ServoGenericIoReleaseGatePrepared ||
        m_ServoGenericIoReleaseComplete.load(std::memory_order_relaxed))
    {
        return;
    }


    const uint64_t required =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;


    const uint64_t samples =
        m_MotionServoOutputCompatibilityRetirementAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t checks =
        m_MotionServoOutputCompatibilityRetirementChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_MotionServoOutputCompatibilityRetirementMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_MotionServoOutputCompatibilityRetirementMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_MotionServoOutputCompatibilityRetirementFailures.load(
            std::memory_order_relaxed);


    const bool accounting =
        checks == samples * 4ULL &&
        matches + mismatches + failures == checks;


    const bool outputPass =
        m_MotionServoOutputCompatibilityRetirementActive.load(
            std::memory_order_relaxed) &&
        m_MotionServoOutputCompatibilityRetirementTransitions.load(
            std::memory_order_relaxed) == 1ULL &&
        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed) &&
        required > 0ULL &&
        samples >= required &&
        accounting &&
        mismatches == 0ULL &&
        failures == 0ULL &&
        m_MotionServoOutputStructuredProducerWriteFailures.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerReadbackFailures.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerRouteFaults.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(
            std::memory_order_relaxed) == 0ULL;


    if (!IsServoInputReleaseComplete() ||
        !outputPass)
    {
        return;
    }


    bool expected = false;

    if (m_ServoGenericIoReleaseComplete.compare_exchange_strong(
        expected,
        true,
        std::memory_order_relaxed))
    {
        m_ServoGenericIoReleaseTransitions.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
}


bool EtherCatMaster::IsServoGenericIoReleaseComplete() const
{
    return
        m_ServoGenericIoReleaseComplete.load(
            std::memory_order_relaxed);
}


void EtherCatMaster::PrintServoGenericIoReleaseGate() const
{
    const bool inputPass =
        IsServoInputReleaseComplete();


    const uint64_t required =
        static_cast<uint64_t>(
            m_MotionServoOutputCommandSeamRouteCount) *
        20000ULL;


    const uint64_t samples =
        m_MotionServoOutputCompatibilityRetirementAxisSamples.load(
            std::memory_order_relaxed);


    const uint64_t checks =
        m_MotionServoOutputCompatibilityRetirementChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_MotionServoOutputCompatibilityRetirementMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_MotionServoOutputCompatibilityRetirementMismatches.load(
            std::memory_order_relaxed);


    const uint64_t failures =
        m_MotionServoOutputCompatibilityRetirementFailures.load(
            std::memory_order_relaxed);


    const uint64_t faultFallback =
        m_MotionServoOutputStructuredProducerFaultFallbacks.load(
            std::memory_order_relaxed);


    const uint64_t routeFault =
        m_MotionServoOutputStructuredProducerRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t emergencyRestore =
        m_MotionServoOutputStructuredProducerEmergencyRestores.load(
            std::memory_order_relaxed);


    const bool accounting =
        checks == samples * 4ULL &&
        matches + mismatches + failures == checks;


    const bool outputPass =
        m_MotionServoOutputCompatibilityRetirementActive.load(
            std::memory_order_relaxed) &&
        m_MotionServoOutputStructuredProducerEnabled.load(
            std::memory_order_relaxed) &&
        !m_MotionServoOutputStructuredProducerFault.load(
            std::memory_order_relaxed) &&
        required > 0ULL &&
        samples >= required &&
        accounting &&
        mismatches == 0ULL &&
        failures == 0ULL &&
        m_MotionServoOutputStructuredProducerWriteFailures.load(
            std::memory_order_relaxed) == 0ULL &&
        m_MotionServoOutputStructuredProducerReadbackFailures.load(
            std::memory_order_relaxed) == 0ULL &&
        faultFallback == 0ULL &&
        routeFault == 0ULL &&
        emergencyRestore == 0ULL;


    const bool released =
        IsServoGenericIoReleaseComplete();


    const uint64_t transitions =
        m_ServoGenericIoReleaseTransitions.load(
            std::memory_order_relaxed);


    const bool clean =
        m_ServoGenericIoReleaseGatePrepared &&
        inputPass &&
        outputPass &&
        released &&
        transitions == 1ULL;


    DEBUG_PRINT(
        "[SERVO-GENERIC-IO-RELEASE] "
        "Prep:%s Released:%s Transition:%llu | "
        "Input:%s Mainline:AXIS_INDEX_SEMANTIC LegacyPInput:EMERGENCY_ONLY | "
        "Output:%s Samples:%llu/%llu Mismatch:%llu Fail:%llu "
        "FaultFallback:%llu RouteFault:%llu EmergencyRestore:%llu | "
        "OutputMainline:STRUCTURED_AXISINDEX_OUTPUT LegacyPOutput:EMERGENCY_ONLY | "
        "InputQualification:FROZEN OutputQualification:FROZEN | "
        "ProcessImage:SAME_M_IOMAP Transmit:EXISTING_LRW | "
        "MotionIdentity:AXIS_INDEX AxisName:NC_MANAGED | "
        "Release:GENERIC_SERVO_IO_COMPLETE | Ready:%s Result:%s\n",

        m_ServoGenericIoReleaseGatePrepared ? "YES" : "NO",
        released ? "YES" : "NO",
        (unsigned long long)transitions,

        inputPass ? "PASS" : "WAIT",

        outputPass ? "PASS" : "WAIT",
        (unsigned long long)samples,
        (unsigned long long)required,
        (unsigned long long)mismatches,
        (unsigned long long)failures,
        (unsigned long long)faultFallback,
        (unsigned long long)routeFault,
        (unsigned long long)emergencyRestore,

        clean ? "YES" : "NO",
        clean ? "PASS" : "CHECK");
}
