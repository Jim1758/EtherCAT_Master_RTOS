#pragma once

#include <cstddef>
#include <cstdint>

// EDM18: producer-owned, Windows-read-only observation channel.
// Does not extend EDM_CNC_SHM or permit any hardware/recipe/voltage writes.
// The whole mapping is one seqlock: readers copy only an even, unchanged
// Sequence. Heartbeat progression is checked with the consumer's own clock;
// ObservedAtMs is the NC owner's clock, not a Windows UTC timestamp.
static constexpr std::uint32_t SHM_EDM_DIAG_MAGIC = 0x444D4445U;
static constexpr std::uint16_t SHM_EDM_DIAG_VERSION_MAJOR = 1U;
static constexpr std::uint16_t SHM_EDM_DIAG_VERSION_MINOR = 0U;
static constexpr std::uint32_t SHM_EDM_DIAG_FIELD_CAPACITY = 256U;
static constexpr std::uint32_t SHM_EDM_DIAG_FIELD_SIZE = 144U;
static constexpr std::uint32_t SHM_EDM_DIAG_SIZE = 37184U;

static constexpr std::uint32_t SHM_EDM_DIAG_PUBLISHED = 1U << 0U;
static constexpr std::uint32_t SHM_EDM_DIAG_CLOCK_VALID = 1U << 1U;

static constexpr std::uint8_t SHM_EDM_GAP_CONFIG_VALID = 1U << 0U;
static constexpr std::uint8_t SHM_EDM_GAP_LIVE_VALID = 1U << 1U;
static constexpr std::uint8_t SHM_EDM_GAP_RAW_AVAILABLE = 1U << 2U;
static constexpr std::uint8_t SHM_EDM_GAP_LAST_GOOD_AVAILABLE = 1U << 3U;
static constexpr std::uint8_t SHM_EDM_GAP_AGE_KNOWN = 1U << 4U;
static constexpr std::uint8_t SHM_EDM_GAP_CALIBRATION_CONFIRMED = 1U << 5U;
static constexpr std::uint8_t SHM_EDM_GAP_BOARD_VOLTAGE_VALID = 1U << 6U;

static constexpr std::uint8_t SHM_EDM_FIELD_CONFIGURED = 1U << 0U;
static constexpr std::uint8_t SHM_EDM_FIELD_OVERRIDE_PRESENT = 1U << 1U;
static constexpr std::uint8_t SHM_EDM_FIELD_MODIFIED = 1U << 2U;

#pragma pack(push, 1)
struct SHM_EDM_DiagHeader
{
    std::uint32_t Magic;
    std::uint16_t VersionMajor;
    std::uint16_t VersionMinor;
    std::uint32_t StructSize;
    std::uint32_t Sequence;
    std::uint32_t Heartbeat;
    std::uint32_t Flags;
    std::uint64_t PublishCount;
    std::uint64_t ObservedAtMs;
    std::uint32_t Reserved32[6];
};

struct SHM_EDM_DiagGap
{
    std::uint8_t ConfiguredSource; // EDMGap::Source: NONE=0, SIMULATED=1, PHYSICAL=2.
    std::uint8_t Source;           // EDMGap::Source.
    std::uint8_t Quality;          // EDMGap::Quality.
    std::uint8_t Band;             // EDMGap::Band.
    std::uint8_t PendingBand;
    std::uint8_t Flags;
    std::uint8_t Status;           // EDMGapInput::InputStatus.
    std::uint8_t Reserved8;
    std::uint32_t SelectedAd;      // One-based configured global AD index.
    std::uint32_t DeviceId;
    std::uint32_t ChannelId;
    std::uint32_t ProfileRevision;
    std::uint32_t CalibrationRevision;
    std::int32_t RawCode;          // 0 unless RawAvailable.
    std::int32_t VoltageMv;        // 0 unless LiveVoltageValid.
    std::int32_t LastGoodRawCode;
    std::int32_t LastGoodVoltageMv;
    std::int32_t RawMin;
    std::int32_t RawMax;
    std::int32_t MvAtMin;
    std::int32_t MvAtMax;
    std::uint32_t PdoOffset;
    std::uint64_t SampleSequence;
    std::uint64_t SampledAtMs;
    std::uint64_t ObservedAtMs;
    std::uint64_t AgeMs;
    std::uint64_t LastGoodSequence;
    std::uint64_t LastGoodAtMs;
    std::uint32_t MaxAgeMs;
    std::uint32_t DwellMs;
    std::int32_t BoardVoltageMv;   // AD terminal voltage, distinct from GAP.
    std::uint32_t Reserved32;
};

struct SHM_EDM_DiagRecipe
{
    std::uint8_t CatalogReady;
    std::uint8_t TableSelected;
    std::uint8_t RowSelected;
    std::uint8_t Mode;             // 0 SHADOW_ONLY; no discharge output.
    std::uint8_t LastError;        // EDMRecipe::Error.
    std::uint8_t Reserved8[3];
    std::uint32_t CatalogRevision;
    std::uint32_t TableId;
    std::uint32_t TableRevision;
    std::uint16_t ECode;
    std::uint16_t FieldCount;
    std::uint16_t ConfiguredCount;
    std::uint16_t ModifiedCount;
    std::uint32_t Reserved32;
    std::uint64_t Generation;
    char TableName[64];            // UTF-8, NUL terminated.
    std::uint32_t Schema;
    std::uint16_t FieldCapacity;
    std::uint16_t FieldRecordSize;
    // Schema 2: [0] machine profile ID, [1] definition revision; [2..3] reserved.
    std::uint32_t ReservedTail32[4];
};

struct SHM_EDM_DiagField
{
    std::uint16_t FieldId;
    std::uint8_t Decimals;
    std::uint8_t RowKind;          // Unset=0, Direct=1, Stage=2.
    std::uint8_t Flags;
    std::uint8_t Reserved8;
    std::uint16_t StageId;
    std::uint16_t RowStageId;
    std::uint16_t Reserved16;
    // Exact signed fixed-point millionths, never float or double.
    std::int64_t RowBaseValue;
    std::int64_t BaseValue;
    std::int64_t ActualValue;
    std::int64_t Minimum;
    std::int64_t Maximum;
    char Name[64];                 // UTF-8, NUL terminated.
    char Unit[24];
    std::uint32_t Reserved32;
};

struct SHM_EDM_DiagData
{
    SHM_EDM_DiagHeader Header;
    SHM_EDM_DiagGap Gap;
    SHM_EDM_DiagRecipe Recipe;
    SHM_EDM_DiagField Fields[SHM_EDM_DIAG_FIELD_CAPACITY];
};
#pragma pack(pop)

static_assert(sizeof(SHM_EDM_DiagHeader) == 64U, "EDM diagnostic header ABI");
static_assert(offsetof(SHM_EDM_DiagHeader, Sequence) == 12U, "EDM sequence ABI");
static_assert(offsetof(SHM_EDM_DiagHeader, Heartbeat) == 16U, "EDM heartbeat ABI");
static_assert(sizeof(SHM_EDM_DiagGap) == 128U, "EDM GAP ABI");
static_assert(offsetof(SHM_EDM_DiagGap, BoardVoltageMv) == 120U, "EDM board voltage ABI");
static_assert(sizeof(SHM_EDM_DiagRecipe) == 128U, "EDM recipe ABI");
static_assert(sizeof(SHM_EDM_DiagField) == 144U, "EDM field ABI");
static_assert(offsetof(SHM_EDM_DiagData, Gap) == 64U, "EDM GAP offset ABI");
static_assert(offsetof(SHM_EDM_DiagData, Recipe) == 192U, "EDM recipe offset ABI");
static_assert(offsetof(SHM_EDM_DiagData, Fields) == 320U, "EDM field offset ABI");
static_assert(sizeof(SHM_EDM_DiagData) == SHM_EDM_DIAG_SIZE, "EDM mapping ABI");
