#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 17A.3C.1 - OSCARMAX EtherCAT DC MAD Qualification Diagnostic Contract
// ============================================================================
//
// Shared Memory:
//     OSCARMAX_ECAT_DC_QUAL_DIAG
//
// Producer:
//     Existing Priority-50 supervisory loop (~10 ms).
//
// Consumer:
//     Windows ENI Tool read-only (Stage 17A.3C.2).
//
// Purpose:
//     Preserve MAD qualification statistics and recent near-threshold /
//     abnormal windows so we can distinguish:
//
//       A. repeated 151..160 ppb threshold chatter
//       B. isolated large MAD spikes
//
// Design:
//     - Existing OSCARMAX_ECAT_DIAG ABI remains unchanged.
//     - No EtherCAT transaction is generated.
//     - No SDO / PDO write.
//     - No DC tuning / state-machine change.
//     - Priority-64 never writes this mapping.
//
// Source fingerprint:
//     OSCARMAX_DC_QUAL_DIAG_CONTRACT_17A3C1_20260828
// ============================================================================

static constexpr uint32_t SHM_ECAT_DC_QUAL_DIAG_MAGIC =
0x51444345u; // little-endian "ECDQ"

static constexpr uint16_t SHM_ECAT_DC_QUAL_DIAG_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_DC_QUAL_DIAG_VERSION_MINOR = 0u;

static constexpr uint32_t SHM_ECAT_DC_QUAL_DIAG_CAPACITY = 512u;

static constexpr int32_t SHM_ECAT_DC_QUAL_DIAG_NEAR_MAD_PPB = 120;
static constexpr int32_t SHM_ECAT_DC_QUAL_DIAG_BAD_MAD_PPB = 150;


// 64 bytes.
struct SHM_ECAT_DcQualDiagHeader
{
    uint32_t Magic;
    uint16_t VersionMajor;
    uint16_t VersionMinor;
    uint32_t StructSize;

    uint32_t Sequence;
    uint32_t Heartbeat;
    uint64_t PublishCount;

    uint32_t WriteIndex;
    uint32_t EntryCount;
    uint32_t Capacity;

    uint32_t Reserved32[6];
};


// 96 bytes.
struct SHM_ECAT_DcQualDiagSummary
{
    uint8_t Armed;
    uint8_t CurrentV2State;
    uint8_t CurrentCandidateGood;
    uint8_t CurrentRejectMask;

    uint8_t CurrentObserverLocked;
    uint8_t CurrentPointAccepted;
    uint8_t CurrentTripMask;
    uint8_t Reserved8;

    int32_t CurrentSlopeMadPpb;
    int32_t CurrentRecommendedPpb;
    int32_t CurrentBaselinePpb;
    int32_t CurrentRecommendedDeltaPpb;

    uint32_t PeakMadPpb;
    uint32_t FirstBadMadPpb;
    uint32_t WorstBadMadPpb;

    uint32_t NearThresholdSamples;
    uint32_t AboveThresholdSamples;

    uint32_t CurrentBadStreak;
    uint32_t LongestBadStreak;

    uint32_t CandidateBadTransitions;

    uint32_t TrackToHoldCount;
    uint32_t HoldToTrackCount;
    uint32_t HoldToFallbackCount;

    uint32_t RejectMaskOr;

    uint32_t Reserved32[6];
};


// 32 bytes.
struct SHM_ECAT_DcQualDiagEntry
{
    uint64_t CaptureTickMs;
    uint32_t SampleOrdinal;

    int32_t SlopeMadPpb;

    int16_t RecommendedPpb;
    int16_t RecommendedDeltaPpb;

    int32_t PhaseErrorNs;

    uint8_t V2State;

    // bit0 CandidateGood
    // bit1 ObserverLocked
    // bit2 PointAccepted
    // bit3 BaselineLocked
    uint8_t Flags;

    uint8_t RejectMask;
    uint8_t TripMask;

    uint8_t RealFfState;
    uint8_t PhasePState;

    // bit0 MAD >= 120
    // bit1 MAD > 150
    // bit2 qualification / DC-chain abnormal
    uint8_t EventFlags;

    uint8_t Reserved8;
};


struct SHM_ECAT_DcQualDiagData
{
    SHM_ECAT_DcQualDiagHeader Header;
    SHM_ECAT_DcQualDiagSummary Summary;

    SHM_ECAT_DcQualDiagEntry
        Entries[SHM_ECAT_DC_QUAL_DIAG_CAPACITY];
};


#if !defined(__INTELLISENSE__)

static_assert(
    sizeof(SHM_ECAT_DcQualDiagHeader) == 64,
    "SHM_ECAT_DcQualDiagHeader ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcQualDiagSummary) == 96,
    "SHM_ECAT_DcQualDiagSummary ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcQualDiagEntry) == 32,
    "SHM_ECAT_DcQualDiagEntry ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcQualDiagData) == 16544,
    "SHM_ECAT_DcQualDiagData ABI mismatch.");

#endif

#pragma pack(pop)
