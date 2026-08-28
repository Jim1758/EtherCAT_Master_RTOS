#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 17A.5.1 - OSCARMAX DC V1A Observer Forensics Contract
// ============================================================================
//
// Shared Memory:
//     OSCARMAX_ECAT_DC_V1A_DIAG
//
// Producer:
//     Existing Priority-50 supervisory loop (~10 ms).
//
// Source:
//     Existing coherent g_qpcDcPhaseResidualV1A* diagnostic snapshot.
//
// Consumer:
//     Windows ENI Tool read-only (Stage 17A.5.2).
//
// Every NEW V1A sequence is recorded (~1 second/window).
//
// IMPORTANT:
//     - V1A algorithm unchanged.
//     - V2 150 ppb threshold unchanged.
//     - DC state machine unchanged.
//     - No EtherCAT frame / SDO / PDO write.
//     - Priority-64 never writes Shared Memory.
//
// Source fingerprint:
//     OSCARMAX_DC_V1A_DIAG_CONTRACT_17A5_20260828
// ============================================================================

static constexpr uint32_t SHM_ECAT_DC_V1A_DIAG_MAGIC =
0x31415645u;

static constexpr uint16_t SHM_ECAT_DC_V1A_DIAG_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_DC_V1A_DIAG_VERSION_MINOR = 0u;

static constexpr uint32_t SHM_ECAT_DC_V1A_DIAG_CAPACITY = 512u;


// 64 bytes.
struct SHM_ECAT_DcV1aDiagHeader
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


// 128 bytes.
struct SHM_ECAT_DcV1aDiagSummary
{
    uint8_t Initialized;
    uint8_t PointAccepted;
    uint8_t Locked;
    uint8_t CurrentV2State;

    uint8_t CurrentCandidateGood;
    uint8_t CurrentRejectMask;
    uint8_t Reserved8[2];

    int64_t MeanPhaseNs;
    int64_t PhaseSpanNs;
    int64_t WindowElapsedNs;
    int64_t WindowRttMaxNs;
    int64_t TimeSpanNs;

    int32_t TheilSenPpb;
    int32_t SlopeMadPpb;
    int32_t SlopeMinPpb;
    int32_t SlopeMaxPpb;

    int32_t RecommendedPpb;
    int32_t TrustedDriftPpb;
    int32_t TrustedMinusRecommendedPpb;

    uint32_t MeanSamples;
    uint32_t PointBufferCount;
    uint32_t PairSlopeCount;

    uint32_t AcceptedPointTotal;
    uint32_t RejectedPointTotal;

    uint32_t RejectSamplesTotal;
    uint32_t RejectElapsedTotal;
    uint32_t RejectRttTotal;
    uint32_t RejectSpanTotal;

    uint32_t PeakMadPpb;
    uint32_t PeakPhaseSpanNs;
    uint32_t PeakRttNs;
    uint32_t PeakSlopeSpreadPpb;
};


// 112 bytes.
struct SHM_ECAT_DcV1aDiagEntry
{
    uint64_t CapturePublishCount;

    uint32_t V1aSequence;

    // bit0      Initialized
    // bit1      PointAccepted
    // bit2      Locked
    // bit3      V2 CandidateGood
    // bits4..5  V2 State
    // bits8..15 Current RealFF PhaseRejectMask
    uint32_t Flags;

    int64_t MeanPhaseNs;
    int64_t PhaseSpanNs;
    int64_t WindowElapsedNs;
    int64_t WindowRttMaxNs;
    int64_t TimeSpanNs;

    int32_t TheilSenPpb;
    int32_t SlopeMadPpb;
    int32_t SlopeMinPpb;
    int32_t SlopeMaxPpb;

    int32_t RecommendedPpb;
    int32_t TrustedDriftPpb;
    int32_t TrustedMinusRecommendedPpb;

    uint16_t MeanSamples;
    uint8_t PointBufferCount;
    uint8_t PairSlopeCount;

    uint32_t AcceptedPointTotal;
    uint32_t RejectedPointTotal;

    uint32_t RejectSamplesTotal;
    uint32_t RejectElapsedTotal;
    uint32_t RejectRttTotal;
    uint32_t RejectSpanTotal;
};


struct SHM_ECAT_DcV1aDiagData
{
    SHM_ECAT_DcV1aDiagHeader Header;
    SHM_ECAT_DcV1aDiagSummary Summary;

    SHM_ECAT_DcV1aDiagEntry
        Entries[SHM_ECAT_DC_V1A_DIAG_CAPACITY];
};


#if !defined(__INTELLISENSE__)

static_assert(
    sizeof(SHM_ECAT_DcV1aDiagHeader) == 64,
    "SHM_ECAT_DcV1aDiagHeader ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcV1aDiagSummary) == 128,
    "SHM_ECAT_DcV1aDiagSummary ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcV1aDiagEntry) == 112,
    "SHM_ECAT_DcV1aDiagEntry ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_DcV1aDiagData) == 57536,
    "SHM_ECAT_DcV1aDiagData ABI mismatch.");

#endif

#pragma pack(pop)
