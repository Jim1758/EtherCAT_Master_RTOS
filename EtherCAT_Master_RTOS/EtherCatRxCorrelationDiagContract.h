#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 17A.8.1 - RX Deadline / V1A Correlation Diagnostic Contract
// ============================================================================
//
// Shared Memory:
//     OSCARMAX_ECAT_RX_CORR_DIAG
//
// Producer:
//     Existing Priority-50 supervisory loop (~10 ms).
//
// One ring entry is written ONLY when g_ecatRxDiagSequence changes.
// RX diagnostics publish every 4000 RX calls (~1 second at 250 us).
//
// Correlates:
//     RX deadline diagnostics
//     nearest coherent V1A observer snapshot
//     PDO RT execution diagnostics
//     DC state diagnostics
//
// IMPORTANT:
//     - RX Soft/Hard deadlines unchanged.
//     - V1A algorithm unchanged.
//     - MAD threshold unchanged.
//     - DC state machine unchanged.
//     - No EtherCAT / SDO / PDO write.
//     - Priority-64 never writes Shared Memory.
//
// Source fingerprint:
//     OSCARMAX_RX_V1A_CORR_CONTRACT_17A8_20260828
// ============================================================================

static constexpr uint32_t SHM_ECAT_RX_CORR_DIAG_MAGIC =
0x43525845u;

static constexpr uint16_t SHM_ECAT_RX_CORR_DIAG_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_RX_CORR_DIAG_VERSION_MINOR = 0u;

static constexpr uint32_t SHM_ECAT_RX_CORR_DIAG_CAPACITY = 512u;


// 64 bytes.
struct SHM_ECAT_RxCorrDiagHeader
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


// Exactly 256 bytes.
//
// Flags:
//   bit0  RX snapshot coherent
//   bit1  V1A snapshot coherent
//   bit2  HardTimeout > 0
//   bit3  PostReceiveLate > 0
//   bit4  TimeoutPreReceive > 0
//   bit5  V1A CandidateGood
//   bit6  DC Gate Open
//   bit7  V1A Point Accepted
//
struct SHM_ECAT_RxCorrDiagEntry
{
    uint64_t CapturePublishCount;

    uint32_t RxSequence;
    uint32_t V1aSequence;
    uint32_t Flags;


    // RX 1-second snapshot.
    uint32_t Calls;
    uint32_t FirstRxSuccess;
    uint32_t EmptyRx;
    uint32_t InvalidFrame;

    uint32_t SoftLateAccepted;
    uint32_t HardTimeout;
    uint32_t PostReceiveLate;

    uint32_t CurrentConsecutiveTimeout;
    uint32_t MaxConsecutiveTimeout;
    uint32_t RecoveryAfterTimeout;

    uint32_t QpcFail;
    uint32_t SleepCount;
    uint32_t ElapsedValid;

    uint32_t TimeoutPreReceive;
    uint32_t TimeoutSleep0;
    uint32_t TimeoutSleep1;
    uint32_t TimeoutSleep2;

    uint32_t TimeoutAttemptAvg;
    uint32_t TimeoutAttemptMax;

    uint32_t SoftDeadlineNs;
    uint32_t HardDeadlineNs;


    uint64_t TotalSoftLateAccepted;
    uint64_t SoftLateElapsedMaxNs;

    uint64_t TotalHardTimeout;

    uint64_t ElapsedAvgNs;
    uint64_t ElapsedMaxNs;

    uint64_t ReceiveCallMaxNs;
    uint64_t TimeoutReceiveCallMaxNs;


    // Nearest V1A snapshot.
    int64_t V1aMeanPhaseNs;
    int64_t V1aPhaseSpanNs;
    int64_t V1aWindowRttMaxNs;

    int32_t V1aSlopeMadPpb;
    int32_t V1aTheilSenPpb;
    int32_t V1aSlopeMinPpb;
    int32_t V1aSlopeMaxPpb;


    // PDO / DC runtime correlation.
    int32_t PdoExecAvgNs;
    int32_t PdoExecMaxNs;
    int32_t PdoCombinedMaxNs;
    int32_t DcPhaseErrorNs;

    int32_t LrwWkc;
    int32_t DcWkc;

    uint32_t PdoExecOver250Count;
    uint32_t PdoExecOver300Count;
    uint32_t PdoExecOver400Count;


    // State correlation.
    uint8_t V1aPointAccepted;
    uint8_t V1aLocked;

    uint8_t V2State;
    uint8_t V2CandidateGood;

    uint8_t RealFfRejectMask;

    uint8_t DcGateOpen;
    uint8_t RealFfState;
    uint8_t PhasePState;
    uint8_t TripMask;


    uint8_t Reserved8[11];
};


struct SHM_ECAT_RxCorrDiagData
{
    SHM_ECAT_RxCorrDiagHeader Header;

    SHM_ECAT_RxCorrDiagEntry
        Entries[SHM_ECAT_RX_CORR_DIAG_CAPACITY];
};


#if !defined(__INTELLISENSE__)

static_assert(
    sizeof(SHM_ECAT_RxCorrDiagHeader) == 64,
    "SHM_ECAT_RxCorrDiagHeader ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_RxCorrDiagEntry) == 256,
    "SHM_ECAT_RxCorrDiagEntry ABI mismatch.");

static_assert(
    sizeof(SHM_ECAT_RxCorrDiagData) == 131136,
    "SHM_ECAT_RxCorrDiagData ABI mismatch.");

#endif

#pragma pack(pop)
