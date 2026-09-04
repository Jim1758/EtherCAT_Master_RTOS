#pragma once

#include <windows.h>
#include <stdint.h>

/*
 * EtherCatRxForensics.h
 * DC-RX.4A - RX incident forensics and per-port ESC error counters.
 *
 * This contract is internal to the RTOS executable. It does not change the
 * existing SHM layout or any EtherCAT control/safety threshold.
 */

constexpr uint32_t ETHERCAT_RX4A_INCIDENT_RING_CAPACITY = 128u;
constexpr uint32_t ETHERCAT_RX4A_ESC_MAX_SLAVES = 128u;
constexpr uint32_t ETHERCAT_RX4A_ESC_PORT_COUNT = 4u;

enum class NicRxCallOutcome : uint32_t
{
    QueueUnavailable = 0u,
    NoData = 1u,
    Frame = 2u,
    InvalidLength = 3u,
    NalError = 4u
};

struct NicRxCallDiagnostic
{
    uint64_t CallSequence = 0ULL;
    uint32_t Outcome = static_cast<uint32_t>(NicRxCallOutcome::QueueUnavailable);
    uint32_t NalCallSucceeded = 0u;
    uint32_t LastError = ERROR_SUCCESS;
    uint32_t Length = 0u;
};

enum class EtherCatRxIncidentKind : uint32_t
{
    HardTimeout = 1u,
    Recovery = 2u,
    QueueStopped = 3u
};

enum class EtherCatRxIncidentReason : uint32_t
{
    None = 0u,
    QpcFailure = 1u,
    PreReceiveDeadline = 2u,
    PostReceiveLate = 3u,
    PreWaitDeadline = 4u,
    PreFallbackDeadline = 5u,
    FallbackExhausted = 6u,
    EventStopped = 7u
};

enum class EtherCatRxIncidentClassification : uint32_t
{
    Unknown = 0u,
    NoFrameByDeadline = 1u,
    NalCallStall = 2u,
    LateFrameReturned = 3u,
    LateFrameQueued = 4u,
    NoLateFrameQueued = 5u,
    QpcFailure = 6u,
    QueueStopped = 7u,
    Recovered = 8u
};

struct EtherCatRxIncidentRecord
{
    uint64_t IncidentId = 0ULL;
    uint64_t RelatedIncidentId = 0ULL;
    uint64_t BurstId = 0ULL;
    uint64_t PdoTick = 0ULL;
    uint64_t Qpc = 0ULL;
    uint64_t QpcFrequency = 0ULL;
    uint64_t TotalHardTimeout = 0ULL;
    uint64_t RxElapsedNs = 0ULL;
    uint64_t ReceiveCallNs = 0ULL;
    uint64_t ReceiveCallMaxNs = 0ULL;
    uint64_t SchedulerRecoveryTotal = 0ULL;
    uint64_t EscPortChangeSerial = 0ULL;
    uint64_t NalCallSequence = 0ULL;

    int64_t PhaseErrorNs = 0LL;
    int64_t AppliedPpb = 0LL;
    int64_t RecommendedPpb = 0LL;

    uint32_t Kind = 0u;
    uint32_t Reason = 0u;
    uint32_t Classification = 0u;
    uint32_t ConsecutiveTimeout = 0u;
    uint32_t AttemptCount = 0u;
    uint32_t CoarseWaitCount = 0u;
    uint32_t EventSignaledCount = 0u;
    uint32_t EventTimeoutCount = 0u;
    uint32_t EventFailedCount = 0u;
    uint32_t EventFallbackSleepCount = 0u;
    uint32_t LastEventResult = 0u;
    uint32_t LastEventError = ERROR_SUCCESS;
    uint32_t NalOutcome = static_cast<uint32_t>(NicRxCallOutcome::QueueUnavailable);
    uint32_t NalCallSucceeded = 0u;
    uint32_t NalError = ERROR_SUCCESS;
    uint32_t NalLength = 0u;

    int32_t RxLength = 0;
    int32_t LrwWkc = -1;
    int32_t DcWkc = 0;
    int32_t ExpectedLrwWkc = 0;

    uint32_t ResyncAttempted = 0u;
    uint32_t ResyncDrainCount = 0u;
    uint32_t ResyncDrainLimitHit = 0u;
    uint32_t HoldMask = 0u;
    uint32_t TripMask = 0u;
    uint32_t ProcessDataValid = 0u;
    uint32_t DcTransportValid = 0u;
};

struct EtherCatEscPortErrorSnapshot
{
    uint32_t Valid = 0u;
    uint32_t SlavePosition = 0u;
    uint32_t ConfiguredAddress = 0u;
    uint32_t BaselineValid = 0u;

    uint64_t SampleTick = 0ULL;
    uint64_t InitialBaselineTick = 0ULL;
    uint64_t RxBaselineTick = 0ULL;
    uint64_t EcatProcessingUnitBaselineTick = 0ULL;
    uint64_t PdiBaselineTick = 0ULL;
    uint64_t LostLinkBaselineTick = 0ULL;
    uint64_t LastChangeTick = 0ULL;
    uint64_t SampleCount = 0ULL;
    uint64_t RxGroupResetCount = 0ULL;
    uint64_t EcatProcessingUnitResetCount = 0ULL;
    uint64_t PdiResetCount = 0ULL;
    uint64_t LostLinkGroupResetCount = 0ULL;
    uint64_t ChangeSerial = 0ULL;

    uint8_t InvalidFrame[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t PhysicalRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t ForwardedRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t LostLink[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t EcatProcessingUnitError = 0u;
    uint8_t PdiError = 0u;
    uint8_t Reserved0[2] = {};

    uint8_t BaselineInvalidFrame[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t BaselinePhysicalRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t BaselineForwardedRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t BaselineLostLink[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint8_t BaselineEcatProcessingUnitError = 0u;
    uint8_t BaselinePdiError = 0u;
    uint8_t Reserved1[2] = {};

    uint32_t DeltaInvalidFrame[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint32_t DeltaPhysicalRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint32_t DeltaForwardedRxError[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint32_t DeltaLostLink[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
    uint32_t DeltaEcatProcessingUnitError = 0u;
    uint32_t DeltaPdiError = 0u;

    uint32_t SaturatedInvalidFrameMask = 0u;
    uint32_t SaturatedPhysicalRxErrorMask = 0u;
    uint32_t SaturatedForwardedRxErrorMask = 0u;
    uint32_t SaturatedLostLinkMask = 0u;
    uint32_t SaturatedMiscMask = 0u; // bit 0 = ECAT PU, bit 1 = PDI.
};

uint64_t EtherCatRx4aIncidentLatestId();
bool EtherCatRx4aIncidentRead(
    uint64_t incidentId,
    EtherCatRxIncidentRecord* record);

uint32_t EtherCatRx4aEscPortSlaveCount();
bool EtherCatRx4aEscPortRead(
    uint16_t slavePosition,
    EtherCatEscPortErrorSnapshot* snapshot);

// These fields are published inside the existing g_ecatRxDiagSequence seqlock.
extern volatile LONG g_ecatRx4aNalFrame;
extern volatile LONG g_ecatRx4aNalNoData;
extern volatile LONG g_ecatRx4aNalApiError;
extern volatile LONG g_ecatRx4aNalInvalidLength;
extern volatile LONG g_ecatRx4aNalUnavailable;
extern volatile LONG g_ecatRx4aNalLastError;
extern volatile LONG g_ecatRx4aCallOver50us;
extern volatile LONG g_ecatRx4aCallOver100us;
extern volatile LONG g_ecatRx4aCallOver150us;
extern volatile LONG g_ecatRx4aCallOver200us;
extern volatile LONG g_ecatRx4aTimeoutNoFrame;
extern volatile LONG g_ecatRx4aTimeoutNalStall;
extern volatile LONG g_ecatRx4aTimeoutLateFrame;
extern volatile LONG g_ecatRx4aResyncEvents;
extern volatile LONG g_ecatRx4aResyncFrames;
extern volatile LONG g_ecatRx4aResyncLimitHit;

extern volatile LONGLONG g_ecatRx4aEscPortChangeSerial;
