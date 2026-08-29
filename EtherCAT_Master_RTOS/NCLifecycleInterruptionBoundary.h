#pragma once

#include "NCBlockLifecycleLedger.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2J.1 - Lifecycle Failure / Epoch Cancellation Shadow Boundary
//
// This observer records interruption evidence only.  It does not stop Motion,
// advance the Program Counter, close Ledger entries, raise Alarm, change Owner,
// publish a new Epoch, clear a queue, or alter the existing Reset / Alarm path.
//
// The shadow boundary answers three questions before NC-0.2J cutover:
//
//   1. Which event interrupted the execution?
//   2. Was the expected Epoch change published and were terminal feedback
//      events accepted by the Block Lifecycle Ledger?
//   3. Did Lifecycle, transport, callback and physical motion reach a proven
//      quiescent boundary without an evidence gap?
//
// All state is fixed-size and trivially copyable.  Observe() is bounded and
// allocation-free so NCManager may call it from the existing 10 ms task.
// =============================================================================

enum class NCLifecycleInterruptionCause : std::uint8_t
{
    NONE = 0,
    RESET = 1,
    ALARM = 2,
    PROGRAM_REPLACED = 3,
    MDI_REPLACED = 4,
    MANUAL_AUTO_REPLACED = 5,
    DYNAMIC_CODE_REPLACED = 6,
    GOTO_EPOCH = 7,
    MOTION_REJECTED = 8,
    MOTION_CANCELLED = 9,
    MOTION_ABORTED = 10,
    MOTION_FAULTED = 11
};

enum class NCLifecycleInterruptionPhase : std::uint8_t
{
    IDLE = 0,
    REQUESTED = 1,
    EPOCH_PUBLISHED = 2,
    DRAINING = 3,
    STABLE_CONFIRMATION = 4,
    QUIESCENT = 5,
    EVIDENCE_GAP = 6,
    SUPERSEDED = 7,
    ALARM_STOP_CLOSED = 8
};

enum class NCLifecycleInterruptionDecision : std::uint8_t
{
    NONE = 0,
    REQUEST_LATCHED = 1,
    WAIT_EPOCH_PUBLICATION = 2,
    EPOCH_PUBLICATION_OBSERVED = 3,
    WAIT_ACTIVE_BLOCKS = 4,
    WAIT_AXIS_COMMAND = 5,
    WAIT_AXIS_RESULT = 6,
    WAIT_COMMAND_INGRESS = 7,
    WAIT_COMMAND_REPLAY = 8,
    WAIT_COMMAND_QUEUE = 9,
    WAIT_FEEDBACK_NOTICE = 10,
    WAIT_FEEDBACK = 11,
    WAIT_FEEDBACK_SEQUENCE = 12,
    WAIT_CALLBACK = 13,
    WAIT_COMPLETION_BINDING = 14,
    WAIT_SAFETY_REQUEST = 15,
    WAIT_GROUP_STANDSTILL = 16,
    WAIT_STABLE_CONFIRMATION = 17,
    QUIESCENT_PROVED = 18,
    EVIDENCE_FEEDBACK_OVERFLOW = 19,
    EVIDENCE_NOTICE_OVERFLOW = 20,
    EVIDENCE_SEQUENCE_GAP = 21,
    EVIDENCE_LEDGER_INTEGRITY = 22,
    EVIDENCE_LEDGER_REJECTED = 23,
    EPOCH_SUPERSEDED = 24,
    SUPERSEDED = 25,
    WAIT_ALARM_STOP_ACKNOWLEDGEMENT = 26,
    WAIT_ALARM_STOP_TERMINAL = 27,
    WAIT_ALARM_STOP_STABLE = 28,
    ALARM_STOP_CLOSED = 29
};

struct NCLifecycleInterruptionSample
{
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease ownerLease{};
    MotionOwnerLease executionOwnerLease{};

    NCBlockDispatchId lastDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    int activePC = -1;
    std::uint32_t activeBlocks = 0U;

    std::size_t axisCommandDepth = 0U;
    std::size_t axisResultDepth = 0U;
    std::size_t commandQueueDepth = 0U;
    std::size_t commandIngressDepth = 0U;
    std::size_t commandReplayDepth = 0U;
    std::size_t feedbackDepth = 0U;
    std::size_t feedbackNoticeDepth = 0U;

    MotionFeedbackSequence lastPublishedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackSequence lastConsumedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;

    std::uint64_t blockFailed = 0ULL;
    std::uint64_t blocksDispatched = 0ULL;
    std::uint64_t feedbackRejected = 0ULL;
    std::uint64_t feedbackCancelled = 0ULL;
    std::uint64_t feedbackAborted = 0ULL;
    std::uint64_t feedbackFaulted = 0ULL;

    std::uint64_t motionCaptureOverflow = 0ULL;
    std::uint64_t orphanFeedback = 0ULL;
    std::uint64_t duplicateTerminalFeedback = 0ULL;
    std::uint64_t terminalFeedbackConflict = 0ULL;
    std::uint64_t activeBlockOverwrite = 0ULL;
    std::uint64_t activeSegmentIndexOverwrite = 0ULL;

    std::uint64_t feedbackOverflow = 0ULL;
    std::uint64_t feedbackNoticeOverflow = 0ULL;
    std::uint64_t feedbackSequenceGap = 0ULL;

    bool safetyOrRecoveryPending = false;
    bool waitCallbackActive = false;
    bool completionBindingActive = false;
    bool groupStandstill = false;
};

struct NCLifecycleInterruptionSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCLifecycleInterruptionCause cause =
        NCLifecycleInterruptionCause::NONE;
    NCLifecycleInterruptionPhase phase =
        NCLifecycleInterruptionPhase::IDLE;
    NCLifecycleInterruptionDecision decision =
        NCLifecycleInterruptionDecision::NONE;

    MotionExecutionEpoch requestExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch publishedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch currentExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;

    MotionOwner requestOwner = MotionOwner::NONE;
    MotionOwnerGeneration requestOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner requestExecutionOwner = MotionOwner::NONE;
    MotionOwnerGeneration requestExecutionOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    NCBlockDispatchId requestDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    int requestPC = -1;
    std::uint32_t requestActiveBlocks = 0U;
    std::uint32_t activeBlocks = 0U;

    std::size_t axisCommandDepth = 0U;
    std::size_t axisResultDepth = 0U;
    std::size_t commandQueueDepth = 0U;
    std::size_t commandIngressDepth = 0U;
    std::size_t commandReplayDepth = 0U;
    std::size_t feedbackDepth = 0U;
    std::size_t feedbackNoticeDepth = 0U;

    MotionFeedbackSequence lastPublishedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackSequence lastConsumedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;

    MotionFeedbackSequence lastTerminalFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionExecutionIdentity lastTerminalIdentity{};
    MotionFeedbackType lastTerminalFeedbackType = MotionFeedbackType::NONE;
    MotionRejectReason lastTerminalRejectReason = MotionRejectReason::NONE;
    std::uint32_t lastTerminalErrorCode = 0U;

    std::uint64_t blockFailureDelta = 0ULL;
    std::uint64_t expectedAlarmAbortDelta = 0ULL;
    std::uint64_t expectedAlarmPreReadRejectDelta = 0ULL;
    std::uint64_t expectedAlarmOwnerConflictRejectDelta = 0ULL;
    std::uint64_t expectedAlarmStaleEpochRejectDelta = 0ULL;
    std::uint64_t unexpectedBlockFailureDelta = 0ULL;
    std::uint64_t unexpectedFeedbackRejectedDelta = 0ULL;
    std::uint64_t dispatchDelta = 0ULL;
    std::uint64_t feedbackRejectedDelta = 0ULL;
    std::uint64_t feedbackCancelledDelta = 0ULL;
    std::uint64_t feedbackAbortedDelta = 0ULL;
    std::uint64_t feedbackFaultedDelta = 0ULL;
    std::uint64_t ledgerIntegrityDelta = 0ULL;
    std::uint64_t feedbackOverflowDelta = 0ULL;
    std::uint64_t feedbackNoticeOverflowDelta = 0ULL;
    std::uint64_t feedbackSequenceGapDelta = 0ULL;

    std::uint32_t stableSamples = 0U;
    std::uint32_t requiredStableSamples = 2U;

    bool active = false;
    bool expectsEpochChange = false;
    bool epochPublicationObserved = false;
    bool alarmEpochClassificationPending = false;
    bool alarmStopAcknowledged = false;
    bool runtimeAlarmEpochChangeObserved = false;
    bool expectedAlarmAbortObserved = false;
    bool expectedAlarmPreReadRejectObserved = false;
    bool alarmTerminalClassificationValid = true;
    bool alarmStopClosed = false;
    bool unexpectedEpochChangeObserved = false;
    bool postInterruptionDispatchObserved = false;
    bool terminalFailureObserved = false;
    bool terminalFeedbackLedgerAccepted = true;
    bool evidenceGap = false;
    bool safetyOrRecoveryPending = false;
    bool waitCallbackActive = false;
    bool completionBindingActive = false;
    bool groupStandstill = false;
    bool feedbackSequenceSynchronized = false;
    bool quiescentReady = false;
    bool quiescent = false;
    bool superseded = false;
};

struct NCLifecycleInterruptionCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t requestsLatched = 0ULL;
    std::uint64_t resetRequests = 0ULL;
    std::uint64_t alarmRequests = 0ULL;
    std::uint64_t programReplaceRequests = 0ULL;
    std::uint64_t mdiReplaceRequests = 0ULL;
    std::uint64_t manualAutoReplaceRequests = 0ULL;
    std::uint64_t dynamicCodeReplaceRequests = 0ULL;
    std::uint64_t gotoEpochRequests = 0ULL;
    std::uint64_t terminalTriggeredRequests = 0ULL;

    std::uint64_t epochPublicationsExpected = 0ULL;
    std::uint64_t epochPublicationsObserved = 0ULL;
    std::uint64_t unexpectedEpochChanges = 0ULL;
    std::uint64_t epochSuperseded = 0ULL;
    std::uint64_t alarmStopAcknowledgements = 0ULL;
    std::uint64_t runtimeAlarmEpochChanges = 0ULL;
    std::uint64_t expectedAlarmAborts = 0ULL;
    std::uint64_t expectedAlarmPreReadRejects = 0ULL;
    std::uint64_t expectedAlarmOwnerConflictRejects = 0ULL;
    std::uint64_t expectedAlarmStaleEpochRejects = 0ULL;
    std::uint64_t alarmStopsClosed = 0ULL;

    std::uint64_t terminalRejected = 0ULL;
    std::uint64_t terminalCancelled = 0ULL;
    std::uint64_t terminalAborted = 0ULL;
    std::uint64_t terminalFaulted = 0ULL;
    std::uint64_t terminalLedgerRejected = 0ULL;
    std::uint64_t postInterruptionDispatch = 0ULL;

    std::uint64_t evaluations = 0ULL;
    std::uint64_t waitEpochPublication = 0ULL;
    std::uint64_t waitActiveBlocks = 0ULL;
    std::uint64_t waitAxisCommand = 0ULL;
    std::uint64_t waitAxisResult = 0ULL;
    std::uint64_t waitCommandIngress = 0ULL;
    std::uint64_t waitCommandReplay = 0ULL;
    std::uint64_t waitCommandQueue = 0ULL;
    std::uint64_t waitFeedbackNotice = 0ULL;
    std::uint64_t waitFeedback = 0ULL;
    std::uint64_t waitFeedbackSequence = 0ULL;
    std::uint64_t waitCallback = 0ULL;
    std::uint64_t waitCompletionBinding = 0ULL;
    std::uint64_t waitSafetyRequest = 0ULL;
    std::uint64_t waitGroupStandstill = 0ULL;
    std::uint64_t waitStableConfirmation = 0ULL;
    std::uint64_t waitAlarmStopAcknowledgement = 0ULL;
    std::uint64_t waitAlarmStopTerminal = 0ULL;
    std::uint64_t waitAlarmStopStable = 0ULL;

    std::uint64_t quiescent = 0ULL;
    std::uint64_t evidenceGap = 0ULL;
    std::uint64_t feedbackOverflowEvidenceGap = 0ULL;
    std::uint64_t feedbackNoticeOverflowEvidenceGap = 0ULL;
    std::uint64_t feedbackSequenceEvidenceGap = 0ULL;
    std::uint64_t ledgerIntegrityEvidenceGap = 0ULL;
    std::uint64_t ledgerRejectedEvidenceGap = 0ULL;
    std::uint64_t superseded = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCLifecycleInterruptionSample>::value,
    "NCLifecycleInterruptionSample must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCLifecycleInterruptionSnapshot>::value,
    "NCLifecycleInterruptionSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCLifecycleInterruptionCounters>::value,
    "NCLifecycleInterruptionCounters must remain trivially copyable.");

class NCLifecycleInterruptionBoundaryShadow
{
public:
    NCLifecycleInterruptionBoundaryShadow() noexcept = default;

    void Begin(
        NCLifecycleInterruptionCause cause,
        bool expectsEpochChange,
        const NCLifecycleInterruptionSample& sample) noexcept;

    void RecordEpochPublished(MotionExecutionEpoch executionEpoch) noexcept;

    // Stage NC-0.2J.6.3: import only the exact acknowledgement already
    // proved by NCAlarmEmergencyStopBoundaryShadow.  This class remains an
    // observer and does not request E-stop, mutate Motion, or release Safety.
    void RecordAlarmStopAcknowledged(
        MotionExecutionEpoch appliedExecutionEpoch,
        bool epochChangeRequired) noexcept;

    void RecordTerminalFeedback(
        const MotionFeedbackEvent& event,
        bool ledgerAccepted) noexcept;

    void Observe(const NCLifecycleInterruptionSample& sample) noexcept;

    void Supersede() noexcept;

    bool IsActive() const noexcept
    {
        return m_snapshot.active;
    }

    NCLifecycleInterruptionSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCLifecycleInterruptionCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static bool IsTerminalFailure(MotionFeedbackType type) noexcept;
    static std::uint64_t MonotonicDelta(
        std::uint64_t baseline,
        std::uint64_t current) noexcept;
    static std::uint64_t LedgerIntegrityDelta(
        const NCLifecycleInterruptionSample& baseline,
        const NCLifecycleInterruptionSample& current) noexcept;
    bool IsAlarmRequestTerminalCandidate(
        const MotionFeedbackEvent& event) const noexcept;

    std::uint64_t AllocateSequence() noexcept;
    void UpdateSample(const NCLifecycleInterruptionSample& sample) noexcept;
    void SetWaitDecision(NCLifecycleInterruptionDecision decision) noexcept;
    void MarkEvidenceGap(NCLifecycleInterruptionDecision decision) noexcept;
    void MarkSuperseded(NCLifecycleInterruptionDecision decision) noexcept;
    void MarkAlarmStopClosed() noexcept;

    NCLifecycleInterruptionSample m_baseline{};
    NCLifecycleInterruptionSnapshot m_snapshot{};
    NCLifecycleInterruptionCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;

    // Stage NC-0.2J.6.3.1: terminal events may arrive before the exact J.6
    // acknowledgement is imported.  Keep bounded candidate counts here and
    // authorise them only after UpdateSample() observes that acknowledgement.
    std::uint64_t m_alarmAbortCandidateCount = 0ULL;
    std::uint64_t m_alarmOwnerConflictRejectCandidateCount = 0ULL;
    std::uint64_t m_alarmStaleEpochRejectCandidateCount = 0ULL;
};
