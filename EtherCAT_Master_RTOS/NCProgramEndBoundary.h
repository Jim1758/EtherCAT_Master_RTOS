#pragma once

#include "NCProgramCache.h"
#include "NCBlockLifecycleLedger.h"
#include "MotionExecutionContract.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2G - Program End / Cycle End Completion Gate
//
// Natural EOF, M02 and M30 all enter this same fixed-size boundary.  Reaching
// the end marker does not mean that the Program Motion Owner may be released.
// Finalization requires two consecutive NC-task samples in which:
//
//   * the request-time Execution Epoch is still current
//   * the request-time MotionOwner Generation is still current
//   * every Program Block Lifecycle is terminal
//   * Axis / Motion command transports are empty
//   * Producer Notice / Motion Feedback transports are empty and synchronized
//   * no Wait Callback or Block Completion binding remains active
//   * no Safety / Recovery request is pending
//   * the interpolation group is physically standstill
//   * no integrity counter advanced during the active Program Run
//
// The request-time identity is intentionally separate from the run-start
// identity.  GOTO may legitimately advance Execution Epoch inside one Program
// Run; M02/M30/EOF must therefore validate the Epoch captured at End Request.
//
// The gate is allocation-free and owned by the NC 10 ms task.
// =============================================================================

using NCProgramRunId = std::uint64_t;
using NCProgramEndRequestId = std::uint64_t;

constexpr NCProgramRunId NC_PROGRAM_RUN_ID_INVALID = 0ULL;
constexpr NCProgramEndRequestId NC_PROGRAM_END_REQUEST_ID_INVALID = 0ULL;
constexpr std::uint32_t NC_PROGRAM_END_STABLE_PASSES_REQUIRED = 2U;

enum class NCProgramEndCause : std::uint8_t
{
    NONE = 0,
    NATURAL_EOF = 1,
    M02 = 2,
    M30 = 3
};

enum class NCProgramEndPhase : std::uint8_t
{
    IDLE = 0,
    RUN_ACTIVE = 1,
    DRAINING = 2,
    READY_TO_FINALIZE = 3,
    FINALIZED = 4,
    CANCELLED = 5,
    FAIL_CLOSED = 6,
    START_BLOCKED = 7
};

enum class NCProgramEndDecision : std::uint8_t
{
    NONE = 0,
    RUN_STARTED = 1,
    RUN_START_BLOCKED_DIRTY = 2,
    END_REQUESTED = 3,
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
    READY_TO_FINALIZE = 18,
    FAIL_EXECUTION_EPOCH_CHANGED = 19,
    FAIL_OWNER_LEASE_LOST = 20,
    FAIL_INTEGRITY_COUNTER_ADVANCED = 21,
    FINALIZED = 22,
    CANCELLED = 23
};

struct NCProgramRunIdentity
{
    NCProgramRunId runId = NC_PROGRAM_RUN_ID_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    bool IsValid() const noexcept
    {
        return
            runId != NC_PROGRAM_RUN_ID_INVALID &&
            scope != NCProgramScope::NONE &&
            cacheGeneration != NC_PROGRAM_CACHE_GENERATION_INVALID &&
            executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            owner != MotionOwner::NONE &&
            ownerGeneration != MOTION_OWNER_GENERATION_INVALID;
    }
};

struct NCProgramEndIntegrityCounters
{
    std::uint64_t blockFailed = 0ULL;
    std::uint64_t ncDispatchFailed = 0ULL;
    std::uint64_t motionCaptureOverflow = 0ULL;
    std::uint64_t orphanFeedback = 0ULL;
    std::uint64_t duplicateTerminalFeedback = 0ULL;
    std::uint64_t terminalFeedbackConflict = 0ULL;
    std::uint64_t activeBlockOverwrite = 0ULL;
    std::uint64_t activeSegmentIndexOverwrite = 0ULL;

    std::uint64_t axisCommandQueueFull = 0ULL;
    std::uint64_t axisCommandResultOverflow = 0ULL;
    std::uint64_t staleCommandDiscard = 0ULL;
    std::uint64_t ownerConflictReject = 0ULL;
    std::uint64_t commandQueueFullReject = 0ULL;
    std::uint64_t commandReplayOverflow = 0ULL;
    std::uint64_t feedbackOverflow = 0ULL;
    std::uint64_t feedbackNoticeOverflow = 0ULL;
    std::uint64_t feedbackSequenceGap = 0ULL;
};

struct NCProgramEndGateSample
{
    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

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

    // Stage NC-0.2J.5: two supervisory confirmations must consume two
    // different RT publications carrying the same nonzero proof sequence.
    // A stopped Runtime or a new proof episode cannot complete an older one.
    std::uint64_t ncSettlePublicationGeneration = 0ULL;
    std::uint64_t ncSettleProofSequence = 0ULL;

    bool ownerLeaseCurrent = false;
    bool safetyOrRecoveryPending = false;
    bool waitCallbackActive = false;
    bool completionBindingActive = false;
    bool groupStandstill = false;

    NCProgramEndIntegrityCounters integrity{};
};

struct NCProgramEndGateSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCProgramRunIdentity run{};
    NCProgramEndRequestId requestId =
        NC_PROGRAM_END_REQUEST_ID_INVALID;
    NCProgramEndCause cause = NCProgramEndCause::NONE;
    NCProgramEndPhase phase = NCProgramEndPhase::IDLE;
    NCProgramEndDecision decision = NCProgramEndDecision::NONE;

    NCBlockDispatchId markerDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;
    int sourcePC = -1;
    int sourceLineNumber = 0;

    MotionExecutionEpoch requestExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner requestOwner = MotionOwner::NONE;
    MotionOwnerGeneration requestOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

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

    std::uint64_t ncSettlePublicationGeneration = 0ULL;
    std::uint64_t ncSettleProofSequence = 0ULL;

    std::uint64_t integrityDelta = 0ULL;
    std::uint32_t stablePasses = 0U;

    bool feedbackSequenceSynchronized = false;
    bool ownerLeaseCurrent = false;
    bool safetyOrRecoveryPending = false;
    bool waitCallbackActive = false;
    bool completionBindingActive = false;
    bool groupStandstill = false;
    bool requestPending = false;
    bool readyToFinalize = false;
    bool failClosed = false;

    bool IsValid() const noexcept
    {
        return sequence != 0ULL && run.IsValid();
    }
};

struct NCProgramEndGateCounters
{
    std::uint64_t runStartAttempts = 0ULL;
    std::uint64_t runsStarted = 0ULL;
    std::uint64_t runStartBlocked = 0ULL;

    std::uint64_t requests = 0ULL;
    std::uint64_t naturalEofRequests = 0ULL;
    std::uint64_t m02Requests = 0ULL;
    std::uint64_t m30Requests = 0ULL;
    std::uint64_t rejectedRequests = 0ULL;

    std::uint64_t evaluations = 0ULL;
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

    std::uint64_t readyToFinalize = 0ULL;
    std::uint64_t finalized = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t failClosed = 0ULL;
    std::uint64_t epochMismatch = 0ULL;
    std::uint64_t ownerLeaseLost = 0ULL;
    std::uint64_t integrityFailure = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCProgramRunIdentity>::value,
    "NCProgramRunIdentity must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCProgramEndIntegrityCounters>::value,
    "NCProgramEndIntegrityCounters must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCProgramEndGateSample>::value,
    "NCProgramEndGateSample must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCProgramEndGateSnapshot>::value,
    "NCProgramEndGateSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCProgramEndGateCounters>::value,
    "NCProgramEndGateCounters must remain trivially copyable.");

class NCProgramEndBoundary
{
public:
    bool BeginRun(
        NCProgramScope scope,
        NCProgramCacheGeneration cacheGeneration,
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease,
        const NCProgramEndGateSample& baseline) noexcept;

    bool RequestEnd(
        NCProgramEndCause cause,
        int sourcePC,
        int sourceLineNumber,
        NCBlockDispatchId markerDispatchId,
        MotionExecutionEpoch requestExecutionEpoch,
        const MotionOwnerLease& requestOwnerLease) noexcept;

    bool Evaluate(const NCProgramEndGateSample& sample) noexcept;
    bool MarkFinalized() noexcept;
    void Cancel() noexcept;

    bool IsRunActive() const noexcept { return m_runActive; }
    bool IsEndPending() const noexcept { return m_endPending; }
    bool IsFailClosed() const noexcept { return m_failClosed; }

    NCProgramEndGateSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCProgramEndGateCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static bool FeedbackSequenceSynchronized(
        MotionFeedbackSequence published,
        MotionFeedbackSequence consumed) noexcept;

    static bool IsCleanRunStart(
        const NCProgramEndGateSample& sample) noexcept;

    static std::uint64_t IntegrityDelta(
        const NCProgramEndIntegrityCounters& baseline,
        const NCProgramEndIntegrityCounters& current) noexcept;

    static std::uint64_t MonotonicDelta(
        std::uint64_t baseline,
        std::uint64_t current) noexcept;

    NCProgramRunId AllocateRunId() noexcept;
    NCProgramEndRequestId AllocateRequestId() noexcept;
    std::uint64_t AllocateSequence() noexcept;

    void ResetStableConfirmation() noexcept;

    void RefreshSnapshot(
        const NCProgramEndGateSample& sample,
        NCProgramEndPhase phase,
        NCProgramEndDecision decision,
        bool readyToFinalize,
        bool failClosed) noexcept;

    bool FailClosed(
        const NCProgramEndGateSample& sample,
        NCProgramEndDecision decision) noexcept;

    NCProgramRunId m_nextRunId = 1ULL;
    NCProgramEndRequestId m_nextRequestId = 1ULL;
    std::uint64_t m_nextSequence = 1ULL;

    NCProgramRunIdentity m_run{};
    NCProgramEndIntegrityCounters m_runBaselineIntegrity{};
    NCProgramEndRequestId m_requestId =
        NC_PROGRAM_END_REQUEST_ID_INVALID;
    NCProgramEndCause m_cause = NCProgramEndCause::NONE;
    NCBlockDispatchId m_markerDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;
    int m_sourcePC = -1;
    int m_sourceLineNumber = 0;

    MotionExecutionEpoch m_requestExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease m_requestOwnerLease{};
    std::uint32_t m_stablePasses = 0U;
    std::uint64_t m_lastStableSettlePublicationGeneration = 0ULL;
    std::uint64_t m_stableSettleProofSequence = 0ULL;

    bool m_runActive = false;
    bool m_endPending = false;
    bool m_failClosed = false;
    bool m_readyRecorded = false;
    bool m_failureRecorded = false;

    NCProgramEndGateSample m_lastSample{};
    NCProgramEndGateSnapshot m_snapshot{};
    NCProgramEndGateCounters m_counters{};
};
