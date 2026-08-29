#include "NCProgramEndBoundary.h"

#include <limits>

namespace
{
    template <typename T>
    T AllocateNonZero(T& next) noexcept
    {
        T value = next++;
        if (value == static_cast<T>(0))
        {
            value = next++;
        }
        return value;
    }

    std::uint64_t AddSaturating(
        std::uint64_t lhs,
        std::uint64_t rhs) noexcept
    {
        const std::uint64_t maxValue =
            (std::numeric_limits<std::uint64_t>::max)();
        return rhs > maxValue - lhs ? maxValue : lhs + rhs;
    }
}

bool NCProgramEndBoundary::FeedbackSequenceSynchronized(
    MotionFeedbackSequence published,
    MotionFeedbackSequence consumed) noexcept
{
    return published == consumed;
}

bool NCProgramEndBoundary::IsCleanRunStart(
    const NCProgramEndGateSample& sample) noexcept
{
    // Run-start cleanliness is a transport / ownership boundary, not the
    // final physical standstill boundary.  On the EDM test runtime an
    // enabled, stationary servo can legitimately remain in a non-IDLE
    // AxisContext state, so IsGroupStandstill() may be false before the first
    // NC command even though every queue, callback and lifecycle record is
    // clean.  Requiring groupStandstill here blocks every Cycle Start.
    //
    // Physical standstill remains mandatory in Evaluate() before EOF/M02/M30
    // can be finalized, so this relaxation does not weaken Program End.
    return
        sample.executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        sample.currentOwner != MotionOwner::NONE &&
        sample.currentOwnerGeneration != MOTION_OWNER_GENERATION_INVALID &&
        sample.activeBlocks == 0U &&
        sample.axisCommandDepth == 0U &&
        sample.axisResultDepth == 0U &&
        sample.commandQueueDepth == 0U &&
        sample.commandIngressDepth == 0U &&
        sample.commandReplayDepth == 0U &&
        sample.feedbackDepth == 0U &&
        sample.feedbackNoticeDepth == 0U &&
        FeedbackSequenceSynchronized(
            sample.lastPublishedFeedbackSequence,
            sample.lastConsumedFeedbackSequence) &&
        sample.ownerLeaseCurrent &&
        !sample.safetyOrRecoveryPending &&
        !sample.waitCallbackActive &&
        !sample.completionBindingActive;
}

std::uint64_t NCProgramEndBoundary::MonotonicDelta(
    std::uint64_t baseline,
    std::uint64_t current) noexcept
{
    // Counter regression is also an integrity failure.  Returning one rather
    // than zero prevents a reset/wrap from silently looking unchanged.
    return current >= baseline ? current - baseline : 1ULL;
}

std::uint64_t NCProgramEndBoundary::IntegrityDelta(
    const NCProgramEndIntegrityCounters& baseline,
    const NCProgramEndIntegrityCounters& current) noexcept
{
    std::uint64_t total = 0ULL;
    const auto add = [&](std::uint64_t before, std::uint64_t now) noexcept
    {
        total = AddSaturating(total, MonotonicDelta(before, now));
    };

    add(baseline.blockFailed, current.blockFailed);
    add(baseline.ncDispatchFailed, current.ncDispatchFailed);
    add(baseline.motionCaptureOverflow, current.motionCaptureOverflow);
    add(baseline.orphanFeedback, current.orphanFeedback);
    add(baseline.duplicateTerminalFeedback, current.duplicateTerminalFeedback);
    add(baseline.terminalFeedbackConflict, current.terminalFeedbackConflict);
    add(baseline.activeBlockOverwrite, current.activeBlockOverwrite);
    add(baseline.activeSegmentIndexOverwrite, current.activeSegmentIndexOverwrite);
    add(baseline.axisCommandQueueFull, current.axisCommandQueueFull);
    add(baseline.axisCommandResultOverflow, current.axisCommandResultOverflow);
    add(baseline.staleCommandDiscard, current.staleCommandDiscard);
    add(baseline.ownerConflictReject, current.ownerConflictReject);
    add(baseline.commandQueueFullReject, current.commandQueueFullReject);
    add(baseline.commandReplayOverflow, current.commandReplayOverflow);
    add(baseline.feedbackOverflow, current.feedbackOverflow);
    add(baseline.feedbackNoticeOverflow, current.feedbackNoticeOverflow);
    add(baseline.feedbackSequenceGap, current.feedbackSequenceGap);
    return total;
}

NCProgramRunId NCProgramEndBoundary::AllocateRunId() noexcept
{
    return AllocateNonZero(m_nextRunId);
}

NCProgramEndRequestId NCProgramEndBoundary::AllocateRequestId() noexcept
{
    return AllocateNonZero(m_nextRequestId);
}

std::uint64_t NCProgramEndBoundary::AllocateSequence() noexcept
{
    return AllocateNonZero(m_nextSequence);
}

void NCProgramEndBoundary::ResetStableConfirmation() noexcept
{
    m_stablePasses = 0U;
    m_lastStableSettlePublicationGeneration = 0ULL;
    m_stableSettleProofSequence = 0ULL;
    m_readyRecorded = false;
}

bool NCProgramEndBoundary::BeginRun(
    NCProgramScope scope,
    NCProgramCacheGeneration cacheGeneration,
    MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease,
    const NCProgramEndGateSample& baseline) noexcept
{
    ++m_counters.runStartAttempts;

    if (m_runActive || m_endPending)
    {
        Cancel();
    }

    m_run = NCProgramRunIdentity{};
    m_run.runId = AllocateRunId();
    m_run.scope = scope;
    m_run.cacheGeneration = cacheGeneration;
    m_run.executionEpoch = executionEpoch;
    m_run.owner = ownerLease.owner;
    m_run.ownerGeneration = ownerLease.generation;

    m_requestId = NC_PROGRAM_END_REQUEST_ID_INVALID;
    m_cause = NCProgramEndCause::NONE;
    m_markerDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    m_sourcePC = -1;
    m_sourceLineNumber = 0;
    m_requestExecutionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_requestOwnerLease = MotionOwnerLease{};
    m_endPending = false;
    m_failClosed = false;
    m_failureRecorded = false;
    ResetStableConfirmation();

    m_runBaselineIntegrity = baseline.integrity;
    m_lastSample = baseline;

    const bool identityValid = m_run.IsValid();
    const bool exactOwner =
        baseline.currentOwner == ownerLease.owner &&
        baseline.currentOwnerGeneration == ownerLease.generation;
    const bool cleanStart =
        identityValid &&
        baseline.executionEpoch == executionEpoch &&
        exactOwner &&
        IsCleanRunStart(baseline);

    if (!cleanStart)
    {
        m_runActive = false;
        ++m_counters.runStartBlocked;
        RefreshSnapshot(
            baseline,
            NCProgramEndPhase::START_BLOCKED,
            NCProgramEndDecision::RUN_START_BLOCKED_DIRTY,
            false,
            true);
        return false;
    }

    m_runActive = true;
    ++m_counters.runsStarted;

    RefreshSnapshot(
        baseline,
        NCProgramEndPhase::RUN_ACTIVE,
        NCProgramEndDecision::RUN_STARTED,
        false,
        false);
    return true;
}

bool NCProgramEndBoundary::RequestEnd(
    NCProgramEndCause cause,
    int sourcePC,
    int sourceLineNumber,
    NCBlockDispatchId markerDispatchId,
    MotionExecutionEpoch requestExecutionEpoch,
    const MotionOwnerLease& requestOwnerLease) noexcept
{
    const bool requestIdentityValid =
        requestExecutionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        requestOwnerLease.IsValid();

    if (!m_runActive || !m_run.IsValid() ||
        cause == NCProgramEndCause::NONE ||
        !requestIdentityValid)
    {
        ++m_counters.rejectedRequests;
        return false;
    }

    if (m_endPending)
    {
        return
            m_cause == cause &&
            m_sourcePC == sourcePC &&
            m_markerDispatchId == markerDispatchId &&
            m_requestExecutionEpoch == requestExecutionEpoch &&
            m_requestOwnerLease.owner == requestOwnerLease.owner &&
            m_requestOwnerLease.generation == requestOwnerLease.generation;
    }

    m_requestId = AllocateRequestId();
    m_cause = cause;
    m_sourcePC = sourcePC;
    m_sourceLineNumber = sourceLineNumber;
    m_markerDispatchId = markerDispatchId;
    m_requestExecutionEpoch = requestExecutionEpoch;
    m_requestOwnerLease = requestOwnerLease;
    m_endPending = true;
    m_failClosed = false;
    m_failureRecorded = false;
    ResetStableConfirmation();

    ++m_counters.requests;
    switch (cause)
    {
    case NCProgramEndCause::NATURAL_EOF:
        ++m_counters.naturalEofRequests;
        break;
    case NCProgramEndCause::M02:
        ++m_counters.m02Requests;
        break;
    case NCProgramEndCause::M30:
        ++m_counters.m30Requests;
        break;
    case NCProgramEndCause::NONE:
    default:
        break;
    }

    NCProgramEndGateSample requestSample = m_lastSample;
    requestSample.executionEpoch = requestExecutionEpoch;
    requestSample.currentOwner = requestOwnerLease.owner;
    requestSample.currentOwnerGeneration = requestOwnerLease.generation;
    requestSample.ownerLeaseCurrent = true;
    RefreshSnapshot(
        requestSample,
        NCProgramEndPhase::DRAINING,
        NCProgramEndDecision::END_REQUESTED,
        false,
        false);
    return true;
}

bool NCProgramEndBoundary::FailClosed(
    const NCProgramEndGateSample& sample,
    NCProgramEndDecision decision) noexcept
{
    m_failClosed = true;
    ResetStableConfirmation();

    if (!m_failureRecorded)
    {
        ++m_counters.failClosed;
        switch (decision)
        {
        case NCProgramEndDecision::FAIL_EXECUTION_EPOCH_CHANGED:
            ++m_counters.epochMismatch;
            break;
        case NCProgramEndDecision::FAIL_OWNER_LEASE_LOST:
            ++m_counters.ownerLeaseLost;
            break;
        case NCProgramEndDecision::FAIL_INTEGRITY_COUNTER_ADVANCED:
            ++m_counters.integrityFailure;
            break;
        default:
            break;
        }
        m_failureRecorded = true;
    }

    RefreshSnapshot(
        sample,
        NCProgramEndPhase::FAIL_CLOSED,
        decision,
        false,
        true);
    return false;
}

bool NCProgramEndBoundary::Evaluate(
    const NCProgramEndGateSample& sample) noexcept
{
    if (!m_runActive || !m_endPending || m_failClosed)
    {
        return false;
    }

    ++m_counters.evaluations;
    m_lastSample = sample;

    // Validate against the End Request identity, not the CycleStart identity.
    // GOTO may legitimately have advanced Epoch during this Program Run.
    if (sample.executionEpoch != m_requestExecutionEpoch)
    {
        return FailClosed(
            sample,
            NCProgramEndDecision::FAIL_EXECUTION_EPOCH_CHANGED);
    }

    const std::uint64_t integrityDelta =
        IntegrityDelta(m_runBaselineIntegrity, sample.integrity);
    if (integrityDelta != 0ULL)
    {
        return FailClosed(
            sample,
            NCProgramEndDecision::FAIL_INTEGRITY_COUNTER_ADVANCED);
    }

    const bool exactOwner =
        sample.currentOwner == m_requestOwnerLease.owner &&
        sample.currentOwnerGeneration == m_requestOwnerLease.generation;
    if (!sample.ownerLeaseCurrent || !exactOwner)
    {
        return FailClosed(
            sample,
            NCProgramEndDecision::FAIL_OWNER_LEASE_LOST);
    }

    NCProgramEndDecision decision = NCProgramEndDecision::NONE;

    if (sample.activeBlocks != 0U)
    {
        decision = NCProgramEndDecision::WAIT_ACTIVE_BLOCKS;
        ++m_counters.waitActiveBlocks;
    }
    else if (sample.axisCommandDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_AXIS_COMMAND;
        ++m_counters.waitAxisCommand;
    }
    else if (sample.axisResultDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_AXIS_RESULT;
        ++m_counters.waitAxisResult;
    }
    else if (sample.commandIngressDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_COMMAND_INGRESS;
        ++m_counters.waitCommandIngress;
    }
    else if (sample.commandReplayDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_COMMAND_REPLAY;
        ++m_counters.waitCommandReplay;
    }
    else if (sample.commandQueueDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_COMMAND_QUEUE;
        ++m_counters.waitCommandQueue;
    }
    else if (sample.feedbackNoticeDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_FEEDBACK_NOTICE;
        ++m_counters.waitFeedbackNotice;
    }
    else if (sample.feedbackDepth != 0U)
    {
        decision = NCProgramEndDecision::WAIT_FEEDBACK;
        ++m_counters.waitFeedback;
    }
    else if (!FeedbackSequenceSynchronized(
        sample.lastPublishedFeedbackSequence,
        sample.lastConsumedFeedbackSequence))
    {
        decision = NCProgramEndDecision::WAIT_FEEDBACK_SEQUENCE;
        ++m_counters.waitFeedbackSequence;
    }
    else if (sample.waitCallbackActive)
    {
        decision = NCProgramEndDecision::WAIT_CALLBACK;
        ++m_counters.waitCallback;
    }
    else if (sample.completionBindingActive)
    {
        decision = NCProgramEndDecision::WAIT_COMPLETION_BINDING;
        ++m_counters.waitCompletionBinding;
    }
    else if (sample.safetyOrRecoveryPending)
    {
        decision = NCProgramEndDecision::WAIT_SAFETY_REQUEST;
        ++m_counters.waitSafetyRequest;
    }
    else if (!sample.groupStandstill ||
        sample.ncSettlePublicationGeneration == 0ULL ||
        sample.ncSettleProofSequence == 0ULL)
    {
        decision = NCProgramEndDecision::WAIT_GROUP_STANDSTILL;
        ++m_counters.waitGroupStandstill;
    }
    else
    {
        // NC-0.2J.5: two supervisory passes must belong to one continuous
        // RT proof episode.  A new nonzero proofSequence starts again at
        // pass one even when the previous episode was already release-ready.
        if (sample.ncSettleProofSequence !=
            m_stableSettleProofSequence)
        {
            ResetStableConfirmation();
            m_stableSettleProofSequence =
                sample.ncSettleProofSequence;
            m_lastStableSettlePublicationGeneration =
                sample.ncSettlePublicationGeneration;
            m_stablePasses = 1U;
        }
        else if (sample.ncSettlePublicationGeneration !=
            m_lastStableSettlePublicationGeneration &&
            m_stablePasses < NC_PROGRAM_END_STABLE_PASSES_REQUIRED)
        {
            m_lastStableSettlePublicationGeneration =
                sample.ncSettlePublicationGeneration;
            ++m_stablePasses;
        }

        if (m_stablePasses < NC_PROGRAM_END_STABLE_PASSES_REQUIRED)
        {
            ++m_counters.waitStableConfirmation;
            RefreshSnapshot(
                sample,
                NCProgramEndPhase::DRAINING,
                NCProgramEndDecision::WAIT_STABLE_CONFIRMATION,
                false,
                false);
            return false;
        }

        if (!m_readyRecorded)
        {
            ++m_counters.readyToFinalize;
            m_readyRecorded = true;
        }

        RefreshSnapshot(
            sample,
            NCProgramEndPhase::READY_TO_FINALIZE,
            NCProgramEndDecision::READY_TO_FINALIZE,
            true,
            false);
        return true;
    }

    ResetStableConfirmation();
    RefreshSnapshot(
        sample,
        NCProgramEndPhase::DRAINING,
        decision,
        false,
        false);
    return false;
}

bool NCProgramEndBoundary::MarkFinalized() noexcept
{
    if (!m_runActive || !m_endPending || m_failClosed ||
        !m_snapshot.readyToFinalize ||
        m_stableSettleProofSequence == 0ULL ||
        m_snapshot.ncSettleProofSequence !=
        m_stableSettleProofSequence ||
        m_stablePasses < NC_PROGRAM_END_STABLE_PASSES_REQUIRED)
    {
        return false;
    }

    m_endPending = false;
    m_runActive = false;
    ++m_counters.finalized;

    RefreshSnapshot(
        m_lastSample,
        NCProgramEndPhase::FINALIZED,
        NCProgramEndDecision::FINALIZED,
        true,
        false);
    return true;
}

void NCProgramEndBoundary::Cancel() noexcept
{
    if (!m_runActive && !m_endPending)
    {
        return;
    }

    ++m_counters.cancelled;
    m_endPending = false;
    m_runActive = false;
    m_failClosed = false;
    ResetStableConfirmation();

    RefreshSnapshot(
        m_lastSample,
        NCProgramEndPhase::CANCELLED,
        NCProgramEndDecision::CANCELLED,
        false,
        false);
}

void NCProgramEndBoundary::RefreshSnapshot(
    const NCProgramEndGateSample& sample,
    NCProgramEndPhase phase,
    NCProgramEndDecision decision,
    bool readyToFinalize,
    bool failClosed) noexcept
{
    m_lastSample = sample;

    NCProgramEndGateSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.run = m_run;
    snapshot.requestId = m_requestId;
    snapshot.cause = m_cause;
    snapshot.phase = phase;
    snapshot.decision = decision;
    snapshot.markerDispatchId = m_markerDispatchId;
    snapshot.sourcePC = m_sourcePC;
    snapshot.sourceLineNumber = m_sourceLineNumber;

    snapshot.requestExecutionEpoch = m_requestExecutionEpoch;
    snapshot.requestOwner = m_requestOwnerLease.owner;
    snapshot.requestOwnerGeneration = m_requestOwnerLease.generation;
    snapshot.currentOwner = sample.currentOwner;
    snapshot.currentOwnerGeneration = sample.currentOwnerGeneration;

    snapshot.activeBlocks = sample.activeBlocks;
    snapshot.axisCommandDepth = sample.axisCommandDepth;
    snapshot.axisResultDepth = sample.axisResultDepth;
    snapshot.commandQueueDepth = sample.commandQueueDepth;
    snapshot.commandIngressDepth = sample.commandIngressDepth;
    snapshot.commandReplayDepth = sample.commandReplayDepth;
    snapshot.feedbackDepth = sample.feedbackDepth;
    snapshot.feedbackNoticeDepth = sample.feedbackNoticeDepth;
    snapshot.lastPublishedFeedbackSequence =
        sample.lastPublishedFeedbackSequence;
    snapshot.lastConsumedFeedbackSequence =
        sample.lastConsumedFeedbackSequence;
    snapshot.ncSettlePublicationGeneration =
        sample.ncSettlePublicationGeneration;
    snapshot.ncSettleProofSequence =
        sample.ncSettleProofSequence;

    snapshot.integrityDelta =
        IntegrityDelta(m_runBaselineIntegrity, sample.integrity);
    snapshot.stablePasses = m_stablePasses;

    snapshot.feedbackSequenceSynchronized =
        FeedbackSequenceSynchronized(
            sample.lastPublishedFeedbackSequence,
            sample.lastConsumedFeedbackSequence);
    snapshot.ownerLeaseCurrent = sample.ownerLeaseCurrent;
    snapshot.safetyOrRecoveryPending = sample.safetyOrRecoveryPending;
    snapshot.waitCallbackActive = sample.waitCallbackActive;
    snapshot.completionBindingActive = sample.completionBindingActive;
    snapshot.groupStandstill = sample.groupStandstill;
    snapshot.requestPending = m_endPending;
    snapshot.readyToFinalize = readyToFinalize;
    snapshot.failClosed = failClosed;

    m_snapshot = snapshot;
}
