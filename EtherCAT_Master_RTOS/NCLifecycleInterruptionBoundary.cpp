#include "NCLifecycleInterruptionBoundary.h"

#include <limits>

namespace
{
    std::uint64_t AddSaturating(
        std::uint64_t lhs,
        std::uint64_t rhs) noexcept
    {
        const std::uint64_t maximum =
            (std::numeric_limits<std::uint64_t>::max)();
        return rhs > maximum - lhs ? maximum : lhs + rhs;
    }
}

bool NCLifecycleInterruptionBoundaryShadow::IsTerminalFailure(
    MotionFeedbackType type) noexcept
{
    return
        type == MotionFeedbackType::REJECTED ||
        type == MotionFeedbackType::CANCELLED ||
        type == MotionFeedbackType::ABORTED ||
        type == MotionFeedbackType::FAULTED;
}

std::uint64_t NCLifecycleInterruptionBoundaryShadow::MonotonicDelta(
    std::uint64_t baseline,
    std::uint64_t current) noexcept
{
    return current >= baseline ? current - baseline : 1ULL;
}

std::uint64_t NCLifecycleInterruptionBoundaryShadow::LedgerIntegrityDelta(
    const NCLifecycleInterruptionSample& baseline,
    const NCLifecycleInterruptionSample& current) noexcept
{
    std::uint64_t total = 0ULL;
    const auto add = [&](std::uint64_t before, std::uint64_t now) noexcept
    {
        total = AddSaturating(total, MonotonicDelta(before, now));
    };

    add(baseline.motionCaptureOverflow, current.motionCaptureOverflow);
    add(baseline.orphanFeedback, current.orphanFeedback);
    add(baseline.duplicateTerminalFeedback, current.duplicateTerminalFeedback);
    add(baseline.terminalFeedbackConflict, current.terminalFeedbackConflict);
    add(baseline.activeBlockOverwrite, current.activeBlockOverwrite);
    add(
        baseline.activeSegmentIndexOverwrite,
        current.activeSegmentIndexOverwrite);
    return total;
}

std::uint64_t NCLifecycleInterruptionBoundaryShadow::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCLifecycleInterruptionBoundaryShadow::UpdateSample(
    const NCLifecycleInterruptionSample& sample) noexcept
{
    m_snapshot.currentExecutionEpoch = sample.executionEpoch;
    m_snapshot.currentOwner = sample.ownerLease.owner;
    m_snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    m_snapshot.activeBlocks = sample.activeBlocks;
    m_snapshot.axisCommandDepth = sample.axisCommandDepth;
    m_snapshot.axisResultDepth = sample.axisResultDepth;
    m_snapshot.commandQueueDepth = sample.commandQueueDepth;
    m_snapshot.commandIngressDepth = sample.commandIngressDepth;
    m_snapshot.commandReplayDepth = sample.commandReplayDepth;
    m_snapshot.feedbackDepth = sample.feedbackDepth;
    m_snapshot.feedbackNoticeDepth = sample.feedbackNoticeDepth;
    m_snapshot.lastPublishedFeedbackSequence =
        sample.lastPublishedFeedbackSequence;
    m_snapshot.lastConsumedFeedbackSequence =
        sample.lastConsumedFeedbackSequence;
    m_snapshot.feedbackSequenceSynchronized =
        sample.lastPublishedFeedbackSequence ==
        sample.lastConsumedFeedbackSequence;
    m_snapshot.safetyOrRecoveryPending =
        sample.safetyOrRecoveryPending;
    m_snapshot.waitCallbackActive = sample.waitCallbackActive;
    m_snapshot.completionBindingActive =
        sample.completionBindingActive;
    m_snapshot.groupStandstill = sample.groupStandstill;

    m_snapshot.blockFailureDelta =
        MonotonicDelta(m_baseline.blockFailed, sample.blockFailed);
    m_snapshot.dispatchDelta =
        MonotonicDelta(
            m_baseline.blocksDispatched,
            sample.blocksDispatched);
    if (m_snapshot.dispatchDelta != 0ULL &&
        !m_snapshot.postInterruptionDispatchObserved)
    {
        m_snapshot.postInterruptionDispatchObserved = true;
        ++m_counters.postInterruptionDispatch;
    }
    m_snapshot.feedbackRejectedDelta =
        MonotonicDelta(
            m_baseline.feedbackRejected,
            sample.feedbackRejected);
    m_snapshot.feedbackCancelledDelta =
        MonotonicDelta(
            m_baseline.feedbackCancelled,
            sample.feedbackCancelled);
    m_snapshot.feedbackAbortedDelta =
        MonotonicDelta(
            m_baseline.feedbackAborted,
            sample.feedbackAborted);
    m_snapshot.feedbackFaultedDelta =
        MonotonicDelta(
            m_baseline.feedbackFaulted,
            sample.feedbackFaulted);
    m_snapshot.ledgerIntegrityDelta =
        LedgerIntegrityDelta(m_baseline, sample);
    m_snapshot.feedbackOverflowDelta =
        MonotonicDelta(
            m_baseline.feedbackOverflow,
            sample.feedbackOverflow);
    m_snapshot.feedbackNoticeOverflowDelta =
        MonotonicDelta(
            m_baseline.feedbackNoticeOverflow,
            sample.feedbackNoticeOverflow);
    m_snapshot.feedbackSequenceGapDelta =
        MonotonicDelta(
            m_baseline.feedbackSequenceGap,
            sample.feedbackSequenceGap);
}

void NCLifecycleInterruptionBoundaryShadow::Begin(
    NCLifecycleInterruptionCause cause,
    bool expectsEpochChange,
    const NCLifecycleInterruptionSample& sample) noexcept
{
    ++m_counters.requestAttempts;

    if (cause == NCLifecycleInterruptionCause::NONE)
    {
        return;
    }

    if (m_snapshot.active)
    {
        MarkSuperseded(NCLifecycleInterruptionDecision::SUPERSEDED);
    }

    m_baseline = sample;

    NCLifecycleInterruptionSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.cause = cause;
    snapshot.phase = NCLifecycleInterruptionPhase::REQUESTED;
    snapshot.decision = NCLifecycleInterruptionDecision::REQUEST_LATCHED;
    snapshot.requestExecutionEpoch = sample.executionEpoch;
    snapshot.currentExecutionEpoch = sample.executionEpoch;
    snapshot.requestOwner = sample.ownerLease.owner;
    snapshot.requestOwnerGeneration = sample.ownerLease.generation;
    snapshot.currentOwner = sample.ownerLease.owner;
    snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    snapshot.requestDispatchId = sample.lastDispatchId;
    snapshot.requestPC = sample.activePC;
    snapshot.requestActiveBlocks = sample.activeBlocks;
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
    snapshot.feedbackSequenceSynchronized =
        sample.lastPublishedFeedbackSequence ==
        sample.lastConsumedFeedbackSequence;
    snapshot.active = true;
    snapshot.expectsEpochChange = expectsEpochChange;
    snapshot.safetyOrRecoveryPending =
        sample.safetyOrRecoveryPending;
    snapshot.waitCallbackActive = sample.waitCallbackActive;
    snapshot.completionBindingActive =
        sample.completionBindingActive;
    snapshot.groupStandstill = sample.groupStandstill;
    m_snapshot = snapshot;

    ++m_counters.requestsLatched;
    if (expectsEpochChange)
    {
        ++m_counters.epochPublicationsExpected;
    }

    switch (cause)
    {
    case NCLifecycleInterruptionCause::RESET:
        ++m_counters.resetRequests;
        break;
    case NCLifecycleInterruptionCause::ALARM:
        ++m_counters.alarmRequests;
        break;
    case NCLifecycleInterruptionCause::PROGRAM_REPLACED:
        ++m_counters.programReplaceRequests;
        break;
    case NCLifecycleInterruptionCause::MDI_REPLACED:
        ++m_counters.mdiReplaceRequests;
        break;
    case NCLifecycleInterruptionCause::MANUAL_AUTO_REPLACED:
        ++m_counters.manualAutoReplaceRequests;
        break;
    case NCLifecycleInterruptionCause::DYNAMIC_CODE_REPLACED:
        ++m_counters.dynamicCodeReplaceRequests;
        break;
    case NCLifecycleInterruptionCause::GOTO_EPOCH:
        ++m_counters.gotoEpochRequests;
        break;
    case NCLifecycleInterruptionCause::MOTION_REJECTED:
    case NCLifecycleInterruptionCause::MOTION_CANCELLED:
    case NCLifecycleInterruptionCause::MOTION_ABORTED:
    case NCLifecycleInterruptionCause::MOTION_FAULTED:
        ++m_counters.terminalTriggeredRequests;
        break;
    case NCLifecycleInterruptionCause::NONE:
    default:
        break;
    }
}

void NCLifecycleInterruptionBoundaryShadow::RecordEpochPublished(
    MotionExecutionEpoch executionEpoch) noexcept
{
    if (!m_snapshot.active || !m_snapshot.expectsEpochChange)
    {
        return;
    }

    m_snapshot.publishedExecutionEpoch = executionEpoch;
    m_snapshot.epochPublicationObserved =
        executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        executionEpoch != m_snapshot.requestExecutionEpoch;

    if (!m_snapshot.epochPublicationObserved)
    {
        m_snapshot.decision =
            NCLifecycleInterruptionDecision::WAIT_EPOCH_PUBLICATION;
        return;
    }

    m_snapshot.phase = NCLifecycleInterruptionPhase::EPOCH_PUBLISHED;
    m_snapshot.decision =
        NCLifecycleInterruptionDecision::EPOCH_PUBLICATION_OBSERVED;
    ++m_counters.epochPublicationsObserved;
}

void NCLifecycleInterruptionBoundaryShadow::RecordTerminalFeedback(
    const MotionFeedbackEvent& event,
    bool ledgerAccepted) noexcept
{
    if (!m_snapshot.active || !IsTerminalFailure(event.type))
    {
        return;
    }

    m_snapshot.terminalFailureObserved = true;
    m_snapshot.lastTerminalFeedbackSequence = event.sequence;
    m_snapshot.lastTerminalIdentity = event.identity;
    m_snapshot.lastTerminalFeedbackType = event.type;
    m_snapshot.lastTerminalRejectReason = event.rejectReason;
    m_snapshot.lastTerminalErrorCode = event.errorCode;

    switch (event.type)
    {
    case MotionFeedbackType::REJECTED:
        ++m_counters.terminalRejected;
        break;
    case MotionFeedbackType::CANCELLED:
        ++m_counters.terminalCancelled;
        break;
    case MotionFeedbackType::ABORTED:
        ++m_counters.terminalAborted;
        break;
    case MotionFeedbackType::FAULTED:
        ++m_counters.terminalFaulted;
        break;
    default:
        break;
    }

    if (!ledgerAccepted)
    {
        m_snapshot.terminalFeedbackLedgerAccepted = false;
        ++m_counters.terminalLedgerRejected;
        MarkEvidenceGap(
            NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_REJECTED);
    }
}

void NCLifecycleInterruptionBoundaryShadow::SetWaitDecision(
    NCLifecycleInterruptionDecision decision) noexcept
{
    m_snapshot.phase = NCLifecycleInterruptionPhase::DRAINING;
    m_snapshot.decision = decision;
    m_snapshot.stableSamples = 0U;
    m_snapshot.quiescentReady = false;

    switch (decision)
    {
    case NCLifecycleInterruptionDecision::WAIT_EPOCH_PUBLICATION:
        ++m_counters.waitEpochPublication;
        break;
    case NCLifecycleInterruptionDecision::WAIT_ACTIVE_BLOCKS:
        ++m_counters.waitActiveBlocks;
        break;
    case NCLifecycleInterruptionDecision::WAIT_AXIS_COMMAND:
        ++m_counters.waitAxisCommand;
        break;
    case NCLifecycleInterruptionDecision::WAIT_AXIS_RESULT:
        ++m_counters.waitAxisResult;
        break;
    case NCLifecycleInterruptionDecision::WAIT_COMMAND_INGRESS:
        ++m_counters.waitCommandIngress;
        break;
    case NCLifecycleInterruptionDecision::WAIT_COMMAND_REPLAY:
        ++m_counters.waitCommandReplay;
        break;
    case NCLifecycleInterruptionDecision::WAIT_COMMAND_QUEUE:
        ++m_counters.waitCommandQueue;
        break;
    case NCLifecycleInterruptionDecision::WAIT_FEEDBACK_NOTICE:
        ++m_counters.waitFeedbackNotice;
        break;
    case NCLifecycleInterruptionDecision::WAIT_FEEDBACK:
        ++m_counters.waitFeedback;
        break;
    case NCLifecycleInterruptionDecision::WAIT_FEEDBACK_SEQUENCE:
        ++m_counters.waitFeedbackSequence;
        break;
    case NCLifecycleInterruptionDecision::WAIT_CALLBACK:
        ++m_counters.waitCallback;
        break;
    case NCLifecycleInterruptionDecision::WAIT_COMPLETION_BINDING:
        ++m_counters.waitCompletionBinding;
        break;
    case NCLifecycleInterruptionDecision::WAIT_SAFETY_REQUEST:
        ++m_counters.waitSafetyRequest;
        break;
    case NCLifecycleInterruptionDecision::WAIT_GROUP_STANDSTILL:
        ++m_counters.waitGroupStandstill;
        break;
    default:
        break;
    }
}

void NCLifecycleInterruptionBoundaryShadow::MarkEvidenceGap(
    NCLifecycleInterruptionDecision decision) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.phase = NCLifecycleInterruptionPhase::EVIDENCE_GAP;
    m_snapshot.decision = decision;
    m_snapshot.active = false;
    m_snapshot.evidenceGap = true;
    m_snapshot.quiescentReady = false;
    m_snapshot.quiescent = false;
    ++m_counters.evidenceGap;

    switch (decision)
    {
    case NCLifecycleInterruptionDecision::EVIDENCE_FEEDBACK_OVERFLOW:
        ++m_counters.feedbackOverflowEvidenceGap;
        break;
    case NCLifecycleInterruptionDecision::EVIDENCE_NOTICE_OVERFLOW:
        ++m_counters.feedbackNoticeOverflowEvidenceGap;
        break;
    case NCLifecycleInterruptionDecision::EVIDENCE_SEQUENCE_GAP:
        ++m_counters.feedbackSequenceEvidenceGap;
        break;
    case NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_INTEGRITY:
        ++m_counters.ledgerIntegrityEvidenceGap;
        break;
    case NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_REJECTED:
        ++m_counters.ledgerRejectedEvidenceGap;
        break;
    default:
        break;
    }
}

void NCLifecycleInterruptionBoundaryShadow::MarkSuperseded(
    NCLifecycleInterruptionDecision decision) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.phase = NCLifecycleInterruptionPhase::SUPERSEDED;
    m_snapshot.decision = decision;
    m_snapshot.active = false;
    m_snapshot.superseded = true;
    ++m_counters.superseded;

    if (decision == NCLifecycleInterruptionDecision::EPOCH_SUPERSEDED)
    {
        ++m_counters.epochSuperseded;
    }
}

void NCLifecycleInterruptionBoundaryShadow::Observe(
    const NCLifecycleInterruptionSample& sample) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    ++m_counters.evaluations;
    UpdateSample(sample);

    if (m_snapshot.feedbackOverflowDelta != 0ULL)
    {
        MarkEvidenceGap(
            NCLifecycleInterruptionDecision::EVIDENCE_FEEDBACK_OVERFLOW);
        return;
    }
    if (m_snapshot.feedbackNoticeOverflowDelta != 0ULL)
    {
        MarkEvidenceGap(
            NCLifecycleInterruptionDecision::EVIDENCE_NOTICE_OVERFLOW);
        return;
    }
    if (m_snapshot.feedbackSequenceGapDelta != 0ULL)
    {
        MarkEvidenceGap(
            NCLifecycleInterruptionDecision::EVIDENCE_SEQUENCE_GAP);
        return;
    }
    if (m_snapshot.ledgerIntegrityDelta != 0ULL)
    {
        MarkEvidenceGap(
            NCLifecycleInterruptionDecision::EVIDENCE_LEDGER_INTEGRITY);
        return;
    }

    if (m_snapshot.expectsEpochChange)
    {
        if (!m_snapshot.epochPublicationObserved)
        {
            SetWaitDecision(
                NCLifecycleInterruptionDecision::WAIT_EPOCH_PUBLICATION);
            return;
        }

        if (sample.executionEpoch != m_snapshot.publishedExecutionEpoch)
        {
            MarkSuperseded(
                NCLifecycleInterruptionDecision::EPOCH_SUPERSEDED);
            return;
        }
    }
    else if (sample.executionEpoch != m_snapshot.requestExecutionEpoch &&
        !m_snapshot.unexpectedEpochChangeObserved)
    {
        m_snapshot.unexpectedEpochChangeObserved = true;
        ++m_counters.unexpectedEpochChanges;
    }

    if (sample.activeBlocks != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_ACTIVE_BLOCKS);
        return;
    }
    if (sample.axisCommandDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_AXIS_COMMAND);
        return;
    }
    if (sample.axisResultDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_AXIS_RESULT);
        return;
    }
    if (sample.commandIngressDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_COMMAND_INGRESS);
        return;
    }
    if (sample.commandReplayDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_COMMAND_REPLAY);
        return;
    }
    if (sample.commandQueueDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_COMMAND_QUEUE);
        return;
    }
    if (sample.feedbackNoticeDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_FEEDBACK_NOTICE);
        return;
    }
    if (sample.feedbackDepth != 0U)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_FEEDBACK);
        return;
    }
    if (!m_snapshot.feedbackSequenceSynchronized)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_FEEDBACK_SEQUENCE);
        return;
    }
    if (sample.waitCallbackActive)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_CALLBACK);
        return;
    }
    if (sample.completionBindingActive)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_COMPLETION_BINDING);
        return;
    }
    if (sample.safetyOrRecoveryPending)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_SAFETY_REQUEST);
        return;
    }
    if (!sample.groupStandstill)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::WAIT_GROUP_STANDSTILL);
        return;
    }

    m_snapshot.quiescentReady = true;
    if (m_snapshot.stableSamples < m_snapshot.requiredStableSamples)
    {
        ++m_snapshot.stableSamples;
    }

    if (m_snapshot.stableSamples < m_snapshot.requiredStableSamples)
    {
        m_snapshot.phase =
            NCLifecycleInterruptionPhase::STABLE_CONFIRMATION;
        m_snapshot.decision =
            NCLifecycleInterruptionDecision::WAIT_STABLE_CONFIRMATION;
        ++m_counters.waitStableConfirmation;
        return;
    }

    m_snapshot.phase = NCLifecycleInterruptionPhase::QUIESCENT;
    m_snapshot.decision =
        NCLifecycleInterruptionDecision::QUIESCENT_PROVED;
    m_snapshot.active = false;
    m_snapshot.quiescent = true;
    ++m_counters.quiescent;
}

void NCLifecycleInterruptionBoundaryShadow::Supersede() noexcept
{
    MarkSuperseded(NCLifecycleInterruptionDecision::SUPERSEDED);
}
