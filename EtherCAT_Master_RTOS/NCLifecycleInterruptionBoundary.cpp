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

    bool IsFeedbackSequenceAfter(
        MotionFeedbackSequence candidate,
        MotionFeedbackSequence baseline) noexcept
    {
        if (candidate == MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            return false;
        }
        if (baseline == MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            return true;
        }

        // Motion feedback wraps from UINT64_MAX to 1.  A forward distance in
        // the lower half of the unsigned range is newer than the baseline.
        const MotionFeedbackSequence forwardDistance = candidate - baseline;
        return
            forwardDistance != 0ULL &&
            forwardDistance <=
            ((std::numeric_limits<MotionFeedbackSequence>::max)() / 2ULL);
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

bool NCLifecycleInterruptionBoundaryShadow::IsAlarmRequestTerminalCandidate(
    const MotionFeedbackEvent& event) const noexcept
{
    if (m_snapshot.cause != NCLifecycleInterruptionCause::ALARM ||
        m_snapshot.requestActiveBlocks == 0U ||
        m_snapshot.postInterruptionDispatchObserved ||
        !event.identity.IsAssigned() ||
        event.identity.epoch != m_snapshot.requestExecutionEpoch ||
        event.identity.source != ResolveMotionCommandSourceForOwner(
            m_snapshot.requestExecutionOwner) ||
        event.owner != m_snapshot.requestExecutionOwner ||
        event.ownerGeneration !=
        m_snapshot.requestExecutionOwnerGeneration ||
        !IsFeedbackSequenceAfter(
            event.sequence,
            m_baseline.lastPublishedFeedbackSequence))
    {
        return false;
    }

    const std::uint64_t candidateTotal = AddSaturating(
        m_alarmAbortCandidateCount,
        AddSaturating(
            m_alarmOwnerConflictRejectCandidateCount,
            m_alarmStaleEpochRejectCandidateCount));
    return candidateTotal < m_snapshot.requestActiveBlocks;
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
    const std::uint64_t previousExpectedAlarmAbortDelta =
        m_snapshot.expectedAlarmAbortDelta;
    const std::uint64_t previousExpectedAlarmPreReadRejectDelta =
        m_snapshot.expectedAlarmPreReadRejectDelta;
    const std::uint64_t previousExpectedAlarmOwnerConflictRejectDelta =
        m_snapshot.expectedAlarmOwnerConflictRejectDelta;
    const std::uint64_t previousExpectedAlarmStaleEpochRejectDelta =
        m_snapshot.expectedAlarmStaleEpochRejectDelta;

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
    m_snapshot.expectedAlarmAbortDelta = 0ULL;
    m_snapshot.expectedAlarmPreReadRejectDelta = 0ULL;
    m_snapshot.expectedAlarmOwnerConflictRejectDelta = 0ULL;
    m_snapshot.expectedAlarmStaleEpochRejectDelta = 0ULL;
    m_snapshot.unexpectedBlockFailureDelta =
        m_snapshot.blockFailureDelta;
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
    m_snapshot.unexpectedFeedbackRejectedDelta =
        m_snapshot.feedbackRejectedDelta;
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

    // A Motion Alarm invalidation intentionally reports ABORTED for the
    // executing command.  Commands already accepted into the NC lifecycle but
    // still waiting in producer/ingress/queue may instead be retired as
    // OWNER_CONFLICT (Safety took the lease first) or STALE_EPOCH (the RT epoch
    // seam won first).  Only exact old-Epoch, old-owner terminal candidates
    // published after this boundary and backed by the exact J.6 acknowledgement
    // may be reclassified.  Raw counters are never erased.
    if (m_snapshot.cause == NCLifecycleInterruptionCause::ALARM &&
        m_snapshot.alarmStopAcknowledged &&
        m_snapshot.runtimeAlarmEpochChangeObserved &&
        m_snapshot.terminalFeedbackLedgerAccepted)
    {
        const std::uint64_t expectedRejectCandidateTotal = AddSaturating(
            m_alarmOwnerConflictRejectCandidateCount,
            m_alarmStaleEpochRejectCandidateCount);
        const std::uint64_t expectedFailureCandidateTotal = AddSaturating(
            m_alarmAbortCandidateCount,
            expectedRejectCandidateTotal);

        m_snapshot.alarmTerminalClassificationValid =
            m_alarmAbortCandidateCount <=
            m_snapshot.feedbackAbortedDelta &&
            expectedRejectCandidateTotal <=
            m_snapshot.feedbackRejectedDelta &&
            expectedFailureCandidateTotal <=
            m_snapshot.blockFailureDelta &&
            expectedFailureCandidateTotal <=
            static_cast<std::uint64_t>(
                m_snapshot.requestActiveBlocks);

        if (m_snapshot.alarmTerminalClassificationValid)
        {
            m_snapshot.expectedAlarmAbortDelta =
                m_alarmAbortCandidateCount;
            m_snapshot.expectedAlarmOwnerConflictRejectDelta =
                m_alarmOwnerConflictRejectCandidateCount;
            m_snapshot.expectedAlarmStaleEpochRejectDelta =
                m_alarmStaleEpochRejectCandidateCount;
            m_snapshot.expectedAlarmPreReadRejectDelta =
                expectedRejectCandidateTotal;
            m_snapshot.unexpectedBlockFailureDelta =
                m_snapshot.blockFailureDelta -
                expectedFailureCandidateTotal;
            m_snapshot.unexpectedFeedbackRejectedDelta =
                m_snapshot.feedbackRejectedDelta -
                expectedRejectCandidateTotal;
        }

        m_snapshot.expectedAlarmAbortObserved =
            m_snapshot.expectedAlarmAbortDelta != 0ULL;
        m_snapshot.expectedAlarmPreReadRejectObserved =
            m_snapshot.expectedAlarmPreReadRejectDelta != 0ULL;

        if (m_snapshot.expectedAlarmAbortDelta >
            previousExpectedAlarmAbortDelta)
        {
            m_counters.expectedAlarmAborts = AddSaturating(
                m_counters.expectedAlarmAborts,
                m_snapshot.expectedAlarmAbortDelta -
                previousExpectedAlarmAbortDelta);
        }
        if (m_snapshot.expectedAlarmPreReadRejectDelta >
            previousExpectedAlarmPreReadRejectDelta)
        {
            m_counters.expectedAlarmPreReadRejects = AddSaturating(
                m_counters.expectedAlarmPreReadRejects,
                m_snapshot.expectedAlarmPreReadRejectDelta -
                previousExpectedAlarmPreReadRejectDelta);
        }
        if (m_snapshot.expectedAlarmOwnerConflictRejectDelta >
            previousExpectedAlarmOwnerConflictRejectDelta)
        {
            m_counters.expectedAlarmOwnerConflictRejects = AddSaturating(
                m_counters.expectedAlarmOwnerConflictRejects,
                m_snapshot.expectedAlarmOwnerConflictRejectDelta -
                previousExpectedAlarmOwnerConflictRejectDelta);
        }
        if (m_snapshot.expectedAlarmStaleEpochRejectDelta >
            previousExpectedAlarmStaleEpochRejectDelta)
        {
            m_counters.expectedAlarmStaleEpochRejects = AddSaturating(
                m_counters.expectedAlarmStaleEpochRejects,
                m_snapshot.expectedAlarmStaleEpochRejectDelta -
                previousExpectedAlarmStaleEpochRejectDelta);
        }
    }
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
    m_alarmAbortCandidateCount = 0ULL;
    m_alarmOwnerConflictRejectCandidateCount = 0ULL;
    m_alarmStaleEpochRejectCandidateCount = 0ULL;

    NCLifecycleInterruptionSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.cause = cause;
    snapshot.phase = NCLifecycleInterruptionPhase::REQUESTED;
    snapshot.decision = NCLifecycleInterruptionDecision::REQUEST_LATCHED;
    snapshot.requestExecutionEpoch = sample.executionEpoch;
    snapshot.currentExecutionEpoch = sample.executionEpoch;
    snapshot.requestOwner = sample.ownerLease.owner;
    snapshot.requestOwnerGeneration = sample.ownerLease.generation;
    snapshot.requestExecutionOwner = sample.executionOwnerLease.owner;
    snapshot.requestExecutionOwnerGeneration =
        sample.executionOwnerLease.generation;
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
    snapshot.alarmEpochClassificationPending =
        cause == NCLifecycleInterruptionCause::ALARM;
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

void NCLifecycleInterruptionBoundaryShadow::RecordAlarmStopAcknowledged(
    MotionExecutionEpoch appliedExecutionEpoch,
    bool epochChangeRequired) noexcept
{
    if (!m_snapshot.active ||
        m_snapshot.cause != NCLifecycleInterruptionCause::ALARM)
    {
        return;
    }

    if (!m_snapshot.alarmStopAcknowledged)
    {
        m_snapshot.alarmStopAcknowledged = true;
        m_snapshot.alarmEpochClassificationPending = false;
        ++m_counters.alarmStopAcknowledgements;
    }

    if (!epochChangeRequired)
    {
        return;
    }

    if (!m_snapshot.expectsEpochChange)
    {
        m_snapshot.expectsEpochChange = true;
        ++m_counters.epochPublicationsExpected;
    }

    if (appliedExecutionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        appliedExecutionEpoch == m_snapshot.requestExecutionEpoch)
    {
        m_snapshot.decision =
            NCLifecycleInterruptionDecision::WAIT_EPOCH_PUBLICATION;
        return;
    }

    m_snapshot.publishedExecutionEpoch = appliedExecutionEpoch;
    if (!m_snapshot.epochPublicationObserved)
    {
        m_snapshot.epochPublicationObserved = true;
        m_snapshot.runtimeAlarmEpochChangeObserved = true;
        ++m_counters.epochPublicationsObserved;
        ++m_counters.runtimeAlarmEpochChanges;
    }
    m_snapshot.phase = NCLifecycleInterruptionPhase::EPOCH_PUBLISHED;
    m_snapshot.decision =
        NCLifecycleInterruptionDecision::EPOCH_PUBLICATION_OBSERVED;
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

    if (ledgerAccepted &&
        event.errorCode == 0U &&
        IsAlarmRequestTerminalCandidate(event))
    {
        if (event.type == MotionFeedbackType::ABORTED &&
            event.rejectReason == MotionRejectReason::NONE)
        {
            ++m_alarmAbortCandidateCount;
        }
        else if (event.type == MotionFeedbackType::REJECTED &&
            event.rejectReason == MotionRejectReason::OWNER_CONFLICT)
        {
            ++m_alarmOwnerConflictRejectCandidateCount;
        }
        else if (event.type == MotionFeedbackType::REJECTED &&
            event.rejectReason == MotionRejectReason::STALE_EPOCH)
        {
            ++m_alarmStaleEpochRejectCandidateCount;
        }
    }

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
    case NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_ACKNOWLEDGEMENT:
        ++m_counters.waitAlarmStopAcknowledgement;
        break;
    case NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_TERMINAL:
        ++m_counters.waitAlarmStopTerminal;
        break;
    default:
        break;
    }
}

void NCLifecycleInterruptionBoundaryShadow::MarkAlarmStopClosed() noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.phase =
        NCLifecycleInterruptionPhase::ALARM_STOP_CLOSED;
    m_snapshot.decision =
        NCLifecycleInterruptionDecision::ALARM_STOP_CLOSED;
    m_snapshot.active = false;
    m_snapshot.alarmStopClosed = true;

    // This is an Alarm-stop lifecycle closure, not physical quiescence and
    // not Reset release permission.  Keep those semantic flags false.
    m_snapshot.quiescentReady = false;
    m_snapshot.quiescent = false;
    ++m_counters.alarmStopsClosed;
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

    // Alarm epoch ownership is determined by the exact 250 us emergency-stop
    // acknowledgement, not by inference from a changed runtime value.  Wait
    // for that evidence so a legitimate Motion invalidation is never counted
    // as an unexpected epoch change while the acknowledgement is in flight.
    if (m_snapshot.cause == NCLifecycleInterruptionCause::ALARM &&
        m_snapshot.alarmEpochClassificationPending &&
        !m_snapshot.alarmStopAcknowledged)
    {
        SetWaitDecision(
            NCLifecycleInterruptionDecision::
            WAIT_ALARM_STOP_ACKNOWLEDGEMENT);
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


    // J.6.3.1 closes the program lifecycle side of an acknowledged Alarm stop
    // once Ledger/transport evidence is drained and every raw failure is either
    // an exact ABORTED active command or an exact pre-read retirement produced
    // by that stop.  Callback, binding, SAFETY
    // request and physical standstill remain visible but belong to the later
    // Reset/recovery contract; they are deliberately not called quiescent.
    if (m_snapshot.cause == NCLifecycleInterruptionCause::ALARM &&
        m_snapshot.alarmStopAcknowledged)
    {
        const bool terminalContractMismatch =
            m_snapshot.postInterruptionDispatchObserved ||
            m_snapshot.unexpectedEpochChangeObserved ||
            !m_snapshot.alarmTerminalClassificationValid ||
            m_snapshot.unexpectedBlockFailureDelta != 0ULL ||
            m_snapshot.unexpectedFeedbackRejectedDelta != 0ULL ||
            m_snapshot.feedbackCancelledDelta != 0ULL ||
            m_snapshot.feedbackFaultedDelta != 0ULL ||
            m_snapshot.feedbackRejectedDelta !=
            m_snapshot.expectedAlarmPreReadRejectDelta ||
            m_snapshot.feedbackAbortedDelta !=
            m_snapshot.expectedAlarmAbortDelta;

        if (terminalContractMismatch)
        {
            SetWaitDecision(
                NCLifecycleInterruptionDecision::
                WAIT_ALARM_STOP_TERMINAL);
            return;
        }

        if (m_snapshot.stableSamples <
            m_snapshot.requiredStableSamples)
        {
            ++m_snapshot.stableSamples;
        }

        if (m_snapshot.stableSamples <
            m_snapshot.requiredStableSamples)
        {
            m_snapshot.phase =
                NCLifecycleInterruptionPhase::STABLE_CONFIRMATION;
            m_snapshot.decision =
                NCLifecycleInterruptionDecision::WAIT_ALARM_STOP_STABLE;
            ++m_counters.waitAlarmStopStable;
            return;
        }

        MarkAlarmStopClosed();
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
