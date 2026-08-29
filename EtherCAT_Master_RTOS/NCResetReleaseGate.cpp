#include "NCResetReleaseGate.h"

namespace
{
    bool IsValidSafetyLease(const MotionOwnerLease& lease) noexcept
    {
        return
            lease.IsValid() &&
            lease.owner == MotionOwner::SAFETY;
    }

    bool IsValidResetBoundaryForArm(
        const NCLifecycleInterruptionSnapshot& boundary,
        MotionExecutionEpoch expectedExecutionEpoch) noexcept
    {
        return
            boundary.sequence != 0ULL &&
            boundary.cause == NCLifecycleInterruptionCause::RESET &&
            boundary.expectsEpochChange &&
            boundary.epochPublicationObserved &&
            expectedExecutionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            boundary.publishedExecutionEpoch == expectedExecutionEpoch;
    }
}

std::uint64_t NCResetReleaseGate::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCResetReleaseGate::UpdateBoundary(
    const NCLifecycleInterruptionSnapshot& boundary,
    bool safetyLeaseCurrent) noexcept
{
    if (m_snapshot.boundarySequence == 0ULL)
    {
        m_snapshot.boundarySequence = boundary.sequence;
    }
    m_snapshot.boundaryPublishedExecutionEpoch =
        boundary.publishedExecutionEpoch;
    m_snapshot.boundaryCurrentExecutionEpoch =
        boundary.currentExecutionEpoch;
    m_snapshot.boundaryCurrentOwner = boundary.currentOwner;
    m_snapshot.boundaryCurrentOwnerGeneration =
        boundary.currentOwnerGeneration;
    m_snapshot.boundaryPhase = boundary.phase;
    m_snapshot.boundaryDecision = boundary.decision;
    m_snapshot.stableSamples = boundary.stableSamples;
    m_snapshot.requiredStableSamples = boundary.requiredStableSamples;

    m_snapshot.boundaryMatched =
        boundary.sequence != 0ULL &&
        boundary.sequence == m_snapshot.boundarySequence;
    m_snapshot.publishedEpochMatched =
        boundary.epochPublicationObserved &&
        boundary.publishedExecutionEpoch ==
        m_snapshot.expectedExecutionEpoch;
    m_snapshot.currentEpochMatched =
        boundary.currentExecutionEpoch ==
        m_snapshot.expectedExecutionEpoch;
    m_snapshot.safetyLeaseCurrent = safetyLeaseCurrent;
    m_snapshot.boundaryOwnerMatched =
        boundary.currentOwner == m_snapshot.safetyOwner &&
        boundary.currentOwnerGeneration ==
        m_snapshot.safetyOwnerGeneration;
    m_snapshot.groupStandstill = boundary.groupStandstill;
}

void NCResetReleaseGate::UpdateResetRebaseAck(
    const MotionNCResetRebaseAck& resetRebaseAck) noexcept
{
    m_snapshot.ackRequestSequence =
        resetRebaseAck.requestSequence;
    m_snapshot.ackExecutionEpoch =
        resetRebaseAck.executionEpoch;
    m_snapshot.ackOwner = resetRebaseAck.owner;
    m_snapshot.ackOwnerGeneration =
        resetRebaseAck.ownerGeneration;
    m_snapshot.ackRequestedAxisMask =
        resetRebaseAck.requestedAxisMask;
    m_snapshot.ackAppliedAxisMask =
        resetRebaseAck.appliedAxisMask;
    m_snapshot.ackPhase = resetRebaseAck.phase;

    m_snapshot.ackObserved =
        resetRebaseAck.requestSequence !=
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    m_snapshot.ackSequenceMatched =
        m_snapshot.ackObserved &&
        resetRebaseAck.requestSequence ==
        m_snapshot.expectedResetRequestSequence;
    m_snapshot.ackEpochMatched =
        resetRebaseAck.executionEpoch ==
        m_snapshot.expectedExecutionEpoch;
    m_snapshot.ackOwnerMatched =
        resetRebaseAck.owner == MotionOwner::SAFETY &&
        resetRebaseAck.owner == m_snapshot.safetyOwner &&
        resetRebaseAck.ownerGeneration ==
        m_snapshot.safetyOwnerGeneration;
    m_snapshot.ackRequestAccepted =
        resetRebaseAck.requestAccepted;
    m_snapshot.ackRebaseApplied =
        resetRebaseAck.rebaseApplied;
    m_snapshot.ackPostVerifyPassed =
        resetRebaseAck.postVerifyPassed;
    m_snapshot.ackAcknowledged =
        resetRebaseAck.acknowledged &&
        resetRebaseAck.acked &&
        resetRebaseAck.phase ==
        MotionNCResetRebasePhase::ACKNOWLEDGED;
    m_snapshot.ackPhaseAcknowledged =
        resetRebaseAck.phase ==
        MotionNCResetRebasePhase::ACKNOWLEDGED;
    m_snapshot.ackBlocked = resetRebaseAck.blocked;
    m_snapshot.ackSuperseded = resetRebaseAck.superseded;

    // The axis scope becomes part of the armed Reset transaction only after
    // a coherent RT publication proves the exact Request/Epoch/SAFETY lease.
    // Stale, mismatched, rejected or terminal-failure ACKs must never seed it.
    const bool blockedOrSupersededPhase =
        resetRebaseAck.phase == MotionNCResetRebasePhase::BLOCKED ||
        resetRebaseAck.phase == MotionNCResetRebasePhase::SUPERSEDED;
    const bool axisMaskLatchEligible =
        m_snapshot.ackSequenceMatched &&
        m_snapshot.ackEpochMatched &&
        m_snapshot.ackOwnerMatched &&
        resetRebaseAck.requestAccepted &&
        resetRebaseAck.requestedAxisMask != 0U &&
        !resetRebaseAck.blocked &&
        !resetRebaseAck.superseded &&
        !blockedOrSupersededPhase;

    if (!m_snapshot.ackAxisMaskLatched && axisMaskLatchEligible)
    {
        m_snapshot.expectedAckAxisMask =
            resetRebaseAck.requestedAxisMask;
        m_snapshot.ackAxisMaskLatched = true;
    }

    m_snapshot.ackRequestedAxisMaskMatched =
        m_snapshot.ackAxisMaskLatched &&
        resetRebaseAck.requestedAxisMask ==
        m_snapshot.expectedAckAxisMask;
    m_snapshot.ackAppliedAxisMaskMatched =
        m_snapshot.ackAxisMaskLatched &&
        resetRebaseAck.appliedAxisMask ==
        m_snapshot.expectedAckAxisMask;
    m_snapshot.ackAxisMaskChanged =
        m_snapshot.ackAxisMaskLatched &&
        !m_snapshot.ackRequestedAxisMaskMatched;
    m_snapshot.ackAxisMaskMatched =
        m_snapshot.ackRequestedAxisMaskMatched &&
        m_snapshot.ackAppliedAxisMaskMatched;

    m_snapshot.rebaseAckMatched =
        m_snapshot.ackSequenceMatched &&
        m_snapshot.ackEpochMatched &&
        m_snapshot.ackOwnerMatched &&
        m_snapshot.ackAxisMaskMatched &&
        m_snapshot.ackRequestAccepted &&
        m_snapshot.ackRebaseApplied &&
        m_snapshot.ackPostVerifyPassed &&
        m_snapshot.ackAcknowledged &&
        m_snapshot.ackPhaseAcknowledged &&
        !m_snapshot.ackBlocked &&
        !m_snapshot.ackSuperseded;
}

NCResetReleaseGate::ResetRebaseAckEvaluation
NCResetReleaseGate::EvaluateResetRebaseAck() const noexcept
{
    if (m_snapshot.expectedResetRequestSequence ==
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID)
    {
        return ResetRebaseAckEvaluation::MISMATCH;
    }

    if (!m_snapshot.ackObserved)
    {
        return ResetRebaseAckEvaluation::WAITING;
    }

    // The publication may still contain the preceding Reset ACK while the RT
    // task has not consumed this request.  A lower sequence is stale evidence,
    // not a failure of the newly armed transaction.
    if (m_snapshot.ackRequestSequence <
        m_snapshot.expectedResetRequestSequence)
    {
        return ResetRebaseAckEvaluation::WAITING;
    }

    if (!m_snapshot.ackSequenceMatched ||
        !m_snapshot.ackEpochMatched ||
        !m_snapshot.ackOwnerMatched)
    {
        return ResetRebaseAckEvaluation::MISMATCH;
    }

    // Once latched, neither side of the RT ACK may change the Reset scope.
    // appliedAxisMask==0 is permitted only while the rebase is still pending;
    // any non-zero different value is already contradictory evidence.
    if (m_snapshot.ackAxisMaskChanged ||
        (m_snapshot.ackAxisMaskLatched &&
            m_snapshot.ackAppliedAxisMask != 0ULL &&
            !m_snapshot.ackAppliedAxisMaskMatched))
    {
        return ResetRebaseAckEvaluation::MISMATCH;
    }

    if (m_snapshot.ackBlocked ||
        m_snapshot.ackSuperseded ||
        m_snapshot.ackPhase == MotionNCResetRebasePhase::BLOCKED ||
        m_snapshot.ackPhase == MotionNCResetRebasePhase::SUPERSEDED)
    {
        return ResetRebaseAckEvaluation::FAILED;
    }


    if (!m_snapshot.ackAxisMaskLatched)
    {
        // A completed/rebased ACK without a safely latched scope is malformed.
        // A pre-proof publication may still be waiting for RT acceptance.
        if (m_snapshot.ackRebaseApplied ||
            m_snapshot.ackPostVerifyPassed ||
            m_snapshot.ackAcknowledged ||
            m_snapshot.ackPhaseAcknowledged)
        {
            return ResetRebaseAckEvaluation::MISMATCH;
        }
        return ResetRebaseAckEvaluation::WAITING;
    }

    if (!m_snapshot.ackRequestAccepted ||
        !m_snapshot.ackRebaseApplied ||
        !m_snapshot.ackPostVerifyPassed ||
        !m_snapshot.ackAcknowledged ||
        !m_snapshot.ackPhaseAcknowledged)
    {
        // An ACK publication claiming completion without the required rebase
        // and post-verify evidence is a terminal failure, not a pending ACK.
        if (m_snapshot.ackAcknowledged ||
            m_snapshot.ackPhaseAcknowledged)
        {
            return ResetRebaseAckEvaluation::FAILED;
        }
        return ResetRebaseAckEvaluation::WAITING;
    }

    if (!m_snapshot.ackAxisMaskMatched)
    {
        return ResetRebaseAckEvaluation::MISMATCH;
    }

    return m_snapshot.rebaseAckMatched
        ? ResetRebaseAckEvaluation::MATCHED
        : ResetRebaseAckEvaluation::MISMATCH;
}

void NCResetReleaseGate::Block(
    NCResetReleaseGateDecision decision) noexcept
{
    if (m_snapshot.blocked)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.releaseReady = false;
    m_snapshot.blocked = true;
    m_snapshot.phase = NCResetReleaseGatePhase::BLOCKED;
    m_snapshot.decision = decision;

    switch (decision)
    {
    case NCResetReleaseGateDecision::INVALID_BOUNDARY:
        ++m_counters.blockedInvalidBoundary;
        break;
    case NCResetReleaseGateDecision::BOUNDARY_SEQUENCE_MISMATCH:
        ++m_counters.blockedBoundarySequence;
        break;
    case NCResetReleaseGateDecision::BOUNDARY_CAUSE_MISMATCH:
        ++m_counters.blockedBoundaryCause;
        break;
    case NCResetReleaseGateDecision::BOUNDARY_EVIDENCE_GAP:
        ++m_counters.blockedEvidenceGap;
        break;
    case NCResetReleaseGateDecision::BOUNDARY_SUPERSEDED:
        ++m_counters.blockedSuperseded;
        break;
    case NCResetReleaseGateDecision::BOUNDARY_INCOMPLETE:
        ++m_counters.blockedIncomplete;
        break;
    case NCResetReleaseGateDecision::EPOCH_MISMATCH:
        ++m_counters.blockedEpochMismatch;
        break;
    case NCResetReleaseGateDecision::SAFETY_LEASE_INVALID:
    case NCResetReleaseGateDecision::SAFETY_LEASE_LOST:
        ++m_counters.blockedSafetyLease;
        break;
    case NCResetReleaseGateDecision::OWNER_RELEASE_FAILED:
        ++m_counters.blockedOwnerRelease;
        break;
    case NCResetReleaseGateDecision::POST_INTERRUPTION_DISPATCH:
        ++m_counters.blockedPostInterruptionDispatch;
        break;
    case NCResetReleaseGateDecision::ACK_MISMATCH:
        ++m_counters.ackMismatch;
        break;
    case NCResetReleaseGateDecision::REBASE_FAILED:
        ++m_counters.rebaseFailed;
        break;
    default:
        break;
    }
}

bool NCResetReleaseGate::Arm(
    const NCLifecycleInterruptionSnapshot& boundary,
    MotionExecutionEpoch expectedExecutionEpoch,
    const MotionOwnerLease& safetyLease,
    MotionNCSettleRequestSequence expectedResetRequestSequence,
    bool safetyLeaseCurrent) noexcept
{
    ++m_counters.armAttempts;
    if (m_snapshot.active)
    {
        ++m_counters.supersededArms;
    }

    NCResetReleaseGateSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.expectedResetRequestSequence =
        expectedResetRequestSequence;
    snapshot.expectedExecutionEpoch = expectedExecutionEpoch;
    snapshot.safetyOwner = safetyLease.owner;
    snapshot.safetyOwnerGeneration = safetyLease.generation;
    snapshot.safetyLeaseValid = IsValidSafetyLease(safetyLease);
    snapshot.active = true;
    snapshot.phase = NCResetReleaseGatePhase::ARMED;
    snapshot.decision = NCResetReleaseGateDecision::CONTROL_ARMED;
    m_snapshot = snapshot;
    UpdateBoundary(boundary, safetyLeaseCurrent);

    if (!IsValidResetBoundaryForArm(
        boundary,
        expectedExecutionEpoch))
    {
        Block(NCResetReleaseGateDecision::INVALID_BOUNDARY);
        return false;
    }

    if (!m_snapshot.safetyLeaseValid)
    {
        Block(NCResetReleaseGateDecision::SAFETY_LEASE_INVALID);
        return false;
    }

    if (expectedResetRequestSequence ==
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID)
    {
        Block(NCResetReleaseGateDecision::ACK_MISMATCH);
        return false;
    }

    if (!safetyLeaseCurrent)
    {
        Block(NCResetReleaseGateDecision::SAFETY_LEASE_LOST);
        return false;
    }

    ++m_counters.armed;
    return true;
}

bool NCResetReleaseGate::IsMatchingQuiescenceProof(
    const NCLifecycleInterruptionSnapshot& boundary) const noexcept
{
    return
        boundary.sequence == m_snapshot.boundarySequence &&
        boundary.cause == NCLifecycleInterruptionCause::RESET &&
        boundary.phase == NCLifecycleInterruptionPhase::QUIESCENT &&
        boundary.decision ==
        NCLifecycleInterruptionDecision::QUIESCENT_PROVED &&
        boundary.quiescent &&
        !boundary.active &&
        !boundary.evidenceGap &&
        !boundary.superseded &&
        !boundary.postInterruptionDispatchObserved &&
        boundary.dispatchDelta == 0ULL &&
        boundary.terminalFeedbackLedgerAccepted &&
        boundary.ledgerIntegrityDelta == 0ULL &&
        boundary.feedbackOverflowDelta == 0ULL &&
        boundary.feedbackNoticeOverflowDelta == 0ULL &&
        boundary.feedbackSequenceGapDelta == 0ULL &&
        boundary.activeBlocks == 0U &&
        boundary.axisCommandDepth == 0U &&
        boundary.axisResultDepth == 0U &&
        boundary.commandQueueDepth == 0U &&
        boundary.commandIngressDepth == 0U &&
        boundary.commandReplayDepth == 0U &&
        boundary.feedbackDepth == 0U &&
        boundary.feedbackNoticeDepth == 0U &&
        boundary.feedbackSequenceSynchronized &&
        !boundary.safetyOrRecoveryPending &&
        !boundary.waitCallbackActive &&
        !boundary.completionBindingActive &&
        boundary.groupStandstill &&
        boundary.quiescentReady &&
        boundary.requiredStableSamples >= 2U &&
        boundary.stableSamples >= boundary.requiredStableSamples &&
        boundary.epochPublicationObserved &&
        boundary.publishedExecutionEpoch ==
        m_snapshot.expectedExecutionEpoch &&
        boundary.currentExecutionEpoch ==
        m_snapshot.expectedExecutionEpoch &&
        boundary.currentOwner == m_snapshot.safetyOwner &&
        boundary.currentOwnerGeneration ==
        m_snapshot.safetyOwnerGeneration &&
        m_snapshot.rebaseAckMatched;
}

void NCResetReleaseGate::ObserveBoundary(
    const NCLifecycleInterruptionSnapshot& boundary,
    const MotionNCResetRebaseAck& resetRebaseAck,
    bool safetyLeaseCurrent) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    ++m_counters.evaluations;

    const std::uint64_t expectedBoundarySequence =
        m_snapshot.boundarySequence;
    UpdateBoundary(boundary, safetyLeaseCurrent);
    UpdateResetRebaseAck(resetRebaseAck);

    if (boundary.sequence != expectedBoundarySequence)
    {
        Block(
            NCResetReleaseGateDecision::
            BOUNDARY_SEQUENCE_MISMATCH);
        return;
    }

    if (boundary.cause != NCLifecycleInterruptionCause::RESET)
    {
        Block(
            NCResetReleaseGateDecision::BOUNDARY_CAUSE_MISMATCH);
        return;
    }

    if (boundary.evidenceGap ||
        boundary.phase == NCLifecycleInterruptionPhase::EVIDENCE_GAP)
    {
        Block(
            NCResetReleaseGateDecision::BOUNDARY_EVIDENCE_GAP);
        return;
    }

    if (boundary.superseded ||
        boundary.phase == NCLifecycleInterruptionPhase::SUPERSEDED)
    {
        Block(
            NCResetReleaseGateDecision::BOUNDARY_SUPERSEDED);
        return;
    }

    if (boundary.postInterruptionDispatchObserved ||
        boundary.dispatchDelta != 0ULL)
    {
        Block(
            NCResetReleaseGateDecision::
            POST_INTERRUPTION_DISPATCH);
        return;
    }

    if (!m_snapshot.publishedEpochMatched ||
        !m_snapshot.currentEpochMatched)
    {
        Block(NCResetReleaseGateDecision::EPOCH_MISMATCH);
        return;
    }

    if (!m_snapshot.safetyLeaseValid ||
        !safetyLeaseCurrent ||
        !m_snapshot.boundaryOwnerMatched)
    {
        Block(NCResetReleaseGateDecision::SAFETY_LEASE_LOST);
        return;
    }

    const ResetRebaseAckEvaluation ackEvaluation =
        EvaluateResetRebaseAck();
    if (ackEvaluation == ResetRebaseAckEvaluation::MISMATCH)
    {
        Block(NCResetReleaseGateDecision::ACK_MISMATCH);
        return;
    }
    if (ackEvaluation == ResetRebaseAckEvaluation::FAILED)
    {
        Block(NCResetReleaseGateDecision::REBASE_FAILED);
        return;
    }
    if (ackEvaluation == ResetRebaseAckEvaluation::WAITING)
    {
        ++m_counters.waitRebaseAck;
        m_snapshot.phase =
            NCResetReleaseGatePhase::WAITING_QUIESCENCE;
        m_snapshot.decision =
            NCResetReleaseGateDecision::WAIT_REBASE_ACK;
        return;
    }

    if (IsMatchingQuiescenceProof(boundary))
    {
        if (!m_snapshot.releaseReady)
        {
            ++m_counters.releaseReady;
        }
        m_snapshot.quiescenceProved = true;
        m_snapshot.releaseReady = true;
        m_snapshot.phase = NCResetReleaseGatePhase::RELEASE_READY;
        m_snapshot.decision =
            NCResetReleaseGateDecision::READY_TO_RELEASE;
        return;
    }

    if (!boundary.active)
    {
        Block(NCResetReleaseGateDecision::BOUNDARY_INCOMPLETE);
        return;
    }

    ++m_counters.waitQuiescence;
    m_snapshot.phase = NCResetReleaseGatePhase::WAITING_QUIESCENCE;
    m_snapshot.decision =
        NCResetReleaseGateDecision::WAIT_QUIESCENCE_PROOF;
}

void NCResetReleaseGate::MarkReleaseResult(
    const NCLifecycleInterruptionSnapshot& boundary,
    const MotionNCResetRebaseAck& resetRebaseAck,
    bool safetyLeaseCurrent,
    bool releaseSucceeded) noexcept
{
    if (!ShouldReleaseSafetyOwner())
    {
        return;
    }

    ++m_counters.releaseAttempts;
    m_snapshot.releaseAttempted = true;

    const std::uint64_t expectedBoundarySequence =
        m_snapshot.boundarySequence;
    UpdateBoundary(boundary, safetyLeaseCurrent);
    UpdateResetRebaseAck(resetRebaseAck);

    const ResetRebaseAckEvaluation ackEvaluation =
        EvaluateResetRebaseAck();
    if (ackEvaluation == ResetRebaseAckEvaluation::FAILED)
    {
        Block(NCResetReleaseGateDecision::REBASE_FAILED);
        return;
    }
    if (ackEvaluation != ResetRebaseAckEvaluation::MATCHED)
    {
        Block(NCResetReleaseGateDecision::ACK_MISMATCH);
        return;
    }

    if (boundary.sequence != expectedBoundarySequence ||
        !IsMatchingQuiescenceProof(boundary))
    {
        Block(NCResetReleaseGateDecision::BOUNDARY_INCOMPLETE);
        return;
    }

    if (!safetyLeaseCurrent)
    {
        Block(NCResetReleaseGateDecision::SAFETY_LEASE_LOST);
        return;
    }

    if (!releaseSucceeded)
    {
        Block(NCResetReleaseGateDecision::OWNER_RELEASE_FAILED);
        return;
    }

    m_snapshot.active = false;
    m_snapshot.releaseReady = false;
    m_snapshot.releaseApplied = true;
    m_snapshot.phase = NCResetReleaseGatePhase::RELEASED;
    m_snapshot.decision = NCResetReleaseGateDecision::RELEASE_APPLIED;
    ++m_counters.released;
}
