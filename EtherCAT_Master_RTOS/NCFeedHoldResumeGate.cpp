#include "NCFeedHoldResumeGate.h"

namespace
{
    bool IsProgramFeedHoldBoundary(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept
    {
        return
            boundary.sequence != 0ULL &&
            boundary.source == NCFeedHoldSource::PROGRAM &&
            boundary.requestLatched &&
            !boundary.resumeApplied;
    }
}

std::uint64_t NCFeedHoldResumeGate::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCFeedHoldResumeGate::BeginSnapshot(
    const NCFeedHoldBoundarySnapshot& boundary) noexcept
{
    NCFeedHoldResumeGateSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.enabled = m_enabled;
    snapshot.active = true;
    snapshot.resumeRequestLatched = true;
    m_snapshot = snapshot;
    UpdateBoundary(boundary);
}

void NCFeedHoldResumeGate::UpdateBoundary(
    const NCFeedHoldBoundarySnapshot& boundary) noexcept
{
    m_snapshot.boundarySequence = boundary.sequence;
    m_snapshot.source = boundary.source;
    m_snapshot.executionEpoch = boundary.requestExecutionEpoch;
    m_snapshot.owner = boundary.requestOwner;
    m_snapshot.ownerGeneration = boundary.requestOwnerGeneration;
    m_snapshot.dispatchId = boundary.dispatchId;
    m_snapshot.requestPC = boundary.requestPC;

    m_snapshot.boundaryActive = boundary.active;
    m_snapshot.boundaryAcknowledged = boundary.acknowledged;
    m_snapshot.boundaryFailed =
        boundary.failed || boundary.acknowledgeLost;
    m_snapshot.boundaryCancelled = boundary.cancelled;
    m_snapshot.acknowledgeObserved = boundary.acknowledged;
}

void NCFeedHoldResumeGate::Block(
    NCFeedHoldResumeGateDecision decision) noexcept
{
    m_snapshot.phase = NCFeedHoldResumeGatePhase::BLOCKED;
    m_snapshot.decision = decision;
    m_snapshot.active = false;
    m_snapshot.blocked = true;
    m_snapshot.releaseReady = false;

    if (decision == NCFeedHoldResumeGateDecision::BOUNDARY_FAILED)
    {
        ++m_counters.blockedBoundaryFailed;
    }
    else if (decision == NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED)
    {
        ++m_counters.blockedBoundaryCancelled;
    }
}

void NCFeedHoldResumeGate::SetEnabled(bool enabled) noexcept
{
    if (m_enabled == enabled)
    {
        return;
    }

    if (!enabled && m_snapshot.active)
    {
        m_snapshot.active = false;
        m_snapshot.cancelled = true;
        m_snapshot.releaseReady = false;
        m_snapshot.phase = NCFeedHoldResumeGatePhase::CANCELLED;
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::ROLLBACK_DISABLED;
        ++m_counters.rollbackDisabled;
    }

    m_enabled = enabled;
    m_snapshot.enabled = enabled;
}

NCFeedHoldResumeGateRequestResult NCFeedHoldResumeGate::RequestResume(
    const NCFeedHoldBoundarySnapshot& boundary) noexcept
{
    ++m_counters.requestAttempts;

    if (!m_enabled)
    {
        NCFeedHoldResumeGateSnapshot snapshot{};
        snapshot.sequence = AllocateSequence();
        snapshot.enabled = false;
        snapshot.phase = NCFeedHoldResumeGatePhase::BYPASSED;
        snapshot.decision =
            NCFeedHoldResumeGateDecision::LEGACY_BYPASS_DISABLED;
        m_snapshot = snapshot;
        UpdateBoundary(boundary);
        ++m_counters.legacyBypassDisabled;
        return NCFeedHoldResumeGateRequestResult::BYPASS_LEGACY;
    }

    if (!IsProgramFeedHoldBoundary(boundary))
    {
        NCFeedHoldResumeGateSnapshot snapshot{};
        snapshot.sequence = AllocateSequence();
        snapshot.enabled = true;
        snapshot.phase = NCFeedHoldResumeGatePhase::BYPASSED;
        snapshot.decision =
            NCFeedHoldResumeGateDecision::
            LEGACY_BYPASS_NOT_PROGRAM_FEED_HOLD;
        m_snapshot = snapshot;
        UpdateBoundary(boundary);
        ++m_counters.legacyBypassNotProgramFeedHold;
        return NCFeedHoldResumeGateRequestResult::BYPASS_LEGACY;
    }

    if (m_snapshot.active &&
        m_snapshot.boundarySequence == boundary.sequence)
    {
        ++m_counters.duplicateRequests;
        UpdateBoundary(boundary);

        if (boundary.failed || boundary.acknowledgeLost ||
            !boundary.ownerLeaseValid || !boundary.executionEpochValid)
        {
            Block(NCFeedHoldResumeGateDecision::BOUNDARY_FAILED);
            return NCFeedHoldResumeGateRequestResult::BLOCKED;
        }

        if (boundary.cancelled)
        {
            Block(NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED);
            return NCFeedHoldResumeGateRequestResult::BLOCKED;
        }

        if (m_snapshot.releaseReady || boundary.acknowledged)
        {
            m_snapshot.releaseReady = true;
            m_snapshot.phase = NCFeedHoldResumeGatePhase::RELEASE_READY;
            m_snapshot.decision =
                NCFeedHoldResumeGateDecision::
                DUPLICATE_RELEASE_READY_REQUEST;
            return NCFeedHoldResumeGateRequestResult::APPLY_NOW;
        }

        m_snapshot.phase = NCFeedHoldResumeGatePhase::DEFERRED;
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::DUPLICATE_DEFERRED_REQUEST;
        return NCFeedHoldResumeGateRequestResult::DEFERRED;
    }

    if (m_snapshot.active)
    {
        Cancel(true);
    }

    BeginSnapshot(boundary);

    if (boundary.failed || boundary.acknowledgeLost)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_FAILED);
        return NCFeedHoldResumeGateRequestResult::BLOCKED;
    }

    if (boundary.cancelled)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED);
        return NCFeedHoldResumeGateRequestResult::BLOCKED;
    }

    if (!boundary.ownerLeaseValid || !boundary.executionEpochValid)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_FAILED);
        return NCFeedHoldResumeGateRequestResult::BLOCKED;
    }

    if (boundary.acknowledged)
    {
        m_snapshot.acknowledgeObserved = true;
        m_snapshot.releaseReady = true;
        m_snapshot.phase = NCFeedHoldResumeGatePhase::RELEASE_READY;
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::APPLY_IMMEDIATE_AFTER_ACK;
        ++m_counters.immediateAfterAcknowledge;
        return NCFeedHoldResumeGateRequestResult::APPLY_NOW;
    }

    if (!boundary.active)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED);
        return NCFeedHoldResumeGateRequestResult::BLOCKED;
    }

    m_snapshot.deferredUntilAcknowledge = true;
    m_snapshot.phase = NCFeedHoldResumeGatePhase::DEFERRED;
    m_snapshot.decision =
        NCFeedHoldResumeGateDecision::DEFER_UNTIL_ACK;
    ++m_counters.deferredBeforeAcknowledge;
    return NCFeedHoldResumeGateRequestResult::DEFERRED;
}

void NCFeedHoldResumeGate::ObserveBoundary(
    const NCFeedHoldBoundarySnapshot& boundary) noexcept
{
    if (!m_snapshot.active || !m_enabled)
    {
        return;
    }

    if (boundary.sequence != m_snapshot.boundarySequence)
    {
        Cancel(true);
        return;
    }

    UpdateBoundary(boundary);

    if (boundary.failed || boundary.acknowledgeLost)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_FAILED);
        return;
    }

    if (boundary.cancelled)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_CANCELLED);
        return;
    }

    if (boundary.resumeApplied)
    {
        MarkResumeApplied(boundary);
        return;
    }

    if (!boundary.ownerLeaseValid || !boundary.executionEpochValid)
    {
        Block(NCFeedHoldResumeGateDecision::BOUNDARY_FAILED);
        return;
    }

    if (boundary.acknowledged && !m_snapshot.releaseReady)
    {
        m_snapshot.acknowledgeObserved = true;
        m_snapshot.releaseReady = true;
        m_snapshot.phase = NCFeedHoldResumeGatePhase::RELEASE_READY;
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::RELEASE_ON_ACK;
        ++m_counters.releaseOnAcknowledge;
    }
}

void NCFeedHoldResumeGate::MarkResumeApplied(
    const NCFeedHoldBoundarySnapshot& boundary) noexcept
{
    if (m_snapshot.resumeApplied)
    {
        return;
    }

    if (boundary.sequence != 0ULL)
    {
        UpdateBoundary(boundary);
    }

    m_snapshot.active = false;
    m_snapshot.releaseReady = false;
    m_snapshot.resumeApplied = true;
    m_snapshot.phase = NCFeedHoldResumeGatePhase::APPLIED;
    m_snapshot.decision =
        NCFeedHoldResumeGateDecision::RESUME_APPLIED;
    ++m_counters.resumeApplied;
}

void NCFeedHoldResumeGate::Cancel(bool superseded) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.cancelled = true;
    m_snapshot.releaseReady = false;
    m_snapshot.phase = NCFeedHoldResumeGatePhase::CANCELLED;

    if (superseded)
    {
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::SUPERSEDED;
        ++m_counters.superseded;
    }
    else
    {
        m_snapshot.decision =
            NCFeedHoldResumeGateDecision::CANCELLED;
        ++m_counters.cancelled;
    }
}
