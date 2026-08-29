#include "NCSingleBlockHoldGate.h"

namespace
{
    bool IsValidControlledBoundary(
        const NCSingleBlockShadowSnapshot& boundary) noexcept
    {
        return
            boundary.sequence != 0ULL &&
            boundary.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            boundary.scope != NCProgramScope::NONE &&
            boundary.cacheGeneration !=
            NC_PROGRAM_CACHE_GENERATION_INVALID &&
            boundary.sourcePC >= 0 &&
            boundary.candidateKind !=
            NCSingleBlockCandidateKind::NONE;
    }


    bool MatchesControlledBoundary(
        const NCSingleBlockHoldGateSnapshot& gate,
        const NCSingleBlockShadowSnapshot& boundary) noexcept
    {
        return
            boundary.dispatchId == gate.dispatchId &&
            boundary.scope == gate.scope &&
            boundary.cacheGeneration == gate.cacheGeneration &&
            boundary.frameId == gate.frameId &&
            boundary.sourcePC == gate.sourcePC;
    }

    bool IsTransactionFailure(
        const NCSingleBlockShadowSnapshot& boundary) noexcept
    {
        return
            boundary.decision ==
            NCSingleBlockShadowDecision::TRANSACTION_FAILED;
    }

    bool IsTrackingOverflow(
        const NCSingleBlockShadowSnapshot& boundary) noexcept
    {
        return
            boundary.decision ==
            NCSingleBlockShadowDecision::TRACKING_OVERFLOW;
    }

    bool IsMotionFailure(
        const NCSingleBlockShadowSnapshot& boundary) noexcept
    {
        return
            boundary.motionFailed ||
            boundary.decision ==
            NCSingleBlockShadowDecision::MOTION_FAILED;
    }
}

std::uint64_t NCSingleBlockHoldGate::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCSingleBlockHoldGate::BeginSnapshot(
    const NCSingleBlockShadowSnapshot& boundary) noexcept
{
    NCSingleBlockHoldGateSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.enabled = m_enabled;
    snapshot.active = true;
    snapshot.phase = NCSingleBlockHoldGatePhase::ARMED;
    snapshot.decision =
        NCSingleBlockHoldGateDecision::CONTROL_ARMED;
    m_snapshot = snapshot;
    UpdateBoundary(boundary);
}

void NCSingleBlockHoldGate::UpdateBoundary(
    const NCSingleBlockShadowSnapshot& boundary) noexcept
{
    m_snapshot.boundarySequence = boundary.sequence;
    m_snapshot.dispatchId = boundary.dispatchId;
    m_snapshot.scope = boundary.scope;
    m_snapshot.cacheGeneration = boundary.cacheGeneration;
    m_snapshot.frameId = boundary.frameId;
    m_snapshot.candidateKind = boundary.candidateKind;
    m_snapshot.sourcePC = boundary.sourcePC;
    m_snapshot.sourceLineNumber = boundary.sourceLineNumber;
    m_snapshot.boundaryMatched =
        boundary.sequence != 0ULL &&
        boundary.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID;
    m_snapshot.boundaryReady = boundary.boundaryReady;
    m_snapshot.programEndSuppressed =
        boundary.programEndSuppressed ||
        boundary.decision ==
        NCSingleBlockShadowDecision::PROGRAM_END_SUPPRESSED;
}

void NCSingleBlockHoldGate::Block(
    NCSingleBlockHoldGateDecision decision) noexcept
{
    if (m_snapshot.blocked &&
        m_snapshot.decision == decision)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.holdReady = false;
    m_snapshot.blocked = true;
    m_snapshot.phase = NCSingleBlockHoldGatePhase::BLOCKED;
    m_snapshot.decision = decision;

    switch (decision)
    {
    case NCSingleBlockHoldGateDecision::BOUNDARY_MOTION_FAILED:
        ++m_counters.blockedMotionFailure;
        break;
    case NCSingleBlockHoldGateDecision::BOUNDARY_TRANSACTION_FAILED:
        ++m_counters.blockedTransactionFailure;
        break;
    case NCSingleBlockHoldGateDecision::BOUNDARY_TRACKING_OVERFLOW:
        ++m_counters.blockedTrackingOverflow;
        break;
    case NCSingleBlockHoldGateDecision::BOUNDARY_CANCELLED:
        ++m_counters.blockedCancelled;
        break;
    default:
        break;
    }
}

void NCSingleBlockHoldGate::SetEnabled(bool enabled) noexcept
{
    if (m_enabled == enabled)
    {
        return;
    }

    if (!enabled &&
        (m_snapshot.active || IsHoldApplied()))
    {
        ++m_counters.rollbackDisabled;
        m_snapshot.enabled = false;
        m_snapshot.decision =
            NCSingleBlockHoldGateDecision::ROLLBACK_DISABLED;

        // A HOLD that is already applied must remain resumable. A boundary
        // that has not yet applied HOLD is cancelled and may be converted to
        // the legacy path by NCManager.
        if (!m_snapshot.holdApplied)
        {
            m_snapshot.active = false;
            m_snapshot.cancelled = true;
            m_snapshot.holdReady = false;
            m_snapshot.phase =
                NCSingleBlockHoldGatePhase::CANCELLED;
        }
    }

    m_enabled = enabled;
    m_snapshot.enabled = enabled;
}

NCSingleBlockHoldGateRequestResult
NCSingleBlockHoldGate::RequestControl(
    const NCSingleBlockShadowSnapshot& boundary,
    bool explicitStopBypass) noexcept
{
    ++m_counters.requestAttempts;

    if (!m_enabled)
    {
        NCSingleBlockHoldGateSnapshot snapshot{};
        snapshot.sequence = AllocateSequence();
        snapshot.enabled = false;
        snapshot.phase = NCSingleBlockHoldGatePhase::BYPASSED;
        snapshot.decision =
            NCSingleBlockHoldGateDecision::LEGACY_BYPASS_DISABLED;
        m_snapshot = snapshot;
        UpdateBoundary(boundary);
        ++m_counters.legacyBypassDisabled;
        return NCSingleBlockHoldGateRequestResult::BYPASS_LEGACY;
    }

    if (explicitStopBypass)
    {
        NCSingleBlockHoldGateSnapshot snapshot{};
        snapshot.sequence = AllocateSequence();
        snapshot.enabled = true;
        snapshot.explicitStopBypass = true;
        snapshot.phase = NCSingleBlockHoldGatePhase::BYPASSED;
        snapshot.decision =
            NCSingleBlockHoldGateDecision::
            LEGACY_BYPASS_EXPLICIT_STOP;
        m_snapshot = snapshot;
        UpdateBoundary(boundary);
        ++m_counters.legacyBypassExplicitStop;
        return NCSingleBlockHoldGateRequestResult::BYPASS_LEGACY;
    }

    if (!IsValidControlledBoundary(boundary))
    {
        NCSingleBlockHoldGateSnapshot snapshot{};
        snapshot.sequence = AllocateSequence();
        snapshot.enabled = true;
        snapshot.blocked = true;
        snapshot.phase = NCSingleBlockHoldGatePhase::BLOCKED;
        snapshot.decision =
            NCSingleBlockHoldGateDecision::BOUNDARY_CANCELLED;
        m_snapshot = snapshot;
        UpdateBoundary(boundary);
        ++m_counters.blockedCancelled;
        return NCSingleBlockHoldGateRequestResult::BLOCKED;
    }

    if (m_snapshot.active || IsHoldApplied())
    {
        Cancel(true);
    }

    BeginSnapshot(boundary);
    ++m_counters.controlledArms;
    ObserveBoundary(boundary);

    return m_snapshot.blocked
        ? NCSingleBlockHoldGateRequestResult::BLOCKED
        : NCSingleBlockHoldGateRequestResult::CONTROLLED;
}

void NCSingleBlockHoldGate::ObserveBoundary(
    const NCSingleBlockShadowSnapshot& boundary) noexcept
{
    if (!m_snapshot.active || !m_enabled)
    {
        return;
    }

    if (!MatchesControlledBoundary(m_snapshot, boundary))
    {
        Cancel(true);
        return;
    }

    UpdateBoundary(boundary);

    if (m_snapshot.programEndSuppressed)
    {
        m_snapshot.active = false;
        m_snapshot.holdReady = false;
        m_snapshot.phase =
            NCSingleBlockHoldGatePhase::PROGRAM_END_SUPPRESSED;
        m_snapshot.decision =
            NCSingleBlockHoldGateDecision::PROGRAM_END_SUPPRESSED;
        ++m_counters.programEndSuppressed;
        return;
    }

    if (IsTrackingOverflow(boundary))
    {
        Block(
            NCSingleBlockHoldGateDecision::
            BOUNDARY_TRACKING_OVERFLOW);
        return;
    }

    if (IsTransactionFailure(boundary))
    {
        Block(
            NCSingleBlockHoldGateDecision::
            BOUNDARY_TRANSACTION_FAILED);
        return;
    }

    if (IsMotionFailure(boundary))
    {
        Block(
            NCSingleBlockHoldGateDecision::
            BOUNDARY_MOTION_FAILED);
        return;
    }

    if (boundary.phase == NCSingleBlockShadowPhase::CANCELLED ||
        boundary.decision == NCSingleBlockShadowDecision::CANCELLED)
    {
        Block(
            NCSingleBlockHoldGateDecision::BOUNDARY_CANCELLED);
        return;
    }

    if (boundary.boundaryReady)
    {
        if (!m_snapshot.holdReady)
        {
            m_snapshot.holdReady = true;
            ++m_counters.holdReady;
        }
        m_snapshot.phase = NCSingleBlockHoldGatePhase::HOLD_READY;
        m_snapshot.decision =
            NCSingleBlockHoldGateDecision::READY_TO_HOLD;
        return;
    }

    ++m_counters.waitBoundarySamples;
    m_snapshot.phase =
        NCSingleBlockHoldGatePhase::WAITING_BOUNDARY;
    m_snapshot.decision =
        NCSingleBlockHoldGateDecision::WAIT_BOUNDARY;
}

void NCSingleBlockHoldGate::MarkHoldApplied(
    const NCSingleBlockShadowSnapshot& boundary) noexcept
{
    if (!ShouldApplyHold())
    {
        return;
    }

    if (!MatchesControlledBoundary(m_snapshot, boundary) ||
        !boundary.boundaryReady)
    {
        Block(
            NCSingleBlockHoldGateDecision::BOUNDARY_CANCELLED);
        return;
    }

    UpdateBoundary(boundary);
    m_snapshot.holdReady = false;
    m_snapshot.holdApplied = true;
    m_snapshot.phase = NCSingleBlockHoldGatePhase::HOLD_APPLIED;
    m_snapshot.decision =
        NCSingleBlockHoldGateDecision::HOLD_APPLIED;
    ++m_counters.holdApplied;
}

void NCSingleBlockHoldGate::MarkResumeApplied() noexcept
{
    if (!IsHoldApplied())
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.holdReady = false;
    m_snapshot.resumeApplied = true;
    m_snapshot.phase = NCSingleBlockHoldGatePhase::RESUMED;
    m_snapshot.decision =
        NCSingleBlockHoldGateDecision::RESUME_APPLIED;
    ++m_counters.resumeApplied;
}

void NCSingleBlockHoldGate::Cancel(bool superseded) noexcept
{
    if (!m_snapshot.active && !IsHoldApplied())
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.holdReady = false;
    m_snapshot.cancelled = true;
    m_snapshot.phase = NCSingleBlockHoldGatePhase::CANCELLED;

    if (superseded)
    {
        m_snapshot.decision =
            NCSingleBlockHoldGateDecision::SUPERSEDED;
        ++m_counters.superseded;
    }
    else
    {
        m_snapshot.decision =
            NCSingleBlockHoldGateDecision::CANCELLED;
        ++m_counters.cancelled;
    }
}
