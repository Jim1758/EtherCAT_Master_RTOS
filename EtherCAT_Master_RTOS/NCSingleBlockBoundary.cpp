#include "NCSingleBlockBoundary.h"


std::uint64_t NCSingleBlockBoundaryShadow::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCSingleBlockBoundaryShadow::Touch() noexcept
{
    m_snapshot.sequence = AllocateSequence();
}

void NCSingleBlockBoundaryShadow::SetDecision(
    NCSingleBlockShadowPhase phase,
    NCSingleBlockShadowDecision decision) noexcept
{
    if (m_snapshot.phase == phase &&
        m_snapshot.decision == decision)
    {
        return;
    }

    m_snapshot.phase = phase;
    m_snapshot.decision = decision;
    Touch();
}

bool NCSingleBlockBoundaryShadow::Arm(
    const NCSingleBlockShadowArmRequest& request) noexcept
{
    ++m_counters.armAttempts;

    const bool targetValid =
        request.programTarget.scope != NCProgramScope::NONE &&
        request.programTarget.cacheGeneration !=
        NC_PROGRAM_CACHE_GENERATION_INVALID &&
        request.programTarget.sourcePC >= 0 &&
        (request.programTarget.scope != NCProgramScope::MACRO ||
            request.programTarget.frameId != NC_PROGRAM_FRAME_ID_INVALID);

    if (request.dispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        !targetValid ||
        request.candidateKind == NCSingleBlockCandidateKind::NONE)
    {
        return false;
    }

    if (m_snapshot.active)
    {
        Cancel(true);
    }

    NCSingleBlockShadowSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.dispatchId = request.dispatchId;
    snapshot.scope = request.programTarget.scope;
    snapshot.cacheGeneration = request.programTarget.cacheGeneration;
    snapshot.frameId = request.programTarget.frameId;
    snapshot.sourcePC = request.programTarget.sourcePC;
    snapshot.sourceLineNumber = request.sourceLineNumber;
    snapshot.candidateKind = request.candidateKind;
    snapshot.phase = NCSingleBlockShadowPhase::ARMED;
    snapshot.decision = NCSingleBlockShadowDecision::WAIT_LIFECYCLE;
    snapshot.active = true;
    snapshot.callbackRequired = request.callbackRequired;
    snapshot.callbackComplete = !request.callbackRequired;
    snapshot.transactionRequired = request.transactionRequired;
    snapshot.transactionComplete = !request.transactionRequired;
    snapshot.legacyPausePending = request.legacyPausePending;

    m_snapshot = snapshot;
    ++m_counters.armed;
    return true;
}

void NCSingleBlockBoundaryShadow::NoteNotEligible(
    NCBlockDispatchId dispatchId,
    const NCProgramCommitSnapshot& target,
    int sourceLineNumber) noexcept
{
    if (m_snapshot.active)
    {
        Cancel(true);
    }

    NCSingleBlockShadowSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.dispatchId = dispatchId;
    snapshot.scope = target.scope;
    snapshot.cacheGeneration = target.cacheGeneration;
    snapshot.frameId = target.frameId;
    snapshot.sourcePC = target.sourcePC;
    snapshot.sourceLineNumber = sourceLineNumber;
    snapshot.phase = NCSingleBlockShadowPhase::IDLE;
    snapshot.decision = NCSingleBlockShadowDecision::NOT_ELIGIBLE;
    snapshot.active = false;
    snapshot.legacyPausePending = true;

    m_snapshot = snapshot;
    ++m_counters.notEligible;
}

bool NCSingleBlockBoundaryShadow::Evaluate(
    const NCSingleBlockShadowSample& sample) noexcept
{
    if (!m_snapshot.active)
    {
        return false;
    }

    ++m_counters.evaluations;

    if (sample.programEndPending)
    {
        SuppressForProgramEnd();
        return false;
    }

    if (!sample.lifecycleFound || !sample.motionBoundary.IsValid())
    {
        ++m_counters.waitLifecycle;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::WAIT_LIFECYCLE);
        return false;
    }

    m_snapshot.motionState = sample.motionBoundary.state;
    m_snapshot.lifecycleState = sample.motionBoundary.lifecycleState;
    m_snapshot.programCommitted = sample.motionBoundary.programCommitted;

    if (!m_snapshot.programCommitted)
    {
        ++m_counters.waitProgramCommit;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::WAIT_PROGRAM_COMMIT);
        return false;
    }

    if (sample.transactionFailed)
    {
        if (!m_snapshot.motionFailed)
        {
            ++m_counters.transactionFailures;
        }
        m_snapshot.motionFailed = true;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::TRANSACTION_FAILED);
        return false;
    }

    if (m_snapshot.transactionRequired)
    {
        m_snapshot.transactionComplete =
            m_snapshot.transactionComplete || sample.transactionComplete;

        if (!m_snapshot.transactionComplete)
        {
            ++m_counters.waitTransaction;
            SetDecision(
                NCSingleBlockShadowPhase::WAITING_BOUNDARY,
                NCSingleBlockShadowDecision::WAIT_TRANSACTION);
            return false;
        }
    }

    if (m_snapshot.callbackRequired)
    {
        m_snapshot.callbackComplete =
            m_snapshot.callbackComplete || sample.callbackComplete;

        if (!m_snapshot.callbackComplete)
        {
            ++m_counters.waitCallback;
            SetDecision(
                NCSingleBlockShadowPhase::WAITING_BOUNDARY,
                NCSingleBlockShadowDecision::WAIT_CALLBACK);
            return false;
        }
    }

    switch (sample.motionBoundary.state)
    {
    case NCBlockMotionBoundaryState::NOT_TRACKED:
        m_snapshot.motionTracked = false;
        m_snapshot.motionComplete =
            sample.motionBoundary.lifecycleTerminal &&
            sample.motionBoundary.lifecycleSuccess;
        break;

    case NCBlockMotionBoundaryState::PENDING:
        m_snapshot.motionTracked = true;
        m_snapshot.motionComplete = false;
        ++m_counters.waitMotion;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::WAIT_MOTION);
        return false;

    case NCBlockMotionBoundaryState::SUCCEEDED:
        m_snapshot.motionTracked = true;
        m_snapshot.motionComplete = true;
        break;

    case NCBlockMotionBoundaryState::FAILED:
        if (!m_snapshot.motionFailed)
        {
            ++m_counters.motionFailures;
        }
        m_snapshot.motionTracked = true;
        m_snapshot.motionFailed = true;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::MOTION_FAILED);
        return false;

    case NCBlockMotionBoundaryState::TRACKING_OVERFLOW:
        if (!m_snapshot.motionFailed)
        {
            ++m_counters.trackingOverflow;
        }
        m_snapshot.motionTracked = true;
        m_snapshot.motionFailed = true;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::TRACKING_OVERFLOW);
        return false;

    case NCBlockMotionBoundaryState::NONE:
    default:
        ++m_counters.waitLifecycle;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::WAIT_LIFECYCLE);
        return false;
    }

    if (!m_snapshot.motionComplete)
    {
        ++m_counters.waitMotion;
        SetDecision(
            NCSingleBlockShadowPhase::WAITING_BOUNDARY,
            NCSingleBlockShadowDecision::WAIT_MOTION);
        return false;
    }

    if (!m_snapshot.boundaryReady)
    {
        m_snapshot.boundaryReady = true;
        ++m_counters.boundaryReady;
    }

    SetDecision(
        NCSingleBlockShadowPhase::BOUNDARY_READY,
        NCSingleBlockShadowDecision::READY_FOR_HOLD);
    return true;
}

void NCSingleBlockBoundaryShadow::ObserveLegacyHold(
    bool causedBySingleBlock) noexcept
{
    if (!causedBySingleBlock)
    {
        return;
    }

    ++m_counters.legacyHolds;

    if (!m_snapshot.active)
    {
        ++m_counters.legacyHoldWithoutArm;
        m_snapshot.legacyHoldObserved = true;
        SetDecision(
            NCSingleBlockShadowPhase::LEGACY_HOLD_MISMATCH,
            NCSingleBlockShadowDecision::LEGACY_HOLD_WITHOUT_ARM);
        return;
    }

    m_snapshot.legacyHoldObserved = true;

    if (m_snapshot.boundaryReady)
    {
        ++m_counters.agreeHolds;
        SetDecision(
            NCSingleBlockShadowPhase::LEGACY_HOLD_CONFIRMED,
            NCSingleBlockShadowDecision::AGREE_HOLD);
    }
    else
    {
        ++m_counters.legacyEarlyHolds;
        SetDecision(
            NCSingleBlockShadowPhase::LEGACY_HOLD_MISMATCH,
            NCSingleBlockShadowDecision::LEGACY_EARLY_HOLD);
    }
}

void NCSingleBlockBoundaryShadow::ObserveLegacyResume() noexcept
{
    if (!m_snapshot.legacyHoldObserved)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.legacyPausePending = false;
    ++m_counters.resumed;
    SetDecision(
        NCSingleBlockShadowPhase::IDLE,
        NCSingleBlockShadowDecision::RESUMED);
}

void NCSingleBlockBoundaryShadow::SuppressForProgramEnd() noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.programEndSuppressed = true;
    m_snapshot.legacyPausePending = false;
    ++m_counters.programEndSuppressed;
    SetDecision(
        NCSingleBlockShadowPhase::PROGRAM_END_SUPPRESSED,
        NCSingleBlockShadowDecision::PROGRAM_END_SUPPRESSED);
}

void NCSingleBlockBoundaryShadow::Cancel(bool superseded) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    if (superseded &&
        m_snapshot.boundaryReady &&
        !m_snapshot.legacyHoldObserved)
    {
        ++m_counters.legacyMissingHold;
    }

    m_snapshot.active = false;
    m_snapshot.legacyPausePending = false;
    ++m_counters.cancelled;
    if (superseded)
    {
        ++m_counters.superseded;
    }

    SetDecision(
        NCSingleBlockShadowPhase::CANCELLED,
        NCSingleBlockShadowDecision::CANCELLED);
}
