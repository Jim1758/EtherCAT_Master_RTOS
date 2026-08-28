#include "NCBlockCompletionBoundary.h"


std::uint64_t NCBlockCompletionBoundaryObserver::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

bool NCBlockCompletionBoundaryObserver::IsMotionGuardEligible() const noexcept
{
    return
        m_activeWaitKind == NCBlockWaitKind::MOTION_HANDLER ||
        m_activeWaitKind == NCBlockWaitKind::MOTION_QUEUE_DRAIN;
}

void NCBlockCompletionBoundaryObserver::RecordFailClosedOnce() noexcept
{
    if (!m_failClosedRecorded)
    {
        ++m_counters.failClosedBindings;
        m_failClosedRecorded = true;
    }
}

void NCBlockCompletionBoundaryObserver::ResetBindingFlags() noexcept
{
    m_ledgerReadyBeforeLegacyRecorded = false;
    m_ledgerFailureRecorded = false;
    m_trackingOverflowRecorded = false;
    m_missingLifecycleRecorded = false;
    m_nonMotionRecorded = false;

    m_guardPendingBlockRecorded = false;
    m_guardFailureBlockRecorded = false;
    m_guardOverflowBlockRecorded = false;
    m_guardMissingBlockRecorded = false;
    m_guardNotTrackedBlockRecorded = false;
    m_failClosedRecorded = false;
}

void NCBlockCompletionBoundaryObserver::Bind(
    NCBlockDispatchId dispatchId,
    NCBlockWaitKind waitKind) noexcept
{
    if (dispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        waitKind == NCBlockWaitKind::NONE)
    {
        return;
    }

    if (m_activeDispatchId == dispatchId &&
        m_activeWaitKind == waitKind)
    {
        return;
    }

    if (HasActiveBinding())
    {
        ++m_counters.supersededBindings;
    }

    m_activeDispatchId = dispatchId;
    m_activeWaitKind = waitKind;
    ResetBindingFlags();

    ++m_counters.bindings;
    switch (waitKind)
    {
    case NCBlockWaitKind::MOTION_HANDLER:
    case NCBlockWaitKind::MOTION_QUEUE_DRAIN:
        ++m_counters.motionBindings;
        break;

    case NCBlockWaitKind::AUXILIARY_CALLBACK:
        ++m_counters.auxiliaryBindings;
        break;

    case NCBlockWaitKind::PROGRAM_FLOW_DRAIN:
        ++m_counters.programFlowBindings;
        break;

    case NCBlockWaitKind::NONE:
    default:
        break;
    }

    m_lastSnapshot = NCBlockCompletionBoundarySnapshot{};
    m_lastSnapshot.sequence = AllocateSequence();
    m_lastSnapshot.dispatchId = dispatchId;
    m_lastSnapshot.waitKind = waitKind;
    m_lastSnapshot.bound = true;
    m_lastSnapshot.guardEligible = IsMotionGuardEligible();
}

bool NCBlockCompletionBoundaryObserver::ObserveAndGate(
    bool hasLifecycle,
    const NCBlockMotionBoundarySnapshot& boundary,
    bool legacyReady) noexcept
{
    if (!HasActiveBinding())
    {
        return legacyReady;
    }

    ++m_counters.observations;
    ++m_counters.guardEvaluations;

    NCBlockCompletionComparison comparison =
        NCBlockCompletionComparison::NONE;

    const bool lifecycleMatches =
        hasLifecycle &&
        boundary.dispatchId == m_activeDispatchId;

    if (!lifecycleMatches)
    {
        comparison = NCBlockCompletionComparison::MISSING_LIFECYCLE;
        if (!m_missingLifecycleRecorded)
        {
            ++m_counters.missingLifecycle;
            m_missingLifecycleRecorded = true;
        }
    }
    else
    {
        switch (boundary.state)
        {
        case NCBlockMotionBoundaryState::NOT_TRACKED:
            comparison = NCBlockCompletionComparison::NOT_MOTION_TRACKED;
            if (!m_nonMotionRecorded)
            {
                ++m_counters.nonMotionWait;
                m_nonMotionRecorded = true;
            }
            break;

        case NCBlockMotionBoundaryState::TRACKING_OVERFLOW:
            comparison = NCBlockCompletionComparison::TRACKING_OVERFLOW;
            if (!m_trackingOverflowRecorded)
            {
                ++m_counters.trackingOverflow;
                m_trackingOverflowRecorded = true;
            }
            break;

        case NCBlockMotionBoundaryState::FAILED:
            comparison = legacyReady
                ? NCBlockCompletionComparison::LEGACY_READY_LEDGER_FAILED
                : NCBlockCompletionComparison::LEDGER_FAILED_LEGACY_WAITING;
            if (!m_ledgerFailureRecorded)
            {
                ++m_counters.ledgerFailureObserved;
                m_ledgerFailureRecorded = true;
            }
            if (legacyReady)
            {
                ++m_counters.releaseOnLedgerFailure;
            }
            break;

        case NCBlockMotionBoundaryState::SUCCEEDED:
            if (legacyReady)
            {
                comparison = NCBlockCompletionComparison::AGREE_READY;
                ++m_counters.agreeRelease;
            }
            else
            {
                comparison =
                    NCBlockCompletionComparison::LEDGER_READY_LEGACY_WAITING;
                if (!m_ledgerReadyBeforeLegacyRecorded)
                {
                    ++m_counters.ledgerReadyBeforeLegacy;
                    m_ledgerReadyBeforeLegacyRecorded = true;
                }
            }
            break;

        case NCBlockMotionBoundaryState::PENDING:
            if (legacyReady)
            {
                comparison =
                    NCBlockCompletionComparison::LEGACY_READY_LEDGER_PENDING;
                ++m_counters.legacyEarlyRelease;
            }
            else
            {
                comparison = NCBlockCompletionComparison::AGREE_WAITING;
                ++m_counters.agreeWaitingSamples;
            }
            break;

        case NCBlockMotionBoundaryState::NONE:
        default:
            comparison = NCBlockCompletionComparison::MISSING_LIFECYCLE;
            if (!m_missingLifecycleRecorded)
            {
                ++m_counters.missingLifecycle;
                m_missingLifecycleRecorded = true;
            }
            break;
        }
    }

    if (legacyReady)
    {
        ++m_counters.releaseChecks;
    }

    const bool guardEligible = IsMotionGuardEligible();
    bool effectiveReady = legacyReady;
    bool failClosed = false;
    NCBlockCompletionGateDecision gateDecision =
        NCBlockCompletionGateDecision::NONE;

    if (!guardEligible)
    {
        ++m_counters.guardBypassSamples;
        gateDecision = legacyReady
            ? NCBlockCompletionGateDecision::LEGACY_BYPASS_READY
            : NCBlockCompletionGateDecision::LEGACY_BYPASS_WAIT;
    }
    else
    {
        ++m_counters.guardEligibleSamples;

        if (!lifecycleMatches ||
            boundary.state == NCBlockMotionBoundaryState::NONE)
        {
            effectiveReady = false;
            failClosed = true;
            gateDecision =
                NCBlockCompletionGateDecision::BLOCK_MISSING_LIFECYCLE;

            if (!m_guardMissingBlockRecorded)
            {
                ++m_counters.blockedMissingLifecycle;
                m_guardMissingBlockRecorded = true;
            }
            RecordFailClosedOnce();
        }
        else
        {
            switch (boundary.state)
            {
            case NCBlockMotionBoundaryState::SUCCEEDED:
                effectiveReady = legacyReady;
                if (legacyReady)
                {
                    gateDecision =
                        NCBlockCompletionGateDecision::RELEASE_DUAL_KEY;
                    ++m_counters.dualKeyRelease;
                }
                else
                {
                    gateDecision =
                        NCBlockCompletionGateDecision::WAIT_LEGACY;
                }
                break;

            case NCBlockMotionBoundaryState::PENDING:
                effectiveReady = false;
                if (legacyReady)
                {
                    gateDecision =
                        NCBlockCompletionGateDecision::BLOCK_LEDGER_PENDING;
                    if (!m_guardPendingBlockRecorded)
                    {
                        ++m_counters.blockedLegacyEarly;
                        m_guardPendingBlockRecorded = true;
                    }
                }
                else
                {
                    gateDecision =
                        NCBlockCompletionGateDecision::WAIT_BOTH;
                }
                break;

            case NCBlockMotionBoundaryState::FAILED:
                effectiveReady = false;
                failClosed = true;
                gateDecision =
                    NCBlockCompletionGateDecision::BLOCK_LEDGER_FAILED;
                if (!m_guardFailureBlockRecorded)
                {
                    ++m_counters.blockedLedgerFailure;
                    m_guardFailureBlockRecorded = true;
                }
                RecordFailClosedOnce();
                break;

            case NCBlockMotionBoundaryState::TRACKING_OVERFLOW:
                effectiveReady = false;
                failClosed = true;
                gateDecision =
                    NCBlockCompletionGateDecision::BLOCK_TRACKING_OVERFLOW;
                if (!m_guardOverflowBlockRecorded)
                {
                    ++m_counters.blockedTrackingOverflow;
                    m_guardOverflowBlockRecorded = true;
                }
                RecordFailClosedOnce();
                break;

            case NCBlockMotionBoundaryState::NOT_TRACKED:
                effectiveReady = false;
                failClosed = true;
                gateDecision =
                    NCBlockCompletionGateDecision::BLOCK_NOT_TRACKED;
                if (!m_guardNotTrackedBlockRecorded)
                {
                    ++m_counters.blockedNotTracked;
                    m_guardNotTrackedBlockRecorded = true;
                }
                RecordFailClosedOnce();
                break;

            case NCBlockMotionBoundaryState::NONE:
            default:
                effectiveReady = false;
                failClosed = true;
                gateDecision =
                    NCBlockCompletionGateDecision::BLOCK_MISSING_LIFECYCLE;
                if (!m_guardMissingBlockRecorded)
                {
                    ++m_counters.blockedMissingLifecycle;
                    m_guardMissingBlockRecorded = true;
                }
                RecordFailClosedOnce();
                break;
            }
        }

        if (!effectiveReady)
        {
            ++m_counters.guardWaitSamples;
        }
    }

    UpdateLastSnapshot(
        lifecycleMatches,
        boundary,
        legacyReady,
        effectiveReady,
        guardEligible,
        failClosed,
        comparison,
        gateDecision);

    return effectiveReady;
}

void NCBlockCompletionBoundaryObserver::ClearBinding(
    bool superseded) noexcept
{
    if (!HasActiveBinding())
    {
        return;
    }

    if (superseded)
    {
        ++m_counters.supersededBindings;
    }

    m_activeDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    m_activeWaitKind = NCBlockWaitKind::NONE;
    ResetBindingFlags();
}

void NCBlockCompletionBoundaryObserver::UpdateLastSnapshot(
    bool hasLifecycle,
    const NCBlockMotionBoundarySnapshot& boundary,
    bool legacyReady,
    bool effectiveReady,
    bool guardEligible,
    bool failClosed,
    NCBlockCompletionComparison comparison,
    NCBlockCompletionGateDecision gateDecision) noexcept
{
    NCBlockCompletionBoundarySnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.dispatchId = m_activeDispatchId;
    snapshot.waitKind = m_activeWaitKind;
    snapshot.comparison = comparison;
    snapshot.gateDecision = gateDecision;
    snapshot.guardEligible = guardEligible;
    snapshot.guardApplied = guardEligible;
    snapshot.legacyReady = legacyReady;
    snapshot.effectiveReady = effectiveReady;
    snapshot.releaseObserved = legacyReady;
    snapshot.failClosed = failClosed;
    snapshot.bound = true;

    if (hasLifecycle)
    {
        snapshot.ledgerBoundary = boundary.state;
        snapshot.lifecycleState = boundary.lifecycleState;
        snapshot.motionSegmentCount = boundary.motionSegmentCount;
        snapshot.motionCompletedCount = boundary.motionCompletedCount;
        snapshot.motionTerminalCount = boundary.motionTerminalCount;
        snapshot.motionFailedCount = boundary.motionFailedCount;
    }

    m_lastSnapshot = snapshot;
}
