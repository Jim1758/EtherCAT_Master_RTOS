#include "NCFeedHoldBoundary.h"

namespace
{
    constexpr std::uint8_t FEED_HOLD_STABLE_SAMPLES_REQUIRED = 2U;
}

std::uint64_t NCFeedHoldBoundaryShadowObserver::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}

void NCFeedHoldBoundaryShadowObserver::UpdateSample(
    const NCFeedHoldBoundarySample& sample) noexcept
{
    m_snapshot.currentExecutionEpoch = sample.executionEpoch;
    m_snapshot.currentOwner = sample.ownerLease.owner;
    m_snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    m_snapshot.motion = sample.motion;
    m_snapshot.homeActive = sample.homeActive;
    m_snapshot.homeHoldDecelerating = sample.homeHoldDecelerating;
    m_snapshot.homePaused = sample.homePaused;
    m_snapshot.homeResumeRequested = sample.homeResumeRequested;
}

void NCFeedHoldBoundaryShadowObserver::BeginRequest(
    NCFeedHoldSource source,
    const NCFeedHoldBoundarySample& sample) noexcept
{
    ++m_counters.requestAttempts;

    if (source == NCFeedHoldSource::NONE)
    {
        return;
    }

    if (m_snapshot.active)
    {
        Cancel(true);
    }

    NCFeedHoldBoundarySnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.source = source;
    snapshot.phase = NCFeedHoldShadowPhase::REQUESTED;
    snapshot.decision = NCFeedHoldShadowDecision::REQUEST_LATCHED;
    snapshot.requestExecutionEpoch = sample.executionEpoch;
    snapshot.currentExecutionEpoch = sample.executionEpoch;
    snapshot.requestOwner = sample.ownerLease.owner;
    snapshot.requestOwnerGeneration = sample.ownerLease.generation;
    snapshot.currentOwner = sample.ownerLease.owner;
    snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    snapshot.expectedSettleRequestSequence =
        sample.expectedSettleRequestSequence;
    snapshot.dispatchId = sample.dispatchId;
    snapshot.requestPC = sample.activePC;
    snapshot.motion = sample.motion;
    snapshot.requiredStableSamples = FEED_HOLD_STABLE_SAMPLES_REQUIRED;
    snapshot.active = true;
    snapshot.requestLatched = true;
    snapshot.homeActive = sample.homeActive;
    snapshot.homeHoldDecelerating = sample.homeHoldDecelerating;
    snapshot.homePaused = sample.homePaused;
    snapshot.homeResumeRequested = sample.homeResumeRequested;
    snapshot.ownerLeaseValid = sample.ownerLease.IsValid();
    snapshot.executionEpochValid =
        sample.executionEpoch != MOTION_EXECUTION_EPOCH_INVALID;
    m_snapshot = snapshot;

    ++m_counters.requestsLatched;
    if (source == NCFeedHoldSource::PROGRAM)
    {
        ++m_counters.programRequests;
    }
    else if (source == NCFeedHoldSource::HOME)
    {
        ++m_counters.homeRequests;
    }
}

bool NCFeedHoldBoundaryShadowObserver::ValidateIdentity() noexcept
{
    m_snapshot.executionEpochValid =
        m_snapshot.requestExecutionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
        m_snapshot.currentExecutionEpoch == m_snapshot.requestExecutionEpoch;

    if (!m_snapshot.executionEpochValid)
    {
        Fail(NCFeedHoldShadowDecision::EPOCH_CHANGED);
        return false;
    }

    m_snapshot.ownerLeaseValid =
        m_snapshot.requestOwner != MotionOwner::NONE &&
        m_snapshot.requestOwnerGeneration != MOTION_OWNER_GENERATION_INVALID &&
        m_snapshot.currentOwner == m_snapshot.requestOwner &&
        m_snapshot.currentOwnerGeneration ==
        m_snapshot.requestOwnerGeneration;

    if (!m_snapshot.ownerLeaseValid)
    {
        Fail(NCFeedHoldShadowDecision::OWNER_CHANGED);
        return false;
    }

    return true;
}

void NCFeedHoldBoundaryShadowObserver::Fail(
    NCFeedHoldShadowDecision decision) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.phase = NCFeedHoldShadowPhase::FAILED;
    m_snapshot.decision = decision;
    m_snapshot.failed = true;
    m_snapshot.active = false;

    switch (decision)
    {
    case NCFeedHoldShadowDecision::OWNER_CHANGED:
        ++m_counters.ownerChanged;
        break;
    case NCFeedHoldShadowDecision::EPOCH_CHANGED:
        ++m_counters.executionEpochChanged;
        break;
    case NCFeedHoldShadowDecision::MOTION_FAULT:
        ++m_counters.motionFault;
        break;
    case NCFeedHoldShadowDecision::ACK_LOST:
        ++m_counters.acknowledgeLost;
        break;
    default:
        break;
    }
}

void NCFeedHoldBoundaryShadowObserver::Observe(
    const NCFeedHoldBoundarySample& sample) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    ++m_counters.evaluations;
    UpdateSample(sample);

    if (!ValidateIdentity())
    {
        return;
    }

    if (sample.motion.groupFaulted ||
        sample.motion.groupEmergencyStopped ||
        sample.motion.safetyOrRecoveryPending ||
        sample.motion.faultedAxes != 0U)
    {
        Fail(NCFeedHoldShadowDecision::MOTION_FAULT);
        return;
    }

    if (m_snapshot.acknowledged)
    {
        // -------------------------------------------------------------
        // NC-0.2I.2.3 - ACK latch semantics
        //
        // Physical stop acknowledgement is an event boundary. Once the
        // machine has satisfied the acquisition conditions and remained
        // stable for the required samples, the ACK must remain latched
        // until an explicit Resume, Reset, Owner/Epoch change or fault.
        //
        // currentActVel is derived from a single 250 us encoder delta. A
        // stationary closed-loop servo can therefore report an occasional
        // quantization / correction spike after ACK. Treating one such
        // sample as ACK_LOST produced a false failure even though:
        //
        //   - Feedrate Override remained zero
        //   - Command Velocity remained stopped
        //   - NC remained in HOLD
        //   - Owner / Epoch remained unchanged
        //
        // Actual Velocity remains visible in the diagnostic snapshot, but
        // after ACK it is advisory. ACK_LOST is reserved for loss of the
        // control-side hold condition without a recorded Resume request.
        // -------------------------------------------------------------
        if (m_snapshot.resumeRequested)
        {
            return;
        }

        const bool acknowledgeControlStillValid =
            (m_snapshot.source == NCFeedHoldSource::HOME)
            ? (sample.legacyHoldState && sample.homePaused)
            : (sample.legacyHoldState &&
                sample.motion.overrideZero &&
                sample.motion.commandStopped);

        if (!acknowledgeControlStillValid)
        {
            m_snapshot.acknowledgeLost = true;
            Fail(NCFeedHoldShadowDecision::ACK_LOST);
        }
        return;
    }

    bool stopCandidate = false;

    if (m_snapshot.source == NCFeedHoldSource::HOME)
    {
        stopCandidate = sample.homePaused;

        if (!stopCandidate)
        {
            m_snapshot.stableSamples = 0U;
            m_snapshot.stopCandidate = false;
            m_snapshot.phase = NCFeedHoldShadowPhase::DECELERATING;
            m_snapshot.decision = NCFeedHoldShadowDecision::WAIT_HOME_PAUSED;
            ++m_counters.waitHomePaused;
            return;
        }
    }
    else
    {
        if (!sample.motion.overrideZero)
        {
            m_snapshot.stableSamples = 0U;
            m_snapshot.stopCandidate = false;
            m_snapshot.phase = NCFeedHoldShadowPhase::DECELERATING;
            m_snapshot.decision = NCFeedHoldShadowDecision::WAIT_OVERRIDE_ZERO;
            ++m_counters.waitOverrideZero;
            return;
        }

        if (!sample.motion.commandStopped)
        {
            m_snapshot.stableSamples = 0U;
            m_snapshot.stopCandidate = false;
            m_snapshot.phase = NCFeedHoldShadowPhase::DECELERATING;
            m_snapshot.decision = NCFeedHoldShadowDecision::WAIT_COMMAND_STOP;
            ++m_counters.waitCommandStop;
            return;
        }

        const bool matchingRTSettleProof =
            m_snapshot.expectedSettleRequestSequence !=
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            sample.motion.settleRequestAccepted &&
            sample.motion.settleProofValid &&
            sample.motion.settleRequestSequence ==
            m_snapshot.expectedSettleRequestSequence &&
            sample.motion.settleProofSequence != 0ULL &&
            sample.motion.settleScopeMask != 0U &&
            sample.motion.settleRequiredCycles ==
            MOTION_NC_SETTLE_REQUIRED_CYCLES &&
            sample.motion.settleDwellCycles >=
            sample.motion.settleRequiredCycles &&
            sample.motion.ncSettled;

        if (!matchingRTSettleProof)
        {
            m_snapshot.stableSamples = 0U;
            m_snapshot.stopCandidate = false;
            m_snapshot.phase = NCFeedHoldShadowPhase::DECELERATING;
            m_snapshot.decision = NCFeedHoldShadowDecision::WAIT_ACTUAL_STOP;
            ++m_counters.waitActualStop;
            return;
        }

        stopCandidate = true;
    }

    m_snapshot.stopCandidate = stopCandidate;

    if (m_snapshot.stableSamples < m_snapshot.requiredStableSamples)
    {
        ++m_snapshot.stableSamples;
    }

    if (m_snapshot.stableSamples < m_snapshot.requiredStableSamples)
    {
        m_snapshot.phase = NCFeedHoldShadowPhase::STOPPED_UNSTABLE;
        m_snapshot.decision = NCFeedHoldShadowDecision::WAIT_STABLE;
        ++m_counters.waitStable;
        return;
    }

    m_snapshot.acknowledgeReady = true;
    m_snapshot.acknowledged = true;
    m_snapshot.phase = NCFeedHoldShadowPhase::ACKNOWLEDGED;
    m_snapshot.decision = NCFeedHoldShadowDecision::ACK_READY;
    ++m_counters.acknowledged;
}

void NCFeedHoldBoundaryShadowObserver::ObserveLegacyHoldEntered(
    const NCFeedHoldBoundarySample& sample) noexcept
{
    if (!m_snapshot.active || m_snapshot.legacyHoldEntered)
    {
        return;
    }

    UpdateSample(sample);
    m_snapshot.legacyHoldEntered = true;
    ++m_counters.legacyHolds;

    if (m_snapshot.acknowledged)
    {
        m_snapshot.legacyHoldAgreed = true;
        m_snapshot.decision = NCFeedHoldShadowDecision::LEGACY_HOLD_AGREE;
        ++m_counters.legacyAgreeHolds;
    }
    else
    {
        m_snapshot.legacyHoldWasEarly = true;
        m_snapshot.decision = NCFeedHoldShadowDecision::LEGACY_HOLD_EARLY;
        ++m_counters.legacyEarlyHolds;
    }
}

void NCFeedHoldBoundaryShadowObserver::ObserveResumeRequested(
    const NCFeedHoldBoundarySample& sample) noexcept
{
    if (!m_snapshot.active || m_snapshot.resumeRequested)
    {
        return;
    }

    UpdateSample(sample);
    m_snapshot.resumeRequested = true;
    m_snapshot.phase = NCFeedHoldShadowPhase::RESUME_REQUESTED;
    ++m_counters.resumeRequests;

    if (m_snapshot.acknowledged)
    {
        m_snapshot.resumeAfterAcknowledge = true;
        m_snapshot.decision = NCFeedHoldShadowDecision::RESUME_AFTER_ACK;
        ++m_counters.resumeAfterAcknowledge;
    }
    else
    {
        m_snapshot.resumeBeforeAcknowledge = true;
        m_snapshot.decision = NCFeedHoldShadowDecision::RESUME_BEFORE_ACK;
        ++m_counters.resumeBeforeAcknowledge;
    }
}

void NCFeedHoldBoundaryShadowObserver::ObserveResumeApplied(
    const NCFeedHoldBoundarySample& sample) noexcept
{
    if (!m_snapshot.active || m_snapshot.resumeApplied)
    {
        return;
    }

    UpdateSample(sample);
    m_snapshot.resumeApplied = true;
    m_snapshot.phase = NCFeedHoldShadowPhase::RESUMED;
    m_snapshot.decision = NCFeedHoldShadowDecision::RESUMED;
    m_snapshot.active = false;
    ++m_counters.resumed;
}

void NCFeedHoldBoundaryShadowObserver::Cancel(bool superseded) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.cancelled = true;
    m_snapshot.phase = NCFeedHoldShadowPhase::CANCELLED;

    if (superseded)
    {
        m_snapshot.decision = NCFeedHoldShadowDecision::SUPERSEDED;
        ++m_counters.superseded;
    }
    else
    {
        m_snapshot.decision = NCFeedHoldShadowDecision::CANCELLED;
        ++m_counters.cancelled;
    }
}
