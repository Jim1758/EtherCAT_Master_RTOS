#include "NCAlarmEmergencyStopBoundary.h"


NCAlarmEmergencyStopTrigger
NCAlarmEmergencyStopBoundaryShadow::ClassifyAlarmCode(
    std::int32_t alarmCode) noexcept
{
    switch (alarmCode)
    {
    case 1001:
        return NCAlarmEmergencyStopTrigger::EMERGENCY_STOP;
    case 3006:
        return NCAlarmEmergencyStopTrigger::AXIS_PROTECTION;
    case 3002:
    case 3016:
    case 3017:
        return NCAlarmEmergencyStopTrigger::HARD_LIMIT;
    case 3003:
    case 3005:
    case 3018:
        return NCAlarmEmergencyStopTrigger::DRIVE_FAULT;
    case 3004:
        return NCAlarmEmergencyStopTrigger::LAG_ERROR;
    default:
        break;
    }

    if (alarmCode >= 1000 && alarmCode < 2000)
    {
        return NCAlarmEmergencyStopTrigger::SYSTEM;
    }
    if (alarmCode >= 2000 && alarmCode < 3000)
    {
        return NCAlarmEmergencyStopTrigger::NC_PROGRAM;
    }
    if (alarmCode >= 3000 && alarmCode < 4000)
    {
        return NCAlarmEmergencyStopTrigger::AXIS;
    }
    if (alarmCode >= 4000 && alarmCode < 5000)
    {
        return NCAlarmEmergencyStopTrigger::EDM_PROCESS;
    }
    return alarmCode == 0
        ? NCAlarmEmergencyStopTrigger::NONE
        : NCAlarmEmergencyStopTrigger::UNKNOWN;
}


std::uint64_t NCAlarmEmergencyStopBoundaryShadow::AllocateSequence() noexcept
{
    std::uint64_t sequence = m_nextSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextSequence++;
    }
    return sequence;
}


bool NCAlarmEmergencyStopBoundaryShadow::HasExactEpochInvalidation(
    const NCAlarmEmergencyStopSample& sample) const noexcept
{
    if (!sample.emergencyEvidenceCoherent ||
        sample.emergency.publicationGeneration == 0ULL ||
        m_snapshot.requestExecutionEpoch ==
        MOTION_EXECUTION_EPOCH_INVALID ||
        sample.executionEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return false;
    }

    const MotionEmergencyStopEvidence& emergency = sample.emergency;

    const bool counterRecordValid =
        sample.emergencyCounters.rtApplications != 0ULL &&
        sample.emergencyCounters.epochInvalidations != 0ULL &&
        sample.epochInvalidation.invalidationCount ==
        sample.emergencyCounters.epochInvalidations &&
        sample.emergencyCounters.requestsPublished != 0ULL;

    const bool epochRecordValid =
        sample.epochInvalidation.fromExecutionEpoch ==
        m_snapshot.requestExecutionEpoch &&
        sample.epochInvalidation.toExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID &&
        sample.epochInvalidation.toExecutionEpoch !=
        m_snapshot.requestExecutionEpoch &&
        sample.epochInvalidation.toExecutionEpoch ==
        sample.executionEpoch &&
        emergency.currentExecutionEpoch == sample.executionEpoch &&
        emergency.lastAppliedExecutionEpoch == sample.executionEpoch;

    const bool ownerRecordValid =
        emergency.safetyLeaseMatchesLastApply &&
        sample.ownerLease.owner == MotionOwner::SAFETY &&
        sample.ownerLease.owner == emergency.currentOwner &&
        sample.ownerLease.generation ==
        emergency.currentOwnerGeneration &&
        sample.ownerLease.owner == emergency.lastAppliedOwner &&
        sample.ownerLease.generation ==
        emergency.lastAppliedOwnerGeneration;

    return counterRecordValid &&
        epochRecordValid &&
        ownerRecordValid;
}


void NCAlarmEmergencyStopBoundaryShadow::Begin(
    const NCAlarmEmergencyStopSample& sample) noexcept
{
    ++m_counters.requestAttempts;

    if (!sample.alarmActive)
    {
        return;
    }

    if (m_snapshot.active)
    {
        m_snapshot.active = false;
        m_snapshot.phase = NCAlarmEmergencyStopPhase::SUPERSEDED;
        m_snapshot.decision = NCAlarmEmergencyStopDecision::SUPERSEDED;
        m_snapshot.superseded = true;
        ++m_counters.superseded;
    }

    NCAlarmEmergencyStopSnapshot snapshot{};
    snapshot.sequence = AllocateSequence();
    snapshot.trigger = ClassifyAlarmCode(sample.alarmCode);
    snapshot.phase = NCAlarmEmergencyStopPhase::REQUESTED;
    snapshot.decision = NCAlarmEmergencyStopDecision::ALARM_LATCHED;
    snapshot.alarmUpdateCount = sample.alarmUpdateCount;
    snapshot.alarmCount = sample.alarmCount;
    snapshot.alarmCode = sample.alarmCode;
    snapshot.alarmAxisIndex = sample.alarmAxisIndex;
    snapshot.lifecycleSequence = sample.lifecycle.sequence;

    // Stage NC-0.2J.6.3.2: the generic Lifecycle boundary is opened first and
    // owns the pre-Alarm execution identity.  The 250 us consumer can run
    // between that Begin() and this Begin(), so sample.executionEpoch may
    // already be the post-E-stop Epoch.  Prefer the exact Lifecycle request
    // Epoch when both observers refer to the same active Alarm boundary.
    const bool lifecycleRequestEpochValid =
        sample.lifecycle.sequence != 0ULL &&
        sample.lifecycle.active &&
        sample.lifecycle.cause ==
        NCLifecycleInterruptionCause::ALARM &&
        sample.lifecycle.requestExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID;

    snapshot.requestExecutionEpoch = lifecycleRequestEpochValid
        ? sample.lifecycle.requestExecutionEpoch
        : sample.executionEpoch;
    snapshot.currentExecutionEpoch = sample.executionEpoch;
    snapshot.lastAppliedExecutionEpoch =
        sample.emergency.lastAppliedExecutionEpoch;
    snapshot.lastInvalidatedFromExecutionEpoch =
        sample.epochInvalidation.fromExecutionEpoch;
    snapshot.lastInvalidatedToExecutionEpoch =
        sample.epochInvalidation.toExecutionEpoch;
    snapshot.lastInvalidationCount =
        sample.epochInvalidation.invalidationCount;
    snapshot.requestOwner = sample.ownerLease.owner;
    snapshot.requestOwnerGeneration = sample.ownerLease.generation;
    snapshot.currentOwner = sample.ownerLease.owner;
    snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    snapshot.lastAppliedOwner = sample.emergency.lastAppliedOwner;
    snapshot.lastAppliedOwnerGeneration =
        sample.emergency.lastAppliedOwnerGeneration;
    snapshot.requestPublishedBaseline =
        sample.emergencyCounters.requestsPublished;
    snapshot.requestPublishedCurrent =
        sample.emergencyCounters.requestsPublished;
    snapshot.rtApplyBaseline =
        sample.emergencyCounters.rtApplications;
    snapshot.rtApplyCurrent =
        sample.emergencyCounters.rtApplications;
    snapshot.epochInvalidationBaseline =
        sample.emergencyCounters.epochInvalidations;
    snapshot.epochInvalidationCurrent =
        sample.emergencyCounters.epochInvalidations;
    snapshot.axisScopeCaptured =
        sample.emergencyEvidenceCoherent &&
        sample.emergency.publicationGeneration != 0ULL;
    snapshot.expectedAxisMask = snapshot.axisScopeCaptured
        ? sample.emergency.existingAxisMask
        : 0U;
    snapshot.currentAxisMask =
        sample.emergency.existingAxisMask;
    snapshot.activeBlocks = sample.lifecycle.activeBlocks;
    snapshot.commandQueueDepth = sample.lifecycle.commandQueueDepth;
    snapshot.commandIngressDepth = sample.lifecycle.commandIngressDepth;
    snapshot.commandReplayDepth = sample.lifecycle.commandReplayDepth;
    snapshot.feedbackDepth = sample.lifecycle.feedbackDepth;
    snapshot.feedbackNoticeDepth = sample.lifecycle.feedbackNoticeDepth;
    snapshot.active = true;
    snapshot.alarmActive = true;
    snapshot.hadExecutionToInvalidate =
        sample.emergency.groupActive ||
        sample.lifecycle.requestActiveBlocks != 0U ||
        sample.lifecycle.activeBlocks != 0U ||
        sample.lifecycle.commandQueueDepth != 0U ||
        sample.lifecycle.commandIngressDepth != 0U ||
        sample.lifecycle.commandReplayDepth != 0U;
    snapshot.epochChangeRequired =
        snapshot.hadExecutionToInvalidate;
    snapshot.emergencyEvidenceCoherent =
        sample.emergencyEvidenceCoherent;
    snapshot.feedbackSequenceSynchronized =
        sample.lifecycle.feedbackSequenceSynchronized;
    snapshot.lifecycleEvidenceGap =
        sample.lifecycle.evidenceGap;
    snapshot.postAlarmDispatchObserved =
        sample.lifecycle.postInterruptionDispatchObserved;
    m_snapshot = snapshot;

    // A real E-stop invalidation may already have completed after the generic
    // Lifecycle sample but before this J.6 baseline.  Accept it only when the
    // persistent RT record proves the exact Lifecycle old Epoch -> current
    // Epoch transition under the current SAFETY lease and its application /
    // invalidation sequence is already covered by this baseline.
    const bool exactPreLatchedInvalidation =
        m_snapshot.epochChangeRequired &&
        HasExactEpochInvalidation(sample) &&
        m_snapshot.rtApplyBaseline != 0ULL &&
        m_snapshot.epochInvalidationBaseline != 0ULL &&
        m_snapshot.lastInvalidationCount ==
        m_snapshot.epochInvalidationBaseline;

    if (exactPreLatchedInvalidation)
    {
        m_snapshot.preLatchedRTApplication = true;
        m_snapshot.epochInvalidationCorrelated = true;
        m_snapshot.emergencyRequestObserved = true;
        m_snapshot.rtApplyObserved = true;
        m_snapshot.executionEpochMatched = true;
        ++m_counters.preLatchedCorrelations;
    }

    ++m_counters.requestsLatched;
    switch (snapshot.trigger)
    {
    case NCAlarmEmergencyStopTrigger::EMERGENCY_STOP:
        ++m_counters.emergencyStopTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::AXIS_PROTECTION:
        ++m_counters.axisProtectionTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::HARD_LIMIT:
        ++m_counters.hardLimitTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::DRIVE_FAULT:
        ++m_counters.driveFaultTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::LAG_ERROR:
        ++m_counters.lagErrorTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::NC_PROGRAM:
        ++m_counters.ncProgramTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::EDM_PROCESS:
        ++m_counters.edmProcessTriggers;
        break;
    case NCAlarmEmergencyStopTrigger::SYSTEM:
    case NCAlarmEmergencyStopTrigger::AXIS:
    case NCAlarmEmergencyStopTrigger::UNKNOWN:
    case NCAlarmEmergencyStopTrigger::NONE:
    default:
        ++m_counters.otherTriggers;
        break;
    }
}


void NCAlarmEmergencyStopBoundaryShadow::UpdateSample(
    const NCAlarmEmergencyStopSample& sample) noexcept
{
    m_snapshot.alarmActive = sample.alarmActive;
    m_snapshot.currentExecutionEpoch = sample.executionEpoch;
    m_snapshot.currentOwner = sample.ownerLease.owner;
    m_snapshot.currentOwnerGeneration = sample.ownerLease.generation;
    m_snapshot.lastAppliedExecutionEpoch =
        sample.emergency.lastAppliedExecutionEpoch;
    m_snapshot.lastInvalidatedFromExecutionEpoch =
        sample.epochInvalidation.fromExecutionEpoch;
    m_snapshot.lastInvalidatedToExecutionEpoch =
        sample.epochInvalidation.toExecutionEpoch;
    m_snapshot.lastInvalidationCount =
        sample.epochInvalidation.invalidationCount;
    m_snapshot.motionPublicationGeneration =
        sample.emergency.publicationGeneration;
    m_snapshot.lastAppliedOwner =
        sample.emergency.lastAppliedOwner;
    m_snapshot.lastAppliedOwnerGeneration =
        sample.emergency.lastAppliedOwnerGeneration;
    m_snapshot.requestPublishedCurrent =
        sample.emergencyCounters.requestsPublished;
    m_snapshot.rtApplyCurrent =
        sample.emergencyCounters.rtApplications;
    m_snapshot.rtApplyDelta =
        m_snapshot.rtApplyCurrent >= m_snapshot.rtApplyBaseline
        ? m_snapshot.rtApplyCurrent - m_snapshot.rtApplyBaseline
        : 0ULL;
    m_snapshot.epochInvalidationCurrent =
        sample.emergencyCounters.epochInvalidations;

    m_snapshot.currentAxisMask =
        sample.emergency.existingAxisMask;
    m_snapshot.estopAxisMask =
        sample.emergency.estopAxisMask;
    m_snapshot.errorAxisMask =
        sample.emergency.errorAxisMask;
    m_snapshot.commandZeroAxisMask =
        sample.emergency.commandZeroAxisMask;
    m_snapshot.targetSealedAxisMask =
        sample.emergency.targetSealedAxisMask;
    m_snapshot.pdoSampledAxisMask =
        sample.emergency.pdoTargetVelocitySampledAxisMask;
    m_snapshot.pdoZeroAxisMask =
        sample.emergency.pdoTargetVelocityZeroAxisMask;

    m_snapshot.activeBlocks = sample.lifecycle.activeBlocks;
    m_snapshot.commandQueueDepth = sample.lifecycle.commandQueueDepth;
    m_snapshot.commandIngressDepth = sample.lifecycle.commandIngressDepth;
    m_snapshot.commandReplayDepth = sample.lifecycle.commandReplayDepth;
    m_snapshot.feedbackDepth = sample.lifecycle.feedbackDepth;
    m_snapshot.feedbackNoticeDepth = sample.lifecycle.feedbackNoticeDepth;

    m_snapshot.emergencyEvidenceCoherent =
        sample.emergencyEvidenceCoherent &&
        sample.emergency.publicationGeneration != 0ULL;
    const bool exactEpochInvalidation =
        m_snapshot.epochChangeRequired &&
        HasExactEpochInvalidation(sample);
    const bool postBaselineEpochInvalidation =
        exactEpochInvalidation &&
        m_snapshot.rtApplyCurrent >
        m_snapshot.rtApplyBaseline &&
        m_snapshot.epochInvalidationCurrent >
        m_snapshot.epochInvalidationBaseline;

    if (postBaselineEpochInvalidation)
    {
        m_snapshot.epochInvalidationCorrelated = true;
    }

    m_snapshot.emergencyRequestObserved =
        m_snapshot.emergencyRequestObserved ||
        m_snapshot.preLatchedRTApplication ||
        m_snapshot.requestPublishedCurrent >
        m_snapshot.requestPublishedBaseline ||
        sample.emergency.requestPending ||
        sample.emergency.requestInProgress ||
        m_snapshot.rtApplyDelta != 0ULL;
    m_snapshot.rtApplyObserved =
        m_snapshot.rtApplyObserved ||
        m_snapshot.preLatchedRTApplication ||
        m_snapshot.rtApplyDelta != 0ULL;

    const bool currentApplySafetyOwnerMatched =
        sample.emergency.safetyLeaseMatchesLastApply &&
        sample.ownerLease.owner == MotionOwner::SAFETY &&
        sample.ownerLease.owner == sample.emergency.lastAppliedOwner &&
        sample.ownerLease.generation ==
        sample.emergency.lastAppliedOwnerGeneration;

    m_snapshot.safetyOwnerMatched =
        currentApplySafetyOwnerMatched;
    m_snapshot.executionEpochMatched =
        !m_snapshot.epochChangeRequired ||
        m_snapshot.epochInvalidationCorrelated;
    m_snapshot.groupStopApplied =
        !sample.emergency.groupActive &&
        (sample.emergency.groupEmergencyStopped ||
            sample.emergency.groupError) &&
        sample.emergency.virtualCommandZero &&
        sample.emergency.virtualTargetSealed;
    m_snapshot.allExistingAxesSafe =
        sample.emergency.allExistingAxesSafe;
    m_snapshot.allExistingAxisCommandsZero =
        sample.emergency.allExistingAxisCommandsZero;
    m_snapshot.allExistingAxisTargetsSealed =
        sample.emergency.allExistingAxisTargetsSealed;
    m_snapshot.allSampledPdoTargetVelocitiesZero =
        sample.emergency.allSampledPdoTargetVelocitiesZero;
    m_snapshot.feedbackSequenceSynchronized =
        sample.lifecycle.feedbackSequenceSynchronized;
    m_snapshot.lifecycleEvidenceGap =
        sample.lifecycle.evidenceGap;
    m_snapshot.postAlarmDispatchObserved =
        sample.lifecycle.postInterruptionDispatchObserved;
}


void NCAlarmEmergencyStopBoundaryShadow::SetWait(
    NCAlarmEmergencyStopDecision decision) noexcept
{
    m_snapshot.phase = NCAlarmEmergencyStopPhase::WAITING_RT;
    m_snapshot.decision = decision;

    switch (decision)
    {
    case NCAlarmEmergencyStopDecision::WAIT_PUBLICATION:
        ++m_counters.waitPublication;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_REQUEST:
        ++m_counters.waitRequest;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_RT_APPLY:
        ++m_counters.waitRTApply;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_SAFETY_OWNER:
        ++m_counters.waitSafetyOwner;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_EXECUTION_EPOCH:
        ++m_counters.waitExecutionEpoch;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_GROUP_STOP:
        ++m_counters.waitGroupStop;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_AXIS_SAFE_STATE:
        ++m_counters.waitAxisSafeState;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_COMMAND_ZERO:
        ++m_counters.waitCommandZero;
        break;
    case NCAlarmEmergencyStopDecision::WAIT_TARGET_SEALED:
        ++m_counters.waitTargetSealed;
        break;
    default:
        break;
    }
}


void NCAlarmEmergencyStopBoundaryShadow::MarkEvidenceGap(
    NCAlarmEmergencyStopDecision decision) noexcept
{
    m_snapshot.active = false;
    m_snapshot.phase = NCAlarmEmergencyStopPhase::EVIDENCE_GAP;
    m_snapshot.decision = decision;
    m_snapshot.evidenceGap = true;

    switch (decision)
    {
    case NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_GAP:
        ++m_counters.lifecycleEvidenceGap;
        break;
    case NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_REPLACED:
        ++m_counters.lifecycleReplaced;
        break;
    case NCAlarmEmergencyStopDecision::EVIDENCE_AXIS_SCOPE_CHANGED:
        ++m_counters.axisScopeChanged;
        break;
    default:
        break;
    }
}


void NCAlarmEmergencyStopBoundaryShadow::Observe(
    const NCAlarmEmergencyStopSample& sample) noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    ++m_counters.evaluations;
    UpdateSample(sample);

    if (!sample.alarmActive)
    {
        m_snapshot.active = false;
        m_snapshot.phase = NCAlarmEmergencyStopPhase::CLEARED;
        m_snapshot.decision =
            NCAlarmEmergencyStopDecision::ALARM_CLEARED_BEFORE_ACK;
        m_snapshot.clearedBeforeAcknowledge = true;
        ++m_counters.clearedBeforeAcknowledge;
        return;
    }

    if (sample.lifecycle.sequence != m_snapshot.lifecycleSequence ||
        sample.lifecycle.cause != NCLifecycleInterruptionCause::ALARM)
    {
        MarkEvidenceGap(
            NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_REPLACED);
        return;
    }
    if (sample.lifecycle.evidenceGap ||
        sample.lifecycle.postInterruptionDispatchObserved)
    {
        MarkEvidenceGap(
            NCAlarmEmergencyStopDecision::EVIDENCE_LIFECYCLE_GAP);
        return;
    }
    if (!m_snapshot.emergencyEvidenceCoherent)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_PUBLICATION);
        return;
    }
    if (!m_snapshot.axisScopeCaptured)
    {
        m_snapshot.expectedAxisMask =
            sample.emergency.existingAxisMask;
        m_snapshot.axisScopeCaptured = true;
    }
    else if (sample.emergency.existingAxisMask !=
        m_snapshot.expectedAxisMask)
    {
        MarkEvidenceGap(
            NCAlarmEmergencyStopDecision::EVIDENCE_AXIS_SCOPE_CHANGED);
        return;
    }
    if (!m_snapshot.emergencyRequestObserved)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_REQUEST);
        return;
    }
    if (!m_snapshot.rtApplyObserved)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_RT_APPLY);
        return;
    }
    if (!m_snapshot.safetyOwnerMatched)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_SAFETY_OWNER);
        return;
    }
    if (!m_snapshot.executionEpochMatched)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_EXECUTION_EPOCH);
        return;
    }
    if (!m_snapshot.groupStopApplied)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_GROUP_STOP);
        return;
    }
    if (!m_snapshot.allExistingAxesSafe)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_AXIS_SAFE_STATE);
        return;
    }
    if (!m_snapshot.allExistingAxisCommandsZero)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_COMMAND_ZERO);
        return;
    }
    if (!m_snapshot.allExistingAxisTargetsSealed)
    {
        SetWait(NCAlarmEmergencyStopDecision::WAIT_TARGET_SEALED);
        return;
    }

    m_snapshot.active = false;
    m_snapshot.phase = NCAlarmEmergencyStopPhase::ACKNOWLEDGED;
    m_snapshot.decision =
        NCAlarmEmergencyStopDecision::STOP_ACKNOWLEDGED;
    m_snapshot.acknowledged = true;
    ++m_counters.acknowledged;
    if (m_snapshot.preLatchedRTApplication)
    {
        ++m_counters.preLatchedAcknowledged;
    }
}


void NCAlarmEmergencyStopBoundaryShadow::Supersede() noexcept
{
    if (!m_snapshot.active)
    {
        return;
    }

    m_snapshot.active = false;
    m_snapshot.phase = NCAlarmEmergencyStopPhase::SUPERSEDED;
    m_snapshot.decision = NCAlarmEmergencyStopDecision::SUPERSEDED;
    m_snapshot.superseded = true;
    ++m_counters.superseded;
}
