// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "NCManager_GapInternal.h"

using namespace NCGapDetail;

bool NCManager::IsEDMZFixtureM30ContinuationSameThread() const noexcept
{
    const int pc = GetActiveDispatchPC();
    if (pc < 0 || pc > (std::numeric_limits<int>::max)() - 33) return false;
    for (int offset = 1; offset <= 32; ++offset)
    {
        const NCProgramCacheLine* line = GetBaseProgramCache().TryGetLine(pc + offset);
        if (!line) return false;
        const NCParsedBlock& next = line->parsedBlock;
        if (next.isEmpty) continue;
        if (next.isBlockSkip || next.dependsOnMacroState || next.error != NCParseError::NONE ||
            next.controlType != NCParsedControlType::NONE || next.gCount != 0 ||
            next.mCount != 1 || next.mExpressions[0] != "30" || next.duplicateAddressMask != 0U) return false;
        for (int i = 0; i < 26; ++i)
            if (next.hasParam[static_cast<std::size_t>(i)] && i != ('N' - 'A')) return false;
        return true;
    }
    return false;
}

WaitConditionFunc NCManager::StartEDMZFixtureSameThread(const NCBlock& block)
{
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_edmPhysicalServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM69_RETIREMENT_PENDING"); return nullptr; }
    const int line = m_gapDryRun.sourceLine;
    const bool speedStep = block.has('P') && block.val('P') == 14.0;
    const bool repeatedShort = block.has('P') && block.val('P') == 15.0;
    const bool gapEndurance = block.has('P') && (block.val('P') == 31.0 || block.val('P') == 32.0);
    const bool gapRapid = gapEndurance || (block.has('P') && (block.val('P') == 29.0 || block.val('P') == 30.0));
    const bool gapFeedUpdate = block.has('P') && (block.val('P') == 27.0 || block.val('P') == 28.0);
    const bool gapShortReplan = block.has('P') && (block.val('P') == 25.0 || block.val('P') == 26.0);
    const bool gapReplan = block.has('P') && block.val('P') == 24.0;
    const bool gapCurve = (block.has('P') && block.val('P') == 23.0) || gapReplan || gapShortReplan || gapFeedUpdate || gapRapid;
    const bool gapPersistentShort = block.has('P') && block.val('P') == 22.0;
    const bool gapShortRetreat = block.has('P') && block.val('P') == 21.0;
    const bool p20SpeedRetreat = block.has('P') && block.val('P') == 20.0;
    const bool nextSpeedRetreat = block.has('P') && block.val('P') == 19.0;
    const bool speedRetreat = block.has('P') && (block.val('P') == 18.0 || nextSpeedRetreat || p20SpeedRetreat || gapShortRetreat);
    const bool retreatShort = (block.has('P') && block.val('P') == 16.0) || speedRetreat;
    const bool persistentShort = (block.has('P') && block.val('P') == 17.0) || gapPersistentShort;
    const EDM28::Profile profile = gapEndurance ? (block.val('P') == 32.0 ? EDM28::Profile::P32GapEndurancePersistent : EDM28::Profile::P31GapEndurance) : gapRapid ? (block.val('P') == 30.0 ? EDM28::Profile::P30GapRapidPersistent : EDM28::Profile::P29GapRapid) : gapFeedUpdate ? (block.val('P') == 28.0 ? EDM28::Profile::P28GapFeedPersistent : EDM28::Profile::P27GapFeedUpdate) : gapShortReplan ? (block.val('P') == 26.0 ? EDM28::Profile::P26GapPersistentReplan : EDM28::Profile::P25GapShortReplan) : gapReplan ? EDM28::Profile::P24GapCurveReplan : gapCurve ? EDM28::Profile::P23GapCurveSegments : gapPersistentShort ? EDM28::Profile::P22GapPersistentShort : gapShortRetreat ? EDM28::Profile::P21GapShortRetreat : p20SpeedRetreat ? EDM28::Profile::P20UnloadedSpeedRetreat : nextSpeedRetreat ? EDM28::Profile::P19UnloadedSpeedRetreat : speedRetreat ? EDM28::Profile::P18UnloadedSpeedRetreat : persistentShort ? EDM28::Profile::P17PersistentShort : retreatShort ? EDM28::Profile::P16UnloadedRetreat : repeatedShort ? EDM28::Profile::P15UnloadedRepeat :
        speedStep ? EDM28::Profile::P14UnloadedSpeed : EDM28::Profile::P13Unloaded;
    // Attribute rejected fixed-profile preflight to its requested profile as well.
    m_edmZFixture.config.profile = profile;
    m_gapDryRun.physicalZFixtureTest = true;
    if (!IsGapDryRunBlockShapeValid(block) || !IsEDMZFixtureM30ContinuationSameThread())
    {
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, gapEndurance ? "P31_P32_REQUIRES_STANDALONE_M30" : gapRapid ? "P29_P30_REQUIRES_STANDALONE_M30" : gapFeedUpdate ? "P27_P28_REQUIRES_STANDALONE_M30" : gapShortReplan ? "P25_P26_REQUIRES_STANDALONE_M30" : gapReplan ? "P24_REQUIRES_STANDALONE_M30" : gapCurve ? "P23_REQUIRES_STANDALONE_M30" : gapPersistentShort ? "P22_REQUIRES_STANDALONE_M30" : gapShortRetreat ? "P21_REQUIRES_STANDALONE_M30" : p20SpeedRetreat ? "P20_REQUIRES_STANDALONE_M30" : nextSpeedRetreat ? "P19_REQUIRES_STANDALONE_M30" : speedRetreat ? "P18_REQUIRES_STANDALONE_M30" : persistentShort ? "P17_REQUIRES_STANDALONE_M30" : retreatShort ? "P16_REQUIRES_STANDALONE_M30" : repeatedShort ? "P15_REQUIRES_STANDALONE_M30" : speedStep ? "P14_REQUIRES_STANDALONE_M30" : "P13_REQUIRES_STANDALONE_M30");
        return nullptr;
    }
    if (!IsEDMShortFlushRunReadySameThread() || Homing.IsActive() || m_isG66Active || !m_macroStack.empty() ||
        m_pathFeed.pending || m_pathArc.pending || m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound ||
        !m_programMotionLease.IsValid() || m_programMotionLease.owner != MotionOwner::AUTO ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreLiveBookkeeping.currentRunToken == 0ULL || GetBaseProgramCache().GetGeneration() == 0ULL ||
        m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
        m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, gapEndurance ? "P31_P32_NOT_READY" : gapRapid ? "P29_P30_NOT_READY" : gapFeedUpdate ? "P27_P28_NOT_READY" : gapShortReplan ? "P25_P26_NOT_READY" : gapReplan ? "P24_NOT_READY" : gapCurve ? "P23_NOT_READY" : gapPersistentShort ? "P22_NOT_READY" : gapShortRetreat ? "P21_NOT_READY" : p20SpeedRetreat ? "P20_NOT_READY" : nextSpeedRetreat ? "P19_NOT_READY" : speedRetreat ? "P18_NOT_READY" : persistentShort ? "P17_NOT_READY" : retreatShort ? "P16_NOT_READY" : repeatedShort ? "P15_NOT_READY" : speedStep ? "P14_NOT_READY" : "P13_NOT_READY"); return nullptr;
    }
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.physicalZFixtureTest = true; m_gapDryRun.active = true;
    m_gapDryRun.sourceLine = line; m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    m_edmZFixture = EDMZFixtureState{};
    m_edmGapRapid = EDMGapRapidState{};
    m_edmGapRapid.shortState.active = gapRapid;
    m_edmGapRapid.shortState.persistent = EDM28::IsGapRapidPersistentProfile(profile);
    m_edmGapRapid.shortState.profile = profile;
    m_edmGapFeedUpdate = EDMGapFeedUpdateState{};
    m_edmGapFeedUpdate.shortState.active = gapFeedUpdate;
    m_edmGapFeedUpdate.shortState.persistent = profile == EDM28::Profile::P28GapFeedPersistent;
    m_edmGapFeedUpdate.shortState.profile = profile;
    m_edmGapShortReplan = EDMGapShortReplanState{};
    m_edmGapShortReplan.active = gapShortReplan;
    m_edmGapShortReplan.persistent = profile == EDM28::Profile::P26GapPersistentReplan;
    m_edmGapShortReplan.profile = profile;
    m_edmZFixture.config.profile = profile;
    m_edmZFixture.config.feedMmMin = EDM28::ProfileFeedMmMin(profile);
    m_edmZFixture.config.pdoCapMmS = EDM28::ProfilePdoCapMmS(profile);
    if (persistentShort)
    { m_edmZFixture.armNextPhase = EDMZFixturePhase::Negative;
      if (block.has('Q')) m_edmZFixture.operatorWindowQ = static_cast<std::uint8_t>(block.val('Q')); }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, gapEndurance ? "P31_P32_CLOCK_FREQUENCY" : gapRapid ? "P29_P30_CLOCK_FREQUENCY" : gapFeedUpdate ? "P27_P28_CLOCK_FREQUENCY" : gapShortReplan ? "P25_P26_CLOCK_FREQUENCY" : gapReplan ? "P24_CLOCK_FREQUENCY" : gapCurve ? "P23_CLOCK_FREQUENCY" : gapPersistentShort ? "P22_CLOCK_FREQUENCY" : gapShortRetreat ? "P21_CLOCK_FREQUENCY" : p20SpeedRetreat ? "P20_CLOCK_FREQUENCY" : nextSpeedRetreat ? "P19_CLOCK_FREQUENCY" : speedRetreat ? "P18_CLOCK_FREQUENCY" : persistentShort ? "P17_CLOCK_FREQUENCY" : retreatShort ? "P16_CLOCK_FREQUENCY" : repeatedShort ? "P15_CLOCK_FREQUENCY" : speedStep ? "P14_CLOCK_FREQUENCY" : "P13_CLOCK_FREQUENCY"); return nullptr; }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || !ReadEDMZFixtureFeedbackSameThread(false))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, gapEndurance ? "P31_P32_NO_COHERENT_SOURCE" : gapRapid ? "P29_P30_NO_COHERENT_SOURCE" : gapFeedUpdate ? "P27_P28_NO_COHERENT_SOURCE" : gapShortReplan ? "P25_P26_NO_COHERENT_SOURCE" : gapReplan ? "P24_NO_COHERENT_SOURCE" : gapCurve ? "P23_NO_COHERENT_SOURCE" : gapPersistentShort ? "P22_NO_COHERENT_SOURCE" : gapShortRetreat ? "P21_NO_COHERENT_SOURCE" : p20SpeedRetreat ? "P20_NO_COHERENT_SOURCE" : nextSpeedRetreat ? "P19_NO_COHERENT_SOURCE" : speedRetreat ? "P18_NO_COHERENT_SOURCE" : persistentShort ? "P17_NO_COHERENT_SOURCE" : retreatShort ? "P16_NO_COHERENT_SOURCE" : repeatedShort ? "P15_NO_COHERENT_SOURCE" : speedStep ? "P14_NO_COHERENT_SOURCE" : "P13_NO_COHERENT_SOURCE"); return nullptr; }
    const auto& feedback = m_edmZFixture.feedback;
    if (feedback.capHeld || !feedback.axisExists || !feedback.linear || !feedback.servoReady ||
        !feedback.modeReady || feedback.fault || feedback.hardPositive || feedback.hardNegative ||
        !std::isfinite(feedback.pulsePerMm) || feedback.pulsePerMm < 1000.0 ||
        !feedback.axisMapGeneration || !feedback.configGeneration)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, gapEndurance ? "P31_P32_PREFLIGHT_BLOCKED" : gapRapid ? "P29_P30_PREFLIGHT_BLOCKED" : gapFeedUpdate ? "P27_P28_PREFLIGHT_BLOCKED" : gapShortReplan ? "P25_P26_PREFLIGHT_BLOCKED" : gapReplan ? "P24_PREFLIGHT_BLOCKED" : gapCurve ? "P23_PREFLIGHT_BLOCKED" : gapPersistentShort ? "P22_PREFLIGHT_BLOCKED" : gapShortRetreat ? "P21_PREFLIGHT_BLOCKED" : p20SpeedRetreat ? "P20_PREFLIGHT_BLOCKED" : nextSpeedRetreat ? "P19_PREFLIGHT_BLOCKED" : speedRetreat ? "P18_PREFLIGHT_BLOCKED" : persistentShort ? "P17_PREFLIGHT_BLOCKED" : retreatShort ? "P16_PREFLIGHT_BLOCKED" : repeatedShort ? "P15_PREFLIGHT_BLOCKED" : speedStep ? "P14_PREFLIGHT_BLOCKED" : "P13_PREFLIGHT_BLOCKED"); return nullptr; }
    m_edmZFixture.scope.session = ++m_edmZFixtureSessionSerial;
    m_edmZFixture.scope.runGeneration = m_gapDryRun.run; m_edmZFixture.scope.cacheGeneration = m_gapDryRun.cache;
    m_edmZFixture.scope.dispatchGeneration = m_gapDryRun.dispatch;
    m_edmZFixture.scope.executionEpoch = m_gapDryRun.epoch;
    m_edmZFixture.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
    m_edmZFixture.scope.ownerGeneration = m_gapDryRun.lease.generation;
    m_edmZFixture.scope.axisMapGeneration = feedback.axisMapGeneration;
    m_edmZFixture.scope.configGeneration = feedback.configGeneration;
    m_edmZFixture.config.pulsePerMm = feedback.pulsePerMm;
    m_edmZFixture.config.referenceOffsetPulse = feedback.referenceOffsetPulse;
    m_edmZFixture.config.hardwareSign = feedback.hardwareSign;
    m_edmZFixture.config.positionToleranceMm = .001; m_edmZFixture.config.excursionToleranceMm = .001;
    if (gapCurve)
    {
        m_edmZFixture.gapCurveProfile = true;
        m_edmZFixture.gapCurveReplanProfile = gapReplan;
        if (gapReplan || gapShortReplan || gapFeedUpdate) m_edmZFixture.config.targetHalfMm = EDM40::TargetHalfMm;
        if (gapRapid)
        {
            m_edmZFixture.config.targetHalfMm = EDM43::TargetHalfMm;
            m_edmZFixture.config.outerHalfMm = .4;
            m_edmZFixture.config.accelerationTimeSec = m_edmZFixture.config.decelerationTimeSec = .3;
        }
        m_edmZFixture.gapCurveConfig = m_edmZFixture.config;
        m_edmZFixture.curveLastTick = feedback.sampledTick;
        m_edmZFixture.curveLastUs = feedback.monotonicUs;
    }
    if (gapPersistentShort)
    {
        m_edmZFixture.gapPersistentProfile = true;
        m_edmZFixture.gapPersistentConfig = m_edmZFixture.config;
        m_edmZFixture.gapPersistentLastTick = feedback.sampledTick;
        m_edmZFixture.gapPersistentLastUs = feedback.monotonicUs;
    }
    m_edmZFixture.lastFreshMs = m_edmZFixture.lastClockMs = m_edmZFixture.phaseStartMs = m_gapDryRun.lastServiceMs = now;
    LogEDMZFixtureSameThread("PREFLIGHT", "WAIT", "AWAIT_200_RT_IDLE_CYCLES");
    return WaitForGapDryRunCallback;
}

bool NCManager::IsEDMZFixtureAuthorityCurrentSameThread(bool allowHeld) noexcept
{
    const bool held = allowHeld && m_gapDryRun.paused;
    return m_mode == NCOperationMode::MEMORY && (m_state == NCState::RUN || (allowHeld && m_state == NCState::HOLD)) &&
        m_edmState != EDMState::ALARM && m_edmState != EDMState::NOT_READY &&
        m_externalReadyInterlock && !Close_System_Com_flag && !AlarmManager::GetInstance().HasAlarm() && !Homing.IsActive() &&
        m_resetContinuationPhase == ResetContinuationPhase::IDLE && !m_resetSafetyOutputHoldActive &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() && !m_isG66Active && m_macroStack.empty() &&
        !m_pathFeed.pending && !m_pathArc.pending && !m_pathReplay.pending && !m_pathHold.armed && !m_pathHold.bound &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() && m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_programMotionLease.owner == MotionOwner::AUTO && m_programMotionLease.IsValid() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) &&
        (held || (m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() && m_gapDryRun.lease.Matches(m_programMotionLease))) &&
        m_motion.IsGroupDone() && m_motion.GetCommandIngressSize() == 0U && m_motion.GetCommandReplaySize() == 0U;
}

bool NCManager::ReadEDMZFixtureFeedbackSameThread(bool requireNew) noexcept
{
    EDM28::Feedback feedback{};
    if (!m_motion.ReadEDMZFixtureFeedback(feedback) || !feedback.publicationSequence || !feedback.sampledTick ||
        !feedback.sourceFresh || !feedback.pdoValid || !feedback.contiguous || !feedback.clockValid ||
        !feedback.axisExists || !feedback.linear || !feedback.servoReady || !feedback.modeReady || feedback.fault ||
        feedback.hardPositive || feedback.hardNegative || !std::isfinite(feedback.actualMm) ||
        !std::isfinite(feedback.actualPulse) || !std::isfinite(feedback.commandPulse) || !std::isfinite(feedback.planningPulse) ||
        !std::isfinite(feedback.cmdSpeedMmS) || !std::isfinite(feedback.pdoSpeedMmS) || !std::isfinite(feedback.actualVelocityPps) ||
        !std::isfinite(feedback.feedrateOverride) || feedback.feedrateOverride < 0.0 || feedback.feedrateOverride > 1.0) return false;
    if (requireNew && (feedback.publicationSequence <= m_edmZFixture.publicationFloor || feedback.sampledTick <= m_edmZFixture.tickFloor)) return false;
    m_edmZFixture.feedback = feedback;
    if (m_edmZFixture.gapPersistentProfile && !m_edmZFixture.cleanupPending)
    {
        const bool armed = m_edmZFixture.originPinned || IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm);
        if (!IsEDMGapPersistentConfigValidSameThread(armed) ||
            feedback.publicationSequence < m_edmZFixture.publicationFloor ||
            feedback.sampledTick < m_edmZFixture.gapPersistentLastTick ||
            (feedback.sampledTick == m_edmZFixture.gapPersistentLastTick && feedback.monotonicUs != m_edmZFixture.gapPersistentLastUs) ||
            (feedback.sampledTick > m_edmZFixture.gapPersistentLastTick && feedback.monotonicUs <= m_edmZFixture.gapPersistentLastUs)) return false;
        m_edmZFixture.gapPersistentLastTick = feedback.sampledTick;
        m_edmZFixture.gapPersistentLastUs = feedback.monotonicUs;
    }
    if (m_edmZFixture.gapCurveProfile && !m_edmZFixture.cleanupPending)
    {
        const bool armed = m_edmZFixture.originPinned || IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm);
        if (!IsEDMGapCurveConfigValidSameThread(armed) || feedback.publicationSequence < m_edmZFixture.publicationFloor ||
            feedback.sampledTick < m_edmZFixture.curveLastTick ||
            (feedback.sampledTick == m_edmZFixture.curveLastTick && feedback.monotonicUs != m_edmZFixture.curveLastUs) ||
            (feedback.sampledTick > m_edmZFixture.curveLastTick && feedback.monotonicUs <= m_edmZFixture.curveLastUs)) return false;
        m_edmZFixture.curveLastTick = feedback.sampledTick; m_edmZFixture.curveLastUs = feedback.monotonicUs;
    }
    if (feedback.lastRequestSequence > m_edmZFixture.requestSequence) m_edmZFixture.requestSequence = feedback.lastRequestSequence;
    return true;
}

bool NCManager::SubmitEDMZFixtureSameThread(EDM28::RequestKind kind, double targetMm) noexcept
{
    if (m_edmZFixture.requestSequence == (std::numeric_limits<std::uint64_t>::max)()) return false;
    if (kind == EDM28::RequestKind::FeedUpdate &&
        (!m_edmGapFeedUpdate.shortState.active || !EDM28::IsGapFeedUpdateProfile(m_edmZFixture.config.profile))) return false;
    EDM28::Request request{};
    request.kind = kind; request.scope = m_edmZFixture.scope;
    request.sequence = ++m_edmZFixture.requestSequence;
    request.issueTick = m_edmZFixture.feedback.sampledTick;
    request.issueMonotonicUs = m_edmZFixture.feedback.monotonicUs;
    request.config = m_edmZFixture.config; request.relativeTargetMm = targetMm;
    if (m_edmZFixture.gapCurveProfile && (kind == EDM28::RequestKind::Position || kind == EDM28::RequestKind::FeedUpdate))
    {
        const auto& gap = m_edmZFixture.gapShort.Snapshot();
        const auto curve = m_edmGapRapid.shortState.active ? EDM43::Evaluate(m_edmZFixture.curveVoltage) : EDM39::Evaluate(m_edmZFixture.curveVoltage);
        const auto& shortState = m_edmGapRapid.shortState.active ? m_edmGapRapid.shortState : m_edmGapFeedUpdate.shortState.active ? m_edmGapFeedUpdate.shortState : m_edmGapShortReplan;
        const bool shortRetreat = kind == EDM28::RequestKind::Position && shortState.active && shortState.shortIntent &&
            shortState.entryProven && !shortState.pendingClear &&
            gap.voltage == 20.0 && gap.state == EDMGapServo::ShortState::Active && gap.shortActive &&
            targetMm == 0.0 && curve.limitedSpeedMmPerMin == (m_edmGapRapid.shortState.active ? -30.0 : -20.0);
        if (!IsEDMGapCurveConfigValidSameThread(true) || !curve.valid || !curve.limitedSpeedMmPerMin ||
            !gap.valid || !gap.fresh || ((gap.feedInhibited || gap.shortActive) && !shortRetreat) ||
            (shortState.active && shortState.shortIntent && !shortRetreat) || gap.fault != EDM37::GapShortFault::None ||
            gap.voltage != m_edmZFixture.curveVoltage || gap.sampleTick != request.issueTick || gap.sampleMonotonicUs != request.issueMonotonicUs)
            return false;
        request.curveSampleTick = gap.sampleTick; request.curveSampleUs = gap.sampleMonotonicUs;
        request.curveVoltage = gap.voltage; request.signedCurveMmMin = curve.limitedSpeedMmPerMin;
    }
    if (!m_motion.SubmitEDMZFixtureRequest(request)) return false;
    if (m_edmZFixture.gapCurveProfile && (kind == EDM28::RequestKind::Position || kind == EDM28::RequestKind::FeedUpdate))
    {
        m_edmZFixture.curveSequence = request.sequence;
        m_edmZFixture.curveSourceTick = request.curveSampleTick; m_edmZFixture.curveSourceUs = request.curveSampleUs;
        m_edmZFixture.curveSignedMmMin = request.signedCurveMmMin; m_edmZFixture.curveTargetMm = targetMm;
        m_edmZFixture.curveAppliedVoltage = request.curveVoltage;
        if (kind == EDM28::RequestKind::Position && (m_edmZFixture.gapCurveReplanProfile || m_edmGapShortReplan.active || m_edmGapFeedUpdate.shortState.active || m_edmGapRapid.shortState.active))
        {
            m_edmZFixture.replanLaunchActual = m_edmZFixture.feedback.actualPulse;
            m_edmZFixture.replanLaunchCommand = m_edmZFixture.feedback.commandPulse;
            m_edmZFixture.replanLaunchPlanning = m_edmZFixture.feedback.planningPulse;
            m_edmZFixture.replanLaunchCounted = false;
            if (m_edmGapRapid.shortState.active) m_edmGapRapid.speed = EDMGapRapidSpeedProof{};
            if (m_edmGapFeedUpdate.shortState.active)
            {
                auto& u = m_edmGapFeedUpdate;
                u.legUpdates = 0U; u.launchSequence = u.launchTick = u.launchUs = 0ULL;
                u.launchSourceTick = u.launchSourceUs = 0ULL;
                u.acceptedSequence = u.acceptedTick = u.acceptedUs = 0ULL;
                u.launchVoltage = u.launchMmMin = u.launchTargetMm = 0.0;
            }
        }
    }
    if (kind != EDM28::RequestKind::Heartbeat)
    {
        m_edmZFixture.pending = true; m_edmZFixture.pendingKind = kind;
        m_edmZFixture.pendingSequence = request.sequence; m_edmZFixture.appliedTick = 0ULL;
    }
    return true;
}

bool NCManager::IsEDMZFixtureAppliedSameThread(EDM28::RequestKind kind) const noexcept
{
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    return state.pending && state.pendingKind == kind && EDM28::SameScope(feedback.scope, state.scope) &&
        feedback.lastAppliedSequence == state.pendingSequence && feedback.lastAppliedKind == kind &&
        feedback.lastAppliedTick != 0ULL && feedback.lastAppliedTick <= feedback.sampledTick;
}

bool NCManager::IsEDMPersistentShortReleasedSameThread() const noexcept
{
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    const bool positionProof = state.endpointProofKind == 2U;
    const double tolerance = .001 * state.config.pulsePerMm;
    const auto& frozen = feedback.frozenConfig; const auto& config = state.config;
    const bool sameFrozen = frozen.axisIndex == config.axisIndex && frozen.pulsePerMm == config.pulsePerMm &&
        frozen.referenceOffsetPulse == config.referenceOffsetPulse && frozen.hardwareSign == config.hardwareSign &&
        frozen.outerHalfMm == config.outerHalfMm && frozen.targetHalfMm == config.targetHalfMm &&
        frozen.feedMmMin == config.feedMmMin && frozen.pdoCapMmS == config.pdoCapMmS &&
        frozen.accelerationTimeSec == config.accelerationTimeSec && frozen.decelerationTimeSec == config.decelerationTimeSec &&
        frozen.followingLimitMm == config.followingLimitMm && frozen.positionToleranceMm == config.positionToleranceMm &&
        frozen.excursionToleranceMm == config.excursionToleranceMm && frozen.heartbeatTicks == config.heartbeatTicks &&
        frozen.maximumIssueAgeTicks == config.maximumIssueAgeTicks && frozen.stopTimeoutTicks == config.stopTimeoutTicks &&
        frozen.settleCycles == config.settleCycles && frozen.profile == config.profile;
    return (state.config.profile == EDM28::Profile::P17PersistentShort || state.config.profile == EDM28::Profile::P22GapPersistentShort) && m_gapDryRun.paused &&
        feedback.publicationSequence && feedback.sampledTick && feedback.sourceFresh && feedback.pdoValid && feedback.contiguous &&
        feedback.clockValid && feedback.axisExists && feedback.linear && feedback.servoReady && feedback.modeReady &&
        feedback.modeValue == 9 && (feedback.statusWord & 0x006FU) == 0x0027U && !feedback.fault &&
        !feedback.hardPositive && !feedback.hardNegative && sameFrozen && EDM28::ValidScope(state.scope) &&
        feedback.axisMapGeneration == state.scope.axisMapGeneration && feedback.configGeneration == state.scope.configGeneration &&
        feedback.pulsePerMm == state.config.pulsePerMm && feedback.referenceOffsetPulse == state.config.referenceOffsetPulse &&
        feedback.hardwareSign == state.config.hardwareSign &&
        state.persistentShortLatched && state.shortSourceTest && (state.endpointProofKind == 1U || positionProof) &&
        state.endpointProofSequence && state.endpointAppliedSequence && state.endpointAppliedTick && state.endpointAppliedMonotonicUs &&
        state.cycles == 0U && state.simulatedShortStops == 1U && state.verifiedShortClears == 0U &&
        state.verifiedRetreats == (positionProof ? 1U : 0U) && state.originPinned &&
        EDM28::SameScope(feedback.scope, state.scope) && feedback.frameCurrent && feedback.outerOriginPulse == state.originPulse &&
        feedback.state == EDM28::State::Disarmed && !feedback.capHeld && feedback.stopProven && !feedback.latchedFault &&
        !feedback.forceZeroOutput && feedback.axisIdle && feedback.stableCycles >= state.config.settleCycles &&
        state.pendingKind == EDM28::RequestKind::Disarm && state.pendingSequence &&
        feedback.lastAppliedKind == EDM28::RequestKind::Disarm && feedback.lastAppliedSequence == state.pendingSequence &&
        feedback.proofSequence == state.receiptProof && feedback.proofSequence >= state.endpointProofSequence &&
        state.terminalStopAppliedSequence && state.terminalStopAppliedTick && state.terminalStopAppliedMonotonicUs &&
        feedback.stopAppliedSequence == state.terminalStopAppliedSequence && feedback.stopAppliedTick == state.terminalStopAppliedTick &&
        feedback.stopAppliedMonotonicUs == state.terminalStopAppliedMonotonicUs &&
        feedback.lastAppliedTick > state.terminalStopAppliedTick && feedback.lastAppliedMonotonicUs > state.terminalStopAppliedMonotonicUs &&
        feedback.sampledTick >= feedback.lastAppliedTick && feedback.monotonicUs >= feedback.lastAppliedMonotonicUs &&
        feedback.sampledTick >= state.terminalStopAppliedTick && feedback.sampledTick - state.terminalStopAppliedTick >= state.config.settleCycles &&
        feedback.monotonicUs >= state.terminalStopAppliedMonotonicUs &&
        feedback.monotonicUs - state.terminalStopAppliedMonotonicUs >= state.config.settleCycles * EDM28::CycleUs &&
        (!positionProof || (state.terminalStopAppliedTick > state.endpointAppliedTick &&
            state.terminalStopAppliedMonotonicUs > state.endpointAppliedMonotonicUs && feedback.proofSequence > state.endpointProofSequence)) &&
        std::isfinite(tolerance) && tolerance > 0.0 && std::isfinite(state.originPulse) && std::isfinite(feedback.targetPulse) &&
        std::isfinite(feedback.actualPulse) && std::isfinite(feedback.commandPulse) && std::isfinite(feedback.planningPulse) &&
        std::fabs(feedback.actualPulse - state.originPulse) <= tolerance && std::fabs(feedback.commandPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.planningPulse - state.originPulse) <= tolerance &&
        (!positionProof || std::fabs(feedback.targetPulse - state.originPulse) <= tolerance) &&
        std::isfinite(feedback.cmdSpeedMmS) && std::isfinite(feedback.pdoSpeedMmS) && std::isfinite(feedback.actualVelocityPps) &&
        std::isfinite(feedback.feedrateOverride) && feedback.feedrateOverride >= 0.0 && feedback.feedrateOverride <= 1.0 &&
        std::fabs(feedback.cmdSpeedMmS) <= 1.0 / state.config.pulsePerMm;
}

bool NCManager::IsEDMZFixtureStoppedSameThread() const noexcept
{
    if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile)) return IsEDMGapDynamicStoppedSameThread();
    // CycleStart consults this proof before changing HOLD to RUN. P23's
    // never-applied Arm and already-released terminal are stopped states too;
    // the process callback still owns fresh authority, source restart and the
    // strictly newer terminal receipt. This predicate changes no identity.
    if (m_edmZFixture.gapCurveProfile && m_gapDryRun.paused)
    {
        const auto& s = m_edmZFixture; const auto& f = s.feedback;
        if (s.curveTerminalHold && IsEDMGapCurveReleasedSameThread()) return true;
        if (s.curvePreArmHold)
        {
            const bool coherent = IsEDMGapCurveConfigValidSameThread(false) && f.publicationSequence && f.sampledTick &&
                f.monotonicUs && f.sourceFresh && f.pdoValid && f.contiguous && f.clockValid && f.axisExists && f.linear &&
                f.servoReady && f.modeReady && f.modeValue == 9 && (f.statusWord & 0x006FU) == 0x0027U &&
                !f.fault && !f.hardPositive && !f.hardNegative &&
                (!f.latchedFault || (s.gapCurveReplanProfile && IsEDMGapReplanRetiredFaultSameThread()) ||
                    (m_edmGapShortReplan.active && IsEDMGapShortReplanRetiredFaultSameThread()) ||
                    (m_edmGapFeedUpdate.shortState.active && IsEDMGapFeedUpdateRetiredFaultSameThread()) ||
                    (m_edmGapRapid.shortState.active && IsEDMGapRapidRetiredFaultSameThread())) && !f.forceZeroOutput &&
                f.axisMapGeneration == s.scope.axisMapGeneration && f.configGeneration == s.scope.configGeneration &&
                f.pulsePerMm == s.config.pulsePerMm && f.referenceOffsetPulse == s.config.referenceOffsetPulse &&
                f.hardwareSign == s.config.hardwareSign && f.axisIdle &&
                std::isfinite(f.actualPulse) && std::isfinite(f.commandPulse) && std::isfinite(f.planningPulse) &&
                std::isfinite(f.cmdSpeedMmS) && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm;
            if (!coherent) return false;
            if (!f.capHeld && !f.originValid && !s.motionApplied && f.armReady &&
                (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                    f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence))) return true;
            return IsEDMGapCurveConfigValidSameThread(true) && EDM28::SameScope(f.scope, s.scope) &&
                f.capHeld && f.frameCurrent && f.originValid && std::isfinite(f.outerOriginPulse) &&
                f.state == EDM28::State::Held && f.stopLatched && f.stopProven && f.proofSequence &&
                f.stopAppliedSequence && f.stopAppliedTick && f.stopAppliedMonotonicUs &&
                f.lastAppliedKind == EDM28::RequestKind::Stop && f.lastAppliedSequence == f.stopAppliedSequence &&
                f.lastAppliedTick == f.stopAppliedTick && f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs &&
                f.stableCycles >= s.config.settleCycles && f.sampledTick >= f.stopAppliedTick &&
                f.sampledTick - f.stopAppliedTick >= s.config.settleCycles && f.monotonicUs >= f.stopAppliedMonotonicUs &&
                f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
                std::fabs(f.actualPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm &&
                std::fabs(f.commandPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm &&
                std::fabs(f.planningPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm;
        }
    }
    if ((m_edmZFixture.config.profile == EDM28::Profile::P17PersistentShort || m_edmZFixture.config.profile == EDM28::Profile::P22GapPersistentShort) &&
        m_gapDryRun.paused && IsEDMPersistentShortReleasedSameThread()) return true;
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    return EDM28::SameScope(feedback.scope, state.scope) && feedback.state == EDM28::State::Held &&
        feedback.stopLatched && feedback.stopProven && feedback.stopAppliedSequence != 0ULL &&
        feedback.stopAppliedTick != 0ULL && feedback.sampledTick > feedback.stopAppliedTick &&
        feedback.proofSequence != 0ULL && feedback.stableCycles >= state.config.settleCycles && feedback.axisIdle &&
        std::fabs(feedback.cmdSpeedMmS) <= 1.0 / state.config.pulsePerMm &&
        std::fabs(feedback.actualPulse - state.originPulse) <= state.config.outerHalfMm * state.config.pulsePerMm;
}

void NCManager::LogEDMZFixtureSameThread(const char* event, const char* result, const char* reason) noexcept
{
    if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile))
    { LogEDMGapDynamicSameThread(event, result, reason); return; }
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    // P24 snapshots all events, including RT_STOP, before the legacy formatter.
    // These event diagnostics never block armed fixture service observations.
    if (state.gapCurveReplanProfile || state.config.profile == EDM28::Profile::P24GapCurveReplan ||
        m_edmGapShortReplan.active || state.config.profile == EDM28::Profile::P25GapShortReplan ||
        state.config.profile == EDM28::Profile::P26GapPersistentReplan ||
        m_edmGapFeedUpdate.shortState.active || EDM28::IsGapFeedUpdateProfile(state.config.profile) ||
        m_edmGapRapid.shortState.active || EDM28::IsGapRapidProfile(state.config.profile))
    { LogEDMGapReplanFixtureSameThread(event, result, reason); return; }
    if (event && std::strcmp(event, "RT_STOP") == 0)
    {
        char line[480]{};
        int size = std::snprintf(line, sizeof(line),
            "[EDM34-FIX1] event=RT_STOP part=STATE result=%.10s reason=%.32s test=%llu session=%llu profile=%u state=%u rtReason=%u sourceFresh=%u pdoValid=%u contiguous=%u clockValid=%u frame=%u cap=%u zero=%u fault=%u stopLatched=%u stopped=%u dwell=%u sameScope=%u physicalPermit=0 discharge=0\n",
            result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned>(state.config.profile), static_cast<unsigned>(feedback.state), static_cast<unsigned>(feedback.reason),
            feedback.sourceFresh ? 1U : 0U, feedback.pdoValid ? 1U : 0U, feedback.contiguous ? 1U : 0U, feedback.clockValid ? 1U : 0U,
            feedback.frameCurrent ? 1U : 0U, feedback.capHeld ? 1U : 0U, feedback.forceZeroOutput ? 1U : 0U,
            feedback.latchedFault ? 1U : 0U, feedback.stopLatched ? 1U : 0U, feedback.stopProven ? 1U : 0U,
            static_cast<unsigned>(feedback.stableCycles), EDM28::SameScope(feedback.scope, state.scope) ? 1U : 0U);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM34-FIX1] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        size = std::snprintf(line, sizeof(line),
            "[EDM34-FIX1] event=RT_STOP part=STAMP result=%.10s test=%llu session=%llu seq=%llu kind=%u at=%llu appliedUs=%llu stopSeq=%llu stopTick=%llu stopUs=%llu tick=%llu us=%llu proof=%llu physicalPermit=0 discharge=0\n",
            result, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned>(feedback.lastAppliedKind),
            static_cast<unsigned long long>(feedback.lastAppliedTick), static_cast<unsigned long long>(feedback.lastAppliedMonotonicUs),
            static_cast<unsigned long long>(feedback.stopAppliedSequence), static_cast<unsigned long long>(feedback.stopAppliedTick),
            static_cast<unsigned long long>(feedback.stopAppliedMonotonicUs), static_cast<unsigned long long>(feedback.sampledTick),
            static_cast<unsigned long long>(feedback.monotonicUs), static_cast<unsigned long long>(feedback.proofSequence));
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM34-FIX1] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        return;
    }
    if (state.gapCurveProfile || state.config.profile == EDM28::Profile::P23GapCurveSegments)
    {
        const auto& gap = state.gapShort.Snapshot(); const auto curve = EDM39::Evaluate(state.curveVoltage);
        char line[480]{};
        int size = std::snprintf(line, sizeof(line),
            "[EDM39] event=%.16s result=%.10s reason=%.40s test=%llu session=%llu run=%llu phase=%u step=%u cycles=%u moves=%u zeros=%u elapsedMs=%llu restart=%u originPulse=%.9g actualPulse=%.9g physicalPermit=0 discharge=0\n",
            event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned>(state.phase), static_cast<unsigned>(state.curveStep),
            static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.curvePositions), static_cast<unsigned>(state.curveZeros),
            static_cast<unsigned long long>(state.windowStartMs && state.lastClockMs >= state.windowStartMs ? state.lastClockMs - state.windowStartMs : 0ULL),
            static_cast<unsigned>(m_gapDryRun.restarts), state.originPulse, feedback.actualPulse);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM39] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        size = std::snprintf(line, sizeof(line),
            "[EDM39] event=%.16s part=CURVE test=%llu simV=%.0f curveF=%.9g sampleTick=%llu sampleUs=%llu shortState=%u inhibit=%u gapFault=%u request=%llu sourceTick=%llu sourceUs=%llu requestV=%.0f requestF=%.9g targetMm=%.4f zeroStartTick=%llu zeroStartUs=%llu physicalPermit=0 discharge=0\n",
            event, static_cast<unsigned long long>(m_gapDryRun.test), state.curveVoltage, curve.limitedSpeedMmPerMin,
            static_cast<unsigned long long>(gap.sampleTick), static_cast<unsigned long long>(gap.sampleMonotonicUs),
            static_cast<unsigned>(gap.state), gap.feedInhibited ? 1U : 0U, static_cast<unsigned>(gap.fault),
            static_cast<unsigned long long>(state.curveSequence), static_cast<unsigned long long>(state.curveSourceTick),
            static_cast<unsigned long long>(state.curveSourceUs), state.curveAppliedVoltage, state.curveSignedMmMin,
            state.curveTargetMm, static_cast<unsigned long long>(state.curveZeroStartTick), static_cast<unsigned long long>(state.curveZeroStartUs));
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM39] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        if (std::strcmp(event, "BOUNDARY") == 0 || std::strcmp(event, "RECOVERED") == 0 ||
            std::strcmp(event, "ORIGIN_PROOF") == 0 || std::strcmp(event, "SUMMARY") == 0 || std::strcmp(event, "RECEIPT") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM39] event=%.16s part=APPLIED test=%llu seq=%llu kind=%u at=%llu appliedUs=%llu curveSeq=%llu curveTick=%llu curveUs=%llu curveV=%.0f curveF=%.9g physicalPermit=0 discharge=0\n",
                event, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(feedback.lastAppliedSequence),
                static_cast<unsigned>(feedback.lastAppliedKind), static_cast<unsigned long long>(feedback.lastAppliedTick),
                static_cast<unsigned long long>(feedback.lastAppliedMonotonicUs), static_cast<unsigned long long>(feedback.lastCurveSequence),
                static_cast<unsigned long long>(feedback.lastCurveSampleTick), static_cast<unsigned long long>(feedback.lastCurveSampleUs),
                feedback.lastCurveVoltage, feedback.lastSignedCurveMmMin);
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM39] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
            size = std::snprintf(line, sizeof(line),
                "[EDM39] event=%.16s part=PROOF test=%llu stopSeq=%llu stopTick=%llu stopUs=%llu proof=%llu tick=%llu us=%llu pub=%llu physicalPermit=0 discharge=0\n",
                event, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(feedback.stopAppliedSequence),
                static_cast<unsigned long long>(feedback.stopAppliedTick), static_cast<unsigned long long>(feedback.stopAppliedMonotonicUs),
                static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned long long>(feedback.sampledTick),
                static_cast<unsigned long long>(feedback.monotonicUs), static_cast<unsigned long long>(feedback.publicationSequence));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM39] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        if (std::strcmp(event, "BEGIN") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM39] event=BEGIN part=SCOPE test=%llu session=%llu cache=%llu dispatch=%llu epoch=%u lease=%u originPulse=%.9g capMmS=.35 maxF=20 cycles=6 zeroMs=500 segmentOnly=1 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(state.scope.cacheGeneration), static_cast<unsigned long long>(state.scope.dispatchGeneration),
                state.scope.executionEpoch, state.scope.ownerGeneration, state.originPulse);
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM39] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        m_edmZFixture.logMs = state.lastClockMs; return;
    }
    if (state.config.profile == EDM28::Profile::P22GapPersistentShort || state.gapPersistentProfile)
    {
        const auto& gap = state.gapShort.Snapshot(); char line[480]{};
        int size = std::snprintf(line, sizeof(line),
            "[EDM38] event=%.16s result=%.10s reason=%.32s test=%llu session=%llu run=%llu sourceTest=%llu phase=%u cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u latched=%u expectedAlarm=4001 physicalPermit=0 discharge=0\n",
            event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(state.scope.session), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(state.shortSourceTest), static_cast<unsigned>(state.phase),
            static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
            static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears), state.persistentShortLatched ? 1U : 0U);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM38] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        size = std::snprintf(line, sizeof(line),
            "[EDM38] event=%.16s part=GAP test=%llu simV=%.0f sampleTick=%llu sampleUs=%llu entryUs=%llu state=%u valid=%u fresh=%u inhibited=%u active=%u fault=%u originPulse=%.9g actualPulse=%.9g proof=%llu endpoint=%u stopped=%u cap=%u F=5 pdoCap=0.1 physicalPermit=0 discharge=0\n",
            event, static_cast<unsigned long long>(m_gapDryRun.test), gap.voltage,
            static_cast<unsigned long long>(gap.sampleTick), static_cast<unsigned long long>(gap.sampleMonotonicUs),
            static_cast<unsigned long long>(state.gapPersistentEntryUs), static_cast<unsigned>(gap.state), gap.valid ? 1U : 0U, gap.fresh ? 1U : 0U,
            gap.feedInhibited ? 1U : 0U, gap.shortActive ? 1U : 0U, static_cast<unsigned>(gap.fault),
            state.originPulse, feedback.actualPulse, static_cast<unsigned long long>(feedback.proofSequence),
            static_cast<unsigned>(state.endpointProofKind), feedback.stopProven ? 1U : 0U, feedback.capHeld ? 1U : 0U);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM38] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        if (std::strcmp(event, "BEGIN") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM38] event=BEGIN part=SCOPE test=%llu session=%llu cache=%llu dispatch=%llu epoch=%u lease=%u restart=%u q=%u physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(m_gapDryRun.cache), static_cast<unsigned long long>(m_gapDryRun.dispatch),
                static_cast<unsigned>(m_gapDryRun.epoch), static_cast<unsigned>(m_gapDryRun.lease.generation),
                static_cast<unsigned>(m_gapDryRun.restarts), static_cast<unsigned>(state.operatorWindowQ));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM38] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        if (std::strcmp(event, "ORIGIN_PROOF") == 0 || std::strcmp(event, "TERMINAL_FREE") == 0 ||
            std::strcmp(event, "EXPECTED_ALARM") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM38] event=%.16s part=RECEIPT test=%llu kind=%u seq=%llu at=%llu appliedUs=%llu stopSeq=%llu stopTick=%llu stopUs=%llu proof=%llu endpointProof=%llu tick=%llu us=%llu pub=%llu physicalPermit=0 discharge=0\n",
                event, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned>(feedback.lastAppliedKind),
                static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.lastAppliedTick),
                static_cast<unsigned long long>(feedback.lastAppliedMonotonicUs), static_cast<unsigned long long>(feedback.stopAppliedSequence),
                static_cast<unsigned long long>(feedback.stopAppliedTick), static_cast<unsigned long long>(feedback.stopAppliedMonotonicUs),
                static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned long long>(state.endpointProofSequence),
                static_cast<unsigned long long>(feedback.sampledTick), static_cast<unsigned long long>(feedback.monotonicUs),
                static_cast<unsigned long long>(feedback.publicationSequence));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM38] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        m_edmZFixture.logMs = m_edmZFixture.lastClockMs; return;
    }
    if (state.config.profile == EDM28::Profile::P17PersistentShort)
    {
        const std::uint64_t elapsed = state.windowStartMs && state.lastClockMs >= state.windowStartMs ? state.lastClockMs - state.windowStartMs : 0ULL;
        const unsigned simV = state.persistentShortLatched ? 20U : state.phase == EDMZFixturePhase::Negative ? 40U : 50U;
        const char* endpoint = state.endpointProofKind == 2U ? "POSITION" : state.endpointProofKind == 1U ? "STOP" : "NONE";
        char line[480]{}; int size = 0;
        if (std::strcmp(event, "BEGIN") == 0)
            size = std::snprintf(line, sizeof(line),
                "[EDM32] event=BEGIN result=RUNNING test=%llu session=%llu sourceTest=%llu run=%llu cache=%llu dispatch=%llu epoch=%u lease=%u restart=%u minMs=0 maxMs=30000 expectedAlarm=4001 cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u simV=%u latched=%u physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(state.shortSourceTest), static_cast<unsigned long long>(m_gapDryRun.run),
                static_cast<unsigned long long>(m_gapDryRun.cache), static_cast<unsigned long long>(m_gapDryRun.dispatch),
                static_cast<unsigned>(m_gapDryRun.epoch), static_cast<unsigned>(m_gapDryRun.lease.generation), static_cast<unsigned>(m_gapDryRun.restarts),
                static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops), static_cast<unsigned>(state.verifiedRetreats),
                static_cast<unsigned>(state.verifiedShortClears), simV, state.persistentShortLatched ? 1U : 0U);
        else size = std::snprintf(line, sizeof(line),
            "[EDM32] event=%.14s result=%.10s reason=%.30s test=%llu session=%llu sourceTest=%llu phase=%u ms=%llu simV=%u latched=%u endpoint=%s seq=%llu tick=%llu proof=%llu cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
            event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(state.shortSourceTest), static_cast<unsigned>(state.phase), static_cast<unsigned long long>(elapsed),
            simV, state.persistentShortLatched ? 1U : 0U, endpoint,
            static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.sampledTick),
            static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
            static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM32] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        if (std::strcmp(event, "BEGIN") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM32] event=LIMITS test=%llu session=%llu outerMm=.1 feedTargetMm=-.02 retreatTargetMm=0 feedMmMin=5 pdoCapMmS=.1 deadmanMs=50 dwellCycles=200 toleranceMm=.001 anchor=FIRST_ARM shortPolicy=PERSISTENT_TO_START_ALARM physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM32] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        if (std::strcmp(event, "ORIGIN_PROOF") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM32] event=ENDPOINT_PROOF test=%llu session=%llu sourceTest=%llu endpoint=%s originPulse=%.9g maxPulse=%.9g zPulse=%.9g stopProof=%llu stopSeq=%llu stopTick=%llu stopUs=%llu seq=%llu at=%llu proof=%llu simV=20 latched=1 expectedAlarm=4001 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(state.shortSourceTest), endpoint, state.originPulse, state.shortStopMaxPulse, feedback.actualPulse,
                static_cast<unsigned long long>(state.shortStopProof), static_cast<unsigned long long>(state.shortStopAppliedSequence),
                static_cast<unsigned long long>(state.shortStopAppliedTick), static_cast<unsigned long long>(state.shortStopAppliedMonotonicUs),
                static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.lastAppliedTick),
                static_cast<unsigned long long>(feedback.proofSequence));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM32] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        if (std::strcmp(event, "EXPECTED_ALARM") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[EDM32] event=ALARM_PROOF test=%llu session=%llu sourceTest=%llu endpoint=%s originProof=%llu originSeq=%llu stopSeq=%llu stopTick=%llu stopUs=%llu disarmSeq=%llu disarmAt=%llu proof=%llu tick=%llu stopped=1 disarmed=1 capHeld=0 simV=20 alarm=4001 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(state.shortSourceTest), endpoint, static_cast<unsigned long long>(state.endpointProofSequence),
                static_cast<unsigned long long>(state.endpointAppliedSequence), static_cast<unsigned long long>(state.terminalStopAppliedSequence),
                static_cast<unsigned long long>(state.terminalStopAppliedTick), static_cast<unsigned long long>(state.terminalStopAppliedMonotonicUs),
                static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.lastAppliedTick),
                static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned long long>(feedback.sampledTick));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM32] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        m_edmZFixture.logMs = m_edmZFixture.lastClockMs;
        return;
    }
    if (state.config.profile == EDM28::Profile::P16UnloadedRetreat || EDM28::IsSpeedRetreatProfile(state.config.profile))
    {
        const bool speedRetreat = EDM28::IsSpeedRetreatProfile(state.config.profile);
        const bool gapShortRetreat = state.config.profile == EDM28::Profile::P21GapShortRetreat;
        const char* tag = gapShortRetreat ? "EDM37" : state.config.profile == EDM28::Profile::P20UnloadedSpeedRetreat ? "EDM36" : state.config.profile == EDM28::Profile::P19UnloadedSpeedRetreat ? "EDM35" : speedRetreat ? "EDM34" : "EDM31";
        // Each P16 line fits the 512-byte console adapter even with full-width
        // legal identities/counters. BEGIN scope and fixed limits are separate
        // records tied to the same test/session; prior profile formats stay intact.
        const std::uint64_t elapsed = state.windowStartMs && state.lastClockMs >= state.windowStartMs ? state.lastClockMs - state.windowStartMs : 0ULL;
        const bool shortHeld = state.phase == EDMZFixturePhase::ShortStop || state.phase == EDMZFixturePhase::Retreat ||
            (state.phase == EDMZFixturePhase::ArmPending && !state.beginPending && state.armNextPhase == EDMZFixturePhase::Retreat);
        const unsigned simV = gapShortRetreat ? (state.gapShort.Snapshot().valid ? static_cast<unsigned>(state.gapShort.Snapshot().voltage) : 0U) :
            std::strcmp(event, "SHORT_CLEAR") == 0 ? 50U : shortHeld ? 20U :
            state.phase == EDMZFixturePhase::Negative ? 40U : state.phase == EDMZFixturePhase::Positive ? 60U : 50U;
        char line[480]{};
        int size = 0;
        if (std::strcmp(event, "BEGIN") == 0)
            size = std::snprintf(line, sizeof(line),
                "[%s] event=BEGIN result=RUNNING test=%llu session=%llu run=%llu cache=%llu dispatch=%llu epoch=%u lease=%u restart=%u minMs=10000 maxMs=30000 cyclesMin=6 cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalMotion=%u physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.cache),
                static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.epoch),
                static_cast<unsigned>(m_gapDryRun.lease.generation), static_cast<unsigned>(m_gapDryRun.restarts),
                static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
                static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears), state.motionApplied ? 1U : 0U);
        else if (std::strcmp(event, "PREFLIGHT") == 0)
            size = std::snprintf(line, sizeof(line),
                "[%s] event=PREFLIGHT result=%.10s reason=%.32s test=%llu session=%llu source=RT_PDO homeFact=%u softLimitsFact=%u ppu=%.9g sign=%d map=%llu config=%llu cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
                tag, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                feedback.homeFact ? 1U : 0U, feedback.softLimitsFact ? 1U : 0U, feedback.pulsePerMm, feedback.hardwareSign,
                static_cast<unsigned long long>(state.scope.axisMapGeneration), static_cast<unsigned long long>(state.scope.configGeneration),
                static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
                static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
        else if (std::strcmp(event, "SUMMARY") == 0 || std::strcmp(event, "RECEIPT") == 0)
            size = std::snprintf(line, sizeof(line),
                "[%s] event=%.12s result=%.10s reason=%.32s test=%llu session=%llu run=%llu dispatch=%llu cycles=%u elapsedMs=%llu seq=%llu tick=%llu proof=%llu stopped=%u disarmed=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
                tag, event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.dispatch),
                static_cast<unsigned>(state.cycles), static_cast<unsigned long long>(elapsed),
                static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.sampledTick),
                static_cast<unsigned long long>(feedback.proofSequence),
                feedback.stopProven ? 1U : 0U, feedback.state == EDM28::State::Disarmed ? 1U : 0U,
                static_cast<unsigned>(state.simulatedShortStops), static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
        else size = std::snprintf(line, sizeof(line),
            "[%s] event=%.12s result=%.10s reason=%.32s test=%llu session=%llu phase=%u cycles=%u ms=%llu simV=%u target=%.6g z=%.6g cmd=%.6g pdo=%.6g seq=%llu tick=%llu proof=%llu simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
            tag, event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned>(state.phase), static_cast<unsigned>(state.cycles), static_cast<unsigned long long>(elapsed), simV,
            (feedback.targetPulse - state.originPulse) / (state.config.pulsePerMm > 0.0 ? state.config.pulsePerMm : 1.0),
            feedback.actualMm, feedback.cmdSpeedMmS, feedback.pdoSpeedMmS,
            static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.sampledTick),
            static_cast<unsigned long long>(feedback.proofSequence),
            static_cast<unsigned>(state.simulatedShortStops), static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n", tag);
        if (std::strcmp(event, "BEGIN") == 0)
        {
            if (speedRetreat) size = std::snprintf(line, sizeof(line),
                "[%s] event=LIMITS test=%llu session=%llu outerMm=.1 targetMm=.02 feedMmMin=%.9g pdoCapMmS=%.9g deadmanMs=50 dwellCycles=200 toleranceMm=.001 simRefV=50 anchor=FIRST_ARM shortPolicy=RETREAT_BEFORE_CLEAR cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                state.config.feedMmMin, state.config.pdoCapMmS,
                static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
                static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
            else size = std::snprintf(line, sizeof(line),
                "[EDM31] event=LIMITS test=%llu session=%llu outerMm=.1 targetMm=.02 feedMmMin=5 pdoCapMmS=.1 deadmanMs=50 dwellCycles=200 toleranceMm=.001 simRefV=50 anchor=FIRST_ARM shortPolicy=RETREAT_BEFORE_CLEAR cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
                static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n", tag);
        }
        if (gapShortRetreat && (std::strcmp(event, "BEGIN") == 0 || std::strcmp(event, "SHORT_STOP") == 0 ||
            std::strcmp(event, "RETREAT_ARM") == 0 || std::strcmp(event, "CLEAR_WAIT") == 0 ||
            std::strcmp(event, "SHORT_CLEAR") == 0 || std::strcmp(event, "PAUSED") == 0 || std::strcmp(event, "SUMMARY") == 0))
        {
            const auto& gap = state.gapShort.Snapshot();
            size = std::snprintf(line, sizeof(line),
                "[EDM37] event=GAP_DETECTOR edge=%.12s test=%llu session=%llu source=LOCAL_SIM sampleTick=%llu sampleUs=%llu sampleMs=%llu simV=%.3g gapState=%u valid=%u inhibited=%u active=%u gapFault=%u highSinceUs=%llu clearProof=%llu thresholdV=30 hysteresisV=1 enterMs=2 exitMs=5 physicalPermit=0 discharge=0\n",
                event, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
                static_cast<unsigned long long>(gap.sampleTick), static_cast<unsigned long long>(gap.sampleMonotonicUs),
                static_cast<unsigned long long>(gap.sampleTimeMs), gap.voltage, static_cast<unsigned>(gap.state),
                gap.valid ? 1U : 0U, gap.feedInhibited ? 1U : 0U, gap.shortActive ? 1U : 0U, static_cast<unsigned>(gap.fault),
                static_cast<unsigned long long>(state.gapClearHighUs), static_cast<unsigned long long>(state.gapClearProof));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM37] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        }
        if (std::strcmp(event, "SHORT_CLEAR") == 0)
        {
            size = std::snprintf(line, sizeof(line),
                "[%s] event=RETREAT_PROOF test=%llu session=%llu cycles=%u stopProof=%llu stopSeq=%llu stopTick=%llu stopUs=%llu stoppedMax=%.9g zPulse=%.9g seq=%llu at=%llu proof=%llu dwell=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session), static_cast<unsigned>(state.cycles),
                static_cast<unsigned long long>(state.shortStopProof), static_cast<unsigned long long>(state.shortStopAppliedSequence),
                static_cast<unsigned long long>(state.shortStopAppliedTick), static_cast<unsigned long long>(state.shortStopAppliedMonotonicUs),
                state.shortStopMaxPulse, feedback.actualPulse,
                static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.lastAppliedTick),
                static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned>(feedback.stableCycles), static_cast<unsigned>(state.simulatedShortStops),
                static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n", tag);
        }
        m_edmZFixture.logMs = m_edmZFixture.lastClockMs;
        return;
    }
    const bool repeatedShort = state.config.profile == EDM28::Profile::P15UnloadedRepeat;
    const char* tag = repeatedShort ? "EDM30" : state.config.profile == EDM28::Profile::P14UnloadedSpeed ? "EDM29" : "EDM28";
    const unsigned simV = state.phase == EDMZFixturePhase::Negative ? 40U :
        state.phase == EDMZFixturePhase::ShortStop ? 20U : state.phase == EDMZFixturePhase::Positive ? 60U : 50U;
    char line[512]{};
    const std::uint64_t elapsed = state.windowStartMs && state.lastClockMs >= state.windowStartMs ? state.lastClockMs - state.windowStartMs : 0ULL;
    int size = 0;
    if (std::strcmp(event, "PREFLIGHT") == 0)
    {
        size = std::snprintf(line, sizeof(line),
            "[%s] event=PREFLIGHT result=%.10s reason=%.32s test=%llu session=%llu axis=Z source=RT_PDO homeFact=%u softLimitsFact=%u ppu=%.9g sign=%d map=%llu config=%llu physicalMotion=0 physicalPermit=0 discharge=0\n",
            tag, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            feedback.homeFact ? 1U : 0U, feedback.softLimitsFact ? 1U : 0U, feedback.pulsePerMm, feedback.hardwareSign,
            static_cast<unsigned long long>(state.scope.axisMapGeneration), static_cast<unsigned long long>(state.scope.configGeneration));
    }
    else if (std::strcmp(event, "BEGIN") == 0)
    {
        size = std::snprintf(line, sizeof(line),
            "[%s] event=BEGIN result=RUNNING test=%llu session=%llu run=%llu cache=%llu dispatch=%llu epoch=%u lease=%u restart=%u minMs=10000 maxMs=30000 cyclesMin=%u outerMm=.1 targetMm=.02 feedMmMin=%.1f pdoCapMmS=.1 deadmanMs=50 dwellCycles=200 toleranceMm=.001 simRefV=50 anchor=FIRST_ARM physicalMotion=%u physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.cache),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.epoch),
            static_cast<unsigned>(m_gapDryRun.lease.generation), static_cast<unsigned>(m_gapDryRun.restarts),
            static_cast<unsigned>(EDM28::ProfileMinimumCycles(state.config.profile)), state.config.feedMmMin, state.motionApplied ? 1U : 0U);
    }
    else if (std::strcmp(event, "SUMMARY") == 0 || std::strcmp(event, "RECEIPT") == 0)
    {
        size = std::snprintf(line, sizeof(line),
            "[%s] event=%.12s result=%.10s reason=%.32s test=%llu session=%llu run=%llu dispatch=%llu restart=%u cycles=%u elapsedMs=%llu seq=%llu tick=%llu proof=%llu dwell=%u stopped=%u disarmed=%u physicalMotion=%u physicalPermit=0 discharge=0\n",
            tag, event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.dispatch),
            static_cast<unsigned>(m_gapDryRun.restarts), static_cast<unsigned>(state.cycles), static_cast<unsigned long long>(elapsed),
            static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.sampledTick),
            static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned>(feedback.stableCycles),
            feedback.stopProven ? 1U : 0U, feedback.state == EDM28::State::Disarmed ? 1U : 0U, state.motionApplied ? 1U : 0U);
    }
    else size = std::snprintf(line, sizeof(line),
        "[%s] event=%.12s result=%.10s reason=%.32s test=%llu session=%llu phase=%u cycles=%u ms=%llu simV=%u target=%.6g z=%.6g cmd=%.6g pdo=%.6g seq=%llu at=%llu tick=%llu dwell=%u proof=%llu state=%u why=%u physicalMotion=%u physicalPermit=0 discharge=0\n",
        tag, event, result, reason, static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(state.scope.session),
        static_cast<unsigned>(state.phase), static_cast<unsigned>(state.cycles),
        static_cast<unsigned long long>(elapsed),
        simV, (feedback.targetPulse - state.originPulse) / (state.config.pulsePerMm > 0.0 ? state.config.pulsePerMm : 1.0),
        feedback.actualMm, feedback.cmdSpeedMmS, feedback.pdoSpeedMmS,
        static_cast<unsigned long long>(feedback.lastAppliedSequence), static_cast<unsigned long long>(feedback.lastAppliedTick),
        static_cast<unsigned long long>(feedback.sampledTick), static_cast<unsigned>(feedback.stableCycles),
        static_cast<unsigned long long>(feedback.proofSequence), static_cast<unsigned>(feedback.state), static_cast<unsigned>(feedback.reason),
        state.motionApplied ? 1U : 0U);
    // Existing profile lines remain identical. P15 adds bounded evidence to
    // each line; injections and proved clear applications are distinct facts.
    if (repeatedShort && size > 0 && static_cast<std::size_t>(size) < sizeof(line))
    {
        const std::size_t tail = static_cast<std::size_t>(size - 1);
        if (line[tail] == '\n')
        {
            const int extra = std::snprintf(line + tail, sizeof(line) - tail,
                " simShortStops=%u verifiedShortClears=%u%s\n",
                static_cast<unsigned>(state.simulatedShortStops), static_cast<unsigned>(state.verifiedShortClears),
                std::strcmp(event, "BEGIN") == 0 ? " shortPolicy=EACH_NEGATIVE_LEG" : "");
            size = extra >= 0 && static_cast<std::size_t>(extra) < sizeof(line) - tail ?
                static_cast<int>(tail) + extra : -1;
        }
        else size = -1;
    }
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n", tag);
    m_edmZFixture.logMs = m_edmZFixture.lastClockMs;
}

void NCManager::PauseEDMZFixtureSameThread(const char* reason) noexcept
{
    if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile))
    { PauseEDMGapDynamicSameThread(reason); return; }
    if (m_edmZFixture.gapCurveProfile && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmZFixture.phase == EDMZFixturePhase::Disarm || m_edmZFixture.phase == EDMZFixturePhase::Receipt))
    {
        m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
        // A second HOLD may arrive while the prior resume still awaits fresh evidence.
        m_edmZFixture.terminalResumeFloorSet = false; m_edmZFixture.resumeStartMs = 0ULL;
        if (m_edmGapRapid.shortState.active) m_edmGapRapid.shortState.terminalResumeObserved = false;
        if (m_gapDryRun.paused) return;
        m_gapDryRun.paused = true; m_edmZFixture.curveTerminalHold = true;
        m_edmZFixture.receiptPublication = m_edmZFixture.feedback.publicationSequence;
        m_edmZFixture.receiptTick = m_edmZFixture.feedback.sampledTick;
        m_edmZFixture.receiptUs = m_edmZFixture.feedback.monotonicUs;
        LogEDMZFixtureSameThread("PAUSED", "WAIT", "TERMINAL_HOLD_RETAIN_EXACT_RECEIPT"); return;
    }
    if (m_edmZFixture.gapCurveProfile && m_gapDryRun.active && !m_edmZFixture.originPinned)
    {
        m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
        if (m_gapDryRun.paused) return;
        m_edmZFixture.curvePreArmHold = true;
        m_edmZFixture.curvePreArmSequence = m_edmZFixture.pending && m_edmZFixture.pendingKind == EDM28::RequestKind::Arm ?
            m_edmZFixture.pendingSequence : 0ULL;
        m_gapDryRun.paused = true; m_edmZFixture.resumeStartMs = 0ULL;
        LogEDMZFixtureSameThread("PAUSED", "WAIT", "PREARM_HOLD_AWAIT_REAL_APPLICATION"); return;
    }
    if (!m_gapDryRun.active) { if (m_gapDryRun.result == 1U) EndEDMZFixtureSameThread("CANCELLED", reason); return; }
    m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
    if (!m_edmZFixture.originPinned) { EndEDMZFixtureSameThread("CANCELLED", "HOLD_BEFORE_ARM_APPLICATION"); return; }
    if (m_gapDryRun.paused) return;
    if ((m_edmZFixture.config.profile == EDM28::Profile::P17PersistentShort || m_edmZFixture.config.profile == EDM28::Profile::P22GapPersistentShort) && m_edmZFixture.operatorWindowActive)
    {
        m_edmZFixture.operatorWindowActive = false; m_edmZFixture.operatorWindowConsumed = true;
        LogEDMPersistentShortOperatorWindowSameThread("EXIT", "HOLD_CONSUMED");
    }
    if (m_edmZFixture.gapPersistentProfile) m_edmZFixture.gapPersistentResumeObserved = false;
    m_gapDryRun.paused = true; m_edmZFixture.pending = false; m_edmZFixture.resumeStartMs = 0ULL;
    m_edmZFixture.phase = EDMZFixturePhase::ShortStop;
    LogEDMZFixtureSameThread("PAUSED", "WAIT", "STOP_THEN_RESTART_FULL_WINDOW");
}

void NCManager::EndEDMZFixtureSameThread(const char* result, const char* reason) noexcept
{
    InvalidateEDMPhysicalSequencePlanSameThread();
    if (EDM28::IsPhysicalServoProfile(m_edmZFixture.config.profile) && !m_gapDryRun.active && m_gapDryRun.result == 3U) return;
    if (m_edmZFixture.phase == EDMZFixturePhase::Finished)
    {
        if (m_edmGapDynamic.sequenceMode)
        {
            // No cap remains after PASS, but a boundary cancellation must
            // invalidate the chain and engage the existing RESET-only guard.
            m_edmZFixture.phase = EDMZFixturePhase::Cancelled;
            m_edmGapDynamic.sequenceReceiptPublication = 0ULL;
            m_edmGapDynamic.sequenceReceiptTick = m_edmGapDynamic.sequenceReceiptUs = 0ULL;
            m_gapDryRun.active = m_gapDryRun.paused = false;
            m_gapDryRun.result = result && std::strcmp(result, "FAIL") == 0 ? 3U : 2U;
            RtPrintf("[EDM77] event=SEQUENCE_BOUNDARY result=CANCELLED session=%llu reason=%s resetRequired=1 discharge=0\n",
                static_cast<unsigned long long>(m_edmZFixture.scope.session), reason ? reason : "UNKNOWN");
            return;
        }
        m_gapDryRun.result = 2U; return;
    }
    // A never-issued EDM56 admission owns no physical output and needs no
    // fictitious Stop/Disarm acknowledgement. Failures still use the bounded
    // existing deferred alarm service; cancellation does not create an alarm.
    if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile) && !m_edmGapDynamic.armIssued)
    {
        const bool failed = result && std::strcmp(result, "FAIL") == 0;
        if (failed && !m_edmZFixture.failureAlarmPending)
        {
            m_edmZFixture.failureAlarmPending = true;
            m_edmZFixture.failureRtReason = static_cast<std::uint8_t>(m_edmZFixture.feedback.reason);
            std::uint64_t now = m_edmZFixture.lastClockMs;
            (void)ReadGapDryRunClockSameThread(now); m_edmZFixture.failureStartMs = now;
            (void)std::snprintf(m_edmZFixture.failureReason, sizeof(m_edmZFixture.failureReason), "%.47s", reason ? reason : "UNKNOWN");
        }
        m_gapDryRun.active = m_gapDryRun.paused = false; m_gapDryRun.result = failed ? 3U : 2U;
        m_edmZFixture.pending = m_edmZFixture.cleanupPending = false;
        m_edmZFixture.phase = EDMZFixturePhase::Cancelled;
        LogEDMGapDynamicSameThread("SUMMARY", result, reason); return;
    }
    // Preserve the last motion-service log cost before fault logging replaces
    // it. A subsequent CONTROL_GAP diagnostic must describe the earlier batch.
    const std::uint64_t precedingLogEndMs = m_edmZFixture.lastDynamicLogEndMs;
    const std::uint64_t precedingLogDurationMs = m_edmZFixture.lastDynamicLogDurationMs;
    // Publish the priority Stop before any failure diagnostic can block NC.
    m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
    const bool failed = result && std::strcmp(result, "FAIL") == 0;
    const bool expectedShortAlarm = failed && (m_edmZFixture.config.profile == EDM28::Profile::P17PersistentShort || m_edmZFixture.config.profile == EDM28::Profile::P22GapPersistentShort ||
        (m_edmGapShortReplan.active && m_edmGapShortReplan.terminalAlarm) ||
        (m_edmGapFeedUpdate.shortState.active && m_edmGapFeedUpdate.shortState.terminalAlarm) ||
        (m_edmGapRapid.shortState.active && m_edmGapRapid.shortState.terminalAlarm) ||
        (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile) && m_edmGapDynamic.terminalObserved && m_edmGapDynamic.terminal.persistentShort)) &&
        reason && std::strcmp(reason, "PERSISTENT_SHORT_AT_ORIGIN") == 0;
    const bool captureFailure = failed && !expectedShortAlarm && !m_edmZFixture.failureAlarmPending;
    const bool controlGap = failed && reason && std::strcmp(reason, "CONTROL_DEADMAN_50MS") == 0;
    // HOLD can outlive the last motion-service timestamp. Capture the actual
    // failure boundary before console IO; cached feedback is not a fresh RT read.
    std::uint64_t failureNow = m_edmZFixture.lastClockMs;
    const bool failureClockValid = (captureFailure || controlGap) && ReadGapDryRunClockSameThread(failureNow);
    if (captureFailure)
    {
        m_edmZFixture.failureAlarmPending = true;
        m_edmZFixture.failureStartMs = failureNow;
        m_edmZFixture.failurePolls = 0U;
        m_edmZFixture.failureRtReason = static_cast<std::uint8_t>(m_edmZFixture.feedback.reason);
        (void)std::snprintf(m_edmZFixture.failureReason, sizeof(m_edmZFixture.failureReason),
            "%.47s", reason ? reason : "UNKNOWN");
        // Preserve the fault observation before a recovered Stop/Disarm
        // publication replaces it. HOLD already records this at its boundary.
        if (m_state == NCState::RUN && !m_gapDryRun.paused)
            LogEDMZFixtureSameThread("RT_STOP", "FAULT", "UNEXPECTED_FIXTURE_FAILURE");
    }
    if (controlGap && (m_edmGapShortReplan.active || m_edmGapFeedUpdate.shortState.active || m_edmGapRapid.shortState.active)) LogEDMZFixtureSameThread("CONTROL_GAP", "FAIL", "CONTROL_DEADMAN_50MS");
    if (controlGap && !m_edmGapShortReplan.active && !m_edmGapFeedUpdate.shortState.active && !m_edmGapRapid.shortState.active)
    {
        const bool gapValid = failureClockValid && failureNow >= m_edmZFixture.lastClockMs;
        const bool cachedAgeValid = failureClockValid && m_edmZFixture.lastFreshMs != 0ULL &&
            failureNow >= m_edmZFixture.lastFreshMs;
        const bool preCallbackValid = failureClockValid && m_edmProcessObservedGap.ownerClockValid &&
            failureNow >= m_edmProcessObservedGap.observedAtMs;
        // This is elapsed NC time since the most recent GAP observation, not
        // a measurement of any individual log call or a new RT publication.
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM35-FIX2] event=CONTROL_GAP lastServiceMs=%llu failureNowMs=%llu serviceGapMs=%llu clockValid=%u gapValid=%u observedGapMs=%llu gapClockValid=%u preCallbackMs=%llu preCallbackValid=%u cachedPub=%llu cachedTick=%llu cachedUs=%llu cachedReadAgeMs=%llu cachedReadAgeValid=%u feedback=CACHED stopRequested=1\n",
            static_cast<unsigned long long>(m_edmZFixture.lastClockMs),
            static_cast<unsigned long long>(failureNow),
            static_cast<unsigned long long>(gapValid ? failureNow - m_edmZFixture.lastClockMs : 0ULL),
            failureClockValid ? 1U : 0U, gapValid ? 1U : 0U,
            static_cast<unsigned long long>(m_edmProcessObservedGap.observedAtMs),
            m_edmProcessObservedGap.ownerClockValid ? 1U : 0U,
            static_cast<unsigned long long>(preCallbackValid ? failureNow - m_edmProcessObservedGap.observedAtMs : 0ULL),
            preCallbackValid ? 1U : 0U,
            static_cast<unsigned long long>(m_edmZFixture.feedback.publicationSequence),
            static_cast<unsigned long long>(m_edmZFixture.feedback.sampledTick),
            static_cast<unsigned long long>(m_edmZFixture.feedback.monotonicUs),
            static_cast<unsigned long long>(cachedAgeValid ? failureNow - m_edmZFixture.lastFreshMs : 0ULL),
            cachedAgeValid ? 1U : 0U);
        if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM35-FIX2] event=CONTROL_GAP reason=LOG_FORMAT action=PRIORITY_STOP_REQUESTED\n");
        if (EDM28::IsGapServoFlushProfile(m_edmZFixture.config.profile))
        {
            const bool costValid = failureClockValid && precedingLogEndMs != 0ULL &&
                failureNow >= precedingLogEndMs && precedingLogDurationMs <= precedingLogEndMs;
            const int costLength = std::snprintf(line, sizeof(line),
                "[EDM60-FIX1] event=CONTROL_GAP_LOG_COST diagnosticEndMs=%llu durationMs=%llu ageMs=%llu valid=%u observationOnly=1 stopRequested=1 discharge=0\n",
                static_cast<unsigned long long>(precedingLogEndMs),
                static_cast<unsigned long long>(precedingLogDurationMs),
                static_cast<unsigned long long>(costValid ? failureNow - precedingLogEndMs : 0ULL),
                costValid ? 1U : 0U);
            if (costLength >= 0 && static_cast<std::size_t>(costLength) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM60-FIX1] event=CONTROL_GAP_LOG_COST reason=LOG_FORMAT stopRequested=1 discharge=0\n");
        }
    }
    const bool mayBeArmed = m_edmZFixture.feedback.capHeld ||
        (m_edmZFixture.originPinned && m_edmZFixture.feedback.state != EDM28::State::Disarmed) ||
        (m_edmZFixture.pending && m_edmZFixture.pendingKind == EDM28::RequestKind::Arm);
    // A submitted Disarm may finish while cancellation detaches this run.
    // Keep its exact pending identity so cleanup can consume the existing
    // same-scope/sequence/source/stop-proof receipt; do not issue a replacement.
    const bool retainPendingDisarm = m_edmZFixture.pending &&
        m_edmZFixture.pendingKind == EDM28::RequestKind::Disarm &&
        m_edmZFixture.pendingSequence != 0ULL && m_edmZFixture.scope.session != 0ULL;
    m_gapDryRun.active = false; m_gapDryRun.paused = false;
    m_gapDryRun.result = result && std::strcmp(result, "FAIL") == 0 ? 3U : 2U;
    m_edmZFixture.phase = EDMZFixturePhase::Cancelled; m_edmZFixture.pending = retainPendingDisarm;
    if (m_edmZFixture.operatorWindowActive)
    { m_edmZFixture.operatorWindowActive = false; m_edmZFixture.operatorWindowConsumed = true;
      LogEDMPersistentShortOperatorWindowSameThread("EXIT", "CANCELLED"); }
    m_edmZFixture.cleanupPending = m_edmZFixture.scope.session != 0ULL && (mayBeArmed || retainPendingDisarm);
    LogEDMZFixtureSameThread("SUMMARY", result, reason);
}

bool NCManager::RearmEDMZFixtureSameThread(std::uint64_t nowMs, bool restart) noexcept
{
    if (!IsEDMZFixtureStoppedSameThread()) return false;
    if (restart)
    {
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        m_edmZFixture.scope.executionEpoch = m_gapDryRun.epoch;
        m_edmZFixture.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        m_edmZFixture.scope.ownerGeneration = m_gapDryRun.lease.generation;
        // The previous window includes time spent in HOLD. Retire it before
        // ArmPending so it cannot expire ahead of the fresh Arm receipt.
        // The exact Arm application starts the new full window below;
        // its existing 50ms application deadline remains in force.
        m_edmZFixture.windowStartMs = 0ULL;
        m_edmZFixture.beginPending = true; m_edmZFixture.armNextPhase = EDMZFixturePhase::Positive;
        m_edmZFixture.cycles = 0U; m_edmZFixture.shortDone = false; m_gapDryRun.paused = false;
        m_edmZFixture.simulatedShortStops = m_edmZFixture.verifiedShortClears = m_edmZFixture.verifiedRetreats = 0U;
        m_edmZFixture.shortStopMaxPulse = 0.0;
        m_edmZFixture.shortStopProof = m_edmZFixture.shortStopProofTick = 0ULL;
        m_edmZFixture.shortStopAppliedSequence = m_edmZFixture.shortStopAppliedTick = m_edmZFixture.shortStopAppliedMonotonicUs = 0ULL;
        if (m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat)
        {
            // A HOLD retires this isolated SIM script; it does not earn a clear.
            // A new test must first reach Positive with fresh position proof.
            m_edmZFixture.gapShort.Reset(); m_edmZFixture.gapShortLow = false;
            m_edmZFixture.gapClearProof = m_edmZFixture.gapClearAppliedSequence = 0ULL;
            m_edmZFixture.gapClearAppliedTick = m_edmZFixture.gapClearAppliedUs = 0ULL;
            m_edmZFixture.gapClearPublication = m_edmZFixture.gapClearTick = m_edmZFixture.gapClearUs = m_edmZFixture.gapClearHighUs = 0ULL;
            m_edmZFixture.gapClearTargetPulse = 0.0;
        }
    }
    else
    {
        if (m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat &&
            (!m_edmZFixture.gapShortLow || !m_edmZFixture.gapShort.Snapshot().valid ||
                !m_edmZFixture.gapShort.Snapshot().shortActive || !m_edmZFixture.gapShort.Snapshot().feedInhibited))
        { EndEDMZFixtureSameThread("FAIL", "P21_RETREAT_WITHOUT_ACTIVE"); return false; }
        const bool retreatShort = m_edmZFixture.config.profile == EDM28::Profile::P16UnloadedRetreat ||
            EDM28::IsSpeedRetreatProfile(m_edmZFixture.config.profile);
        if (m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat || retreatShort)
        {
            if (!m_edmZFixture.shortDone ||
                m_edmZFixture.verifiedShortClears == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.verifiedShortClears + 1U)
            { EndEDMZFixtureSameThread("FAIL", "SHORT_STOP_COUNT_MISMATCH"); return false; }
            const auto& stopped = m_edmZFixture.feedback;
            m_edmZFixture.shortStopProof = stopped.proofSequence;
            m_edmZFixture.shortStopProofTick = stopped.sampledTick;
            m_edmZFixture.shortStopAppliedSequence = stopped.stopAppliedSequence;
            m_edmZFixture.shortStopAppliedTick = stopped.stopAppliedTick;
            m_edmZFixture.shortStopAppliedMonotonicUs = stopped.stopAppliedMonotonicUs;
            if (retreatShort)
            {
                const double stoppedMax = (std::fmax)(stopped.actualPulse, (std::fmax)(stopped.commandPulse, stopped.planningPulse));
                const double retreatTarget = m_edmZFixture.originPulse + .02 * m_edmZFixture.config.pulsePerMm;
                const double margin = 2.0 * .001 * m_edmZFixture.config.pulsePerMm + 1.0;
                if (!std::isfinite(stopped.actualPulse) || !std::isfinite(stopped.commandPulse) || !std::isfinite(stopped.planningPulse) ||
                    !std::isfinite(stoppedMax) || !std::isfinite(retreatTarget) ||
                    m_edmZFixture.verifiedRetreats != m_edmZFixture.verifiedShortClears ||
                    m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles || !(retreatTarget - stoppedMax > margin))
                { EndEDMZFixtureSameThread("FAIL", "RETREAT_DIRECTION_NOT_PROVEN"); return false; }
                m_edmZFixture.shortStopMaxPulse = stoppedMax;
            }
        }
        m_edmZFixture.armNextPhase = retreatShort ? EDMZFixturePhase::Retreat : EDMZFixturePhase::Negative;
    }
    m_edmZFixture.phaseStartMs = nowMs;
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
    { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
    m_edmZFixture.phase = EDMZFixturePhase::ArmPending;
    return true;
}

void NCManager::ServiceEDMZFixtureCleanupSameThread() noexcept
{
    auto& fixture = m_edmZFixture;
    if (!fixture.cleanupPending) return;
    const auto count = [](std::uint32_t& value) noexcept
    { if (value != (std::numeric_limits<std::uint32_t>::max)()) ++value; };
    count(fixture.cleanupPolls);
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { count(fixture.cleanupReadBlocked); fixture.cleanupLastBlock = 1U; return; }
    const auto& feedback = fixture.feedback;
    if (!EDM28::SameScope(feedback.scope, fixture.scope))
    { count(fixture.cleanupScopeBlocked); fixture.cleanupLastBlock = 2U; return; }
    if (feedback.queuedCancellation && feedback.state == EDM28::State::Disarmed && !feedback.capHeld &&
        !fixture.originPinned && !fixture.motionApplied)
    { fixture.cleanupPending = false; fixture.pending = false; fixture.cleanupLastBlock = 7U; return; }
    if (fixture.pendingKind == EDM28::RequestKind::Disarm && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        feedback.state == EDM28::State::Disarmed && !feedback.capHeld && feedback.stopProven)
    {
        if (feedback.latchedFault) LogEDMZFixtureSameThread("RT_STOP", "DISARMED", "FAULT_STOP_CLEANUP");
        fixture.cleanupPending = false; fixture.pending = false; fixture.cleanupLastBlock = 7U;
        if (fixture.cleanupTimeoutLogged) LogEDMZFixtureCleanupSameThread("LATE_COMPLETE");
        return;
    }
    if (fixture.pending)
    { count(fixture.cleanupWaitReceipt); fixture.cleanupLastBlock = 4U; return; }
    if (!IsEDMZFixtureStoppedSameThread())
    { count(fixture.cleanupWaitStop); fixture.cleanupLastBlock = 3U; return; }
    if (SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
    { count(fixture.cleanupSubmitted); fixture.cleanupLastBlock = 6U; }
    else
    { count(fixture.cleanupSubmitFailed); fixture.cleanupLastBlock = 5U; }
}

void NCManager::LogEDMZFixtureCleanupSameThread(const char* stage) noexcept
{
    const auto& fixture = m_edmZFixture;
    // Bounded NC-side snapshot at the existing timeout/completion boundary.
    // Raw feedback is diagnostic only: a rejected read never becomes a proof.
    // SERVICE counters cover history; the other parts share one raw snapshot.
    EDM28::Feedback sample{};
    const bool read = m_motion.ReadEDMZFixtureFeedback(sample);
    if (!read) sample = EDM28::Feedback{};
    const bool sameScope = read && EDM28::SameScope(sample.scope, fixture.scope);
    const double delta = sample.actualPulse - sample.commandPulse;
    const bool errorValid = read && std::isfinite(delta) && std::isfinite(sample.pulsePerMm) && sample.pulsePerMm > 0.0 &&
        std::isfinite(delta / sample.pulsePerMm);
    const double errorMm = errorValid ? delta / sample.pulsePerMm : 0.0;
    const unsigned long long test = static_cast<unsigned long long>(m_gapDryRun.test);
    const unsigned long long session = static_cast<unsigned long long>(fixture.scope.session);
    const unsigned long long run = static_cast<unsigned long long>(m_gapDryRun.run);
    const char* label = stage ? stage : "UNKNOWN";
    char line[512]{};
    const auto emit = [&](int size, const char* part) noexcept
    {
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM45-CLEANUP] stage=%.16s part=%s test=%llu session=%llu run=%llu reason=LOG_FORMAT diagnosticOnly=1\n",
            label, part, test, session, run);
    };
    int size = std::snprintf(line, sizeof(line),
        "[EDM45-CLEANUP] stage=%.16s part=SERVICE test=%llu session=%llu run=%llu nc=%u alarm=%u cleanup=%u reset=%u lastBlock=%u polls=%u readBlocked=%u scopeBlocked=%u waitStop=%u waitReceipt=%u submitFailed=%u submitted=%u\n",
        label, test, session, run, static_cast<unsigned>(m_state.load(std::memory_order_acquire)), AlarmManager::GetInstance().HasAlarm() ? 1U : 0U,
        fixture.cleanupPending ? 1U : 0U, fixture.resetPending ? 1U : 0U,
        static_cast<unsigned>(fixture.cleanupLastBlock), static_cast<unsigned>(fixture.cleanupPolls),
        static_cast<unsigned>(fixture.cleanupReadBlocked), static_cast<unsigned>(fixture.cleanupScopeBlocked),
        static_cast<unsigned>(fixture.cleanupWaitStop), static_cast<unsigned>(fixture.cleanupWaitReceipt),
        static_cast<unsigned>(fixture.cleanupSubmitFailed), static_cast<unsigned>(fixture.cleanupSubmitted));
    emit(size, "SERVICE");
    size = std::snprintf(line, sizeof(line),
        "[EDM45-CLEANUP] stage=%.16s part=SOURCE test=%llu session=%llu run=%llu rawRead=%u samplePub=%llu sampleTick=%llu sampleUs=%llu cachedPub=%llu cachedTick=%llu sameScope=%u source=%u pdo=%u contiguous=%u clock=%u axis=%u linear=%u servo=%u mode=%u fault=%u hard=%u/%u\n",
        label, test, session, run, read ? 1U : 0U,
        static_cast<unsigned long long>(sample.publicationSequence), static_cast<unsigned long long>(sample.sampledTick),
        static_cast<unsigned long long>(sample.monotonicUs), static_cast<unsigned long long>(fixture.feedback.publicationSequence),
        static_cast<unsigned long long>(fixture.feedback.sampledTick), sameScope ? 1U : 0U,
        sample.sourceFresh ? 1U : 0U, sample.pdoValid ? 1U : 0U, sample.contiguous ? 1U : 0U,
        sample.clockValid ? 1U : 0U, sample.axisExists ? 1U : 0U, sample.linear ? 1U : 0U,
        sample.servoReady ? 1U : 0U, sample.modeReady ? 1U : 0U, sample.fault ? 1U : 0U,
        sample.hardPositive ? 1U : 0U, sample.hardNegative ? 1U : 0U);
    emit(size, "SOURCE");
    size = std::snprintf(line, sizeof(line),
        "[EDM45-CLEANUP] stage=%.16s part=RECEIPT test=%llu session=%llu run=%llu state=%u reason=%u ack=%u pending=%u kind=%u seq=%llu request=%llu applied=%llu/%u/%llu stop=%llu/%llu proof=%llu epoch=%u/%u owner=%u/%u:%u/%u\n",
        label, test, session, run, static_cast<unsigned>(sample.state), static_cast<unsigned>(sample.reason),
        static_cast<unsigned>(sample.ack), fixture.pending ? 1U : 0U, static_cast<unsigned>(fixture.pendingKind),
        static_cast<unsigned long long>(fixture.pendingSequence), static_cast<unsigned long long>(sample.lastRequestSequence),
        static_cast<unsigned long long>(sample.lastAppliedSequence), static_cast<unsigned>(sample.lastAppliedKind),
        static_cast<unsigned long long>(sample.lastAppliedTick), static_cast<unsigned long long>(sample.stopAppliedSequence),
        static_cast<unsigned long long>(sample.stopAppliedTick), static_cast<unsigned long long>(sample.proofSequence),
        static_cast<unsigned>(sample.currentEpoch), static_cast<unsigned>(fixture.scope.executionEpoch),
        static_cast<unsigned>(sample.currentOwner), static_cast<unsigned>(sample.currentOwnerGeneration),
        static_cast<unsigned>(fixture.scope.owner), static_cast<unsigned>(fixture.scope.ownerGeneration));
    emit(size, "RECEIPT");
    size = std::snprintf(line, sizeof(line),
        "[EDM45-CLEANUP] stage=%.16s part=MOTION test=%llu session=%llu run=%llu frame=%u cap=%u zero=%u latched=%u idle=%u stopped=%u dwell=%u/%u actual=%.9g command=%.9g planning=%.9g errorValid=%u errorMm=%.9g toleranceMm=%.9g cmdMmS=%.9g pdoMmS=%.9g actualPps=%.9g\n",
        label, test, session, run, sample.frameCurrent ? 1U : 0U, sample.capHeld ? 1U : 0U,
        sample.forceZeroOutput ? 1U : 0U, sample.latchedFault ? 1U : 0U, sample.axisIdle ? 1U : 0U,
        sample.stopProven ? 1U : 0U, static_cast<unsigned>(sample.stableCycles), static_cast<unsigned>(fixture.config.settleCycles),
        sample.actualPulse, sample.commandPulse, sample.planningPulse, errorValid ? 1U : 0U, errorMm,
        fixture.config.positionToleranceMm, sample.cmdSpeedMmS, sample.pdoSpeedMmS, sample.actualVelocityPps);
    emit(size, "MOTION");
}

void NCManager::ServiceEDMZFixtureFailureAlarmSameThread() noexcept
{
    auto& fixture = m_edmZFixture;
    if (!fixture.failureAlarmPending) return;
    // An explicit Reset supersedes this deferred notification. A specific
    // alarm (including P17's 4001) or shutdown owns its existing stop path.
    if (fixture.resetPending || fixture.resetReleased || m_state == NCState::RESET_STATE ||
        m_resetContinuationPhase != ResetContinuationPhase::IDLE || m_resetSafetyOutputHoldActive ||
        AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM || Close_System_Com_flag)
    { fixture.failureAlarmPending = false; return; }
    std::uint64_t now = fixture.failureStartMs;
    const bool clock = ReadGapDryRunClockSameThread(now);
    if (fixture.failurePolls < 100U) ++fixture.failurePolls;
    const bool clockInvalid = !clock || now < fixture.failureStartMs;
    const bool timedOut = clockInvalid || now - fixture.failureStartMs >= 1000ULL || fixture.failurePolls >= 100U;
    if (fixture.cleanupPending && !timedOut) return;
    // ServiceEDMZFixtureCleanupSameThread only releases an armed session
    // after its exact same-scope stopped Disarm receipt. If proof remains
    // unavailable, a bounded fallback enters the ordinary Alarm emergency
    // path; never manufacture cleanup completion, READY or M30 success.
    const char* stage = !fixture.cleanupPending ? "CLEANUP_COMPLETE" : clockInvalid ? "CLOCK_INVALID" : "CLEANUP_TIMEOUT";
    fixture.failureAlarmPending = false; // Clear before ChangeState cancellation.
    if (fixture.cleanupPending)
    {
        fixture.cleanupTimeoutLogged = true;
        LogEDMZFixtureCleanupSameThread(stage);
    }
    char line[480]{};
    const int size = std::snprintf(line, sizeof(line),
        "[EDM35-FIX1] event=FAILURE_ALARM stage=%s reason=%.47s rtReason=%u alarm=2020 test=%llu session=%llu run=%llu profile=%u cleanup=%u polls=%u elapsedMs=%llu action=RESET_REQUIRED physicalPermit=0 discharge=0\n",
        stage, fixture.failureReason, static_cast<unsigned>(fixture.failureRtReason),
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(fixture.scope.session),
        static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned>(fixture.config.profile),
        fixture.cleanupPending ? 1U : 0U, static_cast<unsigned>(fixture.failurePolls),
        static_cast<unsigned long long>(clockInvalid ? 0ULL : now - fixture.failureStartMs));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM35-FIX1] event=FAILURE_ALARM stage=LOG_FORMAT alarm=2020 action=RESET_REQUIRED\n");
    AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
    ChangeState(NCState::ALARM);
}

bool NCManager::AdmitEDMZFixtureResetSameThread() noexcept
{
    m_edmZFixture.failureAlarmPending = false;
    if (m_edmZFixture.resetReleased)
    { m_edmZFixture.resetReleased = false; m_edmZFixture.resetPending = false; m_edmZFixture.resetBypass = true; return true; }
    if (m_edmZFixture.resetPending) return false;
    if (!m_gapDryRun.physicalZFixtureTest || (!m_gapDryRun.active && !m_edmZFixture.cleanupPending && m_gapDryRun.result != 1U)) return true;
    if (m_gapDryRun.active || m_gapDryRun.result == 1U)
    {
        m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
        EndEDMZFixtureSameThread("CANCELLED", "RESET");
    }
    if (!m_edmZFixture.cleanupPending) return true;
    m_edmZFixture.resetPending = true; m_edmZFixture.resetPolls = 0U;
    std::uint64_t now = m_edmZFixture.lastClockMs;
    (void)ReadGapDryRunClockSameThread(now); m_edmZFixture.resetStartMs = now;
    m_state = NCState::RESET_STATE;
    return false;
}

bool NCManager::PollEDMZFixtureResetSameThread() noexcept
{
    if (!m_edmZFixture.resetPending) return false;
    ServiceEDMZFixtureCleanupSameThread();
    std::uint64_t now = m_edmZFixture.resetStartMs;
    const bool clock = ReadGapDryRunClockSameThread(now);
    if (m_edmZFixture.resetPolls < 100U) ++m_edmZFixture.resetPolls;
    const bool timeout = !clock || now < m_edmZFixture.resetStartMs || now - m_edmZFixture.resetStartMs >= 1000ULL ||
        m_edmZFixture.resetPolls >= 100U || AlarmManager::GetInstance().HasAlarm() || Close_System_Com_flag ||
        m_motion.HasPendingSafetyOrRecoveryRequests();
    if (!m_edmZFixture.cleanupPending || timeout)
    { m_edmZFixture.resetReleased = true; return true; }
    return false;
}

// EDM38 guards are additive: the accepted P17 path keeps its original proofs.
bool NCManager::IsEDMGapPersistentConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm &&
            a.referenceOffsetPulse == b.referenceOffsetPulse && a.hardwareSign == b.hardwareSign &&
            a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS &&
            a.accelerationTimeSec == b.accelerationTimeSec && a.decelerationTimeSec == b.decelerationTimeSec &&
            a.followingLimitMm == b.followingLimitMm && a.positionToleranceMm == b.positionToleranceMm &&
            a.excursionToleranceMm == b.excursionToleranceMm && a.heartbeatTicks == b.heartbeatTicks &&
            a.maximumIssueAgeTicks == b.maximumIssueAgeTicks && a.stopTimeoutTicks == b.stopTimeoutTicks &&
            a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapPersistentProfile && c.profile == EDM28::Profile::P22GapPersistentShort &&
        c.axisIndex == 2U && std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 5.0 && c.pdoCapMmS == .1 && c.outerHalfMm == .1 && c.targetHalfMm == .02 &&
        c.accelerationTimeSec == .2 && c.decelerationTimeSec == .2 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 &&
        c.heartbeatTicks == 200U && c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapPersistentConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapPersistentActiveSameThread() const noexcept
{
    const auto& state = m_edmZFixture; const auto& gap = state.gapShort.Snapshot();
    return IsEDMGapPersistentConfigValidSameThread(true) && state.persistentShortLatched && state.shortSourceTest &&
        state.gapShortLow && state.gapPersistentEntryUs && gap.valid && gap.fresh &&
        gap.fault == EDM37::GapShortFault::None && gap.feedInhibited && gap.shortActive &&
        gap.state == EDMGapServo::ShortState::Active && gap.voltage == 20.0 &&
        gap.sampleTick == state.feedback.sampledTick && gap.sampleMonotonicUs == state.feedback.monotonicUs &&
        gap.sampleMonotonicUs >= state.gapPersistentEntryUs && gap.sampleMonotonicUs - state.gapPersistentEntryUs >= 2000ULL;
}

bool NCManager::ObserveEDMGapPersistentShortSameThread(bool restart) noexcept
{
    auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    // The caller proves Held/terminal Disarmed plus current authority before
    // restarting this observation chain. Latched intent and origin never reset.
    if (restart && !state.gapPersistentResumeObserved)
    {
        const bool terminal = state.endpointProofKind && feedback.state == EDM28::State::Disarmed && !feedback.capHeld;
        const bool held = IsEDMZFixtureStoppedSameThread() && feedback.state == EDM28::State::Held &&
            feedback.lastAppliedKind == EDM28::RequestKind::Stop && feedback.lastAppliedSequence == feedback.stopAppliedSequence &&
            feedback.lastAppliedTick == feedback.stopAppliedTick && feedback.lastAppliedMonotonicUs == feedback.stopAppliedMonotonicUs &&
            feedback.stopAppliedMonotonicUs && feedback.sampledTick >= feedback.stopAppliedTick &&
            feedback.sampledTick - feedback.stopAppliedTick >= state.config.settleCycles &&
            feedback.monotonicUs >= feedback.stopAppliedMonotonicUs &&
            feedback.monotonicUs - feedback.stopAppliedMonotonicUs >= state.config.settleCycles * EDM28::CycleUs;
        if (!m_gapDryRun.paused || !IsEDMGapPersistentConfigValidSameThread(true) ||
            !IsEDMZFixtureAuthorityCurrentSameThread(true) ||
            feedback.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            feedback.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) ||
            feedback.currentOwnerGeneration != m_programMotionLease.generation ||
            !(terminal ? IsEDMPersistentShortReleasedSameThread() : held))
        { EndEDMZFixtureSameThread("FAIL", "P22_RESUME_STOP_PROOF_INVALID"); return false; }
        state.gapShort.Reset(); state.gapPersistentEntryUs = 0ULL;
        state.gapShortLow = state.persistentShortLatched;
        state.gapPersistentResumeObserved = true;
    }
    const bool inject = !restart && state.phase == EDMZFixturePhase::Negative && !state.shortDone &&
        IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) &&
        feedback.sampledTick - feedback.lastAppliedTick >= 800ULL;
    if (inject)
    {
        if (feedback.state != EDM28::State::Moving || feedback.targetProven || feedback.axisIdle)
        { EndEDMZFixtureSameThread("FAIL", "SHORT_TEST_REQUIRES_MOVING_LEG"); return false; }
        if (state.persistentShortLatched || state.gapShortLow || state.shortSourceTest || state.cycles ||
            state.simulatedShortStops || state.verifiedRetreats || state.verifiedShortClears)
        { EndEDMZFixtureSameThread("FAIL", "SHORT_INJECTION_COUNT_MISMATCH"); return false; }
        state.gapShortLow = true;
    }
    if (!inject && state.gapShortLow != state.persistentShortLatched)
    { EndEDMZFixtureSameThread("FAIL", "P22_SIM_LATCH_MISMATCH"); return false; }
    const auto& gap = state.gapShort.Observe(feedback.sampledTick, feedback.monotonicUs, state.gapShortLow ? 20.0 : 50.0);
    if (!gap.valid || !gap.fresh)
    { EndEDMZFixtureSameThread("FAIL", "P22_GAP_SAMPLE_INVALID"); return false; }
    if (state.gapShortLow && !state.gapPersistentEntryUs) state.gapPersistentEntryUs = gap.sampleMonotonicUs;
    if (inject)
    {
        if (!gap.feedInhibited || gap.shortActive || gap.state != EDMGapServo::ShortState::Entering)
        { EndEDMZFixtureSameThread("FAIL", "P22_SHORT_ENTRY_INVALID"); return false; }
        // The first feed-inhibited sample publishes Stop before logs/heartbeat.
        m_motion.RequestEDMZFixtureStop(state.scope.session);
        state.persistentShortLatched = true; state.shortSourceTest = m_gapDryRun.test;
        state.simulatedShortStops = 1U; state.shortDone = true;
        state.phase = EDMZFixturePhase::ShortStop; state.pending = false; state.phaseStartMs = state.lastClockMs;
        LogEDMZFixtureSameThread("SHORT_STOP", "WAIT", "DETECTOR_ENTERING_PRIORITY_STOP"); return false;
    }
    if (state.persistentShortLatched)
    {
        // Waiting for the 2 ms detector dwell is allowed only while the
        // original latched identity and once-only accounting remain intact.
        if (!state.shortSourceTest || !state.shortDone || state.cycles || state.simulatedShortStops != 1U ||
            state.verifiedShortClears || state.endpointProofKind > 2U ||
            state.verifiedRetreats != (state.endpointProofKind == 2U ? 1U : 0U))
        { EndEDMZFixtureSameThread("FAIL", "P22_LATCH_COUNT_MISMATCH"); return false; }
        if (!gap.feedInhibited || gap.voltage != 20.0 ||
            (gap.state != EDMGapServo::ShortState::Entering && gap.state != EDMGapServo::ShortState::Active))
        { EndEDMZFixtureSameThread("FAIL", "P22_PERSISTENT_SAMPLE_CLEARED"); return false; }
        if (!restart && state.phase != EDMZFixturePhase::ShortStop && !IsEDMGapPersistentActiveSameThread())
        { EndEDMZFixtureSameThread("FAIL", "P22_ACTIVE_PROOF_REQUIRED"); return false; }
    }
    else if (gap.feedInhibited || gap.shortActive || gap.voltage != 50.0)
    { EndEDMZFixtureSameThread("FAIL", "P22_UNLATCHED_SAMPLE_INVALID"); return false; }
    return true;
}

// P17 is an isolated expected-alarm fixture. Its latched simulated short is
// never converted to a normal gap sample by re-arm, HOLD or endpoint arrival.
bool NCManager::PrepareEDMPersistentShortAlarmSameThread(bool positionProof) noexcept
{
    if (m_edmZFixture.gapPersistentProfile && !IsEDMGapPersistentActiveSameThread())
    { EndEDMZFixtureSameThread("FAIL", "P22_ALARM_WITHOUT_ACTIVE"); return false; }
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    const double tolerance = .001 * state.config.pulsePerMm;
    const bool endpoint = state.originPinned && feedback.originValid && feedback.outerOriginPulse == state.originPulse &&
        std::isfinite(state.originPulse) && std::isfinite(tolerance) && tolerance > 0.0 &&
        std::isfinite(feedback.actualPulse) && std::isfinite(feedback.commandPulse) && std::isfinite(feedback.planningPulse) &&
        std::fabs(feedback.actualPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.commandPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.planningPulse - state.originPulse) <= tolerance;
    const bool counts = state.persistentShortLatched && state.shortSourceTest != 0ULL && state.shortDone &&
        state.cycles == 0U && state.simulatedShortStops == 1U && state.verifiedShortClears == 0U && state.verifiedRetreats == 0U;
    const bool stopStamp = state.shortStopProof != 0ULL && state.shortStopAppliedSequence != 0ULL &&
        state.shortStopAppliedTick != 0ULL && state.shortStopAppliedMonotonicUs != 0ULL &&
        feedback.stopAppliedSequence == state.shortStopAppliedSequence && feedback.stopAppliedTick == state.shortStopAppliedTick &&
        feedback.stopAppliedMonotonicUs == state.shortStopAppliedMonotonicUs;
    bool proved = endpoint && counts && stopStamp && !feedback.latchedFault && !feedback.forceZeroOutput &&
        EDM28::SameScope(feedback.scope, state.scope) && feedback.frameCurrent && feedback.axisIdle &&
        feedback.stableCycles >= state.config.settleCycles;
    if (positionProof)
        proved = proved && state.phase == EDMZFixturePhase::Retreat && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) &&
            feedback.state == EDM28::State::AtTarget && feedback.targetProven && std::isfinite(feedback.targetPulse) &&
            std::fabs(feedback.targetPulse - state.originPulse) <= tolerance && std::isfinite(state.shortStopMaxPulse) &&
            feedback.actualPulse > state.shortStopMaxPulse + tolerance && feedback.proofSequence > state.shortStopProof &&
            feedback.lastAppliedTick > state.shortStopProofTick && feedback.lastAppliedMonotonicUs > state.shortStopAppliedMonotonicUs &&
            feedback.sampledTick >= feedback.lastAppliedTick && feedback.sampledTick - feedback.lastAppliedTick >= state.config.settleCycles &&
            feedback.monotonicUs >= feedback.lastAppliedMonotonicUs &&
            feedback.monotonicUs - feedback.lastAppliedMonotonicUs >= state.config.settleCycles * EDM28::CycleUs;
    else
        proved = proved && IsEDMZFixtureStoppedSameThread() && feedback.proofSequence == state.shortStopProof &&
            feedback.lastAppliedKind == EDM28::RequestKind::Stop && feedback.lastAppliedSequence == state.shortStopAppliedSequence &&
            feedback.lastAppliedTick == state.shortStopAppliedTick && feedback.lastAppliedMonotonicUs == state.shortStopAppliedMonotonicUs &&
            feedback.sampledTick >= state.shortStopAppliedTick && feedback.sampledTick - state.shortStopAppliedTick >= state.config.settleCycles &&
            feedback.monotonicUs >= state.shortStopAppliedMonotonicUs &&
            feedback.monotonicUs - state.shortStopAppliedMonotonicUs >= state.config.settleCycles * EDM28::CycleUs;
    if (!proved) { EndEDMZFixtureSameThread("FAIL", "P17_ORIGIN_ALARM_PROOF_INVALID"); return false; }
    m_edmZFixture.endpointProofKind = positionProof ? 2U : 1U;
    if (positionProof) ++m_edmZFixture.verifiedRetreats;
    m_edmZFixture.endpointProofSequence = feedback.proofSequence;
    m_edmZFixture.endpointAppliedSequence = feedback.lastAppliedSequence;
    m_edmZFixture.endpointAppliedTick = feedback.lastAppliedTick;
    m_edmZFixture.endpointAppliedMonotonicUs = feedback.lastAppliedMonotonicUs;
    LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "SIM20_ORIGIN_PROVEN_STOP_NEXT");
    m_edmZFixture.pending = false; m_edmZFixture.phaseStartMs = m_edmZFixture.lastClockMs;
    if (positionProof)
    { m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session); m_edmZFixture.phase = EDMZFixturePhase::FinalStop; }
    else
    {
        m_edmZFixture.receiptProof = feedback.proofSequence;
        m_edmZFixture.terminalStopAppliedSequence = feedback.stopAppliedSequence;
        m_edmZFixture.terminalStopAppliedTick = feedback.stopAppliedTick;
        m_edmZFixture.terminalStopAppliedMonotonicUs = feedback.stopAppliedMonotonicUs;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        m_edmZFixture.phase = EDMZFixturePhase::Disarm;
    }
    return true;
}

bool NCManager::AlarmEDMPersistentShortAtOriginSameThread(bool positionProof) noexcept
{
    if (m_edmZFixture.gapPersistentProfile && !IsEDMGapPersistentActiveSameThread())
    { EndEDMZFixtureSameThread("FAIL", "P22_ALARM_WITHOUT_ACTIVE"); return false; }
    const auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    const double tolerance = .001 * state.config.pulsePerMm;
    const bool proved = state.phase == EDMZFixturePhase::Receipt && state.persistentShortLatched && state.shortSourceTest &&
        state.endpointProofKind == (positionProof ? 2U : 1U) && state.endpointProofSequence && state.endpointAppliedSequence &&
        state.endpointAppliedTick && state.endpointAppliedMonotonicUs && state.cycles == 0U && state.simulatedShortStops == 1U &&
        state.verifiedShortClears == 0U && state.verifiedRetreats == (positionProof ? 1U : 0U) &&
        state.originPinned && feedback.outerOriginPulse == state.originPulse && feedback.frameCurrent &&
        EDM28::SameScope(feedback.scope, state.scope) && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        feedback.state == EDM28::State::Disarmed && !feedback.capHeld && feedback.stopProven && !feedback.latchedFault &&
        !feedback.forceZeroOutput && feedback.axisIdle && feedback.stableCycles >= state.config.settleCycles &&
        feedback.proofSequence == state.receiptProof && feedback.proofSequence >= state.endpointProofSequence &&
        (!positionProof || (feedback.proofSequence > state.endpointProofSequence &&
            state.terminalStopAppliedTick > state.endpointAppliedTick &&
            state.terminalStopAppliedMonotonicUs > state.endpointAppliedMonotonicUs)) &&
        feedback.stopAppliedSequence == state.terminalStopAppliedSequence && feedback.stopAppliedTick == state.terminalStopAppliedTick &&
        feedback.stopAppliedMonotonicUs == state.terminalStopAppliedMonotonicUs && state.terminalStopAppliedSequence &&
        state.terminalStopAppliedTick && state.terminalStopAppliedMonotonicUs &&
        feedback.lastAppliedTick > state.terminalStopAppliedTick && feedback.lastAppliedMonotonicUs > state.terminalStopAppliedMonotonicUs &&
        feedback.publicationSequence > state.receiptPublication && feedback.sampledTick > state.receiptTick &&
        feedback.sampledTick > feedback.lastAppliedTick && feedback.monotonicUs > feedback.lastAppliedMonotonicUs &&
        std::isfinite(tolerance) && tolerance > 0.0 && std::isfinite(feedback.targetPulse) && std::isfinite(state.originPulse) &&
        std::isfinite(feedback.actualPulse) && std::isfinite(feedback.commandPulse) && std::isfinite(feedback.planningPulse) &&
        std::fabs(feedback.actualPulse - state.originPulse) <= tolerance && std::fabs(feedback.commandPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.planningPulse - state.originPulse) <= tolerance &&
        (!positionProof || std::fabs(feedback.targetPulse - state.originPulse) <= tolerance) &&
        std::fabs(feedback.cmdSpeedMmS) <= 1.0 / state.config.pulsePerMm;
    if (!proved) { EndEDMZFixtureSameThread("FAIL", "P17_FINAL_ALARM_RECEIPT_INVALID"); return false; }
    // The exact stopped Disarm and a later coherent sample precede Trigger:
    // alarm-bearing PDO observations cannot earn a fresh RT source proof.
    m_edmZFixture.pending = false;
    LogEDMZFixtureSameThread("EXPECTED_ALARM", "AL4001", "PERSISTENT_SHORT_AT_ORIGIN");
    RejectGapDryRunSameThread(AlarmManager::SHORT_CIRCUIT, m_gapDryRun.sourceLine, "PERSISTENT_SHORT_AT_ORIGIN");
    return true;
}

bool NCManager::RearmEDMPersistentShortSameThread(std::uint64_t nowMs, bool restart) noexcept
{
    if (m_edmZFixture.gapPersistentProfile && !IsEDMGapPersistentConfigValidSameThread(true))
    { EndEDMZFixtureSameThread("FAIL", "P22_REARM_CONFIG_INVALID"); return false; }
    if (!IsEDMZFixtureStoppedSameThread()) return false;
    const auto& stopped = m_edmZFixture.feedback;
    if (!stopped.originValid || stopped.outerOriginPulse != m_edmZFixture.originPulse || !stopped.frameCurrent ||
        stopped.latchedFault || stopped.forceZeroOutput || stopped.lastAppliedKind != EDM28::RequestKind::Stop ||
        stopped.lastAppliedSequence != stopped.stopAppliedSequence || stopped.lastAppliedTick != stopped.stopAppliedTick ||
        stopped.lastAppliedMonotonicUs != stopped.stopAppliedMonotonicUs || !stopped.stopAppliedMonotonicUs ||
        stopped.sampledTick < stopped.stopAppliedTick || stopped.sampledTick - stopped.stopAppliedTick < m_edmZFixture.config.settleCycles ||
        stopped.monotonicUs < stopped.stopAppliedMonotonicUs ||
        stopped.monotonicUs - stopped.stopAppliedMonotonicUs < m_edmZFixture.config.settleCycles * EDM28::CycleUs)
    { EndEDMZFixtureSameThread("FAIL", "P17_STOP_STAMP_INVALID"); return false; }
    if (m_edmZFixture.persistentShortLatched)
    {
        if (!m_edmZFixture.shortSourceTest || m_edmZFixture.cycles != 0U || m_edmZFixture.simulatedShortStops != 1U ||
            m_edmZFixture.verifiedShortClears != 0U || m_edmZFixture.verifiedRetreats != (m_edmZFixture.endpointProofKind == 2U ? 1U : 0U))
        { EndEDMZFixtureSameThread("FAIL", "P17_LATCH_COUNT_MISMATCH"); return false; }
        if (m_edmZFixture.gapPersistentProfile && !IsEDMGapPersistentActiveSameThread()) return false;
        m_edmZFixture.shortDone = true;
        if (!m_edmZFixture.endpointProofKind)
        {
        m_edmZFixture.shortStopProof = stopped.proofSequence; m_edmZFixture.shortStopProofTick = stopped.sampledTick;
        m_edmZFixture.shortStopAppliedSequence = stopped.stopAppliedSequence;
        m_edmZFixture.shortStopAppliedTick = stopped.stopAppliedTick;
        m_edmZFixture.shortStopAppliedMonotonicUs = stopped.stopAppliedMonotonicUs;
        }
        if (m_edmZFixture.endpointProofKind == 2U &&
            (stopped.stopAppliedTick <= m_edmZFixture.endpointAppliedTick ||
                stopped.stopAppliedMonotonicUs <= m_edmZFixture.endpointAppliedMonotonicUs ||
                stopped.proofSequence <= m_edmZFixture.endpointProofSequence))
        { EndEDMZFixtureSameThread("FAIL", "P17_TERMINAL_STOP_ASSOC_INVALID"); return false; }
        const double tolerance = .001 * m_edmZFixture.config.pulsePerMm;
        if (std::fabs(stopped.actualPulse - m_edmZFixture.originPulse) <= tolerance &&
            std::fabs(stopped.commandPulse - m_edmZFixture.originPulse) <= tolerance &&
            std::fabs(stopped.planningPulse - m_edmZFixture.originPulse) <= tolerance)
        {
            if (restart)
            {
                if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                // A stopped Disarm deliberately retains the original RT scope;
                // it needs no new Arm or motion authority to release its cap.
                m_gapDryRun.paused = false; m_edmZFixture.beginPending = false; m_edmZFixture.windowStartMs = nowMs;
            }
            if (!m_edmZFixture.endpointProofKind) (void)PrepareEDMPersistentShortAlarmSameThread(false);
            else
            {
                m_edmZFixture.receiptProof = stopped.proofSequence;
                m_edmZFixture.terminalStopAppliedSequence = stopped.stopAppliedSequence;
                m_edmZFixture.terminalStopAppliedTick = stopped.stopAppliedTick;
                m_edmZFixture.terminalStopAppliedMonotonicUs = stopped.stopAppliedMonotonicUs;
                m_edmZFixture.phaseStartMs = nowMs;
                if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
                { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
                m_edmZFixture.phase = EDMZFixturePhase::Disarm;
                LogEDMZFixtureSameThread("TERMINAL_STOP", "WAIT", "SIM20_ORIGIN_HELD_DISARM_NEXT");
            }
            return false;
        }
        if (m_edmZFixture.endpointProofKind)
        { EndEDMZFixtureSameThread("FAIL", "P17_TERMINAL_ORIGIN_LOST"); return false; }
        const double stoppedMax = (std::fmax)(stopped.actualPulse, (std::fmax)(stopped.commandPulse, stopped.planningPulse));
        const double margin = 2.0 * tolerance + 1.0;
        if (!std::isfinite(stoppedMax) || !std::isfinite(m_edmZFixture.originPulse) || !std::isfinite(margin) ||
            !(m_edmZFixture.originPulse - stoppedMax > margin))
        { EndEDMZFixtureSameThread("FAIL", "RETREAT_DIRECTION_NOT_PROVEN"); return false; }
        m_edmZFixture.shortStopMaxPulse = stoppedMax;
        if (!restart && ServiceEDMPersistentShortOperatorWindowSameThread(nowMs, 2U)) return false;
        m_edmZFixture.armNextPhase = EDMZFixturePhase::Retreat;
    }
    else
    {
        if (!restart || m_edmZFixture.shortSourceTest || m_edmZFixture.simulatedShortStops ||
            m_edmZFixture.verifiedShortClears || m_edmZFixture.verifiedRetreats || m_edmZFixture.cycles)
        { EndEDMZFixtureSameThread("FAIL", "P17_REANCHOR_COUNT_MISMATCH"); return false; }
        // This is recovery positioning before short injection, not a retreat.
        m_edmZFixture.armNextPhase = EDMZFixturePhase::Origin; m_edmZFixture.shortDone = false;
    }
    if (restart)
    {
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        m_edmZFixture.scope.executionEpoch = m_gapDryRun.epoch;
        m_edmZFixture.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        m_edmZFixture.scope.ownerGeneration = m_gapDryRun.lease.generation;
        m_edmZFixture.windowStartMs = 0ULL; m_edmZFixture.beginPending = true;
        m_gapDryRun.paused = false;
    }
    m_edmZFixture.phaseStartMs = nowMs;
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
    { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
    m_edmZFixture.phase = EDMZFixturePhase::ArmPending;
    return true;
}

// A stopped operator window is not short clearance or motion permission.
// Its formal RT Stop association remains exact throughout each fresh poll.
void NCManager::LogEDMPersistentShortOperatorWindowSameThread(const char* edge, const char* reason) noexcept
{
    const auto& state = m_edmZFixture;
    if (state.gapPersistentProfile)
    {
        const auto& gap = state.gapShort.Snapshot(); char text[480]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM38] event=WINDOW edge=%.5s reason=%.23s q=%u test=%llu session=%llu sourceTest=%llu proof=%llu stopTick=%llu sampleTick=%llu sampleUs=%llu simV=%.0f active=%u latched=%u windowMs=10000 stoppedOperatorOnly=1 physicalPermit=0 discharge=0\n",
            edge ? edge : "?", reason ? reason : "UNKNOWN", static_cast<unsigned>(state.operatorWindowQ),
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
            static_cast<unsigned long long>(state.shortSourceTest), static_cast<unsigned long long>(state.operatorWindowProof),
            static_cast<unsigned long long>(state.operatorWindowStopTick), static_cast<unsigned long long>(gap.sampleTick),
            static_cast<unsigned long long>(gap.sampleMonotonicUs), gap.voltage, gap.shortActive ? 1U : 0U, state.persistentShortLatched ? 1U : 0U);
        if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM38] event=WINDOW edge=ERROR reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        return;
    }
    char text[480]{};
    const int length = std::snprintf(text, sizeof(text),
        "[EDM33] event=WINDOW edge=%.5s reason=%.23s q=%u test=%llu session=%llu source=SIM20 sourceTest=%llu proof=%llu stopSeq=%llu stopTick=%llu stopUs=%llu originPulse=%.9g cycles=%u simShortStops=%u verifiedRetreats=%u verifiedShortClears=%u windowMs=10000 stoppedOperatorOnly=1 physicalPermit=0 discharge=0\n",
        edge ? edge : "?", reason ? reason : "UNKNOWN", static_cast<unsigned>(state.operatorWindowQ),
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(state.scope.session),
        static_cast<unsigned long long>(state.shortSourceTest), static_cast<unsigned long long>(state.operatorWindowProof),
        static_cast<unsigned long long>(state.operatorWindowStopSequence), static_cast<unsigned long long>(state.operatorWindowStopTick),
        static_cast<unsigned long long>(state.operatorWindowStopUs), state.originPulse,
        static_cast<unsigned>(state.cycles), static_cast<unsigned>(state.simulatedShortStops),
        static_cast<unsigned>(state.verifiedRetreats), static_cast<unsigned>(state.verifiedShortClears));
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("%s", "[EDM33] event=WINDOW edge=ERROR reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
}

bool NCManager::ServiceEDMPersistentShortOperatorWindowSameThread(std::uint64_t nowMs, std::uint8_t q) noexcept
{
    if (m_edmZFixture.gapPersistentProfile && !IsEDMGapPersistentActiveSameThread())
    { EndEDMZFixtureSameThread("FAIL", "P22_WINDOW_WITHOUT_ACTIVE"); return true; }
    auto& state = m_edmZFixture; const auto& feedback = state.feedback;
    if (state.operatorWindowQ != q || state.operatorWindowConsumed) return false;
    const bool phase = (q == 2U && state.phase == EDMZFixturePhase::ShortStop && !state.endpointProofKind && !state.verifiedRetreats) ||
        (q == 3U && state.phase == EDMZFixturePhase::FinalStop && state.endpointProofKind == 2U && state.verifiedRetreats == 1U);
    const double tolerance = state.config.positionToleranceMm * state.config.pulsePerMm;
    const bool origin = q != 3U || (std::isfinite(tolerance) && tolerance > 0.0 &&
        std::fabs(feedback.actualPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.commandPulse - state.originPulse) <= tolerance &&
        std::fabs(feedback.planningPulse - state.originPulse) <= tolerance && state.endpointProofSequence &&
        state.endpointAppliedSequence && state.endpointAppliedTick && state.endpointAppliedMonotonicUs &&
        feedback.proofSequence > state.endpointProofSequence && feedback.stopAppliedTick > state.endpointAppliedTick &&
        feedback.stopAppliedMonotonicUs > state.endpointAppliedMonotonicUs);
    const bool stop = phase && origin && (state.config.profile == EDM28::Profile::P17PersistentShort || state.config.profile == EDM28::Profile::P22GapPersistentShort) && !m_gapDryRun.paused &&
        state.originPinned && state.persistentShortLatched && state.shortDone && state.shortSourceTest &&
        state.cycles == 0U && state.simulatedShortStops == 1U && state.verifiedShortClears == 0U &&
        IsEDMZFixtureStoppedSameThread() && feedback.capHeld && feedback.originValid && feedback.outerOriginPulse == state.originPulse &&
        feedback.frameCurrent && !feedback.latchedFault && !feedback.forceZeroOutput && !feedback.targetProven &&
        std::fabs(feedback.actualPulse - feedback.commandPulse) <= state.config.positionToleranceMm * state.config.pulsePerMm &&
        feedback.modeValue == 9 && (feedback.statusWord & 0x006FU) == 0x0027U &&
        feedback.lastAppliedKind == EDM28::RequestKind::Stop && feedback.lastAppliedSequence == feedback.stopAppliedSequence &&
        feedback.lastAppliedTick == feedback.stopAppliedTick && feedback.lastAppliedMonotonicUs == feedback.stopAppliedMonotonicUs &&
        feedback.stopAppliedMonotonicUs && feedback.sampledTick >= feedback.stopAppliedTick &&
        feedback.sampledTick - feedback.stopAppliedTick >= state.config.settleCycles &&
        feedback.monotonicUs >= feedback.stopAppliedMonotonicUs &&
        feedback.monotonicUs - feedback.stopAppliedMonotonicUs >= state.config.settleCycles * EDM28::CycleUs;
    if (!stop || (state.operatorWindowActive && (feedback.proofSequence != state.operatorWindowProof ||
        feedback.stopAppliedSequence != state.operatorWindowStopSequence || feedback.stopAppliedTick != state.operatorWindowStopTick ||
        feedback.stopAppliedMonotonicUs != state.operatorWindowStopUs || nowMs < state.operatorWindowStartMs)))
    { EndEDMZFixtureSameThread("FAIL", "P17_OPERATOR_WINDOW_PROOF_INVALID"); return true; }
    if (!state.operatorWindowActive)
    {
        state.operatorWindowActive = true; state.operatorWindowStartMs = nowMs;
        state.operatorWindowProof = feedback.proofSequence; state.operatorWindowStopSequence = feedback.stopAppliedSequence;
        state.operatorWindowStopTick = feedback.stopAppliedTick; state.operatorWindowStopUs = feedback.stopAppliedMonotonicUs;
        LogEDMPersistentShortOperatorWindowSameThread("ENTER", "STOPPED_OPERATOR_WINDOW");
    }
    if (nowMs - state.operatorWindowStartMs < 10000ULL) return true;
    state.operatorWindowActive = false; state.operatorWindowConsumed = true;
    LogEDMPersistentShortOperatorWindowSameThread("EXIT", "TIME_ELAPSED"); return false;
}

bool NCManager::ProcessEDMPersistentShortSameThread()
{
    if (m_edmZFixture.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        // P22 still checks current/frozen identity during a long operator HOLD;
        // detector dwell is not accumulated while its observation chain is idle.
        if (m_edmZFixture.gapPersistentProfile && !ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P22_HELD_SOURCE_OR_CONFIG_INVALID"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < m_edmZFixture.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - m_edmZFixture.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == m_edmZFixture.lastClockMs)
    {
        if (++m_edmZFixture.stalledCalls >= 4096U)
        { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; }
    }
    else m_edmZFixture.stalledCalls = 0U;
    m_edmZFixture.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& feedback = m_edmZFixture.feedback;
    if (feedback.axisMapGeneration != m_edmZFixture.scope.axisMapGeneration ||
        feedback.configGeneration != m_edmZFixture.scope.configGeneration ||
        feedback.pulsePerMm != m_edmZFixture.config.pulsePerMm ||
        feedback.referenceOffsetPulse != m_edmZFixture.config.referenceOffsetPulse ||
        feedback.hardwareSign != m_edmZFixture.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (feedback.publicationSequence <= m_edmZFixture.publicationFloor || feedback.sampledTick <= m_edmZFixture.tickFloor)
    {
        if (now - m_edmZFixture.lastFreshMs >= (!m_edmZFixture.gapPersistentProfile && m_edmZFixture.phase == EDMZFixturePhase::Receipt ? 250ULL : 50ULL)) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    m_edmZFixture.publicationFloor = feedback.publicationSequence; m_edmZFixture.tickFloor = feedback.sampledTick;
    m_edmZFixture.lastFreshMs = now;
    if (feedback.latchedFault && feedback.capHeld)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_edmZFixture.originPinned && ((!feedback.originValid &&
        !(m_edmZFixture.endpointProofKind && feedback.state == EDM28::State::Disarmed && !feedback.capHeld)) ||
        feedback.outerOriginPulse != m_edmZFixture.originPulse ||
        std::fabs(feedback.actualPulse - m_edmZFixture.originPulse) > m_edmZFixture.config.outerHalfMm * m_edmZFixture.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!m_edmZFixture.resumeStartMs) m_edmZFixture.resumeStartMs = now;
        if (now - m_edmZFixture.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        if (m_edmZFixture.endpointProofKind && feedback.state == EDM28::State::Disarmed && !feedback.capHeld && feedback.stopProven)
        {
            if (!IsEDMPersistentShortReleasedSameThread() ||
                feedback.currentEpoch != m_motion.GetCurrentExecutionEpoch() || feedback.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) ||
                feedback.currentOwnerGeneration != m_programMotionLease.generation)
            { EndEDMZFixtureSameThread("FAIL", "P17_HELD_DISARM_RECEIPT_INVALID"); return false; }
            if (m_edmZFixture.gapPersistentProfile)
            {
                if (!ObserveEDMGapPersistentShortSameThread(true)) return false;
                if (!IsEDMGapPersistentActiveSameThread()) return false;
            }
            if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() || m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
            { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
            m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            m_gapDryRun.paused = false; m_edmZFixture.pending = true; m_edmZFixture.beginPending = false;
            m_edmZFixture.windowStartMs = now; m_edmZFixture.phaseStartMs = now;
            m_edmZFixture.receiptPublication = feedback.publicationSequence; m_edmZFixture.receiptTick = feedback.sampledTick;
            m_edmZFixture.phase = EDMZFixturePhase::Receipt;
            LogEDMZFixtureSameThread("TERMINAL_FREE", "WAIT", "HOLD_FRESH_DISARM_BEFORE_ALARM"); return false;
        }
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (feedback.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            feedback.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) ||
            feedback.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - m_edmZFixture.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_edmZFixture.gapPersistentProfile)
        {
            if (!ObserveEDMGapPersistentShortSameThread(true)) return false;
            if (m_edmZFixture.persistentShortLatched && !IsEDMGapPersistentActiveSameThread()) return false;
        }
        (void)RearmEDMPersistentShortSameThread(now, true); return false;
    }
    const bool terminalDisarm = m_edmZFixture.phase == EDMZFixturePhase::Disarm || m_edmZFixture.phase == EDMZFixturePhase::Receipt;
    if (feedback.currentOwner != (terminalDisarm ? static_cast<std::uint8_t>(m_gapDryRun.lease.owner) : m_edmZFixture.scope.owner) ||
        feedback.currentOwnerGeneration != (terminalDisarm ? m_gapDryRun.lease.generation : m_edmZFixture.scope.ownerGeneration) ||
        feedback.currentEpoch != (terminalDisarm ? m_gapDryRun.epoch : m_edmZFixture.scope.executionEpoch))
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (m_edmZFixture.windowStartMs && now - m_edmZFixture.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (m_edmZFixture.phase != EDMZFixturePhase::PreArm && m_edmZFixture.phase != EDMZFixturePhase::ArmPending &&
        !EDM28::SameScope(feedback.scope, m_edmZFixture.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (feedback.forceZeroOutput || feedback.state == EDM28::State::Fault ||
        (feedback.stopLatched && m_edmZFixture.phase != EDMZFixturePhase::ShortStop && m_edmZFixture.phase != EDMZFixturePhase::ArmPending &&
         m_edmZFixture.phase != EDMZFixturePhase::FinalStop && m_edmZFixture.phase != EDMZFixturePhase::Disarm && m_edmZFixture.phase != EDMZFixturePhase::Receipt))
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    if (m_edmZFixture.gapPersistentProfile && !ObserveEDMGapPersistentShortSameThread(false)) return false;
    if (m_edmZFixture.operatorWindowActive &&
        ServiceEDMPersistentShortOperatorWindowSameThread(now, m_edmZFixture.operatorWindowQ)) return false;
    if (m_edmZFixture.phase == EDMZFixturePhase::PreArm)
    {
        if (feedback.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            m_edmZFixture.phase = EDMZFixturePhase::ArmPending; m_edmZFixture.phaseStartMs = now;
        }
        else if (now - m_edmZFixture.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - m_edmZFixture.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!feedback.capHeld || feedback.state != EDM28::State::Armed || !feedback.frameCurrent || !feedback.originValid)
        { EndEDMZFixtureSameThread("FAIL", "ARM_NOT_ACTIVE"); return false; }
        if (m_edmZFixture.persistentShortLatched)
        {
            if (m_edmZFixture.armNextPhase != EDMZFixturePhase::Retreat || !m_edmZFixture.shortDone || !m_edmZFixture.shortSourceTest ||
                !m_edmZFixture.shortStopProof || feedback.proofSequence != m_edmZFixture.shortStopProof ||
                feedback.stopAppliedSequence != m_edmZFixture.shortStopAppliedSequence ||
                feedback.stopAppliedTick != m_edmZFixture.shortStopAppliedTick ||
                feedback.stopAppliedMonotonicUs != m_edmZFixture.shortStopAppliedMonotonicUs ||
                feedback.lastAppliedTick <= m_edmZFixture.shortStopProofTick ||
                m_edmZFixture.cycles || m_edmZFixture.simulatedShortStops != 1U || m_edmZFixture.verifiedRetreats || m_edmZFixture.verifiedShortClears)
            { EndEDMZFixtureSameThread("FAIL", "P17_RETREAT_ARM_PROOF_INVALID"); return false; }
        }
        else if (!m_edmZFixture.beginPending ||
            (m_edmZFixture.armNextPhase != EDMZFixturePhase::Negative && m_edmZFixture.armNextPhase != EDMZFixturePhase::Origin) ||
            m_edmZFixture.shortSourceTest || m_edmZFixture.shortDone || m_edmZFixture.cycles ||
            m_edmZFixture.simulatedShortStops || m_edmZFixture.verifiedRetreats || m_edmZFixture.verifiedShortClears)
        { EndEDMZFixtureSameThread("FAIL", "P17_INITIAL_PHASE_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!m_edmZFixture.originPinned)
        { m_edmZFixture.originPulse = feedback.outerOriginPulse; m_edmZFixture.originPinned = true; }
        m_edmZFixture.pending = false; m_edmZFixture.phase = m_edmZFixture.armNextPhase;
        if (m_edmZFixture.beginPending)
        { m_edmZFixture.windowStartMs = now; m_edmZFixture.beginPending = false; LogEDMZFixtureSameThread("BEGIN", "RUNNING", "LOCAL_Z_PERSISTENT_SHORT_ALARM"); }
        if (m_edmZFixture.persistentShortLatched) LogEDMZFixtureSameThread("RETREAT_ARM", "RUNNING", "STOP_PROVEN_SIM20_RETAINED");
        return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::ShortStop)
    {
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        (void)RearmEDMPersistentShortSameThread(now, false); return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::FinalStop)
    {
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        const double tolerance = .001 * m_edmZFixture.config.pulsePerMm;
        if (!m_edmZFixture.endpointProofKind || !m_edmZFixture.persistentShortLatched ||
            feedback.lastAppliedKind != EDM28::RequestKind::Stop || feedback.lastAppliedSequence != feedback.stopAppliedSequence ||
            feedback.lastAppliedTick != feedback.stopAppliedTick || feedback.lastAppliedMonotonicUs != feedback.stopAppliedMonotonicUs ||
            !feedback.stopAppliedMonotonicUs || feedback.sampledTick < feedback.stopAppliedTick ||
            feedback.sampledTick - feedback.stopAppliedTick < m_edmZFixture.config.settleCycles ||
            feedback.monotonicUs < feedback.stopAppliedMonotonicUs ||
            feedback.monotonicUs - feedback.stopAppliedMonotonicUs < m_edmZFixture.config.settleCycles * EDM28::CycleUs ||
            feedback.stopAppliedTick <= m_edmZFixture.endpointAppliedTick || feedback.stopAppliedMonotonicUs <= m_edmZFixture.endpointAppliedMonotonicUs ||
            feedback.proofSequence <= m_edmZFixture.endpointProofSequence ||
            std::fabs(feedback.actualPulse - m_edmZFixture.originPulse) > tolerance ||
            std::fabs(feedback.commandPulse - m_edmZFixture.originPulse) > tolerance ||
            std::fabs(feedback.planningPulse - m_edmZFixture.originPulse) > tolerance)
        { EndEDMZFixtureSameThread("FAIL", "P17_FINAL_STOP_NOT_PROVEN"); return false; }
        m_edmZFixture.receiptProof = feedback.proofSequence;
        m_edmZFixture.terminalStopAppliedSequence = feedback.stopAppliedSequence;
        m_edmZFixture.terminalStopAppliedTick = feedback.stopAppliedTick;
        m_edmZFixture.terminalStopAppliedMonotonicUs = feedback.stopAppliedMonotonicUs;
        if (ServiceEDMPersistentShortOperatorWindowSameThread(now, 3U)) return false;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        m_edmZFixture.phase = EDMZFixturePhase::Disarm; m_edmZFixture.phaseStartMs = now;
        LogEDMZFixtureSameThread("TERMINAL_STOP", "WAIT", "SIM20_ORIGIN_HELD_DISARM_NEXT"); return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - m_edmZFixture.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        if (feedback.capHeld || feedback.state != EDM28::State::Disarmed || !feedback.stopProven ||
            feedback.proofSequence != m_edmZFixture.receiptProof || feedback.stopAppliedSequence != m_edmZFixture.terminalStopAppliedSequence ||
            feedback.stopAppliedTick != m_edmZFixture.terminalStopAppliedTick ||
            feedback.stopAppliedMonotonicUs != m_edmZFixture.terminalStopAppliedMonotonicUs)
        { EndEDMZFixtureSameThread("FAIL", "DISARM_NOT_PROVEN"); return false; }
        m_edmZFixture.receiptPublication = feedback.publicationSequence; m_edmZFixture.receiptTick = feedback.sampledTick;
        m_edmZFixture.phase = EDMZFixturePhase::Receipt; m_edmZFixture.phaseStartMs = now;
        LogEDMZFixtureSameThread("TERMINAL_FREE", "WAIT", "FRESH_RECEIPT_BEFORE_ALARM"); return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::Receipt)
    {
        if (now - m_edmZFixture.phaseStartMs > 250ULL)
        { EndEDMZFixtureSameThread("FAIL", "P17_FINAL_RECEIPT_TIMEOUT"); return false; }
        if (feedback.publicationSequence <= m_edmZFixture.receiptPublication || feedback.sampledTick <= m_edmZFixture.receiptTick) return false;
        (void)AlarmEDMPersistentShortAtOriginSameThread(m_edmZFixture.endpointProofKind == 2U); return false;
    }
    if (m_edmZFixture.phase != EDMZFixturePhase::Negative && m_edmZFixture.phase != EDMZFixturePhase::Origin && m_edmZFixture.phase != EDMZFixturePhase::Retreat)
    { EndEDMZFixtureSameThread("FAIL", "P17_TERMINAL_PHASE_INVALID"); return false; }
    if (m_edmZFixture.phase == EDMZFixturePhase::Retreat &&
        (!m_edmZFixture.persistentShortLatched || !m_edmZFixture.shortDone || !m_edmZFixture.shortSourceTest ||
            !m_edmZFixture.shortStopProof || !m_edmZFixture.shortStopAppliedSequence || !m_edmZFixture.shortStopAppliedTick ||
            !m_edmZFixture.shortStopAppliedMonotonicUs || m_edmZFixture.cycles || m_edmZFixture.simulatedShortStops != 1U ||
            m_edmZFixture.verifiedRetreats || m_edmZFixture.verifiedShortClears ||
            feedback.stopAppliedSequence != m_edmZFixture.shortStopAppliedSequence ||
            feedback.stopAppliedTick != m_edmZFixture.shortStopAppliedTick ||
            feedback.stopAppliedMonotonicUs != m_edmZFixture.shortStopAppliedMonotonicUs))
    { EndEDMZFixtureSameThread("FAIL", "RETREAT_STOP_PROOF_MISMATCH"); return false; }
    if (m_edmZFixture.phase != EDMZFixturePhase::Retreat && m_edmZFixture.persistentShortLatched)
    { EndEDMZFixtureSameThread("FAIL", "P17_LATCHED_FORWARD_DENIED"); return false; }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (!m_edmZFixture.pending)
    {
        const bool ready = feedback.axisIdle && std::fabs(feedback.cmdSpeedMmS) <= 1.0 / m_edmZFixture.config.pulsePerMm &&
            (feedback.state == EDM28::State::Armed || (feedback.state == EDM28::State::AtTarget && feedback.targetProven &&
                feedback.proofSequence && feedback.stableCycles >= m_edmZFixture.config.settleCycles));
        if (!ready)
        { if (now - m_edmZFixture.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "WAIT", "AWAIT_REAL_POSITION_PROOF"); return false; }
        if (m_edmZFixture.phase == EDMZFixturePhase::Retreat)
        {
            const double currentMax = (std::fmax)(m_edmZFixture.shortStopMaxPulse,
                (std::fmax)(feedback.actualPulse, (std::fmax)(feedback.commandPulse, feedback.planningPulse)));
            const double margin = 2.0 * .001 * m_edmZFixture.config.pulsePerMm + 1.0;
            if (!std::isfinite(m_edmZFixture.shortStopMaxPulse) || !std::isfinite(currentMax) || !std::isfinite(m_edmZFixture.originPulse) ||
                feedback.proofSequence != m_edmZFixture.shortStopProof || !(m_edmZFixture.originPulse - currentMax > margin))
            { EndEDMZFixtureSameThread("FAIL", "RETREAT_DIRECTION_NOT_PROVEN"); return false; }
        }
        const double target = m_edmZFixture.phase == EDMZFixturePhase::Negative ? -.02 : 0.0;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position, target))
        { EndEDMZFixtureSameThread("FAIL", "POSITION_SUBMIT_FAILED"); return false; }
        m_edmZFixture.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        if (m_edmZFixture.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(feedback.scope, m_edmZFixture.scope) &&
            !feedback.stopLatched && !feedback.latchedFault && !feedback.forceZeroOutput &&
            feedback.lastRequestSequence >= m_edmZFixture.pendingSequence && feedback.lastAppliedSequence < m_edmZFixture.pendingSequence)
        { m_edmZFixture.pending = false; LogEDMZFixtureSameThread("PROGRESS", "WAIT", "POSITION_CONSUMED_RETRY_PROOF"); return false; }
        if (now - m_edmZFixture.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    m_edmZFixture.motionApplied = true; m_edmZFixture.appliedTick = feedback.lastAppliedTick;
    if (!m_edmZFixture.gapPersistentProfile && m_edmZFixture.phase == EDMZFixturePhase::Negative && !m_edmZFixture.shortDone && feedback.sampledTick - feedback.lastAppliedTick >= 800ULL)
    {
        if (feedback.state != EDM28::State::Moving || feedback.targetProven || feedback.axisIdle)
        { EndEDMZFixtureSameThread("FAIL", "SHORT_TEST_REQUIRES_MOVING_LEG"); return false; }
        if (m_edmZFixture.persistentShortLatched || m_edmZFixture.shortSourceTest || m_edmZFixture.cycles ||
            m_edmZFixture.simulatedShortStops || m_edmZFixture.verifiedRetreats || m_edmZFixture.verifiedShortClears)
        { EndEDMZFixtureSameThread("FAIL", "SHORT_INJECTION_COUNT_MISMATCH"); return false; }
        m_edmZFixture.persistentShortLatched = true; m_edmZFixture.shortSourceTest = m_gapDryRun.test;
        m_edmZFixture.simulatedShortStops = 1U; m_edmZFixture.shortDone = true;
        m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
        m_edmZFixture.phase = EDMZFixturePhase::ShortStop; m_edmZFixture.pending = false; m_edmZFixture.phaseStartMs = now;
        LogEDMZFixtureSameThread("SHORT_STOP", "WAIT", "SIM20_PERSISTENT_NEGATIVE_LEG"); return false;
    }
    if (feedback.state == EDM28::State::AtTarget && feedback.targetProven && feedback.proofSequence &&
        feedback.sampledTick > feedback.lastAppliedTick && feedback.stableCycles >= m_edmZFixture.config.settleCycles && feedback.axisIdle)
    {
        const double expected = m_edmZFixture.originPulse + (m_edmZFixture.phase == EDMZFixturePhase::Negative ? -.02 : 0.0) * m_edmZFixture.config.pulsePerMm;
        const double tolerance = .001 * m_edmZFixture.config.pulsePerMm;
        if (!std::isfinite(feedback.targetPulse) || !std::isfinite(expected) || !std::isfinite(tolerance) ||
            std::fabs(feedback.targetPulse - expected) > tolerance || std::fabs(feedback.actualPulse - expected) > tolerance ||
            std::fabs(feedback.commandPulse - expected) > tolerance || std::fabs(feedback.planningPulse - expected) > tolerance)
        { EndEDMZFixtureSameThread("FAIL", "POSITION_PROOF_WRONG_ENDPOINT"); return false; }
        if (m_edmZFixture.phase == EDMZFixturePhase::Retreat)
        { (void)PrepareEDMPersistentShortAlarmSameThread(true); return false; }
        if (m_edmZFixture.phase != EDMZFixturePhase::Origin || m_edmZFixture.persistentShortLatched)
        { EndEDMZFixtureSameThread("FAIL", "P17_INJECTION_NOT_COMPLETED"); return false; }
        m_edmZFixture.pending = false; m_edmZFixture.phase = EDMZFixturePhase::Negative;
        LogEDMZFixtureSameThread("REANCHOR", "RUNNING", "FIRST_ORIGIN_BEFORE_SHORT"); return false;
    }
    if (now - m_edmZFixture.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_POSITION_PROOF");
    return false;
}

namespace
{
    bool SpeedRetreatFrozenConfigMatches(const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return EDM28::IsSpeedRetreatProfile(a.profile) && a.feedMmMin == EDM28::ProfileFeedMmMin(a.profile) &&
            a.pdoCapMmS == EDM28::ProfilePdoCapMmS(a.profile) && a.axisIndex == b.axisIndex &&
            a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse && a.hardwareSign == b.hardwareSign &&
            a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm && a.feedMmMin == b.feedMmMin &&
            a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec && a.decelerationTimeSec == b.decelerationTimeSec &&
            a.followingLimitMm == b.followingLimitMm && a.positionToleranceMm == b.positionToleranceMm &&
            a.excursionToleranceMm == b.excursionToleranceMm && a.heartbeatTicks == b.heartbeatTicks &&
            a.maximumIssueAgeTicks == b.maximumIssueAgeTicks && a.stopTimeoutTicks == b.stopTimeoutTicks &&
            a.settleCycles == b.settleCycles && a.profile == b.profile;
    }
}

bool NCManager::IsEDMGapCurveConfigValidSameThread(bool requireFrozen) const noexcept
{
    if (m_edmGapRapid.shortState.active) return IsEDMGapRapidConfigValidSameThread(requireFrozen);
    if (m_edmGapFeedUpdate.shortState.active) return IsEDMGapFeedUpdateConfigValidSameThread(requireFrozen);
    if (m_edmGapShortReplan.active) return IsEDMGapShortReplanConfigValidSameThread(requireFrozen);
    if (m_edmZFixture.gapCurveReplanProfile) return IsEDMGapReplanConfigValidSameThread(requireFrozen);
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse &&
            a.hardwareSign == b.hardwareSign && a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec &&
            a.decelerationTimeSec == b.decelerationTimeSec && a.followingLimitMm == b.followingLimitMm &&
            a.positionToleranceMm == b.positionToleranceMm && a.excursionToleranceMm == b.excursionToleranceMm &&
            a.heartbeatTicks == b.heartbeatTicks && a.maximumIssueAgeTicks == b.maximumIssueAgeTicks &&
            a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapCurveProfile && c.profile == EDM28::Profile::P23GapCurveSegments && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 20.0 && c.pdoCapMmS == .35 && c.outerHalfMm == .1 && c.targetHalfMm == .02 &&
        c.accelerationTimeSec == .2 && c.decelerationTimeSec == .2 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapCurveConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapCurvePositionSameThread(bool endpoint) const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double target = s.originPulse + s.curveTargetMm * s.config.pulsePerMm;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const auto curve = EDM39::Evaluate(s.curveAppliedVoltage);
    const bool receipt = IsEDMGapCurveConfigValidSameThread(true) && EDM28::SameScope(f.scope, s.scope) &&
        s.curveSequence && s.curveSourceTick && s.curveSourceUs && curve.valid && curve.limitedSpeedMmPerMin &&
        curve.limitedSpeedMmPerMin == s.curveSignedMmMin && f.lastAppliedKind == EDM28::RequestKind::Position &&
        f.lastAppliedSequence == s.curveSequence && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        f.lastAppliedTick > s.curveSourceTick && f.lastAppliedMonotonicUs > s.curveSourceUs &&
        f.lastAppliedTick - s.curveSourceTick <= s.config.maximumIssueAgeTicks &&
        f.lastAppliedMonotonicUs - s.curveSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.sampledTick >= f.lastAppliedTick && f.monotonicUs >= f.lastAppliedMonotonicUs &&
        f.frameCurrent && f.originValid && f.capHeld && !f.stopLatched && !f.latchedFault && !f.forceZeroOutput &&
        f.outerOriginPulse == s.originPulse && std::isfinite(target) && std::isfinite(tolerance) && tolerance > 0.0 &&
        std::isfinite(f.targetPulse) && f.targetPulse == target;
    if (!receipt || !endpoint) return receipt;
    return f.state == EDM28::State::AtTarget && f.targetProven && f.axisIdle && f.proofSequence &&
        f.stableCycles >= s.config.settleCycles && f.sampledTick - f.lastAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.lastAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        std::fabs(f.actualPulse - target) <= tolerance && std::fabs(f.commandPulse - target) <= tolerance &&
        std::fabs(f.planningPulse - target) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm;
}

bool NCManager::IsEDMGapCurveReleasedSameThread() const noexcept
{
    if (m_edmGapRapid.shortState.active) return IsEDMGapRapidReleasedSameThread();
    if (m_edmGapFeedUpdate.shortState.active) return IsEDMGapFeedUpdateReleasedSameThread();
    if (m_edmGapShortReplan.active) return IsEDMGapShortReplanReleasedSameThread();
    if (m_edmZFixture.gapCurveReplanProfile) return IsEDMGapReplanReleasedSameThread();
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    return IsEDMGapCurveConfigValidSameThread(true) && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        EDM28::SameScope(f.scope, s.scope) && s.originPinned && f.outerOriginPulse == s.originPulse &&
        f.state == EDM28::State::Disarmed && !f.capHeld && !f.latchedFault && !f.forceZeroOutput &&
        f.stopProven && f.frameCurrent && f.axisIdle && f.stableCycles >= s.config.settleCycles &&
        std::isfinite(f.targetPulse) && f.targetPulse == s.originPulse &&
        std::fabs(f.actualPulse - s.originPulse) <= tolerance && std::fabs(f.commandPulse - s.originPulse) <= tolerance &&
        std::fabs(f.planningPulse - s.originPulse) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm &&
        s.terminalStopAppliedSequence && s.terminalStopAppliedTick && s.terminalStopAppliedMonotonicUs &&
        f.stopAppliedSequence == s.terminalStopAppliedSequence && f.stopAppliedTick == s.terminalStopAppliedTick &&
        f.stopAppliedMonotonicUs == s.terminalStopAppliedMonotonicUs && f.lastAppliedTick > f.stopAppliedTick &&
        f.lastAppliedMonotonicUs > f.stopAppliedMonotonicUs && f.sampledTick >= f.lastAppliedTick &&
        f.monotonicUs >= f.lastAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs && f.proofSequence == s.receiptProof &&
        f.proofSequence > s.curveBoundaryProof && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        s.curveAppliedVoltage == 40.0 && s.curveSignedMmMin == -20.0 && s.curveTargetMm == 0.0 &&
        s.cycles == 6U && s.curvePositions == 24U && s.curveZeros == 12U;
}

bool NCManager::ProcessEDMGapCurveFixtureSameThread()
{
    auto& s = m_edmZFixture;
    if (s.phase == EDMZFixturePhase::Finished) return false;
    if (s.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        if (!ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P23_HELD_SOURCE_INVALID"); return false; }
        if (s.feedback.capHeld && s.feedback.latchedFault)
        { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (f.publicationSequence <= s.publicationFloor || f.sampledTick <= s.tickFloor)
    {
        if (now - s.lastFreshMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick; s.lastFreshMs = now;
    if (f.latchedFault || f.forceZeroOutput || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        const bool currentOwner = f.currentEpoch == m_motion.GetCurrentExecutionEpoch() &&
            f.currentOwner == static_cast<std::uint8_t>(m_programMotionLease.owner) && f.currentOwnerGeneration == m_programMotionLease.generation;
        if (s.curveTerminalHold && f.state == EDM28::State::Disarmed && !f.capHeld)
        {
            if (!IsEDMGapCurveReleasedSameThread())
            { EndEDMZFixtureSameThread("FAIL", "P23_TERMINAL_HOLD_PROOF_INVALID"); return false; }
            if (!currentOwner) return false;
            if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick ||
                f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs) return false;
            // The session is already retired: this fresh exact released proof
            // permits completion only, never a new Arm or another motion.
            s.gapShort.Reset(); s.curveVoltage = 60.0;
            const auto& terminalGap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, 60.0);
            if (!terminalGap.valid || !terminalGap.fresh || terminalGap.feedInhibited || terminalGap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P23_TERMINAL_SOURCE_INVALID"); return false; }
            if (s.phase == EDMZFixturePhase::Disarm) LogEDMZFixtureSameThread("SUMMARY", "PASS", "TERMINAL_HOLD_DISARM_ALREADY_PROVEN");
            m_gapDryRun.paused = false; m_gapDryRun.active = false; m_gapDryRun.result = 1U; s.curveTerminalHold = false;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            s.phase = EDMZFixturePhase::Finished;
            LogEDMZFixtureSameThread("RECEIPT", "READY", "TERMINAL_HOLD_FRESH_RELEASE_NO_MOTION"); return true;
        }
        if (s.curvePreArmHold)
        {
            if (f.capHeld && f.originValid && EDM28::SameScope(f.scope, s.scope))
            {
                if (!IsEDMGapCurveConfigValidSameThread(true) || !std::isfinite(f.outerOriginPulse))
                { EndEDMZFixtureSameThread("FAIL", "P23_FIRST_ARM_PROOF_INVALID"); return false; }
                s.originPulse = f.outerOriginPulse; s.originPinned = true; s.curvePreArmHold = false;
                s.phase = EDMZFixturePhase::ShortStop; s.pending = false;
            }
            else
            {
                const bool neverApplied = !f.capHeld && !f.originValid && !s.motionApplied && f.armReady && f.axisIdle &&
                    (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                        f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence));
                if (!neverApplied || !currentOwner) return false;
                if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                // A priority-cancelled, never-applied Arm cannot reuse its
                // retired session. No FIRST_ARM origin exists to change.
                s.scope.session = ++m_edmZFixtureSessionSerial;
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
                s.scope.ownerGeneration = m_gapDryRun.lease.generation;
                s.gapShort.Reset(); s.curveVoltage = 60.0; s.curvePreArmHold = false; s.curvePreArmSequence = 0ULL;
                m_gapDryRun.paused = false; s.pending = false; s.phase = EDMZFixturePhase::PreArm; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("RESTART", "WAIT", "UNAPPLIED_ARM_NEW_SESSION_NO_ORIGIN"); return false;
            }
        }
        const bool stopped = IsEDMZFixtureStoppedSameThread() && f.capHeld && f.frameCurrent && f.originValid &&
            f.outerOriginPulse == s.originPulse && f.lastAppliedKind == EDM28::RequestKind::Stop &&
            f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
            f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.stopAppliedMonotonicUs &&
            f.sampledTick >= f.stopAppliedTick && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
            f.monotonicUs >= f.stopAppliedMonotonicUs && f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs;
        if (!stopped) return false;
        if (f.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            f.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) || f.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - s.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        // Reset source/script only after pure motion Stop proof. HOLD duration
        // cannot become detector dwell, a zero-speed proof, or a cycle credit.
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        s.scope.ownerGeneration = m_gapDryRun.lease.generation;
        s.gapShort.Reset(); s.curveVoltage = 60.0; s.curveSignedMmMin = 0.0; s.curveAppliedVoltage = 0.0;
        s.curveSequence = s.curveSourceTick = s.curveSourceUs = s.curveBoundaryProof = 0ULL;
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL; s.curveStep = 0U; s.curveTerminalHold = false;
        s.cycles = s.curvePositions = s.curveZeros = 0U; s.windowStartMs = 0ULL;
        s.terminalStopAppliedSequence = s.terminalStopAppliedTick = s.terminalStopAppliedMonotonicUs = 0ULL;
        m_gapDryRun.paused = false; s.pending = false; s.phaseStartMs = now;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ArmPending; return false;
    }
    if (f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration || f.currentEpoch != s.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending && !EDM28::SameScope(f.scope, s.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (f.stopLatched && s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending &&
        s.phase != EDMZFixturePhase::FinalStop && s.phase != EDMZFixturePhase::Disarm && s.phase != EDMZFixturePhase::Receipt)
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    const auto& gap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, s.curveVoltage);
    const auto curve = EDM39::Evaluate(s.curveVoltage);
    // Unexpected source/short inhibition publishes priority Stop in End before
    // diagnostics or heartbeat, and retires through the existing AL2020 path.
    if (!gap.valid || !gap.fresh || gap.fault != EDM37::GapShortFault::None || gap.feedInhibited || gap.shortActive ||
        gap.state != EDMGapServo::ShortState::Clear || !EDM39::IsScriptVoltage(s.curveVoltage) || !curve.valid)
    { EndEDMZFixtureSameThread("FAIL", "P23_GAP_SOURCE_OR_SHORT"); return false; }
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const bool atOrigin = std::fabs(f.actualPulse - s.originPulse) <= tolerance &&
        std::fabs(f.commandPulse - s.originPulse) <= tolerance && std::fabs(f.planningPulse - s.originPulse) <= tolerance;
    if (s.phase == EDMZFixturePhase::Receipt || s.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        const bool safe = IsEDMGapCurveReleasedSameThread();
        if (!safe) { EndEDMZFixtureSameThread("FAIL", "P23_FINAL_RECEIPT_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.phaseStartMs = now;
            s.phase = EDMZFixturePhase::Receipt; m_gapDryRun.active = false; m_gapDryRun.result = 1U;
            LogEDMZFixtureSameThread("SUMMARY", "PASS", "CURVE_24_MOVES_12_ZERO_ORIGIN"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL) { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick) return false;
        s.phase = EDMZFixturePhase::Finished;
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (s.windowStartMs && now - s.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        if (f.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        }
        else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapCurveConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            !f.originValid || !f.frameCurrent || !f.axisIdle || !std::isfinite(f.outerOriginPulse))
        { EndEDMZFixtureSameThread("FAIL", "P23_ARM_NOT_ACTIVE"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.pending = false; s.phase = EDMZFixturePhase::Origin; s.windowStartMs = now; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("BEGIN", "RUNNING", "FIXED_CURVE_FINITE_SEGMENTS"); return false;
    }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (!atOrigin || !f.frameCurrent || !f.originValid || f.lastAppliedKind != EDM28::RequestKind::Stop ||
            f.lastAppliedSequence != f.stopAppliedSequence || f.lastAppliedTick != f.stopAppliedTick ||
            f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs || !f.stopAppliedMonotonicUs ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs ||
            f.proofSequence <= s.curveBoundaryProof || f.stopAppliedTick <= s.curveSourceTick)
        { EndEDMZFixtureSameThread("FAIL", "P23_FINAL_STOP_PROOF_INVALID"); return false; }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "EXACT_STOP_DISARM_SUBMITTED"); return false;
    }
    if (s.phase != EDMZFixturePhase::Origin && s.phase != EDMZFixturePhase::Positive)
    { EndEDMZFixtureSameThread("FAIL", "P23_PHASE_INVALID"); return false; }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && (s.curveStep == 2U || s.curveStep == 5U))
    {
        if (s.pending || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
            !IsEDMGapCurvePositionSameThread(true) || !s.curveBoundaryProof || f.proofSequence != s.curveBoundaryProof)
        { EndEDMZFixtureSameThread("FAIL", "P23_ZERO_ENDPOINT_REVOKED"); return false; }
        if (!s.curveZeroStartUs)
        {
            s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
            LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "ZERO_CURVE_NO_POSITION"); return false;
        }
        if (f.monotonicUs - s.curveZeroStartUs < 500000ULL || f.sampledTick - s.curveZeroStartTick < 2000ULL) return false;
        if (s.curveStep == 5U && s.cycles == 5U && now - s.windowStartMs < 10000ULL) return false;
        ++s.curveZeros;
        LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "REAL_500MS_NO_POSITION");
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        if (s.curveStep == 2U) { s.curveStep = 3U; s.curveVoltage = 50.0; }
        else
        {
            ++s.cycles;
            if (s.cycles == 6U)
            {
                if (s.curvePositions != 24U || s.curveZeros != 12U || !atOrigin)
                { EndEDMZFixtureSameThread("FAIL", "P23_FINAL_COUNT_MISMATCH"); return false; }
                m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop;
            }
            else { s.curveStep = 0U; s.curveVoltage = 70.0; }
        }
        s.phaseStartMs = now; return false;
    }
    if (!s.pending)
    {
        const bool ready = f.axisIdle && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm &&
            (f.state == EDM28::State::Armed || (f.state == EDM28::State::AtTarget && f.targetProven &&
                f.proofSequence && f.stableCycles >= s.config.settleCycles));
        if (!ready) return false;
        double target = 0.0;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (atOrigin) { s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0; return false; }
            const double low = (std::fmin)(f.actualPulse, (std::fmin)(f.commandPulse, f.planningPulse));
            const double high = (std::fmax)(f.actualPulse, (std::fmax)(f.commandPulse, f.planningPulse));
            const double recoveryVoltage = high < s.originPulse - tolerance ? 50.0 : low > s.originPulse + tolerance ? 70.0 : 0.0;
            if (!recoveryVoltage) { EndEDMZFixtureSameThread("FAIL", "P23_RECOVERY_DIRECTION_AMBIGUOUS"); return false; }
            if (s.curveVoltage != recoveryVoltage) { s.curveVoltage = recoveryVoltage; return false; }
        }
        else
        {
            if (s.curveStep != 0U && s.curveStep != 1U && s.curveStep != 3U && s.curveStep != 4U)
            { EndEDMZFixtureSameThread("FAIL", "P23_STEP_INVALID"); return false; }
            const double voltage = s.curveStep == 0U ? 70.0 : s.curveStep == 1U ? 80.0 : s.curveStep == 3U ? 50.0 : 40.0;
            if (s.curveVoltage != voltage) { EndEDMZFixtureSameThread("FAIL", "P23_SCRIPT_VOLTAGE_MISMATCH"); return false; }
            target = s.curveStep == 1U ? -.02 : s.curveStep == 4U ? 0.0 : -.01;
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position, target))
        { EndEDMZFixtureSameThread("FAIL", "P23_CURVE_POSITION_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "CURVE_FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        if (s.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(f.scope, s.scope) &&
            !f.stopLatched && f.lastRequestSequence >= s.pendingSequence && f.lastAppliedSequence < s.pendingSequence)
        { s.pending = false; return false; }
        if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    if (!IsEDMGapCurvePositionSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "P23_CURVE_RECEIPT_MISMATCH"); return false; }
    s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
    if (f.targetProven)
    {
        if (!IsEDMGapCurvePositionSameThread(true))
        { EndEDMZFixtureSameThread("FAIL", "P23_POSITION_PROOF_INVALID"); return false; }
        s.pending = false; s.curveBoundaryProof = f.proofSequence;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (!atOrigin) { EndEDMZFixtureSameThread("FAIL", "P23_RECOVERY_NOT_ORIGIN"); return false; }
            LogEDMZFixtureSameThread("RECOVERED", "RUNNING", "FRESH_CURVE_ORIGIN_NO_CYCLE_CREDIT");
            s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0;
        }
        else
        {
            ++s.curvePositions;
            LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "CURVE_RECEIPT_200_RT_POSITION");
            ++s.curveStep; s.curveVoltage = s.curveStep == 1U ? 80.0 : s.curveStep == 4U ? 40.0 : 60.0;
        }
        s.phaseStartMs = now; return false;
    }
    if (now - s.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_CURVE_POSITION_PROOF");
    return false;
}

bool NCManager::IsEDMGapReplanConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse &&
            a.hardwareSign == b.hardwareSign && a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec &&
            a.decelerationTimeSec == b.decelerationTimeSec && a.followingLimitMm == b.followingLimitMm &&
            a.positionToleranceMm == b.positionToleranceMm && a.excursionToleranceMm == b.excursionToleranceMm &&
            a.heartbeatTicks == b.heartbeatTicks && a.maximumIssueAgeTicks == b.maximumIssueAgeTicks &&
            a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapCurveProfile && state.gapCurveReplanProfile && c.profile == EDM28::Profile::P24GapCurveReplan && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 20.0 && c.pdoCapMmS == .35 && c.outerHalfMm == .1 && c.targetHalfMm == EDM40::TargetHalfMm &&
        c.accelerationTimeSec == .2 && c.decelerationTimeSec == .2 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapCurveConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapReplanRetiredFaultSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    // Disarm retains the previous fault for diagnostics. It is not a fault of
    // this new session until a first Arm has actually acquired its own cap.
    // Never clear RT state here: RT Arm still validates current authority,
    // source, configuration and 200 new idle cycles before clearing its latch.
    const bool firstArmPending = s.phase == EDMZFixturePhase::ArmPending && s.pending &&
        s.pendingKind == EDM28::RequestKind::Arm && s.pendingSequence > f.lastAppliedSequence;
    if (!IsEDMGapReplanConfigValidSameThread(false) || s.originPinned || s.motionApplied || s.windowStartMs ||
        s.curvePositions || s.cycles || !((s.phase == EDMZFixturePhase::PreArm && !s.pending) || firstArmPending) ||
        !f.latchedFault || f.reason == EDM28::Reason::None || f.state != EDM28::State::Disarmed || f.capHeld || f.stopLatched || f.forceZeroOutput || f.originValid ||
        !EDM28::ValidScope(f.scope) || f.scope.session > s.scope.session ||
        !f.publicationSequence || !f.sampledTick || !f.monotonicUs || !f.sourceFresh || !f.pdoValid || !f.contiguous || !f.clockValid ||
        !f.axisExists || !f.linear || !f.servoReady || !f.modeReady || f.modeValue != 9 || (f.statusWord & 0x006FU) != 0x0027U ||
        f.fault || f.hardPositive || f.hardNegative || !f.axisIdle || !f.frameCurrent ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.scope.axisMapGeneration != s.scope.axisMapGeneration || f.scope.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign ||
        !std::isfinite(f.actualPulse) || !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
        !std::isfinite(f.cmdSpeedMmS) || std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.commandPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        f.lastAppliedKind != EDM28::RequestKind::Disarm || !f.stopAppliedSequence || !f.stopAppliedTick || !f.stopAppliedMonotonicUs ||
        f.lastAppliedSequence <= f.stopAppliedSequence || f.lastAppliedTick <= f.stopAppliedTick ||
        f.lastAppliedMonotonicUs <= f.stopAppliedMonotonicUs || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.stopAppliedTick < s.config.settleCycles ||
        f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs || !f.proofSequence)
        return false;
    if (!f.queuedCancellation)
        return f.scope.session < s.scope.session && f.lastRequestSequence == f.lastAppliedSequence &&
            f.stopProven && f.stableCycles >= s.config.settleCycles;
    // HOLD can cancel a queued first Arm before it applies. RT then changes
    // the cancellation scope but deliberately retains the prior fault and
    // Disarm stamps. Require that exact receipt, previously proved here, plus
    // a fresh idle window; a current-session applied fault cannot use this path.
    const bool exactRetired = s.replanRetiredFaultSession && s.replanRetiredFaultSession < f.scope.session &&
        f.lastAppliedSequence == s.replanRetiredFaultDisarmSequence && f.lastAppliedTick == s.replanRetiredFaultDisarmTick &&
        f.lastAppliedMonotonicUs == s.replanRetiredFaultDisarmUs && f.stopAppliedSequence == s.replanRetiredFaultStopSequence &&
        f.stopAppliedTick == s.replanRetiredFaultStopTick && f.stopAppliedMonotonicUs == s.replanRetiredFaultStopUs &&
        f.reason == s.replanRetiredFaultReason && f.armReady && f.lastRequestSequence > f.lastAppliedSequence;
    const bool exactCancellation = f.scope.session < s.scope.session ||
        (m_gapDryRun.paused && s.curvePreArmHold && firstArmPending && EDM28::SameScope(f.scope, s.scope) &&
            s.curvePreArmSequence == s.pendingSequence && f.lastRequestSequence == s.curvePreArmSequence);
    return exactRetired && exactCancellation;
}

bool NCManager::IsEDMGapReplanReleasedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    return IsEDMGapReplanConfigValidSameThread(true) && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        EDM28::SameScope(f.scope, s.scope) && s.originPinned && f.outerOriginPulse == s.originPulse &&
        f.state == EDM28::State::Disarmed && !f.capHeld && !f.latchedFault && !f.forceZeroOutput &&
        f.stopProven && f.frameCurrent && f.axisIdle && f.stableCycles >= s.config.settleCycles &&
        std::isfinite(f.targetPulse) && f.targetPulse == s.originPulse &&
        std::fabs(f.actualPulse - s.originPulse) <= tolerance && std::fabs(f.commandPulse - s.originPulse) <= tolerance &&
        std::fabs(f.planningPulse - s.originPulse) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm &&
        s.terminalStopAppliedSequence && s.terminalStopAppliedTick && s.terminalStopAppliedMonotonicUs &&
        f.stopAppliedSequence == s.terminalStopAppliedSequence && f.stopAppliedTick == s.terminalStopAppliedTick &&
        f.stopAppliedMonotonicUs == s.terminalStopAppliedMonotonicUs && f.lastAppliedTick > f.stopAppliedTick &&
        f.lastAppliedMonotonicUs > f.stopAppliedMonotonicUs && f.sampledTick >= f.lastAppliedTick &&
        f.monotonicUs >= f.lastAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs && f.proofSequence == s.receiptProof &&
        f.proofSequence > s.curveBoundaryProof && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        s.curveAppliedVoltage == 50.0 && s.curveSignedMmMin == -10.0 && s.curveTargetMm == 0.0 &&
        s.cycles == EDM40::CycleCount && s.curvePositions == EDM40::LaunchCount && s.curveZeros == EDM40::ZeroCount &&
        s.replanInterrupts == EDM40::InterruptCount && s.replanReturns == EDM40::ReturnCount;
}

bool NCManager::IsEDMGapReplanTriggerSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{};
    if (!EDM40::TryLeg(s.curveStep, leg) || leg.triggerVoltage == 0.0 ||
        !IsEDMGapReplanConfigValidSameThread(true) || !IsEDMGapCurvePositionSameThread(false) ||
        s.curveVoltage != leg.voltage || s.curveAppliedVoltage != leg.voltage || s.curveTargetMm != leg.targetMm ||
        f.state != EDM28::State::Moving || f.axisIdle || f.targetProven ||
        f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.lastAppliedTick < EDM40::TriggerMinimumTicks ||
        f.monotonicUs - f.lastAppliedMonotonicUs < EDM40::TriggerMinimumUs ||
        std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm) return false;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const double progress = EDM40::TriggerProgressMm * s.config.pulsePerMm;
    const double remaining = EDM40::TriggerRemainingMm * s.config.pulsePerMm;
    const double target = s.originPulse + leg.targetMm * s.config.pulsePerMm;
    return direction * f.cmdSpeedMmS > 0.0 &&
        direction * (f.actualPulse - s.replanLaunchActual) >= progress &&
        direction * (f.commandPulse - s.replanLaunchCommand) >= progress &&
        direction * (f.planningPulse - s.replanLaunchPlanning) >= progress &&
        direction * (target - f.actualPulse) >= remaining &&
        direction * (target - f.commandPulse) >= remaining &&
        direction * (target - f.planningPulse) >= remaining;
}

bool NCManager::IsEDMGapReplanStoppedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{}; const auto changed = EDM39::Evaluate(s.replanChangeVoltage);
    return IsEDMGapReplanConfigValidSameThread(true) && EDM40::TryLeg(s.curveStep, leg) &&
        leg.triggerVoltage != 0.0 && s.replanChangeVoltage == leg.triggerVoltage && changed.valid &&
        changed.limitedSpeedMmPerMin == s.replanChangeMmMin && s.curveVoltage == s.replanChangeVoltage &&
        IsEDMZFixtureStoppedSameThread() && f.state == EDM28::State::Held && f.capHeld && f.stopLatched &&
        f.frameCurrent && f.originValid && f.outerOriginPulse == s.originPulse &&
        s.replanPositionSequence && s.replanPositionSequence == s.curveSequence &&
        s.replanPositionSourceTick == s.curveSourceTick && s.replanPositionSourceUs == s.curveSourceUs &&
        s.replanPositionVoltage == s.curveAppliedVoltage && s.replanPositionMmMin == s.curveSignedMmMin &&
        s.replanPositionTargetMm == s.curveTargetMm && s.replanPositionVoltage == leg.voltage &&
        s.replanPositionTargetMm == leg.targetMm &&
        f.targetPulse == s.originPulse + s.replanPositionTargetMm * s.config.pulsePerMm &&
        s.replanPositionTick > s.replanPositionSourceTick && s.replanPositionUs > s.replanPositionSourceUs &&
        s.replanPositionTick - s.replanPositionSourceTick <= s.config.maximumIssueAgeTicks &&
        s.replanPositionUs - s.replanPositionSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.lastCurveSequence == s.replanPositionSequence && f.lastCurveSampleTick == s.replanPositionSourceTick &&
        f.lastCurveSampleUs == s.replanPositionSourceUs && f.lastCurveVoltage == s.replanPositionVoltage &&
        f.lastSignedCurveMmMin == s.replanPositionMmMin &&
        s.replanChangeTick > s.replanPositionTick && s.replanChangeUs > s.replanPositionUs &&
        f.stopAppliedSequence > s.replanPositionSequence && f.stopAppliedTick > s.replanChangeTick &&
        f.stopAppliedMonotonicUs > s.replanChangeUs &&
        f.lastAppliedKind == EDM28::RequestKind::Stop && f.lastAppliedSequence == f.stopAppliedSequence &&
        f.lastAppliedTick == f.stopAppliedTick && f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs &&
        f.proofSequence > s.replanPriorProof && f.sampledTick >= f.stopAppliedTick &&
        f.monotonicUs >= f.stopAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        (!s.replanStopSequence || (f.stopAppliedSequence == s.replanStopSequence &&
            f.stopAppliedTick == s.replanStopTick && f.stopAppliedMonotonicUs == s.replanStopUs &&
            f.proofSequence == s.replanStopProof));
}

void NCManager::LogEDMGapReplanFixtureSameThread(const char* event, const char* result, const char* reason) noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& g = s.gapShort.Snapshot();
    auto& q = m_edmGapReplanDiagnostics;
    const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
    // No allocation, formatting, console IO, or control authority here. Even
    // BEGIN is immutable deferred evidence, not an operator timing signal.
    m_edmZFixture.logMs = s.lastClockMs;
    if (m_gapDryRun.frequency) q.clockFrequency = m_gapDryRun.frequency;
    q.endPending = true; q.lastProfile = static_cast<std::uint32_t>(s.config.profile);
    if (q.captured == maximum) q.saturated = true;
    else ++q.captured;
    // EDM44 keeps every completed CYCLE and every lifecycle/failure event.
    // Middle-cycle detail is counted explicitly, with full first/last-cycle
    // snapshots. No control, stop proof or source state depends on logging.
    // captured = enqueued + dropped + coalesced while counters are unsaturated.
    if (!q.saturated && EDM28::IsGapEnduranceProfile(s.config.profile) &&
        s.cycles > 0U && s.cycles < EDM44::CycleCount - 1U &&
        !f.latchedFault && !f.forceZeroOutput && !f.fault && !f.hardPositive && !f.hardNegative &&
        event && result && (std::strcmp(result, "WAIT") == 0 || std::strcmp(result, "RUNNING") == 0 || std::strcmp(result, "PROVEN") == 0) &&
        (std::strcmp(event, "LEG") == 0 || std::strcmp(event, "APPLIED") == 0 ||
            std::strcmp(event, "SHORT_ENTRY") == 0 || std::strcmp(event, "SHORT_CLEAR") == 0 ||
            std::strcmp(event, "CHANGE_PROVEN") == 0 || std::strcmp(event, "REPLAN") == 0 ||
            std::strcmp(event, "CHANGE_STOP") == 0 || std::strcmp(event, "BOUNDARY") == 0 ||
            std::strcmp(event, "ZERO_BEGIN") == 0 || std::strcmp(event, "ZERO_PROVEN") == 0 ||
            std::strcmp(event, "PROGRESS") == 0))
    {
        if (q.coalesced != maximum) { ++q.coalesced; return; }
        q.saturated = true;
    }
    if (q.saturated || q.count == EDMGapReplanDiagnosticState::Capacity)
    {
        if (q.dropped != maximum) ++q.dropped;
        q.lastDroppedEvent = q.captured; q.lastDroppedTest = m_gapDryRun.test;
        return;
    }
    auto& r = q.records[q.write];
    const auto copyLabel = [](char* target, std::size_t capacity, const char* source) noexcept
    {
        std::size_t i = 0U;
        if (source) for (; i + 1U < capacity && source[i]; ++i) target[i] = source[i];
        target[i] = '\0';
    };
    copyLabel(r.event, sizeof(r.event), event);
    copyLabel(r.result, sizeof(r.result), result);
    copyLabel(r.reason, sizeof(r.reason), reason);
    r.evt = q.captured; r.test = m_gapDryRun.test; r.run = m_gapDryRun.run; r.scope = s.scope;
    r.elapsedMs = s.windowStartMs && s.lastClockMs >= s.windowStartMs ? s.lastClockMs - s.windowStartMs : 0ULL;
    r.phase = static_cast<std::uint32_t>(s.phase); r.step = s.curveStep; r.cycles = s.cycles;
    r.launches = s.curvePositions; r.interrupts = s.replanInterrupts; r.returns = s.replanReturns;
    r.zeros = s.curveZeros; r.restart = m_gapDryRun.restarts;
    r.originPulse = s.originPulse; r.actualPulse = f.actualPulse; r.simVoltage = s.curveVoltage;
    r.sampleTick = g.sampleTick; r.sampleUs = g.sampleMonotonicUs;
    r.curveSequence = s.curveSequence; r.curveSourceTick = s.curveSourceTick; r.curveSourceUs = s.curveSourceUs;
    r.requestVoltage = s.curveAppliedVoltage; r.requestFeed = s.curveSignedMmMin; r.targetMm = s.curveTargetMm;
    r.zeroTick = s.curveZeroStartTick; r.zeroUs = s.curveZeroStartUs;
    r.inhibit = g.feedInhibited; r.gapFault = static_cast<std::uint32_t>(g.fault);
    r.positionSequence = s.replanPositionSequence; r.positionTick = s.replanPositionTick; r.positionUs = s.replanPositionUs;
    r.oldVoltage = s.replanPositionVoltage; r.oldFeed = s.replanPositionMmMin;
    r.newVoltage = s.replanChangeVoltage; r.newFeed = s.replanChangeMmMin;
    r.changeTick = s.replanChangeTick; r.changeUs = s.replanChangeUs; r.priorProof = s.replanPriorProof;
    r.stopSequence = s.replanStopSequence; r.stopTick = s.replanStopTick; r.stopUs = s.replanStopUs; r.stopProof = s.replanStopProof;
    r.appliedSequence = f.lastAppliedSequence; r.kind = static_cast<std::uint32_t>(f.lastAppliedKind);
    r.appliedTick = f.lastAppliedTick; r.appliedUs = f.lastAppliedMonotonicUs;
    r.appliedCurveSequence = f.lastCurveSequence; r.appliedCurveTick = f.lastCurveSampleTick; r.appliedCurveUs = f.lastCurveSampleUs;
    r.appliedCurveVoltage = f.lastCurveVoltage; r.appliedCurveFeed = f.lastSignedCurveMmMin;
    r.rtStopSequence = f.stopAppliedSequence; r.rtStopTick = f.stopAppliedTick; r.rtStopUs = f.stopAppliedMonotonicUs;
    r.proof = f.proofSequence; r.tick = f.sampledTick; r.us = f.monotonicUs; r.publication = f.publicationSequence;
    r.state = static_cast<std::uint32_t>(f.state); r.targetProven = f.targetProven; r.idle = f.axisIdle; r.cmdSpeedMmS = f.cmdSpeedMmS;
    r.profile = static_cast<std::uint32_t>(s.config.profile); r.rtReason = static_cast<std::uint32_t>(f.reason);
    r.sourceFresh = f.sourceFresh; r.pdoValid = f.pdoValid; r.contiguous = f.contiguous; r.clockValid = f.clockValid;
    r.frame = f.frameCurrent; r.cap = f.capHeld; r.zero = f.forceZeroOutput; r.fault = f.latchedFault;
    r.stopLatched = f.stopLatched; r.stopped = f.stopProven; r.stableCycles = f.stableCycles;
    r.sameScope = EDM28::SameScope(f.scope, s.scope);
    const auto& shortState = m_edmGapRapid.shortState.active ? m_edmGapRapid.shortState : m_edmGapFeedUpdate.shortState.active ? m_edmGapFeedUpdate.shortState : m_edmGapShortReplan;
    r.shortEntries = shortState.entries; r.shortClears = shortState.clears;
    r.shortState = static_cast<std::uint32_t>(g.state);
    r.shortIntent = shortState.shortIntent; r.pendingClear = shortState.pendingClear; r.terminalAlarm = shortState.terminalAlarm;
    r.lowTick = shortState.lowTick; r.lowUs = shortState.lowUs; r.highTick = shortState.highTick; r.highUs = shortState.highUs;
    r.entryTick = shortState.entryTick; r.entryUs = shortState.entryUs; r.clearTick = shortState.clearTick; r.clearUs = shortState.clearUs;
    const bool feedProfile = EDM28::IsGapFeedUpdateProfile(s.config.profile);
    const auto& u = m_edmGapFeedUpdate;
    r.feedUpdates = feedProfile ? u.updates : 0U; r.legFeedUpdates = feedProfile ? u.legUpdates : 0U;
    r.launchSequence = feedProfile ? u.launchSequence : 0ULL;
    r.launchTick = feedProfile ? u.launchTick : 0ULL; r.launchUs = feedProfile ? u.launchUs : 0ULL;
    r.launchSourceTick = feedProfile ? u.launchSourceTick : 0ULL; r.launchSourceUs = feedProfile ? u.launchSourceUs : 0ULL;
    r.launchVoltage = feedProfile ? u.launchVoltage : 0.0; r.launchFeed = feedProfile ? u.launchMmMin : 0.0;
    r.launchTargetMm = feedProfile ? u.launchTargetMm : 0.0;
    const bool rapidProfile = EDM28::IsGapRapidProfile(s.config.profile);
    r.rapid = rapidProfile ? m_edmGapRapid.speed : EDMGapRapidSpeedProof{};
    r.advanceSpeedProofs = rapidProfile ? m_edmGapRapid.advanceProofs : 0U;
    r.retreatSpeedProofs = rapidProfile ? m_edmGapRapid.retreatProofs : 0U;
    r.configTargetHalfMm = s.config.targetHalfMm; r.configOuterHalfMm = s.config.outerHalfMm;
    r.configCapMmS = s.config.pdoCapMmS; r.configFeedMmMin = s.config.feedMmMin;
    r.configAccelerationSec = s.config.accelerationTimeSec; r.configDecelerationSec = s.config.decelerationTimeSec;
    r.zeroDwellMs = rapidProfile ? static_cast<std::uint32_t>(EDM43::ZeroDwellUs / 1000ULL) : 500U;
    q.write = (q.write + 1U) % EDMGapReplanDiagnosticState::Capacity;
    ++q.count;
    if (q.enqueued != maximum) ++q.enqueued;
}

bool NCManager::DrainEDMGapReplanDiagnosticsSameThread() noexcept
{
    auto& q = m_edmGapReplanDiagnostics;
    const auto tagFor = [](std::uint32_t profile) noexcept { return profile == 31U || profile == 32U ? "EDM44" : profile == 29U || profile == 30U ? "EDM43" : profile == 27U || profile == 28U ? "EDM42" : profile == 25U || profile == 26U ? "EDM41" : "EDM40"; };
    if (!q.endPending) return false;
    const auto state = m_state.load(std::memory_order_acquire);
    if ((state != NCState::IDLE && state != NCState::READY && state != NCState::P_END && state != NCState::ALARM) ||
        IsEDMZFixtureDiagnosticQuietSameThread() || IsGapPathSimulationActiveSameThread() ||
        m_gapTail.active || m_edmSourceSession.active || m_edmPathProcess.active || m_gapSignal.active ||
        m_gapInlet.active || m_gapQueue.active || m_gapRecovery.active || m_gapPending.active ||
        Close_System_Com_flag || Homing.IsActive() || m_resetContinuationPhase != ResetContinuationPhase::IDLE ||
        m_resetSafetyOutputHoldActive || m_programRunStartPending || m_pendingProgramRunPhase != ProgramRunStartPhase::IDLE ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_motion.GetAxisCommandMailboxDepth() != 0U) return false;
    const auto lease = m_motion.GetMotionOwnerLease();
    const bool alarmSafety = state == NCState::ALARM && lease.owner == MotionOwner::SAFETY;
    if (lease.owner != MotionOwner::NONE && lease.owner != MotionOwner::IDLE_HOLD && !alarmSafety) return false;
    EDM28::Feedback fresh{};
    if (!m_motion.ReadEDMZFixtureFeedback(fresh) || !fresh.publicationSequence || !fresh.sampledTick || !fresh.monotonicUs ||
        !fresh.sourceFresh || !fresh.pdoValid || !fresh.contiguous || !fresh.clockValid || !fresh.axisIdle || fresh.capHeld ||
        !std::isfinite(fresh.pulsePerMm) || fresh.pulsePerMm <= 0.0 || !std::isfinite(fresh.cmdSpeedMmS) ||
        std::fabs(fresh.cmdSpeedMmS) > 1.0 / fresh.pulsePerMm ||
        fresh.sampledTick <= q.drainTick || fresh.monotonicUs <= q.drainUs) return false;
    if (alarmSafety)
    {
        // ALARM intentionally retains SAFETY until RESET. Permit its already
        // acknowledged, freshly quiet state to report the fault without
        // requiring the operator to erase the alarm first. No owner is changed.
        MotionExecutionEpoch safetyEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        if (!fresh.armReady || !m_motion.TryGetSafetyMotionOwnerEpoch(lease, safetyEpoch) ||
            fresh.currentOwner != static_cast<std::uint8_t>(MotionOwner::SAFETY) ||
            fresh.currentOwnerGeneration != lease.generation || fresh.currentEpoch != safetyEpoch) return false;
    }
    // A formal Disarm retains its stop proof. After RESET changes the owner
    // frame, RT's independent 200-cycle armReady proof also proves current
    // quietness. It grants only diagnostic output, never Arm or motion.
    const bool released = fresh.state == EDM28::State::Disarmed &&
        ((fresh.stopProven && fresh.lastAppliedKind == EDM28::RequestKind::Disarm) || fresh.armReady);
    const bool neverArmed = fresh.state == EDM28::State::Idle && fresh.scope.session == 0ULL && fresh.armReady;
    if (!released && !neverArmed) return false;
    LARGE_INTEGER counter{};
    const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
    if (!q.clockFrequency || !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0) return false;
    const auto now = static_cast<std::uint64_t>(counter.QuadPart);
    const auto whole = now / q.clockFrequency, remainder = now % q.clockFrequency;
    if (whole > maximum / 1000000ULL || remainder > maximum / 1000000ULL) return false;
    const auto wholeUs = whole * 1000000ULL, fractionUs = remainder * 1000000ULL / q.clockFrequency;
    if (fractionUs > maximum - wholeUs) return false;
    const auto nowUs = wholeUs + fractionUs;
    if (nowUs < fresh.monotonicUs || nowUs - fresh.monotonicUs > 50000ULL) return false;
    q.drainTick = fresh.sampledTick; q.drainUs = fresh.monotonicUs;
    if (!q.count)
    {
        if (q.coalesced || q.lastProfile == 31U || q.lastProfile == 32U)
            RtPrintf("[%s] event=DIAG_END deferred=1 captured=%llu enqueued=%llu drained=%llu dropped=%llu coalesced=%llu lastDroppedEvt=%llu lastDroppedTest=%llu formatFailures=%llu saturated=%u complete=%u detail=FIRST_LAST_CYCLE cycleSnapshots=ALL physicalPermit=0 discharge=0\n",
                tagFor(q.lastProfile), static_cast<unsigned long long>(q.captured), static_cast<unsigned long long>(q.enqueued),
                static_cast<unsigned long long>(q.drained), static_cast<unsigned long long>(q.dropped),
                static_cast<unsigned long long>(q.coalesced), static_cast<unsigned long long>(q.lastDroppedEvent),
                static_cast<unsigned long long>(q.lastDroppedTest), static_cast<unsigned long long>(q.formatFailures),
                q.saturated ? 1U : 0U, !q.saturated && !q.dropped && !q.formatFailures && q.drained == q.enqueued ? 1U : 0U);
        else
        RtPrintf("[%s] event=DIAG_END deferred=1 captured=%llu enqueued=%llu drained=%llu dropped=%llu lastDroppedEvt=%llu lastDroppedTest=%llu formatFailures=%llu saturated=%u complete=%u physicalPermit=0 discharge=0\n",
            tagFor(q.lastProfile), static_cast<unsigned long long>(q.captured), static_cast<unsigned long long>(q.enqueued),
            static_cast<unsigned long long>(q.drained), static_cast<unsigned long long>(q.dropped),
            static_cast<unsigned long long>(q.lastDroppedEvent), static_cast<unsigned long long>(q.lastDroppedTest),
            static_cast<unsigned long long>(q.formatFailures),
            q.saturated ? 1U : 0U, !q.saturated && !q.dropped && !q.formatFailures && q.drained == q.enqueued ? 1U : 0U);
        q.endPending = false;
        return true;
    }
    const auto& r = q.records[q.read];
    const char* tag = tagFor(r.profile);
    const bool feedProfile = r.profile == 27U || r.profile == 28U;
    const bool enduranceProfile = r.profile == 31U || r.profile == 32U;
    const bool rapidProfile = r.profile == 29U || r.profile == 30U || enduranceProfile;
    const bool shortProfile = r.profile == 25U || r.profile == 26U || feedProfile || rapidProfile;
    const bool begin = std::strcmp(r.event, "BEGIN") == 0;
    const bool rtStop = std::strcmp(r.event, "RT_STOP") == 0;
    const std::uint8_t lastPart = static_cast<std::uint8_t>(((begin || rtStop) ? 5U : 4U) + (shortProfile ? 1U : 0U) + (feedProfile ? 1U : 0U) + (rapidProfile ? 2U : 0U) + (begin && enduranceProfile ? 1U : 0U));
    char line[512]{};
    int n = -1;
    const auto formatShort = [&]() noexcept
    {
        return std::snprintf(line, sizeof(line),
            "[%s] deferred=1 evt=%llu event=%.16s part=SHORT test=%llu profile=%u intent=%u pendingClear=%u terminalAlarm=%u state=%u entries=%u clears=%u lowTick=%llu lowUs=%llu highTick=%llu highUs=%llu entryTick=%llu entryUs=%llu clearTick=%llu clearUs=%llu physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt), r.event, static_cast<unsigned long long>(r.test), r.profile,
            r.shortIntent?1U:0U, r.pendingClear?1U:0U, r.terminalAlarm?1U:0U, r.shortState, r.shortEntries, r.shortClears,
            static_cast<unsigned long long>(r.lowTick), static_cast<unsigned long long>(r.lowUs),
            static_cast<unsigned long long>(r.highTick), static_cast<unsigned long long>(r.highUs),
            static_cast<unsigned long long>(r.entryTick), static_cast<unsigned long long>(r.entryUs),
            static_cast<unsigned long long>(r.clearTick), static_cast<unsigned long long>(r.clearUs));
    };
    const auto formatFeed = [&]() noexcept
    {
        return std::snprintf(line, sizeof(line),
            "[%s] deferred=1 evt=%llu event=%.16s part=FEED test=%llu profile=%u updates=%u legUpdates=%u launchSeq=%llu launchTick=%llu launchUs=%llu sourceTick=%llu sourceUs=%llu launchV=%.0f launchF=%.9g targetMm=%.4f physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt), r.event, static_cast<unsigned long long>(r.test), r.profile,
            r.feedUpdates, r.legFeedUpdates, static_cast<unsigned long long>(r.launchSequence),
            static_cast<unsigned long long>(r.launchTick), static_cast<unsigned long long>(r.launchUs),
            static_cast<unsigned long long>(r.launchSourceTick), static_cast<unsigned long long>(r.launchSourceUs),
            r.launchVoltage, r.launchFeed, r.launchTargetMm);
    };
    const auto formatSpeed = [&]() noexcept
    {
        const auto& w = r.rapid;
        return std::snprintf(line, sizeof(line),
            "[%s] deferred=1 evt=%llu event=%.16s part=SPEED test=%llu profile=%u launchSeq=%llu launchTick=%llu launchUs=%llu advanceProofs=%u retreatProofs=%u samples=%u proven=%u actualMmS=%.9g commandMmS=%.9g planningMmS=%.9g cmdMmS=%.9g pdoMmS=%.9g physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt), r.event, static_cast<unsigned long long>(r.test), r.profile,
            static_cast<unsigned long long>(w.launchSequence), static_cast<unsigned long long>(w.launchTick), static_cast<unsigned long long>(w.launchUs),
            r.advanceSpeedProofs, r.retreatSpeedProofs, w.samples, w.proven ? 1U : 0U,
            w.actualMmS, w.commandMmS, w.planningMmS, w.cmdMmS, w.pdoMmS);
    };
    const auto formatSpeedWindow = [&]() noexcept
    {
        const auto& w = r.rapid;
        return std::snprintf(line, sizeof(line),
            "[%s] deferred=1 evt=%llu event=%.16s part=SPEED_WINDOW test=%llu startTick=%llu startUs=%llu endTick=%llu endUs=%llu startActual=%.17g startCommand=%.17g startPlanning=%.17g endActual=%.17g endCommand=%.17g endPlanning=%.17g physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt), r.event, static_cast<unsigned long long>(r.test),
            static_cast<unsigned long long>(w.startTick), static_cast<unsigned long long>(w.startUs),
            static_cast<unsigned long long>(w.endTick), static_cast<unsigned long long>(w.endUs),
            w.startActual, w.startCommand, w.startPlanning, w.endActual, w.endCommand, w.endPlanning);
    };
    switch (q.part)
    {
    case 0U:
        n = std::snprintf(line, sizeof(line),
        "[%s] deferred=1 evt=%llu event=%.16s result=%.10s reason=%.40s test=%llu session=%llu run=%llu phase=%u step=%u cycles=%u launches=%u interrupts=%u returns=%u zeros=%u elapsedMs=%llu restart=%u originPulse=%.9g actualPulse=%.9g physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(r.evt),r.event,r.result,r.reason,static_cast<unsigned long long>(r.test),static_cast<unsigned long long>(r.scope.session),
        static_cast<unsigned long long>(r.run),static_cast<unsigned>(r.phase),static_cast<unsigned>(r.step),
        static_cast<unsigned>(r.cycles),static_cast<unsigned>(r.launches),static_cast<unsigned>(r.interrupts),
        static_cast<unsigned>(r.returns),static_cast<unsigned>(r.zeros),
        static_cast<unsigned long long>(r.elapsedMs),
        static_cast<unsigned>(r.restart),r.originPulse,r.actualPulse);
        break;
    case 1U:
        n = std::snprintf(line,sizeof(line),
        "[%s] deferred=1 evt=%llu event=%.16s part=CURVE test=%llu simV=%.0f sampleTick=%llu sampleUs=%llu seq=%llu sourceTick=%llu sourceUs=%llu requestV=%.0f requestF=%.9g targetMm=%.4f zeroTick=%llu zeroUs=%llu inhibit=%u gapFault=%u physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(r.evt),r.event,static_cast<unsigned long long>(r.test),r.simVoltage,static_cast<unsigned long long>(r.sampleTick),
        static_cast<unsigned long long>(r.sampleUs),static_cast<unsigned long long>(r.curveSequence),
        static_cast<unsigned long long>(r.curveSourceTick),static_cast<unsigned long long>(r.curveSourceUs),r.requestVoltage,
        r.requestFeed,r.targetMm,static_cast<unsigned long long>(r.zeroTick),
        static_cast<unsigned long long>(r.zeroUs),r.inhibit?1U:0U,static_cast<unsigned>(r.gapFault));
        break;
    case 2U:
        n = std::snprintf(line,sizeof(line),
        "[%s] deferred=1 evt=%llu event=%.16s part=CHANGE test=%llu seq=%llu appliedTick=%llu appliedUs=%llu oldV=%.0f oldF=%.9g newV=%.0f newF=%.9g changeTick=%llu changeUs=%llu priorProof=%llu stopSeq=%llu stopTick=%llu stopUs=%llu stopProof=%llu physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(r.evt),r.event,static_cast<unsigned long long>(r.test),static_cast<unsigned long long>(r.positionSequence),
        static_cast<unsigned long long>(r.positionTick),static_cast<unsigned long long>(r.positionUs),
        r.oldVoltage,r.oldFeed,r.newVoltage,r.newFeed,
        static_cast<unsigned long long>(r.changeTick),static_cast<unsigned long long>(r.changeUs),
        static_cast<unsigned long long>(r.priorProof),static_cast<unsigned long long>(r.stopSequence),
        static_cast<unsigned long long>(r.stopTick),static_cast<unsigned long long>(r.stopUs),static_cast<unsigned long long>(r.stopProof));
        break;
    case 3U:
        n = std::snprintf(line,sizeof(line),
        "[%s] deferred=1 evt=%llu event=%.16s part=APPLIED test=%llu seq=%llu kind=%u tick=%llu us=%llu curveSeq=%llu curveTick=%llu curveUs=%llu curveV=%.0f curveF=%.9g physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(r.evt),r.event,static_cast<unsigned long long>(r.test),static_cast<unsigned long long>(r.appliedSequence),
        static_cast<unsigned>(r.kind),static_cast<unsigned long long>(r.appliedTick),static_cast<unsigned long long>(r.appliedUs),
        static_cast<unsigned long long>(r.appliedCurveSequence),static_cast<unsigned long long>(r.appliedCurveTick),
        static_cast<unsigned long long>(r.appliedCurveUs),r.appliedCurveVoltage,r.appliedCurveFeed);
        break;
    case 4U:
        n = std::snprintf(line,sizeof(line),
        "[%s] deferred=1 evt=%llu event=%.16s part=PROOF test=%llu stopSeq=%llu stopTick=%llu stopUs=%llu proof=%llu tick=%llu us=%llu pub=%llu state=%u targetProven=%u idle=%u cmdMmS=%.9g physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(r.evt),r.event,static_cast<unsigned long long>(r.test),static_cast<unsigned long long>(r.rtStopSequence),
        static_cast<unsigned long long>(r.rtStopTick),static_cast<unsigned long long>(r.rtStopUs),
        static_cast<unsigned long long>(r.proof),static_cast<unsigned long long>(r.tick),static_cast<unsigned long long>(r.us),
        static_cast<unsigned long long>(r.publication),static_cast<unsigned>(r.state),r.targetProven?1U:0U,r.idle?1U:0U,r.cmdSpeedMmS);
        break;
    case 5U:
        if (begin && rapidProfile)
        {
            n = std::snprintf(line, sizeof(line),
                "[%s] deferred=1 evt=%llu event=BEGIN part=SCOPE test=%llu session=%llu cache=%llu dispatch=%llu epoch=%u lease=%u originPulse=%.9g targetHalfMm=%.3f outerHalfMm=%.3f capMmS=%.3f maxF=%.0f accSec=%.3f decSec=%.3f cycles=%u zeroMs=%u speedWindowProof=1 physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(r.evt), static_cast<unsigned long long>(r.test), static_cast<unsigned long long>(r.scope.session),
                static_cast<unsigned long long>(r.scope.cacheGeneration), static_cast<unsigned long long>(r.scope.dispatchGeneration),
                r.scope.executionEpoch, r.scope.ownerGeneration, r.originPulse, r.configTargetHalfMm, r.configOuterHalfMm,
                r.configCapMmS, r.configFeedMmMin, r.configAccelerationSec, r.configDecelerationSec,
                (r.profile == 30U || r.profile == 32U) ? 0U : r.profile == 31U ? EDM44::CycleCount : 6U, r.zeroDwellMs);
        }
        else if (begin)
        {
            n = std::snprintf(line,sizeof(line),
            "[%s] deferred=1 evt=%llu event=BEGIN part=SCOPE test=%llu session=%llu cache=%llu dispatch=%llu epoch=%u lease=%u originPulse=%.9g targetHalfMm=%.3f capMmS=.35 maxF=20 cycles=%u zeroMs=500 finiteReplan=1 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt),static_cast<unsigned long long>(r.test),static_cast<unsigned long long>(r.scope.session),
            static_cast<unsigned long long>(r.scope.cacheGeneration),static_cast<unsigned long long>(r.scope.dispatchGeneration),
            r.scope.executionEpoch,r.scope.ownerGeneration,r.originPulse,EDM40::TargetHalfMm,(r.profile == 26U || r.profile == 28U) ? 0U : 6U);
        }
        else if (rtStop)
        {
            n = std::snprintf(line, sizeof(line),
                "[%s] deferred=1 evt=%llu event=RT_STOP part=STATE test=%llu session=%llu profile=%u state=%u rtReason=%u sourceFresh=%u pdoValid=%u contiguous=%u clockValid=%u frame=%u cap=%u zero=%u fault=%u stopLatched=%u stopped=%u dwell=%u sameScope=%u physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(r.evt), static_cast<unsigned long long>(r.test),
                static_cast<unsigned long long>(r.scope.session), r.profile, r.state, r.rtReason,
                r.sourceFresh ? 1U : 0U, r.pdoValid ? 1U : 0U, r.contiguous ? 1U : 0U, r.clockValid ? 1U : 0U,
                r.frame ? 1U : 0U, r.cap ? 1U : 0U, r.zero ? 1U : 0U, r.fault ? 1U : 0U,
                r.stopLatched ? 1U : 0U, r.stopped ? 1U : 0U, r.stableCycles, r.sameScope ? 1U : 0U);
        }
        else if (shortProfile) n = formatShort();
        break;
    case 6U:
        if (shortProfile && (begin || rtStop)) n = formatShort();
        else if (feedProfile) n = formatFeed();
        else if (rapidProfile) n = formatSpeed();
        break;
    case 7U:
        if (feedProfile && (begin || rtStop)) n = formatFeed();
        else if (rapidProfile) n = (begin || rtStop) ? formatSpeed() : formatSpeedWindow();
        break;
    case 8U:
        if (rapidProfile && (begin || rtStop)) n = formatSpeedWindow();
        break;
    case 9U:
        if (begin && enduranceProfile)
            n = std::snprintf(line, sizeof(line),
                "[%s] deferred=1 evt=%llu event=BEGIN part=ENDURANCE test=%llu profile=%u cycles=%u windowMs=%llu detail=FIRST_LAST_CYCLE cycleSnapshots=ALL physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(r.evt), static_cast<unsigned long long>(r.test), r.profile,
                r.profile == 32U ? 0U : EDM44::CycleCount,
                static_cast<unsigned long long>(EDM28::RapidWindowTimeoutMs(static_cast<EDM28::Profile>(r.profile))));
        break;
    default:
        return false;
    }
    // RTX console truncates calls at 511 bytes. Emit exactly one bounded part
    // per safe NC loop and retain the record until every part was emitted.
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else
    {
        if (q.formatFailures != maximum) ++q.formatFailures;
        RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT deferred=1 evt=%llu physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(r.evt));
    }
    if (q.part < lastPart) ++q.part;
    else
    {
        q.part = 0U; q.read = (q.read + 1U) % EDMGapReplanDiagnosticState::Capacity;
        --q.count;
        if (q.drained != maximum) ++q.drained;
    }
    return true;
}
bool NCManager::IsEDMGapShortReplanConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse &&
            a.hardwareSign == b.hardwareSign && a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec &&
            a.decelerationTimeSec == b.decelerationTimeSec && a.followingLimitMm == b.followingLimitMm &&
            a.positionToleranceMm == b.positionToleranceMm && a.excursionToleranceMm == b.excursionToleranceMm &&
            a.heartbeatTicks == b.heartbeatTicks && a.maximumIssueAgeTicks == b.maximumIssueAgeTicks &&
            a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapCurveProfile && !state.gapCurveReplanProfile && m_edmGapShortReplan.active &&
        (c.profile == EDM28::Profile::P25GapShortReplan || c.profile == EDM28::Profile::P26GapPersistentReplan) &&
        c.profile == m_edmGapShortReplan.profile &&
        m_edmGapShortReplan.persistent == (c.profile == EDM28::Profile::P26GapPersistentReplan) && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 20.0 && c.pdoCapMmS == .35 && c.outerHalfMm == .1 && c.targetHalfMm == EDM40::TargetHalfMm &&
        c.accelerationTimeSec == .2 && c.decelerationTimeSec == .2 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapCurveConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapShortReplanRetiredFaultSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    // Disarm retains the previous fault for diagnostics. It is not a fault of
    // this new session until a first Arm has actually acquired its own cap.
    // Never clear RT state here: RT Arm still validates current authority,
    // source, configuration and 200 new idle cycles before clearing its latch.
    const bool firstArmPending = s.phase == EDMZFixturePhase::ArmPending && s.pending &&
        s.pendingKind == EDM28::RequestKind::Arm && s.pendingSequence > f.lastAppliedSequence;
    if (!IsEDMGapShortReplanConfigValidSameThread(false) || s.originPinned || s.motionApplied || s.windowStartMs ||
        s.curvePositions || s.cycles || !((s.phase == EDMZFixturePhase::PreArm && !s.pending) || firstArmPending) ||
        !f.latchedFault || f.reason == EDM28::Reason::None || f.state != EDM28::State::Disarmed || f.capHeld || f.stopLatched || f.forceZeroOutput || f.originValid ||
        !EDM28::ValidScope(f.scope) || f.scope.session > s.scope.session ||
        !f.publicationSequence || !f.sampledTick || !f.monotonicUs || !f.sourceFresh || !f.pdoValid || !f.contiguous || !f.clockValid ||
        !f.axisExists || !f.linear || !f.servoReady || !f.modeReady || f.modeValue != 9 || (f.statusWord & 0x006FU) != 0x0027U ||
        f.fault || f.hardPositive || f.hardNegative || !f.axisIdle || !f.frameCurrent ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.scope.axisMapGeneration != s.scope.axisMapGeneration || f.scope.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign ||
        !std::isfinite(f.actualPulse) || !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
        !std::isfinite(f.cmdSpeedMmS) || std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.commandPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        f.lastAppliedKind != EDM28::RequestKind::Disarm || !f.stopAppliedSequence || !f.stopAppliedTick || !f.stopAppliedMonotonicUs ||
        f.lastAppliedSequence <= f.stopAppliedSequence || f.lastAppliedTick <= f.stopAppliedTick ||
        f.lastAppliedMonotonicUs <= f.stopAppliedMonotonicUs || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.stopAppliedTick < s.config.settleCycles ||
        f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs || !f.proofSequence)
        return false;
    if (!f.queuedCancellation)
        return f.scope.session < s.scope.session && f.lastRequestSequence == f.lastAppliedSequence &&
            f.stopProven && f.stableCycles >= s.config.settleCycles;
    // HOLD can cancel a queued first Arm before it applies. RT then changes
    // the cancellation scope but deliberately retains the prior fault and
    // Disarm stamps. Require that exact receipt, previously proved here, plus
    // a fresh idle window; a current-session applied fault cannot use this path.
    const bool exactRetired = s.replanRetiredFaultSession && s.replanRetiredFaultSession < f.scope.session &&
        f.lastAppliedSequence == s.replanRetiredFaultDisarmSequence && f.lastAppliedTick == s.replanRetiredFaultDisarmTick &&
        f.lastAppliedMonotonicUs == s.replanRetiredFaultDisarmUs && f.stopAppliedSequence == s.replanRetiredFaultStopSequence &&
        f.stopAppliedTick == s.replanRetiredFaultStopTick && f.stopAppliedMonotonicUs == s.replanRetiredFaultStopUs &&
        f.reason == s.replanRetiredFaultReason && f.armReady && f.lastRequestSequence > f.lastAppliedSequence;
    const bool exactCancellation = f.scope.session < s.scope.session ||
        (m_gapDryRun.paused && s.curvePreArmHold && firstArmPending && EDM28::SameScope(f.scope, s.scope) &&
            s.curvePreArmSequence == s.pendingSequence && f.lastRequestSequence == s.curvePreArmSequence);
    return exactRetired && exactCancellation;
}

bool NCManager::IsEDMGapShortReplanReleasedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    return IsEDMGapShortReplanConfigValidSameThread(true) && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        EDM28::SameScope(f.scope, s.scope) && s.originPinned && f.outerOriginPulse == s.originPulse &&
        f.state == EDM28::State::Disarmed && !f.capHeld && !f.latchedFault && !f.forceZeroOutput &&
        f.stopProven && f.frameCurrent && f.axisIdle && f.stableCycles >= s.config.settleCycles &&
        std::isfinite(f.targetPulse) && (f.targetPulse == s.originPulse ||
            (m_edmGapShortReplan.terminalAlarm && s.endpointProofKind == 1U)) &&
        std::fabs(f.actualPulse - s.originPulse) <= tolerance && std::fabs(f.commandPulse - s.originPulse) <= tolerance &&
        std::fabs(f.planningPulse - s.originPulse) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm &&
        s.terminalStopAppliedSequence && s.terminalStopAppliedTick && s.terminalStopAppliedMonotonicUs &&
        f.stopAppliedSequence == s.terminalStopAppliedSequence && f.stopAppliedTick == s.terminalStopAppliedTick &&
        f.stopAppliedMonotonicUs == s.terminalStopAppliedMonotonicUs && f.lastAppliedTick > f.stopAppliedTick &&
        f.lastAppliedMonotonicUs > f.stopAppliedMonotonicUs && f.sampledTick >= f.lastAppliedTick &&
        f.monotonicUs >= f.lastAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs && f.proofSequence == s.receiptProof &&
        f.proofSequence > s.curveBoundaryProof && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        (m_edmGapShortReplan.terminalAlarm ?
            (m_edmGapShortReplan.shortIntent && !m_edmGapShortReplan.pendingClear &&
                m_edmGapShortReplan.entryProven && s.curveVoltage == 20.0 &&
                ((s.endpointProofKind == 1U && s.endpointProofSequence && s.endpointAppliedSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs) ||
                 (s.endpointProofKind == 2U && s.curveAppliedVoltage == 20.0 && s.curveSignedMmMin == -20.0 &&
                    s.curveTargetMm == 0.0 && s.endpointAppliedSequence == s.curveSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs))) :
            (!m_edmGapShortReplan.persistent && !m_edmGapShortReplan.shortIntent &&
                s.curveAppliedVoltage == 50.0 && s.curveSignedMmMin == -10.0 && s.curveTargetMm == 0.0 &&
                s.cycles == EDM40::CycleCount && s.curvePositions == EDM40::LaunchCount && s.curveZeros == EDM40::ZeroCount &&
                s.replanInterrupts == EDM40::InterruptCount && s.replanReturns == EDM40::ReturnCount &&
                m_edmGapShortReplan.entries == EDM41::ShortEntryCount && m_edmGapShortReplan.clears == EDM41::ShortClearCount));
}

bool NCManager::IsEDMGapShortReplanTriggerSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{};
    if (!EDM41::TryLeg(s.curveStep, m_edmGapShortReplan.persistent, leg) || leg.triggerVoltage == 0.0 ||
        !IsEDMGapShortReplanConfigValidSameThread(true) || !IsEDMGapCurvePositionSameThread(false) ||
        s.curveVoltage != leg.voltage || s.curveAppliedVoltage != leg.voltage || s.curveTargetMm != leg.targetMm ||
        f.state != EDM28::State::Moving || f.axisIdle || f.targetProven ||
        f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.lastAppliedTick < EDM40::TriggerMinimumTicks ||
        f.monotonicUs - f.lastAppliedMonotonicUs < EDM40::TriggerMinimumUs ||
        std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm) return false;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const double progress = EDM40::TriggerProgressMm * s.config.pulsePerMm;
    const double remaining = EDM40::TriggerRemainingMm * s.config.pulsePerMm;
    const double target = s.originPulse + leg.targetMm * s.config.pulsePerMm;
    return direction * f.cmdSpeedMmS > 0.0 &&
        direction * (f.actualPulse - s.replanLaunchActual) >= progress &&
        direction * (f.commandPulse - s.replanLaunchCommand) >= progress &&
        direction * (f.planningPulse - s.replanLaunchPlanning) >= progress &&
        direction * (target - f.actualPulse) >= remaining &&
        direction * (target - f.commandPulse) >= remaining &&
        direction * (target - f.planningPulse) >= remaining;
}

bool NCManager::IsEDMGapShortReplanStoppedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{}; const auto changed = EDM39::Evaluate(s.replanChangeVoltage);
    return IsEDMGapShortReplanConfigValidSameThread(true) && EDM41::TryLeg(s.curveStep, m_edmGapShortReplan.persistent, leg) &&
        leg.triggerVoltage != 0.0 && s.replanChangeVoltage == leg.triggerVoltage && changed.valid &&
        changed.limitedSpeedMmPerMin == s.replanChangeMmMin && s.curveVoltage == s.replanChangeVoltage &&
        IsEDMZFixtureStoppedSameThread() && f.state == EDM28::State::Held && f.capHeld && f.stopLatched &&
        f.frameCurrent && f.originValid && f.outerOriginPulse == s.originPulse &&
        s.replanPositionSequence && s.replanPositionSequence == s.curveSequence &&
        s.replanPositionSourceTick == s.curveSourceTick && s.replanPositionSourceUs == s.curveSourceUs &&
        s.replanPositionVoltage == s.curveAppliedVoltage && s.replanPositionMmMin == s.curveSignedMmMin &&
        s.replanPositionTargetMm == s.curveTargetMm && s.replanPositionVoltage == leg.voltage &&
        s.replanPositionTargetMm == leg.targetMm &&
        f.targetPulse == s.originPulse + s.replanPositionTargetMm * s.config.pulsePerMm &&
        s.replanPositionTick > s.replanPositionSourceTick && s.replanPositionUs > s.replanPositionSourceUs &&
        s.replanPositionTick - s.replanPositionSourceTick <= s.config.maximumIssueAgeTicks &&
        s.replanPositionUs - s.replanPositionSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.lastCurveSequence == s.replanPositionSequence && f.lastCurveSampleTick == s.replanPositionSourceTick &&
        f.lastCurveSampleUs == s.replanPositionSourceUs && f.lastCurveVoltage == s.replanPositionVoltage &&
        f.lastSignedCurveMmMin == s.replanPositionMmMin &&
        s.replanChangeTick > s.replanPositionTick && s.replanChangeUs > s.replanPositionUs &&
        f.stopAppliedSequence > s.replanPositionSequence && f.stopAppliedTick > s.replanChangeTick &&
        f.stopAppliedMonotonicUs > s.replanChangeUs &&
        f.lastAppliedKind == EDM28::RequestKind::Stop && f.lastAppliedSequence == f.stopAppliedSequence &&
        f.lastAppliedTick == f.stopAppliedTick && f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs &&
        f.proofSequence > s.replanPriorProof && f.sampledTick >= f.stopAppliedTick &&
        f.monotonicUs >= f.stopAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        (!s.replanStopSequence || (f.stopAppliedSequence == s.replanStopSequence &&
            f.stopAppliedTick == s.replanStopTick && f.stopAppliedMonotonicUs == s.replanStopUs &&
            f.proofSequence == s.replanStopProof));
}

bool NCManager::ObserveEDMGapShortReplanSameThread() noexcept
{
    auto& q = m_edmGapShortReplan; const auto& s = m_edmZFixture; const auto& g = s.gapShort.Snapshot();
    if (!q.active || !g.valid || !g.fresh || g.fault != EDM37::GapShortFault::None ||
        g.voltage != s.curveVoltage || g.sampleTick != s.feedback.sampledTick ||
        g.sampleMonotonicUs != s.feedback.monotonicUs || !EDM41::IsScriptVoltage(g.voltage)) return false;
    // Boolean intent cannot substitute for a retained, coherent source proof.
    if (q.entries > EDM41::ShortEntryCount || q.clears > EDM41::ShortClearCount ||
        (!q.recovering && q.clears > q.entries) ||
        (!q.entryProven && (q.entryTick || q.entryUs)) || (!q.clearProven && (q.clearTick || q.clearUs))) return false;
    if (q.entryProven && (!q.entries || !q.entryTick || !q.entryUs ||
        q.entryTick > g.sampleTick || q.entryUs > g.sampleMonotonicUs ||
        (!q.terminalResumeObserved && (!q.lowTick || !q.lowUs || q.entryTick < q.lowTick || q.entryUs < q.lowUs ||
            q.entryTick - q.lowTick < EDM41::ShortEntryTicks || q.entryUs - q.lowUs < EDM41::ShortEntryUs)))) return false;
    if (q.clearProven && (!q.clears || !q.highTick || !q.highUs || !q.clearTick || !q.clearUs ||
        q.clearTick > g.sampleTick || q.clearUs > g.sampleMonotonicUs || q.clearTick < q.highTick || q.clearUs < q.highUs ||
        q.clearTick - q.highTick < EDM41::ShortClearTicks || q.clearUs - q.highUs < EDM41::ShortClearUs)) return false;
    if (g.voltage == 20.0)
    {
        if (!q.shortIntent || q.pendingClear ||
            (g.state != EDMGapServo::ShortState::Entering && g.state != EDMGapServo::ShortState::Active) || !g.feedInhibited) return false;
        if (!q.lowTick) { q.lowTick = g.sampleTick; q.lowUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs) return false;
        if (!q.entryProven && g.shortActive && g.state == EDMGapServo::ShortState::Active &&
            g.sampleTick - q.lowTick >= EDM41::ShortEntryTicks && g.sampleMonotonicUs - q.lowUs >= EDM41::ShortEntryUs)
        {
            if (q.entries == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.entryProven = true; q.entryTick = g.sampleTick; q.entryUs = g.sampleMonotonicUs; ++q.entries;
        }
        return true;
    }
    if (q.shortIntent)
    {
        if (!q.pendingClear || g.voltage != 50.0) return false;
        if (!q.highTick) { q.highTick = g.sampleTick; q.highUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.highTick || g.sampleMonotonicUs < q.highUs) return false;
        if (g.state == EDMGapServo::ShortState::Clear && !g.feedInhibited && !g.shortActive &&
            g.sampleTick - q.highTick >= EDM41::ShortClearTicks && g.sampleMonotonicUs - q.highUs >= EDM41::ShortClearUs)
        {
            if (q.clears == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.shortIntent = q.pendingClear = false; q.clearProven = true;
            q.clearTick = g.sampleTick; q.clearUs = g.sampleMonotonicUs; ++q.clears;
        }
        return g.state == EDMGapServo::ShortState::Exiting || g.state == EDMGapServo::ShortState::Clear;
    }
    return !q.pendingClear && !g.feedInhibited && !g.shortActive && g.state == EDMGapServo::ShortState::Clear;
}

bool NCManager::RaiseEDMGapShortReplanAlarmSameThread()
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& q = m_edmGapShortReplan;
    const auto& g = s.gapShort.Snapshot();
    if (!q.active || !q.terminalAlarm || !q.shortIntent || q.pendingClear || !q.entryProven ||
        !g.valid || !g.fresh || !g.shortActive || !g.feedInhibited || g.state != EDMGapServo::ShortState::Active ||
        g.voltage != 20.0 || g.sampleTick != f.sampledTick || g.sampleMonotonicUs != f.monotonicUs ||
        !q.lowTick || !q.lowUs || g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs ||
        g.sampleTick - q.lowTick < EDM41::ShortEntryTicks || g.sampleMonotonicUs - q.lowUs < EDM41::ShortEntryUs ||
        !IsEDMGapShortReplanReleasedSameThread() || f.publicationSequence <= s.receiptPublication ||
        f.sampledTick <= s.receiptTick || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs)
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_ALARM_RECEIPT_INVALID"); return false; }
    LogEDMZFixtureSameThread("EXPECTED_ALARM", "AL4001", "PERSISTENT_SHORT_AT_ORIGIN");
    m_edmZFixture.pending = false;
    RejectGapDryRunSameThread(AlarmManager::SHORT_CIRCUIT, m_gapDryRun.sourceLine, "PERSISTENT_SHORT_AT_ORIGIN");
    return false; // Expected alarm never releases the waiting G180 to M30.
}

bool NCManager::ProcessEDMGapShortReplanFixtureSameThread()
{
    auto& s = m_edmZFixture; auto& q = m_edmGapShortReplan;
    if (s.phase == EDMZFixturePhase::Finished) return false;
    if (s.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        if (!ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_HELD_SOURCE_INVALID"); return false; }
        if (s.feedback.capHeld && s.feedback.latchedFault)
        { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (f.publicationSequence <= s.publicationFloor || f.sampledTick <= s.tickFloor)
    {
        if (now - s.lastFreshMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick; s.lastFreshMs = now;
    const bool retiredFault = IsEDMGapShortReplanRetiredFaultSameThread();
    if (retiredFault && !f.queuedCancellation)
    {
        s.replanRetiredFaultSession = f.scope.session;
        s.replanRetiredFaultDisarmSequence = f.lastAppliedSequence; s.replanRetiredFaultDisarmTick = f.lastAppliedTick;
        s.replanRetiredFaultDisarmUs = f.lastAppliedMonotonicUs;
        s.replanRetiredFaultStopSequence = f.stopAppliedSequence; s.replanRetiredFaultStopTick = f.stopAppliedTick;
        s.replanRetiredFaultStopUs = f.stopAppliedMonotonicUs; s.replanRetiredFaultReason = f.reason;
    }
    if ((f.latchedFault && !retiredFault) || f.forceZeroOutput || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        const bool currentOwner = f.currentEpoch == m_motion.GetCurrentExecutionEpoch() &&
            f.currentOwner == static_cast<std::uint8_t>(m_programMotionLease.owner) && f.currentOwnerGeneration == m_programMotionLease.generation;
        if (s.curveTerminalHold && f.state == EDM28::State::Disarmed && !f.capHeld)
        {
            if (!IsEDMGapShortReplanReleasedSameThread())
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_TERMINAL_HOLD_PROOF_INVALID"); return false; }
            if (!currentOwner) return false;
            if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick ||
                f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs) return false;
            if (q.terminalAlarm)
            {
                if (!q.terminalResumeObserved)
                {
                    s.gapShort.Reset(); s.curveVoltage = 20.0;
                    q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                    q.clearTick = q.clearUs = 0ULL;
                    q.clearProven = false; q.terminalResumeObserved = true;
                }
                s.gapShort.Observe(f.sampledTick, f.monotonicUs, 20.0);
                if (!ObserveEDMGapShortReplanSameThread())
                { EndEDMZFixtureSameThread("FAIL", "P25_P26_TERMINAL_SOURCE_INVALID"); return false; }
                const auto& heldGap = s.gapShort.Snapshot();
                if (heldGap.state != EDMGapServo::ShortState::Active || !heldGap.shortActive ||
                    !q.lowTick || !q.lowUs || heldGap.sampleTick < q.lowTick || heldGap.sampleMonotonicUs < q.lowUs ||
                    heldGap.sampleTick - q.lowTick < EDM41::ShortEntryTicks ||
                    heldGap.sampleMonotonicUs - q.lowUs < EDM41::ShortEntryUs) return false;
                // This fresh source window replaces only the entry receipt,
                // never increments a script counter or credits the HOLD time.
                q.entryTick = heldGap.sampleTick; q.entryUs = heldGap.sampleMonotonicUs;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                // Released RT scope stays immutable. Only this final receipt may complete its retirement.
                m_gapDryRun.paused = false; s.curveTerminalHold = false;
                return RaiseEDMGapShortReplanAlarmSameThread();
            }
            // The session is already retired: this fresh exact released proof
            // permits completion only, never a new Arm or another motion.
            s.gapShort.Reset(); s.curveVoltage = 60.0;
            const auto& terminalGap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, 60.0);
            if (!terminalGap.valid || !terminalGap.fresh || terminalGap.feedInhibited || terminalGap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_TERMINAL_SOURCE_INVALID"); return false; }
            if (s.phase == EDMZFixturePhase::Disarm) LogEDMZFixtureSameThread("SUMMARY", "PASS", "TERMINAL_HOLD_DISARM_ALREADY_PROVEN");
            m_gapDryRun.paused = false; m_gapDryRun.active = false; m_gapDryRun.result = 1U; s.curveTerminalHold = false;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            s.phase = EDMZFixturePhase::Finished;
            LogEDMZFixtureSameThread("RECEIPT", "READY", "TERMINAL_HOLD_FRESH_RELEASE_NO_MOTION"); return true;
        }
        if (s.curvePreArmHold)
        {
            if (f.capHeld && f.originValid && EDM28::SameScope(f.scope, s.scope))
            {
                if (!IsEDMGapShortReplanConfigValidSameThread(true) || !std::isfinite(f.outerOriginPulse))
                { EndEDMZFixtureSameThread("FAIL", "P25_P26_FIRST_ARM_PROOF_INVALID"); return false; }
                s.originPulse = f.outerOriginPulse; s.originPinned = true; s.curvePreArmHold = false;
                s.phase = EDMZFixturePhase::ShortStop; s.pending = false;
            }
            else
            {
                const bool neverApplied = !f.capHeld && !f.originValid && !s.motionApplied && f.armReady && f.axisIdle &&
                    (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                        f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence));
                if (!neverApplied || !currentOwner) return false;
                if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                // A priority-cancelled, never-applied Arm cannot reuse its
                // retired session. No FIRST_ARM origin exists to change.
                s.scope.session = ++m_edmZFixtureSessionSerial;
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
                s.scope.ownerGeneration = m_gapDryRun.lease.generation;
                s.gapShort.Reset(); s.curveVoltage = 60.0; s.curvePreArmHold = false; s.curvePreArmSequence = 0ULL;
                m_gapDryRun.paused = false; s.pending = false; s.phase = EDMZFixturePhase::PreArm; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("RESTART", "WAIT", "UNAPPLIED_ARM_NEW_SESSION_NO_ORIGIN"); return false;
            }
        }
        const bool stopped = IsEDMZFixtureStoppedSameThread() && f.capHeld && f.frameCurrent && f.originValid &&
            f.outerOriginPulse == s.originPulse && f.lastAppliedKind == EDM28::RequestKind::Stop &&
            f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
            f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.stopAppliedMonotonicUs &&
            f.sampledTick >= f.stopAppliedTick && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
            f.monotonicUs >= f.stopAppliedMonotonicUs && f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs;
        if (!stopped) return false;
        if (f.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            f.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) || f.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - s.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        // Reset source/script only after pure motion Stop proof. HOLD duration
        // cannot become detector dwell, a zero-speed proof, or a cycle credit.
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        s.scope.ownerGeneration = m_gapDryRun.lease.generation;
        q.recovering = true; q.originStopPending = false; q.terminalResumeObserved = false;
        q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
        q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        q.entryProven = q.clearProven = false; q.entries = q.clears = 0U;
        q.terminalAlarm = false;
        s.gapShort.Reset(); s.curveVoltage = q.shortIntent ? (q.pendingClear ? 50.0 : 20.0) : 60.0;
        // Keep the last real Position tuple until a new Position replaces it.
        // A low, already-origin HOLD may retire without any further Position.
        s.curveBoundaryProof = 0ULL;
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL; s.curveStep = 0U; s.curveTerminalHold = false;
        s.cycles = s.curvePositions = s.curveZeros = 0U; s.windowStartMs = 0ULL;
        s.replanInterrupts = s.replanReturns = 0U; s.replanLaunchCounted = false;
        s.replanChangeTick = s.replanChangeUs = s.replanPriorProof = 0ULL;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        s.replanPositionSequence = s.replanPositionTick = s.replanPositionUs = 0ULL;
        s.replanPositionSourceTick = s.replanPositionSourceUs = 0ULL;
        s.replanChangeVoltage = s.replanChangeMmMin = s.replanPositionVoltage = s.replanPositionMmMin = s.replanPositionTargetMm = 0.0;
        s.terminalStopAppliedSequence = s.terminalStopAppliedTick = s.terminalStopAppliedMonotonicUs = 0ULL;
        m_gapDryRun.paused = false; s.pending = false; s.phaseStartMs = now;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ArmPending; return false;
    }
    if (f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration || f.currentEpoch != s.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending && !EDM28::SameScope(f.scope, s.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (f.stopLatched && s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending &&
        s.phase != EDMZFixturePhase::FinalStop && s.phase != EDMZFixturePhase::Disarm && s.phase != EDMZFixturePhase::Receipt &&
        s.phase != EDMZFixturePhase::ShortStop && s.phase != EDMZFixturePhase::ShortClearWait && s.phase != EDMZFixturePhase::ShortRearm && !q.originStopPending)
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    EDM40::ScriptLeg leg{};
    const bool hasLeg = EDM41::TryLeg(s.curveStep, q.persistent, leg);
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep < 4U && (!hasLeg || s.curveVoltage != leg.voltage))
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_SCRIPT_SOURCE_CHANGED"); return false; }
    const bool changed = s.phase == EDMZFixturePhase::Positive && s.pending && hasLeg && leg.triggerVoltage != 0.0 &&
        IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) && IsEDMGapCurvePositionSameThread(false) &&
        IsEDMGapShortReplanTriggerSameThread();
    if (changed)
    {
        s.curveVoltage = leg.triggerVoltage;
        if (s.curveVoltage == 20.0)
        {
            q.shortIntent = true; q.pendingClear = false; q.entryProven = q.clearProven = false;
            q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
            q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        }
        else if (s.curveVoltage == 50.0 && q.shortIntent)
        { q.pendingClear = true; q.highTick = q.highUs = 0ULL; q.clearProven = false; }
    }
    const auto& gap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, s.curveVoltage);
    const auto curve = EDM39::Evaluate(s.curveVoltage);
    // Unexpected source/short inhibition publishes priority Stop in End before
    // diagnostics or heartbeat, and retires through the existing AL2020 path.
    if (!ObserveEDMGapShortReplanSameThread() || !EDM41::IsScriptVoltage(s.curveVoltage) || !curve.valid)
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_GAP_SOURCE_OR_SHORT"); return false; }
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const bool atOrigin = std::fabs(f.actualPulse - s.originPulse) <= tolerance &&
        std::fabs(f.commandPulse - s.originPulse) <= tolerance && std::fabs(f.planningPulse - s.originPulse) <= tolerance;
    const auto beginStoppedOriginAlarm = [&]() noexcept
    {
        if (!atOrigin || !q.shortIntent || q.pendingClear || !q.entryProven || s.curveVoltage != 20.0 ||
            !IsEDMZFixtureStoppedSameThread() || !f.frameCurrent || !f.originValid || f.outerOriginPulse != s.originPulse ||
            f.lastAppliedKind != EDM28::RequestKind::Stop || f.lastAppliedSequence != f.stopAppliedSequence ||
            f.lastAppliedTick != f.stopAppliedTick || f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs ||
            !f.stopAppliedMonotonicUs || f.sampledTick < f.stopAppliedTick ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs) return false;
        q.terminalAlarm = true; q.originStopPending = false;
        s.endpointProofKind = 1U; s.endpointProofSequence = f.proofSequence;
        s.endpointAppliedSequence = f.stopAppliedSequence; s.endpointAppliedTick = f.stopAppliedTick;
        s.endpointAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.curveBoundaryProof = f.proofSequence;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        // PriorityStop is intentionally idempotent while already Held. A
        // sequenced Stop creates the distinct terminal proof without moving.
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Stop))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_TERMINAL_STOP_SUBMIT_FAILED"); return true; }
        s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ALREADY_STOPPED_NEW_TERMINAL_STOP"); return true;
    };
    if (s.phase == EDMZFixturePhase::Receipt || s.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        const bool safe = IsEDMGapShortReplanReleasedSameThread();
        if (!safe) { EndEDMZFixtureSameThread("FAIL", "P25_P26_FINAL_RECEIPT_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.phaseStartMs = now;
            s.phase = EDMZFixturePhase::Receipt;
            if (q.terminalAlarm)
            { LogEDMZFixtureSameThread("ALARM_RECEIPT", "WAIT", "LOW_ORIGIN_STOP_DISARM_PROVEN"); return false; }
            m_gapDryRun.active = false; m_gapDryRun.result = 1U;
            LogEDMZFixtureSameThread("SUMMARY", "PASS", "24_LAUNCH_18_STOP_6_RETURN_6_CLEAR"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL) { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick) return false;
        if (q.terminalAlarm) return RaiseEDMGapShortReplanAlarmSameThread();
        s.phase = EDMZFixturePhase::Finished;
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (s.windowStartMs && now - s.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        if (f.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        }
        else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapShortReplanConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            !f.originValid || !f.frameCurrent || !f.axisIdle || !std::isfinite(f.outerOriginPulse))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_ARM_NOT_ACTIVE"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.pending = false; s.phase = EDMZFixturePhase::Origin; s.windowStartMs = now; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("BEGIN", "RUNNING", "DYNAMIC_SIM_STOP_PROVE_REPLAN"); return false;
    }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        if (q.terminalAlarm && s.endpointProofKind == 1U && !IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Stop))
        {
            if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P25_P26_TERMINAL_STOP_APPLY_TIMEOUT");
            return false;
        }
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (!atOrigin || !f.frameCurrent || !f.originValid || f.lastAppliedKind != EDM28::RequestKind::Stop ||
            f.lastAppliedSequence != f.stopAppliedSequence || f.lastAppliedTick != f.stopAppliedTick ||
            f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs || !f.stopAppliedMonotonicUs ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs ||
            f.proofSequence <= s.curveBoundaryProof || f.stopAppliedTick <= s.curveSourceTick)
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_FINAL_STOP_PROOF_INVALID"); return false; }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "EXACT_STOP_DISARM_SUBMITTED"); return false;
    }
    if (s.phase == EDMZFixturePhase::ShortStop || s.phase == EDMZFixturePhase::ShortClearWait)
    {
        if (!IsEDMGapShortReplanStoppedSameThread())
        {
            if (s.phase == EDMZFixturePhase::ShortClearWait || f.stopProven)
                EndEDMZFixtureSameThread("FAIL", "P25_P26_CHANGED_STOP_PROOF_INVALID");
            else if (now - s.phaseStartMs > 1000ULL)
                EndEDMZFixtureSameThread("FAIL", "P25_P26_CHANGED_STOP_TIMEOUT");
            return false;
        }
        if (s.curveStep == 1U)
        {
            if (!q.shortIntent || q.pendingClear || s.curveVoltage != 20.0)
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_SHORT_INTENT_LOST"); return false; }
            if (!q.entryProven) return false;
            if (atOrigin && beginStoppedOriginAlarm()) return false;
        }
        if (s.curveStep == 2U && (!q.clearProven || q.shortIntent || q.pendingClear)) return false;
        if (s.phase == EDMZFixturePhase::ShortStop)
        {
            s.replanStopSequence = f.stopAppliedSequence; s.replanStopTick = f.stopAppliedTick;
            s.replanStopUs = f.stopAppliedMonotonicUs; s.replanStopProof = f.proofSequence;
            ++s.replanInterrupts;
            if (s.curveStep == 1U) LogEDMZFixtureSameThread("SHORT_ENTRY", "PROVEN", "FRESH_ACTIVE_2000US_8TICKS");
            if (s.curveStep == 2U) LogEDMZFixtureSameThread("SHORT_CLEAR", "PROVEN", "FRESH_CLEAR_5000US_20TICKS");
            LogEDMZFixtureSameThread("CHANGE_PROVEN", "RUNNING", "NEW_SAMPLE_STOP_200_RT_HELD");
            if (s.curveStep == 0U)
            {
                if (s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0)
                { EndEDMZFixtureSameThread("FAIL", "P25_P26_HELD_ZERO_CURVE_INVALID"); return false; }
                s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
                s.phase = EDMZFixturePhase::ShortClearWait; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "HELD_ZERO_NO_POSITION"); return false;
            }
        }
        else
        {
            if (s.curveStep != 0U || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
                !s.curveZeroStartUs || f.monotonicUs < s.curveZeroStartUs || f.sampledTick < s.curveZeroStartTick)
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_HELD_ZERO_REVOKED"); return false; }
            if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
                f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
            ++s.curveZeros;
            LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "HELD_500MS_NO_POSITION");
            s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        }
        EDM40::ScriptLeg next{};
        if (!EDM41::TryLeg(static_cast<unsigned>(s.curveStep)+1U,q.persistent,next))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_REPLAN_NEXT_STEP_INVALID"); return false; }
        s.curveVoltage = next.voltage;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_CHANGED_REARM_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ShortRearm; s.phaseStartMs = now; return false;
    }
    if (s.phase == EDMZFixturePhase::ShortRearm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P25_P26_REARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapShortReplanConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            f.stopLatched || !f.axisIdle || !f.originValid || !f.frameCurrent || f.outerOriginPulse != s.originPulse ||
            !s.replanStopSequence || f.stopAppliedSequence != s.replanStopSequence || f.stopAppliedTick != s.replanStopTick ||
            f.stopAppliedMonotonicUs != s.replanStopUs || f.proofSequence != s.replanStopProof ||
            f.lastAppliedTick <= s.replanStopTick || f.lastAppliedMonotonicUs <= s.replanStopUs ||
            f.lastCurveSequence != s.replanPositionSequence || f.lastCurveSampleTick != s.replanPositionSourceTick ||
            f.lastCurveSampleUs != s.replanPositionSourceUs || f.lastCurveVoltage != s.replanPositionVoltage ||
            f.lastSignedCurveMmMin != s.replanPositionMmMin)
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_REARM_RECEIPT_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_REARM_HEARTBEAT_FAILED"); return false; }
        ++s.curveStep; s.pending = false; s.phase = EDMZFixturePhase::Positive; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("REPLAN", "RUNNING", "SAME_ORIGIN_FRESH_ARM_NO_CREDIT"); return false;
    }
    if (s.phase != EDMZFixturePhase::Origin && s.phase != EDMZFixturePhase::Positive)
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_PHASE_INVALID"); return false; }
    // The new GAP sample is observed exactly once on this RT publication.
    // Stop goes to the priority mailbox before heartbeat or any console output.
    if (changed)
    {
        if (!s.replanLaunchCounted)
        { ++s.curvePositions; s.replanLaunchCounted = true; }
        s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
        s.replanPositionSequence = s.curveSequence; s.replanPositionTick = f.lastAppliedTick;
        s.replanPositionUs = f.lastAppliedMonotonicUs; s.replanPositionSourceTick = s.curveSourceTick;
        s.replanPositionSourceUs = s.curveSourceUs; s.replanPositionVoltage = s.curveAppliedVoltage;
        s.replanPositionMmMin = s.curveSignedMmMin; s.replanPositionTargetMm = s.curveTargetMm;
        s.replanChangeTick = gap.sampleTick; s.replanChangeUs = gap.sampleMonotonicUs;
        s.replanChangeVoltage = gap.voltage; s.replanChangeMmMin = curve.limitedSpeedMmPerMin;
        s.replanPriorProof = f.proofSequence;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        s.pending = false; s.phase = EDMZFixturePhase::ShortStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("CHANGE_STOP", "WAIT", "FRESH_MOVING_SAMPLE_PRIORITY_STOP"); return false;
    }
    if (q.originStopPending)
    {
        if (beginStoppedOriginAlarm()) return false;
        if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "P25_P26_ORIGIN_STOP_TIMEOUT");
        return false;
    }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep == 4U)
    {
        if (s.pending || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
            !IsEDMGapCurvePositionSameThread(true) || !s.curveBoundaryProof || f.proofSequence != s.curveBoundaryProof)
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_ZERO_ENDPOINT_REVOKED"); return false; }
        if (!s.curveZeroStartUs)
        {
            s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
            LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "ORIGIN_ZERO_NO_POSITION"); return false;
        }
        if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
            f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
        if (s.cycles+1U == EDM40::CycleCount && now-s.windowStartMs < 10000ULL) return false;
        ++s.curveZeros; ++s.cycles;
        LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "ORIGIN_500MS_NO_POSITION");
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        if (s.cycles == EDM40::CycleCount)
        {
            if (s.curvePositions != EDM40::LaunchCount || s.curveZeros != EDM40::ZeroCount ||
                s.replanInterrupts != EDM40::InterruptCount || s.replanReturns != EDM40::ReturnCount || !atOrigin ||
                q.persistent || q.shortIntent || q.entries != EDM41::ShortEntryCount || q.clears != EDM41::ShortClearCount)
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_FINAL_COUNT_MISMATCH"); return false; }
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop;
        }
        else { s.curveStep = 0U; s.curveVoltage = 70.0; }
        s.phaseStartMs = now; return false;
    }
    if (!s.pending)
    {
        const bool ready = f.axisIdle && std::fabs(f.cmdSpeedMmS) <= 1.0/s.config.pulsePerMm &&
            (f.state == EDM28::State::Armed || (f.state == EDM28::State::AtTarget && f.targetProven &&
                f.proofSequence && f.stableCycles >= s.config.settleCycles));
        if (!ready) return false;
        double target = 0.0;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (q.shortIntent)
            {
                if (q.pendingClear || !q.entryProven) return false;
                if (atOrigin)
                {
                    m_motion.RequestEDMZFixtureStop(s.scope.session); q.originStopPending = true; s.phaseStartMs = now;
                    LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ORIGIN_REQUEST_FRESH_STOP"); return false;
                }
            }
            if (atOrigin)
            {
                q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
                s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0; return false;
            }
            const double low = (std::fmin)(f.actualPulse,(std::fmin)(f.commandPulse,f.planningPulse));
            const double high = (std::fmax)(f.actualPulse,(std::fmax)(f.commandPulse,f.planningPulse));
            const double recoveryVoltage = q.shortIntent ? (high < s.originPulse-tolerance ? 20.0 : 0.0) :
                high < s.originPulse-tolerance ? 50.0 : low > s.originPulse+tolerance ? 70.0 : 0.0;
            if (!recoveryVoltage) { EndEDMZFixtureSameThread("FAIL", "P25_P26_RECOVERY_DIRECTION_AMBIGUOUS"); return false; }
            if (s.curveVoltage != recoveryVoltage) { s.curveVoltage = recoveryVoltage; return false; }
        }
        else
        {
            if (!hasLeg || s.curveVoltage != leg.voltage)
            { EndEDMZFixtureSameThread("FAIL", "P25_P26_SCRIPT_VOLTAGE_MISMATCH"); return false; }
            target = leg.targetMm;
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position,target))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_CURVE_POSITION_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FRESH_CURVE_FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        if (s.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(f.scope,s.scope) &&
            !f.stopLatched && f.lastRequestSequence >= s.pendingSequence && f.lastAppliedSequence < s.pendingSequence)
        { s.pending = false; return false; }
        if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    if (!IsEDMGapCurvePositionSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_CURVE_RECEIPT_MISMATCH"); return false; }
    s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
    if (s.phase == EDMZFixturePhase::Positive && !s.replanLaunchCounted)
    {
        ++s.curvePositions; s.replanLaunchCounted = true;
        LogEDMZFixtureSameThread("APPLIED", "RUNNING", "SCRIPT_POSITION_APPLIED_ONCE");
    }
    if (f.targetProven)
    {
        if (!IsEDMGapCurvePositionSameThread(true))
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_POSITION_PROOF_INVALID"); return false; }
        const bool lowOrigin = q.shortIntent && !q.pendingClear && q.entryProven &&
            s.curveVoltage == 20.0 && s.curveAppliedVoltage == 20.0 && s.curveTargetMm == 0.0;
        if (s.phase == EDMZFixturePhase::Positive && s.curveStep != 3U && !lowOrigin)
        { EndEDMZFixtureSameThread("FAIL", "P25_P26_MOVING_CHANGE_WINDOW_MISSED"); return false; }
        s.pending = false; s.curveBoundaryProof = f.proofSequence;
        if (!atOrigin) { EndEDMZFixtureSameThread("FAIL", "P25_P26_RETURN_NOT_ORIGIN"); return false; }
        if (lowOrigin)
        {
            q.terminalAlarm = true; q.recovering = false;
            s.endpointProofKind = 2U; s.endpointProofSequence = f.proofSequence;
            s.endpointAppliedSequence = f.lastAppliedSequence; s.endpointAppliedTick = f.lastAppliedTick;
            s.endpointAppliedMonotonicUs = f.lastAppliedMonotonicUs;
            ++s.replanReturns;
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
            LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_POSITION_ORIGIN_NEW_TERMINAL_STOP"); return false;
        }
        if (s.phase == EDMZFixturePhase::Origin)
        {
            q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
            LogEDMZFixtureSameThread("RECOVERED", "RUNNING", "FRESH_CURVE_ORIGIN_NO_CYCLE_CREDIT");
            s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0;
        }
        else
        {
            ++s.replanReturns;
            LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "F10_ORIGIN_POSITION_200_RT");
            s.curveStep = 4U; s.curveVoltage = 60.0;
        }
        s.phaseStartMs = now; return false;
    }
    if (now-s.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_MOVING_CHANGE_OR_ORIGIN");
    return false;
}

void NCManager::ObserveEDMGapRapidSpeedSameThread() noexcept
{
    auto& w = m_edmGapRapid.speed; const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const bool eligible = !m_edmGapRapid.shortState.recovering && IsEDMGapRapidPositionSameThread(false) &&
        f.state == EDM28::State::Moving && !f.axisIdle && !f.targetProven &&
        std::isfinite(f.actualPulse) && std::isfinite(f.commandPulse) && std::isfinite(f.planningPulse) &&
        std::isfinite(f.cmdSpeedMmS) && std::isfinite(f.pdoSpeedMmS) &&
        direction * f.cmdSpeedMmS > 0.0 && direction * f.pdoSpeedMmS > 0.0 &&
        f.sampledTick > f.lastAppliedTick && f.monotonicUs > f.lastAppliedMonotonicUs;
    if (!eligible) { w = EDMGapRapidSpeedProof{}; return; }
    const auto anchor = [&]() noexcept
    {
        w = EDMGapRapidSpeedProof{};
        w.launchSequence = s.curveSequence; w.launchTick = f.lastAppliedTick; w.launchUs = f.lastAppliedMonotonicUs;
        w.startTick = w.endTick = f.sampledTick; w.startUs = w.endUs = f.monotonicUs;
        w.startActual = w.endActual = f.actualPulse; w.startCommand = w.endCommand = f.commandPulse;
        w.startPlanning = w.endPlanning = f.planningPulse; w.samples = 1U;
    };
    if (!w.samples || w.launchSequence != s.curveSequence || w.launchTick != f.lastAppliedTick ||
        w.launchUs != f.lastAppliedMonotonicUs || f.sampledTick <= w.endTick || f.monotonicUs <= w.endUs ||
        direction * (f.actualPulse-w.endActual) <= 0.0 || direction * (f.commandPulse-w.endCommand) <= 0.0 ||
        direction * (f.planningPulse-w.endPlanning) <= 0.0 ||
        f.sampledTick-w.startTick > EDM43::SpeedWindowMaxTicks || f.monotonicUs-w.startUs > EDM43::SpeedWindowMaxUs)
    { anchor(); return; }
    // A previous proof may not be reused on a new sample. Advance the anchor
    // to its prior endpoint and build another consecutive fresh window.
    if (w.proven)
    {
        w.startTick = w.endTick; w.startUs = w.endUs;
        w.startActual = w.endActual; w.startCommand = w.endCommand; w.startPlanning = w.endPlanning;
        w.samples = 1U;
    }
    w.proven = false; ++w.samples;
    w.endTick = f.sampledTick; w.endUs = f.monotonicUs;
    w.endActual = f.actualPulse; w.endCommand = f.commandPulse; w.endPlanning = f.planningPulse;
    w.cmdMmS = f.cmdSpeedMmS; w.pdoMmS = f.pdoSpeedMmS;
    if (w.endTick-w.startTick < EDM43::SpeedWindowMinTicks || w.endUs-w.startUs < EDM43::SpeedWindowMinUs) return;
    const double seconds = static_cast<double>(w.endUs-w.startUs) / 1000000.0;
    w.actualMmS = direction * (w.endActual-w.startActual) / s.config.pulsePerMm / seconds;
    w.commandMmS = direction * (w.endCommand-w.startCommand) / s.config.pulsePerMm / seconds;
    w.planningMmS = direction * (w.endPlanning-w.startPlanning) / s.config.pulsePerMm / seconds;
    w.proven = std::isfinite(w.actualMmS) && std::isfinite(w.commandMmS) && std::isfinite(w.planningMmS) &&
        w.actualMmS >= EDM43::SpeedMinimumMmS && w.commandMmS >= EDM43::SpeedMinimumMmS && w.planningMmS >= EDM43::SpeedMinimumMmS;
    if (!w.proven) anchor();
}

bool NCManager::IsEDMGapRapidPositionSameThread(bool endpoint) const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double target = s.originPulse + s.curveTargetMm * s.config.pulsePerMm;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const auto curve = EDM43::Evaluate(s.curveAppliedVoltage);
    const bool receipt = IsEDMGapRapidConfigValidSameThread(true) && EDM28::SameScope(f.scope, s.scope) &&
        s.curveSequence && s.curveSourceTick && s.curveSourceUs && curve.valid && curve.limitedSpeedMmPerMin &&
        curve.limitedSpeedMmPerMin == s.curveSignedMmMin && f.lastAppliedKind == EDM28::RequestKind::Position &&
        f.lastAppliedSequence == s.curveSequence && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        f.lastAppliedTick > s.curveSourceTick && f.lastAppliedMonotonicUs > s.curveSourceUs &&
        f.lastAppliedTick - s.curveSourceTick <= s.config.maximumIssueAgeTicks &&
        f.lastAppliedMonotonicUs - s.curveSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.sampledTick >= f.lastAppliedTick && f.monotonicUs >= f.lastAppliedMonotonicUs &&
        f.frameCurrent && f.originValid && f.capHeld && !f.stopLatched && !f.latchedFault && !f.forceZeroOutput &&
        f.outerOriginPulse == s.originPulse && std::isfinite(target) && std::isfinite(tolerance) && tolerance > 0.0 &&
        std::isfinite(f.targetPulse) && f.targetPulse == target;
    if (!receipt || !endpoint) return receipt;
    return f.state == EDM28::State::AtTarget && f.targetProven && f.axisIdle && f.proofSequence &&
        f.stableCycles >= s.config.settleCycles && f.sampledTick - f.lastAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.lastAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        std::fabs(f.actualPulse - target) <= tolerance && std::fabs(f.commandPulse - target) <= tolerance &&
        std::fabs(f.planningPulse - target) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm;
}

bool NCManager::IsEDMGapRapidConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse &&
            a.hardwareSign == b.hardwareSign && a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec &&
            a.decelerationTimeSec == b.decelerationTimeSec && a.followingLimitMm == b.followingLimitMm &&
            a.positionToleranceMm == b.positionToleranceMm && a.excursionToleranceMm == b.excursionToleranceMm &&
            a.heartbeatTicks == b.heartbeatTicks && a.maximumIssueAgeTicks == b.maximumIssueAgeTicks &&
            a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapCurveProfile && !state.gapCurveReplanProfile && !m_edmGapShortReplan.active && !m_edmGapFeedUpdate.shortState.active && m_edmGapRapid.shortState.active &&
        EDM28::IsGapRapidProfile(c.profile) &&
        c.profile == m_edmGapRapid.shortState.profile &&
        m_edmGapRapid.shortState.persistent == EDM28::IsGapRapidPersistentProfile(c.profile) && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 30.0 && c.pdoCapMmS == .5 && c.outerHalfMm == .4 && c.targetHalfMm == EDM43::TargetHalfMm &&
        c.accelerationTimeSec == .3 && c.decelerationTimeSec == .3 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapCurveConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapRapidRetiredFaultSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    // Disarm retains the previous fault for diagnostics. It is not a fault of
    // this new session until a first Arm has actually acquired its own cap.
    // Never clear RT state here: RT Arm still validates current authority,
    // source, configuration and 200 new idle cycles before clearing its latch.
    const bool firstArmPending = s.phase == EDMZFixturePhase::ArmPending && s.pending &&
        s.pendingKind == EDM28::RequestKind::Arm && s.pendingSequence > f.lastAppliedSequence;
    if (!IsEDMGapRapidConfigValidSameThread(false) || s.originPinned || s.motionApplied || s.windowStartMs ||
        s.curvePositions || s.cycles || !((s.phase == EDMZFixturePhase::PreArm && !s.pending) || firstArmPending) ||
        !f.latchedFault || f.reason == EDM28::Reason::None || f.state != EDM28::State::Disarmed || f.capHeld || f.stopLatched || f.forceZeroOutput || f.originValid ||
        !EDM28::ValidScope(f.scope) || f.scope.session > s.scope.session ||
        !f.publicationSequence || !f.sampledTick || !f.monotonicUs || !f.sourceFresh || !f.pdoValid || !f.contiguous || !f.clockValid ||
        !f.axisExists || !f.linear || !f.servoReady || !f.modeReady || f.modeValue != 9 || (f.statusWord & 0x006FU) != 0x0027U ||
        f.fault || f.hardPositive || f.hardNegative || !f.axisIdle || !f.frameCurrent ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.scope.axisMapGeneration != s.scope.axisMapGeneration || f.scope.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign ||
        !std::isfinite(f.actualPulse) || !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
        !std::isfinite(f.cmdSpeedMmS) || std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.commandPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        f.lastAppliedKind != EDM28::RequestKind::Disarm || !f.stopAppliedSequence || !f.stopAppliedTick || !f.stopAppliedMonotonicUs ||
        f.lastAppliedSequence <= f.stopAppliedSequence || f.lastAppliedTick <= f.stopAppliedTick ||
        f.lastAppliedMonotonicUs <= f.stopAppliedMonotonicUs || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.stopAppliedTick < s.config.settleCycles ||
        f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs || !f.proofSequence)
        return false;
    if (!f.queuedCancellation)
        return f.scope.session < s.scope.session && f.lastRequestSequence == f.lastAppliedSequence &&
            f.stopProven && f.stableCycles >= s.config.settleCycles;
    // HOLD can cancel a queued first Arm before it applies. RT then changes
    // the cancellation scope but deliberately retains the prior fault and
    // Disarm stamps. Require that exact receipt, previously proved here, plus
    // a fresh idle window; a current-session applied fault cannot use this path.
    const bool exactRetired = s.replanRetiredFaultSession && s.replanRetiredFaultSession < f.scope.session &&
        f.lastAppliedSequence == s.replanRetiredFaultDisarmSequence && f.lastAppliedTick == s.replanRetiredFaultDisarmTick &&
        f.lastAppliedMonotonicUs == s.replanRetiredFaultDisarmUs && f.stopAppliedSequence == s.replanRetiredFaultStopSequence &&
        f.stopAppliedTick == s.replanRetiredFaultStopTick && f.stopAppliedMonotonicUs == s.replanRetiredFaultStopUs &&
        f.reason == s.replanRetiredFaultReason && f.armReady && f.lastRequestSequence > f.lastAppliedSequence;
    const bool exactCancellation = f.scope.session < s.scope.session ||
        (m_gapDryRun.paused && s.curvePreArmHold && firstArmPending && EDM28::SameScope(f.scope, s.scope) &&
            s.curvePreArmSequence == s.pendingSequence && f.lastRequestSequence == s.curvePreArmSequence);
    return exactRetired && exactCancellation;
}

EDM49::Result NCManager::EvaluateEDMGapRapidCompletionSameThread(EDM49::Stage stage) const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM49::Result invalid{}; invalid.selectedMask = 1U << 2U;
    if (!IsEDMGapRapidConfigValidSameThread(true)) { invalid.reason = EDM49::Reason::InvalidRule; return invalid; }
    if (!s.originPinned) { invalid.reason = EDM49::Reason::Origin; return invalid; }
    const auto scope = [](const EDM28::Scope& source) noexcept -> EDM49::Scope
    {
        EDM49::Scope out{};
        out.session = source.session; out.run = source.runGeneration; out.cache = source.cacheGeneration;
        out.dispatch = source.dispatchGeneration; out.axisMap = source.axisMapGeneration; out.config = source.configGeneration;
        out.epoch = source.executionEpoch;
        out.ownerLease = (static_cast<std::uint64_t>(source.owner) << 32U) | source.ownerGeneration;
        return out;
    };
    EDM49::Request request{}; EDM49::Evidence evidence{};
    request.backend = evidence.backend = EDM49::Backend::LocalRTMotion;
    request.stage = stage; request.scope = scope(s.scope); evidence.scope = scope(f.scope);
    request.selectedAxisMask = 1U << 2U; // Existing unloaded physical adapter: Z only.
    auto& rule = request.axes[2]; auto& axis = evidence.axes[2];
    rule.origin = s.originPulse; rule.positionTolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    // Preserve the original mm/s comparison, including its rounding. No PDO-zero condition.
    rule.maximumCommandRate = 1.0 / s.config.pulsePerMm;
    rule.settleTicks = s.config.settleCycles; rule.settleUs = s.config.settleCycles * EDM28::CycleUs;
    rule.proofFloor = s.curveBoundaryProof; rule.stopSourceTickFloor = s.curveSourceTick;
    rule.expectedStop = { s.terminalStopAppliedSequence, s.terminalStopAppliedTick, s.terminalStopAppliedMonotonicUs };
    rule.disarmSequence = s.pending && s.pendingKind == EDM28::RequestKind::Disarm ? s.pendingSequence : 0ULL;
    rule.releasedProof = s.receiptProof;
    rule.requireTargetAtOrigin = !(m_edmGapRapid.shortState.terminalAlarm && s.endpointProofKind == 1U);
    request.freshFloor = { s.receiptPublication, s.receiptTick, s.receiptUs };
    evidence.publication = f.publicationSequence; evidence.tick = f.sampledTick; evidence.timeUs = f.monotonicUs;
    axis.present = true; axis.axisIndex = 2U;
    // The released execution scope is immutable. Current observer owner/epoch is
    // checked by the NC lifecycle, and must not overwrite that historical scope.
    axis.sourceCurrent = EDM28::ValidScope(s.scope) && f.sourceFresh && f.pdoValid && f.contiguous && f.clockValid &&
        f.axisExists && f.linear && f.servoReady && f.modeReady && f.modeValue == 9 && (f.statusWord & 0x006FU) == 0x0027U &&
        !f.fault && !f.hardPositive && !f.hardNegative && f.axisMapGeneration == s.scope.axisMapGeneration &&
        f.configGeneration == s.scope.configGeneration && f.pulsePerMm == s.config.pulsePerMm &&
        f.referenceOffsetPulse == s.config.referenceOffsetPulse && f.hardwareSign == s.config.hardwareSign;
    axis.state = f.state == EDM28::State::Held ? EDM49::AxisState::Held :
        f.state == EDM28::State::Disarmed ? EDM49::AxisState::Disarmed : EDM49::AxisState::Other;
    axis.appliedKind = f.lastAppliedKind == EDM28::RequestKind::Stop ? EDM49::AppliedKind::Stop :
        f.lastAppliedKind == EDM28::RequestKind::Disarm ? EDM49::AppliedKind::Disarm : EDM49::AppliedKind::Other;
    axis.capHeld = f.capHeld; axis.stopLatched = f.stopLatched; axis.latchedFault = f.latchedFault;
    axis.forceZeroOutput = f.forceZeroOutput; axis.stopProven = f.stopProven; axis.frameCurrent = f.frameCurrent;
    axis.originValid = f.originValid; axis.idle = f.axisIdle; axis.stableTicks = f.stableCycles;
    axis.origin = f.outerOriginPulse; axis.target = f.targetPulse; axis.actual = f.actualPulse;
    axis.commanded = f.commandPulse; axis.planned = f.planningPulse; axis.commandRate = f.cmdSpeedMmS;
    axis.stop = { f.stopAppliedSequence, f.stopAppliedTick, f.stopAppliedMonotonicUs };
    axis.applied = { f.lastAppliedSequence, f.lastAppliedTick, f.lastAppliedMonotonicUs };
    axis.proof = f.proofSequence;
    return EDM49::Evaluate(request, evidence);
}

void NCManager::LogEDMGapRapidCompletionSameThread(const char* event, EDM49::Stage stage, const EDM49::Result& result) noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    char line[512]{};
    const int size = std::snprintf(line, sizeof(line),
        "[EDM49] event=%s backend=LOCAL_RT_MOTION stage=%s status=%s reason=%s session=%llu mask=%u proven=%u axis=%u"
        " pub=%llu tick=%llu us=%llu stop=%llu disarm=%llu floorPub=%llu floorTick=%llu floorUs=%llu driveAck=0 discharge=0\n",
        event, EDM49::StageName(stage), EDM49::StatusName(result.status), EDM49::ReasonName(result.reason),
        static_cast<unsigned long long>(s.scope.session), static_cast<unsigned>(result.selectedMask),
        static_cast<unsigned>(result.provenMask), static_cast<unsigned>(result.failingAxis),
        static_cast<unsigned long long>(f.publicationSequence), static_cast<unsigned long long>(f.sampledTick),
        static_cast<unsigned long long>(f.monotonicUs), static_cast<unsigned long long>(f.stopAppliedSequence),
        static_cast<unsigned long long>(f.lastAppliedKind == EDM28::RequestKind::Disarm ? f.lastAppliedSequence : 0ULL),
        static_cast<unsigned long long>(s.receiptPublication), static_cast<unsigned long long>(s.receiptTick),
        static_cast<unsigned long long>(s.receiptUs));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM49] event=LOG status=INVALID reason=LOG_FORMAT driveAck=0 discharge=0\n");
}

bool NCManager::IsEDMGapRapidReleasedSameThread() const noexcept
{
    return IsEDMGapRapidCompletionProfileSameThread() &&
        EvaluateEDMGapRapidCompletionSameThread(EDM49::Stage::Released).Ready();
}

bool NCManager::IsEDMGapRapidCompletionProfileSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    return f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        (m_edmGapRapid.shortState.terminalAlarm ?
            (m_edmGapRapid.shortState.shortIntent && !m_edmGapRapid.shortState.pendingClear &&
                m_edmGapRapid.shortState.entryProven && s.curveVoltage == 20.0 &&
                ((s.endpointProofKind == 1U && s.endpointProofSequence && s.endpointAppliedSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs) ||
                 (s.endpointProofKind == 2U && s.curveAppliedVoltage == 20.0 && s.curveSignedMmMin == -30.0 &&
                    s.curveTargetMm == 0.0 && s.endpointAppliedSequence == s.curveSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs))) :
            (!m_edmGapRapid.shortState.persistent && !m_edmGapRapid.shortState.shortIntent &&
                s.curveAppliedVoltage == 50.0 && s.curveSignedMmMin == -15.0 && s.curveTargetMm == 0.0 &&
                s.cycles == EDM28::RapidCycleCount(s.config.profile) && s.curvePositions == 3U * EDM28::RapidCycleCount(s.config.profile) && s.curveZeros == EDM28::RapidCycleCount(s.config.profile) &&
                s.replanInterrupts == 2U * EDM28::RapidCycleCount(s.config.profile) && s.replanReturns == EDM28::RapidCycleCount(s.config.profile) &&
                m_edmGapRapid.shortState.entries == EDM28::RapidCycleCount(s.config.profile) && m_edmGapRapid.shortState.clears == EDM28::RapidCycleCount(s.config.profile) &&
                m_edmGapRapid.advanceProofs == EDM28::RapidCycleCount(s.config.profile) && m_edmGapRapid.retreatProofs == EDM28::RapidCycleCount(s.config.profile)));
}

bool NCManager::IsEDMGapRapidTriggerSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& w = m_edmGapRapid.speed;
    EDM43::Leg leg{};
    if (!EDM43::TryLeg(s.curveStep, m_edmGapRapid.shortState.persistent, leg) || leg.triggerVoltage == 0.0 ||
        !IsEDMGapRapidConfigValidSameThread(true) || !IsEDMGapRapidPositionSameThread(false) ||
        s.curveVoltage != leg.voltage || s.curveAppliedVoltage != leg.voltage || s.curveTargetMm != leg.targetMm ||
        f.state != EDM28::State::Moving || f.axisIdle || f.targetProven || !w.proven ||
        w.launchSequence != s.curveSequence || w.launchTick != f.lastAppliedTick || w.launchUs != f.lastAppliedMonotonicUs ||
        w.startTick <= w.launchTick || w.startUs <= w.launchUs || w.endTick != f.sampledTick || w.endUs != f.monotonicUs ||
        !std::isfinite(w.startActual) || !std::isfinite(w.startCommand) || !std::isfinite(w.startPlanning) ||
        !std::isfinite(w.actualMmS) || !std::isfinite(w.commandMmS) || !std::isfinite(w.planningMmS) ||
        w.cmdMmS != f.cmdSpeedMmS || w.pdoMmS != f.pdoSpeedMmS ||
        w.endActual != f.actualPulse || w.endCommand != f.commandPulse || w.endPlanning != f.planningPulse ||
        w.samples < 2U || w.endTick < w.startTick || w.endUs < w.startUs ||
        w.endTick - w.startTick < EDM43::SpeedWindowMinTicks || w.endUs - w.startUs < EDM43::SpeedWindowMinUs ||
        w.endTick - w.startTick > EDM43::SpeedWindowMaxTicks || w.endUs - w.startUs > EDM43::SpeedWindowMaxUs ||
        w.actualMmS < EDM43::SpeedMinimumMmS || w.commandMmS < EDM43::SpeedMinimumMmS || w.planningMmS < EDM43::SpeedMinimumMmS)
        return false;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const double progress = (s.curveStep == 0U ? EDM43::AdvanceProgressMm : EDM43::RetreatProgressMm) * s.config.pulsePerMm;
    const double remaining = EDM43::TriggerRemainingMm * s.config.pulsePerMm;
    const double target = s.originPulse + leg.targetMm * s.config.pulsePerMm;
    return direction * f.cmdSpeedMmS > 0.0 && direction * f.pdoSpeedMmS > 0.0 &&
        direction * (f.actualPulse - s.replanLaunchActual) >= progress &&
        direction * (f.commandPulse - s.replanLaunchCommand) >= progress &&
        direction * (f.planningPulse - s.replanLaunchPlanning) >= progress &&
        direction * (target - f.actualPulse) >= remaining &&
        direction * (target - f.commandPulse) >= remaining &&
        direction * (target - f.planningPulse) >= remaining;
}

bool NCManager::IsEDMGapRapidStoppedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM43::Leg leg{}; const auto changed = EDM43::Evaluate(s.replanChangeVoltage);
    return IsEDMGapRapidConfigValidSameThread(true) && EDM43::TryLeg(s.curveStep, m_edmGapRapid.shortState.persistent, leg) &&
        leg.triggerVoltage != 0.0 && s.replanChangeVoltage == leg.triggerVoltage && changed.valid &&
        changed.limitedSpeedMmPerMin == s.replanChangeMmMin && s.curveVoltage == s.replanChangeVoltage &&
        IsEDMZFixtureStoppedSameThread() && f.state == EDM28::State::Held && f.capHeld && f.stopLatched &&
        f.frameCurrent && f.originValid && f.outerOriginPulse == s.originPulse &&
        s.replanPositionSequence && s.replanPositionSequence == s.curveSequence &&
        s.replanPositionSourceTick == s.curveSourceTick && s.replanPositionSourceUs == s.curveSourceUs &&
        s.replanPositionVoltage == s.curveAppliedVoltage && s.replanPositionMmMin == s.curveSignedMmMin &&
        s.replanPositionTargetMm == s.curveTargetMm && s.replanPositionVoltage == leg.voltage &&
        s.replanPositionTargetMm == leg.targetMm &&
        f.targetPulse == s.originPulse + s.replanPositionTargetMm * s.config.pulsePerMm &&
        s.replanPositionTick > s.replanPositionSourceTick && s.replanPositionUs > s.replanPositionSourceUs &&
        s.replanPositionTick - s.replanPositionSourceTick <= s.config.maximumIssueAgeTicks &&
        s.replanPositionUs - s.replanPositionSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.lastCurveSequence == s.replanPositionSequence && f.lastCurveSampleTick == s.replanPositionSourceTick &&
        f.lastCurveSampleUs == s.replanPositionSourceUs && f.lastCurveVoltage == s.replanPositionVoltage &&
        f.lastSignedCurveMmMin == s.replanPositionMmMin &&
        s.replanChangeTick > s.replanPositionTick && s.replanChangeUs > s.replanPositionUs &&
        f.stopAppliedSequence > s.replanPositionSequence && f.stopAppliedTick > s.replanChangeTick &&
        f.stopAppliedMonotonicUs > s.replanChangeUs &&
        f.lastAppliedKind == EDM28::RequestKind::Stop && f.lastAppliedSequence == f.stopAppliedSequence &&
        f.lastAppliedTick == f.stopAppliedTick && f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs &&
        f.proofSequence > s.replanPriorProof && f.sampledTick >= f.stopAppliedTick &&
        f.monotonicUs >= f.stopAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        (!s.replanStopSequence || (f.stopAppliedSequence == s.replanStopSequence &&
            f.stopAppliedTick == s.replanStopTick && f.stopAppliedMonotonicUs == s.replanStopUs &&
            f.proofSequence == s.replanStopProof));
}

bool NCManager::ObserveEDMGapRapidSameThread() noexcept
{
    auto& q = m_edmGapRapid.shortState; const auto& s = m_edmZFixture; const auto& g = s.gapShort.Snapshot();
    if (!q.active || !g.valid || !g.fresh || g.fault != EDM37::GapShortFault::None ||
        g.voltage != s.curveVoltage || g.sampleTick != s.feedback.sampledTick ||
        g.sampleMonotonicUs != s.feedback.monotonicUs || !EDM43::IsScriptVoltage(g.voltage)) return false;
    // Boolean intent cannot substitute for a retained, coherent source proof.
    if (q.entries > EDM28::RapidCycleCount(s.config.profile) || q.clears > EDM28::RapidCycleCount(s.config.profile) ||
        (!q.recovering && q.clears > q.entries) ||
        (!q.entryProven && (q.entryTick || q.entryUs)) || (!q.clearProven && (q.clearTick || q.clearUs))) return false;
    if (q.entryProven && (!q.entries || !q.entryTick || !q.entryUs ||
        q.entryTick > g.sampleTick || q.entryUs > g.sampleMonotonicUs ||
        (!q.terminalResumeObserved && (!q.lowTick || !q.lowUs || q.entryTick < q.lowTick || q.entryUs < q.lowUs ||
            q.entryTick - q.lowTick < EDM43::ShortEntryTicks || q.entryUs - q.lowUs < EDM43::ShortEntryUs)))) return false;
    if (q.clearProven && (!q.clears || !q.highTick || !q.highUs || !q.clearTick || !q.clearUs ||
        q.clearTick > g.sampleTick || q.clearUs > g.sampleMonotonicUs || q.clearTick < q.highTick || q.clearUs < q.highUs ||
        q.clearTick - q.highTick < EDM43::ShortClearTicks || q.clearUs - q.highUs < EDM43::ShortClearUs)) return false;
    const bool mayProve = q.recovering || q.terminalResumeObserved ||
        (s.phase == EDMZFixturePhase::ShortStop && m_edmGapRapid.stopSourceRestarted);
    if (g.voltage == 20.0)
    {
        if (!q.shortIntent || q.pendingClear ||
            (g.state != EDMGapServo::ShortState::Entering && g.state != EDMGapServo::ShortState::Active) || !g.feedInhibited) return false;
        if (!q.lowTick) { q.lowTick = g.sampleTick; q.lowUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs) return false;
        if (mayProve && !q.entryProven && g.shortActive && g.state == EDMGapServo::ShortState::Active &&
            g.sampleTick - q.lowTick >= EDM43::ShortEntryTicks && g.sampleMonotonicUs - q.lowUs >= EDM43::ShortEntryUs)
        {
            if (q.entries == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.entryProven = true; q.entryTick = g.sampleTick; q.entryUs = g.sampleMonotonicUs; ++q.entries;
        }
        return true;
    }
    if (q.shortIntent)
    {
        if (!q.pendingClear || g.voltage != 50.0) return false;
        if (!q.highTick) { q.highTick = g.sampleTick; q.highUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.highTick || g.sampleMonotonicUs < q.highUs) return false;
        if (mayProve && g.state == EDMGapServo::ShortState::Clear && !g.feedInhibited && !g.shortActive &&
            g.sampleTick - q.highTick >= EDM43::ShortClearTicks && g.sampleMonotonicUs - q.highUs >= EDM43::ShortClearUs)
        {
            if (q.clears == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.shortIntent = q.pendingClear = false; q.clearProven = true;
            q.clearTick = g.sampleTick; q.clearUs = g.sampleMonotonicUs; ++q.clears;
        }
        return g.state == EDMGapServo::ShortState::Exiting || g.state == EDMGapServo::ShortState::Clear;
    }
    return !q.pendingClear && !g.feedInhibited && !g.shortActive && g.state == EDMGapServo::ShortState::Clear;
}

bool NCManager::RaiseEDMGapRapidAlarmSameThread()
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& q = m_edmGapRapid.shortState;
    const auto& g = s.gapShort.Snapshot();
    const auto completion = EvaluateEDMGapRapidCompletionSameThread(EDM49::Stage::FreshReleased);
    if (!q.active || !q.terminalAlarm || !q.shortIntent || q.pendingClear || !q.entryProven ||
        !g.valid || !g.fresh || !g.shortActive || !g.feedInhibited || g.state != EDMGapServo::ShortState::Active ||
        g.voltage != 20.0 || g.sampleTick != f.sampledTick || g.sampleMonotonicUs != f.monotonicUs ||
        !q.lowTick || !q.lowUs || g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs ||
        g.sampleTick - q.lowTick < EDM43::ShortEntryTicks || g.sampleMonotonicUs - q.lowUs < EDM43::ShortEntryUs ||
        !IsEDMGapRapidCompletionProfileSameThread() || !completion.Ready())
    {
        LogEDMGapRapidCompletionSameThread("REJECT_ALARM", EDM49::Stage::FreshReleased, completion);
        EndEDMZFixtureSameThread("FAIL", "P29_P30_ALARM_RECEIPT_INVALID"); return false;
    }
    LogEDMGapRapidCompletionSameThread("ALARM_RELEASED", EDM49::Stage::FreshReleased, completion);
    LogEDMZFixtureSameThread("EXPECTED_ALARM", "AL4001", "PERSISTENT_SHORT_AT_ORIGIN");
    m_edmZFixture.pending = false;
    RejectGapDryRunSameThread(AlarmManager::SHORT_CIRCUIT, m_gapDryRun.sourceLine, "PERSISTENT_SHORT_AT_ORIGIN");
    return false; // Expected alarm never releases the waiting G180 to M30.
}

bool NCManager::ProcessEDMGapRapidFixtureSameThread()
{
    auto& s = m_edmZFixture; auto& r = m_edmGapRapid; auto& q = r.shortState;
    const auto requiredCycles = EDM28::RapidCycleCount(s.config.profile);
    const bool endurance = EDM28::IsGapEnduranceProfile(s.config.profile);
    if (s.phase == EDMZFixturePhase::Finished) return false;
    if (s.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        if (!ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_HELD_SOURCE_INVALID"); return false; }
        if (s.feedback.capHeld && s.feedback.latchedFault)
        { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (f.publicationSequence <= s.publicationFloor || f.sampledTick <= s.tickFloor)
    {
        if (now - s.lastFreshMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick; s.lastFreshMs = now;
    const bool retiredFault = IsEDMGapRapidRetiredFaultSameThread();
    if (retiredFault && !f.queuedCancellation)
    {
        s.replanRetiredFaultSession = f.scope.session;
        s.replanRetiredFaultDisarmSequence = f.lastAppliedSequence; s.replanRetiredFaultDisarmTick = f.lastAppliedTick;
        s.replanRetiredFaultDisarmUs = f.lastAppliedMonotonicUs;
        s.replanRetiredFaultStopSequence = f.stopAppliedSequence; s.replanRetiredFaultStopTick = f.stopAppliedTick;
        s.replanRetiredFaultStopUs = f.stopAppliedMonotonicUs; s.replanRetiredFaultReason = f.reason;
    }
    if ((f.latchedFault && !retiredFault) || f.forceZeroOutput || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        const bool currentOwner = f.currentEpoch == m_motion.GetCurrentExecutionEpoch() &&
            f.currentOwner == static_cast<std::uint8_t>(m_programMotionLease.owner) && f.currentOwnerGeneration == m_programMotionLease.generation;
        if (s.curveTerminalHold && f.state == EDM28::State::Disarmed && !f.capHeld)
        {
            const auto stage = s.terminalResumeFloorSet ? EDM49::Stage::FreshReleased : EDM49::Stage::Released;
            const auto completion = EvaluateEDMGapRapidCompletionSameThread(stage);
            if (!IsEDMGapRapidCompletionProfileSameThread() || completion.status == EDM49::Status::Invalid)
            {
                LogEDMGapRapidCompletionSameThread("REJECT_HOLD", stage, completion);
                EndEDMZFixtureSameThread("FAIL", "P29_P30_TERMINAL_HOLD_PROOF_INVALID"); return false;
            }
            if (!currentOwner) return false;
            if (!s.terminalResumeFloorSet)
            {
                // A new observer authority must first establish its own source
                // floor. Keep the released scope and Stop/Disarm receipt intact.
                s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.receiptUs = f.monotonicUs;
                s.terminalResumeFloorSet = true;
                LogEDMGapRapidCompletionSameThread("RESUME_FLOOR", stage, completion); return false;
            }
            if (!completion.Ready()) return false;
            if (q.terminalAlarm)
            {
                if (!q.terminalResumeObserved)
                {
                    s.gapShort.Reset(); s.curveVoltage = 20.0;
                    q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                    q.clearTick = q.clearUs = 0ULL;
                    q.clearProven = false; q.terminalResumeObserved = true;
                }
                s.gapShort.Observe(f.sampledTick, f.monotonicUs, 20.0);
                if (!ObserveEDMGapRapidSameThread())
                { EndEDMZFixtureSameThread("FAIL", "P29_P30_TERMINAL_SOURCE_INVALID"); return false; }
                const auto& heldGap = s.gapShort.Snapshot();
                if (heldGap.state != EDMGapServo::ShortState::Active || !heldGap.shortActive ||
                    !q.lowTick || !q.lowUs || heldGap.sampleTick < q.lowTick || heldGap.sampleMonotonicUs < q.lowUs ||
                    heldGap.sampleTick - q.lowTick < EDM43::ShortEntryTicks ||
                    heldGap.sampleMonotonicUs - q.lowUs < EDM43::ShortEntryUs) return false;
                // This fresh source window replaces only the entry receipt,
                // never increments a script counter or credits the HOLD time.
                q.entryTick = heldGap.sampleTick; q.entryUs = heldGap.sampleMonotonicUs;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                // Released RT scope stays immutable. Only this final receipt may complete its retirement.
                m_gapDryRun.paused = false; s.curveTerminalHold = false;
                return RaiseEDMGapRapidAlarmSameThread();
            }
            // The session is already retired: this fresh exact released proof
            // permits completion only, never a new Arm or another motion.
            s.gapShort.Reset(); s.curveVoltage = 60.0;
            const auto& terminalGap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, 60.0);
            if (!terminalGap.valid || !terminalGap.fresh || terminalGap.feedInhibited || terminalGap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P29_P30_TERMINAL_SOURCE_INVALID"); return false; }
            if (s.phase == EDMZFixturePhase::Disarm) LogEDMZFixtureSameThread("SUMMARY", "PASS", "TERMINAL_HOLD_DISARM_ALREADY_PROVEN");
            m_gapDryRun.paused = false; m_gapDryRun.active = false; m_gapDryRun.result = 1U; s.curveTerminalHold = false;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            s.phase = EDMZFixturePhase::Finished;
            LogEDMGapRapidCompletionSameThread("COMPLETED", EDM49::Stage::FreshReleased, completion);
            LogEDMZFixtureSameThread("RECEIPT", "READY", "TERMINAL_HOLD_FRESH_RELEASE_NO_MOTION"); return true;
        }
        if (s.curvePreArmHold)
        {
            if (f.capHeld && f.originValid && EDM28::SameScope(f.scope, s.scope))
            {
                if (!IsEDMGapRapidConfigValidSameThread(true) || !std::isfinite(f.outerOriginPulse))
                { EndEDMZFixtureSameThread("FAIL", "P29_P30_FIRST_ARM_PROOF_INVALID"); return false; }
                s.originPulse = f.outerOriginPulse; s.originPinned = true; s.curvePreArmHold = false;
                s.phase = EDMZFixturePhase::ShortStop; s.pending = false;
            }
            else
            {
                const bool neverApplied = !f.capHeld && !f.originValid && !s.motionApplied && f.armReady && f.axisIdle &&
                    (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                        f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence));
                if (!neverApplied || !currentOwner) return false;
                if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                // A priority-cancelled, never-applied Arm cannot reuse its
                // retired session. No FIRST_ARM origin exists to change.
                s.scope.session = ++m_edmZFixtureSessionSerial;
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
                s.scope.ownerGeneration = m_gapDryRun.lease.generation;
                s.gapShort.Reset(); s.curveVoltage = 60.0; s.curvePreArmHold = false; s.curvePreArmSequence = 0ULL;
                m_gapDryRun.paused = false; s.pending = false; s.phase = EDMZFixturePhase::PreArm; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("RESTART", "WAIT", "UNAPPLIED_ARM_NEW_SESSION_NO_ORIGIN"); return false;
            }
        }
        const bool stopped = IsEDMZFixtureStoppedSameThread() && f.capHeld && f.frameCurrent && f.originValid &&
            f.outerOriginPulse == s.originPulse && f.lastAppliedKind == EDM28::RequestKind::Stop &&
            f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
            f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.stopAppliedMonotonicUs &&
            f.sampledTick >= f.stopAppliedTick && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
            f.monotonicUs >= f.stopAppliedMonotonicUs && f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs;
        if (!stopped) return false;
        if (f.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            f.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) || f.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - s.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        // Reset source/script only after pure motion Stop proof. HOLD duration
        // cannot become detector dwell, a zero-speed proof, or a cycle credit.
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        s.scope.ownerGeneration = m_gapDryRun.lease.generation;
        q.recovering = true; q.originStopPending = false; q.terminalResumeObserved = false;
        q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
        q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        q.entryProven = q.clearProven = false; q.entries = q.clears = 0U;
        q.terminalAlarm = false;
        s.gapShort.Reset(); s.curveVoltage = q.shortIntent ? (q.pendingClear ? 50.0 : 20.0) : 60.0;
        // Keep the last real Position tuple until a new Position replaces it.
        // A low, already-origin HOLD may retire without any further Position.
        s.curveBoundaryProof = 0ULL;
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL; s.curveStep = 0U; s.curveTerminalHold = false;
        s.cycles = s.curvePositions = s.curveZeros = 0U; s.windowStartMs = 0ULL;
        s.replanInterrupts = s.replanReturns = 0U; s.replanLaunchCounted = false;
        r.speed = EDMGapRapidSpeedProof{}; r.advanceProofs = r.retreatProofs = 0U; r.stopSourceRestarted = false;
        s.replanChangeTick = s.replanChangeUs = s.replanPriorProof = 0ULL;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        s.replanPositionSequence = s.replanPositionTick = s.replanPositionUs = 0ULL;
        s.replanPositionSourceTick = s.replanPositionSourceUs = 0ULL;
        s.replanChangeVoltage = s.replanChangeMmMin = s.replanPositionVoltage = s.replanPositionMmMin = s.replanPositionTargetMm = 0.0;
        s.terminalStopAppliedSequence = s.terminalStopAppliedTick = s.terminalStopAppliedMonotonicUs = 0ULL;
        m_gapDryRun.paused = false; s.pending = false; s.phaseStartMs = now;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ArmPending; return false;
    }
    if (f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration || f.currentEpoch != s.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending && !EDM28::SameScope(f.scope, s.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (f.stopLatched && s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending &&
        s.phase != EDMZFixturePhase::FinalStop && s.phase != EDMZFixturePhase::Disarm && s.phase != EDMZFixturePhase::Receipt &&
        s.phase != EDMZFixturePhase::ShortStop && s.phase != EDMZFixturePhase::ShortClearWait && s.phase != EDMZFixturePhase::ShortRearm && !q.originStopPending)
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    EDM43::Leg leg{};
    const bool hasLeg = EDM43::TryLeg(s.curveStep, q.persistent, leg);
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep < 3U && (!hasLeg || s.curveVoltage != leg.voltage))
    { EndEDMZFixtureSameThread("FAIL", "P29_P30_SCRIPT_SOURCE_CHANGED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && hasLeg && leg.triggerVoltage != 0.0)
        ObserveEDMGapRapidSpeedSameThread();
    const bool changed = s.phase == EDMZFixturePhase::Positive && s.pending && hasLeg && leg.triggerVoltage != 0.0 &&
        IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) && IsEDMGapRapidPositionSameThread(false) &&
        IsEDMGapRapidTriggerSameThread();
    if (changed)
    {
        s.curveVoltage = leg.triggerVoltage;
        if (s.curveVoltage == 20.0)
        {
            q.shortIntent = true; q.pendingClear = false; q.entryProven = q.clearProven = false;
            q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
            q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        }
        else if (s.curveVoltage == 50.0 && q.shortIntent)
        { q.pendingClear = true; q.highTick = q.highUs = 0ULL; q.clearProven = false; }
    }
    // A deceleration interval cannot count as the new post-Held source dwell.
    if (s.phase == EDMZFixturePhase::ShortStop && !r.stopSourceRestarted && IsEDMGapRapidStoppedSameThread())
    {
        s.gapShort.Reset(); r.stopSourceRestarted = true;
        if (s.curveStep == 0U)
        { q.lowTick = q.lowUs = q.entryTick = q.entryUs = 0ULL; q.entryProven = false; }
        else
        { q.highTick = q.highUs = q.clearTick = q.clearUs = 0ULL; q.clearProven = false; }
    }
    const auto& gap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, s.curveVoltage);
    const auto curve = EDM43::Evaluate(s.curveVoltage);
    // Unexpected source/short inhibition publishes priority Stop in End before
    // diagnostics or heartbeat, and retires through the existing AL2020 path.
    if (!ObserveEDMGapRapidSameThread() || !EDM43::IsScriptVoltage(s.curveVoltage) || !curve.valid)
    { EndEDMZFixtureSameThread("FAIL", "P29_P30_GAP_SOURCE_OR_SHORT"); return false; }
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const bool atOrigin = std::fabs(f.actualPulse - s.originPulse) <= tolerance &&
        std::fabs(f.commandPulse - s.originPulse) <= tolerance && std::fabs(f.planningPulse - s.originPulse) <= tolerance;
    const auto beginStoppedOriginAlarm = [&]() noexcept
    {
        if (!atOrigin || !q.shortIntent || q.pendingClear || !q.entryProven || s.curveVoltage != 20.0 ||
            !IsEDMZFixtureStoppedSameThread() || !f.frameCurrent || !f.originValid || f.outerOriginPulse != s.originPulse ||
            f.lastAppliedKind != EDM28::RequestKind::Stop || f.lastAppliedSequence != f.stopAppliedSequence ||
            f.lastAppliedTick != f.stopAppliedTick || f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs ||
            !f.stopAppliedMonotonicUs || f.sampledTick < f.stopAppliedTick ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs) return false;
        q.terminalAlarm = true; q.originStopPending = false;
        s.endpointProofKind = 1U; s.endpointProofSequence = f.proofSequence;
        s.endpointAppliedSequence = f.stopAppliedSequence; s.endpointAppliedTick = f.stopAppliedTick;
        s.endpointAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.curveBoundaryProof = f.proofSequence;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        // PriorityStop is intentionally idempotent while already Held. A
        // sequenced Stop creates the distinct terminal proof without moving.
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Stop))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_TERMINAL_STOP_SUBMIT_FAILED"); return true; }
        s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ALREADY_STOPPED_NEW_TERMINAL_STOP"); return true;
    };
    if (s.phase == EDMZFixturePhase::Receipt || s.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        const auto stage = s.phase == EDMZFixturePhase::Disarm ? EDM49::Stage::Released : EDM49::Stage::FreshReleased;
        const auto completion = EvaluateEDMGapRapidCompletionSameThread(stage);
        if (!IsEDMGapRapidCompletionProfileSameThread() || completion.status == EDM49::Status::Invalid)
        {
            LogEDMGapRapidCompletionSameThread("REJECT_RECEIPT", stage, completion);
            EndEDMZFixtureSameThread("FAIL", "P29_P30_FINAL_RECEIPT_INVALID"); return false;
        }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.receiptUs = f.monotonicUs;
            s.phaseStartMs = now; s.phase = EDMZFixturePhase::Receipt;
            LogEDMGapRapidCompletionSameThread("RELEASED", stage, completion);
            if (q.terminalAlarm)
            { LogEDMZFixtureSameThread("ALARM_RECEIPT", "WAIT", "LOW_ORIGIN_STOP_DISARM_PROVEN"); return false; }
            m_gapDryRun.active = false; m_gapDryRun.result = 1U;
            LogEDMZFixtureSameThread("SUMMARY", "PASS", endurance ? "300_LAUNCH_200_STOP_100_RETURN_CLEAR" : "18_LAUNCH_12_STOP_6_RETURN_6_CLEAR"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL) { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (!completion.Ready()) return false;
        if (q.terminalAlarm) return RaiseEDMGapRapidAlarmSameThread();
        s.phase = EDMZFixturePhase::Finished;
        LogEDMGapRapidCompletionSameThread("COMPLETED", stage, completion);
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (s.windowStartMs && now - s.windowStartMs > EDM28::RapidWindowTimeoutMs(s.config.profile))
    { EndEDMZFixtureSameThread("FAIL", endurance && !q.persistent ? "WINDOW_TIMEOUT_600000MS" : "WINDOW_TIMEOUT_30000MS"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        if (f.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        }
        else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapRapidConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            !f.originValid || !f.frameCurrent || !f.axisIdle || !std::isfinite(f.outerOriginPulse))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_ARM_NOT_ACTIVE"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.pending = false; s.phase = EDMZFixturePhase::Origin; s.windowStartMs = now; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("BEGIN", "RUNNING", "SIM_F30_SPEED_PROOF_SHORT_CLEAR"); return false;
    }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        if (q.terminalAlarm && s.endpointProofKind == 1U && !IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Stop))
        {
            if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P29_P30_TERMINAL_STOP_APPLY_TIMEOUT");
            return false;
        }
        const auto completion = EvaluateEDMGapRapidCompletionSameThread(EDM49::Stage::StopSettled);
        if (completion.status == EDM49::Status::Wait) return false;
        if (!completion.Ready())
        {
            LogEDMGapRapidCompletionSameThread("REJECT_STOP", EDM49::Stage::StopSettled, completion);
            EndEDMZFixtureSameThread("FAIL", "P29_P30_FINAL_STOP_PROOF_INVALID"); return false;
        }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        LogEDMGapRapidCompletionSameThread("STOP_PROVEN", EDM49::Stage::StopSettled, completion);
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "EXACT_STOP_DISARM_SUBMITTED"); return false;
    }
    if (s.phase == EDMZFixturePhase::ShortStop)
    {
        if (!IsEDMGapRapidStoppedSameThread())
        {
            if (f.stopProven) EndEDMZFixtureSameThread("FAIL", "P29_P30_CHANGED_STOP_PROOF_INVALID");
            else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "P29_P30_CHANGED_STOP_TIMEOUT");
            return false;
        }
        if (s.curveStep == 0U)
        {
            if (!q.shortIntent || q.pendingClear || s.curveVoltage != 20.0)
            { EndEDMZFixtureSameThread("FAIL", "P29_P30_SHORT_INTENT_LOST"); return false; }
            if (!q.entryProven) return false;
            if (atOrigin && beginStoppedOriginAlarm()) return false;
        }
        else if (s.curveStep != 1U || !q.clearProven || q.shortIntent || q.pendingClear) return false;
        s.replanStopSequence = f.stopAppliedSequence; s.replanStopTick = f.stopAppliedTick;
        s.replanStopUs = f.stopAppliedMonotonicUs; s.replanStopProof = f.proofSequence;
        ++s.replanInterrupts;
        if (s.curveStep == 0U) LogEDMZFixtureSameThread("SHORT_ENTRY", "PROVEN", "FRESH_ACTIVE_2000US_8TICKS");
        else LogEDMZFixtureSameThread("SHORT_CLEAR", "PROVEN", "FRESH_CLEAR_5000US_20TICKS");
        LogEDMZFixtureSameThread("CHANGE_PROVEN", "RUNNING", "SPEED_WINDOW_STOP_200_RT_HELD");
        EDM43::Leg next{};
        if (!EDM43::TryLeg(static_cast<unsigned>(s.curveStep)+1U,q.persistent,next))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_REPLAN_NEXT_STEP_INVALID"); return false; }
        s.curveVoltage = next.voltage;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_CHANGED_REARM_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ShortRearm; s.phaseStartMs = now; return false;
    }
    if (s.phase == EDMZFixturePhase::ShortRearm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P29_P30_REARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapRapidConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            f.stopLatched || !f.axisIdle || !f.originValid || !f.frameCurrent || f.outerOriginPulse != s.originPulse ||
            !s.replanStopSequence || f.stopAppliedSequence != s.replanStopSequence || f.stopAppliedTick != s.replanStopTick ||
            f.stopAppliedMonotonicUs != s.replanStopUs || f.proofSequence != s.replanStopProof ||
            f.lastAppliedTick <= s.replanStopTick || f.lastAppliedMonotonicUs <= s.replanStopUs ||
            f.lastCurveSequence != s.replanPositionSequence || f.lastCurveSampleTick != s.replanPositionSourceTick ||
            f.lastCurveSampleUs != s.replanPositionSourceUs || f.lastCurveVoltage != s.replanPositionVoltage ||
            f.lastSignedCurveMmMin != s.replanPositionMmMin)
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_REARM_RECEIPT_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_REARM_HEARTBEAT_FAILED"); return false; }
        ++s.curveStep; s.pending = false; s.phase = EDMZFixturePhase::Positive; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("REPLAN", "RUNNING", "SAME_ORIGIN_FRESH_ARM_NO_CREDIT"); return false;
    }
    if (s.phase != EDMZFixturePhase::Origin && s.phase != EDMZFixturePhase::Positive)
    { EndEDMZFixtureSameThread("FAIL", "P29_P30_PHASE_INVALID"); return false; }
    // The new GAP sample is observed exactly once on this RT publication.
    // Stop goes to the priority mailbox before heartbeat or any console output.
    if (changed)
    {
        if (!s.replanLaunchCounted)
        { ++s.curvePositions; s.replanLaunchCounted = true; }
        s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
        s.replanPositionSequence = s.curveSequence; s.replanPositionTick = f.lastAppliedTick;
        s.replanPositionUs = f.lastAppliedMonotonicUs; s.replanPositionSourceTick = s.curveSourceTick;
        s.replanPositionSourceUs = s.curveSourceUs; s.replanPositionVoltage = s.curveAppliedVoltage;
        s.replanPositionMmMin = s.curveSignedMmMin; s.replanPositionTargetMm = s.curveTargetMm;
        s.replanChangeTick = gap.sampleTick; s.replanChangeUs = gap.sampleMonotonicUs;
        s.replanChangeVoltage = gap.voltage; s.replanChangeMmMin = curve.limitedSpeedMmPerMin;
        s.replanPriorProof = f.proofSequence;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        if (s.curveStep == 0U) ++r.advanceProofs; else if (s.curveStep == 1U) ++r.retreatProofs;
        r.stopSourceRestarted = false;
        s.pending = false; s.phase = EDMZFixturePhase::ShortStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("CHANGE_STOP", "WAIT", "FRESH_MOVING_SAMPLE_PRIORITY_STOP"); return false;
    }
    if (q.originStopPending)
    {
        if (beginStoppedOriginAlarm()) return false;
        if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "P29_P30_ORIGIN_STOP_TIMEOUT");
        return false;
    }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep == 3U)
    {
        if (s.pending || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
            !IsEDMGapRapidPositionSameThread(true) || !s.curveBoundaryProof || f.proofSequence != s.curveBoundaryProof)
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_ZERO_ENDPOINT_REVOKED"); return false; }
        if (!s.curveZeroStartUs)
        {
            s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
            LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "ORIGIN_ZERO_NO_POSITION"); return false;
        }
        if (f.monotonicUs-s.curveZeroStartUs < EDM43::ZeroDwellUs ||
            f.sampledTick-s.curveZeroStartTick < EDM43::ZeroDwellTicks) return false;
        if (s.cycles+1U == requiredCycles && now-s.windowStartMs < 10000ULL) return false;
        ++s.curveZeros; ++s.cycles;
        LogEDMZFixtureSameThread(endurance ? "CYCLE" : "ZERO_PROVEN", "RUNNING", "ORIGIN_100MS_NO_POSITION");
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        if (s.cycles == requiredCycles)
        {
            if (s.curvePositions != 3U * requiredCycles || s.curveZeros != requiredCycles ||
                s.replanInterrupts != 2U * requiredCycles || s.replanReturns != requiredCycles || !atOrigin ||
                q.persistent || q.shortIntent || q.entries != requiredCycles || q.clears != requiredCycles ||
                r.advanceProofs != requiredCycles || r.retreatProofs != requiredCycles)
            { EndEDMZFixtureSameThread("FAIL", "P29_P30_FINAL_COUNT_MISMATCH"); return false; }
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop;
        }
        else { s.curveStep = 0U; s.curveVoltage = 80.0; }
        s.phaseStartMs = now; return false;
    }
    if (!s.pending)
    {
        const bool ready = f.axisIdle && std::fabs(f.cmdSpeedMmS) <= 1.0/s.config.pulsePerMm &&
            (f.state == EDM28::State::Armed || (f.state == EDM28::State::AtTarget && f.targetProven &&
                f.proofSequence && f.stableCycles >= s.config.settleCycles));
        if (!ready) return false;
        double target = 0.0;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (q.shortIntent)
            {
                if (q.pendingClear || !q.entryProven) return false;
                if (atOrigin)
                {
                    m_motion.RequestEDMZFixtureStop(s.scope.session); q.originStopPending = true; s.phaseStartMs = now;
                    LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ORIGIN_REQUEST_FRESH_STOP"); return false;
                }
            }
            if (atOrigin)
            {
                q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
                s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 80.0; return false;
            }
            const double low = (std::fmin)(f.actualPulse,(std::fmin)(f.commandPulse,f.planningPulse));
            const double high = (std::fmax)(f.actualPulse,(std::fmax)(f.commandPulse,f.planningPulse));
            const double recoveryVoltage = q.shortIntent ? (high < s.originPulse-tolerance ? 20.0 : 0.0) :
                high < s.originPulse-tolerance ? 50.0 : low > s.originPulse+tolerance ? 70.0 : 0.0;
            if (!recoveryVoltage) { EndEDMZFixtureSameThread("FAIL", "P29_P30_RECOVERY_DIRECTION_AMBIGUOUS"); return false; }
            if (s.curveVoltage != recoveryVoltage) { s.curveVoltage = recoveryVoltage; return false; }
        }
        else
        {
            if (!hasLeg || s.curveVoltage != leg.voltage)
            { EndEDMZFixtureSameThread("FAIL", "P29_P30_SCRIPT_VOLTAGE_MISMATCH"); return false; }
            target = leg.targetMm;
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position,target))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_CURVE_POSITION_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FRESH_CURVE_FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        if (s.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(f.scope,s.scope) &&
            !f.stopLatched && f.lastRequestSequence >= s.pendingSequence && f.lastAppliedSequence < s.pendingSequence)
        { s.pending = false; return false; }
        if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    if (!IsEDMGapRapidPositionSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "P29_P30_CURVE_RECEIPT_MISMATCH"); return false; }
    s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
    if (s.phase == EDMZFixturePhase::Positive && !s.replanLaunchCounted)
    {
        ++s.curvePositions; s.replanLaunchCounted = true;
        LogEDMZFixtureSameThread("APPLIED", "RUNNING", "SCRIPT_POSITION_APPLIED_ONCE");
    }
    if (s.phase == EDMZFixturePhase::Positive && hasLeg && leg.triggerVoltage != 0.0)
    {
        const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
        const double target = s.originPulse + s.curveTargetMm * s.config.pulsePerMm;
        const double margin = EDM43::TriggerRemainingMm * s.config.pulsePerMm;
        if (f.targetProven || direction * (target-f.actualPulse) < margin ||
            direction * (target-f.commandPulse) < margin || direction * (target-f.planningPulse) < margin)
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_MOVING_CHANGE_WINDOW_MISSED"); return false; }
    }
    if (f.targetProven)
    {
        if (!IsEDMGapRapidPositionSameThread(true))
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_POSITION_PROOF_INVALID"); return false; }
        const bool lowOrigin = q.shortIntent && !q.pendingClear && q.entryProven &&
            s.curveVoltage == 20.0 && s.curveAppliedVoltage == 20.0 && s.curveTargetMm == 0.0;
        if (s.phase == EDMZFixturePhase::Positive && s.curveStep != 2U && !lowOrigin)
        { EndEDMZFixtureSameThread("FAIL", "P29_P30_MOVING_CHANGE_WINDOW_MISSED"); return false; }
        s.pending = false; s.curveBoundaryProof = f.proofSequence;
        if (!atOrigin) { EndEDMZFixtureSameThread("FAIL", "P29_P30_RETURN_NOT_ORIGIN"); return false; }
        if (lowOrigin)
        {
            q.terminalAlarm = true; q.recovering = false;
            s.endpointProofKind = 2U; s.endpointProofSequence = f.proofSequence;
            s.endpointAppliedSequence = f.lastAppliedSequence; s.endpointAppliedTick = f.lastAppliedTick;
            s.endpointAppliedMonotonicUs = f.lastAppliedMonotonicUs;
            ++s.replanReturns;
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
            LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_POSITION_ORIGIN_NEW_TERMINAL_STOP"); return false;
        }
        if (s.phase == EDMZFixturePhase::Origin)
        {
            q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
            LogEDMZFixtureSameThread("RECOVERED", "RUNNING", "FRESH_CURVE_ORIGIN_NO_CYCLE_CREDIT");
            s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 80.0;
        }
        else
        {
            ++s.replanReturns;
            LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "F15_ORIGIN_POSITION_200_RT");
            s.curveStep = 3U; s.curveVoltage = 60.0;
        }
        s.phaseStartMs = now; return false;
    }
    if (now-s.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_MOVING_CHANGE_OR_ORIGIN");
    return false;
}

bool NCManager::IsEDMGapFeedUpdatePositionSameThread(bool endpoint) const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double target = s.originPulse + s.curveTargetMm * s.config.pulsePerMm;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const auto curve = EDM39::Evaluate(s.curveAppliedVoltage);
    const bool receipt = IsEDMGapFeedUpdateConfigValidSameThread(true) && EDM28::SameScope(f.scope, s.scope) &&
        s.curveSequence && s.curveSourceTick && s.curveSourceUs && curve.valid && curve.limitedSpeedMmPerMin &&
        curve.limitedSpeedMmPerMin == s.curveSignedMmMin && (!s.pending || s.pendingKind == f.lastAppliedKind) &&
        ((f.lastAppliedKind == EDM28::RequestKind::Position &&
            (!m_edmGapFeedUpdate.launchSequence || f.lastAppliedSequence == m_edmGapFeedUpdate.launchSequence)) ||
            (f.lastAppliedKind == EDM28::RequestKind::FeedUpdate && m_edmGapFeedUpdate.launchSequence &&
                m_edmGapFeedUpdate.launchSequence < f.lastAppliedSequence &&
                m_edmGapFeedUpdate.launchSequence > f.stopAppliedSequence &&
                m_edmGapFeedUpdate.launchSourceTick && m_edmGapFeedUpdate.launchSourceUs &&
                m_edmGapFeedUpdate.launchTick > m_edmGapFeedUpdate.launchSourceTick &&
                m_edmGapFeedUpdate.launchUs > m_edmGapFeedUpdate.launchSourceUs &&
                m_edmGapFeedUpdate.launchTick - m_edmGapFeedUpdate.launchSourceTick <= s.config.maximumIssueAgeTicks &&
                m_edmGapFeedUpdate.launchUs - m_edmGapFeedUpdate.launchSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
                f.lastAppliedTick > m_edmGapFeedUpdate.launchTick && f.lastAppliedMonotonicUs > m_edmGapFeedUpdate.launchUs &&
                EDM39::Evaluate(m_edmGapFeedUpdate.launchVoltage).valid &&
                EDM39::Evaluate(m_edmGapFeedUpdate.launchVoltage).limitedSpeedMmPerMin == m_edmGapFeedUpdate.launchMmMin &&
                m_edmGapFeedUpdate.launchTargetMm == s.curveTargetMm &&
                m_edmGapFeedUpdate.launchMmMin * s.curveSignedMmMin > 0.0)) &&
        f.lastAppliedSequence == s.curveSequence && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        f.lastAppliedTick > s.curveSourceTick && f.lastAppliedMonotonicUs > s.curveSourceUs &&
        f.lastAppliedTick - s.curveSourceTick <= s.config.maximumIssueAgeTicks &&
        f.lastAppliedMonotonicUs - s.curveSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.sampledTick >= f.lastAppliedTick && f.monotonicUs >= f.lastAppliedMonotonicUs &&
        f.frameCurrent && f.originValid && f.capHeld && !f.stopLatched && !f.latchedFault && !f.forceZeroOutput &&
        f.outerOriginPulse == s.originPulse && std::isfinite(target) && std::isfinite(tolerance) && tolerance > 0.0 &&
        std::isfinite(f.targetPulse) && f.targetPulse == target;
    if (!receipt || !endpoint) return receipt;
    return f.state == EDM28::State::AtTarget && f.targetProven && f.axisIdle && f.proofSequence &&
        f.stableCycles >= s.config.settleCycles && f.sampledTick - f.lastAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.lastAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        std::fabs(f.actualPulse - target) <= tolerance && std::fabs(f.commandPulse - target) <= tolerance &&
        std::fabs(f.planningPulse - target) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm;
}

bool NCManager::IsEDMGapFeedUpdateEligibleSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& u = m_edmGapFeedUpdate;
    if (!IsEDMGapFeedUpdateConfigValidSameThread(true) || u.shortState.recovering || u.shortState.shortIntent ||
        !EDM42::HasFeedUpdates(s.curveStep) || u.legUpdates >= EDM42::FeedUpdatesPerLeg || !u.launchSequence ||
        !s.pending || u.acceptedSequence != s.pendingSequence || !IsEDMGapFeedUpdatePositionSameThread(false) ||
        f.state != EDM28::State::Moving || f.axisIdle || f.targetProven ||
        f.lastAppliedSequence != u.acceptedSequence || f.lastAppliedTick != u.acceptedTick || f.lastAppliedMonotonicUs != u.acceptedUs ||
        f.sampledTick < u.acceptedTick || f.monotonicUs < u.acceptedUs ||
        f.sampledTick - u.acceptedTick < EDM42::FeedUpdateMinTicks ||
        f.monotonicUs - u.acceptedUs < EDM42::FeedUpdateMinUs ||
        std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm) return false;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const double progress = EDM42::FeedUpdateMinProgressMm * s.config.pulsePerMm;
    const double remaining = EDM42::FeedUpdateMinRemainingMm * s.config.pulsePerMm;
    return direction * f.cmdSpeedMmS > 0.0 &&
        direction * (f.actualPulse - u.acceptedActual) >= progress &&
        direction * (f.commandPulse - u.acceptedCommand) >= progress &&
        direction * (f.planningPulse - u.acceptedPlanning) >= progress &&
        direction * (f.targetPulse - f.actualPulse) >= remaining &&
        direction * (f.targetPulse - f.commandPulse) >= remaining &&
        direction * (f.targetPulse - f.planningPulse) >= remaining;
}

bool NCManager::IsEDMGapFeedUpdateConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& state = m_edmZFixture; const auto& c = state.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm && a.referenceOffsetPulse == b.referenceOffsetPulse &&
            a.hardwareSign == b.hardwareSign && a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS && a.accelerationTimeSec == b.accelerationTimeSec &&
            a.decelerationTimeSec == b.decelerationTimeSec && a.followingLimitMm == b.followingLimitMm &&
            a.positionToleranceMm == b.positionToleranceMm && a.excursionToleranceMm == b.excursionToleranceMm &&
            a.heartbeatTicks == b.heartbeatTicks && a.maximumIssueAgeTicks == b.maximumIssueAgeTicks &&
            a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles && a.profile == b.profile;
    };
    return state.gapCurveProfile && !state.gapCurveReplanProfile && !m_edmGapShortReplan.active && m_edmGapFeedUpdate.shortState.active &&
        (c.profile == EDM28::Profile::P27GapFeedUpdate || c.profile == EDM28::Profile::P28GapFeedPersistent) &&
        c.profile == m_edmGapFeedUpdate.shortState.profile &&
        m_edmGapFeedUpdate.shortState.persistent == (c.profile == EDM28::Profile::P28GapFeedPersistent) && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        c.feedMmMin == 20.0 && c.pdoCapMmS == .35 && c.outerHalfMm == .1 && c.targetHalfMm == EDM40::TargetHalfMm &&
        c.accelerationTimeSec == .2 && c.decelerationTimeSec == .2 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        same(c, state.gapCurveConfig) && (!requireFrozen || same(c, state.feedback.frozenConfig));
}

bool NCManager::IsEDMGapFeedUpdateRetiredFaultSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    // Disarm retains the previous fault for diagnostics. It is not a fault of
    // this new session until a first Arm has actually acquired its own cap.
    // Never clear RT state here: RT Arm still validates current authority,
    // source, configuration and 200 new idle cycles before clearing its latch.
    const bool firstArmPending = s.phase == EDMZFixturePhase::ArmPending && s.pending &&
        s.pendingKind == EDM28::RequestKind::Arm && s.pendingSequence > f.lastAppliedSequence;
    if (!IsEDMGapFeedUpdateConfigValidSameThread(false) || s.originPinned || s.motionApplied || s.windowStartMs ||
        s.curvePositions || s.cycles || !((s.phase == EDMZFixturePhase::PreArm && !s.pending) || firstArmPending) ||
        !f.latchedFault || f.reason == EDM28::Reason::None || f.state != EDM28::State::Disarmed || f.capHeld || f.stopLatched || f.forceZeroOutput || f.originValid ||
        !EDM28::ValidScope(f.scope) || f.scope.session > s.scope.session ||
        !f.publicationSequence || !f.sampledTick || !f.monotonicUs || !f.sourceFresh || !f.pdoValid || !f.contiguous || !f.clockValid ||
        !f.axisExists || !f.linear || !f.servoReady || !f.modeReady || f.modeValue != 9 || (f.statusWord & 0x006FU) != 0x0027U ||
        f.fault || f.hardPositive || f.hardNegative || !f.axisIdle || !f.frameCurrent ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.scope.axisMapGeneration != s.scope.axisMapGeneration || f.scope.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign ||
        !std::isfinite(f.actualPulse) || !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
        !std::isfinite(f.cmdSpeedMmS) || std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.commandPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        f.lastAppliedKind != EDM28::RequestKind::Disarm || !f.stopAppliedSequence || !f.stopAppliedTick || !f.stopAppliedMonotonicUs ||
        f.lastAppliedSequence <= f.stopAppliedSequence || f.lastAppliedTick <= f.stopAppliedTick ||
        f.lastAppliedMonotonicUs <= f.stopAppliedMonotonicUs || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.stopAppliedTick < s.config.settleCycles ||
        f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs || !f.proofSequence)
        return false;
    if (!f.queuedCancellation)
        return f.scope.session < s.scope.session && f.lastRequestSequence == f.lastAppliedSequence &&
            f.stopProven && f.stableCycles >= s.config.settleCycles;
    // HOLD can cancel a queued first Arm before it applies. RT then changes
    // the cancellation scope but deliberately retains the prior fault and
    // Disarm stamps. Require that exact receipt, previously proved here, plus
    // a fresh idle window; a current-session applied fault cannot use this path.
    const bool exactRetired = s.replanRetiredFaultSession && s.replanRetiredFaultSession < f.scope.session &&
        f.lastAppliedSequence == s.replanRetiredFaultDisarmSequence && f.lastAppliedTick == s.replanRetiredFaultDisarmTick &&
        f.lastAppliedMonotonicUs == s.replanRetiredFaultDisarmUs && f.stopAppliedSequence == s.replanRetiredFaultStopSequence &&
        f.stopAppliedTick == s.replanRetiredFaultStopTick && f.stopAppliedMonotonicUs == s.replanRetiredFaultStopUs &&
        f.reason == s.replanRetiredFaultReason && f.armReady && f.lastRequestSequence > f.lastAppliedSequence;
    const bool exactCancellation = f.scope.session < s.scope.session ||
        (m_gapDryRun.paused && s.curvePreArmHold && firstArmPending && EDM28::SameScope(f.scope, s.scope) &&
            s.curvePreArmSequence == s.pendingSequence && f.lastRequestSequence == s.curvePreArmSequence);
    return exactRetired && exactCancellation;
}

bool NCManager::IsEDMGapFeedUpdateReleasedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    return IsEDMGapFeedUpdateConfigValidSameThread(true) && IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) &&
        EDM28::SameScope(f.scope, s.scope) && s.originPinned && f.outerOriginPulse == s.originPulse &&
        f.state == EDM28::State::Disarmed && !f.capHeld && !f.latchedFault && !f.forceZeroOutput &&
        f.stopProven && f.frameCurrent && f.axisIdle && f.stableCycles >= s.config.settleCycles &&
        std::isfinite(f.targetPulse) && (f.targetPulse == s.originPulse ||
            (m_edmGapFeedUpdate.shortState.terminalAlarm && s.endpointProofKind == 1U)) &&
        std::fabs(f.actualPulse - s.originPulse) <= tolerance && std::fabs(f.commandPulse - s.originPulse) <= tolerance &&
        std::fabs(f.planningPulse - s.originPulse) <= tolerance && std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm &&
        s.terminalStopAppliedSequence && s.terminalStopAppliedTick && s.terminalStopAppliedMonotonicUs &&
        f.stopAppliedSequence == s.terminalStopAppliedSequence && f.stopAppliedTick == s.terminalStopAppliedTick &&
        f.stopAppliedMonotonicUs == s.terminalStopAppliedMonotonicUs && f.lastAppliedTick > f.stopAppliedTick &&
        f.lastAppliedMonotonicUs > f.stopAppliedMonotonicUs && f.sampledTick >= f.lastAppliedTick &&
        f.monotonicUs >= f.lastAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs && f.proofSequence == s.receiptProof &&
        f.proofSequence > s.curveBoundaryProof && f.lastCurveSequence == s.curveSequence &&
        f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
        f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin &&
        (m_edmGapFeedUpdate.shortState.terminalAlarm ?
            (m_edmGapFeedUpdate.shortState.shortIntent && !m_edmGapFeedUpdate.shortState.pendingClear &&
                m_edmGapFeedUpdate.shortState.entryProven && s.curveVoltage == 20.0 &&
                ((s.endpointProofKind == 1U && s.endpointProofSequence && s.endpointAppliedSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs) ||
                 (s.endpointProofKind == 2U && s.curveAppliedVoltage == 20.0 && s.curveSignedMmMin == -20.0 &&
                    s.curveTargetMm == 0.0 && s.endpointAppliedSequence == s.curveSequence &&
                    s.terminalStopAppliedSequence > s.endpointAppliedSequence &&
                    s.terminalStopAppliedTick > s.endpointAppliedTick &&
                    s.terminalStopAppliedMonotonicUs > s.endpointAppliedMonotonicUs))) :
            (!m_edmGapFeedUpdate.shortState.persistent && !m_edmGapFeedUpdate.shortState.shortIntent &&
                s.curveAppliedVoltage == 50.0 && s.curveSignedMmMin == -10.0 && s.curveTargetMm == 0.0 &&
                s.cycles == EDM40::CycleCount && s.curvePositions == EDM40::LaunchCount && s.curveZeros == EDM40::ZeroCount &&
                s.replanInterrupts == EDM40::InterruptCount && s.replanReturns == EDM40::ReturnCount &&
                m_edmGapFeedUpdate.shortState.entries == EDM41::ShortEntryCount && m_edmGapFeedUpdate.shortState.clears == EDM41::ShortClearCount &&
                m_edmGapFeedUpdate.updates == EDM42::RequiredFeedUpdates));
}

bool NCManager::IsEDMGapFeedUpdateTriggerSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{};
    if (!EDM42::TryLeg(s.curveStep, m_edmGapFeedUpdate.shortState.persistent, leg) || leg.triggerVoltage == 0.0 ||
        !IsEDMGapFeedUpdateConfigValidSameThread(true) || !IsEDMGapFeedUpdatePositionSameThread(false) ||
        (EDM42::HasFeedUpdates(s.curveStep) && m_edmGapFeedUpdate.legUpdates != EDM42::FeedUpdatesPerLeg) ||
        s.curveVoltage != leg.voltage || s.curveAppliedVoltage != leg.voltage || s.curveTargetMm != leg.targetMm ||
        f.state != EDM28::State::Moving || f.axisIdle || f.targetProven ||
        f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.lastAppliedTick < EDM40::TriggerMinimumTicks ||
        f.monotonicUs - f.lastAppliedMonotonicUs < EDM40::TriggerMinimumUs ||
        std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm) return false;
    const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
    const double progress = EDM40::TriggerProgressMm * s.config.pulsePerMm;
    const double remaining = EDM40::TriggerRemainingMm * s.config.pulsePerMm;
    const double target = s.originPulse + leg.targetMm * s.config.pulsePerMm;
    return direction * f.cmdSpeedMmS > 0.0 &&
        direction * (f.actualPulse - s.replanLaunchActual) >= progress &&
        direction * (f.commandPulse - s.replanLaunchCommand) >= progress &&
        direction * (f.planningPulse - s.replanLaunchPlanning) >= progress &&
        direction * (target - f.actualPulse) >= remaining &&
        direction * (target - f.commandPulse) >= remaining &&
        direction * (target - f.planningPulse) >= remaining;
}

bool NCManager::IsEDMGapFeedUpdateStoppedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM40::ScriptLeg leg{}; const auto changed = EDM39::Evaluate(s.replanChangeVoltage);
    return IsEDMGapFeedUpdateConfigValidSameThread(true) && EDM42::TryLeg(s.curveStep, m_edmGapFeedUpdate.shortState.persistent, leg) &&
        leg.triggerVoltage != 0.0 && s.replanChangeVoltage == leg.triggerVoltage && changed.valid &&
        changed.limitedSpeedMmPerMin == s.replanChangeMmMin && s.curveVoltage == s.replanChangeVoltage &&
        IsEDMZFixtureStoppedSameThread() && f.state == EDM28::State::Held && f.capHeld && f.stopLatched &&
        f.frameCurrent && f.originValid && f.outerOriginPulse == s.originPulse &&
        s.replanPositionSequence && s.replanPositionSequence == s.curveSequence &&
        s.replanPositionSourceTick == s.curveSourceTick && s.replanPositionSourceUs == s.curveSourceUs &&
        s.replanPositionVoltage == s.curveAppliedVoltage && s.replanPositionMmMin == s.curveSignedMmMin &&
        s.replanPositionTargetMm == s.curveTargetMm && s.replanPositionVoltage == leg.voltage &&
        s.replanPositionTargetMm == leg.targetMm &&
        f.targetPulse == s.originPulse + s.replanPositionTargetMm * s.config.pulsePerMm &&
        s.replanPositionTick > s.replanPositionSourceTick && s.replanPositionUs > s.replanPositionSourceUs &&
        s.replanPositionTick - s.replanPositionSourceTick <= s.config.maximumIssueAgeTicks &&
        s.replanPositionUs - s.replanPositionSourceUs <= s.config.maximumIssueAgeTicks * EDM28::CycleUs &&
        f.lastCurveSequence == s.replanPositionSequence && f.lastCurveSampleTick == s.replanPositionSourceTick &&
        f.lastCurveSampleUs == s.replanPositionSourceUs && f.lastCurveVoltage == s.replanPositionVoltage &&
        f.lastSignedCurveMmMin == s.replanPositionMmMin &&
        s.replanChangeTick > s.replanPositionTick && s.replanChangeUs > s.replanPositionUs &&
        f.stopAppliedSequence > s.replanPositionSequence && f.stopAppliedTick > s.replanChangeTick &&
        f.stopAppliedMonotonicUs > s.replanChangeUs &&
        f.lastAppliedKind == EDM28::RequestKind::Stop && f.lastAppliedSequence == f.stopAppliedSequence &&
        f.lastAppliedTick == f.stopAppliedTick && f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs &&
        f.proofSequence > s.replanPriorProof && f.sampledTick >= f.stopAppliedTick &&
        f.monotonicUs >= f.stopAppliedMonotonicUs && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        (!s.replanStopSequence || (f.stopAppliedSequence == s.replanStopSequence &&
            f.stopAppliedTick == s.replanStopTick && f.stopAppliedMonotonicUs == s.replanStopUs &&
            f.proofSequence == s.replanStopProof));
}

bool NCManager::ObserveEDMGapFeedUpdateSameThread() noexcept
{
    auto& q = m_edmGapFeedUpdate.shortState; const auto& s = m_edmZFixture; const auto& g = s.gapShort.Snapshot();
    if (!q.active || !g.valid || !g.fresh || g.fault != EDM37::GapShortFault::None ||
        g.voltage != s.curveVoltage || g.sampleTick != s.feedback.sampledTick ||
        g.sampleMonotonicUs != s.feedback.monotonicUs || !EDM42::IsScriptVoltage(g.voltage)) return false;
    // Boolean intent cannot substitute for a retained, coherent source proof.
    if (q.entries > EDM41::ShortEntryCount || q.clears > EDM41::ShortClearCount ||
        (!q.recovering && q.clears > q.entries) ||
        (!q.entryProven && (q.entryTick || q.entryUs)) || (!q.clearProven && (q.clearTick || q.clearUs))) return false;
    if (q.entryProven && (!q.entries || !q.entryTick || !q.entryUs ||
        q.entryTick > g.sampleTick || q.entryUs > g.sampleMonotonicUs ||
        (!q.terminalResumeObserved && (!q.lowTick || !q.lowUs || q.entryTick < q.lowTick || q.entryUs < q.lowUs ||
            q.entryTick - q.lowTick < EDM41::ShortEntryTicks || q.entryUs - q.lowUs < EDM41::ShortEntryUs)))) return false;
    if (q.clearProven && (!q.clears || !q.highTick || !q.highUs || !q.clearTick || !q.clearUs ||
        q.clearTick > g.sampleTick || q.clearUs > g.sampleMonotonicUs || q.clearTick < q.highTick || q.clearUs < q.highUs ||
        q.clearTick - q.highTick < EDM41::ShortClearTicks || q.clearUs - q.highUs < EDM41::ShortClearUs)) return false;
    if (g.voltage == 20.0)
    {
        if (!q.shortIntent || q.pendingClear ||
            (g.state != EDMGapServo::ShortState::Entering && g.state != EDMGapServo::ShortState::Active) || !g.feedInhibited) return false;
        if (!q.lowTick) { q.lowTick = g.sampleTick; q.lowUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs) return false;
        if (!q.entryProven && g.shortActive && g.state == EDMGapServo::ShortState::Active &&
            g.sampleTick - q.lowTick >= EDM41::ShortEntryTicks && g.sampleMonotonicUs - q.lowUs >= EDM41::ShortEntryUs)
        {
            if (q.entries == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.entryProven = true; q.entryTick = g.sampleTick; q.entryUs = g.sampleMonotonicUs; ++q.entries;
        }
        return true;
    }
    if (q.shortIntent)
    {
        if (!q.pendingClear || g.voltage != 50.0) return false;
        if (!q.highTick) { q.highTick = g.sampleTick; q.highUs = g.sampleMonotonicUs; }
        if (g.sampleTick < q.highTick || g.sampleMonotonicUs < q.highUs) return false;
        if (g.state == EDMGapServo::ShortState::Clear && !g.feedInhibited && !g.shortActive &&
            g.sampleTick - q.highTick >= EDM41::ShortClearTicks && g.sampleMonotonicUs - q.highUs >= EDM41::ShortClearUs)
        {
            if (q.clears == (std::numeric_limits<std::uint32_t>::max)()) return false;
            q.shortIntent = q.pendingClear = false; q.clearProven = true;
            q.clearTick = g.sampleTick; q.clearUs = g.sampleMonotonicUs; ++q.clears;
        }
        return g.state == EDMGapServo::ShortState::Exiting || g.state == EDMGapServo::ShortState::Clear;
    }
    return !q.pendingClear && !g.feedInhibited && !g.shortActive && g.state == EDMGapServo::ShortState::Clear;
}

bool NCManager::RaiseEDMGapFeedUpdateAlarmSameThread()
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& q = m_edmGapFeedUpdate.shortState;
    const auto& g = s.gapShort.Snapshot();
    if (!q.active || !q.terminalAlarm || !q.shortIntent || q.pendingClear || !q.entryProven ||
        !g.valid || !g.fresh || !g.shortActive || !g.feedInhibited || g.state != EDMGapServo::ShortState::Active ||
        g.voltage != 20.0 || g.sampleTick != f.sampledTick || g.sampleMonotonicUs != f.monotonicUs ||
        !q.lowTick || !q.lowUs || g.sampleTick < q.lowTick || g.sampleMonotonicUs < q.lowUs ||
        g.sampleTick - q.lowTick < EDM41::ShortEntryTicks || g.sampleMonotonicUs - q.lowUs < EDM41::ShortEntryUs ||
        !IsEDMGapFeedUpdateReleasedSameThread() || f.publicationSequence <= s.receiptPublication ||
        f.sampledTick <= s.receiptTick || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs)
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_ALARM_RECEIPT_INVALID"); return false; }
    LogEDMZFixtureSameThread("EXPECTED_ALARM", "AL4001", "PERSISTENT_SHORT_AT_ORIGIN");
    m_edmZFixture.pending = false;
    RejectGapDryRunSameThread(AlarmManager::SHORT_CIRCUIT, m_gapDryRun.sourceLine, "PERSISTENT_SHORT_AT_ORIGIN");
    return false; // Expected alarm never releases the waiting G180 to M30.
}

bool NCManager::ProcessEDMGapFeedUpdateFixtureSameThread()
{
    auto& s = m_edmZFixture; auto& u = m_edmGapFeedUpdate; auto& q = u.shortState;
    if (s.phase == EDMZFixturePhase::Finished) return false;
    if (s.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        if (!ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_HELD_SOURCE_INVALID"); return false; }
        if (s.feedback.capHeld && s.feedback.latchedFault)
        { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (f.publicationSequence <= s.publicationFloor || f.sampledTick <= s.tickFloor)
    {
        if (now - s.lastFreshMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick; s.lastFreshMs = now;
    const bool retiredFault = IsEDMGapFeedUpdateRetiredFaultSameThread();
    if (retiredFault && !f.queuedCancellation)
    {
        s.replanRetiredFaultSession = f.scope.session;
        s.replanRetiredFaultDisarmSequence = f.lastAppliedSequence; s.replanRetiredFaultDisarmTick = f.lastAppliedTick;
        s.replanRetiredFaultDisarmUs = f.lastAppliedMonotonicUs;
        s.replanRetiredFaultStopSequence = f.stopAppliedSequence; s.replanRetiredFaultStopTick = f.stopAppliedTick;
        s.replanRetiredFaultStopUs = f.stopAppliedMonotonicUs; s.replanRetiredFaultReason = f.reason;
    }
    if ((f.latchedFault && !retiredFault) || f.forceZeroOutput || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        const bool currentOwner = f.currentEpoch == m_motion.GetCurrentExecutionEpoch() &&
            f.currentOwner == static_cast<std::uint8_t>(m_programMotionLease.owner) && f.currentOwnerGeneration == m_programMotionLease.generation;
        if (s.curveTerminalHold && f.state == EDM28::State::Disarmed && !f.capHeld)
        {
            if (!IsEDMGapFeedUpdateReleasedSameThread())
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_TERMINAL_HOLD_PROOF_INVALID"); return false; }
            if (!currentOwner) return false;
            if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick ||
                f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs) return false;
            if (q.terminalAlarm)
            {
                if (!q.terminalResumeObserved)
                {
                    s.gapShort.Reset(); s.curveVoltage = 20.0;
                    q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                    q.clearTick = q.clearUs = 0ULL;
                    q.clearProven = false; q.terminalResumeObserved = true;
                }
                s.gapShort.Observe(f.sampledTick, f.monotonicUs, 20.0);
                if (!ObserveEDMGapFeedUpdateSameThread())
                { EndEDMZFixtureSameThread("FAIL", "P27_P28_TERMINAL_SOURCE_INVALID"); return false; }
                const auto& heldGap = s.gapShort.Snapshot();
                if (heldGap.state != EDMGapServo::ShortState::Active || !heldGap.shortActive ||
                    !q.lowTick || !q.lowUs || heldGap.sampleTick < q.lowTick || heldGap.sampleMonotonicUs < q.lowUs ||
                    heldGap.sampleTick - q.lowTick < EDM41::ShortEntryTicks ||
                    heldGap.sampleMonotonicUs - q.lowUs < EDM41::ShortEntryUs) return false;
                // This fresh source window replaces only the entry receipt,
                // never increments a script counter or credits the HOLD time.
                q.entryTick = heldGap.sampleTick; q.entryUs = heldGap.sampleMonotonicUs;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                // Released RT scope stays immutable. Only this final receipt may complete its retirement.
                m_gapDryRun.paused = false; s.curveTerminalHold = false;
                return RaiseEDMGapFeedUpdateAlarmSameThread();
            }
            // The session is already retired: this fresh exact released proof
            // permits completion only, never a new Arm or another motion.
            s.gapShort.Reset(); s.curveVoltage = 60.0;
            const auto& terminalGap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, 60.0);
            if (!terminalGap.valid || !terminalGap.fresh || terminalGap.feedInhibited || terminalGap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_TERMINAL_SOURCE_INVALID"); return false; }
            if (s.phase == EDMZFixturePhase::Disarm) LogEDMZFixtureSameThread("SUMMARY", "PASS", "TERMINAL_HOLD_DISARM_ALREADY_PROVEN");
            m_gapDryRun.paused = false; m_gapDryRun.active = false; m_gapDryRun.result = 1U; s.curveTerminalHold = false;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            s.phase = EDMZFixturePhase::Finished;
            LogEDMZFixtureSameThread("RECEIPT", "READY", "TERMINAL_HOLD_FRESH_RELEASE_NO_MOTION"); return true;
        }
        if (s.curvePreArmHold)
        {
            if (f.capHeld && f.originValid && EDM28::SameScope(f.scope, s.scope))
            {
                if (!IsEDMGapFeedUpdateConfigValidSameThread(true) || !std::isfinite(f.outerOriginPulse))
                { EndEDMZFixtureSameThread("FAIL", "P27_P28_FIRST_ARM_PROOF_INVALID"); return false; }
                s.originPulse = f.outerOriginPulse; s.originPinned = true; s.curvePreArmHold = false;
                s.phase = EDMZFixturePhase::ShortStop; s.pending = false;
            }
            else
            {
                const bool neverApplied = !f.capHeld && !f.originValid && !s.motionApplied && f.armReady && f.axisIdle &&
                    (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                        f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence));
                if (!neverApplied || !currentOwner) return false;
                if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                // A priority-cancelled, never-applied Arm cannot reuse its
                // retired session. No FIRST_ARM origin exists to change.
                s.scope.session = ++m_edmZFixtureSessionSerial;
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
                s.scope.ownerGeneration = m_gapDryRun.lease.generation;
                s.gapShort.Reset(); s.curveVoltage = 60.0; s.curvePreArmHold = false; s.curvePreArmSequence = 0ULL;
                m_gapDryRun.paused = false; s.pending = false; s.phase = EDMZFixturePhase::PreArm; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("RESTART", "WAIT", "UNAPPLIED_ARM_NEW_SESSION_NO_ORIGIN"); return false;
            }
        }
        const bool stopped = IsEDMZFixtureStoppedSameThread() && f.capHeld && f.frameCurrent && f.originValid &&
            f.outerOriginPulse == s.originPulse && f.lastAppliedKind == EDM28::RequestKind::Stop &&
            f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
            f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.stopAppliedMonotonicUs &&
            f.sampledTick >= f.stopAppliedTick && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
            f.monotonicUs >= f.stopAppliedMonotonicUs && f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs;
        if (!stopped) return false;
        if (f.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            f.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) || f.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - s.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        // A queued update may be cancelled before RT applies it. Only a known
        // applied curve tuple may survive HOLD; a requested feed is no receipt.
        if (s.pendingKind == EDM28::RequestKind::FeedUpdate && u.launchSequence)
        {
            const bool requestedApplied = f.lastCurveSequence == s.curveSequence &&
                f.lastCurveSampleTick == s.curveSourceTick && f.lastCurveSampleUs == s.curveSourceUs &&
                f.lastCurveVoltage == s.curveAppliedVoltage && f.lastSignedCurveMmMin == s.curveSignedMmMin;
            const bool previousApplied = f.lastCurveSequence == u.acceptedSequence &&
                f.lastCurveSampleTick == u.acceptedSourceTick && f.lastCurveSampleUs == u.acceptedSourceUs &&
                f.lastCurveVoltage == u.acceptedVoltage && f.lastSignedCurveMmMin == u.acceptedMmMin;
            if ((!requestedApplied && !previousApplied) || f.targetPulse != s.originPulse + u.launchTargetMm * s.config.pulsePerMm)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_HELD_FEED_RECEIPT_INVALID"); return false; }
            s.curveSequence = f.lastCurveSequence; s.curveSourceTick = f.lastCurveSampleTick; s.curveSourceUs = f.lastCurveSampleUs;
            s.curveAppliedVoltage = f.lastCurveVoltage; s.curveSignedMmMin = f.lastSignedCurveMmMin;
            s.curveTargetMm = u.launchTargetMm;
        }
        // Reset source/script only after pure motion Stop proof. HOLD duration
        // cannot become detector dwell, a zero-speed proof, or a cycle credit.
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        s.scope.ownerGeneration = m_gapDryRun.lease.generation;
        q.recovering = true; q.originStopPending = false; q.terminalResumeObserved = false;
        q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
        q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        q.entryProven = q.clearProven = false; q.entries = q.clears = 0U;
        q.terminalAlarm = false; u.updates = u.legUpdates = 0U;
        s.gapShort.Reset(); s.curveVoltage = q.shortIntent ? (q.pendingClear ? 50.0 : 20.0) : 60.0;
        // Keep the last real Position tuple until a new Position replaces it.
        // A low, already-origin HOLD may retire without any further Position.
        s.curveBoundaryProof = 0ULL;
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL; s.curveStep = 0U; s.curveTerminalHold = false;
        s.cycles = s.curvePositions = s.curveZeros = 0U; s.windowStartMs = 0ULL;
        s.replanInterrupts = s.replanReturns = 0U; s.replanLaunchCounted = false;
        s.replanChangeTick = s.replanChangeUs = s.replanPriorProof = 0ULL;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        s.replanPositionSequence = s.replanPositionTick = s.replanPositionUs = 0ULL;
        s.replanPositionSourceTick = s.replanPositionSourceUs = 0ULL;
        s.replanChangeVoltage = s.replanChangeMmMin = s.replanPositionVoltage = s.replanPositionMmMin = s.replanPositionTargetMm = 0.0;
        s.terminalStopAppliedSequence = s.terminalStopAppliedTick = s.terminalStopAppliedMonotonicUs = 0ULL;
        m_gapDryRun.paused = false; s.pending = false; s.phaseStartMs = now;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ArmPending; return false;
    }
    if (f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration || f.currentEpoch != s.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending && !EDM28::SameScope(f.scope, s.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (f.stopLatched && s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending &&
        s.phase != EDMZFixturePhase::FinalStop && s.phase != EDMZFixturePhase::Disarm && s.phase != EDMZFixturePhase::Receipt &&
        s.phase != EDMZFixturePhase::ShortStop && s.phase != EDMZFixturePhase::ShortClearWait && s.phase != EDMZFixturePhase::ShortRearm && !q.originStopPending)
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    if (s.pending && (s.pendingKind == EDM28::RequestKind::Position || s.pendingKind == EDM28::RequestKind::FeedUpdate) &&
        IsEDMZFixtureAppliedSameThread(s.pendingKind) && u.acceptedSequence != s.pendingSequence)
    {
        if (!IsEDMGapFeedUpdatePositionSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_CURVE_RECEIPT_MISMATCH"); return false; }
        if (s.pendingKind == EDM28::RequestKind::FeedUpdate)
        {
            if (!u.launchSequence || u.legUpdates >= EDM42::FeedUpdatesPerLeg || u.updates >= EDM42::RequiredFeedUpdates ||
                now - s.phaseStartMs >= 50ULL || f.monotonicUs - s.curveSourceUs > 50000ULL ||
                f.state != EDM28::State::Moving || f.axisIdle || f.targetProven ||
                f.lastAppliedTick <= u.acceptedTick || f.lastAppliedMonotonicUs <= u.acceptedUs ||
                std::fabs(f.cmdSpeedMmS) <= 1.0 / s.config.pulsePerMm ||
                f.cmdSpeedMmS * (s.curveSignedMmMin > 0.0 ? -1.0 : 1.0) <= 0.0)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_UPDATE_APPLIED_NOT_MOVING"); return false; }
            ++u.legUpdates; ++u.updates;
        }
        else
        {
            if (u.launchSequence)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_POSITION_IDENTITY_CHANGED"); return false; }
            u.launchSequence = f.lastAppliedSequence; u.launchTick = f.lastAppliedTick; u.launchUs = f.lastAppliedMonotonicUs;
            u.launchSourceTick = s.curveSourceTick; u.launchSourceUs = s.curveSourceUs;
            u.launchVoltage = s.curveAppliedVoltage; u.launchMmMin = s.curveSignedMmMin; u.launchTargetMm = s.curveTargetMm;
            if (s.phase == EDMZFixturePhase::Positive && !s.replanLaunchCounted)
            { ++s.curvePositions; s.replanLaunchCounted = true; }
        }
        u.acceptedSequence = f.lastAppliedSequence; u.acceptedTick = f.lastAppliedTick; u.acceptedUs = f.lastAppliedMonotonicUs;
        u.acceptedActual = f.actualPulse; u.acceptedCommand = f.commandPulse; u.acceptedPlanning = f.planningPulse;
        u.acceptedSourceTick = s.curveSourceTick; u.acceptedSourceUs = s.curveSourceUs;
        u.acceptedVoltage = s.curveAppliedVoltage; u.acceptedMmMin = s.curveSignedMmMin; u.acceptedTargetMm = s.curveTargetMm;
        s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
        LogEDMZFixtureSameThread(s.pendingKind == EDM28::RequestKind::FeedUpdate ? "FEED_APPLIED" : "APPLIED", "RUNNING",
            s.pendingKind == EDM28::RequestKind::FeedUpdate ? "SAME_TARGET_MOVING_FEED_APPLIED" : "SCRIPT_POSITION_APPLIED_ONCE");
    }
    EDM40::ScriptLeg leg{};
    const bool hasLeg = EDM42::TryLeg(s.curveStep, q.persistent, leg);
    double expectedVoltage = leg.voltage;
    if (hasLeg && EDM42::HasFeedUpdates(s.curveStep))
    {
        const bool awaitingUpdate = s.pending && s.pendingKind == EDM28::RequestKind::FeedUpdate &&
            u.acceptedSequence != s.pendingSequence;
        if (awaitingUpdate || u.legUpdates)
        {
            const unsigned index = awaitingUpdate ? u.legUpdates : u.legUpdates - 1U;
            if (!EDM42::TryFeedUpdateVoltage(s.curveStep, index, expectedVoltage))
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_UPDATE_INDEX_INVALID"); return false; }
        }
    }
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep < 4U && (!hasLeg || s.curveVoltage != expectedVoltage))
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_SCRIPT_SOURCE_CHANGED"); return false; }
    const bool changed = s.phase == EDMZFixturePhase::Positive && s.pending && hasLeg && leg.triggerVoltage != 0.0 &&
        (IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) || IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::FeedUpdate)) &&
        IsEDMGapFeedUpdatePositionSameThread(false) &&
        IsEDMGapFeedUpdateTriggerSameThread();
    if (changed)
    {
        s.curveVoltage = leg.triggerVoltage;
        if (s.curveVoltage == 20.0)
        {
            q.shortIntent = true; q.pendingClear = false; q.entryProven = q.clearProven = false;
            q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
            q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
        }
        else if (s.curveVoltage == 50.0 && q.shortIntent)
        { q.pendingClear = true; q.highTick = q.highUs = 0ULL; q.clearProven = false; }
    }
    const bool update = !changed && s.phase == EDMZFixturePhase::Positive && IsEDMGapFeedUpdateEligibleSameThread();
    if (update && !EDM42::TryFeedUpdateVoltage(s.curveStep, u.legUpdates, s.curveVoltage))
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_UPDATE_SOURCE_INVALID"); return false; }
    const auto& gap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, s.curveVoltage);
    const auto curve = EDM39::Evaluate(s.curveVoltage);
    // Unexpected source/short inhibition publishes priority Stop in End before
    // diagnostics or heartbeat, and retires through the existing AL2020 path.
    if (!ObserveEDMGapFeedUpdateSameThread() || !EDM42::IsScriptVoltage(s.curveVoltage) || !curve.valid)
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_GAP_SOURCE_OR_SHORT"); return false; }
    if (update)
    {
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::FeedUpdate, u.launchTargetMm))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_FEED_UPDATE_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now;
        LogEDMZFixtureSameThread("FEED_REQUEST", "WAIT", "SAME_FINITE_TARGET_NO_RELAUNCH"); return false;
    }
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const bool atOrigin = std::fabs(f.actualPulse - s.originPulse) <= tolerance &&
        std::fabs(f.commandPulse - s.originPulse) <= tolerance && std::fabs(f.planningPulse - s.originPulse) <= tolerance;
    const auto beginStoppedOriginAlarm = [&]() noexcept
    {
        if (!atOrigin || !q.shortIntent || q.pendingClear || !q.entryProven || s.curveVoltage != 20.0 ||
            !IsEDMZFixtureStoppedSameThread() || !f.frameCurrent || !f.originValid || f.outerOriginPulse != s.originPulse ||
            f.lastAppliedKind != EDM28::RequestKind::Stop || f.lastAppliedSequence != f.stopAppliedSequence ||
            f.lastAppliedTick != f.stopAppliedTick || f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs ||
            !f.stopAppliedMonotonicUs || f.sampledTick < f.stopAppliedTick ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs) return false;
        q.terminalAlarm = true; q.originStopPending = false;
        s.endpointProofKind = 1U; s.endpointProofSequence = f.proofSequence;
        s.endpointAppliedSequence = f.stopAppliedSequence; s.endpointAppliedTick = f.stopAppliedTick;
        s.endpointAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.curveBoundaryProof = f.proofSequence;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        // PriorityStop is intentionally idempotent while already Held. A
        // sequenced Stop creates the distinct terminal proof without moving.
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Stop))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_TERMINAL_STOP_SUBMIT_FAILED"); return true; }
        s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ALREADY_STOPPED_NEW_TERMINAL_STOP"); return true;
    };
    if (s.phase == EDMZFixturePhase::Receipt || s.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        const bool safe = IsEDMGapFeedUpdateReleasedSameThread();
        if (!safe) { EndEDMZFixtureSameThread("FAIL", "P27_P28_FINAL_RECEIPT_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.phaseStartMs = now;
            s.phase = EDMZFixturePhase::Receipt;
            if (q.terminalAlarm)
            { LogEDMZFixtureSameThread("ALARM_RECEIPT", "WAIT", "LOW_ORIGIN_STOP_DISARM_PROVEN"); return false; }
            m_gapDryRun.active = false; m_gapDryRun.result = 1U;
            LogEDMZFixtureSameThread("SUMMARY", "PASS", "24_LAUNCH_36_FEED_18_STOP_6_CLEAR"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL) { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick) return false;
        if (q.terminalAlarm) return RaiseEDMGapFeedUpdateAlarmSameThread();
        s.phase = EDMZFixturePhase::Finished;
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (s.windowStartMs && now - s.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        if (f.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        }
        else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapFeedUpdateConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            !f.originValid || !f.frameCurrent || !f.axisIdle || !std::isfinite(f.outerOriginPulse))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_ARM_NOT_ACTIVE"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.pending = false; s.phase = EDMZFixturePhase::Origin; s.windowStartMs = now; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("BEGIN", "RUNNING", "DYNAMIC_SIM_STOP_PROVE_REPLAN"); return false;
    }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        if (q.terminalAlarm && s.endpointProofKind == 1U && !IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Stop))
        {
            if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P27_P28_TERMINAL_STOP_APPLY_TIMEOUT");
            return false;
        }
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (!atOrigin || !f.frameCurrent || !f.originValid || f.lastAppliedKind != EDM28::RequestKind::Stop ||
            f.lastAppliedSequence != f.stopAppliedSequence || f.lastAppliedTick != f.stopAppliedTick ||
            f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs || !f.stopAppliedMonotonicUs ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs ||
            f.proofSequence <= s.curveBoundaryProof || f.stopAppliedTick <= s.curveSourceTick)
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_FINAL_STOP_PROOF_INVALID"); return false; }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "EXACT_STOP_DISARM_SUBMITTED"); return false;
    }
    if (s.phase == EDMZFixturePhase::ShortStop || s.phase == EDMZFixturePhase::ShortClearWait)
    {
        if (!IsEDMGapFeedUpdateStoppedSameThread())
        {
            if (s.phase == EDMZFixturePhase::ShortClearWait || f.stopProven)
                EndEDMZFixtureSameThread("FAIL", "P27_P28_CHANGED_STOP_PROOF_INVALID");
            else if (now - s.phaseStartMs > 1000ULL)
                EndEDMZFixtureSameThread("FAIL", "P27_P28_CHANGED_STOP_TIMEOUT");
            return false;
        }
        if (s.curveStep == 1U)
        {
            if (!q.shortIntent || q.pendingClear || s.curveVoltage != 20.0)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_SHORT_INTENT_LOST"); return false; }
            if (!q.entryProven) return false;
            if (atOrigin && beginStoppedOriginAlarm()) return false;
        }
        if (s.curveStep == 2U && (!q.clearProven || q.shortIntent || q.pendingClear)) return false;
        if (s.phase == EDMZFixturePhase::ShortStop)
        {
            s.replanStopSequence = f.stopAppliedSequence; s.replanStopTick = f.stopAppliedTick;
            s.replanStopUs = f.stopAppliedMonotonicUs; s.replanStopProof = f.proofSequence;
            ++s.replanInterrupts;
            if (s.curveStep == 1U) LogEDMZFixtureSameThread("SHORT_ENTRY", "PROVEN", "FRESH_ACTIVE_2000US_8TICKS");
            if (s.curveStep == 2U) LogEDMZFixtureSameThread("SHORT_CLEAR", "PROVEN", "FRESH_CLEAR_5000US_20TICKS");
            LogEDMZFixtureSameThread("CHANGE_PROVEN", "RUNNING", "NEW_SAMPLE_STOP_200_RT_HELD");
            if (s.curveStep == 0U)
            {
                if (s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0)
                { EndEDMZFixtureSameThread("FAIL", "P27_P28_HELD_ZERO_CURVE_INVALID"); return false; }
                s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
                s.phase = EDMZFixturePhase::ShortClearWait; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "HELD_ZERO_NO_POSITION"); return false;
            }
        }
        else
        {
            if (s.curveStep != 0U || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
                !s.curveZeroStartUs || f.monotonicUs < s.curveZeroStartUs || f.sampledTick < s.curveZeroStartTick)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_HELD_ZERO_REVOKED"); return false; }
            if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
                f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
            ++s.curveZeros;
            LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "HELD_500MS_NO_POSITION");
            s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        }
        EDM40::ScriptLeg next{};
        if (!EDM42::TryLeg(static_cast<unsigned>(s.curveStep)+1U,q.persistent,next))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_REPLAN_NEXT_STEP_INVALID"); return false; }
        s.curveVoltage = next.voltage;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_CHANGED_REARM_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ShortRearm; s.phaseStartMs = now; return false;
    }
    if (s.phase == EDMZFixturePhase::ShortRearm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P27_P28_REARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapFeedUpdateConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            f.stopLatched || !f.axisIdle || !f.originValid || !f.frameCurrent || f.outerOriginPulse != s.originPulse ||
            !s.replanStopSequence || f.stopAppliedSequence != s.replanStopSequence || f.stopAppliedTick != s.replanStopTick ||
            f.stopAppliedMonotonicUs != s.replanStopUs || f.proofSequence != s.replanStopProof ||
            f.lastAppliedTick <= s.replanStopTick || f.lastAppliedMonotonicUs <= s.replanStopUs ||
            f.lastCurveSequence != s.replanPositionSequence || f.lastCurveSampleTick != s.replanPositionSourceTick ||
            f.lastCurveSampleUs != s.replanPositionSourceUs || f.lastCurveVoltage != s.replanPositionVoltage ||
            f.lastSignedCurveMmMin != s.replanPositionMmMin)
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_REARM_RECEIPT_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_REARM_HEARTBEAT_FAILED"); return false; }
        ++s.curveStep; s.pending = false; s.phase = EDMZFixturePhase::Positive; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("REPLAN", "RUNNING", "SAME_ORIGIN_FRESH_ARM_NO_CREDIT"); return false;
    }
    if (s.phase != EDMZFixturePhase::Origin && s.phase != EDMZFixturePhase::Positive)
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_PHASE_INVALID"); return false; }
    // The new GAP sample is observed exactly once on this RT publication.
    // Stop goes to the priority mailbox before heartbeat or any console output.
    if (changed)
    {
        if (!s.replanLaunchCounted)
        { ++s.curvePositions; s.replanLaunchCounted = true; }
        s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
        s.replanPositionSequence = s.curveSequence; s.replanPositionTick = f.lastAppliedTick;
        s.replanPositionUs = f.lastAppliedMonotonicUs; s.replanPositionSourceTick = s.curveSourceTick;
        s.replanPositionSourceUs = s.curveSourceUs; s.replanPositionVoltage = s.curveAppliedVoltage;
        s.replanPositionMmMin = s.curveSignedMmMin; s.replanPositionTargetMm = s.curveTargetMm;
        s.replanChangeTick = gap.sampleTick; s.replanChangeUs = gap.sampleMonotonicUs;
        s.replanChangeVoltage = gap.voltage; s.replanChangeMmMin = curve.limitedSpeedMmPerMin;
        s.replanPriorProof = f.proofSequence;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        s.pending = false; s.phase = EDMZFixturePhase::ShortStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("CHANGE_STOP", "WAIT", "FRESH_MOVING_SAMPLE_PRIORITY_STOP"); return false;
    }
    if (q.originStopPending)
    {
        if (beginStoppedOriginAlarm()) return false;
        if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "P27_P28_ORIGIN_STOP_TIMEOUT");
        return false;
    }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep == 4U)
    {
        if (s.pending || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
            !IsEDMGapFeedUpdatePositionSameThread(true) || !s.curveBoundaryProof || f.proofSequence != s.curveBoundaryProof)
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_ZERO_ENDPOINT_REVOKED"); return false; }
        if (!s.curveZeroStartUs)
        {
            s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
            LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "ORIGIN_ZERO_NO_POSITION"); return false;
        }
        if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
            f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
        if (s.cycles+1U == EDM40::CycleCount && now-s.windowStartMs < 10000ULL) return false;
        ++s.curveZeros; ++s.cycles;
        LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "ORIGIN_500MS_NO_POSITION");
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        if (s.cycles == EDM40::CycleCount)
        {
            if (s.curvePositions != EDM40::LaunchCount || s.curveZeros != EDM40::ZeroCount ||
                s.replanInterrupts != EDM40::InterruptCount || s.replanReturns != EDM40::ReturnCount || !atOrigin ||
                q.persistent || q.shortIntent || q.entries != EDM41::ShortEntryCount || q.clears != EDM41::ShortClearCount ||
                u.updates != EDM42::RequiredFeedUpdates)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_FINAL_COUNT_MISMATCH"); return false; }
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop;
        }
        else { s.curveStep = 0U; s.curveVoltage = 70.0; }
        s.phaseStartMs = now; return false;
    }
    if (!s.pending)
    {
        const bool ready = f.axisIdle && std::fabs(f.cmdSpeedMmS) <= 1.0/s.config.pulsePerMm &&
            (f.state == EDM28::State::Armed || (f.state == EDM28::State::AtTarget && f.targetProven &&
                f.proofSequence && f.stableCycles >= s.config.settleCycles));
        if (!ready) return false;
        double target = 0.0;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (q.shortIntent)
            {
                if (q.pendingClear || !q.entryProven) return false;
                if (atOrigin)
                {
                    m_motion.RequestEDMZFixtureStop(s.scope.session); q.originStopPending = true; s.phaseStartMs = now;
                    LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_ORIGIN_REQUEST_FRESH_STOP"); return false;
                }
            }
            if (atOrigin)
            {
                q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
                s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0; return false;
            }
            const double low = (std::fmin)(f.actualPulse,(std::fmin)(f.commandPulse,f.planningPulse));
            const double high = (std::fmax)(f.actualPulse,(std::fmax)(f.commandPulse,f.planningPulse));
            const double recoveryVoltage = q.shortIntent ? (high < s.originPulse-tolerance ? 20.0 : 0.0) :
                high < s.originPulse-tolerance ? 50.0 : low > s.originPulse+tolerance ? 70.0 : 0.0;
            if (!recoveryVoltage) { EndEDMZFixtureSameThread("FAIL", "P27_P28_RECOVERY_DIRECTION_AMBIGUOUS"); return false; }
            if (s.curveVoltage != recoveryVoltage) { s.curveVoltage = recoveryVoltage; return false; }
        }
        else
        {
            if (!hasLeg || s.curveVoltage != leg.voltage)
            { EndEDMZFixtureSameThread("FAIL", "P27_P28_SCRIPT_VOLTAGE_MISMATCH"); return false; }
            target = leg.targetMm;
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position,target))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_CURVE_POSITION_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FRESH_CURVE_FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) && !IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::FeedUpdate))
    {
        if (s.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(f.scope,s.scope) &&
            !f.stopLatched && f.lastRequestSequence >= s.pendingSequence && f.lastAppliedSequence < s.pendingSequence)
        { s.pending = false; return false; }
        if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", s.pendingKind == EDM28::RequestKind::FeedUpdate ? "FEED_UPDATE_APPLICATION_TIMEOUT" : "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    if (!IsEDMGapFeedUpdatePositionSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_CURVE_RECEIPT_MISMATCH"); return false; }
    s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
    if (s.phase == EDMZFixturePhase::Positive && !s.replanLaunchCounted)
    {
        ++s.curvePositions; s.replanLaunchCounted = true;
        LogEDMZFixtureSameThread("APPLIED", "RUNNING", "SCRIPT_POSITION_APPLIED_ONCE");
    }
    if (s.phase == EDMZFixturePhase::Positive && EDM42::HasFeedUpdates(s.curveStep) && u.legUpdates < EDM42::FeedUpdatesPerLeg)
    {
        const double direction = s.curveSignedMmMin > 0.0 ? -1.0 : 1.0;
        const double margin = EDM42::FeedUpdateMinRemainingMm * s.config.pulsePerMm;
        if (f.targetProven || direction * (f.targetPulse - f.actualPulse) < margin ||
            direction * (f.targetPulse - f.commandPulse) < margin || direction * (f.targetPulse - f.planningPulse) < margin)
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_FEED_UPDATE_WINDOW_MISSED"); return false; }
    }
    if (f.targetProven)
    {
        if (!IsEDMGapFeedUpdatePositionSameThread(true))
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_POSITION_PROOF_INVALID"); return false; }
        const bool lowOrigin = q.shortIntent && !q.pendingClear && q.entryProven &&
            s.curveVoltage == 20.0 && s.curveAppliedVoltage == 20.0 && s.curveTargetMm == 0.0;
        if (s.phase == EDMZFixturePhase::Positive && s.curveStep != 3U && !lowOrigin)
        { EndEDMZFixtureSameThread("FAIL", "P27_P28_MOVING_CHANGE_WINDOW_MISSED"); return false; }
        s.pending = false; s.curveBoundaryProof = f.proofSequence;
        if (!atOrigin) { EndEDMZFixtureSameThread("FAIL", "P27_P28_RETURN_NOT_ORIGIN"); return false; }
        if (lowOrigin)
        {
            q.terminalAlarm = true; q.recovering = false;
            s.endpointProofKind = 2U; s.endpointProofSequence = f.proofSequence;
            s.endpointAppliedSequence = f.lastAppliedSequence; s.endpointAppliedTick = f.lastAppliedTick;
            s.endpointAppliedMonotonicUs = f.lastAppliedMonotonicUs;
            ++s.replanReturns;
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
            LogEDMZFixtureSameThread("SHORT_ORIGIN", "WAIT", "LOW_POSITION_ORIGIN_NEW_TERMINAL_STOP"); return false;
        }
        if (s.phase == EDMZFixturePhase::Origin)
        {
            q.recovering = false; q.entries = q.clears = 0U;
                q.entryProven = q.clearProven = false;
                q.lowTick = q.lowUs = q.highTick = q.highUs = 0ULL;
                q.entryTick = q.entryUs = q.clearTick = q.clearUs = 0ULL;
            LogEDMZFixtureSameThread("RECOVERED", "RUNNING", "FRESH_CURVE_ORIGIN_NO_CYCLE_CREDIT");
            s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0;
        }
        else
        {
            ++s.replanReturns;
            LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "F10_ORIGIN_POSITION_200_RT");
            s.curveStep = 4U; s.curveVoltage = 60.0;
        }
        s.phaseStartMs = now; return false;
    }
    if (now-s.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_MOVING_CHANGE_OR_ORIGIN");
    return false;
}

bool NCManager::ProcessEDMGapReplanFixtureSameThread()
{
    auto& s = m_edmZFixture;
    if (s.phase == EDMZFixturePhase::Finished) return false;
    if (s.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        if (!ReadEDMZFixtureFeedbackSameThread(false))
        { EndEDMZFixtureSameThread("FAIL", "P24_HELD_SOURCE_INVALID"); return false; }
        if (s.feedback.capHeld && s.feedback.latchedFault)
        { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
        PauseEDMZFixtureSameThread("HOLD"); return false;
    }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (f.publicationSequence <= s.publicationFloor || f.sampledTick <= s.tickFloor)
    {
        if (now - s.lastFreshMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick; s.lastFreshMs = now;
    const bool retiredFault = IsEDMGapReplanRetiredFaultSameThread();
    if (retiredFault && !f.queuedCancellation)
    {
        s.replanRetiredFaultSession = f.scope.session;
        s.replanRetiredFaultDisarmSequence = f.lastAppliedSequence; s.replanRetiredFaultDisarmTick = f.lastAppliedTick;
        s.replanRetiredFaultDisarmUs = f.lastAppliedMonotonicUs;
        s.replanRetiredFaultStopSequence = f.stopAppliedSequence; s.replanRetiredFaultStopTick = f.stopAppliedTick;
        s.replanRetiredFaultStopUs = f.stopAppliedMonotonicUs; s.replanRetiredFaultReason = f.reason;
    }
    if ((f.latchedFault && !retiredFault) || f.forceZeroOutput || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        const bool currentOwner = f.currentEpoch == m_motion.GetCurrentExecutionEpoch() &&
            f.currentOwner == static_cast<std::uint8_t>(m_programMotionLease.owner) && f.currentOwnerGeneration == m_programMotionLease.generation;
        if (s.curveTerminalHold && f.state == EDM28::State::Disarmed && !f.capHeld)
        {
            if (!IsEDMGapReplanReleasedSameThread())
            { EndEDMZFixtureSameThread("FAIL", "P24_TERMINAL_HOLD_PROOF_INVALID"); return false; }
            if (!currentOwner) return false;
            if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick ||
                f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs) return false;
            // The session is already retired: this fresh exact released proof
            // permits completion only, never a new Arm or another motion.
            s.gapShort.Reset(); s.curveVoltage = 60.0;
            const auto& terminalGap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, 60.0);
            if (!terminalGap.valid || !terminalGap.fresh || terminalGap.feedInhibited || terminalGap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P24_TERMINAL_SOURCE_INVALID"); return false; }
            if (s.phase == EDMZFixturePhase::Disarm) LogEDMZFixtureSameThread("SUMMARY", "PASS", "TERMINAL_HOLD_DISARM_ALREADY_PROVEN");
            m_gapDryRun.paused = false; m_gapDryRun.active = false; m_gapDryRun.result = 1U; s.curveTerminalHold = false;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            s.phase = EDMZFixturePhase::Finished;
            LogEDMZFixtureSameThread("RECEIPT", "READY", "TERMINAL_HOLD_FRESH_RELEASE_NO_MOTION"); return true;
        }
        if (s.curvePreArmHold)
        {
            if (f.capHeld && f.originValid && EDM28::SameScope(f.scope, s.scope))
            {
                if (!IsEDMGapReplanConfigValidSameThread(true) || !std::isfinite(f.outerOriginPulse))
                { EndEDMZFixtureSameThread("FAIL", "P24_FIRST_ARM_PROOF_INVALID"); return false; }
                s.originPulse = f.outerOriginPulse; s.originPinned = true; s.curvePreArmHold = false;
                s.phase = EDMZFixturePhase::ShortStop; s.pending = false;
            }
            else
            {
                const bool neverApplied = !f.capHeld && !f.originValid && !s.motionApplied && f.armReady && f.axisIdle &&
                    (!s.curvePreArmSequence || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
                        f.state == EDM28::State::Disarmed && f.lastRequestSequence == s.curvePreArmSequence));
                if (!neverApplied || !currentOwner) return false;
                if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
                    m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
                { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
                // A priority-cancelled, never-applied Arm cannot reuse its
                // retired session. No FIRST_ARM origin exists to change.
                s.scope.session = ++m_edmZFixtureSessionSerial;
                m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
                m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
                s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
                s.scope.ownerGeneration = m_gapDryRun.lease.generation;
                s.gapShort.Reset(); s.curveVoltage = 60.0; s.curvePreArmHold = false; s.curvePreArmSequence = 0ULL;
                m_gapDryRun.paused = false; s.pending = false; s.phase = EDMZFixturePhase::PreArm; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("RESTART", "WAIT", "UNAPPLIED_ARM_NEW_SESSION_NO_ORIGIN"); return false;
            }
        }
        const bool stopped = IsEDMZFixtureStoppedSameThread() && f.capHeld && f.frameCurrent && f.originValid &&
            f.outerOriginPulse == s.originPulse && f.lastAppliedKind == EDM28::RequestKind::Stop &&
            f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
            f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.stopAppliedMonotonicUs &&
            f.sampledTick >= f.stopAppliedTick && f.sampledTick - f.stopAppliedTick >= s.config.settleCycles &&
            f.monotonicUs >= f.stopAppliedMonotonicUs && f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs;
        if (!stopped) return false;
        if (f.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            f.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) || f.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - s.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "RESTART_EXHAUSTED"); return false; }
        // Reset source/script only after pure motion Stop proof. HOLD duration
        // cannot become detector dwell, a zero-speed proof, or a cycle credit.
        m_gapDryRun.test = ++m_gapDryRunSerial; ++m_gapDryRun.restarts;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
        s.scope.executionEpoch = m_gapDryRun.epoch; s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner);
        s.scope.ownerGeneration = m_gapDryRun.lease.generation;
        s.gapShort.Reset(); s.curveVoltage = 60.0; s.curveSignedMmMin = 0.0; s.curveAppliedVoltage = 0.0;
        s.curveSequence = s.curveSourceTick = s.curveSourceUs = s.curveBoundaryProof = 0ULL;
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL; s.curveStep = 0U; s.curveTerminalHold = false;
        s.cycles = s.curvePositions = s.curveZeros = 0U; s.windowStartMs = 0ULL;
        s.replanInterrupts = s.replanReturns = 0U; s.replanLaunchCounted = false;
        s.replanChangeTick = s.replanChangeUs = s.replanPriorProof = 0ULL;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        s.replanPositionSequence = s.replanPositionTick = s.replanPositionUs = 0ULL;
        s.replanPositionSourceTick = s.replanPositionSourceUs = 0ULL;
        s.replanChangeVoltage = s.replanChangeMmMin = s.replanPositionVoltage = s.replanPositionMmMin = s.replanPositionTargetMm = 0.0;
        s.terminalStopAppliedSequence = s.terminalStopAppliedTick = s.terminalStopAppliedMonotonicUs = 0ULL;
        m_gapDryRun.paused = false; s.pending = false; s.phaseStartMs = now;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "REARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ArmPending; return false;
    }
    if (f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration || f.currentEpoch != s.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending && !EDM28::SameScope(f.scope, s.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (f.stopLatched && s.phase != EDMZFixturePhase::PreArm && s.phase != EDMZFixturePhase::ArmPending &&
        s.phase != EDMZFixturePhase::FinalStop && s.phase != EDMZFixturePhase::Disarm && s.phase != EDMZFixturePhase::Receipt &&
        s.phase != EDMZFixturePhase::ShortStop && s.phase != EDMZFixturePhase::ShortClearWait && s.phase != EDMZFixturePhase::ShortRearm)
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    EDM40::ScriptLeg leg{};
    const bool hasLeg = EDM40::TryLeg(s.curveStep, leg);
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep < 4U && (!hasLeg || s.curveVoltage != leg.voltage))
    { EndEDMZFixtureSameThread("FAIL", "P24_SCRIPT_SOURCE_CHANGED"); return false; }
    const bool changed = s.phase == EDMZFixturePhase::Positive && s.pending && hasLeg && leg.triggerVoltage != 0.0 &&
        IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) && IsEDMGapCurvePositionSameThread(false) &&
        IsEDMGapReplanTriggerSameThread();
    if (changed) s.curveVoltage = leg.triggerVoltage;
    const auto& gap = s.gapShort.Observe(f.sampledTick, f.monotonicUs, s.curveVoltage);
    const auto curve = EDM39::Evaluate(s.curveVoltage);
    // Unexpected source/short inhibition publishes priority Stop in End before
    // diagnostics or heartbeat, and retires through the existing AL2020 path.
    if (!gap.valid || !gap.fresh || gap.fault != EDM37::GapShortFault::None || gap.feedInhibited || gap.shortActive ||
        gap.state != EDMGapServo::ShortState::Clear || !EDM39::IsScriptVoltage(s.curveVoltage) || !curve.valid)
    { EndEDMZFixtureSameThread("FAIL", "P24_GAP_SOURCE_OR_SHORT"); return false; }
    const double tolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
    const bool atOrigin = std::fabs(f.actualPulse - s.originPulse) <= tolerance &&
        std::fabs(f.commandPulse - s.originPulse) <= tolerance && std::fabs(f.planningPulse - s.originPulse) <= tolerance;
    if (s.phase == EDMZFixturePhase::Receipt || s.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        const bool safe = IsEDMGapReplanReleasedSameThread();
        if (!safe) { EndEDMZFixtureSameThread("FAIL", "P24_FINAL_RECEIPT_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.phaseStartMs = now;
            s.phase = EDMZFixturePhase::Receipt; m_gapDryRun.active = false; m_gapDryRun.result = 1U;
            LogEDMZFixtureSameThread("SUMMARY", "PASS", "REPLAN_24_LAUNCH_18_STOP_6_RETURN_12_ZERO"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL) { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (f.publicationSequence <= s.receiptPublication || f.sampledTick <= s.receiptTick) return false;
        s.phase = EDMZFixturePhase::Finished;
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (s.windowStartMs && now - s.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        if (f.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        }
        else if (now - s.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapReplanConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            !f.originValid || !f.frameCurrent || !f.axisIdle || !std::isfinite(f.outerOriginPulse))
        { EndEDMZFixtureSameThread("FAIL", "P24_ARM_NOT_ACTIVE"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.pending = false; s.phase = EDMZFixturePhase::Origin; s.windowStartMs = now; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("BEGIN", "RUNNING", "DYNAMIC_SIM_STOP_PROVE_REPLAN"); return false;
    }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (!atOrigin || !f.frameCurrent || !f.originValid || f.lastAppliedKind != EDM28::RequestKind::Stop ||
            f.lastAppliedSequence != f.stopAppliedSequence || f.lastAppliedTick != f.stopAppliedTick ||
            f.lastAppliedMonotonicUs != f.stopAppliedMonotonicUs || !f.stopAppliedMonotonicUs ||
            f.sampledTick - f.stopAppliedTick < s.config.settleCycles || f.monotonicUs < f.stopAppliedMonotonicUs ||
            f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs ||
            f.proofSequence <= s.curveBoundaryProof || f.stopAppliedTick <= s.curveSourceTick)
        { EndEDMZFixtureSameThread("FAIL", "P24_FINAL_STOP_PROOF_INVALID"); return false; }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("ORIGIN_PROOF", "WAIT", "EXACT_STOP_DISARM_SUBMITTED"); return false;
    }
    if (s.phase == EDMZFixturePhase::ShortStop || s.phase == EDMZFixturePhase::ShortClearWait)
    {
        if (!IsEDMGapReplanStoppedSameThread())
        {
            if (s.phase == EDMZFixturePhase::ShortClearWait || f.stopProven)
                EndEDMZFixtureSameThread("FAIL", "P24_CHANGED_STOP_PROOF_INVALID");
            else if (now - s.phaseStartMs > 1000ULL)
                EndEDMZFixtureSameThread("FAIL", "P24_CHANGED_STOP_TIMEOUT");
            return false;
        }
        if (s.phase == EDMZFixturePhase::ShortStop)
        {
            s.replanStopSequence = f.stopAppliedSequence; s.replanStopTick = f.stopAppliedTick;
            s.replanStopUs = f.stopAppliedMonotonicUs; s.replanStopProof = f.proofSequence;
            ++s.replanInterrupts;
            LogEDMZFixtureSameThread("CHANGE_PROVEN", "RUNNING", "NEW_SAMPLE_STOP_200_RT_HELD");
            if (s.curveStep == 0U)
            {
                if (s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0)
                { EndEDMZFixtureSameThread("FAIL", "P24_HELD_ZERO_CURVE_INVALID"); return false; }
                s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
                s.phase = EDMZFixturePhase::ShortClearWait; s.phaseStartMs = now;
                LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "HELD_ZERO_NO_POSITION"); return false;
            }
        }
        else
        {
            if (s.curveStep != 0U || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
                !s.curveZeroStartUs || f.monotonicUs < s.curveZeroStartUs || f.sampledTick < s.curveZeroStartTick)
            { EndEDMZFixtureSameThread("FAIL", "P24_HELD_ZERO_REVOKED"); return false; }
            if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
                f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
            ++s.curveZeros;
            LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "HELD_500MS_NO_POSITION");
            s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        }
        EDM40::ScriptLeg next{};
        if (!EDM40::TryLeg(static_cast<unsigned>(s.curveStep)+1U,next))
        { EndEDMZFixtureSameThread("FAIL", "P24_REPLAN_NEXT_STEP_INVALID"); return false; }
        s.curveVoltage = next.voltage;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "P24_CHANGED_REARM_FAILED"); return false; }
        s.phase = EDMZFixturePhase::ShortRearm; s.phaseStartMs = now; return false;
    }
    if (s.phase == EDMZFixturePhase::ShortRearm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "P24_REARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapReplanConfigValidSameThread(true) || !f.capHeld || f.state != EDM28::State::Armed ||
            f.stopLatched || !f.axisIdle || !f.originValid || !f.frameCurrent || f.outerOriginPulse != s.originPulse ||
            !s.replanStopSequence || f.stopAppliedSequence != s.replanStopSequence || f.stopAppliedTick != s.replanStopTick ||
            f.stopAppliedMonotonicUs != s.replanStopUs || f.proofSequence != s.replanStopProof ||
            f.lastAppliedTick <= s.replanStopTick || f.lastAppliedMonotonicUs <= s.replanStopUs ||
            f.lastCurveSequence != s.replanPositionSequence || f.lastCurveSampleTick != s.replanPositionSourceTick ||
            f.lastCurveSampleUs != s.replanPositionSourceUs || f.lastCurveVoltage != s.replanPositionVoltage ||
            f.lastSignedCurveMmMin != s.replanPositionMmMin)
        { EndEDMZFixtureSameThread("FAIL", "P24_REARM_RECEIPT_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "P24_REARM_HEARTBEAT_FAILED"); return false; }
        ++s.curveStep; s.pending = false; s.phase = EDMZFixturePhase::Positive; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("REPLAN", "RUNNING", "SAME_ORIGIN_FRESH_ARM_NO_CREDIT"); return false;
    }
    if (s.phase != EDMZFixturePhase::Origin && s.phase != EDMZFixturePhase::Positive)
    { EndEDMZFixtureSameThread("FAIL", "P24_PHASE_INVALID"); return false; }
    // The new GAP sample is observed exactly once on this RT publication.
    // Stop goes to the priority mailbox before heartbeat or any console output.
    if (changed)
    {
        if (!s.replanLaunchCounted)
        { ++s.curvePositions; s.replanLaunchCounted = true; }
        s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
        s.replanPositionSequence = s.curveSequence; s.replanPositionTick = f.lastAppliedTick;
        s.replanPositionUs = f.lastAppliedMonotonicUs; s.replanPositionSourceTick = s.curveSourceTick;
        s.replanPositionSourceUs = s.curveSourceUs; s.replanPositionVoltage = s.curveAppliedVoltage;
        s.replanPositionMmMin = s.curveSignedMmMin; s.replanPositionTargetMm = s.curveTargetMm;
        s.replanChangeTick = gap.sampleTick; s.replanChangeUs = gap.sampleMonotonicUs;
        s.replanChangeVoltage = gap.voltage; s.replanChangeMmMin = curve.limitedSpeedMmPerMin;
        s.replanPriorProof = f.proofSequence;
        s.replanStopSequence = s.replanStopTick = s.replanStopUs = s.replanStopProof = 0ULL;
        m_motion.RequestEDMZFixtureStop(s.scope.session);
        s.pending = false; s.phase = EDMZFixturePhase::ShortStop; s.phaseStartMs = now;
        LogEDMZFixtureSameThread("CHANGE_STOP", "WAIT", "FRESH_MOVING_SAMPLE_PRIORITY_STOP"); return false;
    }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (s.phase == EDMZFixturePhase::Positive && s.curveStep == 4U)
    {
        if (s.pending || s.curveVoltage != 60.0 || curve.limitedSpeedMmPerMin != 0.0 ||
            !IsEDMGapCurvePositionSameThread(true) || !s.curveBoundaryProof || f.proofSequence != s.curveBoundaryProof)
        { EndEDMZFixtureSameThread("FAIL", "P24_ZERO_ENDPOINT_REVOKED"); return false; }
        if (!s.curveZeroStartUs)
        {
            s.curveZeroStartUs = f.monotonicUs; s.curveZeroStartTick = f.sampledTick;
            LogEDMZFixtureSameThread("ZERO_BEGIN", "WAIT", "ORIGIN_ZERO_NO_POSITION"); return false;
        }
        if (f.monotonicUs-s.curveZeroStartUs < EDM40::ZeroDwellUs ||
            f.sampledTick-s.curveZeroStartTick < EDM40::ZeroDwellTicks) return false;
        if (s.cycles+1U == EDM40::CycleCount && now-s.windowStartMs < 10000ULL) return false;
        ++s.curveZeros; ++s.cycles;
        LogEDMZFixtureSameThread("ZERO_PROVEN", "RUNNING", "ORIGIN_500MS_NO_POSITION");
        s.curveZeroStartUs = s.curveZeroStartTick = 0ULL;
        if (s.cycles == EDM40::CycleCount)
        {
            if (s.curvePositions != EDM40::LaunchCount || s.curveZeros != EDM40::ZeroCount ||
                s.replanInterrupts != EDM40::InterruptCount || s.replanReturns != EDM40::ReturnCount || !atOrigin)
            { EndEDMZFixtureSameThread("FAIL", "P24_FINAL_COUNT_MISMATCH"); return false; }
            m_motion.RequestEDMZFixtureStop(s.scope.session); s.phase = EDMZFixturePhase::FinalStop;
        }
        else { s.curveStep = 0U; s.curveVoltage = 70.0; }
        s.phaseStartMs = now; return false;
    }
    if (!s.pending)
    {
        const bool ready = f.axisIdle && std::fabs(f.cmdSpeedMmS) <= 1.0/s.config.pulsePerMm &&
            (f.state == EDM28::State::Armed || (f.state == EDM28::State::AtTarget && f.targetProven &&
                f.proofSequence && f.stableCycles >= s.config.settleCycles));
        if (!ready) return false;
        double target = 0.0;
        if (s.phase == EDMZFixturePhase::Origin)
        {
            if (atOrigin) { s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0; return false; }
            const double low = (std::fmin)(f.actualPulse,(std::fmin)(f.commandPulse,f.planningPulse));
            const double high = (std::fmax)(f.actualPulse,(std::fmax)(f.commandPulse,f.planningPulse));
            const double recoveryVoltage = high < s.originPulse-tolerance ? 50.0 : low > s.originPulse+tolerance ? 70.0 : 0.0;
            if (!recoveryVoltage) { EndEDMZFixtureSameThread("FAIL", "P24_RECOVERY_DIRECTION_AMBIGUOUS"); return false; }
            if (s.curveVoltage != recoveryVoltage) { s.curveVoltage = recoveryVoltage; return false; }
        }
        else
        {
            if (!hasLeg || s.curveVoltage != leg.voltage)
            { EndEDMZFixtureSameThread("FAIL", "P24_SCRIPT_VOLTAGE_MISMATCH"); return false; }
            target = leg.targetMm;
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position,target))
        { EndEDMZFixtureSameThread("FAIL", "P24_CURVE_POSITION_SUBMIT_FAILED"); return false; }
        s.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FRESH_CURVE_FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        if (s.pendingKind == EDM28::RequestKind::Position && EDM28::SameScope(f.scope,s.scope) &&
            !f.stopLatched && f.lastRequestSequence >= s.pendingSequence && f.lastAppliedSequence < s.pendingSequence)
        { s.pending = false; return false; }
        if (now-s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    if (!IsEDMGapCurvePositionSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "P24_CURVE_RECEIPT_MISMATCH"); return false; }
    s.motionApplied = true; s.appliedTick = f.lastAppliedTick;
    if (s.phase == EDMZFixturePhase::Positive && !s.replanLaunchCounted)
    {
        ++s.curvePositions; s.replanLaunchCounted = true;
        LogEDMZFixtureSameThread("APPLIED", "RUNNING", "SCRIPT_POSITION_APPLIED_ONCE");
    }
    if (f.targetProven)
    {
        if (!IsEDMGapCurvePositionSameThread(true))
        { EndEDMZFixtureSameThread("FAIL", "P24_POSITION_PROOF_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Positive && s.curveStep != 3U)
        { EndEDMZFixtureSameThread("FAIL", "P24_MOVING_CHANGE_WINDOW_MISSED"); return false; }
        s.pending = false; s.curveBoundaryProof = f.proofSequence;
        if (!atOrigin) { EndEDMZFixtureSameThread("FAIL", "P24_RETURN_NOT_ORIGIN"); return false; }
        if (s.phase == EDMZFixturePhase::Origin)
        {
            LogEDMZFixtureSameThread("RECOVERED", "RUNNING", "FRESH_CURVE_ORIGIN_NO_CYCLE_CREDIT");
            s.phase = EDMZFixturePhase::Positive; s.curveStep = 0U; s.curveVoltage = 70.0;
        }
        else
        {
            ++s.replanReturns;
            LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "F10_ORIGIN_POSITION_200_RT");
            s.curveStep = 4U; s.curveVoltage = 60.0;
        }
        s.phaseStartMs = now; return false;
    }
    if (now-s.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_MOVING_CHANGE_OR_ORIGIN");
    return false;
}

bool NCManager::ProcessEDMZFixtureSameThread()
{
    if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile)) return ProcessEDMGapDynamicSameThread();
    if (!m_edmZFixture.cleanupPending &&
        (m_edmGapRapid.shortState.active || EDM28::IsGapRapidProfile(m_edmZFixture.config.profile) ||
            (m_edmZFixture.originPinned && EDM28::IsGapRapidProfile(m_edmZFixture.feedback.frozenConfig.profile))) &&
        !IsEDMGapRapidConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P29_P30_CONFIG_CHANGED"); return false; }
    if (m_edmGapRapid.shortState.active) return ProcessEDMGapRapidFixtureSameThread();
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmGapFeedUpdate.shortState.active || EDM28::IsGapFeedUpdateProfile(m_edmZFixture.config.profile) ||
            (m_edmZFixture.originPinned && EDM28::IsGapFeedUpdateProfile(m_edmZFixture.feedback.frozenConfig.profile))) &&
        !IsEDMGapFeedUpdateConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P27_P28_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmGapFeedUpdate.shortState.active) return ProcessEDMGapFeedUpdateFixtureSameThread();
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmGapShortReplan.active || m_edmZFixture.config.profile == EDM28::Profile::P25GapShortReplan ||
            m_edmZFixture.config.profile == EDM28::Profile::P26GapPersistentReplan ||
            (m_edmZFixture.originPinned && (m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P25GapShortReplan ||
                m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P26GapPersistentReplan))) &&
        !IsEDMGapShortReplanConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P25_P26_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmGapShortReplan.active) return ProcessEDMGapShortReplanFixtureSameThread();
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmZFixture.gapCurveReplanProfile || m_edmZFixture.config.profile == EDM28::Profile::P24GapCurveReplan ||
            (m_edmZFixture.originPinned && m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P24GapCurveReplan)) &&
        !IsEDMGapReplanConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P24_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmZFixture.gapCurveReplanProfile) return ProcessEDMGapReplanFixtureSameThread();
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmZFixture.gapCurveProfile || m_edmZFixture.config.profile == EDM28::Profile::P23GapCurveSegments ||
            (m_edmZFixture.originPinned && m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P23GapCurveSegments)) &&
        !IsEDMGapCurveConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P23_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmZFixture.gapCurveProfile) return ProcessEDMGapCurveFixtureSameThread();
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        (m_edmZFixture.gapPersistentProfile || m_edmZFixture.config.profile == EDM28::Profile::P22GapPersistentShort ||
            (m_edmZFixture.originPinned && m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P22GapPersistentShort)) &&
        !IsEDMGapPersistentConfigValidSameThread(m_edmZFixture.originPinned))
    { EndEDMZFixtureSameThread("FAIL", "P22_FROZEN_PROFILE_MISMATCH"); return false; }
    if (!m_edmZFixture.cleanupPending && (m_gapDryRun.active || m_gapDryRun.result == 1U) &&
        m_edmZFixture.originPinned && EDM28::IsSpeedRetreatProfile(m_edmZFixture.feedback.frozenConfig.profile) &&
        !SpeedRetreatFrozenConfigMatches(m_edmZFixture.config, m_edmZFixture.feedback.frozenConfig))
    { EndEDMZFixtureSameThread("FAIL", (m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P21GapShortRetreat) ?
        "P21_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P20UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P20UnloadedSpeedRetreat) ?
        "P20_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P19UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P19UnloadedSpeedRetreat) ?
        "P19_FROZEN_PROFILE_MISMATCH" : "P18_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmZFixture.config.profile == EDM28::Profile::P17PersistentShort || m_edmZFixture.config.profile == EDM28::Profile::P22GapPersistentShort) return ProcessEDMPersistentShortSameThread();
    if (m_edmZFixture.phase == EDMZFixturePhase::Finished) return false;
    const bool gapShortRetreat = m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat;
    const bool speedRetreat = EDM28::IsSpeedRetreatProfile(m_edmZFixture.config.profile);
    const bool retreatShort = m_edmZFixture.config.profile == EDM28::Profile::P16UnloadedRetreat || speedRetreat;
    if (m_edmZFixture.cleanupPending) { ServiceEDMZFixtureCleanupSameThread(); return false; }
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return false;
    if (!IsEDMZFixtureAuthorityCurrentSameThread(true))
    { EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED"); return false; }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { PauseEDMZFixtureSameThread("HOLD"); return false; }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < m_edmZFixture.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && now - m_edmZFixture.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == m_edmZFixture.lastClockMs)
    {
        if (++m_edmZFixture.stalledCalls >= 4096U)
        { EndEDMZFixtureSameThread("FAIL", "CLOCK_NOT_ADVANCING"); return false; }
    }
    else m_edmZFixture.stalledCalls = 0U;
    m_edmZFixture.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "INVALID_RT_SOURCE"); return false; }
    const auto& feedback = m_edmZFixture.feedback;
    if (feedback.axisMapGeneration != m_edmZFixture.scope.axisMapGeneration ||
        feedback.configGeneration != m_edmZFixture.scope.configGeneration ||
        feedback.pulsePerMm != m_edmZFixture.config.pulsePerMm ||
        feedback.referenceOffsetPulse != m_edmZFixture.config.referenceOffsetPulse ||
        feedback.hardwareSign != m_edmZFixture.config.hardwareSign)
    { EndEDMZFixtureSameThread("CANCELLED", "FRAME_CHANGED"); return false; }
    if (gapShortRetreat && (feedback.publicationSequence < m_edmZFixture.publicationFloor ||
        feedback.sampledTick < m_edmZFixture.tickFloor ||
        (m_edmZFixture.gapShort.Snapshot().valid &&
            ((feedback.sampledTick == m_edmZFixture.gapShort.Snapshot().sampleTick &&
                feedback.monotonicUs != m_edmZFixture.gapShort.Snapshot().sampleMonotonicUs) ||
             (feedback.sampledTick > m_edmZFixture.gapShort.Snapshot().sampleTick &&
                feedback.monotonicUs <= m_edmZFixture.gapShort.Snapshot().sampleMonotonicUs)))))
    { EndEDMZFixtureSameThread("FAIL", "P21_RT_SAMPLE_REVERSED"); return false; }
    if (feedback.publicationSequence <= m_edmZFixture.publicationFloor || feedback.sampledTick <= m_edmZFixture.tickFloor)
    {
        if (now - m_edmZFixture.lastFreshMs >= (m_edmZFixture.phase == EDMZFixturePhase::Receipt ? 250ULL : 50ULL))
            EndEDMZFixtureSameThread("FAIL", "RT_SOURCE_NOT_ADVANCING");
        return false;
    }
    m_edmZFixture.publicationFloor = feedback.publicationSequence; m_edmZFixture.tickFloor = feedback.sampledTick;
    m_edmZFixture.lastFreshMs = now;
    if (feedback.latchedFault && feedback.capHeld)
    { EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (!m_edmZFixture.resumeStartMs) m_edmZFixture.resumeStartMs = now;
        if (now - m_edmZFixture.resumeStartMs > 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "HELD_STOP_PROOF_TIMEOUT"); return false; }
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (feedback.currentEpoch != m_motion.GetCurrentExecutionEpoch() ||
            feedback.currentOwner != static_cast<std::uint8_t>(m_programMotionLease.owner) ||
            feedback.currentOwnerGeneration != m_programMotionLease.generation)
        {
            if (now - m_edmZFixture.resumeStartMs > 250ULL) EndEDMZFixtureSameThread("FAIL", "RESUME_RT_AUTHORITY_TIMEOUT");
            return false;
        }
        (void)RearmEDMZFixtureSameThread(now, true); return false;
    }
    if (feedback.currentOwner != m_edmZFixture.scope.owner || feedback.currentOwnerGeneration != m_edmZFixture.scope.ownerGeneration ||
        feedback.currentEpoch != m_edmZFixture.scope.executionEpoch)
    { EndEDMZFixtureSameThread("CANCELLED", "RT_AUTHORITY_CHANGED"); return false; }
    if (m_edmZFixture.originPinned && (feedback.outerOriginPulse != m_edmZFixture.originPulse ||
        std::fabs(feedback.actualPulse - m_edmZFixture.originPulse) > m_edmZFixture.config.outerHalfMm * m_edmZFixture.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "OUTER_ANCHOR_CHANGED"); return false; }
    if (m_edmZFixture.originPinned && (speedRetreat || EDM28::IsSpeedRetreatProfile(feedback.frozenConfig.profile)) &&
        !SpeedRetreatFrozenConfigMatches(m_edmZFixture.config, feedback.frozenConfig))
    { EndEDMZFixtureSameThread("FAIL", (m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P21GapShortRetreat) ?
        "P21_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P20UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P20UnloadedSpeedRetreat) ?
        "P20_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P19UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P19UnloadedSpeedRetreat) ?
        "P19_FROZEN_PROFILE_MISMATCH" : "P18_FROZEN_PROFILE_MISMATCH"); return false; }
    if (m_edmZFixture.phase == EDMZFixturePhase::Receipt)
    {
        const bool fresh = feedback.publicationSequence > m_edmZFixture.receiptPublication && feedback.sampledTick > m_edmZFixture.receiptTick;
        const bool safe = EDM28::SameScope(feedback.scope, m_edmZFixture.scope) &&
            IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) && feedback.state == EDM28::State::Disarmed &&
            !feedback.capHeld && feedback.stopProven && feedback.proofSequence == m_edmZFixture.receiptProof && feedback.axisIdle &&
            std::fabs(feedback.cmdSpeedMmS) <= 1.0 / m_edmZFixture.config.pulsePerMm &&
            std::fabs(feedback.actualPulse - m_edmZFixture.originPulse) <= .001 * m_edmZFixture.config.pulsePerMm &&
            std::fabs(feedback.commandPulse - m_edmZFixture.originPulse) <= .001 * m_edmZFixture.config.pulsePerMm &&
            std::fabs(feedback.planningPulse - m_edmZFixture.originPulse) <= .001 * m_edmZFixture.config.pulsePerMm;
        if (!safe || now - m_edmZFixture.phaseStartMs > 250ULL)
        { EndEDMZFixtureSameThread("FAIL", "FINAL_RECEIPT_INVALID"); return false; }
        if (!fresh) return false;
        m_edmZFixture.phase = EDMZFixturePhase::Finished;
        LogEDMZFixtureSameThread("RECEIPT", "READY", "FRESH_REAL_ORIGIN_STOP_DISARM"); return true;
    }
    if (m_edmZFixture.windowStartMs && now - m_edmZFixture.windowStartMs > 30000ULL)
    { EndEDMZFixtureSameThread("FAIL", "WINDOW_TIMEOUT_30000MS"); return false; }
    if (m_edmZFixture.phase != EDMZFixturePhase::PreArm && m_edmZFixture.phase != EDMZFixturePhase::ArmPending &&
        !EDM28::SameScope(feedback.scope, m_edmZFixture.scope))
    { EndEDMZFixtureSameThread("FAIL", "RT_SCOPE_CHANGED"); return false; }
    if (feedback.forceZeroOutput || feedback.state == EDM28::State::Fault ||
        (feedback.stopLatched && m_edmZFixture.phase != EDMZFixturePhase::ShortStop &&
         m_edmZFixture.phase != EDMZFixturePhase::ArmPending && m_edmZFixture.phase != EDMZFixturePhase::FinalStop && m_edmZFixture.phase != EDMZFixturePhase::Disarm))
    { EndEDMZFixtureSameThread("FAIL", "RT_STOP_OR_FAULT"); return false; }
    if (gapShortRetreat)
    {
        // A real RT sample is the only clock and identity supplied to the
        // detector. The local SIM script never writes the system GAP source.
        if (m_edmZFixture.phase == EDMZFixturePhase::ShortClearWait)
        {
            const double tolerance = .001 * m_edmZFixture.config.pulsePerMm;
            const double target = m_edmZFixture.originPulse + .02 * m_edmZFixture.config.pulsePerMm;
            // Recovery voltage is authorized only while the exact retreat
            // Position/proof and the stopped boundary survive every sample.
            if (!m_edmZFixture.gapClearProof || !m_edmZFixture.gapClearAppliedSequence ||
                !m_edmZFixture.gapClearAppliedTick || !m_edmZFixture.gapClearAppliedUs ||
                !feedback.capHeld || !feedback.frameCurrent || !feedback.originValid ||
                feedback.state != EDM28::State::AtTarget || !feedback.targetProven || !feedback.axisIdle ||
                feedback.proofSequence != m_edmZFixture.gapClearProof ||
                feedback.lastAppliedKind != EDM28::RequestKind::Position ||
                feedback.lastAppliedSequence != m_edmZFixture.gapClearAppliedSequence ||
                feedback.lastAppliedTick != m_edmZFixture.gapClearAppliedTick ||
                feedback.lastAppliedMonotonicUs != m_edmZFixture.gapClearAppliedUs ||
                feedback.stopAppliedSequence != m_edmZFixture.shortStopAppliedSequence ||
                feedback.stopAppliedTick != m_edmZFixture.shortStopAppliedTick ||
                feedback.stopAppliedMonotonicUs != m_edmZFixture.shortStopAppliedMonotonicUs ||
                feedback.stableCycles < m_edmZFixture.config.settleCycles ||
                feedback.publicationSequence <= m_edmZFixture.gapClearPublication ||
                feedback.sampledTick <= m_edmZFixture.gapClearTick || feedback.monotonicUs <= m_edmZFixture.gapClearUs ||
                !std::isfinite(m_edmZFixture.gapClearTargetPulse) || feedback.targetPulse != m_edmZFixture.gapClearTargetPulse ||
                !std::isfinite(target) || !std::isfinite(tolerance) || tolerance <= 0.0 ||
                std::fabs(feedback.targetPulse - target) > tolerance ||
                std::fabs(feedback.actualPulse - target) > tolerance || std::fabs(feedback.commandPulse - target) > tolerance ||
                std::fabs(feedback.planningPulse - target) > tolerance ||
                std::fabs(feedback.cmdSpeedMmS) > 1.0 / m_edmZFixture.config.pulsePerMm ||
                !m_edmZFixture.shortDone || m_edmZFixture.cycles == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles + 1U ||
                m_edmZFixture.verifiedRetreats != m_edmZFixture.simulatedShortStops ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles)
            { EndEDMZFixtureSameThread("FAIL", "P21_CLEAR_ENDPOINT_REVOKED"); return false; }
            if (!m_edmZFixture.gapClearHighUs) m_edmZFixture.gapClearHighUs = feedback.monotonicUs;
            m_edmZFixture.gapShortLow = false;
        }
        const bool inject = m_edmZFixture.phase == EDMZFixturePhase::Negative && !m_edmZFixture.shortDone &&
            IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position) &&
            feedback.sampledTick - feedback.lastAppliedTick >= 800ULL;
        if (inject)
        {
            if (feedback.state != EDM28::State::Moving || feedback.targetProven || feedback.axisIdle)
            { EndEDMZFixtureSameThread("FAIL", "SHORT_TEST_REQUIRES_MOVING_LEG"); return false; }
            if (m_edmZFixture.gapShortLow || m_edmZFixture.simulatedShortStops == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles || m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles)
            { EndEDMZFixtureSameThread("FAIL", "SHORT_INJECTION_COUNT_MISMATCH"); return false; }
            m_edmZFixture.gapShortLow = true;
        }
        const auto& gap = m_edmZFixture.gapShort.Observe(feedback.sampledTick, feedback.monotonicUs,
            m_edmZFixture.gapShortLow ? 20.0 : 50.0);
        if (!gap.valid || !gap.fresh)
        { EndEDMZFixtureSameThread("FAIL", "P21_GAP_SAMPLE_INVALID"); return false; }
        if (inject)
        {
            if (!gap.feedInhibited || gap.state != EDMGapServo::ShortState::Entering || gap.shortActive)
            { EndEDMZFixtureSameThread("FAIL", "P21_SHORT_ENTRY_INVALID"); return false; }
            // Entering already inhibits feed: Stop has priority before any
            // log, heartbeat, re-arm or continuation of the negative leg.
            m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
            ++m_edmZFixture.simulatedShortStops;
            m_edmZFixture.phase = EDMZFixturePhase::ShortStop; m_edmZFixture.pending = false;
            m_edmZFixture.phaseStartMs = now;
            LogEDMZFixtureSameThread("SHORT_STOP", "WAIT", "DETECTOR_ENTERING_PRIORITY_STOP"); return false;
        }
        const bool retreatArm = m_edmZFixture.phase == EDMZFixturePhase::ArmPending && !m_edmZFixture.beginPending;
        if ((m_edmZFixture.phase == EDMZFixturePhase::Retreat || retreatArm) &&
            (!m_edmZFixture.gapShortLow || !gap.shortActive || !gap.feedInhibited || gap.voltage != 20.0))
        { EndEDMZFixtureSameThread("FAIL", "P21_RETREAT_WITHOUT_ACTIVE"); return false; }
        if ((m_edmZFixture.phase == EDMZFixturePhase::Positive || m_edmZFixture.phase == EDMZFixturePhase::Negative ||
            m_edmZFixture.phase == EDMZFixturePhase::Origin) && (gap.feedInhibited || gap.shortActive || m_edmZFixture.gapShortLow))
        { EndEDMZFixtureSameThread("FAIL", "P21_FEED_INHIBITED"); return false; }
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::PreArm)
    {
        if (feedback.armReady)
        {
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "ARM_SUBMIT_FAILED"); return false; }
            m_edmZFixture.phase = EDMZFixturePhase::ArmPending; m_edmZFixture.phaseStartMs = now;
        }
        else if (now - m_edmZFixture.phaseStartMs > 1000ULL) EndEDMZFixtureSameThread("FAIL", "ARM_IDLE_PROOF_TIMEOUT");
        return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - m_edmZFixture.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "ARM_APPLICATION_TIMEOUT"); return false; }
        if (speedRetreat && !SpeedRetreatFrozenConfigMatches(m_edmZFixture.config, feedback.frozenConfig))
        { EndEDMZFixtureSameThread("FAIL", (m_edmZFixture.config.profile == EDM28::Profile::P21GapShortRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P21GapShortRetreat) ?
        "P21_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P20UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P20UnloadedSpeedRetreat) ?
        "P20_FROZEN_PROFILE_MISMATCH" : (m_edmZFixture.config.profile == EDM28::Profile::P19UnloadedSpeedRetreat ||
        m_edmZFixture.feedback.frozenConfig.profile == EDM28::Profile::P19UnloadedSpeedRetreat) ?
        "P19_FROZEN_PROFILE_MISMATCH" : "P18_FROZEN_PROFILE_MISMATCH"); return false; }
        if (!feedback.capHeld || feedback.state != EDM28::State::Armed)
        { EndEDMZFixtureSameThread("FAIL", "ARM_NOT_ACTIVE"); return false; }
        if ((m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat || retreatShort) && !m_edmZFixture.beginPending)
        {
            // A queued Arm is not a clear. Match its exact application to the
            // retained proof and Stop stamp earned before this re-arm request.
            if (m_edmZFixture.armNextPhase != (retreatShort ? EDMZFixturePhase::Retreat : EDMZFixturePhase::Negative) || !m_edmZFixture.shortDone ||
                !m_edmZFixture.shortStopProof || feedback.proofSequence != m_edmZFixture.shortStopProof ||
                feedback.stopAppliedSequence != m_edmZFixture.shortStopAppliedSequence ||
                feedback.stopAppliedTick != m_edmZFixture.shortStopAppliedTick ||
                feedback.stopAppliedMonotonicUs != m_edmZFixture.shortStopAppliedMonotonicUs ||
                feedback.lastAppliedTick <= m_edmZFixture.shortStopProofTick ||
                m_edmZFixture.verifiedShortClears == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.verifiedShortClears + 1U)
            { EndEDMZFixtureSameThread("FAIL", "SHORT_CLEAR_PROOF_MISMATCH"); return false; }
            if (retreatShort && (m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles))
            { EndEDMZFixtureSameThread("FAIL", "RETREAT_ARM_COUNT_MISMATCH"); return false; }
        }
        if (retreatShort && m_edmZFixture.beginPending && m_edmZFixture.armNextPhase != EDMZFixturePhase::Positive)
        { EndEDMZFixtureSameThread("FAIL", "RETREAT_INITIAL_PHASE_INVALID"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "ARM_HEARTBEAT_SUBMIT_FAILED"); return false; }
        if (!m_edmZFixture.originPinned)
        { m_edmZFixture.originPulse = feedback.outerOriginPulse; m_edmZFixture.originPinned = true; }
        m_edmZFixture.pending = false; m_edmZFixture.phase = m_edmZFixture.armNextPhase;
        if (m_edmZFixture.beginPending)
        { m_edmZFixture.windowStartMs = now; m_edmZFixture.beginPending = false; LogEDMZFixtureSameThread("BEGIN", "RUNNING", "LOCAL_Z_SIM_GAP_10_TO_30SEC"); }
        else
        {
            if (m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat) ++m_edmZFixture.verifiedShortClears;
            if (retreatShort) LogEDMZFixtureSameThread("RETREAT_ARM", "RUNNING", "STOP_PROVEN_SIM20_RETAINED");
            else LogEDMZFixtureSameThread("SHORT_CLEAR", "RUNNING", "REAL_STOP_PROVEN_NEGATIVE_RESUME");
        }
        return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::ShortStop || m_edmZFixture.phase == EDMZFixturePhase::FinalStop)
    {
        if (!IsEDMZFixtureStoppedSameThread()) return false;
        if (m_edmZFixture.phase == EDMZFixturePhase::ShortStop)
        {
            if (gapShortRetreat && !m_edmZFixture.gapShort.Snapshot().shortActive) return false;
            m_edmZFixture.shortDone = true; (void)RearmEDMZFixtureSameThread(now, false); return false;
        }
        if (std::fabs(feedback.actualPulse - m_edmZFixture.originPulse) > .001 * m_edmZFixture.config.pulsePerMm)
        { EndEDMZFixtureSameThread("FAIL", "FINAL_STOP_NOT_ORIGIN"); return false; }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "DISARM_SUBMIT_FAILED"); return false; }
        m_edmZFixture.phase = EDMZFixturePhase::Disarm; m_edmZFixture.phaseStartMs = now; return false;
    }
    if (m_edmZFixture.phase == EDMZFixturePhase::Disarm)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - m_edmZFixture.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "DISARM_APPLICATION_TIMEOUT"); return false; }
        if (feedback.capHeld || feedback.state != EDM28::State::Disarmed || !feedback.stopProven)
        { EndEDMZFixtureSameThread("FAIL", "DISARM_NOT_PROVEN"); return false; }
        if ((m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat || retreatShort) &&
            (m_edmZFixture.cycles < EDM28::ProfileMinimumCycles(m_edmZFixture.config.profile) ||
                !m_edmZFixture.shortDone || m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles ||
                (retreatShort && m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles) ||
                !m_edmZFixture.windowStartMs || now - m_edmZFixture.windowStartMs < 10000ULL))
        { EndEDMZFixtureSameThread("FAIL", "SHORT_FINAL_COUNT_MISMATCH"); return false; }
        m_edmZFixture.receiptPublication = feedback.publicationSequence; m_edmZFixture.receiptTick = feedback.sampledTick;
        m_edmZFixture.receiptProof = feedback.proofSequence; m_edmZFixture.phaseStartMs = now;
        m_edmZFixture.phase = EDMZFixturePhase::Receipt; m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        LogEDMZFixtureSameThread("SUMMARY", "PASS", "REAL_CYCLES_ORIGIN_STOP_DISARM"); return false;
    }
    if (gapShortRetreat && m_edmZFixture.phase == EDMZFixturePhase::ShortClearWait)
    {
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
        { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
        const auto& gap = m_edmZFixture.gapShort.Snapshot();
        if (gap.state == EDMGapServo::ShortState::Clear && !gap.feedInhibited && !gap.shortActive &&
            m_edmZFixture.gapClearHighUs && feedback.monotonicUs >= m_edmZFixture.gapClearHighUs &&
            feedback.monotonicUs - m_edmZFixture.gapClearHighUs >= 5000ULL)
        {
            ++m_edmZFixture.verifiedShortClears;
            m_edmZFixture.pending = false; m_edmZFixture.phase = EDMZFixturePhase::Negative;
            LogEDMZFixtureSameThread("SHORT_CLEAR", "RUNNING", "DETECTOR_CLEAR_RETREAT_HELD_5MS");
        }
        return false;
    }
    if (retreatShort)
    {
        if (m_edmZFixture.phase != EDMZFixturePhase::Positive && m_edmZFixture.phase != EDMZFixturePhase::Negative &&
            m_edmZFixture.phase != EDMZFixturePhase::Origin && m_edmZFixture.phase != EDMZFixturePhase::Retreat)
        { EndEDMZFixtureSameThread("FAIL", "RETREAT_PHASE_INVALID"); return false; }
        if (m_edmZFixture.phase == EDMZFixturePhase::Retreat &&
            (!m_edmZFixture.shortDone || !m_edmZFixture.shortStopProof || !m_edmZFixture.shortStopAppliedSequence ||
                !m_edmZFixture.shortStopAppliedTick || !m_edmZFixture.shortStopAppliedMonotonicUs ||
                m_edmZFixture.cycles == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles + 1U ||
                m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles || m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles ||
                feedback.stopAppliedSequence != m_edmZFixture.shortStopAppliedSequence ||
                feedback.stopAppliedTick != m_edmZFixture.shortStopAppliedTick ||
                feedback.stopAppliedMonotonicUs != m_edmZFixture.shortStopAppliedMonotonicUs))
        { EndEDMZFixtureSameThread("FAIL", "RETREAT_STOP_PROOF_MISMATCH"); return false; }
    }
    if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
    { EndEDMZFixtureSameThread("FAIL", "HEARTBEAT_SUBMIT_FAILED"); return false; }
    if (!m_edmZFixture.pending)
    {
        // A boundary receipt can be revoked by a later RT sample before
        // this next leg reaches the single command consumer. Keep earning
        // fresh proof; queue admission alone is not a native application.
        const bool ready = feedback.axisIdle &&
            std::fabs(feedback.cmdSpeedMmS) <= 1.0 / m_edmZFixture.config.pulsePerMm &&
            (feedback.state == EDM28::State::Armed ||
                (feedback.state == EDM28::State::AtTarget && feedback.targetProven &&
                    feedback.proofSequence != 0ULL && feedback.stableCycles >= m_edmZFixture.config.settleCycles));
        if (!ready)
        {
            if (now - m_edmZFixture.logMs >= 1000ULL)
                LogEDMZFixtureSameThread("PROGRESS", "WAIT", "AWAIT_REAL_POSITION_PROOF");
            return false;
        }
        if (retreatShort && m_edmZFixture.phase == EDMZFixturePhase::Retreat)
        {
            const double currentMax = (std::fmax)(m_edmZFixture.shortStopMaxPulse,
                (std::fmax)(feedback.actualPulse, (std::fmax)(feedback.commandPulse, feedback.planningPulse)));
            const double retreatTarget = m_edmZFixture.originPulse + .02 * m_edmZFixture.config.pulsePerMm;
            const double margin = 2.0 * .001 * m_edmZFixture.config.pulsePerMm + 1.0;
            if (!std::isfinite(m_edmZFixture.shortStopMaxPulse) || !std::isfinite(feedback.actualPulse) ||
                !std::isfinite(feedback.commandPulse) || !std::isfinite(feedback.planningPulse) || !std::isfinite(retreatTarget) ||
                feedback.proofSequence != m_edmZFixture.shortStopProof || !(retreatTarget - currentMax > margin))
            { EndEDMZFixtureSameThread("FAIL", "RETREAT_DIRECTION_NOT_PROVEN"); return false; }
        }
        const double target = (m_edmZFixture.phase == EDMZFixturePhase::Positive || m_edmZFixture.phase == EDMZFixturePhase::Retreat) ? .02 :
            m_edmZFixture.phase == EDMZFixturePhase::Negative ? -.02 : 0.0;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Position, target))
        { EndEDMZFixtureSameThread("FAIL", "POSITION_SUBMIT_FAILED"); return false; }
        m_edmZFixture.phaseStartMs = now; LogEDMZFixtureSameThread("LEG", "WAIT", "FINITE_POSITION_SUBMITTED"); return false;
    }
    if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Position))
    {
        // Request high-water is published after the FIFO consumer decision.
        // It can advance on a Heartbeat without applying Position. With the
        // exact scope still current and no Stop/fault, an older application
        // stamp proves this Position was consumed without a native commit.
        // Retire it, then wait for fresh proof and a fresh request sequence.
        if (m_edmZFixture.pendingKind == EDM28::RequestKind::Position &&
            EDM28::SameScope(feedback.scope, m_edmZFixture.scope) &&
            !feedback.stopLatched && !feedback.latchedFault && !feedback.forceZeroOutput &&
            feedback.lastRequestSequence >= m_edmZFixture.pendingSequence &&
            feedback.lastAppliedSequence < m_edmZFixture.pendingSequence)
        {
            m_edmZFixture.pending = false;
            LogEDMZFixtureSameThread("PROGRESS", "WAIT", "POSITION_CONSUMED_RETRY_PROOF");
            return false;
        }
        if (now - m_edmZFixture.phaseStartMs >= 50ULL)
            EndEDMZFixtureSameThread("FAIL", "POSITION_APPLICATION_TIMEOUT");
        return false;
    }
    m_edmZFixture.motionApplied = true; m_edmZFixture.appliedTick = feedback.lastAppliedTick;
    if (!gapShortRetreat && m_edmZFixture.phase == EDMZFixturePhase::Negative && !m_edmZFixture.shortDone &&
        feedback.sampledTick - feedback.lastAppliedTick >= 800ULL)
    {
        if (feedback.state != EDM28::State::Moving || feedback.targetProven || feedback.axisIdle)
        { EndEDMZFixtureSameThread("FAIL", "SHORT_TEST_REQUIRES_MOVING_LEG"); return false; }
        if (m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat || retreatShort)
        {
            if (m_edmZFixture.simulatedShortStops == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles ||
                (retreatShort && m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles))
            { EndEDMZFixtureSameThread("FAIL", "SHORT_INJECTION_COUNT_MISMATCH"); return false; }
            ++m_edmZFixture.simulatedShortStops;
        }
        m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session);
        m_edmZFixture.phase = EDMZFixturePhase::ShortStop; m_edmZFixture.pending = false;
        m_edmZFixture.phaseStartMs = now; LogEDMZFixtureSameThread("SHORT_STOP", "WAIT", "SIM20_DURING_REAL_NEGATIVE_LEG"); return false;
    }
    if (feedback.state == EDM28::State::AtTarget && feedback.targetProven && feedback.proofSequence &&
        feedback.sampledTick > feedback.lastAppliedTick && feedback.stableCycles >= m_edmZFixture.config.settleCycles && feedback.axisIdle)
    {
        const double expectedTarget = m_edmZFixture.originPulse +
            ((m_edmZFixture.phase == EDMZFixturePhase::Positive || m_edmZFixture.phase == EDMZFixturePhase::Retreat) ? .02 :
             m_edmZFixture.phase == EDMZFixturePhase::Negative ? -.02 : 0.0) * m_edmZFixture.config.pulsePerMm;
        const double tolerance = .001 * m_edmZFixture.config.pulsePerMm;
        if (retreatShort && (!std::isfinite(feedback.targetPulse) || !std::isfinite(expectedTarget) || !std::isfinite(tolerance)))
        { EndEDMZFixtureSameThread("FAIL", "RETREAT_ENDPOINT_NOT_FINITE"); return false; }
        if (std::fabs(feedback.targetPulse - expectedTarget) > tolerance ||
            std::fabs(feedback.actualPulse - expectedTarget) > tolerance ||
            std::fabs(feedback.commandPulse - expectedTarget) > tolerance ||
            std::fabs(feedback.planningPulse - expectedTarget) > tolerance)
        { EndEDMZFixtureSameThread("FAIL", "POSITION_PROOF_WRONG_ENDPOINT"); return false; }
        if (retreatShort && m_edmZFixture.phase == EDMZFixturePhase::Retreat)
        {
            // Re-arm only permits this finite retreat. A new exact Position
            // receipt and its complete RT target proof earn clear together.
            if (!std::isfinite(m_edmZFixture.shortStopMaxPulse) ||
                !(feedback.actualPulse > m_edmZFixture.shortStopMaxPulse + tolerance) ||
                feedback.proofSequence <= m_edmZFixture.shortStopProof ||
                feedback.lastAppliedTick <= m_edmZFixture.shortStopProofTick ||
                feedback.lastAppliedMonotonicUs <= m_edmZFixture.shortStopAppliedMonotonicUs ||
                feedback.sampledTick - feedback.lastAppliedTick < m_edmZFixture.config.settleCycles ||
                feedback.monotonicUs < feedback.lastAppliedMonotonicUs ||
                feedback.monotonicUs - feedback.lastAppliedMonotonicUs < m_edmZFixture.config.settleCycles * EDM28::CycleUs ||
                m_edmZFixture.verifiedRetreats == (std::numeric_limits<std::uint32_t>::max)() ||
                m_edmZFixture.verifiedShortClears == (std::numeric_limits<std::uint32_t>::max)())
            { EndEDMZFixtureSameThread("FAIL", "RETREAT_TARGET_NOT_PROVEN"); return false; }
            if (gapShortRetreat)
            {
                // This sample still carries SIM20. Save the exact earned
                // receipt; only a newer sample may begin recovery dwell.
                ++m_edmZFixture.verifiedRetreats;
                m_edmZFixture.gapClearProof = feedback.proofSequence;
                m_edmZFixture.gapClearAppliedSequence = feedback.lastAppliedSequence;
                m_edmZFixture.gapClearAppliedTick = feedback.lastAppliedTick;
                m_edmZFixture.gapClearAppliedUs = feedback.lastAppliedMonotonicUs;
                m_edmZFixture.gapClearPublication = feedback.publicationSequence;
                m_edmZFixture.gapClearTick = feedback.sampledTick; m_edmZFixture.gapClearUs = feedback.monotonicUs;
                m_edmZFixture.gapClearHighUs = 0ULL; m_edmZFixture.gapClearTargetPulse = feedback.targetPulse;
                m_edmZFixture.pending = false; m_edmZFixture.phase = EDMZFixturePhase::ShortClearWait;
                m_edmZFixture.phaseStartMs = now;
                LogEDMZFixtureSameThread("CLEAR_WAIT", "WAIT", "RETREAT_PROVEN_DETECTOR_DWELL_NEXT"); return false;
            }
            ++m_edmZFixture.verifiedRetreats; ++m_edmZFixture.verifiedShortClears;
            m_edmZFixture.pending = false; m_edmZFixture.phase = EDMZFixturePhase::Negative;
            LogEDMZFixtureSameThread("SHORT_CLEAR", "RUNNING", "RETREAT_PROVEN_SIM50_NEGATIVE_NEXT");
            return false;
        }
        m_edmZFixture.pending = false;
        if (m_edmZFixture.phase == EDMZFixturePhase::Positive) m_edmZFixture.phase = EDMZFixturePhase::Negative;
        else if (m_edmZFixture.phase == EDMZFixturePhase::Negative) m_edmZFixture.phase = EDMZFixturePhase::Origin;
        else
        {
            const bool repeatedShort = m_edmZFixture.config.profile == EDM28::Profile::P15UnloadedRepeat || retreatShort;
            if (repeatedShort && (m_edmZFixture.cycles == (std::numeric_limits<std::uint32_t>::max)() ||
                !m_edmZFixture.shortDone || m_edmZFixture.simulatedShortStops != m_edmZFixture.cycles + 1U ||
                m_edmZFixture.verifiedShortClears != m_edmZFixture.cycles + 1U ||
                (retreatShort && m_edmZFixture.verifiedRetreats != m_edmZFixture.cycles + 1U)))
            { EndEDMZFixtureSameThread("FAIL", "SHORT_CYCLE_COUNT_MISMATCH"); return false; }
            ++m_edmZFixture.cycles;
            if (m_edmZFixture.cycles >= EDM28::ProfileMinimumCycles(m_edmZFixture.config.profile) && m_edmZFixture.shortDone && now - m_edmZFixture.windowStartMs >= 10000ULL)
            { m_motion.RequestEDMZFixtureStop(m_edmZFixture.scope.session); m_edmZFixture.phase = EDMZFixturePhase::FinalStop; }
            else
            {
                // Only a completed Origin retires this cycle's injection.
                // Clearing this flag at short re-arm would retrigger on the
                // same resumed Negative leg instead of the next full cycle.
                if (repeatedShort)
                {
                    m_edmZFixture.shortDone = false;
                    m_edmZFixture.shortStopMaxPulse = 0.0;
                    m_edmZFixture.shortStopProof = m_edmZFixture.shortStopProofTick = 0ULL;
                    m_edmZFixture.shortStopAppliedSequence = m_edmZFixture.shortStopAppliedTick = m_edmZFixture.shortStopAppliedMonotonicUs = 0ULL;
                }
                m_edmZFixture.phase = EDMZFixturePhase::Positive;
            }
        }
        LogEDMZFixtureSameThread("BOUNDARY", "RUNNING", "POST_APPLY_200_RT_REAL_POSITION"); return false;
    }
    if (now - m_edmZFixture.logMs >= 1000ULL) LogEDMZFixtureSameThread("PROGRESS", "RUNNING", "AWAIT_REAL_POSITION_PROOF");
    return false;
}

// P10 owns isolated arbitration fixtures, including SHORT versus flush return.
// It cannot install synthetic samples, edit COND, or dispatch Motion/IO writes.
