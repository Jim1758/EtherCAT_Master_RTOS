// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "NCManager_GapInternal.h"

using namespace NCGapDetail;

bool NCManager::InstallEDMToolHeadCatalogBeforeStart(const EDM87::Catalog& catalog) noexcept
{
    if (m_edmToolHeadCatalogAttempted) return false;
    m_edmToolHeadCatalogAttempted = true;
    if (m_state != NCState::IDLE || m_gapDryRun.active || m_gapDryRunSerial != 0ULL ||
        m_pathCoreLiveBookkeeping.currentRunToken != 0ULL ||
        m_edmCoordinatedRT.submitted || m_edmCoordinatedRT.retirePending ||
        !EDM87::ValidCatalog(catalog)) return false;
    m_edmToolHeadCatalog = catalog;
    m_edmToolHeadCatalogReady = true;
    return true;
}

// EDM84 requires every inspected source line to be clean BEFORE an empty
// line can be skipped. A malformed empty comment or block skip is a rejection.
bool NCManager::IsEDMCoordinatedRTM30ContinuationSameThread() const noexcept
{
    const int pc = GetActiveDispatchPC();
    if (pc < 0 || pc > (std::numeric_limits<int>::max)() - 33) return false;
    for (int offset = 1; offset <= 32; ++offset)
    {
        const NCProgramCacheLine* line = GetBaseProgramCache().TryGetLine(pc + offset);
        if (!line || !EDM77::Detail::Clean(line->parsedBlock)) return false;
        if (EDM77::Detail::Empty(line->parsedBlock)) continue;
        return EDM77::Detail::Terminal(line->parsedBlock);
    }
    return false;
}

// EDM86: bounded NC-only records. Keep formatting frames outside admission
// and completion; no console work is performed in the real-time callback.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static bool LogEDM86ToolHeadConfiguration(const EDM84RT::Request& request) noexcept
{
    if (request.scenario != EDM84RT::Scenario::ToolHeadCUVCompensation) return true;
    const char* tag = EDM84RT::DiagnosticTag(request);
    const auto& selection = request.toolHeadSelection;
    if (!EDM87::IsEmptySelection(selection))
        RtPrintf("[%s] event=PROFILE_FROZEN session=%llu id=%u profileRevision=%llu catalogRevision=%llu fingerprint=%llu source=BOOT_FILE calibration=REFERENCE_ONLY livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(request.session), selection.profile.id,
            static_cast<unsigned long long>(selection.profile.revision),
            static_cast<unsigned long long>(selection.catalogRevision),
            static_cast<unsigned long long>(selection.fingerprint));
    const auto& mapping = request.config.mapping;
    const auto& direction = request.config.direction.unit;
    char record[640]{};
    int count = std::snprintf(record, sizeof(record),
        "[%s] event=KINEMATICS_CONFIG session=%llu model=SERIAL_HEAD_CUV calibration=REFERENCE_ONLY transform=PARENT_TRANSLATION_THEN_ROTATION theta=SIGN_TIMES_Q_PLUS_ZERO tipMm=%.9g,%.9g,%.9g localDirection=%.9g,%.9g,%.9g frameRev=%llu kinematicsRev=%llu livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(request.session),
        mapping.toolOffsetMm[0], mapping.toolOffsetMm[1], mapping.toolOffsetMm[2],
        direction[0], direction[1], direction[2],
        static_cast<unsigned long long>(request.config.direction.revision),
        static_cast<unsigned long long>(mapping.revision));
    if (count < 0 || static_cast<std::size_t>(count) >= sizeof(record)) return false;
    RtPrintf("%s", record);
    for (unsigned order = 0U; order < 3U; ++order)
    {
        const auto& joint = mapping.chain.joints[order];
        if (joint.channel < 3U || joint.channel > 5U) return false;
        const char channel = joint.channel == 3U ? 'C' : joint.channel == 4U ? 'U' : 'V';
        count = std::snprintf(record, sizeof(record),
            "[%s] event=KINEMATICS_JOINT session=%llu order=%u channel=%c axisParent=%.9g,%.9g,%.9g parentToPivotMm=%.9g,%.9g,%.9g sign=%d zeroDeg=%.9g output=NONE\n",
            tag, static_cast<unsigned long long>(request.session), order + 1U, channel,
            joint.axis[0], joint.axis[1], joint.axis[2],
            joint.parentToPivotMm[0], joint.parentToPivotMm[1], joint.parentToPivotMm[2],
            joint.sign, joint.zeroAngleDeg);
        if (count < 0 || static_cast<std::size_t>(count) >= sizeof(record)) return false;
        RtPrintf("%s", record);
    }
    return true;
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM86ToolHeadCompletion(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback) noexcept
{
    if (request.scenario != EDM84RT::Scenario::ToolHeadCUVCompensation) return;
    const char* tag = EDM84RT::DiagnosticTag(request);
    if (!EDM87::IsEmptySelection(request.toolHeadSelection))
        RtPrintf("[%s] event=PROFILE_EVIDENCE result=PASS session=%llu id=%u profileRevision=%llu catalogRevision=%llu fingerprint=%llu immutable=1 machineCalibrated=0 livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(request.session), feedback.toolHeadProfileId,
            static_cast<unsigned long long>(feedback.toolHeadProfileRevision),
            static_cast<unsigned long long>(feedback.toolHeadCatalogRevision),
            static_cast<unsigned long long>(feedback.toolHeadFingerprint));
    const auto& resolved = feedback.resolvedDirectionUnit;
    const auto& frozen = feedback.sample.frozenDirection;
    char record[640]{};
    const int count = std::snprintf(record, sizeof(record),
        "[%s] event=KINEMATICS_EVIDENCE result=PASS session=%llu model=SERIAL_HEAD_CUV orientationSamples=%llu runtimeSteps=%llu resolvedDirection=%.12g,%.12g,%.12g lastFrozenDirection=%.12g,%.12g,%.12g maxOrientationNormError=%.12g maxFrozenDirectionError=%.12g machineCalibrated=0 livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        tag, static_cast<unsigned long long>(request.session),
        static_cast<unsigned long long>(feedback.orientationSamples),
        static_cast<unsigned long long>(feedback.runtimeSteps),
        resolved[0], resolved[1], resolved[2], frozen[0], frozen[1], frozen[2],
        feedback.maxOrientationNormError, feedback.maxFrozenDirectionError);
    if (count > 0 && static_cast<std::size_t>(count) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[%s] event=KINEMATICS_EVIDENCE result=LOG_FORMAT_ERROR output=NONE\n", tag);
}

// EDM89: source-time decisions are logged only by the NC owner. Holding a
// conversion does not advance short-entry/exit evidence on the control clock.
static void LogEDM89GapDecisionConfiguration(const EDM84RT::Request& request) noexcept
{
    if (!EDM84RT::IsSampled(request.profile)) return;
    const auto c = EDM89::FrozenConfig(request.config.motion.servo);
    char record[480]{};
    const int count = std::snprintf(record, sizeof(record),
        "[%s] event=GAP_DECISION_CONFIG session=%llu policy=SOURCE_CAPTURE_US held=REUSE_ONLY enabled=%u revision=%u thresholdV=%.12g hysteresisV=%.12g enterUs=%llu exitUs=%llu pulseProbe=%u physicalAD=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(request), static_cast<unsigned long long>(request.session),
        c.enabled ? 1U : 0U, static_cast<unsigned>(c.revision), c.thresholdV, c.hysteresisV,
        static_cast<unsigned long long>(c.enterUs), static_cast<unsigned long long>(c.exitUs),
        EDM84RT::IsSourceDwell(request.profile) ? 1U : 0U);
    if (count > 0 && static_cast<std::size_t>(count) < sizeof(record)) RtPrintf("%s", record);
    else RtPrintf("[%s] event=GAP_DECISION_CONFIG result=LOG_FORMAT_ERROR output=NONE\n",
        EDM84RT::DiagnosticTag(request));
}

static void LogEDM89GapDwellEvidence(const EDM84RT::Request& request,
    const char* result, const char* kind, const EDM89::Mark& start,
    const EDM89::Mark& confirm, std::uint64_t requiredUs) noexcept
{
    const bool confirmed = confirm.sequence != 0ULL && start.sequence != 0ULL &&
        confirm.capturedAtUs >= start.capturedAtUs;
    const std::uint64_t elapsedUs = confirmed ? confirm.capturedAtUs - start.capturedAtUs : 0ULL;
    RtPrintf("[%s] event=GAP_DWELL_EVIDENCE result=%s session=%llu kind=%s confirmed=%u startSeq=%llu confirmSeq=%llu startUs=%llu confirmUs=%llu requiredUs=%llu elapsedUs=%llu clock=SOURCE_CAPTURE_US physicalAD=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(request), result, static_cast<unsigned long long>(request.session), kind,
        confirmed ? 1U : 0U, static_cast<unsigned long long>(start.sequence),
        static_cast<unsigned long long>(confirm.sequence), static_cast<unsigned long long>(start.capturedAtUs),
        static_cast<unsigned long long>(confirm.capturedAtUs), static_cast<unsigned long long>(requiredUs),
        static_cast<unsigned long long>(elapsedUs));
}

static void LogEDM89GapDecisionEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsSampled(request.profile)) return;
    const auto& e = feedback.sampledDecision;
    RtPrintf("[%s] event=GAP_DECISION_EVIDENCE result=%s session=%llu reason=%s faulted=%u state=%s processed=%llu held=%llu rejected=%llu enterCount=%llu exitCount=%llu entryAborts=%llu exitRestarts=%llu inhibit=%u shortActive=%u physicalAD=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(request), result, static_cast<unsigned long long>(request.session),
        EDM89::ReasonName(e.reason), e.faulted ? 1U : 0U, EDM89::StateName(e.state),
        static_cast<unsigned long long>(e.processedFrames), static_cast<unsigned long long>(e.heldReads),
        static_cast<unsigned long long>(e.rejectedFrames), static_cast<unsigned long long>(e.enterCount),
        static_cast<unsigned long long>(e.exitCount), static_cast<unsigned long long>(e.entryAborts),
        static_cast<unsigned long long>(e.exitRestarts), e.feedInhibited ? 1U : 0U, e.shortActive ? 1U : 0U);
    LogEDM89GapDwellEvidence(request, result, "ENTER", e.enterStart, e.enterConfirm, e.config.enterUs);
    LogEDM89GapDwellEvidence(request, result, "EXIT", e.exitStart, e.exitConfirm, e.config.exitUs);
}

// EDM88: all console evidence remains on the NC owner. Source acquisition
// timestamps are synthetic-original timestamps, not the QPC callback clock.
static void LogEDM88SampledSourceConfiguration(const EDM84RT::Request& request) noexcept
{
    if (!EDM84RT::IsSampled(request.profile)) return;
    const auto& c = request.config.gapSource;
    const auto& id = c.identity;
    RtPrintf("[%s] event=GAP_SOURCE_CONFIG session=%llu source=SIMULATED_SAMPLE_HOLD samplePeriodUs=%llu maxAgeUs=%llu faultMode=%u producer=%llu device=%u channel=%u configRev=%llu calibrationRev=%llu physicalAD=0 sourceClock=MODEL_ACQUISITION controlStepUs=250 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(request), static_cast<unsigned long long>(request.session), static_cast<unsigned long long>(c.samplePeriodUs),
        static_cast<unsigned long long>(c.maxAgeUs), static_cast<unsigned>(request.sampledFault),
        static_cast<unsigned long long>(id.producerGeneration), static_cast<unsigned>(id.deviceId),
        static_cast<unsigned>(id.channelId), static_cast<unsigned long long>(id.configRevision),
        static_cast<unsigned long long>(id.calibrationRevision));
    LogEDM89GapDecisionConfiguration(request);
}

static void LogEDM88SampledSourceEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsSampled(request.profile)) return;
    const auto& evidence = feedback.sampledSource;
    RtPrintf("[%s] event=GAP_SOURCE_EVIDENCE result=%s session=%llu faultMode=%u reason=%s faulted=%u controlSteps=%llu freshFrames=%llu heldFrames=%llu rejectedFrames=%llu maximumAgeUs=%llu lastSequence=%llu capturedAtUs=%llu physicalAD=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(request), result, static_cast<unsigned long long>(request.session), static_cast<unsigned>(feedback.sampledFault),
        EDM88::ReasonName(evidence.reason), evidence.faulted ? 1U : 0U,
        static_cast<unsigned long long>(feedback.runtimeSteps), static_cast<unsigned long long>(evidence.freshFrames),
        static_cast<unsigned long long>(evidence.heldFrames), static_cast<unsigned long long>(evidence.rejectedFrames),
        static_cast<unsigned long long>(evidence.maximumAgeUs), static_cast<unsigned long long>(evidence.lastFrame.sequence),
        static_cast<unsigned long long>(evidence.lastFrame.capturedAtUs));
    LogEDM89GapDecisionEvidence(request, feedback, result);
}

// EDM90 formats only NC-owned copies of the RT preview receipt. These records
// describe candidate values; no formatter performs axis or PDO output.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM90OutputPreviewConfiguration(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback) noexcept
{
    if (!EDM84RT::IsOutputPreview(request.profile)) return;
    const auto& config = feedback.previewConfig;
    const auto& machine = config.machine;
    RtPrintf("[EDM90] event=OUTPUT_PREVIEW_CONFIG session=%llu captureTick=%llu captureUs=%llu mapGeneration=%llu axisCount=%u mode=LIVE_NATIVE_PULSE_PREVIEW anchor=CURRENT_COMMAND delta=COMPOSED_FROM_INITIAL periodUs=%llu compensation=UNSUPPORTED actualOutput=NONE motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(config.session), static_cast<unsigned long long>(machine.tick),
        static_cast<unsigned long long>(machine.nowUs), static_cast<unsigned long long>(machine.mapGeneration), machine.axisCount,
        static_cast<unsigned long long>(config.periodUs));
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& a = machine.axes[axis];
        const char name = "XYZCUV"[axis];
        char text[512]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM90] event=OUTPUT_PREVIEW_AXIS session=%llu axis=%c id=%u units=%s pulsesPerUnit=%.12g anchorPulse=%.12g actualPulse=%.12g hardwareSign=%d referenceOffsetPulse=%.12g initialPose=%.12g velocityCapUnitsMin=%.12g travelCapUnits=%.12g output=NONE\n",
            static_cast<unsigned long long>(request.session), name, a.axisId, axis < 3U ? "mm" : "deg",
            a.pulsesPerUnit, a.anchorNativePulse, a.actualNativePulse, a.hardwareSign,
            a.referenceOffsetPulse, config.initialPose[axis], a.maxVelocityUnitsPerMin, EDM90::MaximumTravelUnits(axis));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM90] event=OUTPUT_PREVIEW_AXIS result=LOG_FORMAT_ERROR axis=%c output=NONE\n", name);
        const int limitLength = std::snprintf(text, sizeof(text), "[EDM90] event=OUTPUT_PREVIEW_LIMIT session=%llu axis=%c minEnabled=%u minPulse=%.12g maxEnabled=%u maxPulse=%.12g configGeneration=%llu noCompensation=%u diagnosticVelocityCapUnitsSec=%.12g output=NONE\n",
            static_cast<unsigned long long>(request.session), name, a.hasMinLimit ? 1U : 0U, a.minNativePulse,
            a.hasMaxLimit ? 1U : 0U, a.maxNativePulse, static_cast<unsigned long long>(a.configGeneration),
            a.noCompensation ? 1U : 0U, EDM90::MaximumVelocityUnitsPerSecond(axis));
        if (limitLength > 0 && static_cast<std::size_t>(limitLength) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM90] event=OUTPUT_PREVIEW_LIMIT result=LOG_FORMAT_ERROR axis=%c output=NONE\n", name);
    }
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM90OutputPreviewEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsOutputPreview(request.profile)) return;
    const auto& e = feedback.previewEvidence;
    const char* kernelState = feedback.sample.state == EDM81::State::ShortRetreat ? "SHORT_RETREAT" :
        feedback.sample.state == EDM81::State::JumpRetreat ? "JUMP_RETREAT" :
        feedback.sample.state == EDM81::State::Returning ? "RETURNING" :
        feedback.sample.state == EDM81::State::Running ? "RUNNING" : "TERMINAL";
    char text[512]{};
    const int length = std::snprintf(text, sizeof(text), "[EDM90] event=OUTPUT_PREVIEW_EVIDENCE result=%s session=%llu state=%u reason=%s machineRejectMask=%llu frames=%llu captureTick=%llu lastTick=%llu sequence=%llu axisMask=%u rejectedAxis=%u candidateValid=%u kernelState=%s offsetMm=%.12g actualOutput=NONE motion=0 physicalPermit=0 discharge=0 physicalStopProven=0\n",
        result, static_cast<unsigned long long>(request.session), static_cast<unsigned>(e.state), EDM90::ReasonName(e.reason),
        static_cast<unsigned long long>(feedback.previewMachineRejectMask),
        static_cast<unsigned long long>(e.processedFrames), static_cast<unsigned long long>(e.captureTick),
        static_cast<unsigned long long>(e.lastTick), static_cast<unsigned long long>(e.lastSequence), e.axisMovedMask,
        e.rejectedAxis, e.candidateValid ? 1U : 0U, kernelState, feedback.sample.offsetMm);
    if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM90] event=OUTPUT_PREVIEW_EVIDENCE result=LOG_FORMAT_ERROR output=NONE\n");
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const int targetLength = std::snprintf(text, sizeof(text), "[EDM90] event=OUTPUT_PREVIEW_TARGET result=%s session=%llu axis=%c targetPulse=%.12g velocityPps=%.12g maxVelocityPps=%.12g maxAccelerationPps2=%.12g maxTravelUnits=%.12g accelerationValidated=0 actualOutput=NONE\n",
            result, static_cast<unsigned long long>(request.session), "XYZCUV"[axis], e.nativeTarget[axis],
            e.nativeVelocityPps[axis], e.maximumVelocityPps[axis], e.maximumAccelerationPps2[axis], e.maximumTravelUnits[axis]);
        if (targetLength > 0 && static_cast<std::size_t>(targetLength) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM90] event=OUTPUT_PREVIEW_TARGET result=LOG_FORMAT_ERROR output=NONE\n");
    }
}

// EDM91 formats frozen numerical trajectory receipts only on the NC thread.
// Joint-coordinate filtering changes the path and is not calibrated TCP output.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM91TrajectoryConfiguration(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback) noexcept
{
    if (!EDM84RT::IsTrajectoryPreview(request.profile)) return;
    const auto& config = feedback.previewConfig;
    RtPrintf("[EDM91] event=TRAJECTORY_CONFIG session=%llu filter=JOINT_POSITION_FIR clock=MODEL_250US taps=64 periodUs=%llu history=INITIAL_POSE delayUs=7875 maximumDrainUs=16000 maximumStopUs=8000 maximumStopTicks=32 stop=COMMON_VELOCITY_RAMP limitSpace=JOINT pathPreserved=0 tcpValidated=0 accelerationValidation=NUMERIC_ONLY jerkValidated=0 actualOutput=NONE physicalStop=0 discharge=0\n",
        static_cast<unsigned long long>(request.session), static_cast<unsigned long long>(config.periodUs));
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        char text[448]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM91] event=TRAJECTORY_AXIS_CONFIG session=%llu axis=%c units=%s inputVelocityCap=%.12g accelerationCap=%.12g stopReserveUnits=%.12g actualOutput=NONE physicalStop=0\n",
            static_cast<unsigned long long>(request.session), "XYZCUV"[axis], axis < 3U ? "mm" : "deg",
            EDM91::VelocityLimitUnitsPerSecond(config, axis), EDM91::AccelerationLimitUnitsPerSecond2(config, axis),
            EDM91::VelocityLimitUnitsPerSecond(config, axis) * 0.004);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM91] event=TRAJECTORY_AXIS_CONFIG result=LOG_FORMAT_ERROR actualOutput=NONE\n");
    }
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM91TrajectoryEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsTrajectoryPreview(request.profile)) return;
    const auto& e = feedback.trajectoryEvidence;
    RtPrintf("[EDM91] event=TRAJECTORY_EVIDENCE result=%s phase=RT_ACKNOWLEDGED clock=MODEL_250US session=%llu state=%s reason=%s rawFrames=%llu shapeSteps=%llu tailSteps=%llu drainSteps=%llu stopSteps=%llu stopTicks=%llu lastTick=%llu candidateValid=%u numericalStopped=%u rejectedAxis=%u accelerationValidation=NUMERIC_ONLY actualOutput=NONE motion=0 physicalPermit=0 physicalStop=0 discharge=0\n",
        result, static_cast<unsigned long long>(request.session), EDM91::StateName(e.state), EDM91::ReasonName(e.reason),
        static_cast<unsigned long long>(e.rawFrames), static_cast<unsigned long long>(e.shapeSteps),
        static_cast<unsigned long long>(e.tailSteps), static_cast<unsigned long long>(e.drainSteps),
        static_cast<unsigned long long>(e.stopSteps), static_cast<unsigned long long>(e.stopTicks),
        static_cast<unsigned long long>(e.lastTick), e.candidateValid ? 1U : 0U, e.numericalStopped ? 1U : 0U, e.rejectedAxis);
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        char text[608]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM91] event=TRAJECTORY_AXIS result=%s session=%llu axis=%c rawPose=%.12g shapedPose=%.12g targetPulse=%.12g velocityUnitsSec=%.12g maxVelocityUnitsSec=%.12g maxAccelerationUnitsSec2=%.12g maxTravelUnits=%.12g maxTrackingErrorUnits=%.12g terminalPose=%.12g numericalStopped=%u actualOutput=NONE physicalStop=0\n",
            result, static_cast<unsigned long long>(request.session), "XYZCUV"[axis], e.sourcePose[axis], e.shapedPose[axis], e.nativeTarget[axis],
            e.velocityUnitsPerSecond[axis], e.maximumVelocityUnitsPerSecond[axis], e.maximumAccelerationUnitsPerSecond2[axis],
            e.maximumTravelUnits[axis], e.maximumTrackingErrorUnits[axis], e.shapedPose[axis], e.numericalStopped ? 1U : 0U);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM91] event=TRAJECTORY_AXIS result=LOG_FORMAT_ERROR actualOutput=NONE\n");
    }
}

// EDM92 validates logical six-axis pre-PID batches in a private RT shadow.
// These accepted values never enter the ordinary axis or PDO writers.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM92CommandBatchConfiguration(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback) noexcept
{
    if (!EDM84RT::IsCommandBatch(request.profile)) return;
    RtPrintf("[EDM92] event=COMMAND_BATCH_CONFIG session=%llu mode=PRE_PID_SEMANTIC_SHADOW axes=XYZCUV batchSize=6 anchor=CURRENT_COMMAND coordinates=LOGICAL_NATIVE targetUnits=pulse velocityUnits=pulse_per_sec hardwareSignApplied=0 atomicity=PRIVATE_ACCEPTANCE_ONLY actualOutput=NONE physicalPermit=0 physicalStop=0 discharge=0\n",
        static_cast<unsigned long long>(request.session));
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& a = feedback.previewConfig.machine.axes[axis];
        char text[384]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM92] event=COMMAND_BATCH_AXIS_CONFIG session=%llu axis=%c id=%u anchorPulse=%.12g pulsesPerUnit=%.12g hardwareSign=%d signUse=OBSERVER_ONLY actualOutput=NONE\n",
            static_cast<unsigned long long>(request.session), "XYZCUV"[axis], a.axisId,
            a.anchorNativePulse, a.pulsesPerUnit, a.hardwareSign);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM92] event=COMMAND_BATCH_AXIS_CONFIG result=LOG_FORMAT_ERROR actualOutput=NONE\n");
    }
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM92CommandBatchEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsCommandBatch(request.profile)) return;
    const auto& e = feedback.batchEvidence;
    const auto& t = e.trajectory;
    const char* cancelPhase = feedback.batchCancelPhase == EDM81::State::ShortRetreat ? "SHORT_RETREAT" :
        feedback.batchCancelPhase == EDM81::State::JumpRetreat ? "JUMP_RETREAT" :
        feedback.batchCancelPhase == EDM81::State::Returning ? "RETURNING" :
        feedback.batchCancelPhase == EDM81::State::Running ? "RUNNING" :
        feedback.batchCancelPhase == EDM81::State::Completed ? "COMPLETED" :
        feedback.batchCancelPhase == EDM81::State::Fault ? "FAULT" : "IDLE";
    RtPrintf("[EDM92] event=COMMAND_BATCH_EVIDENCE result=%s phase=RT_ACKNOWLEDGED session=%llu state=%s reason=%s kind=%s accepted=%llu raw=%llu drain=%llu stop=%llu lastTick=%llu candidateValid=%u numericalStopped=%u revoked=%u rejectedAxis=%u cancelTick=%llu cancelModelUs=%llu cancelPhase=%s cancelStage=%u actualOutput=NONE physicalPermit=0 physicalStop=0 discharge=0\n",
        result, static_cast<unsigned long long>(request.session), EDM92::StateName(e.state), EDM92::ReasonName(e.reason), EDM92::KindName(e.lastKind),
        static_cast<unsigned long long>(e.acceptedBatches), static_cast<unsigned long long>(e.rawBatches),
        static_cast<unsigned long long>(e.drainBatches), static_cast<unsigned long long>(e.stopBatches),
        static_cast<unsigned long long>(t.lastTick), e.candidateValid ? 1U : 0U,
        e.numericalStopped ? 1U : 0U, e.revoked ? 1U : 0U, e.rejectedAxis,
        static_cast<unsigned long long>(feedback.batchCancelTick),
        static_cast<unsigned long long>(feedback.batchCancelModelUs), cancelPhase, feedback.batchCancelStage);
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        char text[384]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM92] event=COMMAND_BATCH_AXIS result=%s session=%llu axis=%c targetPulse=%.12g velocityPps=%.12g positionUnits=%.12g velocityUnitsSec=%.12g numericalStopped=%u actualOutput=NONE physicalStop=0\n",
            result, static_cast<unsigned long long>(request.session), "XYZCUV"[axis],
            t.nativeTarget[axis], e.nativeVelocityPps[axis], t.shapedPose[axis],
            t.velocityUnitsPerSecond[axis], e.numericalStopped ? 1U : 0U);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM92] event=COMMAND_BATCH_AXIS result=LOG_FORMAT_ERROR actualOutput=NONE\n");
    }
}

// EDM93 formats private ideal-plant receipts only on the NC thread. The PI
// gains are captured by Motion from G00; this never changes machine tuning.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM93ServoSimulationConfiguration(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback) noexcept
{
    if (!EDM84RT::IsServoSimulation(request.profile)) return;
    RtPrintf("[EDM93] event=SERVO_SIM_CONFIG session=%llu plant=IDEAL_CSV_INTEGRATOR idealPlant=1 zeroLoad=1 noInertia=1 friction=0 transportDelay=0 seed=CURRENT_COMMAND feedback=PRIVATE_MODEL controller=SHARED_G00_PI derivative=DISABLED gains=FROZEN_G00 controllerFingerprint=%llu periodUs=%llu physicalOutput=NONE discharge=0\n",
        static_cast<unsigned long long>(request.session),
        static_cast<unsigned long long>(feedback.servoEvidence.controllerFingerprint),
        static_cast<unsigned long long>(feedback.previewConfig.periodUs));
    char settleText[384]{};
    const int settleLength = std::snprintf(settleText, sizeof(settleText),
        "[EDM93] event=SERVO_SIM_SETTLE_CONFIG session=%llu samples=%u tolerancePulse=%.12g maxTicks=%llu clock=MODEL_250US condition=ALL_SIX_ZERO_CSV_AND_ERROR_WINDOW simulatedStopOnly=1 physicalOutput=NONE physicalStop=0\n",
        static_cast<unsigned long long>(request.session), EDM93::SettleSamples,
        EDM93::SettleTolerancePulse, static_cast<unsigned long long>(EDM93::MaximumSettleTicks));
    if (settleLength > 0 && static_cast<std::size_t>(settleLength) < sizeof(settleText)) RtPrintf("%s", settleText);
    else RtPrintf("[EDM93] event=SERVO_SIM_SETTLE_CONFIG result=LOG_FORMAT_ERROR physicalOutput=NONE\n");
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        const auto& a = feedback.previewConfig.machine.axes[axis];
        char text[448]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM93] event=SERVO_SIM_AXIS_CONFIG session=%llu axis=%c id=%u anchorPulse=%.12g pulsesPerUnit=%.12g velocityCapPps=%.12g hardwareSign=%d signUse=PRIVATE_CSV_ONCE simulatedFeedback=IDEAL_INTEGRATOR physicalOutput=NONE\n",
            static_cast<unsigned long long>(request.session), "XYZCUV"[axis], a.axisId,
            a.anchorNativePulse, a.pulsesPerUnit,
            EDM93::VelocityCap(feedback.previewConfig, axis), a.hardwareSign);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM93] event=SERVO_SIM_AXIS_CONFIG result=LOG_FORMAT_ERROR physicalOutput=NONE\n");
    }
}

#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
static void LogEDM93ServoSimulationEvidence(const EDM84RT::Request& request,
    const EDM84RT::Feedback& feedback, const char* result) noexcept
{
    if (!EDM84RT::IsServoSimulation(request.profile)) return;
    const auto& e = feedback.servoEvidence;
    RtPrintf("[EDM93] event=SERVO_SIM_EVIDENCE result=%s phase=RT_ACKNOWLEDGED session=%llu state=%s reason=%s accepted=%llu modelTicks=%llu settleTicks=%llu lastTick=%llu settledSamples=%u numericalStopped=%u modelSettled=%u revoked=%u saturationMask=%u everSaturationMask=%u rejectedAxis=%u controllerFingerprint=%llu idealPlant=1 zeroLoad=1 noInertia=1 physicalOutput=NONE physicalStop=0 discharge=0\n",
        result, static_cast<unsigned long long>(request.session), EDM93::StateName(e.state), EDM93::ReasonName(e.reason),
        static_cast<unsigned long long>(e.acceptedBatches), static_cast<unsigned long long>(e.modelTicks),
        static_cast<unsigned long long>(e.settleTicks), static_cast<unsigned long long>(e.lastTick),
        e.settledSamples, e.modelSettled ? 1U : 0U, e.modelSettled ? 1U : 0U, e.revoked ? 1U : 0U,
        e.saturationMask, e.everSaturationMask, e.rejectedAxis,
        static_cast<unsigned long long>(e.controllerFingerprint));
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        char text[512]{};
        const double target = feedback.batchEvidence.trajectory.nativeTarget[axis];
        const int length = std::snprintf(text, sizeof(text),
            "[EDM93] event=SERVO_SIM_AXIS result=%s session=%llu axis=%c targetPulse=%.12g simulatedPulse=%.12g followingErrorPulse=%.12g signedCsvPps=%ld integralAcc=%.12g maxFollowingErrorPulse=%.12g maxLogicalVelocityPps=%.12g numericalStopped=%u physicalOutput=NONE physicalStop=0\n",
            result, static_cast<unsigned long long>(request.session), "XYZCUV"[axis], target,
            e.positionNativePulse[axis], target - e.positionNativePulse[axis], static_cast<long>(e.csv[axis]),
            e.integralAcc[axis], e.maximumFollowingErrorPulse[axis], e.maximumLogicalVelocityPps[axis],
            e.modelSettled ? 1U : 0U);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM93] event=SERVO_SIM_AXIS result=LOG_FORMAT_ERROR physicalOutput=NONE\n");
    }
}

// NC-thread snapshots only, never called by the cyclic RT callback. These
// observations identify startup boundaries; they are not stack high-water or
// callback execution-time proofs. Preserve the caller's OS last-error value.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
void NCManager::LogEDMCoordinatedRTStartupStageSameThread(const char* stage) noexcept
{
    const DWORD incomingError = GetLastError();
    DWORD stackBytes = 0U, availableBytes = 0U;
    ULONG_PTR stackBase = 0U, stackLimit = 0U, stackTop = 0U;
    const bool queried = RtGetThreadStack(GetCurrentThread(), &stackBytes, &availableBytes,
        &stackBase, &stackLimit, &stackTop);
    const DWORD queryError = queried ? 0U : GetLastError();
    const DWORD rawStackBytes = stackBytes, rawAvailableBytes = availableBytes;
    const bool valid = queried && stackBytes != 0U && availableBytes <= stackBytes;
    if (!valid) { stackBytes = 0U; availableBytes = 0U; }
    RtPrintf("[%s] event=STARTUP_STAGE build=EDM93_20261012 stage=%.16s session=%llu P=%u Q=%u stackQuery=%s stackBytes=%lu availableBytes=%lu error=%lu rawStackBytes=%lu rawAvailableBytes=%lu threadId=%lu base=%llu limit=%llu top=%llu topInRange=%u output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(m_edmCoordinatedRT.request), stage ? stage : "UNKNOWN", static_cast<unsigned long long>(m_edmCoordinatedRT.request.session),
        static_cast<unsigned>(m_edmCoordinatedRT.request.profile), static_cast<unsigned>(m_edmCoordinatedRT.request.scenario),
        valid ? "OK" : queried ? "INVALID" : "FAILED", static_cast<unsigned long>(stackBytes),
        static_cast<unsigned long>(availableBytes), static_cast<unsigned long>(queryError),
        static_cast<unsigned long>(rawStackBytes), static_cast<unsigned long>(rawAvailableBytes),
        static_cast<unsigned long>(GetCurrentThreadId()), static_cast<unsigned long long>(stackBase),
        static_cast<unsigned long long>(stackLimit), static_cast<unsigned long long>(stackTop),
        queried && NCMainStack::ContainsTop(stackBase, stackLimit, stackTop) ? 1U : 0U);
    SetLastError(incomingError);
}

// EDM84 supervises an explicit private retained-path calculator in Motion's RT
// callback. None of these methods publishes an axis target or discharge permit.
WaitConditionFunc NCManager::StartEDMCoordinatedRTSameThread(const NCBlock& block)
{
    const int line = m_gapDryRun.sourceLine;
    ServiceEDMCoordinatedRTRetirementSameThread();
    ServiceEDMCNCRetainedRTRetirementSameThread();
    ServiceEDMGapServoRetirementSameThread();
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_gapDryRun.active || m_edmCNCRetainedRT.retirePending ||
        (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired) || m_edmCoordinatedRT.retirePending ||
        (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired) ||
        m_edmZFixture.cleanupPending || m_edmZFixture.resetPending || m_edmZFixture.failureAlarmPending ||
        m_edmPhysicalServoRT.retirePending || m_edmGapServoRT.retirePending)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM84_PREVIOUS_SESSION_BUSY"); return nullptr; }
    const unsigned profile = block.has('P') && block.val('P') == 84.0 ? 84U :
        block.has('P') && block.val('P') == 83.0 ? 83U :
        block.has('P') && block.val('P') == 82.0 ? 82U :
        block.has('P') && block.val('P') == 81.0 ? 81U :
        block.has('P') && block.val('P') == 80.0 ? 80U :
        block.has('P') && block.val('P') == 79.0 ? 79U :
        block.has('P') && block.val('P') == 78.0 ? 78U :
        block.has('P') && block.val('P') == 77.0 ? 77U :
        block.has('P') && block.val('P') == 76.0 ? 76U :
        block.has('P') && block.val('P') == 75.0 ? 75U :
        block.has('P') && block.val('P') == 74.0 ? 74U :
        block.has('P') && block.val('P') == 73.0 ? 73U :
        block.has('P') && block.val('P') == 72.0 ? 72U :
        block.has('P') && block.val('P') == 71.0 ? 71U :
        block.has('P') && block.val('P') == 70.0 ? 70U : 69U;
    const bool ncAuthored = EDM84RT::IsAuthored(static_cast<EDM84RT::Profile>(profile));
    const bool sampled = EDM84RT::IsSampled(static_cast<EDM84RT::Profile>(profile));
    const char* tag = EDM84RT::IsServoSimulation(static_cast<EDM84RT::Profile>(profile)) ? "EDM93" :
        EDM84RT::IsCommandBatch(static_cast<EDM84RT::Profile>(profile)) ? "EDM92" :
        EDM84RT::IsTrajectoryPreview(static_cast<EDM84RT::Profile>(profile)) ? "EDM91" :
        EDM84RT::IsOutputPreview(static_cast<EDM84RT::Profile>(profile)) ? "EDM90" :
        EDM84RT::IsSourceDwell(static_cast<EDM84RT::Profile>(profile)) ? "EDM89" :
        sampled ? "EDM88" : block.has('R') ? "EDM87" : EDM84RT::DiagnosticTag(static_cast<EDM84RT::Profile>(profile),
        block.has('Q') && block.val('Q') == 4.0 ? EDM84RT::Scenario::ToolHeadCUVCompensation : EDM84RT::Scenario::SixAxisPath);
    const int pc = GetActiveDispatchPC();
    const NCProgramCacheLine* authored = pc >= 0 ? GetBaseProgramCache().TryGetLine(pc) : nullptr;
    bool literal = authored && authored->sourceLineNumber == line &&
        EDM77::Detail::Clean(authored->parsedBlock) && !authored->parsedBlock.isEmpty &&
        authored->parsedBlock.gCount == 1 && authored->parsedBlock.mCount == 0 &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0], 180U) &&
        EDM77::Detail::OnlyAddresses(authored->parsedBlock, sampled ? "NPQRS" : "NPQR") && authored->parsedBlock.has('P') &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.expression('P'), profile) &&
        authored->parsedBlock.has('Q') && block.has('Q') &&
        (block.val('Q') == 1.0 || block.val('Q') == 2.0 || block.val('Q') == 3.0 ||
            (ncAuthored && block.val('Q') == 4.0));
    if (literal)
    {
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (block.has(letter) != authored->parsedBlock.has(letter)) { literal = false; break; }
            if (!block.has(letter)) continue;
            std::uint32_t value = 0U;
            if (!EDM77::Detail::UnsignedLiteral(authored->parsedBlock.expression(letter),
                (std::numeric_limits<std::uint32_t>::max)(), value) || block.val(letter) != static_cast<double>(value))
            { literal = false; break; }
        }
    }
    if (!IsGapDryRunBlockShapeValid(block) || block.isBlockSkip || !literal || !IsEDMCoordinatedRTM30ContinuationSameThread())
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM84_LITERAL_FIXTURE_M30_REQUIRED"); return nullptr; }
    const EDM87::HeadProfile* selectedToolHead = nullptr;
    if (block.has('R'))
    {
        if (!m_edmToolHeadCatalogReady || !EDM87::ValidCatalog(m_edmToolHeadCatalog))
        { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM87_PROFILE_CATALOG_DISABLED"); return nullptr; }
        selectedToolHead = EDM87::FindProfile(m_edmToolHeadCatalog, static_cast<unsigned>(block.val('R')));
        if (!selectedToolHead)
        { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM87_PROFILE_ID_UNAVAILABLE"); return nullptr; }
    }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    if ((mask & ~(1ULL << 9U)) || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_motion.GetCurrentExecutionEpoch() == MOTION_EXECUTION_EPOCH_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM84_NOT_READY"); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM84_RECIPE_SCOPE"); return nullptr; }
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    if (!servo.ready || !flush.ready || !EDM75::SameAdmission(flush.recipe, servo.recipe))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM84_RECIPE_BIND_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM84_CLOCK_FREQUENCY"); return nullptr; }
    const auto latest = m_motion.ReadEDMCoordinatedRTLatestSession();
    const auto serial = latest > m_gapDryRunSerial ? latest : m_gapDryRunSerial;
    if (serial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM84_SESSION_OVERFLOW"); return nullptr; }
    EDM84RT::FixtureDetail::ResetInPlace(m_edmCoordinatedRT);
    auto& control = m_edmCoordinatedRT;
    control.servo = servo.recipe; control.flush = flush.recipe;
    auto& request = control.request;
    request.session = serial + 1ULL;
    request.profile = static_cast<EDM84RT::Profile>(profile);
    request.sampledFault = block.has('S') ?
        static_cast<EDM84RT::SampledFault>(static_cast<unsigned>(block.val('S'))) : EDM84RT::SampledFault::Normal;
    request.scenario = static_cast<EDM84RT::Scenario>(static_cast<unsigned>(block.val('Q')));
    request.qpcFrequency = static_cast<std::uint64_t>(frequency.QuadPart);
    request.scope.run = m_pathCoreLiveBookkeeping.currentRunToken;
    request.scope.cache = GetBaseProgramCache().GetGeneration();
    request.scope.dispatch = m_currentExecutingBlockDispatchId;
    request.scope.executionEpoch = m_motion.GetCurrentExecutionEpoch();
    request.scope.ownerLease = (static_cast<std::uint64_t>(m_programMotionLease.owner) << 32U) |
        static_cast<std::uint64_t>(m_programMotionLease.generation);
    // Fixed diagnostic coordinates/frame/kinematics revision; not live CNC/TCP.
    request.scope.configRevision = request.scope.frameRevision = request.scope.kinematicsRevision = 1ULL;
    if (selectedToolHead)
    {
        request.toolHeadSelection.catalogRevision = m_edmToolHeadCatalog.revision;
        request.toolHeadSelection.profile = *selectedToolHead;
        request.toolHeadSelection.fingerprint = EDM87::Fingerprint(*selectedToolHead);
        if (!EDM87::ValidSelection(request.toolHeadSelection))
        { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM87_PROFILE_SELECTION_INVALID"); return nullptr; }
        // The copied canonical path freezes this same scope during Capture.
        request.scope.configRevision = request.toolHeadSelection.catalogRevision;
        request.scope.frameRevision = request.scope.kinematicsRevision = selectedToolHead->revision;
    }
    request.scope.recipeGeneration = servo.recipe.binding.generation;
    request.scope.sourceGeneration = request.session;
    if (!ReadEDMCoordinatedRTClockSameThread(request.issuedUs))
    { RejectEDMGapAdmissionSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM84_CLOCK_READ"); return nullptr; }
    LogEDMCoordinatedRTStartupStageSameThread("PREPARE");
    if (ncAuthored)
    {
        // The first M30 stays in normal dispatch. Data AFTER it is read-only:
        // never advance/jump the PC, resolve macros, or dispatch its G0/G1/G2/G3.
        if (pc < 0 || pc > (std::numeric_limits<int>::max)() - 97)
        { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM85_SOURCE_RANGE"); return nullptr; }
        const auto get = [this, pc](unsigned offset) noexcept -> const NCParsedBlock*
        {
            const auto* row = GetBaseProgramCache().TryGetLine(pc + static_cast<int>(offset));
            return row ? &row->parsedBlock : nullptr;
        };
        control.authoredCapture = EDM85NC::Capture(request, control.authoredWorkspace, get);
        if (!control.authoredCapture.valid)
        {
            RtPrintf("[%s] event=CAPTURE_REJECT reason=%s sourcePC=%d sourceOffset=%u newSession=0 armIssued=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
                tag, EDM85NC::ErrorName(control.authoredCapture.error), pc,
                control.authoredCapture.errorOffset);
            RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM85_AUTHORED_PATH_REJECTED"); return nullptr;
        }
        if (!EDM84RT::BuildAuthoredRequest(request, servo.recipe.curve,
            servo.recipe.referenceV, servo.recipe.overridePercent, control.authoredCapture.feedMmMin))
        { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM85_PATH_CONFIG_OR_BUDGET"); return nullptr; }
    }
    else if (!EDM84RT::BuildFixtureRequest(request, control.workspace, servo.recipe.curve, servo.recipe.referenceV, servo.recipe.overridePercent))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM84_FIXTURE_BUILD"); return nullptr; }
    if (ncAuthored)
    {
        char detail[640]{};
        const int count = std::snprintf(detail, sizeof(detail),
            "[%s] event=PATH_CAPTURE session=%llu plan=NC_AUTHORED_AFTER_M30 sourcePC=%d terminalOffset=%u segments=%u lines=%u arcs=%u expectedAxisMask=%u expectedRowMask=%u lengthMm=%.9f authoredF=%.6f effectiveF=%.6f modelBudgetUs=%llu livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(request.session), pc, control.authoredCapture.terminalOffset,
            control.authoredCapture.segmentCount, control.authoredCapture.lineCount, control.authoredCapture.arcCount,
            request.expectedAxisMask, request.expectedRowMask, request.path.LengthMm(),
            control.authoredCapture.feedMmMin, request.config.motion.maxScalarMmMin,
            static_cast<unsigned long long>(EDM84RT::DurationUs(request)));
        if (count < 0 || static_cast<std::size_t>(count) >= sizeof(detail))
        { RejectEDMGapAdmissionSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM85_CAPTURE_LOG_FORMAT"); return nullptr; }
        RtPrintf("%s", detail);
    }
    if (!LogEDM86ToolHeadConfiguration(request))
    { RejectEDMGapAdmissionSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM86_KINEMATICS_LOG_FORMAT"); return nullptr; }
    LogEDM88SampledSourceConfiguration(request);
    LogEDMCoordinatedRTStartupStageSameThread("BUILT");
    // Preparation/console latency must not age a request before it is even
    // submitted. This refresh occurs only before RT publication, never as an
    // extension of an active session's heartbeat or completion deadline.
    const auto preparedUs = request.issuedUs;
    if (!ReadEDMCoordinatedRTClockSameThread(request.issuedUs) || request.issuedUs < preparedUs)
    { RejectEDMGapAdmissionSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM84_SUBMIT_CLOCK_READ"); return nullptr; }
    InvalidateEDMPhysicalSequencePlanSameThread();
    m_edmGapDynamic.strictSequenceMode = false;
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.coordinatedRTTest = m_gapDryRun.active = true;
    m_gapDryRun.sourceLine = line; m_gapDryRun.test = request.session;
    m_gapDryRunSerial = request.session;
    m_gapDryRun.run = request.scope.run; m_gapDryRun.cache = request.scope.cache;
    m_gapDryRun.dispatch = request.scope.dispatch;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    control.lastClockUs = control.lastServiceUs = control.lastFeedbackUs = request.issuedUs;
    control.deadlineUs = EDM84RT::DurationUs(request) + 15000000ULL;
    // Mark before submission: a failing transport may already have reserved
    // the session. Cancellation and a later RT receipt are still mandatory.
    control.submitted = true; control.retired = false;
    if (!m_motion.StartEDMCoordinatedRT(request))
    {
        // A request rejected before reservation has no RT-owned state to retire.
        if (m_motion.ReadEDMCoordinatedRTLatestSession() < request.session)
        { control.submitted = false; control.retired = true; }
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM84_START_REJECTED"); return nullptr;
    }
    LogEDMCoordinatedRTStartupStageSameThread("SUBMITTED");
    RtPrintf("[%s] event=BEGIN build=EDM93_20261012 P=%u Q=%u scenario=%s session=%llu run=%llu dispatch=%llu source=SIM executor=RT_CALLBACK modelStepUs=250 output=NONE axes=XYZCUV rotary=COORDINATED geometry=CNC_RETAINED_COMPOSITION livePath=0 hold=CANCEL_RESET_REQUIRED wcetVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        tag, profile, static_cast<unsigned>(request.scenario), EDM84RT::ScenarioName(request.scenario), static_cast<unsigned long long>(request.session), static_cast<unsigned long long>(request.scope.run),
        static_cast<unsigned long long>(request.scope.dispatch));
    char text[480]{};
    const int length = std::snprintf(text, sizeof(text),
        "[%s] event=RECIPE_FROZEN table=%u E=%u generation=%llu processRev=%u tuningGen=%llu E8=%.6f E9=%.6f curve=INSTALLED input=SYNTHETIC output=NONE motion=0 physicalPermit=0 discharge=0\n",
        tag, control.servo.binding.tableId, static_cast<unsigned>(control.servo.binding.eCode),
        static_cast<unsigned long long>(control.servo.binding.generation), control.servo.binding.processRevision,
        static_cast<unsigned long long>(control.servo.binding.tuningGeneration), control.servo.overridePercent, control.servo.referenceV);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(text))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM84_RECIPE_LOG_FORMAT"); return nullptr; }
    RtPrintf("%s", text);
    return WaitForGapDryRunCallback;
}

bool NCManager::ReadEDMCoordinatedRTClockSameThread(std::uint64_t& nowUs) noexcept
{
    auto& control = m_edmCoordinatedRT;
    const auto frequency = control.request.qpcFrequency;
    LARGE_INTEGER counter{};
    if (!frequency || frequency > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0) return false;
    const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
    if (ticks < control.lastTicks) return false;
    const auto seconds = ticks / frequency;
    if (seconds > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL) return false;
    const auto fraction = (ticks % frequency) * 1000000ULL / frequency;
    if (seconds * 1000000ULL > (std::numeric_limits<std::uint64_t>::max)() - fraction) return false;
    nowUs = seconds * 1000000ULL + fraction;
    control.lastTicks = ticks;
    return true;
}

bool NCManager::IsEDMCoordinatedRTScopeCurrentSameThread() const noexcept
{
    const auto& selection = m_edmCoordinatedRT.request.toolHeadSelection;
    if (!EDM87::IsEmptySelection(selection))
    {
        const auto* profile = m_edmToolHeadCatalogReady ?
            EDM87::FindProfile(m_edmToolHeadCatalog, selection.profile.id) : nullptr;
        if (!profile || !EDM87::ValidSelection(selection) ||
            selection.catalogRevision != m_edmToolHeadCatalog.revision ||
            !EDM87::ProfilesEqual(selection.profile, *profile)) return false;
    }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    return m_gapDryRun.coordinatedRTTest && !(mask & ~(1ULL << 9U)) &&
        !m_edmPhysicalServoRT.retirePending && !m_edmGapServoRT.retirePending &&
        !m_edmCNCRetainedRT.retirePending && (!m_edmCNCRetainedRT.submitted || m_edmCNCRetainedRT.retired) &&
        m_gapDryRun.run != 0ULL && m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache != 0ULL && m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch != NC_BLOCK_DISPATCH_ID_INVALID && m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch != MOTION_EXECUTION_EPOCH_INVALID && m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease);
}

bool NCManager::IsEDMCoordinatedRTRecipeCurrentSameThread() const noexcept
{
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
        m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease)) return false;
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    return servo.ready && flush.ready && EDM75::SameAdmission(flush.recipe, servo.recipe) &&
        EDM59::SameRecipe(servo.recipe, m_edmCoordinatedRT.servo) && EDM57::SameRecipe(flush.recipe, m_edmCoordinatedRT.flush);
}

void NCManager::LogEDMCoordinatedRTSameThread(const char* event, const char* reason) noexcept
{
    const auto& c = m_edmCoordinatedRT;
    const auto& f = c.feedback;
    const char* result = std::strcmp(event ? event : "", "SUMMARY") == 0 ? "PASS" :
        std::strcmp(event ? event : "", "FAIL") == 0 ? "FAIL" :
        std::strcmp(event ? event : "", "CANCEL") == 0 ? "CANCELLED" : "PENDING";
    RtPrintf("[%s] event=%.24s result=%s reason=%.64s P=%u Q=%u session=%llu state=%s rtReason=%s kernelReason=%u stage=%u pub=%llu rtTick=%llu terminalTick=%llu quiescentTick=%llu quiescent=%u output=NONE motion=0 physicalPermit=0 discharge=0\n",
        EDM84RT::DiagnosticTag(c.request), event ? event : "UNKNOWN", result, reason ? reason : "NONE", static_cast<unsigned>(c.request.profile), static_cast<unsigned>(c.request.scenario),
        static_cast<unsigned long long>(c.request.session), EDM84RT::StateName(f.state), EDM84RT::ReasonName(f.reason),
        static_cast<unsigned>(f.kernelReason), static_cast<unsigned>(f.stage), static_cast<unsigned long long>(f.publication), static_cast<unsigned long long>(f.rtTick),
        static_cast<unsigned long long>(f.terminalTick), static_cast<unsigned long long>(f.quiescentTick), f.quiescent ? 1U : 0U);
}

void NCManager::EndEDMCoordinatedRTSameThread(bool failed, const char* reason) noexcept
{
    auto& c = m_edmCoordinatedRT;
    const bool report = m_gapDryRun.coordinatedRTTest &&
        (m_gapDryRun.active || m_gapDryRun.result == 1U || (failed && m_gapDryRun.result == 2U));
    if (c.submitted && !c.retired)
    {
        // Publish the exact-session cancellation before any console work.
        m_motion.CancelEDMCoordinatedRT(c.request.session);
        if (!c.retirePending)
        {
            c.cancelPublication = c.feedback.publication; c.cancelTick = c.feedback.rtTick;
            c.cancelUs = c.lastClockUs; c.retirePending = true;
        }
    }
    c.completionPending = false;
    if (m_gapDryRun.coordinatedRTTest)
    {
        m_gapDryRun.active = m_gapDryRun.paused = false;
        if (report) m_gapDryRun.result = failed ? 3U : 2U;
    }
    if (report)
    {
        if (EDM84RT::IsTrajectoryPreview(c.request.profile))
        {
            // The NC copy may predate this cancellation. Report the request
            // now; report numerical stop/abort evidence only after RT receipt.
            c.trajectoryCancelLogPending = true;
            c.trajectoryCancelFailed = c.trajectoryCancelFailed || failed;
            LogEDMCoordinatedRTSameThread(failed ? "FAIL_REQUEST" : "CANCEL_REQUEST", reason);
        }
        else
        {
            LogEDMCoordinatedRTSameThread(failed ? "FAIL" : "CANCEL", reason);
            LogEDM88SampledSourceEvidence(c.request, c.feedback, failed ? "FAIL" : "CANCELLED");
            LogEDM90OutputPreviewEvidence(c.request, c.feedback, failed ? "FAIL" : "CANCELLED");
        }
    }
}

void NCManager::ServiceEDMCoordinatedRTRetirementSameThread() noexcept
{
    auto& c = m_edmCoordinatedRT;
    if (!c.retirePending) return;
    EDM84RT::Feedback receipt{};
    if (m_motion.ReadEDMCoordinatedRT(receipt) && receipt.session == c.request.session &&
        receipt.latestSession == c.request.session && receipt.profile == c.request.profile && receipt.scenario == c.request.scenario &&
        EDM84RT::ToolHeadFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::SampledFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::PreviewFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::TrajectoryFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::TrajectoryRetirementValid(c.request, receipt) &&
        EDM84RT::BatchFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::BatchRetirementValid(c.request, receipt) &&
        EDM84RT::ServoFeedbackIdentityValid(c.request, receipt) &&
        EDM84RT::ServoRetirementValid(c.request, receipt) &&
        (!c.machinePreviewSeen || EDM90::SameConfig(c.acceptedPreviewConfig, receipt.previewConfig)) &&
        (!c.completionPending || EDM84RT::SameSampledTerminalEvidence(receipt, c.completedFeedback)) &&
        EDM81::SameScope(receipt.scope, c.request.scope) && receipt.cancelThroughSession >= c.request.session &&
        receipt.publication > c.cancelPublication && receipt.rtTick > c.cancelTick &&
        receipt.state == EDM84RT::State::Cancelled && receipt.quiescent &&
        receipt.terminalTick > c.cancelTick && receipt.quiescentTick > receipt.terminalTick &&
        receipt.rtTick >= receipt.quiescentTick && receipt.qpcFrequency == c.request.qpcFrequency)
    {
        // RT stops republishing after exact cancellation and quiescence. This
        // immutable acknowledgement remains valid after a long NC HOLD/RESET.
        c.feedback = receipt; c.retired = true; c.retirePending = false;
        LogEDMCoordinatedRTSameThread("RETIRED", "EXACT_SESSION_RT_QUIESCENT");
        if (c.trajectoryCancelLogPending)
        {
            const char* result = c.trajectoryCancelFailed ? "FAIL" : "CANCELLED";
            LogEDM88SampledSourceEvidence(c.request, receipt, result);
            LogEDM90OutputPreviewEvidence(c.request, receipt, result);
            LogEDM91TrajectoryEvidence(c.request, receipt, result);
            LogEDM92CommandBatchEvidence(c.request, receipt, result);
            LogEDM93ServoSimulationEvidence(c.request, receipt, result);
            c.trajectoryCancelLogPending = false;
        }
        return;
    }
    std::uint64_t nowUs = 0ULL;
    const std::uint64_t retirementWarningUs = EDM84RT::IsServoSimulation(c.request.profile) ? 6000000ULL : 2000000ULL;
    if (!c.retirementWarning && ReadEDMCoordinatedRTClockSameThread(nowUs) &&
        nowUs >= c.cancelUs && nowUs - c.cancelUs >= retirementWarningUs)
    {
        c.retirementWarning = true;
        LogEDMCoordinatedRTSameThread("RETIRE_WAIT", "NO_VALID_RT_QUIESCENT_ACK_START_BLOCKED");
    }
}

void NCManager::ValidateEDMCoordinatedRTSameThread()
{
    auto& c = m_edmCoordinatedRT;
    if (!m_gapDryRun.active) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMCoordinatedRTSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMCoordinatedRTScopeCurrentSameThread())
    { EndEDMCoordinatedRTSameThread(false, "NC_SCOPE_REVOKED"); return; }
    if (!IsEDMCoordinatedRTRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_LIVE_RECIPE_CHANGED"); return; }
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMCoordinatedRTClockSameThread(nowUs) || nowUs < c.lastClockUs || nowUs < c.request.issuedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_CLOCK_INVALID"); return; }
    if (nowUs - c.lastServiceUs > 100000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_NC_SERVICE_GAP"); return; }
    if (nowUs == c.lastClockUs) ++c.stalledPolls;
    else { c.lastClockUs = nowUs; c.stalledPolls = 0U; }
    if (c.stalledPolls >= 10000U || nowUs - c.request.issuedUs >= c.deadlineUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        c.stalledPolls >= 10000U ? "EDM84_CLOCK_STALLED" : "EDM84_WINDOW_TIMEOUT"); return; }
    c.lastServiceUs = nowUs;
    if (!c.completionPending && !m_motion.TouchEDMCoordinatedRT(c.request.session, nowUs))
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_HEARTBEAT_REJECTED");
}

bool NCManager::ProcessEDMCoordinatedRTSameThread()
{
    ServiceEDMCoordinatedRTRetirementSameThread();
    ValidateEDMCoordinatedRTSameThread();
    auto& c = m_edmCoordinatedRT;
    const char* tag = EDM84RT::DiagnosticTag(c.request);
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && c.retired && IsEDMCoordinatedRTScopeCurrentSameThread() &&
            IsEDMCoordinatedRTRecipeCurrentSameThread()) return true;
        if ((m_gapDryRun.result == 1U || m_gapDryRun.result == 2U) && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_CANCELLED_CALLBACK_RESET_REQUIRED");
        return false;
    }
    if (c.completionPending)
    {
        if (!c.retired) return false;
        // Scope/recipe were rechecked above AFTER exact-session retirement.
        m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        LogEDMCoordinatedRTSameThread("SUMMARY", "PASS");
        RtPrintf("[%s] event=TIMING result=PASS session=%llu Q=%u axisMask=%u callbackSteps=%llu modelUs=%llu observedUs=%llu minCallbackUs=%llu maxCallbackUs=%llu coverage=%llu clock=QPC modelStepUs=250 wcetVerified=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(c.request.session), static_cast<unsigned>(c.request.scenario),
            c.completedFeedback.axisMovedMask, static_cast<unsigned long long>(c.completedSteps),
            static_cast<unsigned long long>(c.completedModelUs), static_cast<unsigned long long>(c.lastClockUs - c.request.issuedUs),
            static_cast<unsigned long long>(c.feedback.minimumStepUs), static_cast<unsigned long long>(c.feedback.maximumStepUs),
            static_cast<unsigned long long>(c.completedCoverage));
        const auto& final = c.completedFeedback;
        LogEDM86ToolHeadCompletion(c.request, final);
        LogEDM88SampledSourceEvidence(c.request, final, "PASS");
        LogEDM90OutputPreviewEvidence(c.request, final, "PASS");
        LogEDM91TrajectoryEvidence(c.request, final, "PASS");
        LogEDM92CommandBatchEvidence(c.request, final, "PASS");
        LogEDM93ServoSimulationEvidence(c.request, final, "PASS");
        if (EDM84RT::IsAuthored(c.request.profile))
            RtPrintf("[%s] event=PATH_EVIDENCE result=PASS session=%llu nominalAxisMask=%u expectedAxisMask=%u visitedRowMask=%u expectedRowMask=%u modelBudgetUs=%llu plan=NC_AUTHORED_AFTER_M30 livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
                tag, static_cast<unsigned long long>(c.request.session), final.nominalAxisMovedMask, c.request.expectedAxisMask,
                final.visitedRowMask, c.request.expectedRowMask, static_cast<unsigned long long>(EDM84RT::DurationUs(c.request)));
        const auto& nominal = final.sample.nominal; const auto& pose = final.sample.pose;
        char composition[1280]{};
        const int length = std::snprintf(composition, sizeof(composition),
            "[%s] event=COMPOSITION result=PASS session=%llu Q=%u axisMask=%u nominalXYZCUV=%.9f,%.9f,%.9f,%.9f,%.9f,%.9f composedXYZCUV=%.9f,%.9f,%.9f,%.9f,%.9f,%.9f maxLinearErrorMm=%.12g maxRotaryErrorDeg=%.12g maxMappingErrorMm=%.12g maxOffsetErrorMm=%.12g cursorMm=%.9f offsetMm=%.9f scopeRun=%llu scopeCache=%llu scopeDispatch=%llu livePath=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, static_cast<unsigned long long>(c.request.session), static_cast<unsigned>(c.request.scenario), final.axisMovedMask,
            nominal[0], nominal[1], nominal[2], nominal[3], nominal[4], nominal[5],
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
            final.maxLinearErrorMm, final.maxRotaryErrorDeg, final.maxMappingErrorMm, final.maxOffsetErrorMm,
            final.sample.cursorMm, final.sample.offsetMm, static_cast<unsigned long long>(c.request.scope.run),
            static_cast<unsigned long long>(c.request.scope.cache), static_cast<unsigned long long>(c.request.scope.dispatch));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(composition)) RtPrintf("%s", composition);
        return true;
    }
    EDM84RT::Feedback receipt{};
    if (!m_motion.ReadEDMCoordinatedRT(receipt) || !receipt.publication || !receipt.session)
    {
        if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_RT_RECEIPT_TIMEOUT");
        return false;
    }
    // A new asynchronous request can initially read the previous session's
    // terminal bank. It grants no credit and is bounded by the start timeout.
    if (!c.lastPublication && receipt.session < c.request.session)
    {
        if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_RT_START_ACK_TIMEOUT");
        return false;
    }
    if (receipt.session != c.request.session || receipt.latestSession != c.request.session ||
        receipt.profile != c.request.profile || receipt.scenario != c.request.scenario || !EDM81::SameScope(receipt.scope, c.request.scope) ||
        !EDM84RT::ToolHeadFeedbackIdentityValid(c.request, receipt) ||
        !EDM84RT::SampledFeedbackIdentityValid(c.request, receipt) ||
        !EDM84RT::PreviewFeedbackIdentityValid(c.request, receipt) ||
        !EDM84RT::TrajectoryFeedbackIdentityValid(c.request, receipt) ||
        !EDM84RT::BatchFeedbackIdentityValid(c.request, receipt) ||
        !EDM84RT::ServoFeedbackIdentityValid(c.request, receipt) ||
        receipt.cancelThroughSession >= c.request.session || receipt.publication < c.lastPublication || receipt.rtTick < c.lastRTTick)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_RT_RECEIPT_SCOPE"); return false; }
    // Read the NC clock AFTER copying the RT bank: normal RT preemption may
    // publish a sample newer than Validate's earlier QPC reading.
    std::uint64_t observedUs = 0ULL;
    if (!ReadEDMCoordinatedRTClockSameThread(observedUs) || observedUs < c.lastClockUs ||
        observedUs - c.lastServiceUs > 100000ULL || observedUs - c.request.issuedUs >= c.deadlineUs ||
        receipt.qpcFrequency != c.request.qpcFrequency || !receipt.nowUs ||
        receipt.nowUs > observedUs || observedUs - receipt.nowUs > 100000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_RT_RECEIPT_CLOCK"); return false; }
    c.lastClockUs = observedUs;
    if (EDM84RT::IsOutputPreview(c.request.profile) && receipt.startAcknowledged)
    {
        if (c.machinePreviewSeen && !EDM90::SameConfig(c.acceptedPreviewConfig, receipt.previewConfig))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM90_PREVIEW_CONFIG_CHANGED"); return false; }
        if (!c.machinePreviewSeen)
        {
            c.acceptedPreviewConfig = receipt.previewConfig;
            c.machinePreviewSeen = true;
            LogEDM90OutputPreviewConfiguration(c.request, receipt);
            LogEDM91TrajectoryConfiguration(c.request, receipt);
            LogEDM92CommandBatchConfiguration(c.request, receipt);
            LogEDM93ServoSimulationConfiguration(c.request, receipt);
        }
    }
    c.feedback = receipt;
    if (receipt.publication > c.lastPublication && receipt.rtTick > c.lastRTTick)
    { c.lastFeedbackUs = c.lastClockUs; c.lastPublication = receipt.publication; c.lastRTTick = receipt.rtTick; }
    else if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_RT_RECEIPT_STALLED"); return false; }
    if (receipt.state == EDM84RT::State::Fault || receipt.state == EDM84RT::State::Cancelled)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, EDM84RT::ReasonName(receipt.reason)); return false; }
    // Bounded NC-side milestones only; never log inside the 250 us callback.
    const auto kernelState = static_cast<unsigned>(receipt.sample.state);
    if (receipt.startAcknowledged && kernelState != c.lastLoggedKernelState && c.excursionLogCount < 24U)
    {
        const char* name = receipt.sample.state == EDM81::State::ShortRetreat ? "SHORT_RETREAT" :
            receipt.sample.state == EDM81::State::JumpRetreat ? "JUMP_RETREAT" :
            receipt.sample.state == EDM81::State::Returning ? "RETURNING" :
            receipt.sample.state == EDM81::State::Running ? "RUNNING" : "TERMINAL";
        char milestone[480]{};
        const int length = std::snprintf(milestone, sizeof(milestone),
            "[%s] event=EXCURSION state=%s session=%llu Q=%u stage=%u modelUs=%llu cursorMm=%.9f anchorMm=%.9f offsetMm=%.9f output=NONE motion=0 physicalPermit=0 discharge=0\n",
            tag, name, static_cast<unsigned long long>(c.request.session), static_cast<unsigned>(c.request.scenario),
            receipt.stage, static_cast<unsigned long long>(receipt.modelUs),
            receipt.sample.cursorMm, receipt.sample.anchorMm, receipt.sample.offsetMm);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(milestone)) RtPrintf("%s", milestone);
        c.lastLoggedKernelState = kernelState; ++c.excursionLogCount;
    }
    if (receipt.state != EDM84RT::State::Completed || !receipt.startAcknowledged || !receipt.quiescent ||
        !receipt.terminalTick || receipt.quiescentTick <= receipt.terminalTick || receipt.rtTick < receipt.quiescentTick)
        return false;
    if (receipt.reason != EDM84RT::Reason::None || !EDM84RT::CompletionCoverageValid(c.request, receipt) ||
        !EDM84RT::BatchCompletionValid(c.request, receipt) ||
        !EDM84RT::ServoCompletionValid(c.request, receipt) ||
        !receipt.runtimeSteps || receipt.modelUs < EDM84RT::DurationUs(c.request) ||
        receipt.sample.state != EDM81::State::Completed || receipt.sample.reason != EDM81::Reason::None ||
        !EDM81::SameScope(receipt.sample.scope, c.request.scope) ||
        receipt.sample.offsetMm != 0.0 || receipt.sample.cursorMm != receipt.sample.pathLengthMm ||
        !std::isfinite(receipt.maxLinearErrorMm) || !std::isfinite(receipt.maxRotaryErrorDeg) ||
        !std::isfinite(receipt.maxMappingErrorMm) || !std::isfinite(receipt.maxOffsetErrorMm) ||
        receipt.maxLinearErrorMm < 0.0 || receipt.maxLinearErrorMm > 1e-10 ||
        receipt.maxRotaryErrorDeg < 0.0 || receipt.maxRotaryErrorDeg > 1e-10 ||
        receipt.maxMappingErrorMm < 0.0 || receipt.maxMappingErrorMm > 1e-10 ||
        receipt.maxOffsetErrorMm < 0.0 || receipt.maxOffsetErrorMm > 1e-10)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_COMPLETION_EVIDENCE"); return false; }
    if (!c.completedPublication)
    {
        c.completedFeedback = receipt;
        c.completedPublication = receipt.publication; c.completedTick = receipt.rtTick;
        c.completedTerminalTick = receipt.terminalTick; c.completedQuiescentTick = receipt.quiescentTick;
        c.completedSteps = receipt.runtimeSteps;
        c.completedModelUs = receipt.modelUs; c.completedCoverage = receipt.coverageMask;
        LogEDMCoordinatedRTSameThread("COMPLETE_OBSERVED", "WAIT_SECOND_PUBLICATION");
        return false;
    }
    if (receipt.publication <= c.completedPublication || receipt.rtTick <= c.completedTick) return false;
    bool samePose = true;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        samePose = samePose && receipt.sample.pose[axis] == c.completedFeedback.sample.pose[axis] &&
            receipt.sample.nominal[axis] == c.completedFeedback.sample.nominal[axis] &&
            receipt.sample.anchorPose[axis] == c.completedFeedback.sample.anchorPose[axis];
    for (unsigned axis = 0U; axis < 3U; ++axis)
        samePose = samePose && receipt.sample.frozenDirection[axis] == c.completedFeedback.sample.frozenDirection[axis];
    for (unsigned axis = 0U; axis < 3U; ++axis)
        samePose = samePose && receipt.resolvedDirectionUnit[axis] == c.completedFeedback.resolvedDirectionUnit[axis];
    if (!samePose || !EDM84RT::SameSampledTerminalEvidence(receipt, c.completedFeedback) ||
        receipt.toolHeadProfileId != c.completedFeedback.toolHeadProfileId ||
        receipt.toolHeadProfileRevision != c.completedFeedback.toolHeadProfileRevision ||
        receipt.toolHeadCatalogRevision != c.completedFeedback.toolHeadCatalogRevision ||
        receipt.toolHeadFingerprint != c.completedFeedback.toolHeadFingerprint ||
        receipt.orientationSamples != c.completedFeedback.orientationSamples ||
        receipt.maxOrientationNormError != c.completedFeedback.maxOrientationNormError ||
        receipt.maxFrozenDirectionError != c.completedFeedback.maxFrozenDirectionError ||
        receipt.sample.anchorMm != c.completedFeedback.sample.anchorMm ||
        receipt.sample.ordinal != c.completedFeedback.sample.ordinal || receipt.sample.u != c.completedFeedback.sample.u ||
        receipt.sample.steps != c.completedFeedback.sample.steps || receipt.stageMask != c.completedFeedback.stageMask ||
        receipt.axisMovedMask != c.completedFeedback.axisMovedMask ||
        receipt.nominalAxisMovedMask != c.completedFeedback.nominalAxisMovedMask ||
        receipt.visitedRowMask != c.completedFeedback.visitedRowMask ||
        receipt.source != c.completedFeedback.source || receipt.durationUs != c.completedFeedback.durationUs ||
        receipt.maxLinearErrorMm != c.completedFeedback.maxLinearErrorMm ||
        receipt.maxRotaryErrorDeg != c.completedFeedback.maxRotaryErrorDeg ||
        receipt.maxMappingErrorMm != c.completedFeedback.maxMappingErrorMm ||
        receipt.maxOffsetErrorMm != c.completedFeedback.maxOffsetErrorMm ||
        receipt.terminalTick != c.completedTerminalTick || receipt.quiescentTick != c.completedQuiescentTick ||
        receipt.runtimeSteps != c.completedSteps ||
        receipt.modelUs != c.completedModelUs || receipt.coverageMask != c.completedCoverage ||
        !IsEDMCoordinatedRTScopeCurrentSameThread() || !IsEDMCoordinatedRTRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM84_TERMINAL_RECEIPT_CHANGED"); return false; }
    // Even PASS retires the exact RT session. M30 cannot consume this wait
    // until a later valid callback proves the cancelled calculator quiescent.
    m_motion.CancelEDMCoordinatedRT(c.request.session);
    c.cancelPublication = receipt.publication; c.cancelTick = receipt.rtTick; c.cancelUs = c.lastClockUs;
    c.retirePending = c.completionPending = true;
    LogEDMCoordinatedRTSameThread("RETIRE_REQUEST", "COMPLETED_TWO_PUBLICATIONS");
    return false;
}

// EDM55: NC supervises a SIM-only RT servo session. No axis or PDO output.
bool NCManager::ReadEDMGapServoClockSameThread(std::uint64_t& nowUs) noexcept
{
    LARGE_INTEGER counter{};
    if (!m_gapDryRun.frequency ||
        m_gapDryRun.frequency > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0) return false;
    const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
    if (ticks < m_gapDryRun.lastTicks) return false;
    const auto seconds = ticks / m_gapDryRun.frequency;
    const auto fraction = (ticks % m_gapDryRun.frequency) * 1000000ULL / m_gapDryRun.frequency;
    if (seconds > ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000000ULL) return false;
    nowUs = seconds * 1000000ULL + fraction;
    m_gapDryRun.lastTicks = ticks;
    return nowUs != 0ULL;
}

bool NCManager::IsEDMGapServoRunReadySameThread(std::uint64_t* reasonMask,
    MotionStopSettleSnapshot* diagnostic) const noexcept
{
    // Bits: machine, cleanup, macro, path, lease, owner, safety, group,
    // queues, exact RT readiness. Raw derivative velocity is diagnostic only.
    std::uint64_t mask = 0ULL;
    if (!IsEDMShortFlushRunReadySameThread()) mask |= 1ULL << 0U;
    if (m_edmZFixture.cleanupPending || m_edmZFixture.resetPending || m_edmZFixture.failureAlarmPending)
        mask |= 1ULL << 1U;
    if (m_isG66Active || !m_macroStack.empty()) mask |= 1ULL << 2U;
    if (m_pathFeed.pending || m_pathArc.pending || m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound)
        mask |= 1ULL << 3U;
    if (!m_programMotionLease.IsValid() || m_programMotionLease.owner != MotionOwner::AUTO) mask |= 1ULL << 4U;
    if (!m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)) mask |= 1ULL << 5U;
    if (m_motion.HasPendingSafetyOrRecoveryRequests()) mask |= 1ULL << 6U;
    if (!m_motion.IsGroupDone()) mask |= 1ULL << 7U;
    if (m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U) mask |= 1ULL << 8U;
    // Use the same paired publication / incoming-axis predicate as program
    // START. It retains servo/fault/IDLE/in-position/finite-command checks,
    // zero commanded velocity, following window and exact owner/epoch drain.
    if (!m_motion.HasExactProgramStartQuiescenceAcknowledgement(
        m_motion.GetCurrentExecutionEpoch(), m_programMotionLease)) mask |= 1ULL << 9U;
    if (reasonMask) *reasonMask = mask;
    if (diagnostic) *diagnostic = m_motion.GetStopSettleSnapshot();
    return mask == 0ULL;
}

void NCManager::LogEDMGapServoAdmissionSameThread(EDM55RT::Profile profile,
    std::uint64_t mask, const MotionStopSettleSnapshot& raw, const char* reason) noexcept
{
    char text[480]{};
    const int length = std::snprintf(text, sizeof(text),
        "[EDM55] event=ADMISSION build=EDM55_FIX1 reason=%.40s P=%u line=%d mask=0x%llX formalReady=%u rawDiagOnly=1 rawGen=%llu rawSample=%llu axes=%u nonIdle=%u cmdMove=%u actMove=%u rawStand=%u outside=%u motion=0 discharge=0\n",
        reason ? reason : "UNKNOWN", static_cast<unsigned>(profile), m_gapDryRun.sourceLine,
        static_cast<unsigned long long>(mask), (mask & (1ULL << 9U)) == 0ULL ? 1U : 0U,
        static_cast<unsigned long long>(raw.publicationGeneration), static_cast<unsigned long long>(raw.sampleSequence),
        raw.existingAxisCount, raw.nonIdleAxisCount, raw.commandMovingAxisCount, raw.actualMovingAxisCount,
        raw.standstill ? 1U : 0U, raw.outsideInPositionWindowAxisCount);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM55] event=ADMISSION reason=LOG_FORMAT motion=0 discharge=0\n");
}

bool NCManager::IsEDMGapServoScopeCurrentSameThread(bool allowHold) const noexcept
{
    return m_gapDryRun.runtimeServoTest && m_mode == NCOperationMode::MEMORY &&
        (m_state == NCState::RUN || (allowHold && m_state == NCState::HOLD)) &&
        !Close_System_Com_flag && !AlarmManager::GetInstance().HasAlarm() &&
        m_edmState != EDMState::ALARM && !Homing.IsActive() &&
        m_externalReadyInterlock && m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
        !m_resetSafetyOutputHoldActive && !m_isG66Active && m_macroStack.empty() &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        (allowHold || (m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
            m_gapDryRun.lease.Matches(m_programMotionLease) &&
            m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)));
}

void NCManager::LogEDMGapServoRTSameThread(const char* event, const char* reason) noexcept
{
    const auto& f = m_edmGapServoRT.feedback;
    char text[480]{};
    const int length = std::snprintf(text, sizeof(text),
        "[EDM55] event=%.12s reason=%.36s P=%u session=%llu restart=%u rtTick=%llu pub=%llu state=%u fault=%u phase=%u mask=%llu steps=%llu fresh=%llu raw=%.2f V=%.2f target=%.3f speed=%.3f end=%llu zeroTick=%llu zero=%u source=SIM motion=0 discharge=0\n",
        event ? event : "UNKNOWN", reason ? reason : "NONE", static_cast<unsigned>(m_edmGapServoRT.profile),
        static_cast<unsigned long long>(m_edmGapServoRT.scope.session), static_cast<unsigned>(m_edmGapServoRT.restartCount),
        static_cast<unsigned long long>(f.rtTick), static_cast<unsigned long long>(f.publication),
        static_cast<unsigned>(f.state), static_cast<unsigned>(f.reason), static_cast<unsigned>(f.stage),
        static_cast<unsigned long long>(f.coverageMask), static_cast<unsigned long long>(f.runtimeSteps),
        static_cast<unsigned long long>(f.freshSamples),
        f.rawVoltageV, f.filteredVoltageV, f.targetMmMin, f.speedMmMin, static_cast<unsigned long long>(f.terminalTick),
        static_cast<unsigned long long>(f.zeroReceiptTick), f.zeroReceipt ? 1U : 0U);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM55] event=LOG_FAILURE reason=FORMAT source=SIM motion=0 physicalPermit=0 discharge=0\n");
}

void NCManager::EndEDMGapServoRTSameThread(bool failed, const char* reason) noexcept
{
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U && !(failed && m_gapDryRun.result == 2U)) return;
    if (!m_edmGapServoRT.admissionPending)
    {
        m_motion.CancelEDMGapServoRT(m_edmGapServoRT.scope.session);
        m_motion.RetireEDMGapServoRT(m_edmGapServoRT.scope.session);
        m_edmGapServoRT.retirePending = !m_edmGapServoRT.retired;
    }
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = failed ? 3U : 2U;
    m_edmGapServoRT.terminalPublication = 0ULL;
    m_edmGapServoRT.terminalObservedTick = 0ULL;
    LogEDMGapServoRTSameThread(failed ? "FAIL" : "CANCEL", reason);
}

void NCManager::PauseEDMGapServoRTSameThread(const char* reason) noexcept
{
    if (!m_gapDryRun.active)
    {
        // HOLD must invalidate even a completed result before NC consumes M30.
        if (m_gapDryRun.result == 1U) EndEDMGapServoRTSameThread(false, "HOLD_AFTER_COMPLETE");
        return;
    }
    if (m_gapDryRun.paused) return;
    if (!m_edmGapServoRT.admissionPending)
    {
        m_motion.CancelEDMGapServoRT(m_edmGapServoRT.scope.session);
        m_motion.RetireEDMGapServoRT(m_edmGapServoRT.scope.session);
        m_edmGapServoRT.retirePending = !m_edmGapServoRT.retired;
    }
    m_edmGapServoRT.terminalPublication = 0ULL;
    m_edmGapServoRT.terminalObservedTick = 0ULL;
    m_edmGapServoRT.resumeWaitUs = 0ULL;
    if (!m_gapDryRun.paused) LogEDMGapServoRTSameThread("CANCEL", reason);
    m_gapDryRun.paused = true;
}

void NCManager::PrepareEDMGapServoAdmissionSameThread(std::uint64_t nowUs)
{
    const auto profile = m_edmGapServoRT.profile;
    m_edmGapServoRT = EDMGapServoRTControlState{};
    auto& control = m_edmGapServoRT;
    control.profile = profile;
    control.restartCount = m_gapDryRun.restarts;
    control.scope.session = m_gapDryRun.test;
    control.scope.run = m_gapDryRun.run;
    control.scope.cache = m_gapDryRun.cache;
    control.scope.dispatch = m_gapDryRun.dispatch;
    control.scope.executionEpoch = m_gapDryRun.epoch;
    control.scope.ownerLease = (static_cast<std::uint64_t>(m_gapDryRun.lease.owner) << 32U) |
        static_cast<std::uint64_t>(m_gapDryRun.lease.generation);
    control.startedUs = control.lastClockUs = control.lastFreshUs = nowUs;
    control.admissionPending = true;
    control.admissionStartUs = nowUs;
}

bool NCManager::RestartEDMGapServoRTSameThread(std::uint64_t nowUs)
{
    // Preserve the admission's seed; a current proof cannot be rebound to a
    // different epoch/lease between NC validation and the RT request.
    if (m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapDryRun.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)) return false;
    PrepareEDMGapServoAdmissionSameThread(nowUs);
    auto& control = m_edmGapServoRT;
    const auto profile = control.profile;
    control.admissionPending = false; // The transport attempt is issued below.
    if (!m_motion.StartEDMGapServoRT(control.scope, profile, nowUs))
    {
        // No admission/ACK was obtained. Retire the failed transport attempt;
        // do not wait forever for a scope that RT may never have accepted.
        m_motion.RetireEDMGapServoRT(control.scope.session);
        control.retired = true;
        return false;
    }
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 0U;
    LogEDMGapServoRTSameThread("BEGIN", "AWAIT_RT_ACK");
    return true;
}

WaitConditionFunc NCManager::StartEDMGapServoRTSameThread(const NCBlock& block)
{
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_edmPhysicalServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM69_RETIREMENT_PENDING"); return nullptr; }
    const int line = m_gapDryRun.sourceLine;
    const auto profile = block.val('P') == 34.0 ? EDM55RT::Profile::Stale : EDM55RT::Profile::Normal;
    const auto entryEpoch = m_motion.GetCurrentExecutionEpoch();
    const auto entryLease = m_programMotionLease;
    std::uint64_t mask = 0ULL;
    MotionStopSettleSnapshot diagnostic{};
    const bool ready = IsEDMGapServoRunReadySameThread(&mask, &diagnostic);
    if (m_gapDryRun.active || !IsGapDryRunBlockShapeValid(block) || !IsEDMZFixtureM30ContinuationSameThread())
    {
        LogEDMGapServoAdmissionSameThread(profile, mask | (1ULL << 12U), diagnostic, "BLOCK_SHAPE");
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM55_REQUIRES_STANDALONE_M30");
        return nullptr;
    }
    if (m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreLiveBookkeeping.currentRunToken == 0ULL || GetBaseProgramCache().GetGeneration() == 0ULL ||
        m_motion.GetCurrentExecutionEpoch() == MOTION_EXECUTION_EPOCH_INVALID) mask |= 1ULL << 10U;
    if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)()) mask |= 1ULL << 11U;
    // Only an unavailable paired RT readiness proof may wait. Every explicit
    // machine, queue, scope and safety prerequisite still rejects immediately.
    if ((mask & ~(1ULL << 9U)) != 0ULL)
    {
        LogEDMGapServoAdmissionSameThread(profile, mask, diagnostic, "NOT_READY");
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM55_ADMISSION_NOT_READY");
        return nullptr;
    }
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.runtimeServoTest = true;
    m_gapDryRun.active = true;
    m_gapDryRun.sourceLine = line;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = entryEpoch; m_gapDryRun.lease = entryLease;
    m_edmGapServoRT = EDMGapServoRTControlState{};
    m_edmGapServoRT.profile = profile;
    PrepareEDMGapServoAdmissionSameThread(0ULL);
    LARGE_INTEGER frequency{};
    std::uint64_t nowUs = 0ULL;
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    {
        LogEDMGapServoAdmissionSameThread(profile, mask | (1ULL << 13U), diagnostic, "CLOCK_FREQUENCY");
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM55_CLOCK_FREQUENCY");
        return nullptr;
    }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    if (!ReadEDMGapServoClockSameThread(nowUs))
    {
        LogEDMGapServoAdmissionSameThread(profile, mask | (1ULL << 13U), diagnostic, "CLOCK_READ");
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM55_CLOCK_READ");
        return nullptr;
    }
    PrepareEDMGapServoAdmissionSameThread(nowUs);
    if (!ready || m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapDryRun.lease.Matches(m_programMotionLease) || !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease))
    {
        if (ready) mask |= 1ULL << 10U;
        LogEDMGapServoAdmissionSameThread(profile, mask, diagnostic, ready ? "WAIT_IDENTITY" : "WAIT_RT_READINESS");
        return &NCManager::WaitForGapDryRunCallback;
    }
    LogEDMGapServoAdmissionSameThread(profile, mask, diagnostic, "READY");
    if (!RestartEDMGapServoRTSameThread(nowUs))
    {
        LogEDMGapServoAdmissionSameThread(profile, mask, diagnostic, "RT_START_REJECTED");
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM55_RT_START_REJECTED");
        return nullptr;
    }
    return &NCManager::WaitForGapDryRunCallback;
}

void NCManager::ServiceEDMGapServoRetirementSameThread() noexcept
{
    ServiceEDMPhysicalServoRetirementSameThread();
    auto& control = m_edmGapServoRT;
    if (!control.retirePending || control.retired || !control.scope.session) return;
    EDM55RT::Feedback feedback{};
    // This frozen value proves transport retirement only, never physical
    // standstill or M30 permission. Cleanup must work even if NC QPC failed.
    if (!m_motion.ReadEDMGapServoRT(feedback) ||
        !EDM55RT::SameScope(feedback.scope, control.scope) || feedback.profile != control.profile ||
        feedback.latestSession != control.scope.session || feedback.cancelThroughSession < control.scope.session ||
        !feedback.publication || !feedback.rtTick || !feedback.zeroReceipt || !feedback.terminalTick ||
        feedback.zeroReceiptTick <= feedback.terminalTick || feedback.rtTick < feedback.zeroReceiptTick ||
        feedback.speedMmMin != 0.0 || feedback.targetMmMin != 0.0 ||
        feedback.state != EDM55RT::State::Cancelled) return;
    control.feedback = feedback;
    control.retired = true;
    control.retirePending = false;
    LogEDMGapServoRTSameThread("READBACK", "CANCEL_ZERO_RETIRED");
}

void NCManager::ValidateEDMGapServoRTSameThread()
{
    ServiceEDMGapServoRetirementSameThread();
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return;
    if (!IsEDMGapServoScopeCurrentSameThread(true))
    {
        EndEDMGapServoRTSameThread(false, "AUTHORITY_CHANGED");
        return;
    }
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    {
        PauseEDMGapServoRTSameThread("HOLD");
        return;
    }
    if (m_gapDryRun.paused) return;
    if (m_edmGapServoRT.admissionPending)
    {
        // ProcessExecutionEngine may skip the wait callback after authority
        // loss. Age/cancel this unissued admission here, before those returns.
        // This observer never issues an RT request before callback binding.
        auto& control = m_edmGapServoRT;
        std::uint64_t nowUs = 0ULL;
        if (!ReadEDMGapServoClockSameThread(nowUs))
        {
            std::uint64_t mask = 0ULL; MotionStopSettleSnapshot diagnostic{};
            (void)IsEDMGapServoRunReadySameThread(&mask, &diagnostic);
            LogEDMGapServoAdmissionSameThread(control.profile, mask | (1ULL << 13U), diagnostic, "WAIT_CLOCK_INVALID");
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_ADMISSION_CLOCK_INVALID");
            return;
        }
        if (nowUs == control.admissionCheckUs) ++control.admissionStalledPolls;
        else { control.admissionCheckUs = nowUs; control.admissionStalledPolls = 0U; }
        const bool scope = IsEDMGapServoScopeCurrentSameThread(false);
        if (!scope && !control.authorityWaitUs) control.authorityWaitUs = nowUs;
        if (scope) control.authorityWaitUs = 0ULL;
        const bool identityExpired = !scope && nowUs - control.authorityWaitUs > 250000ULL;
        const bool expired = nowUs < control.admissionStartUs || nowUs - control.admissionStartUs > 1000000ULL;
        if (identityExpired || expired || control.admissionStalledPolls >= 4096U)
        {
            std::uint64_t mask = 0ULL; MotionStopSettleSnapshot diagnostic{};
            (void)IsEDMGapServoRunReadySameThread(&mask, &diagnostic);
            if (!scope) mask |= 1ULL << 10U;
            if (control.admissionStalledPolls >= 4096U) mask |= 1ULL << 13U;
            const char* reason = identityExpired ? "EDM55_ADMISSION_IDENTITY_CHANGED" :
                control.admissionStalledPolls >= 4096U ? "EDM55_ADMISSION_CLOCK_STALLED" : "EDM55_ADMISSION_TIMEOUT";
            LogEDMGapServoAdmissionSameThread(control.profile, mask, diagnostic, reason);
            RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, reason);
        }
        return;
    }
    if (m_edmGapServoRT.retirePending || m_edmGapServoRT.retired) return;
    // The RT owner may observe HOLD's revoked lease before the NC input pass.
    // Do not renew authority or classify that edge as a machining fault here.
    if (!IsEDMGapServoScopeCurrentSameThread(false) || !IsEDMGapServoRunReadySameThread()) return;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) ||
        !m_motion.TouchEDMGapServoRT(m_edmGapServoRT.scope.session, nowUs))
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_HEARTBEAT_REJECTED");
}

bool NCManager::ProcessEDMGapServoRTSameThread()
{
    ValidateEDMGapServoRTSameThread();
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && IsEDMGapServoScopeCurrentSameThread(false)) return true;
        if (m_gapDryRun.result == 2U && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_CANCELLED_CALLBACK");
        return false;
    }
    if (m_state != NCState::RUN || m_edmState == EDMState::HOLD) return false;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_CLOCK");
        return false;
    }
    auto& control = m_edmGapServoRT;
    if (nowUs == control.lastClockUs)
    {
        if (++control.stalledPolls >= 4096U)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_CLOCK_STALLED");
        return false;
    }
    control.stalledPolls = 0U;
    control.lastClockUs = nowUs;
    if (control.admissionPending)
    {
        if (m_gapDryRun.paused)
        {
            // HOLD preceded any RT request. There is no cancellation receipt
            // to wait for; a fresh START creates a new bounded admission.
            if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ||
                m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RESTART_EXHAUSTED");
                return false;
            }
            ++m_gapDryRun.restarts;
            m_gapDryRun.test = ++m_gapDryRunSerial;
            m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
            PrepareEDMGapServoAdmissionSameThread(nowUs);
            m_gapDryRun.paused = false;
        }
        std::uint64_t mask = 0ULL;
        MotionStopSettleSnapshot diagnostic{};
        const bool ready = IsEDMGapServoRunReadySameThread(&mask, &diagnostic);
        if (!IsEDMGapServoScopeCurrentSameThread(false)) return false; // Validate owns the bounded HOLD-edge grace.
        if ((mask & ~(1ULL << 9U)) != 0ULL)
        {
            LogEDMGapServoAdmissionSameThread(control.profile, mask, diagnostic, "WAIT_AUTHORITY_CHANGED");
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_ADMISSION_AUTHORITY_CHANGED");
            return false;
        }
        if (nowUs < control.admissionStartUs || nowUs - control.admissionStartUs > 1000000ULL)
        {
            LogEDMGapServoAdmissionSameThread(control.profile, mask, diagnostic, "WAIT_TIMEOUT");
            RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM55_ADMISSION_TIMEOUT");
            return false;
        }
        if (!ready) return false;
        LogEDMGapServoAdmissionSameThread(control.profile, mask, diagnostic, "READY_AFTER_WAIT");
        if (!RestartEDMGapServoRTSameThread(nowUs))
            RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM55_RT_START_REJECTED");
        return false;
    }
    if (!m_gapDryRun.paused &&
        (!IsEDMGapServoScopeCurrentSameThread(false) || !IsEDMGapServoRunReadySameThread()))
    {
        if (!control.authorityWaitUs) control.authorityWaitUs = nowUs;
        if (nowUs - control.authorityWaitUs > 250000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_AUTHORITY_NOT_RESTORED");
        return false;
    }
    control.authorityWaitUs = 0ULL;
    EDM55RT::Feedback feedback{};
    const bool captured = m_motion.ReadEDMGapServoRT(feedback);
    // Stamp the consumer after its read; a concurrently published RT sample
    // must not appear to be from the future merely because it won this race.
    if (!ReadEDMGapServoClockSameThread(nowUs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_READBACK_CLOCK");
        return false;
    }
    const bool read = captured &&
        EDM55RT::SameScope(feedback.scope, control.scope) && feedback.profile == control.profile &&
        feedback.publication != 0ULL && feedback.rtTick != 0ULL && feedback.nowUs != 0ULL &&
        feedback.nowUs <= nowUs && nowUs - feedback.nowUs <= 100000ULL;
    if (m_gapDryRun.paused)
    {
        // RT may have seen the revoked owner before NC saw HOLD. Either
        // cancellation or a fault may retire it, but a new session is mandatory.
        if (!control.resumeWaitUs) control.resumeWaitUs = nowUs;
        const bool retired = control.retired;
        if (!retired)
        {
            if (nowUs - control.resumeWaitUs > 1000000ULL)
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_CANCEL_RECEIPT_TIMEOUT");
            return false;
        }
        const auto resumeEpoch = m_motion.GetCurrentExecutionEpoch();
        const auto resumeLease = m_programMotionLease;
        if (!IsEDMGapServoRunReadySameThread() || !IsEDMGapServoScopeCurrentSameThread(true) ||
            resumeEpoch != m_motion.GetCurrentExecutionEpoch() || !resumeLease.Matches(m_programMotionLease) ||
            !m_motion.IsMotionOwnerLeaseCurrent(resumeLease))
        {
            if (nowUs - control.resumeWaitUs > 1000000ULL)
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RESUME_NOT_READY");
            return false;
        }
        if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ||
            m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RESTART_EXHAUSTED");
            return false;
        }
        ++m_gapDryRun.restarts;
        m_gapDryRun.test = ++m_gapDryRunSerial;
        m_gapDryRun.epoch = resumeEpoch; m_gapDryRun.lease = resumeLease;
        if (!RestartEDMGapServoRTSameThread(nowUs))
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RESTART_REJECTED");
        return false;
    }
    if (!read || feedback.publication <= control.feedback.publication)
    {
        if (nowUs - control.lastFreshUs > 250000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_FEEDBACK_STALE");
        return false;
    }
    if (control.feedback.rtTick != 0ULL && feedback.rtTick <= control.feedback.rtTick)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RT_TICK_REGRESSION");
        return false;
    }
    control.feedback = feedback;
    control.lastFreshUs = nowUs;
    if (!feedback.startAcknowledged)
    {
        if (nowUs - control.startedUs > 1000000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_START_ACK_TIMEOUT");
        return false;
    }
    if (!control.ackLogged)
    {
        control.ackLogged = true;
        LogEDMGapServoRTSameThread("READBACK", "RT_START_ACK");
    }
    if (feedback.stage != control.loggedStage)
    {
        control.loggedStage = feedback.stage;
        LogEDMGapServoRTSameThread("PHASE", "RT_OBSERVED");
    }
    const bool expectedStale = control.profile == EDM55RT::Profile::Stale &&
        feedback.state == EDM55RT::State::Fault && feedback.reason == EDM55RT::Reason::SourceStale && feedback.expectedFault;
    const bool completed = control.profile == EDM55RT::Profile::Normal && feedback.state == EDM55RT::State::Completed &&
        feedback.reason == EDM55RT::Reason::None && !feedback.expectedFault &&
        feedback.coverageMask == EDM55RT::RequiredCoverage && feedback.stageMask == 0x1ffULL;
    if (feedback.state == EDM55RT::State::Cancelled || (feedback.state == EDM55RT::State::Fault && !expectedStale) ||
        (feedback.state == EDM55RT::State::Completed && !completed))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_UNEXPECTED_RT_TERMINAL");
        return false;
    }
    if (completed || expectedStale)
    {
        const bool zero = feedback.zeroReceipt && feedback.startedTick != 0ULL && feedback.startedUs != 0ULL &&
            feedback.terminalTick > feedback.startedTick && feedback.runtimeSteps != 0ULL &&
            feedback.freshSamples != 0ULL && feedback.sourceSequence != 0ULL &&
            feedback.zeroReceiptTick > feedback.terminalTick && feedback.rtTick >= feedback.zeroReceiptTick &&
            feedback.speedMmMin == 0.0 && feedback.targetMmMin == 0.0;
        if (!zero)
        {
            if (nowUs - control.startedUs > 20000000ULL)
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_ZERO_RECEIPT_TIMEOUT");
            return false;
        }
        if (control.terminalPublication == 0ULL)
        {
            control.terminalPublication = feedback.publication;
            control.terminalObservedTick = feedback.rtTick;
            control.terminalIdentityTick = feedback.terminalTick;
            control.zeroIdentityTick = feedback.zeroReceiptTick;
            LogEDMGapServoRTSameThread("READBACK", expectedStale ? "EXPECTED_STALE_ZERO" : "RT_COMPLETED_ZERO");
            return false;
        }
        if (feedback.terminalTick != control.terminalIdentityTick || feedback.zeroReceiptTick != control.zeroIdentityTick)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_TERMINAL_IDENTITY_CHANGED");
            return false;
        }
        if (feedback.publication > control.terminalPublication && feedback.rtTick > control.terminalObservedTick &&
            IsEDMGapServoScopeCurrentSameThread(false) && IsEDMGapServoRunReadySameThread())
        {
            m_motion.RetireEDMGapServoRT(control.scope.session);
            control.retirePending = true;
            m_gapDryRun.active = false;
            m_gapDryRun.result = 1U;
            LogEDMGapServoRTSameThread("PASS", expectedStale ? "EXPECTED_STALE_LATER_ZERO" : "COMPLETE_LATER_ZERO");
            return true;
        }
        return false;
    }
    // This watchdog can only fail. NC elapsed time can never manufacture PASS.
    if (nowUs - control.startedUs > 20000000ULL)
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM55_RT_COMPLETION_TIMEOUT");
    return false;
}

// EDM69 owns a live, explicitly cancellable physical-source RT calculation.
// It has no axis output/arm path; terminal zero proves this consumer only.


bool NCManager::IsEDMPhysicalServoScopeCurrentSameThread() const noexcept
{
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    return m_gapDryRun.physicalServoRTTest && !(mask & ~(1ULL << 9U)) &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease);
}

void NCManager::EndEDMPhysicalServoRTSameThread(bool failed, const char* reason) noexcept
{
    // ChangeState(ALARM) re-enters generic cancellation; never downgrade FAIL.
    if (!m_gapDryRun.active && m_gapDryRun.result == 3U) return;
    auto& control = m_edmPhysicalServoRT;
    const bool report = m_gapDryRun.active || (failed && m_gapDryRun.result != 3U);
    // Atomic cancellation precedes console IO and NC result bookkeeping.
    if (control.issued && !control.retired)
    {
        m_motion.CancelEDMPhysicalServoRT(control.request.capture.scope.session);
        if (!control.retirePending)
        {
            control.retirePending = true; control.retireStartedUs = control.lastClockUs;
            control.retirePolls = 0U;
        }
    }
    m_gapDryRun.active = false; m_gapDryRun.paused = false;
    m_gapDryRun.result = failed ? 3U : 2U;
    if (report)
        RtPrintf("[EDM69] event=SUMMARY result=%s reason=%.64s P=55 session=%llu retainedCredit=0 next=RESET_THEN_EXPLICIT_P55 consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
            failed ? "FAIL" : "CANCELLED", reason ? reason : "UNKNOWN",
            static_cast<unsigned long long>(control.request.capture.scope.session));
}

void NCManager::ServiceEDMPhysicalServoRetirementSameThread() noexcept
{
    auto& control = m_edmPhysicalServoRT;
    if (!control.retirePending || control.retired || !control.issued) return;
    EDM69::Feedback feedback{};
    const bool captured = m_motion.ReadEDMPhysicalServoRT(feedback);
    std::uint64_t nowUs = 0ULL;
    const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
    // During recovery a failed NC clock cannot erase an exact later RT zero
    // receipt. This proves channel retirement only, never physical standstill.
    const bool fresh = !clockValid || (feedback.nowUs && feedback.nowUs <= nowUs &&
        nowUs - feedback.nowUs <= EDM69::HeartbeatMaximumAgeUs);
    if (captured && fresh && EDM69::SameRequest(feedback.request, control.request) &&
        feedback.latestSession == control.request.capture.scope.session &&
        feedback.cancelThroughSession >= control.request.capture.scope.session &&
        feedback.publication && feedback.rtTick && feedback.terminalTick && feedback.zeroReceipt &&
        feedback.zeroReceiptTick > feedback.terminalTick && feedback.rtTick >= feedback.zeroReceiptTick &&
        feedback.output.targetMmPerMin == 0.0 && feedback.output.speedMmPerMin == 0.0 &&
        !feedback.physicalPermit && !feedback.discharge &&
        (feedback.state == EDM69::State::Cancelled || feedback.state == EDM69::State::Fault))
    {
        control.feedback = feedback; control.retired = true; control.retirePending = false;
        RtPrintf("[EDM69] event=RETIRED session=%llu terminalTick=%llu zeroTick=%llu consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(control.request.capture.scope.session),
            static_cast<unsigned long long>(feedback.terminalTick), static_cast<unsigned long long>(feedback.zeroReceiptTick));
        return;
    }
    if (control.retirePolls < 4096U) ++control.retirePolls;
    const bool expired = clockValid && nowUs >= control.retireStartedUs &&
        nowUs - control.retireStartedUs >= 1000000ULL;
    if (!control.retireTimeoutReported && (expired || control.retirePolls >= 4096U))
    {
        control.retireTimeoutReported = true;
        RtPrintf("[EDM69] event=RETIREMENT_TIMEOUT result=FAIL session=%llu pending=1 consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(control.request.capture.scope.session));
        if (!AlarmManager::GetInstance().HasAlarm())
        { AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine); ChangeState(NCState::ALARM); }
    }
}

void NCManager::ValidateEDMPhysicalServoRTSameThread()
{
    ServiceEDMPhysicalServoRetirementSameThread();
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return;
    // Classify HOLD locally before observing the RT owner's authority fault.
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMPhysicalServoRTSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMPhysicalServoScopeCurrentSameThread())
    { EndEDMPhysicalServoRTSameThread(false, "NC_SCOPE_REVOKED"); return; }
    auto& control = m_edmPhysicalServoRT;
    // Read only the already-bound owner-thread recipe while active. The idle
    // summary reader deliberately rejects active diagnostics and stays intact.
    const auto binding = EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
        m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease) || !binding.ready ||
        !EDM59::SameRecipe(binding.recipe, control.request.capture.recipe))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_LIVE_RECIPE_CHANGED"); return; }
    if (control.completed && m_gapDryRun.result == 1U) return;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.submittedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_CLOCK_INVALID"); return; }
    if (nowUs == control.lastClockUs) ++control.stalledPolls;
    else { control.lastClockUs = nowUs; control.stalledPolls = 0U; }
    if (control.stalledPolls >= 4096U || nowUs - control.submittedUs >= EDM69::TotalDeadlineUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        control.stalledPolls >= 4096U ? "EDM69_CLOCK_STALLED" : "EDM69_TOTAL_TIMEOUT"); return; }
    if (!m_motion.TouchEDMPhysicalServoRT(control.request.capture.scope.session, nowUs))
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_HEARTBEAT_REJECTED");
}

bool NCManager::ProcessEDMPhysicalServoRTSameThread()
{
    ValidateEDMPhysicalServoRTSameThread();
    auto& control = m_edmPhysicalServoRT;
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && control.completed && IsEDMPhysicalServoScopeCurrentSameThread()) return true;
        if (m_gapDryRun.result == 2U && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_CANCELLED_CALLBACK_RESET_REQUIRED");
        return false;
    }
    EDM69::Feedback feedback{};
    const bool captured = m_motion.ReadEDMPhysicalServoRT(feedback);
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_READBACK_CLOCK"); return false; }
    const bool matching = captured && EDM69::SameRequest(feedback.request, control.request) &&
        feedback.latestSession == control.request.capture.scope.session;
    const bool fresh = matching && feedback.publication && feedback.rtTick && feedback.nowUs &&
        feedback.qpcFrequency == control.request.capture.qpcFrequency && feedback.nowUs <= nowUs &&
        nowUs - feedback.nowUs <= EDM69::HeartbeatMaximumAgeUs;
    if (!fresh)
    {
        if ((captured && feedback.latestSession > control.request.capture.scope.session) ||
            (!control.feedback.startAcknowledged && nowUs - control.submittedUs >= EDM69::StartDeadlineUs) ||
            (control.feedback.startAcknowledged && nowUs - control.lastFreshUs >= EDM69::HeartbeatMaximumAgeUs))
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_RECEIPT_TIMEOUT_OR_SCOPE");
        return false;
    }
    if (feedback.publication < control.feedback.publication || feedback.rtTick < control.feedback.rtTick ||
        feedback.physicalPermit || feedback.discharge ||
        feedback.state == EDM69::State::Fault || feedback.state == EDM69::State::Cancelled)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        feedback.state == EDM69::State::Fault ? EDM69::ReasonName(feedback.reason) : "EDM69_RECEIPT_REVOKED"); return false; }
    const bool advanced = feedback.publication > control.feedback.publication && feedback.rtTick > control.feedback.rtTick;
    if (advanced) control.lastFreshUs = nowUs;
    control.feedback = feedback;
    if (!feedback.startAcknowledged && nowUs - control.submittedUs >= EDM69::StartDeadlineUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_START_ACK_TIMEOUT"); return false; }
    if (!advanced)
    {
        if (nowUs - control.lastFreshUs >= EDM69::HeartbeatMaximumAgeUs)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_FROZEN_FEEDBACK");
        return false;
    }
    if (feedback.state != EDM69::State::Completed) return false;
    auto identity = control.request.physicalIdentity; identity.session = control.request.capture.scope.session;
    const auto& coverage = feedback.coverage;
    const bool validTerminal = feedback.startAcknowledged && feedback.zeroReceipt && feedback.terminalTick &&
        feedback.zeroReceiptTick > feedback.terminalTick && feedback.rtTick >= feedback.zeroReceiptTick &&
        feedback.output.targetMmPerMin == 0.0 && feedback.output.speedMmPerMin == 0.0 &&
        EDM55::SameIdentity(feedback.kernelIdentity, identity) &&
        coverage.hasSamples && !coverage.saturated && coverage.freshSamples >= 2ULL &&
        coverage.validTicks == feedback.runtimeSteps && coverage.freshSamples <= coverage.validTicks &&
        feedback.firstSourceCapturedUs > control.request.baselineCapturedUs &&
        feedback.output.capturedAtUs >= feedback.firstSourceCapturedUs &&
        feedback.output.capturedAtUs - feedback.firstSourceCapturedUs >= EDM69::DurationUs &&
        feedback.sourceCapturedUs >= feedback.output.capturedAtUs &&
        coverage.lastObservedUs >= coverage.firstObservedUs &&
        coverage.lastObservedUs - coverage.firstObservedUs >= EDM69::DurationUs &&
        coverage.firstSourceSequence > control.request.baselineSequence &&
        coverage.lastSourceSequence >= coverage.firstSourceSequence &&
        feedback.output.sequence == coverage.lastSourceSequence &&
        feedback.sourceSequence >= coverage.lastSourceSequence && feedback.minimumStepUs &&
        feedback.maximumStepUs <= control.request.maxTickGapUs &&
        feedback.maximumSourceAgeUs <= control.request.maxAgeUs &&
        std::isfinite(feedback.maximumTargetError) && feedback.maximumTargetError <= 1e-8;
    if (!validTerminal)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_TERMINAL_PROOF_INVALID"); return false; }
    if (!control.terminalPublication)
    {
        control.terminalPublication = feedback.publication; control.terminalObservedTick = feedback.rtTick;
        control.terminalIdentityTick = feedback.terminalTick; control.zeroIdentityTick = feedback.zeroReceiptTick;
        return false;
    }
    if (feedback.terminalTick != control.terminalIdentityTick || feedback.zeroReceiptTick != control.zeroIdentityTick)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM69_TERMINAL_IDENTITY_CHANGED"); return false; }
    if (feedback.publication <= control.terminalPublication || feedback.rtTick <= control.terminalObservedTick) return false;
    // Completion is a stored two-read proof. Retire asynchronously before logs;
    // an outer completion poll can retain this proof without relabelling Cancelled.
    m_motion.CancelEDMPhysicalServoRT(control.request.capture.scope.session);
    control.retirePending = true; control.retireStartedUs = nowUs; control.retirePolls = 0U;
    control.completed = true; m_gapDryRun.active = false; m_gapDryRun.result = 1U;
    char text[480]{};
    int size = std::snprintf(text, sizeof(text),
        "[EDM69] event=PASS result=PASS P=55 session=%llu observedMs=%llu steps=%llu fresh=%llu E8=%.6f E9=%.6f terminalTick=%llu zeroTick=%llu targetCheck=PASS consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(control.request.capture.scope.session),
        static_cast<unsigned long long>((feedback.output.capturedAtUs-feedback.firstSourceCapturedUs)/1000ULL),
        static_cast<unsigned long long>(feedback.runtimeSteps), static_cast<unsigned long long>(coverage.freshSamples),
        control.request.capture.recipe.overridePercent, control.request.capture.recipe.referenceV,
        static_cast<unsigned long long>(feedback.terminalTick), static_cast<unsigned long long>(feedback.zeroReceiptTick));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    size = std::snprintf(text, sizeof(text),
        "[EDM69] event=SOURCE session=%llu source=PHYSICAL sourceId=%llu sourceGen=%llu calibration=%llu firstSeq=%llu lastSeq=%llu maxSourceAgeUs=%llu minStepUs=%llu maxStepUs=%llu scope=MOTION_RT_CONSUMER consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(control.request.capture.scope.session),
        static_cast<unsigned long long>(identity.sourceId), static_cast<unsigned long long>(identity.sourceGeneration),
        static_cast<unsigned long long>(identity.calibrationRevision), static_cast<unsigned long long>(coverage.firstSourceSequence),
        static_cast<unsigned long long>(coverage.lastSourceSequence), static_cast<unsigned long long>(feedback.maximumSourceAgeUs),
        static_cast<unsigned long long>(feedback.minimumStepUs), static_cast<unsigned long long>(feedback.maximumStepUs));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    size = std::snprintf(text, sizeof(text),
        "[EDM69] event=RANGE session=%llu rawMinV=%.6f rawMaxV=%.6f advanceTicks=%llu retreatTicks=%llu shortTicks=%llu shortEntries=%llu shortExits=%llu maxTargetError=%.9g scope=MOTION_RT_CONSUMER consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(control.request.capture.scope.session), coverage.minRawV, coverage.maxRawV,
        static_cast<unsigned long long>(coverage.advanceTicks), static_cast<unsigned long long>(coverage.retreatTicks),
        static_cast<unsigned long long>(coverage.shortTicks), static_cast<unsigned long long>(coverage.shortEntries),
        static_cast<unsigned long long>(coverage.shortExits), feedback.maximumTargetError);
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    return true;
}

// EDM67 owns only an NC-side receipt for an independent PDO diagnostic.
// It never starts, cancels, rearms or acknowledges axis motion.


void NCManager::EndEDMPhysicalRecipeCaptureSameThread(bool failed, const char* reason) noexcept
{
    if (!m_gapDryRun.active && m_gapDryRun.result == 3U) return;
    const bool report = m_gapDryRun.active || failed || m_gapDryRun.result == 0U;
    m_gapDryRun.active = m_gapDryRun.paused = false;
    m_gapDryRun.result = failed ? 3U : 2U;
    m_edmPhysicalRecipeCapture.receipt.Cancel();
    m_edmPhysicalRecipeWindow.Cancel();
    // This cancels only the NC receipt. The already-published immutable
    // shadow recipe and its history continue until an explicit new capture.
    if (report)
        RtPrintf("[%s] event=SUMMARY result=%s reason=%.64s test=%llu session=%llu observerRetained=1 resetRequired=1 motion=0 physicalPermit=0 discharge=0\n",
            m_gapDryRun.physicalRecipeWindowTest ? "EDM68" : "EDM67", failed ? "FAIL" : "CANCELLED", reason ? reason : "UNKNOWN",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.physicalRecipeWindowTest ?
                m_edmPhysicalRecipeWindow.Request().scope.session : m_edmPhysicalRecipeCapture.receipt.Request().scope.session));
    if (report && m_gapDryRun.physicalRecipeWindowTest)
    {
        const auto& window = m_edmPhysicalRecipeWindow.Report();
        RtPrintf("[EDM68] event=WINDOW_CANCELLED session=%llu observedMs=%llu reads=%llu pdoDelta=%llu retainedCredit=0 next=RESET_THEN_EXPLICIT_P54 motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_edmPhysicalRecipeWindow.Request().scope.session),
            static_cast<unsigned long long>(window.durationUs / 1000ULL),
            static_cast<unsigned long long>(window.freshObservations), static_cast<unsigned long long>(window.sourcePdoDelta));
    }
}

void NCManager::ValidateEDMPhysicalRecipeCaptureSameThread()
{
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMPhysicalRecipeCaptureSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMShortFlushRunReadySameThread() ||
        m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        m_gapDryRun.dispatch != m_waitingBlockDispatchId ||
        m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapDryRun.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_isG66Active || !m_macroStack.empty())
    { EndEDMPhysicalRecipeCaptureSameThread(false, "NC_SCOPE_REVOKED"); return; }
    if (m_gapDryRun.physicalRecipeWindowTest)
    {
        // An external recipe/tuning edit cannot silently replace a running
        // diagnostic's frozen configuration. A new explicit P54 is required.
        // ReadEDMRecipeSummarySameThread is an idle/admission reader and
        // deliberately rejects active diagnostics. Keep that guard intact;
        // verify the already-bound recipe authority without rebinding it.
        const auto binding = EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
        if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
            m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
            m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease) || !binding.ready ||
            !EDM59::SameRecipe(binding.recipe, m_edmPhysicalRecipeWindow.Request().recipe))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM68_LIVE_RECIPE_CHANGED"); return; }
    }
    auto& control = m_edmPhysicalRecipeCapture;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.submittedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, (m_gapDryRun.physicalRecipeWindowTest ? "EDM68_CLOCK_INVALID" : "EDM67_CLOCK_INVALID")); return; }
    if (nowUs == control.lastClockUs) ++control.stalledPolls;
    else { control.lastClockUs = nowUs; control.stalledPolls = 0U; }
    const auto deadline = m_gapDryRun.physicalRecipeWindowTest ? EDM68::TotalDeadlineUs : EDM67::ReceiptDeadlineUs;
    if (nowUs - control.submittedUs >= deadline || control.stalledPolls >= EDM67::MaximumStalledCalls)
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            control.stalledPolls >= EDM67::MaximumStalledCalls ?
                (m_gapDryRun.physicalRecipeWindowTest ? "EDM68_CLOCK_STALLED" : "EDM67_CLOCK_STALLED") :
            m_gapDryRun.physicalRecipeWindowTest ? "EDM68_WINDOW_TIMEOUT" : "EDM67_CAPTURE_TIMEOUT");
}



// Called only after the shared NC authority/recipe validation and master check.
// P54 uses source time and fresh evidence, never just the NC's elapsed timer.


// P13/P14/P15/P16 own only a bounded, finite-position diagnostic channel. Their receipts
// are coherent RT observations; no APPLIED mailbox result proves motion done.
// EDM56: NC owns admission/lifecycle only. The 250 us RT adapter owns all
// synthetic voltage, servo calculations and continuous physical references.


void NCManager::LogEDMGapDynamicAdmissionSameThread(std::uint64_t mask,
    const MotionStopSettleSnapshot& raw, const char* reason) noexcept
{
    if (EDM28::IsPhysicalServoProfile(m_edmZFixture.config.profile))
    {
        // The first Arm also starts the physical profile's 50 ms heartbeat deadline.
        // Never put console output between submission and its RT receipt.
        if (m_edmGapDynamic.armIssued) return;
        RtPrintf("[%s] event=ADMISSION reason=%.32s P=%u session=%llu mask=0x%llX source=PHYSICAL axis=Z armIssued=0 discharge=0\n",
            EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(m_edmZFixture.config.profile) ? "EDM76" : EDM28::IsPhysicalBidirectionalFlushFamilyProfile(m_edmZFixture.config.profile) ? "EDM75" : EDM28::IsPhysicalBidirectionalProfile(m_edmZFixture.config.profile) ? "EDM74" :
            EDM28::IsPhysicalRetreatCycleProfile(m_edmZFixture.config.profile) ? "EDM73" :
            EDM28::IsPhysicalRetreatProfile(m_edmZFixture.config.profile) ? "EDM72" :
            EDM28::IsPhysicalServoFlushProfile(m_edmZFixture.config.profile) ? "EDM71" : "EDM70",
            reason ? reason : "UNKNOWN", static_cast<unsigned>(m_edmZFixture.config.profile),
            static_cast<unsigned long long>(m_edmZFixture.scope.session),
            static_cast<unsigned long long>(mask));
        return;
    }
    // EDM66-FIX2: admission diagnostics are allowed only before Arm. Once
    // submitted, console latency must not consume the application deadline.
    if (EDM28::IsGapServoFlushProfile(m_edmZFixture.config.profile) &&
        m_edmGapDynamic.armIssued) return;
    const bool flush = EDM28::IsGapFlushProfile(m_edmZFixture.config.profile);
    const char* build = EDM28::IsGapServoFlushProfile(m_edmZFixture.config.profile) ? "EDM65" : EDM28::IsGapConditionServoProfile(m_edmZFixture.config.profile) ? "EDM59" : EDM57::IsBoundProfile(static_cast<unsigned>(m_edmZFixture.config.profile)) ? "EDM58" : flush ? "EDM57" : "EDM56";
    char line[480]{};
    const int n = std::snprintf(line, sizeof(line),
        "[%s] event=ADMISSION build=%s reason=%.32s P=%u session=%llu mask=0x%llX formal=%u rawDiagOnly=1 rawStand=%u nonIdle=%u cmdMove=%u actMove=%u source=SIM axis=Z discharge=0\n",
        build, build, reason ? reason : "UNKNOWN", static_cast<unsigned>(m_edmZFixture.config.profile),
        static_cast<unsigned long long>(m_edmZFixture.scope.session), static_cast<unsigned long long>(mask),
        (mask & (1ULL << 9U)) ? 0U : 1U, raw.standstill ? 1U : 0U,
        raw.nonIdleAxisCount, raw.commandMovingAxisCount, raw.actualMovingAxisCount);
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else if (flush) RtPrintf("[%s] event=ADMISSION reason=LOG_FORMAT discharge=0\n", build);
    else RtPrintf("[EDM56] event=ADMISSION reason=LOG_FORMAT discharge=0\n");
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    if (!f.latchedFault) return;
    // Admission entry/timeout only, never the polling path. These current
    // readback facts distinguish a retired receipt from proof renewal or a
    // real fault without changing RT state or manufacturing readiness.
    const int receipt = std::snprintf(line, sizeof(line),
        "[%s] event=RETIRED_FAULT detail=%.32s part=RECEIPT session=%llu oldSession=%llu rtReason=%u state=%u cap=%u origin=%u stopLatch=%u zero=%u retired=%u stop=%llu disarm=%llu kind=%u discharge=0\n",
        build, reason ? reason : "UNKNOWN", static_cast<unsigned long long>(s.scope.session),
        static_cast<unsigned long long>(f.scope.session), static_cast<unsigned>(f.reason), static_cast<unsigned>(f.state),
        f.capHeld ? 1U : 0U, f.originValid ? 1U : 0U, f.stopLatched ? 1U : 0U, f.forceZeroOutput ? 1U : 0U,
        IsEDMGapDynamicRetiredFaultSameThread() ? 1U : 0U, static_cast<unsigned long long>(f.stopAppliedSequence),
        static_cast<unsigned long long>(f.lastAppliedSequence), static_cast<unsigned>(f.lastAppliedKind));
    if (receipt >= 0 && static_cast<std::size_t>(receipt) < sizeof(line)) RtPrintf("%s", line);
    const double pulsePerMm = std::isfinite(f.pulsePerMm) && f.pulsePerMm > 0.0 ? f.pulsePerMm : 1.0;
    const int proof = std::snprintf(line, sizeof(line),
        "[%s] event=RETIRED_FAULT detail=%.32s part=PROOF tick=%llu fresh=%u clock=%u pdoValid=%u frame=%u idle=%u armReady=%u stopProven=%u stable=%u proof=%llu cmdMmS=%.9g pdoMmS=%.9g commandErrorMm=%.9g planningErrorMm=%.9g discharge=0\n",
        build, reason ? reason : "UNKNOWN", static_cast<unsigned long long>(f.sampledTick),
        f.sourceFresh ? 1U : 0U, f.clockValid ? 1U : 0U, f.pdoValid ? 1U : 0U, f.frameCurrent ? 1U : 0U,
        f.axisIdle ? 1U : 0U, f.armReady ? 1U : 0U, f.stopProven ? 1U : 0U, f.stableCycles,
        static_cast<unsigned long long>(f.proofSequence), f.cmdSpeedMmS, f.pdoSpeedMmS,
        (f.actualPulse - f.commandPulse) / pulsePerMm, (f.actualPulse - f.planningPulse) / pulsePerMm);
    if (proof >= 0 && static_cast<std::size_t>(proof) < sizeof(line)) RtPrintf("%s", line);
}

void NCManager::LogEDMGapDynamicSameThread(const char* event, const char* result, const char* reason) noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& r = f.dynamic;
    if (EDM28::IsPhysicalServoProfile(s.config.profile))
    {
        if (m_gapDryRun.active && !m_gapDryRun.paused && !s.failureAlarmPending) return;
        const auto& p = f.physical;
        const bool current = r.session == s.scope.session && r.generation == m_edmGapDynamic.generation &&
            r.profile == static_cast<std::uint8_t>(s.config.profile) && p.session == s.scope.session && p.generation == r.generation;
        // Format floating values on this inactive NC boundary, then send the
        // completed string to RtPrintf (native logs previously printed Z=f).
        const bool physicalFlush = EDM28::IsPhysicalServoFlushProfile(s.config.profile);
        const bool physicalRetreat = EDM28::IsPhysicalRetreatProfile(s.config.profile);
        const bool physicalRetreatCycle = EDM28::IsPhysicalRetreatCycleProfile(s.config.profile);
        const bool physicalEndurance = EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile);
        const bool physicalBidirectionalFlush = EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile);
        const bool physicalBidirectional = EDM28::IsPhysicalBidirectionalProfile(s.config.profile) || physicalBidirectionalFlush;
        const char* build = physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : physicalBidirectional ? "EDM74" : physicalRetreatCycle ? "EDM73" : physicalRetreat ? "EDM72" : physicalFlush ? "EDM71" : "EDM70";
        char line[768]{};
        int size = std::snprintf(line, sizeof(line), "[%s] event=%.16s result=%.12s reason=%.48s P=%u session=%llu gen=%llu current=%u phase=%u cycles=%u Z=%.6f stop=%llu disarm=%llu source=PHYSICAL axis=Z %s driveAck=0 discharge=0\n",
            build, event ? event : "UNKNOWN", result ? result : "UNKNOWN", reason ? reason : "NONE",
            static_cast<unsigned>(s.config.profile),
            static_cast<unsigned long long>(s.scope.session), static_cast<unsigned long long>(m_edmGapDynamic.generation),
            current ? 1U : 0U, r.phase, r.cycles, s.originPinned ? (f.actualPulse-s.originPulse)/s.config.pulsePerMm : 0.0,
            static_cast<unsigned long long>(f.stopAppliedSequence),
            static_cast<unsigned long long>(f.lastAppliedKind == EDM28::RequestKind::Disarm ? f.lastAppliedSequence : 0ULL),
            (physicalBidirectional || physicalRetreatCycle) ? "return=BOUNDED_FIXTURE anchor=FIRST_ARM" :
            physicalRetreat ? "endpoint=HOLD_NO_RETURN" : "return=FIXTURE");
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[%s] event=SUMMARY result=FAIL reason=LOG_FORMAT discharge=0\n", build);
        if (current)
        {
            if (physicalBidirectional)
                size = std::snprintf(line, sizeof(line), "[%s] event=SOURCE session=%llu windowMs=%llu steps=%llu fresh=%llu firstSeq=%llu lastSeq=%llu maxAgeUs=%llu maxStepUs=%llu rawMinV=%.3f rawMaxV=%.3f filteredMinV=%.3f filteredMaxV=%.3f E8=%.3f E9=%.3f maxTargetError=%.9g windowComplete=%u lowInput=%u inputReason=%s physicalShortRecoveryVerified=0 discharge=0\n",
                    build, static_cast<unsigned long long>(p.session), static_cast<unsigned long long>(p.windowUs/1000ULL),
                    static_cast<unsigned long long>(p.runtimeSteps), static_cast<unsigned long long>(p.freshSamples),
                    static_cast<unsigned long long>(p.firstSequence), static_cast<unsigned long long>(p.lastSequence),
                    static_cast<unsigned long long>(p.maximumSourceAgeUs), static_cast<unsigned long long>(p.maximumStepUs),
                    p.minimumRawV, p.maximumRawV, p.minimumFilteredV, p.maximumFilteredV,
                    s.config.servoRecipe.overridePercent, s.config.servoRecipe.referenceV, p.maximumTargetError,
                    p.completedWindow ? 1U : 0U, p.lowInput ? 1U : 0U, EDM69::ReasonName(p.inputReason));
            else if (physicalRetreatCycle)
                size = std::snprintf(line, sizeof(line), "[EDM73] event=SOURCE session=%llu windowMs=%llu steps=%llu fresh=%llu firstSeq=%llu lastSeq=%llu maxAgeUs=%llu maxStepUs=%llu maxRetreatMm=%.6f negativeTargetTicks=%llu negativeAppliedTicks=%llu fixtureReturnTicks=%llu retreatEndpointCount=%u partialRetreatAborted=%u rawMinV=%.3f rawMaxV=%.3f E8=%.3f E9=%.3f maxTargetError=%.9g windowComplete=%u lowInput=%u inputReason=%s physicalShortRecoveryVerified=0 discharge=0\n",
                    static_cast<unsigned long long>(p.session), static_cast<unsigned long long>(p.windowUs/1000ULL),
                    static_cast<unsigned long long>(p.runtimeSteps), static_cast<unsigned long long>(p.freshSamples),
                    static_cast<unsigned long long>(p.firstSequence), static_cast<unsigned long long>(p.lastSequence),
                    static_cast<unsigned long long>(p.maximumSourceAgeUs), static_cast<unsigned long long>(p.maximumStepUs),
                    p.maximumActualRetreatMm, static_cast<unsigned long long>(p.negativeTargetTicks),
                    static_cast<unsigned long long>(p.negativeAppliedTicks), static_cast<unsigned long long>(p.fixtureReturnTicks),
                    p.retreatEndpointCount, p.partialRetreatAborted ? 1U : 0U, p.minimumRawV, p.maximumRawV,
                    s.config.servoRecipe.overridePercent, s.config.servoRecipe.referenceV, p.maximumTargetError,
                    p.completedWindow ? 1U : 0U, p.lowInput ? 1U : 0U, EDM69::ReasonName(p.inputReason));
            else if (physicalRetreat)
                size = std::snprintf(line, sizeof(line), "[EDM72] event=SOURCE session=%llu windowMs=%llu steps=%llu fresh=%llu firstSeq=%llu lastSeq=%llu maxAgeUs=%llu maxStepUs=%llu maxRetreatMm=%.6f negativeTargetTicks=%llu negativeAppliedTicks=%llu rawMinV=%.3f rawMaxV=%.3f E8=%.3f E9=%.3f maxTargetError=%.9g windowComplete=%u lowInput=%u inputReason=%s physicalShortRecoveryVerified=0 discharge=0\n",
                    static_cast<unsigned long long>(p.session), static_cast<unsigned long long>(p.windowUs/1000ULL),
                    static_cast<unsigned long long>(p.runtimeSteps), static_cast<unsigned long long>(p.freshSamples),
                    static_cast<unsigned long long>(p.firstSequence), static_cast<unsigned long long>(p.lastSequence),
                    static_cast<unsigned long long>(p.maximumSourceAgeUs), static_cast<unsigned long long>(p.maximumStepUs),
                    p.maximumActualRetreatMm, static_cast<unsigned long long>(p.negativeTargetTicks),
                    static_cast<unsigned long long>(p.negativeAppliedTicks), p.minimumRawV, p.maximumRawV,
                    s.config.servoRecipe.overridePercent, s.config.servoRecipe.referenceV, p.maximumTargetError,
                    p.completedWindow ? 1U : 0U, p.lowInput ? 1U : 0U, EDM69::ReasonName(p.inputReason));
            else
                size = std::snprintf(line, sizeof(line), "[%s] event=SOURCE session=%llu windowMs=%llu steps=%llu fresh=%llu firstSeq=%llu lastSeq=%llu maxAgeUs=%llu maxStepUs=%llu maxDepthMm=%.6f rawMinV=%.3f rawMaxV=%.3f E8=%.3f E9=%.3f maxTargetError=%.9g windowComplete=%u lowInput=%u inputReason=%s discharge=0\n",
                build, static_cast<unsigned long long>(p.session), static_cast<unsigned long long>(p.windowUs/1000ULL),
                static_cast<unsigned long long>(p.runtimeSteps), static_cast<unsigned long long>(p.freshSamples),
                static_cast<unsigned long long>(p.firstSequence), static_cast<unsigned long long>(p.lastSequence),
                static_cast<unsigned long long>(p.maximumSourceAgeUs), static_cast<unsigned long long>(p.maximumStepUs),
                p.maximumActualDepthMm, p.minimumRawV, p.maximumRawV, s.config.servoRecipe.overridePercent,
                s.config.servoRecipe.referenceV, p.maximumTargetError, p.completedWindow ? 1U : 0U,
                p.lowInput ? 1U : 0U, EDM69::ReasonName(p.inputReason));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[%s] event=SOURCE result=FAIL reason=LOG_FORMAT discharge=0\n", build);
            if (physicalBidirectional)
            {
                size = std::snprintf(line, sizeof(line), "[%s] event=DIRECTION session=%llu motion=LIVE_AD_BIDIRECTIONAL workTicks=%llu positiveTargetTicks=%llu negativeTargetTicks=%llu positiveAppliedTicks=%llu negativeAppliedTicks=%llu workReversals=%u bidirectionalObserved=%u maxCommandDepthMm=%.6f maxCommandRetreatMm=%.6f fixtureReturnTicks=%llu directionCoverage=OBSERVED_ONLY return=BOUNDED_FIXTURE discharge=0\n",
                    build, static_cast<unsigned long long>(p.session), static_cast<unsigned long long>(p.workTicks),
                    static_cast<unsigned long long>(p.positiveTargetTicks), static_cast<unsigned long long>(p.negativeTargetTicks),
                    static_cast<unsigned long long>(p.positiveAppliedTicks), static_cast<unsigned long long>(p.negativeAppliedTicks),
                    p.workReversals, p.bidirectionalObserved ? 1U : 0U, p.maximumWorkCommandDepthMm,
                    p.maximumWorkCommandRetreatMm, static_cast<unsigned long long>(p.fixtureReturnTicks));
                if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
                else RtPrintf("[%s] event=DIRECTION result=FAIL reason=LOG_FORMAT discharge=0\n", build);
                size = std::snprintf(line, sizeof(line), "[%s] event=CONTROLLER detail=%.16s session=%llu gen=%llu rtReason=%u phase=%u active=%u terminal=%u completed=%u needStop=%u elapsedMs=%llu observationOnly=1 discharge=0\n",
                    physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : "EDM74-FIX2", event ? event : "UNKNOWN", static_cast<unsigned long long>(r.session),
                    static_cast<unsigned long long>(r.generation), static_cast<unsigned>(r.reason), r.phase,
                    r.active ? 1U : 0U, r.terminal ? 1U : 0U, r.completed ? 1U : 0U, r.needFormalStop ? 1U : 0U,
                    static_cast<unsigned long long>(r.nowUs >= r.startedUs ? (r.nowUs-r.startedUs)/1000ULL : 0ULL));
                if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
                else RtPrintf("[%s] event=LOG_FORMAT part=CONTROLLER discharge=0\n", physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : "EDM74-FIX2");
                LogEDMGapOriginSettleSameThread(event, true);
            }
            if (physicalFlush || physicalBidirectionalFlush)
            {
                const auto& flush = r.flush; const auto& recipe = s.config.flush;
                size = std::snprintf(line, sizeof(line), "[%s] event=FLUSH P=%u session=%llu phase=%s cycles=%u deep=%u topDwell=%u slowApproach=%u workPointMm=%.6f topMm=%.6f destinationMm=%.6f liftMm=%.6f complete=%u E5us=%llu E6=%.6f E7=%.3f E21=%u E22=%.6f discharge=0\n",
                    build, static_cast<unsigned>(s.config.profile), static_cast<unsigned long long>(s.scope.session), EDM57::PhaseName(flush.phase),
                    flush.cycles, flush.deepCycles, flush.topDwellCount, flush.slowApproachCount,
                    flush.workPointMm, flush.topMm, flush.destinationMm, flush.liftHeightMm, flush.complete ? 1U : 0U,
                    static_cast<unsigned long long>(recipe.workTimeUs), recipe.baseHeightMm,
                    recipe.jumpSpeedOverridePercent, recipe.deepInterval, recipe.deepMultiplier);
                if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
                else RtPrintf("[%s] event=FLUSH result=FAIL reason=LOG_FORMAT discharge=0\n", build);
            }
        }
        return;
    }
    // EDM66-FIX2: no formatting, clock reads or console IO on the live
    // Arm/controller/Stop/Disarm/fresh-release path. PASS clears active first;
    // HOLD sets paused after requesting Stop; a post-Arm failure sets
    // failureAlarmPending after priority Stop. Cancellation requests Stop before clearing
    // active. These boundaries retain their full cached diagnostic snapshot.
    // Never refresh service time or relax any watchdog to pay for logging.
    if (EDM28::IsGapServoFlushProfile(s.config.profile) &&
        m_gapDryRun.active && !m_gapDryRun.paused && !s.failureAlarmPending) return;
    const bool measureLog = EDM28::IsGapServoFlushProfile(s.config.profile);
    // Read the same counter without changing the control clock's regression
    // floor or service timestamps. This measures elapsed time across the log
    // batch (including any preemption), not console CPU time or RT progress.
    const auto observeMs = [this](std::uint64_t& value) noexcept
    {
        LARGE_INTEGER counter{};
        const std::uint64_t frequency = m_gapDryRun.frequency;
        if (!frequency || !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0)
            return false;
        const std::uint64_t ticks = static_cast<std::uint64_t>(counter.QuadPart);
        if (ticks < m_gapDryRun.lastTicks) return false;
        const std::uint64_t limit = (std::numeric_limits<std::uint64_t>::max)();
        const std::uint64_t seconds = ticks / frequency, remainder = ticks % frequency;
        if (remainder > limit / 1000ULL) return false;
        const std::uint64_t fraction = remainder * 1000ULL / frequency;
        if (seconds > (limit - fraction) / 1000ULL) return false;
        value = seconds * 1000ULL + fraction;
        return true;
    };
    std::uint64_t logStartMs = 0ULL;
    const bool logClockValid = measureLog && observeMs(logStartMs);
    const double relative = s.originPinned && s.config.pulsePerMm > 0.0 ?
        (f.actualPulse - s.originPulse) / s.config.pulsePerMm : 0.0;
    const bool flush = EDM28::IsGapFlushProfile(m_edmZFixture.config.profile);
    const char* build = EDM28::IsGapServoFlushProfile(m_edmZFixture.config.profile) ? "EDM65" : EDM28::IsGapConditionServoProfile(m_edmZFixture.config.profile) ? "EDM59" : EDM57::IsBoundProfile(static_cast<unsigned>(m_edmZFixture.config.profile)) ? "EDM58" : flush ? "EDM57" : "EDM56";
    const bool currentTelemetry = m_edmGapDynamic.generation && r.session == s.scope.session &&
        r.generation == m_edmGapDynamic.generation && r.profile == static_cast<std::uint8_t>(s.config.profile);
    char line[480]{};
    int n = 0;
    if (measureLog && !currentTelemetry)
    {
        // Arm and pre-Arm failures can still carry the last controller's
        // snapshot. Attribute that history explicitly; it is not this run's
        // voltage, servo speed, phase or flush work point.
        n = std::snprintf(line, sizeof(line),
            "[%s] event=%.16s result=%.12s reason=%.40s P=%u session=%llu gen=%llu tick=%llu telemetry=RETAINED rtSession=%llu rtGen=%llu Z=%.6f stop=%llu disarm=%llu source=SIM axis=Z driveAck=0 discharge=0\n",
            build, event ? event : "UNKNOWN", result ? result : "UNKNOWN", reason ? reason : "NONE",
            static_cast<unsigned>(s.config.profile), static_cast<unsigned long long>(s.scope.session),
            static_cast<unsigned long long>(m_edmGapDynamic.generation), static_cast<unsigned long long>(f.sampledTick),
            static_cast<unsigned long long>(r.session), static_cast<unsigned long long>(r.generation), relative,
            static_cast<unsigned long long>(f.stopAppliedSequence),
            static_cast<unsigned long long>(f.lastAppliedKind == EDM28::RequestKind::Disarm ? f.lastAppliedSequence : 0ULL));
    }
    else n = std::snprintf(line, sizeof(line),
        "[%s] event=%.16s result=%.12s reason=%.40s P=%u session=%llu gen=%llu tick=%llu phase=%u cycles=%u raw=%.1f filt=%.1f target=%.2f applied=%.2f Z=%.6f stop=%llu disarm=%llu source=SIM axis=Z driveAck=0 discharge=0\n",
        build, event ? event : "UNKNOWN", result ? result : "UNKNOWN", reason ? reason : "NONE",
        static_cast<unsigned>(s.config.profile), static_cast<unsigned long long>(s.scope.session),
        static_cast<unsigned long long>(m_edmGapDynamic.generation), static_cast<unsigned long long>(f.sampledTick),
        static_cast<unsigned>(r.phase), static_cast<unsigned>(r.cycles), r.rawVoltageV, r.filteredVoltageV,
        r.targetMmMin, r.appliedMmMin, relative, static_cast<unsigned long long>(f.stopAppliedSequence),
        static_cast<unsigned long long>(f.lastAppliedKind == EDM28::RequestKind::Disarm ? f.lastAppliedSequence : 0ULL));
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else if (flush) RtPrintf("[%s] event=LOG result=FAIL reason=LOG_FORMAT discharge=0\n", build);
    else RtPrintf("[EDM56] event=LOG result=FAIL reason=LOG_FORMAT discharge=0\n");
    if (measureLog && currentTelemetry && (!event || std::strcmp(event, "PHASE") != 0))
    {
        // Expected binding is frozen by the RT source contract; only sequence
        // and capture time below are serialized producer observations. Never
        // present a computed identity as an independently observed IO value.
        const bool ageValid = r.sourceSequence && r.sourceCapturedAtUs &&
            r.sourceCapturedAtUs <= f.monotonicUs;
        n = std::snprintf(line, sizeof(line),
            "[EDM65] event=SOURCE detail=%.16s session=%llu gen=%llu expectedKind=SIM expectedId=60 expectedGeneration=%llu expectedCalibration=1 seq=%llu capturedUs=%llu ageUs=%llu ageValid=%u rtActive=%u sourcePort=61 observationOnly=1 discharge=0\n",
            event ? event : "UNKNOWN", static_cast<unsigned long long>(r.session),
            static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(r.generation),
            static_cast<unsigned long long>(r.sourceSequence), static_cast<unsigned long long>(r.sourceCapturedAtUs),
            static_cast<unsigned long long>(ageValid ? f.monotonicUs - r.sourceCapturedAtUs : 0ULL),
            ageValid ? 1U : 0U, r.active ? 1U : 0U);
        if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM65] event=SOURCE reason=LOG_FORMAT discharge=0\n");
    }
    if (flush) LogEDMGapFlushSameThread(event);
    // Servo-flush diagnostics now reach only stopped/failure/completed NC
    // boundaries. Other condition-servo profiles retain their WAIT cadence.
    if (!measureLog || !event || std::strcmp(event, "PHASE") != 0)
        LogEDMGapOriginSettleSameThread(event, true);
    if (measureLog)
    {
        std::uint64_t logEndMs = 0ULL;
        const bool valid = logClockValid && observeMs(logEndMs) && logEndMs >= logStartMs;
        m_edmZFixture.lastDynamicLogEndMs = valid ? logEndMs : 0ULL;
        m_edmZFixture.lastDynamicLogDurationMs = valid ? logEndMs - logStartMs : 0ULL;
    }
}

void NCManager::LogEDMGapOriginSettleSameThread(const char* event, bool force) noexcept
{
    const bool physicalEndurance = EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(m_edmZFixture.config.profile);
    const bool physicalBidirectionalFlush = EDM28::IsPhysicalBidirectionalFlushFamilyProfile(m_edmZFixture.config.profile);
    const bool physicalBidirectional = EDM28::IsPhysicalBidirectionalProfile(m_edmZFixture.config.profile) || physicalBidirectionalFlush;
    if (EDM28::IsPhysicalServoProfile(m_edmZFixture.config.profile) && !physicalBidirectional) return;
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& r = f.dynamic;
    // FIX1 diagnostics remain silent during Arm/work/return/Stop/Disarm.
    // Format only the retained observations at an already-inactive boundary.
    if (physicalBidirectional && m_gapDryRun.active && !m_gapDryRun.paused && !s.failureAlarmPending) return;
    // Also guard the direct active WAIT call; do not advance log cadence when
    // silent, so the eventual boundary can still print the retained sample.
    if (EDM28::IsGapServoFlushProfile(s.config.profile) &&
        m_gapDryRun.active && !m_gapDryRun.paused && !s.failureAlarmPending) return;
    const auto& q = r.originSettle; auto& d = m_edmGapDynamic;
    // Last validated RT observations remain explicitly timestamped across a
    // terminal/fault. Logging owns only its two cadence words, not a receipt.
    if (!EDM28::IsGapConditionServoProfile(s.config.profile) ||
        !d.generation || r.session != s.scope.session || r.generation != d.generation ||
        r.profile != static_cast<std::uint8_t>(s.config.profile) || !q.enteredUs ||
        !q.sampledTick || !q.sampledUs || q.sampledUs < q.enteredUs ||
        r.tick > f.sampledTick || r.nowUs > f.monotonicUs ||
        q.sampledTick > r.tick || q.sampledUs > r.nowUs || q.sampledUs > f.monotonicUs)
        return;
    const bool sameEntry = d.originDiagLastEntryUs == q.enteredUs;
    if (sameEntry && (d.originDiagLastLogUs == f.monotonicUs ||
        (!force && (f.monotonicUs < d.originDiagLastLogUs ||
            f.monotonicUs - d.originDiagLastLogUs < 1000000ULL)))) return;
    d.originDiagLastEntryUs = q.enteredUs; d.originDiagLastLogUs = f.monotonicUs;
    const char* build = physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : physicalBidirectional ? "EDM74-FIX2" :
        EDM28::IsGapServoFlushProfile(s.config.profile) ? "EDM65" : "EDM59-FIX2";
    char line[640]{};
    int n = std::snprintf(line, sizeof(line),
        "[%s] event=ORIGIN_SETTLE detail=%.16s part=GATES session=%llu gen=%llu tick=%llu phase=%u active=%u rtReason=%u cycles=%u enteredUs=%llu sampleTick=%llu sampleUs=%llu sampleAgeUs=%llu waitUs=%llu mask=0x%X stableUs=%u stableTicks=%u bestUs=%u bestTicks=%u observationOnly=1 discharge=0\n",
        build, event ? event : "UNKNOWN", static_cast<unsigned long long>(r.session),
        static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(f.sampledTick),
        r.phase, r.active ? 1U : 0U, static_cast<unsigned>(r.reason), r.cycles,
        static_cast<unsigned long long>(q.enteredUs), static_cast<unsigned long long>(q.sampledTick),
        static_cast<unsigned long long>(q.sampledUs),
        static_cast<unsigned long long>(f.monotonicUs - q.sampledUs),
        static_cast<unsigned long long>(q.sampledUs - q.enteredUs), q.blockerMask,
        q.stableUs, q.stableTicks, q.bestStableUs, q.bestStableTicks);
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[%s] event=LOG_FORMAT part=GATES discharge=0\n", build);
    n = std::snprintf(line, sizeof(line),
        "[%s] event=ORIGIN_SETTLE detail=%.16s part=VALUES session=%llu gen=%llu sampleTick=%llu errorMm=%.9g pdoMmS=%.9g windowRangeMm=%.9g minErrorMm=%.9g maxErrorMm=%.9g maxAbsPdoMmS=%.9g samples=%u refReject=%u positionReject=%u pdoHigh=%u rangeReset=%u observationOnly=1 discharge=0\n",
        build, event ? event : "UNKNOWN", static_cast<unsigned long long>(r.session),
        static_cast<unsigned long long>(r.generation), static_cast<unsigned long long>(q.sampledTick),
        q.errorMm, q.pdoMmS, q.windowRangeMm, q.errorMinMm, q.errorMaxMm, q.maxAbsPdoMmS,
        q.observations, q.referenceRejectTicks, q.positionRejectTicks, q.pdoHighTicks, q.rangeResetCount);
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[%s] event=LOG_FORMAT part=VALUES discharge=0\n", build);
    if (physicalBidirectional && f.physical.session == r.session && f.physical.generation == r.generation)
    {
        const auto& c = f.physical.originControl;
        n = std::snprintf(line, sizeof(line), "[%s] event=ORIGIN_CONTROL session=%llu gen=%llu sampleTick=%llu samples=%u idleGainMatch=%u motionGainMatch=%u otherGain=%u clamped=%u invalid=%u Kp=%.9g Ki=%.9g Kvff=%.9g pMmS=%.9g iMmS=%.9g imageHardwareMmS=%.9g image=LOCAL_PRE_SERIALIZATION driveAck=0 observationOnly=1 discharge=0\n",
            build, static_cast<unsigned long long>(r.session), static_cast<unsigned long long>(r.generation),
            static_cast<unsigned long long>(c.sampleTick), c.samples, c.idleGainTicks, c.motionGainTicks,
            c.otherGainTicks, c.clampedTicks, c.invalidTicks, c.kp, c.ki, c.kvff,
            c.proportionalMmS, c.integralMmS, c.guardedHardwareMmS);
        if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[%s] event=LOG_FORMAT part=CONTROL discharge=0\n", build);
        if (physicalBidirectionalFlush)
            n = std::snprintf(line, sizeof(line), "[%s] event=SETTLE_HANDOFF session=%llu gen=%llu applied=%u count=%u tick=%llu enteredUs=%llu settlePhase=%u priorIMmS=%.9g action=RESET_ACTIVE_I_ONCE_PER_SETTLE_ENTRY receipt=LOCAL_CONTROLLER driveAck=0 discharge=0\n",
                build, static_cast<unsigned long long>(r.session), static_cast<unsigned long long>(r.generation),
                c.handoffTick ? 1U : 0U, c.handoffCount, static_cast<unsigned long long>(c.handoffTick),
                static_cast<unsigned long long>(c.handoffEntryUs), c.handoffPhase, c.handoffIntegralMmS);
        else
            n = std::snprintf(line, sizeof(line), "[EDM74-FIX2] event=ORIGIN_HANDOFF session=%llu gen=%llu applied=%u tick=%llu priorIMmS=%.9g action=RESET_ACTIVE_I_ONCE receipt=LOCAL_CONTROLLER driveAck=0 discharge=0\n",
            static_cast<unsigned long long>(r.session), static_cast<unsigned long long>(r.generation),
            c.handoffTick ? 1U : 0U, static_cast<unsigned long long>(c.handoffTick), c.handoffIntegralMmS);
        if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[%s] event=LOG_FORMAT part=HANDOFF discharge=0\n", build);
    }
}


void NCManager::LogEDMGapFlushSameThread(const char* event) noexcept
{
    const auto& s = m_edmZFixture; const auto& r = s.feedback.dynamic; const auto& f = r.flush;
    if (EDM28::IsGapServoFlushProfile(s.config.profile) &&
        m_gapDryRun.active && !m_gapDryRun.paused && !s.failureAlarmPending) return;
    if (EDM28::IsGapServoFlushProfile(s.config.profile) &&
        (!m_edmGapDynamic.generation || r.session != s.scope.session ||
            r.generation != m_edmGapDynamic.generation || r.profile != static_cast<std::uint8_t>(s.config.profile)))
        return;
    const auto& recipe = s.config.flush;
    const char* build = EDM28::IsGapServoFlushProfile(s.config.profile) ? "EDM65" : EDM57::IsBoundProfile(static_cast<unsigned>(s.config.profile)) ? "EDM58" : "EDM57";
    char line[480]{};
    const int n = std::snprintf(line, sizeof(line),
        "[%s] event=FLUSH detail=%.16s gen=%llu phase=%.24s cycles=%u deep=%u blocked=%u resumed=%u slow=%u dwell=%u inhibit=%u persistent=%u work=%.6f top=%.6f destination=%.6f height=%.6f anchor=FIRST_ARM recipe=%u E5us=%llu E6=%.6f E7=%.6f E21=%u E22=%.6f source=SIM discharge=0\n",
        build, event ? event : "UNKNOWN", static_cast<unsigned long long>(r.generation),
        EDM57::PhaseName(static_cast<EDM57::Phase>(f.phase)), f.cycles, f.deepCycles, f.blockedCount,
        f.resumedReturnCount, f.slowApproachCount, f.topDwellCount, f.returnBlocked ? 1U : 0U,
        f.persistentLatched ? 1U : 0U, f.workPointMm, f.topMm, f.destinationMm, f.liftHeightMm,
        recipe.revision, static_cast<unsigned long long>(recipe.workTimeUs), recipe.baseHeightMm,
        recipe.jumpSpeedOverridePercent, recipe.deepInterval, recipe.deepMultiplier);
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[%s] event=FLUSH result=FAIL reason=LOG_FORMAT discharge=0\n", build);
}

bool NCManager::IsEDMPhysicalSequenceReleaseCurrentSameThread(NCBlockDispatchId nextDispatch,
    std::uint64_t& receiptAgeUs) noexcept
{
    receiptAgeUs = 0ULL;
    auto& s = m_edmZFixture; const auto& d = m_edmGapDynamic;
    // A released segment is historical evidence, not permission to change the
    // current observer scope. Recipe edits are allowed only after this release.
    if (!d.sequenceMode || !d.active || !d.terminalObserved || !d.armIssued ||
        !EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile) ||
        m_gapDryRun.active || m_gapDryRun.paused || m_gapDryRun.result != 1U ||
        s.phase != EDMZFixturePhase::Finished || s.cleanupPending || s.resetPending ||
        s.failureAlarmPending || !s.originPinned || !s.motionApplied ||
        !d.sequenceReceiptPublication || !d.sequenceReceiptTick || !d.sequenceReceiptUs ||
        !m_gapDryRun.run || m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapDryRun.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease) ||
        nextDispatch == NC_BLOCK_DISPATCH_ID_INVALID ||
        nextDispatch <= m_gapDryRun.dispatch ||
        !IsEDMShortFlushRunReadySameThread() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !ReadEDMZFixtureFeedbackSameThread(false)) return false;
    const auto& f = s.feedback;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || f.monotonicUs > nowUs ||
        nowUs - f.monotonicUs > 5000ULL ||
        f.publicationSequence < d.sequenceReceiptPublication ||
        f.sampledTick < d.sequenceReceiptTick || f.monotonicUs < d.sequenceReceiptUs ||
        (f.sampledTick == d.sequenceReceiptTick && f.monotonicUs != d.sequenceReceiptUs) ||
        (f.sampledTick > d.sequenceReceiptTick && f.monotonicUs <= d.sequenceReceiptUs) ||
        f.currentEpoch != m_gapDryRun.epoch ||
        f.currentOwner != static_cast<std::uint8_t>(m_gapDryRun.lease.owner) ||
        f.currentOwnerGeneration != m_gapDryRun.lease.generation ||
        !IsEDMGapDynamicConfigValidSameThread(true) || !IsEDMGapDynamicTerminalSameThread() ||
        !IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm) ||
        !EvaluateEDMGapDynamicCompletionSameThread(EDM49::Stage::FreshReleased).Ready()) return false;
    receiptAgeUs = nowUs - f.monotonicUs;
    return true;
}

bool NCManager::IsEDMPhysicalSequenceBoundarySameThread() noexcept
{
    std::uint64_t receiptAgeUs = 0ULL;
    if (!IsEDMPhysicalSequenceReleaseCurrentSameThread(m_currentExecutingBlockDispatchId, receiptAgeUs))
        return false;
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    RtPrintf("[EDM77] event=SEQUENCE_BOUNDARY result=PROVEN previousSession=%llu previousDispatch=%llu nextDispatch=%llu run=%llu stopSeq=%llu disarmSeq=%llu receiptPub=%llu receiptTick=%llu receiptAgeUs=%llu axis=Z origin=FIRST_ARM discharge=0\n",
        static_cast<unsigned long long>(s.scope.session), static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned long long>(m_currentExecutingBlockDispatchId), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(s.terminalStopAppliedSequence), static_cast<unsigned long long>(s.pendingSequence),
        static_cast<unsigned long long>(f.publicationSequence), static_cast<unsigned long long>(f.sampledTick),
        static_cast<unsigned long long>(receiptAgeUs));
    return true;
}

// EDM80 closes the pinned Q2 plan at the authored M30. The existing program
// end gate and idle-hold handoff remain authoritative; this adds no motion.
bool NCManager::ValidateEDMPhysicalSequenceEndSameThread(NCProgramEndCause cause,
    int sourcePC, int sourceLineNumber, NCBlockDispatchId markerDispatchId,
    bool finalCheck) noexcept
{
    auto& plan = m_edmPhysicalSequencePlan;
    const auto run = m_pathCoreLiveBookkeeping.currentRunToken;
    const bool required = plan.plan.valid || plan.run != 0ULL ||
        (run != 0ULL && m_gapDryRun.run == run && m_edmGapDynamic.strictSequenceMode);
    if (!required) return true; // Q1 and legacy programs retain their end rules.
    const auto reject = [this, sourceLineNumber, markerDispatchId, finalCheck](const char* reason) noexcept -> bool
    {
        CancelProgramEndBoundary();
        // A finished, disarmed fixture is not an active session to stop again.
        // Contain genuinely busy work before any new terminal diagnostic IO.
        RejectEDMGapAdmissionSameThread(AlarmManager::PROGRAM_END_GATE_ERROR,
            sourceLineNumber, "EDM80_PLAN_END_REJECTED");
        RtPrintf("[EDM80] event=PLAN_END_REJECT result=REJECT phase=%s reason=%s run=%llu dispatch=%llu armIssued=0 discharge=0\n",
            finalCheck ? "FINALIZE" : "REQUEST", reason,
            static_cast<unsigned long long>(m_pathCoreLiveBookkeeping.currentRunToken),
            static_cast<unsigned long long>(markerDispatchId));
        return false;
    };
    if (!plan.plan.valid || !plan.plan.count || plan.plan.count > EDM77::MaximumFixtures ||
        plan.next != plan.plan.count || plan.legPending || !run || plan.run != run ||
        plan.cache != GetBaseProgramCache().GetGeneration() ||
        plan.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !plan.lease.Matches(m_programMotionLease) || !m_motion.IsMotionOwnerLeaseCurrent(plan.lease) ||
        !m_edmGapDynamic.strictSequenceMode || !m_edmGapDynamic.sequenceMode ||
        m_gapDryRun.run != plan.run || m_gapDryRun.cache != plan.cache ||
        m_gapDryRun.epoch != plan.epoch || !m_gapDryRun.lease.Matches(plan.lease) ||
        plan.dispatch != m_gapDryRun.dispatch || plan.dispatch == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_isG66Active || !m_macroStack.empty()) return reject("PLAN_SCOPE");
    if (cause != NCProgramEndCause::M30 || plan.firstPC < 0 ||
        plan.firstPC > (std::numeric_limits<int>::max)() - static_cast<int>(EDM77::MaximumPhysicalLines) ||
        !plan.plan.terminalOffset || plan.plan.terminalOffset > EDM77::MaximumPhysicalLines ||
        sourcePC != plan.firstPC + static_cast<int>(plan.plan.terminalOffset)) return reject("END_SOURCE");
    const NCProgramCacheLine* terminal = GetBaseProgramCache().TryGetLine(sourcePC);
    if (!terminal || sourceLineNumber != terminal->sourceLineNumber ||
        !EDM77::Detail::Clean(terminal->parsedBlock) ||
        !EDM77::Detail::Terminal(terminal->parsedBlock)) return reject("END_LITERAL");
    if (markerDispatchId == NC_BLOCK_DISPATCH_ID_INVALID || markerDispatchId <= plan.dispatch ||
        (plan.terminalDispatch != NC_BLOCK_DISPATCH_ID_INVALID &&
            (plan.terminalDispatch != markerDispatchId || plan.terminalPC != sourcePC)) ||
        (finalCheck && plan.terminalDispatch == NC_BLOCK_DISPATCH_ID_INVALID)) return reject("END_MARKER");
    const auto& last = plan.plan.checkpoints[plan.plan.count - 1U];
    if (last.sourceOffset >= plan.plan.terminalOffset ||
        last.profile != static_cast<unsigned>(m_edmZFixture.config.profile)) return reject("LAST_FIXTURE");
    const NCProgramCacheLine* fixture = GetBaseProgramCache().TryGetLine(
        plan.firstPC + static_cast<int>(last.sourceOffset));
    if (!fixture || fixture->sourceLineNumber != m_gapDryRun.sourceLine ||
        EDM77::SequenceMode(fixture->parsedBlock) != 2U ||
        !EDM77::Detail::EqualLiteral(fixture->parsedBlock.expression('P'), last.profile))
        return reject("LAST_LITERAL");
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != plan.run || m_recipeCache != plan.cache || m_recipeEpoch != plan.epoch ||
        !m_recipeLease.Matches(plan.lease)) return reject("RECIPE_AUTHORITY");
    const auto match = EDM79::MatchCheckpoint(last, last.sourceOffset, last.profile,
        m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    if (match != EDM79::MatchError::None) return reject(EDM79::MatchErrorName(match));
    if (!EDM57::SameRecipe(last.flush, m_edmZFixture.config.flush) ||
        !EDM59::SameRecipe(last.servo, m_edmZFixture.config.servoRecipe)) return reject("FROZEN_RECIPE");
    std::uint64_t receiptAgeUs = 0ULL;
    if (!IsEDMPhysicalSequenceReleaseCurrentSameThread(markerDispatchId, receiptAgeUs))
        return reject("RELEASE_PROOF");
    if (!finalCheck && plan.terminalDispatch == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        plan.terminalDispatch = markerDispatchId; plan.terminalPC = sourcePC;
        RtPrintf("[EDM80] event=PLAN_END_REQUEST result=ACCEPT count=%u run=%llu cache=%llu dispatch=%llu terminalPC=%d generation=%llu proof=FRESH_RELEASED consumed=0 discharge=0\n",
            plan.plan.count, static_cast<unsigned long long>(plan.run),
            static_cast<unsigned long long>(plan.cache), static_cast<unsigned long long>(markerDispatchId),
            sourcePC, static_cast<unsigned long long>(last.key.generation));
    }
    return true;
}

// Called only after the original program-end gate, idle handoff, MarkFinalized
// and final NC state CAS succeeded. Old Program ownership is consumed already;
// do not try to recreate or revalidate that lease after the handoff.
void NCManager::CommitEDMPhysicalSequenceEndSameThread() noexcept
{
    const auto& plan = m_edmPhysicalSequencePlan;
    if (plan.plan.valid && plan.terminalDispatch != NC_BLOCK_DISPATCH_ID_INVALID)
    {
        RtPrintf("[EDM80] event=PLAN_END_PASS result=PASS count=%u run=%llu cache=%llu dispatch=%llu terminalPC=%d consumed=1 next=NEW_RUN_REQUIRED discharge=0\n",
            plan.plan.count, static_cast<unsigned long long>(plan.run),
            static_cast<unsigned long long>(plan.cache), static_cast<unsigned long long>(plan.terminalDispatch),
            plan.terminalPC);
        m_edmGapDynamic.strictSequenceMode = false;
    }
    InvalidateEDMPhysicalSequencePlanSameThread();
}


bool NCManager::IsEDMGapDynamicConfigValidSameThread(bool requireFrozen) const noexcept
{
    const auto& s = m_edmZFixture; const auto& c = s.config;
    const auto same = [](const EDM28::Config& a, const EDM28::Config& b) noexcept
    {
        return a.profile == b.profile && a.axisIndex == b.axisIndex && a.pulsePerMm == b.pulsePerMm &&
            a.referenceOffsetPulse == b.referenceOffsetPulse && a.hardwareSign == b.hardwareSign &&
            a.outerHalfMm == b.outerHalfMm && a.targetHalfMm == b.targetHalfMm &&
            a.feedMmMin == b.feedMmMin && a.pdoCapMmS == b.pdoCapMmS &&
            a.accelerationTimeSec == b.accelerationTimeSec && a.decelerationTimeSec == b.decelerationTimeSec &&
            a.followingLimitMm == b.followingLimitMm && a.positionToleranceMm == b.positionToleranceMm &&
            a.excursionToleranceMm == b.excursionToleranceMm && a.heartbeatTicks == b.heartbeatTicks &&
            a.maximumIssueAgeTicks == b.maximumIssueAgeTicks && a.stopTimeoutTicks == b.stopTimeoutTicks && a.settleCycles == b.settleCycles &&
            EDM57::SameRecipe(a.flush, b.flush) && EDM59::SameRecipe(a.servoRecipe, b.servoRecipe) &&
            EDM69::SameRequest(a.physicalInput, b.physicalInput);
    };
    return m_edmGapDynamic.active && EDM28::IsGapDynamicProfile(c.profile) && c.axisIndex == 2U &&
        std::isfinite(c.pulsePerMm) && c.pulsePerMm >= 1000.0 && c.pulsePerMm <= 1000000000.0 &&
        std::isfinite(c.referenceOffsetPulse) && (c.hardwareSign == 1 || c.hardwareSign == -1) &&
        (EDM28::IsPhysicalServoProfile(c.profile) ? (EDM70::ValidConfig(c) && EDM70::MatchesScope(c.physicalInput, s.scope)) :
            (c.outerHalfMm == .4 && c.targetHalfMm == .25 && c.feedMmMin == 30.0 && c.pdoCapMmS == .5 &&
                EDM69::SameRequest(c.physicalInput, EDM69::Request{}))) &&
        c.accelerationTimeSec == .3 && c.decelerationTimeSec == .3 && c.followingLimitMm == .02 &&
        c.positionToleranceMm == .001 && c.excursionToleranceMm == .001 && c.heartbeatTicks == 200U &&
        c.maximumIssueAgeTicks == 80U && c.stopTimeoutTicks == 4000U && c.settleCycles == 200U &&
        (EDM28::IsPhysicalBidirectionalFlushFamilyProfile(c.profile) ?
            (EDM75::ValidateRecipe(c.flush) && EDM71::SameAdmission(c.flush, c.servoRecipe)) :
            EDM28::IsPhysicalServoFlushProfile(c.profile) ?
            (EDM71::ValidateRecipe(c.flush) && EDM71::SameAdmission(c.flush, c.servoRecipe)) :
            EDM28::IsGapFlushProfile(c.profile) ? EDM57::ValidateRecipeForProfile(c.flush, static_cast<unsigned>(c.profile)) :
            EDM57::SameRecipe(c.flush, EDM57::Recipe{})) &&
        ((EDM28::IsPhysicalBidirectionalProfile(c.profile) || EDM28::IsPhysicalBidirectionalFlushFamilyProfile(c.profile)) ? EDM74::ValidateRecipe(c.servoRecipe) :
            EDM28::IsPhysicalNegativeProfile(c.profile) ? EDM72::ValidateRecipe(c.servoRecipe) :
            EDM28::IsGapConditionServoProfile(c.profile) ? EDM59::ValidateRecipe(c.servoRecipe) : EDM59::IsEmptyRecipe(c.servoRecipe)) &&
        (!EDM28::IsGapServoFlushProfile(c.profile) || EDM28::ValidateGapServoFlushConfig(c)) &&
        same(c, m_edmGapDynamic.frozenConfig) && (!requireFrozen || same(c, s.feedback.frozenConfig));
}

bool NCManager::IsEDMGapDynamicRetiredFaultSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback; const auto& d = m_edmGapDynamic;
    // Disarm retains the previous fault for diagnostics. It is not a fault of
    // this new session until a first Arm has actually acquired its own cap.
    // Classify only that historical Stop/Disarm receipt here. A new idle
    // proof can be renewing after Disarm; PreArm waits within its existing
    // admission deadline instead of calling that old latch a new fault.
    // Never clear RT state here: RT Arm still validates current authority,
    // source, configuration and 200 new idle cycles before clearing its latch.
    const bool firstArmPending = s.phase == EDMZFixturePhase::ArmPending && s.pending &&
        s.pendingKind == EDM28::RequestKind::Arm && d.armIssued && s.pendingSequence == d.armSequence &&
        s.pendingSequence > f.lastAppliedSequence;
    const bool cancelledFirstArm = m_gapDryRun.paused && s.phase == EDMZFixturePhase::ShortStop &&
        d.armIssued && s.curvePreArmHold && s.curvePreArmSequence == d.armSequence;
    if (!IsEDMGapDynamicConfigValidSameThread(false) || s.originPinned || s.motionApplied || s.windowStartMs ||
        s.cleanupPending || s.failureAlarmPending || s.resetPending || d.generation || d.acknowledged || d.terminalObserved ||
        !((s.phase == EDMZFixturePhase::PreArm && !s.pending && !d.armIssued) || firstArmPending || cancelledFirstArm) ||
        !f.latchedFault || f.reason == EDM28::Reason::None || f.state != EDM28::State::Disarmed || f.capHeld || f.stopLatched || f.forceZeroOutput || f.originValid ||
        !EDM28::ValidScope(f.scope) || f.scope.session > s.scope.session ||
        !f.publicationSequence || !f.sampledTick || !f.monotonicUs || !f.sourceFresh || !f.pdoValid || !f.contiguous || !f.clockValid ||
        !f.axisExists || !f.linear || !f.servoReady || !f.modeReady || f.modeValue != 9 || (f.statusWord & 0x006FU) != 0x0027U ||
        f.fault || f.hardPositive || f.hardNegative || !f.axisIdle || !f.frameCurrent ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.scope.axisMapGeneration != s.scope.axisMapGeneration || f.scope.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign ||
        !std::isfinite(f.actualPulse) || !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
        !std::isfinite(f.cmdSpeedMmS) || !std::isfinite(f.pdoSpeedMmS) ||
        // PDO velocity is the outgoing PID correction, not measured motion.
        // Keep it finite; physical settling is proved by RT below PreArm.
        std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.commandPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        std::fabs(f.actualPulse - f.planningPulse) > s.config.positionToleranceMm * s.config.pulsePerMm ||
        f.lastAppliedKind != EDM28::RequestKind::Disarm || !f.stopAppliedSequence || !f.stopAppliedTick || !f.stopAppliedMonotonicUs ||
        f.lastAppliedSequence <= f.stopAppliedSequence || f.lastAppliedTick <= f.stopAppliedTick ||
        f.lastAppliedMonotonicUs <= f.stopAppliedMonotonicUs || f.sampledTick <= f.lastAppliedTick || f.monotonicUs <= f.lastAppliedMonotonicUs ||
        f.sampledTick - f.stopAppliedTick < s.config.settleCycles ||
        f.monotonicUs - f.stopAppliedMonotonicUs < s.config.settleCycles * EDM28::CycleUs || !f.proofSequence)
        return false;
    const bool sameRetiredReceipt = f.lastAppliedSequence == s.replanRetiredFaultDisarmSequence &&
        f.lastAppliedTick == s.replanRetiredFaultDisarmTick && f.lastAppliedMonotonicUs == s.replanRetiredFaultDisarmUs &&
        f.stopAppliedSequence == s.replanRetiredFaultStopSequence && f.stopAppliedTick == s.replanRetiredFaultStopTick &&
        f.stopAppliedMonotonicUs == s.replanRetiredFaultStopUs && f.reason == s.replanRetiredFaultReason;
    if (!f.queuedCancellation)
        return f.scope.session < s.scope.session && f.lastRequestSequence == f.lastAppliedSequence &&
            (!s.replanRetiredFaultSession || (f.scope.session == s.replanRetiredFaultSession && sameRetiredReceipt));
    // HOLD can cancel a queued first Arm before it applies. RT then changes
    // the cancellation scope but deliberately retains the prior fault and
    // Disarm stamps. Require that exact receipt, previously proved here, plus
    // a fresh idle window; a current-session applied fault cannot use this path.
    const bool exactRetired = s.replanRetiredFaultSession && s.replanRetiredFaultSession < f.scope.session &&
        sameRetiredReceipt && f.armReady && f.lastRequestSequence > f.lastAppliedSequence;
    const bool exactCancellation = s.curvePreArmHold && s.curvePreArmSequence &&
        f.lastRequestSequence == s.curvePreArmSequence &&
        (f.scope.session < s.scope.session || (cancelledFirstArm && EDM28::SameScope(f.scope, s.scope)));
    return exactRetired && exactCancellation;
}

bool NCManager::ObserveEDMGapDynamicRetiredFaultSameThread() noexcept
{
    if (!IsEDMGapDynamicRetiredFaultSameThread()) return false;
    auto& s = m_edmZFixture; const auto& f = s.feedback;
    if (!f.queuedCancellation)
    {
        s.replanRetiredFaultSession = f.scope.session;
        s.replanRetiredFaultDisarmSequence = f.lastAppliedSequence; s.replanRetiredFaultDisarmTick = f.lastAppliedTick;
        s.replanRetiredFaultDisarmUs = f.lastAppliedMonotonicUs;
        s.replanRetiredFaultStopSequence = f.stopAppliedSequence; s.replanRetiredFaultStopTick = f.stopAppliedTick;
        s.replanRetiredFaultStopUs = f.stopAppliedMonotonicUs; s.replanRetiredFaultReason = f.reason;
    }
    return true;
}

bool NCManager::IsEDMGapDynamicStoppedSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    if (!IsEDMGapDynamicConfigValidSameThread(s.originPinned) || !f.sourceFresh || !f.pdoValid || !f.contiguous ||
        !f.clockValid || !f.axisIdle || (!s.cleanupPending && (f.forceZeroOutput ||
            (f.latchedFault && !IsEDMGapDynamicRetiredFaultSameThread()))) ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse ||
        f.hardwareSign != s.config.hardwareSign || !std::isfinite(f.cmdSpeedMmS) ||
        std::fabs(f.cmdSpeedMmS) > 1.0 / s.config.pulsePerMm) return false;
    if (!s.cleanupPending && m_edmGapDynamic.generation && f.dynamic.session == s.scope.session &&
        f.dynamic.generation == m_edmGapDynamic.generation && f.dynamic.reason != EDM56::Reason::None &&
        f.dynamic.reason != EDM56::Reason::Cancelled) return false;
    if (!s.originPinned && !f.capHeld && !f.originValid && !s.motionApplied && f.armReady)
        return !m_edmGapDynamic.armIssued || (EDM28::SameScope(f.scope, s.scope) && f.queuedCancellation &&
            f.state == EDM28::State::Disarmed && f.lastRequestSequence == m_edmGapDynamic.armSequence);
    return EDM28::SameScope(f.scope, s.scope) && f.state == EDM28::State::Held && f.capHeld && f.frameCurrent &&
        f.originValid && f.stopLatched && f.stopProven && f.proofSequence &&
        f.lastAppliedKind == EDM28::RequestKind::Stop && f.stopAppliedSequence && f.stopAppliedTick && f.stopAppliedMonotonicUs &&
        f.lastAppliedSequence == f.stopAppliedSequence && f.lastAppliedTick == f.stopAppliedTick &&
        f.lastAppliedMonotonicUs == f.stopAppliedMonotonicUs && f.sampledTick >= f.stopAppliedTick &&
        f.sampledTick - f.stopAppliedTick >= s.config.settleCycles && f.monotonicUs >= f.stopAppliedMonotonicUs &&
        f.monotonicUs - f.stopAppliedMonotonicUs >= s.config.settleCycles * EDM28::CycleUs &&
        f.stableCycles >= s.config.settleCycles && (!s.originPinned || f.outerOriginPulse == s.originPulse) &&
        std::fabs(f.actualPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm &&
        std::fabs(f.commandPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm &&
        std::fabs(f.planningPulse - f.outerOriginPulse) <= s.config.outerHalfMm * s.config.pulsePerMm;
}

bool NCManager::IsEDMGapDynamicTerminalSameThread() const noexcept
{
    const auto& s = m_edmZFixture; const auto& r = s.feedback.dynamic; const auto& d = m_edmGapDynamic;
    if (!d.generation || r.session != s.scope.session || r.generation != d.generation ||
        r.profile != static_cast<std::uint8_t>(s.config.profile) || !r.terminal || r.active || !r.needFormalStop ||
        !r.startedTick || !r.startedUs || !r.steps || !r.tick || !r.nowUs || r.tick < r.startedTick || r.nowUs < r.startedUs ||
        !r.sourceSequence || !r.sourceCapturedAtUs || r.sourceCapturedAtUs > r.nowUs ||
        !r.appliedSequence || r.appliedSequence > r.candidateSequence || r.originPulse != s.originPulse ||
        !std::isfinite(r.rawVoltageV) || !std::isfinite(r.filteredVoltageV) || !std::isfinite(r.commandPulse) ||
        !std::isfinite(r.nativeVelocityPps) || !std::isfinite(r.targetMmMin) || !std::isfinite(r.servoMmMin) ||
        !std::isfinite(r.appliedMmMin)) return false;
    if (d.terminalObserved && (r.tick != d.terminal.tick || r.nowUs != d.terminal.nowUs ||
        r.sourceSequence != d.terminal.sourceSequence || r.sourceCapturedAtUs != d.terminal.sourceCapturedAtUs ||
        r.reason != d.terminal.reason || r.cycles != d.terminal.cycles || r.steps != d.terminal.steps ||
        r.appliedSequence != d.terminal.appliedSequence)) return false;
    const std::uint64_t sourceAgeUs = r.nowUs - r.sourceCapturedAtUs;
    const bool neutralOrigin = r.kernelFault == EDM55::Fault::None && r.nativeVelocityPps == 0.0 &&
        r.appliedMmMin == 0.0 && r.commandPulse == s.originPulse && sourceAgeUs <= 5000ULL;
    if (EDM28::IsPhysicalServoProfile(s.config.profile))
    {
        const auto& p = s.feedback.physical;
        const bool physicalFlush = EDM28::IsPhysicalServoFlushProfile(s.config.profile);
        const bool physicalRetreat = EDM28::IsPhysicalRetreatProfile(s.config.profile);
        const bool physicalRetreatCycle = EDM28::IsPhysicalRetreatCycleProfile(s.config.profile);
        const bool physicalBidirectionalFlush = EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile);
        const bool physicalBidirectional = EDM28::IsPhysicalBidirectionalProfile(s.config.profile) || physicalBidirectionalFlush;
        const bool physicalNegative = EDM28::IsPhysicalNegativeProfile(s.config.profile);
        if ((physicalBidirectional || physicalRetreatCycle) && (!EDM70::ValidConfig(s.config) ||
            !EDM70::MatchesScope(s.config.physicalInput, s.scope))) return false;
        // EDM76 must carry both a continuous source window and a matching
        // controller lifetime. A legacy-duration receipt cannot prove endurance.
        if (EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile) &&
            (r.nowUs - r.startedUs < EDM69::DurationUsFor(s.config.physicalInput.recipeFamily) ||
             r.nowUs - r.startedUs >= EDM69::TotalDeadlineUsFor(s.config.physicalInput.recipeFamily) ||
             p.runtimeSteps > EDM69::MaximumRuntimeStepsFor(s.config.physicalInput.recipeFamily))) return false;
        const double endpointPulse = s.originPulse + EDM72::RetreatDepthMm * s.config.pulsePerMm;
        const bool neutralEndpoint = r.kernelFault == EDM55::Fault::None && r.nativeVelocityPps == 0.0 &&
            r.appliedMmMin == 0.0 && r.commandPulse == endpointPulse && sourceAgeUs <= 5000ULL;
        if (physicalBidirectional && (r.phase != static_cast<std::uint32_t>(EDM56::Phase::Terminal) ||
            (physicalBidirectionalFlush ? r.cycles < EDM70::MinimumSignedFlushCyclesFor(static_cast<unsigned>(s.config.profile)) : r.cycles != 0U) || r.shortActive || !EDM74::InputAllowed(s.config.servoRecipe, r.rawVoltageV) ||
            !p.workTicks || p.workTicks > r.steps || p.positiveTargetTicks > p.workTicks ||
            p.negativeTargetTicks > p.workTicks - p.positiveTargetTicks ||
            p.positiveAppliedTicks > p.workTicks || p.negativeAppliedTicks > p.workTicks - p.positiveAppliedTicks ||
            p.workReversals > p.workTicks || p.fixtureReturnTicks > r.steps ||
            p.bidirectionalObserved != (p.positiveAppliedTicks > 0ULL && p.negativeAppliedTicks > 0ULL && p.workReversals > 0U) ||
            !std::isfinite(p.minimumFilteredV) || !std::isfinite(p.maximumFilteredV) ||
            p.minimumFilteredV < 0.0 || p.maximumFilteredV > 300.0 || p.minimumFilteredV > p.maximumFilteredV ||
            !std::isfinite(p.maximumWorkCommandDepthMm) || !std::isfinite(p.maximumWorkCommandRetreatMm) ||
            p.maximumWorkCommandDepthMm < 0.0 || p.maximumWorkCommandRetreatMm < 0.0 ||
            p.maximumWorkCommandDepthMm > EDM70::DepthMm || p.maximumWorkCommandRetreatMm > EDM70::DepthMm ||
            (d.terminalObserved && (r.phase != d.terminal.phase || r.commandPulse != d.terminal.commandPulse ||
                r.originPulse != d.terminal.originPulse || r.kernelFault != d.terminal.kernelFault ||
                r.nativeVelocityPps != d.terminal.nativeVelocityPps || r.appliedMmMin != d.terminal.appliedMmMin ||
                r.candidateSequence != d.terminal.candidateSequence || r.completed != d.terminal.completed ||
                r.shortActive != d.terminal.shortActive || r.feedInhibited != d.terminal.feedInhibited)))) return false;
        if (physicalNegative && (r.phase != static_cast<std::uint32_t>(EDM56::Phase::Terminal) ||
            (physicalRetreat ? r.cycles != 1U : r.cycles < 2U) ||
            !std::isfinite(endpointPulse) || r.shortActive ||
            !EDM72::NegativeInputAllowed(s.config.servoRecipe, r.rawVoltageV) ||
            // The completed EDM69 source neutralizes its kernel and sets
            // feedInhibited. That terminal state is sealed, not live input.
            (d.terminalObserved && (r.phase != d.terminal.phase || r.commandPulse != d.terminal.commandPulse ||
                r.originPulse != d.terminal.originPulse || r.kernelFault != d.terminal.kernelFault ||
                r.nativeVelocityPps != d.terminal.nativeVelocityPps || r.appliedMmMin != d.terminal.appliedMmMin ||
                r.candidateSequence != d.terminal.candidateSequence || r.completed != d.terminal.completed ||
                r.shortActive != d.terminal.shortActive || r.feedInhibited != d.terminal.feedInhibited)))) return false;
        if (physicalBidirectionalFlush && (!EDM75::CompletionValid(r.flush, s.config.flush) ||
            r.flush.cycles != r.cycles || r.flush.phaseSinceUs > r.nowUs ||
            r.flush.stableSinceUs > r.nowUs ||
            (d.terminalObserved && !EDM71::SameCompletion(r.flush, d.terminal.flush)))) return false;
        if (physicalFlush && (!EDM71::CompletionValid(r.flush, s.config.flush) ||
            r.flush.cycles != r.cycles || r.flush.phaseSinceUs > r.nowUs ||
            r.flush.stableSinceUs > r.nowUs ||
            (d.terminalObserved && !EDM71::SameCompletion(r.flush, d.terminal.flush)))) return false;
        return (physicalRetreat ? neutralEndpoint : neutralOrigin) && r.completed && r.reason == EDM56::Reason::Complete && !r.persistentShort &&
            !r.expectedStale && (physicalBidirectionalFlush ? r.cycles >= EDM70::MinimumSignedFlushCyclesFor(static_cast<unsigned>(s.config.profile)) : physicalBidirectional ? r.cycles == 0U : r.cycles > 0U) && p.session == s.scope.session && p.generation == d.generation &&
            p.completedWindow && !p.lowInput && p.inputReason == EDM69::Reason::None &&
            p.windowUs >= EDM69::DurationUsFor(s.config.physicalInput.recipeFamily) && p.runtimeSteps >= p.freshSamples && p.freshSamples >= 2ULL &&
            p.firstSequence > s.config.physicalInput.baselineSequence && p.lastSequence >= p.firstSequence &&
            p.maximumSourceAgeUs <= 5000ULL && p.maximumStepUs && p.maximumStepUs <= 2000ULL &&
            std::isfinite(p.maximumTargetError) && p.maximumTargetError <= 1e-8 &&
            (physicalBidirectional ? (std::isfinite(p.maximumActualDepthMm) && std::isfinite(p.maximumActualRetreatMm) &&
                p.maximumActualDepthMm >= 0.0 && p.maximumActualDepthMm <= EDM70::OuterMm &&
                p.maximumActualRetreatMm >= 0.0 && p.maximumActualRetreatMm <= EDM70::OuterMm) :
            physicalNegative ? (std::isfinite(p.maximumActualRetreatMm) &&
                p.maximumActualRetreatMm >= EDM72::RetreatDepthMm-.001 && p.maximumActualRetreatMm <= EDM70::OuterMm &&
                p.negativeTargetTicks > 0ULL && p.negativeTargetTicks <= r.steps &&
                p.negativeAppliedTicks > 0ULL && p.negativeAppliedTicks <= r.steps &&
                (!physicalRetreatCycle || (p.fixtureReturnTicks > 0ULL && p.fixtureReturnTicks <= r.steps &&
                    p.retreatEndpointCount == r.cycles))) :
                (std::isfinite(p.maximumActualDepthMm) &&
                 (physicalFlush ? p.maximumActualDepthMm > 0.0 : p.maximumActualDepthMm >= EDM70::DepthMm-.001) &&
                 p.maximumActualDepthMm <= EDM70::OuterMm));
    }
    if (EDM28::IsGapFlushProfile(s.config.profile))
    {
        const auto& f = r.flush; const auto& saved = d.terminal.flush;
        if (!EDM57::ValidateRecipeForProfile(s.config.flush, static_cast<unsigned>(s.config.profile)) || !std::isfinite(f.workPointMm) ||
            !std::isfinite(f.topMm) || !std::isfinite(f.destinationMm) || !std::isfinite(f.liftHeightMm) ||
            !f.phaseSinceUs || f.phaseSinceUs > r.nowUs || f.stableSinceUs > r.nowUs ||
            f.blockedSinceUs > r.nowUs || f.cycles != r.cycles || f.cycles > s.config.flush.cycleCount ||
            f.deepCycles > EDM57::ExpectedDeepCycles(s.config.flush, f.cycles) ||
            std::fabs(f.workPointMm) > s.config.outerHalfMm || std::fabs(f.topMm) > s.config.outerHalfMm ||
            std::fabs(f.destinationMm) > s.config.outerHalfMm || f.liftHeightMm < 0.0 ||
            f.liftHeightMm > EDM57::MaximumLiftHeight(s.config.flush)) return false;
        // Completion telemetry is sealed once RT terminates. Physical proof
        // still comes exclusively from the Stop/Disarm/fresh receipt chain.
        if (d.terminalObserved && (f.phase != saved.phase || f.cycles != saved.cycles ||
            f.deepCycles != saved.deepCycles || f.blockedCount != saved.blockedCount ||
            f.resumedReturnCount != saved.resumedReturnCount || f.slowApproachCount != saved.slowApproachCount ||
            f.topDwellCount != saved.topDwellCount || f.phaseSinceUs != saved.phaseSinceUs ||
            f.stableSinceUs != saved.stableSinceUs || f.blockedSinceUs != saved.blockedSinceUs ||
            f.blockedSequence != saved.blockedSequence || f.workPointMm != saved.workPointMm ||
            f.topMm != saved.topMm || f.destinationMm != saved.destinationMm || f.liftHeightMm != saved.liftHeightMm ||
            f.returnBlocked != saved.returnBlocked || f.persistentLatched != saved.persistentLatched ||
            f.complete != saved.complete)) return false;
        if (EDM57::IsStaleProfile(static_cast<unsigned>(s.config.profile)))
            return !r.completed && !r.persistentShort && r.expectedStale && !f.complete && !f.persistentLatched &&
                r.reason == EDM56::Reason::ExpectedStale && r.kernelFault == EDM55::Fault::Stale &&
                sourceAgeUs > 5000ULL && f.phase == static_cast<std::uint32_t>(EDM57::Phase::Lift) &&
                f.cycles == 0U;
        if (!neutralOrigin || f.phase != static_cast<std::uint32_t>(EDM57::Phase::Terminal) ||
            f.destinationMm != 0.0 || r.expectedStale) return false;
        if (EDM57::IsPersistentProfile(static_cast<unsigned>(s.config.profile)))
            return !r.completed && r.persistentShort && !f.complete && f.persistentLatched &&
                r.reason == EDM56::Reason::PersistentShort && r.shortActive && r.feedInhibited &&
                r.rawVoltageV == 20.0 && f.cycles == 0U;
        const bool gateProven = EDM57::IsBlockedProfile(static_cast<unsigned>(s.config.profile)) ?
            (f.blockedCount == 1U && f.resumedReturnCount == 1U && f.blockedSinceUs && f.blockedSequence) :
            (f.blockedCount == 0U && f.resumedReturnCount == 0U);
        return gateProven && r.completed && !r.persistentShort && r.reason == EDM56::Reason::Complete &&
            f.complete && !f.persistentLatched && !f.returnBlocked && f.cycles == s.config.flush.cycleCount &&
            f.deepCycles == EDM57::ExpectedDeepCycles(s.config.flush, s.config.flush.cycleCount) &&
            f.slowApproachCount == s.config.flush.cycleCount && f.topDwellCount == s.config.flush.cycleCount;
    }
    if (s.config.profile == EDM28::Profile::P35GapDynamic || s.config.profile == EDM28::Profile::P46GapServoRecipe)
        return neutralOrigin && r.completed && !r.persistentShort && !r.expectedStale && r.reason == EDM56::Reason::Complete && r.cycles == 6U;
    if (s.config.profile == EDM28::Profile::P36GapDynamicPersistent || s.config.profile == EDM28::Profile::P47GapServoRecipePersistent)
        return neutralOrigin && !r.completed && r.persistentShort && !r.expectedStale && r.reason == EDM56::Reason::PersistentShort &&
            r.shortActive && r.feedInhibited && r.rawVoltageV == 20.0;
    return (s.config.profile == EDM28::Profile::P37GapDynamicStale || s.config.profile == EDM28::Profile::P48GapServoRecipeStale) && !r.completed && !r.persistentShort &&
        r.expectedStale && r.reason == EDM56::Reason::ExpectedStale &&
        r.kernelFault == EDM55::Fault::Stale && sourceAgeUs > 5000ULL;
}

void NCManager::PauseEDMGapDynamicSameThread(const char* reason) noexcept
{
    if (EDM28::IsPhysicalServoProfile(m_edmZFixture.config.profile))
    { EndEDMZFixtureSameThread("CANCELLED", reason ? reason :
        EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(m_edmZFixture.config.profile) ? "EDM76_HOLD_RESET_REQUIRED" : EDM28::IsPhysicalBidirectionalFlushFamilyProfile(m_edmZFixture.config.profile) ? "EDM75_HOLD_RESET_REQUIRED" : EDM28::IsPhysicalBidirectionalProfile(m_edmZFixture.config.profile) ? "EDM74_HOLD_RESET_REQUIRED" :
        EDM28::IsPhysicalRetreatCycleProfile(m_edmZFixture.config.profile) ? "EDM73_HOLD_RESET_REQUIRED" :
        EDM28::IsPhysicalRetreatProfile(m_edmZFixture.config.profile) ? "EDM72_HOLD_RESET_REQUIRED" :
        EDM28::IsPhysicalServoFlushProfile(m_edmZFixture.config.profile) ? "EDM71_HOLD_RESET_REQUIRED" :
        "EDM70_HOLD_RESET_REQUIRED"); return; }
    auto& s = m_edmZFixture; auto& d = m_edmGapDynamic;
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U)
        {
            // Completion has not yet left the NC receipt lifecycle. A HOLD
            // cannot turn that old receipt into a later START/M30 success.
            s.phase = EDMZFixturePhase::Receipt;
            EndEDMZFixtureSameThread("FAIL", "EDM56_TERMINAL_HOLD_RESET_REQUIRED");
        }
        return;
    }
    // A released stream has only historical SIM data. HOLD revokes its
    // completion instead of treating the frozen source as a new observation.
    if (d.terminalObserved || s.phase == EDMZFixturePhase::FinalStop ||
        s.phase == EDMZFixturePhase::Disarm || s.phase == EDMZFixturePhase::Receipt ||
        (ReadEDMZFixtureFeedbackSameThread(false) && d.generation &&
            s.feedback.dynamic.session == s.scope.session && s.feedback.dynamic.generation == d.generation &&
            s.feedback.dynamic.reason != EDM56::Reason::None && s.feedback.dynamic.reason != EDM56::Reason::Cancelled))
    { EndEDMZFixtureSameThread("FAIL", "EDM56_TERMINAL_HOLD_RESET_REQUIRED"); return; }
    if (m_gapDryRun.paused) return;
    // Preserve the old Disarm receipt before a queued first Arm is cancelled.
    // An applied current-session fault can never satisfy this classification.
    (void)ObserveEDMGapDynamicRetiredFaultSameThread();
    if (!s.originPinned && d.armIssued)
    { s.curvePreArmHold = true; s.curvePreArmSequence = d.armSequence; }
    if (d.armIssued) m_motion.RequestEDMZFixtureStop(s.scope.session);
    m_gapDryRun.paused = true; s.pending = false; s.resumeStartMs = 0ULL; d.authorityWaitMs = 0ULL;
    if (d.armIssued) s.phase = EDMZFixturePhase::ShortStop;
    LogEDMGapDynamicSameThread("HOLD", "WAIT", reason ? reason : "STOP_THEN_FRESH_CONTROLLER");
}

void NCManager::ValidateEDMGapDynamicSameThread()
{
    auto& s = m_edmZFixture; auto& d = m_edmGapDynamic;
    if (s.cleanupPending) ServiceEDMZFixtureCleanupSameThread();
    ServiceEDMZFixtureFailureAlarmSameThread();
    // Single-block and interlock HOLD may enter directly after a completed
    // wait. Keep that seam from reusing the preceding successful Q1 receipt.
    if (d.sequenceMode && m_gapDryRun.result == 1U &&
        (m_state == NCState::HOLD || m_edmState == EDMState::HOLD))
    { EndEDMZFixtureSameThread("CANCELLED", "EDM77_BOUNDARY_HOLD_RESET_REQUIRED"); return; }
    if (!m_gapDryRun.active) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { PauseEDMGapDynamicSameThread("HOLD"); return; }
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM || m_edmState == EDMState::ALARM ||
        Close_System_Com_flag || m_state == NCState::RESET_STATE || m_mode != NCOperationMode::MEMORY ||
        m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken || m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        m_gapDryRun.dispatch != m_waitingBlockDispatchId)
    { EndEDMZFixtureSameThread("CANCELLED", "EDM56_SCOPE_REVOKED"); return; }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "EDM56_CLOCK_INVALID"); return; }
    if (m_gapDryRun.paused)
    {
        if (ReadEDMZFixtureFeedbackSameThread(false) && s.feedback.dynamic.session == s.scope.session &&
            s.feedback.dynamic.generation == d.generation && d.generation &&
            s.feedback.dynamic.reason != EDM56::Reason::None && s.feedback.dynamic.reason != EDM56::Reason::Cancelled)
            EndEDMZFixtureSameThread("FAIL", "EDM56_HELD_CONTROLLER_RESET_REQUIRED");
        return; // START callback owns fresh authority/rearm.
    }
    if (now == d.validationClockMs)
    { if (++d.validationStalled >= 4096U) { EndEDMZFixtureSameThread("FAIL", "EDM56_CLOCK_NOT_ADVANCING"); return; } }
    else { d.validationClockMs = now; d.validationStalled = 0U; }
    if (!IsEDMZFixtureAuthorityCurrentSameThread(false))
    {
        // RT may revoke authority before this scan observes HOLD/RESET.
        // Contain immediately, but retain a bounded NC classification window.
        if (d.armIssued) m_motion.RequestEDMZFixtureStop(s.scope.session);
        if (!d.authorityWaitMs) d.authorityWaitMs = now;
        if (now - d.authorityWaitMs >= 250ULL || (d.admissionPending && now - d.admissionStartMs >= 1000ULL))
            EndEDMZFixtureSameThread("FAIL", "EDM56_AUTHORITY_TIMEOUT");
        return;
    }
    d.authorityWaitMs = 0ULL;
    if (EDM28::IsPhysicalServoProfile(s.config.profile))
    {
        const bool physicalRetreat = EDM28::IsPhysicalRetreatProfile(s.config.profile);
        const bool physicalRetreatCycle = EDM28::IsPhysicalRetreatCycleProfile(s.config.profile);
        const bool physicalEndurance = EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile);
        const bool physicalBidirectionalFlush = EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile);
        const bool physicalBidirectional = EDM28::IsPhysicalBidirectionalProfile(s.config.profile) || physicalBidirectionalFlush;
        const bool physicalNegative = EDM28::IsPhysicalNegativeProfile(s.config.profile);
        const auto binding = physicalBidirectional ? EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration) :
            physicalNegative ? EDM72::BindPhysicalRetreatRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration) :
            EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
        if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
            m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache || m_recipeEpoch != m_gapDryRun.epoch ||
            !m_recipeLease.Matches(m_gapDryRun.lease) || !binding.ready || !EDM59::SameRecipe(binding.recipe, s.config.servoRecipe))
        { EndEDMZFixtureSameThread("FAIL", physicalEndurance ? "EDM76_LIVE_RECIPE_CHANGED" : physicalBidirectionalFlush ? "EDM75_LIVE_RECIPE_CHANGED" : physicalBidirectional ? "EDM74_LIVE_RECIPE_CHANGED" : physicalRetreatCycle ? "EDM73_LIVE_RECIPE_CHANGED" : physicalRetreat ? "EDM72_LIVE_RECIPE_CHANGED" : EDM28::IsPhysicalServoFlushProfile(s.config.profile) ?
            "EDM71_LIVE_RECIPE_CHANGED" : "EDM70_LIVE_RECIPE_CHANGED"); return; }
        if (physicalBidirectionalFlush)
        {
            const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
            if (!flush.ready || !EDM57::SameRecipe(flush.recipe, s.config.flush) ||
                !EDM71::SameAdmission(flush.recipe, binding.recipe))
            { EndEDMZFixtureSameThread("FAIL", physicalEndurance ? "EDM76_LIVE_RECIPE_CHANGED" : "EDM75_LIVE_RECIPE_CHANGED"); return; }
        }
        if (EDM28::IsPhysicalServoFlushProfile(s.config.profile))
        {
            const auto flush = EDM71::BindPhysicalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
            if (!flush.ready || !EDM57::SameRecipe(flush.recipe, s.config.flush) ||
                !EDM71::SameAdmission(flush.recipe, binding.recipe))
            { EndEDMZFixtureSameThread("FAIL", "EDM71_LIVE_RECIPE_CHANGED"); return; }
        }
    }
    if (d.admissionPending && now - d.admissionStartMs >= 1000ULL)
    {
        std::uint64_t mask = 0ULL; MotionStopSettleSnapshot raw{};
        (void)IsEDMGapServoRunReadySameThread(&mask, &raw);
        LogEDMGapDynamicAdmissionSameThread(mask, raw, "TIMEOUT");
        EndEDMZFixtureSameThread("FAIL", "EDM56_ADMISSION_TIMEOUT");
    }
}

bool NCManager::ProcessEDMGapDynamicSameThread()
{
    auto& s = m_edmZFixture; auto& d = m_edmGapDynamic;
    if (EDM28::IsPhysicalServoProfile(s.config.profile) && !m_gapDryRun.active && m_gapDryRun.result == 2U &&
        m_state == NCState::RUN && !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
    { EndEDMZFixtureSameThread("FAIL", EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile) ? "EDM76_CANCELLED_CALLBACK_RESET_REQUIRED" : EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile) ? "EDM75_CANCELLED_CALLBACK_RESET_REQUIRED" : EDM28::IsPhysicalBidirectionalProfile(s.config.profile) ? "EDM74_CANCELLED_CALLBACK_RESET_REQUIRED" :
        EDM28::IsPhysicalRetreatCycleProfile(s.config.profile) ? "EDM73_CANCELLED_CALLBACK_RESET_REQUIRED" :
        EDM28::IsPhysicalRetreatProfile(s.config.profile) ? "EDM72_CANCELLED_CALLBACK_RESET_REQUIRED" :
        EDM28::IsPhysicalServoFlushProfile(s.config.profile) ? "EDM71_CANCELLED_CALLBACK_RESET_REQUIRED" : "EDM70_CANCELLED_CALLBACK_RESET_REQUIRED"); return false; }
    if (s.phase == EDMZFixturePhase::Finished || !m_gapDryRun.active) return false;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { PauseEDMGapDynamicSameThread("HOLD"); return false; }
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || now < s.lastClockMs)
    { EndEDMZFixtureSameThread("FAIL", "EDM56_CLOCK_INVALID"); return false; }
    if (!m_gapDryRun.paused && !d.admissionPending && now - s.lastClockMs >= 50ULL)
    { EndEDMZFixtureSameThread("FAIL", "CONTROL_DEADMAN_50MS"); return false; }
    if (now == s.lastClockMs)
    { if (++s.stalledCalls >= 4096U) { EndEDMZFixtureSameThread("FAIL", "EDM56_CLOCK_NOT_ADVANCING"); return false; } }
    else s.stalledCalls = 0U;
    s.lastClockMs = m_gapDryRun.lastServiceMs = now;
    if (!ReadEDMZFixtureFeedbackSameThread(false))
    { EndEDMZFixtureSameThread("FAIL", "EDM56_INVALID_RT_SOURCE"); return false; }
    const auto& f = s.feedback;
    if (!IsEDMGapDynamicConfigValidSameThread(s.originPinned) ||
        f.axisMapGeneration != s.scope.axisMapGeneration || f.configGeneration != s.scope.configGeneration ||
        f.pulsePerMm != s.config.pulsePerMm || f.referenceOffsetPulse != s.config.referenceOffsetPulse || f.hardwareSign != s.config.hardwareSign)
    { EndEDMZFixtureSameThread("FAIL", "EDM56_FROZEN_FRAME_CHANGED"); return false; }
    if (f.publicationSequence < s.publicationFloor || f.sampledTick < s.tickFloor ||
        (f.sampledTick == d.lastSourceTick && f.monotonicUs != d.lastSourceUs) ||
        (f.sampledTick > d.lastSourceTick && f.monotonicUs <= d.lastSourceUs))
    { EndEDMZFixtureSameThread("FAIL", "EDM56_RT_SOURCE_REGRESSED"); return false; }
    if (f.publicationSequence == s.publicationFloor || f.sampledTick == s.tickFloor)
    {
        if (!m_gapDryRun.paused && now - s.lastFreshMs >= 50ULL)
            EndEDMZFixtureSameThread("FAIL", "EDM56_RT_SOURCE_STALLED");
        return false;
    }
    s.publicationFloor = f.publicationSequence; s.tickFloor = f.sampledTick;
    d.lastSourceTick = f.sampledTick; d.lastSourceUs = f.monotonicUs; s.lastFreshMs = now;
    const bool retiredFault = ObserveEDMGapDynamicRetiredFaultSameThread();
    if (f.forceZeroOutput || (f.latchedFault && !retiredFault) || f.state == EDM28::State::Fault)
    { EndEDMZFixtureSameThread("FAIL", "EDM56_RT_LATCHED_FAULT"); return false; }
    if (m_gapDryRun.paused)
    {
        if (d.generation && f.dynamic.session == s.scope.session && f.dynamic.generation == d.generation &&
            f.dynamic.reason != EDM56::Reason::None && f.dynamic.reason != EDM56::Reason::Cancelled)
        { EndEDMZFixtureSameThread("FAIL", "EDM56_HELD_CONTROLLER_RESET_REQUIRED"); return false; }
        if (!s.resumeStartMs) s.resumeStartMs = now;
        if (now - s.resumeStartMs >= 1000ULL)
        { EndEDMZFixtureSameThread("FAIL", "EDM56_HELD_STOP_TIMEOUT"); return false; }
        if (!IsEDMGapDynamicStoppedSameThread()) return false;
        const auto epoch = m_motion.GetCurrentExecutionEpoch(); const auto lease = m_programMotionLease;
        if (!IsEDMZFixtureAuthorityCurrentSameThread(true) || f.currentEpoch != epoch ||
            f.currentOwner != static_cast<std::uint8_t>(lease.owner) || f.currentOwnerGeneration != lease.generation) return false;
        if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        { EndEDMZFixtureSameThread("FAIL", "EDM56_RESTART_EXHAUSTED"); return false; }
        if (f.capHeld && !s.originPinned)
        { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        if (!s.originPinned && d.armIssued)
        {
            if (m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)())
            { EndEDMZFixtureSameThread("FAIL", "EDM56_SESSION_EXHAUSTED"); return false; }
            s.scope.session = ++m_edmZFixtureSessionSerial; d.armIssued = false; d.armSequence = 0ULL;
        }
        if (epoch != m_motion.GetCurrentExecutionEpoch() || !m_motion.IsMotionOwnerLeaseCurrent(lease)) return false;
        // Only explicit START changes observer authority. The original
        // physical session and FIRST_ARM anchor survive an applied Arm.
        m_gapDryRun.epoch = epoch; m_gapDryRun.lease = lease; ++m_gapDryRun.restarts;
        m_gapDryRun.test = ++m_gapDryRunSerial; s.scope.executionEpoch = epoch;
        s.scope.owner = static_cast<std::uint8_t>(lease.owner); s.scope.ownerGeneration = lease.generation;
        d.generation = 0ULL; d.acknowledged = d.terminalObserved = false; d.terminal = EDM56::Feedback{};
        d.loggedPhase = d.loggedFlushPhase = (std::numeric_limits<std::uint32_t>::max)();
        d.loggedFlushBlocked = false; d.authorityWaitMs = 0ULL;
        s.windowStartMs = 0ULL; s.phaseStartMs = now; m_gapDryRun.paused = false;
        if (!s.originPinned)
        { d.admissionPending = true; d.admissionStartMs = now; s.phase = EDMZFixturePhase::PreArm; }
        else
        {
            d.admissionPending = false;
            if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
            { EndEDMZFixtureSameThread("FAIL", "EDM56_REARM_SUBMIT_FAILED"); return false; }
            d.armIssued = true; d.armSequence = s.pendingSequence; s.phase = EDMZFixturePhase::ArmPending;
        }
        LogEDMGapDynamicSameThread("RESTART", "WAIT", "FRESH_RT_CONTROLLER_FIRST_ORIGIN"); return false;
    }
    if (!IsEDMZFixtureAuthorityCurrentSameThread(false)) return false; // Validate bounds the HOLD race.
    if (f.currentEpoch != s.scope.executionEpoch || f.currentOwner != s.scope.owner || f.currentOwnerGeneration != s.scope.ownerGeneration)
    {
        if (!d.rtAuthorityWaitMs) d.rtAuthorityWaitMs = now;
        if (now - d.rtAuthorityWaitMs >= 250ULL) EndEDMZFixtureSameThread("FAIL", "EDM56_RT_AUTHORITY_TIMEOUT");
        return false;
    }
    d.rtAuthorityWaitMs = 0ULL;
    if (s.originPinned && (f.outerOriginPulse != s.originPulse ||
        std::fabs(f.actualPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.commandPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm ||
        std::fabs(f.planningPulse - s.originPulse) > s.config.outerHalfMm * s.config.pulsePerMm))
    { EndEDMZFixtureSameThread("FAIL", "EDM56_FIRST_ORIGIN_BOUNDARY"); return false; }
    if (s.phase == EDMZFixturePhase::PreArm)
    {
        std::uint64_t mask = 0ULL; MotionStopSettleSnapshot raw{};
        (void)IsEDMGapServoRunReadySameThread(&mask, &raw);
        if (mask & ~(1ULL << 9U))
        { LogEDMGapDynamicAdmissionSameThread(mask, raw, "REJECT_WAIT"); EndEDMZFixtureSameThread("FAIL", "EDM56_ADMISSION_REVOKED"); return false; }
        // A recognized retired fault is not permission to move. Require a
        // fresh physical proof as well as RT's 200-cycle idle admission;
        // transient proof renewal remains bounded by admissionStartMs. A
        // cancelled first Arm never moved and cannot renew StopProof: its
        // exact previously saved Disarm plus fresh armReady were checked
        // by the retired-fault classifier instead.
        if (mask || !f.armReady || (retiredFault && !f.queuedCancellation &&
            (!f.stopProven || f.stableCycles < s.config.settleCycles))) return false;
        if (s.scope.executionEpoch != m_motion.GetCurrentExecutionEpoch() ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)) return false;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Arm))
        { EndEDMZFixtureSameThread("FAIL", "EDM56_ARM_SUBMIT_FAILED"); return false; }
        d.armIssued = true; d.armSequence = s.pendingSequence; d.admissionPending = false;
        s.phase = EDMZFixturePhase::ArmPending; s.phaseStartMs = now;
        LogEDMGapDynamicAdmissionSameThread(0ULL, raw, "ARM_SUBMITTED"); return false;
    }
    if (!EDM28::SameScope(f.scope, s.scope))
    {
        if (s.phase != EDMZFixturePhase::ArmPending || now - s.phaseStartMs >= 50ULL)
            EndEDMZFixtureSameThread("FAIL", "EDM56_RT_SCOPE_MISMATCH");
        return false;
    }
    if (s.phase == EDMZFixturePhase::ArmPending)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Arm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "EDM56_ARM_APPLICATION_TIMEOUT"); return false; }
        if (!IsEDMGapDynamicConfigValidSameThread(true) || !f.capHeld || !f.originValid || !f.frameCurrent ||
            !f.axisIdle || f.state != EDM28::State::Armed || !std::isfinite(f.outerOriginPulse) ||
            (s.originPinned && f.outerOriginPulse != s.originPulse))
        { EndEDMZFixtureSameThread("FAIL", "EDM56_ARM_RECEIPT_INVALID"); return false; }
        if (!s.originPinned) { s.originPulse = f.outerOriginPulse; s.originPinned = true; }
        s.curveBoundaryProof = f.proofSequence;
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::ServoStart))
        { EndEDMZFixtureSameThread("FAIL", "EDM56_SERVO_START_SUBMIT_FAILED"); return false; }
        d.generation = s.pendingSequence; s.phase = EDMZFixturePhase::Positive; s.phaseStartMs = now;
        LogEDMGapDynamicSameThread("ARM", "APPLIED", "SERVO_START_SUBMITTED"); return false;
    }
    if (s.phase == EDMZFixturePhase::Positive)
    {
        const auto& r = f.dynamic;
        if (!d.acknowledged)
        {
            if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::ServoStart) ||
                r.session != s.scope.session || r.generation != d.generation || !r.startedTick || !r.startedUs)
            { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "EDM56_SERVO_START_TIMEOUT"); return false; }
            d.acknowledged = true; s.motionApplied = true; s.pending = false;
            s.windowStartMs = now; d.startedTick = r.startedTick; d.startedUs = r.startedUs;
            LogEDMGapDynamicSameThread("BEGIN", "RUNNING", "RT_CONTINUOUS_SERVO");
        }
        if (r.session != s.scope.session || r.generation != d.generation ||
            r.profile != static_cast<std::uint8_t>(s.config.profile) || r.originPulse != s.originPulse ||
            r.startedTick != d.startedTick || r.startedUs != d.startedUs || r.tick > f.sampledTick || r.nowUs > f.monotonicUs)
        { EndEDMZFixtureSameThread("FAIL", "EDM56_CONTROLLER_IDENTITY_CHANGED"); return false; }
        if (r.active && r.phase == static_cast<std::uint32_t>(EDM56::Phase::OriginSettle))
            LogEDMGapOriginSettleSameThread("WAIT", false);
        if (r.terminal)
        {
            if (!IsEDMGapDynamicTerminalSameThread())
            { EndEDMZFixtureSameThread("FAIL", "EDM56_CONTROLLER_TERMINAL_INVALID"); return false; }
            d.terminal = r; d.terminalObserved = true;
            if (r.expectedStale)
            {
                LogEDMGapDynamicSameThread("EXPECTED_FAULT", "AL2020", "STALE_PHYSICAL_STOP_RESET_REQUIRED");
                EndEDMZFixtureSameThread("FAIL", "EDM56_EXPECTED_STALE_RESET_REQUIRED"); return false;
            }
            s.phase = EDMZFixturePhase::FinalStop; s.phaseStartMs = now;
            LogEDMGapDynamicSameThread("RT_TERMINAL", "WAIT", EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile) ? "LIVE_AD_BIDIRECTIONAL_FLUSH_60S_ORIGIN_COMPLETE" : EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile) ? "LIVE_AD_BIDIRECTIONAL_FLUSH_10S_ORIGIN_COMPLETE" : EDM28::IsPhysicalBidirectionalProfile(s.config.profile) ?
                "LIVE_AD_BIDIRECTIONAL_10S_ORIGIN_COMPLETE" : EDM28::IsPhysicalRetreatCycleProfile(s.config.profile) ?
                "REAL_AD_RETREAT_CYCLES_10S_ORIGIN_COMPLETE" : EDM28::IsPhysicalRetreatProfile(s.config.profile) ?
                "REAL_AD_RETREAT_10S_ENDPOINT_COMPLETE" : r.persistentShort ? "SHORT_RETURN_FIRST_ORIGIN" : "SIX_RT_CYCLES_COMPLETE");
            return false;
        }
        if (!r.active || r.needFormalStop || f.stopLatched || now - s.windowStartMs >=
            (EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile) ? 65000ULL : 30000ULL))
        { EndEDMZFixtureSameThread("FAIL", "EDM56_CONTROLLER_STOP_OR_TIMEOUT"); return false; }
        if (r.phase != d.loggedPhase || (EDM28::IsGapFlushProfile(s.config.profile) &&
            (r.flush.phase != d.loggedFlushPhase || r.flush.returnBlocked != d.loggedFlushBlocked)))
        {
            d.loggedPhase = r.phase; d.loggedFlushPhase = r.flush.phase; d.loggedFlushBlocked = r.flush.returnBlocked;
            LogEDMGapDynamicSameThread("PHASE", "RUNNING", "RT_SOURCE_AND_PHYSICAL_READBACK");
        }
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Heartbeat))
            EndEDMZFixtureSameThread("FAIL", "EDM56_HEARTBEAT_SUBMIT_FAILED");
        return false;
    }
    if (!IsEDMGapDynamicTerminalSameThread())
    { EndEDMZFixtureSameThread("FAIL", "EDM56_TERMINAL_IDENTITY_CHANGED"); return false; }
    if (s.phase == EDMZFixturePhase::FinalStop)
    {
        const auto completion = EvaluateEDMGapDynamicCompletionSameThread(EDM49::Stage::StopSettled);
        if (completion.status == EDM49::Status::Wait)
        { if (now - s.phaseStartMs >= 1000ULL) EndEDMZFixtureSameThread("FAIL", "EDM56_STOP_SETTLE_TIMEOUT"); return false; }
        if (!completion.Ready())
        { EndEDMZFixtureSameThread("FAIL", "EDM56_STOP_RECEIPT_INVALID"); return false; }
        s.terminalStopAppliedSequence = f.stopAppliedSequence; s.terminalStopAppliedTick = f.stopAppliedTick;
        s.terminalStopAppliedMonotonicUs = f.stopAppliedMonotonicUs; s.receiptProof = f.proofSequence;
        LogEDMGapDynamicSameThread("STOP_SETTLED", "PROVEN", EDM28::IsPhysicalRetreatProfile(s.config.profile) ?
            "GENUINE_PHYSICAL_ENDPOINT_STOP" : "GENUINE_PHYSICAL_ORIGIN_STOP");
        if (!SubmitEDMZFixtureSameThread(EDM28::RequestKind::Disarm))
        { EndEDMZFixtureSameThread("FAIL", "EDM56_DISARM_SUBMIT_FAILED"); return false; }
        s.phase = EDMZFixturePhase::Disarm; s.phaseStartMs = now; return false;
    }
    if (s.phase == EDMZFixturePhase::Disarm || s.phase == EDMZFixturePhase::Receipt)
    {
        if (!IsEDMZFixtureAppliedSameThread(EDM28::RequestKind::Disarm))
        { if (now - s.phaseStartMs >= 50ULL) EndEDMZFixtureSameThread("FAIL", "EDM56_DISARM_APPLICATION_TIMEOUT"); return false; }
        const auto stage = s.phase == EDMZFixturePhase::Disarm ? EDM49::Stage::Released : EDM49::Stage::FreshReleased;
        const auto completion = EvaluateEDMGapDynamicCompletionSameThread(stage);
        if (completion.status == EDM49::Status::Invalid)
        { EndEDMZFixtureSameThread("FAIL", "EDM56_RELEASE_RECEIPT_INVALID"); return false; }
        if (s.phase == EDMZFixturePhase::Disarm)
        {
            if (!completion.Ready()) return false;
            s.receiptPublication = f.publicationSequence; s.receiptTick = f.sampledTick; s.receiptUs = f.monotonicUs;
            s.phase = EDMZFixturePhase::Receipt; s.phaseStartMs = now;
            LogEDMGapDynamicSameThread("RELEASED", "WAIT", "AWAIT_STRICTLY_NEWER_PHYSICAL_RECEIPT"); return false;
        }
        if (now - s.phaseStartMs >= 50ULL)
        { EndEDMZFixtureSameThread("FAIL", "EDM56_FRESH_RELEASE_TIMEOUT"); return false; }
        if (!completion.Ready()) return false;
        if (d.terminal.persistentShort)
        {
            LogEDMGapDynamicSameThread("FRESH_RELEASED", "AL4001", "PERSISTENT_SHORT_AT_ORIGIN");
            s.pending = false;
            RejectGapDryRunSameThread(AlarmManager::SHORT_CIRCUIT, m_gapDryRun.sourceLine, "PERSISTENT_SHORT_AT_ORIGIN");
            return false;
        }
        if (d.strictSequenceMode)
        {
            auto& plan = m_edmPhysicalSequencePlan;
            const int pc = GetActiveDispatchPC();
            if (!plan.plan.valid || !plan.legPending || !plan.plan.count ||
                plan.plan.count > EDM77::MaximumFixtures || plan.next >= plan.plan.count ||
                plan.run != m_gapDryRun.run || plan.cache != m_gapDryRun.cache ||
                plan.epoch != m_gapDryRun.epoch || !plan.lease.Matches(m_gapDryRun.lease) ||
                plan.dispatch != m_gapDryRun.dispatch || plan.firstPC < 0 || pc < plan.firstPC ||
                plan.plan.checkpoints[plan.next].sourceOffset != static_cast<unsigned>(pc-plan.firstPC) ||
                plan.plan.checkpoints[plan.next].profile != static_cast<unsigned>(s.config.profile) ||
                !EDM57::SameRecipe(plan.plan.checkpoints[plan.next].flush, s.config.flush) ||
                !EDM59::SameRecipe(plan.plan.checkpoints[plan.next].servo, s.config.servoRecipe))
            { EndEDMZFixtureSameThread("FAIL", "EDM79_PLAN_COMPLETION_SCOPE"); return false; }
            ++plan.next; plan.legPending = false;
            RtPrintf("[EDM79] event=%s result=PASS completed=%u count=%u session=%llu run=%llu dispatch=%llu proof=FRESH_RELEASED discharge=0\n",
                plan.next == plan.plan.count ? "PLAN_COMPLETE" : "PLAN_ADVANCE", plan.next, plan.plan.count,
                static_cast<unsigned long long>(s.scope.session), static_cast<unsigned long long>(plan.run),
                static_cast<unsigned long long>(plan.dispatch));
        }
        s.phase = EDMZFixturePhase::Finished; m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        if (d.sequenceMode)
        {
            d.sequenceReceiptPublication = f.publicationSequence;
            d.sequenceReceiptTick = f.sampledTick; d.sequenceReceiptUs = f.monotonicUs;
            RtPrintf("[EDM77] event=SEGMENT_RELEASE result=PASS session=%llu run=%llu dispatch=%llu stopSeq=%llu disarmSeq=%llu receiptPub=%llu receiptTick=%llu receiptUs=%llu next=FRESH_BOUNDARY_REQUIRED discharge=0\n",
                static_cast<unsigned long long>(s.scope.session), static_cast<unsigned long long>(m_gapDryRun.run),
                static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned long long>(s.terminalStopAppliedSequence),
                static_cast<unsigned long long>(s.pendingSequence), static_cast<unsigned long long>(f.publicationSequence),
                static_cast<unsigned long long>(f.sampledTick), static_cast<unsigned long long>(f.monotonicUs));
        }
        LogEDMGapDynamicSameThread("PASS", "PASS", EDM28::IsPhysicalServoProfile(s.config.profile) ?
            (EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(s.config.profile) ? "PHYSICAL_PN_FLUSH_60S_FRESH_ORIGIN_RELEASE" : EDM28::IsPhysicalBidirectionalFlushFamilyProfile(s.config.profile) ? "PHYSICAL_PN_FLUSH_10S_FRESH_ORIGIN_RELEASE" : EDM28::IsPhysicalBidirectionalProfile(s.config.profile) ? "PHYSICAL_PN_10S_FRESH_ORIGIN_RELEASE" :
                EDM28::IsPhysicalRetreatCycleProfile(s.config.profile) ? "PHYSICAL_RETREAT_CYCLES_10S_FRESH_ORIGIN_RELEASE" :
                EDM28::IsPhysicalRetreatProfile(s.config.profile) ? "PHYSICAL_AD_RETREAT_10S_FRESH_ENDPOINT_RELEASE" :
                EDM28::IsPhysicalServoFlushProfile(s.config.profile) ? "PHYSICAL_AD_FLUSH_10S_FRESH_ORIGIN_RELEASE" :
                "PHYSICAL_AD_10S_FRESH_ORIGIN_RELEASE") : "SIX_CYCLES_FRESH_PHYSICAL_RELEASE"); return true;
    }
    EndEDMZFixtureSameThread("FAIL", "EDM56_INVALID_NC_PHASE"); return false;
}

EDM49::Result NCManager::EvaluateEDMGapDynamicCompletionSameThread(EDM49::Stage stage) const noexcept
{
    const auto& s = m_edmZFixture; const auto& f = s.feedback;
    EDM49::Result invalid{}; invalid.selectedMask = 1U << 2U;
    if (!IsEDMGapDynamicConfigValidSameThread(true)) { invalid.reason = EDM49::Reason::InvalidRule; return invalid; }
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
    if (EDM28::IsPhysicalRetreatProfile(s.config.profile))
    {
        // P58 stops at its retreat endpoint. Preserve both original anchor
        // identities for EDM49, bound them by the known .03 mm excursion,
        // and independently demand the unchanged .001 mm endpoint tolerance.
        // No other profile uses this endpoint criterion or anchor envelope.
        rule.positionTolerance = (EDM72::RetreatDepthMm + s.config.positionToleranceMm) * s.config.pulsePerMm;
    }
    // Preserve the original mm/s comparison, including its rounding. No PDO-zero condition.
    rule.maximumCommandRate = 1.0 / s.config.pulsePerMm;
    rule.settleTicks = s.config.settleCycles; rule.settleUs = s.config.settleCycles * EDM28::CycleUs;
    rule.proofFloor = s.curveBoundaryProof; rule.stopSourceTickFloor = m_edmGapDynamic.startedTick;
    rule.expectedStop = { s.terminalStopAppliedSequence, s.terminalStopAppliedTick, s.terminalStopAppliedMonotonicUs };
    rule.disarmSequence = s.pending && s.pendingKind == EDM28::RequestKind::Disarm ? s.pendingSequence : 0ULL;
    rule.releasedProof = s.receiptProof;
    rule.requireTargetAtOrigin = false; // Continuous servo has no finite Position endpoint.
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
    auto result = EDM49::Evaluate(request, evidence);
    if (result.Ready() && EDM28::IsPhysicalRetreatProfile(s.config.profile))
    {
        // Preserve EDM49's AwaitStop/settling Wait. Once its physical receipt
        // is ready, the P58 endpoint must also satisfy the original tolerance
        // before this method can return Ready at any lifecycle stage.
        const double endpointPulse = s.originPulse + EDM72::RetreatDepthMm * s.config.pulsePerMm;
        const double endpointTolerance = s.config.positionToleranceMm * s.config.pulsePerMm;
        if (!std::isfinite(endpointPulse) || !std::isfinite(f.actualPulse) ||
            !std::isfinite(f.commandPulse) || !std::isfinite(f.planningPulse) ||
            std::fabs(f.actualPulse - endpointPulse) > endpointTolerance ||
            std::fabs(f.commandPulse - endpointPulse) > endpointTolerance ||
            std::fabs(f.planningPulse - endpointPulse) > endpointTolerance)
        { result.status = EDM49::Status::Invalid; result.reason = EDM49::Reason::Position; }
    }
    // A terminal generated earlier in this same RT tick may cause Stop.
    // Require that actual application to follow the terminal cause without
    // pretending it must occur on a strictly later callback.
    if (result.Ready() && (!m_edmGapDynamic.terminalObserved ||
        f.stopAppliedTick < m_edmGapDynamic.terminal.tick ||
        f.stopAppliedMonotonicUs < m_edmGapDynamic.terminal.nowUs))
    { result.status = EDM49::Status::Invalid; result.reason = EDM49::Reason::StopStamp; }
    return result;
}


