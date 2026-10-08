#include "NCManager.h"
#include "EDMProcessTuningCodec.h"
#include "EDMAutomaticFlushRecipe.h"
#include "AlarmManager.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <windows.h>
#include <rtapi.h>

// EDM20: NC owner (nominal 10 ms) shadow preview. This translation unit has
// no Motion submission, axis/PDO/pulse write, allocation or file operation.
// NC RUN does not imply electrical discharge is active. Both threshold
// scenarios are observed explicitly; neither authorizes a physical output.
namespace
{
    using ShadowReason = NCManager::EDMProcessShadowReason;

    const char* EDM20ReasonName(ShadowReason value) noexcept
    {
        switch (value)
        {
        case ShadowReason::ConfigUnavailable: return "CONFIG_UNAVAILABLE";
        case ShadowReason::Closing: return "CLOSING";
        case ShadowReason::ClockInvalid: return "CLOCK_INVALID";
        case ShadowReason::GapInvalid: return "GAP_INVALID";
        case ShadowReason::RecoveryDisplay: return "RECOVERY_DISPLAY";
        case ShadowReason::RecipeUnavailable: return "RECIPE_UNAVAILABLE";
        case ShadowReason::MachineProfileMismatch: return "MACHINE_PROFILE_MISMATCH";
        case ShadowReason::SemanticUnitMismatch: return "SEMANTIC_UNIT_MISMATCH";
        case ShadowReason::Alarm: return "ALARM";
        case ShadowReason::Reset: return "RESET";
        case ShadowReason::Hold: return "HOLD";
        case ShadowReason::NotReady: return "NOT_READY";
        case ShadowReason::AwaitFreshSample: return "AWAIT_FRESH_SAMPLE";
        case ShadowReason::SampleIdentity: return "SAMPLE_IDENTITY";
        case ShadowReason::CurveInvalid: return "CURVE_INVALID";
        case ShadowReason::Ready: return "READY";
        }
        return "UNKNOWN";
    }

    const char* EDM20ShortName(EDMGapServo::ShortState value) noexcept
    {
        switch (value)
        {
        case EDMGapServo::ShortState::Clear: return "CLEAR";
        case EDMGapServo::ShortState::Entering: return "ENTERING";
        case EDMGapServo::ShortState::Active: return "ACTIVE";
        case EDMGapServo::ShortState::Exiting: return "EXITING";
        case EDMGapServo::ShortState::Invalid: return "INVALID";
        }
        return "UNKNOWN";
    }

    const EDMRecipe::FieldDefinition* EDM20SemanticField(const EDMRecipe::Catalog& catalog,
        const char* semantic, const char* unit) noexcept
    {
        for (const auto& field : catalog.fields)
        {
            if (std::strcmp(field.semanticKey, semantic) != 0) continue;
            return field.enabled && field.unitConfirmed && std::strcmp(field.unit, unit) == 0 ?
                &field : nullptr;
        }
        return nullptr;
    }

    bool EDM20SameSource(const EDMGapInput::Snapshot& a, const EDMGapInput::Snapshot& b) noexcept
    {
        if (a.configuredSource != b.configuredSource || a.gap.source != b.gap.source ||
            a.profileRevision != b.profileRevision) return false;
        // The synthetic stream is independent of all physical ADC diagnostics.
        if (a.configuredSource == EDMGap::Source::SIMULATED) return true;
        return a.adIndex == b.adIndex && a.deviceId == b.deviceId && a.channelId == b.channelId &&
            a.pdoOffset == b.pdoOffset && a.calibrationRevision == b.calibrationRevision;
    }

    bool EDM20SameSample(const EDMGapInput::Snapshot& a, const EDMGapInput::Snapshot& b) noexcept
    {
        return a.sampleSequence == b.sampleSequence && a.sampledAtMs == b.sampledAtMs &&
            a.voltageMv == b.voltageMv &&
            (a.configuredSource == EDMGap::Source::SIMULATED || a.rawCode == b.rawCode);
    }

    void EDM20PrintShadowLine(const char (&buffer)[512], int length) noexcept
    {
        // RTSS RtPrintf does not support floating-point conversions and limits
        // one output to 512 characters. CRT snprintf formats into a fixed local
        // buffer first; only the resulting string crosses the RtPrintf boundary.
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(buffer))
        {
            RtPrintf("[EDM20] event=SHADOW_FORMAT_ERROR logger=FIX2 physicalPermit=0 discharge=0\n");
            return;
        }
        RtPrintf("%s", buffer);
    }

    // Read-only binding of the selected COND. This does not arm an automatic
    // machining cycle: NC RUN alone is not electrical machining permission.
    void EDM23LogAutomaticRecipe(const EDMRecipe::Controller& recipe,
        std::uint32_t profileId, std::uint32_t configRevision) noexcept
    {
        const EDM23::RecipeBindingSnapshot binding = EDM23::BindRecipe(recipe, profileId);
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM23] event=LIVE_RECIPE result=%s reason=%.40s field=%u config=%u COND=%u E=%u generation=%llu E5ms=%llu E6mm=%.9g E7pct=%.9g E21cycles=%u E22x=%.9g automaticActive=0 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            binding.ready ? "READY" : "BLOCKED", EDM23::RecipeBindingErrorName(binding.error),
            static_cast<unsigned>(binding.failedFieldId), static_cast<unsigned>(configRevision),
            static_cast<unsigned>(binding.tableId), static_cast<unsigned>(binding.eCode),
            static_cast<unsigned long long>(binding.generation), static_cast<unsigned long long>(binding.workTimeMs),
            binding.jumpHeightMm, binding.speedOverridePercent,
            static_cast<unsigned>(binding.deepFlushCycleInterval), binding.deepFlushHeightMultiplier);
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
            RtPrintf("[EDM23] event=LIVE_RECIPE result=BLOCKED reason=LOG_FORMAT automaticActive=0 physicalPermit=0 discharge=0\n");
        else RtPrintf("%s", line);
    }

    bool EDM21CriticalGate(ShadowReason reason) noexcept
    {
        return reason == ShadowReason::Hold || reason == ShadowReason::Reset ||
            reason == ShadowReason::Alarm || reason == ShadowReason::Closing;
    }

    void EDM21LogShadowGateTransition(const NCManager::EDMProcessShadowSnapshot& previous,
        const NCManager::EDMProcessShadowSnapshot& next) noexcept
    {
        // Critical transitions can be shorter than the regular 1-second
        // snapshot interval. Observe the already computed result only; this
        // diagnostic neither changes the gate nor controls a physical output.
        if (previous.reason == next.reason ||
            (!EDM21CriticalGate(previous.reason) && !EDM21CriticalGate(next.reason))) return;
        char line[512]{};
        const int count = std::snprintf(line, sizeof(line),
            "[EDM21] event=SHADOW_GATE prev=%s next=%s seq=%llu observedMs=%llu config=%u COND=%u E=%u curvePreviewZero=%u machiningPreviewZero=%u physicalPermit=0 discharge=0\n",
            EDM20ReasonName(previous.reason), EDM20ReasonName(next.reason),
            static_cast<unsigned long long>(next.sampleSequence),
            static_cast<unsigned long long>(next.observedAtMs),
            static_cast<unsigned>(next.configRevision), static_cast<unsigned>(next.tableId),
            static_cast<unsigned>(next.eCode), next.previewSpeedMmPerMin == 0.0 ? 1U : 0U,
            next.machiningScenarioSpeedMmPerMin == 0.0 ? 1U : 0U);
        if (count >= 0 && static_cast<std::size_t>(count) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM21] event=SHADOW_GATE_FORMAT_ERROR physicalPermit=0 discharge=0\n");
    }
}

bool NCManager::InstallEDMProcessProfileBeforeStart(const EDM20::ProcessProfile& profile,
    const AxisContext* axes, std::size_t axisCount) noexcept
{
    if (m_edmProcessInstallAttempted)
    {
        m_edmProcessReady = false;
        m_edmProcessShadow = EDMProcessShadowSnapshot{};
        return false;
    }
    m_edmProcessInstallAttempted = true;
    if (!EDM20::ValidateProcessProfile(profile) || axes == nullptr || axisCount == 0U ||
        axisCount > EDM20::AxisGainSlotCount) return false;
    std::array<EDM20::GainResolution, 8> discharge{}, flush{};
    for (std::size_t axis = 0U; axis < axisCount; ++axis)
    {
        EDM20::AxisGains cnc{};
        cnc.Kp = axes[axis].Pid_G00.Kp; cnc.Ki = axes[axis].Pid_G00.Ki;
        cnc.Kd = axes[axis].Pid_G00.Kd; cnc.Kvff = axes[axis].Pid_G00.Kvff;
        m_edmCncGainBaseline[axis] = cnc;
        discharge[axis] = EDM20::ResolveAxisGains(profile.axisGains, EDM20::GainBank::Discharge, axis, cnc);
        flush[axis] = EDM20::ResolveAxisGains(profile.axisGains, EDM20::GainBank::Flush, axis, cnc);
        if (discharge[axis].status != EDM20::GainResolveStatus::Ready ||
            flush[axis].status != EDM20::GainResolveStatus::Ready) return false;
    }
    m_edmProcessProfile = profile;
    m_edmDischargeGains = discharge; m_edmFlushGains = flush;
    m_edmProcessActiveAxisMask = (1U << static_cast<unsigned>(axisCount)) - 1U;
    m_edmProcessTuningGeneration = 1ULL;
    m_edmProcessReady = true;
    return true;
}

void NCManager::ReadEDMProcessShadowSnapshotSameThread(EDMProcessShadowSnapshot& output) const noexcept
{
    output = m_edmProcessShadow;
}

void NCManager::ObserveEDMProcessShadowSameThread(const EDMGapInput::Snapshot& gap) noexcept
{
    // EDM24 keeps this scan's raw observation separate from the validated
    // high-water cache. Even invalid/source-changed observations must reach
    // the live shadow session without replaying the previous valid identity.
    m_edmProcessObservedGap = gap;
    const bool firstObservation = !m_edmProcessBootTraceSeen;
    if (firstObservation) { m_edmProcessBootTraceSeen = true; RtPrintf("[EDM20_FIX1][BOOT] phase=SHADOW_FIRST_ENTER\n"); }
    EDMProcessShadowSnapshot next{};
    next.profileReady = m_edmProcessReady;
    next.configRevision = m_edmProcessReady ? m_edmProcessProfile.revision : 0U;
    next.machineProfileId = m_edmProcessReady ? m_edmProcessProfile.machineProfileId : 0U;
    next.activeAxisMask = m_edmProcessReady ? m_edmProcessActiveAxisMask : 0U;
    next.dischargeGains = m_edmDischargeGains; next.flushGains = m_edmFlushGains;
    next.observedAtMs = gap.observedAtMs; next.sampleSequence = gap.sampleSequence;
    next.source = gap.configuredSource;
    const EDMRecipe::Summary summary = m_recipe.Current();
    next.tableId = summary.tableId; next.eCode = summary.eCode;
    next.recipeGeneration = summary.generation;
    const EDMRecipe::Catalog* catalog = m_recipe.GetCatalog();
    const bool clockValid = gap.ownerClockValid &&
        (!m_edmProcessHaveClock || gap.observedAtMs >= m_edmProcessLastObservedMs);
    if (clockValid)
    {
        m_edmProcessHaveClock = true;
        m_edmProcessLastObservedMs = gap.observedAtMs;
    }
    const bool live = gap.configValid && gap.liveVoltageValid && gap.ageKnown &&
        gap.sampleSequence != 0ULL && gap.sampledAtMs <= gap.observedAtMs &&
        gap.ageMs == gap.observedAtMs - gap.sampledAtMs && gap.ageMs <= gap.maxAgeMs &&
        gap.gap.quality == EDMGap::Quality::VALID && gap.gap.source == gap.configuredSource &&
        gap.gap.sequence == gap.sampleSequence && gap.gap.sampledAtMs == gap.sampledAtMs &&
        gap.gap.voltageMv == gap.voltageMv &&
        (gap.configuredSource == EDMGap::Source::SIMULATED ||
            (gap.configuredSource == EDMGap::Source::PHYSICAL && gap.calibrationConfirmed &&
                gap.inputStatus == EDMGapInput::InputStatus::Valid));
    next.reason = ShadowReason::Ready;
    if (!m_edmProcessReady) next.reason = ShadowReason::ConfigUnavailable;
    else if (Close_System_Com_flag) next.reason = ShadowReason::Closing;
    else if (!clockValid) next.reason = ShadowReason::ClockInvalid;
    else if (!live) next.reason = ShadowReason::GapInvalid;
    else if (m_recipeRecoveryDisplayOnly) next.reason = ShadowReason::RecoveryDisplay;
    else if (catalog == nullptr || !m_recipe.Ready() || !summary.rowSelected ||
        !summary.tableSelected || summary.generation == 0ULL || catalog->schemaVersion != 2U ||
        catalog->mode != EDMRecipe::Mode::ShadowOnly) next.reason = ShadowReason::RecipeUnavailable;
    else if (catalog->profileId != m_edmProcessProfile.machineProfileId)
        next.reason = ShadowReason::MachineProfileMismatch;
    else
    {
        const EDMRecipe::FieldDefinition* servo = EDM20SemanticField(*catalog, "MachiningServoOverride", "%");
        const EDMRecipe::FieldDefinition* reference = EDM20SemanticField(*catalog, "GapVoltageSetpoint", "V");
        EDMRecipe::ParameterSnapshot servoValue{}, referenceValue{};
        if (servo == nullptr || reference == nullptr ||
            !m_recipe.ReadField(servo->id, servoValue) || !servoValue.configured ||
            !m_recipe.ReadField(reference->id, referenceValue) || !referenceValue.configured ||
            servoValue.actualValue < servo->minimum || servoValue.actualValue > servo->maximum ||
            referenceValue.actualValue < reference->minimum || referenceValue.actualValue > reference->maximum)
            next.reason = ShadowReason::SemanticUnitMismatch;
        else
        {
            next.recipeReady = true;
            next.voltageV = static_cast<double>(gap.voltageMv) / 1000.0;
            next.referenceV = static_cast<double>(referenceValue.actualValue) / EDMRecipe::Scale;
            next.overridePercent = static_cast<double>(servoValue.actualValue) / EDMRecipe::Scale;
            if (next.referenceV < 0.0 || next.referenceV > 1000.0)
                next.reason = ShadowReason::CurveInvalid;
            else
            {
                next.curve = EDMGapServo::Evaluate(m_edmProcessProfile.servo,
                    next.voltageV, next.referenceV, next.overridePercent);
                if (!next.curve.valid) next.reason = ShadowReason::CurveInvalid;
            }
        }
    }

    if (next.reason == ShadowReason::Ready)
    {
        if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM || m_edmState == EDMState::ALARM)
            next.reason = ShadowReason::Alarm;
        else if (m_state == NCState::RESET_STATE || m_resetContinuationPhase != ResetContinuationPhase::IDLE ||
            m_resetSafetyOutputHoldActive) next.reason = ShadowReason::Reset;
        else if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD) next.reason = ShadowReason::Hold;
        else if (m_state == NCState::NOT_READY || m_edmState == EDMState::NOT_READY ||
            !m_externalReadyInterlock) next.reason = ShadowReason::NotReady;
    }

    const bool sourceChanged = !m_edmProcessHaveGap || !EDM20SameSource(gap, m_edmProcessLastGap);
    const bool recipeChanged = m_edmProcessLastRecipeGeneration != summary.generation;
    const bool serviceGap = !sourceChanged && m_edmProcessHaveGap &&
        ((gap.sampledAtMs > m_edmProcessLastGap.sampledAtMs &&
            gap.sampledAtMs - m_edmProcessLastGap.sampledAtMs > gap.maxAgeMs) ||
            (gap.observedAtMs > m_edmProcessLastGap.observedAtMs &&
                gap.observedAtMs - m_edmProcessLastGap.observedAtMs > gap.maxAgeMs));
    if (next.reason == ShadowReason::Ready && serviceGap)
        next.reason = ShadowReason::AwaitFreshSample;
    if (next.reason != ShadowReason::Ready || sourceChanged || recipeChanged)
    {
        m_edmIdleShortDetector.Reset(); m_edmMachiningShortDetector.Reset();
        m_edmIdleShortResult = EDMGapServo::ShortResult{};
        m_edmMachiningShortResult = EDMGapServo::ShortResult{};
        m_edmProcessNeedFreshSample = true;
    }

    if (next.reason == ShadowReason::Ready)
    {
        const bool duplicate = !sourceChanged && gap.sampleSequence == m_edmProcessLastGap.sampleSequence;
        const bool sequenceBad = !sourceChanged &&
            (gap.sampleSequence < m_edmProcessLastGap.sampleSequence ||
                (duplicate && !EDM20SameSample(gap, m_edmProcessLastGap)) ||
                gap.sampledAtMs < m_edmProcessLastGap.sampledAtMs);
        if (sequenceBad)
        {
            next.reason = ShadowReason::SampleIdentity;
            m_edmIdleShortDetector.Reset(); m_edmMachiningShortDetector.Reset();
            m_edmIdleShortResult = EDMGapServo::ShortResult{};
            m_edmMachiningShortResult = EDMGapServo::ShortResult{};
            m_edmProcessNeedFreshSample = true;
        }
        else if (duplicate && m_edmProcessNeedFreshSample)
            next.reason = ShadowReason::AwaitFreshSample;
        else
        {
            if (!duplicate)
            {
                EDMGapServo::Sample sample{};
                sample.valid = true; sample.voltage = next.voltageV;
                // Dwell is sample time, never a repeatedly advancing polling clock.
                sample.nowMs = gap.sampledAtMs; sample.sequence = gap.sampleSequence;
                m_edmIdleShortResult = m_edmIdleShortDetector.Step(m_edmProcessProfile.servo, sample, false);
                m_edmMachiningShortResult = m_edmMachiningShortDetector.Step(m_edmProcessProfile.servo, sample, true);
                m_edmProcessNeedFreshSample = false;
            }
            next.idleShort = m_edmIdleShortResult;
            next.machiningShort = m_edmMachiningShortResult;
            if (!next.idleShort.valid || !next.machiningShort.valid)
                next.reason = ShadowReason::SampleIdentity;
            else
            {
                next.previewAvailable = true;
                next.previewSpeedMmPerMin = next.curve.limitedSpeedMmPerMin;
                next.machiningScenarioSpeedMmPerMin = next.previewSpeedMmPerMin;
                if (next.machiningShort.shortActive)
                {
                    next.shortRetreatPreview = EDMGapServo::EvaluateShortRetreat(m_edmProcessProfile.servo, next.curve);
                    if (next.shortRetreatPreview.valid)
                        next.machiningScenarioSpeedMmPerMin = next.shortRetreatPreview.signedSpeedMmPerMin;
                    else
                    {
                        next.reason = ShadowReason::CurveInvalid;
                        next.previewAvailable = false;
                        next.previewSpeedMmPerMin = next.machiningScenarioSpeedMmPerMin = 0.0;
                    }
                }
                else if (next.machiningShort.feedInhibited && next.machiningScenarioSpeedMmPerMin > 0.0)
                    next.machiningScenarioSpeedMmPerMin = 0.0;
            }
        }
    }
    // Do not lower the within-source high-water after a contradictory sample.
    const bool mayAdvanceFloor = sourceChanged ? (live && clockValid) :
        (gap.sampleSequence >= m_edmProcessLastGap.sampleSequence &&
            gap.sampledAtMs >= m_edmProcessLastGap.sampledAtMs);
    if (next.reason != ShadowReason::SampleIdentity && mayAdvanceFloor)
    {
        m_edmProcessLastGap = gap;
        m_edmProcessHaveGap = true;
    }
    m_edmProcessLastRecipeGeneration = summary.generation;
    EDM21LogShadowGateTransition(m_edmProcessShadow, next);
    m_edmProcessShadow = next;
    if (firstObservation) RtPrintf("[EDM20_FIX1][BOOT] phase=SHADOW_FIRST_EXIT reason=%u\n", static_cast<unsigned>(next.reason));
    if (!IsEDMZFixtureDiagnosticQuietSameThread() &&
        (!m_edmProcessLogSeen || (next.observedAtMs >= m_edmProcessLastLogMs &&
            next.observedAtMs - m_edmProcessLastLogMs >= 1000ULL)))
        EDM23LogAutomaticRecipe(m_recipe, next.machineProfileId, next.configRevision);
    LogEDMProcessShadowSameThread();
}

void NCManager::LogEDMProcessShadowSameThread() noexcept
{
    if (IsEDMZFixtureDiagnosticQuietSameThread()) return;
    const EDMProcessShadowSnapshot& s = m_edmProcessShadow;
    if (m_edmProcessLogSeen && (s.observedAtMs < m_edmProcessLastLogMs ||
        s.observedAtMs - m_edmProcessLastLogMs < 1000ULL)) return;
    // Three independently bounded records retain the same observation identity.
    // Nine significant digits keep even extreme finite doubles compact; CRT
    // also represents nonfinite diagnostic values without unbounded expansion.
    char line[512]{};
    int length = std::snprintf(line, sizeof(line),
        "[EDM20] event=SHADOW part=STATE logger=FIX2 mode=SHADOW_ONLY cadence=NC_10MS source=%.16s config=%u machine=%u COND=%u E=%u generation=%llu seq=%llu reason=%.32s gainAxisMask=%02X gainApplied=0 machiningActive=0 physicalPermit=0 discharge=0\n",
        EDMGap::SourceName(s.source), static_cast<unsigned>(s.configRevision),
        static_cast<unsigned>(s.machineProfileId), static_cast<unsigned>(s.tableId),
        static_cast<unsigned>(s.eCode), static_cast<unsigned long long>(s.recipeGeneration),
        static_cast<unsigned long long>(s.sampleSequence), EDM20ReasonName(s.reason),
        static_cast<unsigned>(s.activeAxisMask));
    EDM20PrintShadowLine(line, length);
    length = std::snprintf(line, sizeof(line),
        "[EDM20] event=SHADOW part=CURVE logger=FIX2 config=%u generation=%llu seq=%llu V=%.9g refV=%.9g E8=%.9g errorV=%.9g rawMmMin=%.9g limitedMmMin=%.9g\n",
        static_cast<unsigned>(s.configRevision), static_cast<unsigned long long>(s.recipeGeneration),
        static_cast<unsigned long long>(s.sampleSequence), s.voltageV, s.referenceV,
        s.overridePercent, s.curve.errorV, s.curve.rawSpeedMmPerMin, s.curve.limitedSpeedMmPerMin);
    EDM20PrintShadowLine(line, length);
    length = std::snprintf(line, sizeof(line),
        "[EDM20] event=SHADOW part=PREVIEW logger=FIX2 config=%u generation=%llu seq=%llu curvePreviewMmMin=%.9g machiningScenarioMmMin=%.9g idleShort=%.16s machiningShort=%.16s shortRetreatMmMin=%.9g segment=%u saturated=%u limitsConfirmed=%u\n",
        static_cast<unsigned>(s.configRevision), static_cast<unsigned long long>(s.recipeGeneration),
        static_cast<unsigned long long>(s.sampleSequence), s.previewSpeedMmPerMin,
        s.machiningScenarioSpeedMmPerMin, EDM20ShortName(s.idleShort.state), EDM20ShortName(s.machiningShort.state),
        s.shortRetreatPreview.signedSpeedMmPerMin, static_cast<unsigned>(s.curve.segment),
        s.curve.saturated ? 1U : 0U, s.curve.limitsConfirmed ? 1U : 0U);
    EDM20PrintShadowLine(line, length);
    m_edmProcessLogSeen = true;
    m_edmProcessLastLogMs = s.observedAtMs;
}

void NCManager::ReadEDMProcessTuningSameThread(EDMProcessTuningContext& context,
    EDMProcessTuningValues& values) const noexcept
{
    context = EDMProcessTuningContext{}; values = EDMProcessTuningValues{};
    if (!m_edmProcessReady) return;
    context.Generation = m_edmProcessTuningGeneration;
    context.ProfileRevision = m_edmProcessProfile.revision;
    context.MachineProfileId = m_edmProcessProfile.machineProfileId;
    context.SchemaVersion = m_edmProcessProfile.schemaVersion;
    context.ActiveAxisMask = m_edmProcessActiveAxisMask;
    context.Flags = EDM_PROCESS_TUNING_READY;
    if (!Close_System_Com_flag && m_edmProcessTuningGeneration != UINT64_MAX &&
        m_edmProcessProfile.revision != UINT32_MAX) context.Flags |= EDM_PROCESS_TUNING_CAN_APPLY;
    EDMProcessTuning::Encode(m_edmProcessProfile, values);
}

EDMProcessTuningStatus NCManager::ApplyEDMProcessTuningSameThread(
    const EDMProcessTuningRequest& request) noexcept
{
    if (!EDMProcessTuning::Canonical(request)) return EDMProcessTuningStatus::InvalidCommand;
    if (!m_edmProcessReady) return EDMProcessTuningStatus::Unavailable;
    if (Close_System_Com_flag || m_edmProcessTuningGeneration == UINT64_MAX ||
        m_edmProcessProfile.revision == UINT32_MAX) return EDMProcessTuningStatus::NotAllowed;
    if (request.ExpectedGeneration != m_edmProcessTuningGeneration) return EDMProcessTuningStatus::Stale;
    EDM20::ProcessProfile candidate{};
    if (!EDMProcessTuning::DecodeSelected(request, m_edmProcessProfile, candidate))
        return EDMProcessTuningStatus::DomainRejected;
    std::array<EDM20::GainResolution, 8> discharge{}, flush{};
    for (std::size_t axis = 0U; axis < 8U; ++axis) {
        if ((m_edmProcessActiveAxisMask & (1U << axis)) == 0U) continue;
        discharge[axis] = EDM20::ResolveAxisGains(candidate.axisGains, EDM20::GainBank::Discharge, axis, m_edmCncGainBaseline[axis]);
        flush[axis] = EDM20::ResolveAxisGains(candidate.axisGains, EDM20::GainBank::Flush, axis, m_edmCncGainBaseline[axis]);
        if (discharge[axis].status != EDM20::GainResolveStatus::Ready ||
            flush[axis].status != EDM20::GainResolveStatus::Ready) return EDMProcessTuningStatus::DomainRejected;
    }
    // Only the owning NC scan observes this all-or-nothing commit. No physical
    // gain, CNC baseline, file, recipe, voltage calibration or mode is changed.
    candidate.revision = m_edmProcessProfile.revision + 1U;
    candidate.servo.revision = candidate.revision;
    m_edmProcessProfile = candidate;
    m_edmDischargeGains = discharge; m_edmFlushGains = flush;
    ++m_edmProcessTuningGeneration;
    m_edmIdleShortDetector.Reset(); m_edmMachiningShortDetector.Reset();
    m_edmIdleShortResult = EDMGapServo::ShortResult{};
    m_edmMachiningShortResult = EDMGapServo::ShortResult{};
    m_edmProcessNeedFreshSample = true;
    m_edmProcessShadow = EDMProcessShadowSnapshot{};
    return EDMProcessTuningStatus::Success;
}
