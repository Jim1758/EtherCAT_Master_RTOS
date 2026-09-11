// CG: explicit, NC-thread-only GAP input simulation. No Motion producer or IO.
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <limits>
#include <windows.h>
#include <rtapi.h>

namespace
{
    constexpr std::uint32_t GapCaseCount = 16U;
    constexpr std::uint32_t GapAllCases = (1U << GapCaseCount) - 1U;
    constexpr std::uint64_t GapCaseMs = 750ULL;
    constexpr std::uint64_t GapMaxServiceGapMs = 250ULL;
    const char* const GapCaseNames[GapCaseCount] = {
        "NO_SAMPLE", "NORMAL", "ZERO_LOW", "LOW_HYSTERESIS",
        "LOW_RECOVER", "HIGH", "HIGH_HYSTERESIS", "HIGH_RECOVER",
        "INVALID", "VALID_RECOVER", "FROZEN", "FRESH_RECOVER",
        "REPLAY", "SEQUENCE_RECOVER", "OUT_OF_RANGE", "FINAL_RECOVER"
    };
    EDMGap::Quality GapExpectedQuality(std::uint32_t phase) noexcept
    {
        if (phase == 0U) return EDMGap::Quality::NO_SAMPLE;
        if (phase == 8U) return EDMGap::Quality::INVALID;
        if (phase == 10U) return EDMGap::Quality::STALE;
        if (phase == 12U) return EDMGap::Quality::SEQUENCE_ERROR;
        if (phase == 14U) return EDMGap::Quality::OUT_OF_RANGE;
        return EDMGap::Quality::VALID;
    }
    EDMGap::Band GapExpectedBand(std::uint32_t phase) noexcept
    {
        if (GapExpectedQuality(phase) != EDMGap::Quality::VALID)
            return EDMGap::Band::UNKNOWN;
        if (phase == 2U || phase == 3U) return EDMGap::Band::LOW;
        if (phase == 5U || phase == 6U) return EDMGap::Band::HIGH;
        return EDMGap::Band::NORMAL;
    }
}

bool NCManager::IsGapDryRunBlockShapeValid(const NCBlock& block) noexcept
{
    if (block.isEmpty || block.isGoto || !block.hasG || block.gCode != 180 ||
        block.gCount != 1 || block.gCodes[0] != 180 || block.mCount != 0 ||
        !block.has('P') || block.val('P') != 1.0) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        if ((i != ('N' - 'A') && i != ('P' - 'A')) ||
            !std::isfinite(block.param[i])) return false;
    }
    return true;
}

void NCManager::RejectGapDryRunSameThread(int alarmCode, int line, const char* reason)
{
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 3U;
    RtPrintf("[GAP-CG] event=SUMMARY result=FAIL test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%04X reason=%s alarm=%d\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), line,
        static_cast<unsigned int>(m_gapDryRun.phase),
        static_cast<unsigned int>(m_gapDryRun.passedMask), reason, alarmCode);
    m_gapInput.Reset();
    AlarmManager::GetInstance().Trigger(alarmCode, line);
    ChangeState(NCState::ALARM);
}

bool NCManager::ReadGapDryRunClockSameThread(std::uint64_t& nowMs) noexcept
{
    LARGE_INTEGER counter{};
    if (m_gapDryRun.frequency == 0ULL || !RtQueryPerformanceCounter(&counter) ||
        counter.QuadPart < 0) return false;
    const std::uint64_t ticks = static_cast<std::uint64_t>(counter.QuadPart);
    if (ticks < m_gapDryRun.lastTicks) return false;
    const std::uint64_t seconds = ticks / m_gapDryRun.frequency;
    const std::uint64_t fraction = (ticks % m_gapDryRun.frequency) * 1000ULL /
        m_gapDryRun.frequency;
    if (seconds > ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000ULL)
        return false;
    nowMs = seconds * 1000ULL + fraction;
    m_gapDryRun.lastTicks = ticks;
    return true;
}

bool NCManager::RestartGapDryRunSameThread()
{
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) >
        (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_FREQUENCY");
        return false;
    }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    m_gapDryRun.lastTicks = 0ULL;
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_READ");
        return false;
    }
    // These thresholds belong only to this synthetic test profile.
    if (!m_gapInput.Configure(EDMGap::Config{}, EDMGap::Source::SIMULATED))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "SIM_CONFIG");
        return false;
    }
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
    m_gapDryRun.lease = m_programMotionLease;
    m_gapDryRun.phase = 0U;
    m_gapDryRun.passedMask = 0U;
    m_gapDryRun.observations = 0U;
    m_gapDryRun.phaseSamples = 0U;
    m_gapDryRun.sequence = 0ULL;
    m_gapDryRun.sample = EDMGap::Sample{};
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.lastServiceMs = nowMs;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 0U;
    RtPrintf("[GAP-CG] event=BEGIN mode=SIMULATED test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=%llu dwellMs=30 staleMs=100 lowEnterMv=30000 lowExitMv=35000 highExitMv=75000 highEnterMv=80000 motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(GapCaseCount), static_cast<unsigned long long>(GapCaseMs));
    return true;
}

WaitConditionFunc NCManager::StartGapDryRunSameThread(const NCBlock& block)
{
    const int sourceLine = m_gapDryRun.sourceLine;
    if (!IsGapDryRunBlockShapeValid(block))
    {
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter,
            sourceLine, "BLOCK_SHAPE");
        return nullptr;
    }
    if (m_gapDryRun.active || m_mode != NCOperationMode::MEMORY || m_state != NCState::RUN ||
        AlarmManager::GetInstance().HasAlarm() || Close_System_Com_flag ||
        m_edmState == EDMState::NOT_READY || Homing.IsActive() ||
        m_isG66Active || !m_macroStack.empty() || m_pathFeed.pending || m_pathArc.pending ||
        m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreLiveBookkeeping.currentRunToken == 0ULL ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY,
            sourceLine, "NOT_READY");
        return nullptr;
    }
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.active = true;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.sourceLine = sourceLine;
    if (!RestartGapDryRunSameThread()) return nullptr;
    return WaitForGapDryRunCallback;
}

void NCManager::PauseGapDryRunSameThread(const char* reason) noexcept
{
    if (!m_gapDryRun.active || m_gapDryRun.paused) return;
    m_gapDryRun.paused = true;
    m_gapInput.Reset();
    RtPrintf("[GAP-CG] event=PAUSED test=%llu run=%llu dispatch=%llu case=%u reason=%s restart=REQUIRED\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned int>(m_gapDryRun.phase), reason);
}

void NCManager::CancelGapDryRunSameThread(const char* reason) noexcept
{
    if (!m_gapDryRun.active) return;
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 2U;
    m_gapInput.Reset();
    RtPrintf("[GAP-CG] event=SUMMARY result=CANCELLED test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%04X reason=%s\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.phase),
        static_cast<unsigned int>(m_gapDryRun.passedMask), reason);
}

void NCManager::ValidateGapDryRunSameThread()
{
    if (!m_gapDryRun.active) return;
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM)
        CancelGapDryRunSameThread("ALARM");
    else if (Close_System_Com_flag)
        CancelGapDryRunSameThread("SHUTDOWN");
    else if (m_mode != NCOperationMode::MEMORY ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) ||
        m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        m_isG66Active || !m_macroStack.empty())
        CancelGapDryRunSameThread("SCOPE_CHANGED");
    else if (m_state == NCState::HOLD || m_edmState == EDMState::NOT_READY)
        PauseGapDryRunSameThread("HOLD_OR_INTERLOCK");
    else if (!m_gapDryRun.paused &&
        (m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
            !m_gapDryRun.lease.Matches(m_programMotionLease) ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)))
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "IDENTITY_CHANGED");
}

bool NCManager::WaitForGapDryRunCallback(NCManager* nc)
{
    return nc != nullptr && nc->ProcessGapDryRunSameThread();
}

bool NCManager::ProcessGapDryRunSameThread()
{
    ValidateGapDryRunSameThread();
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && m_state == NCState::RUN &&
            m_mode == NCOperationMode::MEMORY && !AlarmManager::GetInstance().HasAlarm() &&
            m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
            m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
            m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
            m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
            m_gapDryRun.lease.Matches(m_programMotionLease) &&
            m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)) return true;
        if (m_state == NCState::RUN && !AlarmManager::GetInstance().HasAlarm())
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "CANCELLED_CALLBACK");
        return false;
    }
    if (m_state != NCState::RUN || m_edmState == EDMState::NOT_READY) return false;
    if (m_gapDryRun.dispatch != m_waitingBlockDispatchId ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CALLBACK_SCOPE");
        return false;
    }
    if (m_gapDryRun.paused)
    {
        ++m_gapDryRun.restarts;
        if (!RestartGapDryRunSameThread()) return false;
    }
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs) || nowMs < m_gapDryRun.lastServiceMs ||
        nowMs - m_gapDryRun.lastServiceMs > GapMaxServiceGapMs)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_OR_SERVICE_GAP");
        return false;
    }
    m_gapDryRun.lastServiceMs = nowMs;
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase >= GapCaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CASE_INDEX");
        return false;
    }
    if (phase == 0U) m_gapInput.Poll(nowMs);
    else if (phase == 10U) m_gapInput.Publish(m_gapDryRun.sample, nowMs);
    else if (phase == 12U)
    {
        EDMGap::Sample replay = m_gapDryRun.sample;
        ++replay.voltageMv;
        m_gapInput.Publish(replay, nowMs);
    }
    else
    {
        EDMGap::Sample sample{};
        sample.source = EDMGap::Source::SIMULATED;
        sample.sequence = ++m_gapDryRun.sequence;
        sample.sampledAtMs = nowMs;
        sample.valid = phase != 8U;
        sample.voltageMv = 50000;
        if (phase == 2U) sample.voltageMv = 0;
        else if (phase == 3U) sample.voltageMv = nowMs - m_gapDryRun.phaseStartMs >= 375ULL ?
            31000 : ((sample.sequence & 1ULL) ? 29000 : 31000);
        else if (phase == 4U) sample.voltageMv = 36000;
        else if (phase == 5U) sample.voltageMv = 90000;
        else if (phase == 6U) sample.voltageMv = nowMs - m_gapDryRun.phaseStartMs >= 375ULL ?
            79000 : ((sample.sequence & 1ULL) ? 79000 : 81000);
        else if (phase == 7U) sample.voltageMv = 70000;
        else if (phase == 14U) sample.voltageMv = 200001;
        m_gapDryRun.sample = sample;
        m_gapInput.Publish(sample, nowMs);
        ++m_gapDryRun.phaseSamples;
    }
    ++m_gapDryRun.observations;
    if (m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_NOT_ADVANCING");
        return false;
    }
    const EDMGap::Snapshot& observed = m_gapInput.Current();
    if (observed.quality != EDMGap::Quality::VALID && observed.band != EDMGap::Band::UNKNOWN)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "INVALID_BAND");
        return false;
    }
    if ((phase == 3U || phase == 6U) &&
        (observed.quality != EDMGap::Quality::VALID || observed.band != GapExpectedBand(phase)))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "HYSTERESIS_CHANGED");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < GapCaseMs) return false;
    const bool matching = observed.quality == GapExpectedQuality(phase) &&
        observed.band == GapExpectedBand(phase) && m_gapDryRun.observations >= 4U &&
        (GapExpectedQuality(phase) != EDMGap::Quality::VALID || m_gapDryRun.phaseSamples >= 4U);
    RtPrintf("[GAP-CG] event=CASE test=%llu restart=%u case=%u name=%s result=%s source=%s quality=%s band=%s mv=%d seq=%llu ageMs=%llu observations=%u samples=%u\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        GapCaseNames[phase], matching ? "PASS" : "FAIL", EDMGap::SourceName(observed.source),
        EDMGap::QualityName(observed.quality), EDMGap::BandName(observed.band),
        static_cast<int>(observed.voltageMv), static_cast<unsigned long long>(observed.sequence),
        static_cast<unsigned long long>(nowMs >= observed.sampledAtMs ? nowMs - observed.sampledAtMs : 0ULL),
        static_cast<unsigned int>(m_gapDryRun.observations), static_cast<unsigned int>(m_gapDryRun.phaseSamples));
    if (!matching)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, GapCaseNames[phase]);
        return false;
    }
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    m_gapDryRun.phaseSamples = 0U;
    if (m_gapDryRun.phase != GapCaseCount) return false;
    if (m_gapDryRun.passedMask != GapAllCases)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "COVERAGE");
        return false;
    }
    m_gapDryRun.active = false;
    m_gapDryRun.result = 1U;
    RtPrintf("[GAP-CG] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=16 passedMask=FFFF mode=SIMULATED motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts));
    m_gapInput.Reset(); // A completed simulation cannot become a live voltage source.
    return true;
}
