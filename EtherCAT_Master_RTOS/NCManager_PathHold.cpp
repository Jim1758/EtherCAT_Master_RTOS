// CB/CC/CD. Bounded Feed Hold excursions under the original unfinished identity.
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define NC_PATH_HOLD_NOINLINE __declspec(noinline)
#else
#define NC_PATH_HOLD_NOINLINE __attribute__((noinline))
#endif

namespace
{
    bool HoldTranslationCurrent(const CoordinateManager& coord, std::uint64_t generation) noexcept
    {
        return generation != 0ULL && coord.IsTranslationRunFrozen() &&
            coord.IsTranslationRunCurrent() && coord.GetTranslationSnapshot().generation == generation;
    }
    bool HoldIdentityEqual(const MotionExecutionIdentity& a, const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool HoldTranslationIdentityEqual(const MotionPathCoreHoldExcursionSnapshot& snapshot,
        const MotionExecutionIdentity& identity, const CoordinateManager& coord,
        std::uint64_t generation) noexcept
    {
        return HoldTranslationCurrent(coord, generation) && snapshot.translationGeneration == generation &&
            HoldIdentityEqual(snapshot.identity, identity);
    }
    bool HoldGeometryEqual(const NCPathCoreRetainedGeometry& a, const NCPathCoreRetainedGeometry& b) noexcept
    {
        // Compare actual members, never object padding introduced by a compiler.
        return std::memcmp(a.startMCS.data(), b.startMCS.data(), sizeof(a.startMCS)) == 0 &&
            std::memcmp(a.endMCS.data(), b.endMCS.data(), sizeof(a.endMCS)) == 0 &&
            std::memcmp(a.startPulse.data(), b.startPulse.data(), sizeof(a.startPulse)) == 0 &&
            std::memcmp(a.endPulse.data(), b.endPulse.data(), sizeof(a.endPulse)) == 0 &&
            std::memcmp(a.centerMCS.data(), b.centerMCS.data(), sizeof(a.centerMCS)) == 0 &&
            std::memcmp(a.centerPulse.data(), b.centerPulse.data(), sizeof(a.centerPulse)) == 0 &&
            std::memcmp(a.boundsMinMCS.data(), b.boundsMinMCS.data(), sizeof(a.boundsMinMCS)) == 0 &&
            std::memcmp(a.boundsMaxMCS.data(), b.boundsMaxMCS.data(), sizeof(a.boundsMaxMCS)) == 0 &&
            a.radiusMM == b.radiusMM && a.radiusPulse == b.radiusPulse &&
            a.startAngle == b.startAngle && a.sweepRadians == b.sweepRadians &&
            a.lengthMM == b.lengthMM && a.lengthPulse == b.lengthPulse &&
            a.axisMask == b.axisMask && a.direction == b.direction && a.kind == b.kind &&
            a.fullCircle == b.fullCircle && a.point == b.point && a.valid == b.valid;
    }
    // CT: 0 is before the station, 1 contains it, 2 is ambiguous or invalid.
    int HoldStationSourceRelation(double prefixMM, double lengthMM, double lengthPulse, double targetMM) noexcept
    {
        if (!std::isfinite(prefixMM) || prefixMM < 0.0 || !std::isfinite(lengthMM) || lengthMM <= 0.0 ||
            !std::isfinite(lengthPulse) || lengthPulse <= 0.0 || !std::isfinite(targetMM) || targetMM <= prefixMM) return 2;
        const double projectedMM = prefixMM + lengthMM;
        const double localMM = targetMM - prefixMM;
        const double localPulse = localMM * (lengthPulse / lengthMM);
        if (!std::isfinite(projectedMM) || projectedMM <= prefixMM ||
            !std::isfinite(localMM) || !std::isfinite(localPulse) || localPulse <= 0.0) return 2;
        if (projectedMM < targetMM && lengthMM < localMM && lengthPulse < localPulse) return 0;
        if (projectedMM > targetMM && lengthMM > localMM && lengthPulse > localPulse) return 1;
        return 2;
    }
    std::uint64_t HoldDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsPathCoreHoldBlockShapeValid(const NCBlock& block) noexcept
{
    if (block.isEmpty || block.isGoto || !block.hasG ||
        (block.gCode != 178 && block.gCode != 179) || block.gCount != 1 ||
        block.gCodes[0] != block.gCode || block.mCount != 0) return false;
    if (block.gCode == 178 && (!block.has('D') || !block.has('F') ||
        !std::isfinite(block.val('D')) || block.val('D') <= 0.0 ||
        !std::isfinite(block.val('F')) || block.val('F') <= 0.0 || block.val('F') > 100.0)) return false;
    if (block.gCode == 178 && block.has('L') &&
        (!std::isfinite(block.val('L')) || block.val('L') < 1.0 || block.val('L') > 32.0 ||
            std::floor(block.val('L')) != block.val('L'))) return false;
    if (block.gCode == 178 && block.has('P') &&
        block.val('P') != 1.0 && block.val('P') != 2.0 && block.val('P') != 3.0 && block.val('P') != 4.0 && block.val('P') != 5.0 && block.val('P') != 6.0 && block.val('P') != 7.0 && block.val('P') != 8.0 && block.val('P') != 9.0 && block.val('P') != 10.0 && block.val('P') != 11.0 && block.val('P') != 12.0 && block.val('P') != 13.0 && block.val('P') != 14.0 && block.val('P') != 15.0 && (block.val('P') != 16.0 && (block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0))) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 2.0 || block.val('P') == 3.0 || block.val('P') == 5.0 || block.val('P') == 7.0) &&
        (!block.has('Q') || (block.has('L') && block.val('L') != 1.0))) return false;
    if (block.gCode == 178 && block.has('P') && block.val('P') == 12.0 &&
        (block.has('L') && block.val('L') != 1.0)) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0))))) &&
        ((block.has('L') && block.val('L') > 8.0) ||
            !std::isfinite(block.val('Q') * (block.has('L') ? block.val('L') : 1.0)))) return false;
    if (block.gCode == 178 && block.has('P') && block.val('P') == 13.0 &&
        ((block.has('L') && block.val('L') > block.val('K')) ||
            !std::isfinite(block.val('Q') * (block.has('L') ? block.val('L') : 1.0)))) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 4.0 || block.val('P') == 6.0 || block.val('P') == 8.0 || block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))) && !block.has('Q')) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))) && !block.has('R')) return false;
    if (block.gCode == 178 && block.has('R') &&
        (!block.has('P') || (block.val('P') != 9.0 && block.val('P') != 10.0 && block.val('P') != 11.0 && block.val('P') != 12.0 && block.val('P') != 13.0 && block.val('P') != 14.0 && block.val('P') != 15.0 && (block.val('P') != 16.0 && (block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0))) || !std::isfinite(block.val('R')) ||
            block.val('R') < 1.0 || block.val('R') > 8.0 || std::floor(block.val('R')) != block.val('R'))) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))) && !block.has('K')) return false;
    if (block.has('K') && (block.gCode != 178 || !block.has('P') || (block.val('P') != 10.0 && block.val('P') != 11.0 && block.val('P') != 12.0 && block.val('P') != 13.0 && block.val('P') != 14.0 && block.val('P') != 15.0 && (block.val('P') != 16.0 && (block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0))) ||
        !std::isfinite(block.val('K')) || block.val('K') < 2.0 || block.val('K') > 8.0 ||
        std::floor(block.val('K')) != block.val('K'))) return false;
    if (block.gCode == 178 && block.has('Q') &&
        (!block.has('P') || !std::isfinite(block.val('Q')) || block.val('Q') <= 0.0)) return false;
    if (block.has('V') && (block.gCode != 178 || !block.has('P') || (block.val('P') != 16.0 && (block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0)) ||
        (block.val('V') != 50.0 && block.val('V') != 20.0))) return false;
    if (block.has('H') && (block.gCode != 178 || !block.has('P') || (block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0) ||
        (block.val('P') == 21.0 ? block.val('H') != 40.0 : (block.val('H') != 0.0 && block.val('H') != 150.0)) ||
        (block.val('H') == 150.0 && block.has('V') && block.val('V') != 50.0))) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0) &&
        block.has('V') && block.val('V') != 50.0) return false;
    if (block.has('U') && (block.gCode != 178 || !block.has('P') || (block.val('P') != 18.0 && block.val('P') != 21.0) ||
        (block.val('U') != 50.0 && block.val('U') != 20.0))) return false;
    if (block.gCode == 178 && block.has('P') && block.val('P') == 18.0 &&
        (!block.has('U') || (block.has('V') && block.val('V') != 50.0) ||
            (block.has('H') && block.val('H') != 0.0))) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && !(block.gCode == 178 && (address == 'D' || address == 'F' || address == 'L' || address == 'P' || address == 'Q' || address == 'R' || address == 'K' || address == 'V' || address == 'H' || address == 'U'))) return false;
        if (!std::isfinite(block.val(address))) return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsPathCoreHoldInputOmission(const NCBlock& block) const noexcept
{
    if (!m_pathHold.explicitControl || block.hasG || block.gCount != 0) return false;
    for (int i = 0; i < 26; ++i)
        if (block.hasParam[i] && i != ('N' - 'A')) return true;
    return false;
}

NC_PATH_HOLD_NOINLINE
void NCManager::InvalidatePathCoreHoldSameThread(bool cancelMotion) noexcept
{
    CancelPathCoreHoldAutomaticSameThread("INVALIDATED");
    // Constructor paths may revoke NC state before Motion is initialized.
    // Only a successfully bound live RT request requires a Motion cancellation.
    // ChangeState invalidates before publishing ALARM. HasAlarm also covers
    // that path; alarm=0/origin=2 means the external alarm number is unknown.
    // Preserve an exact CB rejection captured before its own Trigger call.
    if ((m_pathHold.armed || m_pathHold.bound) && !m_pathHoldLastFault.present &&
        (m_state.load(std::memory_order_acquire) == NCState::ALARM ||
            AlarmManager::GetInstance().HasAlarm()))
        CapturePathCoreHoldFaultSameThread(m_pathHold.code, 0, 2U);
    if (m_pathHold.armed || m_pathHold.bound) LogPathCoreHoldSameThread("INVALIDATED");
    // CL_FIX1: RESET revokes NC admission now, but its exact safety batch
    // owns RT retirement. Keep immutable excursion geometry alive until then.
    if (cancelMotion && m_pathHold.bound) m_motion.CancelPathCoreHoldExcursion();
    m_pathHold.armed = false;
    m_pathHold.bound = false;
    m_pathHold.requested = false;
    m_pathHold.startCommitted = false;
    m_pathHold.requestedHoldSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    m_pathHold.blocked = false;
    m_pathHold.explicitControl = false;
    m_pathHold.candidateDispatch = 0ULL;
}

NC_PATH_HOLD_NOINLINE
void NCManager::RejectPathCoreHoldSameThread(std::uint32_t code, int alarmCode,
    const MotionPathCoreHoldExcursionSnapshot* observed)
{
    const NCPathCoreRetainedFault historyFault = m_pathReplayStore.Fault();
    if (alarmCode == AlarmManager::G_Code_Invalid_parameter)
    {
        if (code == 3U) alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
        else if (code == 17U)
            alarmCode = historyFault == NCPathCoreRetainedFault::CAPACITY ?
            AlarmManager::PATH_REPLAY_HISTORY_CAPACITY : AlarmManager::PATH_REPLAY_HISTORY_UNAVAILABLE;
        else if (code == 15U && observed != nullptr &&
            (observed->reason == 4U || (observed->crossSegment && observed->reason == 9U)))
            alarmCode = AlarmManager::PATH_RETREAT_UNAVAILABLE;
    }
    m_pathHold.code = code;
    CapturePathCoreHoldFaultSameThread(code, alarmCode, 1U, observed);
    RtPrintf("[PCORE-ALARM] alarm=%d unit=HOLD run=%llu dispatch=%llu code=%u pc=%d line=%d producer=NA geometry=NA history=%u historyFault=%u cursor=%u rtValid=%u rtReason=%u\n",
        alarmCode, static_cast<unsigned long long>(m_pathHoldLastFault.run), static_cast<unsigned long long>(m_pathHoldLastFault.dispatch),
        static_cast<unsigned int>(code), m_pathHoldLastFault.sourcePC, m_pathHoldLastFault.sourceLine,
        static_cast<unsigned int>(m_pathReplayStore.Count()), static_cast<unsigned int>(historyFault),
        static_cast<unsigned int>(m_pathReplayStore.State()), m_pathHoldLastFault.rtValid ? 1U : 0U,
        static_cast<unsigned int>(m_pathHoldLastFault.snapshot.reason));
    LogPathCoreHoldSameThread("REJECTED");
    AlarmManager::GetInstance().Trigger(alarmCode, m_pathHoldLastFault.sourceLine);
    ChangeState(NCState::ALARM);
}

NC_PATH_HOLD_NOINLINE
WaitConditionFunc NCManager::StartPathCoreHoldSameThread(const NCBlock& block)
{
    if (!IsPathCoreHoldBlockShapeValid(block))
    {
        RejectPathCoreHoldSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        m_pathFeed.pending || m_pathArc.pending || m_pathReplay.pending || m_pathHold.bound ||
        m_gapDryRun.active || !IsPathCoreReplayConfigurationValid() || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        AlarmManager::GetInstance().HasAlarm() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreLiveBookkeeping.currentRunToken == 0ULL || GetBaseProgramCache().GetGeneration() == 0ULL)
    {
        RejectPathCoreHoldSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    InvalidatePathCoreHoldSameThread();
    m_pathHold = PathHoldState{};
    m_pathHoldLastFault = PathHoldLastFault{}; // Only a newly admitted G178/G179 clears it.
    m_gapServiceCurrent = GapServiceDiagnostic{};
    m_gapServiceLastFault = GapServiceDiagnostic{};
    m_gapServiceAgeOnlyCalls = 0U;
    m_pathHold.explicitControl = true;
    m_pathHoldView.completedCount = 0U;
    m_pathHoldView.translationGeneration = 0ULL;
    if (block.gCode == 179)
    {
        LogPathCoreHoldSameThread("DISARMED");
        return nullptr;
    }
    m_pathHold.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathHold.cache = GetBaseProgramCache().GetGeneration();
    m_pathHold.lease = m_programMotionLease;
    m_pathHold.distanceMM = block.val('D');
    m_pathHold.feedMMMin = block.val('F');
    m_pathHold.cycleLimit = block.has('P') && (block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))) ? 1U :
        (block.has('L') ? static_cast<std::uint32_t>(block.val('L')) : 1U);
    m_pathHold.crossSegment = block.has('P');
    m_pathHold.requireReturnAuthorization = block.has('P') && (block.val('P') == 5.0 || block.val('P') == 6.0 || block.val('P') == 7.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))));
    m_pathHold.automaticEnabled = block.has('Q');
    m_pathHold.automaticIntervalMM = block.has('Q') ? block.val('Q') : 0.0;
    if (m_pathHold.crossSegment && !PreparePathCoreHoldHistorySameThread())
    {
        RejectPathCoreHoldSameThread(17U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (block.has('P') && (block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))))
    {
        const std::uint32_t sources = static_cast<std::uint32_t>(block.val('K'));
        if (m_pathReplayStore.Count() > NCPathCoreRetainedPath::Capacity - sources ||
            m_isSingleBlockEnabled || m_legacySingleBlockPausePending || !m_feedHoldResumeGate.IsEnabled())
        {
            RejectGapPathSimulationSameThread("WINDOW_CAPACITY_OR_CONTROL");
            return nullptr;
        }
        m_gapWindow = GapPathSourceWindowState{};
        m_gapWindow.run = m_pathHold.run;
        m_gapWindow.cache = m_pathHold.cache;
        m_gapWindow.lease = m_pathHold.lease;
        m_gapWindow.previousDispatch = m_currentExecutingBlockDispatchId;
        m_gapWindow.distanceMM = m_pathHold.distanceMM;
        m_gapWindow.feedMMMin = m_pathHold.feedMMMin;
        m_gapWindow.intervalMM = m_pathHold.automaticIntervalMM;
        m_gapWindow.initialHistoryCount = m_pathReplayStore.Count();
        m_gapWindow.sourceIndex = 1U;
        m_gapWindow.sourceLimit = static_cast<std::uint16_t>(sources);
        m_gapWindow.cycleLimit = m_pathHold.cycleLimit;
        m_gapWindow.probeLimit = static_cast<std::uint32_t>(block.val('R'));
        m_gapWindow.active = true;
        m_gapWindow.allowNormalSources = block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))));
        m_gapWindow.cumulativeStation = block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))));
        m_gapWindow.repeatedCumulativeStation = block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))));
        m_gapWindow.multipleStationsPerSource = (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))));
        m_gapWindow.allowSeamStations = (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0))));
        m_gapWindow.tailSupervision = (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)));
        m_gapWindow.tailVoltage = block.has('V') ? static_cast<std::uint8_t>(block.val('V')) : 50U;
        m_gapWindow.continuousSignal = (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0));
        m_gapWindow.sourceGapMs = block.has('H') ? static_cast<std::uint8_t>(block.val('H')) : (block.val('P') == 21.0 ? 40U : 0U);
        m_gapWindow.sampledInput = (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0);
        m_gapInlet.active = m_gapWindow.sampledInput;
        m_gapWindow.queuedInput = block.val('P') == 20.0 || block.val('P') == 21.0;
        m_gapQueue.active = m_gapWindow.queuedInput;
        m_gapWindow.recoverQueuedInput = block.val('P') == 21.0;
        m_gapWindow.queuedProbeVoltage = m_gapWindow.recoverQueuedInput && block.has('U') ?
            static_cast<std::uint8_t>(block.val('U')) : 50U;
        m_gapRecovery.active = m_gapWindow.recoverQueuedInput;
        m_gapWindow.pendingLowAcrossSources = block.val('P') == 18.0;
        m_gapWindow.pendingLowVoltage = m_gapWindow.pendingLowAcrossSources && block.has('U') ? static_cast<std::uint8_t>(block.val('U')) : 50U;
        if (m_gapWindow.pendingLowAcrossSources)
        {
            m_gapPending.active = true;
            m_gapPending.run = m_gapWindow.run;
            m_gapPending.cache = m_gapWindow.cache;
        }
        if (m_gapWindow.continuousSignal)
        {
            m_gapSignal.active = true;
            m_gapSignal.run = m_gapWindow.run;
            m_gapSignal.cache = m_gapWindow.cache;
            m_gapSignal.sourceIndex = 1U;
        }
        if (m_gapWindow.repeatedCumulativeStation)
            m_gapWindow.stationLimit = block.has('L') ? static_cast<std::uint32_t>(block.val('L')) : 1U;
        LogGapPathSourceWindowSameThread("WINDOW_ARMED");
    }
    m_pathHold.armed = true;
    m_pathHold.code = 1U;
    if (block.has('P') && (block.val('P') == 2.0 || block.val('P') == 3.0 || block.val('P') == 4.0 || block.val('P') == 5.0 || block.val('P') == 6.0 || block.val('P') == 7.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0))))))) &&
        !StartGapPathSimulationSameThread(block.val('P') != 2.0,
            block.val('P') == 4.0 || block.val('P') == 5.0 || block.val('P') == 6.0 || block.val('P') == 7.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))),
            block.val('P') == 5.0 || block.val('P') == 6.0 || block.val('P') == 7.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))),
            block.val('P') == 6.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))),
            block.val('P') == 7.0 || block.val('P') == 8.0 || (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))),
            (block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0 || block.val('P') == 13.0 || (block.val('P') == 14.0 || (block.val('P') == 15.0 || (block.val('P') == 16.0 || (block.val('P') == 17.0 || block.val('P') == 18.0 || (block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0)))))), block.has('R') ? static_cast<std::uint8_t>(block.val('R')) : 1U))
        return nullptr;
    LogPathCoreHoldSameThread("ARMED");
    if (m_pathHold.automaticEnabled) LogPathCoreHoldAutomaticSameThread("ARMED");
    return nullptr;
}

// CD admission copies only completed, same-run canonical history. Earlier rows
// may have older execution epochs: they grant geometry, never execution authority.
NC_PATH_HOLD_NOINLINE
bool NCManager::PreparePathCoreHoldHistorySameThread(NCBlockDispatchId dispatchCeiling) noexcept
{
    if (dispatchCeiling == NC_BLOCK_DISPATCH_ID_INVALID) dispatchCeiling = m_currentExecutingBlockDispatchId;
    const std::uint32_t count = m_pathReplayStore.Count();
    if (!m_pathReplay.armed || m_pathReplay.pending || count == 0U ||
        count > NCPathCoreRetainedPath::Capacity ||
        m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE ||
        m_pathReplayStore.State() != NCPathCoreRetainedCursorState::IDLE || m_pathReplayStore.Pending() ||
        m_pathReplay.run != m_pathHold.run || m_pathReplay.cache != m_pathHold.cache ||
        !m_pathReplayLease.Matches(m_pathHold.lease) || !IsPathCoreReplayConfigurationValid() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease)) return false;
    const NCBlockLifecycleCounters counters = m_blockLifecycleLedger.GetCounters();
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || counters.activeBlockOverwrite != 0ULL ||
        counters.activeSegmentIndexOverwrite != 0ULL || counters.orphanFeedback != 0ULL ||
        counters.duplicateTerminalFeedback != 0ULL || counters.terminalFeedbackConflict != 0ULL) return false;
    const std::uint64_t translationGeneration = CoordSys.GetTranslationSnapshot().generation;
    if (!HoldTranslationCurrent(CoordSys, translationGeneration)) return false;
    m_pathHoldView.translationGeneration = translationGeneration;
    m_pathHoldView.validAxisMask = m_pathReplayStore.ValidAxisMask();
    m_pathHoldView.pulsePerMM = m_pathReplayPulsePerMM;
    if (m_pathHoldView.validAxisMask == 0U || (m_pathHoldView.validAxisMask & ~255U) != 0U) return false;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if ((m_pathHoldView.validAxisMask & (1U << axis)) == 0U) continue;
        const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
        const double currentScale = context.resolution_PPR / context.finalLead;
        if (!std::isfinite(currentScale) || currentScale <= 0.0 ||
            currentScale != m_pathHoldView.pulsePerMM[axis]) return false;
    }
    for (std::uint32_t i = 0U; i < count; ++i)
    {
        const NCPathCoreRetainedGeometry* row = m_pathReplayStore.Get(i);
        const PathReplaySource& source = m_pathReplaySource[i];
        if (row == nullptr || !IsNCPathCoreRetainedGeometryValid(*row) ||
            source.translationGeneration != m_pathHoldView.translationGeneration ||
            !source.identity.IsAssigned() || source.identity.source != MotionCommandSource::NC_MEMORY ||
            source.sourcePC < 0 || source.sourceLine <= 0 ||
            source.identity.sourceBlockId != static_cast<MotionSourceBlockId>(source.sourcePC) ||
            source.dispatch == 0ULL || source.commit == 0ULL ||
            source.dispatch >= dispatchCeiling ||
            (i != 0U && (source.dispatch <= m_pathReplaySource[i - 1U].dispatch ||
                source.commit <= m_pathReplaySource[i - 1U].commit)) ||
            (i != 0U && !AreNCPathCoreRetainedEndpointsConnected(m_pathHoldView.completed[i - 1U],
                *row, m_pathHoldView.validAxisMask, m_pathHoldView.pulsePerMM))) return false;
        m_pathHoldView.completed[i] = *row;
    }
    m_pathHoldView.original.Clear();
    m_pathHoldView.completedCount = count; // Publish NC readiness only after the bounded copy succeeds.
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::BuildPathCoreHoldViewSameThread(bool line) noexcept
{
    const std::uint32_t count = m_pathHoldView.completedCount;
    if (count == 0U || count > NCPathCoreRetainedPath::Capacity || !m_pathReplay.armed ||
        m_pathReplay.pending || m_pathReplayStore.Pending() ||
        m_pathReplayStore.State() != NCPathCoreRetainedCursorState::IDLE ||
        m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE ||
        m_pathReplayStore.Count() != count || m_pathReplayStore.ValidAxisMask() != m_pathHoldView.validAxisMask ||
        m_pathReplay.run != m_pathHold.run || m_pathReplay.cache != m_pathHold.cache ||
        !m_pathReplayLease.Matches(m_pathHold.lease) || !IsPathCoreReplayConfigurationValid()) return false;
    const NCBlockLifecycleCounters counters = m_blockLifecycleLedger.GetCounters();
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || counters.activeBlockOverwrite != 0ULL ||
        counters.activeSegmentIndexOverwrite != 0ULL || counters.orphanFeedback != 0ULL ||
        counters.duplicateTerminalFeedback != 0ULL || counters.terminalFeedbackConflict != 0ULL) return false;
    const std::uint32_t mask = line ? m_pathFeedMotion.receipt.validAxisMask : m_pathArcMotion.receipt.validAxisMask;
    const std::uint64_t translationGeneration = line ?
        m_pathFeedMotion.receipt.translationGeneration : m_pathArcMotion.receipt.translationGeneration;
    if (mask != m_pathHoldView.validAxisMask ||
        !HoldTranslationCurrent(CoordSys, translationGeneration) ||
        translationGeneration != m_pathHoldView.translationGeneration) return false;
    for (std::uint32_t i = 0U; i < count; ++i)
    {
        const NCPathCoreRetainedGeometry* row = m_pathReplayStore.Get(i);
        const PathReplaySource& source = m_pathReplaySource[i];
        if (row == nullptr || !HoldGeometryEqual(*row, m_pathHoldView.completed[i]) ||
            source.translationGeneration != m_pathHoldView.translationGeneration ||
            !source.identity.IsAssigned() || source.identity.source != MotionCommandSource::NC_MEMORY ||
            source.sourcePC < 0 || source.sourceLine <= 0 ||
            source.identity.sourceBlockId != static_cast<MotionSourceBlockId>(source.sourcePC) ||
            source.dispatch == 0ULL || source.dispatch >= m_pathHold.dispatch ||
            source.commit == 0ULL || source.commit >= m_pathHold.commit ||
            HoldIdentityEqual(source.identity, m_pathHold.identity) ||
            (i != 0U && (source.dispatch <= m_pathReplaySource[i - 1U].dispatch ||
                source.commit <= m_pathReplaySource[i - 1U].commit))) return false;
    }
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if ((mask & (1U << axis)) == 0U) continue;
        const AxisContext& context = m_motion.GetAxisContext(static_cast<int>(axis));
        if (m_pathHoldView.pulsePerMM[axis] != context.resolution_PPR / context.finalLead) return false;
    }
    const bool built = line ? BuildNCPathCoreRetainedLine(m_pathFeedMotion.receipt.line, m_pathHoldView.original) :
        BuildNCPathCoreRetainedArc(m_pathArcMotion.receipt.arc, m_pathHoldView.original);
    return built && !m_pathHoldView.original.point &&
        AreNCPathCoreRetainedEndpointsConnected(m_pathHoldView.completed[count - 1U],
            m_pathHoldView.original, mask, m_pathHoldView.pulsePerMM);
}

NC_PATH_HOLD_NOINLINE
void NCManager::BeginPathCoreHoldCaptureSameThread(const NCBlock& block,
    NCBlockDispatchId dispatchId) noexcept
{
    if (m_gapWindow.active && !block.isEmpty &&
        !NCGCodeSemantics::Contains(block, 178) && !NCGCodeSemantics::Contains(block, 179) &&
        (!block.hasG || block.gCount != 1 || block.mCount != 0 || block.isGoto ||
            (!NCGCodeSemantics::Contains(block, 1) && !NCGCodeSemantics::Contains(block, 2) &&
                !NCGCodeSemantics::Contains(block, 3))))
        CancelPathCoreHoldAutomaticSameThread("WINDOW_INTERVENING_BLOCK");
    if (block.isEmpty || IsPathCoreHoldInputOmission(block)) return;
    if (m_pathHold.bound) return; // Original callback prevents another real dispatch.
    if (NCGCodeSemantics::Contains(block, 178) || NCGCodeSemantics::Contains(block, 179)) return;
    m_pathHold.explicitControl = false;
    if (!m_pathHold.armed) return;
    const bool source = NCGCodeSemantics::Contains(block, 1) ||
        NCGCodeSemantics::Contains(block, 2) || NCGCodeSemantics::Contains(block, 3);
    if (!source || dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        m_pathHold.code = 4U;
        LogPathCoreHoldSameThread("UNUSED_INTERVENING_BLOCK");
        InvalidatePathCoreHoldSameThread();
        return;
    }
    if (m_gapWindow.active && (!IsGapPathSourceWindowScopeValidSameThread() ||
        m_gapWindow.budgetProven || dispatchId <= m_gapWindow.previousDispatch ||
        m_pathReplayStore.Count() != m_gapWindow.initialHistoryCount + m_gapWindow.sourceIndex - 1U ||
        !PreparePathCoreHoldHistorySameThread(dispatchId)))
    {
        RejectGapPathSimulationSameThread("WINDOW_NEXT_HISTORY_OR_SCOPE");
        return;
    }
    m_pathHold.candidateDispatch = dispatchId;
}

NC_PATH_HOLD_NOINLINE
void NCManager::CommitPathCoreHoldCaptureSameThread(NCBlockDispatchId dispatchId)
{
    if (!m_pathHold.armed || m_pathHold.bound || m_pathHold.candidateDispatch != dispatchId) return;
    const bool sourceEpochPending = m_gapWindow.active && m_motion.HasPendingSafetyOrRecoveryRequests();
    ObservePathCoreHoldSameThread(true);
    if (!m_pathHold.armed) return;
    double lengthMM = 0.0, lengthPulse = 0.0, sourceFeed = 0.0, sourceVelocity = 0.0;
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == dispatchId;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == dispatchId;
    if (line == arc)
    {
        RejectPathCoreHoldSameThread(5U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (line)
    {
        const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
        m_pathHold.identity = r.identity;
        m_pathHold.commit = m_pathFeed.commit;
        m_pathHold.sourcePC = m_pathFeed.sourcePC;
        m_pathHold.sourceLine = m_pathFeed.sourceLine;
        lengthMM = r.line.lengthMM;
        lengthPulse = r.line.lengthPulse;
        sourceFeed = r.line.feedMMMin;
        sourceVelocity = r.line.velocityPPS;
    }
    else
    {
        const MotionFeedArcReceipt& r = m_pathArcMotion.receipt;
        m_pathHold.identity = r.identity;
        m_pathHold.commit = m_pathArc.commit;
        m_pathHold.sourcePC = m_pathArc.sourcePC;
        m_pathHold.sourceLine = m_pathArc.sourceLine;
        lengthMM = r.arc.lengthMM;
        lengthPulse = r.arc.lengthPulse;
        sourceFeed = r.arc.feedMMMin;
        sourceVelocity = r.arc.velocityPPS;
    }
    const std::uint64_t translationGeneration = line ?
        m_pathFeedMotion.receipt.translationGeneration : m_pathArcMotion.receipt.translationGeneration;
    if (!HoldTranslationCurrent(CoordSys, translationGeneration) ||
        (m_pathHold.crossSegment && m_pathHoldView.translationGeneration != translationGeneration))
    {
        RejectPathCoreHoldSameThread(7U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathHoldView.translationGeneration = translationGeneration;
    m_pathHold.dispatch = dispatchId;
    if (m_gapWindow.active && (!IsGapPathSourceWindowScopeValidSameThread(true) ||
        m_gapWindow.budgetProven || !m_gapPath.active ||
        m_gapPath.returnProbeLimit != m_gapWindow.probeLimit ||
        dispatchId <= m_gapWindow.previousDispatch || m_pathHold.commit <= m_gapWindow.previousCommit ||
        (m_gapWindow.sourceIndex > 1U &&
            (!m_gapWindow.previousIdentity.IsAssigned() ||
                m_pathHold.identity.epoch <= m_gapWindow.previousIdentity.epoch ||
                m_pathHold.identity.segmentId == m_gapWindow.previousIdentity.segmentId ||
                m_gapWindow.previousIdentity.sourceBlockId < 0 ||
                m_pathHold.identity.sourceBlockId <= m_gapWindow.previousIdentity.sourceBlockId))))
    {
        RejectGapPathSimulationSameThread("WINDOW_NEW_SOURCE_IDENTITY");
        return;
    }
    if (lengthMM == 0.0 || lengthPulse == 0.0)
    {
        if (m_gapPath.active)
        {
            RejectGapPathSimulationSameThread("POINT_SOURCE_UNAVAILABLE");
            return;
        }
        m_pathHold.code = 6U;
        LogPathCoreHoldSameThread("UNUSED_POINT");
        InvalidatePathCoreHoldSameThread();
        return;
    }
    if (m_gapWindow.active && m_gapWindow.multipleStationsPerSource)
    {
        std::uint32_t planned = 0U;
        if (!CountGapPathSourceStationsSameThread(m_gapWindow.completedForwardMM, lengthMM, lengthPulse,
            m_gapWindow.completedStations, planned))
        {
            RejectGapPathSimulationSameThread("WINDOW_SOURCE_STATIONS_INVALID");
            return;
        }
        m_gapWindow.normalSource = planned == 0U;
        m_gapWindow.cycleLimit = planned == 0U ? 1U : planned;
        m_pathHold.cycleLimit = m_gapWindow.cycleLimit;
    }
    if (m_gapPending.active && m_gapPending.injected && !m_gapPending.resolved &&
        m_gapWindow.sourceIndex == m_gapPending.targetSourceIndex && !m_gapWindow.normalSource)
    {
        RejectGapPathSimulationSameThread("CY_TARGET_NOT_NORMAL");
        return;
    }
    if (!m_pathHold.identity.IsAssigned() || m_pathHold.identity.source != MotionCommandSource::NC_MEMORY ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() || m_pathHold.commit == 0ULL ||
        m_pathHold.sourcePC < 0 || m_pathHold.sourceLine <= 0 ||
        m_pathHold.identity.sourceBlockId != static_cast<MotionSourceBlockId>(m_pathHold.sourcePC) ||
        !std::isfinite(sourceFeed) || sourceFeed <= 0.0 || m_pathHold.feedMMMin > sourceFeed ||
        !std::isfinite(sourceVelocity) || sourceVelocity <= 0.0 ||
        (m_pathHold.crossSegment && !BuildPathCoreHoldViewSameThread(line)) ||
        !m_motion.BindPathCoreHoldExcursion(m_pathHold.identity, m_pathHold.lease,
            lengthMM, lengthPulse, m_pathHold.distanceMM, m_pathHold.feedMMMin, sourceFeed, sourceVelocity,
            m_pathHold.cycleLimit, m_pathHold.crossSegment ? &m_pathHoldView : nullptr,
            m_pathHold.requireReturnAuthorization))
    {
        RejectPathCoreHoldSameThread(7U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathHold.armed = false;
    m_pathHold.bound = true;
    RtPrintf("[CNC-TRANSLATION-PATH] phase=BOUND kind=HOLD translationGen=%llu wcs=%d dispatch=%llu epoch=%llu seg=%llu sourcePC=%d\n",
        static_cast<unsigned long long>(m_pathHoldView.translationGeneration), CoordSys.GetTranslationSnapshot().wcsCode,
        static_cast<unsigned long long>(dispatchId), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_pathHold.identity.segmentId), m_pathHold.sourcePC);
    m_pathHold.code = 1U;
    if (m_pathHold.automaticEnabled)
    {
        // CS keeps Q in the window's original millimetres. Only exact retained
        // source completion advances completedForwardMM; retreat/return never do.
        const double stationMM = m_gapWindow.active && m_gapWindow.cumulativeStation &&
            !m_gapWindow.stationConsumed ?
            GetGapPathWindowStationMMSameThread() - m_gapWindow.completedForwardMM : m_pathHold.automaticIntervalMM;
        m_pathHold.automaticIntervalPulse = stationMM * (lengthPulse / lengthMM);
        m_pathHold.automaticNextS = m_pathHold.automaticIntervalPulse;
        const bool seamStart = m_gapWindow.allowSeamStations && m_gapWindow.sourceIndex > 1U &&
            !m_gapWindow.normalSource && !m_gapWindow.stationConsumed && stationMM == 0.0 &&
            m_gapWindow.completedForwardMM > 0.0 &&
            ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM, lengthMM, lengthPulse,
                GetGapPathWindowStationMMSameThread()) == 1;
        if (!std::isfinite(stationMM) || (stationMM <= 0.0 && !seamStart) ||
            !std::isfinite(lengthMM) || lengthMM <= 0.0 || !std::isfinite(lengthPulse) || lengthPulse <= 0.0 ||
            !std::isfinite(m_pathHold.automaticIntervalPulse) ||
            (m_pathHold.automaticIntervalPulse <= 0.0 && !(seamStart && m_pathHold.automaticIntervalPulse == 0.0)))
        {
            RejectPathCoreHoldSameThread(18U, AlarmManager::PATH_GEOMETRY_INVALID);
            return;
        }
        if (m_gapWindow.active && m_gapWindow.cumulativeStation)
        {
            const double projectedMM = m_gapWindow.completedForwardMM + lengthMM;
            if (!std::isfinite(projectedMM) || projectedMM <= m_gapWindow.completedForwardMM)
            {
                RejectGapPathSimulationSameThread("WINDOW_CUMULATIVE_DISTANCE_INVALID");
                return;
            }
            // Floating-point subtraction and addition need not round identically.
            // Reject equality in both canonical cumulative and local-station forms.
            if (!m_gapWindow.allowSeamStations && !m_gapWindow.stationConsumed &&
                projectedMM == GetGapPathWindowStationMMSameThread())
            {
                RejectGapPathSimulationSameThread("STATION_OUTSIDE_SOURCE");
                return;
            }
            if (m_gapWindow.repeatedCumulativeStation && !m_gapWindow.stationConsumed &&
                ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM, lengthMM, lengthPulse,
                    GetGapPathWindowStationMMSameThread()) == 2)
            {
                RejectGapPathSimulationSameThread("STATION_ENDPOINT_AMBIGUOUS");
                return;
            }
            // CT admits at most one required nQ station in each original source.
            if (m_gapWindow.repeatedCumulativeStation && !m_gapWindow.multipleStationsPerSource &&
                !m_gapWindow.stationConsumed && !IsGapPathNextStationBeyondSourceSameThread(m_gapWindow.completedForwardMM,
                    lengthMM, lengthPulse, m_gapWindow.completedStations))
            {
                RejectGapPathSimulationSameThread("MULTIPLE_STATIONS_IN_SOURCE");
                return;
            }
        }
        // CR retains its per-source rule. CS classifies against remaining Q,
        // then keeps every later source NORMAL after its one proven station.
        if (m_gapWindow.active && m_gapWindow.allowNormalSources && !m_gapWindow.allowSeamStations)
            m_gapWindow.normalSource = (m_gapWindow.cumulativeStation && m_gapWindow.stationConsumed) ||
            lengthMM < stationMM;
        if (m_gapPath.active && !m_gapWindow.normalSource &&
            (m_pathHold.automaticIntervalPulse >= lengthPulse ||
                (m_gapWindow.allowNormalSources && lengthMM == stationMM)))
        {
            RejectGapPathSimulationSameThread("STATION_OUTSIDE_SOURCE");
            return;
        }
        LogPathCoreHoldAutomaticSameThread("BOUND");
    }
    LogPathCoreHoldSameThread("BOUND");
    if (m_gapWindow.active)
    {
        if (m_gapSignal.active && (m_gapWindow.sourceGapMs == 150U || m_gapWindow.recoverQueuedInput) &&
            m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex && !m_gapWindow.normalSource)
        {
            RejectGapPathSimulationSameThread("HOOK_TARGET_NOT_NORMAL");
            return;
        }
        LogGapPathSourceWindowSameThread("SOURCE_BOUND");
        if (sourceEpochPending) LogGapPathSourceWindowSameThread("SOURCE_EPOCH_PENDING_BOUND");
    }
}

NC_PATH_HOLD_NOINLINE
void NCManager::ObservePathCoreHoldSameThread(bool bindingCurrentSource)
{
    const NCState state = m_state.load(std::memory_order_acquire);
    if (m_pathHoldLastFault.present)
    {
        // Called by the NC task, never the 250 us Motion consumer. At a
        // 10 ms NC cycle this retains evidence in rolling logs every 2 s.
        if (state == NCState::ALARM)
        {
            if (++m_pathHoldLastFault.repeatCalls >= 200U)
            {
                m_pathHoldLastFault.repeatCalls = 0U;
                LogPathCoreHoldLastFaultSameThread();
                LogGapServiceFaultSameThread();
            }
        }
        else m_pathHoldLastFault.repeatCalls = 0U;
    }
    if (m_gapTail.active)
    {
        const auto terminal = m_motion.GetPathCoreHoldExcursionSnapshot();
        const bool completed = (m_pathFeed.dispatch == m_gapTail.dispatch && m_pathFeed.completed) ||
            (m_pathArc.dispatch == m_gapTail.dispatch && m_pathArc.completed);
        if (completed) (void)CompleteGapPathTailProofSameThread(terminal);
        else (void)ServiceGapPathTailSameThread(terminal);
        return;
    }
    if (m_gapWindow.active && !IsGapPathSourceWindowScopeValidSameThread(bindingCurrentSource))
    {
        CancelPathCoreHoldAutomaticSameThread("WINDOW_SCOPE_REVOKED");
    }
    if (!m_pathHold.armed && !m_pathHold.bound) return;
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() || Homing.IsActive() ||
        m_isG66Active || !m_macroStack.empty() || m_pathHold.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathHold.cache != GetBaseProgramCache().GetGeneration() ||
        !m_pathHold.lease.Matches(m_programMotionLease) || !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        !IsPathCoreReplayConfigurationValid())
    {
        m_pathHold.code = 8U;
        InvalidatePathCoreHoldSameThread();
        return;
    }
    if (!m_pathHold.bound) return;
    if (!HoldTranslationCurrent(CoordSys, m_pathHoldView.translationGeneration))
    {
        RejectPathCoreHoldSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    const bool completed = (m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.completed) ||
        (m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.completed);
    if (completed)
    {
        if (m_gapWindow.active && m_gapWindow.normalSource)
        {
            (void)CompleteGapPathNormalSourceProofSameThread(m_motion.GetPathCoreHoldExcursionSnapshot());
            return; // Only the real callback may Retain and consume this source.
        }
        // Observe runs before the automatic service. A final return and source
        // completion can arrive together, so consume only its exact proof here.
        if (m_gapPath.active && m_gapPath.repeating)
        {
            const auto terminal = m_motion.GetPathCoreHoldExcursionSnapshot();
            if (terminal.publicationSequence == 0ULL)
            {
                // A bounded publication collision grants no completion and
                // cannot refresh GAP age. Keep this source pending for retry.
                (void)ServiceGapPathSimulationSameThread(0.0, false, "FINAL_NO_RT");
                return;
            }
            if (!m_pathHold.requested || !m_pathHold.startCommitted || !m_gapPath.held ||
                m_pathHold.automaticHoldOwned ||
                !HoldTranslationIdentityEqual(terminal, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
                m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
                !terminal.ownerLease.Matches(m_pathHold.lease) || !terminal.crossSegment ||
                terminal.historyCount != m_pathHoldView.completedCount ||
                terminal.cycleLimit != m_pathHold.cycleLimit ||
                terminal.phase != MotionPathCoreHoldExcursionPhase::COMPLETE ||
                m_pathHold.requestedHoldSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
                (terminal.holdRequestSequence != m_pathHold.requestedHoldSequence &&
                    !(m_gapPath.returnLowTest && m_gapPath.repeatedLowRetreat &&
                        terminal.holdRequestSequence == m_gapPath.returnReholdSequence)) ||
                // Completion is a global J5 fence, not the accepted request ID.
                terminal.completedHoldRequestSequence < m_pathHold.requestedHoldSequence ||
                terminal.activeOrdinal != terminal.historyCount ||
                terminal.retreatCount != terminal.returnCount ||
                terminal.returnCount != m_pathHold.automaticObservedReturns + 1ULL ||
                terminal.returnCount != m_pathHold.cycleLimit ||
                !std::isfinite(terminal.lengthPulse) || terminal.lengthPulse <= 0.0 ||
                !std::isfinite(terminal.returnedS) || terminal.returnedS < 0.0 ||
                terminal.returnedS > terminal.lengthPulse || terminal.returnedS != terminal.heldS)
            {
                RejectGapPathSimulationSameThread("SOURCE_COMPLETED_WITHOUT_FINAL_RETURN");
                return;
            }
            if (!ValidateGapPathReturnLowSnapshotSameThread(terminal)) return;
            if (!ServiceGapPathSimulationSameThread(0.0, true, "FINAL_RETURN", false, nullptr, &terminal)) return;
            if (!IsGapPathAutomaticNormalSameThread() ||
                (!m_gapSignal.active && !IsGapPathCurrentSampleProvenSameThread()))
            {
                RejectGapPathSimulationSameThread("FINAL_RETURN_NOT_NORMAL");
                return;
            }
            m_pathHold.automaticObservedReturns = terminal.returnCount;
            if (m_gapWindow.active && m_gapWindow.multipleStationsPerSource)
            {
                m_pathHold.automaticNextS = 0.0;
                LogGapPathSourceWindowSameThread("SOURCE_RETURN_PROVEN");
            }
            LogPathCoreHoldAutomaticSameThread("RETURNED");
            if (m_gapWindow.active)
            {
                if (!CompleteGapPathSourceBudgetSameThread(terminal)) return;
            }
            else CancelPathCoreHoldAutomaticSameThread("BUDGET_DONE");
        }
        if (m_gapWindow.active)
        {
            if (!m_gapWindow.budgetProven)
                RejectGapPathSimulationSameThread("WINDOW_SOURCE_WITHOUT_BUDGET");
            return; // The real Feed/Arc callback alone can retain and advance.
        }
        if (m_gapPath.active && !m_gapPath.held)
        {
            RejectGapPathSimulationSameThread("SOURCE_COMPLETED_BEFORE_LOW");
            return;
        }
        if (m_gapPath.active && m_gapPath.automaticResume && m_pathHold.automaticHoldOwned)
        {
            RejectGapPathSimulationSameThread("SOURCE_COMPLETED_BEFORE_RESUME");
            return;
        }
        LogPathCoreHoldSameThread(m_pathHold.requested ? "SOURCE_COMPLETED" : "UNUSED_COMPLETED");
        InvalidatePathCoreHoldSameThread();
        return;
    }
    if (m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch())
    {
        m_pathHold.code = 9U;
        InvalidatePathCoreHoldSameThread();
        return;
    }
    const MotionPathCoreHoldExcursionSnapshot snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (snapshot.publicationSequence != 0ULL && HoldIdentityEqual(snapshot.identity, m_pathHold.identity) &&
        snapshot.translationGeneration != m_pathHoldView.translationGeneration)
    {
        RejectPathCoreHoldSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY, &snapshot);
        return;
    }
    if (HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) &&
        (snapshot.transitionSequence != m_pathHold.observedTransition ||
            (snapshot.crossSegment && snapshot.seamCount != m_pathHold.observedSeamCount)))
    {
        m_pathHold.observedTransition = snapshot.transitionSequence;
        m_pathHold.observedSeamCount = snapshot.seamCount;
        LogPathCoreHoldSameThread("RT_PHASE");
    }
    if (HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) &&
        snapshot.ownerLease.Matches(m_pathHold.lease) &&
        snapshot.phase == MotionPathCoreHoldExcursionPhase::REJECTED)
    {
        // RT has fenced this original source. Publish an explicit NC alarm
        // instead of leaving its pending callback in a silent endless RUN.
        RejectPathCoreHoldSameThread(15U, (snapshot.reason == 4U || (snapshot.crossSegment && snapshot.reason == 9U)) ?
            static_cast<int>(AlarmManager::G_Code_Invalid_parameter) :
            static_cast<int>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY), &snapshot);
    }
}

// CH/CI/CK use the CG input monitor; the original path owns all motion authority.
NC_PATH_HOLD_NOINLINE
bool NCManager::StartGapPathSimulationSameThread(bool automaticResume, bool repeating, bool lowRetreat,
    bool repeatedLowRetreat, bool returnLowTest, bool repeatedReturnLow, std::uint8_t returnProbeLimit) noexcept
{
    m_gapPath = GapPathSimulationState{};
    m_gapAdmission = GapPathAdmissionState{};
    m_gapServiceCurrent = GapServiceDiagnostic{};
    m_gapServiceAgeOnlyCalls = 0U;
    m_gapPath.active = true;
    m_gapPath.automaticResume = automaticResume;
    m_gapPath.repeating = repeating;
    m_gapPath.lowRetreat = lowRetreat;
    m_gapPath.repeatedLowRetreat = repeatedLowRetreat;
    m_gapPath.returnLowTest = returnLowTest;
    m_gapPath.repeatedReturnLow = repeatedReturnLow;
    m_gapPath.returnProbeLimit = returnProbeLimit;
    LARGE_INTEGER frequency{};
    if (m_gapDryRun.active || !RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) >
        (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
    {
        RejectGapPathSimulationSameThread("CLOCK_FREQUENCY_OR_SESSION");
        return false;
    }
    m_gapPath.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    if (!m_gapInput.Configure(EDMGap::Config{}, EDMGap::Source::SIMULATED))
    {
        RejectGapPathSimulationSameThread("INPUT_CONFIGURATION");
        return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::RejectGapPathSimulationSameThread(const char* reason,
    const MotionPathCoreHoldExcursionSnapshot* observed) noexcept
{
    m_gapServiceLastFault = m_gapServiceCurrent;
    m_gapServiceLastFault.present = true;
    m_gapServiceLastFault.quality = m_gapInput.Current().quality;
    m_gapServiceLastFault.run = m_pathHold.run;
    m_gapServiceLastFault.dispatch = m_pathHold.dispatch;
    RtPrintf("[%s] phase=FAILED reason=%s run=%llu dispatch=%llu line=%d alarm=%d\n",
        m_gapWindow.active ? (m_gapWindow.tailSupervision ? "GAP-CW" : m_gapWindow.multipleStationsPerSource ? "GAP-CU" : m_gapWindow.repeatedCumulativeStation ? "GAP-CT" : m_gapWindow.cumulativeStation ? "GAP-CS" : (m_gapWindow.allowNormalSources ? "GAP-CR" : "GAP-CQ")) : m_gapPath.repeatedReturnLow ? "GAP-CP" : m_gapPath.returnLowTest ? (m_gapPath.repeatedLowRetreat ? "GAP-CO" : "GAP-CN") : m_gapPath.lowRetreat ? (m_gapPath.repeatedLowRetreat ? "GAP-CM" : "GAP-CL") : (m_gapPath.repeating ? "GAP-CK" : (m_gapPath.automaticResume ? "GAP-CI" : "GAP-CH")), reason, static_cast<unsigned long long>(m_pathHold.run),
        static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
        static_cast<int>(AlarmManager::GAP_PATH_SIMULATION_FAILED));
    if (m_pathHold.bound && m_state == NCState::RUN &&
        m_pathHold.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease))
        FeedHoldInternal();
    RejectPathCoreHoldSameThread(19U, AlarmManager::GAP_PATH_SIMULATION_FAILED, observed);
    LogGapServiceFaultSameThread();
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapServiceFaultSameThread() const noexcept
{
    const GapServiceDiagnostic& d = m_gapServiceLastFault;
    if (!d.present) return;
    const bool serviceGapValid = d.nowValid && d.lastServiceValid && d.nowMs >= d.lastServiceMs;
    const bool ageValid = d.nowValid && d.sampleSequence != 0ULL && d.nowMs >= d.sampledAtMs;
    RtPrintf("[GAP-SERVICE-FAULT] site=%s nowValid=%u nowMs=%llu lastServiceMs=%llu serviceGapValid=%u serviceGapMs=%llu sampleMs=%llu ageValid=%u ageMs=%llu sampleSeq=%llu publish=%u ageOnlyCalls=%u quality=%s run=%llu dispatch=%llu\n",
        d.site, d.nowValid ? 1U : 0U, static_cast<unsigned long long>(d.nowMs),
        static_cast<unsigned long long>(d.lastServiceMs), serviceGapValid ? 1U : 0U,
        static_cast<unsigned long long>(serviceGapValid ? d.nowMs - d.lastServiceMs : 0ULL),
        static_cast<unsigned long long>(d.sampledAtMs), ageValid ? 1U : 0U,
        static_cast<unsigned long long>(ageValid ? d.nowMs - d.sampledAtMs : 0ULL),
        static_cast<unsigned long long>(d.sampleSequence), d.publishSample ? 1U : 0U,
        static_cast<unsigned int>(d.ageOnlyCalls), EDMGap::QualityName(d.quality),
        static_cast<unsigned long long>(d.run), static_cast<unsigned long long>(d.dispatch));
    if (d.admissionSeen)
        RtPrintf("[GAP-ADMISSION-FAULT] seen=%u closed=%u originMs=%llu totalMs=%llu heartbeatMs=%llu heartbeatAgeMs=%llu sampleOriginMs=%llu sampleClock=%u limitMs=1000 freshMs=100\n",
            d.admissionSeen ? 1U : 0U, d.admissionClosed ? 1U : 0U,
            static_cast<unsigned long long>(d.admissionOriginMs),
            static_cast<unsigned long long>(d.nowValid && d.nowMs >= d.admissionOriginMs ? d.nowMs - d.admissionOriginMs : 0ULL),
            static_cast<unsigned long long>(d.admissionHeartbeatMs),
            static_cast<unsigned long long>(d.nowValid && d.admissionSeen && d.nowMs >= d.admissionHeartbeatMs ? d.nowMs - d.admissionHeartbeatMs : 0ULL),
            static_cast<unsigned long long>(d.sourceSampleOriginMs), d.sourceSampleClockStarted ? 1U : 0U);
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathAdmissionSameThread(const char* phase, std::uint64_t nowMs) const noexcept
{
    RtPrintf("[GAP-ADMISSION] phase=%s dispatch=%llu epoch=%llu owner=%u gen=%u requestGen=%llu pendingPub=%llu loadedPub=%llu tick=%llu axis=%d error=%016llX window=%016llX elapsedMs=%llu closed=%u sample=%llu\n",
        phase, static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned int>(m_pathHold.lease.owner), static_cast<unsigned int>(m_pathHold.lease.generation),
        static_cast<unsigned long long>(m_gapAdmission.generation),
        static_cast<unsigned long long>((std::max)(m_gapAdmission.publication, m_gapAdmission.obsoletePublication)),
        static_cast<unsigned long long>(m_gapAdmission.loadedPublication),
        static_cast<unsigned long long>((std::max)(m_gapAdmission.tick, m_gapAdmission.obsoleteTick)),
        static_cast<int>(m_gapAdmission.blockedAxis), static_cast<unsigned long long>(HoldDoubleBits(m_gapAdmission.followingError)),
        static_cast<unsigned long long>(HoldDoubleBits(m_gapAdmission.windowPulse)),
        static_cast<unsigned long long>(nowMs >= m_gapPath.firstServiceMs ? nowMs - m_gapPath.firstServiceMs : 0ULL),
        m_gapAdmission.closed ? 1U : 0U, static_cast<unsigned long long>(m_gapPath.sequence));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::HasGapPathSourceConsumerAcceptedSameThread() const noexcept
{
    return (m_pathFeed.dispatch == m_pathHold.dispatch &&
        (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_pathHold.identity)) &&
        (m_pathFeed.consumerAccepted || m_pathFeed.consumerStarted || m_pathFeed.completed)) ||
        (m_pathArc.dispatch == m_pathHold.dispatch &&
            (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_pathHold.identity)) &&
            (m_pathArc.consumerAccepted || m_pathArc.consumerStarted || m_pathArc.completed));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ServiceGapPathAdmissionSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == m_pathHold.dispatch;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == m_pathHold.dispatch;
    const double length = line ? m_pathFeedMotion.receipt.line.lengthPulse : m_pathArcMotion.receipt.arc.lengthPulse;
    const std::uint32_t axisMask = line ? m_pathFeedMotion.receipt.line.axisMask : m_pathArcMotion.receipt.arc.axisMask;
    const bool receipt = line ?
        (m_pathFeedMotion.receipt.valid && m_pathFeedMotion.receipt.commandAccepted &&
            m_pathFeedMotion.receipt.tailCommitted && m_pathFeedMotion.receipt.captureBound &&
            m_pathFeed.commit == m_pathHold.commit &&
            (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_pathHold.identity)) &&
            m_pathFeedMotion.receipt.ownerLease.Matches(m_pathHold.lease)) :
        (m_pathArcMotion.receipt.valid && m_pathArcMotion.receipt.commandAccepted &&
            m_pathArcMotion.receipt.tailCommitted && m_pathArcMotion.receipt.captureBound &&
            m_pathArc.commit == m_pathHold.commit &&
            (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_pathHold.identity)) &&
            m_pathArcMotion.receipt.ownerLease.Matches(m_pathHold.lease));
    if (!snapshot.admissionPending || snapshot.publicationSequence == 0ULL ||
        snapshot.admissionWaitTick == 0ULL || snapshot.admissionWaitAxis < 0 || snapshot.admissionWaitAxis >= 8 ||
        (axisMask & (1U << static_cast<unsigned int>(snapshot.admissionWaitAxis))) == 0U ||
        !std::isfinite(snapshot.admissionFollowingError) || !std::isfinite(snapshot.admissionWindowPulse) ||
        snapshot.admissionWindowPulse <= 0.0 || snapshot.admissionFollowingError <= snapshot.admissionWindowPulse ||
        !std::isnan(snapshot.activeS) || snapshot.ready || snapshot.boundaryOnly ||
        snapshot.phase != MotionPathCoreHoldExcursionPhase::ARMED || snapshot.reason != 0U ||
        snapshot.holdRequestSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        snapshot.completedHoldRequestSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        snapshot.retreatCount != 0ULL || snapshot.returnCount != 0ULL || snapshot.seamCount != 0ULL ||
        snapshot.heldS != 0.0 || snapshot.retreatS != 0.0 || snapshot.returnedS != 0.0 || snapshot.retreatLocalS != 0.0 ||
        snapshot.requestGeneration == 0ULL || (m_gapAdmission.seen && snapshot.requestGeneration != m_gapAdmission.generation) ||
        !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) || !snapshot.ownerLease.Matches(m_pathHold.lease) ||
        snapshot.lengthPulse != length || !std::isfinite(length) || length <= 0.0 ||
        snapshot.distanceMM != m_pathHold.distanceMM || snapshot.feedMMMin != m_pathHold.feedMMMin ||
        snapshot.cycleLimit != m_pathHold.cycleLimit || snapshot.crossSegment != m_pathHold.crossSegment ||
        snapshot.requireReturnAuthorization != m_pathHold.requireReturnAuthorization ||
        snapshot.historyCount != m_pathHoldView.completedCount || snapshot.activeOrdinal != snapshot.historyCount ||
        snapshot.retreatOrdinal != snapshot.historyCount ||
        line == arc || !receipt || !m_gapPath.active || m_gapAdmission.progressSeen ||
        (m_gapSignal.active ?
            (m_gapSignal.sourceSampleSeen || m_gapPath.sequence != m_gapSignal.sourceStartSequence ||
                m_gapInput.Current().sequence != m_gapSignal.sourceStartSequence) :
            (m_gapPath.sequence != 0ULL || m_gapInput.Current().sequence != 0ULL)) || m_gapPath.lowInjected || m_gapPath.held || m_gapPath.returnHold ||
        m_gapPath.returnLowInjected || m_gapPath.returnRehold || m_gapPath.returnResumeApplied || m_gapPath.returnWatchStarted ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        !m_pathHold.bound || !m_pathHold.automaticEnabled || m_pathHold.blocked ||
        m_pathHold.requested || m_pathHold.startCommitted || m_pathHold.automaticHoldOwned || m_pathHold.automaticAdmissionOwned ||
        m_pathHold.returnHoldRequested || m_pathHold.returnHoldRetreatCount != 0ULL ||
        m_pathHold.automaticObservedReturns != 0ULL || m_pathHold.automaticBoundarySequence != 0ULL ||
        m_pathHold.requestedHoldSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_pathHold.automaticSettleSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE || m_feedHoldResumeGate.GetSnapshot().active ||
        Close_System_Com_flag || AlarmManager::GetInstance().HasAlarm() || Homing.IsActive() ||
        m_isG66Active || !m_macroStack.empty() || m_isSingleBlockEnabled || m_legacySingleBlockPausePending ||
        !m_feedHoldResumeGate.IsEnabled() || m_pathHold.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathHold.cache != GetBaseProgramCache().GetGeneration() || m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathHold.lease.Matches(m_programMotionLease) || !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !IsPathCoreReplayConfigurationValid() ||
        (m_gapWindow.active && !IsGapPathSourceWindowScopeValidSameThread()))
    {
        RejectGapPathSimulationSameThread("ADMISSION_PENDING_INVALID", &snapshot);
        return false;
    }
    return ServiceGapPathSimulationSameThread(0.0, false, "SOURCE_ADMISSION_WAIT", false, &snapshot);
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathLoadedPublicationSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (snapshot.admissionPending || snapshot.admissionWaitTick != 0ULL || snapshot.admissionWaitAxis != -1 ||
        snapshot.admissionFollowingError != 0.0 || snapshot.admissionWindowPulse != 0.0 ||
        (!m_gapAdmission.progressSeen && m_gapAdmission.seen &&
            (snapshot.requestGeneration != m_gapAdmission.generation ||
                snapshot.publicationSequence <= m_gapAdmission.publication ||
                snapshot.publicationSequence <= m_gapAdmission.obsoletePublication)))
    {
        RejectGapPathSimulationSameThread("ADMISSION_LOADED_PUBLICATION_INVALID", &snapshot);
        return false;
    }
    if (m_gapSignal.active && (snapshot.publicationSequence == 0ULL ||
        snapshot.publicationSequence < m_gapAdmission.loadedPublication))
    {
        RejectGapPathSimulationSameThread("CX_LOADED_PUBLICATION_REGRESSION", &snapshot);
        return false;
    }
    if (!m_gapAdmission.progressSeen || m_gapSignal.active)
        m_gapAdmission.loadedPublication = snapshot.publicationSequence;
    m_gapAdmission.progressSeen = true;
    return true;
}

// CZ: frame contents never change after acquisition. Validate the exact producer
// seal before another acquisition can overwrite the cell or a consumer can use it.
NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathSampleFrameSameThread() const noexcept
{
    if (!m_gapInlet.active || !m_gapWindow.sampledInput || !m_gapWindow.active ||
        !m_gapSignal.active || m_gapWindow.pendingLowAcrossSources ||
        m_gapInlet.acquisitionSequence < m_gapInlet.sourceFloor ||
        m_gapInlet.acquisitionSequence < m_gapPath.sequence) return false;
    if (!m_gapInlet.available) return !m_gapInlet.accepted;
    const auto& a = m_gapInlet.frame;
    const auto& b = m_gapInlet.sealedFrame;
    if (!m_gapInlet.clockStarted || a.run != b.run || a.cache != b.cache ||
        a.publication != b.publication || a.sourceIndex != b.sourceIndex ||
        !HoldIdentityEqual(a.identity, b.identity) || !a.lease.Matches(b.lease) ||
        a.sample.sequence != b.sample.sequence || a.sample.sampledAtMs != b.sample.sampledAtMs ||
        a.sample.voltageMv != b.sample.voltageMv || a.sample.source != b.sample.source ||
        a.sample.valid != b.sample.valid || a.sample.sequence != m_gapInlet.acquisitionSequence ||
        a.sample.sequence <= m_gapInlet.sourceFloor ||
        a.sample.sampledAtMs != m_gapInlet.lastAcquiredMs || a.publication == 0ULL ||
        a.sample.source != EDMGap::Source::SIMULATED || !a.sample.valid ||
        a.run != m_gapWindow.run || a.cache != m_gapWindow.cache ||
        a.sourceIndex != m_gapWindow.sourceIndex ||
        !HoldIdentityEqual(a.identity, m_pathHold.identity) || !a.lease.Matches(m_pathHold.lease)) return false;
    if (m_gapInlet.accepted)
    {
        const auto& accepted = m_gapInlet.acceptedFrame;
        const auto& gap = m_gapInput.Current();
        if (accepted.run != a.run || accepted.cache != a.cache || accepted.sourceIndex != a.sourceIndex ||
            !HoldIdentityEqual(accepted.identity, a.identity) || !accepted.lease.Matches(a.lease) ||
            accepted.sample.sequence <= m_gapInlet.sourceFloor ||
            accepted.sample.sequence > a.sample.sequence || accepted.publication == 0ULL ||
            m_gapInlet.acceptedPublication < accepted.publication ||
            accepted.sample.sequence != m_gapPath.sequence || accepted.sample.sequence != gap.sequence ||
            accepted.sample.sampledAtMs != gap.sampledAtMs || accepted.sample.voltageMv != gap.voltageMv ||
            accepted.sample.source != gap.source || !accepted.sample.valid) return false;
        if (accepted.sample.sequence == a.sample.sequence &&
            (accepted.sample.sampledAtMs != a.sample.sampledAtMs ||
                accepted.sample.voltageMv != a.sample.voltageMv || accepted.publication != a.publication)) return false;
    }
    return true;
}

// DA: inspect all three slots at most. No padding comparisons or silent loss.
NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathSampleQueueSameThread() const noexcept
{
    if (!m_gapQueue.active) return !m_gapWindow.queuedInput && !m_gapRecovery.active;
    if (!m_gapWindow.queuedInput || !m_gapInlet.active || !m_gapWindow.active ||
        m_gapQueue.head >= 3U || m_gapQueue.count > 3U ||
        m_gapInlet.freezeStarted || m_gapInlet.freezeConsumed ||
        (!m_gapRecovery.active && (m_gapQueue.consumerPaused != m_gapQueue.pauseConsumed)) ||
        !ValidateGapPathQueueRecoverySameThread() ||
        (m_gapQueue.terminalDrained && !m_gapQueue.terminalSealed)) return false;
    const auto equal = [](const GapPathSampleFrame& a, const GapPathSampleFrame& b) noexcept {
        return a.run == b.run && a.cache == b.cache && a.sourceIndex == b.sourceIndex &&
            a.publication == b.publication && HoldIdentityEqual(a.identity, b.identity) &&
            a.lease.Matches(b.lease) && a.sample.sequence == b.sample.sequence &&
            a.sample.sampledAtMs == b.sample.sampledAtMs && a.sample.voltageMv == b.sample.voltageMv &&
            a.sample.source == b.sample.source && a.sample.valid == b.sample.valid;
    };
    const std::uint64_t base = m_gapInlet.accepted ? m_gapPath.sequence : m_gapInlet.sourceFloor;
    if (base > m_gapInlet.acquisitionSequence ||
        m_gapInlet.acquisitionSequence - base != m_gapQueue.count ||
        (m_gapInlet.accepted && base <= m_gapInlet.sourceFloor)) return false;
    std::uint64_t previousMs = m_gapInlet.accepted ? m_gapInlet.acceptedFrame.sample.sampledAtMs : 0ULL;
    std::uint64_t previousPublication = m_gapInlet.accepted ? m_gapInlet.acceptedFrame.publication : 0ULL;
    for (std::uint32_t i = 0U; i < m_gapQueue.count; ++i)
    {
        const std::uint32_t slot = (m_gapQueue.head + i) % 3U;
        const auto& frame = m_gapQueue.frames[slot];
        if (!equal(frame, m_gapQueue.seals[slot]) || frame.sample.sequence != base + i + 1ULL ||
            frame.sample.source != EDMGap::Source::SIMULATED || !frame.sample.valid ||
            frame.run != m_gapWindow.run || frame.cache != m_gapWindow.cache ||
            frame.sourceIndex != m_gapWindow.sourceIndex ||
            !HoldIdentityEqual(frame.identity, m_pathHold.identity) || !frame.lease.Matches(m_pathHold.lease) ||
            frame.publication == 0ULL || frame.publication < previousPublication ||
            ((i != 0U || m_gapInlet.accepted) &&
                (frame.sample.sampledAtMs < previousMs || frame.sample.sampledAtMs - previousMs < 20ULL)))
            return false;
        previousMs = frame.sample.sampledAtMs;
        previousPublication = frame.publication;
        if (i + 1U == m_gapQueue.count && !equal(frame, m_gapInlet.frame)) return false;
    }
    if (m_gapQueue.consumerPaused && (!m_gapInlet.accepted || !m_gapSignal.sourceSampleSeen ||
        !m_gapWindow.normalSource || (m_gapWindow.sourceGapMs != 150U && !m_gapRecovery.active) ||
        m_gapSignal.sourceIndex != m_gapSignal.dropTargetSourceIndex ||
        !m_gapSignal.dropStarted || !m_gapSignal.dropConsumed ||
        m_gapQueue.pauseStartMs != m_gapSignal.dropStartMs ||
        m_gapQueue.pauseStartMs < m_gapInlet.acceptedFrame.sample.sampledAtMs ||
        m_gapQueue.pauseStartMs - m_gapInlet.acceptedFrame.sample.sampledAtMs < 20ULL)) return false;
    if (m_gapQueue.terminalSealed)
    {
        if (!m_gapInlet.accepted || m_gapQueue.consumerPaused ||
            (m_gapRecovery.active && m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex && !m_gapRecovery.fenceAccepted) ||
            !equal(m_gapQueue.terminalFrame, m_gapQueue.terminalSeal) ||
            !equal(m_gapQueue.terminalFrame, m_gapInlet.frame) ||
            m_gapQueue.terminalPublication != m_gapQueue.terminalPublicationSeal ||
            m_gapQueue.terminalPublication < m_gapQueue.terminalFrame.publication ||
            m_gapQueue.terminalPublication > m_gapSignal.publication ||
            (m_gapQueue.terminalDrained && (m_gapQueue.count != 0U ||
                m_gapPath.sequence != m_gapQueue.terminalFrame.sample.sequence))) return false;
    }
    return true;
}

// DB keeps a second fence: it proves recovery of unread acquisitions, not source completion.
NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathQueueRecoverySameThread(bool previousSource) const noexcept
{
    const auto& r = m_gapRecovery;
    if (!r.active) return !m_gapWindow.recoverQueuedInput;
    if (!m_gapWindow.recoverQueuedInput || !m_gapQueue.active || !m_gapInlet.active ||
        !m_gapSignal.active || m_gapWindow.sourceGapMs != 40U ||
        (m_gapWindow.queuedProbeVoltage != 50U && m_gapWindow.queuedProbeVoltage != 20U)) return false;
    const auto equal = [](const GapPathSampleFrame& a, const GapPathSampleFrame& b) noexcept {
        return a.run == b.run && a.cache == b.cache && a.sourceIndex == b.sourceIndex &&
            a.publication == b.publication && a.identity.epoch == b.identity.epoch &&
            a.identity.segmentId == b.identity.segmentId && a.identity.sourceBlockId == b.identity.sourceBlockId &&
            a.identity.source == b.identity.source &&
            a.lease.Matches(b.lease) && a.sample.sequence == b.sample.sequence &&
            a.sample.sampledAtMs == b.sample.sampledAtMs && a.sample.voltageMv == b.sample.voltageMv &&
            a.sample.source == b.sample.source && a.sample.valid == b.sample.valid;
    };
    if (!m_gapQueue.pauseConsumed)
    {
        return !m_gapQueue.consumerPaused && !r.resumed && !r.fenceAccepted && !r.probeAcquired &&
            r.pauseAcquisitionSequence == 0ULL && r.pauseAcquisitionSeal == 0ULL &&
            r.pauseAcceptedSequence == 0ULL && r.pauseSampleMs == 0ULL && r.pauseSampleSeal == 0ULL &&
            r.resumedAtMs == 0ULL && r.resumedAtSeal == 0ULL &&
            r.publication == 0ULL && r.publicationSeal == 0ULL && r.probeSequence == 0ULL &&
            equal(r.frame, GapPathSampleFrame{}) && equal(r.sealedFrame, GapPathSampleFrame{}) &&
            !(m_gapSignal.dropConsumed && m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex);
    }
    if (!m_gapInlet.accepted || !m_gapSignal.sourceSampleSeen ||
        (!previousSource && !m_gapWindow.normalSource) ||
        m_gapSignal.sourceIndex != m_gapSignal.dropTargetSourceIndex ||
        !m_gapSignal.dropStarted || !m_gapSignal.dropConsumed ||
        m_gapQueue.pauseStartMs != m_gapSignal.dropStartMs ||
        r.pauseAcceptedSequence != m_gapInlet.sourceFloor + 1ULL ||
        r.pauseAcceptedSequence > m_gapPath.sequence || r.pauseSampleMs != r.pauseSampleSeal ||
        m_gapQueue.pauseStartMs < r.pauseSampleMs || m_gapQueue.pauseStartMs - r.pauseSampleMs < 20ULL ||
        r.pauseAcquisitionSequence != r.pauseAcquisitionSeal ||
        r.pauseAcquisitionSequence < r.pauseAcceptedSequence ||
        r.pauseAcquisitionSequence > m_gapInlet.acquisitionSequence ||
        r.probeAcquired != (m_gapInlet.acquisitionSequence > r.pauseAcquisitionSequence) ||
        (r.probeAcquired ? r.probeSequence != r.pauseAcquisitionSequence + 1ULL : r.probeSequence != 0ULL)) return false;
    if (!r.resumed)
    {
        return m_gapQueue.consumerPaused && !r.fenceAccepted &&
            m_gapPath.sequence == r.pauseAcceptedSequence && m_gapInlet.acceptedFrame.sample.sampledAtMs == r.pauseSampleMs &&
            r.resumedAtMs == 0ULL && r.resumedAtSeal == 0ULL &&
            r.publication == 0ULL && r.publicationSeal == 0ULL &&
            equal(r.frame, GapPathSampleFrame{}) && equal(r.sealedFrame, GapPathSampleFrame{});
    }
    const auto& f = r.frame;
    const auto& identity = previousSource ? m_gapWindow.previousIdentity : m_pathHold.identity;
    if (m_gapQueue.consumerPaused || !r.probeAcquired || !equal(f, r.sealedFrame) ||
        r.resumedAtMs != r.resumedAtSeal || r.resumedAtMs < m_gapQueue.pauseStartMs ||
        r.resumedAtMs - m_gapQueue.pauseStartMs < 40ULL ||
        r.publication == 0ULL || r.publication != r.publicationSeal ||
        f.publication == 0ULL || f.publication > r.publication || r.publication > m_gapSignal.publication ||
        f.sample.source != EDMGap::Source::SIMULATED || !f.sample.valid ||
        f.run != m_gapWindow.run || f.cache != m_gapWindow.cache ||
        f.sourceIndex != m_gapSignal.sourceIndex || !HoldIdentityEqual(f.identity, identity) ||
        !f.lease.Matches(m_gapWindow.lease) || f.sample.sequence < r.probeSequence ||
        f.sample.sequence <= r.pauseAcquisitionSequence || f.sample.sequence > m_gapInlet.acquisitionSequence ||
        f.sample.sampledAtMs > r.resumedAtMs || r.resumedAtMs - f.sample.sampledAtMs > 100ULL ||
        r.fenceAccepted != (m_gapPath.sequence >= f.sample.sequence)) return false;
    if (m_gapPath.sequence == f.sample.sequence && !equal(f, m_gapInlet.acceptedFrame)) return false;
    if (m_gapPath.sequence < f.sample.sequence)
    {
        const std::uint64_t offset = f.sample.sequence - m_gapPath.sequence - 1ULL;
        if (offset >= m_gapQueue.count || offset >= 3ULL ||
            !equal(f, m_gapQueue.frames[(m_gapQueue.head + static_cast<std::uint32_t>(offset)) % 3U])) return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathQueueRecoverySameThread(const char* phase, std::uint64_t nowMs) const noexcept
{
    const auto& r = m_gapRecovery;
    const auto& gap = m_gapInput.Current();
    RtPrintf("[GAP-DB] phase=%s run=%llu source=%u dispatch=%llu epoch=%llu depth=%u acquired=%llu accepted=%llu pauseSeq=%llu probeSeq=%llu recoveryFence=%llu recoveryPub=%llu resumed=%u recovered=%u U=%u\n",
        phase, static_cast<unsigned long long>(m_gapSignal.run), m_gapSignal.sourceIndex,
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        m_gapQueue.count, static_cast<unsigned long long>(m_gapInlet.acquisitionSequence),
        static_cast<unsigned long long>(m_gapPath.sequence), static_cast<unsigned long long>(r.pauseAcquisitionSequence),
        static_cast<unsigned long long>(r.probeSequence), static_cast<unsigned long long>(r.frame.sample.sequence),
        static_cast<unsigned long long>(r.publication), r.resumed ? 1U : 0U, r.fenceAccepted ? 1U : 0U,
        static_cast<unsigned int>(m_gapWindow.queuedProbeVoltage));
    RtPrintf("[GAP-DB-SAMPLE] frameSeq=%llu frameMs=%llu frameMv=%d fenceMs=%llu fenceMv=%d sampleSeq=%llu sampleMs=%llu nowMs=%llu ageMs=%llu pauseMs=%llu resumeMs=%llu mv=%d quality=%s band=%s pending=%s\n",
        static_cast<unsigned long long>(m_gapInlet.frame.sample.sequence), static_cast<unsigned long long>(m_gapInlet.frame.sample.sampledAtMs),
        m_gapInlet.frame.sample.voltageMv, static_cast<unsigned long long>(r.frame.sample.sampledAtMs), r.frame.sample.voltageMv,
        static_cast<unsigned long long>(gap.sequence), static_cast<unsigned long long>(gap.sampledAtMs),
        static_cast<unsigned long long>(nowMs), static_cast<unsigned long long>(nowMs >= gap.sampledAtMs ? nowMs - gap.sampledAtMs : 0ULL),
        static_cast<unsigned long long>(m_gapQueue.pauseStartMs), static_cast<unsigned long long>(r.resumedAtMs),
        gap.voltageMv, EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand));
}

// Called only after normal/tail Motion completion and current own NORMAL proof.
// Freeze acquisition, then inspect every queued sample before source retirement.
NC_PATH_HOLD_NOINLINE
bool NCManager::CompleteGapPathSampleQueueSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!m_gapQueue.active) return true;
    if (!ValidateGapPathSampleFrameSameThread() || !ValidateGapPathSampleQueueSameThread())
    {
        RejectGapPathSimulationSameThread("DA_QUEUE_INVALID", &snapshot);
        return false;
    }
    if (m_gapQueue.consumerPaused || (m_gapRecovery.active &&
        m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex && !m_gapRecovery.fenceAccepted)) return false;
    const bool line = m_pathFeed.armed && m_pathFeed.bound && m_pathFeed.completed &&
        m_pathFeed.consumerAccepted && m_pathFeed.consumerStarted && m_pathFeedMotion.receipt.valid &&
        m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.commit == m_pathHold.commit &&
        (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_pathHold.identity)) &&
        m_pathFeedMotion.receipt.ownerLease.Matches(m_pathHold.lease);
    const bool arc = m_pathArc.armed && m_pathArc.bound && m_pathArc.completed &&
        m_pathArc.consumerAccepted && m_pathArc.consumerStarted && m_pathArcMotion.receipt.valid &&
        m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.commit == m_pathHold.commit &&
        (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_pathHold.identity)) &&
        m_pathArcMotion.receipt.ownerLease.Matches(m_pathHold.lease);
    if (line == arc || !m_pathHold.bound || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        !snapshot.ownerLease.Matches(m_pathHold.lease) || snapshot.admissionPending ||
        snapshot.publicationSequence != m_gapSignal.publication ||
        !std::isfinite(snapshot.activeS) || !std::isfinite(snapshot.lengthPulse) || snapshot.lengthPulse <= 0.0 ||
        std::abs(snapshot.activeS - snapshot.lengthPulse) > 0.001 + 1e-9 * snapshot.lengthPulse ||
        !m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
        m_motion.GetCommandReplaySize() != 0U || !IsGapPathCurrentSampleProvenSameThread())
    {
        RejectGapPathSimulationSameThread("DA_TERMINAL_SCOPE", &snapshot);
        return false;
    }
    const auto& gap = m_gapInput.Current();
    if (gap.band != EDMGap::Band::NORMAL || gap.pendingBand != EDMGap::Band::UNKNOWN ||
        (m_gapTail.active && !m_gapTail.sampleSeen)) return false;
    if (!m_gapQueue.terminalSealed)
    {
        m_gapQueue.terminalFrame = m_gapInlet.frame;
        m_gapQueue.terminalSeal = m_gapInlet.frame;
        m_gapQueue.terminalPublication = snapshot.publicationSequence;
        m_gapQueue.terminalPublicationSeal = snapshot.publicationSequence;
        m_gapQueue.terminalSealed = true;
        LogGapPathSampleQueueSameThread("TERMINAL_SEALED", m_gapPath.lastServiceMs);
    }
    if (m_gapQueue.count != 0U) return false;
    if (m_gapPath.sequence != m_gapQueue.terminalFrame.sample.sequence)
    {
        RejectGapPathSimulationSameThread("DA_TERMINAL_NOT_DRAINED", &snapshot);
        return false;
    }
    if (!m_gapQueue.terminalDrained)
    {
        m_gapQueue.terminalDrained = true;
        LogGapPathSampleQueueSameThread("TERMINAL_DRAINED", m_gapPath.lastServiceMs);
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSampleQueueSameThread(const char* phase, std::uint64_t nowMs) const noexcept
{
    const auto& gap = m_gapInput.Current();
    RtPrintf("[GAP-DA] phase=%s run=%llu source=%u dispatch=%llu epoch=%llu depth=%u head=%u acquired=%llu accepted=%llu floor=%llu paused=%u sealed=%u drained=%u fence=%llu terminalPub=%llu\n",
        phase, static_cast<unsigned long long>(m_gapSignal.run), m_gapSignal.sourceIndex,
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        m_gapQueue.count, m_gapQueue.head, static_cast<unsigned long long>(m_gapInlet.acquisitionSequence),
        static_cast<unsigned long long>(m_gapPath.sequence), static_cast<unsigned long long>(m_gapInlet.sourceFloor),
        m_gapQueue.consumerPaused ? 1U : 0U, m_gapQueue.terminalSealed ? 1U : 0U, m_gapQueue.terminalDrained ? 1U : 0U,
        static_cast<unsigned long long>(m_gapQueue.terminalFrame.sample.sequence),
        static_cast<unsigned long long>(m_gapQueue.terminalPublication));
    RtPrintf("[GAP-DA-SAMPLE] frameSeq=%llu frameMs=%llu sampleSeq=%llu sampleMs=%llu nowMs=%llu ageMs=%llu acqPub=%llu acceptedPub=%llu quality=%s band=%s pending=%s\n",
        static_cast<unsigned long long>(m_gapInlet.frame.sample.sequence), static_cast<unsigned long long>(m_gapInlet.frame.sample.sampledAtMs),
        static_cast<unsigned long long>(gap.sequence), static_cast<unsigned long long>(gap.sampledAtMs),
        static_cast<unsigned long long>(nowMs), static_cast<unsigned long long>(nowMs >= gap.sampledAtMs ? nowMs - gap.sampledAtMs : 0ULL),
        static_cast<unsigned long long>(m_gapInlet.frame.publication), static_cast<unsigned long long>(m_gapInlet.acceptedPublication),
        EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::PrepareGapPathSampleAcquisitionSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot, std::uint64_t nowMs, bool& acquire, bool acquisitionIntent) noexcept
{
    acquire = false;
    if (!ValidateGapPathSampleFrameSameThread())
    {
        RejectGapPathSimulationSameThread("CZ_FRAME_INVALID", &snapshot);
        return false;
    }
    if (!ValidateGapPathSampleQueueSameThread())
    {
        RejectGapPathSimulationSameThread("DA_QUEUE_INVALID", &snapshot);
        return false;
    }
    if (!m_pathHold.bound || !m_gapPath.active || m_gapDryRun.active ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) || m_mode != NCOperationMode::MEMORY ||
        AlarmManager::GetInstance().HasAlarm() || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        m_gapSignal.run != m_gapWindow.run || m_gapSignal.cache != m_gapWindow.cache ||
        m_gapSignal.sourceIndex != m_gapWindow.sourceIndex ||
        m_pathHold.run != m_gapWindow.run || m_pathHold.cache != m_gapWindow.cache ||
        !m_pathHold.lease.Matches(m_gapWindow.lease) || !m_pathHold.lease.Matches(m_programMotionLease) ||
        snapshot.publicationSequence == 0ULL || snapshot.publicationSequence < m_gapSignal.publication ||
        snapshot.phase == MotionPathCoreHoldExcursionPhase::REJECTED ||
        snapshot.phase == MotionPathCoreHoldExcursionPhase::INVALIDATED ||
        snapshot.admissionPending || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        !snapshot.ownerLease.Matches(m_pathHold.lease) ||
        m_pathHold.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathHold.cache != GetBaseProgramCache().GetGeneration() ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        !HasGapPathSourceConsumerAcceptedSameThread())
    {
        RejectGapPathSimulationSameThread("CZ_FRAME_SCOPE", &snapshot);
        return false;
    }
    // Expire the previously accepted physical-time sample BEFORE replacement.
    if (m_gapPath.sequence != 0ULL && m_gapInput.Poll(nowMs).quality != EDMGap::Quality::VALID)
    {
        LogGapPathSampleInletSameThread("SAMPLE_STALE", nowMs);
        RejectGapPathSimulationSameThread("CZ_SAMPLE_STALE", &snapshot);
        return false;
    }
    if (m_gapInlet.clockStarted && nowMs < m_gapInlet.lastAcquiredMs)
    {
        RejectGapPathSimulationSameThread("CZ_ACQUISITION_CLOCK", &snapshot);
        return false;
    }
    if (m_gapInlet.freezeStarted)
    {
        if (nowMs < m_gapInlet.freezeStartMs)
        {
            RejectGapPathSimulationSameThread("CZ_ACQUISITION_CLOCK", &snapshot);
            return false;
        }
        if (nowMs - m_gapInlet.freezeStartMs < 150ULL) return true;
    }
    if (m_gapQueue.active && m_gapQueue.terminalSealed) return true;
    acquire = !m_gapInlet.clockStarted || nowMs - m_gapInlet.lastAcquiredMs >= 20ULL;
    // Reject before either the sample or its voltage-model transition changes.
    if (acquisitionIntent && acquire && m_gapQueue.active && m_gapQueue.count == 3U)
    {
        LogGapPathSampleQueueSameThread("QUEUE_FULL", nowMs);
        RejectGapPathSimulationSameThread("DA_SAMPLE_QUEUE_FULL", &snapshot);
        return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::AcquireGapPathSampleSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot,
    std::uint64_t nowMs, std::int32_t voltageMv) noexcept
{
    bool acquire = false;
    if (!PrepareGapPathSampleAcquisitionSameThread(snapshot, nowMs, acquire)) return false;
    if (!acquire) return true;
    if (m_gapInlet.acquisitionSequence == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapPathSimulationSameThread("CZ_SAMPLE_SEQUENCE", &snapshot);
        return false;
    }
    const bool probe = m_gapRecovery.active && m_gapQueue.pauseConsumed && !m_gapRecovery.probeAcquired;
    GapPathSampleFrame next{};
    next.sample.source = EDMGap::Source::SIMULATED;
    next.sample.valid = true;
    next.sample.voltageMv = probe ? static_cast<std::int32_t>(m_gapWindow.queuedProbeVoltage) * 1000 : voltageMv;
    next.sample.sequence = m_gapInlet.acquisitionSequence + 1ULL;
    next.sample.sampledAtMs = nowMs; // Actual acquisition time; never catch-up ticks.
    next.identity = snapshot.identity;
    next.lease = snapshot.ownerLease;
    next.run = m_gapWindow.run;
    next.cache = m_gapWindow.cache;
    next.sourceIndex = m_gapWindow.sourceIndex;
    next.publication = snapshot.publicationSequence;
    m_gapInlet.frame = next;
    m_gapInlet.sealedFrame = next;
    m_gapInlet.acquisitionSequence = next.sample.sequence;
    m_gapInlet.lastAcquiredMs = nowMs;
    m_gapInlet.clockStarted = true;
    m_gapInlet.available = true;
    if (m_gapQueue.active)
    {
        const std::uint32_t slot = (m_gapQueue.head + m_gapQueue.count) % 3U;
        m_gapQueue.frames[slot] = next;
        m_gapQueue.seals[slot] = next;
        ++m_gapQueue.count;
        if (probe)
        {
            m_gapRecovery.probeAcquired = true;
            m_gapRecovery.probeSequence = next.sample.sequence;
            LogGapPathQueueRecoverySameThread("QUEUED_PROBE_ACQUIRED", nowMs);
        }
        if (!m_gapQueue.queuedLogged || m_gapQueue.consumerPaused)
        {
            m_gapQueue.queuedLogged = true;
            LogGapPathSampleQueueSameThread(m_gapQueue.consumerPaused ? "PAUSED_ACQUISITION" : "SOURCE_QUEUED", nowMs);
        }
    }
    if (!m_gapInlet.acquiredLogged)
    {
        m_gapInlet.acquiredLogged = true;
        LogGapPathSampleInletSameThread("SOURCE_ACQUIRED", nowMs);
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ConsumeGapPathSampleSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot,
    std::uint64_t nowMs, bool freshPublication, bool& accepted) noexcept
{
    accepted = false;
    bool unusedAcquisition = false;
    if (!PrepareGapPathSampleAcquisitionSameThread(snapshot, nowMs, unusedAcquisition, false)) return false;
    if (!m_gapInlet.available) return true;
    if (m_gapQueue.active)
    {
        if (m_gapQueue.consumerPaused)
        {
            if (nowMs < m_gapQueue.pauseStartMs ||
                (!m_gapRecovery.active && nowMs - m_gapQueue.pauseStartMs >= 150ULL))
            {
                RejectGapPathSimulationSameThread(m_gapRecovery.active ? "DB_CONSUMER_PAUSE_CLOCK" : "DA_CONSUMER_PAUSE_CLOCK", &snapshot);
                return false;
            }
            if (!m_gapRecovery.active || nowMs - m_gapQueue.pauseStartMs < 40ULL ||
                !freshPublication || snapshot.publicationSequence <= m_gapInlet.acceptedPublication) return true;
            // Producer ran first. Pin exactly what was acquired; do not flush or refresh it.
            if (!m_gapRecovery.probeAcquired || m_gapQueue.count == 0U ||
                m_gapInlet.frame.publication > snapshot.publicationSequence ||
                nowMs < m_gapInlet.frame.sample.sampledAtMs || nowMs - m_gapInlet.frame.sample.sampledAtMs > 100ULL)
            {
                RejectGapPathSimulationSameThread("DB_RECOVERY_INVALID", &snapshot);
                return false;
            }
            m_gapRecovery.frame = m_gapInlet.frame;
            m_gapRecovery.sealedFrame = m_gapInlet.frame;
            m_gapRecovery.resumedAtMs = nowMs;
            m_gapRecovery.resumedAtSeal = nowMs;
            m_gapRecovery.publication = snapshot.publicationSequence;
            m_gapRecovery.publicationSeal = snapshot.publicationSequence;
            m_gapRecovery.resumed = true;
            m_gapQueue.consumerPaused = false;
            m_gapServiceCurrent.site = "DB_CONSUMER_RECOVERY";
            LogGapPathQueueRecoverySameThread("CONSUMER_RESUMED", nowMs);
        }
        if (m_gapQueue.count == 0U) return true;
    }
    const GapPathSampleFrame frame = m_gapQueue.active ? m_gapQueue.frames[m_gapQueue.head] : m_gapInlet.frame;
    if (snapshot.publicationSequence < frame.publication ||
        snapshot.publicationSequence < m_gapInlet.acceptedPublication ||
        nowMs < frame.sample.sampledAtMs || nowMs - frame.sample.sampledAtMs > 100ULL)
    {
        RejectGapPathSimulationSameThread("CZ_FRAME_SCOPE", &snapshot);
        return false;
    }
    if (m_gapQueue.active && nowMs - frame.sample.sampledAtMs < 20ULL) return true;
    if (frame.sample.sequence == m_gapPath.sequence)
    {
        // Poll above observes time, but the immutable sample never gets restamped
        // or re-published; duplicate reads cannot qualify dwell or a new source.
        if (!m_gapInlet.accepted || !m_gapSignal.sourceSampleSeen)
        {
            RejectGapPathSimulationSameThread("CZ_FRAME_INVALID", &snapshot);
            return false;
        }
        if (!m_gapInlet.cachedLogged)
        {
            m_gapInlet.cachedLogged = true;
            LogGapPathSampleInletSameThread("CACHED_READ", nowMs);
        }
        return true;
    }
    if (frame.sample.sequence < m_gapPath.sequence)
    {
        RejectGapPathSimulationSameThread("CZ_SAMPLE_SEQUENCE", &snapshot);
        return false;
    }
    if (!freshPublication || snapshot.publicationSequence <= m_gapInlet.acceptedPublication) return true;
    m_gapServiceCurrent.publishSample = true; // P19: an actual Monitor publication, not a Motion hint.
    const EDMGap::Snapshot& gap = m_gapInput.Publish(frame.sample, nowMs);
    if (gap.quality != EDMGap::Quality::VALID || gap.source != EDMGap::Source::SIMULATED)
    {
        RejectGapPathSimulationSameThread("CZ_INPUT_NOT_VALID", &snapshot);
        return false;
    }
    const bool first = !m_gapInlet.accepted;
    m_gapInlet.acceptedFrame = frame;
    m_gapInlet.acceptedPublication = snapshot.publicationSequence;
    m_gapInlet.accepted = true;
    m_gapPath.sequence = frame.sample.sequence;
    m_gapSignal.sourceSampleSeen = true;
    m_gapServiceAgeOnlyCalls = 0U;
    accepted = true;
    if (m_gapQueue.active)
    {
        m_gapQueue.frames[m_gapQueue.head] = GapPathSampleFrame{};
        m_gapQueue.seals[m_gapQueue.head] = GapPathSampleFrame{};
        m_gapQueue.head = (m_gapQueue.head + 1U) % 3U;
        --m_gapQueue.count;
        if (m_gapRecovery.active && m_gapRecovery.resumed && !m_gapRecovery.fenceAccepted)
        {
            m_gapRecovery.fenceAccepted = frame.sample.sequence == m_gapRecovery.frame.sample.sequence;
            LogGapPathQueueRecoverySameThread(m_gapRecovery.fenceAccepted ? "RECOVERY_FENCE_ACCEPTED" : "RECOVERY_ACCEPTED", nowMs);
        }
        if (!m_gapQueue.acceptedLogged)
        {
            m_gapQueue.acceptedLogged = true;
            LogGapPathSampleQueueSameThread("DELAYED_ACCEPTED", nowMs);
        }
    }
    if (first)
    {
        LogGapPathSampleInletSameThread("SOURCE_ACCEPTED", nowMs);
        LogGapPathSignalSessionSameThread("SOURCE_SAMPLE_READY");
        if ((m_gapWindow.sourceGapMs == 150U || m_gapRecovery.active) &&
            m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex)
        {
            if (!m_gapWindow.normalSource || m_gapInlet.freezeConsumed || m_gapQueue.pauseConsumed)
            {
                RejectGapPathSimulationSameThread("CZ_FREEZE_SCOPE", &snapshot);
                return false;
            }
            if (m_gapQueue.active)
            {
                m_gapQueue.consumerPaused = true;
                m_gapQueue.pauseConsumed = true;
                m_gapQueue.pauseStartMs = nowMs;
                if (m_gapRecovery.active)
                {
                    m_gapRecovery.pauseAcquisitionSequence = m_gapInlet.acquisitionSequence;
                    m_gapRecovery.pauseAcquisitionSeal = m_gapInlet.acquisitionSequence;
                    m_gapRecovery.pauseAcceptedSequence = frame.sample.sequence;
                    m_gapRecovery.pauseSampleMs = frame.sample.sampledAtMs;
                    m_gapRecovery.pauseSampleSeal = frame.sample.sampledAtMs;
                }
                m_gapServiceCurrent.site = m_gapRecovery.active ? "DB_CONSUMER_PAUSED" : "DA_CONSUMER_PAUSED";
            }
            else
            {
                m_gapInlet.freezeStarted = true;
                m_gapInlet.freezeConsumed = true;
                m_gapInlet.freezeStartMs = nowMs;
                m_gapServiceCurrent.site = "CZ_PRODUCER_FROZEN";
            }
            m_gapSignal.dropStarted = true;
            m_gapSignal.dropConsumed = true;
            m_gapSignal.dropStartMs = nowMs;
            m_gapInlet.cachedLogged = false;
            if (m_gapRecovery.active) LogGapPathQueueRecoverySameThread("CONSUMER_PAUSED_40MS", nowMs);
            else if (m_gapQueue.active) LogGapPathSampleQueueSameThread("CONSUMER_PAUSED_150MS", nowMs);
            else LogGapPathSampleInletSameThread("PRODUCER_FROZEN_150MS", nowMs);
        }
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathCurrentSampleProvenSameThread() const noexcept
{
    if (!m_gapInlet.active) return m_gapInput.Current().sampledAtMs == m_gapPath.lastServiceMs;
    const auto& gap = m_gapInput.Current();
    return ValidateGapPathSampleFrameSameThread() && ValidateGapPathSampleQueueSameThread() &&
        m_gapInlet.accepted && m_gapSignal.sourceSampleSeen &&
        m_gapInlet.acceptedFrame.sample.sequence > m_gapInlet.sourceFloor &&
        m_gapInlet.acceptedPublication != 0ULL &&
        m_gapInlet.acceptedPublication <= m_gapSignal.publication && m_gapPath.sequence != 0ULL &&
        gap.configured && gap.quality == EDMGap::Quality::VALID &&
        gap.source == EDMGap::Source::SIMULATED && gap.sequence == m_gapPath.sequence &&
        gap.observedAtMs == m_gapPath.lastServiceMs && gap.sampledAtMs <= m_gapPath.lastServiceMs &&
        m_gapPath.lastServiceMs - gap.sampledAtMs <= 100ULL &&
        m_pathHold.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease);
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSampleInletSameThread(const char* phase, std::uint64_t nowMs) const noexcept
{
    const auto& frame = m_gapInlet.frame;
    const auto& gap = m_gapInput.Current();
    RtPrintf("[GAP-CZ] phase=%s run=%llu source=%u dispatch=%llu epoch=%llu acquired=%llu accepted=%llu floor=%llu acqPub=%llu acceptedPub=%llu observedPub=%llu\n",
        phase, static_cast<unsigned long long>(m_gapSignal.run), m_gapSignal.sourceIndex,
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_gapInlet.acquisitionSequence), static_cast<unsigned long long>(m_gapPath.sequence),
        static_cast<unsigned long long>(m_gapInlet.sourceFloor), static_cast<unsigned long long>(frame.publication),
        static_cast<unsigned long long>(m_gapInlet.acceptedPublication), static_cast<unsigned long long>(m_gapSignal.publication));
    RtPrintf("[GAP-CZ-SAMPLE] frameSeq=%llu frameMs=%llu sampleSeq=%llu sampleMs=%llu nowMs=%llu ageMs=%llu quality=%s band=%s pending=%s frozen=%u\n",
        static_cast<unsigned long long>(frame.sample.sequence), static_cast<unsigned long long>(frame.sample.sampledAtMs),
        static_cast<unsigned long long>(gap.sequence), static_cast<unsigned long long>(gap.sampledAtMs),
        static_cast<unsigned long long>(nowMs),
        static_cast<unsigned long long>(nowMs >= gap.sampledAtMs ? nowMs - gap.sampledAtMs : 0ULL),
        EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand),
        m_gapInlet.freezeStarted ? 1U : 0U);
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ServiceGapPathSimulationSameThread(double activeS, bool publishSample, const char* site,
    bool returningSample, const MotionPathCoreHoldExcursionSnapshot* pendingAdmission,
    const MotionPathCoreHoldExcursionSnapshot* sourcePublication) noexcept
{
    if (!m_gapPath.active) return false;
    GapServiceDiagnostic& diagnostic = m_gapServiceCurrent;
    diagnostic = GapServiceDiagnostic{};
    diagnostic.present = true;
    diagnostic.site = site != nullptr ? site : "AUTOMATIC";
    if (m_gapSignal.active && m_gapSignal.dropStarted &&
        m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex)
        diagnostic.site = m_gapRecovery.active ? (m_gapQueue.consumerPaused ? "DB_CONSUMER_PAUSED" : (m_gapRecovery.fenceAccepted ? "DB_CONSUMER_RECOVERED" : "DB_CONSUMER_RECOVERY")) : m_gapQueue.active ? "DA_CONSUMER_PAUSED" : m_gapInlet.active ? "CZ_PRODUCER_FROZEN" : "CX_SOURCE_SAMPLE_DROP";
    diagnostic.lastServiceMs = m_gapPath.lastServiceMs;
    diagnostic.lastServiceValid = m_gapPath.clockStarted;
    diagnostic.sampledAtMs = m_gapInput.Current().sampledAtMs;
    diagnostic.sampleSequence = m_gapInput.Current().sequence;
    diagnostic.quality = m_gapInput.Current().quality;
    diagnostic.publishSample = !m_gapInlet.active && publishSample;
    const bool requestedSample = publishSample;
    diagnostic.run = m_pathHold.run;
    diagnostic.dispatch = m_pathHold.dispatch;
    diagnostic.admissionOriginMs = m_gapPath.firstServiceMs;
    diagnostic.admissionHeartbeatMs = m_gapAdmission.heartbeatMs;
    diagnostic.sourceSampleOriginMs = m_gapAdmission.sampleOriginMs;
    diagnostic.admissionSeen = m_gapAdmission.seen;
    diagnostic.admissionClosed = m_gapAdmission.closed;
    diagnostic.sourceSampleClockStarted = m_gapAdmission.sampleClockStarted;
    if (m_gapSignal.active &&
        (!m_gapWindow.active || !m_gapWindow.continuousSignal ||
            m_gapSignal.run != m_gapWindow.run || m_gapSignal.cache != m_gapWindow.cache ||
            m_gapSignal.sourceIndex != m_gapWindow.sourceIndex ||
            m_gapSignal.sourceStartSequence > m_gapPath.sequence ||
            m_gapSignal.sourceSampleSeen != (m_gapPath.sequence > m_gapSignal.sourceStartSequence) ||
            m_gapInput.Current().sequence != m_gapPath.sequence ||
            (!m_gapSignal.sourceSampleSeen && m_gapSignal.sourceStartSequence != 0ULL &&
                m_gapInput.Current().sampledAtMs != m_gapSignal.sourceSampledAtMs)))
    {
        RejectGapPathSimulationSameThread("CX_SIGNAL_SESSION_CHANGED", pendingAdmission);
        return false;
    }
    if (m_gapSignal.active && m_gapSignal.sourceSampleSeen && !HasGapPathSourceConsumerAcceptedSameThread())
    {
        RejectGapPathSimulationSameThread("CX_SOURCE_ACCEPTANCE_LOST", sourcePublication);
        return false;
    }
    if (!publishSample && m_gapServiceAgeOnlyCalls != (std::numeric_limits<std::uint32_t>::max)())
        ++m_gapServiceAgeOnlyCalls;
    diagnostic.ageOnlyCalls = m_gapServiceAgeOnlyCalls;
    LARGE_INTEGER counter{};
    if (m_gapDryRun.active || m_gapPath.frequency == 0ULL ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0)
    {
        RejectGapPathSimulationSameThread("CLOCK_READ_OR_SESSION", pendingAdmission);
        return false;
    }
    const std::uint64_t ticks = static_cast<std::uint64_t>(counter.QuadPart);
    const std::uint64_t seconds = ticks / m_gapPath.frequency;
    const std::uint64_t fraction = (ticks % m_gapPath.frequency) * 1000ULL / m_gapPath.frequency;
    if (seconds <= ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000ULL)
    {
        diagnostic.nowMs = seconds * 1000ULL + fraction;
        diagnostic.nowValid = true;
    }
    if ((m_gapPath.clockStarted && ticks < m_gapPath.lastTicks) ||
        seconds > ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000ULL)
    {
        RejectGapPathSimulationSameThread("CLOCK_REGRESSION_OR_RANGE", pendingAdmission);
        return false;
    }
    const std::uint64_t nowMs = seconds * 1000ULL + fraction;
    const bool sourceUnsampled = m_gapSignal.active ? !m_gapSignal.sourceSampleSeen : m_gapPath.sequence == 0ULL;
    if (m_gapSignal.active && !m_gapSignal.sourceClockStarted)
    {
        m_gapSignal.sourceFirstServiceMs = nowMs;
        m_gapSignal.sourceClockStarted = true;
    }
    const std::uint64_t sourceOriginMs = m_gapSignal.active ?
        m_gapSignal.sourceFirstServiceMs : m_gapPath.firstServiceMs;
    if (m_gapPath.clockStarted)
    {
        if (nowMs < m_gapPath.lastServiceMs || nowMs - m_gapPath.lastServiceMs > 250ULL)
        {
            RejectGapPathSimulationSameThread("SERVICE_GAP", pendingAdmission);
            return false;
        }
        m_gapPath.stalledCalls = nowMs == m_gapPath.lastServiceMs ? m_gapPath.stalledCalls + 1U : 0U;
        if (m_gapPath.stalledCalls >= 4096U)
        {
            RejectGapPathSimulationSameThread("CLOCK_NOT_ADVANCING", pendingAdmission);
            return false;
        }
        if (m_gapPath.sequence != 0ULL && m_gapInput.Poll(nowMs).quality != EDMGap::Quality::VALID)
        {
            const bool stale = m_gapSignal.active && m_gapInput.Current().quality == EDMGap::Quality::STALE;
            if (stale) LogGapPathSignalSessionSameThread("SIGNAL_STALE");
            if (stale && m_gapInlet.active) LogGapPathSampleInletSameThread("SAMPLE_STALE", nowMs);
            RejectGapPathSimulationSameThread(stale ? (m_gapInlet.active ? "CZ_SAMPLE_STALE" : "CX_SIGNAL_STALE") : "INPUT_QUALITY_OR_AGE", pendingAdmission);
            return false;
        }
        if (sourceUnsampled)
        {
            if (!m_gapAdmission.seen && nowMs - sourceOriginMs > 100ULL)
            {
                RejectGapPathSimulationSameThread("SOURCE_SAMPLE_UNAVAILABLE", pendingAdmission);
                return false;
            }
            if (m_gapAdmission.seen && !m_gapAdmission.closed)
            {
                // Chosen admission policy: 1000 ms total from the earliest source
                // service, with no refresh by heartbeat, source or axis changes.
                if (nowMs - sourceOriginMs > 1000ULL)
                {
                    RejectGapPathSimulationSameThread("SOURCE_ADMISSION_TIMEOUT", pendingAdmission);
                    return false;
                }
                if (nowMs < m_gapAdmission.heartbeatMs || nowMs - m_gapAdmission.heartbeatMs > 100ULL)
                {
                    RejectGapPathSimulationSameThread("ADMISSION_HEARTBEAT_STALE", pendingAdmission);
                    return false;
                }
            }
            if (m_gapAdmission.closed && m_gapAdmission.sampleClockStarted &&
                (nowMs < m_gapAdmission.sampleOriginMs || nowMs - m_gapAdmission.sampleOriginMs > 100ULL))
            {
                RejectGapPathSimulationSameThread("SOURCE_SAMPLE_UNAVAILABLE", pendingAdmission);
                return false;
            }
        }
    }
    else m_gapPath.firstServiceMs = nowMs;
    bool sampledSourceReady = false;
    if (m_gapSignal.active && (publishSample || (m_gapInlet.active && sourcePublication != nullptr)))
    {
        if (sourcePublication == nullptr || sourcePublication->publicationSequence == 0ULL ||
            sourcePublication->admissionPending || !m_pathHold.bound ||
            !HoldIdentityEqual(sourcePublication->identity, m_pathHold.identity) ||
            !sourcePublication->ownerLease.Matches(m_pathHold.lease) ||
            m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease))
        {
            RejectGapPathSimulationSameThread("CX_SOURCE_PUBLICATION_INVALID", sourcePublication);
            return false;
        }
        if (!HasGapPathSourceConsumerAcceptedSameThread())
        {
            if (m_gapSignal.sourceSampleSeen)
            {
                RejectGapPathSimulationSameThread("CX_SOURCE_ACCEPTANCE_LOST", sourcePublication);
                return false;
            }
            // RT can load after ProcessMotionFeedback ran in this NC scan.
            // That loaded publication is not yet this source's sample frontier.
            publishSample = false;
            diagnostic.site = "CX_CONSUMER_WAIT";
        }
        else
        {
            if (sourcePublication->publicationSequence < m_gapSignal.publication)
            {
                RejectGapPathSimulationSameThread("CX_SOURCE_PUBLICATION_REGRESSION", sourcePublication);
                return false;
            }
            publishSample = sourcePublication->publicationSequence > m_gapSignal.publication;
            m_gapSignal.publication = sourcePublication->publicationSequence;
            m_gapSignal.handoffPending = false;
            sampledSourceReady = m_gapInlet.active;
            if (!m_gapInlet.active && publishSample && m_gapWindow.sourceGapMs == 150U &&
                m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex)
            {
                if (!m_gapWindow.normalSource)
                {
                    RejectGapPathSimulationSameThread("HOOK_TARGET_NOT_NORMAL", sourcePublication);
                    return false;
                }
                if (!m_gapSignal.dropStarted)
                {
                    if (m_gapSignal.dropConsumed || m_gapSignal.sourceSampleSeen)
                    {
                        RejectGapPathSimulationSameThread("HOOK_ALREADY_CONSUMED", sourcePublication);
                        return false;
                    }
                    m_gapSignal.dropStarted = true;
                    m_gapSignal.dropConsumed = true;
                    m_gapSignal.dropStartMs = nowMs;
                    LogGapPathSignalSessionSameThread("SOURCE_SAMPLE_DROP");
                }
                if (nowMs < m_gapSignal.dropStartMs)
                {
                    RejectGapPathSimulationSameThread("HOOK_CLOCK_REGRESSION", sourcePublication);
                    return false;
                }
                if (nowMs - m_gapSignal.dropStartMs < 150ULL) publishSample = false;
            }
            if (!publishSample) diagnostic.site = m_gapSignal.dropStarted &&
                (!m_gapRecovery.active || m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex) ?
                (m_gapRecovery.active ? (m_gapQueue.consumerPaused ? "DB_CONSUMER_PAUSED" : (m_gapRecovery.fenceAccepted ? "DB_RECOVERED_PUBLICATION_WAIT" : "DB_RECOVERY_PUBLICATION_WAIT")) : m_gapQueue.active ? "DA_CONSUMER_PAUSED" : m_gapInlet.active ? "CZ_PRODUCER_FROZEN" : "CX_SOURCE_SAMPLE_DROP") : "CX_PUBLICATION_WAIT";
        }
        diagnostic.publishSample = !m_gapInlet.active && publishSample;
    }
    if (requestedSample && !publishSample &&
        m_gapServiceAgeOnlyCalls != (std::numeric_limits<std::uint32_t>::max)())
    {
        ++m_gapServiceAgeOnlyCalls;
        diagnostic.ageOnlyCalls = m_gapServiceAgeOnlyCalls;
    }
    // Once loaded or consumer-accepted, this source can never regain pending
    // admission. Its first-sample origin is written once, not by heartbeat.
    if (!m_gapAdmission.closed && (publishSample || HasGapPathSourceConsumerAcceptedSameThread() ||
        (m_gapSignal.active && m_gapAdmission.progressSeen)))
    {
        m_gapAdmission.closed = true;
        m_gapAdmission.sampleClockStarted = true;
        m_gapAdmission.sampleOriginMs = nowMs;
    }
    if (pendingAdmission != nullptr)
    {
        const bool firstPending = !m_gapAdmission.seen;
        if (m_gapAdmission.progressSeen || publishSample)
        {
            RejectGapPathSimulationSameThread("ADMISSION_AFTER_SOURCE_PROGRESS", pendingAdmission);
            return false;
        }
        if (m_gapAdmission.closed)
        {
            // ACCEPTED/STARTED feedback is published before the replacement
            // Path snapshot. Latch that one obsolete pending publication once;
            // repeated reads only age the first-sample clock, never heartbeat.
            if ((!m_gapAdmission.obsoleteSeen && m_gapAdmission.seen &&
                !((pendingAdmission->publicationSequence == m_gapAdmission.publication &&
                    pendingAdmission->admissionWaitTick == m_gapAdmission.tick) ||
                    (pendingAdmission->publicationSequence > m_gapAdmission.publication &&
                        pendingAdmission->admissionWaitTick > m_gapAdmission.tick))) ||
                (m_gapAdmission.obsoleteSeen &&
                    (pendingAdmission->publicationSequence != m_gapAdmission.obsoletePublication ||
                        pendingAdmission->admissionWaitTick != m_gapAdmission.obsoleteTick)))
            {
                RejectGapPathSimulationSameThread("ADMISSION_AFTER_ACCEPTED_PUBLICATION", pendingAdmission);
                return false;
            }
            if (!m_gapAdmission.obsoleteSeen)
            {
                m_gapAdmission.obsoletePublication = pendingAdmission->publicationSequence;
                m_gapAdmission.obsoleteTick = pendingAdmission->admissionWaitTick;
                m_gapAdmission.generation = pendingAdmission->requestGeneration;
                m_gapAdmission.obsoleteSeen = true;
                m_gapAdmission.seen = true;
            }
        }
        else
        {
            const bool first = !m_gapAdmission.seen;
            const bool fresh = first || (pendingAdmission->publicationSequence > m_gapAdmission.publication &&
                pendingAdmission->admissionWaitTick > m_gapAdmission.tick);
            if (!first && (!fresh && (pendingAdmission->publicationSequence != m_gapAdmission.publication ||
                pendingAdmission->admissionWaitTick != m_gapAdmission.tick)))
            {
                RejectGapPathSimulationSameThread("ADMISSION_HEARTBEAT_ORDER", pendingAdmission);
                return false;
            }
            if (fresh)
            {
                m_gapAdmission.publication = pendingAdmission->publicationSequence;
                m_gapAdmission.tick = pendingAdmission->admissionWaitTick;
                m_gapAdmission.generation = pendingAdmission->requestGeneration;
                m_gapAdmission.heartbeatMs = nowMs;
                m_gapAdmission.seen = true;
            }
        }
        m_gapAdmission.blockedAxis = pendingAdmission->admissionWaitAxis;
        m_gapAdmission.followingError = pendingAdmission->admissionFollowingError;
        m_gapAdmission.windowPulse = pendingAdmission->admissionWindowPulse;
        if (firstPending) LogGapPathAdmissionSameThread("WAIT_ENTER", nowMs);
    }
    if (publishSample) m_gapAdmission.progressSeen = true;
    m_gapPath.clockStarted = true;
    m_gapPath.lastTicks = ticks;
    m_gapPath.lastServiceMs = nowMs;
    // CZ acquisition uses an actual 20 ms clock, including a repeated validated
    // Motion publication. Consumption still needs a new Motion observation.
    // Unavailable/admission-only publications never grant acquisition scope.
    if (m_gapInlet.active)
    {
        if (!sampledSourceReady || sourcePublication == nullptr) return true;
        bool acquire = false;
        if (!PrepareGapPathSampleAcquisitionSameThread(*sourcePublication, nowMs, acquire)) return false;
        if (!acquire)
        {
            bool accepted = false;
            return ConsumeGapPathSampleSameThread(*sourcePublication, nowMs, publishSample, accepted);
        }
    }
    else if (!publishSample) return true;
    if (m_gapTail.active)
    {
        if (!std::isfinite(activeS) || activeS < 0.0 ||
            m_gapPath.sequence == (std::numeric_limits<std::uint64_t>::max)())
        {
            RejectGapPathSimulationSameThread("TAIL_PROGRESS_OR_SEQUENCE", pendingAdmission);
            return false;
        }
        EDMGap::Sample sample{};
        sample.source = EDMGap::Source::SIMULATED;
        sample.valid = true;
        sample.voltageMv = static_cast<std::int32_t>(m_gapTail.voltage) * 1000;
        if (m_gapPending.active && (sourcePublication == nullptr ||
            !PrepareGapPathPendingLowSampleSameThread(*sourcePublication, nowMs, sample.voltageMv))) return false;
        if (m_gapInlet.active)
        {
            bool accepted = false;
            if (!AcquireGapPathSampleSameThread(*sourcePublication, nowMs, sample.voltageMv) ||
                !ConsumeGapPathSampleSameThread(*sourcePublication, nowMs, publishSample, accepted)) return false;
            if (!accepted) return true;
        }
        else
        {
            sample.sequence = ++m_gapPath.sequence;
            sample.sampledAtMs = nowMs;
            m_gapInput.Publish(sample, nowMs);
        }
        const auto& gap = m_gapInput.Current();
        m_gapServiceAgeOnlyCalls = 0U;
        if (gap.quality != EDMGap::Quality::VALID || gap.source != EDMGap::Source::SIMULATED)
        {
            RejectGapPathSimulationSameThread("TAIL_INPUT_NOT_VALID", pendingAdmission);
            return false;
        }

        if (m_gapSignal.active && !m_gapSignal.sourceSampleSeen)
        {
            m_gapSignal.sourceSampleSeen = true;
            LogGapPathSignalSessionSameThread("SOURCE_SAMPLE_READY");
        }
        if (m_gapPending.active && !ObserveGapPathPendingLowSampleSameThread(*sourcePublication)) return false;
        return true;
    }
    if (!std::isfinite(activeS) || activeS < 0.0 ||
        !std::isfinite(m_pathHold.automaticNextS) ||
        (m_pathHold.automaticNextS <= 0.0 && !IsGapPathInitialStationAtSeamSameThread()) ||
        m_gapPath.sequence == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapPathSimulationSameThread("PROGRESS_OR_SEQUENCE", pendingAdmission);
        return false;
    }
    bool lowEvent = false, recoveryEvent = false;
    if (!m_gapWindow.normalSource && !m_gapPath.held && !m_gapPath.lowInjected &&
        activeS >= m_pathHold.automaticNextS &&
        (m_pathHold.automaticNextS > 0.0 || activeS > 0.0))
    {
        m_gapPath.lowInjected = true;
        lowEvent = true;
    }
    // CN/CO inject once per return; CP re-arms only after an applied probe resume.
    // The clock is independent of local activeS and the earlier endpoint recovery.
    bool returnLowEvent = false;
    if (m_gapPath.returnLowTest && returningSample && !m_gapPath.returnLowInjected &&
        (!m_gapPath.repeatedReturnLow || m_gapPath.returnProbeResumeCount < m_gapPath.returnProbeLimit))
    {
        if (!m_gapPath.returnWatchStarted)
        {
            m_gapPath.returnWatchStarted = true;
            m_gapPath.returnWatchStartMs = nowMs;
            LogGapPathSimulationSameThread("RETURN_WATCH_2S");
        }
        if (nowMs - m_gapPath.returnWatchStartMs >= 2000ULL)
        {
            m_gapPath.returnLowInjected = true;
            m_gapPath.recoveryInjected = false;
            m_gapPath.recoveryLogged = false;
            returnLowEvent = true;
        }
    }
    // CM_TEST1: give the operator a visible second-round test window.
    const bool secondRoundTestWindow = m_gapPath.repeatedLowRetreat && m_gapPath.returnHold &&
        m_pathHold.automaticObservedReturns == 1ULL && m_pathHold.returnHoldRetreatCount == 2ULL;
    const std::uint64_t recoveryDelayMs = (secondRoundTestWindow || m_gapPath.returnRehold) ? 30000ULL :
        (m_gapPath.automaticResume ? 3000ULL : 1000ULL);
    if (m_gapPath.held && (!m_gapPath.lowRetreat || m_gapPath.returnHold) &&
        (!m_gapPath.returnLowInjected || m_gapPath.returnRehold) && !m_gapPath.recoveryInjected &&
        nowMs >= m_gapPath.holdStartMs &&
        nowMs - m_gapPath.holdStartMs >= recoveryDelayMs)
    {
        m_gapPath.recoveryInjected = true;
        recoveryEvent = true;
    }
    EDMGap::Sample sample{};
    sample.source = EDMGap::Source::SIMULATED;
    sample.valid = true;
    sample.voltageMv = m_gapPath.lowInjected && !m_gapPath.recoveryInjected ? 20000 : 50000;
    if (m_gapPending.active && (sourcePublication == nullptr ||
        !PrepareGapPathPendingLowSampleSameThread(*sourcePublication, nowMs, sample.voltageMv))) return false;
    bool accepted = true;
    if (m_gapInlet.active)
    {
        if (!AcquireGapPathSampleSameThread(*sourcePublication, nowMs, sample.voltageMv) ||
            !ConsumeGapPathSampleSameThread(*sourcePublication, nowMs, publishSample, accepted)) return false;
        // Policy events denote real acquisitions even if Motion has not yet
        // provided a newer observation to accept that immutable frame.
        if (lowEvent) LogGapPathSimulationSameThread("INJECT_LOW");
        if (returnLowEvent) LogGapPathSimulationSameThread("INJECT_RETURN_LOW");
        if (recoveryEvent) LogGapPathSimulationSameThread("INJECT_RECOVERY");
        lowEvent = recoveryEvent = returnLowEvent = false;
        if (!accepted) return true;
    }
    else
    {
        sample.sequence = ++m_gapPath.sequence;
        sample.sampledAtMs = nowMs;
        m_gapInput.Publish(sample, nowMs);
    }
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    if (gap.quality != EDMGap::Quality::VALID || gap.source != EDMGap::Source::SIMULATED)
    {
        RejectGapPathSimulationSameThread("INPUT_NOT_VALID", pendingAdmission);
        return false;
    }
    m_gapServiceAgeOnlyCalls = 0U;
    if (m_gapSignal.active && !m_gapSignal.sourceSampleSeen)
    {
        m_gapSignal.sourceSampleSeen = true;
        LogGapPathSignalSessionSameThread("SOURCE_SAMPLE_READY");
    }
    if (m_gapPending.active && !ObserveGapPathPendingLowSampleSameThread(*sourcePublication)) return false;
    if (m_gapAdmission.seen && m_gapAdmission.progressSeen && !m_gapAdmission.readyLogged)
    {
        m_gapAdmission.readyLogged = true;
        LogGapPathAdmissionSameThread("SOURCE_READY", nowMs);
    }
    if (lowEvent) LogGapPathSimulationSameThread("INJECT_LOW");
    if (returnLowEvent) LogGapPathSimulationSameThread("INJECT_RETURN_LOW");
    if (recoveryEvent) LogGapPathSimulationSameThread("INJECT_RECOVERY");
    if (!m_gapPath.lowInjected && gap.band == EDMGap::Band::NORMAL && !m_gapPath.normalLogged)
    {
        m_gapPath.normalLogged = true;
        LogGapPathSimulationSameThread("NORMAL_CONFIRMED");
    }
    if (m_gapPath.recoveryInjected && gap.band == EDMGap::Band::NORMAL && !m_gapPath.recoveryLogged)
    {
        m_gapPath.recoveryLogged = true;
        LogGapPathSimulationSameThread(m_gapPath.automaticResume ?
            "RECOVERED_NORMAL" : "RECOVERED_WAIT_START");
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathAutomaticNormalSameThread() const noexcept
{
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    return m_gapPath.active && m_gapPath.automaticResume && m_gapPath.held &&
        m_gapPath.lowInjected && m_gapPath.recoveryInjected && gap.configured &&
        gap.source == EDMGap::Source::SIMULATED && gap.quality == EDMGap::Quality::VALID &&
        gap.band == EDMGap::Band::NORMAL && gap.pendingBand == EDMGap::Band::UNKNOWN &&
        m_gapPath.sequence != 0ULL && gap.sequence == m_gapPath.sequence &&
        gap.observedAtMs == m_gapPath.lastServiceMs && gap.sampledAtMs <= m_gapPath.lastServiceMs &&
        m_gapPath.lastServiceMs - gap.sampledAtMs <= 100ULL;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathAutomaticResumeSignalSameThread() const noexcept
{
    if (!m_gapPath.lowRetreat || m_gapPath.returnHold)
        return IsGapPathAutomaticNormalSameThread();
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    return m_gapPath.active && m_gapPath.automaticResume && m_gapPath.held &&
        m_gapPath.lowInjected && !m_gapPath.recoveryInjected && gap.configured &&
        gap.source == EDMGap::Source::SIMULATED && gap.quality == EDMGap::Quality::VALID &&
        gap.band == EDMGap::Band::LOW && gap.pendingBand == EDMGap::Band::UNKNOWN &&
        m_gapPath.sequence != 0ULL && gap.sequence == m_gapPath.sequence &&
        gap.observedAtMs == m_gapPath.lastServiceMs && gap.sampledAtMs <= m_gapPath.lastServiceMs &&
        m_gapPath.lastServiceMs - gap.sampledAtMs <= 100ULL;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathAutomaticResumeSameThread() noexcept
{
    if (!m_gapPath.active || !m_gapPath.automaticResume || !m_pathHold.automaticHoldOwned) return true;
    // Admission can be delayed between the service, Prepare and Commit. Poll
    // the actual clock without publishing or qualifying a replacement sample.
    if (!ServiceGapPathSimulationSameThread(0.0, false, "ADMISSION")) return false;
    if (m_gapPath.returnLowTest && m_gapPath.returnRehold)
    {
        const auto snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
        // A bounded-read collision before admission commit grants no action.
        // Keep HOLD and retry with the existing sample age; after provisional
        // RUN, the final commit check must fail closed instead.
        if (snapshot.publicationSequence == 0ULL && m_state == NCState::HOLD) return false;
        if (!ValidateGapPathReturnLowSnapshotSameThread(snapshot)) return false;
    }
    if (!m_pathHold.automaticAdmissionOwned || !IsGapPathAutomaticResumeSignalSameThread())
    {
        RejectGapPathSimulationSameThread(m_gapPath.lowRetreat ?
            "CL_SIGNAL_NOT_QUALIFIED_AT_ADMISSION" : "RECOVERY_NOT_NORMAL_AT_ADMISSION");
        return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSimulationSameThread(const char* phase) const noexcept
{
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    if (m_gapPath.repeatedReturnLow)
    {
        if (m_gapWindow.active)
        {
            RtPrintf("[%s] phase=%s source=%s quality=%s band=%s mv=%d seq=%llu ms=%llu run=%llu dispatch=%llu line=%d owner=%u generation=%llu epoch=%llu hold=%llu owned=%u ack=%u low=%u recovered=%u resume=%s completed=%llu limit=%u nextS=%016llX probe=%u probeLimit=%u probeHolds=%u probeResumed=%u returnHold=%llu latestStop=%llu sourceIndex=%u sourceLimit=%u motion=1 discharge=0\n",
                "GAP-CQ", phase, EDMGap::SourceName(gap.source), EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band),
                static_cast<int>(gap.voltageMv), static_cast<unsigned long long>(gap.sequence),
                static_cast<unsigned long long>(m_gapPath.lastServiceMs), static_cast<unsigned long long>(m_pathHold.run),
                static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
                static_cast<unsigned int>(m_pathHold.lease.owner), static_cast<unsigned long long>(m_pathHold.lease.generation),
                static_cast<unsigned long long>(m_pathHold.identity.epoch),
                static_cast<unsigned long long>(m_pathHold.automaticSettleSequence),
                m_pathHold.automaticHoldOwned ? 1U : 0U, m_gapPath.ackLogged ? 1U : 0U,
                m_gapPath.lowInjected ? 1U : 0U, m_gapPath.recoveryLogged ? 1U : 0U,
                m_gapPath.lowRetreat ? (m_gapPath.returnHold ? "NORMAL_RETURN" : "LOW_RETREAT") :
                (m_gapPath.automaticResume ? "AUTO_NORMAL" : "OPERATOR"),
                static_cast<unsigned long long>(m_pathHold.automaticObservedReturns),
                static_cast<unsigned int>(m_pathHold.cycleLimit),
                static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)),
                static_cast<unsigned int>(m_gapPath.returnRehold ? m_gapPath.returnProbeHoldCount :
                    (m_gapPath.returnProbeResumeCount < m_gapPath.returnProbeLimit ? m_gapPath.returnProbeResumeCount + 1U : m_gapPath.returnProbeLimit)),
                static_cast<unsigned int>(m_gapPath.returnProbeLimit),
                static_cast<unsigned int>(m_gapPath.returnProbeHoldCount),
                static_cast<unsigned int>(m_gapPath.returnProbeResumeCount),
                static_cast<unsigned long long>(m_pathHold.requestedHoldSequence),
                static_cast<unsigned long long>(m_gapPath.returnReholdSequence),
                static_cast<unsigned int>(m_gapWindow.sourceIndex), static_cast<unsigned int>(m_gapWindow.sourceLimit));
            return;
        }
        RtPrintf("[%s] phase=%s source=%s quality=%s band=%s mv=%d seq=%llu ms=%llu run=%llu dispatch=%llu line=%d owner=%u generation=%llu epoch=%llu hold=%llu owned=%u ack=%u low=%u recovered=%u resume=%s completed=%llu limit=%u nextS=%016llX probe=%u probeLimit=%u probeHolds=%u probeResumed=%u returnHold=%llu latestStop=%llu motion=1 discharge=0\n",
            "GAP-CP", phase, EDMGap::SourceName(gap.source), EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band),
            static_cast<int>(gap.voltageMv), static_cast<unsigned long long>(gap.sequence),
            static_cast<unsigned long long>(m_gapPath.lastServiceMs), static_cast<unsigned long long>(m_pathHold.run),
            static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
            static_cast<unsigned int>(m_pathHold.lease.owner), static_cast<unsigned long long>(m_pathHold.lease.generation),
            static_cast<unsigned long long>(m_pathHold.identity.epoch),
            static_cast<unsigned long long>(m_pathHold.automaticSettleSequence),
            m_pathHold.automaticHoldOwned ? 1U : 0U, m_gapPath.ackLogged ? 1U : 0U,
            m_gapPath.lowInjected ? 1U : 0U, m_gapPath.recoveryLogged ? 1U : 0U,
            m_gapPath.lowRetreat ? (m_gapPath.returnHold ? "NORMAL_RETURN" : "LOW_RETREAT") :
            (m_gapPath.automaticResume ? "AUTO_NORMAL" : "OPERATOR"),
            static_cast<unsigned long long>(m_pathHold.automaticObservedReturns),
            static_cast<unsigned int>(m_pathHold.cycleLimit),
            static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)),
            static_cast<unsigned int>(m_gapPath.returnRehold ? m_gapPath.returnProbeHoldCount :
                (m_gapPath.returnProbeResumeCount < m_gapPath.returnProbeLimit ? m_gapPath.returnProbeResumeCount + 1U : m_gapPath.returnProbeLimit)),
            static_cast<unsigned int>(m_gapPath.returnProbeLimit),
            static_cast<unsigned int>(m_gapPath.returnProbeHoldCount),
            static_cast<unsigned int>(m_gapPath.returnProbeResumeCount),
            static_cast<unsigned long long>(m_pathHold.requestedHoldSequence),
            static_cast<unsigned long long>(m_gapPath.returnReholdSequence));
        return;
    }
    RtPrintf("[%s] phase=%s source=%s quality=%s band=%s mv=%d seq=%llu ms=%llu run=%llu dispatch=%llu line=%d owner=%u generation=%llu epoch=%llu hold=%llu owned=%u ack=%u low=%u recovered=%u resume=%s completed=%llu limit=%u nextS=%016llX motion=1 discharge=0\n",
        m_gapPath.returnLowTest ? (m_gapPath.repeatedLowRetreat ? "GAP-CO" : "GAP-CN") : m_gapPath.lowRetreat ? (m_gapPath.repeatedLowRetreat ? "GAP-CM" : "GAP-CL") : (m_gapPath.repeating ? "GAP-CK" : (m_gapPath.automaticResume ? "GAP-CI" : "GAP-CH")), phase, EDMGap::SourceName(gap.source), EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band),
        static_cast<int>(gap.voltageMv), static_cast<unsigned long long>(gap.sequence),
        static_cast<unsigned long long>(m_gapPath.lastServiceMs), static_cast<unsigned long long>(m_pathHold.run),
        static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
        static_cast<unsigned int>(m_pathHold.lease.owner), static_cast<unsigned long long>(m_pathHold.lease.generation),
        static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_pathHold.automaticSettleSequence),
        m_pathHold.automaticHoldOwned ? 1U : 0U, m_gapPath.ackLogged ? 1U : 0U,
        m_gapPath.lowInjected ? 1U : 0U, m_gapPath.recoveryLogged ? 1U : 0U,
        m_gapPath.lowRetreat ? (m_gapPath.returnHold ? "NORMAL_RETURN" : "LOW_RETREAT") :
        (m_gapPath.automaticResume ? "AUTO_NORMAL" : "OPERATOR"),
        static_cast<unsigned long long>(m_pathHold.automaticObservedReturns),
        static_cast<unsigned int>(m_pathHold.cycleLimit),
        static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathSourceWindowScopeValidSameThread(bool bindingCurrentSource) noexcept
{
    if (m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        // Only the synchronous Commit of an already accepted exact source may
        // enqueue its binding before RT consumes that source's normal epoch.
        // Ordinary observation remains strict, and real safety always revokes.
        if (!bindingCurrentSource || !m_pathHold.armed || m_pathHold.bound ||
            m_pathHold.candidateDispatch == NC_BLOCK_DISPATCH_ID_INVALID ||
            m_pathHold.candidateDispatch <= m_gapWindow.previousDispatch ||
            m_motion.HasPendingSafetyIntent()) return false;
        const bool line = m_pathFeed.pending && m_pathFeed.bound &&
            m_pathFeed.dispatch == m_pathHold.candidateDispatch;
        const bool arc = m_pathArc.pending && m_pathArc.bound &&
            m_pathArc.dispatch == m_pathHold.candidateDispatch;
        if (line == arc) return false;
        const MotionExecutionIdentity& identity = line ? m_pathFeedMotion.receipt.identity : m_pathArcMotion.receipt.identity;
        const MotionOwnerLease& lease = line ? m_pathFeedMotion.receipt.ownerLease : m_pathArcMotion.receipt.ownerLease;
        const int sourcePC = line ? m_pathFeed.sourcePC : m_pathArc.sourcePC;
        const std::uint64_t commit = line ? m_pathFeed.commit : m_pathArc.commit;
        const bool receiptValid = line ?
            (m_pathFeedMotion.receipt.valid && m_pathFeedMotion.receipt.commandAccepted &&
                m_pathFeedMotion.receipt.tailCommitted && m_pathFeedMotion.receipt.captureBound) :
            (m_pathArcMotion.receipt.valid && m_pathArcMotion.receipt.commandAccepted &&
                m_pathArcMotion.receipt.tailCommitted && m_pathArcMotion.receipt.captureBound);
        if (!receiptValid || !identity.IsAssigned() || identity.source != MotionCommandSource::NC_MEMORY ||
            identity.epoch != m_motion.GetCurrentExecutionEpoch() || sourcePC < 0 ||
            identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
            !lease.Matches(m_gapWindow.lease) || !m_motion.IsMotionOwnerLeaseCurrent(lease) ||
            commit == 0ULL || commit <= m_gapWindow.previousCommit) return false;
    }
    if (m_gapWindow.recoverQueuedInput != m_gapRecovery.active ||
        (m_gapRecovery.active && (!m_gapWindow.queuedInput || m_gapWindow.sourceGapMs != 40U ||
            (m_gapWindow.queuedProbeVoltage != 50U && m_gapWindow.queuedProbeVoltage != 20U))) ||
        (!m_gapRecovery.active && m_gapWindow.queuedProbeVoltage != 50U) ||
        m_gapWindow.queuedInput != m_gapQueue.active ||
        (m_gapQueue.active && !m_gapInlet.active) ||
        m_gapWindow.sampledInput != m_gapInlet.active ||
        (m_gapInlet.active && (!m_gapWindow.continuousSignal || m_gapWindow.pendingLowAcrossSources ||
            m_gapWindow.tailVoltage != 50U))) return false;
    if (m_gapWindow.pendingLowAcrossSources != m_gapPending.active ||
        (m_gapPending.active && (!m_gapWindow.continuousSignal || m_gapWindow.sourceGapMs != 0U ||
            m_gapWindow.tailVoltage != 50U ||
            (m_gapWindow.pendingLowVoltage != 50U && m_gapWindow.pendingLowVoltage != 20U) ||
            m_gapPending.run != m_gapWindow.run || m_gapPending.cache != m_gapWindow.cache ||
            (m_gapPending.injected && (m_gapPending.firstSequence == 0ULL ||
                m_gapPending.originSourceIndex == 0U ||
                m_gapPending.targetSourceIndex != m_gapPending.originSourceIndex + 1U ||
                m_gapPending.targetSourceIndex > m_gapWindow.sourceLimit ||
                m_gapWindow.sourceIndex < m_gapPending.originSourceIndex ||
                (!m_gapPending.carried && m_gapWindow.sourceIndex != m_gapPending.originSourceIndex) ||
                (m_gapPending.carried && m_gapWindow.sourceIndex < m_gapPending.targetSourceIndex) ||
                (!m_gapPending.resolved && m_gapWindow.sourceIndex > m_gapPending.targetSourceIndex) ||
                (m_gapPending.resolved && (!m_gapPending.carried || m_gapWindow.pendingLowVoltage != 50U)))) ||
            (!m_gapPending.injected && (m_gapPending.carried || m_gapPending.resolved ||
                m_gapPending.originSourceIndex != 0U || m_gapPending.targetSourceIndex != 0U)))) ||
        (!m_gapPending.active && m_gapWindow.pendingLowVoltage != 50U)) return false;
    if (m_gapWindow.continuousSignal != m_gapSignal.active ||
        (m_gapWindow.continuousSignal && (!m_gapWindow.tailSupervision ||
            m_gapSignal.run != m_gapWindow.run || m_gapSignal.cache != m_gapWindow.cache ||
            m_gapSignal.sourceIndex != m_gapWindow.sourceIndex ||
            (m_gapWindow.sourceGapMs != 0U && m_gapWindow.sourceGapMs != 150U &&
                !(m_gapRecovery.active && m_gapWindow.sourceGapMs == 40U)) ||
            (m_gapWindow.sourceGapMs == 150U && m_gapWindow.tailVoltage != 50U))) ||
        (!m_gapWindow.continuousSignal && m_gapWindow.sourceGapMs != 0U)) return false;
    if ((m_gapWindow.tailSupervision && (!m_gapWindow.allowSeamStations ||
        (m_gapWindow.tailVoltage != 50U && m_gapWindow.tailVoltage != 20U))) ||
        (!m_gapWindow.tailSupervision && m_gapWindow.tailVoltage != 50U)) return false;
    if (m_gapWindow.allowSeamStations && (!m_gapWindow.multipleStationsPerSource ||
        !m_gapWindow.repeatedCumulativeStation || !m_gapWindow.cumulativeStation ||
        !m_gapWindow.allowNormalSources)) return false;
    if (m_gapWindow.seamPublication != 0ULL && (!m_gapWindow.allowSeamStations ||
        !m_pathHold.bound || m_gapWindow.normalSource)) return false;
    // Recompute at most seven retained original lengths and CT station crossings.
    // Neither a freely changed counter nor retreat/return distance grants a station.
    if (m_gapWindow.cumulativeStation)
    {
        if (!m_gapWindow.allowNormalSources ||
            (!m_gapWindow.multipleStationsPerSource && m_gapWindow.cycleLimit != 1U) ||
            m_gapWindow.sourceIndex < 1U || m_gapWindow.sourceIndex > 8U ||
            m_gapWindow.initialHistoryCount > NCPathCoreRetainedPath::Capacity - (m_gapWindow.sourceIndex - 1U) ||
            !std::isfinite(m_gapWindow.completedForwardMM) || m_gapWindow.completedForwardMM < 0.0 ||
            !std::isfinite(m_gapWindow.intervalMM) || m_gapWindow.intervalMM <= 0.0) return false;
        if (m_gapWindow.repeatedCumulativeStation)
        {
            if (m_gapWindow.stationLimit < 1U ||
                (!m_gapWindow.multipleStationsPerSource && m_gapWindow.stationLimit > m_gapWindow.sourceLimit) ||
                m_gapWindow.stationLimit > 8U || m_gapWindow.completedStations > m_gapWindow.stationLimit ||
                (!m_gapWindow.multipleStationsPerSource && m_gapWindow.completedStations >= m_gapWindow.sourceIndex) ||
                !std::isfinite(m_gapWindow.intervalMM * static_cast<double>(m_gapWindow.stationLimit)) ||
                m_gapWindow.stationConsumed != (m_gapWindow.completedStations == m_gapWindow.stationLimit) ||
                (m_gapWindow.stationConsumed && (m_gapWindow.budgetProven ||
                    (m_pathHold.bound && !m_gapWindow.normalSource)))) return false;
        }
        else if (m_gapWindow.multipleStationsPerSource || m_gapWindow.stationLimit != 0U || m_gapWindow.completedStations != 0U ||
            (m_gapWindow.stationConsumed && (m_gapWindow.budgetProven ||
                m_gapWindow.sourceIndex == 1U || m_gapWindow.completedForwardMM <= m_gapWindow.intervalMM ||
                (m_pathHold.bound && !m_gapWindow.normalSource))) ||
            (!m_gapWindow.stationConsumed && m_gapWindow.completedForwardMM >= m_gapWindow.intervalMM)) return false;
        double completedMM = 0.0;
        std::uint32_t stations = 0U;
        for (std::uint32_t i = 0U; i + 1U < m_gapWindow.sourceIndex; ++i)
        {
            const NCPathCoreRetainedGeometry* geometry = m_pathReplayStore.Get(m_gapWindow.initialHistoryCount + i);
            if (geometry == nullptr || !geometry->valid || geometry->point ||
                !std::isfinite(geometry->lengthMM) || geometry->lengthMM <= 0.0 ||
                !std::isfinite(geometry->lengthPulse) || geometry->lengthPulse <= 0.0) return false;
            const double nextMM = completedMM + geometry->lengthMM;
            if (!std::isfinite(nextMM) || nextMM <= completedMM) return false;
            if (m_gapWindow.multipleStationsPerSource)
            {
                std::uint32_t planned = 0U;
                if (!CountGapPathSourceStationsSameThread(completedMM, geometry->lengthMM,
                    geometry->lengthPulse, stations, planned)) return false;
                stations += planned;
            }
            else if (m_gapWindow.repeatedCumulativeStation && stations < m_gapWindow.stationLimit)
            {
                const double targetMM = m_gapWindow.intervalMM * static_cast<double>(stations + 1U);
                const int relation = HoldStationSourceRelation(completedMM,
                    geometry->lengthMM, geometry->lengthPulse, targetMM);
                if (relation == 2) return false;
                if (relation == 1)
                {
                    if (!IsGapPathNextStationBeyondSourceSameThread(completedMM,
                        geometry->lengthMM, geometry->lengthPulse, stations)) return false;
                    ++stations;
                }
            }
            completedMM = nextMM;
        }
        if (completedMM != m_gapWindow.completedForwardMM ||
            (m_gapWindow.repeatedCumulativeStation && stations != m_gapWindow.completedStations)) return false;
        if (m_gapWindow.multipleStationsPerSource)
        {
            if (!m_pathHold.bound)
            {
                if (m_gapWindow.cycleLimit != 1U || m_pathHold.cycleLimit != 1U ||
                    m_gapWindow.normalSource || m_gapWindow.budgetProven ||
                    m_pathHold.automaticObservedReturns != 0ULL) return false;
            }
            else
            {
                std::uint32_t planned = 0U;
                if (!m_pathHoldView.original.valid || m_pathHoldView.original.point ||
                    !CountGapPathSourceStationsSameThread(completedMM, m_pathHoldView.original.lengthMM,
                        m_pathHoldView.original.lengthPulse, stations, planned) ||
                    m_gapWindow.cycleLimit != (planned == 0U ? 1U : planned) ||
                    m_gapWindow.normalSource != (planned == 0U) ||
                    m_pathHold.automaticObservedReturns > planned) return false;
            }
        }
    }
    else if (m_gapWindow.stationConsumed || m_gapWindow.completedForwardMM != 0.0 ||
        m_gapWindow.repeatedCumulativeStation || m_gapWindow.multipleStationsPerSource ||
        m_gapWindow.stationLimit != 0U || m_gapWindow.completedStations != 0U) return false;
    if ((!m_gapWindow.allowNormalSources &&
        (m_gapWindow.normalSource || m_gapWindow.normalProven || m_gapWindow.normalPublication != 0ULL ||
            m_gapWindow.normalGeneration != 0ULL)) ||
        (m_gapWindow.normalProven && !m_gapWindow.normalSource) ||
        (m_gapWindow.normalSource && m_gapWindow.budgetProven)) return false;
    if (!m_gapWindow.active || m_gapWindow.sourceLimit < 2U || m_gapWindow.sourceLimit > 8U ||
        m_gapWindow.sourceIndex < 1U || m_gapWindow.sourceIndex > m_gapWindow.sourceLimit ||
        m_gapWindow.cycleLimit < 1U || m_gapWindow.cycleLimit > 32U ||
        m_gapWindow.probeLimit < 1U || m_gapWindow.probeLimit > 8U ||
        m_gapWindow.initialHistoryCount == 0U ||
        m_gapWindow.initialHistoryCount > NCPathCoreRetainedPath::Capacity - m_gapWindow.sourceLimit ||
        m_gapWindow.run == 0ULL || m_gapWindow.cache == 0ULL || !m_gapWindow.lease.IsValid() ||
        m_gapWindow.previousDispatch == 0ULL ||
        !std::isfinite(m_gapWindow.distanceMM) || m_gapWindow.distanceMM <= 0.0 ||
        !std::isfinite(m_gapWindow.feedMMMin) || m_gapWindow.feedMMMin <= 0.0 || m_gapWindow.feedMMMin > 100.0 ||
        !std::isfinite(m_gapWindow.intervalMM) || m_gapWindow.intervalMM <= 0.0 ||
        Close_System_Com_flag || AlarmManager::GetInstance().HasAlarm() ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) || m_mode != NCOperationMode::MEMORY ||
        Homing.IsActive() || m_isG66Active || !m_macroStack.empty() || m_isSingleBlockEnabled ||
        m_legacySingleBlockPausePending || !m_feedHoldResumeGate.IsEnabled() ||
        m_pathHold.blocked || !m_pathHold.crossSegment ||
        !m_pathHold.requireReturnAuthorization || m_pathReplay.pending || !m_pathReplay.armed ||
        m_pathReplayStore.Pending() || m_pathReplayStore.State() != NCPathCoreRetainedCursorState::IDLE ||
        m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE ||
        m_pathReplayStore.Count() > NCPathCoreRetainedPath::Capacity ||
        m_gapWindow.run != m_pathCoreLiveBookkeeping.currentRunToken || m_pathHold.run != m_gapWindow.run ||
        m_pathReplay.run != m_gapWindow.run || m_gapWindow.cache != GetBaseProgramCache().GetGeneration() ||
        m_pathHold.cache != m_gapWindow.cache || m_pathReplay.cache != m_gapWindow.cache ||
        !m_gapWindow.lease.Matches(m_programMotionLease) || !m_gapWindow.lease.Matches(m_pathHold.lease) ||
        !m_gapWindow.lease.Matches(m_pathReplayLease) || !m_motion.IsMotionOwnerLeaseCurrent(m_gapWindow.lease) ||
        !IsPathCoreReplayConfigurationValid() ||
        m_pathHold.distanceMM != m_gapWindow.distanceMM || m_pathHold.feedMMMin != m_gapWindow.feedMMMin ||
        m_pathHold.cycleLimit != m_gapWindow.cycleLimit ||
        (!m_gapWindow.budgetProven && m_pathHold.automaticIntervalMM != m_gapWindow.intervalMM)) return false;
    const NCBlockLifecycleCounters counters = m_blockLifecycleLedger.GetCounters();
    return m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL && counters.activeBlockOverwrite == 0ULL &&
        counters.activeSegmentIndexOverwrite == 0ULL && counters.orphanFeedback == 0ULL &&
        counters.duplicateTerminalFeedback == 0ULL && counters.terminalFeedbackConflict == 0ULL;
}

// CV keeps old selectors strict. Only three agreeing exact endpoint forms may defer.
NC_PATH_HOLD_NOINLINE
int NCManager::ClassifyGapPathStationSameThread(double prefixMM, double lengthMM,
    double lengthPulse, double targetMM) const noexcept
{
    if (!m_gapWindow.allowSeamStations)
        return HoldStationSourceRelation(prefixMM, lengthMM, lengthPulse, targetMM);
    if (!std::isfinite(prefixMM) || prefixMM < 0.0 || !std::isfinite(lengthMM) || lengthMM <= 0.0 ||
        !std::isfinite(lengthPulse) || lengthPulse <= 0.0 || !std::isfinite(targetMM) ||
        targetMM <= 0.0 || targetMM < prefixMM) return 2;
    const double endMM = prefixMM + lengthMM;
    const double scale = lengthPulse / lengthMM;
    const double localMM = targetMM - prefixMM;
    const double localPulse = localMM * scale;
    if (!std::isfinite(endMM) || endMM <= prefixMM || !std::isfinite(scale) || scale <= 0.0 ||
        !std::isfinite(localMM) || !std::isfinite(localPulse)) return 2;
    if (targetMM == prefixMM && localMM == 0.0 && localPulse == 0.0) return 1;
    if (localMM <= 0.0 || localPulse <= 0.0) return 2;
    if (endMM == targetMM && lengthMM == localMM && lengthPulse == localPulse) return 0;
    if (endMM < targetMM && lengthMM < localMM && lengthPulse < localPulse) return 0;
    if (endMM > targetMM && lengthMM > localMM && lengthPulse > localPulse) return 1;
    return 2;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathInitialStationAtSeamSameThread() noexcept
{
    // This remains true during the first excursion, until its return is consumed.
    return m_gapWindow.active && m_gapWindow.allowSeamStations &&
        m_gapWindow.multipleStationsPerSource && m_gapWindow.repeatedCumulativeStation &&
        m_gapWindow.cumulativeStation && !m_gapWindow.normalSource && !m_gapWindow.budgetProven &&
        !m_gapWindow.stationConsumed && m_gapWindow.sourceIndex > 1U &&
        m_gapWindow.completedStations < m_gapWindow.stationLimit&&
        std::isfinite(m_gapWindow.completedForwardMM) && m_gapWindow.completedForwardMM > 0.0 &&
        GetGapPathWindowStationMMSameThread() == m_gapWindow.completedForwardMM &&
        m_pathHold.bound && m_pathHold.automaticEnabled &&
        m_pathHold.automaticObservedReturns == 0ULL &&
        m_pathHold.automaticNextS == 0.0 && m_pathHold.automaticIntervalPulse == 0.0 &&
        m_pathHoldView.original.valid && !m_pathHoldView.original.point &&
        ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM,
            m_pathHoldView.original.lengthMM, m_pathHoldView.original.lengthPulse,
            GetGapPathWindowStationMMSameThread()) == 1 &&
        IsGapPathSourceWindowScopeValidSameThread();
}

NC_PATH_HOLD_NOINLINE
double NCManager::GetGapPathWindowStationMMSameThread() const noexcept
{
    return m_gapWindow.repeatedCumulativeStation && !m_gapWindow.stationConsumed ?
        m_gapWindow.intervalMM * static_cast<double>(m_gapWindow.completedStations + 1U) : m_gapWindow.intervalMM;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CountGapPathSourceStationsSameThread(double prefixMM, double lengthMM, double lengthPulse,
    std::uint32_t completedStations, std::uint32_t& planned) const noexcept
{
    planned = 0U;
    if (!m_gapWindow.multipleStationsPerSource || !m_gapWindow.repeatedCumulativeStation ||
        !m_gapWindow.cumulativeStation || m_gapWindow.stationLimit < 1U || m_gapWindow.stationLimit > 8U ||
        completedStations > m_gapWindow.stationLimit ||
        !std::isfinite(m_gapWindow.intervalMM * static_cast<double>(m_gapWindow.stationLimit)) ||
        !std::isfinite(prefixMM) || prefixMM < 0.0 ||
        !std::isfinite(lengthMM) || lengthMM <= 0.0 || !std::isfinite(lengthPulse) || lengthPulse <= 0.0 ||
        !std::isfinite(prefixMM + lengthMM) || prefixMM + lengthMM <= prefixMM ||
        !std::isfinite(m_gapWindow.intervalMM) || m_gapWindow.intervalMM <= 0.0) return false;
    double previousTargetMM = m_gapWindow.intervalMM * static_cast<double>(completedStations);
    double previousLocalMM = 0.0, previousLocalPulse = 0.0;
    bool beyond = false;
    // At most eight fixed nQ targets, including those beyond this source.
    for (std::uint32_t i = completedStations; i < m_gapWindow.stationLimit; ++i)
    {
        const double targetMM = m_gapWindow.intervalMM * static_cast<double>(i + 1U);
        const double localMM = targetMM - prefixMM;
        const double localPulse = localMM * (lengthPulse / lengthMM);
        const int relation = ClassifyGapPathStationSameThread(prefixMM, lengthMM, lengthPulse, targetMM);
        const bool firstAtStart = m_gapWindow.allowSeamStations && i == completedStations &&
            prefixMM > 0.0 && targetMM == prefixMM && localMM == 0.0 && localPulse == 0.0;
        if (relation == 2 || !std::isfinite(previousTargetMM) || targetMM <= previousTargetMM ||
            (!firstAtStart && (localMM <= previousLocalMM || localPulse <= previousLocalPulse)) ||
            (beyond && relation == 1)) return false;
        if (relation == 1) ++planned;
        else beyond = true;
        previousTargetMM = targetMM;
        previousLocalMM = localMM;
        previousLocalPulse = localPulse;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathNextStationBeyondSourceSameThread(double prefixMM, double lengthMM,
    double lengthPulse, std::uint32_t completedStations) const noexcept
{
    if (completedStations + 1U >= m_gapWindow.stationLimit) return true;
    const double nextTargetMM = m_gapWindow.intervalMM * static_cast<double>(completedStations + 2U);
    return HoldStationSourceRelation(prefixMM, lengthMM, lengthPulse, nextTargetMM) == 0;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSourceWindowSameThread(const char* phase, const char* reason) const noexcept
{
    RtPrintf("[%s] phase=%s reason=%s sourceIndex=%u sourceLimit=%u run=%llu cache=%llu dispatch=%llu commit=%llu epoch=%llu segment=%llu sourceBlock=%llu previousDispatch=%llu previousCommit=%llu previousEpoch=%llu previousSegment=%llu history=%u budget=%u fence=%llu request=%llu latestStop=%llu returnedS=%016llX R=%u L=%u motion=1 discharge=0\n",
        m_gapWindow.tailSupervision ? "GAP-CW" : m_gapWindow.allowSeamStations ? "GAP-CV" : m_gapWindow.multipleStationsPerSource ? "GAP-CU" : m_gapWindow.repeatedCumulativeStation ? "GAP-CT" : m_gapWindow.cumulativeStation ? "GAP-CS" : (m_gapWindow.allowNormalSources ? "GAP-CR" : "GAP-CQ"), phase, reason, static_cast<unsigned int>(m_gapWindow.sourceIndex), static_cast<unsigned int>(m_gapWindow.sourceLimit),
        static_cast<unsigned long long>(m_gapWindow.run), static_cast<unsigned long long>(m_gapWindow.cache),
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.commit),
        static_cast<unsigned long long>(m_pathHold.identity.epoch), static_cast<unsigned long long>(m_pathHold.identity.segmentId),
        static_cast<unsigned long long>(m_pathHold.identity.sourceBlockId),
        static_cast<unsigned long long>(m_gapWindow.previousDispatch), static_cast<unsigned long long>(m_gapWindow.previousCommit),
        static_cast<unsigned long long>(m_gapWindow.previousIdentity.epoch),
        static_cast<unsigned long long>(m_gapWindow.previousIdentity.segmentId),
        static_cast<unsigned int>(m_pathReplayStore.Count()), m_gapWindow.budgetProven ? 1U : 0U,
        static_cast<unsigned long long>(m_gapWindow.budgetFence), static_cast<unsigned long long>(m_gapWindow.budgetRequest),
        static_cast<unsigned long long>(m_gapWindow.budgetLatestStop), static_cast<unsigned long long>(m_gapWindow.budgetReturnedSBits),
        static_cast<unsigned int>(m_gapWindow.probeLimit), static_cast<unsigned int>(m_gapWindow.repeatedCumulativeStation ?
            m_gapWindow.stationLimit : m_gapWindow.cycleLimit));
    if (m_gapWindow.multipleStationsPerSource)
    {
        const std::uint64_t returned = m_gapWindow.budgetProven ?
            m_gapWindow.cycleLimit : m_pathHold.automaticObservedReturns;
        const bool retired = std::strcmp(phase, "STATION_CONSUMED") == 0 ||
            std::strcmp(phase, "FORWARD_RETAINED") == 0 || std::strcmp(phase, "SOURCE_RETAINED") == 0 ||
            std::strcmp(phase, "WINDOW_COMPLETED") == 0;
        const std::uint64_t next = m_gapWindow.completedStations + (retired ? 0ULL : returned) + 1ULL;
        const double targetMM = next <= m_gapWindow.stationLimit ?
            m_gapWindow.intervalMM * static_cast<double>(next) : 0.0;
        RtPrintf("[%s] phase=%s sourceIndex=%u dispatch=%llu Q=%016llX completedMM=%016llX targetMM=%016llX remainingMM=%016llX consumed=%u normal=%u localStation=%016llX stationCount=%u stationLimit=%u sourceL=%u sourcePlanned=%u sourceReturned=%llu\n",
            m_gapWindow.tailSupervision ? "GAP-CW-DISTANCE" : m_gapWindow.allowSeamStations ? "GAP-CV-DISTANCE" : "GAP-CU-DISTANCE",
            phase, static_cast<unsigned int>(m_gapWindow.sourceIndex),
            static_cast<unsigned long long>(m_pathHold.dispatch),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.intervalMM)),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.completedForwardMM)),
            static_cast<unsigned long long>(HoldDoubleBits(targetMM)),
            static_cast<unsigned long long>(HoldDoubleBits(targetMM == 0.0 ? 0.0 : targetMM - m_gapWindow.completedForwardMM)),
            m_gapWindow.stationConsumed ? 1U : 0U, m_gapWindow.normalSource ? 1U : 0U,
            static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)),
            static_cast<unsigned int>(m_gapWindow.completedStations), static_cast<unsigned int>(m_gapWindow.stationLimit),
            static_cast<unsigned int>(m_gapWindow.cycleLimit),
            static_cast<unsigned int>(m_pathHold.bound && !m_gapWindow.normalSource ? m_gapWindow.cycleLimit : 0U),
            static_cast<unsigned long long>(returned));
    }
    else if (m_gapWindow.repeatedCumulativeStation)
        RtPrintf("[GAP-CT-DISTANCE] phase=%s sourceIndex=%u dispatch=%llu Q=%016llX completedMM=%016llX targetMM=%016llX remainingMM=%016llX consumed=%u normal=%u localStation=%016llX stationCount=%u stationLimit=%u sourceL=%u\n",
            phase, static_cast<unsigned int>(m_gapWindow.sourceIndex),
            static_cast<unsigned long long>(m_pathHold.dispatch),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.intervalMM)),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.completedForwardMM)),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.stationConsumed ? 0.0 : GetGapPathWindowStationMMSameThread())),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.stationConsumed ? 0.0 :
                GetGapPathWindowStationMMSameThread() - m_gapWindow.completedForwardMM)),
            m_gapWindow.stationConsumed ? 1U : 0U, m_gapWindow.normalSource ? 1U : 0U,
            static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)),
            static_cast<unsigned int>(m_gapWindow.completedStations), static_cast<unsigned int>(m_gapWindow.stationLimit),
            static_cast<unsigned int>(m_gapWindow.cycleLimit));
    else if (m_gapWindow.cumulativeStation)
        RtPrintf("[GAP-CS-DISTANCE] phase=%s sourceIndex=%u dispatch=%llu Q=%016llX completedMM=%016llX remainingMM=%016llX consumed=%u normal=%u localStation=%016llX\n",
            phase, static_cast<unsigned int>(m_gapWindow.sourceIndex),
            static_cast<unsigned long long>(m_pathHold.dispatch),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.intervalMM)),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.completedForwardMM)),
            static_cast<unsigned long long>(HoldDoubleBits(m_gapWindow.stationConsumed ? 0.0 :
                m_gapWindow.intervalMM - m_gapWindow.completedForwardMM)),
            m_gapWindow.stationConsumed ? 1U : 0U, m_gapWindow.normalSource ? 1U : 0U,
            static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathNormalSourceSnapshotSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot) const noexcept
{
    const double length = m_pathHoldView.original.lengthPulse;
    const double endpointBudget = 0.001 + 1e-9 * length;
    return m_gapWindow.active && m_gapWindow.allowNormalSources && m_gapWindow.normalSource &&
        !m_gapWindow.budgetProven && m_gapWindow.budgetPublication == 0ULL &&
        m_gapWindow.budgetFence == 0ULL && m_gapWindow.budgetRequest == 0ULL &&
        m_gapWindow.budgetLatestStop == 0ULL && m_gapWindow.budgetReturnedSBits == 0ULL &&
        m_gapWindow.budgetGeneration == 0ULL && m_state == NCState::RUN &&
        m_pathHold.bound && m_pathHold.automaticEnabled && !m_pathHold.requested &&
        !m_pathHold.startCommitted && !m_pathHold.automaticHoldOwned && !m_pathHold.automaticAdmissionOwned &&
        m_pathHold.requestedHoldSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
        m_pathHold.automaticSettleSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
        m_pathHold.automaticBoundarySequence == 0ULL && m_pathHold.automaticObservedReturns == 0ULL &&
        !m_pathHold.returnHoldRequested && m_pathHold.returnHoldRetreatCount == 0ULL &&
        m_holdResumeAdmissionKind == HoldResumeAdmissionKind::NONE && !m_feedHoldResumeGate.GetSnapshot().active &&
        m_gapPath.active && m_gapPath.automaticResume && m_gapPath.repeating &&
        m_gapPath.lowRetreat && m_gapPath.repeatedLowRetreat && m_gapPath.returnLowTest &&
        m_gapPath.repeatedReturnLow && m_gapPath.returnProbeLimit == m_gapWindow.probeLimit &&
        !m_gapPath.lowInjected && !m_gapPath.held && !m_gapPath.returnHold && !m_gapPath.recoveryInjected &&
        !m_gapPath.returnLowInjected && !m_gapPath.returnRehold && !m_gapPath.returnResumeApplied &&
        !m_gapPath.returnWatchStarted && m_gapPath.returnReholdSequence == 0ULL &&
        m_gapPath.returnProbeHoldCount == 0U && m_gapPath.returnProbeResumeCount == 0U &&
        m_gapPath.returnCompletedFence == 0ULL && m_gapPath.returnCompletedHold == 0ULL &&
        m_gapPath.returnCompletedRehold == 0ULL && m_gapPath.returnCompletedSBits == 0ULL &&
        m_gapPath.returnGeneration == 0ULL && m_gapPath.returnRoundPhase == 0U &&
        m_gapPath.returnWatchPublication == 0ULL && m_gapPath.holdStartMs == 0ULL &&
        m_pathHoldView.original.valid && !m_pathHoldView.original.point &&
        std::isfinite(m_pathHoldView.original.lengthMM) && m_pathHoldView.original.lengthMM > 0.0 &&
        ((m_gapWindow.cumulativeStation && m_gapWindow.stationConsumed) ||
            (m_gapWindow.allowSeamStations &&
                ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM,
                    m_pathHoldView.original.lengthMM, m_pathHoldView.original.lengthPulse,
                    GetGapPathWindowStationMMSameThread()) == 0) ||
            (!m_gapWindow.allowSeamStations && m_pathHoldView.original.lengthMM < (m_gapWindow.cumulativeStation ?
                GetGapPathWindowStationMMSameThread() - m_gapWindow.completedForwardMM : m_gapWindow.intervalMM))) &&
        snapshot.publicationSequence != 0ULL && snapshot.publicationSequence >= m_gapWindow.normalPublication &&
        (!m_gapSignal.active || (snapshot.activeS >= m_gapSignal.normalActiveS &&
            (snapshot.publicationSequence != m_gapWindow.normalPublication || snapshot.activeS == m_gapSignal.normalActiveS))) &&
        snapshot.requestGeneration != 0ULL &&
        (m_gapWindow.normalGeneration == 0ULL || snapshot.requestGeneration == m_gapWindow.normalGeneration) &&
        HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) &&
        m_pathHold.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        snapshot.ownerLease.Matches(m_pathHold.lease) && snapshot.ownerLease.Matches(m_gapWindow.lease) &&
        !snapshot.admissionPending && snapshot.phase == MotionPathCoreHoldExcursionPhase::ARMED && snapshot.reason == 0U &&
        !snapshot.ready && !snapshot.boundaryOnly && snapshot.crossSegment && snapshot.requireReturnAuthorization &&
        snapshot.historyCount == m_pathHoldView.completedCount &&
        snapshot.historyCount == m_gapWindow.initialHistoryCount + m_gapWindow.sourceIndex - 1U &&
        snapshot.activeOrdinal == snapshot.historyCount && snapshot.cycleLimit == m_gapWindow.cycleLimit &&
        snapshot.retreatCount == 0ULL && snapshot.returnCount == 0ULL &&
        snapshot.holdRequestSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
        snapshot.completedHoldRequestSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
        snapshot.heldS == 0.0 && snapshot.retreatS == 0.0 && snapshot.returnedS == 0.0 &&
        std::isfinite(length) && length > 0.0 && snapshot.lengthPulse == length &&
        std::isfinite(snapshot.activeS) && snapshot.activeS >= 0.0 && snapshot.activeS <= length + endpointBudget;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ServiceGapPathNormalSourceSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (snapshot.admissionPending)
    {
        (void)ServiceGapPathAdmissionSameThread(snapshot);
        return false;
    }
    if (snapshot.publicationSequence == 0ULL)
    {
        (void)ServiceGapPathSimulationSameThread(0.0, false, "NORMAL_NO_RT");
        return false;
    }
    if (!IsGapPathSourceWindowScopeValidSameThread() || !ValidateGapPathNormalSourceSnapshotSameThread(snapshot))
    {
        RejectGapPathSimulationSameThread("NORMAL_SOURCE_SCOPE_OR_MOTION");
        return false;
    }
    if (!ValidateGapPathLoadedPublicationSameThread(snapshot)) return false;
    const EDMGap::Snapshot& prior = m_gapInput.Current();
    if (m_gapPath.sequence != 0ULL &&
        (!prior.configured || prior.source != EDMGap::Source::SIMULATED ||
            prior.quality != EDMGap::Quality::VALID || prior.sequence != m_gapPath.sequence ||
            (prior.band != EDMGap::Band::UNKNOWN && prior.band != EDMGap::Band::NORMAL) ||
            (prior.pendingBand != EDMGap::Band::UNKNOWN && prior.pendingBand != EDMGap::Band::NORMAL &&
                !IsGapPathPendingLowCarrySameThread())))
    {
        RejectGapPathSimulationSameThread("NORMAL_SOURCE_INPUT_REVOKED");
        return false;
    }
    // A repeated RT publication may age a known sample but cannot refresh it.
    const bool freshPublication = snapshot.publicationSequence >
        (m_gapSignal.active ? m_gapSignal.publication : m_gapWindow.normalPublication);
    if (!ServiceGapPathSimulationSameThread(snapshot.activeS, freshPublication,
        freshPublication ? "NORMAL_SOURCE" : "NORMAL_PUBLICATION_WAIT", false, nullptr, &snapshot)) return false;
    if (m_gapSignal.active) m_gapSignal.normalActiveS = snapshot.activeS;
    m_gapWindow.normalPublication = snapshot.publicationSequence;
    m_gapWindow.normalGeneration = snapshot.requestGeneration;
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CompleteGapPathNormalSourceProofSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!ServiceGapPathNormalSourceSameThread(snapshot)) return false;
    const bool line = m_pathFeed.armed && m_pathFeed.bound && m_pathFeed.completed &&
        m_pathFeed.consumerAccepted && m_pathFeed.consumerStarted && m_pathFeedMotion.receipt.valid &&
        m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.commit == m_pathHold.commit &&
        (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_pathHold.identity));
    const bool arc = m_pathArc.armed && m_pathArc.bound && m_pathArc.completed &&
        m_pathArc.consumerAccepted && m_pathArc.consumerStarted && m_pathArcMotion.receipt.valid &&
        m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.commit == m_pathHold.commit &&
        (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_pathHold.identity));
    if (line == arc)
    {
        RejectGapPathSimulationSameThread("NORMAL_SOURCE_NOT_COMPLETED");
        return false;
    }
    if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
        m_motion.GetCommandReplaySize() != 0U) return false;
    // Ordinary Motion completion has an existing sub-pulse endpoint tolerance.
    // This observes it; it does not snap coordinates or alter the planner.
    const double endpointBudget = 0.001 + 1e-9 * snapshot.lengthPulse;
    if (std::abs(snapshot.activeS - snapshot.lengthPulse) > endpointBudget)
    {
        RejectGapPathSimulationSameThread("NORMAL_SOURCE_ENDPOINT_MISMATCH");
        return false;
    }
    // DA can finish a short source before the first delayed frame is eligible.
    // Keep servicing its bounded own-sample clock; do not treat NO_SAMPLE as proof.
    if (m_gapQueue.active && !m_gapSignal.sourceSampleSeen) return false;
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    if (!gap.configured || gap.source != EDMGap::Source::SIMULATED || gap.quality != EDMGap::Quality::VALID ||
        m_gapPath.sequence == 0ULL || gap.sequence != m_gapPath.sequence ||
        gap.observedAtMs != m_gapPath.lastServiceMs || gap.sampledAtMs > m_gapPath.lastServiceMs ||
        m_gapPath.lastServiceMs - gap.sampledAtMs > 100ULL)
    {
        RejectGapPathSimulationSameThread("NORMAL_SOURCE_NOT_FRESH");
        return false;
    }
    // An active negative probe cannot complete through a short target endpoint.
    // P19 waits for the frozen producer sample to expire; P20 keeps acquiring
    // until its paused consumer causes queue-full (or an earlier protection).
    if (m_gapInlet.active && (m_gapInlet.freezeStarted || m_gapQueue.consumerPaused ||
        (m_gapRecovery.active && !m_gapRecovery.fenceAccepted)) &&
        m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex) return false;
    // A very short source may finish before the existing 30 ms NORMAL dwell.
    // Keep the completed source and live monitor until genuinely confirmed.
    if ((m_gapInlet.active && !IsGapPathCurrentSampleProvenSameThread()) ||
        gap.band != EDMGap::Band::NORMAL || gap.pendingBand != EDMGap::Band::UNKNOWN ||
        (m_gapSignal.active && (!m_gapSignal.sourceSampleSeen ||
            m_gapPath.sequence <= m_gapSignal.sourceStartSequence))) return false;
    if (!CompleteGapPathSampleQueueSameThread(snapshot)) return false;
    if (!m_gapWindow.normalProven)
    {
        m_gapWindow.normalProven = true;
        LogGapPathSourceWindowSameThread("SOURCE_NORMAL_PROVEN");
        RtPrintf("[%s] sourceIndex=%u dispatch=%llu epoch=%llu publication=%llu generation=%llu sample=%llu ms=%llu activeS=%016llX length=%016llX holds=0 excursions=0\n",
            m_gapWindow.multipleStationsPerSource ? "GAP-CU-NORMAL" : m_gapWindow.repeatedCumulativeStation ? "GAP-CT-NORMAL" : m_gapWindow.cumulativeStation ? "GAP-CS-NORMAL" : "GAP-CR-NORMAL",
            static_cast<unsigned int>(m_gapWindow.sourceIndex), static_cast<unsigned long long>(m_pathHold.dispatch),
            static_cast<unsigned long long>(m_pathHold.identity.epoch), static_cast<unsigned long long>(snapshot.publicationSequence),
            static_cast<unsigned long long>(snapshot.requestGeneration), static_cast<unsigned long long>(gap.sequence),
            static_cast<unsigned long long>(gap.sampledAtMs), static_cast<unsigned long long>(HoldDoubleBits(snapshot.activeS)),
            static_cast<unsigned long long>(HoldDoubleBits(snapshot.lengthPulse)));
    }
    return true;
}

// CW: once the exact excursion budget is spent, input remains supervised until
// the real original source completes. This state carries no resume authority.
NC_PATH_HOLD_NOINLINE
bool NCManager::StartGapPathTailSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!m_gapWindow.tailSupervision || !m_gapWindow.budgetProven || m_gapTail.active ||
        snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE || snapshot.reason != 0U ||
        snapshot.admissionPending || snapshot.ready || snapshot.boundaryOnly ||
        !std::isfinite(snapshot.activeS) || snapshot.activeS < snapshot.returnedS ||
        snapshot.activeS > snapshot.lengthPulse || m_gapPath.sequence == 0ULL)
    {
        RejectGapPathSimulationSameThread("TAIL_ARM_INVALID", &snapshot);
        return false;
    }
    m_gapTail = GapPathTailState{};
    m_gapTail.identity = m_pathHold.identity;
    m_gapTail.lease = m_pathHold.lease;
    m_gapTail.run = m_pathHold.run;
    m_gapTail.cache = m_pathHold.cache;
    m_gapTail.dispatch = m_pathHold.dispatch;
    m_gapTail.commit = m_pathHold.commit;
    m_gapTail.generation = m_gapWindow.budgetGeneration;
    m_gapTail.fence = m_gapWindow.budgetFence;
    m_gapTail.request = m_gapWindow.budgetRequest;
    m_gapTail.latestStop = m_gapWindow.budgetLatestStop;
    m_gapTail.returnedSBits = m_gapWindow.budgetReturnedSBits;
    m_gapTail.budgetPublication = m_gapWindow.budgetPublication;
    m_gapTail.publication = snapshot.publicationSequence;
    m_gapTail.sequence = m_gapPath.sequence;
    m_gapTail.sampleFloor = m_gapPath.sequence;
    m_gapTail.activeS = snapshot.activeS;
    m_gapTail.lengthPulse = snapshot.lengthPulse;
    m_gapTail.historyCount = snapshot.historyCount;
    m_gapTail.cycleLimit = snapshot.cycleLimit;
    m_gapTail.sourceIndex = m_gapWindow.sourceIndex;
    m_gapTail.voltage = m_gapWindow.tailVoltage;
    m_gapTail.active = true;
    // Completion already proved these two grants absent. Retire the remaining
    // automatic arm without resetting its continuous sample/clock session.
    m_pathHold.automaticEnabled = false;
    m_pathHold.automaticBoundarySequence = 0ULL;
    m_pathHold.automaticSettleSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    m_pathHold.automaticNextS = 0.0;
    LogGapPathTailSameThread("TAIL_ARMED");
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathTailScopeSameThread() noexcept
{
    if (!m_gapTail.active || !m_gapWindow.tailSupervision || !m_gapWindow.allowSeamStations ||
        !m_gapWindow.budgetProven || m_gapWindow.normalSource || !m_gapPath.active ||
        !IsGapPathSourceWindowScopeValidSameThread() || m_state != NCState::RUN ||
        m_edmState == EDMState::NOT_READY || !m_pathHold.bound || !m_pathHold.requested ||
        !m_pathHold.startCommitted || m_pathHold.automaticEnabled || m_pathHold.automaticHoldOwned ||
        m_pathHold.automaticAdmissionOwned || m_pathHold.automaticBoundarySequence != 0ULL ||
        m_pathHold.automaticSettleSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE || m_feedHoldResumeGate.GetSnapshot().active ||
        m_gapTail.run != m_pathHold.run || m_gapTail.cache != m_pathHold.cache ||
        m_gapTail.dispatch != m_pathHold.dispatch || m_gapTail.commit != m_pathHold.commit ||
        !HoldIdentityEqual(m_gapTail.identity, m_pathHold.identity) ||
        m_gapTail.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapTail.lease.Matches(m_pathHold.lease) || !m_motion.IsMotionOwnerLeaseCurrent(m_gapTail.lease) ||
        m_gapTail.sourceIndex != m_gapWindow.sourceIndex || m_gapTail.voltage != m_gapWindow.tailVoltage ||
        m_gapTail.generation != m_gapWindow.budgetGeneration || m_gapTail.fence != m_gapWindow.budgetFence ||
        m_gapTail.request != m_gapWindow.budgetRequest || m_gapTail.latestStop != m_gapWindow.budgetLatestStop ||
        m_gapTail.returnedSBits != m_gapWindow.budgetReturnedSBits ||
        m_gapTail.budgetPublication != m_gapWindow.budgetPublication ||
        m_gapTail.request != m_pathHold.requestedHoldSequence || m_gapTail.latestStop != m_gapPath.returnReholdSequence ||
        m_gapTail.historyCount != m_pathHoldView.completedCount || m_gapTail.cycleLimit != m_gapWindow.cycleLimit ||
        m_gapTail.cycleLimit != m_pathHold.automaticObservedReturns ||
        m_gapTail.lengthPulse != m_pathHoldView.original.lengthPulse ||
        m_gapTail.sequence != m_gapPath.sequence || m_gapTail.publication < m_gapTail.budgetPublication)
        return false;
    const bool line = m_pathFeed.armed && m_pathFeed.bound && m_pathFeedMotion.receipt.valid &&
        m_pathFeed.consumerAccepted && m_pathFeed.consumerStarted &&
        m_pathFeed.dispatch == m_gapTail.dispatch && m_pathFeed.commit == m_gapTail.commit &&
        (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_gapTail.identity)) &&
        m_pathFeedMotion.receipt.ownerLease.Matches(m_gapTail.lease);
    const bool arc = m_pathArc.armed && m_pathArc.bound && m_pathArcMotion.receipt.valid &&
        m_pathArc.consumerAccepted && m_pathArc.consumerStarted &&
        m_pathArc.dispatch == m_gapTail.dispatch && m_pathArc.commit == m_gapTail.commit &&
        (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_gapTail.identity)) &&
        m_pathArcMotion.receipt.ownerLease.Matches(m_gapTail.lease);
    if (line == arc) return false;
    const bool completed = line ? m_pathFeed.completed : m_pathArc.completed;
    const std::uint32_t history = m_pathReplayStore.Count();
    return history == m_gapTail.historyCount || (completed && history == m_gapTail.historyCount + 1U);
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathTailSnapshotSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    return snapshot.publicationSequence != 0ULL &&
        HoldTranslationIdentityEqual(snapshot, m_gapTail.identity, CoordSys, m_pathHoldView.translationGeneration) && snapshot.ownerLease.Matches(m_gapTail.lease) &&
        snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE && snapshot.reason == 0U &&
        snapshot.crossSegment && snapshot.requireReturnAuthorization && !snapshot.boundaryOnly &&
        !snapshot.admissionPending && !snapshot.ready && snapshot.requestGeneration == m_gapTail.generation &&
        snapshot.completedHoldRequestSequence == m_gapTail.fence &&
        (snapshot.holdRequestSequence == m_gapTail.request || snapshot.holdRequestSequence == m_gapTail.latestStop) &&
        snapshot.historyCount == m_gapTail.historyCount && snapshot.activeOrdinal == m_gapTail.historyCount &&
        snapshot.cycleLimit == m_gapTail.cycleLimit && snapshot.retreatCount == m_gapTail.cycleLimit &&
        snapshot.returnCount == m_gapTail.cycleLimit &&
        HoldDoubleBits(snapshot.heldS) == m_gapTail.returnedSBits &&
        HoldDoubleBits(snapshot.returnedS) == m_gapTail.returnedSBits &&
        snapshot.lengthPulse == m_gapTail.lengthPulse &&
        snapshot.distanceMM == m_gapWindow.distanceMM && snapshot.feedMMMin == m_gapWindow.feedMMMin &&
        std::isfinite(snapshot.activeS) && snapshot.activeS >= snapshot.returnedS &&
        snapshot.activeS <= m_gapTail.lengthPulse;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ServiceGapPathTailSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!ValidateGapPathTailScopeSameThread())
    {
        RejectGapPathSimulationSameThread("TAIL_SCOPE_REVOKED", &snapshot);
        return false;
    }
    if (snapshot.publicationSequence == 0ULL)
    {
        (void)ServiceGapPathSimulationSameThread(0.0, false, "TAIL_NO_RT");
        return false;
    }
    if (!ValidateGapPathTailSnapshotSameThread(snapshot))
    {
        RejectGapPathSimulationSameThread("TAIL_SCOPE_OR_MOTION", &snapshot);
        return false;
    }
    if (snapshot.publicationSequence < m_gapTail.publication)
    {
        RejectGapPathSimulationSameThread("TAIL_PUBLICATION_REGRESSION", &snapshot);
        return false;
    }
    if (snapshot.activeS < m_gapTail.activeS ||
        (snapshot.publicationSequence == m_gapTail.publication && snapshot.activeS != m_gapTail.activeS))
    {
        RejectGapPathSimulationSameThread("TAIL_PROGRESS_REGRESSION", &snapshot);
        return false;
    }
    const auto& prior = m_gapInput.Current();
    if (!prior.configured || prior.source != EDMGap::Source::SIMULATED ||
        prior.quality != EDMGap::Quality::VALID || prior.sequence != m_gapPath.sequence ||
        (prior.band != EDMGap::Band::NORMAL && prior.band != EDMGap::Band::LOW) ||
        (prior.pendingBand != EDMGap::Band::UNKNOWN && prior.pendingBand != EDMGap::Band::LOW))
    {
        RejectGapPathSimulationSameThread("TAIL_INPUT_REVOKED", &snapshot);
        return false;
    }
    const bool fresh = snapshot.publicationSequence > m_gapTail.publication;
    if (!ServiceGapPathSimulationSameThread(snapshot.activeS, fresh,
        fresh ? "TAIL_SAMPLE" : "TAIL_PUBLICATION_WAIT", false, nullptr, &snapshot)) return false;
    m_gapTail.publication = snapshot.publicationSequence;
    m_gapTail.activeS = snapshot.activeS;
    m_gapTail.sequence = m_gapPath.sequence;
    if (m_gapInlet.active)
    {
        if (IsGapPathCurrentSampleProvenSameThread() &&
            m_gapInlet.acceptedFrame.sample.sequence > m_gapTail.sampleFloor &&
            m_gapInlet.acceptedFrame.publication > m_gapTail.budgetPublication)
            m_gapTail.sampleSeen = true;
    }
    else if (fresh) m_gapTail.sampleSeen = true;
    const auto& gap = m_gapInput.Current();
    if (gap.band == EDMGap::Band::LOW)
    {
        LogGapPathTailSameThread("TAIL_LOW");
        RejectGapPathSimulationSameThread("TAIL_LOW", &snapshot);
        return false;
    }
    if (m_gapTail.sampleSeen && gap.band == EDMGap::Band::NORMAL &&
        gap.pendingBand == EDMGap::Band::UNKNOWN && !m_gapTail.normalLogged)
    {
        m_gapTail.normalLogged = true;
        LogGapPathTailSameThread("TAIL_NORMAL_PROVEN");
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CompleteGapPathTailProofSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!ServiceGapPathTailSameThread(snapshot)) return false;
    const bool line = m_pathFeed.armed && m_pathFeed.bound && m_pathFeed.completed &&
        m_pathFeed.dispatch == m_gapTail.dispatch;
    const bool arc = m_pathArc.armed && m_pathArc.bound && m_pathArc.completed &&
        m_pathArc.dispatch == m_gapTail.dispatch;
    if (line == arc)
    {
        RejectGapPathSimulationSameThread("TAIL_SOURCE_NOT_COMPLETED", &snapshot);
        return false;
    }
    if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
        m_motion.GetCommandReplaySize() != 0U) return false;
    // Keep the existing ordinary Motion endpoint observation tolerance. No
    // coordinate, station or planner threshold is changed by this check.
    const double endpointBudget = 0.001 + 1e-9 * snapshot.lengthPulse;
    if (std::abs(snapshot.activeS - snapshot.lengthPulse) > endpointBudget)
    {
        RejectGapPathSimulationSameThread("TAIL_SOURCE_ENDPOINT_MISMATCH", &snapshot);
        return false;
    }
    const auto& gap = m_gapInput.Current();
    if (!gap.configured || gap.source != EDMGap::Source::SIMULATED || gap.quality != EDMGap::Quality::VALID ||
        gap.sequence != m_gapTail.sequence || gap.observedAtMs != m_gapPath.lastServiceMs ||
        gap.sampledAtMs > m_gapPath.lastServiceMs || m_gapPath.lastServiceMs - gap.sampledAtMs > 100ULL)
    {
        RejectGapPathSimulationSameThread("TAIL_END_NOT_FRESH", &snapshot);
        return false;
    }
    // CY waits for a fresh terminal publication if completion feedback arrived later.
    if (m_gapPending.active && !m_gapPending.injected) return false;
    // Neither the budget publication nor a duplicate can create tail evidence.
    // A completed short source waits for a new publication and real dwell.
    if ((m_gapInlet.active && !IsGapPathCurrentSampleProvenSameThread()) ||
        !m_gapTail.sampleSeen || m_gapTail.publication <= m_gapTail.budgetPublication ||
        gap.band != EDMGap::Band::NORMAL ||
        (gap.pendingBand != EDMGap::Band::UNKNOWN && !IsGapPathPendingLowCarrySameThread())) return false;
    if (!CompleteGapPathSampleQueueSameThread(snapshot)) return false;
    m_gapTail.endProven = true;
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathTailSameThread(const char* phase) const noexcept
{
    const auto& gap = m_gapInput.Current();
    RtPrintf("[GAP-CW-TAIL] phase=%s run=%llu cache=%llu dispatch=%llu commit=%llu epoch=%llu segment=%llu owner=%u ownerGeneration=%llu sourceIndex=%u history=%u generation=%llu publication=%llu budgetPublication=%llu sample=%llu ms=%llu V=%u quality=%s band=%s pending=%s activeS=%016llX length=%016llX returnedS=%016llX retreat=%u return=%u request=%llu latestStop=%llu fence=%llu freshTail=%u endProven=%u motion=1 discharge=0\n",
        phase, static_cast<unsigned long long>(m_gapTail.run), static_cast<unsigned long long>(m_gapTail.cache),
        static_cast<unsigned long long>(m_gapTail.dispatch), static_cast<unsigned long long>(m_gapTail.commit),
        static_cast<unsigned long long>(m_gapTail.identity.epoch), static_cast<unsigned long long>(m_gapTail.identity.segmentId),
        static_cast<unsigned int>(m_gapTail.lease.owner), static_cast<unsigned long long>(m_gapTail.lease.generation),
        static_cast<unsigned int>(m_gapTail.sourceIndex), static_cast<unsigned int>(m_gapTail.historyCount),
        static_cast<unsigned long long>(m_gapTail.generation), static_cast<unsigned long long>(m_gapTail.publication),
        static_cast<unsigned long long>(m_gapTail.budgetPublication), static_cast<unsigned long long>(gap.sequence),
        static_cast<unsigned long long>(gap.sampledAtMs), static_cast<unsigned int>(m_gapTail.voltage),
        EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand),
        static_cast<unsigned long long>(HoldDoubleBits(m_gapTail.activeS)),
        static_cast<unsigned long long>(HoldDoubleBits(m_gapTail.lengthPulse)),
        static_cast<unsigned long long>(m_gapTail.returnedSBits), static_cast<unsigned int>(m_gapTail.cycleLimit),
        static_cast<unsigned int>(m_gapTail.cycleLimit), static_cast<unsigned long long>(m_gapTail.request),
        static_cast<unsigned long long>(m_gapTail.latestStop), static_cast<unsigned long long>(m_gapTail.fence),
        m_gapTail.sampleSeen ? 1U : 0U, m_gapTail.endProven ? 1U : 0U);
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CompleteGapPathSourceBudgetSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!IsGapPathSourceWindowScopeValidSameThread() || m_gapWindow.budgetProven || m_gapWindow.normalSource ||
        m_state != NCState::RUN || !m_pathHold.bound || !m_pathHold.requested || !m_pathHold.startCommitted ||
        !m_pathHold.automaticEnabled || m_pathHold.automaticHoldOwned || m_pathHold.automaticAdmissionOwned ||
        !m_gapPath.active || !m_gapPath.repeatedReturnLow || !m_gapPath.returnResumeApplied ||
        !m_gapPath.returnRehold || !m_gapPath.held || m_gapPath.returnRoundPhase != 3U ||
        m_gapPath.returnProbeLimit != m_gapWindow.probeLimit ||
        m_gapPath.returnProbeHoldCount != m_gapWindow.probeLimit ||
        m_gapPath.returnProbeResumeCount != m_gapWindow.probeLimit ||
        snapshot.publicationSequence == 0ULL || snapshot.publicationSequence <= m_gapPath.returnWatchPublication ||
        !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() || !snapshot.ownerLease.Matches(m_gapWindow.lease) ||
        snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE || !snapshot.crossSegment ||
        !snapshot.requireReturnAuthorization || snapshot.boundaryOnly ||
        snapshot.historyCount != m_pathHoldView.completedCount ||
        snapshot.historyCount != m_gapWindow.initialHistoryCount + m_gapWindow.sourceIndex - 1U ||
        m_pathReplayStore.Count() != snapshot.historyCount || snapshot.activeOrdinal != snapshot.historyCount ||
        snapshot.cycleLimit != m_gapWindow.cycleLimit || snapshot.retreatCount != snapshot.cycleLimit ||
        snapshot.returnCount != snapshot.cycleLimit || m_pathHold.automaticObservedReturns != snapshot.returnCount ||
        snapshot.requestGeneration == 0ULL || snapshot.requestGeneration != m_gapPath.returnGeneration ||
        m_pathHold.requestedHoldSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_gapPath.returnReholdSequence <= m_pathHold.requestedHoldSequence ||
        (snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence &&
            snapshot.holdRequestSequence != m_gapPath.returnReholdSequence) ||
        snapshot.completedHoldRequestSequence < m_gapPath.returnReholdSequence ||
        !std::isfinite(snapshot.lengthPulse) || snapshot.lengthPulse <= 0.0 ||
        snapshot.lengthPulse != m_pathHoldView.original.lengthPulse ||
        !std::isfinite(snapshot.heldS) || snapshot.heldS < 0.0 || snapshot.heldS > snapshot.lengthPulse ||
        HoldDoubleBits(snapshot.returnedS) != HoldDoubleBits(snapshot.heldS) ||
        !IsGapPathAutomaticNormalSameThread() || !IsGapPathCurrentSampleProvenSameThread())
    {
        RejectGapPathSimulationSameThread("WINDOW_BUDGET_NOT_PROVEN");
        return false;
    }
    m_gapWindow.budgetPublication = snapshot.publicationSequence;
    m_gapWindow.budgetFence = snapshot.completedHoldRequestSequence;
    m_gapWindow.budgetRequest = m_pathHold.requestedHoldSequence;
    m_gapWindow.budgetLatestStop = m_gapPath.returnReholdSequence;
    m_gapWindow.budgetReturnedSBits = HoldDoubleBits(snapshot.returnedS);
    m_gapWindow.budgetGeneration = snapshot.requestGeneration;
    m_gapWindow.budgetProven = true;
    LogGapPathSourceWindowSameThread("SOURCE_BUDGET_PROVEN");
    if (m_gapWindow.tailSupervision) return StartGapPathTailSameThread(snapshot);
    // Only this exact internal completion retires per-source GAP while retaining K authority.
    ClearPathCoreHoldAutomaticStateSameThread("BUDGET_DONE");
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CompleteGapPathSourceSameThread(bool line) noexcept
{
    if (!m_gapWindow.active) return true;
    const bool sourceCompleted = line ?
        (m_pathFeed.armed && !m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.completed &&
            m_pathFeed.consumerAccepted && m_pathFeed.consumerStarted && m_pathFeedMotion.receipt.valid &&
            m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.commit == m_pathHold.commit &&
            (m_pathFeedMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathFeedMotion.receipt.identity, m_pathHold.identity))) :
        (m_pathArc.armed && !m_pathArc.pending && m_pathArc.bound && m_pathArc.completed &&
            m_pathArc.consumerAccepted && m_pathArc.consumerStarted && m_pathArcMotion.receipt.valid &&
            m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.commit == m_pathHold.commit &&
            (m_pathArcMotion.receipt.translationGeneration == m_pathHoldView.translationGeneration &&
            HoldIdentityEqual(m_pathArcMotion.receipt.identity, m_pathHold.identity)));
    const std::uint32_t expectedCount = m_gapWindow.initialHistoryCount + m_gapWindow.sourceIndex;
    const bool normal = m_gapWindow.allowNormalSources && m_gapWindow.normalSource;
    if (!IsGapPathSourceWindowScopeValidSameThread() || m_state != NCState::RUN ||
        !m_pathHold.bound || m_pathHold.automaticHoldOwned || m_pathHold.automaticAdmissionOwned ||
        (!normal && (!m_gapWindow.budgetProven || !m_pathHold.requested || !m_pathHold.startCommitted ||
            (m_gapWindow.tailSupervision ? (!m_gapTail.active || !m_gapPath.active) : m_gapPath.active) ||
            m_pathHold.automaticEnabled)) ||
        (normal && (!m_gapWindow.normalProven || m_pathHold.requested || m_pathHold.startCommitted ||
            !m_gapPath.active || !m_pathHold.automaticEnabled)) || !sourceCompleted ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_pathReplayStore.Count() != expectedCount || expectedCount == 0U || expectedCount > NCPathCoreRetainedPath::Capacity ||
        (!normal && (m_gapWindow.budgetPublication == 0ULL || m_gapWindow.budgetRequest == 0ULL ||
            m_gapWindow.budgetRequest != m_pathHold.requestedHoldSequence ||
            m_gapWindow.budgetLatestStop <= m_gapWindow.budgetRequest ||
            m_gapWindow.budgetFence < m_gapWindow.budgetLatestStop || m_gapWindow.budgetGeneration == 0ULL)))
    {
        RejectGapPathSimulationSameThread("WINDOW_SOURCE_FINALIZATION_SCOPE");
        return false;
    }
    const PathReplaySource& retained = m_pathReplaySource[expectedCount - 1U];
    const NCPathCoreRetainedGeometry* geometry = m_pathReplayStore.Get(expectedCount - 1U);
    if (geometry == nullptr || !HoldGeometryEqual(*geometry, m_pathHoldView.original) ||
        retained.translationGeneration != m_pathHoldView.translationGeneration ||
        !HoldIdentityEqual(retained.identity, m_pathHold.identity) || retained.dispatch != m_pathHold.dispatch ||
        retained.commit != m_pathHold.commit || retained.sourcePC != m_pathHold.sourcePC ||
        retained.sourceLine != m_pathHold.sourceLine)
    {
        RejectGapPathSimulationSameThread("WINDOW_RETAINED_SOURCE_MISMATCH");
        return false;
    }
    const auto terminal = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (terminal.publicationSequence == 0ULL)
    {
        if (m_gapTail.active) (void)ServiceGapPathTailSameThread(terminal);
        else if (normal) (void)ServiceGapPathSimulationSameThread(0.0, false, "NORMAL_FINAL_NO_RT");
        return false; // Retry callback; never append this source twice.
    }
    if (normal)
    {
        if (!CompleteGapPathNormalSourceProofSameThread(terminal)) return false;
    }
    else if (terminal.publicationSequence < m_gapWindow.budgetPublication ||
        !HoldTranslationIdentityEqual(terminal, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) || !terminal.ownerLease.Matches(m_gapWindow.lease) ||
        terminal.phase != MotionPathCoreHoldExcursionPhase::COMPLETE || !terminal.crossSegment ||
        !terminal.requireReturnAuthorization || terminal.boundaryOnly ||
        terminal.requestGeneration != m_gapWindow.budgetGeneration ||
        terminal.historyCount != expectedCount - 1U || terminal.activeOrdinal != terminal.historyCount ||
        terminal.cycleLimit != m_gapWindow.cycleLimit || terminal.retreatCount != terminal.cycleLimit ||
        terminal.returnCount != terminal.cycleLimit || terminal.completedHoldRequestSequence != m_gapWindow.budgetFence ||
        (terminal.holdRequestSequence != m_gapWindow.budgetRequest && terminal.holdRequestSequence != m_gapWindow.budgetLatestStop) ||
        terminal.lengthPulse != m_pathHoldView.original.lengthPulse ||
        HoldDoubleBits(terminal.heldS) != m_gapWindow.budgetReturnedSBits ||
        HoldDoubleBits(terminal.returnedS) != m_gapWindow.budgetReturnedSBits)
    {
        RejectGapPathSimulationSameThread("WINDOW_FINAL_RETURN_PROOF_CHANGED");
        return false;
    }
    if (!normal && m_gapWindow.tailSupervision && !CompleteGapPathTailProofSameThread(terminal)) return false;
    const bool continueSignal = m_gapSignal.active && m_gapWindow.sourceIndex < m_gapWindow.sourceLimit;
    if (m_gapPending.active &&
        ((!continueSignal && (!m_gapPending.injected || !m_gapPending.carried || !m_gapPending.resolved)) ||
            (m_gapTail.active && (!m_gapPending.injected ||
                (!m_gapPending.resolved && !IsGapPathPendingLowCarrySameThread())))))
    {
        RejectGapPathSimulationSameThread("CY_PROBE_NOT_COMPLETED", &terminal);
        return false;
    }
    if (m_gapSignal.active && (m_gapWindow.sourceGapMs == 150U || m_gapWindow.recoverQueuedInput))
    {
        if (m_gapTail.active && m_gapSignal.dropTargetSourceIndex == 0U)
        {
            if (!continueSignal)
            {
                RejectGapPathSimulationSameThread("HOOK_TARGET_UNAVAILABLE");
                return false;
            }
            m_gapSignal.dropTargetSourceIndex = m_gapWindow.sourceIndex + 1U;
        }
        if (!continueSignal && !m_gapSignal.dropConsumed)
        {
            RejectGapPathSimulationSameThread("HOOK_NOT_COMPLETED");
            return false;
        }
    }
    if (m_gapWindow.cumulativeStation)
    {
        const double completedMM = m_gapWindow.completedForwardMM + geometry->lengthMM;
        const bool consumeStation = !normal && m_gapWindow.budgetProven && !m_gapWindow.stationConsumed;
        if (!std::isfinite(completedMM) || completedMM <= m_gapWindow.completedForwardMM ||
            (!m_gapWindow.stationConsumed &&
                ((normal && (completedMM > GetGapPathWindowStationMMSameThread() ||
                    (!m_gapWindow.allowSeamStations && completedMM == GetGapPathWindowStationMMSameThread()))) ||
                    (!normal && (!consumeStation || completedMM <= GetGapPathWindowStationMMSameThread())))))
        {
            RejectGapPathSimulationSameThread("WINDOW_CUMULATIVE_DISTANCE_INVALID");
            return false;
        }
        if (m_gapWindow.repeatedCumulativeStation && !m_gapWindow.stationConsumed &&
            ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM, geometry->lengthMM,
                geometry->lengthPulse, GetGapPathWindowStationMMSameThread()) != (normal ? 0 : 1))
        {
            RejectGapPathSimulationSameThread("STATION_ENDPOINT_AMBIGUOUS");
            return false;
        }
        if (m_gapWindow.repeatedCumulativeStation && !m_gapWindow.multipleStationsPerSource && consumeStation &&
            (m_gapWindow.completedStations >= m_gapWindow.stationLimit ||
                !IsGapPathNextStationBeyondSourceSameThread(m_gapWindow.completedForwardMM,
                    geometry->lengthMM, geometry->lengthPulse, m_gapWindow.completedStations)))
        {
            RejectGapPathSimulationSameThread("MULTIPLE_STATIONS_IN_SOURCE");
            return false;
        }
        std::uint32_t consumedCount = 1U;
        if (m_gapWindow.multipleStationsPerSource)
        {
            if (!CountGapPathSourceStationsSameThread(m_gapWindow.completedForwardMM,
                geometry->lengthMM, geometry->lengthPulse, m_gapWindow.completedStations, consumedCount) ||
                (normal ? consumedCount != 0U :
                    (!consumeStation || consumedCount == 0U || consumedCount != terminal.returnCount ||
                        consumedCount != m_gapWindow.cycleLimit)))
            {
                RejectGapPathSimulationSameThread("WINDOW_SOURCE_STATIONS_NOT_PROVEN");
                return false;
            }
        }
        // No earlier admission, HOLD, returnedS or budget event advances Q.
        // The callback cannot reach this point twice: success changes source
        // identity/index below; publication collisions return before this write.
        m_gapWindow.completedForwardMM = completedMM;
        if (consumeStation)
        {
            if (m_gapWindow.repeatedCumulativeStation)
            {
                m_gapWindow.completedStations += consumedCount;
                m_gapWindow.stationConsumed = m_gapWindow.completedStations == m_gapWindow.stationLimit;
            }
            else m_gapWindow.stationConsumed = true;
        }
        LogGapPathSourceWindowSameThread(consumeStation ? "STATION_CONSUMED" : "FORWARD_RETAINED");
        if (m_gapWindow.sourceIndex == m_gapWindow.sourceLimit && !m_gapWindow.stationConsumed)
        {
            RejectGapPathSimulationSameThread(m_gapWindow.repeatedCumulativeStation ?
                "WINDOW_STATIONS_NOT_COMPLETED" : "WINDOW_STATION_NOT_REACHED");
            return false;
        }
    }
    if (m_gapTail.active)
    {
        LogGapPathTailSameThread("TAIL_COMPLETED");
        if (!continueSignal) ClearPathCoreHoldAutomaticStateSameThread("TAIL_SOURCE_DONE");
    }
    LogGapPathSourceWindowSameThread("SOURCE_RETAINED");
    if (normal && !continueSignal) ClearPathCoreHoldAutomaticStateSameThread("NORMAL_SOURCE_DONE");
    m_gapWindow.previousIdentity = m_pathHold.identity;
    m_gapWindow.previousDispatch = m_pathHold.dispatch;
    m_gapWindow.previousCommit = m_pathHold.commit;
    // This callback already proved real source completion and groupDone. RESET never enters here.
    m_motion.CancelPathCoreHoldExcursion();
    if (m_gapWindow.sourceIndex == m_gapWindow.sourceLimit)
    {
        LogGapPathSourceWindowSameThread("WINDOW_COMPLETED");
        m_gapWindow = GapPathSourceWindowState{};
        m_pathHold = PathHoldState{};
        return true;
    }
    ++m_gapWindow.sourceIndex;
    if (m_gapWindow.multipleStationsPerSource) m_gapWindow.cycleLimit = 1U;
    m_gapWindow.budgetProven = false;
    m_gapWindow.budgetPublication = 0ULL;
    m_gapWindow.budgetFence = 0ULL;
    m_gapWindow.budgetRequest = 0ULL;
    m_gapWindow.budgetLatestStop = 0ULL;
    m_gapWindow.budgetReturnedSBits = 0ULL;
    m_gapWindow.budgetGeneration = 0ULL;
    m_gapWindow.normalSource = false;
    m_gapWindow.normalProven = false;
    m_gapWindow.normalPublication = 0ULL;
    m_gapWindow.normalGeneration = 0ULL;
    m_gapWindow.seamPublication = 0ULL;
    m_pathHold = PathHoldState{};
    m_pathHold.run = m_gapWindow.run;
    m_pathHold.cache = m_gapWindow.cache;
    m_pathHold.lease = m_gapWindow.lease;
    m_pathHold.distanceMM = m_gapWindow.distanceMM;
    m_pathHold.feedMMMin = m_gapWindow.feedMMMin;
    m_pathHold.cycleLimit = m_gapWindow.cycleLimit;
    m_pathHold.automaticIntervalMM = m_gapWindow.intervalMM;
    m_pathHold.crossSegment = true;
    m_pathHold.requireReturnAuthorization = true;
    m_pathHold.automaticEnabled = true;
    m_pathHold.armed = true;
    m_pathHold.code = 1U;
    m_pathHoldView.completedCount = 0U; // Next real Begin supplies its own dispatch ceiling.
    if (continueSignal)
    {
        if (!StartNextGapPathSignalSourceSameThread()) return false;
    }
    else if (!StartGapPathSimulationSameThread(true, true, true, true, true, true,
        static_cast<std::uint8_t>(m_gapWindow.probeLimit))) return false;
    LogGapPathSourceWindowSameThread("NEXT_SOURCE_ARMED");
    return true;
}

// CY preserves one terminal LOW candidate. Signal evidence never grants Motion authority.
NC_PATH_HOLD_NOINLINE
bool NCManager::IsGapPathPendingLowCarrySameThread() const noexcept
{
    const auto& p = m_gapPending;
    const auto& gap = m_gapInput.Current();
    if (!p.active || !p.injected || p.resolved || !m_gapWindow.active ||
        !m_gapWindow.pendingLowAcrossSources || !m_gapWindow.continuousSignal || !m_gapSignal.active ||
        p.run != m_gapWindow.run || p.cache != m_gapWindow.cache ||
        !p.originIdentity.IsAssigned() || p.originDispatch == 0ULL || p.originCommit == 0ULL ||
        p.originGeneration == 0ULL || p.originPublication == 0ULL || p.firstSequence == 0ULL ||
        p.originSourceIndex == 0U || p.targetSourceIndex != p.originSourceIndex + 1U ||
        p.targetSourceIndex > m_gapWindow.sourceLimit || !gap.configured ||
        gap.source != EDMGap::Source::SIMULATED || gap.quality != EDMGap::Quality::VALID ||
        gap.band != EDMGap::Band::NORMAL || gap.pendingBand != EDMGap::Band::LOW ||
        gap.voltageMv != 20000 || gap.pendingSinceMs != p.pendingSinceMs ||
        gap.pendingSinceMs > gap.sampledAtMs || gap.sequence < p.firstSequence ||
        gap.sequence != m_gapPath.sequence || gap.observedAtMs != m_gapPath.lastServiceMs ||
        gap.sampledAtMs > m_gapPath.lastServiceMs || m_gapPath.lastServiceMs - gap.sampledAtMs > 100ULL)
        return false;
    if (m_gapWindow.sourceIndex == p.originSourceIndex)
        return !p.carried && m_gapTail.active && m_pathHold.bound &&
        HoldIdentityEqual(p.originIdentity, m_pathHold.identity) &&
        HoldIdentityEqual(p.originIdentity, m_gapTail.identity) &&
        p.originDispatch == m_pathHold.dispatch && p.originCommit == m_pathHold.commit &&
        p.originGeneration == m_gapTail.generation && m_gapSignal.sourceSampleSeen;
    return m_gapWindow.sourceIndex == p.targetSourceIndex &&
        HoldIdentityEqual(p.originIdentity, m_gapWindow.previousIdentity) &&
        p.originDispatch == m_gapWindow.previousDispatch && p.originCommit == m_gapWindow.previousCommit &&
        (p.carried || (m_pathHold.armed && !m_pathHold.bound));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::PrepareGapPathPendingLowSampleSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot, std::uint64_t nowMs, std::int32_t& voltageMv) noexcept
{
    if (!m_gapPending.active) return true;
    if (m_gapPending.resolved) return true;
    if (!m_gapPending.injected)
    {
        if (!m_gapTail.active) return true;
        const bool line = m_pathFeed.dispatch == m_gapTail.dispatch && m_pathFeed.completed;
        const bool arc = m_pathArc.dispatch == m_gapTail.dispatch && m_pathArc.completed;
        if (!line && !arc) return true;
        if (line == arc || !ValidateGapPathTailScopeSameThread() ||
            !ValidateGapPathTailSnapshotSameThread(snapshot) ||
            snapshot.publicationSequence <= m_gapTail.publication ||
            snapshot.publicationSequence <= m_gapTail.budgetPublication)
        {
            RejectGapPathSimulationSameThread("CY_ENDPOINT_PUBLICATION_INVALID", &snapshot);
            return false;
        }
        if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
            m_motion.GetCommandReplaySize() != 0U) return true;
        const double endpointBudget = 0.001 + 1e-9 * snapshot.lengthPulse;
        if (std::abs(snapshot.activeS - snapshot.lengthPulse) > endpointBudget)
        {
            RejectGapPathSimulationSameThread("CY_ENDPOINT_MISMATCH", &snapshot);
            return false;
        }
        if (m_gapWindow.sourceIndex >= m_gapWindow.sourceLimit)
        {
            RejectGapPathSimulationSameThread("CY_TARGET_UNAVAILABLE", &snapshot);
            return false;
        }
        const auto& gap = m_gapInput.Current();
        if (!gap.configured || gap.source != EDMGap::Source::SIMULATED ||
            gap.quality != EDMGap::Quality::VALID || gap.band != EDMGap::Band::NORMAL ||
            gap.pendingBand != EDMGap::Band::UNKNOWN || gap.sequence != m_gapPath.sequence ||
            !m_gapSignal.sourceSampleSeen || gap.sequence <= m_gapSignal.sourceStartSequence ||
            gap.observedAtMs != nowMs || gap.sampledAtMs > nowMs || nowMs - gap.sampledAtMs > 100ULL ||
            gap.sequence == (std::numeric_limits<std::uint64_t>::max)())
        {
            RejectGapPathSimulationSameThread("CY_ENDPOINT_INPUT_INVALID", &snapshot);
            return false;
        }
        m_gapPending.originIdentity = snapshot.identity;
        m_gapPending.originDispatch = m_pathHold.dispatch;
        m_gapPending.originCommit = m_pathHold.commit;
        m_gapPending.originGeneration = snapshot.requestGeneration;
        m_gapPending.originPublication = snapshot.publicationSequence;
        m_gapPending.originSourceIndex = m_gapWindow.sourceIndex;
        m_gapPending.targetSourceIndex = m_gapWindow.sourceIndex + 1U;
        m_gapPending.firstSequence = m_gapPath.sequence + 1ULL;
        m_gapPending.pendingSinceMs = nowMs;
        m_gapPending.injected = true;
        voltageMv = 20000;
        return true;
    }
    if (!IsGapPathPendingLowCarrySameThread())
    {
        RejectGapPathSimulationSameThread("CY_PENDING_TOKEN_CHANGED", &snapshot);
        return false;
    }
    if (m_gapWindow.sourceIndex == m_gapPending.originSourceIndex)
    {
        voltageMv = 20000; // Do not clear the candidate on a later terminal publication.
        return true;
    }
    if (!m_gapPending.carried || m_gapWindow.sourceIndex != m_gapPending.targetSourceIndex ||
        !m_gapWindow.normalSource || !ValidateGapPathNormalSourceSnapshotSameThread(snapshot))
    {
        RejectGapPathSimulationSameThread("CY_TARGET_NOT_NORMAL", &snapshot);
        return false;
    }
    voltageMv = static_cast<std::int32_t>(m_gapWindow.pendingLowVoltage) * 1000;
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ObserveGapPathPendingLowSampleSameThread(const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!m_gapPending.active || !m_gapPending.injected || m_gapPending.resolved) return true;
    const auto& gap = m_gapInput.Current();
    const bool origin = m_gapWindow.sourceIndex == m_gapPending.originSourceIndex && !m_gapPending.carried;
    const bool target = m_gapWindow.sourceIndex == m_gapPending.targetSourceIndex && m_gapPending.carried;
    if ((!origin && !target) || !gap.configured || gap.source != EDMGap::Source::SIMULATED ||
        gap.quality != EDMGap::Quality::VALID || gap.sequence != m_gapPath.sequence ||
        gap.sequence < m_gapPending.firstSequence || !IsGapPathCurrentSampleProvenSameThread() ||
        !m_gapSignal.sourceSampleSeen ||
        (target && (gap.sequence <= m_gapSignal.sourceStartSequence || !m_gapWindow.normalSource)))
    {
        RejectGapPathSimulationSameThread("CY_PENDING_SAMPLE_INVALID", &snapshot);
        return false;
    }
    if (gap.band == EDMGap::Band::LOW)
    {
        if (gap.sequence <= m_gapPending.firstSequence || gap.sampledAtMs < m_gapPending.pendingSinceMs ||
            gap.sampledAtMs - m_gapPending.pendingSinceMs < 30ULL)
        {
            RejectGapPathSimulationSameThread("CY_LOW_PROOF_INVALID", &snapshot);
            return false;
        }
        LogGapPathPendingLowSameThread("LOW_CONFIRMED");
        RejectGapPathSimulationSameThread("CY_LOW_CONFIRMED", &snapshot);
        return false;
    }
    if (target && m_gapWindow.pendingLowVoltage == 50U && gap.band == EDMGap::Band::NORMAL &&
        gap.pendingBand == EDMGap::Band::UNKNOWN && gap.voltageMv == 50000)
    {
        m_gapPending.resolved = true;
        LogGapPathPendingLowSameThread("LOW_CANCELLED");
        return true;
    }
    if (!IsGapPathPendingLowCarrySameThread())
    {
        RejectGapPathSimulationSameThread("CY_PENDING_NOT_PRESERVED", &snapshot);
        return false;
    }
    if (origin && gap.sequence == m_gapPending.firstSequence)
        LogGapPathPendingLowSameThread("ENDPOINT_LOW_PENDING");
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathPendingLowSameThread(const char* phase) const noexcept
{
    const auto& gap = m_gapInput.Current();
    RtPrintf("[GAP-CY] phase=%s run=%llu source=%u origin=%u target=%u dispatch=%llu epoch=%llu pub=%llu carried=%u resolved=%u U=%u\n",
        phase, static_cast<unsigned long long>(m_gapPending.run), m_gapWindow.sourceIndex,
        m_gapPending.originSourceIndex, m_gapPending.targetSourceIndex,
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_gapSignal.publication), m_gapPending.carried ? 1U : 0U,
        m_gapPending.resolved ? 1U : 0U, static_cast<unsigned int>(m_gapWindow.pendingLowVoltage));
    RtPrintf("[GAP-CY-SAMPLE] seq=%llu first=%llu sampleMs=%llu sinceMs=%llu elapsedMs=%llu quality=%s band=%s pending=%s\n",
        static_cast<unsigned long long>(gap.sequence), static_cast<unsigned long long>(m_gapPending.firstSequence),
        static_cast<unsigned long long>(gap.sampledAtMs), static_cast<unsigned long long>(m_gapPending.pendingSinceMs),
        static_cast<unsigned long long>(gap.sampledAtMs >= m_gapPending.pendingSinceMs ?
            gap.sampledAtMs - m_gapPending.pendingSinceMs : 0ULL),
        EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand));
}

// CX: this helper is reached only after the real source callback validated
// completion, retention and fresh NORMAL, and installed the next source arm.
NC_PATH_HOLD_NOINLINE
bool NCManager::StartNextGapPathSignalSourceSameThread() noexcept
{
    const auto& gap = m_gapInput.Current();
    if (!m_gapSignal.active || !m_gapWindow.active || !m_gapWindow.continuousSignal ||
        !m_gapPath.active || !m_gapPath.clockStarted || m_gapPath.frequency == 0ULL ||
        m_gapSignal.run != m_gapWindow.run || m_gapSignal.cache != m_gapWindow.cache ||
        m_gapSignal.sourceIndex + 1U != m_gapWindow.sourceIndex ||
        m_gapWindow.sourceIndex > m_gapWindow.sourceLimit ||
        !m_gapWindow.previousIdentity.IsAssigned() || m_gapWindow.previousDispatch == 0ULL ||
        m_gapWindow.previousCommit == 0ULL || !m_gapSignal.sourceSampleSeen ||
        m_gapPath.sequence <= m_gapSignal.sourceStartSequence || !gap.configured ||
        gap.source != EDMGap::Source::SIMULATED || gap.quality != EDMGap::Quality::VALID ||
        gap.band != EDMGap::Band::NORMAL ||
        (gap.pendingBand != EDMGap::Band::UNKNOWN && !IsGapPathPendingLowCarrySameThread()) ||
        gap.sequence != m_gapPath.sequence || gap.observedAtMs != m_gapPath.lastServiceMs ||
        gap.sampledAtMs > m_gapPath.lastServiceMs || m_gapPath.lastServiceMs - gap.sampledAtMs > 100ULL ||
        !m_pathHold.armed || m_pathHold.bound || m_pathHold.requested || m_pathHold.startCommitted ||
        m_pathHold.automaticHoldOwned || m_pathHold.automaticAdmissionOwned ||
        m_pathHold.requestedHoldSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_pathHold.automaticSettleSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        m_pathHold.automaticBoundarySequence != 0ULL || m_pathHold.automaticObservedReturns != 0ULL ||
        m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE || m_feedHoldResumeGate.GetSnapshot().active)
    {
        RejectGapPathSimulationSameThread("CX_HANDOFF_NOT_PROVEN");
        return false;
    }
    if (m_gapQueue.active)
    {
        const auto& fence = m_gapQueue.terminalFrame;
        const auto& seal = m_gapQueue.terminalSeal;
        if (fence.run != seal.run || fence.cache != seal.cache || fence.sourceIndex != seal.sourceIndex ||
            fence.publication != seal.publication || !HoldIdentityEqual(fence.identity, seal.identity) ||
            !fence.lease.Matches(seal.lease) || fence.sample.sequence != seal.sample.sequence ||
            fence.sample.sampledAtMs != seal.sample.sampledAtMs || fence.sample.voltageMv != seal.sample.voltageMv ||
            fence.sample.source != seal.sample.source || fence.sample.valid != seal.sample.valid ||
            !m_gapQueue.terminalSealed || !m_gapQueue.terminalDrained || m_gapQueue.count != 0U ||
            m_gapQueue.consumerPaused || !m_gapInlet.accepted ||
            m_gapPath.sequence != fence.sample.sequence || m_gapInlet.acquisitionSequence != fence.sample.sequence ||
            fence.sample.sequence <= m_gapInlet.sourceFloor ||
            fence.run != m_gapWindow.run || fence.cache != m_gapWindow.cache ||
            fence.sourceIndex + 1U != m_gapWindow.sourceIndex ||
            !HoldIdentityEqual(fence.identity, m_gapWindow.previousIdentity) ||
            !fence.lease.Matches(m_gapWindow.lease) ||
            m_gapQueue.terminalPublication != m_gapQueue.terminalPublicationSeal ||
            fence.publication > m_gapQueue.terminalPublication ||
            m_gapQueue.terminalPublication > m_gapSignal.publication ||
            fence.sample.sampledAtMs != gap.sampledAtMs || fence.sample.voltageMv != gap.voltageMv ||
            fence.sample.source != gap.source || !fence.sample.valid)
        {
            RejectGapPathSimulationSameThread("DA_HANDOFF_NOT_DRAINED");
            return false;
        }
        if (m_gapRecovery.active && (!ValidateGapPathQueueRecoverySameThread(true) ||
            (m_gapSignal.sourceIndex == m_gapSignal.dropTargetSourceIndex && !m_gapRecovery.fenceAccepted)))
        {
            RejectGapPathSimulationSameThread("DB_HANDOFF_NOT_RECOVERED");
            return false;
        }
        m_gapQueue = GapPathSampleQueueState{};
        m_gapQueue.active = true;
        m_gapRecovery = GapPathQueueRecoveryState{};
        m_gapRecovery.active = m_gapWindow.recoverQueuedInput;
    }
    if (m_gapPending.active && m_gapPending.injected && !m_gapPending.resolved)
    {
        if (m_gapPending.carried || m_gapWindow.sourceIndex != m_gapPending.targetSourceIndex ||
            !IsGapPathPendingLowCarrySameThread())
        {
            RejectGapPathSimulationSameThread("CY_PENDING_HANDOFF_CHANGED");
            return false;
        }
        m_gapPending.carried = true;
        LogGapPathPendingLowSameThread("PENDING_LOW_CARRIED");
    }
    GapPathSimulationState next{};
    next.active = true;
    next.automaticResume = true;
    next.repeating = true;
    next.lowRetreat = true;
    next.repeatedLowRetreat = true;
    next.returnLowTest = true;
    next.repeatedReturnLow = true;
    next.returnProbeLimit = static_cast<std::uint8_t>(m_gapWindow.probeLimit);
    next.frequency = m_gapPath.frequency;
    next.lastTicks = m_gapPath.lastTicks;
    next.lastServiceMs = m_gapPath.lastServiceMs;
    next.firstServiceMs = m_gapPath.lastServiceMs;
    next.sequence = m_gapPath.sequence;
    next.stalledCalls = m_gapPath.stalledCalls;
    next.clockStarted = true;
    m_gapPath = next;
    m_gapTail = GapPathTailState{};
    m_gapAdmission = GapPathAdmissionState{};
    m_gapSignal.sourceStartSequence = gap.sequence;
    m_gapSignal.sourceSampledAtMs = gap.sampledAtMs;
    m_gapSignal.sourceFirstServiceMs = m_gapPath.lastServiceMs;
    m_gapSignal.publication = 0ULL;
    m_gapSignal.normalActiveS = 0.0;
    m_gapSignal.sourceIndex = m_gapWindow.sourceIndex;
    m_gapSignal.sourceClockStarted = true;
    m_gapSignal.sourceSampleSeen = false;
    m_gapSignal.handoffPending = true;
    if (m_gapInlet.active)
    {
        m_gapInlet.frame = GapPathSampleFrame{};
        m_gapInlet.sealedFrame = GapPathSampleFrame{};
        m_gapInlet.acceptedFrame = GapPathSampleFrame{};
        m_gapInlet.sourceFloor = m_gapInlet.acquisitionSequence;
        m_gapInlet.acceptedPublication = 0ULL;
        m_gapInlet.available = m_gapInlet.accepted = false;
        m_gapInlet.acquiredLogged = m_gapInlet.cachedLogged = false;
        // H150 cannot survive to a later source: its stale alarm comes first.
    }
    LogGapPathSignalSessionSameThread("SOURCE_SIGNAL_CARRIED");
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSignalSessionSameThread(const char* phase) const noexcept
{
    const auto& gap = m_gapInput.Current();
    const std::uint64_t nowMs = m_gapServiceCurrent.nowValid ?
        m_gapServiceCurrent.nowMs : m_gapPath.lastServiceMs;
    RtPrintf("[GAP-CX] phase=%s run=%llu source=%u dispatch=%llu epoch=%llu pub=%llu first=%u target=%u\n",
        phase, static_cast<unsigned long long>(m_gapSignal.run), m_gapSignal.sourceIndex,
        static_cast<unsigned long long>(m_pathHold.dispatch), static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_gapSignal.publication), m_gapSignal.sourceSampleSeen ? 1U : 0U,
        m_gapSignal.dropTargetSourceIndex);
    RtPrintf("[GAP-CX-SAMPLE] seq=%llu frontier=%llu sampleMs=%llu nowMs=%llu ageMs=%llu quality=%s band=%s pending=%s\n",
        static_cast<unsigned long long>(gap.sequence), static_cast<unsigned long long>(m_gapSignal.sourceStartSequence),
        static_cast<unsigned long long>(gap.sampledAtMs), static_cast<unsigned long long>(nowMs),
        static_cast<unsigned long long>(nowMs >= gap.sampledAtMs ? nowMs - gap.sampledAtMs : 0ULL),
        EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band), EDMGap::BandName(gap.pendingBand));
}


// CF: distance-triggered dry-run supervision. It never supplies GAP velocity.
NC_PATH_HOLD_NOINLINE
void NCManager::CancelPathCoreHoldAutomaticSameThread(const char* reason) noexcept
{
    if (m_gapWindow.active)
    {
        LogGapPathSourceWindowSameThread("WINDOW_CANCELLED", reason);
        if (!m_pathHold.bound) m_pathHold.armed = false;
    }
    m_gapWindow = GapPathSourceWindowState{};
    ClearPathCoreHoldAutomaticStateSameThread(reason);
}

NC_PATH_HOLD_NOINLINE
void NCManager::ClearPathCoreHoldAutomaticStateSameThread(const char* reason) noexcept
{
    if (m_gapSignal.active) LogGapPathSignalSessionSameThread("SIGNAL_CANCELLED");
    m_gapSignal = GapPathSignalSessionState{};
    m_gapInlet = GapPathSampleInletState{};
    m_gapQueue = GapPathSampleQueueState{};
    m_gapRecovery = GapPathQueueRecoveryState{};
    m_gapPending = GapPathPendingLowState{};
    if (m_gapTail.active && (reason == nullptr || std::strcmp(reason, "TAIL_SOURCE_DONE") != 0))
        LogGapPathTailSameThread("TAIL_CANCELLED");
    m_gapTail = GapPathTailState{};
    if (!m_pathHold.automaticEnabled && !m_pathHold.automaticHoldOwned &&
        !m_pathHold.automaticAdmissionOwned && !m_gapPath.active) return;
    LogPathCoreHoldAutomaticSameThread(reason);
    if (m_gapPath.active)
    {
        m_gapPath = GapPathSimulationState{};
        m_gapInput.Reset();
    }
    if (m_pathHold.automaticHoldOwned)
    {
        const auto gate = m_feedHoldResumeGate.GetSnapshot();
        if (gate.boundarySequence == m_pathHold.automaticBoundarySequence &&
            gate.dispatchId == m_pathHold.dispatch &&
            gate.executionEpoch == m_pathHold.identity.epoch &&
            gate.owner == m_pathHold.lease.owner && gate.ownerGeneration == m_pathHold.lease.generation)
            m_feedHoldResumeGate.Cancel(true);
    }
    // Every manual Start revokes CF before arming its own admission.
    if (m_pathHold.automaticAdmissionOwned)
        ClearHoldResumeAlarmAdmission(HoldResumeAdmissionKind::PROGRAM_HOLD);
    m_pathHold.automaticEnabled = false;
    m_pathHold.automaticHoldOwned = false;
    m_pathHold.automaticAdmissionOwned = false;
    m_pathHold.automaticIntervalMM = 0.0;
    m_pathHold.automaticIntervalPulse = 0.0;
    m_pathHold.automaticNextS = 0.0;
    m_pathHold.automaticObservedReturns = 0ULL;
    m_pathHold.automaticBoundarySequence = 0ULL;
    m_pathHold.automaticSettleSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
}

// CL: a separate endpoint boundary is mandatory even after operator takeover.
// This NC helper creates no velocity command and cannot release WAIT_RETURN.
NC_PATH_HOLD_NOINLINE
bool NCManager::ProcessPathCoreReturnWaitSameThread() noexcept
{
    if (!m_pathHold.bound || !m_pathHold.requireReturnAuthorization ||
        !m_pathHold.requested || !m_pathHold.startCommitted ||
        Close_System_Com_flag || AlarmManager::GetInstance().HasAlarm() ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || Homing.IsActive() || m_isG66Active || !m_macroStack.empty() ||
        m_pathHold.blocked || m_pathHold.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathHold.cache != GetBaseProgramCache().GetGeneration() ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathHold.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !IsPathCoreReplayConfigurationValid()) return false;
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == m_pathHold.dispatch;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == m_pathHold.dispatch;
    if (line == arc) return false;
    const auto snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (snapshot.publicationSequence == 0ULL || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        !snapshot.ownerLease.Matches(m_pathHold.lease) ||
        snapshot.phase != MotionPathCoreHoldExcursionPhase::WAIT_RETURN) return false;
    if (m_gapPath.active && m_gapPath.returnLowTest && m_gapPath.repeatedLowRetreat &&
        !ValidateGapPathRepeatedReturnLowSnapshotSameThread(snapshot)) return true;
    if (!snapshot.requireReturnAuthorization || !snapshot.crossSegment ||
        snapshot.historyCount != m_pathHoldView.completedCount || snapshot.cycleLimit != m_pathHold.cycleLimit ||
        snapshot.cycleLimit < 1U || snapshot.cycleLimit > 32U ||
        snapshot.returnCount >= snapshot.cycleLimit || snapshot.retreatCount != snapshot.returnCount + 1ULL)
    {
        RejectPathCoreHoldSameThread(15U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY, &snapshot);
        return true;
    }
    // CM: keep the endpoint generation across automatic or manual takeover.
    if (snapshot.retreatCount <= m_pathHold.returnHoldRetreatCount) return false;
    if (snapshot.retreatCount != m_pathHold.returnHoldRetreatCount + 1ULL ||
        snapshot.holdRequestSequence < m_pathHold.requestedHoldSequence)
    {
        RejectPathCoreHoldSameThread(15U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY, &snapshot);
        return true;
    }
    if (m_gapPath.active && !ServiceGapPathSimulationSameThread(0.0, true, "RETURN_HOLD", false, nullptr, &snapshot)) return true;
    if (m_state == NCState::RUN) FeedHoldInternal();
    else
    {
        // A HOLD/START pressed while retreating cannot preauthorize return.
        // Refresh the boundary while remaining HOLD; Begin cancels its old START.
        m_feedHoldNCSettleRequestSequence = m_motion.RequestFeedHoldNCSettle(
            m_motion.GetCurrentExecutionEpoch(), m_programMotionLease);
        BeginFeedHoldBoundaryShadow(NCFeedHoldSource::PROGRAM);
        m_motion.SetGroupFeedrateOverride(0.0);
        ObserveFeedHoldLegacyHoldShadow();
    }
    const auto boundary = m_feedHoldBoundaryShadow.GetSnapshot();
    if (m_state != NCState::HOLD || !IsProgramFeedHoldResumeCandidate() ||
        boundary.sequence == 0ULL || boundary.dispatchId != m_pathHold.dispatch ||
        boundary.requestExecutionEpoch != m_pathHold.identity.epoch ||
        boundary.requestOwner != m_pathHold.lease.owner ||
        boundary.requestOwnerGeneration != m_pathHold.lease.generation ||
        m_feedHoldNCSettleRequestSequence <= m_pathHold.requestedHoldSequence ||
        m_feedHoldNCSettleRequestSequence <= snapshot.holdRequestSequence ||
        (m_gapPath.repeatedReturnLow && m_feedHoldNCSettleRequestSequence <= m_gapPath.returnReholdSequence) ||
        boundary.expectedSettleRequestSequence != m_feedHoldNCSettleRequestSequence ||
        boundary.failed || boundary.cancelled)
    {
        RejectPathCoreHoldSameThread(15U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY, &snapshot);
        return true;
    }
    m_pathHold.returnHoldRequested = true;
    m_pathHold.returnHoldRetreatCount = snapshot.retreatCount;
    if (m_pathHold.automaticEnabled && m_gapPath.active && m_gapPath.lowRetreat)
    {
        m_pathHold.automaticBoundarySequence = boundary.sequence;
        m_pathHold.automaticSettleSequence = m_feedHoldNCSettleRequestSequence;
        m_pathHold.automaticHoldOwned = true;
        m_pathHold.automaticAdmissionOwned = false;
        m_gapPath.returnHold = true;
        m_gapPath.holdStartMs = m_gapPath.lastServiceMs;
        m_gapPath.ackLogged = false;
        m_gapPath.waitJ5Logged = false;
    }
    LogPathCoreHoldAutomaticSameThread("RETURN_HOLD_REQUESTED");
    if (m_gapPath.active && m_gapPath.repeatedLowRetreat && m_gapPath.returnHold &&
        m_pathHold.automaticObservedReturns == 1ULL && snapshot.retreatCount == 2ULL)
        LogGapPathSimulationSameThread("TEST_SECOND_ROUND_WAIT_30S");
    return true;
}

// CO keeps the source generation across rounds and retires each probe only
// after its exact completion. Phase 0 allows an already consumed publication.
NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathRepeatedReturnLowSnapshotSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    const std::uint64_t observed = m_pathHold.automaticObservedReturns;
    const bool complete = snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE;
    const bool returning = snapshot.phase == MotionPathCoreHoldExcursionPhase::RETURNING;
    if (snapshot.publicationSequence == 0ULL || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        !snapshot.ownerLease.Matches(m_pathHold.lease) || !snapshot.crossSegment ||
        !snapshot.requireReturnAuthorization || snapshot.cycleLimit != m_pathHold.cycleLimit ||
        snapshot.cycleLimit < 1U || snapshot.cycleLimit > 32U || observed >= snapshot.cycleLimit ||
        snapshot.historyCount != m_pathHoldView.completedCount || snapshot.activeOrdinal > snapshot.historyCount ||
        snapshot.requestGeneration == 0ULL ||
        (m_gapPath.returnGeneration != 0ULL && snapshot.requestGeneration != m_gapPath.returnGeneration) ||
        snapshot.returnCount < observed || snapshot.returnCount > observed + 1ULL ||
        snapshot.retreatCount < snapshot.returnCount || snapshot.retreatCount > observed + 1ULL ||
        snapshot.completedHoldRequestSequence < m_gapPath.returnCompletedFence)
    {
        RejectGapPathSimulationSameThread("RETURN_LOW_SCOPE_REVOKED");
        return false;
    }
    if (m_gapPath.repeatedReturnLow &&
        (m_gapPath.returnProbeLimit < 1U || m_gapPath.returnProbeLimit > 8U ||
            m_gapPath.returnProbeHoldCount > m_gapPath.returnProbeLimit ||
            m_gapPath.returnProbeResumeCount > m_gapPath.returnProbeHoldCount ||
            m_gapPath.returnProbeHoldCount > m_gapPath.returnProbeResumeCount + 1U ||
            (m_gapPath.returnRehold ?
                (m_gapPath.returnProbeHoldCount != m_gapPath.returnProbeResumeCount +
                    (m_gapPath.returnResumeApplied ? 0U : 1U)) :
                (m_gapPath.returnProbeHoldCount != m_gapPath.returnProbeResumeCount || m_gapPath.returnResumeApplied)) ||
            (m_gapPath.returnProbeHoldCount == 0U ? m_gapPath.returnReholdSequence != 0ULL :
                m_gapPath.returnReholdSequence <= m_pathHold.requestedHoldSequence) ||
            ((returning || (complete && snapshot.returnCount != observed)) &&
                (snapshot.publicationSequence < m_gapPath.returnWatchPublication ||
                    (complete && snapshot.publicationSequence == m_gapPath.returnWatchPublication)))))
    {
        RejectGapPathSimulationSameThread("RETURN_PROBE_STATE_OR_PUBLICATION");
        return false;
    }
    m_gapPath.returnGeneration = snapshot.requestGeneration;
    if (complete && snapshot.returnCount == observed)
    {
        // Motion may republish COMPLETE until it consumes the next start. Its
        // current J5 may change, but the retired completion fence cannot.
        const bool originalHold = snapshot.holdRequestSequence == m_gapPath.returnCompletedHold ||
            snapshot.holdRequestSequence == m_gapPath.returnCompletedRehold;
        const bool freshHold = snapshot.holdRequestSequence > m_gapPath.returnCompletedFence &&
            ((m_pathHold.automaticHoldOwned &&
                snapshot.holdRequestSequence == m_pathHold.automaticSettleSequence &&
                snapshot.holdRequestSequence == m_feedHoldNCSettleRequestSequence) ||
                (m_pathHold.requested && m_pathHold.startCommitted &&
                    snapshot.holdRequestSequence == m_pathHold.requestedHoldSequence));
        if (observed == 0ULL || m_gapPath.returnRoundPhase != 0U ||
            m_gapPath.returnWatchStarted || m_gapPath.returnLowInjected ||
            m_gapPath.returnRehold || m_gapPath.returnResumeApplied ||
            (m_gapPath.repeatedReturnLow && (m_gapPath.returnProbeHoldCount != 0U ||
                m_gapPath.returnProbeResumeCount != 0U || m_gapPath.returnWatchPublication != 0ULL)) ||
            snapshot.retreatCount != observed || snapshot.activeOrdinal != snapshot.historyCount ||
            m_gapPath.returnCompletedFence == 0ULL ||
            m_gapPath.returnCompletedHold == 0ULL ||
            m_gapPath.returnCompletedRehold <= m_gapPath.returnCompletedHold ||
            m_gapPath.returnCompletedFence < m_gapPath.returnCompletedRehold ||
            snapshot.completedHoldRequestSequence != m_gapPath.returnCompletedFence ||
            !std::isfinite(snapshot.heldS) || snapshot.returnedS != snapshot.heldS ||
            HoldDoubleBits(snapshot.returnedS) != m_gapPath.returnCompletedSBits ||
            (!originalHold && !freshHold))
        {
            RejectGapPathSimulationSameThread("RETURN_LOW_COMPLETION_REPLAY");
            return false;
        }
        return true;
    }
    if (returning || complete)
    {
        if (!m_pathHold.requested || !m_pathHold.startCommitted || !m_pathHold.returnHoldRequested ||
            m_pathHold.returnHoldRetreatCount != observed + 1ULL || snapshot.retreatCount != observed + 1ULL ||
            (snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence &&
                (!complete || snapshot.holdRequestSequence != m_gapPath.returnReholdSequence)) || snapshot.boundaryOnly ||
            m_pathHold.requestedHoldSequence <= m_gapPath.returnCompletedFence ||
            (!complete && (snapshot.returnCount != observed ||
                snapshot.completedHoldRequestSequence != m_gapPath.returnCompletedFence ||
                snapshot.completedHoldRequestSequence >= m_pathHold.requestedHoldSequence)))
        {
            RejectGapPathSimulationSameThread("RETURN_LOW_SCOPE_REVOKED");
            return false;
        }
        if (complete && (snapshot.returnCount != observed + 1ULL || m_gapPath.returnRoundPhase != 3U ||
            (m_gapPath.repeatedReturnLow && (m_gapPath.returnProbeHoldCount != m_gapPath.returnProbeLimit ||
                m_gapPath.returnProbeResumeCount != m_gapPath.returnProbeLimit)) ||
            !m_gapPath.returnLowInjected || !m_gapPath.returnRehold || !m_gapPath.returnResumeApplied ||
            m_gapPath.returnReholdSequence <= m_pathHold.requestedHoldSequence ||
            snapshot.completedHoldRequestSequence < m_gapPath.returnReholdSequence ||
            snapshot.activeOrdinal != snapshot.historyCount ||
            !std::isfinite(snapshot.heldS) || snapshot.returnedS != snapshot.heldS))
        {
            RejectGapPathSimulationSameThread("RETURN_LOW_NOT_COMPLETED");
            return false;
        }
        if (!complete) m_gapPath.returnRoundPhase = 3U;
        return true;
    }
    const bool waiting = snapshot.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN;
    const bool retreating = snapshot.phase == MotionPathCoreHoldExcursionPhase::RETREATING;
    const bool armed = snapshot.phase == MotionPathCoreHoldExcursionPhase::ARMED;
    const bool freshEndpointHold = waiting && m_pathHold.automaticHoldOwned &&
        m_pathHold.returnHoldRequested && m_pathHold.returnHoldRetreatCount == observed + 1ULL &&
        snapshot.holdRequestSequence == m_pathHold.automaticSettleSequence &&
        snapshot.holdRequestSequence == m_feedHoldNCSettleRequestSequence &&
        snapshot.holdRequestSequence > m_pathHold.requestedHoldSequence;
    if ((!waiting && !retreating && !armed) || m_gapPath.returnWatchStarted ||
        m_gapPath.returnLowInjected || m_gapPath.returnRehold || m_gapPath.returnResumeApplied ||
        (m_gapPath.repeatedReturnLow && (m_gapPath.returnProbeHoldCount != 0U ||
            m_gapPath.returnProbeResumeCount != 0U)) ||
        snapshot.returnCount != observed || snapshot.retreatCount != observed + (waiting ? 1ULL : 0ULL) ||
        snapshot.completedHoldRequestSequence != m_gapPath.returnCompletedFence ||
        m_gapPath.returnRoundPhase > (waiting ? 2U : 1U) ||
        ((waiting || retreating) && (!m_pathHold.requested || !m_pathHold.startCommitted ||
            m_pathHold.requestedHoldSequence <= m_gapPath.returnCompletedFence ||
            (snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence && !freshEndpointHold))))
    {
        RejectGapPathSimulationSameThread("RETURN_LOW_ROUND_REVOKED");
        return false;
    }
    m_gapPath.returnRoundPhase = waiting ? 2U : 1U;
    return true;
}

// CN is a single-cycle simulated returning-LOW probe. These checks are used
// both before sample publication and again at the actual resume admission.
NC_PATH_HOLD_NOINLINE
bool NCManager::ValidateGapPathReturnLowSnapshotSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!m_gapPath.active || !m_gapPath.returnLowTest) return true;
    if (m_gapPath.repeatedLowRetreat) return ValidateGapPathRepeatedReturnLowSnapshotSameThread(snapshot);
    if (!m_gapPath.returnWatchStarted && snapshot.phase != MotionPathCoreHoldExcursionPhase::RETURNING &&
        snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE) return true;
    const bool complete = snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE;
    if (snapshot.admissionPending || snapshot.publicationSequence == 0ULL || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration) ||
        !snapshot.ownerLease.Matches(m_pathHold.lease) || !snapshot.crossSegment ||
        !snapshot.requireReturnAuthorization || snapshot.cycleLimit != 1U ||
        snapshot.historyCount != m_pathHoldView.completedCount || snapshot.activeOrdinal > snapshot.historyCount ||
        snapshot.requestGeneration == 0ULL ||
        (m_gapPath.returnGeneration != 0ULL && snapshot.requestGeneration != m_gapPath.returnGeneration) ||
        !m_pathHold.requested || !m_pathHold.startCommitted || !m_pathHold.returnHoldRequested ||
        m_pathHold.returnHoldRetreatCount != 1ULL || snapshot.retreatCount != 1ULL ||
        snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence ||
        snapshot.boundaryOnly || (!complete && (snapshot.phase != MotionPathCoreHoldExcursionPhase::RETURNING ||
            snapshot.returnCount != 0ULL || snapshot.completedHoldRequestSequence >= m_pathHold.requestedHoldSequence)))
    {
        RejectGapPathSimulationSameThread("RETURN_LOW_SCOPE_REVOKED");
        return false;
    }
    if (complete && (!m_gapPath.returnLowInjected || !m_gapPath.returnRehold || !m_gapPath.returnResumeApplied ||
        m_gapPath.returnReholdSequence == 0ULL || snapshot.returnCount != 1ULL ||
        snapshot.completedHoldRequestSequence < m_gapPath.returnReholdSequence))
    {
        // A return too short to reach the probe is not a successful CN test.
        // A completion racing the new Hold must never launch another leg.
        RejectGapPathSimulationSameThread("RETURN_LOW_NOT_COMPLETED");
        return false;
    }
    if (!complete) m_gapPath.returnGeneration = snapshot.requestGeneration;
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ProcessGapPathReturnLowHoldSameThread(
    const MotionPathCoreHoldExcursionSnapshot& snapshot) noexcept
{
    if (!m_gapPath.active || !m_gapPath.returnLowTest || !m_gapPath.returnLowInjected ||
        m_gapPath.returnRehold || snapshot.phase != MotionPathCoreHoldExcursionPhase::RETURNING ||
        (m_gapPath.repeatedReturnLow && m_gapPath.returnProbeHoldCount >= m_gapPath.returnProbeLimit)) return false;
    const auto& gap = m_gapInput.Current();
    if (gap.quality != EDMGap::Quality::VALID || gap.source != EDMGap::Source::SIMULATED ||
        gap.band != EDMGap::Band::LOW || gap.pendingBand != EDMGap::Band::UNKNOWN ||
        gap.sequence != m_gapPath.sequence || !IsGapPathCurrentSampleProvenSameThread()) return false;
    if (m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE || m_feedHoldResumeGate.GetSnapshot().active)
    {
        RejectGapPathSimulationSameThread("RETURN_LOW_CONTROL_CONFLICT");
        return true;
    }
    FeedHoldInternal();
    const auto boundary = m_feedHoldBoundaryShadow.GetSnapshot();
    if (m_state != NCState::HOLD || !IsProgramFeedHoldResumeCandidate() ||
        boundary.sequence == 0ULL || boundary.dispatchId != m_pathHold.dispatch ||
        boundary.requestExecutionEpoch != m_pathHold.identity.epoch ||
        boundary.requestOwner != m_pathHold.lease.owner ||
        boundary.requestOwnerGeneration != m_pathHold.lease.generation ||
        m_feedHoldNCSettleRequestSequence <= snapshot.holdRequestSequence ||
        (m_gapPath.repeatedReturnLow && m_feedHoldNCSettleRequestSequence <= m_gapPath.returnReholdSequence) ||
        boundary.expectedSettleRequestSequence != m_feedHoldNCSettleRequestSequence ||
        boundary.failed || boundary.cancelled)
    {
        RejectGapPathSimulationSameThread("RETURN_LOW_HOLD_UNAVAILABLE");
        return true;
    }
    // Keep the original excursion request and committed start. This new J5
    // boundary only controls override resume on the already returning leg.
    m_pathHold.automaticBoundarySequence = boundary.sequence;
    m_pathHold.automaticSettleSequence = m_feedHoldNCSettleRequestSequence;
    m_pathHold.automaticHoldOwned = true;
    m_pathHold.automaticAdmissionOwned = false;
    m_gapPath.returnRehold = true;
    if (m_gapPath.repeatedReturnLow) ++m_gapPath.returnProbeHoldCount;
    m_gapPath.returnReholdSequence = m_feedHoldNCSettleRequestSequence;
    m_gapPath.holdStartMs = m_gapPath.lastServiceMs;
    m_gapPath.ackLogged = false;
    m_gapPath.waitJ5Logged = false;
    LogGapPathSimulationSameThread("RETURN_LOW_HOLD_30S");
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ProcessPathCoreHoldAutomaticSameThread() noexcept
{
    if (m_gapTail.active)
    {
        const auto snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
        const bool completed = (m_pathFeed.dispatch == m_gapTail.dispatch && m_pathFeed.completed) ||
            (m_pathArc.dispatch == m_gapTail.dispatch && m_pathArc.completed);
        if (completed) return !CompleteGapPathTailProofSameThread(snapshot);
        return !ServiceGapPathTailSameThread(snapshot);
    }
    if (ProcessPathCoreReturnWaitSameThread()) return true;
    if (!m_pathHold.automaticEnabled) return false;
    if (Close_System_Com_flag || AlarmManager::GetInstance().HasAlarm() ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || Homing.IsActive() ||
        m_isG66Active || !m_macroStack.empty() || m_isSingleBlockEnabled ||
        m_legacySingleBlockPausePending || !m_feedHoldResumeGate.IsEnabled() ||
        m_pathHold.blocked || !m_pathHold.crossSegment ||
        m_pathHold.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathHold.cache != GetBaseProgramCache().GetGeneration() ||
        !m_pathHold.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !IsPathCoreReplayConfigurationValid())
    {
        CancelPathCoreHoldAutomaticSameThread("SCOPE_REVOKED");
        return false;
    }
    if (!m_pathHold.bound)
    {
        if (m_gapSignal.active && m_gapSignal.handoffPending)
            return !ServiceGapPathSimulationSameThread(0.0, false, "CX_HANDOFF_WAIT");
        if (m_gapPath.active && m_gapPath.automaticResume &&
            (m_pathHold.automaticHoldOwned || (m_gapPath.repeating && m_pathHold.requested)))
            CancelPathCoreHoldAutomaticSameThread("SOURCE_UNBOUND");
        return false;
    }
    if (m_gapPath.active && m_gapPath.repeating &&
        ((m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.completed) ||
            (m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.completed)))
    {
        // Observe may have seen a temporary unavailable final publication.
        // Consume this scan so the generic callback cannot advance the source.
        ObservePathCoreHoldSameThread();
        return !(m_gapWindow.active && m_gapWindow.normalSource && m_gapWindow.normalProven);
    }
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == m_pathHold.dispatch;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == m_pathHold.dispatch;
    if (line == arc || m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch())
    {
        CancelPathCoreHoldAutomaticSameThread("SOURCE_REVOKED");
        return false;
    }
    const auto snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (snapshot.admissionPending)
    {
        if (!m_gapPath.active) return false;
        (void)ServiceGapPathAdmissionSameThread(snapshot);
        return true; // Only the real RT consumer can leave the pending phase.
    }
    // A bounded read can be unavailable during publication; it grants no new action.
    if (snapshot.publicationSequence == 0ULL)
    {
        if (m_gapPath.active && !ServiceGapPathSimulationSameThread(0.0, false, "NO_RT")) return true;
        return m_pathHold.automaticHoldOwned;
    }
    if (!HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration))
    {
        if (m_gapPath.active && !ServiceGapPathSimulationSameThread(0.0, false, "IDENTITY_WAIT")) return true;
        if (m_pathHold.automaticHoldOwned ||
            (m_gapPath.active && m_gapPath.repeating && m_pathHold.requested))
            CancelPathCoreHoldAutomaticSameThread("IDENTITY_REVOKED");
        return false; // The asynchronous Bind may not have been consumed yet.
    }
    if (!snapshot.ownerLease.Matches(m_pathHold.lease) || !snapshot.crossSegment ||
        snapshot.historyCount != m_pathHoldView.completedCount ||
        snapshot.cycleLimit != m_pathHold.cycleLimit ||
        snapshot.phase == MotionPathCoreHoldExcursionPhase::REJECTED ||
        snapshot.phase == MotionPathCoreHoldExcursionPhase::INVALIDATED)
    {
        CancelPathCoreHoldAutomaticSameThread("RT_REVOKED");
        return false;
    }
    if (m_gapWindow.active && m_gapWindow.normalSource)
    {
        // A failed observation consumes this scan after publishing its alarm.
        return !ServiceGapPathNormalSourceSameThread(snapshot);
    }
    if (m_gapPath.active && !ValidateGapPathLoadedPublicationSameThread(snapshot)) return true;
    // CK retains the session through the excursion. Historical activeS is
    // local to its current leg and may exceed the original source length.
    const bool gapReturnPending = m_gapPath.active && m_gapPath.repeating &&
        m_gapPath.held && (!m_pathHold.automaticHoldOwned || m_gapPath.returnHold) &&
        m_pathHold.requested && m_pathHold.startCommitted;
    if (m_gapPath.active)
    {
        if (!std::isfinite(snapshot.activeS) || snapshot.activeS < 0.0 ||
            (!gapReturnPending && snapshot.activeS > snapshot.lengthPulse) ||
            !std::isfinite(snapshot.lengthPulse) || snapshot.lengthPulse <= 0.0 ||
            !std::isfinite(m_pathHold.automaticNextS) ||
            (m_pathHold.automaticNextS <= 0.0 && !IsGapPathInitialStationAtSeamSameThread()))
        {
            RejectGapPathSimulationSameThread("PROGRESS_INVALID");
            return true;
        }
        if (!ValidateGapPathReturnLowSnapshotSameThread(snapshot)) return true;
        if (m_gapWindow.allowSeamStations && m_pathHold.automaticNextS == 0.0 && !m_gapPath.held)
        {
            if (snapshot.publicationSequence < m_gapWindow.seamPublication)
            {
                RejectGapPathSimulationSameThread("SEAM_PUBLICATION_REGRESSION", &snapshot);
                return true;
            }
            if (snapshot.publicationSequence == m_gapWindow.seamPublication)
            {
                (void)ServiceGapPathSimulationSameThread(m_gapInlet.active ? snapshot.activeS : 0.0, false,
                    "SEAM_PUBLICATION_WAIT", false, nullptr, m_gapInlet.active ? &snapshot : nullptr);
                return true;
            }
            m_gapWindow.seamPublication = snapshot.publicationSequence;
        }
        bool returningSample = snapshot.phase == MotionPathCoreHoldExcursionPhase::RETURNING;
        if (m_gapPath.repeatedReturnLow && returningSample)
        {
            if (snapshot.publicationSequence == m_gapPath.returnWatchPublication)
            {
                (void)ServiceGapPathSimulationSameThread(0.0, false, "RETURN_PUBLICATION_WAIT",
                    m_gapInlet.active && m_gapPath.returnWatchStarted, nullptr, m_gapInlet.active ? &snapshot : nullptr);
                return true;
            }
            m_gapPath.returnWatchPublication = snapshot.publicationSequence;
            double resumeS = 0.0;
            std::memcpy(&resumeS, &m_gapPath.returnWatchSBits, sizeof(resumeS));
            returningSample = m_state == NCState::RUN && !m_pathHold.automaticHoldOwned &&
                !m_pathHold.automaticAdmissionOwned &&
                (m_gapPath.returnWatchStarted || snapshot.activeOrdinal > m_gapPath.returnWatchOrdinal ||
                    (snapshot.activeOrdinal == m_gapPath.returnWatchOrdinal && snapshot.activeS > resumeS));
        }
        if (!ServiceGapPathSimulationSameThread(gapReturnPending ? 0.0 : snapshot.activeS,
            true, "AUTOMATIC", returningSample, nullptr, &snapshot)) return true;
    }
    if (m_pathHold.automaticHoldOwned)
    {
        const auto boundary = m_feedHoldBoundaryShadow.GetSnapshot();
        if (m_state != NCState::HOLD || !IsProgramFeedHoldResumeCandidate() ||
            boundary.sequence != m_pathHold.automaticBoundarySequence ||
            boundary.dispatchId != m_pathHold.dispatch || boundary.failed || boundary.cancelled ||
            boundary.acknowledgeLost || boundary.resumeApplied ||
            boundary.requestExecutionEpoch != m_pathHold.identity.epoch ||
            boundary.requestOwner != m_pathHold.lease.owner ||
            boundary.requestOwnerGeneration != m_pathHold.lease.generation ||
            boundary.expectedSettleRequestSequence != m_pathHold.automaticSettleSequence ||
            m_feedHoldNCSettleRequestSequence != m_pathHold.automaticSettleSequence)
        {
            CancelPathCoreHoldAutomaticSameThread("HOLD_REVOKED");
            return false;
        }
        if (!boundary.acknowledged || !boundary.motion.settleProofValid ||
            !boundary.motion.ncSettled ||
            boundary.motion.settleRequestSequence != m_pathHold.automaticSettleSequence)
        {
            if (m_gapPath.active && m_gapPath.automaticResume && !m_gapPath.waitJ5Logged)
            {
                m_gapPath.waitJ5Logged = true;
                LogGapPathSimulationSameThread("WAIT_J5");
            }
            return true;
        }
        if (m_gapPath.active)
        {
            if (!m_gapPath.ackLogged)
            {
                m_gapPath.ackLogged = true;
                LogGapPathSimulationSameThread(m_gapPath.lowRetreat && !m_gapPath.returnHold ?
                    "STOP_CONFIRMED_LOW_RETREAT" : (m_gapPath.automaticResume ?
                        "STOP_CONFIRMED_WAIT_NORMAL" : "STOP_CONFIRMED_WAIT_START"));
            }
            if (!m_gapPath.automaticResume) return true;
            // Diagnostic latches never grant admission: require this scan's
            // actual, fully qualified sample and the exact owned J5 boundary.
            if (!IsGapPathAutomaticResumeSignalSameThread() ||
                !IsGapPathCurrentSampleProvenSameThread()) return true;
            if (snapshot.boundaryOnly)
            {
                RejectGapPathSimulationSameThread("SOURCE_BOUNDARY_NO_EXCURSION");
                return true;
            }
        }
        if (!m_pathHold.automaticAdmissionOwned)
        {
            // Never adopt an admission or button ticket created by another source.
            if (m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE ||
                m_feedHoldResumeGate.GetSnapshot().active)
            {
                CancelPathCoreHoldAutomaticSameThread("RESUME_CONFLICT");
                return false;
            }
            if (!ArmHoldResumeAlarmAdmission(HoldResumeAdmissionKind::PROGRAM_HOLD)) return true;
            m_pathHold.automaticAdmissionOwned = true;
            ObserveFeedHoldResumeRequestedShadow();
            const auto result = m_feedHoldResumeGate.RequestResume(m_feedHoldBoundaryShadow.GetSnapshot());
            if (result != NCFeedHoldResumeGateRequestResult::APPLY_NOW &&
                result != NCFeedHoldResumeGateRequestResult::DEFERRED)
            {
                CancelPathCoreHoldAutomaticSameThread("RESUME_REJECTED");
                return true;
            }
            m_holdResumeGateControlled = true;
            LogPathCoreHoldAutomaticSameThread("RESUME_REQUESTED");
        }
        if (m_feedHoldResumeGate.ShouldApplyResume() && ApplyProgramHoldResume(true))
        {
            LogPathCoreHoldAutomaticSameThread("RESUME_APPLIED");
            if (m_gapPath.repeatedReturnLow && m_gapPath.returnHold)
            {
                // The next watch starts only after a newer publication proves
                // movement beyond this resume's stopped geometry. HOLD time is excluded.
                m_gapPath.returnWatchPublication = snapshot.publicationSequence;
                m_gapPath.returnWatchSBits = HoldDoubleBits(snapshot.activeS);
                m_gapPath.returnWatchOrdinal = snapshot.activeOrdinal;
            }
            if (m_gapPath.returnLowTest && m_gapPath.returnRehold)
            {
                m_gapPath.returnResumeApplied = true;
                if (m_gapPath.repeatedReturnLow) ++m_gapPath.returnProbeResumeCount;
                LogGapPathSimulationSameThread("RETURN_RESUME_APPLIED");
                if (m_gapPath.repeatedReturnLow && m_gapPath.returnProbeResumeCount < m_gapPath.returnProbeLimit)
                {
                    // Retain the original return authorization and latest stop fence.
                    // Only this same return is re-probed; no excursion is requested.
                    m_gapPath.returnWatchStartMs = 0ULL;
                    m_gapPath.returnWatchStarted = false;
                    m_gapPath.returnLowInjected = false;
                    m_gapPath.returnRehold = false;
                    m_gapPath.returnResumeApplied = false;
                    LogGapPathSimulationSameThread("RETURN_PROBE_REARMED");
                }
            }
            if (m_gapPath.active && m_gapPath.automaticResume && !m_gapPath.repeating)
            {
                // Consume only the simulated automatic authority. The staged
                // original-identity excursion must survive until its return.
                CancelPathCoreHoldAutomaticSameThread("SESSION_CONSUMED");
                return true;
            }
            m_pathHold.automaticHoldOwned = false;
            m_pathHold.automaticAdmissionOwned = false;
            m_pathHold.automaticBoundarySequence = 0ULL;
            m_pathHold.automaticSettleSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
        }
        // BUSY preserves exact ownership, so the next operator Hold can cancel it.
        return true;
    }
    if (m_state != NCState::RUN)
    {
        CancelPathCoreHoldAutomaticSameThread("EXTERNAL_HOLD");
        return false;
    }
    if (ProcessGapPathReturnLowHoldSameThread(snapshot)) return true;
    // CF FIX1: an applied Resume is not a completed excursion. The RT
    // consumer can still publish ARMED during launch dwell, or the previous
    // COMPLETE before consuming a repeated start. Only the actual return
    // completion fence retires our last accepted excursion request.
    if (m_pathHold.requested &&
        (snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE ||
            snapshot.completedHoldRequestSequence < m_pathHold.requestedHoldSequence)) return false;
    if (snapshot.phase != MotionPathCoreHoldExcursionPhase::ARMED &&
        snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE) return false;
    if (snapshot.activeOrdinal != snapshot.historyCount || snapshot.retreatCount != snapshot.returnCount)
        return false;
    if (snapshot.returnCount != m_pathHold.automaticObservedReturns)
    {
        if (m_gapPath.active && m_gapPath.repeating &&
            (!gapReturnPending || m_pathHold.requestedHoldSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
                snapshot.completedHoldRequestSequence < m_pathHold.requestedHoldSequence ||
                (snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence &&
                    !(m_gapPath.returnLowTest && m_gapPath.repeatedLowRetreat &&
                        snapshot.holdRequestSequence == m_gapPath.returnReholdSequence)) ||
                snapshot.returnCount > m_pathHold.cycleLimit ||
                !std::isfinite(snapshot.heldS) || snapshot.returnedS != snapshot.heldS ||
                !IsGapPathAutomaticNormalSameThread() ||
                !IsGapPathCurrentSampleProvenSameThread()))
        {
            RejectGapPathSimulationSameThread("RETURN_NOT_PROVEN_OR_NORMAL");
            return true;
        }
        if (snapshot.returnCount != m_pathHold.automaticObservedReturns + 1ULL ||
            snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE ||
            !std::isfinite(snapshot.returnedS) || snapshot.returnedS < 0.0 ||
            snapshot.returnedS > snapshot.lengthPulse)
        {
            if (m_gapPath.active && m_gapPath.repeating)
            {
                RejectGapPathSimulationSameThread("RETURN_COUNT_OR_POSITION");
                return true;
            }
            CancelPathCoreHoldAutomaticSameThread("RETURN_REVOKED");
            return false;
        }
        if (m_gapPath.returnLowTest && m_gapPath.repeatedLowRetreat)
        {
            m_gapPath.returnCompletedFence = snapshot.completedHoldRequestSequence;
            m_gapPath.returnCompletedHold = m_pathHold.requestedHoldSequence;
            m_gapPath.returnCompletedRehold = m_gapPath.returnReholdSequence;
            m_gapPath.returnCompletedSBits = HoldDoubleBits(snapshot.returnedS);
        }
        m_pathHold.automaticObservedReturns = snapshot.returnCount;
        if (m_gapWindow.active && m_gapWindow.multipleStationsPerSource)
        {
            if (!IsGapPathSourceWindowScopeValidSameThread() ||
                snapshot.returnCount > m_gapWindow.cycleLimit)
            {
                RejectGapPathSimulationSameThread("WINDOW_RETURN_STATION_SCOPE");
                return true;
            }
            m_pathHold.automaticNextS = 0.0;
            if (snapshot.returnCount < m_gapWindow.cycleLimit)
            {
                const double targetMM = m_gapWindow.intervalMM *
                    static_cast<double>(m_gapWindow.completedStations + snapshot.returnCount + 1ULL);
                const double localMM = targetMM - m_gapWindow.completedForwardMM;
                if (ClassifyGapPathStationSameThread(m_gapWindow.completedForwardMM,
                    m_pathHoldView.original.lengthMM, m_pathHoldView.original.lengthPulse, targetMM) != 1)
                {
                    RejectGapPathSimulationSameThread("WINDOW_NEXT_STATION_INVALID");
                    return true;
                }
                m_pathHold.automaticNextS = localMM *
                    (m_pathHoldView.original.lengthPulse / m_pathHoldView.original.lengthMM);
                if (!std::isfinite(m_pathHold.automaticNextS) ||
                    m_pathHold.automaticNextS <= snapshot.returnedS ||
                    m_pathHold.automaticNextS >= snapshot.lengthPulse)
                {
                    RejectGapPathSimulationSameThread("FIXED_STATION_ALREADY_PASSED_OR_END");
                    return true;
                }
            }
            LogGapPathSourceWindowSameThread("SOURCE_RETURN_PROVEN");
        }
        else m_pathHold.automaticNextS = snapshot.returnedS + m_pathHold.automaticIntervalPulse;
        LogPathCoreHoldAutomaticSameThread("RETURNED");
        if (m_gapPath.active && m_gapPath.repeating)
        {
            if (snapshot.returnCount >= m_pathHold.cycleLimit)
            {
                if (m_gapWindow.active)
                {
                    if (!CompleteGapPathSourceBudgetSameThread(snapshot)) return true;
                }
                else CancelPathCoreHoldAutomaticSameThread("BUDGET_DONE");
                return false;
            }
            if (!std::isfinite(m_pathHold.automaticNextS) ||
                m_pathHold.automaticNextS <= snapshot.returnedS ||
                m_pathHold.automaticNextS >= snapshot.lengthPulse)
            {
                RejectGapPathSimulationSameThread("SOURCE_REMAINING_INSUFFICIENT");
                return true;
            }
            // Preserve the monitor, sequence, clock and accepted Motion fence.
            // Only this proven return may arm a new simulated LOW station.
            m_gapPath.lowInjected = false;
            m_gapPath.returnHold = false;
            m_gapPath.held = false;
            m_gapPath.holdStartMs = 0ULL;
            m_gapPath.recoveryInjected = false;
            m_gapPath.normalLogged = false;
            m_gapPath.recoveryLogged = false;
            m_gapPath.ackLogged = false;
            m_gapPath.waitJ5Logged = false;
            if (m_gapPath.returnLowTest && m_gapPath.repeatedLowRetreat)
            {
                m_gapPath.returnWatchStartMs = 0ULL;
                m_gapPath.returnReholdSequence = 0ULL;
                m_gapPath.returnWatchStarted = false;
                m_gapPath.returnLowInjected = false;
                m_gapPath.returnRehold = false;
                m_gapPath.returnResumeApplied = false;
                m_gapPath.returnRoundPhase = 0U;
                m_gapPath.returnProbeHoldCount = 0U;
                m_gapPath.returnProbeResumeCount = 0U;
                m_gapPath.returnWatchPublication = 0ULL;
                m_gapPath.returnWatchSBits = 0ULL;
                m_gapPath.returnWatchOrdinal = 0U;
            }
            LogPathCoreHoldAutomaticSameThread("REARMED");
            return false;
        }
        if (!std::isfinite(m_pathHold.automaticNextS) || m_pathHold.automaticNextS <= snapshot.returnedS)
        {
            CancelPathCoreHoldAutomaticSameThread("INTERVAL_UNAVAILABLE");
            return false;
        }
    }
    if (snapshot.returnCount >= m_pathHold.cycleLimit)
    {
        if (m_gapPath.active && m_gapPath.repeating)
        {
            RejectGapPathSimulationSameThread("BUDGET_WITHOUT_RETURN_PROOF");
            return true;
        }
        CancelPathCoreHoldAutomaticSameThread("BUDGET_DONE");
        return false;
    }
    if (!std::isfinite(snapshot.activeS) || !std::isfinite(snapshot.lengthPulse) ||
        snapshot.activeS < 0.0 || snapshot.lengthPulse <= 0.0 ||
        !std::isfinite(m_pathHold.automaticNextS) ||
        (m_pathHold.automaticNextS <= 0.0 && !IsGapPathInitialStationAtSeamSameThread()))
    {
        CancelPathCoreHoldAutomaticSameThread("PROGRESS_INVALID");
        return false;
    }
    if (m_pathHold.automaticNextS >= snapshot.lengthPulse || snapshot.activeS >= snapshot.lengthPulse)
    {
        if (m_gapPath.active)
        {
            RejectGapPathSimulationSameThread("SOURCE_END_BEFORE_LOW");
            return true;
        }
        CancelPathCoreHoldAutomaticSameThread("SOURCE_END");
        return false;
    }
    if (m_gapPath.active)
    {
        const EDMGap::Snapshot& gap = m_gapInput.Current();
        if (!m_gapPath.lowInjected || gap.quality != EDMGap::Quality::VALID ||
            gap.band != EDMGap::Band::LOW || gap.source != EDMGap::Source::SIMULATED ||
            !IsGapPathCurrentSampleProvenSameThread()) return false;
    }
    else if (snapshot.activeS < m_pathHold.automaticNextS) return false;
    if (m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE ||
        m_feedHoldResumeGate.GetSnapshot().active)
    {
        CancelPathCoreHoldAutomaticSameThread("CONTROL_CONFLICT");
        return false;
    }
    FeedHoldInternal();
    const auto boundary = m_feedHoldBoundaryShadow.GetSnapshot();
    if (m_state != NCState::HOLD || !IsProgramFeedHoldResumeCandidate() ||
        boundary.sequence == 0ULL || boundary.dispatchId != m_pathHold.dispatch ||
        boundary.requestExecutionEpoch != m_pathHold.identity.epoch ||
        boundary.requestOwner != m_pathHold.lease.owner ||
        boundary.requestOwnerGeneration != m_pathHold.lease.generation ||
        m_feedHoldNCSettleRequestSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
        boundary.expectedSettleRequestSequence != m_feedHoldNCSettleRequestSequence ||
        boundary.failed || boundary.cancelled)
    {
        CancelPathCoreHoldAutomaticSameThread("HOLD_UNAVAILABLE");
        return true;
    }
    m_pathHold.automaticBoundarySequence = boundary.sequence;
    m_pathHold.automaticSettleSequence = m_feedHoldNCSettleRequestSequence;
    m_pathHold.automaticHoldOwned = true;
    if (m_gapPath.active)
    {
        m_gapPath.held = true;
        m_gapPath.holdStartMs = m_gapPath.lastServiceMs;
    }
    LogPathCoreHoldAutomaticSameThread("HOLD_REQUESTED");
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogPathCoreHoldAutomaticSameThread(const char* phase) const noexcept
{
    if (m_gapPath.active)
    {
        LogGapPathSimulationSameThread(phase);
        return;
    }
    RtPrintf("[PCORE-CF] mode=DRY_RUN run=%llu dispatch=%llu phase=%s pc=%d line=%d enabled=%u owned=%u admission=%u Q_BITS=%016llX nextS=%016llX returned=%llu L=%u boundary=%llu hold=%llu\n",
        static_cast<unsigned long long>(m_pathHold.run), static_cast<unsigned long long>(m_pathHold.dispatch),
        phase, m_pathHold.sourcePC, m_pathHold.sourceLine, m_pathHold.automaticEnabled ? 1U : 0U,
        m_pathHold.automaticHoldOwned ? 1U : 0U, m_pathHold.automaticAdmissionOwned ? 1U : 0U,
        static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticIntervalMM)),
        static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)),
        static_cast<unsigned long long>(m_pathHold.automaticObservedReturns), static_cast<unsigned int>(m_pathHold.cycleLimit),
        static_cast<unsigned long long>(m_pathHold.automaticBoundarySequence),
        static_cast<unsigned long long>(m_pathHold.automaticSettleSequence));
}

NC_PATH_HOLD_NOINLINE
bool NCManager::PreparePathCoreHoldResumeSameThread(bool gateControlled) noexcept
{
    if (!ValidateGapPathAutomaticResumeSameThread()) return false;
    if (!m_pathHold.bound) return true;
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == m_pathHold.dispatch;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == m_pathHold.dispatch;
    const bool completed = (m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.completed) ||
        (m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.completed);
    if (completed)
    {
        if (m_gapPath.active && m_gapPath.automaticResume && m_pathHold.automaticHoldOwned)
        {
            RejectGapPathSimulationSameThread("SOURCE_COMPLETED_BEFORE_RESUME");
            return false;
        }
        LogPathCoreHoldSameThread(m_pathHold.requested ? "SOURCE_COMPLETED" : "UNUSED_COMPLETED");
        InvalidatePathCoreHoldSameThread();
        return true;
    }
    const NCFeedHoldBoundarySnapshot hold = m_feedHoldBoundaryShadow.GetSnapshot();
    if (!gateControlled || !IsProgramFeedHoldResumeCandidate() || m_state != NCState::HOLD ||
        !hold.acknowledged || hold.failed || hold.cancelled || hold.acknowledgeLost ||
        hold.dispatchId != m_pathHold.dispatch || hold.requestExecutionEpoch != m_pathHold.identity.epoch ||
        hold.expectedSettleRequestSequence != m_feedHoldNCSettleRequestSequence ||
        !hold.motion.settleProofValid || !hold.motion.ncSettled ||
        hold.motion.settleRequestSequence != m_feedHoldNCSettleRequestSequence ||
        hold.requestOwner != m_pathHold.lease.owner || hold.requestOwnerGeneration != m_pathHold.lease.generation ||
        !m_pathHold.lease.Matches(m_programMotionLease) || !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() || line == arc ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        !IsPathCoreReplayConfigurationValid())
    {
        if (!m_pathHold.blocked) { m_pathHold.code = 10U; LogPathCoreHoldSameThread("RESUME_BLOCKED"); }
        m_pathHold.blocked = true;
        return false;
    }
    const MotionExecutionIdentity& receiptIdentity = line ? m_pathFeedMotion.receipt.identity : m_pathArcMotion.receipt.identity;
    if (!HoldIdentityEqual(receiptIdentity, m_pathHold.identity))
    {
        if (!m_pathHold.blocked) { m_pathHold.code = 11U; LogPathCoreHoldSameThread("RESUME_BLOCKED"); }
        m_pathHold.blocked = true;
        return false;
    }
    const MotionPathCoreHoldExcursionSnapshot snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (snapshot.admissionPending || !HoldTranslationIdentityEqual(snapshot, m_pathHold.identity, CoordSys, m_pathHoldView.translationGeneration)) return false; // Admission is no stop proof.
    if (!snapshot.ownerLease.Matches(m_pathHold.lease) ||
        snapshot.cycleLimit != m_pathHold.cycleLimit || snapshot.cycleLimit < 1U || snapshot.cycleLimit > 32U ||
        snapshot.requireReturnAuthorization != m_pathHold.requireReturnAuthorization)
    {
        if (!m_pathHold.blocked) { m_pathHold.code = 11U; LogPathCoreHoldSameThread("RESUME_BLOCKED"); }
        m_pathHold.blocked = true;
        return false;
    }
    const bool returnStart = m_pathHold.requireReturnAuthorization && m_pathHold.returnHoldRequested &&
        m_pathHold.requested && snapshot.requireReturnAuthorization &&
        snapshot.phase == MotionPathCoreHoldExcursionPhase::WAIT_RETURN &&
        snapshot.returnCount < snapshot.cycleLimit&& snapshot.retreatCount == snapshot.returnCount + 1ULL &&
        m_pathHold.returnHoldRetreatCount == snapshot.retreatCount;
    const bool initial = !m_pathHold.requested && snapshot.phase == MotionPathCoreHoldExcursionPhase::ARMED;
    const bool repeat = m_pathHold.requested && snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE &&
        snapshot.returnCount < m_pathHold.cycleLimit&&
        m_feedHoldNCSettleRequestSequence > snapshot.completedHoldRequestSequence;
    if ((initial || repeat) && !m_pathHold.blocked && snapshot.boundaryOnly &&
        snapshot.holdRequestSequence == m_feedHoldNCSettleRequestSequence)
    {
        if (m_gapPath.active && m_gapPath.automaticResume && m_pathHold.automaticHoldOwned)
        {
            RejectGapPathSimulationSameThread("SOURCE_BOUNDARY_NO_EXCURSION");
            return false;
        }
        // A freshly proven stop exactly at the source boundary has no
        // interior excursion. Expire the option and preserve ordinary resume.
        m_pathHold.code = 14U;
        LogPathCoreHoldSameThread("UNUSED_BOUNDARY");
        InvalidatePathCoreHoldSameThread();
        return true;
    }
    if (snapshot.phase == MotionPathCoreHoldExcursionPhase::REJECTED ||
        snapshot.phase == MotionPathCoreHoldExcursionPhase::INVALIDATED || m_pathHold.blocked)
    {
        if (!m_pathHold.blocked) { m_pathHold.code = 12U; LogPathCoreHoldSameThread("RT_BLOCKED"); }
        m_pathHold.blocked = true;
        return false;
    }
    if (m_pathHold.requested && snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE && !returnStart)
    {
        // A Hold inside the current excursion resumes that leg. It cannot
        // consume a later repetition, including the pending launch dwell.
        return snapshot.phase == MotionPathCoreHoldExcursionPhase::RETREATING ||
            snapshot.phase == MotionPathCoreHoldExcursionPhase::RETURNING ||
            snapshot.phase == MotionPathCoreHoldExcursionPhase::ARMED;
    }
    if (m_pathHold.requested && snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE &&
        snapshot.returnCount >= m_pathHold.cycleLimit) return true; // Budget exhausted: ordinary resume.
    // A Hold allocated during the last return-completion pass belongs to
    // that completed excursion. Resume its continuation without spending
    // another repetition; only a later sequence may stage a fresh loop.
    if (m_pathHold.requested && snapshot.phase == MotionPathCoreHoldExcursionPhase::COMPLETE &&
        m_feedHoldNCSettleRequestSequence != MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
        m_feedHoldNCSettleRequestSequence <= snapshot.completedHoldRequestSequence) return true;
    if ((!initial && !repeat && !returnStart) || !snapshot.ready ||
        snapshot.holdRequestSequence != m_feedHoldNCSettleRequestSequence ||
        !m_motion.RequestPathCoreHoldExcursion(m_pathHold.identity,
            m_pathHold.lease, m_feedHoldNCSettleRequestSequence)) return false;
    if (!returnStart) m_pathHold.returnHoldRequested = false;
    m_pathHold.requested = true;
    m_pathHold.requestedHoldSequence = m_feedHoldNCSettleRequestSequence;
    m_pathHold.startCommitted = false; // Reset only after a new request was successfully staged.
    m_pathHold.code = 1U;
    LogPathCoreHoldSameThread("REQUESTED");
    return true;
}

NC_PATH_HOLD_NOINLINE
bool NCManager::CommitPathCoreHoldResumeSameThread() noexcept
{
    if (!ValidateGapPathAutomaticResumeSameThread()) return false;
    if (!m_pathHold.bound || !m_pathHold.requested) return true;
    if (m_state != NCState::RUN || AlarmManager::GetInstance().HasAlarm() ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathHold.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests()) return false;
    if (m_pathHold.startCommitted) return true;
    if (m_pathHold.requestedHoldSequence != m_feedHoldNCSettleRequestSequence ||
        !m_motion.CommitPathCoreHoldExcursion(m_pathHold.identity, m_pathHold.lease,
            m_pathHold.requestedHoldSequence)) return false;
    m_pathHold.startCommitted = true;
    LogPathCoreHoldSameThread("RESUME_COMMITTED");
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogPathCoreHoldSameThread(const char* phase) const noexcept
{
    const PathHoldState& s = m_pathHold;
    RtPrintf("[PCORE-CB] run=%llu dispatch=%llu phase=%s code=%u pc=%d line=%d commit=%llu armed=%u bound=%u requested=%u blocked=%u L=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
        static_cast<unsigned int>(s.code), s.sourcePC, s.sourceLine, static_cast<unsigned long long>(s.commit),
        s.armed ? 1U : 0U, s.bound ? 1U : 0U, s.requested ? 1U : 0U, s.blocked ? 1U : 0U,
        static_cast<unsigned int>(s.cycleLimit));
    RtPrintf("[PCORE-CB-ID] run=%llu dispatch=%llu epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u D=%016llX F=%016llX\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(s.identity.epoch), static_cast<unsigned long long>(s.identity.segmentId),
        static_cast<unsigned int>(s.identity.sourceBlockId), static_cast<unsigned int>(s.identity.source),
        static_cast<unsigned int>(s.lease.owner), static_cast<unsigned int>(s.lease.generation),
        static_cast<unsigned long long>(HoldDoubleBits(s.distanceMM)), static_cast<unsigned long long>(HoldDoubleBits(s.feedMMMin)));
    if (s.crossSegment)
        RtPrintf("[PCORE-CD] run=%llu dispatch=%llu phase=%s P=1 history=%u validMask=%u\n",
            static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
            static_cast<unsigned int>(m_pathHoldView.completedCount),
            static_cast<unsigned int>(m_pathHoldView.validAxisMask));
    if (!s.bound) return;
    const MotionPathCoreHoldExcursionSnapshot rt = m_motion.GetPathCoreHoldExcursionSnapshot();
    if (!HoldIdentityEqual(rt.identity, s.identity)) return;
    RtPrintf("[PCORE-CB-RT] run=%llu dispatch=%llu seq=%llu phase=%u reason=%u ready=%u boundary=%u hold=%llu retreat=%llu return=%llu heldS=%016llX retreatS=%016llX returnedS=%016llX length=%016llX L=%u fence=%llu generation=%llu\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(rt.transitionSequence), static_cast<unsigned int>(rt.phase),
        static_cast<unsigned int>(rt.reason), rt.ready ? 1U : 0U, rt.boundaryOnly ? 1U : 0U,
        static_cast<unsigned long long>(rt.holdRequestSequence), static_cast<unsigned long long>(rt.retreatCount),
        static_cast<unsigned long long>(rt.returnCount), static_cast<unsigned long long>(HoldDoubleBits(rt.heldS)),
        static_cast<unsigned long long>(HoldDoubleBits(rt.retreatS)), static_cast<unsigned long long>(HoldDoubleBits(rt.returnedS)),
        static_cast<unsigned long long>(HoldDoubleBits(rt.lengthPulse)), static_cast<unsigned int>(rt.cycleLimit),
        static_cast<unsigned long long>(rt.completedHoldRequestSequence),
        static_cast<unsigned long long>(rt.requestGeneration));
    if (rt.crossSegment)
        RtPrintf("[PCORE-CD-RT] run=%llu dispatch=%llu seq=%llu phase=%u reason=%u history=%u ordinal=%u retreatOrdinal=%u seams=%llu activeS=%016llX retreatLocalS=%016llX\n",
            static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
            static_cast<unsigned long long>(rt.transitionSequence), static_cast<unsigned int>(rt.phase),
            static_cast<unsigned int>(rt.reason), static_cast<unsigned int>(rt.historyCount),
            static_cast<unsigned int>(rt.activeOrdinal), static_cast<unsigned int>(rt.retreatOrdinal),
            static_cast<unsigned long long>(rt.seamCount), static_cast<unsigned long long>(HoldDoubleBits(rt.activeS)),
            static_cast<unsigned long long>(HoldDoubleBits(rt.retreatLocalS)));
}

NC_PATH_HOLD_NOINLINE
void NCManager::CapturePathCoreHoldFaultSameThread(std::uint32_t code, int alarmCode,
    std::uint32_t origin, const MotionPathCoreHoldExcursionSnapshot* observed) noexcept
{
    PathHoldLastFault& fault = m_pathHoldLastFault;
    fault = PathHoldLastFault{};
    const bool active = m_pathHold.armed || m_pathHold.bound;
    fault.run = active && m_pathHold.run != 0ULL ? m_pathHold.run : m_pathCoreLiveBookkeeping.currentRunToken;
    fault.dispatch = active && m_pathHold.dispatch != 0ULL ? m_pathHold.dispatch : m_currentExecutingBlockDispatchId;
    if (active)
    {
        fault.identity = m_pathHold.identity;
        fault.lease = m_pathHold.lease;
        fault.sourcePC = m_pathHold.sourcePC;
        fault.sourceLine = m_pathHold.sourceLine;
    } // Otherwise this is the current control rejection, not a stale source.

    fault.code = code;
    fault.alarmCode = alarmCode;
    fault.origin = origin; // 1=CB_REJECT, 2=ALARM_INVALIDATION (alarm number unavailable).
    if (active && observed != nullptr) fault.snapshot = *observed;
    else if (m_pathHold.bound) fault.snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    // A default snapshot is an unavailable/contended read, not RT reason 0.
    fault.rtValid = fault.snapshot.publicationSequence != 0ULL && fault.snapshot.identity.IsAssigned();
    fault.identityMatch = fault.rtValid && HoldIdentityEqual(fault.snapshot.identity, fault.identity);
    fault.leaseMatch = fault.rtValid && fault.snapshot.ownerLease.Matches(fault.lease);
    fault.present = true;
    LogPathCoreHoldLastFaultSameThread();
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogPathCoreHoldLastFaultSameThread() const noexcept
{
    const PathHoldLastFault& f = m_pathHoldLastFault;
    if (!f.present) return;
    RtPrintf("[PCORE-CB-LAST-FAULT] run=%llu dispatch=%llu origin=%u code=%u alarm=%d pc=%d line=%d rtValid=%u identityMatch=%u leaseMatch=%u\n",
        static_cast<unsigned long long>(f.run), static_cast<unsigned long long>(f.dispatch),
        static_cast<unsigned int>(f.origin), static_cast<unsigned int>(f.code), f.alarmCode,
        f.sourcePC, f.sourceLine, f.rtValid ? 1U : 0U, f.identityMatch ? 1U : 0U, f.leaseMatch ? 1U : 0U);
    RtPrintf("[PCORE-CB-LAST-FAULT-ID] epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u rtEpoch=%llu rtSeg=%llu rtPC=%u rtSource=%u rtOwner=%u rtGen=%u\n",
        static_cast<unsigned long long>(f.identity.epoch), static_cast<unsigned long long>(f.identity.segmentId),
        static_cast<unsigned int>(f.identity.sourceBlockId), static_cast<unsigned int>(f.identity.source),
        static_cast<unsigned int>(f.lease.owner), static_cast<unsigned int>(f.lease.generation),
        static_cast<unsigned long long>(f.snapshot.identity.epoch), static_cast<unsigned long long>(f.snapshot.identity.segmentId),
        static_cast<unsigned int>(f.snapshot.identity.sourceBlockId), static_cast<unsigned int>(f.snapshot.identity.source),
        static_cast<unsigned int>(f.snapshot.ownerLease.owner), static_cast<unsigned int>(f.snapshot.ownerLease.generation));
    if (!f.rtValid) return;
    RtPrintf("[PCORE-CB-LAST-FAULT-RT] pub=%llu seq=%llu phase=%u reason=%u hold=%llu retreat=%llu return=%llu heldS=%016llX retreatS=%016llX returnedS=%016llX length=%016llX L=%u fence=%llu generation=%llu\n",
        static_cast<unsigned long long>(f.snapshot.publicationSequence), static_cast<unsigned long long>(f.snapshot.transitionSequence),
        static_cast<unsigned int>(f.snapshot.phase), static_cast<unsigned int>(f.snapshot.reason),
        static_cast<unsigned long long>(f.snapshot.holdRequestSequence), static_cast<unsigned long long>(f.snapshot.retreatCount),
        static_cast<unsigned long long>(f.snapshot.returnCount), static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.heldS)),
        static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.retreatS)), static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.returnedS)),
        static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.lengthPulse)),
        static_cast<unsigned int>(f.snapshot.cycleLimit),
        static_cast<unsigned long long>(f.snapshot.completedHoldRequestSequence),
        static_cast<unsigned long long>(f.snapshot.requestGeneration));
    if (f.snapshot.admissionPending || m_gapServiceLastFault.admissionSeen)
        RtPrintf("[PCORE-ADMISSION-LAST-FAULT] pending=%u tick=%llu axis=%d error=%016llX window=%016llX\n",
            f.snapshot.admissionPending ? 1U : 0U,
            static_cast<unsigned long long>(f.snapshot.admissionWaitTick), static_cast<int>(f.snapshot.admissionWaitAxis),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.admissionFollowingError)),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.admissionWindowPulse)));
    if (f.snapshot.admissionPending || m_gapServiceLastFault.admissionSeen)
        RtPrintf("[PCORE-CU-FIX1-ADMISSION] tick=%llu axis=%d decision=%u reason=%u cycles=%llu mask=%02X firstI=%016llX Kp=%016llX computedPPS=%016llX\n",
            static_cast<unsigned long long>(f.snapshot.admissionCorrectionTick),
            static_cast<int>(f.snapshot.admissionCorrectionAxis),
            static_cast<unsigned int>(f.snapshot.admissionCorrectionDecision),
            static_cast<unsigned int>(f.snapshot.admissionCorrectionReason),
            static_cast<unsigned long long>(f.snapshot.admissionCorrectionCycles),
            static_cast<unsigned int>(f.snapshot.admissionCorrectionMask),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.admissionCorrectionFirstIntegral)),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.admissionCorrectionKp)),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.admissionCorrectionVelocityPPS)));
    if (f.snapshot.crossSegment)
        RtPrintf("[PCORE-CD-LAST-FAULT] history=%u ordinal=%u retreatOrdinal=%u seams=%llu activeS=%016llX retreatLocalS=%016llX\n",
            static_cast<unsigned int>(f.snapshot.historyCount), static_cast<unsigned int>(f.snapshot.activeOrdinal),
            static_cast<unsigned int>(f.snapshot.retreatOrdinal), static_cast<unsigned long long>(f.snapshot.seamCount),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.activeS)),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.retreatLocalS)));
}
