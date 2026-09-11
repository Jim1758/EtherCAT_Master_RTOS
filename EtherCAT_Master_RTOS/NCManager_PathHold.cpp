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
    bool HoldIdentityEqual(const MotionExecutionIdentity& a, const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
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
        block.val('P') != 1.0 && block.val('P') != 2.0 && block.val('P') != 3.0 && block.val('P') != 4.0) return false;
    if (block.gCode == 178 && block.has('P') && (block.val('P') == 2.0 || block.val('P') == 3.0) &&
        (!block.has('Q') || (block.has('L') && block.val('L') != 1.0))) return false;
    if (block.gCode == 178 && block.has('P') && block.val('P') == 4.0 && !block.has('Q')) return false;
    if (block.gCode == 178 && block.has('Q') &&
        (!block.has('P') || !std::isfinite(block.val('Q')) || block.val('Q') <= 0.0)) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && !(block.gCode == 178 && (address == 'D' || address == 'F' || address == 'L' || address == 'P' || address == 'Q'))) return false;
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
void NCManager::InvalidatePathCoreHoldSameThread() noexcept
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
    if (m_pathHold.bound) m_motion.CancelPathCoreHoldExcursion();
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
    m_pathHold.cycleLimit = block.has('L') ? static_cast<std::uint32_t>(block.val('L')) : 1U;
    m_pathHold.crossSegment = block.has('P');
    m_pathHold.automaticEnabled = block.has('Q');
    m_pathHold.automaticIntervalMM = block.has('Q') ? block.val('Q') : 0.0;
    if (m_pathHold.crossSegment && !PreparePathCoreHoldHistorySameThread())
    {
        RejectPathCoreHoldSameThread(17U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    m_pathHold.armed = true;
    m_pathHold.code = 1U;
    if (block.has('P') && (block.val('P') == 2.0 || block.val('P') == 3.0 || block.val('P') == 4.0) &&
        !StartGapPathSimulationSameThread(block.val('P') != 2.0, block.val('P') == 4.0))
        return nullptr;
    LogPathCoreHoldSameThread("ARMED");
    if (m_pathHold.automaticEnabled) LogPathCoreHoldAutomaticSameThread("ARMED");
    return nullptr;
}

// CD admission copies only completed, same-run canonical history. Earlier rows
// may have older execution epochs: they grant geometry, never execution authority.
NC_PATH_HOLD_NOINLINE
bool NCManager::PreparePathCoreHoldHistorySameThread() noexcept
{
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
            !source.identity.IsAssigned() || source.identity.source != MotionCommandSource::NC_MEMORY ||
            source.sourcePC < 0 || source.sourceLine <= 0 ||
            source.identity.sourceBlockId != static_cast<MotionSourceBlockId>(source.sourcePC) ||
            source.dispatch == 0ULL || source.commit == 0ULL ||
            source.dispatch >= m_currentExecutingBlockDispatchId ||
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
    if (mask != m_pathHoldView.validAxisMask) return false;
    for (std::uint32_t i = 0U; i < count; ++i)
    {
        const NCPathCoreRetainedGeometry* row = m_pathReplayStore.Get(i);
        const PathReplaySource& source = m_pathReplaySource[i];
        if (row == nullptr || !HoldGeometryEqual(*row, m_pathHoldView.completed[i]) ||
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
    m_pathHold.candidateDispatch = dispatchId;
}

NC_PATH_HOLD_NOINLINE
void NCManager::CommitPathCoreHoldCaptureSameThread(NCBlockDispatchId dispatchId)
{
    if (!m_pathHold.armed || m_pathHold.bound || m_pathHold.candidateDispatch != dispatchId) return;
    ObservePathCoreHoldSameThread();
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
    m_pathHold.dispatch = dispatchId;
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
    if (!m_pathHold.identity.IsAssigned() || m_pathHold.identity.source != MotionCommandSource::NC_MEMORY ||
        m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() || m_pathHold.commit == 0ULL ||
        m_pathHold.sourcePC < 0 || m_pathHold.sourceLine <= 0 ||
        m_pathHold.identity.sourceBlockId != static_cast<MotionSourceBlockId>(m_pathHold.sourcePC) ||
        !std::isfinite(sourceFeed) || sourceFeed <= 0.0 || m_pathHold.feedMMMin > sourceFeed ||
        !std::isfinite(sourceVelocity) || sourceVelocity <= 0.0 ||
        (m_pathHold.crossSegment && !BuildPathCoreHoldViewSameThread(line)) ||
        !m_motion.BindPathCoreHoldExcursion(m_pathHold.identity, m_pathHold.lease,
            lengthMM, lengthPulse, m_pathHold.distanceMM, m_pathHold.feedMMMin, sourceFeed, sourceVelocity,
            m_pathHold.cycleLimit, m_pathHold.crossSegment ? &m_pathHoldView : nullptr))
    {
        RejectPathCoreHoldSameThread(7U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathHold.armed = false;
    m_pathHold.bound = true;
    m_pathHold.code = 1U;
    if (m_pathHold.automaticEnabled)
    {
        m_pathHold.automaticIntervalPulse = m_pathHold.automaticIntervalMM * (lengthPulse / lengthMM);
        m_pathHold.automaticNextS = m_pathHold.automaticIntervalPulse;
        if (!std::isfinite(m_pathHold.automaticIntervalPulse) || m_pathHold.automaticIntervalPulse <= 0.0)
        {
            RejectPathCoreHoldSameThread(18U, AlarmManager::PATH_GEOMETRY_INVALID);
            return;
        }
        if (m_gapPath.active && m_pathHold.automaticIntervalPulse >= lengthPulse)
        {
            RejectGapPathSimulationSameThread("STATION_OUTSIDE_SOURCE");
            return;
        }
        LogPathCoreHoldAutomaticSameThread("BOUND");
    }
    LogPathCoreHoldSameThread("BOUND");
}

NC_PATH_HOLD_NOINLINE
void NCManager::ObservePathCoreHoldSameThread()
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
    const bool completed = (m_pathFeed.dispatch == m_pathHold.dispatch && m_pathFeed.completed) ||
        (m_pathArc.dispatch == m_pathHold.dispatch && m_pathArc.completed);
    if (completed)
    {
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
                !HoldIdentityEqual(terminal.identity, m_pathHold.identity) ||
                m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
                !terminal.ownerLease.Matches(m_pathHold.lease) || !terminal.crossSegment ||
                terminal.historyCount != m_pathHoldView.completedCount ||
                terminal.cycleLimit != m_pathHold.cycleLimit ||
                terminal.phase != MotionPathCoreHoldExcursionPhase::COMPLETE ||
                m_pathHold.requestedHoldSequence == MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID ||
                terminal.holdRequestSequence != m_pathHold.requestedHoldSequence ||
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
            if (!ServiceGapPathSimulationSameThread(0.0, true, "FINAL_RETURN")) return;
            if (!IsGapPathAutomaticNormalSameThread() ||
                m_gapInput.Current().sampledAtMs != m_gapPath.lastServiceMs)
            {
                RejectGapPathSimulationSameThread("FINAL_RETURN_NOT_NORMAL");
                return;
            }
            m_pathHold.automaticObservedReturns = terminal.returnCount;
            LogPathCoreHoldAutomaticSameThread("RETURNED");
            CancelPathCoreHoldAutomaticSameThread("BUDGET_DONE");
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
    if (HoldIdentityEqual(snapshot.identity, m_pathHold.identity) &&
        (snapshot.transitionSequence != m_pathHold.observedTransition ||
            (snapshot.crossSegment && snapshot.seamCount != m_pathHold.observedSeamCount)))
    {
        m_pathHold.observedTransition = snapshot.transitionSequence;
        m_pathHold.observedSeamCount = snapshot.seamCount;
        LogPathCoreHoldSameThread("RT_PHASE");
    }
    if (HoldIdentityEqual(snapshot.identity, m_pathHold.identity) &&
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
bool NCManager::StartGapPathSimulationSameThread(bool automaticResume, bool repeating) noexcept
{
    m_gapPath = GapPathSimulationState{};
    m_gapServiceCurrent = GapServiceDiagnostic{};
    m_gapServiceAgeOnlyCalls = 0U;
    m_gapPath.active = true;
    m_gapPath.automaticResume = automaticResume;
    m_gapPath.repeating = repeating;
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
void NCManager::RejectGapPathSimulationSameThread(const char* reason) noexcept
{
    m_gapServiceLastFault = m_gapServiceCurrent;
    m_gapServiceLastFault.present = true;
    m_gapServiceLastFault.quality = m_gapInput.Current().quality;
    m_gapServiceLastFault.run = m_pathHold.run;
    m_gapServiceLastFault.dispatch = m_pathHold.dispatch;
    RtPrintf("[%s] phase=FAILED reason=%s run=%llu dispatch=%llu line=%d alarm=%d\n",
        m_gapPath.repeating ? "GAP-CK" : (m_gapPath.automaticResume ? "GAP-CI" : "GAP-CH"), reason, static_cast<unsigned long long>(m_pathHold.run),
        static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
        static_cast<int>(AlarmManager::GAP_PATH_SIMULATION_FAILED));
    if (m_pathHold.bound && m_state == NCState::RUN &&
        m_pathHold.identity.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_pathHold.lease))
        FeedHoldInternal();
    RejectPathCoreHoldSameThread(19U, AlarmManager::GAP_PATH_SIMULATION_FAILED);
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
}

NC_PATH_HOLD_NOINLINE
bool NCManager::ServiceGapPathSimulationSameThread(double activeS, bool publishSample, const char* site) noexcept
{
    if (!m_gapPath.active) return false;
    GapServiceDiagnostic& diagnostic = m_gapServiceCurrent;
    diagnostic = GapServiceDiagnostic{};
    diagnostic.present = true;
    diagnostic.site = site != nullptr ? site : "AUTOMATIC";
    diagnostic.lastServiceMs = m_gapPath.lastServiceMs;
    diagnostic.lastServiceValid = m_gapPath.clockStarted;
    diagnostic.sampledAtMs = m_gapInput.Current().sampledAtMs;
    diagnostic.sampleSequence = m_gapInput.Current().sequence;
    diagnostic.quality = m_gapInput.Current().quality;
    diagnostic.publishSample = publishSample;
    diagnostic.run = m_pathHold.run;
    diagnostic.dispatch = m_pathHold.dispatch;
    if (!publishSample && m_gapServiceAgeOnlyCalls != (std::numeric_limits<std::uint32_t>::max)())
        ++m_gapServiceAgeOnlyCalls;
    diagnostic.ageOnlyCalls = m_gapServiceAgeOnlyCalls;
    LARGE_INTEGER counter{};
    if (m_gapDryRun.active || m_gapPath.frequency == 0ULL ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0)
    {
        RejectGapPathSimulationSameThread("CLOCK_READ_OR_SESSION");
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
        RejectGapPathSimulationSameThread("CLOCK_REGRESSION_OR_RANGE");
        return false;
    }
    const std::uint64_t nowMs = seconds * 1000ULL + fraction;
    if (m_gapPath.clockStarted)
    {
        if (nowMs < m_gapPath.lastServiceMs || nowMs - m_gapPath.lastServiceMs > 250ULL)
        {
            RejectGapPathSimulationSameThread("SERVICE_GAP");
            return false;
        }
        m_gapPath.stalledCalls = nowMs == m_gapPath.lastServiceMs ? m_gapPath.stalledCalls + 1U : 0U;
        if (m_gapPath.stalledCalls >= 4096U)
        {
            RejectGapPathSimulationSameThread("CLOCK_NOT_ADVANCING");
            return false;
        }
        if (m_gapPath.sequence != 0ULL && m_gapInput.Poll(nowMs).quality != EDMGap::Quality::VALID)
        {
            RejectGapPathSimulationSameThread("INPUT_QUALITY_OR_AGE");
            return false;
        }
        if (m_gapPath.sequence == 0ULL && nowMs - m_gapPath.firstServiceMs > 100ULL)
        {
            RejectGapPathSimulationSameThread("SOURCE_SAMPLE_UNAVAILABLE");
            return false;
        }
    }
    else m_gapPath.firstServiceMs = nowMs;
    m_gapPath.clockStarted = true;
    m_gapPath.lastTicks = ticks;
    m_gapPath.lastServiceMs = nowMs;
    // An unavailable RT publication may age input, never refresh or invent it.
    if (!publishSample) return true;
    if (!std::isfinite(activeS) || activeS < 0.0 ||
        !std::isfinite(m_pathHold.automaticNextS) || m_pathHold.automaticNextS <= 0.0 ||
        m_gapPath.sequence == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapPathSimulationSameThread("PROGRESS_OR_SEQUENCE");
        return false;
    }
    bool lowEvent = false, recoveryEvent = false;
    if (!m_gapPath.held && !m_gapPath.lowInjected && activeS >= m_pathHold.automaticNextS)
    {
        m_gapPath.lowInjected = true;
        lowEvent = true;
    }
    if (m_gapPath.held && !m_gapPath.recoveryInjected &&
        nowMs >= m_gapPath.holdStartMs &&
        nowMs - m_gapPath.holdStartMs >= (m_gapPath.automaticResume ? 3000ULL : 1000ULL))
    {
        m_gapPath.recoveryInjected = true;
        recoveryEvent = true;
    }
    EDMGap::Sample sample{};
    sample.source = EDMGap::Source::SIMULATED;
    sample.valid = true;
    sample.voltageMv = m_gapPath.lowInjected && !m_gapPath.recoveryInjected ? 20000 : 50000;
    sample.sequence = ++m_gapPath.sequence;
    sample.sampledAtMs = nowMs;
    const EDMGap::Snapshot& gap = m_gapInput.Publish(sample, nowMs);
    if (gap.quality != EDMGap::Quality::VALID || gap.source != EDMGap::Source::SIMULATED)
    {
        RejectGapPathSimulationSameThread("INPUT_NOT_VALID");
        return false;
    }
    m_gapServiceAgeOnlyCalls = 0U;
    if (lowEvent) LogGapPathSimulationSameThread("INJECT_LOW");
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
bool NCManager::ValidateGapPathAutomaticResumeSameThread() noexcept
{
    if (!m_gapPath.active || !m_gapPath.automaticResume || !m_pathHold.automaticHoldOwned) return true;
    // Admission can be delayed between the service, Prepare and Commit. Poll
    // the actual clock without publishing or qualifying a replacement sample.
    if (!ServiceGapPathSimulationSameThread(0.0, false, "ADMISSION")) return false;
    if (!m_pathHold.automaticAdmissionOwned || !IsGapPathAutomaticNormalSameThread())
    {
        RejectGapPathSimulationSameThread("RECOVERY_NOT_NORMAL_AT_ADMISSION");
        return false;
    }
    return true;
}

NC_PATH_HOLD_NOINLINE
void NCManager::LogGapPathSimulationSameThread(const char* phase) const noexcept
{
    const EDMGap::Snapshot& gap = m_gapInput.Current();
    RtPrintf("[%s] phase=%s source=%s quality=%s band=%s mv=%d seq=%llu ms=%llu run=%llu dispatch=%llu line=%d owner=%u generation=%llu epoch=%llu hold=%llu owned=%u ack=%u low=%u recovered=%u resume=%s completed=%llu limit=%u nextS=%016llX motion=1 discharge=0\n",
        m_gapPath.repeating ? "GAP-CK" : (m_gapPath.automaticResume ? "GAP-CI" : "GAP-CH"), phase, EDMGap::SourceName(gap.source), EDMGap::QualityName(gap.quality), EDMGap::BandName(gap.band),
        static_cast<int>(gap.voltageMv), static_cast<unsigned long long>(gap.sequence),
        static_cast<unsigned long long>(m_gapPath.lastServiceMs), static_cast<unsigned long long>(m_pathHold.run),
        static_cast<unsigned long long>(m_pathHold.dispatch), m_pathHold.sourceLine,
        static_cast<unsigned int>(m_pathHold.lease.owner), static_cast<unsigned long long>(m_pathHold.lease.generation),
        static_cast<unsigned long long>(m_pathHold.identity.epoch),
        static_cast<unsigned long long>(m_pathHold.automaticSettleSequence),
        m_pathHold.automaticHoldOwned ? 1U : 0U, m_gapPath.ackLogged ? 1U : 0U,
        m_gapPath.lowInjected ? 1U : 0U, m_gapPath.recoveryLogged ? 1U : 0U,
        m_gapPath.automaticResume ? "AUTO_NORMAL" : "OPERATOR",
        static_cast<unsigned long long>(m_pathHold.automaticObservedReturns),
        static_cast<unsigned int>(m_pathHold.cycleLimit),
        static_cast<unsigned long long>(HoldDoubleBits(m_pathHold.automaticNextS)));
}

// CF: distance-triggered dry-run supervision. It never supplies GAP velocity.
NC_PATH_HOLD_NOINLINE
void NCManager::CancelPathCoreHoldAutomaticSameThread(const char* reason) noexcept
{
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

NC_PATH_HOLD_NOINLINE
bool NCManager::ProcessPathCoreHoldAutomaticSameThread() noexcept
{
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
        return true;
    }
    const bool line = m_pathFeed.pending && m_pathFeed.bound && m_pathFeed.dispatch == m_pathHold.dispatch;
    const bool arc = m_pathArc.pending && m_pathArc.bound && m_pathArc.dispatch == m_pathHold.dispatch;
    if (line == arc || m_pathHold.identity.epoch != m_motion.GetCurrentExecutionEpoch())
    {
        CancelPathCoreHoldAutomaticSameThread("SOURCE_REVOKED");
        return false;
    }
    const auto snapshot = m_motion.GetPathCoreHoldExcursionSnapshot();
    // A bounded read can be unavailable during publication; it grants no new action.
    if (snapshot.publicationSequence == 0ULL)
    {
        if (m_gapPath.active && !ServiceGapPathSimulationSameThread(0.0, false, "NO_RT")) return true;
        return m_pathHold.automaticHoldOwned;
    }
    if (!HoldIdentityEqual(snapshot.identity, m_pathHold.identity))
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
    // CK retains the session through the excursion. Historical activeS is
    // local to its current leg and may exceed the original source length.
    const bool gapReturnPending = m_gapPath.active && m_gapPath.repeating &&
        m_gapPath.held && !m_pathHold.automaticHoldOwned &&
        m_pathHold.requested && m_pathHold.startCommitted;
    if (m_gapPath.active)
    {
        if (!std::isfinite(snapshot.activeS) || snapshot.activeS < 0.0 ||
            (!gapReturnPending && snapshot.activeS > snapshot.lengthPulse) ||
            !std::isfinite(snapshot.lengthPulse) || snapshot.lengthPulse <= 0.0 ||
            !std::isfinite(m_pathHold.automaticNextS) || m_pathHold.automaticNextS <= 0.0)
        {
            RejectGapPathSimulationSameThread("PROGRESS_INVALID");
            return true;
        }
        if (!ServiceGapPathSimulationSameThread(gapReturnPending ? 0.0 : snapshot.activeS)) return true;
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
                LogGapPathSimulationSameThread(m_gapPath.automaticResume ?
                    "STOP_CONFIRMED_WAIT_NORMAL" : "STOP_CONFIRMED_WAIT_START");
            }
            if (!m_gapPath.automaticResume) return true;
            // Diagnostic latches never grant admission: require this scan's
            // actual, fully qualified sample and the exact owned J5 boundary.
            if (!IsGapPathAutomaticNormalSameThread() ||
                m_gapInput.Current().sampledAtMs != m_gapPath.lastServiceMs) return true;
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
                snapshot.holdRequestSequence != m_pathHold.requestedHoldSequence ||
                snapshot.returnCount > m_pathHold.cycleLimit ||
                !std::isfinite(snapshot.heldS) || snapshot.returnedS != snapshot.heldS ||
                !IsGapPathAutomaticNormalSameThread() ||
                m_gapInput.Current().sampledAtMs != m_gapPath.lastServiceMs))
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
        m_pathHold.automaticObservedReturns = snapshot.returnCount;
        m_pathHold.automaticNextS = snapshot.returnedS + m_pathHold.automaticIntervalPulse;
        LogPathCoreHoldAutomaticSameThread("RETURNED");
        if (m_gapPath.active && m_gapPath.repeating)
        {
            if (snapshot.returnCount >= m_pathHold.cycleLimit)
            {
                CancelPathCoreHoldAutomaticSameThread("BUDGET_DONE");
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
            m_gapPath.held = false;
            m_gapPath.holdStartMs = 0ULL;
            m_gapPath.recoveryInjected = false;
            m_gapPath.normalLogged = false;
            m_gapPath.recoveryLogged = false;
            m_gapPath.ackLogged = false;
            m_gapPath.waitJ5Logged = false;
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
        !std::isfinite(m_pathHold.automaticNextS) || m_pathHold.automaticNextS <= 0.0)
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
            gap.sampledAtMs != m_gapPath.lastServiceMs) return false;
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
    if (!HoldIdentityEqual(snapshot.identity, m_pathHold.identity)) return false; // Bind mailbox not yet consumed.
    if (!snapshot.ownerLease.Matches(m_pathHold.lease) ||
        snapshot.cycleLimit != m_pathHold.cycleLimit || snapshot.cycleLimit < 1U || snapshot.cycleLimit > 32U)
    {
        if (!m_pathHold.blocked) { m_pathHold.code = 11U; LogPathCoreHoldSameThread("RESUME_BLOCKED"); }
        m_pathHold.blocked = true;
        return false;
    }
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
    if (m_pathHold.requested && snapshot.phase != MotionPathCoreHoldExcursionPhase::COMPLETE)
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
    if ((!initial && !repeat) || !snapshot.ready ||
        snapshot.holdRequestSequence != m_feedHoldNCSettleRequestSequence ||
        !m_motion.RequestPathCoreHoldExcursion(m_pathHold.identity,
            m_pathHold.lease, m_feedHoldNCSettleRequestSequence)) return false;
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
    if (f.snapshot.crossSegment)
        RtPrintf("[PCORE-CD-LAST-FAULT] history=%u ordinal=%u retreatOrdinal=%u seams=%llu activeS=%016llX retreatLocalS=%016llX\n",
            static_cast<unsigned int>(f.snapshot.historyCount), static_cast<unsigned int>(f.snapshot.activeOrdinal),
            static_cast<unsigned int>(f.snapshot.retreatOrdinal), static_cast<unsigned long long>(f.snapshot.seamCount),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.activeS)),
            static_cast<unsigned long long>(HoldDoubleBits(f.snapshot.retreatLocalS)));
}
