// BZ / CA. Completed canonical G01/G17-arc geometry and bounded exact-stop replay.
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <cstring>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define NC_PATH_REPLAY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_REPLAY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_REPLAY_NOINLINE
#endif

namespace
{
    bool ReplayFullIdentity(const MotionExecutionIdentity& a, const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool ReplaySameSource(const NCProgramCommitSnapshot& a, const NCProgramCommitSnapshot& b) noexcept
    {
        return a.scope == b.scope && a.cacheGeneration == b.cacheGeneration &&
            a.frameId == b.frameId && a.sourcePC == b.sourcePC;
    }
    NC_PATH_REPLAY_NOINLINE
        bool ReplayLedgerTransportHealthy(const NCBlockLifecycleLedger& ledger) noexcept
    {
        const NCBlockLifecycleCounters c = ledger.GetCounters();
        return c.activeBlockOverwrite == 0ULL && c.activeSegmentIndexOverwrite == 0ULL &&
            c.orphanFeedback == 0ULL && c.duplicateTerminalFeedback == 0ULL && c.terminalFeedbackConflict == 0ULL;
    }
    std::uint64_t ReplayDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_REPLAY_NOINLINE
bool NCManager::IsPathCoreReplayConfigurationValid() noexcept
{
    if (!IsPathCoreFeedConfigurationValid()) return false;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        const double& modulo = m_motion.GetAxisContext(static_cast<int>(i)).rotaryModulo;
        if (std::memcmp(&modulo, &m_pathReplayRotaryModulo[i], sizeof(double)) != 0) return false;
    }
    return true;
}

NC_PATH_REPLAY_NOINLINE
bool NCManager::IsPathCoreReplayBlockShapeValid(const NCBlock& block) noexcept
{
    if (block.isEmpty || block.isGoto || !block.hasG ||
        (block.gCode != 174 && block.gCode != 175 && block.gCode != 176 && block.gCode != 177) || block.gCount != 1 ||
        block.gCodes[0] != block.gCode || block.mCount != 0 ||
        !block.has('F') || !std::isfinite(block.val('F')) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    const bool distance = block.gCode == 176 || block.gCode == 177;
    if (distance && (!block.has('D') || !std::isfinite(block.val('D')) || block.val('D') <= 0.0)) return false;
    if (block.has('L') && ((block.gCode != 174 && block.gCode != 176) || !std::isfinite(block.val('L')) ||
        block.val('L') < 1.0 || block.val('L') > 16.0 || std::floor(block.val('L')) != block.val('L'))) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && address != 'F' && address != 'L' && !(distance && address == 'D')) return false;
        if (!std::isfinite(block.val(address))) return false;
    }
    return true;
}

NC_PATH_REPLAY_NOINLINE
bool NCManager::IsPathCoreReplayInputOmission(const NCBlock& block) const noexcept
{
    if (!m_pathReplay.armed || !m_pathReplay.explicitReplay || block.hasG || block.gCount != 0) return false;
    // A nonmodal replay never grants an implicit axis/feed/parameter action.
    for (int i = 0; i < 26; ++i)
        if (block.hasParam[i] && i != ('N' - 'A')) return true;
    return false;
}

NC_PATH_REPLAY_NOINLINE
void NCManager::ArmPathCoreReplaySameThread() noexcept
{
    m_pathReplayStore.Clear();
    m_pathReplayMotion.receipt.Clear();
    m_pathReplay = PathReplayState{};
    m_pathReplayLease = m_programMotionLease;
    for (std::size_t i = 0U; i < 8U; ++i)
        m_pathReplayRotaryModulo[i] = m_motion.GetAxisContext(static_cast<int>(i)).rotaryModulo;
    m_pathReplay.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathReplay.cache = GetBaseProgramCache().GetGeneration();
    m_pathReplay.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_pathReplay.armed = m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY &&
        m_pathReplay.run != 0ULL && m_pathReplay.cache != 0ULL && m_pathReplayLease.IsValid();
}

NC_PATH_REPLAY_NOINLINE
void NCManager::ClearPathCoreReplayHistorySameThread() noexcept
{
    if (m_pathReplayStore.Count() != 0U || m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE)
        LogPathCoreReplaySameThread("HISTORY_CLEARED");
    m_pathReplayStore.Clear();
}

NC_PATH_REPLAY_NOINLINE
void NCManager::InvalidatePathCoreReplaySameThread() noexcept
{
    if (m_pathReplay.armed && (m_pathReplayStore.Count() != 0U || m_pathReplay.pending))
        LogPathCoreReplaySameThread("INVALIDATED");
    m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
    m_pathReplay.armed = false;
    m_pathReplay.pending = false;
    m_pathReplay.bound = false;
    m_pathReplay.completed = false;
    m_pathReplay.explicitReplay = false;
    m_pathReplayMotion.receipt.valid = false;
}

NC_PATH_REPLAY_NOINLINE
void NCManager::ValidatePathCoreReplaySameThread()
{
    if (!m_pathReplay.armed) return;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        m_pathReplay.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathReplay.cache != GetBaseProgramCache().GetGeneration() || !m_macroStack.empty() ||
        m_isG66Active || Homing.IsActive() || !m_pathReplayLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathReplayLease))
    {
        InvalidatePathCoreReplaySameThread();
        return;
    }
    // Completed rows intentionally survive a later normal segment's new epoch.
    // The currently live receipt alone must match the current execution epoch.
    if (m_pathReplayStore.Count() != 0U && !IsPathCoreReplayConfigurationValid())
    {
        InvalidatePathCoreReplaySameThread();
        return;
    }
    if (!m_pathReplay.pending) return;
    if (!IsPathCoreReplayConfigurationValid() ||
        m_pathReplayMotion.receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathReplayMotion.receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreReplaySameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !ReplayLedgerTransportHealthy(m_blockLifecycleLedger))
        RejectPathCoreReplaySameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_REPLAY_NOINLINE
void NCManager::BeginPathCoreReplayCaptureSameThread(const NCBlock& block, NCBlockDispatchId dispatchId) noexcept
{
    const bool replay = NCGCodeSemantics::Contains(block, 174) || NCGCodeSemantics::Contains(block, 175) ||
        NCGCodeSemantics::Contains(block, 176) || NCGCodeSemantics::Contains(block, 177);
    const bool feed = NCGCodeSemantics::Contains(block, 1) ||
        NCGCodeSemantics::Contains(block, 2) || NCGCodeSemantics::Contains(block, 3);
    if (m_pathReplay.pending) return; // Execute's guard rejects without erasing a live receipt.
    if (replay)
    {
        ClosePathCoreCommittedRunSameThread();
        m_pathFeed.explicitFeed = false;
        m_pathArc.explicitArc = false;
        m_pathReplayMotion.receipt.Clear();
        m_pathReplay.dispatch = dispatchId;
        m_pathReplay.commit = 0ULL;
        m_pathReplay.command = NCGCodeSemantics::Contains(block, 177) ? 177U :
            (NCGCodeSemantics::Contains(block, 176) ? 176U :
                (NCGCodeSemantics::Contains(block, 175) ? 175U : 174U));
        m_pathReplay.distanceMode = m_pathReplay.command == 176U || m_pathReplay.command == 177U;
        m_pathReplay.requestedD = m_pathReplay.distanceMode && block.has('D') ? block.val('D') : 0.0;
        m_pathReplay.appliedD = 0.0;
        m_pathReplay.startU = 0.0;
        m_pathReplay.endU = 0.0;
        m_pathReplay.capped = false;
        m_pathReplay.sourcePC = -1;
        m_pathReplay.sourceLine = 0;
        m_pathReplay.bound = false;
        m_pathReplay.consumerAccepted = false;
        m_pathReplay.consumerStarted = false;
        m_pathReplay.completed = false;
        m_pathReplay.code = 0U;
        return;
    }
    if (feed)
    {
        m_pathReplay.explicitReplay = false;
        m_pathReplay.distanceMode = false;
        // An explicit new forward path after a traversal begins a new retained run.
        if (m_pathReplayStore.State() != NCPathCoreRetainedCursorState::IDLE)
            ClearPathCoreReplayHistorySameThread();
        return;
    }
    if (NCGCodeSemantics::Contains(block, 0)) m_pathReplay.explicitReplay = false;
    bool plainStop = !block.hasG && block.gCount == 0 && block.mCount == 1 &&
        (block.mCode[0] == 0 || block.mCode[0] == 1);
    for (int i = 0; plainStop && i < 26; ++i)
        if (block.hasParam[i] && i != ('N' - 'A')) plainStop = false;
    if (block.isEmpty || plainStop) return;
    // CD: only a well-formed explicit cross-segment arm preserves completed
    // canonical history. Ordinary G178/G179 keep their established CC boundary.
    if (block.gCode == 178 && block.has('P') && block.val('P') == 1.0 &&
        IsPathCoreHoldBlockShapeValid(block)) return;
    ClearPathCoreReplayHistorySameThread();
}

NC_PATH_REPLAY_NOINLINE
void NCManager::RetainPathCoreFeedSameThread() noexcept
{
    if (!m_pathReplay.armed || m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE) return;
    if (!m_pathFeed.bound || !m_pathFeed.completed || !m_pathFeed.consumerAccepted ||
        (!m_pathFeedMotion.receipt.line.point && !m_pathFeed.consumerStarted) ||
        !m_pathFeedMotion.receipt.valid ||
        !BuildNCPathCoreRetainedLine(m_pathFeedMotion.receipt.line, m_pathReplayGeometry))
    {
        m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
        LogPathCoreReplaySameThread("SAVE_INVALIDATED");
        return;
    }
    AppendPathCoreReplayGeometrySameThread(m_pathFeedMotion.receipt.identity,
        m_pathFeedMotion.receipt.validAxisMask, m_pathFeed.dispatch, m_pathFeed.commit,
        m_pathFeed.sourcePC, m_pathFeed.sourceLine);
}

NC_PATH_REPLAY_NOINLINE
void NCManager::RetainPathCoreArcSameThread() noexcept
{
    if (!m_pathReplay.armed || m_pathReplayStore.Fault() != NCPathCoreRetainedFault::NONE) return;
    if (!m_pathArc.bound || !m_pathArc.completed || !m_pathArc.consumerAccepted || !m_pathArc.consumerStarted ||
        !m_pathArcMotion.receipt.valid ||
        !BuildNCPathCoreRetainedArc(m_pathArcMotion.receipt.arc, m_pathReplayGeometry))
    {
        m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
        LogPathCoreReplaySameThread("SAVE_INVALIDATED");
        return;
    }
    AppendPathCoreReplayGeometrySameThread(m_pathArcMotion.receipt.identity,
        m_pathArcMotion.receipt.validAxisMask, m_pathArc.dispatch, m_pathArc.commit,
        m_pathArc.sourcePC, m_pathArc.sourceLine);
}

NC_PATH_REPLAY_NOINLINE
void NCManager::AppendPathCoreReplayGeometrySameThread(const MotionExecutionIdentity& identity,
    std::uint32_t validAxisMask, std::uint64_t dispatch, std::uint64_t commit,
    int sourcePC, int sourceLine) noexcept
{
    const std::uint32_t count = m_pathReplayStore.Count();
    if (!IsPathCoreReplayConfigurationValid() || !identity.IsAssigned() || identity.source != MotionCommandSource::NC_MEMORY ||
        identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) || sourcePC < 0 ||
        sourceLine <= 0 || dispatch == 0ULL || commit == 0ULL ||
        !m_pathReplayLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_pathReplayLease) ||
        m_pathReplay.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathReplay.cache != GetBaseProgramCache().GetGeneration() ||
        (count != 0U && (dispatch <= m_pathReplaySource[count - 1U].dispatch ||
            commit <= m_pathReplaySource[count - 1U].commit)))
    {
        m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
        LogPathCoreReplaySameThread("SAVE_INVALIDATED");
        return;
    }
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        m_pathReplayPulsePerMM[i] = (validAxisMask & (1U << i)) != 0U ?
            axis.resolution_PPR / axis.finalLead : 0.0;
    }
    if (!m_pathReplayStore.Append(m_pathReplayGeometry, validAxisMask, m_pathReplayPulsePerMM))
    {
        LogPathCoreReplaySameThread("SAVE_INVALIDATED");
        return; // Capacity/discontinuity cannot stop the already completed forward motion.
    }
    PathReplaySource& source = m_pathReplaySource[count];
    source.identity = identity;
    source.dispatch = dispatch;
    source.commit = commit;
    source.sourcePC = sourcePC;
    source.sourceLine = sourceLine;
    LogPathCoreReplaySameThread("SAVED");
    LogPathCoreReplayGeometrySameThread(count + 1U, false);
}

NC_PATH_REPLAY_NOINLINE
void NCManager::RejectPathCoreReplaySameThread(std::uint32_t code, int alarmCode)
{
    const NCPathCoreRetainedFault historyFault = m_pathReplayStore.Fault();
    if (alarmCode == AlarmManager::G_Code_Invalid_parameter)
    {
        if (code == 3U) alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
        else if (code == 4U)
            alarmCode = historyFault == NCPathCoreRetainedFault::CAPACITY ?
            AlarmManager::PATH_REPLAY_HISTORY_CAPACITY : AlarmManager::PATH_REPLAY_HISTORY_UNAVAILABLE;
        else if (code == 6U)
        {
            if (m_pathReplayMotion.receipt.code == MotionPathCoreRetainedCode::NOT_READY)
                alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
            else if (m_pathReplayMotion.receipt.code == MotionPathCoreRetainedCode::GEOMETRY_REJECTED)
                alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
            else alarmCode = AlarmManager::PATH_MOTION_NOT_ADMITTED;
        }
    }
    RtPrintf("[PCORE-ALARM] alarm=%d unit=REPLAY run=%llu dispatch=%llu code=%u pc=%d line=%d producer=%u geometry=NA accepted=%u history=%u historyFault=%u cursor=%u\n",
        alarmCode, static_cast<unsigned long long>(m_pathReplay.run), static_cast<unsigned long long>(m_pathReplay.dispatch),
        static_cast<unsigned int>(code), m_pathReplay.sourcePC, m_pathReplay.sourceLine,
        static_cast<unsigned int>(m_pathReplayMotion.receipt.code),
        m_pathReplayMotion.receipt.commandAccepted ? 1U : 0U,
        static_cast<unsigned int>(m_pathReplayStore.Count()), static_cast<unsigned int>(historyFault),
        static_cast<unsigned int>(m_pathReplayStore.State()));
    m_pathReplay.code = code;
    if (m_pathReplayMotion.receipt.commandAccepted) ++m_pathReplay.failed;
    else ++m_pathReplay.rejected;
    if (m_pathReplayStore.Fault() == NCPathCoreRetainedFault::NONE)
        m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
    m_pathReplay.pending = false;
    m_pathReplay.bound = false;
    m_pathReplay.completed = false;
    m_pathReplayMotion.receipt.valid = false;
    LogPathCoreReplaySameThread(m_pathReplayMotion.receipt.commandAccepted ? "FAILED" : "REJECTED");
    AlarmManager::GetInstance().Trigger(alarmCode, m_pathReplay.sourceLine);
    ChangeState(NCState::ALARM);
}

NC_PATH_REPLAY_NOINLINE
WaitConditionFunc NCManager::StartPathCoreReplaySameThread(const NCBlock& block)
{
    if (!IsPathCoreReplayBlockShapeValid(block))
    {
        RejectPathCoreReplaySameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    ValidatePathCoreReplaySameThread();
    if (!m_pathReplay.armed || m_pathReplay.pending || m_pathFeed.pending || m_pathArc.pending ||
        m_state != NCState::RUN || m_mode != NCOperationMode::MEMORY ||
        m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        !IsPathCoreReplayConfigurationValid() || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathReplay.dispatch != m_currentExecutingBlockDispatchId ||
        !m_pathReplayLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U)
    {
        RejectPathCoreReplaySameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const bool distance = block.gCode == 176 || block.gCode == 177;
    if (block.has('L'))
    {
        const std::uint32_t requested = static_cast<std::uint32_t>(block.val('L'));
        const bool begun = distance ? m_pathReplayStore.BeginDistanceRetreat(requested) :
            m_pathReplayStore.BeginRetreat(requested);
        if (!begun)
        {
            RejectPathCoreReplaySameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    m_pathReplay.reverse = block.gCode == 174 || block.gCode == 176;
    m_pathReplay.distanceMode = distance;
    const NCPathCoreRetainedGeometry* geometry = nullptr;
    const bool selected = distance ?
        m_pathReplayStore.SelectDistanceStep(!m_pathReplay.reverse, block.val('D'), geometry,
            m_pathReplay.ordinal, m_pathReplay.startU, m_pathReplay.endU) :
        m_pathReplayStore.SelectStep(!m_pathReplay.reverse, geometry, m_pathReplay.ordinal);
    if (!selected || !geometry || m_pathReplay.ordinal == 0U || m_pathReplay.ordinal > 16U)
    {
        RejectPathCoreReplaySameThread(4U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (distance)
    {
        m_pathReplay.requestedD = block.val('D');
        m_pathReplay.appliedD = std::fabs(m_pathReplay.endU - m_pathReplay.startU) * geometry->lengthMM;
        const double remaining = (m_pathReplay.reverse ? m_pathReplay.startU :
            1.0 - m_pathReplay.startU) * geometry->lengthMM;
        m_pathReplay.capped = m_pathReplay.requestedD > remaining;
    }
    else
    {
        m_pathReplay.startU = m_pathReplay.reverse ? 1.0 : 0.0;
        m_pathReplay.endU = m_pathReplay.reverse ? 0.0 : 1.0;
    }
    m_pathReplay.feedMMMin = block.val('F');
    MotionArcTravelGuard travelGuard{};
    travelGuard.context = this;
    travelGuard.check = [](const void* context, int axisIndex, double target) -> bool
    {
        const NCManager* nc = static_cast<const NCManager*>(context);
        return axisIndex >= 0 && axisIndex < 8 &&
            nc->CoordSys.IsTargetWithinSoftwareTravelLimit(nc->m_motion.GetAxisContext(axisIndex), target);
    };
    const bool accepted = distance ?
        m_motion.TryPathCoreRetainedIntervalMoveTransactionalTail(*geometry,
            m_pathReplay.startU, m_pathReplay.endU, m_pathReplay.feedMMMin, travelGuard,
            CoordSys.commandedMCS, m_pathReplayMotion, m_pathReplayCommand) :
        m_motion.TryPathCoreRetainedMoveTransactionalTail(*geometry,
            m_pathReplay.reverse, m_pathReplay.feedMMMin, travelGuard,
            CoordSys.commandedMCS, m_pathReplayMotion, m_pathReplayCommand);
    if (!accepted)
    {
        RejectPathCoreReplaySameThread(6U, m_pathReplayMotion.receipt.commandAccepted ?
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY :
            (m_pathReplayMotion.receipt.travelLimitRejected ? static_cast<int>(AlarmManager::PROGRAMMED_OVER_TRAVEL) :
                static_cast<int>(AlarmManager::G_Code_Invalid_parameter)));
        return nullptr;
    }
    const MotionPathCoreRetainedReceipt& receipt = m_pathReplayMotion.receipt;
    const PathReplaySource& source = m_pathReplaySource[m_pathReplay.ordinal - 1U];
    if (!receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted || !receipt.captureBound ||
        receipt.travelLimitRejected || !receipt.identity.IsAssigned() ||
        (distance && (!m_pathReplayCommand.pathCoreRetainedTraversal || m_pathReplayCommand.mem_enableTransform ||
            m_pathReplayCommand.pathCoreRetainedReverse != m_pathReplay.reverse ||
            m_pathReplayCommand.mem_transformOrigin[0U] != m_pathReplay.startU ||
            m_pathReplayCommand.mem_transformOrigin[1U] != m_pathReplay.endU ||
            m_pathReplayCommand.mem_transformOrigin[2U] != 1.0)) ||
        receipt.validAxisMask != m_pathReplayStore.ValidAxisMask() ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        !receipt.ownerLease.Matches(m_programMotionLease) ||
        ReplayFullIdentity(receipt.identity, source.identity) || !m_pathReplayStore.MarkSubmitted())
    {
        RejectPathCoreReplaySameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    ++m_pathReplay.submitted;
    m_pathReplay.pending = true;
    m_pathReplay.explicitReplay = true;
    m_pathReplay.code = 1U;
    LogPathCoreReplaySameThread("SUBMITTED");
    LogPathCoreReplayGeometrySameThread(m_pathReplay.ordinal, m_pathReplay.reverse);
    return [](NCManager* nc) { return nc->CompletePathCoreReplaySameThread(); };
}
NC_PATH_REPLAY_NOINLINE
void NCManager::CommitPathCoreReplayCaptureSameThread(NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
    bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
    int sourcePC, int sourceLine)
{
    if (!m_pathReplay.pending || m_pathReplay.dispatch != dispatchId) return;
    m_pathReplay.sourcePC = sourcePC;
    m_pathReplay.sourceLine = sourceLine;
    const MotionPathCoreRetainedReceipt& r = m_pathReplayMotion.receipt;
    // Ledger owner fields are populated by feedback, after this commit.
    // Bind the receipt to the current program lease here; Observe validates
    // each feedback event's owner and generation when it is consumed.
    if (!r.valid || !committed || !commit.IsValid() || !ledgerFound ||
        capture.count != 1U || capture.overflow || ledger.dispatchId != dispatchId ||
        !ledger.programCommitted || ledger.ncDispatchFailed || ledger.motionCaptureOverflow ||
        ledger.motionSegmentCount != 1U || ledger.sourceLineNumber != sourceLine ||
        sourcePC < 0 || sourceLine <= 0 || commit.sourcePC != sourcePC ||
        commit.scope != NCProgramScope::MEMORY || commit.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        commit.cacheGeneration != m_pathReplay.cache || !ReplaySameSource(commit, ledger.programTarget) ||
        !ReplaySameSource(commit, ledger.programCommit) || commit.sequence != ledger.programCommit.sequence ||
        !capture.submissions[0U].producerAccepted ||
        capture.submissions[0U].immediateRejectReason != MotionRejectReason::NONE ||
        capture.submissions[0U].commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        !ledger.motionSegments[0U].producerAccepted ||
        ledger.motionSegments[0U].immediateRejectReason != MotionRejectReason::NONE ||
        !ReplayFullIdentity(capture.submissions[0U].identity, r.identity) ||
        !ReplayFullIdentity(ledger.motionSegments[0U].identity, r.identity) ||
        r.identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
        r.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !r.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreReplaySameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathReplay.commit = commit.sequence;
    m_pathReplay.bound = true;
    LogPathCoreReplaySameThread("BOUND");
}

NC_PATH_REPLAY_NOINLINE
void NCManager::ObservePathCoreReplayFeedbackSameThread(const MotionFeedbackEvent& event,
    bool ledgerAccepted)
{
    if (!m_pathReplay.pending || !m_pathReplay.bound ||
        !ReplayFullIdentity(event.identity, m_pathReplayMotion.receipt.identity)) return;
    if (!ledgerAccepted || event.sequence == 0ULL || event.sequence <= m_pathReplay.lastSequence ||
        event.owner != m_pathReplayMotion.receipt.ownerLease.owner ||
        event.ownerGeneration != m_pathReplayMotion.receipt.ownerLease.generation)
    {
        RejectPathCoreReplaySameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathReplay.lastSequence = event.sequence;
    switch (event.type)
    {
    case MotionFeedbackType::ACCEPTED:
        if (m_pathReplay.consumerAccepted) break;
        m_pathReplay.consumerAccepted = true;
        ++m_pathReplay.accepted;
        return;
    case MotionFeedbackType::STARTED:
        if (!m_pathReplay.consumerAccepted || m_pathReplay.consumerStarted) break;
        m_pathReplay.consumerStarted = true;
        ++m_pathReplay.started;
        return;
    case MotionFeedbackType::COMPLETED:
        if (!m_pathReplay.consumerAccepted || m_pathReplay.completed || event.rejectReason != MotionRejectReason::NONE || event.errorCode != 0U) break;
        {
            const NCPathCoreRetainedGeometry* g = m_pathReplayStore.Get(m_pathReplay.ordinal - 1U);
            if (!g || (!g->point && !m_pathReplay.consumerStarted)) break;
        }
        m_pathReplay.completed = true;
        return;
    case MotionFeedbackType::PROGRESS:
    case MotionFeedbackType::HELD:
    case MotionFeedbackType::RESUMED:
        if (m_pathReplay.consumerAccepted) return;
        break;
    default:
        break;
    }
    RejectPathCoreReplaySameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_REPLAY_NOINLINE
bool NCManager::CompletePathCoreReplaySameThread()
{
    if (!m_pathReplay.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreReplaySameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (m_state == NCState::ALARM) return false;
    if (!m_pathReplay.pending) return m_pathReplay.completed;
    if (IsFeedHoldActive()) return false;
    ValidatePathCoreReplaySameThread();
    if (!m_pathReplay.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreReplaySameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (!m_pathReplay.bound || !m_pathReplay.completed || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone()) return false;
    if (!m_pathReplayStore.CompleteStep())
    {
        RejectPathCoreReplaySameThread(13U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    m_pathReplay.pending = false;
    ++m_pathReplay.done;
    LogPathCoreReplaySameThread("COMPLETED");
    return true;
}

NC_PATH_REPLAY_NOINLINE
void NCManager::FinalizePathCoreReplaySameThread() noexcept
{
    if (m_pathReplay.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_pathReplay.submitted != 0U) LogPathCoreReplaySameThread("FINALIZED");
    m_pathReplay.armed = false;
    m_pathReplay.explicitReplay = false;
    m_pathReplayStore.Fail(NCPathCoreRetainedFault::LIFECYCLE);
    m_pathReplayMotion.receipt.valid = false;
}

NC_PATH_REPLAY_NOINLINE
void NCManager::LogPathCoreReplaySameThread(const char* phase) const noexcept
{
    const PathReplayState& s = m_pathReplay;
    const MotionPathCoreRetainedReceipt& r = m_pathReplayMotion.receipt;
    if (s.distanceMode)
        RtPrintf("[PCORE-CA] run=%llu dispatch=%llu phase=%s command=%u ordinal=%u requestedDBits=%llu appliedDBits=%llu uStartBits=%llu uEndBits=%llu currentOrdinal=%u currentUBits=%llu capped=%u\n",
            static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
            static_cast<unsigned int>(s.command), static_cast<unsigned int>(s.ordinal),
            static_cast<unsigned long long>(ReplayDoubleBits(s.requestedD)),
            static_cast<unsigned long long>(ReplayDoubleBits(s.appliedD)),
            static_cast<unsigned long long>(ReplayDoubleBits(s.startU)),
            static_cast<unsigned long long>(ReplayDoubleBits(s.endU)),
            static_cast<unsigned int>(m_pathReplayStore.CurrentOrdinal()),
            static_cast<unsigned long long>(ReplayDoubleBits(m_pathReplayStore.CurrentU())), s.capped ? 1U : 0U);
    RtPrintf("[PCORE-BZ] run=%llu dispatch=%llu phase=%s code=%u command=%u pc=%d line=%d commit=%llu pending=%u bound=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
        static_cast<unsigned int>(s.code), static_cast<unsigned int>(s.command), s.sourcePC, s.sourceLine,
        static_cast<unsigned long long>(s.commit), s.pending ? 1U : 0U, s.bound ? 1U : 0U);
    RtPrintf("[PCORE-BZ-ID] run=%llu dispatch=%llu epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u valid=%u producerCode=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(r.identity.segmentId),
        static_cast<unsigned int>(r.identity.sourceBlockId), static_cast<unsigned int>(r.identity.source),
        static_cast<unsigned int>(r.ownerLease.owner), static_cast<unsigned int>(r.ownerLease.generation),
        r.valid ? 1U : 0U, static_cast<unsigned int>(r.code));
    RtPrintf("[PCORE-BZ-CURSOR] run=%llu dispatch=%llu count=%u state=%u position=%u lower=%u requested=%u ordinal=%u reverse=%u fault=%u savedPending=%u FBits=%llu\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned int>(m_pathReplayStore.Count()), static_cast<unsigned int>(m_pathReplayStore.State()),
        static_cast<unsigned int>(m_pathReplayStore.Position()), static_cast<unsigned int>(m_pathReplayStore.LowerBound()),
        static_cast<unsigned int>(m_pathReplayStore.Requested()), static_cast<unsigned int>(s.ordinal), s.reverse ? 1U : 0U,
        static_cast<unsigned int>(m_pathReplayStore.Fault()), m_pathReplayStore.Pending() ? 1U : 0U,
        static_cast<unsigned long long>(ReplayDoubleBits(s.feedMMMin)));
    RtPrintf("[PCORE-BZ-CNT] run=%llu submitted=%u accepted=%u started=%u done=%u rejected=%u failed=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned int>(s.submitted),
        static_cast<unsigned int>(s.accepted), static_cast<unsigned int>(s.started), static_cast<unsigned int>(s.done),
        static_cast<unsigned int>(s.rejected), static_cast<unsigned int>(s.failed));
}

NC_PATH_REPLAY_NOINLINE
void NCManager::LogPathCoreReplayGeometrySameThread(std::uint32_t ordinal, bool reverse) const noexcept
{
    if (ordinal == 0U || ordinal > 16U) return;
    const NCPathCoreRetainedGeometry* g = m_pathReplayStore.Get(ordinal - 1U);
    if (!g) return;
    const PathReplaySource& source = m_pathReplaySource[ordinal - 1U];
    RtPrintf("[PCORE-BZ-SOURCE] run=%llu dispatch=%llu ordinal=%u originalDispatch=%llu originalCommit=%llu originalPC=%d originalLine=%d originalEpoch=%llu originalSeg=%llu originalSource=%u\n",
        static_cast<unsigned long long>(m_pathReplay.run), static_cast<unsigned long long>(m_pathReplay.dispatch),
        static_cast<unsigned int>(ordinal), static_cast<unsigned long long>(source.dispatch),
        static_cast<unsigned long long>(source.commit), source.sourcePC, source.sourceLine,
        static_cast<unsigned long long>(source.identity.epoch), static_cast<unsigned long long>(source.identity.segmentId),
        static_cast<unsigned int>(source.identity.source));
    RtPrintf("[PCORE-BZ-GEO] run=%llu dispatch=%llu ordinal=%u kind=%u mask=%u reverse=%u point=%u direction=%d fullCircle=%u radiusMMBits=%llu radiusPulseBits=%llu startAngleBits=%llu sweepBits=%llu lengthMMBits=%llu lengthPulseBits=%llu\n",
        static_cast<unsigned long long>(m_pathReplay.run), static_cast<unsigned long long>(m_pathReplay.dispatch),
        static_cast<unsigned int>(ordinal), static_cast<unsigned int>(g->kind), static_cast<unsigned int>(g->axisMask),
        reverse ? 1U : 0U, g->point ? 1U : 0U, g->direction, g->fullCircle ? 1U : 0U,
        static_cast<unsigned long long>(ReplayDoubleBits(g->radiusMM)),
        static_cast<unsigned long long>(ReplayDoubleBits(g->radiusPulse)),
        static_cast<unsigned long long>(ReplayDoubleBits(g->startAngle)),
        static_cast<unsigned long long>(ReplayDoubleBits(g->sweepRadians)),
        static_cast<unsigned long long>(ReplayDoubleBits(g->lengthMM)),
        static_cast<unsigned long long>(ReplayDoubleBits(g->lengthPulse)));
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if ((m_pathReplayStore.ValidAxisMask() & (1U << i)) == 0U) continue;
        double traversalStart = reverse ? g->endPulse[i] : g->startPulse[i];
        double traversalEnd = reverse ? g->startPulse[i] : g->endPulse[i];
        if (m_pathReplay.distanceMode)
        {
            if (!EvaluateNCPathCoreRetainedPulseCanonical(*g, static_cast<std::uint32_t>(i),
                m_pathReplay.startU, traversalStart) ||
                !EvaluateNCPathCoreRetainedPulseCanonical(*g, static_cast<std::uint32_t>(i),
                    m_pathReplay.endU, traversalEnd)) return;
        }
        RtPrintf("[PCORE-BZ-AXIS] run=%llu dispatch=%llu ordinal=%u axis=%u selected=%u startBits=%llu endBits=%llu startPulseBits=%llu endPulseBits=%llu traversalStartPulseBits=%llu traversalEndPulseBits=%llu\n",
            static_cast<unsigned long long>(m_pathReplay.run), static_cast<unsigned long long>(m_pathReplay.dispatch),
            static_cast<unsigned int>(ordinal), static_cast<unsigned int>(i), (g->axisMask & (1U << i)) != 0U ? 1U : 0U,
            static_cast<unsigned long long>(ReplayDoubleBits(g->startMCS[i])),
            static_cast<unsigned long long>(ReplayDoubleBits(g->endMCS[i])),
            static_cast<unsigned long long>(ReplayDoubleBits(g->startPulse[i])),
            static_cast<unsigned long long>(ReplayDoubleBits(g->endPulse[i])),
            static_cast<unsigned long long>(ReplayDoubleBits(traversalStart)),
            static_cast<unsigned long long>(ReplayDoubleBits(traversalEnd)));
        if (g->kind == NCPathCoreRetainedKind::ARC && i < 2U)
            RtPrintf("[PCORE-BZ-CIRCLE] run=%llu dispatch=%llu ordinal=%u axis=%u centerBits=%llu centerPulseBits=%llu minBits=%llu maxBits=%llu\n",
                static_cast<unsigned long long>(m_pathReplay.run), static_cast<unsigned long long>(m_pathReplay.dispatch),
                static_cast<unsigned int>(ordinal), static_cast<unsigned int>(i),
                static_cast<unsigned long long>(ReplayDoubleBits(g->centerMCS[i])),
                static_cast<unsigned long long>(ReplayDoubleBits(g->centerPulse[i])),
                static_cast<unsigned long long>(ReplayDoubleBits(g->boundsMinMCS[i])),
                static_cast<unsigned long long>(ReplayDoubleBits(g->boundsMaxMCS[i])));
    }
}
#undef NC_PATH_REPLAY_NOINLINE
