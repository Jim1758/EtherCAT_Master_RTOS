// BX / Path Core V2-01. G01 is a distinct feed producer, not a G00 override.
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define NC_PATH_FEED_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_FEED_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_FEED_NOINLINE
#endif

namespace
{
    bool FeedFullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool FeedSameSource(const NCProgramCommitSnapshot& a,
        const NCProgramCommitSnapshot& b) noexcept
    {
        return a.scope == b.scope && a.cacheGeneration == b.cacheGeneration &&
            a.frameId == b.frameId && a.sourcePC == b.sourcePC;
    }
    NC_PATH_FEED_NOINLINE
        bool FeedLedgerTransportHealthy(const NCBlockLifecycleLedger& ledger) noexcept
    {
        const NCBlockLifecycleCounters counters = ledger.GetCounters();
        return counters.activeBlockOverwrite == 0ULL &&
            counters.activeSegmentIndexOverwrite == 0ULL && counters.orphanFeedback == 0ULL &&
            counters.duplicateTerminalFeedback == 0ULL && counters.terminalFeedbackConflict == 0ULL;
    }
    std::uint64_t FeedDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedBlockShapeValid(const NCBlock& block) noexcept
{
    if (block.isEmpty || block.isGoto || !block.hasG || block.gCode != 1 ||
        block.gCount != 1 || block.gCodes[0] != 1 || block.mCount != 0 ||
        !block.has('F') || !std::isfinite(block.val('F')) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0 ||
        !(block.has('X') || block.has('Y') || block.has('Z'))) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && address != 'F' && address != 'X' && address != 'Y' && address != 'Z') return false;
        if (!std::isfinite(block.val(address))) return false;
    }
    return true;
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedInputOmission(const NCBlock& block) const noexcept
{
    return m_pathFeed.armed && m_pathFeed.explicitFeed && !block.hasG && block.gCount == 0 &&
        (block.has('X') || block.has('Y') || block.has('Z') || block.has('F'));
}

NC_PATH_FEED_NOINLINE
bool NCManager::IsPathCoreFeedConfigurationValid() noexcept
{
    if (!CoordSys.isAbsoluteMode || CoordSys.isInchMode || CoordSys.GetCurrentWCSGCode() != 54 ||
        CoordSys.activePlane != 17 || CoordSys.toolLengthMode != 49 || CoordSys.toolRadiusMode != 40 ||
        CoordSys.isG68Active || CoordSys.isWorkpieceRotationActive || CoordSys.isScalingActive ||
        CoordSys.isPolarCoordinateActive || CoordSys.isCAxisOffsetRotationEnabled ||
        m_axisNames[0] != 'X' || m_axisNames[1] != 'Y' || m_axisNames[2] != 'Z' ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    for (std::size_t i = 0U; i < 8U; ++i)
        if (CoordSys.isMirrorActive[i]) return false;
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::ArmPathCoreFeedSameThread() noexcept
{
    m_pathFeed = PathFeedState{};
    m_pathFeedMotion.receipt.Clear();
    m_pathFeed.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathFeed.cache = GetBaseProgramCache().GetGeneration();
    m_pathFeed.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_pathFeed.armed = m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY &&
        m_pathFeed.run != 0ULL && m_pathFeed.cache != 0ULL && m_programMotionLease.IsValid();
}

NC_PATH_FEED_NOINLINE
void NCManager::InvalidatePathCoreFeedSameThread() noexcept
{
    const bool wasPending = m_pathFeed.pending;
    m_pathFeed.armed = false;
    m_pathFeed.pending = false;
    m_pathFeed.bound = false;
    m_pathFeed.completed = false;
    m_pathFeed.explicitFeed = false;
    m_pathFeedMotion.receipt.valid = false;
    if (wasPending)
    {
        m_pathFeed.code = 12U;
        LogPathCoreFeedSameThread("INVALIDATED");
    }
}

NC_PATH_FEED_NOINLINE
void NCManager::ValidatePathCoreFeedSameThread()
{
    if (!m_pathFeed.armed) return;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        m_pathFeed.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathFeed.cache != GetBaseProgramCache().GetGeneration() || !m_macroStack.empty() ||
        Homing.IsActive() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        InvalidatePathCoreFeedSameThread();
        return;
    }
    if (!m_pathFeed.pending) return;
    if (!IsPathCoreFeedConfigurationValid() ||
        m_pathFeedMotion.receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathFeedMotion.receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !FeedLedgerTransportHealthy(m_blockLifecycleLedger))
        RejectPathCoreFeedSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_FEED_NOINLINE
void NCManager::BeginPathCoreFeedCaptureSameThread(const NCBlock& block,
    NCBlockDispatchId dispatchId) noexcept
{
    if (NCGCodeSemantics::Contains(block, 0)) m_pathFeed.explicitFeed = false;
    if (!NCGCodeSemantics::Contains(block, 1)) return;
    // End the old G00-only data service explicitly, without faulting its store
    // or using its receipt/return cursor as a G01 execution permit.
    ClosePathCoreCommittedRunSameThread();
    if (m_pathFeed.pending) return; // Start rejects; never overwrite a live receipt.
    m_pathFeedMotion.receipt.Clear();
    m_pathFeed.dispatch = dispatchId;
    m_pathFeed.commit = 0ULL;
    m_pathFeed.sourcePC = -1;
    m_pathFeed.sourceLine = 0;
    m_pathFeed.bound = false;
    m_pathFeed.consumerAccepted = false;
    m_pathFeed.consumerStarted = false;
    m_pathFeed.completed = false;
    m_pathFeed.code = 0U;
}

NC_PATH_FEED_NOINLINE
void NCManager::RejectPathCoreFeedSameThread(std::uint32_t code, int alarmCode)
{
    if (alarmCode == AlarmManager::G_Code_Invalid_parameter)
    {
        if (code == 3U || code == 4U) alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
        else if (code == 5U) alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
        else if (code == 6U)
        {
            if (m_pathFeedMotion.receipt.code == MotionFeedLineCode::NOT_READY)
                alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
            else if (m_pathFeedMotion.receipt.code == MotionFeedLineCode::GEOMETRY_REJECTED)
                alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
            else alarmCode = AlarmManager::PATH_MOTION_NOT_ADMITTED;
        }
    }
    RtPrintf("[PCORE-ALARM] alarm=%d unit=FEED run=%llu dispatch=%llu code=%u pc=%d line=%d producer=%u geometry=%u accepted=%u\n",
        alarmCode, static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
        static_cast<unsigned int>(code), m_pathFeed.sourcePC, m_pathFeed.sourceLine,
        static_cast<unsigned int>(m_pathFeedMotion.receipt.code),
        static_cast<unsigned int>(m_pathFeedMotion.receipt.geometryCode),
        m_pathFeedMotion.receipt.commandAccepted ? 1U : 0U);
    m_pathFeed.code = code;
    if (m_pathFeedMotion.receipt.commandAccepted) ++m_pathFeed.failed;
    else ++m_pathFeed.rejected;
    m_pathFeed.pending = false;
    m_pathFeed.bound = false;
    m_pathFeed.completed = false;
    m_pathFeedMotion.receipt.valid = false;
    LogPathCoreFeedSameThread(m_pathFeedMotion.receipt.commandAccepted ? "FAILED" : "REJECTED");
    AlarmManager::GetInstance().Trigger(alarmCode, m_pathFeed.sourceLine);
    ChangeState(NCState::ALARM);
}

NC_PATH_FEED_NOINLINE
WaitConditionFunc NCManager::StartPathCoreFeedSameThread(const NCBlock& block)
{
    if (!IsPathCoreFeedBlockShapeValid(block))
    {
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (!m_pathFeed.armed || m_pathFeed.pending || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        !IsPathCoreFeedConfigurationValid() || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathFeed.dispatch != m_currentExecutingBlockDispatchId ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U)
    {
        RejectPathCoreFeedSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    m_pathFeedProgrammed.fill(false);
    m_pathFeedWCS.fill(0.0);
    m_pathFeedCandidate.fill(0.0);
    m_pathFeedAxes.clear();
    m_pathFeedTargets.clear();
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        const char letter = m_axisNames[i];
        if (!block.has(letter)) continue;
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != AxisType::LINEAR)
        {
            RejectPathCoreFeedSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        m_pathFeedProgrammed[i] = true;
        m_pathFeedWCS[i] = block.val(letter);
    }
    CoordSys.Preview_WCS_to_MCS(m_pathFeedWCS.data(), m_pathFeedProgrammed.data(), m_pathFeedCandidate.data());
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        if (!m_pathFeedProgrammed[i]) continue;
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!CoordSys.IsTargetWithinSoftwareTravelLimit(axis, m_pathFeedCandidate[i]))
        {
            RejectPathCoreFeedSameThread(7U, AlarmManager::PROGRAMMED_OVER_TRAVEL);
            return nullptr;
        }
        m_pathFeedAxes.push_back(static_cast<int>(i));
        m_pathFeedTargets.push_back(m_pathFeedCandidate[i]);
    }
    const bool accepted = m_motion.TryG01MoveTransactionalTail(m_pathFeedAxes, m_pathFeedTargets,
        block.val('F'), CoordSys.commandedMCS, m_pathFeedMotion);
    if (!accepted)
    {
        RejectPathCoreFeedSameThread(6U, m_pathFeedMotion.receipt.commandAccepted ?
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const MotionFeedLineReceipt& receipt = m_pathFeedMotion.receipt;
    std::uint32_t expectedMask = 0U;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (m_pathFeedProgrammed[i])
        {
            expectedMask |= (1U << i);
            if (FeedDoubleBits(m_pathFeedCandidate[i]) != FeedDoubleBits(receipt.line.endMCS[i]))
            {
                RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
        }
        else if (FeedDoubleBits(receipt.line.startMCS[i]) != FeedDoubleBits(receipt.line.endMCS[i]))
        {
            RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    if (!receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted || !receipt.captureBound ||
        !receipt.line.valid || receipt.line.axisMask != expectedMask ||
        FeedDoubleBits(receipt.line.feedMMMin) != FeedDoubleBits(block.val('F')) ||
        (receipt.validAxisMask & expectedMask) != expectedMask ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    ++m_pathFeed.submitted;
    m_pathFeed.pending = true;
    m_pathFeed.explicitFeed = true;
    m_pathFeed.code = 1U;
    LogPathCoreFeedSameThread("SUBMITTED");
    LogPathCoreFeedGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreFeedSameThread(); };
}

NC_PATH_FEED_NOINLINE
void NCManager::CommitPathCoreFeedCaptureSameThread(NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
    bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
    int sourcePC, int sourceLine)
{
    if (!m_pathFeed.pending || m_pathFeed.dispatch != dispatchId) return;
    m_pathFeed.sourcePC = sourcePC;
    m_pathFeed.sourceLine = sourceLine;
    const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
    if (!r.valid || !committed || !commit.IsValid() || !ledgerFound ||
        capture.count != 1U || capture.overflow || ledger.dispatchId != dispatchId ||
        !ledger.programCommitted || ledger.ncDispatchFailed || ledger.motionCaptureOverflow ||
        ledger.motionSegmentCount != 1U || ledger.sourceLineNumber != sourceLine ||
        sourcePC < 0 || sourceLine <= 0 || commit.sourcePC != sourcePC ||
        commit.scope != NCProgramScope::MEMORY || commit.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        commit.cacheGeneration != m_pathFeed.cache || !FeedSameSource(commit, ledger.programTarget) ||
        !FeedSameSource(commit, ledger.programCommit) || commit.sequence != ledger.programCommit.sequence ||
        !capture.submissions[0U].producerAccepted ||
        capture.submissions[0U].immediateRejectReason != MotionRejectReason::NONE ||
        capture.submissions[0U].commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        !ledger.motionSegments[0U].producerAccepted ||
        ledger.motionSegments[0U].immediateRejectReason != MotionRejectReason::NONE ||
        !FeedFullIdentity(capture.submissions[0U].identity, r.identity) ||
        !FeedFullIdentity(ledger.motionSegments[0U].identity, r.identity) ||
        r.identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
        r.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !r.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreFeedSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathFeed.commit = commit.sequence;
    m_pathFeed.bound = true;
    LogPathCoreFeedSameThread("BOUND");
}

NC_PATH_FEED_NOINLINE
void NCManager::ObservePathCoreFeedFeedbackSameThread(const MotionFeedbackEvent& event,
    bool ledgerAccepted)
{
    if (!m_pathFeed.pending || !m_pathFeed.bound ||
        !FeedFullIdentity(event.identity, m_pathFeedMotion.receipt.identity)) return;
    if (!ledgerAccepted || event.sequence == 0ULL || event.sequence <= m_pathFeed.lastSequence ||
        event.owner != m_pathFeedMotion.receipt.ownerLease.owner ||
        event.ownerGeneration != m_pathFeedMotion.receipt.ownerLease.generation)
    {
        RejectPathCoreFeedSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathFeed.lastSequence = event.sequence;
    switch (event.type)
    {
    case MotionFeedbackType::ACCEPTED:
        if (m_pathFeed.consumerAccepted) break;
        m_pathFeed.consumerAccepted = true;
        ++m_pathFeed.accepted;
        return;
    case MotionFeedbackType::STARTED:
        if (!m_pathFeed.consumerAccepted || m_pathFeed.consumerStarted) break;
        m_pathFeed.consumerStarted = true;
        ++m_pathFeed.started;
        return;
    case MotionFeedbackType::COMPLETED:
        if (!m_pathFeed.consumerAccepted || m_pathFeed.completed || event.rejectReason != MotionRejectReason::NONE || event.errorCode != 0U) break;
        m_pathFeed.completed = true;
        return;
    case MotionFeedbackType::PROGRESS:
    case MotionFeedbackType::HELD:
    case MotionFeedbackType::RESUMED:
        if (m_pathFeed.consumerAccepted) return;
        break;
    default:
        break;
    }
    RejectPathCoreFeedSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_FEED_NOINLINE
bool NCManager::CompletePathCoreFeedSameThread()
{
    if (!m_pathFeed.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (m_state == NCState::ALARM) return false;
    if (!m_pathFeed.pending) return m_pathFeed.completed;
    if (IsFeedHoldActive()) return false;
    ValidatePathCoreFeedSameThread();
    if (!m_pathFeed.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreFeedSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (!m_pathFeed.bound || !m_pathFeed.completed || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone()) return false;
    RetainPathCoreFeedSameThread(); // BZ: only after the real completion gate.
    m_pathFeed.pending = false;
    ++m_pathFeed.done;
    LogPathCoreFeedSameThread("COMPLETED");
    return true;
}

NC_PATH_FEED_NOINLINE
void NCManager::FinalizePathCoreFeedSameThread() noexcept
{
    if (m_pathFeed.submitted != 0U) LogPathCoreFeedSameThread("FINALIZED");
    m_pathFeed.armed = false;
    m_pathFeed.explicitFeed = false;
    m_pathFeedMotion.receipt.valid = false;
}

NC_PATH_FEED_NOINLINE
void NCManager::LogPathCoreFeedSameThread(const char* phase) const noexcept
{
    const PathFeedState& s = m_pathFeed;
    const MotionFeedLineReceipt& r = m_pathFeedMotion.receipt;
    RtPrintf("[PCORE-BX] run=%llu dispatch=%llu phase=%s code=%u pc=%d line=%d commit=%llu pending=%u bound=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
        static_cast<unsigned int>(s.code), s.sourcePC, s.sourceLine,
        static_cast<unsigned long long>(s.commit), s.pending ? 1U : 0U, s.bound ? 1U : 0U);
    RtPrintf("[PCORE-BX-ID] run=%llu dispatch=%llu epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u valid=%u producerCode=%u geometryCode=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(r.identity.segmentId),
        static_cast<unsigned int>(r.identity.sourceBlockId), static_cast<unsigned int>(r.identity.source),
        static_cast<unsigned int>(r.ownerLease.owner), static_cast<unsigned int>(r.ownerLease.generation),
        r.valid ? 1U : 0U, static_cast<unsigned int>(r.code), static_cast<unsigned int>(r.geometryCode));
    RtPrintf("[PCORE-BX-CNT] run=%llu submitted=%u accepted=%u started=%u done=%u rejected=%u failed=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned int>(s.submitted),
        static_cast<unsigned int>(s.accepted), static_cast<unsigned int>(s.started),
        static_cast<unsigned int>(s.done), static_cast<unsigned int>(s.rejected), static_cast<unsigned int>(s.failed));
}

NC_PATH_FEED_NOINLINE
void NCManager::LogPathCoreFeedGeometrySameThread() const noexcept
{
    const NCPathCoreFeedLineV2& g = m_pathFeedMotion.receipt.line;
    RtPrintf("[PCORE-BX-GEO] run=%llu dispatch=%llu mask=%u validMask=%u FBits=%llu lengthMMBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu point=%u\n",
        static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
        static_cast<unsigned int>(g.axisMask), static_cast<unsigned int>(m_pathFeedMotion.receipt.validAxisMask),
        static_cast<unsigned long long>(FeedDoubleBits(g.feedMMMin)),
        static_cast<unsigned long long>(FeedDoubleBits(g.lengthMM)),
        static_cast<unsigned long long>(FeedDoubleBits(g.lengthPulse)),
        static_cast<unsigned long long>(FeedDoubleBits(g.velocityPPS)), g.point ? 1U : 0U);
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        const bool selected = (g.axisMask & (1U << i)) != 0U;
        RtPrintf("[PCORE-BX-AXIS] run=%llu dispatch=%llu axis=%u selected=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
            static_cast<unsigned long long>(m_pathFeed.run), static_cast<unsigned long long>(m_pathFeed.dispatch),
            static_cast<unsigned int>(i), selected ? 1U : 0U,
            static_cast<unsigned long long>(FeedDoubleBits(g.startMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.endMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(selected ? m_pathFeedCandidate[i] : g.startMCS[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.startPulse[i])),
            static_cast<unsigned long long>(FeedDoubleBits(g.endPulse[i])));
    }
}
#undef NC_PATH_FEED_NOINLINE
