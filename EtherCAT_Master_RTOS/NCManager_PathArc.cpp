// BY / Path Core V2-02. Explicit G17 XY arcs use a distinct exact-stop producer.
#include "NCManager.h"
#include "AlarmManager.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <windows.h>
#include <rtapi.h>

#if defined(_MSC_VER)
#define NC_PATH_ARC_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_ARC_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_ARC_NOINLINE
#endif

namespace
{
    bool ArcTranslationCurrent(const CoordinateManager& coord, std::uint64_t generation) noexcept
    {
        return generation != 0ULL && coord.IsTranslationRunFrozen() &&
            coord.IsTranslationRunCurrent() && coord.GetTranslationSnapshot().generation == generation;
    }
    bool ArcFullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.IsAssigned() && b.IsAssigned() && a.epoch == b.epoch &&
            a.segmentId == b.segmentId && a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
    bool ArcSameSource(const NCProgramCommitSnapshot& a,
        const NCProgramCommitSnapshot& b) noexcept
    {
        return a.scope == b.scope && a.cacheGeneration == b.cacheGeneration &&
            a.frameId == b.frameId && a.sourcePC == b.sourcePC;
    }
    NC_PATH_ARC_NOINLINE
        bool ArcLedgerTransportHealthy(const NCBlockLifecycleLedger& ledger) noexcept
    {
        const NCBlockLifecycleCounters counters = ledger.GetCounters();
        return counters.activeBlockOverwrite == 0ULL &&
            counters.activeSegmentIndexOverwrite == 0ULL && counters.orphanFeedback == 0ULL &&
            counters.duplicateTerminalFeedback == 0ULL && counters.terminalFeedbackConflict == 0ULL;
    }
    std::uint64_t ArcDoubleBits(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

NC_PATH_ARC_NOINLINE
bool NCManager::IsPathCoreArcBlockShapeValid(const NCBlock& block, bool allowMissingFeed, int unitsMode) noexcept
{
    if (unitsMode != 20 && unitsMode != 21) return false;
    const double feedMMMin = block.has('F') ? NCTranslationLengthToMM(block.val('F'), unitsMode) : 0.0;
    if (block.isEmpty || block.isGoto || !block.hasG ||
        (block.gCode != 2 && block.gCode != 3) || block.gCount != 1 ||
        block.gCodes[0] != block.gCode || block.mCount != 0 ||
        (!block.has('I') && !block.has('J')) ||
        (!block.has('F') && !allowMissingFeed) ||
        (block.has('F') && (!std::isfinite(feedMMMin) ||
            feedMMMin <= 0.0 || feedMMMin > 100.0))) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && address != 'F' && address != 'X' && address != 'Y' && address != 'I' && address != 'J' && address != 'P') return false;
        if (!std::isfinite(block.val(address))) return false;
        if ((address == 'X' || address == 'Y' || address == 'I' || address == 'J') &&
            !std::isfinite(NCTranslationLengthToMM(block.val(address), unitsMode))) return false;
    }
    // P1 is this controller's explicit queued planar-transition request;
    // EF: either endpoint word may be omitted; only both omitted is a full circle.
    // P remains a queued transition request, not a turn count.
    if (block.has('P') && block.val('P') != 1.0) return false;
    return true;
}

NC_PATH_ARC_NOINLINE
bool NCManager::IsPathCoreArcInputOmission(const NCBlock& block) const noexcept
{
    return m_pathArc.armed && m_pathArc.explicitArc && !block.hasG && block.gCount == 0 &&
        (block.has('X') || block.has('Y') || block.has('Z') || block.has('F') || block.has('I') || block.has('J'));
}

NC_PATH_ARC_NOINLINE
bool NCManager::IsPathCoreArcConfigurationValid() noexcept
{
    const NCTranslationSnapshot translation = CoordSys.GetTranslationSnapshot();
    if (!IsNCTranslationDistanceModeAllowed(CoordSys.isAbsoluteMode, translation) ||
        !IsNCTranslationUnitModeAllowed(CoordSys.isInchMode, translation) || !IsNCTranslationSourceAllowed(CoordSys.GetCurrentWCSGCode(), translation) ||
        CoordSys.activePlane != 17 || !IsNCTranslationToolModeAllowed(CoordSys.toolLengthMode, translation) ||
        CoordSys.currentHCode != translation.toolHCode || CoordSys.toolRadiusMode != 40 ||
        !IsNCTranslationRotationModeAllowed(CoordSys.isG68Active, CoordSys.g68Angle,
            CoordSys.activePlane, translation) ||
        !IsNCTranslationWorkModeAllowed(CoordSys.isWorkpieceRotationActive, CoordSys.currentWCode, translation) ||
        !IsNCTranslationScaleMirrorModeAllowed(CoordSys.isScalingActive, CoordSys.isMirrorActive, translation) ||
        CoordSys.isPolarCoordinateActive || CoordSys.isCAxisOffsetRotationEnabled ||
        m_axisNames[0] != 'X' || m_axisNames[1] != 'Y' || m_axisNames[2] != 'Z' ||
        !IsPathCoreLiveNativeConfigCurrentSameThread()) return false;
    return true;
}

NC_PATH_ARC_NOINLINE
void NCManager::ArmPathCoreArcSameThread() noexcept
{
    ClearCncModalFeedSameThread();
    m_pathArc = PathArcState{};
    m_pathArcMotion.receipt.Clear();
    m_pathArc.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_pathArc.cache = GetBaseProgramCache().GetGeneration();
    m_pathArc.lastSequence = m_lastConsumedMotionFeedbackSequence;
    m_pathArc.armed = m_state == NCState::RUN && m_mode == NCOperationMode::MEMORY &&
        m_pathArc.run != 0ULL && m_pathArc.cache != 0ULL && m_programMotionLease.IsValid();
}

NC_PATH_ARC_NOINLINE
void NCManager::InvalidatePathCoreArcSameThread(bool byGoto) noexcept
{
    ClearCncModalFeedSameThread();
    const bool wasPending = m_pathArc.pending;
    m_pathArc.armed = false;
    m_pathArc.invalidatedByGoto = byGoto;
    m_pathArc.pending = false;
    m_pathArc.bound = false;
    m_pathArc.completed = false;
    m_pathArc.explicitArc = false;
    m_pathArcMotion.receipt.valid = false;
    if (wasPending)
    {
        m_pathArc.code = 12U;
        LogPathCoreArcSameThread("INVALIDATED");
    }
}

NC_PATH_ARC_NOINLINE
void NCManager::ValidatePathCoreArcSameThread()
{
    if (!m_pathArc.armed) return;
    const NCState state = m_state.load(std::memory_order_acquire);
    if (Close_System_Com_flag || (state != NCState::RUN && state != NCState::HOLD) ||
        m_mode != NCOperationMode::MEMORY || AlarmManager::GetInstance().HasAlarm() ||
        m_pathArc.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_pathArc.cache != GetBaseProgramCache().GetGeneration() || !m_macroStack.empty() ||
        Homing.IsActive() || !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        InvalidatePathCoreArcSameThread();
        return;
    }
    if (!m_pathArc.pending) return;
    if (!IsPathCoreArcConfigurationValid() ||
        !ArcTranslationCurrent(CoordSys, m_pathArcMotion.receipt.translationGeneration) ||
        m_pathArcMotion.receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_pathArcMotion.receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreArcSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    if (m_motion.GetMotionFeedbackOverflowCount() != 0ULL ||
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() != 0ULL ||
        m_motionFeedbackSequenceGapCount != 0ULL || !ArcLedgerTransportHealthy(m_blockLifecycleLedger))
        RejectPathCoreArcSameThread(11U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_ARC_NOINLINE
void NCManager::BeginPathCoreArcCaptureSameThread(const NCBlock& block,
    NCBlockDispatchId dispatchId) noexcept
{
    const bool arc = NCGCodeSemantics::Contains(block, 2) || NCGCodeSemantics::Contains(block, 3);
    if (NCGCodeSemantics::Contains(block, 0) || NCGCodeSemantics::Contains(block, 1))
        m_pathArc.explicitArc = false;
    if (!arc) return;
    // A cross-producer transition retains the fresh-run permit, but never an
    // earlier G00 return cursor or a G01 implicit-input interpretation.
    if (m_pathArc.pending || m_pathFeed.pending) return;
    m_pathFeed.explicitFeed = false;
    ClosePathCoreCommittedRunSameThread();
    m_pathArcMotion.receipt.Clear();
    m_pathArc.dispatch = dispatchId;
    m_pathArc.commit = 0ULL;
    m_pathArc.capturedFeed = CncFeedValueSnapshot{};
    m_pathArc.sourcePC = -1;
    m_pathArc.sourceLine = 0;
    m_pathArc.bound = false;
    m_pathArc.consumerAccepted = false;
    m_pathArc.consumerStarted = false;
    m_pathArc.completed = false;
    m_pathArc.code = 0U;
}

NC_PATH_ARC_NOINLINE
void NCManager::RejectPathCoreArcSameThread(std::uint32_t code, int alarmCode)
{
    if (alarmCode == AlarmManager::G_Code_Invalid_parameter)
    {
        if (code == 3U || code == 4U) alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
        else if (code == 5U) alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
        else if (code == 6U)
        {
            if (m_pathArcMotion.receipt.code == MotionFeedArcCode::NOT_READY)
                alarmCode = AlarmManager::PATH_EXECUTION_NOT_READY;
            else if (m_pathArcMotion.receipt.code == MotionFeedArcCode::GEOMETRY_REJECTED)
                alarmCode = AlarmManager::PATH_GEOMETRY_INVALID;
            else alarmCode = AlarmManager::PATH_MOTION_NOT_ADMITTED;
        }
    }
    RtPrintf("[PCORE-ALARM] alarm=%d unit=ARC run=%llu dispatch=%llu code=%u pc=%d line=%d producer=%u geometry=%u accepted=%u\n",
        alarmCode, static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
        static_cast<unsigned int>(code), m_pathArc.sourcePC, m_pathArc.sourceLine,
        static_cast<unsigned int>(m_pathArcMotion.receipt.code),
        static_cast<unsigned int>(m_pathArcMotion.receipt.geometryCode),
        m_pathArcMotion.receipt.commandAccepted ? 1U : 0U);
    if (alarmCode == AlarmManager::PATH_INVALIDATED_BY_GOTO)
        RtPrintf("[PCORE-CAUSE] alarm=2023 unit=ARC reason=GOTO_INVALIDATED action=EDIT_FLOW_RESET_RESTART pc=%d line=%d\n",
            m_pathArc.sourcePC, m_pathArc.sourceLine);
    m_pathArc.code = code;
    if (m_pathArcMotion.receipt.commandAccepted) ++m_pathArc.failed;
    else ++m_pathArc.rejected;
    m_pathArc.pending = false;
    m_pathArc.bound = false;
    m_pathArc.completed = false;
    m_pathArcMotion.receipt.valid = false;
    LogPathCoreArcSameThread(m_pathArcMotion.receipt.commandAccepted ? "FAILED" : "REJECTED");
    AlarmManager::GetInstance().Trigger(alarmCode, m_pathArc.sourceLine);
    ChangeState(NCState::ALARM);
}

NC_PATH_ARC_NOINLINE
WaitConditionFunc NCManager::StartPathCoreArcSameThread(const NCBlock& block)
{
    if (!IsPathCoreArcBlockShapeValid(block, true, CoordSys.isInchMode ? 20 : 21))
    {
        RejectPathCoreArcSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const bool queued = m_cncFeed.selected && block.has('P');
    const bool buffered = queued && m_cncFeed.active;
    if ((block.has('P') && !queued) || (queued && (!IsCncFeedSelectedBlockSameThread(block) ||
        !IsCncFeedScopeSameThread() || m_cncFeed.count >= m_cncFeed.flights.size())))
    {
        RejectPathCoreArcSameThread(3U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (!m_pathArc.armed || m_pathArc.pending || m_pathFeed.pending || m_state != NCState::RUN ||
        m_mode != NCOperationMode::MEMORY || m_isG66Active || !m_macroStack.empty() || Homing.IsActive() ||
        !IsPathCoreArcConfigurationValid() || AlarmManager::GetInstance().HasAlarm() ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathArc.dispatch != m_currentExecutingBlockDispatchId ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        (!buffered && (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
            m_motion.GetCommandReplaySize() != 0U)))
    {
        // Preserve the original admission failure and stop path. Only its
        // operator alarm differs when this permit was revoked by a taken GOTO.
        RejectPathCoreArcSameThread(3U,
            (!m_pathArc.armed && m_pathArc.invalidatedByGoto) ?
                AlarmManager::PATH_INVALIDATED_BY_GOTO : AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // Selected candidates retain their resolved source and F provenance.
    // Ordinary motion captures only after its original execution scope gates.
    if (queued) m_pathArc.capturedFeed = m_cncFeed.candidateFeed;
    else if (!CaptureCncFeedValueSameThread(block, m_pathArc.capturedFeed))
    {
        RejectPathCoreArcSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    if (!IsCncFeedValueCurrentSameThread(m_pathArc.capturedFeed))
    {
        RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    const double effectiveFeed = m_pathArc.capturedFeed.feedMMMin;
    const std::uint32_t sourceEndpointAxisMask = (block.has('X') ? 1U : 0U) |
        (block.has('Y') ? 2U : 0U);
    const bool fullCircle = sourceEndpointAxisMask == 0U;
    const NCTranslationSnapshot arcSource = CoordSys.GetTranslationSnapshot();
    const bool mirroredXY = ((arcSource.mirrorMask & 1U) != 0U) !=
        ((arcSource.mirrorMask & 2U) != 0U);
    const int direction = (block.gCode == 2 ? -1 : 1) * (mirroredXY ? -1 : 1);
    m_pathArcProgrammed.fill(false);
    m_pathArcWCS.fill(0.0);
    m_pathArcCandidate.fill(0.0);
    // EB: omitted I or J is +0 for this source, never a modal carry.
    // Read only present parameter slots; keep original presence bits intact.
    m_pathArcCenterOffset[0U] = block.has('I') ?
        NCTranslationLengthToMM(block.val('I'), CoordSys.isInchMode ? 20 : 21) : 0.0;
    m_pathArcCenterOffset[1U] = block.has('J') ?
        NCTranslationLengthToMM(block.val('J'), CoordSys.isInchMode ? 20 : 21) : 0.0;
    // I/J are program-space vectors. Apply signed scale then rotate in mm, without
    // adding the G68 center or EXT/WCS/H/WORK translation. Native arc start
    // and full-circle endpoint bits remain owned by the Motion producer.
    double rotatedI = 0.0, rotatedJ = 0.0;
    NCTranslationRotateXYVector(CoordSys.GetTranslationSnapshot(),
        m_pathArcCenterOffset[0U], m_pathArcCenterOffset[1U], rotatedI, rotatedJ);
    m_pathArcCenterOffset[0U] = rotatedI;
    m_pathArcCenterOffset[1U] = rotatedJ;
    if (!std::isfinite(rotatedI) || !std::isfinite(rotatedJ))
    {
        RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (std::size_t i = 0U; i < 2U; ++i)
    {
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != AxisType::LINEAR)
        {
            RejectPathCoreArcSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        m_pathArcProgrammed[i] = block.has(m_axisNames[i]);
        if (m_pathArcProgrammed[i]) m_pathArcWCS[i] =
            NCTranslationLengthToMM(block.val(m_axisNames[i]), CoordSys.isInchMode ? 20 : 21);
    }
    const bool requirePlanarBaselineMatch = CoordSys.IsTranslationRunFrozen() &&
        NCTranslationHasPlanarRotation(CoordSys.GetTranslationSnapshot()) &&
        (m_pathArcProgrammed[0] != m_pathArcProgrammed[1]);
    if (!CoordSys.CompleteFixedPlanarEndpoint(m_pathArcWCS.data(), m_pathArcProgrammed.data()))
    {
        RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const std::uint32_t endpointAxisMask = (m_pathArcProgrammed[0] ? 1U : 0U) |
        (m_pathArcProgrammed[1] ? 2U : 0U);
    if (requirePlanarBaselineMatch)
        RtPrintf("[ROTATION][SPARSE_XY] g=%d rawXYMask=%u effectiveXYMask=3 beforeSubmit=1\n",
            block.gCode, sourceEndpointAxisMask);
    if (!CoordSys.isAbsoluteMode)
        RtPrintf("[INCREMENTAL][TARGET] g=%d rawMask=%u effectiveMask=%u fullCircle=%u beforeSubmit=1\n",
            block.gCode, static_cast<unsigned>(sourceEndpointAxisMask),
            static_cast<unsigned>(endpointAxisMask), fullCircle ? 1U : 0U);
    // Full-circle classification uses original presence, never the effective
    // rotated endpoint mask. I/J above remain vectors in program space.
    CoordSys.Preview_WCS_to_MCS(m_pathArcWCS.data(), m_pathArcProgrammed.data(), m_pathArcCandidate.data());
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(m_pathArcCandidate[i]))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    MotionArcTravelGuard travelGuard{};
    travelGuard.context = this;
    travelGuard.check = [](const void* context, int axisIndex, double target) -> bool
    {
        const NCManager* nc = static_cast<const NCManager*>(context);
        return axisIndex >= 0 && axisIndex < 2 &&
            nc->CoordSys.IsTargetWithinSoftwareTravelLimit(nc->m_motion.GetAxisContext(axisIndex), target);
    };
    const bool accepted = m_motion.TryG02G03MoveTransactionalCncTail(m_pathArcCandidate,
        m_pathArcCenterOffset, direction, fullCircle, effectiveFeed, travelGuard,
        CoordSys.commandedMCS, m_pathArcMotion, m_pathArcCommand,
        buffered ? &m_cncFeed.tail : nullptr, queued, endpointAxisMask, requirePlanarBaselineMatch);
    if (!accepted)
    {
        RejectPathCoreArcSameThread(6U, m_pathArcMotion.receipt.commandAccepted ?
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY :
            (m_pathArcMotion.receipt.travelLimitRejected ? static_cast<int>(AlarmManager::PROGRAMMED_OVER_TRAVEL) :
                static_cast<int>(AlarmManager::G_Code_Invalid_parameter)));
        return nullptr;
    }
    const MotionFeedArcReceipt& receipt = m_pathArcMotion.receipt;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (i < 2U && m_pathArcProgrammed[i])
        {
            if (ArcDoubleBits(m_pathArcCandidate[i]) != ArcDoubleBits(receipt.arc.endMCS[i]))
            {
                RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
                return nullptr;
            }
        }
        else if (ArcDoubleBits(receipt.arc.startMCS[i]) != ArcDoubleBits(receipt.arc.endMCS[i]) ||
            ArcDoubleBits(receipt.arc.startPulse[i]) != ArcDoubleBits(receipt.arc.endPulse[i]))
        {
            RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    for (std::size_t i = 0U; i < 2U; ++i)
    {
        const double expectedCenter = receipt.arc.startMCS[i] + m_pathArcCenterOffset[i];
        if (!std::isfinite(expectedCenter) ||
            ArcDoubleBits(receipt.arc.centerMCS[i]) != ArcDoubleBits(expectedCenter))
        {
            RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    if (!receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted || !receipt.captureBound ||
        !ArcTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        receipt.travelLimitRejected || !receipt.arc.valid || receipt.arc.axisMask != 3U ||
        receipt.arc.direction != direction || receipt.arc.fullCircle != fullCircle ||
        ArcDoubleBits(receipt.arc.feedMMMin) != ArcDoubleBits(effectiveFeed) ||
        (receipt.validAxisMask & 3U) != 3U || !receipt.identity.IsAssigned() ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    ++m_pathArc.submitted;
    m_pathArc.pending = true;
    m_pathArc.explicitArc = true;
    m_pathArc.code = 1U;
    if (queued) return nullptr;
    LogPathCoreArcSameThread("SUBMITTED");
    LogPathCoreArcGeometrySameThread();
    return [](NCManager* nc) { return nc->CompletePathCoreArcSameThread(); };
}

NC_PATH_ARC_NOINLINE
void NCManager::CommitPathCoreArcCaptureSameThread(NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
    bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
    int sourcePC, int sourceLine)
{
    if (!m_pathArc.pending || m_pathArc.dispatch != dispatchId) return;
    m_pathArc.sourcePC = sourcePC;
    m_pathArc.sourceLine = sourceLine;
    const MotionFeedArcReceipt& r = m_pathArcMotion.receipt;
    // Ledger owner fields are populated by feedback, after this commit.
    // Bind the receipt to the current program lease here; Observe validates
    // each feedback event's owner and generation when it is consumed.
    if (!IsCncFeedValueCurrentSameThread(m_pathArc.capturedFeed) ||
        ArcDoubleBits(r.arc.feedMMMin) != ArcDoubleBits(m_pathArc.capturedFeed.feedMMMin) ||
        !r.valid || !committed || !commit.IsValid() || !ledgerFound ||
        capture.count != 1U || capture.overflow || ledger.dispatchId != dispatchId ||
        !ledger.programCommitted || ledger.ncDispatchFailed || ledger.motionCaptureOverflow ||
        ledger.motionSegmentCount != 1U || ledger.sourceLineNumber != sourceLine ||
        sourcePC < 0 || sourceLine <= 0 || commit.sourcePC != sourcePC ||
        commit.scope != NCProgramScope::MEMORY || commit.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        commit.cacheGeneration != m_pathArc.cache || !ArcSameSource(commit, ledger.programTarget) ||
        !ArcSameSource(commit, ledger.programCommit) || commit.sequence != ledger.programCommit.sequence ||
        !ArcTranslationCurrent(CoordSys, r.translationGeneration) ||
        capture.submissions[0U].translationGeneration != r.translationGeneration ||
        !capture.submissions[0U].producerAccepted ||
        capture.submissions[0U].immediateRejectReason != MotionRejectReason::NONE ||
        capture.submissions[0U].commandPathMode != (m_cncFeed.selected ?
            MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP) ||
        !ledger.motionSegments[0U].producerAccepted ||
        ledger.motionSegments[0U].immediateRejectReason != MotionRejectReason::NONE ||
        !ArcFullIdentity(capture.submissions[0U].identity, r.identity) ||
        !ArcFullIdentity(ledger.motionSegments[0U].identity, r.identity) ||
        r.identity.sourceBlockId != static_cast<MotionSourceBlockId>(sourcePC) ||
        r.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !r.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreArcSameThread(9U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathArc.commit = commit.sequence;
    m_pathArc.bound = true;
    RtPrintf("[CNC-TRANSLATION-PATH] phase=BOUND kind=ARC translationGen=%llu wcs=%d dispatch=%llu epoch=%llu seg=%llu sourcePC=%d\n",
        static_cast<unsigned long long>(r.translationGeneration), CoordSys.GetTranslationSnapshot().wcsCode,
        static_cast<unsigned long long>(dispatchId), static_cast<unsigned long long>(r.identity.epoch),
        static_cast<unsigned long long>(r.identity.segmentId), sourcePC);
    if (m_cncFeed.selected)
    {
        CommitCncArcSameThread();
        return;
    }
    CommitCncModalFeedSameThread(m_pathArc.capturedFeed, dispatchId, commit.sequence, sourcePC, sourceLine);
    LogPathCoreArcSameThread("BOUND");
}

NC_PATH_ARC_NOINLINE
void NCManager::ObservePathCoreArcFeedbackSameThread(const MotionFeedbackEvent& event,
    bool ledgerAccepted)
{
    if (!m_pathArc.pending || !m_pathArc.bound ||
        !ArcFullIdentity(event.identity, m_pathArcMotion.receipt.identity)) return;
    if (!ArcTranslationCurrent(CoordSys, m_pathArcMotion.receipt.translationGeneration) ||
        !ledgerAccepted || event.sequence == 0ULL || event.sequence <= m_pathArc.lastSequence ||
        event.owner != m_pathArcMotion.receipt.ownerLease.owner ||
        event.ownerGeneration != m_pathArcMotion.receipt.ownerLease.generation)
    {
        RejectPathCoreArcSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return;
    }
    m_pathArc.lastSequence = event.sequence;
    switch (event.type)
    {
    case MotionFeedbackType::ACCEPTED:
        if (m_pathArc.consumerAccepted) break;
        m_pathArc.consumerAccepted = true;
        ++m_pathArc.accepted;
        return;
    case MotionFeedbackType::STARTED:
        if (!m_pathArc.consumerAccepted || m_pathArc.consumerStarted) break;
        m_pathArc.consumerStarted = true;
        ++m_pathArc.started;
        return;
    case MotionFeedbackType::COMPLETED:
        if (!m_pathArc.consumerAccepted || m_pathArc.completed || event.rejectReason != MotionRejectReason::NONE || event.errorCode != 0U) break;
        m_pathArc.completed = true;
        return;
    case MotionFeedbackType::PROGRESS:
    case MotionFeedbackType::HELD:
    case MotionFeedbackType::RESUMED:
        if (m_pathArc.consumerAccepted) return;
        break;
    default:
        break;
    }
    RejectPathCoreArcSameThread(10U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
}

NC_PATH_ARC_NOINLINE
bool NCManager::CompletePathCoreArcSameThread()
{
    if (!m_pathArc.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreArcSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (m_state == NCState::ALARM) return false;
    if (!m_pathArc.pending)
    {
        if (m_gapWindow.active && (m_gapWindow.budgetProven || m_gapWindow.normalProven) && m_pathArc.completed &&
            m_pathHold.bound && m_pathArc.dispatch == m_pathHold.dispatch)
        {
            if (!CompleteGapPathSourceSameThread(false)) return false;
        }
        return m_pathArc.completed;
    }
    if (IsFeedHoldActive()) return false;
    ValidatePathCoreArcSameThread();
    if (!m_pathArc.armed)
    {
        if (m_state == NCState::RUN || m_state == NCState::HOLD)
            RejectPathCoreArcSameThread(12U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return false;
    }
    if (!m_pathArc.bound || !m_pathArc.completed || m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.IsGroupDone()) return false;
    if (m_gapWindow.active && m_gapWindow.normalSource && !m_gapWindow.normalProven) return false;
    RetainPathCoreArcSameThread(); // BZ: only after the real completion gate.
    m_pathArc.pending = false;
    ++m_pathArc.done;
    LogPathCoreArcSameThread("COMPLETED");
    return CompleteGapPathSourceSameThread(false);
}

NC_PATH_ARC_NOINLINE
void NCManager::FinalizePathCoreArcSameThread() noexcept
{
    ClearCncModalFeedSameThread();
    if (m_pathArc.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_pathArc.submitted != 0U) LogPathCoreArcSameThread("FINALIZED");
    m_pathArc.armed = false;
    m_pathArc.invalidatedByGoto = false;
    m_pathArc.explicitArc = false;
    m_pathArcMotion.receipt.valid = false;
}

NC_PATH_ARC_NOINLINE
void NCManager::LogPathCoreArcSameThread(const char* phase) const noexcept
{
    const PathArcState& s = m_pathArc;
    const MotionFeedArcReceipt& r = m_pathArcMotion.receipt;
    RtPrintf("[PCORE-BY] run=%llu dispatch=%llu phase=%s code=%u pc=%d line=%d commit=%llu pending=%u bound=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch), phase,
        static_cast<unsigned int>(s.code), s.sourcePC, s.sourceLine,
        static_cast<unsigned long long>(s.commit), s.pending ? 1U : 0U, s.bound ? 1U : 0U);
    RtPrintf("[PCORE-BY-ID] run=%llu dispatch=%llu epoch=%llu seg=%llu sourcePC=%u source=%u owner=%u gen=%u valid=%u producerCode=%u geometryCode=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned long long>(s.dispatch),
        static_cast<unsigned long long>(r.identity.epoch), static_cast<unsigned long long>(r.identity.segmentId),
        static_cast<unsigned int>(r.identity.sourceBlockId), static_cast<unsigned int>(r.identity.source),
        static_cast<unsigned int>(r.ownerLease.owner), static_cast<unsigned int>(r.ownerLease.generation),
        r.valid ? 1U : 0U, static_cast<unsigned int>(r.code), static_cast<unsigned int>(r.geometryCode));
    RtPrintf("[PCORE-BY-CNT] run=%llu submitted=%u accepted=%u started=%u done=%u rejected=%u failed=%u\n",
        static_cast<unsigned long long>(s.run), static_cast<unsigned int>(s.submitted),
        static_cast<unsigned int>(s.accepted), static_cast<unsigned int>(s.started),
        static_cast<unsigned int>(s.done), static_cast<unsigned int>(s.rejected), static_cast<unsigned int>(s.failed));
}

NC_PATH_ARC_NOINLINE
void NCManager::LogPathCoreArcGeometrySameThread() const noexcept
{
    const NCPathCoreFeedArcV2& g = m_pathArcMotion.receipt.arc;
    RtPrintf("[PCORE-BY-GEO] run=%llu dispatch=%llu mask=%u validMask=%u direction=%d fullCircle=%u FBits=%llu radiusMMBits=%llu radiusPulseBits=%llu startAngleBits=%llu sweepBits=%llu lengthMMBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu\n",
        static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
        static_cast<unsigned int>(g.axisMask), static_cast<unsigned int>(m_pathArcMotion.receipt.validAxisMask),
        g.direction, g.fullCircle ? 1U : 0U,
        static_cast<unsigned long long>(ArcDoubleBits(g.feedMMMin)),
        static_cast<unsigned long long>(ArcDoubleBits(g.radiusMM)),
        static_cast<unsigned long long>(ArcDoubleBits(g.radiusPulse)),
        static_cast<unsigned long long>(ArcDoubleBits(g.startAngle)),
        static_cast<unsigned long long>(ArcDoubleBits(g.sweepRadians)),
        static_cast<unsigned long long>(ArcDoubleBits(g.lengthMM)),
        static_cast<unsigned long long>(ArcDoubleBits(g.lengthPulse)),
        static_cast<unsigned long long>(ArcDoubleBits(g.velocityPPS)));
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        RtPrintf("[PCORE-BY-AXIS] run=%llu dispatch=%llu axis=%u selected=%u programmed=%u startBits=%llu endBits=%llu targetBits=%llu startPulseBits=%llu endPulseBits=%llu\n",
            static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
            static_cast<unsigned int>(i), i < 2U ? 1U : 0U, m_pathArcProgrammed[i] ? 1U : 0U,
            static_cast<unsigned long long>(ArcDoubleBits(g.startMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.endMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits(i < 2U && m_pathArcProgrammed[i] ? m_pathArcCandidate[i] : g.startMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.startPulse[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.endPulse[i])));
        if (i < 2U)
            RtPrintf("[PCORE-BY-CIRCLE] run=%llu dispatch=%llu axis=%u offsetBits=%llu centerBits=%llu centerPulseBits=%llu minBits=%llu maxBits=%llu\n",
                static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
                static_cast<unsigned int>(i),
                static_cast<unsigned long long>(ArcDoubleBits(m_pathArcCenterOffset[i])),
                static_cast<unsigned long long>(ArcDoubleBits(g.centerMCS[i])),
                static_cast<unsigned long long>(ArcDoubleBits(g.centerPulse[i])),
                static_cast<unsigned long long>(ArcDoubleBits(g.boundsMinMCS[i])),
                static_cast<unsigned long long>(ArcDoubleBits(g.boundsMaxMCS[i])));
    }
}
#undef NC_PATH_ARC_NOINLINE
