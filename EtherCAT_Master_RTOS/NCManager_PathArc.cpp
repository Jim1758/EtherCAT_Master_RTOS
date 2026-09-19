// BASE-PLANE-1. Plane-bearing G17/G18/G19 circles share one audited native producer.
// New planes are exact-stop only; G17 queued/compensated contracts are unchanged.
#include "NCManager.h"
#include "AlarmManager.h"
#include "NCPathCoreRadiusArc.h"
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
bool NCManager::IsPathCoreArcBlockShapeValid(const NCBlock& block, bool allowMissingFeed, int unitsMode, bool polar, int plane, bool allowPlanarPolarRadius, bool allowSparsePolarRadius) noexcept
{
    NCArcPlaneAxes axes{};
    if (!TryGetNCArcPlaneAxes(plane, axes) || (plane != 17 && block.has('P'))) return false;
    if (unitsMode != 20 && unitsMode != 21) return false;
    const bool radiusFormat = block.has('R');
    const double feedMMMin = block.has('F') ? NCTranslationLengthToMM(block.val('F'), unitsMode) : 0.0;
    if (block.isEmpty || block.isGoto || !block.hasG ||
        (block.gCode != 2 && block.gCode != 3) || block.gCount != 1 ||
        block.gCodes[0] != block.gCode || block.mCount != 0 ||
        (!radiusFormat && !block.has(axes.uCenter) && !block.has(axes.vCenter)) ||
        (!block.has('F') && !allowMissingFeed) ||
        (block.has('F') && (!std::isfinite(feedMMMin) ||
            feedMMMin <= 0.0 || feedMMMin > 100.0))) return false;
    // BASE-PLANE-20: sparse R syntax is explicitly requested by the
    // G90 G17/G18/G19 G40 lane or the existing G18/G19 cutter lanes.
    // G40 resolves from the accepted native tail;
    // cutter resolves only from its proved NOMINAL contour. At least one
    // authored endpoint word remains mandatory in either sparse lane.
    // Mixed IJK, P and R full-circle inference are still rejected.
    // This is syntax only, not a frozen-source or Motion ownership permit.
    const bool planarPolarRadiusShape = allowPlanarPolarRadius && polar &&
        IsNCArcPlaneCode(plane) &&
        ((block.has(axes.uAddress) && block.has(axes.vAddress)) ||
            (allowSparsePolarRadius && (block.has(axes.uAddress) || block.has(axes.vAddress))));
    if (radiusFormat && ((polar && !planarPolarRadiusShape) || block.has('I') || block.has('J') || block.has('K') || block.has('P') ||
        (!block.has(axes.uAddress) && !block.has(axes.vAddress)) || block.val('R') == 0.0)) return false;
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const char address = static_cast<char>('A' + i);
        if (address != 'N' && address != 'F' && address != axes.uAddress && address != axes.vAddress &&
            address != axes.uCenter && address != axes.vCenter && address != 'P' && address != 'R') return false;
        if (!std::isfinite(block.val(address))) return false;
        if ((address == axes.uAddress || (!polar && address == axes.vAddress) ||
            address == axes.uCenter || address == axes.vCenter || address == 'R') &&
            !std::isfinite(NCTranslationLengthToMM(block.val(address), unitsMode))) return false;
    }
    if (polar && block.has(axes.uAddress) && block.val(axes.uAddress) < 0.0) return false;
    // P1 is this controller's explicit queued planar-transition request;
    // EF: either endpoint word may be omitted; only both omitted is a full circle.
    // P remains a queued transition request, not a turn count.
    if (block.has('P') && block.val('P') != 1.0) return false;
    return true;
}

NC_PATH_ARC_NOINLINE
bool NCManager::IsPathCoreArcInputOmission(const NCBlock& block) const noexcept
{
    if (!m_pathArc.armed || block.hasG || block.gCount != 0) return false;
    // Bare R has no modal owner, including at a fresh START or after a line.
    // The dispatcher calls this before any M/tool side effect. Explicit G10,
    // G68 and other settings continue to own their own R parameter.
    return block.has('R') || (m_pathArc.explicitArc &&
        (block.has('X') || block.has('Y') || block.has('Z') || block.has('F') || block.has('I') || block.has('J') || block.has('K')));
}

NC_PATH_ARC_NOINLINE
bool NCManager::IsPathCoreArcConfigurationValid() noexcept
{
    const NCTranslationSnapshot translation = CoordSys.GetTranslationSnapshot();
    if (!IsNCTranslationDistanceModeAllowed(CoordSys.isAbsoluteMode, translation) ||
        !IsNCTranslationUnitModeAllowed(CoordSys.isInchMode, translation) || !IsNCTranslationSourceAllowed(CoordSys.GetCurrentWCSGCode(), translation) ||
        CoordSys.activePlane != translation.rotationPlane || !IsNCArcPlaneCode(CoordSys.activePlane) ||
        !IsNCTranslationToolModeAllowed(CoordSys.toolLengthMode, translation) ||
        CoordSys.currentHCode != translation.toolHCode || CoordSys.toolRadiusMode != translation.cutterMode ||
        (translation.cutterMode != 40 && (CoordSys.currentDCode != translation.cutterD ||
            ArcDoubleBits(CoordSys.GetActiveToolRadius()) != ArcDoubleBits(translation.cutterRadiusMM))) ||
        !IsNCTranslationRotationModeAllowed(CoordSys.isG68Active, CoordSys.g68Angle,
            CoordSys.activePlane, translation) ||
        !IsNCTranslationWorkModeAllowed(CoordSys.isWorkpieceRotationActive, CoordSys.currentWCode, translation) ||
        !IsNCTranslationScaleMirrorModeAllowed(CoordSys.isScalingActive, CoordSys.isMirrorActive, translation) ||
        !IsNCTranslationPolarModeAllowed(CoordSys.isPolarCoordinateActive, translation) || CoordSys.isCAxisOffsetRotationEnabled ||
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
    m_cutterLine = CutterLineState{};
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
    m_cutterLine.staged = false;
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
    m_cutterLine = CutterLineState{};
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
    if (!IsPathCoreArcBlockShapeValid(block, true, CoordSys.isInchMode ? 20 : 21,
        CoordSys.isPolarCoordinateActive, CoordSys.activePlane, IsNCPolarRadiusArcNotationAllowed(CoordSys.activePlane, CoordSys.isAbsoluteMode,
                CoordSys.isPolarCoordinateActive, CoordSys.toolRadiusMode),
            IsNCPolarRadiusArcNotationAllowed(CoordSys.activePlane, CoordSys.isAbsoluteMode,
                CoordSys.isPolarCoordinateActive, CoordSys.toolRadiusMode)))
    {
        RejectPathCoreArcSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(CoordSys.activePlane, plane))
    {
        RejectPathCoreArcSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    const unsigned planeSlots[2] = { plane.u, plane.v };
    // A bad active policy is a configuration error, not an out-of-range point.
    // HOME-before-limits and G23/Limit1 semantics are owned by CoordinateManager.
    for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
    {
        const unsigned invalidMask = CoordSys.GetInvalidSoftwareTravelLimitMask(m_motion.GetAxisContext(axisIndex));
        if (invalidMask != 0U)
        {
            RtPrintf("[TRAVEL-CONFIG][REJECT] unit=ARC axis=%d invalidMask=%u beforeSubmit=1\n", axisIndex, invalidMask);
            RejectPathCoreArcSameThread(7U, AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG);
            return nullptr;
        }
    }
    const bool radiusFormat = block.has('R');
    const bool cutter = CoordSys.toolRadiusMode != 40;
    const bool polarRadius = radiusFormat && !cutter && CoordSys.isPolarCoordinateActive;
    if ((cutter && !IsCutterContourBlockShapeValid(block, CoordSys.isInchMode ? 20 : 21, CoordSys.activePlane, CoordSys.isPolarCoordinateActive)) ||
        m_cutterLine.leadOutRequired)
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
    const std::uint32_t sourceEndpointAxisMask = (block.has(plane.uAddress) ? (1U << plane.u) : 0U) |
        (block.has(plane.vAddress) ? (1U << plane.v) : 0U);
    bool fullCircle = sourceEndpointAxisMask == 0U;
    const NCTranslationSnapshot arcSource = CoordSys.GetTranslationSnapshot();
    const bool mirroredPlane = ((arcSource.mirrorMask & (1U << plane.u)) != 0U) !=
        ((arcSource.mirrorMask & (1U << plane.v)) != 0U);
    int direction = (block.gCode == 2 ? -1 : 1) * (mirroredPlane ? -1 : 1);
    m_pathArcProgrammed.fill(false);
    m_pathArcWCS.fill(0.0);
    m_pathArcCandidate.fill(0.0);
    // EB: omitted I or J is +0 for this source, never a modal carry.
    // Read only present parameter slots; keep original presence bits intact.
    m_pathArcCenterOffset[0U] = block.has(plane.uCenter) ?
        NCTranslationLengthToMM(block.val(plane.uCenter), CoordSys.isInchMode ? 20 : 21) : 0.0;
    m_pathArcCenterOffset[1U] = block.has(plane.vCenter) ?
        NCTranslationLengthToMM(block.val(plane.vCenter), CoordSys.isInchMode ? 20 : 21) : 0.0;
    // I/J are program-space vectors. Apply signed scale then rotate in mm, without
    // adding the G68 center or EXT/WCS/H/WORK translation. Native arc start
    // and full-circle endpoint bits remain owned by the Motion producer.
    double rotatedI = 0.0, rotatedJ = 0.0;
    const bool rotated = NCTranslationRotateArcVector(arcSource,
        m_pathArcCenterOffset[0U], m_pathArcCenterOffset[1U], rotatedI, rotatedJ);
    m_pathArcCenterOffset[0U] = rotatedI;
    m_pathArcCenterOffset[1U] = rotatedJ;
    if (!rotated)
    {
        RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    for (unsigned component = 0U; component < 2U; ++component)
    {
        const unsigned i = planeSlots[component];
        const AxisContext& axis = m_motion.GetAxisContext(static_cast<int>(i));
        if (!axis.isExist || axis.axisType != AxisType::LINEAR)
        {
            RejectPathCoreArcSameThread(4U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
        m_pathArcProgrammed[i] = block.has(m_axisNames[i]);
        if (m_pathArcProgrammed[i]) m_pathArcWCS[i] = IsNCPolarAngleAxis(CoordSys.isPolarCoordinateActive, CoordSys.activePlane, i) ?
            block.val(m_axisNames[i]) : NCTranslationLengthToMM(block.val(m_axisNames[i]), CoordSys.isInchMode ? 20 : 21);
    }
    const bool sparsePlanarBaselineMatch = cutter || (CoordSys.IsTranslationRunFrozen() &&
        (NCTranslationHasPlanarRotation(CoordSys.GetTranslationSnapshot()) || CoordSys.isPolarCoordinateActive) &&
        (m_pathArcProgrammed[plane.u] != m_pathArcProgrammed[plane.v]));
    // R centre resolution depends on the accepted start even for full XY G90.
    const bool requirePlanarBaselineMatch = radiusFormat || sparsePlanarBaselineMatch;
    // Polar cutter endpoints use their nominal-source decoder below. Do not
    // infer a nominal coordinate from the physical offset tail, nor decode
    // radius/angle a second time through the generic sparse endpoint path.
    if (!polarRadius && !(cutter && CoordSys.isPolarCoordinateActive) &&
        !CoordSys.CompleteFixedPlanarEndpoint(m_pathArcWCS.data(), m_pathArcProgrammed.data()))
    {
        RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // Full-circle classification uses original presence, never the effective
    // rotated endpoint mask. I/J above remain vectors in program space.
    if (polarRadius)
    {
        // BASE-PLANE-17. Resolve omitted polar words from the SAME accepted
        // native tail through the immutable affine inverse, not a modal cache.
        // The helper decodes and solves native R atomically. Never transform
        // its native centre again. Origin-direction ambiguity fails closed.
        NCPathPolarRadiusArcNative native{};
        if (!TryResolveNCPathPolarRadiusArcEndpoint(arcSource, CoordSys.commandedMCS,
            sourceEndpointAxisMask, block.val(plane.uAddress), block.val(plane.vAddress), block.val('R'),
            block.gCode == 2 ? -1 : 1, native))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
        std::memcpy(m_pathArcCandidate.data(), native.endMCS, sizeof(native.endMCS));
        m_pathArcCenterOffset[0U] = native.centerOffsetMM[0U];
        m_pathArcCenterOffset[1U] = native.centerOffsetMM[1U];
        direction = native.direction;
        fullCircle = false;
        // Authored polar presence is NOT native axis selection. Changing
        // only radius or angle may move both plane axes. Preserve the raw
        // mask above but bind/prove both effective native endpoints.
        m_pathArcProgrammed[plane.u] = m_pathArcProgrammed[plane.v] = true;
    }
    else if (cutter && CoordSys.isPolarCoordinateActive)
    {
        if (!PreviewCutterPolarEndpointSameThread(block, m_pathArcCandidate))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
        // Authored polar presence is not native motion selection. Both axes
        // must be proved even when only radius OR angle was written. Preview
        // and immutable lookahead use the same accepted NOMINAL source, while
        // untouched axes retain the accepted physical-tail bits.
        m_pathArcProgrammed[plane.u] = m_pathArcProgrammed[plane.v] = true;
    }
    else if (cutter && !CoordSys.isAbsoluteMode)
    {
        if (!PreviewCutterIncrementalEndpointSameThread(block, m_pathArcCandidate))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
    }
    else CoordSys.Preview_WCS_to_MCS(m_pathArcWCS.data(), m_pathArcProgrammed.data(), m_pathArcCandidate.data());
    if (cutter && !BuildCutterContourSameThread(block, m_pathArc.sourcePC, m_pathArc.run,
        m_pathArc.cache, m_pathArc.dispatch, m_pathArcCandidate, m_pathArcCenterOffset, direction))
    {
        RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
        return nullptr;
    }
    // BASE-PLANE-10: cutter full-circle classification comes only from
    // the proved authored seam, never from a zero native endpoint delta.
    if (cutter) fullCircle = m_cutterLine.stagedPrimitive.fullCircle;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(m_pathArcCandidate[i]))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::G_Code_Invalid_parameter);
            return nullptr;
        }
    }
    // Cutter construction already resolved the nominal R circle and produced
    // its physical centre/endpoint. Do not solve R again from that offset path.
    if (radiusFormat && !cutter && !polarRadius)
    {
        // The endpoint is already native. R receives units and positive uniform
        // scaling once; the direction above already accounts for XY reflection.
        // The resulting centre offset must not pass through the affine map again.
        const double signedRadiusMM = NCTranslationLengthToMM(block.val('R'), arcSource.unitsMode) *
            (arcSource.scalingMode == 51 ? arcSource.scalingFactor : 1.0);
        if (!TryResolveNCPathRadiusArcCenter(CoordSys.commandedMCS[plane.u], CoordSys.commandedMCS[plane.v],
            m_pathArcCandidate[plane.u], m_pathArcCandidate[plane.v], signedRadiusMM, direction,
            m_pathArcCenterOffset[0], m_pathArcCenterOffset[1]))
        {
            RejectPathCoreArcSameThread(5U, AlarmManager::PATH_GEOMETRY_INVALID);
            return nullptr;
        }
    }
    const std::uint32_t endpointAxisMask = (m_pathArcProgrammed[plane.u] ? (1U << plane.u) : 0U) |
        (m_pathArcProgrammed[plane.v] ? (1U << plane.v) : 0U);
    if (sparsePlanarBaselineMatch)
        RtPrintf("[ROTATION][SPARSE_PLANE] g=%d plane=%d rawMask=%u effectiveMask=%u beforeSubmit=1\n",
            block.gCode, CoordSys.activePlane, sourceEndpointAxisMask, endpointAxisMask);
    if (!CoordSys.isAbsoluteMode)
        RtPrintf("[INCREMENTAL][TARGET] g=%d rawMask=%u effectiveMask=%u fullCircle=%u beforeSubmit=1\n",
            block.gCode, static_cast<unsigned>(sourceEndpointAxisMask),
            static_cast<unsigned>(endpointAxisMask), fullCircle ? 1U : 0U);
    MotionArcTravelGuard travelGuard{};
    travelGuard.context = this;
    travelGuard.check = [](const void* context, int axisIndex, double target) -> bool
    {
        const NCManager* nc = static_cast<const NCManager*>(context);
        return axisIndex >= 0 && axisIndex < 3 &&
            nc->CoordSys.IsTargetWithinSoftwareTravelLimit(nc->m_motion.GetAxisContext(axisIndex), target);
    };
    const bool accepted = m_motion.TryG02G03MoveTransactionalCncTail(m_pathArcCandidate,
        m_pathArcCenterOffset, direction, fullCircle, effectiveFeed, travelGuard,
        CoordSys.commandedMCS, m_pathArcMotion, m_pathArcCommand,
        buffered ? &m_cncFeed.tail : nullptr, queued, endpointAxisMask, requirePlanarBaselineMatch, cutter);
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
        if ((i == plane.u || i == plane.v) && m_pathArcProgrammed[i])
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
        const double expectedCenter = receipt.arc.startMCS[planeSlots[i]] + m_pathArcCenterOffset[i];
        if (!std::isfinite(expectedCenter) ||
            ArcDoubleBits(receipt.arc.centerMCS[i]) != ArcDoubleBits(expectedCenter))
        {
            RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            return nullptr;
        }
    }
    if (!receipt.valid || !receipt.commandAccepted || !receipt.tailCommitted || !receipt.captureBound ||
        !ArcTranslationCurrent(CoordSys, receipt.translationGeneration) ||
        receipt.travelLimitRejected || !receipt.arc.valid || receipt.arc.axisMask != plane.mask ||
        receipt.arc.plane != CoordSys.activePlane ||
        receipt.arc.direction != direction || receipt.arc.fullCircle != fullCircle ||
        ArcDoubleBits(receipt.arc.feedMMMin) != ArcDoubleBits(effectiveFeed) ||
        (receipt.validAxisMask & plane.mask) != plane.mask || !receipt.identity.IsAssigned() ||
        receipt.identity.epoch != m_motion.GetCurrentExecutionEpoch() ||
        receipt.identity.source != MotionCommandSource::NC_MEMORY ||
        !receipt.ownerLease.Matches(m_programMotionLease))
    {
        RejectPathCoreArcSameThread(8U, AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
        return nullptr;
    }
    if (cutter) LogCutterContourSameThread(m_pathArc.run, m_pathArc.dispatch, m_pathArc.sourcePC);
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
    if (CoordSys.toolRadiusMode != 40 &&
        !CommitCutterContourSameThread(m_pathArc.run, m_pathArc.cache, dispatchId,
            commit.sequence, r.translationGeneration, sourcePC, r.arc.endMCS))
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
    if (CoordSys.toolRadiusMode == 40 && CoordSys.activePlane == 17)
        RetainPathCoreArcSameThread(); // Cutter and G18/G19 replay remain a later stage.
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
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(g.plane, plane)) return;
    RtPrintf("[PCORE-BY-GEO] run=%llu dispatch=%llu plane=%u mask=%u validMask=%u direction=%d fullCircle=%u FBits=%llu radiusMMBits=%llu radiusPulseBits=%llu startAngleBits=%llu sweepBits=%llu lengthMMBits=%llu lengthPulseBits=%llu velocityPPSBits=%llu\n",
        static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
        static_cast<unsigned int>(g.plane), static_cast<unsigned int>(g.axisMask), static_cast<unsigned int>(m_pathArcMotion.receipt.validAxisMask),
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
            static_cast<unsigned int>(i), (i == plane.u || i == plane.v) ? 1U : 0U, m_pathArcProgrammed[i] ? 1U : 0U,
            static_cast<unsigned long long>(ArcDoubleBits(g.startMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.endMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits((i == plane.u || i == plane.v) && m_pathArcProgrammed[i] ? m_pathArcCandidate[i] : g.startMCS[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.startPulse[i])),
            static_cast<unsigned long long>(ArcDoubleBits(g.endPulse[i])));
        if (i == plane.u || i == plane.v)
        {
            const unsigned component = i == plane.u ? 0U : 1U;
            RtPrintf("[PCORE-BY-CIRCLE] run=%llu dispatch=%llu axis=%u offsetBits=%llu centerBits=%llu centerPulseBits=%llu minBits=%llu maxBits=%llu\n",
                static_cast<unsigned long long>(m_pathArc.run), static_cast<unsigned long long>(m_pathArc.dispatch),
                static_cast<unsigned int>(i),
                static_cast<unsigned long long>(ArcDoubleBits(m_pathArcCenterOffset[component])),
                static_cast<unsigned long long>(ArcDoubleBits(g.centerMCS[component])),
                static_cast<unsigned long long>(ArcDoubleBits(g.centerPulse[component])),
                static_cast<unsigned long long>(ArcDoubleBits(g.boundsMinMCS[component])),
                static_cast<unsigned long long>(ArcDoubleBits(g.boundsMaxMCS[component])));
        }
    }
}
#undef NC_PATH_ARC_NOINLINE
