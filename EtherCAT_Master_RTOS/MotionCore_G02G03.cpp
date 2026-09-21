#include "MotionCore.h"
#include "NCTranslationArcPrecision.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#if defined(_MSC_VER)
#define BY_ARC_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BY_ARC_NOINLINE __attribute__((noinline))
#else
#define BY_ARC_NOINLINE
#endif

namespace
{
    BY_ARC_NOINLINE bool FeedArcNormalOverride(const MotionCore& motion, bool buffered = false) noexcept
    {
        const MotionFeedHoldStopSnapshot snapshot = motion.GetFeedHoldStopSnapshot();
        return std::isfinite(snapshot.feedrateOverride) &&
            snapshot.feedrateOverride == 1.0 && !snapshot.overrideZero &&
            (buffered || (snapshot.groupDone && !snapshot.groupActive &&
                snapshot.commandQueueDepth == 0U && snapshot.commandIngressDepth == 0U)) &&
            snapshot.commandReplayDepth == 0U &&
            !snapshot.groupFaulted && !snapshot.groupEmergencyStopped &&
            !snapshot.safetyOrRecoveryPending && snapshot.faultedAxes == 0U &&
            (buffered || snapshot.commandStopped);
    }

    BY_ARC_NOINLINE bool PrepareFeedArcGeometry(
        const std::vector<AxisContext>* contexts,
        const std::array<double, 8U>& targetMCS,
        const double* commandedTail,
        MotionFeedArcWorkspace& workspace, const MotionCncPathTail* predecessor,
        std::uint32_t endpointAxisMask,
        const NCTranslationSnapshot& sourceTranslation) noexcept
    {
        MotionFeedArcReceipt& result = workspace.receipt;
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(workspace.input.plane, plane) || contexts == nullptr ||
            contexts->size() <= plane.u || contexts->size() <= plane.v)
        {
            result.code = MotionFeedArcCode::INVALID_INPUT;
            return false;
        }
        const unsigned planeSlots[2] = { plane.u, plane.v };
        // Sample every existing native command baseline, exactly as BX/G00.
        // logicalCmdPos is already machine-referenced after HOME; do not apply
        // machineCoordinateOffsetPulse again. NC preview already applied WCS.
        workspace.stagedPulse.fill(0.0);
        std::uint32_t validMask = 0U;
        for (std::size_t slot = 0U; slot < 8U; ++slot)
        {
            if (!std::isfinite(commandedTail[slot]))
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            workspace.stagedMCS[slot] = commandedTail[slot];
            if (slot >= contexts->size() || !(*contexts)[slot].isExist)
                continue;
            const AxisContext& axis = (*contexts)[slot];
            const double logicalPulse = predecessor != nullptr ? predecessor->endPulse[slot] : axis.logicalCmdPos.Load();
            if (!std::isfinite(logicalPulse) ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            if (predecessor != nullptr &&
                ((predecessor->validAxisMask & (1U << static_cast<unsigned>(slot))) == 0U ||
                    commandedTail[slot] != predecessor->endMCS[slot]))
            {
                result.code = MotionFeedArcCode::NOT_READY; return false;
            }
            double baseline = predecessor != nullptr ? predecessor->endMCS[slot] :
                logicalPulse * axis.finalLead / axis.resolution_PPR;
            // EF: buffered MCS stays canonical even if commanded tail has a numerically
            // equal but bit-distinct representation (for example signed zero).
            if (predecessor == nullptr && axis.axisType == AxisType::LINEAR &&
                std::isfinite(baseline))
            {
                // Keep the commanded MCS representation only after its forward
                // conversion exactly matches the sampled pulse. A round trip
                // must not introduce a displacement on a stationary axis.
                const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
                const double tailPulse = commandedTail[slot] * pulsePerMM;
                if (std::isfinite(pulsePerMM) && pulsePerMM > 0.0 &&
                    std::isfinite(tailPulse) && tailPulse == logicalPulse)
                {
                    baseline = commandedTail[slot];
                }
            }
            if (axis.axisType == AxisType::ROTARY)
            {
                if (!std::isfinite(axis.rotaryModulo) || axis.rotaryModulo <= 0.0)
                {
                    result.code = MotionFeedArcCode::INVALID_INPUT;
                    return false;
                }
                baseline = std::fmod(baseline, axis.rotaryModulo);
                if (baseline < 0.0) baseline += axis.rotaryModulo;
            }
            if (!std::isfinite(baseline))
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            workspace.stagedPulse[slot] = logicalPulse;
            workspace.stagedMCS[slot] = baseline;
            validMask |= (1U << static_cast<unsigned>(slot));
        }
        workspace.input.startMCS = workspace.stagedMCS;
        workspace.input.endMCS = workspace.stagedMCS;
        workspace.input.startPulse = workspace.stagedPulse;
        workspace.input.endPulse = workspace.stagedPulse;
        workspace.accTime = 0.0;
        workspace.decTime = 0.0;
        for (std::size_t component = 0U; component < 2U; ++component)
        {
            const unsigned slot = planeSlots[component];
            const AxisContext& axis = (*contexts)[slot];
            if ((validMask & (1U << static_cast<unsigned>(slot))) == 0U ||
                axis.axisType != AxisType::LINEAR ||
                !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
                !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
            if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0)
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            // EF: an omitted endpoint axis keeps the sampled start bit for bit,
            // including buffered canonical-tail pulses. Full circles keep both.
            // Do not round-trip an unprogrammed axis through MCS conversion.
            if (!workspace.input.fullCircle &&
                (endpointAxisMask & (1U << static_cast<unsigned>(slot))) != 0U)
            {
                // BASE-PLANE-20: preserve an exactly unchanged native component
                // selected by planar rotation or G17 polar endpoint expansion.
                // The other circular component is
                // still interpolated; no endpoint delta is tolerance-collapsed.
                const bool stationaryNative = (workspace.input.plane != 17 ||
                    sourceTranslation.polarMode == 16 ||
                    (sourceTranslation.cutterMode != 40 && sourceTranslation.distanceMode == 90)) &&
                    targetMCS[slot] == workspace.input.startMCS[slot];
                const double targetPulse = stationaryNative ? workspace.input.startPulse[slot] :
                    targetMCS[slot] * pulsePerMM;
                if (!std::isfinite(targetMCS[slot]) || !std::isfinite(targetPulse))
                {
                    result.code = MotionFeedArcCode::INVALID_INPUT;
                    return false;
                }
                workspace.input.endMCS[slot] = targetMCS[slot];
                workspace.input.endPulse[slot] = targetPulse;
                workspace.stagedMCS[slot] = targetMCS[slot];
                workspace.stagedPulse[slot] = targetPulse;
            }
            workspace.input.pulsePerMM[component] = pulsePerMM;
            workspace.input.maxVelocityPPS[component] = axis.maxVel_PPS;
            workspace.accTime = (std::max)(workspace.accTime, axis.G00_acc_time);
            workspace.decTime = (std::max)(workspace.decTime, axis.G00_dec_time);
        }
        if (workspace.accTime < 0.001) workspace.accTime = 0.2;
        if (workspace.decTime < 0.001) workspace.decTime = 0.2;
        // Only the already-authorized frozen source can supply cancellation
        // roundoff. Empty native callers retain the strict original budget.
        float sourceRoundoffMM = 0.0F;
        if (!IsNCTranslationSnapshotEmpty(sourceTranslation) &&
            !TryGetNCTranslationArcRoundoffMM(sourceTranslation, sourceRoundoffMM, workspace.input.fullCircle))
        {
            result.geometryCode = static_cast<std::uint32_t>(NCPathCoreFeedArcCode::PRECISION_BUDGET);
            result.code = MotionFeedArcCode::GEOMETRY_REJECTED;
            return false;
        }
        result.geometryCode = static_cast<std::uint32_t>(
            BuildNCPathCoreFeedArc(workspace.input, result.arc, sourceRoundoffMM));
        if (!result.arc.valid || result.arc.velocityPPS < 1.0 ||
            !std::isfinite(result.arc.velocityPPS / workspace.accTime) ||
            !std::isfinite(result.arc.velocityPPS / workspace.decTime))
        {
            result.code = MotionFeedArcCode::GEOMETRY_REJECTED;
            return false;
        }
        result.validAxisMask = validMask;
        return true;
    }

    BY_ARC_NOINLINE void ClearFeedArcCommand(MotionCommand& command) noexcept
    {
        // MotionCommand is trivially copyable. All zero representations below
        // are valid; initialize the nonzero legacy defaults explicitly. This
        // avoids a full-size MotionCommand{} automatic temporary at /Od.
        std::memset(static_cast<void*>(&command), 0, sizeof(command));
        command.execution.sourceBlockId = MOTION_SOURCE_BLOCK_ID_INVALID;
        command.sourceWCS = 54;
        command.sourceTranslation = NCTranslationSnapshot{};
        command.sourceToolLengthMode = 49;
        command.sourceToolRadiusMode = 40;
        command.sourceIsAbsoluteMode = true;
        command.sourceScaleRatio = 1.0;
        command.sourceG162Active = true;
        command.sourcePlaneMode = 17;
        for (std::size_t i = 0U; i < 3U; ++i)
            command.mem_transformMatrix[i][i] = 1.0;
    }
}

BY_ARC_NOINLINE void MotionCore::RejectPathCoreArcPublication(
    MotionCommand& command,
    MotionExecutionEpoch plannedEpoch,
    MotionExecutionEpoch publishedEpoch,
    MotionCommandSource source,
    const MotionOwnerLease& plannedOwner,
    MotionFeedArcReceipt& result) noexcept
{
    const bool ownerStillCurrent = IsMotionOwnerLeaseCurrent(plannedOwner);
    const MotionExecutionEpoch currentEpoch = GetCurrentExecutionEpoch();
    const MotionRejectReason reason = !ownerStillCurrent
        ? MotionRejectReason::OWNER_CONFLICT
        : (currentEpoch != plannedEpoch && currentEpoch != publishedEpoch)
        ? MotionRejectReason::STALE_EPOCH : MotionRejectReason::NOT_READY;
    // Existing formal producer rejection accounting, isolated on the failure
    // path. Its legacy by-value command copy is deliberately not hidden from
    // stack reports; the successful transaction borrows caller-owned storage.
    RejectNonGeometryProducerMotionCommand(command,
        publishedEpoch != MOTION_EXECUTION_EPOCH_INVALID ? publishedEpoch : plannedEpoch,
        source, plannedOwner, reason, &result.identity, &result.ownerLease);
    result.code = MotionFeedArcCode::PRODUCER_REJECTED;
}

bool MotionCore::TryG02G03MoveTransactionalTail(
    const std::array<double, 8U>& targetMCS,
    const std::array<double, 2U>& centerOffsetMM,
    int direction,
    bool fullCircle,
    double feedMMMin,
    const MotionArcTravelGuard& travelGuard,
    double(&commandedMCSTail)[MAX_AXES],
    MotionFeedArcWorkspace& workspace,
    MotionCommand& commandWorkspace)
{
    return TryG02G03MoveTransactionalCncTail(targetMCS, centerOffsetMM, direction, fullCircle,
        feedMMMin, travelGuard, commandedMCSTail, workspace, commandWorkspace, nullptr, false);
}

bool MotionCore::TryG02G03MoveTransactionalCncTail(
    const std::array<double, 8U>& targetMCS,
    const std::array<double, 2U>& centerOffsetMM,
    int direction,
    bool fullCircle,
    double feedMMMin,
    const MotionArcTravelGuard& travelGuard,
    double(&commandedMCSTail)[MAX_AXES],
    MotionFeedArcWorkspace& workspace,
    MotionCommand& commandWorkspace, const MotionCncPathTail* predecessor,
    bool cncFeedLookahead, std::uint32_t endpointAxisMask,
    bool requirePlanarBaselineMatch, bool preparedCutterArc)
{
    static_assert(MAX_AXES == 8, "BY fixed workspace must match Motion axes.");
    MotionFeedArcReceipt& result = workspace.receipt;
    result.Clear();
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(m_pendingPlaneMode, plane))
    {
        result.code = MotionFeedArcCode::INVALID_INPUT;
        return false;
    }
    workspace.input.plane = static_cast<std::uint8_t>(m_pendingPlaneMode);
    workspace.input.feedMMMin = feedMMMin;
    workspace.input.centerOffsetMM = centerOffsetMM;
    workspace.input.direction = direction;
    workspace.input.fullCircle = fullCircle;
    workspace.input.pulsePerMM.fill(0.0);
    workspace.input.maxVelocityPPS.fill(0.0);
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source =
        m_pendingCommandSource.load(std::memory_order_acquire);
    const bool cutterActive = m_pendingTranslation.cutterMode != 40;
    if (!IsPendingFixedTranslationSourceAllowed() ||
        (m_pendingPlaneMode != 17 && (IsNCTranslationSnapshotEmpty(m_pendingTranslation) ||
            !IsNCTranslationBaseArcPlaneFrame(m_pendingTranslation) ||
            cncFeedLookahead || predecessor != nullptr)) ||
        preparedCutterArc != cutterActive ||
        (cutterActive && (cncFeedLookahead || predecessor != nullptr ||
            !IsNCTranslationCutterArcNotationAllowed(m_pendingPlaneMode, m_pendingTranslation.distanceMode,
                m_pendingTranslation.polarMode, fullCircle) ||
            endpointAxisMask != plane.mask || !requirePlanarBaselineMatch ||
            !IsNCTranslationCutterDistanceModeAllowed(m_pendingPlaneMode, m_pendingTranslation.distanceMode) ||
            m_pendingIsAbsoluteMode != (m_pendingTranslation.distanceMode == 90) ||
            m_pendingG16Active != (m_pendingTranslation.polarMode == 16) || m_pendingG162Active)) ||
        (cncFeedLookahead && (m_pendingTranslation.distanceMode != 90 ||
            !m_pendingIsAbsoluteMode || m_pendingPlaneMode != 17 || m_pendingG162Active)) ||
        m_pContexts == nullptr || m_pContexts->size() <= plane.u || m_pContexts->size() <= plane.v ||
        m_pContexts->size() > 8U || travelGuard.check == nullptr ||
        !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        (direction != -1 && direction != 1) ||
        (fullCircle ? (endpointAxisMask != 0U && endpointAxisMask != plane.mask) :
            (endpointAxisMask == 0U || (endpointAxisMask & ~plane.mask) != 0U)) ||
        !std::isfinite(centerOffsetMM[0]) || !std::isfinite(centerOffsetMM[1]))
    {
        result.code = MotionFeedArcCode::INVALID_INPUT;
        return false;
    }
    // A prepared canonical circle may never hide a normal or auxiliary-axis
    // endpoint. Its packet order is u/v, not ascending physical XYZ slots.
    // G17 retains the established numeric test; new planes additionally
    // retain the bit-exact unselected-axis test immediately below.
    if (cutterActive)
    {
        for (std::size_t slot = 0U; slot < 8U; ++slot)
        {
            // Full circles cannot discard a supplied target and silently
            // reinterpret it as the sampled start. Check it before preparing
            // geometry, publishing an epoch, or changing any queue/tail.
            if (fullCircle && (std::memcmp(&targetMCS[slot], &commandedMCSTail[slot], sizeof(double)) != 0))
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            if (slot == plane.u || slot == plane.v) continue;
            if (!std::isfinite(targetMCS[slot]) || targetMCS[slot] != commandedMCSTail[slot])
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
        }
    }
    // BASE-PLANE-34: G17/G90 as well as G91 cutter arcs preserve every unselected
    // native axis bit, including signed zero, just like the G18/G19 lanes.
    // This is a packet/start invariant, not an endpoint epsilon.
    if (m_pendingPlaneMode != 17 || (m_pendingG16Active && !cncFeedLookahead) ||
        (cutterActive && (m_pendingTranslation.distanceMode == 90 || m_pendingTranslation.distanceMode == 91)))
    {
        for (unsigned slot = 0U; slot < 8U; ++slot)
        {
            if (slot == plane.u || slot == plane.v) continue;
            if (!std::isfinite(targetMCS[slot]) ||
                std::memcmp(&targetMCS[slot], &commandedMCSTail[slot], sizeof(double)) != 0)
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
        }
    }
    const bool buffered = predecessor != nullptr;
    if (buffered && (!cncFeedLookahead || !IsCncPathProducerTailCurrent(*predecessor, 3U,
        commandedMCSTail, plannedEpoch, plannedOwner, source)))
    {
        result.code = MotionFeedArcCode::NOT_READY;
        return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U ||
        HasPendingSafetyOrRecoveryRequests() || !FeedArcNormalOverride(*this, buffered))
    {
        result.code = MotionFeedArcCode::NOT_READY;
        return false;
    }
    if (!PrepareFeedArcGeometry(m_pContexts, targetMCS, commandedMCSTail,
        workspace, predecessor, endpointAxisMask, m_pendingTranslation))
        return false;
    // A first rotated/dependent endpoint is tied to the sampled native basis.
    // A buffered arc already proved its accepted predecessor tuple above;
    // PrepareFeedArcGeometry preserves that canonical MCS/pulse tail, while
    // live axes may still be moving on an earlier queued segment.
    if ((m_pendingPlaneMode != 17 || requirePlanarBaselineMatch ||
            (cncFeedLookahead && NCTranslationHasPlanarRotation(m_pendingTranslation)) ||
            m_pendingTranslation.distanceMode == 91) && !buffered &&
        !IsPlanarEndpointBasisCurrent(commandedMCSTail,
            workspace.input.startPulse, result.validAxisMask, (cutterActive || m_pendingPlaneMode != 17 || m_pendingG16Active) ? 7U : plane.mask))
    {
        result.code = MotionFeedArcCode::NOT_READY;
        return false;
    }
    if (cncFeedLookahead)
    {
        const double budget = 0.5 * (std::min)(result.arc.velocityPPS / workspace.accTime,
            result.arc.velocityPPS / workspace.decTime);
        const double speed = (std::min)(result.arc.velocityPPS,
            std::sqrt(budget) * std::sqrt(result.arc.radiusPulse));
        if (!std::isfinite(speed) || speed < 1.0)
        {
            result.code = MotionFeedArcCode::GEOMETRY_REJECTED; return false;
        }
    }
    // Endpoints alone are insufficient: every directed-sweep cardinal bound
    // must pass the same software travel policy before changing epoch/queue.
    for (int axis = 0; axis < 2; ++axis)
    {
        const std::size_t slot = static_cast<std::size_t>(axis);
        const int physicalAxis = static_cast<int>(axis == 0 ? plane.u : plane.v);
        if (!travelGuard.check(travelGuard.context, physicalAxis, result.arc.boundsMinMCS[slot]) ||
            !travelGuard.check(travelGuard.context, physicalAxis, result.arc.boundsMaxMCS[slot]))
        {
            result.travelLimitRejected = true;
            result.code = MotionFeedArcCode::GEOMETRY_REJECTED;
            return false;
        }
    }

    // BASE-PLANE-20: a stationary normal is part of the full XYZ path
    // for ALL polar circles as well as new-plane prepared cutter circles.
    // Use the existing HOME/Limit1/2/3 policy. This also closes the same
    // normal-axis gap for earlier G16 IJK arcs; their geometry is unchanged.
    // BASE-PLANE-30: the stationary normal is also part of the newly
    // admitted G17/G90 Cartesian cutter full circle; it is not a moved axis.
    // BASE-PLANE-34 includes G17/G90 partial cutter arcs as well.
    // Retain the caller's existing HOME/Limit policy; do not bypass it.
    if (((cutterActive && (m_pendingPlaneMode != 17 ||
            m_pendingTranslation.distanceMode == 90 || m_pendingTranslation.distanceMode == 91 || fullCircle)) || m_pendingG16Active) &&
        !travelGuard.check(travelGuard.context, static_cast<int>(plane.normal),
            result.arc.startMCS[plane.normal]))
    {
        result.travelLimitRejected = true;
        result.code = MotionFeedArcCode::GEOMETRY_REJECTED;
        return false;
    }

    MotionCommand& cmd = commandWorkspace;
    ClearFeedArcCommand(cmd);
    cmd.mode = direction == 1 ? InterpolationMode::CIRCULAR_CCW : InterpolationMode::CIRCULAR_CW;
    cmd.axisCount = 2;
    cmd.axisIndices[0] = static_cast<int>(plane.u);
    cmd.axisIndices[1] = static_cast<int>(plane.v);
    cmd.targetPos[0] = result.arc.endPulse[plane.u];
    cmd.targetPos[1] = result.arc.endPulse[plane.v];
    cmd.centerPos[0] = result.arc.centerPulse[0];
    cmd.centerPos[1] = result.arc.centerPulse[1];
    cmd.startRadius = result.arc.radiusPulse;
    cmd.endRadius = result.arc.radiusPulse;
    cmd.dir = direction;
    cmd.targetVel = result.arc.velocityPPS;
    cmd.accTime = workspace.accTime;
    cmd.decTime = workspace.decTime;
    cmd.commandPathMode = cncFeedLookahead ? MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP;
    cmd.cncFeedLookahead = cncFeedLookahead;
    if (cncFeedLookahead || cutterActive)
    {
        // Prepared cutter circles freeze PHYSICAL XYZ start pulses as a
        // read-only proof. Targets/centres are canonical packet u/v slots;
        // the consumer must map between them before resolving/loading.
        for (std::size_t i = 0U; i < (cutterActive ? 3U : 2U); ++i)
            cmd.mem_startPos[i] = result.arc.startPulse[i];
        cmd.mem_radius = result.arc.radiusPulse;
        cmd.mem_startAngle = result.arc.startAngle;
        cmd.mem_totalAngle = result.arc.sweepRadians;
        cmd.mem_totalDist = result.arc.lengthPulse;
    }
    cmd.pathCorePlanarCircle = true;
    cmd.pathCoreFullCircle = fullCircle;
    cmd.sourceLinePC = m_pendingSourcePC;
    cmd.sourceWCS = m_pendingSourceWCS;
    cmd.sourceTranslation = m_pendingTranslation;
    cmd.sourceToolLengthMode = m_pendingToolMode;
    cmd.sourceHCode = m_pendingHCode;
    cmd.sourceToolRadiusMode = m_pendingToolRadMode;
    cmd.sourceDCode = m_pendingDCode;
    cmd.sourceIsAbsoluteMode = m_pendingIsAbsoluteMode;
    cmd.sourceG68Active = m_pendingG68Active;
    cmd.sourceG68Angle = m_pendingG68Angle;
    cmd.sourceG168Active = m_pendingG168Active;
    cmd.sourceWCode = m_pendingWCode;
    cmd.sourceG51Active = m_pendingG51Active;
    cmd.sourceScaleRatio = m_pendingScaleRatio;
    cmd.sourceMirrorMask = m_pendingMirrorMask;
    cmd.sourceG16Active = m_pendingG16Active;
    cmd.sourceG162Active = m_pendingG162Active;
    cmd.sourcePlaneMode = m_pendingPlaneMode;

    MotionExecutionEpoch publishedEpoch = plannedEpoch;
    if (!buffered && !TryPublishOwnerAuthorizedAbortingExecutionEpoch(
        source, plannedEpoch, plannedOwner, publishedEpoch))
    {
        RejectPathCoreArcPublication(cmd, plannedEpoch, publishedEpoch,
            source, plannedOwner, result);
        return false;
    }
    AssignExecutionIdentity(cmd, publishedEpoch, source, plannedOwner);
    result.identity = cmd.execution;
    result.translationGeneration = cmd.sourceTranslation.generation;
    result.ownerLease = cmd.ownerLease;
    result.commandAccepted = TryEnqueueMotionCommand(cmd);
    if (!result.commandAccepted)
    {
        result.code = MotionFeedArcCode::PRODUCER_REJECTED;
        return false; // Existing enqueue owns formal rejection feedback/capture.
    }

    const auto tupleStable = [&]() -> bool
    {
        const std::uint64_t state1 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionExecutionEpoch epoch1 = GetCurrentExecutionEpoch();
        const MotionOwnerLease owner1 = UnpackMotionOwnerState(state1);
        const MotionExecutionEpoch epoch2 = GetCurrentExecutionEpoch();
        const std::uint64_t state2 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease owner2 = UnpackMotionOwnerState(state2);
        return result.identity.IsAssigned() && result.ownerLease.IsValid() &&
            result.identity.source == source &&
            result.identity.sourceBlockId == m_pendingSourcePC &&
            result.ownerLease.Matches(plannedOwner) &&
            epoch1 == result.identity.epoch && epoch2 == result.identity.epoch &&
            owner1.IsValid() && owner2.IsValid() &&
            owner1.Matches(result.ownerLease) && owner2.Matches(result.ownerLease) &&
            state1 == state2 && !UnpackMotionOwnerSafetyHandshake(state2) &&
            UnpackMotionOwnerSafetyRequestTicket(state2) ==
            m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto revokeAccepted = [&](MotionFeedArcCode code) -> bool
    {
        // Enqueue is irreversible here. Preserve that fact in this receipt,
        // revoke the producer tail tag, and use the existing safety takeover.
        result.code = code;
        result.valid = false;
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };
    if (!tupleStable())
    {
        return revokeAccepted(MotionFeedArcCode::STALE_AFTER_ACCEPT);
    }
    if (!m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow || m_programBlockMotionCapture.count != 1U)
    {
        return revokeAccepted(MotionFeedArcCode::CAPTURE_MISMATCH);
    }
    const MotionProgramBlockSubmission& submission =
        m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted &&
        submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == (cncFeedLookahead ? MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP) &&
        submission.identity.epoch == result.identity.epoch &&
        submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId &&
        submission.identity.source == result.identity.source;
    if (!result.captureBound)
    {
        return revokeAccepted(MotionFeedArcCode::CAPTURE_MISMATCH);
    }

    // Assignment-only commit after enqueue, same tagged pulse tail used by G00.
    m_g00ProducerQueueTailPulse = workspace.stagedPulse;
    for (std::size_t slot = 0U; slot < 8U; ++slot)
    {
        if ((result.validAxisMask & (1U << static_cast<unsigned>(slot))) != 0U)
        {
            (*m_pContexts)[slot].lastQueuedPulse.Store(workspace.stagedPulse[slot]);
        }
        commandedMCSTail[slot] = workspace.stagedMCS[slot];
    }
    result.tailCommitted = true;
    m_g00ProducerQueueTailValidMask = result.validAxisMask;
    if (!tupleStable())
    {
        return revokeAccepted(MotionFeedArcCode::STALE_AFTER_ACCEPT);
    }
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable())
    {
        return revokeAccepted(MotionFeedArcCode::STALE_AFTER_ACCEPT);
    }
    result.code = MotionFeedArcCode::COMMITTED;
    result.valid = true;
    return true;
}

#undef BY_ARC_NOINLINE
