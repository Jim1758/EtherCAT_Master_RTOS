#include "MotionCore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace
{
#if defined(_MSC_VER)
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
    __attribute__((noinline))
#endif
        bool FeedLineNormalOverride(const MotionCore& motion, bool buffered = false) noexcept
    {
        // Existing coherent RT publication; never read raw m_Group state here.
        // M00/Feed Hold resumes to 1.0 before NC admits the next G01.
        const MotionFeedHoldStopSnapshot snapshot =
            motion.GetFeedHoldStopSnapshot();
        // A failed publication read returns groupDone=false. Override=1 alone
        // is a default value and cannot authorize a new transaction.
        if (buffered)
            return (snapshot.groupActive || snapshot.groupDone || snapshot.commandQueueDepth != 0U ||
                snapshot.commandIngressDepth != 0U || snapshot.commandReplayDepth != 0U) &&
            std::isfinite(snapshot.feedrateOverride) && snapshot.feedrateOverride == 1.0 &&
            !snapshot.overrideZero && !snapshot.groupFaulted && !snapshot.groupEmergencyStopped &&
            !snapshot.safetyOrRecoveryPending && snapshot.faultedAxes == 0U;
        return std::isfinite(snapshot.feedrateOverride) &&
            snapshot.feedrateOverride == 1.0 && !snapshot.overrideZero &&
            snapshot.groupDone && !snapshot.groupActive &&
            snapshot.commandQueueDepth == 0U &&
            snapshot.commandIngressDepth == 0U &&
            snapshot.commandReplayDepth == 0U &&
            !snapshot.groupFaulted && !snapshot.groupEmergencyStopped &&
            !snapshot.safetyOrRecoveryPending && snapshot.faultedAxes == 0U &&
            snapshot.commandStopped;
    }
    bool PrepareFeedLineGeometry(
        const std::vector<AxisContext>* contexts,
        const std::vector<int>& axes,
        const std::vector<double>& targetMCS,
        const double* commandedTail,
        MotionFeedLineWorkspace& workspace,
        const MotionCncPathTail* predecessor = nullptr,
        std::uint32_t endpointAxisMask = 0U,
        bool preserveStationaryNativePulse = false) noexcept
    {
        MotionFeedLineReceipt& result = workspace.receipt;
        // Rebuild the ABORTING successor baseline for every existing axis.
        // Native MCS is the existing G00 commanded geometry: pulse * lead / PPR.
        // ApplyHomeReference shifts logicalCmdPos by oldOffset-newOffset before
        // storing machineCoordinateOffsetPulse; ProcessAxis similarly removes
        // that offset before publishing positions. These logical command pulses
        // are already machine-referenced. Adding/subtracting it here again would
        // apply the HOME offset twice. CoordinateManager already converted WCS.
        workspace.stagedPulse.fill(0.0);
        std::uint32_t validMask = 0U;
        for (std::size_t slot = 0U; slot < 8U; ++slot)
        {
            if (!std::isfinite(commandedTail[slot]))
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            workspace.stagedMCS[slot] = commandedTail[slot];
            if (slot >= contexts->size() || !(*contexts)[slot].isExist)
            {
                continue;
            }
            const AxisContext& axis = (*contexts)[slot];
            const double logicalPulse = predecessor != nullptr ?
                predecessor->endPulse[slot] : axis.logicalCmdPos.Load();
            if (!std::isfinite(logicalPulse) ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            if (predecessor != nullptr &&
                ((predecessor->validAxisMask & (1U << static_cast<unsigned>(slot))) == 0U ||
                    commandedTail[slot] != predecessor->endMCS[slot]))
            {
                result.code = MotionFeedLineCode::NOT_READY;
                return false;
            }
            double baseline = predecessor != nullptr ? predecessor->endMCS[slot] :
                logicalPulse * axis.finalLead / axis.resolution_PPR;
            // EG: a buffered predecessor owns its exact MCS representation,
            // including signed zero; do not replace it with an equal NC spelling.
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
                    result.code = MotionFeedLineCode::INVALID_INPUT;
                    return false;
                }
                baseline = std::fmod(baseline, axis.rotaryModulo);
                if (baseline < 0.0)
                {
                    baseline += axis.rotaryModulo;
                }
            }
            if (!std::isfinite(baseline))
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
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
        for (std::size_t index = 0U; index < axes.size(); ++index)
        {
            const int axisIndex = axes[index];
            if (axisIndex < 0 || axisIndex > 2 ||
                static_cast<std::size_t>(axisIndex) >= contexts->size())
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            const std::size_t slot = static_cast<std::size_t>(axisIndex);
            const std::uint32_t bit = (1U << static_cast<unsigned>(axisIndex));
            const AxisContext& axis = (*contexts)[slot];
            const bool programmed = endpointAxisMask == 0U || (endpointAxisMask & bit) != 0U;
            if ((validMask & bit) == 0U ||
                (workspace.input.axisMask & bit) != 0U ||
                axis.axisType != AxisType::LINEAR || (programmed && !std::isfinite(targetMCS[index])) ||
                !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
                !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
            // The omitted endpoint keeps native sampled/committed bits. Its
            // absent payload is never read or converted through MCS arithmetic.
            const double target = programmed ? targetMCS[index] : workspace.input.startMCS[slot];
            // BASE-PLANE-3: rotation can select a coupled axis whose native
            // endpoint is exactly unchanged. Preserve the sampled pulse in
            // that case, including a baseline obtained by pulse*lead/PPR.
            // This is exact equality, not an epsilon or a drift correction;
            // the independent native-start proof still runs before dispatch.
            const bool stationaryNative = preserveStationaryNativePulse &&
                target == workspace.input.startMCS[slot];
            const double targetPulse = programmed && !stationaryNative ?
                targetMCS[index] * pulsePerMM : workspace.input.startPulse[slot];
            if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0 ||
                !std::isfinite(targetPulse))
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            workspace.input.axisMask |= bit;
            workspace.input.endMCS[slot] = target;
            workspace.input.endPulse[slot] = targetPulse;
            workspace.input.maxVelocityPPS[slot] = axis.maxVel_PPS;
            workspace.stagedMCS[slot] = target;
            workspace.stagedPulse[slot] = targetPulse;
            workspace.targetPulse.push_back(targetPulse); // capacity checked above
            workspace.accTime = (std::max)(workspace.accTime, axis.G00_acc_time);
            workspace.decTime = (std::max)(workspace.decTime, axis.G00_dec_time);
        }
        if (workspace.accTime < 0.001) workspace.accTime = 0.2;
        if (workspace.decTime < 0.001) workspace.decTime = 0.2;

        result.geometryCode = static_cast<std::uint32_t>(
            BuildNCPathCoreFeedLine(workspace.input, result.line));
        if (!result.line.valid || (!result.line.point && result.line.velocityPPS < 1.0) ||
            !std::isfinite(result.line.velocityPPS / workspace.accTime) ||
            !std::isfinite(result.line.velocityPPS / workspace.decTime))
        {
            // Existing LINEAR consumer requires at least 1 pulse/s for movement.
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        // The existing LINEAR consumer normalizes each delta with direct division.
        // Reject representable geometry that that exact consumer operation would
        // collapse to zero or make nonfinite; do not hide it with a speed clamp.
        if (!result.line.point)
        {
            for (std::size_t slot = 0U; slot < 3U; ++slot)
            {
                if ((workspace.input.axisMask & (1U << static_cast<unsigned>(slot))) == 0U)
                    continue;
                const double delta = result.line.endPulse[slot] - result.line.startPulse[slot];
                const double ratio = delta / result.line.lengthPulse;
                const double axisVelocity = std::abs(result.line.velocityPPS * ratio);
                if (!std::isfinite(ratio) || (delta != 0.0 && ratio == 0.0) ||
                    !std::isfinite(axisVelocity) ||
                    axisVelocity > workspace.input.maxVelocityPPS[slot])
                {
                    result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
                    return false;
                }
            }
        }
        result.validAxisMask = validMask;
        return true;
    }

}

bool MotionCore::TryG01MoveTransactionalTail(
    const std::vector<int>& axes,
    const std::vector<double>& targetMCS,
    double feedMMMin,
    double(&commandedMCSTail)[MAX_AXES],
    MotionFeedLineWorkspace& workspace)
{
    return TryG01MoveTransactionalTail(axes, targetMCS, feedMMMin,
        commandedMCSTail, workspace, nullptr);
}

bool MotionCore::TryG01MoveTransactionalTail(
    const std::vector<int>& axes, const std::vector<double>& targetMCS,
    double feedMMMin, double(&commandedMCSTail)[MAX_AXES],
    MotionFeedLineWorkspace& workspace, const MotionFeedLineReceipt* predecessor)
{
    return TryG01MoveTransactionalTail(axes, targetMCS, feedMMMin,
        commandedMCSTail, workspace, predecessor, false);
}

bool MotionCore::TryG01MoveTransactionalTail(
    const std::vector<int>& axes, const std::vector<double>& targetMCS,
    double feedMMMin, double(&commandedMCSTail)[MAX_AXES],
    MotionFeedLineWorkspace& workspace, const MotionFeedLineReceipt* predecessor,
    bool cncFeedLookahead)
{
    if (predecessor == &workspace.receipt)
    {
        workspace.receipt.Clear();
        workspace.receipt.code = MotionFeedLineCode::NOT_READY;
        return false;
    }
    if (predecessor != nullptr) workspace.predecessorTail.Assign(*predecessor);
    return TryG01MoveTransactionalCncTail(axes, targetMCS, feedMMMin,
        commandedMCSTail, workspace, predecessor != nullptr ? &workspace.predecessorTail : nullptr,
        cncFeedLookahead);
}

bool MotionCore::TryG01MoveTransactionalCncTail(
    const std::vector<int>& axes, const std::vector<double>& targetMCS,
    double feedMMMin, double(&commandedMCSTail)[MAX_AXES],
    MotionFeedLineWorkspace& workspace, const MotionCncPathTail* predecessor,
    bool cncFeedLookahead,
    const std::array<double, 8U>* cornerNextMCS, double cornerToleranceMM,
    const MotionArcTravelGuard* cornerTravelGuard, double cornerNextFeedMMMin,
    std::uint32_t endpointAxisMask, bool requirePlanarBaselineMatch,
    bool requireNativeXYZBaselineMatch)
{
    static_assert(MAX_AXES == 8, "BX fixed workspace must match Motion axes.");
    MotionFeedLineReceipt& result = workspace.receipt;
    result.Clear();
    workspace.targetPulse.clear();
    workspace.input.axisMask = 0U;
    workspace.input.feedMMMin = feedMMMin;
    workspace.input.maxVelocityPPS.fill(0.0);

    // BX supports exactly one MEMORY/AUTO transaction per NC source block.
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source =
        m_pendingCommandSource.load(std::memory_order_acquire);
    // G16 sparse radius/angle depends on the same accepted XY predecessor as
    // rotated Cartesian endpoints, even when G68 and WORK yaw are both zero.
    const bool basePlaneLinear = m_pendingPlaneMode != 17;
    const bool cutterActive = m_pendingTranslation.cutterMode != 40;
    // BASE-PLANE-39: the NC-requested G40 lead-out proof covers every
    // already-admitted cutter notation, including G18/G19 G91 and G90/G16.
    // The stationary normal must match the accepted XYZ basis before enqueue
    // or tail commit. Native line slots stay ascending; ordinary and queued
    // G01 retain their previous route and all source/geometry gates below.
    const bool nominalLineNotation =
        IsNCTranslationCutterNotationAllowed(m_pendingPlaneMode,
            m_pendingTranslation.distanceMode, m_pendingTranslation.polarMode) &&
        m_pendingIsAbsoluteMode == (m_pendingTranslation.distanceMode == 90) &&
        m_pendingG16Active == (m_pendingTranslation.polarMode == 16);
    const bool nativeXYZBaseline = requireNativeXYZBaselineMatch ||
        (nominalLineNotation && cutterActive);
    NCArcPlaneAxes cutterPlane{};
    const bool cutterPlaneValid = TryGetNCArcPlaneAxes(m_pendingPlaneMode, cutterPlane);
    const bool coupledPlanarEndpoint = NCTranslationHasPlanarRotation(m_pendingTranslation) ||
        m_pendingTranslation.polarMode == 16;
    const bool coupledPlanarQueuedLine = coupledPlanarEndpoint && cncFeedLookahead;
    if (!IsPendingFixedTranslationSourceAllowed() ||
        (nativeXYZBaseline && (!nominalLineNotation ||
            !IsNCTranslationSnapshotValid(m_pendingTranslation) ||
            !IsNCPlaneLinearPairMapping(m_pendingPlaneMode, static_cast<int>(axes.size()), axes.data()) ||
            cncFeedLookahead || predecessor != nullptr || cornerNextMCS != nullptr ||
            cornerToleranceMM != 0.0 || cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0)) ||
        (basePlaneLinear && (!IsNCTranslationSnapshotValid(m_pendingTranslation) ||
            !IsNCTranslationBaseArcPlaneFrame(m_pendingTranslation) ||
            m_pendingPlaneMode != m_pendingTranslation.rotationPlane ||
            !IsNCNativeXYZLinearMapping(static_cast<int>(axes.size()), axes.data()) ||
            cncFeedLookahead || predecessor != nullptr || cornerNextMCS != nullptr ||
            cornerToleranceMM != 0.0 || cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0 ||
            (!cutterActive && (endpointAxisMask != 0U ||
                (requirePlanarBaselineMatch && !nativeXYZBaseline))))) ||
        (cutterActive && (cncFeedLookahead || predecessor != nullptr ||
            cornerNextMCS != nullptr || cornerToleranceMM != 0.0 ||
            cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0 ||
            !cutterPlaneValid || !IsNCPlaneLinearPairMapping(m_pendingPlaneMode,
                static_cast<int>(axes.size()), axes.data()) ||
            endpointAxisMask != cutterPlane.mask || !requirePlanarBaselineMatch)) ||
        (m_pendingTranslation.distanceMode == 91 &&
            (cncFeedLookahead || cornerNextMCS != nullptr)) ||
        (coupledPlanarEndpoint && cornerNextMCS != nullptr && !cncFeedLookahead) ||
        (coupledPlanarQueuedLine && (m_pendingTranslation.distanceMode != 90 ||
            axes.size() != 2U || axes[0] != 0 || axes[1] != 1 || endpointAxisMask != 0U)) ||
        m_pContexts == nullptr || m_pContexts->size() > 8U ||
        axes.empty() || axes.size() > 3U ||
        axes.size() != targetMCS.size() ||
        (endpointAxisMask != 0U && !cutterActive &&
            ((endpointAxisMask != 1U && endpointAxisMask != 2U) || !cncFeedLookahead ||
                axes.size() != 2U || axes[0] != 0 || axes[1] != 1 ||
                cornerNextMCS != nullptr || cornerToleranceMM != 0.0 ||
                cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0)) ||
        !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        workspace.targetPulse.capacity() < 8U)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    const bool buffered = predecessor != nullptr;
    if (buffered)
    {
        std::uint32_t mask = 0U;
        for (int axis : axes)
        {
            if (axis < 0 || axis > 2 || (mask & (1U << static_cast<unsigned>(axis))) != 0U)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            mask |= (1U << static_cast<unsigned>(axis));
        }
        if (!IsCncPathProducerTailCurrent(*predecessor, mask, commandedMCSTail,
            plannedEpoch, plannedOwner, source))
        {
            result.code = MotionFeedLineCode::NOT_READY;
            return false;
        }
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U ||
        HasPendingSafetyOrRecoveryRequests() ||
        !FeedLineNormalOverride(*this, buffered))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }

    if (!PrepareFeedLineGeometry(m_pContexts, axes, targetMCS,
        commandedMCSTail, workspace, predecessor, endpointAxisMask, basePlaneLinear || nativeXYZBaseline))
    {
        return false;
    }

    const bool incrementalEndpoint = m_pendingTranslation.distanceMode == 91;
    const std::uint32_t requiredBaselineMask =
        ((requirePlanarBaselineMatch || coupledPlanarQueuedLine) ?
            (basePlaneLinear && cutterActive ? 7U : 3U) : 0U) |
        ((incrementalEndpoint || basePlaneLinear) ? workspace.input.axisMask : 0U) |
        (nativeXYZBaseline ? 7U : 0U);
    // A first coupled XY plain/Q line proves the sampled native basis used by geometry.
    // A buffered line already proved its immutable accepted predecessor above;
    // live axes can still be inside an earlier segment and must not replace it.
    // New-plane straight envelopes were checked at both native endpoints
    // by NC. Prove that same sampled start even for absolute sparse G00/G01.
    if (((requirePlanarBaselineMatch || incrementalEndpoint || basePlaneLinear || nativeXYZBaseline) &&
            !coupledPlanarQueuedLine && (buffered || cncFeedLookahead)) ||
        ((requirePlanarBaselineMatch || coupledPlanarQueuedLine || incrementalEndpoint || basePlaneLinear || nativeXYZBaseline) && !buffered &&
            !IsPlanarEndpointBasisCurrent(commandedMCSTail,
                workspace.input.startPulse, result.validAxisMask, requiredBaselineMask)))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }

    if (cornerNextMCS != nullptr)
    {
        // Zero is the compatibility spelling for an explicitly same-F producer.
        const double nextFeed = cornerNextFeedMMMin == 0.0 ? feedMMMin : cornerNextFeedMMMin;
        if (!std::isfinite(nextFeed) || nextFeed <= 0.0 || nextFeed > 100.0)
        {
            result.code = MotionFeedLineCode::INVALID_INPUT; return false;
        }
        if (!cncFeedLookahead || axes.size() != 2U || axes[0] != 0 || axes[1] != 1 ||
            cornerTravelGuard == nullptr || cornerTravelGuard->check == nullptr)
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED; return false;
        }
        const std::array<double, 2U> ppm = { {(*m_pContexts)[0].resolution_PPR / (*m_pContexts)[0].finalLead,
            (*m_pContexts)[1].resolution_PPR / (*m_pContexts)[1].finalLead} };
        // DH_FIX1: preview supplies only XY. Unselected axes must still match
        // the NC tail, then inherit the exact sampled/committed native baseline.
        // Never snap native pulses to a nominal NC target or relax geometry Q.
        std::array<double, 8U> anchoredCornerNext = workspace.input.startMCS;
        for (std::size_t i = 0U; i < 8U; ++i)
        {
            if (!std::isfinite((*cornerNextMCS)[i]) ||
                (i >= 2U && (*cornerNextMCS)[i] != commandedMCSTail[i]))
            {
                result.code = MotionFeedLineCode::GEOMETRY_REJECTED; return false;
            }
        }
        anchoredCornerNext[0] = (*cornerNextMCS)[0];
        anchoredCornerNext[1] = (*cornerNextMCS)[1];
        if (!BuildNCPathCoreCornerBlend(workspace.input, anchoredCornerNext, cornerToleranceMM, ppm,
            result.line, workspace.cornerArcInput, workspace.cornerArc, result.blendGeometry, result.blendMetadata))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED; return false;
        }
        for (unsigned i = 0U; i < 2U; ++i)
        {
            if (!cornerTravelGuard->check(cornerTravelGuard->context, int(i), result.blendGeometry.boundsMinMCS[i]) ||
                !cornerTravelGuard->check(cornerTravelGuard->context, int(i), result.blendGeometry.boundsMaxMCS[i]))
            {
                result.travelLimitRejected = true;
                result.code = MotionFeedLineCode::GEOMETRY_REJECTED; return false;
            }
            workspace.targetPulse[i] = result.blendGeometry.endPulse[i];
        }
        // DI packet min still governs the fillet, source seams and future horizon.
        // DK separately transports Fi for the loaded straight prefix; retain the
        // canonical arc's exact scalar conversion, including same-F equality.
        result.cornerNextFeedMMMin = nextFeed;
        result.cornerDispatchFeedMMMin = (std::min)(feedMMMin, nextFeed);
        double capVelocity = workspace.cornerArc.velocityPPS;
        if (nextFeed < feedMMMin)
        {
            int ef = 0, es = 0;
            const double mf = std::frexp(nextFeed, &ef);
            const double ms = std::frexp(ppm[0], &es);
            capVelocity = (std::min)(capVelocity, std::ldexp((mf * ms) / 60.0, ef + es));
        }
        if (!std::isfinite(capVelocity) || capVelocity < 1.0 ||
            !std::isfinite(capVelocity / workspace.accTime) ||
            !std::isfinite(capVelocity / workspace.decTime))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED; return false;
        }
        result.cornerDispatchVelocityPPS = capVelocity;
        workspace.stagedMCS = result.blendGeometry.endMCS;
        workspace.stagedPulse = result.blendGeometry.endPulse;
    }
    else if (cornerToleranceMM != 0.0 || cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT; return false;
    }

    const bool accepted = TryLineMove(
        axes, workspace.targetPulse, result.blendGeometry.valid ? result.cornerDispatchVelocityPPS : result.line.velocityPPS,
        workspace.accTime, workspace.decTime, buffered ? BufferMode::BUFFERED : BufferMode::ABORTING,
        cncFeedLookahead ? MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP,
        &result.identity, &result.ownerLease, plannedEpoch, &plannedOwner, cncFeedLookahead,
        result.blendGeometry.valid ? &result.blendGeometry : nullptr,
        result.blendGeometry.valid ? workspace.cornerArc.velocityPPS : 0.0,
        !cncFeedLookahead && !buffered); // DT: exact native G01 source only.
    result.translationGeneration = m_pendingTranslation.generation;
    result.commandAccepted = accepted;
    if (!accepted)
    {
        result.code = MotionFeedLineCode::PRODUCER_REJECTED;
        return false; // TryLineMove owns formal producer rejection accounting.
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
    const auto revokeAccepted = [&](MotionFeedLineCode code) -> bool
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
        return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    }
    if (!m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow || m_programBlockMotionCapture.count != 1U)
    {
        return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    }
    const MotionProgramBlockSubmission& submission =
        m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted &&
        submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == (cncFeedLookahead ?
            MotionCommandPathMode::CONTINUOUS : MotionCommandPathMode::EXACT_STOP) &&
        submission.identity.epoch == result.identity.epoch &&
        submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId &&
        submission.identity.source == result.identity.source;
    if (!result.captureBound)
    {
        return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
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
        return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    }
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable())
    {
        return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    }
    result.code = MotionFeedLineCode::COMMITTED;
    result.valid = true;
    return true;
}


// BASE69: G90 resolves its absolute rotary target once; G91 retains its signed
// increment. Bookkeeping may start modulo based; its committed end follows the
// actual continuous pulse sweep. The separate G90 proof retains authored WCS/MCS.
bool MotionCore::TryG01RotaryMoveTransactionalTail(int axisIndex, double programmedValue,
    double targetMCS, double feedDegMin, double(&commandedMCSTail)[MAX_AXES],
    MotionFeedLineWorkspace& workspace)
{
    MotionFeedLineReceipt& result = workspace.receipt;
    result.Clear();
    result.rotaryFeed = true;
    workspace.targetPulse.clear();
    workspace.rotaryAxes.clear();
    workspace.rotaryInput = NCRotaryFeedLineInput{};
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source = m_pendingCommandSource.load(std::memory_order_acquire);
    if (m_pContexts == nullptr || m_pContexts->size() > 8U || m_pCoordMgr == nullptr ||
        axisIndex < 3 || axisIndex >= MAX_AXES || static_cast<std::size_t>(axisIndex) >= m_pContexts->size() ||
        !IsNCRotaryFeedNeutralFrame(m_pendingTranslation) ||
        !IsPendingFixedTranslationSourceAllowed() ||
        m_pendingIsAbsoluteMode != (m_pendingTranslation.distanceMode == 90) ||
        !std::isfinite(programmedValue) || !std::isfinite(targetMCS) ||
        !std::isfinite(feedDegMin) || feedDegMin <= 0.0 || feedDegMin > 100.0 ||
        workspace.targetPulse.capacity() < 8U || workspace.rotaryAxes.capacity() < 8U)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID || !plannedOwner.IsValid() ||
        plannedOwner.owner != MotionOwner::AUTO || source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U || HasPendingSafetyOrRecoveryRequests() ||
        !FeedLineNormalOverride(*this))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }
    const unsigned selected = static_cast<unsigned>(axisIndex);
    const std::uint32_t selectedBit = 1U << selected;
    const AxisContext& axis = (*m_pContexts)[selected];
    if (!axis.isExist || axis.axisIndex != axisIndex || axis.axisType != AxisType::ROTARY ||
        !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
        !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
        !std::isfinite(axis.rotaryModulo) || axis.rotaryModulo <= 0.0 ||
        !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
        !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
        !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    NCRotaryFeedLineInput& input = workspace.rotaryInput;
    input.axisIdentity = m_pendingTranslation.axisIdentity;
    input.axisMask = selectedBit;
    input.feedDegMin = feedDegMin;
    input.pulsePerDegree = axis.resolution_PPR / axis.finalLead;
    input.maxVelocityPPS = axis.maxVel_PPS;
    result.validAxisMask = 0U;
    for (unsigned slot = 0U; slot < 8U; ++slot)
    {
        if (!std::isfinite(commandedMCSTail[slot]))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
        input.startMCS[slot] = input.endMCS[slot] = commandedMCSTail[slot];
        double nativePulse = 0.0;
        if (slot < m_pContexts->size() && (*m_pContexts)[slot].isExist)
        {
            nativePulse = (*m_pContexts)[slot].logicalCmdPos.Load();
            if (!std::isfinite(nativePulse))
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            result.validAxisMask |= 1U << slot;
        }
        input.startPulse[slot] = input.endPulse[slot] = nativePulse;
    }
    const bool absolute = m_pendingTranslation.distanceMode == 90;
    const double authoredTarget = absolute ?
        programmedValue + NCTranslationAxisOffsetMM(m_pendingTranslation, selected) :
        commandedMCSTail[selected] + programmedValue;
    if (!std::isfinite(input.pulsePerDegree) || input.pulsePerDegree <= 0.0 ||
        !std::isfinite(authoredTarget) || !NCRotaryFeedDetail::SameBits(targetMCS, authoredTarget) ||
        (!absolute && programmedValue != 0.0 && targetMCS == commandedMCSTail[selected]))
    {
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    const double startPulse = input.startPulse[selected];
    const double baselineMCS = startPulse * axis.finalLead / axis.resolution_PPR;
    double moduloMCS = std::fmod(baselineMCS, axis.rotaryModulo);
    if (moduloMCS < 0.0) moduloMCS += axis.rotaryModulo;
    const double forwardPulse = commandedMCSTail[selected] * input.pulsePerDegree;
    // The accepted predecessor is sealed to the same shared pulse tail and
    // owner/epoch. Otherwise require an exact native or RESET modulo spelling.
    const MotionCncPathTail& previous = m_rotaryFeedProducerTail;
    const bool acceptedBaseline = previous.valid && previous.identity.epoch == plannedEpoch &&
        previous.identity.source == source && previous.ownerLease.Matches(plannedOwner) &&
        (previous.validAxisMask & selectedBit) != 0U &&
        m_g00ProducerQueueTailEpoch == plannedEpoch && m_g00ProducerQueueTailOwnerLease.Matches(plannedOwner) &&
        (m_g00ProducerQueueTailValidMask & selectedBit) != 0U &&
        previous.endPulse[selected] == startPulse && m_g00ProducerQueueTailPulse[selected] == startPulse &&
        NCRotaryFeedDetail::SameBits(previous.endMCS[selected], commandedMCSTail[selected]);
    // BASE70 mixed predecessors retain the exact C native spelling as well.
    const MotionCncPathTail& zcPrevious = m_zcFeedProducerTail;
    // The private Z/C anchor carries only already-proved native bases. A
    // single C move may preserve its stationary Z proof, but an arbitrary
    // unselected Z sampled by a rotary producer is never promoted to one.
    const bool carryZBasis = selected == 3U && zcPrevious.valid && zcPrevious.identity.IsAssigned() &&
        (zcPrevious.axisMask & 4U) != 0U && (zcPrevious.validAxisMask & 4U) != 0U &&
        zcPrevious.identity.epoch == plannedEpoch && zcPrevious.identity.source == source &&
        zcPrevious.ownerLease.Matches(plannedOwner) &&
        m_g00ProducerQueueTailEpoch == plannedEpoch && m_g00ProducerQueueTailOwnerLease.Matches(plannedOwner) &&
        (m_g00ProducerQueueTailValidMask & 4U) != 0U &&
        zcPrevious.endPulse[2] == input.startPulse[2] && m_g00ProducerQueueTailPulse[2] == input.startPulse[2] &&
        NCRotaryFeedDetail::SameBits(zcPrevious.endMCS[2], commandedMCSTail[2]);
    const bool acceptedZCBaseline = zcPrevious.valid && zcPrevious.identity.IsAssigned() &&
        (zcPrevious.axisMask & selectedBit) != 0U && (zcPrevious.validAxisMask & selectedBit) != 0U &&
        zcPrevious.identity.epoch == plannedEpoch && zcPrevious.identity.source == source &&
        zcPrevious.ownerLease.Matches(plannedOwner) &&
        m_g00ProducerQueueTailEpoch == plannedEpoch && m_g00ProducerQueueTailOwnerLease.Matches(plannedOwner) &&
        (m_g00ProducerQueueTailValidMask & selectedBit) != 0U &&
        zcPrevious.endPulse[selected] == startPulse && m_g00ProducerQueueTailPulse[selected] == startPulse &&
        NCRotaryFeedDetail::SameBits(zcPrevious.endMCS[selected], commandedMCSTail[selected]);
    if (!acceptedBaseline && !acceptedZCBaseline &&
        !(std::isfinite(forwardPulse) && forwardPulse == startPulse) &&
        !(std::isfinite(baselineMCS) && baselineMCS == commandedMCSTail[selected]) &&
        !(std::isfinite(moduloMCS) && moduloMCS == commandedMCSTail[selected]))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }
    double targetPulse = startPulse;
    double resolvedMCS = targetMCS;
    if (absolute)
    {
        NCRotaryAbsoluteFeedTarget resolved{};
        if (!TryResolveNCRotaryAbsoluteFeedTarget(commandedMCSTail[selected], startPulse,
            targetMCS, input.pulsePerDegree, axis.useShortestPath, axis.rotaryModulo, resolved))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        targetPulse = resolved.endPulse;
        resolvedMCS = resolved.endMCS;
    }
    else
    {
        const double deltaPulse = programmedValue * input.pulsePerDegree;
        targetPulse = programmedValue == 0.0 ? startPulse : startPulse + deltaPulse;
        if (!std::isfinite(deltaPulse) || !std::isfinite(targetPulse) ||
            (programmedValue != 0.0 && (deltaPulse == 0.0 || targetPulse == startPulse)))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
    }
    input.endMCS[selected] = resolvedMCS;
    input.endPulse[selected] = targetPulse;
    result.geometryCode = static_cast<std::uint32_t>(BuildNCRotaryFeedLine(input, result.rotary));
    if (absolute)
    {
        result.rotary.absoluteTargetWCS = programmedValue;
        result.rotary.absoluteTargetMCS = targetMCS;
        result.rotary.rotaryModulo = axis.rotaryModulo;
        result.rotary.rotaryShortestPath = axis.useShortestPath;
    }
    workspace.accTime = axis.G00_acc_time < 0.001 ? 0.2 : axis.G00_acc_time;
    workspace.decTime = axis.G00_dec_time < 0.001 ? 0.2 : axis.G00_dec_time;
    if (!result.rotary.valid || !std::isfinite(result.rotary.velocityPPS / workspace.accTime) ||
        !std::isfinite(result.rotary.velocityPPS / workspace.decTime))
    {
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    // The actual continuous interval is unwrapped native degrees. Testing only
    // modulo NC endpoints would miss a crossing of a configured travel limit.
    const double nativeStartDeg = startPulse / input.pulsePerDegree;
    const double nativeEndDeg = targetPulse / input.pulsePerDegree;
    if (!std::isfinite(nativeStartDeg) || !std::isfinite(nativeEndDeg) ||
        !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, nativeStartDeg) ||
        !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, nativeEndDeg))
    {
        result.travelLimitRejected = true;
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    workspace.rotaryAxes.push_back(axisIndex);
    workspace.targetPulse.push_back(targetPulse);
    const bool accepted = TryLineMove(workspace.rotaryAxes, workspace.targetPulse,
        result.rotary.velocityPPS, workspace.accTime, workspace.decTime,
        BufferMode::ABORTING, MotionCommandPathMode::EXACT_STOP,
        &result.identity, &result.ownerLease, plannedEpoch, &plannedOwner,
        false, nullptr, 0.0, false, nullptr, true, &result.rotary);
    result.translationGeneration = m_pendingTranslation.generation;
    result.commandAccepted = accepted;
    if (!accepted)
    {
        result.code = MotionFeedLineCode::PRODUCER_REJECTED;
        return false;
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
            result.identity.source == source && result.identity.sourceBlockId == m_pendingSourcePC &&
            result.ownerLease.Matches(plannedOwner) && epoch1 == result.identity.epoch && epoch2 == result.identity.epoch &&
            owner1.IsValid() && owner2.IsValid() && owner1.Matches(result.ownerLease) && owner2.Matches(result.ownerLease) &&
            state1 == state2 && !UnpackMotionOwnerSafetyHandshake(state2) &&
            UnpackMotionOwnerSafetyRequestTicket(state2) == m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto revokeAccepted = [&](MotionFeedLineCode code) -> bool
    {
        result.code = code;
        result.valid = false;
        m_rotaryFeedProducerTail.Clear();
        m_zcFeedProducerTail.Clear();
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    if (!m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 1U) return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    const MotionProgramBlockSubmission& submission = m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted && submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        submission.identity.epoch == result.identity.epoch && submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId && submission.identity.source == result.identity.source;
    if (!result.captureBound) return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    m_g00ProducerQueueTailPulse = input.endPulse;
    for (unsigned slot = 0U; slot < 8U; ++slot)
    {
        if ((result.validAxisMask & (1U << slot)) != 0U)
            (*m_pContexts)[slot].lastQueuedPulse.Store(input.endPulse[slot]);
        commandedMCSTail[slot] = input.endMCS[slot];
    }
    result.tailCommitted = true;
    m_g00ProducerQueueTailValidMask = result.validAxisMask;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_rotaryFeedProducerTail.endMCS = input.endMCS;
    m_rotaryFeedProducerTail.endPulse = input.endPulse;
    m_rotaryFeedProducerTail.identity = result.identity;
    m_rotaryFeedProducerTail.ownerLease = result.ownerLease;
    m_rotaryFeedProducerTail.translationGeneration = result.translationGeneration;
    m_rotaryFeedProducerTail.axisMask = selectedBit;
    m_rotaryFeedProducerTail.validAxisMask = result.validAxisMask;
    m_rotaryFeedProducerTail.valid = true;
    if (selected == 3U)
    {
        m_zcFeedProducerTail = m_rotaryFeedProducerTail;
        // This private mask denotes proven native bases, not command axes.
        m_zcFeedProducerTail.axisMask = 8U | (carryZBasis ? 4U : 0U);
    }
    result.code = MotionFeedLineCode::COMMITTED;
    result.valid = true;
    return true;
}

// BASE71: G90/G91 Z+C uses one time parameter. F remains Z millimetres/minute;
// the C degree rate is derived from the resolved sweep and checked separately.
// G90 resolves absolute Z and the configured C wrap policy once. G91 retains
// both complete signed authored increments without modulo selection.
bool MotionCore::TryG01ZCMoveTransactionalTail(double programmedZ, double programmedC,
    double targetZMCS, double targetCMCS, double feedMMMin,
    double(&commandedMCSTail)[MAX_AXES], MotionFeedLineWorkspace& workspace)
{
    MotionFeedLineReceipt& result = workspace.receipt;
    result.Clear();
    result.zcFeed = true;
    workspace.targetPulse.clear();
    workspace.rotaryAxes.clear();
    workspace.zcInput = NCZCFeedLineInput{};
    const bool absolute = m_pendingTranslation.distanceMode == 90;
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source = m_pendingCommandSource.load(std::memory_order_acquire);
    if (m_pContexts == nullptr || m_pContexts->size() < 4U || m_pContexts->size() > 8U ||
        m_pCoordMgr == nullptr || !IsNCZCFeedNeutralFrame(m_pendingTranslation) ||
        !IsPendingFixedTranslationSourceAllowed() || m_pendingIsAbsoluteMode != absolute ||
        !std::isfinite(programmedZ) || (!absolute && programmedZ == 0.0) ||
        !std::isfinite(programmedC) || (!absolute && programmedC == 0.0) ||
        !std::isfinite(targetZMCS) || !std::isfinite(targetCMCS) ||
        !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        workspace.targetPulse.capacity() < 8U || workspace.rotaryAxes.capacity() < 8U)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID || !plannedOwner.IsValid() ||
        plannedOwner.owner != MotionOwner::AUTO || source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U || HasPendingSafetyOrRecoveryRequests() ||
        !FeedLineNormalOverride(*this))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }

    const std::uint32_t selectedMask = 0x0cU;
    const AxisContext& z = (*m_pContexts)[2U];
    const AxisContext& c = (*m_pContexts)[3U];
    for (unsigned selected = 2U; selected <= 3U; ++selected)
    {
        const AxisContext& axis = (*m_pContexts)[selected];
        if (!axis.isExist || axis.axisIndex != static_cast<int>(selected) ||
            axis.axisType != (selected == 2U ? AxisType::LINEAR : AxisType::ROTARY) ||
            !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0 ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
            !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
            !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
            !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0 ||
            (selected == 3U && (!std::isfinite(axis.rotaryModulo) || axis.rotaryModulo <= 0.0)))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
    }
    NCZCFeedLineInput& input = workspace.zcInput;
    input.axisIdentity = m_pendingTranslation.axisIdentity;
    input.axisMask = selectedMask;
    input.deltaZMM = absolute ? 0.0 : programmedZ;
    input.deltaCDeg = absolute ? 0.0 : programmedC;
    input.absolute = absolute;
    if (absolute)
    {
        input.absoluteTargetZMCS = targetZMCS;
        input.absoluteTargetCMCS = targetCMCS;
        input.rotaryModulo = c.rotaryModulo;
        input.rotaryShortestPath = c.useShortestPath;
    }
    input.feedMMMin = feedMMMin;
    input.pulsePerMM = z.resolution_PPR / z.finalLead;
    input.pulsePerDegree = c.resolution_PPR / c.finalLead;
    input.maxLinearVelocityPPS = z.maxVel_PPS;
    input.maxRotaryVelocityPPS = c.maxVel_PPS;
    input.linearAccTime = z.G00_acc_time;
    input.linearDecTime = z.G00_dec_time;
    input.rotaryAccTime = c.G00_acc_time;
    input.rotaryDecTime = c.G00_dec_time;
    result.validAxisMask = 0U;
    for (unsigned slot = 0U; slot < 8U; ++slot)
    {
        const bool liveExists = slot < m_pContexts->size() && (*m_pContexts)[slot].isExist;
        if (!std::isfinite(commandedMCSTail[slot]) ||
            input.axisIdentity.exists[slot] != (liveExists ? 1U : 0U) ||
            (liveExists && ((*m_pContexts)[slot].axisIndex != static_cast<int>(slot) ||
                static_cast<unsigned>((*m_pContexts)[slot].axisType) != input.axisIdentity.axisType[slot])))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
        input.startMCS[slot] = input.endMCS[slot] = commandedMCSTail[slot];
        const double pulse = liveExists ? (*m_pContexts)[slot].logicalCmdPos.Load() : 0.0;
        if (!std::isfinite(pulse))
        {
            result.code = MotionFeedLineCode::INVALID_INPUT;
            return false;
        }
        input.startPulse[slot] = input.endPulse[slot] = pulse;
        if (liveExists) result.validAxisMask |= 1U << slot;
    }

    const auto acceptedBaseline = [&](const MotionCncPathTail& previous, unsigned selected) -> bool
    {
        const std::uint32_t bit = 1U << selected;
        return previous.valid && previous.identity.IsAssigned() && (previous.axisMask & bit) != 0U &&
            previous.identity.epoch == plannedEpoch && previous.identity.source == source && previous.ownerLease.Matches(plannedOwner) &&
            (previous.validAxisMask & bit) != 0U &&
            m_g00ProducerQueueTailEpoch == plannedEpoch &&
            m_g00ProducerQueueTailOwnerLease.Matches(plannedOwner) &&
            (m_g00ProducerQueueTailValidMask & bit) != 0U &&
            previous.endPulse[selected] == input.startPulse[selected] &&
            m_g00ProducerQueueTailPulse[selected] == input.startPulse[selected] &&
            NCRotaryFeedDetail::SameBits(previous.endMCS[selected], commandedMCSTail[selected]);
    };
    for (unsigned selected = 2U; selected <= 3U; ++selected)
    {
        const AxisContext& axis = (*m_pContexts)[selected];
        const double programmed = selected == 2U ? programmedZ : programmedC;
        const double target = selected == 2U ? targetZMCS : targetCMCS;
        const double ppu = selected == 2U ? input.pulsePerMM : input.pulsePerDegree;
        const double authoredTarget = absolute ?
            programmed + NCTranslationAxisOffsetMM(m_pendingTranslation, selected) :
            commandedMCSTail[selected] + programmed;
        const double startPulse = input.startPulse[selected];
        const double nativeBaseline = startPulse * axis.finalLead / axis.resolution_PPR;
        const double forwardPulse = commandedMCSTail[selected] * ppu;
        double moduloBaseline = 0.0;
        if (selected == 3U)
        {
            moduloBaseline = std::fmod(nativeBaseline, axis.rotaryModulo);
            if (moduloBaseline < 0.0) moduloBaseline += axis.rotaryModulo;
        }
        if (!std::isfinite(ppu) || ppu <= 0.0 || !std::isfinite(authoredTarget) ||
            !NCRotaryFeedDetail::SameBits(target, authoredTarget) ||
            (!absolute && target == commandedMCSTail[selected]))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        // Accepted native spelling is valid only while its pulse tail and tuple
        // still match. Otherwise prove the fresh logical-pulse/MCS baseline.
        // Only a previously selected axis can supply this retained native proof.
        if (!acceptedBaseline(m_zcFeedProducerTail, selected) &&
            !acceptedBaseline(m_rotaryFeedProducerTail, selected) &&
            !(std::isfinite(forwardPulse) && forwardPulse == startPulse) &&
            !(std::isfinite(nativeBaseline) && nativeBaseline == commandedMCSTail[selected]) &&
            !(selected == 3U && std::isfinite(moduloBaseline) && moduloBaseline == commandedMCSTail[selected]))
        {
            result.code = MotionFeedLineCode::NOT_READY;
            return false;
        }
        if (!absolute)
        {
            const double deltaPulse = programmed * ppu;
            const double targetPulse = startPulse + deltaPulse;
            if (!std::isfinite(deltaPulse) || !std::isfinite(targetPulse) ||
                deltaPulse == 0.0 || targetPulse == startPulse)
            {
                result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
                return false;
            }
            input.endMCS[selected] = target;
            input.endPulse[selected] = targetPulse;
        }
    }
    if (absolute)
    {
        NCZCAbsoluteFeedTarget resolved{};
        if (!TryResolveNCZCAbsoluteFeedTarget(input.startMCS[2], input.startPulse[2],
            input.startMCS[3], input.startPulse[3], targetZMCS, targetCMCS,
            input.pulsePerMM, input.pulsePerDegree, c.useShortestPath, c.rotaryModulo, resolved))
        {
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        input.endMCS[2] = resolved.endZMCS;
        input.endMCS[3] = resolved.endCMCS;
        input.endPulse[2] = resolved.endZPulse;
        input.endPulse[3] = resolved.endCPulse;
        input.deltaZMM = resolved.deltaZMM;
        input.deltaCDeg = resolved.deltaCDeg;
    }
    result.geometryCode = static_cast<std::uint32_t>(BuildNCZCFeedLine(input, result.zc));
    if (!result.zc.valid)
    {
        result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
        return false;
    }
    if (absolute)
    {
        result.zc.absoluteTargetZWCS = programmedZ;
        result.zc.absoluteTargetCWCS = programmedC;
    }
    workspace.accTime = result.zc.accTime;
    workspace.decTime = result.zc.decTime;
    for (unsigned selected = 2U; selected <= 3U; ++selected)
    {
        const AxisContext& axis = (*m_pContexts)[selected];
        const double ppu = selected == 2U ? input.pulsePerMM : input.pulsePerDegree;
        const double physicalStart = input.startPulse[selected] / ppu;
        const double physicalEnd = input.endPulse[selected] / ppu;
        // Linear interpolation is monotone on each selected physical axis, so
        // both physical endpoints prove the entire unwrapped segment interval.
        if (!std::isfinite(physicalStart) || !std::isfinite(physicalEnd) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, input.startMCS[selected]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, input.endMCS[selected]) ||
            (absolute && !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,
                selected == 2U ? targetZMCS : targetCMCS)) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, physicalStart) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, physicalEnd))
        {
            result.travelLimitRejected = true;
            result.code = MotionFeedLineCode::GEOMETRY_REJECTED;
            return false;
        }
        workspace.rotaryAxes.push_back(static_cast<int>(selected));
        workspace.targetPulse.push_back(input.endPulse[selected]);
    }
    const bool accepted = TryLineMove(workspace.rotaryAxes, workspace.targetPulse,
        result.zc.velocityPPS, workspace.accTime, workspace.decTime,
        BufferMode::ABORTING, MotionCommandPathMode::EXACT_STOP,
        &result.identity, &result.ownerLease, plannedEpoch, &plannedOwner,
        false, nullptr, 0.0, false, nullptr, false, nullptr, true, &result.zc);
    result.translationGeneration = m_pendingTranslation.generation;
    result.commandAccepted = accepted;
    if (!accepted)
    {
        result.code = MotionFeedLineCode::PRODUCER_REJECTED;
        return false;
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
            result.identity.source == source && result.identity.sourceBlockId == m_pendingSourcePC &&
            result.ownerLease.Matches(plannedOwner) && epoch1 == result.identity.epoch && epoch2 == result.identity.epoch &&
            owner1.IsValid() && owner2.IsValid() && owner1.Matches(result.ownerLease) && owner2.Matches(result.ownerLease) &&
            state1 == state2 && !UnpackMotionOwnerSafetyHandshake(state2) &&
            UnpackMotionOwnerSafetyRequestTicket(state2) == m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto revokeAccepted = [&](MotionFeedLineCode code) -> bool
    {
        result.code = code;
        result.valid = false;
        m_zcFeedProducerTail.Clear();
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    if (!m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 1U) return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    const MotionProgramBlockSubmission& submission = m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted && submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        submission.identity.epoch == result.identity.epoch && submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId && submission.identity.source == result.identity.source;
    if (!result.captureBound) return revokeAccepted(MotionFeedLineCode::CAPTURE_MISMATCH);
    m_g00ProducerQueueTailPulse = input.endPulse;
    for (unsigned slot = 0U; slot < 8U; ++slot)
    {
        if ((result.validAxisMask & (1U << slot)) != 0U)
            (*m_pContexts)[slot].lastQueuedPulse.Store(input.endPulse[slot]);
        commandedMCSTail[slot] = input.endMCS[slot];
    }
    result.tailCommitted = true;
    m_g00ProducerQueueTailValidMask = result.validAxisMask;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable()) return revokeAccepted(MotionFeedLineCode::STALE_AFTER_ACCEPT);
    m_zcFeedProducerTail.endMCS = input.endMCS;
    m_zcFeedProducerTail.endPulse = input.endPulse;
    m_zcFeedProducerTail.identity = result.identity;
    m_zcFeedProducerTail.ownerLease = result.ownerLease;
    m_zcFeedProducerTail.translationGeneration = result.translationGeneration;
    m_zcFeedProducerTail.axisMask = selectedMask;
    m_zcFeedProducerTail.validAxisMask = result.validAxisMask;
    m_zcFeedProducerTail.valid = true;
    result.code = MotionFeedLineCode::COMMITTED;
    result.valid = true;
    return true;
}


// Both mixed producers validate the same tagged tail. No current-position sample
// may substitute for an already committed successor's start.
bool MotionCore::IsCncPathProducerTailCurrent(const MotionCncPathTail& tail,
    std::uint32_t mask, const double* commandedTail, MotionExecutionEpoch epoch,
    const MotionOwnerLease& owner, MotionCommandSource source) const noexcept
{
    if (commandedTail == nullptr || !tail.valid || tail.axisMask != mask ||
        tail.translationGeneration != m_pendingTranslation.generation ||
        !IsPendingFixedTranslationSourceAllowed() ||
        !tail.identity.IsAssigned() || tail.identity.source != source || tail.identity.epoch != epoch ||
        tail.identity.sourceBlockId >= static_cast<MotionSourceBlockId>((std::numeric_limits<int>::max)()) ||
        m_pendingSourcePC != static_cast<int>(tail.identity.sourceBlockId) + 1 ||
        m_nextSegmentId.load(std::memory_order_acquire) !=
        static_cast<MotionSegmentId>(tail.identity.segmentId + 1ULL) ||
        !tail.ownerLease.Matches(owner) || m_g00ProducerQueueTailEpoch != epoch ||
        !m_g00ProducerQueueTailOwnerLease.Matches(owner) ||
        m_g00ProducerQueueTailValidMask != tail.validAxisMask) return false;
    for (std::size_t i = 0U; i < 8U; ++i)
    {
        if ((tail.validAxisMask & (1U << static_cast<unsigned>(i))) == 0U) continue;
        if (!std::isfinite(tail.endMCS[i]) || !std::isfinite(tail.endPulse[i]) ||
            commandedTail[i] != tail.endMCS[i] || m_g00ProducerQueueTailPulse[i] != tail.endPulse[i]) return false;
    }
    return true;
}
