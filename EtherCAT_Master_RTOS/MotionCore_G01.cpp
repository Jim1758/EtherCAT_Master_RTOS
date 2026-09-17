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
        std::uint32_t endpointAxisMask = 0U) noexcept
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
            const double targetPulse = programmed ? targetMCS[index] * pulsePerMM :
                workspace.input.startPulse[slot];
            const double target = programmed ? targetMCS[index] : workspace.input.startMCS[slot];
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
    std::uint32_t endpointAxisMask, bool requirePlanarBaselineMatch)
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
    const bool cutterActive = m_pendingTranslation.cutterMode != 40;
    const bool coupledPlanarEndpoint = NCTranslationHasPlanarRotation(m_pendingTranslation) ||
        m_pendingTranslation.polarMode == 16;
    const bool coupledPlanarQueuedLine = coupledPlanarEndpoint && cncFeedLookahead;
    if (!IsPendingFixedTranslationSourceAllowed() ||
        (cutterActive && (cncFeedLookahead || predecessor != nullptr ||
            cornerNextMCS != nullptr || cornerToleranceMM != 0.0 ||
            cornerTravelGuard != nullptr || cornerNextFeedMMMin != 0.0 ||
            axes.size() != 2U || axes[0] != 0 || axes[1] != 1 ||
            endpointAxisMask != 3U || !requirePlanarBaselineMatch)) ||
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
        commandedMCSTail, workspace, predecessor, endpointAxisMask))
    {
        return false;
    }

    const bool incrementalEndpoint = m_pendingTranslation.distanceMode == 91;
    const std::uint32_t requiredBaselineMask =
        ((requirePlanarBaselineMatch || coupledPlanarQueuedLine) ? 3U : 0U) |
        (incrementalEndpoint ? workspace.input.axisMask : 0U);
    // A first coupled XY plain/Q line proves the sampled native basis used by geometry.
    // A buffered line already proved its immutable accepted predecessor above;
    // live axes can still be inside an earlier segment and must not replace it.
    if (((requirePlanarBaselineMatch || incrementalEndpoint) &&
            !coupledPlanarQueuedLine && (buffered || cncFeedLookahead)) ||
        ((requirePlanarBaselineMatch || coupledPlanarQueuedLine || incrementalEndpoint) && !buffered &&
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
