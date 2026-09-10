#include "MotionCore.h"

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
    BY_ARC_NOINLINE bool FeedArcNormalOverride(const MotionCore& motion) noexcept
    {
        const MotionFeedHoldStopSnapshot snapshot = motion.GetFeedHoldStopSnapshot();
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

    BY_ARC_NOINLINE bool PrepareFeedArcGeometry(
        const std::vector<AxisContext>* contexts,
        const std::array<double, 8U>& targetMCS,
        const double* commandedTail,
        MotionFeedArcWorkspace& workspace) noexcept
    {
        MotionFeedArcReceipt& result = workspace.receipt;
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
            const double logicalPulse = axis.logicalCmdPos.Load();
            if (!std::isfinite(logicalPulse) ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            {
                result.code = MotionFeedArcCode::INVALID_INPUT;
                return false;
            }
            double baseline = logicalPulse * axis.finalLead / axis.resolution_PPR;
            if (axis.axisType == AxisType::LINEAR && std::isfinite(baseline))
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
        for (std::size_t slot = 0U; slot < 2U; ++slot)
        {
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
            // Full-circle endpoint is the actual sampled start, bit for bit.
            // Do not round-trip its pulses through the MCS conversion.
            if (!workspace.input.fullCircle)
            {
                const double targetPulse = targetMCS[slot] * pulsePerMM;
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
            workspace.input.pulsePerMM[slot] = pulsePerMM;
            workspace.input.maxVelocityPPS[slot] = axis.maxVel_PPS;
            workspace.accTime = (std::max)(workspace.accTime, axis.G00_acc_time);
            workspace.decTime = (std::max)(workspace.decTime, axis.G00_dec_time);
        }
        if (workspace.accTime < 0.001) workspace.accTime = 0.2;
        if (workspace.decTime < 0.001) workspace.decTime = 0.2;
        result.geometryCode = static_cast<std::uint32_t>(
            BuildNCPathCoreFeedArc(workspace.input, result.arc));
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
    static_assert(MAX_AXES == 8, "BY fixed workspace must match Motion axes.");
    MotionFeedArcReceipt& result = workspace.receipt;
    result.Clear();
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
    if (m_pContexts == nullptr || m_pContexts->size() < 2U ||
        m_pContexts->size() > 8U || travelGuard.check == nullptr ||
        !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        (direction != -1 && direction != 1) ||
        !std::isfinite(centerOffsetMM[0]) || !std::isfinite(centerOffsetMM[1]))
    {
        result.code = MotionFeedArcCode::INVALID_INPUT;
        return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U ||
        HasPendingSafetyOrRecoveryRequests() || !FeedArcNormalOverride(*this))
    {
        result.code = MotionFeedArcCode::NOT_READY;
        return false;
    }
    if (!PrepareFeedArcGeometry(m_pContexts, targetMCS, commandedMCSTail, workspace))
        return false;
    // Endpoints alone are insufficient: every directed-sweep cardinal bound
    // must pass the same software travel policy before changing epoch/queue.
    for (int axis = 0; axis < 2; ++axis)
    {
        const std::size_t slot = static_cast<std::size_t>(axis);
        if (!travelGuard.check(travelGuard.context, axis, result.arc.boundsMinMCS[slot]) ||
            !travelGuard.check(travelGuard.context, axis, result.arc.boundsMaxMCS[slot]))
        {
            result.travelLimitRejected = true;
            result.code = MotionFeedArcCode::GEOMETRY_REJECTED;
            return false;
        }
    }

    MotionCommand& cmd = commandWorkspace;
    ClearFeedArcCommand(cmd);
    cmd.mode = direction == 1 ? InterpolationMode::CIRCULAR_CCW : InterpolationMode::CIRCULAR_CW;
    cmd.axisCount = 2;
    cmd.axisIndices[0] = 0;
    cmd.axisIndices[1] = 1;
    cmd.targetPos[0] = result.arc.endPulse[0];
    cmd.targetPos[1] = result.arc.endPulse[1];
    cmd.centerPos[0] = result.arc.centerPulse[0];
    cmd.centerPos[1] = result.arc.centerPulse[1];
    cmd.startRadius = result.arc.radiusPulse;
    cmd.endRadius = result.arc.radiusPulse;
    cmd.dir = direction;
    cmd.targetVel = result.arc.velocityPPS;
    cmd.accTime = workspace.accTime;
    cmd.decTime = workspace.decTime;
    cmd.commandPathMode = MotionCommandPathMode::EXACT_STOP;
    cmd.pathCorePlanarCircle = true;
    cmd.pathCoreFullCircle = fullCircle;
    cmd.sourceLinePC = m_pendingSourcePC;
    cmd.sourceWCS = m_pendingSourceWCS;
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

    MotionExecutionEpoch publishedEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    if (!TryPublishOwnerAuthorizedAbortingExecutionEpoch(
        source, plannedEpoch, plannedOwner, publishedEpoch))
    {
        RejectPathCoreArcPublication(cmd, plannedEpoch, publishedEpoch,
            source, plannedOwner, result);
        return false;
    }
    AssignExecutionIdentity(cmd, publishedEpoch, source, plannedOwner);
    result.identity = cmd.execution;
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
    result.captureBound = submission.producerAccepted &&
        submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
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
