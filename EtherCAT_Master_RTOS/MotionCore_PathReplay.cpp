#include "MotionCore.h"
#include "MotionRetainedInterval.h"
#include "NCTranslationArcPrecision.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#if defined(_MSC_VER)
#define BZ_RETAINED_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BZ_RETAINED_NOINLINE __attribute__((noinline))
#else
#define BZ_RETAINED_NOINLINE
#endif

namespace
{
    BZ_RETAINED_NOINLINE bool RetainedNormalOverride(const MotionCore& motion) noexcept
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

    bool RetainedUnitConsistent(double pulse, double mcs, double ppm) noexcept
    {
        if (!std::isfinite(pulse) || !std::isfinite(mcs) || !std::isfinite(ppm) || ppm <= 0.0)
            return false;
        const double converted = pulse / ppm;
        const double scale = (std::max)(1.0, (std::max)(std::abs(mcs), std::abs(converted)));
        const double error = std::abs(converted - mcs);
        return std::isfinite(converted) && error <= 64.0 * std::numeric_limits<double>::epsilon() * scale && error <= 1e-7;
    }

    BZ_RETAINED_NOINLINE bool PrepareRetainedTraversal(
        const std::vector<AxisContext>* contexts,
        const NCPathCoreRetainedGeometry& geometry, double startU, double endU, double feedMMMin,
        const MotionArcTravelGuard& travelGuard,
        const double* commandedTail, MotionPathCoreRetainedWorkspace& workspace) noexcept
    {
        MotionPathCoreRetainedReceipt& result = workspace.receipt;
        workspace.stagedPulse.fill(0.0);
        workspace.pulsePerMM.fill(0.0);
        workspace.accTime = workspace.decTime = workspace.velocityPPS = 0.0;
        std::uint32_t validMask = 0U;
        for (std::size_t axisIndex = 0U; axisIndex < 8U; ++axisIndex)
        {
            if (!std::isfinite(commandedTail[axisIndex]))
            {
                result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
            }
            workspace.stagedMCS[axisIndex] = commandedTail[axisIndex];
            if (axisIndex >= contexts->size() || !(*contexts)[axisIndex].isExist)
                continue;
            const AxisContext& axis = (*contexts)[axisIndex];
            const double pulse = axis.logicalCmdPos.Load();
            if (!std::isfinite(pulse) || !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            {
                result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
            }
            const double ppm = axis.resolution_PPR / axis.finalLead;
            double mcs = pulse * axis.finalLead / axis.resolution_PPR;
            if (axis.axisType == AxisType::ROTARY)
            {
                if (!std::isfinite(axis.rotaryModulo) || axis.rotaryModulo <= 0.0)
                {
                    result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
                }
                mcs = std::fmod(mcs, axis.rotaryModulo);
                if (mcs < 0.0) mcs += axis.rotaryModulo;
            }
            if (!std::isfinite(ppm) || ppm <= 0.0 || !std::isfinite(mcs))
            {
                result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
            }
            workspace.stagedPulse[axisIndex] = pulse;
            workspace.stagedMCS[axisIndex] = mcs;
            workspace.pulsePerMM[axisIndex] = ppm;
            validMask |= (1U << static_cast<unsigned>(axisIndex));
            if ((geometry.axisMask & (1U << static_cast<unsigned>(axisIndex))) == 0U)
                continue;
            if (axis.axisType != AxisType::LINEAR ||
                !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
                !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0 ||
                !RetainedUnitConsistent(geometry.startPulse[axisIndex], geometry.startMCS[axisIndex], ppm) ||
                !RetainedUnitConsistent(geometry.endPulse[axisIndex], geometry.endMCS[axisIndex], ppm))
            {
                result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
            }
            workspace.accTime = (std::max)(workspace.accTime, axis.G00_acc_time);
            workspace.decTime = (std::max)(workspace.decTime, axis.G00_dec_time);
        }
        if ((geometry.axisMask & validMask) != geometry.axisMask)
        {
            result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
        }
        if (!DoesNCPathCoreRetainedStartMatchAt(geometry, startU,
            workspace.stagedMCS, workspace.stagedPulse, validMask, workspace.pulsePerMM))
        {
            result.code = MotionPathCoreRetainedCode::START_MISMATCH; return false;
        }
        if (workspace.accTime < 0.001) workspace.accTime = 0.2;
        if (workspace.decTime < 0.001) workspace.decTime = 0.2;
        if (!geometry.point)
            workspace.velocityPPS = (feedMMMin / 60.0) * (geometry.lengthPulse / geometry.lengthMM);
        if (!std::isfinite(workspace.velocityPPS) || (!geometry.point && workspace.velocityPPS < 1.0) ||
            !std::isfinite(workspace.velocityPPS / workspace.accTime) ||
            !std::isfinite(workspace.velocityPPS / workspace.decTime))
        {
            result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
        }
        if (geometry.kind == NCPathCoreRetainedKind::ARC &&
            (workspace.pulsePerMM[0] != workspace.pulsePerMM[1] ||
                !RetainedUnitConsistent(geometry.centerPulse[0], geometry.centerMCS[0], workspace.pulsePerMM[0]) ||
                !RetainedUnitConsistent(geometry.centerPulse[1], geometry.centerMCS[1], workspace.pulsePerMM[1]) ||
                !RetainedUnitConsistent(geometry.radiusPulse, geometry.radiusMM, workspace.pulsePerMM[0])))
        {
            result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
        }
        for (std::size_t axisIndex = 0U; axisIndex < 3U; ++axisIndex)
        {
            if ((geometry.axisMask & (1U << static_cast<unsigned>(axisIndex))) == 0U)
                continue;
            const double ratio = geometry.point ? 0.0 :
                (geometry.endPulse[axisIndex] - geometry.startPulse[axisIndex]) / geometry.lengthPulse;
            const double peakVelocity = geometry.kind == NCPathCoreRetainedKind::ARC
                ? workspace.velocityPPS : std::abs(workspace.velocityPPS * ratio);
            if (!std::isfinite(ratio) || !std::isfinite(peakVelocity) ||
                (geometry.kind == NCPathCoreRetainedKind::LINE && !geometry.point &&
                    geometry.endPulse[axisIndex] != geometry.startPulse[axisIndex] && ratio == 0.0) ||
                peakVelocity > (*contexts)[axisIndex].maxVel_PPS)
            {
                result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
            }
            const double lower = geometry.kind == NCPathCoreRetainedKind::ARC
                ? geometry.boundsMinMCS[axisIndex] : (std::min)(geometry.startMCS[axisIndex], geometry.endMCS[axisIndex]);
            const double upper = geometry.kind == NCPathCoreRetainedKind::ARC
                ? geometry.boundsMaxMCS[axisIndex] : (std::max)(geometry.startMCS[axisIndex], geometry.endMCS[axisIndex]);
            if (!travelGuard.check(travelGuard.context, static_cast<int>(axisIndex), lower) ||
                !travelGuard.check(travelGuard.context, static_cast<int>(axisIndex), upper))
            {
                result.travelLimitRejected = true; result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
            }
        }
        // Evaluate the ORIGINAL source at the requested interval endpoint.
        // Small permitted starting residue never changes the stored geometry.
        bool pulseChanges = false, mcsChanges = false;
        for (std::size_t axisIndex = 0U; axisIndex < 8U; ++axisIndex)
        {
            const std::uint32_t axis = static_cast<std::uint32_t>(axisIndex);
            double startPulse = 0.0, startMCS = 0.0;
            if (!EvaluateNCPathCoreRetainedPulseCanonical(geometry, axis, startU, startPulse) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(geometry, axis, startU, startMCS) ||
                !EvaluateNCPathCoreRetainedPulseCanonical(geometry, axis, endU, workspace.stagedPulse[axisIndex]) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(geometry, axis, endU, workspace.stagedMCS[axisIndex]))
            {
                result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
            }
            pulseChanges = pulseChanges || startPulse != workspace.stagedPulse[axisIndex];
            mcsChanges = mcsChanges || startMCS != workspace.stagedMCS[axisIndex];
        }
        // A full original circle legitimately returns to its own endpoint.
        // A smaller nonpoint interval must remain representable in both units.
        if (!geometry.point && !(geometry.fullCircle && std::abs(endU - startU) == 1.0) &&
            (!pulseChanges || !mcsChanges))
        {
            result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
        }
        result.validAxisMask = validMask;
        return true;
    }
    BZ_RETAINED_NOINLINE void ClearRetainedCommand(MotionCommand& command) noexcept
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

BZ_RETAINED_NOINLINE void MotionCore::RejectPathCoreRetainedPublication(
    MotionCommand& command,
    MotionExecutionEpoch plannedEpoch,
    MotionExecutionEpoch publishedEpoch,
    MotionCommandSource source,
    const MotionOwnerLease& plannedOwner,
    MotionPathCoreRetainedReceipt& result) noexcept
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
    result.code = MotionPathCoreRetainedCode::PRODUCER_REJECTED;
}

bool MotionCore::TryPathCoreRetainedMoveTransactionalTail(
    const NCPathCoreRetainedGeometry& geometry, bool reverse, double feedMMMin,
    const MotionArcTravelGuard& travelGuard,
    double(&commandedMCSTail)[MAX_AXES],
    MotionPathCoreRetainedWorkspace& workspace, MotionCommand& commandWorkspace)
{
    return TryPathCoreRetainedIntervalMoveTransactionalTail(geometry,
        reverse ? 1.0 : 0.0, reverse ? 0.0 : 1.0, feedMMMin,
        travelGuard, commandedMCSTail, workspace, commandWorkspace);
}

bool MotionCore::TryPathCoreRetainedIntervalMoveTransactionalTail(
    const NCPathCoreRetainedGeometry& geometry, double startU, double endU, double feedMMMin,
    const MotionArcTravelGuard& travelGuard,
    double(&commandedMCSTail)[MAX_AXES],
    MotionPathCoreRetainedWorkspace& workspace, MotionCommand& commandWorkspace)
{
    static_assert(MAX_AXES == 8, "CA fixed native-axis workspace mismatch.");
    MotionPathCoreRetainedReceipt& result = workspace.receipt;
    result.Clear();
    double intervalDistance = 0.0, intervalDistanceMM = 0.0;
    const bool reverse = endU < startU;
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source = m_pendingCommandSource.load(std::memory_order_acquire);
    if (m_pContexts == nullptr || m_pContexts->empty() || m_pContexts->size() > 8U ||
        travelGuard.check == nullptr || !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        (geometry.kind == NCPathCoreRetainedKind::LINE_ARC || !IsNCPathCoreRetainedGeometryValid(geometry)) ||
        !IsMotionRetainedIntervalBoundsValid(startU, endU, geometry.lengthPulse, intervalDistance) ||
        !IsMotionRetainedIntervalBoundsValid(startU, endU, geometry.lengthMM, intervalDistanceMM))
    {
        result.code = MotionPathCoreRetainedCode::INVALID_INPUT; return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U || HasPendingSafetyOrRecoveryRequests() ||
        !RetainedNormalOverride(*this) || !IsPendingFixedTranslationSourceAllowed() ||
        m_pendingToolRadMode != 40 || !m_pendingIsAbsoluteMode ||
        m_pendingG51Active ||
        m_pendingMirrorMask != 0U || m_pendingG16Active || m_pendingG162Active || m_pendingPlaneMode != 17)
    {
        result.code = MotionPathCoreRetainedCode::NOT_READY; return false;
    }
    // Retained arithmetic metadata carries no authority: bind every positive
    // allowance to this fresh submission's current frozen source.
    float sourceRoundoffMM = 0.0F;
    if (geometry.sourceRoundoffMM != 0.0F &&
        (IsNCTranslationSnapshotEmpty(m_pendingTranslation) ||
            !TryGetNCTranslationArcRoundoffMM(m_pendingTranslation, sourceRoundoffMM, geometry.fullCircle) ||
            geometry.sourceRoundoffMM != sourceRoundoffMM))
    {
        result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED;
        return false;
    }
    if (!PrepareRetainedTraversal(m_pContexts, geometry, startU, endU, feedMMMin,
        travelGuard, commandedMCSTail, workspace))
        return false;
    MotionCommand& cmd = commandWorkspace;
    ClearRetainedCommand(cmd);
    const bool arc = geometry.kind == NCPathCoreRetainedKind::ARC;
    cmd.dir = arc ? (reverse ? -geometry.direction : geometry.direction) : 0;
    cmd.mode = !arc ? InterpolationMode::LINEAR : cmd.dir == 1 ? InterpolationMode::CIRCULAR_CCW : InterpolationMode::CIRCULAR_CW;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        cmd.mem_startPos[axis] = geometry.startPulse[axis];
        cmd.mem_ratio[axis] = geometry.endPulse[axis];
        if ((geometry.axisMask & (1U << static_cast<unsigned>(axis))) != 0U)
        {
            cmd.axisIndices[cmd.axisCount] = static_cast<int>(axis);
            cmd.targetPos[cmd.axisCount] = workspace.stagedPulse[axis];
            ++cmd.axisCount;
        }
    }
    cmd.centerPos[0] = geometry.centerPulse[0];
    cmd.centerPos[1] = geometry.centerPulse[1];
    cmd.startRadius = cmd.endRadius = geometry.radiusPulse;
    cmd.mem_radius = geometry.radiusPulse;
    cmd.mem_centerX = geometry.centerPulse[0];
    cmd.mem_centerY = geometry.centerPulse[1];
    cmd.mem_startAngle = geometry.startAngle;
    cmd.mem_totalAngle = geometry.sweepRadians;
    cmd.mem_totalDist = geometry.lengthPulse;
    cmd.mem_transformOrigin[0] = startU;
    cmd.mem_transformOrigin[1] = endU;
    cmd.mem_transformOrigin[2] = 1.0;
    cmd.targetVel = workspace.velocityPPS;
    cmd.accTime = workspace.accTime;
    cmd.decTime = workspace.decTime;
    cmd.commandPathMode = MotionCommandPathMode::EXACT_STOP;
    cmd.pathCorePlanarCircle = arc;
    cmd.pathCoreFullCircle = arc && geometry.fullCircle;
    cmd.pathCoreRetainedTraversal = true;
    cmd.pathCoreRetainedReverse = reverse;
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

    // Both sides of the queue use this exact canonical pulse evaluator.
    for (int slot = 0; slot < cmd.axisCount; ++slot)
    {
        double target = 0.0;
        if (!EvaluateMotionRetainedPulseCanonical(cmd,
            static_cast<std::size_t>(cmd.axisIndices[slot]), endU, target) || target != cmd.targetPos[slot])
        {
            result.code = MotionPathCoreRetainedCode::GEOMETRY_REJECTED; return false;
        }
    }

    MotionExecutionEpoch publishedEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    if (!TryPublishOwnerAuthorizedAbortingExecutionEpoch(
        source, plannedEpoch, plannedOwner, publishedEpoch))
    {
        RejectPathCoreRetainedPublication(cmd, plannedEpoch, publishedEpoch,
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
        result.code = MotionPathCoreRetainedCode::PRODUCER_REJECTED;
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
    const auto revokeAccepted = [&](MotionPathCoreRetainedCode code) -> bool
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
        return revokeAccepted(MotionPathCoreRetainedCode::STALE_AFTER_ACCEPT);
    }
    if (!m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow || m_programBlockMotionCapture.count != 1U)
    {
        return revokeAccepted(MotionPathCoreRetainedCode::CAPTURE_MISMATCH);
    }
    const MotionProgramBlockSubmission& submission =
        m_programBlockMotionCapture.submissions[0U];
    result.captureBound = submission.translationGeneration == result.translationGeneration &&
        submission.producerAccepted &&
        submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        submission.identity.epoch == result.identity.epoch &&
        submission.identity.segmentId == result.identity.segmentId &&
        submission.identity.sourceBlockId == result.identity.sourceBlockId &&
        submission.identity.source == result.identity.source;
    if (!result.captureBound)
    {
        return revokeAccepted(MotionPathCoreRetainedCode::CAPTURE_MISMATCH);
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
        return revokeAccepted(MotionPathCoreRetainedCode::STALE_AFTER_ACCEPT);
    }
    m_g00ProducerQueueTailEpoch = result.identity.epoch;
    m_g00ProducerQueueTailOwnerLease = result.ownerLease;
    if (!tupleStable())
    {
        return revokeAccepted(MotionPathCoreRetainedCode::STALE_AFTER_ACCEPT);
    }
    result.code = MotionPathCoreRetainedCode::COMMITTED;
    result.valid = true;
    return true;
}

#undef BZ_RETAINED_NOINLINE
