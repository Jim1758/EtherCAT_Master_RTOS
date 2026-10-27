#include "MotionCore.h"
#include "MotionRetainedInterval.h"
#include "NCTranslationArcPrecision.h"
#include "NCG76DiagnosticPrint.h"
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

namespace
{
    std::uint64_t MixedNativeTailFingerprint(const std::array<double,8U>& pulse,
        const double* mcs,double overrideRatio) noexcept
    {
        std::uint64_t value=MOTION_QUEUE_TAIL_FINGERPRINT_SEED;
        const auto fold=[&](double v) { std::uint64_t bits=0ULL; std::memcpy(&bits,&v,sizeof(bits)); value=(value^bits)*1099511628211ULL; };
        for (unsigned a=0U;a<8U;++a) { fold(mcs?mcs[a]:0.0); fold(pulse[a]); }
        fold(overrideRatio); return value;
    }
}

bool MotionCore::TryG76MixedMoveTransactionalTail(const NCG76Mixed::Plan& plan,
    double* commandedMCSTail,MotionG76OrbitReceipt& result)
{
    result.Clear();
    auto& receipt=result.tail;
    const auto epoch=GetCurrentExecutionEpoch();
    const auto owner=GetMotionOwnerLease();
    const auto source=m_pendingCommandSource.load(std::memory_order_acquire);
    const double rapidOverride=G00_overrideRatio;
    receipt.transactionSequence=AllocateQueueTailTransactionSequence(); receipt.attempted=true;
    receipt.beforeFingerprint=MixedNativeTailFingerprint(m_g00ProducerQueueTailPulse,commandedMCSTail,rapidOverride);
    receipt.committedFingerprint=receipt.beforeFingerprint;
    const auto reject=[&](bool invalid,bool formal,MotionRejectReason reason,MotionG76AdmissionGate gate,
        int axis=-1,int segment=-1)->bool {
        result.rejectionGate=gate; result.rejectionAxis=axis; result.rejectionSegment=segment; result.rejectionReason=reason;
        if (formal) {
            MotionCommand c{}; c.mode=InterpolationMode::LINEAR; c.sourceLinePC=m_pendingSourcePC; c.commandPathMode=MotionCommandPathMode::EXACT_STOP;
            if (invalid) RejectInvalidProducerMotionCommand(c,GetCurrentExecutionEpoch(),source,GetMotionOwnerLease(),&receipt.identity,&receipt.ownerLease);
            else RejectNonGeometryProducerMotionCommand(c,GetCurrentExecutionEpoch(),source,GetMotionOwnerLease(),reason,&receipt.identity,&receipt.ownerLease);
        }
        receipt.committedFingerprint=MixedNativeTailFingerprint(m_g00ProducerQueueTailPulse,commandedMCSTail,G00_overrideRatio);
        receipt.preservedOnReject=receipt.committedFingerprint==receipt.beforeFingerprint;
        receipt.accountingValid=receipt.preservedOnReject;
        PublishQueueTailTransactionReceipt(receipt,invalid); return false;
    };
    if (!NCG76Mixed::Valid(plan) || !m_g76OrbitProducer || !m_pContexts || !m_pCoordMgr ||
        !commandedMCSTail || m_pContexts->size()<3U || m_pContexts->size()>8U ||
        !std::isfinite(rapidOverride) || rapidOverride<0.0)
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::PLAN_INPUT);
    const auto& input=plan.input;
    if (!epoch || !owner.IsValid() || owner.owner!=MotionOwner::AUTO || source!=MotionCommandSource::NC_MEMORY ||
        !owner.Matches(input.scope.lease) || !IsMotionOwnerLeaseCurrent(owner) ||
        m_pendingSourcePC<0 || input.nominal.pc!=m_pendingSourcePC)
        return reject(false,true,MotionRejectReason::OWNER_CONFLICT,MotionG76AdmissionGate::OWNER_SCOPE);
    if (!IsPendingFixedTranslationSourceAllowed() || !NCG76Mixed::FrameSupported(m_pendingTranslation) ||
        !SameNCTranslationSnapshot(input.translation,m_pendingTranslation) ||
        input.scope.translationGeneration!=m_pendingTranslation.generation ||
        !MatchesNCTranslation(m_pendingTranslation) || m_pendingPlaneMode!=17 || m_pendingToolRadMode!=40 ||
        m_pendingG16Active || m_pendingG162Active)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::SOURCE_FRAME);
    const auto hold=GetFeedHoldStopSnapshot();
    if (HasPendingSafetyOrRecoveryRequests() || m_edm94.Active() || m_edm94Claim.load(std::memory_order_acquire)!=0ULL ||
        !IsGroupDone() || GetQueueSize()!=0U || GetCommandIngressSize()!=0U || GetCommandReplaySize()!=0U ||
        !HasExactExecutionDrainAcknowledgement(epoch,owner) || !hold.groupDone || hold.groupActive || !hold.commandStopped ||
        hold.groupFaulted || hold.groupEmergencyStopped || hold.safetyOrRecoveryPending || hold.faultedAxes ||
        hold.overrideZero || !std::isfinite(hold.feedrateOverride) || hold.feedrateOverride!=1.0)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::RUNTIME_READY);
    if (!m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow || m_programBlockMotionCapture.count!=0U)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::CAPTURE_SCOPE);
    auto& work=*m_g76OrbitProducer;
    const auto& first=input.nominal.geometry[0U];
    work.startPulse.fill(0.0); work.stagedPulse.fill(0.0);
    result.startMCS=first.startMCS; result.translationGeneration=input.translation.generation; result.count=plan.count;
    unsigned enabled=0U;
    for (unsigned a=0U;a<8U;++a) {
        if (std::memcmp(&first.startMCS[a],&commandedMCSTail[a],sizeof(double))!=0)
            return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::START_MCS,static_cast<int>(a));
        work.stagedMCS[a]=commandedMCSTail[a];
        if (a>=m_pContexts->size() || !(*m_pContexts)[a].isExist) continue;
        const auto& axis=(*m_pContexts)[a];
        if (axis.axisIndex!=static_cast<int>(a) || (a<3U && axis.axisType!=AxisType::LINEAR) ||
            !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR<=0.0 || !std::isfinite(axis.finalLead) || axis.finalLead<=0.0)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING,static_cast<int>(a));
        const double pulse=axis.logicalCmdPos.Load(),ppu=axis.resolution_PPR/axis.finalLead;
        if (!std::isfinite(pulse) || !std::isfinite(ppu) || ppu<=0.0 ||
            std::memcmp(&ppu,&input.history.pulsePerUnit[a],sizeof(double))!=0)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_SCALE,static_cast<int>(a));
        // Canonical source pulse is not reconstructed from a rounded MCS.
        // The first command borrows only the same bounded start residue that
        // the existing retained consumer accepts; subsequent seams are exact.
        const double error=std::fabs(pulse-first.startPulse[a]);
        const double scale=(std::max)(1.0,(std::max)(std::fabs(pulse),std::fabs(first.startPulse[a])));
        if (!std::isfinite(error) || error>64.0*std::numeric_limits<double>::epsilon()*scale || error/ppu>1e-7 ||
            (a<3U && !RetainedUnitConsistent(first.startPulse[a],first.startMCS[a],ppu)))
            return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::NATIVE_BASELINE,static_cast<int>(a));
        if (m_pCoordMgr->GetInvalidSoftwareTravelLimitMask(axis)!=0U)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_CONFIG,static_cast<int>(a));
        if (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,first.startMCS[a]))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_START,static_cast<int>(a));
        enabled|=1U<<a; result.pulsePerUnit[a]=ppu; work.startPulse[a]=work.stagedPulse[a]=pulse;
    }
    if ((enabled&7U)!=7U || enabled!=input.history.validAxisMask)
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING);
    result.startPulse=work.startPulse; result.validAxisMask=enabled; receipt.axisMask=enabled;
    if (input.nominal.kind==NCG76Mixed::Kind::G76) {
        // Rebind pure G76 rows to the current sampled native pulse baseline
        // and live axis ratios before permitting any historical interval.
        if (!NCG76Mixed::BuildG76Nominal(input.nominal.orbit,work.startPulse,result.pulsePerUnit,
                input.nominal.pc,input.nominal.line,work.rebuiltNominal) ||
            work.rebuiltNominal.count!=input.nominal.count)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::PLAN_INPUT);
        for (unsigned k=0U;k<input.nominal.count;++k)
            if (!NCG76Mixed::SameGeometry(work.rebuiltNominal.geometry[k],input.nominal.geometry[k]))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
    }
    unsigned movedMask=0U;
    for (unsigned k=0U;k<plan.count;++k) {
        const auto& leg=plan.legs[k]; const auto& g=leg.geometry;
        const bool arc=g.kind==NCPathCoreRetainedKind::ARC,reverse=leg.endU<leg.startU;
        double intervalPulse=0.0,intervalMM=0.0;
        if (!IsNCPathCoreRetainedGeometryValid(g) || g.point || g.kind==NCPathCoreRetainedKind::LINE_ARC ||
            !IsMotionRetainedIntervalBoundsValid(leg.startU,leg.endU,g.lengthPulse,intervalPulse) ||
            !IsMotionRetainedIntervalBoundsValid(leg.startU,leg.endU,g.lengthMM,intervalMM))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
        double acc=0.0,dec=0.0;
        double velocity=(leg.feedPhysicalMmMin/60.0)*(g.lengthPulse/g.lengthMM);
        for (unsigned a=0U;a<8U;++a) {
            double beginPulse=0.0,beginMCS=0.0,endPulse=0.0,endMCS=0.0;
            if (!EvaluateNCPathCoreRetainedPulseCanonical(g,a,leg.startU,beginPulse) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(g,a,leg.startU,beginMCS) ||
                !EvaluateNCPathCoreRetainedPulseCanonical(g,a,leg.endU,endPulse) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(g,a,leg.endU,endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,static_cast<int>(a),static_cast<int>(k));
            if ((k!=0U && beginPulse!=work.stagedPulse[a]) || beginMCS!=work.stagedMCS[a])
                return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::NATIVE_BASELINE,static_cast<int>(a),static_cast<int>(k));
            result.legEndPulse[k][a]=endPulse; result.legEndMCS[k][a]=endMCS;
            if ((enabled&(1U<<a))==0U) continue;
            const auto& axis=(*m_pContexts)[a]; const double ppu=result.pulsePerUnit[a];
            const bool selected=(g.axisMask&(1U<<a))!=0U;
            if (!selected && (beginPulse!=endPulse || beginMCS!=endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,static_cast<int>(a),static_cast<int>(k));
            double low=(std::min)(g.startMCS[a],g.endMCS[a]),high=(std::max)(g.startMCS[a],g.endMCS[a]);
            if (arc && a<2U) { low=g.boundsMinMCS[a]; high=g.boundsMaxMCS[a]; }
            if (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,low) || !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,high) ||
                !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,beginMCS) || !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_ENDPOINT,static_cast<int>(a),static_cast<int>(k));
            if (!selected) continue;
            if (a>=3U || axis.axisType!=AxisType::LINEAR || !RetainedUnitConsistent(g.startPulse[a],g.startMCS[a],ppu) ||
                !RetainedUnitConsistent(g.endPulse[a],g.endMCS[a],ppu) || !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS<=0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time<0.0 || !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time<0.0)
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_DYNAMICS,static_cast<int>(a),static_cast<int>(k));
            const double ratio=arc?1.0:std::fabs((g.endPulse[a]-g.startPulse[a])/g.lengthPulse);
            if (!std::isfinite(ratio) || (!arc && g.endPulse[a]!=g.startPulse[a] && ratio==0.0))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_DYNAMICS,static_cast<int>(a),static_cast<int>(k));
            if (ratio>0.0) velocity=(std::min)(velocity,axis.maxVel_PPS/ratio);
            acc=(std::max)(acc,axis.G00_acc_time); dec=(std::max)(dec,axis.G00_dec_time);
        }
        if (arc && (result.pulsePerUnit[0]!=result.pulsePerUnit[1] ||
            !RetainedUnitConsistent(g.centerPulse[0],g.centerMCS[0],result.pulsePerUnit[0]) ||
            !RetainedUnitConsistent(g.centerPulse[1],g.centerMCS[1],result.pulsePerUnit[1]) ||
            !RetainedUnitConsistent(g.radiusPulse,g.radiusMM,result.pulsePerUnit[0])))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
        if (acc<0.001) acc=0.2;
        if (dec<0.001) dec=0.2;
        if (!std::isfinite(velocity) || velocity<1.0 || !std::isfinite(velocity/acc) || !std::isfinite(velocity/dec))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_DYNAMICS,-1,static_cast<int>(k));
        auto& cmd=work.commands[k]; ClearRetainedCommand(cmd);
        cmd.dir=arc?(reverse?-g.direction:g.direction):0;
        cmd.mode=!arc?InterpolationMode::LINEAR:cmd.dir==1?InterpolationMode::CIRCULAR_CCW:InterpolationMode::CIRCULAR_CW;
        for (unsigned a=0U;a<8U;++a) {
            cmd.mem_startPos[a]=g.startPulse[a]; cmd.mem_ratio[a]=g.endPulse[a];
            if ((g.axisMask&(1U<<a))!=0U) { cmd.axisIndices[cmd.axisCount]=static_cast<int>(a); cmd.targetPos[cmd.axisCount++]=result.legEndPulse[k][a]; }
        }
        cmd.centerPos[0]=cmd.mem_centerX=g.centerPulse[0]; cmd.centerPos[1]=cmd.mem_centerY=g.centerPulse[1];
        cmd.startRadius=cmd.endRadius=cmd.mem_radius=g.radiusPulse;
        cmd.mem_startAngle=g.startAngle; cmd.mem_totalAngle=g.sweepRadians; cmd.mem_totalDist=g.lengthPulse;
        cmd.mem_transformOrigin[0]=leg.startU; cmd.mem_transformOrigin[1]=leg.endU; cmd.mem_transformOrigin[2]=2.0;
        if(input.translation.workMode==168)
        {
            // Disabled transform storage carries immutable ordinary WORK B2
            // provenance; the native interval is already transformed exactly once.
            const double kind=static_cast<double>(static_cast<unsigned>(input.nominal.kind));
            const double ordinal=static_cast<double>(k+1U);
            cmd.mem_transformMatrix[0U][0U]=269.0; cmd.mem_transformMatrix[0U][1U]=1.0;
            cmd.mem_transformMatrix[0U][2U]=kind; cmd.mem_transformMatrix[1U][0U]=static_cast<double>(plan.count);
            cmd.mem_transformMatrix[1U][1U]=ordinal;
            cmd.mem_transformMatrix[1U][2U]=static_cast<double>(static_cast<unsigned>(leg.phase));
            cmd.mem_transformMatrix[2U][0U]=2690000000.0+kind*1000000.0+static_cast<double>(plan.count)*1000.0+ordinal;
            cmd.mem_transformMatrix[2U][1U]=input.nominal.kind==NCG76Mixed::Kind::G76 ?
                static_cast<double>(input.nominal.orbit.modeP) : 0.0;
            cmd.mem_transformMatrix[2U][2U]=static_cast<double>(input.nominal.count);
        }
        cmd.targetVel=velocity; cmd.accTime=acc; cmd.decTime=dec; cmd.commandPathMode=MotionCommandPathMode::EXACT_STOP;
        cmd.pathCorePlanarCircle=arc; cmd.pathCoreFullCircle=arc&&g.fullCircle; cmd.pathCoreRetainedTraversal=true; cmd.pathCoreRetainedReverse=reverse;
        cmd.sourceLinePC=m_pendingSourcePC; cmd.sourceWCS=m_pendingSourceWCS; cmd.sourceTranslation=m_pendingTranslation;
        cmd.sourceToolLengthMode=m_pendingToolMode; cmd.sourceHCode=m_pendingHCode; cmd.sourceToolRadiusMode=m_pendingToolRadMode; cmd.sourceDCode=m_pendingDCode;
        cmd.sourceIsAbsoluteMode=m_pendingIsAbsoluteMode; cmd.sourceG68Active=m_pendingG68Active; cmd.sourceG68Angle=m_pendingG68Angle;
        cmd.sourceG168Active=m_pendingG168Active; cmd.sourceWCode=m_pendingWCode; cmd.sourceG51Active=m_pendingG51Active; cmd.sourceScaleRatio=m_pendingScaleRatio;
        cmd.sourceMirrorMask=m_pendingMirrorMask; cmd.sourceG16Active=m_pendingG16Active; cmd.sourceG162Active=m_pendingG162Active; cmd.sourcePlaneMode=m_pendingPlaneMode;
        work.stagedPulse=result.legEndPulse[k]; work.stagedMCS=result.legEndMCS[k]; movedMask|=g.axisMask;
    }
    result.axisMask=movedMask; result.endPulse=work.stagedPulse; result.endMCS=work.stagedMCS;
    const auto stagedFingerprint=MixedNativeTailFingerprint(work.stagedPulse,work.stagedMCS.data(),rapidOverride);
    if (!m_Group.cmdQueue.ProducerHasCapacity(plan.count))
        return reject(false,true,MotionRejectReason::QUEUE_FULL,MotionG76AdmissionGate::QUEUE_CAPACITY);
    MotionExecutionEpoch published=MOTION_EXECUTION_EPOCH_INVALID;
    if (!TryPublishOwnerAuthorizedAbortingExecutionEpoch(source,epoch,owner,published))
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::EPOCH_COMMIT);
    for (unsigned k=0U;k<plan.count;++k) { AssignExecutionIdentity(work.commands[k],published,source,owner); result.identities[k]=work.commands[k].execution; }
    // Execute the existing complete consumer geometry/source qualification
    // with the genuine assigned identities, before the single queue publish.
    for (unsigned k=0U;k<plan.count;++k)
        if (!IsG76MixedPreparedCommandGeometryValid(work.commands[k]))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::LINE_STAGE,-1,static_cast<int>(k));
    receipt.identity=result.identities[plan.count-1U]; receipt.ownerLease=result.ownerLease=owner;
    if (!TryEnqueueMotionCommandBatch(work.commands.data(),plan.count))
        return reject(false,false,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::QUEUE_PUBLISH);
    result.commandAccepted=receipt.commandAccepted=true;
    const auto stable=[&]() {
        const auto s1=m_motionOwnerState.load(std::memory_order_acquire); const auto e1=GetCurrentExecutionEpoch();
        const auto e2=GetCurrentExecutionEpoch(); const auto s2=m_motionOwnerState.load(std::memory_order_acquire);
        return e1==published && e2==published && s1==s2 && UnpackMotionOwnerState(s1).Matches(owner) &&
            UnpackMotionOwnerState(s2).Matches(owner) && !UnpackMotionOwnerSafetyHandshake(s2) &&
            UnpackMotionOwnerSafetyRequestTicket(s2)==m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto failedAccepted=[&](bool committed,MotionG76AdmissionGate gate) {
        result.rejectionGate=gate; result.rejectionReason=MotionRejectReason::NOT_READY;
        m_g00ProducerQueueTailEpoch=MOTION_EXECUTION_EPOCH_INVALID; m_g00ProducerQueueTailOwnerLease={}; m_g00ProducerQueueTailValidMask=0U;
        receipt.commandedMCSCommitted=receipt.lastQueuedPulseCommitted=receipt.rapidOverrideCommitted=committed;
        receipt.preservedOnReject=receipt.endpointExact=receipt.accountingValid=false;
        receipt.committedFingerprint=committed?stagedFingerprint:receipt.beforeFingerprint;
        PublishQueueTailTransactionReceipt(receipt,false);
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch()); RequestEmergencyStopAllAxes(); return false;
    };
    bool capture=stable() && m_programBlockMotionCaptureActive && !m_programBlockMotionCapture.overflow && m_programBlockMotionCapture.count==plan.count;
    for (unsigned k=0U;capture && k<plan.count;++k) {
        const auto& c=work.commands[k]; const auto& sub=m_programBlockMotionCapture.submissions[k];
        capture=c.execution.IsAssigned() && c.execution.epoch==published && c.execution.source==source &&
            c.execution.sourceBlockId==static_cast<MotionSourceBlockId>(m_pendingSourcePC) && c.ownerLease.Matches(owner) &&
            sub.producerAccepted && sub.immediateRejectReason==MotionRejectReason::NONE && sub.commandPathMode==MotionCommandPathMode::EXACT_STOP &&
            sub.translationGeneration==input.translation.generation && sub.identity.epoch==c.execution.epoch && sub.identity.segmentId==c.execution.segmentId &&
            sub.identity.sourceBlockId==c.execution.sourceBlockId && sub.identity.source==c.execution.source;
        for (unsigned j=0U;capture && j<k;++j) capture=result.identities[j].segmentId!=c.execution.segmentId;
    }
    if (!capture || !IsPendingFixedTranslationSourceAllowed() || !SameNCTranslationSnapshot(input.translation,m_pendingTranslation) || !MatchesNCTranslation(m_pendingTranslation))
        return failedAccepted(false,MotionG76AdmissionGate::CAPTURE_BIND);
    result.captureBound=true; m_g00ProducerQueueTailPulse=work.stagedPulse;
    for (unsigned a=0U;a<8U;++a) { if ((enabled&(1U<<a))!=0U) (*m_pContexts)[a].lastQueuedPulse.Store(work.stagedPulse[a]); commandedMCSTail[a]=work.stagedMCS[a]; }
    m_g00ProducerQueueTailValidMask=enabled; G00_overrideRatio=rapidOverride;
    if (!stable()) return failedAccepted(true,MotionG76AdmissionGate::POST_ACCEPT_TUPLE);
    m_g00ProducerQueueTailEpoch=published; m_g00ProducerQueueTailOwnerLease=owner;
    if (!stable()) return failedAccepted(true,MotionG76AdmissionGate::POST_ACCEPT_TUPLE);
    receipt.commandedMCSCommitted=receipt.lastQueuedPulseCommitted=receipt.rapidOverrideCommitted=true;
    receipt.endpointExact=receipt.accountingValid=true; receipt.preservedOnReject=false; receipt.committedFingerprint=stagedFingerprint;
    result.valid=result.tailCommitted=true; PublishQueueTailTransactionReceipt(receipt,false); return true;
}

// EDM183 FIX2: NC-thread rejection diagnostics only. These observations never
// change admission, precision, coordinates or the producer/RT authority path.
// Keep this out of the cyclic consumer and preserve caller scratch on failure.
namespace
{
#if defined(_MSC_VER)
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
    __attribute__((noinline))
#endif
    void PrintEDM183NominalInputFailure(const EDM173::Plan& plan,
        const NCG76Mixed::Point8& initial,const NCG76Mixed::Point8& ppu,
        const NCG76Mixed::Point8* stationaryRotaryModulo,bool built,unsigned count) noexcept
    {
        const bool planValid=EDM173::Valid(plan);
        unsigned mask=0U;
        for(unsigned a=0U;a<8U;++a)
            if(plan.nominal.translation.axisIdentity.exists[a])mask|=1U<<a;
        const char* cause=planValid?nullptr:"PLAN_VALID";
        const auto unit=[&](unsigned axis,double pulse,double mcs) noexcept {
            return stationaryRotaryModulo && axis>=3U && axis<6U?
                EDM173::StationaryRotaryUnit(pulse,mcs,ppu[axis],(*stationaryRotaryModulo)[axis]):
                NCG76Mixed::Detail::Unit(pulse,mcs,ppu[axis]);
        };
        if(!cause && stationaryRotaryModulo && !EDM173::ValidStationaryRotaryModulo(*stationaryRotaryModulo))
            cause="ROTARY_MODULO";
        int failedAxis=-1,failedChild=-1;
        NCG76Mixed::Geometry previous{},row{};
        NCG76Mixed::Point8 previousPulse=initial;
        if(!cause && !NCG76Mixed::Detail::Calibration(mask,ppu))
        {
            cause="CALIBRATION";
            for(unsigned a=0U;a<8U;++a)
                if(!std::isfinite(ppu[a]) || ((mask&(1U<<a))!=0U?ppu[a]<=0.0:ppu[a]!=0.0))
                {failedAxis=static_cast<int>(a);break;}
        }
        if(!cause)
            for(unsigned a=0U;a<8U;++a)
                if(!std::isfinite(initial[a]) || ((mask&(1U<<a))!=0U &&
                    !unit(a,initial[a],plan.nominal.startMCS[a])))
                {cause="INITIAL_UNIT";failedAxis=static_cast<int>(a);break;}
        if(!cause)
            for(unsigned k=0U;k<EDM173::CountForRecipe(plan.recipe);++k)
            {
                if(!NCG76Mixed::Detail::CanonicalOrbitRow(plan.nominal,k,initial,ppu,previousPulse,row))
                    cause="CANONICAL_ROW";
                else if(!EDM173::StationaryAuxiliaryRowValid(row,plan.nominal.startMCS,initial))
                {
                    for(unsigned a=3U;a<8U;++a)
                        if(std::memcmp(&row.startMCS[a],&plan.nominal.startMCS[a],sizeof(double))!=0 ||
                            std::memcmp(&row.endMCS[a],&plan.nominal.startMCS[a],sizeof(double))!=0 ||
                            std::memcmp(&row.startPulse[a],&initial[a],sizeof(double))!=0 ||
                            std::memcmp(&row.endPulse[a],&initial[a],sizeof(double))!=0)
                        {cause="STATIONARY_AXIS";failedAxis=static_cast<int>(a);break;}
                }
                else if(!NCG76Mixed::Detail::GeometryValid(row,stationaryRotaryModulo?7U:mask,ppu))
                {
                    cause="GEOMETRY_VALID";
                    for(unsigned a=0U;a<8U;++a)
                        if((mask&(1U<<a))!=0U &&
                            (!unit(a,row.startPulse[a],row.startMCS[a]) ||
                                !unit(a,row.endPulse[a],row.endMCS[a])))
                        {failedAxis=static_cast<int>(a);break;}
                }
                if(!cause && k && !AreNCPathCoreRetainedEndpointsConnected(previous,row,mask,ppu))
                    cause="CONTINUITY";
                if(cause){failedChild=static_cast<int>(k);break;}
                previousPulse=row.endPulse;previous=row;
            }
        if(!cause)cause=built?"COUNT":"REBUILD_NOT_REPRODUCED";
        PrintNCG76DiagnosticLine("[EDM183_FIX2] event=PLAN_INPUT_REJECT stage=NOMINAL_REBUILD cause=%s child=%d axis=%d mask=%x rowMask=%x lengthMM=%.17g lengthPulse=%.17g built=%u count=%u planValid=%u session=%llu pc=%d discharge=0\n",
            cause,failedChild,failedAxis,mask,row.axisMask,row.lengthMM,row.lengthPulse,built?1U:0U,count,planValid?1U:0U,
            static_cast<unsigned long long>(plan.scope.session),plan.scope.sourcePC);
        for(unsigned a=0U;a<6U;++a)
        {
            const double converted=std::isfinite(ppu[a])&&ppu[a]>0.0?initial[a]/ppu[a]:
                (std::numeric_limits<double>::quiet_NaN)();
            const double mcs=plan.nominal.startMCS[a];
            const double scale=(std::max)(1.0,(std::max)(std::fabs(mcs),std::fabs(converted)));
            if(failedChild<0)
                PrintNCG76DiagnosticLine("[EDM183_FIX2] event=PLAN_INPUT_AXIS axis=%u exists=%u pulse=%.17g mcs=%.17g ppu=%.17g converted=%.17g error=%.17g epsilonLimit=%.17g absoluteLimit=1e-7 modulo=%.17g unit=%u discharge=0\n",
                    a,(mask&(1U<<a))!=0U?1U:0U,initial[a],mcs,ppu[a],converted,
                    std::fabs(converted-mcs),64.0*std::numeric_limits<double>::epsilon()*scale,
                    stationaryRotaryModulo?(*stationaryRotaryModulo)[a]:0.0,unit(a,initial[a],mcs)?1U:0U);
            else
                PrintNCG76DiagnosticLine("[EDM183_FIX2] event=PLAN_INPUT_AXIS child=%d axis=%u pulse=%.17g mcs=%.17g ppu=%.17g startMCS=%.17g endMCS=%.17g startPulse=%.17g endPulse=%.17g previousEndMCS=%.17g previousEndPulse=%.17g discharge=0\n",
                    failedChild,a,initial[a],mcs,ppu[a],row.startMCS[a],row.endMCS[a],
                    row.startPulse[a],row.endPulse[a],previous.endMCS[a],previous.endPulse[a]);
        }
    }
}

// EDM173 explicit sealed common-GAP admission. Ordinary AIR171 stays above.
bool MotionCore::TryG76GapMoveTransactionalTail(const EDM173::Plan& plan,
    double* commandedMCSTail,MotionG76OrbitReceipt& result,
    const NCG76Mixed::Point8* stationaryRotaryModulo)
{
    result.Clear();
    auto& receipt=result.tail;
    const auto epoch=GetCurrentExecutionEpoch();
    const auto owner=GetMotionOwnerLease();
    const auto source=m_pendingCommandSource.load(std::memory_order_acquire);
    const double rapidOverride=G00_overrideRatio;
    receipt.transactionSequence=AllocateQueueTailTransactionSequence(); receipt.attempted=true;
    receipt.beforeFingerprint=MixedNativeTailFingerprint(m_g00ProducerQueueTailPulse,commandedMCSTail,rapidOverride);
    receipt.committedFingerprint=receipt.beforeFingerprint;
    const auto reject=[&](bool invalid,bool formal,MotionRejectReason reason,MotionG76AdmissionGate gate,
        int axis=-1,int segment=-1)->bool {
        result.rejectionGate=gate; result.rejectionAxis=axis; result.rejectionSegment=segment; result.rejectionReason=reason;
        if (formal) {
            MotionCommand c{}; c.mode=InterpolationMode::LINEAR; c.sourceLinePC=m_pendingSourcePC; c.commandPathMode=MotionCommandPathMode::EXACT_STOP;
            if (invalid) RejectInvalidProducerMotionCommand(c,GetCurrentExecutionEpoch(),source,GetMotionOwnerLease(),&receipt.identity,&receipt.ownerLease);
            else RejectNonGeometryProducerMotionCommand(c,GetCurrentExecutionEpoch(),source,GetMotionOwnerLease(),reason,&receipt.identity,&receipt.ownerLease);
        }
        receipt.committedFingerprint=MixedNativeTailFingerprint(m_g00ProducerQueueTailPulse,commandedMCSTail,G00_overrideRatio);
        receipt.preservedOnReject=receipt.committedFingerprint==receipt.beforeFingerprint;
        receipt.accountingValid=receipt.preservedOnReject;
        PublishQueueTailTransactionReceipt(receipt,invalid); return false;
    };
    const bool planValid=EDM173::Valid(plan);
    if (!planValid || !m_edm173 || !m_g76OrbitProducer || !m_pContexts || !m_pCoordMgr ||
        !commandedMCSTail || m_pContexts->size()!=6U ||
        !std::isfinite(rapidOverride) || rapidOverride<0.0)
    {
        const bool contexts=m_edm173 && m_g76OrbitProducer && m_pContexts && m_pCoordMgr &&
            commandedMCSTail && m_pContexts->size()==6U;
        const char* cause=!planValid?"PLAN_VALID":!contexts?"CONTEXT":"OVERRIDE";
        PrintNCG76DiagnosticLine("[EDM183_FIX2] event=PLAN_INPUT_REJECT stage=ENTRY cause=%s planValid=%u edm173=%u workspace=%u axes=%u coord=%u tail=%u axisCount=%u override=%.17g session=%llu pc=%d discharge=0\n",
            cause,planValid?1U:0U,m_edm173?1U:0U,m_g76OrbitProducer?1U:0U,m_pContexts?1U:0U,
            m_pCoordMgr?1U:0U,commandedMCSTail?1U:0U,m_pContexts?static_cast<unsigned>(m_pContexts->size()):0U,
            rapidOverride,static_cast<unsigned long long>(plan.scope.session),plan.scope.sourcePC);
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::PLAN_INPUT);
    }
    if(stationaryRotaryModulo && !EDM173::ValidStationaryRotaryModulo(*stationaryRotaryModulo))
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING);
    const auto& native=plan.nominal;
    EDM128::Point centerAxisNative{};
    if(EDM173::IsCenterRecipe(plan.recipe)&&!EDM173::MachiningAxisNative(plan,centerAxisNative))
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::PLAN_INPUT);
    EDM94::Feedback guard{};EDM126::MotionFeedback common{};EDM128::MotionFeedback mode{};
    if(!ReadEDM94Feedback(guard) || !ReadEDM126Feedback(common) || !ReadEDM128Feedback(mode) ||
        !guard.active || !guard.armedReady || guard.forceZero || guard.faulted || guard.cancelled || guard.handoffZero ||
        guard.boundaryReadyCycles<EDM94::BoundaryReadyCycles || guard.boundaryNotReadyMask ||
        guard.currentIdentity.session!=plan.scope.session || guard.currentIdentity.epoch!=epoch ||
        guard.currentIdentity.ownerGeneration!=owner.generation || guard.config.commissioningProfile!=plan.recipe.commissioningProfile ||
        guard.config.visibleZFlush158!=(!EDM173::IsB2Recipe(plan.recipe)&&!EDM173::IsB0Recipe(plan.recipe)) ||
        (EDM173::UsesNominalMovementMask(plan.recipe)&&(!guard.boundedProgram||
            guard.requiredMovementMask!=EDM130::ProgramMovementMask(plan.recipe))) ||
        !common.enabled || !common.active || common.faulted || !common.sourceReady || common.fixture!=14U ||
        common.session!=plan.scope.session || common.armEpoch!=epoch || common.liveEpoch!=epoch ||
        common.authorizedRows || common.completedRows || common.rowActive || !mode.enabled ||
        !EDM130::SameRecipe(mode.timerRecipe,plan.recipe))
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::RUNTIME_READY);
    if (!epoch || !owner.IsValid() || owner.owner!=MotionOwner::AUTO || source!=MotionCommandSource::NC_MEMORY ||
        !owner.Matches(plan.scope.owner) || !IsMotionOwnerLeaseCurrent(owner) ||
        m_pendingSourcePC<0 || plan.scope.sourcePC!=m_pendingSourcePC || plan.scope.armEpoch!=epoch ||
        plan.scope.run!=native.translation.runToken || plan.scope.session!=guard.identity.session)
        return reject(false,true,MotionRejectReason::OWNER_CONFLICT,MotionG76AdmissionGate::OWNER_SCOPE);
    if (!IsPendingFixedTranslationSourceAllowed() || !EDM173::FrameSupported(m_pendingTranslation) ||
        !SameNCTranslationSnapshot(native.translation,m_pendingTranslation) ||
        native.translationGeneration!=m_pendingTranslation.generation ||
        !MatchesNCTranslation(m_pendingTranslation) || m_pendingPlaneMode!=17 || m_pendingToolRadMode!=40 ||
        m_pendingG16Active || m_pendingG162Active)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::SOURCE_FRAME);
    const auto hold=GetFeedHoldStopSnapshot();
    if (HasPendingSafetyOrRecoveryRequests() || m_edm94Claim.load(std::memory_order_acquire)!=plan.scope.session ||
        !IsGroupDone() || GetQueueSize()!=0U || GetCommandIngressSize()!=0U || GetCommandReplaySize()!=0U ||
        !HasExactExecutionDrainAcknowledgement(epoch,owner) || !hold.groupDone || hold.groupActive || !hold.commandStopped ||
        hold.groupFaulted || hold.groupEmergencyStopped || hold.safetyOrRecoveryPending || hold.faultedAxes ||
        hold.overrideZero || !std::isfinite(hold.feedrateOverride) || hold.feedrateOverride!=1.0)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::RUNTIME_READY);
    if (!m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow || m_programBlockMotionCapture.count!=0U)
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::CAPTURE_SCOPE);
    auto& work=*m_g76OrbitProducer;
    const auto& first=native.startMCS;
    work.startPulse.fill(0.0); work.stagedPulse.fill(0.0);
    result.startMCS=first; result.translationGeneration=native.translation.generation; result.count=native.count;
    unsigned enabled=0U;
    for (unsigned a=0U;a<8U;++a) {
        if (std::memcmp(&first[a],&commandedMCSTail[a],sizeof(double))!=0)
            return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::START_MCS,static_cast<int>(a));
        work.stagedMCS[a]=commandedMCSTail[a];
        if (a>=m_pContexts->size() || !(*m_pContexts)[a].isExist) continue;
        const auto& axis=(*m_pContexts)[a];
        if (axis.axisIndex!=static_cast<int>(a) || (a<3U && axis.axisType!=AxisType::LINEAR) ||
            !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR<=0.0 || !std::isfinite(axis.finalLead) || axis.finalLead<=0.0)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING,static_cast<int>(a));
        if(stationaryRotaryModulo && a>=3U) {
            if(axis.axisType!=AxisType::ROTARY || !std::isfinite(axis.rotaryModulo) || axis.rotaryModulo<=0.0)
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING,static_cast<int>(a));
            if(std::memcmp(&axis.rotaryModulo,&(*stationaryRotaryModulo)[a],sizeof(double))!=0)
                return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::SOURCE_FRAME,static_cast<int>(a));
        }
        const double pulse=axis.logicalCmdPos.Load(),ppu=axis.resolution_PPR/axis.finalLead;
        if (!std::isfinite(pulse) || !std::isfinite(ppu) || ppu<=0.0 ||
            ppu!=guard.config.axis[a].pulsePerUnit)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_SCALE,static_cast<int>(a));
        // Canonical source pulse is not reconstructed from a rounded MCS.
        // The first command borrows only the same bounded start residue that
        // the existing retained consumer accepts; subsequent seams are exact.
        if(a<3U && !RetainedUnitConsistent(pulse,first[a],ppu))
            return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::NATIVE_BASELINE,static_cast<int>(a));
        if (m_pCoordMgr->GetInvalidSoftwareTravelLimitMask(axis)!=0U)
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_CONFIG,static_cast<int>(a));
        if (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,first[a]))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_START,static_cast<int>(a));
        enabled|=1U<<a; result.pulsePerUnit[a]=ppu; work.startPulse[a]=work.stagedPulse[a]=pulse;
    }
    if (enabled!=63U || m_pContexts->size()!=6U)
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_MAPPING);
    result.startPulse=work.startPulse; result.validAxisMask=enabled; receipt.axisMask=enabled;
    const bool nominalBuilt=EDM173::BuildNominal(plan,work.startPulse,result.pulsePerUnit,work.rebuiltNominal,stationaryRotaryModulo);
    if(!nominalBuilt || work.rebuiltNominal.count!=EDM173::CountForRecipe(plan.recipe))
    {
        PrintEDM183NominalInputFailure(plan,work.startPulse,result.pulsePerUnit,stationaryRotaryModulo,nominalBuilt,work.rebuiltNominal.count);
        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::PLAN_INPUT);
    }
    unsigned movedMask=0U;
    for (unsigned k=0U;k<native.count;++k) {
        NCG76Mixed::Leg leg{};leg.geometry=work.rebuiltNominal.geometry[k];leg.startU=0.0;leg.endU=1.0;leg.feedPhysicalMmMin=0.12;
        const auto& g=leg.geometry;
        NCG76::Point8 centerStartMCS{},centerEndMCS{};
        if(EDM173::IsCenterRecipe(plan.recipe) &&
            (!EDM173::CenterBoundary(plan,k,centerStartMCS)||!EDM173::CenterBoundary(plan,k+1U,centerEndMCS)))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
        const bool arc=g.kind==NCPathCoreRetainedKind::ARC,reverse=leg.endU<leg.startU;
        double intervalPulse=0.0,intervalMM=0.0;
        if (!IsNCPathCoreRetainedGeometryValid(g) || g.point || g.kind==NCPathCoreRetainedKind::LINE_ARC ||
            !IsMotionRetainedIntervalBoundsValid(leg.startU,leg.endU,g.lengthPulse,intervalPulse) ||
            !IsMotionRetainedIntervalBoundsValid(leg.startU,leg.endU,g.lengthMM,intervalMM))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
        NCG76::Point8 tangentShift{};
        if(EDM173::IsB0Recipe(plan.recipe)) {
            // The nominal child already includes the compensation stack.
            // Check its entire possible stopped-anchor interval after a
            // native 2 mm negative chord-tangent translation, on every axis.
            if(arc||g.kind!=NCPathCoreRetainedKind::LINE)return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
            double norm=0.0;
            for(unsigned a=0U;a<6U;++a) {
                tangentShift[a]=(g.endPulse[a]-g.startPulse[a])/g.lengthMM;
                if(a<3U)norm=std::hypot(norm,tangentShift[a]/result.pulsePerUnit[a]);
                else if(tangentShift[a]!=0.0)return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,static_cast<int>(a),static_cast<int>(k));
            }
            if(!std::isfinite(norm)||norm<=0.0)return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
            for(unsigned a=0U;a<6U;++a)tangentShift[a]=-2.0*(tangentShift[a]/norm)/result.pulsePerUnit[a];
        }
        double acc=0.0,dec=0.0;
        double velocity=(leg.feedPhysicalMmMin/60.0)*(g.lengthPulse/g.lengthMM);
        for (unsigned a=0U;a<8U;++a) {
            double beginPulse=0.0,beginMCS=0.0,endPulse=0.0,endMCS=0.0;
            if (!EvaluateNCPathCoreRetainedPulseCanonical(g,a,leg.startU,beginPulse) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(g,a,leg.startU,beginMCS) ||
                !EvaluateNCPathCoreRetainedPulseCanonical(g,a,leg.endU,endPulse) ||
                !EvaluateNCPathCoreRetainedAxisCanonical(g,a,leg.endU,endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,static_cast<int>(a),static_cast<int>(k));
            if ((k!=0U && beginPulse!=work.stagedPulse[a]) || beginMCS!=work.stagedMCS[a])
                return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::NATIVE_BASELINE,static_cast<int>(a),static_cast<int>(k));
            result.legEndPulse[k][a]=endPulse; result.legEndMCS[k][a]=endMCS;
            if ((enabled&(1U<<a))==0U) continue;
            const auto& axis=(*m_pContexts)[a]; const double ppu=result.pulsePerUnit[a];
            const bool selected=(g.axisMask&(1U<<a))!=0U;
            if (!selected && (beginPulse!=endPulse || beginMCS!=endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,static_cast<int>(a),static_cast<int>(k));
            double low=(std::min)(g.startMCS[a],g.endMCS[a]),high=(std::max)(g.startMCS[a],g.endMCS[a]);
            if (arc && a<2U) { low=g.boundsMinMCS[a]; high=g.boundsMaxMCS[a]; }
            // Both endpoint centers and their signed axial tops bound the
            // complete center line and withdrawal route, including B4's join.
            // Use the same pulse conversion as the retained runtime geometry.
            if(EDM173::IsCenterRecipe(plan.recipe)) {
                for(unsigned endpoint=0U;endpoint<2U;++endpoint) {
                    const auto& center=endpoint==0U?centerStartMCS:centerEndMCS;
                    const double cp=a>=3U||center[a]==native.startMCS[a]?work.startPulse[a]:center[a]*ppu;
                    const double tp=centerAxisNative[a]==0.0?cp:
                        cp-EDM173::FrozenHeightMm(plan.recipe)*(centerAxisNative[a]*ppu);
                    if(!std::isfinite(cp)||!std::isfinite(tp)||
                        !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,cp/ppu)||
                        !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,tp/ppu))
                        return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_ENDPOINT,static_cast<int>(a),static_cast<int>(k));
                }
            }
            if(EDM173::IsB0Recipe(plan.recipe) &&
                (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,low+tangentShift[a])||
                 !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,high+tangentShift[a])))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_ENDPOINT,static_cast<int>(a),static_cast<int>(k));
            if(a==2U && !EDM173::IsB2Recipe(plan.recipe) && !EDM173::IsB0Recipe(plan.recipe) && !EDM173::IsCenterRecipe(plan.recipe) &&
                !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,high+EDM173::FrozenHeightMm(plan.recipe)))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_ENDPOINT,2,static_cast<int>(k));
            if (!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,low) || !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,high) ||
                !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,beginMCS) || !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,endMCS))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::TRAVEL_ENDPOINT,static_cast<int>(a),static_cast<int>(k));
            if (!selected) continue;
            if (a>=3U || axis.axisType!=AxisType::LINEAR || !RetainedUnitConsistent(g.startPulse[a],g.startMCS[a],ppu) ||
                !RetainedUnitConsistent(g.endPulse[a],g.endMCS[a],ppu) || !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS<=0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time<0.0 || !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time<0.0)
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::AXIS_DYNAMICS,static_cast<int>(a),static_cast<int>(k));
            const double ratio=arc?1.0:std::fabs((g.endPulse[a]-g.startPulse[a])/g.lengthPulse);
            if (!std::isfinite(ratio) || (!arc && g.endPulse[a]!=g.startPulse[a] && ratio==0.0))
                return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_DYNAMICS,static_cast<int>(a),static_cast<int>(k));
            if (ratio>0.0) velocity=(std::min)(velocity,axis.maxVel_PPS/ratio);
            acc=(std::max)(acc,axis.G00_acc_time); dec=(std::max)(dec,axis.G00_dec_time);
        }
        if (arc && (result.pulsePerUnit[0]!=result.pulsePerUnit[1] ||
            !RetainedUnitConsistent(g.centerPulse[0],g.centerMCS[0],result.pulsePerUnit[0]) ||
            !RetainedUnitConsistent(g.centerPulse[1],g.centerMCS[1],result.pulsePerUnit[1]) ||
            !RetainedUnitConsistent(g.radiusPulse,g.radiusMM,result.pulsePerUnit[0])))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_CONVERSION,-1,static_cast<int>(k));
        if (acc<0.001) acc=0.2;
        if (dec<0.001) dec=0.2;
        if (!std::isfinite(velocity) || velocity<1.0 || !std::isfinite(velocity/acc) || !std::isfinite(velocity/dec))
            return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::POINT_DYNAMICS,-1,static_cast<int>(k));
        auto& cmd=work.commands[k]; ClearRetainedCommand(cmd);
        cmd.edm173G76Gap=true;cmd.edm173ChildOrdinal=static_cast<std::uint8_t>(k);cmd.edm173ChildCount=static_cast<std::uint8_t>(EDM173::CountForRecipe(plan.recipe));
        cmd.dir=arc?(reverse?-g.direction:g.direction):0;
        cmd.mode=!arc?InterpolationMode::LINEAR:cmd.dir==1?InterpolationMode::CIRCULAR_CCW:InterpolationMode::CIRCULAR_CW;
        for (unsigned a=0U;a<8U;++a) {
            cmd.mem_startPos[a]=g.startPulse[a]; cmd.mem_ratio[a]=g.endPulse[a];
            if ((g.axisMask&(1U<<a))!=0U) { cmd.axisIndices[cmd.axisCount]=static_cast<int>(a); cmd.targetPos[cmd.axisCount++]=result.legEndPulse[k][a]; }
        }
        cmd.centerPos[0]=cmd.mem_centerX=g.centerPulse[0]; cmd.centerPos[1]=cmd.mem_centerY=g.centerPulse[1];
        cmd.startRadius=cmd.endRadius=cmd.mem_radius=g.radiusPulse;
        cmd.mem_startAngle=g.startAngle; cmd.mem_totalAngle=g.sweepRadians; cmd.mem_totalDist=g.lengthPulse;
        cmd.mem_transformOrigin[0]=leg.startU; cmd.mem_transformOrigin[1]=leg.endU; cmd.mem_transformOrigin[2]=2.0;
        cmd.targetVel=velocity; cmd.accTime=acc; cmd.decTime=dec; cmd.commandPathMode=MotionCommandPathMode::EXACT_STOP;
        cmd.pathCorePlanarCircle=arc; cmd.pathCoreFullCircle=arc&&g.fullCircle; cmd.pathCoreRetainedTraversal=true; cmd.pathCoreRetainedReverse=reverse;
        cmd.sourceLinePC=m_pendingSourcePC; cmd.sourceWCS=m_pendingSourceWCS; cmd.sourceTranslation=m_pendingTranslation;
        cmd.sourceToolLengthMode=m_pendingToolMode; cmd.sourceHCode=m_pendingHCode; cmd.sourceToolRadiusMode=m_pendingToolRadMode; cmd.sourceDCode=m_pendingDCode;
        cmd.sourceIsAbsoluteMode=m_pendingIsAbsoluteMode; cmd.sourceG68Active=m_pendingG68Active; cmd.sourceG68Angle=m_pendingG68Angle;
        cmd.sourceG168Active=m_pendingG168Active; cmd.sourceWCode=m_pendingWCode; cmd.sourceG51Active=m_pendingG51Active; cmd.sourceScaleRatio=m_pendingScaleRatio;
        cmd.sourceMirrorMask=m_pendingMirrorMask; cmd.sourceG16Active=m_pendingG16Active; cmd.sourceG162Active=m_pendingG162Active; cmd.sourcePlaneMode=m_pendingPlaneMode;
        work.stagedPulse=result.legEndPulse[k]; work.stagedMCS=result.legEndMCS[k]; movedMask|=g.axisMask;
    }
    result.axisMask=movedMask; result.endPulse=work.stagedPulse; result.endMCS=work.stagedMCS;
    const auto stagedFingerprint=MixedNativeTailFingerprint(work.stagedPulse,work.stagedMCS.data(),rapidOverride);
    if (!m_Group.cmdQueue.ProducerHasCapacity(native.count))
        return reject(false,true,MotionRejectReason::QUEUE_FULL,MotionG76AdmissionGate::QUEUE_CAPACITY);
    const MotionExecutionEpoch expected=epoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:epoch+1U;
    for(unsigned k=0U;k<native.count;++k){AssignExecutionIdentity(work.commands[k],expected,source,owner);result.identities[k]=work.commands[k].execution;}
    auto& seal=m_edm173->producer;seal=MotionEDM173Seal{};seal.plan=plan;seal.epoch=expected;seal.startPulse=work.startPulse;seal.pulsePerUnit=result.pulsePerUnit;
    for(unsigned k=0U;k<native.count;++k){seal.identities[k]=result.identities[k];seal.endPulse[k]=result.legEndPulse[k];}
    auto previous=m_edm173->publishedSession.load(std::memory_order_acquire);
    if(previous>=plan.scope.session || !m_edm173->publishedSession.compare_exchange_strong(previous,plan.scope.session,std::memory_order_acq_rel) ||
        !m_edm173->messages.ProducerTryPush(seal))
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::QUEUE_PUBLISH);
    MotionExecutionEpoch published=MOTION_EXECUTION_EPOCH_INVALID;
    if(!TryPublishOwnerAuthorizedAbortingExecutionEpoch(source,epoch,owner,published) || published!=expected){
        RequestEDM94Cancel(plan.scope.session);
        return reject(false,true,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::EPOCH_COMMIT);
    }
    // Execute the existing complete consumer geometry/source qualification
    // with the genuine assigned identities, before the single queue publish.
    for (unsigned k=0U;k<native.count;++k)
        if (!IsG76MixedPreparedCommandGeometryValid(work.commands[k]))
            {RequestEDM94Cancel(plan.scope.session);return reject(true,true,MotionRejectReason::INVALID_GEOMETRY,MotionG76AdmissionGate::LINE_STAGE,-1,static_cast<int>(k));}
    receipt.identity=result.identities[native.count-1U]; receipt.ownerLease=result.ownerLease=owner;
    if (!TryEnqueueMotionCommandBatch(work.commands.data(),native.count)){
        RequestEDM94Cancel(plan.scope.session);return reject(false,false,MotionRejectReason::NOT_READY,MotionG76AdmissionGate::QUEUE_PUBLISH);}
    result.commandAccepted=receipt.commandAccepted=true;
    const auto stable=[&]() {
        const auto s1=m_motionOwnerState.load(std::memory_order_acquire); const auto e1=GetCurrentExecutionEpoch();
        const auto e2=GetCurrentExecutionEpoch(); const auto s2=m_motionOwnerState.load(std::memory_order_acquire);
        return e1==published && e2==published && s1==s2 && UnpackMotionOwnerState(s1).Matches(owner) &&
            UnpackMotionOwnerState(s2).Matches(owner) && !UnpackMotionOwnerSafetyHandshake(s2) &&
            UnpackMotionOwnerSafetyRequestTicket(s2)==m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto failedAccepted=[&](bool committed,MotionG76AdmissionGate gate) {
        result.rejectionGate=gate; result.rejectionReason=MotionRejectReason::NOT_READY;
        m_g00ProducerQueueTailEpoch=MOTION_EXECUTION_EPOCH_INVALID; m_g00ProducerQueueTailOwnerLease={}; m_g00ProducerQueueTailValidMask=0U;
        receipt.commandedMCSCommitted=receipt.lastQueuedPulseCommitted=receipt.rapidOverrideCommitted=committed;
        receipt.preservedOnReject=receipt.endpointExact=receipt.accountingValid=false;
        receipt.committedFingerprint=committed?stagedFingerprint:receipt.beforeFingerprint;
        PublishQueueTailTransactionReceipt(receipt,false);
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch()); RequestEmergencyStopAllAxes(); return false;
    };
    bool capture=stable() && m_programBlockMotionCaptureActive && !m_programBlockMotionCapture.overflow && m_programBlockMotionCapture.count==native.count;
    for (unsigned k=0U;capture && k<native.count;++k) {
        const auto& c=work.commands[k]; const auto& sub=m_programBlockMotionCapture.submissions[k];
        capture=c.execution.IsAssigned() && c.execution.epoch==published && c.execution.source==source &&
            c.execution.sourceBlockId==static_cast<MotionSourceBlockId>(m_pendingSourcePC) && c.ownerLease.Matches(owner) &&
            sub.producerAccepted && sub.immediateRejectReason==MotionRejectReason::NONE && sub.commandPathMode==MotionCommandPathMode::EXACT_STOP &&
            sub.translationGeneration==native.translation.generation && sub.identity.epoch==c.execution.epoch && sub.identity.segmentId==c.execution.segmentId &&
            sub.identity.sourceBlockId==c.execution.sourceBlockId && sub.identity.source==c.execution.source;
        for (unsigned j=0U;capture && j<k;++j) capture=result.identities[j].segmentId!=c.execution.segmentId;
    }
    if (!capture || !IsPendingFixedTranslationSourceAllowed() || !SameNCTranslationSnapshot(native.translation,m_pendingTranslation) || !MatchesNCTranslation(m_pendingTranslation))
        return failedAccepted(false,MotionG76AdmissionGate::CAPTURE_BIND);
    result.captureBound=true; m_g00ProducerQueueTailPulse=work.stagedPulse;
    for (unsigned a=0U;a<8U;++a) { if ((enabled&(1U<<a))!=0U) (*m_pContexts)[a].lastQueuedPulse.Store(work.stagedPulse[a]); commandedMCSTail[a]=work.stagedMCS[a]; }
    m_g00ProducerQueueTailValidMask=enabled; G00_overrideRatio=rapidOverride;
    if (!stable()) return failedAccepted(true,MotionG76AdmissionGate::POST_ACCEPT_TUPLE);
    m_g00ProducerQueueTailEpoch=published; m_g00ProducerQueueTailOwnerLease=owner;
    if (!stable()) return failedAccepted(true,MotionG76AdmissionGate::POST_ACCEPT_TUPLE);
    receipt.commandedMCSCommitted=receipt.lastQueuedPulseCommitted=receipt.rapidOverrideCommitted=true;
    receipt.endpointExact=receipt.accountingValid=true; receipt.preservedOnReject=false; receipt.committedFingerprint=stagedFingerprint;
    result.valid=result.tailCommitted=true; PublishQueueTailTransactionReceipt(receipt,false); return true;
}
