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
        bool FeedLineNormalOverride(const MotionCore& motion) noexcept
    {
        // Existing coherent RT publication; never read raw m_Group state here.
        // M00/Feed Hold resumes to 1.0 before NC admits the next G01.
        const MotionFeedHoldStopSnapshot snapshot =
            motion.GetFeedHoldStopSnapshot();
        // A failed publication read returns groupDone=false. Override=1 alone
        // is a default value and cannot authorize a new transaction.
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
        MotionFeedLineWorkspace& workspace) noexcept
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
            const double logicalPulse = axis.logicalCmdPos.Load();
            if (!std::isfinite(logicalPulse) ||
                !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
                !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
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
            if ((validMask & bit) == 0U ||
                (workspace.input.axisMask & bit) != 0U ||
                axis.axisType != AxisType::LINEAR || !std::isfinite(targetMCS[index]) ||
                !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
                !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time < 0.0 ||
                !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time < 0.0)
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            const double pulsePerMM = axis.resolution_PPR / axis.finalLead;
            const double targetPulse = targetMCS[index] * pulsePerMM;
            if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0 ||
                !std::isfinite(targetPulse))
            {
                result.code = MotionFeedLineCode::INVALID_INPUT;
                return false;
            }
            workspace.input.axisMask |= bit;
            workspace.input.endMCS[slot] = targetMCS[index];
            workspace.input.endPulse[slot] = targetPulse;
            workspace.input.maxVelocityPPS[slot] = axis.maxVel_PPS;
            workspace.stagedMCS[slot] = targetMCS[index];
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
    if (m_pContexts == nullptr || m_pContexts->size() > 8U ||
        axes.empty() || axes.size() > 3U ||
        axes.size() != targetMCS.size() ||
        !std::isfinite(feedMMMin) || feedMMMin <= 0.0 || feedMMMin > 100.0 ||
        workspace.targetPulse.capacity() < 8U)
    {
        result.code = MotionFeedLineCode::INVALID_INPUT;
        return false;
    }
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY ||
        !m_programBlockMotionCaptureActive ||
        m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U ||
        HasPendingSafetyOrRecoveryRequests() ||
        !FeedLineNormalOverride(*this))
    {
        result.code = MotionFeedLineCode::NOT_READY;
        return false;
    }

    if (!PrepareFeedLineGeometry(m_pContexts, axes, targetMCS,
        commandedMCSTail, workspace))
    {
        return false;
    }

    const bool accepted = TryLineMove(
        axes, workspace.targetPulse, result.line.velocityPPS,
        workspace.accTime, workspace.decTime, BufferMode::ABORTING,
        MotionCommandPathMode::EXACT_STOP,
        &result.identity, &result.ownerLease, plannedEpoch, &plannedOwner);
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
    result.captureBound = submission.producerAccepted &&
        submission.immediateRejectReason == MotionRejectReason::NONE &&
        submission.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
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
