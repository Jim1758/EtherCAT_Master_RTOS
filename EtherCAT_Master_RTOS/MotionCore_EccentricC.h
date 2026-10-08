#pragma once

// BASE79F implementation fragment; include only at the end of MotionCore.cpp.
#include <new>
#include <cstring>
#if defined(_MSC_VER)
#define BASE79F_CONSUMER_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79F_CONSUMER_NOINLINE __attribute__((noinline))
#else
#define BASE79F_CONSUMER_NOINLINE
#endif

namespace MotionEccentricCConsumerDetail
{
// Exact primitive comparison, not a hash and never structure padding.
inline bool SamePacket(const MotionCommand& a, const MotionCommand& b) noexcept
{
#define BASE79F_SAME(field) if (std::memcmp(&a.field, &b.field, sizeof(a.field)) != 0) return false
    BASE79F_SAME(execution.epoch);
    BASE79F_SAME(execution.segmentId);
    BASE79F_SAME(execution.sourceBlockId);
    BASE79F_SAME(execution.source);
    BASE79F_SAME(ownerLease.owner);
    BASE79F_SAME(ownerLease.generation);
    BASE79F_SAME(mode);
    BASE79F_SAME(axisCount);
    BASE79F_SAME(axisIndices);
    BASE79F_SAME(targetPos);
    BASE79F_SAME(centerPos);
    BASE79F_SAME(startRadius);
    BASE79F_SAME(endRadius);
    BASE79F_SAME(dir);
    BASE79F_SAME(pathCorePlanarCircle);
    BASE79F_SAME(pathCoreFullCircle);
    BASE79F_SAME(pathCoreRetainedTraversal);
    BASE79F_SAME(pathCoreRetainedReverse);
    BASE79F_SAME(targetVel);
    BASE79F_SAME(accTime);
    BASE79F_SAME(decTime);
    BASE79F_SAME(replayTerminalAlreadyPublished);
    BASE79F_SAME(cncFeedLookahead);
    BASE79F_SAME(cncCornerBlend);
    BASE79F_SAME(pathCoreFeedExactStop);
    BASE79F_SAME(pathCoreRotaryFeedExactStop);
    BASE79F_SAME(pathCoreZCFeedExactStop);
    BASE79F_SAME(pathCoreXYZCFeedExactStop);
    BASE79F_SAME(pathCoreXYZCUVFeedExactStop);
    BASE79F_SAME(pathCoreEccentricCFeedExactStop);
    BASE79F_SAME(mem_startPos);
    BASE79F_SAME(mem_ratio);
    BASE79F_SAME(mem_radius);
    BASE79F_SAME(mem_startAngle);
    BASE79F_SAME(mem_centerX);
    BASE79F_SAME(mem_centerY);
    BASE79F_SAME(mem_totalDist);
    BASE79F_SAME(mem_totalAngle);
    BASE79F_SAME(mem_enableTransform);
    BASE79F_SAME(mem_transformOrigin);
    BASE79F_SAME(mem_transformMatrix);
    BASE79F_SAME(sourceLinePC);
    BASE79F_SAME(sourceWCS);
    BASE79F_SAME(sourceToolLengthMode);
    BASE79F_SAME(sourceHCode);
    BASE79F_SAME(sourceToolRadiusMode);
    BASE79F_SAME(sourceDCode);
    BASE79F_SAME(sourceIsAbsoluteMode);
    BASE79F_SAME(sourceG68Active);
    BASE79F_SAME(sourceG68Angle);
    BASE79F_SAME(sourceG168Active);
    BASE79F_SAME(sourceWCode);
    BASE79F_SAME(sourceG51Active);
    BASE79F_SAME(sourceScaleRatio);
    BASE79F_SAME(sourceMirrorMask);
    BASE79F_SAME(sourceG16Active);
    BASE79F_SAME(sourceG162Active);
    BASE79F_SAME(commandPathMode);
    BASE79F_SAME(sourcePlaneMode);
    BASE79F_SAME(cncPrefixVelocityPPS);
#undef BASE79F_SAME
    return SameNCTranslationSnapshot(a.sourceTranslation, b.sourceTranslation);
}

inline bool PreviousImage(const InterpolationGroup& group,
    const std::vector<AxisContext>* axes, const MotionEccentricCExecutor& executor) noexcept
{
    if (!executor.IsValid() || !group.isActive || axes == nullptr || axes->empty() ||
        axes->size() > 8U || group.axisCount != group.currentCmd.axisCount ||
        group.axisCount <= 0 || group.axisCount > MAX_AXES ||
        !MotionExecutionIdentityExactlyMatches(executor.Identity(), group.currentCmd.execution) ||
        !executor.Owner().Matches(group.currentCmd.ownerLease)) return false;
    const auto& previous = executor.Current();
    const auto& virtualAxis = group.virtualAxis;
    if (!previous.valid || !previous.runtime.valid ||
        virtualAxis.currentCmdPos != previous.scalarPulse ||
        virtualAxis.planningPos != previous.scalarPulse ||
        virtualAxis.logicalCmdPos.Load() != previous.scalarPulse ||
        virtualAxis.currentCmdVel != previous.velocityPulsePerSec ||
        virtualAxis.logicalCmdVel != previous.velocityPulsePerSec) return false;
    std::uint32_t mask = 0U;
    for (int slot = 0; slot < group.axisCount; ++slot)
    {
        const int index = group.axisIndices[slot];
        if (index < 0 || index >= MAX_AXES || static_cast<std::size_t>(index) >= axes->size() ||
            group.currentCmd.axisIndices[slot] != index || (mask & (1U << index)) != 0U) return false;
        mask |= 1U << index;
    }
    if (mask != executor.Runtime().AuthoredPath().groupMask) return false;
    for (unsigned index = 0U; index < 8U; ++index)
    {
        const bool present = index < axes->size() && (*axes)[index].isExist;
        if (present != (executor.Runtime().AuthoredPath().source.axisIdentity.exists[index] == 1U)) return false;
        if (present && (!NCRotaryFeedDetail::SameBits((*axes)[index].logicalCmdPos.Load(),
                previous.runtime.positionPulse[index]) ||
            (*axes)[index].logicalCmdVel != previous.runtime.velocityPulse[index])) return false;
    }
    return true;
}
}

MotionCore::~MotionCore() = default;

BASE79F_CONSUMER_NOINLINE bool MotionCore::InitializeEccentricCConsumer() noexcept
{
    // Startup-only. A repeat does not reset an active transaction or allocate.
    if (m_eccentricCConsumer) return true;
    if (m_Group.isActive) return false;
    m_eccentricCConsumer.reset(new (std::nothrow) MotionEccentricCConsumerState());
    return m_eccentricCConsumer != nullptr;
}

bool MotionCore::IsEccentricCConsumerReady() const noexcept
{ return m_eccentricCConsumer != nullptr; }

std::size_t MotionCore::EccentricCConsumerStorageBytes() const noexcept
{ return m_eccentricCConsumer ? sizeof(MotionEccentricCConsumerState) : 0U; }

BASE79F_CONSUMER_NOINLINE void MotionCore::ClearEccentricCConsumer() noexcept
{
    if (!m_eccentricCConsumer) return;
    auto& state = *m_eccentricCConsumer;
    if (!state.transaction.IsValid() && !state.transaction.HasPending()) return;
    (void)state.transaction.Clear();
    state.ticket = MotionEccentricCTicket{};
}

bool MotionCore::IsEccentricCCommandLiveValid(const MotionCommand& command,
    const NCEccentricCRuntimeValue& runtime, bool requireStart) const noexcept
{
    if (requireStart) return IsEccentricCStartBindingCurrent(command, runtime);
    if (!runtime.IsValid() || !IsMotionEccentricCFeedSourceAllowed(command) ||
        m_pContexts == nullptr || m_pContexts->empty() || m_pContexts->size() > 8U ||
        m_pCoordMgr == nullptr || m_Group.virtualAxis.isFault || m_Group.virtualAxis.isLagAlarm ||
        (requireStart && (m_Group.virtualAxis.currentCmdVel != 0.0 ||
            m_Group.virtualAxis.logicalCmdVel != 0.0)) ||
        !IsNCTranslationAxisIdentityCurrent(command.sourceTranslation) ||
        !SameNCTranslationSnapshot(command.sourceTranslation, runtime.AuthoredPath().source) ||
        ((requireStart || !m_safetyControlledStopInProgress) && !MatchesNCTranslation(command.sourceTranslation)) ||
        m_Group.enableHistory || m_Group.enableTransform ||
        m_Group.jumpManager.state != JumpState::IDLE || m_pathHold.sourceSeen ||
        (!requireStart && m_Group.pathMode != PathMode::EXACT_STOP) ||
        !std::isfinite(m_Group.feedrateOverride) ||
        (m_Group.feedrateOverride != 0.0 && m_Group.feedrateOverride != 1.0)) return false;
    const auto& source = runtime.AuthoredPath();
    const auto& extended = runtime.ExtendedPath();
    for (unsigned index = 0U; index < 8U; ++index)
    {
        const bool present = index < m_pContexts->size() && (*m_pContexts)[index].isExist;
        if (present != (source.source.axisIdentity.exists[index] == 1U)) return false;
        if (!present) continue;
        const AxisContext& axis = (*m_pContexts)[index];
        const bool moving = (source.groupMask & (1U << index)) != 0U;
        double ppu = 0.0;
        if (axis.axisIndex != static_cast<int>(index) || axis.isVirtualAxis ||
            source.source.axisIdentity.physicalIndexPlusOne[index] != index + 1U ||
            static_cast<unsigned>(axis.axisType) != source.source.axisIdentity.axisType[index] ||
            !TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
                axis.axisType != AxisType::LINEAR, ppu) ||
            !NCRotaryFeedDetail::SameBits(ppu, runtime.PulsePerNative()[index]) ||
            ((requireStart || !moving) &&
                !NCRotaryFeedDetail::SameBits(axis.logicalCmdPos.Load(), runtime.StartPulse()[index])) ||
            (!moving && (axis.logicalCmdVel != 0.0 || axis.currentCmdVel != 0.0 ||
                axis.state != MotionState::MotionState_IDLE)) ||
            (moving && axis.state != MotionState::MotionState_INTERPOLATING) ||
            (requireStart && (axis.logicalCmdVel != 0.0 || axis.currentCmdVel != 0.0)) ||
            axis.isFault || axis.isLagAlarm ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, extended.minMCS[index]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, extended.maxMCS[index]) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.MinimumPulse()[index] / ppu) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, runtime.MaximumPulse()[index] / ppu)) return false;
        // BASE79L: dir3's shared XYZ ceiling remains a live contract even
        // for held linear axes. Servo and stop-time checks remain group-only.
        const bool compressedLinear = MotionEccentricCTransportDetail::AuthoredXY(command) && index < 3U;
        if (!moving && !compressedLinear) continue;
        const double maximumNativeVelocity = axis.maxVel_PPS / ppu;
        const double maximumNativeAcceleration = maximumNativeVelocity / axis.G00_acc_time;
        const double maximumNativeDeceleration = maximumNativeVelocity / axis.G00_dec_time;
        if ((moving && !axis.isServoOn) ||
            !NCEccentricCDetail::Positive(MotionEccentricCTransportDetail::Limit(command, index, 0U)) ||
            !NCEccentricCDetail::Positive(MotionEccentricCTransportDetail::Limit(command, index, 1U)) ||
            !NCEccentricCDetail::Positive(MotionEccentricCTransportDetail::Limit(command, index, 2U)) || !std::isfinite(maximumNativeVelocity) || maximumNativeVelocity <= 0.0 ||
            !std::isfinite(axis.G00_acc_time) || axis.G00_acc_time <= 0.0 ||
            !std::isfinite(axis.G00_dec_time) || axis.G00_dec_time <= 0.0 ||
            !std::isfinite(maximumNativeAcceleration) || maximumNativeAcceleration <= 0.0 ||
            !std::isfinite(maximumNativeDeceleration) || maximumNativeDeceleration <= 0.0 ||
            (moving && (!std::isfinite(axis.Stop_dec_time) || axis.Stop_dec_time < 0.001 ||
                runtime.StopSeconds() < axis.Stop_dec_time)) ||
            MotionEccentricCTransportDetail::Limit(command, index, 0U) > maximumNativeVelocity ||
            MotionEccentricCTransportDetail::Limit(command, index, 1U) > maximumNativeAcceleration ||
            MotionEccentricCTransportDetail::Limit(command, index, 2U) > maximumNativeDeceleration ||
            extended.requiredVelocityNative[index] > maximumNativeVelocity ||
            extended.requiredAccelerationNative[index] > maximumNativeAcceleration ||
            extended.requiredDecelerationNative[index] > maximumNativeDeceleration) return false;
    }
    return true;
}

MotionEccentricCContext MotionCore::GetEccentricCContext() const noexcept
{
    MotionEccentricCContext context{};
    context.execution = m_Group.currentCmd.execution;
    context.sourceOwner = m_Group.currentCmd.ownerLease;
    context.currentEpoch = GetCurrentExecutionEpoch();
    context.currentOwner = GetMotionOwnerLease();
    context.fault = !m_Group.isActive || m_Group.virtualAxis.isFault || m_Group.virtualAxis.isLagAlarm;
    if (m_safetyControlledStopInProgress)
    {
        context.safetyTicket = m_safetyControlledStopRequestTicket;
        context.safetyAuthorized = m_Group.axisCount > 0 && m_Group.axisCount <= MAX_AXES &&
            m_pContexts != nullptr && m_pContexts->size() <= MAX_AXES;
        for (int slot = 0; slot < m_Group.axisCount && context.safetyAuthorized; ++slot)
            context.safetyAuthorized = IsSafetyControlledStopAuthorized(m_Group.axisIndices[slot]);
    }
    return context;
}


BASE79F_CONSUMER_NOINLINE bool MotionCore::LoadEccentricCCommand(
    const MotionCommand& front) noexcept
{
    if (m_Group.isActive || HasPendingExecutionEpochChange() || HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(front) != MotionRejectReason::NONE) return false;
    if (!m_eccentricCConsumer)
    {
        // Normal startup fails before enabling RT if the one allocation failed.
        // A missing consumer cannot silently dispatch through the native lane.
        (void)RejectFrontCommandForPathModeAuthority(front,
            MotionCommandPathModeAuthorityDecision::REJECT_INVALID);
        return false;
    }
    auto& state = *m_eccentricCConsumer;
    ClearEccentricCConsumer();
    MotionEccentricCContext context{};
    context.execution = front.execution; context.sourceOwner = front.ownerLease;
    context.currentEpoch = GetCurrentExecutionEpoch(); context.currentOwner = GetMotionOwnerLease();
    if (!state.transaction.PrepareBegin(front, context, state.transport, state.decoded, state.ticket) ||
        !IsEccentricCStartBindingCurrent(front, state.decoded.Runtime()))
    {
        (void)state.transaction.Discard();
        if (!HasPendingExecutionEpochChange() && !HasPendingSafetyOrRecoveryRequests())
            (void)RejectFrontCommandForPathModeAuthority(front,
                MotionCommandPathModeAuthorityDecision::REJECT_INVALID);
        return false;
    }
    EccentricCCommitReservation reservation{};
    if (!TryAcquireEccentricCCommitReservation(context, reservation))
    {
        (void)state.transaction.Discard();
        if (!ReleaseEccentricCCommitReservation(reservation)) EmergencyStopAllAxesImpl(false);
        return false;
    }
    bool popped = false;
    bool integrityFailure = false;
    const bool committed = state.transaction.Commit(state.ticket, context,
        [&](const MotionCommand& packet, const MotionEccentricCExecutor&,
            const MotionEccentricCExecutor& candidate, MotionEccentricCOperation operation) noexcept -> bool
        {
            const bool bindingValid = operation == MotionEccentricCOperation::BEGIN &&
                MotionEccentricCConsumerDetail::SamePacket(front, packet) &&
                IsEccentricCStartBindingCurrent(packet, candidate.Runtime());
            if (!bindingValid)
            {
                integrityFailure = MatchesNCTranslation(packet.sourceTranslation) &&
                    IsEccentricCCommitReservationCurrent(context, reservation);
                return false;
            }
            if (!IsEccentricCCommitReservationCurrent(context, reservation)) return false;
            // The sole RT consumer pops only after all callbacks and authority
            // checks. Exact primitives prevent a same-identity packet swap.
            popped = m_Group.cmdQueue.ConsumerTryPop(state.dequeued);
            if (popped && !MotionEccentricCConsumerDetail::SamePacket(state.dequeued, packet))
            { integrityFailure = true; return false; }
            return popped &&
                MatchesNCTranslation(packet.sourceTranslation) &&
                IsEccentricCCommitReservationCurrent(context, reservation);
        },
        [&](const MotionCommand& cmd, const NCEccentricCProfilePoint& start,
            MotionEccentricCOperation) noexcept
        {
            const auto& runtime = state.decoded.Runtime();
            InvalidateCncLineEndpointProof();
            m_Group.currentCmd = cmd;
            CommitCommandPathModeConsumerAuthority(cmd,
                MotionCommandPathModeAuthorityDecision::APPLY_EXACT_STOP);
            m_Group.currentExecutionPC = cmd.sourceLinePC;
            m_Group.currentExecutionWCS = cmd.sourceWCS;
            m_Group.currentExecutionToolMode = cmd.sourceToolLengthMode;
            m_Group.currentExecutionHCode = cmd.sourceHCode;
            m_Group.currentExecutionToolRadiusMode = cmd.sourceToolRadiusMode;
            m_Group.currentExecutionDCode = cmd.sourceDCode;
            m_Group.currentExecutionIsAbsoluteMode = cmd.sourceIsAbsoluteMode;
            m_Group.currentExecutionG68Active = cmd.sourceG68Active;
            m_Group.currentExecutionG68Angle = cmd.sourceG68Angle;
            m_Group.currentExecutionG168Active = cmd.sourceG168Active;
            m_Group.currentExecutionWCode = cmd.sourceWCode;
            m_Group.currentExecutionG51Active = cmd.sourceG51Active;
            m_Group.currentExecutionScaleRatio = cmd.sourceScaleRatio;
            m_Group.currentExecutionMirrorMask = cmd.sourceMirrorMask;
            m_Group.currentExecutionG16Active = cmd.sourceG16Active;
            m_Group.currentExecutionG162Active = cmd.sourceG162Active;
            m_Group.currentExecutionPlaneMode = cmd.sourcePlaneMode;
            m_Group.mode = cmd.mode; m_Group.axisCount = cmd.axisCount;
            for (int slot = 0; slot < cmd.axisCount; ++slot)
            {
                const int index = cmd.axisIndices[slot];
                m_Group.axisIndices[slot] = index;
                m_Group.startPos[slot] = runtime.StartPulse()[index];
                m_Group.ratio[slot] = 0.0; // Never a chord.
                AxisContext& axis = (*m_pContexts)[index];
                axis.logicalCmdPos = start.runtime.positionPulse[index];
                axis.logicalCmdVel = start.runtime.velocityPulse[index];
                axis.state = MotionState::MotionState_INTERPOLATING; axis.inPosition = false;
            }
            m_Group.totalDist3D = runtime.AuthoredScalarPulse();
            auto& axis = m_Group.virtualAxis;
            axis.currentCmdPos = axis.logicalCmdPos = axis.planningPos = start.scalarPulse;
            axis.startCmdPos = start.scalarPulse;
            axis.currentCmdVel = axis.logicalCmdVel = start.velocityPulsePerSec;
            axis.targetVelocity = axis.targetEndVel = 0.0;
            axis.finalTargetPos = runtime.AuthoredScalarPulse();
            axis.maxVel_PPS = axis.cruiseVel_PPS = runtime.MaximumScalarVelocityPulsePerSec();
            axis.acc_PPS2 = runtime.MaximumScalarAccelerationPulsePerSec2();
            axis.dec_PPS2 = runtime.MaximumScalarDecelerationPulsePerSec2();
            axis.bufferSum = 0.0; axis.bufferIndex = 0;
            std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
            axis.state = MotionState::MotionState_MOVING; axis.inPosition = false;
            TrackMotionCommandAccepted(m_Group.currentCmd);
            m_Group.isActive = true;
            TrackMotionCommandStarted(m_Group.currentCmd);
        });
    const bool released = ReleaseEccentricCCommitReservation(reservation);
    if (!released)
    {
        if (!committed && popped) RejectMotionCommand(state.dequeued, MotionRejectReason::NOT_READY, 0U);
        EmergencyStopAllAxesImpl(false);
        return false;
    }
    if (!committed && popped)
    {
        if (integrityFailure) TriggerGroupMappingIntegrityEmergencyStop(-1, true);
        RejectMotionCommand(state.dequeued, integrityFailure ? MotionRejectReason::INVALID_GEOMETRY :
            MotionRejectReason::NOT_READY, integrityFailure ?
            static_cast<std::uint32_t>(AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY) : 0U);
    }
    else if (!committed && integrityFailure)
        (void)RejectFrontCommandForPathModeAuthority(front,
            MotionCommandPathModeAuthorityDecision::REJECT_INVALID);
    return committed;
}

BASE79F_CONSUMER_NOINLINE bool MotionCore::StepEccentricCProfile(
    AxisCommand& command, NCEccentricCProfilePoint& point) noexcept
{
    point.Clear();
    if (!m_eccentricCConsumer || !m_eccentricCConsumer->transaction.IsValid())
    { TriggerGroupMappingIntegrityEmergencyStop(-1, true); return false; }
    auto& state = *m_eccentricCConsumer;
    auto& transaction = state.transaction;
    const auto& executor = transaction.Committed();
    if (!MotionEccentricCConsumerDetail::SamePacket(m_Group.currentCmd, transaction.Command()) ||
        !IsEccentricCCommandLiveValid(m_Group.currentCmd, executor.Runtime(), false) ||
        !MotionEccentricCConsumerDetail::PreviousImage(m_Group, m_pContexts, executor))
    { TriggerGroupMappingIntegrityEmergencyStop(-1, true); return false; }
    const MotionEccentricCContext context = GetEccentricCContext();
    if (m_safetyControlledStopInProgress)
    {
        if (!context.safetyAuthorized) { EmergencyStopAllAxesImpl(false); return false; }
    }
    else if (HasPendingExecutionEpochChange() || HasPendingSafetyOrRecoveryRequests() ||
        GetCommandAuthorizationFailure(m_Group.currentCmd) != MotionRejectReason::NONE) return false;

    // BASE79H: a bounded HOLD stop may reach/past the authored endpoint when
    // Stop_dec_time is slower than the authored deceleration. It has no
    // remaining forward path to resume. Never clamp or reverse that stop tail,
    // or pretend it completed the authored segment. An explicit resume must
    // report AL2014 and retire the stopped execution instead of remaining RUN
    // forever; the existing operator RESET transaction is required to recover.
    // This is a refusal before preparing/committing any new axis sample. The
    // applied SAFETY stop lane remains independent of ordinary HOLD/resume.
    if (!m_safetyControlledStopInProgress && m_Group.feedrateOverride == 1.0 &&
        executor.State() == NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END)
    {
        AlarmManager::GetInstance().Trigger(
            AlarmManager::PATH_EXECUTION_NOT_READY, m_Group.currentCmd.sourceLinePC);
        // Publish the normal ticket-bound emergency ingress. The next sole RT
        // action establishes SAFETY owner/epoch and retires the cursor; calling
        // the private implementation here would bypass that request authority.
        // The current sample is already stopped and no new sample is emitted.
        RequestEmergencyStopAllAxes();
        return false;
    }

    // A stop/resume request commits its unchanged position this tick. Advancing
    // a second candidate here would invalidate the first transaction ticket.
    bool prepared = false;
    if (!m_safetyControlledStopInProgress && m_Group.feedrateOverride == 0.0 &&
        executor.State() == NCEccentricCProfileState::AUTHORED_ACTIVE)
        prepared = transaction.PrepareStop(context, state.ticket);
    else if (!m_safetyControlledStopInProgress && m_Group.feedrateOverride == 1.0 &&
        executor.State() == NCEccentricCProfileState::STOPPED_BEFORE_END)
        prepared = transaction.PrepareResume(context, state.ticket);
    else prepared = transaction.PrepareAdvance(context, state.ticket);
    if (!prepared) { TriggerGroupMappingIntegrityEmergencyStop(-1, true); return false; }
    EccentricCCommitReservation reservation{};
    if (!TryAcquireEccentricCCommitReservation(context, reservation))
    {
        (void)transaction.Discard();
        if (!ReleaseEccentricCCommitReservation(reservation)) EmergencyStopAllAxesImpl(false);
        return false;
    }
    bool integrityFailure = false;
    const bool committed = transaction.Commit(state.ticket, context,
        [&](const MotionCommand& packet, const MotionEccentricCExecutor& previous,
            const MotionEccentricCExecutor& candidate, MotionEccentricCOperation) noexcept -> bool
        {
            const auto& next = candidate.Current();
            const bool authored = next.state == NCEccentricCProfileState::AUTHORED_COMPLETE;
            const bool stopped = next.state == NCEccentricCProfileState::STOPPED_BEFORE_END ||
                next.state == NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END;
            const bool imageValid = next.valid && next.runtime.valid &&
                (!(authored || (context.safetyAuthorized && stopped)) || next.velocityPulsePerSec == 0.0) &&
                (!authored || next.scalarPulse == candidate.Runtime().AuthoredScalarPulse()) &&
                MotionEccentricCConsumerDetail::SamePacket(m_Group.currentCmd, packet) &&
                IsEccentricCCommandLiveValid(packet, candidate.Runtime(), false) &&
                MotionEccentricCConsumerDetail::PreviousImage(m_Group, m_pContexts, previous) &&
                MotionExecutionIdentityExactlyMatches(candidate.Identity(), packet.execution) &&
                candidate.Owner().Matches(packet.ownerLease);
            const bool sourceCurrent = context.safetyAuthorized || MatchesNCTranslation(packet.sourceTranslation);
            const bool authorityCurrent = IsEccentricCCommitReservationCurrent(context, reservation);
            integrityFailure = !imageValid && sourceCurrent && authorityCurrent;
            return imageValid && sourceCurrent && authorityCurrent;
        },
        [&](const MotionCommand&, const NCEccentricCProfilePoint& next,
            MotionEccentricCOperation) noexcept
        {
            const bool stopped = next.state == NCEccentricCProfileState::STOPPED_BEFORE_END ||
                next.state == NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END;
            const bool retiring = next.state == NCEccentricCProfileState::AUTHORED_COMPLETE ||
                (context.safetyAuthorized && stopped);
            auto& axis = m_Group.virtualAxis;
            axis.currentCmdPos = axis.logicalCmdPos = axis.planningPos = next.scalarPulse;
            axis.currentCmdVel = axis.logicalCmdVel = next.velocityPulsePerSec;
            axis.targetVelocity = axis.targetEndVel = 0.0;
            axis.finalTargetPos = retiring ? next.scalarPulse : executor.Runtime().AuthoredScalarPulse();
            axis.state = retiring ? MotionState::MotionState_IDLE : MotionState::MotionState_MOVING;
            axis.inPosition = retiring;
            for (int slot = 0; slot < m_Group.axisCount; ++slot)
            {
                const int index = m_Group.axisIndices[slot];
                (*m_pContexts)[index].logicalCmdPos = next.runtime.positionPulse[index];
                (*m_pContexts)[index].logicalCmdVel = next.runtime.velocityPulse[index];
            }
            command.instantCmdPos = next.scalarPulse; command.instantCmdVel = next.velocityPulsePerSec;
            point = next; // Copy before Commit invalidates its borrowed view.
        });
    const bool released = ReleaseEccentricCCommitReservation(reservation);
    if (!released) { EmergencyStopAllAxesImpl(false); point.Clear(); return false; }
    if (integrityFailure) { TriggerGroupMappingIntegrityEmergencyStop(-1, true); return false; }
    return committed;
}

BASE79F_CONSUMER_NOINLINE bool MotionCore::BindEccentricCStop(
    const MotionEccentricCContext& context) noexcept
{
    if (!m_eccentricCConsumer || !m_eccentricCConsumer->transaction.IsValid() ||
        !context.safetyRequestAuthorized || context.safetyAuthorized || context.fault) return false;
    auto& state = *m_eccentricCConsumer;
    if (!state.transaction.PrepareStop(context, state.ticket)) return false;
    // StopGroupImpl is the sole RT action owner. Its epoch/action request is
    // still pending, so no logical sample may be emitted here. Bind only the
    // private control cursor; the next applied SAFETY tick uses both locks.
    return state.transaction.Commit(state.ticket, context,
        [&](const MotionCommand& packet, const MotionEccentricCExecutor& previous,
            const MotionEccentricCExecutor&, MotionEccentricCOperation operation) noexcept -> bool
        {
            if (operation != MotionEccentricCOperation::STOP ||
                !MotionEccentricCConsumerDetail::SamePacket(packet, m_Group.currentCmd) ||
                !IsEccentricCCommandLiveValid(packet, previous.Runtime(), false) ||
                !MotionEccentricCConsumerDetail::PreviousImage(m_Group, m_pContexts, previous)) return false;
            const auto owner = m_motionOwnerState.load(std::memory_order_acquire);
            const auto publication = m_executionEpochPublication.load(std::memory_order_acquire);
            return context.currentOwner.owner == MotionOwner::SAFETY && context.safetyTicket != 0ULL &&
                UnpackMotionOwnerState(owner).Matches(context.currentOwner) &&
                UnpackMotionOwnerSafetyRequestTicket(owner) == context.safetyTicket &&
                !UnpackMotionOwnerSafetyHandshake(owner) &&
                (owner & MOTION_OWNER_ANY_OUTPUT_RESERVATION) == 0ULL &&
                (publication & EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED) == 0ULL &&
                UnpackExecutionEpochPublication(publication) == context.currentEpoch &&
                UnpackExecutionEpochPublicationSource(publication) == MotionCommandSource::SAFETY &&
                m_safetyControlledStopInProgress &&
                m_safetyControlledStopOwnerLease.Matches(context.currentOwner) &&
                m_safetyControlledStopEpoch == context.currentEpoch &&
                m_safetyControlledStopRequestTicket == context.safetyTicket;
        },
        [](const MotionCommand&, const NCEccentricCProfilePoint&, MotionEccentricCOperation) noexcept {});
}

#undef BASE79F_CONSUMER_NOINLINE
