#include "MotionCore.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <windows.h>
#include <rtapi.h>

namespace
{
    // Same native-tail seal as G00; this translation unit owns no new RT state.
    std::uint64_t ReferencePairTailFingerprint(
        const std::array<double, MAX_AXES>& pulse, const double* mcs,
        double rapidOverride) noexcept
    {
        std::uint64_t fingerprint = MOTION_QUEUE_TAIL_FINGERPRINT_SEED;
        const auto fold = [&fingerprint](double value)
        {
            std::uint64_t bits = 0ULL;
            static_assert(sizeof(bits) == sizeof(value), "Native tail double size.");
            std::memcpy(&bits, &value, sizeof(bits));
            fingerprint = (fingerprint ^ bits) * 1099511628211ULL;
        };
        for (int axis = 0; axis < MAX_AXES; ++axis)
        {
            fold(mcs != nullptr ? mcs[axis] : 0.0);
            fold(pulse[static_cast<std::size_t>(axis)]);
        }
        fold(rapidOverride);
        return fingerprint;
    }
}

void MotionCore::G28_Move(const std::vector<int>& axes,
    const std::vector<double>& nativeTargets,
    const std::vector<double>* intermediateTargets, BufferMode mode)
{
    // The legacy void API has no native-MCS transaction/capture contract.
    // Only the explicit NC pair API below may publish two reference legs.
    if (intermediateTargets != nullptr)
    {
        MotionCommand rejected{};
        rejected.mode = InterpolationMode::LINEAR;
        rejected.sourceLinePC = m_pendingSourcePC;
        rejected.commandPathMode = MotionCommandPathMode::EXACT_STOP;
        RejectInvalidProducerMotionCommand(rejected, GetCurrentExecutionEpoch(),
            m_pendingCommandSource.load(std::memory_order_acquire),
            GetMotionOwnerLease(), nullptr, nullptr);
        return;
    }
    (void)TryG00MoveInternal(axes, nativeTargets, mode,
        MotionCommandPathMode::EXACT_STOP, G00_overrideRatio,
        nullptr, false, nullptr, false, false, 28);
}

bool MotionCore::TryReferencePositionPairTransactionalTail(
    const std::vector<int>& axes,
    const std::vector<double>& intermediateNativeTargets,
    const std::vector<double>& finalNativeTargets,
    std::uint32_t intermediateMask, int profileCode,
    double* commandedMCSTail)
{
    const MotionExecutionEpoch plannedEpoch = GetCurrentExecutionEpoch();
    const MotionOwnerLease plannedOwner = GetMotionOwnerLease();
    const MotionCommandSource source = m_pendingCommandSource.load(std::memory_order_acquire);
    const double rapidOverride = G00_overrideRatio;
    MotionQueueTailCommitReceipt receipt{};
    receipt.transactionSequence = AllocateQueueTailTransactionSequence();
    receipt.attempted = true;
    receipt.beforeFingerprint = ReferencePairTailFingerprint(
        m_g00ProducerQueueTailPulse, commandedMCSTail, rapidOverride);
    receipt.committedFingerprint = receipt.beforeFingerprint;

    const auto reject = [&](bool invalid, bool formal, MotionRejectReason reason) -> bool
    {
        if (formal)
        {
            MotionCommand command{};
            command.mode = InterpolationMode::LINEAR;
            command.sourceLinePC = m_pendingSourcePC;
            command.commandPathMode = MotionCommandPathMode::EXACT_STOP;
            if (reason == MotionRejectReason::INVALID_GEOMETRY)
                RejectInvalidProducerMotionCommand(command, GetCurrentExecutionEpoch(),
                    source, GetMotionOwnerLease(), &receipt.identity, &receipt.ownerLease);
            else
                RejectNonGeometryProducerMotionCommand(command, GetCurrentExecutionEpoch(),
                    source, GetMotionOwnerLease(), reason, &receipt.identity, &receipt.ownerLease);
        }
        receipt.commandAccepted = false;
        receipt.commandedMCSCommitted = false;
        receipt.lastQueuedPulseCommitted = false;
        receipt.rapidOverrideCommitted = false;
        receipt.endpointExact = false;
        receipt.committedFingerprint = ReferencePairTailFingerprint(
            m_g00ProducerQueueTailPulse, commandedMCSTail, G00_overrideRatio);
        receipt.preservedOnReject = receipt.committedFingerprint == receipt.beforeFingerprint;
        receipt.accountingValid = receipt.preservedOnReject;
        PublishQueueTailTransactionReceipt(receipt, invalid);
        return false;
    };

    if (m_pContexts == nullptr || m_pCoordMgr == nullptr || commandedMCSTail == nullptr ||
        m_pContexts->size() > static_cast<std::size_t>(MAX_AXES) ||
        (profileCode != 28 && profileCode != 30 && profileCode != 32) ||
        axes.empty() || axes.size() > static_cast<std::size_t>(MAX_AXES) ||
        axes.size() != intermediateNativeTargets.size() || axes.size() != finalNativeTargets.size() ||
        intermediateMask == 0U || (intermediateMask >> MAX_AXES) != 0U ||
        !std::isfinite(rapidOverride) || rapidOverride < 0.0)
        return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
    if (plannedEpoch == MOTION_EXECUTION_EPOCH_INVALID)
        return reject(false, true, MotionRejectReason::STALE_EPOCH);
    if (!plannedOwner.IsValid() || plannedOwner.owner != MotionOwner::AUTO ||
        source != MotionCommandSource::NC_MEMORY)
        return reject(false, true, MotionRejectReason::OWNER_CONFLICT);

    // Native reference targets do not apply transformations. A frozen source
    // remains an identity/handoff proof, and keeps the existing XYZ-only gate.
    if (!m_pendingIsAbsoluteMode || m_pendingPlaneMode != 17 ||
        m_pendingTranslation.unitsMode != 21 ||
        m_pendingToolMode != 49 || m_pendingHCode != 0 ||
        m_pendingToolRadMode != 40 || m_pendingDCode != 0 || m_pendingG162Active ||
        m_pendingG68Active || m_pendingG168Active || m_pendingWCode != 0 ||
        m_pendingG51Active || m_pendingMirrorMask != 0U || m_pendingG16Active ||
        !IsPendingFixedTranslationSourceAllowed() || HasPendingSafetyOrRecoveryRequests() ||
        !IsGroupDone() || GetQueueSize() != 0U || GetCommandIngressSize() != 0U ||
        GetCommandReplaySize() != 0U ||
        !HasExactExecutionDrainAcknowledgement(plannedEpoch, plannedOwner) ||
        (!IsNCTranslationSnapshotEmpty(m_pendingTranslation) &&
            CheckG53NativeHandoff(m_pendingTranslation, plannedEpoch, plannedOwner) !=
                MotionNCTranslationTransitionResult::ACCEPTED) ||
        !m_programBlockMotionCaptureActive || m_programBlockMotionCapture.overflow ||
        m_programBlockMotionCapture.count != 0U)
        return reject(false, true, MotionRejectReason::NOT_READY);

    std::array<double, MAX_AXES> startPulse{};
    std::array<double, MAX_AXES> stagedPulse{};
    std::array<double, MAX_AXES> stagedMCS{};
    std::uint32_t enabledMask = 0U;
    for (int axisIndex = 0; axisIndex < MAX_AXES; ++axisIndex)
    {
        const std::size_t axisSlot = static_cast<std::size_t>(axisIndex);
        stagedMCS[axisSlot] = commandedMCSTail[axisIndex];
        if (axisSlot >= m_pContexts->size() || !(*m_pContexts)[axisSlot].isExist) continue;
        // Exactly one logical-position load per enabled axis for both legs.
        startPulse[axisSlot] = (*m_pContexts)[axisSlot].logicalCmdPos.Load();
        if (!std::isfinite(startPulse[axisSlot]))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        enabledMask |= 1U << static_cast<unsigned>(axisIndex);
    }
    if (enabledMask == 0U || (intermediateMask & ~enabledMask) != 0U)
        return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);

    std::vector<double> firstPulse(axes.size(), 0.0);
    std::vector<double> finalPulse(axes.size(), 0.0);
    std::uint32_t selectedMask = 0U;
    double groupAcc = 0.0, groupDec = 0.0;
    double squaredPulse[2] = {}, squaredUnit[2] = {}, maxTime[2] = {};
    bool displaced[2] = {};
    for (std::size_t slot = 0U; slot < axes.size(); ++slot)
    {
        const int index = axes[slot];
        if (index < 0 || index >= MAX_AXES || index >= static_cast<int>(m_pContexts->size()))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        const std::size_t axisSlot = static_cast<std::size_t>(index);
        const std::uint32_t bit = 1U << static_cast<unsigned>(index);
        const AxisContext& axis = (*m_pContexts)[axisSlot];
        if ((selectedMask & bit) != 0U || !axis.isExist || axis.axisIndex != index ||
            (axis.axisType != AxisType::LINEAR && axis.axisType != AxisType::ROTARY &&
                axis.axisType != AxisType::ROTARY_CONTINUOUS) ||
            !std::isfinite(intermediateNativeTargets[slot]) || !std::isfinite(finalNativeTargets[slot]))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        selectedMask |= bit;
        if (!axis.isHomed)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_Homed, m_pendingSourcePC, index);
            return reject(false, true, MotionRejectReason::NOT_READY);
        }

        const double acc = profileCode == 28 ? axis.G28_acc_time :
            profileCode == 30 ? axis.G30_acc_time : axis.G32_acc_time;
        const double dec = profileCode == 28 ? axis.G28_dec_time :
            profileCode == 30 ? axis.G30_dec_time : axis.G32_dec_time;
        const double profilePPS = profileCode == 28 ? axis.G28_PPS :
            profileCode == 30 ? axis.G30_PPS : axis.G32_PPS;
        if (!std::isfinite(acc) || acc < 0.0 || !std::isfinite(dec) || dec < 0.0 ||
            !std::isfinite(profilePPS) || profilePPS <= 0.0 ||
            !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS <= 0.0 ||
            !std::isfinite(axis.finalLead) || axis.finalLead <= 0.0 ||
            !std::isfinite(axis.resolution_PPR) || axis.resolution_PPR <= 0.0)
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        const double maxPPS = (std::min)(profilePPS, axis.maxVel_PPS);
        groupAcc = (std::max)(groupAcc, acc);
        groupDec = (std::max)(groupDec, dec);
        const bool shortest = axis.axisType == AxisType::ROTARY && axis.useShortestPath;
        double pulsePerUnit = 0.0;
        if (!TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead, shortest, pulsePerUnit))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);

        const auto withinTravel = [&](double target) -> bool
        {
            if (!std::isfinite(target)) return false;
            if (m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis, target)) return true;
            AlarmManager::GetInstance().Trigger(m_pCoordMgr->GetSoftwareTravelLimitAlarmCode(
                axis, AlarmManager::PROGRAMMED_OVER_TRAVEL), m_pendingSourcePC, index);
            return false;
        };
        const bool intermediateSelected = (intermediateMask & bit) != 0U;
        const double authoredFirst = intermediateSelected ? intermediateNativeTargets[slot] :
            startPulse[axisSlot] / pulsePerUnit;
        if (!withinTravel(authoredFirst) || !withinTravel(finalNativeTargets[slot]))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);

        // Omitted intermediate words retain their exact sampled pulse. Do not
        // round-trip the native value or apply modulo to that stationary axis.
        firstPulse[slot] = startPulse[axisSlot];
        if (intermediateSelected &&
            !TryResolveMotionTargetPulse(startPulse[axisSlot], authoredFirst * pulsePerUnit,
                pulsePerUnit, shortest, axis.rotaryModulo, firstPulse[slot]))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        // The second rotary leg starts at the resolved, unwrapped first end.
        if (!TryResolveMotionTargetPulse(firstPulse[slot], finalNativeTargets[slot] * pulsePerUnit,
            pulsePerUnit, shortest, axis.rotaryModulo, finalPulse[slot]) ||
            !withinTravel(firstPulse[slot] / pulsePerUnit) ||
            !withinTravel(finalPulse[slot] / pulsePerUnit))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);

        if (shortest)
        {
            // The existing RT loader resolves rotary packets once more.
            // A half-turn rounded across its tie can otherwise reverse the
            // first leg and invalidate the planned start of the second leg.
            // Prove both transported endpoints are exact fixed points before
            // admission; never silently change the authored tie direction.
            double consumerFirst = 0.0, consumerFinal = 0.0;
            if (!TryResolveMotionTargetPulse(startPulse[axisSlot], firstPulse[slot],
                    pulsePerUnit, true, axis.rotaryModulo, consumerFirst) ||
                !TryResolveMotionTargetPulse(firstPulse[slot], finalPulse[slot],
                    pulsePerUnit, true, axis.rotaryModulo, consumerFinal) ||
                std::memcmp(&consumerFirst, &firstPulse[slot], sizeof(double)) != 0 ||
                std::memcmp(&consumerFinal, &finalPulse[slot], sizeof(double)) != 0)
                return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
        }

        stagedPulse[axisSlot] = finalPulse[slot];
        // Keep the same authored-MCS / resolved-pulse convention as G00.
        stagedMCS[axisSlot] = finalNativeTargets[slot];
        const double from[2] = { startPulse[axisSlot], firstPulse[slot] };
        const double to[2] = { firstPulse[slot], finalPulse[slot] };
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            const double pulseDistance = std::abs(to[leg] - from[leg]);
            const double unitDistance = pulseDistance / pulsePerUnit;
            displaced[leg] = displaced[leg] || pulseDistance > 0.0;
            squaredPulse[leg] += pulseDistance * pulseDistance;
            squaredUnit[leg] += unitDistance * unitDistance;
            maxTime[leg] = (std::max)(maxTime[leg], pulseDistance / maxPPS);
        }
    }
    if (selectedMask != enabledMask)
        return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
    receipt.axisMask = enabledMask;
    if (groupAcc < 0.001) groupAcc = 0.2;
    if (groupDec < 0.001) groupDec = 0.2;
    double velocity[2] = {};
    for (unsigned leg = 0U; leg < 2U; ++leg)
    {
        const double pulseDistance = std::sqrt(squaredPulse[leg]);
        const double unitDistance = std::sqrt(squaredUnit[leg]);
        if (maxTime[leg] > 0.0) velocity[leg] = pulseDistance / maxTime[leg];
        if (!std::isfinite(pulseDistance) || !std::isfinite(unitDistance) ||
            !std::isfinite(maxTime[leg]) || !std::isfinite(velocity[leg]) ||
            !std::isfinite(velocity[leg] / groupAcc) ||
            !std::isfinite(velocity[leg] / groupDec) ||
            // The existing RT linear loader requires at least 1 pulse/s
            // for a displaced leg. Reject both before leg one is admitted;
            // never raise the velocity above a configured per-axis cap.
            (displaced[leg] && velocity[leg] < 1.0))
            return reject(true, true, MotionRejectReason::INVALID_GEOMETRY);
    }
    const std::uint64_t stagedFingerprint = ReferencePairTailFingerprint(
        stagedPulse, stagedMCS.data(), rapidOverride);

    MotionCommand first{}, second{};
    if (!TryLineMove(axes, firstPulse, velocity[0], groupAcc, groupDec,
        BufferMode::ABORTING, MotionCommandPathMode::EXACT_STOP,
        &receipt.identity, &receipt.ownerLease, plannedEpoch, &plannedOwner,
        false, nullptr, 0.0, false, &first) ||
        !TryLineMove(axes, finalPulse, velocity[1], groupAcc, groupDec,
            BufferMode::ABORTING, MotionCommandPathMode::EXACT_STOP,
            &receipt.identity, &receipt.ownerLease, plannedEpoch, &plannedOwner,
            false, nullptr, 0.0, false, &second))
        return reject(false, false, MotionRejectReason::NOT_READY);

    // Both complete packets and every endpoint are private at this boundary.
    // One producer owns the ring; its consumer can only increase free space.
    if (!m_Group.cmdQueue.ProducerHasCapacity(2U))
        return reject(false, true, MotionRejectReason::QUEUE_FULL);
    MotionExecutionEpoch publishedEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    if (!TryPublishOwnerAuthorizedAbortingExecutionEpoch(source, plannedEpoch, plannedOwner, publishedEpoch))
    {
        const MotionRejectReason reason = !IsMotionOwnerLeaseCurrent(plannedOwner)
            ? MotionRejectReason::OWNER_CONFLICT : GetCurrentExecutionEpoch() != plannedEpoch
            ? MotionRejectReason::STALE_EPOCH : MotionRejectReason::NOT_READY;
        return reject(false, true, reason);
    }
    AssignExecutionIdentity(first, publishedEpoch, source, plannedOwner);
    AssignExecutionIdentity(second, publishedEpoch, source, plannedOwner);
    receipt.identity = second.execution;
    receipt.ownerLease = second.ownerLease;
    if (!TryEnqueueMotionCommandPair(first, second))
        return reject(false, false, MotionRejectReason::NOT_READY);

    const auto tupleStable = [&]() -> bool
    {
        const std::uint64_t ownerState1 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionExecutionEpoch epoch1 = GetCurrentExecutionEpoch();
        const MotionOwnerLease owner1 = UnpackMotionOwnerState(ownerState1);
        const MotionExecutionEpoch epoch2 = GetCurrentExecutionEpoch();
        const std::uint64_t ownerState2 = m_motionOwnerState.load(std::memory_order_acquire);
        const MotionOwnerLease owner2 = UnpackMotionOwnerState(ownerState2);
        return first.execution.IsAssigned() && second.execution.IsAssigned() &&
            first.execution.segmentId != second.execution.segmentId &&
            first.execution.epoch == second.execution.epoch &&
            first.execution.sourceBlockId == second.execution.sourceBlockId &&
            first.execution.source == source && second.execution.source == source &&
            first.ownerLease.Matches(plannedOwner) && second.ownerLease.Matches(plannedOwner) &&
            epoch1 == publishedEpoch && epoch2 == publishedEpoch &&
            owner1.IsValid() && owner2.IsValid() && owner1.Matches(plannedOwner) &&
            owner2.Matches(plannedOwner) && ownerState1 == ownerState2 &&
            !UnpackMotionOwnerSafetyHandshake(ownerState2) &&
            UnpackMotionOwnerSafetyRequestTicket(ownerState2) ==
                m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire);
    };
    const auto failAcceptedCommit = [&](bool assigned) -> bool
    {
        // Admission is atomic, not completion: later HOLD/RESET may stop either
        // leg. Any post-admission drift invalidates the future-tail seal.
        m_g00ProducerQueueTailEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        m_g00ProducerQueueTailOwnerLease = MotionOwnerLease{};
        m_g00ProducerQueueTailValidMask = 0U;
        receipt.commandAccepted = true;
        receipt.commandedMCSCommitted = assigned;
        receipt.lastQueuedPulseCommitted = assigned;
        receipt.rapidOverrideCommitted = assigned;
        receipt.preservedOnReject = false;
        receipt.endpointExact = false;
        receipt.accountingValid = false;
        receipt.committedFingerprint = assigned ? stagedFingerprint : receipt.beforeFingerprint;
        PublishQueueTailTransactionReceipt(receipt, false);
        (void)TryPublishGroupMappingIntegrityAlarmRequest(GetCurrentExecutionEpoch());
        RequestEmergencyStopAllAxes();
        return false;
    };
    if (!tupleStable()) return failAcceptedCommit(false);
    bool captureMatches = m_programBlockMotionCaptureActive &&
        !m_programBlockMotionCapture.overflow && m_programBlockMotionCapture.count == 2U;
    if (captureMatches)
    {
        const MotionCommand* commands[2] = { &first, &second };
        for (unsigned leg = 0U; leg < 2U; ++leg)
        {
            const MotionProgramBlockSubmission& captured = m_programBlockMotionCapture.submissions[leg];
            const MotionExecutionIdentity& identity = commands[leg]->execution;
            captureMatches = captureMatches && captured.producerAccepted &&
                captured.immediateRejectReason == MotionRejectReason::NONE &&
                captured.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
                captured.translationGeneration == m_pendingTranslation.generation &&
                captured.identity.epoch == identity.epoch && captured.identity.segmentId == identity.segmentId &&
                captured.identity.sourceBlockId == identity.sourceBlockId && captured.identity.source == identity.source;
        }
    }
    if (!captureMatches || !IsPendingFixedTranslationSourceAllowed() ||
        (!IsNCTranslationSnapshotEmpty(m_pendingTranslation) && !MatchesNCTranslation(m_pendingTranslation)))
        return failAcceptedCommit(false);

    // The pair's single release-store is its admission point. Commit only the
    // final endpoint; the intermediate endpoint never becomes a producer tail.
    m_g00ProducerQueueTailPulse = stagedPulse;
    for (int index = 0; index < MAX_AXES; ++index)
    {
        const std::size_t slot = static_cast<std::size_t>(index);
        if ((enabledMask & (1U << static_cast<unsigned>(index))) != 0U)
            (*m_pContexts)[slot].lastQueuedPulse.Store(stagedPulse[slot]);
        commandedMCSTail[index] = stagedMCS[slot];
    }
    m_g00ProducerQueueTailValidMask = enabledMask;
    G00_overrideRatio = rapidOverride;
    if (!tupleStable()) return failAcceptedCommit(true);
    m_g00ProducerQueueTailEpoch = publishedEpoch;
    m_g00ProducerQueueTailOwnerLease = plannedOwner;
    if (!tupleStable()) return failAcceptedCommit(true);

    receipt.commandAccepted = true;
    receipt.commandedMCSCommitted = true;
    receipt.lastQueuedPulseCommitted = true;
    receipt.rapidOverrideCommitted = true;
    receipt.preservedOnReject = false;
    receipt.endpointExact = true;
    receipt.accountingValid = true;
    receipt.committedFingerprint = stagedFingerprint;
    // Existing capture binding scans identities, so only leg two receives this
    // block's final-tail receipt. Both legs retain their independent ledgers.
    PublishQueueTailTransactionReceipt(receipt, false);
    RtPrintf("[REFERENCE][PAIR] g=%d epoch=%llu first=%llu second=%llu capture=2 tail=1\n",
        profileCode, static_cast<unsigned long long>(publishedEpoch),
        static_cast<unsigned long long>(first.execution.segmentId),
        static_cast<unsigned long long>(second.execution.segmentId));
    return true;
}
