#include "MotionCore.h"

void MotionCore::G53_Move(const std::vector<int>& axes,
    const std::vector<double>& nativeTargets, BufferMode mode)
{
    // Compatibility caller: formal rejection remains visible to Motion.
    // NC uses the boolean transactional entry point below.
    (void)TryG00MoveInternal(axes, nativeTargets, mode,
        MotionCommandPathMode::EXACT_STOP, G00_overrideRatio,
        nullptr, false, nullptr, false, true);
}

bool MotionCore::TryG53MoveTransactionalTail(const std::vector<int>& axes,
    const std::vector<double>& nativeTargets, double* commandedMCSTail)
{
    return TryG00MoveInternal(axes, nativeTargets, BufferMode::ABORTING,
        MotionCommandPathMode::EXACT_STOP, G00_overrideRatio,
        commandedMCSTail, true, nullptr, false, true);
}

MotionNCTranslationTransitionResult MotionCore::CheckG53NativeHandoff(
    const NCTranslationSnapshot& snapshot, MotionExecutionEpoch executionEpoch,
    const MotionOwnerLease& ownerLease) noexcept
{
    if (!IsNCTranslationSnapshotValid(snapshot) || snapshot.distanceMode != 90 ||
        snapshot.cutterMode != 40 || snapshot.axisIdentity.eccentricEnabled != 0U ||
        !ownerLease.IsValid() || ownerLease.owner != MotionOwner::AUTO ||
        !MatchesNCTranslation(snapshot) ||
        !HasExactExecutionDrainAcknowledgement(executionEpoch, ownerLease))
        return MotionNCTranslationTransitionResult::DEFERRED;
    MotionExecutionIdentity reservationIdentity{};
    reservationIdentity.epoch = executionEpoch;
    reservationIdentity.segmentId = 1ULL; // Reservation key only; never admitted.
    reservationIdentity.source = MotionCommandSource::NC_MEMORY;
    LifecycleCommitReservationGuard reservation(*this, reservationIdentity);
    if (!reservation.IsAcquired() || !MatchesNCTranslation(snapshot) ||
        GetQueueSize() != 0U || GetCommandIngressSize() != 0U || GetCommandReplaySize() != 0U ||
        !HasExactExecutionDrainAcknowledgementImpl(executionEpoch, ownerLease, true))
        return MotionNCTranslationTransitionResult::DEFERRED;
    const MotionCommand& predecessor = m_Group.currentCmd;
    if (predecessor.execution.IsAssigned() &&
        predecessor.execution.source == MotionCommandSource::NC_MEMORY &&
        predecessor.ownerLease.Matches(ownerLease) &&
        predecessor.sourceTranslation.runToken == snapshot.runToken &&
        (predecessor.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
            predecessor.cncFeedLookahead || predecessor.cncCornerBlend ||
            predecessor.pathCoreRetainedTraversal || predecessor.pathCoreRetainedReverse ||
            predecessor.replayTerminalAlreadyPublished) &&
        !HasCompletedCncLineEndpointProof(predecessor, snapshot, executionEpoch))
        return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;
    return MotionNCTranslationTransitionResult::ACCEPTED;
}
