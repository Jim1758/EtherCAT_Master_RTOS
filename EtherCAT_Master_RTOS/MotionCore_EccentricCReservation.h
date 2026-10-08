#pragma once

// BASE79F: implementation fragment included only at the END of MotionCore.cpp.
// It uses that translation unit's existing packed-word constants and helpers.
// RT is the sole logical-image writer. Caller owns the candidate, validates
// source/axes/previous+next images, then keeps this reservation across Commit.
// No allocation, wait, retry loop, callback, or output publication occurs here.
//
// Frozen translation publication is intentionally unchanged. The NC writer
// may retire/replace it only at the existing exact drain/quiescence boundaries.
// The caller rechecks MatchesNCTranslation during final validation, but these
// owner/epoch bits do not lock an arbitrary direct RetireNCTranslation call.

bool MotionCore::TryAcquireEccentricCCommitReservation(
    const MotionEccentricCContext& context,
    EccentricCCommitReservation& reservation) noexcept
{
    // Do not overwrite a live token and silently abandon either reservation.
    if (reservation.ownerAcquired || reservation.epochAcquired || reservation.acquired ||
        reservation.alarmAdmission.acquired || reservation.releaseFailed)
        return false;
    reservation = EccentricCCommitReservation{};
    if (context.fault || !context.execution.IsAssigned() ||
        context.execution.source != MotionCommandSource::NC_MEMORY ||
        !context.sourceOwner.IsValid() || context.sourceOwner.owner != MotionOwner::AUTO ||
        !context.currentOwner.IsValid() || context.currentEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        context.safetyRequestAuthorized)
        return false;

    const bool safety = context.currentOwner.owner == MotionOwner::SAFETY;
    if (safety)
    {
        if (!context.safetyAuthorized || context.safetyTicket == 0ULL ||
            context.safetyTicket > MOTION_OWNER_SAFETY_TICKET_MAX ||
            !m_safetyControlledStopInProgress || !m_Group.isActive ||
            m_Group.axisCount <= 0 || m_Group.axisCount > MAX_AXES ||
            m_pContexts == nullptr || m_pContexts->size() > MAX_AXES ||
            !m_safetyControlledStopOwnerLease.Matches(context.currentOwner) ||
            m_safetyControlledStopEpoch != context.currentEpoch ||
            m_safetyControlledStopRequestTicket != context.safetyTicket)
            return false;
        // This existing checker rejects COMMIT_RESERVED. Use it only BEFORE
        // acquisition; IsCurrent below proves the equivalent reserved tuple.
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
            if (!IsSafetyControlledStopAuthorized(m_Group.axisIndices[slot])) return false;
    }
    else if (context.currentOwner.owner != MotionOwner::AUTO || context.safetyAuthorized ||
        context.safetyTicket != 0ULL || m_safetyControlledStopInProgress ||
        !context.currentOwner.Matches(context.sourceOwner) ||
        context.currentEpoch != context.execution.epoch || HasPendingSafetyOrRecoveryRequests())
        return false;

    reservation.safetyIntentState = m_frameSafetyIntentState.load(std::memory_order_acquire);
    reservation.revocationGeneration =
        m_executionDrainRevocationGeneration.load(std::memory_order_acquire);
    reservation.sourceExecution = context.execution;
    reservation.sourceOwner = context.sourceOwner;
    reservation.baseOwnerState = m_motionOwnerState.load(std::memory_order_acquire);
    reservation.baseExecutionPublication =
        m_executionEpochPublication.load(std::memory_order_acquire);
    if (static_cast<std::uint32_t>(reservation.safetyIntentState) != 0U ||
        !UnpackMotionOwnerState(reservation.baseOwnerState).Matches(context.currentOwner) ||
        UnpackMotionOwnerSafetyHandshake(reservation.baseOwnerState) ||
        UnpackMotionOwnerSafetyActionPending(reservation.baseOwnerState) ||
        (reservation.baseOwnerState & MOTION_OWNER_ANY_OUTPUT_RESERVATION) != 0ULL ||
        (reservation.baseExecutionPublication &
            (EXECUTION_EPOCH_PUBLICATION_PENDING | EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED)) != 0ULL ||
        UnpackExecutionEpochPublication(reservation.baseExecutionPublication) != context.currentEpoch ||
        UnpackExecutionEpochPublicationSource(reservation.baseExecutionPublication) !=
            (safety ? MotionCommandSource::SAFETY : MotionCommandSource::NC_MEMORY) ||
        (safety ? UnpackMotionOwnerSafetyRequestTicket(reservation.baseOwnerState) != context.safetyTicket :
            UnpackMotionOwnerSafetyRequestTicket(reservation.baseOwnerState) !=
                m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire)))
        return false;

    // This bit blocks ticket publication as well as lease changes. The weaker
    // OUTPUT_COMMIT_RESERVED bit deliberately permits a newer safety ticket.
    reservation.reservedOwnerState =
        reservation.baseOwnerState | MOTION_OWNER_EPOCH_COMMIT_RESERVED;
    std::uint64_t expectedOwner = reservation.baseOwnerState;
    if (!m_motionOwnerState.compare_exchange_strong(expectedOwner, reservation.reservedOwnerState,
        std::memory_order_acq_rel, std::memory_order_acquire))
        return false;
    reservation.ownerAcquired = true;

    reservation.reservedExecutionPublication =
        reservation.baseExecutionPublication | EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED;
    std::uint64_t expectedEpoch = reservation.baseExecutionPublication;
    if (!m_executionEpochPublication.compare_exchange_strong(expectedEpoch,
        reservation.reservedExecutionPublication, std::memory_order_acq_rel, std::memory_order_acquire))
    {
        (void)ReleaseEccentricCCommitReservation(reservation);
        return false;
    }
    reservation.epochAcquired = true;
    // Match the existing physical-frame admission contract for both AUTO and
    // applied SAFETY: a currently present/deferred Alarm cannot admit a new
    // logical sample. One bounded attempt only; no waiting for Alarm/Clear.
    AlarmManager& alarms = AlarmManager::GetInstance();
    const std::uint32_t alarmRevision = alarms.GetUpdateCount();
    if (!alarms.BeginMotionAdmission(alarmRevision, reservation.alarmAdmission, true))
    {
        (void)ReleaseEccentricCCommitReservation(reservation);
        return false;
    }
    reservation.acquired = true;
    if (!IsEccentricCCommitReservationCurrent(context, reservation))
    {
        (void)ReleaseEccentricCCommitReservation(reservation);
        return false;
    }
    return true;
}

bool MotionCore::IsEccentricCCommitReservationCurrent(
    const MotionEccentricCContext& context,
    const EccentricCCommitReservation& reservation) const noexcept
{
    if (!reservation.acquired || !reservation.ownerAcquired || !reservation.epochAcquired ||
        !reservation.alarmAdmission.acquired ||
        context.fault || context.safetyRequestAuthorized ||
        !MotionExecutionIdentityExactlyMatches(context.execution, reservation.sourceExecution) ||
        !context.sourceOwner.Matches(reservation.sourceOwner) ||
        !context.execution.IsAssigned() || context.execution.source != MotionCommandSource::NC_MEMORY ||
        !context.sourceOwner.IsValid() || context.sourceOwner.owner != MotionOwner::AUTO ||
        !context.currentOwner.IsValid() || context.currentEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !UnpackMotionOwnerState(reservation.baseOwnerState).Matches(context.currentOwner) ||
        UnpackExecutionEpochPublication(reservation.baseExecutionPublication) != context.currentEpoch ||
        m_motionOwnerState.load(std::memory_order_acquire) != reservation.reservedOwnerState ||
        m_executionEpochPublication.load(std::memory_order_acquire) != reservation.reservedExecutionPublication)
        return false;

    const bool safety = context.currentOwner.owner == MotionOwner::SAFETY;
    if (safety)
    {
        if (!context.safetyAuthorized || context.safetyTicket == 0ULL ||
            context.safetyTicket > MOTION_OWNER_SAFETY_TICKET_MAX ||
            !m_safetyControlledStopInProgress || !m_Group.isActive ||
            m_Group.axisCount <= 0 || m_Group.axisCount > MAX_AXES ||
            m_pContexts == nullptr || m_pContexts->size() > MAX_AXES ||
            !m_safetyControlledStopOwnerLease.Matches(context.currentOwner) ||
            m_safetyControlledStopEpoch != context.currentEpoch ||
            m_safetyControlledStopRequestTicket != context.safetyTicket ||
            UnpackMotionOwnerSafetyRequestTicket(reservation.baseOwnerState) != context.safetyTicket ||
            UnpackExecutionEpochPublicationSource(reservation.baseExecutionPublication) != MotionCommandSource::SAFETY)
            return false;
        // No call to IsSafetyControlledStopAuthorized here: its unreserved
        // publication requirement is replaced by exact equality above.
        for (int slot = 0; slot < m_Group.axisCount; ++slot)
        {
            const int index = m_Group.axisIndices[slot];
            if (index < 0 || index >= MAX_AXES ||
                static_cast<std::size_t>(index) >= m_pContexts->size() ||
                !(*m_pContexts)[index].isExist)
                return false;
        }
    }
    else if (context.currentOwner.owner != MotionOwner::AUTO || context.safetyAuthorized ||
        context.safetyTicket != 0ULL || m_safetyControlledStopInProgress ||
        !context.currentOwner.Matches(context.sourceOwner) ||
        context.currentEpoch != context.execution.epoch ||
        UnpackExecutionEpochPublicationSource(reservation.baseExecutionPublication) != MotionCommandSource::NC_MEMORY ||
        UnpackMotionOwnerSafetyRequestTicket(reservation.baseOwnerState) !=
            m_safetyRequestAcknowledgedTicket.load(std::memory_order_acquire))
        return false;

    // Applied SAFETY intentionally excludes its own unacknowledged ticket,
    // while every independent pending producer remains a rejection fence.
    if (m_executionDrainRevocationPublishersInProgress.load(std::memory_order_acquire) != 0U ||
        m_executionDrainRevocationGeneration.load(std::memory_order_acquire) != reservation.revocationGeneration ||
        m_safetyRecoveryRequestInProgress.load(std::memory_order_acquire) ||
        (m_emergencyStopRequestPublication.load(std::memory_order_acquire) & EMERGENCY_STOP_REQUEST_PENDING) != 0ULL ||
        m_resetAllFaultsPending.load(std::memory_order_acquire) ||
        m_stopGroupPending.load(std::memory_order_acquire) ||
        m_resetSafetyBatchPending.load(std::memory_order_acquire) != 0ULL ||
        m_axisFaultResetPendingMask.load(std::memory_order_acquire) != 0U ||
        (m_p1MappingIntegrityAlarmRequestPublication.load(std::memory_order_acquire) & P1_MAPPING_ALARM_PENDING) != 0ULL ||
        GetResetControlledStopPhase() == ResetControlledStopPhase::PENDING ||
        m_motionOwnerState.load(std::memory_order_acquire) != reservation.reservedOwnerState ||
        m_executionEpochPublication.load(std::memory_order_acquire) != reservation.reservedExecutionPublication)
        return false;

    // The final authorization observation changes even when a short safety
    // producer could not publish its ticket because our owner bit was held.
    // A later request is ordered after this logical sample; the unchanged
    // whole-frame safety fence still governs the subsequent physical send.
    return AlarmManager::GetInstance().IsMotionAdmissionCurrent(reservation.alarmAdmission) &&
        m_frameSafetyIntentState.load(std::memory_order_acquire) == reservation.safetyIntentState;
}

bool MotionCore::ReleaseEccentricCCommitReservation(
    EccentricCCommitReservation& reservation) noexcept
{
    bool exact = !reservation.releaseFailed;
    if (reservation.epochAcquired)
    {
        std::uint64_t expected = reservation.reservedExecutionPublication;
        if (!m_executionEpochPublication.compare_exchange_strong(expected,
            reservation.baseExecutionPublication, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            m_executionEpochPublication.fetch_and(~EXECUTION_EPOCH_PUBLICATION_COMMIT_RESERVED,
                std::memory_order_acq_rel);
            exact = false;
        }
        reservation.epochAcquired = false;
    }
    if (reservation.ownerAcquired)
    {
        std::uint64_t expected = reservation.reservedOwnerState;
        if (!m_motionOwnerState.compare_exchange_strong(expected, reservation.baseOwnerState,
            std::memory_order_acq_rel, std::memory_order_acquire))
        {
            m_motionOwnerState.fetch_and(~MOTION_OWNER_EPOCH_COMMIT_RESERVED, std::memory_order_acq_rel);
            exact = false;
        }
        reservation.ownerAcquired = false;
    }
    reservation.acquired = false;
    reservation.releaseFailed = !exact;
    if (reservation.alarmAdmission.acquired)
    {
        // Alarm/Clear changes during validation are normal supersession,
        // unlike a violated raw owner/epoch reservation. End also promotes
        // deferred alarms, so it must run only AFTER both Motion bits are free.
        (void)AlarmManager::GetInstance().EndMotionAdmission(reservation.alarmAdmission);
    }
    // Containment, if !exact, belongs to the caller AFTER both bits are free.
    // A committed sample cannot be reported as an uncommitted retry here.
    return exact;
}
