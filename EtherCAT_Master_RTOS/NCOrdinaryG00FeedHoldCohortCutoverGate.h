#pragma once

#include "NCOrdinaryG00FeedHoldCohortShadow.h"

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.7.1
// Strict Non-Exact Feed-Hold Cohort Fail-Closed Classification
//
// K.7.3 proved that the two immutable K.7.1 entries present at a PROGRAM
// Feed-Hold edge remain one exact cohort through physical Hold ACK, ACK-gated
// Resume and two ordered Ledger terminals.
//
// K.7.4 consumes only that already-published proof at the K.7.1 admission
// seam.  While the captured cohort is incomplete, no third ordinary G00 may
// enter read-ahead.  After both original members complete in order, ordinary
// read-ahead is released.  A failed, cancelled or interrupted proof falls
// back to the existing legacy drain path for the remainder of that Queue
// session.
//
// The gate does not submit/cancel Motion, write PDO, change NC state/PC,
// callbacks, Epoch or owner, and owns no Motion storage.
//
// K.7.7.1 hardens the original K.7.4 edge classifier.  Only the two exact
// K.7.3 BYPASSED phase/decision pairs may bypass this gate.  Failed,
// cancelled, malformed, superseding or regressed evidence is session-latched
// to the existing legacy drain path.  A lockout is reset only by a healthy,
// newer Registry receipt for a different nonzero Queue session.
// =============================================================================

enum class NCOrdinaryG00FeedHoldCohortAdmissionResult : std::uint8_t
{
    BYPASS = 0,
    WAIT_COHORT,
    ALLOW_READ_AHEAD,
    FALLBACK_LEGACY
};

enum class NCOrdinaryG00FeedHoldCohortCutoverPhase : std::uint8_t
{
    IDLE = 0,
    BYPASSED,
    BOUND,
    WAIT_TERMINAL,
    RELEASED,
    LEGACY_FALLBACK,
    DISABLED
};

enum class NCOrdinaryG00FeedHoldCohortCutoverDecision : std::uint8_t
{
    NONE = 0,
    DISABLED,
    BYPASS_NO_EXACT_COHORT,
    COHORT_BOUND,
    WAIT_HOLD_ACK,
    WAIT_RESUME,
    WAIT_FIRST_TERMINAL,
    WAIT_SECOND_TERMINAL,
    ALLOW_READ_AHEAD,
    FALLBACK_INTERRUPTED,
    FALLBACK_CANCELLED,
    FALLBACK_FAILED_PROOF,
    FALLBACK_REGISTRY,
    FALLBACK_ACCOUNTING,
    SESSION_RESET
};

struct NCOrdinaryG00FeedHoldCohortCutoverSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t cohortSequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;
    std::uint64_t gateSequence = 0ULL;
    std::uint64_t registryPublicationSequence = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    NCOrdinaryG00FeedHoldCohortCutoverPhase phase =
        NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE;
    NCOrdinaryG00FeedHoldCohortCutoverDecision decision =
        NCOrdinaryG00FeedHoldCohortCutoverDecision::NONE;

    std::uint32_t activeEntriesAtCapture = 0U;
    std::uint32_t registryActiveEntries = 0U;
    std::uint8_t memberCount = 0U;
    std::uint8_t terminalCount = 0U;

    bool enabled = true;
    bool exactCohort = false;
    bool bound = false;
    bool holdAcknowledged = false;
    bool resumeRequested = false;
    bool resumeApplied = false;
    bool terminalOrderValid = true;
    bool waiting = false;
    bool released = false;
    bool fallbackLegacy = false;
    bool sessionLockout = false;
    bool runtimeInfluence = false;
    bool resolverBypassed = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldCohortCutoverCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t cohortsBound = 0ULL;
    std::uint64_t bypassDisabled = 0ULL;
    std::uint64_t bypassNoExactCohort = 0ULL;
    std::uint64_t staleObservations = 0ULL;
    std::uint64_t admissionChecks = 0ULL;
    std::uint64_t admissionBypasses = 0ULL;
    std::uint64_t waitHoldAcknowledge = 0ULL;
    std::uint64_t waitResume = 0ULL;
    std::uint64_t waitFirstTerminal = 0ULL;
    std::uint64_t waitSecondTerminal = 0ULL;
    std::uint64_t releases = 0ULL;
    std::uint64_t allowReadAhead = 0ULL;
    std::uint64_t fallbackLegacy = 0ULL;
    std::uint64_t interruptedCohorts = 0ULL;
    std::uint64_t cancelledCohorts = 0ULL;
    std::uint64_t failedProofs = 0ULL;
    std::uint64_t registryMismatches = 0ULL;
    std::uint64_t accountingMismatches = 0ULL;
    std::uint64_t sessionResets = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t resolverBypasses = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortCutoverSnapshot>::value,
    "K.7.4 cutover snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortCutoverCounters>::value,
    "K.7.4 cutover counters must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortCutoverSnapshot) <= 192U,
    "K.7.4 cutover snapshot exceeded its fixed diagnostic budget.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortCutoverCounters) <= 256U,
    "K.7.4 cutover counters exceeded their fixed diagnostic budget.");

class NCOrdinaryG00FeedHoldCohortCutoverGate
{
public:
    NCOrdinaryG00FeedHoldCohortCutoverGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept
    {
        if (m_enabled == enabled)
        {
            return;
        }
        m_enabled = enabled;
        m_lastObservedCohortSequence = 0ULL;
        m_lastObservedCohortPublicationSequence = 0ULL;
        m_snapshot.enabled = enabled;
        if (!enabled)
        {
            ClearBinding(false);
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortCutoverPhase::DISABLED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::DISABLED;
        }
        else
        {
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::NONE;
        }
        Publish();
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    void Observe(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.observations;
        m_snapshot.enabled = m_enabled;
        if (!m_enabled)
        {
            ++m_counters.bypassDisabled;
            return;
        }

        const bool sessionReset = TryResetForChangedSession(registry);
        if (sessionReset &&
            ((cohort.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
                cohort.session != registry.currentSession) ||
                (cohort.session == NC_PREPARED_QUEUE_SESSION_INVALID &&
                    cohort.cohortSequence ==
                    m_lastObservedCohortSequence)))
        {
            return;
        }
        if (m_snapshot.sessionLockout)
        {
            return;
        }

        if (cohort.cohortSequence == 0ULL)
        {
            if (cohort.publicationSequence != 0ULL ||
                m_lastObservedCohortSequence != 0ULL ||
                m_snapshot.bound || m_snapshot.released)
            {
                AdoptFallbackSession(cohort, registry);
                RecordDecisionRegistryReceipt(registry);
                ++m_counters.failedProofs;
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_FAILED_PROOF,
                    false);
            }
            return;
        }
        if (cohort.publicationSequence == 0ULL ||
            (m_lastObservedCohortSequence != 0ULL &&
                cohort.cohortSequence < m_lastObservedCohortSequence) ||
            (m_lastObservedCohortPublicationSequence != 0ULL &&
                cohort.publicationSequence <
                m_lastObservedCohortPublicationSequence))
        {
            ++m_counters.staleObservations;
            ++m_counters.failedProofs;
            AdoptFallbackSession(cohort, registry);
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }

        const bool newEdge =
            cohort.cohortSequence != m_lastObservedCohortSequence;
        if (newEdge &&
            m_lastObservedCohortPublicationSequence != 0ULL &&
            cohort.publicationSequence <=
            m_lastObservedCohortPublicationSequence)
        {
            ++m_counters.staleObservations;
            ++m_counters.failedProofs;
            AdoptFallbackSession(cohort, registry);
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (!newEdge &&
            m_snapshot.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            cohort.cohortSequence == m_snapshot.cohortSequence &&
            cohort.publicationSequence >
            m_lastObservedCohortPublicationSequence)
        {
            ++m_counters.failedProofs;
            AdoptFallbackSession(cohort, registry);
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }

        if (newEdge)
        {
            if (m_snapshot.bound && !m_snapshot.released)
            {
                // K.7.3 Begin() cancels an active cohort before replacing its
                // snapshot.  K.7.4 can therefore observe only the replacement
                // edge; an unreleased binding makes that edge an implicit
                // supersede and it must never erase the pending gate.
                ++m_counters.cancelledCohorts;
                RecordDecisionRegistryReceipt(registry);
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_CANCELLED,
                    cohort.accountingValid && registry.accountingValid);
                return;
            }
            m_lastObservedCohortSequence = cohort.cohortSequence;
            m_lastObservedCohortPublicationSequence =
                cohort.publicationSequence;
            if (m_snapshot.sessionLockout &&
                registry.currentSession == m_snapshot.session)
            {
                return;
            }
            if (!ExactCapture(cohort))
            {
                ClassifyNonExact(cohort, registry);
                return;
            }
            Bind(cohort, registry);
            if (!m_snapshot.bound)
            {
                return;
            }
        }
        else if (cohort.publicationSequence >
            m_lastObservedCohortPublicationSequence)
        {
            m_lastObservedCohortPublicationSequence =
                cohort.publicationSequence;
        }

        if (m_snapshot.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            cohort.cohortSequence == m_snapshot.cohortSequence)
        {
            (void)PublishedBypassRegistrySafe(registry);
            return;
        }

        if (!m_snapshot.bound ||
            cohort.cohortSequence != m_snapshot.cohortSequence)
        {
            return;
        }
        if (!IdentityStable(cohort))
        {
            ++m_counters.failedProofs;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (cohort.failed ||
            cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_FAILED)
        {
            ClassifyFailed(cohort, registry);
            return;
        }
        if (cohort.cancelled ||
            cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_CANCELLED)
        {
            if (CancelledProofExact(cohort))
            {
                ++m_counters.cancelledCohorts;
                RecordDecisionRegistryReceipt(registry);
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_CANCELLED,
                    cohort.accountingValid && registry.accountingValid);
            }
            else
            {
                ++m_counters.failedProofs;
                RecordDecisionRegistryReceipt(registry);
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_FAILED_PROOF,
                    false);
            }
            return;
        }
        if (!RegistryValid(registry) ||
            registry.currentSession != m_snapshot.session ||
            registry.publicationSequence <
            m_snapshot.registryPublicationSequence)
        {
            ++m_counters.registryMismatches;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                false);
            return;
        }

        if (!GateCorrelationStable(cohort))
        {
            ++m_counters.failedProofs;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (!ProgressionStable(cohort))
        {
            ++m_counters.failedProofs;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }

        if (m_snapshot.released)
        {
            // Revalidate the immutable completed K.7.3 proof.  Registry
            // entries admitted after release are new work and are permitted;
            // they are not folded into the closed two-member accounting.
            if (!ReleasedProofExact(cohort))
            {
                ++m_counters.failedProofs;
                RecordDecisionRegistryReceipt(registry);
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_FAILED_PROOF,
                    false);
                return;
            }
            return;
        }

        const bool proofChanged =
            m_snapshot.gateSequence != cohort.gateSequence ||
            m_snapshot.registryActiveEntries != registry.activeEntries ||
            m_snapshot.memberCount != cohort.memberCount ||
            m_snapshot.terminalCount != cohort.terminalCount ||
            m_snapshot.holdAcknowledged != cohort.holdAcknowledged ||
            m_snapshot.resumeRequested != cohort.resumeRequested ||
            m_snapshot.resumeApplied != cohort.resumeApplied ||
            m_snapshot.terminalOrderValid != cohort.terminalOrderValid;

        m_snapshot.gateSequence = cohort.gateSequence;
        m_snapshot.registryActiveEntries = registry.activeEntries;
        m_snapshot.memberCount = cohort.memberCount;
        m_snapshot.terminalCount = cohort.terminalCount;
        m_snapshot.holdAcknowledged = cohort.holdAcknowledged;
        m_snapshot.resumeRequested = cohort.resumeRequested;
        m_snapshot.resumeApplied = cohort.resumeApplied;
        m_snapshot.terminalOrderValid = cohort.terminalOrderValid;

        if (!ActiveProofExact(cohort) &&
            !cohort.terminalCohortComplete)
        {
            ++m_counters.failedProofs;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (!AccountingExact(cohort, registry))
        {
            ++m_counters.accountingMismatches;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_ACCOUNTING,
                false);
            return;
        }

        if (cohort.terminalCohortComplete)
        {
            if (CompletedProofExact(cohort, registry))
            {
                if (m_snapshot.released)
                {
                    return;
                }
                ++m_counters.releases;
                m_snapshot.registryPublicationSequence =
                    registry.publicationSequence;
                m_snapshot.waiting = false;
                m_snapshot.released = true;
                m_snapshot.fallbackLegacy = false;
                m_snapshot.phase =
                    NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    ALLOW_READ_AHEAD;
                Publish();
            }
            else
            {
                ++m_counters.interruptedCohorts;
                RecordDecisionRegistryReceipt(registry);
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_INTERRUPTED,
                    true);
            }
            return;
        }

        NCOrdinaryG00FeedHoldCohortCutoverDecision nextDecision =
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            WAIT_FIRST_TERMINAL;
        if (!cohort.holdAcknowledged)
        {
            nextDecision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_HOLD_ACK;
        }
        else if (!cohort.resumeApplied)
        {
            nextDecision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_RESUME;
        }
        else if (cohort.terminalCount != 0U)
        {
            nextDecision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                WAIT_SECOND_TERMINAL;
        }
        const bool stateChanged =
            proofChanged ||
            m_snapshot.phase !=
            NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL ||
            m_snapshot.decision != nextDecision ||
            !m_snapshot.waiting;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL;
        m_snapshot.waiting = true;
        m_snapshot.decision = nextDecision;
        if (stateChanged)
        {
            Publish();
        }
    }

    NCOrdinaryG00FeedHoldCohortAdmissionResult EvaluateAdmission(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.admissionChecks;
        if (!m_enabled)
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldCohortAdmissionResult::BYPASS;
        }
        (void)TryResetForChangedSession(registry);
        if (!m_snapshot.bound && !m_snapshot.sessionLockout &&
            m_snapshot.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED)
        {
            if (!PublishedBypassRegistrySafe(registry))
            {
                ++m_counters.fallbackLegacy;
                RecordInfluence();
                return NCOrdinaryG00FeedHoldCohortAdmissionResult::
                    FALLBACK_LEGACY;
            }
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldCohortAdmissionResult::BYPASS;
        }
        if (!m_snapshot.bound && !m_snapshot.sessionLockout)
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldCohortAdmissionResult::BYPASS;
        }
        if (m_snapshot.sessionLockout || m_snapshot.fallbackLegacy)
        {
            ++m_counters.fallbackLegacy;
            RecordInfluence();
            return
                NCOrdinaryG00FeedHoldCohortAdmissionResult::FALLBACK_LEGACY;
        }
        if (!RegistryValid(registry) ||
            registry.currentSession != m_snapshot.session ||
            registry.publicationSequence <
            m_snapshot.registryPublicationSequence)
        {
            ++m_counters.registryMismatches;
            RecordDecisionRegistryReceipt(registry);
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                registry.accountingValid);
            ++m_counters.fallbackLegacy;
            RecordInfluence();
            return
                NCOrdinaryG00FeedHoldCohortAdmissionResult::FALLBACK_LEGACY;
        }
        if (!m_snapshot.released)
        {
            m_snapshot.registryActiveEntries = registry.activeEntries;
        }
        if (m_snapshot.released)
        {
            ++m_counters.allowReadAhead;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                ALLOW_READ_AHEAD;
            Publish();
            return
                NCOrdinaryG00FeedHoldCohortAdmissionResult::ALLOW_READ_AHEAD;
        }

        if (!m_snapshot.holdAcknowledged)
        {
            ++m_counters.waitHoldAcknowledge;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_HOLD_ACK;
        }
        else if (!m_snapshot.resumeApplied)
        {
            ++m_counters.waitResume;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_RESUME;
        }
        else if (m_snapshot.terminalCount == 0U)
        {
            ++m_counters.waitFirstTerminal;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                WAIT_FIRST_TERMINAL;
        }
        else
        {
            ++m_counters.waitSecondTerminal;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                WAIT_SECOND_TERMINAL;
        }
        m_snapshot.waiting = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL;
        RecordInfluence();
        Publish();
        return NCOrdinaryG00FeedHoldCohortAdmissionResult::WAIT_COHORT;
    }

    NCOrdinaryG00FeedHoldCohortCutoverSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldCohortCutoverCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    struct BoundMemberLineage
    {
        std::uint64_t registrySequence = 0ULL;
        NCPreparedQueueSession session =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        NCPreparedEntrySequence entrySequence =
            NC_PREPARED_ENTRY_SEQUENCE_INVALID;
        std::uint64_t dispatchId = 0ULL;
        NCProgramCommitSequence commitSequence =
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
        int sourcePC = -1;
        int sourceLineNumber = 0;
        MotionExecutionIdentity identity{};
        MotionOwnerLease ownerLease{};
        NCOrdinaryG00InflightState captureState =
            NCOrdinaryG00InflightState::K63_EMPTY;
    };

    static bool SameIdentity(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    static bool SameLease(
        const MotionOwnerLease& lhs,
        const MotionOwnerLease& rhs) noexcept
    {
        return lhs.owner == rhs.owner &&
            lhs.generation == rhs.generation;
    }

    static bool LegalInitialMemberState(
        NCOrdinaryG00InflightState state) noexcept
    {
        switch (state)
        {
        case NCOrdinaryG00InflightState::K63_REGISTERED:
        case NCOrdinaryG00InflightState::K63_ACCEPTED:
        case NCOrdinaryG00InflightState::K63_STARTED:
        case NCOrdinaryG00InflightState::K63_ACTIVE:
        case NCOrdinaryG00InflightState::K63_HELD:
            return true;
        case NCOrdinaryG00InflightState::K63_EMPTY:
        case NCOrdinaryG00InflightState::K63_REVOKED_PENDING:
        case NCOrdinaryG00InflightState::K63_COMPLETED:
        case NCOrdinaryG00InflightState::K63_REJECTED:
        case NCOrdinaryG00InflightState::K63_CANCELLED:
        case NCOrdinaryG00InflightState::K63_ABORTED:
        case NCOrdinaryG00InflightState::K63_FAULTED:
        default:
            return false;
        }
    }

    static bool CaptureMembersExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& first =
            cohort.members[0];
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& second =
            cohort.members[1];
        const bool firstExact = first.exact &&
            first.registrySequence != 0ULL &&
            first.session == cohort.session &&
            first.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            first.dispatchId != 0ULL &&
            first.commitSequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            first.identity.IsAssigned() &&
            first.identity.epoch == cohort.executionEpoch &&
            first.identity.source == MotionCommandSource::NC_MEMORY &&
            first.ownerLease.owner == MotionOwner::AUTO &&
            first.ownerLease.generation == cohort.ownerGeneration &&
            first.captureState == first.currentState &&
            LegalInitialMemberState(first.captureState) &&
            !first.heldObserved && !first.resumedObserved &&
            !first.terminalObserved && !first.terminalCompleted &&
            !first.terminalBeforeAcknowledge &&
            !first.terminalBeforeResume && first.terminalOrdinal == 0U;
        const bool secondExact = second.exact &&
            second.registrySequence != 0ULL &&
            second.session == cohort.session &&
            second.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            second.dispatchId != 0ULL &&
            second.commitSequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            second.identity.IsAssigned() &&
            second.identity.epoch == cohort.executionEpoch &&
            second.identity.source == MotionCommandSource::NC_MEMORY &&
            second.ownerLease.owner == MotionOwner::AUTO &&
            second.ownerLease.generation == cohort.ownerGeneration &&
            second.captureState == second.currentState &&
            LegalInitialMemberState(second.captureState) &&
            !second.heldObserved && !second.resumedObserved &&
            !second.terminalObserved && !second.terminalCompleted &&
            !second.terminalBeforeAcknowledge &&
            !second.terminalBeforeResume && second.terminalOrdinal == 0U;
        return firstExact && secondExact &&
            first.registrySequence < second.registrySequence&&
            first.entrySequence < second.entrySequence&&
            first.identity.segmentId != second.identity.segmentId &&
            !SameIdentity(first.identity, second.identity);
    }

    static bool ExactCapture(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        return
            cohort.candidate &&
            cohort.active &&
            cohort.exactTwoEntryCohort &&
            cohort.boundaryCorrelated &&
            cohort.activeEntriesAtCapture ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.memberCount == NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            cohort.executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            cohort.owner == MotionOwner::AUTO &&
            cohort.ownerGeneration != MOTION_OWNER_GENERATION_INVALID &&
            cohort.boundarySequence != 0ULL &&
            cohort.gateSequence == 0ULL &&
            cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_CAPTURED &&
            cohort.decision ==
            NCOrdinaryG00FeedHoldCohortDecision::K73_CAPTURED_EXACT &&
            cohort.terminalCount == 0U &&
            !cohort.failed &&
            !cohort.cancelled &&
            !cohort.gateCorrelated &&
            !cohort.holdAcknowledged &&
            !cohort.resumeRequested &&
            !cohort.resumeBeforeAcknowledge &&
            !cohort.resumeAfterAcknowledge &&
            !cohort.resumeApplied &&
            cohort.terminalOrderValid &&
            !cohort.terminalCohortComplete &&
            !cohort.allMembersCompleted &&
            cohort.bounded &&
            cohort.shadowOnly &&
            !cohort.runtimeInfluence &&
            !cohort.motionWrite &&
            cohort.registryPublicationSequence != 0ULL &&
            CaptureMembersExact(cohort) &&
            cohort.accountingValid;
    }

    static bool RegistryShapeExact(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            registry.capacity ==
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY &&
            registry.activeEntries <= registry.occupiedEntries &&
            registry.occupiedEntries <= registry.capacity &&
            registry.revokedPendingEntries <= registry.activeEntries &&
            registry.bounded &&
            !registry.motionWrite;
    }

    static bool PristineIdentity(
        const MotionExecutionIdentity& identity) noexcept
    {
        return identity.epoch == MOTION_EXECUTION_EPOCH_INVALID &&
            identity.segmentId == MOTION_SEGMENT_ID_INVALID &&
            identity.sourceBlockId == MOTION_SOURCE_BLOCK_ID_INVALID &&
            identity.source == MotionCommandSource::UNKNOWN;
    }

    static bool PristineLease(const MotionOwnerLease& lease) noexcept
    {
        return lease.owner == MotionOwner::NONE &&
            lease.generation == MOTION_OWNER_GENERATION_INVALID;
    }

    static bool PristineQueueTailReceipt(
        const MotionQueueTailCommitReceipt& receipt) noexcept
    {
        return PristineIdentity(receipt.identity) &&
            PristineLease(receipt.ownerLease) &&
            receipt.transactionSequence ==
            MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID &&
            receipt.axisMask == 0U &&
            receipt.beforeFingerprint ==
            MOTION_QUEUE_TAIL_FINGERPRINT_SEED &&
            receipt.committedFingerprint ==
            MOTION_QUEUE_TAIL_FINGERPRINT_SEED &&
            !receipt.attempted && !receipt.commandAccepted &&
            !receipt.commandedMCSCommitted &&
            !receipt.lastQueuedPulseCommitted &&
            !receipt.rapidOverrideCommitted &&
            !receipt.preservedOnReject && !receipt.endpointExact &&
            !receipt.captureBound && receipt.accountingValid;
    }

    static bool PristineRegistryEntry(
        const NCOrdinaryG00InflightEntrySnapshot& entry) noexcept
    {
        return entry.registrySequence == 0ULL &&
            entry.session == NC_PREPARED_QUEUE_SESSION_INVALID &&
            entry.entrySequence == NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            entry.scope == NCProgramScope::NONE &&
            entry.cacheGeneration == NC_PROGRAM_CACHE_GENERATION_INVALID &&
            entry.frameId == NC_PROGRAM_FRAME_ID_INVALID &&
            entry.sourceExecutionEpoch == 0ULL &&
            entry.programFlowGeneration == 0ULL && entry.owner == 0U &&
            entry.panelMask == 0U && entry.ownerGeneration == 0ULL &&
            entry.sourcePC == -1 && entry.sourceLineNumber == 0 &&
            entry.dispatchId == 0ULL &&
            entry.commitSequence == NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            PristineIdentity(entry.identity) &&
            PristineLease(entry.ownerLease) &&
            PristineQueueTailReceipt(entry.queueTailReceipt) &&
            entry.lastFeedbackSequence ==
            MOTION_FEEDBACK_SEQUENCE_INVALID &&
            entry.lastFeedbackType == MotionFeedbackType::NONE &&
            entry.state == NCOrdinaryG00InflightState::K63_EMPTY &&
            entry.revocation ==
            NCOrdinaryG00InflightRevocation::K63_NONE &&
            !entry.occupied && !entry.active && !entry.terminal &&
            !entry.revoked && !entry.ledgerAcceptedTerminal &&
            !entry.readAheadCutover;
    }

    static bool RegistryAuthorityTrusted(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return registry.currentSession !=
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.accountingValid && RegistryShapeExact(registry);
    }

    static bool RegistryReceiptTrusted(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return RegistryAuthorityTrusted(registry) &&
            registry.publicationSequence != 0ULL &&
            registry.publicationSequence !=
            (std::numeric_limits<std::uint64_t>::max)();
    }

    static bool RegistryBypassReceiptHealthy(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (registry.ready)
        {
            return RegistryReceiptTrusted(registry) &&
                !registry.permanentLockout &&
                registry.activeEntries <=
                NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
        }
        const bool exactQuiescent =
            registry.activeEntries == 0U &&
            registry.occupiedEntries == 0U &&
            registry.revokedPendingEntries == 0U &&
            registry.lastRegistrySequence == 0ULL &&
            registry.peakActiveEntries == 0U &&
            !registry.permanentLockout && registry.accountingValid &&
            RegistryShapeExact(registry) && registry.shadowOnly &&
            !registry.runtimeInfluence &&
            PristineRegistryEntry(registry.lastEntry);
        if (!exactQuiescent)
        {
            return false;
        }
        if (registry.publicationSequence == 0ULL)
        {
            return registry.currentSession ==
                NC_PREPARED_QUEUE_SESSION_INVALID &&
                registry.lastObservedFeedbackSequence ==
                MOTION_FEEDBACK_SEQUENCE_INVALID;
        }
        if (registry.publicationSequence ==
            (std::numeric_limits<std::uint64_t>::max)())
        {
            return false;
        }
        return registry.currentSession ==
            NC_PREPARED_QUEUE_SESSION_INVALID ||
            RegistryReceiptTrusted(registry);
    }

    static bool RegistryValid(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            RegistryReceiptTrusted(registry) &&
            registry.ready &&
            !registry.permanentLockout &&
            registry.activeEntries <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    static bool ResumeClassificationExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        if (!cohort.resumeRequested)
        {
            return !cohort.resumeBeforeAcknowledge &&
                !cohort.resumeAfterAcknowledge;
        }
        return cohort.resumeBeforeAcknowledge !=
            cohort.resumeAfterAcknowledge;
    }

    static bool NonTerminalMemberExact(
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member) noexcept
    {
        const bool feedbackNonTerminal =
            member.lastFeedbackType == MotionFeedbackType::NONE ||
            member.lastFeedbackType == MotionFeedbackType::ACCEPTED ||
            member.lastFeedbackType == MotionFeedbackType::STARTED ||
            member.lastFeedbackType == MotionFeedbackType::PROGRESS ||
            member.lastFeedbackType == MotionFeedbackType::HELD ||
            member.lastFeedbackType == MotionFeedbackType::RESUMED;
        return LegalInitialMemberState(member.currentState) &&
            feedbackNonTerminal && !member.terminalObserved &&
            !member.terminalCompleted &&
            !member.terminalBeforeAcknowledge &&
            !member.terminalBeforeResume && member.terminalOrdinal == 0U;
    }

    static bool TerminalMemberExact(
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member,
        std::uint8_t ordinal) noexcept
    {
        if (!member.terminalObserved ||
            member.lastFeedbackSequence ==
            MOTION_FEEDBACK_SEQUENCE_INVALID ||
            member.terminalOrdinal != ordinal)
        {
            return false;
        }
        switch (member.lastFeedbackType)
        {
        case MotionFeedbackType::COMPLETED:
            return member.terminalCompleted && member.currentState ==
                NCOrdinaryG00InflightState::K63_COMPLETED;
        case MotionFeedbackType::REJECTED:
            return !member.terminalCompleted && member.currentState ==
                NCOrdinaryG00InflightState::K63_REJECTED;
        case MotionFeedbackType::CANCELLED:
            return !member.terminalCompleted && member.currentState ==
                NCOrdinaryG00InflightState::K63_CANCELLED;
        case MotionFeedbackType::ABORTED:
            return !member.terminalCompleted && member.currentState ==
                NCOrdinaryG00InflightState::K63_ABORTED;
        case MotionFeedbackType::FAULTED:
            return !member.terminalCompleted && member.currentState ==
                NCOrdinaryG00InflightState::K63_FAULTED;
        case MotionFeedbackType::NONE:
        case MotionFeedbackType::ACCEPTED:
        case MotionFeedbackType::STARTED:
        case MotionFeedbackType::PROGRESS:
        case MotionFeedbackType::HELD:
        case MotionFeedbackType::RESUMED:
        default:
            return false;
        }
    }

    static bool TerminalMembersExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        switch (cohort.terminalCount)
        {
        case 0U:
            return NonTerminalMemberExact(cohort.members[0]) &&
                NonTerminalMemberExact(cohort.members[1]);
        case 1U:
            return TerminalMemberExact(cohort.members[0], 1U) &&
                NonTerminalMemberExact(cohort.members[1]);
        case NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE:
            return TerminalMemberExact(cohort.members[0], 1U) &&
                TerminalMemberExact(cohort.members[1], 2U);
        default:
            return false;
        }
    }

    static bool ActiveProofExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        if (!cohort.candidate || !cohort.active ||
            !cohort.exactTwoEntryCohort || !cohort.boundaryCorrelated ||
            cohort.terminalCohortComplete || cohort.failed ||
            cohort.cancelled || !cohort.bounded || !cohort.shadowOnly ||
            cohort.runtimeInfluence || cohort.motionWrite ||
            !cohort.accountingValid ||
            !ResumeClassificationExact(cohort) ||
            !TerminalMembersExact(cohort) ||
            !cohort.terminalOrderValid ||
            cohort.memberCount !=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE ||
            cohort.terminalCount > 1U)
        {
            return false;
        }
        switch (cohort.phase)
        {
        case NCOrdinaryG00FeedHoldCohortPhase::K73_CAPTURED:
            return cohort.terminalCount == 0U &&
                !cohort.holdAcknowledged && !cohort.resumeRequested &&
                !cohort.resumeApplied && cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::K73_CAPTURED_EXACT;
        case NCOrdinaryG00FeedHoldCohortPhase::K73_HOLD_ACKNOWLEDGED:
            return cohort.holdAcknowledged && !cohort.resumeApplied &&
                cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_HOLD_ACKNOWLEDGED;
        case NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_REQUESTED:
            return cohort.resumeRequested && !cohort.resumeApplied &&
                ((cohort.resumeBeforeAcknowledge &&
                    !cohort.resumeAfterAcknowledge &&
                    cohort.decision ==
                    NCOrdinaryG00FeedHoldCohortDecision::
                    K73_RESUME_BEFORE_ACK) ||
                    (!cohort.resumeBeforeAcknowledge &&
                        cohort.resumeAfterAcknowledge &&
                        cohort.decision ==
                        NCOrdinaryG00FeedHoldCohortDecision::
                        K73_RESUME_AFTER_ACK));
        case NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_APPLIED:
            return cohort.terminalCount == 0U &&
                cohort.holdAcknowledged && cohort.resumeRequested &&
                cohort.resumeApplied && cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_APPLIED;
        case NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_PENDING:
            return cohort.terminalCount == 1U && cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_WAIT_SECOND_TERMINAL;
        default:
            return false;
        }
    }

    static bool ReleasedProofExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        return
            cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_COMPLETE &&
            cohort.decision ==
            NCOrdinaryG00FeedHoldCohortDecision::
            K73_ALL_COMPLETED_EXACT &&
            cohort.candidate && !cohort.active &&
            cohort.exactTwoEntryCohort && cohort.boundaryCorrelated &&
            cohort.gateCorrelated && cohort.gateSequence != 0ULL &&
            !cohort.failed && !cohort.cancelled &&
            cohort.holdAcknowledged && cohort.resumeRequested &&
            ResumeClassificationExact(cohort) && cohort.resumeApplied &&
            cohort.terminalOrderValid &&
            cohort.terminalCohortComplete && cohort.allMembersCompleted &&
            cohort.memberCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.terminalCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.bounded && cohort.shadowOnly &&
            !cohort.runtimeInfluence && !cohort.motionWrite &&
            cohort.accountingValid &&
            cohort.members[0].exact &&
            cohort.members[0].terminalObserved &&
            cohort.members[0].terminalCompleted &&
            cohort.members[0].currentState ==
            NCOrdinaryG00InflightState::K63_COMPLETED &&
            cohort.members[0].lastFeedbackSequence !=
            MOTION_FEEDBACK_SEQUENCE_INVALID &&
            cohort.members[0].lastFeedbackType ==
            MotionFeedbackType::COMPLETED &&
            cohort.members[0].terminalOrdinal == 1U &&
            cohort.members[1].exact &&
            cohort.members[1].terminalObserved &&
            cohort.members[1].terminalCompleted &&
            cohort.members[1].currentState ==
            NCOrdinaryG00InflightState::K63_COMPLETED &&
            cohort.members[1].lastFeedbackSequence !=
            MOTION_FEEDBACK_SEQUENCE_INVALID &&
            cohort.members[1].lastFeedbackType ==
            MotionFeedbackType::COMPLETED &&
            cohort.members[1].terminalOrdinal == 2U;
    }

    static bool CancelledProofExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        return cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_CANCELLED &&
            (cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::K73_CANCELLED ||
                cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::K73_SUPERSEDED) &&
            cohort.cancelled && !cohort.active && !cohort.failed &&
            cohort.candidate && cohort.exactTwoEntryCohort &&
            cohort.boundaryCorrelated && cohort.memberCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.bounded && cohort.shadowOnly &&
            !cohort.runtimeInfluence && !cohort.motionWrite &&
            cohort.accountingValid;
    }

    static bool PristineBypassMember(
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member) noexcept
    {
        return member.registrySequence == 0ULL &&
            member.session == NC_PREPARED_QUEUE_SESSION_INVALID &&
            member.entrySequence == NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            member.dispatchId == 0ULL &&
            member.commitSequence == NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            member.sourcePC == -1 && member.sourceLineNumber == 0 &&
            member.identity.epoch == MOTION_EXECUTION_EPOCH_INVALID &&
            member.identity.segmentId == MOTION_SEGMENT_ID_INVALID &&
            member.identity.sourceBlockId == MOTION_SOURCE_BLOCK_ID_INVALID &&
            member.identity.source == MotionCommandSource::UNKNOWN &&
            member.ownerLease.owner == MotionOwner::NONE &&
            member.ownerLease.generation ==
            MOTION_OWNER_GENERATION_INVALID &&
            member.captureState ==
            NCOrdinaryG00InflightState::K63_EMPTY &&
            member.currentState ==
            NCOrdinaryG00InflightState::K63_EMPTY &&
            member.lastFeedbackSequence ==
            MOTION_FEEDBACK_SEQUENCE_INVALID &&
            member.lastFeedbackType == MotionFeedbackType::NONE &&
            member.terminalOrdinal == 0U && !member.exact &&
            !member.heldObserved && !member.resumedObserved &&
            !member.terminalObserved &&
            !member.terminalBeforeAcknowledge &&
            !member.terminalBeforeResume && !member.terminalCompleted;
    }

    static bool ColdZeroRegistryBypass(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return cohort.activeEntriesAtCapture == 0U &&
            cohort.registryPublicationSequence == 0ULL &&
            RegistryBypassReceiptHealthy(registry) &&
            registry.publicationSequence == 0ULL;
    }

    static bool SafeNonExactBypass(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const bool bypassKind =
            (cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_BYPASS_NOT_PROGRAM && !cohort.candidate) ||
            (cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_BYPASS_NOT_TWO_ACTIVE && cohort.candidate &&
                cohort.activeEntriesAtCapture <
                NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE);
        const bool commonExact =
            cohort.publicationSequence != 0ULL &&
            cohort.cohortSequence != 0ULL && cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_BYPASSED &&
            bypassKind &&
            cohort.session == NC_PREPARED_QUEUE_SESSION_INVALID &&
            cohort.gateSequence == 0ULL && !cohort.active &&
            !cohort.exactTwoEntryCohort && !cohort.boundaryCorrelated &&
            !cohort.gateCorrelated && !cohort.failed &&
            !cohort.cancelled && cohort.memberCount == 0U &&
            cohort.terminalCount == 0U && !cohort.holdAcknowledged &&
            !cohort.resumeRequested && !cohort.resumeApplied &&
            !cohort.resumeBeforeAcknowledge &&
            !cohort.resumeAfterAcknowledge &&
            !cohort.terminalCohortComplete &&
            !cohort.allMembersCompleted && cohort.terminalOrderValid &&
            cohort.bounded && cohort.shadowOnly &&
            !cohort.runtimeInfluence && !cohort.motionWrite &&
            cohort.accountingValid &&
            PristineBypassMember(cohort.members[0]) &&
            PristineBypassMember(cohort.members[1]) &&
            cohort.activeEntriesAtCapture <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
        if (!commonExact)
        {
            return false;
        }
        if (ColdZeroRegistryBypass(cohort, registry))
        {
            return true;
        }
        return RegistryBypassReceiptHealthy(registry) &&
            cohort.registryPublicationSequence ==
            registry.publicationSequence &&
            cohort.activeEntriesAtCapture == registry.activeEntries;
    }

    bool GateCorrelationStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        if (!cohort.gateCorrelated)
        {
            return cohort.gateSequence == 0ULL &&
                m_boundGateSequence == 0ULL;
        }
        if (cohort.gateSequence == 0ULL)
        {
            return false;
        }
        if (m_boundGateSequence == 0ULL)
        {
            m_boundGateSequence = cohort.gateSequence;
            return true;
        }
        return cohort.gateSequence == m_boundGateSequence;
    }

    bool ProgressionStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        if (cohort.terminalCount < m_snapshot.terminalCount ||
            (m_snapshot.holdAcknowledged &&
                !cohort.holdAcknowledged) ||
            (m_snapshot.resumeRequested && !cohort.resumeRequested) ||
            (m_snapshot.resumeApplied && !cohort.resumeApplied) ||
            (!m_snapshot.terminalOrderValid &&
                cohort.terminalOrderValid) ||
            !TerminalMembersExact(cohort) ||
            !ResumeClassificationExact(cohort))
        {
            return false;
        }

        if (!cohort.resumeRequested)
        {
            return !m_resumeClassificationLatched;
        }
        if (!m_resumeClassificationLatched)
        {
            m_resumeClassificationLatched = true;
            m_resumeBeforeAcknowledge =
                cohort.resumeBeforeAcknowledge;
            return true;
        }
        return cohort.resumeBeforeAcknowledge ==
            m_resumeBeforeAcknowledge &&
            cohort.resumeAfterAcknowledge ==
            !m_resumeBeforeAcknowledge;
    }

    bool IdentityStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) const noexcept
    {
        if (!(
            cohort.cohortSequence == m_snapshot.cohortSequence &&
            cohort.boundarySequence == m_snapshot.boundarySequence &&
            cohort.session == m_snapshot.session &&
            cohort.executionEpoch == m_snapshot.executionEpoch &&
            cohort.owner == m_snapshot.owner &&
            cohort.ownerGeneration == m_snapshot.ownerGeneration &&
            cohort.activeEntriesAtCapture ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.memberCount == NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohort.exactTwoEntryCohort &&
            cohort.boundaryCorrelated &&
            cohort.registryPublicationSequence ==
            m_boundRegistryPublicationSequence))
        {
            return false;
        }
        for (std::size_t index = 0U;
            index < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++index)
        {
            const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
                cohort.members[index];
            const BoundMemberLineage& bound = m_boundMembers[index];
            if (!member.exact ||
                member.registrySequence != bound.registrySequence ||
                member.session != bound.session ||
                member.entrySequence != bound.entrySequence ||
                member.dispatchId != bound.dispatchId ||
                member.commitSequence != bound.commitSequence ||
                member.sourcePC != bound.sourcePC ||
                member.sourceLineNumber != bound.sourceLineNumber ||
                !SameIdentity(member.identity, bound.identity) ||
                !SameLease(member.ownerLease, bound.ownerLease) ||
                member.captureState != bound.captureState)
            {
                return false;
            }
        }
        return true;
    }

    static bool AccountingExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            cohort.accountingValid &&
            cohort.registryPublicationSequence != 0ULL &&
            cohort.registryPublicationSequence <=
            registry.publicationSequence &&
            cohort.terminalCount <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            registry.currentSession == cohort.session &&
            static_cast<std::uint32_t>(cohort.terminalCount) +
            registry.activeEntries ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    static bool CompletedProofExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            ReleasedProofExact(cohort) && registry.activeEntries == 0U;
    }

    void PublishSafeBypass(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        const NCPreparedQueueSession authoritySession =
            m_snapshot.session;
        const std::uint64_t authorityReceipt =
            m_snapshot.registryPublicationSequence;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.cohortSequence = cohort.cohortSequence;
        next.boundarySequence = cohort.boundarySequence;
        next.gateSequence = cohort.gateSequence;
        next.registryPublicationSequence =
            registry.publicationSequence > authorityReceipt
            ? registry.publicationSequence
            : authorityReceipt;
        next.session = registry.currentSession !=
            NC_PREPARED_QUEUE_SESSION_INVALID
            ? registry.currentSession
            : authoritySession;
        next.registryActiveEntries = registry.activeEntries;
        next.activeEntriesAtCapture = cohort.activeEntriesAtCapture;
        next.enabled = true;
        next.phase = NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED;
        next.decision =
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            BYPASS_NO_EXACT_COHORT;
        m_snapshot = next;
        m_boundRegistryPublicationSequence = 0ULL;
        m_boundGateSequence = 0ULL;
        m_resumeClassificationLatched = false;
        m_resumeBeforeAcknowledge = false;
        m_boundMembers = {};
        ++m_counters.bypassNoExactCohort;
        Publish();
    }

    void AdoptFallbackSession(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        AdoptLiveRegistryAuthority(registry);
        if (m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            m_snapshot.session = cohort.session;
        }
        m_snapshot.cohortSequence = cohort.cohortSequence;
        m_snapshot.boundarySequence = cohort.boundarySequence;
        m_snapshot.gateSequence = cohort.gateSequence;
        m_snapshot.activeEntriesAtCapture =
            cohort.activeEntriesAtCapture;
    }

    void AdoptLiveRegistryAuthority(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (registry.currentSession != NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            m_snapshot.session = registry.currentSession;
        }
        if (RegistryReceiptTrusted(registry) &&
            registry.publicationSequence >
            m_snapshot.registryPublicationSequence)
        {
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
        }
        m_snapshot.registryActiveEntries = registry.activeEntries;
    }

    void RecordDecisionRegistryReceipt(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (RegistryReceiptTrusted(registry) &&
            registry.publicationSequence >=
            m_snapshot.registryPublicationSequence)
        {
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
            m_snapshot.registryActiveEntries = registry.activeEntries;
        }
    }

    bool BypassAuthorityFresh(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) const noexcept
    {
        if (registry.currentSession == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return registry.publicationSequence >=
                m_snapshot.registryPublicationSequence;
        }
        if (m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return registry.publicationSequence >
                m_snapshot.registryPublicationSequence;
        }
        if (registry.currentSession == m_snapshot.session)
        {
            return registry.publicationSequence >=
                m_snapshot.registryPublicationSequence;
        }
        return registry.publicationSequence >
            m_snapshot.registryPublicationSequence;
    }

    bool PublishedBypassRegistrySafe(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (RegistryBypassReceiptHealthy(registry) &&
            BypassAuthorityFresh(registry))
        {
            const NCPreparedQueueSession nextSession =
                registry.currentSession !=
                NC_PREPARED_QUEUE_SESSION_INVALID
                ? registry.currentSession
                : m_snapshot.session;
            const std::uint64_t nextReceipt =
                registry.publicationSequence >
                m_snapshot.registryPublicationSequence
                ? registry.publicationSequence
                : m_snapshot.registryPublicationSequence;
            if (nextSession != m_snapshot.session ||
                nextReceipt != m_snapshot.registryPublicationSequence)
            {
                m_snapshot.session = nextSession;
                m_snapshot.registryPublicationSequence =
                    nextReceipt;
                m_snapshot.registryActiveEntries = registry.activeEntries;
                Publish();
            }
            return true;
        }

        ++m_counters.registryMismatches;
        if (registry.currentSession != NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            m_snapshot.session = registry.currentSession;
        }
        RecordDecisionRegistryReceipt(registry);
        EnterFallback(registry,
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            FALLBACK_REGISTRY,
            false);
        return false;
    }

    void ClassifyFailed(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        AdoptFallbackSession(cohort, registry);
        RecordDecisionRegistryReceipt(registry);
        const bool registryFailureExact =
            cohort.failed && cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_FAILED &&
            cohort.decision ==
            NCOrdinaryG00FeedHoldCohortDecision::
            K73_INVALID_REGISTRY;
        if (registryFailureExact)
        {
            ++m_counters.registryMismatches;
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                false);
            return;
        }
        ++m_counters.failedProofs;
        EnterFallback(registry,
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            FALLBACK_FAILED_PROOF,
            false);
    }

    void ClassifyNonExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (SafeNonExactBypass(cohort, registry))
        {
            if (!BypassAuthorityFresh(registry))
            {
                AdoptFallbackSession(cohort, registry);
                RecordDecisionRegistryReceipt(registry);
                ++m_counters.registryMismatches;
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_REGISTRY,
                    false);
                return;
            }
            PublishSafeBypass(cohort, registry);
            return;
        }
        AdoptFallbackSession(cohort, registry);
        RecordDecisionRegistryReceipt(registry);
        if (cohort.failed || cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_FAILED)
        {
            ClassifyFailed(cohort, registry);
            return;
        }
        if (cohort.cancelled || cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_CANCELLED)
        {
            if (CancelledProofExact(cohort))
            {
                ++m_counters.cancelledCohorts;
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_CANCELLED,
                    cohort.accountingValid && registry.accountingValid);
            }
            else
            {
                ++m_counters.failedProofs;
                EnterFallback(registry,
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    FALLBACK_FAILED_PROOF,
                    false);
            }
            return;
        }
        if (!cohort.accountingValid || !cohort.bounded ||
            !cohort.shadowOnly || cohort.runtimeInfluence ||
            cohort.motionWrite)
        {
            ++m_counters.accountingMismatches;
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_ACCOUNTING,
                false);
            return;
        }
        if (!RegistryValid(registry) ||
            cohort.registryPublicationSequence !=
            registry.publicationSequence ||
            cohort.activeEntriesAtCapture != registry.activeEntries)
        {
            ++m_counters.registryMismatches;
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                registry.accountingValid);
            return;
        }
        ++m_counters.failedProofs;
        EnterFallback(registry,
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            FALLBACK_FAILED_PROOF,
            false);
    }

    void Bind(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!RegistryValid(registry) ||
            !BypassAuthorityFresh(registry) ||
            registry.currentSession != cohort.session ||
            cohort.registryPublicationSequence !=
            registry.publicationSequence ||
            registry.activeEntries !=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE)
        {
            ++m_counters.registryMismatches;
            AdoptFallbackSession(cohort, registry);
            RecordDecisionRegistryReceipt(registry);
            m_snapshot.bound = false;
            EnterFallback(registry,
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                false);
            return;
        }

        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.cohortSequence = cohort.cohortSequence;
        next.boundarySequence = cohort.boundarySequence;
        next.gateSequence = cohort.gateSequence;
        next.registryPublicationSequence = registry.publicationSequence;
        next.session = cohort.session;
        next.executionEpoch = cohort.executionEpoch;
        next.owner = cohort.owner;
        next.ownerGeneration = cohort.ownerGeneration;
        next.activeEntriesAtCapture = cohort.activeEntriesAtCapture;
        next.registryActiveEntries = registry.activeEntries;
        next.memberCount = cohort.memberCount;
        next.terminalCount = cohort.terminalCount;
        next.enabled = true;
        next.exactCohort = true;
        next.bound = true;
        next.holdAcknowledged = cohort.holdAcknowledged;
        next.resumeRequested = cohort.resumeRequested;
        next.resumeApplied = cohort.resumeApplied;
        next.terminalOrderValid = cohort.terminalOrderValid;
        next.waiting = true;
        next.phase = NCOrdinaryG00FeedHoldCohortCutoverPhase::BOUND;
        next.decision =
            NCOrdinaryG00FeedHoldCohortCutoverDecision::COHORT_BOUND;
        m_snapshot = next;
        m_boundRegistryPublicationSequence =
            cohort.registryPublicationSequence;
        m_boundGateSequence = 0ULL;
        m_resumeClassificationLatched = false;
        m_resumeBeforeAcknowledge = false;
        for (std::size_t index = 0U;
            index < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++index)
        {
            const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
                cohort.members[index];
            BoundMemberLineage& bound = m_boundMembers[index];
            bound.registrySequence = member.registrySequence;
            bound.session = member.session;
            bound.entrySequence = member.entrySequence;
            bound.dispatchId = member.dispatchId;
            bound.commitSequence = member.commitSequence;
            bound.sourcePC = member.sourcePC;
            bound.sourceLineNumber = member.sourceLineNumber;
            bound.identity = member.identity;
            bound.ownerLease = member.ownerLease;
            bound.captureState = member.captureState;
        }
        ++m_counters.cohortsBound;
        Publish();
    }

    void EnterFallback(
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        NCOrdinaryG00FeedHoldCohortCutoverDecision decision,
        bool accountingValid) noexcept
    {
        AdoptLiveRegistryAuthority(registry);
        m_snapshot.waiting = false;
        m_snapshot.released = false;
        m_snapshot.fallbackLegacy = true;
        m_snapshot.sessionLockout = true;
        m_snapshot.accountingValid = accountingValid;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortCutoverPhase::LEGACY_FALLBACK;
        m_snapshot.decision = decision;
        Publish();
    }

    void ClearBinding(bool preservePublication) noexcept
    {
        const std::uint64_t publicationSequence = preservePublication
            ? m_snapshot.publicationSequence
            : 0ULL;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.enabled = m_enabled;
        m_snapshot = next;
        m_boundRegistryPublicationSequence = 0ULL;
        m_boundGateSequence = 0ULL;
        m_resumeClassificationLatched = false;
        m_resumeBeforeAcknowledge = false;
        m_boundMembers = {};
    }

    bool TryResetForChangedSession(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!(m_snapshot.bound || m_snapshot.sessionLockout ||
            m_snapshot.released) ||
            m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == m_snapshot.session)
        {
            return false;
        }
        if (!RegistryValid(registry) ||
            registry.revokedPendingEntries != 0U ||
            registry.publicationSequence <=
            m_snapshot.registryPublicationSequence)
        {
            return false;
        }
        ResetForSession(registry);
        return true;
    }

    void ResetForSession(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.registryPublicationSequence = registry.publicationSequence;
        next.session = registry.currentSession;
        next.enabled = m_enabled;
        next.phase = NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE;
        next.decision =
            NCOrdinaryG00FeedHoldCohortCutoverDecision::SESSION_RESET;
        m_snapshot = next;
        m_boundRegistryPublicationSequence = 0ULL;
        m_boundGateSequence = 0ULL;
        m_resumeClassificationLatched = false;
        m_resumeBeforeAcknowledge = false;
        m_boundMembers = {};
        ++m_counters.sessionResets;
        Publish();
    }

    void RecordInfluence() noexcept
    {
        ++m_counters.runtimeInfluence;
        m_snapshot.runtimeInfluence = true;
    }

    void Publish() noexcept
    {
        if (m_snapshot.publicationSequence ==
            (std::numeric_limits<std::uint64_t>::max)())
        {
            m_snapshot.publicationSequence = 1ULL;
        }
        else
        {
            ++m_snapshot.publicationSequence;
        }
    }

    bool m_enabled = true;
    std::uint64_t m_lastObservedCohortSequence = 0ULL;
    std::uint64_t m_lastObservedCohortPublicationSequence = 0ULL;
    // Immutable K.7.3 capture receipt.  The published snapshot receipt may
    // advance to the terminal-release/fallback decision receipt.
    std::uint64_t m_boundRegistryPublicationSequence = 0ULL;
    std::uint64_t m_boundGateSequence = 0ULL;
    bool m_resumeClassificationLatched = false;
    bool m_resumeBeforeAcknowledge = false;
    std::array<BoundMemberLineage,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> m_boundMembers{};
    NCOrdinaryG00FeedHoldCohortCutoverSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldCohortCutoverCounters m_counters{};
};
