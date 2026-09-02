#pragma once

#include "NCOrdinaryG00FeedHoldCohortShadow.h"

#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.4
// Two-Entry Feed-Hold / Resume Terminal Cohort Controlled Cutover
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

        if ((m_snapshot.bound || m_snapshot.sessionLockout ||
            m_snapshot.released) &&
            m_snapshot.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.currentSession != m_snapshot.session)
        {
            ResetForSession();
        }

        if (cohort.cohortSequence == 0ULL)
        {
            return;
        }
        if (m_lastObservedCohortSequence != 0ULL &&
            cohort.cohortSequence < m_lastObservedCohortSequence)
        {
            ++m_counters.staleObservations;
            return;
        }

        if (cohort.cohortSequence != m_lastObservedCohortSequence)
        {
            m_lastObservedCohortSequence = cohort.cohortSequence;
            if (m_snapshot.sessionLockout &&
                registry.currentSession == m_snapshot.session)
            {
                return;
            }
            if (!ExactCapture(cohort))
            {
                ClearBinding(true);
                ++m_counters.bypassNoExactCohort;
                m_snapshot.phase =
                    NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldCohortCutoverDecision::
                    BYPASS_NO_EXACT_COHORT;
                Publish();
                return;
            }
            Bind(cohort, registry);
            if (!m_snapshot.bound)
            {
                return;
            }
        }

        if (!m_snapshot.bound ||
            cohort.cohortSequence != m_snapshot.cohortSequence)
        {
            return;
        }
        if (m_snapshot.sessionLockout)
        {
            return;
        }
        if (m_snapshot.released)
        {
            // The two captured identities are already closed.  Any later
            // Registry entries belong to subsequent read-ahead work and must
            // never be folded back into the completed cohort accounting.
            return;
        }

        if (!IdentityStable(cohort))
        {
            ++m_counters.failedProofs;
            EnterFallback(
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (!RegistryValid(registry))
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_REGISTRY,
                false);
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

        if (!AccountingExact(cohort, registry))
        {
            ++m_counters.accountingMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_ACCOUNTING,
                false);
            return;
        }
        if (cohort.failed)
        {
            ++m_counters.failedProofs;
            EnterFallback(
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_FAILED_PROOF,
                false);
            return;
        }
        if (cohort.cancelled)
        {
            ++m_counters.cancelledCohorts;
            EnterFallback(
                NCOrdinaryG00FeedHoldCohortCutoverDecision::
                FALLBACK_CANCELLED,
                true);
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
                EnterFallback(
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
        if ((m_snapshot.bound || m_snapshot.sessionLockout ||
            m_snapshot.released) &&
            m_snapshot.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.currentSession != m_snapshot.session)
        {
            ResetForSession();
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
    static bool ExactCapture(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        return
            cohort.candidate &&
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
            !cohort.failed &&
            !cohort.cancelled &&
            cohort.accountingValid;
    }

    static bool RegistryValid(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            registry.ready &&
            !registry.permanentLockout &&
            registry.accountingValid &&
            registry.activeEntries <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    bool IdentityStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) const noexcept
    {
        return
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
            cohort.boundaryCorrelated;
    }

    static bool AccountingExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return
            cohort.accountingValid &&
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
            !cohort.active &&
            !cohort.failed &&
            !cohort.cancelled &&
            cohort.holdAcknowledged &&
            cohort.resumeRequested &&
            cohort.resumeApplied &&
            cohort.terminalOrderValid &&
            cohort.terminalCohortComplete &&
            cohort.allMembersCompleted &&
            cohort.terminalCount == NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            registry.activeEntries == 0U &&
            cohort.members[0].exact &&
            cohort.members[0].terminalObserved &&
            cohort.members[0].terminalCompleted &&
            cohort.members[0].terminalOrdinal == 1U &&
            cohort.members[1].exact &&
            cohort.members[1].terminalObserved &&
            cohort.members[1].terminalCompleted &&
            cohort.members[1].terminalOrdinal == 2U;
    }

    void Bind(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!RegistryValid(registry) ||
            registry.currentSession != cohort.session ||
            registry.activeEntries !=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE)
        {
            ++m_counters.registryMismatches;
            m_snapshot.session = cohort.session;
            m_snapshot.bound = true;
            EnterFallback(
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
        ++m_counters.cohortsBound;
        Publish();
    }

    void EnterFallback(
        NCOrdinaryG00FeedHoldCohortCutoverDecision decision,
        bool accountingValid) noexcept
    {
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
    }

    void ResetForSession() noexcept
    {
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldCohortCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.enabled = m_enabled;
        next.phase = NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE;
        next.decision =
            NCOrdinaryG00FeedHoldCohortCutoverDecision::SESSION_RESET;
        m_snapshot = next;
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
    NCOrdinaryG00FeedHoldCohortCutoverSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldCohortCutoverCounters m_counters{};
};
