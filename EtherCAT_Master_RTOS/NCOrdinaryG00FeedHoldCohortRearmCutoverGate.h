#pragma once

#include "NCOrdinaryG00FeedHoldCohortRearmShadow.h"

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.6
// Two-Generation Feed-Hold / Resume Cohort Re-arm Controlled Cutover
//
// K.7.5 proved, in shadow, that two consecutive exact K.7.4 cohorts in one
// immutable Prepared Queue session are separate generations and that the first
// generation is completely released before the second generation is bound.
//
// K.7.6 consumes only that published evidence at the existing K.7.1 admission
// seam.  The first generation remains controlled by K.7.4.  Once K.7.5 binds
// the second generation, K.7.6 prevents a later ordinary G00 from passing the
// re-arm boundary until K.7.5 publishes the exact SECOND_RELEASED /
// REARM_PROVEN proof.  A cross-generation identity, sequence, session,
// Registry, K.7.4 or accounting mismatch selects the existing legacy drain
// path for the rest of the Queue session.
//
// The gate is reversible and owns no Motion storage.  It does not submit or
// cancel Motion, write PDO, change NC state/PC/callback/Epoch/owner, allocate
// memory, lock, wait or sleep.
// =============================================================================

enum class NCOrdinaryG00FeedHoldRearmAdmissionResult : std::uint8_t
{
    BYPASS = 0,
    WAIT_REARM,
    ALLOW_READ_AHEAD,
    FALLBACK_LEGACY
};

enum class NCOrdinaryG00FeedHoldRearmCutoverPhase : std::uint8_t
{
    K76_IDLE = 0,
    K76_BYPASSED,
    K76_SECOND_BOUND,
    K76_WAIT_SECOND_RELEASE,
    K76_SECOND_RELEASED,
    K76_LEGACY_FALLBACK,
    K76_DISABLED
};

enum class NCOrdinaryG00FeedHoldRearmCutoverDecision : std::uint8_t
{
    K76_NONE = 0,
    K76_DISABLED,
    K76_BYPASS_BEFORE_SECOND_GENERATION,
    K76_SECOND_GENERATION_BOUND,
    K76_WAIT_K74_RELEASE,
    K76_WAIT_K75_REARM_PROOF,
    K76_ALLOW_READ_AHEAD,
    K76_FALLBACK_K75_FAILED,
    K76_FALLBACK_K74,
    K76_FALLBACK_SESSION,
    K76_FALLBACK_SEQUENCE,
    K76_FALLBACK_IDENTITY,
    K76_FALLBACK_REGISTRY,
    K76_FALLBACK_ACCOUNTING,
    K76_SESSION_RESET
};

struct NCOrdinaryG00FeedHoldRearmCutoverSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t rearmPublicationSequence = 0ULL;
    std::uint64_t cohortSequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;
    std::uint64_t gateSequence = 0ULL;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::array<MotionExecutionIdentity,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberIdentity{};
    std::array<NCPreparedEntrySequence,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberEntrySequence{};
    std::array<NCBlockDispatchId,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberDispatchId{};

    NCOrdinaryG00FeedHoldRearmCutoverPhase phase =
        NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_IDLE;
    NCOrdinaryG00FeedHoldRearmCutoverDecision decision =
        NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_NONE;

    std::uint8_t generationCount = 0U;
    std::uint8_t releaseCount = 0U;
    bool enabled = true;
    bool exactSecondGeneration = false;
    bool bound = false;
    bool waiting = false;
    bool released = false;
    bool fallbackLegacy = false;
    bool sessionLockout = false;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldRearmCutoverCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t secondGenerationsBound = 0ULL;
    std::uint64_t bypassDisabled = 0ULL;
    std::uint64_t bypassBeforeSecondGeneration = 0ULL;
    std::uint64_t staleObservations = 0ULL;
    std::uint64_t admissionChecks = 0ULL;
    std::uint64_t admissionBypasses = 0ULL;
    std::uint64_t waitK74Release = 0ULL;
    std::uint64_t waitK75RearmProof = 0ULL;
    std::uint64_t releases = 0ULL;
    std::uint64_t allowReadAhead = 0ULL;
    std::uint64_t fallbackLegacy = 0ULL;
    std::uint64_t k75Failures = 0ULL;
    std::uint64_t k74Failures = 0ULL;
    std::uint64_t sessionMismatches = 0ULL;
    std::uint64_t sequenceMismatches = 0ULL;
    std::uint64_t identityMismatches = 0ULL;
    std::uint64_t registryMismatches = 0ULL;
    std::uint64_t accountingMismatches = 0ULL;
    std::uint64_t sessionResets = 0ULL;
    std::uint64_t failures = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRearmCutoverSnapshot>::value,
    "K.7.6 re-arm cutover snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRearmCutoverCounters>::value,
    "K.7.6 re-arm cutover counters must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldRearmCutoverSnapshot) <= 256U,
    "K.7.6 re-arm cutover snapshot exceeded its fixed budget.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldRearmCutoverCounters) <= 256U,
    "K.7.6 re-arm cutover counters exceeded its fixed budget.");

class NCOrdinaryG00FeedHoldCohortRearmCutoverGate
{
public:
    NCOrdinaryG00FeedHoldCohortRearmCutoverGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept
    {
        if (m_enabled == enabled)
        {
            return;
        }
        m_enabled = enabled;
        m_lastObservedRearmPublicationSequence = 0ULL;
        m_snapshot.enabled = enabled;
        if (!enabled)
        {
            ClearBinding(false);
            m_snapshot.enabled = false;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_DISABLED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_DISABLED;
        }
        else
        {
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_IDLE;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_NONE;
        }
        Publish();
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    void Observe(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.observations;
        m_snapshot.enabled = m_enabled;
        if (!m_enabled)
        {
            ++m_counters.bypassDisabled;
            return;
        }

        ResetForChangedSession(registry);

        if (rearm.publicationSequence != 0ULL &&
            m_lastObservedRearmPublicationSequence != 0ULL &&
            rearm.publicationSequence <
            m_lastObservedRearmPublicationSequence)
        {
            ++m_counters.staleObservations;
            return;
        }
        if (rearm.publicationSequence != 0ULL)
        {
            m_lastObservedRearmPublicationSequence =
                rearm.publicationSequence;
        }

        if (m_snapshot.sessionLockout)
        {
            return;
        }
        if (m_snapshot.released)
        {
            // K.7.6 authorizes exactly the first same-session re-arm seam.
            // Later cohorts remain independently controlled by K.7.4 and
            // must not be folded back into this completed two-generation
            // proof.
            return;
        }

        if (rearm.failed)
        {
            AdoptFallbackSession(rearm.session, registry.currentSession);
            ++m_counters.k75Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_K75_FAILED,
                rearm.accountingValid);
            return;
        }

        if (!rearm.accountingValid || !registry.accountingValid)
        {
            AdoptFallbackSession(rearm.session, registry.currentSession);
            ++m_counters.accountingMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_ACCOUNTING,
                false);
            return;
        }

        if (rearm.generationCount <
            NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS ||
            !rearm.generations[1].captured)
        {
            if (!m_snapshot.bound &&
                m_snapshot.phase !=
                NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_BYPASSED)
            {
                m_snapshot.phase =
                    NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_BYPASSED;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldRearmCutoverDecision::
                    K76_BYPASS_BEFORE_SECOND_GENERATION;
                ++m_counters.bypassBeforeSecondGeneration;
                Publish();
            }
            return;
        }

        if (!m_snapshot.bound)
        {
            if (!ExactSecondGenerationCapture(
                rearm,
                cohortCutover,
                registry))
            {
                ClassifyCaptureFailure(rearm, cohortCutover, registry);
                return;
            }
            BindSecondGeneration(rearm);
        }

        if (!BoundIdentityStable(rearm))
        {
            ++m_counters.identityMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_IDENTITY,
                true);
            return;
        }
        if (!BoundSequenceStable(rearm, cohortCutover))
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_SEQUENCE,
                true);
            return;
        }
        if (!RegistryValid(registry) ||
            registry.currentSession != m_snapshot.session)
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_REGISTRY,
                registry.accountingValid);
            return;
        }
        if (cohortCutover.fallbackLegacy ||
            cohortCutover.sessionLockout)
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_K74,
                cohortCutover.accountingValid);
            return;
        }

        m_snapshot.rearmPublicationSequence = rearm.publicationSequence;
        m_snapshot.generationCount = rearm.generationCount;
        m_snapshot.releaseCount = rearm.releaseCount;
        m_snapshot.gateSequence = rearm.generations[1].gateSequence;

        if (rearm.rearmProven)
        {
            if (!ReleasedProofExact(rearm, cohortCutover, registry))
            {
                ++m_counters.accountingMismatches;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRearmCutoverDecision::
                    K76_FALLBACK_ACCOUNTING,
                    false);
                return;
            }
            if (!m_snapshot.released)
            {
                m_snapshot.waiting = false;
                m_snapshot.released = true;
                m_snapshot.phase =
                    NCOrdinaryG00FeedHoldRearmCutoverPhase::
                    K76_SECOND_RELEASED;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldRearmCutoverDecision::
                    K76_ALLOW_READ_AHEAD;
                ++m_counters.releases;
                Publish();
            }
            return;
        }

        m_snapshot.waiting = true;
        m_snapshot.released = false;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRearmCutoverPhase::
            K76_WAIT_SECOND_RELEASE;
        const NCOrdinaryG00FeedHoldRearmCutoverDecision nextDecision =
            cohortCutover.released
            ? NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_WAIT_K75_REARM_PROOF
            : NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_WAIT_K74_RELEASE;
        if (m_snapshot.decision != nextDecision)
        {
            m_snapshot.decision = nextDecision;
            Publish();
        }
    }

    NCOrdinaryG00FeedHoldRearmAdmissionResult EvaluateAdmission(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.admissionChecks;
        if (!m_enabled)
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldRearmAdmissionResult::BYPASS;
        }

        ResetForChangedSession(registry);
        if (m_snapshot.sessionLockout || m_snapshot.fallbackLegacy)
        {
            ++m_counters.fallbackLegacy;
            const bool stateChanged = !m_snapshot.runtimeInfluence;
            RecordInfluence();
            if (stateChanged)
            {
                Publish();
            }
            return
                NCOrdinaryG00FeedHoldRearmAdmissionResult::FALLBACK_LEGACY;
        }
        if (!m_snapshot.bound)
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldRearmAdmissionResult::BYPASS;
        }
        if (m_snapshot.released)
        {
            ++m_counters.allowReadAhead;
            const bool stateChanged =
                m_snapshot.decision !=
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_ALLOW_READ_AHEAD ||
                !m_snapshot.runtimeInfluence;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_ALLOW_READ_AHEAD;
            RecordInfluence();
            if (stateChanged)
            {
                Publish();
            }
            return
                NCOrdinaryG00FeedHoldRearmAdmissionResult::ALLOW_READ_AHEAD;
        }

        NCOrdinaryG00FeedHoldRearmCutoverDecision nextDecision =
            NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_WAIT_K74_RELEASE;
        if (m_snapshot.decision ==
            NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_WAIT_K75_REARM_PROOF)
        {
            ++m_counters.waitK75RearmProof;
            nextDecision =
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_WAIT_K75_REARM_PROOF;
        }
        else
        {
            ++m_counters.waitK74Release;
        }
        const bool stateChanged =
            m_snapshot.decision != nextDecision ||
            m_snapshot.phase !=
            NCOrdinaryG00FeedHoldRearmCutoverPhase::
            K76_WAIT_SECOND_RELEASE ||
            !m_snapshot.waiting ||
            !m_snapshot.runtimeInfluence;
        m_snapshot.decision = nextDecision;
        m_snapshot.waiting = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRearmCutoverPhase::
            K76_WAIT_SECOND_RELEASE;
        RecordInfluence();
        if (stateChanged)
        {
            Publish();
        }
        return NCOrdinaryG00FeedHoldRearmAdmissionResult::WAIT_REARM;
    }

    NCOrdinaryG00FeedHoldRearmCutoverSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldRearmCutoverCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static bool SameIdentity(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
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

    static bool GenerationMembersExact(
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& generation)
        noexcept
    {
        if (generation.memberCount !=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE ||
            !generation.exact)
        {
            return false;
        }
        for (std::size_t index = 0U;
            index < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++index)
        {
            if (!generation.memberIdentity[index].IsAssigned() ||
                generation.memberEntrySequence[index] ==
                NC_PREPARED_ENTRY_SEQUENCE_INVALID ||
                generation.memberDispatchId[index] ==
                NC_BLOCK_DISPATCH_ID_INVALID)
            {
                return false;
            }
        }
        return !SameIdentity(
            generation.memberIdentity[0],
            generation.memberIdentity[1]);
    }

    static bool ExactSecondGenerationCapture(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& first =
            rearm.generations[0];
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& second =
            rearm.generations[1];
        if (rearm.generationCount !=
            NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS ||
            rearm.releaseCount != 1U ||
            rearm.phase !=
            NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_BOUND ||
            rearm.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            !rearm.sameSession ||
            !rearm.sameExecutionLease ||
            !rearm.cohortSequenceMonotonic ||
            !rearm.boundarySequenceMonotonic ||
            !rearm.memberIsolation ||
            !rearm.noOverlap ||
            rearm.failed ||
            !rearm.bounded ||
            !rearm.shadowOnly ||
            rearm.runtimeInfluence ||
            rearm.motionWrite ||
            !rearm.accountingValid ||
            !first.captured || !first.released || !first.exact ||
            !first.allMembersCompleted ||
            !first.registryEmptyAtRelease ||
            !second.captured || second.released ||
            !GenerationMembersExact(first) ||
            !GenerationMembersExact(second) ||
            first.session != rearm.session ||
            second.session != rearm.session ||
            second.executionEpoch != first.executionEpoch ||
            second.owner != first.owner ||
            second.ownerGeneration != first.ownerGeneration ||
            second.cohortSequence <= first.cohortSequence ||
            second.boundarySequence <= first.boundarySequence)
        {
            return false;
        }
        for (std::size_t firstIndex = 0U;
            firstIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++firstIndex)
        {
            for (std::size_t secondIndex = 0U;
                secondIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
                ++secondIndex)
            {
                if (SameIdentity(
                    first.memberIdentity[firstIndex],
                    second.memberIdentity[secondIndex]))
                {
                    return false;
                }
            }
        }
        return
            cohortCutover.enabled &&
            cohortCutover.exactCohort &&
            cohortCutover.bound &&
            !cohortCutover.released &&
            !cohortCutover.fallbackLegacy &&
            !cohortCutover.sessionLockout &&
            cohortCutover.cohortSequence == second.cohortSequence &&
            cohortCutover.boundarySequence == second.boundarySequence &&
            cohortCutover.session == second.session &&
            cohortCutover.executionEpoch == second.executionEpoch &&
            cohortCutover.owner == second.owner &&
            cohortCutover.ownerGeneration == second.ownerGeneration &&
            cohortCutover.memberCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohortCutover.accountingValid &&
            RegistryValid(registry) &&
            registry.currentSession == second.session &&
            registry.activeEntries ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    bool BoundIdentityStable(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm) const noexcept
    {
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& second =
            rearm.generations[1];
        if (!GenerationMembersExact(second) ||
            rearm.session != m_snapshot.session ||
            second.session != m_snapshot.session ||
            second.executionEpoch != m_snapshot.executionEpoch ||
            second.owner != m_snapshot.owner ||
            second.ownerGeneration != m_snapshot.ownerGeneration)
        {
            return false;
        }
        for (std::size_t index = 0U;
            index < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++index)
        {
            if (!SameIdentity(
                second.memberIdentity[index],
                m_snapshot.memberIdentity[index]) ||
                second.memberEntrySequence[index] !=
                m_snapshot.memberEntrySequence[index] ||
                second.memberDispatchId[index] !=
                m_snapshot.memberDispatchId[index])
            {
                return false;
            }
        }
        return true;
    }

    bool BoundSequenceStable(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover)
        const noexcept
    {
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& second =
            rearm.generations[1];
        return
            second.cohortSequence == m_snapshot.cohortSequence &&
            second.boundarySequence == m_snapshot.boundarySequence &&
            cohortCutover.cohortSequence == m_snapshot.cohortSequence &&
            cohortCutover.boundarySequence == m_snapshot.boundarySequence &&
            cohortCutover.session == m_snapshot.session &&
            cohortCutover.executionEpoch == m_snapshot.executionEpoch &&
            cohortCutover.owner == m_snapshot.owner &&
            cohortCutover.ownerGeneration == m_snapshot.ownerGeneration;
    }

    static bool ReleasedProofExact(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& first =
            rearm.generations[0];
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& second =
            rearm.generations[1];
        return
            rearm.phase ==
            NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_RELEASED &&
            rearm.decision ==
            NCOrdinaryG00FeedHoldCohortRearmDecision::K75_REARM_PROVEN &&
            rearm.generationCount ==
            NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS &&
            rearm.releaseCount ==
            NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS &&
            rearm.rearmProven &&
            !rearm.failed &&
            rearm.sameSession &&
            rearm.sameExecutionLease &&
            rearm.cohortSequenceMonotonic &&
            rearm.boundarySequenceMonotonic &&
            rearm.gateSequenceMonotonic &&
            rearm.memberIsolation &&
            rearm.noOverlap &&
            rearm.accountingValid &&
            first.released && first.exact &&
            first.terminalOrderValid && first.allMembersCompleted &&
            first.registryEmptyAtRelease &&
            second.released && second.exact &&
            second.terminalOrderValid && second.allMembersCompleted &&
            second.registryEmptyAtRelease &&
            second.gateSequence > first.gateSequence &&
            cohortCutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED &&
            cohortCutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            ALLOW_READ_AHEAD &&
            cohortCutover.cohortSequence == second.cohortSequence &&
            cohortCutover.boundarySequence == second.boundarySequence &&
            cohortCutover.gateSequence == second.gateSequence &&
            cohortCutover.session == second.session &&
            cohortCutover.executionEpoch == second.executionEpoch &&
            cohortCutover.owner == second.owner &&
            cohortCutover.ownerGeneration == second.ownerGeneration &&
            cohortCutover.terminalCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cohortCutover.released &&
            !cohortCutover.fallbackLegacy &&
            !cohortCutover.sessionLockout &&
            cohortCutover.accountingValid &&
            RegistryValid(registry) &&
            registry.currentSession == second.session &&
            registry.activeEntries == 0U;
    }

    void BindSecondGeneration(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm) noexcept
    {
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& second =
            rearm.generations[1];
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldRearmCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.rearmPublicationSequence = rearm.publicationSequence;
        next.cohortSequence = second.cohortSequence;
        next.boundarySequence = second.boundarySequence;
        next.gateSequence = second.gateSequence;
        next.session = second.session;
        next.executionEpoch = second.executionEpoch;
        next.owner = second.owner;
        next.ownerGeneration = second.ownerGeneration;
        next.memberIdentity = second.memberIdentity;
        next.memberEntrySequence = second.memberEntrySequence;
        next.memberDispatchId = second.memberDispatchId;
        next.generationCount = rearm.generationCount;
        next.releaseCount = rearm.releaseCount;
        next.enabled = true;
        next.exactSecondGeneration = true;
        next.bound = true;
        next.waiting = true;
        next.phase =
            NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_SECOND_BOUND;
        next.decision =
            NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_SECOND_GENERATION_BOUND;
        m_snapshot = next;
        ++m_counters.secondGenerationsBound;
        Publish();
    }

    void ClassifyCaptureFailure(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& rearm,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        m_snapshot.session = rearm.session;
        m_snapshot.bound = true;
        if (rearm.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession != rearm.session)
        {
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_SESSION,
                rearm.accountingValid && registry.accountingValid);
        }
        else if (!RegistryValid(registry))
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_REGISTRY,
                registry.accountingValid);
        }
        else if (cohortCutover.fallbackLegacy ||
            cohortCutover.sessionLockout ||
            !cohortCutover.accountingValid)
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_K74,
                cohortCutover.accountingValid);
        }
        else if (!rearm.cohortSequenceMonotonic ||
            !rearm.boundarySequenceMonotonic ||
            !rearm.gateSequenceMonotonic)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_SEQUENCE,
                rearm.accountingValid);
        }
        else
        {
            ++m_counters.identityMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRearmCutoverDecision::
                K76_FALLBACK_IDENTITY,
                rearm.accountingValid);
        }
    }

    void EnterFallback(
        NCOrdinaryG00FeedHoldRearmCutoverDecision decision,
        bool accountingValid) noexcept
    {
        if (!m_snapshot.sessionLockout)
        {
            ++m_counters.failures;
        }
        m_snapshot.waiting = false;
        m_snapshot.released = false;
        m_snapshot.fallbackLegacy = true;
        m_snapshot.sessionLockout = true;
        m_snapshot.accountingValid = accountingValid;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_LEGACY_FALLBACK;
        m_snapshot.decision = decision;
        Publish();
    }

    void AdoptFallbackSession(
        NCPreparedQueueSession rearmSession,
        NCPreparedQueueSession registrySession) noexcept
    {
        if (m_snapshot.session != NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }
        m_snapshot.session =
            rearmSession != NC_PREPARED_QUEUE_SESSION_INVALID
            ? rearmSession
            : registrySession;
    }

    void ResetForChangedSession(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!(m_snapshot.bound || m_snapshot.sessionLockout ||
            m_snapshot.released) ||
            m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == m_snapshot.session)
        {
            return;
        }
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldRearmCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.enabled = m_enabled;
        next.phase = NCOrdinaryG00FeedHoldRearmCutoverPhase::K76_IDLE;
        next.decision =
            NCOrdinaryG00FeedHoldRearmCutoverDecision::K76_SESSION_RESET;
        m_snapshot = next;
        m_lastObservedRearmPublicationSequence = 0ULL;
        ++m_counters.sessionResets;
        Publish();
    }

    void ClearBinding(bool preservePublication) noexcept
    {
        const std::uint64_t publicationSequence = preservePublication
            ? m_snapshot.publicationSequence
            : 0ULL;
        NCOrdinaryG00FeedHoldRearmCutoverSnapshot next{};
        next.publicationSequence = publicationSequence;
        next.enabled = m_enabled;
        m_snapshot = next;
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
    std::uint64_t m_lastObservedRearmPublicationSequence = 0ULL;
    NCOrdinaryG00FeedHoldRearmCutoverSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldRearmCutoverCounters m_counters{};
};
