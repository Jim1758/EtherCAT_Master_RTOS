#pragma once

#include "NCOrdinaryG00FeedHoldCohortCutoverGate.h"

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.5
// Same-Session Repeated Feed-Hold / Resume Cohort Re-arm Shadow
//
// K.7.4 proved one exact two-entry Feed-Hold cohort can control the ordinary
// G00 admission seam until both original members reach ordered COMPLETED
// terminals.  K.7.5 observes two consecutive K.7.4 cohorts in one immutable
// Prepared Queue session and proves that the second cohort is a fresh
// generation:
//
//   * generation 0 is fully released before generation 1 is captured;
//   * cohort/boundary/gate sequences advance monotonically;
//   * both generations keep the same Queue session and execution lease;
//   * no Motion identity from generation 0 is reused by generation 1;
//   * each generation closes with two ordered COMPLETED terminals and an
//     empty Registry.
//
// This class is observation-only.  It does not affect admission, submit or
// cancel Motion, write PDO, change NC state/PC/callback/Epoch/owner, allocate
// memory, lock, wait or sleep.
// =============================================================================

constexpr std::size_t NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS = 2U;

enum class NCOrdinaryG00FeedHoldCohortRearmPhase : std::uint8_t
{
    K75_IDLE = 0,
    K75_FIRST_BOUND,
    K75_FIRST_RELEASED,
    K75_SECOND_BOUND,
    K75_SECOND_RELEASED,
    K75_FAILED
};

enum class NCOrdinaryG00FeedHoldCohortRearmDecision : std::uint8_t
{
    K75_NONE = 0,
    K75_FIRST_GENERATION_BOUND,
    K75_WAIT_FIRST_RELEASE,
    K75_FIRST_GENERATION_RELEASED,
    K75_SECOND_GENERATION_BOUND,
    K75_WAIT_SECOND_RELEASE,
    K75_REARM_PROVEN,
    K75_EARLY_REARM,
    K75_SESSION_MISMATCH,
    K75_COHORT_SEQUENCE_MISMATCH,
    K75_BOUNDARY_SEQUENCE_MISMATCH,
    K75_GATE_SEQUENCE_MISMATCH,
    K75_MEMBER_IDENTITY_REUSE,
    K75_REGISTRY_MISMATCH,
    K75_PROOF_MISMATCH,
    K75_FALLBACK_OBSERVED,
    K75_ACCOUNTING_MISMATCH
};

struct NCOrdinaryG00FeedHoldCohortGenerationSnapshot
{
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

    std::uint8_t memberCount = 0U;
    std::uint8_t terminalCount = 0U;
    bool captured = false;
    bool released = false;
    bool exact = false;
    bool terminalOrderValid = true;
    bool allMembersCompleted = false;
    bool registryEmptyAtRelease = false;
};

struct NCOrdinaryG00FeedHoldCohortRearmSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCOrdinaryG00FeedHoldCohortRearmPhase phase =
        NCOrdinaryG00FeedHoldCohortRearmPhase::K75_IDLE;
    NCOrdinaryG00FeedHoldCohortRearmDecision decision =
        NCOrdinaryG00FeedHoldCohortRearmDecision::K75_NONE;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    std::array<NCOrdinaryG00FeedHoldCohortGenerationSnapshot,
        NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS> generations{};

    std::uint8_t generationCount = 0U;
    std::uint8_t releaseCount = 0U;
    bool sameSession = true;
    bool sameExecutionLease = true;
    bool cohortSequenceMonotonic = true;
    bool boundarySequenceMonotonic = true;
    bool gateSequenceMonotonic = true;
    bool memberIsolation = true;
    bool noOverlap = true;
    bool rearmProven = false;
    bool failed = false;
    bool bounded = true;
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldCohortRearmCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t cohortEdges = 0ULL;
    std::uint64_t nonExactBypasses = 0ULL;
    std::uint64_t generationsCaptured = 0ULL;
    std::uint64_t firstBound = 0ULL;
    std::uint64_t firstReleased = 0ULL;
    std::uint64_t secondBound = 0ULL;
    std::uint64_t secondReleased = 0ULL;
    std::uint64_t stableObservations = 0ULL;
    std::uint64_t staleObservations = 0ULL;
    std::uint64_t earlyRearm = 0ULL;
    std::uint64_t sessionMismatches = 0ULL;
    std::uint64_t executionLeaseMismatches = 0ULL;
    std::uint64_t cohortSequenceMismatches = 0ULL;
    std::uint64_t boundarySequenceMismatches = 0ULL;
    std::uint64_t gateSequenceMismatches = 0ULL;
    std::uint64_t memberIdentityReuse = 0ULL;
    std::uint64_t registryMismatches = 0ULL;
    std::uint64_t proofMismatches = 0ULL;
    std::uint64_t fallbackObserved = 0ULL;
    std::uint64_t accountingMismatches = 0ULL;
    std::uint64_t sessionResets = 0ULL;
    std::uint64_t failures = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortGenerationSnapshot>::value,
    "K.7.5 generation snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortRearmSnapshot>::value,
    "K.7.5 re-arm snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortRearmCounters>::value,
    "K.7.5 re-arm counters must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortRearmSnapshot) <= 512U,
    "K.7.5 re-arm snapshot exceeded its fixed diagnostic budget.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortRearmCounters) <= 256U,
    "K.7.5 re-arm counters exceeded their fixed diagnostic budget.");

class NCOrdinaryG00FeedHoldCohortRearmShadow
{
public:
    NCOrdinaryG00FeedHoldCohortRearmShadow() noexcept = default;

    void Observe(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.observations;

        // NC-0.2K.7.6: K.7.5 was originally a one-session qualification
        // observer.  The controlled consumer must be able to qualify the
        // same two-generation seam again after a new Prepared Queue session
        // starts.  Reset only on an exact nonzero Registry session edge;
        // ordinary terminal drain inside the current session is untouched.
        if (m_snapshot.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.currentSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.currentSession != m_snapshot.session)
        {
            ResetForSession();
        }

        if (m_snapshot.failed || m_snapshot.rearmProven)
        {
            ++m_counters.stableObservations;
            return;
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

        const bool newCohort =
            cohort.cohortSequence != m_lastObservedCohortSequence;
        if (newCohort)
        {
            ++m_counters.cohortEdges;
            if (m_snapshot.generationCount != 0U &&
                !m_snapshot.generations[
                    m_snapshot.generationCount - 1U].released)
            {
                ++m_counters.earlyRearm;
                m_snapshot.noOverlap = false;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_EARLY_REARM,
                    true);
                return;
            }
                    m_lastObservedCohortSequence = cohort.cohortSequence;

                    if (!ExactBoundCapture(cohort, cutover, registry))
                    {
                        ++m_counters.nonExactBypasses;
                        return;
                    }

                    if (m_snapshot.generationCount >=
                        NC_ORDINARY_G00_FEED_HOLD_REARM_GENERATIONS)
                    {
                        ++m_counters.stableObservations;
                        return;
                    }

                    if (!CaptureGeneration(cohort, cutover))
                    {
                        return;
                    }
        }
        else
        {
            ++m_counters.stableObservations;
        }

        if (m_snapshot.generationCount == 0U)
        {
            return;
        }

        const std::size_t index =
            static_cast<std::size_t>(m_snapshot.generationCount - 1U);
        NCOrdinaryG00FeedHoldCohortGenerationSnapshot& generation =
            m_snapshot.generations[index];
        if (cohort.cohortSequence != generation.cohortSequence ||
            cutover.cohortSequence != generation.cohortSequence)
        {
            return;
        }

        if (cohort.failed || cohort.cancelled ||
            cutover.fallbackLegacy || cutover.sessionLockout)
        {
            ++m_counters.fallbackObserved;
            Fail(
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_FALLBACK_OBSERVED,
                cohort.accountingValid && cutover.accountingValid);
            return;
        }

        if (!cohort.accountingValid || !cutover.accountingValid ||
            !registry.accountingValid)
        {
            ++m_counters.accountingMismatches;
            Fail(
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_ACCOUNTING_MISMATCH,
                false);
            return;
        }

        if (cutover.released && cohort.terminalCohortComplete)
        {
            ReleaseGeneration(index, cohort, cutover, registry);
            return;
        }

        const NCOrdinaryG00FeedHoldCohortRearmDecision waitDecision =
            index == 0U
            ? NCOrdinaryG00FeedHoldCohortRearmDecision::
            K75_WAIT_FIRST_RELEASE
            : NCOrdinaryG00FeedHoldCohortRearmDecision::
            K75_WAIT_SECOND_RELEASE;
        if (m_snapshot.decision != waitDecision)
        {
            m_snapshot.decision = waitDecision;
            Publish();
        }
    }

    NCOrdinaryG00FeedHoldCohortRearmSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldCohortRearmCounters GetCounters() const noexcept
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

    static bool ExactBoundCapture(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
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
            !cohort.failed &&
            !cohort.cancelled &&
            cohort.accountingValid &&
            cutover.enabled &&
            cutover.exactCohort &&
            cutover.bound &&
            !cutover.released &&
            !cutover.fallbackLegacy &&
            !cutover.sessionLockout &&
            cutover.cohortSequence == cohort.cohortSequence &&
            cutover.boundarySequence == cohort.boundarySequence &&
            cutover.session == cohort.session &&
            cutover.executionEpoch == cohort.executionEpoch &&
            cutover.activeEntriesAtCapture ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cutover.memberCount == NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cutover.accountingValid &&
            registry.ready &&
            !registry.permanentLockout &&
            registry.currentSession == cohort.session &&
            registry.activeEntries ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            registry.accountingValid;
    }

    bool CaptureGeneration(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover) noexcept
    {
        const std::size_t index =
            static_cast<std::size_t>(m_snapshot.generationCount);

        if (index != 0U)
        {
            const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& previous =
                m_snapshot.generations[index - 1U];
            if (cohort.session != m_snapshot.session)
            {
                ++m_counters.sessionMismatches;
                m_snapshot.sameSession = false;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_SESSION_MISMATCH,
                    true);
                return false;
            }
            if (cohort.executionEpoch != previous.executionEpoch ||
                cohort.owner != previous.owner ||
                cohort.ownerGeneration != previous.ownerGeneration)
            {
                ++m_counters.executionLeaseMismatches;
                m_snapshot.sameExecutionLease = false;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_PROOF_MISMATCH,
                    true);
                return false;
            }
            if (cohort.cohortSequence <= previous.cohortSequence)
            {
                ++m_counters.cohortSequenceMismatches;
                m_snapshot.cohortSequenceMonotonic = false;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_COHORT_SEQUENCE_MISMATCH,
                    true);
                return false;
            }
            if (cohort.boundarySequence <= previous.boundarySequence)
            {
                ++m_counters.boundarySequenceMismatches;
                m_snapshot.boundarySequenceMonotonic = false;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_BOUNDARY_SEQUENCE_MISMATCH,
                    true);
                return false;
            }
        }

        NCOrdinaryG00FeedHoldCohortGenerationSnapshot generation{};
        generation.cohortSequence = cohort.cohortSequence;
        generation.boundarySequence = cohort.boundarySequence;
        generation.gateSequence = cutover.gateSequence;
        generation.session = cohort.session;
        generation.executionEpoch = cohort.executionEpoch;
        generation.owner = cohort.owner;
        generation.ownerGeneration = cohort.ownerGeneration;
        generation.memberCount = cohort.memberCount;
        generation.terminalCount = cohort.terminalCount;
        generation.captured = true;
        generation.exact = true;

        for (std::size_t memberIndex = 0U;
            memberIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++memberIndex)
        {
            const NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
                cohort.members[memberIndex];
            if (!member.exact || !member.identity.IsAssigned() ||
                member.entrySequence == NC_PREPARED_ENTRY_SEQUENCE_INVALID ||
                member.dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
            {
                ++m_counters.proofMismatches;
                Fail(
                    NCOrdinaryG00FeedHoldCohortRearmDecision::
                    K75_PROOF_MISMATCH,
                    true);
                return false;
            }
            generation.memberIdentity[memberIndex] = member.identity;
            generation.memberEntrySequence[memberIndex] =
                member.entrySequence;
            generation.memberDispatchId[memberIndex] = member.dispatchId;
        }

        if (SameIdentity(
            generation.memberIdentity[0],
            generation.memberIdentity[1]))
        {
            ++m_counters.memberIdentityReuse;
            m_snapshot.memberIsolation = false;
            Fail(
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_MEMBER_IDENTITY_REUSE,
                true);
            return false;
        }

        if (index != 0U)
        {
            const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& previous =
                m_snapshot.generations[index - 1U];
            for (std::size_t previousIndex = 0U;
                previousIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
                ++previousIndex)
            {
                for (std::size_t memberIndex = 0U;
                    memberIndex < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
                    ++memberIndex)
                {
                    if (SameIdentity(
                        previous.memberIdentity[previousIndex],
                        generation.memberIdentity[memberIndex]))
                    {
                        ++m_counters.memberIdentityReuse;
                        m_snapshot.memberIsolation = false;
                        Fail(
                            NCOrdinaryG00FeedHoldCohortRearmDecision::
                            K75_MEMBER_IDENTITY_REUSE,
                            true);
                        return false;
                    }
                }
            }
        }

        m_snapshot.generations[index] = generation;
        if (index == 0U)
        {
            m_snapshot.session = cohort.session;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FIRST_BOUND;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_FIRST_GENERATION_BOUND;
            ++m_counters.firstBound;
        }
        else
        {
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_BOUND;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_SECOND_GENERATION_BOUND;
            ++m_counters.secondBound;
        }
        ++m_snapshot.generationCount;
        ++m_counters.generationsCaptured;
        Publish();
        return true;
    }

    void ReleaseGeneration(
        std::size_t index,
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        NCOrdinaryG00FeedHoldCohortGenerationSnapshot& generation =
            m_snapshot.generations[index];
        if (generation.released)
        {
            return;
        }

        const bool releaseExact =
            !cohort.active &&
            !cohort.failed &&
            !cohort.cancelled &&
            cohort.holdAcknowledged &&
            cohort.resumeRequested &&
            cohort.resumeApplied &&
            cohort.terminalOrderValid &&
            cohort.terminalCohortComplete &&
            cohort.allMembersCompleted &&
            cohort.terminalCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED &&
            cutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            ALLOW_READ_AHEAD &&
            cutover.holdAcknowledged &&
            cutover.resumeRequested &&
            cutover.resumeApplied &&
            cutover.terminalOrderValid &&
            cutover.terminalCount ==
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cutover.released &&
            !cutover.fallbackLegacy &&
            !cutover.sessionLockout &&
            cutover.accountingValid &&
            registry.currentSession == generation.session &&
            registry.activeEntries == 0U &&
            registry.accountingValid;
        if (!releaseExact)
        {
            ++m_counters.proofMismatches;
            if (registry.activeEntries != 0U ||
                registry.currentSession != generation.session)
            {
                ++m_counters.registryMismatches;
            }
            Fail(
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_PROOF_MISMATCH,
                cohort.accountingValid && cutover.accountingValid &&
                registry.accountingValid);
            return;
        }

        generation.gateSequence = cutover.gateSequence;
        generation.terminalCount = cohort.terminalCount;
        generation.terminalOrderValid = cohort.terminalOrderValid;
        generation.allMembersCompleted = cohort.allMembersCompleted;
        generation.registryEmptyAtRelease = registry.activeEntries == 0U;
        generation.released = true;

        if (index != 0U &&
            generation.gateSequence <=
            m_snapshot.generations[index - 1U].gateSequence)
        {
            ++m_counters.gateSequenceMismatches;
            m_snapshot.gateSequenceMonotonic = false;
            Fail(
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_GATE_SEQUENCE_MISMATCH,
                true);
            return;
        }

        ++m_snapshot.releaseCount;
        if (index == 0U)
        {
            ++m_counters.firstReleased;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FIRST_RELEASED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortRearmDecision::
                K75_FIRST_GENERATION_RELEASED;
        }
        else
        {
            ++m_counters.secondReleased;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_RELEASED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortRearmDecision::K75_REARM_PROVEN;
            m_snapshot.rearmProven = true;
        }
        Publish();
    }

    void Fail(
        NCOrdinaryG00FeedHoldCohortRearmDecision decision,
        bool accountingValid) noexcept
    {
        if (!m_snapshot.failed)
        {
            ++m_counters.failures;
        }
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortRearmPhase::K75_FAILED;
        m_snapshot.decision = decision;
        m_snapshot.failed = true;
        m_snapshot.accountingValid = accountingValid;
        Publish();
    }

    void ResetForSession() noexcept
    {
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldCohortRearmSnapshot next{};
        next.publicationSequence = publicationSequence;
        m_snapshot = next;
        m_lastObservedCohortSequence = 0ULL;
        ++m_counters.sessionResets;
        Publish();
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

    std::uint64_t m_lastObservedCohortSequence = 0ULL;
    NCOrdinaryG00FeedHoldCohortRearmSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldCohortRearmCounters m_counters{};
};
