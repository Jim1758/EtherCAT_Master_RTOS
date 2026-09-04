#pragma once

#include "NCOrdinaryG00FeedHoldCohortRearmCutoverGate.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// NC-0.2K.7.7 - Same-Session Rolling Feed-Hold Re-arm Continuity Shadow
//
// Seed: exact K.7.5 SECOND_RELEASED/REARM_PROVEN plus exact K.7.6
// SECOND_RELEASED/ALLOW_READ_AHEAD, correlated with the still-published K.7.3
// terminal-complete and K.7.4 released second cohort.
//
// Rolling proof: generation 3 and later are represented by fixed
// lastReleased/currentActive records.  Five monotonic lineage high-water marks
// (Registry, Prepared entry, Dispatch, Commit and Motion segment) preserve
// same-session history without a generation-count capacity limit.  Same-
// session wrap/regression is intentionally unsupported and is diagnostic FAIL.
//
// Observation only: no admission API, Motion/PDO write, state/PC/callback/
// Epoch/owner change, allocation, lock, wait or sleep.  runtimeInfluence and
// motionWrite, and their counters, are invariantly zero.
// =============================================================================

enum class NCOrdinaryG00FeedHoldRollingRearmPhase : std::uint8_t
{
    K77_WAIT_SEED = 0,
    K77_SEEDED,
    K77_GENERATION_BOUND,
    K77_WAIT_RELEASE,
    K77_CONTINUITY_RELEASED,
    K77_FAILED
};

enum class NCOrdinaryG00FeedHoldRollingRearmDecision : std::uint8_t
{
    K77_WAIT_K75_SEED = 0,
    K77_WAIT_K76_SEED,
    K77_WAIT_CURRENT_COHORT_SEED,
    K77_SEED_ACCEPTED,
    K77_GENERATION_BOUND,
    K77_WAIT_TERMINAL_RELEASE,
    K77_GENERATION_RELEASED,
    K77_EARLY_GENERATION_EDGE,
    K77_SESSION_MISMATCH,
    K77_EXECUTION_LEASE_MISMATCH,
    K77_SEQUENCE_MISMATCH,
    K77_SEQUENCE_WRAP_UNSUPPORTED,
    K77_LINEAGE_MISMATCH,
    K77_REGISTRY_MISMATCH,
    K77_ACCOUNTING_MISMATCH,
    K77_ACTIVE_INVARIANT_MISMATCH,
    K77_ORDINAL_EXHAUSTED,
    K77_PROOF_MISMATCH,
    K77_FALLBACK_OBSERVED,
    K77_SESSION_RESET
};

inline const char* NCOrdinaryG00FeedHoldRollingRearmPhaseName(
    NCOrdinaryG00FeedHoldRollingRearmPhase value) noexcept
{
    switch (value)
    {
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_SEED:
        return "WAIT_SEED";
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_SEEDED:
        return "SEEDED";
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_GENERATION_BOUND:
        return "GENERATION_BOUND";
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_RELEASE:
        return "WAIT_RELEASE";
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_CONTINUITY_RELEASED:
        return "CONTINUITY_RELEASED";
    case NCOrdinaryG00FeedHoldRollingRearmPhase::K77_FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

inline const char* NCOrdinaryG00FeedHoldRollingRearmDecisionName(
    NCOrdinaryG00FeedHoldRollingRearmDecision value) noexcept
{
    switch (value)
    {
    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_WAIT_K75_SEED:
        return "WAIT_K75_SEED";
    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_WAIT_K76_SEED:
        return "WAIT_K76_SEED";
        case NCOrdinaryG00FeedHoldRollingRearmDecision::
        K77_WAIT_CURRENT_COHORT_SEED:
            return "WAIT_CURRENT_COHORT_SEED";
        case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SEED_ACCEPTED:
            return "SEED_ACCEPTED";
        case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_GENERATION_BOUND:
            return "GENERATION_BOUND";
            case NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_WAIT_TERMINAL_RELEASE:
                return "WAIT_TERMINAL_RELEASE";
                case NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_GENERATION_RELEASED:
                    return "GENERATION_RELEASED";
                    case NCOrdinaryG00FeedHoldRollingRearmDecision::
                    K77_EARLY_GENERATION_EDGE:
                        return "EARLY_GENERATION_EDGE";
                    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SESSION_MISMATCH:
                        return "SESSION_MISMATCH";
                        case NCOrdinaryG00FeedHoldRollingRearmDecision::
                        K77_EXECUTION_LEASE_MISMATCH:
                            return "EXECUTION_LEASE_MISMATCH";
                        case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SEQUENCE_MISMATCH:
                            return "SEQUENCE_MISMATCH";
                            case NCOrdinaryG00FeedHoldRollingRearmDecision::
                            K77_SEQUENCE_WRAP_UNSUPPORTED:
                                return "SEQUENCE_WRAP_UNSUPPORTED";
                            case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_LINEAGE_MISMATCH:
                                return "LINEAGE_MISMATCH";
                            case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_REGISTRY_MISMATCH:
                                return "REGISTRY_MISMATCH";
                                case NCOrdinaryG00FeedHoldRollingRearmDecision::
                                K77_ACCOUNTING_MISMATCH:
                                    return "ACCOUNTING_MISMATCH";
                                    case NCOrdinaryG00FeedHoldRollingRearmDecision::
                                    K77_ACTIVE_INVARIANT_MISMATCH:
                                        return "ACTIVE_INVARIANT_MISMATCH";
                                    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_ORDINAL_EXHAUSTED:
                                        return "ORDINAL_EXHAUSTED";
                                    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_PROOF_MISMATCH:
                                        return "PROOF_MISMATCH";
                                    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_FALLBACK_OBSERVED:
                                        return "FALLBACK_OBSERVED";
                                    case NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SESSION_RESET:
                                        return "SESSION_RESET";
                                    default:
                                        return "UNKNOWN";
    }
}

struct NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot
{
    std::uint64_t registrySequence = 0ULL;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    int sourcePC = -1;
    int sourceLineNumber = 0;
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    MotionFeedbackSequence terminalFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackType terminalFeedbackType = MotionFeedbackType::NONE;
    std::uint64_t lineageFingerprint = 0ULL;
    std::uint8_t terminalOrdinal = 0U;
    bool exact = false;
    bool terminalFeedbackReceived = false;
    bool terminalCompleted = false;
};

struct NCOrdinaryG00FeedHoldRollingGenerationSnapshot
{
    std::uint64_t generationOrdinal = 0ULL;
    std::uint64_t cohortPublicationAtCapture = 0ULL;
    std::uint64_t cutoverPublicationAtCapture = 0ULL;
    std::uint64_t registryPublicationAtCapture = 0ULL;
    std::uint64_t cohortPublicationAtRelease = 0ULL;
    std::uint64_t cutoverPublicationAtRelease = 0ULL;
    std::uint64_t registryPublicationAtRelease = 0ULL;
    std::uint64_t cohortSequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;
    std::uint64_t gateSequence = 0ULL;
    std::uint64_t lineageDigest = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    std::array<NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> members{};
    std::uint8_t memberCount = 0U;
    std::uint8_t terminalCount = 0U;
    bool seedEvidence = false;
    bool rollingEvidence = false;
    bool captured = false;
    bool released = false;
    bool holdAcknowledged = false;
    bool resumeRequested = false;
    bool resumeApplied = false;
    bool terminalOrderValid = true;
    bool allMembersCompleted = false;
    bool terminalFeedbackComplete = false;
    bool registryEmptyAtRelease = false;
};

struct NCOrdinaryG00FeedHoldRollingRearmSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t seedK75PublicationSequence = 0ULL;
    std::uint64_t seedK76PublicationSequence = 0ULL;
    std::uint64_t seedK73PublicationSequence = 0ULL;
    std::uint64_t seedK74PublicationSequence = 0ULL;
    std::uint64_t seedRegistryPublicationSequence = 0ULL;
    std::uint64_t seedCohortSequence = 0ULL;
    std::uint64_t seedBoundarySequence = 0ULL;
    std::uint64_t seedGateSequence = 0ULL;
    std::uint64_t lineageDigest = 0ULL;
    std::uint64_t lastCohortSequence = 0ULL;
    std::uint64_t lastBoundarySequence = 0ULL;
    std::uint64_t lastGateSequence = 0ULL;
    std::uint64_t registrySequenceHighWater = 0ULL;
    NCPreparedEntrySequence entrySequenceHighWater =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCBlockDispatchId dispatchIdHighWater = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSequence commitSequenceHighWater =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    MotionSegmentId segmentIdHighWater = MOTION_SEGMENT_ID_INVALID;
    std::uint64_t rollingCaptureCount = 0ULL;
    std::uint64_t rollingReleaseCount = 0ULL;
    std::uint64_t generationCount = 0ULL;
    std::uint64_t releaseCount = 0ULL;
    std::uint64_t activeGenerationOrdinal = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    NCOrdinaryG00FeedHoldRollingGenerationSnapshot lastReleased{};
    NCOrdinaryG00FeedHoldRollingGenerationSnapshot currentActive{};
    NCOrdinaryG00FeedHoldRollingRearmPhase phase =
        NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_SEED;
    NCOrdinaryG00FeedHoldRollingRearmDecision decision =
        NCOrdinaryG00FeedHoldRollingRearmDecision::K77_WAIT_K75_SEED;
    bool seeded = false;
    bool tracking = false;
    bool active = false;
    bool activeInvariantValid = true;
    bool releasedContinuity = false;
    bool sameSession = true;
    bool sameExecutionLease = true;
    bool sequenceMonotonic = true;
    bool fullLineageValid = true;
    bool noOverlap = true;
    bool failed = false;
    bool bounded = true;
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldRollingRearmCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t seedAttempts = 0ULL;
    std::uint64_t waitK75Seed = 0ULL;
    std::uint64_t waitK76Seed = 0ULL;
    std::uint64_t waitCurrentCohortSeed = 0ULL;
    std::uint64_t seedAccepted = 0ULL;
    std::uint64_t rollingCohortEdges = 0ULL;
    std::uint64_t nonExactBypasses = 0ULL;
    std::uint64_t rollingCaptures = 0ULL;
    std::uint64_t rollingReleases = 0ULL;
    std::uint64_t stableObservations = 0ULL;
    std::uint64_t earlyEdges = 0ULL;
    std::uint64_t sessionMismatches = 0ULL;
    std::uint64_t executionLeaseMismatches = 0ULL;
    std::uint64_t sequenceMismatches = 0ULL;
    std::uint64_t sequenceWrapUnsupported = 0ULL;
    std::uint64_t lineageMismatches = 0ULL;
    std::uint64_t registryMismatches = 0ULL;
    std::uint64_t accountingMismatches = 0ULL;
    std::uint64_t activeInvariantMismatches = 0ULL;
    std::uint64_t ordinalExhausted = 0ULL;
    std::uint64_t proofMismatches = 0ULL;
    std::uint64_t fallbackObserved = 0ULL;
    std::uint64_t sessionResets = 0ULL;
    std::uint64_t resetWhileActive = 0ULL;
    std::uint64_t failures = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot>::value,
    "K.7.7 member lineage must remain trivially copyable.");
static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingGenerationSnapshot>::value,
    "K.7.7 generation evidence must remain trivially copyable.");
static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingRearmSnapshot>::value,
    "K.7.7 snapshot must remain trivially copyable.");
static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingRearmCounters>::value,
    "K.7.7 counters must remain trivially copyable.");
static_assert(sizeof(NCOrdinaryG00FeedHoldRollingRearmSnapshot) <= 1024U,
    "K.7.7 snapshot exceeded its fixed diagnostic budget.");
static_assert(sizeof(NCOrdinaryG00FeedHoldRollingRearmCounters) <= 256U,
    "K.7.7 counters exceeded its fixed diagnostic budget.");

class NCOrdinaryG00FeedHoldRollingRearmShadow
{
public:
    NCOrdinaryG00FeedHoldRollingRearmShadow() noexcept = default;

    void Observe(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& k75,
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& k76,
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.observations;
        if (ResetForChangedSession(registry))
        {
            return;
        }
        if (m_snapshot.failed)
        {
            ++m_counters.stableObservations;
            return;
        }
        if (!m_snapshot.seeded)
        {
            ObserveSeed(k75, k76, cohort, cutover, registry);
            return;
        }
        if (k75.publicationSequence <
            m_snapshot.seedK75PublicationSequence ||
            k76.publicationSequence <
            m_snapshot.seedK76PublicationSequence)
        {
            ++m_counters.sequenceMismatches;
            ++m_counters.sequenceWrapUnsupported;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED, true);
            return;
        }
        if (!UpstreamSeedStable(k75, k76))
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, k75.accountingValid &&
                k76.accountingValid);
            return;
        }
        if (!registry.accountingValid)
        {
            ++m_counters.accountingMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACCOUNTING_MISMATCH, false);
            return;
        }
        if (!RegistryBasic(registry))
        {
            ++m_counters.registryMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_REGISTRY_MISMATCH, registry.accountingValid);
            return;
        }
        if (!ActiveInvariantExact())
        {
            ++m_counters.activeInvariantMismatches;
            m_snapshot.activeInvariantValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACTIVE_INVARIANT_MISMATCH, true);
            return;
        }
        if (cohort.cohortSequence == 0ULL)
        {
            ++m_counters.stableObservations;
            return;
        }
        if (m_lastObservedCohortSequence != 0ULL &&
            cohort.cohortSequence < m_lastObservedCohortSequence)
        {
            ++m_counters.sequenceMismatches;
            ++m_counters.sequenceWrapUnsupported;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED, true);
            return;
        }

        const bool newEdge =
            cohort.cohortSequence != m_lastObservedCohortSequence;
        if (newEdge)
        {
            ++m_counters.rollingCohortEdges;
            if (m_snapshot.active)
            {
                ++m_counters.earlyEdges;
                m_snapshot.noOverlap = false;
                Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                    K77_EARLY_GENERATION_EDGE, true);
                return;
            }
            m_lastObservedCohortSequence = cohort.cohortSequence;
            if (!ExactCaptureShape(cohort))
            {
                ObserveNonExactEdge(cohort, cutover, registry);
                return;
            }
            CaptureRolling(cohort, cutover, registry);
            return;
        }
        if (!m_snapshot.active)
        {
            ++m_counters.stableObservations;
            return;
        }
        ObserveActive(cohort, cutover, registry);
    }

    NCOrdinaryG00FeedHoldRollingRearmSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldRollingRearmCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static constexpr std::uint64_t HASH_OFFSET =
        1469598103934665603ULL;
    static constexpr std::uint64_t HASH_PRIME = 1099511628211ULL;

    static std::uint64_t Hash(
        std::uint64_t digest,
        std::uint64_t value) noexcept
    {
        return (digest ^ value) * HASH_PRIME;
    }

    static bool SameIdentity(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    static bool RegistryBasic(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return registry.ready && !registry.permanentLockout &&
            registry.currentSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.publicationSequence != 0ULL && registry.bounded &&
            !registry.motionWrite && registry.accountingValid &&
            registry.capacity ==
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY &&
            registry.activeEntries <= registry.occupiedEntries &&
            registry.occupiedEntries <= registry.capacity &&
            registry.activeEntries <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    static bool K75Ready(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& value) noexcept
    {
        return value.phase ==
            NCOrdinaryG00FeedHoldCohortRearmPhase::K75_SECOND_RELEASED &&
            value.decision ==
            NCOrdinaryG00FeedHoldCohortRearmDecision::K75_REARM_PROVEN &&
            value.rearmProven;
    }

    static bool K76Ready(
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& value) noexcept
    {
        return value.phase ==
            NCOrdinaryG00FeedHoldRearmCutoverPhase::
            K76_SECOND_RELEASED &&
            value.decision ==
            NCOrdinaryG00FeedHoldRearmCutoverDecision::
            K76_ALLOW_READ_AHEAD &&
            value.released;
    }

    static bool GenerationExact(
        const NCOrdinaryG00FeedHoldCohortGenerationSnapshot& value) noexcept
    {
        if (!value.captured || !value.released || !value.exact ||
            !value.terminalOrderValid || !value.allMembersCompleted ||
            !value.registryEmptyAtRelease || value.memberCount != 2U ||
            value.terminalCount != 2U ||
            value.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            value.executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
            value.owner != MotionOwner::AUTO ||
            value.ownerGeneration == MOTION_OWNER_GENERATION_INVALID ||
            value.cohortSequence == 0ULL ||
            value.boundarySequence == 0ULL || value.gateSequence == 0ULL)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (!value.memberIdentity[i].IsAssigned() ||
                value.memberEntrySequence[i] == 0ULL ||
                value.memberDispatchId[i] == 0ULL)
            {
                return false;
            }
        }
        return !SameIdentity(
            value.memberIdentity[0], value.memberIdentity[1]);
    }

    static bool UpstreamCoreExact(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& k75,
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& k76) noexcept
    {
        const auto& first = k75.generations[0];
        const auto& second = k75.generations[1];
        if (!K75Ready(k75) || !K76Ready(k76) ||
            k75.generationCount != 2U || k75.releaseCount != 2U ||
            !k75.sameSession || !k75.sameExecutionLease ||
            !k75.cohortSequenceMonotonic ||
            !k75.boundarySequenceMonotonic ||
            !k75.gateSequenceMonotonic || !k75.memberIsolation ||
            !k75.noOverlap || k75.failed || !k75.bounded ||
            !k75.shadowOnly || k75.runtimeInfluence || k75.motionWrite ||
            !k75.accountingValid || !GenerationExact(first) ||
            !GenerationExact(second) || first.session != k75.session ||
            second.session != k75.session ||
            second.executionEpoch != first.executionEpoch ||
            second.owner != first.owner ||
            second.ownerGeneration != first.ownerGeneration ||
            second.cohortSequence <= first.cohortSequence ||
            second.boundarySequence <= first.boundarySequence ||
            second.gateSequence <= first.gateSequence ||
            !k76.enabled || !k76.exactSecondGeneration || !k76.bound ||
            k76.waiting || k76.fallbackLegacy || k76.sessionLockout ||
            k76.motionWrite || !k76.accountingValid ||
            k76.rearmPublicationSequence != k75.publicationSequence ||
            k76.session != second.session ||
            k76.executionEpoch != second.executionEpoch ||
            k76.owner != second.owner ||
            k76.ownerGeneration != second.ownerGeneration ||
            k76.cohortSequence != second.cohortSequence ||
            k76.boundarySequence != second.boundarySequence ||
            k76.gateSequence != second.gateSequence ||
            k76.generationCount != 2U || k76.releaseCount != 2U)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (!SameIdentity(k76.memberIdentity[i],
                second.memberIdentity[i]) ||
                k76.memberEntrySequence[i] !=
                second.memberEntrySequence[i] ||
                k76.memberDispatchId[i] != second.memberDispatchId[i])
            {
                return false;
            }
        }
        return true;
    }

    static bool TerminalCohortExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_COMPLETE &&
            cohort.decision ==
            NCOrdinaryG00FeedHoldCohortDecision::
            K73_ALL_COMPLETED_EXACT &&
            !cohort.active && !cohort.failed && !cohort.cancelled &&
            cohort.candidate && cohort.exactTwoEntryCohort &&
            cohort.boundaryCorrelated && cohort.gateCorrelated &&
            cohort.holdAcknowledged && cohort.resumeRequested &&
            cohort.resumeApplied && cohort.terminalOrderValid &&
            cohort.terminalCohortComplete && cohort.allMembersCompleted &&
            cohort.memberCount == 2U && cohort.terminalCount == 2U &&
            cohort.bounded && cohort.shadowOnly &&
            !cohort.runtimeInfluence && !cohort.motionWrite &&
            cohort.accountingValid && cohort.members[0].exact &&
            cohort.members[0].terminalObserved &&
            cohort.members[0].terminalCompleted &&
            cohort.members[0].terminalOrdinal == 1U &&
            cohort.members[0].lastFeedbackSequence != 0ULL &&
            cohort.members[0].lastFeedbackType ==
            MotionFeedbackType::COMPLETED &&
            cohort.members[1].exact && cohort.members[1].terminalObserved &&
            cohort.members[1].terminalCompleted &&
            cohort.members[1].terminalOrdinal == 2U &&
            cohort.members[1].lastFeedbackSequence != 0ULL &&
            cohort.members[1].lastFeedbackType ==
            MotionFeedbackType::COMPLETED &&
            cohort.members[0].lastFeedbackSequence <
            cohort.members[1].lastFeedbackSequence&&
            cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED &&
            cutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            ALLOW_READ_AHEAD &&
            cutover.enabled && cutover.exactCohort && cutover.bound &&
            cutover.holdAcknowledged == cohort.holdAcknowledged &&
            cutover.resumeRequested == cohort.resumeRequested &&
            cutover.resumeApplied == cohort.resumeApplied &&
            cutover.released && !cutover.fallbackLegacy &&
            !cutover.sessionLockout && cutover.terminalOrderValid &&
            cutover.memberCount == 2U && cutover.terminalCount == 2U &&
            cutover.registryActiveEntries == 0U &&
            !cutover.motionWrite && cutover.accountingValid &&
            RegistryBasic(registry) && registry.activeEntries == 0U &&
            registry.lastObservedFeedbackSequence ==
            cohort.members[1].lastFeedbackSequence;
    }

    static bool ActiveCohortPhaseDecisionExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        if (!cohort.candidate || !cohort.active ||
            !cohort.exactTwoEntryCohort || !cohort.boundaryCorrelated ||
            cohort.terminalCohortComplete || cohort.failed ||
            cohort.cancelled || !cohort.bounded || !cohort.shadowOnly ||
            cohort.runtimeInfluence || cohort.motionWrite ||
            !cohort.accountingValid || cohort.memberCount != 2U ||
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
            return cohort.terminalCount == 0U && cohort.holdAcknowledged &&
                !cohort.resumeApplied &&
                cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_HOLD_ACKNOWLEDGED;
        case NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_REQUESTED:
            return cohort.terminalCount == 0U && cohort.resumeRequested &&
                !cohort.resumeApplied &&
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
                cohort.resumeApplied &&
                cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_APPLIED;
        case NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_PENDING:
            return cohort.terminalCount == 1U && cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_WAIT_SECOND_TERMINAL;
        default:
            return false;
        }
    }

    static bool ActiveCutoverPhaseDecisionExact(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover) noexcept
    {
        if (!cutover.enabled || !cutover.exactCohort || !cutover.bound ||
            cutover.released || !cutover.waiting ||
            cutover.fallbackLegacy || cutover.sessionLockout ||
            cutover.motionWrite || !cutover.accountingValid ||
            cutover.memberCount != 2U || cutover.terminalCount > 1U)
        {
            return false;
        }
        if (cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BOUND)
        {
            return cutover.decision ==
                NCOrdinaryG00FeedHoldCohortCutoverDecision::COHORT_BOUND;
        }
        if (cutover.phase !=
            NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL)
        {
            return false;
        }
        if (!cutover.holdAcknowledged)
        {
            return cutover.decision ==
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_HOLD_ACK;
        }
        if (!cutover.resumeApplied)
        {
            return cutover.decision ==
                NCOrdinaryG00FeedHoldCohortCutoverDecision::WAIT_RESUME;
        }
        return cutover.decision == (cutover.terminalCount == 0U
            ? NCOrdinaryG00FeedHoldCohortCutoverDecision::
            WAIT_FIRST_TERMINAL
            : NCOrdinaryG00FeedHoldCohortCutoverDecision::
            WAIT_SECOND_TERMINAL);
    }

    static bool AllSourceSecondGenerationExact(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& k75,
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& k76,
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& second = k75.generations[1];
        if (k75.publicationSequence == 0ULL ||
            k76.publicationSequence == 0ULL ||
            cohort.publicationSequence == 0ULL ||
            cutover.publicationSequence == 0ULL ||
            registry.publicationSequence == 0ULL ||
            cohort.registryPublicationSequence == 0ULL ||
            cohort.registryPublicationSequence >=
            registry.publicationSequence ||
            !TerminalCohortExact(cohort, cutover, registry) ||
            cohort.cohortSequence != second.cohortSequence ||
            cohort.boundarySequence != second.boundarySequence ||
            cohort.gateSequence != second.gateSequence ||
            cohort.session != second.session ||
            cohort.executionEpoch != second.executionEpoch ||
            cohort.owner != second.owner ||
            cohort.ownerGeneration != second.ownerGeneration ||
            cutover.cohortSequence != second.cohortSequence ||
            cutover.boundarySequence != second.boundarySequence ||
            cutover.gateSequence != second.gateSequence ||
            cutover.session != second.session ||
            cutover.executionEpoch != second.executionEpoch ||
            cutover.owner != second.owner ||
            cutover.ownerGeneration != second.ownerGeneration ||
            registry.currentSession != second.session)
        {
            return false;
        }
        if (cohort.members[0].registrySequence >=
            cohort.members[1].registrySequence ||
            cohort.members[0].entrySequence >=
            cohort.members[1].entrySequence ||
            cohort.members[0].dispatchId >=
            cohort.members[1].dispatchId ||
            cohort.members[0].commitSequence >=
            cohort.members[1].commitSequence)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (!SameIdentity(cohort.members[i].identity,
                second.memberIdentity[i]) ||
                !SameIdentity(cohort.members[i].identity,
                    k76.memberIdentity[i]) ||
                cohort.members[i].entrySequence !=
                second.memberEntrySequence[i] ||
                cohort.members[i].entrySequence !=
                k76.memberEntrySequence[i] ||
                cohort.members[i].dispatchId !=
                second.memberDispatchId[i] ||
                cohort.members[i].dispatchId !=
                k76.memberDispatchId[i] ||
                !cohort.members[i].exact ||
                cohort.members[i].identity.source !=
                MotionCommandSource::NC_MEMORY ||
                cohort.members[i].identity.epoch !=
                second.executionEpoch ||
                cohort.members[i].ownerLease.owner != second.owner ||
                cohort.members[i].ownerLease.generation !=
                second.ownerGeneration ||
                cohort.members[i].session != second.session ||
                cohort.members[i].registrySequence == 0ULL ||
                cohort.members[i].commitSequence == 0ULL)
            {
                return false;
            }
        }
        return true;
    }

    void ObserveSeed(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& k75,
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& k76,
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.seedAttempts;
        // Upstream incomplete, failed or fallback evidence is only WAIT before
        // K.7.7 owns a seed.  K.7.7 never claims an upstream failure as its own.
        if (!K75Ready(k75) || k75.failed)
        {
            ++m_counters.waitK75Seed;
            SetWait(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_WAIT_K75_SEED);
            return;
        }
        if (!K76Ready(k76) || k76.fallbackLegacy || k76.sessionLockout)
        {
            ++m_counters.waitK76Seed;
            SetWait(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_WAIT_K76_SEED);
            return;
        }
        if (k75.publicationSequence == 0ULL ||
            k76.publicationSequence == 0ULL ||
            cohort.publicationSequence == 0ULL ||
            cohort.registryPublicationSequence == 0ULL ||
            cutover.publicationSequence == 0ULL ||
            registry.publicationSequence == 0ULL)
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, k75.accountingValid &&
                k76.accountingValid && cohort.accountingValid &&
                cutover.accountingValid && registry.accountingValid);
            return;
        }
        if (!TerminalCohortExact(cohort, cutover, registry))
        {
            ++m_counters.waitCurrentCohortSeed;
            SetWait(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_WAIT_CURRENT_COHORT_SEED);
            return;
        }
        if (!UpstreamCoreExact(k75, k76) ||
            !AllSourceSecondGenerationExact(
                k75, k76, cohort, cutover, registry))
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, k75.accountingValid &&
                k76.accountingValid && cohort.accountingValid &&
                cutover.accountingValid && registry.accountingValid);
            return;
        }

        const auto& first = k75.generations[0];
        const auto& second = k75.generations[1];
        m_snapshot.seedK75PublicationSequence = k75.publicationSequence;
        m_snapshot.seedK76PublicationSequence = k76.publicationSequence;
        m_snapshot.seedK73PublicationSequence = cohort.publicationSequence;
        m_snapshot.seedK74PublicationSequence = cutover.publicationSequence;
        m_snapshot.seedRegistryPublicationSequence =
            registry.publicationSequence;
        m_snapshot.seedCohortSequence = second.cohortSequence;
        m_snapshot.seedBoundarySequence = second.boundarySequence;
        m_snapshot.seedGateSequence = second.gateSequence;
        m_snapshot.session = second.session;
        m_snapshot.executionEpoch = second.executionEpoch;
        m_snapshot.owner = second.owner;
        m_snapshot.ownerGeneration = second.ownerGeneration;
        m_snapshot.lastReleased = MakeGeneration(
            cohort, cutover, registry, 2ULL, true);
        m_snapshot.lineageDigest = m_snapshot.lastReleased.lineageDigest;
        m_snapshot.lastCohortSequence = second.cohortSequence;
        m_snapshot.lastBoundarySequence = second.boundarySequence;
        m_snapshot.lastGateSequence = second.gateSequence;
        m_snapshot.generationCount = 2ULL;
        m_snapshot.releaseCount = 2ULL;

        for (std::size_t i = 0U; i < 2U; ++i)
        {
            Raise(m_snapshot.entrySequenceHighWater,
                first.memberEntrySequence[i]);
            Raise(m_snapshot.entrySequenceHighWater,
                second.memberEntrySequence[i]);
            Raise(m_snapshot.dispatchIdHighWater,
                first.memberDispatchId[i]);
            Raise(m_snapshot.dispatchIdHighWater,
                second.memberDispatchId[i]);
            Raise(m_snapshot.segmentIdHighWater,
                first.memberIdentity[i].segmentId);
            Raise(m_snapshot.segmentIdHighWater,
                second.memberIdentity[i].segmentId);
            Raise(m_snapshot.registrySequenceHighWater,
                cohort.members[i].registrySequence);
            Raise(m_snapshot.commitSequenceHighWater,
                cohort.members[i].commitSequence);
        }
        m_snapshot.seeded = true;
        m_snapshot.tracking = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_SEEDED;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SEED_ACCEPTED;
        m_lastObservedCohortSequence = second.cohortSequence;
        ++m_counters.seedAccepted;
        Publish();
    }

    bool UpstreamSeedStable(
        const NCOrdinaryG00FeedHoldCohortRearmSnapshot& k75,
        const NCOrdinaryG00FeedHoldRearmCutoverSnapshot& k76) const noexcept
    {
        return UpstreamCoreExact(k75, k76) &&
            k75.publicationSequence ==
            m_snapshot.seedK75PublicationSequence &&
            k76.rearmPublicationSequence ==
            m_snapshot.seedK75PublicationSequence &&
            k75.session == m_snapshot.session &&
            k75.generations[1].cohortSequence ==
            m_snapshot.seedCohortSequence &&
            k75.generations[1].boundarySequence ==
            m_snapshot.seedBoundarySequence &&
            k75.generations[1].gateSequence == m_snapshot.seedGateSequence;
    }

    static bool ExactCaptureShape(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        return cohort.candidate && cohort.active &&
            cohort.exactTwoEntryCohort && cohort.boundaryCorrelated &&
            cohort.activeEntriesAtCapture == 2U && cohort.memberCount == 2U;
    }

    static bool SafeNonExactBypass(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const bool bypassKind =
            cohort.decision ==
            NCOrdinaryG00FeedHoldCohortDecision::
            K73_BYPASS_NOT_PROGRAM ||
            (cohort.decision ==
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_BYPASS_NOT_TWO_ACTIVE &&
                cohort.activeEntriesAtCapture != 2U);
        return cohort.phase ==
            NCOrdinaryG00FeedHoldCohortPhase::K73_BYPASSED &&
            bypassKind && !cohort.active && !cohort.exactTwoEntryCohort &&
            !cohort.failed && !cohort.cancelled && cohort.bounded &&
            cohort.shadowOnly && !cohort.runtimeInfluence &&
            !cohort.motionWrite && cohort.accountingValid &&
            cohort.registryPublicationSequence ==
            registry.publicationSequence &&
            cohort.activeEntriesAtCapture == registry.activeEntries &&
            cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            cutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            BYPASS_NO_EXACT_COHORT &&
            cutover.enabled && !cutover.exactCohort && !cutover.bound &&
            !cutover.waiting && !cutover.released &&
            !cutover.fallbackLegacy && !cutover.sessionLockout &&
            !cutover.motionWrite && cutover.accountingValid &&
            RegistryBasic(registry);
    }

    void ObserveNonExactEdge(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (SafeNonExactBypass(cohort, cutover, registry))
        {
            const auto& last = m_snapshot.lastReleased;
            if (cohort.publicationSequence >=
                last.cohortPublicationAtRelease &&
                cutover.publicationSequence >=
                last.cutoverPublicationAtRelease &&
                registry.publicationSequence >=
                last.registryPublicationAtRelease)
            {
                ++m_counters.nonExactBypasses;
                return;
            }
            ++m_counters.sequenceMismatches;
            ++m_counters.sequenceWrapUnsupported;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED, true);
            return;
        }
        if (cohort.failed || cohort.cancelled || cutover.fallbackLegacy ||
            cutover.sessionLockout)
        {
            ++m_counters.fallbackObserved;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_FALLBACK_OBSERVED, cohort.accountingValid &&
                cutover.accountingValid && registry.accountingValid);
            return;
        }
        if (!cohort.accountingValid || !cutover.accountingValid ||
            !registry.accountingValid)
        {
            ++m_counters.accountingMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACCOUNTING_MISMATCH, false);
            return;
        }
        if (!RegistryBasic(registry))
        {
            ++m_counters.registryMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_REGISTRY_MISMATCH, registry.accountingValid);
            return;
        }
        ++m_counters.proofMismatches;
        Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_PROOF_MISMATCH, true);
    }

    bool CapturePublicationsAdvance(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& last = m_snapshot.lastReleased;
        if (cohort.registryPublicationSequence !=
            registry.publicationSequence)
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, true);
            return false;
        }
        const bool wrapped =
            cohort.publicationSequence <
            last.cohortPublicationAtRelease ||
            cutover.publicationSequence <
            last.cutoverPublicationAtRelease ||
            registry.publicationSequence <
            last.registryPublicationAtRelease;
        const bool advances = cohort.publicationSequence != 0ULL &&
            cutover.publicationSequence != 0ULL &&
            registry.publicationSequence != 0ULL &&
            cohort.publicationSequence >
            last.cohortPublicationAtRelease &&
            cutover.publicationSequence >
            last.cutoverPublicationAtRelease &&
            registry.publicationSequence >
            last.registryPublicationAtRelease;
        if (!advances)
        {
            ++m_counters.sequenceMismatches;
            if (wrapped)
            {
                ++m_counters.sequenceWrapUnsupported;
            }
            m_snapshot.sequenceMonotonic = false;
            Fail(wrapped
                ? NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED
                : NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH,
                true);
            return false;
        }
        return true;
    }

    bool ActivePublicationsStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& active = m_snapshot.currentActive;
        if (cohort.registryPublicationSequence !=
            active.registryPublicationAtCapture)
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, true);
            return false;
        }
        if (cohort.publicationSequence == 0ULL ||
            cutover.publicationSequence == 0ULL ||
            registry.publicationSequence == 0ULL ||
            cohort.publicationSequence <
            active.cohortPublicationAtCapture ||
            cutover.publicationSequence <
            active.cutoverPublicationAtCapture ||
            registry.publicationSequence <
            active.registryPublicationAtCapture)
        {
            ++m_counters.sequenceMismatches;
            ++m_counters.sequenceWrapUnsupported;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED, true);
            return false;
        }
        return true;
    }

    bool ReleasePublicationsAdvance(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& active = m_snapshot.currentActive;
        if (cohort.publicationSequence <=
            active.cohortPublicationAtCapture ||
            cutover.publicationSequence <=
            active.cutoverPublicationAtCapture ||
            registry.publicationSequence <=
            active.registryPublicationAtCapture)
        {
            const bool wrapped =
                cohort.publicationSequence <
                active.cohortPublicationAtCapture ||
                cutover.publicationSequence <
                active.cutoverPublicationAtCapture ||
                registry.publicationSequence <
                active.registryPublicationAtCapture;
            ++m_counters.sequenceMismatches;
            if (wrapped)
            {
                ++m_counters.sequenceWrapUnsupported;
            }
            m_snapshot.sequenceMonotonic = false;
            Fail(wrapped
                ? NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED
                : NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH,
                true);
            return false;
        }
        return true;
    }

    bool RollingTerminalContinuityExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort) noexcept
    {
        const MotionFeedbackSequence previous =
            m_snapshot.lastReleased.members[1].terminalFeedbackSequence;
        const MotionFeedbackSequence first =
            cohort.members[0].lastFeedbackSequence;
        if (previous == MOTION_FEEDBACK_SEQUENCE_INVALID ||
            first == MOTION_FEEDBACK_SEQUENCE_INVALID || first <= previous)
        {
            const bool wrapped = first < previous;
            ++m_counters.sequenceMismatches;
            if (wrapped)
            {
                ++m_counters.sequenceWrapUnsupported;
            }
            m_snapshot.sequenceMonotonic = false;
            Fail(wrapped
                ? NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_WRAP_UNSUPPORTED
                : NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH,
                true);
            return false;
        }
        return true;
    }

    void CaptureRolling(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!CaptureSourcesExact(cohort, cutover, registry))
        {
            return;
        }
        if (m_snapshot.generationCount ==
            (std::numeric_limits<std::uint64_t>::max)())
        {
            ++m_counters.ordinalExhausted;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ORDINAL_EXHAUSTED, true);
            return;
        }
        NCOrdinaryG00FeedHoldRollingGenerationSnapshot generation =
            MakeGeneration(cohort, cutover, registry,
                m_snapshot.generationCount + 1ULL, false);
        if (!LineageAdvances(generation))
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, true);
            return;
        }
        RaiseHighWater(generation);
        m_snapshot.currentActive = generation;
        ++m_snapshot.rollingCaptureCount;
        ++m_snapshot.generationCount;
        m_snapshot.activeGenerationOrdinal = generation.generationOrdinal;
        m_snapshot.lastCohortSequence = generation.cohortSequence;
        m_snapshot.lastBoundarySequence = generation.boundarySequence;
        m_snapshot.active = true;
        m_snapshot.releasedContinuity = false;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_GENERATION_BOUND;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingRearmDecision::K77_GENERATION_BOUND;
        ++m_counters.rollingCaptures;
        Publish();
    }

    bool CaptureSourcesExact(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (cohort.failed || cohort.cancelled || cutover.fallbackLegacy ||
            cutover.sessionLockout)
        {
            ++m_counters.fallbackObserved;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_FALLBACK_OBSERVED, cohort.accountingValid &&
                cutover.accountingValid);
            return false;
        }
        if (!cohort.accountingValid || !cutover.accountingValid ||
            !registry.accountingValid)
        {
            ++m_counters.accountingMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACCOUNTING_MISMATCH, false);
            return false;
        }
        if (!ActiveCohortPhaseDecisionExact(cohort) ||
            !ActiveCutoverPhaseDecisionExact(cutover))
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, true);
            return false;
        }
        if (cohort.session != m_snapshot.session ||
            cutover.session != m_snapshot.session ||
            registry.currentSession != m_snapshot.session)
        {
            ++m_counters.sessionMismatches;
            m_snapshot.sameSession = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SESSION_MISMATCH, true);
            return false;
        }
        if (!CapturePublicationsAdvance(cohort, cutover, registry))
        {
            return false;
        }
        if (!LeaseMatches(cohort) || !LeaseMatches(cutover))
        {
            ++m_counters.executionLeaseMismatches;
            m_snapshot.sameExecutionLease = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_EXECUTION_LEASE_MISMATCH, true);
            return false;
        }
        if (cohort.cohortSequence <= m_snapshot.lastCohortSequence ||
            cohort.boundarySequence <= m_snapshot.lastBoundarySequence ||
            cutover.cohortSequence != cohort.cohortSequence ||
            cutover.boundarySequence != cohort.boundarySequence)
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, true);
            return false;
        }
        if (cutover.activeEntriesAtCapture != 2U ||
            cutover.memberCount != cohort.memberCount ||
            cutover.terminalCount != cohort.terminalCount ||
            cutover.registryActiveEntries != registry.activeEntries ||
            cutover.holdAcknowledged != cohort.holdAcknowledged ||
            cutover.resumeRequested != cohort.resumeRequested ||
            cutover.resumeApplied != cohort.resumeApplied ||
            cutover.terminalOrderValid != cohort.terminalOrderValid)
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, cutover.accountingValid);
            return false;
        }
        if (!RegistryBasic(registry) || registry.activeEntries != 2U)
        {
            ++m_counters.registryMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_REGISTRY_MISMATCH, true);
            return false;
        }
        if ((cohort.gateSequence != 0ULL || cutover.gateSequence != 0ULL) &&
            (cohort.gateSequence != cutover.gateSequence ||
                cohort.gateSequence <= m_snapshot.lastGateSequence))
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, true);
            return false;
        }
        for (std::size_t index = 0U; index < 2U; ++index)
        {
            if (cohort.members[index].session != m_snapshot.session)
            {
                ++m_counters.lineageMismatches;
                m_snapshot.fullLineageValid = false;
                Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                    K77_LINEAGE_MISMATCH, true);
                return false;
            }
        }
        return true;
    }

    void ObserveActive(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!ActiveSourcesStable(cohort, cutover, registry))
        {
            return;
        }
        if (cohort.terminalCohortComplete || cutover.released)
        {
            if (!TerminalCohortExact(cohort, cutover, registry))
            {
                ++m_counters.proofMismatches;
                Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                    K77_PROOF_MISMATCH, true);
                return;
            }
            if (!ReleasePublicationsAdvance(cohort, cutover, registry) ||
                !RollingTerminalContinuityExact(cohort))
            {
                return;
            }
            NCOrdinaryG00FeedHoldRollingGenerationSnapshot released =
                m_snapshot.currentActive;
            released.cohortPublicationAtRelease =
                cohort.publicationSequence;
            released.cutoverPublicationAtRelease =
                cutover.publicationSequence;
            released.registryPublicationAtRelease =
                registry.publicationSequence;
            released.gateSequence = cohort.gateSequence;
            released.terminalCount = cohort.terminalCount;
            released.released = true;
            released.rollingEvidence = true;
            released.holdAcknowledged = cohort.holdAcknowledged;
            released.resumeRequested = cohort.resumeRequested;
            released.resumeApplied = cohort.resumeApplied;
            released.terminalOrderValid = cohort.terminalOrderValid;
            released.allMembersCompleted = cohort.allMembersCompleted;
            released.registryEmptyAtRelease =
                registry.activeEntries == 0U;
            for (std::size_t i = 0U; i < 2U; ++i)
            {
                released.members[i].terminalFeedbackSequence =
                    cohort.members[i].lastFeedbackSequence;
                released.members[i].terminalFeedbackType =
                    cohort.members[i].lastFeedbackType;
                released.members[i].terminalOrdinal =
                    cohort.members[i].terminalOrdinal;
                released.members[i].terminalFeedbackReceived =
                    cohort.members[i].terminalObserved;
                released.members[i].terminalCompleted =
                    cohort.members[i].terminalCompleted;
            }
            released.terminalFeedbackComplete =
                released.members[0].terminalFeedbackReceived &&
                released.members[0].terminalCompleted &&
                released.members[1].terminalFeedbackReceived &&
                released.members[1].terminalCompleted;
            released.lineageDigest = Hash(
                m_snapshot.lineageDigest,
                GenerationDigest(released));
            m_snapshot.lineageDigest = released.lineageDigest;
            m_snapshot.lastReleased = released;
            m_snapshot.currentActive = {};
            ++m_snapshot.rollingReleaseCount;
            ++m_snapshot.releaseCount;
            m_snapshot.lastGateSequence = released.gateSequence;
            m_snapshot.activeGenerationOrdinal = 0ULL;
            m_snapshot.active = false;
            m_snapshot.releasedContinuity = true;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldRollingRearmPhase::
                K77_CONTINUITY_RELEASED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_GENERATION_RELEASED;
            ++m_counters.rollingReleases;
            Publish();
            return;
        }

        const bool changed =
            m_snapshot.currentActive.gateSequence != cohort.gateSequence ||
            m_snapshot.currentActive.terminalCount != cohort.terminalCount ||
            m_snapshot.currentActive.holdAcknowledged !=
            cohort.holdAcknowledged ||
            m_snapshot.currentActive.resumeRequested !=
            cohort.resumeRequested ||
            m_snapshot.currentActive.resumeApplied !=
            cohort.resumeApplied ||
            m_snapshot.currentActive.terminalOrderValid !=
            cohort.terminalOrderValid ||
            m_snapshot.phase !=
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_RELEASE ||
            m_snapshot.decision !=
            NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_WAIT_TERMINAL_RELEASE;
        m_snapshot.currentActive.gateSequence = cohort.gateSequence;
        m_snapshot.currentActive.terminalCount = cohort.terminalCount;
        m_snapshot.currentActive.holdAcknowledged =
            cohort.holdAcknowledged;
        m_snapshot.currentActive.resumeRequested =
            cohort.resumeRequested;
        m_snapshot.currentActive.resumeApplied = cohort.resumeApplied;
        m_snapshot.currentActive.terminalOrderValid =
            cohort.terminalOrderValid;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_RELEASE;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_WAIT_TERMINAL_RELEASE;
        if (changed)
        {
            Publish();
        }
        else
        {
            ++m_counters.stableObservations;
        }
    }

    bool ActiveSourcesStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& active = m_snapshot.currentActive;
        if (cohort.session != m_snapshot.session ||
            cutover.session != m_snapshot.session ||
            registry.currentSession != m_snapshot.session)
        {
            ++m_counters.sessionMismatches;
            m_snapshot.sameSession = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SESSION_MISMATCH, true);
            return false;
        }
        if (!LeaseMatches(cohort) || !LeaseMatches(cutover))
        {
            ++m_counters.executionLeaseMismatches;
            m_snapshot.sameExecutionLease = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_EXECUTION_LEASE_MISMATCH, true);
            return false;
        }
        const bool terminalTransition =
            cohort.terminalCohortComplete || cutover.released;
        if (!terminalTransition &&
            (!ActiveCohortPhaseDecisionExact(cohort) ||
                !ActiveCutoverPhaseDecisionExact(cutover)))
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, true);
            return false;
        }
        if (!ActivePublicationsStable(cohort, cutover, registry))
        {
            return false;
        }
        if (cohort.cohortSequence != active.cohortSequence ||
            cohort.boundarySequence != active.boundarySequence ||
            cutover.cohortSequence != active.cohortSequence ||
            cutover.boundarySequence != active.boundarySequence)
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, true);
            return false;
        }
        if (!FullMembersStable(cohort, active))
        {
            ++m_counters.lineageMismatches;
            m_snapshot.fullLineageValid = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_LINEAGE_MISMATCH, true);
            return false;
        }
        if (!registry.accountingValid || !cohort.accountingValid ||
            !cutover.accountingValid)
        {
            ++m_counters.accountingMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACCOUNTING_MISMATCH, false);
            return false;
        }
        if (!RegistryBasic(registry))
        {
            ++m_counters.registryMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_REGISTRY_MISMATCH, registry.accountingValid);
            return false;
        }
        if (static_cast<std::uint32_t>(cohort.terminalCount) +
            registry.activeEntries != 2U)
        {
            ++m_counters.accountingMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_ACCOUNTING_MISMATCH, false);
            return false;
        }
        if (cohort.failed || cohort.cancelled || cutover.fallbackLegacy ||
            cutover.sessionLockout)
        {
            ++m_counters.fallbackObserved;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_FALLBACK_OBSERVED, cohort.accountingValid &&
                cutover.accountingValid);
            return false;
        }
        if (!cutover.enabled || !cutover.exactCohort || !cutover.bound)
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, cutover.accountingValid);
            return false;
        }
        if ((!cutover.released && cutover.phase !=
            NCOrdinaryG00FeedHoldCohortCutoverPhase::WAIT_TERMINAL) ||
            (cutover.released && cutover.phase !=
                NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED))
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, cutover.accountingValid);
            return false;
        }
        if (cutover.memberCount != cohort.memberCount ||
            cutover.terminalCount != cohort.terminalCount ||
            cutover.registryActiveEntries != registry.activeEntries ||
            cutover.holdAcknowledged != cohort.holdAcknowledged ||
            cutover.resumeRequested != cohort.resumeRequested ||
            cutover.resumeApplied != cohort.resumeApplied ||
            cutover.terminalOrderValid != cohort.terminalOrderValid)
        {
            ++m_counters.proofMismatches;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_PROOF_MISMATCH, cutover.accountingValid);
            return false;
        }
        if (m_snapshot.currentActive.gateSequence != 0ULL &&
            cohort.gateSequence !=
            m_snapshot.currentActive.gateSequence)
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, true);
            return false;
        }
        if ((cohort.gateSequence != 0ULL || cutover.gateSequence != 0ULL) &&
            (cohort.gateSequence != cutover.gateSequence ||
                cohort.gateSequence <= m_snapshot.lastGateSequence))
        {
            ++m_counters.sequenceMismatches;
            m_snapshot.sequenceMonotonic = false;
            Fail(NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_SEQUENCE_MISMATCH, true);
            return false;
        }
        return true;
    }

    bool LeaseMatches(
        const NCOrdinaryG00FeedHoldCohortSnapshot& value) const noexcept
    {
        return value.executionEpoch == m_snapshot.executionEpoch &&
            value.owner == m_snapshot.owner &&
            value.ownerGeneration == m_snapshot.ownerGeneration;
    }

    bool LeaseMatches(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& value) const noexcept
    {
        return value.executionEpoch == m_snapshot.executionEpoch &&
            value.owner == m_snapshot.owner &&
            value.ownerGeneration == m_snapshot.ownerGeneration;
    }

    static NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot MakeMember(
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& source,
        bool withTerminal) noexcept
    {
        NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot value{};
        value.registrySequence = source.registrySequence;
        value.entrySequence = source.entrySequence;
        value.dispatchId = source.dispatchId;
        value.commitSequence = source.commitSequence;
        value.sourcePC = source.sourcePC;
        value.sourceLineNumber = source.sourceLineNumber;
        value.identity = source.identity;
        value.ownerLease = source.ownerLease;
        value.exact = source.exact && source.registrySequence != 0ULL &&
            source.entrySequence != 0ULL && source.dispatchId != 0ULL &&
            source.commitSequence != 0ULL && source.identity.IsAssigned() &&
            source.ownerLease.IsValid();
        std::uint64_t digest = HASH_OFFSET;
        digest = Hash(digest, value.registrySequence);
        digest = Hash(digest, value.entrySequence);
        digest = Hash(digest, value.dispatchId);
        digest = Hash(digest, value.commitSequence);
        digest = Hash(digest, static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(value.sourcePC)));
        digest = Hash(digest, static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(value.sourceLineNumber)));
        digest = Hash(digest, value.identity.epoch);
        digest = Hash(digest, value.identity.segmentId);
        digest = Hash(digest, static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(value.identity.sourceBlockId)));
        digest = Hash(digest, static_cast<std::uint64_t>(
            value.identity.source));
        digest = Hash(digest, static_cast<std::uint64_t>(
            value.ownerLease.owner));
        digest = Hash(digest, value.ownerLease.generation);
        value.lineageFingerprint = digest;
        if (withTerminal)
        {
            value.terminalFeedbackSequence = source.lastFeedbackSequence;
            value.terminalFeedbackType = source.lastFeedbackType;
            value.terminalOrdinal = source.terminalOrdinal;
            value.terminalFeedbackReceived = source.terminalObserved &&
                source.terminalCompleted &&
                source.lastFeedbackSequence != 0ULL;
            value.terminalCompleted = source.terminalCompleted;
        }
        return value;
    }

    static NCOrdinaryG00FeedHoldRollingGenerationSnapshot MakeGeneration(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        std::uint64_t ordinal,
        bool seed) noexcept
    {
        const bool released = cohort.terminalCohortComplete &&
            cutover.released;
        NCOrdinaryG00FeedHoldRollingGenerationSnapshot value{};
        value.generationOrdinal = ordinal;
        value.cohortPublicationAtCapture = cohort.publicationSequence;
        value.cutoverPublicationAtCapture = cutover.publicationSequence;
        value.registryPublicationAtCapture =
            cohort.registryPublicationSequence;
        value.cohortPublicationAtRelease = released
            ? cohort.publicationSequence : 0ULL;
        value.cutoverPublicationAtRelease = released
            ? cutover.publicationSequence : 0ULL;
        value.registryPublicationAtRelease = released
            ? registry.publicationSequence : 0ULL;
        value.cohortSequence = cohort.cohortSequence;
        value.boundarySequence = cohort.boundarySequence;
        value.gateSequence = cohort.gateSequence;
        value.session = cohort.session;
        value.executionEpoch = cohort.executionEpoch;
        value.owner = cohort.owner;
        value.ownerGeneration = cohort.ownerGeneration;
        value.memberCount = cohort.memberCount;
        value.terminalCount = cohort.terminalCount;
        value.seedEvidence = seed;
        value.rollingEvidence = !seed;
        value.captured = true;
        value.released = released;
        value.holdAcknowledged = cohort.holdAcknowledged;
        value.resumeRequested = cohort.resumeRequested;
        value.resumeApplied = cohort.resumeApplied;
        value.terminalOrderValid = cohort.terminalOrderValid;
        value.allMembersCompleted = cohort.allMembersCompleted;
        value.registryEmptyAtRelease = released &&
            registry.activeEntries == 0U;
        value.members[0] = MakeMember(cohort.members[0], released);
        value.members[1] = MakeMember(cohort.members[1], released);
        value.terminalFeedbackComplete = released &&
            value.members[0].terminalFeedbackReceived &&
            value.members[1].terminalFeedbackReceived;
        value.lineageDigest = GenerationDigest(value);
        return value;
    }

    static std::uint64_t GenerationDigest(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value) noexcept
    {
        std::uint64_t digest = HASH_OFFSET;
        digest = Hash(digest, value.cohortSequence);
        digest = Hash(digest, value.boundarySequence);
        digest = Hash(digest, value.gateSequence);
        digest = Hash(digest, value.members[0].lineageFingerprint);
        digest = Hash(digest, value.members[1].lineageFingerprint);
        if (value.released)
        {
            digest = Hash(digest,
                value.members[0].terminalFeedbackSequence);
            digest = Hash(digest, static_cast<std::uint64_t>(
                value.members[0].terminalFeedbackType));
            digest = Hash(digest,
                value.members[1].terminalFeedbackSequence);
            digest = Hash(digest, static_cast<std::uint64_t>(
                value.members[1].terminalFeedbackType));
        }
        return digest;
    }

    bool LineageAdvances(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value)
        const noexcept
    {
        const auto& first = value.members[0];
        const auto& second = value.members[1];
        return first.exact && second.exact &&
            first.identity.epoch == m_snapshot.executionEpoch &&
            second.identity.epoch == m_snapshot.executionEpoch &&
            first.identity.source == MotionCommandSource::NC_MEMORY &&
            second.identity.source == MotionCommandSource::NC_MEMORY &&
            first.ownerLease.owner == m_snapshot.owner &&
            second.ownerLease.owner == m_snapshot.owner &&
            first.ownerLease.generation == m_snapshot.ownerGeneration &&
            second.ownerLease.generation == m_snapshot.ownerGeneration &&
            first.registrySequence > m_snapshot.registrySequenceHighWater &&
            second.registrySequence > first.registrySequence &&
            first.entrySequence > m_snapshot.entrySequenceHighWater &&
            second.entrySequence > first.entrySequence &&
            first.dispatchId > m_snapshot.dispatchIdHighWater &&
            second.dispatchId > first.dispatchId &&
            first.commitSequence > m_snapshot.commitSequenceHighWater &&
            second.commitSequence > first.commitSequence &&
            first.identity.segmentId > m_snapshot.segmentIdHighWater &&
            second.identity.segmentId > first.identity.segmentId &&
            !SameIdentity(first.identity, second.identity);
    }

    static bool SameMember(
        const NCOrdinaryG00FeedHoldCohortMemberSnapshot& current,
        const NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot& captured)
        noexcept
    {
        return current.exact &&
            current.registrySequence == captured.registrySequence &&
            current.entrySequence == captured.entrySequence &&
            current.dispatchId == captured.dispatchId &&
            current.commitSequence == captured.commitSequence &&
            current.sourcePC == captured.sourcePC &&
            current.sourceLineNumber == captured.sourceLineNumber &&
            SameIdentity(current.identity, captured.identity) &&
            current.ownerLease.owner == captured.ownerLease.owner &&
            current.ownerLease.generation == captured.ownerLease.generation;
    }

    static bool FullMembersStable(
        const NCOrdinaryG00FeedHoldCohortSnapshot& cohort,
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& active) noexcept
    {
        return cohort.memberCount == 2U && cohort.exactTwoEntryCohort &&
            cohort.boundaryCorrelated &&
            cohort.members[0].session == active.session &&
            cohort.members[1].session == active.session &&
            SameMember(cohort.members[0], active.members[0]) &&
            SameMember(cohort.members[1], active.members[1]);
    }

    void RaiseHighWater(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value) noexcept
    {
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            Raise(m_snapshot.registrySequenceHighWater,
                value.members[i].registrySequence);
            Raise(m_snapshot.entrySequenceHighWater,
                value.members[i].entrySequence);
            Raise(m_snapshot.dispatchIdHighWater,
                value.members[i].dispatchId);
            Raise(m_snapshot.commitSequenceHighWater,
                value.members[i].commitSequence);
            Raise(m_snapshot.segmentIdHighWater,
                value.members[i].identity.segmentId);
        }
    }

    template <typename T>
    static void Raise(T& highWater, T value) noexcept
    {
        if (value > highWater)
        {
            highWater = value;
        }
    }

    bool ActiveInvariantExact() const noexcept
    {
        const bool totalsExact =
            m_snapshot.generationCount ==
            2ULL + m_snapshot.rollingCaptureCount &&
            m_snapshot.releaseCount ==
            2ULL + m_snapshot.rollingReleaseCount;
        const bool lastReleasedExact =
            m_snapshot.lastReleased.captured &&
            m_snapshot.lastReleased.released &&
            m_snapshot.lastReleased.generationOrdinal ==
            m_snapshot.releaseCount;
        return totalsExact && lastReleasedExact && (m_snapshot.active
            ? m_snapshot.generationCount ==
            m_snapshot.releaseCount + 1ULL &&
            m_snapshot.rollingCaptureCount ==
            m_snapshot.rollingReleaseCount + 1ULL &&
            m_snapshot.activeGenerationOrdinal != 0ULL &&
            m_snapshot.activeGenerationOrdinal ==
            m_snapshot.generationCount &&
            m_snapshot.currentActive.generationOrdinal ==
            m_snapshot.generationCount &&
            m_snapshot.currentActive.captured &&
            !m_snapshot.currentActive.released
            : m_snapshot.generationCount == m_snapshot.releaseCount &&
            m_snapshot.rollingCaptureCount ==
            m_snapshot.rollingReleaseCount &&
            m_snapshot.activeGenerationOrdinal == 0ULL &&
            !m_snapshot.currentActive.captured);
    }

    void SetWait(
        NCOrdinaryG00FeedHoldRollingRearmDecision decision) noexcept
    {
        if (m_snapshot.phase ==
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_SEED &&
            m_snapshot.decision == decision)
        {
            ++m_counters.stableObservations;
            return;
        }
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_SEED;
        m_snapshot.decision = decision;
        Publish();
    }

    bool ResetForChangedSession(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == m_snapshot.session)
        {
            return false;
        }
        const bool wasActive = m_snapshot.active;
        const std::uint64_t publication = m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldRollingRearmSnapshot next{};
        next.publicationSequence = publication;
        next.decision =
            NCOrdinaryG00FeedHoldRollingRearmDecision::K77_SESSION_RESET;
        m_snapshot = next;
        m_lastObservedCohortSequence = 0ULL;
        ++m_counters.sessionResets;
        if (wasActive)
        {
            ++m_counters.resetWhileActive;
        }
        Publish();
        return true;
    }

    void Fail(
        NCOrdinaryG00FeedHoldRollingRearmDecision decision,
        bool accountingValid) noexcept
    {
        if (!m_snapshot.failed)
        {
            ++m_counters.failures;
        }
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_FAILED;
        m_snapshot.decision = decision;
        m_snapshot.failed = true;
        m_snapshot.tracking = false;
        m_snapshot.accountingValid = accountingValid;
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
    NCOrdinaryG00FeedHoldRollingRearmSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldRollingRearmCounters m_counters{};
};

static_assert(sizeof(NCOrdinaryG00FeedHoldRollingRearmShadow) <= 1280U,
    "K.7.7 observer exceeded its fixed storage budget.");
