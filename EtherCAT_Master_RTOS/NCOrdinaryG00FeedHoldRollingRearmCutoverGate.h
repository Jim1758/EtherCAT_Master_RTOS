#pragma once

#include "NCOrdinaryG00FeedHoldRollingRearmShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.8
// Same-Session Rolling Feed-Hold Re-arm Continuity Controlled Cutover
//
// K.7.7 proves, without Runtime influence, that generation three and later
// preserve the same Session/lease, monotonic cohort lineage, exact two-member
// identity and release-before-rearm ordering.  K.7.8 consumes only that
// already-published proof at the existing K.7.1 ordinary G00 admission seam.
//
// Before K.7.7 is seeded, K.7.8 bypasses and leaves K.7.4/K.7.6 authoritative.
// Once a rolling generation is bound, admission waits until K.7.7 publishes
// the exact CONTINUITY_RELEASED/GENERATION_RELEASED proof and K.7.4 plus the
// terminal Registry independently agree.  Any disagreement locks the current
// Queue Session to the accepted legacy-drain path.
//
// This gate owns no Motion storage.  It never submits/cancels Motion, writes
// PDO or Feed Override, changes NC state/PC/callback/Epoch/owner, allocates,
// locks, waits or sleeps.
// =============================================================================

enum class NCOrdinaryG00FeedHoldRollingAdmissionResult : std::uint8_t
{
    BYPASS = 0,
    WAIT_ROLLING_REARM,
    ALLOW_READ_AHEAD,
    FALLBACK_LEGACY
};

enum class NCOrdinaryG00FeedHoldRollingCutoverPhase : std::uint8_t
{
    K78_IDLE = 0,
    K78_BYPASSED,
    K78_TRACKING,
    K78_GENERATION_BOUND,
    K78_WAIT_RELEASE,
    K78_CONTINUITY_RELEASED,
    K78_LEGACY_FALLBACK,
    K78_DISABLED
};

enum class NCOrdinaryG00FeedHoldRollingCutoverDecision : std::uint8_t
{
    K78_NONE = 0,
    K78_DISABLED,
    K78_BYPASS_WAIT_SEED,
    K78_BYPASS_NO_ACTIVE_GENERATION,
    K78_TRACKING_ARMED,
    K78_GENERATION_BOUND,
    K78_WAIT_K74_RELEASE,
    K78_WAIT_K77_CONTINUITY_PROOF,
    K78_ALLOW_READ_AHEAD,
    K78_FALLBACK_K77_FAILED,
    K78_FALLBACK_K74,
    K78_FALLBACK_SESSION,
    K78_FALLBACK_EXECUTION_LEASE,
    K78_FALLBACK_SEQUENCE,
    K78_FALLBACK_LINEAGE,
    K78_FALLBACK_IDENTITY,
    K78_FALLBACK_REGISTRY,
    K78_FALLBACK_ACCOUNTING,
    K78_FALLBACK_ACTIVE_INVARIANT,
    K78_SESSION_RESET
};

inline const char* NCOrdinaryG00FeedHoldRollingCutoverPhaseName(
    NCOrdinaryG00FeedHoldRollingCutoverPhase value) noexcept
{
    switch (value)
    {
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_BYPASSED:
        return "BYPASSED";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_TRACKING:
        return "TRACKING";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_GENERATION_BOUND:
        return "GENERATION_BOUND";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_WAIT_RELEASE:
        return "WAIT_RELEASE";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_CONTINUITY_RELEASED:
        return "CONTINUITY_RELEASED";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_LEGACY_FALLBACK:
        return "LEGACY_FALLBACK";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_DISABLED:
        return "DISABLED";
    case NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_IDLE:
    default:
        return "IDLE";
    }
}

inline const char* NCOrdinaryG00FeedHoldRollingCutoverDecisionName(
    NCOrdinaryG00FeedHoldRollingCutoverDecision value) noexcept
{
    switch (value)
    {
    case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_DISABLED:
        return "DISABLED";
    case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_BYPASS_WAIT_SEED:
        return "BYPASS_WAIT_SEED";
        case NCOrdinaryG00FeedHoldRollingCutoverDecision::
        K78_BYPASS_NO_ACTIVE_GENERATION:
            return "BYPASS_NO_ACTIVE";
        case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_TRACKING_ARMED:
            return "TRACKING_ARMED";
        case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_GENERATION_BOUND:
            return "GENERATION_BOUND";
        case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_WAIT_K74_RELEASE:
            return "WAIT_K74_RELEASE";
            case NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K77_CONTINUITY_PROOF:
                return "WAIT_K77_PROOF";
            case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_ALLOW_READ_AHEAD:
                return "ALLOW_READ_AHEAD";
            case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_K77_FAILED:
                return "FALLBACK_K77_FAILED";
            case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_K74:
                return "FALLBACK_K74";
            case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_SESSION:
                return "FALLBACK_SESSION";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_EXECUTION_LEASE:
                    return "FALLBACK_LEASE";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_SEQUENCE:
                    return "FALLBACK_SEQUENCE";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_LINEAGE:
                    return "FALLBACK_LINEAGE";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_IDENTITY:
                    return "FALLBACK_IDENTITY";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_REGISTRY:
                    return "FALLBACK_REGISTRY";
                case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_FALLBACK_ACCOUNTING:
                    return "FALLBACK_ACCOUNTING";
                    case NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_ACTIVE_INVARIANT:
                        return "FALLBACK_ACTIVE_INV";
                    case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_SESSION_RESET:
                        return "SESSION_RESET";
                    case NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_NONE:
                    default:
                        return "NONE";
    }
}

struct NCOrdinaryG00FeedHoldRollingCutoverSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t rollingPublicationSequence = 0ULL;
    std::uint64_t cohortCutoverPublicationSequence = 0ULL;
    std::uint64_t registryPublicationSequence = 0ULL;
    std::uint64_t generationOrdinal = 0ULL;
    std::uint64_t lastReleasedGenerationOrdinal = 0ULL;
    std::uint64_t generationCount = 0ULL;
    std::uint64_t releaseCount = 0ULL;
    std::uint64_t cohortSequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;
    std::uint64_t gateSequence = 0ULL;
    std::uint64_t lineageDigest = 0ULL;
    std::uint64_t generationCohortPublicationAtCapture = 0ULL;
    std::uint64_t generationCutoverPublicationAtCapture = 0ULL;
    std::uint64_t generationRegistryPublicationAtCapture = 0ULL;
    std::uint64_t generationCohortPublicationAtRelease = 0ULL;
    std::uint64_t generationCutoverPublicationAtRelease = 0ULL;
    std::uint64_t generationRegistryPublicationAtRelease = 0ULL;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::array<std::uint64_t,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberRegistrySequence{};
    std::array<NCPreparedEntrySequence,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberEntrySequence{};
    std::array<NCBlockDispatchId,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberDispatchId{};
    std::array<NCProgramCommitSequence,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberCommitSequence{};
    std::array<MotionExecutionIdentity,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberIdentity{};
    std::array<MotionOwnerLease,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberOwnerLease{};
    std::array<std::uint64_t,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberLineageFingerprint{};
    std::array<int,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberSourcePC{ { -1, -1 } };
    std::array<int,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberSourceLineNumber{};
    std::array<MotionFeedbackSequence,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE>
        memberTerminalFeedbackSequence{};
    std::array<MotionFeedbackType,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE>
        memberTerminalFeedbackType{};
    std::array<std::uint8_t,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> memberTerminalOrdinal{};

    NCOrdinaryG00FeedHoldCohortCutoverPhase observedCutoverPhase =
        NCOrdinaryG00FeedHoldCohortCutoverPhase::IDLE;
    NCOrdinaryG00FeedHoldCohortCutoverDecision observedCutoverDecision =
        NCOrdinaryG00FeedHoldCohortCutoverDecision::NONE;
    std::uint64_t observedCutoverCohortSequence = 0ULL;
    std::uint64_t observedCutoverBoundarySequence = 0ULL;
    std::uint64_t observedCutoverGateSequence = 0ULL;
    std::uint64_t observedCutoverRegistryPublicationSequence = 0ULL;
    std::uint32_t observedCutoverActiveEntriesAtCapture = 0U;
    std::uint32_t observedCutoverRegistryActiveEntries = 0U;
    std::uint8_t observedCutoverTerminalCount = 0U;
    bool observedCutoverHoldAcknowledged = false;
    bool observedCutoverResumeRequested = false;
    bool observedCutoverResumeApplied = false;
    bool observedCutoverTerminalOrderValid = true;
    bool observedCutoverWaiting = false;
    bool observedCutoverReleased = false;
    bool observedCutoverRuntimeInfluence = false;
    bool observedCutoverResolverBypassed = false;

    NCOrdinaryG00FeedHoldRollingRearmPhase observedRollingPhase =
        NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_SEED;
    NCOrdinaryG00FeedHoldRollingRearmDecision observedRollingDecision =
        NCOrdinaryG00FeedHoldRollingRearmDecision::K77_WAIT_K75_SEED;
    std::uint64_t observedRollingGateSequence = 0ULL;
    std::uint8_t observedRollingTerminalCount = 0U;
    bool observedRollingHoldAcknowledged = false;
    bool observedRollingResumeRequested = false;
    bool observedRollingResumeApplied = false;
    bool observedRollingTerminalOrderValid = true;

    NCOrdinaryG00FeedHoldRollingCutoverPhase phase =
        NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_IDLE;
    NCOrdinaryG00FeedHoldRollingCutoverDecision decision =
        NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_NONE;

    bool enabled = true;
    bool trackingSeeded = false;
    bool exactGeneration = false;
    bool bound = false;
    bool waiting = false;
    bool released = false;
    bool fallbackLegacy = false;
    bool sessionLockout = false;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldRollingCutoverCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t trackingSeeds = 0ULL;
    std::uint64_t generationsBound = 0ULL;
    std::uint64_t bypassDisabled = 0ULL;
    std::uint64_t bypassWaitSeed = 0ULL;
    std::uint64_t bypassNoActiveGeneration = 0ULL;
    std::uint64_t staleObservations = 0ULL;
    std::uint64_t admissionChecks = 0ULL;
    std::uint64_t admissionBypasses = 0ULL;
    std::uint64_t waitK74Release = 0ULL;
    std::uint64_t waitK77ContinuityProof = 0ULL;
    std::uint64_t releases = 0ULL;
    std::uint64_t allowReadAhead = 0ULL;
    std::uint64_t fallbackLegacy = 0ULL;
    std::uint64_t k77Failures = 0ULL;
    std::uint64_t k74Failures = 0ULL;
    std::uint64_t sessionMismatches = 0ULL;
    std::uint64_t executionLeaseMismatches = 0ULL;
    std::uint64_t sequenceMismatches = 0ULL;
    std::uint64_t lineageMismatches = 0ULL;
    std::uint64_t identityMismatches = 0ULL;
    std::uint64_t registryMismatches = 0ULL;
    std::uint64_t accountingMismatches = 0ULL;
    std::uint64_t activeInvariantMismatches = 0ULL;
    std::uint64_t sessionResets = 0ULL;
    std::uint64_t failures = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingCutoverSnapshot>::value,
    "K.7.8 rolling cutover snapshot must remain trivially copyable.");
static_assert(std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldRollingCutoverCounters>::value,
    "K.7.8 rolling cutover counters must remain trivially copyable.");
static_assert(sizeof(NCOrdinaryG00FeedHoldRollingCutoverSnapshot) <= 512U,
    "K.7.8 rolling cutover snapshot exceeded its fixed budget.");
static_assert(sizeof(NCOrdinaryG00FeedHoldRollingCutoverCounters) <= 256U,
    "K.7.8 rolling cutover counters exceeded their fixed budget.");

class NCOrdinaryG00FeedHoldRollingRearmCutoverGate
{
public:
    NCOrdinaryG00FeedHoldRollingRearmCutoverGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept
    {
        if (m_enabled == enabled)
        {
            return;
        }
        m_enabled = enabled;
        m_lastObservedRollingPublicationSequence = 0ULL;
        if (!enabled)
        {
            Clear(false);
            m_snapshot.enabled = false;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_DISABLED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_DISABLED;
        }
        else
        {
            Clear(false);
            m_snapshot.enabled = true;
        }
        Publish();
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    void Observe(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
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

        if (ResetForChangedSession(registry))
        {
            return;
        }

        if (rolling.publicationSequence != 0ULL &&
            m_lastObservedRollingPublicationSequence != 0ULL &&
            rolling.publicationSequence <
            m_lastObservedRollingPublicationSequence)
        {
            ++m_counters.staleObservations;
            if (m_snapshot.trackingSeeded || m_snapshot.bound ||
                m_snapshot.released)
            {
                ++m_counters.sequenceMismatches;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_SEQUENCE,
                    rolling.accountingValid && registry.accountingValid);
            }
            return;
        }
        if (rolling.publicationSequence != 0ULL)
        {
            m_lastObservedRollingPublicationSequence =
                rolling.publicationSequence;
        }

        if (m_snapshot.sessionLockout)
        {
            return;
        }

        if (rolling.failed)
        {
            AdoptFallbackSession(rolling.session, registry);
            ++m_counters.k77Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K77_FAILED,
                rolling.accountingValid && registry.accountingValid);
            return;
        }

        if (!rolling.seeded)
        {
            if (m_snapshot.trackingSeeded || m_snapshot.bound ||
                m_snapshot.released)
            {
                ++m_counters.sequenceMismatches;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_SEQUENCE,
                    rolling.accountingValid && registry.accountingValid);
                return;
            }
            SetBypass(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_BYPASS_WAIT_SEED,
                m_counters.bypassWaitSeed);
            return;
        }

        if (!rolling.accountingValid || !registry.accountingValid ||
            !cohortCutover.accountingValid)
        {
            AdoptFallbackSession(rolling.session, registry);
            ++m_counters.accountingMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACCOUNTING,
                false);
            return;
        }
        if (!RollingCoreExact(rolling))
        {
            ClassifyRollingFailure(rolling, registry);
            return;
        }
        if (!RegistryBasic(registry) ||
            registry.currentSession != rolling.session)
        {
            AdoptFallbackSession(rolling.session, registry);
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_REGISTRY,
                registry.accountingValid);
            return;
        }

        if (!m_snapshot.trackingSeeded)
        {
            SeedTracking(rolling, registry);
        }
        if (registry.publicationSequence <
            m_snapshot.registryPublicationSequence)
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_REGISTRY,
                true);
            return;
        }
        if (m_snapshot.cohortCutoverPublicationSequence != 0ULL &&
            cohortCutover.publicationSequence != 0ULL &&
            cohortCutover.publicationSequence <
            m_snapshot.cohortCutoverPublicationSequence)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                true);
            return;
        }
        if (registry.publicationSequence >
            m_snapshot.registryPublicationSequence)
        {
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
        }
        if (!TrackingStable(rolling))
        {
            ClassifyTrackingFailure(rolling);
            return;
        }
        if (m_snapshot.bound &&
            !cohortCutover.fallbackLegacy &&
            !cohortCutover.sessionLockout &&
            cohortCutover.enabled && cohortCutover.exactCohort &&
            cohortCutover.bound && cohortCutover.accountingValid &&
            !cohortCutover.motionWrite &&
            cohortCutover.publicationSequence ==
            m_snapshot.cohortCutoverPublicationSequence &&
            !ObservedCutoverStateStable(cohortCutover))
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                cohortCutover.accountingValid);
            return;
        }

        if (m_snapshot.bound)
        {
            ObserveBound(rolling, cohortCutover, registry);
            return;
        }

        if (rolling.active)
        {
            if (rolling.publicationSequence <=
                m_snapshot.rollingPublicationSequence ||
                (m_snapshot.cohortCutoverPublicationSequence != 0ULL &&
                    cohortCutover.publicationSequence <=
                    m_snapshot.cohortCutoverPublicationSequence))
            {
                ++m_counters.sequenceMismatches;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_SEQUENCE,
                    true);
                return;
            }
            if (!ActiveGenerationExact(rolling, cohortCutover, registry))
            {
                ClassifyActiveFailure(rolling, cohortCutover, registry);
                return;
            }
            BindGeneration(rolling, cohortCutover, registry);
            return;
        }

        if (!ReleasedIdleExact(rolling, registry))
        {
            ++m_counters.activeInvariantMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACTIVE_INVARIANT,
                true);
            return;
        }
        if (m_snapshot.released &&
            !ReleasedGenerationStable(rolling))
        {
            ++m_counters.lineageMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_LINEAGE,
                true);
            return;
        }
        if (m_snapshot.released &&
            !ReleasedCutoverStable(rolling, cohortCutover, registry))
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K74,
                cohortCutover.accountingValid);
            return;
        }
        if (rolling.releaseCount !=
            m_snapshot.lastReleasedGenerationOrdinal)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                true);
            return;
        }
        m_snapshot.rollingPublicationSequence = rolling.publicationSequence;
        m_snapshot.cohortCutoverPublicationSequence =
            cohortCutover.publicationSequence;
        m_snapshot.registryPublicationSequence = registry.publicationSequence;
        CaptureObservedCutoverState(cohortCutover);
        m_snapshot.generationCount = rolling.generationCount;
        m_snapshot.releaseCount = rolling.releaseCount;
        if (!m_snapshot.released)
        {
            SetBypass(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_BYPASS_NO_ACTIVE_GENERATION,
                m_counters.bypassNoActiveGeneration);
        }
    }

    NCOrdinaryG00FeedHoldRollingAdmissionResult EvaluateAdmission(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        ++m_counters.admissionChecks;
        if (!m_enabled)
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldRollingAdmissionResult::BYPASS;
        }

        ResetForChangedSession(registry);
        const bool authorityActive =
            m_snapshot.trackingSeeded || m_snapshot.bound ||
            m_snapshot.released;
        if (!m_snapshot.sessionLockout && authorityActive &&
            !registry.accountingValid)
        {
            ++m_counters.accountingMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACCOUNTING,
                false);
        }
        else if (!m_snapshot.sessionLockout && authorityActive &&
            (!RegistryBasic(registry) ||
                registry.currentSession != m_snapshot.session ||
                registry.publicationSequence <
                m_snapshot.registryPublicationSequence))
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_REGISTRY,
                registry.accountingValid);
        }
        if (!m_snapshot.sessionLockout && authorityActive &&
            registry.publicationSequence >
            m_snapshot.registryPublicationSequence)
        {
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
        }
        if (m_snapshot.sessionLockout || m_snapshot.fallbackLegacy)
        {
            ++m_counters.fallbackLegacy;
            const bool changed = !m_snapshot.runtimeInfluence;
            RecordInfluence();
            if (changed)
            {
                Publish();
            }
            return
                NCOrdinaryG00FeedHoldRollingAdmissionResult::FALLBACK_LEGACY;
        }
        if (!m_snapshot.trackingSeeded ||
            (!m_snapshot.bound && !m_snapshot.released))
        {
            ++m_counters.admissionBypasses;
            return NCOrdinaryG00FeedHoldRollingAdmissionResult::BYPASS;
        }
        if (m_snapshot.released && !m_snapshot.bound)
        {
            ++m_counters.allowReadAhead;
            const bool changed =
                m_snapshot.decision !=
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_ALLOW_READ_AHEAD ||
                m_snapshot.phase !=
                NCOrdinaryG00FeedHoldRollingCutoverPhase::
                K78_CONTINUITY_RELEASED ||
                !m_snapshot.runtimeInfluence;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_ALLOW_READ_AHEAD;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldRollingCutoverPhase::
                K78_CONTINUITY_RELEASED;
            RecordInfluence();
            if (changed)
            {
                Publish();
            }
            return
                NCOrdinaryG00FeedHoldRollingAdmissionResult::ALLOW_READ_AHEAD;
        }

        const bool waitForK77 =
            m_snapshot.decision ==
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K77_CONTINUITY_PROOF;
        if (waitForK77)
        {
            ++m_counters.waitK77ContinuityProof;
        }
        else
        {
            ++m_counters.waitK74Release;
        }
        const NCOrdinaryG00FeedHoldRollingCutoverDecision nextDecision =
            waitForK77
            ? NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K77_CONTINUITY_PROOF
            : NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K74_RELEASE;
        const bool changed =
            m_snapshot.phase !=
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_WAIT_RELEASE ||
            m_snapshot.decision != nextDecision || !m_snapshot.waiting ||
            !m_snapshot.runtimeInfluence;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_WAIT_RELEASE;
        m_snapshot.decision = nextDecision;
        m_snapshot.waiting = true;
        RecordInfluence();
        if (changed)
        {
            Publish();
        }
        return
            NCOrdinaryG00FeedHoldRollingAdmissionResult::WAIT_ROLLING_REARM;
    }

    NCOrdinaryG00FeedHoldRollingCutoverSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldRollingCutoverCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
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
            registry.publicationSequence != 0ULL &&
            registry.publicationSequence !=
            (std::numeric_limits<std::uint64_t>::max)() &&
            registry.bounded &&
            !registry.motionWrite && registry.accountingValid &&
            registry.capacity ==
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY &&
            registry.activeEntries <= registry.occupiedEntries &&
            registry.occupiedEntries <= registry.capacity &&
            registry.revokedPendingEntries <= registry.activeEntries &&
            registry.activeEntries <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
    }

    static bool RollingMemberCaptureExact(
        const NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot& value,
        MotionExecutionEpoch executionEpoch,
        MotionOwner owner,
        MotionOwnerGeneration ownerGeneration)
        noexcept
    {
        return value.exact && value.registrySequence != 0ULL &&
            value.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            value.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            value.commitSequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            value.identity.IsAssigned() &&
            value.identity.epoch == executionEpoch &&
            value.identity.source == MotionCommandSource::NC_MEMORY &&
            value.ownerLease.IsValid() &&
            value.ownerLease.owner == owner &&
            value.ownerLease.generation == ownerGeneration;
    }

    static bool RollingMemberReleaseExact(
        const NCOrdinaryG00FeedHoldRollingMemberLineageSnapshot& value,
        MotionExecutionEpoch executionEpoch,
        MotionOwner owner,
        MotionOwnerGeneration ownerGeneration)
        noexcept
    {
        return RollingMemberCaptureExact(
            value, executionEpoch, owner, ownerGeneration) &&
            value.terminalFeedbackReceived && value.terminalCompleted &&
            value.terminalFeedbackSequence !=
            MOTION_FEEDBACK_SEQUENCE_INVALID &&
            value.terminalFeedbackType == MotionFeedbackType::COMPLETED &&
            value.terminalOrdinal != 0U;
    }

    static bool GenerationCaptureExact(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value)
        noexcept
    {
        return value.generationOrdinal >= 3ULL && value.rollingEvidence &&
            !value.seedEvidence && value.captured && !value.released &&
            value.memberCount == 2U && value.terminalCount <= 2U &&
            value.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            value.executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            value.owner == MotionOwner::AUTO &&
            value.ownerGeneration != MOTION_OWNER_GENERATION_INVALID &&
            value.cohortPublicationAtCapture != 0ULL &&
            value.cutoverPublicationAtCapture != 0ULL &&
            value.registryPublicationAtCapture != 0ULL &&
            value.cohortSequence != 0ULL &&
            value.boundarySequence != 0ULL &&
            RollingMemberCaptureExact(
                value.members[0], value.executionEpoch,
                value.owner, value.ownerGeneration) &&
            RollingMemberCaptureExact(
                value.members[1], value.executionEpoch,
                value.owner, value.ownerGeneration) &&
            !SameIdentity(value.members[0].identity,
                value.members[1].identity);
    }

    static bool GenerationReleaseExact(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value)
        noexcept
    {
        return value.generationOrdinal >= 3ULL && value.rollingEvidence &&
            !value.seedEvidence && value.captured && value.released &&
            value.memberCount == 2U && value.terminalCount == 2U &&
            value.holdAcknowledged && value.resumeRequested &&
            value.resumeApplied && value.terminalOrderValid &&
            value.allMembersCompleted && value.terminalFeedbackComplete &&
            value.registryEmptyAtRelease &&
            value.cohortPublicationAtCapture != 0ULL &&
            value.cutoverPublicationAtCapture != 0ULL &&
            value.registryPublicationAtCapture != 0ULL &&
            value.cohortPublicationAtRelease >
            value.cohortPublicationAtCapture &&
            value.cutoverPublicationAtRelease >
            value.cutoverPublicationAtCapture &&
            value.registryPublicationAtRelease >
            value.registryPublicationAtCapture &&
            value.gateSequence != 0ULL &&
            RollingMemberReleaseExact(
                value.members[0], value.executionEpoch,
                value.owner, value.ownerGeneration) &&
            RollingMemberReleaseExact(
                value.members[1], value.executionEpoch,
                value.owner, value.ownerGeneration) &&
            value.members[0].terminalOrdinal == 1U &&
            value.members[1].terminalOrdinal == 2U &&
            value.members[1].terminalFeedbackSequence >
            value.members[0].terminalFeedbackSequence;
    }

    static bool RollingCoreExact(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling) noexcept
    {
        return rolling.seeded && rolling.tracking && !rolling.failed &&
            rolling.bounded && rolling.shadowOnly &&
            !rolling.runtimeInfluence && !rolling.motionWrite &&
            rolling.accountingValid && rolling.activeInvariantValid &&
            rolling.sameSession && rolling.sameExecutionLease &&
            rolling.sequenceMonotonic && rolling.fullLineageValid &&
            rolling.noOverlap &&
            rolling.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            rolling.executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            rolling.owner == MotionOwner::AUTO &&
            rolling.ownerGeneration != MOTION_OWNER_GENERATION_INVALID &&
            rolling.publicationSequence != 0ULL &&
            rolling.generationCount >= 2ULL &&
            rolling.releaseCount >= 2ULL &&
            rolling.generationCount ==
            2ULL + rolling.rollingCaptureCount &&
            rolling.releaseCount ==
            2ULL + rolling.rollingReleaseCount &&
            rolling.releaseCount <= rolling.generationCount;
    }

    static bool ReleasedIdleExact(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const bool phaseExact = rolling.releaseCount == 2ULL
            ? rolling.phase ==
            NCOrdinaryG00FeedHoldRollingRearmPhase::K77_SEEDED &&
            rolling.decision ==
            NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_SEED_ACCEPTED && !rolling.releasedContinuity
            : rolling.phase ==
            NCOrdinaryG00FeedHoldRollingRearmPhase::
            K77_CONTINUITY_RELEASED &&
            rolling.decision ==
            NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_GENERATION_RELEASED && rolling.releasedContinuity;
        return phaseExact && !rolling.active &&
            rolling.activeGenerationOrdinal == 0ULL &&
            rolling.generationCount == rolling.releaseCount &&
            rolling.rollingCaptureCount == rolling.rollingReleaseCount &&
            rolling.currentActive.captured == false &&
            rolling.lastReleased.captured && rolling.lastReleased.released &&
            rolling.lastReleased.generationOrdinal == rolling.releaseCount &&
            rolling.lastReleased.session == rolling.session &&
            registry.currentSession == rolling.session;
    }

    static bool SameLease(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& lhs,
        const NCOrdinaryG00FeedHoldRollingCutoverSnapshot& rhs) noexcept
    {
        return lhs.executionEpoch == rhs.executionEpoch &&
            lhs.owner == rhs.owner &&
            lhs.ownerGeneration == rhs.ownerGeneration;
    }

    bool TrackingStable(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling) const
        noexcept
    {
        return rolling.session == m_snapshot.session &&
            SameLease(rolling, m_snapshot) &&
            rolling.generationCount >=
            m_snapshot.lastReleasedGenerationOrdinal &&
            rolling.releaseCount >=
            m_snapshot.lastReleasedGenerationOrdinal;
    }

    bool BoundGenerationStable(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value) const
        noexcept
    {
        if (!GenerationCaptureExact(value) ||
            value.generationOrdinal != m_snapshot.generationOrdinal ||
            value.session != m_snapshot.session ||
            value.executionEpoch != m_snapshot.executionEpoch ||
            value.owner != m_snapshot.owner ||
            value.ownerGeneration != m_snapshot.ownerGeneration ||
            value.cohortSequence != m_snapshot.cohortSequence ||
            value.boundarySequence != m_snapshot.boundarySequence ||
            value.cohortPublicationAtCapture !=
            m_snapshot.generationCohortPublicationAtCapture ||
            value.cutoverPublicationAtCapture !=
            m_snapshot.generationCutoverPublicationAtCapture ||
            value.registryPublicationAtCapture !=
            m_snapshot.generationRegistryPublicationAtCapture ||
            value.lineageDigest != m_snapshot.lineageDigest)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (value.members[i].registrySequence !=
                m_snapshot.memberRegistrySequence[i] ||
                value.members[i].entrySequence !=
                m_snapshot.memberEntrySequence[i] ||
                value.members[i].dispatchId !=
                m_snapshot.memberDispatchId[i] ||
                value.members[i].commitSequence !=
                m_snapshot.memberCommitSequence[i] ||
                !SameIdentity(value.members[i].identity,
                    m_snapshot.memberIdentity[i]) ||
                value.members[i].ownerLease.owner !=
                m_snapshot.memberOwnerLease[i].owner ||
                value.members[i].ownerLease.generation !=
                m_snapshot.memberOwnerLease[i].generation ||
                value.members[i].lineageFingerprint !=
                m_snapshot.memberLineageFingerprint[i] ||
                value.members[i].sourcePC !=
                m_snapshot.memberSourcePC[i] ||
                value.members[i].sourceLineNumber !=
                m_snapshot.memberSourceLineNumber[i])
            {
                return false;
            }
        }
        return true;
    }

    bool ReleasedGenerationStable(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling) const
        noexcept
    {
        const auto& value = rolling.lastReleased;
        if (!GenerationReleaseExact(value) ||
            value.generationOrdinal !=
            m_snapshot.lastReleasedGenerationOrdinal ||
            value.session != m_snapshot.session ||
            value.executionEpoch != m_snapshot.executionEpoch ||
            value.owner != m_snapshot.owner ||
            value.ownerGeneration != m_snapshot.ownerGeneration ||
            value.cohortSequence != m_snapshot.cohortSequence ||
            value.boundarySequence != m_snapshot.boundarySequence ||
            value.gateSequence != m_snapshot.gateSequence ||
            value.lineageDigest != m_snapshot.lineageDigest ||
            value.cohortPublicationAtCapture !=
            m_snapshot.generationCohortPublicationAtCapture ||
            value.cutoverPublicationAtCapture !=
            m_snapshot.generationCutoverPublicationAtCapture ||
            value.registryPublicationAtCapture !=
            m_snapshot.generationRegistryPublicationAtCapture ||
            value.cohortPublicationAtRelease !=
            m_snapshot.generationCohortPublicationAtRelease ||
            value.cutoverPublicationAtRelease !=
            m_snapshot.generationCutoverPublicationAtRelease ||
            value.registryPublicationAtRelease !=
            m_snapshot.generationRegistryPublicationAtRelease ||
            rolling.lineageDigest != value.lineageDigest ||
            rolling.lastCohortSequence != value.cohortSequence ||
            rolling.lastBoundarySequence != value.boundarySequence ||
            rolling.lastGateSequence != value.gateSequence)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (value.members[i].registrySequence !=
                m_snapshot.memberRegistrySequence[i] ||
                value.members[i].entrySequence !=
                m_snapshot.memberEntrySequence[i] ||
                value.members[i].dispatchId !=
                m_snapshot.memberDispatchId[i] ||
                value.members[i].commitSequence !=
                m_snapshot.memberCommitSequence[i] ||
                !SameIdentity(value.members[i].identity,
                    m_snapshot.memberIdentity[i]) ||
                value.members[i].ownerLease.owner !=
                m_snapshot.memberOwnerLease[i].owner ||
                value.members[i].ownerLease.generation !=
                m_snapshot.memberOwnerLease[i].generation ||
                value.members[i].lineageFingerprint !=
                m_snapshot.memberLineageFingerprint[i] ||
                value.members[i].sourcePC !=
                m_snapshot.memberSourcePC[i] ||
                value.members[i].sourceLineNumber !=
                m_snapshot.memberSourceLineNumber[i] ||
                value.members[i].terminalFeedbackSequence !=
                m_snapshot.memberTerminalFeedbackSequence[i] ||
                value.members[i].terminalFeedbackType !=
                m_snapshot.memberTerminalFeedbackType[i] ||
                value.members[i].terminalOrdinal !=
                m_snapshot.memberTerminalOrdinal[i])
            {
                return false;
            }
        }
        return true;
    }

    bool ObservedCutoverStateStable(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover)
        const noexcept
    {
        return cutover.phase == m_snapshot.observedCutoverPhase &&
            cutover.decision == m_snapshot.observedCutoverDecision &&
            cutover.cohortSequence ==
            m_snapshot.observedCutoverCohortSequence &&
            cutover.boundarySequence ==
            m_snapshot.observedCutoverBoundarySequence &&
            cutover.gateSequence ==
            m_snapshot.observedCutoverGateSequence &&
            cutover.registryPublicationSequence ==
            m_snapshot.observedCutoverRegistryPublicationSequence &&
            cutover.activeEntriesAtCapture ==
            m_snapshot.observedCutoverActiveEntriesAtCapture &&
            cutover.registryActiveEntries ==
            m_snapshot.observedCutoverRegistryActiveEntries &&
            cutover.terminalCount ==
            m_snapshot.observedCutoverTerminalCount &&
            cutover.holdAcknowledged ==
            m_snapshot.observedCutoverHoldAcknowledged &&
            cutover.resumeRequested ==
            m_snapshot.observedCutoverResumeRequested &&
            cutover.resumeApplied ==
            m_snapshot.observedCutoverResumeApplied &&
            cutover.terminalOrderValid ==
            m_snapshot.observedCutoverTerminalOrderValid &&
            cutover.waiting == m_snapshot.observedCutoverWaiting &&
            cutover.released == m_snapshot.observedCutoverReleased &&
            cutover.runtimeInfluence ==
            m_snapshot.observedCutoverRuntimeInfluence &&
            cutover.resolverBypassed ==
            m_snapshot.observedCutoverResolverBypassed;
    }

    void CaptureObservedCutoverState(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover) noexcept
    {
        m_snapshot.observedCutoverPhase = cutover.phase;
        m_snapshot.observedCutoverDecision = cutover.decision;
        m_snapshot.observedCutoverCohortSequence =
            cutover.cohortSequence;
        m_snapshot.observedCutoverBoundarySequence =
            cutover.boundarySequence;
        m_snapshot.observedCutoverGateSequence = cutover.gateSequence;
        m_snapshot.observedCutoverRegistryPublicationSequence =
            cutover.registryPublicationSequence;
        m_snapshot.observedCutoverActiveEntriesAtCapture =
            cutover.activeEntriesAtCapture;
        m_snapshot.observedCutoverRegistryActiveEntries =
            cutover.registryActiveEntries;
        m_snapshot.observedCutoverTerminalCount = cutover.terminalCount;
        m_snapshot.observedCutoverHoldAcknowledged =
            cutover.holdAcknowledged;
        m_snapshot.observedCutoverResumeRequested =
            cutover.resumeRequested;
        m_snapshot.observedCutoverResumeApplied = cutover.resumeApplied;
        m_snapshot.observedCutoverTerminalOrderValid =
            cutover.terminalOrderValid;
        m_snapshot.observedCutoverWaiting = cutover.waiting;
        m_snapshot.observedCutoverReleased = cutover.released;
        m_snapshot.observedCutoverRuntimeInfluence =
            cutover.runtimeInfluence;
        m_snapshot.observedCutoverResolverBypassed =
            cutover.resolverBypassed;
    }

    bool ObservedRollingActiveStateStable(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling)
        const noexcept
    {
        const auto& active = rolling.currentActive;
        return rolling.phase == m_snapshot.observedRollingPhase &&
            rolling.decision == m_snapshot.observedRollingDecision &&
            active.gateSequence ==
            m_snapshot.observedRollingGateSequence &&
            active.terminalCount ==
            m_snapshot.observedRollingTerminalCount &&
            active.holdAcknowledged ==
            m_snapshot.observedRollingHoldAcknowledged &&
            active.resumeRequested ==
            m_snapshot.observedRollingResumeRequested &&
            active.resumeApplied ==
            m_snapshot.observedRollingResumeApplied &&
            active.terminalOrderValid ==
            m_snapshot.observedRollingTerminalOrderValid;
    }

    void CaptureObservedRollingState(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling) noexcept
    {
        const auto& active = rolling.currentActive;
        m_snapshot.observedRollingPhase = rolling.phase;
        m_snapshot.observedRollingDecision = rolling.decision;
        m_snapshot.observedRollingGateSequence = active.gateSequence;
        m_snapshot.observedRollingTerminalCount = active.terminalCount;
        m_snapshot.observedRollingHoldAcknowledged =
            active.holdAcknowledged;
        m_snapshot.observedRollingResumeRequested = active.resumeRequested;
        m_snapshot.observedRollingResumeApplied = active.resumeApplied;
        m_snapshot.observedRollingTerminalOrderValid =
            active.terminalOrderValid;
    }

    bool ReleasedCutoverStable(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry)
        const noexcept
    {
        const auto& released = rolling.lastReleased;
        const bool previouslyAcceptedBypass =
            m_snapshot.observedCutoverPhase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            m_snapshot.observedCutoverDecision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            BYPASS_NO_EXACT_COHORT;
        const bool equalReleasedPublicationStable =
            cutover.publicationSequence !=
            m_snapshot.cohortCutoverPublicationSequence ||
            ObservedCutoverStateStable(cutover);
        const bool exactReleasedProof =
            !previouslyAcceptedBypass &&
            CohortCutoverReleasedExact(cutover) &&
            cutover.publicationSequence >=
            m_snapshot.generationCutoverPublicationAtRelease &&
            cutover.publicationSequence >=
            released.cutoverPublicationAtRelease &&
            cutover.gateSequence == m_snapshot.gateSequence &&
            cutover.gateSequence == released.gateSequence &&
            cutover.registryPublicationSequence ==
            m_snapshot.generationRegistryPublicationAtRelease &&
            cutover.registryPublicationSequence ==
            released.registryPublicationAtRelease &&
            !cutover.resolverBypassed &&
            (!m_snapshot.observedCutoverRuntimeInfluence ||
                cutover.runtimeInfluence) &&
            equalReleasedPublicationStable;
        if (exactReleasedProof)
        {
            return true;
        }

        return SafePostReleaseCutoverBypass(cutover, registry);
    }

    bool SafePostReleaseCutoverBypass(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry)
        const noexcept
    {
        const bool previouslyAcceptedBypass =
            m_snapshot.observedCutoverPhase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            m_snapshot.observedCutoverDecision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            BYPASS_NO_EXACT_COHORT;
        const bool sameAcceptedPublication =
            cutover.publicationSequence ==
            m_snapshot.cohortCutoverPublicationSequence;
        const bool sameAcceptedBypassCohort =
            previouslyAcceptedBypass &&
            cutover.cohortSequence ==
            m_snapshot.observedCutoverCohortSequence &&
            cutover.boundarySequence ==
            m_snapshot.observedCutoverBoundarySequence;
        const bool bypassLineageMonotonic =
            !previouslyAcceptedBypass ||
            cutover.cohortSequence >
            m_snapshot.observedCutoverCohortSequence ||
            sameAcceptedBypassCohort;
        const bool captureReceiptExact = sameAcceptedBypassCohort
            ? cutover.activeEntriesAtCapture ==
            m_snapshot.observedCutoverActiveEntriesAtCapture
            : cutover.activeEntriesAtCapture == registry.activeEntries;
        const bool equalPublicationStable =
            !sameAcceptedPublication ||
            (previouslyAcceptedBypass &&
                ObservedCutoverStateStable(cutover));
        const bool sameCohortPublicationAdvanceExact =
            !sameAcceptedBypassCohort || sameAcceptedPublication ||
            cutover.registryPublicationSequence >
            m_snapshot.observedCutoverRegistryPublicationSequence;

        return cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::BYPASSED &&
            cutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::
            BYPASS_NO_EXACT_COHORT &&
            cutover.enabled && !cutover.exactCohort &&
            !cutover.bound && !cutover.waiting && !cutover.released &&
            !cutover.fallbackLegacy && !cutover.sessionLockout &&
            !cutover.runtimeInfluence && !cutover.resolverBypassed &&
            !cutover.motionWrite && cutover.accountingValid &&
            cutover.session == m_snapshot.session &&
            cutover.executionEpoch == MOTION_EXECUTION_EPOCH_INVALID &&
            cutover.owner == MotionOwner::NONE &&
            cutover.ownerGeneration == MOTION_OWNER_GENERATION_INVALID &&
            cutover.cohortSequence > m_snapshot.cohortSequence &&
            cutover.gateSequence == 0ULL &&
            cutover.publicationSequence >
            m_snapshot.generationCutoverPublicationAtRelease &&
            cutover.publicationSequence >=
            m_snapshot.cohortCutoverPublicationSequence &&
            cutover.registryPublicationSequence >=
            m_snapshot.generationRegistryPublicationAtRelease &&
            cutover.registryPublicationSequence ==
            registry.publicationSequence &&
            cutover.registryActiveEntries == registry.activeEntries &&
            cutover.activeEntriesAtCapture <=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE &&
            cutover.memberCount == 0U && cutover.terminalCount == 0U &&
            !cutover.holdAcknowledged && !cutover.resumeRequested &&
            !cutover.resumeApplied && cutover.terminalOrderValid &&
            RegistryBasic(registry) &&
            registry.currentSession == m_snapshot.session &&
            bypassLineageMonotonic && captureReceiptExact &&
            equalPublicationStable && sameCohortPublicationAdvanceExact;
    }

    bool ActiveGenerationExact(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cohortCutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) const noexcept
    {
        const auto& active = rolling.currentActive;
        const bool phaseExact =
            (rolling.phase ==
                NCOrdinaryG00FeedHoldRollingRearmPhase::
                K77_GENERATION_BOUND &&
                rolling.decision ==
                NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_GENERATION_BOUND) ||
            (rolling.phase ==
                NCOrdinaryG00FeedHoldRollingRearmPhase::K77_WAIT_RELEASE &&
                rolling.decision ==
                NCOrdinaryG00FeedHoldRollingRearmDecision::
                K77_WAIT_TERMINAL_RELEASE);
        return phaseExact && rolling.active &&
            !rolling.releasedContinuity &&
            rolling.activeGenerationOrdinal == rolling.generationCount &&
            rolling.generationCount == rolling.releaseCount + 1ULL &&
            rolling.releaseCount ==
            m_snapshot.lastReleasedGenerationOrdinal &&
            active.generationOrdinal == rolling.activeGenerationOrdinal &&
            GenerationCaptureExact(active) &&
            active.session == rolling.session &&
            active.executionEpoch == rolling.executionEpoch &&
            active.owner == rolling.owner &&
            active.ownerGeneration == rolling.ownerGeneration &&
            cohortCutover.publicationSequence ==
            active.cutoverPublicationAtCapture &&
            cohortCutover.registryPublicationSequence ==
            active.registryPublicationAtCapture &&
            registry.publicationSequence ==
            active.registryPublicationAtCapture &&
            CohortCutoverMatchesActive(active, cohortCutover, registry);
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

    static bool CohortCutoverMatchesActive(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& active,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        return ActiveCutoverPhaseDecisionExact(cutover) &&
            cutover.enabled && cutover.exactCohort &&
            cutover.bound && !cutover.released &&
            !cutover.fallbackLegacy && !cutover.sessionLockout &&
            !cutover.motionWrite && cutover.accountingValid &&
            cutover.session == active.session &&
            cutover.executionEpoch == active.executionEpoch &&
            cutover.owner == active.owner &&
            cutover.ownerGeneration == active.ownerGeneration &&
            cutover.cohortSequence == active.cohortSequence &&
            cutover.boundarySequence == active.boundarySequence &&
            cutover.gateSequence == active.gateSequence &&
            cutover.publicationSequence >=
            active.cutoverPublicationAtCapture &&
            cutover.registryPublicationSequence ==
            active.registryPublicationAtCapture &&
            registry.publicationSequence >=
            active.registryPublicationAtCapture &&
            cutover.activeEntriesAtCapture == 2U &&
            cutover.memberCount == 2U &&
            cutover.terminalCount == active.terminalCount &&
            cutover.holdAcknowledged == active.holdAcknowledged &&
            cutover.resumeRequested == active.resumeRequested &&
            cutover.resumeApplied == active.resumeApplied &&
            cutover.terminalOrderValid == active.terminalOrderValid &&
            cutover.registryActiveEntries == registry.activeEntries &&
            registry.currentSession == active.session &&
            registry.activeEntries <= 2U;
    }

    bool CohortCutoverReleasedExact(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover) const
        noexcept
    {
        return cutover.phase ==
            NCOrdinaryG00FeedHoldCohortCutoverPhase::RELEASED &&
            cutover.decision ==
            NCOrdinaryG00FeedHoldCohortCutoverDecision::ALLOW_READ_AHEAD &&
            cutover.enabled && cutover.exactCohort && cutover.bound &&
            cutover.released && !cutover.waiting &&
            !cutover.fallbackLegacy && !cutover.sessionLockout &&
            !cutover.resolverBypassed && cutover.accountingValid &&
            !cutover.motionWrite &&
            cutover.session == m_snapshot.session &&
            cutover.executionEpoch == m_snapshot.executionEpoch &&
            cutover.owner == m_snapshot.owner &&
            cutover.ownerGeneration == m_snapshot.ownerGeneration &&
            cutover.cohortSequence == m_snapshot.cohortSequence &&
            cutover.boundarySequence == m_snapshot.boundarySequence &&
            cutover.gateSequence != 0ULL &&
            cutover.registryPublicationSequence != 0ULL &&
            cutover.activeEntriesAtCapture == 2U &&
            cutover.memberCount == 2U && cutover.terminalCount == 2U &&
            cutover.registryActiveEntries == 0U &&
            cutover.holdAcknowledged && cutover.resumeRequested &&
            cutover.resumeApplied && cutover.terminalOrderValid;
    }

    bool CohortCutoverReleasedWhileRollingWaits(
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) const noexcept
    {
        return CohortCutoverReleasedExact(cutover) &&
            RegistryBasic(registry) &&
            registry.currentSession == m_snapshot.session &&
            registry.activeEntries == 0U &&
            registry.publicationSequence ==
            cutover.registryPublicationSequence;
    }

    bool ReleaseProofExact(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) const noexcept
    {
        const auto& released = rolling.lastReleased;
        const bool waitedForK77 =
            m_snapshot.decision ==
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K77_CONTINUITY_PROOF;
        const bool cutoverPublicationExact = waitedForK77
            ? cutover.publicationSequence >=
            m_snapshot.cohortCutoverPublicationSequence
            : cutover.publicationSequence >
            m_snapshot.cohortCutoverPublicationSequence;
        if (rolling.active || !rolling.releasedContinuity ||
            rolling.publicationSequence <=
            m_snapshot.rollingPublicationSequence ||
            !cutoverPublicationExact ||
            rolling.phase !=
            NCOrdinaryG00FeedHoldRollingRearmPhase::
            K77_CONTINUITY_RELEASED ||
            rolling.decision !=
            NCOrdinaryG00FeedHoldRollingRearmDecision::
            K77_GENERATION_RELEASED ||
            rolling.generationCount != m_snapshot.generationOrdinal ||
            rolling.releaseCount != m_snapshot.generationOrdinal ||
            released.generationOrdinal != m_snapshot.generationOrdinal ||
            !GenerationReleaseExact(released) ||
            released.session != m_snapshot.session ||
            released.executionEpoch != m_snapshot.executionEpoch ||
            released.owner != m_snapshot.owner ||
            released.ownerGeneration != m_snapshot.ownerGeneration ||
            released.cohortSequence != m_snapshot.cohortSequence ||
            released.boundarySequence != m_snapshot.boundarySequence ||
            released.cohortPublicationAtCapture !=
            m_snapshot.generationCohortPublicationAtCapture ||
            released.cutoverPublicationAtCapture !=
            m_snapshot.generationCutoverPublicationAtCapture ||
            released.registryPublicationAtCapture !=
            m_snapshot.generationRegistryPublicationAtCapture ||
            rolling.lineageDigest != released.lineageDigest ||
            rolling.lastCohortSequence != released.cohortSequence ||
            rolling.lastBoundarySequence != released.boundarySequence ||
            rolling.lastGateSequence != released.gateSequence)
        {
            return false;
        }
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            if (released.members[i].registrySequence !=
                m_snapshot.memberRegistrySequence[i] ||
                released.members[i].entrySequence !=
                m_snapshot.memberEntrySequence[i] ||
                released.members[i].dispatchId !=
                m_snapshot.memberDispatchId[i] ||
                released.members[i].commitSequence !=
                m_snapshot.memberCommitSequence[i] ||
                !SameIdentity(released.members[i].identity,
                    m_snapshot.memberIdentity[i]) ||
                released.members[i].ownerLease.owner !=
                m_snapshot.memberOwnerLease[i].owner ||
                released.members[i].ownerLease.generation !=
                m_snapshot.memberOwnerLease[i].generation ||
                released.members[i].lineageFingerprint !=
                m_snapshot.memberLineageFingerprint[i] ||
                released.members[i].sourcePC !=
                m_snapshot.memberSourcePC[i] ||
                released.members[i].sourceLineNumber !=
                m_snapshot.memberSourceLineNumber[i])
            {
                return false;
            }
        }
        return CohortCutoverReleasedExact(cutover) &&
            cutover.publicationSequence ==
            released.cutoverPublicationAtRelease &&
            cutover.gateSequence == released.gateSequence &&
            RegistryBasic(registry) &&
            registry.currentSession == m_snapshot.session &&
            registry.activeEntries == 0U &&
            registry.publicationSequence ==
            released.registryPublicationAtRelease &&
            cutover.registryPublicationSequence ==
            registry.publicationSequence &&
            registry.lastObservedFeedbackSequence ==
            released.members[1].terminalFeedbackSequence;
    }

    void SeedTracking(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        m_snapshot.trackingSeeded = true;
        m_snapshot.session = rolling.session;
        m_snapshot.executionEpoch = rolling.executionEpoch;
        m_snapshot.owner = rolling.owner;
        m_snapshot.ownerGeneration = rolling.ownerGeneration;
        m_snapshot.rollingPublicationSequence = rolling.publicationSequence;
        m_snapshot.registryPublicationSequence = registry.publicationSequence;
        m_snapshot.generationCount = rolling.generationCount;
        m_snapshot.releaseCount = rolling.releaseCount;
        m_snapshot.lastReleasedGenerationOrdinal = rolling.releaseCount;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_TRACKING;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_TRACKING_ARMED;
        ++m_counters.trackingSeeds;
        Publish();
    }

    void BindGeneration(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& active = rolling.currentActive;
        m_snapshot.rollingPublicationSequence = rolling.publicationSequence;
        m_snapshot.cohortCutoverPublicationSequence =
            cutover.publicationSequence;
        m_snapshot.registryPublicationSequence = registry.publicationSequence;
        m_snapshot.generationOrdinal = active.generationOrdinal;
        m_snapshot.generationCount = rolling.generationCount;
        m_snapshot.releaseCount = rolling.releaseCount;
        m_snapshot.cohortSequence = active.cohortSequence;
        m_snapshot.boundarySequence = active.boundarySequence;
        m_snapshot.gateSequence = active.gateSequence;
        m_snapshot.lineageDigest = active.lineageDigest;
        m_snapshot.generationCohortPublicationAtCapture =
            active.cohortPublicationAtCapture;
        m_snapshot.generationCutoverPublicationAtCapture =
            active.cutoverPublicationAtCapture;
        m_snapshot.generationRegistryPublicationAtCapture =
            active.registryPublicationAtCapture;
        m_snapshot.generationCohortPublicationAtRelease = 0ULL;
        m_snapshot.generationCutoverPublicationAtRelease = 0ULL;
        m_snapshot.generationRegistryPublicationAtRelease = 0ULL;
        CaptureObservedCutoverState(cutover);
        CaptureObservedRollingState(rolling);
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            m_snapshot.memberRegistrySequence[i] =
                active.members[i].registrySequence;
            m_snapshot.memberEntrySequence[i] =
                active.members[i].entrySequence;
            m_snapshot.memberDispatchId[i] = active.members[i].dispatchId;
            m_snapshot.memberCommitSequence[i] =
                active.members[i].commitSequence;
            m_snapshot.memberIdentity[i] = active.members[i].identity;
            m_snapshot.memberOwnerLease[i] = active.members[i].ownerLease;
            m_snapshot.memberLineageFingerprint[i] =
                active.members[i].lineageFingerprint;
            m_snapshot.memberSourcePC[i] = active.members[i].sourcePC;
            m_snapshot.memberSourceLineNumber[i] =
                active.members[i].sourceLineNumber;
            m_snapshot.memberTerminalFeedbackSequence[i] =
                MOTION_FEEDBACK_SEQUENCE_INVALID;
            m_snapshot.memberTerminalFeedbackType[i] =
                MotionFeedbackType::NONE;
            m_snapshot.memberTerminalOrdinal[i] = 0U;
        }
        m_snapshot.exactGeneration = true;
        m_snapshot.bound = true;
        m_snapshot.waiting = true;
        m_snapshot.released = false;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_GENERATION_BOUND;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_GENERATION_BOUND;
        ++m_counters.generationsBound;
        Publish();
    }

    void ObserveBound(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (cutover.fallbackLegacy || cutover.sessionLockout)
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K74,
                cutover.accountingValid);
            return;
        }
        if (rolling.active)
        {
            if (rolling.publicationSequence ==
                m_snapshot.rollingPublicationSequence &&
                !ObservedRollingActiveStateStable(rolling))
            {
                ++m_counters.sequenceMismatches;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_SEQUENCE,
                    rolling.accountingValid);
                return;
            }
            if (!BoundGenerationStable(rolling.currentActive))
            {
                ClassifyBoundGenerationFailure(rolling.currentActive);
                return;
            }
            if (cutover.released)
            {
                const bool alreadyWaitingForK77 =
                    m_snapshot.decision ==
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_WAIT_K77_CONTINUITY_PROOF;
                if (cutover.publicationSequence == 0ULL ||
                    (!alreadyWaitingForK77 &&
                        cutover.publicationSequence <=
                        m_snapshot.
                        cohortCutoverPublicationSequence) ||
                    (alreadyWaitingForK77 &&
                        cutover.publicationSequence <
                        m_snapshot.
                        cohortCutoverPublicationSequence))
                {
                    ++m_counters.sequenceMismatches;
                    EnterFallback(
                        NCOrdinaryG00FeedHoldRollingCutoverDecision::
                        K78_FALLBACK_SEQUENCE,
                        true);
                    return;
                }
                if (alreadyWaitingForK77 &&
                    (cutover.gateSequence !=
                        m_snapshot.observedCutoverGateSequence ||
                        cutover.registryPublicationSequence !=
                        m_snapshot.
                        observedCutoverRegistryPublicationSequence))
                {
                    ++m_counters.k74Failures;
                    EnterFallback(
                        NCOrdinaryG00FeedHoldRollingCutoverDecision::
                        K78_FALLBACK_K74,
                        cutover.accountingValid);
                    return;
                }
                if (!CohortCutoverReleasedWhileRollingWaits(
                    cutover, registry))
                {
                    ++m_counters.k74Failures;
                    EnterFallback(
                        NCOrdinaryG00FeedHoldRollingCutoverDecision::
                        K78_FALLBACK_K74,
                        cutover.accountingValid);
                    return;
                }
                m_snapshot.gateSequence = cutover.gateSequence;
                m_snapshot.rollingPublicationSequence =
                    rolling.publicationSequence;
                m_snapshot.cohortCutoverPublicationSequence =
                    cutover.publicationSequence;
                m_snapshot.registryPublicationSequence =
                    registry.publicationSequence;
                m_snapshot.generationCount = rolling.generationCount;
                m_snapshot.releaseCount = rolling.releaseCount;
                CaptureObservedCutoverState(cutover);
                CaptureObservedRollingState(rolling);
                if (m_snapshot.phase !=
                    NCOrdinaryG00FeedHoldRollingCutoverPhase::
                    K78_WAIT_RELEASE ||
                    m_snapshot.decision !=
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_WAIT_K77_CONTINUITY_PROOF)
                {
                    m_snapshot.phase =
                        NCOrdinaryG00FeedHoldRollingCutoverPhase::
                        K78_WAIT_RELEASE;
                    m_snapshot.decision =
                        NCOrdinaryG00FeedHoldRollingCutoverDecision::
                        K78_WAIT_K77_CONTINUITY_PROOF;
                    Publish();
                }
                return;
            }
            if (!CohortCutoverMatchesActive(
                rolling.currentActive, cutover, registry))
            {
                ++m_counters.k74Failures;
                EnterFallback(
                    NCOrdinaryG00FeedHoldRollingCutoverDecision::
                    K78_FALLBACK_K74,
                    cutover.accountingValid);
                return;
            }
            m_snapshot.gateSequence = rolling.currentActive.gateSequence;
            m_snapshot.rollingPublicationSequence =
                rolling.publicationSequence;
            m_snapshot.cohortCutoverPublicationSequence =
                cutover.publicationSequence;
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
            m_snapshot.generationCount = rolling.generationCount;
            m_snapshot.releaseCount = rolling.releaseCount;
            CaptureObservedCutoverState(cutover);
            CaptureObservedRollingState(rolling);
            const auto nextDecision =
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_WAIT_K74_RELEASE;
            if (m_snapshot.phase !=
                NCOrdinaryG00FeedHoldRollingCutoverPhase::
                K78_WAIT_RELEASE ||
                m_snapshot.decision != nextDecision)
            {
                m_snapshot.phase =
                    NCOrdinaryG00FeedHoldRollingCutoverPhase::
                    K78_WAIT_RELEASE;
                m_snapshot.decision = nextDecision;
                Publish();
            }
            return;
        }

        if (!ReleaseProofExact(rolling, cutover, registry))
        {
            ClassifyReleaseFailure(rolling, cutover, registry);
            return;
        }

        m_snapshot.rollingPublicationSequence = rolling.publicationSequence;
        m_snapshot.cohortCutoverPublicationSequence = cutover.publicationSequence;
        m_snapshot.registryPublicationSequence = registry.publicationSequence;
        m_snapshot.generationCount = rolling.generationCount;
        m_snapshot.releaseCount = rolling.releaseCount;
        m_snapshot.gateSequence = rolling.lastReleased.gateSequence;
        m_snapshot.lineageDigest = rolling.lastReleased.lineageDigest;
        m_snapshot.generationCohortPublicationAtRelease =
            rolling.lastReleased.cohortPublicationAtRelease;
        m_snapshot.generationCutoverPublicationAtRelease =
            rolling.lastReleased.cutoverPublicationAtRelease;
        m_snapshot.generationRegistryPublicationAtRelease =
            rolling.lastReleased.registryPublicationAtRelease;
        CaptureObservedCutoverState(cutover);
        for (std::size_t i = 0U; i < 2U; ++i)
        {
            m_snapshot.memberTerminalFeedbackSequence[i] =
                rolling.lastReleased.members[i].terminalFeedbackSequence;
            m_snapshot.memberTerminalFeedbackType[i] =
                rolling.lastReleased.members[i].terminalFeedbackType;
            m_snapshot.memberTerminalOrdinal[i] =
                rolling.lastReleased.members[i].terminalOrdinal;
        }
        m_snapshot.lastReleasedGenerationOrdinal =
            m_snapshot.generationOrdinal;
        m_snapshot.bound = false;
        m_snapshot.waiting = false;
        m_snapshot.released = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::
            K78_CONTINUITY_RELEASED;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_ALLOW_READ_AHEAD;
        ++m_counters.releases;
        Publish();
    }

    void ClassifyRollingFailure(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        AdoptFallbackSession(rolling.session, registry);
        if (!rolling.activeInvariantValid ||
            rolling.generationCount < rolling.releaseCount)
        {
            ++m_counters.activeInvariantMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACTIVE_INVARIANT,
                rolling.accountingValid);
        }
        else if (!rolling.sameSession)
        {
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SESSION,
                rolling.accountingValid);
        }
        else if (!rolling.sameExecutionLease)
        {
            ++m_counters.executionLeaseMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_EXECUTION_LEASE,
                rolling.accountingValid);
        }
        else if (!rolling.sequenceMonotonic)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                rolling.accountingValid);
        }
        else
        {
            ++m_counters.lineageMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_LINEAGE,
                rolling.accountingValid);
        }
    }

    void ClassifyTrackingFailure(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling) noexcept
    {
        if (rolling.session != m_snapshot.session)
        {
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SESSION,
                true);
        }
        else if (!SameLease(rolling, m_snapshot))
        {
            ++m_counters.executionLeaseMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_EXECUTION_LEASE,
                true);
        }
        else
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                true);
        }
    }

    void ClassifyActiveFailure(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (rolling.session != registry.currentSession)
        {
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SESSION,
                true);
        }
        else if (cutover.fallbackLegacy || cutover.sessionLockout ||
            !cutover.enabled || !cutover.exactCohort || !cutover.bound)
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K74,
                cutover.accountingValid);
        }
        else if (!RegistryBasic(registry))
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_REGISTRY,
                registry.accountingValid);
        }
        else if (!CohortCutoverMatchesActive(
            rolling.currentActive, cutover, registry))
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K74,
                cutover.accountingValid);
        }
        else if (!rolling.activeInvariantValid ||
            rolling.generationCount != rolling.releaseCount + 1ULL)
        {
            ++m_counters.activeInvariantMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACTIVE_INVARIANT,
                true);
        }
        else
        {
            ++m_counters.lineageMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_LINEAGE,
                true);
        }
    }

    void ClassifyBoundGenerationFailure(
        const NCOrdinaryG00FeedHoldRollingGenerationSnapshot& value) noexcept
    {
        if (value.session != m_snapshot.session)
        {
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SESSION,
                true);
            return;
        }
        const bool memberLeaseMismatch =
            value.members[0].ownerLease.owner !=
            m_snapshot.memberOwnerLease[0].owner ||
            value.members[0].ownerLease.generation !=
            m_snapshot.memberOwnerLease[0].generation ||
            value.members[1].ownerLease.owner !=
            m_snapshot.memberOwnerLease[1].owner ||
            value.members[1].ownerLease.generation !=
            m_snapshot.memberOwnerLease[1].generation;
        if (value.executionEpoch != m_snapshot.executionEpoch ||
            value.owner != m_snapshot.owner ||
            value.ownerGeneration != m_snapshot.ownerGeneration ||
            memberLeaseMismatch)
        {
            ++m_counters.executionLeaseMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_EXECUTION_LEASE,
                true);
            return;
        }
        if (value.generationOrdinal != m_snapshot.generationOrdinal ||
            value.cohortSequence != m_snapshot.cohortSequence ||
            value.boundarySequence != m_snapshot.boundarySequence)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                true);
            return;
        }
        if (value.lineageDigest != m_snapshot.lineageDigest ||
            value.members[0].lineageFingerprint !=
            m_snapshot.memberLineageFingerprint[0] ||
            value.members[1].lineageFingerprint !=
            m_snapshot.memberLineageFingerprint[1] ||
            value.members[0].sourcePC != m_snapshot.memberSourcePC[0] ||
            value.members[1].sourcePC != m_snapshot.memberSourcePC[1] ||
            value.members[0].sourceLineNumber !=
            m_snapshot.memberSourceLineNumber[0] ||
            value.members[1].sourceLineNumber !=
            m_snapshot.memberSourceLineNumber[1])
        {
            ++m_counters.lineageMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_LINEAGE,
                true);
            return;
        }
        ++m_counters.identityMismatches;
        EnterFallback(
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_FALLBACK_IDENTITY,
            true);
    }

    void ClassifyReleaseFailure(
        const NCOrdinaryG00FeedHoldRollingRearmSnapshot& rolling,
        const NCOrdinaryG00FeedHoldCohortCutoverSnapshot& cutover,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        const auto& released = rolling.lastReleased;
        const bool waitedForK77 =
            m_snapshot.decision ==
            NCOrdinaryG00FeedHoldRollingCutoverDecision::
            K78_WAIT_K77_CONTINUITY_PROOF;
        const bool cutoverPublicationExact = waitedForK77
            ? cutover.publicationSequence >=
            m_snapshot.cohortCutoverPublicationSequence
            : cutover.publicationSequence >
            m_snapshot.cohortCutoverPublicationSequence;
        if (!RegistryBasic(registry) || registry.activeEntries != 0U ||
            registry.currentSession != m_snapshot.session ||
            registry.publicationSequence !=
            released.registryPublicationAtRelease ||
            cutover.registryPublicationSequence !=
            registry.publicationSequence ||
            registry.lastObservedFeedbackSequence !=
            released.members[1].terminalFeedbackSequence)
        {
            ++m_counters.registryMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_REGISTRY,
                registry.accountingValid);
        }
        else if (!CohortCutoverReleasedExact(cutover) ||
            cutover.publicationSequence !=
            released.cutoverPublicationAtRelease ||
            cutover.gateSequence != released.gateSequence ||
            cutover.registryPublicationSequence !=
            released.registryPublicationAtRelease)
        {
            ++m_counters.k74Failures;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_K74,
                cutover.accountingValid);
        }
        else if (!rolling.activeInvariantValid || rolling.active ||
            rolling.generationCount != rolling.releaseCount ||
            rolling.releaseCount != m_snapshot.generationOrdinal)
        {
            ++m_counters.activeInvariantMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_ACTIVE_INVARIANT,
                rolling.accountingValid);
        }
        else if (!rolling.sequenceMonotonic ||
            rolling.publicationSequence <=
            m_snapshot.rollingPublicationSequence ||
            !cutoverPublicationExact ||
            released.cohortPublicationAtRelease <=
            released.cohortPublicationAtCapture ||
            released.cutoverPublicationAtRelease <=
            released.cutoverPublicationAtCapture ||
            released.registryPublicationAtRelease <=
            released.registryPublicationAtCapture)
        {
            ++m_counters.sequenceMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SEQUENCE,
                rolling.accountingValid);
        }
        else
        {
            ++m_counters.lineageMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_LINEAGE,
                rolling.accountingValid && cutover.accountingValid &&
                registry.accountingValid);
        }
    }

    void SetBypass(
        NCOrdinaryG00FeedHoldRollingCutoverDecision decision,
        std::uint64_t& counter) noexcept
    {
        if (m_snapshot.phase ==
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_BYPASSED &&
            m_snapshot.decision == decision)
        {
            return;
        }
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_BYPASSED;
        m_snapshot.decision = decision;
        ++counter;
        Publish();
    }

    void EnterFallback(
        NCOrdinaryG00FeedHoldRollingCutoverDecision decision,
        bool accountingValid) noexcept
    {
        if (!m_snapshot.sessionLockout)
        {
            ++m_counters.failures;
        }
        m_snapshot.bound = false;
        m_snapshot.waiting = false;
        m_snapshot.released = false;
        m_snapshot.fallbackLegacy = true;
        m_snapshot.sessionLockout = true;
        m_snapshot.accountingValid = accountingValid;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_LEGACY_FALLBACK;
        m_snapshot.decision = decision;
        Publish();
    }

    void AdoptFallbackSession(
        NCPreparedQueueSession rollingSession,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            m_snapshot.session =
                registry.currentSession !=
                NC_PREPARED_QUEUE_SESSION_INVALID
                ? registry.currentSession
                : rollingSession;
        }
        if (registry.currentSession !=
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.publicationSequence >
            m_snapshot.registryPublicationSequence)
        {
            m_snapshot.registryPublicationSequence =
                registry.publicationSequence;
        }
    }

    bool ResetForChangedSession(
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!(m_snapshot.trackingSeeded || m_snapshot.bound ||
            m_snapshot.sessionLockout || m_snapshot.released) ||
            m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == NC_PREPARED_QUEUE_SESSION_INVALID ||
            registry.currentSession == m_snapshot.session)
        {
            return false;
        }

        if (!RegistryBasic(registry) ||
            registry.revokedPendingEntries != 0U ||
            registry.publicationSequence <=
            m_snapshot.registryPublicationSequence)
        {
            // The Registry has already named the new live Queue Session, but
            // its reset receipt is not trustworthy.  Lock that new Session;
            // retaining the old Session here would let the next healthy
            // receipt for the same new Session clear this failure.
            m_snapshot.session = registry.currentSession;
            if (registry.publicationSequence >
                m_snapshot.registryPublicationSequence)
            {
                m_snapshot.registryPublicationSequence =
                    registry.publicationSequence;
            }
            ++m_counters.sessionMismatches;
            EnterFallback(
                NCOrdinaryG00FeedHoldRollingCutoverDecision::
                K78_FALLBACK_SESSION,
                registry.accountingValid);
            return true;
        }

        const std::uint64_t publication = m_snapshot.publicationSequence;
        NCOrdinaryG00FeedHoldRollingCutoverSnapshot next{};
        next.publicationSequence = publication;
        next.enabled = m_enabled;
        next.phase =
            NCOrdinaryG00FeedHoldRollingCutoverPhase::K78_IDLE;
        next.decision =
            NCOrdinaryG00FeedHoldRollingCutoverDecision::K78_SESSION_RESET;
        m_snapshot = next;
        m_lastObservedRollingPublicationSequence = 0ULL;
        ++m_counters.sessionResets;
        Publish();
        return true;
    }

    void Clear(bool preservePublication) noexcept
    {
        const std::uint64_t publication = preservePublication
            ? m_snapshot.publicationSequence : 0ULL;
        NCOrdinaryG00FeedHoldRollingCutoverSnapshot next{};
        next.publicationSequence = publication;
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
    std::uint64_t m_lastObservedRollingPublicationSequence = 0ULL;
    NCOrdinaryG00FeedHoldRollingCutoverSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldRollingCutoverCounters m_counters{};
};

static_assert(sizeof(NCOrdinaryG00FeedHoldRollingRearmCutoverGate) <= 768U,
    "K.7.8 rolling cutover gate exceeded its fixed storage budget.");
