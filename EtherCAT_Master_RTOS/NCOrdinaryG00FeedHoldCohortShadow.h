#pragma once

#include "NCFeedHoldResumeGate.h"
#include "NCOrdinaryG00InflightTerminalRegistryShadow.h"

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.3 - Two-Entry Feed-Hold / Resume Terminal Cohort Shadow
//
// When PROGRAM Feed Hold is pressed while the K.7.1 registry contains exactly
// two active read-ahead G00 entries, this observer captures both immutable
// identities as one cohort.  It then correlates the existing Feed Hold ACK,
// ACK-gated Resume and exactly one Ledger-accepted terminal for each member.
//
// This class is deliberately observation-only.  It does not submit or cancel
// Motion work, change NC state/PC/callback/Epoch/owner, write PDO data, allocate
// memory or wait.  Starting at K.7.4, its already-published immutable proof may
// be read by the separate cohort cutover gate; this observer still performs no
// Runtime action and keeps runtimeInfluence/motionWrite false.
// =============================================================================

constexpr std::size_t NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE = 2U;

enum class NCOrdinaryG00FeedHoldCohortPhase : std::uint8_t
{
    K73_IDLE = 0,
    K73_BYPASSED,
    K73_CAPTURED,
    K73_HOLD_ACKNOWLEDGED,
    K73_RESUME_REQUESTED,
    K73_RESUME_APPLIED,
    K73_TERMINAL_PENDING,
    K73_TERMINAL_COMPLETE,
    K73_CANCELLED,
    K73_FAILED
};

enum class NCOrdinaryG00FeedHoldCohortDecision : std::uint8_t
{
    K73_NONE = 0,
    K73_BYPASS_NOT_PROGRAM,
    K73_BYPASS_NOT_TWO_ACTIVE,
    K73_CAPTURED_EXACT,
    K73_HOLD_ACKNOWLEDGED,
    K73_RESUME_BEFORE_ACK,
    K73_RESUME_AFTER_ACK,
    K73_RESUME_APPLIED,
    K73_WAIT_FIRST_TERMINAL,
    K73_WAIT_SECOND_TERMINAL,
    K73_ALL_COMPLETED_EXACT,
    K73_INTERRUPTED_TERMINAL_EXACT,
    K73_INVALID_BOUNDARY,
    K73_INVALID_REGISTRY,
    K73_INVALID_MEMBER,
    K73_SESSION_MISMATCH,
    K73_IDENTITY_CONFLICT,
    K73_OWNER_MISMATCH,
    K73_LEDGER_REJECTED,
    K73_TERMINAL_OUT_OF_ORDER,
    K73_DUPLICATE_TERMINAL,
    K73_BOUNDARY_MISMATCH,
    K73_GATE_MISMATCH,
    K73_CANCELLED,
    K73_SUPERSEDED
};

struct NCOrdinaryG00FeedHoldCohortMemberSnapshot
{
    std::uint64_t registrySequence = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
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
    NCOrdinaryG00InflightState currentState =
        NCOrdinaryG00InflightState::K63_EMPTY;
    MotionFeedbackSequence lastFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackType lastFeedbackType = MotionFeedbackType::NONE;

    std::uint8_t terminalOrdinal = 0U;
    bool exact = false;
    bool heldObserved = false;
    bool resumedObserved = false;
    bool terminalObserved = false;
    bool terminalBeforeAcknowledge = false;
    bool terminalBeforeResume = false;
    bool terminalCompleted = false;
};

struct NCOrdinaryG00FeedHoldCohortSnapshot
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

    NCOrdinaryG00FeedHoldCohortPhase phase =
        NCOrdinaryG00FeedHoldCohortPhase::K73_IDLE;
    NCOrdinaryG00FeedHoldCohortDecision decision =
        NCOrdinaryG00FeedHoldCohortDecision::K73_NONE;

    std::array<NCOrdinaryG00FeedHoldCohortMemberSnapshot,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE> members{};

    std::uint32_t activeEntriesAtCapture = 0U;
    std::uint8_t memberCount = 0U;
    std::uint8_t terminalCount = 0U;

    bool candidate = false;
    bool active = false;
    bool exactTwoEntryCohort = false;
    bool boundaryCorrelated = false;
    bool gateCorrelated = false;
    bool holdAcknowledged = false;
    bool resumeRequested = false;
    bool resumeBeforeAcknowledge = false;
    bool resumeAfterAcknowledge = false;
    bool resumeApplied = false;
    bool terminalOrderValid = true;
    bool terminalCohortComplete = false;
    bool allMembersCompleted = false;
    bool cancelled = false;
    bool failed = false;

    bool bounded = true;
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00FeedHoldCohortCounters
{
    std::uint64_t captureAttempts = 0ULL;
    std::uint64_t programRequests = 0ULL;
    std::uint64_t bypassNotProgram = 0ULL;
    std::uint64_t bypassNotTwoActive = 0ULL;
    std::uint64_t cohortsCaptured = 0ULL;

    std::uint64_t boundaryTransitions = 0ULL;
    std::uint64_t gateCorrelations = 0ULL;
    std::uint64_t holdAcknowledged = 0ULL;
    std::uint64_t resumeBeforeAcknowledge = 0ULL;
    std::uint64_t resumeAfterAcknowledge = 0ULL;
    std::uint64_t resumeApplied = 0ULL;

    std::uint64_t feedbackObserved = 0ULL;
    std::uint64_t feedbackMatched = 0ULL;
    std::uint64_t feedbackIgnored = 0ULL;
    std::uint64_t held = 0ULL;
    std::uint64_t resumed = 0ULL;
    std::uint64_t terminals = 0ULL;
    std::uint64_t completedTerminals = 0ULL;
    std::uint64_t interruptedTerminals = 0ULL;
    std::uint64_t terminalBeforeAcknowledge = 0ULL;
    std::uint64_t terminalBeforeResume = 0ULL;
    std::uint64_t terminalCohorts = 0ULL;
    std::uint64_t allCompletedCohorts = 0ULL;
    std::uint64_t interruptedCohorts = 0ULL;

    std::uint64_t invalidBoundary = 0ULL;
    std::uint64_t invalidRegistry = 0ULL;
    std::uint64_t invalidMember = 0ULL;
    std::uint64_t sessionMismatch = 0ULL;
    std::uint64_t identityConflict = 0ULL;
    std::uint64_t ownerMismatch = 0ULL;
    std::uint64_t ledgerRejected = 0ULL;
    std::uint64_t terminalOutOfOrder = 0ULL;
    std::uint64_t duplicateTerminal = 0ULL;
    std::uint64_t boundaryMismatch = 0ULL;
    std::uint64_t gateMismatch = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t superseded = 0ULL;
    std::uint64_t failures = 0ULL;

    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortMemberSnapshot>::value,
    "K.7.3 cohort member snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortSnapshot>::value,
    "K.7.3 cohort snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    NCOrdinaryG00FeedHoldCohortCounters>::value,
    "K.7.3 cohort counters must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortSnapshot) <= 384U,
    "K.7.3 cohort snapshot exceeded its fixed diagnostic budget.");
static_assert(
    sizeof(NCOrdinaryG00FeedHoldCohortCounters) <= 384U,
    "K.7.3 cohort counters exceeded their fixed diagnostic budget.");

class NCOrdinaryG00FeedHoldCohortShadow
{
public:
    using CandidateArray =
        std::array<NCOrdinaryG00InflightEntrySnapshot,
        NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE>;

    NCOrdinaryG00FeedHoldCohortShadow() noexcept = default;

    void Begin(
        const NCFeedHoldBoundarySnapshot& boundary,
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        CandidateArray candidates) noexcept
    {
        if (m_snapshot.active)
        {
            Cancel(true);
        }

        ++m_counters.captureAttempts;
        NCOrdinaryG00FeedHoldCohortSnapshot next{};
        next.publicationSequence = m_snapshot.publicationSequence;
        next.cohortSequence = AllocateCohortSequence();
        next.boundarySequence = boundary.sequence;
        next.registryPublicationSequence = registry.publicationSequence;
        next.executionEpoch = boundary.requestExecutionEpoch;
        next.owner = boundary.requestOwner;
        next.ownerGeneration = boundary.requestOwnerGeneration;
        next.activeEntriesAtCapture = registry.activeEntries;
        m_snapshot = next;

        if (boundary.source != NCFeedHoldSource::PROGRAM)
        {
            ++m_counters.bypassNotProgram;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortPhase::K73_BYPASSED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_BYPASS_NOT_PROGRAM;
            Publish();
            return;
        }

        ++m_counters.programRequests;
        m_snapshot.candidate = true;
        if (registry.activeEntries !=
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE)
        {
            ++m_counters.bypassNotTwoActive;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortPhase::K73_BYPASSED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_BYPASS_NOT_TWO_ACTIVE;
            Publish();
            return;
        }

        if (!BoundaryValid(boundary))
        {
            ++m_counters.invalidBoundary;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_BOUNDARY);
            return;
        }
        if (!registry.ready || registry.permanentLockout ||
            !registry.accountingValid)
        {
            ++m_counters.invalidRegistry;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_REGISTRY);
            return;
        }

        if (candidates[1].registrySequence <
            candidates[0].registrySequence)
        {
            const NCOrdinaryG00InflightEntrySnapshot swap = candidates[0];
            candidates[0] = candidates[1];
            candidates[1] = swap;
        }

        if (!MemberValid(candidates[0]) || !MemberValid(candidates[1]))
        {
            ++m_counters.invalidMember;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_MEMBER);
            return;
        }
        if (candidates[0].session != candidates[1].session ||
            candidates[0].session != registry.currentSession)
        {
            ++m_counters.sessionMismatch;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_SESSION_MISMATCH);
            return;
        }
        if (!MembersDistinctAndOrdered(candidates[0], candidates[1]) ||
            !MemberMatchesBoundary(candidates[0], boundary) ||
            !MemberMatchesBoundary(candidates[1], boundary))
        {
            ++m_counters.identityConflict;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_IDENTITY_CONFLICT);
            return;
        }

        m_snapshot.session = candidates[0].session;
        FillMember(candidates[0], m_snapshot.members[0]);
        FillMember(candidates[1], m_snapshot.members[1]);
        m_snapshot.memberCount =
            static_cast<std::uint8_t>(
                NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE);
        m_snapshot.exactTwoEntryCohort = true;
        m_snapshot.boundaryCorrelated = true;
        m_snapshot.active = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortPhase::K73_CAPTURED;
        m_snapshot.decision =
            NCOrdinaryG00FeedHoldCohortDecision::K73_CAPTURED_EXACT;
        ++m_counters.cohortsCaptured;
        Publish();
    }

    void ObserveBoundary(
        const NCFeedHoldBoundarySnapshot& boundary,
        const NCFeedHoldResumeGateSnapshot& gate,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!m_snapshot.active)
        {
            return;
        }
        if (!registry.ready || registry.permanentLockout ||
            !registry.accountingValid)
        {
            ++m_counters.invalidRegistry;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_REGISTRY);
            return;
        }
        if (registry.currentSession != m_snapshot.session)
        {
            ++m_counters.sessionMismatch;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_SESSION_MISMATCH);
            return;
        }
        if (!BoundaryMatchesCapture(boundary))
        {
            ++m_counters.boundaryMismatch;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_BOUNDARY_MISMATCH);
            return;
        }
        if (boundary.failed)
        {
            ++m_counters.invalidBoundary;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_BOUNDARY);
            return;
        }
        const bool gateBelongsToCapture =
            gate.sequence != 0ULL &&
            gate.boundarySequence == m_snapshot.boundarySequence;
        if ((gate.active && !gateBelongsToCapture) ||
            (gateBelongsToCapture && !GateMatchesCapture(gate)))
        {
            ++m_counters.gateMismatch;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_GATE_MISMATCH);
            return;
        }

        bool changed = false;
        if (gateBelongsToCapture && !m_snapshot.gateCorrelated)
        {
            m_snapshot.gateSequence = gate.sequence;
            m_snapshot.gateCorrelated = true;
            ++m_counters.gateCorrelations;
            changed = true;
        }

        const bool acknowledged =
            boundary.acknowledged ||
            (gateBelongsToCapture && gate.acknowledgeObserved);
        if (acknowledged && !m_snapshot.holdAcknowledged)
        {
            m_snapshot.holdAcknowledged = true;
            ++m_counters.holdAcknowledged;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortPhase::K73_HOLD_ACKNOWLEDGED;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_HOLD_ACKNOWLEDGED;
            changed = true;
        }

        const bool resumeRequested =
            boundary.resumeRequested ||
            (gateBelongsToCapture && gate.resumeRequestLatched);
        if (resumeRequested && !m_snapshot.resumeRequested)
        {
            m_snapshot.resumeRequested = true;
            m_snapshot.resumeBeforeAcknowledge =
                boundary.resumeBeforeAcknowledge ||
                (!m_snapshot.holdAcknowledged &&
                    gateBelongsToCapture &&
                    gate.deferredUntilAcknowledge);
            m_snapshot.resumeAfterAcknowledge =
                !m_snapshot.resumeBeforeAcknowledge;
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_REQUESTED;
            if (m_snapshot.resumeBeforeAcknowledge)
            {
                ++m_counters.resumeBeforeAcknowledge;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldCohortDecision::
                    K73_RESUME_BEFORE_ACK;
            }
            else
            {
                ++m_counters.resumeAfterAcknowledge;
                m_snapshot.decision =
                    NCOrdinaryG00FeedHoldCohortDecision::
                    K73_RESUME_AFTER_ACK;
            }
            changed = true;
        }

        const bool resumeApplied =
            boundary.resumeApplied ||
            (gateBelongsToCapture && gate.resumeApplied);
        if (resumeApplied && !m_snapshot.resumeApplied)
        {
            m_snapshot.resumeApplied = true;
            ++m_counters.resumeApplied;
            m_snapshot.phase = m_snapshot.terminalCount == 0U
                ? NCOrdinaryG00FeedHoldCohortPhase::K73_RESUME_APPLIED
                : NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_PENDING;
            m_snapshot.decision = m_snapshot.terminalCount == 0U
                ? NCOrdinaryG00FeedHoldCohortDecision::K73_RESUME_APPLIED
                : NCOrdinaryG00FeedHoldCohortDecision::
                K73_WAIT_SECOND_TERMINAL;
            changed = true;
        }

        if (changed)
        {
            ++m_counters.boundaryTransitions;
            Publish();
        }
    }

    void ObserveMotionFeedback(
        const MotionFeedbackEvent& event,
        bool ledgerAccepted,
        const NCOrdinaryG00InflightRegistrySnapshot& registry) noexcept
    {
        if (!m_snapshot.active)
        {
            return;
        }

        ++m_counters.feedbackObserved;
        if (!registry.ready || registry.permanentLockout ||
            !registry.accountingValid)
        {
            ++m_counters.invalidRegistry;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_INVALID_REGISTRY);
            return;
        }
        std::size_t memberIndex =
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
        bool pairCollision = false;
        for (std::size_t index = 0U;
            index < NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE;
            ++index)
        {
            const MotionExecutionIdentity& identity =
                m_snapshot.members[index].identity;
            if (!SameEpochSegment(identity, event.identity))
            {
                continue;
            }
            if (IdentityExactlyMatches(identity, event.identity))
            {
                memberIndex = index;
                break;
            }
            pairCollision = true;
        }

        if (memberIndex >= NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE)
        {
            if (pairCollision)
            {
                ++m_counters.identityConflict;
                Fail(
                    NCOrdinaryG00FeedHoldCohortDecision::
                    K73_IDENTITY_CONFLICT);
            }
            else
            {
                ++m_counters.feedbackIgnored;
            }
            return;
        }

        ++m_counters.feedbackMatched;
        NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
            m_snapshot.members[memberIndex];
        if (!ledgerAccepted)
        {
            ++m_counters.ledgerRejected;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_LEDGER_REJECTED);
            return;
        }
        if (event.owner != member.ownerLease.owner ||
            event.ownerGeneration != member.ownerLease.generation)
        {
            ++m_counters.ownerMismatch;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_OWNER_MISMATCH);
            return;
        }
        if (member.terminalObserved)
        {
            ++m_counters.duplicateTerminal;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_DUPLICATE_TERMINAL);
            return;
        }

        member.lastFeedbackSequence = event.sequence;
        member.lastFeedbackType = event.type;
        switch (event.type)
        {
        case MotionFeedbackType::ACCEPTED:
            member.currentState =
                NCOrdinaryG00InflightState::K63_ACCEPTED;
            Publish();
            return;
        case MotionFeedbackType::STARTED:
            member.currentState =
                NCOrdinaryG00InflightState::K63_STARTED;
            Publish();
            return;
        case MotionFeedbackType::PROGRESS:
            member.currentState =
                NCOrdinaryG00InflightState::K63_ACTIVE;
            Publish();
            return;
        case MotionFeedbackType::HELD:
            member.currentState =
                NCOrdinaryG00InflightState::K63_HELD;
            member.heldObserved = true;
            ++m_counters.held;
            Publish();
            return;
        case MotionFeedbackType::RESUMED:
            member.currentState =
                NCOrdinaryG00InflightState::K63_ACTIVE;
            member.resumedObserved = true;
            ++m_counters.resumed;
            Publish();
            return;
        case MotionFeedbackType::COMPLETED:
        case MotionFeedbackType::REJECTED:
        case MotionFeedbackType::CANCELLED:
        case MotionFeedbackType::ABORTED:
        case MotionFeedbackType::FAULTED:
            ObserveTerminal(memberIndex, event.type);
            return;
        case MotionFeedbackType::NONE:
        default:
            ++m_counters.identityConflict;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_IDENTITY_CONFLICT);
            return;
        }
    }

    void Cancel(bool superseded) noexcept
    {
        if (!m_snapshot.active)
        {
            return;
        }
        m_snapshot.active = false;
        m_snapshot.cancelled = true;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortPhase::K73_CANCELLED;
        if (superseded)
        {
            ++m_counters.superseded;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::K73_SUPERSEDED;
        }
        else
        {
            ++m_counters.cancelled;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::K73_CANCELLED;
        }
        Publish();
    }

    NCOrdinaryG00FeedHoldCohortSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00FeedHoldCohortCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static bool IdentityExactlyMatches(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    static bool SameEpochSegment(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId;
    }

    static bool IsTerminal(MotionFeedbackType type) noexcept
    {
        return
            type == MotionFeedbackType::COMPLETED ||
            type == MotionFeedbackType::REJECTED ||
            type == MotionFeedbackType::CANCELLED ||
            type == MotionFeedbackType::ABORTED ||
            type == MotionFeedbackType::FAULTED;
    }

    static NCOrdinaryG00InflightState TerminalState(
        MotionFeedbackType type) noexcept
    {
        switch (type)
        {
        case MotionFeedbackType::COMPLETED:
            return NCOrdinaryG00InflightState::K63_COMPLETED;
        case MotionFeedbackType::REJECTED:
            return NCOrdinaryG00InflightState::K63_REJECTED;
        case MotionFeedbackType::CANCELLED:
            return NCOrdinaryG00InflightState::K63_CANCELLED;
        case MotionFeedbackType::ABORTED:
            return NCOrdinaryG00InflightState::K63_ABORTED;
        case MotionFeedbackType::FAULTED:
            return NCOrdinaryG00InflightState::K63_FAULTED;
        default:
            return NCOrdinaryG00InflightState::K63_EMPTY;
        }
    }

    static bool BoundaryValid(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept
    {
        return
            boundary.sequence != 0ULL &&
            boundary.source == NCFeedHoldSource::PROGRAM &&
            boundary.requestLatched &&
            boundary.requestExecutionEpoch !=
            MOTION_EXECUTION_EPOCH_INVALID &&
            boundary.requestOwner == MotionOwner::AUTO &&
            boundary.requestOwnerGeneration !=
            MOTION_OWNER_GENERATION_INVALID &&
            !boundary.failed &&
            !boundary.cancelled;
    }

    static bool MemberValid(
        const NCOrdinaryG00InflightEntrySnapshot& member) noexcept
    {
        return
            member.occupied &&
            member.active &&
            !member.terminal &&
            !member.revoked &&
            member.readAheadCutover &&
            member.registrySequence != 0ULL &&
            member.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            member.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            member.dispatchId != 0ULL &&
            member.commitSequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            member.identity.IsAssigned() &&
            member.ownerLease.IsValid();
    }

    static bool MembersDistinctAndOrdered(
        const NCOrdinaryG00InflightEntrySnapshot& first,
        const NCOrdinaryG00InflightEntrySnapshot& second) noexcept
    {
        return
            first.registrySequence < second.registrySequence&&
            first.entrySequence < second.entrySequence &&
            !SameEpochSegment(first.identity, second.identity);
    }

    static bool MemberMatchesBoundary(
        const NCOrdinaryG00InflightEntrySnapshot& member,
        const NCFeedHoldBoundarySnapshot& boundary) noexcept
    {
        return
            member.identity.epoch == boundary.requestExecutionEpoch &&
            member.ownerLease.owner == boundary.requestOwner &&
            member.ownerLease.generation ==
            boundary.requestOwnerGeneration;
    }

    bool BoundaryMatchesCapture(
        const NCFeedHoldBoundarySnapshot& boundary) const noexcept
    {
        return
            boundary.sequence == m_snapshot.boundarySequence &&
            boundary.source == NCFeedHoldSource::PROGRAM &&
            boundary.requestExecutionEpoch == m_snapshot.executionEpoch &&
            boundary.requestOwner == m_snapshot.owner &&
            boundary.requestOwnerGeneration == m_snapshot.ownerGeneration;
    }

    bool GateMatchesCapture(
        const NCFeedHoldResumeGateSnapshot& gate) const noexcept
    {
        return
            gate.boundarySequence == m_snapshot.boundarySequence &&
            gate.source == NCFeedHoldSource::PROGRAM &&
            gate.executionEpoch == m_snapshot.executionEpoch &&
            gate.owner == m_snapshot.owner &&
            gate.ownerGeneration == m_snapshot.ownerGeneration;
    }

    static void FillMember(
        const NCOrdinaryG00InflightEntrySnapshot& source,
        NCOrdinaryG00FeedHoldCohortMemberSnapshot& target) noexcept
    {
        target.registrySequence = source.registrySequence;
        target.session = source.session;
        target.entrySequence = source.entrySequence;
        target.dispatchId = source.dispatchId;
        target.commitSequence = source.commitSequence;
        target.sourcePC = source.sourcePC;
        target.sourceLineNumber = source.sourceLineNumber;
        target.identity = source.identity;
        target.ownerLease = source.ownerLease;
        target.captureState = source.state;
        target.currentState = source.state;
        target.lastFeedbackSequence = source.lastFeedbackSequence;
        target.lastFeedbackType = source.lastFeedbackType;
        target.exact = true;
    }

    void ObserveTerminal(
        std::size_t memberIndex,
        MotionFeedbackType type) noexcept
    {
        if (!IsTerminal(type))
        {
            ++m_counters.identityConflict;
            Fail(NCOrdinaryG00FeedHoldCohortDecision::K73_IDENTITY_CONFLICT);
            return;
        }
        if (memberIndex == 1U &&
            !m_snapshot.members[0].terminalObserved)
        {
            ++m_counters.terminalOutOfOrder;
            m_snapshot.terminalOrderValid = false;
            Fail(
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_TERMINAL_OUT_OF_ORDER);
            return;
        }

        NCOrdinaryG00FeedHoldCohortMemberSnapshot& member =
            m_snapshot.members[memberIndex];
        member.currentState = TerminalState(type);
        member.terminalObserved = true;
        member.terminalCompleted = type == MotionFeedbackType::COMPLETED;
        member.terminalBeforeAcknowledge =
            !m_snapshot.holdAcknowledged;
        member.terminalBeforeResume = !m_snapshot.resumeApplied;
        ++m_snapshot.terminalCount;
        member.terminalOrdinal = m_snapshot.terminalCount;

        ++m_counters.terminals;
        if (member.terminalCompleted)
        {
            ++m_counters.completedTerminals;
        }
        else
        {
            ++m_counters.interruptedTerminals;
        }
        if (member.terminalBeforeAcknowledge)
        {
            ++m_counters.terminalBeforeAcknowledge;
        }
        if (member.terminalBeforeResume)
        {
            ++m_counters.terminalBeforeResume;
        }

        if (m_snapshot.terminalCount <
            NC_ORDINARY_G00_FEED_HOLD_COHORT_SIZE)
        {
            m_snapshot.phase =
                NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_PENDING;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_WAIT_SECOND_TERMINAL;
            Publish();
            return;
        }

        m_snapshot.active = false;
        m_snapshot.terminalCohortComplete = true;
        m_snapshot.allMembersCompleted =
            m_snapshot.members[0].terminalCompleted &&
            m_snapshot.members[1].terminalCompleted;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortPhase::K73_TERMINAL_COMPLETE;
        ++m_counters.terminalCohorts;
        if (m_snapshot.allMembersCompleted)
        {
            ++m_counters.allCompletedCohorts;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_ALL_COMPLETED_EXACT;
        }
        else
        {
            ++m_counters.interruptedCohorts;
            m_snapshot.decision =
                NCOrdinaryG00FeedHoldCohortDecision::
                K73_INTERRUPTED_TERMINAL_EXACT;
        }
        Publish();
    }

    std::uint64_t AllocateCohortSequence() noexcept
    {
        const std::uint64_t sequence = m_nextCohortSequence;
        if (m_nextCohortSequence ==
            (std::numeric_limits<std::uint64_t>::max)())
        {
            m_nextCohortSequence = 1ULL;
        }
        else
        {
            ++m_nextCohortSequence;
        }
        return sequence;
    }

    void Fail(
        NCOrdinaryG00FeedHoldCohortDecision decision) noexcept
    {
        m_snapshot.active = false;
        m_snapshot.failed = true;
        m_snapshot.accountingValid = false;
        m_snapshot.phase =
            NCOrdinaryG00FeedHoldCohortPhase::K73_FAILED;
        m_snapshot.decision = decision;
        ++m_counters.failures;
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

    NCOrdinaryG00FeedHoldCohortSnapshot m_snapshot{};
    NCOrdinaryG00FeedHoldCohortCounters m_counters{};
    std::uint64_t m_nextCohortSequence = 1ULL;
};
