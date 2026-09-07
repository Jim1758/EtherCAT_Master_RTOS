#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2T / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Run Boundary
// Availability Transition Shadow.
//
// Classifies the availability of the S boundary certificate across two
// directly adjacent canonical S observations. A neutral-to-observed-head
// pair makes it available; a proven-to-direct-extension pair retains it;
// a proven-to-neutral pair loses availability; two neutral records remain
// unavailable. These are certificate states, not committed-run states.
// The unavailable reason is precisely S's current diagnostic, not an
// inferred cause, completed run, motion end or loss of geometric coverage.
// Invalid, malformed, stale or gapped records produce NO transition, not
// a genuine loss. Neutral HEAD_NOT_OBSERVED cannot follow proven S, and
// a direct coverage-boundary diagnostic cannot follow neutral S.
//
// Initial attachment may certify a complete valid adjacent pair. A single
// available extension without its previous S record is only a baseline,
// never a gain. Neutral-to-extension and proven-to-new-head are rejected.
// Retention checks both full S proofs, invariant six head references,
// direct six tail references, counts and coverage/run/input-chain context.
// Source R identities also advance directly when both are nonzero; zero
// belongs only to canonical S RUN_UNAVAILABLE and resets that anchor.
//
// The result retains only source S identities, availability and the current
// unavailable reason. Other source fields are checked during Observe and
// discarded. Previous T binding covers exactly this retained projection:
// it is NOT equality of complete S records or independent reconstruction of
// their proof. Same-thread trusted producers are assumed. No authentication
// against coherent forgery, full-wrap source reuse or permanent replay
// detection is provided. Freshness is local adjacency, not elapsed time.
// Null clears the local S source anchor. Rejected current identities are
// diagnostic resynchronization anchors; no missing transitions are bridged.
//
// Fixed two-record scalar history, NCManager heap-owned. No Observe-time
// allocation, large stack object, source snapshots, coordinates, displacement,
// endpoint arrays, nodes, segment/event list, complete order or reverse data.
// Not Path/Motion Queue, planner, STARTED/DONE proof, actual-position history,
// B2 breadcrumb or execution. Shadow-only and no control consumer. No Motion,
// G00, Gate/Registry, Alarm, PC, HMI/SHM/API, PDO, NIC, EtherCAT or DC changes;
// no logging, thread, timer, mutex, wait or sleep.

constexpr std::size_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail
{
    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : value + 1ULL;
    }

    template<std::size_t Count>
    bool ReservedIsZero(const std::array<std::uint8_t, Count>& reserved) noexcept
    {
        for (std::size_t index = 0U; index < reserved.size(); ++index)
        {
            if (reserved[index] != 0U)
            {
                return false;
            }
        }
        return true;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE = 2U,
    INVALID_CURRENT_BOUNDARY_RECORD = 3U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 4U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 5U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 6U,
    INVALID_PREVIOUS_SOURCE_BINDING = 7U,
    INVALID_BOUNDARY_PAIR_ADVANCE = 8U,
    INVALID_BOUNDARY_STATE_BINDING = 9U,
    INVALID_AVAILABILITY_TRANSITION = 10U,
    PROVEN_DIRECT_BOUNDARY_AVAILABILITY_TRANSITION = 11U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRelation : std::uint8_t
{
    NONE = 0U,
    BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD = 1U,
    RETAINED_BY_DIRECT_BOUNDARY_EXTENSION = 2U,
    BECAME_UNAVAILABLE = 3U,
    REMAINED_UNAVAILABLE = 4U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionAvailability : std::uint8_t
{
    UNKNOWN = 0U,
    UNAVAILABLE = 1U,
    AVAILABLE = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionUnavailableReason : std::uint8_t
{
    NONE = 0U,
    SOURCE_RUN_UNAVAILABLE = 1U,
    SOURCE_RUN_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY = 3U,
    RUN_HEAD_NOT_OBSERVED = 4U,
    DIRECT_COVERAGE_INTERVAL_BOUNDARY = 5U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRelation;
    using Availability = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionAvailability;
    using UnavailableReason = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionUnavailableReason;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousBoundaryPublicationSequence = 0ULL;
    std::uint64_t currentBoundaryPublicationSequence = 0ULL;
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Availability previousAvailability = Availability::UNKNOWN;
    Availability currentAvailability = Availability::UNKNOWN;
    UnavailableReason currentUnavailableReason = UnavailableReason::NONE;
    std::uint8_t reserved = 0U;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransition() const noexcept
    {
        if (publicationSequence == 0ULL || previousBoundaryPublicationSequence == 0ULL ||
            currentBoundaryPublicationSequence != NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(
                previousBoundaryPublicationSequence) ||
            schemaVersion != NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_SCHEMA_V1 ||
            disposition != Disposition::PROVEN_DIRECT_BOUNDARY_AVAILABILITY_TRANSITION ||
            reserved != 0U)
        {
            return false;
        }
        if (relation == Relation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD)
        {
            return previousAvailability == Availability::UNAVAILABLE &&
                currentAvailability == Availability::AVAILABLE &&
                currentUnavailableReason == UnavailableReason::NONE;
        }
        if (relation == Relation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION)
        {
            return previousAvailability == Availability::AVAILABLE &&
                currentAvailability == Availability::AVAILABLE &&
                currentUnavailableReason == UnavailableReason::NONE;
        }
        const bool reasonIsUnavailable =
            currentUnavailableReason == UnavailableReason::SOURCE_RUN_UNAVAILABLE ||
            currentUnavailableReason == UnavailableReason::SOURCE_RUN_NOT_PROVEN_CAUSE_UNKNOWN ||
            currentUnavailableReason == UnavailableReason::SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY ||
            currentUnavailableReason == UnavailableReason::RUN_HEAD_NOT_OBSERVED ||
            currentUnavailableReason == UnavailableReason::DIRECT_COVERAGE_INTERVAL_BOUNDARY;
        if (relation == Relation::BECAME_UNAVAILABLE)
        {
            return previousAvailability == Availability::AVAILABLE &&
                currentAvailability == Availability::UNAVAILABLE && reasonIsUnavailable &&
                currentUnavailableReason != UnavailableReason::RUN_HEAD_NOT_OBSERVED;
        }
        return relation == Relation::REMAINED_UNAVAILABLE &&
            previousAvailability == Availability::UNAVAILABLE &&
            currentAvailability == Availability::UNAVAILABLE && reasonIsUnavailable &&
            currentUnavailableReason != UnavailableReason::DIRECT_COVERAGE_INTERVAL_BOUNDARY;
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRelation;
    using Availability = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionAvailability;
    using UnavailableReason = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionUnavailableReason;
    using BoundaryRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRecordV1;
    using BoundaryDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryDisposition;
    using BoundaryRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryRelation;
    using BoundaryExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        void ObserveImmediateQualificationTransitionPairContinuityRunBoundaryAvailabilitySameThread(
            const BoundaryRecord* previousBoundary, const BoundaryRecord* currentBoundary) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const bool previousObservationIsProven = previousObservation != nullptr &&
            previousObservation->IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransition();
        const bool previousObservationIsValid = previousObservation == nullptr ||
            (previousObservation->publicationSequence == m_publicationSequence &&
                (previousObservationIsProven || IsCanonicalNonProvenTransition(*previousObservation)));
        const std::uint64_t previouslyObservedSourceSequence = previousObservation == nullptr
            ? 0ULL : previousObservation->currentBoundaryPublicationSequence;
        m_publicationSequence = NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentBoundary == nullptr ? 0ULL : currentBoundary->publicationSequence);

        if (!previousObservationIsValid)
        {
            target.disposition = Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        }
        else if (currentBoundary == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentBoundary->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else
        {
            EvaluatePair(previousBoundary, *currentBoundary, previousObservation,
                previousObservationIsProven, previouslyObservedSourceSequence, target);
        }

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount < HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const Record* GetNewestObservationSameThread(std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }
        const std::size_t index = (static_cast<std::size_t>(m_latestIndex) +
            HISTORY_CAPACITY - historyOffset) % HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_HISTORY_CAPACITY;

    static UnavailableReason GetUnavailableReason(BoundaryDisposition disposition) noexcept
    {
        switch (disposition)
        {
        case BoundaryDisposition::NOT_APPLICABLE_RUN_UNAVAILABLE:
            return UnavailableReason::SOURCE_RUN_UNAVAILABLE;
        case BoundaryDisposition::NOT_APPLICABLE_RUN_NOT_PROVEN_CAUSE_UNKNOWN:
            return UnavailableReason::SOURCE_RUN_NOT_PROVEN_CAUSE_UNKNOWN;
        case BoundaryDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY:
            return UnavailableReason::SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
        case BoundaryDisposition::NOT_APPLICABLE_RUN_HEAD_NOT_OBSERVED:
            return UnavailableReason::RUN_HEAD_NOT_OBSERVED;
        case BoundaryDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY:
            return UnavailableReason::DIRECT_COVERAGE_INTERVAL_BOUNDARY;
        default:
            return UnavailableReason::NONE;
        }
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        static bool IsCanonicalNeutralBoundary(const BoundaryRecord& source) noexcept
    {
        const UnavailableReason reason = GetUnavailableReason(source.disposition);
        const bool diagnosticIdentityIsValid =
            reason == UnavailableReason::SOURCE_RUN_UNAVAILABLE
            ? source.sourceRunPublicationSequence == 0ULL
            : source.sourceRunPublicationSequence != 0ULL;
        return reason != UnavailableReason::NONE && diagnosticIdentityIsValid &&
            source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_SCHEMA_V1 &&
            source.firstRunPublicationSequence == 0ULL &&
            source.firstContinuityPublicationSequence == 0ULL && source.latestContinuityPublicationSequence == 0ULL &&
            source.firstPairPublicationSequence == 0ULL && source.latestPairPublicationSequence == 0ULL &&
            source.firstTransitionPublicationSequence == 0ULL && source.latestTransitionPublicationSequence == 0ULL &&
            source.firstQualificationPublicationSequence == 0ULL && source.latestQualificationPublicationSequence == 0ULL &&
            source.firstSummaryPublicationSequence == 0ULL && source.latestSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.firstSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedFirstProvenTransitionCount == 0U &&
            source.latestSharedLastProvenTransitionCount == 0U &&
            source.consecutiveContinuityCount == 0U &&
            source.relation == BoundaryRelation::NONE && source.extent == BoundaryExtent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::ReservedIsZero(source.reserved);
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        static bool IsCanonicalNonProvenTransition(const Record& source) noexcept
    {
        const bool dispositionIsValid =
            source.disposition == Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE ||
            source.disposition == Disposition::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE ||
            source.disposition == Disposition::INVALID_CURRENT_BOUNDARY_RECORD ||
            source.disposition == Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_TRANSITION_RECORD ||
            source.disposition == Disposition::INVALID_PREVIOUS_SOURCE_BINDING ||
            source.disposition == Disposition::INVALID_BOUNDARY_PAIR_ADVANCE ||
            source.disposition == Disposition::INVALID_BOUNDARY_STATE_BINDING ||
            source.disposition == Disposition::INVALID_AVAILABILITY_TRANSITION;
        const bool diagnosticIdentityIsValid = source.disposition ==
            Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE
            ? source.currentBoundaryPublicationSequence == 0ULL
            : source.currentBoundaryPublicationSequence != 0ULL ||
            source.disposition == Disposition::INVALID_CURRENT_BOUNDARY_RECORD ||
            source.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        return dispositionIsValid && diagnosticIdentityIsValid &&
            source.publicationSequence != 0ULL &&
            source.previousBoundaryPublicationSequence == 0ULL &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_SCHEMA_V1 &&
            source.relation == Relation::NONE &&
            source.previousAvailability == Availability::UNKNOWN &&
            source.currentAvailability == Availability::UNKNOWN &&
            source.currentUnavailableReason == UnavailableReason::NONE && source.reserved == 0U;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        static bool DirectBoundaryExtensionMatches(const BoundaryRecord& previous,
            const BoundaryRecord& current) noexcept
    {
        return current.disposition == BoundaryDisposition::PROVEN_DIRECT_RUN_BOUNDARY_EXTENSION &&
            current.firstRunPublicationSequence == previous.firstRunPublicationSequence &&
            current.firstContinuityPublicationSequence == previous.firstContinuityPublicationSequence &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstQualificationPublicationSequence == previous.firstQualificationPublicationSequence &&
            current.firstSummaryPublicationSequence == previous.firstSummaryPublicationSequence &&
            current.firstSharedFirstProvenTransitionCount == previous.firstSharedFirstProvenTransitionCount &&
            current.coverageGeneration == previous.coverageGeneration &&
            current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration == previous.acceptedInputChainGeneration &&
            previous.consecutiveContinuityCount != (std::numeric_limits<std::uint32_t>::max)() &&
            current.consecutiveContinuityCount == previous.consecutiveContinuityCount + 1U &&
            current.latestSharedFirstProvenTransitionCount == previous.latestSharedLastProvenTransitionCount &&
            current.sourceRunPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.latestContinuityPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.latestContinuityPublicationSequence) &&
            current.latestPairPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.latestPairPublicationSequence) &&
            current.latestTransitionPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.latestTransitionPublicationSequence) &&
            current.latestQualificationPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.latestQualificationPublicationSequence) &&
            current.latestSummaryPublicationSequence == NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previous.latestSummaryPublicationSequence);
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE
        static void EvaluatePair(const BoundaryRecord* previousBoundary,
            const BoundaryRecord& currentBoundary, const Record* previousObservation,
            bool previousObservationIsProven, std::uint64_t previouslyObservedSourceSequence,
            Record& target) noexcept
    {
        const bool currentIsProven = currentBoundary.
            IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundary();
        if (!currentIsProven && !IsCanonicalNeutralBoundary(currentBoundary))
        {
            target.disposition = Disposition::INVALID_CURRENT_BOUNDARY_RECORD;
            return;
        }
        if (previousBoundary == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE;
            return;
        }
        const bool previousIsProven = previousBoundary->
            IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundary();
        if (!previousIsProven && !IsCanonicalNeutralBoundary(*previousBoundary))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_BOUNDARY_RECORD;
            return;
        }
        if (currentBoundary.publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previousBoundary->publicationSequence))
        {
            target.disposition = Disposition::INVALID_BOUNDARY_PAIR_ADVANCE;
            return;
        }
        if (previouslyObservedSourceSequence != 0ULL &&
            previousBoundary->publicationSequence != previouslyObservedSourceSequence)
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        const Availability previousAvailability = previousIsProven
            ? Availability::AVAILABLE : Availability::UNAVAILABLE;
        const UnavailableReason previousReason = GetUnavailableReason(previousBoundary->disposition);
        if (previousObservationIsProven && previousObservation != nullptr &&
            (previousAvailability != previousObservation->currentAvailability ||
                previousReason != previousObservation->currentUnavailableReason))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (previousBoundary->sourceRunPublicationSequence != 0ULL &&
            currentBoundary.sourceRunPublicationSequence != 0ULL &&
            currentBoundary.sourceRunPublicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDetail::NextNonZeroSequence(previousBoundary->sourceRunPublicationSequence))
        {
            target.disposition = Disposition::INVALID_BOUNDARY_PAIR_ADVANCE;
            return;
        }
        const UnavailableReason currentReason = GetUnavailableReason(currentBoundary.disposition);
        Relation relation = Relation::NONE;
        if (currentIsProven)
        {
            if (previousIsProven)
            {
                if (!DirectBoundaryExtensionMatches(*previousBoundary, currentBoundary))
                {
                    target.disposition = Disposition::INVALID_BOUNDARY_STATE_BINDING;
                    return;
                }
                relation = Relation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION;
            }
            else
            {
                if (currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_RUN_HEAD ||
                    currentBoundary.consecutiveContinuityCount != 1U)
                {
                    target.disposition = Disposition::INVALID_BOUNDARY_STATE_BINDING;
                    return;
                }
                relation = Relation::BECAME_AVAILABLE_AT_OBSERVED_RUN_HEAD;
            }
        }
        else
        {
            if ((previousIsProven && currentReason == UnavailableReason::RUN_HEAD_NOT_OBSERVED) ||
                (!previousIsProven && currentReason == UnavailableReason::DIRECT_COVERAGE_INTERVAL_BOUNDARY))
            {
                target.disposition = Disposition::INVALID_BOUNDARY_STATE_BINDING;
                return;
            }
            relation = previousIsProven ? Relation::BECAME_UNAVAILABLE : Relation::REMAINED_UNAVAILABLE;
        }
        target.previousBoundaryPublicationSequence = previousBoundary->publicationSequence;
        target.previousAvailability = previousAvailability;
        target.currentAvailability = currentIsProven ? Availability::AVAILABLE : Availability::UNAVAILABLE;
        target.currentUnavailableReason = currentReason;
        target.relation = relation;
        target.disposition = Disposition::PROVEN_DIRECT_BOUNDARY_AVAILABILITY_TRANSITION;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransition())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_AVAILABILITY_TRANSITION;
        }
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.previousBoundaryPublicationSequence = 0ULL;
        target.relation = Relation::NONE;
        target.previousAvailability = Availability::UNKNOWN;
        target.currentAvailability = Availability::UNKNOWN;
        target.currentUnavailableReason = UnavailableReason::NONE;
        target.reserved = 0U;
    }

    static void ResetTarget(Record& target, std::uint64_t publicationSequence,
        std::uint64_t currentBoundaryPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.currentBoundaryPublicationSequence = currentBoundaryPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_RUN_BOUNDARY_AVAILABILITY_TRANSITION_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionDisposition) == 1U,
    "Boundary availability disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRelation) == 1U,
    "Boundary availability relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionAvailability) == 1U,
    "Boundary availability state must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionUnavailableReason) == 1U,
    "Boundary unavailable reason must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1>::value,
    "Boundary availability transition record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1>::value,
    "Boundary availability transition record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1) == 8U,
    "Boundary availability transition record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1) == 32U,
    "Boundary availability transition record must remain exactly 32 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1, schemaVersion) == 24U,
    "Boundary availability transition schema offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1, previousAvailability) == 28U,
    "Boundary availability transition state block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionRecordV1, reserved) == 31U,
    "Boundary availability transition reserved offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow>::value,
    "Boundary availability transition observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow>::value,
    "Boundary availability transition observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow) == 8U,
    "Boundary availability transition observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow) == 80U,
    "Boundary availability transition observer must remain exactly 80 bytes.");
