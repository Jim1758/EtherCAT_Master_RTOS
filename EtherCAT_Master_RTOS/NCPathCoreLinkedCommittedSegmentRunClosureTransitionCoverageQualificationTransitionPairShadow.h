#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2P / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Relation Shadow.
//
// Compares two directly adjacent proven L.2O transitions, A -> B and B -> C,
// in the same coverage interval, run and accepted-input chain.  Their shared
// N/M source identities, transition count and RETAINED SCALAR PROJECTION of
// B (qualification flags, changed-axis universe and three pending masks)
// must match exactly.  L.2O does not retain the complete N record: this is
// neither full intermediate-N equality nor independent reconstruction of
// upstream geometry proof.  Source records are trusted producer records;
// structural validation is not authentication against coherent forgery.
//
// The compact result describes only this immediate pair: first-edge gains
// retained/lost at C, first-edge losses regained/still absent at C, and the
// subset of newly pending axes covered by the next edge.  Expansion-related
// losses are restricted to all-axis qualification bits.  FULL_CLOSURE_CURRENT
// in lost-then-regained describes exit then entry across this pair; it does
// not establish a newly gained historical re-entry flag or an exact count.
// Axes first introduced on A -> B are OPEN at B.  In the next edge their
// returned-state and direct-return recovery must coincide; they cannot yet
// have direct-reopen or bidirectional coverage.  The third recovered-pending
// mask is retained for explicit symmetry but must be zero for a proven pair.
// No result predicts eventual recovery, a durable milestone, or whole-run
// coverage; a coverage interval may be only a proven suffix of a run.
//
// Invalid, neutral, unavailable, nonadjacent and mismatched sources clear
// all proven payload.  A canonical source boundary diagnostic is explicitly
// source-reported, and requires adjacent structurally valid O publications;
// it does not reveal the cause of the boundary.  Freshness is local producer
// publication adjacency, not time freshness or durable replay detection.
// A first attachment may accept any valid direct pair; null clears the
// freshness baseline; a rejected publication becomes only a diagnostic
// resynchronization baseline for the next direct publication.
//
// Fixed two-record scalar history, NCManager heap-owned, no Observe-time
// allocation or large stack snapshot.  No coordinates, displacement,
// endpoint arrays, intermediate nodes, segment/event list, complete event
// order or reverse-traversal data.  Not Path/Motion Queue, planner,
// STARTED/DONE proof, actual-position history, B2 breadcrumb or execution.
// Shadow-only: no Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API, PDO,
// NIC, EtherCAT or DC consumer; no logging, thread, timer, mutex, wait/sleep.

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionPairDetail
{
    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : value + 1ULL;
    }

    constexpr std::uint16_t Difference(std::uint16_t left,
        std::uint16_t right) noexcept
    {
        return static_cast<std::uint16_t>(static_cast<std::uint32_t>(left) &
            ~static_cast<std::uint32_t>(right));
    }

    constexpr std::uint16_t ALL_AXIS_FLAGS = 0x001FU;
    constexpr std::uint16_t HISTORICAL_FULL_FLAGS = 0x01A0U;
    constexpr std::uint16_t ALL_FLAGS = 0x01FFU;
    constexpr std::uint32_t AXIS_MASK = 0xFFU;

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

    inline bool FlagShapeIsValid(std::uint16_t flags,
        std::uint32_t universe) noexcept
    {
        const bool returned = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED) != 0U;
        const bool both = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED) != 0U;
        const bool directReturn = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED) != 0U;
        const bool directReopen = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED) != 0U;
        const bool bidirectional = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED) != 0U;
        const bool observed = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED) != 0U;
        const bool current = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT) != 0U;
        const bool exit = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED) != 0U;
        const bool reentry = (flags &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED) != 0U;
        return (flags & ~ALL_FLAGS) == 0U && (universe != 0U || flags == 0U) &&
            (!both || returned) && (!directReturn || both) &&
            (!directReopen || both) &&
            (bidirectional == (directReturn && directReopen)) &&
            (!current || (observed && returned)) && (!exit || observed) &&
            (!reentry || (observed && exit));
    }

    inline bool PendingFlagsMatch(std::uint16_t flags, std::uint32_t universe,
        std::uint32_t returned, std::uint32_t direct,
        std::uint32_t bidirectional) noexcept
    {
        return FlagShapeIsValid(flags, universe) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && returned == 0U) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && direct == 0U) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && bidirectional == 0U);
    }

    inline bool ClosureFlagsAdvanceExactly(std::uint16_t previous,
        std::uint16_t current) noexcept
    {
        const bool previousObserved = (previous &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED) != 0U;
        const bool currentObserved = (current &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED) != 0U;
        const bool previousCurrent = (previous &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT) != 0U;
        const bool currentCurrent = (current &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT) != 0U;
        const bool previousExit = (previous &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED) != 0U;
        const bool currentExit = (current &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED) != 0U;
        const bool previousReentry = (previous &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED) != 0U;
        const bool currentReentry = (current &
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED) != 0U;
        return currentObserved == (previousObserved || currentCurrent) &&
            currentExit == (previousExit || (previousCurrent && !currentCurrent)) &&
            currentReentry == (previousReentry ||
                (previousObserved && !previousCurrent && currentCurrent));
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_TRANSITION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE = 3U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN = 4U,
    NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY = 5U,
    INVALID_CURRENT_TRANSITION_RECORD = 6U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 7U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 8U,
    INVALID_TRANSITION_ADVANCE_FENCE = 9U,
    INVALID_SHARED_QUALIFICATION_BINDING = 10U,
    INVALID_QUALIFICATION_TRANSITION_PAIR = 11U,
    PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR = 12U,
    NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY = 13U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRelation : std::uint8_t
{
    NONE = 0U,
    DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR = 1U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairExtent : std::uint8_t
{
    NONE = 0U,
    SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_RELATION_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairExtent;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousTransitionPublicationSequence = 0ULL;
    std::uint64_t currentTransitionPublicationSequence = 0ULL;
    std::uint64_t sharedQualificationPublicationSequence = 0ULL;
    std::uint64_t sharedSummaryPublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;

    std::uint32_t firstProvenTransitionCount = 0U;
    std::uint32_t sharedProvenTransitionCount = 0U;
    std::uint32_t latestProvenTransitionCount = 0U;
    std::uint32_t firstCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t sharedCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t latestCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t sharedPendingReturnedStateAxisMask = 0U;
    std::uint32_t latestPendingReturnedStateAxisMask = 0U;
    std::uint32_t sharedPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t latestPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t sharedPendingBidirectionalTransitionAxisMask = 0U;
    std::uint32_t latestPendingBidirectionalTransitionAxisMask = 0U;
    std::uint32_t newlyPendingThenReturnedStateCoveredAxisMask = 0U;
    std::uint32_t newlyPendingThenDirectReturnCoveredAxisMask = 0U;
    std::uint32_t newlyPendingThenBidirectionalCoveredAxisMask = 0U;

    std::uint16_t schemaVersion = 0U;
    std::uint16_t firstQualificationFlags = 0U;
    std::uint16_t sharedQualificationFlags = 0U;
    std::uint16_t latestQualificationFlags = 0U;
    std::uint16_t gainedThenRetainedQualificationFlags = 0U;
    std::uint16_t gainedThenLostQualificationFlags = 0U;
    std::uint16_t lostThenRegainedQualificationFlags = 0U;
    std::uint16_t lostThenStillAbsentQualificationFlags = 0U;
    std::uint16_t universeExpansionLostThenRegainedQualificationFlags = 0U;
    std::uint16_t universeExpansionLostThenStillAbsentQualificationFlags = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Extent extent = Extent::NONE;
    std::array<std::uint8_t, 5U> reserved{};

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransitionPair() const noexcept
    {
        return publicationSequence != 0ULL && previousTransitionPublicationSequence != 0ULL &&
            currentTransitionPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairDetail::NextNonZeroSequence(previousTransitionPublicationSequence) &&
            sharedQualificationPublicationSequence != 0ULL &&
            sharedSummaryPublicationSequence != 0ULL && coverageGeneration != 0ULL &&
            runGeneration != 0ULL && acceptedInputChainGeneration != 0ULL &&
            firstProvenTransitionCount != 0U &&
            firstProvenTransitionCount != (std::numeric_limits<std::uint32_t>::max)() &&
            sharedProvenTransitionCount == firstProvenTransitionCount + 1U &&
            sharedProvenTransitionCount != (std::numeric_limits<std::uint32_t>::max)() &&
            latestProvenTransitionCount == sharedProvenTransitionCount + 1U &&
            schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_SCHEMA_V1 &&
            disposition == Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR &&
            relation == Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR &&
            extent == Extent::SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_RELATION_ONLY &&
            AxisAndPendingMasksAreValid() && QualificationRelationsAreValid() &&
            NCPathCoreReturnQualificationTransitionPairDetail::ReservedIsZero(reserved);
    }

private:
    bool PendingRelationIsValid(std::uint32_t sharedPending,
        std::uint32_t latestPending, std::uint32_t newlyCovered) const noexcept
    {
        const std::uint32_t firstNewAxes = sharedCoordinateChangeAxisUnionMask &
            ~firstCoordinateChangeAxisUnionMask;
        const std::uint32_t latestNewAxes = latestCoordinateChangeAxisUnionMask &
            ~sharedCoordinateChangeAxisUnionMask;
        return (sharedPending & ~sharedCoordinateChangeAxisUnionMask) == 0U &&
            (latestPending & ~latestCoordinateChangeAxisUnionMask) == 0U &&
            (firstNewAxes & ~sharedPending) == 0U &&
            (latestPending & ~sharedPending) == latestNewAxes &&
            newlyCovered == (firstNewAxes & sharedPending & ~latestPending);
    }

    bool AxisAndPendingMasksAreValid() const noexcept
    {
        return (latestCoordinateChangeAxisUnionMask &
            ~NCPathCoreReturnQualificationTransitionPairDetail::AXIS_MASK) == 0U &&
            (firstCoordinateChangeAxisUnionMask & ~sharedCoordinateChangeAxisUnionMask) == 0U &&
            (sharedCoordinateChangeAxisUnionMask & ~latestCoordinateChangeAxisUnionMask) == 0U &&
            PendingRelationIsValid(sharedPendingReturnedStateAxisMask,
                latestPendingReturnedStateAxisMask, newlyPendingThenReturnedStateCoveredAxisMask) &&
            PendingRelationIsValid(sharedPendingDirectReturnTransitionAxisMask,
                latestPendingDirectReturnTransitionAxisMask, newlyPendingThenDirectReturnCoveredAxisMask) &&
            PendingRelationIsValid(sharedPendingBidirectionalTransitionAxisMask,
                latestPendingBidirectionalTransitionAxisMask, newlyPendingThenBidirectionalCoveredAxisMask) &&
            (sharedPendingReturnedStateAxisMask & ~sharedPendingDirectReturnTransitionAxisMask) == 0U &&
            (latestPendingReturnedStateAxisMask & ~latestPendingDirectReturnTransitionAxisMask) == 0U &&
            (sharedPendingDirectReturnTransitionAxisMask & ~sharedPendingBidirectionalTransitionAxisMask) == 0U &&
            (latestPendingDirectReturnTransitionAxisMask & ~latestPendingBidirectionalTransitionAxisMask) == 0U &&
            newlyPendingThenReturnedStateCoveredAxisMask == newlyPendingThenDirectReturnCoveredAxisMask &&
            newlyPendingThenBidirectionalCoveredAxisMask == 0U;
    }

    bool QualificationRelationsAreValid() const noexcept
    {
        const std::uint16_t firstGain =
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(sharedQualificationFlags, firstQualificationFlags);
        const std::uint16_t firstLoss =
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(firstQualificationFlags, sharedQualificationFlags);
        const std::uint16_t latestLoss =
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(sharedQualificationFlags, latestQualificationFlags);
        return NCPathCoreReturnQualificationTransitionPairDetail::FlagShapeIsValid(
            firstQualificationFlags, firstCoordinateChangeAxisUnionMask) &&
            NCPathCoreReturnQualificationTransitionPairDetail::PendingFlagsMatch(
                sharedQualificationFlags, sharedCoordinateChangeAxisUnionMask,
                sharedPendingReturnedStateAxisMask, sharedPendingDirectReturnTransitionAxisMask,
                sharedPendingBidirectionalTransitionAxisMask) &&
            NCPathCoreReturnQualificationTransitionPairDetail::PendingFlagsMatch(
                latestQualificationFlags, latestCoordinateChangeAxisUnionMask,
                latestPendingReturnedStateAxisMask, latestPendingDirectReturnTransitionAxisMask,
                latestPendingBidirectionalTransitionAxisMask) &&
            NCPathCoreReturnQualificationTransitionPairDetail::ClosureFlagsAdvanceExactly(
                firstQualificationFlags, sharedQualificationFlags) &&
            NCPathCoreReturnQualificationTransitionPairDetail::ClosureFlagsAdvanceExactly(
                sharedQualificationFlags, latestQualificationFlags) &&
            (firstCoordinateChangeAxisUnionMask == sharedCoordinateChangeAxisUnionMask ||
                (latestQualificationFlags &
                    (NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED |
                        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED)) == 0U) &&
            ((firstLoss | latestLoss) &
                NCPathCoreReturnQualificationTransitionPairDetail::HISTORICAL_FULL_FLAGS) == 0U &&
            ((firstLoss & NCPathCoreReturnQualificationTransitionPairDetail::ALL_AXIS_FLAGS) == 0U ||
                sharedCoordinateChangeAxisUnionMask != firstCoordinateChangeAxisUnionMask) &&
            ((latestLoss & NCPathCoreReturnQualificationTransitionPairDetail::ALL_AXIS_FLAGS) == 0U ||
                latestCoordinateChangeAxisUnionMask != sharedCoordinateChangeAxisUnionMask) &&
            gainedThenRetainedQualificationFlags == static_cast<std::uint16_t>(firstGain & latestQualificationFlags) &&
            gainedThenLostQualificationFlags ==
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(firstGain, latestQualificationFlags) &&
            lostThenRegainedQualificationFlags == static_cast<std::uint16_t>(firstLoss & latestQualificationFlags) &&
            lostThenStillAbsentQualificationFlags ==
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(firstLoss, latestQualificationFlags) &&
            universeExpansionLostThenRegainedQualificationFlags == static_cast<std::uint16_t>(
                lostThenRegainedQualificationFlags & NCPathCoreReturnQualificationTransitionPairDetail::ALL_AXIS_FLAGS) &&
            universeExpansionLostThenStillAbsentQualificationFlags == static_cast<std::uint16_t>(
                lostThenStillAbsentQualificationFlags & NCPathCoreReturnQualificationTransitionPairDetail::ALL_AXIS_FLAGS);
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairExtent;
    using TransitionRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1;
    using TransitionDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionDisposition;
    using TransitionRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRelation;
    using TransitionExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE
        void ObserveImmediateQualificationTransitionPairSameThread(
            const TransitionRecord* previousTransition,
            const TransitionRecord* currentTransition) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const std::uint64_t previouslyObservedSourceSequence = previousObservation == nullptr
            ? 0ULL : previousObservation->currentTransitionPublicationSequence;
        m_publicationSequence =
            NCPathCoreReturnQualificationTransitionPairDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentTransition == nullptr ? 0ULL : currentTransition->publicationSequence);

        if (currentTransition == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_TRANSITION_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentTransition->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairDetail::NextNonZeroSequence(previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else if (IsCanonicalNonProvenTransition(*currentTransition))
        {
            ClassifyNonProvenCurrent(previousTransition, *currentTransition, target);
        }
        else if (!IsFullyProvenTransition(*currentTransition))
        {
            target.disposition = Disposition::INVALID_CURRENT_TRANSITION_RECORD;
        }
        else if (previousTransition == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE;
        }
        else if (IsCanonicalNonProvenTransition(*previousTransition))
        {
            target.disposition = IsNeutralTransitionDisposition(previousTransition->disposition)
                ? Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN
                : Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        }
        else if (!IsFullyProvenTransition(*previousTransition))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        }
        else if (currentTransition->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairDetail::NextNonZeroSequence(previousTransition->publicationSequence))
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE_FENCE;
        }
        else if (currentTransition->coverageGeneration != previousTransition->coverageGeneration)
        {
            target.disposition = Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY;
        }
        else if (currentTransition->runGeneration != previousTransition->runGeneration ||
            currentTransition->acceptedInputChainGeneration != previousTransition->acceptedInputChainGeneration)
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE_FENCE;
        }
        else if (!SharedQualificationProjectionMatches(*previousTransition, *currentTransition))
        {
            target.disposition = Disposition::INVALID_SHARED_QUALIFICATION_BINDING;
        }
        else
        {
            PopulatePair(*previousTransition, *currentTransition, target);
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
    static constexpr std::size_t HISTORY_CAPACITY =
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_HISTORY_CAPACITY;

    static bool IsNeutralTransitionDisposition(TransitionDisposition disposition) noexcept
    {
        return disposition == TransitionDisposition::NOT_APPLICABLE_QUALIFICATION_UNAVAILABLE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == TransitionDisposition::NOT_APPLICABLE_QUALIFICATION_BASELINE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_QUALIFICATION_UNAVAILABLE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN;
    }

    static bool IsInvalidTransitionDisposition(TransitionDisposition disposition) noexcept
    {
        return disposition == TransitionDisposition::INVALID_CURRENT_QUALIFICATION_RECORD ||
            disposition == TransitionDisposition::INVALID_PREVIOUS_QUALIFICATION_RECORD ||
            disposition == TransitionDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            disposition == TransitionDisposition::INVALID_QUALIFICATION_ADVANCE_FENCE ||
            disposition == TransitionDisposition::INVALID_QUALIFICATION_TRANSITION;
    }

    static bool IsFullyProvenTransition(const TransitionRecord& source) noexcept
    {
        return source.IsProvenRunEndpointReturnCoverageQualificationTransition() &&
            NCPathCoreReturnQualificationTransitionPairDetail::ClosureFlagsAdvanceExactly(
                source.previousQualificationFlags, source.currentQualificationFlags);
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE
        static bool IsCanonicalNonProvenTransition(const TransitionRecord& source) noexcept
    {
        const bool diagnosticIdentityIsValid = source.disposition ==
            TransitionDisposition::NOT_APPLICABLE_QUALIFICATION_UNAVAILABLE
            ? source.currentQualificationPublicationSequence == 0ULL
            : source.currentQualificationPublicationSequence != 0ULL ||
            source.disposition == TransitionDisposition::INVALID_CURRENT_QUALIFICATION_RECORD ||
            source.disposition == TransitionDisposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        return (IsNeutralTransitionDisposition(source.disposition) ||
            IsInvalidTransitionDisposition(source.disposition)) &&
            source.publicationSequence != 0ULL && diagnosticIdentityIsValid &&
            source.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_SCHEMA_V1 &&
            source.previousQualificationPublicationSequence == 0ULL &&
            source.previousSummaryPublicationSequence == 0ULL &&
            source.currentSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.previousProvenTransitionCount == 0U && source.currentProvenTransitionCount == 0U &&
            source.previousCoordinateChangeAxisUnionMask == 0U &&
            source.currentCoordinateChangeAxisUnionMask == 0U && source.newlyChangedAxisMask == 0U &&
            source.previousPendingReturnedStateAxisMask == 0U && source.currentPendingReturnedStateAxisMask == 0U &&
            source.previousPendingDirectReturnTransitionAxisMask == 0U &&
            source.currentPendingDirectReturnTransitionAxisMask == 0U &&
            source.previousPendingBidirectionalTransitionAxisMask == 0U &&
            source.currentPendingBidirectionalTransitionAxisMask == 0U &&
            source.addedPendingReturnedStateAxisMask == 0U && source.removedPendingReturnedStateAxisMask == 0U &&
            source.addedPendingDirectReturnTransitionAxisMask == 0U &&
            source.removedPendingDirectReturnTransitionAxisMask == 0U &&
            source.addedPendingBidirectionalTransitionAxisMask == 0U &&
            source.removedPendingBidirectionalTransitionAxisMask == 0U &&
            source.previousQualificationFlags == 0U && source.currentQualificationFlags == 0U &&
            source.newlyGainedQualificationFlags == 0U && source.retainedQualificationFlags == 0U &&
            source.lostQualificationFlags == 0U && source.universeExpansionLostQualificationFlags == 0U &&
            source.relation == TransitionRelation::NONE && source.extent == TransitionExtent::NONE &&
            NCPathCoreReturnQualificationTransitionPairDetail::ReservedIsZero(source.reserved);
    }

    static void ClassifyNonProvenCurrent(const TransitionRecord* previous,
        const TransitionRecord& current, Record& target) noexcept
    {
        if (!IsNeutralTransitionDisposition(current.disposition))
        {
            target.disposition = Disposition::INVALID_CURRENT_TRANSITION_RECORD;
        }
        else if (current.disposition != TransitionDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN;
        }
        else if (previous == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN;
        }
        else if (!IsFullyProvenTransition(*previous) &&
            !IsCanonicalNonProvenTransition(*previous))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_TRANSITION_RECORD;
        }
        else if (current.publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairDetail::NextNonZeroSequence(previous->publicationSequence))
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE_FENCE;
        }
        else
        {
            target.disposition = Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
        }
    }

    static bool SharedQualificationProjectionMatches(const TransitionRecord& previous,
        const TransitionRecord& current) noexcept
    {
        return previous.currentQualificationPublicationSequence == current.previousQualificationPublicationSequence &&
            previous.currentSummaryPublicationSequence == current.previousSummaryPublicationSequence &&
            previous.currentProvenTransitionCount == current.previousProvenTransitionCount &&
            previous.currentCoordinateChangeAxisUnionMask == current.previousCoordinateChangeAxisUnionMask &&
            previous.currentQualificationFlags == current.previousQualificationFlags &&
            previous.currentPendingReturnedStateAxisMask == current.previousPendingReturnedStateAxisMask &&
            previous.currentPendingDirectReturnTransitionAxisMask == current.previousPendingDirectReturnTransitionAxisMask &&
            previous.currentPendingBidirectionalTransitionAxisMask == current.previousPendingBidirectionalTransitionAxisMask;
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.previousTransitionPublicationSequence = 0ULL;
        target.sharedQualificationPublicationSequence = 0ULL;
        target.sharedSummaryPublicationSequence = 0ULL;
        target.coverageGeneration = 0ULL;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.firstProvenTransitionCount = 0U;
        target.sharedProvenTransitionCount = 0U;
        target.latestProvenTransitionCount = 0U;
        target.firstCoordinateChangeAxisUnionMask = 0U;
        target.sharedCoordinateChangeAxisUnionMask = 0U;
        target.latestCoordinateChangeAxisUnionMask = 0U;
        target.sharedPendingReturnedStateAxisMask = 0U;
        target.latestPendingReturnedStateAxisMask = 0U;
        target.sharedPendingDirectReturnTransitionAxisMask = 0U;
        target.latestPendingDirectReturnTransitionAxisMask = 0U;
        target.sharedPendingBidirectionalTransitionAxisMask = 0U;
        target.latestPendingBidirectionalTransitionAxisMask = 0U;
        target.newlyPendingThenReturnedStateCoveredAxisMask = 0U;
        target.newlyPendingThenDirectReturnCoveredAxisMask = 0U;
        target.newlyPendingThenBidirectionalCoveredAxisMask = 0U;
        target.firstQualificationFlags = 0U;
        target.sharedQualificationFlags = 0U;
        target.latestQualificationFlags = 0U;
        target.gainedThenRetainedQualificationFlags = 0U;
        target.gainedThenLostQualificationFlags = 0U;
        target.lostThenRegainedQualificationFlags = 0U;
        target.lostThenStillAbsentQualificationFlags = 0U;
        target.universeExpansionLostThenRegainedQualificationFlags = 0U;
        target.universeExpansionLostThenStillAbsentQualificationFlags = 0U;
        target.relation = Relation::NONE;
        target.extent = Extent::NONE;
        target.reserved.fill(0U);
    }

    static void ResetTarget(Record& target, std::uint64_t publicationSequence,
        std::uint64_t currentTransitionPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.currentTransitionPublicationSequence = currentTransitionPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_TRANSITION_UNAVAILABLE;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE
        static void PopulatePair(const TransitionRecord& previous,
            const TransitionRecord& current, Record& target) noexcept
    {
        target.previousTransitionPublicationSequence = previous.publicationSequence;
        target.sharedQualificationPublicationSequence = current.previousQualificationPublicationSequence;
        target.sharedSummaryPublicationSequence = current.previousSummaryPublicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration = current.acceptedInputChainGeneration;
        target.firstProvenTransitionCount = previous.previousProvenTransitionCount;
        target.sharedProvenTransitionCount = current.previousProvenTransitionCount;
        target.latestProvenTransitionCount = current.currentProvenTransitionCount;
        target.firstCoordinateChangeAxisUnionMask = previous.previousCoordinateChangeAxisUnionMask;
        target.sharedCoordinateChangeAxisUnionMask = current.previousCoordinateChangeAxisUnionMask;
        target.latestCoordinateChangeAxisUnionMask = current.currentCoordinateChangeAxisUnionMask;
        target.sharedPendingReturnedStateAxisMask = current.previousPendingReturnedStateAxisMask;
        target.latestPendingReturnedStateAxisMask = current.currentPendingReturnedStateAxisMask;
        target.sharedPendingDirectReturnTransitionAxisMask = current.previousPendingDirectReturnTransitionAxisMask;
        target.latestPendingDirectReturnTransitionAxisMask = current.currentPendingDirectReturnTransitionAxisMask;
        target.sharedPendingBidirectionalTransitionAxisMask = current.previousPendingBidirectionalTransitionAxisMask;
        target.latestPendingBidirectionalTransitionAxisMask = current.currentPendingBidirectionalTransitionAxisMask;
        target.newlyPendingThenReturnedStateCoveredAxisMask = previous.newlyChangedAxisMask &
            current.removedPendingReturnedStateAxisMask;
        target.newlyPendingThenDirectReturnCoveredAxisMask = previous.newlyChangedAxisMask &
            current.removedPendingDirectReturnTransitionAxisMask;
        target.newlyPendingThenBidirectionalCoveredAxisMask = previous.newlyChangedAxisMask &
            current.removedPendingBidirectionalTransitionAxisMask;
        target.firstQualificationFlags = previous.previousQualificationFlags;
        target.sharedQualificationFlags = current.previousQualificationFlags;
        target.latestQualificationFlags = current.currentQualificationFlags;
        target.gainedThenRetainedQualificationFlags = static_cast<std::uint16_t>(
            previous.newlyGainedQualificationFlags & current.retainedQualificationFlags);
        target.gainedThenLostQualificationFlags = static_cast<std::uint16_t>(
            previous.newlyGainedQualificationFlags & current.lostQualificationFlags);
        target.lostThenRegainedQualificationFlags = static_cast<std::uint16_t>(
            previous.lostQualificationFlags & current.newlyGainedQualificationFlags);
        target.lostThenStillAbsentQualificationFlags =
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(
                previous.lostQualificationFlags, current.currentQualificationFlags);
        target.universeExpansionLostThenRegainedQualificationFlags = static_cast<std::uint16_t>(
            previous.universeExpansionLostQualificationFlags & current.newlyGainedQualificationFlags);
        target.universeExpansionLostThenStillAbsentQualificationFlags =
            NCPathCoreReturnQualificationTransitionPairDetail::Difference(
                previous.universeExpansionLostQualificationFlags, current.currentQualificationFlags);
        target.disposition = Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR;
        target.relation = Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR;
        target.extent = Extent::SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_RELATION_ONLY;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransitionPair())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_QUALIFICATION_TRANSITION_PAIR;
        }
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairDisposition) == 1U,
    "Qualification-transition pair disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRelation) == 1U,
    "Qualification-transition pair relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairExtent) == 1U,
    "Qualification-transition pair extent must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1>::value,
    "Qualification-transition pair record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1>::value,
    "Qualification-transition pair record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1) == 8U,
    "Qualification-transition pair record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1) == 152U,
    "Qualification-transition pair record must remain exactly 152 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1, firstProvenTransitionCount) == 64U,
    "Qualification-transition pair scalar block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1, schemaVersion) == 124U,
    "Qualification-transition pair flag block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1, reserved) == 147U,
    "Qualification-transition pair reserved block offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow>::value,
    "Qualification-transition pair observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow>::value,
    "Qualification-transition pair observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow) == 8U,
    "Qualification-transition pair observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow) == 320U,
    "Qualification-transition pair observer must remain exactly 320 bytes.");
