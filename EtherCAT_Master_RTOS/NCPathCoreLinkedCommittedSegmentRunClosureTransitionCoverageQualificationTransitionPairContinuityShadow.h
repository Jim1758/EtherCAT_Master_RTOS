#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2Q / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Pair Continuity Shadow.
//
// Compares directly adjacent proven L.2P pair records A,B,C and B,C,D.
// Their overlapping O transition identity must match; their shared N and M
// publication identities must advance directly.  Same coverage interval,
// run and accepted-input chain, both overlapping counts/universes/flags,
// and the three retained pending masks at C must agree exactly.
//
// L.2P does not retain pending masks at its FIRST qualification B.  This
// proves equality only of the RETAINED overlapping scalar projection;
// it does not prove complete overlapping O/N record or geometry equality.
// The result retains source identities and overlap counts only.  Flags,
// masks and the older shared N/M identities are checked during Observe,
// then discarded: the result cannot independently reconstruct that proof.
// Validation assumes trusted same-thread producer records.  It is not
// authentication against coherent forgery or durable replay detection.
//
// Continuity means only these two direct proven pair observations.  It is
// not a run-length, whole-run or eventual-recovery claim.  The underlying
// coverage interval may itself be a proven suffix of a committed run.
// A direct coverage mismatch differs from a canonical source-reported
// boundary; neither establishes its cause.  Neutral cause remains unknown.
// Invalid, unavailable, stale or mismatched inputs clear proven payload.
// Local freshness checks publication adjacency, not time.  Initial attach
// may accept any valid direct pair; null clears the local freshness anchor;
// a rejected publication is only a diagnostic resynchronization anchor.
//
// Fixed two-record scalar history, NCManager heap-owned.  No Observe-time
// allocation or large stack object; no coordinates/displacement/endpoints,
// segment/event list, complete event order or reverse-traversal data.
// Not Path/Motion Queue, planner, STARTED/DONE proof, actual-position
// history, B2 breadcrumb or execution.  Shadow-only, no control consumer:
// no Motion, G00, Gate/Registry, Alarm, PC, HMI/SHM/API, PDO, NIC, EtherCAT
// or DC changes; no logging, thread, timer, mutex, wait or sleep.

constexpr std::size_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionPairContinuityDetail
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

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_PAIR_UNAVAILABLE = 1U,
    NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE = 3U,
    NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN_CAUSE_UNKNOWN = 4U,
    NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY = 5U,
    INVALID_CURRENT_PAIR_RECORD = 6U,
    INVALID_PREVIOUS_PAIR_RECORD = 7U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 8U,
    INVALID_PAIR_ADVANCE_FENCE = 9U,
    INVALID_SHARED_TRANSITION_BINDING = 10U,
    INVALID_PAIR_CONTINUITY = 11U,
    PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY = 12U,
    NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY = 13U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRelation : std::uint8_t
{
    NONE = 0U,
    DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR_CONTINUITY = 1U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityExtent : std::uint8_t
{
    NONE = 0U,
    SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityExtent;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousPairPublicationSequence = 0ULL;
    std::uint64_t currentPairPublicationSequence = 0ULL;
    std::uint64_t sharedTransitionPublicationSequence = 0ULL;
    std::uint64_t latestSharedQualificationPublicationSequence = 0ULL;
    std::uint64_t latestSharedSummaryPublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;

    std::uint32_t sharedFirstProvenTransitionCount = 0U;
    std::uint32_t sharedLastProvenTransitionCount = 0U;

    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Extent extent = Extent::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuity() const noexcept
    {
        return publicationSequence != 0ULL && previousPairPublicationSequence != 0ULL &&
            currentPairPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previousPairPublicationSequence) &&
            sharedTransitionPublicationSequence != 0ULL &&
            latestSharedQualificationPublicationSequence != 0ULL &&
            latestSharedSummaryPublicationSequence != 0ULL &&
            coverageGeneration != 0ULL && runGeneration != 0ULL &&
            acceptedInputChainGeneration != 0ULL &&
            sharedFirstProvenTransitionCount > 1U &&
            sharedFirstProvenTransitionCount != (std::numeric_limits<std::uint32_t>::max)() &&
            sharedLastProvenTransitionCount == sharedFirstProvenTransitionCount + 1U &&
            sharedLastProvenTransitionCount != (std::numeric_limits<std::uint32_t>::max)() &&
            schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_SCHEMA_V1 &&
            disposition ==
            Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY &&
            relation == Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR_CONTINUITY &&
            extent == Extent::SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_ONLY &&
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::ReservedIsZero(reserved);
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityExtent;
    using PairRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRecordV1;
    using PairDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairDisposition;
    using PairRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairRelation;
    using PairExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE
        void ObserveImmediateQualificationTransitionPairContinuitySameThread(
            const PairRecord* previousPair, const PairRecord* currentPair) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const std::uint64_t previouslyObservedSourceSequence = previousObservation == nullptr
            ? 0ULL : previousObservation->currentPairPublicationSequence;
        m_publicationSequence =
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentPair == nullptr ? 0ULL : currentPair->publicationSequence);

        if (currentPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PAIR_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentPair->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else if (IsCanonicalNonProvenPair(*currentPair))
        {
            ClassifyNonProvenCurrent(previousPair, *currentPair, target);
        }
        else if (!IsFullyProvenPair(*currentPair))
        {
            target.disposition = Disposition::INVALID_CURRENT_PAIR_RECORD;
        }
        else if (previousPair == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_UNAVAILABLE;
        }
        else if (IsCanonicalNonProvenPair(*previousPair))
        {
            target.disposition = IsNeutralPairDisposition(previousPair->disposition)
                ? Disposition::NOT_APPLICABLE_PREVIOUS_PAIR_NOT_PROVEN_CAUSE_UNKNOWN
                : Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        }
        else if (!IsFullyProvenPair(*previousPair))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        }
        else if (currentPair->publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previousPair->publicationSequence))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE_FENCE;
        }
        else if (currentPair->coverageGeneration != previousPair->coverageGeneration)
        {
            target.disposition = Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY;
        }
        else if (currentPair->runGeneration != previousPair->runGeneration ||
            currentPair->acceptedInputChainGeneration != previousPair->acceptedInputChainGeneration ||
            !SharedSourceIdentitiesAdvanceDirectly(*previousPair, *currentPair))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE_FENCE;
        }
        else if (!SharedTransitionProjectionMatches(*previousPair, *currentPair))
        {
            target.disposition = Disposition::INVALID_SHARED_TRANSITION_BINDING;
        }
        else
        {
            PopulateContinuity(*previousPair, *currentPair, target);
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
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_HISTORY_CAPACITY;

    static bool IsNeutralPairDisposition(PairDisposition disposition) noexcept
    {
        return disposition == PairDisposition::NOT_APPLICABLE_TRANSITION_UNAVAILABLE ||
            disposition == PairDisposition::NOT_APPLICABLE_CURRENT_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == PairDisposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE ||
            disposition == PairDisposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_NOT_PROVEN_CAUSE_UNKNOWN ||
            disposition == PairDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            disposition == PairDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
    }

    static bool IsInvalidPairDisposition(PairDisposition disposition) noexcept
    {
        return disposition == PairDisposition::INVALID_CURRENT_TRANSITION_RECORD ||
            disposition == PairDisposition::INVALID_PREVIOUS_TRANSITION_RECORD ||
            disposition == PairDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            disposition == PairDisposition::INVALID_TRANSITION_ADVANCE_FENCE ||
            disposition == PairDisposition::INVALID_SHARED_QUALIFICATION_BINDING ||
            disposition == PairDisposition::INVALID_QUALIFICATION_TRANSITION_PAIR;
    }

    static bool IsFullyProvenPair(const PairRecord& source) noexcept
    {
        return source.IsProvenRunEndpointReturnCoverageQualificationTransitionPair();
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE
        static bool IsCanonicalNonProvenPair(const PairRecord& source) noexcept
    {
        const bool diagnosticIdentityIsValid = source.disposition ==
            PairDisposition::NOT_APPLICABLE_TRANSITION_UNAVAILABLE
            ? source.currentTransitionPublicationSequence == 0ULL
            : source.currentTransitionPublicationSequence != 0ULL ||
            source.disposition == PairDisposition::INVALID_CURRENT_TRANSITION_RECORD ||
            source.disposition == PairDisposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        return (IsNeutralPairDisposition(source.disposition) ||
            IsInvalidPairDisposition(source.disposition)) &&
            source.publicationSequence != 0ULL && diagnosticIdentityIsValid &&
            source.schemaVersion == NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_SCHEMA_V1 &&
            source.previousTransitionPublicationSequence == 0ULL &&
            source.sharedQualificationPublicationSequence == 0ULL &&
            source.sharedSummaryPublicationSequence == 0ULL &&
            source.coverageGeneration == 0ULL && source.runGeneration == 0ULL &&
            source.acceptedInputChainGeneration == 0ULL &&
            source.firstProvenTransitionCount == 0U &&
            source.sharedProvenTransitionCount == 0U &&
            source.latestProvenTransitionCount == 0U &&
            source.firstCoordinateChangeAxisUnionMask == 0U &&
            source.sharedCoordinateChangeAxisUnionMask == 0U &&
            source.latestCoordinateChangeAxisUnionMask == 0U &&
            source.sharedPendingReturnedStateAxisMask == 0U &&
            source.latestPendingReturnedStateAxisMask == 0U &&
            source.sharedPendingDirectReturnTransitionAxisMask == 0U &&
            source.latestPendingDirectReturnTransitionAxisMask == 0U &&
            source.sharedPendingBidirectionalTransitionAxisMask == 0U &&
            source.latestPendingBidirectionalTransitionAxisMask == 0U &&
            source.newlyPendingThenReturnedStateCoveredAxisMask == 0U &&
            source.newlyPendingThenDirectReturnCoveredAxisMask == 0U &&
            source.newlyPendingThenBidirectionalCoveredAxisMask == 0U &&
            source.firstQualificationFlags == 0U &&
            source.sharedQualificationFlags == 0U &&
            source.latestQualificationFlags == 0U &&
            source.gainedThenRetainedQualificationFlags == 0U &&
            source.gainedThenLostQualificationFlags == 0U &&
            source.lostThenRegainedQualificationFlags == 0U &&
            source.lostThenStillAbsentQualificationFlags == 0U &&
            source.universeExpansionLostThenRegainedQualificationFlags == 0U &&
            source.universeExpansionLostThenStillAbsentQualificationFlags == 0U &&
            source.relation == PairRelation::NONE && source.extent == PairExtent::NONE &&
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::ReservedIsZero(source.reserved);
    }

    static bool IsSourceReportedBoundary(PairDisposition disposition) noexcept
    {
        return disposition == PairDisposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY ||
            disposition == PairDisposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
    }

    static void ClassifyNonProvenCurrent(const PairRecord* previous,
        const PairRecord& current, Record& target) noexcept
    {
        if (!IsNeutralPairDisposition(current.disposition))
        {
            target.disposition = Disposition::INVALID_CURRENT_PAIR_RECORD;
        }
        else if (!IsSourceReportedBoundary(current.disposition) || previous == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_CURRENT_PAIR_NOT_PROVEN_CAUSE_UNKNOWN;
        }
        else if (!IsFullyProvenPair(*previous) && !IsCanonicalNonProvenPair(*previous))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        }
        else if (current.publicationSequence !=
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previous->publicationSequence))
        {
            target.disposition = Disposition::INVALID_PAIR_ADVANCE_FENCE;
        }
        else
        {
            target.disposition = Disposition::NOT_APPLICABLE_SOURCE_REPORTED_COVERAGE_INTERVAL_BOUNDARY;
        }
    }

    static bool SharedSourceIdentitiesAdvanceDirectly(const PairRecord& previous,
        const PairRecord& current) noexcept
    {
        return current.sharedQualificationPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previous.sharedQualificationPublicationSequence) &&
            current.sharedSummaryPublicationSequence ==
            NCPathCoreReturnQualificationTransitionPairContinuityDetail::NextNonZeroSequence(
                previous.sharedSummaryPublicationSequence);
    }

    static bool SharedTransitionProjectionMatches(const PairRecord& previous,
        const PairRecord& current) noexcept
    {
        return previous.currentTransitionPublicationSequence == current.previousTransitionPublicationSequence &&
            previous.sharedProvenTransitionCount == current.firstProvenTransitionCount &&
            previous.latestProvenTransitionCount == current.sharedProvenTransitionCount &&
            previous.sharedCoordinateChangeAxisUnionMask == current.firstCoordinateChangeAxisUnionMask &&
            previous.latestCoordinateChangeAxisUnionMask == current.sharedCoordinateChangeAxisUnionMask &&
            previous.sharedQualificationFlags == current.firstQualificationFlags &&
            previous.latestQualificationFlags == current.sharedQualificationFlags &&
            previous.latestPendingReturnedStateAxisMask == current.sharedPendingReturnedStateAxisMask &&
            previous.latestPendingDirectReturnTransitionAxisMask ==
            current.sharedPendingDirectReturnTransitionAxisMask &&
            previous.latestPendingBidirectionalTransitionAxisMask ==
            current.sharedPendingBidirectionalTransitionAxisMask;
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.previousPairPublicationSequence = 0ULL;
        target.sharedTransitionPublicationSequence = 0ULL;
        target.latestSharedQualificationPublicationSequence = 0ULL;
        target.latestSharedSummaryPublicationSequence = 0ULL;
        target.coverageGeneration = 0ULL;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.sharedFirstProvenTransitionCount = 0U;
        target.sharedLastProvenTransitionCount = 0U;
        target.relation = Relation::NONE;
        target.extent = Extent::NONE;
        target.reserved.fill(0U);
    }

    static void ResetTarget(Record& target, std::uint64_t publicationSequence,
        std::uint64_t currentPairPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.currentPairPublicationSequence = currentPairPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion = NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_PAIR_UNAVAILABLE;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE
        static void PopulateContinuity(const PairRecord& previous,
            const PairRecord& current, Record& target) noexcept
    {
        target.previousPairPublicationSequence = previous.publicationSequence;
        target.sharedTransitionPublicationSequence = current.previousTransitionPublicationSequence;
        target.latestSharedQualificationPublicationSequence = current.sharedQualificationPublicationSequence;
        target.latestSharedSummaryPublicationSequence = current.sharedSummaryPublicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration = current.acceptedInputChainGeneration;
        target.sharedFirstProvenTransitionCount = current.firstProvenTransitionCount;
        target.sharedLastProvenTransitionCount = current.sharedProvenTransitionCount;
        target.disposition =
            Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY;
        target.relation = Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_TRANSITION_PAIR_CONTINUITY;
        target.extent = Extent::SCALAR_IMMEDIATE_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_ONLY;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransitionPairContinuity())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_PAIR_CONTINUITY;
        }
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_PAIR_CONTINUITY_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityDisposition) == 1U,
    "Qualification-transition pair continuity disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRelation) == 1U,
    "Qualification-transition pair continuity relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityExtent) == 1U,
    "Qualification-transition pair continuity extent must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1>::value,
    "Qualification-transition pair continuity record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1>::value,
    "Qualification-transition pair continuity record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1) == 8U,
    "Qualification-transition pair continuity record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1) == 88U,
    "Qualification-transition pair continuity record must remain exactly 88 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1, sharedFirstProvenTransitionCount) == 72U,
    "Qualification-transition pair continuity count block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1, schemaVersion) == 80U,
    "Qualification-transition pair continuity schema offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRecordV1, reserved) == 85U,
    "Qualification-transition pair continuity reserved block offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow>::value,
    "Qualification-transition pair continuity observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow>::value,
    "Qualification-transition pair continuity observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow) == 8U,
    "Qualification-transition pair continuity observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow) == 192U,
    "Qualification-transition pair continuity observer must remain exactly 192 bytes.");

