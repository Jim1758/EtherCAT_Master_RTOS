#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2O / Proven Linked Committed Segment Run Endpoint-Return
// Coverage Qualification Transition Shadow.
//
// Compares exactly two adjacent, fully proven L.2N qualifications from the
// same L.2M coverage interval.  It retains only scalar source identities,
// qualification flags and pending-mask differences.  A lost all-axis flag
// is attributed to newly changed axes only after the exact source fence
// proves retained coverage of the previous axis universe.  "Lost" describes
// this pair; it does not promise that the qualification will return later.
// FULL_CLOSURE_CURRENT loss is a state exit, never an expansion-cause claim.
//
// A source STARTED record establishes a comparison baseline only.  Different
// proven coverage generations identify different intervals, not the cause
// of their boundary.  A non-proven N source can hide an earlier invalid M
// source: its reason is unknown here.  No loss is reported across any of
// these cases.  Repeated/nonadjacent observed source publications fail closed.
// A first attachment can accept any fully validated direct source pair; a
// null source clears the observation freshness baseline.  A rejected source
// publication becomes only a diagnostic resynchronization baseline, so the
// next direct source pair can be checked.  This is structural producer-stream
// freshness, not time freshness or cryptographic replay authentication.
//
// Fixed two-record history, owned on the NCManager heap; no allocation in
// this observer.  No coordinates, displacements, endpoint arrays, event list,
// segment list, ordered history, or reverse-traversal data.  This remains
// shadow-only and is not Path/Motion Queue, planner, STARTED/DONE proof,
// actual-position history or B2 breadcrumb/execution.  No Gate, PC, Alarm,
// Motion, HMI/SHM/API, PDO, NIC, EtherCAT or DC consumer; no logging, thread,
// timer, mutex, wait or sleep.  Non-proven results clear all proven payload.

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_SCHEMA_V1 = 1U;

namespace NCPathCoreReturnQualificationTransitionDetail
{
    constexpr std::uint64_t NextNonZeroSequence(std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : value + 1ULL;
    }

    constexpr std::uint32_t SaturatingIncrement(std::uint32_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint32_t>::max)()
            ? value : value + 1U;
    }

    constexpr std::uint16_t Difference(
        std::uint16_t left, std::uint16_t right) noexcept
    {
        return static_cast<std::uint16_t>(
            static_cast<std::uint32_t>(left) &
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
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_QUALIFICATION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN = 2U,
    NOT_APPLICABLE_QUALIFICATION_BASELINE = 3U,
    NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY = 4U,
    NOT_APPLICABLE_PREVIOUS_QUALIFICATION_UNAVAILABLE = 5U,
    NOT_APPLICABLE_PREVIOUS_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN = 6U,
    INVALID_CURRENT_QUALIFICATION_RECORD = 7U,
    INVALID_PREVIOUS_QUALIFICATION_RECORD = 8U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 9U,
    INVALID_QUALIFICATION_ADVANCE_FENCE = 10U,
    INVALID_QUALIFICATION_TRANSITION = 11U,
    PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION = 12U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRelation : std::uint8_t
{
    NONE = 0U,
    DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_PAIR = 1U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionExtent : std::uint8_t
{
    NONE = 0U,
    SCALAR_QUALIFICATION_FLAGS_AND_PENDING_MASK_DIFFERENCES_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1
{
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionExtent;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousQualificationPublicationSequence = 0ULL;
    std::uint64_t currentQualificationPublicationSequence = 0ULL;
    std::uint64_t previousSummaryPublicationSequence = 0ULL;
    std::uint64_t currentSummaryPublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;

    std::uint32_t previousProvenTransitionCount = 0U;
    std::uint32_t currentProvenTransitionCount = 0U;
    std::uint32_t previousCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t currentCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t newlyChangedAxisMask = 0U;
    std::uint32_t previousPendingReturnedStateAxisMask = 0U;
    std::uint32_t currentPendingReturnedStateAxisMask = 0U;
    std::uint32_t previousPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t currentPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t previousPendingBidirectionalTransitionAxisMask = 0U;
    std::uint32_t currentPendingBidirectionalTransitionAxisMask = 0U;
    std::uint32_t addedPendingReturnedStateAxisMask = 0U;
    std::uint32_t removedPendingReturnedStateAxisMask = 0U;
    std::uint32_t addedPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t removedPendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t addedPendingBidirectionalTransitionAxisMask = 0U;
    std::uint32_t removedPendingBidirectionalTransitionAxisMask = 0U;

    std::uint16_t schemaVersion = 0U;
    std::uint16_t previousQualificationFlags = 0U;
    std::uint16_t currentQualificationFlags = 0U;
    std::uint16_t newlyGainedQualificationFlags = 0U;
    std::uint16_t retainedQualificationFlags = 0U;
    std::uint16_t lostQualificationFlags = 0U;
    std::uint16_t universeExpansionLostQualificationFlags = 0U;
    Disposition disposition = Disposition::EMPTY;
    Relation relation = Relation::NONE;
    Extent extent = Extent::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualificationTransition() const noexcept
    {
        return publicationSequence != 0ULL &&
            previousQualificationPublicationSequence != 0ULL &&
            currentQualificationPublicationSequence ==
            NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(previousQualificationPublicationSequence) &&
            previousSummaryPublicationSequence != 0ULL &&
            currentSummaryPublicationSequence ==
            NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(previousSummaryPublicationSequence) &&
            coverageGeneration != 0ULL && runGeneration != 0ULL &&
            acceptedInputChainGeneration != 0ULL &&
            previousProvenTransitionCount != 0U &&
            previousProvenTransitionCount !=
            (std::numeric_limits<std::uint32_t>::max)() &&
            currentProvenTransitionCount == previousProvenTransitionCount + 1U &&
            schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_SCHEMA_V1 &&
            disposition == Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION &&
            relation == Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_PAIR &&
            extent == Extent::SCALAR_QUALIFICATION_FLAGS_AND_PENDING_MASK_DIFFERENCES_ONLY &&
            AxisAndPendingMasksAreValid() && QualificationFlagsAreValid() &&
            NCPathCoreReturnQualificationTransitionDetail::ReservedIsZero(reserved);
    }

private:
    static bool PendingPairIsValid(
        std::uint32_t previous, std::uint32_t current,
        std::uint32_t added, std::uint32_t removed,
        std::uint32_t previousUniverse, std::uint32_t currentUniverse,
        std::uint32_t newAxes) noexcept
    {
        return (previous & ~previousUniverse) == 0U &&
            (current & ~currentUniverse) == 0U &&
            added == (current & ~previous) &&
            removed == (previous & ~current) &&
            added == newAxes && (removed & ~previousUniverse) == 0U;
    }

    bool AxisAndPendingMasksAreValid() const noexcept
    {
        return (currentCoordinateChangeAxisUnionMask & ~NCPathCoreReturnQualificationTransitionDetail::AXIS_MASK) == 0U &&
            (previousCoordinateChangeAxisUnionMask &
                ~currentCoordinateChangeAxisUnionMask) == 0U &&
            newlyChangedAxisMask == (currentCoordinateChangeAxisUnionMask &
                ~previousCoordinateChangeAxisUnionMask) &&
            PendingPairIsValid(previousPendingReturnedStateAxisMask,
                currentPendingReturnedStateAxisMask, addedPendingReturnedStateAxisMask,
                removedPendingReturnedStateAxisMask, previousCoordinateChangeAxisUnionMask,
                currentCoordinateChangeAxisUnionMask, newlyChangedAxisMask) &&
            PendingPairIsValid(previousPendingDirectReturnTransitionAxisMask,
                currentPendingDirectReturnTransitionAxisMask, addedPendingDirectReturnTransitionAxisMask,
                removedPendingDirectReturnTransitionAxisMask, previousCoordinateChangeAxisUnionMask,
                currentCoordinateChangeAxisUnionMask, newlyChangedAxisMask) &&
            PendingPairIsValid(previousPendingBidirectionalTransitionAxisMask,
                currentPendingBidirectionalTransitionAxisMask, addedPendingBidirectionalTransitionAxisMask,
                removedPendingBidirectionalTransitionAxisMask, previousCoordinateChangeAxisUnionMask,
                currentCoordinateChangeAxisUnionMask, newlyChangedAxisMask) &&
            (previousPendingReturnedStateAxisMask & ~previousPendingDirectReturnTransitionAxisMask) == 0U &&
            (currentPendingReturnedStateAxisMask & ~currentPendingDirectReturnTransitionAxisMask) == 0U &&
            (previousPendingDirectReturnTransitionAxisMask & ~previousPendingBidirectionalTransitionAxisMask) == 0U &&
            (currentPendingDirectReturnTransitionAxisMask & ~currentPendingBidirectionalTransitionAxisMask) == 0U;
    }

    static bool SourceFlagShapeIsValid(std::uint16_t flags,
        std::uint32_t universe) noexcept
    {
        const bool returned =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED) != 0U;
        const bool both =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED) != 0U;
        const bool directReturn =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED) != 0U;
        const bool directReopen =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED) != 0U;
        const bool bidirectional =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED) != 0U;
        const bool fullObserved =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED) != 0U;
        const bool fullCurrent =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT) != 0U;
        const bool fullExit =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED) != 0U;
        const bool fullReentry =
            (flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED) != 0U;
        return (universe != 0U || flags == 0U) &&
            (!both || returned) && (!directReturn || both) && (!directReopen || both) &&
            (bidirectional == (directReturn && directReopen)) &&
            (!fullCurrent || (fullObserved && returned)) &&
            (!fullExit || fullObserved) && (!fullReentry || (fullObserved && fullExit));
    }

    static bool PendingFlagsMatch(std::uint16_t flags, std::uint32_t universe,
        std::uint32_t pendingReturned, std::uint32_t pendingDirect,
        std::uint32_t pendingBidirectional) noexcept
    {
        return SourceFlagShapeIsValid(flags, universe) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && pendingReturned == 0U) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && pendingDirect == 0U) &&
            ((flags & NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED) != 0U) ==
            (universe != 0U && pendingBidirectional == 0U) &&
            (universe != 0U || (flags & NCPathCoreReturnQualificationTransitionDetail::ALL_AXIS_FLAGS) == 0U);
    }

    bool QualificationFlagsAreValid() const noexcept
    {
        const std::uint16_t expectedLost =
            NCPathCoreReturnQualificationTransitionDetail::Difference(previousQualificationFlags, currentQualificationFlags);
        const std::uint16_t expectedExpansionLost = static_cast<std::uint16_t>(
            expectedLost & NCPathCoreReturnQualificationTransitionDetail::ALL_AXIS_FLAGS);
        return ((previousQualificationFlags | currentQualificationFlags) &
            ~NCPathCoreReturnQualificationTransitionDetail::ALL_FLAGS) == 0U &&
            newlyGainedQualificationFlags ==
            NCPathCoreReturnQualificationTransitionDetail::Difference(currentQualificationFlags, previousQualificationFlags) &&
            retainedQualificationFlags == static_cast<std::uint16_t>(
                previousQualificationFlags & currentQualificationFlags) &&
            lostQualificationFlags == expectedLost &&
            (lostQualificationFlags & NCPathCoreReturnQualificationTransitionDetail::HISTORICAL_FULL_FLAGS) == 0U &&
            universeExpansionLostQualificationFlags == expectedExpansionLost &&
            (expectedExpansionLost == 0U || newlyChangedAxisMask != 0U) &&
            PendingFlagsMatch(previousQualificationFlags, previousCoordinateChangeAxisUnionMask,
                previousPendingReturnedStateAxisMask, previousPendingDirectReturnTransitionAxisMask,
                previousPendingBidirectionalTransitionAxisMask) &&
            PendingFlagsMatch(currentQualificationFlags, currentCoordinateChangeAxisUnionMask,
                currentPendingReturnedStateAxisMask, currentPendingDirectReturnTransitionAxisMask,
                currentPendingBidirectionalTransitionAxisMask);
    }
};

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow final
{
public:
    using Record = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1;
    using Disposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionDisposition;
    using Relation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRelation;
    using Extent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionExtent;
    using QualificationRecord = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1;
    using QualificationDisposition = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition;
    using QualificationRelation = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation;
    using QualificationExtent = NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent;

    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow() noexcept = default;

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        void ObserveLatestQualificationSameThread(
            const QualificationRecord* previousQualification,
            const QualificationRecord* currentQualification) noexcept
    {
        const Record* const previousObservation = GetNewestObservationSameThread();
        const std::uint64_t previouslyObservedSourceSequence =
            previousObservation == nullptr ? 0ULL :
            previousObservation->currentQualificationPublicationSequence;
        m_publicationSequence = NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[targetIndex];
        ResetTarget(target, m_publicationSequence,
            currentQualification == nullptr ? 0ULL : currentQualification->publicationSequence);

        if (currentQualification == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_QUALIFICATION_UNAVAILABLE;
        }
        else if (previouslyObservedSourceSequence != 0ULL &&
            currentQualification->publicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(previouslyObservedSourceSequence))
        {
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        }
        else if (IsStructurallyValidNonProvenQualification(*currentQualification))
        {
            target.disposition = IsNeutralQualification(*currentQualification)
                ? Disposition::NOT_APPLICABLE_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN
                : Disposition::INVALID_CURRENT_QUALIFICATION_RECORD;
        }
        else if (!IsFullyProvenQualification(*currentQualification))
        {
            target.disposition = Disposition::INVALID_CURRENT_QUALIFICATION_RECORD;
        }
        else if (previousQualification != nullptr &&
            IsFullyProvenQualification(*previousQualification) &&
            currentQualification->coverageGeneration != previousQualification->coverageGeneration)
        {
            target.disposition = currentQualification->publicationSequence ==
                NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(previousQualification->publicationSequence)
                ? Disposition::NOT_APPLICABLE_COVERAGE_INTERVAL_BOUNDARY
                : Disposition::INVALID_QUALIFICATION_ADVANCE_FENCE;
        }
        else if (currentQualification->disposition ==
            QualificationDisposition::PROVEN_RETURN_COVERAGE_QUALIFICATION_STARTED)
        {
            target.disposition = Disposition::NOT_APPLICABLE_QUALIFICATION_BASELINE;
        }
        else if (previousQualification == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_QUALIFICATION_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenQualification(*previousQualification))
        {
            target.disposition = IsNeutralQualification(*previousQualification)
                ? Disposition::NOT_APPLICABLE_PREVIOUS_QUALIFICATION_NOT_PROVEN_CAUSE_UNKNOWN
                : Disposition::INVALID_PREVIOUS_QUALIFICATION_RECORD;
        }
        else if (!IsFullyProvenQualification(*previousQualification))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_QUALIFICATION_RECORD;
        }
        else if (!IsDirectQualificationExtensionFenceValid(
            *previousQualification, *currentQualification) ||
            !HistoricalQualificationsAdvanceMonotonically(*previousQualification, *currentQualification))
        {
            target.disposition = Disposition::INVALID_QUALIFICATION_ADVANCE_FENCE;
        }
        else
        {
            PopulateTransition(*previousQualification, *currentQualification, target);
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
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_HISTORY_CAPACITY;

    static bool IsNeutralQualification(const QualificationRecord& source) noexcept
    {
        return source.disposition == QualificationDisposition::NOT_APPLICABLE_SUMMARY_UNAVAILABLE ||
            source.disposition == QualificationDisposition::NOT_APPLICABLE_SUMMARY_NOT_PROVEN;
    }

    static void ClearProvenPayload(Record& target) noexcept
    {
        target.previousQualificationPublicationSequence = 0ULL;
        target.previousSummaryPublicationSequence = 0ULL;
        target.currentSummaryPublicationSequence = 0ULL;
        target.coverageGeneration = 0ULL;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.previousProvenTransitionCount = 0U;
        target.currentProvenTransitionCount = 0U;
        target.previousCoordinateChangeAxisUnionMask = 0U;
        target.currentCoordinateChangeAxisUnionMask = 0U;
        target.newlyChangedAxisMask = 0U;
        target.previousPendingReturnedStateAxisMask = 0U;
        target.currentPendingReturnedStateAxisMask = 0U;
        target.previousPendingDirectReturnTransitionAxisMask = 0U;
        target.currentPendingDirectReturnTransitionAxisMask = 0U;
        target.previousPendingBidirectionalTransitionAxisMask = 0U;
        target.currentPendingBidirectionalTransitionAxisMask = 0U;
        target.addedPendingReturnedStateAxisMask = 0U;
        target.removedPendingReturnedStateAxisMask = 0U;
        target.addedPendingDirectReturnTransitionAxisMask = 0U;
        target.removedPendingDirectReturnTransitionAxisMask = 0U;
        target.addedPendingBidirectionalTransitionAxisMask = 0U;
        target.removedPendingBidirectionalTransitionAxisMask = 0U;
        target.previousQualificationFlags = 0U;
        target.currentQualificationFlags = 0U;
        target.newlyGainedQualificationFlags = 0U;
        target.retainedQualificationFlags = 0U;
        target.lostQualificationFlags = 0U;
        target.universeExpansionLostQualificationFlags = 0U;
        target.relation = Relation::NONE;
        target.extent = Extent::NONE;
        target.reserved.fill(0U);
    }

    static void ResetTarget(Record& target, std::uint64_t publicationSequence,
        std::uint64_t currentQualificationPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.currentQualificationPublicationSequence = currentQualificationPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_TRANSITION_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_QUALIFICATION_UNAVAILABLE;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        static bool IsStructurallyValidNonProvenQualification(
            const QualificationRecord&
            summary) noexcept
    {
        const bool knownDisposition =
            summary.disposition ==
            QualificationDisposition::
            NOT_APPLICABLE_SUMMARY_UNAVAILABLE ||
            summary.disposition ==
            QualificationDisposition::
            NOT_APPLICABLE_SUMMARY_NOT_PROVEN ||
            summary.disposition ==
            QualificationDisposition::
            INVALID_CURRENT_SUMMARY_RECORD ||
            summary.disposition ==
            QualificationDisposition::
            INVALID_PREVIOUS_SUMMARY_RECORD ||
            summary.disposition ==
            QualificationDisposition::
            INVALID_SUMMARY_ADVANCE_FENCE ||
            summary.disposition ==
            QualificationDisposition::
            INVALID_COVERAGE_QUALIFICATION;

        return knownDisposition &&
            summary.publicationSequence != 0ULL &&
            summary.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_SCHEMA_V1 &&
            SourceDiagnosticIdentityIsValid(summary) &&
            summary.previousSummaryPublicationSequence == 0ULL &&
            summary.coverageGeneration == 0ULL &&
            summary.runGeneration == 0ULL &&
            summary.acceptedInputChainGeneration == 0ULL &&
            summary.firstTransitionPublicationSequence == 0ULL &&
            summary.latestTransitionPublicationSequence == 0ULL &&
            summary.firstClosurePublicationSequence == 0ULL &&
            summary.latestClosurePublicationSequence == 0ULL &&
            summary.firstSegmentPublicationSequence == 0ULL &&
            summary.latestSegmentPublicationSequence == 0ULL &&
            summary.firstLinkPublicationSequence == 0ULL &&
            summary.latestLinkPublicationSequence == 0ULL &&
            summary.runStartGeometryPublicationSequence == 0ULL &&
            summary.runEndGeometryPublicationSequence == 0ULL &&
            summary.runStartGeometryMotionSegmentId == 0ULL &&
            summary.runEndGeometryMotionSegmentId == 0ULL &&
            summary.provenTransitionCount == 0U &&
            summary.coverageStartLinkedSegmentRunLength == 0U &&
            summary.latestLinkedSegmentRunLength == 0U &&
            summary.participatingAxisUnionMask == 0U &&
            summary.coordinateChangeAxisUnionMask == 0U &&
            summary.cumulativeNewlyOpenedAxisMask == 0U &&
            summary.cumulativeNewlyReturnedAxisMask == 0U &&
            summary.cumulativeReopenedAxisMask == 0U &&
            summary.cumulativeRetainedOpenAxisMask == 0U &&
            summary.cumulativeRetainedReturnedAxisMask == 0U &&
            summary.coverageStartEndpointOpenAxisMask == 0U &&
            summary.coverageStartEndpointReturnedAxisMask == 0U &&
            summary.latestEndpointOpenAxisMask == 0U &&
            summary.latestEndpointReturnedAxisMask == 0U &&
            summary.fullClosureEntryCount == 0U &&
            summary.fullClosureExitCount == 0U &&
            summary.fullClosureRetentionCount == 0U &&
            summary.observedOpenStateAxisMask == 0U &&
            summary.observedReturnedStateAxisMask == 0U &&
            summary.observedBothEndpointStatesAxisMask == 0U &&
            summary.directReturnTransitionAxisMask == 0U &&
            summary.directReopenTransitionAxisMask == 0U &&
            summary.bidirectionalTransitionAxisMask == 0U &&
            summary.pendingReturnedStateAxisMask == 0U &&
            summary.pendingDirectReturnTransitionAxisMask == 0U &&
            summary.pendingBidirectionalTransitionAxisMask == 0U &&
            summary.qualificationFlags == 0U &&
            summary.firstSourcePC == -1 &&
            summary.coverageStartPreviousLatestSourcePC == -1 &&
            summary.latestSourcePC == -1 &&
            summary.relation ==
            QualificationRelation::
            NONE &&
            summary.extent ==
            QualificationExtent::
            NONE &&
            summary.firstClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            summary.latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            summary.frame == NCPathCoreCommittedGeometryFrame::NONE &&
            summary.kind == NCPathCoreCommittedGeometryKind::NONE &&
            summary.commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE &&
            NCPathCoreReturnQualificationTransitionDetail::ReservedIsZero(summary.reserved);
    }


    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        static bool IsDirectQualificationExtensionFenceValid(
            const QualificationRecord&
            previous,
            const QualificationRecord&
            current) noexcept
    {
        if (current.disposition !=
            QualificationDisposition::
            PROVEN_RETURN_COVERAGE_QUALIFICATION_EXTENDED ||
            current.publicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.publicationSequence) ||
            current.previousSummaryPublicationSequence !=
            previous.currentSummaryPublicationSequence ||
            current.currentSummaryPublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::NextNonZeroSequence(previous.currentSummaryPublicationSequence) ||
            current.coverageGeneration != previous.coverageGeneration ||
            current.runGeneration != previous.runGeneration ||
            current.acceptedInputChainGeneration !=
            previous.acceptedInputChainGeneration ||
            current.firstTransitionPublicationSequence !=
            previous.firstTransitionPublicationSequence ||
            current.latestTransitionPublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.latestTransitionPublicationSequence) ||
            current.firstClosurePublicationSequence !=
            previous.firstClosurePublicationSequence ||
            current.latestClosurePublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.latestClosurePublicationSequence) ||
            current.firstSegmentPublicationSequence !=
            previous.firstSegmentPublicationSequence ||
            current.latestSegmentPublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.latestSegmentPublicationSequence) ||
            current.firstLinkPublicationSequence !=
            previous.firstLinkPublicationSequence ||
            current.latestLinkPublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.latestLinkPublicationSequence) ||
            current.runStartGeometryPublicationSequence !=
            previous.runStartGeometryPublicationSequence ||
            current.runEndGeometryPublicationSequence !=
            NCPathCoreReturnQualificationTransitionDetail::
            NextNonZeroSequence(previous.runEndGeometryPublicationSequence) ||
            current.runStartGeometryMotionSegmentId !=
            previous.runStartGeometryMotionSegmentId ||
            current.runEndGeometryMotionSegmentId <=
            previous.runEndGeometryMotionSegmentId ||
            previous.provenTransitionCount ==
            (std::numeric_limits<std::uint32_t>::max)() ||
            current.provenTransitionCount !=
            NCPathCoreReturnQualificationTransitionDetail::
            SaturatingIncrement(previous.provenTransitionCount) ||
            current.coverageStartLinkedSegmentRunLength !=
            previous.coverageStartLinkedSegmentRunLength ||
            previous.latestLinkedSegmentRunLength ==
            (std::numeric_limits<std::uint32_t>::max)() ||
            current.latestLinkedSegmentRunLength !=
            NCPathCoreReturnQualificationTransitionDetail::
            SaturatingIncrement(previous.latestLinkedSegmentRunLength) ||
            (previous.participatingAxisUnionMask &
                ~current.participatingAxisUnionMask) != 0U ||
            (previous.coordinateChangeAxisUnionMask &
                ~current.coordinateChangeAxisUnionMask) != 0U ||
            !CoverageEventMasksAdvanceExactly(previous, current) ||
            current.coverageStartEndpointOpenAxisMask !=
            previous.coverageStartEndpointOpenAxisMask ||
            current.coverageStartEndpointReturnedAxisMask !=
            previous.coverageStartEndpointReturnedAxisMask ||
            !FullClosureCountsAdvanceExactly(previous, current) ||
            current.firstSourcePC != previous.firstSourcePC ||
            current.coverageStartPreviousLatestSourcePC !=
            previous.coverageStartPreviousLatestSourcePC ||
            previous.latestSourcePC ==
            (std::numeric_limits<std::int32_t>::max)() ||
            current.latestSourcePC != previous.latestSourcePC + 1 ||
            current.firstClosureClass != previous.firstClosureClass ||
            current.frame != previous.frame ||
            current.kind != previous.kind ||
            current.commandPathPolicy != previous.commandPathPolicy)
        {
            return false;
        }

        return true;
    }

    static bool CoverageEventMasksAdvanceExactly(
        const QualificationRecord&
        previous,
        const QualificationRecord&
        current) noexcept
    {
        const std::uint32_t newlyOpenedAxisMask =
            current.latestEndpointOpenAxisMask &
            ~previous.coordinateChangeAxisUnionMask;
        const std::uint32_t newlyReturnedAxisMask =
            current.latestEndpointReturnedAxisMask &
            previous.latestEndpointOpenAxisMask;
        const std::uint32_t reopenedAxisMask =
            current.latestEndpointOpenAxisMask &
            previous.latestEndpointReturnedAxisMask;
        const std::uint32_t retainedOpenAxisMask =
            current.latestEndpointOpenAxisMask &
            previous.latestEndpointOpenAxisMask;
        const std::uint32_t retainedReturnedAxisMask =
            current.latestEndpointReturnedAxisMask &
            previous.latestEndpointReturnedAxisMask;

        return
            current.coordinateChangeAxisUnionMask ==
            (previous.coordinateChangeAxisUnionMask | newlyOpenedAxisMask) &&
            current.cumulativeNewlyOpenedAxisMask ==
            (previous.cumulativeNewlyOpenedAxisMask | newlyOpenedAxisMask) &&
            current.cumulativeNewlyReturnedAxisMask ==
            (previous.cumulativeNewlyReturnedAxisMask |
                newlyReturnedAxisMask) &&
            current.cumulativeReopenedAxisMask ==
            (previous.cumulativeReopenedAxisMask | reopenedAxisMask) &&
            current.cumulativeRetainedOpenAxisMask ==
            (previous.cumulativeRetainedOpenAxisMask | retainedOpenAxisMask) &&
            current.cumulativeRetainedReturnedAxisMask ==
            (previous.cumulativeRetainedReturnedAxisMask |
                retainedReturnedAxisMask);
    }

    static bool FullClosureCountsAdvanceExactly(
        const QualificationRecord&
        previous,
        const QualificationRecord&
        current) noexcept
    {
        const bool previousFull =
            previous.latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool currentFull =
            current.latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const std::uint32_t expectedEntryCount =
            !previousFull && currentFull
            ? NCPathCoreReturnQualificationTransitionDetail::
            SaturatingIncrement(previous.fullClosureEntryCount)
            : previous.fullClosureEntryCount;
        const std::uint32_t expectedExitCount =
            previousFull && !currentFull
            ? NCPathCoreReturnQualificationTransitionDetail::
            SaturatingIncrement(previous.fullClosureExitCount)
            : previous.fullClosureExitCount;
        const std::uint32_t expectedRetentionCount =
            previousFull && currentFull
            ? NCPathCoreReturnQualificationTransitionDetail::
            SaturatingIncrement(previous.fullClosureRetentionCount)
            : previous.fullClosureRetentionCount;
        return current.fullClosureEntryCount == expectedEntryCount &&
            current.fullClosureExitCount == expectedExitCount &&
            current.fullClosureRetentionCount == expectedRetentionCount;
    }


    static bool SourceDiagnosticIdentityIsValid(const QualificationRecord& source) noexcept
    {
        if (source.disposition == QualificationDisposition::NOT_APPLICABLE_SUMMARY_UNAVAILABLE)
        {
            return source.currentSummaryPublicationSequence == 0ULL;
        }
        return source.disposition == QualificationDisposition::INVALID_CURRENT_SUMMARY_RECORD ||
            source.currentSummaryPublicationSequence != 0ULL;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        static bool IsFullyProvenQualification(const QualificationRecord& source) noexcept
    {
        if (!source.IsProvenRunEndpointReturnCoverageQualification() ||
            ((source.directReturnTransitionAxisMask | source.directReopenTransitionAxisMask) &
                ~source.observedBothEndpointStatesAxisMask) != 0U)
        {
            return false;
        }
        const std::uint32_t startUniverse = source.coverageStartEndpointOpenAxisMask |
            source.coverageStartEndpointReturnedAxisMask;
        if (source.disposition == QualificationDisposition::PROVEN_RETURN_COVERAGE_QUALIFICATION_STARTED)
        {
            const bool firstFull = source.firstClosureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
            const bool currentFull = source.latestClosureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
            return (source.latestEndpointReturnedAxisMask & ~startUniverse) == 0U &&
                source.cumulativeNewlyOpenedAxisMask ==
                (source.latestEndpointOpenAxisMask & ~startUniverse) &&
                source.cumulativeNewlyReturnedAxisMask ==
                (source.latestEndpointReturnedAxisMask & source.coverageStartEndpointOpenAxisMask) &&
                source.cumulativeReopenedAxisMask ==
                (source.latestEndpointOpenAxisMask & source.coverageStartEndpointReturnedAxisMask) &&
                source.cumulativeRetainedOpenAxisMask ==
                (source.latestEndpointOpenAxisMask & source.coverageStartEndpointOpenAxisMask) &&
                source.cumulativeRetainedReturnedAxisMask ==
                (source.latestEndpointReturnedAxisMask & source.coverageStartEndpointReturnedAxisMask) &&
                source.fullClosureEntryCount == static_cast<std::uint32_t>(!firstFull && currentFull ? 1U : 0U) &&
                source.fullClosureExitCount == static_cast<std::uint32_t>(firstFull && !currentFull ? 1U : 0U) &&
                source.fullClosureRetentionCount == static_cast<std::uint32_t>(firstFull && currentFull ? 1U : 0U);
        }
        return (source.coordinateChangeAxisUnionMask & ~startUniverse &
            ~source.cumulativeNewlyOpenedAxisMask) == 0U &&
            (source.coverageStartEndpointOpenAxisMask &
                ~(source.cumulativeNewlyReturnedAxisMask | source.cumulativeRetainedOpenAxisMask)) == 0U &&
            (source.coverageStartEndpointReturnedAxisMask &
                ~(source.cumulativeReopenedAxisMask | source.cumulativeRetainedReturnedAxisMask)) == 0U &&
            (source.latestEndpointOpenAxisMask & ~(source.cumulativeNewlyOpenedAxisMask |
                source.cumulativeReopenedAxisMask | source.cumulativeRetainedOpenAxisMask)) == 0U &&
            (source.latestEndpointReturnedAxisMask &
                ~(source.cumulativeNewlyReturnedAxisMask | source.cumulativeRetainedReturnedAxisMask)) == 0U;
    }

    static bool CoveredAxesNeverRegress(std::uint32_t previous,
        std::uint32_t current) noexcept
    {
        return (previous & ~current) == 0U;
    }

    static bool OldUniverseQualificationIsRetained(std::uint16_t previousFlags,
        std::uint16_t flag, std::uint32_t previousUniverse,
        std::uint32_t currentCoverage) noexcept
    {
        return (previousFlags & flag) == 0U ||
            (currentCoverage & previousUniverse) == previousUniverse;
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        static bool HistoricalQualificationsAdvanceMonotonically(
            const QualificationRecord& previous, const QualificationRecord& current) noexcept
    {
        return CoveredAxesNeverRegress(previous.observedOpenStateAxisMask,
            current.observedOpenStateAxisMask) &&
            CoveredAxesNeverRegress(previous.observedReturnedStateAxisMask,
                current.observedReturnedStateAxisMask) &&
            CoveredAxesNeverRegress(previous.observedBothEndpointStatesAxisMask,
                current.observedBothEndpointStatesAxisMask) &&
            CoveredAxesNeverRegress(previous.directReturnTransitionAxisMask,
                current.directReturnTransitionAxisMask) &&
            CoveredAxesNeverRegress(previous.directReopenTransitionAxisMask,
                current.directReopenTransitionAxisMask) &&
            CoveredAxesNeverRegress(previous.bidirectionalTransitionAxisMask,
                current.bidirectionalTransitionAxisMask) &&
            (NCPathCoreReturnQualificationTransitionDetail::Difference(previous.qualificationFlags, current.qualificationFlags) &
                NCPathCoreReturnQualificationTransitionDetail::HISTORICAL_FULL_FLAGS) == 0U &&
            OldUniverseQualificationIsRetained(previous.qualificationFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED,
                previous.coordinateChangeAxisUnionMask, current.observedReturnedStateAxisMask) &&
            OldUniverseQualificationIsRetained(previous.qualificationFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED,
                previous.coordinateChangeAxisUnionMask, current.observedBothEndpointStatesAxisMask) &&
            OldUniverseQualificationIsRetained(previous.qualificationFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED,
                previous.coordinateChangeAxisUnionMask, current.directReturnTransitionAxisMask) &&
            OldUniverseQualificationIsRetained(previous.qualificationFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED,
                previous.coordinateChangeAxisUnionMask, current.directReopenTransitionAxisMask) &&
            OldUniverseQualificationIsRetained(previous.qualificationFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED,
                previous.coordinateChangeAxisUnionMask, current.bidirectionalTransitionAxisMask);
    }

    NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE
        static void PopulateTransition(const QualificationRecord& previous,
            const QualificationRecord& current, Record& target) noexcept
    {
        target.previousQualificationPublicationSequence = previous.publicationSequence;
        target.previousSummaryPublicationSequence = previous.currentSummaryPublicationSequence;
        target.currentSummaryPublicationSequence = current.currentSummaryPublicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration = current.acceptedInputChainGeneration;
        target.previousProvenTransitionCount = previous.provenTransitionCount;
        target.currentProvenTransitionCount = current.provenTransitionCount;
        target.previousCoordinateChangeAxisUnionMask = previous.coordinateChangeAxisUnionMask;
        target.currentCoordinateChangeAxisUnionMask = current.coordinateChangeAxisUnionMask;
        target.newlyChangedAxisMask = current.coordinateChangeAxisUnionMask &
            ~previous.coordinateChangeAxisUnionMask;
        target.previousPendingReturnedStateAxisMask = previous.pendingReturnedStateAxisMask;
        target.currentPendingReturnedStateAxisMask = current.pendingReturnedStateAxisMask;
        target.previousPendingDirectReturnTransitionAxisMask = previous.pendingDirectReturnTransitionAxisMask;
        target.currentPendingDirectReturnTransitionAxisMask = current.pendingDirectReturnTransitionAxisMask;
        target.previousPendingBidirectionalTransitionAxisMask = previous.pendingBidirectionalTransitionAxisMask;
        target.currentPendingBidirectionalTransitionAxisMask = current.pendingBidirectionalTransitionAxisMask;
        target.addedPendingReturnedStateAxisMask = current.pendingReturnedStateAxisMask &
            ~previous.pendingReturnedStateAxisMask;
        target.removedPendingReturnedStateAxisMask = previous.pendingReturnedStateAxisMask &
            ~current.pendingReturnedStateAxisMask;
        target.addedPendingDirectReturnTransitionAxisMask = current.pendingDirectReturnTransitionAxisMask &
            ~previous.pendingDirectReturnTransitionAxisMask;
        target.removedPendingDirectReturnTransitionAxisMask = previous.pendingDirectReturnTransitionAxisMask &
            ~current.pendingDirectReturnTransitionAxisMask;
        target.addedPendingBidirectionalTransitionAxisMask = current.pendingBidirectionalTransitionAxisMask &
            ~previous.pendingBidirectionalTransitionAxisMask;
        target.removedPendingBidirectionalTransitionAxisMask = previous.pendingBidirectionalTransitionAxisMask &
            ~current.pendingBidirectionalTransitionAxisMask;
        target.previousQualificationFlags = previous.qualificationFlags;
        target.currentQualificationFlags = current.qualificationFlags;
        target.newlyGainedQualificationFlags =
            NCPathCoreReturnQualificationTransitionDetail::Difference(current.qualificationFlags, previous.qualificationFlags);
        target.retainedQualificationFlags = static_cast<std::uint16_t>(
            previous.qualificationFlags & current.qualificationFlags);
        target.lostQualificationFlags =
            NCPathCoreReturnQualificationTransitionDetail::Difference(previous.qualificationFlags, current.qualificationFlags);
        target.universeExpansionLostQualificationFlags = static_cast<std::uint16_t>(
            target.lostQualificationFlags & NCPathCoreReturnQualificationTransitionDetail::ALL_AXIS_FLAGS);
        target.disposition = Disposition::PROVEN_DIRECT_RETURN_COVERAGE_QUALIFICATION_TRANSITION;
        target.relation = Relation::DIRECT_SAME_COVERAGE_INTERVAL_QUALIFICATION_PAIR;
        target.extent = Extent::SCALAR_QUALIFICATION_FLAGS_AND_PENDING_MASK_DIFFERENCES_ONLY;
        if (!target.IsProvenRunEndpointReturnCoverageQualificationTransition())
        {
            ClearProvenPayload(target);
            target.disposition = Disposition::INVALID_QUALIFICATION_TRANSITION;
        }
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RETURN_QUALIFICATION_TRANSITION_NOINLINE

static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionDisposition) == 1U,
    "Qualification-transition disposition must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRelation) == 1U,
    "Qualification-transition relation must remain one byte.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionExtent) == 1U,
    "Qualification-transition extent must remain one byte.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1>::value,
    "Qualification-transition record must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1>::value,
    "Qualification-transition record must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1) == 8U,
    "Qualification-transition record alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1) == 152U,
    "Qualification-transition record must remain exactly 152 bytes.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1, previousProvenTransitionCount) == 64U,
    "Qualification-transition mask block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1, schemaVersion) == 132U,
    "Qualification-transition flags block offset changed.");
static_assert(offsetof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionRecordV1, reserved) == 149U,
    "Qualification-transition reserve offset changed.");
static_assert(std::is_standard_layout<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow>::value,
    "Qualification-transition observer must remain standard-layout.");
static_assert(std::is_trivially_copyable<NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow>::value,
    "Qualification-transition observer must remain trivially copyable.");
static_assert(alignof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow) == 8U,
    "Qualification-transition observer alignment changed.");
static_assert(sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow) == 320U,
    "Qualification-transition observer must remain exactly 320 bytes.");
