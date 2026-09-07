#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2N / Proven Linked Committed Segment Run Endpoint-
// Return Coverage Qualification Shadow
//
// This fixed two-record observer consumes only the newest L.2M return-
// transition coverage summary.  It qualifies which axes have been observed
// in the scalar endpoint-open state, the scalar endpoint-returned state, both
// states, a direct OPEN->RETURNED transition, a direct RETURNED->OPEN
// transition, or both direct transition directions during the same proven
// coverage interval.
//
// The output also records which changed axes still lack returned-state,
// direct-return or bidirectional-transition coverage, plus scalar flags for
// full-closure observation/current state/exit/re-entry evidence.  These are
// coverage qualifications only.  They do not preserve transition order,
// coordinates, displacement, segment identity lists or intermediate points.
//
// "Returned" remains only an exact committed commanded-MCS endpoint
// relation.  This shadow is not actual Motion execution, physical reversal,
// travelled distance, retrace order, a Path Queue, a segment list or a B2
// breadcrumb.  It has no Gate, PC, Alarm, Motion, HMI/SHM/API, PDO,
// EtherCAT or DC consumer.  Neutral and invalid observations clear all proven
// payload.  The shadow is same-thread, fixed-capacity and heap-resident
// through NCManager, with no allocation, logging, waiting or synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY =
2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_AXIS_MASK =
0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_SCHEMA_V1 =
1U;

constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED =
0x0001U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED =
0x0002U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED =
0x0004U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED =
0x0008U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED =
0x0010U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED = 0x0020U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT = 0x0040U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED =
0x0080U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED =
0x0100U;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_FLAG_MASK =
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED;

namespace NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail
{
    constexpr std::uint64_t NextNonZeroSequence(
        std::uint64_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL
            : value + 1ULL;
    }

    constexpr std::uint64_t ForwardNonZeroSequenceDistance(
        std::uint64_t first,
        std::uint64_t latest) noexcept
    {
        return latest >= first
            ? latest - first
            : ((std::numeric_limits<std::uint64_t>::max)() - first) + latest;
    }

    constexpr std::uint32_t SaturatingIncrement(
        std::uint32_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint32_t>::max)()
            ? value
            : value + 1U;
    }
}

enum class
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_SUMMARY_UNAVAILABLE = 1U,
    NOT_APPLICABLE_SUMMARY_NOT_PROVEN = 2U,
    INVALID_CURRENT_SUMMARY_RECORD = 3U,
    INVALID_PREVIOUS_SUMMARY_RECORD = 4U,
    INVALID_SUMMARY_ADVANCE_FENCE = 5U,
    INVALID_COVERAGE_QUALIFICATION = 6U,
    PROVEN_RETURN_COVERAGE_QUALIFICATION_STARTED = 7U,
    PROVEN_RETURN_COVERAGE_QUALIFICATION_EXTENDED = 8U
};

enum class
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation :
    std::uint8_t
{
    NONE = 0U,
    FIRST_PROVEN_SUMMARY_ESTABLISHES_QUALIFICATION = 1U,
    DIRECT_PROVEN_SUMMARY_EXTENDS_QUALIFICATION = 2U
};

enum class
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent :
    std::uint8_t
{
    NONE = 0U,
    SCALAR_RETURN_STATE_AND_TRANSITION_COVERAGE_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
#endif

struct
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousSummaryPublicationSequence = 0ULL;
    std::uint64_t currentSummaryPublicationSequence = 0ULL;
    std::uint64_t coverageGeneration = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t firstTransitionPublicationSequence = 0ULL;
    std::uint64_t latestTransitionPublicationSequence = 0ULL;
    std::uint64_t firstClosurePublicationSequence = 0ULL;
    std::uint64_t latestClosurePublicationSequence = 0ULL;
    std::uint64_t firstSegmentPublicationSequence = 0ULL;
    std::uint64_t latestSegmentPublicationSequence = 0ULL;
    std::uint64_t firstLinkPublicationSequence = 0ULL;
    std::uint64_t latestLinkPublicationSequence = 0ULL;
    std::uint64_t runStartGeometryPublicationSequence = 0ULL;
    std::uint64_t runEndGeometryPublicationSequence = 0ULL;
    std::uint64_t runStartGeometryMotionSegmentId = 0ULL;
    std::uint64_t runEndGeometryMotionSegmentId = 0ULL;

    std::uint32_t provenTransitionCount = 0U;
    std::uint32_t coverageStartLinkedSegmentRunLength = 0U;
    std::uint32_t latestLinkedSegmentRunLength = 0U;
    std::uint32_t participatingAxisUnionMask = 0U;
    std::uint32_t coordinateChangeAxisUnionMask = 0U;
    std::uint32_t cumulativeNewlyOpenedAxisMask = 0U;
    std::uint32_t cumulativeNewlyReturnedAxisMask = 0U;
    std::uint32_t cumulativeReopenedAxisMask = 0U;
    std::uint32_t cumulativeRetainedOpenAxisMask = 0U;
    std::uint32_t cumulativeRetainedReturnedAxisMask = 0U;
    std::uint32_t coverageStartEndpointOpenAxisMask = 0U;
    std::uint32_t coverageStartEndpointReturnedAxisMask = 0U;
    std::uint32_t latestEndpointOpenAxisMask = 0U;
    std::uint32_t latestEndpointReturnedAxisMask = 0U;
    std::uint32_t fullClosureEntryCount = 0U;
    std::uint32_t fullClosureExitCount = 0U;
    std::uint32_t fullClosureRetentionCount = 0U;
    std::uint32_t observedOpenStateAxisMask = 0U;
    std::uint32_t observedReturnedStateAxisMask = 0U;
    std::uint32_t observedBothEndpointStatesAxisMask = 0U;
    std::uint32_t directReturnTransitionAxisMask = 0U;
    std::uint32_t directReopenTransitionAxisMask = 0U;
    std::uint32_t bidirectionalTransitionAxisMask = 0U;
    std::uint32_t pendingReturnedStateAxisMask = 0U;
    std::uint32_t pendingDirectReturnTransitionAxisMask = 0U;
    std::uint32_t pendingBidirectionalTransitionAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t coverageStartPreviousLatestSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    std::uint16_t qualificationFlags = 0U;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition
        disposition =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
        EMPTY;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation
        relation =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
        NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent
        extent =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent::
        NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureClass firstClosureClass =
        NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureClass latestClosureClass =
        NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 8U> reserved{};

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        bool IsProvenRunEndpointReturnCoverageQualification() const noexcept
    {
        const bool started =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
            PROVEN_RETURN_COVERAGE_QUALIFICATION_STARTED &&
            relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
            FIRST_PROVEN_SUMMARY_ESTABLISHES_QUALIFICATION;
        const bool extended =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
            PROVEN_RETURN_COVERAGE_QUALIFICATION_EXTENDED &&
            relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
            DIRECT_PROVEN_SUMMARY_EXTENDS_QUALIFICATION;

        return BasicProofFieldsAreValid(started, extended) &&
            SourceCoverageSpansAreValid() &&
            SourceCoverageMasksAreValid() &&
            QualificationMasksAreValid() &&
            FullClosureFactsAreValid();
    }

private:
    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        bool BasicProofFieldsAreValid(
            bool started,
            bool extended) const noexcept
    {
        const bool sourceShapeMatchesRelation =
            started
            ? previousSummaryPublicationSequence == 0ULL &&
            provenTransitionCount == 1U &&
            firstTransitionPublicationSequence ==
            latestTransitionPublicationSequence
            : previousSummaryPublicationSequence != 0ULL &&
            currentSummaryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previousSummaryPublicationSequence) &&
            provenTransitionCount >= 2U &&
            firstTransitionPublicationSequence !=
            latestTransitionPublicationSequence;

        return publicationSequence != 0ULL &&
            currentSummaryPublicationSequence != 0ULL &&
            coverageGeneration != 0ULL &&
            runGeneration != 0ULL &&
            acceptedInputChainGeneration != 0ULL &&
            firstTransitionPublicationSequence != 0ULL &&
            latestTransitionPublicationSequence != 0ULL &&
            firstClosurePublicationSequence != 0ULL &&
            latestClosurePublicationSequence != 0ULL &&
            firstSegmentPublicationSequence != 0ULL &&
            latestSegmentPublicationSequence != 0ULL &&
            firstLinkPublicationSequence != 0ULL &&
            latestLinkPublicationSequence != 0ULL &&
            runStartGeometryPublicationSequence != 0ULL &&
            runEndGeometryPublicationSequence != 0ULL &&
            runStartGeometryMotionSegmentId != 0ULL &&
            runEndGeometryMotionSegmentId > runStartGeometryMotionSegmentId &&
            provenTransitionCount != 0U &&
            coverageStartLinkedSegmentRunLength >= 2U &&
            latestLinkedSegmentRunLength >= 3U &&
            participatingAxisUnionMask != 0U &&
            (participatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_AXIS_MASK) ==
            0U &&
            (coordinateChangeAxisUnionMask & ~participatingAxisUnionMask) == 0U &&
            firstSourcePC >= 0 &&
            coverageStartPreviousLatestSourcePC >= firstSourcePC &&
            latestSourcePC > coverageStartPreviousLatestSourcePC &&
            schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_SCHEMA_V1 &&
            (qualificationFlags &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_FLAG_MASK) ==
            0U &&
            (started || extended) &&
            sourceShapeMatchesRelation &&
            extent ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent::
            SCALAR_RETURN_STATE_AND_TRANSITION_COVERAGE_ONLY &&
            frame == NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE &&
            kind == NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR &&
            commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP &&
            ReservedIsZero();
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        bool SourceCoverageSpansAreValid() const noexcept
    {
        const std::int64_t coveragePcSpan =
            static_cast<std::int64_t>(latestSourcePC) -
            static_cast<std::int64_t>(coverageStartPreviousLatestSourcePC);
        const std::int64_t startRunPcSpan =
            static_cast<std::int64_t>(coverageStartPreviousLatestSourcePC) -
            static_cast<std::int64_t>(firstSourcePC) + 1LL;
        const std::int64_t latestRunPcSpan =
            static_cast<std::int64_t>(latestSourcePC) -
            static_cast<std::int64_t>(firstSourcePC) + 1LL;
        if (coveragePcSpan <= 0LL ||
            static_cast<std::uint64_t>(coveragePcSpan) !=
            static_cast<std::uint64_t>(provenTransitionCount) ||
            startRunPcSpan <= 0LL ||
            static_cast<std::uint64_t>(startRunPcSpan) !=
            static_cast<std::uint64_t>(coverageStartLinkedSegmentRunLength) ||
            latestRunPcSpan <= 0LL ||
            static_cast<std::uint64_t>(latestRunPcSpan) !=
            static_cast<std::uint64_t>(latestLinkedSegmentRunLength))
        {
            return false;
        }

        const std::uint64_t expectedLatestRunLength =
            static_cast<std::uint64_t>(coverageStartLinkedSegmentRunLength) +
            static_cast<std::uint64_t>(provenTransitionCount);
        if (expectedLatestRunLength >
            static_cast<std::uint64_t>(
                (std::numeric_limits<std::uint32_t>::max)()) ||
            latestLinkedSegmentRunLength !=
            static_cast<std::uint32_t>(expectedLatestRunLength))
        {
            return false;
        }

        const std::uint64_t transitionCount =
            static_cast<std::uint64_t>(provenTransitionCount);
        const std::uint64_t runLength =
            static_cast<std::uint64_t>(latestLinkedSegmentRunLength);
        return
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            ForwardNonZeroSequenceDistance(
                firstTransitionPublicationSequence,
                latestTransitionPublicationSequence) ==
            transitionCount - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            ForwardNonZeroSequenceDistance(
                firstClosurePublicationSequence,
                latestClosurePublicationSequence) == transitionCount &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) == runLength;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        bool SourceCoverageMasksAreValid() const noexcept
    {
        const std::uint32_t allCoverageMasks =
            cumulativeNewlyOpenedAxisMask |
            cumulativeNewlyReturnedAxisMask |
            cumulativeReopenedAxisMask |
            cumulativeRetainedOpenAxisMask |
            cumulativeRetainedReturnedAxisMask;
        if ((allCoverageMasks &
            ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_AXIS_MASK) !=
            0U ||
            (allCoverageMasks & ~coordinateChangeAxisUnionMask) != 0U ||
            allCoverageMasks != coordinateChangeAxisUnionMask ||
            !EndpointStateMasksAreValid(
                coverageStartEndpointOpenAxisMask,
                coverageStartEndpointReturnedAxisMask,
                false) ||
            !EndpointStateMasksAreValid(
                latestEndpointOpenAxisMask,
                latestEndpointReturnedAxisMask,
                true) ||
            !ClosureClassMatchesState(
                coverageStartEndpointOpenAxisMask,
                coverageStartEndpointReturnedAxisMask,
                firstClosureClass) ||
            !ClosureClassMatchesState(
                latestEndpointOpenAxisMask,
                latestEndpointReturnedAxisMask,
                latestClosureClass) ||
            fullClosureEntryCount > provenTransitionCount ||
            fullClosureExitCount > provenTransitionCount ||
            fullClosureRetentionCount > provenTransitionCount ||
            static_cast<std::uint64_t>(fullClosureEntryCount) +
            static_cast<std::uint64_t>(fullClosureExitCount) +
            static_cast<std::uint64_t>(fullClosureRetentionCount) >
            static_cast<std::uint64_t>(provenTransitionCount))
        {
            return false;
        }

        const bool firstFull = IsFullClosure(firstClosureClass);
        const bool latestFull = IsFullClosure(latestClosureClass);
        const std::int64_t balance =
            static_cast<std::int64_t>(fullClosureEntryCount) -
            static_cast<std::int64_t>(fullClosureExitCount);
        const std::int64_t expectedBalance =
            !firstFull && latestFull
            ? 1LL
            : (firstFull && !latestFull ? -1LL : 0LL);
        return balance == expectedBalance;
    }

    bool QualificationMasksAreValid() const noexcept
    {
        const std::uint32_t expectedObservedOpenStateAxisMask =
            coverageStartEndpointOpenAxisMask |
            cumulativeNewlyOpenedAxisMask |
            cumulativeReopenedAxisMask |
            cumulativeRetainedOpenAxisMask |
            latestEndpointOpenAxisMask;
        const std::uint32_t expectedObservedReturnedStateAxisMask =
            coverageStartEndpointReturnedAxisMask |
            cumulativeNewlyReturnedAxisMask |
            cumulativeRetainedReturnedAxisMask |
            latestEndpointReturnedAxisMask;
        const std::uint32_t expectedObservedBothEndpointStatesAxisMask =
            expectedObservedOpenStateAxisMask &
            expectedObservedReturnedStateAxisMask;
        const std::uint32_t expectedDirectReturnTransitionAxisMask =
            cumulativeNewlyReturnedAxisMask;
        const std::uint32_t expectedDirectReopenTransitionAxisMask =
            cumulativeReopenedAxisMask;
        const std::uint32_t expectedBidirectionalTransitionAxisMask =
            expectedDirectReturnTransitionAxisMask &
            expectedDirectReopenTransitionAxisMask;
        const std::uint32_t expectedPendingReturnedStateAxisMask =
            coordinateChangeAxisUnionMask &
            ~expectedObservedReturnedStateAxisMask;
        const std::uint32_t expectedPendingDirectReturnTransitionAxisMask =
            coordinateChangeAxisUnionMask &
            ~expectedDirectReturnTransitionAxisMask;
        const std::uint32_t expectedPendingBidirectionalTransitionAxisMask =
            coordinateChangeAxisUnionMask &
            ~expectedBidirectionalTransitionAxisMask;

        return
            (expectedObservedOpenStateAxisMask &
                ~coordinateChangeAxisUnionMask) == 0U &&
            (expectedObservedReturnedStateAxisMask &
                ~coordinateChangeAxisUnionMask) == 0U &&
            (expectedObservedOpenStateAxisMask |
                expectedObservedReturnedStateAxisMask) ==
            coordinateChangeAxisUnionMask &&
            observedOpenStateAxisMask ==
            expectedObservedOpenStateAxisMask &&
            observedReturnedStateAxisMask ==
            expectedObservedReturnedStateAxisMask &&
            observedBothEndpointStatesAxisMask ==
            expectedObservedBothEndpointStatesAxisMask &&
            directReturnTransitionAxisMask ==
            expectedDirectReturnTransitionAxisMask &&
            directReopenTransitionAxisMask ==
            expectedDirectReopenTransitionAxisMask &&
            bidirectionalTransitionAxisMask ==
            expectedBidirectionalTransitionAxisMask &&
            pendingReturnedStateAxisMask ==
            expectedPendingReturnedStateAxisMask &&
            pendingDirectReturnTransitionAxisMask ==
            expectedPendingDirectReturnTransitionAxisMask &&
            pendingBidirectionalTransitionAxisMask ==
            expectedPendingBidirectionalTransitionAxisMask;
    }

    bool FullClosureFactsAreValid() const noexcept
    {
        return qualificationFlags == ExpectedQualificationFlags();
    }

    std::uint16_t ExpectedQualificationFlags() const noexcept
    {
        std::uint16_t flags = 0U;
        const bool hasChangedAxis = coordinateChangeAxisUnionMask != 0U;
        if (hasChangedAxis &&
            observedReturnedStateAxisMask == coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            observedBothEndpointStatesAxisMask == coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            directReturnTransitionAxisMask == coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            directReopenTransitionAxisMask == coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            bidirectionalTransitionAxisMask == coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED;
        }

        const bool firstFull = IsFullClosure(firstClosureClass);
        const bool latestFull = IsFullClosure(latestClosureClass);
        const bool fullClosureObserved =
            firstFull ||
            fullClosureEntryCount != 0U ||
            fullClosureRetentionCount != 0U;
        const bool fullClosureReentryObserved =
            firstFull
            ? fullClosureEntryCount != 0U
            : fullClosureEntryCount >= 2U;
        if (fullClosureObserved)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED;
        }
        if (latestFull)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT;
        }
        if (fullClosureExitCount != 0U)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED;
        }
        if (fullClosureReentryObserved)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED;
        }
        return flags;
    }

    bool EndpointStateMasksAreValid(
        std::uint32_t endpointOpenAxisMask,
        std::uint32_t endpointReturnedAxisMask,
        bool requireCurrentCoverage) const noexcept
    {
        const std::uint32_t stateMask =
            endpointOpenAxisMask | endpointReturnedAxisMask;
        return
            (endpointOpenAxisMask & endpointReturnedAxisMask) == 0U &&
            (stateMask & ~coordinateChangeAxisUnionMask) == 0U &&
            (!requireCurrentCoverage ||
                stateMask == coordinateChangeAxisUnionMask);
    }

    static bool ClosureClassMatchesState(
        std::uint32_t endpointOpenAxisMask,
        std::uint32_t endpointReturnedAxisMask,
        NCPathCoreLinkedCommittedSegmentRunClosureClass closureClass) noexcept
    {
        const std::uint32_t coordinateChangeAxisMask =
            endpointOpenAxisMask | endpointReturnedAxisMask;
        if (coordinateChangeAxisMask == 0U)
        {
            return closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                NO_COMMANDED_COORDINATE_CHANGE;
        }
        if (endpointOpenAxisMask == 0U)
        {
            return closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        }
        if (endpointReturnedAxisMask == 0U)
        {
            return closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                ENDPOINT_OPEN_NO_RETURNED_AXIS;
        }
        return closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            ENDPOINT_OPEN_WITH_RETURNED_AXIS;
    }

    static bool IsFullClosure(
        NCPathCoreLinkedCommittedSegmentRunClosureClass closureClass) noexcept
    {
        return closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
    }

    bool ReservedIsZero() const noexcept
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
};

class
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow() noexcept =
        default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        void ObserveLatestSummarySameThread(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1*
            previousSummary,
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1*
            currentSummary) noexcept
    {
        m_publicationSequence =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY;

        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1&
            target = m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            currentSummary == nullptr
            ? 0ULL
            : currentSummary->publicationSequence);

        if (currentSummary == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                NOT_APPLICABLE_SUMMARY_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenSummary(*currentSummary))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                NOT_APPLICABLE_SUMMARY_NOT_PROVEN;
        }
        else if (!currentSummary->
            IsProvenRunEndpointReturnTransitionCoverageSummary())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                INVALID_CURRENT_SUMMARY_RECORD;
        }
        else if (currentSummary->disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_STARTED)
        {
            PopulateQualification(nullptr, *currentSummary, target);
        }
        else if (previousSummary == nullptr ||
            IsStructurallyValidNonProvenSummary(*previousSummary) ||
            !previousSummary->
            IsProvenRunEndpointReturnTransitionCoverageSummary())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                INVALID_PREVIOUS_SUMMARY_RECORD;
        }
        else if (!IsDirectSummaryExtensionFenceValid(
            *previousSummary,
            *currentSummary))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                INVALID_SUMMARY_ADVANCE_FENCE;
        }
        else
        {
            PopulateQualification(previousSummary, *currentSummary, target);
        }

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1*
        GetNewestObservationSameThread(
            std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1&
        target,
        std::uint64_t publicationSequence,
        std::uint64_t currentSummaryPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.currentSummaryPublicationSequence =
            currentSummaryPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
            NOT_APPLICABLE_SUMMARY_UNAVAILABLE;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
            NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent::
            NONE;
        target.firstClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.latestClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static void ClearProvenPayload(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1&
        target) noexcept
    {
        target.previousSummaryPublicationSequence = 0ULL;
        target.coverageGeneration = 0ULL;
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.firstTransitionPublicationSequence = 0ULL;
        target.latestTransitionPublicationSequence = 0ULL;
        target.firstClosurePublicationSequence = 0ULL;
        target.latestClosurePublicationSequence = 0ULL;
        target.firstSegmentPublicationSequence = 0ULL;
        target.latestSegmentPublicationSequence = 0ULL;
        target.firstLinkPublicationSequence = 0ULL;
        target.latestLinkPublicationSequence = 0ULL;
        target.runStartGeometryPublicationSequence = 0ULL;
        target.runEndGeometryPublicationSequence = 0ULL;
        target.runStartGeometryMotionSegmentId = 0ULL;
        target.runEndGeometryMotionSegmentId = 0ULL;
        target.provenTransitionCount = 0U;
        target.coverageStartLinkedSegmentRunLength = 0U;
        target.latestLinkedSegmentRunLength = 0U;
        target.participatingAxisUnionMask = 0U;
        target.coordinateChangeAxisUnionMask = 0U;
        target.cumulativeNewlyOpenedAxisMask = 0U;
        target.cumulativeNewlyReturnedAxisMask = 0U;
        target.cumulativeReopenedAxisMask = 0U;
        target.cumulativeRetainedOpenAxisMask = 0U;
        target.cumulativeRetainedReturnedAxisMask = 0U;
        target.coverageStartEndpointOpenAxisMask = 0U;
        target.coverageStartEndpointReturnedAxisMask = 0U;
        target.latestEndpointOpenAxisMask = 0U;
        target.latestEndpointReturnedAxisMask = 0U;
        target.fullClosureEntryCount = 0U;
        target.fullClosureExitCount = 0U;
        target.fullClosureRetentionCount = 0U;
        target.observedOpenStateAxisMask = 0U;
        target.observedReturnedStateAxisMask = 0U;
        target.observedBothEndpointStatesAxisMask = 0U;
        target.directReturnTransitionAxisMask = 0U;
        target.directReopenTransitionAxisMask = 0U;
        target.bidirectionalTransitionAxisMask = 0U;
        target.pendingReturnedStateAxisMask = 0U;
        target.pendingDirectReturnTransitionAxisMask = 0U;
        target.pendingBidirectionalTransitionAxisMask = 0U;
        target.firstSourcePC = -1;
        target.coverageStartPreviousLatestSourcePC = -1;
        target.latestSourcePC = -1;
        target.qualificationFlags = 0U;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
            NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent::
            NONE;
        target.firstClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.latestClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    }

    static bool SummaryReservedIsZero(
        const std::array<std::uint8_t, 6U>& reserved) noexcept
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        static bool IsStructurallyValidNonProvenSummary(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            summary) noexcept
    {
        const bool knownDisposition =
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            NOT_APPLICABLE_TRANSITION_UNAVAILABLE ||
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            NOT_APPLICABLE_TRANSITION_NOT_PROVEN ||
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            INVALID_CURRENT_TRANSITION_RECORD ||
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            INVALID_PREVIOUS_SUMMARY_RECORD ||
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            INVALID_TRANSITION_SUMMARY_ADVANCE_FENCE ||
            summary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            INVALID_TRANSITION_SUMMARY_ACCUMULATION;

        return knownDisposition &&
            summary.publicationSequence != 0ULL &&
            summary.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_SCHEMA_V1 &&
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
            summary.firstSourcePC == -1 &&
            summary.coverageStartPreviousLatestSourcePC == -1 &&
            summary.latestSourcePC == -1 &&
            summary.relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            NONE &&
            summary.extent ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
            NONE &&
            summary.firstClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            summary.latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            summary.frame == NCPathCoreCommittedGeometryFrame::NONE &&
            summary.kind == NCPathCoreCommittedGeometryKind::NONE &&
            summary.commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE &&
            SummaryReservedIsZero(summary.reserved);
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        static bool IsDirectSummaryExtensionFenceValid(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            current) noexcept
    {
        if (current.disposition !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_EXTENDED ||
            current.publicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.publicationSequence) ||
            current.sourceTransitionPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.sourceTransitionPublicationSequence) ||
            current.coverageGeneration != previous.coverageGeneration ||
            current.runGeneration != previous.runGeneration ||
            current.acceptedInputChainGeneration !=
            previous.acceptedInputChainGeneration ||
            current.firstTransitionPublicationSequence !=
            previous.firstTransitionPublicationSequence ||
            current.latestTransitionPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.latestTransitionPublicationSequence) ||
            current.firstClosurePublicationSequence !=
            previous.firstClosurePublicationSequence ||
            current.latestClosurePublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.latestClosurePublicationSequence) ||
            current.firstSegmentPublicationSequence !=
            previous.firstSegmentPublicationSequence ||
            current.latestSegmentPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.latestSegmentPublicationSequence) ||
            current.firstLinkPublicationSequence !=
            previous.firstLinkPublicationSequence ||
            current.latestLinkPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.latestLinkPublicationSequence) ||
            current.runStartGeometryPublicationSequence !=
            previous.runStartGeometryPublicationSequence ||
            current.runEndGeometryPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            NextNonZeroSequence(previous.runEndGeometryPublicationSequence) ||
            current.runStartGeometryMotionSegmentId !=
            previous.runStartGeometryMotionSegmentId ||
            current.runEndGeometryMotionSegmentId <=
            previous.runEndGeometryMotionSegmentId ||
            current.provenTransitionCount !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            SaturatingIncrement(previous.provenTransitionCount) ||
            current.coverageStartLinkedSegmentRunLength !=
            previous.coverageStartLinkedSegmentRunLength ||
            current.latestLinkedSegmentRunLength !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
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
        const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
        previous,
        const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
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
        const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
        previous,
        const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
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
            ? NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            SaturatingIncrement(previous.fullClosureEntryCount)
            : previous.fullClosureEntryCount;
        const std::uint32_t expectedExitCount =
            previousFull && !currentFull
            ? NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            SaturatingIncrement(previous.fullClosureExitCount)
            : previous.fullClosureExitCount;
        const std::uint32_t expectedRetentionCount =
            previousFull && currentFull
            ? NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDetail::
            SaturatingIncrement(previous.fullClosureRetentionCount)
            : previous.fullClosureRetentionCount;
        return current.fullClosureEntryCount == expectedEntryCount &&
            current.fullClosureExitCount == expectedExitCount &&
            current.fullClosureRetentionCount == expectedRetentionCount;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE
        static void PopulateQualification(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1*
            previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            current,
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1&
            target) noexcept
    {
        target.previousSummaryPublicationSequence =
            previous == nullptr ? 0ULL : previous->publicationSequence;
        target.coverageGeneration = current.coverageGeneration;
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration =
            current.acceptedInputChainGeneration;
        target.firstTransitionPublicationSequence =
            current.firstTransitionPublicationSequence;
        target.latestTransitionPublicationSequence =
            current.latestTransitionPublicationSequence;
        target.firstClosurePublicationSequence =
            current.firstClosurePublicationSequence;
        target.latestClosurePublicationSequence =
            current.latestClosurePublicationSequence;
        target.firstSegmentPublicationSequence =
            current.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            current.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            current.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            current.latestLinkPublicationSequence;
        target.runStartGeometryPublicationSequence =
            current.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            current.runEndGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            current.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            current.runEndGeometryMotionSegmentId;
        target.provenTransitionCount = current.provenTransitionCount;
        target.coverageStartLinkedSegmentRunLength =
            current.coverageStartLinkedSegmentRunLength;
        target.latestLinkedSegmentRunLength =
            current.latestLinkedSegmentRunLength;
        target.participatingAxisUnionMask =
            current.participatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            current.coordinateChangeAxisUnionMask;
        target.cumulativeNewlyOpenedAxisMask =
            current.cumulativeNewlyOpenedAxisMask;
        target.cumulativeNewlyReturnedAxisMask =
            current.cumulativeNewlyReturnedAxisMask;
        target.cumulativeReopenedAxisMask =
            current.cumulativeReopenedAxisMask;
        target.cumulativeRetainedOpenAxisMask =
            current.cumulativeRetainedOpenAxisMask;
        target.cumulativeRetainedReturnedAxisMask =
            current.cumulativeRetainedReturnedAxisMask;
        target.coverageStartEndpointOpenAxisMask =
            current.coverageStartEndpointOpenAxisMask;
        target.coverageStartEndpointReturnedAxisMask =
            current.coverageStartEndpointReturnedAxisMask;
        target.latestEndpointOpenAxisMask =
            current.latestEndpointOpenAxisMask;
        target.latestEndpointReturnedAxisMask =
            current.latestEndpointReturnedAxisMask;
        target.fullClosureEntryCount = current.fullClosureEntryCount;
        target.fullClosureExitCount = current.fullClosureExitCount;
        target.fullClosureRetentionCount =
            current.fullClosureRetentionCount;

        target.observedOpenStateAxisMask =
            current.coverageStartEndpointOpenAxisMask |
            current.cumulativeNewlyOpenedAxisMask |
            current.cumulativeReopenedAxisMask |
            current.cumulativeRetainedOpenAxisMask |
            current.latestEndpointOpenAxisMask;
        target.observedReturnedStateAxisMask =
            current.coverageStartEndpointReturnedAxisMask |
            current.cumulativeNewlyReturnedAxisMask |
            current.cumulativeRetainedReturnedAxisMask |
            current.latestEndpointReturnedAxisMask;
        target.observedBothEndpointStatesAxisMask =
            target.observedOpenStateAxisMask &
            target.observedReturnedStateAxisMask;
        target.directReturnTransitionAxisMask =
            current.cumulativeNewlyReturnedAxisMask;
        target.directReopenTransitionAxisMask =
            current.cumulativeReopenedAxisMask;
        target.bidirectionalTransitionAxisMask =
            target.directReturnTransitionAxisMask &
            target.directReopenTransitionAxisMask;
        target.pendingReturnedStateAxisMask =
            current.coordinateChangeAxisUnionMask &
            ~target.observedReturnedStateAxisMask;
        target.pendingDirectReturnTransitionAxisMask =
            current.coordinateChangeAxisUnionMask &
            ~target.directReturnTransitionAxisMask;
        target.pendingBidirectionalTransitionAxisMask =
            current.coordinateChangeAxisUnionMask &
            ~target.bidirectionalTransitionAxisMask;
        target.firstSourcePC = current.firstSourcePC;
        target.coverageStartPreviousLatestSourcePC =
            current.coverageStartPreviousLatestSourcePC;
        target.latestSourcePC = current.latestSourcePC;
        target.firstClosureClass = current.firstClosureClass;
        target.latestClosureClass = current.latestClosureClass;
        target.frame = current.frame;
        target.kind = current.kind;
        target.commandPathPolicy = current.commandPathPolicy;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent::
            SCALAR_RETURN_STATE_AND_TRANSITION_COVERAGE_ONLY;

        target.qualificationFlags = ExpectedQualificationFlags(target);
        if (previous == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                PROVEN_RETURN_COVERAGE_QUALIFICATION_STARTED;
            target.relation =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
                FIRST_PROVEN_SUMMARY_ESTABLISHES_QUALIFICATION;
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                PROVEN_RETURN_COVERAGE_QUALIFICATION_EXTENDED;
            target.relation =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation::
                DIRECT_PROVEN_SUMMARY_EXTENDS_QUALIFICATION;
        }

        if (!target.IsProvenRunEndpointReturnCoverageQualification())
        {
            ClearProvenPayload(target);
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition::
                INVALID_COVERAGE_QUALIFICATION;
        }
    }

    static std::uint16_t ExpectedQualificationFlags(
        const
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1&
        record) noexcept
    {
        std::uint16_t flags = 0U;
        const bool hasChangedAxis = record.coordinateChangeAxisUnionMask != 0U;
        if (hasChangedAxis &&
            record.observedReturnedStateAxisMask ==
            record.coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_RETURNED_STATE_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            record.observedBothEndpointStatesAxisMask ==
            record.coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOTH_ENDPOINT_STATES_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            record.directReturnTransitionAxisMask ==
            record.coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_RETURN_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            record.directReopenTransitionAxisMask ==
            record.coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DIRECT_REOPEN_ALL_AXES_COVERED;
        }
        if (hasChangedAxis &&
            record.bidirectionalTransitionAxisMask ==
            record.coordinateChangeAxisUnionMask)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BIDIRECTIONAL_TRANSITION_ALL_AXES_COVERED;
        }

        const bool firstFull =
            record.firstClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool latestFull =
            record.latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool fullClosureObserved =
            firstFull ||
            record.fullClosureEntryCount != 0U ||
            record.fullClosureRetentionCount != 0U;
        const bool fullClosureReentryObserved =
            firstFull
            ? record.fullClosureEntryCount != 0U
            : record.fullClosureEntryCount >= 2U;
        if (fullClosureObserved)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_OBSERVED;
        }
        if (latestFull)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_CURRENT;
        }
        if (record.fullClosureExitCount != 0U)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_EXIT_OBSERVED;
        }
        if (fullClosureReentryObserved)
        {
            flags |=
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_FULL_CLOSURE_REENTRY_OBSERVED;
        }
        return flags;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_COVERAGE_QUALIFICATION_NOINLINE

static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationDisposition) ==
    1U,
    "Return coverage qualification disposition must remain one byte.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRelation) ==
    1U,
    "Return coverage qualification relation must remain one byte.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationExtent) ==
    1U,
    "Return coverage qualification extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1>::
    value,
    "Return coverage qualification record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1>::
    value,
    "Return coverage qualification record must remain trivially copyable.");
static_assert(
    alignof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1) ==
    8U,
    "Return coverage qualification record alignment changed.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1) ==
    280U,
    "Return coverage qualification record must remain exactly 280 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1,
        provenTransitionCount) == 144U,
    "Return coverage qualification mask block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1,
        firstSourcePC) == 248U,
    "Return coverage qualification source-PC block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1,
        schemaVersion) == 260U,
    "Return coverage qualification schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationRecordV1,
        reserved) == 272U,
    "Return coverage qualification reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow>::
    value,
    "Return coverage qualification shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow>::
    value,
    "Return coverage qualification shadow must remain trivially copyable.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow) ==
    576U,
    "Return coverage qualification shadow must remain exactly 576 bytes.");
