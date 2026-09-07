#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2M / Proven Linked Committed Segment Run Endpoint-
// Return Transition Coverage Summary Shadow
//
// This fixed two-record observer consumes only the newest L.2L endpoint-
// return transition observation.  The first proven L.2L transition starts a
// scalar coverage interval.  Each later proven transition may extend that
// interval only when it is the direct next transition of the same committed
// run and its previous endpoint state exactly binds the summary tail.
//
// A proven summary accumulates only event-set masks and saturating counts:
// axes observed newly opened, newly returned, reopened, retained open and
// retained returned, plus full-closure entry, exit and retention counts.  It
// also retains the first and latest endpoint-state masks needed to fence a
// direct extension.  The masks are coverage facts, not an ordered event log.
// They can overlap after multiple transitions and cannot reconstruct when,
// where or in which segment an event occurred.
//
// "Returned" remains only an exact committed commanded-MCS endpoint relation.
// This summary is not actual Motion execution, physical reversal, travelled
// path, retrace order, a Path Queue, a segment list or a B2 breadcrumb.  It
// stores no coordinate, displacement, endpoint or intermediate-point array.
// It has no Gate, PC, Alarm, Motion, HMI/SHM/API, PDO, EtherCAT or DC consumer.
// Neutral and invalid observations clear all proven payload.  The shadow is
// same-thread, fixed-capacity and heap-resident through NCManager, with no
// allocation, logging, waiting or synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY =
2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_AXIS_MASK =
0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_SCHEMA_V1 =
1U;

namespace NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail
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

    constexpr std::uint32_t FlagCount(
        std::uint8_t flags,
        std::uint8_t flag) noexcept
    {
        return (flags & flag) != 0U ? 1U : 0U;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_TRANSITION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_TRANSITION_NOT_PROVEN = 2U,
    INVALID_CURRENT_TRANSITION_RECORD = 3U,
    INVALID_PREVIOUS_SUMMARY_RECORD = 4U,
    INVALID_TRANSITION_SUMMARY_ADVANCE_FENCE = 5U,
    INVALID_TRANSITION_SUMMARY_ACCUMULATION = 6U,
    PROVEN_RETURN_TRANSITION_SUMMARY_STARTED = 7U,
    PROVEN_RETURN_TRANSITION_SUMMARY_EXTENDED = 8U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation :
    std::uint8_t
{
    NONE = 0U,
    FIRST_PROVEN_TRANSITION_ESTABLISHES_COVERAGE = 1U,
    DIRECT_PROVEN_TRANSITION_EXTENDS_COVERAGE = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent :
    std::uint8_t
{
    NONE = 0U,
    SCALAR_RETURN_TRANSITION_EVENT_COVERAGE_ONLY = 1U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
#endif

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourceTransitionPublicationSequence = 0ULL;
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
    std::int32_t firstSourcePC = -1;
    std::int32_t coverageStartPreviousLatestSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition
        disposition =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
        EMPTY;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation relation =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent extent =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::NONE;
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
    std::array<std::uint8_t, 6U> reserved{};

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        bool IsProvenRunEndpointReturnTransitionCoverageSummary() const noexcept
    {
        const bool started =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_STARTED &&
            relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            FIRST_PROVEN_TRANSITION_ESTABLISHES_COVERAGE;
        const bool extended =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_EXTENDED &&
            relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            DIRECT_PROVEN_TRANSITION_EXTENDS_COVERAGE;

        if (!BasicProofFieldsAreValid(started, extended) ||
            !CoverageSpansAndSequencesAreValid() ||
            !FullClosureBalanceIsValid())
        {
            return false;
        }

        if (started)
        {
            return IsValidSingleTransitionCoverage();
        }

        return provenTransitionCount >= 2U &&
            firstTransitionPublicationSequence !=
            latestTransitionPublicationSequence &&
            firstClosurePublicationSequence != latestClosurePublicationSequence &&
            ExtendedCoverageBoundaryFactsAreRepresented();
    }

private:
    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        bool BasicProofFieldsAreValid(
            bool started,
            bool extended) const noexcept
    {
        return publicationSequence != 0ULL &&
            sourceTransitionPublicationSequence != 0ULL &&
            sourceTransitionPublicationSequence ==
            latestTransitionPublicationSequence &&
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
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_AXIS_MASK) ==
            0U &&
            (coordinateChangeAxisUnionMask & ~participatingAxisUnionMask) == 0U &&
            CoverageMasksAreValid() &&
            EndpointStateMasksAreValid(
                coverageStartEndpointOpenAxisMask,
                coverageStartEndpointReturnedAxisMask,
                false) &&
            EndpointStateMasksAreValid(
                latestEndpointOpenAxisMask,
                latestEndpointReturnedAxisMask,
                true) &&
            ClosureClassMatchesState(
                coverageStartEndpointOpenAxisMask,
                coverageStartEndpointReturnedAxisMask,
                firstClosureClass) &&
            ClosureClassMatchesState(
                latestEndpointOpenAxisMask,
                latestEndpointReturnedAxisMask,
                latestClosureClass) &&
            fullClosureEntryCount <= provenTransitionCount &&
            fullClosureExitCount <= provenTransitionCount &&
            fullClosureRetentionCount <= provenTransitionCount &&
            static_cast<std::uint64_t>(fullClosureEntryCount) +
            static_cast<std::uint64_t>(fullClosureExitCount) +
            static_cast<std::uint64_t>(fullClosureRetentionCount) <=
            static_cast<std::uint64_t>(provenTransitionCount) &&
            firstSourcePC >= 0 &&
            coverageStartPreviousLatestSourcePC >= firstSourcePC &&
            latestSourcePC > coverageStartPreviousLatestSourcePC &&
            schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_SCHEMA_V1 &&
            (started || extended) &&
            extent ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
            SCALAR_RETURN_TRANSITION_EVENT_COVERAGE_ONLY &&
            frame == NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE &&
            kind == NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR &&
            commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP &&
            ReservedIsZero();
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        bool CoverageSpansAndSequencesAreValid() const noexcept
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
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            ForwardNonZeroSequenceDistance(
                firstTransitionPublicationSequence,
                latestTransitionPublicationSequence) ==
            transitionCount - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            ForwardNonZeroSequenceDistance(
                firstClosurePublicationSequence,
                latestClosurePublicationSequence) == transitionCount &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) == runLength;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        bool FullClosureBalanceIsValid() const noexcept
    {
        const bool firstFull =
            firstClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool latestFull =
            latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const std::int64_t fullClosureBalance =
            static_cast<std::int64_t>(fullClosureEntryCount) -
            static_cast<std::int64_t>(fullClosureExitCount);
        const std::int64_t expectedFullClosureBalance =
            !firstFull && latestFull
            ? 1LL
            : (firstFull && !latestFull ? -1LL : 0LL);
        return fullClosureBalance == expectedFullClosureBalance;
    }

    bool CoverageMasksAreValid() const noexcept
    {
        const std::uint32_t allCoverageMasks =
            cumulativeNewlyOpenedAxisMask |
            cumulativeNewlyReturnedAxisMask |
            cumulativeReopenedAxisMask |
            cumulativeRetainedOpenAxisMask |
            cumulativeRetainedReturnedAxisMask;
        return
            (allCoverageMasks &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_AXIS_MASK) ==
            0U &&
            (allCoverageMasks & ~coordinateChangeAxisUnionMask) == 0U &&
            allCoverageMasks == coordinateChangeAxisUnionMask;
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

    bool IsValidSingleTransitionCoverage() const noexcept
    {
        const std::uint32_t startCoordinateChangeAxisMask =
            coverageStartEndpointOpenAxisMask |
            coverageStartEndpointReturnedAxisMask;
        const std::uint32_t expectedNewlyOpenedAxisMask =
            latestEndpointOpenAxisMask & ~startCoordinateChangeAxisMask;
        const std::uint32_t expectedNewlyReturnedAxisMask =
            latestEndpointReturnedAxisMask &
            coverageStartEndpointOpenAxisMask;
        const std::uint32_t expectedReopenedAxisMask =
            latestEndpointOpenAxisMask &
            coverageStartEndpointReturnedAxisMask;
        const std::uint32_t expectedRetainedOpenAxisMask =
            latestEndpointOpenAxisMask &
            coverageStartEndpointOpenAxisMask;
        const std::uint32_t expectedRetainedReturnedAxisMask =
            latestEndpointReturnedAxisMask &
            coverageStartEndpointReturnedAxisMask;
        const std::uint32_t unexpectedNewReturnedAxisMask =
            latestEndpointReturnedAxisMask & ~startCoordinateChangeAxisMask;

        const bool firstFull =
            firstClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool latestFull =
            latestClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;

        return provenTransitionCount == 1U &&
            firstTransitionPublicationSequence ==
            latestTransitionPublicationSequence &&
            firstClosurePublicationSequence != latestClosurePublicationSequence &&
            latestClosurePublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(firstClosurePublicationSequence) &&
            latestLinkedSegmentRunLength ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            SaturatingIncrement(coverageStartLinkedSegmentRunLength) &&
            unexpectedNewReturnedAxisMask == 0U &&
            cumulativeNewlyOpenedAxisMask == expectedNewlyOpenedAxisMask &&
            cumulativeNewlyReturnedAxisMask == expectedNewlyReturnedAxisMask &&
            cumulativeReopenedAxisMask == expectedReopenedAxisMask &&
            cumulativeRetainedOpenAxisMask == expectedRetainedOpenAxisMask &&
            cumulativeRetainedReturnedAxisMask ==
            expectedRetainedReturnedAxisMask &&
            fullClosureEntryCount ==
            static_cast<std::uint32_t>(!firstFull && latestFull ? 1U : 0U) &&
            fullClosureExitCount ==
            static_cast<std::uint32_t>(firstFull && !latestFull ? 1U : 0U) &&
            fullClosureRetentionCount ==
            static_cast<std::uint32_t>(firstFull && latestFull ? 1U : 0U);
    }

    bool ExtendedCoverageBoundaryFactsAreRepresented() const noexcept
    {
        const std::uint32_t startCoordinateChangeAxisMask =
            coverageStartEndpointOpenAxisMask |
            coverageStartEndpointReturnedAxisMask;
        const std::uint32_t axesIntroducedAfterCoverageStart =
            coordinateChangeAxisUnionMask & ~startCoordinateChangeAxisMask;

        return
            (axesIntroducedAfterCoverageStart &
                ~cumulativeNewlyOpenedAxisMask) == 0U &&
            (coverageStartEndpointOpenAxisMask &
                ~(cumulativeNewlyReturnedAxisMask |
                    cumulativeRetainedOpenAxisMask)) == 0U &&
            (coverageStartEndpointReturnedAxisMask &
                ~(cumulativeReopenedAxisMask |
                    cumulativeRetainedReturnedAxisMask)) == 0U &&
            (latestEndpointOpenAxisMask &
                ~(cumulativeNewlyOpenedAxisMask |
                    cumulativeReopenedAxisMask |
                    cumulativeRetainedOpenAxisMask)) == 0U &&
            (latestEndpointReturnedAxisMask &
                ~(cumulativeNewlyReturnedAxisMask |
                    cumulativeRetainedReturnedAxisMask)) == 0U;
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

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow() noexcept =
        default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        void ObserveLatestTransitionSameThread(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1*
            transition) noexcept
    {
        const
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1*
            const previousSummary = GetNewestObservationSameThread();

        m_publicationSequence =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY;

        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            target = m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            transition == nullptr ? 0ULL : transition->publicationSequence);

        if (transition == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                NOT_APPLICABLE_TRANSITION_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenTransition(*transition))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                NOT_APPLICABLE_TRANSITION_NOT_PROVEN;
        }
        else if (!transition->IsProvenImmediateRunEndpointReturnTransition())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                INVALID_CURRENT_TRANSITION_RECORD;
        }
        else if (previousSummary == nullptr ||
            IsStructurallyValidNonProvenSummary(*previousSummary))
        {
            StartCoverage(*transition, target);
        }
        else if (!previousSummary->
            IsProvenRunEndpointReturnTransitionCoverageSummary())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                INVALID_PREVIOUS_SUMMARY_RECORD;
        }
        else if (!IsDirectCoverageExtensionFenceValid(
            *previousSummary,
            *transition))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                INVALID_TRANSITION_SUMMARY_ADVANCE_FENCE;
        }
        else
        {
            ExtendCoverage(*previousSummary, *transition, target);
        }

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1*
        GetNewestObservationSameThread(
            std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
        target,
        std::uint64_t publicationSequence,
        std::uint64_t sourceTransitionPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceTransitionPublicationSequence =
            sourceTransitionPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            NOT_APPLICABLE_TRANSITION_UNAVAILABLE;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
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
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
        target) noexcept
    {
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
        target.firstSourcePC = -1;
        target.coverageStartPreviousLatestSourcePC = -1;
        target.latestSourcePC = -1;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
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

    static bool TransitionReservedIsZero(
        const std::array<std::uint8_t, 5U>& reserved) noexcept
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        static bool IsStructurallyValidNonProvenTransition(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            transition) noexcept
    {
        const bool knownDisposition =
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            NOT_APPLICABLE_CLOSURE_UNAVAILABLE ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            NOT_APPLICABLE_CLOSURE_NOT_PROVEN ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            NOT_APPLICABLE_RUN_TRANSITION_BASELINE ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            INVALID_CURRENT_CLOSURE_RECORD ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            INVALID_PREVIOUS_CLOSURE_RECORD ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            INVALID_CLOSURE_ADVANCE_FENCE ||
            transition.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            INVALID_RETURN_TRANSITION_CLASSIFICATION;

        return knownDisposition &&
            transition.publicationSequence != 0ULL &&
            transition.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SCHEMA_V1 &&
            transition.runGeneration == 0ULL &&
            transition.acceptedInputChainGeneration == 0ULL &&
            transition.firstSegmentPublicationSequence == 0ULL &&
            transition.latestSegmentPublicationSequence == 0ULL &&
            transition.firstLinkPublicationSequence == 0ULL &&
            transition.latestLinkPublicationSequence == 0ULL &&
            transition.runStartGeometryPublicationSequence == 0ULL &&
            transition.runEndGeometryPublicationSequence == 0ULL &&
            transition.runStartGeometryMotionSegmentId == 0ULL &&
            transition.runEndGeometryMotionSegmentId == 0ULL &&
            transition.previousLinkedSegmentRunLength == 0U &&
            transition.currentLinkedSegmentRunLength == 0U &&
            transition.previousParticipatingAxisUnionMask == 0U &&
            transition.currentParticipatingAxisUnionMask == 0U &&
            transition.previousCoordinateChangeAxisUnionMask == 0U &&
            transition.currentCoordinateChangeAxisUnionMask == 0U &&
            transition.previousEndpointOpenAxisMask == 0U &&
            transition.currentEndpointOpenAxisMask == 0U &&
            transition.previousEndpointReturnedAxisMask == 0U &&
            transition.currentEndpointReturnedAxisMask == 0U &&
            transition.newlyOpenedAxisMask == 0U &&
            transition.newlyReturnedAxisMask == 0U &&
            transition.reopenedAxisMask == 0U &&
            transition.retainedOpenAxisMask == 0U &&
            transition.retainedReturnedAxisMask == 0U &&
            transition.firstSourcePC == -1 &&
            transition.previousLatestSourcePC == -1 &&
            transition.currentLatestSourcePC == -1 &&
            transition.relation ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::NONE &&
            transition.extent ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::NONE &&
            transition.closureTransitionFlags == 0U &&
            transition.previousClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            transition.currentClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            transition.frame == NCPathCoreCommittedGeometryFrame::NONE &&
            transition.kind == NCPathCoreCommittedGeometryKind::NONE &&
            transition.commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE &&
            TransitionReservedIsZero(transition.reserved);
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        static bool IsStructurallyValidNonProvenSummary(
            const
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        static bool IsDirectCoverageExtensionFenceValid(
            const
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            current) noexcept
    {
        return
            current.publicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(previous.latestTransitionPublicationSequence) &&
            current.previousClosurePublicationSequence ==
            previous.latestClosurePublicationSequence &&
            current.currentClosurePublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(current.previousClosurePublicationSequence) &&
            current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration ==
            previous.acceptedInputChainGeneration &&
            current.firstSegmentPublicationSequence ==
            previous.firstSegmentPublicationSequence &&
            current.latestSegmentPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(previous.latestSegmentPublicationSequence) &&
            current.firstLinkPublicationSequence ==
            previous.firstLinkPublicationSequence &&
            current.latestLinkPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(previous.latestLinkPublicationSequence) &&
            current.runStartGeometryPublicationSequence ==
            previous.runStartGeometryPublicationSequence &&
            current.runEndGeometryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(previous.runEndGeometryPublicationSequence) &&
            current.runStartGeometryMotionSegmentId ==
            previous.runStartGeometryMotionSegmentId &&
            current.runEndGeometryMotionSegmentId >
            previous.runEndGeometryMotionSegmentId &&
            current.previousLinkedSegmentRunLength ==
            previous.latestLinkedSegmentRunLength &&
            current.currentLinkedSegmentRunLength ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            SaturatingIncrement(previous.latestLinkedSegmentRunLength) &&
            current.previousParticipatingAxisUnionMask ==
            previous.participatingAxisUnionMask &&
            (current.previousParticipatingAxisUnionMask &
                ~current.currentParticipatingAxisUnionMask) == 0U &&
            current.previousCoordinateChangeAxisUnionMask ==
            previous.coordinateChangeAxisUnionMask &&
            (current.previousCoordinateChangeAxisUnionMask &
                ~current.currentCoordinateChangeAxisUnionMask) == 0U &&
            current.previousEndpointOpenAxisMask ==
            previous.latestEndpointOpenAxisMask &&
            current.previousEndpointReturnedAxisMask ==
            previous.latestEndpointReturnedAxisMask &&
            current.previousClosureClass == previous.latestClosureClass &&
            current.firstSourcePC == previous.firstSourcePC &&
            current.previousLatestSourcePC == previous.latestSourcePC &&
            previous.latestSourcePC !=
            (std::numeric_limits<std::int32_t>::max)() &&
            current.currentLatestSourcePC == previous.latestSourcePC + 1 &&
            current.frame == previous.frame &&
            current.kind == previous.kind &&
            current.commandPathPolicy == previous.commandPathPolicy;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        void StartCoverage(
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            transition,
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            target) noexcept
    {
        const std::uint64_t candidateCoverageGeneration =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            NextNonZeroSequence(m_coverageGenerationSequence);

        target.coverageGeneration = candidateCoverageGeneration;
        target.runGeneration = transition.runGeneration;
        target.acceptedInputChainGeneration =
            transition.acceptedInputChainGeneration;
        target.firstTransitionPublicationSequence =
            transition.publicationSequence;
        target.latestTransitionPublicationSequence =
            transition.publicationSequence;
        target.firstClosurePublicationSequence =
            transition.previousClosurePublicationSequence;
        target.latestClosurePublicationSequence =
            transition.currentClosurePublicationSequence;
        target.firstSegmentPublicationSequence =
            transition.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            transition.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            transition.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            transition.latestLinkPublicationSequence;
        target.runStartGeometryPublicationSequence =
            transition.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            transition.runEndGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            transition.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            transition.runEndGeometryMotionSegmentId;
        target.provenTransitionCount = 1U;
        target.coverageStartLinkedSegmentRunLength =
            transition.previousLinkedSegmentRunLength;
        target.latestLinkedSegmentRunLength =
            transition.currentLinkedSegmentRunLength;
        target.participatingAxisUnionMask =
            transition.currentParticipatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            transition.currentCoordinateChangeAxisUnionMask;
        target.cumulativeNewlyOpenedAxisMask =
            transition.newlyOpenedAxisMask;
        target.cumulativeNewlyReturnedAxisMask =
            transition.newlyReturnedAxisMask;
        target.cumulativeReopenedAxisMask =
            transition.reopenedAxisMask;
        target.cumulativeRetainedOpenAxisMask =
            transition.retainedOpenAxisMask;
        target.cumulativeRetainedReturnedAxisMask =
            transition.retainedReturnedAxisMask;
        target.coverageStartEndpointOpenAxisMask =
            transition.previousEndpointOpenAxisMask;
        target.coverageStartEndpointReturnedAxisMask =
            transition.previousEndpointReturnedAxisMask;
        target.latestEndpointOpenAxisMask =
            transition.currentEndpointOpenAxisMask;
        target.latestEndpointReturnedAxisMask =
            transition.currentEndpointReturnedAxisMask;
        target.fullClosureEntryCount =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            FlagCount(
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL);
        target.fullClosureExitCount =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            FlagCount(
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL);
        target.fullClosureRetentionCount =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            FlagCount(
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL);
        target.firstSourcePC = transition.firstSourcePC;
        target.coverageStartPreviousLatestSourcePC =
            transition.previousLatestSourcePC;
        target.latestSourcePC = transition.currentLatestSourcePC;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_STARTED;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            FIRST_PROVEN_TRANSITION_ESTABLISHES_COVERAGE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
            SCALAR_RETURN_TRANSITION_EVENT_COVERAGE_ONLY;
        target.firstClosureClass = transition.previousClosureClass;
        target.latestClosureClass = transition.currentClosureClass;
        target.frame = transition.frame;
        target.kind = transition.kind;
        target.commandPathPolicy = transition.commandPathPolicy;

        if (target.IsProvenRunEndpointReturnTransitionCoverageSummary())
        {
            m_coverageGenerationSequence = candidateCoverageGeneration;
        }
        else
        {
            ClearProvenPayload(target);
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                INVALID_TRANSITION_SUMMARY_ACCUMULATION;
        }
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE
        static void ExtendCoverage(
            const
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            transition,
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1&
            target) noexcept
    {
        target.coverageGeneration = previous.coverageGeneration;
        target.runGeneration = previous.runGeneration;
        target.acceptedInputChainGeneration =
            previous.acceptedInputChainGeneration;
        target.firstTransitionPublicationSequence =
            previous.firstTransitionPublicationSequence;
        target.latestTransitionPublicationSequence =
            transition.publicationSequence;
        target.firstClosurePublicationSequence =
            previous.firstClosurePublicationSequence;
        target.latestClosurePublicationSequence =
            transition.currentClosurePublicationSequence;
        target.firstSegmentPublicationSequence =
            previous.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            transition.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            previous.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            transition.latestLinkPublicationSequence;
        target.runStartGeometryPublicationSequence =
            previous.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            transition.runEndGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            previous.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            transition.runEndGeometryMotionSegmentId;
        target.provenTransitionCount =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            SaturatingIncrement(previous.provenTransitionCount);
        target.coverageStartLinkedSegmentRunLength =
            previous.coverageStartLinkedSegmentRunLength;
        target.latestLinkedSegmentRunLength =
            transition.currentLinkedSegmentRunLength;
        target.participatingAxisUnionMask =
            transition.currentParticipatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            transition.currentCoordinateChangeAxisUnionMask;
        target.cumulativeNewlyOpenedAxisMask =
            previous.cumulativeNewlyOpenedAxisMask |
            transition.newlyOpenedAxisMask;
        target.cumulativeNewlyReturnedAxisMask =
            previous.cumulativeNewlyReturnedAxisMask |
            transition.newlyReturnedAxisMask;
        target.cumulativeReopenedAxisMask =
            previous.cumulativeReopenedAxisMask |
            transition.reopenedAxisMask;
        target.cumulativeRetainedOpenAxisMask =
            previous.cumulativeRetainedOpenAxisMask |
            transition.retainedOpenAxisMask;
        target.cumulativeRetainedReturnedAxisMask =
            previous.cumulativeRetainedReturnedAxisMask |
            transition.retainedReturnedAxisMask;
        target.coverageStartEndpointOpenAxisMask =
            previous.coverageStartEndpointOpenAxisMask;
        target.coverageStartEndpointReturnedAxisMask =
            previous.coverageStartEndpointReturnedAxisMask;
        target.latestEndpointOpenAxisMask =
            transition.currentEndpointOpenAxisMask;
        target.latestEndpointReturnedAxisMask =
            transition.currentEndpointReturnedAxisMask;
        target.fullClosureEntryCount =
            AddFlagCount(
                previous.fullClosureEntryCount,
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL);
        target.fullClosureExitCount =
            AddFlagCount(
                previous.fullClosureExitCount,
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL);
        target.fullClosureRetentionCount =
            AddFlagCount(
                previous.fullClosureRetentionCount,
                transition.closureTransitionFlags,
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL);
        target.firstSourcePC = previous.firstSourcePC;
        target.coverageStartPreviousLatestSourcePC =
            previous.coverageStartPreviousLatestSourcePC;
        target.latestSourcePC = transition.currentLatestSourcePC;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
            PROVEN_RETURN_TRANSITION_SUMMARY_EXTENDED;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation::
            DIRECT_PROVEN_TRANSITION_EXTENDS_COVERAGE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent::
            SCALAR_RETURN_TRANSITION_EVENT_COVERAGE_ONLY;
        target.firstClosureClass = previous.firstClosureClass;
        target.latestClosureClass = transition.currentClosureClass;
        target.frame = transition.frame;
        target.kind = transition.kind;
        target.commandPathPolicy = transition.commandPathPolicy;

        if (!target.IsProvenRunEndpointReturnTransitionCoverageSummary())
        {
            ClearProvenPayload(target);
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition::
                INVALID_TRANSITION_SUMMARY_ACCUMULATION;
        }
    }

    static std::uint32_t AddFlagCount(
        std::uint32_t current,
        std::uint8_t flags,
        std::uint8_t flag) noexcept
    {
        return (flags & flag) != 0U
            ? NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDetail::
            SaturatingIncrement(current)
            : current;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint64_t m_coverageGenerationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SUMMARY_NOINLINE

static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryDisposition) ==
    1U,
    "Run closure-transition summary disposition must remain one byte.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRelation) ==
    1U,
    "Run closure-transition summary relation must remain one byte.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryExtent) == 1U,
    "Run closure-transition summary extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1>::value,
    "Run closure-transition summary record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1>::value,
    "Run closure-transition summary record must remain trivially copyable.");
static_assert(
    alignof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1) ==
    8U,
    "Run closure-transition summary record alignment changed.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1) ==
    232U,
    "Run closure-transition summary record must remain exactly 232 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1,
        provenTransitionCount) == 136U,
    "Run closure-transition summary mask block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1,
        firstSourcePC) == 204U,
    "Run closure-transition summary source-PC block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1,
        schemaVersion) == 216U,
    "Run closure-transition summary schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryRecordV1,
        reserved) == 226U,
    "Run closure-transition summary reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow>::value,
    "Run closure-transition summary shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow>::value,
    "Run closure-transition summary shadow must remain trivially copyable.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow) ==
    488U,
    "Run closure-transition summary shadow must remain exactly 488 bytes.");
