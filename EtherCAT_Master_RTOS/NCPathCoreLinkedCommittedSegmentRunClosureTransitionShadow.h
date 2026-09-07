#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunClosureShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2L / Proven Linked Committed Segment Run Endpoint-
// Return Transition Relation Shadow
//
// This fixed two-record observer consumes only the immediately adjacent
// newest two L.2K endpoint-closure observations.  It proves how the scalar
// commanded MCS endpoint state changed when the same linked committed run
// extended by one segment.
//
// Per axis, the proven relation distinguishes only these endpoint states:
//   * not yet coordinate-changed in the run,
//   * endpoint open from the run head, and
//   * endpoint numerically returned to the run head after a commanded change.
//
// The output stores scalar provenance and five transition masks:
//   * newly-opened: unchanged -> open,
//   * newly-returned: open -> returned,
//   * reopened: returned -> open,
//   * retained-open: open -> open, and
//   * retained-returned: returned -> returned.
//
// Full-closure entry, exit and retention are scalar flags derived from those
// masks.  "Returned" remains only an exact committed commanded-endpoint
// relation.  It is not actual Motion execution, physical reversal, travelled
// path, a closed interpolated contour, retrace order or a B2 breadcrumb.
//
// L.2L stores no endpoint/displacement array, intermediate point or segment
// list.  It is not a Path Queue and has no Gate, PC, Alarm, Motion,
// HMI/SHM/API, PDO, EtherCAT or DC consumer.  Neutral and invalid observations
// clear all proven payload.  The shadow is same-thread, fixed-capacity and
// heap-resident through NCManager, with no allocation, logging, waiting or
// synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY =
2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SCHEMA_V1 = 1U;
constexpr std::uint8_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL =
0x01U;
constexpr std::uint8_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL =
0x02U;
constexpr std::uint8_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL =
0x04U;
constexpr std::uint8_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_FLAG_MASK =
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL |
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL;

namespace NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail
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
            : ((std::numeric_limits<std::uint64_t>::max)() - first) +
            latest;
    }

    constexpr std::uint32_t SaturatingIncrement(
        std::uint32_t value) noexcept
    {
        return value == (std::numeric_limits<std::uint32_t>::max)()
            ? value
            : value + 1U;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CLOSURE_UNAVAILABLE = 1U,
    NOT_APPLICABLE_CLOSURE_NOT_PROVEN = 2U,
    NOT_APPLICABLE_RUN_TRANSITION_BASELINE = 3U,
    INVALID_CURRENT_CLOSURE_RECORD = 4U,
    INVALID_PREVIOUS_CLOSURE_RECORD = 5U,
    INVALID_CLOSURE_ADVANCE_FENCE = 6U,
    INVALID_RETURN_TRANSITION_CLASSIFICATION = 7U,
    PROVEN_IMMEDIATE_RUN_ENDPOINT_RETURN_TRANSITION = 8U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation :
    std::uint8_t
{
    NONE = 0U,
    PROVEN_ADJACENT_CLOSURE_STATE_TRANSITION = 1U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent :
    std::uint8_t
{
    NONE = 0U,
    IMMEDIATE_RUN_EXTENSION_ENDPOINT_STATE_MASKS_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousClosurePublicationSequence = 0ULL;
    std::uint64_t currentClosurePublicationSequence = 0ULL;
    std::uint64_t runGeneration = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t firstSegmentPublicationSequence = 0ULL;
    std::uint64_t latestSegmentPublicationSequence = 0ULL;
    std::uint64_t firstLinkPublicationSequence = 0ULL;
    std::uint64_t latestLinkPublicationSequence = 0ULL;
    std::uint64_t runStartGeometryPublicationSequence = 0ULL;
    std::uint64_t runEndGeometryPublicationSequence = 0ULL;
    std::uint64_t runStartGeometryMotionSegmentId = 0ULL;
    std::uint64_t runEndGeometryMotionSegmentId = 0ULL;

    std::uint32_t previousLinkedSegmentRunLength = 0U;
    std::uint32_t currentLinkedSegmentRunLength = 0U;
    std::uint32_t previousParticipatingAxisUnionMask = 0U;
    std::uint32_t currentParticipatingAxisUnionMask = 0U;
    std::uint32_t previousCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t currentCoordinateChangeAxisUnionMask = 0U;
    std::uint32_t previousEndpointOpenAxisMask = 0U;
    std::uint32_t currentEndpointOpenAxisMask = 0U;
    std::uint32_t previousEndpointReturnedAxisMask = 0U;
    std::uint32_t currentEndpointReturnedAxisMask = 0U;
    std::uint32_t newlyOpenedAxisMask = 0U;
    std::uint32_t newlyReturnedAxisMask = 0U;
    std::uint32_t reopenedAxisMask = 0U;
    std::uint32_t retainedOpenAxisMask = 0U;
    std::uint32_t retainedReturnedAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t previousLatestSourcePC = -1;
    std::int32_t currentLatestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition
        disposition =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation relation =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent extent =
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::NONE;
    std::uint8_t closureTransitionFlags = 0U;
    NCPathCoreLinkedCommittedSegmentRunClosureClass previousClosureClass =
        NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureClass currentClosureClass =
        NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 5U> reserved{};

    bool IsProvenImmediateRunEndpointReturnTransition() const noexcept
    {
        if (publicationSequence == 0ULL ||
            previousClosurePublicationSequence == 0ULL ||
            currentClosurePublicationSequence == 0ULL ||
            currentClosurePublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previousClosurePublicationSequence) ||
            runGeneration == 0ULL ||
            acceptedInputChainGeneration == 0ULL ||
            firstSegmentPublicationSequence == 0ULL ||
            latestSegmentPublicationSequence == 0ULL ||
            firstLinkPublicationSequence == 0ULL ||
            latestLinkPublicationSequence == 0ULL ||
            runStartGeometryPublicationSequence == 0ULL ||
            runEndGeometryPublicationSequence == 0ULL ||
            runStartGeometryMotionSegmentId == 0ULL ||
            runEndGeometryMotionSegmentId <=
            runStartGeometryMotionSegmentId ||
            previousLinkedSegmentRunLength < 2U ||
            currentLinkedSegmentRunLength < 3U ||
            currentLinkedSegmentRunLength !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            SaturatingIncrement(previousLinkedSegmentRunLength) ||
            previousParticipatingAxisUnionMask == 0U ||
            currentParticipatingAxisUnionMask == 0U ||
            (previousParticipatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_AXIS_MASK) !=
            0U ||
            (currentParticipatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_AXIS_MASK) !=
            0U ||
            (previousParticipatingAxisUnionMask &
                ~currentParticipatingAxisUnionMask) != 0U ||
            (previousCoordinateChangeAxisUnionMask &
                ~previousParticipatingAxisUnionMask) != 0U ||
            (currentCoordinateChangeAxisUnionMask &
                ~currentParticipatingAxisUnionMask) != 0U ||
            (previousCoordinateChangeAxisUnionMask &
                ~currentCoordinateChangeAxisUnionMask) != 0U ||
            !StateMasksAreValid(
                previousCoordinateChangeAxisUnionMask,
                previousEndpointOpenAxisMask,
                previousEndpointReturnedAxisMask) ||
            !StateMasksAreValid(
                currentCoordinateChangeAxisUnionMask,
                currentEndpointOpenAxisMask,
                currentEndpointReturnedAxisMask) ||
            !TransitionMasksAreValid() ||
            firstSourcePC < 0 ||
            previousLatestSourcePC < firstSourcePC ||
            previousLatestSourcePC ==
            (std::numeric_limits<std::int32_t>::max)() ||
            currentLatestSourcePC != previousLatestSourcePC + 1 ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SCHEMA_V1 ||
            disposition !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            PROVEN_IMMEDIATE_RUN_ENDPOINT_RETURN_TRANSITION ||
            relation !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::
            PROVEN_ADJACENT_CLOSURE_STATE_TRANSITION ||
            extent !=
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::
            IMMEDIATE_RUN_EXTENSION_ENDPOINT_STATE_MASKS_ONLY ||
            (closureTransitionFlags &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_FLAG_MASK) !=
            0U ||
            !ClosureClassMatchesState(
                previousCoordinateChangeAxisUnionMask,
                previousEndpointOpenAxisMask,
                previousEndpointReturnedAxisMask,
                previousClosureClass) ||
            !ClosureClassMatchesState(
                currentCoordinateChangeAxisUnionMask,
                currentEndpointOpenAxisMask,
                currentEndpointReturnedAxisMask,
                currentClosureClass) ||
            closureTransitionFlags != ExpectedClosureTransitionFlags() ||
            frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
            kind != NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR ||
            commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP ||
            !ReservedIsZero())
        {
            return false;
        }

        const std::int64_t sourcePcSpan =
            static_cast<std::int64_t>(currentLatestSourcePC) -
            static_cast<std::int64_t>(firstSourcePC) + 1LL;
        if (sourcePcSpan <= 0LL ||
            static_cast<std::uint64_t>(sourcePcSpan) !=
            static_cast<std::uint64_t>(currentLinkedSegmentRunLength))
        {
            return false;
        }

        const std::uint64_t runLength =
            static_cast<std::uint64_t>(currentLinkedSegmentRunLength);
        return
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) == runLength - 1ULL &&
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) == runLength;
    }

private:
    static bool StateMasksAreValid(
        std::uint32_t coordinateChangeAxisUnionMask,
        std::uint32_t endpointOpenAxisMask,
        std::uint32_t endpointReturnedAxisMask) noexcept
    {
        return
            (coordinateChangeAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_AXIS_MASK) ==
            0U &&
            (endpointOpenAxisMask & ~coordinateChangeAxisUnionMask) == 0U &&
            (endpointReturnedAxisMask & ~coordinateChangeAxisUnionMask) == 0U &&
            (endpointOpenAxisMask & endpointReturnedAxisMask) == 0U &&
            (endpointOpenAxisMask | endpointReturnedAxisMask) ==
            coordinateChangeAxisUnionMask;
    }

    static bool ClosureClassMatchesState(
        std::uint32_t coordinateChangeAxisUnionMask,
        std::uint32_t endpointOpenAxisMask,
        std::uint32_t endpointReturnedAxisMask,
        NCPathCoreLinkedCommittedSegmentRunClosureClass closureClass) noexcept
    {
        if (coordinateChangeAxisUnionMask == 0U)
        {
            return endpointOpenAxisMask == 0U &&
                endpointReturnedAxisMask == 0U &&
                closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                NO_COMMANDED_COORDINATE_CHANGE;
        }
        if (endpointOpenAxisMask == 0U)
        {
            return endpointReturnedAxisMask == coordinateChangeAxisUnionMask &&
                closureClass ==
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

    bool TransitionMasksAreValid() const noexcept
    {
        const std::uint32_t allTransitionMasks =
            newlyOpenedAxisMask |
            newlyReturnedAxisMask |
            reopenedAxisMask |
            retainedOpenAxisMask |
            retainedReturnedAxisMask;
        const std::uint32_t pairwiseOverlap =
            (newlyOpenedAxisMask & newlyReturnedAxisMask) |
            (newlyOpenedAxisMask & reopenedAxisMask) |
            (newlyOpenedAxisMask & retainedOpenAxisMask) |
            (newlyOpenedAxisMask & retainedReturnedAxisMask) |
            (newlyReturnedAxisMask & reopenedAxisMask) |
            (newlyReturnedAxisMask & retainedOpenAxisMask) |
            (newlyReturnedAxisMask & retainedReturnedAxisMask) |
            (reopenedAxisMask & retainedOpenAxisMask) |
            (reopenedAxisMask & retainedReturnedAxisMask) |
            (retainedOpenAxisMask & retainedReturnedAxisMask);

        return
            pairwiseOverlap == 0U &&
            (allTransitionMasks &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_AXIS_MASK) ==
            0U &&
            newlyOpenedAxisMask ==
            (currentEndpointOpenAxisMask &
                ~previousCoordinateChangeAxisUnionMask) &&
            newlyReturnedAxisMask ==
            (currentEndpointReturnedAxisMask &
                previousEndpointOpenAxisMask) &&
            reopenedAxisMask ==
            (currentEndpointOpenAxisMask &
                previousEndpointReturnedAxisMask) &&
            retainedOpenAxisMask ==
            (currentEndpointOpenAxisMask &
                previousEndpointOpenAxisMask) &&
            retainedReturnedAxisMask ==
            (currentEndpointReturnedAxisMask &
                previousEndpointReturnedAxisMask) &&
            previousEndpointOpenAxisMask ==
            (newlyReturnedAxisMask | retainedOpenAxisMask) &&
            previousEndpointReturnedAxisMask ==
            (reopenedAxisMask | retainedReturnedAxisMask) &&
            currentEndpointOpenAxisMask ==
            (newlyOpenedAxisMask |
                reopenedAxisMask |
                retainedOpenAxisMask) &&
            currentEndpointReturnedAxisMask ==
            (newlyReturnedAxisMask | retainedReturnedAxisMask) &&
            currentCoordinateChangeAxisUnionMask == allTransitionMasks;
    }

    std::uint8_t ExpectedClosureTransitionFlags() const noexcept
    {
        const bool previousFull =
            previousClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool currentFull =
            currentClosureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        return static_cast<std::uint8_t>(
            (!previousFull && currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL
                : 0U) |
            (previousFull && !currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL
                : 0U) |
            (previousFull && currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL
                : 0U));
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

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow() noexcept =
        default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
        void ObserveImmediateClosureTransitionSameThread(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
            previousClosure,
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
            currentClosure) noexcept
    {
        m_publicationSequence =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY;

        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1& target =
            m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            previousClosure == nullptr
            ? 0ULL
            : previousClosure->publicationSequence,
            currentClosure == nullptr
            ? 0ULL
            : currentClosure->publicationSequence);

        EvaluateObservation(previousClosure, currentClosure, target);

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1*
        GetNewestObservationSameThread(
            std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
        static void EvaluateObservation(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
            previousClosure,
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
            currentClosure,
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            target) noexcept
    {
        if (currentClosure == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                NOT_APPLICABLE_CLOSURE_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenClosure(*currentClosure))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                NOT_APPLICABLE_CLOSURE_NOT_PROVEN;
        }
        else if (!currentClosure->IsProvenRunEndpointClosureRelation())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                INVALID_CURRENT_CLOSURE_RECORD;
        }
        else if (currentClosure->disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            PROVEN_RUN_ENDPOINT_CLOSURE_STARTED)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                NOT_APPLICABLE_RUN_TRANSITION_BASELINE;
        }
        else if (currentClosure->disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED)
        {
            if (previousClosure == nullptr ||
                !previousClosure->IsProvenRunEndpointClosureRelation())
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                    INVALID_PREVIOUS_CLOSURE_RECORD;
            }
            else if (!IsDirectClosureAdvanceFenceValid(
                *previousClosure,
                *currentClosure))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                    INVALID_CLOSURE_ADVANCE_FENCE;
            }
            else if (!PopulateTransition(
                *previousClosure,
                *currentClosure,
                target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                    INVALID_RETURN_TRANSITION_CLASSIFICATION;
            }
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
                INVALID_CURRENT_CLOSURE_RECORD;
        }
    }

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1& target,
        std::uint64_t publicationSequence,
        std::uint64_t previousClosurePublicationSequence,
        std::uint64_t currentClosurePublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.previousClosurePublicationSequence =
            previousClosurePublicationSequence;
        target.currentClosurePublicationSequence =
            currentClosurePublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            NOT_APPLICABLE_CLOSURE_UNAVAILABLE;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::NONE;
        target.closureTransitionFlags = 0U;
        target.previousClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.currentClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static void ClearProvenPayload(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1& target)
        noexcept
    {
        target.runGeneration = 0ULL;
        target.acceptedInputChainGeneration = 0ULL;
        target.firstSegmentPublicationSequence = 0ULL;
        target.latestSegmentPublicationSequence = 0ULL;
        target.firstLinkPublicationSequence = 0ULL;
        target.latestLinkPublicationSequence = 0ULL;
        target.runStartGeometryPublicationSequence = 0ULL;
        target.runEndGeometryPublicationSequence = 0ULL;
        target.runStartGeometryMotionSegmentId = 0ULL;
        target.runEndGeometryMotionSegmentId = 0ULL;
        target.previousLinkedSegmentRunLength = 0U;
        target.currentLinkedSegmentRunLength = 0U;
        target.previousParticipatingAxisUnionMask = 0U;
        target.currentParticipatingAxisUnionMask = 0U;
        target.previousCoordinateChangeAxisUnionMask = 0U;
        target.currentCoordinateChangeAxisUnionMask = 0U;
        target.previousEndpointOpenAxisMask = 0U;
        target.currentEndpointOpenAxisMask = 0U;
        target.previousEndpointReturnedAxisMask = 0U;
        target.currentEndpointReturnedAxisMask = 0U;
        target.newlyOpenedAxisMask = 0U;
        target.newlyReturnedAxisMask = 0U;
        target.reopenedAxisMask = 0U;
        target.retainedOpenAxisMask = 0U;
        target.retainedReturnedAxisMask = 0U;
        target.firstSourcePC = -1;
        target.previousLatestSourcePC = -1;
        target.currentLatestSourcePC = -1;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::NONE;
        target.closureTransitionFlags = 0U;
        target.previousClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.currentClosureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    }

    static bool ClosureReservedIsZero(
        const std::array<std::uint8_t, 7U>& reserved) noexcept
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
        static bool IsStructurallyValidNonProvenClosure(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& closure)
        noexcept
    {
        const bool knownDisposition =
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            NOT_APPLICABLE_DISPLACEMENT_UNAVAILABLE ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            NOT_APPLICABLE_DISPLACEMENT_NOT_PROVEN ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            INVALID_CURRENT_DISPLACEMENT_RECORD ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            INVALID_PREVIOUS_DISPLACEMENT_RECORD ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            INVALID_PREVIOUS_CLOSURE_RECORD ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            INVALID_DISPLACEMENT_ADVANCE_FENCE ||
            closure.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            INVALID_CLOSURE_CLASSIFICATION;

        return knownDisposition &&
            closure.publicationSequence != 0ULL &&
            closure.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_SCHEMA_V1 &&
            closure.runGeneration == 0ULL &&
            closure.acceptedInputChainGeneration == 0ULL &&
            closure.firstSegmentPublicationSequence == 0ULL &&
            closure.latestSegmentPublicationSequence == 0ULL &&
            closure.firstLinkPublicationSequence == 0ULL &&
            closure.latestLinkPublicationSequence == 0ULL &&
            closure.runStartGeometryPublicationSequence == 0ULL &&
            closure.runEndGeometryPublicationSequence == 0ULL &&
            closure.runStartGeometryMotionSegmentId == 0ULL &&
            closure.runEndGeometryMotionSegmentId == 0ULL &&
            closure.linkedSegmentRunLength == 0U &&
            closure.participatingAxisUnionMask == 0U &&
            closure.coordinateChangeAxisUnionMask == 0U &&
            closure.endpointOpenAxisMask == 0U &&
            closure.endpointReturnedAxisMask == 0U &&
            closure.coordinateUnchangedParticipatingAxisMask == 0U &&
            closure.positiveNetDisplacementAxisMask == 0U &&
            closure.negativeNetDisplacementAxisMask == 0U &&
            closure.firstSourcePC == -1 &&
            closure.latestSourcePC == -1 &&
            closure.closureRelation ==
            NCPathCoreLinkedCommittedSegmentRunClosureRelation::NONE &&
            closure.extent ==
            NCPathCoreLinkedCommittedSegmentRunClosureExtent::NONE &&
            closure.closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE &&
            closure.frame == NCPathCoreCommittedGeometryFrame::NONE &&
            closure.kind == NCPathCoreCommittedGeometryKind::NONE &&
            closure.commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE &&
            ClosureReservedIsZero(closure.reserved);
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
        static bool IsDirectClosureAdvanceFenceValid(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& current)
        noexcept
    {
        return
            current.disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED &&
            current.publicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.publicationSequence) &&
            current.sourceDisplacementPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(
                previous.sourceDisplacementPublicationSequence) &&
            current.sourceBoundaryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.sourceBoundaryPublicationSequence) &&
            current.sourceRunPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration ==
            previous.acceptedInputChainGeneration &&
            current.firstSegmentPublicationSequence ==
            previous.firstSegmentPublicationSequence &&
            current.latestSegmentPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.latestSegmentPublicationSequence) &&
            current.firstLinkPublicationSequence ==
            previous.firstLinkPublicationSequence &&
            current.latestLinkPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.latestLinkPublicationSequence) &&
            current.runStartGeometryPublicationSequence ==
            previous.runStartGeometryPublicationSequence &&
            current.runEndGeometryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            NextNonZeroSequence(previous.runEndGeometryPublicationSequence) &&
            current.runStartGeometryMotionSegmentId ==
            previous.runStartGeometryMotionSegmentId &&
            current.runEndGeometryMotionSegmentId >
            previous.runEndGeometryMotionSegmentId &&
            current.linkedSegmentRunLength ==
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDetail::
            SaturatingIncrement(previous.linkedSegmentRunLength) &&
            (previous.participatingAxisUnionMask &
                ~current.participatingAxisUnionMask) == 0U &&
            (previous.coordinateChangeAxisUnionMask &
                ~current.coordinateChangeAxisUnionMask) == 0U &&
            current.firstSourcePC == previous.firstSourcePC &&
            previous.latestSourcePC !=
            (std::numeric_limits<std::int32_t>::max)() &&
            current.latestSourcePC == previous.latestSourcePC + 1 &&
            current.frame == previous.frame &&
            current.kind == previous.kind &&
            current.commandPathPolicy == previous.commandPathPolicy;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE
        static bool PopulateTransition(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& previous,
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& current,
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1&
            target) noexcept
    {
        target.runGeneration = current.runGeneration;
        target.acceptedInputChainGeneration =
            current.acceptedInputChainGeneration;
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
        target.previousLinkedSegmentRunLength =
            previous.linkedSegmentRunLength;
        target.currentLinkedSegmentRunLength =
            current.linkedSegmentRunLength;
        target.previousParticipatingAxisUnionMask =
            previous.participatingAxisUnionMask;
        target.currentParticipatingAxisUnionMask =
            current.participatingAxisUnionMask;
        target.previousCoordinateChangeAxisUnionMask =
            previous.coordinateChangeAxisUnionMask;
        target.currentCoordinateChangeAxisUnionMask =
            current.coordinateChangeAxisUnionMask;
        target.previousEndpointOpenAxisMask =
            previous.endpointOpenAxisMask;
        target.currentEndpointOpenAxisMask =
            current.endpointOpenAxisMask;
        target.previousEndpointReturnedAxisMask =
            previous.endpointReturnedAxisMask;
        target.currentEndpointReturnedAxisMask =
            current.endpointReturnedAxisMask;
        target.newlyOpenedAxisMask =
            current.endpointOpenAxisMask &
            ~previous.coordinateChangeAxisUnionMask;
        target.newlyReturnedAxisMask =
            current.endpointReturnedAxisMask &
            previous.endpointOpenAxisMask;
        target.reopenedAxisMask =
            current.endpointOpenAxisMask &
            previous.endpointReturnedAxisMask;
        target.retainedOpenAxisMask =
            current.endpointOpenAxisMask & previous.endpointOpenAxisMask;
        target.retainedReturnedAxisMask =
            current.endpointReturnedAxisMask &
            previous.endpointReturnedAxisMask;
        target.firstSourcePC = current.firstSourcePC;
        target.previousLatestSourcePC = previous.latestSourcePC;
        target.currentLatestSourcePC = current.latestSourcePC;
        target.previousClosureClass = previous.closureClass;
        target.currentClosureClass = current.closureClass;

        const std::uint32_t unexpectedNewReturnedAxisMask =
            current.endpointReturnedAxisMask &
            ~previous.coordinateChangeAxisUnionMask;
        if (unexpectedNewReturnedAxisMask != 0U)
        {
            return false;
        }

        const bool previousFull =
            previous.closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        const bool currentFull =
            current.closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        target.closureTransitionFlags = static_cast<std::uint8_t>(
            (!previousFull && currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_ENTERED_FULL
                : 0U) |
            (previousFull && !currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_EXITED_FULL
                : 0U) |
            (previousFull && currentFull
                ? NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_RETAINED_FULL
                : 0U));
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition::
            PROVEN_IMMEDIATE_RUN_ENDPOINT_RETURN_TRANSITION;
        target.relation =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation::
            PROVEN_ADJACENT_CLOSURE_STATE_TRANSITION;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent::
            IMMEDIATE_RUN_EXTENSION_ENDPOINT_STATE_MASKS_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
        return target.IsProvenImmediateRunEndpointReturnTransition();
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_TRANSITION_NOINLINE

static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionDisposition) == 1U,
    "Run closure transition disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionRelation) == 1U,
    "Run closure transition relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureTransitionExtent) == 1U,
    "Run closure transition extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1>::value,
    "Run closure transition record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1>::value,
    "Run closure transition record must remain trivially copyable.");
static_assert(
    alignof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1) == 8U,
    "Run closure transition record alignment changed.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1) == 192U,
    "Run closure transition record must remain exactly 192 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1,
        previousLinkedSegmentRunLength) == 104U,
    "Run closure transition mask block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1,
        schemaVersion) == 176U,
    "Run closure transition schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionRecordV1,
        reserved) == 187U,
    "Run closure transition reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow>::value,
    "Run closure transition shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow>::value,
    "Run closure transition shadow must remain trivially copyable.");
static_assert(
    sizeof(
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow) == 400U,
    "Run closure transition shadow must remain exactly 400 bytes.");
