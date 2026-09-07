#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunShadow.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2I / Proven Linked Committed Segment Run Endpoint-Boundary Shadow
//
// This fixed two-record observer consumes only a proven L.2H scalar run, its
// newest proven L.2G pair and the two newest L.2F segment observations.  When
// L.2H starts a run, L.2I captures the first segment's commanded start MCS and
// the second segment's commanded end MCS.  When L.2H extends that same run,
// L.2I preserves the already-proven head endpoint and advances only the tail
// endpoint after all run, pair, segment and prior-boundary fences bind exactly.
//
// The two endpoint arrays describe only the commanded boundary of the proven
// committed segment run.  No intermediate endpoint or per-segment entry is
// retained, so the record cannot be traversed to reconstruct a path.  It does
// not establish distance, direction, tangent, rotary unwrapping, interpolation,
// timing, actual Motion execution/completion, retrace capacity or a B2
// breadcrumb.  It is not a Path Queue and has no Gate, PC, Alarm, Motion,
// HMI/SHM/API, PDO, EtherCAT or DC consumer.
//
// Neutral and invalid observations clear both endpoint arrays before becoming
// newest history.  The shadow is same-thread, fixed-capacity and heap-resident
// through NCManager, with no allocation, logging, waiting or synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_SCHEMA_V1 = 1U;

namespace NCPathCoreLinkedCommittedSegmentRunBoundaryDetail
{
    constexpr std::uint64_t ForwardNonZeroSequenceDistance(
        std::uint64_t first,
        std::uint64_t latest) noexcept
    {
        return latest >= first
            ? latest - first
            : ((std::numeric_limits<std::uint64_t>::max)() - first) +
            latest;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_RUN_UNAVAILABLE = 1U,
    NOT_APPLICABLE_RUN_NOT_PROVEN = 2U,
    INVALID_RUN_RECORD = 3U,
    INVALID_PAIR_RECORD = 4U,
    INVALID_PREVIOUS_SEGMENT_RECORD = 5U,
    INVALID_CURRENT_SEGMENT_RECORD = 6U,
    INVALID_PROOF_BINDING = 7U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 8U,
    INVALID_RUN_EXTENSION_FENCE = 9U,
    INVALID_BOUNDARY_GEOMETRY = 10U,
    PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED = 11U,
    PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED = 12U
};

enum class NCPathCoreLinkedCommittedSegmentRunBoundaryRelation :
    std::uint8_t
{
    NONE = 0U,
    TWO_SEGMENT_RUN_ESTABLISHES_HEAD_AND_TAIL = 1U,
    OVERLAPPING_PROVEN_PAIR_ADVANCES_TAIL_ONLY = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunBoundaryExtent :
    std::uint8_t
{
    NONE = 0U,
    RUN_HEAD_AND_CURRENT_TAIL_COMMANDED_ENDPOINTS_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourceRunPublicationSequence = 0ULL;
    std::uint64_t sourcePairPublicationSequence = 0ULL;
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

    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> runStartMCS{};
    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> runEndMCS{};

    std::uint32_t linkedSegmentRunLength = 0U;
    std::uint32_t firstAcceptedInputRunLength = 0U;
    std::uint32_t latestAcceptedInputRunLength = 0U;
    std::uint32_t participatingAxisUnionMask = 0U;
    std::uint32_t coordinateChangeAxisUnionMask = 0U;
    std::uint32_t firstSegmentAxisMask = 0U;
    std::uint32_t latestSegmentAxisMask = 0U;
    std::uint32_t endpointDisplacementAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition disposition =
        NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRunBoundaryRelation boundaryRelation =
        NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunBoundaryExtent extent =
        NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 8U> reserved{};

    bool IsProvenRunEndpointBoundary() const noexcept
    {
        const bool started =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED &&
            boundaryRelation ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::
            TWO_SEGMENT_RUN_ESTABLISHES_HEAD_AND_TAIL;
        const bool extended =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED &&
            boundaryRelation ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::
            OVERLAPPING_PROVEN_PAIR_ADVANCES_TAIL_ONLY;

        if (publicationSequence == 0ULL ||
            sourceRunPublicationSequence == 0ULL ||
            sourcePairPublicationSequence == 0ULL ||
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
            linkedSegmentRunLength < 2U ||
            firstAcceptedInputRunLength < 2U ||
            latestAcceptedInputRunLength < firstAcceptedInputRunLength ||
            participatingAxisUnionMask == 0U ||
            (participatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_AXIS_MASK) !=
            0U ||
            (coordinateChangeAxisUnionMask &
                ~participatingAxisUnionMask) != 0U ||
            firstSegmentAxisMask == 0U ||
            latestSegmentAxisMask == 0U ||
            (firstSegmentAxisMask & ~participatingAxisUnionMask) != 0U ||
            (latestSegmentAxisMask & ~participatingAxisUnionMask) != 0U ||
            (endpointDisplacementAxisMask &
                ~coordinateChangeAxisUnionMask) != 0U ||
            firstSourcePC < 0 ||
            latestSourcePC < firstSourcePC ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_SCHEMA_V1 ||
            (!started && !extended) ||
            extent !=
            NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::
            RUN_HEAD_AND_CURRENT_TAIL_COMMANDED_ENDPOINTS_ONLY ||
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
            static_cast<std::int64_t>(latestSourcePC) -
            static_cast<std::int64_t>(firstSourcePC) + 1LL;
        if (sourcePcSpan <= 0LL ||
            static_cast<std::uint64_t>(sourcePcSpan) !=
            static_cast<std::uint64_t>(linkedSegmentRunLength))
        {
            return false;
        }

        if (latestAcceptedInputRunLength !=
            NCPathCoreLinkedCommittedSegmentRunDetail::
            LINKED_SEGMENT_RUN_LENGTH_MAX)
        {
            const std::uint64_t acceptedRunSpan =
                static_cast<std::uint64_t>(latestAcceptedInputRunLength) -
                static_cast<std::uint64_t>(firstAcceptedInputRunLength) +
                1ULL;
            if (acceptedRunSpan !=
                static_cast<std::uint64_t>(linkedSegmentRunLength))
            {
                return false;
            }
        }

        const std::uint64_t runLength =
            static_cast<std::uint64_t>(linkedSegmentRunLength);
        if (NCPathCoreLinkedCommittedSegmentRunBoundaryDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunBoundaryDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunBoundaryDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) != runLength)
        {
            return false;
        }

        std::uint32_t observedEndpointDisplacementAxisMask = 0U;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(runStartMCS[axis]) ||
                !std::isfinite(runEndMCS[axis]))
            {
                return false;
            }

            if (runStartMCS[axis] != runEndMCS[axis])
            {
                observedEndpointDisplacementAxisMask |=
                    static_cast<std::uint32_t>(1U << axis);
            }
        }
        if (observedEndpointDisplacementAxisMask !=
            endpointDisplacementAxisMask)
        {
            return false;
        }

        if (started)
        {
            return linkedSegmentRunLength == 2U;
        }

        return linkedSegmentRunLength >= 3U;
    }

private:
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
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentRunBoundaryShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunBoundaryShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        void ObserveLatestRunSameThread(
            const NCPathCoreLinkedCommittedSegmentRunRecordV1* run,
            const NCPathCoreLinkedCommittedSegmentPairRecordV1* pair,
            const NCPathCoreLinkedCommittedSegmentRecordV1* previousSegment,
            const NCPathCoreLinkedCommittedSegmentRecordV1* currentSegment)
        noexcept
    {
        const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1* const
            previousBoundary = GetNewestObservationSameThread();

        m_publicationSequence =
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY;

        // Write directly into the NCManager-owned fixed slot.  Upstream L.2F,
        // L.2G and L.2H records remain behind const pointers and no endpoint
        // array or boundary record is copied onto the NC thread stack.
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target =
            m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            run == nullptr ? 0ULL : run->publicationSequence,
            pair == nullptr ? 0ULL : pair->publicationSequence);

        if (run == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                NOT_APPLICABLE_RUN_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenRun(*run))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                NOT_APPLICABLE_RUN_NOT_PROVEN;
        }
        else if (!run->IsProvenLinkedCommittedSegmentRun())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_RUN_RECORD;
        }
        else if (pair == nullptr ||
            !pair->IsProvenImmediateLinkedSegmentContinuation() ||
            !PairReservedIsZero(pair->reserved))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_PAIR_RECORD;
        }
        else if (previousSegment == nullptr ||
            !previousSegment->IsFormedLinkedCommandedEndpointSegment())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_PREVIOUS_SEGMENT_RECORD;
        }
        else if (currentSegment == nullptr ||
            !currentSegment->IsFormedLinkedCommandedEndpointSegment())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_CURRENT_SEGMENT_RECORD;
        }
        else if (!IsCommonProofBindingValid(
            *run,
            *pair,
            *previousSegment,
            *currentSegment))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_PROOF_BINDING;
        }
        else if (run->disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_STARTED)
        {
            if (!IsStartedRunBindingValid(
                *run,
                *pair,
                *previousSegment,
                *currentSegment))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                    INVALID_PROOF_BINDING;
            }
            else if (!StartBoundary(
                *run,
                *previousSegment,
                *currentSegment,
                target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                    INVALID_BOUNDARY_GEOMETRY;
            }
        }
        else if (run->disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            PROVEN_LINKED_SEGMENT_RUN_EXTENDED)
        {
            if (previousBoundary == nullptr ||
                !previousBoundary->IsProvenRunEndpointBoundary())
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                    INVALID_PREVIOUS_BOUNDARY_RECORD;
            }
            else if (!IsRunExtensionFenceValid(
                *previousBoundary,
                *run,
                *pair,
                *previousSegment,
                *currentSegment))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                    INVALID_RUN_EXTENSION_FENCE;
            }
            else if (!ExtendBoundary(
                *previousBoundary,
                *run,
                *currentSegment,
                target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                    INVALID_BOUNDARY_GEOMETRY;
            }
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                INVALID_RUN_RECORD;
        }

        // Neutral and invalid records are already zero-payload newest history,
        // preventing an older proven boundary from masquerading as current.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1*
        GetNewestObservationSameThread(
            std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX ||
            historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target,
        std::uint64_t publicationSequence,
        std::uint64_t sourceRunPublicationSequence,
        std::uint64_t sourcePairPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceRunPublicationSequence = sourceRunPublicationSequence;
        target.sourcePairPublicationSequence = sourcePairPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            NOT_APPLICABLE_RUN_UNAVAILABLE;
        target.boundaryRelation =
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static void ClearProvenPayload(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target) noexcept
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
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            target.runStartMCS[axis] = 0.0;
            target.runEndMCS[axis] = 0.0;
        }
        target.linkedSegmentRunLength = 0U;
        target.firstAcceptedInputRunLength = 0U;
        target.latestAcceptedInputRunLength = 0U;
        target.participatingAxisUnionMask = 0U;
        target.coordinateChangeAxisUnionMask = 0U;
        target.firstSegmentAxisMask = 0U;
        target.latestSegmentAxisMask = 0U;
        target.endpointDisplacementAxisMask = 0U;
        target.firstSourcePC = -1;
        target.latestSourcePC = -1;
        target.boundaryRelation =
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    }

    static bool PairReservedIsZero(
        const std::array<std::uint8_t, 3U>& reserved) noexcept
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

    static bool RunReservedIsZero(
        const std::array<std::uint8_t, 4U>& reserved) noexcept
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        static bool IsStructurallyValidNonProvenRun(
            const NCPathCoreLinkedCommittedSegmentRunRecordV1& run) noexcept
    {
        const bool sourcePairExpected =
            run.disposition !=
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_UNAVAILABLE;
        const bool knownDisposition =
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_UNAVAILABLE ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            NOT_APPLICABLE_PAIR_NOT_PROVEN ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_PAIR_RECORD ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_PREVIOUS_RUN_RECORD ||
            run.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisposition::
            INVALID_RUN_EXTENSION_FENCE;

        return knownDisposition &&
            run.publicationSequence != 0ULL &&
            run.schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_SCHEMA_V1 &&
            ((sourcePairExpected &&
                run.sourcePairPublicationSequence != 0ULL) ||
                (!sourcePairExpected &&
                    run.sourcePairPublicationSequence == 0ULL)) &&
            run.runGeneration == 0ULL &&
            run.acceptedInputChainGeneration == 0ULL &&
            run.firstPairPublicationSequence == 0ULL &&
            run.latestPairPublicationSequence == 0ULL &&
            run.firstSegmentPublicationSequence == 0ULL &&
            run.latestSegmentPublicationSequence == 0ULL &&
            run.firstLinkPublicationSequence == 0ULL &&
            run.latestLinkPublicationSequence == 0ULL &&
            run.linkedSegmentRunLength == 0U &&
            run.firstAcceptedInputRunLength == 0U &&
            run.latestAcceptedInputRunLength == 0U &&
            run.participatingAxisUnionMask == 0U &&
            run.coordinateChangeAxisUnionMask == 0U &&
            run.latestSegmentAxisMask == 0U &&
            run.latestCoordinateChangeAxisMask == 0U &&
            run.firstSourcePC == -1 &&
            run.latestSourcePC == -1 &&
            run.runRelation ==
            NCPathCoreLinkedCommittedSegmentRunRelation::NONE &&
            run.extent == NCPathCoreLinkedCommittedSegmentRunExtent::NONE &&
            run.frame == NCPathCoreCommittedGeometryFrame::NONE &&
            run.kind == NCPathCoreCommittedGeometryKind::NONE &&
            run.commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE &&
            RunReservedIsZero(run.reserved);
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        static bool IsCommonProofBindingValid(
            const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
            const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair,
            const NCPathCoreLinkedCommittedSegmentRecordV1& previousSegment,
            const NCPathCoreLinkedCommittedSegmentRecordV1& currentSegment)
        noexcept
    {
        return
            run.sourcePairPublicationSequence == pair.publicationSequence &&
            run.latestPairPublicationSequence == pair.publicationSequence &&
            run.acceptedInputChainGeneration ==
            pair.acceptedInputChainGeneration &&
            run.latestSegmentPublicationSequence ==
            pair.currentSegmentPublicationSequence &&
            run.latestSegmentPublicationSequence ==
            currentSegment.publicationSequence &&
            run.latestLinkPublicationSequence ==
            pair.currentLinkPublicationSequence &&
            run.latestLinkPublicationSequence ==
            currentSegment.linkPublicationSequence &&
            run.latestAcceptedInputRunLength ==
            pair.currentAcceptedInputRunLength &&
            run.latestAcceptedInputRunLength ==
            currentSegment.acceptedInputRunLength &&
            run.latestSegmentAxisMask == pair.currentSegmentAxisMask &&
            run.latestSegmentAxisMask == currentSegment.currentAxisMask &&
            run.latestCoordinateChangeAxisMask ==
            pair.currentCoordinateChangeAxisMask &&
            run.latestCoordinateChangeAxisMask ==
            currentSegment.coordinateChangeAxisMask &&
            run.latestSourcePC == pair.currentSourcePC &&
            run.latestSourcePC == currentSegment.sourcePC &&
            pair.previousSegmentPublicationSequence ==
            previousSegment.publicationSequence &&
            pair.currentSegmentPublicationSequence ==
            currentSegment.publicationSequence &&
            pair.previousLinkPublicationSequence ==
            previousSegment.linkPublicationSequence &&
            pair.currentLinkPublicationSequence ==
            currentSegment.linkPublicationSequence &&
            pair.acceptedInputChainGeneration ==
            previousSegment.acceptedInputChainGeneration &&
            pair.acceptedInputChainGeneration ==
            currentSegment.acceptedInputChainGeneration &&
            pair.previousTerminalGeometryPublicationSequence ==
            previousSegment.currentGeometryPublicationSequence &&
            pair.currentPredecessorGeometryPublicationSequence ==
            currentSegment.previousGeometryPublicationSequence &&
            previousSegment.currentGeometryPublicationSequence ==
            currentSegment.previousGeometryPublicationSequence &&
            pair.previousTerminalMotionSegmentId ==
            previousSegment.currentMotionSegmentId &&
            pair.currentPredecessorMotionSegmentId ==
            currentSegment.previousMotionSegmentId &&
            previousSegment.currentMotionSegmentId ==
            currentSegment.previousMotionSegmentId &&
            pair.previousTerminalAxisMask ==
            previousSegment.currentAxisMask &&
            pair.currentPredecessorAxisMask ==
            currentSegment.previousAxisMask &&
            previousSegment.currentAxisMask == currentSegment.previousAxisMask &&
            pair.currentSegmentAxisMask == currentSegment.currentAxisMask &&
            pair.previousCoordinateChangeAxisMask ==
            previousSegment.coordinateChangeAxisMask &&
            pair.currentCoordinateChangeAxisMask ==
            currentSegment.coordinateChangeAxisMask &&
            pair.previousAcceptedInputRunLength ==
            previousSegment.acceptedInputRunLength &&
            pair.currentAcceptedInputRunLength ==
            currentSegment.acceptedInputRunLength &&
            pair.previousSourcePC == previousSegment.sourcePC &&
            pair.currentSourcePC == currentSegment.sourcePC;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        static bool IsStartedRunBindingValid(
            const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
            const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair,
            const NCPathCoreLinkedCommittedSegmentRecordV1& previousSegment,
            const NCPathCoreLinkedCommittedSegmentRecordV1& currentSegment)
        noexcept
    {
        return
            run.runRelation ==
            NCPathCoreLinkedCommittedSegmentRunRelation::
            FIRST_PROVEN_PAIR_ESTABLISHES_TWO_SEGMENT_RUN &&
            run.linkedSegmentRunLength == 2U &&
            run.firstPairPublicationSequence == pair.publicationSequence &&
            run.firstSegmentPublicationSequence ==
            previousSegment.publicationSequence &&
            run.latestSegmentPublicationSequence ==
            currentSegment.publicationSequence &&
            run.firstLinkPublicationSequence ==
            previousSegment.linkPublicationSequence &&
            run.latestLinkPublicationSequence ==
            currentSegment.linkPublicationSequence &&
            run.firstAcceptedInputRunLength ==
            previousSegment.acceptedInputRunLength &&
            run.latestAcceptedInputRunLength ==
            currentSegment.acceptedInputRunLength &&
            run.participatingAxisUnionMask ==
            (previousSegment.currentAxisMask |
                currentSegment.currentAxisMask) &&
            run.coordinateChangeAxisUnionMask ==
            (previousSegment.coordinateChangeAxisMask |
                currentSegment.coordinateChangeAxisMask) &&
            run.firstSourcePC == previousSegment.sourcePC &&
            run.latestSourcePC == currentSegment.sourcePC;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        static bool IsRunExtensionFenceValid(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& previous,
            const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
            const NCPathCoreLinkedCommittedSegmentPairRecordV1& pair,
            const NCPathCoreLinkedCommittedSegmentRecordV1& previousSegment,
            const NCPathCoreLinkedCommittedSegmentRecordV1& currentSegment)
        noexcept
    {
        if (run.runRelation !=
            NCPathCoreLinkedCommittedSegmentRunRelation::
            OVERLAPPING_PROVEN_PAIR_EXTENDS_EXISTING_RUN ||
            run.publicationSequence !=
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previous.sourceRunPublicationSequence) ||
            pair.publicationSequence !=
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previous.sourcePairPublicationSequence) ||
            run.runGeneration != previous.runGeneration ||
            run.acceptedInputChainGeneration !=
            previous.acceptedInputChainGeneration ||
            run.firstSegmentPublicationSequence !=
            previous.firstSegmentPublicationSequence ||
            run.firstLinkPublicationSequence !=
            previous.firstLinkPublicationSequence ||
            run.firstAcceptedInputRunLength !=
            previous.firstAcceptedInputRunLength ||
            run.firstSourcePC != previous.firstSourcePC ||
            run.linkedSegmentRunLength !=
            NCPathCoreLinkedCommittedSegmentRunDetail::
            SaturatingIncrementRunLength(
                previous.linkedSegmentRunLength) ||
            pair.previousSegmentPublicationSequence !=
            previous.latestSegmentPublicationSequence ||
            pair.previousLinkPublicationSequence !=
            previous.latestLinkPublicationSequence ||
            previousSegment.publicationSequence !=
            previous.latestSegmentPublicationSequence ||
            previousSegment.linkPublicationSequence !=
            previous.latestLinkPublicationSequence ||
            previousSegment.acceptedInputRunLength !=
            previous.latestAcceptedInputRunLength ||
            previousSegment.currentAxisMask != previous.latestSegmentAxisMask ||
            previousSegment.sourcePC != previous.latestSourcePC ||
            previousSegment.currentGeometryPublicationSequence !=
            previous.runEndGeometryPublicationSequence ||
            previousSegment.currentMotionSegmentId !=
            previous.runEndGeometryMotionSegmentId ||
            currentSegment.publicationSequence !=
            run.latestSegmentPublicationSequence ||
            currentSegment.linkPublicationSequence !=
            run.latestLinkPublicationSequence ||
            run.participatingAxisUnionMask !=
            (previous.participatingAxisUnionMask |
                pair.previousTerminalAxisMask |
                pair.currentSegmentAxisMask) ||
            run.coordinateChangeAxisUnionMask !=
            (previous.coordinateChangeAxisUnionMask |
                pair.previousCoordinateChangeAxisMask |
                pair.currentCoordinateChangeAxisMask))
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                previous.runEndMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                    previousSegment.endMCS[axis]))
            {
                return false;
            }
        }
        return true;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE
        static bool StartBoundary(
            const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
            const NCPathCoreLinkedCommittedSegmentRecordV1& previousSegment,
            const NCPathCoreLinkedCommittedSegmentRecordV1& currentSegment,
            NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target) noexcept
    {
        ApplyRunScalarProvenance(run, target);
        target.runStartGeometryPublicationSequence =
            previousSegment.previousGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            currentSegment.currentGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            previousSegment.previousMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            currentSegment.currentMotionSegmentId;
        target.firstSegmentAxisMask = previousSegment.currentAxisMask;
        target.latestSegmentAxisMask = currentSegment.currentAxisMask;

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            target.runStartMCS[axis] = previousSegment.startMCS[axis];
            target.runEndMCS[axis] = currentSegment.endMCS[axis];
            if (target.runStartMCS[axis] != target.runEndMCS[axis])
            {
                target.endpointDisplacementAxisMask |=
                    static_cast<std::uint32_t>(1U << axis);
            }
        }

        if (!IsPopulatedBoundaryGeometryValid(target))
        {
            return false;
        }

        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED;
        target.boundaryRelation =
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::
            TWO_SEGMENT_RUN_ESTABLISHES_HEAD_AND_TAIL;
        ApplyProvenSemanticMetadata(target);
        return true;
    }

    static bool ExtendBoundary(
        const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& previous,
        const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
        const NCPathCoreLinkedCommittedSegmentRecordV1& currentSegment,
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target) noexcept
    {
        ApplyRunScalarProvenance(run, target);
        target.runStartGeometryPublicationSequence =
            previous.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            currentSegment.currentGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            previous.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            currentSegment.currentMotionSegmentId;
        target.firstSegmentAxisMask = previous.firstSegmentAxisMask;
        target.latestSegmentAxisMask = currentSegment.currentAxisMask;

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            target.runStartMCS[axis] = previous.runStartMCS[axis];
            target.runEndMCS[axis] = currentSegment.endMCS[axis];
            if (target.runStartMCS[axis] != target.runEndMCS[axis])
            {
                target.endpointDisplacementAxisMask |=
                    static_cast<std::uint32_t>(1U << axis);
            }
        }

        if (!IsPopulatedBoundaryGeometryValid(target))
        {
            return false;
        }

        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED;
        target.boundaryRelation =
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::
            OVERLAPPING_PROVEN_PAIR_ADVANCES_TAIL_ONLY;
        ApplyProvenSemanticMetadata(target);
        return true;
    }

    static void ApplyRunScalarProvenance(
        const NCPathCoreLinkedCommittedSegmentRunRecordV1& run,
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target) noexcept
    {
        target.runGeneration = run.runGeneration;
        target.acceptedInputChainGeneration =
            run.acceptedInputChainGeneration;
        target.firstSegmentPublicationSequence =
            run.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            run.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence = run.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            run.latestLinkPublicationSequence;
        target.linkedSegmentRunLength = run.linkedSegmentRunLength;
        target.firstAcceptedInputRunLength =
            run.firstAcceptedInputRunLength;
        target.latestAcceptedInputRunLength =
            run.latestAcceptedInputRunLength;
        target.participatingAxisUnionMask =
            run.participatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            run.coordinateChangeAxisUnionMask;
        target.firstSourcePC = run.firstSourcePC;
        target.latestSourcePC = run.latestSourcePC;
    }

    static bool IsPopulatedBoundaryGeometryValid(
        const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target)
        noexcept
    {
        if (target.runStartGeometryPublicationSequence == 0ULL ||
            target.runEndGeometryPublicationSequence == 0ULL ||
            target.runStartGeometryMotionSegmentId == 0ULL ||
            target.runEndGeometryMotionSegmentId <=
            target.runStartGeometryMotionSegmentId ||
            target.firstSegmentAxisMask == 0U ||
            target.latestSegmentAxisMask == 0U ||
            (target.firstSegmentAxisMask &
                ~target.participatingAxisUnionMask) != 0U ||
            (target.latestSegmentAxisMask &
                ~target.participatingAxisUnionMask) != 0U ||
            (target.endpointDisplacementAxisMask &
                ~target.coordinateChangeAxisUnionMask) != 0U ||
            NCPathCoreLinkedCommittedSegmentRunBoundaryDetail::
            ForwardNonZeroSequenceDistance(
                target.runStartGeometryPublicationSequence,
                target.runEndGeometryPublicationSequence) !=
            static_cast<std::uint64_t>(
                target.linkedSegmentRunLength))
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(target.runStartMCS[axis]) ||
                !std::isfinite(target.runEndMCS[axis]))
            {
                return false;
            }
        }
        return true;
    }

    static void ApplyProvenSemanticMetadata(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1& target) noexcept
    {
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::
            RUN_HEAD_AND_CURRENT_TAIL_COMMANDED_ENDPOINTS_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition) == 1U,
    "Run endpoint-boundary disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunBoundaryRelation) == 1U,
    "Run endpoint-boundary relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunBoundaryExtent) == 1U,
    "Run endpoint-boundary extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1>::value,
    "Run endpoint-boundary record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1>::value,
    "Run endpoint-boundary record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1) == 8U,
    "Run endpoint-boundary record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1) == 288U,
    "Run endpoint-boundary record must remain exactly 288 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        runStartMCS) == 104U,
    "Run endpoint-boundary start-array offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        runEndMCS) == 168U,
    "Run endpoint-boundary end-array offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        linkedSegmentRunLength) == 232U,
    "Run endpoint-boundary run-length offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        schemaVersion) == 272U,
    "Run endpoint-boundary schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1,
        reserved) == 280U,
    "Run endpoint-boundary reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunBoundaryShadow>::value,
    "Run endpoint-boundary shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunBoundaryShadow>::value,
    "Run endpoint-boundary shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunBoundaryShadow) == 592U,
    "Run endpoint-boundary shadow must remain exactly 592 bytes.");
