#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunBoundaryShadow.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2J / Proven Linked Committed Segment Run Endpoint-Net-
// Displacement Shadow
//
// This fixed two-record observer consumes only the newest L.2I endpoint
// boundary and, for an extending run, the immediately preceding L.2I boundary.
// A proven observation derives one component-wise commanded MCS displacement:
//
//     endpointNetDisplacementMCS = runEndMCS - runStartMCS
//
// The record retains only that eight-axis net endpoint displacement plus
// scalar provenance and sign masks.  It does not retain either source endpoint,
// any intermediate endpoint, any segment entry or any traversable ordering.
// A zero net displacement may still describe a run which moved away and
// returned to the same commanded endpoint; it never means that no path motion
// occurred.
//
// L.2J establishes no path length, travelled distance, unit direction,
// tangent, curvature, rotary unwrap, timing, actual Motion execution/completion,
// retrace capacity or B2 breadcrumb.  It is not a Path Queue and has no Gate,
// PC, Alarm, Motion, HMI/SHM/API, PDO, EtherCAT or DC consumer.
//
// Neutral and invalid observations clear the displacement array before
// becoming newest history.  The shadow is same-thread, fixed-capacity and
// heap-resident through NCManager, with no allocation, logging, waiting or
// synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_SCHEMA_V1 = 1U;

namespace NCPathCoreLinkedCommittedSegmentRunDisplacementDetail
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

    inline std::uint64_t DoubleObjectBits(double value) noexcept
    {
        static_assert(
            sizeof(double) == sizeof(std::uint64_t),
            "L.2J requires a 64-bit double object representation.");
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_BOUNDARY_UNAVAILABLE = 1U,
    NOT_APPLICABLE_BOUNDARY_NOT_PROVEN = 2U,
    INVALID_CURRENT_BOUNDARY_RECORD = 3U,
    INVALID_PREVIOUS_BOUNDARY_RECORD = 4U,
    INVALID_PREVIOUS_DISPLACEMENT_RECORD = 5U,
    INVALID_BOUNDARY_ADVANCE_FENCE = 6U,
    INVALID_NET_DISPLACEMENT_GEOMETRY = 7U,
    PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED = 8U,
    PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED = 9U
};

enum class NCPathCoreLinkedCommittedSegmentRunDisplacementRelation :
    std::uint8_t
{
    NONE = 0U,
    PROVEN_TWO_SEGMENT_BOUNDARY_DERIVES_NET_DISPLACEMENT = 1U,
    PROVEN_BOUNDARY_ADVANCE_REDERIVES_RUN_NET_DISPLACEMENT = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunDisplacementExtent :
    std::uint8_t
{
    NONE = 0U,
    RUN_HEAD_TO_CURRENT_TAIL_COMPONENT_DISPLACEMENT_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourceBoundaryPublicationSequence = 0ULL;
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
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY>
        endpointNetDisplacementMCS{};

    std::uint32_t linkedSegmentRunLength = 0U;
    std::uint32_t participatingAxisUnionMask = 0U;
    std::uint32_t coordinateChangeAxisUnionMask = 0U;
    std::uint32_t endpointDisplacementAxisMask = 0U;
    std::uint32_t positiveNetDisplacementAxisMask = 0U;
    std::uint32_t negativeNetDisplacementAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition disposition =
        NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRunDisplacementRelation
        displacementRelation =
        NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunDisplacementExtent extent =
        NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 8U> reserved{};

    bool IsProvenRunEndpointNetDisplacement() const noexcept
    {
        const bool started =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED &&
            displacementRelation ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
            PROVEN_TWO_SEGMENT_BOUNDARY_DERIVES_NET_DISPLACEMENT;
        const bool extended =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED &&
            displacementRelation ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
            PROVEN_BOUNDARY_ADVANCE_REDERIVES_RUN_NET_DISPLACEMENT;

        if (publicationSequence == 0ULL ||
            sourceBoundaryPublicationSequence == 0ULL ||
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
            participatingAxisUnionMask == 0U ||
            (participatingAxisUnionMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_AXIS_MASK) !=
            0U ||
            (coordinateChangeAxisUnionMask &
                ~participatingAxisUnionMask) != 0U ||
            (endpointDisplacementAxisMask &
                ~coordinateChangeAxisUnionMask) != 0U ||
            (positiveNetDisplacementAxisMask &
                ~endpointDisplacementAxisMask) != 0U ||
            (negativeNetDisplacementAxisMask &
                ~endpointDisplacementAxisMask) != 0U ||
            (positiveNetDisplacementAxisMask &
                negativeNetDisplacementAxisMask) != 0U ||
            (positiveNetDisplacementAxisMask |
                negativeNetDisplacementAxisMask) !=
            endpointDisplacementAxisMask ||
            firstSourcePC < 0 ||
            latestSourcePC < firstSourcePC ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_SCHEMA_V1 ||
            (!started && !extended) ||
            extent !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::
            RUN_HEAD_TO_CURRENT_TAIL_COMPONENT_DISPLACEMENT_ONLY ||
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

        const std::uint64_t runLength =
            static_cast<std::uint64_t>(linkedSegmentRunLength);
        if (NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) != runLength)
        {
            return false;
        }

        std::uint32_t observedPositiveAxisMask = 0U;
        std::uint32_t observedNegativeAxisMask = 0U;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            const double displacement = endpointNetDisplacementMCS[axis];
            if (!std::isfinite(displacement))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if (displacement > 0.0)
            {
                observedPositiveAxisMask |= axisBit;
            }
            else if (displacement < 0.0)
            {
                observedNegativeAxisMask |= axisBit;
            }
            else if (
                NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(displacement) != 0ULL)
            {
                // All zero components are canonicalized to +0.0.  This keeps
                // the observer deterministic across signed-zero endpoints.
                return false;
            }
        }

        if (observedPositiveAxisMask != positiveNetDisplacementAxisMask ||
            observedNegativeAxisMask != negativeNetDisplacementAxisMask)
        {
            return false;
        }

        return started
            ? linkedSegmentRunLength == 2U
            : linkedSegmentRunLength >= 3U;
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
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentRunDisplacementShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunDisplacementShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        void ObserveLatestBoundarySameThread(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1*
            previousBoundary,
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1*
            currentBoundary) noexcept
    {
        const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1* const
            previousDisplacement = GetNewestObservationSameThread();

        m_publicationSequence =
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY;

        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1& target =
            m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            currentBoundary == nullptr
            ? 0ULL
            : currentBoundary->publicationSequence,
            currentBoundary == nullptr
            ? 0ULL
            : currentBoundary->sourceRunPublicationSequence,
            currentBoundary == nullptr
            ? 0ULL
            : currentBoundary->sourcePairPublicationSequence);

        EvaluateObservation(
            previousBoundary,
            currentBoundary,
            previousDisplacement,
            target);

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
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
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        static void EvaluateObservation(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1*
            previousBoundary,
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1*
            currentBoundary,
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
            previousDisplacement,
            NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1& target)
        noexcept
    {
        if (currentBoundary == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                NOT_APPLICABLE_BOUNDARY_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenBoundary(*currentBoundary))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                NOT_APPLICABLE_BOUNDARY_NOT_PROVEN;
        }
        else if (!currentBoundary->IsProvenRunEndpointBoundary())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                INVALID_CURRENT_BOUNDARY_RECORD;
        }
        else if (currentBoundary->disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED)
        {
            if (!PopulateFromBoundary(*currentBoundary, target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                    INVALID_NET_DISPLACEMENT_GEOMETRY;
            }
        }
        else if (currentBoundary->disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED)
        {
            if (previousBoundary == nullptr ||
                !previousBoundary->IsProvenRunEndpointBoundary())
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                    INVALID_PREVIOUS_BOUNDARY_RECORD;
            }
            else if (previousDisplacement == nullptr ||
                !previousDisplacement->IsProvenRunEndpointNetDisplacement() ||
                !IsPreviousDisplacementBoundToBoundary(
                    *previousDisplacement,
                    *previousBoundary))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                    INVALID_PREVIOUS_DISPLACEMENT_RECORD;
            }
            else if (!IsDirectBoundaryAdvanceFenceValid(
                *previousBoundary,
                *currentBoundary))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                    INVALID_BOUNDARY_ADVANCE_FENCE;
            }
            else if (!PopulateFromBoundary(*currentBoundary, target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                    INVALID_NET_DISPLACEMENT_GEOMETRY;
            }
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                INVALID_CURRENT_BOUNDARY_RECORD;
        }
    }

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1& target,
        std::uint64_t publicationSequence,
        std::uint64_t sourceBoundaryPublicationSequence,
        std::uint64_t sourceRunPublicationSequence,
        std::uint64_t sourcePairPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceBoundaryPublicationSequence =
            sourceBoundaryPublicationSequence;
        target.sourceRunPublicationSequence = sourceRunPublicationSequence;
        target.sourcePairPublicationSequence = sourcePairPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            NOT_APPLICABLE_BOUNDARY_UNAVAILABLE;
        target.displacementRelation =
            NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static void ClearProvenPayload(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1& target)
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
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            target.endpointNetDisplacementMCS[axis] = 0.0;
        }
        target.linkedSegmentRunLength = 0U;
        target.participatingAxisUnionMask = 0U;
        target.coordinateChangeAxisUnionMask = 0U;
        target.endpointDisplacementAxisMask = 0U;
        target.positiveNetDisplacementAxisMask = 0U;
        target.negativeNetDisplacementAxisMask = 0U;
        target.firstSourcePC = -1;
        target.latestSourcePC = -1;
        target.displacementRelation =
            NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    }

    static bool BoundaryReservedIsZero(
        const std::array<std::uint8_t, 8U>& reserved) noexcept
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        static bool IsStructurallyValidNonProvenBoundary(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1&
            boundary) noexcept
    {
        const bool knownDisposition =
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            NOT_APPLICABLE_RUN_UNAVAILABLE ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            NOT_APPLICABLE_RUN_NOT_PROVEN ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_RUN_RECORD ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_PAIR_RECORD ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_PREVIOUS_SEGMENT_RECORD ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_CURRENT_SEGMENT_RECORD ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_PROOF_BINDING ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_PREVIOUS_BOUNDARY_RECORD ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_RUN_EXTENSION_FENCE ||
            boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            INVALID_BOUNDARY_GEOMETRY;

        if (!knownDisposition ||
            boundary.publicationSequence == 0ULL ||
            boundary.schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_BOUNDARY_SCHEMA_V1 ||
            boundary.runGeneration != 0ULL ||
            boundary.acceptedInputChainGeneration != 0ULL ||
            boundary.firstSegmentPublicationSequence != 0ULL ||
            boundary.latestSegmentPublicationSequence != 0ULL ||
            boundary.firstLinkPublicationSequence != 0ULL ||
            boundary.latestLinkPublicationSequence != 0ULL ||
            boundary.runStartGeometryPublicationSequence != 0ULL ||
            boundary.runEndGeometryPublicationSequence != 0ULL ||
            boundary.runStartGeometryMotionSegmentId != 0ULL ||
            boundary.runEndGeometryMotionSegmentId != 0ULL ||
            boundary.linkedSegmentRunLength != 0U ||
            boundary.firstAcceptedInputRunLength != 0U ||
            boundary.latestAcceptedInputRunLength != 0U ||
            boundary.participatingAxisUnionMask != 0U ||
            boundary.coordinateChangeAxisUnionMask != 0U ||
            boundary.firstSegmentAxisMask != 0U ||
            boundary.latestSegmentAxisMask != 0U ||
            boundary.endpointDisplacementAxisMask != 0U ||
            boundary.firstSourcePC != -1 ||
            boundary.latestSourcePC != -1 ||
            boundary.boundaryRelation !=
            NCPathCoreLinkedCommittedSegmentRunBoundaryRelation::NONE ||
            boundary.extent !=
            NCPathCoreLinkedCommittedSegmentRunBoundaryExtent::NONE ||
            boundary.frame != NCPathCoreCommittedGeometryFrame::NONE ||
            boundary.kind != NCPathCoreCommittedGeometryKind::NONE ||
            boundary.commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE ||
            !BoundaryReservedIsZero(boundary.reserved))
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(boundary.runStartMCS[axis]) != 0ULL ||
                NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(boundary.runEndMCS[axis]) != 0ULL)
            {
                return false;
            }
        }
        return true;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        static bool IsPreviousDisplacementBoundToBoundary(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            displacement,
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1&
            boundary) noexcept
    {
        if (displacement.sourceBoundaryPublicationSequence !=
            boundary.publicationSequence ||
            displacement.sourceRunPublicationSequence !=
            boundary.sourceRunPublicationSequence ||
            displacement.sourcePairPublicationSequence !=
            boundary.sourcePairPublicationSequence ||
            displacement.runGeneration != boundary.runGeneration ||
            displacement.acceptedInputChainGeneration !=
            boundary.acceptedInputChainGeneration ||
            displacement.firstSegmentPublicationSequence !=
            boundary.firstSegmentPublicationSequence ||
            displacement.latestSegmentPublicationSequence !=
            boundary.latestSegmentPublicationSequence ||
            displacement.firstLinkPublicationSequence !=
            boundary.firstLinkPublicationSequence ||
            displacement.latestLinkPublicationSequence !=
            boundary.latestLinkPublicationSequence ||
            displacement.runStartGeometryPublicationSequence !=
            boundary.runStartGeometryPublicationSequence ||
            displacement.runEndGeometryPublicationSequence !=
            boundary.runEndGeometryPublicationSequence ||
            displacement.runStartGeometryMotionSegmentId !=
            boundary.runStartGeometryMotionSegmentId ||
            displacement.runEndGeometryMotionSegmentId !=
            boundary.runEndGeometryMotionSegmentId ||
            displacement.linkedSegmentRunLength !=
            boundary.linkedSegmentRunLength ||
            displacement.participatingAxisUnionMask !=
            boundary.participatingAxisUnionMask ||
            displacement.coordinateChangeAxisUnionMask !=
            boundary.coordinateChangeAxisUnionMask ||
            displacement.endpointDisplacementAxisMask !=
            boundary.endpointDisplacementAxisMask ||
            displacement.firstSourcePC != boundary.firstSourcePC ||
            displacement.latestSourcePC != boundary.latestSourcePC)
        {
            return false;
        }

        const bool relationMatches =
            (boundary.disposition ==
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED &&
                displacement.displacementRelation ==
                NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
                PROVEN_TWO_SEGMENT_BOUNDARY_DERIVES_NET_DISPLACEMENT) ||
            (boundary.disposition ==
                NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
                PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED &&
                displacement.displacementRelation ==
                NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
                PROVEN_BOUNDARY_ADVANCE_REDERIVES_RUN_NET_DISPLACEMENT);
        if (!relationMatches)
        {
            return false;
        }

        std::uint32_t positiveAxisMask = 0U;
        std::uint32_t negativeAxisMask = 0U;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            double expected =
                boundary.runEndMCS[axis] - boundary.runStartMCS[axis];
            if (!std::isfinite(expected))
            {
                return false;
            }
            if (expected == 0.0)
            {
                expected = 0.0;
            }

            if (NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(expected) !=
                NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(
                    displacement.endpointNetDisplacementMCS[axis]))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if (expected > 0.0)
            {
                positiveAxisMask |= axisBit;
            }
            else if (expected < 0.0)
            {
                negativeAxisMask |= axisBit;
            }
        }

        return positiveAxisMask ==
            displacement.positiveNetDisplacementAxisMask &&
            negativeAxisMask ==
            displacement.negativeNetDisplacementAxisMask;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        static bool IsDirectBoundaryAdvanceFenceValid(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1&
            previous,
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1&
            current) noexcept
    {
        if (current.disposition !=
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED ||
            current.publicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(previous.publicationSequence) ||
            current.sourceRunPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(
                previous.sourceRunPublicationSequence) ||
            current.sourcePairPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(
                previous.sourcePairPublicationSequence) ||
            current.runGeneration != previous.runGeneration ||
            current.acceptedInputChainGeneration !=
            previous.acceptedInputChainGeneration ||
            current.firstSegmentPublicationSequence !=
            previous.firstSegmentPublicationSequence ||
            current.latestSegmentPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(
                previous.latestSegmentPublicationSequence) ||
            current.firstLinkPublicationSequence !=
            previous.firstLinkPublicationSequence ||
            current.latestLinkPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(
                previous.latestLinkPublicationSequence) ||
            current.runStartGeometryPublicationSequence !=
            previous.runStartGeometryPublicationSequence ||
            current.runEndGeometryPublicationSequence !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            NextNonZeroSequence(
                previous.runEndGeometryPublicationSequence) ||
            current.runStartGeometryMotionSegmentId !=
            previous.runStartGeometryMotionSegmentId ||
            current.runEndGeometryMotionSegmentId <=
            previous.runEndGeometryMotionSegmentId ||
            current.linkedSegmentRunLength !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            SaturatingIncrement(previous.linkedSegmentRunLength) ||
            current.firstAcceptedInputRunLength !=
            previous.firstAcceptedInputRunLength ||
            current.latestAcceptedInputRunLength !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
            SaturatingIncrement(
                previous.latestAcceptedInputRunLength) ||
            current.firstSourcePC != previous.firstSourcePC ||
            previous.latestSourcePC ==
            (std::numeric_limits<std::int32_t>::max)() ||
            current.latestSourcePC != previous.latestSourcePC + 1 ||
            current.firstSegmentAxisMask != previous.firstSegmentAxisMask ||
            (previous.participatingAxisUnionMask &
                ~current.participatingAxisUnionMask) != 0U ||
            (previous.coordinateChangeAxisUnionMask &
                ~current.coordinateChangeAxisUnionMask) != 0U)
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(previous.runStartMCS[axis]) !=
                NCPathCoreLinkedCommittedSegmentRunDisplacementDetail::
                DoubleObjectBits(current.runStartMCS[axis]))
            {
                return false;
            }
        }
        return true;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE
        static bool PopulateFromBoundary(
            const NCPathCoreLinkedCommittedSegmentRunBoundaryRecordV1&
            boundary,
            NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1& target)
        noexcept
    {
        target.runGeneration = boundary.runGeneration;
        target.acceptedInputChainGeneration =
            boundary.acceptedInputChainGeneration;
        target.firstSegmentPublicationSequence =
            boundary.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            boundary.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            boundary.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            boundary.latestLinkPublicationSequence;
        target.runStartGeometryPublicationSequence =
            boundary.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            boundary.runEndGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            boundary.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            boundary.runEndGeometryMotionSegmentId;
        target.linkedSegmentRunLength = boundary.linkedSegmentRunLength;
        target.participatingAxisUnionMask =
            boundary.participatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            boundary.coordinateChangeAxisUnionMask;
        target.endpointDisplacementAxisMask =
            boundary.endpointDisplacementAxisMask;
        target.firstSourcePC = boundary.firstSourcePC;
        target.latestSourcePC = boundary.latestSourcePC;

        std::uint32_t observedEndpointDisplacementAxisMask = 0U;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            double displacement =
                boundary.runEndMCS[axis] - boundary.runStartMCS[axis];
            if (!std::isfinite(displacement))
            {
                return false;
            }
            if (displacement == 0.0)
            {
                displacement = 0.0;
            }
            else
            {
                const std::uint32_t axisBit =
                    static_cast<std::uint32_t>(1U << axis);
                observedEndpointDisplacementAxisMask |= axisBit;
                if (displacement > 0.0)
                {
                    target.positiveNetDisplacementAxisMask |= axisBit;
                }
                else
                {
                    target.negativeNetDisplacementAxisMask |= axisBit;
                }
            }
            target.endpointNetDisplacementMCS[axis] = displacement;
        }

        if (observedEndpointDisplacementAxisMask !=
            boundary.endpointDisplacementAxisMask)
        {
            return false;
        }

        if (boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_STARTED)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED;
            target.displacementRelation =
                NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
                PROVEN_TWO_SEGMENT_BOUNDARY_DERIVES_NET_DISPLACEMENT;
        }
        else if (boundary.disposition ==
            NCPathCoreLinkedCommittedSegmentRunBoundaryDisposition::
            PROVEN_RUN_ENDPOINT_BOUNDARY_EXTENDED)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED;
            target.displacementRelation =
                NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::
                PROVEN_BOUNDARY_ADVANCE_REDERIVES_RUN_NET_DISPLACEMENT;
        }
        else
        {
            return false;
        }

        target.extent =
            NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::
            RUN_HEAD_TO_CURRENT_TAIL_COMPONENT_DISPLACEMENT_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
        return target.IsProvenRunEndpointNetDisplacement();
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition) == 1U,
    "Run net-displacement disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisplacementRelation) == 1U,
    "Run net-displacement relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisplacementExtent) == 1U,
    "Run net-displacement extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1>::value,
    "Run net-displacement record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1>::value,
    "Run net-displacement record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1) == 8U,
    "Run net-displacement record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1) == 224U,
    "Run net-displacement record must remain exactly 224 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1,
        endpointNetDisplacementMCS) == 112U,
    "Run net-displacement array offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1,
        linkedSegmentRunLength) == 176U,
    "Run net-displacement run-length offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1,
        schemaVersion) == 208U,
    "Run net-displacement schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1,
        reserved) == 216U,
    "Run net-displacement reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunDisplacementShadow>::value,
    "Run net-displacement shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunDisplacementShadow>::value,
    "Run net-displacement shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunDisplacementShadow) == 464U,
    "Run net-displacement shadow must remain exactly 464 bytes.");
