#pragma once

#include "NCPathCoreLinkedCommittedSegmentRunDisplacementShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// =============================================================
// NC-0.2L.2K / Proven Linked Committed Segment Run Endpoint-
// Closure and Returned-Axis Relation Shadow
//
// This fixed two-record observer consumes only the newest L.2J endpoint net-
// displacement observation and, for an extending run, the immediately
// preceding L.2J observation.  A proven record classifies the current
// commanded MCS relationship between the proven run head and current tail.
//
// The classification distinguishes:
//   * participating axes which never changed coordinate in this run,
//   * axes whose current run endpoint remains open from the run head, and
//   * axes which changed coordinate in at least one committed segment but
//     whose current run endpoint is numerically back at the run head.
//
// "Returned" is only an exact commanded-endpoint relation derived from L.2J
// and L.2H provenance.  It is not actual Motion execution, travelled distance,
// a detected physical reversal, a closed interpolated contour, retrace
// capacity or a B2 breadcrumb.  The record stores scalar masks only: no
// endpoint array, displacement array, intermediate point or segment list.
//
// L.2K is not a Path Queue and has no Gate, PC, Alarm, Motion, HMI/SHM/API,
// PDO, EtherCAT or DC consumer.  Neutral and invalid observations clear all
// proven payload before becoming newest history.  The shadow is same-thread,
// fixed-capacity and heap-resident through NCManager, with no allocation,
// logging, waiting or synchronization.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_SCHEMA_V1 = 1U;

namespace NCPathCoreLinkedCommittedSegmentRunClosureDetail
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
            "L.2K requires a 64-bit double object representation.");
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

enum class NCPathCoreLinkedCommittedSegmentRunClosureDisposition :
    std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_DISPLACEMENT_UNAVAILABLE = 1U,
    NOT_APPLICABLE_DISPLACEMENT_NOT_PROVEN = 2U,
    INVALID_CURRENT_DISPLACEMENT_RECORD = 3U,
    INVALID_PREVIOUS_DISPLACEMENT_RECORD = 4U,
    INVALID_PREVIOUS_CLOSURE_RECORD = 5U,
    INVALID_DISPLACEMENT_ADVANCE_FENCE = 6U,
    INVALID_CLOSURE_CLASSIFICATION = 7U,
    PROVEN_RUN_ENDPOINT_CLOSURE_STARTED = 8U,
    PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED = 9U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureRelation :
    std::uint8_t
{
    NONE = 0U,
    PROVEN_TWO_SEGMENT_RUN_CLASSIFIED_FROM_NET_DISPLACEMENT = 1U,
    PROVEN_EXTENDED_RUN_RECLASSIFIED_FROM_NET_DISPLACEMENT = 2U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureExtent :
    std::uint8_t
{
    NONE = 0U,
    RUN_HEAD_TO_CURRENT_TAIL_CLOSURE_MASKS_ONLY = 1U
};

enum class NCPathCoreLinkedCommittedSegmentRunClosureClass :
    std::uint8_t
{
    NONE = 0U,
    NO_COMMANDED_COORDINATE_CHANGE = 1U,
    ENDPOINT_OPEN_NO_RETURNED_AXIS = 2U,
    ENDPOINT_OPEN_WITH_RETURNED_AXIS = 3U,
    FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE = 4U
};

struct NCPathCoreLinkedCommittedSegmentRunClosureRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t sourceDisplacementPublicationSequence = 0ULL;
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

    std::uint32_t linkedSegmentRunLength = 0U;
    std::uint32_t participatingAxisUnionMask = 0U;
    std::uint32_t coordinateChangeAxisUnionMask = 0U;
    std::uint32_t endpointOpenAxisMask = 0U;
    std::uint32_t endpointReturnedAxisMask = 0U;
    std::uint32_t coordinateUnchangedParticipatingAxisMask = 0U;
    std::uint32_t positiveNetDisplacementAxisMask = 0U;
    std::uint32_t negativeNetDisplacementAxisMask = 0U;
    std::int32_t firstSourcePC = -1;
    std::int32_t latestSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentRunClosureDisposition disposition =
        NCPathCoreLinkedCommittedSegmentRunClosureDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRunClosureRelation closureRelation =
        NCPathCoreLinkedCommittedSegmentRunClosureRelation::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureExtent extent =
        NCPathCoreLinkedCommittedSegmentRunClosureExtent::NONE;
    NCPathCoreLinkedCommittedSegmentRunClosureClass closureClass =
        NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 7U> reserved{};

    bool IsProvenRunEndpointClosureRelation() const noexcept
    {
        const bool started =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            PROVEN_RUN_ENDPOINT_CLOSURE_STARTED &&
            closureRelation ==
            NCPathCoreLinkedCommittedSegmentRunClosureRelation::
            PROVEN_TWO_SEGMENT_RUN_CLASSIFIED_FROM_NET_DISPLACEMENT;
        const bool extended =
            disposition ==
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED &&
            closureRelation ==
            NCPathCoreLinkedCommittedSegmentRunClosureRelation::
            PROVEN_EXTENDED_RUN_RECLASSIFIED_FROM_NET_DISPLACEMENT;

        if (publicationSequence == 0ULL ||
            sourceDisplacementPublicationSequence == 0ULL ||
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
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_AXIS_MASK) !=
            0U ||
            (coordinateChangeAxisUnionMask &
                ~participatingAxisUnionMask) != 0U ||
            (endpointOpenAxisMask &
                ~coordinateChangeAxisUnionMask) != 0U ||
            (endpointReturnedAxisMask &
                ~coordinateChangeAxisUnionMask) != 0U ||
            (endpointOpenAxisMask & endpointReturnedAxisMask) != 0U ||
            endpointReturnedAxisMask !=
            (coordinateChangeAxisUnionMask & ~endpointOpenAxisMask) ||
            coordinateUnchangedParticipatingAxisMask !=
            (participatingAxisUnionMask &
                ~coordinateChangeAxisUnionMask) ||
            (positiveNetDisplacementAxisMask &
                ~endpointOpenAxisMask) != 0U ||
            (negativeNetDisplacementAxisMask &
                ~endpointOpenAxisMask) != 0U ||
            (positiveNetDisplacementAxisMask &
                negativeNetDisplacementAxisMask) != 0U ||
            (positiveNetDisplacementAxisMask |
                negativeNetDisplacementAxisMask) != endpointOpenAxisMask ||
            firstSourcePC < 0 ||
            latestSourcePC < firstSourcePC ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_SCHEMA_V1 ||
            (!started && !extended) ||
            extent !=
            NCPathCoreLinkedCommittedSegmentRunClosureExtent::
            RUN_HEAD_TO_CURRENT_TAIL_CLOSURE_MASKS_ONLY ||
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
        if (NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            ForwardNonZeroSequenceDistance(
                firstSegmentPublicationSequence,
                latestSegmentPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            ForwardNonZeroSequenceDistance(
                firstLinkPublicationSequence,
                latestLinkPublicationSequence) != runLength - 1ULL ||
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            ForwardNonZeroSequenceDistance(
                runStartGeometryPublicationSequence,
                runEndGeometryPublicationSequence) != runLength)
        {
            return false;
        }

        const bool noCoordinateChange =
            coordinateChangeAxisUnionMask == 0U &&
            endpointOpenAxisMask == 0U &&
            endpointReturnedAxisMask == 0U &&
            closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            NO_COMMANDED_COORDINATE_CHANGE;
        const bool openWithoutReturnedAxis =
            endpointOpenAxisMask != 0U &&
            endpointReturnedAxisMask == 0U &&
            closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            ENDPOINT_OPEN_NO_RETURNED_AXIS;
        const bool openWithReturnedAxis =
            endpointOpenAxisMask != 0U &&
            endpointReturnedAxisMask != 0U &&
            closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            ENDPOINT_OPEN_WITH_RETURNED_AXIS;
        const bool fullEndpointClosure =
            coordinateChangeAxisUnionMask != 0U &&
            endpointOpenAxisMask == 0U &&
            endpointReturnedAxisMask == coordinateChangeAxisUnionMask &&
            closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;

        if (!noCoordinateChange &&
            !openWithoutReturnedAxis &&
            !openWithReturnedAxis &&
            !fullEndpointClosure)
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
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentRunClosureShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentRunClosureShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        void ObserveLatestDisplacementSameThread(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
            previousDisplacement,
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
            currentDisplacement) noexcept
    {
        const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1* const
            previousClosure = GetNewestObservationSameThread();

        m_publicationSequence =
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY;

        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target =
            m_records[targetIndex];
        ResetTarget(
            target,
            m_publicationSequence,
            currentDisplacement == nullptr
            ? 0ULL
            : currentDisplacement->publicationSequence,
            currentDisplacement == nullptr
            ? 0ULL
            : currentDisplacement->sourceBoundaryPublicationSequence,
            currentDisplacement == nullptr
            ? 0ULL
            : currentDisplacement->sourceRunPublicationSequence,
            currentDisplacement == nullptr
            ? 0ULL
            : currentDisplacement->sourcePairPublicationSequence);

        EvaluateObservation(
            previousDisplacement,
            currentDisplacement,
            previousClosure,
            target);

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
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
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        static void EvaluateObservation(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
            previousDisplacement,
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1*
            currentDisplacement,
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1*
            previousClosure,
            NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target)
        noexcept
    {
        if (currentDisplacement == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                NOT_APPLICABLE_DISPLACEMENT_UNAVAILABLE;
        }
        else if (IsStructurallyValidNonProvenDisplacement(
            *currentDisplacement))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                NOT_APPLICABLE_DISPLACEMENT_NOT_PROVEN;
        }
        else if (!currentDisplacement->
            IsProvenRunEndpointNetDisplacement())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                INVALID_CURRENT_DISPLACEMENT_RECORD;
        }
        else if (currentDisplacement->disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED)
        {
            if (!PopulateFromDisplacement(*currentDisplacement, target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                    INVALID_CLOSURE_CLASSIFICATION;
            }
        }
        else if (currentDisplacement->disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED)
        {
            if (previousDisplacement == nullptr ||
                !previousDisplacement->
                IsProvenRunEndpointNetDisplacement())
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                    INVALID_PREVIOUS_DISPLACEMENT_RECORD;
            }
            else if (previousClosure == nullptr ||
                !previousClosure->IsProvenRunEndpointClosureRelation() ||
                !IsPreviousClosureBoundToDisplacement(
                    *previousClosure,
                    *previousDisplacement))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                    INVALID_PREVIOUS_CLOSURE_RECORD;
            }
            else if (!IsDirectDisplacementAdvanceFenceValid(
                *previousDisplacement,
                *currentDisplacement))
            {
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                    INVALID_DISPLACEMENT_ADVANCE_FENCE;
            }
            else if (!PopulateFromDisplacement(
                *currentDisplacement,
                target))
            {
                ClearProvenPayload(target);
                target.disposition =
                    NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                    INVALID_CLOSURE_CLASSIFICATION;
            }
        }
        else
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                INVALID_CURRENT_DISPLACEMENT_RECORD;
        }
    }

    static void ResetTarget(
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target,
        std::uint64_t publicationSequence,
        std::uint64_t sourceDisplacementPublicationSequence,
        std::uint64_t sourceBoundaryPublicationSequence,
        std::uint64_t sourceRunPublicationSequence,
        std::uint64_t sourcePairPublicationSequence) noexcept
    {
        target.publicationSequence = publicationSequence;
        target.sourceDisplacementPublicationSequence =
            sourceDisplacementPublicationSequence;
        target.sourceBoundaryPublicationSequence =
            sourceBoundaryPublicationSequence;
        target.sourceRunPublicationSequence = sourceRunPublicationSequence;
        target.sourcePairPublicationSequence = sourcePairPublicationSequence;
        ClearProvenPayload(target);
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
            NOT_APPLICABLE_DISPLACEMENT_UNAVAILABLE;
        target.closureRelation =
            NCPathCoreLinkedCommittedSegmentRunClosureRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureExtent::NONE;
        target.closureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);
    }

    static void ClearProvenPayload(
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target) noexcept
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
        target.linkedSegmentRunLength = 0U;
        target.participatingAxisUnionMask = 0U;
        target.coordinateChangeAxisUnionMask = 0U;
        target.endpointOpenAxisMask = 0U;
        target.endpointReturnedAxisMask = 0U;
        target.coordinateUnchangedParticipatingAxisMask = 0U;
        target.positiveNetDisplacementAxisMask = 0U;
        target.negativeNetDisplacementAxisMask = 0U;
        target.firstSourcePC = -1;
        target.latestSourcePC = -1;
        target.closureRelation =
            NCPathCoreLinkedCommittedSegmentRunClosureRelation::NONE;
        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureExtent::NONE;
        target.closureClass =
            NCPathCoreLinkedCommittedSegmentRunClosureClass::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    }

    static bool DisplacementReservedIsZero(
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

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        static bool IsStructurallyValidNonProvenDisplacement(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            displacement) noexcept
    {
        const bool knownDisposition =
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            NOT_APPLICABLE_BOUNDARY_UNAVAILABLE ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            NOT_APPLICABLE_BOUNDARY_NOT_PROVEN ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            INVALID_CURRENT_BOUNDARY_RECORD ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            INVALID_PREVIOUS_BOUNDARY_RECORD ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            INVALID_PREVIOUS_DISPLACEMENT_RECORD ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            INVALID_BOUNDARY_ADVANCE_FENCE ||
            displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            INVALID_NET_DISPLACEMENT_GEOMETRY;

        if (!knownDisposition ||
            displacement.publicationSequence == 0ULL ||
            displacement.schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_DISPLACEMENT_SCHEMA_V1 ||
            displacement.runGeneration != 0ULL ||
            displacement.acceptedInputChainGeneration != 0ULL ||
            displacement.firstSegmentPublicationSequence != 0ULL ||
            displacement.latestSegmentPublicationSequence != 0ULL ||
            displacement.firstLinkPublicationSequence != 0ULL ||
            displacement.latestLinkPublicationSequence != 0ULL ||
            displacement.runStartGeometryPublicationSequence != 0ULL ||
            displacement.runEndGeometryPublicationSequence != 0ULL ||
            displacement.runStartGeometryMotionSegmentId != 0ULL ||
            displacement.runEndGeometryMotionSegmentId != 0ULL ||
            displacement.linkedSegmentRunLength != 0U ||
            displacement.participatingAxisUnionMask != 0U ||
            displacement.coordinateChangeAxisUnionMask != 0U ||
            displacement.endpointDisplacementAxisMask != 0U ||
            displacement.positiveNetDisplacementAxisMask != 0U ||
            displacement.negativeNetDisplacementAxisMask != 0U ||
            displacement.firstSourcePC != -1 ||
            displacement.latestSourcePC != -1 ||
            displacement.displacementRelation !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementRelation::NONE ||
            displacement.extent !=
            NCPathCoreLinkedCommittedSegmentRunDisplacementExtent::NONE ||
            displacement.frame != NCPathCoreCommittedGeometryFrame::NONE ||
            displacement.kind != NCPathCoreCommittedGeometryKind::NONE ||
            displacement.commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE ||
            !DisplacementReservedIsZero(displacement.reserved))
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (NCPathCoreLinkedCommittedSegmentRunClosureDetail::
                DoubleObjectBits(
                    displacement.endpointNetDisplacementMCS[axis]) !=
                0ULL)
            {
                return false;
            }
        }
        return true;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        static bool IsPreviousClosureBoundToDisplacement(
            const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& closure,
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            displacement) noexcept
    {
        if (closure.sourceDisplacementPublicationSequence !=
            displacement.publicationSequence ||
            closure.sourceBoundaryPublicationSequence !=
            displacement.sourceBoundaryPublicationSequence ||
            closure.sourceRunPublicationSequence !=
            displacement.sourceRunPublicationSequence ||
            closure.sourcePairPublicationSequence !=
            displacement.sourcePairPublicationSequence ||
            closure.runGeneration != displacement.runGeneration ||
            closure.acceptedInputChainGeneration !=
            displacement.acceptedInputChainGeneration ||
            closure.firstSegmentPublicationSequence !=
            displacement.firstSegmentPublicationSequence ||
            closure.latestSegmentPublicationSequence !=
            displacement.latestSegmentPublicationSequence ||
            closure.firstLinkPublicationSequence !=
            displacement.firstLinkPublicationSequence ||
            closure.latestLinkPublicationSequence !=
            displacement.latestLinkPublicationSequence ||
            closure.runStartGeometryPublicationSequence !=
            displacement.runStartGeometryPublicationSequence ||
            closure.runEndGeometryPublicationSequence !=
            displacement.runEndGeometryPublicationSequence ||
            closure.runStartGeometryMotionSegmentId !=
            displacement.runStartGeometryMotionSegmentId ||
            closure.runEndGeometryMotionSegmentId !=
            displacement.runEndGeometryMotionSegmentId ||
            closure.linkedSegmentRunLength !=
            displacement.linkedSegmentRunLength ||
            closure.participatingAxisUnionMask !=
            displacement.participatingAxisUnionMask ||
            closure.coordinateChangeAxisUnionMask !=
            displacement.coordinateChangeAxisUnionMask ||
            closure.endpointOpenAxisMask !=
            displacement.endpointDisplacementAxisMask ||
            closure.positiveNetDisplacementAxisMask !=
            displacement.positiveNetDisplacementAxisMask ||
            closure.negativeNetDisplacementAxisMask !=
            displacement.negativeNetDisplacementAxisMask ||
            closure.firstSourcePC != displacement.firstSourcePC ||
            closure.latestSourcePC != displacement.latestSourcePC ||
            closure.frame != displacement.frame ||
            closure.kind != displacement.kind ||
            closure.commandPathPolicy != displacement.commandPathPolicy)
        {
            return false;
        }

        const std::uint32_t expectedReturnedMask =
            displacement.coordinateChangeAxisUnionMask &
            ~displacement.endpointDisplacementAxisMask;
        const std::uint32_t expectedUnchangedParticipatingMask =
            displacement.participatingAxisUnionMask &
            ~displacement.coordinateChangeAxisUnionMask;
        if (closure.endpointReturnedAxisMask != expectedReturnedMask ||
            closure.coordinateUnchangedParticipatingAxisMask !=
            expectedUnchangedParticipatingMask)
        {
            return false;
        }

        const bool relationMatches =
            (displacement.disposition ==
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED &&
                closure.disposition ==
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                PROVEN_RUN_ENDPOINT_CLOSURE_STARTED &&
                closure.closureRelation ==
                NCPathCoreLinkedCommittedSegmentRunClosureRelation::
                PROVEN_TWO_SEGMENT_RUN_CLASSIFIED_FROM_NET_DISPLACEMENT) ||
            (displacement.disposition ==
                NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
                PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED &&
                closure.disposition ==
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED &&
                closure.closureRelation ==
                NCPathCoreLinkedCommittedSegmentRunClosureRelation::
                PROVEN_EXTENDED_RUN_RECLASSIFIED_FROM_NET_DISPLACEMENT);
        return relationMatches && ClosureClassMatchesMasks(closure);
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        static bool IsDirectDisplacementAdvanceFenceValid(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            previous,
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            current) noexcept
    {
        return
            current.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED &&
            current.publicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(previous.publicationSequence) &&
            current.sourceBoundaryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.sourceBoundaryPublicationSequence) &&
            current.sourceRunPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.sourcePairPublicationSequence) &&
            current.runGeneration == previous.runGeneration &&
            current.acceptedInputChainGeneration ==
            previous.acceptedInputChainGeneration &&
            current.firstSegmentPublicationSequence ==
            previous.firstSegmentPublicationSequence &&
            current.latestSegmentPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.latestSegmentPublicationSequence) &&
            current.firstLinkPublicationSequence ==
            previous.firstLinkPublicationSequence &&
            current.latestLinkPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.latestLinkPublicationSequence) &&
            current.runStartGeometryPublicationSequence ==
            previous.runStartGeometryPublicationSequence &&
            current.runEndGeometryPublicationSequence ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            NextNonZeroSequence(
                previous.runEndGeometryPublicationSequence) &&
            current.runStartGeometryMotionSegmentId ==
            previous.runStartGeometryMotionSegmentId &&
            current.runEndGeometryMotionSegmentId >
            previous.runEndGeometryMotionSegmentId &&
            current.linkedSegmentRunLength ==
            NCPathCoreLinkedCommittedSegmentRunClosureDetail::
            SaturatingIncrement(previous.linkedSegmentRunLength) &&
            (previous.participatingAxisUnionMask &
                ~current.participatingAxisUnionMask) == 0U &&
            (previous.coordinateChangeAxisUnionMask &
                ~current.coordinateChangeAxisUnionMask) == 0U &&
            current.firstSourcePC == previous.firstSourcePC &&
            previous.latestSourcePC !=
            (std::numeric_limits<std::int32_t>::max)() &&
            current.latestSourcePC == previous.latestSourcePC + 1;
    }

    static bool ClosureClassMatchesMasks(
        const NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target)
        noexcept
    {
        if (target.coordinateChangeAxisUnionMask == 0U)
        {
            return target.endpointOpenAxisMask == 0U &&
                target.endpointReturnedAxisMask == 0U &&
                target.closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                NO_COMMANDED_COORDINATE_CHANGE;
        }
        if (target.endpointOpenAxisMask == 0U)
        {
            return target.endpointReturnedAxisMask ==
                target.coordinateChangeAxisUnionMask &&
                target.closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        }
        if (target.endpointReturnedAxisMask == 0U)
        {
            return target.closureClass ==
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                ENDPOINT_OPEN_NO_RETURNED_AXIS;
        }
        return target.closureClass ==
            NCPathCoreLinkedCommittedSegmentRunClosureClass::
            ENDPOINT_OPEN_WITH_RETURNED_AXIS;
    }

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE
        static bool PopulateFromDisplacement(
            const NCPathCoreLinkedCommittedSegmentRunDisplacementRecordV1&
            displacement,
            NCPathCoreLinkedCommittedSegmentRunClosureRecordV1& target)
        noexcept
    {
        target.runGeneration = displacement.runGeneration;
        target.acceptedInputChainGeneration =
            displacement.acceptedInputChainGeneration;
        target.firstSegmentPublicationSequence =
            displacement.firstSegmentPublicationSequence;
        target.latestSegmentPublicationSequence =
            displacement.latestSegmentPublicationSequence;
        target.firstLinkPublicationSequence =
            displacement.firstLinkPublicationSequence;
        target.latestLinkPublicationSequence =
            displacement.latestLinkPublicationSequence;
        target.runStartGeometryPublicationSequence =
            displacement.runStartGeometryPublicationSequence;
        target.runEndGeometryPublicationSequence =
            displacement.runEndGeometryPublicationSequence;
        target.runStartGeometryMotionSegmentId =
            displacement.runStartGeometryMotionSegmentId;
        target.runEndGeometryMotionSegmentId =
            displacement.runEndGeometryMotionSegmentId;
        target.linkedSegmentRunLength = displacement.linkedSegmentRunLength;
        target.participatingAxisUnionMask =
            displacement.participatingAxisUnionMask;
        target.coordinateChangeAxisUnionMask =
            displacement.coordinateChangeAxisUnionMask;
        target.endpointOpenAxisMask =
            displacement.endpointDisplacementAxisMask;
        target.endpointReturnedAxisMask =
            displacement.coordinateChangeAxisUnionMask &
            ~displacement.endpointDisplacementAxisMask;
        target.coordinateUnchangedParticipatingAxisMask =
            displacement.participatingAxisUnionMask &
            ~displacement.coordinateChangeAxisUnionMask;
        target.positiveNetDisplacementAxisMask =
            displacement.positiveNetDisplacementAxisMask;
        target.negativeNetDisplacementAxisMask =
            displacement.negativeNetDisplacementAxisMask;
        target.firstSourcePC = displacement.firstSourcePC;
        target.latestSourcePC = displacement.latestSourcePC;

        if (target.coordinateChangeAxisUnionMask == 0U)
        {
            target.closureClass =
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                NO_COMMANDED_COORDINATE_CHANGE;
        }
        else if (target.endpointOpenAxisMask == 0U)
        {
            target.closureClass =
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                FULL_ENDPOINT_CLOSURE_AFTER_COMMANDED_CHANGE;
        }
        else if (target.endpointReturnedAxisMask == 0U)
        {
            target.closureClass =
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                ENDPOINT_OPEN_NO_RETURNED_AXIS;
        }
        else
        {
            target.closureClass =
                NCPathCoreLinkedCommittedSegmentRunClosureClass::
                ENDPOINT_OPEN_WITH_RETURNED_AXIS;
        }

        if (displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_STARTED)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                PROVEN_RUN_ENDPOINT_CLOSURE_STARTED;
            target.closureRelation =
                NCPathCoreLinkedCommittedSegmentRunClosureRelation::
                PROVEN_TWO_SEGMENT_RUN_CLASSIFIED_FROM_NET_DISPLACEMENT;
        }
        else if (displacement.disposition ==
            NCPathCoreLinkedCommittedSegmentRunDisplacementDisposition::
            PROVEN_RUN_ENDPOINT_NET_DISPLACEMENT_EXTENDED)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentRunClosureDisposition::
                PROVEN_RUN_ENDPOINT_CLOSURE_EXTENDED;
            target.closureRelation =
                NCPathCoreLinkedCommittedSegmentRunClosureRelation::
                PROVEN_EXTENDED_RUN_RECLASSIFIED_FROM_NET_DISPLACEMENT;
        }
        else
        {
            return false;
        }

        target.extent =
            NCPathCoreLinkedCommittedSegmentRunClosureExtent::
            RUN_HEAD_TO_CURRENT_TAIL_CLOSURE_MASKS_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
        return ClosureClassMatchesMasks(target) &&
            target.IsProvenRunEndpointClosureRelation();
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_RUN_CLOSURE_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureDisposition) == 1U,
    "Run closure disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureRelation) == 1U,
    "Run closure relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureExtent) == 1U,
    "Run closure extent must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureClass) == 1U,
    "Run closure class must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureRecordV1>::value,
    "Run closure record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureRecordV1>::value,
    "Run closure record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentRunClosureRecordV1) == 8U,
    "Run closure record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureRecordV1) == 176U,
    "Run closure record must remain exactly 176 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1,
        linkedSegmentRunLength) == 120U,
    "Run closure mask block offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1,
        schemaVersion) == 160U,
    "Run closure schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRunClosureRecordV1,
        reserved) == 169U,
    "Run closure reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRunClosureShadow>::value,
    "Run closure shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRunClosureShadow>::value,
    "Run closure shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRunClosureShadow) == 368U,
    "Run closure shadow must remain exactly 368 bytes.");
