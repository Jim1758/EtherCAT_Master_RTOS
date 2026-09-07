#pragma once

#include "NCPathCoreCommittedGeometryLinkShadow.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================
// NC-0.2L.2F / Linked Committed Path Segment Geometry Shadow
//
// This fixed two-record history forms a minimal commanded endpoint-segment
// descriptor only when the newest L.2E observation has already proven an
// immediate committed geometry link.  L.2F consumes that proof; it does not
// independently recreate the L.2E endpoint-seam or queue-tail-seal decision.
//
// "Segment" here means only the current ordinary no-P G00 commanded start/end
// descriptor in the MCS axis-native frame.  It does not assert the Motion
// interpolation locus between those endpoints and carries no distance,
// direction, tangent, rotary-unwrapping, timing, velocity, actual-position,
// execution, completion or B2 breadcrumb meaning.
//
// A non-proven or malformed link publishes a neutral/invalid newest record
// with no geometry payload.  The shadow is same-thread, fixed-capacity,
// heap-resident through NCManager, and is never a queue, permit, Gate, Alarm,
// PC decision, Motion command source or HMI/SHM/API publication.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_SCHEMA_V1 = 1U;

enum class NCPathCoreLinkedCommittedSegmentDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_FORMED_LINK_NOT_PROVEN = 1U,
    INVALID_PROOF_BINDING = 2U,
    INVALID_SEGMENT_GEOMETRY = 3U,
    FORMED_LINKED_COMMANDED_ENDPOINT_SEGMENT = 4U
};

enum class NCPathCoreLinkedCommittedSegmentRelation : std::uint8_t
{
    NONE = 0U,
    IMMEDIATE_PREDECESSOR_EXACT_COMMANDED_TAIL_LINK = 1U
};

enum class NCPathCoreLinkedCommittedSegmentExtent : std::uint8_t
{
    NONE = 0U,
    CURRENT_COMMANDED_ENDPOINT_SEGMENT_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t linkPublicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t previousGeometryPublicationSequence = 0ULL;
    std::uint64_t currentGeometryPublicationSequence = 0ULL;
    std::uint64_t previousMotionSegmentId = 0ULL;
    std::uint64_t currentMotionSegmentId = 0ULL;

    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> startMCS{};
    std::array<
        double,
        NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY> endMCS{};

    std::uint32_t previousAxisMask = 0U;
    std::uint32_t currentAxisMask = 0U;
    std::uint32_t coordinateChangeAxisMask = 0U;
    std::uint32_t acceptedInputRunLength = 0U;
    std::int32_t sourcePC = -1;
    std::int32_t sourceLineNumber = 0;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentDisposition disposition =
        NCPathCoreLinkedCommittedSegmentDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentRelation predecessorRelation =
        NCPathCoreLinkedCommittedSegmentRelation::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    NCPathCoreLinkedCommittedSegmentExtent extent =
        NCPathCoreLinkedCommittedSegmentExtent::NONE;

    bool IsFormedLinkedCommandedEndpointSegment() const noexcept
    {
        if (publicationSequence == 0ULL ||
            linkPublicationSequence == 0ULL ||
            schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_SCHEMA_V1 ||
            disposition != NCPathCoreLinkedCommittedSegmentDisposition::
            FORMED_LINKED_COMMANDED_ENDPOINT_SEGMENT ||
            predecessorRelation !=
            NCPathCoreLinkedCommittedSegmentRelation::
            IMMEDIATE_PREDECESSOR_EXACT_COMMANDED_TAIL_LINK ||
            frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
            kind != NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR ||
            commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP ||
            extent != NCPathCoreLinkedCommittedSegmentExtent::
            CURRENT_COMMANDED_ENDPOINT_SEGMENT_ONLY ||
            acceptedInputChainGeneration == 0ULL ||
            previousGeometryPublicationSequence == 0ULL ||
            currentGeometryPublicationSequence !=
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previousGeometryPublicationSequence) ||
            previousMotionSegmentId == 0ULL ||
            currentMotionSegmentId <= previousMotionSegmentId ||
            previousAxisMask == 0U ||
            currentAxisMask == 0U ||
            (previousAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_AXIS_MASK) != 0U ||
            (currentAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_AXIS_MASK) != 0U ||
            (coordinateChangeAxisMask & ~currentAxisMask) != 0U ||
            acceptedInputRunLength < 2U ||
            sourcePC < 0)
        {
            return false;
        }

        std::uint32_t observedChangeAxisMask = 0U;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(startMCS[axis]) ||
                !std::isfinite(endMCS[axis]))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if (startMCS[axis] != endMCS[axis])
            {
                observedChangeAxisMask |= axisBit;
            }
        }

        return observedChangeAxisMask == coordinateChangeAxisMask;
    }
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_NOINLINE
        void ObserveLinkedCommittedPairSameThread(
            const NCPathCoreCommittedGeometryLinkRecordV1* link,
            const NCPathCoreOrdinaryG00CommittedEndpointPairV1* previous,
            const NCPathCoreOrdinaryG00CommittedEndpointPairV1* current) noexcept
    {
        m_publicationSequence =
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY;

        // Write directly into the NCManager-owned fixed workspace.  No record,
        // endpoint array or upstream L.2D/L.2E object is copied onto the NC
        // thread stack.
        NCPathCoreLinkedCommittedSegmentRecordV1& target =
            m_records[targetIndex];
        target.publicationSequence = m_publicationSequence;
        target.linkPublicationSequence =
            link == nullptr ? 0ULL : link->publicationSequence;
        target.acceptedInputChainGeneration =
            current == nullptr
            ? 0ULL
            : current->acceptedInputChainGeneration;
        target.previousGeometryPublicationSequence =
            previous == nullptr ? 0ULL : previous->publicationSequence;
        target.currentGeometryPublicationSequence =
            current == nullptr ? 0ULL : current->publicationSequence;
        target.previousMotionSegmentId =
            previous == nullptr ? 0ULL : previous->motionSegmentId;
        target.currentMotionSegmentId =
            current == nullptr ? 0ULL : current->motionSegmentId;

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            // Invalid/neutral observations carry no geometry payload and can
            // never expose an older formed segment through a recycled slot.
            target.startMCS[axis] = 0.0;
            target.endMCS[axis] = 0.0;
        }

        target.previousAxisMask =
            previous == nullptr ? 0U : previous->axisMask;
        target.currentAxisMask = current == nullptr ? 0U : current->axisMask;
        target.coordinateChangeAxisMask = 0U;
        target.acceptedInputRunLength =
            current == nullptr ? 0U : current->acceptedInputRunLength;
        target.sourcePC = current == nullptr ? -1 : current->sourcePC;
        target.sourceLineNumber =
            current == nullptr ? 0 : current->sourceLineNumber;
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentDisposition::
            NOT_FORMED_LINK_NOT_PROVEN;
        target.predecessorRelation =
            NCPathCoreLinkedCommittedSegmentRelation::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.extent = NCPathCoreLinkedCommittedSegmentExtent::NONE;

        if (link == nullptr ||
            !link->IsProvenContiguousCommandedTailLink())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentDisposition::
                NOT_FORMED_LINK_NOT_PROVEN;
        }
        else if (!IsProofBindingValid(*link, previous, current))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentDisposition::
                INVALID_PROOF_BINDING;
        }
        else if (!IsCurrentSegmentGeometryValid(*current))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentDisposition::
                INVALID_SEGMENT_GEOMETRY;
        }
        else
        {
            for (std::size_t axis = 0U;
                axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
                ++axis)
            {
                target.startMCS[axis] = current->startMCS[axis];
                target.endMCS[axis] = current->endMCS[axis];
                if (current->startMCS[axis] != current->endMCS[axis])
                {
                    target.coordinateChangeAxisMask |=
                        static_cast<std::uint32_t>(1U << axis);
                }
            }

            target.disposition =
                NCPathCoreLinkedCommittedSegmentDisposition::
                FORMED_LINKED_COMMANDED_ENDPOINT_SEGMENT;
            target.predecessorRelation =
                NCPathCoreLinkedCommittedSegmentRelation::
                IMMEDIATE_PREDECESSOR_EXACT_COMMANDED_TAIL_LINK;
            target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
            target.kind = NCPathCoreCommittedGeometryKind::
                ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
            target.commandPathPolicy =
                NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
            target.extent = NCPathCoreLinkedCommittedSegmentExtent::
                CURRENT_COMMANDED_ENDPOINT_SEGMENT_ONLY;
        }

        // Neutral and invalid observations become the newest record so a stale
        // formed segment can never masquerade as the current relationship.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentRecordV1*
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
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static bool IsProofBindingValid(
        const NCPathCoreCommittedGeometryLinkRecordV1& link,
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1* previous,
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1* current) noexcept
    {
        if (previous == nullptr || current == nullptr)
        {
            return false;
        }

        return
            previous->IsAcceptedCommandCommitted() &&
            current->IsAcceptedCommandCommitted() &&
            link.acceptedInputChainGeneration ==
            previous->acceptedInputChainGeneration &&
            link.acceptedInputChainGeneration ==
            current->acceptedInputChainGeneration &&
            link.previousGeometryPublicationSequence ==
            previous->publicationSequence &&
            link.currentGeometryPublicationSequence ==
            current->publicationSequence &&
            link.previousMotionSegmentId == previous->motionSegmentId &&
            link.currentMotionSegmentId == current->motionSegmentId &&
            link.previousQueueTailTransactionSequence ==
            previous->queueTailTransactionSequence &&
            link.currentQueueTailTransactionSequence ==
            current->queueTailTransactionSequence &&
            link.previousQueueTailCommittedFingerprint ==
            previous->queueTailCommittedFingerprint &&
            link.currentQueueTailBeforeFingerprint ==
            current->queueTailBeforeFingerprint &&
            link.previousAxisMask == previous->axisMask &&
            link.currentAxisMask == current->axisMask &&
            link.acceptedInputPairRelation ==
            current->acceptedInputPairRelation;
    }

    static bool IsCurrentSegmentGeometryValid(
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& current) noexcept
    {
        if (!current.IsAcceptedCommandCommitted() ||
            current.acceptedInputPairRelation !=
            NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR ||
            current.acceptedInputRunLength < 2U ||
            current.acceptedInputChainGeneration == 0ULL ||
            current.axisMask == 0U ||
            (current.axisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_AXIS_MASK) != 0U ||
            current.sourcePC < 0)
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(current.startMCS[axis]) ||
                !std::isfinite(current.endMCS[axis]))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if ((current.axisMask & axisBit) == 0U &&
                current.startMCS[axis] != current.endMCS[axis])
            {
                return false;
            }
        }

        return true;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentDisposition) == 1U,
    "Linked committed segment disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRelation) == 1U,
    "Linked committed segment relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentExtent) == 1U,
    "Linked committed segment extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentRecordV1>::value,
    "Linked committed segment record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentRecordV1>::value,
    "Linked committed segment record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentRecordV1) == 8U,
    "Linked committed segment record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentRecordV1) == 216U,
    "Linked committed segment record must remain exactly 216 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRecordV1,
        startMCS) == 56U,
    "Linked committed segment start offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRecordV1,
        endMCS) == 120U,
    "Linked committed segment end offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRecordV1,
        previousAxisMask) == 184U,
    "Linked committed segment mask offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentRecordV1,
        schemaVersion) == 208U,
    "Linked committed segment schema offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentShadow>::value,
    "Linked committed segment shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentShadow>::value,
    "Linked committed segment shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentShadow) == 448U,
    "Linked committed segment shadow must remain exactly 448 bytes.");
