#pragma once

#include "NCPathCoreLinkedCommittedSegmentShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================
// NC-0.2L.2G / Immediate Linked Committed Segment Pair Continuity Shadow
//
// This fixed two-record observer consumes only the two newest L.2F records.
// It classifies whether two independently formed commanded endpoint segments
// are the immediate members of one committed commanded chain.  The shared
// junction is proven only when both L.2F records are valid, their identity and
// accepted-input fences bind directly, and previous.endMCS equals
// current.startMCS numerically across all eight MCS axis-native slots.
//
// L.2G retains only compact relationship evidence.  It does not publish a
// three-point geometry window, accumulate an arbitrary run, create a Path
// Queue, infer interpolation/tangent/direction/timing, observe actual Motion
// execution, or create a B2 breadcrumb.  It is same-thread, fixed-capacity,
// heap-resident through NCManager, and has no Gate, PC, Alarm, Motion,
// HMI/SHM/API, PDO, EtherCAT or DC consumer.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_SCHEMA_V1 = 1U;

enum class NCPathCoreLinkedCommittedSegmentPairDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_INSUFFICIENT_HISTORY = 1U,
    NOT_APPLICABLE_PREVIOUS_SEGMENT_NOT_FORMED = 2U,
    NOT_APPLICABLE_CURRENT_SEGMENT_NOT_FORMED = 3U,
    INVALID_PREVIOUS_SEGMENT_RECORD = 4U,
    INVALID_CURRENT_SEGMENT_RECORD = 5U,
    INVALID_DIRECT_PAIR_FENCE = 6U,
    NOT_PROVEN_JUNCTION_GEOMETRY_MISMATCH = 7U,
    PROVEN_IMMEDIATE_LINKED_SEGMENT_CONTINUATION = 8U
};

enum class NCPathCoreLinkedCommittedSegmentJunctionNumericRelation :
    std::uint8_t
{
    NOT_EVALUATED = 0U,
    EXACT_NUMERIC_AND_REPRESENTATION = 1U,
    EXACT_NUMERIC_REPRESENTATION_VARIANT = 2U,
    NUMERIC_MISMATCH = 3U
};

enum class NCPathCoreLinkedCommittedSegmentPairRelation : std::uint8_t
{
    NONE = 0U,
    IMMEDIATE_SHARED_COMMITTED_GEOMETRY_JUNCTION = 1U
};

enum class NCPathCoreLinkedCommittedSegmentPairExtent : std::uint8_t
{
    NONE = 0U,
    TWO_CONSECUTIVE_COMMANDED_SEGMENTS_RELATION_ONLY = 1U
};

struct NCPathCoreLinkedCommittedSegmentPairRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousSegmentPublicationSequence = 0ULL;
    std::uint64_t currentSegmentPublicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t previousLinkPublicationSequence = 0ULL;
    std::uint64_t currentLinkPublicationSequence = 0ULL;
    std::uint64_t previousTerminalGeometryPublicationSequence = 0ULL;
    std::uint64_t currentPredecessorGeometryPublicationSequence = 0ULL;
    std::uint64_t previousTerminalMotionSegmentId = 0ULL;
    std::uint64_t currentPredecessorMotionSegmentId = 0ULL;

    std::uint32_t previousTerminalAxisMask = 0U;
    std::uint32_t currentPredecessorAxisMask = 0U;
    std::uint32_t currentSegmentAxisMask = 0U;
    std::uint32_t previousCoordinateChangeAxisMask = 0U;
    std::uint32_t currentCoordinateChangeAxisMask = 0U;
    std::uint32_t junctionNumericMismatchAxisMask = 0U;
    std::uint32_t junctionRepresentationDifferenceAxisMask = 0U;
    std::uint32_t previousAcceptedInputRunLength = 0U;
    std::uint32_t currentAcceptedInputRunLength = 0U;
    std::int32_t previousSourcePC = -1;
    std::int32_t currentSourcePC = -1;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreLinkedCommittedSegmentPairDisposition disposition =
        NCPathCoreLinkedCommittedSegmentPairDisposition::EMPTY;
    NCPathCoreLinkedCommittedSegmentJunctionNumericRelation numericRelation =
        NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
        NOT_EVALUATED;
    NCPathCoreLinkedCommittedSegmentPairRelation pairRelation =
        NCPathCoreLinkedCommittedSegmentPairRelation::NONE;
    NCPathCoreLinkedCommittedSegmentPairExtent extent =
        NCPathCoreLinkedCommittedSegmentPairExtent::NONE;
    NCPathCoreCommittedGeometryFrame frame =
        NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommittedGeometryKind kind =
        NCPathCoreCommittedGeometryKind::NONE;
    NCPathCoreCommittedGeometryCommandPathPolicy commandPathPolicy =
        NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    bool IsProvenImmediateLinkedSegmentContinuation() const noexcept
    {
        const bool numericRelationProvesExactJunction =
            (numericRelation ==
                NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
                EXACT_NUMERIC_AND_REPRESENTATION &&
                junctionRepresentationDifferenceAxisMask == 0U) ||
            (numericRelation ==
                NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
                EXACT_NUMERIC_REPRESENTATION_VARIANT &&
                junctionRepresentationDifferenceAxisMask != 0U);

        const bool sourcePcContiguous =
            previousSourcePC >= 0 &&
            currentSourcePC >= 0 &&
            static_cast<std::int64_t>(currentSourcePC) ==
            static_cast<std::int64_t>(previousSourcePC) + 1LL;

        return
            publicationSequence != 0ULL &&
            schemaVersion ==
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_SCHEMA_V1 &&
            disposition == NCPathCoreLinkedCommittedSegmentPairDisposition::
            PROVEN_IMMEDIATE_LINKED_SEGMENT_CONTINUATION &&
            numericRelationProvesExactJunction &&
            junctionNumericMismatchAxisMask == 0U &&
            (junctionRepresentationDifferenceAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK) == 0U &&
            pairRelation == NCPathCoreLinkedCommittedSegmentPairRelation::
            IMMEDIATE_SHARED_COMMITTED_GEOMETRY_JUNCTION &&
            extent == NCPathCoreLinkedCommittedSegmentPairExtent::
            TWO_CONSECUTIVE_COMMANDED_SEGMENTS_RELATION_ONLY &&
            frame == NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE &&
            kind == NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR &&
            commandPathPolicy ==
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP &&
            previousSegmentPublicationSequence != 0ULL &&
            currentSegmentPublicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previousSegmentPublicationSequence) &&
            acceptedInputChainGeneration != 0ULL &&
            previousLinkPublicationSequence != 0ULL &&
            currentLinkPublicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previousLinkPublicationSequence) &&
            previousTerminalGeometryPublicationSequence != 0ULL &&
            previousTerminalGeometryPublicationSequence ==
            currentPredecessorGeometryPublicationSequence &&
            previousTerminalMotionSegmentId != 0ULL &&
            previousTerminalMotionSegmentId ==
            currentPredecessorMotionSegmentId &&
            previousTerminalAxisMask != 0U &&
            previousTerminalAxisMask == currentPredecessorAxisMask &&
            currentSegmentAxisMask != 0U &&
            (previousTerminalAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK) == 0U &&
            (currentSegmentAxisMask &
                ~NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_AXIS_MASK) == 0U &&
            (previousCoordinateChangeAxisMask &
                ~previousTerminalAxisMask) == 0U &&
            (currentCoordinateChangeAxisMask &
                ~currentSegmentAxisMask) == 0U &&
            previousAcceptedInputRunLength >= 2U &&
            currentAcceptedInputRunLength ==
            NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(
                previousAcceptedInputRunLength) &&
            sourcePcContiguous;
    }
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_NOINLINE \
    __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_NOINLINE
#endif

class NCPathCoreLinkedCommittedSegmentPairShadow final
{
public:
    NCPathCoreLinkedCommittedSegmentPairShadow() noexcept = default;

    NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_NOINLINE
        void ObserveImmediatePairSameThread(
            const NCPathCoreLinkedCommittedSegmentRecordV1* previous,
            const NCPathCoreLinkedCommittedSegmentRecordV1* current) noexcept
    {
        m_publicationSequence =
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                m_publicationSequence);
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY;

        // The NC thread writes directly into the NCManager-owned fixed slot.
        // No L.2F record or eight-axis geometry array is copied onto stack.
        NCPathCoreLinkedCommittedSegmentPairRecordV1& target =
            m_records[targetIndex];
        target.publicationSequence = m_publicationSequence;
        target.previousSegmentPublicationSequence =
            previous == nullptr ? 0ULL : previous->publicationSequence;
        target.currentSegmentPublicationSequence =
            current == nullptr ? 0ULL : current->publicationSequence;
        target.acceptedInputChainGeneration =
            current == nullptr
            ? 0ULL
            : current->acceptedInputChainGeneration;
        target.previousLinkPublicationSequence =
            previous == nullptr ? 0ULL : previous->linkPublicationSequence;
        target.currentLinkPublicationSequence =
            current == nullptr ? 0ULL : current->linkPublicationSequence;
        target.previousTerminalGeometryPublicationSequence =
            previous == nullptr
            ? 0ULL
            : previous->currentGeometryPublicationSequence;
        target.currentPredecessorGeometryPublicationSequence =
            current == nullptr
            ? 0ULL
            : current->previousGeometryPublicationSequence;
        target.previousTerminalMotionSegmentId =
            previous == nullptr ? 0ULL : previous->currentMotionSegmentId;
        target.currentPredecessorMotionSegmentId =
            current == nullptr ? 0ULL : current->previousMotionSegmentId;
        target.previousTerminalAxisMask =
            previous == nullptr ? 0U : previous->currentAxisMask;
        target.currentPredecessorAxisMask =
            current == nullptr ? 0U : current->previousAxisMask;
        target.currentSegmentAxisMask =
            current == nullptr ? 0U : current->currentAxisMask;
        target.previousCoordinateChangeAxisMask =
            previous == nullptr ? 0U : previous->coordinateChangeAxisMask;
        target.currentCoordinateChangeAxisMask =
            current == nullptr ? 0U : current->coordinateChangeAxisMask;
        target.junctionNumericMismatchAxisMask = 0U;
        target.junctionRepresentationDifferenceAxisMask = 0U;
        target.previousAcceptedInputRunLength =
            previous == nullptr ? 0U : previous->acceptedInputRunLength;
        target.currentAcceptedInputRunLength =
            current == nullptr ? 0U : current->acceptedInputRunLength;
        target.previousSourcePC =
            previous == nullptr ? -1 : previous->sourcePC;
        target.currentSourcePC = current == nullptr ? -1 : current->sourcePC;
        target.schemaVersion =
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_SCHEMA_V1;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentPairDisposition::
            NOT_APPLICABLE_INSUFFICIENT_HISTORY;
        target.numericRelation =
            NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
            NOT_EVALUATED;
        target.pairRelation =
            NCPathCoreLinkedCommittedSegmentPairRelation::NONE;
        target.extent = NCPathCoreLinkedCommittedSegmentPairExtent::NONE;
        target.frame = NCPathCoreCommittedGeometryFrame::NONE;
        target.kind = NCPathCoreCommittedGeometryKind::NONE;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE;
        target.reserved.fill(0U);

        if (current == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_INSUFFICIENT_HISTORY;
        }
        else if (IsNeutralNotFormedObservation(*current))
        {
            target.disposition =
                previous == nullptr
                ? NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_INSUFFICIENT_HISTORY
                : NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_CURRENT_SEGMENT_NOT_FORMED;
        }
        else if (!current->IsFormedLinkedCommandedEndpointSegment())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                INVALID_CURRENT_SEGMENT_RECORD;
        }
        else if (previous == nullptr)
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_INSUFFICIENT_HISTORY;
        }
        else if (IsNeutralNotFormedObservation(*previous))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_APPLICABLE_PREVIOUS_SEGMENT_NOT_FORMED;
        }
        else if (!previous->IsFormedLinkedCommandedEndpointSegment())
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                INVALID_PREVIOUS_SEGMENT_RECORD;
        }
        else if (!IsDirectPairFenceValid(*previous, *current))
        {
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                INVALID_DIRECT_PAIR_FENCE;
        }
        else
        {
            EvaluateJunction(*previous, *current, target);
        }

        // Every neutral/invalid outcome is published as newest, preventing an
        // older proven pair from being mistaken for current chain state.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreLinkedCommittedSegmentPairRecordV1*
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
                NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static bool IsNeutralNotFormedObservation(
        const NCPathCoreLinkedCommittedSegmentRecordV1& record) noexcept
    {
        if (record.publicationSequence == 0ULL ||
            record.schemaVersion !=
            NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_SCHEMA_V1 ||
            record.disposition !=
            NCPathCoreLinkedCommittedSegmentDisposition::
            NOT_FORMED_LINK_NOT_PROVEN ||
            record.predecessorRelation !=
            NCPathCoreLinkedCommittedSegmentRelation::NONE ||
            record.frame != NCPathCoreCommittedGeometryFrame::NONE ||
            record.kind != NCPathCoreCommittedGeometryKind::NONE ||
            record.commandPathPolicy !=
            NCPathCoreCommittedGeometryCommandPathPolicy::NONE ||
            record.extent != NCPathCoreLinkedCommittedSegmentExtent::NONE ||
            record.coordinateChangeAxisMask != 0U)
        {
            return false;
        }

        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (record.startMCS[axis] != 0.0 ||
                record.endMCS[axis] != 0.0)
            {
                return false;
            }
        }

        return true;
    }

    static bool IsDirectPairFenceValid(
        const NCPathCoreLinkedCommittedSegmentRecordV1& previous,
        const NCPathCoreLinkedCommittedSegmentRecordV1& current) noexcept
    {
        const bool sourcePcContiguous =
            previous.sourcePC >= 0 &&
            current.sourcePC >= 0 &&
            static_cast<std::int64_t>(current.sourcePC) ==
            static_cast<std::int64_t>(previous.sourcePC) + 1LL;

        return
            current.publicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previous.publicationSequence) &&
            previous.acceptedInputChainGeneration != 0ULL &&
            current.acceptedInputChainGeneration ==
            previous.acceptedInputChainGeneration &&
            current.linkPublicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previous.linkPublicationSequence) &&
            previous.currentGeometryPublicationSequence != 0ULL &&
            current.previousGeometryPublicationSequence ==
            previous.currentGeometryPublicationSequence &&
            previous.currentMotionSegmentId != 0ULL &&
            current.previousMotionSegmentId ==
            previous.currentMotionSegmentId &&
            previous.currentAxisMask != 0U &&
            current.previousAxisMask == previous.currentAxisMask &&
            previous.acceptedInputRunLength >= 2U &&
            current.acceptedInputRunLength ==
            NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(
                previous.acceptedInputRunLength) &&
            sourcePcContiguous;
    }

    static void EvaluateJunction(
        const NCPathCoreLinkedCommittedSegmentRecordV1& previous,
        const NCPathCoreLinkedCommittedSegmentRecordV1& current,
        NCPathCoreLinkedCommittedSegmentPairRecordV1& target) noexcept
    {
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            const std::uint32_t axisBit = 1U << axis;
            if (previous.endMCS[axis] != current.startMCS[axis])
            {
                target.junctionNumericMismatchAxisMask |= axisBit;
            }
            if (NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                previous.endMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                    current.startMCS[axis]))
            {
                target.junctionRepresentationDifferenceAxisMask |= axisBit;
            }
        }

        if (target.junctionNumericMismatchAxisMask != 0U)
        {
            target.numericRelation =
                NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
                NUMERIC_MISMATCH;
            target.disposition =
                NCPathCoreLinkedCommittedSegmentPairDisposition::
                NOT_PROVEN_JUNCTION_GEOMETRY_MISMATCH;
            return;
        }

        target.numericRelation =
            target.junctionRepresentationDifferenceAxisMask == 0U
            ? NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
            EXACT_NUMERIC_AND_REPRESENTATION
            : NCPathCoreLinkedCommittedSegmentJunctionNumericRelation::
            EXACT_NUMERIC_REPRESENTATION_VARIANT;
        target.disposition =
            NCPathCoreLinkedCommittedSegmentPairDisposition::
            PROVEN_IMMEDIATE_LINKED_SEGMENT_CONTINUATION;
        target.pairRelation =
            NCPathCoreLinkedCommittedSegmentPairRelation::
            IMMEDIATE_SHARED_COMMITTED_GEOMETRY_JUNCTION;
        target.extent =
            NCPathCoreLinkedCommittedSegmentPairExtent::
            TWO_CONSECUTIVE_COMMANDED_SEGMENTS_RELATION_ONLY;
        target.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        target.kind = NCPathCoreCommittedGeometryKind::
            ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR;
        target.commandPathPolicy =
            NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP;
    }

    std::array<
        NCPathCoreLinkedCommittedSegmentPairRecordV1,
        NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_HISTORY_CAPACITY>
        m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_PAIR_NOINLINE

static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentPairDisposition) == 1U,
    "Linked committed segment-pair disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentJunctionNumericRelation) == 1U,
    "Linked committed segment junction relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentPairRelation) == 1U,
    "Linked committed segment pair relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentPairExtent) == 1U,
    "Linked committed segment pair extent must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentPairRecordV1>::value,
    "Linked committed segment-pair record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentPairRecordV1>::value,
    "Linked committed segment-pair record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreLinkedCommittedSegmentPairRecordV1) == 8U,
    "Linked committed segment-pair record alignment changed.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentPairRecordV1) == 136U,
    "Linked committed segment-pair record must remain exactly 136 bytes.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentPairRecordV1,
        previousTerminalAxisMask) == 80U,
    "Linked committed segment-pair terminal-axis offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentPairRecordV1,
        junctionNumericMismatchAxisMask) == 100U,
    "Linked committed segment-pair numeric-mask offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentPairRecordV1,
        schemaVersion) == 124U,
    "Linked committed segment-pair schema offset changed.");
static_assert(
    offsetof(
        NCPathCoreLinkedCommittedSegmentPairRecordV1,
        reserved) == 133U,
    "Linked committed segment-pair reserve offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreLinkedCommittedSegmentPairShadow>::value,
    "Linked committed segment-pair shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreLinkedCommittedSegmentPairShadow>::value,
    "Linked committed segment-pair shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreLinkedCommittedSegmentPairShadow) == 288U,
    "Linked committed segment-pair shadow must remain exactly 288 bytes.");
