#pragma once

#include "NCPathCoreCommittedGeometryShadow.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

// =============================================================
// NC-0.2L.2E / Immediate Committed Geometry Link Relation Shadow
//
// This fixed two-record history classifies only the immediate relationship
// between two L.2D accepted-command committed ordinary no-P G00 endpoint
// pairs. It observes two independent facts: exact numeric equality of the
// complete eight-slot commanded MCS seam and equality of the opaque Motion
// queue-tail fingerprint seal. Both facts are required before a commanded
// tail link is described as proven.
//
// The result remains same-thread diagnostic history. It is not a path
// planner, Runtime permit, Gate, Alarm, PC decision, Motion command, actual
// position, execution-adjacency proof or B2 breadcrumb. A mismatch never
// changes control flow. No tolerance, rotary modulo, distance, direction,
// tangent, velocity or timing meaning is inferred from the MCS values.
// =============================================================

constexpr std::size_t
NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY = 2U;
constexpr std::uint32_t
NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK = 0xFFU;
constexpr std::uint16_t
NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_SCHEMA_V1 = 1U;

namespace NCPathCoreCommittedGeometryLinkDetail
{
    constexpr std::uint64_t SERIAL_SEQUENCE_HALF_RANGE = 1ULL << 63U;

    constexpr std::uint64_t NextNonZero(std::uint64_t current) noexcept
    {
        return current == UINT64_MAX ? 1ULL : current + 1ULL;
    }

    constexpr bool IsAdvancingNonZeroSerialSequence(
        std::uint64_t previous,
        std::uint64_t current) noexcept
    {
        // Unsigned subtraction supplies wrap-aware serial arithmetic. Gaps
        // are permitted because rejected transactions may occur between two
        // accepted G00 inputs. Exactly half the sequence space is ambiguous
        // and therefore rejected fail-closed.
        return
            previous != 0ULL &&
            current != 0ULL &&
            (current - previous) != 0ULL &&
            (current - previous) < SERIAL_SEQUENCE_HALF_RANGE;
    }

    inline std::uint64_t DoubleObjectBits(const double& value) noexcept
    {
        std::uint64_t bits = 0ULL;
        static_assert(
            sizeof(bits) == sizeof(value),
            "Committed geometry link requires 64-bit double storage.");
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }
}

enum class NCPathCoreCommittedGeometryLinkDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_FIRST_INPUT = 1U,
    NOT_APPLICABLE_CHAIN_BOUNDARY = 2U,
    PROVEN_CONTIGUOUS_COMMANDED_TAIL_LINK = 3U,
    INVALID_CURRENT_RECORD = 4U,
    INVALID_PREVIOUS_RECORD = 5U,
    INVALID_DIRECT_PAIR_FENCE = 6U,
    NOT_PROVEN_GEOMETRY_MISMATCH = 7U,
    NOT_PROVEN_TAIL_SEAL_MISMATCH = 8U,
    NOT_PROVEN_GEOMETRY_AND_TAIL_SEAL_MISMATCH = 9U
};

enum class NCPathCoreCommittedGeometryNumericRelation : std::uint8_t
{
    NOT_EVALUATED = 0U,
    EXACT_NUMERIC_AND_REPRESENTATION = 1U,
    EXACT_NUMERIC_REPRESENTATION_VARIANT = 2U,
    NUMERIC_MISMATCH = 3U
};

enum class NCPathCoreCommittedGeometryTailSealRelation : std::uint8_t
{
    NOT_EVALUATED = 0U,
    MATCHED = 1U,
    MISMATCHED = 2U
};

struct NCPathCoreCommittedGeometryLinkRecordV1
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    std::uint64_t previousGeometryPublicationSequence = 0ULL;
    std::uint64_t currentGeometryPublicationSequence = 0ULL;
    std::uint64_t previousMotionSegmentId = 0ULL;
    std::uint64_t currentMotionSegmentId = 0ULL;
    std::uint64_t previousQueueTailTransactionSequence = 0ULL;
    std::uint64_t currentQueueTailTransactionSequence = 0ULL;
    std::uint64_t previousQueueTailCommittedFingerprint = 0ULL;
    std::uint64_t currentQueueTailBeforeFingerprint = 0ULL;

    std::uint32_t previousAxisMask = 0U;
    std::uint32_t currentAxisMask = 0U;
    std::uint32_t numericMismatchAxisMask = 0U;
    std::uint32_t representationDifferenceAxisMask = 0U;

    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommittedGeometryLinkDisposition disposition =
        NCPathCoreCommittedGeometryLinkDisposition::EMPTY;
    NCPathCoreCommittedGeometryNumericRelation numericRelation =
        NCPathCoreCommittedGeometryNumericRelation::NOT_EVALUATED;
    NCPathCoreCommittedGeometryTailSealRelation tailSealRelation =
        NCPathCoreCommittedGeometryTailSealRelation::NOT_EVALUATED;
    NCPathCoreAcceptedInputPairRelation acceptedInputPairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;
    std::array<std::uint8_t, 2U> reserved{};

    bool IsProvenContiguousCommandedTailLink() const noexcept
    {
        const bool numericRelationProvesExactC0 =
            (numericRelation == NCPathCoreCommittedGeometryNumericRelation::
                EXACT_NUMERIC_AND_REPRESENTATION &&
                representationDifferenceAxisMask == 0U) ||
            (numericRelation == NCPathCoreCommittedGeometryNumericRelation::
                EXACT_NUMERIC_REPRESENTATION_VARIANT &&
                representationDifferenceAxisMask != 0U);
        return
            publicationSequence != 0ULL &&
            schemaVersion ==
            NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_SCHEMA_V1 &&
            disposition == NCPathCoreCommittedGeometryLinkDisposition::
            PROVEN_CONTIGUOUS_COMMANDED_TAIL_LINK &&
            numericMismatchAxisMask == 0U &&
            (representationDifferenceAxisMask &
                ~NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK) ==
            0U &&
            numericRelationProvesExactC0 &&
            tailSealRelation ==
            NCPathCoreCommittedGeometryTailSealRelation::MATCHED &&
            acceptedInputPairRelation ==
            NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR &&
            acceptedInputChainGeneration != 0ULL &&
            previousGeometryPublicationSequence != 0ULL &&
            currentGeometryPublicationSequence != 0ULL &&
            currentGeometryPublicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                previousGeometryPublicationSequence) &&
            previousMotionSegmentId != 0ULL &&
            currentMotionSegmentId != 0ULL &&
            currentMotionSegmentId > previousMotionSegmentId &&
            previousQueueTailTransactionSequence != 0ULL &&
            currentQueueTailTransactionSequence != 0ULL &&
            NCPathCoreCommittedGeometryLinkDetail::
            IsAdvancingNonZeroSerialSequence(
                previousQueueTailTransactionSequence,
                currentQueueTailTransactionSequence) &&
            previousQueueTailCommittedFingerprint ==
            currentQueueTailBeforeFingerprint &&
            previousAxisMask != 0U &&
            currentAxisMask != 0U &&
            (previousAxisMask &
                ~NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK) ==
            0U &&
            (currentAxisMask &
                ~NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK) ==
            0U;
    }
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_NOINLINE \
    __attribute__((noinline))
#else
#define NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_NOINLINE
#endif

class NCPathCoreCommittedGeometryLinkShadow final
{
public:
    NCPathCoreCommittedGeometryLinkShadow() noexcept = default;

    NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_NOINLINE
        void ObserveImmediatePairSameThread(
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
            NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY;

        // Assign directly into the NCManager-owned fixed workspace. The two
        // 240-byte L.2D records remain behind const pointers and are never
        // copied onto the NC thread stack.
        NCPathCoreCommittedGeometryLinkRecordV1& target =
            m_records[targetIndex];
        target.publicationSequence = m_publicationSequence;
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
        target.previousQueueTailTransactionSequence =
            previous == nullptr
            ? 0ULL
            : previous->queueTailTransactionSequence;
        target.currentQueueTailTransactionSequence =
            current == nullptr
            ? 0ULL
            : current->queueTailTransactionSequence;
        target.previousQueueTailCommittedFingerprint =
            previous == nullptr
            ? 0ULL
            : previous->queueTailCommittedFingerprint;
        target.currentQueueTailBeforeFingerprint =
            current == nullptr
            ? 0ULL
            : current->queueTailBeforeFingerprint;
        target.previousAxisMask =
            previous == nullptr ? 0U : previous->axisMask;
        target.currentAxisMask =
            current == nullptr ? 0U : current->axisMask;
        target.numericMismatchAxisMask = 0U;
        target.representationDifferenceAxisMask = 0U;
        target.schemaVersion =
            NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_SCHEMA_V1;
        target.disposition =
            NCPathCoreCommittedGeometryLinkDisposition::
            INVALID_CURRENT_RECORD;
        target.numericRelation =
            NCPathCoreCommittedGeometryNumericRelation::NOT_EVALUATED;
        target.tailSealRelation =
            NCPathCoreCommittedGeometryTailSealRelation::NOT_EVALUATED;
        target.acceptedInputPairRelation =
            current == nullptr
            ? NCPathCoreAcceptedInputPairRelation::NONE
            : current->acceptedInputPairRelation;

        if (current == nullptr || !IsBaseRecordFenceValid(*current))
        {
            target.disposition =
                NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_CURRENT_RECORD;
        }
        else if (
            current->acceptedInputPairRelation ==
            NCPathCoreAcceptedInputPairRelation::FIRST_INPUT)
        {
            target.disposition =
                previous == nullptr &&
                current->acceptedInputRunLength == 1U &&
                current->acceptedInputChainGeneration != 0ULL
                ? NCPathCoreCommittedGeometryLinkDisposition::
                NOT_APPLICABLE_FIRST_INPUT
                : NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_DIRECT_PAIR_FENCE;
        }
        else if (
            current->acceptedInputPairRelation ==
            NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY)
        {
            const bool boundaryFenceValid =
                previous != nullptr &&
                previous->publicationSequence != 0ULL &&
                current->publicationSequence ==
                NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                    previous->publicationSequence) &&
                current->acceptedInputRunLength == 1U &&
                current->acceptedInputChainGeneration ==
                NCPathCoreCommittedGeometryLinkDetail::NextNonZero(
                    previous->acceptedInputChainGeneration);
            target.disposition =
                boundaryFenceValid
                ? NCPathCoreCommittedGeometryLinkDisposition::
                NOT_APPLICABLE_CHAIN_BOUNDARY
                : NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_DIRECT_PAIR_FENCE;
        }
        else if (
            current->acceptedInputPairRelation !=
            NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR)
        {
            target.disposition =
                NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_DIRECT_PAIR_FENCE;
        }
        else if (
            previous == nullptr ||
            !IsBaseRecordFenceValid(*previous))
        {
            target.disposition =
                NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_PREVIOUS_RECORD;
        }
        else if (!IsDirectPairFenceValid(*previous, *current))
        {
            target.disposition =
                NCPathCoreCommittedGeometryLinkDisposition::
                INVALID_DIRECT_PAIR_FENCE;
        }
        else
        {
            ObserveEvaluatedLink(*previous, *current, target);
        }

        // Invalid and neutral observations are retained as the newest record,
        // preventing an older proven link from masquerading as current state.
        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount <
            NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreCommittedGeometryLinkRecordV1*
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
                NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY -
                historyOffset) %
            NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static bool IsStrictlyIncreasingNonZero(
        std::uint64_t previous,
        std::uint64_t current) noexcept
    {
        return previous != 0ULL &&
            current != 0ULL &&
            current > previous;
    }

    static bool IsMatchingNonZero(
        std::uint64_t previous,
        std::uint64_t current) noexcept
    {
        return previous != 0ULL &&
            current != 0ULL &&
            current == previous;
    }

    static bool IsPairRunValid(
        NCPathCoreAcceptedInputPairRelation pairRelation,
        std::uint32_t acceptedInputRunLength) noexcept
    {
        return
            ((pairRelation ==
                NCPathCoreAcceptedInputPairRelation::FIRST_INPUT ||
                pairRelation ==
                NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY) &&
                acceptedInputRunLength == 1U) ||
            (pairRelation ==
                NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR &&
                acceptedInputRunLength >= 2U);
    }

    static bool AreBaseCoordinatesValid(
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& record) noexcept
    {
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            if (!std::isfinite(record.startMCS[axis]) ||
                !std::isfinite(record.endMCS[axis]))
            {
                return false;
            }

            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if ((record.axisMask & axisBit) == 0U &&
                record.startMCS[axis] != record.endMCS[axis])
            {
                return false;
            }
        }
        return true;
    }

    static bool IsBaseRecordFenceValid(
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& record) noexcept
    {
        return
            record.IsAcceptedCommandCommitted() &&
            record.publicationSequence != 0ULL &&
            record.acceptedInputChainGeneration != 0ULL &&
            record.preparedSession != 0ULL &&
            record.preparedEntrySequence != 0ULL &&
            record.dispatchId != 0ULL &&
            record.commitSequence != 0ULL &&
            record.motionExecutionEpoch != 0ULL &&
            record.motionSegmentId != 0ULL &&
            record.queueTailTransactionSequence != 0ULL &&
            record.sourcePC >= 0 &&
            record.axisMask != 0U &&
            (record.axisMask &
                ~NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK) ==
            0U &&
            IsPairRunValid(
                record.acceptedInputPairRelation,
                record.acceptedInputRunLength) &&
            AreBaseCoordinatesValid(record);
    }

    static bool IsDirectPairFenceValid(
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& previous,
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& current) noexcept
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
            IsMatchingNonZero(
                previous.acceptedInputChainGeneration,
                current.acceptedInputChainGeneration) &&
            previous.acceptedInputRunLength != 0U &&
            current.acceptedInputRunLength ==
            NCPathCoreDetail::
            SaturatingIncrementAcceptedHandoffRunLength(
                previous.acceptedInputRunLength) &&
            IsMatchingNonZero(
                previous.preparedSession,
                current.preparedSession) &&
            IsMatchingNonZero(
                previous.motionExecutionEpoch,
                current.motionExecutionEpoch) &&
            sourcePcContiguous &&
            IsStrictlyIncreasingNonZero(
                previous.preparedEntrySequence,
                current.preparedEntrySequence) &&
            IsStrictlyIncreasingNonZero(
                previous.dispatchId,
                current.dispatchId) &&
            IsStrictlyIncreasingNonZero(
                previous.commitSequence,
                current.commitSequence) &&
            IsStrictlyIncreasingNonZero(
                previous.motionSegmentId,
                current.motionSegmentId) &&
            NCPathCoreCommittedGeometryLinkDetail::
            IsAdvancingNonZeroSerialSequence(
                previous.queueTailTransactionSequence,
                current.queueTailTransactionSequence);
    }

    static void ObserveEvaluatedLink(
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& previous,
        const NCPathCoreOrdinaryG00CommittedEndpointPairV1& current,
        NCPathCoreCommittedGeometryLinkRecordV1& target) noexcept
    {
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY;
            ++axis)
        {
            const std::uint32_t axisBit =
                static_cast<std::uint32_t>(1U << axis);
            if (previous.endMCS[axis] != current.startMCS[axis])
            {
                target.numericMismatchAxisMask |= axisBit;
            }
            if (NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                previous.endMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(
                    current.startMCS[axis]))
            {
                target.representationDifferenceAxisMask |= axisBit;
            }
        }

        if (target.numericMismatchAxisMask != 0U)
        {
            target.numericRelation =
                NCPathCoreCommittedGeometryNumericRelation::
                NUMERIC_MISMATCH;
        }
        else if (target.representationDifferenceAxisMask != 0U)
        {
            target.numericRelation =
                NCPathCoreCommittedGeometryNumericRelation::
                EXACT_NUMERIC_REPRESENTATION_VARIANT;
        }
        else
        {
            target.numericRelation =
                NCPathCoreCommittedGeometryNumericRelation::
                EXACT_NUMERIC_AND_REPRESENTATION;
        }

        const bool tailSealMatched =
            previous.queueTailCommittedFingerprint ==
            current.queueTailBeforeFingerprint;
        target.tailSealRelation =
            tailSealMatched
            ? NCPathCoreCommittedGeometryTailSealRelation::MATCHED
            : NCPathCoreCommittedGeometryTailSealRelation::MISMATCHED;

        const bool numericGeometryMatched =
            target.numericMismatchAxisMask == 0U;
        target.disposition =
            numericGeometryMatched && tailSealMatched
            ? NCPathCoreCommittedGeometryLinkDisposition::
            PROVEN_CONTIGUOUS_COMMANDED_TAIL_LINK
            : !numericGeometryMatched && tailSealMatched
            ? NCPathCoreCommittedGeometryLinkDisposition::
            NOT_PROVEN_GEOMETRY_MISMATCH
            : numericGeometryMatched
            ? NCPathCoreCommittedGeometryLinkDisposition::
            NOT_PROVEN_TAIL_SEAL_MISMATCH
            : NCPathCoreCommittedGeometryLinkDisposition::
            NOT_PROVEN_GEOMETRY_AND_TAIL_SEAL_MISMATCH;
    }

    std::array<
        NCPathCoreCommittedGeometryLinkRecordV1,
        NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_NOINLINE

static_assert(
    sizeof(NCPathCoreCommittedGeometryLinkDisposition) == 1U,
    "Committed geometry link disposition must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryNumericRelation) == 1U,
    "Committed geometry numeric relation must remain one byte.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryTailSealRelation) == 1U,
    "Committed geometry tail-seal relation must remain one byte.");
static_assert(
    std::is_standard_layout<
    NCPathCoreCommittedGeometryLinkRecordV1>::value,
    "Committed geometry link record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreCommittedGeometryLinkRecordV1>::value,
    "Committed geometry link record must remain trivially copyable.");
static_assert(
    alignof(NCPathCoreCommittedGeometryLinkRecordV1) == 8U,
    "Committed geometry link record alignment changed.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryLinkRecordV1) == 104U,
    "Committed geometry link record must remain exactly 104 bytes.");
static_assert(
    offsetof(
        NCPathCoreCommittedGeometryLinkRecordV1,
        previousAxisMask) == 80U,
    "Committed geometry link previous-mask offset changed.");
static_assert(
    offsetof(
        NCPathCoreCommittedGeometryLinkRecordV1,
        numericMismatchAxisMask) == 88U,
    "Committed geometry link mismatch-mask offset changed.");
static_assert(
    offsetof(
        NCPathCoreCommittedGeometryLinkRecordV1,
        schemaVersion) == 96U,
    "Committed geometry link schema offset changed.");
static_assert(
    std::is_standard_layout<
    NCPathCoreCommittedGeometryLinkShadow>::value,
    "Committed geometry link shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreCommittedGeometryLinkShadow>::value,
    "Committed geometry link shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreCommittedGeometryLinkShadow) == 224U,
    "Committed geometry link shadow must remain exactly 224 bytes.");
