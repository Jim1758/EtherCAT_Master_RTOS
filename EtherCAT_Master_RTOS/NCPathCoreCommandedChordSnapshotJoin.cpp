#include "NCPathCoreCommandedChordSnapshot.h"
#include <cstring>
#include <limits>

namespace
{
    using Code = NCPathCoreCommandedChordSnapshotCode;
    using Direction = NCPathCoreCommandedChordSubpathDirection;
    using Segment = NCPathCoreCommandedChordSegmentV1;

    bool SameOriginalValue(const Segment& a, const Segment& b) noexcept
    {
        // Semantic fields and endpoint representations, never struct padding.
        if (a.localIdentity.geometryPublicationSequence != b.localIdentity.geometryPublicationSequence ||
            a.localIdentity.acceptedInputChainGeneration != b.localIdentity.acceptedInputChainGeneration ||
            a.axisMask != b.axisMask || a.acceptedInputRunLength != b.acceptedInputRunLength ||
            a.schemaVersion != b.schemaVersion || a.sourcePairRelation != b.sourcePairRelation ||
            a.kind != b.kind || a.frame != b.frame || a.extent != b.extent ||
            a.reserved[0U] != b.reserved[0U] || a.reserved[1U] != b.reserved[1U])
            return false;
        for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
            if (std::memcmp(&a.startMCS[axis], &b.startMCS[axis], sizeof(double)) != 0 ||
                std::memcmp(&a.endMCS[axis], &b.endMCS[axis], sizeof(double)) != 0)
                return false;
        return true;
    }

    bool AdjacentOriginalValues(const Segment& lower, const Segment& higher) noexcept
    {
        const std::uint64_t next = lower.localIdentity.geometryPublicationSequence ==
            (std::numeric_limits<std::uint64_t>::max)()
            ? 1ULL : lower.localIdentity.geometryPublicationSequence + 1ULL;
        // Selected-value validation bounds original runs/ordinals to 1..32.
        if (higher.localIdentity.geometryPublicationSequence != next ||
            higher.acceptedInputRunLength != lower.acceptedInputRunLength + 1U ||
            higher.sourcePairRelation != NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR)
            return false;
        for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
            if (lower.endMCS[axis] != higher.startMCS[axis]) return false;
        return true;
    }

    Direction OuterDirection(const NCPathCoreCommandedChordPositionV1& first,
        const NCPathCoreCommandedChordPositionV1& last) noexcept
    {
        const auto a = first.cursor.binding.ordinal;
        const auto b = last.cursor.binding.ordinal;
        return a < b ? Direction::FORWARD : (a > b ? Direction::REVERSE
            : (first.unitParameter < last.unitParameter ? Direction::FORWARD
                : (first.unitParameter > last.unitParameter ? Direction::REVERSE : Direction::STATIONARY)));
    }
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::CaptureJoined(
    const NCPathCoreCommandedChordSnapshotV1& first,
    const NCPathCoreCommandedChordSnapshotV1& second) noexcept
{
    if (this == &first || this == &second || &first == &second) return Code::ALIASED_SNAPSHOT;
    Clear();
    Code valid = first.ValidateSelected(1U);
    if (valid != Code::NONE) return valid;
    if (first.m_pieceCount != 1U)
    {
        valid = first.ValidateSelected(first.m_pieceCount);
        if (valid != Code::NONE) return valid;
    }
    valid = second.ValidateSelected(1U);
    if (valid != Code::NONE) return valid;
    if (second.m_pieceCount != 1U)
    {
        valid = second.ValidateSelected(second.m_pieceCount);
        if (valid != Code::NONE) return valid;
    }
    const auto& scopeA = first.m_source.start.cursor.binding;
    const auto& scopeB = second.m_source.start.cursor.binding;
    if (scopeA.ownerTag != scopeB.ownerTag || scopeA.lifetime != scopeB.lifetime ||
        scopeA.localIdentity.acceptedInputChainGeneration != scopeB.localIdentity.acceptedInputChainGeneration)
        return Code::SOURCE_SCOPE_MISMATCH;

    const auto& tail = first.m_source.end;
    const auto& head = second.m_source.start;
    const std::uint32_t tailOrdinal = tail.cursor.binding.ordinal;
    const std::uint32_t headOrdinal = head.cursor.binding.ordinal;
    const bool shared = tailOrdinal == headOrdinal;
    Direction joinDirection = Direction::STATIONARY;
    if (shared)
    {
        if (tail.unitParameter != head.unitParameter) return Code::RANGE_NOT_CONTIGUOUS;
    }
    else if (tailOrdinal + 1U == headOrdinal)
    {
        if (tail.unitParameter != 1.0 || head.unitParameter != 0.0) return Code::RANGE_NOT_CONTIGUOUS;
        joinDirection = Direction::FORWARD;
    }
    else if (headOrdinal + 1U == tailOrdinal)
    {
        if (tail.unitParameter != 0.0 || head.unitParameter != 1.0) return Code::RANGE_NOT_CONTIGUOUS;
        joinDirection = Direction::REVERSE;
    }
    else return Code::RANGE_NOT_CONTIGUOUS;

    const Direction direction = OuterDirection(first.m_source.start, second.m_source.end);
    if ((first.m_direction != Direction::STATIONARY && first.m_direction != direction) ||
        (second.m_direction != Direction::STATIONARY && second.m_direction != direction) ||
        (!shared && joinDirection != direction))
        return Code::RANGE_DIRECTION_CHANGE;
    const auto& tailValue = first.m_segments[first.m_pieceCount - 1U];
    const auto& headValue = second.m_segments[0U];
    if (shared ? !SameOriginalValue(tailValue, headValue)
        : (joinDirection == Direction::FORWARD ? !AdjacentOriginalValues(tailValue, headValue)
            : !AdjacentOriginalValues(headValue, tailValue)))
        return Code::SEGMENT_VALUE_CONFLICT;

    const std::uint32_t count = first.m_pieceCount + second.m_pieceCount - (shared ? 1U : 0U);
    const std::uint32_t outerFirst = first.m_source.start.cursor.binding.ordinal;
    const std::uint32_t outerLast = second.m_source.end.cursor.binding.ordinal;
    const std::uint32_t extent = outerFirst < outerLast ? outerLast - outerFirst + 1U : outerFirst - outerLast + 1U;
    if (count > Capacity || count != extent) return Code::RANGE_NOT_CONTIGUOUS;

    m_source.start = first.m_source.start;
    m_source.end = second.m_source.end;
    for (std::uint32_t index = 0U; index < first.m_pieceCount; ++index)
    {
        if (index != 0U && index + 1U != first.m_pieceCount)
        {
            valid = first.ValidateSelected(index + 1U);
            if (valid != Code::NONE) { Clear(); return valid; }
        }
        m_segments[index] = first.m_segments[index];
    }
    std::uint32_t outputIndex = first.m_pieceCount;
    for (std::uint32_t index = shared ? 1U : 0U; index < second.m_pieceCount; ++index)
    {
        if (index != 0U && index + 1U != second.m_pieceCount)
        {
            valid = second.ValidateSelected(index + 1U);
            if (valid != Code::NONE) { Clear(); return valid; }
        }
        m_segments[outputIndex] = second.m_segments[index];
        ++outputIndex;
    }
    m_direction = direction;
    m_pieceCount = count;
    m_schemaVersion = 1U;
    return Code::CAPTURED;
}
