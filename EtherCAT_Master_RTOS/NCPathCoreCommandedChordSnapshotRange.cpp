#include "NCPathCoreCommandedChordSnapshot.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordSnapshotCode;
    using Direction = NCPathCoreCommandedChordSubpathDirection;

    bool ValidRangeParameter(const double u) noexcept
    {
        return std::isfinite(u) && u >= 0.0 && u <= 1.0;
    }

    bool WithinClip(const double u, const double from, const double to) noexcept
    {
        return u >= (from < to ? from : to) && u <= (from < to ? to : from);
    }
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::CaptureRange(
    const NCPathCoreCommandedChordSnapshotV1& source,
    const std::uint32_t firstPieceIndex, const double firstSourceU,
    const std::uint32_t lastPieceIndex, const double lastSourceU) noexcept
{
    // An alias rejection must not erase the source. All disjoint failures do
    // clear destination, as does the original Store-backed Capture contract.
    if (this == &source) return Code::ALIASED_SNAPSHOT;
    Clear();
    Code valid = source.ValidateSelected(firstPieceIndex);
    if (valid != Code::NONE) return valid;
    if (lastPieceIndex != firstPieceIndex)
    {
        valid = source.ValidateSelected(lastPieceIndex);
        if (valid != Code::NONE) return valid;
    }
    if (!ValidRangeParameter(firstSourceU) || !ValidRangeParameter(lastSourceU))
        return Code::INVALID_PARAMETER;
    if (!WithinClip(firstSourceU, source.FromParameter(firstPieceIndex), source.ToParameter(firstPieceIndex)) ||
        !WithinClip(lastSourceU, source.FromParameter(lastPieceIndex), source.ToParameter(lastPieceIndex)))
        return Code::OUTSIDE_PIECE_RANGE;

    const bool increasingIndex = firstPieceIndex <= lastPieceIndex;
    const std::uint32_t count = increasingIndex
        ? lastPieceIndex - firstPieceIndex + 1U : firstPieceIndex - lastPieceIndex + 1U;
    source.WriteCursor(firstPieceIndex, m_source.start.cursor);
    source.WriteCursor(lastPieceIndex, m_source.end.cursor);
    m_source.start.unitParameter = firstSourceU;
    m_source.end.unitParameter = lastSourceU;
    for (std::uint32_t index = 0U; index < count; ++index)
    {
        const std::uint32_t selected = increasingIndex ? firstPieceIndex + index : firstPieceIndex - index;
        if (selected != firstPieceIndex && selected != lastPieceIndex)
        {
            valid = source.ValidateSelected(selected);
            if (valid != Code::NONE)
            {
                Clear();
                return valid;
            }
        }
        m_segments[index] = source.m_segments[selected - 1U];
    }
    const std::uint32_t firstOrdinal = m_source.start.cursor.binding.ordinal;
    const std::uint32_t lastOrdinal = m_source.end.cursor.binding.ordinal;
    m_direction = firstOrdinal < lastOrdinal ? Direction::FORWARD
        : (firstOrdinal > lastOrdinal ? Direction::REVERSE
            : (firstSourceU < lastSourceU ? Direction::FORWARD
                : (firstSourceU > lastSourceU ? Direction::REVERSE : Direction::STATIONARY)));
    m_pieceCount = count;
    m_schemaVersion = 1U;
    return Code::CAPTURED;
}
