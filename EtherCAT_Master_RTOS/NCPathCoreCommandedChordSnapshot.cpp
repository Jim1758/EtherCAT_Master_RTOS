#include "NCPathCoreCommandedChordSnapshot.h"
#include <cmath>

// C++14 needs a definition when a caller odr-uses the public constant.
constexpr std::uint32_t NCPathCoreCommandedChordSnapshotV1::Capacity;

namespace
{
    using Code = NCPathCoreCommandedChordSnapshotCode;
    using SubpathCode = NCPathCoreCommandedChordSubpathCode;
    using StoreCode = NCPathCoreCommandedChordStoreCode;
    using Direction = NCPathCoreCommandedChordSubpathDirection;

    bool ValidParameter(const double u) noexcept
    {
        return std::isfinite(u) && u >= 0.0 && u <= 1.0;
    }

    Code MapBindFailure(const SubpathCode code) noexcept
    {
        switch (code)
        {
        case SubpathCode::STORE_CLOSED: return Code::STORE_CLOSED;
        case SubpathCode::STORE_FAULTED: return Code::STORE_FAULTED;
        case SubpathCode::CURSOR_REJECTED: return Code::CURSOR_REJECTED;
        case SubpathCode::RETAINED_VALUE_INVALID: return Code::RETAINED_VALUE_INVALID;
        case SubpathCode::INVALID_PARAMETER: return Code::INVALID_PARAMETER;
        default: return Code::STORE_REJECTED;
        }
    }

    Code MapReadFailure(const StoreCode code) noexcept
    {
        // After BC validation, the Store cannot change during this call.
        return code == StoreCode::RETAINED_VALUE_INVALID
            ? Code::RETAINED_VALUE_INVALID : Code::STORE_REJECTED;
    }
}

void NCPathCoreCommandedChordSnapshotV1::Clear() noexcept
{
    m_pieceCount = 0U;
    m_schemaVersion = 0U;
    m_direction = Direction::NONE;
    m_reserved = 0U;
    m_source.Clear();
    for (std::size_t index = 0U; index < m_segments.size(); ++index)
        m_segments[index].Clear();
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::Capture(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordSubpathV1& subpath,
    NCPathCoreCommandedChordSegmentV1& workspace) noexcept
{
    Clear();
    NCPathCoreCommandedChordSubpathInfoV1 info{};
    const SubpathCode bound = BindCommandedChordSubpath(
        store, subpath.start, subpath.end, workspace, m_source, info);
    if (bound != SubpathCode::SUBPATH_BOUND)
    {
        Clear();
        workspace.Clear();
        return MapBindFailure(bound);
    }
    // Store immutability spans the entire capture. Do not call BC ReadPiece
    // N times and repeatedly validate both endpoints for every retained copy.
    NCPathCoreCommandedChordStoreHandleV1 handle{};
    for (std::uint32_t index = 0U; index < info.pieceCount; ++index)
    {
        const std::uint32_t ordinal = info.direction == Direction::REVERSE
            ? m_source.start.cursor.binding.ordinal - index
            : m_source.start.cursor.binding.ordinal + index;
        StoreCode read = store.GetHandleAtOrdinal(ordinal, handle);
        if (read == StoreCode::HANDLE_READ)
        {
            read = store.Read(handle, m_segments[index]);
            if (read == StoreCode::VALUE_READ) continue;
        }
        Clear();
        workspace.Clear();
        return MapReadFailure(read);
    }
    m_direction = info.direction;
    m_pieceCount = info.pieceCount;
    m_schemaVersion = 1U; // Same-thread completion marker, not synchronization.
    return Code::CAPTURED;
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::ValidateSelected(
    const std::uint32_t pieceIndex) const noexcept
{
    if (m_schemaVersion == 0U && m_pieceCount == 0U) return Code::NOT_CAPTURED;
    if (m_schemaVersion != 1U || m_pieceCount == 0U || m_pieceCount > Capacity || m_reserved != 0U)
        return Code::INVALID_SNAPSHOT;
    const auto& first = m_source.start.cursor.binding;
    const auto& last = m_source.end.cursor.binding;
    if (first.ownerTag == 0ULL || first.lifetime == 0ULL ||
        first.ownerTag != last.ownerTag || first.lifetime != last.lifetime ||
        first.reserved != 0U || last.reserved != 0U ||
        first.ordinal == 0U || first.ordinal > Capacity ||
        last.ordinal == 0U || last.ordinal > Capacity ||
        first.localIdentity.geometryPublicationSequence == 0ULL ||
        last.localIdentity.geometryPublicationSequence == 0ULL ||
        first.localIdentity.acceptedInputChainGeneration == 0ULL ||
        first.localIdentity.acceptedInputChainGeneration != last.localIdentity.acceptedInputChainGeneration ||
        !ValidParameter(m_source.start.unitParameter) || !ValidParameter(m_source.end.unitParameter))
        return Code::INVALID_SNAPSHOT;
    const std::uint32_t count = first.ordinal < last.ordinal
        ? last.ordinal - first.ordinal + 1U : first.ordinal - last.ordinal + 1U;
    const Direction direction = first.ordinal < last.ordinal ? Direction::FORWARD
        : (first.ordinal > last.ordinal ? Direction::REVERSE
            : (m_source.start.unitParameter < m_source.end.unitParameter ? Direction::FORWARD
                : (m_source.start.unitParameter > m_source.end.unitParameter
                    ? Direction::REVERSE : Direction::STATIONARY)));
    if (count != m_pieceCount || direction != m_direction) return Code::INVALID_SNAPSHOT;
    if (pieceIndex == 0U || pieceIndex > m_pieceCount) return Code::PIECE_INDEX_OUTSIDE;
    const auto& segment = m_segments[pieceIndex - 1U];
    if (!IsValidCommandedChordSegmentValue(segment) ||
        segment.localIdentity.acceptedInputChainGeneration != first.localIdentity.acceptedInputChainGeneration ||
        segment.acceptedInputRunLength != SourceOrdinal(pieceIndex) ||
        (pieceIndex == 1U && segment.localIdentity.geometryPublicationSequence !=
            first.localIdentity.geometryPublicationSequence) ||
        (pieceIndex == m_pieceCount && segment.localIdentity.geometryPublicationSequence !=
            last.localIdentity.geometryPublicationSequence))
        return Code::INVALID_SNAPSHOT;
    return Code::NONE;
}

std::uint32_t NCPathCoreCommandedChordSnapshotV1::SourceOrdinal(
    const std::uint32_t pieceIndex) const noexcept
{
    return m_direction == Direction::REVERSE
        ? m_source.start.cursor.binding.ordinal - (pieceIndex - 1U)
        : m_source.start.cursor.binding.ordinal + (pieceIndex - 1U);
}

double NCPathCoreCommandedChordSnapshotV1::FromParameter(
    const std::uint32_t pieceIndex) const noexcept
{
    return pieceIndex == 1U ? m_source.start.unitParameter
        : (m_direction == Direction::REVERSE ? 1.0 : 0.0);
}

double NCPathCoreCommandedChordSnapshotV1::ToParameter(
    const std::uint32_t pieceIndex) const noexcept
{
    return pieceIndex == m_pieceCount ? m_source.end.unitParameter
        : (m_direction == Direction::REVERSE ? 0.0 : 1.0);
}

void NCPathCoreCommandedChordSnapshotV1::WriteCursor(const std::uint32_t pieceIndex,
    NCPathCoreCommandedChordCursorV1& output) const noexcept
{
    output.binding.ownerTag = m_source.start.cursor.binding.ownerTag;
    output.binding.lifetime = m_source.start.cursor.binding.lifetime;
    output.binding.ordinal = SourceOrdinal(pieceIndex);
    output.binding.reserved = 0U;
    output.binding.localIdentity = m_segments[pieceIndex - 1U].localIdentity;
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::ReadPiece(
    const std::uint32_t pieceIndex, NCPathCoreCommandedChordSegmentV1& segment,
    NCPathCoreCommandedChordSubpathPieceV1& piece,
    NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept
{
    segment.Clear();
    piece.Clear();
    info.Clear();
    const Code valid = ValidateSelected(pieceIndex);
    if (valid != Code::NONE) return valid;
    segment = m_segments[pieceIndex - 1U];
    WriteCursor(pieceIndex, piece.cursor);
    piece.fromParameter = FromParameter(pieceIndex);
    piece.toParameter = ToParameter(pieceIndex);
    info.pieceCount = m_pieceCount;
    info.direction = m_direction;
    return Code::PIECE_READ;
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreCommandedChordSnapshotV1::EvaluatePiece(
    const std::uint32_t pieceIndex, const double sourceU,
    NCPathCoreCommandedChordPositionSampleV1& output) const noexcept
{
    output.Clear();
    const Code valid = ValidateSelected(pieceIndex);
    if (valid != Code::NONE) return valid;
    if (!ValidParameter(sourceU)) return Code::INVALID_PARAMETER;
    const double from = FromParameter(pieceIndex);
    const double to = ToParameter(pieceIndex);
    if (sourceU < (from < to ? from : to) || sourceU >(from < to ? to : from))
        return Code::OUTSIDE_PIECE_RANGE;
    const auto& segment = m_segments[pieceIndex - 1U];
    for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
    {
        const double coordinate = NCPathCoreCommandedChordDetail::InterpolateFinite(
            segment.startMCS[axis], segment.endMCS[axis], sourceU);
        if (!std::isfinite(coordinate))
        {
            output.Clear();
            return Code::NONFINITE_ARITHMETIC;
        }
        output.coordinateMCS[axis] = coordinate;
    }
    WriteCursor(pieceIndex, output.position.cursor);
    output.position.unitParameter = sourceU;
    output.axisMask = segment.axisMask;
    output.schemaVersion = 1U;
    output.kind = segment.kind;
    output.frame = segment.frame;
    return segment.kind == NCPathCoreCommandedChordKind::POINT_CHORD
        ? Code::EVALUATED_POINT_CHORD : Code::EVALUATED_LINE_CHORD;
}
