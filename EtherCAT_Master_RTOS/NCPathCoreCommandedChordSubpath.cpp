#include "NCPathCoreCommandedChordSubpath.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordSubpathCode;
    using Direction = NCPathCoreCommandedChordSubpathDirection;
    using CursorCode = NCPathCoreCommandedChordCursorCode;
    using StoreCode = NCPathCoreCommandedChordStoreCode;
    using Store = NCPathCoreCommandedChordStoreV1;
    using Position = NCPathCoreCommandedChordPositionV1;
    using Segment = NCPathCoreCommandedChordSegmentV1;
    using Info = NCPathCoreCommandedChordSubpathInfoV1;

    Code MapCursorRead(const CursorCode code) noexcept
    {
        switch (code)
        {
        case CursorCode::VALUE_READ: return Code::NONE;
        case CursorCode::STORE_CLOSED: return Code::STORE_CLOSED;
        case CursorCode::STORE_FAULTED: return Code::STORE_FAULTED;
        case CursorCode::CURSOR_REJECTED: return Code::CURSOR_REJECTED;
        case CursorCode::RETAINED_VALUE_INVALID: return Code::RETAINED_VALUE_INVALID;
        default: return Code::STORE_REJECTED;
        }
    }

    bool ValidParameter(const double u) noexcept
    {
        return std::isfinite(u) && u >= 0.0 && u <= 1.0;
    }

    Code ValidateEndpoints(const Store& store, const Position& start,
        const Position& end, Segment& workspace, Info& info) noexcept
    {
        // Do NOT call BB Bind twice: a bad first scalar would mask rejection
        // of the second source binding. Read both sources before either u.
        Code code = MapCursorRead(
            ReadCommandedChordCursorCurrent(store, start.cursor, workspace));
        if (code == Code::NONE)
            code = MapCursorRead(
                ReadCommandedChordCursorCurrent(store, end.cursor, workspace));
        if (code != Code::NONE)
        {
            workspace.Clear();
            return code;
        }
        if (!ValidParameter(start.unitParameter) || !ValidParameter(end.unitParameter))
        {
            workspace.Clear();
            return Code::INVALID_PARAMETER;
        }
        // Both ordinals are now in 1..32 within the SAME live Store lifetime.
        const std::uint32_t first = start.cursor.binding.ordinal;
        const std::uint32_t last = end.cursor.binding.ordinal;
        if (first < last)
        {
            info.pieceCount = last - first + 1U;
            info.direction = Direction::FORWARD;
        }
        else if (first > last)
        {
            info.pieceCount = first - last + 1U;
            info.direction = Direction::REVERSE;
        }
        else
        {
            info.pieceCount = 1U;
            info.direction = start.unitParameter < end.unitParameter
                ? Direction::FORWARD : (start.unitParameter > end.unitParameter
                    ? Direction::REVERSE : Direction::STATIONARY);
        }
        return Code::NONE;
    }

    Code MapSelectedReadFailure(const StoreCode code) noexcept
    {
        // A stable Store cannot close/fault or reject this derived ordinal
        // after both endpoint reads. Unexpected failures remain fail-closed.
        return code == StoreCode::RETAINED_VALUE_INVALID
            ? Code::RETAINED_VALUE_INVALID : Code::STORE_REJECTED;
    }
}

NCPathCoreCommandedChordSubpathCode BindCommandedChordSubpath(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordPositionV1& start,
    const NCPathCoreCommandedChordPositionV1& end,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSubpathV1& output,
    NCPathCoreCommandedChordSubpathInfoV1& info) noexcept
{
    output.Clear();
    info.Clear();
    const Code code = ValidateEndpoints(store, start, end, workspace, info);
    if (code != Code::NONE) return code;
    output.start = start;
    output.end = end;
    return Code::SUBPATH_BOUND;
}

NCPathCoreCommandedChordSubpathCode ReadCommandedChordSubpathPiece(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordSubpathV1& subpath, const std::uint32_t pieceIndex,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSubpathPieceV1& output,
    NCPathCoreCommandedChordSubpathInfoV1& info) noexcept
{
    output.Clear();
    info.Clear();
    const Code code = ValidateEndpoints(store, subpath.start, subpath.end, workspace, info);
    if (code != Code::NONE) return code;
    if (pieceIndex == 0U || pieceIndex > info.pieceCount)
    {
        workspace.Clear();
        info.Clear();
        return Code::PIECE_INDEX_OUTSIDE;
    }
    const bool reverse = info.direction == Direction::REVERSE;
    const std::uint32_t first = subpath.start.cursor.binding.ordinal;
    const std::uint32_t ordinal = reverse ? first - (pieceIndex - 1U)
        : first + (pieceIndex - 1U);
    StoreCode result = store.GetHandleAtOrdinal(ordinal, output.cursor.binding);
    if (result == StoreCode::HANDLE_READ)
        result = store.Read(output.cursor.binding, workspace);
    if (result != StoreCode::VALUE_READ)
    {
        output.Clear();
        workspace.Clear();
        info.Clear();
        return MapSelectedReadFailure(result);
    }
    output.fromParameter = pieceIndex == 1U ? subpath.start.unitParameter
        : (reverse ? 1.0 : 0.0);
    output.toParameter = pieceIndex == info.pieceCount ? subpath.end.unitParameter
        : (reverse ? 0.0 : 1.0);
    return Code::PIECE_READ;
}
