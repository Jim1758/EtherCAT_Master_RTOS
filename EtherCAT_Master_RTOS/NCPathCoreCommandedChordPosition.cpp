#include "NCPathCoreCommandedChordPosition.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordPositionCode;
    using CursorCode = NCPathCoreCommandedChordCursorCode;
    using LocateCode = NCPathCoreCommandedChordLocateCode;
    using Store = NCPathCoreCommandedChordStoreV1;
    using Cursor = NCPathCoreCommandedChordCursorV1;
    using Segment = NCPathCoreCommandedChordSegmentV1;

    Code ReadSegment(const Store& store, const Cursor& cursor,
        Segment& workspace) noexcept
    {
        const CursorCode result = ReadCommandedChordCursorCurrent(store, cursor, workspace);
        switch (result)
        {
        case CursorCode::VALUE_READ: return Code::NONE;
        case CursorCode::STORE_CLOSED: return Code::STORE_CLOSED;
        case CursorCode::STORE_FAULTED: return Code::STORE_FAULTED;
        case CursorCode::CURSOR_REJECTED: return Code::CURSOR_REJECTED;
        case CursorCode::RETAINED_VALUE_INVALID: return Code::RETAINED_VALUE_INVALID;
        default:
            workspace.Clear();
            return Code::STORE_REJECTED;
        }
    }

    bool ValidParameter(const double u) noexcept
    {
        return std::isfinite(u) && u >= 0.0 && u <= 1.0;
    }

    Code MapLocation(const LocateCode code) noexcept
    {
        switch (code)
        {
        case LocateCode::LOCATED_START: return Code::LOCATED_START;
        case LocateCode::LOCATED_INTERIOR: return Code::LOCATED_INTERIOR;
        case LocateCode::LOCATED_END: return Code::LOCATED_END;
        case LocateCode::INVALID_AXIS: return Code::INVALID_AXIS;
        case LocateCode::INVALID_COORDINATE: return Code::INVALID_COORDINATE;
        case LocateCode::INVALID_GEOMETRY: return Code::INVALID_GEOMETRY;
        case LocateCode::OUTSIDE_AXIS_RANGE: return Code::OUTSIDE_AXIS_RANGE;
        case LocateCode::NON_UNIQUE_CONSTANT_AXIS: return Code::NON_UNIQUE_CONSTANT_AXIS;
        case LocateCode::NON_UNIQUE_POINT_CHORD: return Code::NON_UNIQUE_POINT_CHORD;
        case LocateCode::PARAMETER_RESOLUTION_LOSS: return Code::PARAMETER_RESOLUTION_LOSS;
        case LocateCode::NONFINITE_ARITHMETIC: return Code::NONFINITE_ARITHMETIC;
        default: return Code::GEOMETRY_REJECTED;
        }
    }
}

NCPathCoreCommandedChordPositionCode BindCommandedChordPosition(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& cursor, const double unitParameter,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionV1& output) noexcept
{
    output.Clear();
    const Code source = ReadSegment(store, cursor, workspace);
    if (source != Code::NONE) return source;
    if (!ValidParameter(unitParameter))
    {
        workspace.Clear();
        return Code::INVALID_PARAMETER;
    }
    output.cursor = cursor;
    output.unitParameter = unitParameter;
    return Code::POSITION_BOUND;
}

NCPathCoreCommandedChordPositionCode LocateCommandedChordPositionAxis(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& cursor, const std::uint32_t axisIndex,
    const double queryCoordinateMCS, NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionV1& output,
    NCPathCoreCommandedChordLocationV1& detail) noexcept
{
    output.Clear();
    detail.Clear();
    const Code source = ReadSegment(store, cursor, workspace);
    if (source != Code::NONE) return source;
    const LocateCode location = LocateCommandedChordSegmentAxis(
        workspace, axisIndex, queryCoordinateMCS, detail);
    const Code code = MapLocation(location);
    if (location == LocateCode::LOCATED_START ||
        location == LocateCode::LOCATED_INTERIOR || location == LocateCode::LOCATED_END)
    {
        output.cursor = cursor;
        output.unitParameter = detail.unitParameter;
        return code;
    }
    // Workspace is an observable output; never leave a success-looking value
    // after a scalar/nonunique/arithmetic rejection of this complete query.
    workspace.Clear();
    detail.Clear();
    return code;
}

NCPathCoreCommandedChordPositionCode EvaluateCommandedChordPosition(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordPositionV1& position,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionSampleV1& output) noexcept
{
    output.Clear();
    const Code source = ReadSegment(store, position.cursor, workspace);
    if (source != Code::NONE) return source;
    if (!ValidParameter(position.unitParameter))
    {
        workspace.Clear();
        return Code::INVALID_PARAMETER;
    }
    // BA already validated the whole immutable segment. Reuse the original AW
    // arithmetic directly, without eight repeated whole-segment validations.
    for (std::size_t axis = 0U;
        axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
    {
        const double coordinate = NCPathCoreCommandedChordDetail::InterpolateFinite(
            workspace.startMCS[axis], workspace.endMCS[axis], position.unitParameter);
        if (!std::isfinite(coordinate))
        {
            output.Clear();
            workspace.Clear();
            return Code::NONFINITE_ARITHMETIC;
        }
        output.coordinateMCS[axis] = coordinate;
    }
    output.position = position;
    output.axisMask = workspace.axisMask;
    output.schemaVersion = 1U;
    output.kind = workspace.kind;
    output.frame = workspace.frame;
    return workspace.kind == NCPathCoreCommandedChordKind::POINT_CHORD
        ? Code::EVALUATED_POINT_CHORD : Code::EVALUATED_LINE_CHORD;
}
