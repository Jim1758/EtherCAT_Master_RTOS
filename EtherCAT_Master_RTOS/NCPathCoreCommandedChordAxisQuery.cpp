#include "NCPathCoreCommandedChordAxisQuery.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordAxisQueryCode;
    using Kind = NCPathCoreCommandedChordAxisCandidateKind;
    using LocateCode = NCPathCoreCommandedChordLocateCode;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using Workspace = NCPathCoreCommandedChordAxisQueryWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisQueryResultV1;

    Code Fail(const Code code, Workspace& workspace, Result& output) noexcept
    {
        output.Clear();
        workspace.Clear();
        return code;
    }

    Code MapSnapshotFailure(const SnapshotCode code) noexcept
    {
        return code == SnapshotCode::NOT_CAPTURED ? Code::NOT_CAPTURED : Code::SNAPSHOT_REJECTED;
    }

    Code Classify(Workspace& workspace, const std::uint32_t axisIndex,
        const double queryCoordinateMCS, NCPathCoreCommandedChordAxisCandidateV1& entry) noexcept
    {
        const LocateCode located = LocateCommandedChordSegmentAxis(
            workspace.segment, axisIndex, queryCoordinateMCS, workspace.location);
        entry.cursor = workspace.piece.cursor;
        entry.clipFrom = workspace.piece.fromParameter;
        entry.clipTo = workspace.piece.toParameter;
        entry.sourceKind = workspace.segment.kind;
        entry.locateCode = located;
        switch (located)
        {
        case LocateCode::OUTSIDE_AXIS_RANGE:
            entry.kind = Kind::NO_AXIS_MATCH;
            return Code::CLASSIFIED;
        case LocateCode::NON_UNIQUE_CONSTANT_AXIS:
        case LocateCode::NON_UNIQUE_POINT_CHORD:
            entry.kind = Kind::PARAMETER_INTERVAL;
            return Code::CLASSIFIED;
        case LocateCode::PARAMETER_RESOLUTION_LOSS:
            entry.kind = Kind::PARAMETER_RESOLUTION_LOSS;
            return Code::CLASSIFIED;
        case LocateCode::LOCATED_START:
        case LocateCode::LOCATED_INTERIOR:
        case LocateCode::LOCATED_END:
        {
            entry.candidateParameter = workspace.location.unitParameter;
            entry.reconstructedCoordinateMCS = workspace.location.reconstructedCoordinateMCS;
            const double low = entry.clipFrom < entry.clipTo ? entry.clipFrom : entry.clipTo;
            const double high = entry.clipFrom < entry.clipTo ? entry.clipTo : entry.clipFrom;
            entry.kind = entry.candidateParameter >= low && entry.candidateParameter <= high
                ? Kind::LOCATED_IN_CLIP : Kind::CLIP_UNRESOLVED;
            return Code::CLASSIFIED;
        }
        case LocateCode::INVALID_AXIS: return Code::INVALID_AXIS;
        case LocateCode::INVALID_COORDINATE: return Code::INVALID_COORDINATE;
        case LocateCode::INVALID_GEOMETRY: return Code::INVALID_GEOMETRY;
        case LocateCode::NONFINITE_ARITHMETIC: return Code::NONFINITE_ARITHMETIC;
        default: return Code::GEOMETRY_REJECTED;
        }
    }
}

NCPathCoreCommandedChordAxisQueryCode QueryCommandedChordSnapshotAxisCandidates(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, const std::uint32_t axisIndex,
    const double queryCoordinateMCS, NCPathCoreCommandedChordAxisQueryWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisQueryResultV1& output) noexcept
{
    output.Clear();
    workspace.Clear();
    // Reuse the first read in the loop; no extra Describe or second first read.
    SnapshotCode read = snapshot.ReadPiece(1U, workspace.segment, workspace.piece, workspace.info);
    if (read != SnapshotCode::PIECE_READ) return Fail(MapSnapshotFailure(read), workspace, output);
    const std::uint32_t count = workspace.info.pieceCount;
    const auto direction = workspace.info.direction;
    if (count == 0U || count > output.entries.size())
        return Fail(Code::SNAPSHOT_REJECTED, workspace, output);
    if (axisIndex >= NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY)
        return Fail(Code::INVALID_AXIS, workspace, output);
    if (!std::isfinite(queryCoordinateMCS))
        return Fail(Code::INVALID_COORDINATE, workspace, output);

    for (std::uint32_t index = 0U; index < count; ++index)
    {
        if (index != 0U)
        {
            read = snapshot.ReadPiece(index + 1U, workspace.segment, workspace.piece, workspace.info);
            if (read != SnapshotCode::PIECE_READ)
                return Fail(MapSnapshotFailure(read), workspace, output);
            if (workspace.info.pieceCount != count || workspace.info.direction != direction)
                return Fail(Code::SNAPSHOT_REJECTED, workspace, output);
        }
        const Code classified = Classify(workspace, axisIndex, queryCoordinateMCS, output.entries[index]);
        if (classified != Code::CLASSIFIED) return Fail(classified, workspace, output);
    }
    output.queryCoordinateMCS = queryCoordinateMCS;
    output.axisIndex = axisIndex;
    output.pieceCount = count;
    output.direction = direction;
    output.schemaVersion = 1U;
    workspace.Clear();
    return Code::CLASSIFIED;
}
