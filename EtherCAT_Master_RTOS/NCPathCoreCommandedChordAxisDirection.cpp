#include "NCPathCoreCommandedChordAxisDirection.h"

namespace
{
    using Code = NCPathCoreCommandedChordAxisDirectionCode;
    using Kind = NCPathCoreCommandedChordAxisDirectionKind;
    using Trend = NCPathCoreCommandedChordAxisDirectionTrend;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using Workspace = NCPathCoreCommandedChordAxisVariationWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisDirectionResultV1;

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

    void ProfilePiece(const Workspace& workspace, const std::uint32_t index,
        Result& output) noexcept
    {
        auto& entry = output.entries[index];
        entry.cursor = workspace.piece.cursor;
        entry.clipFrom = workspace.piece.fromParameter;
        entry.clipTo = workspace.piece.toParameter;
        entry.sourceKind = workspace.segment.kind;
        const bool zeroSpan = entry.clipFrom == entry.clipTo;
        const bool forward = entry.clipFrom < entry.clipTo;
        for (std::uint32_t axis = 0U; axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
        {
            auto& kind = entry.axisDirections[axis];
            auto& summary = output.axes[axis];
            if (zeroSpan)
            {
                kind = Kind::ZERO_PARAMETER_SPAN;
                ++summary.zeroSpanCount;
            }
            else if (workspace.segment.startMCS[axis] == workspace.segment.endMCS[axis])
            {
                kind = Kind::CONSTANT_AXIS;
                ++summary.constantAxisCount;
            }
            else
            {
                if ((workspace.segment.startMCS[axis] < workspace.segment.endMCS[axis]) == forward)
                {
                    kind = Kind::INCREASING;
                    ++summary.increasingCount;
                }
                else
                {
                    kind = Kind::DECREASING;
                    ++summary.decreasingCount;
                }
                if (summary.lastMovingPieceIndex == 0U)
                    summary.firstMovingPieceIndex = index + 1U;
                else if (output.entries[summary.lastMovingPieceIndex - 1U].axisDirections[axis] != kind)
                    ++summary.reversalCount;
                summary.lastMovingPieceIndex = index + 1U;
            }
        }
    }
}

NCPathCoreCommandedChordAxisDirectionCode ProfileCommandedChordSnapshotAxisDirections(
    const NCPathCoreCommandedChordSnapshotV1& snapshot,
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisDirectionResultV1& output) noexcept
{
    output.Clear();
    workspace.Clear();
    SnapshotCode read = snapshot.ReadPiece(1U, workspace.segment, workspace.piece, workspace.info);
    if (read != SnapshotCode::PIECE_READ) return Fail(MapSnapshotFailure(read), workspace, output);
    const std::uint32_t count = workspace.info.pieceCount;
    const auto direction = workspace.info.direction;
    if (count == 0U || count > output.entries.size())
        return Fail(Code::SNAPSHOT_REJECTED, workspace, output);

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
        ProfilePiece(workspace, index, output);
    }
    for (auto& summary : output.axes)
    {
        if (summary.increasingCount == 0U)
            summary.trend = summary.decreasingCount == 0U ? Trend::FLAT : Trend::NONINCREASING;
        else
            summary.trend = summary.decreasingCount == 0U ? Trend::NONDECREASING : Trend::DIRECTION_REVERSAL;
    }
    output.pieceCount = count;
    output.direction = direction;
    output.schemaVersion = 1U;
    workspace.Clear();
    return Code::PROFILED;
}
