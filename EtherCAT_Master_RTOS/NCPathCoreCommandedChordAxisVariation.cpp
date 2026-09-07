#include "NCPathCoreCommandedChordAxisVariation.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordAxisVariationCode;
    using Kind = NCPathCoreCommandedChordAxisVariationKind;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using Workspace = NCPathCoreCommandedChordAxisVariationWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisVariationResultV1;

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

    Code MeasurePiece(const Workspace& workspace, const std::uint32_t axisIndex,
        const double prefix, NCPathCoreCommandedChordAxisVariationPieceV1& entry) noexcept
    {
        entry.cursor = workspace.piece.cursor;
        entry.clipFrom = workspace.piece.fromParameter;
        entry.clipTo = workspace.piece.toParameter;
        entry.sourceKind = workspace.segment.kind;
        entry.prefixFrom = prefix;
        const double a = workspace.segment.startMCS[axisIndex];
        const double b = workspace.segment.endMCS[axisIndex];
        if (entry.clipFrom == entry.clipTo)
            entry.kind = Kind::ZERO_PARAMETER_SPAN;
        else if (a == b)
            entry.kind = Kind::CONSTANT_AXIS;
        else
        {
            const double span = std::fabs(entry.clipTo - entry.clipFrom);
            const double width = std::fabs(b - a);
            // Overflow of the ORIGINAL full width need not overflow the clip.
            // Scaling is used only here, so tiny finite widths are not halved.
            entry.variation = std::isfinite(width) ? width * span
                : (std::fabs(b * 0.5 - a * 0.5) * span) * 2.0;
            if (!std::isfinite(entry.variation)) return Code::NONFINITE_VARIATION;
            if (!(entry.variation > 0.0)) return Code::VARIATION_RESOLUTION_LOSS;
            entry.kind = Kind::POSITIVE_VARIATION;
        }
        entry.prefixTo = prefix + entry.variation;
        if (!std::isfinite(entry.prefixTo)) return Code::NONFINITE_TOTAL;
        if (entry.variation > 0.0 && !(entry.prefixTo > prefix))
            return Code::PREFIX_RESOLUTION_LOSS;
        return Code::PROFILED;
    }
}

NCPathCoreCommandedChordAxisVariationCode MeasureCommandedChordSnapshotAxisVariation(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, const std::uint32_t axisIndex,
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisVariationResultV1& output) noexcept
{
    output.Clear();
    workspace.Clear();
    SnapshotCode read = snapshot.ReadPiece(1U, workspace.segment, workspace.piece, workspace.info);
    if (read != SnapshotCode::PIECE_READ) return Fail(MapSnapshotFailure(read), workspace, output);
    const std::uint32_t count = workspace.info.pieceCount;
    const auto direction = workspace.info.direction;
    if (count == 0U || count > output.entries.size())
        return Fail(Code::SNAPSHOT_REJECTED, workspace, output);
    if (axisIndex >= NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY)
        return Fail(Code::INVALID_AXIS, workspace, output);

    double prefix = 0.0;
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
        const Code measured = MeasurePiece(workspace, axisIndex, prefix, output.entries[index]);
        if (measured != Code::PROFILED) return Fail(measured, workspace, output);
        prefix = output.entries[index].prefixTo;
    }
    output.totalVariation = prefix;
    output.axisIndex = axisIndex;
    output.pieceCount = count;
    output.direction = direction;
    output.schemaVersion = 1U;
    workspace.Clear();
    return Code::PROFILED;
}
