#include "NCPathCoreCommandedChordAxisStation.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordAxisStationCode;
    using Kind = NCPathCoreCommandedChordAxisStationCandidateKind;
    using VariationCode = NCPathCoreCommandedChordAxisVariationCode;
    using Workspace = NCPathCoreCommandedChordAxisStationWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisStationResultV1;

    Code Fail(const Code code, Workspace& workspace, Result& output) noexcept
    {
        output.Clear();
        workspace.Clear();
        return code;
    }

    Code MapVariationFailure(const VariationCode code) noexcept
    {
        switch (code)
        {
        case VariationCode::NOT_CAPTURED: return Code::NOT_CAPTURED;
        case VariationCode::SNAPSHOT_REJECTED: return Code::SNAPSHOT_REJECTED;
        case VariationCode::INVALID_AXIS: return Code::INVALID_AXIS;
        case VariationCode::NONFINITE_VARIATION: return Code::NONFINITE_VARIATION;
        case VariationCode::VARIATION_RESOLUTION_LOSS: return Code::VARIATION_RESOLUTION_LOSS;
        case VariationCode::NONFINITE_TOTAL: return Code::NONFINITE_TOTAL;
        case VariationCode::PREFIX_RESOLUTION_LOSS: return Code::PREFIX_RESOLUTION_LOSS;
        default: return Code::SNAPSHOT_REJECTED;
        }
    }

    void LocateCandidate(const NCPathCoreCommandedChordAxisVariationPieceV1& source,
        const double station, NCPathCoreCommandedChordAxisStationCandidateV1& entry) noexcept
    {
        entry.cursor = source.cursor;
        entry.clipFrom = source.clipFrom;
        entry.clipTo = source.clipTo;
        entry.variation = source.variation;
        entry.prefixFrom = source.prefixFrom;
        entry.prefixTo = source.prefixTo;
        entry.variationKind = source.kind;
        entry.sourceKind = source.sourceKind;
        if (station < entry.prefixFrom || station > entry.prefixTo)
            entry.kind = Kind::NO_PREFIX_MATCH;
        else if (entry.prefixFrom == entry.prefixTo)
            entry.kind = Kind::PARAMETER_INTERVAL;
        else if (station == entry.prefixFrom)
        {
            entry.kind = Kind::LOCATED_CLIP_FROM;
            entry.candidateParameter = entry.clipFrom;
        }
        else if (station == entry.prefixTo)
        {
            entry.kind = Kind::LOCATED_CLIP_TO;
            entry.candidateParameter = entry.clipTo;
        }
        else
        {
            const double t = (station - entry.prefixFrom) / (entry.prefixTo - entry.prefixFrom);
            const double candidate = entry.clipFrom + (entry.clipTo - entry.clipFrom) * t;
            const bool strictClipInterior = entry.clipFrom < entry.clipTo
                ? candidate > entry.clipFrom && candidate < entry.clipTo
                : candidate > entry.clipTo && candidate < entry.clipFrom;
            if (!std::isfinite(t) || !(t > 0.0 && t < 1.0) ||
                !std::isfinite(candidate) || !strictClipInterior)
                entry.kind = Kind::PARAMETER_RESOLUTION_LOSS;
            else
            {
                entry.kind = Kind::LOCATED_INTERIOR;
                entry.candidateParameter = candidate;
            }
        }
    }
}

NCPathCoreCommandedChordAxisStationCode QueryCommandedChordSnapshotAxisStationCandidates(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, const std::uint32_t axisIndex,
    const double station, NCPathCoreCommandedChordAxisStationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResultV1& output) noexcept
{
    output.Clear();
    workspace.Clear();
    const VariationCode measured = MeasureCommandedChordSnapshotAxisVariation(
        snapshot, axisIndex, workspace.measure, workspace.profile);
    if (measured != VariationCode::PROFILED)
        return Fail(MapVariationFailure(measured), workspace, output);
    if (!std::isfinite(station) || station < 0.0)
        return Fail(Code::INVALID_STATION, workspace, output);
    if (station > workspace.profile.totalVariation)
        return Fail(Code::OUTSIDE_PROFILE, workspace, output);

    for (std::uint32_t index = 0U; index < workspace.profile.pieceCount; ++index)
        LocateCandidate(workspace.profile.entries[index], station, output.entries[index]);

    output.station = station;
    output.totalVariation = workspace.profile.totalVariation;
    output.axisIndex = workspace.profile.axisIndex;
    output.pieceCount = workspace.profile.pieceCount;
    output.direction = workspace.profile.direction;
    output.schemaVersion = 1U;
    workspace.Clear();
    return Code::QUERIED;
}
