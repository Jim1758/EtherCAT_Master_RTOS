#include "NCPathCoreCommandedChordAxisStationResolve.h"
#include <cmath>

namespace
{
    using Code = NCPathCoreCommandedChordAxisStationResolveCode;
    using Kind = NCPathCoreCommandedChordAxisStationCandidateKind;
    using ResolutionKind = NCPathCoreCommandedChordAxisStationResolutionKind;
    using StationCode = NCPathCoreCommandedChordAxisStationCode;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using Workspace = NCPathCoreCommandedChordAxisStationResolveWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisStationResolveResultV1;

    Code Fail(const Code code, Workspace& workspace, Result& output) noexcept
    {
        output.Clear();
        workspace.Clear();
        return code;
    }

    Code MapStationFailure(const StationCode code) noexcept
    {
        switch (code)
        {
        case StationCode::NOT_CAPTURED: return Code::NOT_CAPTURED;
        case StationCode::SNAPSHOT_REJECTED: return Code::SNAPSHOT_REJECTED;
        case StationCode::INVALID_AXIS: return Code::INVALID_AXIS;
        case StationCode::NONFINITE_VARIATION: return Code::NONFINITE_VARIATION;
        case StationCode::VARIATION_RESOLUTION_LOSS: return Code::VARIATION_RESOLUTION_LOSS;
        case StationCode::NONFINITE_TOTAL: return Code::NONFINITE_TOTAL;
        case StationCode::PREFIX_RESOLUTION_LOSS: return Code::PREFIX_RESOLUTION_LOSS;
        case StationCode::INVALID_STATION: return Code::INVALID_STATION;
        case StationCode::OUTSIDE_PROFILE: return Code::OUTSIDE_PROFILE;
        default: return Code::GEOMETRY_REJECTED;
        }
    }

    Code MapSnapshotFailure(const SnapshotCode code) noexcept
    {
        switch (code)
        {
        case SnapshotCode::NOT_CAPTURED: return Code::NOT_CAPTURED;
        case SnapshotCode::INVALID_SNAPSHOT: return Code::SNAPSHOT_REJECTED;
        case SnapshotCode::PIECE_INDEX_OUTSIDE: return Code::PIECE_INDEX_OUTSIDE;
        case SnapshotCode::INVALID_PARAMETER: return Code::INVALID_PARAMETER;
        case SnapshotCode::OUTSIDE_PIECE_RANGE: return Code::OUTSIDE_PIECE_RANGE;
        case SnapshotCode::NONFINITE_ARITHMETIC: return Code::NONFINITE_ARITHMETIC;
        default: return Code::GEOMETRY_REJECTED;
        }
    }

    Code Resolve(const NCPathCoreCommandedChordSnapshotV1& snapshot,
        const std::uint32_t axisIndex, const double station,
        const std::uint32_t pieceIndex, const bool explicitInterval,
        const double sourceU, Workspace& workspace, Result& output) noexcept
    {
        output.Clear();
        workspace.Clear();
        const StationCode queried = QueryCommandedChordSnapshotAxisStationCandidates(
            snapshot, axisIndex, station, workspace.query, workspace.candidates);
        if (queried != StationCode::QUERIED)
            return Fail(MapStationFailure(queried), workspace, output);
        if (pieceIndex == 0U || pieceIndex > workspace.candidates.pieceCount)
            return Fail(Code::PIECE_INDEX_OUTSIDE, workspace, output);

        const NCPathCoreCommandedChordAxisStationCandidateV1& selected =
            workspace.candidates.entries[pieceIndex - 1U];
        double selectedU = 0.0;
        switch (selected.kind)
        {
        case Kind::NO_PREFIX_MATCH:
            return Fail(Code::NO_PREFIX_MATCH, workspace, output);
        case Kind::PARAMETER_RESOLUTION_LOSS:
            return Fail(Code::PARAMETER_RESOLUTION_LOSS, workspace, output);
        case Kind::PARAMETER_INTERVAL:
            if (!explicitInterval)
                return Fail(Code::INTERVAL_PARAMETER_REQUIRED, workspace, output);
            if (!std::isfinite(sourceU) || sourceU < 0.0 || sourceU > 1.0)
                return Fail(Code::INVALID_PARAMETER, workspace, output);
            if (selected.clipFrom <= selected.clipTo
                ? sourceU < selected.clipFrom || sourceU > selected.clipTo
                : sourceU < selected.clipTo || sourceU > selected.clipFrom)
                return Fail(Code::OUTSIDE_PIECE_RANGE, workspace, output);
            selectedU = sourceU;
            break;
        case Kind::LOCATED_CLIP_FROM:
        case Kind::LOCATED_INTERIOR:
        case Kind::LOCATED_CLIP_TO:
            if (explicitInterval)
                return Fail(Code::NOT_PARAMETER_INTERVAL, workspace, output);
            selectedU = selected.candidateParameter;
            break;
        default:
            return Fail(Code::GEOMETRY_REJECTED, workspace, output);
        }

        const SnapshotCode evaluated = snapshot.EvaluatePiece(pieceIndex, selectedU, output.sample);
        if (evaluated != SnapshotCode::EVALUATED_LINE_CHORD &&
            evaluated != SnapshotCode::EVALUATED_POINT_CHORD)
            return Fail(MapSnapshotFailure(evaluated), workspace, output);

        output.candidate = selected;
        output.station = workspace.candidates.station;
        output.totalVariation = workspace.candidates.totalVariation;
        output.axisIndex = workspace.candidates.axisIndex;
        output.pieceIndex = pieceIndex;
        output.direction = workspace.candidates.direction;
        output.resolutionKind = explicitInterval
            ? ResolutionKind::EXPLICIT_INTERVAL_PARAMETER : ResolutionKind::LOCATED_CANDIDATE;
        output.schemaVersion = 1U;
        workspace.Clear();
        return explicitInterval ? Code::RESOLVED_INTERVAL : Code::RESOLVED_CANDIDATE;
    }
}

NCPathCoreCommandedChordAxisStationResolveCode ResolveCommandedChordSnapshotAxisStationCandidate(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, const std::uint32_t axisIndex,
    const double station, const std::uint32_t pieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResolveResultV1& output) noexcept
{
    return Resolve(snapshot, axisIndex, station, pieceIndex, false, 0.0, workspace, output);
}

NCPathCoreCommandedChordAxisStationResolveCode ResolveCommandedChordSnapshotAxisStationInterval(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, const std::uint32_t axisIndex,
    const double station, const std::uint32_t pieceIndex, const double sourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResolveResultV1& output) noexcept
{
    return Resolve(snapshot, axisIndex, station, pieceIndex, true, sourceU, workspace, output);
}
