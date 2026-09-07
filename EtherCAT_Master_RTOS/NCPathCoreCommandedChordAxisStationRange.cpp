#include "NCPathCoreCommandedChordAxisStationRange.h"

namespace
{
    using Code = NCPathCoreCommandedChordAxisStationRangeCode;
    using Status = NCPathCoreCommandedChordAxisStationRangeStatusV1;
    using ResolveCode = NCPathCoreCommandedChordAxisStationResolveCode;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using Snapshot = NCPathCoreCommandedChordSnapshotV1;
    using Workspace = NCPathCoreCommandedChordAxisStationResolveWorkspaceV1;
    using Result = NCPathCoreCommandedChordAxisStationRangeResultV1;

    Status MakeStatus(const Code code, const ResolveCode resolveCode,
        const SnapshotCode snapshotCode) noexcept
    {
        Status status{};
        status.code = code;
        status.resolveCode = resolveCode;
        status.snapshotCode = snapshotCode;
        return status;
    }

    Status Fail(const Code code, const ResolveCode resolveCode,
        const SnapshotCode snapshotCode, Workspace& workspace,
        Snapshot& destination, Result& output) noexcept
    {
        destination.Clear();
        output.Clear();
        workspace.Clear();
        return MakeStatus(code, resolveCode, snapshotCode);
    }

    bool IsResolved(const ResolveCode code) noexcept
    {
        return code == ResolveCode::RESOLVED_CANDIDATE ||
            code == ResolveCode::RESOLVED_INTERVAL;
    }

    Status Capture(const Snapshot& source, const std::uint32_t axisIndex,
        const double firstStation, const std::uint32_t firstPieceIndex,
        const bool firstInterval, const double firstSourceU,
        const double lastStation, const std::uint32_t lastPieceIndex,
        const bool lastInterval, const double lastSourceU,
        Workspace& workspace, Snapshot& destination, Result& output) noexcept
    {
        if (&source == &destination)
            return MakeStatus(Code::ALIASED_SNAPSHOT, ResolveCode::NONE, SnapshotCode::NONE);

        destination.Clear();
        output.Clear();
        workspace.Clear();

        const ResolveCode firstCode = firstInterval
            ? ResolveCommandedChordSnapshotAxisStationInterval(source, axisIndex,
                firstStation, firstPieceIndex, firstSourceU, workspace, output.first)
            : ResolveCommandedChordSnapshotAxisStationCandidate(source, axisIndex,
                firstStation, firstPieceIndex, workspace, output.first);
        if (!IsResolved(firstCode))
            return Fail(Code::FIRST_ENDPOINT_REJECTED, firstCode, SnapshotCode::NONE,
                workspace, destination, output);

        const ResolveCode lastCode = lastInterval
            ? ResolveCommandedChordSnapshotAxisStationInterval(source, axisIndex,
                lastStation, lastPieceIndex, lastSourceU, workspace, output.last)
            : ResolveCommandedChordSnapshotAxisStationCandidate(source, axisIndex,
                lastStation, lastPieceIndex, workspace, output.last);
        if (!IsResolved(lastCode))
            return Fail(Code::LAST_ENDPOINT_REJECTED, lastCode, SnapshotCode::NONE,
                workspace, destination, output);

        const SnapshotCode captured = destination.CaptureRange(source,
            firstPieceIndex, output.first.sample.position.unitParameter,
            lastPieceIndex, output.last.sample.position.unitParameter);
        if (captured != SnapshotCode::CAPTURED)
            return Fail(Code::RANGE_REJECTED, ResolveCode::NONE, captured,
                workspace, destination, output);

        output.schemaVersion = 1U;
        workspace.Clear();
        return MakeStatus(Code::CAPTURED, ResolveCode::NONE, SnapshotCode::NONE);
    }
}

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationCandidateRange(
    const NCPathCoreCommandedChordSnapshotV1& source, const std::uint32_t axisIndex,
    const double firstStation, const std::uint32_t firstPieceIndex,
    const double lastStation, const std::uint32_t lastPieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept
{
    return Capture(source, axisIndex, firstStation, firstPieceIndex, false, 0.0,
        lastStation, lastPieceIndex, false, 0.0, workspace, destination, output);
}

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationCandidateIntervalRange(
    const NCPathCoreCommandedChordSnapshotV1& source, const std::uint32_t axisIndex,
    const double firstStation, const std::uint32_t firstPieceIndex,
    const double lastStation, const std::uint32_t lastPieceIndex, const double lastSourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept
{
    return Capture(source, axisIndex, firstStation, firstPieceIndex, false, 0.0,
        lastStation, lastPieceIndex, true, lastSourceU, workspace, destination, output);
}

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationIntervalCandidateRange(
    const NCPathCoreCommandedChordSnapshotV1& source, const std::uint32_t axisIndex,
    const double firstStation, const std::uint32_t firstPieceIndex, const double firstSourceU,
    const double lastStation, const std::uint32_t lastPieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept
{
    return Capture(source, axisIndex, firstStation, firstPieceIndex, true, firstSourceU,
        lastStation, lastPieceIndex, false, 0.0, workspace, destination, output);
}

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationIntervalRange(
    const NCPathCoreCommandedChordSnapshotV1& source, const std::uint32_t axisIndex,
    const double firstStation, const std::uint32_t firstPieceIndex, const double firstSourceU,
    const double lastStation, const std::uint32_t lastPieceIndex, const double lastSourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept
{
    return Capture(source, axisIndex, firstStation, firstPieceIndex, true, firstSourceU,
        lastStation, lastPieceIndex, true, lastSourceU, workspace, destination, output);
}
