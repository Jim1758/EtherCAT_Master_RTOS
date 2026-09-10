#include "NCPathCoreCompletedSnapshot.h"

constexpr std::uint32_t NCPathCoreCompletedSnapshotV1::Capacity;

namespace
{
    using Code = NCPathCoreCompletedSnapshotCode;
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    using StoreCode = NCPathCoreCommandedChordStoreCode;

    bool SameHandle(const NCPathCoreCommandedChordStoreHandleV1& a,
        const NCPathCoreCommandedChordStoreHandleV1& b) noexcept
    {
        return a.ownerTag == b.ownerTag && a.lifetime == b.lifetime &&
            a.ordinal == b.ordinal && a.reserved == b.reserved &&
            a.localIdentity.geometryPublicationSequence == b.localIdentity.geometryPublicationSequence &&
            a.localIdentity.acceptedInputChainGeneration == b.localIdentity.acceptedInputChainGeneration;
    }

    bool CompletedRecord(const NCPathCoreExecutionRecordV1& record) noexcept
    {
        if (!record.consumerAccepted || record.lastSequence == 0ULL ||
            record.lastType != MotionFeedbackType::COMPLETED ||
            record.terminalType != MotionFeedbackType::COMPLETED ||
            record.errorCode != 0U || record.rejectReason != MotionRejectReason::NONE)
            return false;
        for (std::size_t index = 0U; index < 7U; ++index)
            if (record.reserved[index] != 0U) return false;
        return true;
    }

    Code SourcesReady(const NCPathCoreLiveRetentionStatusV1& retention,
        const NCPathCoreExecutionLinkStatusV1& execution,
        const NCPathCoreLiveRetentionScopeV1& scope,
        const MotionExecutionEpoch currentEpoch) noexcept
    {
        if (retention.state != NCPathCoreLiveRetentionState::OPEN ||
            retention.readableCount == 0U) return Code::NOT_LIVE;
        if (retention.ownerTag == 0ULL || retention.runToken == 0ULL ||
            retention.lifetime == 0ULL || currentEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
            retention.boundEpoch != currentEpoch || retention.runToken != scope.runToken ||
            scope.programScope != 1U || scope.operationMode != 0U ||
            scope.ownerLease.owner != MotionOwner::AUTO || !scope.ownerLease.IsValid() ||
            retention.storedCount > NCPathCoreCompletedSnapshotV1::Capacity ||
            retention.storedCount != retention.readableCount ||
            retention.storedCount != retention.admittedCount)
            return Code::SOURCE_MISMATCH;
        // BN's ordinary START/RESUMED diagnostic reason is legal while OPEN.
        // The prohibited retention/end reasons below are BO's explicit seal.
        if (execution.state != NCPathCoreExecutionLinkState::TRACKING ||
            execution.fault != NCPathCoreExecutionLinkFault::NONE ||
            execution.retentionReason != 0U || execution.endReason != 0U ||
            execution.lastSequence == 0ULL || execution.failed != 0U || execution.pending != 0U ||
            execution.bound != retention.storedCount || execution.accepted != execution.bound ||
            execution.completed != execution.bound || execution.started > execution.bound)
            return Code::EXECUTION_NOT_READY;
        if (execution.ownerTag != retention.ownerTag || execution.runToken != retention.runToken ||
            execution.lifetime != retention.lifetime) return Code::SOURCE_MISMATCH;
        return Code::NONE;
    }
}

void NCPathCoreCompletedSnapshotInfoV1::Clear() noexcept
{
    ownerTag = 0ULL;
    runToken = 0ULL;
    lifetime = 0ULL;
    lastSequence = 0ULL;
    count = 0U;
    schemaVersion = 0U;
    reserved = 0U;
}

void NCPathCoreCompletedSnapshotWorkspaceV1::Clear() noexcept
{
    subpath.Clear();
    segment.Clear();
    piece.Clear();
    info.Clear();
    retention.Clear();
    execution = NCPathCoreExecutionLinkStatusV1{};
}

void NCPathCoreCompletedSnapshotV1::Clear() noexcept
{
    static_assert(offsetof(NCPathCoreCompletedSnapshotV1, m_geometry) == 0U &&
        offsetof(NCPathCoreCompletedSnapshotV1, m_records) == 5224U &&
        offsetof(NCPathCoreCompletedSnapshotV1, m_info) == 8296U,
        "BP private native offsets changed; no wire or SHM ABI is promised.");
    m_info.Clear();
    m_geometry.Clear();
    for (std::uint32_t index = 0U; index < Capacity; ++index) m_records[index].Clear();
}

NCPathCoreCompletedSnapshotCode NCPathCoreCompletedSnapshotV1::FailCapture(
    const NCPathCoreCompletedSnapshotCode code,
    NCPathCoreCompletedSnapshotWorkspaceV1& workspace) noexcept
{
    Clear();
    workspace.Clear();
    return code;
}

NCPathCoreCompletedSnapshotCode NCPathCoreCompletedSnapshotV1::Capture(
    NCPathCoreLiveRetention& retention,
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch,
    const NCPathCoreExecutionLink& execution,
    NCPathCoreCompletedSnapshotWorkspaceV1& workspace) noexcept
{
    Clear();
    workspace.Clear();
    if (!retention.CheckScope(scope, currentEpoch)) return Code::NOT_LIVE;
    retention.Describe(workspace.retention);
    execution.Describe(workspace.execution);
    Code ready = SourcesReady(workspace.retention, workspace.execution, scope, currentEpoch);
    if (ready != Code::NONE) return FailCapture(ready, workspace);

    const std::uint32_t count = workspace.retention.storedCount;
    const std::uint32_t replayCount = workspace.retention.replayCount;
    const std::uint32_t startedCount = workspace.execution.started;
    m_info.ownerTag = workspace.retention.ownerTag;
    m_info.runToken = workspace.retention.runToken;
    m_info.lifetime = workspace.retention.lifetime;
    m_info.lastSequence = workspace.execution.lastSequence;
    if (retention.GetHandleAtOrdinal(scope, currentEpoch, 1U,
        workspace.subpath.start.cursor.binding) != StoreCode::HANDLE_READ ||
        retention.GetHandleAtOrdinal(scope, currentEpoch, count,
            workspace.subpath.end.cursor.binding) != StoreCode::HANDLE_READ)
        return FailCapture(Code::SOURCE_REJECTED, workspace);
    workspace.subpath.start.unitParameter = 0.0;
    workspace.subpath.end.unitParameter = 1.0;
    if (retention.CaptureSnapshot(scope, currentEpoch, workspace.subpath,
        workspace.segment, m_geometry) != SnapshotCode::CAPTURED)
        return FailCapture(Code::SOURCE_REJECTED, workspace);

    std::uint32_t capturedStarted = 0U;
    for (std::uint32_t index = 0U; index < count; ++index)
    {
        if (m_geometry.ReadPiece(index + 1U, workspace.segment, workspace.piece,
            workspace.info) != SnapshotCode::PIECE_READ ||
            !execution.Read(workspace.piece.cursor.binding, m_records[index]))
            return FailCapture(Code::SOURCE_REJECTED, workspace);
        const NCPathCoreExecutionRecordV1& row = m_records[index];
        if (!SameHandle(row.handle, workspace.piece.cursor.binding) ||
            row.handle.ownerTag != m_info.ownerTag || row.handle.lifetime != m_info.lifetime ||
            row.handle.ordinal != index + 1U || row.handle.reserved != 0U ||
            !row.identity.IsAssigned() || row.identity.epoch != currentEpoch ||
            row.identity.source != MotionCommandSource::NC_MEMORY || row.identity.sourceBlockId < 0 ||
            !row.ownerLease.Matches(scope.ownerLease) ||
            workspace.info.pieceCount != count ||
            workspace.info.direction != NCPathCoreCommandedChordSubpathDirection::FORWARD ||
            workspace.piece.fromParameter != 0.0 || workspace.piece.toParameter != 1.0)
            return FailCapture(Code::SOURCE_MISMATCH, workspace);
        if (!CompletedRecord(row)) return FailCapture(Code::EXECUTION_NOT_READY, workspace);
        if (row.started) ++capturedStarted;
    }

    // Same-thread revalidation is a consistency guard, not an atomic protocol.
    // A global feedback sequence can wrap, so compare equality, never <=.
    if (!retention.CheckScope(scope, currentEpoch)) return FailCapture(Code::NOT_LIVE, workspace);
    retention.Describe(workspace.retention);
    execution.Describe(workspace.execution);
    ready = SourcesReady(workspace.retention, workspace.execution, scope, currentEpoch);
    if (ready != Code::NONE) return FailCapture(ready, workspace);
    if (workspace.retention.ownerTag != m_info.ownerTag ||
        workspace.retention.runToken != m_info.runToken ||
        workspace.retention.lifetime != m_info.lifetime ||
        workspace.retention.storedCount != count || workspace.retention.replayCount != replayCount ||
        workspace.execution.lastSequence != m_info.lastSequence ||
        workspace.execution.started != startedCount || capturedStarted != startedCount)
        return FailCapture(Code::SOURCE_MISMATCH, workspace);
    m_info.count = count;
    m_info.schemaVersion = 1U;
    return Code::CAPTURED;
}

NCPathCoreCompletedSnapshotCode NCPathCoreCompletedSnapshotV1::ValidateSelected(
    const std::uint32_t pieceIndex) const noexcept
{
    if (m_info.schemaVersion == 0U && m_info.count == 0U) return Code::NOT_CAPTURED;
    if (m_info.schemaVersion != 1U || m_info.count == 0U || m_info.count > Capacity ||
        m_info.ownerTag == 0ULL || m_info.runToken == 0ULL || m_info.lifetime == 0ULL ||
        m_info.lastSequence == 0ULL || m_info.reserved != 0U) return Code::QUERY_REJECTED;
    if (pieceIndex == 0U || pieceIndex > m_info.count) return Code::PIECE_OUTSIDE;
    const NCPathCoreExecutionRecordV1& row = m_records[pieceIndex - 1U];
    if (!CompletedRecord(row) || row.handle.ownerTag != m_info.ownerTag ||
        row.handle.lifetime != m_info.lifetime || row.handle.ordinal != pieceIndex ||
        row.handle.reserved != 0U || row.handle.localIdentity.geometryPublicationSequence == 0ULL ||
        row.handle.localIdentity.acceptedInputChainGeneration == 0ULL ||
        !row.identity.IsAssigned() || row.identity.source != MotionCommandSource::NC_MEMORY ||
        row.identity.sourceBlockId < 0 || !row.ownerLease.IsValid() ||
        row.ownerLease.owner != MotionOwner::AUTO ||
        row.identity.epoch != m_records[0U].identity.epoch ||
        !row.ownerLease.Matches(m_records[0U].ownerLease)) return Code::QUERY_REJECTED;
    return Code::NONE;
}

NCPathCoreCompletedSnapshotCode NCPathCoreCompletedSnapshotV1::ReadPiece(
    const std::uint32_t pieceIndex,
    NCPathCoreCommandedChordSegmentV1& segment,
    NCPathCoreExecutionRecordV1& execution,
    NCPathCoreCommandedChordSubpathPieceV1& piece,
    NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept
{
    segment.Clear();
    execution.Clear();
    piece.Clear();
    info.Clear();
    const Code valid = ValidateSelected(pieceIndex);
    if (valid != Code::NONE) return valid;
    if (m_geometry.ReadPiece(pieceIndex, segment, piece, info) != SnapshotCode::PIECE_READ ||
        info.pieceCount != m_info.count ||
        info.direction != NCPathCoreCommandedChordSubpathDirection::FORWARD ||
        piece.fromParameter != 0.0 || piece.toParameter != 1.0 ||
        !SameHandle(piece.cursor.binding, m_records[pieceIndex - 1U].handle))
    {
        segment.Clear();
        piece.Clear();
        info.Clear();
        return Code::QUERY_REJECTED;
    }
    execution = m_records[pieceIndex - 1U];
    return Code::PIECE_READ;
}

NCPathCoreCompletedSnapshotCode NCPathCoreCompletedSnapshotV1::EvaluatePiece(
    const std::uint32_t pieceIndex, const double sourceU,
    NCPathCoreCommandedChordPositionSampleV1& sample,
    NCPathCoreExecutionRecordV1& execution) const noexcept
{
    sample.Clear();
    execution.Clear();
    const Code valid = ValidateSelected(pieceIndex);
    if (valid != Code::NONE) return valid;
    const SnapshotCode evaluated = m_geometry.EvaluatePiece(pieceIndex, sourceU, sample);
    if ((evaluated != SnapshotCode::EVALUATED_LINE_CHORD &&
        evaluated != SnapshotCode::EVALUATED_POINT_CHORD) ||
        !SameHandle(sample.position.cursor.binding, m_records[pieceIndex - 1U].handle))
    {
        sample.Clear();
        return Code::QUERY_REJECTED;
    }
    execution = m_records[pieceIndex - 1U];
    return Code::EVALUATED;
}

void NCPathCoreCompletedSnapshotV1::Describe(
    NCPathCoreCompletedSnapshotInfoV1& output) const noexcept
{
    output = m_info;
}
