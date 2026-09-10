#pragma once

#include "NCPathCoreLiveRetention.h"
#include "NCPathCoreExecutionLink.h"

// NC-0.2L.2BP: detached copy of ALL currently retained commanded segments,
// qualified by the existing BO ACCEPTED/COMPLETED records at capture time.
// This covers one BN lifetime, not the full NC program or measured trajectory.
// G00 endpoint chords remain commanded mathematical geometry. The saved Motion
// identity/lease/feedback watermark are historical labels, never live authority,
// queue admission, current owner proof, replay permission or a control command.
//
// Capture requires same-thread BN OPEN and BO TRACKING, matching nonzero scope,
// all retained rows completed, no BO fault/seal/end and no pending/failed row.
// Point completion without STARTED is legal. BN exact replay is legal. The
// complete [ordinal 1,u=0] -> [last ordinal,u=1] range is captured; no missing
// prefix, epoch bridge, synthetic head, partial-completion subset or overwrite.
// All copies validate before count/schema commit. Every capture failure clears
// this object and the entire workspace. BN's existing CheckScope may fence a
// mismatched live scope; this component does not otherwise fault either source.
//
// Pure ReadPiece/EvaluatePiece survive source append, fence, fault, new run or
// destruction. They consult only this immutable detached value. Piece indices
// are one-based and u remains the original segment parameter, including POINT.
// Invalid indices/u clear every query output without changing the snapshot.
// BO progress is never interpreted as u. Interpolation inherits BD/AW native
// units, all-eight-slot endpoint and precise-FP rules without additional math.
//
// Allocate this object, Workspace and all Segment/Record/Sample outputs on the
// heap before use, outside Observe/PDO. No automatic bulk copy/move is allowed.
// ALL complete input/output/workspace/source/this objects must be disjoint and
// alive. In-place aliases, input members used as outputs and overlapping raw
// storage are outside the contract; no reinterpretation or alias probing is
// supplied. One owning NC thread, no reentry or mutation during the WHOLE call.
// Final metadata writes provide no cross-thread synchronization/publication.
// Trusted source provenance and non-reused owner/lifetime remain obligations
// of the caller; valid labels are not authentication or memory-damage proof.
//
// Fixed capacity 32: one BN/BD capture, <=32 BD reads and <=32 BO reads, then
// final scope/count/feedback-watermark recheck. Clear visits all 32 records
// and BD slots, including on late failure. No allocation, local bulk value,
// I/O, observer, thread, synchronization, Motion/Path queue or new motion math.
// Work counts are bounded, not a target WCET or full call-chain stack proof.

enum class NCPathCoreCompletedSnapshotCode : std::uint8_t
{
    NONE = 0U,
    CAPTURED = 1U,
    NOT_LIVE = 2U,
    EXECUTION_NOT_READY = 3U,
    SOURCE_MISMATCH = 4U,
    SOURCE_REJECTED = 5U,
    NOT_CAPTURED = 6U,
    PIECE_OUTSIDE = 7U,
    QUERY_REJECTED = 8U,
    PIECE_READ = 9U,
    EVALUATED = 10U
};

struct NCPathCoreCompletedSnapshotInfoV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t runToken = 0ULL;
    std::uint64_t lifetime = 0ULL;
    MotionFeedbackSequence lastSequence = 0ULL;
    std::uint32_t count = 0U;
    std::uint16_t schemaVersion = 0U;
    std::uint16_t reserved = 0U;
    void Clear() noexcept;
};

struct NCPathCoreCompletedSnapshotWorkspaceV1
{
    NCPathCoreCommandedChordSubpathV1 subpath{};
    NCPathCoreCommandedChordSegmentV1 segment{};
    NCPathCoreCommandedChordSubpathPieceV1 piece{};
    NCPathCoreCommandedChordSubpathInfoV1 info{};
    NCPathCoreLiveRetentionStatusV1 retention{};
    NCPathCoreExecutionLinkStatusV1 execution{};
    void Clear() noexcept;
};

class NCPathCoreCompletedSnapshotV1 final
{
public:
    static constexpr std::uint32_t Capacity = NCPathCoreCommandedChordSnapshotV1::Capacity;
    NCPathCoreCompletedSnapshotV1() noexcept = default;
    NCPathCoreCompletedSnapshotV1(const NCPathCoreCompletedSnapshotV1&) = delete;
    NCPathCoreCompletedSnapshotV1& operator=(const NCPathCoreCompletedSnapshotV1&) = delete;
    NCPathCoreCompletedSnapshotV1(NCPathCoreCompletedSnapshotV1&&) = delete;
    NCPathCoreCompletedSnapshotV1& operator=(NCPathCoreCompletedSnapshotV1&&) = delete;
    ~NCPathCoreCompletedSnapshotV1() = default;

    void Clear() noexcept;
    NCPathCoreCompletedSnapshotCode Capture(
        NCPathCoreLiveRetention& retention,
        const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch,
        const NCPathCoreExecutionLink& execution,
        NCPathCoreCompletedSnapshotWorkspaceV1& workspace) noexcept;
    NCPathCoreCompletedSnapshotCode ReadPiece(
        std::uint32_t pieceIndex,
        NCPathCoreCommandedChordSegmentV1& segment,
        NCPathCoreExecutionRecordV1& execution,
        NCPathCoreCommandedChordSubpathPieceV1& piece,
        NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept;
    NCPathCoreCompletedSnapshotCode EvaluatePiece(
        std::uint32_t pieceIndex, double sourceU,
        NCPathCoreCommandedChordPositionSampleV1& sample,
        NCPathCoreExecutionRecordV1& execution) const noexcept;
    // Captured metadata only; does not consult or revalidate current sources.
    void Describe(NCPathCoreCompletedSnapshotInfoV1& output) const noexcept;

private:
    NCPathCoreCompletedSnapshotCode ValidateSelected(std::uint32_t pieceIndex) const noexcept;
    NCPathCoreCompletedSnapshotCode FailCapture(
        NCPathCoreCompletedSnapshotCode code,
        NCPathCoreCompletedSnapshotWorkspaceV1& workspace) noexcept;
    NCPathCoreCommandedChordSnapshotV1 m_geometry{};
    NCPathCoreExecutionRecordV1 m_records[Capacity]{};
    NCPathCoreCompletedSnapshotInfoV1 m_info{};
};

static_assert(sizeof(NCPathCoreCompletedSnapshotCode) == 1U &&
    NCPathCoreCompletedSnapshotV1::Capacity == 32U,
    "BP has fixed capacity 32 and one-byte result codes.");
static_assert(sizeof(NCPathCoreCompletedSnapshotInfoV1) == 40U &&
    alignof(NCPathCoreCompletedSnapshotInfoV1) == 8U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, ownerTag) == 0U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, runToken) == 8U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, lifetime) == 16U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, lastSequence) == 24U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, count) == 32U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, schemaVersion) == 36U &&
    offsetof(NCPathCoreCompletedSnapshotInfoV1, reserved) == 38U,
    "BP captured info native layout changed; not a wire or SHM ABI.");
static_assert(sizeof(NCPathCoreCompletedSnapshotWorkspaceV1) == 432U &&
    alignof(NCPathCoreCompletedSnapshotWorkspaceV1) == 8U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, subpath) == 0U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, segment) == 96U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, piece) == 256U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, info) == 312U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, retention) == 320U &&
    offsetof(NCPathCoreCompletedSnapshotWorkspaceV1, execution) == 368U,
    "BP heap workspace native layout changed.");
static_assert(sizeof(NCPathCoreCompletedSnapshotV1) == 8336U &&
    alignof(NCPathCoreCompletedSnapshotV1) == 8U,
    "BP detached heap snapshot is BD 5224 + BO 32 * 96 + info 40 bytes.");
static_assert(std::is_standard_layout<NCPathCoreCompletedSnapshotInfoV1>::value&&
    std::is_trivially_copyable<NCPathCoreCompletedSnapshotInfoV1>::value&&
    std::is_trivially_destructible<NCPathCoreCompletedSnapshotInfoV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCompletedSnapshotInfoV1>::value&&
    std::is_standard_layout<NCPathCoreCompletedSnapshotWorkspaceV1>::value&&
    std::is_trivially_copyable<NCPathCoreCompletedSnapshotWorkspaceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCompletedSnapshotWorkspaceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCompletedSnapshotWorkspaceV1>::value&&
    std::is_standard_layout<NCPathCoreCompletedSnapshotV1>::value&&
    std::is_trivially_destructible<NCPathCoreCompletedSnapshotV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCompletedSnapshotV1>::value &&
    !std::is_copy_constructible<NCPathCoreCompletedSnapshotV1>::value &&
    !std::is_copy_assignable<NCPathCoreCompletedSnapshotV1>::value &&
    !std::is_move_constructible<NCPathCoreCompletedSnapshotV1>::value &&
    !std::is_move_assignable<NCPathCoreCompletedSnapshotV1>::value,
    "BP sole detached owner and bounded heap workspace traits changed.");
