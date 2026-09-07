#pragma once

#include "NCPathCoreCommandedChordSnapshot.h"

// NC-0.2L.2BE / Bounded Snapshot Axis-Coordinate Candidate Query Contract.
// Classify EVERY piece of one captured BD subpath for one native-axis query.
// This is a mathematical candidate query, NOT exact intersection certification,
// multi-axis point-on-path proof, nearest-point selection, distance, Motion
// progress, executed history, planning, Path/Motion Queue or B2.
//
// Entries retain snapshot piece order (array index + 1 is its piece index).
// Distinct seam identities, POINT and zero-parameter clips are never merged.
// AY's original full-segment locate determines each classification:
// * NO_AXIS_MATCH: query is outside the ORIGINAL segment's axis range.
// * PARAMETER_INTERVAL: matching constant axis/POINT; the complete directed
//   clipFrom->clipTo interval is the answer, even if numerically a singleton.
//   No arbitrary u is selected. Original AY NON_UNIQUE status is retained.
// * LOCATED_IN_CLIP: AY's RETURNED APPROXIMATE u is within the closed clip.
// * CLIP_UNRESOLVED: AY located a u outside the clip. Keep its u/reconstruction;
//   do not claim no intersection from a rounded inverse outside the boundary.
// * PARAMETER_RESOLUTION_LOSS: AY could not represent an interior parameter.
//   This is a completed unresolved classification, not a manufactured u=0.
// ONLY LOCATED_IN_CLIP and CLIP_UNRESOLVED have valid candidateParameter and
// reconstructedCoordinateMCS. In all other kinds those fields remain zero.
// CLIP_UNRESOLVED and PARAMETER_RESOLUTION_LOSS must not be silently dropped
// from a caller's completeness decision. CLASSIFIED means all pieces received
// a classification, not that all exact intersections are solved or certified.
// No rounded AW clip-endpoint override, clamp or extra inverse math is added.
//
// No live Store is read. Cursor fields are CAPTURED PROVENANCE LABELS inherited
// from BD, not current bindings. A query remains legal after Store destruction;
// it grants no execution/control permission and does not authenticate origin.
// axisMask means submitted axes; constant unmasked axes can yield intervals.
// Values retain native mm/degree units; no mixed norm or rotary unwrap.
//
// Query order: BD ReadPiece(1) (captured-state/value rejection), then axis,
// finite coordinate, then classify all pieces. Structural/geometry/unexpected
// arithmetic failures clear ALL results and workspace. No partial list remains.
// On success workspace is ALSO cleared; all useful results are in output.
// Snapshot is unchanged on every exit. Inactive output entries are cleared.
//
// Heap-allocate Snapshot, Result and Workspace before use outside Observe/PDO.
// Result and Workspace reject implicit copy/move to avoid large stack copies.
// All objects must be disjoint and alive; one owning thread, no reentry or
// input/snapshot mutation for the WHOLE call. No cross-thread publication or
// arbitrary-memory-damage defense is supplied. BD/AY/AW's trusted provenance
// and precise floating-point preconditions are inherited unchanged.
//
// N<=32: exactly N BD ReadPiece and N AY Locate calls on successful queries,
// <=2N fixed eight-axis validations. Clear visits 32 output entries; error
// cleanup can clear again. No local Snapshot/Segment/Piece/Result/Workspace
// snapshot, allocation, unbounded scan, observer, thread, timer, mutex, wait,
// sleep, log, production caller/member, lifecycle or control hook is added.
// Work counts are not WCET or a full RTX64 call-chain stack bound.

enum class NCPathCoreCommandedChordAxisQueryCode : std::uint8_t
{
    NONE = 0U,
    CLASSIFIED = 1U,
    NOT_CAPTURED = 2U,
    SNAPSHOT_REJECTED = 3U,
    INVALID_AXIS = 4U,
    INVALID_COORDINATE = 5U,
    INVALID_GEOMETRY = 6U,
    NONFINITE_ARITHMETIC = 7U,
    GEOMETRY_REJECTED = 8U
};

enum class NCPathCoreCommandedChordAxisCandidateKind : std::uint8_t
{
    NONE = 0U,
    NO_AXIS_MATCH = 1U,
    PARAMETER_INTERVAL = 2U,
    LOCATED_IN_CLIP = 3U,
    CLIP_UNRESOLVED = 4U,
    PARAMETER_RESOLUTION_LOSS = 5U
};

struct NCPathCoreCommandedChordAxisCandidateV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double clipFrom = 0.0;
    double clipTo = 0.0;
    double candidateParameter = 0.0;
    double reconstructedCoordinateMCS = 0.0;
    NCPathCoreCommandedChordAxisCandidateKind kind = NCPathCoreCommandedChordAxisCandidateKind::NONE;
    NCPathCoreCommandedChordKind sourceKind = NCPathCoreCommandedChordKind::NONE;
    NCPathCoreCommandedChordLocateCode locateCode = NCPathCoreCommandedChordLocateCode::NOT_LOCATED;
    std::uint8_t reserved[5U]{};

    void Clear() noexcept
    {
        cursor.Clear();
        clipFrom = 0.0;
        clipTo = 0.0;
        candidateParameter = 0.0;
        reconstructedCoordinateMCS = 0.0;
        kind = NCPathCoreCommandedChordAxisCandidateKind::NONE;
        sourceKind = NCPathCoreCommandedChordKind::NONE;
        locateCode = NCPathCoreCommandedChordLocateCode::NOT_LOCATED;
        for (std::size_t i = 0U; i < 5U; ++i) reserved[i] = 0U;
    }
};

// Caller-owned output, not a wire ABI, observer or authentication object.
struct NCPathCoreCommandedChordAxisQueryResultV1 final
{
    NCPathCoreCommandedChordAxisQueryResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisQueryResultV1(const NCPathCoreCommandedChordAxisQueryResultV1&) = delete;
    NCPathCoreCommandedChordAxisQueryResultV1& operator=(const NCPathCoreCommandedChordAxisQueryResultV1&) = delete;
    NCPathCoreCommandedChordAxisQueryResultV1(NCPathCoreCommandedChordAxisQueryResultV1&&) = delete;
    NCPathCoreCommandedChordAxisQueryResultV1& operator=(NCPathCoreCommandedChordAxisQueryResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisQueryResultV1() = default;

    std::array<NCPathCoreCommandedChordAxisCandidateV1, NCPathCoreCommandedChordSnapshotV1::Capacity> entries{};
    double queryCoordinateMCS = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint32_t pieceCount = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection direction = NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t reserved[5U]{};

    void Clear() noexcept
    {
        for (auto& entry : entries) entry.Clear();
        queryCoordinateMCS = 0.0;
        axisIndex = 0U;
        pieceCount = 0U;
        schemaVersion = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        for (std::size_t i = 0U; i < 5U; ++i) reserved[i] = 0U;
    }
};

struct NCPathCoreCommandedChordAxisQueryWorkspaceV1 final
{
    NCPathCoreCommandedChordAxisQueryWorkspaceV1() noexcept = default;
    NCPathCoreCommandedChordAxisQueryWorkspaceV1(const NCPathCoreCommandedChordAxisQueryWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisQueryWorkspaceV1& operator=(const NCPathCoreCommandedChordAxisQueryWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisQueryWorkspaceV1(NCPathCoreCommandedChordAxisQueryWorkspaceV1&&) = delete;
    NCPathCoreCommandedChordAxisQueryWorkspaceV1& operator=(NCPathCoreCommandedChordAxisQueryWorkspaceV1&&) = delete;
    ~NCPathCoreCommandedChordAxisQueryWorkspaceV1() = default;

    NCPathCoreCommandedChordSegmentV1 segment{};
    NCPathCoreCommandedChordSubpathPieceV1 piece{};
    NCPathCoreCommandedChordSubpathInfoV1 info{};
    NCPathCoreCommandedChordLocationV1 location{};

    void Clear() noexcept
    {
        segment.Clear();
        piece.Clear();
        info.Clear();
        location.Clear();
    }
};

NCPathCoreCommandedChordAxisQueryCode QueryCommandedChordSnapshotAxisCandidates(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, std::uint32_t axisIndex,
    double queryCoordinateMCS, NCPathCoreCommandedChordAxisQueryWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisQueryResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisQueryCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisCandidateKind) == 1U,
    "BE status enums must remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisCandidateV1) == 80U &&
    alignof(NCPathCoreCommandedChordAxisCandidateV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, clipFrom) == 40U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, clipTo) == 48U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, candidateParameter) == 56U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, reconstructedCoordinateMCS) == 64U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, kind) == 72U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, sourceKind) == 73U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, locateCode) == 74U &&
    offsetof(NCPathCoreCommandedChordAxisCandidateV1, reserved) == 75U,
    "BE entry is one source cursor, four doubles and eight classification bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisQueryResultV1) == 2584U &&
    alignof(NCPathCoreCommandedChordAxisQueryResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisQueryResultV1, queryCoordinateMCS) == 2560U &&
    offsetof(NCPathCoreCommandedChordAxisQueryResultV1, pieceCount) == 2572U &&
    offsetof(NCPathCoreCommandedChordAxisQueryResultV1, schemaVersion) == 2576U,
    "BE heap result budget is 32 * 80 + 24 bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisQueryWorkspaceV1) == 272U &&
    alignof(NCPathCoreCommandedChordAxisQueryWorkspaceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisQueryWorkspaceV1, piece) == 160U &&
    offsetof(NCPathCoreCommandedChordAxisQueryWorkspaceV1, info) == 216U &&
    offsetof(NCPathCoreCommandedChordAxisQueryWorkspaceV1, location) == 224U,
    "BE heap workspace is Segment160 + Piece56 + Info8 + Location48.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisCandidateV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisCandidateV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisCandidateV1>::value,
    "BE entries must remain plain bounded values.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisQueryResultV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisQueryResultV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisQueryResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisQueryResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisQueryResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisQueryResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisQueryResultV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisQueryWorkspaceV1>::value,
    "BE result/workspace must forbid implicit bulk copies and remain heap-owned.");
