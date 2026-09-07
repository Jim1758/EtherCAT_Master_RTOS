#pragma once

#include "NCPathCoreCommandedChordSnapshot.h"

// NC-0.2L.2BF / Bounded Snapshot Native-Axis Variation Profile Contract.
// For each captured BD piece, approximate |b-a| * |clipTo-clipFrom| using
// ORIGINAL source endpoints on ONE native MCS axis. This measures commanded
// chord variation, not the difference of AW-rounded clipped coordinates,
// mixed mm/degree length, rotary unwrap, actual G00 travel, Motion progress,
// execution history, station location, planning, Path/Motion Queue or B2.
//
// Entries stay in snapshot traversal order, one per piece, including POINT,
// constant axes, zero clips and distinct seam identities. ZERO_PARAMETER_SPAN
// takes precedence over CONSTANT_AXIS; both have +0 variation. Otherwise the
// piece has POSITIVE_VARIATION, or the WHOLE query fails. No arbitrary u is
// selected on constant axes. Clips (including signed zero) and source kind
// are preserved. Unmasked constant axes are legal; axisMask is not motion.
//
// Rounded-double arithmetic contract, under inherited AY precise-FP state:
// span=abs(clipTo-clipFrom), width=abs(b-a), variation=width*span. If width
// overflows, use (abs(b*0.5-a*0.5)*span)*2 instead. This permits representable
// clipped variation even when the full width overflows. Zero pieces bypass
// width arithmetic. Nonfinite variation and positive variation rounded to zero
// fail explicitly. Prefixes use ordinary addition in TRAVERSAL order starting
// at +0; nonfinite totals or positive variation failing to increase a prefix
// fail explicitly. No partial profile, saturation, nextafter or compensation.
// prefixTo-prefixFrom need not equal variation. Positive subnormal results
// are allowed; finite results are approximations, not correctly rounded exact
// values or certified bounds. Reversal/repartition/additivity invariance and
// exact representability conclusions from failure are NOT supplied. No FTZ
// detection or change of FP control state; arithmetic may set sticky FP flags.
// Round-to-nearest, gradual underflow
// and masked FP exceptions remain caller preconditions (as in AY/AW).
//
// ReadPiece(1) captured-state/value validation precedes axis validation; reuse
// that first read. N<=32 successful pieces use exactly N BD ReadPiece calls
// and N fixed eight-axis validations; no live Store/AY locate/evaluate calls.
// Captured cursors are provenance LABELS, not revalidated current bindings.
// Query is pure even after Store append/invalidate/fault/reopen/destruction.
// It grants no control permission, source authentication or path availability.
//
// All failures clear ALL output/workspace fields; success also clears workspace.
// Inactive entries remain clear; schema/count are written after all pieces.
// Snapshot is unchanged on every exit. Heap-allocate Snapshot/Result/Workspace
// before use outside Observe/PDO. All objects must be disjoint and alive;
// one owning thread, no reentry/mutation during the WHOLE operation. No cross-
// thread publication or arbitrary-memory-damage defense is supplied. Copy/move
// of bulk result/workspace are deleted. No local bulk snapshot or allocation,
// observer, thread, timer, mutex, wait, sleep, log, production caller/member,
// lifecycle, Alarm/Gate or Motion hook. Clear visits 32 result entries and
// may repeat on failure; work counts are not WCET or target call-chain stack.

enum class NCPathCoreCommandedChordAxisVariationCode : std::uint8_t
{
    NONE = 0U,
    PROFILED = 1U,
    NOT_CAPTURED = 2U,
    SNAPSHOT_REJECTED = 3U,
    INVALID_AXIS = 4U,
    NONFINITE_VARIATION = 5U,
    VARIATION_RESOLUTION_LOSS = 6U,
    NONFINITE_TOTAL = 7U,
    PREFIX_RESOLUTION_LOSS = 8U
};

enum class NCPathCoreCommandedChordAxisVariationKind : std::uint8_t
{
    NONE = 0U,
    ZERO_PARAMETER_SPAN = 1U,
    CONSTANT_AXIS = 2U,
    POSITIVE_VARIATION = 3U
};

struct NCPathCoreCommandedChordAxisVariationPieceV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double clipFrom = 0.0;
    double clipTo = 0.0;
    double variation = 0.0;
    double prefixFrom = 0.0;
    double prefixTo = 0.0;
    NCPathCoreCommandedChordAxisVariationKind kind = NCPathCoreCommandedChordAxisVariationKind::NONE;
    NCPathCoreCommandedChordKind sourceKind = NCPathCoreCommandedChordKind::NONE;
    std::uint8_t reserved[6U]{};

    void Clear() noexcept
    {
        cursor.Clear();
        clipFrom = 0.0;
        clipTo = 0.0;
        variation = 0.0;
        prefixFrom = 0.0;
        prefixTo = 0.0;
        kind = NCPathCoreCommandedChordAxisVariationKind::NONE;
        sourceKind = NCPathCoreCommandedChordKind::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

// Process-local caller-owned output, not a wire/SHM ABI or runtime member.
struct NCPathCoreCommandedChordAxisVariationResultV1 final
{
    NCPathCoreCommandedChordAxisVariationResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisVariationResultV1(const NCPathCoreCommandedChordAxisVariationResultV1&) = delete;
    NCPathCoreCommandedChordAxisVariationResultV1& operator=(const NCPathCoreCommandedChordAxisVariationResultV1&) = delete;
    NCPathCoreCommandedChordAxisVariationResultV1(NCPathCoreCommandedChordAxisVariationResultV1&&) = delete;
    NCPathCoreCommandedChordAxisVariationResultV1& operator=(NCPathCoreCommandedChordAxisVariationResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisVariationResultV1() = default;

    std::array<NCPathCoreCommandedChordAxisVariationPieceV1, NCPathCoreCommandedChordSnapshotV1::Capacity> entries{};
    double totalVariation = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint32_t pieceCount = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection direction = NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t reserved[5U]{};

    void Clear() noexcept
    {
        for (auto& entry : entries) entry.Clear();
        totalVariation = 0.0;
        axisIndex = 0U;
        pieceCount = 0U;
        schemaVersion = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

struct NCPathCoreCommandedChordAxisVariationWorkspaceV1 final
{
    NCPathCoreCommandedChordAxisVariationWorkspaceV1() noexcept = default;
    NCPathCoreCommandedChordAxisVariationWorkspaceV1(const NCPathCoreCommandedChordAxisVariationWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& operator=(const NCPathCoreCommandedChordAxisVariationWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisVariationWorkspaceV1(NCPathCoreCommandedChordAxisVariationWorkspaceV1&&) = delete;
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& operator=(NCPathCoreCommandedChordAxisVariationWorkspaceV1&&) = delete;
    ~NCPathCoreCommandedChordAxisVariationWorkspaceV1() = default;

    NCPathCoreCommandedChordSegmentV1 segment{};
    NCPathCoreCommandedChordSubpathPieceV1 piece{};
    NCPathCoreCommandedChordSubpathInfoV1 info{};

    void Clear() noexcept { segment.Clear(); piece.Clear(); info.Clear(); }
};

NCPathCoreCommandedChordAxisVariationCode MeasureCommandedChordSnapshotAxisVariation(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, std::uint32_t axisIndex,
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisVariationResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisVariationCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisVariationKind) == 1U,
    "BF status and classification enums remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisVariationPieceV1) == 88U &&
    alignof(NCPathCoreCommandedChordAxisVariationPieceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, clipFrom) == 40U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, clipTo) == 48U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, variation) == 56U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, prefixFrom) == 64U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, prefixTo) == 72U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, kind) == 80U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, sourceKind) == 81U &&
    offsetof(NCPathCoreCommandedChordAxisVariationPieceV1, reserved) == 82U,
    "BF piece is one cursor, five doubles and eight classification bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisVariationResultV1) == 2840U &&
    alignof(NCPathCoreCommandedChordAxisVariationResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, entries) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, totalVariation) == 2816U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, axisIndex) == 2824U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, pieceCount) == 2828U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, schemaVersion) == 2832U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, direction) == 2834U &&
    offsetof(NCPathCoreCommandedChordAxisVariationResultV1, reserved) == 2835U,
    "BF heap result budget is 32 * 88 + 24 bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisVariationWorkspaceV1) == 224U &&
    alignof(NCPathCoreCommandedChordAxisVariationWorkspaceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisVariationWorkspaceV1, segment) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisVariationWorkspaceV1, piece) == 160U &&
    offsetof(NCPathCoreCommandedChordAxisVariationWorkspaceV1, info) == 216U,
    "BF heap workspace is Segment160 + Piece56 + Info8.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisVariationPieceV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisVariationPieceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisVariationPieceV1>::value,
    "BF entries remain plain bounded mathematical values.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisVariationResultV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisVariationResultV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisVariationResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisVariationResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisVariationResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisVariationResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisVariationResultV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisVariationWorkspaceV1>::value,
    "BF result/workspace must stay heap-owned without implicit bulk copies.");
