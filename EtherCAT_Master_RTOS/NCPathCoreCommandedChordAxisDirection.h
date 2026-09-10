#pragma once

#include "NCPathCoreCommandedChordAxisVariation.h"

// NC-0.2L.2BL / Clipped Snapshot Native-Axis Direction Profile Contract.
// Classify every captured piece on ALL eight native MCS axes, using only
// comparisons of its ORIGINAL finite endpoints and original clip parameters:
// equal clip parameters -> ZERO_PARAMETER_SPAN (first priority);
// otherwise equal source coordinates -> CONSTANT_AXIS;
// otherwise INCREASING iff source-coordinate and parameter order agree,
// DECREASING otherwise. No coordinate/delta/variation/prefix computation,
// interpolation, epsilon, clamping, normalization or arbitrary u selection.
// Signed zeros compare numerically equal; original clip bits are preserved.
// Unmasked axes, POINTs, zero clips and distinct original seam identities are
// retained. One entry per snapshot traversal piece; no merge or renumbering.
//
// Each axis summary partitions pieceCount into four category counts. Its
// reversalCount counts sign changes between successive MOVING entries only,
// ignoring intervening zero clips and constants. first/lastMovingPieceIndex
// are ONE-based SNAPSHOT traversal indices (not original ordinals), or zero
// if no entry moves. Trend is FLAT, NONDECREASING, NONINCREASING or
// DIRECTION_REVERSAL according to whether neither, only increasing, only
// decreasing or both moving kinds occur. Successful trends are never NONE.
//
// This is the exact sign of the ideal affine commanded chord's coordinate
// change under inherited FP comparison preconditions, including when a
// hypothetical subtraction/variation would overflow/underflow or AW-rounded
// clipped coordinates would coincide. It does not certify AW sample strict
// monotonicity, station uniqueness, a unique inverse, real G00 travel/history,
// speed, Motion progress/permission, planning, Path/Motion Queue or B2. No
// distance, mixed mm/degree metric, rotary unwrap or coordinate bound is given.
// Exact reversal swaps increasing/decreasing and maps moving indices; zero/
// constant kinds and reversal count survive. Do not extend that symmetry to
// BF totals or AW rounded values. Split/join may change piece/category counts.
//
// Reuse the existing BF workspace ONLY for BD ReadPiece; no BF query is called.
// ReadPiece(1) validates captured state/value first; reuse that first read.
// N<=32 successful pieces use exactly N selected fixed-eight-axis validations
// and 8N classifications/summary updates, then eight trend updates. Shared
// clip order and coordinate comparisons require at most 18N comparisons,
// excluding BD/AY validation. All source values are copied, not recomputed.
// Every failure clears ALL output/workspace fields. Success clears workspace
// too; inactive entries stay clear. Count/direction/schema commit only after
// all pieces succeed, not as cross-thread publication. Source is unchanged.
//
// Source, output and workspace must be disjoint, alive and heap-preallocated
// outside Observe/PDO; one owning thread with no mutation/reentry during the
// WHOLE call. Output retains values/provenance labels, no pointer/source lease.
// Detached results survive source Clear/recapture/destruction and original
// Store append/fault/invalidate/reopen/destruction. A new query reads current
// captured values, not an old ticket. Trusted owner/lifetime/non-reuse remain
// external; private value checks are not arbitrary-memory-damage detection.
//
// No allocation, local bulk object, observer, thread, timer, mutex, wait,
// sleep, log, runtime member/caller, lifecycle, PDO, Alarm/Gate or Motion hook.
// Clear visits all 32 entries/eight summaries and may repeat on failure.
// Work counts are not WCET or a full RTX64 call-chain stack bound. Inherited
// precise FP, round-to-nearest, gradual underflow and masked exceptions remain
// required. BL adds comparisons, not floating arithmetic, and does not change
// FP controls; no promise about inherited helpers' sticky exception flags.

enum class NCPathCoreCommandedChordAxisDirectionCode : std::uint8_t
{
    NONE = 0U,
    PROFILED = 1U,
    NOT_CAPTURED = 2U,
    SNAPSHOT_REJECTED = 3U
};

enum class NCPathCoreCommandedChordAxisDirectionKind : std::uint8_t
{
    NONE = 0U,
    ZERO_PARAMETER_SPAN = 1U,
    CONSTANT_AXIS = 2U,
    INCREASING = 3U,
    DECREASING = 4U
};

enum class NCPathCoreCommandedChordAxisDirectionTrend : std::uint8_t
{
    NONE = 0U,
    FLAT = 1U,
    NONDECREASING = 2U,
    NONINCREASING = 3U,
    DIRECTION_REVERSAL = 4U
};

struct NCPathCoreCommandedChordAxisDirectionPieceV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double clipFrom = 0.0;
    double clipTo = 0.0;
    std::array<NCPathCoreCommandedChordAxisDirectionKind,
        NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY> axisDirections{};
    NCPathCoreCommandedChordKind sourceKind = NCPathCoreCommandedChordKind::NONE;
    std::uint8_t reserved[7U]{};

    void Clear() noexcept
    {
        cursor.Clear();
        clipFrom = 0.0;
        clipTo = 0.0;
        for (auto& kind : axisDirections) kind = NCPathCoreCommandedChordAxisDirectionKind::NONE;
        sourceKind = NCPathCoreCommandedChordKind::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

struct NCPathCoreCommandedChordAxisDirectionSummaryV1
{
    std::uint32_t zeroSpanCount = 0U;
    std::uint32_t constantAxisCount = 0U;
    std::uint32_t increasingCount = 0U;
    std::uint32_t decreasingCount = 0U;
    std::uint32_t reversalCount = 0U;
    std::uint32_t firstMovingPieceIndex = 0U;
    std::uint32_t lastMovingPieceIndex = 0U;
    NCPathCoreCommandedChordAxisDirectionTrend trend = NCPathCoreCommandedChordAxisDirectionTrend::NONE;
    std::uint8_t reserved[3U]{};

    void Clear() noexcept
    {
        zeroSpanCount = 0U;
        constantAxisCount = 0U;
        increasingCount = 0U;
        decreasingCount = 0U;
        reversalCount = 0U;
        firstMovingPieceIndex = 0U;
        lastMovingPieceIndex = 0U;
        trend = NCPathCoreCommandedChordAxisDirectionTrend::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

// Process-local caller-owned output, not a wire/SHM ABI or runtime member.
struct NCPathCoreCommandedChordAxisDirectionResultV1 final
{
    NCPathCoreCommandedChordAxisDirectionResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisDirectionResultV1(const NCPathCoreCommandedChordAxisDirectionResultV1&) = delete;
    NCPathCoreCommandedChordAxisDirectionResultV1& operator=(const NCPathCoreCommandedChordAxisDirectionResultV1&) = delete;
    NCPathCoreCommandedChordAxisDirectionResultV1(NCPathCoreCommandedChordAxisDirectionResultV1&&) = delete;
    NCPathCoreCommandedChordAxisDirectionResultV1& operator=(NCPathCoreCommandedChordAxisDirectionResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisDirectionResultV1() = default;

    std::array<NCPathCoreCommandedChordAxisDirectionPieceV1,
        NCPathCoreCommandedChordSnapshotV1::Capacity> entries{};
    std::array<NCPathCoreCommandedChordAxisDirectionSummaryV1,
        NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY> axes{};
    std::uint32_t pieceCount = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection direction = NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t reserved = 0U;

    void Clear() noexcept
    {
        for (auto& entry : entries) entry.Clear();
        for (auto& axis : axes) axis.Clear();
        pieceCount = 0U;
        schemaVersion = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        reserved = 0U;
    }
};

NCPathCoreCommandedChordAxisDirectionCode ProfileCommandedChordSnapshotAxisDirections(
    const NCPathCoreCommandedChordSnapshotV1& snapshot,
    NCPathCoreCommandedChordAxisVariationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisDirectionResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisDirectionCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisDirectionKind) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisDirectionTrend) == 1U,
    "BL status, direction and trend enums remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisDirectionPieceV1) == 72U &&
    alignof(NCPathCoreCommandedChordAxisDirectionPieceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, clipFrom) == 40U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, clipTo) == 48U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, axisDirections) == 56U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, sourceKind) == 64U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionPieceV1, reserved) == 65U,
    "BL entry is one cursor, two doubles and sixteen classification bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisDirectionSummaryV1) == 32U &&
    alignof(NCPathCoreCommandedChordAxisDirectionSummaryV1) == 4U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, zeroSpanCount) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, constantAxisCount) == 4U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, increasingCount) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, decreasingCount) == 12U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, reversalCount) == 16U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, firstMovingPieceIndex) == 20U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, lastMovingPieceIndex) == 24U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, trend) == 28U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionSummaryV1, reserved) == 29U,
    "BL summary is seven bounded counters and four trend/reserved bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisDirectionResultV1) == 2568U &&
    alignof(NCPathCoreCommandedChordAxisDirectionResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, entries) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, axes) == 2304U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, pieceCount) == 2560U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, schemaVersion) == 2564U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, direction) == 2566U &&
    offsetof(NCPathCoreCommandedChordAxisDirectionResultV1, reserved) == 2567U,
    "BL heap result budget is 32 * 72 + 8 * 32 + 8 bytes.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisDirectionPieceV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisDirectionPieceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisDirectionPieceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisDirectionPieceV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordAxisDirectionSummaryV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisDirectionSummaryV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisDirectionSummaryV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisDirectionSummaryV1>::value,
    "BL entries and summaries remain plain bounded mathematical values.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisDirectionResultV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisDirectionResultV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisDirectionResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisDirectionResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisDirectionResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisDirectionResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisDirectionResultV1>::value,
    "BL result must stay heap-owned without implicit bulk copies.");
