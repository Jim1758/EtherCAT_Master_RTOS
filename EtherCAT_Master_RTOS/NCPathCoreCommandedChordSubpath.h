#pragma once

#include "NCPathCoreCommandedChordPosition.h"

// NC-0.2L.2BC / Retained Commanded Subpath V1 and Bounded Directed Piece Query
// Contract. Two BB positions delimit a fixed, directed retained subpath.
// This is commanded geometry only: no distance, Motion/feedback/executed
// progress, trajectory planning, queue, B2, rotary unwrap or mixed-unit norm.
//
// Piece indices are ONE-based. Count is abs(end.ordinal-start.ordinal)+1,
// always 1..32 after validation. Direction compares ordinals first, then u
// when ordinals match; equal numeric u (including +/-0) is STATIONARY.
// Forward pieces: start.u->1, zero or more 0->1, then 0->end.u.
// Reverse pieces: start.u->0, zero or more 1->0, then 1->end.u.
// Same ordinal: exactly one start.u->end.u piece, including STATIONARY.
// Zero-parameter endpoint pieces and POINT chords are NEVER skipped.
// In particular (n,1)->(n+1,0) has TWO pieces, preserving BB seam identity.
// Direction is parameter order, not physical displacement or execution.
//
// BOTH APIs revalidate start cursor, THEN end cursor through BA ReadCurrent,
// BEFORE checking either u, then (for ReadPiece) the requested piece index.
// Source rejection takes precedence over bad scalars/index. ReadPiece also
// validates the selected retained segment via AZ GetHandleAtOrdinal + Read.
// No silent rebind of endpoints. Legal append never extends a bound subpath;
// invalidate/new lifetime/fault rejects it. No count/direction is cached.
// Info describes only this successful call, never durable readable coverage.
//
// Every failure clears ALL outputs AND the Segment workspace. On Bind success
// workspace holds the end segment; on ReadPiece success the selected segment.
// Signed-zero input parameters are preserved; invalid u is never clamped.
// Pure copies remain mathematical values, not source authentication/control.
//
// Preconditions inherited from AZ/BA/BB: one owning thread, live Store, no
// reentry or Store mutation during the WHOLE call. ALL input/output/workspace
// objects are disjoint (no in-place Bind or output-member/input aliases).
// Store, Segment workspace and Subpath values are caller-owned heap storage,
// allocated before use outside realtime Observe/PDO paths. No operation
// allocates; no local Segment/Store/Subpath/Piece/Sample snapshots are made.
// ownerTag non-reuse, trusted source lifetime and precise FP remain external
// preconditions. A modified but valid value is not rejected as a forgery.
//
// Bind: <=6 AZ const calls / 4 fixed eight-axis validations. ReadPiece:
// <=8 AZ const calls / 6 fixed eight-axis validations. No history scan or
// per-piece loop. No production caller/member/lifecycle hook, observer, thread,
// timer, mutex, wait, sleep, log, Alarm/Gate or Motion interaction is added.

enum class NCPathCoreCommandedChordSubpathCode : std::uint8_t
{
    NONE = 0U,
    SUBPATH_BOUND = 1U,
    PIECE_READ = 2U,
    STORE_CLOSED = 3U,
    STORE_FAULTED = 4U,
    CURSOR_REJECTED = 5U,
    RETAINED_VALUE_INVALID = 6U,
    STORE_REJECTED = 7U,
    INVALID_PARAMETER = 8U,
    PIECE_INDEX_OUTSIDE = 9U
};

enum class NCPathCoreCommandedChordSubpathDirection : std::uint8_t
{
    NONE = 0U,
    FORWARD = 1U,
    REVERSE = 2U,
    STATIONARY = 3U
};

// Process-local values, not a wire/SHM ABI. Cleared bindings are invalid.
struct NCPathCoreCommandedChordSubpathV1
{
    NCPathCoreCommandedChordPositionV1 start{};
    NCPathCoreCommandedChordPositionV1 end{};

    void Clear() noexcept { start.Clear(); end.Clear(); }
};

struct NCPathCoreCommandedChordSubpathPieceV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double fromParameter = 0.0;
    double toParameter = 0.0;

    void Clear() noexcept
    {
        cursor.Clear();
        fromParameter = 0.0;
        toParameter = 0.0;
    }
};

struct NCPathCoreCommandedChordSubpathInfoV1
{
    std::uint32_t pieceCount = 0U;
    NCPathCoreCommandedChordSubpathDirection direction =
        NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t reserved[3U]{};

    void Clear() noexcept
    {
        pieceCount = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        reserved[0U] = 0U;
        reserved[1U] = 0U;
        reserved[2U] = 0U;
    }
};

NCPathCoreCommandedChordSubpathCode BindCommandedChordSubpath(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordPositionV1& start,
    const NCPathCoreCommandedChordPositionV1& end,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSubpathV1& output,
    NCPathCoreCommandedChordSubpathInfoV1& info) noexcept;

NCPathCoreCommandedChordSubpathCode ReadCommandedChordSubpathPiece(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordSubpathV1& subpath, std::uint32_t pieceIndex,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSubpathPieceV1& output,
    NCPathCoreCommandedChordSubpathInfoV1& info) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordSubpathCode) == 1U &&
    sizeof(NCPathCoreCommandedChordSubpathDirection) == 1U,
    "BC result and direction must remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordSubpathV1) == 96U &&
    alignof(NCPathCoreCommandedChordSubpathV1) == 8U &&
    offsetof(NCPathCoreCommandedChordSubpathV1, start) == 0U &&
    offsetof(NCPathCoreCommandedChordSubpathV1, end) == 48U,
    "BC subpath is exactly two BB positions, not a wire ABI.");
static_assert(sizeof(NCPathCoreCommandedChordSubpathPieceV1) == 56U &&
    alignof(NCPathCoreCommandedChordSubpathPieceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordSubpathPieceV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordSubpathPieceV1, fromParameter) == 40U &&
    offsetof(NCPathCoreCommandedChordSubpathPieceV1, toParameter) == 48U,
    "BC piece is one BA cursor and two parameters.");
static_assert(sizeof(NCPathCoreCommandedChordSubpathInfoV1) == 8U &&
    alignof(NCPathCoreCommandedChordSubpathInfoV1) == 4U &&
    offsetof(NCPathCoreCommandedChordSubpathInfoV1, pieceCount) == 0U &&
    offsetof(NCPathCoreCommandedChordSubpathInfoV1, direction) == 4U &&
    offsetof(NCPathCoreCommandedChordSubpathInfoV1, reserved) == 5U,
    "BC info is bounded derived metadata, never a cached observer.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordSubpathV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordSubpathV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordSubpathV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordSubpathV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordSubpathPieceV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordSubpathPieceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordSubpathPieceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordSubpathPieceV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordSubpathInfoV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordSubpathInfoV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordSubpathInfoV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordSubpathInfoV1>::value,
    "BC values must remain bounded plain caller-owned storage.");
