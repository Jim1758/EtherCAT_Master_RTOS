#pragma once

#include "NCPathCoreCommandedChordStore.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2BA / Retained Commanded Chord Cursor V1 and Bounded Bidirectional
// Traversal Contract. Walk only the commanded segments retained by AZ.
// This is NOT a Motion/execution/B2 cursor, a queue, a segment-internal u,
// a current-D proof, or permission to move an axis.
//
// Caller preconditions, inherited from AZ:
// * Every call uses a live Store on its single owning thread. No reentry,
//   concurrent mutation, append, invalidate or reopen during the WHOLE call.
//   Several const calls do not form a cross-thread atomic snapshot.
// * Store and Segment output workspaces are heap-owned before realtime use.
//   No operation here allocates. No Segment/Store stack temporary is made.
// * ALL outputs are disjoint from inputs and the store. In particular, Move
//   takes separate current/output cursors; in-place/overlapping use is invalid.
// * ownerTag uniqueness, non-reuse while handles survive, trusted source owner
//   and source lifetime are caller obligations. Matching a handle is NOT
//   authentication. Another valid binding cannot be distinguished from a copy.
//
// Every Read/Move validates owner/lifetime/ordinal/reserved/local identity and
// current retained value BEFORE boundary decisions. No stale cursor is silently
// rebound. Explicit Bind is the only way to select a new lifetime/position.
// All errors clear output. AT_HEAD/AT_TAIL are valid, UNMOVED positions: output
// receives the revalidated current cursor. They are not MOVED results.
//
// Tail means the current retained tail for THIS call, not a frozen extent.
// Legal append between calls preserves old cursors; a prior AT_TAIL can later
// MoveNext. Invalidate, a new lifetime or any store fault rejects old cursors.
// Only an OPEN empty store produces EMPTY_STORE, and only during explicit Bind.
// All calls are read-only for the Store; errors never poison it or affect NC.
// Previously copied Segments remain mathematical values, not readable coverage.
//
// Bind: at most 2 AZ const calls. Read/Move: at most 3 AZ const calls, with at
// most two fixed eight-axis validations. No retained-history scan, pointer
// borrow, allocation, observer, thread, timer, mutex, wait, sleep or logging.
// No production instance/caller, NC lifecycle hook or member budget is added.

enum class NCPathCoreCommandedChordCursorCode : std::uint8_t
{
    NONE = 0U,
    BOUND = 1U,
    MOVED_NEXT = 2U,
    MOVED_PREVIOUS = 3U,
    VALUE_READ = 4U,
    AT_HEAD = 5U,
    AT_TAIL = 6U,
    STORE_CLOSED = 7U,
    STORE_FAULTED = 8U,
    EMPTY_STORE = 9U,
    ORDINAL_OUTSIDE = 10U,
    CURSOR_REJECTED = 11U,
    RETAINED_VALUE_INVALID = 12U,
    STORE_REJECTED = 13U
};

// Process-local, copyable value; not a pointer, wire/SHM ABI or owner token.
// All-zero/default is invalid. No cached tail or second lifetime is stored.
struct NCPathCoreCommandedChordCursorV1
{
    NCPathCoreCommandedChordStoreHandleV1 binding{};

    void Clear() noexcept { binding.Clear(); }
};

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorHead(
    const NCPathCoreCommandedChordStoreV1& store,
    NCPathCoreCommandedChordCursorV1& output) noexcept;

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorTail(
    const NCPathCoreCommandedChordStoreV1& store,
    NCPathCoreCommandedChordCursorV1& output) noexcept;

NCPathCoreCommandedChordCursorCode BindCommandedChordCursorAtOrdinal(
    const NCPathCoreCommandedChordStoreV1& store, std::uint32_t ordinal,
    NCPathCoreCommandedChordCursorV1& output) noexcept;

NCPathCoreCommandedChordCursorCode ReadCommandedChordCursorCurrent(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordSegmentV1& output) noexcept;

NCPathCoreCommandedChordCursorCode MoveCommandedChordCursorNext(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordCursorV1& output) noexcept;

NCPathCoreCommandedChordCursorCode MoveCommandedChordCursorPrevious(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& current,
    NCPathCoreCommandedChordCursorV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordCursorCode) == 1U,
    "BA result code must remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordCursorV1) == 40U &&
    alignof(NCPathCoreCommandedChordCursorV1) == 8U &&
    offsetof(NCPathCoreCommandedChordCursorV1, binding) == 0U,
    "BA cursor budget is one AZ handle, 40 bytes / alignment 8, not a wire ABI.");
static_assert(std::is_same<decltype(NCPathCoreCommandedChordCursorV1::binding),
    NCPathCoreCommandedChordStoreHandleV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordCursorV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordCursorV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordCursorV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordCursorV1>::value,
    "BA cursor must remain a small, standalone handle value.");
