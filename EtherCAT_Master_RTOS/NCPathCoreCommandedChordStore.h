#pragma once

#include "NCPathCoreCommandedChordSegment.h"
#include <array>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AZ / Fixed-Capacity Commanded Chord Retention and Store-Lifetime
// Contract. A standalone append-only store of ONE commanded chord chain.
// This is real bounded payload storage, NOT another observer and NOT a queue.
// There is no dispatch/pop/consume, Motion admission/completion, automatic NC
// integration, RESET/STOP hook, current-D proof, cursor, or B2 execution.
//
// INTEGRATION PRECONDITIONS (not supplied or inferred by this component):
// * Allocate the store AND segment workspaces on the heap before use, outside
//   any realtime Observe/PDO path. Do not instantiate this 5-KiB type locally.
// * One owning NC thread, no reentry/mutation during a call; no cross-thread
//   publication. All output workspaces are disjoint from inputs and this store.
// * Assign a nonzero ownerTag, unique across all store instances whose handles
//   can coexist (INCLUDING destroyed/reconstructed stores). Never reuse that
//   tag while an old handle may survive. This is a caller-owned namespace, NOT
//   an authenticated token or a globally allocated ID. There is no registry.
// * All appended AY values must be captured from the SAME trusted D owner and
//   source lifetime, with stable MCS/native-axis configuration. A store ownerTag
//   does NOT authenticate D provenance or detect a forged but valid value.
// * Before a source lifetime/configuration change, explicitly invalidate and
//   open a NEW, strictly larger store lifetime. No Motion epoch is relabelled.
//
// LIFETIME: BeginLifetime(g) requires g>0 and g>the largest accepted g. There is
// no wrap, no reuse, no automatic reopen. Invalidate clears payload but retains
// the high-water lifetime. A rejected BeginLifetime FAILS CLOSED; only a later
// valid greater lifetime recovers. UINT64_MAX can be opened once, never reused.
//
// ADMISSION: FIRST_INPUT/CHAIN_BOUNDARY run=1 is required as stored head. Each
// later value must have same chain key, exact next nonzero D publication
// (UINT64_MAX->1 allowed), CONTIGUOUS_PAIR, saturated-next run length, and equal
// native-coordinate end/start on ALL eight axes (numeric equality; +/-0 join).
// This proves a local numeric commanded seam, NOT E/F evidence or Motion.
// Exact tail-key + bit-exact payload replay is idempotent, even at capacity.
// Same key + changed payload is a conflict. There is NO overwrite/eviction,
// head synthesis, gap repair, chain switch, fuzzy join, or search for old data.
//
// FAILURE: first admission/lifetime fault latches. All handles/read queries
// fail and clear outputs until explicit invalidation/new lifetime. Physically
// retained count is diagnostic only while faulted; it is NOT readable coverage.
// Bad READ handles/ordinals are read-only rejections and do not poison a store.
// Existing copies outside the store remain mathematical values, never current
// execution proof. Validation does not defend against arbitrary memory damage.
//
// All payload copies/clears are bounded. No allocation, thread, timer, mutex,
// wait, sleep, log or source borrow is performed by any operation. Precise FP
// and native-coordinate rules are inherited unchanged from AY/AW/AX.

enum class NCPathCoreCommandedChordStoreState : std::uint8_t
{
    CLOSED = 0U,
    OPEN = 1U,
    FAULTED = 2U
};

enum class NCPathCoreCommandedChordStoreCode : std::uint8_t
{
    NONE = 0U,
    OPENED = 1U,
    APPENDED = 2U,
    ALREADY_RETAINED = 3U,
    VALUE_READ = 4U,
    HANDLE_READ = 5U,
    NOT_OPEN = 6U,
    INVALID_OWNER = 7U,
    INVALID_LIFETIME = 8U,
    LIFETIME_NOT_NEW = 9U,
    LIFETIME_EXHAUSTED = 10U,
    INVALID_VALUE = 11U,
    MISSING_HEAD = 12U,
    KEY_CONFLICT = 13U,
    CHAIN_CHANGE = 14U,
    SOURCE_GAP = 15U,
    RUN_DISCONTINUITY = 16U,
    DISCONNECTED_CHORD = 17U,
    CAPACITY_EXCEEDED = 18U,
    HANDLE_REJECTED = 19U,
    ORDINAL_OUTSIDE = 20U,
    RETAINED_VALUE_INVALID = 21U
};

// Process-local value handle, NOT a pointer, wire/SHM ABI, current ticket,
// source authentication, traversal cursor or permission to move an axis.
struct NCPathCoreCommandedChordStoreHandleV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t lifetime = 0ULL;
    std::uint32_t ordinal = 0U; // One-based, within THIS store lifetime only.
    std::uint32_t reserved = 0U;
    NCPathCoreCommandedChordLocalIdentityV1 localIdentity{};

    void Clear() noexcept
    {
        ownerTag = 0ULL;
        lifetime = 0ULL;
        ordinal = 0U;
        reserved = 0U;
        localIdentity.Clear();
    }
};

// Constant-size metadata, no endpoint arrays/event list/history reconstruction.
struct NCPathCoreCommandedChordStoreInfoV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t lastAcceptedLifetime = 0ULL;
    std::uint32_t storedCount = 0U;
    std::uint32_t readableCount = 0U;
    std::uint32_t capacity = 0U;
    NCPathCoreCommandedChordStoreState state = NCPathCoreCommandedChordStoreState::CLOSED;
    NCPathCoreCommandedChordStoreCode firstFault = NCPathCoreCommandedChordStoreCode::NONE;
    std::uint8_t reserved[2U]{};

    void Clear() noexcept
    {
        ownerTag = 0ULL;
        lastAcceptedLifetime = 0ULL;
        storedCount = 0U;
        readableCount = 0U;
        capacity = 0U;
        state = NCPathCoreCommandedChordStoreState::CLOSED;
        firstFault = NCPathCoreCommandedChordStoreCode::NONE;
        reserved[0U] = 0U;
        reserved[1U] = 0U;
    }
};

class NCPathCoreCommandedChordStoreV1 final
{
public:
    // Deliberately small V1 build-time budget, not a production sizing decision.
    static constexpr std::uint32_t Capacity = 32U;

    explicit NCPathCoreCommandedChordStoreV1(std::uint64_t ownerTag) noexcept;
    NCPathCoreCommandedChordStoreV1(const NCPathCoreCommandedChordStoreV1&) = delete;
    NCPathCoreCommandedChordStoreV1& operator=(const NCPathCoreCommandedChordStoreV1&) = delete;
    NCPathCoreCommandedChordStoreV1(NCPathCoreCommandedChordStoreV1&&) = delete;
    NCPathCoreCommandedChordStoreV1& operator=(NCPathCoreCommandedChordStoreV1&&) = delete;
    ~NCPathCoreCommandedChordStoreV1() = default;

    NCPathCoreCommandedChordStoreCode BeginLifetime(std::uint64_t lifetime) noexcept;
    void Invalidate() noexcept; // Explicit component operation, NOT an NC hook.

    NCPathCoreCommandedChordStoreCode Append(
        const NCPathCoreCommandedChordSegmentV1& value,
        NCPathCoreCommandedChordStoreHandleV1& output) noexcept;
    NCPathCoreCommandedChordStoreCode GetHandleAtOrdinal(
        std::uint32_t ordinal,
        NCPathCoreCommandedChordStoreHandleV1& output) const noexcept;
    NCPathCoreCommandedChordStoreCode Read(
        const NCPathCoreCommandedChordStoreHandleV1& handle,
        NCPathCoreCommandedChordSegmentV1& output) const noexcept;
    void Describe(NCPathCoreCommandedChordStoreInfoV1& output) const noexcept;

private:
    NCPathCoreCommandedChordStoreCode Fault(NCPathCoreCommandedChordStoreCode code) noexcept;
    void ClearPayload() noexcept;
    void WriteHandle(std::uint32_t ordinal,
        NCPathCoreCommandedChordStoreHandleV1& output) const noexcept;

    const std::uint64_t m_ownerTag;
    std::uint64_t m_lastAcceptedLifetime = 0ULL;
    std::array<NCPathCoreCommandedChordSegmentV1, Capacity> m_values{};
    std::uint32_t m_count = 0U;
    NCPathCoreCommandedChordStoreState m_state = NCPathCoreCommandedChordStoreState::CLOSED;
    NCPathCoreCommandedChordStoreCode m_firstFault = NCPathCoreCommandedChordStoreCode::NONE;
};

static_assert(sizeof(NCPathCoreCommandedChordStoreHandleV1) == 40U &&
    alignof(NCPathCoreCommandedChordStoreHandleV1) == 8U,
    "AZ handle layout changed; not a wire ABI.");
static_assert(sizeof(NCPathCoreCommandedChordStoreInfoV1) == 32U &&
    alignof(NCPathCoreCommandedChordStoreInfoV1) == 8U,
    "AZ fixed metadata budget changed.");
static_assert(sizeof(NCPathCoreCommandedChordStoreV1) == 5144U &&
    alignof(NCPathCoreCommandedChordStoreV1) == 8U,
    "AZ heap-owned store budget is 32 * 160 + 24 = 5144 bytes.");
static_assert(std::is_trivially_copyable<NCPathCoreCommandedChordStoreHandleV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordStoreInfoV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordStoreV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordStoreV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordStoreV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordStoreV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordStoreV1>::value,
    "AZ store ownership must remain noncopyable/nonmovable; no generic copy guard.");
