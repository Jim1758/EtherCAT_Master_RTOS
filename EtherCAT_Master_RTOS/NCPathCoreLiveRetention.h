#pragma once

#include "NCPathCoreCommandedChordSnapshot.h"
#include "MotionExecutionContract.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2BN: live commanded-geometry retention boundary. The NC owner supplies
// trusted captured AY values and re-reads its real scope before every operation.
// This is NOT a Motion producer, consumer, completion history or control permit.
// Heap-owned, one NC owning thread, no reentry; all outputs are disjoint/alive.
// No allocation, synchronization, PDO work, logging or mutable Store escape.

struct NCPathCoreLiveRetentionScopeV1
{
    std::uint64_t runToken = 0ULL;
    std::uint64_t cacheGeneration = 0ULL;
    std::uint32_t programScope = 0U;
    std::uint32_t operationMode = 0U;
    MotionOwnerLease ownerLease{};

    void Clear() noexcept;
    bool IsValid() const noexcept;
    bool Matches(const NCPathCoreLiveRetentionScopeV1& other) const noexcept;
};
// Scope encodings match existing NC enums without including the program parser:
// MEMORY=(scope 1, mode 0, AUTO), MDI=(2,1,MDI), MANUAL_AUTO=(3,2,MANUAL_AUTO).
// run/cache and lease generation must be nonzero. Compare semantic fields only;
// MotionOwnerLease padding is not an identity or serialized representation.

enum class NCPathCoreLiveRetentionState : std::uint8_t
{
    CLOSED = 0U, ARMED = 1U, OPEN = 2U, HELD = 3U, FAULTED = 4U, EXHAUSTED = 5U
};

enum class NCPathCoreLiveRetentionReason : std::uint8_t
{
    NONE = 0U,
    START = 1U,
    HOLD = 2U,
    RESUMED = 3U,
    SCOPE_CHANGED = 4U,
    EPOCH_CHANGED = 5U,
    INVALID_SCOPE = 6U,
    CAPTURE_REJECTED = 7U,
    READBACK_REJECTED = 8U,
    ADMISSION_FAILED = 9U,
    ADMISSION_DURING_HOLD = 10U,
    LIFETIME_EXHAUSTED = 11U,
    OWNER_EXHAUSTED = 12U,
    RUN_EXHAUSTED = 13U,
    EXPLICIT_FENCE = 14U,
    HOME = 15U,
    AXIS_CONFIG = 16U,
    MACRO = 17U,
    MODE_CHANGE = 18U,
    STATE_CHANGE = 19U,
    PROGRAM_END = 20U,
    RESET = 21U,
    INTERRUPTION = 22U,
    PROGRAM_RELOAD = 23U
};

struct NCPathCoreLiveRetentionStatusV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t runToken = 0ULL;
    std::uint64_t lifetime = 0ULL;
    MotionExecutionEpoch boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    std::uint32_t storedCount = 0U;
    std::uint32_t readableCount = 0U;
    std::uint32_t admittedCount = 0U;
    std::uint32_t replayCount = 0U;
    NCPathCoreLiveRetentionState state = NCPathCoreLiveRetentionState::CLOSED;
    NCPathCoreLiveRetentionReason reason = NCPathCoreLiveRetentionReason::NONE;
    NCPathCoreCommandedChordStoreCode lastStoreCode = NCPathCoreCommandedChordStoreCode::NONE;
    NCPathCoreLiveRetentionState heldPrior = NCPathCoreLiveRetentionState::CLOSED;

    void Clear() noexcept;
};

// One process-wide instance, called ONLY on the startup/creation thread. Keep
// it alive across all manager reconstructions. Never instantiate a replacement
// namespace while old handles can survive. No addresses, atomics or wrap.
// constexpr construction permits constant initialization of that static owner.
class NCPathCoreLiveRetentionOwnerCounter final
{
public:
    explicit constexpr NCPathCoreLiveRetentionOwnerCounter(
        std::uint64_t lastIssued = 0ULL) noexcept : m_lastIssued(lastIssued) {}
    NCPathCoreLiveRetentionOwnerCounter(const NCPathCoreLiveRetentionOwnerCounter&) = delete;
    NCPathCoreLiveRetentionOwnerCounter& operator=(const NCPathCoreLiveRetentionOwnerCounter&) = delete;
    NCPathCoreLiveRetentionOwnerCounter(NCPathCoreLiveRetentionOwnerCounter&&) = delete;
    NCPathCoreLiveRetentionOwnerCounter& operator=(NCPathCoreLiveRetentionOwnerCounter&&) = delete;
    ~NCPathCoreLiveRetentionOwnerCounter() = default;
    std::uint64_t Allocate() noexcept; // Nonzero fresh tag, or 0 forever at MAX.
private:
    std::uint64_t m_lastIssued;
};

class NCPathCoreLiveRetention final
{
public:
    // initialLifetime is a caller-supplied generation lower bound, not evidence
    // of prior Store payload. Production uses 0; MAX boundary tests may seed it.
    explicit NCPathCoreLiveRetention(std::uint64_t ownerTag,
        std::uint64_t initialLifetime = 0ULL) noexcept;
    NCPathCoreLiveRetention(const NCPathCoreLiveRetention&) = delete;
    NCPathCoreLiveRetention& operator=(const NCPathCoreLiveRetention&) = delete;
    NCPathCoreLiveRetention(NCPathCoreLiveRetention&&) = delete;
    NCPathCoreLiveRetention& operator=(NCPathCoreLiveRetention&&) = delete;
    ~NCPathCoreLiveRetention() = default;

    // EXHAUSTED never revives. Older nonzero run tokens always no-op, even if
    // their other fields differ. Equal token + equal tuple no-ops in every state;
    // equal token + conflicting tuple faults. Invalid/zero new scope faults.
    // Only a strictly newer valid token clears old scope/counters/fault and arms.
    // Arm consumes NO lifetime and binds NO epoch.
    void Arm(const NCPathCoreLiveRetentionScopeV1& scope) noexcept;

    // Active scope mismatch/invalidity fences CLOSED. ARMED or held-ARMED has
    // no bound epoch and permits currentEpoch=0; OPEN/held-OPEN requires exact
    // nonzero bound epoch. Closed/faulted/exhausted returns false unchanged.
    bool CheckScope(const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch) noexcept;

    // Clear output first, then scope guard, then state. A same-scope unexpected
    // Admit during HELD faults. ARMED validates AY/head before requiring nonzero
    // epoch and allocating the next lifetime. Exact replay increments only the
    // saturating replay counter. No chain/overflow/gap automatic reopening.
    // Same-scope HELD admission and a valid ARMED head with epoch 0 return
    // NOT_OPEN but Reject with lastStoreCode=NONE: AZ was not called. A scope
    // guard refusal returns NOT_OPEN while preserving the last Store code.
    // Invalid AY/head refusals set INVALID_VALUE/MISSING_HEAD diagnostics.
    NCPathCoreCommandedChordStoreCode Admit(
        const NCPathCoreCommandedChordSegmentV1& value,
        const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch,
        NCPathCoreCommandedChordStoreHandleV1& output) noexcept;

    void Hold(const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch) noexcept;
    // Scope guard applies even when successfulGate is false. False preserves
    // a still-compatible HELD scope; true restores its saved ARMED/OPEN phase.
    void Resume(const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch, bool successfulGate) noexcept;

    // Fence clears Store payload and bound epoch, retaining token/lifetime and
    // counts. First terminal fault/fence reason + last Store status are sticky
    // until a new valid Arm. Thus CLOSED may report a preceding fault reason.
    void Fence(NCPathCoreLiveRetentionReason reason) noexcept;
    void Reject(NCPathCoreLiveRetentionReason reason,
        NCPathCoreCommandedChordStoreCode code = NCPathCoreCommandedChordStoreCode::NONE) noexcept;
    // Used when a manager's independent run allocator exhausts. Irreversible
    // for this instance; reason is replaced by this explicit exhaustion cause.
    void DisableExhausted(NCPathCoreLiveRetentionReason reason) noexcept;

    NCPathCoreCommandedChordStoreCode GetHandleAtOrdinal(
        const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch, std::uint32_t ordinal,
        NCPathCoreCommandedChordStoreHandleV1& output) noexcept;
    NCPathCoreCommandedChordStoreCode Read(
        const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch,
        const NCPathCoreCommandedChordStoreHandleV1& handle,
        NCPathCoreCommandedChordSegmentV1& output) noexcept;
    // The sole bulk exit delegates existing BD validation to the private Store.
    // Clear output/workspace, guard scope and OPEN, then Capture. FAULTED maps
    // to STORE_FAULTED; every other inactive/HELD state maps to STORE_CLOSED.
    // A bad subpath refuses without poisoning retention. Success keeps the BD
    // end segment in workspace and creates an independent mathematical value.
    // All arguments must be disjoint, alive and heap-owned as BD requires.
    NCPathCoreCommandedChordSnapshotCode CaptureSnapshot(
        const NCPathCoreLiveRetentionScopeV1& scope,
        MotionExecutionEpoch currentEpoch,
        const NCPathCoreCommandedChordSubpathV1& subpath,
        NCPathCoreCommandedChordSegmentV1& workspace,
        NCPathCoreCommandedChordSnapshotV1& output) noexcept;
    // Diagnostic only. HELD readableCount is 0 despite its retained payload.
    // A manager must freshly guard runtime context before exposing live status.
    void Describe(NCPathCoreLiveRetentionStatusV1& output) const noexcept;

private:
    bool HasTerminalReason() const noexcept;
    NCPathCoreCommandedChordStoreV1 m_store;
    NCPathCoreLiveRetentionScopeV1 m_scope{};
    std::uint64_t m_lifetime = 0ULL;
    MotionExecutionEpoch m_boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    std::uint32_t m_admittedCount = 0U;
    std::uint32_t m_replayCount = 0U;
    NCPathCoreLiveRetentionState m_state = NCPathCoreLiveRetentionState::CLOSED;
    NCPathCoreLiveRetentionState m_heldPrior = NCPathCoreLiveRetentionState::CLOSED;
    NCPathCoreLiveRetentionReason m_reason = NCPathCoreLiveRetentionReason::NONE;
    NCPathCoreCommandedChordStoreCode m_lastStoreCode = NCPathCoreCommandedChordStoreCode::NONE;
};

static_assert(sizeof(MotionOwnerLease) == 8U && alignof(MotionOwnerLease) == 4U,
    "BN requires the existing 8-byte Motion owner lease.");
static_assert(sizeof(NCPathCoreLiveRetentionScopeV1) == 32U &&
    alignof(NCPathCoreLiveRetentionScopeV1) == 8U &&
    offsetof(NCPathCoreLiveRetentionScopeV1, runToken) == 0U &&
    offsetof(NCPathCoreLiveRetentionScopeV1, cacheGeneration) == 8U &&
    offsetof(NCPathCoreLiveRetentionScopeV1, programScope) == 16U &&
    offsetof(NCPathCoreLiveRetentionScopeV1, operationMode) == 20U &&
    offsetof(NCPathCoreLiveRetentionScopeV1, ownerLease) == 24U,
    "BN scope exact native layout changed.");
static_assert(sizeof(NCPathCoreLiveRetentionStatusV1) == 48U &&
    alignof(NCPathCoreLiveRetentionStatusV1) == 8U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, ownerTag) == 0U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, runToken) == 8U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, lifetime) == 16U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, boundEpoch) == 24U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, storedCount) == 28U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, readableCount) == 32U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, admittedCount) == 36U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, replayCount) == 40U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, state) == 44U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, reason) == 45U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, lastStoreCode) == 46U &&
    offsetof(NCPathCoreLiveRetentionStatusV1, heldPrior) == 47U,
    "BN status exact native layout changed.");
static_assert(sizeof(NCPathCoreLiveRetention) == 5200U && alignof(NCPathCoreLiveRetention) == 8U,
    "BN heap-owned retention budget is 5200 bytes.");
static_assert(sizeof(NCPathCoreLiveRetentionOwnerCounter) == 8U &&
    alignof(NCPathCoreLiveRetentionOwnerCounter) == 8U,
    "BN owner namespace uses one 8-byte counter.");
static_assert(std::is_standard_layout<NCPathCoreLiveRetentionScopeV1>::value&&
    std::is_trivially_copyable<NCPathCoreLiveRetentionScopeV1>::value&&
    std::is_trivially_destructible<NCPathCoreLiveRetentionScopeV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreLiveRetentionScopeV1>::value&&
    std::is_standard_layout<NCPathCoreLiveRetentionStatusV1>::value&&
    std::is_trivially_copyable<NCPathCoreLiveRetentionStatusV1>::value&&
    std::is_trivially_destructible<NCPathCoreLiveRetentionStatusV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreLiveRetentionStatusV1>::value&&
    std::is_standard_layout<NCPathCoreLiveRetention>::value&&
    std::is_nothrow_constructible<NCPathCoreLiveRetention, std::uint64_t>::value&&
    std::is_trivially_destructible<NCPathCoreLiveRetention>::value &&
    !std::is_copy_constructible<NCPathCoreLiveRetention>::value &&
    !std::is_copy_assignable<NCPathCoreLiveRetention>::value &&
    !std::is_move_constructible<NCPathCoreLiveRetention>::value &&
    !std::is_move_assignable<NCPathCoreLiveRetention>::value,
    "BN value and sole-owner traits changed.");
static_assert(std::is_standard_layout<NCPathCoreLiveRetentionOwnerCounter>::value&&
    std::is_nothrow_default_constructible<NCPathCoreLiveRetentionOwnerCounter>::value&&
    std::is_trivially_destructible<NCPathCoreLiveRetentionOwnerCounter>::value &&
    !std::is_copy_constructible<NCPathCoreLiveRetentionOwnerCounter>::value &&
    !std::is_copy_assignable<NCPathCoreLiveRetentionOwnerCounter>::value &&
    !std::is_move_constructible<NCPathCoreLiveRetentionOwnerCounter>::value &&
    !std::is_move_assignable<NCPathCoreLiveRetentionOwnerCounter>::value,
    "BN counter namespace cannot be copied or moved.");
static_assert(sizeof(NCPathCoreLiveRetentionState) == 1U &&
    sizeof(NCPathCoreLiveRetentionReason) == 1U,
    "BN state and reason use one byte each.");
