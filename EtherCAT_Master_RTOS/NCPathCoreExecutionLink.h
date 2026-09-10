#pragma once

#include "NCPathCoreCommandedChordStore.h"
#include "MotionExecutionContract.h"
#include <cstddef>

// NC-0.2L.2BO: bounded association of retained commanded segments with the
// EXISTING Motion feedback stream. One heap-owned instance, one NC thread.
// Bind only after BN's current D/proof validation and bit-exact Store readback.
// The caller supplies provenance; well-formed IDs alone do not authenticate it.
// No geometry is copied here, no second queue consumer, no Motion permission.
// COMPLETED means the existing runtime event, not measured axis trajectory.
// progress is deliberately not stored or interpreted as chord parameter u.

enum class NCPathCoreExecutionLinkState : std::uint8_t
{
    IDLE = 0U, TRACKING = 1U, SEALED = 2U, CLOSED = 3U, FAULTED = 4U
};

enum class NCPathCoreExecutionLinkFault : std::uint8_t
{
    NONE = 0U, INVALID_SCOPE = 1U, INVALID_BINDING = 2U,
    HANDLE_CONFLICT = 3U, IDENTITY_CONFLICT = 4U, SEQUENCE = 5U,
    TRANSPORT = 6U, LEDGER = 7U, TRANSITION = 8U,
    CAPACITY = 9U, LIFETIME = 10U
};

struct NCPathCoreExecutionRecordV1
{
    NCPathCoreCommandedChordStoreHandleV1 handle{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    MotionFeedbackSequence lastSequence = 0ULL;
    std::uint32_t errorCode = 0U;
    MotionFeedbackType lastType = MotionFeedbackType::NONE;
    MotionFeedbackType terminalType = MotionFeedbackType::NONE;
    MotionRejectReason rejectReason = MotionRejectReason::NONE;
    bool consumerAccepted = false;
    bool started = false;
    std::uint8_t reserved[7U]{};
    void Clear() noexcept;
};

struct NCPathCoreExecutionLinkStatusV1
{
    std::uint64_t ownerTag = 0ULL;
    std::uint64_t runToken = 0ULL;
    std::uint64_t lifetime = 0ULL;
    MotionFeedbackSequence lastSequence = 0ULL;
    std::uint32_t bound = 0U;
    std::uint32_t accepted = 0U;
    std::uint32_t started = 0U;
    std::uint32_t completed = 0U;
    std::uint32_t failed = 0U;
    std::uint32_t pending = 0U;
    NCPathCoreExecutionLinkState state = NCPathCoreExecutionLinkState::IDLE;
    NCPathCoreExecutionLinkFault fault = NCPathCoreExecutionLinkFault::NONE;
    std::uint8_t retentionReason = 0U; // BN reason at first admission seal.
    std::uint8_t endReason = 0U;       // BN reason at explicit runtime boundary.
    std::uint8_t reserved[4U]{};
};

class NCPathCoreExecutionLink final
{
public:
    NCPathCoreExecutionLink() noexcept = default;
    NCPathCoreExecutionLink(const NCPathCoreExecutionLink&) = delete;
    NCPathCoreExecutionLink& operator=(const NCPathCoreExecutionLink&) = delete;
    NCPathCoreExecutionLink(NCPathCoreExecutionLink&&) = delete;
    NCPathCoreExecutionLink& operator=(NCPathCoreExecutionLink&&) = delete;
    ~NCPathCoreExecutionLink() = default;

    // Strictly newer run, same nonzero store owner. Runs/lifetimes never wrap.
    // A new Arm clears rows, not the lifetime high-water. Stale/repeated Arm
    // returns false without erasing evidence. The caller closes/flushes first.
    bool Arm(std::uint64_t ownerTag, std::uint64_t runToken,
        MotionFeedbackSequence lastConsumedSequence) noexcept;
    // Sequential one-based handles. Exact existing binding is idempotent;
    // changed handle/identity/owner and identity reuse fault this data service.
    bool Bind(const NCPathCoreCommandedChordStoreHandleV1& handle,
        const MotionExecutionIdentity& identity,
        const MotionOwnerLease& ownerLease) noexcept;
    // Observe ALL events in original NC consumption order, including unrelated
    // ones, after Ledger classification. Exact next sequence wraps MAX -> 1.
    // A matching event requires full identity and owner, plus ledgerAccepted.
    // ACCEPTED -> COMPLETED without STARTED is legal for zero-distance G00.
    void Observe(const MotionFeedbackEvent& event, bool ledgerAccepted) noexcept;
    // False means the authoritative transport/counter checks are unhealthy.
    void CheckTransport(bool healthy) noexcept;
    // Geometry retention can end at capacity/epoch change. Stop new bindings,
    // keep collecting feedback for ALREADY bound identities until Close.
    void Seal(std::uint8_t retentionReason) noexcept;
    // Freeze evidence immediately on Reset/interruption/config/mode/end. A
    // pending record stays pending; late feedback never fabricates a terminal.
    // Returns true exactly once for each armed run, including a faulted run.
    bool Close(std::uint8_t endReason) noexcept;
    // Caller output is disjoint from the handle and this heap-owned object.
    bool Read(const NCPathCoreCommandedChordStoreHandleV1& handle,
        NCPathCoreExecutionRecordV1& output) const noexcept;
    // Diagnostic only: pending after Close is unresolved-at-boundary. This
    // summary is neither live geometry availability nor an execution permit.
    void Describe(NCPathCoreExecutionLinkStatusV1& output) const noexcept;

private:
    bool IsObserving() const noexcept;
    void Fail(NCPathCoreExecutionLinkFault reason) noexcept;
    NCPathCoreExecutionRecordV1 m_rows[32U]{};
    std::uint64_t m_ownerTag = 0ULL;
    std::uint64_t m_runToken = 0ULL;
    std::uint64_t m_lifetime = 0ULL;
    std::uint64_t m_lastLifetime = 0ULL;
    MotionFeedbackSequence m_lastSequence = 0ULL;
    std::uint32_t m_count = 0U;
    NCPathCoreExecutionLinkState m_state = NCPathCoreExecutionLinkState::IDLE;
    NCPathCoreExecutionLinkFault m_fault = NCPathCoreExecutionLinkFault::NONE;
    std::uint8_t m_retentionReason = 0U;
    std::uint8_t m_endReason = 0U;
    bool m_runOpen = false;
};

static_assert(sizeof(NCPathCoreExecutionRecordV1) == 96U &&
    alignof(NCPathCoreExecutionRecordV1) == 8U &&
    sizeof(NCPathCoreExecutionLinkStatusV1) == 64U &&
    alignof(NCPathCoreExecutionLinkStatusV1) == 8U &&
    sizeof(NCPathCoreExecutionLink) == 3128U &&
    alignof(NCPathCoreExecutionLink) == 8U,
    "BO fixed record/status/heap-owner layout changed.");
static_assert(offsetof(NCPathCoreExecutionRecordV1, handle) == 0U &&
    offsetof(NCPathCoreExecutionRecordV1, identity) == 40U &&
    offsetof(NCPathCoreExecutionRecordV1, ownerLease) == 64U &&
    offsetof(NCPathCoreExecutionRecordV1, lastSequence) == 72U &&
    offsetof(NCPathCoreExecutionRecordV1, errorCode) == 80U &&
    offsetof(NCPathCoreExecutionRecordV1, lastType) == 84U &&
    offsetof(NCPathCoreExecutionRecordV1, terminalType) == 85U &&
    offsetof(NCPathCoreExecutionRecordV1, rejectReason) == 86U &&
    offsetof(NCPathCoreExecutionRecordV1, consumerAccepted) == 87U &&
    offsetof(NCPathCoreExecutionRecordV1, started) == 88U &&
    offsetof(NCPathCoreExecutionRecordV1, reserved) == 89U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, ownerTag) == 0U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, runToken) == 8U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, lifetime) == 16U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, lastSequence) == 24U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, bound) == 32U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, accepted) == 36U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, started) == 40U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, completed) == 44U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, failed) == 48U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, pending) == 52U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, state) == 56U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, fault) == 57U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, retentionReason) == 58U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, endReason) == 59U &&
    offsetof(NCPathCoreExecutionLinkStatusV1, reserved) == 60U,
    "BO native field offsets changed; this is not a wire or SHM ABI.");
static_assert(std::is_standard_layout<NCPathCoreExecutionRecordV1>::value&&
    std::is_trivially_copyable<NCPathCoreExecutionRecordV1>::value&&
    std::is_trivially_copyable<NCPathCoreExecutionLinkStatusV1>::value&&
    std::is_standard_layout<NCPathCoreExecutionLink>::value&&
    std::is_trivially_destructible<NCPathCoreExecutionLink>::value&&
    std::is_nothrow_default_constructible<NCPathCoreExecutionLink>::value &&
    !std::is_copy_constructible<NCPathCoreExecutionLink>::value &&
    !std::is_copy_assignable<NCPathCoreExecutionLink>::value &&
    !std::is_move_constructible<NCPathCoreExecutionLink>::value &&
    !std::is_move_assignable<NCPathCoreExecutionLink>::value,
    "BO values and sole heap-owner traits changed.");
