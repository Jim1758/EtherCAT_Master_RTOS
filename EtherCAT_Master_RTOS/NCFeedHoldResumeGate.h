#pragma once

#include "NCFeedHoldBoundary.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2I.3 - Program Feed Hold ACK-Gated Resume Controlled Cutover
//
// NC-0.2I.2 proved that a Feed Hold Request and a physically acknowledged stop
// are different events.  This gate is the first controlled cutover from the
// diagnostic observer to real flow control:
//
//   Cycle Start before ACK
//       -> latch Resume Request
//       -> keep NC in HOLD
//       -> keep Feedrate Override at zero
//       -> release automatically only after the existing Feed Hold Boundary
//          reports a valid ACK for the same Owner / Epoch / Request sequence
//
//   Cycle Start after ACK
//       -> resume immediately through the same gate
//
// The gate applies only to PROGRAM Feed Hold.  HOME, M00/M01, Single Block and
// all other HOLD reasons keep their existing behavior in this stage.
//
// A runtime enable flag is intentionally retained.  Disabling it returns the
// next Cycle Start to the legacy behavior without changing MotionCore or the
// Feed Hold observer.  Disabling while a request is deferred cancels the gate
// request and leaves the machine in HOLD; it never creates an automatic run.
// =============================================================================

enum class NCFeedHoldResumeGatePhase : std::uint8_t
{
    IDLE = 0,
    BYPASSED = 1,
    DEFERRED = 2,
    RELEASE_READY = 3,
    APPLIED = 4,
    CANCELLED = 5,
    BLOCKED = 6
};

enum class NCFeedHoldResumeGateDecision : std::uint8_t
{
    NONE = 0,
    LEGACY_BYPASS_DISABLED = 1,
    LEGACY_BYPASS_NOT_PROGRAM_FEED_HOLD = 2,
    DEFER_UNTIL_ACK = 3,
    APPLY_IMMEDIATE_AFTER_ACK = 4,
    RELEASE_ON_ACK = 5,
    DUPLICATE_DEFERRED_REQUEST = 6,
    DUPLICATE_RELEASE_READY_REQUEST = 7,
    BOUNDARY_FAILED = 8,
    BOUNDARY_CANCELLED = 9,
    RESUME_APPLIED = 10,
    CANCELLED = 11,
    SUPERSEDED = 12,
    ROLLBACK_DISABLED = 13
};

enum class NCFeedHoldResumeGateRequestResult : std::uint8_t
{
    BYPASS_LEGACY = 0,
    DEFERRED = 1,
    APPLY_NOW = 2,
    BLOCKED = 3
};

struct NCFeedHoldResumeGateSnapshot
{
    std::uint64_t sequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;

    NCFeedHoldResumeGatePhase phase = NCFeedHoldResumeGatePhase::IDLE;
    NCFeedHoldResumeGateDecision decision =
        NCFeedHoldResumeGateDecision::NONE;
    NCFeedHoldSource source = NCFeedHoldSource::NONE;

    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration = MOTION_OWNER_GENERATION_INVALID;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    int requestPC = -1;

    bool enabled = true;
    bool active = false;
    bool resumeRequestLatched = false;
    bool deferredUntilAcknowledge = false;
    bool acknowledgeObserved = false;
    bool releaseReady = false;
    bool resumeApplied = false;

    bool boundaryActive = false;
    bool boundaryAcknowledged = false;
    bool boundaryFailed = false;
    bool boundaryCancelled = false;

    bool blocked = false;
    bool cancelled = false;
};

struct NCFeedHoldResumeGateCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t legacyBypassDisabled = 0ULL;
    std::uint64_t legacyBypassNotProgramFeedHold = 0ULL;
    std::uint64_t deferredBeforeAcknowledge = 0ULL;
    std::uint64_t immediateAfterAcknowledge = 0ULL;
    std::uint64_t duplicateRequests = 0ULL;
    std::uint64_t releaseOnAcknowledge = 0ULL;
    std::uint64_t resumeApplied = 0ULL;
    std::uint64_t blockedBoundaryFailed = 0ULL;
    std::uint64_t blockedBoundaryCancelled = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t superseded = 0ULL;
    std::uint64_t rollbackDisabled = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCFeedHoldResumeGateSnapshot>::value,
    "NCFeedHoldResumeGateSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCFeedHoldResumeGateCounters>::value,
    "NCFeedHoldResumeGateCounters must remain trivially copyable.");

class NCFeedHoldResumeGate
{
public:
    NCFeedHoldResumeGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept;

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    NCFeedHoldResumeGateRequestResult RequestResume(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept;

    void ObserveBoundary(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept;

    bool ShouldApplyResume() const noexcept
    {
        return
            m_enabled &&
            m_snapshot.active &&
            m_snapshot.releaseReady &&
            !m_snapshot.blocked &&
            !m_snapshot.cancelled &&
            !m_snapshot.resumeApplied;
    }

    bool HasDeferredResume() const noexcept
    {
        return
            m_snapshot.active &&
            m_snapshot.deferredUntilAcknowledge &&
            !m_snapshot.resumeApplied;
    }

    void MarkResumeApplied(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept;

    void Cancel(bool superseded) noexcept;

    NCFeedHoldResumeGateSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCFeedHoldResumeGateCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    std::uint64_t AllocateSequence() noexcept;
    void BeginSnapshot(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept;
    void UpdateBoundary(
        const NCFeedHoldBoundarySnapshot& boundary) noexcept;
    void Block(
        NCFeedHoldResumeGateDecision decision) noexcept;

    bool m_enabled = true;
    NCFeedHoldResumeGateSnapshot m_snapshot{};
    NCFeedHoldResumeGateCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
