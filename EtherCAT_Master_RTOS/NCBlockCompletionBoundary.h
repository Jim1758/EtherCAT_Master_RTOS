#pragma once

#include "NCBlockLifecycleLedger.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2F - Motion Completion Dual-Key Guard
//
// A tracked Motion Block may release its NC Wait Callback only when BOTH keys
// are true:
//
//   1. Legacy callback says READY (queue / standstill / handler condition)
//   2. The bound Block Lifecycle Ledger says SUCCEEDED
//
// Program-only, M-code and other non-motion waits continue to use the legacy
// callback unchanged. Missing / failed / overflowed tracked Motion boundaries
// fail closed until RESET / mode change supersedes the binding. The guard never
// changes Motion commands, AxisContext, Program Epoch or RESET behavior.
// =============================================================================

enum class NCBlockWaitKind : std::uint8_t
{
    NONE = 0,
    MOTION_HANDLER = 1,
    MOTION_QUEUE_DRAIN = 2,
    AUXILIARY_CALLBACK = 3,
    PROGRAM_FLOW_DRAIN = 4
};

enum class NCBlockCompletionComparison : std::uint8_t
{
    NONE = 0,
    AGREE_WAITING = 1,
    AGREE_READY = 2,
    LEDGER_READY_LEGACY_WAITING = 3,
    LEGACY_READY_LEDGER_PENDING = 4,
    LEDGER_FAILED_LEGACY_WAITING = 5,
    LEGACY_READY_LEDGER_FAILED = 6,
    NOT_MOTION_TRACKED = 7,
    TRACKING_OVERFLOW = 8,
    MISSING_LIFECYCLE = 9
};

enum class NCBlockCompletionGateDecision : std::uint8_t
{
    NONE = 0,
    LEGACY_BYPASS_WAIT = 1,
    LEGACY_BYPASS_READY = 2,
    WAIT_BOTH = 3,
    WAIT_LEGACY = 4,
    BLOCK_LEDGER_PENDING = 5,
    RELEASE_DUAL_KEY = 6,
    BLOCK_LEDGER_FAILED = 7,
    BLOCK_TRACKING_OVERFLOW = 8,
    BLOCK_MISSING_LIFECYCLE = 9,
    BLOCK_NOT_TRACKED = 10
};

struct NCBlockCompletionBoundarySnapshot
{
    std::uint64_t sequence = 0ULL;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockWaitKind waitKind = NCBlockWaitKind::NONE;
    NCBlockCompletionComparison comparison =
        NCBlockCompletionComparison::NONE;
    NCBlockCompletionGateDecision gateDecision =
        NCBlockCompletionGateDecision::NONE;
    NCBlockMotionBoundaryState ledgerBoundary =
        NCBlockMotionBoundaryState::NONE;
    NCBlockLifecycleState lifecycleState =
        NCBlockLifecycleState::NONE;

    std::uint16_t motionSegmentCount = 0U;
    std::uint16_t motionCompletedCount = 0U;
    std::uint16_t motionTerminalCount = 0U;
    std::uint16_t motionFailedCount = 0U;

    bool bound = false;
    bool guardEligible = false;
    bool guardApplied = false;
    bool legacyReady = false;
    bool effectiveReady = false;
    bool releaseObserved = false;
    bool failClosed = false;

    bool IsValid() const noexcept
    {
        return
            sequence != 0ULL &&
            dispatchId != NC_BLOCK_DISPATCH_ID_INVALID;
    }
};

struct NCBlockCompletionBoundaryCounters
{
    // Stage NC-0.2E shadow-comparison counters.
    std::uint64_t bindings = 0ULL;
    std::uint64_t motionBindings = 0ULL;
    std::uint64_t auxiliaryBindings = 0ULL;
    std::uint64_t programFlowBindings = 0ULL;
    std::uint64_t observations = 0ULL;
    std::uint64_t agreeWaitingSamples = 0ULL;
    std::uint64_t releaseChecks = 0ULL;
    std::uint64_t agreeRelease = 0ULL;
    std::uint64_t ledgerReadyBeforeLegacy = 0ULL;
    std::uint64_t legacyEarlyRelease = 0ULL;
    std::uint64_t ledgerFailureObserved = 0ULL;
    std::uint64_t releaseOnLedgerFailure = 0ULL;
    std::uint64_t trackingOverflow = 0ULL;
    std::uint64_t missingLifecycle = 0ULL;
    std::uint64_t nonMotionWait = 0ULL;
    std::uint64_t supersededBindings = 0ULL;

    // Stage NC-0.2F active dual-key guard counters.
    std::uint64_t guardEvaluations = 0ULL;
    std::uint64_t guardEligibleSamples = 0ULL;
    std::uint64_t guardBypassSamples = 0ULL;
    std::uint64_t guardWaitSamples = 0ULL;
    std::uint64_t dualKeyRelease = 0ULL;
    std::uint64_t blockedLegacyEarly = 0ULL;
    std::uint64_t blockedLedgerFailure = 0ULL;
    std::uint64_t blockedTrackingOverflow = 0ULL;
    std::uint64_t blockedMissingLifecycle = 0ULL;
    std::uint64_t blockedNotTracked = 0ULL;
    std::uint64_t failClosedBindings = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCBlockCompletionBoundarySnapshot>::value,
    "NCBlockCompletionBoundarySnapshot must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCBlockCompletionBoundaryCounters>::value,
    "NCBlockCompletionBoundaryCounters must remain trivially copyable.");

class NCBlockCompletionBoundaryObserver
{
public:
    void Bind(
        NCBlockDispatchId dispatchId,
        NCBlockWaitKind waitKind) noexcept;

    // Returns the effective release decision. For tracked Motion waits this is
    // a dual-key result; for all other waits it is the unmodified legacy value.
    bool ObserveAndGate(
        bool hasLifecycle,
        const NCBlockMotionBoundarySnapshot& boundary,
        bool legacyReady) noexcept;

    void ClearBinding(bool superseded) noexcept;

    bool HasActiveBinding() const noexcept
    {
        return m_activeDispatchId != NC_BLOCK_DISPATCH_ID_INVALID;
    }

    NCBlockDispatchId GetActiveDispatchId() const noexcept
    {
        return m_activeDispatchId;
    }

    NCBlockWaitKind GetActiveWaitKind() const noexcept
    {
        return m_activeWaitKind;
    }

    NCBlockCompletionBoundarySnapshot GetLastSnapshot() const noexcept
    {
        return m_lastSnapshot;
    }

    NCBlockCompletionBoundaryCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    std::uint64_t AllocateSequence() noexcept;
    bool IsMotionGuardEligible() const noexcept;
    void RecordFailClosedOnce() noexcept;
    void ResetBindingFlags() noexcept;

    void UpdateLastSnapshot(
        bool hasLifecycle,
        const NCBlockMotionBoundarySnapshot& boundary,
        bool legacyReady,
        bool effectiveReady,
        bool guardEligible,
        bool failClosed,
        NCBlockCompletionComparison comparison,
        NCBlockCompletionGateDecision gateDecision) noexcept;

    NCBlockDispatchId m_activeDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockWaitKind m_activeWaitKind = NCBlockWaitKind::NONE;

    bool m_ledgerReadyBeforeLegacyRecorded = false;
    bool m_ledgerFailureRecorded = false;
    bool m_trackingOverflowRecorded = false;
    bool m_missingLifecycleRecorded = false;
    bool m_nonMotionRecorded = false;

    bool m_guardPendingBlockRecorded = false;
    bool m_guardFailureBlockRecorded = false;
    bool m_guardOverflowBlockRecorded = false;
    bool m_guardMissingBlockRecorded = false;
    bool m_guardNotTrackedBlockRecorded = false;
    bool m_failClosedRecorded = false;

    std::uint64_t m_nextSequence = 1ULL;
    NCBlockCompletionBoundarySnapshot m_lastSnapshot{};
    NCBlockCompletionBoundaryCounters m_counters{};
};
