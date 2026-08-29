#pragma once

#include "NCSingleBlockBoundary.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2I.4 - Single Block Completion-Gated HOLD Controlled Cutover
//
// NC-0.2I.1 proved the correct Single Block completion boundary in shadow mode.
// This gate is the controlled cutover that allows the proven boundary to create
// the real NC HOLD:
//
//   Program Commit
//   + G/M callback or NC-0.2H transaction completion
//   + Motion Lifecycle terminal success
//       -> controlled Single Block HOLD
//
// Empty/comment/label/skipped lines do not consume a Cycle Start. M02/M30 are
// suppressed and continue through the NC-0.2G Program End gate. M00 and an
// enabled M01 keep their explicit legacy stop semantics so one source creates
// exactly one HOLD.
//
// A runtime enable flag is retained. Disabling the gate returns subsequent
// Single Block execution to the previous legacy m_pauseAfterBlock path. It
// never creates an automatic RUN transition.
// =============================================================================

enum class NCSingleBlockHoldGatePhase : std::uint8_t
{
    IDLE = 0,
    BYPASSED = 1,
    ARMED = 2,
    WAITING_BOUNDARY = 3,
    HOLD_READY = 4,
    HOLD_APPLIED = 5,
    RESUMED = 6,
    PROGRAM_END_SUPPRESSED = 7,
    BLOCKED = 8,
    CANCELLED = 9
};

enum class NCSingleBlockHoldGateDecision : std::uint8_t
{
    NONE = 0,
    LEGACY_BYPASS_DISABLED = 1,
    LEGACY_BYPASS_EXPLICIT_STOP = 2,
    CONTROL_ARMED = 3,
    WAIT_BOUNDARY = 4,
    READY_TO_HOLD = 5,
    HOLD_APPLIED = 6,
    RESUME_APPLIED = 7,
    PROGRAM_END_SUPPRESSED = 8,
    BOUNDARY_MOTION_FAILED = 9,
    BOUNDARY_TRANSACTION_FAILED = 10,
    BOUNDARY_TRACKING_OVERFLOW = 11,
    BOUNDARY_CANCELLED = 12,
    CANCELLED = 13,
    SUPERSEDED = 14,
    ROLLBACK_DISABLED = 15
};

enum class NCSingleBlockHoldGateRequestResult : std::uint8_t
{
    BYPASS_LEGACY = 0,
    CONTROLLED = 1,
    BLOCKED = 2
};

struct NCSingleBlockHoldGateSnapshot
{
    std::uint64_t sequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;

    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    NCSingleBlockCandidateKind candidateKind =
        NCSingleBlockCandidateKind::NONE;

    NCSingleBlockHoldGatePhase phase =
        NCSingleBlockHoldGatePhase::IDLE;
    NCSingleBlockHoldGateDecision decision =
        NCSingleBlockHoldGateDecision::NONE;

    int sourcePC = -1;
    int sourceLineNumber = 0;

    bool enabled = true;
    bool active = false;
    bool boundaryMatched = false;
    bool boundaryReady = false;
    bool holdReady = false;
    bool holdApplied = false;
    bool resumeApplied = false;
    bool explicitStopBypass = false;
    bool programEndSuppressed = false;
    bool blocked = false;
    bool cancelled = false;
};

struct NCSingleBlockHoldGateCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t controlledArms = 0ULL;
    std::uint64_t legacyBypassDisabled = 0ULL;
    std::uint64_t legacyBypassExplicitStop = 0ULL;
    std::uint64_t waitBoundarySamples = 0ULL;
    std::uint64_t holdReady = 0ULL;
    std::uint64_t holdApplied = 0ULL;
    std::uint64_t resumeApplied = 0ULL;
    std::uint64_t programEndSuppressed = 0ULL;
    std::uint64_t blockedMotionFailure = 0ULL;
    std::uint64_t blockedTransactionFailure = 0ULL;
    std::uint64_t blockedTrackingOverflow = 0ULL;
    std::uint64_t blockedCancelled = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t superseded = 0ULL;
    std::uint64_t rollbackDisabled = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCSingleBlockHoldGateSnapshot>::value,
    "NCSingleBlockHoldGateSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCSingleBlockHoldGateCounters>::value,
    "NCSingleBlockHoldGateCounters must remain trivially copyable.");

class NCSingleBlockHoldGate
{
public:
    NCSingleBlockHoldGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept;

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    NCSingleBlockHoldGateRequestResult RequestControl(
        const NCSingleBlockShadowSnapshot& boundary,
        bool explicitStopBypass) noexcept;

    void ObserveBoundary(
        const NCSingleBlockShadowSnapshot& boundary) noexcept;

    bool ShouldApplyHold() const noexcept
    {
        return
            m_enabled &&
            m_snapshot.active &&
            m_snapshot.holdReady &&
            !m_snapshot.holdApplied &&
            !m_snapshot.blocked &&
            !m_snapshot.cancelled &&
            !m_snapshot.programEndSuppressed;
    }

    bool HasPendingControl() const noexcept
    {
        return
            m_snapshot.active &&
            !m_snapshot.holdApplied &&
            !m_snapshot.resumeApplied &&
            !m_snapshot.blocked &&
            !m_snapshot.cancelled &&
            !m_snapshot.programEndSuppressed;
    }

    bool IsHoldApplied() const noexcept
    {
        return
            m_snapshot.holdApplied &&
            !m_snapshot.resumeApplied &&
            !m_snapshot.cancelled;
    }

    void MarkHoldApplied(
        const NCSingleBlockShadowSnapshot& boundary) noexcept;
    void MarkResumeApplied() noexcept;
    void Cancel(bool superseded) noexcept;

    NCSingleBlockHoldGateSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCSingleBlockHoldGateCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    std::uint64_t AllocateSequence() noexcept;
    void BeginSnapshot(
        const NCSingleBlockShadowSnapshot& boundary) noexcept;
    void UpdateBoundary(
        const NCSingleBlockShadowSnapshot& boundary) noexcept;
    void Block(
        NCSingleBlockHoldGateDecision decision) noexcept;

    bool m_enabled = true;
    NCSingleBlockHoldGateSnapshot m_snapshot{};
    NCSingleBlockHoldGateCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
