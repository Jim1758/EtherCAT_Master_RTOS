#pragma once

#include "NCBlockLifecycleLedger.h"
#include "NCProgramCache.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2I.1 - Single Block Completion Boundary Shadow
//
// This observer does NOT control NC execution.  It records where Single Block
// should stop and compares that proven boundary with the current legacy HOLD
// timing.
//
// Proven boundary:
//   Program Commit
//   + G/M callback completion (or NC-0.2H Transaction finalization)
//   + Motion Lifecycle success when motion is tracked
//
// The first release is diagnostic-only.  It never changes PC, HOLD, Motion,
// Owner, Program End, Macro flow, or the existing m_pauseAfterBlock behavior.
// =============================================================================

enum class NCSingleBlockCandidateKind : std::uint8_t
{
    NONE = 0,
    PROGRAM_CONTROL = 1,
    G_M_BLOCK = 2,
    ADDRESS_BLOCK = 3
};

enum class NCSingleBlockShadowPhase : std::uint8_t
{
    IDLE = 0,
    ARMED = 1,
    WAITING_BOUNDARY = 2,
    BOUNDARY_READY = 3,
    LEGACY_HOLD_CONFIRMED = 4,
    LEGACY_HOLD_MISMATCH = 5,
    PROGRAM_END_SUPPRESSED = 6,
    CANCELLED = 7
};

enum class NCSingleBlockShadowDecision : std::uint8_t
{
    NONE = 0,
    NOT_ELIGIBLE = 1,
    WAIT_LIFECYCLE = 2,
    WAIT_PROGRAM_COMMIT = 3,
    WAIT_TRANSACTION = 4,
    WAIT_CALLBACK = 5,
    WAIT_MOTION = 6,
    READY_FOR_HOLD = 7,
    AGREE_HOLD = 8,
    LEGACY_EARLY_HOLD = 9,
    LEGACY_HOLD_WITHOUT_ARM = 10,
    PROGRAM_END_SUPPRESSED = 11,
    MOTION_FAILED = 12,
    TRACKING_OVERFLOW = 13,
    TRANSACTION_FAILED = 14,
    RESUMED = 15,
    CANCELLED = 16
};

struct NCSingleBlockShadowArmRequest
{
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSnapshot programTarget{};
    int sourceLineNumber = 0;
    NCSingleBlockCandidateKind candidateKind =
        NCSingleBlockCandidateKind::NONE;
    bool callbackRequired = false;
    bool transactionRequired = false;
    bool legacyPausePending = false;
};

struct NCSingleBlockShadowSample
{
    bool lifecycleFound = false;
    NCBlockMotionBoundarySnapshot motionBoundary{};
    bool callbackComplete = false;
    bool transactionComplete = false;
    bool transactionFailed = false;
    bool programEndPending = false;
};

struct NCSingleBlockShadowSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;

    NCSingleBlockCandidateKind candidateKind =
        NCSingleBlockCandidateKind::NONE;
    NCSingleBlockShadowPhase phase = NCSingleBlockShadowPhase::IDLE;
    NCSingleBlockShadowDecision decision =
        NCSingleBlockShadowDecision::NONE;

    NCBlockMotionBoundaryState motionState =
        NCBlockMotionBoundaryState::NONE;
    NCBlockLifecycleState lifecycleState =
        NCBlockLifecycleState::NONE;

    int sourcePC = -1;
    int sourceLineNumber = 0;

    bool active = false;
    bool programCommitted = false;
    bool callbackRequired = false;
    bool callbackComplete = false;
    bool transactionRequired = false;
    bool transactionComplete = false;
    bool motionTracked = false;
    bool motionComplete = false;
    bool motionFailed = false;
    bool boundaryReady = false;
    bool legacyPausePending = false;
    bool legacyHoldObserved = false;
    bool programEndSuppressed = false;
};

struct NCSingleBlockShadowCounters
{
    std::uint64_t armAttempts = 0ULL;
    std::uint64_t armed = 0ULL;
    std::uint64_t notEligible = 0ULL;
    std::uint64_t evaluations = 0ULL;

    std::uint64_t waitLifecycle = 0ULL;
    std::uint64_t waitProgramCommit = 0ULL;
    std::uint64_t waitTransaction = 0ULL;
    std::uint64_t waitCallback = 0ULL;
    std::uint64_t waitMotion = 0ULL;

    std::uint64_t boundaryReady = 0ULL;
    std::uint64_t legacyHolds = 0ULL;
    std::uint64_t agreeHolds = 0ULL;
    std::uint64_t legacyEarlyHolds = 0ULL;
    std::uint64_t legacyHoldWithoutArm = 0ULL;
    std::uint64_t legacyMissingHold = 0ULL;

    std::uint64_t motionFailures = 0ULL;
    std::uint64_t trackingOverflow = 0ULL;
    std::uint64_t transactionFailures = 0ULL;
    std::uint64_t programEndSuppressed = 0ULL;

    std::uint64_t resumed = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t superseded = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCSingleBlockShadowArmRequest>::value,
    "NCSingleBlockShadowArmRequest must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCSingleBlockShadowSample>::value,
    "NCSingleBlockShadowSample must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCSingleBlockShadowSnapshot>::value,
    "NCSingleBlockShadowSnapshot must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCSingleBlockShadowCounters>::value,
    "NCSingleBlockShadowCounters must remain trivially copyable.");

class NCSingleBlockBoundaryShadow
{
public:
    NCSingleBlockBoundaryShadow() noexcept = default;

    bool Arm(const NCSingleBlockShadowArmRequest& request) noexcept;

    void NoteNotEligible(
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& target,
        int sourceLineNumber) noexcept;

    bool Evaluate(const NCSingleBlockShadowSample& sample) noexcept;

    void ObserveLegacyHold(bool causedBySingleBlock) noexcept;
    void ObserveLegacyResume() noexcept;
    void SuppressForProgramEnd() noexcept;
    void Cancel(bool superseded) noexcept;

    bool HasActiveBoundary() const noexcept
    {
        return m_snapshot.active;
    }

    NCSingleBlockShadowSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCSingleBlockShadowCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    std::uint64_t AllocateSequence() noexcept;
    void Touch() noexcept;
    void SetDecision(
        NCSingleBlockShadowPhase phase,
        NCSingleBlockShadowDecision decision) noexcept;

    NCSingleBlockShadowSnapshot m_snapshot{};
    NCSingleBlockShadowCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
