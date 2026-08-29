#pragma once

#include "MotionCore.h"
#include "NCBlockLifecycleLedger.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2I.2 - Feed Hold Request / Acknowledge Boundary Shadow
//
// Feed Hold differs from Single Block:
//
//   Single Block
//       current block becomes complete -> enter HOLD
//
//   Feed Hold
//       request may arrive while a block is still active
//       -> controlled deceleration
//       -> command velocity reaches zero
//       -> actual velocity reaches zero
//       -> stable confirmation
//       -> Hold Acknowledged
//
// NC-0.2J.5 controlled cutover: PROGRAM acknowledgement now requires the
// matching 250 us RT settle request and continuous proof.  NCState still
// changes to HOLD immediately, while Cycle Start remains deferred until this
// boundary acknowledges.  HOME continues to use HomingManager::PAUSED and is
// deliberately independent of the NC settle request.
// =============================================================================

enum class NCFeedHoldSource : std::uint8_t
{
    NONE = 0,
    PROGRAM = 1,
    HOME = 2
};

enum class NCFeedHoldShadowPhase : std::uint8_t
{
    IDLE = 0,
    REQUESTED = 1,
    DECELERATING = 2,
    STOPPED_UNSTABLE = 3,
    ACKNOWLEDGED = 4,
    RESUME_REQUESTED = 5,
    RESUMED = 6,
    CANCELLED = 7,
    FAILED = 8
};

enum class NCFeedHoldShadowDecision : std::uint8_t
{
    NONE = 0,
    REQUEST_LATCHED = 1,
    WAIT_OVERRIDE_ZERO = 2,
    WAIT_COMMAND_STOP = 3,
    WAIT_ACTUAL_STOP = 4,
    WAIT_HOME_PAUSED = 5,
    WAIT_STABLE = 6,
    ACK_READY = 7,
    LEGACY_HOLD_EARLY = 8,
    LEGACY_HOLD_AGREE = 9,
    RESUME_BEFORE_ACK = 10,
    RESUME_AFTER_ACK = 11,
    RESUMED = 12,
    OWNER_CHANGED = 13,
    EPOCH_CHANGED = 14,
    MOTION_FAULT = 15,
    ACK_LOST = 16,
    CANCELLED = 17,
    SUPERSEDED = 18
};

struct NCFeedHoldBoundarySample
{
    MotionExecutionEpoch executionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease ownerLease{};
    MotionFeedHoldStopSnapshot motion{};

    // NC-0.2J.5: PROGRAM Feed Hold may only consume the RT proof created for
    // this exact request.  HOME keeps the legacy HomingManager PAUSED proof
    // and therefore leaves this sequence invalid.
    MotionNCSettleRequestSequence expectedSettleRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    int activePC = -1;

    bool legacyHoldState = false;
    bool homeActive = false;
    bool homeHoldDecelerating = false;
    bool homePaused = false;
    bool homeResumeRequested = false;
};

struct NCFeedHoldBoundarySnapshot
{
    std::uint64_t sequence = 0ULL;
    NCFeedHoldSource source = NCFeedHoldSource::NONE;
    NCFeedHoldShadowPhase phase = NCFeedHoldShadowPhase::IDLE;
    NCFeedHoldShadowDecision decision = NCFeedHoldShadowDecision::NONE;

    MotionExecutionEpoch requestExecutionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch currentExecutionEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner requestOwner = MotionOwner::NONE;
    MotionOwnerGeneration requestOwnerGeneration = MOTION_OWNER_GENERATION_INVALID;
    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration = MOTION_OWNER_GENERATION_INVALID;

    MotionNCSettleRequestSequence expectedSettleRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    int requestPC = -1;

    MotionFeedHoldStopSnapshot motion{};

    std::uint8_t stableSamples = 0U;
    std::uint8_t requiredStableSamples = 2U;

    bool active = false;
    bool requestLatched = false;
    bool legacyHoldEntered = false;
    bool legacyHoldWasEarly = false;
    bool legacyHoldAgreed = false;
    bool ownerLeaseValid = false;
    bool executionEpochValid = false;
    bool homeActive = false;
    bool homeHoldDecelerating = false;
    bool homePaused = false;
    bool homeResumeRequested = false;
    bool stopCandidate = false;
    bool acknowledgeReady = false;
    bool acknowledged = false;
    bool acknowledgeLost = false;
    bool resumeRequested = false;
    bool resumeBeforeAcknowledge = false;
    bool resumeAfterAcknowledge = false;
    bool resumeApplied = false;
    bool cancelled = false;
    bool failed = false;
};

struct NCFeedHoldBoundaryCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t requestsLatched = 0ULL;
    std::uint64_t programRequests = 0ULL;
    std::uint64_t homeRequests = 0ULL;

    std::uint64_t evaluations = 0ULL;
    std::uint64_t waitOverrideZero = 0ULL;
    std::uint64_t waitCommandStop = 0ULL;
    std::uint64_t waitActualStop = 0ULL;
    std::uint64_t waitHomePaused = 0ULL;
    std::uint64_t waitStable = 0ULL;

    std::uint64_t acknowledged = 0ULL;
    std::uint64_t acknowledgeLost = 0ULL;

    std::uint64_t legacyHolds = 0ULL;
    std::uint64_t legacyEarlyHolds = 0ULL;
    std::uint64_t legacyAgreeHolds = 0ULL;

    std::uint64_t resumeRequests = 0ULL;
    std::uint64_t resumeBeforeAcknowledge = 0ULL;
    std::uint64_t resumeAfterAcknowledge = 0ULL;
    std::uint64_t resumed = 0ULL;

    std::uint64_t ownerChanged = 0ULL;
    std::uint64_t executionEpochChanged = 0ULL;
    std::uint64_t motionFault = 0ULL;

    std::uint64_t cancelled = 0ULL;
    std::uint64_t superseded = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCFeedHoldBoundarySample>::value,
    "NCFeedHoldBoundarySample must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCFeedHoldBoundarySnapshot>::value,
    "NCFeedHoldBoundarySnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCFeedHoldBoundaryCounters>::value,
    "NCFeedHoldBoundaryCounters must remain trivially copyable.");

class NCFeedHoldBoundaryShadowObserver
{
public:
    NCFeedHoldBoundaryShadowObserver() noexcept = default;

    void BeginRequest(
        NCFeedHoldSource source,
        const NCFeedHoldBoundarySample& sample) noexcept;

    void Observe(const NCFeedHoldBoundarySample& sample) noexcept;

    void ObserveLegacyHoldEntered(
        const NCFeedHoldBoundarySample& sample) noexcept;

    void ObserveResumeRequested(
        const NCFeedHoldBoundarySample& sample) noexcept;

    void ObserveResumeApplied(
        const NCFeedHoldBoundarySample& sample) noexcept;

    void Cancel(bool superseded) noexcept;

    bool IsActive() const noexcept
    {
        return m_snapshot.active;
    }

    NCFeedHoldBoundarySnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCFeedHoldBoundaryCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    std::uint64_t AllocateSequence() noexcept;
    void UpdateSample(const NCFeedHoldBoundarySample& sample) noexcept;
    bool ValidateIdentity() noexcept;
    void Fail(NCFeedHoldShadowDecision decision) noexcept;

    NCFeedHoldBoundarySnapshot m_snapshot{};
    NCFeedHoldBoundaryCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
