#pragma once

#include "MotionCore.h"
#include "NCLifecycleInterruptionBoundary.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2J.5 - Reset Rebase-Acknowledged Release Gate
//
// Reset may release the SAFETY owner and return NC to READY only after the
// matching NC-0.2J lifecycle interruption boundary proves all transport,
// lifecycle, callback and physical-motion evidence quiescent for its required
// stable sample count and the RT-owned Reset rebase transaction publishes a
// coherent, matching post-verify ACK.  This gate is fail-closed: mismatched
// evidence, a failed/superseded rebase, a superseded Epoch, a lost SAFETY lease
// or a failed owner release leaves NC in RESET_STATE.
//
// The gate is fixed-size, trivially copyable and allocation-free.  It is
// evaluated by the existing 10 ms NC supervisory task and does not change SHM.
// =============================================================================

enum class NCResetReleaseGatePhase : std::uint8_t
{
    IDLE = 0,
    ARMED = 1,
    WAITING_QUIESCENCE = 2,
    RELEASE_READY = 3,
    RELEASED = 4,
    BLOCKED = 5
};

enum class NCResetReleaseGateDecision : std::uint8_t
{
    NONE = 0,
    CONTROL_ARMED = 1,
    WAIT_QUIESCENCE_PROOF = 2,
    READY_TO_RELEASE = 3,
    RELEASE_APPLIED = 4,
    INVALID_BOUNDARY = 5,
    BOUNDARY_SEQUENCE_MISMATCH = 6,
    BOUNDARY_CAUSE_MISMATCH = 7,
    BOUNDARY_EVIDENCE_GAP = 8,
    BOUNDARY_SUPERSEDED = 9,
    BOUNDARY_INCOMPLETE = 10,
    EPOCH_MISMATCH = 11,
    SAFETY_LEASE_INVALID = 12,
    SAFETY_LEASE_LOST = 13,
    OWNER_RELEASE_FAILED = 14,
    POST_INTERRUPTION_DISPATCH = 15,
    WAIT_REBASE_ACK = 16,
    ACK_MISMATCH = 17,
    REBASE_FAILED = 18
};

struct NCResetReleaseGateSnapshot
{
    std::uint64_t sequence = 0ULL;
    std::uint64_t boundarySequence = 0ULL;

    MotionNCSettleRequestSequence expectedResetRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    MotionNCSettleRequestSequence ackRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    MotionExecutionEpoch expectedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch boundaryPublishedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch boundaryCurrentExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;

    MotionOwner safetyOwner = MotionOwner::NONE;
    MotionOwnerGeneration safetyOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner boundaryCurrentOwner = MotionOwner::NONE;
    MotionOwnerGeneration boundaryCurrentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    MotionExecutionEpoch ackExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner ackOwner = MotionOwner::NONE;
    MotionOwnerGeneration ackOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::uint64_t ackRequestedAxisMask = 0ULL;
    std::uint64_t ackAppliedAxisMask = 0ULL;
    std::uint32_t expectedAckAxisMask = 0U;

    MotionNCResetRebasePhase ackPhase =
        MotionNCResetRebasePhase::IDLE;

    NCLifecycleInterruptionPhase boundaryPhase =
        NCLifecycleInterruptionPhase::IDLE;
    NCLifecycleInterruptionDecision boundaryDecision =
        NCLifecycleInterruptionDecision::NONE;
    NCResetReleaseGatePhase phase = NCResetReleaseGatePhase::IDLE;
    NCResetReleaseGateDecision decision =
        NCResetReleaseGateDecision::NONE;

    std::uint32_t stableSamples = 0U;
    std::uint32_t requiredStableSamples = 2U;

    bool active = false;
    bool boundaryMatched = false;
    bool publishedEpochMatched = false;
    bool currentEpochMatched = false;
    bool safetyLeaseValid = false;
    bool safetyLeaseCurrent = false;
    bool boundaryOwnerMatched = false;
    bool groupStandstill = false;
    bool ackObserved = false;
    bool ackSequenceMatched = false;
    bool ackEpochMatched = false;
    bool ackOwnerMatched = false;
    bool ackAxisMaskLatched = false;
    bool ackRequestedAxisMaskMatched = false;
    bool ackAppliedAxisMaskMatched = false;
    bool ackAxisMaskChanged = false;
    bool ackAxisMaskMatched = false;
    bool ackRequestAccepted = false;
    bool ackRebaseApplied = false;
    bool ackPostVerifyPassed = false;
    bool ackAcknowledged = false;
    bool ackPhaseAcknowledged = false;
    bool ackBlocked = false;
    bool ackSuperseded = false;
    bool rebaseAckMatched = false;
    bool quiescenceProved = false;
    bool releaseReady = false;
    bool releaseAttempted = false;
    bool releaseApplied = false;
    bool blocked = false;
};

struct NCResetReleaseGateCounters
{
    std::uint64_t armAttempts = 0ULL;
    std::uint64_t armed = 0ULL;
    std::uint64_t supersededArms = 0ULL;
    std::uint64_t evaluations = 0ULL;
    std::uint64_t waitQuiescence = 0ULL;
    std::uint64_t waitRebaseAck = 0ULL;
    std::uint64_t releaseReady = 0ULL;
    std::uint64_t releaseAttempts = 0ULL;
    std::uint64_t released = 0ULL;

    std::uint64_t blockedInvalidBoundary = 0ULL;
    std::uint64_t blockedBoundarySequence = 0ULL;
    std::uint64_t blockedBoundaryCause = 0ULL;
    std::uint64_t blockedEvidenceGap = 0ULL;
    std::uint64_t blockedSuperseded = 0ULL;
    std::uint64_t blockedIncomplete = 0ULL;
    std::uint64_t blockedEpochMismatch = 0ULL;
    std::uint64_t blockedSafetyLease = 0ULL;
    std::uint64_t blockedOwnerRelease = 0ULL;
    std::uint64_t blockedPostInterruptionDispatch = 0ULL;
    std::uint64_t ackMismatch = 0ULL;
    std::uint64_t rebaseFailed = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCResetReleaseGateSnapshot>::value,
    "NCResetReleaseGateSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCResetReleaseGateCounters>::value,
    "NCResetReleaseGateCounters must remain trivially copyable.");

class NCResetReleaseGate
{
public:
    NCResetReleaseGate() noexcept = default;

    bool Arm(
        const NCLifecycleInterruptionSnapshot& boundary,
        MotionExecutionEpoch expectedExecutionEpoch,
        const MotionOwnerLease& safetyLease,
        MotionNCSettleRequestSequence expectedResetRequestSequence,
        bool safetyLeaseCurrent) noexcept;

    void ObserveBoundary(
        const NCLifecycleInterruptionSnapshot& boundary,
        const MotionNCResetRebaseAck& resetRebaseAck,
        bool safetyLeaseCurrent) noexcept;

    bool ShouldReleaseSafetyOwner() const noexcept
    {
        return
            m_snapshot.active &&
            m_snapshot.releaseReady &&
            m_snapshot.quiescenceProved &&
            m_snapshot.safetyLeaseCurrent &&
            m_snapshot.boundaryOwnerMatched &&
            m_snapshot.rebaseAckMatched &&
            !m_snapshot.releaseAttempted &&
            !m_snapshot.releaseApplied &&
            !m_snapshot.blocked;
    }

    void MarkReleaseResult(
        const NCLifecycleInterruptionSnapshot& boundary,
        const MotionNCResetRebaseAck& resetRebaseAck,
        bool safetyLeaseCurrent,
        bool releaseSucceeded) noexcept;

    NCResetReleaseGateSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCResetReleaseGateCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    enum class ResetRebaseAckEvaluation : std::uint8_t
    {
        WAITING = 0,
        MATCHED = 1,
        MISMATCH = 2,
        FAILED = 3
    };

    std::uint64_t AllocateSequence() noexcept;

    void UpdateBoundary(
        const NCLifecycleInterruptionSnapshot& boundary,
        bool safetyLeaseCurrent) noexcept;

    void UpdateResetRebaseAck(
        const MotionNCResetRebaseAck& resetRebaseAck) noexcept;

    ResetRebaseAckEvaluation EvaluateResetRebaseAck() const noexcept;

    void Block(NCResetReleaseGateDecision decision) noexcept;

    bool IsMatchingQuiescenceProof(
        const NCLifecycleInterruptionSnapshot& boundary) const noexcept;

    NCResetReleaseGateSnapshot m_snapshot{};
    NCResetReleaseGateCounters m_counters{};
    std::uint64_t m_nextSequence = 1ULL;
};
