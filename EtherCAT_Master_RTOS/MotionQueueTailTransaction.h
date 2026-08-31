#pragma once

#include "MotionExecutionContract.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.6.2 - Ordinary G00 Transactional Queue-Tail Endpoint
//
// A G00 producer first calculates a complete endpoint candidate.  The candidate
// becomes authoritative only after the fixed SPSC ingress has accepted the
// matching MotionCommand.  Rejected commands must not publish a new NC
// commanded endpoint, Motion pulse tail, or G00 override.
// =============================================================================

using MotionQueueTailTransactionSequence = std::uint64_t;

constexpr MotionQueueTailTransactionSequence
MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID = 0ULL;
constexpr std::uint64_t MOTION_QUEUE_TAIL_FINGERPRINT_SEED =
1469598103934665603ULL;

struct MotionQueueTailCommitReceipt
{
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    MotionQueueTailTransactionSequence transactionSequence =
        MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID;
    std::uint32_t axisMask = 0U;
    std::uint64_t beforeFingerprint =
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED;
    std::uint64_t committedFingerprint =
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED;

    bool attempted = false;
    bool commandAccepted = false;
    bool commandedMCSCommitted = false;
    bool lastQueuedPulseCommitted = false;
    bool rapidOverrideCommitted = false;
    bool preservedOnReject = false;
    bool endpointExact = false;
    bool captureBound = false;
    bool accountingValid = true;

    bool IsCommitted() const noexcept
    {
        return
            attempted &&
            commandAccepted &&
            identity.IsAssigned() &&
            ownerLease.IsValid() &&
            transactionSequence !=
            MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID &&
            axisMask != 0U &&
            commandedMCSCommitted &&
            lastQueuedPulseCommitted &&
            rapidOverrideCommitted &&
            !preservedOnReject &&
            endpointExact &&
            accountingValid;
    }

    bool IsRejectedAndPreserved() const noexcept
    {
        return
            attempted &&
            !commandAccepted &&
            !commandedMCSCommitted &&
            !lastQueuedPulseCommitted &&
            !rapidOverrideCommitted &&
            preservedOnReject &&
            !endpointExact &&
            beforeFingerprint == committedFingerprint &&
            accountingValid;
    }
};

struct MotionQueueTailTransactionSnapshot
{
    std::uint64_t writeSequence = 0ULL;
    std::uint64_t attempts = 0ULL;
    std::uint64_t commandAccepted = 0ULL;
    std::uint64_t commandRejected = 0ULL;
    std::uint64_t committed = 0ULL;
    std::uint64_t rejectPreserved = 0ULL;
    std::uint64_t commandedMCSCommitted = 0ULL;
    std::uint64_t lastQueuedPulseCommitted = 0ULL;
    std::uint64_t rapidOverrideCommitted = 0ULL;
    std::uint64_t endpointExact = 0ULL;
    std::uint64_t captureBound = 0ULL;
    std::uint64_t invalidInputs = 0ULL;
    std::uint64_t mismatches = 0ULL;

    MotionQueueTailTransactionSequence lastTransactionSequence =
        MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID;
    MotionExecutionEpoch lastExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionSegmentId lastSegmentId = MOTION_SEGMENT_ID_INVALID;
    std::uint32_t lastAxisMask = 0U;
    std::uint64_t lastCommittedFingerprint =
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED;

    bool authoritative = true;
    bool shadowOnly = false;
    bool cutoverAttempted = false;
    bool runtimeInfluence = false;
    bool ready = false;
    bool snapshotCoherent = true;
    bool accountingValid = true;
};

static_assert(
    std::is_trivially_copyable<MotionQueueTailCommitReceipt>::value,
    "K.6.2 queue-tail receipt must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionQueueTailTransactionSnapshot>::value,
    "K.6.2 queue-tail diagnostic snapshot must remain trivially copyable.");
