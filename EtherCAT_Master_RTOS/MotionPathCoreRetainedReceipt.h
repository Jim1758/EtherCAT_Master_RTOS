#pragma once

#include "MotionExecutionContract.h"
#include "NCPathCoreRetainedPath.h"
#include "MotionFeedArcReceipt.h"
#include <array>
#include <cstdint>

enum class MotionPathCoreRetainedCode : std::uint8_t
{
    NONE = 0, COMMITTED = 1, INVALID_INPUT = 2, NOT_READY = 3,
    GEOMETRY_REJECTED = 4, PRODUCER_REJECTED = 5, STALE_AFTER_ACCEPT = 6,
    CAPTURE_MISMATCH = 7, START_MISMATCH = 8
};
struct MotionPathCoreRetainedReceipt
{
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint64_t translationGeneration = 0ULL;
    std::uint32_t validAxisMask = 0U;
    MotionPathCoreRetainedCode code = MotionPathCoreRetainedCode::NONE;
    bool commandAccepted = false, tailCommitted = false, captureBound = false;
    bool travelLimitRejected = false, valid = false;
    void Clear() noexcept
    {
        identity = MotionExecutionIdentity{};
        ownerLease = MotionOwnerLease{};
        translationGeneration = 0ULL;
        validAxisMask = 0U;
        code = MotionPathCoreRetainedCode::NONE;
        commandAccepted = tailCommitted = captureBound = travelLimitRejected = valid = false;
    }
};
// Caller-owned scratch. The source geometry remains immutable and borrowed
// synchronously; no saved identity can authorize this fresh NC submission.
struct MotionPathCoreRetainedWorkspace
{
    MotionPathCoreRetainedReceipt receipt{};
    std::array<double, 8U> stagedPulse{}, stagedMCS{}, pulsePerMM{};
    double accTime = 0.0, decTime = 0.0, velocityPPS = 0.0;
};
