#pragma once

#include "MotionExecutionContract.h"
#include "NCPathCoreFeedLine.h"

#include <array>
#include <cstdint>
#include <vector>

// BX: producer-owned G01 receipt; independent of G00 rapid override accounting.
enum class MotionFeedLineCode : std::uint8_t
{
    NONE = 0,
    COMMITTED = 1,
    INVALID_INPUT = 2,
    NOT_READY = 3,
    GEOMETRY_REJECTED = 4,
    PRODUCER_REJECTED = 5,
    STALE_AFTER_ACCEPT = 6,
    CAPTURE_MISMATCH = 7
};

struct MotionFeedLineReceipt
{
    NCPathCoreFeedLineV2 line{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint32_t validAxisMask = 0U;
    std::uint32_t geometryCode = 0U;
    MotionFeedLineCode code = MotionFeedLineCode::NONE;
    bool commandAccepted = false;
    bool tailCommitted = false;
    bool captureBound = false;
    bool valid = false;

    void Clear() noexcept
    {
        line.Clear();
        identity = MotionExecutionIdentity{};
        ownerLease = MotionOwnerLease{};
        validAxisMask = 0U;
        geometryCode = 0U;
        code = MotionFeedLineCode::NONE;
        commandAccepted = false;
        tailCommitted = false;
        captureBound = false;
        valid = false;
    }
};

// Construct once with the NCManager heap workspace. No per-command allocation.
struct MotionFeedLineWorkspace
{
    NCPathCoreFeedLineInput input{};
    MotionFeedLineReceipt receipt{};
    std::array<double, 8U> stagedPulse{};
    std::array<double, 8U> stagedMCS{};
    std::vector<double> targetPulse;
    double accTime = 0.0;
    double decTime = 0.0;

    MotionFeedLineWorkspace()
    {
        targetPulse.reserve(8U);
    }
};
