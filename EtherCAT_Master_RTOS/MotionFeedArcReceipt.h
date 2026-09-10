#pragma once

#include "MotionExecutionContract.h"
#include "NCPathCoreFeedArc.h"

#include <array>
#include <cstdint>

// BY: borrowed only during one synchronous producer call; never retained.
// Both bounds of each selected axis are checked before publishing an epoch.
struct MotionArcTravelGuard
{
    const void* context = nullptr;
    bool (*check)(const void*, int, double) = nullptr;
};

enum class MotionFeedArcCode : std::uint8_t
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

struct MotionFeedArcReceipt
{
    NCPathCoreFeedArcV2 arc{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint32_t validAxisMask = 0U;
    std::uint32_t geometryCode = 0U;
    MotionFeedArcCode code = MotionFeedArcCode::NONE;
    bool commandAccepted = false;
    bool tailCommitted = false;
    bool captureBound = false;
    bool travelLimitRejected = false;
    bool valid = false;

    void Clear() noexcept
    {
        arc.Clear();
        identity = MotionExecutionIdentity{};
        ownerLease = MotionOwnerLease{};
        validAxisMask = 0U;
        geometryCode = 0U;
        code = MotionFeedArcCode::NONE;
        commandAccepted = false;
        tailCommitted = false;
        captureBound = false;
        travelLimitRejected = false;
        valid = false;
    }
};

// Startup-owned NC scratch. MotionCommand is separately caller-owned to avoid
// a MotionCore.h dependency here and a large automatic aggregate temporary.
struct MotionFeedArcWorkspace
{
    NCPathCoreFeedArcInput input{};
    MotionFeedArcReceipt receipt{};
    std::array<double, 8U> stagedPulse{};
    std::array<double, 8U> stagedMCS{};
    double accTime = 0.0;
    double decTime = 0.0;
};
