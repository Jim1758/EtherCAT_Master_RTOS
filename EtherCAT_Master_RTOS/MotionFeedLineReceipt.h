#pragma once

#include "MotionExecutionContract.h"
#include "NCPathCoreFeedLine.h"
#include "NCPathCoreRetainedPath.h"

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
    NCPathCoreRetainedGeometry blendGeometry{};
    NCPathCoreCornerMetadata blendMetadata{};
    // DI timing permission; canonical geometry and authored source F stay unchanged.
    double cornerNextFeedMMMin = 0.0;
    double cornerDispatchFeedMMMin = 0.0;
    double cornerDispatchVelocityPPS = 0.0;
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint64_t translationGeneration = 0ULL;
    std::uint32_t validAxisMask = 0U;
    std::uint32_t geometryCode = 0U;
    MotionFeedLineCode code = MotionFeedLineCode::NONE;
    bool commandAccepted = false;
    bool tailCommitted = false;
    bool captureBound = false;
    bool valid = false;

    void Clear() noexcept
    {
        line.Clear(); blendGeometry.Clear(); blendMetadata = NCPathCoreCornerMetadata{};
        cornerNextFeedMMMin = cornerDispatchFeedMMMin = cornerDispatchVelocityPPS = 0.0;
        identity = MotionExecutionIdentity{};
        ownerLease = MotionOwnerLease{};
        translationGeneration = 0ULL;
        validAxisMask = 0U;
        geometryCode = 0U;
        code = MotionFeedLineCode::NONE;
        commandAccepted = false;
        tailCommitted = false;
        captureBound = false;
        valid = false;
    }
};

// DG: an immutable commanded endpoint anchor, not a replacement line/arc.
// Permission is revalidated against the live owner, epoch and producer tail.
struct MotionCncPathTail
{
    std::array<double, 8U> endMCS{}, endPulse{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    std::uint64_t translationGeneration = 0ULL;
    std::uint32_t axisMask = 0U, validAxisMask = 0U;
    bool valid = false;

    void Clear() noexcept
    {
        endMCS.fill(0.0); endPulse.fill(0.0);
        identity = MotionExecutionIdentity{}; ownerLease = MotionOwnerLease{}; translationGeneration = 0ULL;
        axisMask = validAxisMask = 0U; valid = false;
    }
    void Assign(const MotionFeedLineReceipt& receipt) noexcept
    {
        endMCS = receipt.blendGeometry.valid ? receipt.blendGeometry.endMCS : receipt.line.endMCS;
        endPulse = receipt.blendGeometry.valid ? receipt.blendGeometry.endPulse : receipt.line.endPulse;
        identity = receipt.identity; ownerLease = receipt.ownerLease;
        translationGeneration = receipt.translationGeneration;
        axisMask = receipt.line.axisMask; validAxisMask = receipt.validAxisMask;
        valid = receipt.valid && receipt.commandAccepted && receipt.captureBound &&
            receipt.tailCommitted && receipt.line.valid;
    }
};
static_assert(sizeof(MotionCncPathTail) <= 192U &&
    std::is_trivially_copyable<MotionCncPathTail>::value, "DG bounded endpoint anchor.");

// Construct once with the NCManager heap workspace. No per-command allocation.
struct MotionFeedLineWorkspace
{
    NCPathCoreFeedLineInput input{};
    NCPathCoreFeedArcInput cornerArcInput{};
    NCPathCoreFeedArcV2 cornerArc{};
    MotionFeedLineReceipt receipt{};
    MotionCncPathTail predecessorTail{};
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
