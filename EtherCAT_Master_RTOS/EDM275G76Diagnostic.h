#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

// Independent observation only. No motion, ownership or safety authority.
namespace EDM275
{
enum SnapshotFlag : std::uint32_t
{
    PhysicalValid = 1U << 0U,
    RowComplete = 1U << 1U,
    AtCurrentEnd = 1U << 2U,
    AwaitingSeam = 1U << 3U,
    CompleteReady = 1U << 4U,
    GroupActive = 1U << 5U,
    AllCommandZero = 1U << 6U,
    MappedVelocityZero = 1U << 7U,
    PriorSameReference = 1U << 8U,
    PhysicalReferenceMatchesEnd = 1U << 9U,
    CoordinatorCommittedZero = 1U << 10U,
    CoordinatorImageZero = 1U << 11U,
    CoordinatorSamePulse = 1U << 12U
};

enum AxisFlag : std::uint32_t
{
    Healthy = 1U << 0U,
    Writable = 1U << 1U,
    CommandMatchesReference = 1U << 2U,
    Finite = 1U << 3U,
    ActualInWindow = 1U << 4U,
    FollowingInWindow = 1U << 5U,
    RangePass = 1U << 6U,
    CompensationReady = 1U << 7U
};

struct Axis
{
    double commandPulse = 0.0, actualPulse = 0.0, windowPulse = 0.0, rangePulse = 0.0;
    double commandPps = 0.0, logicalPps = 0.0, targetPps = 0.0, referencePulse = 0.0;
    std::uint32_t flags = 0U;
};

struct Snapshot
{
    std::uint64_t session = 0ULL, epoch = 0ULL, ownerGeneration = 0ULL;
    std::uint64_t tick = 0ULL, nowUs = 0ULL, physicalTick = 0ULL, mappedTick = 0ULL;
    std::uint64_t lastSentTick = 0ULL, proofTick = 0ULL, stableStartUs = 0ULL, dwellUs = 0ULL;
    std::uint32_t phase = 0U, child = 0U, completedChildren = 0U;
    std::uint32_t evaluated = 0U, failed = 0U, axisMask = 0U, flags = 0U, stableCycles = 0U;
    double localMm = 0.0, lengthMm = 0.0, velocityMmSec = 0.0;
    std::array<Axis, 6U> axis{};
};

static_assert(std::is_trivially_copyable<Snapshot>::value,
    "EDM275 diagnostic snapshot must remain a fixed value image.");
static_assert(sizeof(Snapshot) <= 768U,
    "EDM275 diagnostic channel has a bounded 768-byte image.");

// Defined by MotionCore.cpp using the existing independent EDM53 channel.
// A missing/busy observation never changes control or requests a fault.
bool Read(Snapshot& snapshot, std::uint64_t& publication) noexcept;
}
