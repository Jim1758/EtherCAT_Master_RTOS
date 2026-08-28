#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "MotionExecutionContract.h"
#include "MotionCommandRing.h"

// ============================================================================
// Stage NC-0.1F - Fixed-capacity Axis Command Mailbox
//
// Control producer (10 ms): NCPLC / HMI / Homing supervisory state machine.
// RT consumer (250 us):     MotionCore::UpdateInterpolation().
//
// Result direction is reversed and remains strict SPSC.  No allocation,
// mutex, blocking wait or container growth occurs after construction.
// ============================================================================

using MotionAxisCommandSequence = std::uint64_t;
constexpr MotionAxisCommandSequence MOTION_AXIS_COMMAND_SEQUENCE_INVALID = 0ULL;

constexpr std::size_t MOTION_AXIS_COMMAND_CAPACITY = 256U;
constexpr std::size_t MOTION_AXIS_RESULT_CAPACITY = 512U;
constexpr std::size_t MOTION_AXIS_COMMAND_DRAIN_LIMIT_PER_RUNTIME_PASS = 64U;
constexpr std::size_t MOTION_AXIS_RESULT_DRAIN_LIMIT_PER_CONTROL_PASS = 256U;

enum class MotionAxisCommandType : std::uint8_t
{
    NONE = 0,
    MOVE_TO_POSITION = 1,
    VELOCITY_MOVE = 2,
    MPG_MOVE = 3,
    STOP_MOVE = 4,
    APPLY_MACHINE_HOME = 5,
    SET_TOUCH_PROBE_FUNCTION = 6
};

enum class MotionAxisCommandResultType : std::uint8_t
{
    NONE = 0,
    APPLIED = 1,
    REJECTED = 2
};

enum MotionAxisCommandFlags : std::uint8_t
{
    MOTION_AXIS_COMMAND_FLAG_NONE = 0U,
    MOTION_AXIS_COMMAND_FLAG_USE_SHORTEST_PATH = 1U << 0
};

struct MotionAxisCommand
{
    MotionAxisCommandSequence sequence = MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
    MotionAxisCommandType type = MotionAxisCommandType::NONE;
    std::int32_t axisIndex = -1;
    MotionCommandSource source = MotionCommandSource::UNKNOWN;
    MotionOwnerLease ownerLease{};

    double value0 = 0.0;
    double value1 = 0.0;
    double value2 = 0.0;
    double value3 = 0.0;

    std::uint16_t wordValue = 0U;
    std::uint8_t flags = MOTION_AXIS_COMMAND_FLAG_NONE;
};

struct MotionAxisCommandResult
{
    MotionAxisCommandSequence sequence = MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
    MotionAxisCommandType commandType = MotionAxisCommandType::NONE;
    MotionAxisCommandResultType resultType = MotionAxisCommandResultType::NONE;
    std::int32_t axisIndex = -1;
    MotionRejectReason rejectReason = MotionRejectReason::NONE;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration = MOTION_OWNER_GENERATION_INVALID;
};

class MotionAxisCommandChannel
{
public:
    bool ControlTrySubmit(const MotionAxisCommand& command) noexcept
    {
        return m_commands.ProducerTryPush(command);
    }

    bool RuntimeTryConsume(MotionAxisCommand& command) noexcept
    {
        return m_commands.ConsumerTryPop(command);
    }

    bool RuntimeTryPublishResult(const MotionAxisCommandResult& result) noexcept
    {
        return m_results.ProducerTryPush(result);
    }

    bool ControlTryConsumeResult(MotionAxisCommandResult& result) noexcept
    {
        return m_results.ConsumerTryPop(result);
    }

    std::size_t command_size() const noexcept { return m_commands.size(); }
    std::size_t result_size() const noexcept { return m_results.size(); }

private:
    FixedCapacitySpscRing<MotionAxisCommand, MOTION_AXIS_COMMAND_CAPACITY> m_commands{};
    FixedCapacitySpscRing<MotionAxisCommandResult, MOTION_AXIS_RESULT_CAPACITY> m_results{};
};

static_assert(std::is_standard_layout<MotionAxisCommand>::value,
    "MotionAxisCommand must remain standard-layout.");
static_assert(std::is_trivially_copyable<MotionAxisCommand>::value,
    "MotionAxisCommand must remain trivially copyable.");
static_assert(std::is_standard_layout<MotionAxisCommandResult>::value,
    "MotionAxisCommandResult must remain standard-layout.");
static_assert(std::is_trivially_copyable<MotionAxisCommandResult>::value,
    "MotionAxisCommandResult must remain trivially copyable.");
