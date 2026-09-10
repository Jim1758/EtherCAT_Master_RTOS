#pragma once

#include "MotionQueueTailTransaction.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// BQ: Producer-owned native-MCS baselines for one successful ordinary G00.
// The source remains commanded geometry, not an RT start/ACT or rotary unwrap.
constexpr std::size_t MOTION_COMMANDED_ENDPOINT_AXIS_COUNT = 8U;
constexpr std::uint16_t MOTION_COMMANDED_ENDPOINT_SCHEMA_VERSION = 1U;

enum class MotionCommandedBaselineOrigin : std::uint8_t
{
    NONE = 0U,
    BUFFERED_TAIL = 1U,
    ABORTING_BASELINE = 2U
};

struct MotionCommandedEndpointReceiptV1
{
    MotionQueueTailCommitReceipt transaction{};
    std::array<double, MOTION_COMMANDED_ENDPOINT_AXIS_COUNT> startMCS{};
    std::array<double, MOTION_COMMANDED_ENDPOINT_AXIS_COUNT> endMCS{};
    std::uint32_t validAxisMask = 0U;
    std::uint16_t schemaVersion = 0U;
    MotionCommandedBaselineOrigin origin = MotionCommandedBaselineOrigin::NONE;
    bool valid = false;

    void Clear() noexcept
    {
        *this = MotionCommandedEndpointReceiptV1{};
    }
};

static_assert(std::is_standard_layout<MotionCommandedEndpointReceiptV1>::value,
    "BQ commanded endpoint receipt must remain standard layout.");
static_assert(std::is_trivially_copyable<MotionCommandedEndpointReceiptV1>::value,
    "BQ commanded endpoint receipt must remain trivially copyable.");
static_assert(sizeof(MotionCommandedEndpointReceiptV1) == 216U,
    "BQ commanded endpoint receipt ABI changed.");
static_assert(alignof(MotionCommandedEndpointReceiptV1) == 8U,
    "BQ commanded endpoint receipt alignment changed.");
static_assert(offsetof(MotionCommandedEndpointReceiptV1, startMCS) == 80U &&
    offsetof(MotionCommandedEndpointReceiptV1, endMCS) == 144U &&
    offsetof(MotionCommandedEndpointReceiptV1, validAxisMask) == 208U &&
    offsetof(MotionCommandedEndpointReceiptV1, schemaVersion) == 212U &&
    offsetof(MotionCommandedEndpointReceiptV1, origin) == 214U &&
    offsetof(MotionCommandedEndpointReceiptV1, valid) == 215U,
    "BQ commanded endpoint receipt member offsets changed.");
