#pragma once

#include "MotionEccentricCTransport.h"

// BASE79G: one stopped NC producer owns this heap workspace. It is never
// copied into a Motion queue, CNC flight, or the RT consumer's scratch.
// Preparation is allocation-free after explicit NC startup initialization.
struct MotionEccentricCProducerWorkspace
{
    NCEccentricCRuntimeInput input{};
    NCEccentricCRuntimeValue runtime{};
    MotionEccentricCTransportWorkspace transport{};
    MotionCommand command{};
    std::array<double, 8U> originalTail{}, physicalEndMCS{};

    MotionEccentricCProducerWorkspace() noexcept = default;
    MotionEccentricCProducerWorkspace(const MotionEccentricCProducerWorkspace&) = delete;
    MotionEccentricCProducerWorkspace& operator=(const MotionEccentricCProducerWorkspace&) = delete;
};
