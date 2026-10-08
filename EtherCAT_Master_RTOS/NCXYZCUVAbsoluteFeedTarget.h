#pragma once

#include "NCRotaryFeedTarget.h"
#include <array>
#include <cmath>

// BASE74: canonical G90 resolution for fixed XYZ millimetres and C/U/V degrees.
// XYZ use raw MCS absolute pulse targets. Each rotary axis independently uses
// the established positional-rotary shortest/tie policy and continuous pulses.
// This resolver permits exact points; six-axis feed geometry requires every
// selected effective delta to move. Source offsets, travel and ownership remain
// caller responsibilities. Inputs/output are disjoint; failure clears output.
struct NCXYZCUVAbsoluteFeedTarget
{
    std::array<double, 6U> endMCS{}, endPulse{}, deltaNative{};
    bool valid = false;
};

inline bool TryResolveNCXYZCUVAbsoluteFeedTarget(
    const std::array<double, 6U>& startMCS,
    const std::array<double, 6U>& startPulse,
    const std::array<double, 6U>& requestedMCS,
    const std::array<double, 6U>& pulsePerUnit,
    const std::array<bool, 3U>& shortestPath,
    const std::array<double, 3U>& rotaryModulo,
    NCXYZCUVAbsoluteFeedTarget& output) noexcept
{
    output = NCXYZCUVAbsoluteFeedTarget{};
    NCXYZCUVAbsoluteFeedTarget candidate{};
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        if (!std::isfinite(startMCS[axis]) || !std::isfinite(startPulse[axis]) ||
            !std::isfinite(requestedMCS[axis]) || !std::isfinite(pulsePerUnit[axis]) ||
            pulsePerUnit[axis] <= 0.0) return false;
        const double targetPulse = requestedMCS[axis] * pulsePerUnit[axis];
        if (!IsNCRotaryAbsolutePulseResolutionValid(startPulse[axis]) ||
            !IsNCRotaryAbsolutePulseResolutionValid(targetPulse) ||
            (requestedMCS[axis] != 0.0 && targetPulse == 0.0)) return false;
        const double nativeDelta = requestedMCS[axis] - startMCS[axis];
        const double pulseDelta = targetPulse - startPulse[axis];
        if (!std::isfinite(nativeDelta) || !std::isfinite(pulseDelta) ||
            ((nativeDelta == 0.0) != (pulseDelta == 0.0)) ||
            (nativeDelta != 0.0 && (nativeDelta < 0.0) != (pulseDelta < 0.0))) return false;
        candidate.endMCS[axis] = requestedMCS[axis];
        candidate.endPulse[axis] = targetPulse;
        candidate.deltaNative[axis] = nativeDelta;
    }
    for (unsigned rotary = 0U; rotary < 3U; ++rotary)
    {
        const unsigned axis = rotary + 3U;
        NCRotaryAbsoluteFeedTarget resolved{};
        if (!TryResolveNCRotaryAbsoluteFeedTarget(startMCS[axis], startPulse[axis],
            requestedMCS[axis], pulsePerUnit[axis], shortestPath[rotary],
            rotaryModulo[rotary], resolved)) return false;
        candidate.endMCS[axis] = resolved.endMCS;
        candidate.endPulse[axis] = resolved.endPulse;
        candidate.deltaNative[axis] = resolved.sweepDeg;
    }
    candidate.valid = true;
    output = candidate;
    return true;
}
