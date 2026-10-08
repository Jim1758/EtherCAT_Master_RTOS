#pragma once

#include "NCRotaryFeedTarget.h"
#include <array>
#include <cmath>

// BASE75: canonical G90 resolution for fixed XYZ millimetres and C degrees.
// XYZ use raw MCS absolute pulse targets. The C axis uses
// the established positional-rotary shortest/tie policy and continuous pulses.
// This resolver permits exact points; four-axis feed geometry requires every
// selected effective delta to move. Source offsets, travel and ownership remain
// caller responsibilities. Inputs/output are disjoint; failure clears output.
struct NCXYZCAbsoluteFeedTarget
{
    std::array<double, 4U> endMCS{}, endPulse{}, deltaNative{};
    bool valid = false;
};

inline bool TryResolveNCXYZCAbsoluteFeedTarget(
    const std::array<double, 4U>& startMCS,
    const std::array<double, 4U>& startPulse,
    const std::array<double, 4U>& requestedMCS,
    const std::array<double, 4U>& pulsePerUnit,
    bool shortestPath,
    double rotaryModulo,
    NCXYZCAbsoluteFeedTarget& output) noexcept
{
    output = NCXYZCAbsoluteFeedTarget{};
    NCXYZCAbsoluteFeedTarget candidate{};
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
    NCRotaryAbsoluteFeedTarget rotary{};
    if (!TryResolveNCRotaryAbsoluteFeedTarget(startMCS[3], startPulse[3],
        requestedMCS[3], pulsePerUnit[3], shortestPath, rotaryModulo, rotary)) return false;
    candidate.endMCS[3] = rotary.endMCS;
    candidate.endPulse[3] = rotary.endPulse;
    candidate.deltaNative[3] = rotary.sweepDeg;
    candidate.valid = true;
    output = candidate;
    return true;
}
