#pragma once

#include "NCRotaryFeedTarget.h"
#include <cmath>

// BASE71: one canonical G90 target resolution, in separate Z millimetres and
// C degrees. Absolute Z uses its raw MCS target. C uses the already established
// positional-rotary wrap/tie policy once, and retains continuous pulse motion.
// Callers still prove source offsets, owner/epoch, raw and physical travel.
struct NCZCAbsoluteFeedTarget
{
    double endZMCS = 0.0, endCMCS = 0.0;
    double endZPulse = 0.0, endCPulse = 0.0;
    double deltaZMM = 0.0, deltaCDeg = 0.0;
    bool valid = false;
};

inline bool TryResolveNCZCAbsoluteFeedTarget(
    double startZMCS, double startZPulse, double startCMCS, double startCPulse,
    double requestedZMCS, double requestedCMCS, double pulsePerMM,
    double pulsePerDegree, bool shortestPath, double rotaryModulo,
    NCZCAbsoluteFeedTarget& output) noexcept
{
    output = NCZCAbsoluteFeedTarget{};
    if (!std::isfinite(startZMCS) || !std::isfinite(startZPulse) ||
        !std::isfinite(requestedZMCS) || !std::isfinite(pulsePerMM) ||
        pulsePerMM <= 0.0) return false;
    const double targetZPulse = requestedZMCS * pulsePerMM;
    // Absolute endpoints must retain the consumer's 1e-5 pulse resolution.
    // The same finite binary64 spacing guard is already used for absolute C.
    if (!IsNCRotaryAbsolutePulseResolutionValid(startZPulse) ||
        !IsNCRotaryAbsolutePulseResolutionValid(targetZPulse) ||
        (requestedZMCS != 0.0 && targetZPulse == 0.0)) return false;
    const double deltaZMM = requestedZMCS - startZMCS;
    const double deltaZPulse = targetZPulse - startZPulse;
    if (!std::isfinite(deltaZMM) || !std::isfinite(deltaZPulse) ||
        ((deltaZMM == 0.0) != (deltaZPulse == 0.0)) ||
        (deltaZMM != 0.0 && (deltaZMM < 0.0) != (deltaZPulse < 0.0))) return false;
    NCRotaryAbsoluteFeedTarget rotary{};
    if (!TryResolveNCRotaryAbsoluteFeedTarget(startCMCS, startCPulse,
        requestedCMCS, pulsePerDegree, shortestPath, rotaryModulo, rotary)) return false;
    output.endZMCS = requestedZMCS;
    output.endZPulse = targetZPulse;
    output.endCMCS = rotary.endMCS;
    output.endCPulse = rotary.endPulse;
    output.deltaZMM = deltaZMM;
    output.deltaCDeg = rotary.sweepDeg;
    output.valid = true;
    return true;
}
