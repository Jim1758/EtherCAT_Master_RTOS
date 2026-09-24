#pragma once

#include "MotionRotaryTarget.h"
#include <cmath>
#include <limits>

// BASE69: G90 positional-rotary target selection in native logical pulses.
// G00 keeps its existing resolver unchanged. We use its exact wrap/tie choice,
// then form a canonical absolute target + integral-turn offset. In particular,
// repeated identical G90 targets cannot accumulate fmod subtraction residues.
// No servo-window or tiny-move tolerance changes a nonzero command into a point.
inline bool IsNCRotaryAbsolutePulseResolutionValid(double value) noexcept
{
    // G90 phase selection must retain the consumer's smallest supported pulse
    // displacement. A huge finite authored angle must not lose its phase in
    // MCS*PPD and subsequently appear to be an exact modulo point.
    if (!std::isfinite(value)) return false;
    int exponent = 0;
    (void)std::frexp(std::fabs(value), &exponent);
    // binary64 spacing is 2^(exponent-53): exponent36 has 7.629e-6
    // pulse spacing, while exponent37 has 1.526e-5. frexp is also used by
    // the already accepted BASE68 rate converter; no new CRT dependency.
    return exponent <= 36;
}

struct NCRotaryAbsoluteFeedTarget
{
    double endMCS = 0.0;
    double endPulse = 0.0;
    double sweepDeg = 0.0; // Actual represented endMCS - startMCS, after addition.
    bool valid = false;
};

inline bool TryResolveNCRotaryAbsoluteFeedTarget(
    double startMCS, double startPulse, double requestedMCS,
    double pulsePerDegree, bool shortestPath, double rotaryModulo,
    NCRotaryAbsoluteFeedTarget& output) noexcept
{
    output = NCRotaryAbsoluteFeedTarget{};
    if (!std::isfinite(startMCS) || !std::isfinite(startPulse) ||
        !std::isfinite(requestedMCS) || !std::isfinite(pulsePerDegree) ||
        pulsePerDegree <= 0.0 || !std::isfinite(rotaryModulo) || rotaryModulo <= 0.0)
        return false;

    const double rawPulse = requestedMCS * pulsePerDegree;
    if (!IsNCRotaryAbsolutePulseResolutionValid(rawPulse) ||
        !IsNCRotaryAbsolutePulseResolutionValid(startPulse) ||
        (requestedMCS != 0.0 && rawPulse == 0.0))
        return false;
    double targetPulse = rawPulse;
    if (shortestPath)
    {
        const double moduloPulse = rotaryModulo * pulsePerDegree;
        if (!IsNCRotaryAbsolutePulseResolutionValid(moduloPulse) || moduloPulse <= 0.0) return false;
        double legacyTarget = 0.0;
        if (!TryResolveMotionTargetPulse(startPulse, rawPulse, pulsePerDegree,
            true, rotaryModulo, legacyTarget)) return false;
        const double turnDistance = legacyTarget - rawPulse;
        if (!std::isfinite(turnDistance)) return false;
        const double turns = std::round(turnDistance / moduloPulse);
        // Beyond this range a double cannot retain all integer turn choices
        // together with the fractional information needed to identify a turn.
        if (!std::isfinite(turns) || std::fabs(turns) >= 4503599627370496.0)
            return false;
        const double turnPulse = turns * moduloPulse;
        if (!std::isfinite(turnPulse)) return false;
        targetPulse = rawPulse + turnPulse;
        if (!std::isfinite(targetPulse)) return false;

        const double legacyDelta = legacyTarget - startPulse;
        const double canonicalDelta = targetPulse - startPulse;
        if (!std::isfinite(legacyDelta) || !std::isfinite(canonicalDelta)) return false;
        // Bound only local resolution arithmetic, never the possibly enormous
        // authored target or cancelled turn product. Large lossy cancellation
        // therefore rejects instead of concealing an incorrect physical move.
        double localScale = std::fabs(startPulse);
        if (std::fabs(legacyTarget) > localScale) localScale = std::fabs(legacyTarget);
        if (moduloPulse > localScale) localScale = moduloPulse;
        const double roundoff = std::fmin(1.0e-5, localScale *
            (32.0 * (std::numeric_limits<double>::epsilon)()));
        const double correction = targetPulse - legacyTarget;
        if (!std::isfinite(roundoff) || !std::isfinite(correction) ||
            std::fabs(correction) > roundoff ||
            (legacyDelta == 0.0 && canonicalDelta != 0.0) ||
            (legacyDelta != 0.0 && canonicalDelta != 0.0 &&
                (legacyDelta < 0.0) != (canonicalDelta < 0.0))) return false;
        const double halfModulo = moduloPulse / 2.0;
        if (std::fabs(canonicalDelta) > halfModulo &&
            std::fabs(canonicalDelta) - halfModulo > roundoff) return false;
    }

    const double pulseDelta = targetPulse - startPulse;
    if (!std::isfinite(pulseDelta)) return false;
    // Exact canonical equality alone is a point. Copy the native start bits;
    // adding a zero could change the sign of a represented negative zero.
    const double nativeDelta = pulseDelta / pulsePerDegree;
    const double nativeEnd = pulseDelta == 0.0 ? startMCS : startMCS + nativeDelta;
    const double nativeSweep = nativeEnd - startMCS;
    if (!std::isfinite(nativeDelta) || !std::isfinite(nativeEnd) ||
        !std::isfinite(nativeSweep) ||
        (pulseDelta != 0.0 && (nativeDelta == 0.0 || nativeSweep == 0.0 ||
            (nativeSweep < 0.0) != (pulseDelta < 0.0)))) return false;
    output.endMCS = nativeEnd;
    output.endPulse = pulseDelta == 0.0 ? startPulse : targetPulse;
    output.sweepDeg = nativeSweep;
    output.valid = true;
    return true;
}
