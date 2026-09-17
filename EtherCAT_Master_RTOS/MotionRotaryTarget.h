#pragma once
#include <cmath>

// Positions at Motion admission are pulses; rotaryModulo is in native degrees.
// Failure leaves the output untouched. No AxisContext/RT dependency or allocation.
inline bool TryGetMotionPulsePerUnit(
    double resolutionPPR, double finalLead, bool rotaryShortestPath,
    double& pulsePerUnit) noexcept
{
    if (rotaryShortestPath &&
        (!std::isfinite(finalLead) || finalLead <= 0.0 ||
            !std::isfinite(resolutionPPR) || resolutionPPR <= 0.0))
    {
        return false;
    }
    // Retain the legacy lead fallback outside wrapped rotary positioning.
    const double lead = !rotaryShortestPath && finalLead < 1.0e-6 ? 1.0 : finalLead;
    const double candidate = resolutionPPR / lead;
    if (!std::isfinite(candidate) || candidate <= 0.0) return false;
    pulsePerUnit = candidate;
    return true;
}

inline bool TryResolveMotionTargetPulse(
    double currentPulse, double targetPulse, double pulsePerUnit,
    bool rotaryShortestPath, double rotaryModuloUnits,
    double& resolvedTargetPulse) noexcept
{
    if (!std::isfinite(currentPulse) || !std::isfinite(targetPulse) ||
        !std::isfinite(pulsePerUnit) || pulsePerUnit <= 0.0)
    {
        return false;
    }
    double candidate = targetPulse;
    if (rotaryShortestPath)
    {
        if (!std::isfinite(rotaryModuloUnits) || rotaryModuloUnits <= 0.0)
            return false;
        const double moduloPulse = rotaryModuloUnits * pulsePerUnit;
        if (!std::isfinite(moduloPulse) || moduloPulse <= 0.0) return false;

        double currentModulo = std::fmod(currentPulse, moduloPulse);
        if (currentModulo < 0.0) currentModulo += moduloPulse;
        double targetModulo = std::fmod(targetPulse, moduloPulse);
        if (targetModulo < 0.0) targetModulo += moduloPulse;
        double deltaPulse = targetModulo - currentModulo;
        const double halfModuloPulse = moduloPulse / 2.0;
        // Strict comparisons preserve the existing +/- half-turn tie direction.
        if (deltaPulse > halfModuloPulse) deltaPulse -= moduloPulse;
        else if (deltaPulse < -halfModuloPulse) deltaPulse += moduloPulse;
        candidate = currentPulse + deltaPulse;
    }
    // Include the following distance subtraction in the finite contract.
    if (!std::isfinite(candidate) || !std::isfinite(candidate - currentPulse))
        return false;
    resolvedTargetPulse = candidate;
    return true;
}
