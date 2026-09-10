#pragma once
#include <cmath>
#include <cstddef>

// CA: retained commands borrow these transform-origin slots only while the
// transform is disabled. Canonical mem_* geometry always remains the ORIGINAL
// complete source; the interval defines scalar travel, never replacement data.
inline bool IsMotionRetainedIntervalBoundsValid(double startU, double endU,
    double originalLength, double& distance) noexcept
{
    distance = 0.0;
    if (!std::isfinite(startU) || !std::isfinite(endU) ||
        startU < 0.0 || startU > 1.0 || endU < 0.0 || endU > 1.0 || startU == endU ||
        !std::isfinite(originalLength) || originalLength < 0.0)
        return false;
    const double span = std::abs(endU - startU);
    // Only the legacy full traversal of an original point may have no distance.
    if (originalLength == 0.0)
        return span == 1.0;
    distance = span * originalLength;
    return std::isfinite(distance) && distance > 0.0;
}

template<class Command>
inline bool GetMotionRetainedInterval(const Command& command,
    double& startU, double& endU, double& distance) noexcept
{
    startU = endU = distance = 0.0;
    if (!command.pathCoreRetainedTraversal || command.mem_enableTransform)
        return false;
    const double marker = command.mem_transformOrigin[2];
    if (marker == 0.0)
    {
        // BZ full-range command encoding. Stray interval bytes are malformed.
        if (command.mem_transformOrigin[0] != 0.0 || command.mem_transformOrigin[1] != 0.0)
            return false;
        startU = command.pathCoreRetainedReverse ? 1.0 : 0.0;
        endU = command.pathCoreRetainedReverse ? 0.0 : 1.0;
    }
    else if (marker == 1.0)
    {
        startU = command.mem_transformOrigin[0];
        endU = command.mem_transformOrigin[1];
    }
    else return false;
    return IsMotionRetainedIntervalBoundsValid(startU, endU, command.mem_totalDist, distance) &&
        command.pathCoreRetainedReverse == (endU < startU);
}

inline double MotionRetainedParameterAtProgress(double startU, double endU,
    double progress) noexcept
{
    // Copy the stored interval boundaries exactly, including an interior stop.
    if (progress <= 0.0) return startU;
    if (progress >= 1.0) return endU;
    return startU + (endU - startU) * progress;
}

template<class Command>
inline bool EvaluateMotionRetainedPulseCanonical(const Command& command,
    std::size_t axis, double u, double& output) noexcept
{
    output = 0.0;
    if (axis >= 8U || !std::isfinite(u) || u < 0.0 || u > 1.0)
        return false;
    const double start = command.mem_startPos[axis];
    const double end = command.mem_ratio[axis];
    if (!std::isfinite(start) || !std::isfinite(end)) return false;
    if (u == 0.0) { output = start; return true; }
    if (u == 1.0) { output = end; return true; }
    bool selected = false;
    for (int slot = 0; slot < command.axisCount && slot < 8; ++slot)
        selected = selected || command.axisIndices[slot] == static_cast<int>(axis);
    if (!selected)
    {
        if (start != end) return false;
        output = start;
        return true;
    }
    if (!command.pathCorePlanarCircle)
    {
        const double delta = end - start;
        if (!std::isfinite(delta)) return false;
        output = start + u * delta;
    }
    else
    {
        if (axis >= 2U || !std::isfinite(command.mem_startAngle) ||
            !std::isfinite(command.mem_totalAngle) || !std::isfinite(command.mem_radius) ||
            command.mem_radius <= 0.0)
            return false;
        const double center = axis == 0U ? command.mem_centerX : command.mem_centerY;
        const double angle = command.mem_startAngle + u * command.mem_totalAngle;
        output = center + command.mem_radius * (axis == 0U ? std::cos(angle) : std::sin(angle));
    }
    return std::isfinite(output);
}
