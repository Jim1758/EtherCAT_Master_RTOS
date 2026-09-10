#include "NCPathCoreFeedLine.h"

#include <cmath>
#include <cstring>

namespace
{
    bool SameBits(const double a, const double b) noexcept
    {
        std::uint64_t aBits = 0ULL;
        std::uint64_t bBits = 0ULL;
        std::memcpy(&aBits, &a, sizeof(aBits));
        std::memcpy(&bBits, &b, sizeof(bBits));
        return aBits == bBits;
    }

    // Positive finite arguments. Exponent sums fit int for binary64.
    // Do not form a*b, b/c, or 60*c as a potentially lossy intermediate.
    double ScaledProductQuotient(const double a, const double b,
        const double c, const bool perMinute) noexcept
    {
        int aExponent = 0;
        int bExponent = 0;
        int cExponent = 0;
        const double aMantissa = std::frexp(a, &aExponent);
        const double bMantissa = std::frexp(b, &bExponent);
        const double cMantissa = std::frexp(c, &cExponent);
        double mantissa = (aMantissa * bMantissa) / cMantissa;
        if (perMinute) mantissa /= 60.0;
        return std::ldexp(mantissa, aExponent + bExponent - cExponent);
    }
}

void NCPathCoreFeedLineV2::Clear() noexcept
{
    startMCS.fill(0.0);
    endMCS.fill(0.0);
    startPulse.fill(0.0);
    endPulse.fill(0.0);
    lengthMM = 0.0;
    lengthPulse = 0.0;
    feedMMMin = 0.0;
    velocityPPS = 0.0;
    axisMask = 0U;
    point = false;
    valid = false;
}

NCPathCoreFeedLineCode BuildNCPathCoreFeedLine(
    const NCPathCoreFeedLineInput& input,
    NCPathCoreFeedLineV2& output) noexcept
{
    using Code = NCPathCoreFeedLineCode;
    output.Clear();
    if (input.axisMask == 0U || (input.axisMask & ~7U) != 0U)
        return Code::INVALID_AXIS_MASK;
    if (!std::isfinite(input.feedMMMin) || input.feedMMMin <= 0.0 ||
        input.feedMMMin > 100.0)
        return Code::INVALID_FEED;

    double lengthMM = 0.0;
    double lengthPulse = 0.0;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) ||
            !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) ||
            !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if ((input.axisMask & (1U << axis)) == 0U)
        {
            if (!SameBits(input.startMCS[axis], input.endMCS[axis]) ||
                !SameBits(input.startPulse[axis], input.endPulse[axis]))
                return Code::OUTSIDE_AXIS_CHANGED;
            continue;
        }
        if (!std::isfinite(input.maxVelocityPPS[axis]) ||
            input.maxVelocityPPS[axis] <= 0.0)
            return Code::INVALID_AXIS_CONFIGURATION;

        const double deltaMM = input.endMCS[axis] - input.startMCS[axis];
        const double deltaPulse = input.endPulse[axis] - input.startPulse[axis];
        if (!std::isfinite(deltaMM) || !std::isfinite(deltaPulse))
            return Code::NONFINITE_GEOMETRY;
        if ((deltaMM == 0.0) != (deltaPulse == 0.0))
            return Code::ZERO_LENGTH_MISMATCH;
        if (deltaMM != 0.0 && ((deltaMM < 0.0) != (deltaPulse < 0.0)))
            return Code::DIRECTION_MISMATCH;
        lengthMM = std::hypot(lengthMM, deltaMM);
        lengthPulse = std::hypot(lengthPulse, deltaPulse);
        if (!std::isfinite(lengthMM) || !std::isfinite(lengthPulse))
            return Code::NONFINITE_GEOMETRY;
    }

    const bool point = lengthMM == 0.0;
    if (point != (lengthPulse == 0.0)) return Code::ZERO_LENGTH_MISMATCH;
    double velocityPPS = 0.0;
    if (!point)
    {
        velocityPPS = ScaledProductQuotient(
            input.feedMMMin, lengthPulse, lengthMM, true);
        if (!std::isfinite(velocityPPS) || velocityPPS <= 0.0)
            return Code::INVALID_GROUP_VELOCITY;
        for (std::uint32_t axis = 0U; axis < 3U; ++axis)
        {
            if ((input.axisMask & (1U << axis)) == 0U) continue;
            const double distancePulse = std::fabs(
                input.endPulse[axis] - input.startPulse[axis]);
            if (distancePulse == 0.0) continue;
            const double projectedVelocity = ScaledProductQuotient(
                velocityPPS, distancePulse, lengthPulse, false);
            if (!std::isfinite(projectedVelocity) || projectedVelocity <= 0.0)
                return Code::INVALID_GROUP_VELOCITY;
            if (projectedVelocity > input.maxVelocityPPS[axis])
                return Code::AXIS_VELOCITY_LIMIT;
        }
    }

    // Publish a complete value only after all geometry and speed checks pass.
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        output.startMCS[axis] = input.startMCS[axis];
        output.endMCS[axis] = input.endMCS[axis];
        output.startPulse[axis] = input.startPulse[axis];
        output.endPulse[axis] = input.endPulse[axis];
    }
    output.lengthMM = lengthMM;
    output.lengthPulse = lengthPulse;
    output.feedMMMin = input.feedMMMin;
    output.velocityPPS = velocityPPS;
    output.axisMask = input.axisMask;
    output.point = point;
    output.valid = true;
    return point ? Code::BUILT_POINT : Code::BUILT_LINE;
}

bool EvaluateNCPathCoreFeedLineAxis(const NCPathCoreFeedLineV2& line,
    const std::uint32_t axisIndex, const double unitParameter,
    double& outputMCS) noexcept
{
    outputMCS = 0.0;
    if (!line.valid || line.axisMask == 0U || (line.axisMask & ~7U) != 0U ||
        axisIndex >= 8U || !std::isfinite(unitParameter) ||
        unitParameter < 0.0 || unitParameter > 1.0)
        return false;
    const double start = line.startMCS[axisIndex];
    const double end = line.endMCS[axisIndex];
    if (!std::isfinite(start) || !std::isfinite(end)) return false;
    if (unitParameter == 0.0) { outputMCS = start; return true; }
    if (unitParameter == 1.0) { outputMCS = end; return true; }
    if (start == end) { outputMCS = start; return true; }
    const bool straddlesZero =
        (start <= 0.0 && end >= 0.0) || (start >= 0.0 && end <= 0.0);
    const double coordinate = straddlesZero
        ? (1.0 - unitParameter) * start + unitParameter * end
        : start + unitParameter * (end - start);
    if (!std::isfinite(coordinate)) return false;
    const double lower = start < end ? start : end;
    const double upper = start < end ? end : start;
    outputMCS = coordinate < lower ? lower :
        (coordinate > upper ? upper : coordinate);
    return true;
}
