#include "NCPathCoreFeedArc.h"

#include <cmath>
#include <cstring>

namespace
{
    constexpr double Pi = 3.141592653589793238462643383279502884;
    constexpr double TwoPi = 2.0 * Pi;
    constexpr double RoundoffFactor = 64.0 * std::numeric_limits<double>::epsilon();
    constexpr double MaximumBudgetMM = 1.0e-7;

    bool SameBits(const double a, const double b) noexcept
    {
        std::uint64_t aa = 0ULL, bb = 0ULL;
        std::memcpy(&aa, &a, sizeof(aa));
        std::memcpy(&bb, &b, sizeof(bb));
        return aa == bb;
    }

    double MaximumMagnitude(double value, const double candidate) noexcept
    {
        const double magnitude = std::fabs(candidate);
        return magnitude > value ? magnitude : value;
    }

    double CoordinateBudget(const double sx, const double sy,
        const double ex, const double ey, const double cx, const double cy,
        const double radius) noexcept
    {
        double scale = std::fabs(radius);
        scale = MaximumMagnitude(scale, sx);
        scale = MaximumMagnitude(scale, sy);
        scale = MaximumMagnitude(scale, ex);
        scale = MaximumMagnitude(scale, ey);
        scale = MaximumMagnitude(scale, cx);
        scale = MaximumMagnitude(scale, cy);
        return RoundoffFactor * scale;
    }

    bool Near(const double a, const double b, const double tolerance) noexcept
    {
        const double difference = a - b;
        return std::isfinite(difference) && std::fabs(difference) <= tolerance;
    }

    double PositiveAngle(double value) noexcept
    {
        if (value < 0.0) value += TwoPi;
        if (value >= TwoPi) value -= TwoPi;
        return value;
    }

    bool ContainsAngle(const double angle, const double start,
        const double sweep, const bool fullCircle) noexcept
    {
        if (fullCircle) return true;
        const double travel = PositiveAngle(sweep > 0.0 ? angle - start : start - angle);
        // A roundoff-sized overinclusion at a cardinal is conservative.
        return travel <= std::fabs(sweep) + RoundoffFactor * TwoPi;
    }

    // Positive finite inputs; avoid an avoidable overflow in F * scale.
    double FeedVelocity(const double feed, const double scale) noexcept
    {
        int ef = 0, es = 0;
        const double mf = std::frexp(feed, &ef);
        const double ms = std::frexp(scale, &es);
        return std::ldexp((mf * ms) / 60.0, ef + es);
    }
}

void NCPathCoreFeedArcV2::Clear() noexcept
{
    startMCS.fill(0.0); endMCS.fill(0.0);
    startPulse.fill(0.0); endPulse.fill(0.0);
    centerMCS.fill(0.0); centerPulse.fill(0.0);
    boundsMinMCS.fill(0.0); boundsMaxMCS.fill(0.0);
    radiusMM = radiusPulse = startAngle = sweepRadians = 0.0;
    lengthMM = lengthPulse = feedMMMin = velocityPPS = 0.0;
    direction = 0; axisMask = 0U; fullCircle = false; valid = false;
    sourceRoundoffMM = 0.0f; plane = 17U;
}

bool ResolveNCPathCorePlanarCirclePulse(const double sx, const double sy,
    const double ex, const double ey, const double cx, const double cy,
    const double expectedRadius, const int direction, const bool fullCircle,
    NCPathCoreArcPulseGeometry& output, const double additionalTolerance) noexcept
{
    output.radius = output.startAngle = output.sweepRadians = output.lengthPulse = 0.0;
    if (!std::isfinite(additionalTolerance) || additionalTolerance < 0.0 ||
        std::signbit(additionalTolerance) || (direction != -1 && direction != 1) ||
        !std::isfinite(sx) || !std::isfinite(sy) ||
        !std::isfinite(ex) || !std::isfinite(ey) ||
        !std::isfinite(cx) || !std::isfinite(cy) ||
        !std::isfinite(expectedRadius) || expectedRadius <= 0.0)
        return false;
    const double dx0 = sx - cx, dy0 = sy - cy;
    const double dx1 = ex - cx, dy1 = ey - cy;
    if (!std::isfinite(dx0) || !std::isfinite(dy0) ||
        !std::isfinite(dx1) || !std::isfinite(dy1)) return false;
    const double radius = std::hypot(dx0, dy0);
    const double endRadius = std::hypot(dx1, dy1);
    const double tolerance = CoordinateBudget(sx, sy, ex, ey, cx, cy, expectedRadius) +
        additionalTolerance;
    if (!std::isfinite(tolerance)) return false;
    if (!std::isfinite(radius) || !std::isfinite(endRadius) ||
        radius <= tolerance || endRadius <= tolerance ||
        !Near(radius, expectedRadius, tolerance) ||
        !Near(radius, endRadius, tolerance)) return false;
    const bool samePoint = sx == ex && sy == ey;
    if (samePoint != fullCircle) return false;
    const double start = std::atan2(dy0, dx0);
    double sweep = direction * TwoPi;
    if (!fullCircle)
    {
        const double end = std::atan2(dy1, dx1);
        sweep = end - start;
        if (direction == 1 && sweep <= 0.0) sweep += TwoPi;
        if (direction == -1 && sweep >= 0.0) sweep -= TwoPi;
        // Rounded zero angular travel must not silently become a full circle.
        if (std::fabs(sweep) < 1.0e-12 || std::fabs(sweep) >= TwoPi)
            return false;
    }
    const double length = radius * std::fabs(sweep);
    if (!std::isfinite(start) || !std::isfinite(sweep) ||
        !std::isfinite(length) || length <= 0.0) return false;
    output.radius = radius;
    output.startAngle = start;
    output.sweepRadians = sweep;
    output.lengthPulse = length;
    return true;
}

NCPathCoreFeedArcCode BuildNCPathCoreFeedArc(
    const NCPathCoreFeedArcInput& input, NCPathCoreFeedArcV2& output,
    const float sourceRoundoffMM) noexcept
{
    using Code = NCPathCoreFeedArcCode;
    output.Clear();
    NCArcPlaneAxes axes{};
    if (!TryGetNCArcPlaneAxes(input.plane, axes)) return Code::INVALID_PLANE;
    const unsigned slots[2] = { axes.u, axes.v };
    if (!std::isfinite(sourceRoundoffMM) || sourceRoundoffMM < 0.0f ||
        std::signbit(sourceRoundoffMM) || sourceRoundoffMM > MaximumBudgetMM)
        return Code::PRECISION_BUDGET;
    if (input.direction != -1 && input.direction != 1) return Code::INVALID_DIRECTION;
    if (!std::isfinite(input.feedMMMin) || input.feedMMMin <= 0.0 ||
        input.feedMMMin > 100.0) return Code::INVALID_FEED;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) || !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) || !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if (axis != axes.u && axis != axes.v && (!SameBits(input.startMCS[axis], input.endMCS[axis]) ||
            !SameBits(input.startPulse[axis], input.endPulse[axis])))
            return Code::OUTSIDE_AXIS_CHANGED;
    }
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        if (!std::isfinite(input.centerOffsetMM[axis])) return Code::NONFINITE_COORDINATE;
        if (!std::isfinite(input.pulsePerMM[axis]) || input.pulsePerMM[axis] <= 0.0 ||
            !std::isfinite(input.maxVelocityPPS[axis]) || input.maxVelocityPPS[axis] <= 0.0)
            return Code::INVALID_AXIS_CONFIGURATION;
        if (input.fullCircle && (!SameBits(input.startMCS[slots[axis]], input.endMCS[slots[axis]]) ||
            !SameBits(input.startPulse[slots[axis]], input.endPulse[slots[axis]])))
            return Code::INVALID_FULL_CIRCLE;
    }
    if (input.pulsePerMM[0] != input.pulsePerMM[1]) return Code::UNEQUAL_AXIS_SCALE;
    const double scale = input.pulsePerMM[0];
    const double nominalRadius = std::hypot(input.centerOffsetMM[0], input.centerOffsetMM[1]);
    if (!std::isfinite(nominalRadius) || nominalRadius <= 0.0) return Code::INVALID_RADIUS;

    // Small scalar arrays only: input and output remain caller-owned.
    double centerMM[2] = { 0.0, 0.0 }, centerPulse[2] = { 0.0, 0.0 };
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        const double offsetPulse = input.centerOffsetMM[axis] * scale;
        centerMM[axis] = input.startMCS[slots[axis]] + input.centerOffsetMM[axis];
        centerPulse[axis] = input.startPulse[slots[axis]] + offsetPulse;
        if (!std::isfinite(offsetPulse) || !std::isfinite(centerMM[axis]) ||
            !std::isfinite(centerPulse[axis])) return Code::NONFINITE_GEOMETRY;
        if (input.centerOffsetMM[axis] != 0.0 && offsetPulse == 0.0)
            return Code::SPACE_MISMATCH;
    }
    const double pulseRadius = std::hypot(input.startPulse[axes.u] - centerPulse[0],
        input.startPulse[axes.v] - centerPulse[1]);
    if (!std::isfinite(pulseRadius) || pulseRadius <= 0.0) return Code::INVALID_RADIUS;
    const double mmTolerance = CoordinateBudget(input.startMCS[axes.u], input.startMCS[axes.v],
        input.endMCS[axes.u], input.endMCS[axes.v], centerMM[0], centerMM[1], nominalRadius) +
        static_cast<double>(sourceRoundoffMM);
    const double pulseTolerance = CoordinateBudget(input.startPulse[axes.u], input.startPulse[axes.v],
        input.endPulse[axes.u], input.endPulse[axes.v], centerPulse[0], centerPulse[1], pulseRadius) +
        static_cast<double>(sourceRoundoffMM) * scale;
    const double budgetMM = mmTolerance + pulseTolerance / scale;
    // The consumer can reconstruct the pulse-space bound from its immutable
    // packet and native axis scale. Enforce that same cap before publication.
    const double consumerBudgetMM = 2.0 * (pulseTolerance / scale);
    if (!std::isfinite(budgetMM) || budgetMM > MaximumBudgetMM ||
        !std::isfinite(consumerBudgetMM) || consumerBudgetMM > MaximumBudgetMM)
        return Code::PRECISION_BUDGET;
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        const double deltaMM = input.endMCS[slots[axis]] - input.startMCS[slots[axis]];
        const double deltaPulseMM = (input.endPulse[slots[axis]] - input.startPulse[slots[axis]]) / scale;
        const double centerPulseMM = (centerPulse[axis] - input.startPulse[slots[axis]]) / scale;
        if (!std::isfinite(deltaMM) || !std::isfinite(deltaPulseMM) ||
            !std::isfinite(centerPulseMM)) return Code::NONFINITE_GEOMETRY;
        if (!Near(deltaMM, deltaPulseMM, budgetMM) ||
            !Near(centerPulseMM, input.centerOffsetMM[axis], budgetMM))
            return Code::SPACE_MISMATCH;
    }

    NCPathCoreArcPulseGeometry mmGeometry;
    NCPathCoreArcPulseGeometry pulseGeometry;
    if (!ResolveNCPathCorePlanarCirclePulse(input.startMCS[axes.u], input.startMCS[axes.v],
        input.endMCS[axes.u], input.endMCS[axes.v], centerMM[0], centerMM[1], nominalRadius,
        input.direction, input.fullCircle, mmGeometry, sourceRoundoffMM)) return Code::RADIUS_MISMATCH;
    if (!ResolveNCPathCorePlanarCirclePulse(input.startPulse[axes.u], input.startPulse[axes.v],
        input.endPulse[axes.u], input.endPulse[axes.v], centerPulse[0], centerPulse[1], pulseRadius,
        input.direction, input.fullCircle, pulseGeometry,
        static_cast<double>(sourceRoundoffMM) * scale)) return Code::RADIUS_MISMATCH;
    const double radiusMM = pulseGeometry.radius / scale;
    const double lengthMM = pulseGeometry.lengthPulse / scale;
    // Resolve angle wrapping consistently across spaces, including branch cuts.
    const double sweepErrorMM = std::fabs(mmGeometry.sweepRadians - pulseGeometry.sweepRadians) * radiusMM;
    if (!std::isfinite(radiusMM) || !std::isfinite(lengthMM) || radiusMM <= 0.0 || lengthMM <= 0.0)
        return Code::NONFINITE_GEOMETRY;
    if (!Near(radiusMM, nominalRadius, budgetMM) || !std::isfinite(sweepErrorMM) ||
        sweepErrorMM > budgetMM) return Code::SPACE_MISMATCH;
    if (pulseGeometry.lengthPulse < 1.0e-5) return Code::DEGENERATE_ARC;
    // The legacy fixed-radius consumer squares path length. Reject if that
    // intermediate cannot be represented, even when the length alone fits.
    const double squaredLength = pulseGeometry.lengthPulse * pulseGeometry.lengthPulse;
    if (!std::isfinite(squaredLength) || squaredLength == 0.0)
        return Code::NONFINITE_GEOMETRY;
    const double velocity = FeedVelocity(input.feedMMMin, scale);
    if (!std::isfinite(velocity) || velocity < 1.0) return Code::INVALID_GROUP_VELOCITY;
    if (velocity > input.maxVelocityPPS[0] || velocity > input.maxVelocityPPS[1])
        return Code::AXIS_VELOCITY_LIMIT;

    double lower[2] = { 0.0, 0.0 }, upper[2] = { 0.0, 0.0 };
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        lower[axis] = input.startMCS[slots[axis]] < input.endMCS[slots[axis]] ? input.startMCS[slots[axis]] : input.endMCS[slots[axis]];
        upper[axis] = input.startMCS[slots[axis]] > input.endMCS[slots[axis]] ? input.startMCS[slots[axis]] : input.endMCS[slots[axis]];
    }
    for (std::uint32_t cardinal = 0U; cardinal < 4U; ++cardinal)
    {
        const double angle = cardinal * (Pi / 2.0);
        if (!ContainsAngle(angle, PositiveAngle(pulseGeometry.startAngle),
            pulseGeometry.sweepRadians, input.fullCircle)) continue;
        const std::uint32_t axis = cardinal & 1U;
        const double value = centerMM[axis] + (cardinal < 2U ? radiusMM : -radiusMM);
        if (!std::isfinite(value)) return Code::NONFINITE_GEOMETRY;
        if (value < lower[axis]) lower[axis] = value;
        if (value > upper[axis]) upper[axis] = value;
    }
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        // Cover mm/pulse center rounding and libm roundoff. Bounds are
        // deliberately conservative rather than promising exact extrema bits.
        lower[axis] = std::nextafter(lower[axis] - budgetMM,
            -std::numeric_limits<double>::infinity());
        upper[axis] = std::nextafter(upper[axis] + budgetMM,
            std::numeric_limits<double>::infinity());
        if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis]))
            return Code::NONFINITE_GEOMETRY;
    }

    output.startMCS = input.startMCS; output.endMCS = input.endMCS;
    output.startPulse = input.startPulse; output.endPulse = input.endPulse;
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        output.centerMCS[axis] = centerMM[axis]; output.centerPulse[axis] = centerPulse[axis];
        output.boundsMinMCS[axis] = lower[axis]; output.boundsMaxMCS[axis] = upper[axis];
    }
    output.radiusMM = radiusMM; output.radiusPulse = pulseGeometry.radius;
    output.startAngle = pulseGeometry.startAngle; output.sweepRadians = pulseGeometry.sweepRadians;
    output.lengthMM = lengthMM; output.lengthPulse = pulseGeometry.lengthPulse;
    output.feedMMMin = input.feedMMMin; output.velocityPPS = velocity;
    output.direction = input.direction; output.axisMask = axes.mask;
    output.plane = input.plane;
    output.fullCircle = input.fullCircle; output.sourceRoundoffMM = sourceRoundoffMM;
    output.valid = true;
    return Code::BUILT_ARC;
}

bool EvaluateNCPathCoreFeedArcAxis(const NCPathCoreFeedArcV2& arc,
    const std::uint32_t axisIndex, const double unitParameter,
    double& outputMCS) noexcept
{
    outputMCS = 0.0;
    NCArcPlaneAxes axes{};
    if (!TryGetNCArcPlaneAxes(arc.plane, axes)) return false;
    if (!arc.valid || !std::isfinite(arc.sourceRoundoffMM) ||
        arc.sourceRoundoffMM < 0.0f || std::signbit(arc.sourceRoundoffMM) ||
        arc.sourceRoundoffMM > MaximumBudgetMM || arc.axisMask != axes.mask || axisIndex >= 8U ||
        (arc.direction != -1 && arc.direction != 1) ||
        !std::isfinite(unitParameter) || unitParameter < 0.0 || unitParameter > 1.0)
        return false;
    const double start = arc.startMCS[axisIndex], end = arc.endMCS[axisIndex];
    if (!std::isfinite(start) || !std::isfinite(end)) return false;
    if (unitParameter == 0.0) { outputMCS = start; return true; }
    if (unitParameter == 1.0) { outputMCS = end; return true; }
    if (axisIndex != axes.u && axisIndex != axes.v)
    {
        if (!SameBits(start, end)) return false;
        outputMCS = start; return true;
    }
    const unsigned component = axisIndex == axes.u ? 0U : 1U;
    if (!std::isfinite(arc.centerMCS[component]) || !std::isfinite(arc.radiusMM) ||
        arc.radiusMM <= 0.0 || !std::isfinite(arc.startAngle) ||
        !std::isfinite(arc.sweepRadians) || arc.sweepRadians == 0.0) return false;
    const double angle = arc.startAngle + unitParameter * arc.sweepRadians;
    const double coordinate = arc.centerMCS[component] + arc.radiusMM *
        (component == 0U ? std::cos(angle) : std::sin(angle));
    if (!std::isfinite(coordinate)) return false;
    outputMCS = coordinate;
    return true;
}
