#include "NCPathCoreRetainedPath.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
    constexpr double EpsilonBudget = 64.0 * std::numeric_limits<double>::epsilon();
    constexpr double MaximumSeamMM = 1.0e-7;
    constexpr double Pi = 3.141592653589793238462643383279502884;
    constexpr double TwoPi = 2.0 * Pi;
    bool SameBits(double a, double b) noexcept
    {
        return std::memcmp(&a, &b, sizeof(double)) == 0;
    }
    double Maximum(double a, double b) noexcept { return a > b ? a : b; }
    double Magnitude(const NCPathCoreRetainedGeometry& g, std::uint32_t axis, bool pulse) noexcept
    {
        double scale = Maximum(std::fabs(pulse ? g.startPulse[axis] : g.startMCS[axis]),
            std::fabs(pulse ? g.endPulse[axis] : g.endMCS[axis]));
        if ((g.kind == NCPathCoreRetainedKind::ARC || g.kind == NCPathCoreRetainedKind::LINE_ARC) && axis < 2U)
        {
            scale = Maximum(scale, std::fabs(pulse ? g.centerPulse[axis] : g.centerMCS[axis]));
            scale = Maximum(scale, pulse ? g.radiusPulse : g.radiusMM);
        }
        return scale;
    }
    bool Near(double a, double b, double magnitude) noexcept
    {
        const double difference = a - b;
        return std::isfinite(difference) && std::fabs(difference) <= EpsilonBudget * magnitude;
    }
    bool ContainsAngle(double angle, double start, double sweep, bool full) noexcept
    {
        if (full) return true;
        double travel = sweep > 0.0 ? angle - start : start - angle;
        if (travel < 0.0) travel += TwoPi;
        if (travel >= TwoPi) travel -= TwoPi;
        return travel <= std::fabs(sweep) + EpsilonBudget * TwoPi;
    }
    bool CoordinatePairClose(double aMM, double bMM, double aPulse, double bPulse,
        double mmMagnitude, double pulseMagnitude, double pulsePerMM) noexcept
    {
        if (!std::isfinite(aMM) || !std::isfinite(bMM) || !std::isfinite(aPulse) ||
            !std::isfinite(bPulse) || !std::isfinite(pulsePerMM) || pulsePerMM <= 0.0)
            return false;
        const double mmDifference = std::fabs(aMM - bMM);
        const double pulseDifference = std::fabs(aPulse - bPulse);
        if (!std::isfinite(mmDifference) || !std::isfinite(pulseDifference)) return false;
        // Cap each half of the combined budget. Division happens before the
        // comparison, avoiding overflow in MaximumSeamMM * pulsePerMM.
        double mmBudget = EpsilonBudget * mmMagnitude;
        double pulseBudgetMM = (EpsilonBudget * pulseMagnitude) / pulsePerMM;
        if (!std::isfinite(mmBudget) || !std::isfinite(pulseBudgetMM)) return false;
        if (mmBudget > MaximumSeamMM * 0.5) mmBudget = MaximumSeamMM * 0.5;
        if (pulseBudgetMM > MaximumSeamMM * 0.5) pulseBudgetMM = MaximumSeamMM * 0.5;
        const double pulseDifferenceMM = pulseDifference / pulsePerMM;
        return std::isfinite(pulseDifferenceMM) && mmDifference <= mmBudget &&
            pulseDifferenceMM <= pulseBudgetMM;
    }
    bool Evaluate(const NCPathCoreRetainedGeometry& g, std::uint32_t axis,
        double u, bool pulse, double& output) noexcept
    {
        output = 0.0;
        if (!g.valid || axis >= 8U || !std::isfinite(u) || u < 0.0 || u > 1.0 ||
            (g.kind != NCPathCoreRetainedKind::LINE && g.kind != NCPathCoreRetainedKind::ARC &&
                g.kind != NCPathCoreRetainedKind::LINE_ARC))
            return false;
        const double start = pulse ? g.startPulse[axis] : g.startMCS[axis];
        const double end = pulse ? g.endPulse[axis] : g.endMCS[axis];
        if (!std::isfinite(start) || !std::isfinite(end)) return false;
        if (u == 0.0) { output = start; return true; }
        if (u == 1.0) { output = end; return true; }
        if ((g.axisMask & (1U << axis)) == 0U)
        {
            if (!SameBits(start, end)) return false;
            output = start;
            return true;
        }
        double value = 0.0;
        if (g.kind == NCPathCoreRetainedKind::LINE_ARC)
        {
            if (axis > 1U) return false;
            const double radius = pulse ? g.radiusPulse : g.radiusMM;
            const double total = pulse ? g.lengthPulse : g.lengthMM;
            const double arcLength = radius * std::fabs(g.sweepRadians);
            const double prefix = total - arcLength;
            if (!std::isfinite(total) || !std::isfinite(prefix) || prefix <= 0.0 || radius <= 0.0) return false;
            const double center = pulse ? g.centerPulse[axis] : g.centerMCS[axis];
            const double entry = center + radius * (axis == 0U ? std::cos(g.startAngle) : std::sin(g.startAngle));
            const double distance = u * total;
            if (distance <= prefix) value = start + (entry - start) * (distance / prefix);
            else
            {
                const double angle = g.startAngle + (distance - prefix) / arcLength * g.sweepRadians;
                value = center + radius * (axis == 0U ? std::cos(angle) : std::sin(angle));
            }
        }
        else if (g.kind == NCPathCoreRetainedKind::LINE)
        {
            const double delta = end - start;
            if (!std::isfinite(delta)) return false;
            value = start + u * delta;
        }
        else
        {
            if (axis >= 2U || !std::isfinite(g.startAngle) || !std::isfinite(g.sweepRadians))
                return false;
            const double radius = pulse ? g.radiusPulse : g.radiusMM;
            const double center = pulse ? g.centerPulse[axis] : g.centerMCS[axis];
            if (!std::isfinite(radius) || radius <= 0.0 || !std::isfinite(center)) return false;
            const double angle = g.startAngle + u * g.sweepRadians;
            value = center + radius * (axis == 0U ? std::cos(angle) : std::sin(angle));
        }
        if (!std::isfinite(value)) return false;
        output = value;
        return true;
    }
}

void NCPathCoreRetainedGeometry::Clear() noexcept
{
    startMCS.fill(0.0); endMCS.fill(0.0); startPulse.fill(0.0); endPulse.fill(0.0);
    centerMCS.fill(0.0); centerPulse.fill(0.0);
    boundsMinMCS.fill(0.0); boundsMaxMCS.fill(0.0);
    radiusMM = radiusPulse = startAngle = sweepRadians = lengthMM = lengthPulse = 0.0;
    axisMask = 0U; direction = 0; kind = NCPathCoreRetainedKind::NONE;
    fullCircle = point = valid = false;
}

static bool ValidateRetainedGeometry(const NCPathCoreRetainedGeometry& g,
    bool generatedArc) noexcept
{
    if (!g.valid || g.axisMask == 0U || (g.axisMask & ~7U) != 0U ||
        !std::isfinite(g.lengthMM) || !std::isfinite(g.lengthPulse) ||
        g.lengthMM < 0.0 || g.lengthPulse < 0.0) return false;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(g.startMCS[axis]) || !std::isfinite(g.endMCS[axis]) ||
            !std::isfinite(g.startPulse[axis]) || !std::isfinite(g.endPulse[axis])) return false;
        if ((g.axisMask & (1U << axis)) == 0U &&
            (!SameBits(g.startMCS[axis], g.endMCS[axis]) ||
                !SameBits(g.startPulse[axis], g.endPulse[axis]))) return false;
    }
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        if (!std::isfinite(g.boundsMinMCS[axis]) || !std::isfinite(g.boundsMaxMCS[axis]) ||
            g.boundsMinMCS[axis] > g.boundsMaxMCS[axis] ||
            g.startMCS[axis] < g.boundsMinMCS[axis] || g.startMCS[axis] > g.boundsMaxMCS[axis] ||
            g.endMCS[axis] < g.boundsMinMCS[axis] || g.endMCS[axis] > g.boundsMaxMCS[axis]) return false;
    }
    if (g.kind == NCPathCoreRetainedKind::LINE_ARC)
    {
        if (g.axisMask != 3U || g.point || g.fullCircle || !std::isfinite(g.radiusMM) ||
            !std::isfinite(g.radiusPulse) || g.radiusMM <= 0.0 || g.radiusPulse <= 0.0 ||
            !std::isfinite(g.sweepRadians) || std::fabs(g.sweepRadians) < 0.0872664625997164 ||
            std::fabs(g.sweepRadians) > 2.356194490192345) return false;
        NCPathCoreRetainedGeometry arc = g; // bounded 384-byte validation scratch
        arc.kind = NCPathCoreRetainedKind::ARC;
        arc.lengthMM = g.radiusMM * std::fabs(g.sweepRadians);
        arc.lengthPulse = g.radiusPulse * std::fabs(g.sweepRadians);
        double lm = 0.0, lp = 0.0;
        for (unsigned i = 0U; i < 2U; ++i)
        {
            const double trig = i == 0U ? std::cos(g.startAngle) : std::sin(g.startAngle);
            arc.startMCS[i] = g.centerMCS[i] + g.radiusMM * trig;
            arc.startPulse[i] = g.centerPulse[i] + g.radiusPulse * trig;
            lm = std::hypot(lm, arc.startMCS[i] - g.startMCS[i]);
            lp = std::hypot(lp, arc.startPulse[i] - g.startPulse[i]);
        }
        if (!std::isfinite(lm) || !std::isfinite(lp) || lm <= 0.0 || lp <= 0.0 ||
            !Near(lm + arc.lengthMM, g.lengthMM, Maximum(g.lengthMM, 1.0)) ||
            !Near(lp + arc.lengthPulse, g.lengthPulse, Maximum(g.lengthPulse, 1.0)) ||
            !ValidateRetainedGeometry(arc, true)) return false;
        const double tx = -double(g.direction) * std::sin(g.startAngle);
        const double ty = double(g.direction) * std::cos(g.startAngle);
        return std::fabs((arc.startMCS[0] - g.startMCS[0]) / lm - tx) <= 1e-11 &&
            std::fabs((arc.startMCS[1] - g.startMCS[1]) / lm - ty) <= 1e-11 &&
            std::fabs((arc.startPulse[0] - g.startPulse[0]) / lp - tx) <= 1e-11 &&
            std::fabs((arc.startPulse[1] - g.startPulse[1]) / lp - ty) <= 1e-11;
    }
    if (g.kind == NCPathCoreRetainedKind::LINE)
    {
        if (g.direction != 0 || g.fullCircle || g.radiusMM != 0.0 || g.radiusPulse != 0.0 ||
            g.startAngle != 0.0 || g.sweepRadians != 0.0 ||
            g.centerMCS[0] != 0.0 || g.centerMCS[1] != 0.0 ||
            g.centerPulse[0] != 0.0 || g.centerPulse[1] != 0.0) return false;
        double mmLength = 0.0, pulseLength = 0.0;
        for (std::uint32_t axis = 0U; axis < 3U; ++axis)
        {
            const double dm = g.endMCS[axis] - g.startMCS[axis];
            const double dp = g.endPulse[axis] - g.startPulse[axis];
            if (!std::isfinite(dm) || !std::isfinite(dp) || (dm == 0.0) != (dp == 0.0) ||
                (dm < 0.0) != (dp < 0.0)) return false;
            mmLength = std::hypot(mmLength, dm);
            pulseLength = std::hypot(pulseLength, dp);
        }
        return std::isfinite(mmLength) && std::isfinite(pulseLength) &&
            g.point == (mmLength == 0.0) && g.point == (pulseLength == 0.0) &&
            Near(mmLength, g.lengthMM, Maximum(mmLength, g.lengthMM)) &&
            Near(pulseLength, g.lengthPulse, Maximum(pulseLength, g.lengthPulse));
    }
    if (g.kind != NCPathCoreRetainedKind::ARC || g.axisMask != 3U || g.point ||
        (g.direction != -1 && g.direction != 1) ||
        !std::isfinite(g.radiusMM) || !std::isfinite(g.radiusPulse) ||
        !std::isfinite(g.startAngle) || !std::isfinite(g.sweepRadians) ||
        g.radiusMM <= 0.0 || g.radiusPulse <= 0.0 || g.lengthMM <= 0.0 || g.lengthPulse <= 0.0 ||
        g.startAngle < -Pi || g.startAngle > Pi ||
        (g.direction == 1) != (g.sweepRadians > 0.0) ||
        (g.fullCircle ? std::fabs(g.sweepRadians) != TwoPi :
            (std::fabs(g.sweepRadians) < 1.0e-12 || std::fabs(g.sweepRadians) >= TwoPi))) return false;
    if (!g.fullCircle && ((g.startPulse[0] == g.endPulse[0] && g.startPulse[1] == g.endPulse[1]) ||
        (g.startMCS[0] == g.endMCS[0] && g.startMCS[1] == g.endMCS[1]))) return false;
    // DJ_FIX1: ONLY Q-generated arc scratch uses a two-coordinate roundoff
    // budget. Its radius/angle come from both XY pulse coordinates; near-zero
    // X still carries subtraction roundoff from a large Y center, and vice versa.
    // Ordinary ARC retention/replay/PathHold keeps its original per-axis checks.
    // Neither branch changes Q, any stored geometry, or source-to-source seams.
    double pulsePerMM = 1.0, reconstructionBudgetMM = 0.0;
    if (generatedArc)
    {
        pulsePerMM = g.radiusPulse / g.radiusMM;
        if (!std::isfinite(pulsePerMM) || pulsePerMM <= 0.0) return false;
        const double mmMagnitude = Maximum(Magnitude(g, 0U, false), Magnitude(g, 1U, false));
        const double pulseMagnitude = Maximum(Magnitude(g, 0U, true), Magnitude(g, 1U, true));
        reconstructionBudgetMM = EpsilonBudget * mmMagnitude +
            (EpsilonBudget * pulseMagnitude) / pulsePerMM;
        if (!std::isfinite(reconstructionBudgetMM) || reconstructionBudgetMM > 1.0e-7)
            return false;
    }
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        if (!std::isfinite(g.centerMCS[axis]) || !std::isfinite(g.centerPulse[axis]) ||
            (g.fullCircle && (!SameBits(g.startMCS[axis], g.endMCS[axis]) ||
                !SameBits(g.startPulse[axis], g.endPulse[axis])))) return false;
        const double trigStart = axis == 0U ? std::cos(g.startAngle) : std::sin(g.startAngle);
        const double endAngle = g.startAngle + g.sweepRadians;
        const double trigEnd = axis == 0U ? std::cos(endAngle) : std::sin(endAngle);
        if (!generatedArc)
        {
            if (!Near(g.centerMCS[axis] + g.radiusMM * trigStart, g.startMCS[axis], Magnitude(g, axis, false)) ||
                !Near(g.centerMCS[axis] + g.radiusMM * trigEnd, g.endMCS[axis], Magnitude(g, axis, false)) ||
                !Near(g.centerPulse[axis] + g.radiusPulse * trigStart, g.startPulse[axis], Magnitude(g, axis, true)) ||
                !Near(g.centerPulse[axis] + g.radiusPulse * trigEnd, g.endPulse[axis], Magnitude(g, axis, true))) return false;
        }
        else
        {
            const double startMMError = (g.centerMCS[axis] + g.radiusMM * trigStart) - g.startMCS[axis];
            const double endMMError = (g.centerMCS[axis] + g.radiusMM * trigEnd) - g.endMCS[axis];
            const double startPulseErrorMM = ((g.centerPulse[axis] + g.radiusPulse * trigStart) - g.startPulse[axis]) / pulsePerMM;
            const double endPulseErrorMM = ((g.centerPulse[axis] + g.radiusPulse * trigEnd) - g.endPulse[axis]) / pulsePerMM;
            if (!std::isfinite(startMMError) || !std::isfinite(endMMError) ||
                !std::isfinite(startPulseErrorMM) || !std::isfinite(endPulseErrorMM) ||
                std::fabs(startMMError) > reconstructionBudgetMM || std::fabs(endMMError) > reconstructionBudgetMM ||
                std::fabs(startPulseErrorMM) > reconstructionBudgetMM || std::fabs(endPulseErrorMM) > reconstructionBudgetMM)
                return false;
        }
        // Check exact cardinal extrema independently of the source's bounds.
        const double maximumAngle = axis == 0U ? 0.0 : Pi * 0.5;
        const double minimumAngle = axis == 0U ? Pi : -Pi * 0.5;
        if ((ContainsAngle(maximumAngle, g.startAngle, g.sweepRadians, g.fullCircle) &&
            g.centerMCS[axis] + g.radiusMM > g.boundsMaxMCS[axis]) ||
            (ContainsAngle(minimumAngle, g.startAngle, g.sweepRadians, g.fullCircle) &&
                g.centerMCS[axis] - g.radiusMM < g.boundsMinMCS[axis])) return false;
    }
    const double lengthMM = g.radiusMM * std::fabs(g.sweepRadians);
    const double lengthPulse = g.radiusPulse * std::fabs(g.sweepRadians);
    return std::isfinite(lengthMM) && std::isfinite(lengthPulse) &&
        Near(lengthMM, g.lengthMM, Maximum(lengthMM, g.lengthMM)) &&
        Near(lengthPulse, g.lengthPulse, Maximum(lengthPulse, g.lengthPulse));
}

bool IsNCPathCoreRetainedGeometryValid(const NCPathCoreRetainedGeometry& g) noexcept
{
    return ValidateRetainedGeometry(g, false);
}

bool BuildNCPathCoreRetainedLine(const NCPathCoreFeedLineV2& source,
    NCPathCoreRetainedGeometry& output) noexcept
{
    output.Clear();
    if (!source.valid) return false;
    output.startMCS = source.startMCS; output.endMCS = source.endMCS;
    output.startPulse = source.startPulse; output.endPulse = source.endPulse;
    output.lengthMM = source.lengthMM; output.lengthPulse = source.lengthPulse;
    output.axisMask = source.axisMask; output.point = source.point;
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        output.boundsMinMCS[axis] = source.startMCS[axis] < source.endMCS[axis] ?
            source.startMCS[axis] : source.endMCS[axis];
        output.boundsMaxMCS[axis] = Maximum(source.startMCS[axis], source.endMCS[axis]);
    }
    output.kind = NCPathCoreRetainedKind::LINE; output.valid = true;
    if (IsNCPathCoreRetainedGeometryValid(output)) return true;
    output.Clear(); return false;
}

static bool BuildRetainedArc(const NCPathCoreFeedArcV2& source,
    NCPathCoreRetainedGeometry& output, bool generatedArc) noexcept
{
    output.Clear();
    if (!source.valid) return false;
    output.startMCS = source.startMCS; output.endMCS = source.endMCS;
    output.startPulse = source.startPulse; output.endPulse = source.endPulse;
    output.centerMCS = source.centerMCS; output.centerPulse = source.centerPulse;
    output.boundsMinMCS = source.boundsMinMCS; output.boundsMaxMCS = source.boundsMaxMCS;
    output.radiusMM = source.radiusMM; output.radiusPulse = source.radiusPulse;
    output.startAngle = source.startAngle; output.sweepRadians = source.sweepRadians;
    output.lengthMM = source.lengthMM; output.lengthPulse = source.lengthPulse;
    output.axisMask = source.axisMask; output.direction = source.direction;
    output.fullCircle = source.fullCircle; output.kind = NCPathCoreRetainedKind::ARC; output.valid = true;
    if (ValidateRetainedGeometry(output, generatedArc)) return true;
    output.Clear(); return false;
}

bool BuildNCPathCoreRetainedArc(const NCPathCoreFeedArcV2& source,
    NCPathCoreRetainedGeometry& output) noexcept
{
    return BuildRetainedArc(source, output, false);
}

bool EvaluateNCPathCoreRetainedAxisCanonical(const NCPathCoreRetainedGeometry& g,
    std::uint32_t axis, double u, double& output) noexcept {
    return Evaluate(g, axis, u, false, output);
}
bool EvaluateNCPathCoreRetainedPulseCanonical(const NCPathCoreRetainedGeometry& g,
    std::uint32_t axis, double u, double& output) noexcept {
    return Evaluate(g, axis, u, true, output);
}

bool AreNCPathCoreRetainedEndpointsConnected(const NCPathCoreRetainedGeometry& previous,
    const NCPathCoreRetainedGeometry& next, std::uint32_t validMask,
    const std::array<double, 8U>& scales) noexcept
{
    if (!previous.valid || !next.valid || validMask == 0U || (validMask & ~255U) != 0U ||
        ((previous.axisMask | next.axisMask) & validMask) != (previous.axisMask | next.axisMask)) return false;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        if ((validMask & (1U << axis)) == 0U)
        {
            if (!SameBits(previous.endMCS[axis], next.startMCS[axis]) ||
                !SameBits(previous.endPulse[axis], next.startPulse[axis])) return false;
            continue;
        }
        if (!CoordinatePairClose(previous.endMCS[axis], next.startMCS[axis],
            previous.endPulse[axis], next.startPulse[axis],
            Maximum(Magnitude(previous, axis, false), Magnitude(next, axis, false)),
            Maximum(Magnitude(previous, axis, true), Magnitude(next, axis, true)), scales[axis])) return false;
    }
    return true;
}

bool DoesNCPathCoreRetainedStartMatch(const NCPathCoreRetainedGeometry& g,
    bool reverse, const std::array<double, 8U>& actualMCS,
    const std::array<double, 8U>& actualPulse, std::uint32_t validMask,
    const std::array<double, 8U>& scales) noexcept
{
    return DoesNCPathCoreRetainedStartMatchAt(g, reverse ? 1.0 : 0.0,
        actualMCS, actualPulse, validMask, scales);
}

bool DoesNCPathCoreRetainedStartMatchAt(const NCPathCoreRetainedGeometry& g,
    double u, const std::array<double, 8U>& actualMCS,
    const std::array<double, 8U>& actualPulse, std::uint32_t validMask,
    const std::array<double, 8U>& scales) noexcept
{
    if (!g.valid || !std::isfinite(u) || u < 0.0 || u > 1.0 ||
        validMask == 0U || (validMask & ~255U) != 0U ||
        (g.axisMask & validMask) != g.axisMask) return false;
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
    {
        double expectedMM = 0.0, expectedPulse = 0.0;
        if (!EvaluateNCPathCoreRetainedAxisCanonical(g, axis, u, expectedMM) ||
            !EvaluateNCPathCoreRetainedPulseCanonical(g, axis, u, expectedPulse)) return false;
        if ((validMask & (1U << axis)) == 0U)
        {
            if (!SameBits(expectedMM, actualMCS[axis]) || !SameBits(expectedPulse, actualPulse[axis])) return false;
            continue;
        }
        if (!CoordinatePairClose(expectedMM, actualMCS[axis], expectedPulse, actualPulse[axis],
            Magnitude(g, axis, false), Magnitude(g, axis, true), scales[axis])) return false;
    }
    return true;
}

void NCPathCoreRetainedPath::Clear(NCPathCoreRetainedFault reason) noexcept
{
    m_count = m_position = m_lower = m_requested = m_validMask = 0U;
    m_fault = reason; m_state = NCPathCoreRetainedCursorState::IDLE;
    m_selected = m_pending = m_forward = m_distanceMode = false;
    m_currentOrdinal = m_selectedOrdinal = 0U;
    m_currentU = m_selectedStartU = m_selectedEndU = 0.0;
}
void NCPathCoreRetainedPath::Fail(NCPathCoreRetainedFault reason) noexcept
{
    m_fault = reason == NCPathCoreRetainedFault::NONE ? NCPathCoreRetainedFault::INTERRUPTED : reason;
    m_selected = m_pending = false;
}
bool NCPathCoreRetainedPath::Append(const NCPathCoreRetainedGeometry& g,
    std::uint32_t validMask, const std::array<double, 8U>& scales) noexcept
{
    if (m_fault != NCPathCoreRetainedFault::NONE) return false;
    if (m_state != NCPathCoreRetainedCursorState::IDLE || m_selected || m_pending)
    {
        Fail(NCPathCoreRetainedFault::INTERRUPTED); return false;
    }
    if (m_count == Capacity) { Fail(NCPathCoreRetainedFault::CAPACITY); return false; }
    if (!IsNCPathCoreRetainedGeometryValid(g) || validMask == 0U || (validMask & ~255U) != 0U ||
        (g.axisMask & validMask) != g.axisMask)
    {
        Fail(NCPathCoreRetainedFault::INVALID_GEOMETRY); return false;
    }
    for (std::uint32_t axis = 0U; axis < 8U; ++axis)
        if ((validMask & (1U << axis)) != 0U && (!std::isfinite(scales[axis]) || scales[axis] <= 0.0))
        {
            Fail(NCPathCoreRetainedFault::INVALID_GEOMETRY); return false;
        }
    if (m_count != 0U && (validMask != m_validMask ||
        !AreNCPathCoreRetainedEndpointsConnected(m_rows[m_count - 1U], g, validMask, scales)))
    {
        Fail(NCPathCoreRetainedFault::DISCONTINUITY); return false;
    }
    m_rows[m_count] = g; ++m_count; m_validMask = validMask; m_position = m_count;
    return true;
}
const NCPathCoreRetainedGeometry* NCPathCoreRetainedPath::Get(std::uint32_t index) const noexcept
{
    return m_fault == NCPathCoreRetainedFault::NONE && index < m_count ? &m_rows[index] : nullptr;
}
bool NCPathCoreRetainedPath::BeginRetreat(std::uint32_t requested) noexcept
{
    // DH compound rows are retained/evaluable, but not admitted to legacy
    // replay/EDM traversal until that producer supports the compound metric.
    for (std::uint32_t i = 0U; i < m_count; ++i)
        if (m_rows[i].kind == NCPathCoreRetainedKind::LINE_ARC) return false;

    if (m_fault != NCPathCoreRetainedFault::NONE || m_pending || m_selected || requested == 0U ||
        requested > m_count || (m_state != NCPathCoreRetainedCursorState::IDLE &&
            m_state != NCPathCoreRetainedCursorState::RETURNED)) return false;
    m_requested = requested; m_lower = m_count - requested; m_position = m_count;
    m_state = NCPathCoreRetainedCursorState::RETREATING; m_forward = false;
    m_distanceMode = false;
    m_currentOrdinal = m_selectedOrdinal = 0U;
    m_currentU = m_selectedStartU = m_selectedEndU = 0.0;
    return true;
}
bool NCPathCoreRetainedPath::SelectStep(bool forward,
    const NCPathCoreRetainedGeometry*& geometry, std::uint32_t& ordinal) noexcept
{
    geometry = nullptr; ordinal = 0U;
    if (m_fault != NCPathCoreRetainedFault::NONE || m_pending || m_selected || m_distanceMode) return false;
    if (forward)
    {
        if (m_state == NCPathCoreRetainedCursorState::AT_START)
            m_state = NCPathCoreRetainedCursorState::ADVANCING;
        if (m_state != NCPathCoreRetainedCursorState::ADVANCING || m_position >= m_count) return false;
        ordinal = m_position + 1U;
    }
    else
    {
        if (m_state != NCPathCoreRetainedCursorState::RETREATING || m_position <= m_lower) return false;
        ordinal = m_position;
    }
    geometry = &m_rows[ordinal - 1U]; m_selected = true; m_forward = forward;
    return true;
}
bool NCPathCoreRetainedPath::BeginDistanceRetreat(std::uint32_t requested) noexcept
{
    // DH compound rows are retained/evaluable, but not admitted to legacy
    // replay/EDM traversal until that producer supports the compound metric.
    for (std::uint32_t i = 0U; i < m_count; ++i)
        if (m_rows[i].kind == NCPathCoreRetainedKind::LINE_ARC) return false;

    if (m_fault != NCPathCoreRetainedFault::NONE || m_pending || m_selected ||
        requested == 0U || requested > m_count ||
        (m_state != NCPathCoreRetainedCursorState::IDLE &&
            m_state != NCPathCoreRetainedCursorState::RETURNED)) return false;
    const std::uint32_t lower = m_count - requested;
    for (std::uint32_t index = lower; index < m_count; ++index)
    {
        const NCPathCoreRetainedGeometry& g = m_rows[index];
        if (!g.valid || g.point || !std::isfinite(g.lengthMM) || g.lengthMM <= 0.0 ||
            !std::isfinite(g.lengthPulse) || g.lengthPulse <= 0.0) return false;
    }
    m_requested = requested; m_lower = lower; m_position = m_count;
    m_currentOrdinal = m_count; m_currentU = 1.0;
    m_selectedOrdinal = 0U; m_selectedStartU = m_selectedEndU = 0.0;
    m_state = NCPathCoreRetainedCursorState::RETREATING;
    m_forward = false; m_distanceMode = true;
    return true;
}

bool NCPathCoreRetainedPath::SelectDistanceStep(bool forward, double maxDistanceMM,
    const NCPathCoreRetainedGeometry*& geometry, std::uint32_t& ordinal,
    double& startU, double& endU) noexcept
{
    geometry = nullptr; ordinal = 0U; startU = endU = 0.0;
    if (!m_distanceMode || m_fault != NCPathCoreRetainedFault::NONE || m_pending || m_selected ||
        !std::isfinite(maxDistanceMM) || maxDistanceMM <= 0.0 ||
        m_currentOrdinal <= m_lower || m_currentOrdinal > m_count ||
        !std::isfinite(m_currentU) || m_currentU < 0.0 || m_currentU > 1.0) return false;
    std::uint32_t candidateOrdinal = m_currentOrdinal;
    double candidateStart = m_currentU;
    if (forward && candidateStart == 1.0)
    {
        if (candidateOrdinal == m_count) return false;
        ++candidateOrdinal; candidateStart = 0.0;
    }
    else if (!forward && candidateStart == 0.0)
    {
        if (candidateOrdinal == m_lower + 1U) return false;
        --candidateOrdinal; candidateStart = 1.0;
    }
    const NCPathCoreRetainedGeometry& g = m_rows[candidateOrdinal - 1U];
    if (!g.valid || g.point || !std::isfinite(g.lengthMM) || g.lengthMM <= 0.0 ||
        !std::isfinite(g.lengthPulse) || g.lengthPulse <= 0.0) return false;
    const double remainingU = forward ? 1.0 - candidateStart : candidateStart;
    const double remainingMM = remainingU * g.lengthMM;
    if (!std::isfinite(remainingMM) || remainingMM <= 0.0) return false;
    const double candidateEnd = maxDistanceMM >= remainingMM ? (forward ? 1.0 : 0.0) :
        (forward ? candidateStart + maxDistanceMM / g.lengthMM : candidateStart - maxDistanceMM / g.lengthMM);
    const double span = std::fabs(candidateEnd - candidateStart);
    const double distanceMM = span * g.lengthMM;
    const double distancePulse = span * g.lengthPulse;
    if (!std::isfinite(candidateEnd) || candidateEnd < 0.0 || candidateEnd > 1.0 ||
        (forward ? candidateEnd <= candidateStart : candidateEnd >= candidateStart) ||
        !std::isfinite(distanceMM) || distanceMM <= 0.0 ||
        !std::isfinite(distancePulse) || distancePulse <= 0.0) return false;
    // Select only pending state. The live cursor and phase remain committed
    // until the caller confirms this exact submission's completion.
    m_selectedOrdinal = candidateOrdinal;
    m_selectedStartU = candidateStart; m_selectedEndU = candidateEnd;
    m_forward = forward; m_selected = true;
    geometry = &g; ordinal = candidateOrdinal; startU = candidateStart; endU = candidateEnd;
    return true;
}

bool NCPathCoreRetainedPath::MarkSubmitted() noexcept
{
    if (m_fault != NCPathCoreRetainedFault::NONE || !m_selected || m_pending) return false;
    m_pending = true; m_selected = false; return true;
}
bool NCPathCoreRetainedPath::CompleteStep() noexcept
{
    if (m_fault != NCPathCoreRetainedFault::NONE || !m_pending) return false;
    if (m_distanceMode)
    {
        if (m_selectedOrdinal <= m_lower || m_selectedOrdinal > m_count ||
            !std::isfinite(m_selectedStartU) || !std::isfinite(m_selectedEndU) ||
            m_selectedStartU < 0.0 || m_selectedStartU > 1.0 ||
            m_selectedEndU < 0.0 || m_selectedEndU > 1.0 ||
            (m_forward ? m_selectedEndU <= m_selectedStartU : m_selectedEndU >= m_selectedStartU)) return false;
        m_currentOrdinal = m_selectedOrdinal; m_currentU = m_selectedEndU;
        m_position = m_currentU == 0.0 ? m_currentOrdinal - 1U : m_currentOrdinal;
        m_state = m_currentOrdinal == m_lower + 1U && m_currentU == 0.0 ?
            NCPathCoreRetainedCursorState::AT_START :
            m_currentOrdinal == m_count && m_currentU == 1.0 ?
            NCPathCoreRetainedCursorState::RETURNED :
            m_forward ? NCPathCoreRetainedCursorState::ADVANCING : NCPathCoreRetainedCursorState::RETREATING;
        m_pending = false;
        return true;
    }
    if (m_forward)
    {
        if (m_state != NCPathCoreRetainedCursorState::ADVANCING || m_position >= m_count) return false;
        ++m_position;
        if (m_position == m_count) m_state = NCPathCoreRetainedCursorState::RETURNED;
    }
    else
    {
        if (m_state != NCPathCoreRetainedCursorState::RETREATING || m_position <= m_lower) return false;
        --m_position;
        if (m_position == m_lower) m_state = NCPathCoreRetainedCursorState::AT_START;
    }
    m_pending = false; return true;
}

// Local polyline fillet: theta is the change in direction. The vertex-to-arc
// distance d*tan(theta/4) bounds BOTH sides of the local Hausdorff distance.
bool BuildNCPathCoreCornerBlend(NCPathCoreFeedLineInput& input,
    const std::array<double, 8U>& next, double tolerance,
    const std::array<double, 2U>& ppm, NCPathCoreFeedLineV2& prefix,
    NCPathCoreFeedArcInput& ai, NCPathCoreFeedArcV2& arc,
    NCPathCoreRetainedGeometry& out, NCPathCoreCornerMetadata& meta) noexcept
{
    out.Clear(); meta = NCPathCoreCornerMetadata{}; arc.Clear(); prefix.Clear();
    if (input.axisMask != 3U || !std::isfinite(tolerance) || tolerance < 0.0001 || tolerance > 1.0 ||
        !std::isfinite(ppm[0]) || ppm[0] <= 0.0 || ppm[0] != ppm[1]) return false;
    for (unsigned i = 0U; i < 8U; ++i)
        if (!std::isfinite(input.startMCS[i]) || !std::isfinite(input.endMCS[i]) ||
            !std::isfinite(input.startPulse[i]) || !std::isfinite(input.endPulse[i]) || !std::isfinite(next[i]) ||
            (i > 1U && (input.startMCS[i] != input.endMCS[i] || input.startMCS[i] != next[i]))) return false;
    const double ax = input.startMCS[0], ay = input.startMCS[1];
    const double bx = input.endMCS[0], by = input.endMCS[1], cx = next[0], cy = next[1];
    if (std::fabs(ax) > 1e6 || std::fabs(ay) > 1e6 || std::fabs(bx) > 1e6 ||
        std::fabs(by) > 1e6 || std::fabs(cx) > 1e6 || std::fabs(cy) > 1e6) return false;
    const double a = std::hypot(bx - ax, by - ay), b = std::hypot(cx - bx, cy - by);
    if (!std::isfinite(a) || !std::isfinite(b) || a <= 1e-6 || b <= 1e-6) return false;
    const double ux = (bx - ax) / a, uy = (by - ay) / a, vx = (cx - bx) / b, vy = (cy - by) / b;
    const double cross = ux * vy - uy * vx, dot = ux * vx + uy * vy;
    const double theta = std::atan2(std::fabs(cross), dot);
    if (!std::isfinite(theta) || theta < 0.0872664625997165 || theta>2.3561944901923448) return false;
    const double maximumRadius = tolerance / (std::tan(theta * 0.25) * std::tan(theta * 0.5));
    const double magnitude = 1.0 + (std::max)((std::max)((std::max)(std::fabs(ax), std::fabs(ay)),
        (std::max)(std::fabs(bx), std::fabs(by))), (std::max)(std::fabs(cx), std::fabs(cy))) + maximumRadius;
    const double arithmeticBudget = 512.0 * std::numeric_limits<double>::epsilon() * magnitude;
    const double guard = (std::max)(tolerance * 1e-8, arithmeticBudget);
    if (!std::isfinite(guard) || guard >= tolerance * 0.25) return false;
    const double trim = (std::min)((tolerance - guard) / std::tan(theta * 0.25), 0.25 * (std::min)(a, b));
    const double radius = trim / std::tan(theta * 0.5);
    const int dir = cross > 0.0 ? 1 : -1;
    const double t1x = bx - ux * trim, t1y = by - uy * trim, t2x = bx + vx * trim, t2y = by + vy * trim;
    if (!std::isfinite(radius) || radius <= 0.0 || trim <= 1e-6 || a - trim <= 1e-6) return false;
    ai = NCPathCoreFeedArcInput{};
    ai.startMCS = input.startMCS; ai.endMCS = input.startMCS;
    ai.startPulse = input.startPulse; ai.endPulse = input.startPulse;
    ai.startMCS[0] = t1x; ai.startMCS[1] = t1y; ai.endMCS[0] = t2x; ai.endMCS[1] = t2y;
    for (unsigned i = 0; i < 2; ++i) { ai.startPulse[i] = ai.startMCS[i] * ppm[i]; ai.endPulse[i] = ai.endMCS[i] * ppm[i]; }
    // Canonicalize a stationary native-pulse component against the immutable
    // successor vertex; preserve the strict line zero-delta space check.
    for (unsigned i = 0U; i < 2U; ++i) {
        const double nextPulse = next[i] * ppm[i];
        if (std::isfinite(nextPulse) && ai.endPulse[i] == nextPulse &&
            std::fabs(ai.endMCS[i] - next[i]) <= arithmeticBudget)
            ai.endMCS[i] = next[i];
    }
    ai.centerOffsetMM[0] = -double(dir) * uy * radius; ai.centerOffsetMM[1] = double(dir) * ux * radius;
    ai.pulsePerMM = ppm; ai.maxVelocityPPS[0] = input.maxVelocityPPS[0]; ai.maxVelocityPPS[1] = input.maxVelocityPPS[1];
    ai.feedMMMin = input.feedMMMin; ai.direction = dir;
    if (BuildNCPathCoreFeedArc(ai, arc) != NCPathCoreFeedArcCode::BUILT_ARC) return false;
    input.endMCS = arc.startMCS; input.endPulse = arc.startPulse;
    if (BuildNCPathCoreFeedLine(input, prefix) != NCPathCoreFeedLineCode::BUILT_LINE || !BuildRetainedArc(arc, out, true)) return false;
    out.kind = NCPathCoreRetainedKind::LINE_ARC; out.startMCS = prefix.startMCS; out.startPulse = prefix.startPulse;
    out.lengthMM += prefix.lengthMM; out.lengthPulse += prefix.lengthPulse;
    for (unsigned i = 0U; i < 2U; ++i) {
        out.boundsMinMCS[i] = (std::min)(out.boundsMinMCS[i], prefix.startMCS[i]);
        out.boundsMaxMCS[i] = (std::max)(out.boundsMaxMCS[i], prefix.startMCS[i]);
    }
    const double actualDeviation = std::hypot(bx - arc.centerMCS[0], by - arc.centerMCS[1]) - arc.radiusMM;
    if (!std::isfinite(actualDeviation) || actualDeviation<0.0 || actualDeviation + arithmeticBudget>tolerance || !IsNCPathCoreRetainedGeometryValid(out)) { out.Clear(); return false; }
    meta.vertex = { {bx,by} }; meta.next = { {cx,cy} }; meta.entry = { {t1x,t1y} }; meta.exit = { {arc.endMCS[0],arc.endMCS[1]} };
    meta.toleranceMM = tolerance; meta.trimMM = trim; meta.deviationMM = actualDeviation + arithmeticBudget;
    return true;
}
