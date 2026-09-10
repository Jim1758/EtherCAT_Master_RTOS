#include "NCPathCoreRetainedPath.h"
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
        if (g.kind == NCPathCoreRetainedKind::ARC && axis < 2U)
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
            (g.kind != NCPathCoreRetainedKind::LINE && g.kind != NCPathCoreRetainedKind::ARC))
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
        if (g.kind == NCPathCoreRetainedKind::LINE)
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

bool IsNCPathCoreRetainedGeometryValid(const NCPathCoreRetainedGeometry& g) noexcept
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
    for (std::uint32_t axis = 0U; axis < 2U; ++axis)
    {
        if (!std::isfinite(g.centerMCS[axis]) || !std::isfinite(g.centerPulse[axis]) ||
            (g.fullCircle && (!SameBits(g.startMCS[axis], g.endMCS[axis]) ||
                !SameBits(g.startPulse[axis], g.endPulse[axis])))) return false;
        const double trigStart = axis == 0U ? std::cos(g.startAngle) : std::sin(g.startAngle);
        const double endAngle = g.startAngle + g.sweepRadians;
        const double trigEnd = axis == 0U ? std::cos(endAngle) : std::sin(endAngle);
        if (!Near(g.centerMCS[axis] + g.radiusMM * trigStart, g.startMCS[axis], Magnitude(g, axis, false)) ||
            !Near(g.centerMCS[axis] + g.radiusMM * trigEnd, g.endMCS[axis], Magnitude(g, axis, false)) ||
            !Near(g.centerPulse[axis] + g.radiusPulse * trigStart, g.startPulse[axis], Magnitude(g, axis, true)) ||
            !Near(g.centerPulse[axis] + g.radiusPulse * trigEnd, g.endPulse[axis], Magnitude(g, axis, true))) return false;
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

bool BuildNCPathCoreRetainedArc(const NCPathCoreFeedArcV2& source,
    NCPathCoreRetainedGeometry& output) noexcept
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
    if (IsNCPathCoreRetainedGeometryValid(output)) return true;
    output.Clear(); return false;
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
