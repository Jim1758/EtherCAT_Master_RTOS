#pragma once

#include "EDMSixAxisCommonPath.h"
#include "NCArcPlane.h"
#include "NCPathCoreFeedArc.h"
#include <algorithm>

#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error EDM112_curves_require_precise_floating_point
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error EDM112_curves_require_nonfinite_checks
#endif

// Numeric geometry only. No queue, epoch, motor, PDO or replay authority.
// Inputs are immutable during a call. Evaluation outputs must be disjoint
// from the input geometry and from one another. Builders support in-place
// rebuilding of their source fields; failure invalidates the entire output.
// Native coordinates here are logicalPulse / pulsePerNative, not an inferred
// work-coordinate transform. XYZ are mm; C/U/V are unwrapped native degrees.
namespace EDM112
{
using Point = EDM109::Point;
constexpr double Pi = 3.141592653589793238462643383279502884;
constexpr double TwoPi = 2.0 * Pi;
constexpr double RoundoffFactor = 64.0 * std::numeric_limits<double>::epsilon();
constexpr double MaximumArcBudgetMm = 1.0e-7;
enum class Kind : std::uint8_t { None, Line, Arc };

struct Curve
{
    // Line retains the exact EDM109 value. Arc uses only its endpoint/scale
    // arrays; unused line fields remain canonical zero/false.
    EDM109::Line line{};
    std::array<double, 2U> centerOffsetMm{}, centerPulse{};
    double lengthMm = 0.0, radiusMm = 0.0, startAngle = 0.0, sweepRadians = 0.0;
    double precisionBudgetMm = 0.0;
    unsigned plane = 0U;
    int direction = 0;
    Kind kind = Kind::None;
    bool fullCircle = false, valid = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};

inline bool Reject(Curve& output) noexcept { output = Curve{}; return false; }

inline bool BuildLine(const EDM109::Line& line, Curve& output) noexcept
{
    Curve candidate{};
    candidate.line = line; // Preserve an in-place source before invalidation.
    output.valid = false;
    if (!EDM109::ValidLine(candidate.line)) return Reject(output);
    candidate.kind = Kind::Line; candidate.lengthMm = candidate.line.lengthMm;
    candidate.valid = true; output = candidate; return true;
}

inline bool BuildArc(const Point& startPulse, const Point& endPulse,
    const Point& pulsePerNative, unsigned plane,
    const std::array<double, 2U>& centerOffsetMm, int direction, bool fullCircle,
    Curve& output) noexcept
{
    Curve candidate{};
    candidate.line.startPulse = startPulse; candidate.line.endPulse = endPulse;
    candidate.line.pulsePerNative = pulsePerNative; candidate.centerOffsetMm = centerOffsetMm;
    candidate.plane = plane; candidate.direction = direction; candidate.fullCircle = fullCircle;
    output.valid = false;
    NCArcPlaneAxes axes{};
    if (!TryGetNCArcPlaneAxes(static_cast<int>(plane), axes) || (direction != -1 && direction != 1))
        return Reject(output);
    Point deltaNative{};
    for (unsigned i = 0U; i < 6U; ++i)
    {
        const double a = candidate.line.startPulse[i], b = candidate.line.endPulse[i];
        const double scale = candidate.line.pulsePerNative[i];
        if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(scale) || scale <= 0.0 ||
            !std::isfinite(a / scale) || !std::isfinite(b / scale)) return Reject(output);
        deltaNative[i] = (b - a) / scale;
        if (!std::isfinite(deltaNative[i])) return Reject(output);
    }
    const unsigned slots[2U] = {axes.u, axes.v};
    double absoluteScale = 0.0, localScale = 0.0;
    for (unsigned j = 0U; j < 2U; ++j)
    {
        const unsigned i = slots[j]; const double offset = candidate.centerOffsetMm[j];
        const double scale = candidate.line.pulsePerNative[i];
        if (!std::isfinite(offset)) return Reject(output);
        candidate.centerPulse[j] = candidate.line.startPulse[i] + offset * scale;
        if (!std::isfinite(candidate.centerPulse[j])) return Reject(output);
        absoluteScale = (std::max)(absoluteScale, (std::max)(std::fabs(candidate.line.startPulse[i] / scale),
            (std::max)(std::fabs(candidate.line.endPulse[i] / scale), std::fabs(candidate.centerPulse[j] / scale))));
        localScale = (std::max)(localScale, (std::max)(std::fabs(offset), std::fabs(deltaNative[i])));
    }
    const double radius = std::hypot(candidate.centerOffsetMm[0U], candidate.centerOffsetMm[1U]);
    localScale = (std::max)(localScale, radius);
    const double originBudget = RoundoffFactor * absoluteScale;
    const double budget = originBudget + RoundoffFactor * localScale;
    if (!std::isfinite(radius) || !std::isfinite(budget) || radius <= budget || budget <= 0.0 ||
        budget > MaximumArcBudgetMm) return Reject(output);
    // A full turn is explicit, and requires exact planar pulse closure. Other
    // axes may advance linearly (helix / unwrapped rotary blend).
    const bool samePlane = EDM109::SameDouble(candidate.line.startPulse[axes.u], candidate.line.endPulse[axes.u]) &&
        EDM109::SameDouble(candidate.line.startPulse[axes.v], candidate.line.endPulse[axes.v]);
    if (samePlane != fullCircle) return Reject(output);
    for (unsigned j = 0U; j < 2U; ++j)
    {
        const unsigned i = slots[j];
        const double represented = (candidate.centerPulse[j] - candidate.line.startPulse[i]) /
            candidate.line.pulsePerNative[i];
        if (!std::isfinite(represented) || std::fabs(represented - candidate.centerOffsetMm[j]) > budget)
            return Reject(output);
    }
    NCPathCoreArcPulseGeometry resolved{};
    // The shared resolver operates in one unit. Native plane mm remain a
    // circle even when the two physical pulse scales differ.
    if (!ResolveNCPathCorePlanarCirclePulse(0.0, 0.0, deltaNative[axes.u], deltaNative[axes.v],
        candidate.centerOffsetMm[0U], candidate.centerOffsetMm[1U], radius,
        direction, fullCircle, resolved, originBudget)) return Reject(output);
    candidate.radiusMm = resolved.radius; candidate.startAngle = resolved.startAngle;
    candidate.sweepRadians = resolved.sweepRadians;
    candidate.lengthMm = std::hypot(resolved.lengthPulse, deltaNative[axes.normal]);
    candidate.precisionBudgetMm = budget;
    if (!std::isfinite(candidate.lengthMm) || candidate.lengthMm <= 0.0) return Reject(output);
    candidate.kind = Kind::Arc; candidate.valid = true; output = candidate; return true;
}

inline bool SameCurve(const Curve& a, const Curve& b) noexcept
{
    return a.kind == b.kind && a.valid == b.valid && a.plane == b.plane && a.direction == b.direction &&
        a.fullCircle == b.fullCircle && a.line.valid == b.line.valid &&
        EDM109::SamePulse(a.line.startPulse, b.line.startPulse) && EDM109::SamePulse(a.line.endPulse, b.line.endPulse) &&
        EDM109::SamePulse(a.line.pulsePerNative, b.line.pulsePerNative) && EDM109::SamePulse(a.line.ratio, b.line.ratio) &&
        EDM109::SameDouble(a.line.pulseLength, b.line.pulseLength) && EDM109::SameDouble(a.line.lengthMm, b.line.lengthMm) &&
        EDM109::SameDouble(a.lengthMm, b.lengthMm) && EDM109::SameDouble(a.radiusMm, b.radiusMm) &&
        EDM109::SameDouble(a.startAngle, b.startAngle) && EDM109::SameDouble(a.sweepRadians, b.sweepRadians) &&
        EDM109::SameDouble(a.precisionBudgetMm, b.precisionBudgetMm) &&
        EDM109::SameDouble(a.centerOffsetMm[0U], b.centerOffsetMm[0U]) &&
        EDM109::SameDouble(a.centerOffsetMm[1U], b.centerOffsetMm[1U]) &&
        EDM109::SameDouble(a.centerPulse[0U], b.centerPulse[0U]) &&
        EDM109::SameDouble(a.centerPulse[1U], b.centerPulse[1U]);
}
inline bool ValidCurve(const Curve& curve) noexcept
{
    if (!curve.valid) return false;
    Curve rebuilt{};
    const bool built = curve.kind == Kind::Line ? BuildLine(curve.line, rebuilt) :
        curve.kind == Kind::Arc && BuildArc(curve.line.startPulse, curve.line.endPulse,
            curve.line.pulsePerNative, curve.plane, curve.centerOffsetMm,
            curve.direction, curve.fullCircle, rebuilt);
    return built && SameCurve(curve, rebuilt);
}

inline bool ClearEvaluation(Point& position, Point* first = nullptr, Point* second = nullptr) noexcept
{
    position = Point{}; if (first) *first = Point{}; if (second) *second = Point{}; return false;
}

// Canonical u never changes when traversal reverses. Derivatives are with
// respect to that original u, in pulses; callers apply the traversal sign.
inline bool EvaluateUnit(const Curve& curve, double u, Point& position,
    Point* derivativePerU = nullptr, Point* secondDerivativePerU2 = nullptr) noexcept
{
    if ((derivativePerU && derivativePerU == &position) ||
        (secondDerivativePerU2 && (secondDerivativePerU2 == &position || secondDerivativePerU2 == derivativePerU)) ||
        !ValidCurve(curve) || !std::isfinite(u) || u < 0.0 || u > 1.0)
        return ClearEvaluation(position, derivativePerU, secondDerivativePerU2);
    Point p{}, d1{}, d2{};
    for (unsigned i = 0U; i < 6U; ++i)
    {
        const double delta = curve.line.endPulse[i] - curve.line.startPulse[i];
        d1[i] = delta;
        p[i] = curve.kind == Kind::Arc && delta == 0.0 ? curve.line.startPulse[i] :
            curve.line.startPulse[i] + delta * u;
    }
    if (curve.kind == Kind::Arc)
    {
        NCArcPlaneAxes axes{}; if (!TryGetNCArcPlaneAxes(static_cast<int>(curve.plane), axes))
            return ClearEvaluation(position, derivativePerU, secondDerivativePerU2);
        const unsigned slots[2U] = {axes.u, axes.v};
        const double angle = curve.startAngle + u * curve.sweepRadians;
        const double c = std::cos(angle), s = std::sin(angle);
        for (unsigned j = 0U; j < 2U; ++j)
        {
            const unsigned i = slots[j]; const double trig = j == 0U ? c : s;
            const double native = curve.centerOffsetMm[j] + curve.radiusMm * trig;
            p[i] = curve.line.startPulse[i] + native * curve.line.pulsePerNative[i];
            d1[i] = ((j == 0U ? -s : c) * curve.radiusMm * curve.sweepRadians) * curve.line.pulsePerNative[i];
            d2[i] = ((-trig * curve.radiusMm * curve.sweepRadians) * curve.sweepRadians) * curve.line.pulsePerNative[i];
        }
    }
    if (u == 0.0) p = curve.line.startPulse;
    else if (u == 1.0) p = curve.line.endPulse;
    for (unsigned i = 0U; i < 6U; ++i)
        if (!std::isfinite(p[i]) || !std::isfinite(d1[i]) || !std::isfinite(d2[i]))
            return ClearEvaluation(position, derivativePerU, secondDerivativePerU2);
    position = p; if (derivativePerU) *derivativePerU = d1;
    if (secondDerivativePerU2) *secondDerivativePerU2 = d2;
    return true;
}

inline bool EvaluateMm(const Curve& curve, double distanceMm, double signedSpeedMmSec,
    Point& position, Point& velocity) noexcept
{
    if (&position == &velocity || !ValidCurve(curve) || !std::isfinite(distanceMm) ||
        distanceMm < 0.0 || distanceMm > curve.lengthMm || !std::isfinite(signedSpeedMmSec))
        return ClearEvaluation(position, &velocity);
    Point p{}, derivative{}, v{};
    const double u = distanceMm == curve.lengthMm ? 1.0 : distanceMm / curve.lengthMm;
    if (!EvaluateUnit(curve, u, p, &derivative)) return ClearEvaluation(position, &velocity);
    for (unsigned i = 0U; i < 6U; ++i)
    {
        v[i] = (derivative[i] / curve.lengthMm) * signedSpeedMmSec;
        if (!std::isfinite(v[i])) return ClearEvaluation(position, &velocity);
    }
    position = p; velocity = v; return true;
}

// Raw G01 shadow comparison only: preserve EDM109's exact operation order,
// including its endpoint rounding and finite formal-stop scalar domain.
inline bool EvaluateLineScalar(const Curve& curve, double q, double v,
    Point& position, Point& velocity) noexcept
{
    if (&position == &velocity || curve.kind != Kind::Line || !ValidCurve(curve) ||
        !EDM109::EvaluateScalar(curve.line, q, v, position, velocity))
        return ClearEvaluation(position, &velocity);
    return true;
}

struct Bounds
{
    Point minPulse{}, maxPulse{}, minNative{}, maxNative{};
    bool valid = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};
inline bool ContainsAngle(double angle, double start, double sweep, bool fullCircle) noexcept
{
    if (fullCircle) return true;
    double travel = sweep > 0.0 ? angle - start : start - angle;
    travel = std::fmod(travel, TwoPi); if (travel < 0.0) travel += TwoPi;
    return travel <= std::fabs(sweep) + RoundoffFactor * TwoPi;
}
inline bool WholeCurveBounds(const Curve& curve, Bounds& output) noexcept
{
    output = Bounds{}; if (!ValidCurve(curve)) return false;
    Bounds b{};
    for (unsigned i = 0U; i < 6U; ++i)
    {
        b.minPulse[i] = (std::min)(curve.line.startPulse[i], curve.line.endPulse[i]);
        b.maxPulse[i] = (std::max)(curve.line.startPulse[i], curve.line.endPulse[i]);
    }
    if (curve.kind == Kind::Arc)
    {
        NCArcPlaneAxes axes{}; if (!TryGetNCArcPlaneAxes(static_cast<int>(curve.plane), axes)) return false;
        for (unsigned k = 0U; k < 4U; ++k)
        {
            const double angle = static_cast<double>(k) * (0.5 * Pi);
            if (!ContainsAngle(angle, curve.startAngle, curve.sweepRadians, curve.fullCircle)) continue;
            const unsigned j = k % 2U, i = j == 0U ? axes.u : axes.v;
            const double sign = k < 2U ? 1.0 : -1.0;
            const double p = curve.line.startPulse[i] +
                (curve.centerOffsetMm[j] + sign * curve.radiusMm) * curve.line.pulsePerNative[i];
            if (!std::isfinite(p)) return false;
            b.minPulse[i] = (std::min)(b.minPulse[i], p); b.maxPulse[i] = (std::max)(b.maxPulse[i], p);
        }
        // Conservative whole-arc bounds include endpoint/source arithmetic.
        const unsigned slots[2U] = {axes.u, axes.v};
        for (unsigned i : slots)
        {
            const double extra = curve.precisionBudgetMm * curve.line.pulsePerNative[i];
            b.minPulse[i] = std::nextafter(b.minPulse[i] - extra, -std::numeric_limits<double>::infinity());
            b.maxPulse[i] = std::nextafter(b.maxPulse[i] + extra, std::numeric_limits<double>::infinity());
        }
    }
    for (unsigned i = 0U; i < 6U; ++i)
    {
        b.minNative[i] = b.minPulse[i] / curve.line.pulsePerNative[i];
        b.maxNative[i] = b.maxPulse[i] / curve.line.pulsePerNative[i];
        if (!std::isfinite(b.minPulse[i]) || !std::isfinite(b.maxPulse[i]) ||
            !std::isfinite(b.minNative[i]) || !std::isfinite(b.maxNative[i]) || b.minNative[i] > b.maxNative[i]) return false;
    }
    b.valid = true; output = b; return true;
}
inline bool BoundsWithin(const Bounds& bounds, const Point& minimumNative, const Point& maximumNative) noexcept
{
    if (!bounds.valid) return false;
    for (unsigned i = 0U; i < 6U; ++i)
        if (!std::isfinite(minimumNative[i]) || !std::isfinite(maximumNative[i]) || minimumNative[i] > maximumNative[i] ||
            !std::isfinite(bounds.minPulse[i]) || !std::isfinite(bounds.maxPulse[i]) ||
            bounds.minPulse[i] > bounds.maxPulse[i] ||
            !std::isfinite(bounds.minNative[i]) || !std::isfinite(bounds.maxNative[i]) ||
            bounds.minNative[i] > bounds.maxNative[i] || bounds.minNative[i] < minimumNative[i] ||
            bounds.maxNative[i] > maximumNative[i]) return false;
    return true;
}

struct Span
{
    Curve curve{};
    double startMm = 0.0, endMm = 0.0, lengthMm = 0.0;
    bool stopRequired = true, valid = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};
inline bool BuildSpan(const Curve& curve, double startMm, double endMm, Span& output) noexcept
{
    // Copy only the source curve, never a whole plan workspace.
    const Curve source = curve; output.valid = false;
    if (!ValidCurve(source) || !std::isfinite(startMm) || !std::isfinite(endMm) ||
        startMm < 0.0 || startMm > source.lengthMm || endMm < 0.0 || endMm > source.lengthMm || startMm == endMm)
    { output = Span{}; return false; }
    output.curve = source; output.startMm = startMm; output.endMm = endMm;
    output.lengthMm = std::fabs(endMm - startMm); output.stopRequired = true;
    output.valid = std::isfinite(output.lengthMm) && output.lengthMm > 0.0; return output.valid;
}
inline bool ValidSpan(const Span& span) noexcept
{
    return span.valid && span.stopRequired && ValidCurve(span.curve) &&
        std::isfinite(span.startMm) && std::isfinite(span.endMm) &&
        span.startMm >= 0.0 && span.startMm <= span.curve.lengthMm &&
        span.endMm >= 0.0 && span.endMm <= span.curve.lengthMm && span.startMm != span.endMm &&
        EDM109::SameDouble(span.lengthMm, std::fabs(span.endMm - span.startMm));
}
inline bool EvaluateSpan(const Span& span, double travelMm, double signedSpeedMmSec,
    Point& position, Point& velocity) noexcept
{
    if (!ValidSpan(span) || !std::isfinite(travelMm) || travelMm < 0.0 || travelMm > span.lengthMm)
        return ClearEvaluation(position, &velocity);
    const double direction = span.endMm > span.startMm ? 1.0 : -1.0;
    const double local = travelMm == 0.0 ? span.startMm : travelMm == span.lengthMm ? span.endMm :
        span.startMm + direction * travelMm;
    return EvaluateMm(span.curve, local, signedSpeedMmSec * direction, position, velocity);
}

struct B2Plan
{
    std::array<Span, 4U> spans{};
    Point anchorPulse{}, returnedPulse{};
    unsigned count = 0U;
    bool valid = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};
inline void ClearPlan(B2Plan& output) noexcept
{
    output.valid = false; output.count = 0U;
    output.anchorPulse = Point{}; output.returnedPulse = Point{};
    for (auto& span : output.spans) span = Span{};
}
// Caller owns the fixed workspace. Inputs may be curves inside the old output;
// capture them before clearing. No physical excursion or permission is created.
inline bool BuildB2Plan(const Curve& current, const Curve& previous, double anchorMm,
    double previousRetreatFraction, B2Plan& output) noexcept
{
    const Curve a = current, b = previous; ClearPlan(output);
    if (!ValidCurve(a) || !ValidCurve(b) || !std::isfinite(anchorMm) || anchorMm <= 0.0 || anchorMm >= a.lengthMm ||
        !std::isfinite(previousRetreatFraction) || previousRetreatFraction <= 0.0 || previousRetreatFraction >= 1.0 ||
        !EDM109::SamePulse(b.line.endPulse, a.line.startPulse) ||
        !EDM109::SamePulse(b.line.pulsePerNative, a.line.pulsePerNative)) return false;
    const double target = b.lengthMm * (1.0 - previousRetreatFraction);
    if (!std::isfinite(target) || target <= 0.0 || target >= b.lengthMm ||
        !BuildSpan(a, anchorMm, 0.0, output.spans[0U]) ||
        !BuildSpan(b, b.lengthMm, target, output.spans[1U]) ||
        !BuildSpan(b, target, b.lengthMm, output.spans[2U]) ||
        !BuildSpan(a, 0.0, anchorMm, output.spans[3U])) { ClearPlan(output); return false; }
    Point velocity{};
    if (!EvaluateMm(a, anchorMm, 0.0, output.anchorPulse, velocity) ||
        !EvaluateSpan(output.spans[3U], output.spans[3U].lengthMm, 0.0, output.returnedPulse, velocity) ||
        !EDM109::SamePulse(output.anchorPulse, output.returnedPulse)) { ClearPlan(output); return false; }
    output.count = 4U; output.valid = true; return true;
}
inline bool BuildB2Plan(const Curve& current, const Curve& previous, double anchorMm, B2Plan& output) noexcept
{ return BuildB2Plan(current, previous, anchorMm, 0.1, output); }

static_assert(std::is_trivially_copyable<Curve>::value && sizeof(Curve) <= 320U, "EDM112 fixed numeric curve");
static_assert(std::is_trivially_copyable<Span>::value && sizeof(Span) <= 352U, "EDM112 fixed canonical interval");
static_assert(std::is_trivially_copyable<B2Plan>::value && sizeof(B2Plan) <= 1536U, "EDM112 caller-owned numeric plan");
}
