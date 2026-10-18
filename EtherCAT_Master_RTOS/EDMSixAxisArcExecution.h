#pragma once

#include "EDMSixAxisCommonPath.h"
#include "EDMSixAxisCommonCurve.h"
#include <algorithm>

#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error EDM114_arc_execution_requires_precise_floating_point
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error EDM114_arc_execution_requires_nonfinite_checks
#endif

// EDM114: canonical G17 ARC2 execution values. Ownership, freshness and
// output reservation remain the Motion caller's responsibility.
namespace EDM114
{
using Point = EDM109::Point;
using Source = EDM109::Source;
constexpr unsigned RequiredRows = 16U;
constexpr double SampleSeconds = 0.000250;
constexpr double Pi = 3.141592653589793238462643383279502884;
enum class Kind : std::uint8_t { None, Line, Arc };
enum class Reason : std::uint8_t
{ None, Lifecycle, Scope, Sequence, Geometry, Limits, Clock, Domain, Numeric,
  Velocity, Acceleration, StopEnvelope, Completion, Cancelled, Output };
inline bool IsArcStep(unsigned step) noexcept { return step < RequiredRows && ((step / 2U) & 1U) != 0U; }
inline int ExpectedGCode(unsigned step) noexcept
{
    if (step >= RequiredRows) return 0;
    if (!IsArcStep(step)) return 1;
    const bool outgoingCW = ((step / 2U) & 2U) == 0U;
    return (outgoingCW != ((step & 1U) != 0U)) ? 2 : 3;
}
inline bool ValidArcRecipe(int gcode, unsigned step, const Point& increment,
    const std::array<double, 2U>& center, double feed) noexcept
{
    if (!IsArcStep(step) || gcode != ExpectedGCode(step) || feed != 0.12 ||
        increment[0] != ((step & 1U) ? -0.002 : 0.002) ||
        center[0] != ((step & 1U) ? -0.001 : 0.001) || center[1] != 0.0) return false;
    for (unsigned i = 1U; i < 6U; ++i) if (increment[i] != 0.0) return false;
    return true;
}
inline bool SameSource(const Source& a, const Source& b) noexcept
{
    return EDM109::SameScope(a.scope, b.scope) && a.dispatch == b.dispatch && a.epoch == b.epoch &&
        a.segment == b.segment && a.sourcePC == b.sourcePC && a.step == b.step;
}
inline bool ValidSource(const Source& s) noexcept
{
    return EDM109::ValidScope(s.scope) && s.dispatch && s.epoch &&
        s.epoch <= (std::numeric_limits<std::uint32_t>::max)() && s.segment && s.sourcePC >= 0 && s.step < RequiredRows;
}
struct Limits
{
    Point maxVelocityPps{}, maxAccelerationPps2{};
    double scalarMaxVelocity = 0.0, scalarAcceleration = 0.0, scalarDeceleration = 0.0;
};
struct Geometry
{
    Point startPulse{}, endPulse{}, pulsePerNative{}, ratio{};
    std::array<double, 2U> centerPulse{};
    double radiusPulse = 0.0, startAngle = 0.0, sweepRadians = 0.0, pulseLength = 0.0;
    Kind kind = Kind::None;
    bool valid = false;
};
struct Feedback
{
    std::uint64_t session = 0ULL, armEpoch = 0ULL, ownerGeneration = 0ULL;
    Source source{};
    Point startPulse{}, endPulse{}, pulsePerNative{};
    std::array<double, 2U> centerPulse{};
    double radiusPulse = 0.0, startAngle = 0.0, sweepRadians = 0.0, pulseLength = 0.0;
    std::uint64_t lastTick = 0ULL, lastMappingTick = 0ULL, lastMappingUs = 0ULL;
    std::uint64_t samples = 0ULL, forwardSamples = 0ULL, reverseSamples = 0ULL, flushSamples = 0ULL;
    std::uint64_t rowSamples = 0ULL, rowForward = 0ULL, rowReverse = 0ULL, rowFlush = 0ULL;
    unsigned begunRows = 0U, completedRows = 0U, lineRows = 0U, arcRows = 0U;
    unsigned cwRows = 0U, ccwRows = 0U, endpointRows = 0U;
    Kind kind = Kind::None;
    Reason reason = Reason::None;
    bool enabled = false, geometryValid = false, mappedOutput = false, failed = false;
    bool rowComplete = false, completed = false, stopActive = false, stopped = false;
};
static_assert(sizeof(Feedback) <= 512U, "EDM114 fixed publication budget");
static_assert(std::is_trivially_copyable<Feedback>::value, "EDM114 publication is a value");

inline double DiscreteStopDistance(double speed, double deceleration) noexcept;
inline bool StopFits(double q, double v, double length, double deceleration) noexcept;
class Mapping
{
public:
    bool BindArc(const Point& start, const Point& end, const Point& ppu,
        const std::array<double, 2U>& centerPulse, double radiusPulse,
        double startAngle, double sweepRadians, double length, const Limits& limits,
        bool boundedAuthoredArc = false) noexcept;
    bool BindLine(const Point& start, const Point& end, const Point& ppu,
        double length, const Point& ratio, const Limits& limits) noexcept;
    bool Eval(double q, double v, double a, Point& position, Point& velocity, Point& acceleration) const noexcept;
    const EDM114::Geometry& GetGeometry() const noexcept { return geometry_; }
    const Limits& FrozenLimits() const noexcept { return limits_; }
    void Clear() noexcept { geometry_ = EDM114::Geometry{}; limits_ = Limits{}; }
private:
    bool FreezeLimits(const Limits& limits) noexcept;
    EDM114::Geometry geometry_{};
    Limits limits_{};
};
struct StopState
{
    double q = 0.0, v = 0.0, length = 0.0, deceleration = 0.0;
    std::uint64_t steps = 0ULL;
    bool active = false, stopped = false, failed = false;
    Reason reason = Reason::None;
};
class StopScalar
{
public:
    bool Begin(double q, double v, double length, double deceleration) noexcept;
    bool Step() noexcept;
    void Clear() noexcept { state_ = StopState{}; initialSpeed_ = 0.0; initialQ_ = 0.0; }
    const StopState& Current() const noexcept { return state_; }
private:
    bool Fail(Reason reason) noexcept
    { if (!state_.failed) state_.reason = reason; state_.failed = true; state_.active = state_.stopped = false; return false; }
    StopState state_{};
    double initialSpeed_ = 0.0;
    double initialQ_ = 0.0;
};

inline bool Positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }
inline double Roundoff(double scale) noexcept
{ return 128.0 * std::numeric_limits<double>::epsilon() * (std::max)(1.0, std::fabs(scale)); }
// A literal semicircle can acquire a few angular ULPs when an unchanged
// native coordinate is converted back to pulses at a nonzero origin. Test
// its angular discrepancy in the same pulse-space budget as its endpoints;
// never reinterpret or alter the builder's canonical radius/sweep/endpoint.
inline bool HalfCircleSweepFits(const Point& start, const Point& end, const Point& ppu,
    const std::array<double, 2U>& center, double radius, double sweep) noexcept
{
    if (!Positive(radius) || !std::isfinite(sweep) || sweep == 0.0 || std::fabs(sweep) >= 2.0 * Pi) return false;
    double scale = radius;
    for (unsigned i = 0U; i < 2U; ++i)
    {
        if (!std::isfinite(start[i]) || !std::isfinite(end[i]) || !std::isfinite(center[i]) || !Positive(ppu[i])) return false;
        scale = (std::max)(scale, (std::max)(std::fabs(center[i]),
            (std::max)(std::fabs(start[i]), std::fabs(end[i]))));
    }
    const double budget = Roundoff(scale);
    const double error = std::fabs(std::fabs(sweep) - Pi) * radius;
    return std::isfinite(budget) && std::isfinite(error) &&
        budget / ppu[0] <= 5.0e-8 && budget / ppu[1] <= 5.0e-8 && error <= budget;
}
// EDM152 accepts a bounded authored G17 arc only through an explicit caller
// opt-in. Rebuild with the existing CNC circle resolver; these values own no
// epoch, owner, Motion command, output channel, or physical settlement proof.
constexpr double MaximumBoundedArcNativeError = 5.0e-8;
inline bool BoundedArcDimensionsFit(double radiusMm, double sweep, double nativeError) noexcept
{
    if (!Positive(radiusMm) || !std::isfinite(sweep) || sweep == 0.0 ||
        !std::isfinite(nativeError) || nativeError < 0.0 || nativeError > MaximumBoundedArcNativeError) return false;
    const double angleError = nativeError / radiusMm;
    return std::isfinite(angleError) && radiusMm + nativeError >= 0.0005 &&
        radiusMm - nativeError <= 0.002 && std::fabs(sweep) + angleError >= Pi / 4.0 &&
        std::fabs(sweep) - angleError <= Pi;
}
inline bool BuildBoundedXYArc(int code, const Point& increment,
    const std::array<double, 2U>& center, double feed, EDM112::Curve& curve) noexcept
{
    curve = EDM112::Curve{};
    if ((code != 2 && code != 3) || feed != 0.12) return false;
    Point start{}, end{}, ppu{}; ppu.fill(1000000.0);
    for (unsigned i = 0U; i < 6U; ++i) {
        if (!std::isfinite(increment[i]) || (i >= 2U && increment[i] != 0.0)) return false;
        end[i] = increment[i] * ppu[i];
        if (!std::isfinite(end[i])) return false;
    }
    if ((increment[0] == 0.0 && increment[1] == 0.0) ||
        !std::isfinite(center[0]) || !std::isfinite(center[1])) return false;
    EDM112::Curve candidate{};
    if (!EDM112::BuildArc(start, end, ppu, 17U, center, code == 2 ? -1 : 1, false, candidate) ||
        !candidate.valid || candidate.fullCircle ||
        !BoundedArcDimensionsFit(candidate.radiusMm, candidate.sweepRadians, candidate.precisionBudgetMm)) return false;
    curve = candidate; return true;
}
inline bool BoundedArcSweepFits(const Point& start, const Point& end, const Point& ppu,
    const std::array<double, 2U>& center, double radius, double sweep) noexcept
{
    if (!Positive(radius) || !std::isfinite(sweep) || sweep == 0.0 ||
        !Positive(ppu[0]) || ppu[0] != ppu[1]) return false;
    double scale = radius;
    for (unsigned i = 0U; i < 6U; ++i) {
        if (!std::isfinite(start[i]) || !std::isfinite(end[i]) || !Positive(ppu[i]) ||
            (i >= 2U && !EDM109::SameDouble(start[i], end[i]))) return false;
        if (i < 2U) {
            if (!std::isfinite(center[i])) return false;
            scale = (std::max)(scale, (std::max)(std::fabs(center[i]),
                (std::max)(std::fabs(start[i]), std::fabs(end[i]))));
        }
    }
    const double pulseBudget = 4.0 * Roundoff(scale);
    const double nativeBudget = pulseBudget / ppu[0];
    if (!std::isfinite(pulseBudget) || !BoundedArcDimensionsFit(radius / ppu[0], sweep, nativeBudget)) return false;
    const std::array<double, 2U> offset{{(center[0] - start[0]) / ppu[0],
        (center[1] - start[1]) / ppu[1]}};
    EDM112::Curve rebuilt{};
    if (!EDM112::BuildArc(start, end, ppu, 17U, offset, sweep < 0.0 ? -1 : 1, false, rebuilt) ||
        !rebuilt.valid || rebuilt.fullCircle || rebuilt.precisionBudgetMm > MaximumBoundedArcNativeError ||
        !BoundedArcDimensionsFit(rebuilt.radiusMm, rebuilt.sweepRadians, nativeBudget)) return false;
    // Radius controls the acceleration/curvature bound. A different center,
    // endpoint or direction cannot pass by supplying an in-range angle alone.
    const double radiusError = std::fabs(rebuilt.radiusMm * ppu[0] - radius);
    const double sweepError = std::fabs(rebuilt.sweepRadians - sweep) * radius;
    return std::isfinite(radiusError) && std::isfinite(sweepError) &&
        radiusError <= pulseBudget && sweepError <= pulseBudget;
}
inline bool Within(double value, double limit) noexcept
{ return std::isfinite(value) && Positive(limit) && value <= limit + Roundoff(limit); }
inline double DiscreteStopDistance(double speed, double deceleration) noexcept
{
    if (!std::isfinite(speed) || speed < 0.0 || !Positive(deceleration))
        return (std::numeric_limits<double>::infinity)();
    if (speed == 0.0) return 0.0;
    const double drop = deceleration * SampleSeconds;
    if (!Positive(drop)) return (std::numeric_limits<double>::infinity)();
    const double residual = std::fmod(speed, drop);
    const double distance = 0.5 * speed * (speed / deceleration) +
        0.5 * SampleSeconds * residual * (1.0 - residual / drop);
    return std::isfinite(distance) && distance >= 0.0 ? distance : (std::numeric_limits<double>::infinity)();
}
inline bool StopFits(double q, double v, double length, double deceleration) noexcept
{
    if (!Positive(length) || !Positive(deceleration) || !std::isfinite(q) ||
        !std::isfinite(v) || q < 0.0 || q > length) return false;
    const double available = v < 0.0 ? q : length - q;
    return DiscreteStopDistance(std::fabs(v), deceleration) <= available + Roundoff(length);
}
inline bool Mapping::FreezeLimits(const Limits& limits) noexcept
{
    if (!geometry_.valid || !Positive(limits.scalarMaxVelocity) ||
        !Positive(limits.scalarAcceleration) || !Positive(limits.scalarDeceleration)) return false;
    const double maximumA = (std::max)(limits.scalarAcceleration, limits.scalarDeceleration);
    for (unsigned i = 0U; i < 6U; ++i)
    {
        if (!Positive(limits.maxVelocityPps[i]) || !Positive(limits.maxAccelerationPps2[i])) return false;
        const double d1 = geometry_.kind == Kind::Arc ? (i < 2U ? 1.0 : 0.0) : std::fabs(geometry_.ratio[i]);
        const double curvature = geometry_.kind == Kind::Arc && i < 2U ?
            (limits.scalarMaxVelocity / geometry_.radiusPulse) * limits.scalarMaxVelocity : 0.0;
        if (!Within(d1 * limits.scalarMaxVelocity, limits.maxVelocityPps[i]) ||
            !Within(d1 * maximumA + curvature, limits.maxAccelerationPps2[i])) return false;
    }
    limits_ = limits; return true;
}
inline bool Mapping::BindLine(const Point& start, const Point& end, const Point& ppu,
    double length, const Point& ratio, const Limits& limits) noexcept
{
    Clear(); EDM109::Line line{};
    if (!EDM109::BuildLine(start, end, ppu, line) || !EDM109::SameDouble(line.pulseLength, length) ||
        !EDM109::SamePulse(line.ratio, ratio)) return false;
    geometry_.startPulse = start; geometry_.endPulse = end; geometry_.pulsePerNative = ppu;
    geometry_.ratio = ratio; geometry_.pulseLength = length; geometry_.kind = Kind::Line; geometry_.valid = true;
    if (!FreezeLimits(limits)) { Clear(); return false; }
    return true;
}
inline bool Mapping::BindArc(const Point& start, const Point& end, const Point& ppu,
    const std::array<double, 2U>& centerPulse, double radiusPulse,
    double startAngle, double sweepRadians, double length, const Limits& limits,
    bool boundedAuthoredArc) noexcept
{
    Clear();
    if (!Positive(radiusPulse) || !Positive(length) || !std::isfinite(startAngle) ||
        !std::isfinite(sweepRadians) || std::fabs(startAngle) > Pi ||
        !(boundedAuthoredArc ? BoundedArcSweepFits(start, end, ppu, centerPulse, radiusPulse, sweepRadians) :
            HalfCircleSweepFits(start, end, ppu, centerPulse, radiusPulse, sweepRadians)) ||
        length != radiusPulse * std::fabs(sweepRadians)) return false;
    for (unsigned i = 0U; i < 6U; ++i)
        if (!std::isfinite(start[i]) || !std::isfinite(end[i]) || !Positive(ppu[i]) ||
            (i >= 2U && !EDM109::SameDouble(start[i], end[i]))) return false;
    if (ppu[0] != ppu[1] || EDM109::SamePulse(start, end)) return false;
    const double endAngle = startAngle + sweepRadians;
    const double expectedStart[2U] = {radiusPulse * std::cos(startAngle), radiusPulse * std::sin(startAngle)};
    const double expectedEnd[2U] = {radiusPulse * std::cos(endAngle), radiusPulse * std::sin(endAngle)};
    for (unsigned i = 0U; i < 2U; ++i)
    {
        const double scale = (std::max)(radiusPulse, (std::max)(std::fabs(centerPulse[i]),
            (std::max)(std::fabs(start[i]), std::fabs(end[i]))));
        const double tolerance = Roundoff(scale);
        if (!std::isfinite(centerPulse[i]) || !std::isfinite(tolerance) ||
            tolerance / ppu[i] > 5.0e-8 ||
            std::fabs((start[i] - centerPulse[i]) - expectedStart[i]) > tolerance ||
            std::fabs((end[i] - centerPulse[i]) - expectedEnd[i]) > tolerance) return false;
    }
    geometry_.startPulse = start; geometry_.endPulse = end; geometry_.pulsePerNative = ppu;
    geometry_.centerPulse = centerPulse; geometry_.radiusPulse = radiusPulse;
    geometry_.startAngle = startAngle; geometry_.sweepRadians = sweepRadians; geometry_.pulseLength = length;
    geometry_.kind = Kind::Arc; geometry_.valid = true;
    if (!FreezeLimits(limits)) { Clear(); return false; }
    return true;
}
inline bool Mapping::Eval(double q, double v, double a, Point& position,
    Point& velocity, Point& acceleration) const noexcept
{
    if (&position == &velocity || &position == &acceleration || &velocity == &acceleration ||
        !geometry_.valid || !std::isfinite(a) || !Within(std::fabs(v), limits_.scalarMaxVelocity) ||
        !StopFits(q, v, geometry_.pulseLength, limits_.scalarDeceleration)) return false;
    const double maxA = (std::max)(limits_.scalarAcceleration, limits_.scalarDeceleration);
    const double accelerationRoundoff = Roundoff(std::fabs(v) / SampleSeconds + maxA);
    if (std::fabs(a) > maxA + accelerationRoundoff) return false;
    Point p = geometry_.startPulse, first{}, second{}, mappedV{}, mappedA{};
    if (geometry_.kind == Kind::Line)
    {
        for (unsigned i = 0U; i < 6U; ++i)
        { first[i] = geometry_.ratio[i]; p[i] = geometry_.startPulse[i] + q * first[i]; }
    }
    else if (geometry_.kind == Kind::Arc)
    {
        const double angle = geometry_.startAngle + (q / geometry_.pulseLength) * geometry_.sweepRadians;
        const double c = std::cos(angle), s = std::sin(angle);
        const double sign = geometry_.sweepRadians > 0.0 ? 1.0 : -1.0;
        p[0] = geometry_.centerPulse[0] + geometry_.radiusPulse * c;
        p[1] = geometry_.centerPulse[1] + geometry_.radiusPulse * s;
        first[0] = -s * sign; first[1] = c * sign;
        second[0] = -c / geometry_.radiusPulse; second[1] = -s / geometry_.radiusPulse;
    }
    else return false;
    if (q == 0.0) p = geometry_.startPulse;
    else if (q == geometry_.pulseLength) p = geometry_.endPulse;
    for (unsigned i = 0U; i < 6U; ++i)
    {
        mappedV[i] = first[i] * v;
        mappedA[i] = first[i] * a + (second[i] * v) * v;
        if (!std::isfinite(p[i]) || !Within(std::fabs(mappedV[i]), limits_.maxVelocityPps[i]) ||
            !std::isfinite(mappedA[i]) || std::fabs(mappedA[i]) > limits_.maxAccelerationPps2[i] +
                accelerationRoundoff + Roundoff(limits_.maxAccelerationPps2[i])) return false;
    }
    position = p; velocity = mappedV; acceleration = mappedA; return true;
}
inline bool StopScalar::Begin(double q, double v, double length, double deceleration) noexcept
{
    if (state_.active || state_.stopped || state_.failed) return Fail(Reason::Lifecycle);
    if (!StopFits(q, v, length, deceleration)) return Fail(Reason::StopEnvelope);
    const double steps = std::ceil(std::fabs(v) / (deceleration * SampleSeconds));
    if (!std::isfinite(steps) || steps >= static_cast<double>((std::numeric_limits<std::uint64_t>::max)()))
        return Fail(Reason::Numeric);
    state_.q = q; state_.v = v; state_.length = length; state_.deceleration = deceleration;
    initialSpeed_ = std::fabs(v); initialQ_ = q;
    state_.active = v != 0.0; state_.stopped = !state_.active; return true;
}
inline bool StopScalar::Step() noexcept
{
    if (state_.failed || (!state_.active && !state_.stopped)) return Fail(Reason::Lifecycle);
    if (state_.stopped) return true;
    if (!StopFits(state_.q, state_.v, state_.length, state_.deceleration) ||
        state_.steps == (std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::StopEnvelope);
    // EDM115: freeze the initial scalar, then evaluate the same discrete
    // trapezoid by tick count. This avoids accumulating q/v subtraction error.
    // FMA keeps late low-speed differences at their own precision. A positive
    // residue may become zero only inside both the operation roundoff and
    // 64 eps of one D*dt decrement; its final acceleration remains inside the
    // existing Mapping roundoff (128 eps), without changing the frozen D.
    const double stepCount = static_cast<double>(state_.steps + 1ULL);
    const double decrement = state_.deceleration * SampleSeconds;
    const double removed = stepCount * decrement;
    if (!std::isfinite(removed)) return Fail(Reason::Numeric);
    const double velocityRoundoff = (std::min)(
        8.0 * std::numeric_limits<double>::epsilon() * (initialSpeed_ + removed),
        64.0 * std::numeric_limits<double>::epsilon() * decrement);
    const double remainder = std::fma(-stepCount, decrement, initialSpeed_);
    const double magnitude = remainder <= velocityRoundoff ? 0.0 : remainder;
    const double nextV = state_.v < 0.0 ? -magnitude : magnitude;
    // Sum v0/2 + v1 + ... + v(n-1) + vn/2 analytically. The second
    // term accounts for the last clipped-to-zero velocity, including a
    // fractional final decrement. Endpoint closure retains Roundoff(L).
    const double distance = (0.5 * SampleSeconds) *
        (stepCount * (initialSpeed_ + remainder) + (magnitude - remainder));
    double nextQ = initialQ_ + (state_.v < 0.0 ? -distance : distance);
    if (!std::isfinite(nextQ)) return Fail(Reason::Numeric);
    if (nextQ < 0.0 || nextQ > state_.length)
    {
        const double boundary = nextQ < 0.0 ? 0.0 : state_.length;
        if (nextV != 0.0 || std::fabs(nextQ - boundary) > Roundoff(state_.length)) return Fail(Reason::Domain);
        nextQ = boundary; // Final zero only: bounded floating point endpoint closure.
    }
    if (!StopFits(nextQ, nextV, state_.length, state_.deceleration)) return Fail(Reason::StopEnvelope);
    state_.q = nextQ; state_.v = nextV; ++state_.steps;
    state_.stopped = nextV == 0.0; state_.active = !state_.stopped; return true;
}
}
