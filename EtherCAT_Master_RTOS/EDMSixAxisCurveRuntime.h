#pragma once

#include "EDMSixAxisCommonCurve.h"

// EDM113 observes the committed G01 scalar against a cached, hypothetical G17
// semicircle. It never changes a command or grants physical curve authority.
// All axis budgets are frozen software/configuration limits, not motor ratings.
namespace EDM113
{
using Point = EDM112::Point;
using Source = EDM109::Source;
constexpr double SampleSeconds = 0.000250;
constexpr unsigned RequiredRows = 16U;
enum class Reason : std::uint8_t
{
    None, Lifecycle, Session, Scope, Sequence, Geometry, Limits, Clock, Domain,
    Numeric, Velocity, Acceleration, StopEnvelope, Pause, Completion, Cancelled,
    Dynamics, Output
};
struct Limits
{
    Point maxVelocityPps{}, maxAccelerationPps2{};
    // Actual planner bounds, including its effective acceleration/deceleration
    // cap. These are source G01 pulse units, not authored arc feed.
    double scalarMaxVelocity = 0.0, scalarAcceleration = 0.0, scalarDeceleration = 0.0;
};
struct Feedback
{
    std::uint64_t session = 0ULL, armEpoch = 0ULL, ownerGeneration = 0ULL;
    Source source{};
    std::uint64_t bindTick = 0ULL, lastTick = 0ULL, lastSampleTick = 0ULL;
    std::uint64_t lastCompletedMappingTick = 0ULL;
    std::uint64_t samples = 0ULL, forwardSamples = 0ULL, reverseSamples = 0ULL;
    std::uint64_t flushSamples = 0ULL, suspendedTicks = 0ULL;
    std::uint64_t rowSamples = 0ULL, rowForwardSamples = 0ULL, rowReverseSamples = 0ULL;
    std::uint64_t rowFlushSamples = 0ULL, rowSuspendedTicks = 0ULL;
    Point maxAbsVelocityPps{}, maxAbsAccelerationPps2{};
    double maxUnitSpeed = 0.0, maxUnitAcceleration = 0.0;
    double unitSpeedLimit = 0.0, unitAccelerationLimit = 0.0, sourcePulseLength = 0.0;
    unsigned begunRows = 0U, completedRows = 0U, cwRows = 0U, ccwRows = 0U;
    unsigned endpointRows = 0U, suspensions = 0U, resumptions = 0U;
    Reason reason = Reason::None;
    bool enabled = false, failed = false, completed = false, rowComplete = false, suspended = false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
};
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

class Observer
{
public:
    const Feedback& Get() const noexcept { return feedback_; }
    constexpr bool PhysicalPermit() const noexcept { return false; }
    // Clear retires the session. It cannot make a cancelled session reusable.
    void Clear() noexcept
    {
        feedback_ = Feedback{}; curve_ = EDM112::Curve{}; limits_ = Limits{};
        lastQ_ = lastW_ = heldQ_ = 0.0; sampled_ = rowPaused_ = false;
    }
    bool Begin(std::uint64_t session, std::uint64_t armEpoch = 0ULL,
        std::uint64_t ownerGeneration = 0ULL) noexcept
    {
        if (!session || session <= retiredSession_) return Reject(Reason::Session, 0ULL);
        Clear(); retiredSession_ = session; feedback_.session = session;
        feedback_.armEpoch = armEpoch; feedback_.ownerGeneration = ownerGeneration;
        feedback_.enabled = true; return true;
    }
    bool Reject(Reason reason, std::uint64_t tick = 0ULL) noexcept
    {
        if (!feedback_.failed)
        {
            feedback_.failed = true; feedback_.reason = reason == Reason::None ? Reason::Lifecycle : reason;
            feedback_.completed = false;
            if (tick) feedback_.lastTick = tick;
        }
        return false;
    }
    bool Bind(const EDM109::Line& line, const Source& source, const Limits& limits,
        std::uint64_t tick) noexcept
    {
        if (!feedback_.enabled || feedback_.failed || feedback_.completed || feedback_.suspended ||
            feedback_.begunRows != feedback_.completedRows || feedback_.completedRows >= RequiredRows)
            return Reject(Reason::Lifecycle, tick);
        if (!ValidSource(source) || source.scope.session != feedback_.session ||
            (feedback_.ownerGeneration && source.scope.ownerGeneration != feedback_.ownerGeneration))
            return Reject(Reason::Scope, tick);
        if (!tick || source.step != feedback_.completedRows) return Reject(Reason::Sequence, tick);
        if (feedback_.begunRows)
        {
            const auto& prior = feedback_.source;
            if (!feedback_.rowComplete || !EDM109::SameScope(source.scope, prior.scope) ||
                source.dispatch <= prior.dispatch || source.epoch <= prior.epoch || source.segment <= prior.segment ||
                source.sourcePC <= prior.sourcePC || tick <= feedback_.lastTick ||
                !EDM109::SamePulse(curve_.line.endPulse, line.startPulse) ||
                !EDM109::SamePulse(curve_.line.pulsePerNative, line.pulsePerNative))
                return Reject(Reason::Sequence, tick);
        }
        if (!EDM109::ValidLine(line)) return Reject(Reason::Geometry, tick);
        std::array<double, 2U> center{{
            0.5 * ((line.endPulse[0U] - line.startPulse[0U]) / line.pulsePerNative[0U]),
            0.5 * ((line.endPulse[1U] - line.startPulse[1U]) / line.pulsePerNative[1U])}};
        if (!EDM112::BuildArc(line.startPulse, line.endPulse, line.pulsePerNative, 17U,
            center, (source.step & 1U) ? 1 : -1, false, curve_)) return Reject(Reason::Geometry, tick);
        if (!Positive(limits.scalarMaxVelocity) || !Positive(limits.scalarAcceleration) ||
            !Positive(limits.scalarDeceleration)) return Reject(Reason::Limits, tick);
        double wLimit = (std::numeric_limits<double>::max)();
        double aLimit = (std::numeric_limits<double>::max)();
        for (unsigned i = 0U; i < 6U; ++i)
        {
            if (!Positive(limits.maxVelocityPps[i]) || !Positive(limits.maxAccelerationPps2[i]))
                return Reject(Reason::Limits, tick);
            const double d1 = i < 2U ? curve_.radiusMm * std::fabs(curve_.sweepRadians) * line.pulsePerNative[i] :
                std::fabs(line.endPulse[i] - line.startPulse[i]);
            const double d2 = i < 2U ? d1 * std::fabs(curve_.sweepRadians) : 0.0;
            if (!std::isfinite(d1) || !std::isfinite(d2)) return Reject(Reason::Numeric, tick);
            if (d1 > 0.0)
            {
                wLimit = (std::min)(wLimit, limits.maxVelocityPps[i] / d1);
                aLimit = (std::min)(aLimit, (0.5 * limits.maxAccelerationPps2[i]) / d1);
            }
            if (d2 > 0.0)
                wLimit = (std::min)(wLimit, std::sqrt((0.5 * limits.maxAccelerationPps2[i]) / d2));
        }
        if (!Positive(wLimit) || !Positive(aLimit) ||
            !Within(limits.scalarMaxVelocity / line.pulseLength, wLimit) ||
            !Within((std::max)(limits.scalarAcceleration, limits.scalarDeceleration) / line.pulseLength, aLimit))
            return Reject(Reason::Dynamics, tick);
        limits_ = limits; feedback_.source = source; feedback_.sourcePulseLength = line.pulseLength;
        feedback_.unitSpeedLimit = wLimit; feedback_.unitAccelerationLimit = aLimit;
        feedback_.bindTick = feedback_.lastTick = tick; feedback_.lastSampleTick = 0ULL;
        feedback_.lastCompletedMappingTick = 0ULL; ++feedback_.begunRows;
        feedback_.rowSamples = feedback_.rowForwardSamples = feedback_.rowReverseSamples = 0ULL;
        feedback_.rowFlushSamples = feedback_.rowSuspendedTicks = 0ULL;
        feedback_.rowComplete = false; lastQ_ = lastW_ = heldQ_ = 0.0;
        sampled_ = rowPaused_ = false; return true;
    }
    bool Sample(const Source& source, double q, double v, std::uint64_t tick, bool flush = false) noexcept
    {
        if (!Current(source, tick)) return false;
        if (feedback_.rowComplete) return Reject(Reason::Lifecycle, tick);
        const bool resuming = feedback_.suspended;
        if (!Clock(tick, resuming)) return Reject(Reason::Clock, tick);
        if (!std::isfinite(q) || !std::isfinite(v) || q < 0.0 || q > feedback_.sourcePulseLength)
            return Reject(Reason::Domain, tick);
        if (resuming && (q != heldQ_ || v != 0.0)) return Reject(Reason::Pause, tick);
        const double u = q == feedback_.sourcePulseLength ? 1.0 : q / feedback_.sourcePulseLength;
        const double w = v / feedback_.sourcePulseLength;
        const double a = resuming ? 0.0 : (w - lastW_) / SampleSeconds;
        if (!std::isfinite(u) || !std::isfinite(w) || !std::isfinite(a)) return Reject(Reason::Numeric, tick);
        if (!Within(std::fabs(w), feedback_.unitSpeedLimit) ||
            !Within(std::fabs(w), limits_.scalarMaxVelocity / feedback_.sourcePulseLength))
            return Reject(Reason::Velocity, tick);
        const double sourceA = (std::max)(limits_.scalarAcceleration, limits_.scalarDeceleration) / feedback_.sourcePulseLength;
        // Differencing two rounded speeds magnifies roundoff by 1/dt. Keep
        // this explicit, bounded allowance separate from the software budget.
        const double accelerationRoundoff = Roundoff() * ((std::max)(1.0,
            (std::max)(std::fabs(lastW_), std::fabs(w))) / SampleSeconds + sourceA);
        if (std::fabs(a) > feedback_.unitAccelerationLimit + accelerationRoundoff ||
            std::fabs(a) > sourceA + accelerationRoundoff) return Reject(Reason::Acceleration, tick);
        if (!resuming && !IntegralFits(q, w)) return Reject(Reason::Domain, tick);
        if (!StopFits(u, w)) return Reject(Reason::StopEnvelope, tick);
        Point p{}, d1{}, d2{};
        if (!EvaluateCached(u, p, d1, d2)) return Reject(Reason::Numeric, tick);
        Point velocities{}, accelerations{};
        for (unsigned i = 0U; i < 6U; ++i)
        {
            velocities[i] = std::fabs(d1[i] * w);
            accelerations[i] = std::fabs(d1[i] * a + d2[i] * w * w);
            if (!std::isfinite(velocities[i]) || !std::isfinite(accelerations[i])) return Reject(Reason::Numeric, tick);
            if (!Within(velocities[i], limits_.maxVelocityPps[i])) return Reject(Reason::Velocity, tick);
            if (!Within(accelerations[i], limits_.maxAccelerationPps2[i])) return Reject(Reason::Acceleration, tick);
        }
        if ((u == 0.0 && !EDM109::SamePulse(p, curve_.line.startPulse)) ||
            (u == 1.0 && !EDM109::SamePulse(p, curve_.line.endPulse))) return Reject(Reason::Geometry, tick);
        for (unsigned i = 0U; i < 6U; ++i)
        {
            feedback_.maxAbsVelocityPps[i] = (std::max)(feedback_.maxAbsVelocityPps[i], velocities[i]);
            feedback_.maxAbsAccelerationPps2[i] = (std::max)(feedback_.maxAbsAccelerationPps2[i], accelerations[i]);
        }
        feedback_.maxUnitSpeed = (std::max)(feedback_.maxUnitSpeed, std::fabs(w));
        feedback_.maxUnitAcceleration = (std::max)(feedback_.maxUnitAcceleration, std::fabs(a));
        ++feedback_.samples; ++feedback_.rowSamples;
        if (w > 0.0) { ++feedback_.forwardSamples; ++feedback_.rowForwardSamples; }
        if (w < 0.0) { ++feedback_.reverseSamples; ++feedback_.rowReverseSamples; }
        if (flush) { ++feedback_.flushSamples; ++feedback_.rowFlushSamples; }
        if (resuming) { feedback_.suspended = false; ++feedback_.resumptions; }
        lastQ_ = q; lastW_ = w; sampled_ = true;
        feedback_.lastTick = feedback_.lastSampleTick = tick; return true;
    }
    // q/v are the frozen ORIGINAL G01 checkpoint, never an external B2 span's
    // scalar. A suspension is accounted each RT tick but is not a mapped sample.
    bool Suspend(const Source& source, double q, double v, std::uint64_t tick) noexcept
    {
        if (!Current(source, tick)) return false;
        if (feedback_.rowComplete || !sampled_ || !std::isfinite(q) || q <= 0.0 ||
            q >= feedback_.sourcePulseLength || v != 0.0 || (source.step != 2U && source.step != 4U))
            return Reject(Reason::Pause, tick);
        if (feedback_.suspended)
        {
            if (!Clock(tick, false)) return Reject(Reason::Clock, tick);
            if (q != heldQ_) return Reject(Reason::Pause, tick);
        }
        else
        {
            if (rowPaused_ || !Clock(tick, true)) return Reject(Reason::Pause, tick);
            if (tick == feedback_.lastTick)
            { if (q != lastQ_ || lastW_ != 0.0) return Reject(Reason::Pause, tick); }
            else
            {
                const double a = (std::max)(limits_.scalarAcceleration, limits_.scalarDeceleration) / feedback_.sourcePulseLength;
                const double tolerance = Roundoff() * ((std::max)(1.0, std::fabs(lastW_)) / SampleSeconds + a);
                if (!IntegralFits(q, 0.0) || std::fabs(lastW_) / SampleSeconds > a + tolerance ||
                    std::fabs(lastW_) / SampleSeconds > feedback_.unitAccelerationLimit + tolerance)
                    return Reject(Reason::Pause, tick);
            }
            heldQ_ = q; rowPaused_ = feedback_.suspended = true; ++feedback_.suspensions;
        }
        if (tick != feedback_.lastTick) { ++feedback_.suspendedTicks; ++feedback_.rowSuspendedTicks; }
        lastQ_ = q; lastW_ = 0.0; feedback_.lastTick = tick; return true;
    }
    bool Suspend(const Source& source, std::uint64_t tick) noexcept
    { return Suspend(source, lastQ_, 0.0, tick); }
    bool Complete(const Source& source, std::uint64_t tick) noexcept
    {
        if (!Current(source, tick)) return false;
        if (feedback_.rowComplete)
            return tick == feedback_.lastCompletedMappingTick ? true : Reject(Reason::Completion, tick);
        if (feedback_.suspended || !sampled_ || tick != feedback_.lastSampleTick || tick != feedback_.lastTick ||
            lastQ_ != feedback_.sourcePulseLength || lastW_ != 0.0 || !feedback_.rowForwardSamples ||
            !feedback_.rowReverseSamples || (!(source.step & 1U) && !feedback_.rowFlushSamples) ||
            ((source.step == 2U || source.step == 4U) && (!rowPaused_ || !feedback_.rowSuspendedTicks)) ||
            feedback_.suspensions != feedback_.resumptions)
            return Reject(Reason::Completion, tick);
        feedback_.rowComplete = true; feedback_.lastCompletedMappingTick = tick;
        ++feedback_.completedRows; ++feedback_.endpointRows;
        if (source.step & 1U) ++feedback_.ccwRows; else ++feedback_.cwRows;
        if (feedback_.completedRows == RequiredRows)
        {
            if (feedback_.begunRows != RequiredRows || feedback_.cwRows != 8U || feedback_.ccwRows != 8U ||
                feedback_.endpointRows != RequiredRows || feedback_.suspensions != 2U || feedback_.resumptions != 2U)
                return Reject(Reason::Completion, tick);
            feedback_.completed = true;
        }
        return true;
    }
private:
    static bool Positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
    static double Roundoff() noexcept { return 64.0 * (std::numeric_limits<double>::epsilon)(); }
    static bool Within(double x, double limit) noexcept
    { return std::isfinite(x) && std::isfinite(limit) && x <= limit + Roundoff() * (std::max)(1.0, std::fabs(limit)); }
    bool Current(const Source& source, std::uint64_t tick) noexcept
    {
        if (!feedback_.enabled || feedback_.failed || !feedback_.begunRows) return Reject(Reason::Lifecycle, tick);
        if (!ValidSource(source) || !SameSource(source, feedback_.source)) return Reject(Reason::Scope, tick);
        return true;
    }
    bool Clock(std::uint64_t tick, bool allowSame) const noexcept
    {
        if (!tick) return false;
        if ((!sampled_ || allowSame) && tick == feedback_.lastTick) return true;
        return feedback_.lastTick != (std::numeric_limits<std::uint64_t>::max)() && tick == feedback_.lastTick + 1ULL;
    }
    bool StopFits(double u, double w) const noexcept
    {
        const double speed = std::fabs(w);
        if (speed == 0.0) return true;
        const double dec = limits_.scalarDeceleration / feedback_.sourcePulseLength;
        const double drop = dec * SampleSeconds;
        if (!Positive(dec) || !Positive(drop)) return false;
        const double residual = std::fmod(speed, drop);
        const double distance = 0.5 * speed * (speed / dec) +
            0.5 * SampleSeconds * residual * (1.0 - residual / drop);
        return std::isfinite(distance) && distance <= (w > 0.0 ? 1.0 - u : u) + Roundoff();
    }
    bool IntegralFits(double q, double w) const noexcept
    {
        const double current = q / feedback_.sourcePulseLength;
        const double expected = lastQ_ / feedback_.sourcePulseLength + 0.5 * (lastW_ + w) * SampleSeconds;
        // The scalar planner fences/snaps in normalized progress at 64 eps;
        // another 64 eps covers the pulse multiplication/division round trip.
        return std::isfinite(current) && std::isfinite(expected) &&
            std::fabs(current - expected) <= 2.0 * Roundoff();
    }
    // The private canonical value was built once by Bind. No per-tick rebuild,
    // geometry mutation, allocation, publication or command access occurs here.
    bool EvaluateCached(double u, Point& p, Point& d1, Point& d2) const noexcept
    {
        for (unsigned i = 0U; i < 6U; ++i)
        {
            d1[i] = curve_.line.endPulse[i] - curve_.line.startPulse[i];
            p[i] = d1[i] == 0.0 ? curve_.line.startPulse[i] : curve_.line.startPulse[i] + d1[i] * u;
        }
        const double angle = curve_.startAngle + u * curve_.sweepRadians;
        const double c = std::cos(angle), s = std::sin(angle);
        for (unsigned i = 0U; i < 2U; ++i)
        {
            const double trig = i ? s : c;
            p[i] = curve_.line.startPulse[i] + (curve_.centerOffsetMm[i] + curve_.radiusMm * trig) * curve_.line.pulsePerNative[i];
            d1[i] = ((i ? c : -s) * curve_.radiusMm * curve_.sweepRadians) * curve_.line.pulsePerNative[i];
            d2[i] = ((-trig * curve_.radiusMm * curve_.sweepRadians) * curve_.sweepRadians) * curve_.line.pulsePerNative[i];
        }
        if (u == 0.0) p = curve_.line.startPulse;
        else if (u == 1.0) p = curve_.line.endPulse;
        for (unsigned i = 0U; i < 6U; ++i)
            if (!std::isfinite(p[i]) || !std::isfinite(d1[i]) || !std::isfinite(d2[i])) return false;
        return true;
    }
    Feedback feedback_{};
    EDM112::Curve curve_{};
    Limits limits_{};
    std::uint64_t retiredSession_ = 0ULL;
    double lastQ_ = 0.0, lastW_ = 0.0, heldQ_ = 0.0;
    bool sampled_ = false, rowPaused_ = false;
};
static_assert(std::is_trivially_copyable<Feedback>::value, "EDM113 fixed feedback");
static_assert(std::is_trivially_copyable<Observer>::value, "EDM113 fixed observer");
static_assert(sizeof(Feedback) <= 512U, "EDM113 feedback channel bound");
static_assert(sizeof(Observer) <= 1024U, "EDM113 fixed observer bound");
}
