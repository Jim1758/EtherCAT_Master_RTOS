#pragma once

#include "EDMSixAxisCommonPath.h"
#include <algorithm>

// EDM110: one bounded numerical excursion over two frozen six-axis G01 rows.
// This class owns no NC PC, epoch, lease, motor/PDO output or physical proof.
// The RT owner supplies one freshly validated settled observation per tick.
namespace EDM110
{
constexpr std::uint64_t SampleUs = 250ULL;
constexpr double SampleSeconds = 0.00025;
constexpr std::uint64_t MaximumDurationUs = 20000000ULL;
constexpr std::uint64_t MaximumDurationTicks = 80000ULL;
constexpr std::uint64_t MaximumDwellUs = 1000000ULL;
constexpr std::uint64_t MaximumDwellTicks = 4000ULL;
constexpr unsigned RequiredSettledCycles = 200U;
constexpr double PreviousRetreatFraction = 0.1;

// EDM111 repeats the same bounded executor at a second frozen G01 checkpoint.
// A schedule selects events; it does not grant geometry or output authority.
inline bool IsTriggerStep(unsigned step, bool repeat) noexcept
{ return step == 2U || (repeat && step == 4U); }
inline unsigned EventOrdinal(unsigned step) noexcept
{ return step == 2U ? 1U : step == 4U ? 2U : 0U; }

struct CompletionToken
{
    EDM109::Source previous{};
    EDM109::Completion completion{};
    std::uint64_t mappingTick = 0ULL;
    bool valid = false;
};
inline bool SameToken(const CompletionToken& a, const CompletionToken& b) noexcept
{
    const auto& x = a.previous; const auto& y = b.previous;
    return EDM109::SameScope(x.scope, y.scope) && x.dispatch == y.dispatch &&
        x.epoch == y.epoch && x.segment == y.segment && x.sourcePC == y.sourcePC &&
        x.step == y.step && a.completion.tick == b.completion.tick &&
        a.completion.commit == b.completion.commit &&
        a.completion.boundaryValidated == b.completion.boundaryValidated &&
        a.mappingTick == b.mappingTick && a.valid == b.valid;
}
inline bool EmptyToken(const CompletionToken& token) noexcept
{ return SameToken(token, CompletionToken{}); }
inline bool ValidToken(const CompletionToken& token, unsigned targetStep = 2U) noexcept
{
    const auto& p = token.previous;
    return token.valid && EDM109::ValidScope(p.scope) && p.dispatch && p.epoch &&
        p.epoch <= (std::numeric_limits<std::uint32_t>::max)() && p.segment &&
        p.sourcePC >= 0 && (targetStep == 2U || targetStep == 4U) &&
        p.step == targetStep - 1U && token.completion.boundaryValidated &&
        token.completion.tick && token.completion.commit && token.mappingTick &&
        token.mappingTick <= token.completion.tick;
}

enum class Phase : std::uint8_t
{ Idle, InitialSettle, Moving, EndpointSettle, Complete, Stopping, Stopped, Faulted };
enum class Reason : std::uint8_t
{ None, Geometry, Dynamics, Clock, Deadline, Dwell, Lifecycle, StopEnvelope };
inline const char* PhaseName(Phase phase) noexcept
{
    switch (phase) {
    case Phase::Idle: return "IDLE";
    case Phase::InitialSettle: return "INITIAL_SETTLE";
    case Phase::Moving: return "MOVING";
    case Phase::EndpointSettle: return "ENDPOINT_SETTLE";
    case Phase::Complete: return "COMPLETE";
    case Phase::Stopping: return "STOPPING";
    case Phase::Stopped: return "STOPPED";
    case Phase::Faulted: return "FAULTED";
    }
    return "INVALID";
}

struct Feedback
{
    std::uint64_t tick = 0ULL, nowUs = 0ULL, startTick = 0ULL, startUs = 0ULL;
    std::uint64_t started = 0ULL, completed = 0ULL;
    std::uint64_t backwardCrossings = 0ULL, forwardCrossings = 0ULL, anchorReturned = 0ULL;
    double q = 0.0, v = 0.0;
    double maximumVelocity = 0.0, acceleration = 0.0, deceleration = 0.0;
    unsigned span = 0U, settledCycles = 0U, completedDwells = 0U;
    Phase phase = Phase::Idle;
    Reason reason = Reason::None;
    bool geometryValid = false, active = false, moving = false, endpointReady = false;
    bool stopRequested = false, stopped = false, faulted = false, complete = false;
};

class Excursion
{
public:
    const Feedback& Current() const noexcept { return feedback_; }
    const EDM109::Line& Mapping() const noexcept { return spans_[feedback_.span < 4U ? feedback_.span : 0U]; }
    const EDM109::Point& Anchor() const noexcept { return anchor_; }

    void Clear() noexcept
    {
        for (auto& span : spans_) span = EDM109::Line{};
        for (auto& trajectory : trajectories_) trajectory = Trajectory{};
        feedback_ = Feedback{}; anchor_ = EDM109::Point{};
        phaseTick_ = phaseUs_ = 0ULL;
        stopQ_ = stopV_ = stopDuration_ = stopTarget_ = 0.0;
    }

    bool Begin(const EDM109::Line& current, const EDM109::Line& previous,
        const EDM109::Point& anchorPulse, double anchorQ, double nativeSpeedMmSec,
        double accelTimeSec, double decelTimeSec, std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        // A second Begin must not erase live or retained cancelled geometry.
        // The owner explicitly retires that geometry with Clear after drain.
        if (feedback_.geometryValid) return false;
        Clear();
        if (!EDM109::ValidLine(current) || !EDM109::ValidLine(previous) ||
            !EDM109::SamePulse(previous.endPulse, current.startPulse) ||
            !EDM109::SamePulse(previous.pulsePerNative, current.pulsePerNative) ||
            !std::isfinite(anchorQ) || anchorQ < current.pulseLength * 0.5 ||
            anchorQ >= current.pulseLength || current.lengthMm > 0.003464101616 ||
            previous.lengthMm > 0.003464101616) return Fail(Reason::Geometry);
        EDM109::Point evaluated{}, ignored{}, previousTarget{};
        if (!EDM109::EvaluateScalar(current, anchorQ, 0.0, evaluated, ignored) ||
            !EDM109::SamePulse(evaluated, anchorPulse) ||
            !EDM109::EvaluateMm(previous, previous.lengthMm * (1.0 - PreviousRetreatFraction), previousTarget))
            return Fail(Reason::Geometry);
        if (!std::isfinite(nativeSpeedMmSec) || nativeSpeedMmSec < 0.0005 || nativeSpeedMmSec > 0.001 ||
            !std::isfinite(accelTimeSec) || !std::isfinite(decelTimeSec) ||
            accelTimeSec < 0.05 || decelTimeSec < 0.05 || !tick || !nowUs)
            return Fail(Reason::Dynamics);
        if (!EDM109::BuildLine(anchorPulse, current.startPulse, current.pulsePerNative, spans_[0]) ||
            !EDM109::BuildLine(previous.endPulse, previousTarget, current.pulsePerNative, spans_[1]) ||
            !EDM109::BuildLine(previousTarget, previous.endPulse, current.pulsePerNative, spans_[2]) ||
            !EDM109::BuildLine(current.startPulse, anchorPulse, current.pulsePerNative, spans_[3]))
            return Fail(Reason::Geometry);
        double forecast = 5.0; // Five independent physical dwells, each bounded to one second.
        for (unsigned i = 0U; i < 4U; ++i)
        {
            if (!BuildTrajectory(spans_[i], nativeSpeedMmSec, accelTimeSec, decelTimeSec, trajectories_[i]))
                return Fail(Reason::Dynamics);
            forecast += trajectories_[i].duration + SampleSeconds;
        }
        if (!std::isfinite(forecast) || forecast >= static_cast<double>(MaximumDurationUs) / 1e6)
            return Fail(Reason::Deadline);
        anchor_ = anchorPulse;
        feedback_.geometryValid = feedback_.active = feedback_.endpointReady = true;
        feedback_.started = 1ULL;
        feedback_.tick = feedback_.startTick = phaseTick_ = tick;
        feedback_.nowUs = feedback_.startUs = phaseUs_ = nowUs;
        SetSpan(0U); feedback_.phase = Phase::InitialSettle;
        return true;
    }

    bool Step(std::uint64_t tick, std::uint64_t nowUs, bool settled) noexcept
    {
        if (!feedback_.active || feedback_.faulted || feedback_.stopRequested ||
            !feedback_.geometryValid || feedback_.span >= 4U) return false;
        if (!AdvanceClock(tick, nowUs)) return false;
        if (tick - feedback_.startTick > MaximumDurationTicks ||
            nowUs - feedback_.startUs > MaximumDurationUs) return Fail(Reason::Deadline);
        if (feedback_.phase == Phase::InitialSettle || feedback_.phase == Phase::EndpointSettle)
        {
            if (tick - phaseTick_ > MaximumDwellTicks || nowUs - phaseUs_ > MaximumDwellUs)
                return Fail(Reason::Dwell);
            if (feedback_.v != 0.0 || !feedback_.endpointReady) return Fail(Reason::Lifecycle);
            feedback_.settledCycles = settled ? feedback_.settledCycles + 1U : 0U;
            if (feedback_.settledCycles < RequiredSettledCycles) return true;
            ++feedback_.completedDwells;
            if (feedback_.phase == Phase::InitialSettle)
            {
                StartSpan(tick, nowUs); return true;
            }
            if (feedback_.span == 3U)
            {
                EDM109::Point returned{}, velocity{};
                if (feedback_.completedDwells != 5U || !Map(returned, velocity) ||
                    !EDM109::SamePulse(returned, anchor_)) return Fail(Reason::Geometry);
                feedback_.completed = feedback_.anchorReturned = 1ULL;
                feedback_.complete = true; feedback_.active = feedback_.moving = false;
                feedback_.phase = Phase::Complete; return true;
            }
            if (feedback_.span == 0U) ++feedback_.backwardCrossings;
            if (feedback_.span == 2U) ++feedback_.forwardCrossings;
            SetSpan(feedback_.span + 1U); StartSpan(tick, nowUs);
            return true; // The new row's exact start is held until the next tick.
        }
        if (feedback_.phase != Phase::Moving) return Fail(Reason::Lifecycle);
        const auto& trajectory = trajectories_[feedback_.span];
        const double elapsed = static_cast<double>(tick - phaseTick_) * SampleSeconds;
        if (elapsed >= trajectory.duration)
        {
            feedback_.q = Mapping().pulseLength; feedback_.v = 0.0;
            feedback_.moving = false; feedback_.endpointReady = true;
            feedback_.settledCycles = 0U; feedback_.phase = Phase::EndpointSettle;
            phaseTick_ = tick; phaseUs_ = nowUs; return true;
        }
        if (elapsed < trajectory.accelTime)
        {
            feedback_.q = 0.5 * trajectory.acceleration * elapsed * elapsed;
            feedback_.v = trajectory.acceleration * elapsed;
        }
        else if (elapsed < trajectory.accelTime + trajectory.cruiseTime)
        {
            feedback_.q = trajectory.accelDistance + trajectory.peakVelocity * (elapsed - trajectory.accelTime);
            feedback_.v = trajectory.peakVelocity;
        }
        else
        {
            const double remaining = trajectory.duration - elapsed;
            feedback_.q = Mapping().pulseLength - 0.5 * trajectory.deceleration * remaining * remaining;
            feedback_.v = trajectory.deceleration * remaining;
        }
        return ValidSample() || Fail(Reason::Dynamics);
    }

    bool RequestStop() noexcept
    {
        if (!feedback_.geometryValid || feedback_.span >= 4U) return false;
        if (feedback_.stopRequested) return true;
        if (!ValidSample()) return Fail(Reason::StopEnvelope);
        stopQ_ = feedback_.q; stopV_ = feedback_.v;
        stopDuration_ = stopV_ / feedback_.deceleration;
        const double distance = 0.5 * stopV_ * stopDuration_;
        const double remaining = Mapping().pulseLength - stopQ_;
        const double tolerance = Roundoff(Mapping().pulseLength);
        if (!std::isfinite(stopDuration_) || !std::isfinite(distance) ||
            stopDuration_ < 0.0 || distance > remaining + tolerance) return Fail(Reason::StopEnvelope);
        stopTarget_ = (std::min)(Mapping().pulseLength, stopQ_ + distance);
        phaseTick_ = feedback_.tick; phaseUs_ = feedback_.nowUs;
        feedback_.stopRequested = true; feedback_.active = false; feedback_.complete = false;
        feedback_.endpointReady = false; feedback_.settledCycles = 0U;
        feedback_.phase = Phase::Stopping;
        if (stopV_ == 0.0) FinishStop();
        return true;
    }

    bool StepStop(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (!feedback_.geometryValid || !feedback_.stopRequested || feedback_.span >= 4U) return false;
        if (!AdvanceClock(tick, nowUs)) return false;
        if (feedback_.stopped) return true;
        if (feedback_.phase != Phase::Stopping) return Fail(Reason::Lifecycle);
        const double elapsed = static_cast<double>(tick - phaseTick_) * SampleSeconds;
        if (elapsed >= stopDuration_) { FinishStop(); return true; }
        const double remaining = stopDuration_ - elapsed;
        // The same frozen row and at least the planned deceleration are used.
        // No phase promotion or seam crossing can occur on the stop path.
        feedback_.q = stopTarget_ - 0.5 * feedback_.deceleration * remaining * remaining;
        feedback_.v = feedback_.deceleration * remaining;
        const double tolerance = Roundoff(Mapping().pulseLength);
        if (feedback_.q < stopQ_ && stopQ_ - feedback_.q <= tolerance) feedback_.q = stopQ_;
        return (ValidSample() && feedback_.q >= stopQ_ && feedback_.q <= stopTarget_) ||
            Fail(Reason::StopEnvelope);
    }

    bool Map(EDM109::Point& position, EDM109::Point& velocity) const noexcept
    {
        if (!feedback_.geometryValid || !ValidSample()) return false;
        const auto& row = Mapping();
        if (!EDM109::EvaluateScalar(row, feedback_.q, feedback_.v, position, velocity)) return false;
        if (feedback_.q == 0.0) position = row.startPulse;
        else if (feedback_.q == row.pulseLength) position = row.endPulse;
        return true;
    }

private:
    struct Trajectory
    {
        double maximumVelocity = 0.0, peakVelocity = 0.0, acceleration = 0.0, deceleration = 0.0;
        double accelTime = 0.0, cruiseTime = 0.0, duration = 0.0, accelDistance = 0.0;
    };
    static double Roundoff(double length) noexcept
    { return 64.0 * std::numeric_limits<double>::epsilon() * (std::max)(1.0, length); }
    static bool BuildTrajectory(const EDM109::Line& row, double nativeSpeed,
        double accelTime, double decelTime, Trajectory& out) noexcept
    {
        Trajectory t{};
        t.maximumVelocity = nativeSpeed * row.pulseLength / row.lengthMm;
        t.acceleration = t.maximumVelocity / accelTime; t.deceleration = t.maximumVelocity / decelTime;
        if (!std::isfinite(t.maximumVelocity) || t.maximumVelocity <= 0.0 ||
            !std::isfinite(t.acceleration) || t.acceleration <= 0.0 ||
            !std::isfinite(t.deceleration) || t.deceleration <= 0.0) return false;
        const double rampDistance = 0.5 * t.maximumVelocity * (accelTime + decelTime);
        t.peakVelocity = rampDistance <= row.pulseLength ? t.maximumVelocity :
            std::sqrt(2.0 * row.pulseLength / (1.0 / t.acceleration + 1.0 / t.deceleration));
        t.accelTime = t.peakVelocity / t.acceleration;
        const double downTime = t.peakVelocity / t.deceleration;
        t.accelDistance = 0.5 * t.peakVelocity * t.accelTime;
        const double cruiseDistance = (std::max)(0.0, row.pulseLength - t.accelDistance - 0.5 * t.peakVelocity * downTime);
        t.cruiseTime = cruiseDistance / t.peakVelocity;
        t.duration = t.accelTime + t.cruiseTime + downTime;
        if (!std::isfinite(t.peakVelocity) || t.peakVelocity <= 0.0 ||
            !std::isfinite(t.duration) || t.duration <= 0.0) return false;
        out = t; return true;
    }
    void SetSpan(unsigned ordinal) noexcept
    {
        feedback_.span = ordinal; feedback_.q = feedback_.v = 0.0;
        const auto& t = trajectories_[ordinal];
        feedback_.maximumVelocity = t.maximumVelocity;
        feedback_.acceleration = t.acceleration; feedback_.deceleration = t.deceleration;
        feedback_.settledCycles = 0U;
    }
    void StartSpan(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        phaseTick_ = tick; phaseUs_ = nowUs;
        feedback_.phase = Phase::Moving; feedback_.moving = true; feedback_.endpointReady = false;
        feedback_.settledCycles = 0U;
    }
    bool AdvanceClock(std::uint64_t tick, std::uint64_t nowUs) noexcept
    {
        if (feedback_.tick == (std::numeric_limits<std::uint64_t>::max)() ||
            tick != feedback_.tick + 1ULL || nowUs <= feedback_.nowUs ||
            nowUs - feedback_.nowUs > 1500ULL) return Fail(Reason::Clock);
        feedback_.tick = tick; feedback_.nowUs = nowUs; return true;
    }
    bool ValidSample() const noexcept
    {
        return feedback_.span < 4U && Mapping().valid && std::isfinite(feedback_.q) &&
            std::isfinite(feedback_.v) && feedback_.q >= 0.0 && feedback_.q <= Mapping().pulseLength &&
            feedback_.v >= 0.0 && feedback_.v <= feedback_.maximumVelocity * (1.0 + 8.0 * std::numeric_limits<double>::epsilon()) &&
            std::isfinite(feedback_.deceleration) && feedback_.deceleration > 0.0;
    }
    void FinishStop() noexcept
    {
        feedback_.q = stopTarget_; feedback_.v = 0.0; feedback_.moving = false;
        feedback_.stopped = true; feedback_.phase = Phase::Stopped;
    }
    bool Fail(Reason reason) noexcept
    {
        feedback_.faulted = true; feedback_.reason = reason; feedback_.active = false;
        feedback_.complete = feedback_.moving = feedback_.endpointReady = false;
        feedback_.phase = Phase::Faulted; return false;
    }
    std::array<EDM109::Line, 4U> spans_{};
    std::array<Trajectory, 4U> trajectories_{};
    EDM109::Point anchor_{};
    Feedback feedback_{};
    std::uint64_t phaseTick_ = 0ULL, phaseUs_ = 0ULL;
    double stopQ_ = 0.0, stopV_ = 0.0, stopDuration_ = 0.0, stopTarget_ = 0.0;
};
static_assert(std::is_trivially_copyable<CompletionToken>::value, "EDM110 completion token must be fixed-value");
static_assert(std::is_trivially_copyable<Feedback>::value && sizeof(Feedback) <= 512U,
    "EDM110 bounded feedback publication");
static_assert(std::is_trivially_copyable<Excursion>::value && sizeof(Excursion) <= 1536U,
    "EDM110 fixed RT excursion workspace");
}
