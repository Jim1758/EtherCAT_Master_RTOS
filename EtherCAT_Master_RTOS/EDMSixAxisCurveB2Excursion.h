#pragma once

#include "EDMSixAxisArcExecution.h"
#include "EDMSixAxisB2Excursion.h"

// EDM115: bounded traversal of two already-admitted canonical mappings.
// This value owns no NC cursor, owner, epoch, motor/PDO or physical proof.
// The caller validates the supplied settled observation on every fresh tick.
namespace EDM115
{
using Point = EDM114::Point;
using Source = EDM109::Source;
using CompletionToken = EDM110::CompletionToken;
constexpr unsigned ExpectedEvents = 2U, ExpectedDwells = 10U;
constexpr unsigned RequiredSettledCycles = 200U;
constexpr std::uint64_t MaximumDurationUs = 20000000ULL, MaximumDurationTicks = 80000ULL;
constexpr std::uint64_t MaximumDwellUs = 1000000ULL, MaximumDwellTicks = 4000ULL;
constexpr double SampleSeconds = EDM114::SampleSeconds, PreviousRetreatFraction = 0.1;
inline bool IsTriggerStep(unsigned step) noexcept { return step == 2U || step == 4U; }
inline unsigned EventOrdinal(unsigned step) noexcept { return step == 2U ? 1U : step == 4U ? 2U : 0U; }
inline bool SameToken(const CompletionToken& a, const CompletionToken& b) noexcept { return EDM110::SameToken(a,b); }
inline bool EmptyToken(const CompletionToken& token) noexcept { return EDM110::EmptyToken(token); }
inline bool ValidToken(const CompletionToken& token, unsigned step) noexcept { return EDM110::ValidToken(token,step); }
enum class TraversalSchedule : std::uint8_t { Mixed115 = 0U, ArcPair124 = 1U };
enum class Phase : std::uint8_t { Idle, InitialSettle, Moving, EndpointSettle, Complete, Stopping, Stopped, Faulted };
enum class Reason : std::uint8_t { None, Geometry, Source, Dynamics, Clock, Deadline, Dwell, Lifecycle, StopEnvelope };
inline const char* PhaseName(Phase phase) noexcept
{
    switch (phase) {
    case Phase::Idle: return "IDLE"; case Phase::InitialSettle: return "INITIAL_SETTLE";
    case Phase::Moving: return "MOVING"; case Phase::EndpointSettle: return "ENDPOINT_SETTLE";
    case Phase::Complete: return "COMPLETE"; case Phase::Stopping: return "STOPPING";
    case Phase::Stopped: return "STOPPED"; case Phase::Faulted: return "FAULTED";
    }
    return "INVALID";
}
struct Feedback
{
    std::uint64_t tick = 0ULL, nowUs = 0ULL, startTick = 0ULL, startUs = 0ULL;
    std::uint64_t started = 0ULL, completed = 0ULL;
    std::uint64_t backwardCrossings = 0ULL, forwardCrossings = 0ULL, anchorReturned = 0ULL;
    double q = 0.0, v = 0.0, a = 0.0; // Original selected row's signed scalar.
    double progress = 0.0, length = 0.0; // Positive local span coordinate.
    double maximumVelocity = 0.0, acceleration = 0.0, deceleration = 0.0;
    unsigned span = 0U, settledCycles = 0U, completedDwells = 0U;
    int direction = 0;
    EDM114::Kind rowKind = EDM114::Kind::None;
    Phase phase = Phase::Idle;
    Reason reason = Reason::None;
    bool geometryValid = false, active = false, moving = false, endpointReady = false;
    bool stopRequested = false, stopped = false, faulted = false, complete = false;
};

class Excursion
{
public:
    const Feedback& Current() const noexcept { return feedback_; }
    const EDM114::Mapping& CurrentMapping() const noexcept { return rows_[RowIndex()]; }
    const Source& CurrentSource() const noexcept { return sources_[RowIndex()]; }
    const Point& Anchor() const noexcept { return anchor_; }
    const CompletionToken& PreviousToken() const noexcept { return previousToken_; }
    const EDM114::Mapping& OriginalMapping() const noexcept { return rows_[0]; }
    const EDM114::Mapping& PreviousMapping() const noexcept { return rows_[1]; }
    double AnchorQ() const noexcept { return anchorQ_; }
    void Clear() noexcept
    {
        rows_[0].Clear(); rows_[1].Clear(); sources_ = {};
        previousToken_ = CompletionToken{}; spans_ = {}; anchor_ = Point{};
        feedback_ = Feedback{}; stop_.Clear(); anchorQ_ = 0.0;
        phaseTick_ = phaseUs_ = 0ULL;
    }
    bool Begin(const EDM114::Mapping& current, const EDM114::Mapping& previous,
        const Source& currentSource, const CompletionToken& previousToken,
        const Point& anchorPulse, double anchorQ, double nativeSpeedMmSec,
        double accelTimeSec, double decelTimeSec, std::uint64_t tick, std::uint64_t nowUs,
        TraversalSchedule schedule = TraversalSchedule::Mixed115) noexcept;
    bool Step(std::uint64_t tick, std::uint64_t nowUs, bool physicalSettled) noexcept;
    bool RequestStop(std::uint64_t firstStopTick = 0ULL, std::uint64_t firstStopUs = 0ULL) noexcept;
    bool StepStop(std::uint64_t tick, std::uint64_t nowUs) noexcept;
    bool Map(Point& position, Point& velocity, Point* acceleration = nullptr) const noexcept;
private:
    struct Span
    {
        double from = 0.0, to = 0.0, length = 0.0;
        double peak = 0.0, acceleration = 0.0, deceleration = 0.0;
        std::uint64_t upTicks = 0ULL, cruiseTicks = 0ULL, downTicks = 0ULL;
    };
    unsigned RowIndex() const noexcept { return feedback_.span == 1U || feedback_.span == 2U ? 1U : 0U; }
    static double NativeLength(const EDM114::Mapping& mapping) noexcept;
    static bool BuildSpan(const EDM114::Mapping& mapping, double from, double to,
        double nativeSpeed, double accelTime, double decelTime, Span& span) noexcept;
    bool SetSpan(unsigned ordinal) noexcept;
    void StartSpan() noexcept;
    bool SetSample(double progress, double positiveVelocity) noexcept;
    bool ValidSample() const noexcept;
    bool AdvanceClock(std::uint64_t tick, std::uint64_t nowUs) noexcept;
    bool Fail(Reason reason) noexcept
    {
        if (!feedback_.faulted) feedback_.reason = reason;
        feedback_.faulted = true; feedback_.active = feedback_.complete = feedback_.moving = feedback_.endpointReady = false;
        feedback_.phase = Phase::Faulted; return false;
    }
    std::array<EDM114::Mapping, 2U> rows_{};
    std::array<Source, 2U> sources_{};
    CompletionToken previousToken_{};
    std::array<Span, 4U> spans_{};
    Point anchor_{};
    double anchorQ_ = 0.0;
    Feedback feedback_{};
    EDM114::StopScalar stop_{};
    std::uint64_t phaseTick_ = 0ULL, phaseUs_ = 0ULL;
};

inline double Excursion::NativeLength(const EDM114::Mapping& mapping) noexcept
{
    const auto& g = mapping.GetGeometry();
    if (!g.valid) return 0.0;
    if (g.kind == EDM114::Kind::Arc) return g.pulseLength / g.pulsePerNative[0];
    if (g.kind != EDM114::Kind::Line) return 0.0;
    double length = 0.0;
    for (unsigned i = 0U; i < 3U; ++i)
        length = std::hypot(length, (g.endPulse[i] - g.startPulse[i]) / g.pulsePerNative[i]);
    return length;
}
inline bool Excursion::BuildSpan(const EDM114::Mapping& mapping, double from, double to,
    double nativeSpeed, double accelTime, double decelTime, Span& span) noexcept
{
    const auto& g = mapping.GetGeometry(); const auto& limits = mapping.FrozenLimits();
    const double nativeLength = NativeLength(mapping);
    if (!EDM114::Positive(nativeLength) || nativeLength > 0.003464101617 ||
        !std::isfinite(from) || !std::isfinite(to) || from < 0.0 || to < 0.0 ||
        from > g.pulseLength || to > g.pulseLength || from == to) return false;
    Span candidate{}; candidate.from = from; candidate.to = to; candidate.length = std::fabs(to-from);
    const double maximum = (std::min)(nativeSpeed * (g.pulseLength / nativeLength), limits.scalarMaxVelocity);
    const double a = (std::min)(maximum / accelTime, limits.scalarAcceleration);
    const double d = (std::min)(maximum / decelTime, limits.scalarDeceleration);
    if (!EDM114::Positive(maximum) || !EDM114::Positive(a) || !EDM114::Positive(d)) return false;
    const double triangular = std::sqrt(2.0 * candidate.length / (1.0 / a + 1.0 / d));
    const double peak = (std::min)(maximum, triangular);
    const double up = (std::max)(1.0, std::ceil((peak/a) / SampleSeconds));
    const double down = (std::max)(1.0, std::ceil((peak/d) / SampleSeconds));
    const double cruise = (std::max)(0.0, std::ceil((candidate.length / peak) / SampleSeconds - 0.5*(up+down)));
    const double total = up + cruise + down;
    if (!EDM114::Positive(peak) || !std::isfinite(total) || total >= static_cast<double>(MaximumDurationTicks)) return false;
    candidate.upTicks = static_cast<std::uint64_t>(up);
    candidate.downTicks = static_cast<std::uint64_t>(down);
    candidate.cruiseTicks = static_cast<std::uint64_t>(cruise);
    candidate.peak = candidate.length / ((0.5*(up+down)+cruise)*SampleSeconds);
    candidate.acceleration = candidate.peak / (up * SampleSeconds);
    candidate.deceleration = candidate.peak / (down * SampleSeconds);
    if (!EDM114::Within(candidate.peak, maximum) || !EDM114::Within(candidate.acceleration, a) ||
        !EDM114::Within(candidate.deceleration, d)) return false;
    span = candidate; return true;
}
inline bool Excursion::Begin(const EDM114::Mapping& current, const EDM114::Mapping& previous,
    const Source& currentSource, const CompletionToken& previousToken,
    const Point& anchorPulse, double anchorQ, double nativeSpeedMmSec,
    double accelTimeSec, double decelTimeSec, std::uint64_t tick, std::uint64_t nowUs,
    TraversalSchedule schedule) noexcept
{
    // A request cannot erase live, completed or interrupted retained geometry.
    if (feedback_.geometryValid || feedback_.faulted) return false;
    Clear(); const auto& c = current.GetGeometry(); const auto& p = previous.GetGeometry();
    if ((schedule != TraversalSchedule::Mixed115 && schedule != TraversalSchedule::ArcPair124) ||
        !EDM115::IsTriggerStep(currentSource.step) || !EDM114::ValidSource(currentSource) || !EDM115::ValidToken(previousToken,currentSource.step) ||
        !EDM109::SameScope(currentSource.scope,previousToken.previous.scope) ||
        currentSource.dispatch <= previousToken.previous.dispatch || currentSource.epoch <= previousToken.previous.epoch ||
        currentSource.segment <= previousToken.previous.segment || currentSource.sourcePC <= previousToken.previous.sourcePC ||
        previousToken.completion.tick >= tick) return Fail(Reason::Source);
    const bool adjacentArcs = schedule == TraversalSchedule::ArcPair124 && currentSource.step == 4U &&
        previousToken.previous.step == 3U;
    if (!c.valid || !p.valid ||
        c.kind != ((adjacentArcs || EDM114::IsArcStep(currentSource.step)) ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
        p.kind != (EDM114::IsArcStep(previousToken.previous.step) ? EDM114::Kind::Arc : EDM114::Kind::Line) ||
        (adjacentArcs ? c.kind != EDM114::Kind::Arc || p.kind != EDM114::Kind::Arc : c.kind == p.kind) ||
        !EDM109::SamePulse(p.endPulse,c.startPulse) ||
        !EDM109::SamePulse(p.pulsePerNative,c.pulsePerNative) || !std::isfinite(anchorQ) ||
        anchorQ < c.pulseLength*0.5 || anchorQ >= c.pulseLength) return Fail(Reason::Geometry);
    Point position{},velocity{},acceleration{};
    if (!current.Eval(anchorQ,0.0,0.0,position,velocity,acceleration) ||
        !EDM109::SamePulse(position,anchorPulse) || !previous.Eval(p.pulseLength,0.0,0.0,position,velocity,acceleration) ||
        !EDM109::SamePulse(position,c.startPulse)) return Fail(Reason::Geometry);
    if (!std::isfinite(nativeSpeedMmSec) || nativeSpeedMmSec < 0.0005 || nativeSpeedMmSec > 0.001 ||
        !std::isfinite(accelTimeSec) || !std::isfinite(decelTimeSec) || accelTimeSec < 0.05 || decelTimeSec < 0.05 ||
        !tick || !nowUs) return Fail(Reason::Dynamics);
    rows_[0] = current; rows_[1] = previous; sources_[0] = currentSource; sources_[1] = previousToken.previous;
    previousToken_ = previousToken; anchor_ = anchorPulse; anchorQ_ = anchorQ;
    const double retreat = p.pulseLength*(1.0-PreviousRetreatFraction);
    if (!BuildSpan(current,anchorQ,0.0,nativeSpeedMmSec,accelTimeSec,decelTimeSec,spans_[0]) ||
        !BuildSpan(previous,p.pulseLength,retreat,nativeSpeedMmSec,accelTimeSec,decelTimeSec,spans_[1]) ||
        !BuildSpan(previous,retreat,p.pulseLength,nativeSpeedMmSec,accelTimeSec,decelTimeSec,spans_[2]) ||
        !BuildSpan(current,0.0,anchorQ,nativeSpeedMmSec,accelTimeSec,decelTimeSec,spans_[3])) return Fail(Reason::Dynamics);
    std::uint64_t forecast = 5ULL * MaximumDwellTicks;
    for (const auto& s:spans_) forecast += s.upTicks+s.cruiseTicks+s.downTicks+1ULL;
    if (forecast >= MaximumDurationTicks) return Fail(Reason::Deadline);
    feedback_.geometryValid = feedback_.active = feedback_.endpointReady = true;
    feedback_.started = 1ULL; feedback_.tick = feedback_.startTick = phaseTick_ = tick;
    feedback_.nowUs = feedback_.startUs = phaseUs_ = nowUs;
    if (!SetSpan(0U)) return false;
    feedback_.phase = Phase::InitialSettle; return true;
}
inline bool Excursion::SetSpan(unsigned ordinal) noexcept
{
    if (ordinal >= 4U) return Fail(Reason::Lifecycle);
    feedback_.span = ordinal; const auto& s=spans_[ordinal];
    feedback_.q=s.from;feedback_.v=feedback_.a=feedback_.progress=0.0;feedback_.length=s.length;
    feedback_.direction=s.to>s.from?1:-1;feedback_.rowKind=CurrentMapping().GetGeometry().kind;
    feedback_.maximumVelocity=s.peak;feedback_.acceleration=s.acceleration;feedback_.deceleration=s.deceleration;
    feedback_.settledCycles=0U;
    return ValidSample() || Fail(Reason::Geometry);
}
inline void Excursion::StartSpan() noexcept
{
    phaseTick_=feedback_.tick;phaseUs_=feedback_.nowUs;
    feedback_.phase=Phase::Moving;feedback_.moving=true;feedback_.endpointReady=false;feedback_.settledCycles=0U;
}
inline bool Excursion::AdvanceClock(std::uint64_t tick,std::uint64_t nowUs) noexcept
{
    if (feedback_.tick==(std::numeric_limits<std::uint64_t>::max)() || tick!=feedback_.tick+1ULL ||
        nowUs<=feedback_.nowUs || nowUs-feedback_.nowUs>1500ULL) return Fail(Reason::Clock);
    feedback_.tick=tick;feedback_.nowUs=nowUs;return true;
}
inline bool Excursion::SetSample(double progress,double positiveVelocity) noexcept
{
    const auto& s=spans_[feedback_.span];
    const double signedVelocity=feedback_.direction*positiveVelocity;
    feedback_.a=(signedVelocity-feedback_.v)/SampleSeconds;
    feedback_.progress=progress;feedback_.v=signedVelocity;
    feedback_.q=progress==0.0?s.from:progress==s.length?s.to:s.from+feedback_.direction*progress;
    return ValidSample() || Fail(Reason::Dynamics);
}
inline bool Excursion::ValidSample() const noexcept
{
    if (!feedback_.geometryValid || feedback_.span>=4U || !std::isfinite(feedback_.progress) ||
        feedback_.progress<0.0 || feedback_.progress>feedback_.length ||
        !EDM114::Within(std::fabs(feedback_.v),feedback_.maximumVelocity) ||
        (feedback_.v!=0.0 && (feedback_.v>0.0?1:-1)!=feedback_.direction) ||
        !EDM114::StopFits(feedback_.progress,std::fabs(feedback_.v),feedback_.length,feedback_.deceleration)) return false;
    Point position{},velocity{},acceleration{};
    return CurrentMapping().Eval(feedback_.q,feedback_.v,feedback_.a,position,velocity,acceleration);
}
inline bool Excursion::Step(std::uint64_t tick,std::uint64_t nowUs,bool physicalSettled) noexcept
{
    if (!feedback_.active || feedback_.faulted || feedback_.stopRequested || !feedback_.geometryValid || feedback_.span>=4U) return false;
    if (!AdvanceClock(tick,nowUs)) return false;
    if (tick-feedback_.startTick>MaximumDurationTicks || nowUs-feedback_.startUs>MaximumDurationUs) return Fail(Reason::Deadline);
    if (feedback_.phase==Phase::InitialSettle || feedback_.phase==Phase::EndpointSettle)
    {
        if (tick-phaseTick_>MaximumDwellTicks || nowUs-phaseUs_>MaximumDwellUs) return Fail(Reason::Dwell);
        if (feedback_.v!=0.0 || !feedback_.endpointReady) return Fail(Reason::Lifecycle);
        feedback_.a=0.0;
        feedback_.settledCycles=physicalSettled?feedback_.settledCycles+1U:0U;
        if (feedback_.settledCycles<RequiredSettledCycles) return true;
        ++feedback_.completedDwells;
        if (feedback_.phase==Phase::InitialSettle) {StartSpan();return true;}
        if (feedback_.span==3U)
        {
            Point position{},velocity{};
            if (feedback_.completedDwells!=5U || !Map(position,velocity) || !EDM109::SamePulse(position,anchor_)) return Fail(Reason::Geometry);
            feedback_.completed=feedback_.anchorReturned=1ULL;feedback_.complete=true;feedback_.active=feedback_.moving=false;
            feedback_.phase=Phase::Complete;return true;
        }
        Point prior{},velocity{}; if (!Map(prior,velocity)) return Fail(Reason::Geometry);
        if (feedback_.span==0U) ++feedback_.backwardCrossings;
        if (feedback_.span==2U) ++feedback_.forwardCrossings;
        if (!SetSpan(feedback_.span+1U)) return false;
        Point after{}; if (!Map(after,velocity) || !EDM109::SamePulse(prior,after)) return Fail(Reason::Geometry);
        StartSpan(); return true;
    }
    if (feedback_.phase!=Phase::Moving) return Fail(Reason::Lifecycle);
    const auto& s=spans_[feedback_.span];const std::uint64_t elapsed=tick-phaseTick_;
    if (elapsed>=s.upTicks+s.cruiseTicks+s.downTicks)
    {
        if (!SetSample(s.length,0.0)) return false;
        feedback_.moving=false;feedback_.endpointReady=true;feedback_.settledCycles=0U;
        feedback_.phase=Phase::EndpointSettle;phaseTick_=tick;phaseUs_=nowUs;return true;
    }
    double progress=0.0,velocity=0.0;
    if (elapsed<=s.upTicks)
    {const double t=static_cast<double>(elapsed)*SampleSeconds;progress=0.5*s.acceleration*t*t;velocity=s.acceleration*t;}
    else if (elapsed<=s.upTicks+s.cruiseTicks)
    {progress=0.5*s.peak*static_cast<double>(s.upTicks)*SampleSeconds+s.peak*static_cast<double>(elapsed-s.upTicks)*SampleSeconds;velocity=s.peak;}
    else
    {const double remain=static_cast<double>(s.upTicks+s.cruiseTicks+s.downTicks-elapsed)*SampleSeconds;progress=s.length-0.5*s.deceleration*remain*remain;velocity=s.deceleration*remain;}
    return SetSample(progress,velocity);
}
inline bool Excursion::RequestStop(std::uint64_t firstStopTick,std::uint64_t firstStopUs) noexcept
{
    if (!feedback_.geometryValid || feedback_.faulted || feedback_.span>=4U || feedback_.complete) return false;
    if (feedback_.stopRequested) return true;
    // Only the first authorized stop may bridge time spent behind a pending
    // cancellation fence. Position, velocity, span and frozen D never rebase.
    // The caller supplies the first upcoming SAFETY output tick, not authority.
    if ((firstStopTick==0ULL)!=(firstStopUs==0ULL) || (firstStopTick!=0ULL &&
        (firstStopTick<feedback_.tick || firstStopUs<feedback_.nowUs || firstStopUs<250ULL))) return Fail(Reason::Clock);
    if (!ValidSample() || !stop_.Begin(feedback_.progress,std::fabs(feedback_.v),feedback_.length,feedback_.deceleration))
        return Fail(Reason::StopEnvelope);
    feedback_.stopRequested=true;feedback_.active=feedback_.complete=feedback_.endpointReady=false;feedback_.settledCycles=0U;
    feedback_.phase=stop_.Current().stopped?Phase::Stopped:Phase::Stopping;
    feedback_.stopped=stop_.Current().stopped;feedback_.moving=!feedback_.stopped;
    if (feedback_.stopped) feedback_.a=0.0;
    if (firstStopTick!=0ULL)
    {
        if (firstStopTick==feedback_.tick) feedback_.nowUs=firstStopUs;
        else {feedback_.tick=firstStopTick-1ULL;feedback_.nowUs=firstStopUs-250ULL;}
    }
    return true;
}
inline bool Excursion::StepStop(std::uint64_t tick,std::uint64_t nowUs) noexcept
{
    if (!feedback_.geometryValid || !feedback_.stopRequested || feedback_.faulted || feedback_.span>=4U) return false;
    // A same-cycle SAFETY re-entry is a read of the last committed scalar,
    // never a second integration. Only the subsequent fresh tick advances.
    if (tick==feedback_.tick && nowUs==feedback_.nowUs) return true;
    if (!AdvanceClock(tick,nowUs)) return false;
    if (feedback_.stopped) {feedback_.a=0.0;return true;}
    if (feedback_.phase!=Phase::Stopping || !stop_.Step()) return Fail(Reason::StopEnvelope);
    if (!SetSample(stop_.Current().q,stop_.Current().v)) return false;
    if (stop_.Current().stopped)
    {feedback_.stopped=true;feedback_.moving=false;feedback_.phase=Phase::Stopped;}
    return true;
}
inline bool Excursion::Map(Point& position,Point& velocity,Point* acceleration) const noexcept
{
    if (!feedback_.geometryValid || feedback_.faulted || !ValidSample()) return false;
    Point p{},v{},a{};
    if (!CurrentMapping().Eval(feedback_.q,feedback_.v,feedback_.a,p,v,a)) return false;
    if (&position==&velocity || (acceleration && (acceleration==&position || acceleration==&velocity))) return false;
    position=p;velocity=v;if(acceleration)*acceleration=a;return true;
}
static_assert(std::is_trivially_copyable<Feedback>::value && sizeof(Feedback)<=512U,"EDM115 fixed feedback");
static_assert(std::is_trivially_copyable<Excursion>::value && sizeof(Excursion)<=2048U,"EDM115 fixed mixed-curve workspace");
}
