#pragma once

#include "EDMGapServo.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error EDM81_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error EDM81_requires_nonfinite_checks
#endif

// EDM81: single-owner NUMERICAL SIMULATION only. Never pass these poses to a
// drive. There is no motion API, allocation, clock read, output permit or I/O.
// A retained, piecewise-linear CNC path has ONE scalar cursor in XYZ mm. C/U/V
// are explicitly UNWRAPPED degrees interpolated by that same cursor. Degrees
// never enter the XYZ length calculation. Pure rotary paths and TCP transforms
// are rejected until a separately verified kinematic/path-metric adapter exists.
// A future adapter must sit under the existing CNC/RT motion ownership protocol;
// this value type is not an additional axis-command producer.
namespace EDM81
{
constexpr std::size_t AxisCount = 6U;
constexpr unsigned MaxSegments = 8U;
using Pose = std::array<double, AxisCount>;
enum class Mode : std::uint8_t { RetainedPath, DirectionOffset };
enum class Frame : std::uint8_t { MachineCartesian, Workpiece, ToolTCP };
enum class Kinematics : std::uint8_t { DirectJoint, TCP, EccentricTiltedTool };
enum class Event : std::uint8_t { None, Jump, Hold, Reset, Alarm, M30 };
enum class State : std::uint8_t
{ Idle, Running, JumpRetreat, ShortRetreat, Returning, Cancelled, Completed, Fault };
enum class Reason : std::uint8_t
{
    None, Lifecycle, RetiredRun, Scope, Configuration, UnsupportedMapping,
    Path, UnsupportedPathMetric, NonFinite, Sample, Sequence, Cadence, Stale, Authority, AxisLimit,
    TravelLimit, VelocityLimit, CounterOverflow, PrematureEnd, Hold, Reset, Alarm
};
struct Scope
{
    std::uint64_t run=0, cache=0, dispatch=0, executionEpoch=0, ownerLease=0;
    std::uint64_t configRevision=0, recipeGeneration=0, frameRevision=0;
    std::uint64_t kinematicsRevision=0, sourceGeneration=0;
};
inline bool ValidScope(const Scope& s) noexcept
{
    return s.run && s.cache && s.dispatch && s.executionEpoch && s.ownerLease &&
        s.configRevision && s.recipeGeneration && s.frameRevision &&
        s.kinematicsRevision && s.sourceGeneration;
}
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.run==b.run && a.cache==b.cache && a.dispatch==b.dispatch &&
        a.executionEpoch==b.executionEpoch && a.ownerLease==b.ownerLease &&
        a.configRevision==b.configRevision && a.recipeGeneration==b.recipeGeneration &&
        a.frameRevision==b.frameRevision && a.kinematicsRevision==b.kinematicsRevision &&
        a.sourceGeneration==b.sourceGeneration;
}
struct AxisBounds
{
    // XYZ: mm and mm/min; CUV: degrees and degrees/min. Each bound is required.
    double minimum=-1.0, maximum=1.0, maxTravel=1.0, maxVelocityPerMin=10.0;
};
struct Config
{
    EDMGapServo::Profile servo{};
    double referenceV=75.0, overridePercent=75.0, maxScalarMmMin=5.0;
    double maxRetreatMm=0.03, jumpMm=0.01, excursionSpeedMmMin=3.0;
    AxisBounds axes[AxisCount]{};
    std::uint64_t periodUs=250, maxAgeUs=1000;
    Mode mode=Mode::RetainedPath;
    Frame frame=Frame::MachineCartesian;
    Kinematics kinematics=Kinematics::DirectJoint;
    Pose directionOrigin{};
    std::array<double,3> machiningUnit{{0.0,0.0,1.0}};
    double directionExtentMm=0.1;
};
struct Plan
{
    Pose point[MaxSegments+1U]{};
    unsigned count=0; // number of segments, not points
};
struct Input
{
    Scope scope{};
    std::uint64_t sequence=0, capturedAtUs=0, nowUs=0;
    double voltageV=0.0;
    bool valid=true, authorityCurrent=true;
    std::uint32_t limitMask=0;
    Event event=Event::None;
};
struct Snapshot
{
    State state=State::Idle;
    Reason reason=Reason::None;
    Scope scope{};
    Pose pose{};
    double cursorMm=0.0, anchorMm=0.0, pathLengthMm=0.0;
    std::uint64_t steps=0;
    bool committed=false;
    constexpr bool PhysicalPermit() const noexcept { return false; }
    constexpr bool DischargeEnabled() const noexcept { return false; }
};
inline bool Terminal(State s) noexcept
{ return s==State::Cancelled || s==State::Completed || s==State::Fault; }
inline double Minimum(double a,double b) noexcept { return a<b?a:b; }
inline double Maximum(double a,double b) noexcept { return a>b?a:b; }
inline double Clamp(double x,double low,double high) noexcept
{ return x<low?low:(x>high?high:x); }
inline bool Finite(double x) noexcept { return std::isfinite(x); }

class Kernel
{
public:
    const Snapshot& Current() const noexcept { return current_; }

    bool Begin(const Config& config,const Scope& scope,const Plan& plan) noexcept
    {
        if(current_.state!=State::Idle && !Terminal(current_.state))
            return FailBool(Reason::Lifecycle);
        if(!ValidScope(scope)) return FailBool(Reason::Scope);
        if(scope.run<=retiredRun_) return FailBool(Reason::RetiredRun);
        // A failed admission also retires its run. A terminal run cannot resume.
        retiredRun_=scope.run;
        current_=Snapshot{}; current_.scope=scope;
        if(config.frame!=Frame::MachineCartesian || config.kinematics!=Kinematics::DirectJoint)
            return FailBool(Reason::UnsupportedMapping);
        if(!ValidConfig(config)) return FailBool(Reason::Configuration);
        config_=config;
        path_=Plan{};
        if(config.mode==Mode::DirectionOffset)
        {
            double norm=0.0;
            for(unsigned i=0;i<3U;++i)
            {
                const double u=config.machiningUnit[i];
                if(!Finite(u) || std::fabs(u)>1.0) return FailBool(Reason::Configuration);
                norm+=u*u;
            }
            if(!Finite(norm) || std::fabs(norm-1.0)>1e-9 ||
                !Finite(config.directionExtentMm) || config.directionExtentMm<=0.0)
                return FailBool(Reason::Configuration);
            path_.count=1U; path_.point[0]=path_.point[1]=config.directionOrigin;
            for(unsigned i=0;i<3U;++i)
                path_.point[1][i]+=config.machiningUnit[i]*config.directionExtentMm;
        }
        else path_=plan;
        if(path_.count==0U || path_.count>MaxSegments) return FailBool(Reason::Path);
        origin_=path_.point[0];
        station_[0]=0.0;
        for(unsigned p=0;p<=path_.count;++p)
        {
            if(!ValidPosition(path_.point[p],false)) return false;
            if(!p) continue;
            double lengthSquared=0.0;
            for(unsigned axis=0;axis<3U;++axis)
            {
                const double delta=path_.point[p][axis]-path_.point[p-1U][axis];
                lengthSquared+=delta*delta;
            }
            const double length=std::sqrt(lengthSquared);
            if(!Finite(length)) return FailBool(Reason::Path);
            if(length<=1e-12) return FailBool(Reason::UnsupportedPathMetric);
            station_[p]=station_[p-1U]+length;
            if(!Finite(station_[p]) || station_[p]<=station_[p-1U])
                return FailBool(Reason::Path);
        }
        current_.pose=origin_; current_.pathLengthMm=station_[path_.count];
        current_.state=State::Running;
        current_.reason=Reason::None;
        seeded_=false; lastSequence_=lastSourceUs_=lastNowUs_=0;
        highWaterMm_=retreatTargetMm_=0.0;
        shortDetector_.Reset();
        return true;
    }

    const Snapshot& Step(const Input& input) noexcept
    {
        current_.committed=false;
        if(Terminal(current_.state)) return current_;
        if(current_.state==State::Idle) return Fail(Reason::Lifecycle);
        // Operator/safety cancellation is independent of a live GAP source.
        if(input.event==Event::Hold || input.event==Event::Reset || input.event==Event::Alarm)
            return Cancel(input.event);
        if(input.event!=Event::None && input.event!=Event::Jump && input.event!=Event::M30)
            return Fail(Reason::Lifecycle);
        if(!SameScope(input.scope,current_.scope)) return Fail(Reason::Scope);
        if(!input.authorityCurrent) return Fail(Reason::Authority);
        if(input.limitMask) return Fail(Reason::AxisLimit);
        if(!input.valid || !Finite(input.voltageV) || input.voltageV<0.0)
            return Fail(Reason::Sample);
        if(!input.sequence || !input.capturedAtUs || !input.nowUs || input.capturedAtUs>input.nowUs)
            return Fail(Reason::Sample);
        if(input.nowUs-input.capturedAtUs>config_.maxAgeUs) return Fail(Reason::Stale);
        const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
        if(current_.steps==maximum) return Fail(Reason::CounterOverflow);
        if(seeded_)
        {
            if(lastSequence_==maximum || input.sequence!=lastSequence_+1U)
                return Fail(Reason::Sequence);
            if(input.capturedAtUs<=lastSourceUs_ || input.nowUs<=lastNowUs_ ||
                input.capturedAtUs-lastSourceUs_!=config_.periodUs ||
                input.nowUs-lastNowUs_!=config_.periodUs)
                return Fail(Reason::Cadence);
        }
        const auto curve=EDMGapServo::Evaluate(config_.servo,input.voltageV,
            config_.referenceV,config_.overridePercent);
        if(!curve.valid) return Fail(Reason::NonFinite);
        EDMGapServo::Sample sample{}; sample.valid=true; sample.voltage=input.voltageV;
        sample.sequence=input.sequence; sample.nowMs=input.capturedAtUs/1000U;
        const auto shortResult=shortDetector_.Step(config_.servo,sample,true);
        if(!shortResult.valid) return Fail(Reason::Sample);
        if(input.event==Event::M30)
        {
            if(!seeded_ || current_.state!=State::Running || shortResult.feedInhibited ||
                current_.cursorMm!=current_.pathLengthMm)
                return Fail(Reason::PrematureEnd);
            current_.state=State::Completed; current_.reason=Reason::None;
            return current_;
        }
        if(!seeded_)
        {
            // Baseline sample publishes no inferred travel interval.
            if(input.event!=Event::None) return Fail(Reason::Lifecycle);
            AcceptSample(input); seeded_=true;
            return current_;
        }
        State nextState=current_.state;
        double nextAnchor=current_.anchorMm;
        double nextTarget=retreatTargetMm_;
        if(shortResult.shortActive)
        {
            if(nextState==State::Running)
            {
                nextAnchor=current_.cursorMm;
                nextTarget=Maximum(0.0,nextAnchor-config_.maxRetreatMm);
            }
            else if(nextState!=State::ShortRetreat)
                nextTarget=Maximum(0.0,nextAnchor-config_.maxRetreatMm);
            nextState=State::ShortRetreat;
        }
        else if(nextState==State::ShortRetreat && !shortResult.feedInhibited)
            nextState=State::Returning;
        if(input.event==Event::Jump)
        {
            // Overlapping excursions cannot replace the frozen resume anchor.
            if(nextState!=State::Running || shortResult.feedInhibited)
                return Fail(Reason::Lifecycle);
            nextAnchor=current_.cursorMm;
            nextTarget=Maximum(0.0,nextAnchor-config_.jumpMm);
            nextState=State::JumpRetreat;
        }
        const double dtMinutes=static_cast<double>(config_.periodUs)/60000000.0;
        double proposed=current_.cursorMm;
        if(nextState==State::Running && !shortResult.feedInhibited)
        {
            const double speed=Clamp(curve.limitedSpeedMmPerMin,
                -config_.maxScalarMmMin,config_.maxScalarMmMin);
            const double floor=Maximum(0.0,highWaterMm_-config_.maxRetreatMm);
            proposed=Clamp(proposed+speed*dtMinutes,floor,current_.pathLengthMm);
        }
        else if(nextState==State::JumpRetreat || nextState==State::ShortRetreat)
        {
            double speed=config_.excursionSpeedMmMin;
            if(nextState==State::ShortRetreat)
            {
                const auto retreat=EDMGapServo::EvaluateShortRetreat(config_.servo,curve);
                if(!retreat.valid) return Fail(Reason::NonFinite);
                // Fixed short demand uses E8 once, matching EDM55. A curve-
                // sourced demand already contains E8 from Evaluate above.
                const double factor=retreat.usingCurve?1.0:config_.overridePercent/100.0;
                speed=Minimum(speed,-retreat.signedSpeedMmPerMin*factor);
            }
            proposed=Maximum(nextTarget,proposed-speed*dtMinutes);
            if(proposed==nextTarget && nextState==State::JumpRetreat)
                nextState=State::Returning;
        }
        else if(nextState==State::Returning && !shortResult.feedInhibited)
        {
            proposed=Minimum(nextAnchor,proposed+config_.excursionSpeedMmMin*dtMinutes);
            // The anchor is restored in this cycle. Only a later cycle can cut.
            if(proposed==nextAnchor) nextState=State::Running;
        }
        if(!Finite(proposed)) return Fail(Reason::NonFinite);
        Pose candidate{};
        if(!Evaluate(proposed,candidate) || !ValidTraversal(current_.cursorMm,proposed) ||
            !ValidPosition(candidate,true)) return current_;
        // All six coordinates and the common cursor commit as ONE owner value.
        // Snapshot is not a thread-safe publication; future RT transport must
        // publish a coherent copy through the existing ownership protocol.
        current_.pose=candidate; current_.cursorMm=proposed;
        current_.anchorMm=nextAnchor; current_.state=nextState;
        retreatTargetMm_=nextTarget;
        if(nextState==State::Running) highWaterMm_=Maximum(highWaterMm_,proposed);
        AcceptSample(input);
        return current_;
    }

    const Snapshot& Cancel(Event event) noexcept
    {
        current_.committed=false;
        if(Terminal(current_.state)) return current_;
        if(event!=Event::Hold && event!=Event::Reset && event!=Event::Alarm)
            return Fail(Reason::Lifecycle);
        // HOLD is deliberately terminal at this simulation stage. There is no
        // automatic rearm/resume or claimed physical deceleration/release proof.
        current_.state=State::Cancelled;
        current_.reason=event==Event::Hold?Reason::Hold:(event==Event::Reset?Reason::Reset:Reason::Alarm);
        return current_;
    }
private:
    bool ValidConfig(const Config& c) const noexcept
    {
        if(c.mode!=Mode::RetainedPath && c.mode!=Mode::DirectionOffset) return false;
        if(!EDMGapServo::ValidateProfile(c.servo) || !Finite(c.referenceV) || c.referenceV<0.0 ||
            !Finite(c.overridePercent) || c.overridePercent<0.0 || c.overridePercent>100.0 ||
            !Finite(c.maxScalarMmMin) || c.maxScalarMmMin<=0.0 ||
            !Finite(c.maxRetreatMm) || c.maxRetreatMm<=0.0 ||
            !Finite(c.jumpMm) || c.jumpMm<=0.0 || c.jumpMm>c.maxRetreatMm ||
            !Finite(c.excursionSpeedMmMin) || c.excursionSpeedMmMin<=0.0 ||
            c.excursionSpeedMmMin>c.maxScalarMmMin || c.periodUs!=250U ||
            c.maxAgeUs<c.periodUs || c.maxAgeUs>5000U) return false;
        for(const auto& b:c.axes)
            if(!Finite(b.minimum) || !Finite(b.maximum) || b.minimum>=b.maximum ||
                !Finite(b.maxTravel) || b.maxTravel<=0.0 ||
                !Finite(b.maxVelocityPerMin) || b.maxVelocityPerMin<=0.0) return false;
        return true;
    }
    bool Evaluate(double cursor,Pose& out) noexcept
    {
        if(!Finite(cursor) || cursor<0.0 || cursor>current_.pathLengthMm)
            return FailBool(Reason::Path);
        if(cursor==current_.pathLengthMm) { out=path_.point[path_.count]; return true; }
        for(unsigned seg=0;seg<path_.count;++seg)
        {
            if(cursor>station_[seg+1U]) continue;
            const double fraction=(cursor-station_[seg])/(station_[seg+1U]-station_[seg]);
            for(unsigned axis=0;axis<AxisCount;++axis)
                out[axis]=path_.point[seg][axis]+fraction*(path_.point[seg+1U][axis]-path_.point[seg][axis]);
            return true;
        }
        return FailBool(Reason::Path);
    }
    bool ValidTraversal(double from,double to) noexcept
    {
        if(from==to) return true;
        const double low=Minimum(from,to),high=Maximum(from,to);
        const double scalarRate=(high-low)*(60000000.0/static_cast<double>(config_.periodUs));
        if(!Finite(scalarRate)) return FailBool(Reason::NonFinite);
        // Candidate endpoint averaging must not hide a rapid short segment or
        // opposite slopes inside one cycle. Bound every crossed segment's
        // native-unit derivative; this remains at most 8 * 6 comparisons.
        // Acceleration/jerk and corner smoothing are not asserted by this model.
        for(unsigned seg=0;seg<path_.count;++seg)
        {
            if(high<=station_[seg] || low>=station_[seg+1U]) continue;
            const double length=station_[seg+1U]-station_[seg];
            for(unsigned axis=0;axis<AxisCount;++axis)
            {
                const double rate=std::fabs(path_.point[seg+1U][axis]-path_.point[seg][axis])/length*scalarRate;
                if(!Finite(rate) || rate>config_.axes[axis].maxVelocityPerMin*(1.0+1e-12))
                    return FailBool(Reason::VelocityLimit);
            }
        }
        return true;
    }
    bool ValidPosition(const Pose& candidate,bool checkVelocity) noexcept
    {
        const double scale=60000000.0/static_cast<double>(config_.periodUs);
        for(unsigned axis=0;axis<AxisCount;++axis)
        {
            const double value=candidate[axis];
            const auto& b=config_.axes[axis];
            if(!Finite(value)) return FailBool(Reason::NonFinite);
            if(value<b.minimum || value>b.maximum) return FailBool(Reason::AxisLimit);
            const double travel=std::fabs(value-origin_[axis]);
            if(!Finite(travel) || travel>b.maxTravel) return FailBool(Reason::TravelLimit);
            if(checkVelocity)
            {
                const double speed=std::fabs(value-current_.pose[axis])*scale;
                if(!Finite(speed) || speed>b.maxVelocityPerMin*(1.0+1e-12))
                    return FailBool(Reason::VelocityLimit);
            }
        }
        return true;
    }
    void AcceptSample(const Input& input) noexcept
    {
        lastSequence_=input.sequence; lastSourceUs_=input.capturedAtUs; lastNowUs_=input.nowUs;
        ++current_.steps; current_.committed=true;
    }
    const Snapshot& Fail(Reason reason) noexcept
    {
        current_.state=State::Fault; current_.reason=reason; current_.committed=false;
        return current_;
    }
    bool FailBool(Reason reason) noexcept { Fail(reason); return false; }
    Config config_{};
    Plan path_{};
    Pose origin_{};
    double station_[MaxSegments+1U]{};
    double highWaterMm_=0.0,retreatTargetMm_=0.0;
    Snapshot current_{};
    EDMGapServo::ShortDetector shortDetector_{};
    std::uint64_t retiredRun_=0,lastSequence_=0,lastSourceUs_=0,lastNowUs_=0;
    bool seeded_=false;
};
static_assert(sizeof(Kernel)<4096U,"EDM81 fixed-size owner state unexpectedly grew");
}
