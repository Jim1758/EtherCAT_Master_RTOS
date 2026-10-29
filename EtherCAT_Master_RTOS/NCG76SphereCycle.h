#pragma once
#include "NCG76SphereRuntimeGeometry.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

// EDM286: one bounded analytic P50 parent. Pure commanded trajectory only;
// ownership, frame identity, PDO provenance, power inhibition and final-send
// evidence are independently supplied/proved by the production adapter.
namespace NCG76SphereCycle
{
using Point8 = std::array<double,8U>;
using Point3 = std::array<double,3U>;
using Geometry = NCG76SphereRuntimeGeometry::Geometry;

enum class Phase : std::uint8_t
{
    Idle, InitialDepth, InitialRetreat, CircleEntry, Orbit, ReturnCenter,
    AdvanceCenter, Settle, HoldBrake, HoldSettle, HoldToCenter, HoldToStart,
    HeldAtStart, ResumeToCenter, ResumeToAnchor, FaultBrake, FaultSettle,
    Complete, Fault
};
enum class Fault : std::uint8_t
{
    None, InvalidRequest, SourceLost, PdoLost, PowerNotInhibited, Clock,
    PhysicalSequence, IntentSequence, Numeric, TravelLimit, SettleTimeout,
    CycleTimeout, RepeatedHold, Cancelled
};

struct Request
{
    Geometry geometry{};
    std::uint64_t session = 0ULL;
    double feedMmMin = 0.0, returnFeedMmMin = 0.0;
    double velocityCapMmS = 0.0, accelerationMmS2 = 0.0, decelerationMmS2 = 0.0;
    Point8 travelMin{}, travelMax{};
    std::uint32_t travelMask = 0U;
    double positionToleranceMm = 0.001, rotaryToleranceDeg = 0.001;
    unsigned settleSamples = 3U;
    double minimumSettleSeconds = 0.05, settleTimeoutSeconds = 5.0, maximumCycleSeconds = 86400.0;
    std::uint64_t minimumTickUs = 50ULL, maximumTickUs = 1000ULL;
};

struct Input
{
    std::uint64_t tick = 0ULL, nowUs = 0ULL, physicalSequence = 0ULL;
    std::uint64_t holdSequence = 0ULL, startSequence = 0ULL;
    Point8 actualMCS{};
    bool sourceValid = false, pdoValid = false, powerInhibited = false;
    bool hold = false, cancel = false;
    bool physicalSettled = false, zeroFinalSend = false;
    std::uint64_t zeroSendTick = 0ULL;
};

struct Output
{
    Point8 commandMCS{}, velocityNative{}, accelerationNative{};
    bool active = false, complete = false, stopped = true;
    bool faulted = false, emergencyStopRequired = false, cancelled = false;
    bool zeroDemand = true;
};

struct Feedback
{
    Phase phase = Phase::Idle, nominalPhase = Phase::InitialDepth;
    Fault fault = Fault::None;
    std::uint64_t session = 0ULL, tick = 0ULL, holdSequence = 0ULL, resumeSequence = 0ULL;
    unsigned layerIndex = 0U, completedLayers = 0U;
    double nominalProgressMm = 0.0, phaseProgressMm = 0.0, phaseLengthMm = 0.0, speedMmS = 0.0;
    bool held = false, holdActive = false;
};

// Validation is an admission operation, not part of the 250-us tick.
inline bool ValidateRequest(const Request& request) noexcept;

class Cycle
{
public:
    bool Begin(const Request& request,const Input& input) noexcept;
    Output Step(const Input& input) noexcept;
    const Feedback& Current() const noexcept { return feedback_; }
    const Output& Command() const noexcept { return output_; }
    bool ZeroDemand() const noexcept { return output_.zeroDemand; }
    void Reset() noexcept { *this = Cycle{}; }

private:
    enum class SettleAction : std::uint8_t
    { BeginNominal, NextNominal, HoldCenter, HoldStart, Held, ResumeCenter, ResumeAnchor, RestoreNominal, Complete, Fault };
    struct Leg
    {
        Point8 start{},end{},center{};
        Point3 direction{};
        double length = 0.0, position = 0.0, speed = 0.0;
        double speedLimit = 0.0, acceleration = 0.0, deceleration = 0.0;
        double basePosition = 0.0, elapsed = 0.0, peak = 0.0;
        double accelerationTime = 0.0, cruiseTime = 0.0, decelerationTime = 0.0;
        double accelerationDistance = 0.0;
        double accelerationNow = 0.0;
        double stopPosition = 0.0, stopTime = 0.0, stopElapsed = 0.0;
        bool circle = false;
    };
    bool InsideTravel(const Point8& point) const noexcept;
    bool AtCommand(const Point8& actual) const noexcept;
    bool StartProfile(Leg& leg) noexcept;
    bool StartLine(const Point8& end,Phase phase,double feed) noexcept;
    bool StartNominal(Phase phase) noexcept;
    bool EvaluateLeg() noexcept;
    bool AdvanceLeg(double dt,bool brake,bool& finished) noexcept;
    bool StartBrake(Phase phase) noexcept;
    void StartSettle(Phase phase,SettleAction action,const Input& input) noexcept;
    bool Settled(const Input& input,double dt) noexcept;
    void ContinueAfterSettle(const Input& input) noexcept;
    void NextNominal(const Input& input) noexcept;
    void BeginHold(const Input& input) noexcept;
    void SaveHoldAnchor(const Input& input) noexcept;
    void Fail(Fault fault,bool emergency=true) noexcept;
    void RefreshOutput() noexcept;

    Request request_{};
    Output output_{};
    Feedback feedback_{};
    Leg leg_{}, savedLeg_{};
    Point8 savedAnchor_{}, savedCenter_{};
    Phase savedPhase_ = Phase::InitialDepth;
    SettleAction settleAction_ = SettleAction::BeginNominal;
    std::uint64_t lastTick_ = 0ULL,lastNowUs_ = 0ULL,lastPhysical_ = 0ULL;
    std::uint64_t lastHold_ = 0ULL,lastStart_ = 0ULL,heldStartFloor_ = 0ULL,settleTick_ = 0ULL,settlePhysical_ = 0ULL;
    unsigned settledSamples_ = 0U;
    double elapsed_ = 0.0,settleElapsed_ = 0.0,settleProvedSeconds_ = 0.0;
    bool initialized_ = false,lastHoldLevel_ = false;
};
} // namespace NCG76SphereCycle

namespace NCG76SphereCycle
{
namespace Detail
{
inline bool Positive(double value) noexcept { return std::isfinite(value) && value>0.0; }
template<std::size_t N> inline bool Finite(const std::array<double,N>& point) noexcept
{ for(double value:point)if(!std::isfinite(value))return false;return true; }
inline double Norm(const Point3& value) noexcept
{ return std::hypot(std::hypot(value[0],value[1]),value[2]); }
inline bool IsNominal(Phase phase) noexcept
{ return phase>=Phase::InitialDepth && phase<=Phase::AdvanceCenter; }
inline bool IsExcursion(Phase phase) noexcept
{ return phase==Phase::HoldToCenter || phase==Phase::HoldToStart || phase==Phase::ResumeToCenter || phase==Phase::ResumeToAnchor; }
inline bool IsSettling(Phase phase) noexcept
{ return phase==Phase::Settle || phase==Phase::HoldSettle || phase==Phase::FaultSettle; }
}

inline bool ValidateRequest(const Request& r) noexcept
{
    if(!r.session || !NCG76SphereRuntimeGeometry::Valid(r.geometry) ||
       !Detail::Positive(r.feedMmMin) || r.feedMmMin!=r.geometry.feedMmMin ||
       !Detail::Positive(r.returnFeedMmMin) || r.returnFeedMmMin>r.feedMmMin ||
       !Detail::Positive(r.velocityCapMmS) || !Detail::Positive(r.accelerationMmS2) ||
       !Detail::Positive(r.decelerationMmS2) || !std::isfinite(r.accelerationMmS2+r.decelerationMmS2) ||
       !std::isfinite(1.0/r.accelerationMmS2+1.0/r.decelerationMmS2) ||
       !Detail::Positive(r.positionToleranceMm) || !Detail::Positive(r.rotaryToleranceDeg) ||
       !r.settleSamples || r.settleSamples>1000000U ||
       !std::isfinite(r.minimumSettleSeconds) || r.minimumSettleSeconds<0.0 ||
       !Detail::Positive(r.settleTimeoutSeconds) || !Detail::Positive(r.maximumCycleSeconds) ||
       r.minimumSettleSeconds>r.settleTimeoutSeconds ||
       r.maximumCycleSeconds<r.settleTimeoutSeconds || !r.minimumTickUs ||
       r.minimumTickUs>r.maximumTickUs || r.maximumTickUs>1000000ULL ||
       r.settleTimeoutSeconds<=static_cast<double>(r.maximumTickUs)*1e-6 ||
       (r.travelMask&~0xffU)!=0U || !Detail::Finite(r.travelMin) || !Detail::Finite(r.travelMax)) return false;
    for(unsigned a=0U;a<8U;++a)
        if((r.travelMask&(1U<<a)) &&
           (r.travelMin[a]>=r.travelMax[a] || r.geometry.boundsMinMCS[a]<r.travelMin[a] ||
            r.geometry.boundsMaxMCS[a]>r.travelMax[a])) return false;
    return true;
}

inline bool Cycle::InsideTravel(const Point8& point) const noexcept
{
    if(!Detail::Finite(point))return false;
    for(unsigned a=0U;a<8U;++a)
        if((request_.travelMask&(1U<<a)) && (point[a]<request_.travelMin[a] || point[a]>request_.travelMax[a]))return false;
    return true;
}
inline bool Cycle::AtCommand(const Point8& actual) const noexcept
{
    if(!Detail::Finite(actual))return false;
    for(unsigned a=0U;a<8U;++a)
        if(std::fabs(actual[a]-output_.commandMCS[a])>(a<3U?request_.positionToleranceMm:request_.rotaryToleranceDeg))return false;
    return true;
}
inline void Cycle::Fail(Fault fault,bool emergency) noexcept
{
    feedback_.fault=fault;feedback_.phase=Phase::Fault;feedback_.held=false;
    leg_.speed=0.0;leg_.accelerationNow=0.0;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    output_.emergencyStopRequired=emergency;output_.cancelled=fault==Fault::Cancelled;
    RefreshOutput();
}
inline void Cycle::RefreshOutput() noexcept
{
    const auto phase=feedback_.phase;
    output_.active=initialized_ && phase!=Phase::Idle && phase!=Phase::Complete && phase!=Phase::Fault;
    output_.complete=phase==Phase::Complete;output_.faulted=phase==Phase::Fault;
    output_.stopped=leg_.speed==0.0;
    output_.zeroDemand=Detail::IsSettling(phase) || phase==Phase::HeldAtStart ||
        phase==Phase::Complete || phase==Phase::Fault || phase==Phase::Idle;
    feedback_.held=phase==Phase::HeldAtStart;
    feedback_.phaseProgressMm=leg_.position;feedback_.phaseLengthMm=leg_.length;feedback_.speedMmS=leg_.speed;
    if(Detail::IsNominal(phase) || phase==Phase::HoldBrake || (phase==Phase::Settle && !feedback_.holdActive))
        feedback_.nominalProgressMm=leg_.position;
    else if(feedback_.holdActive)feedback_.nominalProgressMm=savedLeg_.position;
}
inline bool Cycle::StartProfile(Leg& leg) noexcept
{
    leg.basePosition=leg.position;leg.elapsed=0.0;leg.speed=0.0;leg.accelerationNow=0.0;
    const double remaining=leg.length-leg.position;
    if(!std::isfinite(remaining) || remaining<0.0)return false;
    if(remaining==0.0)
    {leg.peak=leg.accelerationTime=leg.cruiseTime=leg.decelerationTime=leg.accelerationDistance=0.0;return true;}
    const double reciprocal=1.0/leg.acceleration+1.0/leg.deceleration;
    const double triangle=std::sqrt((2.0*remaining)/reciprocal);
    leg.peak=(std::min)(leg.speedLimit,triangle);
    if(!Detail::Positive(leg.peak))return false;
    leg.accelerationTime=leg.peak/leg.acceleration;
    leg.decelerationTime=leg.peak/leg.deceleration;
    leg.accelerationDistance=0.5*leg.accelerationTime*leg.peak;
    const double decelerationDistance=0.5*leg.decelerationTime*leg.peak;
    const double cruiseDistance=(std::max)(0.0,remaining-leg.accelerationDistance-decelerationDistance);
    leg.cruiseTime=cruiseDistance/leg.peak;
    const double total=leg.accelerationTime+leg.cruiseTime+leg.decelerationTime;
    return Detail::Positive(leg.accelerationTime) && Detail::Positive(leg.decelerationTime) &&
        std::isfinite(leg.accelerationDistance) && std::isfinite(decelerationDistance) &&
        std::isfinite(leg.cruiseTime) && leg.cruiseTime>=0.0 && Detail::Positive(total) && total<=request_.maximumCycleSeconds;
}
inline bool Cycle::StartLine(const Point8& end,Phase phase,double feed) noexcept
{
    if(!InsideTravel(end)) {Fail(Fault::TravelLimit);return false;}
    Leg next{};next.start=output_.commandMCS;next.end=end;
    for(unsigned a=0U;a<3U;++a)next.direction[a]=end[a]-next.start[a];
    for(unsigned a=3U;a<8U;++a)
        if(end[a]!=request_.geometry.startMCS[a] || next.start[a]!=request_.geometry.startMCS[a])
        {Fail(Fault::Numeric);return false;}
    next.length=Detail::Norm(next.direction);
    if(!std::isfinite(next.length)) {Fail(Fault::Numeric);return false;}
    if(next.length>0.0)for(double& value:next.direction)value/=next.length;
    next.speedLimit=(std::min)(feed/60.0,request_.velocityCapMmS);
    next.acceleration=request_.accelerationMmS2;next.deceleration=request_.decelerationMmS2;
    if(!Detail::Positive(next.speedLimit) || !StartProfile(next)) {Fail(Fault::Numeric);return false;}
    leg_=next;feedback_.phase=phase;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    RefreshOutput();return true;
}
inline bool Cycle::StartNominal(Phase phase) noexcept
{
    NCG76SphereRuntimeGeometry::Layer layer{};
    if(!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,feedback_.layerIndex,layer))
    {Fail(Fault::Numeric);return false;}
    feedback_.nominalPhase=phase;
    if(phase==Phase::InitialDepth)return StartLine(request_.geometry.endMCS,phase,request_.feedMmMin);
    if(phase==Phase::InitialRetreat)return StartLine(layer.center,phase,request_.feedMmMin);
    if(phase==Phase::CircleEntry)
    {
        Point8 point{};Point3 tangent{},curvature{};
        if(!NCG76SphereRuntimeGeometry::CircleAt(request_.geometry,feedback_.layerIndex,0.0,point,tangent,curvature))
        {Fail(Fault::Numeric);return false;}
        if(!StartLine(point,phase,request_.feedMmMin))return false;
        leg_.center=layer.center;return true;
    }
    if(phase==Phase::Orbit)
    {
        Leg next{};next.circle=true;next.start=next.end=output_.commandMCS;next.center=layer.center;
        next.length=layer.arcLengthMm;
        // Around a circle an individual axis may decelerate while the scalar
        // path speed accelerates. Use the smaller physical cap in both scalar
        // directions; reserve half for tangent and half for normal v^2/r.
        const double normalBudget=0.5*(std::min)(request_.accelerationMmS2,request_.decelerationMmS2);
        next.acceleration=next.deceleration=normalBudget;
        const double radialSpeed=std::sqrt(normalBudget)*std::sqrt(layer.radiusMm);
        next.speedLimit=(std::min)((std::min)(request_.feedMmMin/60.0,request_.velocityCapMmS),radialSpeed);
        if(!Detail::Positive(next.length) || !Detail::Positive(radialSpeed) || !Detail::Positive(next.speedLimit) ||
           !StartProfile(next)) {Fail(Fault::Numeric);return false;}
        leg_=next;feedback_.phase=phase;
        output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();return true;
    }
    if(phase==Phase::ReturnCenter)
    {
        if(!StartLine(layer.center,phase,request_.feedMmMin))return false;
        leg_.center=layer.center;return true;
    }
    if(phase==Phase::AdvanceCenter)
    {
        NCG76SphereRuntimeGeometry::Layer next{};
        if(!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,feedback_.layerIndex+1U,next))
        {Fail(Fault::Numeric);return false;}
        return StartLine(next.center,phase,request_.feedMmMin);
    }
    Fail(Fault::Numeric);return false;
}
inline bool Cycle::EvaluateLeg() noexcept
{
    Point3 tangent{},curvature{};
    if(leg_.circle)
    {
        const double angle=leg_.position==leg_.length ? 2.0*NCG76Sphere::Pi :
            (leg_.position/leg_.length)*(2.0*NCG76Sphere::Pi);
        if(!NCG76SphereRuntimeGeometry::CircleAt(request_.geometry,feedback_.layerIndex,angle,
            output_.commandMCS,tangent,curvature))return false;
        if(leg_.position==leg_.length)output_.commandMCS=leg_.end;
    }
    else
    {
        tangent=leg_.direction;
        if(leg_.position==leg_.length)output_.commandMCS=leg_.end;
        else if(leg_.position==0.0)output_.commandMCS=leg_.start;
        else
        {
            output_.commandMCS=leg_.start;
            for(unsigned a=0U;a<3U;++a)output_.commandMCS[a]=leg_.start[a]+leg_.position*leg_.direction[a];
        }
    }
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);
    for(unsigned a=0U;a<3U;++a)
    {
        output_.velocityNative[a]=tangent[a]*leg_.speed;
        output_.accelerationNative[a]=tangent[a]*leg_.accelerationNow+curvature[a]*leg_.speed*leg_.speed;
    }
    return InsideTravel(output_.commandMCS) && Detail::Finite(output_.velocityNative) && Detail::Finite(output_.accelerationNative);
}
inline bool Cycle::AdvanceLeg(double dt,bool brake,bool& finished) noexcept
{
    finished=false;
    const double previous=leg_.position;
    if(brake)
    {
        leg_.stopElapsed+=dt;
        if(leg_.stopElapsed>=leg_.stopTime)
        {leg_.position=leg_.stopPosition;leg_.speed=0.0;leg_.accelerationNow=0.0;finished=true;}
        else
        {
            const double remaining=leg_.stopTime-leg_.stopElapsed;
            leg_.position=leg_.stopPosition-0.5*leg_.deceleration*remaining*remaining;
            leg_.speed=leg_.deceleration*remaining;leg_.accelerationNow=-leg_.deceleration;
        }
    }
    else
    {
        leg_.elapsed+=dt;
        const double total=leg_.accelerationTime+leg_.cruiseTime+leg_.decelerationTime;
        if(leg_.elapsed>=total)
        {leg_.position=leg_.length;leg_.speed=0.0;leg_.accelerationNow=0.0;finished=true;}
        else if(leg_.elapsed<leg_.accelerationTime)
        {
            leg_.position=leg_.basePosition+0.5*leg_.acceleration*leg_.elapsed*leg_.elapsed;
            leg_.speed=leg_.acceleration*leg_.elapsed;leg_.accelerationNow=leg_.acceleration;
        }
        else if(leg_.elapsed<leg_.accelerationTime+leg_.cruiseTime)
        {
            leg_.position=leg_.basePosition+leg_.accelerationDistance+leg_.peak*(leg_.elapsed-leg_.accelerationTime);
            leg_.speed=leg_.peak;leg_.accelerationNow=0.0;
        }
        else
        {
            const double remaining=total-leg_.elapsed;
            leg_.position=leg_.length-0.5*leg_.deceleration*remaining*remaining;
            leg_.speed=leg_.deceleration*remaining;leg_.accelerationNow=-leg_.deceleration;
        }
    }
    const double tolerance=(std::max)(8.0*request_.geometry.coordinateErrorBoundMm,
        64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,leg_.length));
    if(!std::isfinite(leg_.position) || !std::isfinite(leg_.speed) || leg_.speed<0.0 ||
       leg_.position<previous-tolerance || leg_.position>leg_.length+tolerance)return false;
    leg_.position=(std::min)(leg_.length,(std::max)(previous,leg_.position));
    return EvaluateLeg();
}
inline bool Cycle::StartBrake(Phase phase) noexcept
{
    const double stopTime=leg_.speed/leg_.deceleration;
    double stopPosition=leg_.position+0.5*stopTime*leg_.speed;
    const double tolerance=(std::max)(8.0*request_.geometry.coordinateErrorBoundMm,
        64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,leg_.length));
    if(!std::isfinite(stopTime) || !std::isfinite(stopPosition) || stopPosition>leg_.length+tolerance)
    {Fail(Fault::Numeric);return false;}
    stopPosition=(std::min)(leg_.length,stopPosition);
    leg_.stopTime=stopTime;leg_.stopPosition=stopPosition;leg_.stopElapsed=0.0;
    feedback_.phase=phase;RefreshOutput();return true;
}
inline void Cycle::StartSettle(Phase phase,SettleAction action,const Input& input) noexcept
{
    feedback_.phase=phase;settleAction_=action;settleTick_=input.tick;settlePhysical_=input.physicalSequence;
    settledSamples_=0U;settleElapsed_=0.0;settleProvedSeconds_=0.0;
    leg_.speed=0.0;leg_.accelerationNow=0.0;
    output_.velocityNative.fill(0.0);output_.accelerationNative.fill(0.0);RefreshOutput();
}
inline bool Cycle::Settled(const Input& input,double dt) noexcept
{
    settleElapsed_+=dt;
    if(settleElapsed_>request_.settleTimeoutSeconds) {Fail(Fault::SettleTimeout);return false;}
    const bool proof=input.tick>settleTick_ && input.physicalSequence>settlePhysical_ &&
        input.physicalSettled && input.zeroFinalSend && input.zeroSendTick>=settleTick_ &&
        input.zeroSendTick<input.tick && input.tick-input.zeroSendTick==1ULL && AtCommand(input.actualMCS);
    settledSamples_=proof?settledSamples_+1U:0U;
    settleProvedSeconds_=proof?settleProvedSeconds_+dt:0.0;
    return settledSamples_>=request_.settleSamples && settleProvedSeconds_>=request_.minimumSettleSeconds;
}
inline void Cycle::NextNominal(const Input& input) noexcept
{
    switch(feedback_.nominalPhase)
    {
    case Phase::InitialDepth:(void)StartNominal(Phase::InitialRetreat);break;
    case Phase::InitialRetreat:(void)StartNominal(Phase::CircleEntry);break;
    case Phase::CircleEntry:(void)StartNominal(Phase::Orbit);break;
    case Phase::Orbit:(void)StartNominal(Phase::ReturnCenter);break;
    case Phase::ReturnCenter:(void)StartNominal(Phase::AdvanceCenter);break;
    case Phase::AdvanceCenter:
        ++feedback_.completedLayers;
        if(feedback_.layerIndex+1U==request_.geometry.circleLayerCount)
        {
            output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
            leg_.speed=0.0;RefreshOutput();
        }
        else {++feedback_.layerIndex;(void)StartNominal(Phase::CircleEntry);}
        break;
    default:Fail(Fault::Numeric);break;
    }
    (void)input;
}
inline void Cycle::SaveHoldAnchor(const Input& input) noexcept
{
    savedLeg_=leg_;savedPhase_=feedback_.nominalPhase;savedAnchor_=output_.commandMCS;
    savedCenter_=(savedPhase_==Phase::CircleEntry || savedPhase_==Phase::Orbit || savedPhase_==Phase::ReturnCenter)
        ?leg_.center:savedAnchor_;
    feedback_.holdActive=true;
    StartSettle(Phase::HoldSettle,SettleAction::HoldCenter,input);
}
inline void Cycle::BeginHold(const Input& input) noexcept
{
    feedback_.holdSequence=input.holdSequence;feedback_.holdActive=true;
    if(leg_.speed==0.0)SaveHoldAnchor(input);
    else (void)StartBrake(Phase::HoldBrake);
}
inline void Cycle::ContinueAfterSettle(const Input& input) noexcept
{
    switch(settleAction_)
    {
    case SettleAction::BeginNominal:
        feedback_.phase=feedback_.nominalPhase;RefreshOutput();break;
    case SettleAction::NextNominal:NextNominal(input);break;
    case SettleAction::HoldCenter:(void)StartLine(savedCenter_,Phase::HoldToCenter,request_.returnFeedMmMin);break;
    case SettleAction::HoldStart:(void)StartLine(request_.geometry.startMCS,Phase::HoldToStart,request_.returnFeedMmMin);break;
    case SettleAction::Held:
        feedback_.phase=Phase::HeldAtStart;heldStartFloor_=input.startSequence;RefreshOutput();break;
    case SettleAction::ResumeCenter:(void)StartLine(savedCenter_,Phase::ResumeToCenter,request_.returnFeedMmMin);break;
    case SettleAction::ResumeAnchor:(void)StartLine(savedAnchor_,Phase::ResumeToAnchor,request_.returnFeedMmMin);break;
    case SettleAction::RestoreNominal:
        leg_=savedLeg_;leg_.speed=0.0;leg_.accelerationNow=0.0;
        if(!StartProfile(leg_)) {Fail(Fault::Numeric);break;}
        output_.commandMCS=savedAnchor_;feedback_.nominalPhase=savedPhase_;feedback_.holdActive=false;
        feedback_.phase=savedPhase_;RefreshOutput();break;
    case SettleAction::Complete:feedback_.phase=Phase::Complete;RefreshOutput();break;
    case SettleAction::Fault:Fail(Fault::RepeatedHold,false);break;
    }
}
inline bool Cycle::Begin(const Request& request,const Input& input) noexcept
{
    if(output_.active)return false;
    const Request frozen=request;
    Reset();initialized_=true;request_=frozen;feedback_.session=frozen.session;feedback_.tick=input.tick;
    if(Detail::Finite(frozen.geometry.startMCS))output_.commandMCS=frozen.geometry.startMCS;
    if(!ValidateRequest(frozen) || !input.tick || !input.nowUs || !input.physicalSequence ||
       !Detail::Finite(input.actualMCS)) {Fail(Fault::InvalidRequest,false);return false;}
    if(input.cancel){Fail(Fault::Cancelled,false);return false;}
    if(!input.sourceValid){Fail(Fault::SourceLost);return false;}
    if(!input.pdoValid){Fail(Fault::PdoLost);return false;}
    if(!input.powerInhibited){Fail(Fault::PowerNotInhibited);return false;}
    if(!InsideTravel(input.actualMCS) || !AtCommand(input.actualMCS)) {Fail(Fault::TravelLimit,false);return false;}
    lastTick_=input.tick;lastNowUs_=input.nowUs;lastPhysical_=input.physicalSequence;
    lastHold_=input.holdSequence;lastStart_=input.startSequence;lastHoldLevel_=input.hold;
    if(!StartNominal(Phase::InitialDepth))return false;
    if(input.hold)BeginHold(input);
    else StartSettle(Phase::Settle,SettleAction::BeginNominal,input);
    return !output_.faulted;
}
inline Output Cycle::Step(const Input& input) noexcept
{
    if(!initialized_ || !output_.active)return output_;
    feedback_.tick=input.tick;
    if(input.cancel){Fail(Fault::Cancelled,false);return output_;}
    if(!input.sourceValid){Fail(Fault::SourceLost);return output_;}
    if(!input.pdoValid){Fail(Fault::PdoLost);return output_;}
    if(!input.powerInhibited){Fail(Fault::PowerNotInhibited);return output_;}
    if(input.tick<=lastTick_ || input.tick-lastTick_!=1ULL || input.nowUs<=lastNowUs_ ||
       input.nowUs-lastNowUs_<request_.minimumTickUs || input.nowUs-lastNowUs_>request_.maximumTickUs)
    {Fail(Fault::Clock);return output_;}
    if(input.physicalSequence<=lastPhysical_){Fail(Fault::PhysicalSequence);return output_;}
    if(input.holdSequence<lastHold_ || input.startSequence<lastStart_){Fail(Fault::IntentSequence);return output_;}
    if(!InsideTravel(input.actualMCS)){Fail(Fault::TravelLimit);return output_;}
    const double dt=static_cast<double>(input.nowUs-lastNowUs_)*1e-6;
    const bool holdEdge=(input.hold && !lastHoldLevel_) || input.holdSequence>lastHold_;
    lastTick_=input.tick;lastNowUs_=input.nowUs;lastPhysical_=input.physicalSequence;
    lastHold_=input.holdSequence;lastStart_=input.startSequence;lastHoldLevel_=input.hold;
    elapsed_+=dt;
    if(!std::isfinite(elapsed_) || elapsed_>request_.maximumCycleSeconds){Fail(Fault::CycleTimeout);return output_;}

    if(holdEdge)
    {
        if(!feedback_.holdActive && (Detail::IsNominal(feedback_.phase) || feedback_.phase==Phase::Settle))
            BeginHold(input);
        else if(feedback_.phase==Phase::HeldAtStart)
            heldStartFloor_=input.startSequence;
        else if(feedback_.phase!=Phase::FaultBrake && feedback_.phase!=Phase::FaultSettle)
        {
            // A second HOLD cancels an excursion/resume instead of queuing an
            // automatic restart. Brake on its current finite leg, then settle.
            if(leg_.speed>0.0)(void)StartBrake(Phase::FaultBrake);
            else StartSettle(Phase::FaultSettle,SettleAction::Fault,input);
        }
    }
    if(output_.faulted)return output_;
    const Phase phase=feedback_.phase;
    if(Detail::IsSettling(phase))
    {
        if(Settled(input,dt))ContinueAfterSettle(input);
    }
    else if(phase==Phase::HeldAtStart)
    {
        if(!AtCommand(input.actualMCS)){Fail(Fault::TravelLimit);return output_;}
        if(!input.hold && input.startSequence>heldStartFloor_)
        {
            feedback_.resumeSequence=input.startSequence;
            StartSettle(Phase::HoldSettle,SettleAction::ResumeCenter,input);
        }
    }
    else if(Detail::IsNominal(phase) || Detail::IsExcursion(phase) || phase==Phase::HoldBrake || phase==Phase::FaultBrake)
    {
        bool finished=false;
        if(!AdvanceLeg(dt,phase==Phase::HoldBrake || phase==Phase::FaultBrake,finished))
        {Fail(Fault::Numeric);return output_;}
        if(finished)
        {
            if(phase==Phase::HoldBrake)SaveHoldAnchor(input);
            else if(phase==Phase::FaultBrake)StartSettle(Phase::FaultSettle,SettleAction::Fault,input);
            else if(phase==Phase::HoldToCenter)StartSettle(Phase::HoldSettle,SettleAction::HoldStart,input);
            else if(phase==Phase::HoldToStart)StartSettle(Phase::HoldSettle,SettleAction::Held,input);
            else if(phase==Phase::ResumeToCenter)StartSettle(Phase::HoldSettle,SettleAction::ResumeAnchor,input);
            else if(phase==Phase::ResumeToAnchor)StartSettle(Phase::HoldSettle,SettleAction::RestoreNominal,input);
            else StartSettle(Phase::Settle,SettleAction::NextNominal,input);
        }
    }
    else Fail(Fault::Numeric);
    RefreshOutput();return output_;
}
} // namespace NCG76SphereCycle
