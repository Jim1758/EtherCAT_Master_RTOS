#pragma once

#include "EDMMultiAxisOutputPreview.h"

// EDM91 JOINT-SPACE NUMERICAL PREVIEW ONLY. A common fixed window smooths
// six relative engineering-unit coordinates. This changes corners and TCP
// geometry; it is NOT a path-preserving planner or a physical stop proof.
// There are no allocations, locks, clock reads, axis/PDO setters or permits.
namespace EDM91
{
constexpr unsigned Window=64U;
constexpr unsigned MaximumDrainTicks=Window;
constexpr unsigned MaximumStopTicks=Window/2U;
constexpr double Seconds=.00025;
enum class State : std::uint8_t { Idle,Running,Draining,Stopping,Completed,Stopped,Aborted,Retired };
enum class Reason : std::uint8_t
{ None,Configuration,Identity,Sequence,Clock,NonFinite,Velocity,Acceleration,Envelope,NativeLimit,StopEnvelope,CounterOverflow,Lifecycle,External };
inline const char* StateName(State s) noexcept
{
    switch(s) {case State::Idle:return "IDLE";case State::Running:return "RUNNING";
    case State::Draining:return "DRAINING";case State::Stopping:return "STOPPING";
    case State::Completed:return "COMPLETED";case State::Stopped:return "STOPPED";
    case State::Aborted:return "ABORTED";case State::Retired:return "RETIRED";} return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) {case Reason::None:return "NONE";case Reason::Configuration:return "CONFIGURATION";
    case Reason::Identity:return "IDENTITY";case Reason::Sequence:return "SEQUENCE";
    case Reason::Clock:return "CLOCK";case Reason::NonFinite:return "NONFINITE";
    case Reason::Velocity:return "VELOCITY";case Reason::Acceleration:return "ACCELERATION";
    case Reason::Envelope:return "ENVELOPE";case Reason::NativeLimit:return "NATIVE_LIMIT";
    case Reason::StopEnvelope:return "STOP_ENVELOPE";case Reason::CounterOverflow:return "COUNTER_OVERFLOW";
    case Reason::Lifecycle:return "LIFECYCLE";case Reason::External:return "EXTERNAL";} return "UNKNOWN";
}
inline double VelocityLimitUnitsPerSecond(const EDM90::Config& c,unsigned i) noexcept
{ return i<6U?EDM81::Minimum(i<3U?.01:.5,c.machine.axes[i].maxVelocityUnitsPerMin/60.0):0.0; }
inline double AccelerationLimitUnitsPerSecond2(const EDM90::Config& c,unsigned i) noexcept
{ return 2.0*VelocityLimitUnitsPerSecond(c,i)/(static_cast<double>(Window)*Seconds); }
inline bool Within(double value,double limit) noexcept
{ return EDM81::Finite(value) && EDM81::Finite(limit) && limit>0.0 && value<=limit*(1.0+1e-6); }
inline bool Valid(const EDM90::Config& c) noexcept
{
    if(!EDM90::Valid(c)) return false;
    for(unsigned i=0;i<6U;++i) {
        const double v=VelocityLimitUnitsPerSecond(c,i),a=AccelerationLimitUnitsPerSecond2(c,i);
        if(!EDM81::Finite(v) || !EDM81::Finite(a) || v<=0.0 || a<=0.0 || v*Seconds==0.0) return false;
    } return true;
}
struct Evidence
{
    std::uint64_t session=0U,mapGeneration=0U,captureTick=0U,lastTick=0U;
    std::uint64_t rawFrames=0U,shapeSteps=0U,tailSteps=0U,drainSteps=0U,stopSteps=0U,stopTicks=0U;
    EDM81::Scope scope{};
    State state=State::Idle;Reason reason=Reason::None;
    unsigned rejectedAxis=6U;
    EDM81::Pose sourcePose{},shapedPose{},displacement{},nativeTarget{},velocityUnitsPerSecond{};
    EDM81::Pose maximumVelocityUnitsPerSecond{},maximumAccelerationUnitsPerSecond2{},maximumTravelUnits{},maximumTrackingErrorUnits{};
    bool candidateValid=false,numericalStopped=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
inline bool SameTerminalEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return a.session==b.session && a.mapGeneration==b.mapGeneration && a.captureTick==b.captureTick &&
        a.lastTick==b.lastTick && a.rawFrames==b.rawFrames && a.shapeSteps==b.shapeSteps &&
        a.tailSteps==b.tailSteps && a.drainSteps==b.drainSteps && a.stopSteps==b.stopSteps && a.stopTicks==b.stopTicks &&
        EDM81::SameScope(a.scope,b.scope) && a.reason==b.reason && a.rejectedAxis==b.rejectedAxis &&
        a.sourcePose==b.sourcePose && a.shapedPose==b.shapedPose && a.displacement==b.displacement &&
        a.nativeTarget==b.nativeTarget && a.velocityUnitsPerSecond==b.velocityUnitsPerSecond &&
        a.maximumVelocityUnitsPerSecond==b.maximumVelocityUnitsPerSecond &&
        a.maximumAccelerationUnitsPerSecond2==b.maximumAccelerationUnitsPerSecond2 &&
        a.maximumTravelUnits==b.maximumTravelUnits && a.maximumTrackingErrorUnits==b.maximumTrackingErrorUnits &&
        a.numericalStopped==b.numericalStopped;
}
inline bool IsEmpty(const Evidence& e) noexcept
{ const Evidence empty{};return e.state==State::Idle && e.reason==Reason::None && !e.candidateValid && SameTerminalEvidence(e,empty); }
inline bool ZeroVelocity(const Evidence& e) noexcept
{ for(double v:e.velocityUnitsPerSecond) if(v!=0.0) return false;return true; }
inline bool ValidEvidence(const EDM90::Config& c,const Evidence& e) noexcept
{
    const auto max=(std::numeric_limits<std::uint64_t>::max)();
    if(!EDM91::Valid(c) || e.session!=c.session || !EDM81::SameScope(e.scope,c.scope) ||
        e.mapGeneration!=c.machine.mapGeneration || e.captureTick!=c.machine.tick ||
        e.state==State::Idle || static_cast<unsigned>(e.state)>static_cast<unsigned>(State::Retired) ||
        static_cast<unsigned>(e.reason)>static_cast<unsigned>(Reason::External) || e.rejectedAxis>6U ||
        (e.state!=State::Aborted && e.state!=State::Retired && (e.reason!=Reason::None || e.rejectedAxis!=6U)) ||
        (e.state==State::Aborted && (e.reason==Reason::None || e.numericalStopped)) ||
        (e.candidateValid && (e.state!=State::Running || !e.rawFrames)) ||
        (e.numericalStopped && (!ZeroVelocity(e) || (e.state!=State::Completed && e.state!=State::Stopped && e.state!=State::Retired))) ||
        ((e.state==State::Completed || e.state==State::Stopped) && (!e.numericalStopped || e.candidateValid)) ||
        (e.state==State::Completed && (e.shapedPose!=e.sourcePose || !e.rawFrames || e.stopSteps || e.stopTicks || e.drainSteps!=Window)) ||
        (e.state==State::Running && (e.tailSteps || e.drainSteps || e.stopSteps || e.stopTicks || e.numericalStopped)) ||
        (e.state==State::Draining && (!e.rawFrames || e.drainSteps>=Window || e.stopSteps || e.stopTicks || e.numericalStopped)) ||
        (e.state==State::Stopping && (!e.stopTicks || e.stopSteps>=e.stopTicks || e.numericalStopped)) ||
        (e.state==State::Stopped && e.stopSteps!=e.stopTicks) ||
        (e.state==State::Retired && (((e.reason==Reason::None)!=e.numericalStopped) ||
            (e.reason==Reason::None && e.stopSteps!=e.stopTicks))) ||
        (!e.rawFrames && e.shapeSteps) ||
        e.rawFrames>max-e.tailSteps || e.shapeSteps!=e.rawFrames+e.tailSteps ||
        e.drainSteps>MaximumDrainTicks || e.stopSteps>e.stopTicks || e.stopTicks>MaximumStopTicks ||
        e.drainSteps>max-e.stopSteps || e.tailSteps!=e.drainSteps+e.stopSteps ||
        (e.shapeSteps && (e.shapeSteps-1U>max-e.captureTick || e.lastTick!=e.captureTick+e.shapeSteps-1U)) ||
        (!e.shapeSteps && (e.lastTick || e.candidateValid))) return false;
    for(unsigned i=0;i<6U;++i) {
        const auto& a=c.machine.axes[i];const double raw=e.sourcePose[i]-c.initialPose[i];
        const double target=a.anchorNativePulse+e.displacement[i]*a.pulsesPerUnit;
        const double rawTarget=a.anchorNativePulse+raw*a.pulsesPerUnit;
        const double error=std::fabs(raw-e.displacement[i]);
        if(!EDM81::Finite(raw) || !EDM81::Finite(rawTarget) || std::fabs(raw)>EDM90::MaximumTravelUnits(i) ||
            (a.hasMinLimit && rawTarget<a.minNativePulse) || (a.hasMaxLimit && rawTarget>a.maxNativePulse) ||
            !EDM81::Finite(e.shapedPose[i]) || !EDM81::Finite(e.displacement[i]) ||
            (e.shapedPose[i]!=c.initialPose[i]+e.displacement[i] &&
                !(e.displacement[i]==raw && e.shapedPose[i]==e.sourcePose[i])) ||
            !EDM81::Finite(target) || e.nativeTarget[i]!=target ||
            !EDM81::Finite(e.velocityUnitsPerSecond[i]) ||
            !Within(e.maximumVelocityUnitsPerSecond[i],VelocityLimitUnitsPerSecond(c,i)) ||
            e.maximumVelocityUnitsPerSecond[i]<std::fabs(e.velocityUnitsPerSecond[i]) ||
            e.maximumAccelerationUnitsPerSecond2[i]<0.0 ||
            !Within(e.maximumAccelerationUnitsPerSecond2[i],AccelerationLimitUnitsPerSecond2(c,i)) ||
            !EDM81::Finite(e.maximumTravelUnits[i]) || e.maximumTravelUnits[i]<std::fabs(e.displacement[i]) ||
            e.maximumTravelUnits[i]>EDM90::MaximumTravelUnits(i) ||
            !EDM81::Finite(e.maximumTrackingErrorUnits[i]) || e.maximumTrackingErrorUnits[i]<error ||
            e.maximumTrackingErrorUnits[i]>2.0*EDM90::MaximumTravelUnits(i) ||
            (!e.shapeSteps && (e.sourcePose[i]!=c.initialPose[i] || e.displacement[i]!=0.0 ||
                e.velocityUnitsPerSecond[i]!=0.0 || e.maximumVelocityUnitsPerSecond[i]!=0.0 ||
                e.maximumAccelerationUnitsPerSecond2[i]!=0.0 || e.maximumTravelUnits[i]!=0.0 ||
                e.maximumTrackingErrorUnits[i]!=0.0)) ||
            (a.hasMinLimit && target<a.minNativePulse) || (a.hasMaxLimit && target>a.maxNativePulse)) return false;
    } return true;
}
class Preview
{
public:
    const Evidence& Current() const noexcept { return evidence_; }
    bool Begin(const EDM90::Config& c) noexcept
    {
        if(evidence_.state==State::Running || evidence_.state==State::Draining || evidence_.state==State::Stopping)
            return Fail(Reason::Lifecycle);
        if(!EDM91::Valid(c)) return Fail(Reason::Configuration);
        if(c.session<=latestSession_) return Fail(Reason::Identity);
        config_=c;latestSession_=c.session;evidence_=Evidence{};
        evidence_.session=c.session;evidence_.scope=c.scope;evidence_.mapGeneration=c.machine.mapGeneration;
        evidence_.captureTick=c.machine.tick;evidence_.state=State::Running;
        evidence_.sourcePose=evidence_.shapedPose=c.initialPose;
        for(unsigned i=0;i<6U;++i) evidence_.nativeTarget[i]=c.machine.axes[i].anchorNativePulse;
        for(auto& row:ring_) for(double& v:row) v=0.0;
        head_=0U;stopOrigin_={};stopVelocity_={};return true;
    }
    bool Step(const EDM90::Frame& frame) noexcept
    {
        if(evidence_.state!=State::Running) return false;
        evidence_.candidateValid=false;
        if(frame.session!=config_.session || !EDM81::SameScope(frame.scope,config_.scope)) return Fail(Reason::Identity);
        if(evidence_.rawFrames==(std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::CounterOverflow);
        if(frame.sequence!=evidence_.rawFrames+1U) return Fail(Reason::Sequence);
        if(!Clock(frame.tick)) return false;
        EDM81::Pose raw{},next{};
        for(unsigned i=0;i<6U;++i) {
            raw[i]=frame.pose[i]-config_.initialPose[i];
            if(!EDM81::Finite(frame.pose[i]) || !EDM81::Finite(raw[i])) return Fail(Reason::NonFinite,i);
            const double speed=std::fabs((frame.pose[i]-evidence_.sourcePose[i])/Seconds);
            if(!Within(speed,VelocityLimitUnitsPerSecond(config_,i))) return Fail(Reason::Velocity,i);
            if(!Position(raw[i],i,false)) return false;
        }
        Mean(raw,next);
        if(!Commit(next,frame.pose,frame.tick,true)) return false;
        ring_[head_]=raw;head_=(head_+1U)%Window;++evidence_.rawFrames;
        evidence_.candidateValid=true;return true;
    }
    void Finish() noexcept
    {
        if(evidence_.state!=State::Running) return;
        evidence_.candidateValid=false;
        if(!evidence_.rawFrames) { (void)Fail(Reason::Lifecycle);return; }
        evidence_.state=State::Draining;
    }
    bool Drain(std::uint64_t tick) noexcept
    {
        const bool endpoint=evidence_.state==State::Draining;
        if(!endpoint && evidence_.state!=State::Stopping) return false;
        if(!Clock(tick)) return false;
        EDM81::Pose next{},raw{};
        if(endpoint) {
            if(evidence_.drainSteps>=MaximumDrainTicks) return Fail(Reason::Lifecycle);
            for(unsigned i=0;i<6U;++i) raw[i]=evidence_.sourcePose[i]-config_.initialPose[i];
            Mean(raw,next);
            // With 63 repeats the ring becomes constant. Assign its exactly
            // represented source displacement; the 64th tick observes zero.
            if(evidence_.drainSteps+1U>=Window-1U) next=raw;
        } else {
            if(!evidence_.stopTicks || evidence_.stopSteps>=evidence_.stopTicks) return Fail(Reason::Lifecycle);
            const double k=static_cast<double>(evidence_.stopSteps+1U),m=static_cast<double>(evidence_.stopTicks);
            const double distance=k-k*(k+1.0)/(2.0*m);
            for(unsigned i=0;i<6U;++i) next[i]=stopOrigin_[i]+stopVelocity_[i]*Seconds*distance;
        }
        if(!Commit(next,evidence_.sourcePose,tick,false)) return false;
        ++evidence_.tailSteps;
        if(endpoint) {
            ring_[head_]=raw;head_=(head_+1U)%Window;++evidence_.drainSteps;
            if(evidence_.drainSteps==MaximumDrainTicks) {
                if(!ZeroVelocity(evidence_) || evidence_.shapedPose!=evidence_.sourcePose) return Fail(Reason::Lifecycle);
                evidence_.state=State::Completed;evidence_.numericalStopped=true;
            }
        } else {
            ++evidence_.stopSteps;
            if(evidence_.stopSteps==evidence_.stopTicks) {
                if(!ZeroVelocity(evidence_)) return Fail(Reason::Lifecycle);
                evidence_.state=State::Stopped;evidence_.numericalStopped=true;
            }
        } return true;
    }
    void RequestStop() noexcept
    {
        evidence_.candidateValid=false;
        if(evidence_.state==State::Stopping || evidence_.state==State::Stopped ||
            evidence_.state==State::Aborted || evidence_.state==State::Retired || evidence_.state==State::Idle) return;
        if(ZeroVelocity(evidence_)) { evidence_.state=State::Stopped;evidence_.numericalStopped=true;return; }
        unsigned ticks=0U;
        if(!StopCount(evidence_.velocityUnitsPerSecond,ticks)) { (void)Fail(Reason::Acceleration);return; }
        if(!StopPosition(evidence_.displacement,evidence_.velocityUnitsPerSecond,ticks)) return;
        stopOrigin_=evidence_.displacement;stopVelocity_=evidence_.velocityUnitsPerSecond;
        evidence_.stopTicks=ticks;evidence_.state=State::Stopping;evidence_.numericalStopped=false;
    }
    void Abort(Reason reason=Reason::External) noexcept
    { if(evidence_.state!=State::Idle && evidence_.state!=State::Aborted) (void)Fail(reason==Reason::None?Reason::External:reason); }
    void Retire() noexcept
    {
        evidence_.candidateValid=false;
        if(evidence_.state==State::Retired || evidence_.state==State::Idle) return;
        if(evidence_.state!=State::Completed && evidence_.state!=State::Stopped && evidence_.state!=State::Aborted)
        { (void)Fail(Reason::Lifecycle);return; }
        evidence_.state=State::Retired;
    }
private:
    EDM90::Config config_{};Evidence evidence_{};
    std::array<EDM81::Pose,Window> ring_{};
    EDM81::Pose stopOrigin_{},stopVelocity_{};
    std::uint64_t latestSession_=0U;unsigned head_=0U;
    bool Fail(Reason reason,unsigned axis=6U) noexcept
    { evidence_.state=State::Aborted;evidence_.reason=reason;evidence_.rejectedAxis=axis;evidence_.candidateValid=false;evidence_.numericalStopped=false;return false; }
    bool Clock(std::uint64_t tick) noexcept
    {
        const auto max=(std::numeric_limits<std::uint64_t>::max)();
        if(evidence_.shapeSteps==max) return Fail(Reason::CounterOverflow);
        if(!tick || (!evidence_.shapeSteps && tick!=config_.machine.tick) ||
            (evidence_.shapeSteps && (evidence_.lastTick==max || tick!=evidence_.lastTick+1U))) return Fail(Reason::Clock);
        return true;
    }
    void Mean(const EDM81::Pose& incoming,EDM81::Pose& result) const noexcept
    {
        // Bounded compensated summation avoids accumulating rolling-sum drift.
        // The rejected candidate never changes the ring or previous evidence.
        for(unsigned axis=0;axis<6U;++axis) {
            double sum=0.0,correction=0.0;
            for(unsigned row=0;row<Window;++row) {
                const double value=row==head_?incoming[axis]:ring_[row][axis];
                const double adjusted=value-correction,next=sum+adjusted;
                correction=(next-sum)-adjusted;sum=next;
            } result[axis]=sum/static_cast<double>(Window);
        }
    }
    bool Position(double displacement,unsigned i,bool stop) noexcept
    {
        const auto& a=config_.machine.axes[i];const double target=a.anchorNativePulse+displacement*a.pulsesPerUnit;
        if(!EDM81::Finite(displacement) || !EDM81::Finite(target)) return Fail(Reason::NonFinite,i);
        if(std::fabs(displacement)>EDM90::MaximumTravelUnits(i)) return Fail(stop?Reason::StopEnvelope:Reason::Envelope,i);
        if((a.hasMinLimit && target<a.minNativePulse) || (a.hasMaxLimit && target>a.maxNativePulse))
            return Fail(stop?Reason::StopEnvelope:Reason::NativeLimit,i);
        return true;
    }
    bool StopCount(const EDM81::Pose& velocity,unsigned& ticks) const noexcept
    {
        double count=0.0;
        for(unsigned i=0;i<6U;++i) {
            const double need=std::fabs(velocity[i])/(AccelerationLimitUnitsPerSecond2(config_,i)*Seconds);
            if(!EDM81::Finite(need)) return false;
            count=EDM81::Maximum(count,need);
        }
        if(!Within(count,static_cast<double>(MaximumStopTicks))) return false;
        ticks=static_cast<unsigned>(std::ceil(count));
        if(ticks>MaximumStopTicks) ticks=MaximumStopTicks;
        return true;
    }
    bool StopPosition(const EDM81::Pose& position,const EDM81::Pose& velocity,unsigned ticks) noexcept
    {
        const double scale=ticks?Seconds*static_cast<double>(ticks-1U)/2.0:0.0;
        for(unsigned i=0;i<6U;++i) if(!Position(position[i]+velocity[i]*scale,i,true)) return false;
        return true;
    }
    bool Commit(const EDM81::Pose& next,const EDM81::Pose& source,std::uint64_t tick,bool raw) noexcept
    {
        EDM81::Pose velocity{},acceleration{},tracking{};
        for(unsigned i=0;i<6U;++i) {
            if(!Position(next[i],i,false)) return false;
            velocity[i]=(next[i]-evidence_.displacement[i])/Seconds;
            acceleration[i]=std::fabs(velocity[i]-evidence_.velocityUnitsPerSecond[i])/Seconds;
            tracking[i]=std::fabs((source[i]-config_.initialPose[i])-next[i]);
            if(!EDM81::Finite(velocity[i]) || !EDM81::Finite(acceleration[i]) || !EDM81::Finite(tracking[i])) return Fail(Reason::NonFinite,i);
            if(!Within(std::fabs(velocity[i]),VelocityLimitUnitsPerSecond(config_,i))) return Fail(Reason::Velocity,i);
            if(!Within(acceleration[i],AccelerationLimitUnitsPerSecond2(config_,i))) return Fail(Reason::Acceleration,i);
        }
        if(raw || evidence_.state==State::Draining) {
            unsigned ticks=0U;if(!StopCount(velocity,ticks)) return Fail(Reason::Acceleration);
            if(!StopPosition(next,velocity,ticks)) return false;
        }
        for(unsigned i=0;i<6U;++i) {
            // Preserve the exact authored endpoint when subtraction/addition are
            // not bitwise inverse (common with nonzero tool-head offsets).
            evidence_.shapedPose[i]=next[i]==source[i]-config_.initialPose[i]?
                source[i]:config_.initialPose[i]+next[i];
            evidence_.nativeTarget[i]=config_.machine.axes[i].anchorNativePulse+next[i]*config_.machine.axes[i].pulsesPerUnit;
            evidence_.maximumVelocityUnitsPerSecond[i]=EDM81::Maximum(evidence_.maximumVelocityUnitsPerSecond[i],std::fabs(velocity[i]));
            evidence_.maximumAccelerationUnitsPerSecond2[i]=EDM81::Maximum(evidence_.maximumAccelerationUnitsPerSecond2[i],acceleration[i]);
            evidence_.maximumTravelUnits[i]=EDM81::Maximum(evidence_.maximumTravelUnits[i],std::fabs(next[i]));
            evidence_.maximumTrackingErrorUnits[i]=EDM81::Maximum(evidence_.maximumTrackingErrorUnits[i],tracking[i]);
        }
        evidence_.sourcePose=source;evidence_.displacement=next;evidence_.velocityUnitsPerSecond=velocity;
        evidence_.lastTick=tick;++evidence_.shapeSteps;evidence_.candidateValid=false;return true;
    }
};
}
