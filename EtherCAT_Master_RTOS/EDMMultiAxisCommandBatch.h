#pragma once

#include "EDMMultiAxisTrajectoryPreview.h"

// EDM92: logical six-axis PRE-PID command-batch shadow. Native position and
// velocity references are doubles; hardwareSign belongs downstream and is NOT
// applied here. One RT owner stages and validates all axes before one logical
// commit. This is not a concurrent mailbox, PDO serializer or motion permit.
namespace EDM92
{
enum class Kind : std::uint8_t { Raw,EndpointDrain,OperatorStop };
enum class State : std::uint8_t { Idle,Running,Revoked,Completed,Stopped,Aborted,Retired };
enum class Reason : std::uint8_t
{ None,Configuration,Identity,Sequence,Clock,Evidence,Mapping,History,Lifecycle,External };
inline const char* KindName(Kind k) noexcept
{ switch(k){case Kind::Raw:return "RAW";case Kind::EndpointDrain:return "ENDPOINT_DRAIN";case Kind::OperatorStop:return "OPERATOR_STOP";}return "UNKNOWN"; }
inline const char* StateName(State s) noexcept
{ switch(s){case State::Idle:return "IDLE";case State::Running:return "RUNNING";case State::Revoked:return "REVOKED";case State::Completed:return "COMPLETED";case State::Stopped:return "STOPPED";case State::Aborted:return "ABORTED";case State::Retired:return "RETIRED";}return "UNKNOWN"; }
inline const char* ReasonName(Reason r) noexcept
{ switch(r){case Reason::None:return "NONE";case Reason::Configuration:return "CONFIGURATION";case Reason::Identity:return "IDENTITY";case Reason::Sequence:return "SEQUENCE";case Reason::Clock:return "CLOCK";case Reason::Evidence:return "EVIDENCE";case Reason::Mapping:return "MAPPING";case Reason::History:return "HISTORY";case Reason::Lifecycle:return "LIFECYCLE";case Reason::External:return "EXTERNAL";}return "UNKNOWN"; }
struct Evidence
{
    EDM91::Evidence trajectory{};
    EDM81::Pose nativeVelocityPps{};
    std::uint64_t acceptedBatches=0U,rawBatches=0U,drainBatches=0U,stopBatches=0U;
    State state=State::Idle;Reason reason=Reason::None;Kind lastKind=Kind::Raw;
    unsigned rejectedAxis=6U;
    bool candidateValid=false,numericalStopped=false,revoked=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
static_assert(sizeof(Evidence)<=800U,"EDM92 evidence must fit the bounded feedback record");
// History deliberately excludes lifecycle metadata: Finish, RequestStop,
// Abort and Retire do not produce another accepted command sample.
inline bool SameHistory(const EDM91::Evidence& a,const EDM91::Evidence& b) noexcept
{
    return a.session==b.session && EDM81::SameScope(a.scope,b.scope) && a.mapGeneration==b.mapGeneration &&
        a.captureTick==b.captureTick && a.lastTick==b.lastTick && a.rawFrames==b.rawFrames &&
        a.shapeSteps==b.shapeSteps && a.tailSteps==b.tailSteps && a.drainSteps==b.drainSteps && a.stopSteps==b.stopSteps &&
        a.sourcePose==b.sourcePose && a.shapedPose==b.shapedPose && a.displacement==b.displacement &&
        a.nativeTarget==b.nativeTarget && a.velocityUnitsPerSecond==b.velocityUnitsPerSecond &&
        a.maximumVelocityUnitsPerSecond==b.maximumVelocityUnitsPerSecond &&
        a.maximumAccelerationUnitsPerSecond2==b.maximumAccelerationUnitsPerSecond2 &&
        a.maximumTravelUnits==b.maximumTravelUnits && a.maximumTrackingErrorUnits==b.maximumTrackingErrorUnits;
}
inline bool SameTerminalEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return EDM91::SameTerminalEvidence(a.trajectory,b.trajectory) && a.trajectory.state==b.trajectory.state &&
        a.trajectory.candidateValid==b.trajectory.candidateValid && a.nativeVelocityPps==b.nativeVelocityPps &&
        a.acceptedBatches==b.acceptedBatches && a.rawBatches==b.rawBatches && a.drainBatches==b.drainBatches &&
        a.stopBatches==b.stopBatches && a.reason==b.reason && a.lastKind==b.lastKind &&
        a.rejectedAxis==b.rejectedAxis && a.numericalStopped==b.numericalStopped;
}
inline bool IsEmpty(const Evidence& e) noexcept
{ const Evidence empty{};return e.state==State::Idle && !e.candidateValid && !e.revoked && SameTerminalEvidence(e,empty); }
inline unsigned StopTicks(const EDM90::Config& c,const EDM91::Evidence& e) noexcept
{
    double n=0.0;
    for(unsigned i=0;i<6U;++i) n=EDM81::Maximum(n,std::fabs(e.velocityUnitsPerSecond[i])/(EDM91::AccelerationLimitUnitsPerSecond2(c,i)*EDM91::Seconds));
    if(!EDM91::Within(n,static_cast<double>(EDM91::MaximumStopTicks))) return EDM91::MaximumStopTicks+1U;
    const unsigned ticks=static_cast<unsigned>(std::ceil(n));
    return ticks>EDM91::MaximumStopTicks?EDM91::MaximumStopTicks:ticks;
}
inline bool ValidEvidence(const EDM90::Config& c,const Evidence& e) noexcept
{
    const auto& s=e.trajectory;
    if(!EDM91::ValidEvidence(c,s) || s.reason!=EDM91::Reason::None || s.rejectedAxis!=6U ||
        s.state==EDM91::State::Idle || s.state==EDM91::State::Aborted || s.state==EDM91::State::Retired ||
        e.state==State::Idle || static_cast<unsigned>(e.state)>static_cast<unsigned>(State::Retired) ||
        static_cast<unsigned>(e.reason)>static_cast<unsigned>(Reason::External) ||
        static_cast<unsigned>(e.lastKind)>static_cast<unsigned>(Kind::OperatorStop) || e.rejectedAxis>6U ||
        (e.reason==Reason::None && e.rejectedAxis!=6U) ||
        e.acceptedBatches!=s.shapeSteps || e.rawBatches!=s.rawFrames || e.drainBatches!=s.drainSteps || e.stopBatches!=s.stopSteps ||
        (e.state!=State::Aborted && e.state!=State::Retired && (e.reason!=Reason::None || e.rejectedAxis!=6U)) ||
        (e.state==State::Aborted && (e.reason==Reason::None || e.candidateValid || e.numericalStopped)) ||
        (e.reason!=Reason::None && (e.state!=State::Aborted && e.state!=State::Retired)) ||
        (e.reason!=Reason::None && (e.candidateValid || e.numericalStopped)) ||
        (e.candidateValid!=(e.state==State::Running && e.acceptedBatches!=0U)) ||
        (e.state==State::Running && (e.revoked || e.numericalStopped || e.stopBatches ||
            (s.state!=EDM91::State::Running && s.state!=EDM91::State::Draining))) ||
        (e.state==State::Revoked && (!e.revoked || e.candidateValid || e.numericalStopped)) ||
        (e.state==State::Completed && (e.revoked || !e.numericalStopped || e.candidateValid || s.state!=EDM91::State::Completed)) ||
        (e.state==State::Stopped && (!e.revoked || !e.numericalStopped || e.candidateValid || s.state!=EDM91::State::Stopped)) ||
        (e.state==State::Retired && (e.candidateValid || ((e.reason==Reason::None)!=e.numericalStopped))) ||
        (e.numericalStopped && (!EDM91::ZeroVelocity(s) ||
            (e.state!=State::Completed && e.state!=State::Stopped && e.state!=State::Retired))) ||
        (e.state==State::Retired && e.reason==Reason::None && !e.revoked && s.state!=EDM91::State::Completed) ||
        (e.stopBatches && (!e.revoked || e.lastKind!=Kind::OperatorStop ||
            (s.state!=EDM91::State::Stopping && s.state!=EDM91::State::Stopped))) ||
        (!e.stopBatches && e.drainBatches && e.lastKind!=Kind::EndpointDrain) ||
        (!e.stopBatches && !e.drainBatches && (e.lastKind!=Kind::Raw || s.state!=EDM91::State::Running || s.candidateValid!=(e.acceptedBatches!=0U))) ||
        (!e.stopBatches && e.drainBatches && (s.candidateValid || (s.state!=EDM91::State::Draining && s.state!=EDM91::State::Completed))) ||
        (!e.acceptedBatches && (s.state!=EDM91::State::Running || s.candidateValid || s.stopTicks || e.candidateValid))) return false;
    for(unsigned i=0;i<6U;++i) {
        const double velocity=s.velocityUnitsPerSecond[i]*c.machine.axes[i].pulsesPerUnit;
        if(!EDM81::Finite(velocity) || !EDM81::Finite(e.nativeVelocityPps[i]) || e.nativeVelocityPps[i]!=velocity) return false;
    }
    return true;
}
// A bounded one-sample lead is legal only after a rejected batch: trajectory
// sampling succeeded first, then the bridge rejected the full command group.
// That unaccepted sample never becomes command evidence or a stopped proof.
inline bool BoundTo(const EDM90::Config& c,const Evidence& e,const EDM91::Evidence& s) noexcept
{
    if(!ValidEvidence(c,e) || !EDM91::ValidEvidence(c,s)) return false;
    const auto& p=e.trajectory;
    if(s.shapeSteps==p.shapeSteps) {
        if(!SameHistory(p,s)) return false;
        if(s.state==EDM91::State::Aborted || (s.state==EDM91::State::Retired && s.reason!=EDM91::Reason::None))
            return (e.state==State::Aborted || (e.state==State::Retired && e.reason!=Reason::None)) && !e.numericalStopped;
        if(e.reason!=Reason::None) return false;
        if(e.numericalStopped && (!s.numericalStopped || !EDM91::ZeroVelocity(s))) return false;
        if(s.state==EDM91::State::Stopping || s.state==EDM91::State::Stopped ||
            (s.state==EDM91::State::Retired && e.revoked)) {
            if(!e.revoked) return false;
            if(p.stopSteps) return s.stopTicks==p.stopTicks;
            return s.stopTicks==StopTicks(c,p);
        }
        return s.stopTicks==p.stopTicks;
    }
    const auto max=(std::numeric_limits<std::uint64_t>::max)();
    if(e.reason==Reason::None || e.numericalStopped || p.shapeSteps==max || s.shapeSteps!=p.shapeSteps+1U ||
        (s.state!=EDM91::State::Aborted && s.state!=EDM91::State::Retired) || s.reason==EDM91::Reason::None) return false;
    const bool raw=s.rawFrames==p.rawFrames+1U && s.drainSteps==p.drainSteps && s.stopSteps==p.stopSteps && !p.drainSteps && !p.stopSteps;
    const bool drain=s.rawFrames==p.rawFrames && s.drainSteps==p.drainSteps+1U && s.stopSteps==p.stopSteps && !p.stopSteps;
    const bool stop=s.rawFrames==p.rawFrames && s.drainSteps==p.drainSteps && s.stopSteps==p.stopSteps+1U && e.revoked;
    return raw || drain || stop;
}
class Bridge
{
public:
    const Evidence& Current() const noexcept { return evidence_; }
    bool Begin(const EDM90::Config& c) noexcept
    {
        if(evidence_.state==State::Running || evidence_.state==State::Revoked || evidence_.state==State::Completed || evidence_.state==State::Stopped)
            return Fail(Reason::Lifecycle);
        if(!EDM91::Valid(c)) return Fail(Reason::Configuration);
        if(c.session<=latestSession_) return Fail(Reason::Identity);
        config_=c;latestSession_=c.session;evidence_=Evidence{};
        auto& s=evidence_.trajectory;s.session=c.session;s.scope=c.scope;s.mapGeneration=c.machine.mapGeneration;
        s.captureTick=c.machine.tick;s.state=EDM91::State::Running;s.sourcePose=s.shapedPose=c.initialPose;
        for(unsigned i=0;i<6U;++i) s.nativeTarget[i]=c.machine.axes[i].anchorNativePulse;
        evidence_.state=State::Running;stopOrigin_={};stopVelocity_={};return true;
    }
    bool Accept(const EDM91::Evidence& s,Kind kind,std::uint64_t tick) noexcept
    {
        if(evidence_.state!=State::Running && evidence_.state!=State::Revoked) return Fail(Reason::Lifecycle);
        if(static_cast<unsigned>(kind)>static_cast<unsigned>(Kind::OperatorStop) ||
            (kind==Kind::OperatorStop)!=evidence_.revoked) return Fail(Reason::Lifecycle);
        if(s.session!=config_.session || !EDM81::SameScope(s.scope,config_.scope) ||
            s.mapGeneration!=config_.machine.mapGeneration || s.captureTick!=config_.machine.tick) return Fail(Reason::Identity);
        const auto& p=evidence_.trajectory;const auto max=(std::numeric_limits<std::uint64_t>::max)();
        if(p.shapeSteps==max || s.shapeSteps!=p.shapeSteps+1U) return Fail(Reason::Sequence);
        if(!tick || tick!=s.lastTick || (p.shapeSteps? (p.lastTick==max || tick!=p.lastTick+1U):tick!=config_.machine.tick))
            return Fail(Reason::Clock);
        if(!EDM91::ValidEvidence(config_,s) || s.reason!=EDM91::Reason::None || s.rejectedAxis!=6U) return Fail(Reason::Evidence);
        if(kind==Kind::Raw) {
            if(p.drainSteps || p.stopSteps || s.state!=EDM91::State::Running || !s.candidateValid ||
                s.rawFrames!=p.rawFrames+1U || s.tailSteps || s.drainSteps || s.stopSteps || s.stopTicks) return Fail(Reason::Sequence);
        } else if(kind==Kind::EndpointDrain) {
            if(!p.rawFrames || p.stopSteps || s.rawFrames!=p.rawFrames || s.drainSteps!=p.drainSteps+1U ||
                s.stopSteps || s.stopTicks || s.candidateValid || s.sourcePose!=p.sourcePose ||
                (s.state!=EDM91::State::Draining && s.state!=EDM91::State::Completed)) return Fail(Reason::Sequence);
        } else {
            const unsigned ticks=p.stopSteps?static_cast<unsigned>(p.stopTicks):StopTicks(config_,p);
            if(!ticks || s.rawFrames!=p.rawFrames || s.drainSteps!=p.drainSteps || s.stopSteps!=p.stopSteps+1U ||
                s.stopTicks!=ticks || s.candidateValid || s.sourcePose!=p.sourcePose ||
                (s.state!=EDM91::State::Stopping && s.state!=EDM91::State::Stopped)) return Fail(Reason::Sequence);
        }
        // Stage every axis. Neither accepted counters nor any target changes if
        // axis V (or any preceding axis) rejects this candidate.
        EDM81::Pose nativeVelocity{};
        for(unsigned i=0;i<6U;++i) {
            const double v=(s.displacement[i]-p.displacement[i])/EDM91::Seconds;
            const double a=std::fabs(v-p.velocityUnitsPerSecond[i])/EDM91::Seconds;
            const double tracking=std::fabs((s.sourcePose[i]-config_.initialPose[i])-s.displacement[i]);
            nativeVelocity[i]=s.velocityUnitsPerSecond[i]*config_.machine.axes[i].pulsesPerUnit;
            if(!EDM81::Finite(nativeVelocity[i])) return Fail(Reason::Mapping,i);
            if(s.velocityUnitsPerSecond[i]!=v ||
                s.maximumVelocityUnitsPerSecond[i]!=EDM81::Maximum(p.maximumVelocityUnitsPerSecond[i],std::fabs(v)) ||
                s.maximumAccelerationUnitsPerSecond2[i]!=EDM81::Maximum(p.maximumAccelerationUnitsPerSecond2[i],a) ||
                s.maximumTravelUnits[i]!=EDM81::Maximum(p.maximumTravelUnits[i],std::fabs(s.displacement[i])) ||
                s.maximumTrackingErrorUnits[i]!=EDM81::Maximum(p.maximumTrackingErrorUnits[i],tracking)) return Fail(Reason::History,i);
            if(kind==Kind::Raw && !EDM91::Within(std::fabs((s.sourcePose[i]-p.sourcePose[i])/EDM91::Seconds),EDM91::VelocityLimitUnitsPerSecond(config_,i)))
                return Fail(Reason::History,i);
            if(kind==Kind::OperatorStop) {
                const double k=static_cast<double>(s.stopSteps),m=static_cast<double>(s.stopTicks);
                const double distance=k-k*(k+1.0)/(2.0*m);
                if(s.displacement[i]!=stopOrigin_[i]+stopVelocity_[i]*EDM91::Seconds*distance) return Fail(Reason::History,i);
            }
        }
        evidence_.trajectory=s;evidence_.nativeVelocityPps=nativeVelocity;
        ++evidence_.acceptedBatches;
        if(kind==Kind::Raw) ++evidence_.rawBatches;else if(kind==Kind::EndpointDrain) ++evidence_.drainBatches;else ++evidence_.stopBatches;
        evidence_.lastKind=kind;evidence_.candidateValid=!evidence_.revoked && !s.numericalStopped;
        evidence_.numericalStopped=s.numericalStopped;
        if(s.state==EDM91::State::Completed) evidence_.state=State::Completed;
        else if(s.state==EDM91::State::Stopped) evidence_.state=State::Stopped;
        return true;
    }
    void Revoke() noexcept
    {
        if(evidence_.state==State::Idle || evidence_.state==State::Retired || evidence_.revoked) return;
        if(evidence_.state==State::Aborted) { evidence_.revoked=true;return; }
        evidence_.revoked=true;evidence_.candidateValid=false;evidence_.numericalStopped=false;evidence_.state=State::Revoked;
        stopOrigin_=evidence_.trajectory.displacement;stopVelocity_=evidence_.trajectory.velocityUnitsPerSecond;
    }
    void Abort(Reason reason=Reason::External) noexcept
    { if(evidence_.state!=State::Idle && evidence_.state!=State::Aborted && evidence_.state!=State::Retired) (void)Fail(reason==Reason::None?Reason::External:reason); }
    bool Retire(const EDM91::Evidence& terminal) noexcept
    {
        if(evidence_.state==State::Idle) return EDM91::IsEmpty(terminal);
        if(evidence_.state==State::Retired) return BoundTo(config_,evidence_,terminal) && terminal.state==EDM91::State::Retired;
        if(terminal.state!=EDM91::State::Retired || !BoundTo(config_,evidence_,terminal)) return Fail(Reason::Lifecycle);
        const bool failed=evidence_.state==State::Aborted;
        if(!failed && (!terminal.numericalStopped || !EDM91::ZeroVelocity(terminal) ||
            (!evidence_.revoked && evidence_.state!=State::Completed))) return Fail(Reason::Lifecycle);
        evidence_.state=State::Retired;evidence_.candidateValid=false;evidence_.numericalStopped=!failed;return true;
    }
private:
    EDM90::Config config_{};Evidence evidence_{};
    EDM81::Pose stopOrigin_{},stopVelocity_{};
    std::uint64_t latestSession_=0U;
    bool Fail(Reason r,unsigned axis=6U) noexcept
    { evidence_.state=State::Aborted;evidence_.reason=r;evidence_.rejectedAxis=axis;evidence_.candidateValid=false;evidence_.numericalStopped=false;return false; }
};
}
