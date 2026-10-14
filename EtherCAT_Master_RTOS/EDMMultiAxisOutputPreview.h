#pragma once

#include "EDMMultiAxisKernel.h"
#include <array>
#include <cstdint>
#include <limits>

// EDM90: six-axis NATIVE-UNIT PREVIEW ONLY. All values remain local numerical
// evidence. No axis/PDO setter, motion permit, discharge or physical-stop proof
// exists here. Mapping is relative to the frozen actual machine command anchor;
// C/U/V are unwrapped degrees and are never normalized at 0/360 degrees.
namespace EDM90
{
struct MachineAxis
{
    std::uint32_t axisId=0U;
    bool exists=false,rotary=false,virtualAxis=false,servoReady=false,idle=false;
    bool fault=false,hardLimit=false,noCompensation=false,hasMinLimit=false,hasMaxLimit=false;
    double pulsesPerUnit=0.0,anchorNativePulse=0.0,actualNativePulse=0.0;
    double commandVelocityPps=0.0,logicalVelocityPps=0.0,targetVelocityPps=0.0;
    double minNativePulse=-(std::numeric_limits<double>::max)();
    double maxNativePulse=(std::numeric_limits<double>::max)();
    double maxVelocityUnitsPerMin=0.0,referenceOffsetPulse=0.0;
    std::uint64_t configGeneration=0U;
    int hardwareSign=1;
};
struct MachineSnapshot
{
    std::uint64_t tick=0U,nowUs=0U,executionEpoch=0U,ownerLease=0U,mapGeneration=0U;
    unsigned axisCount=0U;
    bool current=false,authorityReady=false,groupQuiet=false,mailboxQuiet=false,otherEDMQuiet=false;
    std::array<MachineAxis,EDM81::AxisCount> axes{};
};
struct Config
{
    std::uint64_t session=0U;
    EDM81::Scope scope{};
    MachineSnapshot machine{};
    EDM81::Pose initialPose{};
    std::uint64_t periodUs=250U;
};
enum class State : std::uint8_t { Idle,Running,Completed,Cancelled,Fault };
enum class Reason : std::uint8_t
{ None,Configuration,Machine,Identity,Clock,Sequence,NonFinite,Envelope,NativeLimit,Velocity,CounterOverflow,Cancelled,Lifecycle };
inline const char* ReasonName(Reason r) noexcept
{
    switch(r)
    {
    case Reason::None:return "NONE";case Reason::Configuration:return "CONFIGURATION";
    case Reason::Machine:return "MACHINE";case Reason::Identity:return "IDENTITY";
    case Reason::Clock:return "CLOCK";case Reason::Sequence:return "SEQUENCE";
    case Reason::NonFinite:return "NONFINITE";case Reason::Envelope:return "ENVELOPE";
    case Reason::NativeLimit:return "NATIVE_LIMIT";case Reason::Velocity:return "VELOCITY";
    case Reason::CounterOverflow:return "COUNTER_OVERFLOW";case Reason::Cancelled:return "CANCELLED";
    case Reason::Lifecycle:return "LIFECYCLE";
    }return "UNKNOWN";
}
inline double MaximumTravelUnits(unsigned axis) noexcept { return axis<3U?.25:5.0; }
inline double MaximumVelocityUnitsPerSecond(unsigned axis) noexcept { return axis<3U?1.0:5.0; }
inline bool AxisReady(const MachineAxis& a,unsigned i) noexcept
{
    if(i>=6U) return false;
    if(a.axisId!=i || !a.exists || a.rotary!=(i>=3U) || a.virtualAxis || !a.servoReady || !a.idle ||
        a.fault || a.hardLimit || !a.noCompensation || !a.configGeneration ||
        (a.hardwareSign!=1 && a.hardwareSign!=-1) ||
        !EDM81::Finite(a.pulsesPerUnit) || a.pulsesPerUnit<=0.0 ||
        !EDM81::Finite(a.anchorNativePulse) || !EDM81::Finite(a.actualNativePulse) ||
        !EDM81::Finite(a.referenceOffsetPulse) || !EDM81::Finite(a.commandVelocityPps) ||
        !EDM81::Finite(a.logicalVelocityPps) || !EDM81::Finite(a.targetVelocityPps) ||
        a.commandVelocityPps!=0.0 || a.logicalVelocityPps!=0.0 || a.targetVelocityPps!=0.0 ||
        !EDM81::Finite(a.maxVelocityUnitsPerMin) || a.maxVelocityUnitsPerMin<=0.0 ||
        !EDM81::Finite(a.minNativePulse) || !EDM81::Finite(a.maxNativePulse) ||
        a.minNativePulse>=a.maxNativePulse ||
        (a.hasMinLimit && a.anchorNativePulse<a.minNativePulse) ||
        (a.hasMaxLimit && a.anchorNativePulse>a.maxNativePulse)) return false;
    const double excursion=MaximumTravelUnits(i)*a.pulsesPerUnit;
    const double cap=EDM81::Minimum(a.maxVelocityUnitsPerMin/60.0,MaximumVelocityUnitsPerSecond(i))*a.pulsesPerUnit;
    if(!EDM81::Finite(excursion) || !EDM81::Finite(cap) || cap<=0.0 ||
        !EDM81::Finite(a.anchorNativePulse-excursion) || !EDM81::Finite(a.anchorNativePulse+excursion)) return false;
    return true;
}
// Bits 0..5 name rejected XYZ/CUV axes. Higher bits distinguish a missing or
// stale machine snapshot (8), authority (9), group (10), mailbox (11), and
// another EDM session (12). Ready uses this exact validator, so diagnostics
// cannot disagree with admission. Reserved caller bits 13..15 describe clock,
// binding and Begin-scope failures that are outside this snapshot contract.
inline std::uint64_t ReadyFailureMask(const MachineSnapshot& m) noexcept
{
    std::uint64_t mask=0U;
    if(!m.current || m.axisCount!=6U || !m.tick || !m.nowUs || !m.mapGeneration) mask|=1ULL<<8U;
    if(!m.authorityReady || !m.executionEpoch || !m.ownerLease) mask|=1ULL<<9U;
    if(!m.groupQuiet) mask|=1ULL<<10U;
    if(!m.mailboxQuiet) mask|=1ULL<<11U;
    if(!m.otherEDMQuiet) mask|=1ULL<<12U;
    for(unsigned i=0U;i<6U;++i) if(!AxisReady(m.axes[i],i)) mask|=1ULL<<i;
    return mask;
}
inline bool Ready(const MachineSnapshot& m) noexcept { return ReadyFailureMask(m)==0U; }
inline bool SameBinding(const MachineSnapshot& a,const MachineSnapshot& b) noexcept
{
    if(a.executionEpoch!=b.executionEpoch || a.ownerLease!=b.ownerLease ||
        a.mapGeneration!=b.mapGeneration || a.axisCount!=b.axisCount) return false;
    for(unsigned i=0U;i<6U;++i)
    {
        const auto& x=a.axes[i];const auto& y=b.axes[i];
        if(x.axisId!=y.axisId || x.rotary!=y.rotary || x.virtualAxis!=y.virtualAxis ||
            x.configGeneration!=y.configGeneration || x.pulsesPerUnit!=y.pulsesPerUnit ||
            x.anchorNativePulse!=y.anchorNativePulse || x.referenceOffsetPulse!=y.referenceOffsetPulse ||
            x.hardwareSign!=y.hardwareSign || x.noCompensation!=y.noCompensation ||
            x.hasMinLimit!=y.hasMinLimit || x.hasMaxLimit!=y.hasMaxLimit ||
            x.minNativePulse!=y.minNativePulse || x.maxNativePulse!=y.maxNativePulse ||
            x.maxVelocityUnitsPerMin!=y.maxVelocityUnitsPerMin) return false;
    }
    return true;
}
inline bool Valid(const Config& c) noexcept
{
    if(!c.session || !EDM81::ValidScope(c.scope) || c.periodUs!=250U || !Ready(c.machine) ||
        c.machine.executionEpoch!=c.scope.executionEpoch || c.machine.ownerLease!=c.scope.ownerLease) return false;
    for(double p:c.initialPose) if(!EDM81::Finite(p)) return false;
    return true;
}
inline bool SameConfig(const Config& a,const Config& b) noexcept
{
    const auto& x=a.machine;const auto& y=b.machine;
    if(a.session!=b.session || !EDM81::SameScope(a.scope,b.scope) || a.periodUs!=b.periodUs ||
        a.initialPose!=b.initialPose || x.tick!=y.tick || x.nowUs!=y.nowUs ||
        x.current!=y.current || x.authorityReady!=y.authorityReady || x.groupQuiet!=y.groupQuiet ||
        x.mailboxQuiet!=y.mailboxQuiet || x.otherEDMQuiet!=y.otherEDMQuiet || !SameBinding(x,y)) return false;
    for(unsigned i=0U;i<6U;++i)
    {
        const auto& p=x.axes[i];const auto& q=y.axes[i];
        if(p.exists!=q.exists || p.servoReady!=q.servoReady || p.idle!=q.idle || p.fault!=q.fault ||
            p.hardLimit!=q.hardLimit || p.actualNativePulse!=q.actualNativePulse ||
            p.commandVelocityPps!=q.commandVelocityPps || p.logicalVelocityPps!=q.logicalVelocityPps ||
            p.targetVelocityPps!=q.targetVelocityPps) return false;
    }
    return true;
}
inline bool IsEmpty(const Config& c) noexcept { return SameConfig(c,Config{}); }
struct Frame
{
    std::uint64_t session=0U;
    EDM81::Scope scope{};
    std::uint64_t tick=0U,sequence=0U;
    EDM81::Pose pose{};
};
struct Evidence
{
    std::uint64_t session=0U,mapGeneration=0U,captureTick=0U,lastTick=0U,lastSequence=0U,processedFrames=0U;
    EDM81::Scope scope{};
    State state=State::Idle;Reason reason=Reason::None;
    unsigned axisMovedMask=0U,rejectedAxis=6U;
    EDM81::Pose lastPose{},nativeTarget{},nativeVelocityPps{},maximumVelocityPps{},maximumAccelerationPps2{},maximumTravelUnits{};
    bool candidateValid=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
inline bool SameTerminalEvidence(const Evidence& a,const Evidence& b) noexcept
{
    // Retirement changes only lifecycle state and candidate validity. Historic
    // committed preview evidence must survive cancellation byte-for-value.
    return a.session==b.session && EDM81::SameScope(a.scope,b.scope) &&
        a.mapGeneration==b.mapGeneration && a.captureTick==b.captureTick &&
        a.lastTick==b.lastTick && a.lastSequence==b.lastSequence && a.processedFrames==b.processedFrames &&
        a.axisMovedMask==b.axisMovedMask && a.rejectedAxis==b.rejectedAxis &&
        a.lastPose==b.lastPose && a.nativeTarget==b.nativeTarget && a.nativeVelocityPps==b.nativeVelocityPps &&
        a.maximumVelocityPps==b.maximumVelocityPps && a.maximumAccelerationPps2==b.maximumAccelerationPps2 &&
        a.maximumTravelUnits==b.maximumTravelUnits;
}
inline bool ValidEvidence(const Config& c,const Evidence& e) noexcept
{
    if(!Valid(c) || e.session!=c.session || !EDM81::SameScope(e.scope,c.scope) ||
        e.mapGeneration!=c.machine.mapGeneration || e.captureTick!=c.machine.tick ||
        (e.state!=State::Running && e.state!=State::Completed && e.state!=State::Cancelled) || e.rejectedAxis!=6U ||
        e.lastSequence!=e.processedFrames || (e.axisMovedMask&~0x3fU) ||
        ((e.state==State::Running || e.state==State::Completed) && e.reason!=Reason::None) ||
        (e.state==State::Completed && (e.candidateValid || !e.processedFrames)) ||
        (e.state==State::Cancelled && (e.reason!=Reason::Cancelled || e.candidateValid)) ||
        (e.processedFrames && (e.processedFrames-1U>(std::numeric_limits<std::uint64_t>::max)()-e.captureTick ||
            e.lastTick!=e.captureTick+e.processedFrames-1U)) ||
        (!e.processedFrames && (e.lastTick || e.candidateValid || e.axisMovedMask))) return false;
    for(unsigned i=0U;i<6U;++i)
    {
        const auto& a=c.machine.axes[i];
        const double delta=e.lastPose[i]-c.initialPose[i];
        const double target=a.anchorNativePulse+delta*a.pulsesPerUnit;
        const double cap=EDM81::Minimum(a.maxVelocityUnitsPerMin/60.0,MaximumVelocityUnitsPerSecond(i))*a.pulsesPerUnit;
        if(!EDM81::Finite(e.lastPose[i]) || !EDM81::Finite(delta) || !EDM81::Finite(target) ||
            !EDM81::Finite(e.nativeTarget[i]) || target!=e.nativeTarget[i] ||
            !EDM81::Finite(e.nativeVelocityPps[i]) || !EDM81::Finite(e.maximumVelocityPps[i]) ||
            !EDM81::Finite(e.maximumAccelerationPps2[i]) || !EDM81::Finite(e.maximumTravelUnits[i]) ||
            e.maximumVelocityPps[i]<std::fabs(e.nativeVelocityPps[i]) ||
            e.maximumVelocityPps[i]>cap*(1.0+1e-12) || e.maximumAccelerationPps2[i]<0.0 ||
            e.maximumTravelUnits[i]<std::fabs(delta) || e.maximumTravelUnits[i]>MaximumTravelUnits(i) ||
            (a.hasMinLimit && target<a.minNativePulse) || (a.hasMaxLimit && target>a.maxNativePulse)) return false;
    }
    return true;
}
class Preview
{
public:
    const Evidence& Current() const noexcept { return evidence_; }
    bool Begin(const Config& config) noexcept
    {
        if(evidence_.state==State::Running) return Fail(Reason::Lifecycle);
        if(!Valid(config)) return Fail(Reason::Configuration);
        if(config.session<=latestSession_) return Fail(Reason::Identity);
        latestSession_=config.session; config_=config;evidence_=Evidence{};
        evidence_.session=config.session;evidence_.scope=config.scope;
        evidence_.mapGeneration=config.machine.mapGeneration;evidence_.captureTick=config.machine.tick;
        evidence_.state=State::Running;evidence_.lastPose=config.initialPose;
        for(unsigned i=0U;i<6U;++i) evidence_.nativeTarget[i]=config.machine.axes[i].anchorNativePulse;
        return true;
    }
    bool Step(const Frame& frame) noexcept
    {
        if(evidence_.state!=State::Running) return false;
        evidence_.candidateValid=false;
        if(frame.session!=config_.session || !EDM81::SameScope(frame.scope,config_.scope)) return Fail(Reason::Identity);
        if(evidence_.processedFrames==(std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::CounterOverflow);
        if(frame.sequence!=evidence_.processedFrames+1U) return Fail(Reason::Sequence);
        if(!frame.tick || (!evidence_.processedFrames && frame.tick!=config_.machine.tick) ||
            (evidence_.processedFrames && (evidence_.lastTick==(std::numeric_limits<std::uint64_t>::max)() ||
             frame.tick!=evidence_.lastTick+1U))) return Fail(Reason::Clock);
        // Temporary scalar arrays are not externally visible. No partial-axis
        // evidence is committed when any later axis fails validation.
        EDM81::Pose target{},velocity{},acceleration{},travel{};
        const double seconds=static_cast<double>(config_.periodUs)/1000000.0;
        for(unsigned i=0U;i<6U;++i)
        {
            const auto& a=config_.machine.axes[i];
            const double delta=frame.pose[i]-config_.initialPose[i];
            travel[i]=std::fabs(delta);
            target[i]=a.anchorNativePulse+delta*a.pulsesPerUnit;
            velocity[i]=(frame.pose[i]-evidence_.lastPose[i])*a.pulsesPerUnit/seconds;
            acceleration[i]=std::fabs(velocity[i]-evidence_.nativeVelocityPps[i])/seconds;
            if(!EDM81::Finite(frame.pose[i]) || !EDM81::Finite(delta) || !EDM81::Finite(target[i]) ||
                !EDM81::Finite(velocity[i]) || !EDM81::Finite(acceleration[i])) return Fail(Reason::NonFinite,i);
            if(travel[i]>MaximumTravelUnits(i)) return Fail(Reason::Envelope,i);
            if((a.hasMinLimit && target[i]<a.minNativePulse) || (a.hasMaxLimit && target[i]>a.maxNativePulse))
                return Fail(Reason::NativeLimit,i);
            const double cap=EDM81::Minimum(a.maxVelocityUnitsPerMin/60.0,MaximumVelocityUnitsPerSecond(i))*a.pulsesPerUnit;
            if(std::fabs(velocity[i])>cap*(1.0+1e-12)) return Fail(Reason::Velocity,i);
        }
        for(unsigned i=0U;i<6U;++i)
        {
            if(frame.pose[i]!=evidence_.lastPose[i]) evidence_.axisMovedMask|=1U<<i;
            evidence_.maximumVelocityPps[i]=EDM81::Maximum(evidence_.maximumVelocityPps[i],std::fabs(velocity[i]));
            evidence_.maximumAccelerationPps2[i]=EDM81::Maximum(evidence_.maximumAccelerationPps2[i],acceleration[i]);
            evidence_.maximumTravelUnits[i]=EDM81::Maximum(evidence_.maximumTravelUnits[i],travel[i]);
        }
        evidence_.lastPose=frame.pose;evidence_.nativeTarget=target;evidence_.nativeVelocityPps=velocity;
        evidence_.lastTick=frame.tick;evidence_.lastSequence=frame.sequence;++evidence_.processedFrames;
        evidence_.candidateValid=true;return true;
    }
    void Finish() noexcept
    {
        if(evidence_.state!=State::Running) return;
        evidence_.candidateValid=false;evidence_.state=State::Completed;
    }
    void Cancel() noexcept
    {
        if(evidence_.state==State::Idle) return;
        evidence_.candidateValid=false;
        if(evidence_.state==State::Fault) return;
        evidence_.state=State::Cancelled;evidence_.reason=Reason::Cancelled;
    }
    void Invalidate(Reason reason=Reason::Machine) noexcept { (void)Fail(reason); }
private:
    Config config_{};Evidence evidence_{};std::uint64_t latestSession_=0U;
    bool Fail(Reason reason,unsigned axis=6U) noexcept
    { evidence_.state=State::Fault;evidence_.reason=reason;evidence_.rejectedAxis=axis;evidence_.candidateValid=false;return false; }
};
}
