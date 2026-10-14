#pragma once
#include "EDMMultiAxisCommandBatch.h"
#include "EDMServoPI.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

// EDM93: private, ideal velocity plant. No AxisContext, writer, PDO pointer,
// physical authority or measured motor position is accepted by this model.
namespace EDM93
{
constexpr unsigned SettleSamples=64U;
constexpr std::uint64_t MaximumSettleTicks=16000U;
constexpr double SettleTolerancePulse=1.0;
struct AxisController
{
    EDM93PI::Gains gains{};
    double maxLag=0.0;
    bool enableLagCheck=false,motorEncoder=false;
};
struct ControlSnapshot
{
    std::uint64_t tick=0U;
    bool valid=false,uniqueMapping=false;
    std::array<AxisController,6U> axes{};
};
enum class State : std::uint8_t { Idle,Running,Settling,Completed,Stopped,Aborted,Retired };
enum class Reason : std::uint8_t
{ None,Configuration,Identity,Sequence,Clock,NonFinite,Lag,Envelope,SettleTimeout,Lifecycle,External };
inline const char* StateName(State s) noexcept
{
    switch(s) { case State::Idle:return "IDLE";case State::Running:return "RUNNING";
    case State::Settling:return "SETTLING";case State::Completed:return "COMPLETED";
    case State::Stopped:return "STOPPED";case State::Aborted:return "ABORTED";
    case State::Retired:return "RETIRED"; }return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) {case Reason::None:return "NONE";case Reason::Configuration:return "CONFIGURATION";
    case Reason::Identity:return "IDENTITY";case Reason::Sequence:return "SEQUENCE";
    case Reason::Clock:return "CLOCK";case Reason::NonFinite:return "NONFINITE";
    case Reason::Lag:return "LAG";case Reason::Envelope:return "ENVELOPE";
    case Reason::SettleTimeout:return "SETTLE_TIMEOUT";case Reason::Lifecycle:return "LIFECYCLE";
    case Reason::External:return "EXTERNAL";}return "UNKNOWN";
}
inline bool ValidControllerFrame(const ControlSnapshot& c,std::uint64_t tick) noexcept
{
    if(!c.valid || !c.uniqueMapping || !tick || c.tick!=tick) return false;
    for(const auto& a:c.axes)
        if(!a.motorEncoder || !EDM93PI::ValidGains(a.gains) || a.gains.kp<=0.0 ||
            !EDM81::Finite(a.maxLag) || a.maxLag<0.0 || (a.enableLagCheck && a.maxLag<=0.0)) return false;
    return true;
}
inline bool SameControllerBinding(const ControlSnapshot& a,const ControlSnapshot& b) noexcept
{
    if(a.valid!=b.valid || a.uniqueMapping!=b.uniqueMapping) return false;
    for(unsigned i=0U;i<6U;++i) {
        const auto& x=a.axes[i];const auto& y=b.axes[i];
        if(x.gains.kp!=y.gains.kp || x.gains.ki!=y.gains.ki || x.gains.kd!=y.gains.kd ||
            x.gains.kvff!=y.gains.kvff || x.gains.maxIntegral!=y.gains.maxIntegral ||
            x.maxLag!=y.maxLag || x.enableLagCheck!=y.enableLagCheck || x.motorEncoder!=y.motorEncoder) return false;
    }return true;
}
struct Evidence
{
    std::uint64_t session=0U,captureTick=0U,lastTick=0U,acceptedBatches=0U,modelTicks=0U,settleTicks=0U;
    std::uint64_t controllerFingerprint=0U,firstTick=0U;
    EDM81::Pose positionNativePulse{},integralAcc{},maximumFollowingErrorPulse{},maximumLogicalVelocityPps{};
    std::array<std::int32_t,6U> csv{};
    unsigned settledSamples=0U,saturationMask=0U,everSaturationMask=0U,rejectedAxis=6U;
    State state=State::Idle;Reason reason=Reason::None;
    bool modelSettled=false,revoked=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
static_assert(sizeof(Evidence)<=304U,"EDM93 evidence exceeds feedback budget");
inline bool IsEmpty(const Evidence& e) noexcept
{
    return !e.session && !e.captureTick && !e.lastTick && !e.acceptedBatches && !e.modelTicks &&
        !e.settleTicks && !e.controllerFingerprint && !e.firstTick && e.positionNativePulse==EDM81::Pose{} &&
        e.integralAcc==EDM81::Pose{} && e.maximumFollowingErrorPulse==EDM81::Pose{} &&
        e.maximumLogicalVelocityPps==EDM81::Pose{} && e.csv==std::array<std::int32_t,6U>{} &&
        !e.settledSamples && !e.saturationMask && !e.everSaturationMask && e.rejectedAxis==6U &&
        e.state==State::Idle && e.reason==Reason::None && !e.modelSettled && !e.revoked;
}
inline bool SameTerminalEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return a.session==b.session && a.captureTick==b.captureTick && a.lastTick==b.lastTick &&
        a.acceptedBatches==b.acceptedBatches && a.modelTicks==b.modelTicks && a.settleTicks==b.settleTicks &&
        a.controllerFingerprint==b.controllerFingerprint && a.firstTick==b.firstTick && a.positionNativePulse==b.positionNativePulse &&
        a.integralAcc==b.integralAcc && a.maximumFollowingErrorPulse==b.maximumFollowingErrorPulse &&
        a.maximumLogicalVelocityPps==b.maximumLogicalVelocityPps && a.csv==b.csv &&
        a.settledSamples==b.settledSamples && a.saturationMask==b.saturationMask &&
        a.everSaturationMask==b.everSaturationMask && a.rejectedAxis==b.rejectedAxis &&
        a.reason==b.reason && a.modelSettled==b.modelSettled;
}
inline bool ValidConfiguration(const EDM90::Config& c,const ControlSnapshot& controls) noexcept;
inline bool ValidEvidence(const EDM90::Config& c,const Evidence& e) noexcept;
inline bool BoundTo(const EDM90::Config& c,const Evidence& e,const EDM92::Evidence& b) noexcept;
class Simulation
{
public:
    const Evidence& Current() const noexcept { return evidence_; }
    bool Begin(const EDM90::Config& c,const ControlSnapshot& controls) noexcept;
    bool Accept(const EDM92::Evidence& batch,std::uint64_t tick) noexcept;
    bool Settle(std::uint64_t tick) noexcept;
    bool NeedsSettle() const noexcept { return evidence_.state==State::Settling; }
    bool ControlsCurrent(const ControlSnapshot& controls) const noexcept
    { return ValidControllerFrame(controls,controls.tick) && SameControllerBinding(controls_,controls); }
    void Revoke() noexcept;
    void Abort(Reason reason=Reason::External) noexcept
    { if(evidence_.state!=State::Idle && evidence_.state!=State::Aborted && evidence_.state!=State::Retired)
        (void)Fail(reason==Reason::None?Reason::External:reason); }
    bool Retire() noexcept;
private:
    EDM90::Config config_{};ControlSnapshot controls_{};Evidence evidence_{};
    EDM81::Pose target_{},feedforward_{};
    std::uint64_t latestSession_=0U;
    bool Step(const EDM81::Pose& target,const EDM81::Pose& velocity,std::uint64_t tick,bool settle) noexcept;
    bool Fail(Reason reason,unsigned axis=6U) noexcept
    { evidence_.state=State::Aborted;evidence_.reason=reason;evidence_.rejectedAxis=axis;evidence_.modelSettled=false;return false; }
};
}

namespace EDM93
{
inline double VelocityCap(const EDM90::Config& c,unsigned i) noexcept
{
    return std::floor(EDM91::VelocityLimitUnitsPerSecond(c,i)*c.machine.axes[i].pulsesPerUnit);
}
inline bool ValidConfiguration(const EDM90::Config& c,const ControlSnapshot& controls) noexcept
{
    if(!EDM91::Valid(c) || !ValidControllerFrame(controls,c.machine.tick)) return false;
    for(unsigned i=0U;i<6U;++i) {
        const double cap=VelocityCap(c,i);
        if(!EDM81::Finite(cap) || cap<1.0 || cap>2147483647.0) return false;
    }return true;
}
inline bool ValidEvidence(const EDM90::Config& c,const Evidence& e) noexcept
{
    const auto max=(std::numeric_limits<std::uint64_t>::max)();
    if(!EDM91::Valid(c) || e.session!=c.session || e.captureTick!=c.machine.tick ||
        !e.controllerFingerprint || static_cast<unsigned>(e.state)>static_cast<unsigned>(State::Retired) ||
        static_cast<unsigned>(e.reason)>static_cast<unsigned>(Reason::External) || e.state==State::Idle ||
        e.acceptedBatches>max-e.settleTicks || e.modelTicks!=e.acceptedBatches+e.settleTicks ||
        (e.modelTicks?(!e.firstTick || e.firstTick<e.captureTick || e.firstTick-e.captureTick>1U ||
            (e.firstTick!=e.captureTick && (e.acceptedBatches || !e.revoked)) ||
            e.modelTicks-1U>max-e.firstTick || e.lastTick!=e.firstTick+e.modelTicks-1U):
            (e.lastTick!=0U || e.firstTick!=0U)) ||
        e.settleTicks>MaximumSettleTicks || e.settledSamples>SettleSamples || e.settledSamples>e.settleTicks ||
        (e.saturationMask&~0x3fU) || (e.everSaturationMask&~0x3fU) ||
        (e.saturationMask&~e.everSaturationMask) || e.rejectedAxis>6U ||
        (e.reason==Reason::None && e.rejectedAxis!=6U) ||
        ((e.reason!=Reason::None)!=(e.state==State::Aborted || (e.state==State::Retired && !e.modelSettled))) ||
        (e.modelSettled!=(e.state==State::Completed || e.state==State::Stopped ||
            (e.state==State::Retired && e.reason==Reason::None))) ||
        (e.modelSettled && (e.settledSamples!=SettleSamples || e.settleTicks<SettleSamples)) ||
        (!e.modelSettled && e.reason==Reason::None && e.settledSamples==SettleSamples) ||
        (e.state==State::Completed && e.revoked) || (e.state==State::Stopped && !e.revoked) ||
        (e.state==State::Running && (e.settleTicks || e.settledSamples)) ||
        (!e.modelTicks && (e.saturationMask || e.everSaturationMask || e.settledSamples))) return false;
    for(unsigned i=0U;i<6U;++i) {
        const auto& a=c.machine.axes[i];const double cap=VelocityCap(c,i);
        if(!EDM81::Finite(cap) || cap<1.0 || cap>2147483647.0 ||
            !EDM81::Finite(e.positionNativePulse[i]) || !EDM81::Finite(e.integralAcc[i]) ||
            !EDM81::Finite(e.maximumFollowingErrorPulse[i]) || e.maximumFollowingErrorPulse[i]<0.0 ||
            !EDM81::Finite(e.maximumLogicalVelocityPps[i]) || e.maximumLogicalVelocityPps[i]<0.0 ||
            e.maximumLogicalVelocityPps[i]>cap || std::fabs(static_cast<double>(e.csv[i]))>cap ||
            e.maximumLogicalVelocityPps[i]<std::fabs(static_cast<double>(e.csv[i])) ||
            std::fabs(e.positionNativePulse[i]-a.anchorNativePulse)>EDM90::MaximumTravelUnits(i)*a.pulsesPerUnit ||
            (a.hasMinLimit && e.positionNativePulse[i]<a.minNativePulse) ||
            (a.hasMaxLimit && e.positionNativePulse[i]>a.maxNativePulse) ||
            (e.modelSettled && e.csv[i]!=0) || (!e.modelTicks &&
             (e.positionNativePulse[i]!=a.anchorNativePulse || e.integralAcc[i]!=0.0 ||
              e.maximumFollowingErrorPulse[i]!=0.0 || e.maximumLogicalVelocityPps[i]!=0.0 || e.csv[i]!=0))) return false;
    }return true;
}
inline bool BoundTo(const EDM90::Config& c,const Evidence& e,const EDM92::Evidence& b) noexcept
{
    if(!ValidEvidence(c,e) || !EDM92::ValidEvidence(c,b) || e.revoked!=b.revoked) return false;
    if(e.acceptedBatches!=b.acceptedBatches) {
        return e.reason!=Reason::None && !e.modelSettled && !e.settleTicks &&
            e.acceptedBatches<(std::numeric_limits<std::uint64_t>::max)() &&
            b.acceptedBatches==e.acceptedBatches+1U &&
            (e.acceptedBatches?(e.lastTick<(std::numeric_limits<std::uint64_t>::max)() &&
                b.trajectory.lastTick==e.lastTick+1U):b.trajectory.lastTick==c.machine.tick);
    }
    if(e.acceptedBatches && (e.settleTicks>(std::numeric_limits<std::uint64_t>::max)()-b.trajectory.lastTick ||
        e.lastTick!=b.trajectory.lastTick+e.settleTicks)) return false;
    for(unsigned i=0U;i<6U;++i)
        if(e.maximumFollowingErrorPulse[i]<std::fabs(b.trajectory.nativeTarget[i]-e.positionNativePulse[i])) return false;
    if(e.modelSettled) {
        if(b.reason!=EDM92::Reason::None || (!b.numericalStopped && !b.revoked)) return false;
        for(unsigned i=0U;i<6U;++i)
            if(b.nativeVelocityPps[i]!=0.0 ||
                std::fabs(b.trajectory.nativeTarget[i]-e.positionNativePulse[i])>SettleTolerancePulse) return false;
    }
    if(e.settleTicks && !b.revoked && !b.numericalStopped) return false;
    if(e.reason==Reason::None && b.reason!=EDM92::Reason::None) return false;
    return true;
}
inline bool Simulation::Begin(const EDM90::Config& c,const ControlSnapshot& controls) noexcept
{
    if(evidence_.state!=State::Idle && evidence_.state!=State::Retired && evidence_.state!=State::Aborted)
        return Fail(Reason::Lifecycle);
    if(!ValidConfiguration(c,controls)) return Fail(Reason::Configuration);
    if(c.session<=latestSession_) return Fail(Reason::Identity);
    for(unsigned i=0U;i<6U;++i) {
        const double cap=VelocityCap(c,i);
        if(!EDM81::Finite(cap) || cap<1.0 || cap>2147483647.0) return Fail(Reason::Configuration,i);
    }
    config_=c;controls_=controls;latestSession_=c.session;evidence_=Evidence{};
    evidence_.session=c.session;evidence_.captureTick=c.machine.tick;evidence_.state=State::Running;
    std::uint64_t fingerprint=1469598103934665603ULL;
    const auto add=[&fingerprint](std::uint64_t word) noexcept
    { for(unsigned j=0U;j<8U;++j) {fingerprint^=(word>>(j*8U))&0xffU;fingerprint*=1099511628211ULL;} };
    const auto addDouble=[&add](double x) noexcept {std::uint64_t word=0U;std::memcpy(&word,&x,sizeof(word));add(word);};
    for(unsigned i=0U;i<6U;++i) {
        evidence_.positionNativePulse[i]=target_[i]=c.machine.axes[i].anchorNativePulse;
        feedforward_[i]=0.0;const auto& a=controls.axes[i];
        addDouble(a.gains.kp);addDouble(a.gains.ki);addDouble(a.gains.kd);addDouble(a.gains.kvff);
        addDouble(a.gains.maxIntegral);addDouble(a.maxLag);add(a.enableLagCheck?1U:0U);add(a.motorEncoder?1U:0U);
    }
    evidence_.controllerFingerprint=fingerprint?fingerprint:1U;return true;
}
inline bool Simulation::Step(const EDM81::Pose& target,const EDM81::Pose& velocity,
    std::uint64_t tick,bool settle) noexcept
{
    const auto max=(std::numeric_limits<std::uint64_t>::max)();
    if(!tick || (evidence_.modelTicks ? (evidence_.lastTick==max || tick!=evidence_.lastTick+1U):
        (tick<config_.machine.tick || tick-config_.machine.tick>1U ||
         (tick!=config_.machine.tick && (!settle || !evidence_.revoked)))))
        return Fail(Reason::Clock);
    if(evidence_.modelTicks==max || (settle && evidence_.settleTicks>=MaximumSettleTicks)) return Fail(Reason::SettleTimeout);
    Evidence next=evidence_;next.saturationMask=0U;bool quiet=true;
    for(unsigned i=0U;i<6U;++i) {
        const auto& a=config_.machine.axes[i];const auto& control=controls_.axes[i];
        const double cap=VelocityCap(config_,i);
        // The prior signed integer image drives this interval. The inverse
        // hardware sign is applied exactly once at the ideal plant boundary.
        const double plantVelocity=static_cast<double>(evidence_.csv[i])*static_cast<double>(a.hardwareSign);
        const double position=evidence_.positionNativePulse[i]+plantVelocity*EDM91::Seconds;
        const double error=target[i]-position;
        if(!EDM81::Finite(target[i]) || !EDM81::Finite(velocity[i]) || !EDM81::Finite(position) || !EDM81::Finite(error))
            return Fail(Reason::NonFinite,i);
        if(std::fabs(position-a.anchorNativePulse)>EDM90::MaximumTravelUnits(i)*a.pulsesPerUnit ||
            (a.hasMinLimit && position<a.minNativePulse) || (a.hasMaxLimit && position>a.maxNativePulse))
            return Fail(Reason::Envelope,i);
        if(control.enableLagCheck && std::fabs(error)>control.maxLag) return Fail(Reason::Lag,i);
        const auto result=EDM93PI::Evaluate(control.gains,evidence_.integralAcc[i],error,velocity[i],EDM91::Seconds);
        if(!EDM81::Finite(result.integralAcc) || !EDM81::Finite(result.pTerm) ||
            !EDM81::Finite(result.iTerm) || !EDM81::Finite(result.velocityPps)) return Fail(Reason::NonFinite,i);
        double logical=result.velocityPps;
        if(logical>cap) logical=cap;
        if(logical< -cap) logical= -cap;
        const bool limited=logical!=result.velocityPps;
        if(limited) next.saturationMask|=1U<<i;
        next.integralAcc[i]=limited?evidence_.integralAcc[i]:result.integralAcc;
        const double hardware=logical*static_cast<double>(a.hardwareSign);
        if(!EDM81::Finite(hardware) || hardware< -2147483647.0 || hardware>2147483647.0)
            return Fail(Reason::NonFinite,i);
        next.csv[i]=static_cast<std::int32_t>(hardware);
        next.positionNativePulse[i]=position;
        next.maximumFollowingErrorPulse[i]=(std::max)(next.maximumFollowingErrorPulse[i],std::fabs(error));
        next.maximumLogicalVelocityPps[i]=(std::max)(next.maximumLogicalVelocityPps[i],std::fabs(logical));
        quiet=quiet && next.csv[i]==0 && std::fabs(error)<=SettleTolerancePulse;
    }
    if(!next.modelTicks) next.firstTick=tick;
    ++next.modelTicks;next.lastTick=tick;next.everSaturationMask|=next.saturationMask;
    if(settle) {
        ++next.settleTicks;next.settledSamples=quiet?next.settledSamples+1U:0U;
        if(next.settledSamples==SettleSamples) {
            next.modelSettled=true;next.state=next.revoked?State::Stopped:State::Completed;
        }
    } else ++next.acceptedBatches;
    // No per-axis integral, position, image or counter is committed before all
    // six axes have passed. A rejected final axis preserves the whole history.
    evidence_=next;
    if(settle && evidence_.settleTicks==MaximumSettleTicks && !evidence_.modelSettled)
        return Fail(Reason::SettleTimeout);
    return true;
}
inline bool Simulation::Accept(const EDM92::Evidence& batch,std::uint64_t tick) noexcept
{
    if(evidence_.state!=State::Running) return Fail(Reason::Lifecycle);
    if(!EDM92::ValidEvidence(config_,batch) || batch.reason!=EDM92::Reason::None ||
        batch.trajectory.lastTick!=tick || batch.revoked!=evidence_.revoked) return Fail(Reason::Identity);
    if(evidence_.acceptedBatches==(std::numeric_limits<std::uint64_t>::max)() ||
        batch.acceptedBatches!=evidence_.acceptedBatches+1U) return Fail(Reason::Sequence);
    if(!Step(batch.trajectory.nativeTarget,batch.nativeVelocityPps,tick,false)) return false;
    target_=batch.trajectory.nativeTarget;feedforward_=batch.nativeVelocityPps;
    if(batch.numericalStopped) evidence_.state=State::Settling;
    return true;
}
inline bool Simulation::Settle(std::uint64_t tick) noexcept
{
    if(evidence_.state!=State::Settling) return Fail(Reason::Lifecycle);
    return Step(target_,EDM81::Pose{},tick,true);
}
inline void Simulation::Revoke() noexcept
{
    if(evidence_.state==State::Idle || evidence_.state==State::Retired || evidence_.revoked) return;
    evidence_.revoked=true;
    if(evidence_.state==State::Completed) evidence_.state=State::Stopped;
    else if(evidence_.state==State::Running && feedforward_==EDM81::Pose{}) evidence_.state=State::Settling;
}
inline bool Simulation::Retire() noexcept
{
    if(evidence_.state==State::Idle || evidence_.state==State::Retired) return true;
    if(evidence_.state!=State::Completed && evidence_.state!=State::Stopped && evidence_.state!=State::Aborted)
        return Fail(Reason::Lifecycle);
    evidence_.state=State::Retired;return true;
}
}
