#pragma once

#include "NCG76SphereCycle.h"
#include "NCG76SphereDeepFlush.h"
#include "EDM130WorkTimer.h"
#include "EDMCoordinatedGapSource.h"
#include "EDMGapServo.h"
#include "EDMFlushPlan.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM293: live RT, simulation-only process observation. This observer never
// commands motion, changes a PID bank, writes PLC outputs or permits discharge.
// Its E5 counters are SIMULATED eligible work, never actual discharge time.
// A due request is a candidate checkpoint, not a stopped anchor or completion.
// EDM294: the independent AIR cycle may consume a scoped request, prove its
// own stopped anchor and return, then acknowledge that exact request on RT.
// This observer only rearms the simulated E5 interval after that evidence.
namespace NCG76SphereGapProcess
{
using Point8=NCG76SphereCycle::Point8;
constexpr bool MotionExecutionReleased=false,DischargeExecutionReleased=false;
constexpr std::uint64_t SamplePeriodUs=1000ULL,MaximumSourceAgeUs=2500ULL;
constexpr std::uint64_t HighIntervalUs=200000ULL,LowIntervalUs=100000ULL;

struct Config
{
    EDMGapServo::Profile curve{};
    std::uint64_t workTimeUs=0ULL,recipeGeneration=0ULL,tuningGeneration=0ULL;
    double jumpHeightMm=0.0,jumpSpeedPercent=0.0,machiningOverridePercent=0.0,referenceV=0.0;
    bool enabled=false,flushMotionEnabled=false,signedMotionEnabled=false;
    // EDM302: native pump bases are frozen per parent. E11/E13 are optional
    // confirmed percent/mm values, never an interpretation of old raw data.
    EDM20::FlushProfile flushProfile{};
    double finalApproachPercent=0.0,finalApproachDistanceMm=0.0;
    bool independentFeedsEnabled=false,finalApproachEnabled=false;
    // EDM305: optional outbound slow distance, in native physical mm. The
    // percent scales the frozen initial pump base; E7 remains a separate factor.
    double initialSlowPercent=0.0,initialSlowDistanceMm=0.0;
    bool initialSlowEnabled=false;
    // EDM306: every interval-th successful flush uses this exact native
    // fixed-point E6 * E22 height. Inactive optional fields stay canonical zero.
    std::uint32_t deepFlushCycleInterval=0U;
    double deepFlushHeightMultiplier=0.0,deepFlushHeightMm=0.0;
    bool deepFlushEnabled=false;
    // EDM308: explicit Line/Arc parameter-feed contract; the matching Request
    // is independently checked by Motion. False preserves the P50/F contract.
    bool parameterFeeds=false;
};
inline bool Disabled(const Config& c) noexcept{return !c.enabled;}
namespace ParameterFeedDetail
{
inline bool ProfileCeiling(const Config& c,double& ceiling) noexcept
{
    ceiling=0.0;
    if(!EDMGapServo::ValidateProfile(c.curve)||!EDM20::ValidateFlushProfile(c.flushProfile))return false;
    const double speeds[]={c.curve.maxFeedMmPerMin,c.curve.maxRetreatMmPerMin,
        c.flushProfile.finalApproachMmPerMin,c.flushProfile.centerRetractMmPerMin,c.flushProfile.mainRetractMmPerMin,
        c.flushProfile.centerReturnMmPerMin,c.flushProfile.mainReturnMmPerMin,c.flushProfile.pathRetractMmPerMin,
        c.flushProfile.initialSlowMmPerMin,c.flushProfile.manualReturnMmPerMin};
    double maximum=0.0;
    for(double speed:speeds){
        if(!std::isfinite(speed)||speed<=0.0||speed>100000.0)return false;
        maximum=(std::max)(maximum,speed);
    }
    if(!std::isfinite(maximum)||maximum<=0.0)return false;
    ceiling=maximum;return true;
}
}
inline bool Valid(const Config& c) noexcept
{
    if(c.parameterFeeds){
        double ceiling=0.0;
        if(!c.enabled||!c.signedMotionEnabled||
           (c.flushMotionEnabled&&!c.independentFeedsEnabled)||
           !ParameterFeedDetail::ProfileCeiling(c,ceiling)||
           !std::isfinite(c.jumpHeightMm)||c.jumpHeightMm<0.0||c.jumpHeightMm>10.0)return false;
    }
    if(c.flushMotionEnabled&&(!c.enabled||!(c.jumpHeightMm>0.0)))return false;
    if(c.signedMotionEnabled&&!c.enabled)return false;
    if(c.independentFeedsEnabled&&(!c.enabled||!c.flushMotionEnabled||!c.signedMotionEnabled||
        !EDM20::ValidateFlushProfile(c.flushProfile)))return false;
    if(c.finalApproachEnabled){
        if(!c.independentFeedsEnabled||!std::isfinite(c.finalApproachPercent)||
            c.finalApproachPercent<10.0||c.finalApproachPercent>100.0||
            !std::isfinite(c.finalApproachDistanceMm)||c.finalApproachDistanceMm<=0.0||
            c.finalApproachDistanceMm>100.0)return false;
    }else if(c.finalApproachPercent!=0.0||c.finalApproachDistanceMm!=0.0)return false;
    if(c.initialSlowEnabled){
        if(!c.independentFeedsEnabled||!std::isfinite(c.initialSlowPercent)||
            c.initialSlowPercent<1.0||c.initialSlowPercent>100.0||
            !std::isfinite(c.initialSlowDistanceMm)||c.initialSlowDistanceMm<=0.0||
            c.initialSlowDistanceMm>900.0)return false;
    }else if(c.initialSlowPercent!=0.0||c.initialSlowDistanceMm!=0.0)return false;
    if(c.deepFlushEnabled){
        double expectedHeight=0.0;
        if(!c.independentFeedsEnabled||c.deepFlushCycleInterval==0U||c.deepFlushCycleInterval>20U||
            !NCG76SphereDeepFlush::ResolveHeight(c.jumpHeightMm,c.deepFlushHeightMultiplier,expectedHeight,c.parameterFeeds?10.0:2.0)||
            c.deepFlushHeightMm!=expectedHeight)return false;
    }else if(c.deepFlushCycleInterval!=0U||c.deepFlushHeightMultiplier!=0.0||c.deepFlushHeightMm!=0.0)return false;
    if(Disabled(c))return true;
    return EDMGapServo::ValidateProfile(c.curve)&&EDM130::ValidWorkTime(c.workTimeUs)&&
        c.recipeGeneration&&c.tuningGeneration&&std::isfinite(c.jumpHeightMm)&&c.jumpHeightMm>=0.0&&c.jumpHeightMm<=1000.0&&
        std::isfinite(c.jumpSpeedPercent)&&c.jumpSpeedPercent>0.0&&c.jumpSpeedPercent<=100.0&&
        std::isfinite(c.machiningOverridePercent)&&c.machiningOverridePercent>=0.0&&c.machiningOverridePercent<=100.0&&
        std::isfinite(c.referenceV)&&c.referenceV>=15.0&&c.referenceV<=200.0;
}
// A trajectory/admission ceiling, not a fixed machining feed. Signed motion
// still uses the E8/E9 GAP curve; each flush phase uses its own native pump.
inline bool ResolveParameterMotionCeiling(const Config& c,double& ceiling) noexcept
{
    ceiling=0.0;
    return c.parameterFeeds&&Valid(c)&&ParameterFeedDetail::ProfileCeiling(c,ceiling);
}
// EDM307: a completed, independently scoped path parent may pass only its
// accounting totals to a fresh observer. This value contains no pending
// request, source sample, clock, completion acknowledgement or motion permit.
// The RT adapter proves the previous completed parent and frozen recipe scope;
// accepting these numbers alone never authorizes a cross-parent handoff.
struct Continuation
{
    std::uint64_t intervalWorkUs=0ULL,simulatedWorkUs=0ULL;
    std::uint64_t completedFlushes=0ULL,abortedFlushes=0ULL;
};
inline bool ValidContinuation(const Config& config,const Continuation& value) noexcept
{
    const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
    return config.enabled&&Valid(config)&&EDM130::ValidWorkTime(config.workTimeUs)&&
        value.intervalWorkUs<=config.workTimeUs&&value.simulatedWorkUs>=value.intervalWorkUs&&
        value.completedFlushes<maximum&&value.abortedFlushes<maximum&&
        value.completedFlushes<=maximum-value.abortedFlushes;
}
namespace ContinuationDetail
{
// Compare each value rather than struct storage: padding is not recipe
// identity. Preserve signed zero, and never make two NaNs continuation-equal.
inline bool SameDouble(double a,double b) noexcept
{return a==b&&std::signbit(a)==std::signbit(b);}
inline bool SameCurve(const EDMGapServo::Profile& a,const EDMGapServo::Profile& b) noexcept
{
    if(a.revision!=b.revision||a.limitsConfirmed!=b.limitsConfirmed||
       a.shortIdleEnabled!=b.shortIdleEnabled||a.shortMachiningEnabled!=b.shortMachiningEnabled||
       a.shortEnterMs!=b.shortEnterMs||a.shortExitMs!=b.shortExitMs)return false;
    for(unsigned i=0U;i<3U;++i)
        if(!SameDouble(a.positiveGain[i],b.positiveGain[i])||!SameDouble(a.negativeGain[i],b.negativeGain[i]))return false;
    for(unsigned i=0U;i<2U;++i)
        if(!SameDouble(a.positiveBreakV[i],b.positiveBreakV[i])||!SameDouble(a.negativeBreakV[i],b.negativeBreakV[i]))return false;
    return SameDouble(a.cuttingScaleMmPerVoltMin,b.cuttingScaleMmPerVoltMin)&&
        SameDouble(a.maxFeedMmPerMin,b.maxFeedMmPerMin)&&SameDouble(a.maxRetreatMmPerMin,b.maxRetreatMmPerMin)&&
        SameDouble(a.deadbandV,b.deadbandV)&&SameDouble(a.shortIdleV,b.shortIdleV)&&
        SameDouble(a.shortMachiningV,b.shortMachiningV)&&SameDouble(a.shortHysteresisV,b.shortHysteresisV)&&
        SameDouble(a.shortRetreatMmPerMin,b.shortRetreatMmPerMin);
}
inline bool SameFlush(const EDM20::FlushProfile& a,const EDM20::FlushProfile& b) noexcept
{
    return SameDouble(a.finalApproachMmPerMin,b.finalApproachMmPerMin)&&
        SameDouble(a.centerRetractMmPerMin,b.centerRetractMmPerMin)&&SameDouble(a.mainRetractMmPerMin,b.mainRetractMmPerMin)&&
        SameDouble(a.centerReturnMmPerMin,b.centerReturnMmPerMin)&&SameDouble(a.mainReturnMmPerMin,b.mainReturnMmPerMin)&&
        SameDouble(a.pathRetractMmPerMin,b.pathRetractMmPerMin)&&SameDouble(a.initialSlowMmPerMin,b.initialSlowMmPerMin)&&
        SameDouble(a.manualReturnMmPerMin,b.manualReturnMmPerMin)&&
        SameDouble(a.finalApproachDistanceMm,b.finalApproachDistanceMm)&&
        SameDouble(a.pathDistanceMm,b.pathDistanceMm)&&SameDouble(a.initialSlowDistanceMm,b.initialSlowDistanceMm);
}
}
inline bool EquivalentForContinuation(const Config& a,const Config& b) noexcept
{
    using ContinuationDetail::SameDouble;
    return a.enabled&&b.enabled&&Valid(a)&&Valid(b)&&
        a.workTimeUs==b.workTimeUs&&a.recipeGeneration==b.recipeGeneration&&a.tuningGeneration==b.tuningGeneration&&
        a.enabled==b.enabled&&a.flushMotionEnabled==b.flushMotionEnabled&&a.signedMotionEnabled==b.signedMotionEnabled&&
        a.parameterFeeds==b.parameterFeeds&&
        a.independentFeedsEnabled==b.independentFeedsEnabled&&a.finalApproachEnabled==b.finalApproachEnabled&&
        a.initialSlowEnabled==b.initialSlowEnabled&&a.deepFlushEnabled==b.deepFlushEnabled&&
        a.deepFlushCycleInterval==b.deepFlushCycleInterval&&
        SameDouble(a.jumpHeightMm,b.jumpHeightMm)&&SameDouble(a.jumpSpeedPercent,b.jumpSpeedPercent)&&
        SameDouble(a.machiningOverridePercent,b.machiningOverridePercent)&&SameDouble(a.referenceV,b.referenceV)&&
        SameDouble(a.finalApproachPercent,b.finalApproachPercent)&&SameDouble(a.finalApproachDistanceMm,b.finalApproachDistanceMm)&&
        SameDouble(a.initialSlowPercent,b.initialSlowPercent)&&SameDouble(a.initialSlowDistanceMm,b.initialSlowDistanceMm)&&
        SameDouble(a.deepFlushHeightMultiplier,b.deepFlushHeightMultiplier)&&SameDouble(a.deepFlushHeightMm,b.deepFlushHeightMm)&&
        ContinuationDetail::SameCurve(a.curve,b.curve)&&ContinuationDetail::SameFlush(a.flushProfile,b.flushProfile);
}
static_assert(sizeof(Continuation)==4U*sizeof(std::uint64_t)&&std::is_trivially_copyable<Continuation>::value,
    "Process continuation is bounded accounting data, never an execution receipt");
// Shared NC/RT derivation; whole-cycle travel admission remains the motion
// owner's independent proof. E6 is physical native mm, never authored scale.
inline bool BuildFlushConfig(const Config& config,double feedMmMin,
    NCG76SphereCycle::FlushConfig& output) noexcept
{
    output=NCG76SphereCycle::FlushConfig{};
    if(!Valid(config))return false;
    if(!config.enabled||!config.flushMotionEnabled)return true;
    if(!std::isfinite(feedMmMin)||feedMmMin<=0.0)return false;
    if(config.parameterFeeds){
        double ceiling=0.0;
        if(!ResolveParameterMotionCeiling(config,ceiling)||feedMmMin<ceiling||feedMmMin>100000.0)return false;
    }
    const double feed=(std::min)(feedMmMin,feedMmMin*(config.jumpSpeedPercent/100.0));
    if(!std::isfinite(feed)||feed<=0.0||feed/60.0<=0.0)return false;
    output.heightMm=config.jumpHeightMm;output.feedMmMin=feed;output.enabled=true;
    if(!config.independentFeedsEnabled)return true;
    const double ratio=config.jumpSpeedPercent/100.0;
    const auto cap=[&](double native,double overrideRatio,double& value) noexcept {
        const double scaled=native*overrideRatio;
        if(!std::isfinite(scaled)||scaled<=0.0)return false;
        value=(std::min)(feedMmMin,scaled);
        return std::isfinite(value)&&value>0.0&&value/60.0>0.0;
    };
    // Center speeds are independent settings, as in the established G76
    // center route. E7 is applied once to main/path/final pump speeds.
    if(!cap(config.flushProfile.centerRetractMmPerMin,1.0,output.centerRetreatFeedMmMin)||
       !cap(config.flushProfile.mainRetractMmPerMin,ratio,output.axisRetreatFeedMmMin)||
       !cap(config.flushProfile.centerReturnMmPerMin,1.0,output.centerReturnFeedMmMin)||
       !cap(config.flushProfile.mainReturnMmPerMin,ratio,output.axisReturnFeedMmMin)||
       !cap(config.flushProfile.pathRetractMmPerMin,ratio,output.pathRetreatFeedMmMin)){
        output=NCG76SphereCycle::FlushConfig{};return false;
    }
    output.independentFeedsEnabled=true;
    if(config.finalApproachEnabled){
        if(!cap(config.flushProfile.finalApproachMmPerMin,
            (config.finalApproachPercent/100.0)*ratio,output.finalApproachFeedMmMin)){
            output=NCG76SphereCycle::FlushConfig{};return false;
        }
        output.finalApproachDistanceMm=config.finalApproachDistanceMm;output.finalApproachEnabled=true;
    }
    if(config.initialSlowEnabled){
        if(!cap(config.flushProfile.initialSlowMmPerMin,
            (config.initialSlowPercent/100.0)*ratio,output.initialSlowFeedMmMin)){
            output=NCG76SphereCycle::FlushConfig{};return false;
        }
        output.initialSlowDistanceMm=config.initialSlowDistanceMm;output.initialSlowEnabled=true;
    }
    if(config.deepFlushEnabled){
        output.deepFlushCycleInterval=config.deepFlushCycleInterval;
        output.deepFlushHeightMm=config.deepFlushHeightMm;output.deepFlushEnabled=true;
    }
    return true;
}
enum class Reason : std::uint8_t
{ Disabled,None,Configuration,Lifecycle,Session,Clock,Source,Power,Input,Cancelled,Terminal };
// EDM310: selects a software voltage stimulus only. This enum neither lends
// geometry nor bypasses any Cycle/Motion admission, stop or travel guard.
enum class SimulationMode : std::uint8_t { Wave, PreviousParentProbe };
enum class ProbePhase : std::uint8_t { Disabled, Retreat, Return, Complete, Failed };
constexpr double PreviousParentProbeRetreatMm=0.002;
constexpr std::uint64_t MaximumPreviousParentProbeUs=30000000ULL;
inline const char* SimulationModeName(SimulationMode value) noexcept
{
    switch(value){case SimulationMode::Wave:return "WAVE";
    case SimulationMode::PreviousParentProbe:return "PREVIOUS_PARENT_PROBE";
    default:return "UNKNOWN";}
}
inline const char* ProbePhaseName(ProbePhase value) noexcept
{
    switch(value){case ProbePhase::Disabled:return "DISABLED";
    case ProbePhase::Retreat:return "RETREAT";case ProbePhase::Return:return "RETURN";
    case ProbePhase::Complete:return "COMPLETE";case ProbePhase::Failed:return "FAILED";
    default:return "UNKNOWN";}
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r){
    case Reason::Disabled:return "DISABLED";case Reason::None:return "NONE";
    case Reason::Configuration:return "CONFIGURATION";case Reason::Lifecycle:return "LIFECYCLE";
    case Reason::Session:return "SESSION";case Reason::Clock:return "CLOCK";
    case Reason::Source:return "SOURCE";case Reason::Power:return "POWER";
    case Reason::Input:return "INPUT";case Reason::Cancelled:return "CANCELLED";
    case Reason::Terminal:return "TERMINAL";default:return "UNKNOWN";}
}
struct Input
{
    NCG76SphereCycle::Feedback nominal{}; // Coherent PRE-step P50 feedback.
    Point8 candidateMCS{}; // Matching PRE-step nominal command; no stop proof.
    std::uint64_t session=0ULL,tick=0ULL,nowUs=0ULL,acceptedSendTick=0ULL;
    // Sticky, independently proved post-Step cycle acknowledgements. The
    // nominal/candidate above remain the coherent PRE-step source for E5.
    std::uint64_t completedFlushSequence=0ULL,abortedFlushSequence=0ULL;
    // EDM307: RT alone derives this from its frozen, canonically validated
    // zero-length Line. It does not turn malformed moving feedback into rest.
    bool stationaryNominal=false;
    bool sourceValid=false,powerInhibited=false,suspended=false,terminal=false,cancel=false;
};
struct Feedback
{
    Point8 candidateMCS{};
    std::uint64_t session=0ULL,tick=0ULL,recipeGeneration=0ULL,tuningGeneration=0ULL;
    std::uint64_t simulatedWorkUs=0ULL,intervalWorkUs=0ULL,workTimeUs=0ULL;
    std::uint64_t sourceSequence=0ULL,sourceCapturedAtUs=0ULL,requestSequence=0ULL,requestTick=0ULL;
    std::uint64_t completedFlushSequence=0ULL,abortedFlushSequence=0ULL;
    std::uint64_t completedFlushes=0ULL,abortedFlushes=0ULL;
    double voltageV=0.0,signedGapSpeedMmMin=0.0,nominalProgressMm=0.0,nominalLengthMm=0.0;
    NCG76SphereCycle::Phase nominalPhase=NCG76SphereCycle::Phase::Idle;
    unsigned layerIndex=0U;
    Reason reason=Reason::Disabled;
    EDM88::Reason sourceReason=EDM88::Reason::Disabled;
    SimulationMode simulationMode=SimulationMode::Wave;
    ProbePhase probePhase=ProbePhase::Disabled;
    bool enabled=false,valid=false,eligible=false,suspended=false,pending=false,terminal=false;
};

class Observer
{
public:
    bool Begin(const Config& config,std::uint64_t session,const Continuation* continuation=nullptr,
        SimulationMode mode=SimulationMode::Wave) noexcept
    {
        qualifier_.Cancel();active_=false;feedback_=Feedback{};work_=EDM130::WorkEvidence{};
        frame_=EDM88::Frame{};lastTick_=lastUs_=firstUs_=0ULL;haveClock_=false;
        requestHighWater_=workBaseUs_=0ULL;
        probeElapsedUs_=0ULL;probeLengthMm_=0.0;probeSeeded_=probeEntered_=false;
        feedback_.simulationMode=mode;
        if(Disabled(config)&&!config.flushMotionEnabled&&!config.signedMotionEnabled){
            if(!Valid(config)||continuation||mode!=SimulationMode::Wave){Withdraw(Reason::Configuration);return false;}
            return true;
        }
        feedback_.enabled=config.enabled;feedback_.session=session;feedback_.recipeGeneration=config.recipeGeneration;
        feedback_.tuningGeneration=config.tuningGeneration;feedback_.workTimeUs=config.workTimeUs;
        if(!session||session<=sessionHighWater_){Withdraw(Reason::Lifecycle);return false;}
        sessionHighWater_=session; // Even failed/withdrawn attempts cannot rearm this identity.
        if((mode!=SimulationMode::Wave&&mode!=SimulationMode::PreviousParentProbe)||
           (mode==SimulationMode::PreviousParentProbe&&
                (!continuation||!config.parameterFeeds||!config.enabled||!config.signedMotionEnabled)))
        {Withdraw(Reason::Configuration);return false;}
        if(!Valid(config)||(continuation&&!ValidContinuation(config,*continuation)))
        {Withdraw(Reason::Configuration);return false;}
        config_=config;
        if(mode==SimulationMode::PreviousParentProbe){
            const auto reverse=EDMGapServo::Evaluate(config.curve,config.referenceV-10.0,
                config.referenceV,config.machiningOverridePercent);
            const auto forward=EDMGapServo::Evaluate(config.curve,config.referenceV+10.0,
                config.referenceV,config.machiningOverridePercent);
            feedback_.probePhase=reverse.valid&&forward.valid&&reverse.limitedSpeedMmPerMin<0.0&&
                forward.limitedSpeedMmPerMin>0.0?ProbePhase::Retreat:ProbePhase::Failed;
        }
        auto source=EDM88::SimulatedSource(session);source.samplePeriodUs=SamplePeriodUs;
        // 1 ms conversion plus legal 50..1000 us observer cadence may expose
        // an old conversion up to <2 ms before the next conversion is captured.
        source.maxAgeUs=MaximumSourceAgeUs;
        if(!qualifier_.Begin(source)||!EDM130::BeginWork(work_,config.workTimeUs))
        {Withdraw(Reason::Configuration);return false;}
        if(continuation){
            // The interval may already be saturated when the previous path
            // ended before a legal flush checkpoint. The first eligible new
            // sample proposes a NEW request; no old request is acknowledged.
            work_.elapsedUs=work_.totalWorkUs=continuation->intervalWorkUs;
            work_.due=work_.elapsedUs==config.workTimeUs;work_.previousEligible=false;
            workBaseUs_=continuation->simulatedWorkUs-continuation->intervalWorkUs;
            feedback_.intervalWorkUs=continuation->intervalWorkUs;
            feedback_.simulatedWorkUs=continuation->simulatedWorkUs;
            feedback_.completedFlushes=continuation->completedFlushes;
            feedback_.abortedFlushes=continuation->abortedFlushes;
            if(!EDM130::ValidWorkEvidence(work_)){Withdraw(Reason::Configuration);return false;}
        }
        frame_.identity=source.identity;feedback_.reason=Reason::None;feedback_.sourceReason=EDM88::Reason::None;
        active_=true;return true;
    }
    const Feedback& Step(const Input& input) noexcept
    {
        if(!active_)return feedback_;
        if(input.cancel||input.terminal){
            // Publication identity only: the NC reader can associate this
            // withdrawn page with the terminal cycle. It earns no source
            // freshness, validity, work-time or completion evidence.
            if(input.tick)feedback_.tick=input.tick;
            Withdraw(input.cancel?Reason::Cancelled:Reason::Terminal);
            feedback_.terminal=!input.cancel&&input.terminal;return feedback_;
        }
        if(input.session!=feedback_.session||input.nominal.session!=input.session)
        {Withdraw(Reason::Session);return feedback_;}
        if(!input.tick||!input.nowUs||
           (haveClock_&&(lastTick_==(std::numeric_limits<std::uint64_t>::max)()||input.tick!=lastTick_+1ULL||
               input.nowUs<=lastUs_||input.nowUs-lastUs_<50ULL||input.nowUs-lastUs_>1000ULL))||
           (!haveClock_?!(input.nominal.tick==input.tick||
               (input.nominal.tick&&input.nominal.tick==input.tick-1ULL)):
               input.nominal.tick!=input.tick-1ULL))
        {Withdraw(Reason::Clock);return feedback_;}
        const bool seed=!haveClock_;const auto delta=seed?0ULL:input.nowUs-lastUs_;
        if(seed)firstUs_=input.nowUs;
        lastTick_=input.tick;lastUs_=input.nowUs;haveClock_=true;feedback_.tick=input.tick;
        if(!input.sourceValid){Withdraw(Reason::Source);return feedback_;}
        if(!input.powerInhibited){Withdraw(Reason::Power);return feedback_;}
        if(!ValidNominal(input.nominal,input.stationaryNominal)||!FinitePoint(input.candidateMCS))
        {Withdraw(Reason::Input);return feedback_;}
        UpdateProbe(input,delta);
        if(!frame_.sequence||input.nowUs-frame_.capturedAtUs>=SamplePeriodUs){
            if(frame_.sequence==(std::numeric_limits<std::uint64_t>::max)())
            {Withdraw(Reason::Source);return feedback_;}
            ++frame_.sequence;frame_.capturedAtUs=input.nowUs;frame_.valid=true;
            const auto phase=(input.nowUs-firstUs_)%(HighIntervalUs+LowIntervalUs);
            const bool low=feedback_.probePhase==ProbePhase::Retreat||
                (feedback_.probePhase!=ProbePhase::Return&&phase>=HighIntervalUs);
            frame_.voltageV=config_.referenceV+(low?-10.0:10.0);
        }
        if(!qualifier_.Observe(frame_,input.nowUs)){
            feedback_.sourceReason=qualifier_.Current().reason;Withdraw(Reason::Source);return feedback_;
        }
        const auto gap=EDMGapServo::Evaluate(config_.curve,frame_.voltageV,
            config_.referenceV,config_.machiningOverridePercent);
        if(!gap.valid){Withdraw(Reason::Configuration);return feedback_;}
        feedback_.sourceSequence=frame_.sequence;feedback_.sourceCapturedAtUs=frame_.capturedAtUs;
        feedback_.voltageV=frame_.voltageV;feedback_.signedGapSpeedMmMin=gap.limitedSpeedMmPerMin;
        bool acknowledged=false;
        if(!ApplyFlushAcknowledgement(input,acknowledged)){Withdraw(Reason::Input);return feedback_;}
        // EDM300: historical retreat and replay are an excursion from the
        // saved nominal cursor, even when their signed velocity is positive.
        // Neither direction may earn E5 time or a new flush candidate.
        feedback_.suspended=input.suspended||input.nominal.holdActive||input.nominal.held||
            input.nominal.flushActive||input.nominal.gapHistoryActive||input.nominal.finishActive;
        const auto& n=input.nominal;
        const bool eligible=!input.stationaryNominal&&!seed&&!acknowledged&&!feedback_.suspended&&IsWorkPhase(n.phase)&&n.phase==n.nominalPhase&&
            n.speedMmS>0.0&&n.nominalProgressMm>0.0&&n.nominalProgressMm<n.nominalLengthMm&&
            (!n.gapMotionEnabled||(n.gapApplied&&n.signedSpeedMmS>0.0))&&
            input.acceptedSendTick!=0ULL&&input.acceptedSendTick==input.tick-1ULL&&
            gap.limitedSpeedMmPerMin>0.0;
        feedback_.eligible=eligible&&!feedback_.pending;
        if(!feedback_.pending){
            if(!EDM130::AccrueWork(work_,delta,eligible)){Withdraw(Reason::Input);return feedback_;}
            if(workBaseUs_>(std::numeric_limits<std::uint64_t>::max)()-work_.totalWorkUs)
            {Withdraw(Reason::Input);return feedback_;}
            feedback_.simulatedWorkUs=workBaseUs_+work_.totalWorkUs;feedback_.intervalWorkUs=work_.elapsedUs;
            if(work_.due&&eligible){
                if(requestHighWater_==(std::numeric_limits<std::uint64_t>::max)())
                {Withdraw(Reason::Input);return feedback_;}
                if(!EDM130::TriggerWork(work_,input.tick,input.nowUs)){Withdraw(Reason::Input);return feedback_;}
                feedback_.pending=true;feedback_.eligible=false;feedback_.requestSequence=++requestHighWater_;feedback_.requestTick=input.tick;
                feedback_.candidateMCS=input.candidateMCS;feedback_.nominalPhase=n.nominalPhase;
                feedback_.layerIndex=n.layerIndex;feedback_.nominalProgressMm=n.nominalProgressMm;
                feedback_.nominalLengthMm=n.nominalLengthMm;
            }
        }
        feedback_.valid=true;feedback_.reason=Reason::None;feedback_.sourceReason=EDM88::Reason::None;
        return feedback_;
    }
    void Revoke() noexcept{Withdraw(Reason::Cancelled);}
    const Feedback& Current() const noexcept{return feedback_;}

private:
    void FailProbe(std::uint64_t nowUs) noexcept
    {feedback_.probePhase=ProbePhase::Failed;firstUs_=nowUs;}
    void UpdateProbe(const Input& input,std::uint64_t delta) noexcept
    {
        if(feedback_.simulationMode!=SimulationMode::PreviousParentProbe||
           (feedback_.probePhase!=ProbePhase::Retreat&&feedback_.probePhase!=ProbePhase::Return))return;
        const auto& n=input.nominal;
        // Read already coherent Cycle observations. A ready flag is not an
        // entry receipt: distance and actual enter/return sequences decide.
        if(!n.previousParentHistoryReady||!n.gapMotionEnabled||n.retreatMode!=2U||
           !std::isfinite(n.previousParentLengthMm)||n.previousParentLengthMm<=0.0||
           !std::isfinite(n.previousParentProgressMm)||n.previousParentProgressMm<0.0||
           n.previousParentProgressMm>n.previousParentLengthMm||
           n.historyReturnSequence>n.historyBoundarySequence)
        {FailProbe(input.nowUs);return;}
        if(!probeSeeded_){
            if(n.historyBoundarySequence||n.historyReturnSequence||n.previousParentHistoryActive)
            {FailProbe(input.nowUs);return;}
            probeLengthMm_=n.previousParentLengthMm;probeSeeded_=true;
        }
        if(n.previousParentLengthMm!=probeLengthMm_){FailProbe(input.nowUs);return;}
        // HOLD can take arbitrary operator time. Only active valid source
        // observations consume this bounded test budget; no wall-clock wait.
        if(n.holdActive||n.held)return;
        if(delta>MaximumPreviousParentProbeUs-probeElapsedUs_){FailProbe(input.nowUs);return;}
        probeElapsedUs_+=delta;
        if(n.previousParentHistoryActive){
            if(!n.gapHistoryActive||!n.historyBoundarySequence){FailProbe(input.nowUs);return;}
            probeEntered_=true;
        }
        if(n.historyReturnSequence&&!probeEntered_){FailProbe(input.nowUs);return;}
        if(feedback_.probePhase==ProbePhase::Retreat){
            const double target=(std::min)(PreviousParentProbeRetreatMm,probeLengthMm_*0.25);
            if(probeEntered_&&n.previousParentHistoryActive&&
                n.previousParentProgressMm<=probeLengthMm_-target)
                feedback_.probePhase=ProbePhase::Return;
        }else if(probeEntered_&&n.historyReturnSequence&&!n.gapHistoryActive){
            feedback_.probePhase=ProbePhase::Complete;firstUs_=input.nowUs;
        }
    }
    bool ApplyFlushAcknowledgement(const Input& input,bool& acknowledged) noexcept
    {
        acknowledged=false;
        const auto complete=input.completedFlushSequence,abort=input.abortedFlushSequence;
        if((complete&&complete<feedback_.completedFlushSequence)||
           (abort&&abort<feedback_.abortedFlushSequence))return false;
        const bool newComplete=complete&&complete!=feedback_.completedFlushSequence;
        const bool newAbort=abort&&abort!=feedback_.abortedFlushSequence;
        if(!newComplete&&!newAbort)return true; // Repeated sticky pages are idempotent.
        if(newComplete&&newAbort)return false;
        const auto sequence=newComplete?complete:abort;
        if(!config_.flushMotionEnabled||!feedback_.pending||sequence!=feedback_.requestSequence||
           input.tick<=feedback_.requestTick||input.nowUs<=work_.lastTriggerUs)return false;
        if(newComplete){
            if(feedback_.completedFlushes==(std::numeric_limits<std::uint64_t>::max)()||
               !EDM130::CompleteWork(work_,input.tick,input.nowUs))return false;
            feedback_.completedFlushSequence=sequence;++feedback_.completedFlushes;
        }else{
            if(feedback_.abortedFlushes==(std::numeric_limits<std::uint64_t>::max)())return false;
            // HOLD abandonment is not a successful flush. Archive all simulated
            // work and start a fresh interval without manufacturing CompleteWork.
            workBaseUs_=feedback_.simulatedWorkUs;work_=EDM130::WorkEvidence{};
            if(!EDM130::BeginWork(work_,config_.workTimeUs))return false;
            feedback_.abortedFlushSequence=sequence;++feedback_.abortedFlushes;
        }
        feedback_.pending=false;feedback_.eligible=false;feedback_.intervalWorkUs=0ULL;
        feedback_.requestSequence=feedback_.requestTick=0ULL;feedback_.candidateMCS=Point8{};
        feedback_.nominalPhase=NCG76SphereCycle::Phase::Idle;feedback_.layerIndex=0U;
        feedback_.nominalProgressMm=feedback_.nominalLengthMm=0.0;
        acknowledged=true;return true;
    }
    static bool FinitePoint(const Point8& p) noexcept
    {for(double value:p)if(!std::isfinite(value))return false;return true;}
    static bool IsWorkPhase(NCG76SphereCycle::Phase phase) noexcept
    {
        using P=NCG76SphereCycle::Phase;
        // Return/advance power policy is intentionally not released by this
        // first simulation stage. HOLD/resume/initial retreat never accrue E5.
        return phase==P::InitialDepth||phase==P::CircleEntry||phase==P::Orbit;
    }
    static bool ValidNominal(const NCG76SphereCycle::Feedback& n,bool stationary) noexcept
    {
        using P=NCG76SphereCycle::Phase;
        if(stationary){
            if(n.holdRecoveryEnabled&&n.holdActive&&
               (NCG76SphereCycle::Detail::IsRecovery(n.phase)||n.phase==P::HeldAtStart))
                return n.fault==NCG76SphereCycle::Fault::None&&n.nominalPhase==P::InitialDepth&&
                    n.nominalLengthMm==0.0&&n.nominalProgressMm==0.0&&std::isfinite(n.speedMmS)&&n.speedMmS>=0.0&&
                    std::isfinite(n.signedSpeedMmS)&&n.speedMmS==std::fabs(n.signedSpeedMmS)&&!n.gapApplied&&
                    !n.flushActive&&!n.flushCheckpointSaved&&!n.gapHistoryActive&&n.layerIndex==0U&&n.completedLayers==0U;
            const bool restingPhase=n.phase==P::Settle||n.phase==P::HoldSettle||
                n.phase==P::HeldAtStart||n.phase==P::Complete;
            return n.fault==NCG76SphereCycle::Fault::None&&restingPhase&&n.nominalPhase==P::InitialDepth&&
                n.nominalLengthMm==0.0&&n.nominalProgressMm==0.0&&n.speedMmS==0.0&&n.signedSpeedMmS==0.0&&
                !n.gapApplied&&!n.flushActive&&!n.flushCheckpointSaved&&!n.gapHistoryActive&&
                n.layerIndex==0U&&n.completedLayers==0U;
        }
        return n.fault==NCG76SphereCycle::Fault::None&&
            static_cast<unsigned>(n.phase)<=static_cast<unsigned>(P::InspectionJogReturn)&&
            (!NCG76SphereCycle::Detail::IsRecovery(n.phase)||(n.holdRecoveryEnabled&&n.holdActive))&&
            n.nominalPhase>=P::InitialDepth&&n.nominalPhase<=P::AdvanceCenter&&
            std::isfinite(n.speedMmS)&&n.speedMmS>=0.0&&std::isfinite(n.nominalProgressMm)&&
            (!n.gapMotionEnabled||(std::isfinite(n.signedSpeedMmS)&&
                n.speedMmS==std::fabs(n.signedSpeedMmS)))&&
            std::isfinite(n.nominalLengthMm)&&n.nominalLengthMm>0.0&&n.nominalProgressMm>=0.0&&
            n.nominalProgressMm<=n.nominalLengthMm;
    }
    void Withdraw(Reason reason) noexcept
    {
        if(feedback_.probePhase==ProbePhase::Retreat||feedback_.probePhase==ProbePhase::Return)
            feedback_.probePhase=ProbePhase::Failed;
        active_=false;qualifier_.Cancel();feedback_.valid=feedback_.eligible=feedback_.pending=feedback_.suspended=false;
        feedback_.reason=reason;feedback_.signedGapSpeedMmMin=feedback_.voltageV=0.0;
        feedback_.sourceSequence=feedback_.sourceCapturedAtUs=feedback_.requestSequence=feedback_.requestTick=0ULL;
        feedback_.candidateMCS=Point8{};feedback_.nominalPhase=NCG76SphereCycle::Phase::Idle;
        feedback_.layerIndex=0U;feedback_.nominalProgressMm=feedback_.nominalLengthMm=0.0;
    }
    Config config_{};Feedback feedback_{};EDM130::WorkEvidence work_{};
    EDM88::Qualifier qualifier_{};EDM88::Frame frame_{};
    std::uint64_t sessionHighWater_=0ULL,lastTick_=0ULL,lastUs_=0ULL,firstUs_=0ULL;
    std::uint64_t requestHighWater_=0ULL,workBaseUs_=0ULL;
    std::uint64_t probeElapsedUs_=0ULL;
    double probeLengthMm_=0.0;
    bool probeSeeded_=false,probeEntered_=false;
    bool active_=false,haveClock_=false;
};
static_assert(std::is_trivially_copyable<Config>::value,"P50 SIM config is an immutable value");
static_assert(sizeof(Config)<=400U,"Parameter-feed mode must use existing Config padding");
static_assert(std::is_trivially_copyable<Feedback>::value&&sizeof(Feedback)<=256U,"P50 SIM feedback stays bounded");
static_assert(!MotionExecutionReleased&&!DischargeExecutionReleased,"P50 SIM observer cannot authorize outputs");
}
