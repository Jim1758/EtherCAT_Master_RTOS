#pragma once

#include "NCG76SphereCycle.h"
#include "NCMachiningTime.h"
#include "NCG76SphereDeepFlush.h"
#include "EDM130WorkTimer.h"
#include "EDMCoordinatedGapSource.h"
#include "EDMGapServo.h"
#include "EDMFlushPlan.h"
#include "EDME5WorkTimer386.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM293/382: live RT process observation, SIM by default; explicit physical
// Z mode consumes a qualified ADC frame. This observer never
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
    // EDM339: opt-in P50 machining-role feeds. Ordinary Line/Arc parents keep
    // these fields canonical zero and retain their original E8 curve.
    double orbitScaleMmPerVoltMin=0.0,orbitOverridePercent=0.0;
    bool orbitFeedsEnabled=false;
    // Manual external generator: these limits grant axis motion only.
    bool physicalZManual=false;
    // Native envelope, frozen and revalidated by Motion at admission. Signed
    // EDM derives its finite envelope from MAX_Speed and the PDO interval,
    // independent of G00/Stop times; explicit mode caps are applied separately.
    // Legacy field names: native units/s and native units/s^2 (mm or degrees).
    std::array<double,6U> flushAxisVelocityMmS{},flushAxisAccelerationMmS2{},flushAxisDecelerationMmS2{};
    std::uint32_t flushAxisMask=7U;
    bool flushAxisLimitsEnabled=false;
    // E5-only gate: raw GAP servo and D/H clocks remain independent.
    EDM386E5::Profile e5WorkTimer{};

};
inline bool Disabled(const Config& c) noexcept{return !c.enabled;}
enum class FeedRole : std::uint8_t { Unavailable, Cutting, Orbit };
inline const char* FeedRoleName(FeedRole role) noexcept
{return role==FeedRole::Cutting?"CUTTING":role==FeedRole::Orbit?"ORBIT":"UNAVAILABLE";}
struct FeedSelection
{
    double scaleMmPerVoltMin=0.0,overridePercent=0.0;
    FeedRole role=FeedRole::Unavailable;
};
// The RT owner supplies the active leg's role, including a retained history
// leg. Do not infer this from the saved forward cursor, which intentionally
// stays unchanged while GAP retreats through an earlier depth/entry leg.
inline bool ResolveFeedSelection(const Config& c,NCG76SphereCycle::Phase phase,FeedSelection& out) noexcept
{
    out=FeedSelection{};
    using P=NCG76SphereCycle::Phase;
    if(c.orbitFeedsEnabled&&phase!=P::Idle&&phase!=P::ContourFinish&&
        (phase<P::InitialDepth||phase>P::AdvanceCenter))return false;
    const bool orbit=c.orbitFeedsEnabled&&(phase==P::CircleEntry||phase==P::Orbit||phase==P::ContourFinish);
    out.role=orbit?FeedRole::Orbit:FeedRole::Cutting;
    out.scaleMmPerVoltMin=orbit?c.orbitScaleMmPerVoltMin:c.curve.cuttingScaleMmPerVoltMin;
    out.overridePercent=orbit?c.orbitOverridePercent:c.machiningOverridePercent;
    return true;
}
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
    if(!EDM386E5::Valid(c.e5WorkTimer)||(!c.enabled&&c.e5WorkTimer.mode!=0U))return false;
    if(c.e5WorkTimer.mode==1U){
        const double shortFloor=c.curve.shortMachiningEnabled?
            (std::max)(c.curve.shortIdleV,c.curve.shortMachiningV):c.curve.shortIdleV;
        if(!std::isfinite(shortFloor)||c.e5WorkTimer.gapHighVoltageV<=shortFloor)return false;
    }
    if(!EDM385::Valid(c.flushProfile.axisModeLimits))return false;
    if(!c.enabled&&!EDM385::Empty(c.flushProfile.axisModeLimits))return false;
    if(!c.flushAxisMask||(c.flushAxisMask&~0x3fU)!=0U)return false;
    for(unsigned a=0U;a<6U;++a){
        const double v=c.flushAxisVelocityMmS[a],acc=c.flushAxisAccelerationMmS2[a],dec=c.flushAxisDecelerationMmS2[a];
        if(c.flushAxisLimitsEnabled&&(c.flushAxisMask&(1U<<a))){
            if(!c.enabled||!std::isfinite(v)||v<=0.0||!std::isfinite(acc)||acc<=0.0||!std::isfinite(dec)||dec<=0.0)return false;
        }else if(v!=0.0||acc!=0.0||dec!=0.0)return false;
    }
    if(c.physicalZManual&&(!c.enabled||!c.signedMotionEnabled||!c.parameterFeeds||c.orbitFeedsEnabled))return false;
    if(c.orbitFeedsEnabled){
        if(!c.enabled||!c.signedMotionEnabled||c.parameterFeeds||
           !std::isfinite(c.orbitScaleMmPerVoltMin)||c.orbitScaleMmPerVoltMin<0.001||
           !std::isfinite(c.orbitOverridePercent)||c.orbitOverridePercent<=0.0||c.orbitOverridePercent>100.0)return false;
        auto curve=c.curve;curve.cuttingScaleMmPerVoltMin=c.orbitScaleMmPerVoltMin;
        if(!EDMGapServo::Evaluate(curve,0.0,c.referenceV,100.0).valid||
           !EDMGapServo::Evaluate(curve,300.0,c.referenceV,100.0).valid)return false;
    }else if(c.orbitScaleMmPerVoltMin!=0.0||c.orbitOverridePercent!=0.0)return false;
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
// accounting totals to a fresh observer. Mode 1 also retains its elapsed-clock
// observations. This contains no pending request, source sample, completion
// acknowledgement or motion permit.
// The RT adapter proves the previous completed parent and frozen recipe scope;
// accepting these numbers alone never authorizes a cross-parent handoff.
struct Continuation
{
    std::uint64_t intervalWorkUs=0ULL,simulatedWorkUs=0ULL;
    std::uint64_t completedFlushes=0ULL,abortedFlushes=0ULL;
    EDM386E5::Evidence e5Evidence{};
    // Only the separately proved adjacent-parent handoff may retain these
    // monotonic-clock observations. They carry no sample or motion authority.
    std::uint64_t e5StartUs=0ULL,e5ObservedUs=0ULL;
};
inline bool ValidContinuation(const Config& config,const Continuation& value) noexcept
{
    const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
    return config.enabled&&Valid(config)&&EDM386E5::ValidEvidence(config.e5WorkTimer,value.e5Evidence)&&EDM130::ValidWorkTime(config.workTimeUs)&&
        value.intervalWorkUs<=config.workTimeUs&&value.simulatedWorkUs>=value.intervalWorkUs&&
        (config.e5WorkTimer.mode==0U||
         ((!value.e5StartUs||value.e5ObservedUs>=value.e5StartUs)&&
          ((!value.e5StartUs&&!value.e5ObservedUs)||value.e5Evidence.latched)))&&
        (config.e5WorkTimer.mode==0U||value.intervalWorkUs==0ULL||value.e5Evidence.latched)&&
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
    if(!EDM385::Same(a.axisModeLimits,b.axisModeLimits))return false;
    if(a.axisRampMask!=b.axisRampMask)return false;
    for(unsigned axis=0U;axis<6U;++axis)
        if(!SameDouble(a.axisAccelerationTimeSeconds[axis],b.axisAccelerationTimeSeconds[axis])||
           !SameDouble(a.axisDecelerationTimeSeconds[axis],b.axisDecelerationTimeSeconds[axis]))return false;
    return SameDouble(a.settleTimeSeconds,b.settleTimeSeconds)&&
        SameDouble(a.accelerationTimeSeconds,b.accelerationTimeSeconds)&&
        SameDouble(a.decelerationTimeSeconds,b.decelerationTimeSeconds)&&
        SameDouble(a.finalApproachMmPerMin,b.finalApproachMmPerMin)&&
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
    if(!EDM386E5::Same(a.e5WorkTimer,b.e5WorkTimer)||a.flushAxisLimitsEnabled!=b.flushAxisLimitsEnabled||a.flushAxisMask!=b.flushAxisMask)return false;
    for(unsigned axis=0U;axis<6U;++axis)
        if(!SameDouble(a.flushAxisVelocityMmS[axis],b.flushAxisVelocityMmS[axis])||
           !SameDouble(a.flushAxisAccelerationMmS2[axis],b.flushAxisAccelerationMmS2[axis])||
           !SameDouble(a.flushAxisDecelerationMmS2[axis],b.flushAxisDecelerationMmS2[axis]))return false;
    return a.enabled&&b.enabled&&Valid(a)&&Valid(b)&&
        a.workTimeUs==b.workTimeUs&&a.recipeGeneration==b.recipeGeneration&&a.tuningGeneration==b.tuningGeneration&&
        a.enabled==b.enabled&&a.flushMotionEnabled==b.flushMotionEnabled&&a.signedMotionEnabled==b.signedMotionEnabled&&
        a.parameterFeeds==b.parameterFeeds&&a.orbitFeedsEnabled==b.orbitFeedsEnabled&&
        a.physicalZManual==b.physicalZManual&&
        SameDouble(a.orbitScaleMmPerVoltMin,b.orbitScaleMmPerVoltMin)&&
        SameDouble(a.orbitOverridePercent,b.orbitOverridePercent)&&
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
// A live edit changes numerical curve/feeds only. Recipe, source, timers,
// short qualification, geometry and the frozen native motion envelope stay bound
// to the admitted parent. The caller separately authenticates the new tuning
// generation and preserves the original immutable process publication.
inline void CopyLiveCurveFields(const EDMGapServo::Profile& from,EDMGapServo::Profile& to) noexcept
{
    for(unsigned i=0U;i<3U;++i){to.positiveGain[i]=from.positiveGain[i];to.negativeGain[i]=from.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){to.positiveBreakV[i]=from.positiveBreakV[i];to.negativeBreakV[i]=from.negativeBreakV[i];}
    to.cuttingScaleMmPerVoltMin=from.cuttingScaleMmPerVoltMin;
    to.maxFeedMmPerMin=from.maxFeedMmPerMin;to.maxRetreatMmPerMin=from.maxRetreatMmPerMin;
    to.deadbandV=from.deadbandV;to.shortRetreatMmPerMin=from.shortRetreatMmPerMin;
}
inline bool EquivalentForLiveTuning(const Config& birth,const Config& candidate) noexcept
{
    if(!Valid(birth)||!Valid(candidate)||
       !EDMGapServo::Evaluate(candidate.curve,0.0,candidate.referenceV,candidate.machiningOverridePercent).valid||
       !EDMGapServo::Evaluate(candidate.curve,300.0,candidate.referenceV,candidate.machiningOverridePercent).valid)return false;
    Config normalized=candidate;
    CopyLiveCurveFields(birth.curve,normalized.curve);
    normalized.curve.revision=birth.curve.revision;
    normalized.tuningGeneration=birth.tuningGeneration;
    auto& to=normalized.flushProfile;const auto& from=birth.flushProfile;
    to.finalApproachMmPerMin=from.finalApproachMmPerMin;
    to.centerRetractMmPerMin=from.centerRetractMmPerMin;to.mainRetractMmPerMin=from.mainRetractMmPerMin;
    to.centerReturnMmPerMin=from.centerReturnMmPerMin;to.mainReturnMmPerMin=from.mainReturnMmPerMin;
    to.pathRetractMmPerMin=from.pathRetractMmPerMin;to.initialSlowMmPerMin=from.initialSlowMmPerMin;
    to.manualReturnMmPerMin=from.manualReturnMmPerMin;
    return EquivalentForContinuation(birth,normalized);
}
// Full, canonically authenticated E-condition update. Only recipe-derived
// values and the established hot-tuning curve/feed subset may differ. Source,
// native envelopes, E5 gate profile and protection configuration stay frozen.
inline bool EquivalentForLiveRecipe(const Config& birth,const Config& candidate) noexcept
{
    if(!Valid(birth)||!Valid(candidate)||
       !EDMGapServo::Evaluate(candidate.curve,0.0,candidate.referenceV,candidate.machiningOverridePercent).valid||
       !EDMGapServo::Evaluate(candidate.curve,300.0,candidate.referenceV,candidate.machiningOverridePercent).valid)return false;
    Config normalized=candidate;
    normalized.workTimeUs=birth.workTimeUs;normalized.recipeGeneration=birth.recipeGeneration;
    normalized.jumpHeightMm=birth.jumpHeightMm;normalized.jumpSpeedPercent=birth.jumpSpeedPercent;
    normalized.machiningOverridePercent=birth.machiningOverridePercent;normalized.referenceV=birth.referenceV;
    normalized.flushMotionEnabled=birth.flushMotionEnabled;normalized.independentFeedsEnabled=birth.independentFeedsEnabled;
    normalized.finalApproachEnabled=birth.finalApproachEnabled;
    normalized.finalApproachPercent=birth.finalApproachPercent;normalized.finalApproachDistanceMm=birth.finalApproachDistanceMm;
    normalized.initialSlowEnabled=birth.initialSlowEnabled;
    normalized.initialSlowPercent=birth.initialSlowPercent;normalized.initialSlowDistanceMm=birth.initialSlowDistanceMm;
    normalized.deepFlushEnabled=birth.deepFlushEnabled;normalized.deepFlushCycleInterval=birth.deepFlushCycleInterval;
    normalized.deepFlushHeightMultiplier=birth.deepFlushHeightMultiplier;normalized.deepFlushHeightMm=birth.deepFlushHeightMm;
    normalized.orbitOverridePercent=birth.orbitOverridePercent;
    return EquivalentForLiveTuning(birth,normalized);
}
inline bool SameLiveRecipeFlush(const Config& a,const Config& b) noexcept
{
    using ContinuationDetail::SameDouble;
    return a.flushMotionEnabled==b.flushMotionEnabled&&a.independentFeedsEnabled==b.independentFeedsEnabled&&
        a.finalApproachEnabled==b.finalApproachEnabled&&a.initialSlowEnabled==b.initialSlowEnabled&&
        a.deepFlushEnabled==b.deepFlushEnabled&&a.deepFlushCycleInterval==b.deepFlushCycleInterval&&
        SameDouble(a.jumpHeightMm,b.jumpHeightMm)&&SameDouble(a.jumpSpeedPercent,b.jumpSpeedPercent)&&
        SameDouble(a.finalApproachPercent,b.finalApproachPercent)&&SameDouble(a.finalApproachDistanceMm,b.finalApproachDistanceMm)&&
        SameDouble(a.initialSlowPercent,b.initialSlowPercent)&&SameDouble(a.initialSlowDistanceMm,b.initialSlowDistanceMm)&&
        SameDouble(a.deepFlushHeightMultiplier,b.deepFlushHeightMultiplier)&&SameDouble(a.deepFlushHeightMm,b.deepFlushHeightMm)&&
        ContinuationDetail::SameFlush(a.flushProfile,b.flushProfile);
}
static_assert(sizeof(Continuation)<=64U&&std::is_trivially_copyable<Continuation>::value,
    "Process continuation is bounded accounting data, never an execution receipt");
// Shared NC/RT derivation; whole-cycle travel admission remains the motion
// owner's independent proof. E6 is physical native mm, never authored scale.
inline bool BuildFlushConfig(const Config& config,double feedMmMin,
    NCG76SphereCycle::FlushConfig& output) noexcept
{
    output=NCG76SphereCycle::FlushConfig{};
    if(!Valid(config))return false;
    output.axisModeLimits=config.flushProfile.axisModeLimits;
    if(!config.enabled)return true;
    // Native motion limits govern machining, GAP retreat and HOLD even when
    // the current E condition disables periodic flushing (E6 == 0).
    output.axisVelocityMmS=config.flushAxisVelocityMmS;
    output.axisAccelerationMmS2=config.flushAxisAccelerationMmS2;
    output.axisDecelerationMmS2=config.flushAxisDecelerationMmS2;
    output.axisLimitsEnabled=config.flushAxisLimitsEnabled;
    output.axisMask=config.flushAxisMask;
    output.accelerationTimeSeconds=config.flushProfile.accelerationTimeSeconds;
    output.decelerationTimeSeconds=config.flushProfile.decelerationTimeSeconds;
    output.axisAccelerationTimeSeconds=config.flushProfile.axisAccelerationTimeSeconds;
    output.axisDecelerationTimeSeconds=config.flushProfile.axisDecelerationTimeSeconds;
    output.axisRampMask=config.flushProfile.axisRampMask;
    output.settleTimeSeconds=config.flushProfile.settleTimeSeconds;
    // Keep the frozen native/ramp/settle envelope even when E6 disables periodic
    // flushing, so a later valid E6 enable uses the same admitted envelope.
    if(!config.flushMotionEnabled)return true;
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
    // EDM339: POST-step active leg for the next signed-GAP demand. E5 below
    // still consumes the coherent PRE-step nominal/accepted-send evidence.
    NCG76SphereCycle::Phase gapFeedPhase=NCG76SphereCycle::Phase::Idle;
    Point8 candidateMCS{}; // Matching PRE-step nominal command; no stop proof.
    std::uint64_t session=0ULL,tick=0ULL,nowUs=0ULL,acceptedSendTick=0ULL;
    // Sticky, independently proved post-Step cycle acknowledgements. The
    // nominal/candidate above remain the coherent PRE-step source for E5.
    std::uint64_t completedFlushSequence=0ULL,abortedFlushSequence=0ULL;
    // EDM307: RT alone derives this from its frozen, canonically validated
    // zero-length Line. It does not turn malformed moving feedback into rest.
    bool stationaryNominal=false;
    // EDM340: RT-owned physical endpoint receipt for an active additional
    // primitive. This marker alone earns nothing; Step checks the whole
    // matching PRE-step parent and the current positive GAP sample again.
    bool additionalEndpointQualified=false;
    bool sourceValid=false,powerInhibited=false,suspended=false,terminal=false,cancel=false;
    // HOLD/return-to-process reset is distinct from normal GAP history retreat.
    bool holdResetActive=false,currentFlushActive=false,timingStopActive=false;
    std::uint64_t currentFlushSequence=0ULL;
    EDM88::Frame physicalFrame{}; // Actual immutable PDO receipt, never synthetic.
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
    FeedRole selectedFeedRole=FeedRole::Unavailable;
    bool enabled=false,valid=false,eligible=false,suspended=false,pending=false,terminal=false;
    double selectedScaleMmPerVoltMin=0.0,selectedOverridePercent=0.0;
    EDM386E5::Feedback e5Timer{};
    // Only this historical accounting evidence may cross an admitted adjacent parent.
    EDM386E5::Evidence e5Evidence{};
    std::uint64_t e5DueEvents=0ULL,e5LastDueTick=0ULL;
    bool e5Eligible=false,e5ShortBlocked=false;
    // Actual curve result for this observer tick, independent of nominal
    // motion direction. The motion owner separately proves its consumption.
    std::int32_t gainRegion=0;
    bool gainRegionValid=false;
    double referenceV=0.0;
};

class Observer
{
public:
    bool Begin(const Config& config,std::uint64_t session,const Continuation* continuation=nullptr,
        SimulationMode mode=SimulationMode::Wave,
        const NCG76SphereRuntimeGeometry::Geometry* polygonGeometry=nullptr,
        const EDM88::SourceConfig* physicalSource=nullptr) noexcept
    {
        qualifier_.Cancel();active_=false;feedback_=Feedback{};work_=EDM130::WorkEvidence{};
        e5Gate_=EDM386E5::Gate{};transportPaused_=requalifying_=physicalLineNominal_=false;
        lastRequalifiedTick_=0ULL;requalifyMaxTransportGapUs_=100000ULL;
        physicalShort_.Reset();physicalShortResult_=EDMGapServo::ShortResult{};physicalShortSequence_=0ULL;
        frame_=EDM88::Frame{};lastTick_=lastUs_=firstUs_=0ULL;haveClock_=false;
        requestHighWater_=workBaseUs_=pendingWorkTimeUs_=0ULL;
        e5StartUs_=e5DueUs_=elapsedClockUs_=0ULL;holdRoundReset_=false;
        pendingLiveRecipeFlush_=false;pendingLiveRecipeConfig_=Config{};
        probeElapsedUs_=0ULL;probeLengthMm_=0.0;probeSeeded_=probeEntered_=false;
        feedback_.simulationMode=mode;finishLegCount_=4U;nominalLegCount_=25U;nominalWorkEndIndex_=24U;
        if(Disabled(config)&&!config.flushMotionEnabled&&!config.signedMotionEnabled){
            if(!Valid(config)||continuation||mode!=SimulationMode::Wave||polygonGeometry){Withdraw(Reason::Configuration);return false;}
            return true;
        }
        feedback_.enabled=config.enabled;feedback_.session=session;feedback_.recipeGeneration=config.recipeGeneration;
        feedback_.tuningGeneration=config.tuningGeneration;feedback_.workTimeUs=config.workTimeUs;
        feedback_.referenceV=config.referenceV;
        if(config.physicalZManual!=(physicalSource!=nullptr)||
           (physicalSource&&(!EDM88::ValidPhysical(*physicalSource)||mode!=SimulationMode::Wave)))
        {Withdraw(Reason::Configuration);return false;}
        if(!session||session<=sessionHighWater_){Withdraw(Reason::Lifecycle);return false;}
        sessionHighWater_=session; // Even failed/withdrawn attempts cannot rearm this identity.
        if((mode!=SimulationMode::Wave&&mode!=SimulationMode::PreviousParentProbe)||
           (mode==SimulationMode::PreviousParentProbe&&
                (!continuation||!config.parameterFeeds||!config.enabled||!config.signedMotionEnabled)))
        {Withdraw(Reason::Configuration);return false;}
        if(!Valid(config)||(continuation&&!ValidContinuation(config,*continuation)))
        {Withdraw(Reason::Configuration);return false;}
        // The RT admission owner may bind validated P01-P07/P16/P17/P21-P27 geometry once.
        // Capture its bounded leg counts only; no geometry pointer enters the
        // periodic observer. Legacy callers retain 25/24/4 and their lifecycle.
        if(polygonGeometry){
            if(continuation||mode!=SimulationMode::Wave||
               polygonGeometry->kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||
               (!NCG76OrbitContour::IsFixedRadiusContour(*polygonGeometry)&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=16U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=17U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=21U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=22U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=23U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=24U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=25U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=26U&&
                NCG76OrbitContour::ModeP(*polygonGeometry)!=27U)||
               !NCG76SphereRuntimeGeometry::Valid(*polygonGeometry))
            {Withdraw(Reason::Configuration);return false;}
            const unsigned count=NCG76OrbitContour::FinishLegCount(*polygonGeometry);
            const bool circle=NCG76OrbitContour::IsCircleContour(*polygonGeometry);
            const bool fourSide=NCG76OrbitContour::IsFourSideContour(*polygonGeometry);
            if((circle&&count!=0U)||(!circle&&NCG76OrbitContour::FinishEnabled(*polygonGeometry)&&
                ((fourSide&&count!=4U)||(!fourSide&&
                    (count<3U||count>12U||count!=polygonGeometry->orbit.source.polygonSides))))||
               (!NCG76OrbitContour::FinishEnabled(*polygonGeometry)&&count!=0U))
            {Withdraw(Reason::Configuration);return false;}
            nominalLegCount_=NCG76OrbitContour::NominalLegCountFor(*polygonGeometry);
            if(nominalLegCount_!=((NCG76OrbitContour::IsExpansionContour(*polygonGeometry)||
                NCG76OrbitContour::IsFixedRadiusContour(*polygonGeometry))?26U:25U))
            {Withdraw(Reason::Configuration);return false;}
            nominalWorkEndIndex_=nominalLegCount_-1U;finishLegCount_=count;
        }
        config_=config;
        if(!e5Gate_.Begin(config.e5WorkTimer,continuation?&continuation->e5Evidence:nullptr))
        {Withdraw(Reason::Configuration);return false;}
        feedback_.e5Timer=e5Gate_.Current();feedback_.e5Evidence=e5Gate_.Export();
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
        const bool sourceReady=physicalSource?qualifier_.BeginPhysical(*physicalSource,session):qualifier_.Begin(source);
        if(!sourceReady||!EDM130::BeginWork(work_,config.workTimeUs))
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
            if(config.e5WorkTimer.mode==1U&&continuation->e5Evidence.latched){
                e5StartUs_=continuation->e5StartUs;
                elapsedClockUs_=continuation->e5ObservedUs;
            }
            if(!EDM130::ValidWorkEvidence(work_)){Withdraw(Reason::Configuration);return false;}
        }
        frame_.identity=physicalSource?physicalSource->identity:source.identity;feedback_.reason=Reason::None;feedback_.sourceReason=EDM88::Reason::None;
        active_=true;return true;
    }
    // RT owner only. Retiming never restarts the source, electrical gate or
    // parent identity. An issued flush retains its original timing evidence;
    // only the latest requested duration is adopted after its real ACK.
    bool UpdateWorkTime(std::uint64_t workTimeUs) noexcept
    {
        if(!active_||!feedback_.enabled||feedback_.terminal||
           !EDM130::ValidWorkTime(workTimeUs)||!EDM130::ValidWorkEvidence(work_))return false;
        if(feedback_.pending){
            pendingWorkTimeUs_=workTimeUs==config_.workTimeUs?0ULL:workTimeUs;
            return true;
        }
        if(!ApplyWorkTimeUpdate(workTimeUs))return false;
        pendingWorkTimeUs_=0ULL;
        return true;
    }
    bool HasPendingWorkTimeUpdate() const noexcept
    {return active_&&pendingWorkTimeUs_!=0ULL;}
    bool UpdateLiveCurve(const EDMGapServo::Profile& curve) noexcept
    {
        if(!active_||!feedback_.enabled||feedback_.terminal||!EDMGapServo::ValidateProfile(curve))return false;
        auto normalized=curve;
        CopyLiveCurveFields(config_.curve,normalized);
        // Keep the parent's original revision as well as detector thresholds,
        // dwell and enable flags. This never resets a short or E5 latch.
        if(!ContinuationDetail::SameCurve(config_.curve,normalized)||
           !EDMGapServo::Evaluate(curve,0.0,config_.referenceV,config_.machiningOverridePercent).valid||
           !EDMGapServo::Evaluate(curve,300.0,config_.referenceV,config_.machiningOverridePercent).valid)return false;
        config_.curve=curve;return true;
    }
    bool UpdateLiveRecipe(const Config& config) noexcept
    {
        if(!active_||!feedback_.enabled||feedback_.terminal||
           !EquivalentForLiveRecipe(config_,config)||!EDM130::ValidWorkEvidence(work_))return false;
        // WorkEvidence owns an issued request until its real ACK. Its existing
        // retiming API preserves current credit and queues only the latest E5.
        if(!UpdateWorkTime(config.workTimeUs))return false;
        auto curve=config.curve;curve.revision=config_.curve.revision;
        config_.curve=curve;config_.referenceV=config.referenceV;
        config_.machiningOverridePercent=config.machiningOverridePercent;
        config_.orbitOverridePercent=config.orbitOverridePercent;
        // An E6 disable cannot invalidate the old request's completion proof.
        // Keep that whole flush image until its actual complete/abort ACK.
        if(feedback_.pending&&!SameLiveRecipeFlush(config_,config)){
            pendingLiveRecipeConfig_=config;pendingLiveRecipeFlush_=true;
        }else{
            ApplyLiveRecipeFlush(config);pendingLiveRecipeFlush_=false;pendingLiveRecipeConfig_=Config{};
        }
        return true;
    }
    bool HasPendingLiveRecipeFlush() const noexcept
    {return active_&&pendingLiveRecipeFlush_;}
    bool LiveFlushEnabled() const noexcept
    {return active_&&config_.flushMotionEnabled;}
    // Local interval diagnostics only; neither timestamp grants work credit.
    std::uint64_t E5StartUs() const noexcept {return e5StartUs_;}
    std::uint64_t E5DueUs() const noexcept {return e5DueUs_;}
    std::uint64_t E5ObservedUs() const noexcept {return elapsedClockUs_;}
    const Feedback& Step(const Input& input) noexcept
    {
        if(!requalifying_)lastRequalifiedTick_=0ULL;
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
        if(transportPaused_&&!requalifying_)return feedback_;
        if(!input.tick||!input.nowUs||
           (!requalifying_&&((haveClock_&&(lastTick_==(std::numeric_limits<std::uint64_t>::max)()||input.tick!=lastTick_+1ULL||
               input.nowUs<=lastUs_||input.nowUs-lastUs_<50ULL||input.nowUs-lastUs_>1000ULL))||
           (!haveClock_?!(input.nominal.tick==input.tick||
               (input.nominal.tick&&input.nominal.tick==input.tick-1ULL)):
               input.nominal.tick!=input.tick-1ULL))))
        {Withdraw(Reason::Clock);return feedback_;}
        const bool seed=!haveClock_||requalifying_;const auto delta=seed?0ULL:input.nowUs-lastUs_;
        if(seed&&!requalifying_)firstUs_=input.nowUs;
        lastTick_=input.tick;lastUs_=input.nowUs;haveClock_=true;feedback_.tick=input.tick;
        if(!input.sourceValid){Withdraw(Reason::Source);return feedback_;}
        if(!input.powerInhibited){Withdraw(Reason::Power);return feedback_;}
        if(!ValidNominal(input.nominal,input.stationaryNominal)||!FinitePoint(input.candidateMCS))
        {Withdraw(Reason::Input);return feedback_;}
        UpdateProbe(input,delta);
        if(config_.physicalZManual)frame_=input.physicalFrame;
        else if(!frame_.sequence||input.nowUs-frame_.capturedAtUs>=SamplePeriodUs){
            if(frame_.sequence==(std::numeric_limits<std::uint64_t>::max)())
            {Withdraw(Reason::Source);return feedback_;}
            ++frame_.sequence;frame_.capturedAtUs=input.nowUs;frame_.valid=true;
            const auto phase=(input.nowUs-firstUs_)%(HighIntervalUs+LowIntervalUs);
            const bool low=feedback_.probePhase==ProbePhase::Retreat||
                (feedback_.probePhase!=ProbePhase::Return&&phase>=HighIntervalUs);
            frame_.voltageV=config_.referenceV+(low?-10.0:10.0);
        }
        if(!(requalifying_?qualifier_.RequalifyPhysicalAfterTransportGap(frame_,input.nowUs,requalifyMaxTransportGapUs_):qualifier_.Observe(frame_,input.nowUs))){
            feedback_.sourceReason=qualifier_.Current().reason;Withdraw(Reason::Source);return feedback_;
        }
        FeedSelection selection{};
        if(!ResolveFeedSelection(config_,input.gapFeedPhase,selection))
        {Withdraw(Reason::Input);return feedback_;}
        auto selectedCurve=config_.curve;
        selectedCurve.cuttingScaleMmPerVoltMin=selection.scaleMmPerVoltMin;
        const auto gap=EDMGapServo::Evaluate(selectedCurve,frame_.voltageV,
            config_.referenceV,selection.overridePercent);
        if(!gap.valid){Withdraw(Reason::Configuration);return feedback_;}
        double signedGapSpeed=gap.limitedSpeedMmPerMin;
        bool gainRegionValid=true;
        std::int32_t gainRegion=gap.segment>=1U&&gap.segment<=3U?
            (gap.errorV>0.0?static_cast<std::int32_t>(gap.segment):
             gap.errorV<0.0?-static_cast<std::int32_t>(gap.segment):0):0;
        if(config_.physicalZManual){
            if(frame_.sequence!=physicalShortSequence_){
                EDMGapServo::Sample sample{};sample.valid=true;sample.voltage=frame_.voltageV;
                sample.nowMs=frame_.capturedAtUs/1000ULL;sample.sequence=frame_.sequence;
                physicalShortResult_=physicalShort_.Step(selectedCurve,sample,true);
                physicalShortSequence_=frame_.sequence;
            }
            if(!physicalShortResult_.valid){Withdraw(Reason::Source);return feedback_;}
            if(physicalShortResult_.shortActive){
                const auto retreat=EDMGapServo::EvaluateShortRetreat(selectedCurve,gap);
                if(!retreat.valid){Withdraw(Reason::Configuration);return feedback_;}
                signedGapSpeed=retreat.signedSpeedMmPerMin;
                if(!retreat.usingCurve)signedGapSpeed*=selection.overridePercent/100.0;
                if(!retreat.usingCurve||gap.limitedSpeedMmPerMin>=0.0||signedGapSpeed>=0.0){
                    gainRegion=0;gainRegionValid=false;
                }
            }else if(physicalShortResult_.feedInhibited&&signedGapSpeed>0.0){
                signedGapSpeed=0.0;gainRegion=0;gainRegionValid=false;
            }
            if(!std::isfinite(signedGapSpeed)){Withdraw(Reason::Configuration);return feedback_;}
        }
        if(config_.orbitFeedsEnabled){
            feedback_.selectedFeedRole=selection.role;
            feedback_.selectedScaleMmPerVoltMin=selection.scaleMmPerVoltMin;
            feedback_.selectedOverridePercent=selection.overridePercent;
        }
        feedback_.sourceSequence=frame_.sequence;feedback_.sourceCapturedAtUs=frame_.capturedAtUs;
        feedback_.voltageV=frame_.voltageV;feedback_.signedGapSpeedMmMin=signedGapSpeed;
        feedback_.gainRegion=gainRegion;feedback_.gainRegionValid=gainRegionValid;
        feedback_.referenceV=config_.referenceV;
        bool acknowledged=false;
        if(!ApplyFlushAcknowledgement(input,acknowledged)){Withdraw(Reason::Input);return feedback_;}
        if(acknowledged&&pendingWorkTimeUs_!=0ULL){
            if(!ApplyWorkTimeUpdate(pendingWorkTimeUs_)){Withdraw(Reason::Input);return feedback_;}
            pendingWorkTimeUs_=0ULL;
        }
        if(acknowledged&&pendingLiveRecipeFlush_){
            ApplyLiveRecipeFlush(pendingLiveRecipeConfig_);
            pendingLiveRecipeFlush_=false;pendingLiveRecipeConfig_=Config{};
        }
        const auto& n=input.nominal;
        const bool holdNow=input.holdResetActive||n.holdActive||n.held;
        if(holdNow&&!holdRoundReset_){
            // An accepted excursion may survive primitive/recovery HOLD.
            // Preserve its ledger for the real Cycle ACK; an unaccepted
            // proposal has no ACK and is canceled without inventing one.
            const bool accepted=feedback_.pending&&
                ((n.flushActive&&n.flushRequestSequence==feedback_.requestSequence)||
                 (input.currentFlushActive&&input.currentFlushSequence==feedback_.requestSequence));
            if(!ResetRoundForHold(accepted,input.tick)){Withdraw(Reason::Input);return feedback_;}
        }
        holdRoundReset_=holdNow;
        // EDM300: historical retreat and replay are an excursion from the
        // saved nominal cursor, even when their signed velocity is positive.
        // Neither direction may earn E5 time or a new flush candidate.
        feedback_.suspended=input.suspended||input.nominal.holdActive||input.nominal.held||
            input.nominal.flushActive||input.nominal.gapHistoryActive||input.nominal.finishActive||
            IsTimeLimitStopping(input.nominal);
        const bool movingWork=IsWorkPhase(n.phase,n.layerIndex)&&n.phase==n.nominalPhase&&
            n.speedMmS>0.0&&n.nominalProgressMm>0.0&&n.nominalProgressMm<n.nominalLengthMm&&
            (!n.gapMotionEnabled||(n.gapApplied&&n.signedSpeedMmS>0.0));
        // A completed endpoint does not ordinarily earn E5. Only the optional
        // additional phase may discharge there, after fresh physical/zero-send
        // qualification supplied by the RT adapter. No waiting, saturated,
        // historical, flush or final-L check can manufacture periodic work.
        const bool additionalEndpoint=input.additionalEndpointQualified&&
            n.additionalUs!=0ULL&&n.additionalElapsedUs<n.additionalUs&&
            n.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::Additional)&&
            n.additionalTimerState==static_cast<std::uint8_t>(NCMachiningTime::TimerState::Running)&&
            n.phase==NCG76SphereCycle::Phase::Settle&&n.nominalPhase==NCG76SphereCycle::Phase::InitialDepth&&
            n.nominalLengthMm>0.0&&n.nominalProgressMm==n.nominalLengthMm&&
            n.speedMmS==0.0&&n.signedSpeedMmS==0.0&&n.gapMotionEnabled;
        const bool eligible=!input.stationaryNominal&&!seed&&!acknowledged&&!feedback_.suspended&&
            (movingWork||additionalEndpoint)&&
            input.acceptedSendTick!=0ULL&&input.acceptedSendTick==input.tick-1ULL&&
            signedGapSpeed>0.0;
        feedback_.eligible=eligible&&!feedback_.pending;
        // Mode 0 retains successful-forward accounting. Mode 1 uses voltage
        // only for first-discharge confirmation, then continuous elapsed time.
        // Raw servo demand, short protection and D/H inputs remain unchanged.
        feedback_.e5Eligible=feedback_.eligible&&config_.flushMotionEnabled;feedback_.e5ShortBlocked=false;
        bool mayTrigger=feedback_.e5Eligible;
        if(config_.e5WorkTimer.mode==1U){
            EDM386E5::Input timerInput{};timerInput.tick=input.tick;timerInput.deltaUs=delta;
            timerInput.voltageV=frame_.voltageV;
            // EDM280 independently protects at the idle threshold even during
            // machining; the optional machining short detector may be stricter.
            timerInput.shortThresholdV=config_.curve.shortIdleV;
            if(config_.curve.shortMachiningEnabled)
                timerInput.shortThresholdV=(std::max)(timerInput.shortThresholdV,config_.curve.shortMachiningV);
            timerInput.sourceValid=true;
            const bool blocked=holdNow||n.flushActive||input.currentFlushActive||input.timingStopActive||n.finishActive||IsTimeLimitStopping(n);
            timerInput.active=!input.stationaryNominal&&!seed&&!acknowledged&&!blocked&&!feedback_.pending&&
                (feedback_.e5Timer.latched||(IsWorkPhase(n.phase,n.layerIndex)&&n.phase==n.nominalPhase)||additionalEndpoint||n.gapHistoryActive);
            const auto previousStarts=feedback_.e5Timer.startEvents;
            const auto& timer=e5Gate_.Step(timerInput);
            if(!timer.valid){Withdraw(Reason::Input);return feedback_;}
            if(timer.startEvents>previousStarts&&!e5StartUs_){
                e5StartUs_=input.nowUs;elapsedClockUs_=input.nowUs;
            }
            feedback_.e5Timer=timer;feedback_.e5Evidence=e5Gate_.Export();
            const bool elapsedActive=timer.latched&&!input.stationaryNominal&&!blocked&&!feedback_.pending&&config_.flushMotionEnabled;
            // Requalification/seed still owns a zero-output fence. Recover
            // elapsed wall time only on the next healthy callback; do not
            // alter the transport owner's due/pending recovery predicates.
            feedback_.e5Eligible=elapsedActive&&!seed&&!requalifying_&&!acknowledged;
            if(!feedback_.pending&&!seed&&!requalifying_){
                if(!AccrueElapsed(input.nowUs,feedback_.e5Eligible)){Withdraw(Reason::Input);return feedback_;}
            }else if(!config_.flushMotionEnabled&&!feedback_.pending){
                // E6-off remains the explicit pause of periodic flushing.
                elapsedClockUs_=timer.latched?input.nowUs:0ULL;
            }
            mayTrigger=feedback_.e5Eligible;
        }
        if(config_.e5WorkTimer.mode==0U&&feedback_.e5Eligible&&!e5StartUs_)e5StartUs_=input.nowUs;
        if(!feedback_.pending){
            if(config_.e5WorkTimer.mode==0U&&!EDM130::AccrueWork(work_,delta,feedback_.e5Eligible)){Withdraw(Reason::Input);return feedback_;}
            if(workBaseUs_>(std::numeric_limits<std::uint64_t>::max)()-work_.totalWorkUs)
            {Withdraw(Reason::Input);return feedback_;}
            feedback_.simulatedWorkUs=workBaseUs_+work_.totalWorkUs;feedback_.intervalWorkUs=work_.elapsedUs;
            if(work_.due&&mayTrigger){
                if(requestHighWater_==(std::numeric_limits<std::uint64_t>::max)())
                {Withdraw(Reason::Input);return feedback_;}
                if(!EDM130::TriggerWork(work_,input.tick,input.nowUs)){Withdraw(Reason::Input);return feedback_;}
                feedback_.pending=true;feedback_.eligible=false;feedback_.e5Eligible=false;feedback_.requestSequence=++requestHighWater_;feedback_.requestTick=input.tick;
                feedback_.e5DueEvents=requestHighWater_;feedback_.e5LastDueTick=input.tick;e5DueUs_=input.nowUs;
                feedback_.candidateMCS=input.candidateMCS;feedback_.nominalPhase=n.nominalPhase;
                feedback_.layerIndex=n.layerIndex;feedback_.nominalProgressMm=n.nominalProgressMm;
                feedback_.nominalLengthMm=n.nominalLengthMm;
            }
        }
        physicalLineNominal_ = n.phase == NCG76SphereCycle::Phase::InitialDepth &&
            n.nominalPhase == NCG76SphereCycle::Phase::InitialDepth &&
            n.totalLimitUs == 0ULL && n.additionalUs == 0ULL;
        feedback_.valid=true;feedback_.reason=Reason::None;feedback_.sourceReason=EDM88::Reason::None;
        return feedback_;
    }
    bool CanQuarantinePhysicalLine() const noexcept
    {
        return active_ && !transportPaused_ && config_.physicalZManual && config_.parameterFeeds &&
            config_.signedMotionEnabled && feedback_.valid && feedback_.reason == Reason::None &&
            !feedback_.pending && !feedback_.suspended && !work_.due &&
            physicalShortResult_.valid && !physicalShortResult_.feedInhibited &&
            !physicalShortResult_.shortActive && feedback_.signedGapSpeedMmMin >= 0.0 && physicalLineNominal_;
    }
    bool SuspendPhysicalLine() noexcept
    {
        if (!CanQuarantinePhysicalLine()) return false;
        lastRequalifiedTick_=0ULL;
        transportPaused_ = true; work_.previousEligible = false;
        feedback_.valid = feedback_.eligible = feedback_.e5Eligible = false;
        feedback_.suspended = true; feedback_.signedGapSpeedMmMin = 0.0;feedback_.gainRegion=0;feedback_.gainRegionValid=false;
        e5Gate_.SuspendForTransportGap(); feedback_.e5Timer = e5Gate_.Current();
        return true;
    }
    bool RequalifyPhysicalLine(const Input& fresh,std::uint64_t maxTransportGapUs=100000ULL) noexcept
    {
        if (!maxTransportGapUs || maxTransportGapUs > 5000000ULL || !active_ || !transportPaused_ || requalifying_ || fresh.session != feedback_.session ||
            fresh.nominal.session != fresh.session || !physicalLineNominal_ ||
            fresh.nominal.phase != NCG76SphereCycle::Phase::InitialDepth ||
            fresh.nominal.nominalPhase != NCG76SphereCycle::Phase::InitialDepth ||
            fresh.nominal.totalLimitUs != 0ULL || fresh.nominal.additionalUs != 0ULL || !fresh.sourceValid || !fresh.powerInhibited ||
            fresh.cancel || fresh.terminal || fresh.suspended || fresh.acceptedSendTick != 0ULL ||
            fresh.tick <= lastTick_ || fresh.nowUs <= lastUs_ || fresh.nowUs - lastUs_ > maxTransportGapUs ||
            !fresh.physicalFrame.valid || fresh.physicalFrame.sequence <= frame_.sequence ||
            fresh.physicalFrame.capturedAtUs <= frame_.capturedAtUs) return false;
        lastRequalifiedTick_=0ULL;
        requalifying_ = true; requalifyMaxTransportGapUs_=maxTransportGapUs;
        (void)Step(fresh);
        requalifying_ = false; requalifyMaxTransportGapUs_=100000ULL;
        const bool ready = active_ && feedback_.valid && feedback_.reason == Reason::None;
        if (ready) { transportPaused_ = false; lastRequalifiedTick_=lastTick_; }
        return ready;
    }
    // A successful requalification still sends zero. Only its immediate,
    // unconsumed receipt may be suspended again if the final zero ACK is lost.
    // A fresh negative GAP demand has not moved the axis and may be retained.
    bool ResuspendRequalifiedPhysicalLine() noexcept
    {
        const bool receipt=lastRequalifiedTick_ && lastRequalifiedTick_==lastTick_ &&
            lastRequalifiedTick_==feedback_.tick;
        lastRequalifiedTick_=0ULL;
        if (!receipt || !active_ || transportPaused_ || requalifying_ ||
            !config_.physicalZManual || !config_.parameterFeeds || !config_.signedMotionEnabled ||
            !feedback_.valid || feedback_.reason!=Reason::None || feedback_.pending ||
            feedback_.suspended || work_.due || !physicalShortResult_.valid ||
            physicalShortResult_.feedInhibited || physicalShortResult_.shortActive ||
            !std::isfinite(feedback_.signedGapSpeedMmMin) || !physicalLineNominal_) return false;
        transportPaused_=true;work_.previousEligible=false;
        feedback_.valid=feedback_.eligible=feedback_.e5Eligible=false;
        feedback_.suspended=true;feedback_.signedGapSpeedMmMin=0.0;feedback_.gainRegion=0;feedback_.gainRegionValid=false;
        e5Gate_.SuspendForTransportGap();feedback_.e5Timer=e5Gate_.Current();
        return true;
    }
    void Revoke() noexcept{Withdraw(Reason::Cancelled);}
    const Feedback& Current() const noexcept{return feedback_;}

private:
    bool AccrueElapsed(std::uint64_t nowUs,bool counting) noexcept
    {
        if(!nowUs||!EDM130::ValidWorkEvidence(work_)||!EDM130::IsTimed(work_)||
           work_.triggeredEvents!=work_.completedEvents)return false;
        if(!counting){
            elapsedClockUs_=feedback_.e5Timer.latched?nowUs:0ULL;
            work_.previousEligible=false;return true;
        }
        if(!elapsedClockUs_){elapsedClockUs_=nowUs;work_.previousEligible=true;return true;}
        if(nowUs<elapsedClockUs_)return false;
        const auto delta=nowUs-elapsedClockUs_;
        if(work_.totalWorkUs>(std::numeric_limits<std::uint64_t>::max)()-delta)return false;
        work_.totalWorkUs+=delta;
        const auto remaining=work_.workTimeUs-work_.elapsedUs;
        work_.elapsedUs+=delta<remaining?delta:remaining;
        work_.due=work_.elapsedUs==work_.workTimeUs;work_.previousEligible=true;
        elapsedClockUs_=nowUs;
        return EDM130::ValidWorkEvidence(work_);
    }
    bool ResetRoundForHold(bool preservePending,std::uint64_t tick) noexcept
    {
        e5Gate_.Reset(tick);feedback_.e5Timer=e5Gate_.Current();feedback_.e5Evidence=e5Gate_.Export();
        e5StartUs_=e5DueUs_=elapsedClockUs_=0ULL;
        feedback_.intervalWorkUs=0ULL;feedback_.eligible=feedback_.e5Eligible=feedback_.e5ShortBlocked=false;
        if(preservePending)return true;
        const auto archive=feedback_.simulatedWorkUs;
        const auto duration=pendingWorkTimeUs_?pendingWorkTimeUs_:config_.workTimeUs;
        EDM130::WorkEvidence next{};
        if(!EDM130::BeginWork(next,duration))return false;
        if(pendingLiveRecipeFlush_){
            ApplyLiveRecipeFlush(pendingLiveRecipeConfig_);
            pendingLiveRecipeFlush_=false;pendingLiveRecipeConfig_=Config{};
        }
        config_.workTimeUs=feedback_.workTimeUs=duration;pendingWorkTimeUs_=0ULL;
        work_=next;workBaseUs_=archive;
        feedback_.pending=false;feedback_.requestSequence=feedback_.requestTick=0ULL;
        feedback_.candidateMCS=Point8{};feedback_.nominalPhase=NCG76SphereCycle::Phase::Idle;
        feedback_.layerIndex=0U;feedback_.nominalProgressMm=feedback_.nominalLengthMm=0.0;
        return true;
    }
    void ApplyLiveRecipeFlush(const Config& config) noexcept
    {
        config_.jumpHeightMm=config.jumpHeightMm;config_.jumpSpeedPercent=config.jumpSpeedPercent;
        config_.flushMotionEnabled=config.flushMotionEnabled;config_.independentFeedsEnabled=config.independentFeedsEnabled;
        config_.flushProfile=config.flushProfile;
        config_.finalApproachEnabled=config.finalApproachEnabled;config_.finalApproachPercent=config.finalApproachPercent;
        config_.finalApproachDistanceMm=config.finalApproachDistanceMm;
        config_.initialSlowEnabled=config.initialSlowEnabled;config_.initialSlowPercent=config.initialSlowPercent;
        config_.initialSlowDistanceMm=config.initialSlowDistanceMm;
        config_.deepFlushEnabled=config.deepFlushEnabled;config_.deepFlushCycleInterval=config.deepFlushCycleInterval;
        config_.deepFlushHeightMultiplier=config.deepFlushHeightMultiplier;config_.deepFlushHeightMm=config.deepFlushHeightMm;
    }
    bool ApplyWorkTimeUpdate(std::uint64_t workTimeUs) noexcept
    {
        if(!EDM130::ValidWorkTime(workTimeUs)||!EDM130::ValidWorkEvidence(work_)||
           work_.triggeredEvents!=work_.completedEvents||feedback_.pending)return false;
        if(workTimeUs==config_.workTimeUs)return true;
        const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
        if(workBaseUs_>maximum-work_.totalWorkUs)return false;
        const auto total=workBaseUs_+work_.totalWorkUs;
        // WorkEvidence binds every historical trigger to one duration. Start a
        // fresh local timing epoch rather than rewriting those old receipts.
        // Preserve uncapped credit since the last flush, including a shortened
        // target subsequently lengthened before a legal trigger checkpoint.
        const auto credit=work_.totalWorkUs-work_.lastTriggerTotalUs;
        if(credit>total)return false;
        EDM130::WorkEvidence next{};
        if(!EDM130::BeginWork(next,workTimeUs))return false;
        next.totalWorkUs=credit;
        next.elapsedUs=(std::min)(credit,workTimeUs);
        next.due=next.elapsedUs==workTimeUs;
        if(!EDM130::ValidWorkEvidence(next))return false;
        work_=next;workBaseUs_=total-credit;
        config_.workTimeUs=feedback_.workTimeUs=workTimeUs;
        feedback_.simulatedWorkUs=total;feedback_.intervalWorkUs=next.elapsedUs;
        return true;
    }
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
        e5Gate_.Reset(input.tick);feedback_.e5Timer=e5Gate_.Current();feedback_.e5Evidence=e5Gate_.Export();
        e5StartUs_=e5DueUs_=elapsedClockUs_=0ULL;
        feedback_.pending=false;feedback_.eligible=false;feedback_.e5Eligible=false;feedback_.intervalWorkUs=0ULL;
        feedback_.requestSequence=feedback_.requestTick=0ULL;feedback_.candidateMCS=Point8{};
        feedback_.nominalPhase=NCG76SphereCycle::Phase::Idle;feedback_.layerIndex=0U;
        feedback_.nominalProgressMm=feedback_.nominalLengthMm=0.0;
        acknowledged=true;return true;
    }
    static bool FinitePoint(const Point8& p) noexcept
    {for(double value:p)if(!std::isfinite(value))return false;return true;}
    bool IsWorkPhase(NCG76SphereCycle::Phase phase,unsigned layer) const noexcept
    {
        using P=NCG76SphereCycle::Phase;
        // Return/advance power policy is intentionally not released by this
        // first simulation stage. HOLD/resume/initial retreat never accrue E5.
        return phase==P::InitialDepth||phase==P::CircleEntry||phase==P::Orbit||
            (phase==P::ContourNominal&&layer<nominalWorkEndIndex_)||(phase==P::ContourFinish&&layer<finishLegCount_);
    }
    static bool IsTimeLimitStopping(const NCG76SphereCycle::Feedback& n) noexcept
    {
        using P=NCG76SphereCycle::Phase;
        if(n.timeLimitReached||n.timedCompletion)return IsTimedCompletionFinishing(n);
        return (n.phase==P::TimeLimitBrake||n.phase==P::TimeLimitSettle)&&
            n.fault==NCG76SphereCycle::Fault::None&&n.totalLimitUs!=0ULL&&
            n.totalElapsedUs>=n.totalLimitUs&&
            n.totalTimerState==static_cast<std::uint8_t>(NCMachiningTime::TimerState::Expired)&&
            n.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::TimedOut);
    }
    // EDM359: a reached D ends this parent normally after its admitted drain
    // and stopped center return. It remains an observation, never a new GAP
    // machining interval or a completion/position authority for this observer.
    static bool IsTimedCompletionFinishing(const NCG76SphereCycle::Feedback& n) noexcept
    {
        using P=NCG76SphereCycle::Phase;
        if(!n.timeLimitReached||n.fault!=NCG76SphereCycle::Fault::None||!n.totalLimitUs||
           !NCMachiningTime::ValidConfig({n.totalLimitUs,n.additionalUs})||
           n.totalElapsedUs!=n.totalLimitUs||n.additionalElapsedUs>n.additionalUs||
           n.totalTimerState!=static_cast<std::uint8_t>(NCMachiningTime::TimerState::Completed)||
           n.totalPauseReason!=static_cast<std::uint8_t>(NCMachiningTime::PauseReason::None))return false;
        if(n.timedCompletion)return n.phase==P::Complete&&
            n.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::Completed)&&
            !n.holdActive&&!n.held&&!n.flushActive&&!n.gapHistoryActive&&!n.gapApplied&&
            n.speedMmS==0.0&&n.signedSpeedMmS==0.0;
        if(n.timePhase!=static_cast<std::uint8_t>(NCMachiningTime::TimePhase::TimedFinishing))return false;
        if(n.phase==P::TimeLimitBrake||n.phase==P::TimeLimitSettle)return !n.flushActive;
        if(n.phase==P::TimedHistoryReturn)return n.gapHistoryActive&&!n.flushActive&&!n.holdActive;
        if(n.phase==P::TimedReturnCenter)return !n.gapHistoryActive&&!n.flushActive&&!n.holdActive;
        if(n.phase==P::HoldBrake||n.phase==P::HoldSettle||n.phase==P::HeldAtStart)return n.holdActive;
        if(n.flushActive&&(n.phase==P::FlushBrake||n.phase==P::FlushSettle||
           n.phase==P::FlushHoldBrake||NCG76SphereCycle::Detail::IsFlushMotionPhase(n.phase)))return true;
        return n.gapHistoryActive&&(n.phase==P::GapHistory||n.phase==P::GapHistorySettle);
    }
    bool ValidNominal(const NCG76SphereCycle::Feedback& n,bool stationary) const noexcept
    {
        using P=NCG76SphereCycle::Phase;
        const bool timeLimitStop=IsTimeLimitStopping(n);
        if((n.timeLimitReached||n.timedCompletion)&&!timeLimitStop)return false;
        if(stationary){
            if(n.holdRecoveryEnabled&&n.holdActive&&
               (NCG76SphereCycle::Detail::IsRecovery(n.phase)||n.phase==P::HeldAtStart))
                return n.fault==NCG76SphereCycle::Fault::None&&n.nominalPhase==P::InitialDepth&&
                    n.nominalLengthMm==0.0&&n.nominalProgressMm==0.0&&std::isfinite(n.speedMmS)&&n.speedMmS>=0.0&&
                    std::isfinite(n.signedSpeedMmS)&&n.speedMmS==std::fabs(n.signedSpeedMmS)&&!n.gapApplied&&
                    !n.flushActive&&!n.flushCheckpointSaved&&!n.gapHistoryActive&&n.layerIndex==0U&&n.completedLayers==0U;
            const bool restingPhase=n.phase==P::Settle||n.phase==P::HoldSettle||
                n.phase==P::HeldAtStart||n.phase==P::Complete||timeLimitStop;
            return n.fault==NCG76SphereCycle::Fault::None&&restingPhase&&n.nominalPhase==P::InitialDepth&&
                n.nominalLengthMm==0.0&&n.nominalProgressMm==0.0&&n.speedMmS==0.0&&n.signedSpeedMmS==0.0&&
                !n.gapApplied&&!n.flushActive&&!n.flushCheckpointSaved&&!n.gapHistoryActive&&
                n.layerIndex==0U&&n.completedLayers==0U;
        }
        return n.fault==NCG76SphereCycle::Fault::None&&
            (static_cast<unsigned>(n.phase)<=static_cast<unsigned>(P::InspectionJogReturn)||timeLimitStop||
                (n.phase==P::ContourNominal&&n.layerIndex<nominalLegCount_)||
                (n.phase==P::ContourFinish&&n.layerIndex<finishLegCount_))&&
            (!NCG76SphereCycle::Detail::IsRecovery(n.phase)||(n.holdRecoveryEnabled&&n.holdActive))&&
            ((n.nominalPhase>=P::InitialDepth&&n.nominalPhase<=P::AdvanceCenter)||
                (n.nominalPhase==P::ContourNominal&&n.layerIndex<nominalLegCount_&&n.completedLayers==0U)||
                (n.nominalPhase==P::ContourFinish&&n.layerIndex<finishLegCount_&&n.completedLayers==0U))&&
            std::isfinite(n.speedMmS)&&n.speedMmS>=0.0&&std::isfinite(n.nominalProgressMm)&&
            (!n.gapMotionEnabled||(std::isfinite(n.signedSpeedMmS)&&
                n.speedMmS==std::fabs(n.signedSpeedMmS)))&&
            std::isfinite(n.nominalLengthMm)&&n.nominalLengthMm>0.0&&n.nominalProgressMm>=0.0&&
            n.nominalProgressMm<=n.nominalLengthMm;
    }
    void Withdraw(Reason reason) noexcept
    {
        // A normal, adjacent parent may inherit the last qualified accounting
        // latch through the separately proved RT continuation. Every other
        // withdrawal destroys that evidence immediately.
        if(reason!=Reason::Terminal)feedback_.e5Evidence=EDM386E5::Evidence{};
        e5Gate_.Withdraw(feedback_.tick);feedback_.e5Timer=e5Gate_.Current();feedback_.e5Eligible=feedback_.e5ShortBlocked=false;
        if(feedback_.probePhase==ProbePhase::Retreat||feedback_.probePhase==ProbePhase::Return)
            feedback_.probePhase=ProbePhase::Failed;
        active_=false;transportPaused_=requalifying_=physicalLineNominal_=false;pendingWorkTimeUs_=0ULL;
        pendingLiveRecipeFlush_=false;pendingLiveRecipeConfig_=Config{};
        lastRequalifiedTick_=0ULL;requalifyMaxTransportGapUs_=100000ULL;qualifier_.Cancel();feedback_.valid=feedback_.eligible=feedback_.pending=feedback_.suspended=false;
        feedback_.reason=reason;feedback_.signedGapSpeedMmMin=feedback_.voltageV=0.0;
        feedback_.gainRegion=0;feedback_.gainRegionValid=false;feedback_.referenceV=0.0;
        // Terminal handoff retains only clock/accounting evidence, under the
        // same separate adjacency proof as e5Evidence. Every cancellation,
        // RESET, source failure or revoke destroys it.
        if(reason!=Reason::Terminal||config_.e5WorkTimer.mode!=1U){
            e5StartUs_=elapsedClockUs_=0ULL;
        }
        e5DueUs_=0ULL;holdRoundReset_=false;
        feedback_.selectedFeedRole=FeedRole::Unavailable;
        feedback_.selectedScaleMmPerVoltMin=feedback_.selectedOverridePercent=0.0;
        feedback_.sourceSequence=feedback_.sourceCapturedAtUs=feedback_.requestSequence=feedback_.requestTick=0ULL;
        feedback_.candidateMCS=Point8{};feedback_.nominalPhase=NCG76SphereCycle::Phase::Idle;
        feedback_.layerIndex=0U;feedback_.nominalProgressMm=feedback_.nominalLengthMm=0.0;
    }
    unsigned finishLegCount_=4U,nominalLegCount_=25U,nominalWorkEndIndex_=24U;
    Config config_{};Config pendingLiveRecipeConfig_{};bool pendingLiveRecipeFlush_=false;
    Feedback feedback_{};EDM130::WorkEvidence work_{};
    EDM386E5::Gate e5Gate_{};
    EDM88::Qualifier qualifier_{};EDM88::Frame frame_{};
    EDMGapServo::ShortDetector physicalShort_{};EDMGapServo::ShortResult physicalShortResult_{};
    std::uint64_t physicalShortSequence_=0ULL;
    std::uint64_t sessionHighWater_=0ULL,lastTick_=0ULL,lastUs_=0ULL,firstUs_=0ULL;
    std::uint64_t requestHighWater_=0ULL,workBaseUs_=0ULL,pendingWorkTimeUs_=0ULL;
    std::uint64_t e5StartUs_=0ULL,e5DueUs_=0ULL,elapsedClockUs_=0ULL;
    bool holdRoundReset_=false;
    std::uint64_t probeElapsedUs_=0ULL;
    double probeLengthMm_=0.0;
    bool probeSeeded_=false,probeEntered_=false;
    std::uint64_t lastRequalifiedTick_=0ULL,requalifyMaxTransportGapUs_=100000ULL;
    bool active_=false,haveClock_=false,transportPaused_=false,requalifying_=false,physicalLineNominal_=false;
};
static_assert(std::is_trivially_copyable<Config>::value,"P50 SIM config is an immutable value");
static_assert(sizeof(Config)<=960U,"Role-selected native-axis feed config remains bounded");
static_assert(std::is_trivially_copyable<Feedback>::value&&sizeof(Feedback)<=384U,"P50 SIM feedback stays bounded");
static_assert(!MotionExecutionReleased&&!DischargeExecutionReleased,"P50 SIM observer cannot authorize outputs");
}
