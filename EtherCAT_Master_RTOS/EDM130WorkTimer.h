#pragma once
#include <cstdint>
#include <cmath>
#include <limits>
#include <cstring>
#include "EDMGapServo.h"

// EDM130 bounded value contract. No motion, recipe-catalog or discharge access.
namespace EDM130
{
constexpr std::uint64_t MinimumWorkTimeUs=50000ULL, MaximumWorkTimeUs=10000000ULL;
// EDM132 unloaded-fixture admission fence; the catalog retains its native range.
constexpr std::uint64_t MinimumJumpHeightScaled=100ULL, MaximumJumpHeightScaled=5000ULL;
// EDM136 captures only the installed process curve shape. Bounded path speed,
// voltage-reference policy and physical authorization remain separate contracts.
struct Curve
{
    std::uint32_t processRevision=0U,machineProfileId=0U;
    std::uint64_t tuningGeneration=0ULL;
    double positiveGain[3]{},negativeGain[3]{},positiveBreakV[2]{},negativeBreakV[2]{};
    double deadbandV=0.0;
};
inline std::uint64_t CurveDoubleBits(double value) noexcept
{
    std::uint64_t bits=0ULL;static_assert(sizeof(bits)==sizeof(value),"Curve double identity is 64 bits");
    std::memcpy(&bits,&value,sizeof(bits));return bits;
}
inline bool SameCurve(const Curve& a,const Curve& b) noexcept
{
    if(a.processRevision!=b.processRevision||a.machineProfileId!=b.machineProfileId||
        a.tuningGeneration!=b.tuningGeneration||CurveDoubleBits(a.deadbandV)!=CurveDoubleBits(b.deadbandV))return false;
    for(unsigned i=0U;i<3U;++i)
        if(CurveDoubleBits(a.positiveGain[i])!=CurveDoubleBits(b.positiveGain[i])||
            CurveDoubleBits(a.negativeGain[i])!=CurveDoubleBits(b.negativeGain[i]))return false;
    for(unsigned i=0U;i<2U;++i)
        if(CurveDoubleBits(a.positiveBreakV[i])!=CurveDoubleBits(b.positiveBreakV[i])||
            CurveDoubleBits(a.negativeBreakV[i])!=CurveDoubleBits(b.negativeBreakV[i]))return false;
    return true;
}
inline bool EmptyCurve(const Curve& curve) noexcept{return SameCurve(curve,Curve{});}
inline void ApplyCurveShape(EDMGapServo::Profile& profile,const Curve& curve) noexcept
{
    profile.revision=curve.processRevision;
    for(unsigned i=0U;i<3U;++i){profile.positiveGain[i]=curve.positiveGain[i];profile.negativeGain[i]=curve.negativeGain[i];}
    for(unsigned i=0U;i<2U;++i){profile.positiveBreakV[i]=curve.positiveBreakV[i];profile.negativeBreakV[i]=curve.negativeBreakV[i];}
    profile.deadbandV=curve.deadbandV;
}
inline bool ValidCurve(const Curve& curve,double referenceV=35.0) noexcept
{
    if(!curve.processRevision||!curve.machineProfileId||!curve.tuningGeneration||!std::isfinite(referenceV))return false;
    for(unsigned i=0U;i<3U;++i)
        if(!std::isfinite(curve.positiveGain[i])||curve.positiveGain[i]<0.0||std::signbit(curve.positiveGain[i])||
            !std::isfinite(curve.negativeGain[i])||curve.negativeGain[i]<0.0||std::signbit(curve.negativeGain[i]))return false;
    if(!std::isfinite(curve.deadbandV)||curve.deadbandV<0.0||std::signbit(curve.deadbandV))return false;
    EDMGapServo::Profile profile{};ApplyCurveShape(profile,curve);
    profile.cuttingScaleMmPerVoltMin=0.001;
    // Both input endpoints cover every intervening magnitude for nonnegative
    // slopes. Full override prevents E8 from concealing arithmetic overflow.
    return EDMGapServo::ValidateProfile(profile)&&
        EDMGapServo::Evaluate(profile,0.0,referenceV,100.0).valid&&
        EDMGapServo::Evaluate(profile,300.0,referenceV,100.0).valid;
}
static_assert(sizeof(Curve)==104U,"EDM136 bounded immutable process-curve capture");
// EDM137 freezes confirmed E11/E13/E19/E20/E12 values and their two native
// pump speed bases. A canonical zero tail preserves every prior recipe.
struct FlushStages
{
    std::uint16_t finalSpeedFieldId=0U,approachFieldId=0U,initialSpeedFieldId=0U;
    std::uint16_t initialDistanceFieldId=0U,polarityFieldId=0U;
    std::uint32_t finalSpeedPercentScaled=0U,initialSpeedPercentScaled=0U,polarity=0U;
    std::uint64_t approachDistanceScaled=0ULL,initialDistanceScaled=0ULL; // mm * 1e6.
    double finalApproachMmPerMin=0.0,initialSlowMmPerMin=0.0; // Native Pump_Stg_Spd / Pump_Low_Spd.
};
inline bool SameFlushStages(const FlushStages& a,const FlushStages& b) noexcept
{
    return a.finalSpeedFieldId==b.finalSpeedFieldId&&a.approachFieldId==b.approachFieldId&&
        a.initialSpeedFieldId==b.initialSpeedFieldId&&a.initialDistanceFieldId==b.initialDistanceFieldId&&
        a.polarityFieldId==b.polarityFieldId&&a.finalSpeedPercentScaled==b.finalSpeedPercentScaled&&
        a.initialSpeedPercentScaled==b.initialSpeedPercentScaled&&a.polarity==b.polarity&&
        a.approachDistanceScaled==b.approachDistanceScaled&&a.initialDistanceScaled==b.initialDistanceScaled&&
        CurveDoubleBits(a.finalApproachMmPerMin)==CurveDoubleBits(b.finalApproachMmPerMin)&&
        CurveDoubleBits(a.initialSlowMmPerMin)==CurveDoubleBits(b.initialSlowMmPerMin);
}
inline bool EmptyFlushStages(const FlushStages& s) noexcept{return SameFlushStages(s,FlushStages{});}
inline double FrozenFinalApproachSpeedMmMin(const FlushStages& s) noexcept
{return s.finalApproachMmPerMin*(static_cast<double>(s.finalSpeedPercentScaled)/100000000.0);}
inline double FrozenInitialSlowSpeedMmMin(const FlushStages& s) noexcept
{return s.initialSlowMmPerMin*(static_cast<double>(s.initialSpeedPercentScaled)/100000000.0);}
inline double FrozenApproachDistanceMm(const FlushStages& s) noexcept
{return static_cast<double>(s.approachDistanceScaled)/1000000.0;}
inline double FrozenInitialSlowDistanceMm(const FlushStages& s) noexcept
{return static_cast<double>(s.initialDistanceScaled)/1000000.0;}
inline std::uint32_t FrozenPolarity(const FlushStages& s) noexcept{return s.polarity;}
inline bool ValidFlushStages(const FlushStages& s) noexcept
{
    const std::uint16_t fields[]={s.finalSpeedFieldId,s.approachFieldId,s.initialSpeedFieldId,
        s.initialDistanceFieldId,s.polarityFieldId};
    for(unsigned i=0U;i<5U;++i){if(!fields[i])return false;
        for(unsigned j=0U;j<i;++j)if(fields[i]==fields[j])return false;}
    if(s.finalSpeedPercentScaled<10000000U||s.finalSpeedPercentScaled>100000000U||
        s.initialSpeedPercentScaled<1000000U||s.initialSpeedPercentScaled>100000000U||
        s.approachDistanceScaled>100000000ULL||s.initialDistanceScaled>900000000ULL||s.polarity>1U||
        !std::isfinite(s.finalApproachMmPerMin)||s.finalApproachMmPerMin<=0.0||
        !std::isfinite(s.initialSlowMmPerMin)||s.initialSlowMmPerMin<=0.0)return false;
    const double finalSpeed=FrozenFinalApproachSpeedMmMin(s),initialSpeed=FrozenInitialSlowSpeedMmMin(s);
    // The staged runtime consumes mm/s and E7 admits a minimum 50% override.
    // Reject a positive mm/min value that would disappear in that conversion.
    return std::isfinite(finalSpeed)&&finalSpeed>0.0&&std::isfinite(initialSpeed)&&initialSpeed>0.0&&
        finalSpeed*0.5/60.0>0.0&&initialSpeed*0.5/60.0>0.0;
}
static_assert(sizeof(FlushStages)==56U,"EDM137 immutable flush stages stay bounded");
struct Recipe
{
    std::uint32_t catalogRevision=0U,definitionRevision=0U,profileId=0U,tableId=0U,tableRevision=0U;
    std::uint16_t eCode=0U,fieldId=0U;
    std::uint64_t generation=0ULL,workTimeUs=0ULL;
    // EDM131: optional immutable E7 override. Both zero preserve EDM130.
    std::uint16_t speedFieldId=0U;
    std::uint64_t jumpSpeedPercentScaled=0ULL; // Percent in recipe fixed-point units (1e6).
    // EDM132: optional absolute height in millimetres, captured with E5/E7.
    std::uint16_t heightFieldId=0U;
    std::uint64_t jumpHeightScaled=0ULL; // Millimetres in recipe fixed-point units (1e6).
    // EDM133: explicit E21/E22 binding, including a disabled zero interval.
    std::uint16_t intervalFieldId=0U,multiplierFieldId=0U;
    std::uint32_t deepFlushCycleInterval=0U;
    std::uint64_t deepFlushHeightMultiplierScaled=0ULL; // Multiplier in 1e6 units.
    // EDM134: optional immutable E8 machining override, captured with all five fields.
    std::uint16_t machiningFieldId=0U;
    std::uint32_t machiningSpeedPercentScaled=0U; // Percent in recipe fixed-point units (1e6).
    // EDM135: optional immutable E9 reference voltage, captured with all six fields.
    std::uint16_t referenceFieldId=0U;
    std::uint32_t referenceVoltageScaled=0U; // Volts in recipe fixed-point units (1e6).
    // EDM136: an all-zero tail preserves all earlier timed profiles.
    Curve curve{};
    FlushStages flushStages{};
};
inline bool SameRecipe(const Recipe& a,const Recipe& b) noexcept
{
    return a.catalogRevision==b.catalogRevision&&a.definitionRevision==b.definitionRevision&&
        a.profileId==b.profileId&&a.tableId==b.tableId&&a.tableRevision==b.tableRevision&&
        a.eCode==b.eCode&&a.fieldId==b.fieldId&&a.generation==b.generation&&a.workTimeUs==b.workTimeUs&&
        a.speedFieldId==b.speedFieldId&&a.jumpSpeedPercentScaled==b.jumpSpeedPercentScaled&&
        a.heightFieldId==b.heightFieldId&&a.jumpHeightScaled==b.jumpHeightScaled&&
        a.intervalFieldId==b.intervalFieldId&&a.multiplierFieldId==b.multiplierFieldId&&
        a.deepFlushCycleInterval==b.deepFlushCycleInterval&&
        a.deepFlushHeightMultiplierScaled==b.deepFlushHeightMultiplierScaled&&
        a.machiningFieldId==b.machiningFieldId&&
        a.machiningSpeedPercentScaled==b.machiningSpeedPercentScaled&&
        a.referenceFieldId==b.referenceFieldId&&a.referenceVoltageScaled==b.referenceVoltageScaled&&
        SameCurve(a.curve,b.curve)&&SameFlushStages(a.flushStages,b.flushStages);
}
inline bool EmptyRecipe(const Recipe& r) noexcept {return SameRecipe(r,Recipe{});}
inline bool ValidWorkTime(std::uint64_t us) noexcept
{return us>=MinimumWorkTimeUs&&us<=MaximumWorkTimeUs;}
inline bool ValidSpeedPercent(std::uint64_t percentScaled) noexcept
{return percentScaled>=50000000ULL&&percentScaled<=100000000ULL;}
inline bool HasBoundSpeed(const Recipe& r) noexcept
{return r.speedFieldId!=0U&&r.jumpSpeedPercentScaled!=0ULL;}
inline bool ValidJumpHeight(std::uint64_t heightScaled) noexcept
{return heightScaled>=MinimumJumpHeightScaled&&heightScaled<=MaximumJumpHeightScaled;}
inline bool HasBoundHeight(const Recipe& r) noexcept
{return r.heightFieldId!=0U&&r.jumpHeightScaled!=0ULL;}
inline bool HasBoundDeep(const Recipe& r) noexcept
{return r.intervalFieldId!=0U&&r.multiplierFieldId!=0U;}
inline bool HasBoundMachiningSpeed(const Recipe& r) noexcept
{return r.machiningFieldId!=0U;}
inline bool ValidMachiningSpeedPercent(std::uint32_t percentScaled) noexcept
{return percentScaled>=1000000U&&percentScaled<=100000000U;}
inline double FrozenMachiningOverridePercent(const Recipe& r) noexcept
{return HasBoundMachiningSpeed(r)?static_cast<double>(r.machiningSpeedPercentScaled)/1000000.0:100.0;}
inline bool HasBoundReferenceVoltage(const Recipe& r) noexcept
{return r.referenceFieldId!=0U;}
inline bool ValidReferenceVoltage(std::uint32_t voltageScaled) noexcept
{return voltageScaled>=15000000U&&voltageScaled<=200000000U;}
inline double FrozenGapReferenceVoltageV(const Recipe& r) noexcept
{return HasBoundReferenceVoltage(r)?static_cast<double>(r.referenceVoltageScaled)/1000000.0:35.0;}
inline bool HasBoundCurve(const Recipe& r) noexcept
{return r.curve.processRevision!=0U;}
inline bool HasBoundFlushStages(const Recipe& r) noexcept
{return r.flushStages.finalSpeedFieldId!=0U;}
inline double FrozenFinalApproachSpeedMmMin(const Recipe& r) noexcept
{return FrozenFinalApproachSpeedMmMin(r.flushStages);}
inline double FrozenInitialSlowSpeedMmMin(const Recipe& r) noexcept
{return FrozenInitialSlowSpeedMmMin(r.flushStages);}
inline double FrozenApproachDistanceMm(const Recipe& r) noexcept
{return FrozenApproachDistanceMm(r.flushStages);}
inline double FrozenInitialSlowDistanceMm(const Recipe& r) noexcept
{return FrozenInitialSlowDistanceMm(r.flushStages);}
inline std::uint32_t FrozenPolarity(const Recipe& r) noexcept{return FrozenPolarity(r.flushStages);}
inline bool ValidBoundFlushStages(const Recipe& r) noexcept
{
    if(EmptyFlushStages(r.flushStages))return true;
    if(!HasBoundCurve(r)||!ValidFlushStages(r.flushStages))return false;
    const std::uint16_t oldFields[]={r.fieldId,r.speedFieldId,r.heightFieldId,r.intervalFieldId,
        r.multiplierFieldId,r.machiningFieldId,r.referenceFieldId};
    const std::uint16_t newFields[]={r.flushStages.finalSpeedFieldId,r.flushStages.approachFieldId,
        r.flushStages.initialSpeedFieldId,r.flushStages.initialDistanceFieldId,r.flushStages.polarityFieldId};
    for(auto fresh:newFields)for(auto old:oldFields)if(fresh==old)return false;
    return true;
}
inline bool ValidDeepFlushValues(std::uint64_t heightScaled,std::uint32_t interval,
    std::uint64_t multiplierScaled) noexcept
{
    if(!ValidJumpHeight(heightScaled)||interval>20U||multiplierScaled>20000000ULL)return false;
    if(!interval)return true;
    if(!multiplierScaled||heightScaled>(std::numeric_limits<std::uint64_t>::max)()/multiplierScaled)return false;
    const std::uint64_t product=heightScaled*multiplierScaled;
    // Preserve the full rational result, including fractional nanometres.
    return product>=MinimumJumpHeightScaled*1000000ULL&&product<=MaximumJumpHeightScaled*1000000ULL;
}
inline bool DeepFlushScaledValues(double baseHeightMm,double multiplier,
    std::uint64_t& heightScaled,std::uint64_t& multiplierScaled) noexcept
{
    if(!std::isfinite(baseHeightMm)||baseHeightMm<0.0001||baseHeightMm>0.005||
        !std::isfinite(multiplier)||multiplier<0.0||multiplier>20.0)return false;
    const auto height=static_cast<std::uint64_t>(std::llround(baseHeightMm*1000000.0));
    const auto factor=static_cast<std::uint64_t>(std::llround(multiplier*1000000.0));
    if(static_cast<double>(height)/1000000.0!=baseHeightMm||
        static_cast<double>(factor)/1000000.0!=multiplier)return false;
    heightScaled=height;multiplierScaled=factor;return true;
}
inline bool ValidDeepFlushConfig(double baseHeightMm,std::uint32_t interval,double multiplier,bool bound) noexcept
{
    if(!bound)return interval==0U&&multiplier==0.0;
    std::uint64_t height=0ULL,factor=0ULL;
    return DeepFlushScaledValues(baseHeightMm,multiplier,height,factor)&&ValidDeepFlushValues(height,interval,factor);
}
inline double SelectedJumpHeightMm(double baseHeightMm,std::uint32_t interval,double multiplier,
    std::uint64_t oneBasedEventIndex) noexcept
{
    if(!interval||!oneBasedEventIndex||oneBasedEventIndex%interval!=0ULL)return baseHeightMm;
    std::uint64_t height=0ULL,factor=0ULL;
    if(!DeepFlushScaledValues(baseHeightMm,multiplier,height,factor)||!ValidDeepFlushValues(height,interval,factor))return 0.0;
    // All admitted products are exact integers below 2^53. One final divide
    // avoids an outward one-ULP result at the exact native height fences.
    return static_cast<double>(height*factor)/1000000000000.0;
}
inline bool ValidRecipe(const Recipe& r) noexcept
{
    return r.catalogRevision&&r.definitionRevision&&r.profileId&&r.tableId&&r.tableId<=99999999U&&
        r.tableRevision&&r.eCode&&r.eCode<=1000U&&r.fieldId&&r.generation&&ValidWorkTime(r.workTimeUs)&&
        ((!r.speedFieldId&&!r.jumpSpeedPercentScaled)||
            (r.speedFieldId&&r.speedFieldId!=r.fieldId&&ValidSpeedPercent(r.jumpSpeedPercentScaled)))&&
        ((!r.heightFieldId&&!r.jumpHeightScaled)||
            (r.heightFieldId&&r.heightFieldId!=r.fieldId&&r.heightFieldId!=r.speedFieldId&&
                HasBoundSpeed(r)&&ValidJumpHeight(r.jumpHeightScaled)))&&
        ((!r.intervalFieldId&&!r.multiplierFieldId&&!r.deepFlushCycleInterval&&!r.deepFlushHeightMultiplierScaled)||
            (HasBoundDeep(r)&&HasBoundHeight(r)&&r.intervalFieldId!=r.multiplierFieldId&&
                r.intervalFieldId!=r.fieldId&&r.intervalFieldId!=r.heightFieldId&&r.intervalFieldId!=r.speedFieldId&&
                r.multiplierFieldId!=r.fieldId&&r.multiplierFieldId!=r.heightFieldId&&r.multiplierFieldId!=r.speedFieldId&&
                ValidDeepFlushValues(r.jumpHeightScaled,r.deepFlushCycleInterval,r.deepFlushHeightMultiplierScaled)))&&
        ((!r.machiningFieldId&&!r.machiningSpeedPercentScaled)||
            (HasBoundMachiningSpeed(r)&&HasBoundDeep(r)&&ValidMachiningSpeedPercent(r.machiningSpeedPercentScaled)&&
                r.machiningFieldId!=r.fieldId&&r.machiningFieldId!=r.speedFieldId&&
                r.machiningFieldId!=r.heightFieldId&&r.machiningFieldId!=r.intervalFieldId&&
                r.machiningFieldId!=r.multiplierFieldId))&&
        ((!r.referenceFieldId&&!r.referenceVoltageScaled)||
            (HasBoundReferenceVoltage(r)&&HasBoundMachiningSpeed(r)&&ValidReferenceVoltage(r.referenceVoltageScaled)&&
                r.referenceFieldId!=r.fieldId&&r.referenceFieldId!=r.speedFieldId&&
                r.referenceFieldId!=r.heightFieldId&&r.referenceFieldId!=r.intervalFieldId&&
                r.referenceFieldId!=r.multiplierFieldId&&r.referenceFieldId!=r.machiningFieldId))&&
        (EmptyCurve(r.curve)||(HasBoundCurve(r)&&HasBoundReferenceVoltage(r)&&
            r.curve.machineProfileId==r.profileId&&ValidCurve(r.curve,FrozenGapReferenceVoltageV(r))))&&
        ValidBoundFlushStages(r);
}
inline double FrozenJumpHeightMm(const Recipe& r) noexcept
{return HasBoundHeight(r)?static_cast<double>(r.jumpHeightScaled)/1000000.0:0.0;}
inline double FrozenDeepMultiplier(const Recipe& r) noexcept
{return HasBoundDeep(r)?static_cast<double>(r.deepFlushHeightMultiplierScaled)/1000000.0:0.0;}
inline bool IsDeepFlushEvent(const Recipe& r,std::uint64_t oneBasedEventIndex) noexcept
{return HasBoundDeep(r)&&r.deepFlushCycleInterval&&oneBasedEventIndex&&oneBasedEventIndex%r.deepFlushCycleInterval==0ULL;}
inline double EffectiveJumpHeightMm(const Recipe& r,std::uint64_t oneBasedEventIndex) noexcept
{
    const double base=FrozenJumpHeightMm(r);
    return HasBoundDeep(r)?SelectedJumpHeightMm(base,r.deepFlushCycleInterval,FrozenDeepMultiplier(r),oneBasedEventIndex):base;
}
// EDM131 uses the bounded J50 flush maximum. E7 is its percentage override;
// 50..100% therefore maps to the existing .25..0.5 command-path speed bounds.
inline double FrozenSpeedRatio(const Recipe& r) noexcept
{return HasBoundSpeed(r)?0.5*(static_cast<double>(r.jumpSpeedPercentScaled)/100000000.0):0.0;}
inline bool MatchesFlushSpeed(const Recipe& r,double speedRatio) noexcept
{return ValidRecipe(r)&&(!HasBoundSpeed(r)||speedRatio==FrozenSpeedRatio(r));}
inline bool MatchesFlushHeight(const Recipe& r,double strokeRatio,std::uint64_t dwellUs) noexcept
{return ValidRecipe(r)&&(!HasBoundHeight(r)||(strokeRatio==0.1&&dwellUs==100000ULL));}
struct WorkEvidence
{
    std::uint64_t workTimeUs=0ULL,elapsedUs=0ULL,totalWorkUs=0ULL;
    std::uint64_t triggeredEvents=0ULL,completedEvents=0ULL;
    std::uint64_t lastTriggerElapsedUs=0ULL,lastTriggerTotalUs=0ULL,lastTriggerTick=0ULL,lastTriggerUs=0ULL;
    std::uint64_t lastCompleteTick=0ULL,lastCompleteUs=0ULL;
    bool previousEligible=false,due=false;
};
inline bool SameWorkEvidence(const WorkEvidence& a,const WorkEvidence& b) noexcept
{
    return a.workTimeUs==b.workTimeUs&&a.elapsedUs==b.elapsedUs&&a.totalWorkUs==b.totalWorkUs&&
        a.triggeredEvents==b.triggeredEvents&&a.completedEvents==b.completedEvents&&
        a.lastTriggerElapsedUs==b.lastTriggerElapsedUs&&a.lastTriggerTotalUs==b.lastTriggerTotalUs&&
        a.lastTriggerTick==b.lastTriggerTick&&a.lastTriggerUs==b.lastTriggerUs&&
        a.lastCompleteTick==b.lastCompleteTick&&a.lastCompleteUs==b.lastCompleteUs&&
        a.previousEligible==b.previousEligible&&a.due==b.due;
}
inline bool IsTimed(const WorkEvidence& e) noexcept{return e.workTimeUs!=0ULL;}
inline bool ValidWorkEvidence(const WorkEvidence& e) noexcept
{
    if(!IsTimed(e))return SameWorkEvidence(e,WorkEvidence{});
    if(!ValidWorkTime(e.workTimeUs)||e.elapsedUs>e.workTimeUs||e.elapsedUs>e.totalWorkUs||
        e.due!=(e.elapsedUs==e.workTimeUs)||e.completedEvents>e.triggeredEvents||
        e.triggeredEvents-e.completedEvents>1ULL||e.lastTriggerTotalUs>e.totalWorkUs)return false;
    if(!e.triggeredEvents)return !e.completedEvents&&!e.lastTriggerElapsedUs&&!e.lastTriggerTotalUs&&
        !e.lastTriggerTick&&!e.lastTriggerUs&&!e.lastCompleteTick&&!e.lastCompleteUs;
    if(e.lastTriggerElapsedUs!=e.workTimeUs||e.lastTriggerTotalUs<e.workTimeUs||
        !e.lastTriggerTick||!e.lastTriggerUs)return false;
    if(e.triggeredEvents>e.totalWorkUs/e.workTimeUs)return false;
    if(!e.completedEvents) {if(e.lastCompleteTick||e.lastCompleteUs)return false;}
    else if(!e.lastCompleteTick||!e.lastCompleteUs)return false;
    if(e.triggeredEvents==e.completedEvents)
        return e.lastCompleteTick>e.lastTriggerTick&&e.lastCompleteUs>e.lastTriggerUs;
    return e.due&&!e.previousEligible&&e.lastCompleteTick<e.lastTriggerTick&&e.lastCompleteUs<e.lastTriggerUs;
}
inline bool BeginWork(WorkEvidence& e,std::uint64_t duration) noexcept
{
    if(!ValidWorkTime(duration)||!ValidWorkEvidence(e)||(IsTimed(e)&&e.workTimeUs!=duration))return false;
    e.workTimeUs=duration;e.previousEligible=false;return true;
}
inline bool AccrueWork(WorkEvidence& e,std::uint64_t deltaUs,bool eligible) noexcept
{
    if(!ValidWorkEvidence(e)||!IsTimed(e)||e.triggeredEvents!=e.completedEvents)return false;
    if(eligible&&e.previousEligible) {
        if(deltaUs>1500ULL||e.totalWorkUs>(std::numeric_limits<std::uint64_t>::max)()-deltaUs)return false;
        // Total includes overshoot and legal-target deferral; the due interval saturates.
        e.totalWorkUs+=deltaUs;
        const auto remaining=e.workTimeUs-e.elapsedUs;
        e.elapsedUs+=deltaUs<remaining?deltaUs:remaining;
    }
    e.previousEligible=eligible;e.due=e.elapsedUs==e.workTimeUs;return ValidWorkEvidence(e);
}
inline bool TriggerWork(WorkEvidence& e,std::uint64_t tick,std::uint64_t nowUs) noexcept
{
    if(!ValidWorkEvidence(e)||!IsTimed(e)||!e.due||!tick||!nowUs||e.triggeredEvents!=e.completedEvents||
        e.triggeredEvents==(std::numeric_limits<std::uint64_t>::max)()||
        tick<=e.lastCompleteTick||nowUs<=e.lastCompleteUs)return false;
    ++e.triggeredEvents;e.lastTriggerElapsedUs=e.elapsedUs;e.lastTriggerTotalUs=e.totalWorkUs;
    e.lastTriggerTick=tick;e.lastTriggerUs=nowUs;e.previousEligible=false;return ValidWorkEvidence(e);
}
inline bool CompleteWork(WorkEvidence& e,std::uint64_t tick,std::uint64_t nowUs) noexcept
{
    if(!ValidWorkEvidence(e)||!IsTimed(e)||e.triggeredEvents!=e.completedEvents+1ULL||
        tick<=e.lastTriggerTick||nowUs<=e.lastTriggerUs)return false;
    ++e.completedEvents;e.elapsedUs=0ULL;e.due=false;e.previousEligible=false;
    e.lastCompleteTick=tick;e.lastCompleteUs=nowUs;return ValidWorkEvidence(e);
}
static_assert(sizeof(Recipe)<=264U&&sizeof(WorkEvidence)<=128U,"EDM130 bounded immutable time evidence");
}
