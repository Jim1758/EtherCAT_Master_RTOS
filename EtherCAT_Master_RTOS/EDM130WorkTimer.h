#pragma once
#include "EDM149VoltageMotion.h"
#include <cstdint>
#include <cstddef>
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
constexpr std::uint16_t VisibleFlushProfile158=1U, VisibleFlushProfile159=2U, VisibleFlushProfile167=3U, VisibleFlushProfile173=4U;
// EDM175 B2 follows retained path history inside the ordinary six-axis
// envelope. Its profile identity never grants the B1 visible-Z permissions.
constexpr std::uint16_t G76B2Profile175=5U;
constexpr std::uint16_t G76B3Profile176=6U;
constexpr std::uint64_t G76B3HeightScaled176=2000000ULL;
constexpr std::uint16_t G76B4Profile177=7U;
constexpr std::uint64_t G76B4HeightScaled177=2000000ULL;
// E214: center commissioning admits either the one-mm test stroke or the
// retained two-mm stroke. The selected E6 stays part of the frozen recipe.
constexpr std::uint64_t G76CenterShortHeightScaled214=1000000ULL;
// Center architecture-only stroke. Recipe permission remains explicitly
// selected by L233, L239 or L240 with diagnostic source3.
constexpr std::uint64_t G76ArchitectureHeightScaled233=200000ULL;
// EDM192 B0 is a bounded native tangent excursion, not a visible-Z grant.
constexpr std::uint16_t G76B0Profile192=8U;
constexpr std::uint64_t G76B0HeightScaled192=2000000ULL;
constexpr std::uint64_t G76B2HeightScaled175=1000ULL;
constexpr std::uint64_t VisibleFlushHeightScaled158=5000000ULL, VisibleFlushWorkUs158=500000ULL;
constexpr std::uint64_t VisibleFlushHeightScaled159=2000000ULL, VisibleFlushWorkUs159=500000ULL;
constexpr std::uint64_t VisibleFlushHeightScaled167=2000000ULL, VisibleFlushWorkUs167=500000ULL;
constexpr std::uint64_t VisibleFlushHeightScaled173=2000000ULL;
constexpr std::uint64_t VisibleFlushMinimumWorkUs173=250000ULL, VisibleFlushMaximumWorkUs173=2000000ULL;
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
    // EDM139: 0 = physical AD control, 1 = explicit simulated control stimulus.
    // Legacy source 1 retains physical monitoring. Sources 2/3 use software
    // steps/wave without AD; source 4 uses physical AD with discharge inhibited.
    std::uint16_t diagnosticControlSource=0U;
    // EDM156 seals the bounded admitted NC plan in existing alignment space.
    // Both zero retain all earlier fixed fixtures; no process table field is added.
    std::uint16_t planRows=0U,planMovementMask=0U;
    std::uint64_t jumpSpeedPercentScaled=0ULL; // Percent in recipe fixed-point units (1e6).
    // EDM132: optional absolute height in millimetres, captured with E5/E7.
    std::uint16_t heightFieldId=0U;
    // Explicit commissioning opt-in in existing alignment space; old offsets stay fixed.
    std::uint16_t commissioningProfile=0U;
    // EDM163: literal G180 H1 opts the bounded mainline into path HOLD retreat.
    // Zero preserves ordinary HOLD; this immutable value uses prior padding.
    std::uint16_t holdRetreatMode=0U;
    // EDM165/168: literal H1 L1 selects the first global E5; L2 selects the
    // first B1 E5; L3 selects an eligible B2 E5 after crossing retained history.
    // EDM200 also admits P294 L3 without H for the sealed G76 B2 test.
    // EDM217 uses P296/P298 L1 without H for a nominal GAP work checkpoint.
    // EDM218 uses P296/P298 L2 without H for an in-flight center outbound checkpoint.
    // EDM219/220 use L3/L4 for axial return/final anchor-return checkpoints.
    // EDM221 uses L5 for the first detached machining-anchor-to-center leg.
    // EDM222/224/225/226 use L6-L9 for apex, both center and final anchor dwells.
    // EDM227 uses L10 for initial settlement before capturing the local E5 path.
    // EDM228 uses L11 for the first observed moving E5 braking phase.
    // EDM230 uses L230 as an independent two-E5 commissioning selector,
    // without an automatic HOLD. Earlier L1-L11 keep their one-event meaning.
    // EDM231 uses L231 for one ordinary HOLD on the second E5 outbound leg.
    // EDM232 selects one HOLD on the second E5 axial return with L232.
    // NC submits one existing HOLD transaction; START remains operator-only.
    std::uint16_t flushHoldTest=0U;
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
inline bool IsVisibleFlush(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile158;}
inline bool IsVisibleFlush159(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile159;}
inline bool IsVisibleFlush167(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile167;}
inline bool IsVisibleFlush173(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173;}
// EDM250 selects only the P12 square B1 fixture. The selector is immutable
// recipe identity, not an automatic HOLD request or a new process field.
inline bool IsG76SquareB1_250(const Recipe& r) noexcept
{return IsVisibleFlush173(r)&&r.flushHoldTest==250U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==VisibleFlushHeightScaled173&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM251 uses the existing B2 retained-path distance, with a separate
// immutable selector. It grants no B1 lift or automatic HOLD checkpoint.
inline bool IsG76SquareB2_251(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&r.flushHoldTest==251U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B2HeightScaled175&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM252 binds the existing B3 center route to one square parent. E6 is
// native distance opposite the signed machining axis from its nominal center.
inline bool IsG76SquareB3_252(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&r.flushHoldTest==252U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B3HeightScaled176&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM253 retains B4's diagonal center join. E6 includes the first axial
// contribution and remains total native withdrawal from the nominal center.
inline bool IsG76SquareB4_253(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&r.flushHoldTest==253U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B4HeightScaled177&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
inline bool IsG76SquareMode12(const Recipe& r) noexcept
{return IsG76SquareB1_250(r)||IsG76SquareB2_251(r)||IsG76SquareB3_252(r)||IsG76SquareB4_253(r);}
// EDM255 keeps P13's confirmed diamond separate from P12 recipe identity.
inline bool IsG76DiamondB1_255(const Recipe& r) noexcept
{return IsVisibleFlush173(r)&&r.flushHoldTest==255U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==VisibleFlushHeightScaled173&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM256 uses the same retained-path B2 distance as EDM251, with P13 identity.
inline bool IsG76DiamondB2_256(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&r.flushHoldTest==256U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B2HeightScaled175&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM257 binds the existing B3 center route to the confirmed P13 diamond.
inline bool IsG76DiamondB3_257(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&r.flushHoldTest==257U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B3HeightScaled176&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM258 preserves B4's diagonal center join and total axial withdrawal.
inline bool IsG76DiamondB4_258(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&r.flushHoldTest==258U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B4HeightScaled177&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM260 uses separate immutable identities for the one-mm test family.
// The selector identifies shape; the existing profile identifies B1/B3/B4.
inline bool IsG76OneMmSelector260(unsigned selector) noexcept
{return selector==260U||selector==261U||selector==262U||selector==263U;}
inline unsigned G76OneMmMode260(unsigned selector) noexcept
{return selector==260U?11U:selector==261U?12U:selector==262U?13U:selector==263U?16U:0U;}
inline bool IsG76OneMm260(const Recipe& r) noexcept
{return IsG76OneMmSelector260(r.flushHoldTest)&&
    (r.commissioningProfile==VisibleFlushProfile173||r.commissioningProfile==G76B3Profile176||r.commissioningProfile==G76B4Profile177)&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==1000000ULL&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
inline bool IsG76OneMmB1_260(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173&&IsG76OneMm260(r);}
inline bool IsG76OneMmB3_260(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&IsG76OneMm260(r);}
inline bool IsG76OneMmB4_260(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&IsG76OneMm260(r);}
// P16 B2 retains its existing bounded native-path distance, independently of
// the one-mm B1/B3/B4 family. L264 identifies only this polygon recipe.
inline bool IsG76PolygonB2_261(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&r.flushHoldTest==264U&&
    r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76B2HeightScaled175&&
    r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
    r.planRows==25U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
    !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;}
// EDM263 keeps the target-plane expansion's 26-child identity separate from
// every established 25-child taper recipe. Selectors alone grant no authority.
inline bool IsG76ExpansionSelector263(unsigned selector) noexcept
{return selector>=265U&&selector<=272U;}
inline unsigned G76ExpansionMode263(unsigned selector) noexcept
{return selector==265U||selector==269U?21U:selector==266U||selector==270U?22U:
    selector==267U||selector==271U?23U:selector==268U||selector==272U?26U:0U;}
inline bool IsG76Expansion263(const Recipe& r) noexcept
{
    const bool b2=r.flushHoldTest>=269U&&r.flushHoldTest<=272U;
    return IsG76ExpansionSelector263(r.flushHoldTest)&&
        (b2?r.commissioningProfile==G76B2Profile175:
            r.commissioningProfile==VisibleFlushProfile173||r.commissioningProfile==G76B3Profile176||r.commissioningProfile==G76B4Profile177)&&
        r.workTimeUs==500000ULL&&r.jumpHeightScaled==(b2?G76B2HeightScaled175:1000000ULL)&&
        r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
        r.planRows==26U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
        !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;
}
inline bool IsG76ExpansionB1_263(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173&&IsG76Expansion263(r);}
inline bool IsG76ExpansionB2_263(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&IsG76Expansion263(r);}
inline bool IsG76ExpansionB3_263(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&IsG76Expansion263(r);}
inline bool IsG76ExpansionB4_263(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&IsG76Expansion263(r);}
// Fixed-radius advancing contours have the same bounded child count as
// expansion, but carry their own immutable recipe identities and semantics.
inline bool IsG76FixedRadiusSelector266(unsigned selector) noexcept
{return selector>=273U&&selector<=280U;}
inline unsigned G76FixedRadiusMode266(unsigned selector) noexcept
{return selector==273U||selector==277U?1U:selector==274U||selector==278U?2U:
    selector==275U||selector==279U?3U:selector==276U||selector==280U?6U:0U;}
inline bool IsG76FixedRadius266(const Recipe& r) noexcept
{
    const bool b2=r.flushHoldTest>=277U&&r.flushHoldTest<=280U;
    return IsG76FixedRadiusSelector266(r.flushHoldTest)&&
        (b2?r.commissioningProfile==G76B2Profile175:
            r.commissioningProfile==VisibleFlushProfile173||r.commissioningProfile==G76B3Profile176||r.commissioningProfile==G76B4Profile177)&&
        r.workTimeUs==500000ULL&&r.jumpHeightScaled==(b2?G76B2HeightScaled175:1000000ULL)&&
        r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
        r.planRows==26U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
        !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;
}
inline bool IsG76FixedRadiusB1_266(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173&&IsG76FixedRadius266(r);}
inline bool IsG76FixedRadiusB2_266(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&IsG76FixedRadius266(r);}
inline bool IsG76FixedRadiusB3_266(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&IsG76FixedRadius266(r);}
inline bool IsG76FixedRadiusB4_266(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&IsG76FixedRadius266(r);}
// WORK admission has its own literal recipe identity; the established 266
// selectors retain their neutral-WORK execution contract.
inline bool IsG76FixedRadiusWorkSelector267(unsigned selector) noexcept
{return selector>=281U&&selector<=288U;}
inline unsigned G76FixedRadiusWorkMode267(unsigned selector) noexcept
{return selector==281U||selector==285U?1U:selector==282U||selector==286U?2U:
    selector==283U||selector==287U?3U:selector==284U||selector==288U?6U:0U;}
inline bool IsG76FixedRadiusWork267(const Recipe& r) noexcept
{
    const bool b2=r.flushHoldTest>=285U&&r.flushHoldTest<=288U;
    return IsG76FixedRadiusWorkSelector267(r.flushHoldTest)&&
        (b2?r.commissioningProfile==G76B2Profile175:
            r.commissioningProfile==VisibleFlushProfile173||r.commissioningProfile==G76B3Profile176||r.commissioningProfile==G76B4Profile177)&&
        r.workTimeUs==500000ULL&&r.jumpHeightScaled==(b2?G76B2HeightScaled175:1000000ULL)&&
        r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
        r.planRows==26U&&r.planMovementMask==7U&&!r.holdRetreatMode&&
        !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;
}
inline bool IsG76FixedRadiusWorkB1_267(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173&&IsG76FixedRadiusWork267(r);}
inline bool IsG76FixedRadiusWorkB2_267(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175&&IsG76FixedRadiusWork267(r);}
inline bool IsG76FixedRadiusWorkB3_267(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176&&IsG76FixedRadiusWork267(r);}
inline bool IsG76FixedRadiusWorkB4_267(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177&&IsG76FixedRadiusWork267(r);}
// EDM274: one authored L374 selects the shared full-contour framework.
// NC seals the adjacent original P in an internal selector; the external
// tag alone is never a valid recipe. Existing selectors and Recipe ABI stay fixed.
constexpr unsigned G76SharedShortArmTag274=374U, G76SharedShortSelectorBase274=27400U;
constexpr std::uint64_t G76SharedShortHeightScaled274=200000ULL;
inline bool IsG76SharedShortMode274(unsigned modeP) noexcept
{return modeP==4U||modeP==5U||modeP==7U||modeP==14U||modeP==15U||modeP==17U||modeP==24U||modeP==25U||modeP==27U;}
inline unsigned G76SharedShortSelector274(unsigned modeP) noexcept
{return IsG76SharedShortMode274(modeP)?G76SharedShortSelectorBase274+modeP:0U;}
inline unsigned G76SharedShortMode274(unsigned selector) noexcept
{return selector>G76SharedShortSelectorBase274&&IsG76SharedShortMode274(selector-G76SharedShortSelectorBase274)?
    selector-G76SharedShortSelectorBase274:0U;}
inline bool IsG76SharedShortSelector274(unsigned selector) noexcept
{return G76SharedShortMode274(selector)!=0U;}
inline bool IsG76SharedShort274(const Recipe& r) noexcept
{
    const unsigned mode=G76SharedShortMode274(r.flushHoldTest);
    return mode&&(r.commissioningProfile==VisibleFlushProfile173||
        r.commissioningProfile==G76B3Profile176||r.commissioningProfile==G76B4Profile177)&&
        r.workTimeUs==500000ULL&&r.jumpHeightScaled==G76SharedShortHeightScaled274&&
        r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
        r.planRows==((mode<10U||mode>=20U)?26U:25U)&&r.planMovementMask==7U&&!r.holdRetreatMode&&
        !r.deepFlushCycleInterval&&r.deepFlushHeightMultiplierScaled==1000000ULL;
}
inline bool IsG76SharedShortB1_274(const Recipe& r) noexcept
{return r.commissioningProfile==VisibleFlushProfile173&&IsG76SharedShort274(r);}
inline unsigned G76PlanRows(const Recipe& r) noexcept
{return IsG76Expansion263(r)||IsG76FixedRadius266(r)||IsG76FixedRadiusWork267(r)||
    (IsG76SharedShort274(r)&&(G76SharedShortMode274(r.flushHoldTest)<10U||G76SharedShortMode274(r.flushHoldTest)>=20U))?26U:25U;}
inline bool IsG76NonCircular(const Recipe& r) noexcept
{return IsG76SquareMode12(r)||IsG76DiamondB1_255(r)||IsG76DiamondB2_256(r)||IsG76DiamondB3_257(r)||IsG76DiamondB4_258(r)||IsG76PolygonB2_261(r);}
inline bool IsG76CommissioningRecipe(const Recipe& r) noexcept
{return IsG76NonCircular(r)||IsG76OneMm260(r)||IsG76Expansion263(r)||IsG76FixedRadius266(r)||IsG76FixedRadiusWork267(r)||IsG76SharedShort274(r);}
inline unsigned G76PlanMode(const Recipe& r) noexcept
{return IsG76SharedShort274(r)?G76SharedShortMode274(r.flushHoldTest):IsG76FixedRadiusWork267(r)?G76FixedRadiusWorkMode267(r.flushHoldTest):IsG76FixedRadius266(r)?G76FixedRadiusMode266(r.flushHoldTest):IsG76Expansion263(r)?G76ExpansionMode263(r.flushHoldTest):IsG76OneMm260(r)?G76OneMmMode260(r.flushHoldTest):
    IsG76PolygonB2_261(r)?16U:
    (IsG76DiamondB1_255(r)||IsG76DiamondB2_256(r)||IsG76DiamondB3_257(r)||IsG76DiamondB4_258(r))?13U:IsG76SquareMode12(r)?12U:11U;}
inline bool IsG76B2Flush175(const Recipe& r) noexcept
{return r.commissioningProfile==G76B2Profile175;}
// The automatic crossed-history checkpoint is only the existing L3 meaning
// on the bounded G76 B2 profile; it never enables manual path retreat.
inline bool IsG76B2FlushHold200(const Recipe& r) noexcept
{return IsG76B2Flush175(r)&&r.flushHoldTest==3U&&!r.holdRetreatMode;}
inline bool IsG76B3Flush176(const Recipe& r) noexcept
{return r.commissioningProfile==G76B3Profile176;}
inline bool IsG76B4Flush177(const Recipe& r) noexcept
{return r.commissioningProfile==G76B4Profile177;}
// EDM217: one nominal GAP pause checkpoint, before the first E5 event.
// This flag grants no in-flight center pause, retreat, JOG or auto-resume.
inline bool IsG76CenterHold217(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==1U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM218: one pause on the captured center-to-top axial leg of the first E5.
// L2 selects the test checkpoint; it does not grant JOG or automatic START.
inline bool IsG76CenterFlushHold218(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==2U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM219: L3 selects the captured top-to-center axial return of the first E5.
// The outward L2 checkpoint remains unchanged; neither permits JOG or auto-START.
inline bool IsG76CenterReturnHold219(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==3U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold219(const Recipe& r) noexcept
{return IsG76CenterFlushHold218(r)||IsG76CenterReturnHold219(r);}
// EDM220: L4 selects the captured center-to-machining-anchor final E5 leg.
// This checkpoint retains the same backend and grants no JOG or automatic START.
inline bool IsG76CenterAnchorHold220(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==4U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold220(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold219(r)||IsG76CenterAnchorHold220(r);}
// EDM221: L5 selects the already detached machining-anchor-to-center E5 leg.
// It never admits the initial main/local backend transition or grants JOG.
inline bool IsG76CenterStartHold221(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==5U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold221(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold220(r)||IsG76CenterStartHold221(r);}
// EDM222: L6 selects the captured apex dwell of the first center-mode E5.
// HOLD discards partial dwell qualification; fresh START must qualify it again.
inline bool IsG76CenterTopDwellHold222(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==6U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold222(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold221(r)||IsG76CenterTopDwellHold222(r);}
// EDM224: L7 selects the first center waypoint dwell, before axial Outbound.
// This does not admit CenterReturnDwell or credit time spent under HOLD.
inline bool IsG76CenterDwellHold224(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==7U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold224(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold222(r)||IsG76CenterDwellHold224(r);}
// EDM225: L8 selects the captured center-return dwell, before final ToAnchor.
// The captured top and event remain frozen; START qualifies the full dwell again.
inline bool IsG76CenterReturnDwellHold225(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==8U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold225(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold224(r)||IsG76CenterReturnDwellHold225(r);}
// EDM226: L9 selects the final dwell at the original machining anchor.
// The event stays pending until fresh START and a complete new dwell.
inline bool IsG76CenterFinalDwellHold226(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==9U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold226(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold225(r)||IsG76CenterFinalDwellHold226(r);}
// EDM227: L10 selects initial settlement before E5 captures any local path.
// The main path stays attached; fresh START must qualify the full dwell.
inline bool IsG76CenterInitialDwellHold227(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==10U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold227(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold226(r)||IsG76CenterInitialDwellHold227(r);}
// EDM228: L11 pauses E5 braking on the still-attached main path.
// HOLD reaches rest before START can continue to the full initial dwell.
inline bool IsG76CenterBrakeHold228(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==11U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold228(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold227(r)||IsG76CenterBrakeHold228(r);}
// Keep the commissioning envelope/profile unchanged. L230 permits two E5
// events without an automatic HOLD inside the same sealed G76 parent.
inline bool IsG76CenterRepeat230(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==230U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM231 shares the two-event cap but requires the second outbound HOLD
// receipt before completion. It is never the no-checkpoint L230 selector.
inline bool IsG76CenterSecondFlushHold231(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==231U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold231(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold228(r)||IsG76CenterSecondFlushHold231(r);}
// EDM232 retains the two-event cap and selects only the second axial return.
// It does not change L231's outbound checkpoint or grant retreat/JOG.
inline bool IsG76CenterSecondReturnHold232(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==232U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline bool IsG76CenterAnyFlushHold232(const Recipe& r) noexcept
{return IsG76CenterAnyFlushHold231(r)||IsG76CenterSecondReturnHold232(r);}
// EDM233 opts into two independently retired G76 parents in one fixed NC.
// Each parent retains the one-event limit; this selector never requests HOLD.
inline bool IsG76ParentSequence233(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==233U&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM239 retires four alternating center parents. Its independent selector
// keeps L233's two-parent semantics and freezes the actual E6=0.2 mm source.
inline bool IsG76CenterSequence239(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==239U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM240 binds the center parents of an explicitly authored E6-switching
// sequence. The selected actual height remains 0.2 mm; L239 stays separate.
inline bool IsG76CenterSequence240(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==240U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM241 adds native B2 to the mixed sequence while retaining an independent
// center selector. Only B3/B4 use the actual E6=0.2 mm center architecture.
inline bool IsG76CenterSequence241(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==241U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM242 combines equal-height handoffs with one authored E6 bridge.
// Its center selector retains actual E6=0.2 mm and the one-event limit.
inline bool IsG76CenterSequence242(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==242U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM243 retains both E6 bridge receipts across the intervening equal-height
// center handoff. Its independent center selector captures actual E6=0.2 mm.
inline bool IsG76CenterSequence243(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==243U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM244 repeats an identical center profile under a fresh parent identity.
// The independent selector preserves actual E6=0.2 mm and one E5 per parent.
inline bool IsG76CenterSequence244(const Recipe& r) noexcept
{return (IsG76B3Flush176(r)||IsG76B4Flush177(r))&&r.flushHoldTest==244U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM245 repeats the ordinary history parent around a native B2 parent.
// Its single center parent still freezes the actual E6=0.2 mm recipe.
inline bool IsG76CenterSequence245(const Recipe& r) noexcept
{return IsG76B3Flush176(r)&&r.flushHoldTest==245U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM246 exercises the same ordinary/native handoffs with a B4 center parent.
// Its selector accepts only the actual E6=0.2 mm B4 recipe.
inline bool IsG76CenterSequence246(const Recipe& r) noexcept
{return IsG76B4Flush177(r)&&r.flushHoldTest==246U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM247 isolates consecutive native B2 parents around a B3 center parent.
// Its selector accepts only the actual E6=0.2 mm B3 recipe.
inline bool IsG76CenterSequence247(const Recipe& r) noexcept
{return IsG76B3Flush176(r)&&r.flushHoldTest==247U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
// EDM248 pairs consecutive native B2 parents with a B4 center parent.
// Its selector accepts only the actual E6=0.2 mm B4 recipe.
inline bool IsG76CenterSequence248(const Recipe& r) noexcept
{return IsG76B4Flush177(r)&&r.flushHoldTest==248U&&
    r.jumpHeightScaled==G76ArchitectureHeightScaled233&&
    !r.holdRetreatMode&&r.diagnosticControlSource==3U;}
inline unsigned G76CenterHoldEventIndex(const Recipe& r) noexcept
{return (IsG76CenterSecondFlushHold231(r)||IsG76CenterSecondReturnHold232(r))?2U:1U;}
inline unsigned G76EventLimit(const Recipe& r) noexcept
{return (IsG76CenterRepeat230(r)||IsG76CenterSecondFlushHold231(r)||IsG76CenterSecondReturnHold232(r))?2U:1U;}
inline bool IsG76B0Flush192(const Recipe& r) noexcept
{return r.commissioningProfile==G76B0Profile192;}
inline bool IsG76Gap173(const Recipe& r) noexcept
{return IsVisibleFlush173(r)||IsG76B2Flush175(r)||IsG76B3Flush176(r)||IsG76B4Flush177(r)||IsG76B0Flush192(r);}
inline bool IsOneShotCommissioning(const Recipe& r) noexcept
{return IsVisibleFlush(r)||IsVisibleFlush159(r)||IsVisibleFlush167(r)||IsG76Gap173(r);}
// Explicit L words remain opt-in: omitted L is stored as zero, never L0.
inline bool ValidFlushHoldSelection(unsigned arm,unsigned value) noexcept
{return value==1U?((EDM149::IsFlushHoldTestArm(arm)&&!EDM149::IsManualVoltageTestArm(arm))||arm==296U||arm==298U):
    (arm==274U&&(value==2U||value==3U))||(arm==294U&&value==3U)||
    (arm==292U&&value==250U)||
    ((arm==292U||arm==296U||arm==298U)&&IsG76OneMmSelector260(value))||(arm==294U&&value==264U)||
    ((arm==292U||arm==296U||arm==298U)&&value>=265U&&value<=268U)||(arm==294U&&value>=269U&&value<=272U)||
    ((arm==292U||arm==296U||arm==298U)&&value>=273U&&value<=276U)||(arm==294U&&value>=277U&&value<=280U)||
    ((arm==292U||arm==296U||arm==298U)&&value>=281U&&value<=284U)||(arm==294U&&value>=285U&&value<=288U)||
    ((arm==292U||arm==296U||arm==298U)&&value==G76SharedShortArmTag274)||
    (arm==296U&&value==245U)||(arm==298U&&value==246U)||(arm==296U&&value==247U)||(arm==298U&&value==248U)||
    ((arm==296U||arm==298U)&&(value==2U||value==3U||value==4U||value==5U||value==6U||value==7U||value==8U||value==9U||value==10U||value==11U||value==230U||value==231U||value==232U||value==233U||value==239U||value==240U||value==241U||value==242U||value==243U||value==244U));}
inline unsigned FlushHoldTargetMode(unsigned selector) noexcept
{return selector==2U?1U:selector==3U?2U:3U;}
inline bool SameRecipe(const Recipe& a,const Recipe& b) noexcept
{
    return a.catalogRevision==b.catalogRevision&&a.definitionRevision==b.definitionRevision&&
        a.profileId==b.profileId&&a.tableId==b.tableId&&a.tableRevision==b.tableRevision&&
        a.eCode==b.eCode&&a.fieldId==b.fieldId&&a.generation==b.generation&&a.workTimeUs==b.workTimeUs&&
        a.speedFieldId==b.speedFieldId&&a.diagnosticControlSource==b.diagnosticControlSource&&
        a.planRows==b.planRows&&a.planMovementMask==b.planMovementMask&&a.jumpSpeedPercentScaled==b.jumpSpeedPercentScaled&&
        a.heightFieldId==b.heightFieldId&&a.commissioningProfile==b.commissioningProfile&&
        a.holdRetreatMode==b.holdRetreatMode&&a.flushHoldTest==b.flushHoldTest&&a.jumpHeightScaled==b.jumpHeightScaled&&
        a.intervalFieldId==b.intervalFieldId&&a.multiplierFieldId==b.multiplierFieldId&&
        a.deepFlushCycleInterval==b.deepFlushCycleInterval&&
        a.deepFlushHeightMultiplierScaled==b.deepFlushHeightMultiplierScaled&&
        a.machiningFieldId==b.machiningFieldId&&
        a.machiningSpeedPercentScaled==b.machiningSpeedPercentScaled&&
        a.referenceFieldId==b.referenceFieldId&&a.referenceVoltageScaled==b.referenceVoltageScaled&&
        SameCurve(a.curve,b.curve)&&SameFlushStages(a.flushStages,b.flushStages);
}
inline bool UsesDiagnosticControl(const Recipe& r) noexcept
{return r.diagnosticControlSource==1U;}
inline bool UsesSoftwareVoltage(const Recipe& r) noexcept
{return EDM149::IsSoftwareSource(r.diagnosticControlSource);}
inline bool IsMotionTest(const Recipe& r) noexcept
{return EDM149::IsTestSource(r.diagnosticControlSource);}
inline bool RequiresPhysicalVoltage(const Recipe& r) noexcept
{return !UsesSoftwareVoltage(r);}
inline bool HasBoundProgramPlan(const Recipe& r) noexcept
{return r.planRows!=0U&&r.planMovementMask!=0U;}
inline bool ValidProgramPlan(const Recipe& r) noexcept
{
    // EDM197: B0/B2 may freeze a pure-XY nominal path. The plan owner
    // independently derives this mask from the compensated native geometry.
    if(IsG76Gap173(r))return IsMotionTest(r)&&r.planRows==G76PlanRows(r)&&
        (r.planMovementMask==7U||((IsG76B0Flush192(r)||IsG76B2Flush175(r))&&r.planMovementMask==3U));
    return (!r.planRows&&!r.planMovementMask)||(IsMotionTest(r)&&r.planRows>=1U&&r.planRows<=16U&&
        r.planMovementMask<=63U&&(r.planMovementMask&7U)!=0U&&
        (!IsVisibleFlush(r)||(r.planRows==1U&&r.planMovementMask==4U))&&
        (!IsVisibleFlush159(r)||(r.planRows==3U&&r.planMovementMask==7U))&&
        (!IsVisibleFlush167(r)||(r.planRows==3U&&r.planMovementMask==63U)));
}
inline unsigned ProgramRows(const Recipe& r) noexcept
{return HasBoundProgramPlan(r)?r.planRows:16U;}
inline unsigned ProgramMovementMask(const Recipe& r) noexcept
{return HasBoundProgramPlan(r)?r.planMovementMask:63U;}
inline bool EmptyRecipe(const Recipe& r) noexcept {return SameRecipe(r,Recipe{});}
inline bool ValidWorkTime(std::uint64_t us) noexcept
{return us>=MinimumWorkTimeUs&&us<=MaximumWorkTimeUs;}
inline bool ValidSpeedPercent(std::uint64_t percentScaled) noexcept
{return percentScaled>=50000000ULL&&percentScaled<=100000000ULL;}
// E210: only B3/B4 center commissioning accepts the frozen E7 override.
// Other G76 modes retain their established exact-100-percent contract.
inline bool ValidG76FlushSpeedPercent(unsigned commissioningProfile,std::uint64_t percentScaled) noexcept
{return (commissioningProfile==G76B3Profile176||commissioningProfile==G76B4Profile177)?
    ValidSpeedPercent(percentScaled):percentScaled==100000000ULL;}
inline bool HasBoundSpeed(const Recipe& r) noexcept
{return r.speedFieldId!=0U&&r.jumpSpeedPercentScaled!=0ULL;}
inline bool ValidG76CenterHeightScaled(std::uint64_t heightScaled) noexcept
{return heightScaled==G76ArchitectureHeightScaled233||heightScaled==G76CenterShortHeightScaled214||heightScaled==G76B3HeightScaled176;}
inline bool ValidJumpHeight(std::uint64_t heightScaled,unsigned commissioningProfile=0U) noexcept
{return commissioningProfile<=G76B0Profile192&&(commissioningProfile==G76B0Profile192?heightScaled==G76B0HeightScaled192:
    (commissioningProfile==G76B4Profile177||commissioningProfile==G76B3Profile176)?ValidG76CenterHeightScaled(heightScaled):
    commissioningProfile==G76B2Profile175?heightScaled==G76B2HeightScaled175:
    commissioningProfile==VisibleFlushProfile173?heightScaled==VisibleFlushHeightScaled173:
    commissioningProfile==VisibleFlushProfile167?heightScaled==VisibleFlushHeightScaled167:
    commissioningProfile==VisibleFlushProfile159?heightScaled==VisibleFlushHeightScaled159:
    commissioningProfile==VisibleFlushProfile158?heightScaled==VisibleFlushHeightScaled158:
    heightScaled>=MinimumJumpHeightScaled&&heightScaled<=MaximumJumpHeightScaled);}
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
    std::uint64_t multiplierScaled,unsigned commissioningProfile=0U) noexcept
{
    if(commissioningProfile>G76B0Profile192)return false;
    if(commissioningProfile==G76B0Profile192)return heightScaled==G76B0HeightScaled192&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==G76B4Profile177||commissioningProfile==G76B3Profile176)
        return ValidG76CenterHeightScaled(heightScaled)&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==G76B2Profile175)return heightScaled==G76B2HeightScaled175&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==VisibleFlushProfile173)return heightScaled==VisibleFlushHeightScaled173&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==VisibleFlushProfile167)return heightScaled==VisibleFlushHeightScaled167&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==VisibleFlushProfile159)return heightScaled==VisibleFlushHeightScaled159&&interval==0U&&multiplierScaled==1000000ULL;
    if(commissioningProfile==VisibleFlushProfile158)return heightScaled==VisibleFlushHeightScaled158&&interval==0U&&multiplierScaled==1000000ULL;
    if(!ValidJumpHeight(heightScaled)||interval>20U||multiplierScaled>20000000ULL)return false;
    if(!interval)return true;
    if(!multiplierScaled||heightScaled>(std::numeric_limits<std::uint64_t>::max)()/multiplierScaled)return false;
    const std::uint64_t product=heightScaled*multiplierScaled;
    // Preserve the full rational result, including fractional nanometres.
    return product>=MinimumJumpHeightScaled*1000000ULL&&product<=MaximumJumpHeightScaled*1000000ULL;
}
inline bool DeepFlushScaledValues(double baseHeightMm,double multiplier,
    std::uint64_t& heightScaled,std::uint64_t& multiplierScaled,unsigned commissioningProfile=0U) noexcept
{
    if(commissioningProfile>G76B0Profile192||!std::isfinite(baseHeightMm)||
        (commissioningProfile==G76B2Profile175?baseHeightMm!=0.001:
        (commissioningProfile==G76B3Profile176||commissioningProfile==G76B4Profile177)?(baseHeightMm!=0.2&&baseHeightMm!=1.0&&baseHeightMm!=2.0):
        (commissioningProfile==VisibleFlushProfile159||commissioningProfile==VisibleFlushProfile167||commissioningProfile==VisibleFlushProfile173||commissioningProfile==G76B0Profile192)?baseHeightMm!=2.0:commissioningProfile==VisibleFlushProfile158?baseHeightMm!=5.0:(baseHeightMm<0.0001||baseHeightMm>0.005))||
        !std::isfinite(multiplier)||multiplier<0.0||multiplier>20.0)return false;
    const auto height=static_cast<std::uint64_t>(std::llround(baseHeightMm*1000000.0));
    const auto factor=static_cast<std::uint64_t>(std::llround(multiplier*1000000.0));
    if(static_cast<double>(height)/1000000.0!=baseHeightMm||
        static_cast<double>(factor)/1000000.0!=multiplier)return false;
    heightScaled=height;multiplierScaled=factor;return true;
}
inline bool ValidDeepFlushConfig(double baseHeightMm,std::uint32_t interval,double multiplier,bool bound,unsigned commissioningProfile=0U) noexcept
{
    if(commissioningProfile>G76B0Profile192||(commissioningProfile&&!bound))return false;
    if(!bound)return interval==0U&&multiplier==0.0;
    std::uint64_t height=0ULL,factor=0ULL;
    return DeepFlushScaledValues(baseHeightMm,multiplier,height,factor,commissioningProfile)&&
        ValidDeepFlushValues(height,interval,factor,commissioningProfile);
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
    // Short geometry is not a generic commissioning permission. Only the
    // frozen center architecture selectors can authorize their exact E6.
    return (r.jumpHeightScaled!=G76ArchitectureHeightScaled233||IsG76SharedShort274(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r)||IsG76CenterSequence247(r))&&
        r.holdRetreatMode<=1U&&(r.flushHoldTest<=3U||IsG76CommissioningRecipe(r)||IsG76CenterAnchorHold220(r)||IsG76CenterStartHold221(r)||IsG76CenterTopDwellHold222(r)||IsG76CenterDwellHold224(r)||IsG76CenterReturnDwellHold225(r)||IsG76CenterFinalDwellHold226(r)||IsG76CenterInitialDwellHold227(r)||IsG76CenterBrakeHold228(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r)||IsG76CenterSequence247(r)||IsG76CenterSecondFlushHold231(r)||IsG76CenterSecondReturnHold232(r))&&
        (r.flushHoldTest<=1U||((!r.commissioningProfile||IsG76CommissioningRecipe(r)||IsG76B2FlushHold200(r)||IsG76CenterAnyFlushHold232(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r)||IsG76CenterSequence247(r))&&r.diagnosticControlSource==3U))&&
        (!r.flushHoldTest||(IsMotionTest(r)&&HasBoundProgramPlan(r)&&
            (IsG76CommissioningRecipe(r)||IsG76B2FlushHold200(r)||IsG76CenterHold217(r)||IsG76CenterAnyFlushHold232(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r)||IsG76CenterSequence247(r)||(r.holdRetreatMode==1U&&
             (!r.commissioningProfile||IsVisibleFlush159(r)||IsVisibleFlush167(r))))))&&
        (!r.holdRetreatMode||(IsMotionTest(r)&&HasBoundProgramPlan(r)&&
            (!r.commissioningProfile||IsVisibleFlush159(r)||IsVisibleFlush167(r))))&&
        r.commissioningProfile<=G76B0Profile192&&
        (!IsG76B0Flush192(r)||(HasBoundHeight(r)&&r.jumpHeightScaled==G76B0HeightScaled192&&
            r.workTimeUs>=VisibleFlushMinimumWorkUs173&&r.workTimeUs<=VisibleFlushMaximumWorkUs173&&
            r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
            !r.holdRetreatMode&&!r.flushHoldTest))&&
        (!IsVisibleFlush173(r)||(HasBoundHeight(r)&&(r.jumpHeightScaled==VisibleFlushHeightScaled173||IsG76OneMmB1_260(r)||IsG76ExpansionB1_263(r)||(IsG76FixedRadiusB1_266(r)||IsG76FixedRadiusWorkB1_267(r)||IsG76SharedShortB1_274(r)))&&
            r.workTimeUs>=VisibleFlushMinimumWorkUs173&&r.workTimeUs<=VisibleFlushMaximumWorkUs173&&
            r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
            !r.holdRetreatMode&&(!r.flushHoldTest||IsG76SquareB1_250(r)||IsG76DiamondB1_255(r)||IsG76OneMmB1_260(r)||IsG76ExpansionB1_263(r)||(IsG76FixedRadiusB1_266(r)||IsG76FixedRadiusWorkB1_267(r)||IsG76SharedShortB1_274(r)))))&&
        (!IsG76B3Flush176(r)||(HasBoundHeight(r)&&ValidG76CenterHeightScaled(r.jumpHeightScaled)&&
            r.workTimeUs>=VisibleFlushMinimumWorkUs173&&r.workTimeUs<=VisibleFlushMaximumWorkUs173&&
            ValidG76FlushSpeedPercent(r.commissioningProfile,r.jumpSpeedPercentScaled)&&r.diagnosticControlSource==3U&&
            !r.holdRetreatMode&&(!r.flushHoldTest||IsG76SquareB3_252(r)||IsG76DiamondB3_257(r)||IsG76OneMmB3_260(r)||IsG76ExpansionB3_263(r)||(IsG76FixedRadiusB3_266(r)||IsG76FixedRadiusWorkB3_267(r)||IsG76SharedShort274(r))||IsG76CenterHold217(r)||IsG76CenterAnyFlushHold232(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence247(r))))&&
        (!IsG76B4Flush177(r)||(HasBoundHeight(r)&&ValidG76CenterHeightScaled(r.jumpHeightScaled)&&
            r.workTimeUs>=VisibleFlushMinimumWorkUs173&&r.workTimeUs<=VisibleFlushMaximumWorkUs173&&
            ValidG76FlushSpeedPercent(r.commissioningProfile,r.jumpSpeedPercentScaled)&&r.diagnosticControlSource==3U&&
            !r.holdRetreatMode&&(!r.flushHoldTest||IsG76SquareB4_253(r)||IsG76DiamondB4_258(r)||IsG76OneMmB4_260(r)||IsG76ExpansionB4_263(r)||(IsG76FixedRadiusB4_266(r)||IsG76FixedRadiusWorkB4_267(r)||IsG76SharedShort274(r))||IsG76CenterHold217(r)||IsG76CenterAnyFlushHold232(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r))))&&
        (!IsG76B2Flush175(r)||(HasBoundHeight(r)&&r.jumpHeightScaled==G76B2HeightScaled175&&
            r.workTimeUs>=VisibleFlushMinimumWorkUs173&&r.workTimeUs<=VisibleFlushMaximumWorkUs173&&
            r.jumpSpeedPercentScaled==100000000ULL&&r.diagnosticControlSource==3U&&
            !r.holdRetreatMode&&(!r.flushHoldTest||IsG76B2FlushHold200(r)||IsG76SquareB2_251(r)||IsG76DiamondB2_256(r)||IsG76PolygonB2_261(r)||IsG76ExpansionB2_263(r)||(IsG76FixedRadiusB2_266(r)||IsG76FixedRadiusWorkB2_267(r)))))&&
        (!IsVisibleFlush(r)||(HasBoundHeight(r)&&r.jumpHeightScaled==VisibleFlushHeightScaled158&&
            r.workTimeUs==VisibleFlushWorkUs158&&r.jumpSpeedPercentScaled==100000000ULL&&
            (!r.diagnosticControlSource||r.diagnosticControlSource==3U)))&&
        (!IsVisibleFlush159(r)||(HasBoundHeight(r)&&r.jumpHeightScaled==VisibleFlushHeightScaled159&&
            r.workTimeUs==VisibleFlushWorkUs159&&r.jumpSpeedPercentScaled==100000000ULL&&
            (!r.diagnosticControlSource||r.diagnosticControlSource==3U)))&&
        (!IsVisibleFlush167(r)||(HasBoundHeight(r)&&r.jumpHeightScaled==VisibleFlushHeightScaled167&&
            r.workTimeUs==VisibleFlushWorkUs167&&r.jumpSpeedPercentScaled==100000000ULL&&
            (!r.diagnosticControlSource||r.diagnosticControlSource==3U)))&&
        (r.diagnosticControlSource!=5U||(!r.commissioningProfile&&!r.flushHoldTest&&HasBoundProgramPlan(r)))&&
        r.catalogRevision&&r.definitionRevision&&r.profileId&&r.tableId&&r.tableId<=99999999U&&
        r.tableRevision&&r.eCode&&r.eCode<=1000U&&r.fieldId&&r.generation&&ValidWorkTime(r.workTimeUs)&&
        ((!r.speedFieldId&&!r.jumpSpeedPercentScaled)||
            (r.speedFieldId&&r.speedFieldId!=r.fieldId&&ValidSpeedPercent(r.jumpSpeedPercentScaled)))&&
        ((!r.heightFieldId&&!r.jumpHeightScaled)||
            (r.heightFieldId&&r.heightFieldId!=r.fieldId&&r.heightFieldId!=r.speedFieldId&&
                HasBoundSpeed(r)&&(ValidJumpHeight(r.jumpHeightScaled,r.commissioningProfile)||IsG76OneMmB1_260(r)||IsG76ExpansionB1_263(r)||(IsG76FixedRadiusB1_266(r)||IsG76FixedRadiusWorkB1_267(r)||IsG76SharedShortB1_274(r)))))&&
        ((!r.intervalFieldId&&!r.multiplierFieldId&&!r.deepFlushCycleInterval&&!r.deepFlushHeightMultiplierScaled)||
            (HasBoundDeep(r)&&HasBoundHeight(r)&&r.intervalFieldId!=r.multiplierFieldId&&
                r.intervalFieldId!=r.fieldId&&r.intervalFieldId!=r.heightFieldId&&r.intervalFieldId!=r.speedFieldId&&
                r.multiplierFieldId!=r.fieldId&&r.multiplierFieldId!=r.heightFieldId&&r.multiplierFieldId!=r.speedFieldId&&
                (ValidDeepFlushValues(r.jumpHeightScaled,r.deepFlushCycleInterval,r.deepFlushHeightMultiplierScaled,r.commissioningProfile)||IsG76OneMmB1_260(r)||IsG76ExpansionB1_263(r)||(IsG76FixedRadiusB1_266(r)||IsG76FixedRadiusWorkB1_267(r)||IsG76SharedShortB1_274(r)))))&&
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
        ValidBoundFlushStages(r)&&
        (!IsOneShotCommissioning(r)||
            ((!HasBoundMachiningSpeed(r)||r.machiningSpeedPercentScaled==40000000U)&&
             (!HasBoundReferenceVoltage(r)||r.referenceVoltageScaled==55000000U)&&
             (!HasBoundFlushStages(r)||(r.flushStages.finalSpeedPercentScaled==10000000U&&
                r.flushStages.approachDistanceScaled==100ULL&&r.flushStages.initialSpeedPercentScaled==1000000U&&
                r.flushStages.initialDistanceScaled==0ULL&&r.flushStages.polarity==0U))))&&
        ValidProgramPlan(r)&&r.diagnosticControlSource<=5U&&
        (!r.diagnosticControlSource||HasBoundFlushStages(r))&&
        (!IsMotionTest(r)||(HasBoundCurve(r)&&HasBoundReferenceVoltage(r)&&HasBoundMachiningSpeed(r)&&
            HasBoundHeight(r)&&r.jumpHeightScaled<=(IsVisibleFlush(r)?VisibleFlushHeightScaled158:IsVisibleFlush159(r)?VisibleFlushHeightScaled159:IsVisibleFlush167(r)?VisibleFlushHeightScaled167:IsVisibleFlush173(r)?VisibleFlushHeightScaled173:IsG76B3Flush176(r)?G76B3HeightScaled176:IsG76B4Flush177(r)?G76B4HeightScaled177:IsG76B0Flush192(r)?G76B0HeightScaled192:1000ULL)&&r.workTimeUs<=2000000ULL&&
            HasBoundDeep(r)&&r.deepFlushCycleInterval==0U));
}
inline bool ValidHistoryRecipeBinding(unsigned fixture,const Recipe& r) noexcept
{
    return ValidRecipe(r)&&EDM149::ValidHistoryBinding(fixture,r.diagnosticControlSource)&&
        (!r.holdRetreatMode||fixture==(IsVisibleFlush167(r)?EDM149::SixVisibleFlushHistoryFixture:IsVisibleFlush159(r)?EDM149::XYZVisibleFlushHistoryFixture:EDM149::ProgramHistoryFixture))&&
        (!r.flushHoldTest||fixture==((IsG76CommissioningRecipe(r)||IsG76B2FlushHold200(r)||IsG76CenterHold217(r)||IsG76CenterAnyFlushHold232(r)||IsG76CenterRepeat230(r)||IsG76ParentSequence233(r)||IsG76CenterSequence239(r)||IsG76CenterSequence240(r)||IsG76CenterSequence241(r)||IsG76CenterSequence242(r)||IsG76CenterSequence243(r)||IsG76CenterSequence244(r)||IsG76CenterSequence245(r)||IsG76CenterSequence246(r)||IsG76CenterSequence248(r)||IsG76CenterSequence247(r))?EDM149::G76GapHistoryFixture:
            IsVisibleFlush167(r)?EDM149::SixVisibleFlushHistoryFixture:IsVisibleFlush159(r)?EDM149::XYZVisibleFlushHistoryFixture:EDM149::ProgramHistoryFixture))&&
        ((fixture==EDM149::VisibleFlushHistoryFixture)==IsVisibleFlush(r))&&
        ((fixture==EDM149::XYZVisibleFlushHistoryFixture)==IsVisibleFlush159(r))&&
        ((fixture==EDM149::SixVisibleFlushHistoryFixture)==IsVisibleFlush167(r))&&
        ((fixture==EDM149::G76GapHistoryFixture)==IsG76Gap173(r))&&
        (EDM149::IsProgramHistoryFixture(fixture)==HasBoundProgramPlan(r));
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
static_assert(offsetof(Recipe,heightFieldId)==56U&&offsetof(Recipe,commissioningProfile)==58U&&
    offsetof(Recipe,holdRetreatMode)==60U&&offsetof(Recipe,flushHoldTest)==62U&&offsetof(Recipe,jumpHeightScaled)==64U,
    "EDM163/165 use only prior recipe padding without moving earlier fields");
static_assert(sizeof(Recipe)==264U&&offsetof(Recipe,planRows)==44U&&
    offsetof(Recipe,planMovementMask)==46U&&offsetof(Recipe,jumpSpeedPercentScaled)==48U,
    "EDM156 plan uses existing Recipe padding without moving prior fields");
}
