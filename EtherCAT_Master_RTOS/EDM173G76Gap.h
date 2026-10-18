#pragma once

#include "NCG76OrbitPlan.h"
#include "NCG76MixedPlan.h"
#include "NCXYZCUVFeedScope.h"
#include "MotionExecutionContract.h"
#include "EDMSixAxisGapTuning.h"
#include "EDMSixAxisGapModeCoordinator.h"

// Pure values only. This plan grants no Motion/PDO or discharge authority.
// The NC and RT owners retain the exact full plan and genuine child identities;
// its fingerprint is an additional immutable binding, never their replacement.
namespace EDM173
{
constexpr unsigned ArmP=292U, FinishP=293U, HistoryFixture=14U, CommissioningProfile=4U, NominalCount=25U;
constexpr double MaximumRadiusMm=0.005, MaximumNominalExcursionMm=0.025, FeedMmMin=0.12;
struct Scope
{
    std::uint64_t session=0ULL,run=0ULL,cache=0ULL,dispatch=0ULL;
    int sourcePC=-1,line=0;
    MotionExecutionEpoch armEpoch=MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease owner{};
};
inline bool ValidScope(const Scope& s) noexcept
{
    return s.session&&s.run&&s.cache&&s.dispatch&&s.sourcePC>=0&&s.line>0&&
        s.armEpoch!=MOTION_EXECUTION_EPOCH_INVALID&&s.owner.IsValid()&&s.owner.owner==MotionOwner::AUTO;
}
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.session==b.session&&a.run==b.run&&a.cache==b.cache&&a.dispatch==b.dispatch&&
        a.sourcePC==b.sourcePC&&a.line==b.line&&a.armEpoch==b.armEpoch&&a.owner.Matches(b.owner);
}
struct Plan
{
    Scope scope{};
    NCG76::NativePlan nominal{};
    EDM130::Recipe recipe{};
    EDM97::Tuning tuning{};
    std::uint64_t fingerprint=0ULL;
};
struct Feedback
{
    Scope scope{};
    std::uint64_t fingerprint=0ULL,liveEpoch=0ULL,tick=0ULL;
    unsigned authorizedChildren=0U,completedChildren=0U;
    EDM128::Feedback completedEvent{};
    bool enabled=false,active=false,faulted=false,eventRetained=false;
};
namespace Detail
{
inline void Word(std::uint64_t& h,std::uint64_t v) noexcept
{
    for(unsigned b=0U;b<8U;++b){h^=(v>>(8U*b))&255ULL;h*=1099511628211ULL;}
}
inline void Real(std::uint64_t& h,double v) noexcept {Word(h,EDM97::DoubleBits(v));}
inline void Translation(std::uint64_t& h,const NCTranslationSnapshot& s) noexcept
{
    const std::uint64_t words[]={s.runToken,s.generation,s.revision,static_cast<std::uint64_t>(s.wcsCode),s.schema,
        static_cast<std::uint64_t>(s.toolLengthMode),static_cast<std::uint64_t>(s.toolHCode),
        static_cast<std::uint64_t>(s.workMode),static_cast<std::uint64_t>(s.workWCode),
        static_cast<std::uint64_t>(s.rotationMode),static_cast<std::uint64_t>(s.rotationPlane),
        static_cast<std::uint64_t>(s.distanceMode),static_cast<std::uint64_t>(s.unitsMode),
        static_cast<std::uint64_t>(s.scalingMode),s.mirrorMask,static_cast<std::uint64_t>(s.polarMode),
        static_cast<std::uint64_t>(s.storedStrokeMode),static_cast<std::uint64_t>(s.cutterMode),static_cast<std::uint64_t>(s.cutterD)};
    for(auto w:words)Word(h,w);
    for(unsigned a=0U;a<8U;++a){Real(h,s.extOffsetMM[a]);Real(h,s.wcsOffsetMM[a]);Real(h,s.toolOffsetMM[a]);Real(h,s.workOffset[a]);
        Word(h,s.axisIdentity.exists[a]);Word(h,s.axisIdentity.axisType[a]);Word(h,s.axisIdentity.nativeUnit[a]);
        Word(h,s.axisIdentity.physicalIndexPlusOne[a]);Word(h,static_cast<unsigned char>(s.axisIdentity.address[a]));}
    for(unsigned a=0U;a<2U;++a){Real(h,s.rotationCenterMM[a]);Real(h,s.workRotationCenterMM[a]);}
    for(unsigned a=0U;a<3U;++a){Real(h,s.scalingCenterMM[a]);Real(h,s.mirrorCenterMM[a]);}
    Real(h,s.rotationAngleDeg);Real(h,s.scalingFactor);Real(h,s.cutterRadiusMM);
    Word(h,s.axisIdentity.electrodeAxisPlusOne);Word(h,s.axisIdentity.systemMode);Word(h,s.axisIdentity.eccentricEnabled);Word(h,s.axisIdentity.bound);
    for(auto v:s.axisIdentity.reserved)Word(h,v);
}
inline void Recipe(std::uint64_t& h,const EDM130::Recipe& r) noexcept
{
    const std::uint64_t words[]={r.catalogRevision,r.definitionRevision,r.profileId,r.tableId,r.tableRevision,r.eCode,r.fieldId,
        r.generation,r.workTimeUs,r.speedFieldId,r.diagnosticControlSource,r.planRows,r.planMovementMask,r.jumpSpeedPercentScaled,
        r.heightFieldId,r.commissioningProfile,r.holdRetreatMode,r.flushHoldTest,r.jumpHeightScaled,r.intervalFieldId,r.multiplierFieldId,
        r.deepFlushCycleInterval,r.deepFlushHeightMultiplierScaled,r.machiningFieldId,r.machiningSpeedPercentScaled,r.referenceFieldId,
        r.referenceVoltageScaled,r.curve.processRevision,r.curve.machineProfileId,r.curve.tuningGeneration,
        r.flushStages.finalSpeedFieldId,r.flushStages.approachFieldId,r.flushStages.initialSpeedFieldId,r.flushStages.initialDistanceFieldId,
        r.flushStages.polarityFieldId,r.flushStages.finalSpeedPercentScaled,r.flushStages.initialSpeedPercentScaled,r.flushStages.polarity,
        r.flushStages.approachDistanceScaled,r.flushStages.initialDistanceScaled};
    for(auto w:words)Word(h,w);
    for(unsigned a=0U;a<3U;++a){Real(h,r.curve.positiveGain[a]);Real(h,r.curve.negativeGain[a]);}
    for(unsigned a=0U;a<2U;++a){Real(h,r.curve.positiveBreakV[a]);Real(h,r.curve.negativeBreakV[a]);}
    Real(h,r.curve.deadbandV);Real(h,r.flushStages.finalApproachMmPerMin);Real(h,r.flushStages.initialSlowMmPerMin);
}
inline std::uint64_t Fingerprint(const Scope& s,const NCG76::NativePlan& n,
    const EDM130::Recipe& r,const EDM97::Tuning& t) noexcept
{
    std::uint64_t h=14695981039346656037ULL;
    Word(h,s.session);Word(h,s.run);Word(h,s.cache);Word(h,s.dispatch);Word(h,static_cast<std::uint64_t>(s.sourcePC));
    Word(h,static_cast<std::uint64_t>(s.line));Word(h,s.armEpoch);Word(h,static_cast<unsigned>(s.owner.owner));Word(h,s.owner.generation);
    Word(h,n.version);Word(h,n.modeP);Word(h,n.count);Word(h,n.translationGeneration);Translation(h,n.translation);
    for(unsigned a=0U;a<8U;++a)Real(h,n.startMCS[a]);
    const auto& in=n.program.input;
    for(unsigned a=0U;a<3U;++a){Real(h,in.start[a]);Real(h,in.end[a]);}
    Real(h,in.radiusMm);Real(h,in.feedMmMin);Word(h,in.flushMode);Real(h,in.heightMm);Real(h,in.centerReturnFeedMmMin);
    const auto& program=n.program;
    Word(h,program.version);Word(h,program.modeP);Word(h,program.count);Word(h,program.growCount);
    Word(h,program.flushBeginIndex);Word(h,program.flushEndIndex);Word(h,program.flushLegCount);
    for(auto v:program.flushLegIndices)Word(h,v);
    Real(h,program.chordErrorBoundMm);
    for(unsigned a=0U;a<3U;++a){Real(h,program.boundsMin[a]);Real(h,program.boundsMax[a]);}
    for(unsigned k=0U;k<NCG76::MaximumPoints;++k){
        for(unsigned a=0U;a<8U;++a)Real(h,n.pointsMCS[k][a]);
        Real(h,n.feedMmMin[k]);
        const auto& point=program.points[k];Word(h,static_cast<unsigned>(point.phase));Word(h,point.growIndex);
        Real(h,point.radiusMm);Real(h,point.feedMmMin);
        for(unsigned a=0U;a<3U;++a){Real(h,point.position[a]);Real(h,point.center[a]);Real(h,point.orbit[a]);}
    }
    Recipe(h,r);Real(h,t.positiveGain);Real(h,t.negativeGain);Real(h,t.deadbandV);
    return h?h:1ULL;
}
inline bool Inputs(const Scope& scope,const NCG76::NativePlan& nominal,
    const EDM130::Recipe& recipe,const EDM97::Tuning& tuning) noexcept
{
    if(!ValidScope(scope)||!EDM130::IsVisibleFlush173(recipe)||
        !EDM130::ValidHistoryRecipeBinding(HistoryFixture,recipe)||!EDM97::ValidMotionTestCurve(tuning,recipe)||
        !NCG76::ValidNative(nominal)||!IsNCXYZCUVFeedNeutralFrame(nominal.translation)||
        nominal.translation.runToken!=scope.run||nominal.count!=NominalCount||nominal.program.count!=NominalCount||
        nominal.program.growCount!=24U||nominal.program.input.flushMode!=0U||nominal.program.flushLegCount!=0U||
        nominal.program.input.heightMm!=0.0||nominal.program.input.centerReturnFeedMmMin!=0.0||
        nominal.program.input.radiusMm>MaximumRadiusMm||nominal.program.input.feedMmMin!=FeedMmMin)return false;
    for(unsigned k=0U;k<NominalCount;++k)
    {
        if(nominal.feedMmMin[k]!=FeedMmMin||nominal.program.points[k].phase!=
            (k<24U?NCG76::Phase::Grow:NCG76::Phase::FinishCenter))return false;
        for(unsigned a=0U;a<8U;++a)
            if(a<3U?std::fabs(nominal.pointsMCS[k][a]-nominal.startMCS[a])>MaximumNominalExcursionMm:
                EDM97::DoubleBits(nominal.pointsMCS[k][a])!=EDM97::DoubleBits(nominal.startMCS[a]))return false;
    }
    return true;
}
} // namespace Detail
inline bool Build(const Scope& scope,const NCG76::NativePlan& nominal,const EDM130::Recipe& recipe,
    const EDM97::Tuning& tuning,Plan& out) noexcept
{
    if(!Detail::Inputs(scope,nominal,recipe,tuning)){out=Plan{};return false;}
    const auto fingerprint=Detail::Fingerprint(scope,nominal,recipe,tuning);
    out.scope=scope;out.nominal=nominal;out.recipe=recipe;out.tuning=tuning;out.fingerprint=fingerprint;
    return true;
}
inline bool Valid(const Plan& p) noexcept
{
    return p.fingerprint&&Detail::Inputs(p.scope,p.nominal,p.recipe,p.tuning)&&
        p.fingerprint==Detail::Fingerprint(p.scope,p.nominal,p.recipe,p.tuning);
}
// Reuse the canonical retained LINE evaluator without claiming AIR B2 intent.
// Only geometry is produced; E5 has no preselected chord or trigger index.
inline bool BuildNominal(const Plan& plan,const NCG76Mixed::Point8& startPulse,
    const NCG76Mixed::Point8& pulsePerUnit,NCG76Mixed::NominalSource& out) noexcept
{
    const NCG76Mixed::Point8 initial=startPulse,ppu=pulsePerUnit;
    NCG76Mixed::Detail::ClearNominal(out);
    if(!Valid(plan))return false;
    unsigned mask=0U;
    for(unsigned a=0U;a<8U;++a)if(plan.nominal.translation.axisIdentity.exists[a])mask|=1U<<a;
    if(!NCG76Mixed::Detail::Calibration(mask,ppu))return false;
    for(unsigned a=0U;a<8U;++a)
        if(!std::isfinite(initial[a])||((mask&(1U<<a))!=0U&&
            !NCG76Mixed::Detail::Unit(initial[a],plan.nominal.startMCS[a],ppu[a])))return false;
    NCG76Mixed::Point8 previous=initial;
    for(unsigned k=0U;k<NominalCount;++k)
    {
        if(!NCG76Mixed::Detail::CanonicalOrbitRow(plan.nominal,k,initial,ppu,previous,out.geometry[k])||
            !NCG76Mixed::Detail::GeometryValid(out.geometry[k],mask,ppu)||
            (k&&!AreNCPathCoreRetainedEndpointsConnected(out.geometry[k-1U],out.geometry[k],mask,ppu)))
        {NCG76Mixed::Detail::ClearNominal(out);return false;}
        previous=out.geometry[k].endPulse;out.feedMmMin[k]=FeedMmMin;
    }
    out.count=NominalCount;out.kind=NCG76Mixed::Kind::G76;out.orbit=plan.nominal;
    out.pc=plan.scope.sourcePC;out.line=plan.scope.line;out.triggerAfter=0U;
    return true;
}
inline bool SamePlan(const Plan& a,const Plan& b) noexcept
{
    if(!SameScope(a.scope,b.scope)||a.fingerprint!=b.fingerprint||!EDM130::SameRecipe(a.recipe,b.recipe)||
        !EDM97::SameTuning(a.tuning,b.tuning)||!SameNCTranslationSnapshot(a.nominal.translation,b.nominal.translation)||
        !NCG76::Detail::SameInput(a.nominal.program.input,b.nominal.program.input)||
        a.nominal.count!=b.nominal.count||a.nominal.version!=b.nominal.version||a.nominal.modeP!=b.nominal.modeP||
        a.nominal.translationGeneration!=b.nominal.translationGeneration)return false;
    const auto& x=a.nominal.program;const auto& y=b.nominal.program;
    if(EDM97::DoubleBits(x.input.radiusMm)!=EDM97::DoubleBits(y.input.radiusMm)||
        EDM97::DoubleBits(x.input.feedMmMin)!=EDM97::DoubleBits(y.input.feedMmMin)||
        EDM97::DoubleBits(x.input.heightMm)!=EDM97::DoubleBits(y.input.heightMm)||
        EDM97::DoubleBits(x.input.centerReturnFeedMmMin)!=EDM97::DoubleBits(y.input.centerReturnFeedMmMin))return false;
    if(x.version!=y.version||x.modeP!=y.modeP||x.count!=y.count||x.growCount!=y.growCount||
        x.flushBeginIndex!=y.flushBeginIndex||x.flushEndIndex!=y.flushEndIndex||x.flushLegCount!=y.flushLegCount||
        x.flushLegIndices!=y.flushLegIndices||EDM97::DoubleBits(x.chordErrorBoundMm)!=EDM97::DoubleBits(y.chordErrorBoundMm))return false;
    for(unsigned axis=0U;axis<3U;++axis)
        if(EDM97::DoubleBits(x.boundsMin[axis])!=EDM97::DoubleBits(y.boundsMin[axis])||
            EDM97::DoubleBits(x.boundsMax[axis])!=EDM97::DoubleBits(y.boundsMax[axis]))return false;
    for(unsigned axis=0U;axis<8U;++axis)
        if(EDM97::DoubleBits(a.nominal.startMCS[axis])!=EDM97::DoubleBits(b.nominal.startMCS[axis]))return false;
    for(unsigned k=0U;k<NCG76::MaximumPoints;++k)
    {
        const auto& p=x.points[k];const auto& q=y.points[k];
        if(p.phase!=q.phase||p.growIndex!=q.growIndex||EDM97::DoubleBits(p.radiusMm)!=EDM97::DoubleBits(q.radiusMm)||
            EDM97::DoubleBits(p.feedMmMin)!=EDM97::DoubleBits(q.feedMmMin))return false;
        for(unsigned axis=0U;axis<3U;++axis)
            if(EDM97::DoubleBits(p.position[axis])!=EDM97::DoubleBits(q.position[axis])||
                EDM97::DoubleBits(p.center[axis])!=EDM97::DoubleBits(q.center[axis])||
                EDM97::DoubleBits(p.orbit[axis])!=EDM97::DoubleBits(q.orbit[axis]))return false;
        if(EDM97::DoubleBits(a.nominal.feedMmMin[k])!=EDM97::DoubleBits(b.nominal.feedMmMin[k]))return false;
        for(unsigned axis=0U;axis<8U;++axis)
            if(EDM97::DoubleBits(a.nominal.pointsMCS[k][axis])!=EDM97::DoubleBits(b.nominal.pointsMCS[k][axis]))return false;
    }
    return true;
}
static_assert(NominalCount<=NCG76::MaximumPoints,"EDM173 keeps existing native plan capacity");
static_assert(std::is_trivially_copyable<Plan>::value,"EDM173 is an immutable value plan");

// Opt-in full validation for the RT seal consumer. Scratch is private startup
// heap storage; no legacy stack-heavy validation/build overload is invoked.
using ValidationScratch=NCG76::ValidationScratch;
namespace Detail
{
inline bool Inputs(const Scope& scope,const NCG76::NativePlan& nominal,
    const EDM130::Recipe& recipe,const EDM97::Tuning& tuning,ValidationScratch& scratch) noexcept
{
    if(!ValidScope(scope)||!EDM130::IsVisibleFlush173(recipe)||
        !EDM130::ValidHistoryRecipeBinding(HistoryFixture,recipe)||!EDM97::ValidMotionTestCurve(tuning,recipe)||
        !NCG76::ValidNative(nominal,scratch)||!IsNCXYZCUVFeedNeutralFrame(nominal.translation)||
        nominal.translation.runToken!=scope.run||nominal.count!=NominalCount||nominal.program.count!=NominalCount||
        nominal.program.growCount!=24U||nominal.program.input.flushMode!=0U||nominal.program.flushLegCount!=0U||
        nominal.program.input.heightMm!=0.0||nominal.program.input.centerReturnFeedMmMin!=0.0||
        nominal.program.input.radiusMm>MaximumRadiusMm||nominal.program.input.feedMmMin!=FeedMmMin)return false;
    for(unsigned k=0U;k<NominalCount;++k)
    {
        if(nominal.feedMmMin[k]!=FeedMmMin||nominal.program.points[k].phase!=
            (k<24U?NCG76::Phase::Grow:NCG76::Phase::FinishCenter))return false;
        for(unsigned a=0U;a<8U;++a)
            if(a<3U?std::fabs(nominal.pointsMCS[k][a]-nominal.startMCS[a])>MaximumNominalExcursionMm:
                EDM97::DoubleBits(nominal.pointsMCS[k][a])!=EDM97::DoubleBits(nominal.startMCS[a]))return false;
    }
    return true;
}
} // namespace Detail
inline bool Valid(const Plan& p,ValidationScratch& scratch) noexcept
{
    if(NCG76::Detail::ValidationStorageOverlaps(&p,sizeof(p),&scratch,sizeof(scratch)))return false;
    NCG76::ClearValidationScratch(scratch);
    if(!p.fingerprint || !Detail::Inputs(p.scope,p.nominal,p.recipe,p.tuning,scratch) ||
        p.fingerprint!=Detail::Fingerprint(p.scope,p.nominal,p.recipe,p.tuning))
        return NCG76::ClearValidationScratchFalse(scratch);
    return true;
}
} // namespace EDM173
