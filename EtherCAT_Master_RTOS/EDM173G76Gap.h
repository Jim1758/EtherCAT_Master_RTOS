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
constexpr unsigned B2ArmP=294U, B2FinishP=295U, B2CommissioningProfile=5U;
constexpr unsigned B3ArmP=296U, B3FinishP=297U, B3CommissioningProfile=6U;
constexpr unsigned B4ArmP=298U, B4FinishP=299U, B4CommissioningProfile=7U;
constexpr unsigned B0ArmP=300U, B0FinishP=301U, B0CommissioningProfile=8U;
constexpr double MaximumCenterReturnFeedMmMin=30.0;
constexpr double MaximumRadiusMm=EDM128::B3CenterRadiusLimitMm, MaximumCenterTravelMm=0.005, MaximumNominalExcursionMm=0.025, FeedMmMin=0.12;
constexpr double MaximumAbsoluteStartRoundTripErrorMm=1.e-9;
// EDM182 admits the existing frozen fixed-XYZ H selection with rigid XY
// rotation. EDM184 also admits the existing fixed G168 XY rotation. Snapshot
// validation owns H/W ranges, finite rows and their fixed rotation centers.
// EDM185 admits fixed X/Y mirrors before those rotations. EDM186 also
// admits the existing positive uniform XYZ scale; both authored and physical
// radius/travel remain bounded. EDM188 also admits fixed Z reflection for
// the nominal orbit. B4 E6 stays a native +Z distance, unscaled/unmirrored.
// Other feed lanes retain their existing frame contracts.
inline bool FrameSupported(const NCTranslationSnapshot& source) noexcept
{
    return NCG76::FrameSupported(source,true,true)&&IsNCXYZCUVFeedAxisScopeValid(source.axisIdentity)&&
        source.axisIdentity.exists[6U]==0U&&source.axisIdentity.exists[7U]==0U&&
        source.unitsMode==21&&(source.distanceMode==90||source.distanceMode==91)&&
        (source.toolLengthMode==43||source.toolLengthMode==44||source.toolLengthMode==49)&&
        (source.rotationMode==68||source.rotationMode==69)&&
        (source.scalingMode==50||source.scalingMode==51)&&(source.mirrorMask&~7U)==0U;
}
namespace Detail
{
inline bool TryOrbitProgramStart(const NCTranslationSnapshot& translation,const NCG76::Point8& startMCS,
    NCG76::Point8& programStart) noexcept
{
    programStart=NCG76::Point8{};
    for(double value:startMCS)if(!std::isfinite(value))return false;
    if(translation.distanceMode==91)return true;
    if(translation.distanceMode!=90)return false;
    NCTranslationInversePoint(translation,startMCS.data(),programStart.data());
    for(double value:programStart)if(!std::isfinite(value))return false;
    NCG76::Point8 reconstructed{};
    NCTranslationForwardPoint(translation,programStart.data(),reconstructed.data());
    double error=0.0;
    for(unsigned axis=0U;axis<3U;++axis)
    {
        const double delta=reconstructed[axis]-startMCS[axis];
        if(!std::isfinite(delta))return false;
        error=std::hypot(error,delta);
    }
    // A fixed physical precision fence prevents huge absolute offsets/pivots
    // from hiding an inaccurate inverse/forward baseline in a micro move.
    return std::isfinite(error)&&error<=MaximumAbsoluteStartRoundTripErrorMm;
}
inline bool RelativePointWithinPrecision(const NCTranslationSnapshot& translation,const NCG76::Point8& startMCS,
    const NCG76::Point3& local,const NCG76::Point8& actual) noexcept
{
    NCG76::Point3 rotated{{0.0,0.0,NCTranslationScaleMirrorAxisVector(translation,local[2],2U)}};
    NCTranslationRotateXYVector(translation,local[0],local[1],rotated[0],rotated[1]);
    double error=0.0;
    for(unsigned axis=0U;axis<3U;++axis)
    {
        const double delta=actual[axis]-(startMCS[axis]+rotated[axis]);
        if(!std::isfinite(delta))return false;
        error=std::hypot(error,delta);
    }
    return std::isfinite(error)&&error<=MaximumAbsoluteStartRoundTripErrorMm;
}
inline bool RadiusWithinLimit(double radiusMm,double scale) noexcept
{
    if(!std::isfinite(radiusMm)||radiusMm<=0.0||radiusMm>MaximumRadiusMm||
        !std::isfinite(scale)||scale<=0.0)return false;
    const double physical=radiusMm*scale;
    return std::isfinite(physical)&&physical>0.0&&physical<=MaximumRadiusMm;
}
inline bool CenterTravelWithinLimit(const NCG76::PlanInput& input,double scale=1.0) noexcept
{
    double travel=0.0;
    for(unsigned axis=0U;axis<3U;++axis)
    {
        if(!std::isfinite(input.start[axis])||!std::isfinite(input.end[axis]))return false;
        const double delta=input.end[axis]-input.start[axis];
        if(!std::isfinite(delta))return false;
        travel=std::hypot(travel,delta);
    }
    // Preserve the authored limit and independently bound physical travel.
    // No origin-dependent epsilon may enlarge either limit. Reject loss of a
    // nonzero center vector to multiplication underflow before publication.
    if(!std::isfinite(travel)||travel>MaximumCenterTravelMm||
        !std::isfinite(scale)||scale<=0.0)return false;
    const double physical=travel*scale;
    return std::isfinite(physical)&&physical<=MaximumCenterTravelMm&&
        (travel==0.0||physical>0.0);
}
inline bool NominalPrecisionWithinLimit(const NCG76::NativePlan& nominal) noexcept
{
    const auto& translation=nominal.translation;
    NCG76::Point8 programStart{};
    if(!TryOrbitProgramStart(translation,nominal.startMCS,programStart))return false;
    if(translation.distanceMode==91)return true;
    const auto& program=nominal.program;
    NCG76::Point3 delta{};
    for(unsigned axis=0U;axis<3U;++axis)delta[axis]=program.input.end[axis]-program.input.start[axis];
    for(unsigned child=0U;child<nominal.count;++child)
    {
        const auto& point=program.points[child];
        const double t=static_cast<double>(point.growIndex)/static_cast<double>(NCG76::GrowSegments);
        NCG76::Point3 center{},local{};
        for(unsigned axis=0U;axis<3U;++axis)
        {
            center[axis]=delta[axis]*t;
            local[axis]=center[axis]+point.orbit[axis];
        }
        // Rebuild local geometry before adding a large author origin. Using
        // position-start alone would hide quantization of the tiny orbit.
        if(!RelativePointWithinPrecision(translation,nominal.startMCS,local,nominal.pointsMCS[child]))return false;
        NCG76::Point8 author=programStart,machine{};
        for(unsigned axis=0U;axis<3U;++axis)author[axis]=point.center[axis];
        NCTranslationForwardPoint(translation,author.data(),machine.data());
        if(!RelativePointWithinPrecision(translation,nominal.startMCS,center,machine))return false;
    }
    return true;
}
}
// Authored XYZ is in mm; mask bits 0..2 say which words were present. A G90
// omitted axis preserves the inverse-transformed accepted machine start.
// A G91 omitted axis contributes zero. Failure always clears the output.
inline bool BuildOrbitInput(const NCTranslationSnapshot& translation,const NCG76::Point8& startMCS,
    const NCG76::Point3& authoredXYZ,unsigned authoredMask,double radiusMm,double feedMmMin,
    NCG76::PlanInput& out,double shapeAngleDeg=0.0,bool targetAxis=false) noexcept
{
    const NCG76::Point8 frozenStart=startMCS;
    const NCG76::Point3 frozenAuthored=authoredXYZ;
    out=NCG76::PlanInput{};
    if(!FrameSupported(translation)||!authoredMask||(authoredMask&~7U)!=0U||
        !Detail::RadiusWithinLimit(radiusMm,translation.scalingFactor)||feedMmMin!=FeedMmMin||
        !NCG76::ValidShapeAngle(shapeAngleDeg))return false;
    for(double value:frozenAuthored)if(!std::isfinite(value))return false;
    NCG76::PlanInput input{};input.radiusMm=radiusMm;input.feedMmMin=feedMmMin;input.shapeAngleDeg=shapeAngleDeg;
    input.targetAxis=targetAxis;
    NCG76::Point8 programStart{};
    if(!Detail::TryOrbitProgramStart(translation,frozenStart,programStart))return false;
    for(unsigned axis=0U;axis<3U;++axis)
    {
        input.start[axis]=programStart[axis];
        input.end[axis]=(authoredMask&(1U<<axis))!=0U?frozenAuthored[axis]:input.start[axis];
    }
    if(!Detail::CenterTravelWithinLimit(input,translation.scalingFactor)||
        (targetAxis&&!NCG76::Detail::NonzeroFiniteLeg(input.start,input.end)))return false;
    if(translation.distanceMode==90)
    {
        NCG76::Point8 author=programStart,machine{};NCG76::Point3 delta{};
        for(unsigned axis=0U;axis<3U;++axis){author[axis]=input.end[axis];delta[axis]=input.end[axis]-input.start[axis];}
        NCTranslationForwardPoint(translation,author.data(),machine.data());
        if(!Detail::RelativePointWithinPrecision(translation,frozenStart,delta,machine))return false;
    }
    out=input;return true;
}
inline bool IsArm(unsigned p) noexcept {return p==ArmP||p==B2ArmP||p==B3ArmP||p==B4ArmP||p==B0ArmP;}
inline bool IsFinish(unsigned p) noexcept {return p==FinishP||p==B2FinishP||p==B3FinishP||p==B4FinishP||p==B0FinishP;}
inline unsigned FinishForArm(unsigned p) noexcept {return p==ArmP?FinishP:p==B2ArmP?B2FinishP:p==B3ArmP?B3FinishP:p==B4ArmP?B4FinishP:p==B0ArmP?B0FinishP:0U;}
inline unsigned ProfileForArm(unsigned p) noexcept {return p==ArmP?CommissioningProfile:p==B2ArmP?B2CommissioningProfile:p==B3ArmP?B3CommissioningProfile:p==B4ArmP?B4CommissioningProfile:p==B0ArmP?B0CommissioningProfile:0U;}
inline unsigned ModeForArm(unsigned p) noexcept {return p==ArmP?1U:p==B2ArmP?2U:p==B3ArmP?3U:p==B4ArmP?4U:p==B0ArmP?0U:255U;}
inline bool IsB0Recipe(const EDM130::Recipe& recipe) noexcept {return EDM130::IsG76B0Flush192(recipe);}
inline bool IsB2Recipe(const EDM130::Recipe& recipe) noexcept {return EDM130::IsG76B2Flush175(recipe);}
inline bool IsB3Recipe(const EDM130::Recipe& recipe) noexcept {return EDM130::IsG76B3Flush176(recipe);}
inline bool IsB4Recipe(const EDM130::Recipe& recipe) noexcept {return EDM130::IsG76B4Flush177(recipe);}
inline bool IsCenterRecipe(const EDM130::Recipe& recipe) noexcept {return IsB3Recipe(recipe)||IsB4Recipe(recipe);}
// E203 extends the reviewed XYZ target-axis plane to B3's genuine 3D center.
// B4 retains its existing planar upper-center contract.
inline bool UsesTargetAxis(const EDM130::Recipe& recipe) noexcept
{return IsB0Recipe(recipe)||IsB2Recipe(recipe)||IsB3Recipe(recipe)||EDM130::IsVisibleFlush173(recipe);}
// EDM197: B0 tangent and B2 retained-path motion use only the nominal axes.
// B1/B3/B4 still require Z evidence for their independent native Z flush.
inline bool UsesNominalMovementMask(const EDM130::Recipe& recipe) noexcept
{return IsB0Recipe(recipe)||IsB2Recipe(recipe);}
// A stationary center Z does not imply stationary target-axis orbit Z.
// Keep the legacy fixed-Z rule only for the planar-center B4 lane.
inline bool PreservesG90StationaryZ(const EDM130::Recipe& recipe) noexcept
{return EDM130::IsG76Gap173(recipe)&&!UsesTargetAxis(recipe);}
inline unsigned ArmForRecipe(const EDM130::Recipe& recipe) noexcept
{return IsB0Recipe(recipe)?B0ArmP:IsB4Recipe(recipe)?B4ArmP:IsB3Recipe(recipe)?B3ArmP:IsB2Recipe(recipe)?B2ArmP:EDM130::IsVisibleFlush173(recipe)?ArmP:0U;}
inline bool RecipeMatchesArm(unsigned p,const EDM130::Recipe& recipe) noexcept
{return IsArm(p)&&recipe.commissioningProfile==ProfileForArm(p);}
inline EDM116::Mode ExpectedMode(const EDM130::Recipe& recipe) noexcept
{return IsB0Recipe(recipe)?EDM116::Mode::B0:IsB4Recipe(recipe)?EDM116::Mode::B4:IsB3Recipe(recipe)?EDM116::Mode::B3:IsB2Recipe(recipe)?EDM116::Mode::B2:EDM130::IsVisibleFlush173(recipe)?EDM116::Mode::B1:EDM116::Mode::Invalid;}
inline double FrozenHeightMm(const EDM130::Recipe& recipe) noexcept
{return (IsB0Recipe(recipe)||IsCenterRecipe(recipe))?2.0:IsB2Recipe(recipe)?0.001:EDM130::IsVisibleFlush173(recipe)?2.0:0.0;}
// EDM193 movement evidence is derived from the frozen compensated endpoints,
// not the authored word mask or the producer's structural XYZ axis mask.
// Exact numeric equality treats signed zero as stationary; no tolerance can
// hide a nonzero Z command. Full native-plan validation remains with Inputs.
inline unsigned NominalMovementMask(const NCG76::NativePlan& nominal) noexcept
{
    if(nominal.count!=NominalCount||nominal.program.count!=NominalCount)return 0U;
    for(double value:nominal.startMCS)if(!std::isfinite(value))return 0U;
    unsigned mask=0U;
    for(unsigned child=0U;child<NominalCount;++child)
        for(unsigned axis=0U;axis<8U;++axis)
        {
            const double value=nominal.pointsMCS[child][axis];
            if(!std::isfinite(value))return 0U;
            if(axis<3U) {if(value!=nominal.startMCS[axis])mask|=1U<<axis;}
            else if(EDM97::DoubleBits(value)!=EDM97::DoubleBits(nominal.startMCS[axis]))return 0U;
        }
    return mask==3U||mask==7U?mask:0U;
}
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
    // Frozen requested center-leg feed, separate from nominal machining F.
    // The RT commissioning and per-axis velocity limits still apply.
    double centerReturnFeedMmMin=0.0;
    std::uint64_t fingerprint=0ULL;
};
struct Feedback
{
    Scope scope{};
    std::uint64_t fingerprint=0ULL,liveEpoch=0ULL,tick=0ULL;
    unsigned authorizedChildren=0U,completedChildren=0U;
    EDM128::Feedback completedEvent{};
    // Captured with completedEvent on its final mapped tick, not final-run
    // traversal totals (which also include ordinary GAP direction changes).
    std::uint64_t eventBackwardCrossings=0ULL,eventForwardCrossings=0ULL;
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
    const EDM130::Recipe& r,const EDM97::Tuning& t,double centerReturnFeedMmMin=0.0) noexcept
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
    if(IsCenterRecipe(r))Real(h,centerReturnFeedMmMin);
    Recipe(h,r);Real(h,t.positiveGain);Real(h,t.negativeGain);Real(h,t.deadbandV);
    // Omitted/+0 K retains the complete legacy fingerprint stream. All other
    // bit patterns carry a G76K180 domain, including -0 and +/-180 sources.
    if(EDM97::DoubleBits(in.shapeAngleDeg)!=EDM97::DoubleBits(0.0))
    {Word(h,0x004737364b313830ULL);Real(h,in.shapeAngleDeg);}
    // Preserve the entire legacy stream for false. Bind the target-axis
    // contract even for a pure Z target whose generated points can coincide.
    if(in.targetAxis){Word(h,0x4737364158323031ULL);Word(h,1U);} // G76AX201
    return h?h:1ULL;
}
inline bool Inputs(const Scope& scope,const NCG76::NativePlan& nominal,
    const EDM130::Recipe& recipe,const EDM97::Tuning& tuning) noexcept
{
    if(!ValidScope(scope)||!EDM130::IsG76Gap173(recipe)||
        nominal.program.input.targetAxis!=UsesTargetAxis(recipe)||
        !EDM130::ValidHistoryRecipeBinding(HistoryFixture,recipe)||!EDM97::ValidMotionTestCurve(tuning,recipe)||
        !NCG76::ValidNative(nominal,true,true,PreservesG90StationaryZ(recipe))||!FrameSupported(nominal.translation)||
        nominal.translation.runToken!=scope.run||nominal.count!=NominalCount||nominal.program.count!=NominalCount||
        nominal.program.growCount!=24U||nominal.program.input.flushMode!=0U||nominal.program.flushLegCount!=0U||
        nominal.program.input.heightMm!=0.0||nominal.program.input.centerReturnFeedMmMin!=0.0||
        !RadiusWithinLimit(nominal.program.input.radiusMm,nominal.translation.scalingFactor)||nominal.program.input.feedMmMin!=FeedMmMin||
        !CenterTravelWithinLimit(nominal.program.input,nominal.translation.scalingFactor)||!NominalPrecisionWithinLimit(nominal)||
        (UsesNominalMovementMask(recipe)&&recipe.planMovementMask!=NominalMovementMask(nominal)))return false;
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
inline bool ValidCenterReturnFeed(const EDM130::Recipe& recipe,double feed) noexcept
{
    return IsCenterRecipe(recipe)?std::isfinite(feed)&&feed>0.0&&feed<=MaximumCenterReturnFeedMmMin:
        EDM97::DoubleBits(feed)==EDM97::DoubleBits(0.0);
}
inline bool Build(const Scope& scope,const NCG76::NativePlan& nominal,const EDM130::Recipe& recipe,
    const EDM97::Tuning& tuning,Plan& out,double centerReturnFeedMmMin=0.0) noexcept
{
    if(!ValidCenterReturnFeed(recipe,centerReturnFeedMmMin)||!Detail::Inputs(scope,nominal,recipe,tuning)){out=Plan{};return false;}
    const auto fingerprint=Detail::Fingerprint(scope,nominal,recipe,tuning,centerReturnFeedMmMin);
    out.scope=scope;out.nominal=nominal;out.recipe=recipe;out.tuning=tuning;out.centerReturnFeedMmMin=centerReturnFeedMmMin;out.fingerprint=fingerprint;
    return true;
}
inline bool Valid(const Plan& p) noexcept
{
    return p.fingerprint&&ValidCenterReturnFeed(p.recipe,p.centerReturnFeedMmMin)&&Detail::Inputs(p.scope,p.nominal,p.recipe,p.tuning)&&
        p.fingerprint==Detail::Fingerprint(p.scope,p.nominal,p.recipe,p.tuning,p.centerReturnFeedMmMin);
}
// Scalar boundary conversion uses exactly the native plan's frozen frame.
// Callers retain the validated plan; no full-plan scratch or stack copy here.
inline bool CenterBoundary(const Plan& plan,unsigned boundary,NCG76::Point8& centerMCS) noexcept
{
    centerMCS=NCG76::Point8{};
    if(!IsCenterRecipe(plan.recipe)||boundary>NominalCount||plan.nominal.count!=NominalCount||
        plan.nominal.program.count!=NominalCount||!FrameSupported(plan.nominal.translation))return false;
    const auto& n=plan.nominal;const auto& p=n.program;
    centerMCS=n.startMCS;
    if(!boundary)return true;
    double author[8]{},machine[8]{};
    if(n.translation.distanceMode==91)
    {
        const bool hasXYZ[8]={true,true,true,false,false,false,false,false};
        for(unsigned axis=0U;axis<3U;++axis)author[axis]=p.points[boundary-1U].center[axis]-p.input.start[axis];
        if(!TryNCTranslationIncrementalTarget(n.translation,n.startMCS.data(),author,hasXYZ,machine))return false;
    }
    else
    {
        NCTranslationInversePoint(n.translation,n.startMCS.data(),author);
        for(unsigned axis=0U;axis<3U;++axis)author[axis]=p.points[boundary-1U].center[axis];
        NCTranslationForwardPoint(n.translation,author,machine);
    }
    for(unsigned axis=0U;axis<3U;++axis)
    {
        centerMCS[axis]=machine[axis];
        if(!std::isfinite(centerMCS[axis]))return false;
    }
    // Match nominal anchoring only for the entire stationary authored Z path.
    // Check the raw transformed XYZ first so preservation cannot hide overflow.
    if(n.translation.distanceMode==90&&p.input.flushMode==0U&&p.input.start[2U]==p.input.end[2U])
    {
        if(!std::isfinite(n.startMCS[2U]))return false;
        centerMCS[2U]=n.startMCS[2U];
    }
    return true;
}
inline bool CenterForCursor(const Plan& plan,unsigned child,double ratio,NCG76::Point8& centerMCS) noexcept
{
    centerMCS=NCG76::Point8{};
    if(child>=NominalCount||!std::isfinite(ratio)||ratio<0.0||ratio>1.0)return false;
    NCG76::Point8 from{},to{};
    if(!CenterBoundary(plan,child,from)||!CenterBoundary(plan,child+1U,to))return false;
    for(unsigned axis=0U;axis<8U;++axis)
    {
        centerMCS[axis]=ratio==0.0?from[axis]:ratio==1.0?to[axis]:from[axis]+(to[axis]-from[axis])*ratio;
        if(!std::isfinite(centerMCS[axis]))return false;
    }
    return true;
}
inline void ClearB3Orbit(EDM128::B3Orbit& out) noexcept
{
    for(auto& center:out.centers)for(auto& value:center)value=0.0;
    out.scope=EDM109::Scope{};out.centerReturnMmMin=0.0;
}
// Shared B3/B4 table: centers stay on the nominal orbit plane. The coordinator
// derives B4's upper center only after capturing the actual stopped anchor.
inline bool BuildB3Orbit(const Plan& plan,const NCG76::Point8& startPulse,const NCG76::Point8& pulsePerUnit,
    const EDM109::Scope& scope,EDM128::B3Orbit& out) noexcept
{
    ClearB3Orbit(out);
    if(!IsCenterRecipe(plan.recipe)||!ValidCenterReturnFeed(plan.recipe,plan.centerReturnFeedMmMin)||
        !EDM109::ValidScope(scope)||scope.session!=plan.scope.session||scope.run!=plan.scope.run||
        scope.cache!=plan.scope.cache||scope.ownerGeneration!=plan.scope.owner.generation||
        scope.translationGeneration!=plan.nominal.translationGeneration)return false;
    out.scope=scope;out.centerReturnMmMin=plan.centerReturnFeedMmMin;
    for(unsigned axis=0U;axis<6U;++axis)
        if(!std::isfinite(startPulse[axis])||!std::isfinite(pulsePerUnit[axis])||pulsePerUnit[axis]<=0.0)
        {ClearB3Orbit(out);return false;}
    for(unsigned boundary=0U;boundary<=NominalCount;++boundary)
    {
        NCG76::Point8 center{};
        if(!CenterBoundary(plan,boundary,center)){ClearB3Orbit(out);return false;}
        for(unsigned axis=0U;axis<6U;++axis)
        {
            out.centers[boundary][axis]=axis>=3U||center[axis]==plan.nominal.startMCS[axis]?
                startPulse[axis]:center[axis]*pulsePerUnit[axis];
            if(!std::isfinite(out.centers[boundary][axis])){ClearB3Orbit(out);return false;}
        }
    }
    if(!EDM128::ValidB3Orbit(out,scope)){ClearB3Orbit(out);return false;}
    return true;
}
// Reuse the canonical retained LINE evaluator without claiming AIR B2 intent.
// Only geometry is produced; E5 has no preselected chord or trigger index.
// EDM183 FIX3: only an explicit actual/frozen machine policy can qualify the
// stopped C/U/V display alias. Generic mixed/AIR Unit remains unchanged.
inline bool ValidStationaryRotaryModulo(const NCG76Mixed::Point8& modulo) noexcept
{
    for(unsigned a=0U;a<8U;++a)
        if(!std::isfinite(modulo[a]) || (a>=3U&&a<6U?modulo[a]<=0.0:modulo[a]!=0.0))return false;
    return true;
}
inline bool StationaryRotaryUnit(double pulse,double mcs,double ppu,double modulo) noexcept
{
    if(!std::isfinite(pulse)||!std::isfinite(mcs)||!std::isfinite(ppu)||ppu<=0.0||
        !std::isfinite(modulo)||modulo<=0.0)return false;
    if(NCG76Mixed::Detail::Unit(pulse,mcs,ppu))return true;
    const double native=pulse/ppu;
    if(!std::isfinite(native))return false;
    double represented=std::fmod(native,modulo);
    if(represented<0.0)represented+=modulo;
    if(!std::isfinite(represented))return false;
    // Preserve the existing display's rounding at a tiny negative native zero.
    // This exact case does not admit a different winding or an arbitrary MCS.
    if(represented==modulo&&mcs==modulo)return true;
    if(mcs<0.0||mcs>=modulo)return false;
    const double scale=(std::max)(1.0,(std::max)(std::fabs(mcs),std::fabs(represented)));
    const double error=std::fabs(represented-mcs);
    return std::isfinite(error)&&error<=64.0*std::numeric_limits<double>::epsilon()*scale&&error<=1e-7;
}
inline bool StationaryAuxiliaryRowValid(const NCG76Mixed::Geometry& row,
    const NCG76Mixed::Point8& startMCS,const NCG76Mixed::Point8& startPulse) noexcept
{
    for(unsigned a=3U;a<8U;++a)
        if(!NCG76Mixed::Detail::Bits(row.startMCS[a],startMCS[a])||
            !NCG76Mixed::Detail::Bits(row.endMCS[a],startMCS[a])||
            !NCG76Mixed::Detail::Bits(row.startPulse[a],startPulse[a])||
            !NCG76Mixed::Detail::Bits(row.endPulse[a],startPulse[a]))return false;
    return true;
}
inline bool BuildNominal(const Plan& plan,const NCG76Mixed::Point8& startPulse,
    const NCG76Mixed::Point8& pulsePerUnit,NCG76Mixed::NominalSource& out,
    const NCG76Mixed::Point8* stationaryRotaryModulo=nullptr) noexcept
{
    const NCG76Mixed::Point8 initial=startPulse,ppu=pulsePerUnit;
    const NCG76Mixed::Point8 modulo=stationaryRotaryModulo?*stationaryRotaryModulo:NCG76Mixed::Point8{};
    NCG76Mixed::Detail::ClearNominal(out);
    if(!Valid(plan)||(stationaryRotaryModulo&&!ValidStationaryRotaryModulo(modulo)))return false;
    unsigned mask=0U;
    for(unsigned a=0U;a<8U;++a)if(plan.nominal.translation.axisIdentity.exists[a])mask|=1U<<a;
    if(!NCG76Mixed::Detail::Calibration(mask,ppu))return false;
    for(unsigned a=0U;a<8U;++a)
        if(!std::isfinite(initial[a])||((mask&(1U<<a))!=0U&&
            !(stationaryRotaryModulo&&a>=3U&&a<6U?
                StationaryRotaryUnit(initial[a],plan.nominal.startMCS[a],ppu[a],modulo[a]):
                NCG76Mixed::Detail::Unit(initial[a],plan.nominal.startMCS[a],ppu[a]))))return false;
    NCG76Mixed::Point8 previous=initial;
    for(unsigned k=0U;k<NominalCount;++k)
    {
        if(!NCG76Mixed::Detail::CanonicalOrbitRow(plan.nominal,k,initial,ppu,previous,out.geometry[k])||
            !StationaryAuxiliaryRowValid(out.geometry[k],plan.nominal.startMCS,initial)||
            !NCG76Mixed::Detail::GeometryValid(out.geometry[k],stationaryRotaryModulo?7U:mask,ppu)||
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
    if(!SameScope(a.scope,b.scope)||a.fingerprint!=b.fingerprint||
        EDM97::DoubleBits(a.centerReturnFeedMmMin)!=EDM97::DoubleBits(b.centerReturnFeedMmMin)||!EDM130::SameRecipe(a.recipe,b.recipe)||
        !EDM97::SameTuning(a.tuning,b.tuning)||!SameNCTranslationSnapshot(a.nominal.translation,b.nominal.translation)||
        !NCG76::Detail::SameInput(a.nominal.program.input,b.nominal.program.input)||
        a.nominal.count!=b.nominal.count||a.nominal.version!=b.nominal.version||a.nominal.modeP!=b.nominal.modeP||
        a.nominal.translationGeneration!=b.nominal.translationGeneration)return false;
    const auto& x=a.nominal.program;const auto& y=b.nominal.program;
    if(EDM97::DoubleBits(x.input.radiusMm)!=EDM97::DoubleBits(y.input.radiusMm)||
        EDM97::DoubleBits(x.input.feedMmMin)!=EDM97::DoubleBits(y.input.feedMmMin)||
        EDM97::DoubleBits(x.input.heightMm)!=EDM97::DoubleBits(y.input.heightMm)||
        EDM97::DoubleBits(x.input.centerReturnFeedMmMin)!=EDM97::DoubleBits(y.input.centerReturnFeedMmMin)||
        EDM97::DoubleBits(x.input.shapeAngleDeg)!=EDM97::DoubleBits(y.input.shapeAngleDeg))return false;
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
    if(!ValidScope(scope)||!EDM130::IsG76Gap173(recipe)||
        nominal.program.input.targetAxis!=UsesTargetAxis(recipe)||
        !EDM130::ValidHistoryRecipeBinding(HistoryFixture,recipe)||!EDM97::ValidMotionTestCurve(tuning,recipe)||
        !NCG76::ValidNative(nominal,scratch,true,true,PreservesG90StationaryZ(recipe))||!FrameSupported(nominal.translation)||
        nominal.translation.runToken!=scope.run||nominal.count!=NominalCount||nominal.program.count!=NominalCount||
        nominal.program.growCount!=24U||nominal.program.input.flushMode!=0U||nominal.program.flushLegCount!=0U||
        nominal.program.input.heightMm!=0.0||nominal.program.input.centerReturnFeedMmMin!=0.0||
        !RadiusWithinLimit(nominal.program.input.radiusMm,nominal.translation.scalingFactor)||nominal.program.input.feedMmMin!=FeedMmMin||
        !CenterTravelWithinLimit(nominal.program.input,nominal.translation.scalingFactor)||!NominalPrecisionWithinLimit(nominal)||
        (UsesNominalMovementMask(recipe)&&recipe.planMovementMask!=NominalMovementMask(nominal)))return false;
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
    if(!p.fingerprint || !ValidCenterReturnFeed(p.recipe,p.centerReturnFeedMmMin) || !Detail::Inputs(p.scope,p.nominal,p.recipe,p.tuning,scratch) ||
        p.fingerprint!=Detail::Fingerprint(p.scope,p.nominal,p.recipe,p.tuning,p.centerReturnFeedMmMin))
        return NCG76::ClearValidationScratchFalse(scratch);
    return true;
}
} // namespace EDM173
