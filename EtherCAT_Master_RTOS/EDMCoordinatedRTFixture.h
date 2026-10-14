#pragma once
#include "EDMCoordinatedRTSession.h"
#include <new>
#include <type_traits>

// NC-only construction. Original CNC builders create copied diagnostic LINE,
// G17 ARC, long XYZ LINE plus synchronized unwrapped C/U/V. This is not a live program capture,
// real machine geometry, calibrated pulse scale or physical motion command.
namespace EDM84RT
{
// NC-owned persistent storage. Callers must keep one workspace per NC owner;
// there is no shared static scratch and no allocation in BuildFixtureRequest.
struct FixtureWorkspace
{
    FixtureWorkspace() noexcept {} // Preserve field defaults without a temporary.
    NCPathCoreRetainedPath source{};
    EDM82::RetainedPath base{};
};
namespace FixtureDetail
{
template<class T> inline void ResetInPlace(T& value) noexcept
{
    static_assert(std::is_trivially_destructible<T>::value, "Fixture reset must not release resources");
    static_assert(std::is_nothrow_default_constructible<T>::value, "Fixture reset must not throw");
    value.~T();
    ::new (static_cast<void*>(&value)) T{};
}
using Pose=std::array<double,8U>;
inline bool AppendLine(NCPathCoreRetainedPath& source,const Pose& a,const Pose& b,
    const std::array<double,8U>& scale) noexcept
{
    NCPathCoreFeedLineInput input{}; input.startMCS=a; input.endMCS=b;
    input.axisMask=7U; input.feedMMMin=5.0; input.maxVelocityPPS.fill(100000.0);
    for(unsigned i=0U;i<8U;++i)
    { input.startPulse[i]=a[i]*scale[i]; input.endPulse[i]=b[i]*scale[i]; }
    NCPathCoreFeedLineV2 line{}; NCPathCoreRetainedGeometry geometry{};
    return BuildNCPathCoreFeedLine(input,line)==NCPathCoreFeedLineCode::BUILT_LINE &&
        BuildNCPathCoreRetainedLine(line,geometry) && source.Append(geometry,0x3FU,scale);
}
inline bool AppendArc(NCPathCoreRetainedPath& source,const Pose& a,const Pose& b,
    const std::array<double,8U>& scale) noexcept
{
    NCPathCoreFeedArcInput input{}; input.startMCS=a; input.endMCS=b;
    input.centerOffsetMM={{-a[0],-a[1]}}; input.pulsePerMM={{scale[0],scale[1]}};
    input.maxVelocityPPS={{100000.0,100000.0}}; input.feedMMMin=5.0; input.direction=1;
    for(unsigned i=0U;i<8U;++i)
    { input.startPulse[i]=a[i]*scale[i]; input.endPulse[i]=b[i]*scale[i]; }
    NCPathCoreFeedArcV2 arc{}; NCPathCoreRetainedGeometry geometry{};
    return BuildNCPathCoreFeedArc(input,arc)==NCPathCoreFeedArcCode::BUILT_ARC &&
        BuildNCPathCoreRetainedArc(arc,geometry) && source.Append(geometry,0x3FU,scale);
}
}
inline bool BuildFixtureRequest(Request& request,FixtureWorkspace& workspace,const EDMGapServo::Profile& profile,
    double referenceV,double overridePercent) noexcept
{
    // Reset directly in the caller-owned objects. Aggregate assignment here
    // would create multi-kilobyte temporaries in an unoptimized RTSS build.
    FixtureDetail::ResetInPlace(request.path);
    FixtureDetail::ResetInPlace(request.config);
    FixtureDetail::ResetInPlace(workspace);
    if(!ValidScenario(request.scenario) || request.scenario==Scenario::ToolHeadCUVCompensation ||
        !EDM87::IsEmptySelection(request.toolHeadSelection) ||
        request.sampledFault!=SampledFault::Normal ||
        !EDM81::ValidScope(request.scope) ||
        (request.profile!=Profile::Normal && request.profile!=Profile::OperatorWindow) ||
        !EDMGapServo::ValidateProfile(profile) || !EDM81::Finite(referenceV) || referenceV<10.0 ||
        !EDM81::Finite(overridePercent) || overridePercent<=0.0 || overridePercent>100.0 ||
        !profile.shortMachiningEnabled || profile.shortEnterMs>100U || profile.shortExitMs>100U ||
        profile.shortMachiningV+profile.shortHysteresisV>=referenceV-10.0) return false;
    const auto f=EDMGapServo::Evaluate(profile,referenceV+10.0,referenceV,overridePercent);
    const auto r=EDMGapServo::Evaluate(profile,referenceV-10.0,referenceV,overridePercent);
    const auto s=EDMGapServo::Evaluate(profile,0.0,referenceV,overridePercent);
    const auto retreat=EDMGapServo::EvaluateShortRetreat(profile,s);
    const double shortSpeed=-retreat.signedSpeedMmPerMin*(retreat.usingCurve?1.0:overridePercent/100.0);
    if(!f.valid || !r.valid || !s.valid || !retreat.valid || !EDM81::Finite(shortSpeed)) return false;
    const double speed=EDM81::Minimum(5.0,EDM81::Minimum(f.limitedSpeedMmPerMin,
        EDM81::Minimum(-r.limitedSpeedMmPerMin,shortSpeed)));
    // Diagnostic budget only. Freeze real E8/E9/P-N/short recipe unchanged;
    // bound this private path's rate by the slowest of its required demands.
    // Slow or zero-demand production recipes are valid elsewhere, not here.
    if(speed<0.05) return false;
    request.config.direction.revision=request.scope.frameRevision;
    request.config.mapping.revision=request.scope.kinematicsRevision;
    request.config.direction.unit={{0.36,0.48,0.8}};
    if(request.scenario!=Scenario::SixAxisPath)
        request.config.excursion=EDM84::ExcursionMode::DirectionOffset;
    if(request.scenario==Scenario::ElectrodeCCompensation)
    {
        // Explicit fixed machine-Z C topology: U/V remain native synchronized
        // axes, not an assumed table/head orientation or six-axis TCP chain.
        request.config.direction.frame=EDM84::DirectionFrame::ElectrodeC;
        request.config.mapping.kind=EDM84::MappingKind::FixedZC;
        request.config.mapping.toolOffsetMm={{0.002,-0.001,0.003}};
    }
    auto& c=request.config.motion;
    c.servo=profile; c.referenceV=referenceV; c.overridePercent=overridePercent;
    c.maxScalarMmMin=c.excursionSpeedMmMin=speed;
    const double perSecond=speed/60.0;
    c.jumpMm=perSecond*0.25;
    // P80/P82/P84 keep the longer manual short-retreat window moving at about 4 s.
    // This changes only the private diagnostic envelope, never a machine cap.
    c.maxRetreatMm=perSecond*(IsTrajectoryOperatorWindow(request.profile)?2.25:0.75);
    c.periodUs=PeriodUs; c.maxAgeUs=PeriodUs;
    for(unsigned axis=0U;axis<6U;++axis)
    {
        c.axes[axis].minimum=axis<3U?-10.0:-1000.0;
        c.axes[axis].maximum=axis<3U?10.0:1000.0;
        c.axes[axis].maxTravel=10.0; c.axes[axis].maxVelocityPerMin=axis<3U?10.0:60.0;
    }
    // Initial line + half circle consume about 1.035 nominal seconds.
    // The long final line remains active at the operator's five-second HOLD.
    FixtureDetail::Pose a{{0.0,0.0,0.001,359.0,10.0,-5.0,0.0,0.0}},b=a,d=a;
    const double radius=perSecond*0.25;
    b[0]=radius; auto endArc=b; endArc[0]=-radius;
    constexpr double pi=3.141592653589793238462643383279502884;
    const double total=perSecond*(static_cast<double>(DurationUs(request.profile))/1000000.0-3.0);
    const double remainder=total-radius-pi*radius;
    if(!EDM81::Finite(remainder) || remainder<=0.0) return false;
    d=endArc; d[0]-=remainder*0.6; d[2]+=remainder*0.8;
    std::array<double,8U> scale{}; scale.fill(1000.0);
    auto& source=workspace.source;
    auto& base=workspace.base;
    if(!FixtureDetail::AppendLine(source,a,b,scale) ||
        !FixtureDetail::AppendArc(source,b,endArc,scale) ||
        !FixtureDetail::AppendLine(source,endArc,d,scale) ||
        !base.Capture(source,request.scope,scale)) return false;
    EDM84::RotaryPoint rotary[4U]{};
    for(unsigned boundary=0U;boundary<4U;++boundary)
    {
        const double u=base.Station(boundary)/base.LengthMm();
        rotary[boundary]={{359.0+2.0*u,10.0+u,-5.0-u}};
    }
    if(!request.path.Capture(base,rotary,4U,request.scope)) return false;
    const double envelope=request.config.excursion==EDM84::ExcursionMode::DirectionOffset?c.maxRetreatMm:0.0;
    return EDM84::ValidateMappedBounds(request.path,c,request.config.mapping,envelope)==EDM81::Reason::None;
}

// EDM85: configure a previously copied NC-authored diagnostic path. The
// original coordinate scale and authored feed remain data, never axis output.
// F is one common upper bound; installed GAP demands and native-axis derivative
// bounds may lower it. Variable per-row F is rejected by the NC compiler.
inline bool BuildAuthoredRequest(Request& request,const EDMGapServo::Profile& profile,
    double referenceV,double overridePercent,double feedMmMin) noexcept
{
    FixtureDetail::ResetInPlace(request.config);
    request.source=Source::NCAuthored; request.durationUs=0U;
    if(IsSampled(request.profile)) request.config.gapSource=SampledSourceForProfile(request.profile,request.scope.sourceGeneration);
    request.expectedAxisMask=request.expectedRowMask=request.expectedGeometryMask=0U;
    if(!IsAuthored(request.profile) || !ValidScenario(request.scenario) ||
        !ValidSampledFault(request.sampledFault) ||
        (!IsSampled(request.profile) && request.sampledFault!=SampledFault::Normal) ||
        !ToolHeadSelectionScopeValid(request) ||
        !EDM81::ValidScope(request.scope) || !request.path.Valid() ||
        !EDM81::SameScope(request.scope,request.path.FrozenScope()) ||
        !EDMGapServo::ValidateProfile(profile) || !EDM81::Finite(referenceV) || referenceV<10.0 ||
        (IsSampled(request.profile) && referenceV>290.0) ||
        (IsSourceDwell(request.profile) && (!profile.shortEnterMs || !profile.shortExitMs)) ||
        !EDM81::Finite(overridePercent) || overridePercent<=0.0 || overridePercent>100.0 ||
        !EDM81::Finite(feedMmMin) || feedMmMin<=0.0 || feedMmMin>5.0 ||
        !profile.shortMachiningEnabled || profile.shortEnterMs>100U || profile.shortExitMs>100U ||
        profile.shortMachiningV+profile.shortHysteresisV>=referenceV-10.0) return false;
    const auto forward=EDMGapServo::Evaluate(profile,referenceV+10.0,referenceV,overridePercent);
    const auto reverse=EDMGapServo::Evaluate(profile,referenceV-10.0,referenceV,overridePercent);
    const auto shortValue=EDMGapServo::Evaluate(profile,0.0,referenceV,overridePercent);
    const auto retreat=EDMGapServo::EvaluateShortRetreat(profile,shortValue);
    const double shortSpeed=-retreat.signedSpeedMmPerMin*(retreat.usingCurve?1.0:overridePercent/100.0);
    if(!forward.valid || !reverse.valid || !shortValue.valid || !retreat.valid ||
        !EDM81::Finite(shortSpeed)) return false;
    double speed=EDM81::Minimum(feedMmMin,EDM81::Minimum(forward.limitedSpeedMmPerMin,
        EDM81::Minimum(-reverse.limitedSpeedMmPerMin,shortSpeed)));
    auto& config=request.config;
    config.direction.revision=request.scope.frameRevision;
    config.mapping.revision=request.scope.kinematicsRevision;
    config.direction.unit={{0.36,0.48,0.8}};
    if(request.scenario!=Scenario::SixAxisPath) config.excursion=EDM84::ExcursionMode::DirectionOffset;
    if(request.scenario==Scenario::ElectrodeCCompensation)
    {
        config.direction.frame=EDM84::DirectionFrame::ElectrodeC;
        config.mapping.kind=EDM84::MappingKind::FixedZC;
        config.mapping.toolOffsetMm={{0.002,-0.001,0.003}};
    }
    else if(request.scenario==Scenario::ToolHeadCUVCompensation)
    {
        // Typed reference chain: parent C/Z -> child U/X -> child V/Y.
        // The copied joint order, pivot offsets, signs and angle zeros are
        // diagnostic constants, not inferred machine topology/calibration.
        config.direction.frame=EDM84::DirectionFrame::ElectrodeCUV;
        config.mapping.kind=EDM84::MappingKind::SerialHeadCUV;
        config.mapping.chain.joints[1U].parentToPivotMm={{0.001,0.0,0.002}};
        config.mapping.chain.joints[2U].parentToPivotMm={{0.0,0.001,0.002}};
        config.mapping.toolOffsetMm={{0.002,-0.001,0.003}};
    }
    if(!EDM87::IsEmptySelection(request.toolHeadSelection))
    {
        // Apply the validated frozen model before derivative/feed and whole-path
        // bounds checks. The selected model cannot bypass the existing limits.
        const auto& selected=request.toolHeadSelection.profile;
        config.mapping.chain=selected.chain;
        config.mapping.toolOffsetMm=selected.toolTipMm;
        config.direction.unit=selected.directionUnit;
        config.direction.revision=config.mapping.revision=selected.revision;
    }
    auto& c=config.motion;
    c.servo=profile; c.referenceV=referenceV; c.overridePercent=overridePercent;
    c.periodUs=PeriodUs; c.maxAgeUs=PeriodUs;
    for(unsigned axis=0U;axis<6U;++axis)
    {
        c.axes[axis].minimum=axis<3U?-10.0:-1000.0;
        c.axes[axis].maximum=axis<3U?10.0:1000.0;
        c.axes[axis].maxTravel=10.0;
        c.axes[axis].maxVelocityPerMin=axis<3U?10.0:60.0;
    }
    EDM81::Pose derivative{};
    if(request.path.DerivativeBounds(0.0,request.path.LengthMm(),derivative)!=EDM81::Reason::None)
        return false;
    EDM81::Pose mappedDerivative{};
    if(!EDM84::MappedDerivativeBounds(config.mapping,derivative,mappedDerivative)) return false;
    for(unsigned axis=0U;axis<6U;++axis)
    {
        const double bound=mappedDerivative[axis];
        if(!EDM81::Finite(bound) || bound<0.0) return false;
        if(bound>0.0) speed=EDM81::Minimum(speed,c.axes[axis].maxVelocityPerMin*0.999/bound);
    }
    if(!EDM81::Finite(speed) || speed<0.05) return false;
    c.maxScalarMmMin=c.excursionSpeedMmMin=speed;
    c.jumpMm=speed/60.0*0.25;
    // P80/P82's 3..5 s low interval must not reach the old 0.75 s retreat clamp
    // before the operator's 4 s HOLD. Other profiles retain their envelope.
    c.maxRetreatMm=speed/60.0*(IsTrajectoryOperatorWindow(request.profile)?2.25:0.75);
    // Reserve the profile's negative GAP, short-clear/return and jump-return
    // schedule. P80/P82 add three seconds for its longer moving-retreat window.
    // Longer authored programs are rejected atomically at the existing limit.
    const double budget=std::ceil((request.path.LengthMm()/speed*60000000.0+
        static_cast<double>(AuthoredOverheadUs(request.profile)))/
        static_cast<double>(PeriodUs))*static_cast<double>(PeriodUs);
    if(!EDM81::Finite(budget) || budget>60000000.0 || budget<0.0) return false;
    request.durationUs=static_cast<std::uint64_t>(budget);
    if(request.durationUs<DurationUs(request.profile)) request.durationUs=DurationUs(request.profile);
    if(!PathEvidence(request.path,request.expectedAxisMask,request.expectedRowMask,request.expectedGeometryMask))
        return false;
    const double envelope=config.excursion==EDM84::ExcursionMode::DirectionOffset?c.maxRetreatMm:0.0;
    return RequestPlanValid(request) && ScenarioConfigurationValid(request) &&
        EDM84::ValidateMappedBounds(request.path,c,config.mapping,envelope)==EDM81::Reason::None;
}
}
