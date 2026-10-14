#pragma once
#include "EDMCNCRetainedRTSession.h"

// NC-only construction. Original CNC builders create copied diagnostic LINE,
// G17 ARC, long XYZ LINE. C/U/V stay held. This is not a live program capture,
// real machine geometry, calibrated pulse scale or physical motion command.
namespace EDM83RT
{
namespace FixtureDetail
{
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
inline bool BuildFixtureRequest(Request& request,const EDMGapServo::Profile& profile,
    double referenceV,double overridePercent) noexcept
{
    request.path=EDM82::RetainedPath{}; request.config=EDM81::Config{};
    if(!EDM81::ValidScope(request.scope) ||
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
    auto& c=request.config;
    c.servo=profile; c.referenceV=referenceV; c.overridePercent=overridePercent;
    c.maxScalarMmMin=c.excursionSpeedMmMin=speed;
    const double perSecond=speed/60.0;
    c.jumpMm=perSecond*0.25; c.maxRetreatMm=perSecond*0.75;
    c.periodUs=PeriodUs; c.maxAgeUs=PeriodUs;
    for(unsigned axis=0U;axis<6U;++axis)
    {
        c.axes[axis].minimum=axis<3U?-10.0:-1000.0;
        c.axes[axis].maximum=axis<3U?10.0:1000.0;
        c.axes[axis].maxTravel=10.0; c.axes[axis].maxVelocityPerMin=10.0;
    }
    // Initial line + half circle consume about 1.035 nominal seconds.
    // The long final line remains active at the operator's five-second HOLD.
    FixtureDetail::Pose a{{0.0,0.0,0.001,359.998,10.0,-5.0,0.0,0.0}},b=a,d=a;
    const double radius=perSecond*0.25;
    b[0]=radius; auto endArc=b; endArc[0]=-radius;
    constexpr double pi=3.141592653589793238462643383279502884;
    const double total=perSecond*(static_cast<double>(DurationUs(request.profile))/1000000.0-3.0);
    const double remainder=total-radius-pi*radius;
    if(!EDM81::Finite(remainder) || remainder<=0.0) return false;
    d=endArc; d[0]-=remainder*0.6; d[2]+=remainder*0.8;
    std::array<double,8U> scale{}; scale.fill(1000.0);
    NCPathCoreRetainedPath source{};
    if(!FixtureDetail::AppendLine(source,a,b,scale) ||
        !FixtureDetail::AppendArc(source,b,endArc,scale) ||
        !FixtureDetail::AppendLine(source,endArc,d,scale) ||
        !request.path.Capture(source,request.scope,scale)) return false;
    return request.path.ValidateBounds(c)==EDM81::Reason::None;
}
}
