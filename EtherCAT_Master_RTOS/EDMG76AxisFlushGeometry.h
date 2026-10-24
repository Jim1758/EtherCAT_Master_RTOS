#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

// E204: immutable geometry of the bounded G76 center-return commissioning
// family. The signed machining axis points toward the authored XYZ target;
// every axial withdrawal is opposite it. No machine-Z fallback is allowed.
namespace EDM204
{
using Point=std::array<double,6U>;
constexpr double MaximumRadiusMm=0.01;
constexpr double CommissioningHeightMm=2.0;
constexpr double MaximumRoundoffMm=1.0e-8;
inline bool ValidCommissioningHeightMm(double height) noexcept
// Pure geometry only. The recipe binder separately restricts 0.2 mm to
// the explicit L233 architecture fixture; this grants no Motion authority.
{return height==0.2||height==1.0||height==CommissioningHeightMm;}
inline double Roundoff(double magnitude) noexcept
{return 64.0*(std::numeric_limits<double>::epsilon)()*(std::max)(1.0,std::fabs(magnitude));}
inline bool AxisNative(const Point& axisPulse,const Point& scale,Point& native) noexcept
{
    native=Point{};double norm=0.0;
    for(unsigned i=0U;i<6U;++i) {
        if(!std::isfinite(scale[i])||scale[i]<=0.0||!std::isfinite(axisPulse[i])||
            (i>=3U&&axisPulse[i]!=0.0))return false;
        if(i<3U) {native[i]=axisPulse[i]/scale[i];norm=std::hypot(norm,native[i]);}
    }
    return std::isfinite(norm)&&std::fabs(norm-1.0)<=128.0*(std::numeric_limits<double>::epsilon)();
}
inline bool CoordinateTolerance(const Point& a,const Point& b,const Point& scale,double& tolerance) noexcept
{
    tolerance=Roundoff(MaximumRadiusMm);
    for(unsigned i=0U;i<6U;++i) {
        if(!std::isfinite(a[i])||!std::isfinite(b[i])||!std::isfinite(scale[i])||scale[i]<=0.0)return false;
        if(i<3U) {
            const double error=2.0*Roundoff((std::max)(std::fabs(a[i]),std::fabs(b[i])))/scale[i];
            if(!std::isfinite(error))return false;
            tolerance=std::hypot(tolerance,error);
        }
    }
    return std::isfinite(tolerance)&&tolerance<=MaximumRoundoffMm;
}
struct Geometry
{
    Point anchorPulse{},nominalCenterPulse{},centerPulse{},topPulse{},axisPulsePerMm{},pulsePerNative{};
    unsigned mode=0U;
    double heightMm=0.0,radiusMm=0.0,axialSpanMm=0.0;
};
inline bool SameGeometry(const Geometry& a,const Geometry& b) noexcept
{
    return a.anchorPulse==b.anchorPulse&&a.nominalCenterPulse==b.nominalCenterPulse&&
        a.centerPulse==b.centerPulse&&a.topPulse==b.topPulse&&a.axisPulsePerMm==b.axisPulsePerMm&&
        a.pulsePerNative==b.pulsePerNative&&a.mode==b.mode&&a.heightMm==b.heightMm&&
        a.radiusMm==b.radiusMm&&a.axialSpanMm==b.axialSpanMm;
}
inline bool Build(const Point& anchor,const Point& nominalCenter,const Point& axisPulse,
    const Point& scale,double height,unsigned mode,Geometry& out) noexcept
{
    out=Geometry{};Point native{};double tolerance=0.0;
    if((mode!=3U&&mode!=4U)||!ValidCommissioningHeightMm(height)||
        !AxisNative(axisPulse,scale,native)||!CoordinateTolerance(anchor,nominalCenter,scale,tolerance))return false;
    double radius=0.0,axialError=0.0;
    for(unsigned i=0U;i<6U;++i) {
        if(i>=3U) {if(anchor[i]!=nominalCenter[i])return false;continue;}
        const double radial=(anchor[i]-nominalCenter[i])/scale[i];
        if(!std::isfinite(radial))return false;
        radius=std::hypot(radius,radial);axialError+=radial*native[i];
    }
    if(!std::isfinite(radius)||radius>MaximumRadiusMm+tolerance||
        !std::isfinite(axialError)||std::fabs(axialError)>tolerance)return false;
    // Retain B4's existing 45-degree radial/axial join in the machining frame.
    // E6 is total axial withdrawal from the nominal orbit center plane, so
    // this first axial contribution is included, not added a second time.
    const double rise=mode==4U?radius:0.0;
    const double span=height-rise;
    if(!std::isfinite(span)||span<=0.0)return false;
    Geometry candidate{};candidate.anchorPulse=anchor;candidate.nominalCenterPulse=nominalCenter;
    candidate.axisPulsePerMm=axisPulse;candidate.pulsePerNative=scale;
    candidate.mode=mode;candidate.heightMm=height;candidate.radiusMm=radius;candidate.axialSpanMm=span;
    for(unsigned i=0U;i<6U;++i) {
        candidate.centerPulse[i]=rise==0.0||axisPulse[i]==0.0?nominalCenter[i]:nominalCenter[i]-rise*axisPulse[i];
        candidate.topPulse[i]=axisPulse[i]==0.0?nominalCenter[i]:nominalCenter[i]-height*axisPulse[i];
        if(!std::isfinite(candidate.centerPulse[i])||!std::isfinite(candidate.topPulse[i]))return false;
    }
    out=candidate;return true;
}
inline bool ValidGeometry(const Geometry& geometry) noexcept
{
    Geometry expected{};
    return Build(geometry.anchorPulse,geometry.nominalCenterPulse,geometry.axisPulsePerMm,
        geometry.pulsePerNative,geometry.heightMm,geometry.mode,expected)&&SameGeometry(expected,geometry);
}
}
