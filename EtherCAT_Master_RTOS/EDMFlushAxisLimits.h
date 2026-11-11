#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

// EDM383: one scalar trajectory for the moving XYZ components. These helpers
// produce limits only; they never start independent per-axis interpolators.
namespace EDM383Axis
{
using Axis3 = std::array<double,3U>;
struct Budget
{
    Axis3 velocityMmS{},accelerationMmS2{},decelerationMmS2{};
};
struct Limits
{
    double velocityMmS=0.0,accelerationMmS2=0.0,decelerationMmS2=0.0;
    unsigned activeMask=0U;
};
inline bool Positive(double x) noexcept {return std::isfinite(x)&&x>0.0;}
inline bool Valid(const Budget& b) noexcept
{
    for(unsigned i=0U;i<3U;++i)
        if(!Positive(b.velocityMmS[i])||!Positive(b.accelerationMmS2[i])||
           !Positive(b.decelerationMmS2[i]))return false;
    return true;
}
// Saturating division preserves tiny nonzero components without producing an
// infinite publication. A large quotient imposes no extra cap; the component
// remains in the active mask and its finite axis limit is still respected.
inline double Quotient(double limit,double component) noexcept
{
    const double largest=(std::numeric_limits<double>::max)();
    return limit>largest*component?largest:limit/component;
}
inline bool Line(const Budget& b,const Axis3& unitDirection,Limits& out) noexcept
{
    out=Limits{};
    if(!Valid(b))return false;
    for(double component:unitDirection)if(!std::isfinite(component))return false;
    const double norm=std::hypot(std::hypot(unitDirection[0],unitDirection[1]),unitDirection[2]);
    if(!Positive(norm)||std::fabs(norm-1.0)>1.e-12)return false;
    Limits value{};
    value.velocityMmS=value.accelerationMmS2=value.decelerationMmS2=(std::numeric_limits<double>::max)();
    for(unsigned i=0U;i<3U;++i){
        const double component=std::fabs(unitDirection[i]);
        if(component==0.0)continue;
        value.activeMask|=1U<<i;
        value.velocityMmS=(std::min)(value.velocityMmS,Quotient(b.velocityMmS[i],component));
        value.accelerationMmS2=(std::min)(value.accelerationMmS2,Quotient(b.accelerationMmS2[i],component));
        value.decelerationMmS2=(std::min)(value.decelerationMmS2,Quotient(b.decelerationMmS2[i],component));
    }
    if(!value.activeMask||!Positive(value.velocityMmS)||!Positive(value.accelerationMmS2)||
       !Positive(value.decelerationMmS2))return false;
    out=value;return true;
}
inline bool Line(const Budget& b,const Axis3& start,const Axis3& end,Limits& out) noexcept
{
    Axis3 delta{};
    for(unsigned i=0U;i<3U;++i){
        if(!std::isfinite(start[i])||!std::isfinite(end[i])){out=Limits{};return false;}
        delta[i]=end[i]-start[i];
        if(!std::isfinite(delta[i])){out=Limits{};return false;}
    }
    const double length=std::hypot(std::hypot(delta[0],delta[1]),delta[2]);
    if(!Positive(length)){out=Limits{};return false;}
    for(double& component:delta)component/=length;
    // If normalization underflows a real displacement, reject rather than
    // silently reclassifying that physical component as stationary.
    for(unsigned i=0U;i<3U;++i)if(start[i]!=end[i]&&delta[i]==0.0){out=Limits{};return false;}
    return Line(b,delta,out);
}
inline bool Conservative(const Budget& b,unsigned mask,Limits& out) noexcept
{
    out=Limits{};
    if(!Valid(b)||!mask||(mask&~7U)!=0U)return false;
    Limits value{};value.activeMask=mask;
    value.velocityMmS=value.accelerationMmS2=value.decelerationMmS2=(std::numeric_limits<double>::max)();
    for(unsigned i=0U;i<3U;++i)if((mask&(1U<<i))!=0U){
        value.velocityMmS=(std::min)(value.velocityMmS,b.velocityMmS[i]);
        value.accelerationMmS2=(std::min)(value.accelerationMmS2,b.accelerationMmS2[i]);
        value.decelerationMmS2=(std::min)(value.decelerationMmS2,b.decelerationMmS2[i]);
    }
    out=value;return true;
}
}
