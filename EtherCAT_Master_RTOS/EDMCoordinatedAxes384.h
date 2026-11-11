#pragma once
#include "NCAxisIdentitySnapshot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// Mixed native units never enter one Euclidean norm. A common metric station
// produces each native position via its admitted native-units/mm ratio.
namespace EDM384Axes
{
using Native6=std::array<double,6U>;
struct Budget {Native6 velocity{},acceleration{},deceleration{};std::uint32_t mask=0U;};
struct Limits {double velocity=0.0,acceleration=0.0,deceleration=0.0;std::uint32_t activeMask=0U;};
inline bool Positive(double value) noexcept{return std::isfinite(value)&&value>0.0;}
inline std::uint32_t ExistingMask(const NCAxisIdentitySnapshot& identity) noexcept
{
    if(!IsNCAxisIdentitySnapshotValid(identity))return 0U;
    std::uint32_t mask=0U;for(unsigned i=0U;i<6U;++i)if(identity.exists[i])mask|=1U<<i;return mask;
}
inline bool IsRotary(const NCAxisIdentitySnapshot& identity,unsigned axis) noexcept
{return axis<6U&&identity.exists[axis]==1U&&identity.axisType[axis]!=0U;}
inline bool Valid(const Budget& budget) noexcept
{
    if(!budget.mask||(budget.mask&~0x3fU)!=0U)return false;
    for(unsigned i=0U;i<6U;++i){
        const double v=budget.velocity[i],a=budget.acceleration[i],d=budget.deceleration[i];
        if(budget.mask&(1U<<i)){if(!Positive(v)||!Positive(a)||!Positive(d))return false;}
        else if(v!=0.0||a!=0.0||d!=0.0)return false;
    }
    return true;
}
inline double Quotient(double limit,double ratio) noexcept
{
    const double largest=(std::numeric_limits<double>::max)();
    return limit>largest*ratio?largest:limit/ratio;
}
inline bool LimitsForRatio(const Budget& budget,const Native6& nativePerMetric,Limits& output) noexcept
{
    output=Limits{};if(!Valid(budget))return false;
    Limits result{};result.velocity=result.acceleration=result.deceleration=(std::numeric_limits<double>::max)();
    for(unsigned axis=0U;axis<6U;++axis){
        if(!std::isfinite(nativePerMetric[axis]))return false;
        const double ratio=std::fabs(nativePerMetric[axis]);
        if(ratio==0.0)continue;
        if(!(budget.mask&(1U<<axis)))return false;
        result.activeMask|=1U<<axis;
        result.velocity=(std::min)(result.velocity,Quotient(budget.velocity[axis],ratio));
        result.acceleration=(std::min)(result.acceleration,Quotient(budget.acceleration[axis],ratio));
        result.deceleration=(std::min)(result.deceleration,Quotient(budget.deceleration[axis],ratio));
    }
    if(!result.activeMask||!Positive(result.velocity)||!Positive(result.acceleration)||!Positive(result.deceleration))return false;
    output=result;return true;
}
inline bool LinearDistance(const NCAxisIdentitySnapshot& identity,const std::array<double,8U>& start,
    const std::array<double,8U>& end,double& output) noexcept
{
    output=0.0;if(!IsNCAxisIdentitySnapshotValid(identity)||(ExistingMask(identity)&7U)!=7U)return false;
    for(unsigned axis=0U;axis<3U;++axis)
        if(identity.nativeUnit[axis]!=1U||identity.axisType[axis]!=0U)return false;
    double length=0.0;
    for(unsigned axis=0U;axis<8U;++axis){
        if(!std::isfinite(start[axis])||!std::isfinite(end[axis]))return false;
        const double delta=end[axis]-start[axis];if(!std::isfinite(delta))return false;
        if(axis>=6U||identity.exists[axis]==0U){if(std::memcmp(&start[axis],&end[axis],sizeof(double))!=0)return false;continue;}
        if(identity.nativeUnit[axis]==1U)length=std::hypot(length,delta);
    }
    if(!std::isfinite(length))return false;output=length;return true;
}
}
