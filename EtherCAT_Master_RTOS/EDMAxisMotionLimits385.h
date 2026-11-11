#pragma once
#include "EDMCoordinatedAxes384.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace EDM385
{
using Native6=std::array<double,6U>;
// Native acceleration units: mm/s^2 for linear, degrees/s^2 for rotary.
// Normal signed EDM uses a speed-only native envelope, independent of G00
// and Stop time settings. Zero adds no mode acceleration ceiling; positive
// values are EDM-specific native limits. No motion authority.
struct AxisLimits
{
    Native6 machiningAcceleration{},machiningDeceleration{};
    Native6 flushAcceleration{},flushDeceleration{};
};
// The servo interval is 250 us. A full +MAX to -MAX change over one interval
// is the largest possible finite velocity difference under MAX_Speed alone.
// This is a numerical envelope, not an inherited acceleration/ramp setting.
// Flush profile times and positive EDM mode limits may impose a lower slew.
constexpr double IndependentCycleSeconds=0.00025;
inline bool BuildIndependentBudget(const Native6& velocity,std::uint32_t mask,EDM384Axes::Budget& out) noexcept
{
    out=EDM384Axes::Budget{};
    if(!mask||(mask&~63U))return false;
    out.mask=mask;
    for(unsigned a=0U;a<6U;++a)if(mask&(1U<<a)){
        const double v=velocity[a];
        if(!std::isfinite(v)||v<=0.0)return false;
        const double rate=(v/IndependentCycleSeconds)*2.0;
        if(!std::isfinite(rate)||rate<=0.0)return false;
        out.velocity[a]=v;out.acceleration[a]=rate;out.deceleration[a]=rate;
    }
    return EDM384Axes::Valid(out);
}
inline bool Valid(const AxisLimits& limits) noexcept
{
    for(unsigned a=0U;a<6U;++a)
        for(double v:{limits.machiningAcceleration[a],limits.machiningDeceleration[a],limits.flushAcceleration[a],limits.flushDeceleration[a]})
            if(!std::isfinite(v)||v<0.0)return false;
    return true;
}
inline bool Same(const AxisLimits& a,const AxisLimits& b) noexcept
{
    for(unsigned i=0U;i<6U;++i)
        if(std::memcmp(&a.machiningAcceleration[i],&b.machiningAcceleration[i],sizeof(double))||
           std::memcmp(&a.machiningDeceleration[i],&b.machiningDeceleration[i],sizeof(double))||
           std::memcmp(&a.flushAcceleration[i],&b.flushAcceleration[i],sizeof(double))||
           std::memcmp(&a.flushDeceleration[i],&b.flushDeceleration[i],sizeof(double)))return false;
    return true;
}
inline bool Empty(const AxisLimits& limits) noexcept
{
    for(unsigned a=0U;a<6U;++a)
        if(limits.machiningAcceleration[a]!=0.0||limits.machiningDeceleration[a]!=0.0||
           limits.flushAcceleration[a]!=0.0||limits.flushDeceleration[a]!=0.0)return false;
    return true;
}
inline double Ceiling(double installed,double requested) noexcept
{return requested>0.0?(std::min)(installed,requested):installed;}
inline bool ResolveBudget(const EDM384Axes::Budget& installed,const AxisLimits& limits,bool flush,EDM384Axes::Budget& out) noexcept
{
    out=EDM384Axes::Budget{};
    if(!EDM384Axes::Valid(installed)||!Valid(limits))return false;
    out=installed;
    for(unsigned a=0U;a<6U;++a)if(installed.mask&(1U<<a)){
        out.acceleration[a]=Ceiling(installed.acceleration[a],flush?limits.flushAcceleration[a]:limits.machiningAcceleration[a]);
        out.deceleration[a]=Ceiling(installed.deceleration[a],flush?limits.flushDeceleration[a]:limits.machiningDeceleration[a]);
    }
    return EDM384Axes::Valid(out);
}
}
