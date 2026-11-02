#pragma once
#include <cmath>
#include <cstdint>

// EDM306: E6 and E22 each contain at most six decimal places. Multiply their
// exact fixed-point integers, retaining the complete 1e-12 mm product rather
// than rounding a deep excursion to an E-field's six decimal places.
namespace NCG76SphereDeepFlush
{
inline bool ResolveHeight(double base,double multiplier,double& height,double maximumHeightMm=2.0) noexcept
{
    height=0.0;
    // Only the established P50 envelope and the separately admitted EDM308
    // parameter-feed envelope are supported. The caller cannot invent a
    // larger numeric scope by supplying an arbitrary fourth argument.
    if((maximumHeightMm!=2.0&&maximumHeightMm!=10.0)||
       !std::isfinite(base)||base<=0.0||base>maximumHeightMm||!std::isfinite(multiplier)||
       multiplier<=0.0||multiplier>20.0)return false;
    const double baseRounded=std::round(base*1000000.0);
    const double multiplierRounded=std::round(multiplier*1000000.0);
    if(baseRounded<1.0||multiplierRounded<1.0||
       baseRounded/1000000.0!=base||multiplierRounded/1000000.0!=multiplier)return false;
    const auto baseInteger=static_cast<std::uint64_t>(baseRounded);
    const auto multiplierInteger=static_cast<std::uint64_t>(multiplierRounded);
    // Proven maxima 10e6 * 20e6 are below both UINT64_MAX and 2^53.
    const std::uint64_t product=baseInteger*multiplierInteger;
    const std::uint64_t maximumProduct=maximumHeightMm==10.0?10000000000000ULL:2000000000000ULL;
    if(!product||product>maximumProduct)return false;
    height=static_cast<double>(product)/1000000000000.0;
    return std::isfinite(height)&&height>0.0&&height<=maximumHeightMm;
}
}
