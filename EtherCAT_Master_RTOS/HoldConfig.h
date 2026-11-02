#pragma once
#include <cmath>
#include <cstdint>
#include <string>

// EDM331: boot-only, independent HOLD inspection/recovery profiles.
// All feeds are physical mm/min; ramps are seconds. Runtime copies a frozen
// profile at admission and still enforces every participating axis limit.
namespace NCHoldRecovery
{
struct Config
{
    double retreatFeedMmMin=5.0, resumeFeedMmMin=5.0, jogRecoveryFeedMmMin=5.0;
    double retreatAccelSeconds=0.25, retreatDecelSeconds=0.25;
    double resumeAccelSeconds=0.25, resumeDecelSeconds=0.25;
    double jogAccelSeconds=0.25, jogDecelSeconds=0.25;
    std::uint32_t historyMaxPrevious=15U;
};
inline bool Valid(const Config& c) noexcept
{
    if(c.historyMaxPrevious!=15U)return false;
    const double feeds[]={c.retreatFeedMmMin,c.resumeFeedMmMin,c.jogRecoveryFeedMmMin};
    const double accel[]={c.retreatAccelSeconds,c.resumeAccelSeconds,c.jogAccelSeconds};
    const double decel[]={c.retreatDecelSeconds,c.resumeDecelSeconds,c.jogDecelSeconds};
    for(unsigned i=0U;i<3U;++i){
        if(!std::isfinite(feeds[i])||feeds[i]<=0.0||!std::isfinite(accel[i])||accel[i]<=0.0||
           !std::isfinite(decel[i])||decel[i]<=0.0)return false;
        const double velocity=feeds[i]/60.0;
        if(!std::isfinite(velocity)||velocity<=0.0||!std::isfinite(velocity/accel[i])||
           velocity/accel[i]<=0.0||!std::isfinite(velocity/decel[i])||velocity/decel[i]<=0.0)return false;
    }
    return true;
}
inline bool SameConfig(const Config& a,const Config& b) noexcept
{
    return a.retreatFeedMmMin==b.retreatFeedMmMin&&a.resumeFeedMmMin==b.resumeFeedMmMin&&
        a.jogRecoveryFeedMmMin==b.jogRecoveryFeedMmMin&&a.retreatAccelSeconds==b.retreatAccelSeconds&&
        a.retreatDecelSeconds==b.retreatDecelSeconds&&a.resumeAccelSeconds==b.resumeAccelSeconds&&
        a.resumeDecelSeconds==b.resumeDecelSeconds&&a.jogAccelSeconds==b.jogAccelSeconds&&
        a.jogDecelSeconds==b.jogDecelSeconds&&a.historyMaxPrevious==b.historyMaxPrevious;
}
// Plain lowerCamel key=value, no sections. Missing fields inherit defaults.
// Both functions are non-RT and transactional: failure leaves output intact.
bool ParseText(const std::string& text,Config& output,std::string& errorKey);
// A missing file uses defaults. An existing file is read, never rewritten.
bool LoadFile(const std::string& path,Config& output,std::string& errorKey);
}
