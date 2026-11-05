#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

// EDM355: conservative geometric coverage for one closed inspection contour.
// A bin retains a qualified prefix only; a gap/reversal/qualification failure
// makes it bad for this attempt. Retracing cannot earn the same length twice.
// Input stations are fresh physically acknowledged scalar commands, never a
// predicted next command. This is process qualification, not part metrology.
namespace NCG76OrbitCompletion
{
constexpr unsigned BinCount=256U;
constexpr std::uint32_t PercentScale=1000000U,FullPercent=100U*PercentScale;
enum class State:std::uint8_t { Disabled,Armed,Inspecting,Retry,Passed,Fault,StoppedByTime,RetryExhausted };
constexpr unsigned MaximumRetryLimit=99U;
// EDM365: retained observations of completed whole inspection sweeps only.
// The runtime owns its failed-sweep counter independently. These observations
// cannot supply source, physical-settle or endpoint-release authority.
struct AttemptEvidence
{
    std::uint64_t failedAttempts=0ULL;
    std::uint32_t lastPercentScaled=0U,lastQualifiedBefore=0U,peakQualified=0U;
    bool lastAvailable=false,lastAccepted=false;
};
static_assert(sizeof(AttemptEvidence)==24U,"Completed inspection evidence remains fixed-size value data");
static_assert(std::is_trivially_copyable<AttemptEvidence>::value,"Inspection evidence cannot own runtime state");
inline bool Same(const AttemptEvidence& a,const AttemptEvidence& b) noexcept
{
    return a.failedAttempts==b.failedAttempts&&a.lastPercentScaled==b.lastPercentScaled&&
        a.lastQualifiedBefore==b.lastQualifiedBefore&&a.peakQualified==b.peakQualified&&
        a.lastAvailable==b.lastAvailable&&a.lastAccepted==b.lastAccepted;
}
inline bool Empty(const AttemptEvidence& e) noexcept {return Same(e,AttemptEvidence{});}
inline bool ValidConfig(unsigned rounds,std::uint32_t target,unsigned retryLimit=0U) noexcept
{return rounds==0U?target==0U&&retryLimit==0U:
    rounds<=99U&&target>=10U*PercentScale&&target<=FullPercent&&retryLimit<=MaximumRetryLimit;}
// Diagnostic consistency only. Legacy fault snapshots may predate this
// evidence; their canonical empty value means unknown, never a passed sweep.
inline bool ValidAttemptEvidence(const AttemptEvidence& e,std::uint64_t attempts,
    unsigned required,std::uint32_t target,unsigned qualified,bool allowLegacyUnavailable=false) noexcept
{
    if(!ValidConfig(required,target)||qualified>required||attempts<qualified)return false;
    if(!required||!attempts)return !qualified&&!attempts&&Empty(e);
    if(!e.lastAvailable)return allowLegacyUnavailable&&Empty(e);
    if(e.failedAttempts>attempts||e.lastPercentScaled>FullPercent||
       e.lastAccepted!=(e.lastPercentScaled>=target)||e.lastQualifiedBefore>=required||
       e.peakQualified>required||e.peakQualified<qualified||e.peakQualified<e.lastQualifiedBefore||
       qualified!=(e.lastAccepted?e.lastQualifiedBefore+1U:0U)||
       (!e.lastAccepted&&!e.failedAttempts))return false;
    const auto successful=attempts-e.failedAttempts;
    if(successful<qualified||successful<e.peakQualified||
       successful<static_cast<std::uint64_t>(e.lastQualifiedBefore)+(e.lastAccepted?1ULL:0ULL))return false;
    if(!e.failedAttempts)return e.lastAccepted&&attempts==qualified&&e.peakQualified==qualified;
    return true;
}
class Coverage
{
public:
    bool Begin(double length) noexcept
    {
        length_=length;qualified_=0.0;fullBins_=0U;bad_.fill(false);
        if(!std::isfinite(length)||length<=0.0){length_=0.0;end_.fill(0.0);return false;}
        for(unsigned i=0U;i<BinCount;++i)end_[i]=Boundary(i);
        return true;
    }
    bool Observe(double from,double to,bool qualified) noexcept
    {
        if(!std::isfinite(from)||!std::isfinite(to)||length_<=0.0||from<0.0||to<0.0||from>length_||to>length_)return false;
        if(to<from){std::swap(from,to);qualified=false;}
        const unsigned first=Index(from),last=Index(to);
        for(unsigned i=first;i<=last;++i){
            const double begin=Boundary(i),finish=Boundary(i+1U);
            const double a=(std::max)(from,begin),b=(std::min)(to,finish);
            if(!qualified){if(b>a||from==to)MarkBad(i);continue;}
            if(b<=a||bad_[i])continue;
            // Missing/unqualified travel is not fabricated as coverage.
            if(a>end_[i])MarkBad(i);
            else if(b>end_[i]){
                if(b==finish&&end_[i]!=finish)++fullBins_;
                qualified_+=b-end_[i];end_[i]=b;
            }
        }
        return true;
    }
    double QualifiedLength()const noexcept
    {return fullBins_==BinCount?length_:(std::max)(0.0,qualified_);}
    double Length()const noexcept{return length_;}
private:
    void MarkBad(unsigned i) noexcept
    {
        if(bad_[i])return;
        if(end_[i]==Boundary(i+1U))--fullBins_;
        qualified_-=end_[i]-Boundary(i);bad_[i]=true;
    }
    double Boundary(unsigned i)const noexcept{return i==BinCount?length_:length_*(static_cast<double>(i)/BinCount);}
    unsigned Index(double station)const noexcept
    {return station==length_?BinCount-1U:(std::min)(BinCount-1U,static_cast<unsigned>((station/length_)*BinCount));}
    std::array<double,BinCount> end_{};
    std::array<bool,BinCount> bad_{};
    double length_=0.0,qualified_=0.0;
    unsigned fullBins_=0U;
};
inline std::uint32_t Percent(double qualified,double total) noexcept
{
    if(!std::isfinite(qualified)||!std::isfinite(total)||qualified<0.0||total<=0.0)return 0U;
    if(qualified>=total)return FullPercent;
    return static_cast<std::uint32_t>((qualified/total)*FullPercent);
}
}
