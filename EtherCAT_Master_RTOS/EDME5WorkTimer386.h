#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// E5-only voltage qualification. This helper never commands motion, changes
// the GAP servo/short detector, measures physical discharge, or owns E5 time.
namespace EDM386E5
{
struct Profile
{
    std::uint32_t mode=0U;
    std::uint32_t gapFilterTimeMs=3U,startConfirmTimeMs=10U,abnormalGapPauseTimeMs=100U;
    double gapLowVoltageV=10.0,gapHighVoltageV=70.0;
};
inline bool Valid(const Profile& p) noexcept
{
    return p.mode<=1U&&std::isfinite(p.gapLowVoltageV)&&std::isfinite(p.gapHighVoltageV)&&
        p.gapLowVoltageV>=0.0&&p.gapLowVoltageV<p.gapHighVoltageV&&p.gapHighVoltageV<=300.0;
}
inline bool Same(const Profile& a,const Profile& b) noexcept
{
    return a.mode==b.mode&&a.gapFilterTimeMs==b.gapFilterTimeMs&&a.startConfirmTimeMs==b.startConfirmTimeMs&&
        a.abnormalGapPauseTimeMs==b.abnormalGapPauseTimeMs&&
        std::memcmp(&a.gapLowVoltageV,&b.gapLowVoltageV,sizeof(double))==0&&
        std::memcmp(&a.gapHighVoltageV,&b.gapHighVoltageV,sizeof(double))==0;
}
inline std::uint64_t Microseconds(std::uint32_t ms) noexcept
{return static_cast<std::uint64_t>(ms)*1000ULL;}

enum class State : std::uint8_t { Legacy=0U,Waiting,Confirming,Running,Paused,Suspended,Invalid };
inline const char* StateName(State s) noexcept
{
    switch(s){case State::Legacy:return "LEGACY";case State::Waiting:return "WAITING";
    case State::Confirming:return "CONFIRMING";case State::Running:return "RUNNING";
    case State::Paused:return "PAUSED";case State::Suspended:return "SUSPENDED";
    case State::Invalid:return "INVALID";default:return "UNKNOWN";}
}
// Logical interval state only. No sample, filter, tick, electrical permit or
// source freshness crosses a parent boundary. Partial first confirmation is
// deliberately restarted because continuity across that boundary is unproved.
struct Evidence
{
    std::uint64_t abnormalUs=0ULL;
    bool latched=false,paused=false;
};
inline bool ValidEvidence(const Profile& p,const Evidence& e) noexcept
{
    if(!Valid(p))return false;
    if(p.mode==0U)return e.abnormalUs==0ULL&&!e.latched&&!e.paused;
    if(!e.latched)return e.abnormalUs==0ULL&&!e.paused;
    const auto limit=Microseconds(p.abnormalGapPauseTimeMs);
    if(e.abnormalUs>limit)return false;
    if(e.paused)return e.abnormalUs==limit;
    return limit==0ULL||e.abnormalUs<limit;
}
struct Input
{
    std::uint64_t tick=0ULL,deltaUs=0ULL;
    double voltageV=0.0,shortThresholdV=0.0;
    bool sourceValid=false,active=false;
};
struct Feedback
{
    std::uint64_t waitEvents=0ULL,startEvents=0ULL,pauseEvents=0ULL,resumeEvents=0ULL;
    std::uint64_t lastEventTick=0ULL,confirmUs=0ULL,abnormalUs=0ULL;
    double filteredVoltageV=0.0;
    State state=State::Invalid;
    bool allow=false,latched=false,paused=false,filterReady=false,valid=false;
    bool rawInBand=false,filteredInBand=false;
};
static_assert(sizeof(Profile)==32U,"E5 gate configuration remains a bounded native profile");
static_assert(sizeof(Evidence)==16U&&std::is_trivially_copyable<Evidence>::value,"E5 logical continuation is bounded");
static_assert(sizeof(Feedback)<=80U&&std::is_trivially_copyable<Feedback>::value,"E5 diagnostic snapshot remains bounded");

class Gate
{
public:
    bool Begin(const Profile& profile,const Evidence* retained=nullptr) noexcept
    {
        profile_=profile;feedback_=Feedback{};active_=false;ClearFilter();lastTick_=0ULL;
        if(!Valid(profile)||(retained&&!ValidEvidence(profile,*retained)))return false;
        active_=true;feedback_.valid=true;
        if(profile.mode==0U){feedback_.state=State::Legacy;feedback_.allow=true;return true;}
        // Mode 1 now uses voltage only to start an elapsed-time round. A
        // proved retained latch does not acquire another voltage prerequisite.
        if(retained)feedback_.latched=retained->latched;
        feedback_.waitEvents=1ULL;
        feedback_.state=feedback_.paused?State::Paused:State::Waiting;
        return true;
    }
    // A completed or abandoned existing E5 excursion starts a fresh interval.
    // Keep monotonic event counters so a slower NC reader sees the transition.
    void Reset(std::uint64_t tick=0ULL) noexcept
    {
        if(!active_||!feedback_.valid)return;
        feedback_.allow=false;feedback_.latched=feedback_.paused=false;
        feedback_.confirmUs=feedback_.abnormalUs=0ULL;ClearFilter();lastTick_=0ULL;
        if(profile_.mode==0U){feedback_.state=State::Legacy;feedback_.allow=true;return;}
        feedback_.state=State::Waiting;
        if(!Increment(feedback_.waitEvents,tick))Withdraw(tick);
    }
    void Withdraw(std::uint64_t tick=0ULL) noexcept
    {
        active_=false;feedback_.valid=feedback_.allow=feedback_.latched=feedback_.paused=false;
        feedback_.state=State::Invalid;feedback_.confirmUs=feedback_.abnormalUs=0ULL;
        if(tick)feedback_.lastEventTick=tick;
        ClearFilter();lastTick_=0ULL;
    }
    const Feedback& Step(const Input& input) noexcept
    {
        if(!active_)return feedback_;
        // Mode 0 is intentionally unused by the integration's legacy path.
        // It does not introduce a second source or timing prerequisite.
        if(profile_.mode==0U)return feedback_;
        if(!input.tick||input.deltaUs>1500ULL||
           (lastTick_&&(lastTick_==(std::numeric_limits<std::uint64_t>::max)()||input.tick!=lastTick_+1ULL))||
           !input.sourceValid||!std::isfinite(input.voltageV)||input.voltageV<0.0||input.voltageV>300.0||
           !std::isfinite(input.shortThresholdV)||input.shortThresholdV<0.0||input.shortThresholdV>300.0)
        {Withdraw(input.tick);return feedback_;}
        const auto delta=lastTick_?input.deltaUs:0ULL;lastTick_=input.tick;
        const bool wasAllowed=feedback_.allow;
        if(!input.active){
            feedback_.allow=false;feedback_.confirmUs=0ULL;ClearFilter();
            feedback_.state=State::Suspended;
            if(wasAllowed&&feedback_.latched&&!Increment(feedback_.pauseEvents,input.tick))Withdraw(input.tick);
            return feedback_;
        }
        if(!filtered_){
            feedback_.filteredVoltageV=input.voltageV;filtered_=true;warmupUs_=0ULL;
        }else if(profile_.gapFilterTimeMs==0U){feedback_.filteredVoltageV=input.voltageV;}
        else{
            // Causal first-order low-pass: y += dt/(tau+dt) * (raw-y).
            // The fixed voltage domain and bounded dt make every operation finite.
            const double alpha=static_cast<double>(delta)/
                (static_cast<double>(Microseconds(profile_.gapFilterTimeMs))+static_cast<double>(delta));
            feedback_.filteredVoltageV+=alpha*(input.voltageV-feedback_.filteredVoltageV);
            warmupUs_=Saturate(warmupUs_,delta,Microseconds(profile_.gapFilterTimeMs));
        }
        feedback_.filterReady=profile_.gapFilterTimeMs==0U||warmupUs_>=Microseconds(profile_.gapFilterTimeMs);
        feedback_.rawInBand=InBand(input.voltageV)&&input.voltageV>input.shortThresholdV;
        feedback_.filteredInBand=InBand(feedback_.filteredVoltageV)&&feedback_.filteredVoltageV>input.shortThresholdV;
        const bool qualified=feedback_.filterReady&&feedback_.rawInBand&&feedback_.filteredInBand;
        if(qualified)freshQualificationRequired_=false;
        if(!feedback_.latched){
            if(qualified){
                if(previousQualified_)feedback_.confirmUs=Saturate(feedback_.confirmUs,delta,Microseconds(profile_.startConfirmTimeMs));
                if(feedback_.confirmUs>=Microseconds(profile_.startConfirmTimeMs)){
                    feedback_.latched=true;feedback_.paused=false;feedback_.abnormalUs=0ULL;
                    if(!Increment(feedback_.startEvents,input.tick)){Withdraw(input.tick);return feedback_;}
                }
            }else feedback_.confirmUs=0ULL;
            feedback_.allow=feedback_.latched;
            feedback_.state=feedback_.allow?State::Running:qualified?State::Confirming:State::Waiting;
            previousQualified_=qualified;previousAbnormal_=false;
            return feedback_;
        }
        // The first valid discharge confirmation latches this round. N-side
        // retreat, short voltage and out-of-window voltage no longer pause E5.
        // AbnormalGapPauseTimeMs remains a readable compatibility parameter.
        feedback_.abnormalUs=0ULL;feedback_.paused=false;feedback_.allow=true;
        if(!wasAllowed&&!Increment(feedback_.resumeEvents,input.tick)){Withdraw(input.tick);return feedback_;}
        feedback_.state=State::Running;previousAbnormal_=false;
        previousQualified_=qualified;
        return feedback_;
    }
    void SuspendForTransportGap() noexcept
    {
        if (!active_ || !feedback_.valid || profile_.mode == 0U) return;
        feedback_.allow = false; feedback_.state = State::Suspended;
        feedback_.confirmUs = 0ULL; ClearFilter(); lastTick_ = 0ULL;
    }
    const Feedback& Current() const noexcept{return feedback_;}
    Evidence Export() const noexcept
    {
        Evidence e{};
        if(active_&&feedback_.valid&&profile_.mode==1U){e.abnormalUs=feedback_.abnormalUs;e.latched=feedback_.latched;e.paused=feedback_.paused;}
        return e;
    }
private:
    bool InBand(double value) const noexcept
    {return value>=profile_.gapLowVoltageV&&value<=profile_.gapHighVoltageV;}
    static std::uint64_t Saturate(std::uint64_t value,std::uint64_t delta,std::uint64_t limit) noexcept
    {return value>=limit||delta>=limit-value?limit:value+delta;}
    bool Increment(std::uint64_t& value,std::uint64_t tick) noexcept
    {
        if(value==(std::numeric_limits<std::uint64_t>::max)())return false;
        ++value;feedback_.lastEventTick=tick;return true;
    }
    void ClearFilter() noexcept
    {
        filtered_=previousQualified_=previousAbnormal_=false;freshQualificationRequired_=true;warmupUs_=0ULL;
        feedback_.filteredVoltageV=0.0;feedback_.filterReady=feedback_.rawInBand=feedback_.filteredInBand=false;
    }
    Profile profile_{};Feedback feedback_{};
    std::uint64_t lastTick_=0ULL,warmupUs_=0ULL;
    bool active_=false,filtered_=false,previousQualified_=false,previousAbnormal_=false,freshQualificationRequired_=true;
};
}
