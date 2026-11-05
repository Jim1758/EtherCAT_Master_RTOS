#pragma once
#include "NCMachiningTime.h"
#include "NCG76OrbitCompletion.h"
#include "NCG76OscillationMode.h"
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM366: admitted G76 terminal observations and run totals survive the next NC parent.
// No motion, timer, source admission or endpoint permission is derived here.
namespace NCG76DHBatchDiagnostic
{
constexpr unsigned Capacity=16U;
inline bool SupportedMode(unsigned modeP) noexcept
{return NCG76Mode::IsFixedRadius(modeP)||NCG76Mode::IsTaper(modeP)||NCG76Mode::IsHoleExpansion(modeP)||modeP==50U;}
inline bool SupportedRetreatMode(unsigned modeP,unsigned retreatMode) noexcept
{return SupportedMode(modeP)&&(retreatMode==0U||retreatMode==2U||
    (modeP==50U&&(retreatMode==3U||retreatMode==4U)));}
enum class Result : std::uint8_t { Complete=1U,TimeLimit=2U,TimedComplete=3U,RetryExhausted=4U };
struct Scope
{
    std::uint64_t run=0ULL,cache=0ULL,epoch=0ULL,ownerGeneration=0ULL;
    std::uint32_t owner=0U;
};
inline bool Valid(const Scope& s) noexcept
{return s.run&&s.cache&&s.epoch&&s.ownerGeneration&&s.owner;}
inline bool Same(const Scope& a,const Scope& b) noexcept
{return a.run==b.run&&a.cache==b.cache&&a.epoch==b.epoch&&
    a.ownerGeneration==b.ownerGeneration&&a.owner==b.owner;}
struct Row
{
    std::uint64_t session=0ULL,dispatch=0ULL;
    std::uint64_t totalLimitUs=0ULL,additionalUs=0ULL,totalElapsedUs=0ULL,additionalElapsedUs=0ULL,flushes=0ULL;
    int pc=-1,line=0,sequenceN=-1;
    unsigned modeP=0U,polygonSides=0U,retreatMode=0U;
    bool hasD=false,hasH=false,hasL=false,inheritedB=false,endpointCommitted=false;
    std::uint64_t orbitAttemptCount=0ULL;
    std::uint32_t orbitRequiredPasses=0U,orbitQualifiedPasses=0U;
    std::uint32_t orbitPercentScaled=0U,orbitTargetPercentScaled=0U,orbitCompletionState=0U;
    bool orbitActive=false,orbitPercentAvailable=false;
    Result result=Result::Complete;
    // Existing Row padding; zero preserves the released unlimited policy.
    std::uint8_t orbitRetryLimit=0U;
    NCG76OrbitCompletion::AttemptEvidence orbitEvaluation{};
};
inline bool Same(const Row& a,const Row& b) noexcept
{
    return a.session==b.session&&a.dispatch==b.dispatch&&a.totalLimitUs==b.totalLimitUs&&
        a.additionalUs==b.additionalUs&&a.totalElapsedUs==b.totalElapsedUs&&
        a.additionalElapsedUs==b.additionalElapsedUs&&a.flushes==b.flushes&&a.pc==b.pc&&
        a.line==b.line&&a.sequenceN==b.sequenceN&&a.modeP==b.modeP&&a.polygonSides==b.polygonSides&&
        a.retreatMode==b.retreatMode&&a.hasD==b.hasD&&a.hasH==b.hasH&&a.inheritedB==b.inheritedB&&
        a.endpointCommitted==b.endpointCommitted&&a.result==b.result&&a.hasL==b.hasL&&
        a.orbitAttemptCount==b.orbitAttemptCount&&a.orbitRequiredPasses==b.orbitRequiredPasses&&
        a.orbitQualifiedPasses==b.orbitQualifiedPasses&&a.orbitPercentScaled==b.orbitPercentScaled&&
        a.orbitTargetPercentScaled==b.orbitTargetPercentScaled&&a.orbitCompletionState==b.orbitCompletionState&&
        a.orbitActive==b.orbitActive&&a.orbitPercentAvailable==b.orbitPercentAvailable&&
        a.orbitRetryLimit==b.orbitRetryLimit&&
        NCG76OrbitCompletion::Same(a.orbitEvaluation,b.orbitEvaluation);
}
inline bool ValidOrbit(const Row& r) noexcept
{
    if(!NCG76OrbitCompletion::ValidConfig(r.orbitRequiredPasses,r.orbitTargetPercentScaled)||
       (!r.hasL&&r.orbitRequiredPasses)||r.orbitQualifiedPasses>r.orbitRequiredPasses||
       r.orbitAttemptCount<r.orbitQualifiedPasses||r.orbitPercentScaled>NCG76OrbitCompletion::FullPercent||
       r.orbitRetryLimit>99U||(!r.orbitRequiredPasses&&r.orbitRetryLimit)||
       !NCG76OrbitCompletion::ValidAttemptEvidence(r.orbitEvaluation,r.orbitAttemptCount,
           r.orbitRequiredPasses,r.orbitTargetPercentScaled,r.orbitQualifiedPasses,
           r.result==Result::TimeLimit))return false;
    if(!r.orbitRequiredPasses)return r.result!=Result::RetryExhausted&&r.orbitQualifiedPasses==0U&&r.orbitAttemptCount==0ULL&&
        r.orbitCompletionState==0U&&!r.orbitActive&&!r.orbitPercentAvailable&&r.orbitPercentScaled==0U;
    // Rows are recorded only after the original qualified terminal proof.
    if(r.orbitActive)return false;
    if(r.result==Result::RetryExhausted)return r.orbitRetryLimit!=0U&&
        r.orbitCompletionState==7U&&r.orbitPercentAvailable&&r.orbitQualifiedPasses==0U&&
        r.orbitEvaluation.lastAvailable&&!r.orbitEvaluation.lastAccepted&&
        r.orbitEvaluation.failedAttempts==static_cast<std::uint64_t>(r.orbitRetryLimit)+1ULL&&
        r.orbitPercentScaled==r.orbitEvaluation.lastPercentScaled&&
        r.orbitPercentScaled<r.orbitTargetPercentScaled;
    if(r.orbitRetryLimit&&r.orbitEvaluation.failedAttempts>r.orbitRetryLimit)return false;
    if(r.result==Result::Complete)return r.orbitCompletionState==4U&&r.orbitPercentAvailable&&
        r.orbitQualifiedPasses==r.orbitRequiredPasses&&r.orbitPercentScaled>=r.orbitTargetPercentScaled&&
        r.orbitPercentScaled==r.orbitEvaluation.lastPercentScaled;
    if(r.result==Result::TimedComplete)return r.orbitCompletionState==6U&&
        !r.orbitPercentAvailable&&r.orbitPercentScaled==0U;
    return r.result==Result::TimeLimit&&r.orbitCompletionState==5U&&
        !r.orbitPercentAvailable&&r.orbitPercentScaled==0U;
}
inline bool Valid(const Row& r) noexcept
{
    if(!r.session||!r.dispatch||r.pc<0||r.line<=0||r.sequenceN< -1||!SupportedMode(r.modeP)||
       !SupportedRetreatMode(r.modeP,r.retreatMode)||(!r.hasD&&!r.hasH&&!r.hasL)||!ValidOrbit(r)||
       (!r.hasD&&r.totalLimitUs)||(!r.hasH&&r.additionalUs)||
       !NCMachiningTime::ValidConfig({r.totalLimitUs,r.additionalUs})||
       r.totalElapsedUs>r.totalLimitUs||r.additionalElapsedUs>r.additionalUs||
       (NCG76Mode::IsPolygon(r.modeP)?!NCG76Mode::ValidPolygonSides(r.polygonSides):r.polygonSides!=0U))return false;
    if(r.result==Result::Complete||r.result==Result::RetryExhausted)return r.endpointCommitted&&r.additionalElapsedUs==r.additionalUs&&
        (!r.totalLimitUs||r.totalElapsedUs<r.totalLimitUs);
    if(r.result==Result::TimedComplete)return r.endpointCommitted&&r.hasD&&r.totalLimitUs&&r.totalElapsedUs==r.totalLimitUs;
    return r.result==Result::TimeLimit&&!r.endpointCommitted&&r.totalLimitUs&&r.totalElapsedUs==r.totalLimitUs;
}
// Fixed run totals are separate from the bounded last-sixteen-row window.
// Mechanical endpoint completion and the process inspection result remain
// distinct: a timed end or disabled inspection is not a passed inspection.
struct Totals
{
    std::uint64_t completedParents=0ULL,inspectionPassed=0ULL,inspectionDisabled=0ULL;
    std::uint64_t timedCompleted=0ULL,retryExhausted=0ULL,attempts=0ULL,rejectedFullSweeps=0ULL;
};
static_assert(sizeof(Row)==144U,"The retry limit uses existing diagnostic Row padding");
static_assert(sizeof(Totals)==56U,"Run totals remain bounded value-only data");
inline bool Same(const Totals& a,const Totals& b) noexcept
{
    return a.completedParents==b.completedParents&&a.inspectionPassed==b.inspectionPassed&&
        a.inspectionDisabled==b.inspectionDisabled&&a.timedCompleted==b.timedCompleted&&
        a.retryExhausted==b.retryExhausted&&a.attempts==b.attempts&&
        a.rejectedFullSweeps==b.rejectedFullSweeps;
}
inline bool CheckedAdd(std::uint64_t& value,std::uint64_t increment) noexcept
{
    if(increment>(std::numeric_limits<std::uint64_t>::max)()-value)return false;
    value+=increment;return true;
}
inline bool Accumulate(const Row& row,Totals& totals) noexcept
{
    // Legacy alarm exits do not have a committed completed endpoint.
    if(row.result==Result::TimeLimit)return true;
    if(!CheckedAdd(totals.completedParents,1ULL)||
       !CheckedAdd(totals.attempts,row.orbitAttemptCount)||
       !CheckedAdd(totals.rejectedFullSweeps,row.orbitEvaluation.failedAttempts))return false;
    if(row.result==Result::RetryExhausted)return CheckedAdd(totals.retryExhausted,1ULL);
    if(row.result==Result::TimedComplete)return CheckedAdd(totals.timedCompleted,1ULL);
    return row.orbitRequiredPasses?CheckedAdd(totals.inspectionPassed,1ULL):
        CheckedAdd(totals.inspectionDisabled,1ULL);
}
struct Batch
{
    Scope scope{};
    std::array<Row,Capacity> rows{};
    unsigned count=0U;
    std::uint64_t dropped=0ULL;
    bool invalid=false;
    Totals totals{};
    // Sessions strictly increase in one bound RT/NC run. Keep a floor after
    // eviction so replaying an old row cannot count it as a new terminal.
    std::uint64_t lastRecordedSession=0ULL;
    void Clear() noexcept
    {
        scope=Scope{};count=0U;dropped=0ULL;invalid=false;
        totals=Totals{};lastRecordedSession=0ULL;
    }
    void Bind(const Scope& next) noexcept
    {
        // Only [0,count) is observable. Reset metadata in place instead of
        // constructing a full Batch temporary on the NC thread's stack.
        if(!Valid(next)){scope=Scope{};count=0U;dropped=0ULL;invalid=true;
            totals=Totals{};lastRecordedSession=0ULL;return;}
        if(!Same(scope,next)){scope=next;count=0U;dropped=0ULL;invalid=false;
            totals=Totals{};lastRecordedSession=0ULL;}
    }
    bool Append(const Row& row) noexcept
    {
        if(invalid||!Valid(scope)||!Valid(row)||count>Capacity){invalid=true;return false;}
        for(unsigned i=0U;i<count;++i)if(rows[i].session==row.session){
            if(!Same(rows[i],row))invalid=true;
            return !invalid;
        }
        if(row.session<=lastRecordedSession){invalid=true;return false;}
        Totals next=totals;
        if(!Accumulate(row,next)||(count==Capacity&&dropped==(std::numeric_limits<std::uint64_t>::max)()))
        {invalid=true;return false;}
        if(count==Capacity){
            for(unsigned i=1U;i<Capacity;++i)rows[i-1U]=rows[i];
            --count;
            ++dropped;
        }
        rows[count++]=row;totals=next;lastRecordedSession=row.session;return true;
    }
};
static_assert(std::is_trivially_copyable<Batch>::value,"NC batch observations are value-only data");
}
