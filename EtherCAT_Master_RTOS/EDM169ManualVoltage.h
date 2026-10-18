#pragma once
#include "EDM130WorkTimer.h"
#include <cstdint>
#include <limits>
#include <type_traits>

// Manual voltage intent only. No Motion/IO handle, trajectory, cursor or
// start authority is present. The existing RT producer creates actual samples.
namespace EDM169
{
constexpr std::uint64_t MaximumPublisherAgeUs=100000ULL;
constexpr std::uint64_t MaximumRTClockStepUs=2000ULL;
struct Scope
{
    std::uint64_t session=0ULL,run=0ULL,cache=0ULL,armEpoch=0ULL,ownerGeneration=0ULL;
    EDM130::Recipe recipe{};
};
inline bool ValidVoltageV(double value) noexcept
{return std::isfinite(value)&&value>=0.0&&value<=300.0;}
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.session==b.session&&a.run==b.run&&a.cache==b.cache&&a.armEpoch==b.armEpoch&&
        a.ownerGeneration==b.ownerGeneration&&EDM130::SameRecipe(a.recipe,b.recipe);
}
inline bool ValidScope(const Scope& scope) noexcept
{
    return scope.session&&scope.run&&scope.cache&&scope.armEpoch&&scope.ownerGeneration&&
        scope.recipe.diagnosticControlSource==5U&&!scope.recipe.commissioningProfile&&
        !scope.recipe.flushHoldTest&&EDM130::HasBoundProgramPlan(scope.recipe)&&
        EDM130::ValidHistoryRecipeBinding(EDM149::ProgramHistoryFixture,scope.recipe);
}
struct Request
{
    Scope scope{};
    // requestSequence advances for NC heartbeats. inputSequence advances only
    // for an accepted operator write; an unchanged human value has no expiry.
    std::uint64_t requestSequence=0ULL,inputSequence=0ULL,issuedTick=0ULL,issuedUs=0ULL;
    double voltageV=0.0;
};
inline bool ValidRequest(const Request& request) noexcept
{
    return ValidScope(request.scope)&&request.requestSequence&&request.issuedTick&&request.issuedUs&&
        ValidVoltageV(request.voltageV);
}
struct Feedback
{
    Scope scope{};
    std::uint64_t requestSequence=0ULL,inputSequence=0ULL,tick=0ULL,nowUs=0ULL;
    std::uint64_t issuedTick=0ULL,issuedUs=0ULL,publisherAgeUs=0ULL;
    double voltageV=0.0;
    bool enabled=false,active=false,faulted=false;
};
class Receiver
{
public:
    bool Begin(const Scope& scope,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        Clear();
        if(!ValidScope(scope)||!tick||!nowUs)return Fail();
        feedback_.scope=scope;feedback_.tick=feedback_.issuedTick=tick;
        feedback_.nowUs=feedback_.issuedUs=nowUs;
        feedback_.voltageV=EDM130::FrozenGapReferenceVoltageV(scope.recipe);
        feedback_.enabled=feedback_.active=true;return true;
    }
    void Clear() noexcept {*this=Receiver{};}
    bool Enabled() const noexcept {return feedback_.enabled;}
    bool Faulted() const noexcept {return feedback_.faulted;}
    double ValueV() const noexcept
    {return feedback_.enabled&&feedback_.active&&!feedback_.faulted?feedback_.voltageV:
        (std::numeric_limits<double>::quiet_NaN)();}
    std::uint64_t Sequence() const noexcept {return feedback_.requestSequence;}
    std::uint64_t InputSequence() const noexcept {return feedback_.inputSequence;}
    const Scope& FrozenScope() const noexcept {return feedback_.scope;}
    const Feedback& Current() const noexcept {return feedback_;}
    bool Tick(std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!feedback_.enabled||!feedback_.active||feedback_.faulted)return false;
        const bool same=tick==feedback_.tick&&nowUs==feedback_.nowUs;
        const bool next=feedback_.tick!=(std::numeric_limits<std::uint64_t>::max)()&&
            tick==feedback_.tick+1ULL&&nowUs>feedback_.nowUs&&
            nowUs-feedback_.nowUs<=MaximumRTClockStepUs;
        if(!tick||!nowUs||(!same&&!next)||nowUs<feedback_.issuedUs)return Fail();
        feedback_.tick=tick;feedback_.nowUs=nowUs;
        feedback_.publisherAgeUs=nowUs-feedback_.issuedUs;
        if(feedback_.publisherAgeUs>MaximumPublisherAgeUs)return Fail();
        return true;
    }
    bool Accept(const Request& request,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        // Invalid requests never change the latched setpoint or extend its
        // lease. The RT clock still progresses and eventually expires it.
        if(!Tick(tick,nowUs)||!ValidRequest(request)||!SameScope(request.scope,feedback_.scope)||
            request.issuedTick>tick||request.issuedUs>nowUs||
            nowUs-request.issuedUs>MaximumPublisherAgeUs||
            request.issuedTick<feedback_.issuedTick||request.issuedUs<feedback_.issuedUs||
            request.requestSequence<feedback_.requestSequence||request.inputSequence<feedback_.inputSequence)return false;
        if(request.requestSequence==feedback_.requestSequence)
            return request.inputSequence==feedback_.inputSequence&&request.issuedTick==feedback_.issuedTick&&
                request.issuedUs==feedback_.issuedUs&&request.voltageV==feedback_.voltageV;
        if(request.inputSequence==feedback_.inputSequence&&request.voltageV!=feedback_.voltageV)return false;
        feedback_.requestSequence=request.requestSequence;feedback_.inputSequence=request.inputSequence;
        feedback_.issuedTick=request.issuedTick;feedback_.issuedUs=request.issuedUs;
        feedback_.voltageV=request.voltageV;feedback_.publisherAgeUs=nowUs-request.issuedUs;
        return true;
    }
private:
    bool Fail() noexcept {feedback_.faulted=true;feedback_.active=false;return false;}
    Feedback feedback_{};
};
static_assert(std::is_trivially_copyable<Scope>::value&&std::is_trivially_copyable<Request>::value&&
    std::is_trivially_copyable<Feedback>::value,"Manual voltage uses fixed cross-thread values.");
static_assert(sizeof(Scope)==304U&&sizeof(Request)==344U&&sizeof(Feedback)<=384U,
    "EDM169 manual voltage publications stay bounded without changing Recipe.");
}
