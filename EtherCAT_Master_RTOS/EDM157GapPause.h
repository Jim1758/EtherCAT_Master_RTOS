#pragma once
#include "EDMSixAxisGapModeCoordinator.h"
#include <type_traits>

// A bounded transaction around the existing GAP traversal. No Motion/IO
// handle, cursor integration, owner transfer or physical proof is owned here.
namespace EDM157
{
constexpr unsigned RequiredStableCycles = 200U;
// EDM162 uses this same pause transaction for the bounded mainline program.
// EDM166/167 admit only their existing 2 mm E5 lift, never visible GAP retreat.
inline bool SupportsArm(unsigned arm) noexcept
{ return EDM149::IsFlushHoldTestArm(arm) || EDM149::IsPauseTestArm(arm); }
inline bool SupportsFixture(unsigned history, EDM116::Fixture policy) noexcept
{
    return (history == EDM149::ProgramHistoryFixture && policy == EDM116::Fixture::ModalProgram156) ||
        (history == EDM149::PauseHistoryFixture && policy == EDM116::Fixture::ModalPause157) ||
        (history == EDM149::XYZVisibleFlushHistoryFixture && policy == EDM116::Fixture::ModalXYZVisibleFlush159) ||
        (history == EDM149::SixVisibleFlushHistoryFixture && policy == EDM116::Fixture::ModalSixVisibleFlush167);
}
enum class Phase : std::uint8_t { Idle, Braking, Qualifying, Held, Resuming, Running, Rejected,
    Retreating, TopQualifying, Returning, ReturnQualifying, Rebraking, Requalifying };
struct Scope
{
    EDM126::Source authorizedSource{};
    std::uint64_t ownerGeneration = 0ULL;
    unsigned historyFixture = 0U;
    EDM130::Recipe recipe{};
    EDM116::Policy policy{};
};
inline bool SameScope(const Scope& a, const Scope& b) noexcept
{
    return EDM126::SameSource(a.authorizedSource,b.authorizedSource) && a.ownerGeneration==b.ownerGeneration &&
        a.historyFixture==b.historyFixture && EDM130::SameRecipe(a.recipe,b.recipe) && EDM116::SamePolicy(a.policy,b.policy);
}
inline bool ValidScope(const Scope& s) noexcept
{
    const auto& a=s.authorizedSource;
    return EDM126::ValidSource(a) && s.ownerGeneration && s.ownerGeneration==a.scope.ownerGeneration &&
        SupportsFixture(s.historyFixture,s.policy.fixture) && EDM130::ValidHistoryRecipeBinding(s.historyFixture,s.recipe) &&
        (!(EDM130::IsVisibleFlush159(s.recipe)||EDM130::IsVisibleFlush167(s.recipe))||s.recipe.holdRetreatMode==1U) &&
        a.ordinal<EDM130::ProgramRows(s.recipe) &&
        EDM116::PolicyMatchesBinding(s.policy,static_cast<unsigned>(a.ordinal),a.sourcePC,a.dispatch);
}
inline bool PauseModeAllowed(const Scope& scope,const EDM128::Feedback& e) noexcept
{
    if(!ValidScope(scope)||e.faulted||e.stopped||!EDM116::SamePolicy(e.policy,scope.policy))return false;
    if(e.phase==EDM128::Phase::Gap)return !EDM130::IsOneShotCommissioning(scope.recipe)&&
        !e.detached&&e.work.triggeredEvents==e.work.completedEvents;
    return scope.recipe.holdRetreatMode==1U&&
        (scope.historyFixture==EDM149::ProgramHistoryFixture||
            scope.historyFixture==EDM149::XYZVisibleFlushHistoryFixture||
            scope.historyFixture==EDM149::SixVisibleFlushHistoryFixture)&&EDM128::CanPauseTimedFlush(e)&&
        e.work.workTimeUs==scope.recipe.workTimeUs&&e.eventIndex==e.work.triggeredEvents&&
        (!(EDM130::IsVisibleFlush159(scope.recipe)||EDM130::IsVisibleFlush167(scope.recipe))||e.eventIndex==1ULL);
}
struct Feedback
{
    Scope scope{};
    std::uint64_t holdSequence=0ULL, startFloor=0ULL, resumeStartSequence=0ULL, rejectedStartSequence=0ULL;
    Phase phase=Phase::Idle;
    unsigned stableCycles=0U;
    EDM126::Source cursorSource{};
    double localMm=0.0, stationMm=0.0;
    unsigned historyDepth=0U;
    EDM126::Point positionPulse{};
    std::uint64_t tick=0ULL, nowUs=0ULL, resumeSeedTick=0ULL;
    // EDM163: original machining pose above is retained throughout the
    // excursion. The target below is resolved by the existing CNC history.
    EDM126::Cursor retreatTarget{};
    EDM126::Point retreatPositionPulse{};
    double retreatStationMm=0.0;
    bool anchorCaptured=false,retreatPlanned=false,retreatUnavailable=false;
    bool topCaptured=false,exactReturn=false,interrupted=false;
    // EDM164: this is the stopped flush pose, not a new machining anchor.
    // The coordinator retains the actual E5 anchor and target unchanged.
    bool flushPause=false;
    EDM128::Phase flushPhase=EDM128::Phase::Gap;
    std::uint64_t flushEventIndex=0ULL;
};
static_assert(std::is_trivially_copyable<Scope>::value && std::is_trivially_copyable<Feedback>::value,
    "EDM157 cross-thread receipts are fixed values.");
static_assert(sizeof(Feedback)<=1024U,"EDM157 bounded pause publication.");
inline bool ValidPauseContext(const Feedback& f) noexcept
{
    if(!f.flushPause)return !EDM130::IsOneShotCommissioning(f.scope.recipe)&&
        f.flushPhase==EDM128::Phase::Gap&&!f.flushEventIndex;
    return ValidScope(f.scope)&&f.scope.recipe.holdRetreatMode==1U&&
        (f.scope.historyFixture==EDM149::ProgramHistoryFixture||
            f.scope.historyFixture==EDM149::XYZVisibleFlushHistoryFixture||
            f.scope.historyFixture==EDM149::SixVisibleFlushHistoryFixture)&&EDM128::IsFlushPhase(f.flushPhase)&&
        f.flushEventIndex&&(!(EDM130::IsVisibleFlush159(f.scope.recipe)||EDM130::IsVisibleFlush167(f.scope.recipe))||f.flushEventIndex==1ULL)&&
        !f.retreatPlanned&&!f.retreatUnavailable&&!f.topCaptured&&!f.exactReturn;
}
inline bool SamePauseContext(const Feedback& a,const Feedback& b) noexcept
{return a.flushPause==b.flushPause&&a.flushPhase==b.flushPhase&&a.flushEventIndex==b.flushEventIndex;}
inline bool Holding(Phase p) noexcept
{ return p==Phase::Braking||p==Phase::Qualifying||p==Phase::Held||p==Phase::Resuming||
    p==Phase::Retreating||p==Phase::TopQualifying||p==Phase::Returning||p==Phase::ReturnQualifying||
    p==Phase::Rebraking||p==Phase::Requalifying; }
inline const char* PhaseName(Phase p) noexcept
{
    switch(p) {
    case Phase::Idle:return "IDLE";case Phase::Braking:return "BRAKING";case Phase::Qualifying:return "QUALIFYING";
    case Phase::Held:return "HELD";case Phase::Resuming:return "RESUMING";case Phase::Running:return "RUNNING";
    case Phase::Rejected:return "REJECTED";
    case Phase::Retreating:return "RETREATING";case Phase::TopQualifying:return "TOP_QUALIFYING";
    case Phase::Returning:return "RETURNING";case Phase::ReturnQualifying:return "RETURN_QUALIFYING";
    case Phase::Rebraking:return "REBRAKING";case Phase::Requalifying:return "REQUALIFYING";
    }return "UNKNOWN";
}
inline bool ValidMapped(const Scope& scope,const EDM126::Feedback& f) noexcept
{
    if(!ValidScope(scope)||!f.active||f.faulted||f.stopRequested||f.stopped||
        !EDM126::SameSource(f.authorizedSource,scope.authorizedSource)||!EDM126::ValidSource(f.cursorSource)||
        !EDM109::SameScope(f.cursorSource.scope,scope.authorizedSource.scope)||
        f.cursorSource.ordinal>scope.authorizedSource.ordinal||
        f.historyDepth!=scope.authorizedSource.ordinal-f.cursorSource.ordinal||
        !f.tick||!f.nowUs||!std::isfinite(f.localMm)||!std::isfinite(f.stationMm)||
        !std::isfinite(f.lengthMm)||f.lengthMm<=0.0||f.localMm<0.0||f.localMm>f.lengthMm||f.stationMm<0.0)return false;
    for(unsigned axis=0U;axis<6U;++axis)
        if(!std::isfinite(f.positionPulse[axis])||!std::isfinite(f.velocityPps[axis])||
            !std::isfinite(f.accelerationPps2[axis]))return false;
    return std::isfinite(f.velocityMmSec)&&std::isfinite(f.accelerationMmSec2);
}
class Transaction
{
public:
    const Feedback& Current() const noexcept {return feedback_;}
    void Clear() noexcept {*this=Transaction{};}
    bool DemandZero() const noexcept {return Holding(feedback_.phase);}
    bool FlushPause() const noexcept {return feedback_.flushPause;}
    bool PathReturn() const noexcept {return !FlushPause()&&!feedback_.scope.recipe.commissioningProfile&&
        feedback_.scope.recipe.holdRetreatMode==1U;}
    bool FlushContextCurrent(const EDM128::Feedback& e) const noexcept
    {
        return !FlushPause()||(PauseModeAllowed(feedback_.scope,e)&&e.phase==feedback_.flushPhase&&
            e.work.triggeredEvents==feedback_.flushEventIndex);
    }
    bool ZeroOutput() const noexcept
    {return feedback_.phase==Phase::Qualifying||feedback_.phase==Phase::Held||feedback_.phase==Phase::Resuming||
        feedback_.phase==Phase::TopQualifying||feedback_.phase==Phase::ReturnQualifying||feedback_.phase==Phase::Requalifying;}
    const EDM126::Cursor* PathTarget() const noexcept
    {
        return feedback_.phase==Phase::Retreating?&feedback_.retreatTarget:
            feedback_.phase==Phase::Returning?&anchorCursor_:nullptr;
    }
    bool NeedsRetreatPlan() const noexcept
    {return PathReturn()&&feedback_.phase==Phase::Qualifying&&feedback_.anchorCaptured&&
        !feedback_.retreatPlanned&&!feedback_.retreatUnavailable;}
    bool PlanRetreat(const EDM126::Cursor& target,const EDM126::Point& position,double station) noexcept
    {
        const double height=EDM130::FrozenJumpHeightMm(feedback_.scope.recipe);
        if(!NeedsRetreatPlan()||!EDM126::ValidSource(target.source)||
            !EDM109::SameScope(target.source.scope,feedback_.scope.authorizedSource.scope)||
            target.source.ordinal>feedback_.cursorSource.ordinal||!std::isfinite(target.localMm)||target.localMm<0.0||
            !std::isfinite(station)||station<0.0||station>=feedback_.stationMm||
            !EDM128::HeightClose(feedback_.stationMm-station,height))return false;
        for(double pulse:position)if(!std::isfinite(pulse))return false;
        feedback_.retreatTarget=target;feedback_.retreatPositionPulse=position;
        feedback_.retreatStationMm=station;feedback_.retreatPlanned=true;return true;
    }
    bool PlanRetreatUnavailable() noexcept
    {
        if(!NeedsRetreatPlan())return false;
        feedback_.retreatUnavailable=true;return true;
    }
    bool BeginHold(const Scope& scope,std::uint64_t holdSequence,std::uint64_t startFloor,
        std::uint64_t tick,std::uint64_t nowUs,const EDM126::Feedback& committed,
        const EDM128::Feedback* flush=nullptr) noexcept
    {
        if((EDM130::IsOneShotCommissioning(scope.recipe)&&!flush)||
            (Holding(feedback_.phase)&&!SameScope(scope,feedback_.scope))||
            (flush&&(!PauseModeAllowed(scope,*flush)||!EDM128::CanPauseTimedFlush(*flush)))||
            (DemandZero()&&FlushPause()&&(!flush||!FlushContextCurrent(*flush)))||
            (DemandZero()&&!FlushPause()&&flush)||
            startFloor<feedback_.startFloor||startFloor<feedback_.resumeStartSequence||startFloor<feedback_.rejectedStartSequence||
            !ValidMapped(scope,committed)||!holdSequence||holdSequence<=feedback_.holdSequence||
            !tick||!nowUs||tick<committed.tick||tick-committed.tick>1ULL||nowUs<committed.nowUs||
            nowUs-committed.nowUs>EDM126::MaximumSourceAgeUs||feedback_.phase==Phase::Rejected)return false;
        if(PathReturn()&&Holding(feedback_.phase)&&feedback_.anchorCaptured) {
            // A second HOLD stops the excursion and revokes its START. It
            // must never replace the original machining anchor with this pose.
            feedback_.holdSequence=holdSequence;feedback_.startFloor=startFloor;
            feedback_.resumeStartSequence=feedback_.rejectedStartSequence=0ULL;
            feedback_.interrupted=true;feedback_.exactReturn=false;
            SetPhase(Phase::Rebraking);feedback_.tick=tick;feedback_.nowUs=nowUs;return true;
        }
        feedback_=Feedback{};feedback_.scope=scope;feedback_.holdSequence=holdSequence;
        feedback_.startFloor=startFloor;feedback_.phase=Phase::Braking;feedback_.tick=tick;feedback_.nowUs=nowUs;
        if(flush){feedback_.flushPause=true;feedback_.flushPhase=flush->phase;
            feedback_.flushEventIndex=flush->work.triggeredEvents;}
        lastProofTick_=0ULL;Capture(committed);return true;
    }
    bool BeginResume(const Scope& scope,std::uint64_t holdSequence,std::uint64_t startSequence,
        std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!ValidScope(scope)||!SameScope(scope,feedback_.scope)||!ValidPauseContext(feedback_)||
            (FlushPause()&&!feedback_.anchorCaptured)||feedback_.phase!=Phase::Held||
            feedback_.stableCycles<RequiredStableCycles||holdSequence!=feedback_.holdSequence||
            !startSequence||startSequence<=feedback_.startFloor||startSequence<=feedback_.rejectedStartSequence||!lastProofTick_||
            tick<lastProofTick_||tick-lastProofTick_>1ULL||nowUs<feedback_.nowUs||
            nowUs-feedback_.nowUs>EDM126::MaximumSourceAgeUs)return false;
        feedback_.resumeStartSequence=startSequence;
        if(PathReturn()&&!RestIsAnchor())SetPhase(Phase::Returning);
        else {feedback_.phase=Phase::Resuming;feedback_.resumeSeedTick=0ULL;
            if(PathReturn())feedback_.exactReturn=true;}
        return true;
    }
    void RejectStart(std::uint64_t startSequence) noexcept
    {if(startSequence>feedback_.rejectedStartSequence)feedback_.rejectedStartSequence=startSequence;}
    bool ObserveMapped(const Scope& scope,const EDM126::Feedback& mapped) noexcept
    {
        if(!DemandZero())return feedback_.phase==Phase::Idle||feedback_.phase==Phase::Running;
        if(!SameScope(scope,feedback_.scope)||!ValidMapped(scope,mapped)||mapped.tick<feedback_.tick||
            mapped.nowUs<feedback_.nowUs){Reject(mapped.tick,mapped.nowUs);return false;}
        const bool zero=mapped.velocityMmSec==0.0&&mapped.velocityPps==EDM126::Point{};
        if(PathReturn())return ObservePathMapped(mapped,zero);
        if(feedback_.phase==Phase::Braking){
            Capture(mapped);
            if(zero){feedback_.anchorCaptured=FlushPause();feedback_.phase=Phase::Qualifying;
                feedback_.stableCycles=0U;lastProofTick_=0ULL;}
        }else if(!zero||!SamePose(mapped)){
            Reject(mapped.tick,mapped.nowUs);return false;
        }
        if(feedback_.phase==Phase::Resuming&&!feedback_.resumeSeedTick)feedback_.resumeSeedTick=mapped.tick;
        feedback_.tick=mapped.tick;feedback_.nowUs=mapped.nowUs;return true;
    }
    void ObserveProof(bool valid,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!ZeroOutput())return;
        valid=valid&&tick==feedback_.tick&&nowUs==feedback_.nowUs&&
            (!lastProofTick_||tick==lastProofTick_+1ULL);
        if(PathReturn()) {
            if(!valid) {
                if(feedback_.phase==Phase::Resuming){Reject(tick,nowUs);return;}
                feedback_.stableCycles=0U;lastProofTick_=0ULL;return;
            }
            lastProofTick_=tick;
            if(feedback_.stableCycles<RequiredStableCycles)++feedback_.stableCycles;
            if(feedback_.stableCycles>=RequiredStableCycles) {
                if(feedback_.phase==Phase::Qualifying) {
                    if(feedback_.retreatPlanned)SetPhase(Phase::Retreating);
                    else if(feedback_.retreatUnavailable)feedback_.phase=Phase::Held;
                }else if(feedback_.phase==Phase::TopQualifying) {
                    feedback_.topCaptured=true;feedback_.phase=Phase::Held;
                }else if(feedback_.phase==Phase::ReturnQualifying) {
                    feedback_.exactReturn=true;feedback_.phase=Phase::Resuming;feedback_.resumeSeedTick=0ULL;
                }else if(feedback_.phase==Phase::Requalifying)feedback_.phase=Phase::Held;
            }
            if(feedback_.phase==Phase::Resuming&&feedback_.resumeSeedTick&&tick>feedback_.resumeSeedTick)
                feedback_.phase=Phase::Running;
            return;
        }
        if(!valid){
            if(feedback_.phase==Phase::Resuming){Reject(tick,nowUs);return;}
            feedback_.phase=Phase::Qualifying;feedback_.stableCycles=0U;lastProofTick_=0ULL;return;
        }
        lastProofTick_=tick;
        if(feedback_.stableCycles<RequiredStableCycles)++feedback_.stableCycles;
        if(feedback_.phase==Phase::Qualifying&&feedback_.stableCycles>=RequiredStableCycles)feedback_.phase=Phase::Held;
        if(feedback_.phase==Phase::Resuming&&feedback_.resumeSeedTick&&tick>feedback_.resumeSeedTick)
            feedback_.phase=Phase::Running;
    }
    void Reject(std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        feedback_.phase=Phase::Rejected;feedback_.stableCycles=0U;feedback_.resumeSeedTick=0ULL;
        feedback_.tick=tick;feedback_.nowUs=nowUs;lastProofTick_=0ULL;
    }
private:
    void SetPhase(Phase phase) noexcept
    {feedback_.phase=phase;feedback_.stableCycles=0U;feedback_.resumeSeedTick=0ULL;lastProofTick_=0ULL;}
    void CaptureRest(const EDM126::Feedback& f) noexcept
    {restCursor_.source=f.cursorSource;restCursor_.localMm=f.localMm;restPosition_=f.positionPulse;
        restStation_=f.stationMm;restDepth_=f.historyDepth;}
    bool SameRest(const EDM126::Feedback& f) const noexcept
    {return EDM126::SameSource(f.cursorSource,restCursor_.source)&&f.localMm==restCursor_.localMm&&
        f.stationMm==restStation_&&f.historyDepth==restDepth_&&f.positionPulse==restPosition_;}
    bool RestIsAnchor() const noexcept
    {return EDM126::SameCursor(restCursor_,anchorCursor_)&&restStation_==feedback_.stationMm&&
        restDepth_==feedback_.historyDepth&&restPosition_==feedback_.positionPulse;}
    bool ExactTarget(const EDM126::Feedback& f) const noexcept
    {return EDM126::SameSource(f.cursorSource,feedback_.retreatTarget.source)&&
        f.localMm==feedback_.retreatTarget.localMm&&f.stationMm==feedback_.retreatStationMm&&
        f.positionPulse==feedback_.retreatPositionPulse;}
    bool ObservePathMapped(const EDM126::Feedback& f,bool zero) noexcept
    {
        const auto phase=feedback_.phase;
        if(phase==Phase::Braking) {
            Capture(f);
            if(zero){feedback_.anchorCaptured=true;CaptureRest(f);SetPhase(Phase::Qualifying);}
        }else if(phase==Phase::Rebraking) {
            if(zero){CaptureRest(f);SetPhase(Phase::Requalifying);}
        }else if(phase==Phase::Retreating||phase==Phase::Returning) {
            const double tolerance=EDM126::Roundoff(feedback_.stationMm);
            if(!feedback_.retreatPlanned||f.stationMm<feedback_.retreatStationMm-tolerance||
                f.stationMm>feedback_.stationMm+tolerance||
                (phase==Phase::Retreating?f.velocityMmSec>0.0:f.velocityMmSec<0.0)) {
                Reject(f.tick,f.nowUs);return false;
            }
            if(zero&&(phase==Phase::Retreating?ExactTarget(f):SamePose(f))) {
                CaptureRest(f);SetPhase(phase==Phase::Retreating?Phase::TopQualifying:Phase::ReturnQualifying);
            }
        }else if(!zero||!SameRest(f)) {
            Reject(f.tick,f.nowUs);return false;
        }
        if(feedback_.phase==Phase::Resuming&&!feedback_.resumeSeedTick)feedback_.resumeSeedTick=f.tick;
        feedback_.tick=f.tick;feedback_.nowUs=f.nowUs;return true;
    }
    void Capture(const EDM126::Feedback& f) noexcept
    {
        feedback_.cursorSource=f.cursorSource;feedback_.localMm=f.localMm;feedback_.stationMm=f.stationMm;
        feedback_.historyDepth=f.historyDepth;feedback_.positionPulse=f.positionPulse;
        anchorCursor_.source=f.cursorSource;anchorCursor_.localMm=f.localMm;
    }
    bool SamePose(const EDM126::Feedback& f) const noexcept
    {
        return EDM126::SameSource(f.cursorSource,feedback_.cursorSource)&&f.localMm==feedback_.localMm&&
            f.stationMm==feedback_.stationMm&&f.historyDepth==feedback_.historyDepth&&f.positionPulse==feedback_.positionPulse;
    }
    Feedback feedback_{};
    std::uint64_t lastProofTick_=0ULL;
    EDM126::Cursor anchorCursor_{},restCursor_{};
    EDM126::Point restPosition_{};
    double restStation_=0.0;
    unsigned restDepth_=0U;
};
}
