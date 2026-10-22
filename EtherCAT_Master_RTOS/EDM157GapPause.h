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
{ return EDM149::IsFlushHoldTestArm(arm) || EDM149::IsPauseTestArm(arm) || arm==294U || arm==296U || arm==298U; }
inline bool SupportsFixture(unsigned history, EDM116::Fixture policy) noexcept
{
    return (history == EDM149::ProgramHistoryFixture && policy == EDM116::Fixture::ModalProgram156) ||
        (history == EDM149::PauseHistoryFixture && policy == EDM116::Fixture::ModalPause157) ||
        (history == EDM149::XYZVisibleFlushHistoryFixture && policy == EDM116::Fixture::ModalXYZVisibleFlush159) ||
        (history == EDM149::SixVisibleFlushHistoryFixture && policy == EDM116::Fixture::ModalSixVisibleFlush167) ||
        (history == EDM149::G76GapHistoryFixture && policy == EDM116::Fixture::ModalG76Gap173);
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
    // E199: compact receipt for the retained immutable G76 parent. The
    // Source above binds its exact genuine child; both owners retain and
    // independently compare the full sealed parent and child identity.
    std::uint64_t compoundFingerprint=0ULL,compoundArmEpoch=0ULL;
    int compoundLine=0;
};
inline bool IsG76B2Pause(const Scope& s) noexcept
{
    return s.historyFixture==EDM149::G76GapHistoryFixture && s.policy.fixture==EDM116::Fixture::ModalG76Gap173 &&
        s.policy.mode==EDM116::Mode::B2 && EDM130::IsG76B2Flush175(s.recipe) &&
        !s.recipe.holdRetreatMode && s.compoundFingerprint && s.compoundArmEpoch && s.compoundLine>0;
}
// EDM217: center modes retain the same sealed parent while pausing only
// its nominal GAP child. In-flight center E5 and paused JOG stay excluded.
inline bool IsG76CenterNominalPause(const Scope& s) noexcept
{
    return s.historyFixture==EDM149::G76GapHistoryFixture && s.policy.fixture==EDM116::Fixture::ModalG76Gap173 &&
        ((s.policy.mode==EDM116::Mode::B3 && EDM130::IsG76B3Flush176(s.recipe)) ||
         (s.policy.mode==EDM116::Mode::B4 && EDM130::IsG76B4Flush177(s.recipe))) &&
        s.recipe.diagnosticControlSource==3U && !s.recipe.holdRetreatMode &&
        s.compoundFingerprint && s.compoundArmEpoch && s.compoundLine>0;
}
// Keep the E199 name for callers that identify a sealed compound parent.
// Only B2 additionally permits its proven in-flight E5 pause transaction.
inline bool IsG76NominalPause(const Scope& s) noexcept
{return IsG76B2Pause(s)||IsG76CenterNominalPause(s);}
inline bool SameScope(const Scope& a, const Scope& b) noexcept
{
    return EDM126::SameSource(a.authorizedSource,b.authorizedSource) && a.ownerGeneration==b.ownerGeneration &&
        a.historyFixture==b.historyFixture && EDM130::SameRecipe(a.recipe,b.recipe) && EDM116::SamePolicy(a.policy,b.policy) &&
        a.compoundFingerprint==b.compoundFingerprint && a.compoundArmEpoch==b.compoundArmEpoch &&
        a.compoundLine==b.compoundLine;
}
inline bool ValidScope(const Scope& s) noexcept
{
    const auto& a=s.authorizedSource;
    return EDM126::ValidSource(a) && s.ownerGeneration && s.ownerGeneration==a.scope.ownerGeneration &&
        SupportsFixture(s.historyFixture,s.policy.fixture) && EDM130::ValidHistoryRecipeBinding(s.historyFixture,s.recipe) &&
        (s.historyFixture==EDM149::G76GapHistoryFixture ? IsG76NominalPause(s) :
            (!s.compoundFingerprint && !s.compoundArmEpoch && s.compoundLine==0)) &&
        (!(EDM130::IsVisibleFlush159(s.recipe)||EDM130::IsVisibleFlush167(s.recipe))||s.recipe.holdRetreatMode==1U) &&
        a.ordinal<EDM130::ProgramRows(s.recipe) &&
        EDM116::PolicyMatchesBinding(s.policy,static_cast<unsigned>(a.ordinal),a.sourcePC,a.dispatch);
}
inline bool G76FlushSourceBound(const EDM126::Source& source,const EDM126::Source& authorized) noexcept
{
    return EDM126::ValidSource(source)&&EDM109::SameScope(source.scope,authorized.scope)&&
        source.dispatch==authorized.dispatch&&source.epoch==authorized.epoch&&source.sourcePC==authorized.sourcePC&&
        source.ordinal<=authorized.ordinal;
}
inline bool G76FlushAnchorBound(const Scope& scope,const EDM128::Feedback& e) noexcept
{
    const auto& a=e.anchor;
    if(!e.anchorCaptured)return !e.targetCaptured&&
        EDM126::SameSource(a.authorizedSource,EDM126::Source{})&&EDM126::SameSource(a.cursorSource,EDM126::Source{})&&
        !a.tick&&!a.nowUs&&EDM109::SameDouble(a.localMm,0.0)&&EDM109::SameDouble(a.stationMm,0.0)&&
        EDM109::SamePulse(a.positionPulse,EDM126::Point{});
    return e.targetCaptured&&EDM126::SameSource(a.authorizedSource,scope.authorizedSource)&&
        G76FlushSourceBound(a.cursorSource,scope.authorizedSource)&&G76FlushSourceBound(e.targetSource,scope.authorizedSource)&&
        e.targetSource.ordinal<=a.cursorSource.ordinal;
}
inline bool PauseModeAllowed(const Scope& scope,const EDM128::Feedback& e) noexcept
{
    if(!ValidScope(scope)||e.faulted||e.stopped||!EDM116::SamePolicy(e.policy,scope.policy))return false;
    if(e.phase==EDM128::Phase::Gap)return (!EDM130::IsOneShotCommissioning(scope.recipe)||IsG76NominalPause(scope))&&
        !e.detached&&e.work.triggeredEvents==e.work.completedEvents;
    if(IsG76B2Pause(scope))return EDM128::CanPauseTimedFlush(e)&&G76FlushAnchorBound(scope,e)&&
        e.work.workTimeUs==scope.recipe.workTimeUs&&e.eventIndex==1ULL&&
        e.work.triggeredEvents==1ULL&&!e.work.completedEvents&&
        EDM128::ValidHeightProof(e,EDM130::FrozenJumpHeightMm(scope.recipe),
            scope.recipe.deepFlushCycleInterval,EDM130::FrozenDeepMultiplier(scope.recipe),
            EDM130::HasBoundDeep(scope.recipe));
    return scope.recipe.holdRetreatMode==1U&&
        (scope.historyFixture==EDM149::ProgramHistoryFixture||
            scope.historyFixture==EDM149::XYZVisibleFlushHistoryFixture||
            scope.historyFixture==EDM149::SixVisibleFlushHistoryFixture)&&EDM128::CanPauseTimedFlush(e)&&
        e.work.workTimeUs==scope.recipe.workTimeUs&&e.eventIndex==e.work.triggeredEvents&&
        (!(EDM130::IsVisibleFlush159(scope.recipe)||EDM130::IsVisibleFlush167(scope.recipe))||e.eventIndex==1ULL);
}
// Public, value-only E5 receipt. Full sources, pulses, work evidence and
// captured geometry are independently retained and compared by Transaction.
// These fields describe the canonical event, never the pause stopping pose.
struct FlushReceipt
{
    std::uint64_t elapsedUs=0ULL,totalWorkUs=0ULL,lastTriggerTick=0ULL,lastTriggerUs=0ULL;
    std::uint64_t anchorSegment=0ULL,anchorOrdinal=0ULL,targetSegment=0ULL,targetOrdinal=0ULL;
    double anchorLocalMm=0.0,anchorStationMm=0.0,targetLocalMm=0.0,targetStationMm=0.0;
    unsigned qualifiedDwells=0U;
    bool anchorCaptured=false,targetCaptured=false,topCaptured=false,exactReturn=false;
};
inline FlushReceipt CaptureFlushReceipt(const EDM128::Feedback& e) noexcept
{
    FlushReceipt r{};r.elapsedUs=e.work.elapsedUs;r.totalWorkUs=e.work.totalWorkUs;
    r.lastTriggerTick=e.work.lastTriggerTick;r.lastTriggerUs=e.work.lastTriggerUs;
    r.anchorCaptured=e.anchorCaptured;r.targetCaptured=e.targetCaptured;
    r.topCaptured=e.topCaptured;r.exactReturn=e.exactReturn;r.qualifiedDwells=e.qualifiedDwells;
    if(e.anchorCaptured){r.anchorSegment=e.anchor.cursorSource.segment;r.anchorOrdinal=e.anchor.cursorSource.ordinal;
        r.anchorLocalMm=e.anchor.localMm;r.anchorStationMm=e.anchor.stationMm;}
    if(e.targetCaptured){r.targetSegment=e.targetSource.segment;r.targetOrdinal=e.targetSource.ordinal;
        r.targetLocalMm=e.targetLocalMm;r.targetStationMm=e.targetStationMm;}
    return r;
}
inline bool SameFlushReceipt(const FlushReceipt& a,const FlushReceipt& b) noexcept
{
    return a.elapsedUs==b.elapsedUs&&a.totalWorkUs==b.totalWorkUs&&
        a.lastTriggerTick==b.lastTriggerTick&&a.lastTriggerUs==b.lastTriggerUs&&
        a.anchorSegment==b.anchorSegment&&a.anchorOrdinal==b.anchorOrdinal&&
        a.targetSegment==b.targetSegment&&a.targetOrdinal==b.targetOrdinal&&
        EDM109::SameDouble(a.anchorLocalMm,b.anchorLocalMm)&&EDM109::SameDouble(a.anchorStationMm,b.anchorStationMm)&&
        EDM109::SameDouble(a.targetLocalMm,b.targetLocalMm)&&EDM109::SameDouble(a.targetStationMm,b.targetStationMm)&&
        a.qualifiedDwells==b.qualifiedDwells&&a.anchorCaptured==b.anchorCaptured&&
        a.targetCaptured==b.targetCaptured&&a.topCaptured==b.topCaptured&&a.exactReturn==b.exactReturn;
}
// EDM217: frozen committed GAP work totals survive braking and the Running
// acknowledgement; live coordinator observation clocks are not this receipt.
struct NominalWorkReceipt
{
    std::uint64_t elapsedUs=0ULL,totalWorkUs=0ULL,triggeredEvents=0ULL,completedEvents=0ULL;
};
inline NominalWorkReceipt CaptureNominalWorkReceipt(const EDM130::WorkEvidence& w) noexcept
{return {w.elapsedUs,w.totalWorkUs,w.triggeredEvents,w.completedEvents};}
inline bool SameNominalWorkReceipt(const NominalWorkReceipt& a,const NominalWorkReceipt& b) noexcept
{return a.elapsedUs==b.elapsedUs&&a.totalWorkUs==b.totalWorkUs&&
    a.triggeredEvents==b.triggeredEvents&&a.completedEvents==b.completedEvents;}
struct Feedback
{
    Scope scope{};
    std::uint64_t holdSequence=0ULL, startFloor=0ULL, resumeStartSequence=0ULL, rejectedStartSequence=0ULL;
    Phase phase=Phase::Idle;
    bool nominalWorkCaptured=false;
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
    FlushReceipt flush{};
    NominalWorkReceipt nominalWork{};
};
static_assert(std::is_trivially_copyable<Scope>::value && std::is_trivially_copyable<Feedback>::value,
    "EDM157 cross-thread receipts are fixed values.");
static_assert(sizeof(Feedback)<=1024U,"EDM157 bounded pause publication.");
inline bool ValidNominalWorkReceipt(const Feedback& f) noexcept
{
    const auto& w=f.nominalWork;const auto duration=f.scope.recipe.workTimeUs;
    return f.nominalWorkCaptured&&!f.flushPause&&IsG76NominalPause(f.scope)&&ValidScope(f.scope)&&
        EDM130::ValidWorkTime(duration)&&w.elapsedUs<=duration&&w.elapsedUs<=w.totalWorkUs&&
        w.triggeredEvents==w.completedEvents&&w.triggeredEvents<=w.totalWorkUs/duration;
}
inline bool ValidG76FlushReceipt(const Feedback& f) noexcept
{
    const auto& r=f.flush;
    if(!f.flushPause||!IsG76B2Pause(f.scope)||!ValidScope(f.scope)||f.flushEventIndex!=1ULL||
        r.elapsedUs!=f.scope.recipe.workTimeUs||r.totalWorkUs<r.elapsedUs||!r.lastTriggerTick||!r.lastTriggerUs||
        r.lastTriggerTick>f.tick||r.lastTriggerUs>f.nowUs)return false;
    unsigned dwells=0U;bool anchor=false,top=false,returned=false;
    switch(f.flushPhase){
    case EDM128::Phase::Brake:case EDM128::Phase::AnchorDwell:break;
    case EDM128::Phase::Outbound:case EDM128::Phase::TopDwell:dwells=1U;anchor=true;break;
    case EDM128::Phase::Returning:case EDM128::Phase::ReturnDwell:dwells=2U;anchor=top=true;break;
    case EDM128::Phase::Resume:dwells=3U;anchor=top=returned=true;break;
    default:return false;
    }
    if(r.qualifiedDwells!=dwells||r.anchorCaptured!=anchor||r.targetCaptured!=anchor||
        r.topCaptured!=top||r.exactReturn!=returned)return false;
    if(!anchor)return !r.anchorSegment&&!r.anchorOrdinal&&!r.targetSegment&&!r.targetOrdinal&&
        EDM109::SameDouble(r.anchorLocalMm,0.0)&&EDM109::SameDouble(r.anchorStationMm,0.0)&&
        EDM109::SameDouble(r.targetLocalMm,0.0)&&EDM109::SameDouble(r.targetStationMm,0.0);
    return r.anchorSegment&&r.targetSegment&&r.anchorOrdinal<=f.scope.authorizedSource.ordinal&&
        r.targetOrdinal<=r.anchorOrdinal&&std::isfinite(r.anchorLocalMm)&&r.anchorLocalMm>=0.0&&
        std::isfinite(r.anchorStationMm)&&r.anchorStationMm>=r.anchorLocalMm&&
        std::isfinite(r.targetLocalMm)&&r.targetLocalMm>=0.0&&
        std::isfinite(r.targetStationMm)&&r.targetStationMm>=r.targetLocalMm&&
        r.targetStationMm<r.anchorStationMm&&
        EDM128::HeightClose(r.anchorStationMm-r.targetStationMm,EDM130::FrozenJumpHeightMm(f.scope.recipe));
}
inline bool ValidPauseContext(const Feedback& f) noexcept
{
    if(!f.flushPause)return (!EDM130::IsOneShotCommissioning(f.scope.recipe)||
        (IsG76NominalPause(f.scope)&&ValidScope(f.scope)))&&
        (IsG76NominalPause(f.scope)?ValidNominalWorkReceipt(f):
            (!f.nominalWorkCaptured&&SameNominalWorkReceipt(f.nominalWork,NominalWorkReceipt{})))&&
        f.flushPhase==EDM128::Phase::Gap&&!f.flushEventIndex&&SameFlushReceipt(f.flush,FlushReceipt{});
    if(f.nominalWorkCaptured||!SameNominalWorkReceipt(f.nominalWork,NominalWorkReceipt{}))return false;
    if(IsG76B2Pause(f.scope))return ValidG76FlushReceipt(f)&&
        !f.retreatPlanned&&!f.retreatUnavailable&&!f.topCaptured&&!f.exactReturn;
    return ValidScope(f.scope)&&f.scope.recipe.holdRetreatMode==1U&&
        (f.scope.historyFixture==EDM149::ProgramHistoryFixture||
            f.scope.historyFixture==EDM149::XYZVisibleFlushHistoryFixture||
            f.scope.historyFixture==EDM149::SixVisibleFlushHistoryFixture)&&EDM128::IsFlushPhase(f.flushPhase)&&
        f.flushEventIndex&&(!(EDM130::IsVisibleFlush159(f.scope.recipe)||EDM130::IsVisibleFlush167(f.scope.recipe))||f.flushEventIndex==1ULL)&&
        !f.retreatPlanned&&!f.retreatUnavailable&&!f.topCaptured&&!f.exactReturn&&SameFlushReceipt(f.flush,FlushReceipt{});
}
inline bool SamePauseContext(const Feedback& a,const Feedback& b) noexcept
{return a.flushPause==b.flushPause&&a.flushPhase==b.flushPhase&&a.flushEventIndex==b.flushEventIndex&&
    SameFlushReceipt(a.flush,b.flush)&&a.nominalWorkCaptured==b.nominalWorkCaptured&&
    SameNominalWorkReceipt(a.nominalWork,b.nominalWork);}
// Compare fixed values rather than object bytes: padding has no identity.
// Dwell sample clocks and the current speed sample intentionally reset while
// paused. Every canonical capture, completed dwell and E5 work clock stays.
inline bool SameFrozenG76Flush(const EDM128::Feedback& a,const EDM128::Feedback& b) noexcept
{
    const auto& x=a.anchor;const auto& y=b.anchor;
    for(unsigned i=0U;i<sizeof(a.modeEvents)/sizeof(a.modeEvents[0]);++i)
        if(a.modeEvents[i]!=b.modeEvents[i])return false;
    return EDM116::SamePolicy(a.policy,b.policy)&&a.phase==b.phase&&
        (a.phase!=EDM128::Phase::Resume||(a.stableCycles==b.stableCycles&&a.stableStartUs==b.stableStartUs))&&
        EDM130::SameWorkEvidence(a.work,b.work)&&a.eventIndex==b.eventIndex&&
        a.eventCount==b.eventCount&&a.qualifiedDwells==b.qualifiedDwells&&
        a.rowEventComplete==b.rowEventComplete&&a.detached==b.detached&&a.stopped==b.stopped&&a.faulted==b.faulted&&
        a.liveAnchor==b.liveAnchor&&a.anchorCaptured==b.anchorCaptured&&a.targetCaptured==b.targetCaptured&&
        a.topCaptured==b.topCaptured&&a.exactReturn==b.exactReturn&&
        EDM126::SameSource(x.authorizedSource,y.authorizedSource)&&EDM126::SameSource(x.cursorSource,y.cursorSource)&&
        x.tick==y.tick&&x.nowUs==y.nowUs&&EDM109::SameDouble(x.localMm,y.localMm)&&
        EDM109::SameDouble(x.lengthMm,y.lengthMm)&&EDM109::SameDouble(x.stationMm,y.stationMm)&&
        EDM109::SameDouble(x.velocityMmSec,y.velocityMmSec)&&EDM109::SameDouble(x.accelerationMmSec2,y.accelerationMmSec2)&&
        EDM109::SamePulse(x.positionPulse,y.positionPulse)&&EDM109::SamePulse(x.velocityPps,y.velocityPps)&&
        EDM109::SamePulse(x.accelerationPps2,y.accelerationPps2)&&
        x.historyCount==y.historyCount&&x.historyDepth==y.historyDepth&&x.maximumHistoryDepth==y.maximumHistoryDepth&&
        x.completedRows==y.completedRows&&x.backwardCrossings==y.backwardCrossings&&x.forwardCrossings==y.forwardCrossings&&
        x.seamStableCycles==y.seamStableCycles&&x.active==y.active&&x.awaitingCurrent==y.awaitingCurrent&&
        x.awaitingSeam==y.awaitingSeam&&x.atCurrentEnd==y.atCurrentEnd&&x.atRetreatLimit==y.atRetreatLimit&&
        x.completeReady==y.completeReady&&x.stopRequested==y.stopRequested&&x.stopped==y.stopped&&x.faulted==y.faulted&&x.reason==y.reason&&
        EDM126::SameSource(a.targetSource,b.targetSource)&&EDM109::SameDouble(a.targetLocalMm,b.targetLocalMm)&&
        EDM109::SameDouble(a.targetStationMm,b.targetStationMm)&&EDM109::SameDouble(a.actualJumpHeightMm,b.actualJumpHeightMm)&&
        EDM109::SamePulse(a.topPositionPulse,b.topPositionPulse)&&EDM109::SamePulse(a.tangentPulsePerMm,b.tangentPulsePerMm)&&
        EDM109::SamePulse(a.heightPulsePerNative,b.heightPulsePerNative)&&
        EDM109::SameDouble(a.absoluteJumpHeightMm,b.absoluteJumpHeightMm)&&a.deepBound==b.deepBound&&
        a.deepFlushCycleInterval==b.deepFlushCycleInterval&&EDM109::SameDouble(a.deepFlushHeightMultiplier,b.deepFlushHeightMultiplier)&&
        a.completedDeepEvents==b.completedDeepEvents&&a.deepCycle==b.deepCycle&&
        EDM109::SameDouble(a.effectiveJumpHeightMm,b.effectiveJumpHeightMm)&&
        EDM109::SameDouble(a.initialStageDistanceMm,b.initialStageDistanceMm)&&
        EDM109::SameDouble(a.finalStageDistanceMm,b.finalStageDistanceMm)&&EDM128::SameB3Proof(a,b);
}
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
        if(!FlushPause()) {
            if(!IsG76NominalPause(feedback_.scope))return true;
            auto frozen=e.work;frozen.previousEligible=frozenNominalWork_.previousEligible;
            return ValidNominalWorkReceipt(feedback_)&&PauseModeAllowed(feedback_.scope,e)&&
                EDM130::ValidWorkEvidence(e.work)&&e.work.workTimeUs==feedback_.scope.recipe.workTimeUs&&
                SameNominalWorkReceipt(feedback_.nominalWork,CaptureNominalWorkReceipt(e.work))&&
                EDM130::SameWorkEvidence(frozen,frozenNominalWork_);
        }
        return PauseModeAllowed(feedback_.scope,e)&&e.phase==feedback_.flushPhase&&
            e.work.triggeredEvents==feedback_.flushEventIndex&&(!IsG76B2Pause(feedback_.scope)||
                (ValidG76FlushReceipt(feedback_)&&SameFlushReceipt(feedback_.flush,CaptureFlushReceipt(e))&&
                    SameFrozenG76Flush(e,frozenFlush_)));
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
        const EDM128::Feedback* flush=nullptr,const EDM128::Feedback* nominal=nullptr) noexcept
    {
        if((EDM130::IsOneShotCommissioning(scope.recipe)&&!flush&&!IsG76NominalPause(scope))||
            (Holding(feedback_.phase)&&!SameScope(scope,feedback_.scope))||
            (flush&&(!PauseModeAllowed(scope,*flush)||!EDM128::CanPauseTimedFlush(*flush)))||
            (!flush&&IsG76NominalPause(scope)&&(!nominal||!PauseModeAllowed(scope,*nominal)||
                nominal->phase!=EDM128::Phase::Gap||!EDM130::ValidWorkEvidence(nominal->work)||
                nominal->work.workTimeUs!=scope.recipe.workTimeUs))||
            (nominal&&(flush||!IsG76NominalPause(scope)))||
            (DemandZero()&&!FlushPause()&&IsG76NominalPause(scope)&&
                (!nominal||!FlushContextCurrent(*nominal)))||
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
            feedback_.flushEventIndex=flush->work.triggeredEvents;
            if(IsG76B2Pause(scope)){frozenFlush_=*flush;feedback_.flush=CaptureFlushReceipt(*flush);}}
        frozenNominalWork_=EDM130::WorkEvidence{};
        if(nominal){feedback_.nominalWorkCaptured=true;feedback_.nominalWork=CaptureNominalWorkReceipt(nominal->work);
            frozenNominalWork_=nominal->work;}
        lastProofTick_=0ULL;Capture(committed);
        return (!flush||!IsG76B2Pause(scope)||ValidG76FlushReceipt(feedback_))&&
            (!nominal||ValidNominalWorkReceipt(feedback_));
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
    // RT-owned canonical snapshot. It is never an output authority and is
    // never sent through the bounded cross-thread publication channel.
    EDM128::Feedback frozenFlush_{};
    EDM130::WorkEvidence frozenNominalWork_{};
    std::uint64_t lastProofTick_=0ULL;
    EDM126::Cursor anchorCursor_{},restCursor_{};
    EDM126::Point restPosition_{};
    double restStation_=0.0;
    unsigned restDepth_=0U;
};
}
