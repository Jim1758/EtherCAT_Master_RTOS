#pragma once

#include "NCG76SphereCoordinates.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM285 phase 1: bounded values and a receipt-driven model only. This file
// never requests braking, moves an axis, publishes Motion, enables power or
// changes ordinary HOLD. A future runtime adapter must first produce a fresh
// stopped receipt from the normal controlled stop and physical settle proofs.
namespace NCG76SphereHold
{
using Point = NCG76SphereCoordinates::Point8;
using NativePlan = NCG76SphereCoordinates::NativePlan;
using Cursor = NCG76Sphere::Cursor;
constexpr bool RuntimeMotionAdmission = false;
constexpr std::uint64_t MaximumReceiptAgeUs = 1500ULL;
constexpr std::uint64_t MinimumStoppedUs = 50000ULL;
constexpr unsigned RequiredStoppedSamples = 200U;

struct Scope
{
    std::uint64_t session=0ULL,run=0ULL,cache=0ULL,parent=0ULL,ownerGeneration=0ULL;
    std::uint64_t frameToken=0ULL,planFingerprint=0ULL;
    int sourcePC=-1;
};
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.session==b.session&&a.run==b.run&&a.cache==b.cache&&a.parent==b.parent&&
        a.ownerGeneration==b.ownerGeneration&&a.frameToken==b.frameToken&&
        a.planFingerprint==b.planFingerprint&&a.sourcePC==b.sourcePC;
}
inline bool ScopeFits(const Scope& s,const NativePlan& plan) noexcept
{
    return s.session&&s.run&&s.cache&&s.parent&&s.ownerGeneration&&s.frameToken&&
        s.planFingerprint&&s.sourcePC>=0&&s.run==plan.frame.translation.runToken&&
        s.frameToken==plan.frame.translation.generation&&s.planFingerprint==plan.descriptor.fingerprint;
}
inline bool SamePoint(const Point& a,const Point& b) noexcept
{return std::memcmp(a.data(),b.data(),sizeof(a))==0;}
inline bool SameCursor(const Cursor& a,const Cursor& b) noexcept
{return a.initial==b.initial&&a.layerIndex==b.layerIndex&&a.segmentIndex==b.segmentIndex;}
inline bool SameFraction(double a,double b) noexcept
{return std::memcmp(&a,&b,sizeof(double))==0;}

// The caller supplies the currently retained parent and frame. Tokens are
// freshness fences, not substitutes for the full NativePlan comparison.
struct Context
{
    Scope scope{};
    std::uint64_t tick=0ULL,nowUs=0ULL,holdSequence=0ULL,startSequence=0ULL,checkpointSequence=0ULL;
    Cursor checkpointCursor{};
    double checkpointFraction=0.0;
    bool checkpointCurrent=false;
    bool frameCurrent=false,powerInhibited=false,dischargeOff=false,powerAuthorized=false;
};
struct StoppedProof
{
    Scope scope{};
    std::uint64_t holdSequence=0ULL,checkpointSequence=0ULL,proofSequence=0ULL,transitionSequence=0ULL;
    std::uint64_t tick=0ULL,nowUs=0ULL,settledSinceUs=0ULL;
    unsigned settledSamples=0U;
    Point position{},velocity{},acceleration{}; // Commanded native coordinates; C/U/V are retained.
    bool stopped=false,physicalSettled=false,sourceCurrent=false;
    bool powerInhibited=false,dischargeOff=false,powerAuthorized=false;
};
struct StopReceipt
{
    StoppedProof proof{};
    Cursor cursor{};
    double fraction=0.0;
};
enum class Code : std::uint8_t
{
    None,InvalidPlan,ScopeMismatch,FrameMismatch,Power,Clock,Checkpoint,NotStopped,
    PositionMismatch,AlreadyActive,State,Sequence,Reset,Fault
};
enum class LegKind : std::uint8_t { None,ToCenter,ToStart,BackToCenter,BackToAnchor };
struct Leg
{
    LegKind kind=LegKind::None;
    Point start{},end{};
    bool stationary=true;
};
struct Plan
{
    NativePlan nominal{};
    StopReceipt stopped{};
    Point anchor{},center{},start{};
    std::array<Leg,4U> legs{};
    bool valid=false;
};

inline Code CheckContext(const NativePlan& nominal,const Context& context) noexcept
{
    if(!NCG76SphereCoordinates::Valid(nominal))return Code::InvalidPlan;
    if(!ScopeFits(context.scope,nominal))return Code::ScopeMismatch;
    if(!context.frameCurrent)return Code::FrameMismatch;
    if(!context.powerInhibited||!context.dischargeOff||context.powerAuthorized)return Code::Power;
    if(!context.tick||!context.nowUs)return Code::Clock;
    if(!context.holdSequence||!context.checkpointSequence||!context.checkpointCurrent||
        !std::isfinite(context.checkpointFraction)||context.checkpointFraction<0.0||context.checkpointFraction>1.0)
        return Code::Checkpoint;
    return Code::None;
}
inline Code CheckStopped(const StoppedProof& proof,const Context& context) noexcept
{
    if(!SameScope(proof.scope,context.scope))return Code::ScopeMismatch;
    if(!proof.powerInhibited||!proof.dischargeOff||proof.powerAuthorized)return Code::Power;
    if(!proof.holdSequence||proof.holdSequence!=context.holdSequence||
        !proof.checkpointSequence||proof.checkpointSequence!=context.checkpointSequence||!proof.proofSequence)
        return Code::Checkpoint;
    if(!proof.tick||!proof.nowUs||proof.tick>context.tick||context.tick-proof.tick>1ULL||
        proof.nowUs>context.nowUs||context.nowUs-proof.nowUs>MaximumReceiptAgeUs)return Code::Clock;
    if(!proof.stopped||!proof.physicalSettled||!proof.sourceCurrent||
        proof.settledSamples<RequiredStoppedSamples||!proof.settledSinceUs||
        proof.settledSinceUs>proof.nowUs||proof.nowUs-proof.settledSinceUs<MinimumStoppedUs)return Code::NotStopped;
    for(unsigned axis=0U;axis<8U;++axis)
        if(!std::isfinite(proof.position[axis])||!std::isfinite(proof.velocity[axis])||
            !std::isfinite(proof.acceleration[axis])||proof.velocity[axis]!=0.0||proof.acceleration[axis]!=0.0)
            return Code::NotStopped;
    return Code::None;
}
inline Leg MakeLeg(LegKind kind,const Point& start,const Point& end) noexcept
{
    Leg leg{};leg.kind=kind;leg.start=start;leg.end=end;leg.stationary=SamePoint(start,end);return leg;
}
inline Code Build(const NativePlan& nominal,const StopReceipt& stopped,const Context& context,Plan& output) noexcept
{
    const NativePlan frozen=nominal;const StopReceipt checkpoint=stopped;const Context current=context;
    output=Plan{};
    Code code=CheckContext(frozen,current);if(code!=Code::None)return code;
    code=CheckStopped(checkpoint.proof,current);if(code!=Code::None)return code;
    if(checkpoint.proof.transitionSequence!=0ULL)return Code::Sequence;
    if(!SameCursor(checkpoint.cursor,current.checkpointCursor)||
        !SameFraction(checkpoint.fraction,current.checkpointFraction))return Code::Checkpoint;
    Point anchor{},center{};
    if(!std::isfinite(checkpoint.fraction)||checkpoint.fraction<0.0||checkpoint.fraction>1.0||
        NCG76SphereCoordinates::EvaluateNative(frozen,checkpoint.cursor,checkpoint.fraction,anchor,center)!=
            NCG76SphereCoordinates::Status::Ok)return Code::Checkpoint;
    if(!SamePoint(anchor,checkpoint.proof.position))return Code::PositionMismatch;
    // EvaluateNative supplies the frozen, compensated center. On initial
    // depth, initial R retreat and axial layer advance it follows the stopped
    // position; no unvisited layer endpoint is selected and no world-Z leg is
    // invented. Circle phases use their own exact retained layer center.
    Plan result{};result.nominal=frozen;result.stopped=checkpoint;
    result.anchor=anchor;result.center=center;result.start=frozen.frame.startMCS;
    for(unsigned axis=3U;axis<8U;++axis)
        if(std::memcmp(&anchor[axis],&result.start[axis],sizeof(double))!=0||
            std::memcmp(&center[axis],&result.start[axis],sizeof(double))!=0)
            return Code::PositionMismatch;
    result.legs[0]=MakeLeg(LegKind::ToCenter,anchor,center);
    result.legs[1]=MakeLeg(LegKind::ToStart,center,result.start);
    result.legs[2]=MakeLeg(LegKind::BackToCenter,result.start,center);
    result.legs[3]=MakeLeg(LegKind::BackToAnchor,center,anchor);
    result.valid=true;output=result;return Code::None;
}

enum class Phase : std::uint8_t
{ Idle,ToCenter,ToStart,HeldAtStart,BackToCenter,BackToAnchor,ResumeReady,Completed,Revoked };
struct Feedback
{
    Phase phase=Phase::Idle;
    Code code=Code::None;
    std::uint64_t holdSequence=0ULL,checkpointSequence=0ULL,startFloor=0ULL,startSequence=0ULL;
    std::uint64_t transitionSequence=0ULL,lastProofSequence=0ULL,lastTick=0ULL,lastNowUs=0ULL;
};
struct ResumeCheckpoint
{
    Scope scope{};
    Cursor cursor{};
    double fraction=0.0;
    Point anchor{};
    std::uint64_t holdSequence=0ULL,checkpointSequence=0ULL,startSequence=0ULL;
    bool valid=false;
};

class Transaction
{
public:
    const Feedback& Current() const noexcept {return feedback_;}
    const Plan& FrozenPlan() const noexcept {return plan_;}
    // Phase 1 can describe proposed legs but cannot grant their execution.
    // In particular, RESET/FAULT never expose a replacement return route.
    bool DemandZero() const noexcept {return true;}
    bool CurrentLeg(Leg& output) const noexcept
    {
        output=Leg{};
        const unsigned index=ActiveLeg();
        if(index>=4U||!plan_.valid||plan_.legs[index].stationary)return false;
        output=plan_.legs[index];return true;
    }
    bool BeginHold(const NativePlan& nominal,const StopReceipt& stopped,const Context& context) noexcept
    {
        if(feedback_.phase!=Phase::Idle&&feedback_.phase!=Phase::Completed)return Reject(Code::AlreadyActive);
        if(context.holdSequence<=lastHoldSequence_||context.checkpointSequence<=lastCheckpointSequence_)
            return Reject(Code::Sequence);
        if(feedback_.phase==Phase::Completed&&
            (stopped.proof.proofSequence<=feedback_.lastProofSequence||stopped.proof.tick<=feedback_.lastTick||
                stopped.proof.nowUs<=feedback_.lastNowUs))return Reject(Code::Sequence);
        Plan candidate{};const Code code=Build(nominal,stopped,context,candidate);
        if(code!=Code::None)return Reject(code);
        plan_=candidate;feedback_=Feedback{};feedback_.holdSequence=context.holdSequence;
        feedback_.checkpointSequence=context.checkpointSequence;feedback_.startFloor=context.startSequence;
        Capture(stopped.proof);
        lastHoldSequence_=context.holdSequence;lastCheckpointSequence_=context.checkpointSequence;
        return SetPhase(!plan_.legs[0].stationary?Phase::ToCenter:
            !plan_.legs[1].stationary?Phase::ToStart:Phase::HeldAtStart);
    }
    bool AcceptEndpoint(const NativePlan& currentPlan,const Context& context,const StoppedProof& proof) noexcept
    {
        const unsigned index=ActiveLeg();
        if(index>=4U)return Reject(Code::State);
        if(!ValidateCurrent(currentPlan,context,proof))return false;
        if(!SamePoint(proof.position,plan_.legs[index].end))return Reject(Code::PositionMismatch);
        Capture(proof);
        // START queued before the retreat has finished cannot become the
        // later permission to return from the machining start.
        if(index<2U&&context.startSequence>feedback_.startFloor)feedback_.startFloor=context.startSequence;
        switch(index)
        {
        case 0U:return SetPhase(plan_.legs[1].stationary?Phase::HeldAtStart:Phase::ToStart);
        case 1U:return SetPhase(Phase::HeldAtStart);
        case 2U:return SetPhase(plan_.legs[3].stationary?Phase::ResumeReady:Phase::BackToAnchor);
        case 3U:return SetPhase(Phase::ResumeReady);
        default:return Reject(Code::State);
        }
    }
    bool BeginResume(const NativePlan& currentPlan,const Context& context,const StoppedProof& proof) noexcept
    {
        if(feedback_.phase!=Phase::HeldAtStart)return Reject(Code::State);
        if(!context.startSequence||context.startSequence<=feedback_.startFloor||
            context.startSequence<=feedback_.startSequence)return Reject(Code::Sequence);
        if(!ValidateCurrent(currentPlan,context,proof))return false;
        if(!SamePoint(proof.position,plan_.start))return Reject(Code::PositionMismatch);
        feedback_.startSequence=context.startSequence;Capture(proof);
        return SetPhase(!plan_.legs[2].stationary?Phase::BackToCenter:
            !plan_.legs[3].stationary?Phase::BackToAnchor:Phase::ResumeReady);
    }
    bool CompleteResume(const NativePlan& currentPlan,const Context& context,const StoppedProof& proof,
        ResumeCheckpoint& output) noexcept
    {
        output=ResumeCheckpoint{};
        if(feedback_.phase!=Phase::ResumeReady)return Reject(Code::State);
        if(!feedback_.startSequence||context.startSequence!=feedback_.startSequence)return Reject(Code::Sequence);
        if(!ValidateCurrent(currentPlan,context,proof))return false;
        if(!SamePoint(proof.position,plan_.anchor))return Reject(Code::PositionMismatch);
        Capture(proof);
        ResumeCheckpoint result{};result.scope=plan_.stopped.proof.scope;result.cursor=plan_.stopped.cursor;
        result.fraction=plan_.stopped.fraction;result.anchor=plan_.anchor;
        result.holdSequence=feedback_.holdSequence;result.checkpointSequence=feedback_.checkpointSequence;
        result.startSequence=feedback_.startSequence;result.valid=true;
        if(!SetPhase(Phase::Completed))return false;
        output=result;return true;
    }
    void Reset() noexcept {Revoke(Code::Reset);}
    void Fault() noexcept {Revoke(Code::Fault);}
private:
    unsigned ActiveLeg() const noexcept
    {
        switch(feedback_.phase)
        {
        case Phase::ToCenter:return 0U;case Phase::ToStart:return 1U;
        case Phase::BackToCenter:return 2U;case Phase::BackToAnchor:return 3U;
        default:return 4U;
        }
    }
    bool ValidateCurrent(const NativePlan& currentPlan,const Context& context,const StoppedProof& proof) noexcept
    {
        const Code code=CheckContext(currentPlan,context);if(code!=Code::None)return Reject(code);
        if(!plan_.valid||!NCG76SphereCoordinates::SameNativePlan(plan_.nominal,currentPlan))return Reject(Code::FrameMismatch);
        if(!SameScope(context.scope,plan_.stopped.proof.scope))return Reject(Code::ScopeMismatch);
        if(context.holdSequence!=feedback_.holdSequence||context.checkpointSequence!=feedback_.checkpointSequence)
            return Reject(Code::Checkpoint);
        if(!SameCursor(context.checkpointCursor,plan_.stopped.cursor)||
            !SameFraction(context.checkpointFraction,plan_.stopped.fraction))return Reject(Code::Checkpoint);
        if(feedback_.startSequence&&context.startSequence!=feedback_.startSequence)return Reject(Code::Sequence);
        const Code stopped=CheckStopped(proof,context);if(stopped!=Code::None)return Reject(stopped);
        if(proof.settledSinceUs<feedback_.lastNowUs)return Reject(Code::NotStopped);
        if(proof.transitionSequence!=feedback_.transitionSequence||
            proof.proofSequence<=feedback_.lastProofSequence||proof.tick<=feedback_.lastTick||
            proof.nowUs<=feedback_.lastNowUs)return Reject(Code::Sequence);
        return true;
    }
    void Capture(const StoppedProof& proof) noexcept
    {
        feedback_.lastProofSequence=proof.proofSequence;
        feedback_.lastTick=proof.tick;feedback_.lastNowUs=proof.nowUs;
    }
    bool SetPhase(Phase phase) noexcept
    {
        if(feedback_.transitionSequence==(std::numeric_limits<std::uint64_t>::max)())return Reject(Code::Sequence);
        feedback_.phase=phase;++feedback_.transitionSequence;return true;
    }
    bool Reject(Code code) noexcept {Revoke(code);return false;}
    void Revoke(Code code) noexcept
    {
        feedback_.phase=Phase::Revoked;feedback_.code=code;
        // Keep the original checkpoint for read-only diagnostics; no future
        // method can expose a leg or START this revoked transaction.
    }
    Plan plan_{};
    Feedback feedback_{};
    std::uint64_t lastHoldSequence_=0ULL,lastCheckpointSequence_=0ULL;
};
static_assert(std::is_trivially_copyable<Plan>::value&&std::is_trivially_copyable<StopReceipt>::value,
    "EDM285 HOLD architecture uses bounded immutable values.");
static_assert(sizeof(Plan)<=8192U&&sizeof(Transaction)<=8448U,
    "EDM285 HOLD architecture must remain bounded; no per-layer allocation.");
}
