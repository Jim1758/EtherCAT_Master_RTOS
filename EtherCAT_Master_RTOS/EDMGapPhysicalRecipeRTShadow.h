#pragma once

#include "EDMGapPhysicalRTShadow.h"
#include "EDMGapServoRecipeBinding.h"
#include <cstdint>
#include <limits>

// EDM65 observes an immutable, previously validated frozen SIM recipe against
// physical PDO receipts. Captured NC scope is provenance, NOT live authority.
// HOLD/RESET/M30 do not rearm or cancel this independent diagnostic observer.
// Only NC publishes Request; only the PDO owner mutates the observer/runtime.
// No Motion calls, allocation, locks, logging, clock reads or output writes.
namespace EDM65
{
constexpr bool PhysicalMotion=false;
constexpr bool Discharge=false;
constexpr const char* ConfigurationKind="FROZEN_SIM_RECIPE_PHYSICAL_SHADOW";
constexpr std::uint64_t RequestWindowUs=100000ULL;

struct Scope
{
    std::uint64_t session=0,run=0,cache=0,dispatch=0,epoch=0,ownerGeneration=0;
    std::uint32_t owner=0;
};
inline bool ValidScope(const Scope& s) noexcept
{ return s.session && s.run && s.cache && s.dispatch && s.epoch && s.ownerGeneration && s.owner==1U; }
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.session==b.session && a.run==b.run && a.cache==b.cache &&
        a.dispatch==b.dispatch && a.epoch==b.epoch && a.ownerGeneration==b.ownerGeneration && a.owner==b.owner;
}
struct Request
{
    EDM59::Recipe recipe{};
    Scope scope{};
    // Derived from NC's existing QPC value, without an extra clock read.
    // A PDO callback may already have sampled its clock when this is published.
    std::uint64_t capturedUs=0,qpcFrequency=0;
};
inline bool SameRequest(const Request& a,const Request& b) noexcept
{
    return SameScope(a.scope,b.scope) && EDM59::SameRecipe(a.recipe,b.recipe) &&
        a.capturedUs==b.capturedUs && a.qpcFrequency==b.qpcFrequency;
}
inline bool AdvanceCounter(std::uint64_t& value) noexcept
{
    if(value==(std::numeric_limits<std::uint64_t>::max)()) return false;
    ++value; return true;
}
enum class State : std::uint8_t { WaitRecipe, WaitSource, WaitRequestClock, AwaitFresh, Valid, Revoked };
enum class Reason : std::uint8_t
{
    None, WaitRecipe, WaitSource, WaitRequestClock, AwaitFresh, RequestInvalid,
    RequestScope, RequestClock, RequestClosed, Source, SourceIdentity,
    SourceSequence, SourceClock, SourceStale, Clock, Runtime, CounterExhausted, PublicationClosed
};
inline const char* StateName(State s) noexcept
{
    switch(s) {
    case State::WaitRecipe:return "WAIT_RECIPE"; case State::WaitSource:return "WAIT_SOURCE";
    case State::WaitRequestClock:return "WAIT_REQUEST_CLOCK"; case State::AwaitFresh:return "AWAIT_FRESH";
    case State::Valid:return "VALID"; case State::Revoked:return "REVOKED";
    } return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) {
    case Reason::None:return "NONE"; case Reason::WaitRecipe:return "WAIT_RECIPE";
    case Reason::WaitSource:return "WAIT_SOURCE"; case Reason::WaitRequestClock:return "WAIT_REQUEST_CLOCK";
    case Reason::AwaitFresh:return "AWAIT_FRESH"; case Reason::RequestInvalid:return "REQUEST_INVALID";
    case Reason::RequestScope:return "REQUEST_SCOPE"; case Reason::RequestClock:return "REQUEST_CLOCK";
    case Reason::RequestClosed:return "REQUEST_CLOSED"; case Reason::Source:return "SOURCE";
    case Reason::SourceIdentity:return "SOURCE_IDENTITY"; case Reason::SourceSequence:return "SOURCE_SEQUENCE";
    case Reason::SourceClock:return "SOURCE_CLOCK"; case Reason::SourceStale:return "SOURCE_STALE";
    case Reason::Clock:return "CLOCK"; case Reason::Runtime:return "RUNTIME";
    case Reason::CounterExhausted:return "COUNTER_EXHAUSTED";
    case Reason::PublicationClosed:return "PUBLICATION_CLOSED";
    } return "UNKNOWN";
}
// EDM66: historical coverage for this immutable recipe only. Counts describe
// accepted RT calculations and host PDO receipts, never ADC conversions,
// physical axis movement, or machining permission. A 1 Hz reader can therefore
// observe a short transition that began and ended between diagnostic reads.
struct Coverage
{
    std::uint64_t firstObservedUs=0,lastObservedUs=0,firstSourceSequence=0,lastSourceSequence=0;
    std::uint64_t validTicks=0,freshSamples=0,repeatSamples=0;
    std::uint64_t advanceTicks=0,retreatTicks=0,zeroTicks=0;
    std::uint64_t shortTicks=0,shortEntries=0,shortExits=0,feedInhibitTicks=0;
    double minRawV=0.0,maxRawV=0.0;
    bool hasSamples=false,initialShort=false,lastShort=false,saturated=false;
};
inline void CountCoverage(std::uint64_t& counter,bool& saturated) noexcept
{
    if(counter==(std::numeric_limits<std::uint64_t>::max)()) saturated=true;
    else ++counter;
}
inline void ObserveCoverage(Coverage& c,const EDM55::Output& output) noexcept
{
    // Only the PDO owner calls this after the runtime returns valid. Retaining
    // history on invalid output is deliberate; no counter changes authority.
    if(!output.valid) return;
    if(!c.hasSamples)
    {
        c.hasSamples=true; c.firstObservedUs=output.nowUs;
        c.firstSourceSequence=output.sequence;
        c.minRawV=c.maxRawV=output.rawVoltageV;
        // A first observation already short has no observed entry edge.
        c.initialShort=output.shortActive;
    }
    else
    {
        if(output.rawVoltageV<c.minRawV) c.minRawV=output.rawVoltageV;
        if(output.rawVoltageV>c.maxRawV) c.maxRawV=output.rawVoltageV;
        if(output.shortActive && !c.lastShort) CountCoverage(c.shortEntries,c.saturated);
        if(!output.shortActive && c.lastShort) CountCoverage(c.shortExits,c.saturated);
    }
    c.lastObservedUs=output.nowUs; c.lastSourceSequence=output.sequence;
    c.lastShort=output.shortActive;
    CountCoverage(c.validTicks,c.saturated);
    if(output.fresh) CountCoverage(c.freshSamples,c.saturated);
    else CountCoverage(c.repeatSamples,c.saturated);
    if(output.targetMmPerMin>0.0) CountCoverage(c.advanceTicks,c.saturated);
    else if(output.targetMmPerMin<0.0) CountCoverage(c.retreatTicks,c.saturated);
    else CountCoverage(c.zeroTicks,c.saturated);
    if(output.shortActive) CountCoverage(c.shortTicks,c.saturated);
    if(output.feedInhibited) CountCoverage(c.feedInhibitTicks,c.saturated);
}
struct Snapshot
{
    Request request{};
    EDM55::Identity identity{};
    EDM55::Output output{};
    State state=State::WaitRecipe;
    Reason reason=Reason::WaitRecipe;
    EDM64::ReaderState readerState=EDM64::ReaderState::Unchecked;
    std::uint64_t publication=0,recipePublication=0,observedUs=0,cycle=0,qpcFrequency=0;
    std::uint64_t observerGeneration=0,sourceSequence=0,sourceCapturedUs=0,sourceGeneration=0,sourceAgeUs=0;
    std::uint64_t maxAgeUs=5000,maxTickGapUs=2000;
    bool sourceAgeKnown=false,recipeAdopted=false,physicalPermit=false,discharge=false;
    Coverage coverage{};
};
inline void InvalidateDerived(Snapshot& s) noexcept
{
    const auto fault=s.output.fault;
    s.output=EDM55::Output{}; s.output.fault=fault; s.output.stopRequired=true;
    s.physicalPermit=false; s.discharge=false;
}
inline EDM55::Config RecipeConfig(const Request& request,std::uint64_t revision,
    std::uint64_t maxAgeUs,std::uint64_t maxTickGapUs) noexcept
{
    EDM55::Config c{};
    c.revision=revision; c.curve=request.recipe.curve;
    c.referenceV=request.recipe.referenceV; c.overridePercent=request.recipe.overridePercent;
    if(maxAgeUs<c.maxAgeUs) c.maxAgeUs=maxAgeUs;
    if(maxTickGapUs<c.maxTickGapUs) c.maxTickGapUs=maxTickGapUs;
    if(c.maxTickGapUs>c.maxAgeUs) c.maxTickGapUs=c.maxAgeUs;
    return c;
}

class PhysicalRecipeRTShadow
{
public:
    // No observer state is touched here. Invalid publications revoke on the
    // next PDO callback rather than silently preserving an earlier recipe.
    bool PublishFromNC(const Request& request) noexcept { return requests_.Publish(request); }

    void ObserveFromPdo(const EDM64::Snapshot& physical,std::uint64_t nowUs,
        bool clockValid,std::uint64_t frequency) noexcept
    {
        if(publicationClosed_) return;
        if(!AdvanceCounter(current_.cycle)) { Fail(Reason::CounterExhausted); Publish(); return; }
        if(current_.state==State::Revoked) { Publish(); return; }
        if(physical.state==EDM64::State::Revoked || physical.state==EDM64::State::Disabled)
        { Fail(Reason::Source); Publish(); return; }
        if(!clockValid || !nowUs || !frequency)
        {
            if(lastNowUs_ || seenRequest_) Fail(Reason::Clock);
            else Wait(State::WaitRecipe,Reason::WaitRecipe);
            Publish(); return;
        }
        if((lastNowUs_ && nowUs<=lastNowUs_) || (frequency_ && frequency!=frequency_) ||
            (sourceSeen_ && nowUs-lastNowUs_>current_.maxTickGapUs))
        { Fail(Reason::Clock); Publish(); return; }
        lastNowUs_=nowUs; frequency_=frequency;
        current_.observedUs=nowUs; current_.qpcFrequency=frequency;

        // Source health is checked BEFORE a replacement request can discard
        // history. A fresh receipt/recipe cannot repair an expired interval.
        const bool sourceValid=CheckSource(physical,nowUs,frequency);
        if(current_.state==State::Revoked) { Publish(); return; }

        Request request{};
        auto stamp=requests_.Current();
        // The immutable recipe normally remains unchanged for many RT ticks.
        // Acquire the full coherent bank only when its publication changes.
        if(stamp.Ready() && stamp.publication!=seenPublication_) stamp=requests_.Read(request);
        if(stamp.status==EDM53::Status::Closed)
        { Fail(Reason::RequestClosed); Publish(); return; }
        // Busy is bounded; retain the previous immutable request only while
        // the independent physical source/owner clock remain healthy.
        if(stamp.Ready() && stamp.publication!=seenPublication_)
        {
            if(stamp.publication<seenPublication_)
            { Fail(Reason::RequestScope); Publish(); return; }
            seenPublication_=stamp.publication;
            if(!seenRequest_ || !SameRequest(request,current_.request))
            {
                const auto previousSession=current_.request.scope.session;
                runtime_.Cancel(); InvalidateDerived(current_);
                current_.request=request; current_.recipePublication=stamp.publication;
                // Provenance changes here, even if the replacement is later
                // rejected. Never label the old recipe's history as new.
                // Same-request publications, HOLD, RESET and reads do not reset.
                current_.coverage=Coverage{};
                current_.recipeAdopted=false; seeded_=false; pending_=true;
                if(!ValidScope(request.scope) || (seenRequest_ && request.scope.session<=previousSession))
                { Fail(Reason::RequestScope); Publish(); return; }
                seenRequest_=true;
                if(!EDM59::ValidateRecipe(request.recipe))
                { Fail(Reason::RequestInvalid); Publish(); return; }
                if(!request.capturedUs || !request.qpcFrequency || request.qpcFrequency!=frequency ||
                    (request.capturedUs>nowUs && request.capturedUs-nowUs>RequestWindowUs))
                { Fail(Reason::RequestClock); Publish(); return; }
            }
        }
        if(!seenRequest_) { Wait(State::WaitRecipe,Reason::WaitRecipe); Publish(); return; }
        if(pending_)
        {
            if(current_.request.capturedUs>nowUs)
            { Wait(State::WaitRequestClock,Reason::WaitRequestClock); Publish(); return; }
            if(nowUs-current_.request.capturedUs>RequestWindowUs)
            { Fail(Reason::RequestClock); Publish(); return; }
            if(!sourceValid) { Wait(State::WaitSource,Reason::WaitSource); Publish(); return; }
            if(!AdvanceCounter(current_.observerGeneration))
            { Fail(Reason::CounterExhausted); Publish(); return; }
            const auto config=RecipeConfig(current_.request,current_.recipePublication,
                current_.maxAgeUs,current_.maxTickGapUs);
            current_.identity=physical.identity;
            current_.identity.session=current_.observerGeneration;
            if(!runtime_.Configure(config) || !runtime_.Start(current_.identity))
            { current_.output=runtime_.Current(); Fail(Reason::Runtime); Publish(); return; }
            current_.recipeAdopted=true; pending_=false;
            admittedUs_=nowUs; baselineSequence_=current_.sourceSequence;
            baselineCapturedUs_=current_.sourceCapturedUs;
            Wait(State::AwaitFresh,Reason::AwaitFresh); Publish(); return;
        }
        if(!sourceValid) { Fail(Reason::Source); Publish(); return; }
        if(!seeded_)
        {
            if(nowUs-admittedUs_>current_.maxTickGapUs)
            { Fail(Reason::Clock); Publish(); return; }
            if(current_.sourceSequence<=baselineSequence_ || current_.sourceCapturedUs<=baselineCapturedUs_ ||
                current_.sourceCapturedUs<=admittedUs_)
            { Wait(State::AwaitFresh,Reason::AwaitFresh); Publish(); return; }
        }
        EDM55::Sample sample{};
        sample.valid=true; sample.identity=current_.identity;
        sample.sequence=current_.sourceSequence; sample.capturedAtUs=current_.sourceCapturedUs;
        sample.voltageV=static_cast<double>(physical.input.conversion.voltageMv)/1000.0;
        current_.output=runtime_.Step(sample,nowUs,current_.cycle);
        if(!current_.output.valid)
        { Fail(Reason::Runtime); Publish(); return; }
        ObserveCoverage(current_.coverage,current_.output);
        seeded_=true; current_.state=State::Valid; current_.reason=Reason::None;
        Publish();
    }

    EDM53::Stamp ReadRaw(Snapshot& out) const noexcept { return snapshots_.Read(out); }
    // Raw read MUST precede the consumer clock; validate exactly that copy.
    static EDM53::Stamp ValidateRead(Snapshot& out,const EDM53::Stamp& stamp,
        std::uint64_t nowUs,bool clockValid,std::uint64_t frequency) noexcept
    {
        out.publication=stamp.publication;
        if(!stamp.Ready())
        {
            out.readerState=stamp.status==EDM53::Status::Closed?EDM64::ReaderState::Closed:
                (stamp.status==EDM53::Status::Busy?EDM64::ReaderState::Busy:EDM64::ReaderState::Empty);
            out.sourceAgeKnown=false; InvalidateDerived(out); return stamp;
        }
        if(!out.observedUs)
        { out.readerState=EDM64::ReaderState::Unobserved; InvalidateDerived(out); return stamp; }
        if(!clockValid || !nowUs || !frequency || frequency!=out.qpcFrequency || nowUs<out.observedUs ||
            (out.sourceCapturedUs && nowUs<out.sourceCapturedUs))
        {
            out.readerState=EDM64::ReaderState::Clock;
            out.sourceAgeKnown=false; InvalidateDerived(out); return stamp;
        }
        out.sourceAgeKnown=out.sourceCapturedUs!=0;
        out.sourceAgeUs=out.sourceAgeKnown?nowUs-out.sourceCapturedUs:0;
        if(nowUs-out.observedUs>out.maxTickGapUs || (out.sourceAgeKnown && out.sourceAgeUs>out.maxAgeUs))
        { out.readerState=EDM64::ReaderState::Stale; InvalidateDerived(out); return stamp; }
        out.readerState=EDM64::ReaderState::Fresh; return stamp;
    }
    // Same-owner accessor only. Concurrent diagnostics must use ReadRaw.
    const Snapshot& SnapshotFromOwner() const noexcept { return current_; }
private:
    bool CheckSource(const EDM64::Snapshot& p,std::uint64_t nowUs,std::uint64_t frequency) noexcept
    {
        if(p.state!=EDM64::State::Valid)
        {
            if(sourceSeen_) Fail(Reason::Source);
            return false;
        }
        if(!p.input.Usable() || !p.output.valid || p.output.fault!=EDM55::Fault::None ||
            EDM63::SampleStatus(p.input.sample,true)!=EDM63::Reason::None ||
            p.input.conversion.inputStatus!=EDMGapInput::InputStatus::Valid ||
            !p.input.conversion.boardVoltageValid || !p.input.conversion.gapInRange ||
            p.physicalPermit || p.discharge || p.input.physicalPermit ||
            !p.maxAgeUs || p.maxAgeUs>5000ULL || !p.maxTickGapUs || p.maxTickGapUs>2000ULL ||
            p.maxTickGapUs>p.maxAgeUs)
        { Fail(Reason::Source); return false; }
        if(p.observedUs!=nowUs || p.qpcFrequency!=frequency ||
            p.input.sample.qpcFrequency!=frequency)
        { Fail(Reason::SourceClock); return false; }
        if(!EDM55::ValidIdentity(p.identity) || p.identity.source!=EDM55::SourceKind::Physical ||
            !p.input.sample.producerGeneration || p.identity.sourceGeneration!=p.input.observerGeneration ||
            p.identity.calibrationRevision!=p.input.expectedCalibrationRevision ||
            p.identity.sourceId!=EDM63::PhysicalSourceId(p.input.sample.deviceId,p.input.sample.channelId))
        { Fail(Reason::SourceIdentity); return false; }
        if(sourceSeen_ && (!EDM55::SameIdentity(sourceIdentity_,p.identity) ||
            current_.sourceGeneration!=p.input.sample.producerGeneration ||
            current_.maxAgeUs!=p.maxAgeUs || current_.maxTickGapUs!=p.maxTickGapUs))
        { Fail(Reason::SourceIdentity); return false; }
        if(sourceSeen_ && nowUs-current_.sourceCapturedUs>current_.maxAgeUs)
        { Fail(Reason::SourceStale); return false; }
        const auto seq=p.input.sample.sequence;
        const auto capture=p.input.sample.capturedAtUs;
        if(!seq || (sourceSeen_ && seq<current_.sourceSequence))
        { Fail(Reason::SourceSequence); return false; }
        if(!capture || capture>nowUs || (sourceSeen_ && seq>current_.sourceSequence && capture<=current_.sourceCapturedUs))
        { Fail(Reason::SourceClock); return false; }
        if(nowUs-capture>p.maxAgeUs) { Fail(Reason::SourceStale); return false; }
        if(sourceSeen_ && seq==current_.sourceSequence && (capture!=current_.sourceCapturedUs ||
            p.input.conversion.voltageMv!=lastVoltageMv_ || p.input.sample.rawCode!=lastRawCode_))
        { Fail(Reason::SourceSequence); return false; }
        sourceSeen_=true; sourceIdentity_=p.identity;
        current_.sourceSequence=seq; current_.sourceCapturedUs=capture;
        current_.sourceGeneration=p.input.sample.producerGeneration;
        current_.sourceAgeUs=nowUs-capture; current_.sourceAgeKnown=true;
        current_.maxAgeUs=p.maxAgeUs; current_.maxTickGapUs=p.maxTickGapUs;
        lastVoltageMv_=p.input.conversion.voltageMv; lastRawCode_=p.input.sample.rawCode;
        return true;
    }
    void Wait(State state,Reason reason) noexcept
    { current_.state=state; current_.reason=reason; InvalidateDerived(current_); }
    void Fail(Reason reason) noexcept
    {
        if(current_.state!=State::Revoked) { current_.state=State::Revoked; current_.reason=reason; }
        runtime_.Cancel(); InvalidateDerived(current_);
    }
    void Publish() noexcept
    {
        if(!snapshots_.Publish(current_))
        { publicationClosed_=true; Fail(Reason::PublicationClosed); }
    }
    EDM53::Channel<Request> requests_{};
    EDM53::Channel<Snapshot> snapshots_{};
    EDM55::Runtime runtime_{};
    Snapshot current_{};
    EDM55::Identity sourceIdentity_{};
    std::uint64_t lastNowUs_=0,frequency_=0,seenPublication_=0,admittedUs_=0;
    std::uint64_t baselineSequence_=0,baselineCapturedUs_=0;
    std::int64_t lastVoltageMv_=0;
    std::int32_t lastRawCode_=0;
    bool sourceSeen_=false,seenRequest_=false,pending_=false,seeded_=false,publicationClosed_=false;
};
static_assert(sizeof(Request)<=384U,"EDM65 recipe mailbox must remain bounded.");
static_assert(sizeof(Coverage)<=144U,"EDM66 historical coverage must stay bounded.");
static_assert(sizeof(Snapshot)<=768U,"EDM65 diagnostic snapshot must remain bounded.");
static_assert(sizeof(PhysicalRecipeRTShadow)<=4096U,"EDM65 observer must remain bounded.");
}
