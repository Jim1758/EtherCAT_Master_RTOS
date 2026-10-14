#pragma once

#include "EDMGapPhysicalRecipeRTShadow.h"
#include "EDMGapServoRTSession.h"
#include "EDMPhysicalRetreatContract.h"
#include "EDMPhysicalBidirectionalContract.h"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// EDM69: a finite, live-authority PHYSICAL input consumer on the Motion owner.
// Its own EDM55 kernel calculates advisory values only. There is no axis,
// planner, discharge or physical-stop interface. NC owns control methods; one
// actual RT owner calls RuntimeCycle. All storage and callback work are bounded.
namespace EDM69
{
using Facts = EDM55RT::Facts;
constexpr std::uint64_t DurationUs=10000000ULL;
constexpr std::uint64_t StartDeadlineUs=500000ULL;
constexpr std::uint64_t TotalDeadlineUs=12000000ULL;
constexpr std::uint64_t HeartbeatMaximumAgeUs=100000ULL;
enum class State : std::uint8_t { Idle, AwaitingStart, Running, Draining, Completed, Cancelled, Fault };
enum class Reason : std::uint8_t
{
    None, AwaitStart, Cancelled, InvalidStart, StartDeadline, TotalDeadline,
    RuntimeClock, RuntimeSource, AuthorityLost, Heartbeat, Source,
    SourceIdentity, SourceSequence, SourceClock, SourceStale, Kernel,
    TargetMismatch, Coverage, CounterOverflow
};
enum class RecipeFamily : std::uint8_t { Legacy=0, PhysicalRetreat=1, PhysicalBidirectional=2, PhysicalBidirectionalEndurance=3 };
// EDM76 is a distinct immutable source family. Its longer window does not
// extend any earlier physical fixture or relax the live source/authority gates.
constexpr std::uint64_t DurationUsFor(RecipeFamily family) noexcept
{
    switch(family) {
    case RecipeFamily::Legacy: case RecipeFamily::PhysicalRetreat:
    case RecipeFamily::PhysicalBidirectional:return DurationUs;
    case RecipeFamily::PhysicalBidirectionalEndurance:return 60000000ULL;
    } return 0ULL;
}
constexpr std::uint64_t TotalDeadlineUsFor(RecipeFamily family) noexcept
{
    switch(family) {
    case RecipeFamily::Legacy: case RecipeFamily::PhysicalRetreat:
    case RecipeFamily::PhysicalBidirectional:return TotalDeadlineUs;
    case RecipeFamily::PhysicalBidirectionalEndurance:return 62000000ULL;
    } return 0ULL;
}
constexpr std::uint64_t MaximumRuntimeStepsFor(RecipeFamily family) noexcept
{
    switch(family) {
    case RecipeFamily::Legacy: case RecipeFamily::PhysicalRetreat:
    case RecipeFamily::PhysicalBidirectional:return 100000ULL;
    case RecipeFamily::PhysicalBidirectionalEndurance:return 600000ULL;
    } return 0ULL;
}
struct Request
{
    EDM65::Request capture{};
    RecipeFamily recipeFamily=RecipeFamily::Legacy;
    EDM55::Identity physicalIdentity{}; // Independent physical observer identity.
    EDM63::Route route{};
    std::uint64_t baselineSequence=0,baselineCapturedUs=0,maxAgeUs=5000,maxTickGapUs=2000;
};
inline bool ValidRequest(const Request& r) noexcept
{
    bool recipeValid=false;
    switch(r.recipeFamily)
    {
    case RecipeFamily::Legacy: recipeValid=EDM59::ValidateRecipe(r.capture.recipe); break;
    case RecipeFamily::PhysicalRetreat: recipeValid=EDM72::ValidateRecipe(r.capture.recipe); break;
    case RecipeFamily::PhysicalBidirectional:
    case RecipeFamily::PhysicalBidirectionalEndurance: recipeValid=EDM74::ValidateRecipe(r.capture.recipe); break;
    default: return false;
    }
    const auto& route=r.route;
    return EDM65::ValidScope(r.capture.scope) && r.capture.scope.ownerGeneration<=0xffffffffULL && recipeValid &&
        r.capture.capturedUs && r.capture.qpcFrequency &&
        r.capture.qpcFrequency<=(std::numeric_limits<std::uint64_t>::max)()/1000000ULL &&
        EDM55::ValidIdentity(r.physicalIdentity) && r.physicalIdentity.source==EDM55::SourceKind::Physical &&
        r.physicalIdentity.sourceGeneration==r.physicalIdentity.session &&
        route.adIndex && route.adIndex<=64U && route.channelId==route.adIndex &&
        route.channelCount>=route.adIndex && route.deviceId && route.deviceId<=65535U &&
        route.slaveIndex>=0 && route.inputBitOffset>=0 && route.producerGeneration &&
        route.qpcFrequency==r.capture.qpcFrequency &&
        r.physicalIdentity.sourceId==EDM63::PhysicalSourceId(route.deviceId,route.channelId) &&
        r.baselineSequence && r.baselineSequence!=(std::numeric_limits<std::uint64_t>::max)() &&
        r.baselineCapturedUs && r.baselineCapturedUs<=r.capture.capturedUs &&
        r.capture.capturedUs-r.baselineCapturedUs<=r.maxAgeUs &&
        r.maxAgeUs && r.maxAgeUs<=5000ULL && r.maxTickGapUs &&
        r.maxTickGapUs<=2000ULL && r.maxTickGapUs<=r.maxAgeUs;
}
inline bool SameRequest(const Request& a,const Request& b) noexcept
{
    return a.recipeFamily==b.recipeFamily && EDM65::SameRequest(a.capture,b.capture) && EDM55::SameIdentity(a.physicalIdentity,b.physicalIdentity) &&
        EDM63::SameRoute(a.route,b.route) && a.baselineSequence==b.baselineSequence &&
        a.baselineCapturedUs==b.baselineCapturedUs && a.maxAgeUs==b.maxAgeUs && a.maxTickGapUs==b.maxTickGapUs;
}
struct Feedback
{
    Request request{};
    State state=State::Idle;
    Reason reason=Reason::None;
    EDM55::Output output{};
    EDM65::Coverage coverage{};
    EDM55::Identity kernelIdentity{};
    std::uint64_t publication=0,latestSession=0,cancelThroughSession=0,rtTick=0,nowUs=0,qpcFrequency=0;
    std::uint64_t startedTick=0,startedUs=0,terminalTick=0,zeroReceiptTick=0;
    std::uint64_t firstSourceCapturedUs=0,sourceSequence=0,sourceCapturedUs=0,sourcePublication=0;
    std::uint64_t runtimeSteps=0,minimumStepUs=0,maximumStepUs=0,maximumSourceAgeUs=0;
    double maximumTargetError=0.0;
    bool startAcknowledged=false,zeroReceipt=false,physicalPermit=false,discharge=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
inline bool Terminal(State state) noexcept
{ return state==State::Completed || state==State::Cancelled || state==State::Fault; }
inline const char* StateName(State state) noexcept
{
    switch(state) {
    case State::Idle:return "IDLE"; case State::AwaitingStart:return "AWAITING_START";
    case State::Running:return "RUNNING"; case State::Draining:return "DRAINING";
    case State::Completed:return "COMPLETED"; case State::Cancelled:return "CANCELLED";
    case State::Fault:return "FAULT";
    } return "UNKNOWN";
}
inline const char* ReasonName(Reason reason) noexcept
{
    switch(reason) {
    case Reason::None:return "NONE"; case Reason::AwaitStart:return "AWAIT_START";
    case Reason::Cancelled:return "CANCELLED"; case Reason::InvalidStart:return "INVALID_START";
    case Reason::StartDeadline:return "START_DEADLINE"; case Reason::TotalDeadline:return "TOTAL_DEADLINE";
    case Reason::RuntimeClock:return "RUNTIME_CLOCK"; case Reason::RuntimeSource:return "RUNTIME_SOURCE";
    case Reason::AuthorityLost:return "AUTHORITY_LOST"; case Reason::Heartbeat:return "HEARTBEAT";
    case Reason::Source:return "SOURCE"; case Reason::SourceIdentity:return "SOURCE_IDENTITY";
    case Reason::SourceSequence:return "SOURCE_SEQUENCE"; case Reason::SourceClock:return "SOURCE_CLOCK";
    case Reason::SourceStale:return "SOURCE_STALE"; case Reason::Kernel:return "KERNEL";
    case Reason::TargetMismatch:return "TARGET_MISMATCH"; case Reason::Coverage:return "COVERAGE";
    case Reason::CounterOverflow:return "COUNTER_OVERFLOW";
    } return "UNKNOWN";
}

class Session
{
public:
    bool ControlStart(const Request& request) noexcept
    {
        const auto session=request.capture.scope.session;
        const auto latest=latest_.load(std::memory_order_acquire);
        if(!session || session<=latest || session<=cancelled_.load(std::memory_order_acquire)) return false;
        if(latest)
        {
            Feedback previous{};
            if(!ControlRead(previous) || !Terminal(previous.state) || !previous.zeroReceipt ||
                previous.request.capture.scope.session!=latest || previous.rtTick<previous.zeroReceiptTick ||
                !previous.nowUs || request.capture.capturedUs<previous.nowUs ||
                request.capture.capturedUs-previous.nowUs>(previous.request.maxTickGapUs &&
                    previous.request.maxTickGapUs<=2000ULL?previous.request.maxTickGapUs:2000ULL)) return false;
        }
        // Reserve every submitted new identity, including malformed starts.
        // Its immutable request is still delivered for same-request cancel/zero
        // evidence; a corrected payload must use another explicit session.
        latest_.store(session,std::memory_order_release);
        if(!requests_.ProducerTryPush(request)) { ControlCancel(session); return false; }
        if(!ValidRequest(request) || !ControlTouch(session,request.capture.capturedUs))
        { ControlCancel(session); return false; }
        return true;
    }
    bool ControlTouch(std::uint64_t session,std::uint64_t nowUs) noexcept
    {
        if(!session || !nowUs || session!=latest_.load(std::memory_order_acquire) ||
            session<=cancelled_.load(std::memory_order_acquire) || nowUs<lastControlHeartbeatUs_) return false;
        Heartbeat heartbeat{}; heartbeat.session=session; heartbeat.atUs=nowUs;
        if(!heartbeats_.Publish(heartbeat)) { ControlCancel(session); return false; }
        lastControlHeartbeatUs_=nowUs; return true;
    }
    void ControlCancel(std::uint64_t session) noexcept
    {
        if(session>cancelled_.load(std::memory_order_relaxed))
            cancelled_.store(session,std::memory_order_release);
    }
    bool ControlRead(Feedback& output) const noexcept
    {
        output=Feedback{};
        const auto latest=latest_.load(std::memory_order_acquire);
        const auto cancelled=cancelled_.load(std::memory_order_acquire);
        Feedback candidate{}; const auto stamp=feedback_.Read(candidate);
        if(!stamp.Ready() || latest!=latest_.load(std::memory_order_acquire) ||
            cancelled!=cancelled_.load(std::memory_order_acquire) ||
            candidate.latestSession!=latest || candidate.cancelThroughSession!=cancelled) return false;
        candidate.publication=stamp.publication; output=candidate; return true;
    }
    std::uint64_t ControlLatestSession() const noexcept
    { return latest_.load(std::memory_order_acquire); }
    bool NeedsRuntimeService() const noexcept
    { return latest_.load(std::memory_order_acquire)!=0; }

    // The adapter captures raw/stamp BEFORE its own QPC read and supplies that
    // same clock/frequency. Never reread the source after taking this clock.
    void RuntimeCycle(const Facts& facts,EDM64::Snapshot raw,const EDM53::Stamp& stamp,
        std::uint64_t frequency) noexcept
    {
        if(!NeedsRuntimeService()) return;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs; f_.qpcFrequency=frequency;
        ObserveFences();
        Request request{};
        for(unsigned i=0;i<4U && requests_.ConsumerTryPop(request);++i)
        {
            ObserveFences();
            if(request.capture.scope.session==f_.latestSession) Adopt(request,facts,frequency);
        }
        ObserveFences();
        const auto gap=f_.request.maxTickGapUs && f_.request.maxTickGapUs<=2000ULL?f_.request.maxTickGapUs:2000ULL;
        const bool monotonic=facts.clockValid && facts.tick && facts.nowUs && frequency &&
            (!haveClock_ || (facts.tick>previousTick_ && facts.nowUs>previousUs_));
        const auto delta=monotonic && haveClock_?facts.nowUs-previousUs_:0ULL;
        const bool clock=monotonic && (!haveClock_ ||
            (facts.tick-previousTick_==1ULL && delta<=gap && frequency==previousFrequency_));
        // A bad interval can be followed by new valid neutral observations, but
        // it cannot repair a running/completed session's sticky fault.
        if(monotonic) { previousTick_=facts.tick; previousUs_=facts.nowUs; previousFrequency_=frequency; haveClock_=true; }
        else if(facts.clockValid && facts.tick && facts.nowUs &&
            facts.tick>previousTick_ && facts.nowUs>previousUs_)
        { previousTick_=facts.tick; previousUs_=facts.nowUs; previousFrequency_=frequency; haveClock_=true; }
        Heartbeat heartbeat{};
        if(heartbeats_.Read(heartbeat).Ready() && heartbeat.session==f_.request.capture.scope.session &&
            heartbeat.atUs>=lastRuntimeHeartbeatUs_ && heartbeat.atUs<=facts.nowUs)
            lastRuntimeHeartbeatUs_=heartbeat.atUs;
        if(f_.request.capture.scope.session==f_.latestSession &&
            (f_.state==State::AwaitingStart || f_.state==State::Running ||
            f_.state==State::Draining || f_.state==State::Completed))
        {
            if(!clock || frequency!=f_.request.capture.qpcFrequency) Trip(Reason::RuntimeClock);
            else if(!facts.pdoValid || !facts.contiguous) Trip(Reason::RuntimeSource);
            else if(!Authority(facts)) Trip(Reason::AuthorityLost);
            else if(f_.request.capture.capturedUs>facts.nowUs)
            {
                if(f_.request.capture.capturedUs-facts.nowUs>HeartbeatMaximumAgeUs) Trip(Reason::InvalidStart);
            }
            else if(facts.nowUs-f_.request.capture.capturedUs>=TotalDeadlineUsFor(f_.request.recipeFamily)) Trip(Reason::TotalDeadline);
            else if(!HeartbeatReady(facts.nowUs)) Trip(Reason::Heartbeat);
            else if(f_.state==State::AwaitingStart &&
                facts.nowUs-f_.request.capture.capturedUs>=StartDeadlineUs) Trip(Reason::StartDeadline);
            else if(CheckSource(raw,stamp,facts.nowUs,frequency))
            {
                if(f_.state==State::AwaitingStart) Start(facts);
                if(f_.state==State::Running) Step(facts,delta);
                else if(f_.state==State::Draining && facts.tick>f_.terminalTick)
                {
                    Neutralize(); f_.zeroReceipt=true; f_.zeroReceiptTick=facts.tick;
                    f_.state=State::Completed; f_.reason=Reason::None;
                }
            }
        }
        if(f_.state==State::Fault || f_.state==State::Cancelled)
        {
            // Cancellation/fault zero needs only a later valid contiguous PDO
            // clock, not the retired run/lease/source authority.
            const bool zeroClock=clock && facts.pdoValid && facts.contiguous;
            if(!zeroClock)
            { f_.zeroReceipt=false; f_.zeroReceiptTick=0; neutralFloorTick_=0; }
            else if(f_.request.capture.scope.session==f_.latestSession)
            {
                if(!f_.terminalTick) f_.terminalTick=facts.tick;
                if(!neutralFloorTick_) neutralFloorTick_=facts.tick;
                else if(facts.tick>neutralFloorTick_ && facts.tick>f_.terminalTick)
                {
                    Neutralize();
                    if(!f_.zeroReceipt) { f_.zeroReceipt=true; f_.zeroReceiptTick=facts.tick; }
                }
            }
        }
        ObserveFences();
        if(!feedback_.Publish(f_)) Trip(Reason::CounterOverflow);
    }
private:
    struct Heartbeat { std::uint64_t session=0,atUs=0; };
    FixedCapacitySpscRing<Request,8U> requests_{};
    EDM53::Channel<Heartbeat> heartbeats_{};
    EDM53::Channel<Feedback> feedback_{};
    std::atomic<std::uint64_t> latest_{0},cancelled_{0};
    std::uint64_t lastControlHeartbeatUs_=0;
    EDM55::Runtime core_{};
    Feedback f_{};
    EDM64::Snapshot physical_{};
    EDM53::Stamp physicalStamp_{};
    std::uint64_t previousTick_=0,previousUs_=0,previousFrequency_=0,lastRuntimeHeartbeatUs_=0,neutralFloorTick_=0;
    std::uint64_t admissionSequence_=0,admissionCapturedUs_=0,admissionUs_=0;
    bool haveClock_=false,havePhysical_=false,configured_=false;
    void Neutralize() noexcept
    {
        f_.output.valid=f_.output.running=f_.output.fresh=false;
        f_.output.targetMmPerMin=f_.output.speedMmPerMin=0.0;
        f_.output.feedInhibited=true; f_.output.stopRequired=true;
        f_.physicalPermit=f_.discharge=false;
    }
    void Trip(Reason reason) noexcept
    {
        if(f_.state!=State::Fault)
        {
            f_.state=State::Fault; f_.reason=reason; f_.terminalTick=f_.rtTick;
            f_.zeroReceipt=false; f_.zeroReceiptTick=0; neutralFloorTick_=0;
        }
        core_.Cancel(); Neutralize();
    }
    void Adopt(const Request& request,const Facts& facts,std::uint64_t frequency) noexcept
    {
        core_.Cancel();
        const auto latest=f_.latestSession,cancelled=f_.cancelThroughSession;
        f_=Feedback{}; f_.request=request; f_.latestSession=latest; f_.cancelThroughSession=cancelled;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs; f_.qpcFrequency=frequency;
        f_.state=State::AwaitingStart; f_.reason=Reason::AwaitStart;
        lastRuntimeHeartbeatUs_=request.capture.capturedUs;
        neutralFloorTick_=0; havePhysical_=configured_=false;
        admissionSequence_=admissionCapturedUs_=admissionUs_=0;
        physical_=EDM64::Snapshot{}; physicalStamp_=EDM53::Stamp{};
        if(!ValidRequest(request)) Trip(Reason::InvalidStart);
    }
    void ObserveFences() noexcept
    {
        const auto latest=latest_.load(std::memory_order_acquire);
        const auto cancelled=cancelled_.load(std::memory_order_acquire);
        const bool changed=latest!=f_.latestSession || cancelled!=f_.cancelThroughSession;
        f_.latestSession=latest; f_.cancelThroughSession=cancelled;
        if(latest && latest<=cancelled)
        {
            if(f_.state!=State::Cancelled && f_.state!=State::Fault)
            {
                core_.Cancel(); Neutralize(); f_.state=State::Cancelled; f_.reason=Reason::Cancelled;
                f_.terminalTick=f_.rtTick; f_.zeroReceipt=false; f_.zeroReceiptTick=0; neutralFloorTick_=0;
            }
            else if(changed)
            { f_.zeroReceipt=false; f_.zeroReceiptTick=0; neutralFloorTick_=0; }
        }
        else if(f_.request.capture.scope.session!=latest)
        {
            core_.Cancel(); Neutralize(); f_.state=State::AwaitingStart; f_.reason=Reason::AwaitStart;
            f_.startAcknowledged=f_.zeroReceipt=false; f_.zeroReceiptTick=0;
        }
    }
    bool Authority(const Facts& facts) const noexcept
    {
        return facts.authorityReady && facts.executionEpoch==f_.request.capture.scope.epoch &&
            facts.ownerLease==((static_cast<std::uint64_t>(f_.request.capture.scope.owner)<<32U) |
                f_.request.capture.scope.ownerGeneration);
    }
    bool HeartbeatReady(std::uint64_t nowUs) const noexcept
    { return lastRuntimeHeartbeatUs_ && lastRuntimeHeartbeatUs_<=nowUs && nowUs-lastRuntimeHeartbeatUs_<=HeartbeatMaximumAgeUs; }
    bool CheckSource(EDM64::Snapshot raw,EDM53::Stamp stamp,std::uint64_t nowUs,std::uint64_t frequency) noexcept
    {
        // A transient bounded read collision retains the genuine old timestamp.
        // It cannot refresh source age, source sequence or observer freshness.
        if(stamp.status==EDM53::Status::Busy && havePhysical_) { raw=physical_; stamp=physicalStamp_; }
        if(!stamp.Ready())
        {
            if(f_.state==State::AwaitingStart && (stamp.status==EDM53::Status::Busy || stamp.status==EDM53::Status::Empty))
                return false;
            Trip(Reason::Source); return false;
        }
        EDM64::PhysicalRTShadow::ValidateRead(raw,stamp,nowUs,true,frequency);
        if(raw.readerState!=EDM64::ReaderState::Fresh)
        { Trip(raw.readerState==EDM64::ReaderState::Stale?Reason::SourceStale:Reason::SourceClock); return false; }
        const auto& r=f_.request;
        if(raw.state!=EDM64::State::Valid || raw.reason!=EDM64::Reason::None || !raw.input.Usable() ||
            !raw.output.valid || raw.output.fault!=EDM55::Fault::None ||
            raw.physicalPermit || raw.discharge || raw.input.physicalPermit ||
            raw.maxAgeUs!=r.maxAgeUs || raw.maxTickGapUs!=r.maxTickGapUs ||
            EDM63::SampleStatus(raw.input.sample,true)!=EDM63::Reason::None ||
            raw.input.conversion.inputStatus!=EDMGapInput::InputStatus::Valid ||
            !raw.input.conversion.boardVoltageValid || !raw.input.conversion.gapInRange)
        { Trip(Reason::Source); return false; }
        const auto& s=raw.input.sample;
        if(!EDM55::SameIdentity(raw.identity,r.physicalIdentity) ||
            !EDM63::SameRoute(EDM63::ProducerRoute(s),r.route) ||
            raw.identity.sourceGeneration!=raw.input.observerGeneration ||
            raw.identity.calibrationRevision!=raw.input.expectedCalibrationRevision ||
            raw.identity.session!=raw.input.observerGeneration)
        { Trip(Reason::SourceIdentity); return false; }
        if(!s.sequence || !s.capturedAtUs || s.capturedAtUs!=s.observedAtUs ||
            s.capturedAtMs!=s.capturedAtUs/1000ULL || s.observedAtMs!=s.observedAtUs/1000ULL ||
            s.qpcFrequency!=frequency || s.capturedAtUs>nowUs || s.capturedAtUs>raw.observedUs ||
            raw.output.sequence!=s.sequence || raw.output.capturedAtUs!=s.capturedAtUs ||
            raw.output.tick!=raw.cycle || raw.output.nowUs!=raw.observedUs)
        { Trip(Reason::SourceClock); return false; }
        if(nowUs-s.capturedAtUs>r.maxAgeUs ||
            (havePhysical_ && nowUs-physical_.input.sample.capturedAtUs>r.maxAgeUs))
        { Trip(Reason::SourceStale); return false; }
        if(havePhysical_)
        {
            const auto& old=physical_.input.sample;
            if(stamp.publication<physicalStamp_.publication || raw.cycle<physical_.cycle ||
                raw.observedUs<physical_.observedUs || s.sequence<old.sequence ||
                (s.sequence>old.sequence && s.capturedAtUs<=old.capturedAtUs) ||
                (s.sequence==old.sequence && (!EDM63::SameSample(s,old) ||
                    raw.input.conversion.voltageMv!=physical_.input.conversion.voltageMv)) ||
                (stamp.publication==physicalStamp_.publication &&
                    (raw.cycle!=physical_.cycle || raw.observedUs!=physical_.observedUs || !EDM63::SameSample(s,old))))
            { Trip(Reason::SourceSequence); return false; }
        }
        havePhysical_=true; physical_=raw; physicalStamp_=stamp;
        f_.sourceSequence=s.sequence; f_.sourceCapturedUs=s.capturedAtUs; f_.sourcePublication=stamp.publication;
        const auto age=nowUs-s.capturedAtUs;
        if(age>f_.maximumSourceAgeUs) f_.maximumSourceAgeUs=age;
        return true;
    }
    void Start(const Facts& facts) noexcept
    {
        const auto& r=f_.request;
        if(!admissionUs_)
        {
            admissionUs_=facts.nowUs; admissionSequence_=f_.sourceSequence;
            admissionCapturedUs_=f_.sourceCapturedUs; return;
        }
        if(f_.sourceSequence<=admissionSequence_ || f_.sourceCapturedUs<=admissionCapturedUs_ ||
            f_.sourceCapturedUs<=admissionUs_ || f_.sourceSequence<=r.baselineSequence || f_.sourceCapturedUs<=r.baselineCapturedUs ||
            f_.sourceCapturedUs<=r.capture.capturedUs) return;
        const auto config=EDM65::RecipeConfig(r.capture,r.capture.scope.session,r.maxAgeUs,r.maxTickGapUs);
        f_.kernelIdentity=r.physicalIdentity;
        f_.kernelIdentity.session=r.capture.scope.session;
        if(!configured_)
        {
            configured_=true;
            if(!core_.Configure(config) || !core_.Start(f_.kernelIdentity))
            { f_.output=core_.Current(); Trip(Reason::Kernel); return; }
        }
        f_.state=State::Running; f_.reason=Reason::None; f_.startAcknowledged=true;
        f_.startedTick=facts.tick; f_.startedUs=facts.nowUs; f_.firstSourceCapturedUs=f_.sourceCapturedUs;
    }
    void Step(const Facts& facts,std::uint64_t delta) noexcept
    {
        if(f_.runtimeSteps>=MaximumRuntimeStepsFor(f_.request.recipeFamily)) { Trip(Reason::CounterOverflow); return; }
        EDM55::Sample sample{}; sample.valid=true; sample.identity=f_.kernelIdentity;
        sample.sequence=f_.sourceSequence; sample.capturedAtUs=f_.sourceCapturedUs;
        sample.voltageV=static_cast<double>(physical_.input.conversion.voltageMv)/1000.0;
        f_.output=core_.Step(sample,facts.nowUs,facts.tick); ++f_.runtimeSteps;
        if(!f_.output.valid || !f_.output.running || f_.output.fault!=EDM55::Fault::None)
        { Trip(f_.output.fault==EDM55::Fault::Stale?Reason::SourceStale:Reason::Kernel); return; }
        const auto& r=f_.request.capture.recipe;
        const auto expected=EDMGapServo::Evaluate(r.curve,f_.output.filteredVoltageV,r.referenceV,r.overridePercent);
        double target=expected.limitedSpeedMmPerMin;
        if(f_.output.shortActive) target=-r.curve.shortRetreatMmPerMin*(r.overridePercent/100.0);
        else if(f_.output.feedInhibited && target>0.0) target=0.0;
        const double error=std::fabs(f_.output.targetMmPerMin-target);
        if(!expected.valid || !std::isfinite(error) || error>1e-9 || !std::isfinite(f_.output.speedMmPerMin))
        { Trip(Reason::TargetMismatch); return; }
        if(error>f_.maximumTargetError) f_.maximumTargetError=error;
        EDM65::ObserveCoverage(f_.coverage,f_.output);
        if(f_.coverage.saturated) { Trip(Reason::CounterOverflow); return; }
        if(f_.runtimeSteps>1 && delta)
        {
            if(!f_.minimumStepUs || delta<f_.minimumStepUs) f_.minimumStepUs=delta;
            if(delta>f_.maximumStepUs) f_.maximumStepUs=delta;
        }
        if(facts.nowUs-f_.startedUs>=DurationUsFor(f_.request.recipeFamily) &&
            f_.sourceCapturedUs-f_.firstSourceCapturedUs>=DurationUsFor(f_.request.recipeFamily))
        {
            if(f_.coverage.freshSamples<2 || !f_.coverage.hasSamples ||
                f_.coverage.validTicks!=f_.runtimeSteps || !f_.maximumStepUs)
            { Trip(Reason::Coverage); return; }
            core_.Cancel(); Neutralize(); f_.state=State::Draining; f_.reason=Reason::None;
            f_.terminalTick=facts.tick; f_.zeroReceipt=false; f_.zeroReceiptTick=0;
        }
    }
};
static_assert(sizeof(Request)<=512U,"EDM69 request must stay bounded.");
static_assert(sizeof(Feedback)<=1024U,"EDM69 feedback must stay bounded.");
static_assert(sizeof(Session)<=12288U,"EDM69 session must stay bounded.");
}
