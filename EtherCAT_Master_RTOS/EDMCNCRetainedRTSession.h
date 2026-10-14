#pragma once
#include "EDMCNCRetainedKernel.h"
#include "EDMGapServoRTSession.h"
#include "EDMExecutionFeedbackChannel.h"
#include "MotionCommandRing.h"
#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM83: one real Motion callback -> at most one numerical Kernel::Step.
// This diagnostic has NO axis/PDO/output writer, allocation, lock or logger.
// modelUs is nominal 250 us sample time, NOT the observed callback clock.
// Callback gaps are independently checked; neither jitter nor WCET is certified.
namespace EDM83RT
{
using Facts=EDM55RT::Facts;
enum class Profile : std::uint8_t { Normal=67U, OperatorWindow=68U };
enum class State : std::uint8_t { Idle, AwaitingStart, Running, Completed, Cancelled, Fault };
enum class Reason : std::uint8_t
{ None, AwaitStart, InvalidStart, Cancelled, RuntimeClock, RuntimeSource,
  AuthorityLost, Heartbeat, Kernel, Coverage, Deadline, CounterOverflow };
constexpr std::uint64_t PeriodUs=250U, HeartbeatMaximumAgeUs=100000U;
constexpr std::uint64_t StartTimeoutUs=1000000U, MaximumCallbackGapUs=2000U;
inline std::uint64_t DurationUs(Profile p) noexcept
{ return p==Profile::OperatorWindow?20000000ULL:12000000ULL; }
inline bool Terminal(State s) noexcept
{ return s==State::Completed || s==State::Cancelled || s==State::Fault; }
inline const char* StateName(State s) noexcept
{
    switch(s) { case State::Idle:return "IDLE"; case State::AwaitingStart:return "AWAIT_START";
    case State::Running:return "RUNNING"; case State::Completed:return "COMPLETED";
    case State::Cancelled:return "CANCELLED"; case State::Fault:return "FAULT"; } return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) { case Reason::None:return "NONE"; case Reason::AwaitStart:return "AWAIT_START";
    case Reason::InvalidStart:return "INVALID_START"; case Reason::Cancelled:return "CANCELLED";
    case Reason::RuntimeClock:return "RT_CLOCK"; case Reason::RuntimeSource:return "RT_SOURCE";
    case Reason::AuthorityLost:return "AUTHORITY"; case Reason::Heartbeat:return "HEARTBEAT";
    case Reason::Kernel:return "KERNEL"; case Reason::Coverage:return "COVERAGE";
    case Reason::Deadline:return "DEADLINE"; case Reason::CounterOverflow:return "COUNTER_OVERFLOW"; }
    return "UNKNOWN";
}
enum Coverage : std::uint64_t
{
    Fresh=1ULL<<0U, GapForward=1ULL<<1U, GapReverse=1ULL<<2U,
    InitialLine=1ULL<<3U, Arc=1ULL<<4U, FinalLine=1ULL<<5U,
    ShortRetreat=1ULL<<6U, ShortReturn=1ULL<<7U,
    JumpRetreat=1ULL<<8U, JumpReturn=1ULL<<9U,
    CUVHeld=1ULL<<10U, Endpoint=1ULL<<11U, CanonicalComplete=1ULL<<12U
};
constexpr std::uint64_t RequiredCoverage=(1ULL<<13U)-1ULL;
struct Request
{
    std::uint64_t session=0U, issuedUs=0U, qpcFrequency=0U;
    EDM81::Scope scope{};
    EDM81::Config config{};
    EDM82::RetainedPath path{};
    Profile profile=Profile::Normal;
};
struct Feedback
{
    EDM81::Scope scope{};
    std::uint64_t session=0U;
    Profile profile=Profile::Normal;
    State state=State::Idle;
    Reason reason=Reason::None;
    EDM81::Reason kernelReason=EDM81::Reason::None;
    std::uint64_t latestSession=0U,cancelThroughSession=0U,publication=0U,rtTick=0U,nowUs=0U;
    std::uint64_t startedUs=0U,startedTick=0U,terminalTick=0U,quiescentTick=0U;
    std::uint64_t runtimeSteps=0U,modelUs=0U,minimumStepUs=0U,maximumStepUs=0U;
    std::uint64_t coverageMask=0U,stageMask=0U,qpcFrequency=0U;
    unsigned stage=0U;
    double voltageV=0.0;
    EDM82::Snapshot sample{};
    bool startAcknowledged=false,quiescent=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
static_assert(std::is_trivially_copyable<Request>::value,"EDM83 request must be a fixed value");
static_assert(sizeof(Feedback)<=2048U,"EDM83 feedback exceeds atomic channel budget");

class Session
{
public:
    Session() noexcept=default;
    Session(const Session&)=delete;
    Session& operator=(const Session&)=delete;
    bool ControlStart(const Request& request) noexcept
    {
        // NC-owned attempt watermark also retires rejected starts. Reusing a
        // failed identity cannot accidentally adopt a queued older request.
        if(!request.session || request.session<=attemptedSession_) return false;
        attemptedSession_=request.session;
        const auto latest=latest_.load(std::memory_order_acquire);
        if(request.session<=latest || request.session<=cancelled_.load(std::memory_order_acquire) ||
            !EDM81::ValidScope(request.scope) || !request.issuedUs || !request.qpcFrequency ||
            request.issuedUs<lastControlHeartbeatUs_ ||
            (request.profile!=Profile::Normal && request.profile!=Profile::OperatorWindow) ||
            !request.path.Valid() || !EDM81::SameScope(request.scope,request.path.FrozenScope()) ||
            request.config.periodUs!=PeriodUs) return false;
        if(latest)
        {
            Feedback previous{};
            if(!ControlRead(previous) || previous.session!=latest || !Terminal(previous.state) ||
                !previous.quiescent || !previous.terminalTick ||
                previous.quiescentTick<=previous.terminalTick) return false;
        }
        if(!requests_.ProducerHasCapacity(1U)) return false;
        Descriptor d{}; d.session=request.session; d.scope=request.scope; d.profile=request.profile;
        d.issuedUs=request.issuedUs; d.frequency=request.qpcFrequency;
        // Metadata first: cancellation between reservation and ring publication
        // can always be attributed to the exact newly reserved scope.
        if(!descriptors_.Publish(d)) return false;
        latest_.store(request.session,std::memory_order_release);
        if(!ControlTouch(request.session,request.issuedUs) || !requests_.ProducerTryPush(request))
        { ControlCancel(request.session); return false; }
        return true;
    }
    bool ControlTouch(std::uint64_t session,std::uint64_t nowUs) noexcept
    {
        if(!session || !nowUs || session!=latest_.load(std::memory_order_acquire) ||
            session<=cancelled_.load(std::memory_order_acquire) || nowUs<lastControlHeartbeatUs_) return false;
        Heartbeat h{}; h.session=session; h.nowUs=nowUs;
        if(!heartbeats_.Publish(h)) { ControlCancel(session); return false; }
        lastControlHeartbeatUs_=nowUs; return true;
    }
    void ControlCancel(std::uint64_t session) noexcept
    {
        // One NC writer. Earlier detached cancellation cannot retire a newer id.
        if(session>cancelled_.load(std::memory_order_relaxed))
            cancelled_.store(session,std::memory_order_release);
    }
    std::uint64_t ControlLatestSession() const noexcept
    { return latest_.load(std::memory_order_acquire); }
    bool ControlRead(Feedback& out) const noexcept
    {
        out=Feedback{};
        const auto latest=latest_.load(std::memory_order_acquire);
        const auto cancel=cancelled_.load(std::memory_order_acquire);
        Feedback candidate{}; const auto stamp=feedback_.Read(candidate);
        if(!stamp.Ready() || latest!=latest_.load(std::memory_order_acquire) ||
            cancel!=cancelled_.load(std::memory_order_acquire) || candidate.session!=latest ||
            candidate.latestSession!=latest || candidate.cancelThroughSession!=cancel) return false;
        candidate.publication=stamp.publication; out=candidate; return true;
    }
    bool NeedsRuntimeService() const noexcept
    {
        const auto latest=latest_.load(std::memory_order_acquire);
        return latest && (latest!=retired_.load(std::memory_order_acquire) ||
            cancelled_.load(std::memory_order_acquire)!=retiredCancel_.load(std::memory_order_acquire));
    }
    void RuntimeCycle(const Facts& facts,std::uint64_t frequency) noexcept
    {
        if(!NeedsRuntimeService()) return;
        if(!AdoptDescriptor(facts)) return;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs;
        const bool clock=facts.clockValid && facts.tick && facts.nowUs && frequency &&
            frequency==descriptor_.frequency && (!haveClock_ ||
                (previousTick_!=(std::numeric_limits<std::uint64_t>::max)() &&
                 facts.tick==previousTick_+1U && facts.nowUs>previousUs_ &&
                 facts.nowUs-previousUs_<=MaximumCallbackGapUs));
        const auto delta=clock && haveClock_?facts.nowUs-previousUs_:0U;
        previousTick_=facts.tick; previousUs_=facts.nowUs; haveClock_=true;
        const bool source=facts.pdoValid && facts.contiguous;
        ObserveCancel();
        // Admission gates permit one unconsumed request at a time. Two slots
        // allow the revoked-before-enqueue race to drain with bounded work.
        for(unsigned n=0U;n<2U && requests_.ConsumerTryPop(incoming_);++n)
            if(incoming_.session==f_.session)
            { request_=incoming_; haveRequest_=true; }
        Heartbeat h{};
        if(heartbeats_.Read(h).Ready() && h.session==f_.session &&
            h.nowUs>=lastRuntimeHeartbeatUs_ && h.nowUs<=facts.nowUs)
            lastRuntimeHeartbeatUs_=h.nowUs;
        ObserveCancel();
        if(f_.state==State::AwaitingStart)
        {
            if(!clock) Trip(Reason::RuntimeClock);
            else if(!source) Trip(Reason::RuntimeSource);
            else if(facts.nowUs<descriptor_.issuedUs)
            {
                if(descriptor_.issuedUs-facts.nowUs>HeartbeatMaximumAgeUs) Trip(Reason::InvalidStart);
            }
            else if(facts.nowUs-descriptor_.issuedUs>StartTimeoutUs) Trip(Reason::Deadline);
            else if(haveRequest_) Start(facts);
        }
        if(f_.state==State::Running || f_.state==State::Completed)
        {
            if(!clock) Trip(Reason::RuntimeClock);
            else if(!source) Trip(Reason::RuntimeSource);
            else if(!Authority(facts)) Trip(Reason::AuthorityLost);
            else if(!HeartbeatReady(facts.nowUs)) Trip(Reason::Heartbeat);
            else if(facts.nowUs<f_.startedUs ||
                facts.nowUs-f_.startedUs>DurationUs(f_.profile)+5000000ULL) Trip(Reason::Deadline);
            else if(f_.state==State::Running)
            {
                if(delta && facts.tick>f_.startedTick)
                {
                    if(!f_.minimumStepUs || delta<f_.minimumStepUs) f_.minimumStepUs=delta;
                    if(delta>f_.maximumStepUs) f_.maximumStepUs=delta;
                }
                Step();
            }
        }
        ObserveCancel();
        if(Terminal(f_.state) && !f_.quiescent && clock && source)
        {
            if(!f_.terminalTick) f_.terminalTick=facts.tick;
            else if(facts.tick>f_.terminalTick)
            { f_.quiescent=true; f_.quiescentTick=facts.tick; f_.sample.committed=false; }
        }
        if(!feedback_.Publish(f_)) { Trip(Reason::CounterOverflow); return; }
        // Completed is deliberately not retired; it stays guarded until NC's
        // explicit cancellation and a separate later quiescent receipt.
        if(f_.state==State::Cancelled && f_.quiescent &&
            f_.latestSession==latest_.load(std::memory_order_acquire) &&
            f_.cancelThroughSession==cancelled_.load(std::memory_order_acquire))
        { retiredCancel_.store(f_.cancelThroughSession,std::memory_order_release);
          retired_.store(f_.session,std::memory_order_release); haveClock_=false; }
    }
private:
    struct Descriptor
    {
        std::uint64_t session=0U,issuedUs=0U,frequency=0U;
        EDM81::Scope scope{}; Profile profile=Profile::Normal;
    };
    struct Heartbeat { std::uint64_t session=0U,nowUs=0U; };
    FixedCapacitySpscRing<Request,2U> requests_{};
    EDM53::Channel<Descriptor> descriptors_{};
    EDM53::Channel<Heartbeat> heartbeats_{};
    EDM53::Channel<Feedback> feedback_{};
    std::atomic<std::uint64_t> latest_{0U},cancelled_{0U},retired_{0U},retiredCancel_{0U};
    std::uint64_t attemptedSession_=0U,lastControlHeartbeatUs_=0U; // NC only.
    EDM82::Kernel kernel_{};
    Request request_{},incoming_{}; // RT-owned scratch avoids clearing a large request each callback.
    Descriptor descriptor_{};
    Feedback f_{};
    std::uint64_t previousTick_=0U,previousUs_=0U,lastRuntimeHeartbeatUs_=0U;
    bool haveClock_=false,haveRequest_=false,jumpIssued_=false;
    bool shortSeen_=false,jumpSeen_=false;
    bool AdoptDescriptor(const Facts& facts) noexcept
    {
        const auto latest=latest_.load(std::memory_order_acquire);
        if(f_.session==latest) return true;
        Descriptor d{};
        if(!descriptors_.Read(d).Ready() || d.session!=latest ||
            latest!=latest_.load(std::memory_order_acquire)) return false;
        kernel_.Cancel(EDM81::Event::Reset);
        descriptor_=d; f_=Feedback{}; f_.scope=d.scope; f_.session=d.session;
        f_.latestSession=d.session; f_.profile=d.profile; f_.qpcFrequency=d.frequency;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs;
        f_.state=State::AwaitingStart; f_.reason=Reason::AwaitStart;
        f_.sample.scope=d.scope; haveRequest_=false; haveClock_=false;
        lastRuntimeHeartbeatUs_=d.issuedUs; jumpIssued_=shortSeen_=jumpSeen_=false;
        return true;
    }
    bool Authority(const Facts& f) const noexcept
    { return f.authorityReady && f.executionEpoch==f_.scope.executionEpoch && f.ownerLease==f_.scope.ownerLease; }
    bool HeartbeatReady(std::uint64_t nowUs) const noexcept
    { return lastRuntimeHeartbeatUs_ && nowUs>=lastRuntimeHeartbeatUs_ && nowUs-lastRuntimeHeartbeatUs_<=HeartbeatMaximumAgeUs; }
    void ObserveCancel() noexcept
    {
        f_.latestSession=latest_.load(std::memory_order_acquire);
        f_.cancelThroughSession=cancelled_.load(std::memory_order_acquire);
        if(f_.session && f_.session<=f_.cancelThroughSession && f_.state!=State::Cancelled)
        {
            kernel_.Cancel(EDM81::Event::Reset);
            f_.state=State::Cancelled; f_.reason=Reason::Cancelled;
            f_.terminalTick=f_.rtTick; f_.quiescent=false; f_.quiescentTick=0U;
            f_.sample.state=EDM81::State::Cancelled; f_.sample.reason=EDM81::Reason::Reset;
            f_.sample.committed=false;
        }
    }
    void Trip(Reason reason) noexcept
    {
        if(f_.state!=State::Fault)
        {
            f_.state=State::Fault; f_.reason=reason; f_.terminalTick=f_.rtTick;
            f_.quiescent=false; f_.quiescentTick=0U;
        }
        kernel_.Cancel(EDM81::Event::Alarm); f_.sample.state=EDM81::State::Fault;
        f_.sample.committed=false;
    }
    void Start(const Facts& facts) noexcept
    {
        if(request_.session!=f_.session || !EDM81::SameScope(request_.scope,f_.scope) ||
            request_.issuedUs!=descriptor_.issuedUs || request_.qpcFrequency!=descriptor_.frequency ||
            request_.profile!=f_.profile || request_.config.periodUs!=PeriodUs)
        { Trip(Reason::InvalidStart); return; }
        if(!Authority(facts)) { Trip(Reason::AuthorityLost); return; }
        if(!HeartbeatReady(facts.nowUs)) { Trip(Reason::Heartbeat); return; }
        kernel_=EDM82::Kernel{}; // New transport identity; NC run may contain several G180 blocks.
        if(!kernel_.Begin(request_.config,request_.scope,request_.path))
        { f_.kernelReason=kernel_.Current().reason; Trip(Reason::Kernel); return; }
        ObserveCancel(); if(f_.state==State::Cancelled) return;
        f_.sample=kernel_.Current(); f_.state=State::Running; f_.reason=Reason::None;
        f_.startedUs=facts.nowUs; f_.startedTick=facts.tick; f_.startAcknowledged=true;
    }
    void Step() noexcept
    {
        ObserveCancel(); if(f_.state!=State::Running) return;
        if(f_.modelUs>=DurationUs(f_.profile)+3000000ULL) { Trip(Reason::Coverage); return; }
        const auto previous=f_.sample;
        EDM81::Input input{}; input.scope=f_.scope;
        input.sequence=f_.runtimeSteps+1U;
        input.nowUs=input.capturedAtUs=f_.modelUs+PeriodUs;
        input.voltageV=request_.config.referenceV+10.0;
        f_.stage=0U;
        if(f_.modelUs>=1000000U && f_.modelUs<1500000U)
        { input.voltageV=request_.config.referenceV-10.0; f_.stage=1U; }
        else if(f_.modelUs>=1500000U && f_.modelUs<3000000U) f_.stage=2U;
        else if(f_.modelUs>=3000000U && f_.modelUs<3500000U)
        { input.voltageV=0.0; f_.stage=3U; }
        else if(f_.modelUs>=3500000U && f_.modelUs<5000000U) f_.stage=4U;
        else if(f_.modelUs>=5000000U && f_.modelUs<6000000U) f_.stage=5U;
        else if(f_.modelUs>=6000000U)
        {
            f_.stage=6U;
            if(!jumpIssued_) { input.event=EDM81::Event::Jump; jumpIssued_=true; }
            else if(previous.state==EDM81::State::Running) f_.stage=7U;
        }
        if(f_.modelUs>=DurationUs(f_.profile) && previous.state==EDM81::State::Running &&
            previous.cursorMm==previous.pathLengthMm)
        {
            if((f_.coverageMask & (RequiredCoverage^CanonicalComplete))!=(RequiredCoverage^CanonicalComplete))
            { Trip(Reason::Coverage); return; }
            input.event=EDM81::Event::M30;
        }
        const auto candidate=kernel_.Step(input); // exactly one per real callback
        ObserveCancel(); if(f_.state!=State::Running) return;
        f_.sample=candidate; ++f_.runtimeSteps; f_.modelUs+=PeriodUs;
        f_.voltageV=input.voltageV; f_.stageMask|=1ULL<<f_.stage;
        if(candidate.state==EDM81::State::Fault || candidate.state==EDM81::State::Cancelled)
        { f_.kernelReason=candidate.reason; Trip(Reason::Kernel); return; }
        if(candidate.committed) f_.coverageMask|=Fresh;
        if(candidate.state==EDM81::State::Running && candidate.cursorMm>previous.cursorMm)
            f_.coverageMask|=GapForward;
        if(f_.stage==1U && candidate.state==EDM81::State::Running && candidate.cursorMm<previous.cursorMm)
            f_.coverageMask|=GapReverse;
        if(candidate.ordinal==1U) f_.coverageMask|=InitialLine;
        if(candidate.ordinal==2U) f_.coverageMask|=Arc;
        if(candidate.ordinal==3U) f_.coverageMask|=FinalLine;
        if(candidate.state==EDM81::State::ShortRetreat && candidate.cursorMm<previous.cursorMm)
        { shortSeen_=true; f_.coverageMask|=ShortRetreat; }
        if(shortSeen_ && !jumpIssued_ && previous.state==EDM81::State::Returning &&
            candidate.state==EDM81::State::Running && candidate.cursorMm==candidate.anchorMm)
            f_.coverageMask|=ShortReturn;
        if(candidate.state==EDM81::State::JumpRetreat && candidate.cursorMm<previous.cursorMm)
        { jumpSeen_=true; f_.coverageMask|=JumpRetreat; }
        if(jumpSeen_ && previous.state==EDM81::State::Returning &&
            candidate.state==EDM81::State::Running && candidate.cursorMm==candidate.anchorMm)
            f_.coverageMask|=JumpReturn;
        bool held=true;
        for(unsigned axis=3U;axis<6U;++axis)
            held=held && candidate.pose[axis]==request_.path.Row(0U)->startMCS[axis];
        if(!held) { Trip(Reason::Coverage); return; }
        f_.coverageMask|=CUVHeld;
        if(candidate.cursorMm==candidate.pathLengthMm) f_.coverageMask|=Endpoint;
        if(candidate.state==EDM81::State::Completed)
        {
            f_.coverageMask|=CanonicalComplete;
            if(f_.coverageMask!=RequiredCoverage) { Trip(Reason::Coverage); return; }
            f_.state=State::Completed; f_.reason=Reason::None; f_.terminalTick=f_.rtTick;
            f_.quiescent=false; f_.quiescentTick=0U;
        }
    }
};
}
