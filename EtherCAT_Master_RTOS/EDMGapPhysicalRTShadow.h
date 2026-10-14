#pragma once

#include "EDMGapPhysicalSource.h"
#include "EDMGapServoRuntime.h"
#include "EDMExecutionFeedbackChannel.h"
#include <cstdint>
#include <limits>

// EDM64: one PDO-owner observer, independent of every machining session.
// No Motion call, permit, output write, allocation, clock read or blocking work.
// Configure once before starting the owner. Observe only from that owner;
// concurrent diagnostics use ReadRaw -> consumer clock -> ValidateRead.
namespace EDM64
{
constexpr bool PhysicalMotion = false;
constexpr bool Discharge = false;
constexpr const char* ConfigurationKind = "FIXED_SHADOW";

enum class State : std::uint8_t { Uninitialized, AwaitProducer, AwaitFresh, Valid, Disabled, Revoked };
enum class Reason : std::uint8_t
{
    None, AwaitProducer, AwaitFresh, Uninitialized, Configuration, Reinitialize,
    Input, Runtime, Clock, CycleExhausted, PublicationClosed
};
enum class ReaderState : std::uint8_t { Unchecked, Fresh, Unobserved, Empty, Busy, Closed, Clock, Stale };
inline const char* StateName(State s) noexcept
{
    switch(s) {
    case State::Uninitialized:return "UNINITIALIZED"; case State::AwaitProducer:return "AWAIT_PRODUCER";
    case State::AwaitFresh:return "AWAIT_FRESH"; case State::Valid:return "VALID";
    case State::Disabled:return "DISABLED"; case State::Revoked:return "REVOKED";
    } return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) {
    case Reason::None:return "NONE"; case Reason::AwaitProducer:return "AWAIT_PRODUCER";
    case Reason::AwaitFresh:return "AWAIT_FRESH"; case Reason::Uninitialized:return "UNINITIALIZED";
    case Reason::Configuration:return "CONFIGURATION"; case Reason::Reinitialize:return "REINITIALIZE";
    case Reason::Input:return "INPUT"; case Reason::Runtime:return "RUNTIME";
    case Reason::Clock:return "CLOCK"; case Reason::CycleExhausted:return "CYCLE_EXHAUSTED";
    case Reason::PublicationClosed:return "PUBLICATION_CLOSED";
    } return "UNKNOWN";
}
inline const char* ReaderStateName(ReaderState s) noexcept
{
    switch(s) {
    case ReaderState::Unchecked:return "UNCHECKED"; case ReaderState::Fresh:return "FRESH";
    case ReaderState::Unobserved:return "UNOBSERVED"; case ReaderState::Empty:return "EMPTY";
    case ReaderState::Busy:return "BUSY"; case ReaderState::Closed:return "CLOSED";
    case ReaderState::Clock:return "CLOCK"; case ReaderState::Stale:return "STALE";
    } return "UNKNOWN";
}
inline const char* FaultName(EDM55::Fault f) noexcept
{
    switch(f) {
    case EDM55::Fault::None:return "NONE"; case EDM55::Fault::Configuration:return "CONFIGURATION";
    case EDM55::Fault::ConfigurationChange:return "CONFIGURATION_CHANGE";
    case EDM55::Fault::Session:return "SESSION"; case EDM55::Fault::Identity:return "IDENTITY";
    case EDM55::Fault::Sample:return "SAMPLE"; case EDM55::Fault::Sequence:return "SEQUENCE";
    case EDM55::Fault::SourceClock:return "SOURCE_CLOCK"; case EDM55::Fault::Clock:return "CLOCK";
    case EDM55::Fault::Stale:return "STALE"; case EDM55::Fault::Arithmetic:return "ARITHMETIC";
    } return "UNKNOWN";
}

// Deliberately diagnostic, not a copy of an active E-code or permission to use
// these gains on a machine. Defaults are the unchanged EDM55 reference curve:
// 60 V, +/-30 mm/min, 2 ms filter, 25/30 V short hysteresis, 2/5 ms short dwell,
// 120/240 mm/min/s ramp, 5 ms reversal dwell. Age <=5 ms; owner gap <=2 ms.
inline EDM55::Config DiagnosticConfig() noexcept
{
    EDM55::Config c{}; c.revision=64ULL; return c;
}
inline bool AdvanceCycle(std::uint64_t& cycle) noexcept
{
    if(cycle==(std::numeric_limits<std::uint64_t>::max)()) return false;
    ++cycle; return true;
}
struct Snapshot
{
    State state=State::Uninitialized;
    Reason reason=Reason::Uninitialized;
    ReaderState readerState=ReaderState::Unchecked;
    EDM63::Receipt input{};
    // Both sourceGeneration and session below name this observer's admission,
    // not a real NC session. Actual producerGeneration remains in input.sample.
    EDM55::Identity identity{};
    EDM55::Output output{};
    std::uint64_t observedUs=0, cycle=0, qpcFrequency=0, maxAgeUs=5000, maxTickGapUs=2000;
    std::uint64_t publication=0, configRevision=64;
    bool physicalPermit=false, discharge=false;
};
inline void InvalidateDerived(Snapshot& s) noexcept
{
    const auto fault=s.output.fault;
    s.output=EDM55::Output{}; s.output.fault=fault; s.output.stopRequired=true;
    s.input.usable=false; s.input.physicalPermit=false;
    s.input.conversion=EDMGapInput::ConversionResult{};
    s.physicalPermit=false; s.discharge=false;
}

class PhysicalRTShadow
{
public:
    bool InitializeBeforeStart(const EDMGapInput::Profile& profile) noexcept
    {
        if(attempted_ || current_.cycle)
        { Fail(Reason::Reinitialize); Publish(); return false; }
        attempted_=true;
        auto config=DiagnosticConfig();
        current_.maxAgeUs=config.maxAgeUs; current_.maxTickGapUs=config.maxTickGapUs;
        current_.configRevision=config.revision;
        const bool inputReady=observer_.Initialize(profile);
        current_.input=observer_.Snapshot();
        if(!inputReady)
        {
            current_.state=State::Disabled; current_.reason=Reason::Configuration;
            InvalidateDerived(current_); Publish(); return false;
        }
        const auto configuredAge=profile.gap.maxAgeMs*1000ULL;
        // The diagnostic curve is fixed. Its safety time limits can only be
        // tightened by the installed physical input age requirement.
        if(configuredAge<config.maxAgeUs) config.maxAgeUs=configuredAge;
        if(config.maxTickGapUs>config.maxAgeUs) config.maxTickGapUs=config.maxAgeUs;
        current_.maxAgeUs=config.maxAgeUs; current_.maxTickGapUs=config.maxTickGapUs;
        if(!runtime_.Configure(config))
        {
            current_.output=runtime_.Current(); current_.state=State::Disabled;
            current_.reason=Reason::Configuration; InvalidateDerived(current_); Publish(); return false;
        }
        current_.state=State::AwaitProducer; current_.reason=Reason::AwaitProducer;
        Publish(); return true;
    }

    // nowUs is the publisher's HOST_PDO_RECEIPT clock conversion, supplied by
    // the owner from its existing QPC result. No poll updates source time.
    void ObserveFromPdo(const EDMAnalogInput::Snapshot& sample,std::uint64_t nowUs,
        bool clockValid,std::uint64_t frequency) noexcept
    {
        if(publicationClosed_) return;
        if(!AdvanceCycle(current_.cycle)) { Fail(Reason::CycleExhausted); Publish(); return; }
        if(current_.state==State::Revoked || current_.state==State::Disabled)
        { Publish(); return; }
        if(!attempted_) { Fail(Reason::Uninitialized); Publish(); return; }
        const bool validClock=clockValid && nowUs && frequency;
        if(validClock)
        {
            if((lastNowUs_ && nowUs<=lastNowUs_) || (frequency_ && frequency!=frequency_))
            { Fail(Reason::Clock); Publish(); return; }
            // Include the admitted baseline in the deadline. A first fresh
            // sample cannot conceal a missing interval before Runtime's seed.
            if(admitted_ && nowUs-lastNowUs_>current_.maxTickGapUs)
            { Fail(Reason::Clock); Publish(); return; }
            lastNowUs_=nowUs; frequency_=frequency;
            current_.observedUs=nowUs; current_.qpcFrequency=frequency;
        }
        else if(admitted_)
        { Fail(Reason::Clock); Publish(); return; }
        current_.input=observer_.Observe(sample,true,nowUs,validClock,frequency);
        if(current_.input.state==EDM63::State::Revoked)
        { Fail(Reason::Input); Publish(); return; }
        if(current_.input.observerGeneration && !admitted_)
        {
            admitted_=true; admittedUs_=current_.input.admittedUs;
            current_.identity.source=EDM55::SourceKind::Physical;
            current_.identity.sourceId=EDM63::PhysicalSourceId(sample.deviceId,sample.channelId);
            current_.identity.sourceGeneration=current_.input.observerGeneration;
            current_.identity.calibrationRevision=current_.input.expectedCalibrationRevision;
            current_.identity.session=current_.input.observerGeneration;
            if(!runtime_.Start(current_.identity))
            { current_.output=runtime_.Current(); Fail(Reason::Runtime); Publish(); return; }
        }
        if(admitted_ && !seeded_ && nowUs-admittedUs_>current_.maxTickGapUs)
        { Fail(Reason::Clock); Publish(); return; }
        if(!current_.input.Usable() && current_.input.state!=EDM63::State::Stale)
        {
            current_.state=admitted_?State::AwaitFresh:State::AwaitProducer;
            current_.reason=admitted_?Reason::AwaitFresh:Reason::AwaitProducer;
            InvalidateDerived(current_); Publish(); return;
        }
        EDM55::Sample input{};
        input.valid=true; input.identity=current_.identity;
        input.sequence=current_.input.sample.sequence;
        input.capturedAtUs=current_.input.sample.capturedAtUs;
        input.voltageV=static_cast<double>(current_.input.conversion.voltageMv)/1000.0;
        current_.output=runtime_.Step(input,nowUs,current_.cycle);
        if(!current_.output.valid)
        { Fail(Reason::Runtime); Publish(); return; }
        // A stricter installed source-age cap must also revoke this observer;
        // stale physical data never revives on the next healthy receipt.
        if(!current_.input.Usable()) { Fail(Reason::Input); Publish(); return; }
        seeded_=true; current_.state=State::Valid; current_.reason=Reason::None; Publish();
    }

    // Bounded concurrent read; callers supply their own nowUs/QPC validity so
    // a stopped PDO owner cannot leave an indefinitely live voltage or speed.
    // Reader errors never reset/rearm or mutate the RT observer.
    // PDO-owner access only: the next observer consumes this exact cycle copy.
    // Cross-thread readers must continue using ReadRaw and ValidateRead.
    const Snapshot& SnapshotFromOwner() const noexcept { return current_; }

    EDM53::Stamp ReadRaw(Snapshot& out) const noexcept
    { return channel_.Read(out); }

    // Capture ReadRaw BEFORE the consumer's clock read, then validate exactly
    // that coherent value. Do not reread the bank after measuring nowUs: a newer
    // PDO publication could legitimately have a later timestamp than nowUs.
    static EDM53::Stamp ValidateRead(Snapshot& out,const EDM53::Stamp& stamp,
        std::uint64_t nowUs,bool clockValid,std::uint64_t frequency) noexcept
    {
        out.publication=stamp.publication;
        if(!stamp.Ready())
        {
            out.readerState=stamp.status==EDM53::Status::Closed?ReaderState::Closed:
                (stamp.status==EDM53::Status::Busy?ReaderState::Busy:ReaderState::Empty);
            InvalidateDerived(out); return stamp;
        }
        if(!out.observedUs)
        { out.readerState=ReaderState::Unobserved; InvalidateDerived(out); return stamp; }
        if(!clockValid || !nowUs || !frequency || frequency!=out.qpcFrequency || nowUs<out.observedUs ||
            (out.input.sample.capturedAtUs && nowUs<out.input.sample.capturedAtUs))
        { out.readerState=ReaderState::Clock; InvalidateDerived(out); return stamp; }
        if(nowUs-out.observedUs>out.maxTickGapUs || (out.input.sample.capturedAtUs &&
            nowUs-out.input.sample.capturedAtUs>out.maxAgeUs))
        { out.readerState=ReaderState::Stale; InvalidateDerived(out); return stamp; }
        out.readerState=ReaderState::Fresh; return stamp;
    }
    // Convenience for single-threaded tests or an already later supplied time.
    // Native concurrent consumers use ReadRaw -> own clock -> ValidateRead.
    EDM53::Stamp ReadAt(Snapshot& out,std::uint64_t nowUs,bool clockValid,
        std::uint64_t frequency) const noexcept
    {
        const auto stamp=ReadRaw(out);
        return ValidateRead(out,stamp,nowUs,clockValid,frequency);
    }
private:
    void Fail(Reason reason) noexcept
    {
        if(current_.state!=State::Revoked)
        { current_.state=State::Revoked; current_.reason=reason; }
        runtime_.Cancel(); InvalidateDerived(current_);
    }
    void Publish() noexcept
    {
        if(!channel_.Publish(current_))
        { publicationClosed_=true; Fail(Reason::PublicationClosed); }
    }
    EDM63::ShadowObserver observer_{};
    EDM55::Runtime runtime_{};
    EDM53::Channel<Snapshot> channel_{};
    Snapshot current_{};
    std::uint64_t lastNowUs_=0,frequency_=0,admittedUs_=0;
    bool attempted_=false,admitted_=false,seeded_=false,publicationClosed_=false;
};
static_assert(sizeof(Snapshot)<=512U,"EDM64 diagnostic snapshot must stay bounded.");
static_assert(sizeof(PhysicalRTShadow)<=4096U,"EDM64 observer must stay bounded.");
}
