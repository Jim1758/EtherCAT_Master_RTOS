#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// EDM88: simulation-only acquisition qualification. The producer timestamp and
// sequence are immutable evidence; observing a held conversion never refreshes
// either. No clock read, I/O, allocation, output permission or borrowed pointer.
namespace EDM88
{
enum class SourceKind : std::uint8_t { None, Simulation, Physical };
struct Identity
{
    SourceKind kind=SourceKind::None;
    std::uint64_t producerGeneration=0U;
    std::uint32_t deviceId=0U,channelId=0U,configRevision=0U,calibrationRevision=0U;
};
inline bool SameIdentity(const Identity& a,const Identity& b) noexcept
{
    return a.kind==b.kind && a.producerGeneration==b.producerGeneration &&
        a.deviceId==b.deviceId && a.channelId==b.channelId &&
        a.configRevision==b.configRevision && a.calibrationRevision==b.calibrationRevision;
}
struct SourceConfig
{
    bool enabled=false;
    Identity identity{};
    // Nominal producer cadence; capture times remain independently monotonic.
    // maxAgeUs, rather than an invented periodic timestamp, bounds held data.
    std::uint64_t samplePeriodUs=1000U,maxAgeUs=1500U;
};
inline bool SameConfig(const SourceConfig& a,const SourceConfig& b) noexcept
{
    return a.enabled==b.enabled && SameIdentity(a.identity,b.identity) &&
        a.samplePeriodUs==b.samplePeriodUs && a.maxAgeUs==b.maxAgeUs;
}
inline bool IsDisabled(const SourceConfig& c) noexcept
{ return SameConfig(c,SourceConfig{}); }
inline bool Valid(const SourceConfig& c) noexcept
{
    if(!c.enabled) return IsDisabled(c);
    return c.identity.kind==SourceKind::Simulation && c.identity.producerGeneration &&
        c.identity.deviceId && c.identity.channelId<=65535U && c.identity.configRevision &&
        c.identity.calibrationRevision && c.samplePeriodUs>=250U && c.samplePeriodUs<=5000U &&
        c.maxAgeUs>=c.samplePeriodUs && c.maxAgeUs<=5000U;
}
// Explicit physical admission only. Legacy Begin/Valid keep their SIM-only
// contract; a real PDO receipt is never relabelled or assigned a synthetic ID.
inline bool ValidPhysical(const SourceConfig& c) noexcept
{
    return c.enabled && c.identity.kind==SourceKind::Physical && c.identity.producerGeneration &&
        c.identity.deviceId && c.identity.deviceId<=65535U && c.identity.channelId &&
        c.identity.channelId<=64U && c.identity.configRevision && c.identity.calibrationRevision &&
        c.samplePeriodUs>=250U && c.samplePeriodUs<=5000U &&
        c.maxAgeUs>=c.samplePeriodUs && c.maxAgeUs<=5000U;
}
inline SourceConfig SimulatedSource(std::uint64_t generation) noexcept
{
    SourceConfig c{}; c.enabled=true; c.identity.kind=SourceKind::Simulation;
    c.identity.producerGeneration=generation; c.identity.deviceId=88U;
    c.identity.channelId=1U; c.identity.configRevision=c.identity.calibrationRevision=1U;
    return c;
}
struct Frame
{
    Identity identity{};
    std::uint64_t sequence=0U,capturedAtUs=0U;
    double voltageV=0.0;
    bool valid=false;
};
inline bool SameFrame(const Frame& a,const Frame& b) noexcept
{
    // Signed zero is part of the immutable payload, as are all identity fields.
    return SameIdentity(a.identity,b.identity) && a.sequence==b.sequence &&
        a.capturedAtUs==b.capturedAtUs && a.voltageV==b.voltageV &&
        std::signbit(a.voltageV)==std::signbit(b.voltageV) && a.valid==b.valid;
}
enum class Reason : std::uint8_t
{ None,Disabled,Configuration,Lifecycle,ObserverClock,Identity,Invalid,NonFinite,
  Sequence,ChangedDuplicate,SourceClock,Stale,CounterOverflow };
inline const char* ReasonName(Reason reason) noexcept
{
    switch(reason)
    {
    case Reason::None:return "NONE"; case Reason::Disabled:return "DISABLED";
    case Reason::Configuration:return "CONFIGURATION"; case Reason::Lifecycle:return "LIFECYCLE";
    case Reason::ObserverClock:return "OBSERVER_CLOCK"; case Reason::Identity:return "IDENTITY";
    case Reason::Invalid:return "INVALID"; case Reason::NonFinite:return "NONFINITE";
    case Reason::Sequence:return "SEQUENCE"; case Reason::ChangedDuplicate:return "CHANGED_DUPLICATE";
    case Reason::SourceClock:return "SOURCE_CLOCK"; case Reason::Stale:return "STALE";
    case Reason::CounterOverflow:return "COUNTER_OVERFLOW";
    }
    return "UNKNOWN";
}
struct Evidence
{
    std::uint64_t freshFrames=0U,heldFrames=0U,rejectedFrames=0U,maximumAgeUs=0U;
    Frame lastFrame{};
    Reason reason=Reason::Disabled;
    bool faulted=false;
};
inline bool SameEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return a.freshFrames==b.freshFrames && a.heldFrames==b.heldFrames &&
        a.rejectedFrames==b.rejectedFrames && a.maximumAgeUs==b.maximumAgeUs &&
        SameFrame(a.lastFrame,b.lastFrame) && a.reason==b.reason && a.faulted==b.faulted;
}
class Qualifier
{
public:
    bool Begin(const SourceConfig& config) noexcept
    {
        evidence_=Evidence{}; active_=false; physical_=false; lastObserverUs_=0U;
        config_=SourceConfig{};
        if(IsDisabled(config)) return true;
        const auto generation=config.identity.producerGeneration;
        if(!generation || generation<=generationHighWater_) return Fail(Reason::Lifecycle);
        // An attempted generation cannot be reused after a subsequent failure.
        generationHighWater_=generation;
        if(!config.enabled || !Valid(config)) return Fail(Reason::Configuration);
        config_=config; evidence_.reason=Reason::None; active_=true; return true;
    }
    bool BeginPhysical(const SourceConfig& config,std::uint64_t session) noexcept
    {
        const bool wasActive=active_;
        evidence_=Evidence{}; active_=false; physical_=false; lastObserverUs_=0U; config_=SourceConfig{};
        if(!session || session<=generationHighWater_) return Fail(Reason::Lifecycle);
        generationHighWater_=session;
        if(wasActive || !ValidPhysical(config)) return Fail(Reason::Configuration);
        config_=config; evidence_.reason=Reason::None; active_=physical_=true; return true;
    }
    void Cancel() noexcept { active_=false; }
    bool Observe(const Frame& frame,std::uint64_t sourceNowUs) noexcept
    {
        if(evidence_.faulted) return false;
        if(!active_) return Fail(Reason::Lifecycle);
        if(!sourceNowUs || (lastObserverUs_ && sourceNowUs<=lastObserverUs_))
            return Fail(Reason::ObserverClock);
        lastObserverUs_=sourceNowUs;
        // Check the OLD accepted conversion before considering the new one.
        // A late fresh conversion cannot erase a stale acquisition gap.
        if(evidence_.freshFrames && !AcceptAge(evidence_.lastFrame.capturedAtUs,sourceNowUs))
            return false;
        if(!SameIdentity(frame.identity,config_.identity)) return Fail(Reason::Identity);
        if(!frame.valid) return Fail(Reason::Invalid);
        if(!std::isfinite(frame.voltageV)) return Fail(Reason::NonFinite);
        if(frame.voltageV<0.0 || frame.voltageV>300.0) return Fail(Reason::Invalid);
        if(!frame.sequence) return Fail(Reason::Sequence);
        if(evidence_.freshFrames && frame.sequence==evidence_.lastFrame.sequence)
        {
            if(!SameFrame(frame,evidence_.lastFrame)) return Fail(Reason::ChangedDuplicate);
            if(evidence_.heldFrames==(std::numeric_limits<std::uint64_t>::max)())
                return Fail(Reason::CounterOverflow);
            ++evidence_.heldFrames; return true;
        }
        const auto previous=evidence_.lastFrame.sequence;
        if((!physical_ && !evidence_.freshFrames && frame.sequence!=1U) ||
            (evidence_.freshFrames && (previous==(std::numeric_limits<std::uint64_t>::max)() ||
                (physical_ ? frame.sequence<=previous : frame.sequence!=previous+1U)))) return Fail(Reason::Sequence);
        if(evidence_.freshFrames && frame.capturedAtUs<=evidence_.lastFrame.capturedAtUs)
            return Fail(Reason::SourceClock);
        if(!AcceptAge(frame.capturedAtUs,sourceNowUs)) return false;
        if(evidence_.freshFrames==(std::numeric_limits<std::uint64_t>::max)())
            return Fail(Reason::CounterOverflow);
        evidence_.lastFrame=frame; ++evidence_.freshFrames; return true;
    }
    const SourceConfig& FrozenConfig() const noexcept { return config_; }
    const Evidence& Current() const noexcept { return evidence_; }
private:
    bool AcceptAge(std::uint64_t capturedAtUs,std::uint64_t nowUs) noexcept
    {
        if(!capturedAtUs || capturedAtUs>nowUs) return Fail(Reason::SourceClock);
        const auto age=nowUs-capturedAtUs;
        if(age>evidence_.maximumAgeUs) evidence_.maximumAgeUs=age;
        if(age>config_.maxAgeUs) return Fail(Reason::Stale);
        return true;
    }
    bool Fail(Reason reason) noexcept
    {
        if(!evidence_.faulted)
        {
            evidence_.faulted=true; evidence_.reason=reason;
            if(evidence_.rejectedFrames!=(std::numeric_limits<std::uint64_t>::max)())
                ++evidence_.rejectedFrames;
        }
        active_=false; return false;
    }
    SourceConfig config_{};
    Evidence evidence_{};
    std::uint64_t lastObserverUs_=0U,generationHighWater_=0U;
    bool active_=false,physical_=false;
};
static_assert(std::is_trivially_copyable<SourceConfig>::value,"EDM88 source is a frozen value");
static_assert(std::is_trivially_copyable<Frame>::value,"EDM88 frame is an immutable copied value");
static_assert(sizeof(Qualifier)<=256U,"EDM88 bounded qualifier storage exceeded");
}
