#pragma once
#include "EDMCoordinatedGapSource.h"
#include "EDMGapServo.h"

// EDM89: source-time short decision only. Re-reading an accepted conversion
// holds its decision; it never advances debounce time. No observer clock,
// allocation, I/O, axis command or physical permit is part of this value type.
namespace EDM89
{
struct Mark
{
    std::uint64_t sequence=0U,capturedAtUs=0U;
};
inline bool SameMark(const Mark& a,const Mark& b) noexcept
{ return a.sequence==b.sequence && a.capturedAtUs==b.capturedAtUs; }
inline bool EmptyMark(const Mark& value) noexcept
{ return !value.sequence && !value.capturedAtUs; }
struct Config
{
    std::uint32_t revision=0U;
    bool enabled=false;
    double thresholdV=0.0,hysteresisV=0.0;
    std::uint64_t enterUs=0U,exitUs=0U;
};
inline Config FrozenConfig(const EDMGapServo::Profile& profile) noexcept
{
    Config result{}; result.revision=profile.revision;
    result.enabled=profile.shortMachiningEnabled;
    result.thresholdV=profile.shortMachiningV; result.hysteresisV=profile.shortHysteresisV;
    // Promote BEFORE multiplying; the full uint32 millisecond domain fits uint64.
    result.enterUs=static_cast<std::uint64_t>(profile.shortEnterMs)*1000ULL;
    result.exitUs=static_cast<std::uint64_t>(profile.shortExitMs)*1000ULL;
    return result;
}
inline bool SameConfig(const Config& a,const Config& b) noexcept
{
    return a.revision==b.revision && a.enabled==b.enabled &&
        a.thresholdV==b.thresholdV && std::signbit(a.thresholdV)==std::signbit(b.thresholdV) &&
        a.hysteresisV==b.hysteresisV && std::signbit(a.hysteresisV)==std::signbit(b.hysteresisV) &&
        a.enterUs==b.enterUs && a.exitUs==b.exitUs;
}
enum class Reason : std::uint8_t
{ None,Disabled,Configuration,Lifecycle,Identity,Invalid,NonFinite,Sequence,
  ChangedDuplicate,SourceClock,CounterOverflow };
inline const char* ReasonName(Reason reason) noexcept
{
    switch(reason)
    {
    case Reason::None:return "NONE"; case Reason::Disabled:return "DISABLED";
    case Reason::Configuration:return "CONFIGURATION"; case Reason::Lifecycle:return "LIFECYCLE";
    case Reason::Identity:return "IDENTITY"; case Reason::Invalid:return "INVALID";
    case Reason::NonFinite:return "NONFINITE"; case Reason::Sequence:return "SEQUENCE";
    case Reason::ChangedDuplicate:return "CHANGED_DUPLICATE"; case Reason::SourceClock:return "SOURCE_CLOCK";
    case Reason::CounterOverflow:return "COUNTER_OVERFLOW";
    }
    return "UNKNOWN";
}
inline const char* StateName(EDMGapServo::ShortState state) noexcept
{
    switch(state)
    {
    case EDMGapServo::ShortState::Invalid:return "INVALID";
    case EDMGapServo::ShortState::Clear:return "CLEAR";
    case EDMGapServo::ShortState::Entering:return "ENTERING";
    case EDMGapServo::ShortState::Active:return "ACTIVE";
    case EDMGapServo::ShortState::Exiting:return "EXITING";
    }
    return "UNKNOWN";
}
struct Evidence
{
    Config config{};
    std::uint64_t processedFrames=0U,heldReads=0U,rejectedFrames=0U;
    std::uint64_t entryAborts=0U,exitRestarts=0U,enterCount=0U,exitCount=0U;
    // pending is the current attempt. Completed start/confirm pairs change only
    // on a confirmed transition, so later aborted attempts cannot erase proof.
    Mark pending{},enterStart{},enterConfirm{},exitStart{},exitConfirm{};
    EDMGapServo::ShortState state=EDMGapServo::ShortState::Invalid;
    bool feedInhibited=true,shortActive=false,faulted=false;
    Reason reason=Reason::Disabled;
    EDM88::Frame lastFrame{};
};
inline bool SameEvidence(const Evidence& a,const Evidence& b) noexcept
{
    return SameConfig(a.config,b.config) && a.processedFrames==b.processedFrames &&
        a.heldReads==b.heldReads && a.rejectedFrames==b.rejectedFrames &&
        a.entryAborts==b.entryAborts && a.exitRestarts==b.exitRestarts &&
        a.enterCount==b.enterCount && a.exitCount==b.exitCount &&
        SameMark(a.pending,b.pending) && SameMark(a.enterStart,b.enterStart) &&
        SameMark(a.enterConfirm,b.enterConfirm) && SameMark(a.exitStart,b.exitStart) &&
        SameMark(a.exitConfirm,b.exitConfirm) && a.state==b.state &&
        a.feedInhibited==b.feedInhibited && a.shortActive==b.shortActive &&
        a.faulted==b.faulted && a.reason==b.reason && EDM88::SameFrame(a.lastFrame,b.lastFrame);
}
inline bool ValidFrameIdentity(const EDM88::Identity& identity) noexcept
{
    return identity.kind==EDM88::SourceKind::Simulation && identity.producerGeneration &&
        identity.deviceId && identity.channelId<=65535U && identity.configRevision && identity.calibrationRevision;
}
inline bool ValidPair(const Mark& start,const Mark& confirm,std::uint64_t count,
    std::uint64_t dwellUs,const EDM88::Frame& last) noexcept
{
    if(!count) return EmptyMark(start) && EmptyMark(confirm);
    if(!start.sequence || !start.capturedAtUs || !confirm.sequence || !confirm.capturedAtUs ||
        start.sequence>confirm.sequence || start.capturedAtUs>confirm.capturedAtUs ||
        confirm.sequence>last.sequence || confirm.capturedAtUs>last.capturedAtUs ||
        confirm.capturedAtUs-start.capturedAtUs<dwellUs) return false;
    return (start.sequence==confirm.sequence)==(start.capturedAtUs==confirm.capturedAtUs);
}
// Structural completed-proof gate. The scripted RT fixture additionally checks
// its exact expected transition/probe counts and marks; equality guards receipt
// copies and retirement. This helper does not replay arbitrary source history.
inline bool ValidCompletedEvidence(const EDMGapServo::Profile& profile,const Evidence& evidence,
    const EDM88::Evidence& source) noexcept
{
    const auto& e=evidence;
    if(!EDMGapServo::ValidateProfile(profile) || !SameConfig(e.config,FrozenConfig(profile)) ||
        e.faulted || e.reason!=Reason::None || e.rejectedFrames ||
        e.state!=EDMGapServo::ShortState::Clear || e.feedInhibited || e.shortActive ||
        !EmptyMark(e.pending) || !e.processedFrames || source.faulted ||
        source.reason!=EDM88::Reason::None || source.rejectedFrames ||
        e.processedFrames!=source.freshFrames || e.heldReads!=source.heldFrames ||
        !EDM88::SameFrame(e.lastFrame,source.lastFrame) ||
        !ValidFrameIdentity(e.lastFrame.identity) || !e.lastFrame.valid ||
        !e.lastFrame.capturedAtUs || e.lastFrame.sequence!=e.processedFrames ||
        !std::isfinite(e.lastFrame.voltageV) || e.lastFrame.voltageV<0.0 || e.lastFrame.voltageV>300.0 ||
        e.enterCount!=e.exitCount || e.enterCount>e.processedFrames ||
        e.entryAborts>e.processedFrames || e.exitRestarts>e.processedFrames ||
        !ValidPair(e.enterStart,e.enterConfirm,e.enterCount,e.config.enterUs,e.lastFrame) ||
        !ValidPair(e.exitStart,e.exitConfirm,e.exitCount,e.config.exitUs,e.lastFrame)) return false;
    if(!e.config.enabled)
        return !e.enterCount && !e.exitCount && !e.entryAborts && !e.exitRestarts;
    if(e.lastFrame.voltageV<=e.config.thresholdV) return false;
    if(e.enterCount && (e.enterConfirm.sequence>=e.exitStart.sequence ||
        e.enterConfirm.capturedAtUs>=e.exitStart.capturedAtUs)) return false;
    return true;
}
class Decision
{
public:
    bool Begin(const EDMGapServo::Profile& profile) noexcept
    {
        evidence_=Evidence{}; active_=false;
        if(!EDMGapServo::ValidateProfile(profile)) return Fail(Reason::Configuration);
        evidence_.config=FrozenConfig(profile); evidence_.state=EDMGapServo::ShortState::Clear;
        evidence_.feedInhibited=false; evidence_.reason=Reason::None; active_=true; return true;
    }
    const Evidence& Current() const noexcept { return evidence_; }
    EDMGapServo::ShortResult Result() const noexcept
    {
        EDMGapServo::ShortResult result{};
        result.valid=active_ && !evidence_.faulted && evidence_.processedFrames!=0U;
        result.error=result.valid?EDMGapServo::Error::None:EDMGapServo::Error::Sample;
        result.enabled=evidence_.config.enabled; result.thresholdV=evidence_.config.thresholdV;
        result.state=evidence_.state; result.feedInhibited=evidence_.feedInhibited;
        result.shortActive=evidence_.shortActive;
        return result;
    }
    bool Observe(const EDM88::Frame& frame) noexcept
    {
        if(evidence_.faulted) return false;
        if(!active_) return Fail(Reason::Lifecycle);
        if(!ValidFrameIdentity(frame.identity) ||
            (evidence_.processedFrames && !EDM88::SameIdentity(frame.identity,evidence_.lastFrame.identity)))
            return Fail(Reason::Identity);
        if(!frame.valid) return Fail(Reason::Invalid);
        if(!std::isfinite(frame.voltageV)) return Fail(Reason::NonFinite);
        if(frame.voltageV<0.0 || frame.voltageV>300.0) return Fail(Reason::Invalid);
        if(!frame.sequence) return Fail(Reason::Sequence);
        if(evidence_.processedFrames && frame.sequence==evidence_.lastFrame.sequence)
        {
            if(!EDM88::SameFrame(frame,evidence_.lastFrame)) return Fail(Reason::ChangedDuplicate);
            if(!Increment(evidence_.heldReads)) return false;
            return true;
        }
        const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
        if((!evidence_.processedFrames && frame.sequence!=1U) ||
            (evidence_.processedFrames && (evidence_.lastFrame.sequence==maximum ||
                frame.sequence!=evidence_.lastFrame.sequence+1U))) return Fail(Reason::Sequence);
        if(!frame.capturedAtUs || (evidence_.processedFrames &&
            frame.capturedAtUs<=evidence_.lastFrame.capturedAtUs)) return Fail(Reason::SourceClock);
        // Every transition counter is bounded by processedFrames. Preflight all
        // potential increments before mutating the accepted decision or marks.
        if(evidence_.processedFrames==maximum || evidence_.entryAborts==maximum ||
            evidence_.exitRestarts==maximum || evidence_.enterCount==maximum ||
            evidence_.exitCount==maximum) return Fail(Reason::CounterOverflow);
        const Mark mark{frame.sequence,frame.capturedAtUs};
        const auto& c=evidence_.config;
        using EDMGapServo::ShortState;
        if(!c.enabled)
        { evidence_.state=ShortState::Clear; evidence_.pending=Mark{}; }
        else if(evidence_.state==ShortState::Clear || evidence_.state==ShortState::Entering)
        {
            if(frame.voltageV<=c.thresholdV)
            {
                if(evidence_.state!=ShortState::Entering) evidence_.pending=mark;
                evidence_.state=ShortState::Entering;
                if(frame.capturedAtUs-evidence_.pending.capturedAtUs>=c.enterUs)
                {
                    evidence_.enterStart=evidence_.pending; evidence_.enterConfirm=mark;
                    ++evidence_.enterCount; evidence_.pending=Mark{}; evidence_.state=ShortState::Active;
                }
            }
            else
            {
                if(evidence_.state==ShortState::Entering) ++evidence_.entryAborts;
                evidence_.pending=Mark{}; evidence_.state=ShortState::Clear;
            }
        }
        else
        {
            if(frame.voltageV>c.thresholdV+c.hysteresisV)
            {
                if(evidence_.state!=ShortState::Exiting) evidence_.pending=mark;
                evidence_.state=ShortState::Exiting;
                if(frame.capturedAtUs-evidence_.pending.capturedAtUs>=c.exitUs)
                {
                    evidence_.exitStart=evidence_.pending; evidence_.exitConfirm=mark;
                    ++evidence_.exitCount; evidence_.pending=Mark{}; evidence_.state=ShortState::Clear;
                }
            }
            else
            {
                if(evidence_.state==ShortState::Exiting) ++evidence_.exitRestarts;
                evidence_.pending=Mark{}; evidence_.state=ShortState::Active;
            }
        }
        evidence_.feedInhibited=c.enabled && evidence_.state!=ShortState::Clear;
        evidence_.shortActive=c.enabled && (evidence_.state==ShortState::Active || evidence_.state==ShortState::Exiting);
        evidence_.lastFrame=frame; ++evidence_.processedFrames; return true;
    }
private:
    bool Increment(std::uint64_t& counter) noexcept
    {
        if(counter==(std::numeric_limits<std::uint64_t>::max)()) return Fail(Reason::CounterOverflow);
        ++counter; return true;
    }
    bool Fail(Reason reason) noexcept
    {
        if(!evidence_.faulted)
        {
            evidence_.faulted=true; evidence_.reason=reason;
            if(evidence_.rejectedFrames!=(std::numeric_limits<std::uint64_t>::max)()) ++evidence_.rejectedFrames;
        }
        // Preserve the last accepted state, marks and frame for diagnosis.
        active_=false; return false;
    }
    Evidence evidence_{};
    bool active_=false;
};
static_assert(std::is_trivially_copyable<Evidence>::value,"EDM89 evidence must remain a copied value");
static_assert(sizeof(Decision)<=384U,"EDM89 bounded decision storage exceeded");
}
