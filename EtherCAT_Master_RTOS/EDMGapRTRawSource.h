#pragma once
#include "EDMGapRTSource.h"
#include "EDMGapInput.h"
#include <cmath>
#include <cstdint>
#include <limits>

// EDM62: raw capture/conversion only. One RT owner, no clocks, locks, I/O or
// allocation. This boundary is not motion authorization. Physical frames keep
// Physical classification and remain rejected by the EDM61 motion consumer.
namespace EDM62
{
enum class Reason : std::uint8_t
{ AwaitBegin, None, Configuration, Generation, Identity, ObserverClock,
  SourceClock, Sequence, Invalid, RawRange, Conversion, Revoked };
struct Descriptor
{
    EDM61::Binding binding{};
    EDMGapInput::Profile calibration{};
    std::uint32_t schemaRevision=1, deviceId=0, channelId=0;
};
struct RawFrame
{
    EDM61::Binding binding{};
    std::uint32_t schemaRevision=1, profileRevision=0, calibrationRevision=0;
    std::uint32_t deviceId=0, channelId=0;
    std::uint64_t sequence=0, capturedAtUs=0;
    std::int32_t rawCode=0;
    EDMGap::Source source=EDMGap::Source::NONE;
    bool live=false, valid=false, transportValid=false;
};
inline bool SameCalibration(const EDMGapInput::Profile&a,const EDMGapInput::Profile&b) noexcept
{
    return a.schemaVersion==b.schemaVersion && a.profileRevision==b.profileRevision &&
        a.source==b.source && a.adIndex==b.adIndex && a.expectedDeviceId==b.expectedDeviceId &&
        a.calibrationRevision==b.calibrationRevision && a.calibrationConfirmed==b.calibrationConfirmed &&
        a.rawMin==b.rawMin && a.rawMax==b.rawMax && a.boardMvAtMin==b.boardMvAtMin && a.boardMvAtMax==b.boardMvAtMax &&
        a.calibrationRawMin==b.calibrationRawMin && a.calibrationRawMax==b.calibrationRawMax &&
        a.zeroClampBoardUv==b.zeroClampBoardUv && a.voltageGainMillionths==b.voltageGainMillionths &&
        a.voltageOffsetMv==b.voltageOffsetMv && a.simulationMv==b.simulationMv &&
        a.gap.minMv==b.gap.minMv && a.gap.maxMv==b.gap.maxMv && a.gap.lowEnterMv==b.gap.lowEnterMv &&
        a.gap.lowExitMv==b.gap.lowExitMv && a.gap.highExitMv==b.gap.highExitMv && a.gap.highEnterMv==b.gap.highEnterMv &&
        a.gap.maxAgeMs==b.gap.maxAgeMs && a.gap.dwellMs==b.gap.dwellMs;
}
inline bool Valid(const Descriptor&d) noexcept
{
    const auto&b=d.binding; const auto&p=d.calibration;
    // Raw conversion accepts either declared source class. Do not coerce it to
    // SIM in order to reuse the stricter motion admission check.
    return d.schemaRevision==1U && d.deviceId && d.channelId==p.adIndex &&
        b.schemaRevision==1U && b.clockDomain==1U && b.profile>=35U && b.profile<=52U &&
        b.session && b.run && b.cache && b.dispatch && b.map && b.config && b.epoch &&
        b.ownerGeneration && b.owner==1U && b.servoGeneration && b.admittedTick && b.admittedUs &&
        EDM55::ValidIdentity(b.identity) && b.identity.sourceGeneration==b.servoGeneration &&
        b.identity.session==b.servoGeneration && b.identity.calibrationRevision==p.calibrationRevision &&
        ((b.identity.source==EDM55::SourceKind::Simulation && p.source==EDMGap::Source::SIMULATED) ||
         (b.identity.source==EDM55::SourceKind::Physical && p.source==EDMGap::Source::PHYSICAL)) &&
        (!p.expectedDeviceId || p.expectedDeviceId==d.deviceId) &&
        p.calibrationConfirmed && EDMGapInput::Validate(p);
}
inline bool Matches(const RawFrame&r,const Descriptor&d) noexcept
{
    return EDM61::Same(r.binding,d.binding) && r.schemaRevision==d.schemaRevision &&
        r.profileRevision==d.calibration.profileRevision && r.calibrationRevision==d.calibration.calibrationRevision &&
        r.deviceId==d.deviceId && r.channelId==d.channelId && r.source==d.calibration.source;
}
inline bool Same(const RawFrame&a,const RawFrame&b) noexcept
{
    return EDM61::Same(a.binding,b.binding) && a.schemaRevision==b.schemaRevision &&
        a.profileRevision==b.profileRevision && a.calibrationRevision==b.calibrationRevision &&
        a.deviceId==b.deviceId && a.channelId==b.channelId && a.source==b.source &&
        a.sequence==b.sequence && a.capturedAtUs==b.capturedAtUs && a.rawCode==b.rawCode &&
        a.live==b.live && a.valid==b.valid && a.transportValid==b.transportValid;
}
inline RawFrame InitialFrame(const Descriptor&d) noexcept
{
    RawFrame r{}; r.binding=d.binding; r.schemaRevision=d.schemaRevision;
    r.profileRevision=d.calibration.profileRevision; r.calibrationRevision=d.calibration.calibrationRevision;
    r.deviceId=d.deviceId; r.channelId=d.channelId; r.source=d.calibration.source; r.live=true; return r;
}
class RawIngress
{
public:
    bool Begin(const Descriptor&d) noexcept
    {
        const bool wasLive=frame_.live; frame_.live=false;
        const auto generation=d.binding.servoGeneration;
        if(!generation || generation<=generationHighWater_) { Fail(Reason::Generation); return false; }
        generationHighWater_=generation;
        if(wasLive || !Valid(d)) { Fail(Reason::Configuration); return false; }
        descriptor_=d; frame_=EDM61::Frame{}; frame_.binding=d.binding;
        frame_.sample.identity=d.binding.identity; frame_.live=true;
        lastRaw_=RawFrame{}; haveRaw_=false; lastTick_=lastUs_=0; reason_=Reason::None; return true;
    }
    void Cancel() noexcept { Fail(Reason::Revoked); }
    const EDM61::Frame& Snapshot() const noexcept { return frame_; }
    const Descriptor& FrozenDescriptor() const noexcept { return descriptor_; }
    Reason CurrentReason() const noexcept { return reason_; }
    std::uint64_t GenerationHighWater() const noexcept { return generationHighWater_; }
    EDM61::Frame Publish(const RawFrame&r,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!frame_.live) return frame_;
        if(!tick || !nowUs || tick<descriptor_.binding.admittedTick || nowUs<descriptor_.binding.admittedUs ||
            (lastTick_ && (tick<lastTick_ || nowUs<lastUs_ || ((tick==lastTick_)!=(nowUs==lastUs_)))))
            return Fail(Reason::ObserverClock);
        lastTick_=tick; lastUs_=nowUs;
        if(!Matches(r,descriptor_)) return Fail(Reason::Identity);
        if(!r.live || !r.valid || !r.transportValid) return Fail(Reason::Invalid);
        const bool simulated=r.source==EDMGap::Source::SIMULATED;
        // The fixture encoder is contiguous and starts at one. Physical raw
        // acquisition may retain its publisher sequence across admissions and
        // skip unread snapshots; never rebase that evidence to a fake one.
        if(!r.sequence || (!haveRaw_ && simulated && r.sequence!=1ULL) ||
            (haveRaw_ && r.sequence<lastRaw_.sequence)) return Fail(Reason::Sequence);
        if(r.capturedAtUs<descriptor_.binding.admittedUs || r.capturedAtUs>nowUs)
            return Fail(Reason::SourceClock);
        if(haveRaw_ && r.sequence==lastRaw_.sequence)
        {
            if(!Same(r,lastRaw_)) return Fail(Reason::Sequence);
            return frame_;
        }
        if(haveRaw_ && (lastRaw_.sequence==(std::numeric_limits<std::uint64_t>::max)() ||
            (simulated && r.sequence!=lastRaw_.sequence+1ULL))) return Fail(Reason::Sequence);
        if(haveRaw_ && r.capturedAtUs<=lastRaw_.capturedAtUs) return Fail(Reason::SourceClock);
        const auto converted=EDMGapInput::ConvertRawValidated(descriptor_.calibration,r.rawCode);
        if(converted.inputStatus==EDMGapInput::InputStatus::RawOutOfRange) return Fail(Reason::RawRange);
        if(converted.inputStatus!=EDMGapInput::InputStatus::Valid || !converted.boardVoltageValid ||
            !converted.gapInRange) return Fail(Reason::Conversion);
        // No poll timestamp or observer sequence becomes acquisition evidence.
        // The EDM55 consumer owns its canonical 5 ms maxAge and late-arrival
        // checks. Duplicate/frozen conversions retain the original capture.
        frame_.sample.identity=r.binding.identity; frame_.sample.valid=true;
        frame_.sample.sequence=r.sequence; frame_.sample.capturedAtUs=r.capturedAtUs;
        frame_.sample.voltageV=static_cast<double>(converted.voltageMv)/1000.0;
        lastRaw_=r; haveRaw_=true; return frame_;
    }
private:
    EDM61::Frame Fail(Reason reason) noexcept
    {
        if(reason_==Reason::None || reason_==Reason::AwaitBegin) reason_=reason;
        frame_.live=false; return frame_;
    }
    Descriptor descriptor_{}; RawFrame lastRaw_{}; EDM61::Frame frame_{};
    std::uint64_t generationHighWater_=0, lastTick_=0, lastUs_=0;
    Reason reason_=Reason::AwaitBegin; bool haveRaw_=false;
};
inline EDMGapInput::Profile SyntheticProfile() noexcept
{
    EDMGapInput::Profile p{}; p.schemaVersion=1; p.profileRevision=1;
    p.source=EDMGap::Source::SIMULATED; p.adIndex=1; p.expectedDeviceId=62;
    p.calibrationRevision=1; p.calibrationConfirmed=true;
    p.rawMin=0; p.rawMax=30000; p.boardMvAtMin=0; p.boardMvAtMax=300000;
    p.voltageGainMillionths=EDMGapInput::GainScale; p.gap.maxMv=300000; return p;
}
inline Descriptor SyntheticDescriptor(const EDM61::Binding&binding) noexcept
{
    Descriptor d{}; d.binding=binding; d.calibration=SyntheticProfile();
    d.deviceId=62; d.channelId=1; return d;
}
class RawSimEncoder
{
public:
    bool Begin(const Descriptor&d) noexcept
    {
        const bool wasLive=raw_.live; raw_.live=false;
        if(!d.binding.servoGeneration || d.binding.servoGeneration<=generationHighWater_) return false;
        generationHighWater_=d.binding.servoGeneration;
        if(wasLive || !Valid(d) || !EDM61::Valid(d.binding) || d.deviceId!=62U || d.channelId!=1U ||
            !SameCalibration(d.calibration,SyntheticProfile())) return false;
        raw_=InitialFrame(d); lastTick_=lastUs_=0; return true;
    }
    void Cancel() noexcept { raw_.live=false; }
    const RawFrame& Snapshot() const noexcept { return raw_; }
    RawFrame Capture(const EDM61::Request&r) noexcept
    {
        if(!raw_.live) return raw_;
        if(!r.tick || !r.nowUs || r.tick<raw_.binding.admittedTick || r.nowUs<raw_.binding.admittedUs ||
            !std::isfinite(r.voltageV) || r.voltageV<0.0 || r.voltageV>300.0 ||
            (lastTick_ && (r.tick<lastTick_ || r.nowUs<lastUs_ || ((r.tick==lastTick_)!=(r.nowUs==lastUs_)))))
        { Cancel(); return raw_; }
        lastTick_=r.tick; lastUs_=r.nowUs;
        const double code=std::round(r.voltageV*100.0);
        if(code<0.0 || code>30000.0 || code/100.0!=r.voltageV) { Cancel(); return raw_; }
        if(!r.freezeCapture && (!raw_.sequence || r.nowUs-raw_.capturedAtUs>=1000ULL))
        {
            if(raw_.sequence==(std::numeric_limits<std::uint64_t>::max)()) { Cancel(); return raw_; }
            raw_.rawCode=static_cast<std::int32_t>(code); ++raw_.sequence;
            raw_.capturedAtUs=r.nowUs; raw_.valid=raw_.transportValid=true;
        }
        return raw_;
    }
private:
    RawFrame raw_{};
    std::uint64_t generationHighWater_=0, lastTick_=0, lastUs_=0;
};
class SimRawSource
{
public:
    bool Begin(const EDM61::Binding&binding) noexcept
    {
        const auto d=SyntheticDescriptor(binding);
        const bool ingressReady=ingress_.Begin(d);
        const bool encoderReady=encoder_.Begin(d);
        if(!ingressReady || !encoderReady) { Cancel(); return false; }
        return true;
    }
    void Cancel() noexcept { encoder_.Cancel(); ingress_.Cancel(); }
    const EDM61::Frame& Snapshot() const noexcept { return ingress_.Snapshot(); }
    std::uint64_t GenerationHighWater() const noexcept { return ingress_.GenerationHighWater(); }
    RawFrame CaptureRaw(const EDM61::Request&r) noexcept { return encoder_.Capture(r); }
    EDM61::Frame PublishRaw(const RawFrame&raw,const EDM61::Request&r) noexcept
    { return ingress_.Publish(raw,r.tick,r.nowUs); }
    EDM61::Frame Capture(const EDM61::Request&r) noexcept
    { return PublishRaw(CaptureRaw(r),r); }
private:
    RawSimEncoder encoder_{}; RawIngress ingress_{};
};
static_assert(sizeof(RawFrame)<=224U,"EDM62 raw frame must remain bounded.");
static_assert(sizeof(RawIngress)<=768U,"EDM62 raw ingress must remain bounded.");
static_assert(sizeof(SimRawSource)<=1024U,"EDM62 synthetic raw source must remain bounded.");
}
