#pragma once
#include "EDMGapServoRuntime.h"
#include <cstdint>
#include <cmath>
#include <limits>

// EDM61: one serialized RT source owner. No allocation, clocks, locks or I/O.
// Request voltage is SIM fixture intent, never a physical ADC conversion.
// External providers implement Capture(Request) -> Frame; no consumer fallback.
namespace EDM61
{
struct Binding
{
    EDM55::Identity identity{};
    std::uint32_t schemaRevision=1, clockDomain=1; // 1 = RuntimeMonotonicUs
    std::uint64_t session=0, run=0, cache=0, dispatch=0, map=0, config=0;
    std::uint64_t servoGeneration=0, admittedTick=0, admittedUs=0;
    std::uint32_t epoch=0, ownerGeneration=0;
    std::uint8_t owner=0, profile=0;
};
inline std::uint64_t SourceId(unsigned profile) noexcept
{ return profile>=49U?60U:profile>=46U?59U:profile>=42U?58U:profile>=38U?57U:56U; }
inline bool Valid(const Binding& b) noexcept
{
    return b.schemaRevision==1U && b.clockDomain==1U && b.profile>=35U && b.profile<=52U && b.session && b.run && b.cache && b.dispatch &&
        b.map && b.config && b.epoch && b.ownerGeneration && b.owner==1U &&
        b.servoGeneration && b.admittedTick && b.admittedUs &&
        EDM55::ValidIdentity(b.identity) && b.identity.source==EDM55::SourceKind::Simulation &&
        b.identity.sourceId==SourceId(b.profile) && b.identity.calibrationRevision==1U &&
        b.identity.sourceGeneration==b.servoGeneration && b.identity.session==b.servoGeneration;
}
inline bool Same(const Binding&a,const Binding&b) noexcept
{
    return a.schemaRevision==b.schemaRevision && a.clockDomain==b.clockDomain && EDM55::SameIdentity(a.identity,b.identity) && a.session==b.session && a.run==b.run &&
        a.cache==b.cache && a.dispatch==b.dispatch && a.map==b.map && a.config==b.config &&
        a.servoGeneration==b.servoGeneration && a.admittedTick==b.admittedTick && a.admittedUs==b.admittedUs &&
        a.epoch==b.epoch && a.ownerGeneration==b.ownerGeneration && a.owner==b.owner && a.profile==b.profile;
}
template<class S> Binding MakeBinding(const S&s,unsigned profile,std::uint64_t generation,
    std::uint64_t tick,std::uint64_t nowUs) noexcept
{
    Binding b{}; b.identity.source=EDM55::SourceKind::Simulation;
    b.identity.sourceId=SourceId(profile); b.identity.sourceGeneration=generation;
    b.identity.calibrationRevision=1; b.identity.session=generation;
    b.session=s.session; b.run=s.runGeneration; b.cache=s.cacheGeneration; b.dispatch=s.dispatchGeneration;
    b.map=s.axisMapGeneration; b.config=s.configGeneration; b.epoch=s.executionEpoch;
    b.ownerGeneration=s.ownerGeneration; b.owner=s.owner;
    b.servoGeneration=generation; b.admittedTick=tick; b.admittedUs=nowUs;
    b.profile=profile<=255U?static_cast<std::uint8_t>(profile):0U; return b;
}
struct Request
{
    std::uint64_t tick=0, nowUs=0;
    double voltageV=0;
    bool freezeCapture=false;
};
struct Frame
{
    Binding binding{};
    EDM55::Sample sample{};
    bool live=false;
};
class SimSource
{
public:
    bool Begin(const Binding& binding) noexcept
    {
        const bool wasLive=frame_.live;
        frame_.live=false;
        // Reserve attempted generations before validation, including failed
        // reconfiguration. Cancel/Begin must never recycle a source identity.
        if(!binding.servoGeneration || binding.servoGeneration<=generationHighWater_) return false;
        generationHighWater_=binding.servoGeneration;
        if(wasLive || !Valid(binding)) return false;
        frame_=Frame{}; frame_.binding=binding; frame_.sample.identity=binding.identity;
        frame_.live=true; lastTick_=lastUs_=0; return true;
    }
    void Cancel() noexcept { frame_.live=false; }
    const Frame& Snapshot() const noexcept { return frame_; }
    std::uint64_t GenerationHighWater() const noexcept { return generationHighWater_; }
    Frame Capture(const Request&r) noexcept
    {
        if(!frame_.live) return frame_;
        if(!r.tick || !r.nowUs || r.tick<frame_.binding.admittedTick || r.nowUs<frame_.binding.admittedUs ||
            !std::isfinite(r.voltageV) || r.voltageV<0.0 || r.voltageV>300.0 ||
            (lastTick_ && (r.tick<lastTick_ || r.nowUs<lastUs_ ||
                ((r.tick==lastTick_)!=(r.nowUs==lastUs_)))))
        { Cancel(); return frame_; }
        // Polling an existing conversion never changes its timestamp/payload.
        lastTick_=r.tick; lastUs_=r.nowUs;
        auto& sample=frame_.sample;
        if(!r.freezeCapture && (!sample.sequence || r.nowUs-sample.capturedAtUs>=1000ULL))
        {
            if(sample.sequence==(std::numeric_limits<std::uint64_t>::max)())
            { Cancel(); return frame_; }
            sample.valid=true; ++sample.sequence; sample.capturedAtUs=r.nowUs; sample.voltageV=r.voltageV;
        }
        return frame_;
    }
private:
    Frame frame_{};
    std::uint64_t generationHighWater_=0, lastTick_=0, lastUs_=0;
};
static_assert(sizeof(Frame)<=224U,"EDM61 frame must remain bounded.");
static_assert(sizeof(SimSource)<=256U,"EDM61 source must remain bounded.");
}
