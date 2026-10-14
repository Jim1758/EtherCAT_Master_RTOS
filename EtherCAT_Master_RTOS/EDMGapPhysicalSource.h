#pragma once
#include "EDMGapRTRawSource.h"
#include "EDMAnalogInput.h"
#include <cstdint>
#include <limits>

// EDM63: physical PDO receipt admission, not motion or discharge authority.
// One serialized owner; no clock access, allocation, locks, or I/O. A PDO
// sequence/time proves a host receipt, never the device ADC conversion/boot.
namespace EDM63
{
constexpr std::uint64_t MaxReceiptAgeUs = 5000ULL;
enum class State : std::uint8_t { AwaitBegin, AwaitFresh, Valid, Stale, Revoked };
enum class Reason : std::uint8_t
{
    AwaitBegin, AwaitProducer, AwaitFresh, None, Configuration, CalibrationRequired,
    Generation, ReadFailure, TransportInvalid, ProducerClock, ObserverClock,
    FrequencyMismatch, SourceStatus, ProducerChanged, RouteChanged, Sequence,
    SourceClock, RawRange, Conversion, Stale, Scope, Revoked
};
inline const char* StateName(State value) noexcept
{
    switch(value) {
    case State::AwaitBegin:return "AWAIT_BEGIN"; case State::AwaitFresh:return "AWAIT_FRESH";
    case State::Valid:return "VALID"; case State::Stale:return "STALE";
    case State::Revoked:return "REVOKED"; } return "UNKNOWN";
}
inline const char* ReasonName(Reason value) noexcept
{
    switch(value) {
    case Reason::AwaitBegin:return "AWAIT_BEGIN"; case Reason::AwaitProducer:return "AWAIT_PRODUCER";
    case Reason::AwaitFresh:return "AWAIT_FRESH"; case Reason::None:return "NONE";
    case Reason::Configuration:return "CONFIGURATION"; case Reason::CalibrationRequired:return "CALIBRATION_REQUIRED";
    case Reason::Generation:return "GENERATION"; case Reason::ReadFailure:return "READ_FAILURE";
    case Reason::TransportInvalid:return "TRANSPORT_INVALID"; case Reason::ProducerClock:return "PRODUCER_CLOCK";
    case Reason::ObserverClock:return "OBSERVER_CLOCK"; case Reason::FrequencyMismatch:return "FREQUENCY_MISMATCH";
    case Reason::SourceStatus:return "SOURCE_STATUS"; case Reason::ProducerChanged:return "PRODUCER_CHANGED";
    case Reason::RouteChanged:return "ROUTE_CHANGED"; case Reason::Sequence:return "SEQUENCE";
    case Reason::SourceClock:return "SOURCE_CLOCK"; case Reason::RawRange:return "RAW_RANGE";
    case Reason::Conversion:return "CONVERSION"; case Reason::Stale:return "STALE";
    case Reason::Scope:return "SCOPE"; case Reason::Revoked:return "REVOKED";
    } return "UNKNOWN";
}
struct Route
{
    std::uint32_t adIndex=0, deviceId=0, channelId=0, channelCount=0;
    std::int32_t slaveIndex=-1, inputBitOffset=-1;
    std::uint64_t producerGeneration=0, qpcFrequency=0;
    bool unsignedRaw=false;
};
inline Route ProducerRoute(const EDMAnalogInput::Snapshot&s) noexcept
{
    Route r{}; r.adIndex=s.adIndex; r.deviceId=s.deviceId; r.channelId=s.channelId;
    r.channelCount=s.channelCount; r.slaveIndex=s.slaveIndex; r.inputBitOffset=s.inputBitOffset;
    r.producerGeneration=s.producerGeneration; r.qpcFrequency=s.qpcFrequency;
    r.unsignedRaw=s.unsignedRaw; return r;
}
inline bool SameRoute(const Route&a,const Route&b) noexcept
{
    return a.adIndex==b.adIndex && a.deviceId==b.deviceId && a.channelId==b.channelId &&
        a.channelCount==b.channelCount && a.slaveIndex==b.slaveIndex && a.inputBitOffset==b.inputBitOffset &&
        a.producerGeneration==b.producerGeneration && a.qpcFrequency==b.qpcFrequency && a.unsignedRaw==b.unsignedRaw;
}
inline bool SameSample(const EDMAnalogInput::Snapshot&a,const EDMAnalogInput::Snapshot&b) noexcept
{
    return SameRoute(ProducerRoute(a),ProducerRoute(b)) && a.rawCode==b.rawCode &&
        a.sequence==b.sequence && a.capturedAtMs==b.capturedAtMs && a.observedAtMs==b.observedAtMs &&
        a.capturedAtUs==b.capturedAtUs && a.observedAtUs==b.observedAtUs &&
        a.configured==b.configured && a.hasSample==b.hasSample && a.transportValid==b.transportValid &&
        a.clockValid==b.clockValid && a.rtClockValid==b.rtClockValid && a.rtSampleValid==b.rtSampleValid && a.reason==b.reason;
}
inline bool ValidRoute(const Route&r,const EDMGapInput::Profile&p) noexcept
{
    return r.adIndex==p.adIndex && r.adIndex!=0U && r.adIndex<=64U && r.channelId==r.adIndex &&
        r.deviceId!=0U && r.deviceId<=65535U && (!p.expectedDeviceId || p.expectedDeviceId==r.deviceId) &&
        r.channelCount>=r.adIndex && r.slaveIndex>=0 && r.inputBitOffset>=0 &&
        r.producerGeneration && r.qpcFrequency &&
        r.qpcFrequency<=(std::numeric_limits<std::uint64_t>::max)()/1000000ULL &&
        (r.unsignedRaw ? p.rawMin>=0 && p.rawMax<=65535 : p.rawMin>=-32768 && p.rawMax<=32767);
}
inline bool ValidProfile(const EDMGapInput::Profile&p) noexcept
{ return p.source==EDMGap::Source::PHYSICAL && p.calibrationConfirmed && EDMGapInput::Validate(p); }
inline Reason SampleStatus(const EDMAnalogInput::Snapshot&s,bool readSucceeded) noexcept
{
    if(!readSucceeded) return Reason::ReadFailure;
    if(!s.configured || !s.hasSample || !s.producerGeneration || !s.qpcFrequency) return Reason::AwaitProducer;
    if(!s.transportValid) return Reason::TransportInvalid;
    if(!s.clockValid || !s.rtClockValid || !s.rtSampleValid) return Reason::ProducerClock;
    if(s.reason!=EDMAnalogInput::Reason::Valid) return Reason::SourceStatus;
    return Reason::None;
}
struct Admission
{
    EDMGapInput::Profile calibration{};
    Route route{};
    EDMAnalogInput::Snapshot baseline{};
    std::uint64_t observerGeneration=0, admittedUs=0, observerFrequency=0;
};
inline Admission MakeAdmission(const EDMGapInput::Profile&p,const EDMAnalogInput::Snapshot&s,
    std::uint64_t generation,std::uint64_t nowUs,std::uint64_t observerFrequency) noexcept
{
    Admission a{}; a.calibration=p; a.route=ProducerRoute(s); a.baseline=s;
    a.observerGeneration=generation; a.admittedUs=nowUs; a.observerFrequency=observerFrequency; return a;
}
struct Receipt
{
    State state=State::AwaitBegin;
    Reason reason=Reason::AwaitBegin;
    EDMAnalogInput::Snapshot sample{}; // Actual producer evidence; no calibration stamp.
    EDMGapInput::ConversionResult conversion{};
    std::uint64_t observerGeneration=0, admittedUs=0, ageUs=0;
    std::uint32_t expectedProfileRevision=0, expectedCalibrationRevision=0;
    bool ageKnown=false, usable=false;
    // Permanently false, including valid physical observation.
    bool physicalPermit=false;
    bool Usable() const noexcept { return state==State::Valid && usable; }
};
class PhysicalAdapter
{
public:
    bool Begin(const Admission&a) noexcept
    {
        const bool wasLive=live_; live_=false;
        receipt_=Receipt{}; receipt_.observerGeneration=a.observerGeneration; receipt_.admittedUs=a.admittedUs;
        receipt_.expectedProfileRevision=a.calibration.profileRevision;
        receipt_.expectedCalibrationRevision=a.calibration.calibrationRevision; receipt_.sample=a.baseline;
        if(!a.observerGeneration || a.observerGeneration<=generationHighWater_) { Fail(Reason::Generation); return false; }
        generationHighWater_=a.observerGeneration;
        if(wasLive) { Fail(Reason::Configuration); return false; }
        if(!a.calibration.calibrationConfirmed) { Fail(Reason::CalibrationRequired); return false; }
        if(!ValidProfile(a.calibration) || !ValidRoute(a.route,a.calibration) ||
            !SameRoute(a.route,ProducerRoute(a.baseline))) { Fail(Reason::Configuration); return false; }
        if(a.admittedUs<lastNowUs_) { Fail(Reason::ObserverClock); return false; }
        if(!a.admittedUs || !a.observerFrequency || a.observerFrequency!=a.route.qpcFrequency ||
            (observerFrequency_ && observerFrequency_!=a.observerFrequency))
        { Fail(Reason::FrequencyMismatch); return false; }
        const auto status=SampleStatus(a.baseline,true);
        if(status!=Reason::None) { Fail(status); return false; }
        if(!a.baseline.sequence || a.baseline.sequence==(std::numeric_limits<std::uint64_t>::max)())
        { Fail(Reason::Sequence); return false; }
        if(!ValidSourceTime(a.baseline,a.admittedUs)) { Fail(Reason::SourceClock); return false; }
        if(a.route.producerGeneration<producerHighWater_) { Fail(Reason::ProducerChanged); return false; }
        if(a.route.producerGeneration==producerHighWater_ && producerHighWater_)
        {
            if(!SameRoute(a.route,historyRoute_)) { Fail(Reason::RouteChanged); return false; }
            if(a.baseline.sequence<historySample_.sequence ||
                (a.baseline.sequence==historySample_.sequence && !SameSample(a.baseline,historySample_)))
            { Fail(Reason::Sequence); return false; }
            if(a.baseline.sequence>historySample_.sequence && a.baseline.capturedAtUs<=historySample_.capturedAtUs)
            { Fail(Reason::SourceClock); return false; }
        }
        const auto conversion=EDMGapInput::ConvertRawValidated(a.calibration,a.baseline.rawCode);
        if(!AcceptConversion(conversion)) return false;
        admission_=a; last_=a.baseline; Remember(a.baseline); lastNowUs_=a.admittedUs;
        observerFrequency_=a.observerFrequency;
        live_=true; haveFresh_=false;
        receipt_.state=State::AwaitFresh; receipt_.reason=Reason::AwaitFresh;
        // A valid pre-admission conversion is only a fence; not a fresh receipt.
        receipt_.conversion=EDMGapInput::ConversionResult{}; return true;
    }
    Receipt Observe(const EDMAnalogInput::Snapshot&s,bool readSucceeded,std::uint64_t nowUs,
        bool clockValid,std::uint64_t observerFrequency) noexcept
    {
        if(!live_) return receipt_;
        receipt_.sample=s; receipt_.usable=false; receipt_.ageKnown=false; receipt_.ageUs=0;
        receipt_.conversion=EDMGapInput::ConversionResult{};
        if(!clockValid || !nowUs || nowUs<lastNowUs_ || nowUs<admission_.admittedUs) return Fail(Reason::ObserverClock);
        lastNowUs_=nowUs;
        if(!readSucceeded) return Fail(Reason::ReadFailure);
        if(observerFrequency!=admission_.observerFrequency || s.qpcFrequency!=admission_.route.qpcFrequency)
            return Fail(Reason::FrequencyMismatch);
        if(s.producerGeneration!=admission_.route.producerGeneration) return Fail(Reason::ProducerChanged);
        if(!SameRoute(ProducerRoute(s),admission_.route)) return Fail(Reason::RouteChanged);
        const auto status=SampleStatus(s,readSucceeded);
        if(status!=Reason::None) return Fail(status);
        if(!s.sequence || s.sequence<last_.sequence) return Fail(Reason::Sequence);
        if(!ValidSourceTime(s,nowUs)) return Fail(Reason::SourceClock);
        const bool duplicate=s.sequence==last_.sequence;
        if(duplicate && !SameSample(s,last_)) return Fail(Reason::Sequence);
        if(!duplicate && s.capturedAtUs<=last_.capturedAtUs) return Fail(Reason::SourceClock);
        if(!haveFresh_ && duplicate)
        { receipt_.state=State::AwaitFresh; receipt_.reason=Reason::AwaitFresh; return receipt_; }
        if(!haveFresh_ && s.capturedAtUs<=admission_.admittedUs) return Fail(Reason::SourceClock);
        // Skipped producer sequences are normal for a slower NC observer.
        const auto conversion=EDMGapInput::ConvertRawValidated(admission_.calibration,s.rawCode);
        if(!AcceptConversion(conversion)) return receipt_;
        last_=s; Remember(s); haveFresh_=true; receipt_.conversion=conversion;
        receipt_.ageKnown=true; receipt_.ageUs=nowUs-s.capturedAtUs;
        const std::uint64_t configuredAgeUs=admission_.calibration.gap.maxAgeMs*1000ULL;
        const std::uint64_t maxAgeUs=configuredAgeUs<MaxReceiptAgeUs?configuredAgeUs:MaxReceiptAgeUs;
        receipt_.usable=receipt_.ageUs<=maxAgeUs;
        receipt_.state=receipt_.usable?State::Valid:State::Stale;
        receipt_.reason=receipt_.usable?Reason::None:Reason::Stale;
        return receipt_;
    }
    void Cancel() noexcept { Fail(Reason::Revoked); }
    const Receipt& Snapshot() const noexcept { return receipt_; }
    const Admission& FrozenAdmission() const noexcept { return admission_; }
    std::uint64_t GenerationHighWater() const noexcept { return generationHighWater_; }
private:
    static bool ValidSourceTime(const EDMAnalogInput::Snapshot&s,std::uint64_t nowUs) noexcept
    {
        return s.capturedAtUs && s.capturedAtUs==s.observedAtUs && s.observedAtUs<=nowUs &&
            s.capturedAtMs==s.capturedAtUs/1000ULL && s.observedAtMs==s.observedAtUs/1000ULL;
    }
    bool AcceptConversion(const EDMGapInput::ConversionResult&c) noexcept
    {
        if(c.inputStatus==EDMGapInput::InputStatus::RawOutOfRange) { Fail(Reason::RawRange); return false; }
        if(c.inputStatus!=EDMGapInput::InputStatus::Valid || !c.boardVoltageValid || !c.gapInRange)
        { Fail(Reason::Conversion); return false; } return true;
    }
    void Remember(const EDMAnalogInput::Snapshot&s) noexcept
    { producerHighWater_=s.producerGeneration; historyRoute_=ProducerRoute(s); historySample_=s; }
    Receipt Fail(Reason reason) noexcept
    {
        if(receipt_.state!=State::Revoked) receipt_.reason=reason;
        live_=false; receipt_.state=State::Revoked; receipt_.usable=false;
        receipt_.conversion=EDMGapInput::ConversionResult{}; return receipt_;
    }
    Admission admission_{}; Receipt receipt_{}; EDMAnalogInput::Snapshot last_{},historySample_{};
    Route historyRoute_{};
    std::uint64_t generationHighWater_=0, producerHighWater_=0, lastNowUs_=0, observerFrequency_=0;
    bool live_=false,haveFresh_=false;
};

// Native NC shadow has no machining scope. Initialize is explicit profile
// installation, never an automatic renewal after a fault. Missing boot-time
// evidence awaits the first healthy baseline. Post-admission failures latch.
class ShadowObserver
{
public:
    bool Initialize(const EDMGapInput::Profile&p) noexcept
    {
        adapter_.Cancel(); profile_=p; receipt_=Receipt{}; pending_=false;
        receipt_.expectedProfileRevision=p.profileRevision; receipt_.expectedCalibrationRevision=p.calibrationRevision;
        if(!p.calibrationConfirmed) { Revoke(Reason::CalibrationRequired); return false; }
        if(!ValidProfile(p)) { Revoke(Reason::Configuration); return false; }
        pending_=true; receipt_.state=State::AwaitFresh; receipt_.reason=Reason::AwaitProducer; return true;
    }
    Receipt Observe(const EDMAnalogInput::Snapshot&s,bool readSucceeded,std::uint64_t nowUs,
        bool clockValid,std::uint64_t observerFrequency) noexcept
    {
        if(receipt_.state==State::Revoked) return receipt_;
        if(!pending_) { receipt_=adapter_.Observe(s,readSucceeded,nowUs,clockValid,observerFrequency); return receipt_; }
        receipt_.sample=s;
        if(s.qpcFrequency && observerFrequency && s.qpcFrequency!=observerFrequency)
            return Revoke(Reason::FrequencyMismatch);
        if(!clockValid || !nowUs || !observerFrequency)
        { receipt_.reason=Reason::ObserverClock; return receipt_; }
        const auto status=SampleStatus(s,readSucceeded);
        if(status!=Reason::None) { receipt_.reason=status; return receipt_; }
        if(adapter_.GenerationHighWater()==(std::numeric_limits<std::uint64_t>::max)()) return Revoke(Reason::Generation);
        const auto admission=MakeAdmission(profile_,s,adapter_.GenerationHighWater()+1ULL,nowUs,observerFrequency);
        pending_=false; adapter_.Begin(admission); receipt_=adapter_.Snapshot(); return receipt_;
    }
    void Cancel() noexcept { adapter_.Cancel(); Revoke(Reason::Revoked); }
    const Receipt& Snapshot() const noexcept { return receipt_; }
    const Admission& FrozenAdmission() const noexcept { return adapter_.FrozenAdmission(); }
private:
    Receipt Revoke(Reason reason) noexcept
    { pending_=false; receipt_.state=State::Revoked; receipt_.reason=reason; receipt_.usable=false; return receipt_; }
    EDMGapInput::Profile profile_{}; PhysicalAdapter adapter_{}; Receipt receipt_{}; bool pending_=false;
};
inline std::uint64_t PhysicalSourceId(std::uint32_t deviceId,std::uint32_t channelId) noexcept
{ return (static_cast<std::uint64_t>(deviceId)<<32U)|channelId; }

// Future scope projection. The caller must provide actual full scope and an
// independently admitted producer; no native code wires this into Motion.
// AwaitFresh exposes no valid sample; any first-sample timeout is owned by the
// caller. Stale receipts retain acquisition evidence so unchanged EDM55 owns
// canonical freshness/late-arrival fault classification, without late repair.
class BoundPhysicalSource
{
public:
    bool Begin(const EDM62::Descriptor&d,const Admission&a) noexcept
    {
        const bool rawReady=ingress_.Begin(d);
        const bool admitted=adapter_.Begin(a);
        lastTick_=lastNowUs_=0;
        if(!rawReady || !admitted || d.binding.identity.source!=EDM55::SourceKind::Physical ||
            d.binding.identity.sourceId!=PhysicalSourceId(a.route.deviceId,a.route.channelId) ||
            !EDM62::SameCalibration(d.calibration,a.calibration) || d.deviceId!=a.route.deviceId ||
            d.channelId!=a.route.channelId || d.binding.admittedUs!=a.admittedUs)
        { Cancel(); return false; }
        return true;
    }
    EDM61::Frame Capture(const EDMAnalogInput::Snapshot&s,bool readSucceeded,std::uint64_t tick,
        std::uint64_t nowUs,bool clockValid,std::uint64_t observerFrequency) noexcept
    {
        const auto&d=ingress_.FrozenDescriptor();
        if(!ingress_.Snapshot().live) return ingress_.Snapshot();
        if(!tick || !nowUs || tick<d.binding.admittedTick || nowUs<d.binding.admittedUs ||
            (lastTick_ && (tick<lastTick_ || nowUs<lastNowUs_ || ((tick==lastTick_)!=(nowUs==lastNowUs_)))))
        { Cancel(); return ingress_.Snapshot(); }
        lastTick_=tick; lastNowUs_=nowUs;
        const auto receipt=adapter_.Observe(s,readSucceeded,nowUs,clockValid,observerFrequency);
        if(receipt.state==State::Revoked) { ingress_.Cancel(); return ingress_.Snapshot(); }
        if(receipt.state!=State::Valid && receipt.state!=State::Stale) return ingress_.Snapshot();
        // Expected scope/calibration are stamped only after the actual producer
        // receipt passed the independent physical admission above.
        auto raw=EDM62::InitialFrame(d); raw.sequence=receipt.sample.sequence;
        raw.capturedAtUs=receipt.sample.capturedAtUs; raw.rawCode=receipt.sample.rawCode;
        raw.valid=raw.transportValid=true;
        return ingress_.Publish(raw,tick,nowUs);
    }
    void Cancel() noexcept { adapter_.Cancel(); ingress_.Cancel(); }
    const EDM61::Frame& Snapshot() const noexcept { return ingress_.Snapshot(); }
    const Receipt& ReceiptSnapshot() const noexcept { return adapter_.Snapshot(); }
    const EDM62::Descriptor& FrozenDescriptor() const noexcept { return ingress_.FrozenDescriptor(); }
private:
    PhysicalAdapter adapter_{}; EDM62::RawIngress ingress_{};
    std::uint64_t lastTick_=0,lastNowUs_=0;
};
static_assert(sizeof(Receipt)<=224U,"EDM63 receipt must remain bounded.");
static_assert(sizeof(PhysicalAdapter)<=1024U,"EDM63 physical adapter must remain bounded.");
static_assert(sizeof(ShadowObserver)<=1536U,"EDM63 physical shadow must remain bounded.");
static_assert(sizeof(BoundPhysicalSource)<=2048U,"EDM63 physical source must remain bounded.");
}
