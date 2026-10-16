#pragma once
#include "EDMGapPhysicalRTShadow.h"
#include "EDMCoordinatedGapSource.h"

// EDM129 physical acquisition admission. This consumes the coherent, calibrated
// PDO receipt only. The diagnostic observer's target/speed confer no motion
// authority and are never projected into the path servo. All source sequence,
// capture time and producer identity remain the actual host PDO evidence.
namespace EDM129
{
constexpr std::uint64_t MaximumSourceAgeUs=1500ULL;
enum class SourceReason : std::uint8_t
{ None,AwaitFresh,Configuration,Lifecycle,Read,Clock,Source,Identity,Sequence,Stale,Overflow,Cancelled };
inline const char* SourceReasonName(SourceReason reason) noexcept
{
    switch(reason) {
    case SourceReason::None:return "NONE"; case SourceReason::AwaitFresh:return "AWAIT_FRESH";
    case SourceReason::Configuration:return "CONFIGURATION"; case SourceReason::Lifecycle:return "LIFECYCLE";
    case SourceReason::Read:return "READ"; case SourceReason::Clock:return "CLOCK";
    case SourceReason::Source:return "SOURCE"; case SourceReason::Identity:return "IDENTITY";
    case SourceReason::Sequence:return "SEQUENCE"; case SourceReason::Stale:return "STALE";
    case SourceReason::Overflow:return "OVERFLOW"; case SourceReason::Cancelled:return "CANCELLED";
    } return "UNKNOWN";
}
inline bool SameConversion(const EDMGapInput::ConversionResult& a,const EDMGapInput::ConversionResult& b) noexcept
{
    return a.inputStatus==b.inputStatus && a.boardVoltageValid==b.boardVoltageValid &&
        a.zeroClamped==b.zeroClamped && a.gapInRange==b.gapInRange &&
        a.boardVoltageMv==b.boardVoltageMv && a.voltageMv==b.voltageMv;
}
struct Binding
{
    EDM55::Identity physicalIdentity{};
    EDM63::Route route{};
    EDMAnalogInput::Snapshot baselineSample{};
    EDMGapInput::ConversionResult baselineConversion{};
    std::uint64_t baselineSequence=0,baselineCapturedUs=0,baselinePublication=0,baselineCycle=0,baselineObservedUs=0;
    std::uint64_t admittedUs=0,qpcFrequency=0,observerAdmittedUs=0;
    std::uint64_t shadowConfigRevision=0,shadowMaxAgeUs=0,shadowMaxTickGapUs=0;
    std::uint32_t expectedProfileRevision=0,expectedCalibrationRevision=0;
};
inline bool SameBinding(const Binding& a,const Binding& b) noexcept
{
    return EDM55::SameIdentity(a.physicalIdentity,b.physicalIdentity) && EDM63::SameRoute(a.route,b.route) &&
        EDM63::SameSample(a.baselineSample,b.baselineSample) && SameConversion(a.baselineConversion,b.baselineConversion) &&
        a.baselineSequence==b.baselineSequence && a.baselineCapturedUs==b.baselineCapturedUs &&
        a.baselinePublication==b.baselinePublication && a.baselineCycle==b.baselineCycle &&
        a.baselineObservedUs==b.baselineObservedUs && a.admittedUs==b.admittedUs &&
        a.qpcFrequency==b.qpcFrequency && a.observerAdmittedUs==b.observerAdmittedUs &&
        a.shadowConfigRevision==b.shadowConfigRevision && a.shadowMaxAgeUs==b.shadowMaxAgeUs &&
        a.shadowMaxTickGapUs==b.shadowMaxTickGapUs && a.expectedProfileRevision==b.expectedProfileRevision &&
        a.expectedCalibrationRevision==b.expectedCalibrationRevision;
}
inline bool ValidRoute(const EDM63::Route& r) noexcept
{
    return r.adIndex && r.adIndex<=64U && r.channelId==r.adIndex && r.channelCount>=r.adIndex &&
        r.deviceId && r.deviceId<=65535U && r.slaveIndex>=0 && r.inputBitOffset>=0 &&
        r.producerGeneration && r.qpcFrequency &&
        r.qpcFrequency<=(std::numeric_limits<std::uint64_t>::max)()/1000000ULL;
}
inline bool ValidBinding(const Binding& b) noexcept
{
    const auto& s=b.baselineSample; const auto& c=b.baselineConversion;
    return ValidRoute(b.route) && EDM55::ValidIdentity(b.physicalIdentity) &&
        b.physicalIdentity.source==EDM55::SourceKind::Physical &&
        b.physicalIdentity.sourceGeneration==b.physicalIdentity.session &&
        b.physicalIdentity.sourceId==EDM63::PhysicalSourceId(b.route.deviceId,b.route.channelId) &&
        b.physicalIdentity.calibrationRevision==b.expectedCalibrationRevision &&
        b.expectedProfileRevision && b.expectedCalibrationRevision && b.shadowConfigRevision==64ULL &&
        b.shadowMaxAgeUs && b.shadowMaxAgeUs<=5000ULL && b.shadowMaxTickGapUs &&
        b.shadowMaxTickGapUs<=2000ULL && b.shadowMaxTickGapUs<=b.shadowMaxAgeUs &&
        b.qpcFrequency==b.route.qpcFrequency && b.observerAdmittedUs &&
        EDM63::SameRoute(EDM63::ProducerRoute(s),b.route) && EDM63::SampleStatus(s,true)==EDM63::Reason::None &&
        b.baselineSequence && b.baselineSequence!=(std::numeric_limits<std::uint64_t>::max)() &&
        b.baselineSequence==s.sequence && b.baselineCapturedUs==s.capturedAtUs &&
        s.capturedAtUs && s.capturedAtUs==s.observedAtUs &&
        s.capturedAtMs==s.capturedAtUs/1000ULL && s.observedAtMs==s.observedAtUs/1000ULL &&
        s.capturedAtUs>b.observerAdmittedUs && b.baselineObservedUs>=s.capturedAtUs &&
        b.baselinePublication && b.baselineCycle && b.admittedUs>=b.baselineObservedUs &&
        b.admittedUs-b.baselineCapturedUs<=MaximumSourceAgeUs &&
        b.admittedUs-b.baselineCapturedUs<=b.shadowMaxAgeUs &&
        b.admittedUs-b.baselineObservedUs<=b.shadowMaxTickGapUs &&
        c.inputStatus==EDMGapInput::InputStatus::Valid && c.boardVoltageValid && c.gapInRange &&
        c.voltageMv>=0 && c.voltageMv<=300000;
}
inline bool ValidSnapshot(EDM64::Snapshot& raw,const EDM53::Stamp& stamp,
    std::uint64_t nowUs,bool clockValid,std::uint64_t frequency) noexcept
{
    EDM64::PhysicalRTShadow::ValidateRead(raw,stamp,nowUs,clockValid,frequency);
    const auto& s=raw.input.sample; const auto& c=raw.input.conversion;
    return stamp.Ready() && raw.readerState==EDM64::ReaderState::Fresh &&
        raw.state==EDM64::State::Valid && raw.reason==EDM64::Reason::None &&
        raw.input.Usable() && raw.input.reason==EDM63::Reason::None &&
        EDM63::SampleStatus(s,true)==EDM63::Reason::None &&
        !raw.physicalPermit && !raw.discharge && !raw.input.physicalPermit &&
        EDM55::ValidIdentity(raw.identity) && raw.identity.source==EDM55::SourceKind::Physical &&
        raw.identity.sourceGeneration==raw.input.observerGeneration && raw.identity.session==raw.input.observerGeneration &&
        raw.identity.sourceId==EDM63::PhysicalSourceId(s.deviceId,s.channelId) &&
        raw.identity.calibrationRevision==raw.input.expectedCalibrationRevision &&
        raw.input.expectedProfileRevision && raw.input.expectedCalibrationRevision && raw.configRevision==64ULL &&
        raw.maxAgeUs && raw.maxAgeUs<=5000ULL && raw.maxTickGapUs &&
        raw.maxTickGapUs<=2000ULL && raw.maxTickGapUs<=raw.maxAgeUs &&
        ValidRoute(EDM63::ProducerRoute(s)) && s.qpcFrequency==frequency &&
        (s.unsignedRaw ? (s.rawCode>=0 && s.rawCode<=65535) : (s.rawCode>=-32768 && s.rawCode<=32767)) &&
        s.sequence && s.capturedAtUs && s.capturedAtUs==s.observedAtUs &&
        s.capturedAtMs==s.capturedAtUs/1000ULL && s.observedAtMs==s.observedAtUs/1000ULL &&
        raw.input.admittedUs && s.capturedAtUs>raw.input.admittedUs &&
        s.capturedAtUs<=raw.observedUs && raw.observedUs<=nowUs &&
        nowUs-s.capturedAtUs<=MaximumSourceAgeUs && raw.input.ageKnown &&
        raw.input.ageUs==raw.observedUs-s.capturedAtUs &&
        c.inputStatus==EDMGapInput::InputStatus::Valid && c.boardVoltageValid && c.gapInRange &&
        c.voltageMv>=0 && c.voltageMv<=300000 &&
        raw.output.valid && raw.output.running && !raw.output.stopRequired && raw.output.fault==EDM55::Fault::None &&
        raw.output.sequence==s.sequence && raw.output.capturedAtUs==s.capturedAtUs &&
        raw.output.tick==raw.cycle && raw.output.nowUs==raw.observedUs &&
        raw.output.rawVoltageV==static_cast<double>(c.voltageMv)/1000.0;
}
inline bool MakeBinding(EDM64::Snapshot raw,const EDM53::Stamp& stamp,std::uint64_t nowUs,
    bool clockValid,std::uint64_t frequency,Binding& out) noexcept
{
    out=Binding{};
    if(!ValidSnapshot(raw,stamp,nowUs,clockValid,frequency)) return false;
    Binding b{}; b.physicalIdentity=raw.identity; b.route=EDM63::ProducerRoute(raw.input.sample);
    b.baselineSample=raw.input.sample; b.baselineConversion=raw.input.conversion;
    b.baselineSequence=raw.input.sample.sequence; b.baselineCapturedUs=raw.input.sample.capturedAtUs;
    b.baselinePublication=stamp.publication; b.baselineCycle=raw.cycle; b.baselineObservedUs=raw.observedUs;
    b.admittedUs=nowUs; b.qpcFrequency=frequency; b.observerAdmittedUs=raw.input.admittedUs;
    b.shadowConfigRevision=raw.configRevision; b.shadowMaxAgeUs=raw.maxAgeUs; b.shadowMaxTickGapUs=raw.maxTickGapUs;
    b.expectedProfileRevision=raw.input.expectedProfileRevision; b.expectedCalibrationRevision=raw.input.expectedCalibrationRevision;
    if(!ValidBinding(b)) return false;
    out=b; return true;
}
inline EDM88::SourceConfig SourceConfig(const Binding& b) noexcept
{
    EDM88::SourceConfig c{}; if(!ValidBinding(b)) return c;
    c.enabled=true; c.identity.kind=EDM88::SourceKind::Physical;
    c.identity.producerGeneration=b.route.producerGeneration; c.identity.deviceId=b.route.deviceId;
    c.identity.channelId=b.route.channelId; c.identity.configRevision=b.expectedProfileRevision;
    c.identity.calibrationRevision=b.expectedCalibrationRevision; c.samplePeriodUs=250ULL;
    c.maxAgeUs=b.shadowMaxAgeUs<MaximumSourceAgeUs?b.shadowMaxAgeUs:MaximumSourceAgeUs;
    return c;
}
struct SourceEvidence
{
    EDM88::Frame frame{};
    std::uint64_t session=0,nowUs=0,sourcePublication=0,sourceCycle=0,maximumAgeUs=0;
    std::uint64_t freshFrames=0,heldFrames=0,busyReads=0,firstSourceCapturedUs=0;
    bool ready=false,active=false,faulted=false;
    SourceReason reason=SourceReason::AwaitFresh;
};
class PhysicalSource
{
public:
    bool Begin(const Binding& binding,std::uint64_t session) noexcept
    {
        const bool wasActive=evidence_.active;
        evidence_=SourceEvidence{}; evidence_.session=session; havePhysical_=false;
        if(!session || session<=sessionHighWater_) return Fail(SourceReason::Lifecycle);
        sessionHighWater_=session;
        if(wasActive || !ValidBinding(binding) || !EDM88::ValidPhysical(SourceConfig(binding)))
            return Fail(SourceReason::Configuration);
        binding_=binding; config_=SourceConfig(binding); evidence_.active=true; return true;
    }
    void Cancel() noexcept { evidence_.active=evidence_.ready=false; if(!evidence_.faulted)evidence_.reason=SourceReason::Cancelled; }
    void Clear() noexcept { Cancel(); binding_=Binding{}; config_=EDM88::SourceConfig{}; evidence_=SourceEvidence{}; havePhysical_=false; }
    const Binding& FrozenBinding() const noexcept { return binding_; }
    const EDM88::SourceConfig& Config() const noexcept { return config_; }
    const EDM88::SourceConfig& FrozenConfig() const noexcept { return config_; }
    const SourceEvidence& Current() const noexcept { return evidence_; }
    const EDM88::Frame& CurrentFrame() const noexcept { return evidence_.frame; }
    bool CurrentAt(std::uint64_t nowUs,std::uint64_t frequency) const noexcept
    {
        const auto& f=evidence_.frame;
        return evidence_.active && evidence_.ready && !evidence_.faulted && evidence_.reason==SourceReason::None &&
            evidence_.session && evidence_.freshFrames && f.valid && frequency==binding_.qpcFrequency &&
            EDM88::SameIdentity(f.identity,config_.identity) && f.sequence>binding_.baselineSequence &&
            f.capturedAtUs>binding_.admittedUs && nowUs>=evidence_.nowUs && nowUs>=f.capturedAtUs &&
            nowUs-f.capturedAtUs<=config_.maxAgeUs && nowUs>=physical_.observedUs &&
            nowUs-physical_.observedUs<=binding_.shadowMaxTickGapUs;
    }
    bool Observe(EDM64::Snapshot raw,EDM53::Stamp stamp,std::uint64_t nowUs,
        bool clockValid,std::uint64_t frequency) noexcept
    {
        if(evidence_.faulted || !evidence_.active) return false;
        if(!clockValid || !nowUs || nowUs<binding_.admittedUs || frequency!=binding_.qpcFrequency ||
            (evidence_.nowUs && nowUs<=evidence_.nowUs)) return Fail(SourceReason::Clock);
        evidence_.nowUs=nowUs;
        // Check the OLD accepted conversion before a new arrival can replace it.
        const auto previousCapture=havePhysical_?physical_.input.sample.capturedAtUs:binding_.baselineCapturedUs;
        if(nowUs<previousCapture) return Fail(SourceReason::Clock);
        const auto oldAge=nowUs-previousCapture;
        if(oldAge>evidence_.maximumAgeUs)evidence_.maximumAgeUs=oldAge;
        if(oldAge>config_.maxAgeUs) return Fail(SourceReason::Stale);
        // A transient coherent-copy collision retains the exact previous value,
        // publication and timestamps. No empty/closed/error read gets repaired.
        if(stamp.status==EDM53::Status::Busy && havePhysical_) {
            if(evidence_.busyReads==(std::numeric_limits<std::uint64_t>::max)())return Fail(SourceReason::Overflow);
            ++evidence_.busyReads;raw=physical_;stamp=physicalStamp_;
        }
        if(!stamp.Ready())return Fail(SourceReason::Read);
        if(!ValidSnapshot(raw,stamp,nowUs,clockValid,frequency))return Fail(SourceReason::Source);
        const auto& b=binding_;const auto& s=raw.input.sample;
        if(!EDM55::SameIdentity(raw.identity,b.physicalIdentity) || !EDM63::SameRoute(EDM63::ProducerRoute(s),b.route) ||
            raw.input.admittedUs!=b.observerAdmittedUs || raw.input.expectedProfileRevision!=b.expectedProfileRevision ||
            raw.input.expectedCalibrationRevision!=b.expectedCalibrationRevision || raw.configRevision!=b.shadowConfigRevision ||
            raw.maxAgeUs!=b.shadowMaxAgeUs || raw.maxTickGapUs!=b.shadowMaxTickGapUs)return Fail(SourceReason::Identity);
        const auto& old=havePhysical_?physical_.input.sample:b.baselineSample;
        const auto& oldConversion=havePhysical_?physical_.input.conversion:b.baselineConversion;
        const auto oldPublication=havePhysical_?physicalStamp_.publication:b.baselinePublication;
        const auto oldCycle=havePhysical_?physical_.cycle:b.baselineCycle;
        const auto oldObservedUs=havePhysical_?physical_.observedUs:b.baselineObservedUs;
        if(stamp.publication<oldPublication || raw.cycle<oldCycle || raw.observedUs<oldObservedUs ||
            s.sequence<old.sequence || (s.sequence>old.sequence && s.capturedAtUs<=old.capturedAtUs) ||
            (s.sequence==old.sequence && (!EDM63::SameSample(s,old) || !SameConversion(raw.input.conversion,oldConversion))) ||
            (stamp.publication==oldPublication &&
                (raw.cycle!=oldCycle || raw.observedUs!=oldObservedUs || !EDM63::SameSample(s,old) ||
                    !SameConversion(raw.input.conversion,oldConversion))))return Fail(SourceReason::Sequence);
        physical_=raw;physicalStamp_=stamp;havePhysical_=true;
        evidence_.sourcePublication=stamp.publication;evidence_.sourceCycle=raw.cycle;
        if(!evidence_.ready && (s.sequence<=b.baselineSequence || s.capturedAtUs<=b.admittedUs)) {
            evidence_.reason=SourceReason::AwaitFresh;return true;
        }
        const bool fresh=!evidence_.ready || s.sequence>evidence_.frame.sequence;
        auto& count=fresh?evidence_.freshFrames:evidence_.heldFrames;
        if(count==(std::numeric_limits<std::uint64_t>::max)())return Fail(SourceReason::Overflow);
        ++count;
        auto& frame=evidence_.frame;frame.identity=config_.identity;frame.sequence=s.sequence;
        frame.capturedAtUs=s.capturedAtUs;frame.voltageV=static_cast<double>(raw.input.conversion.voltageMv)/1000.0;
        frame.valid=true;evidence_.ready=true;evidence_.reason=SourceReason::None;
        if(!evidence_.firstSourceCapturedUs)evidence_.firstSourceCapturedUs=s.capturedAtUs;
        return true;
    }
private:
    bool Fail(SourceReason reason) noexcept
    { if(!evidence_.faulted)evidence_.reason=reason;evidence_.faulted=true;evidence_.active=evidence_.ready=false;return false; }
    Binding binding_{};EDM88::SourceConfig config_{};SourceEvidence evidence_{};
    EDM64::Snapshot physical_{};EDM53::Stamp physicalStamp_{};
    std::uint64_t sessionHighWater_=0;
    bool havePhysical_=false;
};
static_assert(std::is_trivially_copyable<Binding>::value,"EDM129 source binding is copied by value");
static_assert(std::is_trivially_copyable<SourceEvidence>::value,"EDM129 source evidence is copied by value");
static_assert(sizeof(PhysicalSource)<=2048U,"EDM129 physical source storage must remain bounded");
}
