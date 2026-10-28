#include "PositionAutoTuneCapture.h"
#include "PositionAutoTuneConfig.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>

namespace PositionAutoTune {
namespace {
CaptureRecorder recorder;
bool FinitePositive(double v) noexcept { return std::isfinite(v)&&v>0.0; }
bool ValidGains(const CaptureGains& g) noexcept {
    return std::isfinite(g.kp)&&g.kp>=0.0&&std::isfinite(g.ki)&&g.ki>=0.0&&
        std::isfinite(g.kd)&&g.kd==0.0&&std::isfinite(g.kvff)&&g.kvff>=0.0;
}
bool SameGains(const CaptureSample& s,const CaptureGains& g) noexcept {
    return s.kp==g.kp&&s.ki==g.ki&&s.kd==g.kd&&s.kvff==g.kvff;
}
bool FiniteSample(const CaptureSample& s) noexcept {
    return std::isfinite(s.commandPulse)&&std::isfinite(s.actualPulse)&&std::isfinite(s.referencePps)&&
        std::isfinite(s.actualVelocityPps)&&std::isfinite(s.proposedLogicalPps)&&
        std::isfinite(s.outputLogicalPps)&&std::isfinite(s.pTerm)&&std::isfinite(s.iTerm)&&
        std::isfinite(s.feedforwardPps)&&std::isfinite(s.kp)&&std::isfinite(s.ki)&&
        std::isfinite(s.kd)&&std::isfinite(s.kvff)&&std::isfinite(s.integral);
}
bool AddTrackingMoment(CaptureTrackingMoments& moments,double error,
    std::uint64_t elapsedUs) noexcept {
    ++moments.sampleCount;moments.observedUs+=elapsedUs;
    moments.peakAbsoluteErrorPulse=(std::max)(moments.peakAbsoluteErrorPulse,std::fabs(error));
    moments.meanErrorPulse+=(error-moments.meanErrorPulse)/static_cast<double>(moments.sampleCount);
    moments.meanSquaredErrorPulse+=(error*error-moments.meanSquaredErrorPulse)/
        static_cast<double>(moments.sampleCount);
    return std::isfinite(moments.peakAbsoluteErrorPulse)&&std::isfinite(moments.meanErrorPulse)&&
        std::isfinite(moments.meanSquaredErrorPulse);
}
}
static_assert(std::is_trivially_copyable<CaptureSample>::value,"Capture sample must be POD-like");
static_assert(std::is_trivially_copyable<CaptureFaultSnapshot>::value,"Fault snapshot must be POD-like");
static_assert(sizeof(CaptureSample)%sizeof(std::uint64_t)==0U,"Whole-word atomic snapshot required");
static_assert(ActualVelocityWindowUs==ActualVelocityIntervals*250ULL,"Velocity window must match the nominal capture period");
CaptureRecorder& Capture() noexcept { return recorder; }
double ReferenceVelocityLimitPps(const CaptureRequest& request) noexcept {
    return request.maximumReferenceVelocityPps>0.0?
        request.maximumReferenceVelocityPps:request.maximumVelocityPps;
}

bool CaptureRecorder::Start(const CaptureRequest& r) noexcept {
    const auto old=State();
    if(old!=CaptureState::Idle&&old!=CaptureState::Complete&&old!=CaptureState::Fault)return false;
    if(m_runtimeStopPending.load(std::memory_order_acquire))return false;
    if(!r.session||r.axis<0||r.axis>=static_cast<int>(kAxisCount)||r.axisType>1U||!r.owner||!r.ownerGeneration||!r.epoch||
        !FinitePositive(r.pulsesPerMm)||!std::isfinite(r.originPulse)||
        !FinitePositive(r.excursionPulse)||!FinitePositive(r.boundaryMarginPulse)||
        !FinitePositive(r.followingLimitPulse)||!FinitePositive(r.maximumVelocityPps)||
        !std::isfinite(r.maximumReferenceVelocityPps)||r.maximumReferenceVelocityPps<0.0||
        r.maximumReferenceVelocityPps>r.maximumVelocityPps||
        !std::isfinite(r.cruiseVelocityPps)||r.cruiseVelocityPps<0.0||
        r.cruiseVelocityPps>ReferenceVelocityLimitPps(r)||
        !FinitePositive(r.settleWindowPulse)||!FinitePositive(r.settleVelocityPps)||
        r.maximumVelocityPps>static_cast<double>((std::numeric_limits<std::int32_t>::max)())||
        !r.maximumTicks||r.maximumTicks>14400000ULL||!r.sampleStride||r.sampleStride>64U||
        (r.sampleStride==1U?r.maximumTicks>CaptureCapacity:
            (r.maximumTicks+r.sampleStride-1ULL)/r.sampleStride+1ULL>CaptureCapacity)||!ValidGains(r.motionGains)||
        !ValidGains(r.idleGains)||r.motionGains.kp<=0.0||r.motionGains.ki!=0.0||
        !std::isfinite(r.originPulse+r.excursionPulse+r.boundaryMarginPulse)||
        !std::isfinite(r.originPulse-(r.pointToPoint?0.0:r.excursionPulse)-r.boundaryMarginPulse))return false;
    if(r.approachEnabled) {
        if(!r.pointToPoint||!std::isfinite(r.approachStartPulse)||
            !std::isfinite(r.approachStartPulse-r.boundaryMarginPulse)||
            !std::isfinite(r.approachStartPulse+r.boundaryMarginPulse)||
            !std::isfinite(r.approachStartPulse-r.settleWindowPulse-1.0)||
            !std::isfinite(r.approachStartPulse+r.settleWindowPulse+1.0)||!r.measurementSettleHoldUs||
            r.measurementSettleHoldUs>=r.maximumTicks*250ULL)
            return false;
    }
    if(r.pointToPoint&&(r.originPulse+r.excursionPulse<=r.originPulse||
        !std::isfinite(r.originPulse-r.boundaryMarginPulse)||
        !std::isfinite(r.originPulse-r.settleWindowPulse-1.0)||
        !std::isfinite(r.originPulse+r.excursionPulse+r.settleWindowPulse+1.0)))return false;
    std::uint32_t expected=static_cast<std::uint32_t>(old);
    if(!m_state.compare_exchange_strong(expected,static_cast<std::uint32_t>(CaptureState::Preparing),
        std::memory_order_acq_rel,std::memory_order_acquire))return false;
    if(!m_samples) {
        m_samples.reset(new(std::nothrow) CaptureSample[CaptureCapacity]);
        if(!m_samples) {
            m_state.store(static_cast<std::uint32_t>(old),std::memory_order_release);
            return false;
        }
    }
    m_request=r;m_count=0U;m_rawCount=0ULL;m_storageAnchorRawCount=0ULL;
    m_statistics=CaptureStatistics{};m_statistics.session=r.session;
    m_statistics.cruiseQualificationVersion=r.cruiseVelocityPps>0.0?CruiseQualificationVersion:0U;
    m_trackingDirection=0;m_cruiseStableSinceUs=m_cruiseObservedSinceUs=0ULL;
    m_cruiseReferenceMin=m_cruiseReferenceMax=0.0;m_cruiseLegQualified=false;
    m_firstCycle=m_firstUs=m_lastCycle=m_lastUs=0ULL;
    m_cycleOpen=false;m_staged=false;m_pending=CaptureSample{};m_cycle=CaptureCycle{};
    m_fault=CaptureFaultSnapshot{};
    m_measurementBoundary=CaptureMeasurementBoundary{};m_measurementActive=!r.approachEnabled;
    m_measurementRequestSession.store(0ULL,std::memory_order_relaxed);
    m_measurementBoundaryPublished.store(false,std::memory_order_relaxed);
    m_settledSinceUs=0ULL;m_idleWindowSinceUs=0ULL;m_settledCycles=0U;m_previousCommand=0.0;
    m_settleEligibleSamples=0U;m_settleMinimumPulse=m_settleMaximumPulse=0.0;
    m_reason.store(0U,std::memory_order_relaxed);m_cancelReason.store(0U,std::memory_order_relaxed);
    m_finish.store(false,std::memory_order_relaxed);m_publishedCount.store(0U,std::memory_order_relaxed);
    m_latestSequence.store(0ULL,std::memory_order_seq_cst);
    m_legSequence.store(0ULL,std::memory_order_seq_cst);m_legPublishedIndex=0ULL;
    m_legConsumedSequence=m_liveLegIndex=m_liveLegFirstCycle=m_liveEndpointCount=m_livePublishedCount=0ULL;
    m_liveTargetPulse=m_liveStartPulse=m_liveOvershootPulse=m_liveCurrentErrorPulse=0.0;
    m_liveLegEndpointSeen=false;m_liveOvershootCoverage=true;
    PublishLiveTelemetry(false,false);
    m_state.store(static_cast<std::uint32_t>(CaptureState::Armed),std::memory_order_release);
    return true;
}
void CaptureRecorder::RequestFinish() noexcept { if(Active())m_finish.store(true,std::memory_order_release); }
bool CaptureRecorder::RequestMeasurement(std::uint64_t session) noexcept {
    if(!Active()||!session||session!=m_request.session||!m_request.approachEnabled)return false;
    std::uint64_t zero=0ULL;
    return m_measurementRequestSession.compare_exchange_strong(zero,session,
        std::memory_order_release,std::memory_order_relaxed)||zero==session;
}
bool CaptureRecorder::TryReadMeasurementBoundary(std::uint64_t session,CaptureMeasurementBoundary& boundary) const noexcept {
    if(!m_measurementBoundaryPublished.load(std::memory_order_acquire))return false;
    if(!session||m_measurementBoundary.session!=session)return false;
    boundary=m_measurementBoundary;return true;
}
void CaptureRecorder::Cancel(CaptureReason reason) noexcept {
    if(reason==CaptureReason::None)reason=CaptureReason::Cancelled;
    if(Active()) {std::uint32_t zero=0U;m_cancelReason.compare_exchange_strong(zero,
        static_cast<std::uint32_t>(reason),std::memory_order_release,std::memory_order_relaxed);}
}
bool CaptureRecorder::RuntimeFail(CaptureReason reason) noexcept {
    if(!Active())return false;
    return RuntimeFailAt(reason,CaptureFaultStage::External,m_cycleOpen?&m_cycle:nullptr,
        m_cycleOpen&&m_staged?&m_pending:nullptr);
}
bool CaptureRecorder::RuntimeFailAt(CaptureReason reason,CaptureFaultStage stage,
    const CaptureCycle* cycle,const CaptureSample* sample,bool hasObservedValue,
    double observedValue,bool observedValid,int observedAxis,
    const CaptureVelocityEvidence* velocity) noexcept {
    if(!Active())return false;
    if(reason==CaptureReason::None)reason=CaptureReason::Input;
    // Single RT writer, written only on the first failing call. Publish Fault
    // after this bounded copy so the NC owner can read it without RT logging,
    // allocation or a lock. Never substitute the previous accepted sample.
    m_fault.session=m_request.session;m_fault.reason=reason;m_fault.stage=stage;
    m_fault.lastGoodCycle=m_lastCycle;m_fault.lastGoodTimestampUs=m_lastUs;
    m_fault.sampleCount=static_cast<std::uint64_t>(m_count);
    m_fault.hasCycle=cycle!=nullptr;m_fault.hasSample=sample!=nullptr;
    if(cycle)m_fault.cycle=*cycle;
    if(sample)m_fault.sample=*sample;
    m_fault.velocity=velocity?*velocity:CaptureVelocityEvidence{};
    m_fault.approachActive=m_request.approachEnabled&&!m_measurementActive;
    m_fault.cycleOpen=m_cycleOpen;m_fault.sampleStaged=m_staged;
    m_fault.hasObservedValue=hasObservedValue;m_fault.observedValue=observedValue;
    m_fault.observedValid=observedValid;m_fault.observedAxis=observedAxis;
    m_cycleOpen=false;m_staged=false;m_statistics.complete=false;
    m_reason.store(static_cast<std::uint32_t>(reason),std::memory_order_relaxed);
    m_publishedCount.store(m_count,std::memory_order_relaxed);
    m_runtimeStopPending.store(true,std::memory_order_release);
    PublishLiveTelemetry(true,false);
    m_state.store(static_cast<std::uint32_t>(CaptureState::Fault),std::memory_order_release);
    return false;
}
bool CaptureRecorder::TryReadFault(CaptureFaultSnapshot& snapshot) const noexcept {
    if(State()!=CaptureState::Fault)return false;
    snapshot=m_fault;return true;
}
bool CaptureRecorder::TryReadStatistics(CaptureStatistics& statistics) const noexcept {
    if(!Terminal())return false;
    statistics=m_statistics;return true;
}
bool CaptureRecorder::RuntimeBegin(const CaptureCycle& c) noexcept {
    if(!Active())return true;
    const auto fail=[this,&c](CaptureReason reason) noexcept {
        return RuntimeFailAt(reason,CaptureFaultStage::Begin,&c,nullptr);
    };
    const auto cancelled=m_cancelReason.load(std::memory_order_acquire);
    if(cancelled)return fail(static_cast<CaptureReason>(cancelled));
    if(m_cycleOpen)return fail(CaptureReason::MissingSample);
    if(!c.sourceValid||!c.layoutGood)return fail(CaptureReason::Source);
    if(!c.clockValid||!c.cycle||!c.timestampUs)return fail(CaptureReason::Clock);
    if(!c.safetyGood||c.owner!=m_request.owner||c.ownerGeneration!=m_request.ownerGeneration||
        c.epoch!=m_request.epoch)return fail(CaptureReason::Authority);
    if(m_lastCycle&&(c.cycle!=m_lastCycle+1ULL||c.timestampUs<=m_lastUs||
        c.timestampUs-m_lastUs>1000ULL))return fail(CaptureReason::Source);
    if(!m_firstCycle) {m_firstCycle=c.cycle;m_firstUs=c.timestampUs;}
    if(c.cycle<m_firstCycle||c.cycle-m_firstCycle>=m_request.maximumTicks||
        c.timestampUs<m_firstUs||c.timestampUs-m_firstUs>=m_request.maximumTicks*250ULL)
        return fail(CaptureReason::Timeout);
    m_cycle=c;m_cycleOpen=true;m_staged=false;
    if(State()==CaptureState::Armed)m_state.store(static_cast<std::uint32_t>(CaptureState::Running),std::memory_order_release);
    return true;
}
bool CaptureRecorder::RuntimeStage(CaptureSample s,double pulsesPerMm,bool inputValid) noexcept {
    if(!Active()||State()==CaptureState::Armed)return true;
    CaptureVelocityEvidence velocity{};
    const auto fail=[this,&s,pulsesPerMm,inputValid,&velocity](CaptureReason reason) noexcept {
        return RuntimeFailAt(reason,CaptureFaultStage::Stage,m_cycleOpen?&m_cycle:nullptr,
            &s,true,pulsesPerMm,inputValid,-1,velocity.present?&velocity:nullptr);
    };
    const auto cancelled=m_cancelReason.load(std::memory_order_acquire);
    if(cancelled)return fail(static_cast<CaptureReason>(cancelled));
    if(!m_cycleOpen||m_staged||!inputValid||!s.valid||!FiniteSample(s)||
        !std::isfinite(pulsesPerMm)||std::fabs(pulsesPerMm-m_request.pulsesPerMm)>
            std::fabs(m_request.pulsesPerMm)*1e-12)return fail(CaptureReason::Input);
    const bool motion=s.gainBank==static_cast<std::uint32_t>(GainBank::Motion);
    const bool idle=s.gainBank==static_cast<std::uint32_t>(GainBank::Idle);
    if((!motion&&!idle)||!SameGains(s,motion?m_request.motionGains:m_request.idleGains))
        return fail(CaptureReason::GainsChanged);
    // Ordinary MoveToPosition rebases an IDLE command to the actual position.
    // Permit only its admitted endpoint settle window. Q1 targets are A/B;
    // legacy targets remain within origin +/- excursion.
    if(m_request.pointToPoint) {
        const double end=m_request.originPulse+m_request.excursionPulse;
        const bool approach=m_request.approachEnabled&&!m_measurementActive;
        const double lower=approach?(std::min)(m_request.originPulse,m_request.approachStartPulse):m_request.originPulse;
        const double upper=approach?(std::max)(end,m_request.approachStartPulse):end;
        if(s.commandPulse<lower-m_request.settleWindowPulse-1.0||
            s.commandPulse>upper+m_request.settleWindowPulse+1.0||
            s.actualPulse<lower-m_request.boundaryMarginPulse||
            s.actualPulse>upper+m_request.boundaryMarginPulse)return fail(CaptureReason::Boundary);
    } else if(std::fabs(s.commandPulse-m_request.originPulse)>
            m_request.excursionPulse+m_request.settleWindowPulse+1.0||
        std::fabs(s.actualPulse-m_request.originPulse)>
            m_request.excursionPulse+m_request.boundaryMarginPulse)
        return fail(CaptureReason::Boundary);
    if(std::fabs(s.commandPulse-s.actualPulse)>m_request.followingLimitPulse)
        return fail(CaptureReason::Following);
    if(std::fabs(s.outputLogicalPps)>m_request.maximumVelocityPps+1.0 ||
        std::fabs(s.referencePps)>ReferenceVelocityLimitPps(m_request)+1.0)
        return fail(CaptureReason::Velocity);
    // TUNE04 FIX1: one-cycle encoder differences can contain short holding dither.
    // Preserve the original actual-velocity limit, use four continuous
    // position intervals once available, and never dilute it with a longer
    // observed timestamp interval. No smoothing touches command/output guards
    // or the raw evidence published to CSV. Startup remains the old raw test.
    velocity.present=true;
    velocity.limitPps=ReferenceVelocityLimitPps(m_request)*1.5+1.0/CapturePeriodSeconds;
    velocity.stepLimitPulse=velocity.limitPps*static_cast<double>(ActualVelocityWindowUs)*1e-6;
    velocity.rawStepPulse=s.actualVelocityPps*CapturePeriodSeconds;
    velocity.windowReady=m_rawCount>=ActualVelocityIntervals;
    velocity.intervals=static_cast<std::uint32_t>((std::min)(m_rawCount,static_cast<std::uint64_t>(ActualVelocityIntervals)));
    velocity.estimatedPps=s.actualVelocityPps;
    if(m_rawCount) {
        velocity.hasPositionStep=true;
        velocity.positionStepPulse=s.actualPulse-m_rawHistory[(m_rawCount-1ULL)%RawHistoryCapacity].actualPulse;
    }
    if(velocity.windowReady) {
        const auto& previous=m_rawHistory[(m_rawCount-ActualVelocityIntervals)%RawHistoryCapacity];
        if(m_cycle.timestampUs<=previous.timestampUs)return fail(CaptureReason::Source);
        velocity.elapsedUs=m_cycle.timestampUs-previous.timestampUs;
        velocity.estimationUs=(std::min)(velocity.elapsedUs,ActualVelocityWindowUs);
        velocity.estimatedPps=(s.actualPulse-previous.actualPulse)/
            (static_cast<double>(velocity.estimationUs)*1e-6);
    }
    // A single observed or raw-implied encoder jump may never exceed the
    // travel permitted for ONE full nominal window, even if it would cancel
    // out in the signed window estimate. This gate runs before output.
    if(!std::isfinite(velocity.positionStepPulse)||!std::isfinite(velocity.rawStepPulse)||
        !std::isfinite(velocity.estimatedPps))return fail(CaptureReason::Input);
    if((std::max)(std::fabs(velocity.positionStepPulse),std::fabs(velocity.rawStepPulse))>
        velocity.stepLimitPulse)return fail(CaptureReason::EncoderJump);
    if(std::fabs(velocity.estimatedPps)>velocity.limitPps)
        return fail(CaptureReason::ActualVelocity);
    s.session=m_request.session;s.cycle=m_cycle.cycle;s.timestampUs=m_cycle.timestampUs;
    m_pending=s;m_staged=true;return true;
}
bool CaptureRecorder::RuntimeAllowOutput(int axis,double logicalVelocityPps) noexcept {
    if(!Active()||State()==CaptureState::Armed||axis!=m_request.axis)return true;
    const auto fail=[this,axis,logicalVelocityPps](CaptureReason reason) noexcept {
        return RuntimeFailAt(reason,CaptureFaultStage::AllowOutput,m_cycleOpen?&m_cycle:nullptr,
            m_cycleOpen&&m_staged?&m_pending:nullptr,true,logicalVelocityPps,
            std::isfinite(logicalVelocityPps),axis);
    };
    const auto cancelled=m_cancelReason.load(std::memory_order_acquire);
    if(cancelled)return fail(static_cast<CaptureReason>(cancelled));
    if(!std::isfinite(logicalVelocityPps)||std::fabs(logicalVelocityPps)>m_request.maximumVelocityPps+1.0)
        return fail(CaptureReason::Velocity);
    // Early-return zero writes remain safe; End rejects the missing PI sample.
    if(logicalVelocityPps!=0.0&&(!m_cycleOpen||!m_staged))return fail(CaptureReason::MissingSample);
    return true;
}
void CaptureRecorder::PublishLatest(const CaptureSample& s) noexcept {
    std::uint64_t words[LatestWords]{};
    std::memcpy(words,&s,sizeof(s));
    const auto sequence=m_latestSequence.load(std::memory_order_seq_cst);
    m_latestSequence.store(sequence+1ULL,std::memory_order_seq_cst);
    for(std::size_t i=0U;i<LatestWords;++i)m_latest[i].store(words[i],std::memory_order_seq_cst);
    m_latestSequence.store(sequence+2ULL,std::memory_order_seq_cst);
}
bool CaptureRecorder::TryReadLatest(CaptureSample& s) const noexcept {
    for(unsigned attempt=0U;attempt<3U;++attempt) {
        const auto before=m_latestSequence.load(std::memory_order_seq_cst);
        if(!before||(before&1ULL))continue;
        std::uint64_t words[LatestWords]{};
        for(std::size_t i=0U;i<LatestWords;++i)words[i]=m_latest[i].load(std::memory_order_seq_cst);
        if(m_latestSequence.load(std::memory_order_seq_cst)!=before)continue;
        std::memcpy(&s,words,sizeof(s));return true;
    }
    return false;
}
void CaptureRecorder::SetTelemetryLeg(std::uint64_t session,std::uint64_t legIndex,
    std::uint64_t firstCycle,double startPulse,double targetPulse) noexcept {
    if(!Active()||session!=m_request.session||!session)return;
    const bool valid=legIndex&&legIndex<=128ULL&&firstCycle&&std::isfinite(startPulse)&&
        std::isfinite(targetPulse)&&startPulse!=targetPulse;
    if(valid&&legIndex==m_legPublishedIndex)return;
    // A malformed description for this capture retires overshoot coverage.
    // Wrong-session calls are ignored above; this marker has no motion effect.
    std::uint64_t words[5]={session,valid?legIndex:0ULL,firstCycle,0ULL,0ULL};
    std::memcpy(&words[3],&startPulse,sizeof(startPulse));std::memcpy(&words[4],&targetPulse,sizeof(targetPulse));
    const auto odd=(m_legSequence.load(std::memory_order_seq_cst)+1ULL)|1ULL;
    m_legSequence.store(odd,std::memory_order_seq_cst);
    for(std::size_t i=0U;i<5U;++i)m_legWords[i].store(words[i],std::memory_order_seq_cst);
    m_legSequence.store(odd+1ULL,std::memory_order_seq_cst);m_legPublishedIndex=valid?legIndex:0ULL;
}
void CaptureRecorder::ObserveTelemetryLeg() noexcept {
    // A bounded atomic observation, never an admission/command channel. A
    // delayed descriptor cannot be applied retroactively to an earlier cycle.
    for(unsigned attempt=0U;attempt<3U;++attempt) {
        const auto sequence=m_legSequence.load(std::memory_order_seq_cst);
        if(!sequence||(sequence&1ULL)||sequence==m_legConsumedSequence)break;
        std::uint64_t words[5]{};
        for(std::size_t i=0U;i<5U;++i)words[i]=m_legWords[i].load(std::memory_order_seq_cst);
        if(m_legSequence.load(std::memory_order_seq_cst)!=sequence)continue;
        if(words[0]!=m_request.session||!words[1]||words[1]>128ULL||!words[2]) {
            m_liveOvershootCoverage=false;m_legConsumedSequence=sequence;break;
        }
        if(m_pending.cycle<words[2])break;
        double start=0.0,target=0.0;
        std::memcpy(&start,&words[3],sizeof(start));std::memcpy(&target,&words[4],sizeof(target));
        if(!std::isfinite(start)||!std::isfinite(target)||start==target||
            words[1]!=m_liveLegIndex+1ULL||
            (m_liveLegIndex&&(!m_liveLegEndpointSeen||std::fabs(start-m_liveTargetPulse)>1.0)))
            m_liveOvershootCoverage=false;
        m_liveLegIndex=words[1];m_liveLegFirstCycle=words[2];
        m_liveStartPulse=start;m_liveTargetPulse=target;m_liveLegEndpointSeen=false;
        m_legConsumedSequence=sequence;break;
    }
    const bool moving=m_pending.gainBank==static_cast<std::uint32_t>(GainBank::Motion)&&
        std::fabs(m_pending.referencePps)>1.0;
    if(!m_liveLegIndex) {if(moving)m_liveOvershootCoverage=false;return;}
    if(m_pending.cycle<m_liveLegFirstCycle)return;
    const double direction=m_liveTargetPulse>m_liveStartPulse?1.0:-1.0;
    if(moving&&m_pending.referencePps*direction<0.0)m_liveOvershootCoverage=false;
    const double beyond=direction*(m_pending.actualPulse-m_liveTargetPulse);
    if(!std::isfinite(beyond)){m_liveOvershootCoverage=false;return;}
    m_liveOvershootPulse=(std::max)(m_liveOvershootPulse,(std::max)(0.0,beyond));
    if(!m_liveLegEndpointSeen&&std::fabs(m_pending.commandPulse-m_liveTargetPulse)<=1.0&&
        std::fabs(m_pending.referencePps)<=1.0) {
        m_liveLegEndpointSeen=true;++m_liveEndpointCount;
    }
}
void CaptureRecorder::PublishLiveTelemetry(bool terminal,bool complete) noexcept {
    CaptureLiveTelemetry live{};live.session=m_request.session;
    const auto& s=m_statistics;
    live.sampleCount=s.sampleCount;live.movingSampleCount=s.movingSampleCount;
    live.cycle=s.lastCycle;live.timestampUs=s.lastTimestampUs;live.firstTimestampUs=s.firstTimestampUs;
    live.observedEndpointCount=m_liveEndpointCount;
    if(s.sampleCount)live.flags|=1ULL;
    if(s.valid&&s.sampleCount) {
        live.flags|=2ULL;live.currentFollowingErrorPulse=m_liveCurrentErrorPulse;
        live.peakFollowingErrorPulse=s.peakFollowingErrorPulse;
        if(s.movingSampleCount){live.flags|=4ULL;live.meanSquaredMovingFollowingErrorPulse=s.meanSquaredMovingFollowingErrorPulse;}
        if(m_liveOvershootCoverage&&m_liveEndpointCount) {live.flags|=8ULL;live.maximumOvershootPulse=m_liveOvershootPulse;}
        if(s.cruisePositive.sampleCount||s.cruiseNegative.sampleCount) {
            live.flags|=128ULL;live.cruisePeakPulse=(std::max)(s.cruisePositive.peakAbsoluteErrorPulse,s.cruiseNegative.peakAbsoluteErrorPulse);
        }
    }
    if(terminal)live.flags|=32ULL;
    if(s.anySaturated)live.flags|=64ULL;
    if(complete)live.flags|=256ULL;
    std::uint64_t words[LiveWords]{};std::memcpy(words,&live,sizeof(live));
    const auto odd=(m_liveSequence.load(std::memory_order_seq_cst)+1ULL)|1ULL;
    m_liveSequence.store(odd,std::memory_order_seq_cst);
    for(std::size_t i=0U;i<LiveWords;++i)m_liveWords[i].store(words[i],std::memory_order_seq_cst);
    m_liveSequence.store(odd+1ULL,std::memory_order_seq_cst);m_livePublishedCount=s.sampleCount;
}
bool CaptureRecorder::TryReadLiveTelemetry(CaptureLiveTelemetry& live) const noexcept {
    for(unsigned attempt=0U;attempt<3U;++attempt) {
        const auto sequence=m_liveSequence.load(std::memory_order_seq_cst);
        if(!sequence||(sequence&1ULL))continue;
        std::uint64_t words[LiveWords]{};
        for(std::size_t i=0U;i<LiveWords;++i)words[i]=m_liveWords[i].load(std::memory_order_seq_cst);
        if(m_liveSequence.load(std::memory_order_seq_cst)!=sequence)continue;
        std::memcpy(&live,words,sizeof(live));return true;
    }
    return false;
}
bool CaptureRecorder::RuntimeEnd(double imageLogicalVelocityPps,bool imageValid) noexcept {
    if(!Active()||State()==CaptureState::Armed)return true;
    const auto fail=[this,imageLogicalVelocityPps,imageValid](CaptureReason reason) noexcept {
        return RuntimeFailAt(reason,CaptureFaultStage::End,m_cycleOpen?&m_cycle:nullptr,
            m_cycleOpen&&m_staged?&m_pending:nullptr,true,imageLogicalVelocityPps,imageValid);
    };
    const auto cancelled=m_cancelReason.load(std::memory_order_acquire);
    if(cancelled)return fail(static_cast<CaptureReason>(cancelled));
    if(!m_cycleOpen||!m_staged)return fail(CaptureReason::MissingSample);
    if(!imageValid||!std::isfinite(imageLogicalVelocityPps))return fail(CaptureReason::Output);
    if(std::fabs(imageLogicalVelocityPps)>m_request.maximumVelocityPps+1.0)return fail(CaptureReason::Velocity);
    // A later guard changed the selected command: keep the actual local image
    // but do not let a scrubbed/saturated trajectory look like valid plant data.
    m_pending.saturated=m_pending.saturated||std::fabs(imageLogicalVelocityPps-m_pending.outputLogicalPps)>1.0;
    m_pending.outputLogicalPps=imageLogicalVelocityPps;
    // TUNE03 FIX2: raw encoder velocity is a one-cycle derivative. Normal
    // IDLE holding dither can exceed its one-pulse floor despite a stationary
    // target and bounded position error. Keep that raw value; the independent
    // TUNE04 FIX1 velocity/jump guards above do not qualify settling. Use IDLE
    // position evidence and drift over eighty intervals (nominal 20 ms).
    // A complete window requires 81 eligible samples, so it never reaches
    // back across a moving state, command change or failed position window.
    const bool eligible=m_rawCount!=0ULL&&m_pending.motionState==0U&&
        m_pending.gainBank==static_cast<std::uint32_t>(GainBank::Idle)&&
        std::fabs(m_pending.referencePps)<=1e-9&&
        m_pending.commandPulse==m_previousCommand&&
        std::fabs(m_pending.commandPulse-m_pending.actualPulse)<=m_request.settleWindowPulse;
    // TUNE05: a brief drift estimate may fluctuate during otherwise bounded
    // holding dither. Keep the initial settling proof unchanged, but track the
    // subsequent fixed-command, in-window IDLE observation independently.
    // Every per-cycle eligibility break resets this duration; NC cannot miss
    // a short excursion between its slower scans.
    if(!eligible)m_idleWindowSinceUs=0ULL;
    else if(!m_idleWindowSinceUs)m_idleWindowSinceUs=m_pending.timestampUs;
    m_pending.idleWindowDurationUs=m_idleWindowSinceUs?
        m_pending.timestampUs-m_idleWindowSinceUs:0ULL;
    bool settled=false;
    if(!eligible) m_settleEligibleSamples=0U;
    else {
        if(!m_settleEligibleSamples)
            m_settleMinimumPulse=m_settleMaximumPulse=m_pending.actualPulse;
        else {
            m_settleMinimumPulse=(std::min)(m_settleMinimumPulse,m_pending.actualPulse);
            m_settleMaximumPulse=(std::max)(m_settleMaximumPulse,m_pending.actualPulse);
        }
        if(m_settleEligibleSamples<=SettleDriftIntervals)++m_settleEligibleSamples;
        const double range=m_settleMaximumPulse-m_settleMinimumPulse;
        // The configured position window is +/- settleWindowPulse about one
        // fixed command; its full peak-to-peak extent is twice that value.
        // IDLE RMS/jitter quality limits remain separate analysis checks.
        if(!std::isfinite(range)||range*0.5>m_request.settleWindowPulse)
            m_settleEligibleSamples=0U;
        else if(m_settleEligibleSamples>SettleDriftIntervals&&m_rawCount>=SettleDriftIntervals) {
            const CaptureSample& previous=m_rawHistory[(m_rawCount-SettleDriftIntervals)%RawHistoryCapacity];
            const std::uint64_t elapsedUs=m_pending.timestampUs>previous.timestampUs?
                m_pending.timestampUs-previous.timestampUs:0ULL;
            const double displacement=std::fabs(m_pending.actualPulse-previous.actualPulse);
            const double driftPps=elapsedUs?displacement/(static_cast<double>(elapsedUs)*1e-6):
                (std::numeric_limits<double>::infinity)();
            settled=std::isfinite(driftPps)&&driftPps<=m_request.settleVelocityPps;
            if(!settled)m_settleEligibleSamples=0U;
        }
    }
    if(settled) {
        // Count only time after a complete drift window passed, rather than
        // retroactively treating its warm-up as settled observation time.
        if(!m_settledCycles)m_settledSinceUs=m_pending.timestampUs;
        if(m_settledCycles!=(std::numeric_limits<std::uint32_t>::max)())++m_settledCycles;
    } else {m_settledCycles=0U;m_settledSinceUs=0ULL;}
    m_pending.settledCycles=m_settledCycles;
    m_pending.settledDurationUs=m_settledSinceUs?m_pending.timestampUs-m_settledSinceUs:0ULL;
    // One RT-only handoff. The sample that publishes this boundary has passed
    // the endpoint/tracking guard shared by approach and the measurement.
    // Do not reset the sample count, total clock or velocity history.
    const bool beginMeasurement=!m_measurementActive&&m_request.approachEnabled&&
        m_measurementRequestSession.load(std::memory_order_acquire)==m_request.session&&
        eligible&&m_pending.settledDurationUs>=m_request.measurementSettleHoldUs&&
        std::fabs(m_pending.commandPulse-m_request.originPulse)<=1.0;
    if(beginMeasurement) {
        m_measurementActive=true;m_storageAnchorRawCount=m_rawCount;
        m_measurementBoundary.session=m_request.session;m_measurementBoundary.sampleOffset=m_count;
        m_measurementBoundary.cycle=m_pending.cycle;m_measurementBoundary.timestampUs=m_pending.timestampUs;
        m_measurementBoundary.approachDurationUs=m_pending.timestampUs-m_firstUs;
        m_liveOvershootPulse=m_liveCurrentErrorPulse=0.0;m_liveEndpointCount=0ULL;
        m_liveLegEndpointSeen=false;m_liveOvershootCoverage=true;
    }
    const bool store=m_request.sampleStride==1U||beginMeasurement||
        (m_rawCount-m_storageAnchorRawCount)%m_request.sampleStride==0ULL;
    if(store&&m_count>=CaptureCapacity)return fail(CaptureReason::Capacity);
    if(m_measurementActive) {
        auto& statistics=m_statistics;
        if(!statistics.sampleCount) {
            statistics.valid=true;statistics.firstCycle=m_pending.cycle;
            statistics.firstTimestampUs=m_pending.timestampUs;
        }
        ++statistics.sampleCount;
        m_liveCurrentErrorPulse=m_pending.commandPulse-m_pending.actualPulse;
        ObserveTelemetryLeg();
        statistics.lastCycle=m_pending.cycle;statistics.lastTimestampUs=m_pending.timestampUs;
        statistics.anySaturated=statistics.anySaturated||m_pending.saturated!=0U;
        const bool moving=m_pending.gainBank==static_cast<std::uint32_t>(GainBank::Motion)&&
            std::fabs(m_pending.referencePps)>1.0;
        if(moving)++statistics.movingSampleCount;
        const double error=std::fabs(m_pending.commandPulse-m_pending.actualPulse);
        const double errorSquared=error*error;
        if(!std::isfinite(error)||!std::isfinite(errorSquared))statistics.valid=false;
        if(statistics.valid) {
            statistics.peakFollowingErrorPulse=(std::max)(statistics.peakFollowingErrorPulse,error);
            statistics.peakReferencePps=(std::max)(statistics.peakReferencePps,std::fabs(m_pending.referencePps));
            statistics.meanSquaredFollowingErrorPulse+=(errorSquared-statistics.meanSquaredFollowingErrorPulse)/
                static_cast<double>(statistics.sampleCount);
            if(moving)statistics.meanSquaredMovingFollowingErrorPulse+=
                (errorSquared-statistics.meanSquaredMovingFollowingErrorPulse)/static_cast<double>(statistics.movingSampleCount);
            if(!std::isfinite(statistics.meanSquaredFollowingErrorPulse)||
                !std::isfinite(statistics.meanSquaredMovingFollowingErrorPulse))statistics.valid=false;
            // Every raw moving sample is assigned exactly once. A stable
            // requested-speed plateau is qualified from command evidence;
            // acceleration, deceleration and plateau warm-up stay separate.
            // No CSV stride or encoder derivative can hide a cruise spike.
            if(moving&&m_request.cruiseVelocityPps>0.0) {
                const int direction=m_pending.referencePps>0.0?1:-1;
                if(direction!=m_trackingDirection) {
                    m_trackingDirection=direction;m_cruiseStableSinceUs=m_cruiseObservedSinceUs=0ULL;
                    m_cruiseLegQualified=false;
                }
                const double reference=std::fabs(m_pending.referencePps);
                const double targetTolerance=(std::max)(1e-9,m_request.cruiseVelocityPps*CruiseTargetRelativeTolerance);
                const double stableTolerance=(std::max)(1e-9,m_request.cruiseVelocityPps*CruiseStableRelativeTolerance);
                const bool atRequestedSpeed=std::fabs(reference-m_request.cruiseVelocityPps)<=targetTolerance;
                if(atRequestedSpeed) {
                    if(!m_cruiseStableSinceUs) {
                        m_cruiseStableSinceUs=m_pending.timestampUs;
                        m_cruiseReferenceMin=m_cruiseReferenceMax=reference;
                    } else {
                        m_cruiseReferenceMin=(std::min)(m_cruiseReferenceMin,reference);
                        m_cruiseReferenceMax=(std::max)(m_cruiseReferenceMax,reference);
                        if(m_cruiseReferenceMax-m_cruiseReferenceMin>stableTolerance) {
                            m_cruiseStableSinceUs=m_pending.timestampUs;
                            m_cruiseReferenceMin=m_cruiseReferenceMax=reference;
                        }
                    }
                } else m_cruiseStableSinceUs=0ULL;
                const bool cruise=m_cruiseStableSinceUs&&
                    m_pending.timestampUs-m_cruiseStableSinceUs>=CruiseStableWindowUs;
                auto& moments=cruise?(direction>0?statistics.cruisePositive:statistics.cruiseNegative):
                    statistics.nonCruiseMoving;
                const std::uint64_t elapsedUs=statistics.sampleCount>1ULL?m_pending.timestampUs-m_lastUs:0ULL;
                if(!AddTrackingMoment(moments,m_pending.commandPulse-m_pending.actualPulse,elapsedUs))
                    statistics.valid=false;
                if(cruise) {
                    if(!m_cruiseObservedSinceUs)m_cruiseObservedSinceUs=m_pending.timestampUs;
                    if(!m_cruiseLegQualified&&
                        m_pending.timestampUs-m_cruiseObservedSinceUs>=CruiseMinimumObservedUs) {
                        ++moments.qualifiedLegCount;m_cruiseLegQualified=true;
                    }
                } else m_cruiseObservedSinceUs=0ULL;
            } else {
                m_trackingDirection=0;m_cruiseStableSinceUs=m_cruiseObservedSinceUs=0ULL;
                m_cruiseLegQualified=false;
            }
        }
    }
    m_previousCommand=m_pending.commandPulse;
    m_rawHistory[m_rawCount%RawHistoryCapacity]=m_pending;++m_rawCount;
    if(store)m_samples[m_count++]=m_pending;
    PublishLatest(m_pending);
    m_lastCycle=m_cycle.cycle;m_lastUs=m_cycle.timestampUs;m_cycleOpen=false;m_staged=false;
    m_publishedCount.store(m_count,std::memory_order_release);
    if(beginMeasurement)m_measurementBoundaryPublished.store(true,std::memory_order_release);
    const bool finishing=m_finish.load(std::memory_order_acquire);
    if(finishing) {
        m_statistics.complete=m_measurementActive&&m_statistics.sampleCount!=0ULL;
        PublishLiveTelemetry(true,m_statistics.complete);
        m_state.store(static_cast<std::uint32_t>(CaptureState::Complete),std::memory_order_release);
    } else if(beginMeasurement||(m_measurementActive&&
        (!m_livePublishedCount||m_statistics.sampleCount-m_livePublishedCount>=40ULL)))
        PublishLiveTelemetry(false,false);
    return true;
}
const char* CaptureReasonName(CaptureReason r) noexcept {
    switch(r) {
    case CaptureReason::None:return "NONE";case CaptureReason::Cancelled:return "CANCELLED";
    case CaptureReason::Configuration:return "CONFIGURATION";case CaptureReason::Source:return "SOURCE";
    case CaptureReason::Clock:return "CLOCK";case CaptureReason::Authority:return "AUTHORITY";
    case CaptureReason::Input:return "INPUT";case CaptureReason::GainsChanged:return "GAINS_CHANGED";
    case CaptureReason::Boundary:return "BOUNDARY";case CaptureReason::Following:return "FOLLOWING";
    case CaptureReason::Velocity:return "VELOCITY";case CaptureReason::Capacity:return "CAPACITY";
    case CaptureReason::Timeout:return "TIMEOUT";case CaptureReason::MissingSample:return "MISSING_SAMPLE";
    case CaptureReason::Output:return "OUTPUT";case CaptureReason::Interference:return "INTERFERENCE";
    case CaptureReason::ActualVelocity:return "ACTUAL_VELOCITY";case CaptureReason::EncoderJump:return "ENCODER_JUMP";
    default:return "UNKNOWN";
    }
}
const char* CaptureFaultStageName(CaptureFaultStage stage) noexcept {
    switch(stage) {
    case CaptureFaultStage::None:return "NONE";
    case CaptureFaultStage::Begin:return "BEGIN";
    case CaptureFaultStage::Stage:return "STAGE";
    case CaptureFaultStage::AllowOutput:return "ALLOW_OUTPUT";
    case CaptureFaultStage::End:return "END";
    case CaptureFaultStage::External:return "EXTERNAL";
    default:return "UNKNOWN";
    }
}
}
