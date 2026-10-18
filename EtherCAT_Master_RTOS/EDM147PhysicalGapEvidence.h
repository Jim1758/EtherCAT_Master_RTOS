#pragma once
#include "EDMCoordinatedGapSource.h"
#include "EDMSixAxisGapModeCoordinator.h"

// Observes only images already committed by the existing six-axis mapper.
// This receipt cannot move, stop, authorize, or refresh an AD sample.
namespace EDM147
{
enum class Event : std::uint8_t { None, Ready, Reverse, BackCross, ForwardCross, RestoreReady, Recovered, Interrupted, Invalid };
enum class Reason : std::uint8_t { None, Configuration, Source, Geometry, Clock, Sequence, Overflow };
enum Milestone : std::uint32_t { HistoryReady=1U, ReverseMapped=2U, BackCrossed=4U, ForwardCrossed=8U, RecoveryMapped=16U };
inline const char* EventName(Event event) noexcept
{
    switch(event) {
    case Event::None:return "NONE";case Event::Ready:return "HISTORY_READY";
    case Event::Reverse:return "REVERSE_MAPPED";case Event::BackCross:return "BACK_CROSS";
    case Event::ForwardCross:return "FORWARD_CROSS";case Event::RestoreReady:return "RESTORE_READY";case Event::Recovered:return "RECOVERED";
    case Event::Interrupted:return "INTERRUPTED";case Event::Invalid:return "INVALID";
    } return "UNKNOWN";
}
inline const char* ReasonName(Reason reason) noexcept
{
    switch(reason) {
    case Reason::None:return "NONE";case Reason::Configuration:return "CONFIGURATION";
    case Reason::Source:return "SOURCE";case Reason::Geometry:return "GEOMETRY";
    case Reason::Clock:return "CLOCK";case Reason::Sequence:return "SEQUENCE";
    case Reason::Overflow:return "OVERFLOW";
    } return "UNKNOWN";
}
struct Snapshot
{
    std::uint64_t session=0ULL,revision=0ULL,tick=0ULL,nowUs=0ULL,sourceSequence=0ULL,sourceCapturedUs=0ULL;
    std::uint64_t eventTick=0ULL,eventNowUs=0ULL;
    std::uint64_t cyclesStarted=0ULL,cyclesCompleted=0ULL,qualifiedCycles=0ULL,interruptedCycles=0ULL;
    std::uint64_t gapBackwardCrossings=0ULL,gapForwardCrossings=0ULL,reverseMappedSamples=0ULL,forwardMappedSamples=0ULL;
    std::uint64_t cycleBackwardCrossings=0ULL,cycleForwardCrossings=0ULL;
    double referenceV=0.0,voltageV=0.0,targetMmSec=0.0,stationMm=0.0,localMm=0.0,anchorStationMm=0.0;
    unsigned row=0U,cursorRow=0U,historyDepth=0U,maximumGapDepth=0U,geometryKind=0U;
    unsigned retreatAxisMask=0U,returnAxisMask=0U,retreatKindMask=0U,returnKindMask=0U;
    unsigned cycleRetreatAxisMask=0U,cycleReturnAxisMask=0U,cycleRetreatKindMask=0U,cycleReturnKindMask=0U;
    std::uint32_t milestones=0U;
    int direction=0;
    EDM128::Phase phase=EDM128::Phase::Gap;
    Event event=Event::None;
    Reason reason=Reason::None;
    bool begun=false,valid=false,cycleActive=false,historyReady=false,restoreReady=false;
};
static_assert(sizeof(Snapshot)<=512U && std::is_trivially_copyable<Snapshot>::value,
    "EDM147 evidence is bounded, numeric, and grants no output authority.");
bool ReadEvidence(Snapshot& evidence) noexcept;

class Collector
{
public:
    bool Begin(std::uint64_t session,const EDM88::Identity& identity,double referenceV) noexcept
    {
        *this=Collector{};
        EDM88::SourceConfig config{};config.enabled=true;config.identity=identity;
        config.samplePeriodUs=250ULL;config.maxAgeUs=EDM126::MaximumSourceAgeUs;
        if(!session||!EDM88::ValidPhysical(config)||!std::isfinite(referenceV)||referenceV<15.0||referenceV>200.0)
        { Invalidate(Reason::Configuration);return false; }
        e_.session=session;e_.referenceV=referenceV;e_.begun=e_.valid=true;identity_=identity;return true;
    }
    const Snapshot& Current() const noexcept { return e_; }
    void Observe(const EDM126::Feedback& f,const EDM88::Frame& source,EDM128::Phase phase,
        double targetMmSec,int direction,bool stopping=false,unsigned geometryKind=0U) noexcept
    {
        if(!e_.begun||!e_.valid)return;
        if(!ValidSource(f,source)) { Invalidate(Reason::Source);return; }
        if(!ValidGeometry(f,phase,targetMmSec,direction,geometryKind)) { Invalidate(Reason::Geometry);return; }
        if(scopeBound_&&!EDM109::SameScope(scope_,f.authorizedSource.scope)) { Invalidate(Reason::Sequence);return; }
        if(previousValid_) {
            const auto& old=previous_.authorizedSource;const auto& current=f.authorizedSource;
            if(current.ordinal==old.ordinal ? !EDM126::SameSource(current,old) :
                (current.ordinal!=old.ordinal+1ULL||current.dispatch<=old.dispatch||current.epoch<=old.epoch||
                    current.segment<=old.segment||current.sourcePC<=old.sourcePC))
            { Invalidate(Reason::Sequence);return; }
        }
        if(previousValid_ && (f.tick<previous_.tick||f.nowUs<previous_.nowUs)) { Invalidate(Reason::Clock);return; }
        if(previousValid_ && (source.sequence<previousSource_.sequence || source.capturedAtUs<previousSource_.capturedAtUs ||
            (source.sequence==previousSource_.sequence&&!EDM88::SameFrame(source,previousSource_)) ||
            (source.sequence>previousSource_.sequence&&source.capturedAtUs<=previousSource_.capturedAtUs)))
        { Invalidate(Reason::Sequence);return; }
        if(previousValid_&&f.tick==previous_.tick) {
            if(f.nowUs!=previous_.nowUs||!SameImage(f,previous_)||phase!=previousPhase_||stopping!=previousStopping_)
                Invalidate(Reason::Clock);
            return; // Re-observing one commit cannot add a sample or an edge.
        }
        if(!scopeBound_) {scope_=f.authorizedSource.scope;scopeBound_=true;}
        Stamp(f,source,phase,targetMmSec,direction,geometryKind);
        const bool sameRow=previousValid_&&EDM126::SameSource(f.authorizedSource,previous_.authorizedSource);
        const bool contiguous=sameRow&&previous_.tick!=(std::numeric_limits<std::uint64_t>::max)()&&f.tick==previous_.tick+1ULL&&
            f.nowUs>previous_.nowUs&&f.nowUs-previous_.nowUs<=EDM126::MaximumSourceAgeUs;
        const bool gap=phase==EDM128::Phase::Gap&&!stopping;
        const bool pair=contiguous&&gap&&previousPhase_==EDM128::Phase::Gap&&!previousStopping_;
        if(e_.cycleActive&&!pair) Interrupt();
        e_.historyReady=gap&&f.historyCount>=2U&&f.historyDepth==0U&&!f.completeReady&&!f.atCurrentEnd&&
            !f.awaitingSeam&&f.velocityMmSec>0.0&&direction==1&&targetMmSec>0.0&&source.voltageV>e_.referenceV+5.0;
        if(e_.historyReady&&!(e_.milestones&HistoryReady)) { e_.milestones|=HistoryReady;Mark(Event::Ready); }
        if(pair) Classify(f,source,targetMmSec,direction,geometryKind);
        previous_=f;previousSource_=source;previousPhase_=phase;previousStopping_=stopping;previousValid_=true;
    }
private:
    Snapshot e_{};
    EDM88::Identity identity_{};
    EDM126::Feedback previous_{};
    EDM88::Frame previousSource_{};
    EDM126::Source cycleSource_{};
    EDM109::Scope scope_{};
    EDM128::Phase previousPhase_=EDM128::Phase::Gap;
    bool previousValid_=false,previousStopping_=false,scopeBound_=false;
    static bool SameImage(const EDM126::Feedback& a,const EDM126::Feedback& b) noexcept
    {
        return EDM126::SameSource(a.authorizedSource,b.authorizedSource)&&EDM126::SameSource(a.cursorSource,b.cursorSource)&&
            a.stationMm==b.stationMm&&a.localMm==b.localMm&&a.lengthMm==b.lengthMm&&a.velocityMmSec==b.velocityMmSec&&
            a.positionPulse==b.positionPulse&&a.velocityPps==b.velocityPps&&a.accelerationPps2==b.accelerationPps2&&
            a.backwardCrossings==b.backwardCrossings&&a.forwardCrossings==b.forwardCrossings&&a.historyDepth==b.historyDepth;
    }
    bool ValidSource(const EDM126::Feedback& f,const EDM88::Frame& source) const noexcept
    {
        return source.valid&&EDM88::SameIdentity(source.identity,identity_)&&source.sequence&&source.capturedAtUs&&
            std::isfinite(source.voltageV)&&source.voltageV>=0.0&&source.voltageV<=300.0&&f.nowUs>=source.capturedAtUs&&
            f.nowUs-source.capturedAtUs<=EDM126::MaximumSourceAgeUs;
    }
    bool ValidGeometry(const EDM126::Feedback& f,EDM128::Phase phase,double target,int direction,unsigned kind) const noexcept
    {
        if(!f.tick||!f.nowUs||!f.active||f.faulted||!EDM126::ValidSource(f.authorizedSource)||!EDM126::ValidSource(f.cursorSource)||
            f.authorizedSource.scope.session!=e_.session||!EDM109::SameScope(f.authorizedSource.scope,f.cursorSource.scope)||
            f.authorizedSource.ordinal>=16ULL||f.cursorSource.ordinal>f.authorizedSource.ordinal||
            f.historyDepth!=f.authorizedSource.ordinal-f.cursorSource.ordinal||f.historyCount>EDM126::HistoryCapacity||
            f.historyDepth>f.historyCount||!std::isfinite(f.localMm)||!EDM126::Positive(f.lengthMm)||
            f.localMm<0.0||f.localMm>f.lengthMm||!std::isfinite(f.stationMm)||f.stationMm<0.0||
            !std::isfinite(f.velocityMmSec)||!std::isfinite(f.accelerationMmSec2)||
            !std::isfinite(target)||target>0.12/60.0||target< -0.03/60.0||direction< -1||direction>1||
            (target>0.0&&direction!=1)||(target<0.0&&direction!=-1)||kind<1U||kind>3U||
            static_cast<unsigned>(phase)>static_cast<unsigned>(EDM128::Phase::Fault))return false;
        for(unsigned axis=0U;axis<6U;++axis)if(!std::isfinite(f.positionPulse[axis])||!std::isfinite(f.velocityPps[axis])||
            !std::isfinite(f.accelerationPps2[axis]))return false;
        return true;
    }
    void Stamp(const EDM126::Feedback& f,const EDM88::Frame& source,EDM128::Phase phase,double target,int direction,unsigned kind) noexcept
    {
        e_.tick=f.tick;e_.nowUs=f.nowUs;e_.sourceSequence=source.sequence;e_.sourceCapturedUs=source.capturedAtUs;
        e_.voltageV=source.voltageV;e_.targetMmSec=target;e_.stationMm=f.stationMm;e_.localMm=f.localMm;
        e_.row=static_cast<unsigned>(f.authorizedSource.ordinal);e_.cursorRow=static_cast<unsigned>(f.cursorSource.ordinal);
        e_.historyDepth=f.historyDepth;e_.geometryKind=kind;e_.phase=phase;e_.direction=direction;
    }
    bool Increment(std::uint64_t& value,std::uint64_t amount=1ULL) noexcept
    {
        if(amount>(std::numeric_limits<std::uint64_t>::max)()-value) { Invalidate(Reason::Overflow);return false; }
        value+=amount;return true;
    }
    void Mark(Event event) noexcept
    {
        if(e_.revision==(std::numeric_limits<std::uint64_t>::max)()) {e_.valid=false;e_.reason=Reason::Overflow;return;}
        ++e_.revision;e_.event=event;e_.eventTick=e_.tick;e_.eventNowUs=e_.nowUs;
    }
    void Interrupt() noexcept
    {
        if(!e_.cycleActive)return;
        e_.cycleActive=e_.restoreReady=false;
        if(Increment(e_.interruptedCycles))Mark(Event::Interrupted);
    }
    void Invalidate(Reason reason) noexcept
    {
        e_.valid=false;if(e_.reason==Reason::None)e_.reason=reason;
        Interrupt();Mark(Event::Invalid);
    }
    void Classify(const EDM126::Feedback& f,const EDM88::Frame& source,double target,int direction,unsigned kind) noexcept
    {
        if(f.backwardCrossings<previous_.backwardCrossings||f.forwardCrossings<previous_.forwardCrossings)
        { Invalidate(Reason::Sequence);return; }
        const auto back=f.backwardCrossings-previous_.backwardCrossings,forward=f.forwardCrossings-previous_.forwardCrossings;
        if(back>1ULL||forward>1ULL||(back&&forward)||(back&&(target>=0.0||direction!=-1))||
            (forward&&(target<=0.0||direction!=1))) { Invalidate(Reason::Sequence);return; }
        const bool reverse=target<0.0&&direction==-1&&source.voltageV<e_.referenceV&&f.stationMm<previous_.stationMm;
        const bool advancing=target>0.0&&direction==1&&source.voltageV>e_.referenceV+5.0&&f.stationMm>previous_.stationMm;
        if(reverse) {
            if(!Increment(e_.reverseMappedSamples))return;
            if(!e_.cycleActive&&previous_.historyCount>=2U&&previous_.historyDepth==0U) {
                if(!Increment(e_.cyclesStarted))return;
                e_.cycleActive=true;e_.restoreReady=false;e_.anchorStationMm=previous_.stationMm;cycleSource_=f.authorizedSource;
                e_.cycleBackwardCrossings=e_.cycleForwardCrossings=0ULL;
                e_.cycleRetreatAxisMask=e_.cycleReturnAxisMask=e_.cycleRetreatKindMask=e_.cycleReturnKindMask=0U;
                e_.milestones|=ReverseMapped;Mark(Event::Reverse);
            }
        }
        if(advancing&&!Increment(e_.forwardMappedSamples))return;
        if(!Increment(e_.gapBackwardCrossings,back)||!Increment(e_.gapForwardCrossings,forward))return;
        if(back)e_.milestones|=BackCrossed;
        if(forward)e_.milestones|=ForwardCrossed;
        e_.maximumGapDepth=(std::max)(e_.maximumGapDepth,f.historyDepth);
        if(!e_.cycleActive)return;
        if(!EDM126::SameSource(cycleSource_,f.authorizedSource)) { Interrupt();return; }
        if(!Increment(e_.cycleBackwardCrossings,back)||!Increment(e_.cycleForwardCrossings,forward))return;
        unsigned moved=0U;
        for(unsigned axis=0U;axis<6U;++axis)if(f.positionPulse[axis]!=previous_.positionPulse[axis])moved|=1U<<axis;
        if(reverse) {e_.cycleRetreatAxisMask|=moved;e_.retreatAxisMask|=moved;e_.cycleRetreatKindMask|=1U<<(kind-1U);e_.retreatKindMask|=1U<<(kind-1U);}
        if(advancing) {e_.cycleReturnAxisMask|=moved;e_.returnAxisMask|=moved;e_.cycleReturnKindMask|=1U<<(kind-1U);e_.returnKindMask|=1U<<(kind-1U);}
        if(!e_.restoreReady&&e_.cycleBackwardCrossings>0ULL&&f.historyDepth>0U&&
            e_.cycleRetreatAxisMask==63U&&(e_.cycleRetreatKindMask&6U)!=0U) {
            e_.restoreReady=true;Mark(Event::RestoreReady);
        }
        if(advancing&&f.historyDepth==0U&&f.stationMm>=e_.anchorStationMm&&e_.cycleBackwardCrossings&&
            e_.cycleBackwardCrossings==e_.cycleForwardCrossings) {
            if(!Increment(e_.cyclesCompleted))return;
            if(e_.restoreReady&&e_.cycleRetreatAxisMask==63U&&e_.cycleReturnAxisMask==63U&&
                (e_.cycleRetreatKindMask&6U)!=0U&&(e_.cycleReturnKindMask&6U)!=0U&&!Increment(e_.qualifiedCycles))return;
            e_.cycleActive=e_.restoreReady=false;e_.milestones|=RecoveryMapped;Mark(Event::Recovered);
        }
    }
};
}
