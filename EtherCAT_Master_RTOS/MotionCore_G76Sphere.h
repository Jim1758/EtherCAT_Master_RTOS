#pragma once
#include "NCG76SphereCycle.h"
#include "NCG76SphereRetreat.h"
#include "NCG76SpherePath.h"
#include "NCG76SphereGapProcess.h"
#include "MotionExecutionContract.h"
#include "MotionServoHandoffDiagnostic.h"
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM286 FIX1: a stationary rotary axis owns two representations: the NC
// native spelling (which may be modulo-reduced) and its accepted raw pulse.
// Prove the pair once; never reconstruct a stationary command from the angle.
namespace MotionG76SphereFixedAxes
{
inline bool SameBits(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
inline bool MatchesNative(double native,double pulse,double resolution,double lead,
    bool moduloAxis,double modulo) noexcept
{
    if(!std::isfinite(native)||!std::isfinite(pulse)||!std::isfinite(resolution)||resolution<=0.0||
        !std::isfinite(lead)||lead<=0.0||(moduloAxis&&(!std::isfinite(modulo)||modulo<=0.0)))return false;
    const double ppu=resolution/lead,rawNative=pulse*lead/resolution,forward=native*ppu;
    if(!std::isfinite(ppu)||ppu<=0.0||!std::isfinite(rawNative)||!std::isfinite(forward))return false;
    if(forward==pulse||native==rawNative)return true;
    if(!moduloAxis)return false;
    double represented=std::fmod(rawNative,modulo);if(represented<0.0)represented+=modulo;
    return std::isfinite(represented)&&native==represented;
}
inline bool FeedbackNative(double referenceNative,double referencePulse,double observedPulse,
    double ppu,double& native) noexcept
{
    if(!std::isfinite(referenceNative)||!std::isfinite(referencePulse)||!std::isfinite(observedPulse)||
        !std::isfinite(ppu)||ppu<=0.0)return false;
    const double delta=observedPulse-referencePulse;
    if(!std::isfinite(delta))return false;
    // Keep signed zero and the exact frozen spelling at the reference. Do not
    // modulo the delta: a real whole-turn error must remain a whole-turn error.
    native=delta==0.0?referenceNative:referenceNative+delta/ppu;
    return std::isfinite(native);
}
inline bool CommandPulse(double native,double referenceNative,double referencePulse,double& pulse) noexcept
{
    if(!std::isfinite(native)||!std::isfinite(referenceNative)||!std::isfinite(referencePulse)||
        !SameBits(native,referenceNative))return false;
    pulse=referencePulse;return true;
}
}

// EDM293: software NIC-handoff evidence for the SIM-only observer. This is
// neither drive reception nor a discharge/work/flush completion receipt.
namespace MotionG76SphereProcessSend
{
inline bool ExactNonzeroSend(const MotionServoHandoffSample& sample,
    MotionExecutionEpoch epoch,const MotionOwnerLease& owner,std::uint32_t mask,
    std::uint64_t tick) noexcept
{
    if(!epoch||!owner.IsValid()||mask!=0x3FU||!tick||!sample.sequence||!sample.frameGeneration||
       sample.sourceTick!=tick||sample.epoch!=epoch||sample.owner!=static_cast<std::uint32_t>(owner.owner)||
       sample.ownerGeneration!=owner.generation||sample.idleScope||sample.scrubbed||
       !sample.finalChecked||!sample.attempted||!sample.apiAccepted||
       sample.slotMask!=mask||(sample.observedMask&mask)!=mask||
       (sample.sourceInputMask&mask)!=mask)return false;
    bool nonzero=false;
    for(unsigned axis=0U;axis<6U;++axis){
        if((sample.sourceStatusWord[axis]&0x006FU)!=0x0027U||sample.sourceMode[axis]!=9||
           sample.proposedVelocity[axis]!=sample.copiedVelocity[axis])return false;
        if(axis<3U)nonzero=nonzero||sample.copiedVelocity[axis]!=0;
        // Fixed rotary axes may need CSV holding corrections from the existing
        // position PID. The caller proves their nominal targets/velocities are
        // fixed; a nonzero correction is not a rotary-spindle command.
    }
    return nonzero;
}
}

// EDM294: only a frozen, explicitly enabled SIM recipe may request an AIR
// flush excursion. This conversion adds no authority to the observer's signed
// GAP target; the unchanged P50 parent remains the sole motion owner.
namespace MotionG76SphereFlush
{
inline bool AxisEnvelope(const NCG76SphereRuntimeGeometry::Geometry& geometry,
    const NCG76SphereCycle::FlushConfig& config,unsigned axis,double& minimum,double& maximum,
    unsigned retreatMode=0U) noexcept
{
    double height=config.enabled?config.heightMm:0.0;
    // EDM306 admits every height this same parent may select, before any
    // ordinal begins. A later deep flush cannot widen its frozen travel bound.
    if(config.deepFlushEnabled){
        if(!config.enabled||!std::isfinite(config.deepFlushHeightMm)||config.deepFlushHeightMm<=0.0)return false;
        height=(std::max)(height,config.deepFlushHeightMm);
    }
    // EDM307 single line/arc parents use the canonical per-kind envelope.
    // Arc B0 must cover every possible stopped tangent, and B1 is machine +Z.
    if(retreatMode!=0U||geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere)
        return NCG76SphereRetreat::AxisEnvelope(geometry,retreatMode,
            height,axis,minimum,maximum);
    if(axis>=8U)return false;
    minimum=geometry.boundsMinMCS[axis];maximum=geometry.boundsMaxMCS[axis];
    if(config.enabled&&axis<3U){
        const double offset=-geometry.axis[axis]*height;
        minimum=std::nextafter((std::min)(minimum,minimum+offset),
            -(std::numeric_limits<double>::infinity)());
        maximum=std::nextafter((std::max)(maximum,maximum+offset),
            (std::numeric_limits<double>::infinity)());
    }
    return std::isfinite(minimum)&&std::isfinite(maximum)&&minimum<=maximum;
}
}

// EDM308: the configured curve and native pumps supply one immutable
// admission ceiling. Optional program F is a separate nominal-only cap.
namespace MotionG76SphereParameterFeeds
{
inline bool Matches(const NCG76SphereCycle::Request& request,
    const NCG76SphereGapProcess::Config& config) noexcept
{
    if(config.orbitFeedsEnabled&&((request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&
        request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour)||
       request.parameterFeeds||!NCG76SphereGapProcess::Valid(config)))return false;
    if(request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
       (!NCG76OrbitContour::ModeP(request.geometry)||!config.enabled||!config.signedMotionEnabled||
        !config.orbitFeedsEnabled||request.parameterFeeds))return false;
    if(request.parameterFeeds!=config.parameterFeeds)return false;
    if(!request.parameterFeeds)return request.nominalFeedCapMmMin==0.0;
    if((request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
       (request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&request.retreatMode!=2U)||
       !std::isfinite(request.nominalFeedCapMmMin)||request.nominalFeedCapMmMin<0.0||
       request.nominalFeedCapMmMin>100.0)return false;
    double ceiling=0.0;
    return NCG76SphereGapProcess::ResolveParameterMotionCeiling(config,ceiling)&&
        request.feedMmMin==ceiling&&request.geometry.feedMmMin==ceiling;
}
}

// EDM340: both sidecar and active Request retain exact, nonmodal clock data.
namespace MotionG76SphereMachiningTime
{
inline NCMachiningTime::Config Config(const NCG76SphereCycle::Request& request) noexcept
{NCMachiningTime::Config value{};value.totalLimitUs=request.totalLimitUs;value.additionalUs=request.additionalUs;return value;}
inline bool Matches(const NCMachiningTime::Config& value,const NCG76SphereCycle::Request& request) noexcept
{return NCMachiningTime::ValidConfig(value)&&NCMachiningTime::SameConfig(value,Config(request));}
inline bool TerminalQualified(const NCG76SphereCycle::Feedback& feedback,const NCG76SphereCycle::Request& request) noexcept
{
    using namespace NCMachiningTime;
    if(feedback.totalLimitUs!=request.totalLimitUs||feedback.additionalUs!=request.additionalUs||
       feedback.totalElapsedUs>request.totalLimitUs||feedback.additionalElapsedUs>request.additionalUs)return false;
    if(!request.totalLimitUs&&!request.additionalUs)
        return feedback.totalElapsedUs==0ULL&&feedback.additionalElapsedUs==0ULL&&
            feedback.totalTimerState==0U&&feedback.additionalTimerState==0U&&feedback.timePhase==0U;
    if(feedback.timePhase!=static_cast<std::uint8_t>(TimePhase::Completed))return false;
    if(request.totalLimitUs?(feedback.totalTimerState!=static_cast<std::uint8_t>(TimerState::Completed)||
        feedback.totalElapsedUs>=request.totalLimitUs):feedback.totalTimerState!=0U)return false;
    return request.additionalUs?(feedback.additionalTimerState==static_cast<std::uint8_t>(TimerState::Completed)&&
        feedback.additionalElapsedUs==request.additionalUs):feedback.additionalTimerState==0U;
}
}

enum class MotionG76SphereAdmissionGate : std::uint8_t
{
    None,Request,Frame,Compensation,OwnerDrain,AxisConfiguration,NativeBaseline,
    Travel,RuntimeRequest,Reservation,Publication,Enqueue,FlushEnvelope
};
struct MotionG76SphereAdmission
{
    MotionG76SphereAdmissionGate gate=MotionG76SphereAdmissionGate::None;
    int axis=-1;
};
inline const char* MotionG76SphereAdmissionGateName(MotionG76SphereAdmissionGate gate) noexcept
{
    switch(gate){
    case MotionG76SphereAdmissionGate::None:return "NONE";
    case MotionG76SphereAdmissionGate::Request:return "REQUEST";
    case MotionG76SphereAdmissionGate::Frame:return "FRAME";
    case MotionG76SphereAdmissionGate::Compensation:return "COMPENSATION";
    case MotionG76SphereAdmissionGate::OwnerDrain:return "OWNER_OR_DRAIN";
    case MotionG76SphereAdmissionGate::AxisConfiguration:return "AXIS_CONFIG";
    case MotionG76SphereAdmissionGate::NativeBaseline:return "NATIVE_BASELINE";
    case MotionG76SphereAdmissionGate::Travel:return "TRAVEL";
    case MotionG76SphereAdmissionGate::RuntimeRequest:return "RUNTIME_REQUEST";
    case MotionG76SphereAdmissionGate::Reservation:return "RESERVATION";
    case MotionG76SphereAdmissionGate::Publication:return "PUBLICATION";
    case MotionG76SphereAdmissionGate::Enqueue:return "ENQUEUE";
    case MotionG76SphereAdmissionGate::FlushEnvelope:return "FLUSH_ENVELOPE";
    default:return "UNKNOWN";}
}

// EDM296: bounded observations only; these fields never grant motion authority.
enum class MotionG76SphereInputStage : std::uint8_t {None,Observe,Admission,Step};
enum class MotionG76SphereInputGate : std::uint8_t
{
    None,ClaimMissing,RequestUnavailable,SessionMissing,PdoCycleInvalid,PdoNotContiguous,
    PowerNotInhibited,ClockInvalid,ClockFrequencyMissing,ClockTimeMissing,ContextsMissing,
    AxisMissing,AxisFirstCycle,AxisFault,AxisLag,ServoSnapshotUnavailable,ServoStatus,ServoMode,
    ScalePointerMissing,ScaleNonfinite,ScaleDeviation,NominalNonfinite,NativeNonfinite,
    InputSourceInvalid,InputPdoInvalid,InputPowerNotInhibited,InputTickMismatch,InputScopeMismatch,
    AdmissionTickMissing,AdmissionTimeMissing,AdmissionCancelled,AdmissionPhysicalSequenceMismatch,
    AdmissionTickBackwards,AdmissionActualNonfinite
};
namespace MotionG76SphereInputFlags
{
enum : std::uint32_t {
    Claim=1U,Running=2U,RequestRead=4U,PdoCycleValid=8U,Contiguous=16U,
    PowerInhibited=32U,ClockValid=64U,ClockFrequency=128U,ClockNow=256U,Contexts=512U,
    SourceValid=1024U,InputPdoValid=2048U,InputPowerInhibited=4096U,Cancelled=8192U,
    TickMatches=16384U,ScopeMatches=32768U,PhysicalSequenceMatches=65536U,AdmissionTickValid=131072U
};
}
struct MotionG76SphereInputDiagnostic
{
    std::uint64_t tick=0ULL,inputTick=0ULL,physicalSequence=0ULL,nowUs=0ULL,clockFrequency=0ULL;
    std::uint64_t inputSession=0ULL,expectedSession=0ULL;
    MotionExecutionIdentity inputIdentity{};
    std::uint32_t flags=0U,validAxisMask=0U;
    std::int32_t rawPosition=0;
    std::uint16_t statusWord=0U;
    MotionG76SphereInputStage stage=MotionG76SphereInputStage::None;
    MotionG76SphereInputGate gate=MotionG76SphereInputGate::None;
    std::int8_t axis=-1,mode=0;
    bool valid=false;
};
static_assert(sizeof(MotionG76SphereInputDiagnostic)<=128U,"P50 first input fault must remain a bounded small value.");
static_assert(std::is_trivially_copyable<MotionG76SphereInputDiagnostic>::value,"P50 input diagnostics must remain POD transport data.");
inline const char* MotionG76SphereInputStageName(MotionG76SphereInputStage stage) noexcept
{
    switch(stage){case MotionG76SphereInputStage::None:return "NONE";
    case MotionG76SphereInputStage::Observe:return "OBSERVE";case MotionG76SphereInputStage::Admission:return "ADMISSION";
    case MotionG76SphereInputStage::Step:return "STEP";default:return "UNKNOWN";}
}
inline const char* MotionG76SphereInputGateName(MotionG76SphereInputGate gate) noexcept
{
    switch(gate){
#define G76_INPUT_GATE_NAME(name) case MotionG76SphereInputGate::name:return #name
    G76_INPUT_GATE_NAME(None);G76_INPUT_GATE_NAME(ClaimMissing);G76_INPUT_GATE_NAME(RequestUnavailable);
    G76_INPUT_GATE_NAME(SessionMissing);G76_INPUT_GATE_NAME(PdoCycleInvalid);G76_INPUT_GATE_NAME(PdoNotContiguous);
    G76_INPUT_GATE_NAME(PowerNotInhibited);G76_INPUT_GATE_NAME(ClockInvalid);G76_INPUT_GATE_NAME(ClockFrequencyMissing);
    G76_INPUT_GATE_NAME(ClockTimeMissing);G76_INPUT_GATE_NAME(ContextsMissing);G76_INPUT_GATE_NAME(AxisMissing);
    G76_INPUT_GATE_NAME(AxisFirstCycle);G76_INPUT_GATE_NAME(AxisFault);G76_INPUT_GATE_NAME(AxisLag);
    G76_INPUT_GATE_NAME(ServoSnapshotUnavailable);G76_INPUT_GATE_NAME(ServoStatus);G76_INPUT_GATE_NAME(ServoMode);
    G76_INPUT_GATE_NAME(ScalePointerMissing);G76_INPUT_GATE_NAME(ScaleNonfinite);G76_INPUT_GATE_NAME(ScaleDeviation);
    G76_INPUT_GATE_NAME(NominalNonfinite);G76_INPUT_GATE_NAME(NativeNonfinite);G76_INPUT_GATE_NAME(InputSourceInvalid);
    G76_INPUT_GATE_NAME(InputPdoInvalid);G76_INPUT_GATE_NAME(InputPowerNotInhibited);G76_INPUT_GATE_NAME(InputTickMismatch);
    G76_INPUT_GATE_NAME(InputScopeMismatch);G76_INPUT_GATE_NAME(AdmissionTickMissing);G76_INPUT_GATE_NAME(AdmissionTimeMissing);
    G76_INPUT_GATE_NAME(AdmissionCancelled);G76_INPUT_GATE_NAME(AdmissionPhysicalSequenceMismatch);
    G76_INPUT_GATE_NAME(AdmissionTickBackwards);G76_INPUT_GATE_NAME(AdmissionActualNonfinite);
#undef G76_INPUT_GATE_NAME
    default:return "UNKNOWN";}
}

// EDM286: one custom AIR parent. Internal layers/HOLD legs never enter the
// NC ledger. All feedback is published by the RT owner as a bounded value.
struct MotionG76SphereFeedback
{
    NCG76SphereCycle::Feedback cycle{};
    NCG76SphereGapProcess::Feedback processShadow{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease owner{};
    std::array<double,8U> endpointMCS{},commandMCS{},velocityNative{},actualMCS{};
    std::array<double,8U> holdRetreatAnchorMCS{},holdProcessPointMCS{};
    bool holdInspectionJogReady=false,holdRecoveryResumeReady=false;
    std::uint32_t holdInspectionAxisMask=0U;
    // Diagnostic copy of the producer's installed-window-clamped tolerances;
    // feedback never grants start authority or changes the frozen request.
    double positionToleranceMm=0.0,rotaryToleranceDeg=0.0;
    std::uint64_t session=0ULL,inputSequence=0ULL,holdSequence=0ULL,startSequence=0ULL;
    // EDM307: RT-published lineage of the completed analytic parent. These
    // values do not authorize a continuation without exact endpoint/drain
    // release and matching owner, epoch, adjacent source and frozen config.
    std::uint64_t translationGeneration=0ULL;
    std::uint64_t initialIntervalWorkUs=0ULL,completedFlushOffset=0ULL;
    // Observations only: the loader's consumed endpoint release is authority.
    std::uint64_t previousParentSession=0ULL;
    std::int32_t previousParentPC=-1;
    NCG76SphereRuntimeGeometry::Kind nominalKind=NCG76SphereRuntimeGeometry::Kind::Sphere;
    std::uint32_t runtimeFault=0U;
    MotionG76SphereInputDiagnostic firstInputFault{};
    std::uint32_t inputValidAxisMask=0U;
    bool accepted=false,started=false,terminal=false,completed=false,endpointCommitted=false;
    // RT receipt: deadline braking, physical zero settle and group retirement all succeeded.
    bool timeLimitStopped=false;
};
static_assert(sizeof(MotionG76SphereFeedback)<=2048U,"P50 diagnostics must fit the existing bounded feedback channel.");

namespace MotionG76SphereMachiningTime
{
inline bool TimeLimitStoppedQualified(const MotionG76SphereFeedback& report,const NCG76SphereCycle::Request& request) noexcept
{
    using namespace NCMachiningTime;
    const auto& f=report.cycle;
    if(!report.timeLimitStopped||!report.accepted||!report.started||!report.terminal||report.completed||
       report.endpointCommitted||report.runtimeFault!=10U||!request.session||report.session!=request.session||
       f.session!=request.session||!f.tick||f.phase!=NCG76SphereCycle::Phase::Fault||
       f.fault!=NCG76SphereCycle::Fault::TimeLimit||!ValidConfig(Config(request))||!request.totalLimitUs||
       f.totalLimitUs!=request.totalLimitUs||f.totalElapsedUs!=request.totalLimitUs||
       f.additionalUs!=request.additionalUs||f.additionalElapsedUs>request.additionalUs||
       f.totalTimerState!=static_cast<std::uint8_t>(TimerState::Expired)||
       f.timePhase!=static_cast<std::uint8_t>(TimePhase::TimedOut))return false;
    for(unsigned i=0U;i<8U;++i)
        if(!std::isfinite(report.commandMCS[i])||!std::isfinite(report.actualMCS[i])||
           report.velocityNative[i]!=0.0)return false;
    return true;
}
}
