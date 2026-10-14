#pragma once
#include "EDMCoordinatedKernel.h"
#include "EDMMultiAxisOutputPreview.h"
#include "EDMMultiAxisTrajectoryPreview.h"
#include "EDMMultiAxisCommandBatch.h"
#include "EDMMultiAxisServoSimulation.h"
#include "EDMToolHeadProfile.h"
#include "EDMGapServoRTSession.h"
#include "EDMExecutionFeedbackChannel.h"
#include "MotionCommandRing.h"
#include <atomic>
#include <cstdint>
#include <limits>
#include <new>
#include <type_traits>

// EDM84: one real Motion callback -> at most one numerical Kernel::Step.
// This diagnostic has NO axis/PDO/output writer, allocation, lock or logger.
// modelUs is nominal 250 us sample time, NOT the observed callback clock.
// Callback gaps are independently checked; neither jitter nor WCET is certified.
namespace EDM84RT
{
using Facts=EDM55RT::Facts;
enum class Profile : std::uint8_t
{ Normal=69U, OperatorWindow=70U, AuthoredNormal=71U, AuthoredOperatorWindow=72U,
  SampledNormal=73U, SampledOperatorWindow=74U,
  SourceDwellNormal=75U, SourceDwellOperatorWindow=76U,
  OutputPreviewNormal=77U, OutputPreviewOperatorWindow=78U,
  TrajectoryPreviewNormal=79U, TrajectoryPreviewOperatorWindow=80U,
  CommandBatchNormal=81U, CommandBatchOperatorWindow=82U,
  ServoSimulationNormal=83U, ServoSimulationOperatorWindow=84U };
// Fault injection belongs only to the explicit sampled simulation profiles.
// It never changes the installed AD route or an ordinary CNC/EDM source.
enum class SampledFault : std::uint8_t
{ Normal=0U, FrozenSource=1U, SequenceRollback=2U, ChangedDuplicate=3U };
inline bool ValidSampledFault(SampledFault f) noexcept
{ return f==SampledFault::Normal || f==SampledFault::FrozenSource ||
    f==SampledFault::SequenceRollback || f==SampledFault::ChangedDuplicate; }
inline const char* SampledFaultName(SampledFault f) noexcept
{
    switch(f) { case SampledFault::Normal:return "NORMAL"; case SampledFault::FrozenSource:return "FROZEN_SOURCE";
    case SampledFault::SequenceRollback:return "SEQUENCE_ROLLBACK";
    case SampledFault::ChangedDuplicate:return "CHANGED_DUPLICATE"; } return "INVALID";
}
inline bool IsServoSimulation(Profile p) noexcept
{ return p==Profile::ServoSimulationNormal || p==Profile::ServoSimulationOperatorWindow; }
inline bool IsCommandBatch(Profile p) noexcept
{ return p==Profile::CommandBatchNormal || p==Profile::CommandBatchOperatorWindow || IsServoSimulation(p); }
inline bool IsTrajectoryOperatorWindow(Profile p) noexcept
{ return p==Profile::TrajectoryPreviewOperatorWindow || p==Profile::CommandBatchOperatorWindow ||
    p==Profile::ServoSimulationOperatorWindow; }
inline bool IsTrajectoryPreview(Profile p) noexcept
{ return p==Profile::TrajectoryPreviewNormal || IsTrajectoryOperatorWindow(p) || IsCommandBatch(p); }
inline bool IsOutputPreview(Profile p) noexcept
{ return p==Profile::OutputPreviewNormal || p==Profile::OutputPreviewOperatorWindow || IsTrajectoryPreview(p); }
inline bool IsSourceDwell(Profile p) noexcept
{ return p==Profile::SourceDwellNormal || p==Profile::SourceDwellOperatorWindow || IsOutputPreview(p); }
inline bool IsSampled(Profile p) noexcept
{ return p==Profile::SampledNormal || p==Profile::SampledOperatorWindow || IsSourceDwell(p); }
inline EDM88::SourceConfig SampledSourceForProfile(Profile p,std::uint64_t generation) noexcept
{
    auto c=EDM88::SimulatedSource(generation);
    if(IsSourceDwell(p)) { c.samplePeriodUs=2000U; c.maxAgeUs=2500U; }
    return c;
}
enum class Source : std::uint8_t { Fixture, NCAuthored };
inline bool IsAuthored(Profile p) noexcept
{ return p==Profile::AuthoredNormal || p==Profile::AuthoredOperatorWindow || IsSampled(p); }
inline bool ValidProfile(Profile p) noexcept
{ return p==Profile::Normal || p==Profile::OperatorWindow || IsAuthored(p); }
enum class Scenario : std::uint8_t { SixAxisPath=1U, MachineDirection=2U, ElectrodeCCompensation=3U, ToolHeadCUVCompensation=4U };
inline bool ValidScenario(Scenario s) noexcept
{ return s==Scenario::SixAxisPath || s==Scenario::MachineDirection || s==Scenario::ElectrodeCCompensation ||
    s==Scenario::ToolHeadCUVCompensation; }
inline const char* ScenarioName(Scenario s) noexcept
{ switch(s) { case Scenario::SixAxisPath:return "SIX_AXIS_PATH"; case Scenario::MachineDirection:return "MACHINE_DIRECTION";
  case Scenario::ElectrodeCCompensation:return "ELECTRODE_C_COMPENSATION";
  case Scenario::ToolHeadCUVCompensation:return "TOOL_HEAD_CUV_COMPENSATION"; } return "INVALID"; }
inline const char* DiagnosticTag(Profile p,Scenario s) noexcept
{ return IsServoSimulation(p)?"EDM93":IsCommandBatch(p)?"EDM92":IsTrajectoryPreview(p)?"EDM91":IsOutputPreview(p)?"EDM90":IsSourceDwell(p)?"EDM89":(IsSampled(p)?"EDM88":(IsAuthored(p)?(s==Scenario::ToolHeadCUVCompensation?"EDM86":"EDM85"):"EDM84")); }
enum class State : std::uint8_t { Idle, AwaitingStart, Running, Completed, Cancelled, Fault };
enum class Reason : std::uint8_t
{ None, AwaitStart, InvalidStart, Cancelled, RuntimeClock, RuntimeSource,
  AuthorityLost, Heartbeat, Kernel, Coverage, Deadline, CounterOverflow, SampledGap, OutputPreview, TrajectoryPreview, CommandBatch, ServoSimulation };
constexpr std::uint64_t PeriodUs=250U, HeartbeatMaximumAgeUs=100000U;
constexpr std::uint64_t StartTimeoutUs=1000000U, MaximumCallbackGapUs=2000U;
inline std::uint64_t DurationUs(Profile p) noexcept
{ return p==Profile::OperatorWindow || p==Profile::AuthoredOperatorWindow || p==Profile::SampledOperatorWindow || p==Profile::SourceDwellOperatorWindow || p==Profile::OutputPreviewOperatorWindow || IsTrajectoryOperatorWindow(p)?20000000ULL:12000000ULL; }
inline std::uint64_t ShortReleaseUs(Profile p) noexcept
{ return IsTrajectoryOperatorWindow(p)?5000000ULL:3500000ULL; }
inline std::uint64_t JumpStartUs(Profile p) noexcept
{ return IsTrajectoryOperatorWindow(p)?9000000ULL:6000000ULL; }
inline std::uint64_t AuthoredOverheadUs(Profile p) noexcept
{ return IsTrajectoryOperatorWindow(p)?11000000ULL:8000000ULL; }
inline bool Terminal(State s) noexcept
{ return s==State::Completed || s==State::Cancelled || s==State::Fault; }
inline const char* StateName(State s) noexcept
{
    switch(s) { case State::Idle:return "IDLE"; case State::AwaitingStart:return "AWAIT_START";
    case State::Running:return "RUNNING"; case State::Completed:return "COMPLETED";
    case State::Cancelled:return "CANCELLED"; case State::Fault:return "FAULT"; } return "UNKNOWN";
}
inline const char* ReasonName(Reason r) noexcept
{
    switch(r) { case Reason::None:return "NONE"; case Reason::AwaitStart:return "AWAIT_START";
    case Reason::InvalidStart:return "INVALID_START"; case Reason::Cancelled:return "CANCELLED";
    case Reason::RuntimeClock:return "RT_CLOCK"; case Reason::RuntimeSource:return "RT_SOURCE";
    case Reason::AuthorityLost:return "AUTHORITY"; case Reason::Heartbeat:return "HEARTBEAT";
    case Reason::Kernel:return "KERNEL"; case Reason::Coverage:return "COVERAGE";
    case Reason::Deadline:return "DEADLINE"; case Reason::CounterOverflow:return "COUNTER_OVERFLOW";
    case Reason::SampledGap:return "SAMPLED_GAP"; case Reason::OutputPreview:return "OUTPUT_PREVIEW";
    case Reason::TrajectoryPreview:return "TRAJECTORY_PREVIEW"; case Reason::CommandBatch:return "COMMAND_BATCH"; case Reason::ServoSimulation:return "SERVO_SIMULATION"; }
    return "UNKNOWN";
}
enum Coverage : std::uint64_t
{
    Fresh=1ULL<<0U, GapForward=1ULL<<1U, GapReverse=1ULL<<2U,
    InitialLine=1ULL<<3U, Arc=1ULL<<4U, FinalLine=1ULL<<5U,
    ShortRetreat=1ULL<<6U, ShortReturn=1ULL<<7U,
    JumpRetreat=1ULL<<8U, JumpReturn=1ULL<<9U,
    CMoving=1ULL<<10U, UMoving=1ULL<<11U, VMoving=1ULL<<12U,
    Endpoint=1ULL<<13U, CanonicalComplete=1ULL<<14U,
    OffsetFrozen=1ULL<<15U, OffsetReturned=1ULL<<16U,
    MappingRoundTrip=1ULL<<17U, DirectionFrozen=1ULL<<18U,
    CanonicalCorrespondence=1ULL<<19U, OrientationResolved=1ULL<<20U
};
inline std::uint64_t RequiredCoverage(Scenario s) noexcept
{
    const std::uint64_t common=((1ULL<<15U)-1ULL)|CanonicalCorrespondence;
    if(s==Scenario::SixAxisPath) return common;
    if(s==Scenario::MachineDirection) return common|OffsetFrozen|OffsetReturned|DirectionFrozen;
    if(s==Scenario::ElectrodeCCompensation) return common|OffsetFrozen|OffsetReturned|DirectionFrozen|MappingRoundTrip;
    return 0U;
}
struct Request
{
    std::uint64_t session=0U, issuedUs=0U, qpcFrequency=0U;
    EDM81::Scope scope{};
    EDM84::Config config{};
    EDM84::SynchronizedPath path{};
    // Optional frozen NC-owned model. Empty preserves every legacy Q4 caller;
    // no catalog pointer, mutable configuration or I/O reaches the RT owner.
    EDM87::Selection toolHeadSelection{};
    Profile profile=Profile::Normal;
    Scenario scenario=Scenario::SixAxisPath;
    SampledFault sampledFault=SampledFault::Normal;
    Source source=Source::Fixture;
    std::uint64_t durationUs=0U;
    unsigned expectedAxisMask=0U,expectedRowMask=0U,expectedGeometryMask=0U;
};
inline const char* DiagnosticTag(const Request& r) noexcept
{ return IsServoSimulation(r.profile)?"EDM93":IsCommandBatch(r.profile)?"EDM92":IsTrajectoryPreview(r.profile)?"EDM91":IsOutputPreview(r.profile)?"EDM90":IsSourceDwell(r.profile)?"EDM89":(IsSampled(r.profile)?"EDM88":(EDM87::IsEmptySelection(r.toolHeadSelection)?DiagnosticTag(r.profile,r.scenario):"EDM87")); }
inline bool SampledRequestValid(const Request& r) noexcept
{
    if(!ValidSampledFault(r.sampledFault)) return false;
    if(!IsSampled(r.profile))
        return r.sampledFault==SampledFault::Normal && EDM88::IsDisabled(r.config.gapSource);
    return EDM81::Finite(r.config.motion.referenceV) && r.config.motion.referenceV>=10.0 &&
        r.config.motion.referenceV<=290.0 && EDM88::Valid(r.config.gapSource) &&
        EDM88::SameConfig(r.config.gapSource,SampledSourceForProfile(r.profile,r.scope.sourceGeneration)) &&
        (!IsSourceDwell(r.profile) || (r.config.motion.servo.shortEnterMs && r.config.motion.servo.shortExitMs &&
            r.config.motion.servo.shortEnterMs<=100U && r.config.motion.servo.shortExitMs<=100U));
}
inline bool ToolHeadSelectionScopeValid(const Request& r) noexcept
{
    const auto& selected=r.toolHeadSelection;
    if(EDM87::IsEmptySelection(selected)) return true;
    return IsAuthored(r.profile) && r.scenario==Scenario::ToolHeadCUVCompensation &&
        EDM87::ValidSelection(selected) && r.scope.configRevision==selected.catalogRevision &&
        r.scope.frameRevision==selected.profile.revision &&
        r.scope.kinematicsRevision==selected.profile.revision;
}
inline bool ToolHeadSelectionConfigurationValid(const Request& r) noexcept
{
    if(!ToolHeadSelectionScopeValid(r)) return false;
    if(EDM87::IsEmptySelection(r.toolHeadSelection)) return true;
    const auto& profile=r.toolHeadSelection.profile;
    return EDM86::ChainsEqual(r.config.mapping.chain,profile.chain) &&
        r.config.mapping.toolOffsetMm==profile.toolTipMm &&
        r.config.direction.unit==profile.directionUnit &&
        r.config.direction.revision==profile.revision &&
        r.config.mapping.revision==profile.revision;
}
inline std::uint64_t DurationUs(const Request& r) noexcept
{ return IsAuthored(r.profile)?r.durationUs:DurationUs(r.profile); }
// Authored metadata is derived from the copied path, never a caller's claim.
// Geometry flags: bit 0 LINE, bit 1 ARC. Nominal axis mask uses full bounds,
// so a full circle with identical endpoints still requires X/Y evidence.
inline bool PathEvidence(const EDM84::SynchronizedPath& path,unsigned& axes,unsigned& rows,unsigned& kinds) noexcept
{
    axes=rows=kinds=0U;
    if(!path.Valid() || !path.Count() || path.Count()>16U) return false;
    EDM81::Pose low{},high{};
    if(path.NativeBounds(low,high)!=EDM81::Reason::None) return false;
    for(unsigned axis=0U;axis<6U;++axis) if(high[axis]>low[axis]) axes|=1U<<axis;
    for(unsigned row=0U;row<path.Count();++row)
    {
        const auto* g=path.Row(row); if(!g) return false;
        if(g->kind==NCPathCoreRetainedKind::LINE) kinds|=1U;
        else if(g->kind==NCPathCoreRetainedKind::ARC) kinds|=2U;
        else return false;
        rows|=1U<<row;
    }
    return (axes&7U)!=0U;
}
inline bool RequestPlanValid(const Request& r) noexcept
{
    if(!ValidProfile(r.profile) || !ValidScenario(r.scenario) || !SampledRequestValid(r) || !ToolHeadSelectionScopeValid(r) ||
        (r.scenario==Scenario::ToolHeadCUVCompensation && !IsAuthored(r.profile))) return false;
    if(!IsAuthored(r.profile)) return r.source==Source::Fixture;
    unsigned axes=0U,rows=0U,kinds=0U;
    const double speed=r.config.motion.maxScalarMmMin;
    if(!EDM81::Finite(speed) || speed<=0.0 || !r.path.Valid()) return false;
    const double required=std::ceil((r.path.LengthMm()/speed*60000000.0+static_cast<double>(AuthoredOverheadUs(r.profile)))/
        static_cast<double>(PeriodUs))*static_cast<double>(PeriodUs);
    if(!EDM81::Finite(required) || required<0.0 || required>60000000.0 ||
        static_cast<double>(r.durationUs)<required) return false;
    return r.source==Source::NCAuthored && r.durationUs>=DurationUs(r.profile) &&
        r.durationUs<=60000000ULL && r.durationUs%PeriodUs==0U &&
        PathEvidence(r.path,axes,rows,kinds) && r.expectedAxisMask==axes &&
        r.expectedRowMask==rows && r.expectedGeometryMask==kinds;
}
inline std::uint64_t ExpectedCoverage(const Request& r) noexcept
{
    if(!IsAuthored(r.profile)) return RequiredCoverage(r.scenario);
    std::uint64_t mask=Fresh|GapForward|GapReverse|FinalLine|ShortRetreat|ShortReturn|
        JumpRetreat|JumpReturn|Endpoint|CanonicalComplete|CanonicalCorrespondence;
    if(r.expectedGeometryMask&1U) mask|=InitialLine;
    if(r.expectedGeometryMask&2U) mask|=Arc;
    if(r.expectedAxisMask&(1U<<3U)) mask|=CMoving;
    if(r.expectedAxisMask&(1U<<4U)) mask|=UMoving;
    if(r.expectedAxisMask&(1U<<5U)) mask|=VMoving;
    if(r.scenario!=Scenario::SixAxisPath) mask|=OffsetFrozen|OffsetReturned|DirectionFrozen;
    if(r.scenario==Scenario::ElectrodeCCompensation || r.scenario==Scenario::ToolHeadCUVCompensation)
        mask|=MappingRoundTrip;
    if(r.scenario==Scenario::ToolHeadCUVCompensation) mask|=OrientationResolved;
    return mask;
}
inline bool ScenarioConfigurationValid(const Request& request) noexcept
{
    if(!ValidScenario(request.scenario) || !ToolHeadSelectionConfigurationValid(request)) return false;
    const auto& c=request.config;
    if(request.scenario==Scenario::SixAxisPath)
        return c.excursion==EDM84::ExcursionMode::RetainedPath &&
            c.mapping.kind==EDM84::MappingKind::Identity && c.direction.frame==EDM84::DirectionFrame::Machine;
    if(request.scenario==Scenario::MachineDirection)
        return c.excursion==EDM84::ExcursionMode::DirectionOffset &&
            c.mapping.kind==EDM84::MappingKind::Identity && c.direction.frame==EDM84::DirectionFrame::Machine;
    if(request.scenario==Scenario::ElectrodeCCompensation)
        return c.excursion==EDM84::ExcursionMode::DirectionOffset &&
            c.mapping.kind==EDM84::MappingKind::FixedZC && c.direction.frame==EDM84::DirectionFrame::ElectrodeC;
    // Q4 is deliberately NC-authored only. A legacy P69/P70 fixture must not
    // acquire a new kinematic interpretation merely by changing its Q label.
    return request.scenario==Scenario::ToolHeadCUVCompensation && IsAuthored(request.profile) &&
        c.excursion==EDM84::ExcursionMode::DirectionOffset &&
        c.mapping.kind==EDM84::MappingKind::SerialHeadCUV &&
        c.direction.frame==EDM84::DirectionFrame::ElectrodeCUV &&
        c.direction.revision==request.scope.frameRevision &&
        c.mapping.revision==request.scope.kinematicsRevision &&
        EDM84::ValidMapping(c.mapping) && EDM84::ValidDirection(c.direction);
}
struct Feedback
{
    EDM81::Scope scope{};
    std::uint64_t session=0U;
    Profile profile=Profile::Normal;
    Scenario scenario=Scenario::SixAxisPath;
    State state=State::Idle;
    Reason reason=Reason::None;
    EDM81::Reason kernelReason=EDM81::Reason::None;
    std::uint64_t latestSession=0U,cancelThroughSession=0U,publication=0U,rtTick=0U,nowUs=0U;
    std::uint64_t startedUs=0U,startedTick=0U,terminalTick=0U,quiescentTick=0U;
    std::uint64_t runtimeSteps=0U,modelUs=0U,minimumStepUs=0U,maximumStepUs=0U;
    std::uint64_t coverageMask=0U,stageMask=0U,qpcFrequency=0U;
    unsigned stage=0U,axisMovedMask=0U;
    unsigned nominalAxisMovedMask=0U,visitedRowMask=0U;
    Source source=Source::Fixture;
    std::uint64_t durationUs=0U;
    unsigned toolHeadProfileId=0U;
    std::uint64_t toolHeadProfileRevision=0U,toolHeadCatalogRevision=0U,toolHeadFingerprint=0U;
    double maxLinearErrorMm=0.0,maxRotaryErrorDeg=0.0,maxMappingErrorMm=0.0,maxOffsetErrorMm=0.0;
    // Q4's live tool-frame orientation is separate from the last excursion
    // anchor direction stored in sample.frozenDirection. It is not an axis target.
    std::array<double,3U> resolvedDirectionUnit{};
    std::uint64_t orientationSamples=0U;
    double maxOrientationNormError=0.0,maxFrozenDirectionError=0.0;
    double voltageV=0.0;
    SampledFault sampledFault=SampledFault::Normal;
    EDM88::SourceConfig sampledConfig{};
    EDM88::Evidence sampledSource{};
    EDM89::Evidence sampledDecision{};
    EDM84::Snapshot sample{};
    std::uint64_t previewMachineRejectMask=0U;
    EDM90::Config previewConfig{};
    EDM90::Evidence previewEvidence{};
    EDM91::Evidence trajectoryEvidence{};
    EDM92::Evidence batchEvidence{};
    EDM93::Evidence servoEvidence{};
    std::uint64_t batchCancelTick=0U,batchCancelModelUs=0U;
    EDM81::State batchCancelPhase=EDM81::State::Idle;
    unsigned batchCancelStage=0U;
    bool startAcknowledged=false,quiescent=false;
    constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
    constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    constexpr bool PhysicalStopProven() const noexcept { return false; }
};
// The machine snapshot is captured by the RT owner on admission. NC verifies
// its immutable identity, initial composed pose and complete numeric history.
inline bool PreviewFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    if(!IsOutputPreview(r.profile)) return EDM90::IsEmpty(f.previewConfig) &&
        !f.previewEvidence.session && f.previewEvidence.state==EDM90::State::Idle;
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return !f.previewEvidence.session;
    const auto& c=f.previewConfig;
    if(!EDM90::Valid(c) || c.session!=r.session || !EDM81::SameScope(c.scope,r.scope) ||
        c.machine.tick!=f.startedTick || c.machine.nowUs!=f.startedUs || c.periodUs!=PeriodUs) return false;
    EDM81::Pose nominal{},initial{}; unsigned ordinal=0U; double u=0.0;
    std::array<double,3U> direction{};
    return r.path.Evaluate(0.0,nominal,ordinal,u) &&
        EDM84::ResolveDirection(r.config.direction,r.config.mapping,nominal,direction) &&
        EDM84::ComposePose(r.config.mapping,nominal,direction,0.0,initial) && initial==c.initialPose &&
        f.previewEvidence.session==r.session && EDM81::SameScope(f.previewEvidence.scope,r.scope);
}
inline bool PreviewCompletionValid(const Request& r,const Feedback& f) noexcept
{
    if(!PreviewFeedbackIdentityValid(r,f)) return false;
    if(!IsOutputPreview(r.profile)) return true;
    const auto& e=f.previewEvidence;
    return !f.previewMachineRejectMask && EDM90::ValidEvidence(f.previewConfig,e) && e.state==EDM90::State::Completed &&
        !e.candidateValid && e.processedFrames==f.runtimeSteps && e.lastSequence==f.runtimeSteps &&
        e.axisMovedMask==f.axisMovedMask && e.lastPose==f.sample.pose &&
        e.lastTick==f.startedTick+f.runtimeSteps-1U && e.lastTick<=f.rtTick;
}
inline bool PreviewHistoryValid(const Feedback& f) noexcept
{
    const auto& e=f.previewEvidence;
    if(e.state!=EDM90::State::Fault) return EDM90::ValidEvidence(f.previewConfig,e);
    if(e.reason==EDM90::Reason::None || static_cast<unsigned>(e.reason)>static_cast<unsigned>(EDM90::Reason::Lifecycle) ||
        e.rejectedAxis>6U || e.candidateValid ||
        (e.reason==EDM90::Reason::Machine && !f.previewMachineRejectMask)) return false;
    // A rejected candidate preserves all previously committed raw history.
    // The EDM90 validator accepts a cancelled receipt; normalize only these
    // fault lifecycle fields on a local copy to verify that unchanged history.
    auto history=e; history.state=EDM90::State::Cancelled;
    history.reason=EDM90::Reason::Cancelled; history.rejectedAxis=6U;
    return EDM90::ValidEvidence(f.previewConfig,history);
}
// Shaped output remains a diagnostic history. It never carries a physical
// motion permit, including while a revoked session numerically decelerates.
inline bool TrajectoryFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    const auto& e=f.trajectoryEvidence;
    if(!IsTrajectoryPreview(r.profile)) return EDM91::IsEmpty(e);
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM91::IsEmpty(e);
    if(!PreviewFeedbackIdentityValid(r,f) || !PreviewHistoryValid(f) ||
        !EDM91::ValidEvidence(f.previewConfig,e) ||
        e.session!=r.session || !EDM81::SameScope(e.scope,r.scope) ||
        e.captureTick!=f.startedTick || e.mapGeneration!=f.previewConfig.machine.mapGeneration ||
        e.rawFrames>f.runtimeSteps || e.rawFrames>f.previewEvidence.processedFrames ||
        e.lastTick>f.rtTick) return false;
    const auto raw=f.previewEvidence.processedFrames;
    if(raw>f.runtimeSteps || f.runtimeSteps-e.rawFrames>1U) return false;
    if(e.reason==EDM91::Reason::None && (e.rawFrames!=raw || raw!=f.runtimeSteps)) return false;
    if(e.rawFrames && e.lastTick!=f.startedTick+e.shapeSteps-1U) return false;
    if(e.rawFrames==f.previewEvidence.processedFrames && e.sourcePose!=f.previewEvidence.lastPose) return false;
    return true;
}
inline bool TrajectoryCompletionValid(const Request& r,const Feedback& f) noexcept
{
    if(!TrajectoryFeedbackIdentityValid(r,f)) return false;
    if(!IsTrajectoryPreview(r.profile)) return true;
    const auto& e=f.trajectoryEvidence;
    return e.state==EDM91::State::Completed && e.numericalStopped && !e.candidateValid &&
        e.rawFrames==f.runtimeSteps && e.rawFrames==f.previewEvidence.processedFrames &&
        e.sourcePose==f.sample.pose && e.shapedPose==f.sample.pose && !e.stopSteps &&
        e.lastTick<=f.rtTick;
}
inline bool TrajectoryRetirementValid(const Request& r,const Feedback& f) noexcept
{
    if(!TrajectoryFeedbackIdentityValid(r,f)) return false;
    if(!IsTrajectoryPreview(r.profile)) return true;
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM91::IsEmpty(f.trajectoryEvidence);
    const auto& e=f.trajectoryEvidence;
    return !e.candidateValid && e.state==EDM91::State::Retired &&
        f.state==State::Cancelled && f.quiescent && f.quiescentTick>f.terminalTick && e.lastTick<=f.terminalTick;
}
// Batch receipts acknowledge an RT-owned six-axis numerical command group.
// They are pre-PID native references, never CSV/PDO output or a motion permit.
inline bool BatchFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    const auto& b=f.batchEvidence;
    if(!IsCommandBatch(r.profile)) return EDM92::IsEmpty(b) && !f.batchCancelTick &&
        !f.batchCancelModelUs && f.batchCancelPhase==EDM81::State::Idle && !f.batchCancelStage;
    if(f.state==State::Cancelled) {
        if(!f.batchCancelTick || f.batchCancelTick>f.rtTick || f.batchCancelModelUs!=f.modelUs ||
            f.batchCancelStage!=f.stage || f.batchCancelStage>7U ||
            static_cast<unsigned>(f.batchCancelPhase)>static_cast<unsigned>(EDM81::State::Fault)) return false;
    } else if(f.batchCancelTick || f.batchCancelModelUs ||
        f.batchCancelPhase!=EDM81::State::Idle || f.batchCancelStage) return false;
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM92::IsEmpty(b);
    return TrajectoryFeedbackIdentityValid(r,f) &&
        EDM92::BoundTo(f.previewConfig,b,f.trajectoryEvidence);
}
inline bool BatchCompletionValid(const Request& r,const Feedback& f) noexcept
{
    if(!BatchFeedbackIdentityValid(r,f)) return false;
    if(!IsCommandBatch(r.profile)) return true;
    const auto& b=f.batchEvidence;
    return TrajectoryCompletionValid(r,f) && b.state==EDM92::State::Completed &&
        b.numericalStopped && !b.candidateValid && !b.revoked &&
        b.acceptedBatches==f.trajectoryEvidence.shapeSteps;
}
inline bool BatchRetirementValid(const Request& r,const Feedback& f) noexcept
{
    if(!BatchFeedbackIdentityValid(r,f)) return false;
    if(!IsCommandBatch(r.profile)) return true;
    if(!f.batchCancelTick || f.batchCancelTick>f.terminalTick ||
        (f.startAcknowledged && f.batchCancelTick<f.startedTick)) return false;
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM92::IsEmpty(f.batchEvidence);
    return TrajectoryRetirementValid(r,f) && f.batchEvidence.state==EDM92::State::Retired &&
        !f.batchEvidence.candidateValid && f.batchEvidence.revoked;
}
// EDM93 receipts describe a private quantized CSV velocity plant. They are
// bound to the accepted group, never to physical encoder following or PDO I/O.
inline bool ServoFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    const auto& e=f.servoEvidence;
    if(!IsServoSimulation(r.profile)) return EDM93::IsEmpty(e);
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM93::IsEmpty(e);
    return BatchFeedbackIdentityValid(r,f) && EDM93::BoundTo(f.previewConfig,e,f.batchEvidence) &&
        e.lastTick<=f.rtTick && (!e.lastTick || e.lastTick>=f.startedTick);
}
inline bool ServoCompletionValid(const Request& r,const Feedback& f) noexcept
{
    if(!ServoFeedbackIdentityValid(r,f)) return false;
    if(!IsServoSimulation(r.profile)) return true;
    const auto& e=f.servoEvidence;
    return BatchCompletionValid(r,f) && e.state==EDM93::State::Completed &&
        e.modelSettled && !e.revoked && e.reason==EDM93::Reason::None &&
        e.acceptedBatches==f.batchEvidence.acceptedBatches;
}
inline bool ServoRetirementValid(const Request& r,const Feedback& f) noexcept
{
    if(!ServoFeedbackIdentityValid(r,f)) return false;
    if(!IsServoSimulation(r.profile)) return true;
    if(!f.startAcknowledged && EDM90::IsEmpty(f.previewConfig)) return EDM93::IsEmpty(f.servoEvidence);
    return BatchRetirementValid(r,f) && f.servoEvidence.state==EDM93::State::Retired &&
        f.servoEvidence.revoked && f.servoEvidence.lastTick<=f.terminalTick;
}
inline bool ToolHeadFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    if(!ToolHeadSelectionScopeValid(r)) return false;
    const auto& selected=r.toolHeadSelection;
    if(EDM87::IsEmptySelection(selected))
        return !f.toolHeadProfileId && !f.toolHeadProfileRevision &&
            !f.toolHeadCatalogRevision && !f.toolHeadFingerprint;
    return f.toolHeadProfileId==selected.profile.id &&
        f.toolHeadProfileRevision==selected.profile.revision &&
        f.toolHeadCatalogRevision==selected.catalogRevision &&
        f.toolHeadFingerprint==selected.fingerprint;
}
inline bool SampledFeedbackIdentityValid(const Request& r,const Feedback& f) noexcept
{
    return SampledRequestValid(r) && f.sampledFault==r.sampledFault &&
        EDM88::SameConfig(f.sampledConfig,r.config.gapSource) &&
        EDM89::SameConfig(f.sampledDecision.config,IsSampled(r.profile)?
            EDM89::FrozenConfig(r.config.motion.servo):EDM89::Config{});
}
inline bool SameSampledTerminalEvidence(const Feedback& a,const Feedback& b) noexcept
{
    return a.previewMachineRejectMask==b.previewMachineRejectMask &&
        EDM90::SameConfig(a.previewConfig,b.previewConfig) &&
        EDM90::SameTerminalEvidence(a.previewEvidence,b.previewEvidence) &&
        EDM91::SameTerminalEvidence(a.trajectoryEvidence,b.trajectoryEvidence) &&
        EDM92::SameTerminalEvidence(a.batchEvidence,b.batchEvidence) &&
        EDM93::SameTerminalEvidence(a.servoEvidence,b.servoEvidence) &&
        a.sampledFault==b.sampledFault && EDM88::SameConfig(a.sampledConfig,b.sampledConfig) &&
        EDM88::SameEvidence(a.sampledSource,b.sampledSource) &&
        EDM89::SameEvidence(a.sampledDecision,b.sampledDecision);
}
inline bool SampledFeedbackEvidenceValid(const Request& r,const Feedback& f) noexcept
{
    if(!SampledFeedbackIdentityValid(r,f)) return false;
    if(!IsSampled(r.profile)) return EDM88::SameEvidence(f.sampledSource,EDM88::Evidence{}) &&
        EDM89::SameEvidence(f.sampledDecision,EDM89::Evidence{});
    const auto& decision=f.sampledDecision;
    if(!EDM89::ValidCompletedEvidence(r.config.motion.servo,decision,f.sampledSource) ||
        decision.enterCount!=1U || decision.exitCount!=1U ||
        decision.entryAborts!=(IsSourceDwell(r.profile)?1U:0U) ||
        decision.exitRestarts!=(IsSourceDwell(r.profile)?1U:0U)) return false;
    // Script-specific proof, beyond the generic decision's structural checks.
    // A plausible altered first receipt cannot manufacture a confirmation.
    const auto period=r.config.gapSource.samplePeriodUs;
    const auto mark=[period](std::uint64_t capturedUs) noexcept {
        return EDM89::Mark{(capturedUs-PeriodUs)/period+1U,capturedUs};
    };
    const std::uint64_t enterUs=3000000ULL+PeriodUs;
    const std::uint64_t exitUs=ShortReleaseUs(r.profile)+(IsSourceDwell(r.profile)?4000ULL:0ULL)+PeriodUs;
    if(!EDM89::SameMark(decision.enterStart,mark(enterUs)) ||
        !EDM89::SameMark(decision.enterConfirm,mark(enterUs+((decision.config.enterUs+period-1U)/period)*period)) ||
        !EDM89::SameMark(decision.exitStart,mark(exitUs)) ||
        !EDM89::SameMark(decision.exitConfirm,mark(exitUs+((decision.config.exitUs+period-1U)/period)*period))) return false;
    const auto& e=f.sampledSource;
    if(r.sampledFault!=SampledFault::Normal || e.faulted || e.reason!=EDM88::Reason::None ||
        e.rejectedFrames || !f.runtimeSteps || f.runtimeSteps>(DurationUs(r)+3000000ULL)/PeriodUs+1U ||
        f.modelUs!=f.runtimeSteps*PeriodUs ||
        !e.lastFrame.valid || !EDM88::SameIdentity(e.lastFrame.identity,r.config.gapSource.identity) ||
        !EDM81::Finite(e.lastFrame.voltageV) || e.lastFrame.voltageV<0.0 || e.lastFrame.voltageV>300.0 ||
        e.lastFrame.voltageV!=r.config.motion.referenceV+10.0 || f.voltageV!=e.lastFrame.voltageV ||
        e.maximumAgeUs!=r.config.gapSource.samplePeriodUs || !e.lastFrame.capturedAtUs ||
        e.lastFrame.capturedAtUs>f.modelUs) return false;
    const auto fresh=(f.runtimeSteps-1U)/(r.config.gapSource.samplePeriodUs/PeriodUs)+1U;
    return e.freshFrames==fresh && e.heldFrames==f.runtimeSteps-fresh &&
        e.lastFrame.sequence==fresh &&
        e.lastFrame.capturedAtUs==(fresh-1U)*r.config.gapSource.samplePeriodUs+PeriodUs &&
        f.modelUs-e.lastFrame.capturedAtUs<r.config.gapSource.samplePeriodUs;
}
inline bool CompletionCoverageValid(const Request& r,const Feedback& f) noexcept
{
    if(f.coverageMask!=ExpectedCoverage(r) || !PreviewCompletionValid(r,f) || !TrajectoryCompletionValid(r,f) || !BatchCompletionValid(r,f) || !ServoCompletionValid(r,f) || !ToolHeadFeedbackIdentityValid(r,f) ||
        !SampledFeedbackEvidenceValid(r,f)) return false;
    if(r.scenario==Scenario::ToolHeadCUVCompensation)
    {
        if(!ScenarioConfigurationValid(r) || !EDM81::SameScope(r.scope,f.scope) ||
            r.config.direction.revision!=r.scope.frameRevision ||
            r.config.mapping.revision!=r.scope.kinematicsRevision ||
            !f.runtimeSteps || f.orientationSamples!=f.runtimeSteps ||
            !EDM84::MappingDetail::Unit(f.resolvedDirectionUnit) ||
            !EDM81::Finite(f.maxOrientationNormError) || f.maxOrientationNormError<0.0 ||
            f.maxOrientationNormError>1e-10 || !EDM81::Finite(f.maxFrozenDirectionError) ||
            f.maxFrozenDirectionError<0.0 || f.maxFrozenDirectionError>1e-12) return false;
        std::array<double,3U> expectedDirection{};
        if(!EDM84::ResolveDirection(r.config.direction,r.config.mapping,f.sample.nominal,expectedDirection)) return false;
        for(unsigned axis=0U;axis<3U;++axis)
            if(std::fabs(expectedDirection[axis]-f.resolvedDirectionUnit[axis])>1e-12) return false;
    }
    if(!IsAuthored(r.profile)) return f.axisMovedMask==0x3FU;
    return f.source==Source::NCAuthored && f.durationUs==r.durationUs &&
        f.nominalAxisMovedMask==r.expectedAxisMask && f.visitedRowMask==r.expectedRowMask;
}
static_assert(std::is_trivially_copyable<Request>::value,"EDM84 request must be a fixed value");
static_assert(sizeof(Feedback)<=4096U,"EDM84 feedback exceeds atomic channel budget");
static_assert(std::is_nothrow_default_constructible<EDM84::Kernel>::value,
    "EDM84 in-place RT kernel reset must not throw");
static_assert(std::is_trivially_destructible<EDM84::Kernel>::value,
    "EDM84 in-place RT kernel reset requires a trivial destructor");

class Session
{
public:
    Session() noexcept=default;
    Session(const Session&)=delete;
    Session& operator=(const Session&)=delete;
    bool ControlStart(const Request& request) noexcept
    {
        // NC-owned attempt watermark also retires rejected starts. Reusing a
        // failed identity cannot accidentally adopt a queued older request.
        if(!request.session || request.session<=attemptedSession_) return false;
        attemptedSession_=request.session;
        const auto latest=latest_.load(std::memory_order_acquire);
        if(request.session<=latest || request.session<=cancelled_.load(std::memory_order_acquire) ||
            !EDM81::ValidScope(request.scope) || !request.issuedUs || !request.qpcFrequency ||
            request.issuedUs<lastControlHeartbeatUs_ ||
            !RequestPlanValid(request) ||
            !ScenarioConfigurationValid(request) || !request.path.Valid() || !EDM81::SameScope(request.scope,request.path.FrozenScope()) ||
            request.config.motion.periodUs!=PeriodUs) return false;
        if(latest)
        {
            Feedback previous{};
            if(!ControlRead(previous) || previous.session!=latest || !Terminal(previous.state) ||
                !previous.quiescent || !previous.terminalTick ||
                previous.quiescentTick<=previous.terminalTick) return false;
        }
        if(!requests_.ProducerHasCapacity(1U)) return false;
        Descriptor d{}; d.session=request.session; d.scope=request.scope; d.profile=request.profile; d.scenario=request.scenario;
        d.issuedUs=request.issuedUs; d.frequency=request.qpcFrequency; d.source=request.source; d.durationUs=DurationUs(request);
        d.sampledFault=request.sampledFault; d.sampledConfig=request.config.gapSource;
        d.sampledDecisionConfig=IsSampled(request.profile)?EDM89::FrozenConfig(request.config.motion.servo):EDM89::Config{};
        if(!EDM87::IsEmptySelection(request.toolHeadSelection))
        {
            const auto& selected=request.toolHeadSelection;
            d.toolHeadProfileId=selected.profile.id; d.toolHeadProfileRevision=selected.profile.revision;
            d.toolHeadCatalogRevision=selected.catalogRevision; d.toolHeadFingerprint=selected.fingerprint;
        }
        // Metadata first: cancellation between reservation and ring publication
        // can always be attributed to the exact newly reserved scope.
        if(!descriptors_.Publish(d)) return false;
        previewMode_.store(IsOutputPreview(request.profile),std::memory_order_release);
        latest_.store(request.session,std::memory_order_release);
        if(!ControlTouch(request.session,request.issuedUs) || !requests_.ProducerTryPush(request))
        { ControlCancel(request.session); return false; }
        return true;
    }
    bool ControlTouch(std::uint64_t session,std::uint64_t nowUs) noexcept
    {
        if(!session || !nowUs || session!=latest_.load(std::memory_order_acquire) ||
            session<=cancelled_.load(std::memory_order_acquire) || nowUs<lastControlHeartbeatUs_) return false;
        Heartbeat h{}; h.session=session; h.nowUs=nowUs;
        if(!heartbeats_.Publish(h)) { ControlCancel(session); return false; }
        lastControlHeartbeatUs_=nowUs; return true;
    }
    void ControlCancel(std::uint64_t session) noexcept
    {
        // One NC writer. Earlier detached cancellation cannot retire a newer id.
        if(session>cancelled_.load(std::memory_order_relaxed))
            cancelled_.store(session,std::memory_order_release);
    }
    std::uint64_t ControlLatestSession() const noexcept
    { return latest_.load(std::memory_order_acquire); }
    bool ControlRead(Feedback& out) const noexcept
    {
        out=Feedback{};
        const auto latest=latest_.load(std::memory_order_acquire);
        const auto cancel=cancelled_.load(std::memory_order_acquire);
        Feedback candidate{}; const auto stamp=feedback_.Read(candidate);
        if(!stamp.Ready() || latest!=latest_.load(std::memory_order_acquire) ||
            cancel!=cancelled_.load(std::memory_order_acquire) || candidate.session!=latest ||
            candidate.latestSession!=latest || candidate.cancelThroughSession!=cancel) return false;
        candidate.publication=stamp.publication; out=candidate; return true;
    }
    bool NeedsRuntimeService() const noexcept
    {
        const auto latest=latest_.load(std::memory_order_acquire);
        return latest && (latest!=retired_.load(std::memory_order_acquire) ||
            cancelled_.load(std::memory_order_acquire)!=retiredCancel_.load(std::memory_order_acquire));
    }
    // Legacy diagnostic scheduling is unchanged. P77/P78 are serviced only
    // after the complete six-axis input/motion scan, never twice in one tick.
    bool RequiresMachineFrame() const noexcept
    { return NeedsRuntimeService() && previewMode_.load(std::memory_order_acquire); }
    void RuntimeCycle(const Facts& facts,std::uint64_t frequency,
        const EDM90::MachineSnapshot* machine=nullptr,
        const EDM93::ControlSnapshot* controller=nullptr) noexcept
    {
        if(!NeedsRuntimeService() || RequiresMachineFrame()!=(machine!=nullptr)) return;
        if(!AdoptDescriptor(facts) || IsOutputPreview(f_.profile)!=(machine!=nullptr)) return;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs;
        const bool clock=facts.clockValid && facts.tick && facts.nowUs && frequency &&
            frequency==descriptor_.frequency && (!haveClock_ ||
                (previousTick_!=(std::numeric_limits<std::uint64_t>::max)() &&
                 facts.tick==previousTick_+1U && facts.nowUs>previousUs_ &&
                 facts.nowUs-previousUs_<=MaximumCallbackGapUs));
        const auto delta=clock && haveClock_?facts.nowUs-previousUs_:0U;
        previousTick_=facts.tick; previousUs_=facts.nowUs; haveClock_=true;
        const bool source=facts.pdoValid && facts.contiguous;
        ObserveCancel();
        // Admission gates permit one unconsumed request at a time. Two slots
        // allow the revoked-before-enqueue race to drain with bounded work.
        for(unsigned n=0U;n<2U && requests_.ConsumerTryPop(incoming_);++n)
            if(incoming_.session==f_.session)
            { request_=incoming_; haveRequest_=true; }
        Heartbeat h{};
        if(heartbeats_.Read(h).Ready() && h.session==f_.session &&
            h.nowUs>=lastRuntimeHeartbeatUs_ && h.nowUs<=facts.nowUs)
            lastRuntimeHeartbeatUs_=h.nowUs;
        ObserveCancel();
        if(f_.state==State::AwaitingStart)
        {
            if(!clock) Trip(Reason::RuntimeClock);
            else if(!source) Trip(Reason::RuntimeSource);
            else if(facts.nowUs<descriptor_.issuedUs)
            {
                if(descriptor_.issuedUs-facts.nowUs>HeartbeatMaximumAgeUs) Trip(Reason::InvalidStart);
            }
            else if(facts.nowUs-descriptor_.issuedUs>StartTimeoutUs) Trip(Reason::Deadline);
            else if(haveRequest_) Start(facts,machine,controller);
        }
        if(f_.state==State::Running || f_.state==State::Completed)
        {
            if(!clock) Trip(Reason::RuntimeClock);
            else if(!source) Trip(Reason::RuntimeSource);
            else if(!Authority(facts)) Trip(Reason::AuthorityLost);
            else if(!HeartbeatReady(facts.nowUs)) Trip(Reason::Heartbeat);
            else if(IsOutputPreview(f_.profile) && !MachineCurrent(facts,machine)) Trip(Reason::OutputPreview);
            else if(IsServoSimulation(f_.profile) && !ServoControlsCurrent(controller)) Trip(Reason::ServoSimulation);
            else if(facts.nowUs<f_.startedUs ||
                facts.nowUs-f_.startedUs>f_.durationUs+5000000ULL) Trip(Reason::Deadline);
            else if(f_.state==State::Running)
            {
                if(delta && facts.tick>f_.startedTick)
                {
                    if(!f_.minimumStepUs || delta<f_.minimumStepUs) f_.minimumStepUs=delta;
                    if(delta>f_.maximumStepUs) f_.maximumStepUs=delta;
                }
                if(rawCompleted_) {
                    if(!IsServoSimulation(f_.profile) || f_.trajectoryEvidence.state!=EDM91::State::Completed) DrainTrajectory();
                } else Step();
            }
        }
        ObserveCancel();
        ServiceTrajectoryStop(clock,source && (!IsServoSimulation(f_.profile) || ServoControlsCurrent(controller)));
        ServiceServoSettle(clock,source,controller);
        if(Terminal(f_.state) && !f_.quiescent && clock && source && TrajectoryTerminalReady() && ServoTerminalReady())
        {
            if(!f_.terminalTick) f_.terminalTick=facts.tick;
            else if(facts.tick>f_.terminalTick)
            {
                f_.quiescent=true; f_.quiescentTick=facts.tick; f_.sample.committed=false;
                if(f_.state==State::Cancelled && IsTrajectoryPreview(f_.profile) &&
                    !EDM91::IsEmpty(f_.trajectoryEvidence))
                {
                    trajectory_.Retire(); f_.trajectoryEvidence=trajectory_.Current();
                    if(IsCommandBatch(f_.profile)) {
                        if(!batch_.Retire(f_.trajectoryEvidence)) { batch_.Abort(); f_.reason=Reason::CommandBatch; }
                        f_.batchEvidence=batch_.Current();
                    }
                    if(IsServoSimulation(f_.profile) && !EDM93::IsEmpty(f_.servoEvidence)) {
                        if(!servo_.Retire()) { servo_.Abort(); f_.reason=Reason::ServoSimulation; }
                        f_.servoEvidence=servo_.Current();
                    }
                }
            }
        }
        if(!feedback_.Publish(f_)) { Trip(Reason::CounterOverflow); return; }
        // Completed is deliberately not retired; it stays guarded until NC's
        // explicit cancellation and a separate later quiescent receipt.
        if(f_.state==State::Cancelled && f_.quiescent &&
            f_.latestSession==latest_.load(std::memory_order_acquire) &&
            f_.cancelThroughSession==cancelled_.load(std::memory_order_acquire))
        { retiredCancel_.store(f_.cancelThroughSession,std::memory_order_release);
          retired_.store(f_.session,std::memory_order_release); haveClock_=false; }
    }
private:
    struct Descriptor
    {
        std::uint64_t session=0U,issuedUs=0U,frequency=0U;
        EDM81::Scope scope{}; Profile profile=Profile::Normal;
        Scenario scenario=Scenario::SixAxisPath;
        Source source=Source::Fixture; std::uint64_t durationUs=0U;
        SampledFault sampledFault=SampledFault::Normal; EDM88::SourceConfig sampledConfig{};
        EDM89::Config sampledDecisionConfig{};
        unsigned toolHeadProfileId=0U;
        std::uint64_t toolHeadProfileRevision=0U,toolHeadCatalogRevision=0U,toolHeadFingerprint=0U;
    };
    struct Heartbeat { std::uint64_t session=0U,nowUs=0U; };
    FixedCapacitySpscRing<Request,2U> requests_{};
    EDM53::Channel<Descriptor> descriptors_{};
    EDM53::Channel<Heartbeat> heartbeats_{};
    EDM53::Channel<Feedback,4096U> feedback_{};
    std::atomic<std::uint64_t> latest_{0U},cancelled_{0U},retired_{0U},retiredCancel_{0U};
    std::uint64_t attemptedSession_=0U,lastControlHeartbeatUs_=0U; // NC only.
    std::atomic<bool> previewMode_{false};
    EDM90::Preview preview_{};
    EDM91::Preview trajectory_{};
    EDM92::Bridge batch_{};
    EDM93::Simulation servo_{};
    EDM84::Kernel kernel_{};
    Request request_{},incoming_{}; // RT-owned scratch avoids clearing a large request each callback.
    Descriptor descriptor_{};
    Feedback f_{};
    std::uint64_t previousTick_=0U,previousUs_=0U,lastRuntimeHeartbeatUs_=0U;
    bool haveClock_=false,haveRequest_=false,jumpIssued_=false;
    bool shortSeen_=false,jumpSeen_=false,rawCompleted_=false;
    EDM81::Pose frozenJoint_{};
    EDM88::Frame sampledFrame_{};
    bool AdoptDescriptor(const Facts& facts) noexcept
    {
        const auto latest=latest_.load(std::memory_order_acquire);
        if(f_.session==latest) return true;
        Descriptor d{};
        if(!descriptors_.Read(d).Ready() || d.session!=latest ||
            latest!=latest_.load(std::memory_order_acquire)) return false;
        kernel_.Cancel(EDM81::Event::Reset);
        preview_=EDM90::Preview{};
        // The FIR history is member storage, never a callback-stack temporary.
        using TrajectoryType=EDM91::Preview;
        trajectory_.~TrajectoryType();
        ::new (static_cast<void*>(&trajectory_)) TrajectoryType{};
        using BatchType=EDM92::Bridge;
        batch_.~BatchType();
        ::new (static_cast<void*>(&batch_)) BatchType{};
        using ServoType=EDM93::Simulation;
        servo_.~ServoType();
        ::new (static_cast<void*>(&servo_)) ServoType{};
        rawCompleted_=false;
        descriptor_=d; f_=Feedback{}; f_.scope=d.scope; f_.session=d.session;
        f_.latestSession=d.session; f_.profile=d.profile; f_.scenario=d.scenario; f_.qpcFrequency=d.frequency; f_.source=d.source; f_.durationUs=d.durationUs;
        f_.toolHeadProfileId=d.toolHeadProfileId; f_.toolHeadProfileRevision=d.toolHeadProfileRevision;
        f_.toolHeadCatalogRevision=d.toolHeadCatalogRevision; f_.toolHeadFingerprint=d.toolHeadFingerprint;
        f_.sampledFault=d.sampledFault; f_.sampledConfig=d.sampledConfig; sampledFrame_=EDM88::Frame{};
        f_.sampledDecision.config=d.sampledDecisionConfig;
        f_.rtTick=facts.tick; f_.nowUs=facts.nowUs;
        f_.state=State::AwaitingStart; f_.reason=Reason::AwaitStart;
        f_.sample.scope=d.scope; haveRequest_=false; haveClock_=false;
        lastRuntimeHeartbeatUs_=d.issuedUs; jumpIssued_=shortSeen_=jumpSeen_=false;
        return true;
    }
    bool Authority(const Facts& f) const noexcept
    { return f.authorityReady && f.executionEpoch==f_.scope.executionEpoch && f.ownerLease==f_.scope.ownerLease; }
    bool HeartbeatReady(std::uint64_t nowUs) const noexcept
    { return lastRuntimeHeartbeatUs_ && nowUs>=lastRuntimeHeartbeatUs_ && nowUs-lastRuntimeHeartbeatUs_<=HeartbeatMaximumAgeUs; }
    void ObserveCancel() noexcept
    {
        f_.latestSession=latest_.load(std::memory_order_acquire);
        f_.cancelThroughSession=cancelled_.load(std::memory_order_acquire);
        if(f_.session && f_.session<=f_.cancelThroughSession && f_.state!=State::Cancelled)
        {
            if(IsCommandBatch(f_.profile)) {
                f_.batchCancelTick=f_.rtTick; f_.batchCancelModelUs=f_.modelUs;
                f_.batchCancelPhase=f_.sample.state; f_.batchCancelStage=f_.stage;
            }
            if(IsServoSimulation(f_.profile) && !EDM93::IsEmpty(f_.servoEvidence))
            { servo_.Revoke(); f_.servoEvidence=servo_.Current(); }
            kernel_.Cancel(EDM81::Event::Reset);
            if(IsCommandBatch(f_.profile) && !EDM92::IsEmpty(f_.batchEvidence))
            { batch_.Revoke(); f_.batchEvidence=batch_.Current(); }
            if(IsOutputPreview(f_.profile)) { preview_.Cancel(); f_.previewEvidence=preview_.Current(); }
            if(IsTrajectoryPreview(f_.profile) &&
                (f_.trajectoryEvidence.state==EDM91::State::Running ||
                 f_.trajectoryEvidence.state==EDM91::State::Draining))
            { trajectory_.RequestStop(); f_.trajectoryEvidence=trajectory_.Current(); }
            f_.state=State::Cancelled; f_.reason=Reason::Cancelled;
            f_.terminalTick=f_.rtTick; f_.quiescent=false; f_.quiescentTick=0U;
            f_.sample.state=EDM81::State::Cancelled; f_.sample.reason=EDM81::Reason::Reset;
            f_.sample.committed=false;
        }
    }
    void Trip(Reason reason) noexcept
    {
        if(f_.state!=State::Fault)
        {
            f_.state=State::Fault; f_.reason=reason; f_.terminalTick=f_.rtTick;
            f_.quiescent=false; f_.quiescentTick=0U;
        }
        kernel_.Cancel(EDM81::Event::Alarm);
        if(IsOutputPreview(f_.profile)) { preview_.Cancel(); f_.previewEvidence=preview_.Current(); }
        if(IsTrajectoryPreview(f_.profile) && !EDM91::IsEmpty(f_.trajectoryEvidence))
        { trajectory_.Abort(); f_.trajectoryEvidence=trajectory_.Current(); }
        if(IsCommandBatch(f_.profile) && !EDM92::IsEmpty(f_.batchEvidence))
        { batch_.Abort(); f_.batchEvidence=batch_.Current(); }
        if(IsServoSimulation(f_.profile) && !EDM93::IsEmpty(f_.servoEvidence))
        { servo_.Abort(); f_.servoEvidence=servo_.Current(); }
        f_.sample.state=EDM81::State::Fault;
        f_.sample.committed=false;
    }
    bool TrajectoryTerminalReady() const noexcept
    {
        if(!IsTrajectoryPreview(f_.profile) || EDM91::IsEmpty(f_.trajectoryEvidence)) return true;
        const auto state=f_.trajectoryEvidence.state;
        return state==EDM91::State::Completed || state==EDM91::State::Stopped ||
            state==EDM91::State::Aborted || state==EDM91::State::Retired;
    }
    void ServiceTrajectoryStop(bool clock,bool source) noexcept
    {
        if(!IsTrajectoryPreview(f_.profile) || f_.state!=State::Cancelled ||
            f_.trajectoryEvidence.state!=EDM91::State::Stopping) return;
        // Cancellation intentionally revokes AUTO authority/epoch. This tail
        // uses only frozen numerical values and never republishes a candidate.
        if(!clock || !source)
        {
            trajectory_.Abort(); f_.trajectoryEvidence=trajectory_.Current();
            if(IsCommandBatch(f_.profile)) { batch_.Abort(); f_.batchEvidence=batch_.Current(); }
            if(IsServoSimulation(f_.profile) && !EDM93::IsEmpty(f_.servoEvidence))
            { servo_.Abort(); f_.servoEvidence=servo_.Current(); }
            f_.terminalTick=f_.rtTick; return;
        }
        // A cancel can be observed after Step in this same callback. That
        // callback already advanced the FIR and must not advance the stop too.
        if(f_.trajectoryEvidence.lastTick>=f_.rtTick) return;
        const bool drained=trajectory_.Drain(f_.rtTick); f_.trajectoryEvidence=trajectory_.Current();
        if(IsCommandBatch(f_.profile)) {
            if(!drained) batch_.Abort();
            else if(!batch_.Accept(f_.trajectoryEvidence,EDM92::Kind::OperatorStop,f_.rtTick)) {
                trajectory_.Abort(); f_.trajectoryEvidence=trajectory_.Current();
            }
            f_.batchEvidence=batch_.Current();
        }
        if(IsServoSimulation(f_.profile) && !EDM93::IsEmpty(f_.servoEvidence)) {
            if(!drained || f_.batchEvidence.state==EDM92::State::Aborted ||
                !servo_.Accept(f_.batchEvidence,f_.rtTick)) {
                servo_.Abort(); trajectory_.Abort(); f_.trajectoryEvidence=trajectory_.Current();
                batch_.Abort(); f_.batchEvidence=batch_.Current();
            }
            f_.servoEvidence=servo_.Current();
        }
        if(TrajectoryTerminalReady() && ServoTerminalReady()) f_.terminalTick=f_.rtTick;
    }
    bool ServoControlsCurrent(const EDM93::ControlSnapshot* controller) const noexcept
    {
        return controller && controller->tick==f_.rtTick && servo_.ControlsCurrent(*controller);
    }
    bool ServoTerminalReady() const noexcept
    {
        if(!IsServoSimulation(f_.profile) || EDM93::IsEmpty(f_.servoEvidence)) return true;
        const auto state=f_.servoEvidence.state;
        return state==EDM93::State::Completed || state==EDM93::State::Stopped ||
            state==EDM93::State::Aborted || state==EDM93::State::Retired;
    }
    bool AcceptServo() noexcept
    {
        const bool accepted=servo_.Accept(f_.batchEvidence,f_.rtTick);
        f_.servoEvidence=servo_.Current();
        if(!accepted) { Trip(Reason::ServoSimulation); return false; }
        return true;
    }
    void ServiceServoSettle(bool clock,bool source,const EDM93::ControlSnapshot* controller) noexcept
    {
        if(!IsServoSimulation(f_.profile) || !servo_.NeedsSettle() ||
            (f_.state!=State::Running && f_.state!=State::Cancelled)) return;
        // The model's output has no physical writer. Revoking AUTO prevents
        // admission while its frozen ideal plant may numerically decelerate.
        if(!clock || !source || !ServoControlsCurrent(controller)) {
            servo_.Abort(); f_.servoEvidence=servo_.Current();
            if(f_.state==State::Running) Trip(Reason::ServoSimulation);
            else f_.terminalTick=f_.rtTick;
            return;
        }
        // The last shaped batch already advanced this callback's ideal plant.
        if(f_.servoEvidence.lastTick>=f_.rtTick) return;
        const bool accepted=servo_.Settle(f_.rtTick);
        f_.servoEvidence=servo_.Current();
        if(!accepted) {
            if(f_.state==State::Running) Trip(Reason::ServoSimulation);
            else { servo_.Abort(); f_.servoEvidence=servo_.Current(); f_.terminalTick=f_.rtTick; }
            return;
        }
        if(ServoTerminalReady()) {
            if(f_.state==State::Running) Complete();
            else f_.terminalTick=f_.rtTick;
        }
    }
    void Complete() noexcept
    {
        if(!CompletionCoverageValid(request_,f_)) { Trip(Reason::Coverage); return; }
        f_.state=State::Completed; f_.reason=Reason::None; f_.terminalTick=f_.rtTick;
        f_.quiescent=false; f_.quiescentTick=0U;
    }
    void DrainTrajectory() noexcept
    {
        if(!IsTrajectoryPreview(f_.profile) || !trajectory_.Drain(f_.rtTick))
        { f_.trajectoryEvidence=trajectory_.Current(); Trip(Reason::TrajectoryPreview); return; }
        f_.trajectoryEvidence=trajectory_.Current();
        if(IsCommandBatch(f_.profile)) {
            if(!batch_.Accept(f_.trajectoryEvidence,EDM92::Kind::EndpointDrain,f_.rtTick)) {
                f_.batchEvidence=batch_.Current(); Trip(Reason::CommandBatch); return;
            }
            f_.batchEvidence=batch_.Current();
        }
        if(IsServoSimulation(f_.profile) && !AcceptServo()) return;
        if(f_.trajectoryEvidence.state==EDM91::State::Completed && !IsServoSimulation(f_.profile)) Complete();
    }
    bool MachineCurrent(const Facts& facts,const EDM90::MachineSnapshot* machine) noexcept
    {
        std::uint64_t rejected=machine?EDM90::ReadyFailureMask(*machine):(1ULL<<8U);
        if(machine && (machine->tick!=facts.tick || machine->nowUs!=facts.nowUs)) rejected|=1ULL<<13U;
        if(machine && !EDM90::SameBinding(f_.previewConfig.machine,*machine)) rejected|=1ULL<<14U;
        if(rejected)
        {
            f_.previewMachineRejectMask|=rejected;
            preview_.Invalidate(EDM90::Reason::Machine); f_.previewEvidence=preview_.Current();
            return false;
        }
        return true;
    }
    void Start(const Facts& facts,const EDM90::MachineSnapshot* machine,
        const EDM93::ControlSnapshot* controller) noexcept
    {
        if(request_.session!=f_.session || !EDM81::SameScope(request_.scope,f_.scope) ||
            request_.issuedUs!=descriptor_.issuedUs || request_.qpcFrequency!=descriptor_.frequency ||
            request_.profile!=f_.profile || request_.scenario!=f_.scenario ||
            !ToolHeadFeedbackIdentityValid(request_,f_) || !SampledFeedbackIdentityValid(request_,f_) ||
            request_.source!=f_.source || DurationUs(request_)!=f_.durationUs || !RequestPlanValid(request_) ||
            !ScenarioConfigurationValid(request_) || request_.config.motion.periodUs!=PeriodUs)
        { Trip(Reason::InvalidStart); return; }
        if(!Authority(facts)) { Trip(Reason::AuthorityLost); return; }
        if(!HeartbeatReady(facts.nowUs)) { Trip(Reason::Heartbeat); return; }
        if(IsServoSimulation(f_.profile) && (!controller ||
            !EDM93::ValidControllerFrame(*controller,facts.tick))) { Trip(Reason::ServoSimulation); return; }
        // The RT owner exclusively accesses this member. Reconstruct directly
        // in its existing storage: placement new does not allocate. Avoid the
        // complete Kernel temporary on the fixed RTX64 callback stack while
        // retaining fresh-kernel semantics for each new transport identity.
        using KernelType=EDM84::Kernel;
        kernel_.~KernelType();
        ::new (static_cast<void*>(&kernel_)) KernelType{};
        if(!kernel_.Begin(request_.config,request_.scope,request_.path))
        { f_.kernelReason=kernel_.Current().reason; Trip(Reason::Kernel); return; }
        if(IsOutputPreview(f_.profile))
        {
            f_.previewMachineRejectMask=machine?EDM90::ReadyFailureMask(*machine):(1ULL<<8U);
            if(machine && (machine->tick!=facts.tick || machine->nowUs!=facts.nowUs))
                f_.previewMachineRejectMask|=1ULL<<13U;
            if(f_.previewMachineRejectMask)
            { preview_.Invalidate(EDM90::Reason::Machine); f_.previewEvidence=preview_.Current(); Trip(Reason::OutputPreview); return; }
            auto& config=f_.previewConfig;
            config.session=f_.session; config.scope=f_.scope; config.machine=*machine;
            config.initialPose=kernel_.Current().pose; config.periodUs=PeriodUs;
            if(!preview_.Begin(config))
            {
                // Failed admission grants no frozen output identity. Retiring
                // this exact session must still work with an empty preview.
                f_.previewMachineRejectMask|=1ULL<<15U;
                preview_=EDM90::Preview{}; config=EDM90::Config{};
                f_.previewEvidence=EDM90::Evidence{}; Trip(Reason::OutputPreview); return;
            }
            f_.previewEvidence=preview_.Current();
        }
        if(IsTrajectoryPreview(f_.profile) && !EDM91::Valid(f_.previewConfig))
        {
            // A rejected shaping configuration has no admitted trajectory.
            preview_=EDM90::Preview{}; f_.previewConfig=EDM90::Config{};
            f_.previewEvidence=EDM90::Evidence{}; Trip(Reason::TrajectoryPreview); return;
        }
        if(IsServoSimulation(f_.profile) && !EDM93::ValidConfiguration(f_.previewConfig,*controller))
        {
            // No controller was admitted. Preserve a truly empty receipt so
            // the rejected session can retire and a fresh configuration retry.
            preview_=EDM90::Preview{}; f_.previewConfig=EDM90::Config{};
            f_.previewEvidence=EDM90::Evidence{}; Trip(Reason::ServoSimulation); return;
        }
        // Publish admission metadata before the final asynchronous cancel
        // observation: a cancel here must remain a fully identifiable receipt.
        f_.sample=kernel_.Current(); f_.startedUs=facts.nowUs;
        f_.startedTick=facts.tick; f_.startAcknowledged=true;
        if(IsTrajectoryPreview(f_.profile))
        {
            if(!trajectory_.Begin(f_.previewConfig))
            { f_.trajectoryEvidence=trajectory_.Current(); Trip(Reason::TrajectoryPreview); return; }
            f_.trajectoryEvidence=trajectory_.Current();
            if(IsCommandBatch(f_.profile)) {
                if(!batch_.Begin(f_.previewConfig)) {
                    f_.batchEvidence=batch_.Current(); Trip(Reason::CommandBatch); return;
                }
                f_.batchEvidence=batch_.Current();
            }
        }
        if(IsServoSimulation(f_.profile)) {
            if(!controller || controller->tick!=f_.rtTick || !servo_.Begin(f_.previewConfig,*controller)) {
                f_.servoEvidence=servo_.Current(); Trip(Reason::ServoSimulation); return;
            }
            f_.servoEvidence=servo_.Current();
        }
        ObserveCancel(); if(f_.state==State::Cancelled) return;
        f_.state=State::Running; f_.reason=Reason::None;
    }
    bool CheckComposition(const EDM84::Snapshot& previous,const EDM84::Snapshot& candidate) noexcept
    {
        EDM81::Pose canonical{}; unsigned ordinal=0U; double u=0.0;
        if(!request_.path.Evaluate(candidate.cursorMm,canonical,ordinal,u)) return false;
        if(ordinal!=candidate.ordinal || std::fabs(u-candidate.u)>1e-10) return false;
        for(unsigned axis=0U;axis<6U;++axis)
        {
            const double error=std::fabs(candidate.nominal[axis]-canonical[axis]);
            if(!EDM81::Finite(error) || error>1e-10) return false;
            double& maximum=axis<3U?f_.maxLinearErrorMm:f_.maxRotaryErrorDeg;
            maximum=EDM81::Maximum(maximum,error);
            if(candidate.pose[axis]!=previous.pose[axis]) f_.axisMovedMask|=1U<<axis;
            if(candidate.nominal[axis]!=previous.nominal[axis]) f_.nominalAxisMovedMask|=1U<<axis;
        }
        f_.coverageMask|=CanonicalCorrespondence;
        const unsigned nativeMoved=IsAuthored(f_.profile)?f_.nominalAxisMovedMask:f_.axisMovedMask;
        if(nativeMoved&(1U<<3U)) f_.coverageMask|=CMoving;
        if(nativeMoved&(1U<<4U)) f_.coverageMask|=UMoving;
        if(nativeMoved&(1U<<5U)) f_.coverageMask|=VMoving;
        const bool cuv=request_.scenario==Scenario::ToolHeadCUVCompensation;
        if(cuv)
        {
            std::array<double,3U> currentDirection{};
            if(!EDM84::ResolveDirection(request_.config.direction,request_.config.mapping,
                candidate.nominal,currentDirection)) return false;
            double normSquared=0.0;
            for(const double value:currentDirection) normSquared+=value*value;
            const double error=std::fabs(normSquared-1.0);
            if(!EDM81::Finite(error) || error>1e-10 ||
                f_.orientationSamples==(std::numeric_limits<std::uint64_t>::max)()) return false;
            f_.resolvedDirectionUnit=currentDirection;
            f_.maxOrientationNormError=EDM81::Maximum(f_.maxOrientationNormError,error);
            ++f_.orientationSamples; f_.coverageMask|=OrientationResolved;
        }
        const bool directional=request_.config.excursion==EDM84::ExcursionMode::DirectionOffset;
        const bool previousExcursion=previous.state==EDM81::State::ShortRetreat ||
            previous.state==EDM81::State::JumpRetreat || previous.state==EDM81::State::Returning;
        const bool candidateExcursion=candidate.state==EDM81::State::ShortRetreat ||
            candidate.state==EDM81::State::JumpRetreat || candidate.state==EDM81::State::Returning;
        if(directional && candidateExcursion && !previousExcursion) frozenJoint_=previous.pose;
        if(directional && (candidateExcursion || previousExcursion))
        {
            if(candidate.cursorMm!=candidate.anchorMm) return false;
            for(unsigned axis=0U;axis<6U;++axis)
                if(candidate.nominal[axis]!=candidate.anchorPose[axis]) return false;
            for(unsigned axis=3U;axis<6U;++axis)
                if(candidate.pose[axis]!=frozenJoint_[axis]) return false;
            std::array<double,3> direction{};
            if(!EDM84::ResolveDirection(request_.config.direction,request_.config.mapping,
                candidate.anchorPose,direction)) return false;
            for(unsigned axis=0U;axis<3U;++axis)
            {
                const double error=std::fabs(direction[axis]-candidate.frozenDirection[axis]);
                if(!EDM81::Finite(error) || error>1e-12) return false;
                if(cuv) f_.maxFrozenDirectionError=EDM81::Maximum(f_.maxFrozenDirectionError,error);
            }
            f_.coverageMask|=OffsetFrozen|DirectionFrozen;
            if(candidate.state==EDM81::State::Running)
            {
                if(candidate.offsetMm!=0.0 || candidate.pose!=frozenJoint_) return false;
                f_.coverageMask|=OffsetReturned;
            }
        }
        std::array<double,3> recovered{};
        if(!EDM84::RecoverToolPoint(request_.config.mapping,candidate.pose,recovered)) return false;
        for(unsigned axis=0U;axis<3U;++axis)
        {
            const double expected=candidate.nominal[axis]+candidate.frozenDirection[axis]*candidate.offsetMm;
            const double error=std::fabs(recovered[axis]-expected);
            if(!EDM81::Finite(error) || error>1e-10) return false;
            f_.maxMappingErrorMm=EDM81::Maximum(f_.maxMappingErrorMm,error);
            if(candidate.offsetMm!=0.0) f_.maxOffsetErrorMm=EDM81::Maximum(f_.maxOffsetErrorMm,error);
        }
        if(f_.scenario==Scenario::ElectrodeCCompensation &&
            request_.config.mapping.kind==EDM84::MappingKind::FixedZC &&
            (IsAuthored(f_.profile) || (f_.axisMovedMask&(1U<<3U)))) f_.coverageMask|=MappingRoundTrip;
        if(cuv && request_.config.mapping.kind==EDM84::MappingKind::SerialHeadCUV &&
            request_.config.direction.frame==EDM84::DirectionFrame::ElectrodeCUV)
            f_.coverageMask|=MappingRoundTrip;
        return true;
    }
    void SampleFrame(double voltageV,std::uint64_t nowUs) noexcept
    {
        // The publisher is deliberately slower than the control callback.
        // Repeated reads retain the original source sequence and timestamp.
        if(f_.modelUs>=2000000ULL && request_.sampledFault!=SampledFault::Normal)
        {
            if(request_.sampledFault==SampledFault::SequenceRollback && f_.modelUs==2000000ULL)
                --sampledFrame_.sequence;
            else if(request_.sampledFault==SampledFault::ChangedDuplicate && f_.modelUs==2000000ULL)
                sampledFrame_.voltageV+=1.0;
            return;
        }
        if(!sampledFrame_.valid || f_.modelUs%request_.config.gapSource.samplePeriodUs==0U)
        {
            sampledFrame_.identity=request_.config.gapSource.identity;
            ++sampledFrame_.sequence; sampledFrame_.capturedAtUs=nowUs;
            sampledFrame_.voltageV=voltageV; sampledFrame_.valid=true;
        }
    }
    void Step() noexcept
    {
        ObserveCancel(); if(f_.state!=State::Running) return;
        if(f_.modelUs>=f_.durationUs+3000000ULL) { Trip(Reason::Coverage); return; }
        const auto previous=f_.sample;
        EDM81::Input input{}; input.scope=f_.scope;
        input.sequence=f_.runtimeSteps+1U;
        input.nowUs=input.capturedAtUs=f_.modelUs+PeriodUs;
        input.voltageV=request_.config.motion.referenceV+10.0;
        f_.stage=0U;
        if(f_.modelUs>=1000000U && f_.modelUs<1500000U)
        { input.voltageV=request_.config.motion.referenceV-10.0; f_.stage=1U; }
        else if(f_.modelUs>=1500000U && f_.modelUs<3000000U) f_.stage=2U;
        else if(f_.modelUs>=3000000U && f_.modelUs<ShortReleaseUs(f_.profile))
        { input.voltageV=0.0; f_.stage=3U; }
        else if(f_.modelUs>=ShortReleaseUs(f_.profile) && f_.modelUs<JumpStartUs(f_.profile)-1000000ULL) f_.stage=4U;
        else if(f_.modelUs>=JumpStartUs(f_.profile)-1000000ULL && f_.modelUs<JumpStartUs(f_.profile)) f_.stage=5U;
        else if(f_.modelUs>=JumpStartUs(f_.profile))
        {
            f_.stage=6U;
            if(!jumpIssued_) { input.event=EDM81::Event::Jump; jumpIssued_=true; }
            else if(previous.state==EDM81::State::Running) f_.stage=7U;
        }
        // EDM89 probes operate only on this private simulated waveform. One
        // low acquisition cannot satisfy a positive entry dwell; equality at
        // the release boundary interrupts a pending exit confirmation.
        if(IsSourceDwell(request_.profile))
        {
            if(f_.modelUs>=500000U && f_.modelUs<502000U) input.voltageV=0.0;
            else if(f_.modelUs>=ShortReleaseUs(f_.profile)+2000ULL && f_.modelUs<ShortReleaseUs(f_.profile)+4000ULL)
                input.voltageV=request_.config.motion.servo.shortMachiningV+
                    request_.config.motion.servo.shortHysteresisV;
        }
        if(f_.modelUs>=f_.durationUs && previous.state==EDM81::State::Running &&
            previous.cursorMm==previous.pathLengthMm)
        {
            const auto required=ExpectedCoverage(request_)^CanonicalComplete;
            const bool axes=IsAuthored(f_.profile)?
                (f_.nominalAxisMovedMask==request_.expectedAxisMask && f_.visitedRowMask==request_.expectedRowMask):
                f_.axisMovedMask==0x3FU;
            if(!axes || (f_.coverageMask & required)!=required)
            { Trip(Reason::Coverage); return; }
            input.event=EDM81::Event::M30;
        }
        // Qualified source time is independent from the nominal control time.
        // This explicit simulation publisher never reads or changes AD/PDO.
        if(IsSampled(request_.profile)) SampleFrame(input.voltageV,input.nowUs);
        const auto candidate=IsSampled(request_.profile)?
            kernel_.StepSampled(input,sampledFrame_,input.nowUs):kernel_.Step(input);
        if(IsSampled(request_.profile))
        { f_.sampledSource=kernel_.GapSourceEvidence(); f_.sampledDecision=kernel_.GapDecisionEvidence(); }
        ObserveCancel(); if(f_.state!=State::Running) return;
        f_.sample=candidate; ++f_.runtimeSteps; f_.modelUs+=PeriodUs;
        f_.voltageV=IsSampled(request_.profile)?sampledFrame_.voltageV:input.voltageV; f_.stageMask|=1ULL<<f_.stage;
        if(candidate.state==EDM81::State::Fault || candidate.state==EDM81::State::Cancelled)
        { f_.kernelReason=candidate.reason; Trip(IsSampled(request_.profile) && f_.sampledSource.faulted?
                Reason::SampledGap:Reason::Kernel); return; }
        if(candidate.committed) f_.coverageMask|=Fresh;
        if(candidate.state==EDM81::State::Running && candidate.cursorMm>previous.cursorMm)
            f_.coverageMask|=GapForward;
        if(f_.stage==1U && candidate.state==EDM81::State::Running && candidate.cursorMm<previous.cursorMm)
            f_.coverageMask|=GapReverse;
        if(!IsAuthored(f_.profile))
        {
            if(candidate.ordinal==1U) f_.coverageMask|=InitialLine;
            if(candidate.ordinal==2U) f_.coverageMask|=Arc;
            if(candidate.ordinal==3U) f_.coverageMask|=FinalLine;
        }
        else if(candidate.committed && candidate.cursorMm!=previous.cursorMm)
        {
            // Include each positive-length crossed interval, even when one
            // callback passes a short row completely. Reaching a seam alone
            // never gives credit for motion in the following row.
            const double low=EDM81::Minimum(candidate.cursorMm,previous.cursorMm);
            const double high=EDM81::Maximum(candidate.cursorMm,previous.cursorMm);
            for(unsigned row=0U;row<request_.path.Count();++row)
                if(high>request_.path.Station(row) && low<request_.path.Station(row+1U))
                {
                    f_.visitedRowMask|=1U<<row;
                    const auto* geometry=request_.path.Row(row);
                    if(geometry->kind==NCPathCoreRetainedKind::LINE) f_.coverageMask|=InitialLine;
                    if(geometry->kind==NCPathCoreRetainedKind::ARC) f_.coverageMask|=Arc;
                }
            if(f_.visitedRowMask==request_.expectedRowMask) f_.coverageMask|=FinalLine;
        }
        if(candidate.state==EDM81::State::ShortRetreat &&
            (candidate.cursorMm<previous.cursorMm || candidate.offsetMm<previous.offsetMm))
        { shortSeen_=true; f_.coverageMask|=ShortRetreat; }
        if(shortSeen_ && !jumpIssued_ && previous.state==EDM81::State::Returning &&
            candidate.state==EDM81::State::Running && candidate.cursorMm==candidate.anchorMm)
            f_.coverageMask|=ShortReturn;
        if(candidate.state==EDM81::State::JumpRetreat &&
            (candidate.cursorMm<previous.cursorMm || candidate.offsetMm<previous.offsetMm))
        { jumpSeen_=true; f_.coverageMask|=JumpRetreat; }
        if(jumpSeen_ && previous.state==EDM81::State::Returning &&
            candidate.state==EDM81::State::Running && candidate.cursorMm==candidate.anchorMm)
            f_.coverageMask|=JumpReturn;
        if(!CheckComposition(previous,candidate)) { Trip(Reason::Coverage); return; }
        if(IsOutputPreview(f_.profile))
        {
            EDM90::Frame frame{}; frame.session=f_.session; frame.scope=f_.scope;
            frame.tick=f_.rtTick; frame.sequence=f_.runtimeSteps; frame.pose=candidate.pose;
            if(!preview_.Step(frame))
            { f_.previewEvidence=preview_.Current(); Trip(Reason::OutputPreview); return; }
            if(candidate.state==EDM81::State::Completed) preview_.Finish();
            f_.previewEvidence=preview_.Current();
            if(IsTrajectoryPreview(f_.profile))
            {
                if(!trajectory_.Step(frame))
                { f_.trajectoryEvidence=trajectory_.Current(); Trip(Reason::TrajectoryPreview); return; }
                f_.trajectoryEvidence=trajectory_.Current();
                if(IsCommandBatch(f_.profile)) {
                    if(!batch_.Accept(f_.trajectoryEvidence,EDM92::Kind::Raw,f_.rtTick)) {
                        f_.batchEvidence=batch_.Current(); Trip(Reason::CommandBatch); return;
                    }
                    f_.batchEvidence=batch_.Current();
                    if(IsServoSimulation(f_.profile) && !AcceptServo()) return;
                }
                if(candidate.state==EDM81::State::Completed) trajectory_.Finish();
                f_.trajectoryEvidence=trajectory_.Current();
            }
        }
        if(candidate.cursorMm==candidate.pathLengthMm) f_.coverageMask|=Endpoint;
        if(candidate.state==EDM81::State::Completed)
        {
            f_.coverageMask|=CanonicalComplete;
            if(IsTrajectoryPreview(f_.profile)) rawCompleted_=true;
            else Complete();
        }
    }
};
}
