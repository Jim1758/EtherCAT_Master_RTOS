#include "NCManager.h"
#include "NCGCodeSemantics.h"
#include "AlarmManager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <windows.h>
#include <rtapi.h>

namespace {
bool SameIdentity(const MotionExecutionIdentity& a,const MotionExecutionIdentity& b) noexcept
{ return a.IsAssigned()&&b.IsAssigned()&&a.Matches(b)&&a.source==b.source&&a.sourceBlockId==b.sourceBlockId; }
bool SameSource(const NCProgramCommitSnapshot& a,const NCProgramCommitSnapshot& b) noexcept
{ return a.scope==b.scope&&a.cacheGeneration==b.cacheGeneration&&a.frameId==b.frameId&&a.sourcePC==b.sourcePC; }
bool SameDouble(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
bool Healthy(const NCBlockLifecycleLedger& ledger) noexcept {
    const auto c=ledger.GetCounters();return !c.activeBlockOverwrite&&!c.activeSegmentIndexOverwrite&&
        !c.orphanFeedback&&!c.duplicateTerminalFeedback&&!c.terminalFeedbackConflict;
}
// Formatting stays on the NC thread. RtPrintf receives only a bounded string.
void PrintSphere(const char* format,...) noexcept {
    char line[481]{};va_list args;va_start(args,format);
    const int n=std::vsnprintf(line,sizeof(line),format,args);va_end(args);
    if(n>=0&&static_cast<std::size_t>(n)<sizeof(line))RtPrintf("%s",line);
    else RtPrintf("%s","[P50] build=EDM287_FIX1 event=DIAGNOSTIC_FORMAT_FAILURE\n");
}
const char* PhaseName(NCG76SphereCycle::Phase phase) noexcept {
    using P=NCG76SphereCycle::Phase;
    switch(phase){
    case P::Idle:return "IDLE";case P::InitialDepth:return "INITIAL_DEPTH";
    case P::InitialRetreat:return "INITIAL_RETREAT";case P::CircleEntry:return "CIRCLE_ENTRY";
    case P::Orbit:return "ORBIT";case P::ReturnCenter:return "RETURN_CENTER";
    case P::AdvanceCenter:return "ADVANCE_CENTER";case P::Settle:return "SETTLE";
    case P::HoldBrake:return "HOLD_BRAKE";case P::HoldSettle:return "HOLD_SETTLE";
    case P::HoldToCenter:return "HOLD_TO_CENTER";case P::HoldToStart:return "HOLD_TO_START";
    case P::HeldAtStart:return "HELD_AT_START";case P::ResumeToCenter:return "RESUME_TO_CENTER";
    case P::ResumeToAnchor:return "RESUME_TO_ANCHOR";case P::FaultBrake:return "FAULT_BRAKE";
    case P::FaultSettle:return "FAULT_SETTLE";case P::Complete:return "COMPLETE";case P::Fault:return "FAULT";
    default:return "UNKNOWN";}
}
const char* RuntimeFaultName(std::uint32_t fault) noexcept {
    switch(fault){
    case 0U:return "NONE";case 1U:return "INPUT";case 2U:return "AUTHORITY";
    case 3U:return "CYCLE";case 4U:return "SETTLE";case 5U:return "PARENT_BIND_TIMEOUT";
    case 6U:return "CONTROL_INTENT";case 7U:return "START_POSITION_TIMEOUT";
    case 8U:return "START_TRAVEL";default:return "UNKNOWN";}
}
bool FailedSphereFeedback(const MotionG76SphereFeedback& report) noexcept {
    return report.runtimeFault||report.cycle.fault!=NCG76SphereCycle::Fault::None||
        (report.terminal&&!report.completed);
}
// Diagnostic-only readback remains useful after the RT failure revokes its
// owner/epoch. It never grants motion or releases the NC completion barrier.
void PrintSphereRuntimeFailure(const MotionG76SphereFeedback& report,
    const NCG76SphereCycle::Request& request,int line) noexcept {
    PrintSphere("[P50] build=EDM287_FIX1 event=FAULT session=%llu line=%d tick=%llu phase=%s cycleFault=%u runtimeFault=%u gate=%s started=%u discharge=0\n",
        static_cast<unsigned long long>(report.session),line,static_cast<unsigned long long>(report.inputSequence),
        PhaseName(report.cycle.phase),static_cast<unsigned>(report.cycle.fault),report.runtimeFault,
        RuntimeFaultName(report.runtimeFault),report.started?1U:0U);
    const auto& target=report.started?report.commandMCS:request.geometry.startMCS;
    for(unsigned axis=0U;axis<6U;++axis){
        // Motion may tighten both tolerances to the installed axis windows.
        // Report the actual frozen RT values, not the NC request defaults.
        const double tolerance=axis<3U?report.positionToleranceMm:report.rotaryToleranceDeg;
        const double error=report.actualMCS[axis]-target[axis];
        PrintSphere("[P50] build=EDM287_FIX1 event=FAULT_AXIS session=%llu axis=%u actual=%.12f target=%.12f error=%.12f tolerance=%.12f unit=%s inWindow=%u discharge=0\n",
            static_cast<unsigned long long>(report.session),axis,report.actualMCS[axis],target[axis],
            error,tolerance,axis<3U?"mm":"deg",std::isfinite(tolerance)&&tolerance>0.0&&
                std::isfinite(error)&&std::fabs(error)<=tolerance?1U:0U);
    }
}
}

bool NCManager::IsG76SphereBlock(const NCBlock& block) noexcept
{ return NCGCodeSemantics::Contains(block,76)&&block.has('P')&&block.val('P')==50.0; }

NCG76SphereSource::ModalB NCManager::CaptureG76SphereModalBSameThread() const noexcept
{
    NCG76SphereSource::ModalB value{};value.value=m_g76Modal.mode;value.valid=m_g76Modal.valid;
    value.commit=m_g76Modal.commit;value.committed.run=m_g76Modal.run;value.committed.cache=m_g76Modal.cache;
    value.committed.lease=m_g76Modal.lease;value.current.run=m_pathCoreLiveBookkeeping.currentRunToken;
    value.current.cache=GetBaseProgramCache().GetGeneration();value.current.lease=m_programMotionLease;
    return value;
}

void NCManager::RejectG76SphereSameThread(const char* reason,int alarm,int sourceLine)
{
    const auto session=(m_g76Sphere->active||m_g76Sphere->staged||
        (m_g76Sphere->submitted&&!m_g76Sphere->completed))?m_g76Sphere->session:0ULL;
    const int line=sourceLine?sourceLine:m_g76Sphere->line;
    const bool submitted=m_g76Sphere->submitted&&!m_g76Sphere->completed;
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_REJECT");
    m_g76Sphere->active=m_g76Sphere->staged=m_g76Sphere->resumePending=false;
    AlarmManager::GetInstance().Trigger(alarm,line);ChangeState(NCState::ALARM);
    if(submitted)m_motion.RequestEmergencyStopAllAxes();
    PrintSphere("[P50] build=EDM287_FIX1 event=REJECT session=%llu line=%d reason=%s discharge=0\n",
        static_cast<unsigned long long>(session),line,reason?reason:"UNSPECIFIED");
}

bool NCManager::ValidateG76SphereBlockSameThread(const NCBlock& block,int pc,int line)
{
    const auto* source=GetBaseProgramCache().TryGetLine(pc);
    NCG76SphereSource::Words words{};
    const auto status=source?NCG76SphereSource::Decode(source->parsedBlock,CaptureG76SphereModalBSameThread(),words):
        NCG76SphereSource::Status::InvalidSource;
    bool valid=source&&(!line||source->sourceLineNumber==line)&&status==NCG76SphereSource::Status::Ok&&
        IsG76SphereBlock(block)&&block.hasG&&block.gCount==1&&block.gCode==76&&block.gCodes[0]==76&&
        block.mCount==0&&!block.isEmpty&&!block.isGoto&&!block.isBlockSkip&&
        block.gotoTarget==words.authored.gotoTarget&&m_macroStack.empty()&&!m_isG66Active;
    for(char c='A';c<='Z'&&valid;++c)valid=block.has(c)==words.authored.has(c)&&
        SameDouble(block.val(c),words.authored.val(c));
    for(int i=0;i<NC_MAX_G_CODES_PER_BLOCK&&valid;++i)valid=block.gCodes[i]==words.authored.gCodes[i];
    for(int i=0;i<NC_MAX_M_CODES_PER_BLOCK&&valid;++i)valid=block.mCode[i]==words.authored.mCode[i];
    if(!valid){RejectG76SphereSameThread(status==NCG76SphereSource::Status::UnresolvedModalB?
        "B_REQUIRES_COMMITTED_SOURCE":"EXACT_LITERAL_SOURCE",AlarmManager::G_Code_Invalid_parameter,
        source?source->sourceLineNumber:line);return false;}
    // B remains a committed process choice. This release executes B0 only;
    // a nonzero inherited B cannot silently become an AIR/no-flush request.
    if(words.requestedB!=0U){RejectG76SphereSameThread("P50_B_PROCESS_NOT_RELEASED",AlarmManager::G_Code_Invalid_parameter,
        source->sourceLineNumber);return false;}
    if(m_g76Sphere->active){RejectG76SphereSameThread("P50_PARENT_STILL_ACTIVE",AlarmManager::PATH_EXECUTION_NOT_READY,
        source->sourceLineNumber);return false;}
    return true;
}

bool NCManager::PrepareG76SphereTranslationSameThread(const NCBlock& block)
{
    const int pc=GetActiveDispatchPC();const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(!ValidateG76SphereBlockSameThread(block,pc,source?source->sourceLineNumber:0))return false;
    // Allocate NC-only preparation storage once per parent; no large temporary
    // session is constructed on the NC stack or on any servo callback.
    m_g76Sphere.reset(new G76SphereNCState{});auto& s=*m_g76Sphere;
    s.pc=pc;s.line=source->sourceLineNumber;s.dispatch=m_currentExecutingBlockDispatchId;
    const auto reject=[this](const char* reason,int alarm){RejectG76SphereSameThread(reason,alarm);return false;};
    if(m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||!IsEDM171AirGrantCurrentSameThread()||
        !s.dispatch||s.dispatch<=m_g76Confirmation.dispatch||pc<m_g76Confirmation.firstPC||
        Homing.IsActive()||IsFeedHoldActive()||m_motion.HasPendingSafetyOrRecoveryRequests()||
        m_g80.active||m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending||m_pathReplay.pending||
        m_pathHold.armed||m_pathHold.bound||m_cncFeed.active||m_cncFeed.selected||m_cncFeed.count||
        m_gapDryRun.active||m_gapPath.active||m_gapWindow.active||m_edm94.latched||
        CoordSys.toolRadiusMode!=40||CoordSys.IsManualFrameEnabled()||m_cutterLine.leadOutRequired)
        return reject("AIR_GRANT_OR_RUNTIME_SCOPE",AlarmManager::PATH_EXECUTION_NOT_READY);
    s.run=m_pathCoreLiveBookkeeping.currentRunToken;s.cache=GetBaseProgramCache().GetGeneration();
    s.owner=m_motion.GetMotionOwnerLease();s.epoch=m_motion.GetCurrentExecutionEpoch();
    if(!s.run||!s.cache||!s.owner.IsValid()||s.owner.owner!=MotionOwner::AUTO||
        !s.owner.Matches(m_programMotionLease)||!s.epoch||!m_motion.IsGroupNCDrained()||!m_motion.IsGroupDone()||
        m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||
        !m_motion.HasExactExecutionDrainAcknowledgement(s.epoch,s.owner)||
        m_motion.GetMotionFeedbackOverflowCount()||m_motion.GetMotionFeedbackProducerNoticeOverflowCount()||
        m_motionFeedbackSequenceGapCount||!Healthy(m_blockLifecycleLedger))
        return reject("EXACT_DRAIN_OWNER_OR_TRANSPORT",AlarmManager::PATH_EXECUTION_NOT_READY);
    s.frame=CoordSys.GetTranslationSnapshot();
    if(!CoordSys.IsTranslationRunBound()||!CoordSys.IsTranslationRunCurrent()||
        !IsNCTranslationSnapshotValid(s.frame)||!IsPathCoreLiveNativeConfigCurrentSameThread())
        return reject("FRAME_OR_NATIVE_CONFIG",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    NCG76SphereCoordinates::Point8 start{};
    for(unsigned a=0U;a<8U;++a)start[a]=CoordSys.commandedMCS[a];
    if(NCG76SphereArchitecture::Prepare(source->parsedBlock,s.frame,start,NCG76Sphere::Limits{},
        CaptureG76SphereModalBSameThread(),s.prepared,s.workspace)!=NCG76SphereArchitecture::Status::Ok||
        !NCG76SphereArchitecture::Valid(s.prepared)||s.prepared.requestedB!=0U||
        !NCG76SphereRuntimeGeometry::Build(s.prepared.native,s.request.geometry)||
        !CaptureCncFeedValueSameThread(block,s.feed)||
        s.feed.feedMMMin!=s.prepared.native.descriptor.input.feedMmMin)
        return reject("SOURCE_GEOMETRY_OR_FEED",AlarmManager::G_Code_Invalid_parameter);
    s.request.feedMmMin=s.feed.feedMMMin;s.request.returnFeedMmMin=s.feed.feedMMMin;
    s.request.velocityCapMmS=(std::numeric_limits<double>::max)();
    s.request.accelerationMmS2=s.request.decelerationMmS2=(std::numeric_limits<double>::max)();
    s.request.settleSamples=200U;
    for(unsigned i=0U;i<8U;++i){
        const auto& axis=m_motion.GetAxisContext(static_cast<int>(i));
        s.request.travelMin[i]=-(std::numeric_limits<double>::max)();
        s.request.travelMax[i]=(std::numeric_limits<double>::max)();
        if(i<3U){
            const double ppu=axis.resolution_PPR/axis.finalLead;
            const double velocity=axis.maxVel_PPS/ppu;
            if(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||axis.axisType!=AxisType::LINEAR||
                !std::isfinite(ppu)||ppu<=0.0||!std::isfinite(velocity)||velocity<=0.0||
                !std::isfinite(axis.G00_acc_time)||axis.G00_acc_time<0.0||
                !std::isfinite(axis.G00_dec_time)||axis.G00_dec_time<0.0||
                !std::isfinite(axis.Stop_dec_time)||axis.Stop_dec_time<0.0)
                return reject("XYZ_PROFILE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            // Installed zero means the existing Motion 0.2 s default; retain
            // that policy rather than requiring a new parameter-file edit.
            const double accTime=axis.G00_acc_time<0.001?0.2:axis.G00_acc_time;
            const double decTime=(std::max)(axis.G00_dec_time<0.001?0.2:axis.G00_dec_time,
                axis.Stop_dec_time<0.001?0.2:axis.Stop_dec_time);
            const double accel=velocity/accTime,decel=velocity/decTime;
            if(!std::isfinite(accel)||accel<=0.0||!std::isfinite(decel)||decel<=0.0)
                return reject("XYZ_PROFILE_RANGE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            s.request.velocityCapMmS=(std::min)(s.request.velocityCapMmS,velocity);
            s.request.accelerationMmS2=(std::min)(s.request.accelerationMmS2,accel);
            s.request.decelerationMmS2=(std::min)(s.request.decelerationMmS2,decel);
        }
        if(!axis.isExist)continue;
        const bool enabled[3]={axis.travelLimit1Enable&&CoordSys.IsProgrammableTravelLimitEnabled(),axis.travelLimit2Enable,axis.travelLimit3Enable};
        const double low[3]={axis.travelLimit1Negative_unit,axis.travelLimit2Negative_unit,axis.travelLimit3Negative_unit};
        const double high[3]={axis.travelLimit1Positive_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Positive_unit};
        for(unsigned n=0U;n<3U;++n)if(enabled[n]){
            if(!std::isfinite(low[n])||!std::isfinite(high[n])||low[n]>high[n])
                return reject("TRAVEL_CONFIG",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            s.request.travelMask|=1U<<i;s.request.travelMin[i]=(std::max)(s.request.travelMin[i],low[n]);
            s.request.travelMax[i]=(std::min)(s.request.travelMax[i],high[n]);
        }
        if(s.request.travelMin[i]>s.request.travelMax[i]||
            s.request.geometry.boundsMinMCS[i]<s.request.travelMin[i]||
            s.request.geometry.boundsMaxMCS[i]>s.request.travelMax[i]||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(axis,start[i]))
            return reject("WHOLE_CYCLE_TRAVEL_ENVELOPE",AlarmManager::PROGRAMMED_OVER_TRAVEL);
    }
    if(!m_g76SphereNextSession||m_g76SphereNextSession==(std::numeric_limits<std::uint64_t>::max)())
        return reject("SESSION_EXHAUSTED",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    s.session=m_g76SphereNextSession++;s.request.session=s.session;
    if(!NCG76SphereCycle::ValidateRequest(s.request))return reject("CYCLE_REQUEST",AlarmManager::G_Code_Invalid_parameter);
    if(!CoordSys.FreezeTranslationRun()||!SameNCTranslationSnapshot(s.frame,CoordSys.GetTranslationSnapshot())||
        (!m_motion.MatchesNCTranslation(s.frame)&&!m_motion.PublishNCTranslation(s.frame)))
        return reject("FRAME_FREEZE_PUBLICATION",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    for(unsigned i=0U;i<8U;++i){
        const auto& a=m_motion.GetAxisContext(static_cast<int>(i));auto& c=m_fixedTranslationTravel[i];
        c.minimum[0]=a.travelLimit1Negative_unit;c.minimum[1]=a.travelLimit2Negative_unit;c.minimum[2]=a.travelLimit3Negative_unit;
        c.maximum[0]=a.travelLimit1Positive_unit;c.maximum[1]=a.travelLimit2Positive_unit;c.maximum[2]=a.travelLimit3Positive_unit;
        c.enabled[0]=a.travelLimit1Enable;c.enabled[1]=a.travelLimit2Enable;c.enabled[2]=a.travelLimit3Enable;
        c.homed=a.isHomed;c.positionalRotary=a.isExist&&a.axisType==AxisType::ROTARY;
        c.shortestPath=a.useShortestPath;c.rotaryModulo=a.rotaryModulo;c.maximumVelocity=a.maxVel_PPS;
        c.accelerationTime=a.G00_acc_time;c.decelerationTime=a.G00_dec_time;c.stopDecelerationTime=a.Stop_dec_time;
    }
    m_fixedTranslationTravelGeneration=s.frame.generation;
    if(s.frame.rotationMode==68)m_motion.SetCoordinateTransform(false,0.0,0.0,0.0,0.0,0.0,0.0);
    s.staged=true;return true;
}

bool NCManager::IsG76SphereScopeCurrentSameThread()
{
    const auto& s=*m_g76Sphere;
    const auto reject=[&](const char* gate,int axis=-1){
        PrintSphere("[P50] build=EDM287_FIX1 event=SCOPE_REJECT session=%llu line=%d gate=%s axis=%d expectedEpoch=%llu actualEpoch=%llu discharge=0\n",
            static_cast<unsigned long long>(s.session),s.line,gate,axis,
            static_cast<unsigned long long>(s.epoch),static_cast<unsigned long long>(m_motion.GetCurrentExecutionEpoch()));
        return false;
    };
    if(!IsEDM171AirGrantCurrentSameThread())return reject("AIR_GRANT");
    if(s.run!=m_pathCoreLiveBookkeeping.currentRunToken||s.cache!=GetBaseProgramCache().GetGeneration())return reject("RUN_SOURCE");
    if(!s.owner.Matches(m_programMotionLease)||!m_motion.IsMotionOwnerLeaseCurrent(s.owner))return reject("OWNER");
    if(m_motion.GetCurrentExecutionEpoch()!=s.epoch)return reject("EPOCH");
    if(!CoordSys.IsTranslationRunFrozen()||!CoordSys.IsTranslationRunCurrent())return reject("COORDINATE_SCOPE");
    if(!SameNCTranslationSnapshot(s.frame,CoordSys.GetTranslationSnapshot())||!m_motion.MatchesNCTranslation(s.frame))return reject("FRAME");
    if(!IsPathCoreLiveNativeConfigCurrentSameThread())return reject("NATIVE_CONFIG");
    if(!IsFixedTranslationTravelCurrentSameThread())return reject("TRAVEL_CONFIG");
    if(m_motion.GetMotionFeedbackOverflowCount()||m_motion.GetMotionFeedbackProducerNoticeOverflowCount()||
        m_motionFeedbackSequenceGapCount)return reject("FEEDBACK_TRANSPORT");
    if(!Healthy(m_blockLifecycleLedger))return reject("LEDGER_INTEGRITY");
    // XYZ ramps and stop times belong to this complete native envelope as well.
    for(unsigned i=0U;i<8U;++i){const auto& a=m_motion.GetAxisContext(static_cast<int>(i));
        const auto& c=m_fixedTranslationTravel[i];if(a.isExist&&
            (!SameDouble(c.maximumVelocity,a.maxVel_PPS)||!SameDouble(c.accelerationTime,a.G00_acc_time)||
             !SameDouble(c.decelerationTime,a.G00_dec_time)||!SameDouble(c.stopDecelerationTime,a.Stop_dec_time)))return reject("AXIS_PROFILE",static_cast<int>(i));
    }
    return true;
}

WaitConditionFunc NCManager::StartG76SphereSameThread(const NCBlock& block)
{
    if(!m_g76Sphere->staged&&!PrepareG76SphereTranslationSameThread(block))return nullptr;
    auto& s=*m_g76Sphere;
    if(s.pc!=GetActiveDispatchPC()||s.dispatch!=m_currentExecutingBlockDispatchId||
        !ValidateG76SphereBlockSameThread(block,s.pc,s.line)||!IsG76SphereScopeCurrentSameThread()){
        RejectG76SphereSameThread("SUBMIT_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;
    }
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_AIR_PARENT");
    MotionG76SphereAdmission admission{};
    if(!m_motion.TryG76SphereMove(s.request,s.frame,s.epoch,s.owner,s.pc,s.identity,&admission)){
        PrintSphere("[P50] build=EDM287_FIX1 event=ADMISSION_REJECT session=%llu line=%d gate=%s axis=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),s.line,MotionG76SphereAdmissionGateName(admission.gate),admission.axis);
        RejectG76SphereSameThread("MOTION_ADMISSION",AlarmManager::PATH_MOTION_NOT_ADMITTED);return nullptr;
    }
    s.active=s.submitted=true;s.staged=false;
    PrintSphere("[P50] build=EDM287_FIX1 event=BEGIN session=%llu pc=%d layers=%u R=%.9f K=%.9f F=%.9f B=0 inheritedB=%u radiusLaw=R_MINUS_DEPTH contour=ANALYTIC_CIRCLE discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.circleLayerCount,
        s.request.geometry.radiusMm,s.request.geometry.layerStepMm,s.request.feedMmMin,s.prepared.inheritedB?1U:0U);
    return [](NCManager* nc){return nc->PollG76SphereSameThread();};
}

void NCManager::CommitG76SphereCaptureSameThread(NCBlockDispatchId dispatch,const MotionProgramBlockCapture& capture,
    const NCProgramCommitSnapshot& commit,bool committed,bool ledgerFound,const NCBlockLifecycleSnapshot& ledger,int pc,int line)
{
    auto& s=*m_g76Sphere;if(!s.active||s.dispatch!=dispatch)return;
    bool valid=committed&&commit.IsValid()&&ledgerFound&&!s.bound&&pc==s.pc&&line==s.line&&
        commit.scope==NCProgramScope::MEMORY&&commit.frameId==NC_PROGRAM_FRAME_ID_INVALID&&
        commit.sourcePC==pc&&commit.cacheGeneration==s.cache&&ledger.dispatchId==dispatch&&
        ledger.programCommitted&&!ledger.ncDispatchFailed&&!ledger.motionCaptureOverflow&&ledger.sourceLineNumber==line&&
        SameSource(commit,ledger.programTarget)&&SameSource(commit,ledger.programCommit)&&commit.sequence==ledger.programCommit.sequence&&
        capture.count==1U&&!capture.overflow&&ledger.motionSegmentCount==1U&&IsCncFeedValueCurrentSameThread(s.feed)&&
        IsG76SphereScopeCurrentSameThread()&&s.identity.source==MotionCommandSource::NC_MEMORY&&
        s.identity.sourceBlockId==static_cast<MotionSourceBlockId>(pc)&&s.identity.epoch==s.epoch;
    if(valid){const auto& sub=capture.submissions[0U];const auto& seg=ledger.motionSegments[0U];
        valid=sub.producerAccepted&&sub.immediateRejectReason==MotionRejectReason::NONE&&
            sub.commandPathMode==MotionCommandPathMode::EXACT_STOP&&sub.translationGeneration==s.frame.generation&&
            seg.producerAccepted&&seg.immediateRejectReason==MotionRejectReason::NONE&&
            SameIdentity(sub.identity,s.identity)&&SameIdentity(seg.identity,s.identity);
    }
    if(!valid||!m_motion.BindG76SphereParent(s.session,s.identity,s.owner,dispatch,commit.sequence)){
        RejectG76SphereSameThread("FORMAL_PARENT_BINDING",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
    }
    s.commit=commit.sequence;s.bound=true;CommitCncModalFeedSameThread(s.feed,dispatch,s.commit,pc,line);
    // P50 is not a retained legacy history segment. Reusing pre-P50 B2 or
    // return history would bridge across an unrepresented three-dimensional path.
    m_edm171Air->history.Clear();ClearPathCoreReplayHistorySameThread();InvalidatePathCoreReturnCursorSameThread();
    m_g76Modal.lease=s.owner;m_g76Modal.run=s.run;m_g76Modal.cache=s.cache;m_g76Modal.commit=s.commit;
    m_g76Modal.mode=s.prepared.requestedB;m_g76Modal.valid=true;
    PrintSphere("[P50] build=EDM287_FIX1 event=BOUND session=%llu dispatch=%llu commit=%llu children=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.dispatch),static_cast<unsigned long long>(s.commit));
}

void NCManager::ServiceG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return;
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_ACTIVE");
    if(m_state!=NCState::RUN&&m_state!=NCState::HOLD){CancelG76SphereSameThread("STATE");return;}
    MotionG76SphereFeedback observed{};
    const bool matching=m_motion.ReadG76SphereFeedback(observed)&&observed.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner);
    // Preserve the original RT failure before a resulting alarm/epoch change
    // invalidates the NC scope. All original scope checks still run below.
    if(matching&&FailedSphereFeedback(observed))PrintSphereRuntimeFailure(observed,s.request,s.line);
    if(!IsG76SphereScopeCurrentSameThread()){
        RejectG76SphereSameThread("LIVE_SOURCE_FRAME_OR_TRANSPORT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
    }
    if(!matching)return;
    s.feedback=observed;
    if(FailedSphereFeedback(observed)){
        RejectG76SphereSameThread("RUNTIME_FAULT",AlarmManager::PATH_EXECUTION_NOT_READY);return;
    }
    if(s.resumePending&&observed.cycle.resumeSequence==s.startSequence&&!observed.cycle.holdActive&&
        !observed.cycle.held&&observed.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        s.resumePending=false;m_state=NCState::RUN;
        PrintSphere("[P50] build=EDM287_FIX1 event=RESUMED session=%llu start=%llu discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.startSequence));
    }
    if(!s.diagnosticInitialized||s.lastPhase!=observed.cycle.phase||s.lastLayer!=observed.cycle.layerIndex){
        s.diagnosticInitialized=true;s.lastPhase=observed.cycle.phase;s.lastLayer=observed.cycle.layerIndex;
        if(s.diagnosticEvents++<512U||observed.terminal||observed.cycle.held)
            PrintSphere("[P50] build=EDM287_FIX1 event=STATE session=%llu tick=%llu phase=%s layer=%u finishedLayers=%u progress=%.9f held=%u discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.cycle.tick),
                PhaseName(observed.cycle.phase),observed.cycle.layerIndex,observed.cycle.completedLayers,
                observed.cycle.nominalProgressMm,observed.cycle.held?1U:0U);
    }
}

bool NCManager::PollG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return s.completed;
    ServiceG76SphereSameThread();if(!s.active)return false;
    if(!s.bound||m_state!=NCState::RUN||s.resumePending||!s.feedback.terminal)return false;
    if(!s.feedback.completed||s.feedback.cycle.phase!=NCG76SphereCycle::Phase::Complete||
        s.feedback.cycle.completedLayers!=s.request.geometry.circleLayerCount){
        RejectG76SphereSameThread("TERMINAL_STATE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
    }
    auto& ledger=m_edm171Air->ledger;
    if(!m_blockLifecycleLedger.TryGetSnapshot(s.dispatch,ledger)||!ledger.programCommitted||
        ledger.programCommit.sequence!=s.commit||ledger.motionSegmentCount!=1U||ledger.terminalFailure||
        ledger.motionFailedCount||!SameIdentity(ledger.motionSegments[0U].identity,s.identity)){
        RejectG76SphereSameThread("TERMINAL_LEDGER",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
    }
    const auto& segment=ledger.motionSegments[0U];
    if(!segment.completed||!segment.terminal||!ledger.terminalSuccess||ledger.motionCompletedCount!=1U||
        ledger.motionTerminalCount!=1U||m_motion.HasPendingSafetyOrRecoveryRequests())return false;
    if(!s.endpointCommitted){
        if(!m_motion.CommitG76SphereCompletedEndpoint(s.session,s.identity,s.owner,s.endpoint))return false;
        for(unsigned a=0U;a<8U;++a)if(!SameDouble(s.endpoint[a],s.request.geometry.endMCS[a])){
            RejectG76SphereSameThread("TERMINAL_ENDPOINT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
        }
        for(unsigned a=0U;a<8U;++a)CoordSys.commandedMCS[a]=s.endpoint[a];s.endpointCommitted=true;
    }
    if(!m_motion.ReleaseG76SphereBarrier(s.session,s.identity,s.owner))return false;
    s.active=false;s.completed=true;s.submitted=false;UpdateSystemVariables();
    PrintSphere("[P50] build=EDM287_FIX1 event=COMPLETE session=%llu layers=%u X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),s.request.geometry.circleLayerCount,s.endpoint[0],s.endpoint[1],s.endpoint[2]);
    return true;
}

bool NCManager::TryHoldG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return false;
    ServiceG76SphereSameThread();if(!s.active)return true;
    if(s.feedback.terminal&&s.feedback.completed){
        // The RT parent is already physically stopped, but its NC completion
        // may still be waiting for ledger feedback. Retain that boundary;
        // never turn a valid late HOLD into an invalid running intent.
        m_state=NCState::HOLD;s.resumePending=false;
        PrintSphere("[P50] build=EDM287_FIX1 event=HOLD_AT_COMPLETED_PARENT session=%llu discharge=0\n",
            static_cast<unsigned long long>(s.session));return true;
    }
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_HOLD");
    if(!s.bound||!IsG76SphereScopeCurrentSameThread()||
        s.holdSequence==(std::numeric_limits<std::uint64_t>::max)()){
        RejectG76SphereSameThread("HOLD_INTENT",AlarmManager::PATH_EXECUTION_NOT_READY);return true;
    }
    if(!m_motion.RequestG76SphereHold(s.session,s.identity,s.owner,++s.holdSequence,m_startDiagnosticRequests)){
        // Completion can win between the first feedback read and intent
        // admission. Re-read the same exact scope before classifying rejection.
        ServiceG76SphereSameThread();if(!s.active)return true;
        if(s.feedback.terminal&&s.feedback.completed){m_state=NCState::HOLD;s.resumePending=false;return true;}
        RejectG76SphereSameThread("HOLD_INTENT",AlarmManager::PATH_EXECUTION_NOT_READY);return true;
    }
    s.resumePending=false;m_state=NCState::HOLD;
    PrintSphere("[P50] build=EDM287_FIX1 event=HOLD_REQUEST session=%llu hold=%llu path=CENTER_THEN_ORIGINAL_START discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.holdSequence));
    return true;
}

bool NCManager::TryResumeG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return false;
    ServiceG76SphereSameThread();if(!s.active)return true;
    if(m_state==NCState::HOLD&&s.feedback.terminal&&s.feedback.completed){
        m_state=NCState::RUN;return true; // Poll still proves ledger/endpoint/barrier.
    }
    if(m_state!=NCState::HOLD||s.resumePending||!s.feedback.cycle.held||
        s.feedback.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        PrintSphere("[P50] build=EDM287_FIX1 event=START_CONSUMED session=%llu reason=WAIT_HELD_AT_START freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session));return true;
    }
    if(!s.bound||!m_motion.RequestG76SphereResume(s.session,s.identity,s.owner,s.holdSequence,m_startDiagnosticRequests)){
        RejectG76SphereSameThread("RESUME_INTENT",AlarmManager::PATH_EXECUTION_NOT_READY);return true;
    }
    s.startSequence=m_startDiagnosticRequests;s.resumePending=true;
    // Remain NC HOLD through return-to-center/anchor. The RT-owned excursion
    // neither uses generic FeedHold override nor releases the source callback.
    return true;
}

void NCManager::CancelG76SphereSameThread(const char* reason)
{
    auto& s=*m_g76Sphere;if(!s.active&&!s.staged)return;
    MotionG76SphereFeedback observed{};
    const bool reportFailure=s.submitted&&m_motion.ReadG76SphereFeedback(observed)&&observed.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&FailedSphereFeedback(observed);
    const bool submitted=s.submitted;s.active=s.staged=s.resumePending=false;s.completed=false;
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_CANCEL");
    // RESET retains exact sidecar identity for the existing ticketed zero-hold
    // batch. All other cancellation is containment, never an automatic return.
    const bool reset=reason&&std::strcmp(reason,"RESET")==0;
    if(submitted&&!reset)m_motion.RequestEmergencyStopAllAxes();
    if(reportFailure)PrintSphereRuntimeFailure(observed,s.request,s.line);
    PrintSphere("[P50] build=EDM287_FIX1 event=CANCEL session=%llu reason=%s automaticReturn=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),reason?reason:"UNSPECIFIED");
}
