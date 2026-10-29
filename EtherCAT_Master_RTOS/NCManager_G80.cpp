#include "NCManager.h"
#include "NCG80Results.h"
#include "MotionCore_G80.h"
#include "NCGCodeSemantics.h"
#include "GlobalConfig.h"
#include "AlarmManager.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <windows.h> // RTX rtapi.h requires the Windows SDK PVOID/ULONG/etc. first.
#include <rtapi.h>

namespace {
constexpr std::uint32_t NCAdmission=100U,NCFrame=101U,NCRuntime=102U,NCCancelled=103U;
bool FinitePoint(const G80::Point6& point) noexcept {
    for(double value:point)if(!std::isfinite(value))return false;
    return true;
}
bool Literal(const std::string& s,double& out) noexcept {
    if(s.empty())return false;
    errno=0;char* end=nullptr;const double value=std::strtod(s.c_str(),&end);
    if(errno||end==s.c_str()||*end||!std::isfinite(value))return false;
    out=value;return true;
}
// Lookahead resolves no macro expression and commits no modal/PC state.
bool LiteralG80(const NCProgramCacheLine& source,NCBlock& block) noexcept {
    const auto& p=source.parsedBlock;
    if(p.error!=NCParseError::NONE||p.isEmpty||p.isBlockSkip||p.dependsOnMacroState||
       p.controlType!=NCParsedControlType::NONE||p.duplicateAddressMask||p.gCount!=1||p.mCount)return false;
    double g=0.0;if(!Literal(p.gExpressions[0],g)||g!=80.0)return false;
    block=NCBlock{};block.isEmpty=false;block.hasG=true;block.gCount=1;block.gCode=80;block.gCodes[0]=80;
    for(char c='A';c<='Z';++c)if(p.has(c)){
        double value=0.0;if(!Literal(p.expression(c),value))return false;
        block.hasParam[c-'A']=true;block.param[c-'A']=value;
    }
    NCG80::Program decoded{};NCG80::Error error{};
    return NCG80::Decode(block,p.duplicateAddressMask,decoded,error);
}
// The compound rapid is an explicit, literal XYZCUV endpoint row. It must
// never grant a native-frame exception to P/Q, macros or an unrelated G00.
bool LiteralRapid(const NCParsedBlock& p,NCBlock& block) noexcept {
    if(p.error!=NCParseError::NONE||p.isEmpty||p.isBlockSkip||p.dependsOnMacroState||
       p.controlType!=NCParsedControlType::NONE||p.duplicateAddressMask||p.gCount!=1||p.mCount)return false;
    double g=0.0;if(!Literal(p.gExpressions[0],g)||g!=0.0)return false;
    block=NCBlock{};block.isEmpty=false;block.hasG=true;block.gCount=1;block.gCode=0;block.gCodes[0]=0;
    bool axis=false;
    for(char c='A';c<='Z';++c)if(p.has(c)){
        const bool nativeAxis=c=='X'||c=='Y'||c=='Z'||c=='C'||c=='U'||c=='V';
        if(!nativeAxis&&c!='F'&&c!='N'&&c!='G')return false;
        double value=0.0;if(!Literal(p.expression(c),value))return false;
        if(c=='F'&&(value<=0.0||value>100.0))return false;
        block.hasParam[c-'A']=true;block.param[c-'A']=value;axis=axis||nativeAxis;
    }
    return axis;
}
// All output below runs on the supervisory NC thread, never on the 250 us RT path.
constexpr std::uint32_t G80DiagnosticEventLimit=512U;
// RtPrintf has no floating-point conversions in RTSS and limits both the
// message length and parameter count. Format through the existing NC-thread
// CRT first; the RTSS console only receives one bounded %s argument.
// Reject an oversized record as a whole so partial numbers never look valid.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, format(printf, 1, 2)))
#endif
void PrintG80DiagnosticLine(const char* format, ...) noexcept {
    if(format==nullptr){
        RtPrintf("%s","[G80] build=EDM279 event=LOG_FORMAT_FAILURE reason=NULL_FORMAT dataValid=0\n");
        return;
    }
    char line[481]{};
    va_list args;va_start(args,format);
    const int length=std::vsnprintf(line,sizeof(line),format,args);
    va_end(args);
    if(length<0){
        RtPrintf("%s","[G80] build=EDM279 event=LOG_FORMAT_FAILURE reason=CRT_FAILURE dataValid=0\n");
        return;
    }
    if(static_cast<std::size_t>(length)>=sizeof(line)){
        RtPrintf("%s","[G80] build=EDM279 event=LOG_FORMAT_FAILURE reason=LINE_TOO_LONG dataValid=0\n");
        return;
    }
    RtPrintf("%s",line);
}
const char* G80PhaseName(G80::Phase phase) noexcept {
    switch(phase){
    case G80::Phase::Idle:return "IDLE";
    case G80::Phase::InitialRelease:return "INITIAL_RELEASE";
    case G80::Phase::Approach:return "APPROACH";
    case G80::Phase::FastSeek:return "FAST_SEEK";
    case G80::Phase::SlowSeek:return "SLOW_SEEK";
    case G80::Phase::ContactBrake:return "CONTACT_BRAKE";
    case G80::Phase::ContactSettle:return "CONTACT_SETTLE";
    case G80::Phase::Retreat:return "RETREAT";
    case G80::Phase::ReleaseSettle:return "RELEASE_SETTLE";
    case G80::Phase::FaultBrake:return "FAULT_BRAKE";
    case G80::Phase::Complete:return "COMPLETE";
    case G80::Phase::Fault:return "FAULT";
    default:return "UNKNOWN";
    }
}
void G80PointDiagnostic(std::uint64_t session,const char* view,const char* frameRole,
    const NCTranslationSnapshot& frame,int sysFirst,const G80::Point6& point) noexcept {
    PrintG80DiagnosticLine("[G80] build=EDM279 event=COORD session=%llu view=%s frame=%s generation=%llu revision=%llu sysFirst=%d XYZunit=mm CUVunit=deg X=%.9f Y=%.9f Z=%.9f C=%.9f U=%.9f V=%.9f\n",
        static_cast<unsigned long long>(session),view,frameRole,
        static_cast<unsigned long long>(frame.generation),static_cast<unsigned long long>(frame.revision),sysFirst,
        point[0],point[1],point[2],point[3],point[4],point[5]);
}
int AlarmFor(G80::Fault f) noexcept {
    switch(f){
    case G80::Fault::SourceLost:case G80::Fault::PdoLost:
    case G80::Fault::NonMonotonicTick:case G80::Fault::SourceSequenceRegression:return AlarmManager::G80_SIGNAL_INVALID;
    case G80::Fault::InitialContact:case G80::Fault::ApproachContact:return AlarmManager::G80_CONTACT_AT_START;
    case G80::Fault::SearchDistanceExceeded:case G80::Fault::CycleTimeout:return AlarmManager::G80_EDGE_NOT_FOUND;
    case G80::Fault::ReleaseTimeout:return AlarmManager::G80_CONTACT_NOT_RELEASED;
    case G80::Fault::MeanErrorExceeded:return AlarmManager::G80_REPEAT_ERROR;
    case G80::Fault::SettleTimeout:return AlarmManager::G80_SETTLE_TIMEOUT;
    default:return AlarmManager::G80_RUNTIME_FAULT;
    }
}
int AlarmForRuntime(std::uint32_t fault) noexcept {
    if(fault>=4000U&&fault<=static_cast<std::uint32_t>((std::numeric_limits<int>::max)()))
        return static_cast<int>(fault);
    if(fault==1U)return AlarmManager::G80_SIGNAL_INVALID;
    if(fault==2U)return AlarmManager::G80_FRAME_CHANGED;
    if(fault==4U)return AlarmManager::G80_SETTLE_TIMEOUT;
    return AlarmManager::G80_RUNTIME_FAULT;
}
}

bool NCManager::ValidateG80BlockSameThread(const NCBlock& block,int pc,int line,bool report)
{
    const auto* authored=GetBaseProgramCache().TryGetLine(pc);
    NCG80::Program decoded{};NCG80::Error error{};
    const bool valid=authored&&(!line||authored->sourceLineNumber==line)&&
        authored->parsedBlock.error==NCParseError::NONE&&
        authored->parsedBlock.controlType==NCParsedControlType::NONE&&
        m_macroStack.empty()&&!m_isG66Active&&
        NCG80::Decode(block,authored->parsedBlock.duplicateAddressMask,decoded,error)&&
        (!m_g80.active||IsG80CompoundContinuationSameThread(block,pc,authored->sourceLineNumber));
    if(valid)return true;
    if(report){
        // Diagnostic failure publication is the only admission-side mutation.
        if(!m_g80.active){m_g80=G80NCSession{};m_g80.program=decoded;m_g80.ncFault=NCAdmission;
            m_g80.alarmCode=AlarmManager::G_Code_Invalid_parameter;PublishG80SameThread();}
        PrintG80DiagnosticLine("[G80] build=EDM279 event=REJECT pc=%d line=%d syntax=%u motion=0 coordinateWrite=0\n",pc,line,static_cast<unsigned>(error));
        AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter,line);
        m_state=NCState::ALARM;
    }
    return false;
}

bool NCManager::StageG80SameThread(const NCBlock& block,int pc,int line,
    const G80::Point6* approachEnd,double rapidOverride)
{
    if(!ValidateG80BlockSameThread(block,pc,line))return false;
    if(m_g80.active)return IsG80CompoundContinuationSameThread(block,pc,line);
    const auto reject=[this,line](const char* reason,int alarm) noexcept {
        m_g80.active=m_g80.staged=false;m_g80.success=false;m_g80.ncFault=NCAdmission;m_g80.alarmCode=alarm;
        PublishG80SameThread();
        PrintG80DiagnosticLine("[G80] build=EDM279 event=ADMISSION_REJECT line=%d reason=%s motion=0 coordinateWrite=0\n",line,reason);
        AlarmManager::GetInstance().Trigger(alarm,line);m_state=NCState::ALARM;return false;
    };
    G80NCSession candidate{};NCG80::Error syntax{};
    if(!NCG80::Decode(block,GetBaseProgramCache().TryGetLine(pc)->parsedBlock.duplicateAddressMask,
        candidate.program,syntax))return reject("SYNTAX",AlarmManager::G_Code_Invalid_parameter);
    // Retire the previous report immediately; a later admission failure cannot
    // expose an older successful measurement as the new request's result.
    m_g80=candidate;
    std::string configError;
    try {
        const char* file=candidate.program.simulation?"EdgeFindSimulationConfig.txt":"EdgeFindConfig.txt";
        const auto loaded=G80::LoadConfig(GlobalConfig::GetInstance().ParameterDir+file,
            candidate.request.config,configError);
        if(loaded!=G80::ConfigLoadStatus::Loaded||!candidate.request.config.enabled)
            return reject("CONFIG_DISABLED_OR_INVALID",AlarmManager::G80_CONFIG_INVALID);
    }catch(...){return reject("CONFIG_READ",AlarmManager::G80_CONFIG_INVALID);}
    if((m_mode!=NCOperationMode::MEMORY&&m_mode!=NCOperationMode::MDI)||m_state!=NCState::RUN||
       Close_System_Com_flag||Homing.IsActive()||IsFeedHoldActive()||
       AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()||
       m_pathFeed.pending||m_pathArc.pending||m_pathReplay.pending||m_pathHold.armed||m_pathHold.bound||
       m_cncFeed.active||m_cncFeed.selected||m_cncFeed.count||m_gapDryRun.active||m_gapPath.active||m_gapWindow.active||
       m_g76Orbit->active||m_edm94.latched||CoordSys.toolRadiusMode!=40||CoordSys.IsManualFrameEnabled())
        return reject("RUNTIME_OR_G40_SCOPE",AlarmManager::G_Code_Invalid_parameter);
    candidate.run=m_pathCoreLiveBookkeeping.currentRunToken;
    candidate.cache=GetBaseProgramCache().GetGeneration();
    candidate.owner=m_motion.GetMotionOwnerLease();candidate.epoch=m_motion.GetCurrentExecutionEpoch();
    const MotionOwner expected=m_mode==NCOperationMode::MDI?MotionOwner::MDI:MotionOwner::AUTO;
    if(!candidate.run||!candidate.cache||!candidate.owner.IsValid()||candidate.owner.owner!=expected||
       !candidate.owner.Matches(m_programMotionLease)||!candidate.epoch||
       !m_motion.IsGroupNCDrained()||!m_motion.HasExactExecutionDrainAcknowledgement(candidate.epoch,candidate.owner))
        return reject("EXACT_DRAIN_OR_OWNER",AlarmManager::PATH_EXECUTION_NOT_READY);
    // MDI receives its own immutable run descriptor at the first G80. This
    // does not reinterpret its axis words through the ordinary endpoint path.
    if(!CoordSys.IsTranslationRunBound()&&
       (m_mode!=NCOperationMode::MDI||!CoordSys.BeginTranslationRun(candidate.run,this)))
        return reject("FRAME_BIND",AlarmManager::G80_FRAME_CHANGED);
    candidate.frame=CoordSys.GetTranslationSnapshot();
    if(!IsNCTranslationSnapshotValid(candidate.frame)||!CoordSys.IsTranslationRunCurrent())
        return reject("FRAME",AlarmManager::G80_FRAME_CHANGED);
    for(unsigned i=0;i<6U;++i){
        candidate.request.startMCS[i]=CoordSys.commandedMCS[i];
        if(!std::isfinite(candidate.request.startMCS[i]))return reject("COMMAND_POSITION",AlarmManager::G80_FRAME_CHANGED);
    }
    G80::CoordinateError geometry{};
    const G80::Point6& directionStart=approachEnd?*approachEnd:candidate.request.startMCS;
    if(!G80::ResolveDirection(candidate.frame,directionStart,candidate.program.direction,
        candidate.program.p==1U,candidate.request.direction,geometry))
        return reject("DIRECTION_FRAME",AlarmManager::G_Code_Invalid_parameter);
    for(unsigned i=0;i<6U;++i)if(candidate.request.direction[i]!=0.0)candidate.machineMask|=1U<<i;
    for(unsigned i=0;i<6U;++i){
        const auto& axis=m_motion.GetAxisContext(static_cast<int>(i));
        candidate.request.travelMin[i]=-(std::numeric_limits<double>::max)();
        candidate.request.travelMax[i]=(std::numeric_limits<double>::max)();
        if(!axis.isExist){
            if((candidate.machineMask&(1U<<i))||
               (approachEnd&&(*approachEnd)[i]!=candidate.request.startMCS[i]))
                return reject("PARTICIPATING_AXIS_ABSENT",AlarmManager::G80_FRAME_CHANGED);
            candidate.request.velocityCapsNative[i]=candidate.request.accelerationCapsNative[i]=
                candidate.request.decelerationCapsNative[i]=1.0;
            continue;
        }
        const double ppu=axis.resolution_PPR/axis.finalLead;
        const double velocity=axis.maxVel_PPS/ppu;
        if(axis.axisIndex!=static_cast<int>(i)||!std::isfinite(ppu)||ppu<=0.0||
           !std::isfinite(velocity)||velocity<=0.0)
            return reject("AXIS_CAPS",AlarmManager::G80_FRAME_CHANGED);
        candidate.request.velocityCapsNative[i]=velocity;
        // The G80 parameter file owns this parent's ramps. The installed native
        // velocity limit remains authoritative; ordinary G00 ramp settings do not.
        if(!G80::BuildNativeRampCaps(candidate.request.config,velocity,
            candidate.request.accelerationCapsNative[i],candidate.request.decelerationCapsNative[i]))
            return reject("G80_RAMP_CAPS",AlarmManager::G80_FRAME_CHANGED);
        const bool enabled[3]={axis.travelLimit1Enable&&CoordSys.IsProgrammableTravelLimitEnabled(),axis.travelLimit2Enable,axis.travelLimit3Enable};
        const double low[3]={axis.travelLimit1Negative_unit,axis.travelLimit2Negative_unit,axis.travelLimit3Negative_unit};
        const double high[3]={axis.travelLimit1Positive_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Positive_unit};
        for(unsigned n=0;n<3U;++n)if(enabled[n]){
            if(!std::isfinite(low[n])||!std::isfinite(high[n])||low[n]>high[n])return reject("TRAVEL_CONFIG",AlarmManager::G80_FRAME_CHANGED);
            candidate.request.travelMask|=1U<<i;
            candidate.request.travelMin[i]=(std::max)(candidate.request.travelMin[i],low[n]);
            candidate.request.travelMax[i]=(std::min)(candidate.request.travelMax[i],high[n]);
        }
        if(candidate.request.travelMin[i]>candidate.request.travelMax[i]||
           !CoordSys.IsTargetWithinSoftwareTravelLimit(axis,candidate.request.startMCS[i]))
            return reject("START_TRAVEL",AlarmManager::PROGRAMMED_OVER_TRAVEL);
    }
    candidate.request.slowPercent=candidate.program.feedPercent;
    candidate.request.inputSource=candidate.program.simulation?1U:0U;
    candidate.request.fastFirst=candidate.program.j==1U;candidate.request.finalRetreat=candidate.program.k==1U;
    if(approachEnd){
        if(!std::isfinite(rapidOverride)||rapidOverride<=0.0||rapidOverride>1.0)
            return reject("RAPID_PERCENT",AlarmManager::G_Code_Invalid_parameter);
        candidate.request.hasApproach=true;candidate.request.approachEndMCS=*approachEnd;
        for(unsigned i=0;i<6U;++i)candidate.request.rapidVelocityNative[i]=candidate.request.velocityCapsNative[i]*rapidOverride;
    }
    if(m_g80NextSession==0ULL||m_g80NextSession==(std::numeric_limits<std::uint64_t>::max)())
        return reject("SESSION_EXHAUSTED",AlarmManager::G80_RUNTIME_FAULT);
    // The complete authored row and frozen config/direction are now valid.
    if(!CoordSys.FreezeTranslationRun()||!SameNCTranslationSnapshot(candidate.frame,CoordSys.GetTranslationSnapshot()))
        return reject("FRAME_FREEZE",AlarmManager::G80_FRAME_CHANGED);
    if(!m_motion.MatchesNCTranslation(candidate.frame)&&!m_motion.PublishNCTranslation(candidate.frame))
        return reject("FRAME_PUBLISH",AlarmManager::G80_FRAME_CHANGED);
    for(unsigned i=0;i<8U;++i){
        const auto& axis=m_motion.GetAxisContext(static_cast<int>(i));auto& captured=m_fixedTranslationTravel[i];
        captured.minimum[0]=axis.travelLimit1Negative_unit;captured.minimum[1]=axis.travelLimit2Negative_unit;captured.minimum[2]=axis.travelLimit3Negative_unit;
        captured.maximum[0]=axis.travelLimit1Positive_unit;captured.maximum[1]=axis.travelLimit2Positive_unit;captured.maximum[2]=axis.travelLimit3Positive_unit;
        captured.enabled[0]=axis.travelLimit1Enable;captured.enabled[1]=axis.travelLimit2Enable;captured.enabled[2]=axis.travelLimit3Enable;
        captured.homed=axis.isHomed;captured.positionalRotary=axis.isExist&&axis.axisType==AxisType::ROTARY;
        captured.shortestPath=axis.useShortestPath;captured.rotaryModulo=axis.rotaryModulo;captured.maximumVelocity=axis.maxVel_PPS;
        captured.accelerationTime=axis.G00_acc_time;captured.decelerationTime=axis.G00_dec_time;captured.stopDecelerationTime=axis.Stop_dec_time;
    }
    m_fixedTranslationTravelGeneration=candidate.frame.generation;
    candidate.session=m_g80NextSession++;candidate.request.session=candidate.session;
    candidate.measurementFrame=candidate.frame;
    candidate.sourcePC=pc;candidate.sourceLine=line;candidate.staged=true;
    candidate.feedback.session=candidate.session;candidate.feedback.targetCount=candidate.request.config.slowTouches;
    m_g80=candidate;PublishG80SameThread();
    PrintG80DiagnosticLine("[G80] build=EDM279 event=PROFILE session=%llu source=%s accelerationSeconds=%.9f decelerationSeconds=%.9f scope=G80_PARENT frozen=1 G00RampShared=0\n",
        static_cast<unsigned long long>(candidate.session),candidate.program.simulation?"SIM":"PHYSICAL",
        candidate.request.config.accelerationSeconds,candidate.request.config.decelerationSeconds);
    return true;
}

bool NCManager::PrepareG80TranslationSameThread(const NCBlock& block)
{
    const int pc=GetActiveDispatchPC();const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(IsG80CompoundContinuationSameThread(block,pc,source?source->sourceLineNumber:0))return true;
    return StageG80SameThread(block,pc,source?source->sourceLineNumber:0);
}

WaitConditionFunc NCManager::StartG80SameThread(const NCBlock& block)
{
    if(m_g80.active&&IsG80CompoundContinuationSameThread(block,GetActiveDispatchPC(),m_g80.sourceLine)){
        m_g80.claimed=true;return &NCManager::WaitG80Callback;
    }
    if(!m_g80.staged&&!PrepareG80TranslationSameThread(block))return nullptr;
    auto& s=m_g80;
    if(!m_motion.TryG80Move(s.request,s.frame,s.epoch,s.owner,s.sourcePC,s.identity)){
        s.ncFault=NCRuntime;s.success=false;s.staged=false;s.alarmCode=AlarmManager::G80_RUNTIME_FAULT;PublishG80SameThread();
        AlarmManager::GetInstance().Trigger(AlarmManager::G80_RUNTIME_FAULT,s.sourceLine);m_state=NCState::ALARM;return nullptr;
    }
    s.submitted=s.active=s.claimed=true;s.staged=false;
    PublishG80SameThread();return &NCManager::WaitG80Callback;
}

bool NCManager::IsG80CompoundContinuationSameThread(const NCBlock& block,int pc,int line) const noexcept
{
    const auto& s=m_g80;
    if(!s.active||!s.compound||s.claimed||m_mode!=NCOperationMode::MEMORY||m_isSingleBlockEnabled||
       pc!=s.sourcePC||(line&&line!=s.sourceLine)||GetBaseProgramCache().GetGeneration()!=s.cache||
       m_pathCoreLiveBookkeeping.currentRunToken!=s.run||!m_macroStack.empty()||m_isG66Active||
       !s.owner.Matches(m_motion.GetMotionOwnerLease())||m_motion.GetCurrentExecutionEpoch()!=s.epoch||
       !SameNCTranslationSnapshot(s.frame,CoordSys.GetTranslationSnapshot()))return false;
    const auto* source=GetBaseProgramCache().TryGetLine(pc);NCG80::Program p{};NCG80::Error error{};
    if(!source||!NCG80::Decode(block,source->parsedBlock.duplicateAddressMask,p,error))return false;
    return p.direction==s.program.direction&&p.presentMask==s.program.presentMask&&p.p==s.program.p&&
        p.j==s.program.j&&p.k==s.program.k&&p.a==s.program.a&&p.l==s.program.l&&p.feedPercent==s.program.feedPercent&&
        p.simulation==s.program.simulation;
}

bool NCManager::IsG80ApproachCandidateSameThread(const NCParsedBlock& rapid,int pc) const noexcept
{
    if(m_mode!=NCOperationMode::MEMORY||m_isSingleBlockEnabled||!m_macroStack.empty()||m_isG66Active)return false;
    NCBlock endpoint{};if(!LiteralRapid(rapid,endpoint))return false;
    const auto* next=GetBaseProgramCache().TryGetLine(pc+1);NCBlock edge{};
    return next&&LiteralG80(*next,edge);
}

bool NCManager::IsG80ApproachMotionSameThread(const NCBlock& rapid) const noexcept
{
    const int pc=GetActiveDispatchPC();const auto* current=GetBaseProgramCache().TryGetLine(pc);
    NCBlock literal{};
    if(!current||!IsG80ApproachCandidateSameThread(current->parsedBlock,pc)||
       !LiteralRapid(current->parsedBlock,literal)||rapid.isEmpty||rapid.isGoto||rapid.isBlockSkip||
       !rapid.hasG||rapid.gCount!=1||rapid.gCode!=0||rapid.gCodes[0]!=0||rapid.mCount)return false;
    for(char c='A';c<='Z';++c)
        if(rapid.has(c)!=literal.has(c)||(rapid.has(c)&&rapid.val(c)!=literal.val(c)))return false;
    // This exception belongs only to the same installed native G80 axes.
    const auto frame=CoordSys.GetTranslationSnapshot();
    if(!IsNCTranslationSnapshotValid(frame)||frame.cutterMode!=40||CoordSys.IsManualFrameEnabled()||
       (frame.axisIdentity.eccentricEnabled&&frame.toolLengthMode!=49))return false;
    for(unsigned i=0;i<8U;++i)if(frame.axisIdentity.exists[i]){
        if(i>=6U||frame.axisIdentity.address[i]!="XYZCUV"[i]||m_axisNames[i]!="XYZCUV"[i]||
           frame.axisIdentity.nativeUnit[i]!=(i<3U?1U:2U))return false;
    }
    return true;
}

bool NCManager::TryStartG80ApproachSameThread(const NCBlock& rapid,const double* target,
    const bool* programmed,double overrideRatio,bool& handled)
{
    handled=false;
    if(!target||!programmed||m_g80.active||!IsG80ApproachMotionSameThread(rapid))return false;
    // FIX1: a selected pair owns this rapid. Failure may not fall through to
    // the legacy G00 lane after its frame was admitted for the G80 parent.
    handled=true;
    const auto reject=[this](const char* reason,int alarm) noexcept {
        PrintG80DiagnosticLine("[G80] build=EDM279 event=APPROACH_REJECT reason=%s beforeSubmit=1\n",reason);
        AlarmManager::GetInstance().Trigger(alarm);ChangeState(NCState::ALARM);return false;
    };
    if(!CoordSys.IsTranslationRunFrozen()||m_motion.GetQueueSize()!=0U||!m_motion.IsGroupNCDrained()||
       !m_motion.HasExactExecutionDrainAcknowledgement(m_motion.GetCurrentExecutionEpoch(),m_programMotionLease))
        return reject("FRAME_OR_EXACT_DRAIN",AlarmManager::PATH_EXECUTION_NOT_READY);
    const int pc=GetActiveDispatchPC();
    const auto* next=GetBaseProgramCache().TryGetLine(pc+1);NCBlock edge{};
    if(!next||!LiteralG80(*next,edge))return reject("SOURCE_CHANGED",AlarmManager::G_Code_Invalid_parameter);
    for(unsigned i=6U;i<8U;++i)if(programmed[i])return reject("AXIS_SCOPE",AlarmManager::G_Code_Invalid_parameter);
    G80::Point6 endpoint{};
    for(unsigned i=0;i<6U;++i)endpoint[i]=programmed[i]?target[i]:CoordSys.commandedMCS[i];
    if(!StageG80SameThread(edge,pc+1,next->sourceLineNumber,&endpoint,overrideRatio)){
        handled=true;return false;
    }
    auto& s=m_g80;s.compound=true;s.approachPC=pc;s.claimed=false;
    if(!m_motion.TryG80Move(s.request,s.frame,s.epoch,s.owner,pc,s.identity)){
        s.ncFault=NCRuntime;s.staged=false;s.alarmCode=AlarmManager::G80_RUNTIME_FAULT;PublishG80SameThread();handled=true;
        AlarmManager::GetInstance().Trigger(AlarmManager::G80_RUNTIME_FAULT,next->sourceLineNumber);
        m_state=NCState::ALARM;return false;
    }
    // This is the queued nominal tail. Actual completion replaces it with the
    // measured stop/retreat endpoint before the next ordinary motion may run.
    for(unsigned i=0;i<6U;++i)CoordSys.commandedMCS[i]=endpoint[i];
    s.active=s.submitted=true;s.staged=false;handled=true;PublishG80SameThread();
    PrintG80DiagnosticLine("[G80] build=EDM279 event=APPROACH_ACCEPT session=%llu rapidPC=%d edgePC=%d mode=%d source=%s oneParent=1\n",
        static_cast<unsigned long long>(s.session),pc,s.sourcePC,s.frame.distanceMode,
        s.program.simulation?"SIM":"PHYSICAL");
    return true;
}

void NCManager::PublishG80SameThread() noexcept
{
    // Every publication is a whole image. Sequence stays exactly representable
    // in a macro double; wrapping reuses 1 only after invalidating the end marker.
    constexpr std::uint64_t lastExact=9007199254740991ULL;
    m_g80Publication=m_g80Publication>=lastExact?1ULL:m_g80Publication+1ULL;
    auto& s=m_g80;
    NCG80::Extension extension{};
    extension.source=s.reportSource;extension.simScenario=s.simScenario;
    extension.held=s.feedback.held;extension.inputSequence=s.inputSequence;
    extension.simProjectedDistance=s.simProjectedDistance;extension.simPlaneDistance=s.simPlaneDistance;
    const auto& frame=s.measurementFrame;
    extension.frameGeneration=frame.generation;extension.frameRevision=frame.revision;
    extension.wcsCode=frame.wcsCode;extension.rotationMode=frame.rotationMode;
    extension.workMode=frame.workMode;extension.toolLengthMode=frame.toolLengthMode;
    extension.hCode=frame.toolHCode;extension.rotationAngle=frame.rotationAngleDeg;
    // Statistics are computed only after all slow contacts. A failed spread
    // check may still retain valid statistics; merely slowCount>0 cannot.
    if(s.feedback.statisticsValid){
        G80::CoordinateError error{};G80::CoordinateViews actual{},commanded{};
        if(FinitePoint(s.feedback.meanMCS))extension.coordinateValidMask|=NCG80::ActualMeanMachine;
        if(G80::BuildCoordinateViews(frame,s.feedback.meanMCS,actual,error)){
            s.meanWCS=actual.work;
            extension.coordinateValidMask|=NCG80::ActualMeanWork|NCG80::RotationRemoved|NCG80::ToolRemoved|NCG80::BothRemoved;
            extension.rotationRemovedMCS=actual.frameRotationRemovedMachine;
            extension.toolRemovedMCS=actual.toolOffsetRemovedMachine;
            extension.bothRemovedMCS=actual.frameRotationAndToolOffsetRemovedMachine;
        }
        extension.meanCommandedMCS=s.feedback.meanCommandedMCS;
        if(FinitePoint(s.feedback.meanCommandedMCS))extension.coordinateValidMask|=NCG80::CommandedMeanMachine;
        if(G80::BuildCoordinateViews(frame,s.feedback.meanCommandedMCS,commanded,error)){
            extension.meanCommandedWCS=commanded.work;extension.coordinateValidMask|=NCG80::CommandedMeanWork;
        }
    }
    const auto image=NCG80::BuildResults(s.program,s.feedback,s.meanWCS,s.machineMask,
        static_cast<unsigned>(s.feedback.phase),s.feedback.fault!=G80::Fault::None?
            static_cast<unsigned>(s.feedback.fault):s.ncFault,
        s.success,s.voltage,m_g80Publication,s.session,s.runtimeFault,s.alarmCode,s.active,extension);
    NCG80::Results complete=image;NCG80::SetMachineDirection(complete,s.request.direction);
    NCG80::PublishImage(complete,[this](int index,double value) noexcept {MacroSys.SetSysVar(index,value);});
    SyncG80ActiveToPLCSameThread();
}

bool NCManager::IsG80ActiveSameThread() const noexcept
{
    const auto& s=m_g80;const auto state=m_state.load(std::memory_order_acquire);
    if(!s.active||!s.submitted||!s.session||s.success||s.ncFault||s.runtimeFault||
       s.feedback.fault!=G80::Fault::None||(state!=NCState::RUN&&state!=NCState::HOLD)||
       Close_System_Com_flag||AlarmManager::GetInstance().HasAlarm())return false;
    const auto expected=s.owner.owner==MotionOwner::MDI?NCOperationMode::MDI:NCOperationMode::MEMORY;
    return m_mode==expected&&(s.owner.owner==MotionOwner::AUTO||s.owner.owner==MotionOwner::MDI)&&
        s.owner.IsValid()&&m_motion.IsMotionOwnerLeaseCurrent(s.owner)&&
        m_motion.GetCurrentExecutionEpoch()==s.epoch&&
        GetBaseProgramCache().GetGeneration()==s.cache&&m_pathCoreLiveBookkeeping.currentRunToken==s.run&&
        SameNCTranslationSnapshot(s.frame,CoordSys.GetTranslationSnapshot());
}

void NCManager::CancelG80SameThread(const char* reason) noexcept
{
    if(!m_g80.active&&!m_g80.staged)return;
    auto& s=m_g80;
    // Copy the reason into fixed session storage: callers may supply transient text.
    const char* text=reason&&*reason?reason:"UNSPECIFIED";
    unsigned n=0U;
    for(;n+1U<sizeof(s.cancelReason)&&text[n];++n){
        const unsigned char c=static_cast<unsigned char>(text[n]);
        s.cancelReason[n]=(c>=33U&&c<=126U)?static_cast<char>(c):'_';
    }
    s.cancelReason[n]='\0';
    const auto previousTick=s.feedback.tick;const auto previousPhase=s.feedback.phase;
    const auto previousSlow=s.feedback.slowCount;const bool previousHeld=s.feedback.held;
    m_g80.active=m_g80.staged=false;m_g80.success=false;
    if(!m_g80.ncFault)m_g80.ncFault=NCCancelled;
    m_g80.feedback.phase=G80::Phase::Fault;
    if(m_g80.feedback.fault==G80::Fault::None)m_g80.feedback.fault=G80::Fault::Cancelled;
    PublishG80SameThread(); // Reset's existing safety transaction stops RT.
    // Cancellation and its S206 OFF publication precede all diagnostic formatting.
    PrintG80DiagnosticLine("[G80] build=EDM279 event=CANCEL session=%llu reason=%s tick=%llu phaseBefore=%s slowCount=%u heldBefore=%u nc=%u S206Requested=0\n",
        static_cast<unsigned long long>(s.session),s.cancelReason,static_cast<unsigned long long>(previousTick),
        G80PhaseName(previousPhase),previousSlow,previousHeld?1U:0U,
        static_cast<unsigned>(m_state.load(std::memory_order_acquire)));
}

void NCManager::ServiceG80SameThread()
{
    auto& s=m_g80;if(!s.active)return;
    const auto state=m_state.load(std::memory_order_acquire);
    if(state!=NCState::RUN&&state!=NCState::HOLD){CancelG80SameThread("STATE");return;}
    if(m_mode!=(s.owner.owner==MotionOwner::MDI?NCOperationMode::MDI:NCOperationMode::MEMORY)||
       GetBaseProgramCache().GetGeneration()!=s.cache||m_pathCoreLiveBookkeeping.currentRunToken!=s.run||
       !m_motion.IsMotionOwnerLeaseCurrent(s.owner)||m_motion.GetCurrentExecutionEpoch()!=s.epoch||
       !SameNCTranslationSnapshot(s.frame,CoordSys.GetTranslationSnapshot())){
        s.ncFault=NCFrame;s.success=false;s.alarmCode=AlarmManager::G80_FRAME_CHANGED;PublishG80SameThread();
        AlarmManager::GetInstance().Trigger(AlarmManager::G80_FRAME_CHANGED,s.sourceLine);
        m_state=NCState::ALARM;CancelG80SameThread("FRAME");return;
    }
    MotionG80Feedback observed{};
    if(m_motion.ReadG80Feedback(observed)&&observed.session==s.session&&observed.identity.Matches(s.identity)){
        const bool changed=observed.cycle.tick!=s.feedback.tick||observed.terminal;
        s.feedback=observed.cycle;s.voltage=observed.voltageV;s.runtimeFault=observed.runtimeFault;
        s.reportSource=observed.source;s.simScenario=observed.simScenario;
        s.inputSequence=observed.inputSequence;s.simProjectedDistance=observed.simProjectedDistance;
        s.simPlaneDistance=observed.simPlaneDistance;
        G80::CoordinateError error{};G80::Point6 mean{};
        if(s.feedback.statisticsValid&&G80::MachineToWork(s.measurementFrame,s.feedback.meanMCS,mean,error))s.meanWCS=mean;
        if(observed.runtimeFault||s.feedback.fault!=G80::Fault::None){
            s.ncFault=s.feedback.fault==G80::Fault::None?NCRuntime:0U;s.success=false;
            s.alarmCode=s.feedback.fault!=G80::Fault::None?AlarmFor(s.feedback.fault):AlarmForRuntime(observed.runtimeFault);
            s.active=false;PublishG80SameThread();
            AlarmManager::GetInstance().Trigger(s.alarmCode,s.sourceLine);
            m_state=NCState::ALARM;
            // Active was cleared and the alarm/S206 publication completed first.
            // This terminal branch executes once for the admitted NC session.
            PrintG80DiagnosticLine("[G80] build=EDM279 event=FAULT session=%llu tick=%llu phase=%s cycleFault=%u runtimeFault=%u alarmCode=%u source=%u inputSequence=%llu slowCount=%u originCommitted=%u faultSite=RUNTIME actualPoseValid=UNPROVED\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.feedback.tick),
                G80PhaseName(s.feedback.phase),static_cast<unsigned>(s.feedback.fault),s.runtimeFault,s.alarmCode,
                s.reportSource,static_cast<unsigned long long>(s.inputSequence),s.feedback.slowCount,s.originCommitted?1U:0U);
            LogG80FeedbackSameThread(observed);return;
        }
        if(changed&&(observed.terminal||s.feedback.tick-s.lastPublishedTick>=40ULL)){
            s.lastPublishedTick=s.feedback.tick;PublishG80SameThread();
        }
        LogG80FeedbackSameThread(observed);
    }
}

bool NCManager::PollG80SameThread()
{
    auto& s=m_g80;
    if(!s.active)return true;
    if(IsFeedHoldActive()||m_state!=NCState::RUN)return false;
    ServiceG80SameThread();if(!s.active)return true;
    MotionG80Feedback observed{};
    if(!m_motion.ReadG80Feedback(observed)||observed.session!=s.session||
       !observed.identity.Matches(s.identity)||!observed.terminal)return false;
    const auto fail=[this,&s](int alarm) noexcept {
        s.success=false;s.active=false;s.ncFault=NCFrame;s.alarmCode=alarm;PublishG80SameThread();
        AlarmManager::GetInstance().Trigger(alarm,s.sourceLine);m_state=NCState::ALARM;
        m_motion.RequestEmergencyStopAllAxes();
        PrintG80DiagnosticLine("[G80] build=EDM279 event=FAULT session=%llu tick=%llu phase=%s cycleFault=%u runtimeFault=%u alarmCode=%u source=%u inputSequence=%llu slowCount=%u originCommitted=%u faultSite=COMPLETION actualPoseValid=UNPROVED\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.feedback.tick),
            G80PhaseName(s.feedback.phase),static_cast<unsigned>(s.feedback.fault),s.runtimeFault,s.alarmCode,
            s.reportSource,static_cast<unsigned long long>(s.inputSequence),s.feedback.slowCount,s.originCommitted?1U:0U);
        return true;
    };
    if(observed.source!=(s.program.simulation?2U:1U))return fail(AlarmManager::G80_SIGNAL_INVALID);
    if(!observed.completed||s.feedback.phase!=G80::Phase::Complete||s.feedback.fault!=G80::Fault::None||
       !s.feedback.statisticsValid||
       s.feedback.slowCount!=s.request.config.slowTouches||
       !std::isfinite(s.feedback.linearMeanError)||!std::isfinite(s.feedback.rotaryMeanError)||
       s.feedback.linearMeanError>s.request.config.maxMeanErrorMm||
       s.feedback.rotaryMeanError>s.request.config.maxMeanErrorDeg)return fail(AlarmManager::G80_REPEAT_ERROR);
    if(!s.endpointCommitted){
        if(!m_motion.CommitG80CompletedEndpoint(s.session,s.identity,s.owner,s.endpoint))return false;
        for(unsigned i=0;i<6U;++i)CoordSys.commandedMCS[i]=s.endpoint[i];
        s.endpointCommitted=true;
    }
    if(s.program.a>=0&&!s.originCommitted){
        if(!m_motion.HasExactExecutionDrainAcknowledgement(s.epoch,s.owner))return false;
        if(!s.originPrepared){
            G80::Point6 desired{};
            if(!CoordSys.PrepareG80OriginTransition(s.feedback.meanMCS,s.program.authoredMask,s.program.a,
                s.program.l,s.originCandidate,s.meanWCS,desired))return fail(AlarmManager::G80_FRAME_CHANGED);
            s.originPrepared=true;
        }
        const auto commit=m_motion.TryCommitG80OriginTranslation(s.frame,s.originCandidate,s.epoch,s.owner,s.session);
        if(commit==MotionNCTranslationTransitionResult::DEFERRED)return false;
        if(commit!=MotionNCTranslationTransitionResult::ACCEPTED)return fail(AlarmManager::G80_FRAME_CHANGED);
        // Motion committed both the descriptor and CoordinateManager row under
        // one lifecycle reservation. Calling the coordinate setter twice would
        // invalidate the completed generation and is deliberately forbidden.
        m_fixedTranslationTravelGeneration=s.originCandidate.generation;
        m_motion.SetNextCommandTranslation(s.originCandidate);
        ClearPathCoreReplayHistorySameThread();InvalidatePathCoreReturnCursorSameThread();
        s.frame=s.originCandidate;s.originCommitted=true;
    }
    if(!m_motion.ReleaseG80Barrier(s.session,s.identity,s.owner))return false;
    s.success=true;s.active=false;s.claimed=true;PublishG80SameThread();UpdateSystemVariables();
    LogG80CompletedCoordinatesSameThread();
    PrintG80DiagnosticLine("[G80] build=EDM279 event=COMPLETE session=%llu slowCount=%u origin=%d retreat=%u coordinateWrite=%u\n",
        static_cast<unsigned long long>(s.session),s.feedback.slowCount,s.program.a,s.program.k,s.originCommitted?1U:0U);
    return true;
}


void NCManager::LogG80FeedbackSameThread(const MotionG80Feedback& observed) noexcept
{
    auto& s=m_g80;const auto& f=observed.cycle;
    if(!s.junctionLogged&&f.approachJunctionValid){
        s.junctionLogged=true;
        // Both vectors are frozen at the RT G00->seek transition. These are
        // commanded native velocities, never a claim about actual-axis waveforms.
        PrintG80DiagnosticLine("[G80] build=EDM279 event=JUNCTION session=%llu tick=%llu kind=COMMAND_VELOCITY XYZunit=mm/s CUVunit=deg/s exit=(%.9f,%.9f,%.9f,%.9f,%.9f,%.9f) entry=(%.9f,%.9f,%.9f,%.9f,%.9f,%.9f) axisOrder=XYZCUV\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.approachJunctionTick),
            f.approachExitVelocityNative[0],f.approachExitVelocityNative[1],f.approachExitVelocityNative[2],
            f.approachExitVelocityNative[3],f.approachExitVelocityNative[4],f.approachExitVelocityNative[5],
            f.seekEntryVelocityNative[0],f.seekEntryVelocityNative[1],f.seekEntryVelocityNative[2],
            f.seekEntryVelocityNative[3],f.seekEntryVelocityNative[4],f.seekEntryVelocityNative[5]);
    }
    const bool changed=!s.diagnosticInitialized||f.phase!=s.lastDiagnosticPhase||
        f.slowCount!=s.lastDiagnosticSlowCount||f.held!=s.lastDiagnosticHeld;
    if(!changed)return;
    const bool first=!s.diagnosticInitialized;
    const auto previousPhase=s.lastDiagnosticPhase;const auto previousSlow=s.lastDiagnosticSlowCount;
    const bool previousHeld=s.lastDiagnosticHeld;
    s.diagnosticInitialized=true;s.lastDiagnosticPhase=f.phase;
    s.lastDiagnosticSlowCount=f.slowCount;s.lastDiagnosticHeld=f.held;
    if(s.diagnosticEvents>=G80DiagnosticEventLimit){
        if(s.diagnosticEvents==G80DiagnosticEventLimit){
            ++s.diagnosticEvents;
            PrintG80DiagnosticLine("[G80] build=EDM279 event=EVENT_LIMIT session=%llu limit=%u terminalDiagnosticsRetained=1\n",
                static_cast<unsigned long long>(s.session),G80DiagnosticEventLimit);
        }
        return;
    }
    ++s.diagnosticEvents;
    PrintG80DiagnosticLine("[G80] build=EDM279 event=STATE session=%llu eventIndex=%u tick=%llu first=%u phase=%s previousPhase=%s slowCount=%u previousSlowCount=%u targetCount=%u held=%u previousHeld=%u contact=%u nc=%u source=%u inputSequence=%llu\n",
        static_cast<unsigned long long>(s.session),s.diagnosticEvents,static_cast<unsigned long long>(f.tick),first?1U:0U,
        G80PhaseName(f.phase),G80PhaseName(previousPhase),f.slowCount,previousSlow,f.targetCount,
        f.held?1U:0U,previousHeld?1U:0U,f.contact?1U:0U,static_cast<unsigned>(m_state.load(std::memory_order_acquire)),
        observed.source,static_cast<unsigned long long>(observed.inputSequence));
    // Separate the three vectors to stay below the console's line limit.
    const G80::Point6* samples[]={&observed.commandMCS,&observed.actualMCS,&observed.velocityNative};
    const char* parts[]={"COMMAND_POSITION","ACTUAL_POSITION","COMMAND_VELOCITY"};
    for(unsigned part=0U;part<3U;++part){
        // A source/PDO failure resets the input image before containment. Its
        // default zeros are unavailable data, never a measured machine origin.
        if(part==1U&&(!observed.started||observed.runtimeFault||f.fault!=G80::Fault::None)){
            PrintG80DiagnosticLine("[G80] build=EDM279 event=STATE_SAMPLE session=%llu tick=%llu part=ACTUAL_POSITION validity=UNAVAILABLE reason=FAULT_OR_UNSTARTED_SNAPSHOT\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick));
            continue;
        }
        const auto& point=*samples[part];
        PrintG80DiagnosticLine("[G80] build=EDM279 event=STATE_SAMPLE session=%llu tick=%llu part=%s XYZunit=%s CUVunit=%s X=%.9f Y=%.9f Z=%.9f C=%.9f U=%.9f V=%.9f\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick),parts[part],
            part==2U?"mm/s":"mm",part==2U?"deg/s":"deg",
            point[0],point[1],point[2],point[3],point[4],point[5]);
    }
}

void NCManager::LogG80CompletedCoordinatesSameThread() const noexcept
{
    const auto& s=m_g80;const auto& before=s.measurementFrame;
    // Public $ results retain the immutable measurement frame, including after A.
    // Recompute only diagnostic locals so A-after WCS never overwrites $830..835.
    PrintG80DiagnosticLine("[G80] build=EDM279 event=COORD_FRAME session=%llu frame=MEASUREMENT_PRE_A generation=%llu revision=%llu WCS=%d rotationMode=%d workMode=%d Hmode=%d Hcode=%d rotationDeg=%.9f A=%d coordinateWrite=%u resultsUseMeasurementFrame=1 meanWCS=transformOfMeanMCS\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(before.generation),
        static_cast<unsigned long long>(before.revision),before.wcsCode,before.rotationMode,before.workMode,
        before.toolLengthMode,before.toolHCode,before.rotationAngleDeg,s.program.a,s.originCommitted?1U:0U);
    G80::CoordinateError error{};G80::CoordinateViews actual{},commanded{};
    G80PointDiagnostic(s.session,"meanActualMCS","MEASUREMENT_PRE_A",before,820,s.feedback.meanMCS);
    G80PointDiagnostic(s.session,"meanCommandedMCS","MEASUREMENT_PRE_A",before,720,s.feedback.meanCommandedMCS);
    if(G80::BuildCoordinateViews(before,s.feedback.meanMCS,actual,error)){
        G80PointDiagnostic(s.session,"meanWCS","MEASUREMENT_PRE_A",before,830,actual.work);
        G80PointDiagnostic(s.session,"rotationRemovedComparisonMCS","MEASUREMENT_PRE_A",before,740,actual.frameRotationRemovedMachine);
        G80PointDiagnostic(s.session,"HRemovedComparisonMCS","MEASUREMENT_PRE_A",before,750,actual.toolOffsetRemovedMachine);
        G80PointDiagnostic(s.session,"rotationAndHRemovedComparisonMCS","MEASUREMENT_PRE_A",before,760,actual.frameRotationAndToolOffsetRemovedMachine);
    }else PrintG80DiagnosticLine("[G80] build=EDM279 event=COORD_VIEW_INVALID session=%llu point=MEAN_ACTUAL error=%u\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned>(error));
    if(G80::BuildCoordinateViews(before,s.feedback.meanCommandedMCS,commanded,error))
        G80PointDiagnostic(s.session,"meanCommandedWCS","MEASUREMENT_PRE_A",before,730,commanded.work);
    else PrintG80DiagnosticLine("[G80] build=EDM279 event=COORD_VIEW_INVALID session=%llu point=MEAN_COMMAND error=%u\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned>(error));
    // Current HMI position is the completed (possibly retreated) endpoint,
    // distinct from either contact mean. Both endpoint views use FINAL_FRAME.
    G80PointDiagnostic(s.session,"completedCommandEndpointMCS","FINAL_FRAME",s.frame,0,s.endpoint);
    G80::Point6 endpointWork{};
    if(G80::MachineToWork(s.frame,s.endpoint,endpointWork,error))
        G80PointDiagnostic(s.session,"completedCommandEndpointWCS","FINAL_FRAME",s.frame,0,endpointWork);
    else PrintG80DiagnosticLine("[G80] build=EDM279 event=DIAG_INVALID session=%llu point=COMMAND_ENDPOINT_WCS reason=FRAME_TRANSFORM error=%u\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned>(error));
    MotionG80Feedback terminal{};
    if(!m_motion.ReadG80Feedback(terminal)||!terminal.terminal||terminal.session!=s.session||
       !terminal.identity.Matches(s.identity)){
        PrintG80DiagnosticLine("[G80] build=EDM279 event=DIAG_INVALID session=%llu point=TERMINAL_ACTUAL reason=NO_MATCHING_TERMINAL_FEEDBACK\n",
            static_cast<unsigned long long>(s.session));
    }else if(!FinitePoint(terminal.actualMCS)){
        PrintG80DiagnosticLine("[G80] build=EDM279 event=DIAG_INVALID session=%llu point=TERMINAL_ACTUAL reason=NONFINITE_ACTUAL\n",
            static_cast<unsigned long long>(s.session));
    }else{
        G80PointDiagnostic(s.session,"terminalActualMCS","FINAL_FRAME",s.frame,0,terminal.actualMCS);
        G80::Point6 actualWork{};
        if(G80::MachineToWork(s.frame,terminal.actualMCS,actualWork,error))
            G80PointDiagnostic(s.session,"terminalActualWCS","FINAL_FRAME",s.frame,0,actualWork);
        else PrintG80DiagnosticLine("[G80] build=EDM279 event=DIAG_INVALID session=%llu point=TERMINAL_ACTUAL_WCS reason=FRAME_TRANSFORM error=%u\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(error));
    }
    G80::Point6 after{};
    if(G80::MachineToWork(s.frame,s.feedback.meanMCS,after,error))
        G80PointDiagnostic(s.session,"meanWCSAfterA","FINAL_FRAME",s.frame,0,after);
    else PrintG80DiagnosticLine("[G80] build=EDM279 event=COORD_VIEW_INVALID session=%llu point=MEAN_AFTER_A error=%u\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned>(error));
}
