#include "NCManager.h"
#include "NCGCodeSemantics.h"
#include "AlarmManager.h"
#include "NCG76StartDiagnostic.h"
#include "NCPathCoreRadiusArc.h"
#include "NCHoldTelemetry.h"
#include "NCFinishPolicy.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <windows.h>
#include <rtapi.h>

namespace {
void PrintSphereMemoryStage(const char* stage,int pc) noexcept
{
    PrintG76SphereMemoryDiagnostic(stage,pc,sizeof(NCTranslationSnapshot),
        sizeof(NCG76SphereCoordinates::FrozenFrame),sizeof(NCG76SphereRuntimeGeometry::Geometry),
        sizeof(NCG76SphereCycle::Request),sizeof(MotionG76SphereFeedback),sizeof(NCManager));
}
bool SameIdentity(const MotionExecutionIdentity& a,const MotionExecutionIdentity& b) noexcept
{ return a.IsAssigned()&&b.IsAssigned()&&a.Matches(b)&&a.source==b.source&&a.sourceBlockId==b.sourceBlockId; }
bool SameSource(const NCProgramCommitSnapshot& a,const NCProgramCommitSnapshot& b) noexcept
{ return a.scope==b.scope&&a.cacheGeneration==b.cacheGeneration&&a.frameId==b.frameId&&a.sourcePC==b.sourcePC; }
bool SameDouble(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
bool Healthy(const NCBlockLifecycleLedger& ledger) noexcept {
    const auto c=ledger.GetCounters();return !c.activeBlockOverwrite&&!c.activeSegmentIndexOverwrite&&
        !c.orphanFeedback&&!c.duplicateTerminalFeedback&&!c.terminalFeedbackConflict;
}
bool FinishTerminalQualified(const NCG76SphereCycle::Feedback& cycle,std::uint32_t required) noexcept
{
    return required<=NCFinishPolicy::MaximumForcedFlushes&&cycle.finishRequiredCount==required&&
        cycle.finishVerifiedCount==required&&(!required||(!cycle.finishActive&&cycle.finishPhase==4U&&
        cycle.finishAttemptCount>=required&&cycle.finishForcedCompletedCount>=required&&
        cycle.finishForcedCompletedCount<=cycle.finishAttemptCount));
}
const char* ParentName(int sourceG) noexcept
{ return sourceG==1?"[G01]":sourceG==2?"[G02]":sourceG==3?"[G03]":"[P50]"; }
bool SameLiteral(const NCBlock& a,const NCBlock& b) noexcept
{
    if(a.hasG!=b.hasG||a.gCode!=b.gCode||a.gCount!=b.gCount||a.mCount!=b.mCount||
        a.isEmpty!=b.isEmpty||a.isBlockSkip!=b.isBlockSkip||a.isGoto!=b.isGoto||a.gotoTarget!=b.gotoTarget)return false;
    for(char c='A';c<='Z';++c)if(a.has(c)!=b.has(c)||!SameDouble(a.val(c),b.val(c)))return false;
    for(int i=0;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(a.gCodes[i]!=b.gCodes[i])return false;
    for(int i=0;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(a.mCode[i]!=b.mCode[i])return false;
    return true;
}
bool ExactProcessG(const std::string& word,int expected) noexcept
{
    if(expected<0||word.empty()||word.find('\0')!=std::string::npos)return false;
    const auto space=[](char c) noexcept {return c==' '||c=='\t'||c=='\r'||c=='\n';};
    std::size_t i=0U;while(i<word.size()&&space(word[i]))++i;
    if(i<word.size()&&word[i]=='+')++i;
    const unsigned target=static_cast<unsigned>(expected);unsigned value=0U;bool digits=false;
    while(i<word.size()&&word[i]>='0'&&word[i]<='9'){
        digits=true;
        // Compare before multiplication; huge words cannot overflow into the
        // expected code, and decimal fractions never pass through binary64.
        if(value>target/10U)return false;
        value=value*10U+static_cast<unsigned>(word[i++]-'0');
        if(value>target)return false;
    }
    if(!digits||value!=target)return false;
    if(i<word.size()&&word[i]=='.'){
        ++i;
        while(i<word.size()&&word[i]>='0'&&word[i]<='9')if(word[i++]!='0')return false;
    }
    while(i<word.size()&&space(word[i]))++i;
    return i==word.size();
}
bool StationaryProcessLine(int sourceG,const NCG76SphereRuntimeGeometry::Geometry& geometry) noexcept
{
    return sourceG==1&&geometry.valid&&geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
        geometry.travelMm==0.0&&std::memcmp(geometry.startMCS.data(),geometry.endMCS.data(),sizeof(geometry.startMCS))==0;
}
// Literal and modal provenance only. This classifier cannot arm motion, GAP
// or electrical discharge; the normal stopped parent admission follows it.
bool DecodeProcessPath(const NCParsedBlock& parsed,const NCBlock& resolved,int plane,
    const NCG76SphereSource::ModalB& modal,NCBlock& literal,unsigned& mode) noexcept
{
    literal=NCBlock{};mode=0U;
    if(parsed.error!=NCParseError::NONE||parsed.isEmpty||parsed.isBlockSkip||parsed.dependsOnMacroState||
        parsed.duplicateAddressMask||parsed.controlType!=NCParsedControlType::NONE||parsed.gCount!=1||parsed.mCount!=0||
        parsed.assignment.prefix!=0||!parsed.assignment.indexExpression.empty()||!parsed.assignment.valueExpression.empty()||
        parsed.gotoStatement.conditional||!parsed.gotoStatement.conditionExpression.empty()||!parsed.gotoStatement.targetExpression.empty())return false;
    for(std::size_t i=1U;i<parsed.gExpressions.size();++i)if(!parsed.gExpressions[i].empty())return false;
    for(const auto& word:parsed.mExpressions)if(!word.empty())return false;
    for(char c='A';c<='Z';++c)if((!parsed.has(c)&&!parsed.expression(c).empty())||
        parsed.expression(c).find('\0')!=std::string::npos)return false;
    if(!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,literal)||!SameLiteral(literal,resolved)||
        !literal.hasG||literal.gCount!=1||literal.gCodes[0]!=literal.gCode||
        (literal.gCode!=1&&literal.gCode!=2&&literal.gCode!=3)||!ExactProcessG(parsed.gExpressions[0],literal.gCode)||
        literal.mCount||literal.isEmpty||literal.isGoto||literal.isBlockSkip)return false;
    std::uint32_t finishRequired=0U;
    if(!NCFinishPolicy::Decode(parsed,resolved,finishRequired))return false;
    NCArcPlaneAxes axes{};if(!TryGetNCArcPlaneAxes(plane,axes))return false;
    if(literal.gCode==1&&!(literal.has('X')||literal.has('Y')||literal.has('Z')))return false;
    for(char c='A';c<='Z';++c){
        if(!literal.has(c))continue;
        if(!std::isfinite(literal.val(c)))return false;
        const bool allowed=c=='N'||c=='F'||c=='B'||c=='L'||(literal.gCode==1?(c=='X'||c=='Y'||c=='Z'):
            (c==axes.uAddress||c==axes.vAddress||c==axes.uCenter||c==axes.vCenter||c=='R'));
        if(!allowed)return false;
    }
    if(literal.has('F')&&(!(literal.val('F')>0.0)||literal.val('F')>100.0))return false;
    if(literal.gCode!=1){
        if(literal.has('R')){
            if(literal.has('I')||literal.has('J')||literal.has('K')||literal.val('R')==0.0||
                (!literal.has(axes.uAddress)&&!literal.has(axes.vAddress)))return false;
        }else if(!literal.has(axes.uCenter)&&!literal.has(axes.vCenter))return false;
    }
    if(literal.has('B')){
        const double b=literal.val('B');
        if(b<0.0||b>4.0||std::floor(b)!=b||!ExactProcessG(parsed.expression('B'),static_cast<int>(b)))return false;
        mode=static_cast<unsigned>(b);
    }else{
        // Preserve ordinary G01's initial B0 default without manufacturing
        // modal provenance. An existing stale or unsupported B is rejected.
        if(modal.valid){
            if(!NCG76SphereSource::ValidModalB(modal))return false;
            mode=modal.value;
        }
    }
    // Circular interpolation always retains its actual authored geometry.
    // Only its effective retreat route is forced to B2; never rewrite B in
    // the source block or let an inherited B3/B4 escape as an arc chord.
    if(literal.gCode!=1)mode=2U;
    return true;
}
bool BuildProcessPath(const NCBlock& block,const NCG76SphereCoordinates::FrozenFrame& frame,
    double feed,NCG76SphereRuntimeGeometry::Geometry& output) noexcept
{
    using namespace NCG76SphereCoordinates;
    Point3 targetProgram{{frame.programStart[0],frame.programStart[1],frame.programStart[2]}};
    for(unsigned a=0U;a<3U;++a)if(block.has(static_cast<char>('X'+a)))targetProgram[a]=block.val(static_cast<char>('X'+a));
    NCArcPlaneAxes plane{};if(!TryGetNCArcPlaneAxes(frame.translation.rotationPlane,plane))return false;
    const bool fullCircle=block.gCode!=1&&!block.has(plane.uAddress)&&!block.has(plane.vAddress);
    Point8 target{};
    // An omitted circular endpoint is the exact accepted native start. Keep
    // its bits across G90 inverse/forward transforms at a nonzero work origin.
    if(fullCircle)target=frame.startMCS;
    else if(MapPoint(frame,targetProgram,target)!=Status::Ok)return false;
    if(block.gCode==1)return NCG76SphereRuntimeGeometry::BuildLine(frame,target,feed,output);
    Point3 offset{};
    if(block.has('R')){
        if(!TryResolveNCPathRadiusArcCenter(frame.programStart[plane.u],frame.programStart[plane.v],
            targetProgram[plane.u],targetProgram[plane.v],block.val('R'),block.gCode==2?-1:1,
            offset[plane.u],offset[plane.v]))return false;
    }else{
        offset[plane.u]=block.has(plane.uCenter)?block.val(plane.uCenter):0.0;
        offset[plane.v]=block.has(plane.vCenter)?block.val(plane.vCenter):0.0;
    }
    Point3 centerVector{},uProgram{},vProgram{},uNative{},vNative{},normal{};
    if(MapVector(frame,offset,centerVector)!=Status::Ok)return false;
    Point3 center{};for(unsigned a=0U;a<3U;++a)center[a]=frame.startMCS[a]+centerVector[a];
    uProgram[plane.u]=1.0;vProgram[plane.v]=1.0;
    if(MapVector(frame,uProgram,uNative)!=Status::Ok||MapVector(frame,vProgram,vNative)!=Status::Ok)return false;
    normal={{uNative[1]*vNative[2]-uNative[2]*vNative[1],uNative[2]*vNative[0]-uNative[0]*vNative[2],
        uNative[0]*vNative[1]-uNative[1]*vNative[0]}};
    const double norm=std::hypot(std::hypot(normal[0],normal[1]),normal[2]);if(!std::isfinite(norm)||norm<=0.0)return false;
    for(double& value:normal)value/=norm;
    return NCG76SphereRuntimeGeometry::BuildArc(frame,target,center,normal,block.gCode==2?-1:1,feed,fullCircle,output);
}
// Formatting stays on the NC thread. RtPrintf receives only a bounded string.
void PrintParent(int sourceG,const char* format,...) noexcept {
    char line[481]{};
    const int prefix=std::snprintf(line,sizeof(line),"%s build=EDM335 ",ParentName(sourceG));
    if(prefix<0||static_cast<std::size_t>(prefix)>=sizeof(line))return;
    va_list args;va_start(args,format);
    const int n=std::vsnprintf(line+prefix,sizeof(line)-static_cast<std::size_t>(prefix),format,args);va_end(args);
    if(n>=0&&static_cast<std::size_t>(n)<sizeof(line)-static_cast<std::size_t>(prefix))RtPrintf("%s",line);
    else RtPrintf("%s","[EDM308_FIX4] event=DIAGNOSTIC_FORMAT_FAILURE\n");
}
struct StartupStackPattern
{
    std::array<std::uint64_t,4U> words{};
    unsigned matchedBytes=0U;
    bool readable=false;
};
// NC-only first-failure evidence. Read no word until the complete fixed-size
// candidate fits in this thread's live stack. No saved return slot is written.
void ReadStartupStackPattern(const void* slot,const NCTranslationSnapshot& expected,
    StartupStackPattern& out) noexcept
{
    out=StartupStackPattern{};
    if(slot==nullptr)return;
    const DWORD incomingError=GetLastError();
    DWORD stackBytes=0U,availableBytes=0U;
    ULONG_PTR base=0U,limit=0U,top=0U;
    const bool queried=RtGetThreadStack(GetCurrentThread(),&stackBytes,
        &availableBytes,&base,&limit,&top)!=0;
    SetLastError(incomingError);
    const ULONG_PTR address=reinterpret_cast<ULONG_PTR>(slot);
    if(!queried||!stackBytes||availableBytes>stackBytes||!limit||base<=limit||
        address<limit||address>base||sizeof(expected)>base-address)return;
    const volatile unsigned char* actual=static_cast<const volatile unsigned char*>(slot);
    const unsigned char* reference=reinterpret_cast<const unsigned char*>(&expected);
    for(std::size_t i=0U;i<sizeof(expected);++i){
        const unsigned char byte=actual[i];
        if(byte==reference[i])++out.matchedBytes;
        if(i<sizeof(out.words))out.words[i/8U]|=static_cast<std::uint64_t>(byte)<<((i%8U)*8U);
    }
    out.readable=true;
}
void PrintSphereApproachRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76) noexcept {
    PrintParent(sourceG,"event=APPROACH_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::ApproachReasonName(binding.approachReason),
        EDM58::BindingErrorName(binding.approachError),binding.approachFailedFieldId);
}
void PrintSphereInitialSlowRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76) noexcept {
    PrintParent(sourceG,"event=INITIAL_SLOW_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::ApproachReasonName(binding.initialSlowReason),
        EDM58::BindingErrorName(binding.initialSlowError),binding.initialSlowFailedFieldId);
}
void PrintSphereDeepFlushRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76) noexcept {
    PrintParent(sourceG,"event=DEEP_FLUSH_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::DeepFlushReasonName(binding.deepFlushReason),
        EDM58::BindingErrorName(binding.deepFlushError),binding.deepFlushFailedFieldId);
}
const char* RejectTypedBinding(NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG) noexcept
{
    if(binding.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereApproachRejection(binding,mode,sourceG);
        return "TYPED_APPROACH_FIELD_INVALID";
    }
    if(binding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereInitialSlowRejection(binding,mode,sourceG);
        return "TYPED_INITIAL_SLOW_FIELD_INVALID";
    }
    if(binding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereDeepFlushRejection(binding,mode,sourceG);
        return "TYPED_DEEP_FLUSH_FIELD_INVALID";
    }
    return nullptr;
}
const char* PhaseName(NCG76SphereCycle::Phase phase,int sourceG=76) noexcept {
    using P=NCG76SphereCycle::Phase;
    switch(phase){
    case P::Idle:return "IDLE";case P::InitialDepth:return "INITIAL_DEPTH";
    case P::InitialRetreat:return "INITIAL_RETREAT";case P::CircleEntry:return "CIRCLE_ENTRY";
    case P::Orbit:return "ORBIT";case P::ReturnCenter:return "RETURN_CENTER";
    case P::AdvanceCenter:return "ADVANCE_CENTER";case P::Settle:return "SETTLE";
    case P::HoldBrake:return "HOLD_BRAKE";case P::HoldSettle:return "HOLD_SETTLE";
    case P::HoldToCenter:return "HOLD_TO_CENTER";case P::HoldToStart:return "HOLD_TO_START";
    case P::HeldAtStart:return sourceG==76?"HELD_AT_START":"HELD_IN_PLACE";case P::ResumeToCenter:return "RESUME_TO_CENTER";
    case P::ResumeToAnchor:return "RESUME_TO_ANCHOR";case P::FaultBrake:return "FAULT_BRAKE";
    case P::FaultSettle:return "FAULT_SETTLE";case P::Complete:return "COMPLETE";case P::Fault:return "FAULT";
    case P::ResumeBrake:return "RESUME_BRAKE";
    case P::FlushBrake:return "FLUSH_BRAKE";case P::FlushSettle:return "FLUSH_SETTLE";
    case P::FlushLift:return "FLUSH_LIFT";case P::FlushReturn:return "FLUSH_RETURN";
    case P::FlushHoldBrake:return "FLUSH_HOLD_BRAKE";
    case P::GapHistorySettle:return "GAP_HISTORY_SETTLE";case P::GapHistory:return "GAP_HISTORY";
    case P::FlushToCenter:return "FLUSH_TO_CENTER";case P::FlushFromCenter:return "FLUSH_FROM_CENTER";
    case P::FlushPathRetreat:return "FLUSH_PATH_RETREAT";case P::FlushPathReturn:return "FLUSH_PATH_RETURN";
    case P::RecoveryBrake:return "HOLD_RECOVERY_BRAKE";case P::RecoverySettle:return "HOLD_RECOVERY_SETTLE";
    case P::RecoveryMove:return "HOLD_RECOVERY_PATH";case P::InspectionJog:return "HOLD_INSPECTION_JOG";
    case P::InspectionJogBrake:return "HOLD_INSPECTION_BRAKE";case P::InspectionJogReturn:return "HOLD_INSPECTION_RETURN";
    default:return "UNKNOWN";}
}
const char* RuntimeFaultName(std::uint32_t fault) noexcept {
    switch(fault){
    case 0U:return "NONE";case 1U:return "INPUT";case 2U:return "AUTHORITY";
    case 3U:return "CYCLE";case 4U:return "SETTLE";case 5U:return "PARENT_BIND_TIMEOUT";
    case 6U:return "CONTROL_INTENT";case 7U:return "START_POSITION_TIMEOUT";
    case 8U:return "START_TRAVEL";case 9U:return "FLUSH_PROCESS_SCOPE";default:return "UNKNOWN";}
}
const char* CycleFaultName(NCG76SphereCycle::Fault fault) noexcept {
    using F=NCG76SphereCycle::Fault;
    switch(fault){
    case F::None:return "NONE";case F::InvalidRequest:return "INVALID_REQUEST";
    case F::SourceLost:return "SOURCE_LOST";case F::PdoLost:return "PDO_LOST";
    case F::PowerNotInhibited:return "POWER_NOT_INHIBITED";case F::Clock:return "CLOCK";
    case F::PhysicalSequence:return "PHYSICAL_SEQUENCE";case F::IntentSequence:return "INTENT_SEQUENCE";
    case F::Numeric:return "NUMERIC";case F::TravelLimit:return "TRAVEL_LIMIT";
    case F::SettleTimeout:return "SETTLE_TIMEOUT";case F::CycleTimeout:return "CYCLE_TIMEOUT";
    case F::RepeatedHold:return "REPEATED_HOLD";case F::Cancelled:return "CANCELLED";
    default:return "UNKNOWN";}
}
bool FailedSphereFeedback(const MotionG76SphereFeedback& report) noexcept {
    return report.runtimeFault||report.cycle.fault!=NCG76SphereCycle::Fault::None||
        (report.terminal&&!report.completed);
}
// Diagnostic-only readback remains useful after the RT failure revokes its
// owner/epoch. It never grants motion or releases the NC completion barrier.
void PrintSphereRuntimeFailure(const MotionG76SphereFeedback& report,
    const NCG76SphereCycle::Request& request,int line,int sourceG=76) noexcept {
    PrintParent(sourceG,"event=FAULT session=%llu line=%d tick=%llu phase=%s nominal=%s cycleFault=%u cause=%s runtimeFault=%u gate=%s started=%u discharge=0\n",
        static_cast<unsigned long long>(report.session),line,static_cast<unsigned long long>(report.inputSequence),
        PhaseName(report.cycle.phase,sourceG),PhaseName(report.cycle.nominalPhase,sourceG),static_cast<unsigned>(report.cycle.fault),
        CycleFaultName(report.cycle.fault),report.runtimeFault,
        RuntimeFaultName(report.runtimeFault),report.started?1U:0U);
    const auto& clock=report.cycle.clockFault;
    if(report.cycle.fault==NCG76SphereCycle::Fault::Clock&&clock.valid){
        // RT freezes the first violating sample before fault containment.
        // Format it here on the NC owner; later cancellation cannot replace
        // the original clock evidence with a new sample or owner rejection.
        PrintParent(sourceG,"event=CLOCK_FAULT session=%llu reasonBits=0x%02X previousTick=%llu currentTick=%llu previousPhysical=%llu currentPhysical=%llu discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned>(clock.reasonBits),
            static_cast<unsigned long long>(clock.previousTick),static_cast<unsigned long long>(clock.currentTick),
            static_cast<unsigned long long>(clock.previousPhysicalSequence),static_cast<unsigned long long>(clock.currentPhysicalSequence));
        PrintParent(sourceG,"event=CLOCK_INTERVAL session=%llu previousNowUs=%llu currentNowUs=%llu deltaValid=%u deltaUs=%llu minimumUs=%llu maximumUs=%llu discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(clock.previousNowUs),
            static_cast<unsigned long long>(clock.currentNowUs),clock.deltaValid?1U:0U,
            static_cast<unsigned long long>(clock.deltaUs),static_cast<unsigned long long>(clock.minimumTickUs),
            static_cast<unsigned long long>(clock.maximumTickUs));
    }
    const auto& input=report.firstInputFault;
    if(input.valid){
        // This is RT's first failing sample, frozen before cancellation. A
        // later ALARM/owner change must not replace its source with a generic
        // NC AIR_GRANT rejection or with a newer physical input snapshot.
        PrintParent(sourceG,"event=INPUT_FAULT session=%llu tick=%llu stage=%s gate=%s axis=%d flags=0x%08X validAxisMask=0x%02X statusWord=0x%04X mode=%d rawPosition=%d discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(input.tick),
            MotionG76SphereInputStageName(input.stage),MotionG76SphereInputGateName(input.gate),
            static_cast<int>(input.axis),static_cast<unsigned>(input.flags),static_cast<unsigned>(input.validAxisMask),
            static_cast<unsigned>(input.statusWord),static_cast<int>(input.mode),static_cast<int>(input.rawPosition));
        PrintParent(sourceG,"event=INPUT_FAULT_CLOCK session=%llu inputTick=%llu physicalSequence=%llu nowUs=%llu clockFrequency=%llu discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(input.inputTick),
            static_cast<unsigned long long>(input.physicalSequence),static_cast<unsigned long long>(input.nowUs),
            static_cast<unsigned long long>(input.clockFrequency));
        PrintParent(sourceG,"event=INPUT_FAULT_SCOPE session=%llu inputSession=%llu expectedSession=%llu inputEpoch=%llu expectedEpoch=%llu inputSegment=%llu expectedSegment=%llu inputSource=%u expectedSource=%u inputBlock=%d expectedBlock=%d discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(input.inputSession),
            static_cast<unsigned long long>(input.expectedSession),static_cast<unsigned long long>(input.inputIdentity.epoch),
            static_cast<unsigned long long>(report.identity.epoch),static_cast<unsigned long long>(input.inputIdentity.segmentId),
            static_cast<unsigned long long>(report.identity.segmentId),static_cast<unsigned>(input.inputIdentity.source),
            static_cast<unsigned>(report.identity.source),static_cast<int>(input.inputIdentity.sourceBlockId),
            static_cast<int>(report.identity.sourceBlockId));
    }
    const auto& target=report.started?report.commandMCS:request.geometry.startMCS;
    for(unsigned axis=0U;axis<6U;++axis){
        // Motion may tighten both tolerances to the installed axis windows.
        // Report the actual frozen RT values, not the NC request defaults.
        const double tolerance=axis<3U?report.positionToleranceMm:report.rotaryToleranceDeg;
        const bool sampleValid=(report.inputValidAxisMask&(1U<<axis))!=0U;
        if(!sampleValid){
            PrintParent(sourceG,"event=FAULT_AXIS session=%llu axis=%u actual=UNAVAILABLE target=%.12f error=UNAVAILABLE tolerance=%.12f unit=%s inWindow=UNKNOWN sampleValid=0 fallbackActual=%.12g discharge=0\n",
                static_cast<unsigned long long>(report.session),axis,target[axis],tolerance,
                axis<3U?"mm":"deg",report.actualMCS[axis]);
            continue;
        }
        const double error=report.actualMCS[axis]-target[axis];
        PrintParent(sourceG,"event=FAULT_AXIS session=%llu axis=%u actual=%.12f target=%.12f error=%.12f tolerance=%.12f unit=%s inWindow=%u sampleValid=1 discharge=0\n",
            static_cast<unsigned long long>(report.session),axis,report.actualMCS[axis],target[axis],
            error,tolerance,axis<3U?"mm":"deg",std::isfinite(tolerance)&&tolerance>0.0&&
                std::isfinite(error)&&std::fabs(error)<=tolerance?1U:0U);
    }
}
}

bool NCManager::IsG76SphereBlock(const NCBlock& block) noexcept
{ return NCGCodeSemantics::Contains(block,76)&&block.has('P')&&block.val('P')==50.0; }

bool NCManager::IsEDMProcessPathCandidateSameThread(const NCBlock& block) const noexcept
{
    return !m_edm94.latched&&IsEDM171AirGrantCurrentSameThread(true)&&
        (NCGCodeSemantics::Contains(block,1)||NCGCodeSemantics::Contains(block,2)||NCGCodeSemantics::Contains(block,3));
}

bool NCManager::IsDeclaredHoldRecoveryPathSupportedSameThread(const NCBlock& block) const noexcept
{
    // EDM331 adapters retain their original exact execution/owner contract.
    // Ordinary CNC and legacy G76 batches need their own suspension adapter;
    // an explicit G181 must never be accepted and silently ignored there.
    return IsG76SphereBlock(block)||IsEDMProcessPathCandidateSameThread(block);
}

void NCManager::ExportHoldRecoveryTelemetrySameThread(NCHoldTelemetry::Image& image) const noexcept
{
    using namespace NCHoldTelemetry;
    image=Image{};
    image.declared=IsHoldRetreatDeclarationActiveSameThread();
    image.declaredSegments=GetHoldRetreatSegmentsSameThread();
    image.capabilities=Declaration|Config|CurrentParentRecovery|PreviousParentRecovery|InspectionXYZ;
    image.phase=Phase::Idle;
    if(!m_g76Sphere||!m_g76Sphere->active){
        if(GetState()==NCState::RUN||GetState()==NCState::HOLD)image.phase=Phase::Unsupported;
        return;
    }
    const auto& state=*m_g76Sphere;
    MotionG76SphereFeedback report{};
    if(!m_motion.ReadG76SphereFeedback(report)||!state.bound||report.session!=state.session||
       !SameIdentity(report.identity,state.identity)||!report.owner.Matches(state.owner)||
       report.cycle.session!=state.session||!report.cycle.holdRecoveryEnabled){
        image.phase=Phase::Unavailable;return;
    }
    const auto& cycle=report.cycle;
    image.currentSourceG=static_cast<std::uint32_t>(state.sourceG);
    image.parentSession=state.session;
    image.runtimeSupported=true;
    image.effectiveSegments=cycle.holdEffectiveCount;
    image.availableSegments=cycle.holdAvailableCount;
    image.holdSequence=cycle.holdSequence;
    image.phase=cycle.holdRecoveryPhase<=10U?static_cast<Phase>(cycle.holdRecoveryPhase):Phase::Fault;
    image.inspectionAxisMask=report.holdInspectionAxisMask;
    image.manualTraceValid=cycle.inspectionTraceValid;
    image.canJog=GetState()==NCState::HOLD&&IsExternalReadyInterlock()&&!report.terminal&&
        !CoordSys.IsManualFrameEnabled()&&!AlarmManager::GetInstance().HasAlarm()&&report.holdInspectionJogReady&&
        (image.phase==Phase::Held||image.phase==Phase::ManualJog);
    image.canResume=GetState()==NCState::HOLD&&IsExternalReadyInterlock()&&!report.terminal&&
        !CoordSys.IsManualFrameEnabled()&&!AlarmManager::GetInstance().HasAlarm()&&report.holdRecoveryResumeReady&&
        image.phase==Phase::Held;
    image.retreatAnchorValid=cycle.holdActive&&cycle.holdRecoveryPhase>=3U&&cycle.holdRecoveryPhase<=8U;
    image.processPointValid=image.retreatAnchorValid;
    for(unsigned axis=0U;axis<3U;++axis){
        image.retreatAnchorXYZ[axis]=report.holdRetreatAnchorMCS[axis];
        image.processPointXYZ[axis]=report.holdProcessPointMCS[axis];
    }
    if(report.runtimeFault||cycle.fault!=NCG76SphereCycle::Fault::None){
        image.phase=Phase::Fault;image.canJog=image.canResume=false;
        image.retreatAnchorValid=image.processPointValid=false;
    }
    if(!Valid(image)){
        // A torn/foreign/inconsistent observation cannot look like a valid
        // zero-segment declaration or give the UI a JOG/START permission.
        image.runtimeSupported=image.canJog=image.canResume=false;
        image.effectiveSegments=image.availableSegments=0U;
        image.inspectionAxisMask=0U;image.retreatAnchorValid=image.processPointValid=false;
        image.phase=Phase::Unavailable;
    }
}

// EDM333: only the NC owner reads the immutable source receipt. The RT
// channel supplies an exact running-parent proof; the interpreter cursor and
// unversioned m_Group fields are not executing-row evidence for this display.
bool NCManager::TryGetCommittedProcessParentDisplayPCSameThread(int& pc) const noexcept
{
    pc=-1;
    const auto mode=GetMode();const auto state=GetState();
    if(mode!=NCOperationMode::MEMORY||(state!=NCState::RUN&&state!=NCState::HOLD)||
       Close_System_Com_flag||!m_macroStack.empty()||m_isG66Active||!m_g76Sphere)return false;
    const auto& s=*m_g76Sphere;
    if(!s.active||!s.submitted||!s.bound||s.completed||!s.commit||!s.session||s.pc<0||
       (s.sourceG!=1&&s.sourceG!=2&&s.sourceG!=3&&s.sourceG!=76)||
       !s.run||s.run!=m_pathCoreLiveBookkeeping.currentRunToken||!s.cache||
       s.cache!=GetBaseProgramCache().GetGeneration()||!s.owner.Matches(m_programMotionLease)||
       !m_motion.IsMotionOwnerLeaseCurrent(s.owner)||m_motion.GetCurrentExecutionEpoch()!=s.epoch||
       !s.identity.IsAssigned()||s.identity.source!=MotionCommandSource::NC_MEMORY||s.identity.epoch!=s.epoch)return false;
    const auto* source=GetBaseProgramCache().TryGetLine(s.pc);
    if(source==nullptr||source->sourceLineNumber!=s.line)return false;
    MotionG76SphereFeedback report{};
    if(!m_motion.ReadG76SphereFeedback(report)||report.session!=s.session||
       !SameIdentity(report.identity,s.identity)||!report.owner.Matches(s.owner)||report.cycle.session!=s.session||
       !report.accepted||!report.started||report.terminal||report.completed||report.runtimeFault||
       report.cycle.fault!=NCG76SphereCycle::Fault::None||GetMode()!=mode||GetState()!=state)return false;
    pc=s.pc;return true;
}

bool NCManager::TryG76SphereInspectionJogSameThread(int axis,int direction)
{
    if(GetMode()!=NCOperationMode::MEMORY||GetState()!=NCState::HOLD||!m_g76Sphere||
       !m_g76Sphere->active||!m_g76Sphere->bound)return false;
    auto& state=*m_g76Sphere;MotionG76SphereFeedback report{};
    if(!m_motion.ReadG76SphereFeedback(report)||report.session!=state.session||
       !SameIdentity(report.identity,state.identity)||!report.owner.Matches(state.owner)||
       !report.cycle.holdRecoveryEnabled)return false;
    // This branch handles inspection inputs only. It never acquires a JOG
    // owner, changes mode, thaws a translation, or releases the parent.
    ForceEDMPLCOutputsOffSameThread("HOLD_INSPECTION");
    if(report.terminal||!IsG76SphereScopeCurrentSameThread())return true;
    if(axis<0||axis>2||(direction!=1&&direction!=-1)){axis=-1;direction=0;}
    if(axis>=0){
        const auto& physical=m_motion.GetAxisContext(axis);
        if(!IsExternalReadyInterlock()||Homing.IsActive()||AlarmManager::GetInstance().HasAlarm()||
           CoordSys.IsManualFrameEnabled()||!physical.isExist||!physical.isServoOn||
           physical.isFault||physical.isLagAlarm||physical.axisType!=AxisType::LINEAR||
           (direction>0&&physical.positiveTravelBlocked)||(direction<0&&physical.negativeTravelBlocked)){
            axis=-1;direction=0;
        }
    }
    if(axis>=0&&!report.holdInspectionJogReady&&!report.cycle.inspectionJogActive)return true;
    if(m_g76InspectionJogSequence==(std::numeric_limits<std::uint64_t>::max)())return true;
    (void)m_motion.RequestG76SphereInspectionJog(state.session,state.identity,state.owner,
        report.cycle.holdSequence,axis,direction,++m_g76InspectionJogSequence);
    return true;
}

bool NCManager::ValidateEDMProcessPathBlockSameThread(const NCBlock& block,int pc,int line)
{
    const auto* source=GetBaseProgramCache().TryGetLine(pc);NCBlock literal{};unsigned mode=0U;
    CoordSys.CopyTranslationSnapshot(m_ncDispatchScratch->processValidationTranslation);
    const bool valid=IsEDMProcessPathCandidateSameThread(block)&&source&&
        (!line||source->sourceLineNumber==line)&&m_macroStack.empty()&&!m_isG66Active&&
        CoordSys.activePlane==m_ncDispatchScratch->processValidationTranslation.rotationPlane&&
        DecodeProcessPath(source->parsedBlock,block,CoordSys.activePlane,CaptureG76SphereModalBSameThread(),literal,mode);
    if(!valid){
        PrintParent(block.gCode,"event=SOURCE_REJECT pc=%d reason=EXACT_LITERAL_OR_COMMITTED_B01234 discharge=0\n",pc);
        RejectG76SphereSameThread("PROCESS_PATH_LITERAL_OR_B_SCOPE",AlarmManager::G_Code_Invalid_parameter,
            source?source->sourceLineNumber:line);return false;
    }
    if(m_g76Sphere->active){RejectG76SphereSameThread("PROCESS_PARENT_STILL_ACTIVE",AlarmManager::PATH_EXECUTION_NOT_READY,
        source->sourceLineNumber);return false;}
    return true;
}

bool NCManager::PrepareEDMProcessPathTranslationSameThread(const NCBlock& block)
{ return PrepareG76SphereTranslationSameThread(block); }

WaitConditionFunc NCManager::StartEDMProcessPathSameThread(const NCBlock& block)
{ return StartG76SphereSameThread(block); }

bool NCManager::IsCompletedEDMProcessPathTerminalAllowedSameThread(const NCBlock& block)
{
    const auto& s=*m_g76Sphere;const int pc=GetActiveDispatchPC();
    const auto* row=GetBaseProgramCache().TryGetLine(pc);NCBlock literal{};
    if((s.sourceG!=1&&s.sourceG!=2&&s.sourceG!=3)||!s.completed||!s.bound||!s.endpointCommitted||
        s.active||s.staged||s.submitted||!s.session||!s.commit||!s.identity.IsAssigned()||pc<=s.pc||
        !s.feedback.terminal||!s.feedback.completed||s.feedback.runtimeFault||
        s.feedback.cycle.phase!=NCG76SphereCycle::Phase::Complete||s.feedback.cycle.fault!=NCG76SphereCycle::Fault::None||
        !row||row->parsedBlock.error!=NCParseError::NONE||row->parsedBlock.isEmpty||row->parsedBlock.isBlockSkip||
        row->parsedBlock.dependsOnMacroState||row->parsedBlock.duplicateAddressMask||
        row->parsedBlock.controlType!=NCParsedControlType::NONE||row->parsedBlock.gCount!=0||row->parsedBlock.mCount!=1||
        row->parsedBlock.assignment.prefix!=0||!row->parsedBlock.assignment.indexExpression.empty()||
        !row->parsedBlock.assignment.valueExpression.empty()||row->parsedBlock.gotoStatement.conditional||
        !row->parsedBlock.gotoStatement.conditionExpression.empty()||!row->parsedBlock.gotoStatement.targetExpression.empty()||
        !NCPreparedBlockQueueShadow::TryBuildLiteralBlock(row->parsedBlock,literal)||!SameLiteral(literal,block)||
        literal.hasG||literal.gCount||literal.mCount!=1||literal.mCode[0]!=30||literal.isGoto||
        !ExactProcessG(row->parsedBlock.mExpressions[0],30))return false;
    for(const auto& word:row->parsedBlock.gExpressions)if(!word.empty())return false;
    for(std::size_t i=1U;i<row->parsedBlock.mExpressions.size();++i)
        if(!row->parsedBlock.mExpressions[i].empty())return false;
    for(char c='A';c<='Z';++c)if((literal.has(c)&&(c!='N'||!std::isfinite(literal.val(c))))||
        (!row->parsedBlock.has(c)&&!row->parsedBlock.expression(c).empty())||
        row->parsedBlock.expression(c).find('\0')!=std::string::npos)return false;
    if(m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||m_isG66Active||!m_macroStack.empty()||
        Homing.IsActive()||IsFeedHoldActive()||m_g80.active||m_g76Orbit->active||m_pathFeed.pending||m_pathArc.pending||
        m_pathReplay.pending||m_pathHold.armed||m_pathHold.bound||m_cncFeed.active||m_cncFeed.selected||m_cncFeed.count||
        m_gapDryRun.active||m_gapPath.active||m_gapWindow.active||m_edm94.latched||CoordSys.IsManualFrameEnabled()||
        !IsG76SphereScopeCurrentSameThread()||m_motion.HasPendingSafetyOrRecoveryRequests()||
        !m_motion.IsGroupNCDrained()||!m_motion.IsGroupDone()||m_motion.GetCommandIngressSize()||m_motion.GetCommandReplaySize()||
        !m_motion.HasExactExecutionDrainAcknowledgement(s.epoch,s.owner))return false;
    for(unsigned a=0U;a<8U;++a)if(!SameDouble(CoordSys.commandedMCS[a],s.endpoint[a])||
        !SameDouble(s.endpoint[a],s.request.geometry.endMCS[a]))return false;
    auto& ledger=m_edm171Air->ledger; // Reuse persistent owner-thread scratch, not the NC stack.
    if(!m_blockLifecycleLedger.TryGetSnapshot(s.dispatch,ledger)||!ledger.programCommitted||
        ledger.programCommit.sequence!=s.commit||ledger.dispatchId!=s.dispatch||ledger.sourceLineNumber!=s.line||
        ledger.motionSegmentCount!=1U||ledger.terminalFailure||ledger.motionFailedCount||!ledger.terminalSuccess||
        ledger.motionCompletedCount!=1U||ledger.motionTerminalCount!=1U||
        !ledger.motionSegments[0U].completed||!ledger.motionSegments[0U].terminal||
        !SameIdentity(ledger.motionSegments[0U].identity,s.identity))return false;
    return true;
}

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
    m_g76Sphere->process=NCG76SphereProcess::Envelope{};
    m_g76Sphere->gapShadow=NCG76SphereGapProcess::Feedback{};
    if(m_g76Sphere->submitted)
        (void)m_motion.RevokeG76SphereProcessShadow(m_g76Sphere->session,m_g76Sphere->identity,m_g76Sphere->owner);
    m_g76Sphere->gapShadowRevoked=true;m_g76Sphere->gapBinding.ready=false;
    m_g76Sphere->active=m_g76Sphere->staged=m_g76Sphere->resumePending=false;
    AlarmManager::GetInstance().Trigger(alarm,line);ChangeState(NCState::ALARM);
    if(submitted)m_motion.RequestEmergencyStopAllAxes();
    PrintParent(m_g76Sphere->sourceG,"event=REJECT session=%llu line=%d reason=%s discharge=0\n",
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
    // A nonzero committed mode is never silently downgraded to B0. These
    // modes select SIM-only E5 retreat paths; B1 remains unreleased for P50.
    if(!NCG76SphereRetreat::ValidMode(words.requestedB)){RejectG76SphereSameThread("P50_B_PROCESS_NOT_RELEASED",AlarmManager::G_Code_Invalid_parameter,
        source->sourceLineNumber);return false;}
    if(m_g76Sphere->active){RejectG76SphereSameThread("P50_PARENT_STILL_ACTIVE",AlarmManager::PATH_EXECUTION_NOT_READY,
        source->sourceLineNumber);return false;}
    return true;
}

bool NCManager::PrepareG76SphereTranslationSameThread(const NCBlock& block)
{
    const int pc=GetActiveDispatchPC();const auto* source=GetBaseProgramCache().TryGetLine(pc);
    const bool processPath=block.gCode==1||block.gCode==2||block.gCode==3;
    if(!processPath)PrintSphereMemoryStage("NC_PREPARE_ENTER",pc);
    if(processPath?!ValidateEDMProcessPathBlockSameThread(block,pc,source?source->sourceLineNumber:0):
        !ValidateG76SphereBlockSameThread(block,pc,source?source->sourceLineNumber:0))return false;
    // Allocate NC-only preparation storage once per parent; no large temporary
    // session is constructed on the NC stack or on any servo callback.
    m_g76Sphere.reset(new G76SphereNCState{});auto& s=*m_g76Sphere;
    s.pc=pc;s.line=source->sourceLineNumber;s.dispatch=m_currentExecutingBlockDispatchId;
    s.sourceG=processPath?block.gCode:76;s.pathSource=block;
    const auto reject=[this](const char* reason,int alarm){RejectG76SphereSameThread(reason,alarm);return false;};
    if(processPath&&!NCFinishPolicy::Decode(source->parsedBlock,block,s.finishRequiredCount))
        return reject("FINISH_PARAMETER_LITERAL",AlarmManager::G_Code_Invalid_parameter);
    if(m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||!IsEDM171AirGrantCurrentSameThread(processPath)||
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
    CoordSys.CopyTranslationSnapshot(s.frame);
    if(!CoordSys.IsTranslationRunBound()||!CoordSys.IsTranslationRunCurrent()||
        !IsNCTranslationSnapshotValid(s.frame)||!IsPathCoreLiveNativeConfigCurrentSameThread())
        return reject("FRAME_OR_NATIVE_CONFIG",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    NCG76SphereCoordinates::Point8 start{};
    for(unsigned a=0U;a<8U;++a)start[a]=CoordSys.commandedMCS[a];
    if(processPath){
        NCBlock literal{};unsigned mode=0U;const auto modal=CaptureG76SphereModalBSameThread();
        NCG76SphereCoordinates::FrozenFrame pathFrame{};
        if(!DecodeProcessPath(source->parsedBlock,block,s.frame.rotationPlane,modal,literal,mode))
            return reject("PROCESS_PATH_SOURCE",AlarmManager::G_Code_Invalid_parameter);
        // E parameters and native pump bases own the motion ceiling. An
        // optional F is captured only from this authored block and caps its
        // nominal machining speed; an omitted F never reads modal F.
        s.gapBinding=NCG76SphereGapBinding::Capture(m_recipe,m_edmProcessProfile,
            m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly,true);
        if(const char* typed=RejectTypedBinding(s.gapBinding,mode,s.sourceG))return reject(typed,AlarmManager::G_Code_Invalid_parameter);
        if(!s.gapBinding.ready||!s.gapBinding.config.enabled||!s.gapBinding.config.signedMotionEnabled||
            !NCG76SphereGapProcess::ResolveParameterMotionCeiling(s.gapBinding.config,s.processMotionCeiling)){
            PrintParent(s.sourceG,"event=PARAMETER_BINDING_REJECT reason=%s error=%s field=%u discharge=0\n",
                NCG76SphereGapBinding::ReasonName(s.gapBinding.reason),EDM58::BindingErrorName(s.gapBinding.error),s.gapBinding.failedFieldId);
            return reject("PROCESS_PATH_REQUIRES_FROZEN_GAP_RECIPE",AlarmManager::G_Code_Invalid_parameter);
        }
        if(block.has('F')&&(!CaptureCncFeedValueSameThread(block,s.feed)||!s.feed.programmed||s.feed.feedMMMin>100.0))
            return reject("PROCESS_PATH_CURRENT_F_CAP",AlarmManager::G_Code_Invalid_parameter);
        if(NCG76SphereCoordinates::BuildFrame(s.frame,start,pathFrame)!=NCG76SphereCoordinates::Status::Ok||
            !BuildProcessPath(literal,pathFrame,s.processMotionCeiling,s.request.geometry))
            return reject("PROCESS_PATH_GEOMETRY_OR_FEED",AlarmManager::G_Code_Invalid_parameter);
        // These provenance fields are shared; no P50 preparation or sphere
        // descriptor is fabricated for a real line or circular source.
        s.prepared.requestedB=mode;s.prepared.explicitB=block.has('B');s.prepared.inheritedB=!block.has('B')&&modal.valid;
        if(s.prepared.inheritedB)s.prepared.inheritedBSource=modal;
    }else{
        PrintSphereMemoryStage("NC_SOURCE_PREPARE_BEGIN",pc);
        const auto prepared=NCG76SphereArchitecture::Prepare(source->parsedBlock,s.frame,start,NCG76Sphere::Limits{},
            CaptureG76SphereModalBSameThread(),s.prepared,s.workspace);
        PrintSphereMemoryStage("NC_SOURCE_PREPARE_END",pc);
        if(prepared!=NCG76SphereArchitecture::Status::Ok)
            return reject("SOURCE_GEOMETRY_OR_FEED",AlarmManager::G_Code_Invalid_parameter);
        PrintSphereMemoryStage("NC_SOURCE_VALIDATE_BEGIN",pc);
        const bool preparationValid=NCG76SphereArchitecture::Valid(s.prepared)&&NCG76SphereRetreat::ValidMode(s.prepared.requestedB);
        PrintSphereMemoryStage("NC_SOURCE_VALIDATE_END",pc);
        if(!preparationValid)
            return reject("SOURCE_GEOMETRY_OR_FEED",AlarmManager::G_Code_Invalid_parameter);
        PrintSphereMemoryStage("NC_RUNTIME_GEOMETRY_BEGIN",pc);
        const bool geometry=NCG76SphereRuntimeGeometry::Build(s.prepared.native,s.request.geometry);
        PrintSphereMemoryStage("NC_RUNTIME_GEOMETRY_END",pc);
        if(!geometry||!CaptureCncFeedValueSameThread(block,s.feed)||
            s.feed.feedMMMin!=s.prepared.native.descriptor.input.feedMmMin)
            return reject("SOURCE_GEOMETRY_OR_FEED",AlarmManager::G_Code_Invalid_parameter);
    }
    // Build the independent nominal station once on NC. B0 may retain AIR
    // motion without process metadata. B2/B3/B4 require a valid same-parent
    // checkpoint below; no cursor grants motion or discharge authority.
    if(!processPath)PrintSphereMemoryStage("NC_PATH_BUILD_BEGIN",pc);
    (void)NCG76SpherePath::Build(s.request.geometry,s.path);
    if(!processPath)PrintSphereMemoryStage("NC_PATH_BUILD_END",pc);
    // Capture the selected COND/E once. Missing fields leave B0 AIR available;
    // a selected B2/B3/B4 route requires the complete SIM binding below.
    if(!processPath)s.gapBinding=NCG76SphereGapBinding::Capture(m_recipe,m_edmProcessProfile,
        m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly);
    s.request.retreatMode=s.prepared.requestedB;
    // A confirmed typed error is different from unavailable/unconfirmed
    // optional fields. It rejects B0 too, before frame freeze or submission.
    if(s.gapBinding.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        return reject("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if(s.gapBinding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        return reject("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if(s.gapBinding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        return reject("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if(processPath&&(!s.path.valid||!s.gapBinding.ready||!s.gapBinding.config.enabled||
        !s.gapBinding.config.signedMotionEnabled))
        return reject("PROCESS_PATH_REQUIRES_FROZEN_GAP_RECIPE",AlarmManager::G_Code_Invalid_parameter);
    // B2 needs a canonical same-parent station. B3/B4 also retain this
    // original checkpoint for HOLD and diagnostics throughout their routes.
    if(s.request.retreatMode!=0U&&(!s.path.valid||!s.gapBinding.ready||
        !s.gapBinding.config.enabled||(!s.gapBinding.config.flushMotionEnabled&&!StationaryProcessLine(s.sourceG,s.request.geometry))||
        !s.gapBinding.config.signedMotionEnabled)){
        PrintParent(s.sourceG,"event=B_MODE_BINDING_REJECT B=%u reason=%s error=%s field=%u approachReason=%s approachError=%s approachField=%u discharge=0\n",
            s.request.retreatMode,NCG76SphereGapBinding::ReasonName(s.gapBinding.reason),
            EDM58::BindingErrorName(s.gapBinding.error),s.gapBinding.failedFieldId,
            NCG76SphereGapBinding::ApproachReasonName(s.gapBinding.approachReason),
            EDM58::BindingErrorName(s.gapBinding.approachError),s.gapBinding.approachFailedFieldId);
        return reject("B_MODE_REQUIRES_SIM_FLUSH_BINDING",AlarmManager::G_Code_Invalid_parameter);
    }
    if(s.finishRequiredCount&&(!s.gapBinding.config.flushMotionEnabled||
        !s.gapBinding.config.signedMotionEnabled||s.request.geometry.travelMm<=0.0))
        return reject("FINISH_REQUIRES_NONZERO_GAP_FLUSH_PATH",AlarmManager::G_Code_Invalid_parameter);
    s.request.parameterFeeds=processPath;
    s.request.nominalFeedCapMmMin=processPath&&block.has('F')?s.feed.feedMMMin:0.0;
    s.request.feedMmMin=processPath?s.processMotionCeiling:s.feed.feedMMMin;
    s.request.returnFeedMmMin=s.request.feedMmMin;
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
    if(!processPath)PrintSphereMemoryStage("NC_REQUEST_VALIDATE_BEGIN",pc);
    const bool requestValid=NCG76SphereCycle::ValidateRequest(s.request);
    if(!processPath)PrintSphereMemoryStage("NC_REQUEST_VALIDATE_END",pc);
    if(!requestValid)return reject("CYCLE_REQUEST",AlarmManager::G_Code_Invalid_parameter);
    // E6 is already a physical millimetre distance. The frozen native cut
    // axis supplies its direction; never scale or rotate the height again.
    NCG76SphereCycle::FlushConfig flush{};
    if(!NCG76SphereGapProcess::BuildFlushConfig(s.gapBinding.config,s.request.feedMmMin,flush))
        return reject("FLUSH_CONFIG",AlarmManager::G_Code_Invalid_parameter);
    if(s.finishRequiredCount&&!flush.enabled)return reject("FINISH_FLUSH_DISABLED",AlarmManager::G_Code_Invalid_parameter);
    if(!NCG76SphereCycle::ValidateFlushConfig(s.request,flush))
        return reject("FLUSH_TRAVEL_ENVELOPE",AlarmManager::PROGRAMMED_OVER_TRAVEL);
    if(flush.enabled)for(unsigned a=0U;a<6U;++a){
        double minimum=0.0,maximum=0.0;
        if(!MotionG76SphereFlush::AxisEnvelope(s.request.geometry,flush,a,minimum,maximum,s.request.retreatMode)||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(m_motion.GetAxisContext(static_cast<int>(a)),minimum)||
            !CoordSys.IsTargetWithinSoftwareTravelLimit(m_motion.GetAxisContext(static_cast<int>(a)),maximum))
            return reject("FLUSH_SOFTWARE_TRAVEL",AlarmManager::PROGRAMMED_OVER_TRAVEL);
    }
    if(!CoordSys.FreezeTranslationRun())return reject("FRAME_FREEZE_PUBLICATION",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    CoordSys.CopyTranslationSnapshot(s.scopeTranslation);
    if(!SameNCTranslationSnapshot(s.frame,s.scopeTranslation)||
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
    auto& s=*m_g76Sphere;
    const auto reject=[&](const char* gate,int axis=-1){
        PrintParent(s.sourceG,"event=SCOPE_REJECT session=%llu line=%d gate=%s axis=%d expectedEpoch=%llu actualEpoch=%llu discharge=0\n",
            static_cast<unsigned long long>(s.session),s.line,gate,axis,
            static_cast<unsigned long long>(s.epoch),static_cast<unsigned long long>(m_motion.GetCurrentExecutionEpoch()));
        return false;
    };
    if(!IsEDM171AirGrantCurrentSameThread(s.sourceG>=1&&s.sourceG<=3))return reject("AIR_GRANT");
    if(s.run!=m_pathCoreLiveBookkeeping.currentRunToken||s.cache!=GetBaseProgramCache().GetGeneration())return reject("RUN_SOURCE");
    if(!s.owner.Matches(m_programMotionLease)||!m_motion.IsMotionOwnerLeaseCurrent(s.owner))return reject("OWNER");
    if(m_motion.GetCurrentExecutionEpoch()!=s.epoch)return reject("EPOCH");
    TraceG76SphereStartupSameThread(G76SphereStartupCoordinateCurrentBegin,"COORD_CURRENT_BEGIN");
    const bool coordinateCurrent=CoordSys.IsTranslationRunFrozen()&&CoordSys.IsTranslationRunCurrent();
    TraceG76SphereStartupSameThread(G76SphereStartupCoordinateCurrentEnd,"COORD_CURRENT_END");
    if(!coordinateCurrent)return reject("COORDINATE_SCOPE");
    TraceG76SphereStartupSameThread(G76SphereStartupFrameCompareBegin,"FRAME_COMPARE_BEGIN");
    CoordSys.CopyTranslationSnapshot(s.scopeTranslation);
    const bool frameCurrent=SameNCTranslationSnapshot(s.frame,s.scopeTranslation)&&m_motion.MatchesNCTranslation(s.frame);
    TraceG76SphereStartupSameThread(G76SphereStartupFrameCompareEnd,"FRAME_COMPARE_END");
    if(!frameCurrent)return reject("FRAME");
    TraceG76SphereStartupSameThread(G76SphereStartupNativeConfigBegin,"NATIVE_CONFIG_BEGIN");
    const bool nativeCurrent=IsPathCoreLiveNativeConfigCurrentSameThread();
    TraceG76SphereStartupSameThread(G76SphereStartupNativeConfigEnd,"NATIVE_CONFIG_END");
    if(!nativeCurrent)return reject("NATIVE_CONFIG");
    TraceG76SphereStartupSameThread(G76SphereStartupTravelConfigBegin,"TRAVEL_CONFIG_BEGIN");
    const bool travelCurrent=IsFixedTranslationTravelCurrentSameThread();
    TraceG76SphereStartupSameThread(G76SphereStartupTravelConfigEnd,"TRAVEL_CONFIG_END");
    if(!travelCurrent)return reject("TRAVEL_CONFIG");
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
    TraceG76SphereStartupSameThread(40U,"START_ENTER");
    auto& s=*m_g76Sphere;
    const bool processPath=s.sourceG!=76;
    const auto currentModal=CaptureG76SphereModalBSameThread();
    const auto& inherited=s.prepared.inheritedBSource;
    const bool processModalCurrent=!processPath||s.prepared.explicitB||
        (s.prepared.inheritedB?(NCG76SphereSource::ValidModalB(currentModal)&&
            currentModal.value==inherited.value&&currentModal.commit==inherited.commit&&
            NCG76SphereSource::SameModalBScope(currentModal.committed,inherited.committed)):
            !currentModal.valid);
    const auto* finishSource=GetBaseProgramCache().TryGetLine(s.pc);std::uint32_t finishCount=0U;
    const bool finishCurrent=processPath?(finishSource&&
        NCFinishPolicy::Decode(finishSource->parsedBlock,block,finishCount)&&finishCount==s.finishRequiredCount):
        s.finishRequiredCount==0U;
    const bool processFeedCurrent=!processPath||(s.request.parameterFeeds&&
        SameDouble(s.request.feedMmMin,s.processMotionCeiling)&&
        SameDouble(s.request.geometry.feedMmMin,s.processMotionCeiling)&&
        (s.pathSource.has('F')?(s.feed.valid&&s.feed.programmed&&IsCncFeedValueCurrentSameThread(s.feed)&&
            SameDouble(s.request.nominalFeedCapMmMin,s.feed.feedMMMin)):
            (!s.feed.valid&&!s.feed.programmed&&s.feed.feedMMMin==0.0&&s.request.nominalFeedCapMmMin==0.0)));
    if(s.pc!=GetActiveDispatchPC()||s.dispatch!=m_currentExecutingBlockDispatchId||
        !processModalCurrent||!processFeedCurrent||!finishCurrent||s.sourceG!=block.gCode||s.request.retreatMode!=s.prepared.requestedB||
        (processPath?(!SameLiteral(s.pathSource,block)||!ValidateEDMProcessPathBlockSameThread(block,s.pc,s.line)):
            !ValidateG76SphereBlockSameThread(block,s.pc,s.line))){
        RejectG76SphereSameThread("SUBMIT_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;
    }
    TraceG76SphereStartupSameThread(41U,"SOURCE_CHECK_DONE");
    const bool scopeCurrent=IsG76SphereScopeCurrentSameThread();
    TraceG76SphereStartupSameThread(42U,"SCOPE_CHECK_DONE");
    if(!scopeCurrent){
        RejectG76SphereSameThread("SUBMIT_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return nullptr;
    }
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_AIR_PARENT");
    TraceG76SphereStartupSameThread(43U,"POWER_OFF_DONE");
    if(s.gapBinding.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    // NC-only recapture also detects a typed error introduced after Prepare.
    // Never infer its state from the old ready bit or old approach reason.
    const auto currentApproach=NCG76SphereGapBinding::Capture(m_recipe,m_edmProcessProfile,
        m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly,processPath);
    TraceG76SphereStartupSameThread(44U,"RECIPE_CAPTURE_DONE");
    if(currentApproach.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(currentApproach.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(currentApproach.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG);
        RejectG76SphereSameThread("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.ready&&!NCG76SphereGapBinding::IsCurrent(s.gapBinding,m_recipe,
        m_edmProcessProfile,m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly)){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};s.gapBinding.ready=false;
        s.gapBinding.reason=NCG76SphereGapBinding::Reason::CaptureChanged;
        s.gapBinding.error=EDM58::BindingError::CaptureChanged;s.gapShadowRevoked=true;
    }
    TraceG76SphereStartupSameThread(45U,"RECIPE_CURRENT_DONE");
    if(processPath&&(!s.gapBinding.ready||!s.gapBinding.config.enabled||!s.gapBinding.config.signedMotionEnabled)){
        RejectG76SphereSameThread("PROCESS_PATH_GAP_RECIPE_CHANGED",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(processPath){
        double currentCeiling=0.0;
        if(!NCG76SphereGapProcess::ResolveParameterMotionCeiling(s.gapBinding.config,currentCeiling)||
            !SameDouble(currentCeiling,s.processMotionCeiling)){
            RejectG76SphereSameThread("PROCESS_PARAMETER_CEILING_CHANGED",AlarmManager::G_Code_Invalid_parameter);return nullptr;
        }
    }
    if(s.request.retreatMode!=0U&&(!s.gapBinding.ready||!s.gapBinding.config.enabled||
        (!s.gapBinding.config.flushMotionEnabled&&!StationaryProcessLine(s.sourceG,s.request.geometry))||
        !s.gapBinding.config.signedMotionEnabled)){
        RejectG76SphereSameThread("B_MODE_SIM_FLUSH_BINDING_CHANGED",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    MotionG76SphereAdmission admission{};
    TraceG76SphereStartupSameThread(46U,"BEFORE_MOTION_ADMISSION");
    const auto& holdConfig=GetHoldRecoveryConfigSameThread();
    const bool admitted=m_motion.TryG76SphereMove(s.request,s.frame,s.epoch,s.owner,s.pc,s.identity,
        &admission,&s.gapBinding.config,&holdConfig,GetHoldRetreatSegmentsSameThread(),s.finishRequiredCount);
    TraceG76SphereStartupSameThread(47U,"AFTER_MOTION_ADMISSION");
    if(!admitted){
        PrintParent(s.sourceG,"event=ADMISSION_REJECT session=%llu line=%d gate=%s axis=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),s.line,MotionG76SphereAdmissionGateName(admission.gate),admission.axis);
        RejectG76SphereSameThread("MOTION_ADMISSION",AlarmManager::PATH_MOTION_NOT_ADMITTED);return nullptr;
    }
    s.active=s.submitted=true;s.staged=false;
    if(processPath)PrintParent(s.sourceG,"event=FINISH_CONFIG session=%llu required=%u scope=THIS_PARENT rework=RESIDUAL_GAP_PATH periodicE5=INDEPENDENT forcedHeight=NORMAL_E6 discharge=0\n",
        static_cast<unsigned long long>(s.session),s.finishRequiredCount);
    if(processPath){
        const unsigned requested=s.prepared.explicitB?static_cast<unsigned>(s.pathSource.val('B')):
            s.prepared.inheritedB?s.prepared.inheritedBSource.value:0U;
        PrintParent(s.sourceG,"event=BEGIN session=%llu pc=%d g=%d geometry=%s Fmode=%s nominalFcapMmMin=%.9f motionCeilingMmMin=%.9f B=%u inheritedB=%u parents=1 layers=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.sourceG,s.sourceG==1?"ANALYTIC_LINE":"ANALYTIC_ARC",
            s.pathSource.has('F')?"CURRENT_BLOCK_CAP":"PARAMETER_ONLY",s.request.nominalFeedCapMmMin,
            s.processMotionCeiling,s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        PrintParent(s.sourceG,"event=B_RESOLUTION session=%llu g=%d authoredB=%d requestedB=%u effectiveB=%u forced=%u policy=%s origin=%s discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceG,s.prepared.explicitB?static_cast<int>(requested):-1,
            requested,s.request.retreatMode,s.sourceG!=1&&requested!=2U?1U:0U,
            s.sourceG==1?"SOURCE_MODAL":"ARC_ALWAYS_B2",s.prepared.explicitB?"EXPLICIT":
                s.prepared.inheritedB?"COMMITTED":"DEFAULT_B0");
        PrintParent(s.sourceG,"event=FEED_POLICY session=%llu source=PARAMETERS optionalF=%u nominalFcapMmMin=%.9f modalFRead=0 flushIndependent=1 internalCeilingMmMin=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pathSource.has('F')?1U:0U,
            s.request.nominalFeedCapMmMin,s.processMotionCeiling);
        PrintParent(s.sourceG,"event=PROCESS_PATH session=%llu valid=%u pathMm=%.9f nominalScope=THIS_PARENT previousParentPolicy=RELEASED_ONE_B2 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.path.valid?1U:0U,s.path.totalLengthMm);
        // Observe the admitted native geometry on NC, after the immutable
        // frame has been checked. Never apply a second coordinate transform.
        PrintParent(s.sourceG,"event=FRAME_MAPPING session=%llu distance=%d plane=%d rotation=%d angleDeg=%.9f wcs=%d Hmode=%d H=%d work=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.distanceMode,s.frame.rotationPlane,
            s.frame.rotationMode,s.frame.rotationAngleDeg,s.frame.wcsCode,s.frame.toolLengthMode,
            s.frame.toolHCode,s.frame.workMode);
        PrintParent(s.sourceG,"event=NATIVE_PATH session=%llu startX=%.9f startY=%.9f startZ=%.9f endX=%.9f endY=%.9f endZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.startMCS[0],
            s.request.geometry.startMCS[1],s.request.geometry.startMCS[2],s.request.geometry.endMCS[0],
            s.request.geometry.endMCS[1],s.request.geometry.endMCS[2]);
        PrintParent(s.sourceG,"event=PROGRAM_SOURCE session=%llu startX=%.9f startY=%.9f startZ=%.9f Xword=%.9f Yword=%.9f Zword=%.9f axisMask=%u centerU=%.9f centerV=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.frame.programStart[0],
            s.request.geometry.frame.programStart[1],s.request.geometry.frame.programStart[2],
            s.pathSource.val('X'),s.pathSource.val('Y'),s.pathSource.val('Z'),
            (s.pathSource.has('X')?1U:0U)|(s.pathSource.has('Y')?2U:0U)|(s.pathSource.has('Z')?4U:0U),
            s.frame.rotationCenterMM[0],s.frame.rotationCenterMM[1]);
        PrintParent(s.sourceG,"event=FRAME_OFFSETS session=%llu extX=%.9f extY=%.9f extZ=%.9f wcsX=%.9f wcsY=%.9f wcsZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.extOffsetMM[0],s.frame.extOffsetMM[1],
            s.frame.extOffsetMM[2],s.frame.wcsOffsetMM[0],s.frame.wcsOffsetMM[1],s.frame.wcsOffsetMM[2]);
        PrintParent(s.sourceG,"event=TOOL_OFFSETS session=%llu Hmode=%d H=%d rawX=%.9f rawY=%.9f rawZ=%.9f effectiveX=%.9f effectiveY=%.9f effectiveZ=%.9f eccentricFrozen=%u discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.toolLengthMode,s.frame.toolHCode,
            s.frame.toolOffsetMM[0],s.frame.toolOffsetMM[1],s.frame.toolOffsetMM[2],
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[0]:NCTranslationToolOffsetMM(s.frame,0U),
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[1]:NCTranslationToolOffsetMM(s.frame,1U),
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[2]:NCTranslationToolOffsetMM(s.frame,2U),
            s.request.geometry.frame.fixedEccentric?1U:0U);
        PrintParent(s.sourceG,"event=WORK_OFFSETS session=%llu mode=%d W=%d X=%.9f Y=%.9f Z=%.9f yawDeg=%.9f pitchDeg=%.9f rollDeg=%.9f centerU=%.9f centerV=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.workMode,s.frame.workWCode,
            s.frame.workOffset[0],s.frame.workOffset[1],s.frame.workOffset[2],
            s.frame.workOffset[3],s.frame.workOffset[4],s.frame.workOffset[5],
            s.frame.workRotationCenterMM[0],s.frame.workRotationCenterMM[1]);
        PrintParent(s.sourceG,"event=SCALE_FRAME session=%llu mode=%d factor=%.9f centerX=%.9f centerY=%.9f centerZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.scalingMode,s.frame.scalingFactor,
            s.frame.scalingCenterMM[0],s.frame.scalingCenterMM[1],s.frame.scalingCenterMM[2]);
        PrintParent(s.sourceG,"event=MIRROR_FRAME session=%llu mask=%u centerX=%.9f centerY=%.9f centerZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(s.frame.mirrorMask),
            s.frame.mirrorCenterMM[0],s.frame.mirrorCenterMM[1],s.frame.mirrorCenterMM[2]);
    }else{
        PrintParent(s.sourceG,"event=BEGIN session=%llu pc=%d layers=%u R=%.9f K=%.9f F=%.9f B=%u inheritedB=%u radiusLaw=R_MINUS_DEPTH contour=ANALYTIC_CIRCLE discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.circleLayerCount,
            s.request.geometry.radiusMm,s.request.geometry.layerStepMm,s.request.feedMmMin,s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        PrintParent(s.sourceG,"event=PROCESS_PATH session=%llu valid=%u pathMm=%.9f nativeR=%.9f nativeK=%.9f gap=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.path.valid?1U:0U,s.path.totalLengthMm,
            s.request.geometry.radiusMm,s.request.geometry.layerStepMm);
    }
    TraceG76SphereStartupSameThread(48U,"SOURCE_LOGS_DONE");
    const auto& binding=s.gapBinding;const auto& config=binding.config;
    PrintParent(s.sourceG,"event=GAP_BINDING session=%llu enabled=%u reason=%s error=%s field=%u recipe=%llu tuning=%llu E5us=%llu E6mm=%.6f E7pct=%.3f E8pct=%.3f E9V=%.3f source=SOFTWARE_SIMULATION AD=0 motionApplied=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.ready?1U:0U,NCG76SphereGapBinding::ReasonName(binding.reason),
        EDM58::BindingErrorName(binding.error),binding.failedFieldId,static_cast<unsigned long long>(config.recipeGeneration),
        static_cast<unsigned long long>(config.tuningGeneration),static_cast<unsigned long long>(config.workTimeUs),
        config.jumpHeightMm,config.jumpSpeedPercent,config.machiningOverridePercent,config.referenceV);
    PrintParent(s.sourceG,"event=SIM_MOTION_CONFIG session=%llu enabled=%u domain=%s historyCapacity=%u nominalParentOnly=1 work=%s E5=NOMINAL_FORWARD_ONLY source=SOFTWARE_SIMULATION AD=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.ready&&config.signedMotionEnabled?1U:0U,
        processPath?"SOURCE_AND_RELEASED_PREDECESSOR_B2":"PREVIOUS_COMPLETED_LEG",1U,
        processPath?"SOURCE_FORWARD":"INITIAL_DEPTH_CIRCLE_ENTRY_ORBIT");
    TraceG76SphereStartupSameThread(49U,"RECIPE_LOGS_DONE");
    NCG76SphereCycle::FlushConfig flush{};
    const bool flushReady=NCG76SphereGapProcess::BuildFlushConfig(config,s.request.feedMmMin,flush)&&flush.enabled;
    const char* route=s.request.retreatMode==2U?"SAME_PARENT_PATH":s.request.retreatMode==3U?
        (processPath?"CENTERLINE_AXIS_COALESCED":"CENTER_THEN_AXIS"):s.request.retreatMode==4U?
        (processPath?"CENTERLINE_DIAGONAL_COALESCED":"DIAGONAL_CENTER_THEN_AXIS"):s.request.retreatMode==1U?
        "MACHINE_POSITIVE_Z":processPath?"OPPOSITE_STOPPED_TANGENT":"OPPOSITE_NATIVE_CUT_AXIS";
    PrintParent(s.sourceG,"event=FLUSH_CONFIG session=%llu enabled=%u B=%u route=%s heightMm=%.6f feedMmMin=%.6f zeroSamples=%u historyShort=DEFER source=SOFTWARE_SIMULATION discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady?1U:0U,s.request.retreatMode,route,
        flushReady?flush.heightMm:0.0,flushReady?flush.feedMmMin:0.0,s.request.settleSamples);
    TraceG76SphereStartupSameThread(50U,"FLUSH_LOG_DONE");
    // Cooked native feeds are frozen once with this parent. A disabled E11/E13
    // approach is explicit metadata; it never rewrites a stored raw recipe.
    PrintParent(s.sourceG,"event=FLUSH_SPEED_CONFIG session=%llu independent=%u Pump1=%.6f Pump2=%.6f Pump3=%.6f Pump4=%.6f Pump5=%.6f approachEnabled=%u reason=%s field=%u E11pct=%.3f E13mm=%.6f nativeBase=%.6g final=%.6f unit=mm/min source=SOFTWARE_SIMULATION discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.independentFeedsEnabled?1U:0U,
        flush.centerRetreatFeedMmMin,flush.axisRetreatFeedMmMin,flush.centerReturnFeedMmMin,
        flush.axisReturnFeedMmMin,flush.pathRetreatFeedMmMin,flushReady&&flush.finalApproachEnabled?1U:0U,
        NCG76SphereGapBinding::ApproachReasonName(binding.approachReason),binding.approachFailedFieldId,
        binding.resolvedE11Percent,config.finalApproachDistanceMm,config.flushProfile.finalApproachMmPerMin,
        flush.finalApproachFeedMmMin);
    TraceG76SphereStartupSameThread(51U,"SPEED_LOG_DONE");
    // E11 has its own confirmed unit even when E13 has not enabled an
    // approach. This is diagnostic resolution only, never a motion target.
    double resolvedApproachCap=0.0;
    if(binding.e11UnitSource!=NCG76SphereGapBinding::E11UnitSource::Unavailable&&
        config.enabled&&std::isfinite(binding.resolvedE11Percent)&&binding.resolvedE11Percent>=10.0&&
        binding.resolvedE11Percent<=100.0){
        const double scaled=config.flushProfile.finalApproachMmPerMin*
            ((binding.resolvedE11Percent/100.0)*(config.jumpSpeedPercent/100.0));
        if(std::isfinite(scaled)&&scaled>0.0)resolvedApproachCap=(std::min)(s.request.feedMmMin,scaled);
    }
    PrintParent(s.sourceG,"event=E11_PERCENT_CONFIG session=%llu E11pct=%.6g unit=%% source=%s basis=NATIVE_PUMP_STG_AND_E7 nativeBase=%.6g E7pct=%.6g resolvedCapMmMin=%.6g approachEnabled=%u distanceZeroSentinel=%u discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.resolvedE11Percent,
        NCG76SphereGapBinding::E11UnitSourceName(binding.e11UnitSource),config.flushProfile.finalApproachMmPerMin,
        config.jumpSpeedPercent,resolvedApproachCap,flushReady&&flush.finalApproachEnabled?1U:0U,
        binding.distanceZeroSentinel?1U:0U);
    TraceG76SphereStartupSameThread(52U,"E11_LOG_DONE");
    PrintParent(s.sourceG,"event=APPROACH_CONFIG session=%llu status=%s reason=%s error=%s field=%u E11field=%u E13field=%u rawE11=%lld rawE13=%lld rawOnlyWhenDisabled=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.finalApproachEnabled?"APPROACH_ENABLED":"APPROACH_DISABLED",
        NCG76SphereGapBinding::ApproachReasonName(binding.approachReason),EDM58::BindingErrorName(binding.approachError),
        binding.approachFailedFieldId,binding.approachFields[0],binding.approachFields[1],
        static_cast<long long>(binding.approachValues[0]),static_cast<long long>(binding.approachValues[1]));
    PrintParent(s.sourceG,"event=INITIAL_SLOW_CONFIG session=%llu enabled=%u reason=%s error=%s field=%u E19field=%u E20field=%u rawE19=%lld rawE20=%lld activeE19pct=%.6g activeE20mm=%.6g nativeBase=%.6g capMmMin=%.6g unit=mm/min discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.initialSlowEnabled?1U:0U,
        NCG76SphereGapBinding::ApproachReasonName(binding.initialSlowReason),EDM58::BindingErrorName(binding.initialSlowError),
        binding.initialSlowFailedFieldId,binding.initialSlowFields[0],binding.initialSlowFields[1],
        static_cast<long long>(binding.initialSlowValues[0]),static_cast<long long>(binding.initialSlowValues[1]),
        config.initialSlowPercent,config.initialSlowDistanceMm,config.flushProfile.initialSlowMmPerMin,
        flush.initialSlowFeedMmMin);
    PrintParent(s.sourceG,"event=DEEP_FLUSH_CONFIG session=%llu enabled=%u reason=%s error=%s field=%u E21field=%u E22field=%u rawE21=%lld rawE22=%lld activePeriod=%u activeFactor=%.6g baseHeightMm=%.12f deepHeightMm=%.12f scope=%s count=SUCCESS_ONLY discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.deepFlushEnabled?1U:0U,
        NCG76SphereGapBinding::DeepFlushReasonName(binding.deepFlushReason),EDM58::BindingErrorName(binding.deepFlushError),
        binding.deepFlushFailedFieldId,binding.deepFlushFields[0],binding.deepFlushFields[1],
        static_cast<long long>(binding.deepFlushValues[0]),static_cast<long long>(binding.deepFlushValues[1]),
        config.deepFlushCycleInterval,config.deepFlushHeightMultiplier,config.jumpHeightMm,config.deepFlushHeightMm,
        processPath?"CONSECUTIVE_PROCESS_CHAIN":"SAME_PARENT");
    TraceG76SphereStartupSameThread(53U,"START_LOGS_DONE");
    return WaitForG76SphereCallback;
}

// EDM308 FIX1: a named callback makes the expected target independently
// comparable before NC invokes a saved function pointer. It does not alter
// the parent poll, completion proof, HOLD, or RT execution state machine.
bool NCManager::WaitForG76SphereCallback(NCManager* nc)
{
    if(nc==nullptr)return false;
    nc->TraceG76SphereStartupSameThread(G76SphereStartupCallbackEnter,"CALLBACK_ENTER");
    const bool ready=nc->PollG76SphereSameThread();
    nc->TraceG76SphereStartupSameThread(G76SphereStartupCallbackLeave,"CALLBACK_LEAVE");
    return ready;
}

bool NCManager::ValidateG76SphereWaitCallbackSameThread(WaitConditionFunc callback)
{
    const auto& s=*m_g76Sphere;
    if(!s.active||!s.bound||!s.submitted)return true;
    if(callback==WaitForG76SphereCallback)return true;
    // Compare representations only; never call or dereference the suspect
    // address. The existing rejection path revokes this parent and requests
    // the same controlled emergency stop used by other integrity failures.
    std::uint64_t actual=0ULL,expected=0ULL;
    const WaitConditionFunc target=WaitForG76SphereCallback;
    static_assert(sizeof(callback)<=sizeof(actual),"Callback diagnostic requires a bounded x64 pointer.");
    std::memcpy(&actual,&callback,sizeof(callback));std::memcpy(&expected,&target,sizeof(target));
    PrintParent(s.sourceG,"event=WAIT_CALLBACK_REJECT session=%llu pc=%d callback=0x%llx expected=0x%llx invoked=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc,static_cast<unsigned long long>(actual),
        static_cast<unsigned long long>(expected));
    // Clear the saved pointer before rejection retires active state. A later
    // ordinary wait boundary must never inherit this invalid address.
    m_waitCallback=nullptr;
    RejectG76SphereSameThread("WAIT_CALLBACK_CORRUPT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
    return false;
}

bool NCManager::ReadNCReturnProbeSameThread(const NCReturnProbe& probe,std::uintptr_t& actual) noexcept
{
    actual=0U;
    if(!probe.active||!probe.available||probe.slot==nullptr)return false;
    // Read exactly the representation of one saved pointer. Volatile byte
    // loads prevent reusing the entry sample and avoid pointer-type aliasing.
    // No preceding/following stack word is inspected, and nothing is written
    // through the slot pointer. Only the owning live frame supplies this slot.
    const volatile unsigned char* source=static_cast<const volatile unsigned char*>(probe.slot);
    unsigned char* target=reinterpret_cast<unsigned char*>(&actual);
    for(std::size_t i=0U;i<sizeof(actual);++i)target[i]=source[i];
    return true;
}

void NCManager::TraceG76SphereReturnProbeSameThread(NCReturnProbe& probe,const char* frame,unsigned stage,bool finish)
{
    const auto& s=*m_g76Sphere;
    const bool early=stage>=32U&&stage<64U&&m_g76Confirmation.confirmed&&m_state==NCState::RUN;
    if(probe.active&&stage<64U&&frame!=nullptr&&((s.active&&s.submitted&&s.bound)||early)){
        std::uintptr_t actual=0U;
        const bool read=ReadNCReturnProbeSameThread(probe,actual);
        const bool mismatch=read&&actual!=probe.expected;
        const std::uint64_t context=s.session?s.session:m_g76SphereNextSession;
        const bool first=probe.loggedSession!=context;
        if(first||(mismatch&&probe.mismatchLoggedSession!=context)){
            StartupStackPattern pattern{};
            if(mismatch)ReadStartupStackPattern(probe.slot,s.frame,pattern);
            probe.loggedSession=context;
            if(mismatch)probe.mismatchLoggedSession=context;
            PrintParent(s.sourceG,"event=RETURN_PROBE session=%llu pc=%d frame=%.16s stage=%u available=%u status=%s slot=0x%llx expected=0x%llx actual=0x%llx readBytes=%u action=OBSERVE_ONLY discharge=0\n",
                static_cast<unsigned long long>(s.session),s.pc,frame,stage,read?1U:0U,
                !read?"UNAVAILABLE":mismatch?"MISMATCH":"MATCH",
                static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(probe.slot)),
                static_cast<unsigned long long>(probe.expected),static_cast<unsigned long long>(actual),
                read?static_cast<unsigned>(sizeof(actual)):0U);
            if(mismatch)PrintParent(s.sourceG,"event=RETURN_PATTERN session=%llu frame=%.16s stage=%u readable=%u bytes=%u matched=%u exactSnapshot=%u q0=0x%llx q1=0x%llx q2=0x%llx q3=0x%llx expectedRun=%llu action=OBSERVE_ONLY discharge=0\n",
                static_cast<unsigned long long>(s.session),frame,stage,pattern.readable?1U:0U,
                pattern.readable?static_cast<unsigned>(sizeof(s.frame)):0U,pattern.matchedBytes,
                pattern.readable&&pattern.matchedBytes==sizeof(s.frame)?1U:0U,
                static_cast<unsigned long long>(pattern.words[0]),static_cast<unsigned long long>(pattern.words[1]),
                static_cast<unsigned long long>(pattern.words[2]),static_cast<unsigned long long>(pattern.words[3]),
                static_cast<unsigned long long>(s.frame.runToken));
        }
    }
    if(finish){
        // The RAII owner calls this before its function frame disappears.
        // Expired addresses cannot be sampled by a later startup stage.
        probe.active=false;probe.available=false;probe.slot=nullptr;probe.expected=0U;
    }
}

void NCManager::TraceG76SphereStartupSameThread(unsigned stage,const char* name)
{
    auto& s=*m_g76Sphere;
    const bool early=stage>=32U&&stage<64U&&m_g76Confirmation.confirmed&&m_state==NCState::RUN;
    if(stage>=64U||name==nullptr||(!(s.active&&s.submitted&&s.bound)&&!early))return;
    const std::uint64_t bit=1ULL<<stage;
    if((s.startupTraceMask&bit)!=0ULL)return;
    s.startupTraceMask|=bit;
    TraceG76SphereReturnProbeSameThread(m_ncDispatchScratch->engineReturnProbe,"ENGINE",stage);
    TraceG76SphereReturnProbeSameThread(m_ncDispatchScratch->taskReturnProbe,"TASK",stage);
    // The fine-grained Start checks only sample the owning live slots.
    // Healthy checks emit no additional formatted log. A first mismatch is
    // reported by RETURN_PROBE/RETURN_PATTERN with this exact stage number.
    if(stage>=40U&&stage<=53U)return;
    const WaitConditionFunc callback=m_waitCallback,target=WaitForG76SphereCallback;
    std::uint64_t actual=0ULL,expected=0ULL;
    static_assert(sizeof(callback)<=sizeof(actual),"Callback diagnostic requires a bounded x64 pointer.");
    std::memcpy(&actual,&callback,sizeof(callback));std::memcpy(&expected,&target,sizeof(target));
    // This helper is reached only by NC. At most 64 stage records per parent,
    // plus at most two return-probe records for each of the two NC frames.
    // No RT printing, allocation, stack query, or timing-guard modification.
    PrintParent(s.sourceG,"event=STARTUP_STAGE session=%llu pc=%d stage=%u name=%.32s callback=0x%llx expected=0x%llx match=%u sizeNC=%llu sizeSnapshot=%llu sizeFeedback=%llu discharge=0\n",
        static_cast<unsigned long long>(s.session),s.pc,stage,name,
        static_cast<unsigned long long>(actual),static_cast<unsigned long long>(expected),callback==target?1U:0U,
        static_cast<unsigned long long>(sizeof(NCManager)),static_cast<unsigned long long>(sizeof(NCTranslationSnapshot)),
        static_cast<unsigned long long>(sizeof(MotionG76SphereFeedback)));
}

void NCManager::CommitG76SphereCaptureSameThread(NCBlockDispatchId dispatch,const MotionProgramBlockCapture& capture,
    const NCProgramCommitSnapshot& commit,bool committed,bool ledgerFound,const NCBlockLifecycleSnapshot& ledger,int pc,int line)
{
    auto& s=*m_g76Sphere;if(!s.active||s.dispatch!=dispatch)return;
    const bool capturesF=s.sourceG==76||s.pathSource.has('F');
    const bool feedCurrent=capturesF?IsCncFeedValueCurrentSameThread(s.feed):
        !s.feed.valid&&!s.feed.programmed&&s.feed.feedMMMin==0.0&&s.request.nominalFeedCapMmMin==0.0;
    bool valid=committed&&commit.IsValid()&&ledgerFound&&!s.bound&&pc==s.pc&&line==s.line&&
        commit.scope==NCProgramScope::MEMORY&&commit.frameId==NC_PROGRAM_FRAME_ID_INVALID&&
        commit.sourcePC==pc&&commit.cacheGeneration==s.cache&&ledger.dispatchId==dispatch&&
        ledger.programCommitted&&!ledger.ncDispatchFailed&&!ledger.motionCaptureOverflow&&ledger.sourceLineNumber==line&&
        SameSource(commit,ledger.programTarget)&&SameSource(commit,ledger.programCommit)&&commit.sequence==ledger.programCommit.sequence&&
        capture.count==1U&&!capture.overflow&&ledger.motionSegmentCount==1U&&feedCurrent&&
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
    s.commit=commit.sequence;s.bound=true;
    if(capturesF)CommitCncModalFeedSameThread(s.feed,dispatch,s.commit,pc,line);
    // P50 is not a retained legacy history segment. Reusing pre-P50 B2 or
    // return history would bridge across an unrepresented three-dimensional path.
    m_edm171Air->history.Clear();ClearPathCoreReplayHistorySameThread();InvalidatePathCoreReturnCursorSameThread();
    m_g76Modal.lease=s.owner;m_g76Modal.run=s.run;m_g76Modal.cache=s.cache;m_g76Modal.commit=s.commit;
    m_g76Modal.mode=s.prepared.requestedB;m_g76Modal.valid=true;
    PrintParent(s.sourceG,"event=BOUND session=%llu dispatch=%llu commit=%llu children=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.dispatch),static_cast<unsigned long long>(s.commit));
}

void NCManager::ServiceG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return;
    TraceG76SphereStartupSameThread(G76SphereStartupServiceEnter,"SERVICE_ENTER");
    s.process=NCG76SphereProcess::Envelope{};
    s.gapShadow=NCG76SphereGapProcess::Feedback{};
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_ACTIVE");
    if(m_state!=NCState::RUN&&m_state!=NCState::HOLD){CancelG76SphereSameThread("STATE");return;}
    MotionG76SphereFeedback observed{};
    TraceG76SphereStartupSameThread(G76SphereStartupFeedbackReadBegin,"FEEDBACK_READ_BEGIN");
    const bool matching=m_motion.ReadG76SphereFeedback(observed)&&observed.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner);
    TraceG76SphereStartupSameThread(G76SphereStartupFeedbackReadEnd,"FEEDBACK_READ_END");
    // Preserve the original RT failure before a resulting alarm/epoch change
    // invalidates the NC scope. All original scope checks still run below.
    if(matching&&FailedSphereFeedback(observed))PrintSphereRuntimeFailure(observed,s.request,s.line,s.sourceG);
    TraceG76SphereStartupSameThread(G76SphereStartupScopeCheckBegin,"SCOPE_CHECK_BEGIN");
    const bool scopeCurrent=IsG76SphereScopeCurrentSameThread();
    TraceG76SphereStartupSameThread(G76SphereStartupScopeCheckEnd,"SCOPE_CHECK_END");
    if(!scopeCurrent){
        RejectG76SphereSameThread("LIVE_SOURCE_FRAME_OR_TRANSPORT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
    }
    TraceG76SphereStartupSameThread(G76SphereStartupRecipeCheckBegin,"RECIPE_CHECK_BEGIN");
    const bool recipeChanged=s.gapBinding.ready&&!s.gapShadowRevoked&&!NCG76SphereGapBinding::IsCurrent(s.gapBinding,m_recipe,
        m_edmProcessProfile,m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly);
    TraceG76SphereStartupSameThread(G76SphereStartupRecipeCheckEnd,"RECIPE_CHECK_END");
    if(recipeChanged){
        s.gapShadowRevoked=true;s.gapBinding.ready=false;
        (void)m_motion.RevokeG76SphereProcessShadow(s.session,s.identity,s.owner);
        PrintParent(s.sourceG,"event=SIM_GAP_WITHDRAWN session=%llu reason=RECIPE_OR_TUNING_CHANGED rearm=NEXT_PARENT motionApplied=0 discharge=0\n",
            static_cast<unsigned long long>(s.session));
    }
    if(!matching){
        TraceG76SphereStartupSameThread(G76SphereStartupServiceDone,"SERVICE_DONE");return;
    }
    const bool flushLegChanged=s.feedback.cycle.flushLegSequence!=observed.cycle.flushLegSequence;
    const bool approachChanged=s.feedback.cycle.flushApproachActive!=observed.cycle.flushApproachActive;
    const bool initialSlowChanged=s.feedback.cycle.flushInitialSlowActive!=observed.cycle.flushInitialSlowActive;
    const bool probeChanged=s.feedback.processShadow.probePhase!=observed.processShadow.probePhase||
        s.feedback.processShadow.valid!=observed.processShadow.valid;
    s.feedback=observed;
    if(observed.started&&observed.cycle.session==s.session&&observed.cycle.retreatMode!=s.request.retreatMode){
        RejectG76SphereSameThread("RETREAT_MODE_FEEDBACK",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
    }
    if(observed.started&&observed.cycle.session==s.session&&
       (observed.cycle.finishRequiredCount!=s.finishRequiredCount||
        observed.cycle.finishVerifiedCount>s.finishRequiredCount||
        observed.cycle.finishForcedCompletedCount>observed.cycle.finishAttemptCount)){
        RejectG76SphereSameThread("FINISH_FEEDBACK_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
    }
    if(FailedSphereFeedback(observed)){
        RejectG76SphereSameThread("RUNTIME_FAULT",AlarmManager::PATH_EXECUTION_NOT_READY);return;
    }
    if(s.finishRequiredCount&&observed.started&&observed.cycle.session==s.session){
        const auto& f=observed.cycle;
        if(!s.finishLogInitialized||s.finishLoggedPhase!=f.finishPhase||s.finishLoggedVerified!=f.finishVerifiedCount||
           s.finishLoggedAttempt!=f.finishAttemptCount||s.finishLoggedCompleted!=f.finishForcedCompletedCount||
           s.finishLoggedRework!=f.finishReworkCount){
            s.finishLogInitialized=true;s.finishLoggedPhase=f.finishPhase;s.finishLoggedVerified=f.finishVerifiedCount;
            s.finishLoggedAttempt=f.finishAttemptCount;s.finishLoggedCompleted=f.finishForcedCompletedCount;
            s.finishLoggedRework=f.finishReworkCount;
            PrintParent(s.sourceG,"event=FINISH_VERIFY session=%llu tick=%llu required=%u verified=%u phase=%u active=%u attempts=%llu forcedCompleted=%llu reworks=%llu origin=%u E5=SUSPENDED_DURING_FINISH discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick),s.finishRequiredCount,
                f.finishVerifiedCount,static_cast<unsigned>(f.finishPhase),f.finishActive?1U:0U,
                static_cast<unsigned long long>(f.finishAttemptCount),static_cast<unsigned long long>(f.finishForcedCompletedCount),
                static_cast<unsigned long long>(f.finishReworkCount),static_cast<unsigned>(f.flushOrigin));
        }
    }
    if(s.sourceG!=76&&observed.started&&observed.cycle.session==s.session&&!s.processSeedLogged){
        s.processSeedLogged=true;
        PrintParent(s.sourceG,"event=CHAIN_SEED session=%llu initialIntervalWorkUs=%llu completedFlushOffset=%llu previousParentReady=%u previousSession=%llu previousPC=%d historyCapacity=1 E5=NOMINAL_FORWARD_ONLY ordinal=SUCCESS_ONLY discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.initialIntervalWorkUs),
            static_cast<unsigned long long>(observed.completedFlushOffset),
            observed.cycle.previousParentHistoryReady?1U:0U,
            static_cast<unsigned long long>(observed.previousParentSession),observed.previousParentPC);
        PrintParent(s.sourceG,"event=SIM_TEST_POLICY session=%llu mode=%s probe=%s targetMaxMm=%.6f source=SOFTWARE_SIMULATION motionAuthority=UNCHANGED discharge=0\n",
            static_cast<unsigned long long>(s.session),
            NCG76SphereGapProcess::SimulationModeName(observed.processShadow.simulationMode),
            NCG76SphereGapProcess::ProbePhaseName(observed.processShadow.probePhase),
            NCG76SphereGapProcess::PreviousParentProbeRetreatMm);
    }
    if(observed.started&&observed.cycle.session==s.session)
        (void)NCG76SphereProcess::Observe(s.path,observed.cycle,s.process);
    const auto& cycle=observed.cycle;
    const auto& probe=observed.processShadow;
    if(s.sourceG!=76&&observed.started&&cycle.session==s.session&&
       probe.enabled&&probe.session==s.session&&probe.tick==cycle.tick&&
       probe.simulationMode==NCG76SphereGapProcess::SimulationMode::PreviousParentProbe&&
       probeChanged){
        PrintParent(s.sourceG,"event=SIM_PROBE session=%llu phase=%s tick=%llu ready=%u entered=%llu returned=%llu previousProgressMm=%.9f previousLengthMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),NCG76SphereGapProcess::ProbePhaseName(probe.probePhase),
            static_cast<unsigned long long>(cycle.tick),cycle.previousParentHistoryReady?1U:0U,
            static_cast<unsigned long long>(cycle.historyBoundarySequence),
            static_cast<unsigned long long>(cycle.historyReturnSequence),
            cycle.previousParentProgressMm,cycle.previousParentLengthMm,
            observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
            observed.actualMCS[0],observed.actualMCS[1],observed.actualMCS[2]);
    }
    if(observed.started&&cycle.session==s.session){
        const auto printEvent=[&](const char* event,std::uint64_t sequence){
            if(s.flushDiagnosticEvents++<192U)
                PrintParent(s.sourceG,"event=%s session=%llu request=%llu tick=%llu phase=%s completed=%llu checkpoint=%u B=%u leg=%llu selectionRequest=%llu selectionCurrent=%u ordinal=%llu heightMm=%.12f deep=%u source=SOFTWARE_SIMULATION discharge=0\n",
                    event,static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(sequence),
                    static_cast<unsigned long long>(cycle.tick),PhaseName(cycle.phase,s.sourceG),
                    static_cast<unsigned long long>(cycle.flushCount),cycle.flushCheckpointSaved?1U:0U,s.request.retreatMode,
                    static_cast<unsigned long long>(cycle.flushLegSequence),
                    static_cast<unsigned long long>(cycle.flushRequestSequence),sequence==cycle.flushRequestSequence?1U:0U,
                    static_cast<unsigned long long>(cycle.flushOrdinal),cycle.flushSelectedHeightMm,cycle.flushDeepActive?1U:0U);
        };
        if(cycle.flushRequestSequence>s.flushAcceptedLogged){
            s.flushAcceptedLogged=cycle.flushRequestSequence;printEvent("FLUSH_START",cycle.flushRequestSequence);
        }
        if(cycle.flushCompletedSequence>s.flushCompletedLogged){
            s.flushCompletedLogged=cycle.flushCompletedSequence;printEvent("FLUSH_COMPLETE",cycle.flushCompletedSequence);
        }
        if(cycle.flushAbandonedSequence>s.flushAbandonedLogged){
            s.flushAbandonedLogged=cycle.flushAbandonedSequence;printEvent("FLUSH_ABORT_HOLD",cycle.flushAbandonedSequence);
        }
    }
    const bool phaseChanged=!s.diagnosticInitialized||s.lastPhase!=observed.cycle.phase||s.lastLayer!=observed.cycle.layerIndex||flushLegChanged||approachChanged||initialSlowChanged;
    if(s.gapBinding.ready&&!s.gapShadowRevoked&&observed.started){
        const auto& shadow=observed.processShadow;
        const bool coherent=shadow.enabled&&shadow.session==s.session&&shadow.tick==observed.cycle.tick&&
            shadow.recipeGeneration==s.gapBinding.config.recipeGeneration&&
            shadow.tuningGeneration==s.gapBinding.config.tuningGeneration;
        const bool valid=coherent&&shadow.valid;
        const bool gapApplied=valid&&s.gapBinding.config.signedMotionEnabled&&
            cycle.session==s.session&&cycle.gapMotionEnabled&&cycle.gapApplied;
        if(valid)s.gapShadow=shadow;
        const int direction=valid?(shadow.signedGapSpeedMmMin>0.0?1:shadow.signedGapSpeedMmMin<0.0?-1:0):0;
        if(coherent&&(!s.gapDiagnosticInitialized||phaseChanged||s.gapLastDirection!=direction||
            s.gapLastValid!=valid||s.gapLastSuspended!=shadow.suspended)){
            s.gapDiagnosticInitialized=true;s.gapLastDirection=direction;s.gapLastValid=valid;
            s.gapLastSuspended=shadow.suspended;
            if(s.gapDiagnosticEvents++<64U)
                PrintParent(s.sourceG,"event=SIM_GAP session=%llu tick=%llu valid=%u phase=%s layer=%u voltageV=%.3f signedTargetMmMin=%.6f eligible=%u suspended=%u simulatedWorkUs=%llu pending=%u reason=%s source=SOFTWARE_SIMULATION motionApplied=%u actualSignedMmMin=%.6f discharge=0\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(shadow.tick),valid?1U:0U,
                    PhaseName(observed.cycle.phase,s.sourceG),observed.cycle.layerIndex,valid?shadow.voltageV:0.0,
                    valid?shadow.signedGapSpeedMmMin:0.0,valid&&shadow.eligible?1U:0U,shadow.suspended?1U:0U,
                    static_cast<unsigned long long>(valid?shadow.simulatedWorkUs:0ULL),valid&&shadow.pending?1U:0U,
                    NCG76SphereGapProcess::ReasonName(shadow.reason),gapApplied?1U:0U,
                    gapApplied?cycle.signedSpeedMmS*60.0:0.0);
        }
        if(valid&&shadow.pending&&shadow.requestSequence>s.gapRequestLogged){
            s.gapRequestLogged=shadow.requestSequence;
            PrintParent(s.sourceG,"event=FLUSH_REQUEST session=%llu request=%llu tick=%llu workUs=%llu phase=%s layer=%u candidateX=%.9f candidateY=%.9f candidateZ=%.9f time=SIMULATED_WORK candidateOnly=1 stoppedProof=0 motionApplied=0 discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(shadow.requestSequence),
                static_cast<unsigned long long>(shadow.requestTick),static_cast<unsigned long long>(shadow.simulatedWorkUs),
                PhaseName(shadow.nominalPhase,s.sourceG),shadow.layerIndex,shadow.candidateMCS[0],shadow.candidateMCS[1],shadow.candidateMCS[2]);
        }
    }
    // RT can consume a START on the same callback that a held position
    // leaves its arrival window. That callback requalifies the fixed start;
    // it must neither resume automatically nor leave NC waiting forever for
    // a resume intent which RT has already consumed. The published control
    // high-water proves consumption; a merely pending queue is insufficient.
    if(s.resumePending&&m_state==NCState::HOLD&&observed.started&&!observed.terminal&&
        observed.session==s.session&&SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&
        !observed.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
        cycle.session==s.session&&observed.holdSequence==s.holdSequence&&cycle.holdSequence==s.holdSequence&&
        cycle.holdActive&&observed.startSequence>=s.startSequence&&cycle.resumeSequence<s.startSequence&&
        (cycle.phase==NCG76SphereCycle::Phase::HoldSettle||cycle.phase==NCG76SphereCycle::Phase::RecoverySettle||
         cycle.phase==NCG76SphereCycle::Phase::HeldAtStart)){
        s.resumePending=false;
        PrintParent(s.sourceG,"event=START_CONSUMED session=%llu start=%llu reason=POSITION_REQUALIFICATION freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.startSequence));
    }
    if(s.resumePending&&m_state==NCState::HOLD&&observed.started&&!observed.terminal&&
        observed.session==s.session&&cycle.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&
        !observed.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
        observed.holdSequence==s.holdSequence&&cycle.holdSequence==s.holdSequence&&
        cycle.holdRecoveryEnabled&&cycle.holdRecoveryPaused&&cycle.held&&cycle.holdActive&&
        observed.startSequence>=s.startSequence){
        // A fresh START may already own a multi-leg return when a new HOLD
        // or inspection protection pauses it. Retire that admitted START at
        // the proved stopped boundary, without discarding its saved route.
        s.resumePending=false;
        PrintParent(s.sourceG,"event=START_CONSUMED session=%llu start=%llu reason=RECOVERY_PAUSED freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.startSequence));
    }
    if(s.resumePending&&observed.session==s.session&&cycle.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&
        !observed.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
        observed.cycle.resumeSequence==s.startSequence&&!observed.cycle.holdActive&&
        !observed.cycle.held&&observed.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        s.resumePending=false;m_state=NCState::RUN;
        PrintParent(s.sourceG,"event=RESUMED session=%llu hold=%llu start=%llu tick=%llu nominalProgressMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.holdSequence),
            static_cast<unsigned long long>(s.startSequence),static_cast<unsigned long long>(cycle.tick),
            cycle.nominalProgressMm,observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
            observed.actualMCS[0],observed.actualMCS[1],observed.actualMCS[2]);
    }
    if(phaseChanged){
        s.diagnosticInitialized=true;s.lastPhase=observed.cycle.phase;s.lastLayer=observed.cycle.layerIndex;
        // EDM311: distinguish an accepted HOLD request from the RT's proved
        // stopped state. This observes feedback only and grants no resume.
        if((s.sourceG!=76||cycle.holdRecoveryEnabled)&&cycle.held&&cycle.phase==NCG76SphereCycle::Phase::HeldAtStart)
            PrintParent(s.sourceG,"event=HELD session=%llu hold=%llu tick=%llu nominalProgressMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f path=%s freshStartRequired=1 discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(cycle.holdSequence),
                static_cast<unsigned long long>(cycle.tick),cycle.nominalProgressMm,
                observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
                observed.actualMCS[0],observed.actualMCS[1],observed.actualMCS[2],
                cycle.holdRecoveryEnabled?"RETAINED_PROCESS_PATH":"IN_PLACE");
        if(s.diagnosticEvents++<512U||observed.terminal||observed.cycle.held){
            PrintParent(s.sourceG,"event=STATE session=%llu tick=%llu phase=%s layer=%u finishedLayers=%u progress=%.9f held=%u heldRequal=%llu xyzTol=%.9f rotaryTol=%.9f flushActive=%u request=%llu flushes=%llu discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.cycle.tick),
                PhaseName(observed.cycle.phase,s.sourceG),observed.cycle.layerIndex,observed.cycle.completedLayers,
                observed.cycle.nominalProgressMm,observed.cycle.held?1U:0U,
                static_cast<unsigned long long>(cycle.heldRequalificationSequence),observed.positionToleranceMm,observed.rotaryToleranceDeg,cycle.flushActive?1U:0U,
                static_cast<unsigned long long>(cycle.flushRequestSequence),static_cast<unsigned long long>(cycle.flushCount));
            if(cycle.holdRecoveryEnabled&&cycle.holdActive)
                PrintParent(s.sourceG,"event=HOLD_ROUTE session=%llu hold=%llu stage=%u declared=%u effective=%u available=%u jog=%u trace=%u anchorX=%.9f anchorY=%.9f anchorZ=%.9f processX=%.9f processY=%.9f processZ=%.9f E5=SUSPENDED discharge=0\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(cycle.holdSequence),
                    static_cast<unsigned>(cycle.holdRecoveryPhase),cycle.holdDeclaredCount,cycle.holdEffectiveCount,
                    cycle.holdAvailableCount,cycle.inspectionJogActive?1U:0U,cycle.inspectionTraceCount,
                    observed.holdRetreatAnchorMCS[0],observed.holdRetreatAnchorMCS[1],observed.holdRetreatAnchorMCS[2],
                    observed.holdProcessPointMCS[0],observed.holdProcessPointMCS[1],observed.holdProcessPointMCS[2]);
            if(cycle.gapHistoryActive||cycle.historyBoundarySequence||cycle.historyReturnSequence)
                PrintParent(s.sourceG,"event=HISTORY_STATE session=%llu tick=%llu phase=%s active=%u boundary=%llu returned=%llu nominal=%s layer=%u X=%.9f Y=%.9f Z=%.9f scope=%s depth=1 previousProgressMm=%.9f previousLengthMm=%.9f E5=SUSPENDED_WHILE_ACTIVE discharge=0\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(cycle.tick),
                    PhaseName(cycle.phase,s.sourceG),cycle.gapHistoryActive?1U:0U,
                    static_cast<unsigned long long>(cycle.historyBoundarySequence),
                    static_cast<unsigned long long>(cycle.historyReturnSequence),PhaseName(cycle.nominalPhase,s.sourceG),
                    cycle.layerIndex,observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
                    cycle.previousParentHistoryReady?"PREVIOUS_PARENT":"SAME_PARENT",
                    cycle.previousParentProgressMm,cycle.previousParentLengthMm);
            // Nominal station stays at its saved machining checkpoint during
            // return/resume; it is deliberately not the excursion's position.
            const auto& p=s.process;
            PrintParent(s.sourceG,"event=PROCESS_CURSOR session=%llu valid=%u phase=%s role=%u stage=%u layer=%u radiusMm=%.9f depthMm=%.9f stationMm=%.9f saved=%u atNominal=%u gap=0 discharge=0\n",
                static_cast<unsigned long long>(s.session),p.valid?1U:0U,PhaseName(observed.cycle.phase,s.sourceG),
                static_cast<unsigned>(p.role),static_cast<unsigned>(p.nominal.cursor.span.stage),
                p.nominal.cursor.span.layerIndex,p.layerRadiusMm,p.layerDepthMm,p.nominal.cursor.stationMm,
                p.checkpointSaved?1U:0U,p.atNominalPath?1U:0U);
            if(cycle.flushActive&&(!cycle.holdRecoveryEnabled||!cycle.holdActive)){
                const bool finishFlush=cycle.flushOrigin==2U;
                const auto diagnosticSequence=finishFlush?cycle.finishAttemptCount:cycle.flushRequestSequence;
                PrintParent(s.sourceG,"event=%s session=%llu request=%llu B=%u phase=%s leg=%llu stationMm=%.9f anchorStationMm=%.9f checkpoint=%u E5=SUSPENDED scope=SAME_PARENT discharge=0\n",
                    finishFlush?"FINISH_FLUSH_ROUTE":"FLUSH_ROUTE",static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(diagnosticSequence),
                    s.request.retreatMode,PhaseName(cycle.phase,s.sourceG),static_cast<unsigned long long>(cycle.flushLegSequence),
                    cycle.flushPathStationMm,cycle.flushAnchorStationMm,cycle.flushCheckpointSaved?1U:0U);
                PrintParent(s.sourceG,"event=%s session=%llu request=%llu B=%u phase=%s leg=%llu stageSpeedMmMin=%.6f selectedMmMin=%.6f targetMmMin=%.6f remainingMm=%.9f approachActive=%u initialSlowActive=%u E5=SUSPENDED discharge=0\n",
                    finishFlush?"FINISH_FLUSH_SPEED":"FLUSH_SPEED",static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(diagnosticSequence),
                    s.request.retreatMode,PhaseName(cycle.phase,s.sourceG),static_cast<unsigned long long>(cycle.flushLegSequence),
                    cycle.speedMmS*60.0,cycle.flushSelectedFeedMmMin,cycle.flushTargetFeedMmMin,
                    cycle.flushRemainingMm,cycle.flushApproachActive?1U:0U,cycle.flushInitialSlowActive?1U:0U);
                PrintParent(s.sourceG,"event=%s session=%llu request=%llu phase=%s X=%.9f Y=%.9f Z=%.9f nominalX=%.9f nominalY=%.9f nominalZ=%.9f checkpoint=%u discharge=0\n",
                    finishFlush?"FINISH_FLUSH_POSITION":"FLUSH_POSITION",static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(diagnosticSequence),
                    PhaseName(cycle.phase,s.sourceG),observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
                    p.nominal.position[0],p.nominal.position[1],p.nominal.position[2],cycle.flushCheckpointSaved?1U:0U);
            }
        }
    }
    TraceG76SphereStartupSameThread(G76SphereStartupServiceDone,"SERVICE_DONE");
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
    if(!FinishTerminalQualified(s.feedback.cycle,s.finishRequiredCount)){
        RejectG76SphereSameThread("FINISH_TERMINAL_NOT_QUALIFIED",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
    }
    if(!s.endpointCommitted){
        if(!m_motion.CommitG76SphereCompletedEndpoint(s.session,s.identity,s.owner,s.endpoint))return false;
        for(unsigned a=0U;a<8U;++a)if(!SameDouble(s.endpoint[a],s.request.geometry.endMCS[a])){
            RejectG76SphereSameThread("TERMINAL_ENDPOINT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
        }
        for(unsigned a=0U;a<8U;++a)CoordSys.commandedMCS[a]=s.endpoint[a];s.endpointCommitted=true;
    }
    if(!m_motion.ReleaseG76SphereBarrier(s.session,s.identity,s.owner))return false;
    s.active=false;s.completed=true;s.submitted=false;UpdateSystemVariables();
    if(s.sourceG!=76){
        if(s.finishRequiredCount)PrintParent(s.sourceG,"event=FINISH_COMPLETE session=%llu required=%u verified=%u attempts=%llu forcedCompleted=%llu reworks=%llu endpointCommitted=1 countScope=THIS_PARENT discharge=0\n",
            static_cast<unsigned long long>(s.session),s.finishRequiredCount,s.feedback.cycle.finishVerifiedCount,
            static_cast<unsigned long long>(s.feedback.cycle.finishAttemptCount),
            static_cast<unsigned long long>(s.feedback.cycle.finishForcedCompletedCount),
            static_cast<unsigned long long>(s.feedback.cycle.finishReworkCount));
        PrintParent(s.sourceG,"event=COMPLETE session=%llu g=%d X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu completedFlushOffset=%llu historyEntered=%llu historyReturned=%llu simProbe=%s countScope=THIS_PARENT discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceG,s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            static_cast<unsigned long long>(s.feedback.completedFlushOffset),
            static_cast<unsigned long long>(s.feedback.cycle.historyBoundarySequence),
            static_cast<unsigned long long>(s.feedback.cycle.historyReturnSequence),
            NCG76SphereGapProcess::ProbePhaseName(s.feedback.processShadow.probePhase));
    }else{
        PrintParent(s.sourceG,"event=COMPLETE session=%llu layers=%u X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.circleLayerCount,s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }
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
        PrintParent(s.sourceG,"event=HOLD_AT_COMPLETED_PARENT session=%llu discharge=0\n",
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
    PrintParent(s.sourceG,"event=HOLD_REQUEST session=%llu hold=%llu path=%s discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.holdSequence),
        s.feedback.cycle.holdRecoveryEnabled?"RETAINED_PROCESS_PATH_FRESH_START":
        s.sourceG==76?"CENTER_THEN_ORIGINAL_START":"IN_PLACE_FRESH_START");
    return true;
}

bool NCManager::TryResumeG76SphereSameThread()
{
    auto& s=*m_g76Sphere;if(!s.active)return false;
    ServiceG76SphereSameThread();if(!s.active)return true;
    if(m_state==NCState::HOLD&&s.feedback.terminal&&s.feedback.completed){
        m_state=NCState::RUN;return true; // Poll still proves ledger/endpoint/barrier.
    }
    if(m_state==NCState::HOLD&&s.feedback.cycle.holdRecoveryEnabled&&s.feedback.cycle.held&&
        !s.feedback.holdRecoveryResumeReady){
        PrintParent(s.sourceG,"event=START_CONSUMED session=%llu reason=INSPECTION_RECOVERY_NOT_READY freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session));return true;
    }
    if(m_state!=NCState::HOLD||s.resumePending||!s.feedback.started||
        s.feedback.holdSequence!=s.holdSequence||s.feedback.cycle.holdSequence!=s.holdSequence||
        !s.feedback.cycle.held||
        s.feedback.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        PrintParent(s.sourceG,"event=START_CONSUMED session=%llu reason=%s freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.feedback.cycle.holdRecoveryEnabled?
            "WAIT_HOLD_RECOVERY_READY":s.sourceG==76?"WAIT_HELD_AT_START":"WAIT_HELD_IN_PLACE");return true;
    }
    if(!s.bound){
        RejectG76SphereSameThread("RESUME_INTENT",AlarmManager::PATH_EXECUTION_NOT_READY);return true;
    }
    const auto heldRequalification=s.feedback.cycle.heldRequalificationSequence;
    if(!m_motion.RequestG76SphereResume(s.session,s.identity,s.owner,s.holdSequence,m_startDiagnosticRequests)){
        // Held feedback may enter and even finish position requalification
        // between Service and intent admission. Its monotone RT counter proves
        // that boundary even if the next NC read already observes Held again.
        // An unexplained rejection with the same counter remains a fault.
        ServiceG76SphereSameThread();if(!s.active)return true;
        const auto& report=s.feedback;const auto& cycle=report.cycle;
        if(m_state==NCState::HOLD&&report.started&&!report.terminal&&report.session==s.session&&cycle.session==s.session&&
            SameIdentity(report.identity,s.identity)&&report.owner.Matches(s.owner)&&
            !report.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
            report.holdSequence==s.holdSequence&&cycle.holdSequence==s.holdSequence&&cycle.holdActive&&
            cycle.heldRequalificationSequence>heldRequalification&&
            (cycle.phase==NCG76SphereCycle::Phase::HoldSettle||cycle.phase==NCG76SphereCycle::Phase::RecoverySettle||
             cycle.phase==NCG76SphereCycle::Phase::HeldAtStart)&&IsG76SphereScopeCurrentSameThread()){
            PrintParent(s.sourceG,"event=START_CONSUMED session=%llu reason=POSITION_REQUALIFICATION freshStartRequired=1 discharge=0\n",
                static_cast<unsigned long long>(s.session));return true;
        }
        if(cycle.holdRecoveryEnabled&&m_state==NCState::HOLD&&report.started&&!report.terminal&&
            !report.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
            report.session==s.session&&cycle.session==s.session&&
            SameIdentity(report.identity,s.identity)&&report.owner.Matches(s.owner)&&
            report.holdSequence==s.holdSequence&&cycle.holdSequence==s.holdSequence&&
            cycle.holdActive&&(cycle.phase==NCG76SphereCycle::Phase::HeldAtStart||
                cycle.phase==NCG76SphereCycle::Phase::RecoverySettle)&&IsG76SphereScopeCurrentSameThread()){
            // A protection/lease/queue qualification can change after the NC
            // read. Denied inspection recovery remains stopped and consumes
            // this START; it neither grants motion nor queues a later resume.
            PrintParent(s.sourceG,"event=START_CONSUMED session=%llu reason=RECOVERY_INTENT_NOT_ADMITTED freshStartRequired=1 discharge=0\n",
                static_cast<unsigned long long>(s.session));return true;
        }
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
    const bool reset=reason&&std::strcmp(reason,"RESET")==0;
    MotionG76SphereFeedback observed{};
    const bool reportFailure=s.submitted&&m_motion.ReadG76SphereFeedback(observed)&&observed.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&FailedSphereFeedback(observed);
    const bool submitted=s.submitted;s.active=s.staged=s.resumePending=false;s.completed=false;
    s.process=NCG76SphereProcess::Envelope{};
    s.gapShadow=NCG76SphereGapProcess::Feedback{};
    // EDM311 FIX1: a 250 us RT callback may interrupt this NC cancellation
    // before Reset() establishes the whole-PDO hold and exact reset scope.
    // Keep its immutable SIM source valid through that handoff; withdrawing
    // it here makes a still-authorized signed-motion parent fail source proof.
    // StepG76SphereRuntime withdraws it when the exact Reset output hold owns
    // the callback. Other cancellation retains immediate containment.
    if(submitted&&!reset)(void)m_motion.RevokeG76SphereProcessShadow(s.session,s.identity,s.owner);
    s.gapShadowRevoked=true;s.gapBinding.ready=false;
    m_g76PowerInhibited=true;ForceEDMPLCOutputsOffSameThread("P50_CANCEL");
    // RESET retains exact sidecar identity for the existing ticketed zero-hold
    // batch. All other cancellation is containment, never an automatic return.
    if(submitted&&!reset)m_motion.RequestEmergencyStopAllAxes();
    if(reportFailure)PrintSphereRuntimeFailure(observed,s.request,s.line,s.sourceG);
    PrintParent(s.sourceG,"event=CANCEL session=%llu reason=%s observerWithdraw=%s automaticReturn=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),reason?reason:"UNSPECIFIED",
        reset?"EXACT_RESET_OUTPUT_HOLD":"NC_CONTAINMENT");
}
