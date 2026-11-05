#include "NCManager.h"
#include "NCGCodeSemantics.h"
#include "AlarmManager.h"
#include "NCG76StartDiagnostic.h"
#include "NCPathCoreRadiusArc.h"
#include "NCHoldTelemetry.h"
#include "NCMachiningTelemetry.h"
#include "NCMachiningTimePolicy.h"
#include "NCG76TimedP11Source.h"
#include "NCFinishPolicy.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <utility>
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
    if(cycle.timedCompletion)return cycle.timeLimitReached&&
        required<=NCFinishPolicy::MaximumForcedFlushes&&cycle.finishRequiredCount==required&&
        !cycle.finishActive&&cycle.finishVerifiedCount<=required&&
        cycle.finishForcedCompletedCount<=cycle.finishAttemptCount&&
        cycle.finishPhase==(required?6U:0U);
    return required<=NCFinishPolicy::MaximumForcedFlushes&&cycle.finishRequiredCount==required&&
        cycle.finishVerifiedCount==required&&(!required||(!cycle.finishActive&&cycle.finishPhase==4U&&
        cycle.finishAttemptCount>=required&&cycle.finishForcedCompletedCount>=required&&
        cycle.finishForcedCompletedCount<=cycle.finishAttemptCount));
}
bool OrbitTerminalQualified(const NCG76SphereCycle::Feedback& f,const NCG76SphereCycle::Request& r) noexcept
{
    if(f.orbitRetryLimit!=r.orbitRetryLimit)return false;
    if(f.timedCompletion)return MotionG76SphereMachiningTime::TimedCompletionQualified(f,r);
    if(f.orbitCompletionState==static_cast<std::uint8_t>(NCG76OrbitCompletion::State::RetryExhausted))
        return MotionG76SphereMachiningTime::RetryExhaustionQualified(f,r);
    if(f.orbitRequiredPasses!=r.orbitInspectionRounds||f.orbitTargetPercentScaled!=r.orbitThresholdScaled)return false;
    if(!r.orbitInspectionRounds)return f.orbitCompletionState==0U&&!f.orbitCompletionActive&&!f.orbitPercentAvailable&&
        f.orbitQualifiedPasses==0U&&f.orbitAttemptCount==0ULL&&f.orbitPercentScaled==0U;
    return f.orbitCompletionState==4U&&!f.orbitCompletionActive&&f.orbitPercentAvailable&&
        f.orbitQualifiedPasses==r.orbitInspectionRounds&&f.orbitAttemptCount>=r.orbitInspectionRounds&&
        f.orbitPercentScaled>=r.orbitThresholdScaled&&f.orbitPercentScaled<=NCOrbitCompletionPolicy::MaximumPercentScaled;
}
const char* ParentName(int sourceG,unsigned sourceP=50U) noexcept
{ return sourceG==1?"[G01]":sourceG==2?"[G02]":sourceG==3?"[G03]":sourceP==1U?"[P01]":sourceP==2U?"[P02]":sourceP==3U?"[P03]":sourceP==4U?"[P04]":sourceP==5U?"[P05]":sourceP==6U?"[P06]":sourceP==7U?"[P07]":sourceP==11U?"[P11]":sourceP==12U?"[P12]":sourceP==13U?"[P13]":sourceP==14U?"[P14]":sourceP==15U?"[P15]":sourceP==16U?"[P16]":sourceP==17U?"[P17]":sourceP==21U?"[P21]":sourceP==22U?"[P22]":sourceP==23U?"[P23]":sourceP==24U?"[P24]":sourceP==25U?"[P25]":sourceP==27U?"[P27]":sourceP==26U?"[P26]":"[P50]"; }
bool SameLiteral(const NCBlock& a,const NCBlock& b) noexcept
{
    if(a.hasG!=b.hasG||a.gCode!=b.gCode||a.gCount!=b.gCount||a.mCount!=b.mCount||
        a.isEmpty!=b.isEmpty||a.isBlockSkip!=b.isBlockSkip||a.isGoto!=b.isGoto||a.gotoTarget!=b.gotoTarget)return false;
    for(char c='A';c<='Z';++c)if(a.has(c)!=b.has(c)||!SameDouble(a.val(c),b.val(c)))return false;
    for(int i=0;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(a.gCodes[i]!=b.gCodes[i])return false;
    for(int i=0;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(a.mCode[i]!=b.mCode[i])return false;
    return true;
}
bool TimedP11SourceMatchesGeometry(const NCG76TimedP11Source::Words& words,
    const NCMachiningTime::Config& time,const NCG76SphereRuntimeGeometry::Geometry& geometry) noexcept
{
    if(!words.valid||geometry.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||
       !words.authored.has('P')||!NCG76TimedP11Source::SupportedMode(words.authored.val('P'))||
       NCG76OrbitContour::ModeP(geometry)!=static_cast<unsigned>(words.authored.val('P'))||
       geometry.orbit.source.targetAxis!=(words.authored.val('P')!=11.0)||
       geometry.orbit.source.polygonSides!=words.polygonSides||
       !SameDouble(geometry.orbit.source.radiusMm,words.radiusProgramMm)||
       !SameDouble(geometry.orbit.source.shapeAngleDeg,words.contourAngleDeg)||
       !SameDouble(geometry.orbit.source.feedMmMin,words.feedMmMin)||
       !SameDouble(geometry.feedMmMin,words.feedMmMin)||
       geometry.orbit.finishEnabled!=(time.additionalUs!=0ULL||words.inspectionRounds!=0U))return false;
    for(unsigned axis=0U;axis<3U;++axis){
        const double target=words.targetMask&(1U<<axis)?words.authoredXYZ[axis]:geometry.frame.programStart[axis];
        if(!SameDouble(geometry.orbit.source.start[axis],geometry.frame.programStart[axis])||
           !SameDouble(geometry.orbit.source.end[axis],target))return false;
    }
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
    NCMachiningTime::Config machiningTime{};
    if(!NCMachiningTimePolicy::StripValidated(parsed,resolved,literal,machiningTime))return false;
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
void PrintParentV(int sourceG,unsigned sourceP,const char* format,va_list args) noexcept {
    char line[481]{};
    const int prefix=std::snprintf(line,sizeof(line),"%s build=EDM368 ",ParentName(sourceG,sourceP));
    if(prefix<0||static_cast<std::size_t>(prefix)>=sizeof(line))return;
    const int n=std::vsnprintf(line+prefix,sizeof(line)-static_cast<std::size_t>(prefix),format,args);
    if(n>=0&&static_cast<std::size_t>(n)<sizeof(line)-static_cast<std::size_t>(prefix))RtPrintf("%s",line);
    else RtPrintf("%s","[EDM341] event=DIAGNOSTIC_FORMAT_FAILURE\n");
}
void PrintParent(int sourceG,const char* format,...) noexcept {
    va_list args;va_start(args,format);PrintParentV(sourceG,50U,format,args);va_end(args);
}
void PrintParent(int sourceG,unsigned sourceP,const char* format,...) noexcept {
    va_list args;va_start(args,format);PrintParentV(sourceG,sourceP,format,args);va_end(args);
}
// EDM366: a passive, cumulative result observation; no gate consumes it.
void PrintOrbitRunSummary(const NCG76DHBatchDiagnostic::Batch& batch,const char* stage) noexcept
{
    if(!NCG76DHBatchDiagnostic::Valid(batch.scope)||!batch.totals.completedParents)return;
    const auto& t=batch.totals;char line[481]{};
    const int n=std::snprintf(line,sizeof(line),"[G76] build=EDM368 event=ORBIT_RUN_SUMMARY run=%llu cache=%llu epoch=%llu stage=%s completed=%llu passed=%llu disabled=%llu timed=%llu unqualified=%llu attempts=%llu failedAttempts=%llu retained=%u dropped=%llu invalid=%u discharge=0\n",
        static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
        static_cast<unsigned long long>(batch.scope.epoch),stage,
        static_cast<unsigned long long>(t.completedParents),static_cast<unsigned long long>(t.inspectionPassed),
        static_cast<unsigned long long>(t.inspectionDisabled),static_cast<unsigned long long>(t.timedCompleted),
        static_cast<unsigned long long>(t.retryExhausted),static_cast<unsigned long long>(t.attempts),
        static_cast<unsigned long long>(t.rejectedFullSweeps),batch.count,
        static_cast<unsigned long long>(batch.dropped),batch.invalid?1U:0U);
    if(n>=0&&static_cast<std::size_t>(n)<sizeof(line))RtPrintf("%s",line);
    else RtPrintf("%s","[EDM366] event=DIAGNOSTIC_FORMAT_FAILURE\n");
}
// EDM366: retain opted-in G76 terminal rows after each parent so a final
// console snapshot includes earlier cases of the same run. Values are copied
// after the original terminal proof; this code cannot grant completion/STOP.
template<class State>
void RecordDHTerminal(State& s,NCG76DHBatchDiagnostic::Result result) noexcept
{
    namespace D=NCG76DHBatchDiagnostic;
    if(s.sourceG!=76||!D::SupportedMode(s.sourceP)||!s.bound||
       (!s.pathSource.has('D')&&!s.pathSource.has('H')&&!s.pathSource.has('L')))return;
    s.dhBatch.Bind({s.run,s.cache,static_cast<std::uint64_t>(s.epoch),s.owner.generation,
        static_cast<std::uint32_t>(s.owner.owner)});
    const auto& f=s.feedback.cycle;
    D::Row row{};
    row.session=s.session;row.dispatch=s.dispatch;row.pc=s.pc;row.line=s.line;
    row.modeP=s.sourceP;
    row.polygonSides=s.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour?
        s.request.geometry.orbit.source.polygonSides:0U;
    row.retreatMode=s.request.retreatMode;row.hasD=s.pathSource.has('D');row.hasH=s.pathSource.has('H');row.hasL=s.pathSource.has('L');
    row.inheritedB=s.prepared.inheritedB;row.endpointCommitted=s.endpointCommitted;row.result=result;
    row.totalLimitUs=f.totalLimitUs;row.additionalUs=f.additionalUs;
    row.totalElapsedUs=f.totalElapsedUs;row.additionalElapsedUs=f.additionalElapsedUs;row.flushes=f.flushCount;
    row.orbitRequiredPasses=f.orbitRequiredPasses;row.orbitQualifiedPasses=f.orbitQualifiedPasses;
    row.orbitAttemptCount=f.orbitAttemptCount;row.orbitPercentScaled=f.orbitPercentScaled;
    row.orbitTargetPercentScaled=f.orbitTargetPercentScaled;row.orbitCompletionState=f.orbitCompletionState;
    row.orbitActive=f.orbitCompletionActive;row.orbitPercentAvailable=f.orbitPercentAvailable;
    row.orbitEvaluation=f.orbitEvaluation;row.orbitRetryLimit=f.orbitRetryLimit;
    if(s.pathSource.has('N')){
        const double n=s.pathSource.val('N');
        if(std::isfinite(n)&&n>=0.0&&n<=2147483647.0&&std::floor(n)==n)row.sequenceN=static_cast<int>(n);
    }
    (void)s.dhBatch.Append(row);
    const auto& batch=s.dhBatch;
    PrintParent(s.sourceG,s.sourceP,"event=DH_BATCH_SUMMARY run=%llu cache=%llu rows=%u capacity=%u dropped=%llu invalid=%u trigger=%s motionAuthority=UNCHANGED discharge=0\n",
        static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
        batch.count,D::Capacity,static_cast<unsigned long long>(batch.dropped),batch.invalid?1U:0U,
        result==D::Result::Complete?"COMPLETE":result==D::Result::TimedComplete?"D_COMPLETE":result==D::Result::RetryExhausted?"RETRY_EXHAUSTED":"TIME_LIMIT");
    for(unsigned i=0U;i<batch.count&&i<D::Capacity;++i){
        const auto& r=batch.rows[i];
        PrintParent(76,r.modeP,"event=DH_BATCH_ROW run=%llu cache=%llu index=%u N=%d session=%llu dispatch=%llu pc=%d P=%u Q=%u B=%u inheritedB=%u hasD=%u hasH=%u hasL=%u result=%s endpointCommitted=%u flushes=%llu\n",
            static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
            i+1U,r.sequenceN,static_cast<unsigned long long>(r.session),static_cast<unsigned long long>(r.dispatch),
            r.pc,r.modeP,r.polygonSides,r.retreatMode,r.inheritedB?1U:0U,r.hasD?1U:0U,r.hasH?1U:0U,r.hasL?1U:0U,
            r.result==D::Result::Complete?"COMPLETE":r.result==D::Result::TimedComplete?"D_COMPLETE":r.result==D::Result::RetryExhausted?"RETRY_EXHAUSTED":"TIME_LIMIT",r.endpointCommitted?1U:0U,
            static_cast<unsigned long long>(r.flushes));
        PrintParent(76,r.modeP,"event=DH_BATCH_TIME run=%llu cache=%llu index=%u session=%llu totalLimitUs=%llu totalElapsedUs=%llu additionalUs=%llu additionalElapsedUs=%llu\n",
            static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
            i+1U,static_cast<unsigned long long>(r.session),static_cast<unsigned long long>(r.totalLimitUs),
            static_cast<unsigned long long>(r.totalElapsedUs),static_cast<unsigned long long>(r.additionalUs),
            static_cast<unsigned long long>(r.additionalElapsedUs));
        PrintParent(76,r.modeP,"event=DH_BATCH_ORBIT run=%llu cache=%llu index=%u session=%llu state=%u requiredPasses=%u qualifiedPasses=%u attempts=%llu available=%u percent=%.6f targetPercent=%.6f\n",
            static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
            i+1U,static_cast<unsigned long long>(r.session),r.orbitCompletionState,r.orbitRequiredPasses,
            r.orbitQualifiedPasses,static_cast<unsigned long long>(r.orbitAttemptCount),r.orbitPercentAvailable?1U:0U,
            r.orbitPercentScaled/1000000.0,r.orbitTargetPercentScaled/1000000.0);
        const auto& e=r.orbitEvaluation;
        PrintParent(76,r.modeP,"event=DH_BATCH_EVAL run=%llu cache=%llu index=%u session=%llu attempts=%llu failedAttempts=%llu lastAvailable=%u lastPercent=%.6f targetPercent=%.6f lastAccepted=%u streakBefore=%u qualified=%u peak=%u retryLimit=%u\n",
            static_cast<unsigned long long>(batch.scope.run),static_cast<unsigned long long>(batch.scope.cache),
            i+1U,static_cast<unsigned long long>(r.session),static_cast<unsigned long long>(r.orbitAttemptCount),
            static_cast<unsigned long long>(e.failedAttempts),e.lastAvailable?1U:0U,
            e.lastPercentScaled/1000000.0,r.orbitTargetPercentScaled/1000000.0,e.lastAccepted?1U:0U,
            e.lastQualifiedBefore,r.orbitQualifiedPasses,e.peakQualified,static_cast<unsigned>(r.orbitRetryLimit));
    }
    PrintOrbitRunSummary(batch,"PARENT_TERMINAL");
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
void PrintSphereApproachRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76,unsigned sourceP=50U) noexcept {
    PrintParent(sourceG,sourceP,"event=APPROACH_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::ApproachReasonName(binding.approachReason),
        EDM58::BindingErrorName(binding.approachError),binding.approachFailedFieldId);
}
void PrintSphereInitialSlowRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76,unsigned sourceP=50U) noexcept {
    PrintParent(sourceG,sourceP,"event=INITIAL_SLOW_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::ApproachReasonName(binding.initialSlowReason),
        EDM58::BindingErrorName(binding.initialSlowError),binding.initialSlowFailedFieldId);
}
void PrintSphereDeepFlushRejection(const NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG=76,unsigned sourceP=50U) noexcept {
    PrintParent(sourceG,sourceP,"event=DEEP_FLUSH_BINDING_REJECT B=%u reason=%s error=%s field=%u typedInvalid=1 discharge=0\n",
        mode,NCG76SphereGapBinding::DeepFlushReasonName(binding.deepFlushReason),
        EDM58::BindingErrorName(binding.deepFlushError),binding.deepFlushFailedFieldId);
}
const char* RejectTypedBinding(NCG76SphereGapBinding::Binding& binding,unsigned mode,int sourceG,unsigned sourceP=50U) noexcept
{
    if(binding.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereApproachRejection(binding,mode,sourceG,sourceP);
        return "TYPED_APPROACH_FIELD_INVALID";
    }
    if(binding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereInitialSlowRejection(binding,mode,sourceG,sourceP);
        return "TYPED_INITIAL_SLOW_FIELD_INVALID";
    }
    if(binding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        binding.config=NCG76SphereGapProcess::Config{};PrintSphereDeepFlushRejection(binding,mode,sourceG,sourceP);
        return "TYPED_DEEP_FLUSH_FIELD_INVALID";
    }
    return nullptr;
}
const char* PhaseName(NCG76SphereCycle::Phase phase,int sourceG=76) noexcept {
    using P=NCG76SphereCycle::Phase;
    switch(phase){
    case P::ContourFinish:return "CONTOUR_FINISH";
    case P::ContourNominal:return "CONTOUR_NOMINAL";case P::Idle:return "IDLE";case P::InitialDepth:return "INITIAL_DEPTH";
    case P::InitialRetreat:return "INITIAL_RETREAT";case P::CircleEntry:return "CIRCLE_ENTRY";
    case P::Orbit:return "ORBIT";case P::ReturnCenter:return "RETURN_CENTER";
    case P::AdvanceCenter:return "ADVANCE_CENTER";case P::Settle:return "SETTLE";
    case P::HoldBrake:return "HOLD_BRAKE";case P::HoldSettle:return "HOLD_SETTLE";
    case P::HoldToCenter:return "HOLD_TO_CENTER";case P::HoldToStart:return "HOLD_TO_START";
    case P::HeldAtStart:return sourceG==76?"HELD_AT_START":"HELD_IN_PLACE";case P::ResumeToCenter:return "RESUME_TO_CENTER";
    case P::ResumeToAnchor:return "RESUME_TO_ANCHOR";case P::FaultBrake:return "FAULT_BRAKE";
    case P::FaultSettle:return "FAULT_SETTLE";case P::Complete:return "COMPLETE";case P::Fault:return "FAULT";
    case P::ResumeBrake:return "RESUME_BRAKE";
    case P::TimeLimitBrake:return "TIME_LIMIT_BRAKE";case P::TimeLimitSettle:return "TIME_LIMIT_SETTLE";
    case P::TimedHistoryReturn:return "D_HISTORY_RETURN";
    case P::TimedReturnCenter:return "D_RETURN_CENTER";
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
    case 8U:return "START_TRAVEL";case 9U:return "FLUSH_PROCESS_SCOPE";case 10U:return "MACHINING_TIME_LIMIT";default:return "UNKNOWN";}
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
    case F::TimeLimit:return "MACHINING_TIME_LIMIT";
    default:return "UNKNOWN";}
}
bool FailedSphereFeedback(const MotionG76SphereFeedback& report) noexcept {
    return report.runtimeFault||report.cycle.fault!=NCG76SphereCycle::Fault::None||
        (report.terminal&&!report.completed);
}
// Diagnostic-only readback remains useful after the RT failure revokes its
// owner/epoch. It never grants motion or releases the NC completion barrier.
void PrintSphereRuntimeFailure(const MotionG76SphereFeedback& report,
    const NCG76SphereCycle::Request& request,int line,int sourceG=76,unsigned sourceP=50U) noexcept {
    PrintParent(sourceG,sourceP,"event=FAULT session=%llu line=%d tick=%llu phase=%s nominal=%s cycleFault=%u cause=%s runtimeFault=%u gate=%s started=%u discharge=0\n",
        static_cast<unsigned long long>(report.session),line,static_cast<unsigned long long>(report.inputSequence),
        PhaseName(report.cycle.phase,sourceG),PhaseName(report.cycle.nominalPhase,sourceG),static_cast<unsigned>(report.cycle.fault),
        CycleFaultName(report.cycle.fault),report.runtimeFault,
        RuntimeFaultName(report.runtimeFault),report.started?1U:0U);
    const auto& clock=report.cycle.clockFault;
    if(report.cycle.fault==NCG76SphereCycle::Fault::Clock&&clock.valid){
        // RT freezes the first violating sample before fault containment.
        // Format it here on the NC owner; later cancellation cannot replace
        // the original clock evidence with a new sample or owner rejection.
        PrintParent(sourceG,sourceP,"event=CLOCK_FAULT session=%llu reasonBits=0x%02X previousTick=%llu currentTick=%llu previousPhysical=%llu currentPhysical=%llu discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned>(clock.reasonBits),
            static_cast<unsigned long long>(clock.previousTick),static_cast<unsigned long long>(clock.currentTick),
            static_cast<unsigned long long>(clock.previousPhysicalSequence),static_cast<unsigned long long>(clock.currentPhysicalSequence));
        PrintParent(sourceG,sourceP,"event=CLOCK_INTERVAL session=%llu previousNowUs=%llu currentNowUs=%llu deltaValid=%u deltaUs=%llu minimumUs=%llu maximumUs=%llu discharge=0\n",
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
        PrintParent(sourceG,sourceP,"event=INPUT_FAULT session=%llu tick=%llu stage=%s gate=%s axis=%d flags=0x%08X validAxisMask=0x%02X statusWord=0x%04X mode=%d rawPosition=%d discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(input.tick),
            MotionG76SphereInputStageName(input.stage),MotionG76SphereInputGateName(input.gate),
            static_cast<int>(input.axis),static_cast<unsigned>(input.flags),static_cast<unsigned>(input.validAxisMask),
            static_cast<unsigned>(input.statusWord),static_cast<int>(input.mode),static_cast<int>(input.rawPosition));
        PrintParent(sourceG,sourceP,"event=INPUT_FAULT_CLOCK session=%llu inputTick=%llu physicalSequence=%llu nowUs=%llu clockFrequency=%llu discharge=0\n",
            static_cast<unsigned long long>(report.session),static_cast<unsigned long long>(input.inputTick),
            static_cast<unsigned long long>(input.physicalSequence),static_cast<unsigned long long>(input.nowUs),
            static_cast<unsigned long long>(input.clockFrequency));
        PrintParent(sourceG,sourceP,"event=INPUT_FAULT_SCOPE session=%llu inputSession=%llu expectedSession=%llu inputEpoch=%llu expectedEpoch=%llu inputSegment=%llu expectedSegment=%llu inputSource=%u expectedSource=%u inputBlock=%d expectedBlock=%d discharge=0\n",
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
            PrintParent(sourceG,sourceP,"event=FAULT_AXIS session=%llu axis=%u actual=UNAVAILABLE target=%.12f error=UNAVAILABLE tolerance=%.12f unit=%s inWindow=UNKNOWN sampleValid=0 fallbackActual=%.12g discharge=0\n",
                static_cast<unsigned long long>(report.session),axis,target[axis],tolerance,
                axis<3U?"mm":"deg",report.actualMCS[axis]);
            continue;
        }
        const double error=report.actualMCS[axis]-target[axis];
        PrintParent(sourceG,sourceP,"event=FAULT_AXIS session=%llu axis=%u actual=%.12f target=%.12f error=%.12f tolerance=%.12f unit=%s inWindow=%u sampleValid=1 discharge=0\n",
            static_cast<unsigned long long>(report.session),axis,report.actualMCS[axis],target[axis],
            error,tolerance,axis<3U?"mm":"deg",std::isfinite(tolerance)&&tolerance>0.0&&
                std::isfinite(error)&&std::fabs(error)<=tolerance?1U:0U);
    }
}
}

bool NCManager::IsG76SphereBlock(const NCBlock& block) noexcept
{
    // EDM358: authored D/H/L selects the native contour parent for P01-P27.
    // The complete source/shape proof follows before any publication.
    return (NCGCodeSemantics::Contains(block,76)&&block.has('P')&&block.val('P')==50.0)||
        NCG76TimedP11Source::IsOptIn(block);
}

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

void NCManager::ClearMachiningTimeTelemetrySameThread() noexcept
{
    if(m_g76Sphere)m_g76Sphere->timeTelemetryCleared=true;
}

void NCManager::ExportMachiningTelemetrySameThread(NCMachiningTelemetry::Image& image) const noexcept
{
    namespace T=NCMachiningTelemetry;
    image=T::Image{};image.phase=T::Phase::Idle;
    image.capabilities=T::TotalLimit|T::AdditionalMachining|T::HoldPause|T::P50FullSweep;
    if(!m_g76Sphere||m_g76Sphere->timeTelemetryCleared)return;
    const auto& s=*m_g76Sphere;
    if(!s.bound||!s.session)return;
    if(s.sourceG==76&&NCG76Mode::IsFixedRadius(s.sourceP))
        image.capabilities=T::TotalLimit|T::AdditionalMachining|T::HoldPause;
    MotionG76SphereFeedback report{};
    if(!m_motion.ReadG76SphereFeedback(report)||report.session!=s.session||
       !SameIdentity(report.identity,s.identity)||!report.owner.Matches(s.owner)||
       report.cycle.session!=s.session||!report.cycle.tick){
        if(s.active)image.phase=T::Phase::Unavailable;
        return;
    }
    const auto& f=report.cycle;
    const bool timed=MotionG76SphereMachiningTime::TimedCompletionQualified(f,s.request);
    const bool exhausted=MotionG76SphereMachiningTime::RetryExhaustionQualified(f,s.request);
    const bool timeLimit=f.fault==NCG76SphereCycle::Fault::TimeLimit||
        (f.totalLimitUs&&f.totalTimerState==static_cast<std::uint8_t>(NCMachiningTime::TimerState::Expired));
    const bool fault=report.runtimeFault||f.fault!=NCG76SphereCycle::Fault::None;
    const bool completed=report.terminal&&report.completed&&!fault;
    // A completed prior row must not present its budget as the next row's
    // clock. Keep only the final program result or an explicit alarm receipt.
    if(!s.active&&!timeLimit&&!(completed&&GetState()==NCState::P_END)&&
       !(fault&&GetState()==NCState::ALARM))return;
    if(f.totalLimitUs!=s.request.totalLimitUs||f.additionalUs!=s.request.additionalUs||
       f.totalElapsedUs>f.totalLimitUs||f.additionalElapsedUs>f.additionalUs){
        image.phase=T::Phase::Unavailable;return;
    }
    const bool straightSideContourSource=s.sourceG==76&&NCG76OrbitContour::IsStraightSideContour(s.request.geometry);
    const bool circleContourSource=s.sourceG==76&&NCG76OrbitContour::IsCircleContour(s.request.geometry);
    if(s.sourceG==76&&(!s.pathSource.has('P')||s.pathSource.val('P')!=static_cast<double>(s.sourceP)||
       ((circleContourSource||straightSideContourSource)?(s.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||
           NCG76OrbitContour::ModeP(s.request.geometry)!=s.sourceP||
           (NCG76Mode::IsPolygon(s.sourceP)?(!s.pathSource.has('Q')||s.pathSource.val('Q')!=
                static_cast<double>(s.request.geometry.orbit.source.polygonSides)):
                s.request.geometry.orbit.source.polygonSides!=0U)||
           report.nominalKind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||f.completedLayers!=0U||
           f.layerIndex>=NCG76OrbitContour::NominalLegCountFor(s.request.geometry)||
           (straightSideContourSource&&f.nominalPhase==NCG76SphereCycle::Phase::ContourFinish?
                f.layerIndex>=NCG76OrbitContour::FinishLegCount(s.request.geometry):
                (f.nominalPhase!=NCG76SphereCycle::Phase::ContourNominal&&f.layerIndex!=0U))||
           (circleContourSource&&f.nominalPhase==NCG76SphereCycle::Phase::ContourFinish)||
           (straightSideContourSource&&f.nominalPhase!=NCG76SphereCycle::Phase::ContourNominal&&
                f.nominalPhase!=NCG76SphereCycle::Phase::ContourFinish&&
                f.nominalPhase!=NCG76SphereCycle::Phase::CircleEntry&&
                f.nominalPhase!=NCG76SphereCycle::Phase::ReturnCenter&&
                f.nominalPhase!=NCG76SphereCycle::Phase::Idle)):
        s.sourceP==50U?(s.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere||
           report.nominalKind!=NCG76SphereRuntimeGeometry::Kind::Sphere):true))){
        image.phase=T::Phase::Unavailable;return;
    }
    image.parentSession=s.session;image.sampleTick=f.tick;
    image.currentSourceG=static_cast<std::uint32_t>(s.sourceG);
    image.currentSourceP=s.sourceG==76?s.sourceP:0U;
    image.currentSourceLine=s.line>0?static_cast<std::uint32_t>(s.line):0U;
    if(s.pathSource.has('N')){
        const double n=s.pathSource.val('N');
        if(std::isfinite(n)&&n>=0.0&&n<=2147483647.0&&std::floor(n)==n)
            image.currentSequenceN=static_cast<std::int32_t>(n);
    }
    image.parentValid=image.runtimeSupported=true;image.active=s.active&&!report.terminal;
    if(s.sourceG==76&&s.sourceP==50U){image.layerIndex=f.layerIndex;image.layerCount=s.request.geometry.circleLayerCount;}
    const auto pause=[&](std::uint8_t value) noexcept {
        using P=NCMachiningTime::PauseReason;
        switch(static_cast<P>(value)){
        case P::None:return T::PauseReason::None;
        case P::Hold:return f.inspectionJogActive?T::PauseReason::InspectionJog:T::PauseReason::Held;
        case P::Flush:return T::PauseReason::Flush;
        case P::Gap:return T::PauseReason::GapRetreat;
        case P::Returning:return T::PauseReason::Returning;
        case P::Fault:return T::PauseReason::Terminated;
        case P::Qualification:return T::PauseReason::EndpointQualification;
        case P::Waiting:return T::PauseReason::NotStarted;
        default:return T::PauseReason::Terminated;
        }
    };
    if(s.request.orbitInspectionRounds){
        if(s.sourceG!=76||f.orbitRequiredPasses!=s.request.orbitInspectionRounds||
           f.orbitTargetPercentScaled!=s.request.orbitThresholdScaled||
           f.orbitRetryLimit!=s.request.orbitRetryLimit){image=T::Image{};image.phase=T::Phase::Unavailable;return;}
        image.capabilities|=T::OrbitCompletion;
        image.orbitCompletionState=f.orbitCompletionState;
        image.orbitCompletionFlags=T::OrbitEnabledFlag|T::OrbitSimulationOnlyFlag|
            (f.orbitCompletionActive?T::OrbitActiveFlag:0U)|(f.orbitPercentAvailable?T::OrbitRatioAvailableFlag:0U);
        image.orbitPercentScaled=f.orbitPercentScaled;image.orbitTargetPercentScaled=f.orbitTargetPercentScaled;
        image.orbitQualifiedPasses=f.orbitQualifiedPasses;image.orbitRequiredPasses=f.orbitRequiredPasses;
        image.orbitAttemptCount=f.orbitAttemptCount;
        image.orbitRetryLimit=f.orbitRetryLimit;
        image.orbitEvaluationFlags=(f.orbitEvaluation.lastAvailable?1U:0U)|(f.orbitEvaluation.lastAccepted?2U:0U);
        image.orbitFailedAttemptCount=f.orbitEvaluation.failedAttempts;
        image.orbitLastPercentScaled=f.orbitEvaluation.lastPercentScaled;
        image.orbitLastQualifiedBefore=f.orbitEvaluation.lastQualifiedBefore;
        image.orbitPeakQualifiedPasses=f.orbitEvaluation.peakQualified;
        if(f.orbitRetryLimit)image.capabilities|=T::OrbitRetryLimit;
    }
    image.total.configuredUs=f.totalLimitUs;image.total.elapsedUs=f.totalElapsedUs;
    image.total.remainingUs=f.totalLimitUs-f.totalElapsedUs;
    image.total.state=static_cast<T::TimerState>(f.totalTimerState);image.total.pauseReason=pause(f.totalPauseReason);
    image.additional.configuredUs=f.additionalUs;image.additional.elapsedUs=f.additionalElapsedUs;
    image.additional.remainingUs=f.additionalUs-f.additionalElapsedUs;
    image.additional.state=static_cast<T::TimerState>(f.additionalTimerState);image.additional.pauseReason=pause(f.additionalPauseReason);
    if(exhausted){image.phase=T::Phase::Complete;image.endReason=T::EndReason::ProcessUnqualified;}
    else if(timed){image.phase=T::Phase::Complete;image.endReason=T::EndReason::TimedCompleted;}
    else if(timeLimit){image.phase=T::Phase::TimeLimit;image.endReason=T::EndReason::TimeLimit;}
    else if(fault){image.phase=T::Phase::Fault;image.endReason=f.fault==NCG76SphereCycle::Fault::Cancelled?T::EndReason::Cancelled:T::EndReason::Fault;}
    else if(completed){image.phase=T::Phase::Complete;image.endReason=T::EndReason::Completed;}
    else if(f.holdActive){
        image.phase=f.holdRecoveryPhase>=6U&&f.holdRecoveryPhase<=8U?T::Phase::Returning:T::Phase::Held;
    }else if(f.timeLimitReached){image.phase=T::Phase::TimedFinishing;
    }else if(f.orbitCompletionActive){image.phase=T::Phase::FinishVerification;
    }else{
        using P=NCMachiningTime::TimePhase;
        switch(static_cast<P>(f.timePhase)){
        case P::Additional:image.phase=T::Phase::AdditionalMachining;break;
        case P::Finishing:image.phase=T::Phase::AdditionalFinishing;break;
        case P::Verification:image.phase=T::Phase::FinishVerification;break;
        default:image.phase=T::Phase::Machining;break;
        }
    }
    const auto& batch=s.dhBatch;const auto& scope=batch.scope;
    if(!batch.invalid&&NCG76DHBatchDiagnostic::Valid(scope)&&scope.run==s.run&&scope.cache==s.cache&&
       scope.epoch==static_cast<std::uint64_t>(s.epoch)&&scope.owner==static_cast<std::uint32_t>(s.owner.owner)&&
       scope.ownerGeneration==s.owner.generation&&batch.totals.completedParents){
        image.capabilities|=T::OrbitRunStatistics;
        image.orbitRunCompletedParents=batch.totals.completedParents;
        image.orbitRunInspectionPassed=batch.totals.inspectionPassed;
        image.orbitRunInspectionDisabled=batch.totals.inspectionDisabled;
        image.orbitRunTimedCompleted=batch.totals.timedCompleted;
        image.orbitRunRetryExhausted=batch.totals.retryExhausted;
    }
    if(!T::Valid(image)){
        image=T::Image{};image.phase=T::Phase::Unavailable;
        image.capabilities=T::TotalLimit|T::AdditionalMachining|T::HoldPause|T::P50FullSweep;
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

bool NCManager::IsCompletedEDMProcessPathTerminalAllowedSameThread(const NCBlock& block,int expectedM)
{
    const auto& s=*m_g76Sphere;const int pc=GetActiveDispatchPC();
    const auto* row=GetBaseProgramCache().TryGetLine(pc);NCBlock literal{};
    // M00 is a nonpositioning checkpoint after this exact completed parent.
    // Timed contours also require their source-bound terminal proof for M30.
    const bool analytic=s.sourceG==1||s.sourceG==2||s.sourceG==3;
    const bool timed=MotionG76SphereMachiningTime::TimedCompletionQualified(s.feedback.cycle,s.request);
    const bool sphereCheckpoint=(expectedM==0||s.request.orbitInspectionRounds!=0U||timed)&&s.sourceG==76&&s.pathSource.has('P')&&
        s.pathSource.val('P')==50.0&&s.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere;
    const bool timedContour=s.sourceG==76&&NCG76TimedP11Source::SupportedMode(static_cast<double>(s.sourceP))&&
        s.pathSource.has('P')&&s.pathSource.val('P')==static_cast<double>(s.sourceP)&&
        NCG76OrbitContour::ModeP(s.request.geometry)==s.sourceP&&NCG76TimedP11Source::IsOptIn(s.pathSource)&&
        s.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
        (s.request.retreatMode==0U||s.request.retreatMode==2U);
    if((expectedM!=0&&expectedM!=30)||(!analytic&&!sphereCheckpoint&&!timedContour)||!s.completed||!s.bound||!s.endpointCommitted||
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
        literal.hasG||literal.gCount||literal.mCount!=1||literal.mCode[0]!=expectedM||literal.isGoto||
        !ExactProcessG(row->parsedBlock.mExpressions[0],expectedM))return false;
    if((expectedM==0||timedContour||sphereCheckpoint)&&(!s.request.geometry.valid||!s.feedback.accepted||!s.feedback.started||
        s.identity.source!=MotionCommandSource::NC_MEMORY||s.identity.epoch!=s.epoch||
        s.identity.sourceBlockId!=static_cast<MotionSourceBlockId>(s.pc)||s.feedback.session!=s.session||
        s.feedback.cycle.session!=s.session||!s.feedback.cycle.tick||s.feedback.cycle.holdActive||s.feedback.cycle.held||
        !SameIdentity(s.feedback.identity,s.identity)||!s.feedback.owner.Matches(s.owner)||
        s.feedback.nominalKind!=s.request.geometry.kind||
        (s.sourceG==1&&s.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line)||
        ((s.sourceG==2||s.sourceG==3)&&s.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
        (sphereCheckpoint&&(!s.request.geometry.circleLayerCount||
            (!timed&&s.feedback.cycle.completedLayers!=s.request.geometry.circleLayerCount)))||
        !OrbitTerminalQualified(s.feedback.cycle,s.request)||
        !FinishTerminalQualified(s.feedback.cycle,s.finishRequiredCount)||
        !MotionG76SphereMachiningTime::TerminalQualified(s.feedback.cycle,s.request)))return false;
    if(s.request.orbitInspectionRounds){
        const auto* parent=GetBaseProgramCache().TryGetLine(s.pc);std::uint32_t rounds=0U;
        if(!parent||parent->sourceLineNumber!=s.line||!SameLiteral(s.orbitCompletionSource,s.pathSource)||
           !NCOrbitCompletionPolicy::Decode(parent->parsedBlock,s.pathSource,rounds)||rounds!=s.request.orbitInspectionRounds||
           !s.orbitCompletionBinding.ready||s.orbitCompletionBinding.value!=s.request.orbitThresholdScaled||
           s.orbitCompletionBinding.retryLimit!=s.request.orbitRetryLimit)return false;
    }
    if(timedContour){
        const auto* parent=GetBaseProgramCache().TryGetLine(s.pc);
        NCG76TimedP11Source::Words words{};NCMachiningTime::Config time{};
        if(!parent||parent->sourceLineNumber!=s.line||s.timeTelemetryCleared||
           !SameNCTranslationSnapshot(s.frame,s.request.geometry.frame.translation)||
           !NCG76SphereRuntimeGeometry::Valid(s.request.geometry)||
           NCG76TimedP11Source::Decode(parent->parsedBlock,s.pathSource,s.prepared.inheritedBSource,words)!=
                NCG76TimedP11Source::Status::Ok||words.inspectionRounds!=s.request.orbitInspectionRounds||words.requestedB!=s.request.retreatMode||
           s.prepared.requestedB!=s.request.retreatMode||
           !NCMachiningTimePolicy::Decode(parent->parsedBlock,s.pathSource,time)||
           !NCMachiningTime::SameConfig(time,s.machiningTime)||
           !MotionG76SphereMachiningTime::Matches(time,s.request)||
           !TimedP11SourceMatchesGeometry(words,time,s.request.geometry)||
           s.feedback.cycle.completedLayers!=0U||
           (!timed&&(s.feedback.cycle.layerIndex!=((time.additionalUs||s.request.orbitInspectionRounds)?0U:NCG76OrbitContour::NominalLegCountFor(s.request.geometry)-1U)||
           s.feedback.cycle.nominalPhase!=((time.additionalUs||s.request.orbitInspectionRounds)?NCG76SphereCycle::Phase::ReturnCenter:
                NCG76SphereCycle::Phase::ContourNominal)))||
           words.explicitB!=s.prepared.explicitB||words.inheritedB!=s.prepared.inheritedB||
           (words.inheritedB&&(words.inheritedBSource.committed.run!=s.run||
            words.inheritedBSource.committed.cache!=s.cache||
            !words.inheritedBSource.committed.lease.Matches(s.owner)||
            words.inheritedBSource.commit>=s.commit)))return false;
    }
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
    if(!MotionG76SphereMachiningTime::CompletedEndpointQualified(s.feedback,s.request))return false;
    for(unsigned a=0U;a<8U;++a)if(!SameDouble(CoordSys.commandedMCS[a],s.endpoint[a])||
        !SameDouble(s.endpoint[a],s.feedback.endpointMCS[a]))return false;
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
    MotionG76SphereFeedback stopped{};
    const bool deadlineAlreadyStopped=submitted&&m_g76Sphere->bound&&
        alarm==AlarmManager::EDM_MACHINING_TIME_LIMIT&&m_motion.ReadG76SphereFeedback(stopped)&&
        SameIdentity(stopped.identity,m_g76Sphere->identity)&&stopped.owner.Matches(m_g76Sphere->owner)&&
        stopped.translationGeneration==m_g76Sphere->request.geometry.frame.translation.generation&&
        MotionG76SphereMachiningTime::TimeLimitStoppedQualified(stopped,m_g76Sphere->request);
    if(submitted&&!deadlineAlreadyStopped)m_motion.RequestEmergencyStopAllAxes();
    PrintParent(m_g76Sphere->sourceG,m_g76Sphere->sourceP,"event=REJECT session=%llu line=%d reason=%s discharge=0\n",
        static_cast<unsigned long long>(session),line,reason?reason:"UNSPECIFIED");
}

bool NCManager::ValidateG76SphereBlockSameThread(const NCBlock& block,int pc,int line)
{
    const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(NCG76TimedP11Source::IsOptIn(block)){
        NCG76TimedP11Source::Words words{};
        const auto status=source?NCG76TimedP11Source::Decode(source->parsedBlock,block,
            CaptureG76SphereModalBSameThread(),words):NCG76TimedP11Source::Status::InvalidSource;
        if(!source||(line&&source->sourceLineNumber!=line)||status!=NCG76TimedP11Source::Status::Ok||
           !SameLiteral(block,words.authored)||!m_macroStack.empty()||m_isG66Active||
           CoordSys.isInchMode||CoordSys.activePlane!=17){
            RejectG76SphereSameThread(status==NCG76TimedP11Source::Status::UnresolvedModalB?
                (NCMachiningTimePolicy::FixedRadiusMode(block.val('P'))?"P01_P07_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==27.0?"P27_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==26.0?"P26_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==25.0?"P25_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==24.0?"P24_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==23.0?"P23_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==22.0?"P22_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==21.0?"P21_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==17.0?"P17_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==16.0?"P16_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==15.0?"P15_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==14.0?"P14_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==13.0?"P13_B_REQUIRES_COMMITTED_SOURCE":block.val('P')==12.0?"P12_B_REQUIRES_COMMITTED_SOURCE":"P11_B_REQUIRES_COMMITTED_SOURCE"):
                (NCMachiningTimePolicy::FixedRadiusMode(block.val('P'))?"P01_P07_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==27.0?"P27_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==26.0?"P26_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==25.0?"P25_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==24.0?"P24_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==23.0?"P23_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==22.0?"P22_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==21.0?"P21_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==17.0?"P17_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==16.0?"P16_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==15.0?"P15_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==14.0?"P14_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==13.0?"P13_EXACT_LITERAL_SOURCE_G17_G21":block.val('P')==12.0?"P12_EXACT_LITERAL_SOURCE_G17_G21":"P11_EXACT_LITERAL_SOURCE_G17_G21"),
                AlarmManager::G_Code_Invalid_parameter,source?source->sourceLineNumber:line);return false;
        }
        if(m_g76Sphere->active){RejectG76SphereSameThread(NCMachiningTimePolicy::FixedRadiusMode(block.val('P'))?"P01_P07_PARENT_STILL_ACTIVE":block.val('P')==27.0?"P27_PARENT_STILL_ACTIVE":block.val('P')==26.0?"P26_PARENT_STILL_ACTIVE":block.val('P')==25.0?"P25_PARENT_STILL_ACTIVE":block.val('P')==24.0?"P24_PARENT_STILL_ACTIVE":block.val('P')==23.0?"P23_PARENT_STILL_ACTIVE":block.val('P')==22.0?"P22_PARENT_STILL_ACTIVE":block.val('P')==21.0?"P21_PARENT_STILL_ACTIVE":block.val('P')==17.0?"P17_PARENT_STILL_ACTIVE":block.val('P')==16.0?"P16_PARENT_STILL_ACTIVE":block.val('P')==15.0?"P15_PARENT_STILL_ACTIVE":block.val('P')==14.0?"P14_PARENT_STILL_ACTIVE":block.val('P')==13.0?"P13_PARENT_STILL_ACTIVE":block.val('P')==12.0?"P12_PARENT_STILL_ACTIVE":"P11_PARENT_STILL_ACTIVE",
            AlarmManager::PATH_EXECUTION_NOT_READY,source->sourceLineNumber);return false;}
        return true;
    }
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
    const bool timedContour=NCG76TimedP11Source::IsOptIn(block);
    if(!processPath&&!timedContour)PrintSphereMemoryStage("NC_PREPARE_ENTER",pc);
    if(processPath?!ValidateEDMProcessPathBlockSameThread(block,pc,source?source->sourceLineNumber:0):
        !ValidateG76SphereBlockSameThread(block,pc,source?source->sourceLineNumber:0))return false;
    // Allocate NC-only preparation storage once per parent; no large temporary
    // session is constructed on the NC stack or on any servo callback.
    std::unique_ptr<G76SphereNCState> next(new G76SphereNCState{});
    if(m_g76Sphere)next->dhBatch=m_g76Sphere->dhBatch;
    m_g76Sphere=std::move(next);auto& s=*m_g76Sphere;
    s.pc=pc;s.line=source->sourceLineNumber;s.dispatch=m_currentExecutingBlockDispatchId;
    s.sourceG=processPath?block.gCode:76;
    s.sourceP=processPath?0U:timedContour?static_cast<unsigned>(block.val('P')):50U;s.pathSource=block;
    const auto reject=[this](const char* reason,int alarm){RejectG76SphereSameThread(reason,alarm);return false;};
    if(!NCMachiningTimePolicy::Decode(source->parsedBlock,block,s.machiningTime))
        return reject("MACHINING_TIME_LITERAL",AlarmManager::G_Code_Invalid_parameter);
    s.request.totalLimitUs=s.machiningTime.totalLimitUs;s.request.additionalUs=s.machiningTime.additionalUs;
    if(!processPath){
        std::uint32_t rounds=0U;
        if(!NCOrbitCompletionPolicy::Decode(source->parsedBlock,block,rounds))
            return reject("ORBIT_COMPLETION_LITERAL",AlarmManager::G_Code_Invalid_parameter);
        s.orbitCompletionSource=block;
        s.orbitCompletionBinding=NCOrbitCompletionBinding::Capture(m_recipe,rounds);
        if(!s.orbitCompletionBinding.ready)return reject("ORBIT_COMPLETION_RATIO_REQUIRES_CONFIRMED_E_FIELD",AlarmManager::G_Code_Invalid_parameter);
        s.request.orbitInspectionRounds=static_cast<std::uint8_t>(rounds);
        s.request.orbitThresholdScaled=rounds?static_cast<std::uint32_t>(s.orbitCompletionBinding.value):0U;
        s.request.orbitRetryLimit=rounds?s.orbitCompletionBinding.retryLimit:0U;
    }

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
    s.dhBatch.Bind({s.run,s.cache,static_cast<std::uint64_t>(s.epoch),s.owner.generation,
        static_cast<std::uint32_t>(s.owner.owner)});
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
        if(const char* typed=RejectTypedBinding(s.gapBinding,mode,s.sourceG,s.sourceP))return reject(typed,AlarmManager::G_Code_Invalid_parameter);
        if(!s.gapBinding.ready||!s.gapBinding.config.enabled||!s.gapBinding.config.signedMotionEnabled||
            !NCG76SphereGapProcess::ResolveParameterMotionCeiling(s.gapBinding.config,s.processMotionCeiling)){
            PrintParent(s.sourceG,s.sourceP,"event=PARAMETER_BINDING_REJECT reason=%s error=%s field=%u discharge=0\n",
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
    }else if(timedContour){
        NCG76TimedP11Source::Words words{};
        NCG76SphereCoordinates::FrozenFrame pathFrame{};
        if(NCG76TimedP11Source::Decode(source->parsedBlock,block,CaptureG76SphereModalBSameThread(),words)!=
                NCG76TimedP11Source::Status::Ok||
           NCG76SphereCoordinates::BuildFrame(s.frame,start,pathFrame)!=NCG76SphereCoordinates::Status::Ok||
           !CaptureCncFeedValueSameThread(block,s.feed)||!s.feed.programmed||
           !SameDouble(s.feed.feedMMMin,words.feedMmMin))
            return reject(NCG76Mode::IsFixedRadius(s.sourceP)?"P01_P07_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==27U?"P27_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==26U?"P26_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==25U?"P25_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==24U?"P24_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==23U?"P23_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==22U?"P22_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==21U?"P21_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==17U?"P17_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==16U?"P16_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==15U?"P15_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==14U?"P14_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==13U?"P13_SOURCE_GEOMETRY_OR_CURRENT_F":s.sourceP==12U?"P12_SOURCE_GEOMETRY_OR_CURRENT_F":"P11_SOURCE_GEOMETRY_OR_CURRENT_F",AlarmManager::G_Code_Invalid_parameter);
        NCG76SphereCoordinates::Point3 target{{pathFrame.programStart[0],pathFrame.programStart[1],pathFrame.programStart[2]}};
        for(unsigned axis=0U;axis<3U;++axis)if(words.targetMask&(1U<<axis))target[axis]=words.authoredXYZ[axis];
        const bool geometryBuilt=NCG76Mode::IsFixedRadius(s.sourceP)?
            NCG76OrbitContour::BuildTimedFixedRadius(s.sourceP,pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry,words.polygonSides):s.sourceP==27U?
            NCG76OrbitContour::BuildTimedP27(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),words.polygonSides,s.request.geometry):s.sourceP==26U?
            NCG76OrbitContour::BuildTimedP26(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),words.polygonSides,s.request.geometry):s.sourceP==25U?
            NCG76OrbitContour::BuildTimedP25(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==24U?
            NCG76OrbitContour::BuildTimedP24(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==23U?
            NCG76OrbitContour::BuildTimedP23(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==22U?
            NCG76OrbitContour::BuildTimedP22(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==21U?
            NCG76OrbitContour::BuildTimedP21(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==17U?
            NCG76OrbitContour::BuildTimedP17(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),words.polygonSides,s.request.geometry):s.sourceP==16U?
            NCG76OrbitContour::BuildTimedP16(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),words.polygonSides,s.request.geometry):s.sourceP==15U?
            NCG76OrbitContour::BuildTimedP15(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==14U?
            NCG76OrbitContour::BuildTimedP14(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==13U?
            NCG76OrbitContour::BuildTimedP13(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):s.sourceP==12U?
            NCG76OrbitContour::BuildTimedP12(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry):
            NCG76OrbitContour::BuildTimedP11(pathFrame,target,words.radiusProgramMm,
                words.contourAngleDeg,words.feedMmMin,(s.machiningTime.additionalUs!=0ULL||s.request.orbitInspectionRounds!=0U),s.request.geometry);
        if(!geometryBuilt||!TimedP11SourceMatchesGeometry(words,s.machiningTime,s.request.geometry))
            return reject(NCG76Mode::IsFixedRadius(s.sourceP)?"P01_P07_NATIVE_ENVELOPE":s.sourceP==27U?"P27_NATIVE_ENVELOPE":s.sourceP==26U?"P26_NATIVE_ENVELOPE":s.sourceP==25U?"P25_NATIVE_ENVELOPE":s.sourceP==24U?"P24_NATIVE_ENVELOPE":s.sourceP==23U?"P23_NATIVE_ENVELOPE":s.sourceP==22U?"P22_NATIVE_ENVELOPE":s.sourceP==21U?"P21_NATIVE_ENVELOPE":s.sourceP==17U?"P17_NATIVE_ENVELOPE":s.sourceP==16U?"P16_NATIVE_ENVELOPE":s.sourceP==15U?"P15_NATIVE_ENVELOPE":s.sourceP==14U?"P14_NATIVE_ENVELOPE":s.sourceP==13U?"P13_NATIVE_ENVELOPE":s.sourceP==12U?"P12_NATIVE_ENVELOPE":"P11_NATIVE_ENVELOPE",AlarmManager::G_Code_Invalid_parameter);
        // Contours share provenance and one parent, never a fabricated sphere
        // descriptor, layer step, radius law or prepared native receipt.
        s.prepared.requestedB=words.requestedB;s.prepared.explicitB=words.explicitB;
        s.prepared.inheritedB=words.inheritedB;s.prepared.inheritedBSource=words.inheritedBSource;
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
    // Build the independent nominal station once on NC. Every P50 now
    // requires the frozen SIM process binding below. Nonzero B routes also
    // require this checkpoint; no cursor grants motion or discharge authority.
    if(!processPath&&!timedContour)PrintSphereMemoryStage("NC_PATH_BUILD_BEGIN",pc);
    (void)NCG76SpherePath::Build(s.request.geometry,s.path);
    if(!processPath&&!timedContour)PrintSphereMemoryStage("NC_PATH_BUILD_END",pc);
    // EDM339: P50 always freezes both machining roles. An invalid/missing
    // orbit field rejects before publication, including B0; no AIR fallback.
    if(!processPath)s.gapBinding=NCG76SphereGapBinding::Capture(m_recipe,m_edmProcessProfile,
        m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly,false,true);
    s.request.retreatMode=s.prepared.requestedB;
    if(!processPath&&(!s.gapBinding.ready||!s.gapBinding.config.orbitFeedsEnabled)){
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_BINDING_REJECT reason=%s error=%s field=%u discharge=0\n",
            NCG76SphereGapBinding::ReasonName(s.gapBinding.reason),
            EDM58::BindingErrorName(s.gapBinding.error),s.gapBinding.failedFieldId);
        return reject("G76_ORBIT_REQUIRES_FROZEN_GAP_RECIPE",AlarmManager::G_Code_Invalid_parameter);
    }
    // A confirmed typed error is different from unavailable/unconfirmed
    // optional fields. It rejects B0 too, before frame freeze or submission.
    if(s.gapBinding.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        return reject("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if(s.gapBinding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        return reject("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if(s.gapBinding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        return reject("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);
    }
    if((processPath||timedContour)&&(!s.path.valid||!s.gapBinding.ready||!s.gapBinding.config.enabled||
        !s.gapBinding.config.signedMotionEnabled))
        return reject("PROCESS_PATH_REQUIRES_FROZEN_GAP_RECIPE",AlarmManager::G_Code_Invalid_parameter);
    // B2 needs a canonical same-parent station. B3/B4 also retain this
    // original checkpoint for HOLD and diagnostics throughout their routes.
    if(s.request.retreatMode!=0U&&(!s.path.valid||!s.gapBinding.ready||
        !s.gapBinding.config.enabled||(!s.gapBinding.config.flushMotionEnabled&&!StationaryProcessLine(s.sourceG,s.request.geometry))||
        !s.gapBinding.config.signedMotionEnabled)){
        PrintParent(s.sourceG,s.sourceP,"event=B_MODE_BINDING_REJECT B=%u reason=%s error=%s field=%u approachReason=%s approachError=%s approachField=%u discharge=0\n",
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
    if(!processPath&&!timedContour)PrintSphereMemoryStage("NC_REQUEST_VALIDATE_BEGIN",pc);
    const bool requestValid=NCG76SphereCycle::ValidateRequest(s.request);
    if(!processPath&&!timedContour)PrintSphereMemoryStage("NC_REQUEST_VALIDATE_END",pc);
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
        PrintParent(s.sourceG,s.sourceP,"event=SCOPE_REJECT session=%llu line=%d gate=%s axis=%d expectedEpoch=%llu actualEpoch=%llu discharge=0\n",
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
    const bool timedContour=s.sourceG==76&&NCG76TimedP11Source::SupportedMode(static_cast<double>(s.sourceP));
    const auto currentModal=CaptureG76SphereModalBSameThread();
    const auto& inherited=s.prepared.inheritedBSource;
    const bool processModalCurrent=(!processPath&&!timedContour)||s.prepared.explicitB||
        (s.prepared.inheritedB?(NCG76SphereSource::ValidModalB(currentModal)&&
            currentModal.value==inherited.value&&currentModal.commit==inherited.commit&&
            NCG76SphereSource::SameModalBScope(currentModal.committed,inherited.committed)):
            !currentModal.valid);
    const auto* finishSource=GetBaseProgramCache().TryGetLine(s.pc);std::uint32_t finishCount=0U;
    const bool finishCurrent=processPath?(finishSource&&
        NCFinishPolicy::Decode(finishSource->parsedBlock,block,finishCount)&&finishCount==s.finishRequiredCount):
        s.finishRequiredCount==0U;
    NCMachiningTime::Config currentTime{};
    const bool timeCurrent=finishSource&&NCMachiningTimePolicy::Decode(finishSource->parsedBlock,block,currentTime)&&
        NCMachiningTime::SameConfig(currentTime,s.machiningTime)&&
        MotionG76SphereMachiningTime::Matches(currentTime,s.request);
    std::uint32_t currentRounds=0U;
    const bool completionCurrent=processPath?(s.request.orbitInspectionRounds==0U&&s.request.orbitThresholdScaled==0U&&s.request.orbitRetryLimit==0U):
        (finishSource&&NCOrbitCompletionPolicy::Decode(finishSource->parsedBlock,block,currentRounds)&&
         currentRounds==s.request.orbitInspectionRounds&&SameLiteral(s.orbitCompletionSource,block)&&
         NCOrbitCompletionBinding::IsCurrent(s.orbitCompletionBinding,m_recipe,currentRounds)&&
         s.request.orbitThresholdScaled==(currentRounds?static_cast<std::uint32_t>(s.orbitCompletionBinding.value):0U)&&
         s.request.orbitRetryLimit==(currentRounds?s.orbitCompletionBinding.retryLimit:0U));
    const bool processFeedCurrent=!processPath||(s.request.parameterFeeds&&
        SameDouble(s.request.feedMmMin,s.processMotionCeiling)&&
        SameDouble(s.request.geometry.feedMmMin,s.processMotionCeiling)&&
        (s.pathSource.has('F')?(s.feed.valid&&s.feed.programmed&&IsCncFeedValueCurrentSameThread(s.feed)&&
            SameDouble(s.request.nominalFeedCapMmMin,s.feed.feedMMMin)):
            (!s.feed.valid&&!s.feed.programmed&&s.feed.feedMMMin==0.0&&s.request.nominalFeedCapMmMin==0.0)));
    NCG76TimedP11Source::Words currentContour{};
    const bool contourCurrent=!timedContour||(finishSource&&NCG76TimedP11Source::IsOptIn(block)&&
        NCG76TimedP11Source::Decode(finishSource->parsedBlock,block,currentModal,currentContour)==
            NCG76TimedP11Source::Status::Ok&&SameLiteral(s.pathSource,block)&&
        SameNCTranslationSnapshot(s.frame,s.request.geometry.frame.translation)&&
        TimedP11SourceMatchesGeometry(currentContour,currentTime,s.request.geometry)&&
        !s.request.parameterFeeds&&s.feed.valid&&s.feed.programmed&&IsCncFeedValueCurrentSameThread(s.feed)&&
        SameDouble(s.feed.feedMMMin,s.request.feedMmMin)&&
        SameDouble(s.request.feedMmMin,s.request.geometry.feedMmMin)&&s.request.nominalFeedCapMmMin==0.0&&
        s.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour);
    if(s.pc!=GetActiveDispatchPC()||s.dispatch!=m_currentExecutingBlockDispatchId||
        !processModalCurrent||!processFeedCurrent||!contourCurrent||!finishCurrent||!timeCurrent||!completionCurrent||s.sourceG!=block.gCode||
        s.sourceP!=(processPath?0U:static_cast<unsigned>(block.val('P')))||s.request.retreatMode!=s.prepared.requestedB||
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
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    // NC-only recapture also detects a typed error introduced after Prepare.
    // Never infer its state from the old ready bit or old approach reason.
    const auto currentApproach=NCG76SphereGapBinding::Capture(m_recipe,m_edmProcessProfile,
        m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly,processPath,!processPath);
    TraceG76SphereStartupSameThread(44U,"RECIPE_CAPTURE_DONE");
    if(currentApproach.approachReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereApproachRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_APPROACH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(currentApproach.initialSlowReason==NCG76SphereGapBinding::ApproachReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereInitialSlowRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_INITIAL_SLOW_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(currentApproach.deepFlushReason==NCG76SphereGapBinding::DeepFlushReason::InvalidField){
        s.gapBinding=currentApproach;s.gapBinding.config=NCG76SphereGapProcess::Config{};
        PrintSphereDeepFlushRejection(s.gapBinding,s.request.retreatMode,s.sourceG,s.sourceP);
        RejectG76SphereSameThread("TYPED_DEEP_FLUSH_FIELD_INVALID",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
    if(s.gapBinding.ready&&!NCG76SphereGapBinding::IsCurrent(s.gapBinding,m_recipe,
        m_edmProcessProfile,m_edmProcessTuningGeneration,m_edmProcessReady&&!m_recipeRecoveryDisplayOnly)){
        s.gapBinding.config=NCG76SphereGapProcess::Config{};s.gapBinding.ready=false;
        s.gapBinding.reason=NCG76SphereGapBinding::Reason::CaptureChanged;
        s.gapBinding.error=EDM58::BindingError::CaptureChanged;s.gapShadowRevoked=true;
    }
    TraceG76SphereStartupSameThread(45U,"RECIPE_CURRENT_DONE");
    if(!processPath&&(!s.gapBinding.ready||!s.gapBinding.config.orbitFeedsEnabled||
       !currentApproach.ready||!currentApproach.config.orbitFeedsEnabled)){
        RejectG76SphereSameThread("G76_ORBIT_RECIPE_CHANGED",AlarmManager::G_Code_Invalid_parameter);return nullptr;
    }
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
        PrintParent(s.sourceG,s.sourceP,"event=ADMISSION_REJECT session=%llu line=%d gate=%s axis=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),s.line,MotionG76SphereAdmissionGateName(admission.gate),admission.axis);
        RejectG76SphereSameThread("MOTION_ADMISSION",AlarmManager::PATH_MOTION_NOT_ADMITTED);return nullptr;
    }
    s.active=s.submitted=true;s.staged=false;
    if(processPath)PrintParent(s.sourceG,s.sourceP,"event=FINISH_CONFIG session=%llu required=%u scope=THIS_PARENT rework=RESIDUAL_GAP_PATH periodicE5=INDEPENDENT forcedHeight=NORMAL_E6 discharge=0\n",
        static_cast<unsigned long long>(s.session),s.finishRequiredCount);
    if(processPath){
        const unsigned requested=s.prepared.explicitB?static_cast<unsigned>(s.pathSource.val('B')):
            s.prepared.inheritedB?s.prepared.inheritedBSource.value:0U;
        PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d g=%d geometry=%s Fmode=%s nominalFcapMmMin=%.9f motionCeilingMmMin=%.9f B=%u inheritedB=%u parents=1 layers=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.sourceG,s.sourceG==1?"ANALYTIC_LINE":"ANALYTIC_ARC",
            s.pathSource.has('F')?"CURRENT_BLOCK_CAP":"PARAMETER_ONLY",s.request.nominalFeedCapMmMin,
            s.processMotionCeiling,s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        PrintParent(s.sourceG,s.sourceP,"event=B_RESOLUTION session=%llu g=%d authoredB=%d requestedB=%u effectiveB=%u forced=%u policy=%s origin=%s discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceG,s.prepared.explicitB?static_cast<int>(requested):-1,
            requested,s.request.retreatMode,s.sourceG!=1&&requested!=2U?1U:0U,
            s.sourceG==1?"SOURCE_MODAL":"ARC_ALWAYS_B2",s.prepared.explicitB?"EXPLICIT":
                s.prepared.inheritedB?"COMMITTED":"DEFAULT_B0");
        PrintParent(s.sourceG,s.sourceP,"event=FEED_POLICY session=%llu source=PARAMETERS optionalF=%u nominalFcapMmMin=%.9f modalFRead=0 flushIndependent=1 internalCeilingMmMin=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pathSource.has('F')?1U:0U,
            s.request.nominalFeedCapMmMin,s.processMotionCeiling);
        PrintParent(s.sourceG,s.sourceP,"event=PROCESS_PATH session=%llu valid=%u pathMm=%.9f nominalScope=THIS_PARENT previousParentPolicy=RELEASED_ONE_B2 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.path.valid?1U:0U,s.path.totalLengthMm);
        // Observe the admitted native geometry on NC, after the immutable
        // frame has been checked. Never apply a second coordinate transform.
        PrintParent(s.sourceG,s.sourceP,"event=FRAME_MAPPING session=%llu distance=%d plane=%d rotation=%d angleDeg=%.9f wcs=%d Hmode=%d H=%d work=%d discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.distanceMode,s.frame.rotationPlane,
            s.frame.rotationMode,s.frame.rotationAngleDeg,s.frame.wcsCode,s.frame.toolLengthMode,
            s.frame.toolHCode,s.frame.workMode);
        PrintParent(s.sourceG,s.sourceP,"event=NATIVE_PATH session=%llu startX=%.9f startY=%.9f startZ=%.9f endX=%.9f endY=%.9f endZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.startMCS[0],
            s.request.geometry.startMCS[1],s.request.geometry.startMCS[2],s.request.geometry.endMCS[0],
            s.request.geometry.endMCS[1],s.request.geometry.endMCS[2]);
        PrintParent(s.sourceG,s.sourceP,"event=PROGRAM_SOURCE session=%llu startX=%.9f startY=%.9f startZ=%.9f Xword=%.9f Yword=%.9f Zword=%.9f axisMask=%u centerU=%.9f centerV=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.frame.programStart[0],
            s.request.geometry.frame.programStart[1],s.request.geometry.frame.programStart[2],
            s.pathSource.val('X'),s.pathSource.val('Y'),s.pathSource.val('Z'),
            (s.pathSource.has('X')?1U:0U)|(s.pathSource.has('Y')?2U:0U)|(s.pathSource.has('Z')?4U:0U),
            s.frame.rotationCenterMM[0],s.frame.rotationCenterMM[1]);
        PrintParent(s.sourceG,s.sourceP,"event=FRAME_OFFSETS session=%llu extX=%.9f extY=%.9f extZ=%.9f wcsX=%.9f wcsY=%.9f wcsZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.extOffsetMM[0],s.frame.extOffsetMM[1],
            s.frame.extOffsetMM[2],s.frame.wcsOffsetMM[0],s.frame.wcsOffsetMM[1],s.frame.wcsOffsetMM[2]);
        PrintParent(s.sourceG,s.sourceP,"event=TOOL_OFFSETS session=%llu Hmode=%d H=%d rawX=%.9f rawY=%.9f rawZ=%.9f effectiveX=%.9f effectiveY=%.9f effectiveZ=%.9f eccentricFrozen=%u discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.toolLengthMode,s.frame.toolHCode,
            s.frame.toolOffsetMM[0],s.frame.toolOffsetMM[1],s.frame.toolOffsetMM[2],
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[0]:NCTranslationToolOffsetMM(s.frame,0U),
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[1]:NCTranslationToolOffsetMM(s.frame,1U),
            s.request.geometry.frame.fixedEccentric?s.request.geometry.frame.signedEccentricOffset[2]:NCTranslationToolOffsetMM(s.frame,2U),
            s.request.geometry.frame.fixedEccentric?1U:0U);
        PrintParent(s.sourceG,s.sourceP,"event=WORK_OFFSETS session=%llu mode=%d W=%d X=%.9f Y=%.9f Z=%.9f yawDeg=%.9f pitchDeg=%.9f rollDeg=%.9f centerU=%.9f centerV=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.workMode,s.frame.workWCode,
            s.frame.workOffset[0],s.frame.workOffset[1],s.frame.workOffset[2],
            s.frame.workOffset[3],s.frame.workOffset[4],s.frame.workOffset[5],
            s.frame.workRotationCenterMM[0],s.frame.workRotationCenterMM[1]);
        PrintParent(s.sourceG,s.sourceP,"event=SCALE_FRAME session=%llu mode=%d factor=%.9f centerX=%.9f centerY=%.9f centerZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),s.frame.scalingMode,s.frame.scalingFactor,
            s.frame.scalingCenterMM[0],s.frame.scalingCenterMM[1],s.frame.scalingCenterMM[2]);
        PrintParent(s.sourceG,s.sourceP,"event=MIRROR_FRAME session=%llu mask=%u centerX=%.9f centerY=%.9f centerZ=%.9f discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(s.frame.mirrorMask),
            s.frame.mirrorCenterMM[0],s.frame.mirrorCenterMM[1],s.frame.mirrorCenterMM[2]);
    }else if(timedContour){
        if(NCG76Mode::IsFixedRadius(s.sourceP))PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=%u Q=%u geometry=ORBIT_CONTOUR nativeR=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=FIXED_RADIUS_ADVANCE shape=%s plane=TARGET_NORMAL additional=TERMINAL_FULL_CONTOUR completion=SOURCE_L directionalClipping=0 depthReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.sourceP,s.request.geometry.orbit.source.polygonSides,
            s.request.geometry.radiusMm,s.request.feedMmMin,s.request.retreatMode,s.prepared.inheritedB?1U:0U,
            NCG76Mode::ShapeName(s.sourceP));
        else if(s.sourceP==27U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=27 Q=%u geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_VECTOR_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.orbit.source.polygonSides,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U,s.request.geometry.orbit.source.polygonSides);
        else if(s.sourceP==26U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=26 Q=%u geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.orbit.source.polygonSides,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U,s.request.geometry.orbit.source.polygonSides);
        else if(s.sourceP==25U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=25 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_VECTOR_DIAMOND sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==24U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=24 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_VECTOR_SQUARE sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==23U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=23 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_DIAMOND sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==22U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=22 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_SQUARE sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==21U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=21 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=26 path=APPROACH_THEN_FIXED_PLANE_GROW additional=TERMINAL_ANALYTIC_CIRCLE plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==17U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=17 Q=%u geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_VECTOR_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.orbit.source.polygonSides,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U,s.request.geometry.orbit.source.polygonSides);
        else if(s.sourceP==16U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=16 Q=%u geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.orbit.source.polygonSides,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U,s.request.geometry.orbit.source.polygonSides);
        else if(s.sourceP==15U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=15 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_VECTOR_DIAMOND sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==14U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=14 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_VECTOR_SQUARE sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==13U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=13 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_DIAMOND sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else if(s.sourceP==12U)PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=12 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_SQUARE sideLegs=4 corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        else
        PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d P=11 geometry=ORBIT_CONTOUR nativeR=%.9f contourAngleDeg=%.9f F=%.9f B=%u inheritedB=%u nominalLegs=25 additional=TERMINAL_ANALYTIC_CIRCLE depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.radiusMm,
            s.pathSource.has('K')?s.pathSource.val('K'):0.0,s.request.feedMmMin,
            s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        PrintParent(s.sourceG,s.sourceP,"event=PROCESS_PATH session=%llu valid=%u pathMm=%.9f kind=ORBIT_CONTOUR gap=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.path.valid?1U:0U,s.path.totalLengthMm);
    }else{
        PrintParent(s.sourceG,s.sourceP,"event=BEGIN session=%llu pc=%d layers=%u R=%.9f K=%.9f F=%.9f B=%u inheritedB=%u radiusLaw=R_MINUS_DEPTH contour=ANALYTIC_CIRCLE discharge=0\n",
            static_cast<unsigned long long>(s.session),s.pc,s.request.geometry.circleLayerCount,
            s.request.geometry.radiusMm,s.request.geometry.layerStepMm,s.request.feedMmMin,s.request.retreatMode,s.prepared.inheritedB?1U:0U);
        PrintParent(s.sourceG,s.sourceP,"event=PROCESS_PATH session=%llu valid=%u pathMm=%.9f nativeR=%.9f nativeK=%.9f gap=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.path.valid?1U:0U,s.path.totalLengthMm,
            s.request.geometry.radiusMm,s.request.geometry.layerStepMm);
    }
    TraceG76SphereStartupSameThread(48U,"SOURCE_LOGS_DONE");
    const auto& binding=s.gapBinding;const auto& config=binding.config;
    if(NCMachiningTime::Enabled(s.machiningTime))
        PrintParent(s.sourceG,s.sourceP,"event=TIME_CONFIG session=%llu totalLimitUs=%llu additionalUs=%llu scope=THIS_PARENT H=VALID_PROCESS_TIME HOLD=PAUSED timeLimit=NORMAL_COMPLETION G76=CURRENT_CENTER discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.request.totalLimitUs),
            static_cast<unsigned long long>(s.request.additionalUs));
    if(s.request.orbitInspectionRounds)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_COMPLETION_CONFIG build=EDM355 session=%llu requiredPasses=%u targetPercent=%.6f field=%u source=SOFTWARE_SIMULATION order=NOMINAL_H_INSPECTION pass=CONSECUTIVE_FULL_SWEEPS retryLimit=%u retries=ADDITIONAL_FAILED_SWEEPS zero=UNLIMITED discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(s.request.orbitInspectionRounds),
            s.request.orbitThresholdScaled/1000000.0,s.orbitCompletionBinding.field,static_cast<unsigned>(s.request.orbitRetryLimit));
    if(config.orbitFeedsEnabled&&s.sourceG==76&&NCG76Mode::IsFixedRadius(s.sourceP))
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f entry=ORBIT fixedRadiusAdvance=ORBIT terminalContour=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==27U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT vectorPolygonSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==26U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT polygonSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==25U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT vectorDiamondSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==24U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT vectorSquareSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==23U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT diamondSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==22U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT squareSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==21U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING grow=ORBIT entry=ORBIT circle=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==17U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT vectorPolygonSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==16U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT polygonSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==15U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT vectorDiamondSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==14U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT vectorSquareSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==13U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT diamondSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled&&s.sourceG==76&&s.sourceP==12U)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f grow=ORBIT entry=ORBIT squareSides=ORBIT return=RETURN multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    else if(config.orbitFeedsEnabled)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_FEED_CONFIG session=%llu cuttingScale=%.9f machiningPct=%.6f orbitScale=%.9f orbitPct=%.6f FcapMmMin=%.6f depth=CUTTING entry=ORBIT circle=ORBIT multiplier=ONCE source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),config.curve.cuttingScaleMmPerVoltMin,
            config.machiningOverridePercent,config.orbitScaleMmPerVoltMin,config.orbitOverridePercent,s.request.feedMmMin);
    PrintParent(s.sourceG,s.sourceP,"event=GAP_BINDING session=%llu enabled=%u reason=%s error=%s field=%u recipe=%llu tuning=%llu E5us=%llu E6mm=%.6f E7pct=%.3f E8pct=%.3f E9V=%.3f source=SOFTWARE_SIMULATION AD=0 motionApplied=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.ready?1U:0U,NCG76SphereGapBinding::ReasonName(binding.reason),
        EDM58::BindingErrorName(binding.error),binding.failedFieldId,static_cast<unsigned long long>(config.recipeGeneration),
        static_cast<unsigned long long>(config.tuningGeneration),static_cast<unsigned long long>(config.workTimeUs),
        config.jumpHeightMm,config.jumpSpeedPercent,config.machiningOverridePercent,config.referenceV);
    PrintParent(s.sourceG,s.sourceP,"event=GAP_GAIN_CONFIG session=%llu bindingReady=%u enabled=%u gainSource=%s table=%u E=%u PositiveGain1=%.6f PositiveGain2=%.6f PositiveGain3=%.6f NegativeGain1=%.6f NegativeGain2=%.6f NegativeGain3=%.6f fileUnchanged=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.ready?1U:0U,binding.specialGain.enabled?1U:0U,
        binding.ready?(binding.specialGain.enabled?"E_ROW":"PROCESS_FILE"):"UNAVAILABLE",binding.specialGain.selection.tableId,
        static_cast<unsigned>(binding.specialGain.selection.eCode),config.curve.positiveGain[0],config.curve.positiveGain[1],
        config.curve.positiveGain[2],config.curve.negativeGain[0],config.curve.negativeGain[1],config.curve.negativeGain[2]);
    PrintParent(s.sourceG,s.sourceP,"event=SIM_MOTION_CONFIG session=%llu enabled=%u domain=%s historyCapacity=%u nominalParentOnly=1 work=%s E5=NOMINAL_FORWARD_ONLY source=SOFTWARE_SIMULATION AD=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.ready&&config.signedMotionEnabled?1U:0U,
        processPath?"SOURCE_AND_RELEASED_PREDECESSOR_B2":"PREVIOUS_COMPLETED_LEG",1U,
        processPath?"SOURCE_FORWARD":timedContour?(NCG76Mode::IsFixedRadius(s.sourceP)?"P01_P07_26_FIXED_RADIUS_LEGS":s.sourceP==27U?"P27_26_EXACT_STOP_LEGS":s.sourceP==26U?"P26_26_EXACT_STOP_LEGS":s.sourceP==25U?"P25_26_EXACT_STOP_LEGS":s.sourceP==24U?"P24_26_EXACT_STOP_LEGS":s.sourceP==23U?"P23_26_EXACT_STOP_LEGS":s.sourceP==22U?"P22_26_EXACT_STOP_LEGS":s.sourceP==21U?"P21_26_EXACT_STOP_LEGS":s.sourceP==17U?"P17_25_EXACT_STOP_LEGS":s.sourceP==16U?"P16_25_EXACT_STOP_LEGS":s.sourceP==15U?"P15_25_EXACT_STOP_LEGS":s.sourceP==14U?"P14_25_EXACT_STOP_LEGS":s.sourceP==13U?"P13_25_EXACT_STOP_LEGS":s.sourceP==12U?"P12_25_EXACT_STOP_LEGS":"P11_25_EXACT_STOP_LEGS"):"INITIAL_DEPTH_CIRCLE_ENTRY_ORBIT");
    TraceG76SphereStartupSameThread(49U,"RECIPE_LOGS_DONE");
    NCG76SphereCycle::FlushConfig flush{};
    const bool flushReady=NCG76SphereGapProcess::BuildFlushConfig(config,s.request.feedMmMin,flush)&&flush.enabled;
    const char* route=s.request.retreatMode==2U?"SAME_PARENT_PATH":s.request.retreatMode==3U?
        (processPath?"CENTERLINE_AXIS_COALESCED":"CENTER_THEN_AXIS"):s.request.retreatMode==4U?
        (processPath?"CENTERLINE_DIAGONAL_COALESCED":"DIAGONAL_CENTER_THEN_AXIS"):s.request.retreatMode==1U?
        "MACHINE_POSITIVE_Z":(processPath||NCG76Mode::IsFixedRadius(s.sourceP)||s.sourceP==12U||s.sourceP==13U||s.sourceP==14U||s.sourceP==15U||s.sourceP==16U||s.sourceP==17U||s.sourceP==21U||s.sourceP==22U||s.sourceP==23U||s.sourceP==24U||s.sourceP==25U||s.sourceP==26U||s.sourceP==27U)?"OPPOSITE_STOPPED_TANGENT":"OPPOSITE_NATIVE_CUT_AXIS";
    PrintParent(s.sourceG,s.sourceP,"event=FLUSH_CONFIG session=%llu enabled=%u B=%u route=%s heightMm=%.6f feedMmMin=%.6f zeroSamples=%u historyShort=DEFER source=SOFTWARE_SIMULATION discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady?1U:0U,s.request.retreatMode,route,
        flushReady?flush.heightMm:0.0,flushReady?flush.feedMmMin:0.0,s.request.settleSamples);
    TraceG76SphereStartupSameThread(50U,"FLUSH_LOG_DONE");
    // Cooked native feeds are frozen once with this parent. A disabled E11/E13
    // approach is explicit metadata; it never rewrites a stored raw recipe.
    PrintParent(s.sourceG,s.sourceP,"event=FLUSH_SPEED_CONFIG session=%llu independent=%u Pump1=%.6f Pump2=%.6f Pump3=%.6f Pump4=%.6f Pump5=%.6f approachEnabled=%u reason=%s field=%u E11pct=%.3f E13mm=%.6f nativeBase=%.6g final=%.6f unit=mm/min source=SOFTWARE_SIMULATION discharge=0\n",
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
    PrintParent(s.sourceG,s.sourceP,"event=E11_PERCENT_CONFIG session=%llu E11pct=%.6g unit=%% source=%s basis=NATIVE_PUMP_STG_AND_E7 nativeBase=%.6g E7pct=%.6g resolvedCapMmMin=%.6g approachEnabled=%u distanceZeroSentinel=%u discharge=0\n",
        static_cast<unsigned long long>(s.session),binding.resolvedE11Percent,
        NCG76SphereGapBinding::E11UnitSourceName(binding.e11UnitSource),config.flushProfile.finalApproachMmPerMin,
        config.jumpSpeedPercent,resolvedApproachCap,flushReady&&flush.finalApproachEnabled?1U:0U,
        binding.distanceZeroSentinel?1U:0U);
    TraceG76SphereStartupSameThread(52U,"E11_LOG_DONE");
    PrintParent(s.sourceG,s.sourceP,"event=APPROACH_CONFIG session=%llu status=%s reason=%s error=%s field=%u E11field=%u E13field=%u rawE11=%lld rawE13=%lld rawOnlyWhenDisabled=1 discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.finalApproachEnabled?"APPROACH_ENABLED":"APPROACH_DISABLED",
        NCG76SphereGapBinding::ApproachReasonName(binding.approachReason),EDM58::BindingErrorName(binding.approachError),
        binding.approachFailedFieldId,binding.approachFields[0],binding.approachFields[1],
        static_cast<long long>(binding.approachValues[0]),static_cast<long long>(binding.approachValues[1]));
    PrintParent(s.sourceG,s.sourceP,"event=INITIAL_SLOW_CONFIG session=%llu enabled=%u reason=%s error=%s field=%u E19field=%u E20field=%u rawE19=%lld rawE20=%lld activeE19pct=%.6g activeE20mm=%.6g nativeBase=%.6g capMmMin=%.6g unit=mm/min discharge=0\n",
        static_cast<unsigned long long>(s.session),flushReady&&flush.initialSlowEnabled?1U:0U,
        NCG76SphereGapBinding::ApproachReasonName(binding.initialSlowReason),EDM58::BindingErrorName(binding.initialSlowError),
        binding.initialSlowFailedFieldId,binding.initialSlowFields[0],binding.initialSlowFields[1],
        static_cast<long long>(binding.initialSlowValues[0]),static_cast<long long>(binding.initialSlowValues[1]),
        config.initialSlowPercent,config.initialSlowDistanceMm,config.flushProfile.initialSlowMmPerMin,
        flush.initialSlowFeedMmMin);
    PrintParent(s.sourceG,s.sourceP,"event=DEEP_FLUSH_CONFIG session=%llu enabled=%u reason=%s error=%s field=%u E21field=%u E22field=%u rawE21=%lld rawE22=%lld activePeriod=%u activeFactor=%.6g baseHeightMm=%.12f deepHeightMm=%.12f scope=%s count=SUCCESS_ONLY discharge=0\n",
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
    PrintParent(s.sourceG,s.sourceP,"event=WAIT_CALLBACK_REJECT session=%llu pc=%d callback=0x%llx expected=0x%llx invoked=0 discharge=0\n",
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
            PrintParent(s.sourceG,s.sourceP,"event=RETURN_PROBE session=%llu pc=%d frame=%.16s stage=%u available=%u status=%s slot=0x%llx expected=0x%llx actual=0x%llx readBytes=%u action=OBSERVE_ONLY discharge=0\n",
                static_cast<unsigned long long>(s.session),s.pc,frame,stage,read?1U:0U,
                !read?"UNAVAILABLE":mismatch?"MISMATCH":"MATCH",
                static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(probe.slot)),
                static_cast<unsigned long long>(probe.expected),static_cast<unsigned long long>(actual),
                read?static_cast<unsigned>(sizeof(actual)):0U);
            if(mismatch)PrintParent(s.sourceG,s.sourceP,"event=RETURN_PATTERN session=%llu frame=%.16s stage=%u readable=%u bytes=%u matched=%u exactSnapshot=%u q0=0x%llx q1=0x%llx q2=0x%llx q3=0x%llx expectedRun=%llu action=OBSERVE_ONLY discharge=0\n",
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
    PrintParent(s.sourceG,s.sourceP,"event=STARTUP_STAGE session=%llu pc=%d stage=%u name=%.32s callback=0x%llx expected=0x%llx match=%u sizeNC=%llu sizeSnapshot=%llu sizeFeedback=%llu discharge=0\n",
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
    PrintParent(s.sourceG,s.sourceP,"event=BOUND session=%llu dispatch=%llu commit=%llu children=1 discharge=0\n",
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
    if(matching&&FailedSphereFeedback(observed))PrintSphereRuntimeFailure(observed,s.request,s.line,s.sourceG,s.sourceP);
    if(matching&&MotionG76SphereMachiningTime::TimeLimitStoppedQualified(observed,s.request)){
        s.feedback=observed;
        RecordDHTerminal(s,NCG76DHBatchDiagnostic::Result::TimeLimit);
        PrintParent(s.sourceG,s.sourceP,"event=TIME_LIMIT session=%llu tick=%llu totalElapsedUs=%llu totalLimitUs=%llu endpointCommitted=0 alarm=4040 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.cycle.tick),
            static_cast<unsigned long long>(observed.cycle.totalElapsedUs),static_cast<unsigned long long>(observed.cycle.totalLimitUs));
        RejectG76SphereSameThread("MACHINING_TIME_LIMIT",AlarmManager::EDM_MACHINING_TIME_LIMIT);return;
    }
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
    const bool completionRecipeChanged=s.sourceG==76&&!s.gapShadowRevoked&&!NCOrbitCompletionBinding::IsCurrent(
        s.orbitCompletionBinding,m_recipe,s.request.orbitInspectionRounds);
    if(recipeChanged||completionRecipeChanged){
        s.gapShadowRevoked=true;s.gapBinding.ready=false;
        (void)m_motion.RevokeG76SphereProcessShadow(s.session,s.identity,s.owner);
        PrintParent(s.sourceG,s.sourceP,"event=SIM_GAP_WITHDRAWN session=%llu reason=RECIPE_OR_TUNING_CHANGED rearm=NEXT_PARENT motionApplied=0 discharge=0\n",
            static_cast<unsigned long long>(s.session));
    }
    if(!matching){
        TraceG76SphereStartupSameThread(G76SphereStartupServiceDone,"SERVICE_DONE");return;
    }
    const bool flushLegChanged=s.feedback.cycle.flushLegSequence!=observed.cycle.flushLegSequence;
    const bool approachChanged=s.feedback.cycle.flushApproachActive!=observed.cycle.flushApproachActive;
    const bool initialSlowChanged=s.feedback.cycle.flushInitialSlowActive!=observed.cycle.flushInitialSlowActive;
    const bool feedSelectionChanged=s.feedback.processShadow.selectedFeedRole!=observed.processShadow.selectedFeedRole;
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
    if(observed.started&&observed.cycle.session==s.session){
        const auto& f=observed.cycle;
        if(f.orbitRequiredPasses!=s.request.orbitInspectionRounds||f.orbitTargetPercentScaled!=s.request.orbitThresholdScaled||
           f.orbitQualifiedPasses>f.orbitRequiredPasses||f.orbitAttemptCount<f.orbitQualifiedPasses||
           f.orbitPercentScaled>NCOrbitCompletionPolicy::MaximumPercentScaled||f.orbitCompletionState>7U||
           f.orbitRetryLimit!=s.request.orbitRetryLimit||
           (f.orbitCompletionState==7U&&!MotionG76SphereMachiningTime::RetryExhaustionQualified(f,s.request))||
           (f.orbitCompletionState==6U&&!MotionG76SphereMachiningTime::TimeLimitReachedQualified(f,s.request))||
           f.orbitCompletionActive!=(f.orbitCompletionState==2U||f.orbitCompletionState==3U)||
           (observed.completed&&!OrbitTerminalQualified(f,s.request))){
            RejectG76SphereSameThread("ORBIT_COMPLETION_FEEDBACK_SCOPE",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return;
        }
        if(s.request.orbitInspectionRounds&&(!s.orbitLogInitialized||s.orbitLoggedState!=f.orbitCompletionState||
           s.orbitLoggedAttempt!=f.orbitAttemptCount||s.orbitLoggedQualified!=f.orbitQualifiedPasses||
           s.orbitLoggedPercent/1000000U!=f.orbitPercentScaled/1000000U)){
            s.orbitLogInitialized=true;s.orbitLoggedState=f.orbitCompletionState;s.orbitLoggedAttempt=f.orbitAttemptCount;
            s.orbitLoggedQualified=f.orbitQualifiedPasses;s.orbitLoggedPercent=f.orbitPercentScaled;
            if(s.orbitDiagnosticEvents++<256U)PrintParent(s.sourceG,s.sourceP,"event=ORBIT_COMPLETION_STATE build=EDM355 session=%llu tick=%llu state=%u active=%u available=%u percent=%.6f targetPercent=%.6f qualifiedPasses=%u requiredPasses=%u attempts=%llu source=SOFTWARE_SIMULATION discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick),
                static_cast<unsigned>(f.orbitCompletionState),f.orbitCompletionActive?1U:0U,f.orbitPercentAvailable?1U:0U,
                f.orbitPercentScaled/1000000.0,f.orbitTargetPercentScaled/1000000.0,f.orbitQualifiedPasses,f.orbitRequiredPasses,
                static_cast<unsigned long long>(f.orbitAttemptCount));
        }
    }

    if(observed.started&&observed.cycle.session==s.session&&NCMachiningTime::Enabled(s.machiningTime)){
        const auto& f=observed.cycle;const auto second=f.totalElapsedUs/1000000ULL;
        if(!s.timeLogInitialized||s.timeLoggedPhase!=f.timePhase||s.timeLoggedTotalState!=f.totalTimerState||
           s.timeLoggedAdditionalState!=f.additionalTimerState||s.timeLoggedSecond!=second){
            s.timeLogInitialized=true;s.timeLoggedPhase=f.timePhase;s.timeLoggedTotalState=f.totalTimerState;
            s.timeLoggedAdditionalState=f.additionalTimerState;s.timeLoggedSecond=second;
            if(s.timeDiagnosticEvents++<192U)PrintParent(s.sourceG,s.sourceP,"event=TIME_STATE session=%llu tick=%llu phase=%u totalState=%u totalElapsedUs=%llu totalRemainingUs=%llu additionalState=%u additionalElapsedUs=%llu additionalRemainingUs=%llu totalPause=%u additionalPause=%u discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick),static_cast<unsigned>(f.timePhase),static_cast<unsigned>(f.totalTimerState),
                static_cast<unsigned long long>(f.totalElapsedUs),static_cast<unsigned long long>(f.totalLimitUs-f.totalElapsedUs),
                static_cast<unsigned>(f.additionalTimerState),static_cast<unsigned long long>(f.additionalElapsedUs),static_cast<unsigned long long>(f.additionalUs-f.additionalElapsedUs),
                static_cast<unsigned>(f.totalPauseReason),static_cast<unsigned>(f.additionalPauseReason));
        }
    }
    if(s.finishRequiredCount&&observed.started&&observed.cycle.session==s.session){
        const auto& f=observed.cycle;
        if(!s.finishLogInitialized||s.finishLoggedPhase!=f.finishPhase||s.finishLoggedVerified!=f.finishVerifiedCount||
           s.finishLoggedAttempt!=f.finishAttemptCount||s.finishLoggedCompleted!=f.finishForcedCompletedCount||
           s.finishLoggedRework!=f.finishReworkCount){
            s.finishLogInitialized=true;s.finishLoggedPhase=f.finishPhase;s.finishLoggedVerified=f.finishVerifiedCount;
            s.finishLoggedAttempt=f.finishAttemptCount;s.finishLoggedCompleted=f.finishForcedCompletedCount;
            s.finishLoggedRework=f.finishReworkCount;
            PrintParent(s.sourceG,s.sourceP,"event=FINISH_VERIFY session=%llu tick=%llu required=%u verified=%u phase=%u active=%u attempts=%llu forcedCompleted=%llu reworks=%llu origin=%u E5=SUSPENDED_DURING_FINISH discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(f.tick),s.finishRequiredCount,
                f.finishVerifiedCount,static_cast<unsigned>(f.finishPhase),f.finishActive?1U:0U,
                static_cast<unsigned long long>(f.finishAttemptCount),static_cast<unsigned long long>(f.finishForcedCompletedCount),
                static_cast<unsigned long long>(f.finishReworkCount),static_cast<unsigned>(f.flushOrigin));
        }
    }
    if(s.sourceG!=76&&observed.started&&observed.cycle.session==s.session&&!s.processSeedLogged){
        s.processSeedLogged=true;
        PrintParent(s.sourceG,s.sourceP,"event=CHAIN_SEED session=%llu initialIntervalWorkUs=%llu completedFlushOffset=%llu previousParentReady=%u previousSession=%llu previousPC=%d historyCapacity=1 E5=NOMINAL_FORWARD_ONLY ordinal=SUCCESS_ONLY discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.initialIntervalWorkUs),
            static_cast<unsigned long long>(observed.completedFlushOffset),
            observed.cycle.previousParentHistoryReady?1U:0U,
            static_cast<unsigned long long>(observed.previousParentSession),observed.previousParentPC);
        PrintParent(s.sourceG,s.sourceP,"event=SIM_TEST_POLICY session=%llu mode=%s probe=%s targetMaxMm=%.6f source=SOFTWARE_SIMULATION motionAuthority=UNCHANGED discharge=0\n",
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
        PrintParent(s.sourceG,s.sourceP,"event=SIM_PROBE session=%llu phase=%s tick=%llu ready=%u entered=%llu returned=%llu previousProgressMm=%.9f previousLengthMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f source=SOFTWARE_SIMULATION discharge=0\n",
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
                PrintParent(s.sourceG,s.sourceP,"event=%s session=%llu request=%llu tick=%llu phase=%s completed=%llu checkpoint=%u B=%u leg=%llu selectionRequest=%llu selectionCurrent=%u ordinal=%llu heightMm=%.12f deep=%u source=SOFTWARE_SIMULATION discharge=0\n",
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
        if(valid&&s.gapBinding.config.orbitFeedsEnabled&&feedSelectionChanged&&s.orbitFeedDiagnosticEvents++<96U)
            PrintParent(s.sourceG,s.sourceP,"event=FEED_SELECTION session=%llu tick=%llu nextGapRole=%s scale=%.9f overridePct=%.6f nextSignedTargetMmMin=%.6f phase=%s layer=%u history=%u flush=%u hold=%u discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(shadow.tick),
                NCG76SphereGapProcess::FeedRoleName(shadow.selectedFeedRole),shadow.selectedScaleMmPerVoltMin,
                shadow.selectedOverridePercent,shadow.signedGapSpeedMmMin,PhaseName(cycle.phase,s.sourceG),cycle.layerIndex,
                cycle.gapHistoryActive?1U:0U,cycle.flushActive?1U:0U,cycle.holdActive?1U:0U);
        const int direction=valid?(shadow.signedGapSpeedMmMin>0.0?1:shadow.signedGapSpeedMmMin<0.0?-1:0):0;
        if(coherent&&(!s.gapDiagnosticInitialized||phaseChanged||s.gapLastDirection!=direction||
            s.gapLastValid!=valid||s.gapLastSuspended!=shadow.suspended)){
            s.gapDiagnosticInitialized=true;s.gapLastDirection=direction;s.gapLastValid=valid;
            s.gapLastSuspended=shadow.suspended;
            if(s.gapDiagnosticEvents++<64U)
                PrintParent(s.sourceG,s.sourceP,"event=SIM_GAP session=%llu tick=%llu valid=%u phase=%s layer=%u voltageV=%.3f signedTargetMmMin=%.6f eligible=%u suspended=%u simulatedWorkUs=%llu pending=%u reason=%s source=SOFTWARE_SIMULATION motionApplied=%u actualSignedMmMin=%.6f discharge=0\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(shadow.tick),valid?1U:0U,
                    PhaseName(observed.cycle.phase,s.sourceG),observed.cycle.layerIndex,valid?shadow.voltageV:0.0,
                    valid?shadow.signedGapSpeedMmMin:0.0,valid&&shadow.eligible?1U:0U,shadow.suspended?1U:0U,
                    static_cast<unsigned long long>(valid?shadow.simulatedWorkUs:0ULL),valid&&shadow.pending?1U:0U,
                    NCG76SphereGapProcess::ReasonName(shadow.reason),gapApplied?1U:0U,
                    gapApplied?cycle.signedSpeedMmS*60.0:0.0);
        }
        if(valid&&shadow.pending&&shadow.requestSequence>s.gapRequestLogged){
            s.gapRequestLogged=shadow.requestSequence;
            PrintParent(s.sourceG,s.sourceP,"event=FLUSH_REQUEST session=%llu request=%llu tick=%llu workUs=%llu phase=%s layer=%u candidateX=%.9f candidateY=%.9f candidateZ=%.9f time=SIMULATED_WORK candidateOnly=1 stoppedProof=0 motionApplied=0 discharge=0\n",
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
        PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu start=%llu reason=POSITION_REQUALIFICATION freshStartRequired=1 discharge=0\n",
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
        PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu start=%llu reason=RECOVERY_PAUSED freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.startSequence));
    }
    if(s.resumePending&&observed.session==s.session&&cycle.session==s.session&&
        SameIdentity(observed.identity,s.identity)&&observed.owner.Matches(s.owner)&&
        !observed.runtimeFault&&cycle.fault==NCG76SphereCycle::Fault::None&&
        observed.cycle.resumeSequence==s.startSequence&&!observed.cycle.holdActive&&
        !observed.cycle.held&&observed.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        s.resumePending=false;m_state=NCState::RUN;
        PrintParent(s.sourceG,s.sourceP,"event=RESUMED session=%llu hold=%llu start=%llu tick=%llu nominalProgressMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f discharge=0\n",
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
            PrintParent(s.sourceG,s.sourceP,"event=HELD session=%llu hold=%llu tick=%llu nominalProgressMm=%.9f commandX=%.9f commandY=%.9f commandZ=%.9f actualX=%.9f actualY=%.9f actualZ=%.9f path=%s freshStartRequired=1 discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(cycle.holdSequence),
                static_cast<unsigned long long>(cycle.tick),cycle.nominalProgressMm,
                observed.commandMCS[0],observed.commandMCS[1],observed.commandMCS[2],
                observed.actualMCS[0],observed.actualMCS[1],observed.actualMCS[2],
                cycle.holdRecoveryEnabled?"RETAINED_PROCESS_PATH":"IN_PLACE");
        if(s.diagnosticEvents++<512U||observed.terminal||observed.cycle.held){
            PrintParent(s.sourceG,s.sourceP,"event=STATE session=%llu tick=%llu phase=%s layer=%u finishedLayers=%u progress=%.9f held=%u heldRequal=%llu xyzTol=%.9f rotaryTol=%.9f flushActive=%u request=%llu flushes=%llu discharge=0\n",
                static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(observed.cycle.tick),
                PhaseName(observed.cycle.phase,s.sourceG),observed.cycle.layerIndex,observed.cycle.completedLayers,
                observed.cycle.nominalProgressMm,observed.cycle.held?1U:0U,
                static_cast<unsigned long long>(cycle.heldRequalificationSequence),observed.positionToleranceMm,observed.rotaryToleranceDeg,cycle.flushActive?1U:0U,
                static_cast<unsigned long long>(cycle.flushRequestSequence),static_cast<unsigned long long>(cycle.flushCount));
            if(cycle.holdRecoveryEnabled&&cycle.holdActive)
                PrintParent(s.sourceG,s.sourceP,"event=HOLD_ROUTE session=%llu hold=%llu stage=%u declared=%u effective=%u available=%u jog=%u trace=%u anchorX=%.9f anchorY=%.9f anchorZ=%.9f processX=%.9f processY=%.9f processZ=%.9f E5=SUSPENDED discharge=0\n",
                    static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(cycle.holdSequence),
                    static_cast<unsigned>(cycle.holdRecoveryPhase),cycle.holdDeclaredCount,cycle.holdEffectiveCount,
                    cycle.holdAvailableCount,cycle.inspectionJogActive?1U:0U,cycle.inspectionTraceCount,
                    observed.holdRetreatAnchorMCS[0],observed.holdRetreatAnchorMCS[1],observed.holdRetreatAnchorMCS[2],
                    observed.holdProcessPointMCS[0],observed.holdProcessPointMCS[1],observed.holdProcessPointMCS[2]);
            if(cycle.gapHistoryActive||cycle.historyBoundarySequence||cycle.historyReturnSequence)
                PrintParent(s.sourceG,s.sourceP,"event=HISTORY_STATE session=%llu tick=%llu phase=%s active=%u boundary=%llu returned=%llu nominal=%s layer=%u X=%.9f Y=%.9f Z=%.9f scope=%s depth=1 previousProgressMm=%.9f previousLengthMm=%.9f E5=SUSPENDED_WHILE_ACTIVE discharge=0\n",
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
            PrintParent(s.sourceG,s.sourceP,"event=PROCESS_CURSOR session=%llu valid=%u phase=%s role=%u stage=%u layer=%u radiusMm=%.9f depthMm=%.9f stationMm=%.9f saved=%u atNominal=%u gap=0 discharge=0\n",
                static_cast<unsigned long long>(s.session),p.valid?1U:0U,PhaseName(observed.cycle.phase,s.sourceG),
                static_cast<unsigned>(p.role),static_cast<unsigned>(p.nominal.cursor.span.stage),
                p.nominal.cursor.span.layerIndex,p.layerRadiusMm,p.layerDepthMm,p.nominal.cursor.stationMm,
                p.checkpointSaved?1U:0U,p.atNominalPath?1U:0U);
            if(cycle.flushActive&&(!cycle.holdRecoveryEnabled||!cycle.holdActive)){
                const bool finishFlush=cycle.flushOrigin==2U;
                const auto diagnosticSequence=finishFlush?cycle.finishAttemptCount:cycle.flushRequestSequence;
                PrintParent(s.sourceG,s.sourceP,"event=%s session=%llu request=%llu B=%u phase=%s leg=%llu stationMm=%.9f anchorStationMm=%.9f checkpoint=%u E5=SUSPENDED scope=SAME_PARENT discharge=0\n",
                    finishFlush?"FINISH_FLUSH_ROUTE":"FLUSH_ROUTE",static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(diagnosticSequence),
                    s.request.retreatMode,PhaseName(cycle.phase,s.sourceG),static_cast<unsigned long long>(cycle.flushLegSequence),
                    cycle.flushPathStationMm,cycle.flushAnchorStationMm,cycle.flushCheckpointSaved?1U:0U);
                PrintParent(s.sourceG,s.sourceP,"event=%s session=%llu request=%llu B=%u phase=%s leg=%llu stageSpeedMmMin=%.6f selectedMmMin=%.6f targetMmMin=%.6f remainingMm=%.9f approachActive=%u initialSlowActive=%u E5=SUSPENDED discharge=0\n",
                    finishFlush?"FINISH_FLUSH_SPEED":"FLUSH_SPEED",static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(diagnosticSequence),
                    s.request.retreatMode,PhaseName(cycle.phase,s.sourceG),static_cast<unsigned long long>(cycle.flushLegSequence),
                    cycle.speedMmS*60.0,cycle.flushSelectedFeedMmMin,cycle.flushTargetFeedMmMin,
                    cycle.flushRemainingMm,cycle.flushApproachActive?1U:0U,cycle.flushInitialSlowActive?1U:0U);
                PrintParent(s.sourceG,s.sourceP,"event=%s session=%llu request=%llu phase=%s X=%.9f Y=%.9f Z=%.9f nominalX=%.9f nominalY=%.9f nominalZ=%.9f checkpoint=%u discharge=0\n",
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
    const bool timed=MotionG76SphereMachiningTime::TimedCompletionQualified(s.feedback.cycle,s.request);
    if(!s.feedback.completed||s.feedback.cycle.phase!=NCG76SphereCycle::Phase::Complete||
        (!timed&&s.feedback.cycle.completedLayers!=s.request.geometry.circleLayerCount)||
        !MotionG76SphereMachiningTime::CompletedEndpointQualified(s.feedback,s.request)){
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
    if(!OrbitTerminalQualified(s.feedback.cycle,s.request)||
        !FinishTerminalQualified(s.feedback.cycle,s.finishRequiredCount)||
       !MotionG76SphereMachiningTime::TerminalQualified(s.feedback.cycle,s.request)){
        RejectG76SphereSameThread("FINISH_TERMINAL_NOT_QUALIFIED",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
    }
    if(!s.endpointCommitted){
        if(!m_motion.CommitG76SphereCompletedEndpoint(s.session,s.identity,s.owner,s.endpoint))return false;
        for(unsigned a=0U;a<8U;++a)if(!SameDouble(s.endpoint[a],s.feedback.endpointMCS[a])){
            RejectG76SphereSameThread("TERMINAL_ENDPOINT",AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);return false;
        }
        for(unsigned a=0U;a<8U;++a)CoordSys.commandedMCS[a]=s.endpoint[a];s.endpointCommitted=true;
    }
    if(!m_motion.ReleaseG76SphereBarrier(s.session,s.identity,s.owner))return false;
    s.active=false;s.completed=true;s.submitted=false;UpdateSystemVariables();
    const bool exhausted=MotionG76SphereMachiningTime::RetryExhaustionQualified(s.feedback.cycle,s.request);
    RecordDHTerminal(s,exhausted?NCG76DHBatchDiagnostic::Result::RetryExhausted:
        timed?NCG76DHBatchDiagnostic::Result::TimedComplete:NCG76DHBatchDiagnostic::Result::Complete);
    if(exhausted)PrintParent(s.sourceG,s.sourceP,"event=ORBIT_COMPLETION_EXHAUSTED session=%llu retryLimit=%u failedAttempts=%llu attempts=%llu percent=%.6f targetPercent=%.6f requiredPasses=%u qualifiedPasses=%u processResult=UNQUALIFIED endpointCommitted=1 continueNext=1 alarm=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned>(s.request.orbitRetryLimit),
        static_cast<unsigned long long>(s.feedback.cycle.orbitEvaluation.failedAttempts),
        static_cast<unsigned long long>(s.feedback.cycle.orbitAttemptCount),
        s.feedback.cycle.orbitPercentScaled/1000000.0,s.request.orbitThresholdScaled/1000000.0,
        static_cast<unsigned>(s.request.orbitInspectionRounds),s.feedback.cycle.orbitQualifiedPasses);
    if(timed)PrintParent(s.sourceG,s.sourceP,"event=D_COMPLETE session=%llu totalElapsedUs=%llu totalLimitUs=%llu X=%.9f Y=%.9f Z=%.9f returnedCenter=%u endpointCommitted=1 Hfulfilled=%u Lfulfilled=%u alarm=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),static_cast<unsigned long long>(s.feedback.cycle.totalElapsedUs),
        static_cast<unsigned long long>(s.request.totalLimitUs),s.endpoint[0],s.endpoint[1],s.endpoint[2],s.sourceG==76?1U:0U,
        s.feedback.cycle.additionalElapsedUs==s.request.additionalUs?1U:0U,
        s.feedback.cycle.orbitQualifiedPasses==s.request.orbitInspectionRounds?1U:0U);
    if(!timed&&!exhausted&&s.request.orbitInspectionRounds)
        PrintParent(s.sourceG,s.sourceP,"event=ORBIT_COMPLETION_COMPLETE build=EDM355 session=%llu requiredPasses=%u qualifiedPasses=%u attempts=%llu percent=%.6f targetPercent=%.6f endpointCommitted=1 source=SOFTWARE_SIMULATION discharge=0\n",
            static_cast<unsigned long long>(s.session),static_cast<unsigned>(s.request.orbitInspectionRounds),
            s.feedback.cycle.orbitQualifiedPasses,static_cast<unsigned long long>(s.feedback.cycle.orbitAttemptCount),
            s.feedback.cycle.orbitPercentScaled/1000000.0,s.request.orbitThresholdScaled/1000000.0);
    if(exhausted){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=%u X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 processResult=UNQUALIFIED retryExhausted=1 returnedCenter=1 alarm=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceP,s.endpoint[0],s.endpoint[1],s.endpoint[2]);
    }else if(s.sourceG!=76){
        if(!timed&&s.finishRequiredCount)PrintParent(s.sourceG,s.sourceP,"event=FINISH_COMPLETE session=%llu required=%u verified=%u attempts=%llu forcedCompleted=%llu reworks=%llu endpointCommitted=1 countScope=THIS_PARENT discharge=0\n",
            static_cast<unsigned long long>(s.session),s.finishRequiredCount,s.feedback.cycle.finishVerifiedCount,
            static_cast<unsigned long long>(s.feedback.cycle.finishAttemptCount),
            static_cast<unsigned long long>(s.feedback.cycle.finishForcedCompletedCount),
            static_cast<unsigned long long>(s.feedback.cycle.finishReworkCount));
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu g=%d X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu completedFlushOffset=%llu historyEntered=%llu historyReturned=%llu simProbe=%s countScope=THIS_PARENT discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceG,s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            static_cast<unsigned long long>(s.feedback.completedFlushOffset),
            static_cast<unsigned long long>(s.feedback.cycle.historyBoundarySequence),
            static_cast<unsigned long long>(s.feedback.cycle.historyReturnSequence),
            NCG76SphereGapProcess::ProbePhaseName(s.feedback.processShadow.probePhase));
    }else if(NCG76Mode::IsFixedRadius(s.sourceP)){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=%u Q=%u nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_FULL_CONTOUR completion=SOURCE_L depthReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.sourceP,s.request.geometry.orbit.source.polygonSides,
            s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==27U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=27 Q=%u nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.orbit.source.polygonSides,
            s.endpoint[0],s.endpoint[1],s.endpoint[2],static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            s.request.geometry.orbit.source.polygonSides);
    }else if(s.sourceP==26U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=26 Q=%u nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_POLYGON sideLegs=%u corner=EXACT_STOP plane=TARGET_NORMAL depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.orbit.source.polygonSides,
            s.endpoint[0],s.endpoint[1],s.endpoint[2],static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            s.request.geometry.orbit.source.polygonSides);
    }else if(s.sourceP==25U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=25 nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_DIAMOND sideLegs=4 corner=EXACT_STOP vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==24U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=24 nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_SQUARE sideLegs=4 corner=EXACT_STOP vector=COMPLETE_CONTOUR directionalClipping=0 depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==23U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=23 nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_DIAMOND sideLegs=4 corner=EXACT_STOP depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==22U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=22 nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_SQUARE sideLegs=4 corner=EXACT_STOP depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==21U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=21 nominalLegs=26 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_ANALYTIC_CIRCLE depthGrowReplay=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],
            s.endpoint[2],static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==17U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=17 Q=%u nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_POLYGON sideLegs=%u corner=EXACT_STOP vector=COMPLETE_CONTOUR directionalClipping=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.orbit.source.polygonSides,
            s.endpoint[0],s.endpoint[1],s.endpoint[2],static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            s.request.geometry.orbit.source.polygonSides);
    }else if(s.sourceP==16U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=16 Q=%u nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_POLYGON sideLegs=%u corner=EXACT_STOP discharge=0\n",
            static_cast<unsigned long long>(s.session),s.request.geometry.orbit.source.polygonSides,
            s.endpoint[0],s.endpoint[1],s.endpoint[2],static_cast<unsigned long long>(s.feedback.cycle.flushCount),
            s.request.geometry.orbit.source.polygonSides);
    }else if(s.sourceP==15U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=15 nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_DIAMOND sideLegs=4 corner=EXACT_STOP vector=COMPLETE_CONTOUR directionalClipping=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==14U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=14 nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_VECTOR_SQUARE sideLegs=4 corner=EXACT_STOP vector=COMPLETE_CONTOUR directionalClipping=0 discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==13U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=13 nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_DIAMOND sideLegs=4 corner=EXACT_STOP discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==12U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=12 nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_SQUARE sideLegs=4 corner=EXACT_STOP discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else if(s.sourceP==11U){
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu P=11 nominalLegs=25 X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu additional=TERMINAL_ANALYTIC_CIRCLE discharge=0\n",
            static_cast<unsigned long long>(s.session),s.endpoint[0],s.endpoint[1],s.endpoint[2],
            static_cast<unsigned long long>(s.feedback.cycle.flushCount));
    }else{
        PrintParent(s.sourceG,s.sourceP,"event=COMPLETE session=%llu layers=%u X=%.9f Y=%.9f Z=%.9f endpointCommitted=1 flushes=%llu discharge=0\n",
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
        PrintParent(s.sourceG,s.sourceP,"event=HOLD_AT_COMPLETED_PARENT session=%llu discharge=0\n",
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
    PrintParent(s.sourceG,s.sourceP,"event=HOLD_REQUEST session=%llu hold=%llu path=%s discharge=0\n",
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
        PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu reason=INSPECTION_RECOVERY_NOT_READY freshStartRequired=1 discharge=0\n",
            static_cast<unsigned long long>(s.session));return true;
    }
    if(m_state!=NCState::HOLD||s.resumePending||!s.feedback.started||
        s.feedback.holdSequence!=s.holdSequence||s.feedback.cycle.holdSequence!=s.holdSequence||
        !s.feedback.cycle.held||
        s.feedback.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart){
        PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu reason=%s freshStartRequired=1 discharge=0\n",
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
            PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu reason=POSITION_REQUALIFICATION freshStartRequired=1 discharge=0\n",
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
            PrintParent(s.sourceG,s.sourceP,"event=START_CONSUMED session=%llu reason=RECOVERY_INTENT_NOT_ADMITTED freshStartRequired=1 discharge=0\n",
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
    const bool deadlineAlreadyStopped=reportFailure&&
        observed.translationGeneration==s.request.geometry.frame.translation.generation&&
        MotionG76SphereMachiningTime::TimeLimitStoppedQualified(observed,s.request);
    if(submitted&&!reset&&!deadlineAlreadyStopped)m_motion.RequestEmergencyStopAllAxes();
    if(reportFailure)PrintSphereRuntimeFailure(observed,s.request,s.line,s.sourceG,s.sourceP);
    PrintParent(s.sourceG,s.sourceP,"event=CANCEL session=%llu reason=%s observerWithdraw=%s automaticReturn=0 discharge=0\n",
        static_cast<unsigned long long>(s.session),reason?reason:"UNSPECIFIED",
        reset?"EXACT_RESET_OUTPUT_HOLD":"NC_CONTAINMENT");
}


// Called only after the original program-end gate has finalized successfully.
void NCManager::FinalizeG76OrbitCompletionSummarySameThread() noexcept
{
    if(!m_g76Sphere)return;
    const auto end=m_programEndBoundary.GetSnapshot();
    const auto& batch=m_g76Sphere->dhBatch;const auto& scope=batch.scope;
    if(!end.IsValid()||end.phase!=NCProgramEndPhase::FINALIZED||end.failClosed||
       !NCG76DHBatchDiagnostic::Valid(scope)||scope.run!=end.run.runId||
       scope.cache!=end.run.cacheGeneration||scope.epoch!=end.requestExecutionEpoch||
       scope.owner!=static_cast<std::uint32_t>(end.requestOwner)||scope.ownerGeneration!=end.requestOwnerGeneration)return;
    PrintOrbitRunSummary(batch,"PROGRAM_END");
}
