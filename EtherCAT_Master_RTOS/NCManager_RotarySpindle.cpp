#include "NCManager.h"
#include "NCSpindleSource.h"
#include "NCRotarySpindle.h"
#include "NCSpindleDispatch.h"
#include "MotionCore_RotarySpindle.h"
#include "AlarmManager.h"
#include "GMCodeHandlers.h"
#include <cmath>
#include <cstring>
#include <windows.h>
#include <rtapi.h>

namespace {
using SpindleState=NCSpindleDispatch::State;
// RTX RtPrintf does not support floating conversions. Diagnostics use signed
// milli-units, rounded to the nearest integer and saturated before conversion.
// INT64_MIN denotes a nonfinite input (not zero). These bounded display values
// never participate in admission or commands.
long long SpindleMilli(double value) noexcept
{
    if(!std::isfinite(value))return (std::numeric_limits<long long>::min)();
    constexpr double bound=9000000000000.0;
    if(value>=bound)return 9000000000000000LL;
    if(value<=-bound)return -9000000000000000LL;
    return static_cast<long long>(std::round(value*1000.0));
}

template<class Motion>
void PrintSpindleCapabilities(const RotarySpindleFile::Settings& settings,Motion& motion,const char* result,int line) noexcept
{
    // At most three records at admission/rejection, never in the RT loop or
    // normal per-scan scope check. The same conversion helper owns both the
    // reported cap and BuildRuntime's effective limit.
    for(unsigned i=0U;i<RotarySpindleFile::SpindleCount;++i){
        const auto& configured=settings.spindle[i];if(!configured.enabled)continue;
        RotarySpindleFile::AxisFacts facts{};facts.axisSlot=RotarySpindleFile::AxisCapacity;
        if(configured.axisNumber>=1U&&configured.axisNumber<=RotarySpindleFile::AxisCapacity){
            const auto& axis=motion.GetAxisContext(static_cast<int>(configured.axisNumber-1U));
            facts.axisSlot=axis.axisIndex<0?RotarySpindleFile::AxisCapacity:static_cast<unsigned>(axis.axisIndex);
            facts.exists=axis.isExist;facts.rotary=axis.axisType==AxisType::ROTARY||axis.axisType==AxisType::ROTARY_CONTINUOUS;
            facts.pulsesPerNative=std::isfinite(axis.finalLead)&&axis.finalLead>0.0?axis.resolution_PPR/axis.finalLead:0.0;
            facts.nativeUnitsPerOutputRevolution=360.0;facts.maximumOutputPps=axis.maxVel_PPS;
        }
        RotarySpindleFile::Capability cap{};RotarySpindleFile::Error error{};
        const bool valid=RotarySpindleFile::ComputeCapability(configured,facts,cap,error);
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=CAPABILITY result=%s line=%d spindle=%u axisNumber=%u capValid=%u configuredRPM_milli=%lld axisRPM_milli=%lld pdoRPM_milli=%lld effectiveRPM_milli=%lld axisMaxPPS_milli=%lld allowedPPS_milli=%lld pulsesPerNative_milli=%lld pulsesPerRev_milli=%lld reason=%s\n",
            result,line,i+1U,configured.axisNumber,valid?1U:0U,SpindleMilli(configured.maximumRpm),
            SpindleMilli(cap.axisRpm),SpindleMilli(cap.pdoRpm),SpindleMilli(cap.effectiveRpm),
            SpindleMilli(facts.maximumOutputPps),SpindleMilli(cap.maximumOutputPps),
            SpindleMilli(facts.pulsesPerNative),SpindleMilli(cap.pulsesPerOutputRevolution),RotarySpindleFile::ErrorName(error.code));
    }
}
bool TerminalSpindleState(SpindleState state) noexcept
{return state==SpindleState::End||state==SpindleState::AuthorityLost||state==SpindleState::Alarm||state==SpindleState::Reset;}
bool SpindleProgramRow(const NCParsedBlock& row) noexcept
{
    if(row.error!=NCParseError::NONE||row.isBlockSkip||row.dependsOnMacroState||row.duplicateAddressMask||
        row.controlType!=NCParsedControlType::NONE||row.assignment.prefix||!row.assignment.indexExpression.empty()||
        !row.assignment.valueExpression.empty()||row.gotoStatement.conditional||
        !row.gotoStatement.conditionExpression.empty()||!row.gotoStatement.targetExpression.empty())return false;
    if(NCSpindleSource::HasReservedM(row)){
        NCSpindleSource::Words words{};return NCSpindleSource::Decode(row,words)==NCSpindleSource::Status::Ok;
    }
    int m=-1,g=-1,n=0;
    if(row.gCount<0||row.gCount>1||row.mCount<0||row.mCount>1||(row.gCount&&row.mCount))return false;
    for(unsigned i=0;i<row.gExpressions.size();++i)
        if(i>=static_cast<unsigned>(row.gCount)&&!row.gExpressions[i].empty())return false;
    for(unsigned i=0;i<row.mExpressions.size();++i)
        if(i>=static_cast<unsigned>(row.mCount)&&!row.mExpressions[i].empty())return false;
    if(row.gCount&&(!NCSpindleSource::Detail::Integer(row.gExpressions[0],g)||g!=4))return false;
    if(row.mCount&&(!NCSpindleSource::Detail::Integer(row.mExpressions[0],m)||(m!=0&&m!=1&&m!=2&&m!=30)))return false;
    for(char word='A';word<='Z';++word){
        if(!row.has(word)){if(!row.expression(word).empty())return false;continue;}
        if(word=='N'){if(!NCSpindleSource::Detail::Integer(row.expression(word),n))return false;}
        else if(g==4&&(word=='X'||word=='P')){
            double duration=0.0;if(!NCSpindleSource::Detail::Literal(row.expression(word),duration)||duration<0.0)return false;
        }else return false;
    }
    return !(g==4&&row.has('X')&&row.has('P'));
}
bool SpindleResolvedRow(const NCParsedBlock& row,const NCBlock& block) noexcept
{
    if(!SpindleProgramRow(row))return false;
    NCBlock expected{};expected.isEmpty=row.isEmpty;
    if(row.gCount){int g=0;if(!NCSpindleSource::Detail::Integer(row.gExpressions[0],g))return false;
        expected.hasG=true;expected.gCount=1;expected.gCode=g;expected.gCodes[0]=g;}
    if(row.mCount){int m=0;if(!NCSpindleSource::Detail::Integer(row.mExpressions[0],m))return false;
        expected.mCount=1;expected.mCode[0]=m;}
    for(char word='A';word<='Z';++word)if(row.has(word)){
        double value=0.0;if(!NCSpindleSource::Detail::Literal(row.expression(word),value))return false;
        expected.hasParam[word-'A']=true;expected.param[word-'A']=value;}
    if(block.isEmpty!=expected.isEmpty||block.isBlockSkip||block.isGoto||block.gotoTarget!=expected.gotoTarget||
        block.hasG!=expected.hasG||block.gCount!=expected.gCount||block.gCode!=expected.gCode||block.mCount!=expected.mCount)return false;
    for(unsigned i=0;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]!=expected.gCodes[i])return false;
    for(unsigned i=0;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(block.mCode[i]!=expected.mCode[i])return false;
    for(unsigned i=0;i<26;++i)if(block.hasParam[i]!=expected.hasParam[i]||std::memcmp(&block.param[i],&expected.param[i],sizeof(double)))return false;
    return true;
}
std::uint64_t SpindleConfigReceipt(const NCSpindleDispatch::Config& config) noexcept
{
    std::uint64_t hash=1469598103934665603ULL;
    const auto add=[&](std::uint64_t value){hash^=value;hash*=1099511628211ULL;};
    const auto number=[&](double value){std::uint64_t bits=0ULL;std::memcpy(&bits,&value,sizeof(bits));add(bits);};
    for(const auto& c:config){add(c.axisSlot);add(c.enabled?1U:0U);add(c.rotary?1U:0U);
        number(c.maximumRpm);number(c.accelerationRpmPerSecond);number(c.decelerationRpmPerSecond);
        number(c.stopDecelerationRpmPerSecond);number(c.pulsesPerOutputRevolution);
        number(c.stoppedToleranceRpm);number(c.readyToleranceRpm);}
    return hash?hash:1ULL;
}
}

void NCManager::LoadRotarySpindleParametersBeforeStart(const std::string& path) noexcept
{
    // Startup only. Config/source/model flags describe pure helpers; physical
    // capability belongs to the separately validated Motion RT/PDO adapter.
    static_assert(MotionRotarySpindle::RuntimeConnected,"EDM289 requires the physical spindle adapter.");
    m_rotarySpindleParameterStatus=RotarySpindleFile::LoadOrCreateDefault(path,
        m_rotarySpindleParameters,m_rotarySpindleParameterError);
    const auto status=m_rotarySpindleParameterStatus;
    const char* const result=status==RotarySpindleFile::LoadStatus::Loaded?"LOADED":
        status==RotarySpindleFile::LoadStatus::CreatedDisabled?"CREATED_DISABLED":
        status==RotarySpindleFile::LoadStatus::Invalid?"INVALID":"IO_ERROR";
    RtPrintf("[SPINDLE] build=EDM289_FIX1 event=CONFIG result=%s line=%u spindle=%u key=%s reason=%s runtime=PHYSICAL_ADAPTER\n",
        result,m_rotarySpindleParameterError.line,m_rotarySpindleParameterError.spindle,
        m_rotarySpindleParameterError.key,RotarySpindleFile::ErrorName(m_rotarySpindleParameterError.code));
    if(status!=RotarySpindleFile::LoadStatus::Loaded&&status!=RotarySpindleFile::LoadStatus::CreatedDisabled)return;
    ++m_rotarySpindleParameterGeneration;
    for(unsigned i=0U;i<RotarySpindleFile::SpindleCount;++i){const auto& config=m_rotarySpindleParameters.spindle[i];
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=CONFIG_AXIS spindle=%u axisNumber=%u enabled=%u maxRPM_milli=%lld accelSec_milli=%lld decelSec_milli=%lld stopSec_milli=%lld stopToleranceRPM_milli=%lld readyToleranceRPM_milli=%lld\n",
            i+1U,config.axisNumber,config.enabled?1U:0U,SpindleMilli(config.maximumRpm),
            SpindleMilli(config.accelTimeSeconds),SpindleMilli(config.decelTimeSeconds),SpindleMilli(config.stopTimeSeconds),
            SpindleMilli(config.stoppedToleranceRpm),SpindleMilli(config.readyToleranceRpm));}
}

void NCManager::RejectRotarySpindleSameThread(const char* reason,int line,int alarm) noexcept
{
    AlarmManager::GetInstance().Trigger(alarm?alarm:AlarmManager::G_Code_Invalid_parameter,line);
    CancelRotarySpindleSameThread(SpindleState::Alarm,reason);
    ChangeState(NCState::ALARM);
    PrintSpindleCapabilities(m_rotarySpindleParameters,m_motion,"REJECT",line);
    RtPrintf("[SPINDLE] build=EDM289_FIX1 event=REJECT line=%d reason=%s\n",line,reason);
}

bool NCManager::ValidateRotarySpindleParsedScopeSameThread(const NCParsedBlock& parsed,int line) noexcept
{
    if(!m_rotarySpindle.active||SpindleProgramRow(parsed))return true;
    RejectRotarySpindleSameThread("SPINDLE_PROGRAM_ONLY",line);return false;
}

bool NCManager::ValidateRotarySpindleSourceSameThread(const NCBlock& block,
    int sourcePC,int sourceLineNumber,NCSpindleSource::Words& words) noexcept
{
    words={};
    const auto* source=m_mode==NCOperationMode::MEMORY&&m_macroStack.empty()?
        GetBaseProgramCache().TryGetLine(sourcePC):nullptr;
    if(m_rotarySpindle.active&&(!source||!ValidateRotarySpindleParsedScopeSameThread(source->parsedBlock,sourceLineNumber))){
        if(!source)RejectRotarySpindleSameThread("SPINDLE_SCOPE_SOURCE",sourceLineNumber);return false;}
    if(m_rotarySpindle.active&&source&&!SpindleResolvedRow(source->parsedBlock,block)){
        RejectRotarySpindleSameThread("SPINDLE_RESOLVED_SCOPE",sourceLineNumber);return false;}
    const bool reserved=NCSpindleSource::HasReservedM(block)||(source&&NCSpindleSource::HasReservedM(source->parsedBlock));
    const bool speedOnly=NCSpindleSource::IsStandaloneSpeed(block)||(source&&NCSpindleSource::IsStandaloneSpeed(source->parsedBlock));
    if(!reserved&&!speedOnly)return true;
    if(!reserved){RejectRotarySpindleSameThread("S_REQUIRES_SPINDLE_M",sourceLineNumber);return false;}
    if(m_mode!=NCOperationMode::MEMORY||m_state!=NCState::RUN||!m_macroStack.empty()||m_isG66Active||
        !source||source->sourceLineNumber!=sourceLineNumber){
        RejectRotarySpindleSameThread("MEMORY_LITERAL_SOURCE_REQUIRED",sourceLineNumber);return false;}
    const auto decoded=NCSpindleSource::ValidateResolved(source->parsedBlock,block,words);
    if(decoded!=NCSpindleSource::Status::Ok){
        RejectRotarySpindleSameThread(decoded==NCSpindleSource::Status::InvalidSpeed?"EXPLICIT_POSITIVE_S_REQUIRED":
            "EXACT_STANDALONE_M_SOURCE_REQUIRED",sourceLineNumber);return false;}
    return true;
}

bool NCManager::BuildRotarySpindleRuntimeSameThread(NCSpindleDispatch::Config& runtime,RotarySpindleFile::Error& error) noexcept
{
    if(m_rotarySpindleParameterStatus!=RotarySpindleFile::LoadStatus::Loaded&&
        m_rotarySpindleParameterStatus!=RotarySpindleFile::LoadStatus::CreatedDisabled)return false;
    std::array<RotarySpindleFile::AxisFacts,RotarySpindleFile::AxisCapacity> axes{};
    for(unsigned i=0U;i<axes.size();++i){const auto& axis=m_motion.GetAxisContext(static_cast<int>(i));auto& facts=axes[i];
        facts.axisSlot=axis.axisIndex<0?RotarySpindleFile::AxisCapacity:static_cast<unsigned>(axis.axisIndex);
        facts.exists=axis.isExist;facts.rotary=axis.axisType==AxisType::ROTARY||axis.axisType==AxisType::ROTARY_CONTINUOUS;
        facts.pulsesPerNative=std::isfinite(axis.finalLead)&&axis.finalLead>0.0?axis.resolution_PPR/axis.finalLead:0.0;
        facts.nativeUnitsPerOutputRevolution=360.0;facts.maximumOutputPps=axis.maxVel_PPS;}
    return RotarySpindleFile::BuildRuntime(m_rotarySpindleParameters,axes,runtime,error);
}

bool NCManager::IsRotarySpindleScopeCurrentSameThread() noexcept
{
    if(!m_rotarySpindle.active||m_mode!=NCOperationMode::MEMORY||!m_macroStack.empty()||m_isG66Active||m_isSingleBlockEnabled||
        !m_programMotionLease.IsValid()||m_programMotionLease.owner!=MotionOwner::AUTO||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)||m_motion.HasPendingSafetyOrRecoveryRequests())return false;
    const auto& a=m_rotarySpindle.authority;
    if(a.scope.runToken!=m_pathCoreLiveBookkeeping.currentRunToken||a.scope.epoch!=m_motion.GetCurrentExecutionEpoch()||
        a.scope.ownerGeneration!=m_programMotionLease.generation||a.cacheGeneration!=GetBaseProgramCache().GetGeneration()||
        a.cacheIdentity!=static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(&GetBaseProgramCache()))||
        a.configGeneration!=m_rotarySpindleParameterGeneration)return false;
    NCSpindleDispatch::Config config{};RotarySpindleFile::Error error{};
    return BuildRotarySpindleRuntimeSameThread(config,error)&&SpindleConfigReceipt(config)==a.configReceipt;
}

bool NCManager::ValidateRotarySpindleBlockSameThread(const NCBlock& block,int sourcePC,int sourceLineNumber) noexcept
{
    NCSpindleSource::Words words{};
    if(!ValidateRotarySpindleSourceSameThread(block,sourcePC,sourceLineNumber,words))return false;
    if(!words.valid)return true;
    if(m_isSingleBlockEnabled){RejectRotarySpindleSameThread("SPINDLE_SINGLE_BLOCK_UNSUPPORTED",sourceLineNumber);return false;}
    if(!m_currentExecutingBlockDispatchId||!m_pathCoreLiveBookkeeping.currentRunToken||
        !GetBaseProgramCache().GetGeneration()||!m_programMotionLease.IsValid()||m_programMotionLease.owner!=MotionOwner::AUTO||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)||!m_motion.GetCurrentExecutionEpoch()||
        m_motion.HasPendingSafetyOrRecoveryRequests()||AlarmManager::GetInstance().HasAlarm()||CoordSys.IsTranslationRunFrozen()){
        RejectRotarySpindleSameThread("PROGRAM_AUTHORITY",sourceLineNumber,AlarmManager::PATH_EXECUTION_NOT_READY);return false;}
    NCSpindleDispatch::Config config{};RotarySpindleFile::Error error{};
    if(!m_rotarySpindleParameterGeneration||!BuildRotarySpindleRuntimeSameThread(config,error)){
        CancelRotarySpindleSameThread(SpindleState::Alarm,"CONFIG_REJECT");
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=CONFIG_REJECT line=%d spindle=%u key=%s code=%u reason=%s loadStatus=%u\n",
            sourceLineNumber,error.spindle,error.key,static_cast<unsigned>(error.code),
            RotarySpindleFile::ErrorName(error.code),static_cast<unsigned>(m_rotarySpindleParameterStatus));
        RejectRotarySpindleSameThread(RotarySpindleFile::ErrorName(error.code),sourceLineNumber);return false;}
    const auto admitWords=[&](const NCSpindleSource::Words& request,int line){
        const auto& limit=config[request.spindle];
        const char* reason=!limit.enabled?"SPINDLE_DISABLED":
            !request.stop&&request.rpm>limit.maximumRpm?"S_EXCEEDS_MAX_RPM":nullptr;
        if(!reason)return true;
        RejectRotarySpindleSameThread(reason,line);
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=REQUEST_REJECT line=%d M=%d spindle=%u enabled=%u requestedRPM_milli=%lld effectiveRPM_milli=%lld reason=%s\n",
            line,request.mCode,request.spindle+1U,limit.enabled?1U:0U,SpindleMilli(request.rpm),
            SpindleMilli(limit.enabled?limit.maximumRpm:0.0),reason);
        return false;
    };
    if(!admitWords(words,sourceLineNumber))return false;
    if(m_rotarySpindle.active){
        if(!IsRotarySpindleScopeCurrentSameThread()||m_rotarySpindle.broker.CurrentState()!=SpindleState::Run){
            RejectRotarySpindleSameThread("SPINDLE_SCOPE_CHANGED",sourceLineNumber);return false;}
        return true;
    }
    // Bound the NC-owner preflight and reject every unsupported later row
    // before the first spindle command can reach RT. No concurrent XYZ here.
    const auto& program=GetBaseProgramCache();
    if(program.Size()>256U){RejectRotarySpindleSameThread("SPINDLE_PROGRAM_LIMIT_256",sourceLineNumber);return false;}
    for(std::size_t i=0;i<program.Size();++i){const auto* row=program.TryGetLine(static_cast<int>(i));
        if(!row||!SpindleProgramRow(row->parsedBlock)){
            RejectRotarySpindleSameThread("SPINDLE_PROGRAM_ONLY",row?row->sourceLineNumber:sourceLineNumber);return false;}
        if(NCSpindleSource::HasReservedM(row->parsedBlock)){
            NCSpindleSource::Words request{};
            if(NCSpindleSource::Decode(row->parsedBlock,request)!=NCSpindleSource::Status::Ok){
                RejectRotarySpindleSameThread("EXACT_STANDALONE_M_SOURCE_REQUIRED",row->sourceLineNumber);return false;}
            // Validate every future S/map before this run can claim an axis.
            // An over-cap S is rejected unchanged, never reduced to the cap.
            if(!admitWords(request,row->sourceLineNumber))return false;
        }
    }
    NCSpindleDispatch::Authority authority{};
    authority.scope.runToken=m_pathCoreLiveBookkeeping.currentRunToken;
    authority.scope.epoch=m_motion.GetCurrentExecutionEpoch();authority.scope.ownerGeneration=m_programMotionLease.generation;
    authority.cacheIdentity=static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(&GetBaseProgramCache()));
    authority.cacheGeneration=program.GetGeneration();authority.configGeneration=m_rotarySpindleParameterGeneration;
    authority.configReceipt=SpindleConfigReceipt(config);authority.memoryProgram=authority.autoOwner=true;
    if(m_rotarySpindle.broker.Begin(authority,config,m_startDiagnosticRequests)!=NCSpindleDispatch::Status::Ok){
        RejectRotarySpindleSameThread("SPINDLE_PREVIOUS_SESSION_PENDING",sourceLineNumber);return false;}
    m_rotarySpindle.authority=authority;m_rotarySpindle.config=config;m_rotarySpindle.submitted={};
    m_rotarySpindle.active=true;m_rotarySpindle.rtBegun=false;
    m_rotarySpindle.waitActive=m_rotarySpindle.waitComplete=m_rotarySpindle.resumeReadyPending=false;
    PrintSpindleCapabilities(m_rotarySpindleParameters,m_motion,"NC_ACCEPT",sourceLineNumber);
    return true;
}

WaitConditionFunc NCManager::HandleRotarySpindleMCodeSameThread(const NCBlock& block) noexcept
{
    const int pc=GetActiveDispatchPC();const auto* source=GetBaseProgramCache().TryGetLine(pc);const int line=source?source->sourceLineNumber:0;
    if(!NCSpindleSource::HasReservedM(block)){RejectRotarySpindleSameThread("SPINDLE_M_REQUIRED",line);return nullptr;}
    if(!ValidateRotarySpindleBlockSameThread(block,pc,line))return nullptr;
    NCSpindleDispatch::Row row{};row.programCounter=static_cast<std::uint64_t>(pc);row.sourceLine=static_cast<std::uint64_t>(line);
    row.dispatchId=m_currentExecutingBlockDispatchId; // MEMORY frame identity is zero.
    if(!source||m_rotarySpindle.broker.Prepare(m_rotarySpindle.authority,row,source->parsedBlock,block)!=NCSpindleDispatch::Status::Ok){
        RejectRotarySpindleSameThread("SPINDLE_PREPARE",line);return nullptr;}
    m_rotarySpindle.stagedRow=row;m_rotarySpindle.waitActive=true;m_rotarySpindle.waitComplete=false;m_rotarySpindle.waitSequence=0ULL;
    return WaitForRotarySpindleCallback;
}

void NCManager::CommitRotarySpindleCaptureSameThread(NCBlockDispatchId dispatchId,const MotionProgramBlockCapture& capture,
    const NCProgramCommitSnapshot& commit,bool committed,bool found,const NCBlockLifecycleSnapshot& ledger,int pc,int line) noexcept
{
    if(!m_rotarySpindle.active||!m_rotarySpindle.broker.HasPrepared())return;
    const auto& row=m_rotarySpindle.stagedRow;const auto& a=m_rotarySpindle.authority;
    const auto exact=[&](const NCProgramCommitSnapshot& v){return v.scope==NCProgramScope::MEMORY&&v.cacheGeneration==a.cacheGeneration&&
        v.frameId==row.frameIdentity&&v.sourcePC==pc;};
    const bool valid=committed&&found&&IsRotarySpindleScopeCurrentSameThread()&&m_state==NCState::RUN&&
        !AlarmManager::GetInstance().HasAlarm()&&dispatchId==row.dispatchId&&pc>=0&&static_cast<std::uint64_t>(pc)==row.programCounter&&
        line>0&&static_cast<std::uint64_t>(line)==row.sourceLine&&commit.IsValid()&&exact(commit)&&
        ledger.dispatchId==dispatchId&&ledger.sourceLineNumber==line&&exact(ledger.programTarget)&&exact(ledger.programCommit)&&
        ledger.programCommit.sequence==commit.sequence&&ledger.programCommitted&&!ledger.ncDispatchFailed&&!ledger.motionCaptureOverflow&&
        ledger.motionSegmentCount==0U&&capture.count==0U&&!capture.overflow;
    if(!valid){RejectRotarySpindleSameThread("SPINDLE_FORMAL_COMMIT",line,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
    // Begin itself is inert; it freezes installed axes and waits for RT's fresh
    // stopped baseline. Neither Begin nor Submit is called before formal proof.
    if(!m_rotarySpindle.rtBegun){
        if(!m_motion.BeginRotarySpindleSession(a,m_rotarySpindle.config)){
            RejectRotarySpindleSameThread("SPINDLE_RT_BEGIN",line,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
        m_rotarySpindle.rtBegun=true;
        // This whole program has no positional rows. Retire its unused
        // coordinate/retained-path receipts before RT invalidates HOME.
        // The immutable program run token and AUTO lease remain unchanged.
        RetireFixedTranslationSameThread();
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::EXPLICIT_FENCE);
    }
    NCSpindleDispatch::FormalCommit facts{};facts.authority=a;facts.row=row;facts.commitSequence=commit.sequence;
    facts.targetProgramCounter=facts.committedProgramCounter=row.programCounter;
    facts.committed=facts.ledgerFound=facts.captureValid=true;
    if(m_rotarySpindle.broker.Commit(facts)!=NCSpindleDispatch::Status::Ok){
        RejectRotarySpindleSameThread("SPINDLE_COMMIT_BROKER",line,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
    NCSpindleSource::Words words{};const auto* source=GetBaseProgramCache().TryGetLine(pc);
    if(!source||NCSpindleSource::Decode(source->parsedBlock,words)!=NCSpindleSource::Status::Ok){
        RejectRotarySpindleSameThread("SPINDLE_COMMIT_SOURCE",line);return;}
    NCSpindleDispatch::Packet packet{};
    if(!m_rotarySpindle.broker.ReadRequest(words.spindle,packet)){
        RejectRotarySpindleSameThread("SPINDLE_COMMIT_PACKET",line);return;}
    m_rotarySpindle.waitSpindle=words.spindle;m_rotarySpindle.waitSequence=packet.sequence;
    if(!PublishRotarySpindleRequestsSameThread()){
        RejectRotarySpindleSameThread("SPINDLE_RT_SUBMIT",line,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
    RtPrintf("[SPINDLE] build=EDM289_FIX1 event=COMMIT line=%d M=%d spindle=%u request=%llu rpm_milli=%lld\n",
        line,packet.mCode,words.spindle+1U,static_cast<unsigned long long>(packet.sequence),SpindleMilli(packet.signedRpm));
}

bool NCManager::PublishRotarySpindleRequestsSameThread() noexcept
{
    if(!m_rotarySpindle.rtBegun)return true;
    for(unsigned i=0;i<NCRotarySpindle::Count;++i){NCSpindleDispatch::Packet packet{};
        if(!m_rotarySpindle.broker.ReadRequest(i,packet)||m_rotarySpindle.submitted[i]==packet.sequence)continue;
        if(!m_motion.SubmitRotarySpindlePacket(packet))return false;
        m_rotarySpindle.submitted[i]=packet.sequence;}
    return true;
}

void NCManager::CancelRotarySpindleSameThread(SpindleState state,const char* reason) noexcept
{
    if(!m_rotarySpindle.active)return;
    if(!m_rotarySpindle.rtBegun){
        if(m_rotarySpindle.broker.AbortUnpublished()==NCSpindleDispatch::Status::Ok){
            m_rotarySpindle.active=false;m_rotarySpindle.waitActive=false;m_rotarySpindle.waitComplete=false;}
        return;
    }
    m_rotarySpindle.resumeReadyPending=false;
    const auto previous=m_rotarySpindle.broker.CurrentState();
    if(previous==state)return;
    if(m_rotarySpindle.lifecycleSequence==(std::numeric_limits<std::uint64_t>::max)()){
        m_motion.RequestEmergencyStopAllAxes();return;}
    const auto result=m_rotarySpindle.broker.ApplyLifecycle(state,++m_rotarySpindle.lifecycleSequence,m_startDiagnosticRequests);
    if(result==NCSpindleDispatch::Status::Ok){
        m_rotarySpindle.waitComplete=false;
        if(TerminalSpindleState(state))m_rotarySpindle.waitActive=false;
        if(!PublishRotarySpindleRequestsSameThread()){
            m_motion.RequestEmergencyStopAllAxes();
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY);
            m_state=NCState::ALARM;
        }
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=CANCEL reason=%s state=%u\n",reason,static_cast<unsigned>(state));
    }else if(result==NCSpindleDispatch::Status::SequenceExhausted)m_motion.RequestEmergencyStopAllAxes();
}

bool NCManager::TryHoldRotarySpindleSameThread() noexcept
{
    if(!m_rotarySpindle.active)return false;
    // An End/Reset/Alarm stop cannot become a resumable HOLD or stall End.
    if(TerminalSpindleState(m_rotarySpindle.broker.CurrentState()))return true;
    if(m_state!=NCState::RUN&&m_state!=NCState::HOLD)return true;
    if(!GCodeHandlers::Pause_G04(this))return true;
    m_state=NCState::HOLD;m_pauseAfterBlock=false;
    CancelRotarySpindleSameThread(SpindleState::Hold,"HOLD");
    return true;
}

bool NCManager::TryResumeRotarySpindleSameThread() noexcept
{
    if(!m_rotarySpindle.active)return false;
    ServiceRotarySpindleSameThread();
    if(!m_rotarySpindle.active)return true;
    if(m_state!=NCState::HOLD||AlarmManager::GetInstance().HasAlarm()||!IsRotarySpindleScopeCurrentSameThread())return true;
    const auto result=m_rotarySpindle.broker.TryResume(m_rotarySpindle.authority,m_startDiagnosticRequests);
    if(result==NCSpindleDispatch::Status::Ok&&!PublishRotarySpindleRequestsSameThread())
        RejectRotarySpindleSameThread("SPINDLE_RESUME_SUBMIT",0,AlarmManager::PATH_EXECUTION_NOT_READY);
    // Premature START is consumed by the broker; it never becomes a queued resume.
    return true;
}

void NCManager::ServiceRotarySpindleSameThread() noexcept
{
    if(!m_rotarySpindle.active||m_rotarySpindle.servicing)return;
    struct Guard{bool& flag;~Guard() noexcept{flag=false;}} guard{m_rotarySpindle.servicing};m_rotarySpindle.servicing=true;
    auto state=m_rotarySpindle.broker.CurrentState();
    if(m_state==NCState::RESET_STATE)CancelRotarySpindleSameThread(SpindleState::Reset,"RESET_STATE");
    else if(m_state==NCState::ALARM||AlarmManager::GetInstance().HasAlarm())CancelRotarySpindleSameThread(SpindleState::Alarm,"ALARM");
    else if(!TerminalSpindleState(state)&&m_isSingleBlockEnabled){
        RejectRotarySpindleSameThread("SPINDLE_SINGLE_BLOCK_UNSUPPORTED",0);return;}
    else if(!TerminalSpindleState(state)&&(!IsRotarySpindleScopeCurrentSameThread()||
        (m_state!=NCState::RUN&&m_state!=NCState::HOLD))){
        RejectRotarySpindleSameThread("SPINDLE_AUTHORITY_CHANGED",0,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
    else if(m_state==NCState::HOLD&&state==SpindleState::Run&&!m_rotarySpindle.resumeReadyPending)(void)TryHoldRotarySpindleSameThread();
    if(!m_rotarySpindle.active||!m_rotarySpindle.rtBegun)return;
    state=m_rotarySpindle.broker.CurrentState();
    if(!PublishRotarySpindleRequestsSameThread()&&!TerminalSpindleState(state)){
        RejectRotarySpindleSameThread("SPINDLE_TRANSPORT",0,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
    MotionRotarySpindleFeedback feedback{};
    if(!m_motion.ReadRotarySpindleFeedback(feedback)||!feedback.authority.Same(m_rotarySpindle.authority))return;
    if(feedback.fault&&!TerminalSpindleState(state)){
        RejectRotarySpindleSameThread("SPINDLE_RT_FAULT",0,AlarmManager::PATH_EXECUTION_NOT_READY);
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=RT_FAULT code=%u tick=%llu\n",feedback.faultCode,
            static_cast<unsigned long long>(feedback.tick));return;}
    for(unsigned i=0;i<NCRotarySpindle::Count;++i){
        const auto& receipt=feedback.receipt[i];
        if(m_rotarySpindle.broker.Poll(receipt)!=NCSpindleDispatch::Status::Ok)continue;
        if(m_rotarySpindle.waitActive&&receipt.packet.kind==NCSpindleDispatch::Kind::Command&&
            i==m_rotarySpindle.waitSpindle&&receipt.packet.sequence==m_rotarySpindle.waitSequence)
            m_rotarySpindle.waitComplete=true;
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=ACK spindle=%u kind=%u request=%llu commandRPM_milli=%lld actualRPM_milli=%lld\n",
            i+1U,static_cast<unsigned>(receipt.packet.kind),static_cast<unsigned long long>(receipt.packet.sequence),
            SpindleMilli(receipt.commandRpm),SpindleMilli(receipt.actualRpm));
    }
    if(state==SpindleState::Resuming&&m_rotarySpindle.broker.CurrentState()==SpindleState::Run)
        m_rotarySpindle.resumeReadyPending=true;
    if(m_rotarySpindle.resumeReadyPending){
        if(!IsRotarySpindleScopeCurrentSameThread()||AlarmManager::GetInstance().HasAlarm()){
            CancelRotarySpindleSameThread(SpindleState::AuthorityLost,"RESUME_AUTHORITY");return;}
        // RT already accepted this explicit START and proved readiness. NC's
        // final RUN publication also participates in the existing alarm CAS.
        auto& alarms=AlarmManager::GetInstance();
        const auto update=alarms.GetUpdateCount();const auto safety=alarms.GetMotionSafetyIntentState();
        AlarmManager::MotionAdmissionReservation admission{};
        const auto admitted=alarms.TryBeginMotionAdmission(update,safety,admission,true);
        if(admitted==AlarmManager::MotionAdmissionResult::BUSY)return;
        if(admitted!=AlarmManager::MotionAdmissionResult::ACQUIRED){
            RejectRotarySpindleSameThread("SPINDLE_RESUME_ALARM_ADMISSION",0,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
        NCState expected=NCState::HOLD;
        const bool committed=IsRotarySpindleScopeCurrentSameThread()&&
            m_state.compare_exchange_strong(expected,NCState::RUN,std::memory_order_acq_rel,std::memory_order_acquire);
        const bool ended=alarms.EndMotionAdmission(admission);
        if(!committed||!ended||!IsRotarySpindleScopeCurrentSameThread()||alarms.HasAlarm()){
            NCState running=NCState::RUN;(void)m_state.compare_exchange_strong(running,NCState::ALARM,
                std::memory_order_acq_rel,std::memory_order_acquire);
            CancelRotarySpindleSameThread(SpindleState::AuthorityLost,"RESUME_SUPERSEDED");
            m_motion.RequestEmergencyStopAllAxes();return;
        }
        m_rotarySpindle.resumeReadyPending=false;
        if(m_rotarySpindle.waitActive)m_rotarySpindle.waitComplete=true;
        m_pauseAfterBlock=false;
        if(!GCodeHandlers::Resume_G04(this))CancelRotarySpindleSameThread(SpindleState::Alarm,"DWELL_RESUME");
    }
    state=m_rotarySpindle.broker.CurrentState();
    if(TerminalSpindleState(state)&&!m_rotarySpindle.broker.HasPending()&&feedback.allStopped&&feedback.handbackReady&&
        m_motion.ReleaseRotarySpindleSession(m_rotarySpindle.authority)){
        if(m_rotarySpindle.broker.Retire()!=NCSpindleDispatch::Status::Ok)return;
        CoordSys.SyncMachinePosition(feedback.handbackMcsUnit.data());
        m_rotarySpindle.active=m_rotarySpindle.rtBegun=m_rotarySpindle.waitActive=m_rotarySpindle.waitComplete=m_rotarySpindle.resumeReadyPending=false;
        RtPrintf("[SPINDLE] build=EDM289_FIX1 event=HANDBACK state=%u homeRestored=0\n",static_cast<unsigned>(state));
    }
}

bool NCManager::WaitForRotarySpindleCallback(NCManager* nc) noexcept
{
    if(!nc)return false;
    nc->ServiceRotarySpindleSameThread();
    if(!nc->m_rotarySpindle.active||!nc->m_rotarySpindle.waitActive||!nc->m_rotarySpindle.waitSequence||
        !nc->m_rotarySpindle.waitComplete||nc->m_state!=NCState::RUN||AlarmManager::GetInstance().HasAlarm()||
        !nc->IsRotarySpindleScopeCurrentSameThread())return false;
    nc->m_rotarySpindle.waitActive=false;return true;
}
