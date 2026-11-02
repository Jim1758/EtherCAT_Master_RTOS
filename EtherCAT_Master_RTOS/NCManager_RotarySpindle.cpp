#include "NCManager.h"
#include "NCSpindleSource.h"
#include "NCRotarySpindle.h"
#include "NCSpindleDispatch.h"
#include "MotionCore_RotarySpindle.h"
#include "AlarmManager.h"
#include "GMCodeHandlers.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include "RotarySpindleTelemetry.h"
#include "SHM_Types.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <windows.h>
#include <rtapi.h>

extern PLCManager* g_PLC;

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

// Diagnostic cadence only. Clock loss suppresses progress printing; it never
// changes a deadline, command, callback result, or safety action.
std::uint64_t SpindleDiagnosticMilliseconds() noexcept
{
    LARGE_INTEGER now{};
    return RtGetClockTime(CLOCK_2,&now)&&now.QuadPart>0?
        static_cast<std::uint64_t>(now.QuadPart)/10000ULL:0ULL;
}
const char* SpindleFaultName(std::uint32_t code) noexcept
{
    switch(static_cast<MotionRotarySpindle::Fault>(code)){
    case MotionRotarySpindle::Fault::None:return "NONE";
    case MotionRotarySpindle::Fault::Source:return "SOURCE";
    case MotionRotarySpindle::Fault::Authority:return "AUTHORITY";
    case MotionRotarySpindle::Fault::Configuration:return "CONFIGURATION";
    case MotionRotarySpindle::Fault::Positioning:return "POSITIONING";
    case MotionRotarySpindle::Fault::Admission:return "ADMISSION";
    case MotionRotarySpindle::Fault::Output:return "OUTPUT";
    case MotionRotarySpindle::Fault::Feedback:return "FEEDBACK";
    case MotionRotarySpindle::Fault::Mailbox:return "MAILBOX";
    case MotionRotarySpindle::Fault::Handback:return "HANDBACK";
    case MotionRotarySpindle::Fault::Timeout:return "TIMEOUT";
    default:return "UNKNOWN";
    }
}
// Native RtPrintf truncates records at 511 bytes. Keep complete records below
// 480 bytes, including newline, and correlate every part using the same copied
// run/spindle/record/tick. Formatting is NC-only and never affects control.
template<class... Values>
void PrintSpindleRecord(const char* format,Values... values) noexcept
{
    char record[481]{};
    const int length=std::snprintf(record,sizeof(record),format,values...);
    if(length>0&&length<=480)RtPrintf("%s",record);
    else RtPrintf("[SPINDLE] build=EDM295 event=LOG_FORMAT_ERROR\n");
}
void PrintSpindleProgress(const char* event,std::uint64_t run,unsigned spindle,unsigned slot,unsigned record,
    const MotionRotarySpindleProgress& p,std::uint64_t expectedRequest,std::uint64_t receiptPublication) noexcept
{
    const auto runNumber=static_cast<unsigned long long>(run);
    const auto tick=static_cast<unsigned long long>(p.observationTick);
    PrintSpindleRecord("[SPINDLE] build=EDM295 event=%.16s run=%llu spindle=%u axisNumber=%u record=%u observedTick=%llu part=STATE expectedRequest=%llu requested=%llu applied=%llu qualified=%llu deadlineTick=%llu receiptPublication=%llu\n",
        event,runNumber,spindle+1U,slot+1U,record,tick,
        static_cast<unsigned long long>(expectedRequest),static_cast<unsigned long long>(p.requestedSequence),
        static_cast<unsigned long long>(p.appliedSequence),static_cast<unsigned long long>(p.qualifiedSequence),
        static_cast<unsigned long long>(p.deadlineTick),static_cast<unsigned long long>(receiptPublication));
    PrintSpindleRecord("[SPINDLE] build=EDM295 event=%.16s run=%llu spindle=%u axisNumber=%u record=%u observedTick=%llu part=VELOCITY rawDelta=%d rawDeltaValid=%u peakAbsRawDelta=%llu rawRPM_milli=%lld peakAbsRawRPM_milli=%lld priorCommandRPM_milli=%lld stopToleranceRPM_milli=%lld\n",
        event,runNumber,spindle+1U,slot+1U,record,tick,static_cast<int>(p.rawDelta),p.rawDeltaValid?1U:0U,
        static_cast<unsigned long long>(p.maximumAbsRawDelta),SpindleMilli(p.actualRpm),SpindleMilli(p.maximumAbsActualRpm),
        SpindleMilli(p.priorCommandRpm),SpindleMilli(p.stoppedToleranceRpm));
    PrintSpindleRecord("[SPINDLE] build=EDM295 event=%.16s run=%llu spindle=%u axisNumber=%u record=%u observedTick=%llu part=ESTIMATE windowUs=50000 requiredSpeedSamples=200 speedSamples=%u speedReady=%u filteredRPM_milli=%lld stopMotionRPM_milli=%lld\n",
        event,runNumber,spindle+1U,slot+1U,record,tick,p.speedSamples,p.speedReady?1U:0U,
        SpindleMilli(p.filteredRpm),SpindleMilli(p.stopMotionRpm));
    PrintSpindleRecord("[SPINDLE] build=EDM295 event=%.16s run=%llu spindle=%u axisNumber=%u record=%u observedTick=%llu part=PROOF zero=%u required=200 peakZero=%u sourceValid=%u exactPriorSend=%u zeroSent=%u packetApplied=%u finalSendTick=%llu finalSendRequest=%llu\n",
        event,runNumber,spindle+1U,slot+1U,record,tick,p.zeroSamples,p.maximumZeroSamples,
        p.sourceValid?1U:0U,p.exactPriorSend?1U:0U,p.zeroSent?1U:0U,p.packetApplied?1U:0U,
        static_cast<unsigned long long>(p.finalSendTick),static_cast<unsigned long long>(p.finalSendRequestSequence));
    PrintSpindleRecord("[SPINDLE] build=EDM295 event=%.16s run=%llu spindle=%u axisNumber=%u record=%u observedTick=%llu part=RESET reasonMask=%u lastResetMask=%u zeroResets=%u sourceBadSamples=%u sendBadSamples=%u overStopSamples=%u requestMismatchSamples=%u\n",
        event,runNumber,spindle+1U,slot+1U,record,tick,p.currentReasonMask,p.lastResetReasonMask,p.zeroResetCount,
        p.invalidSourceSamples,p.missingSendSamples,p.overStopSamples,p.requestMismatchSamples);
}

template<class Motion>
void PrintSpindleCapabilities(const RotarySpindleFile::Settings& settings,Motion& motion,const char* result,int line) noexcept
{
    // At most three pairs at admission/rejection, never in the RT loop or
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
        PrintSpindleRecord("[SPINDLE] build=EDM295 event=CAPABILITY result=%.16s line=%d spindle=%u axisNumber=%u capValid=%u configuredRPM_milli=%lld axisRPM_milli=%lld pdoRPM_milli=%lld effectiveRPM_milli=%lld reason=%.32s\n",
            result,line,i+1U,configured.axisNumber,valid?1U:0U,SpindleMilli(configured.maximumRpm),
            SpindleMilli(cap.axisRpm),SpindleMilli(cap.pdoRpm),SpindleMilli(cap.effectiveRpm),RotarySpindleFile::ErrorName(error.code));
        PrintSpindleRecord("[SPINDLE] build=EDM295 event=CAPABILITY_SCALE result=%.16s line=%d spindle=%u axisNumber=%u capValid=%u axisMaxPPS_milli=%lld allowedPPS_milli=%lld pulsesPerNative_milli=%lld pulsesPerRev_milli=%lld\n",
            result,line,i+1U,configured.axisNumber,valid?1U:0U,SpindleMilli(facts.maximumOutputPps),
            SpindleMilli(cap.maximumOutputPps),SpindleMilli(facts.pulsesPerNative),SpindleMilli(cap.pulsesPerOutputRevolution));
    }
}
bool TerminalSpindleState(SpindleState state) noexcept
{return state==SpindleState::End||state==SpindleState::AuthorityLost||state==SpindleState::Alarm||state==SpindleState::Reset;}
bool SpindleProgramRow(const NCParsedBlock& row) noexcept
{return NCSpindleSource::ClassifyProgramRow(row)!=NCSpindleSource::ProgramRowKind::Invalid;}
bool SpindleResolvedRow(const NCParsedBlock& row,const NCBlock& block) noexcept
{return NCSpindleSource::ValidateProgramResolved(row,block);}
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
    RtPrintf("[SPINDLE] build=EDM295 event=CONFIG result=%s line=%u spindle=%u key=%s reason=%s runtime=PHYSICAL_ADAPTER\n",
        result,m_rotarySpindleParameterError.line,m_rotarySpindleParameterError.spindle,
        m_rotarySpindleParameterError.key,RotarySpindleFile::ErrorName(m_rotarySpindleParameterError.code));
    if(status!=RotarySpindleFile::LoadStatus::Loaded&&status!=RotarySpindleFile::LoadStatus::CreatedDisabled)return;
    ++m_rotarySpindleParameterGeneration;
    for(unsigned i=0U;i<RotarySpindleFile::SpindleCount;++i){const auto& config=m_rotarySpindleParameters.spindle[i];
        RtPrintf("[SPINDLE] build=EDM295 event=CONFIG_AXIS spindle=%u axisNumber=%u enabled=%u maxRPM_milli=%lld accelSec_milli=%lld decelSec_milli=%lld stopSec_milli=%lld stopToleranceRPM_milli=%lld readyToleranceRPM_milli=%lld\n",
            i+1U,config.axisNumber,config.enabled?1U:0U,SpindleMilli(config.maximumRpm),
            SpindleMilli(config.accelTimeSeconds),SpindleMilli(config.decelTimeSeconds),SpindleMilli(config.stopTimeSeconds),
            SpindleMilli(config.stoppedToleranceRpm),SpindleMilli(config.readyToleranceRpm));}
}

void NCManager::ExportRotarySpindleTelemetrySameThread(const SHM_AxisDebugInfo* axes,
    RotarySpindleTelemetry::Image& out) noexcept
{
    namespace T=RotarySpindleTelemetry;
    T::Initialize(out);
    const bool claim=m_rotarySpindle.active&&m_rotarySpindle.rtBegun;
    const auto scope=claim?m_rotarySpindle.authority.scope.runToken:0ULL;
    if(claim!=m_rotarySpindle.telemetryClaim||scope!=m_rotarySpindle.telemetryScope){
        m_rotarySpindle.telemetryClaim=claim;m_rotarySpindle.telemetryScope=scope;
        m_rotarySpindle.telemetryTick=m_rotarySpindle.telemetryTickMs=m_rotarySpindle.telemetryClockMs=0ULL;
    }
    const auto now=SpindleDiagnosticMilliseconds();
    // Display freshness only. Loss/regression of the owner clock or the RT
    // observation cannot authorize motion or turn an unknown speed into zero.
    const auto fresh=[&](std::uint64_t tick,bool source) noexcept {
        auto& s=m_rotarySpindle;
        if(!now||!source||!tick){
            s.telemetryTick=s.telemetryTickMs=0ULL;s.telemetryClockMs=now;return false;
        }
        if((s.telemetryClockMs&&now<s.telemetryClockMs)||(s.telemetryTick&&tick<s.telemetryTick)){
            s.telemetryTick=tick;s.telemetryTickMs=0ULL;s.telemetryClockMs=now;return false;
        }
        s.telemetryClockMs=now;
        // A first copy is only a baseline: its RT tick could be historical.
        // Require observed progression before calling the measurement live.
        if(!s.telemetryTick){s.telemetryTick=tick;s.telemetryTickMs=0ULL;return false;}
        if(tick!=s.telemetryTick){s.telemetryTick=tick;s.telemetryTickMs=now;}
        return s.telemetryTickMs&&now>=s.telemetryTickMs&&now-s.telemetryTickMs<=350ULL;
    };
    MotionRotarySpindleFeedback feedback{};
    const bool coherent=claim&&m_motion.ReadRotarySpindleFeedback(feedback)&&
        feedback.authority.Same(m_rotarySpindle.authority);
    bool observationFresh=false;
    if(claim)observationFresh=fresh(feedback.tick,coherent);
    else{
        MotionNCSettleSnapshot source{};MotionNCSettleCounters counters{};
        const bool read=m_motion.TryGetNCSettleEvidence(MotionNCSettleProfile::GROUP_COMPLETION,source,counters);
        observationFresh=fresh(source.runtimeCycleTick,read&&source.runtimeObserved&&
            source.runtimeCycleValid&&source.runtimeCycleContiguous);
    }
    const bool loaded=m_rotarySpindleParameterStatus==RotarySpindleFile::LoadStatus::Loaded||
        m_rotarySpindleParameterStatus==RotarySpindleFile::LoadStatus::CreatedDisabled;
    for(unsigned i=0U;i<NCRotarySpindle::Count;++i){
        T::Record record{};
        const unsigned axisNumber=claim?m_rotarySpindle.config[i].axisSlot+1U:m_rotarySpindleParameters.spindle[i].axisNumber;
        const bool enabled=claim?m_rotarySpindle.config[i].enabled:loaded&&m_rotarySpindleParameters.spindle[i].enabled;
        record.axisNumber=axisNumber<=8U?static_cast<std::uint8_t>(axisNumber):0U;
        if(!enabled){record.state=loaded||claim?T::State::Disabled:T::State::Unavailable;T::Encode(out,i,record);continue;}
        record.flags=T::Enabled;
        if(m_rotarySpindle.plcActiveMask&(1U<<i))record.flags|=T::PLCActive;
        record.state=T::State::Unavailable;
        if(claim){
            record.flags|=T::ClaimActive;
            if(coherent&&(feedback.fault||m_state==NCState::ALARM)){
                record.flags|=T::Fault;record.state=T::State::Fault;
            }
            if(observationFresh){
                const auto& p=feedback.progress[i];
                record.targetRpm=feedback.requestedRpm[i];
                NCSpindleDispatch::Packet pending{};
                const bool queued=m_rotarySpindle.broker.ReadRequest(i,pending);
                if(queued)record.targetRpm=pending.signedRpm;
                record.commandRpm=feedback.commandRpm[i];record.actualRpm=feedback.actualRpm[i];
                if(p.observationTick==feedback.tick&&p.sourceValid&&p.speedReady)record.flags|=T::RpmValid;
                if(feedback.positionValidMask&(1U<<i)){
                    record.flags|=T::PositionValid;record.continuousMcsDegrees=feedback.actualMcsDegrees[i];
                    record.angleDegrees=record.continuousMcsDegrees;
                    if(feedback.homedMask&(1U<<i))record.flags|=T::Homed;
                }
                if(!(record.flags&T::Fault)){
                    const auto phase=feedback.phase[i];
                    if(m_state==NCState::HOLD){
                        record.state=(feedback.rotationActiveMask&(1U<<i))?T::State::Stopping:T::State::Held;
                    }else if(queued&&p.appliedSequence!=pending.sequence)record.state=T::State::Starting;
                    else if(phase==NCRotarySpindle::Phase::Stopping)record.state=T::State::Stopping;
                    else if(phase==NCRotarySpindle::Phase::Ramping)record.state=T::State::Starting;
                    else if(phase==NCRotarySpindle::Phase::Held)record.state=T::State::Held;
                    else if(record.commandRpm>0.0)record.state=T::State::Forward;
                    else if(record.commandRpm<0.0)record.state=T::State::Reverse;
                    else record.state=T::State::Idle;
                }
                if(!(record.flags&T::RpmValid)&&!(record.flags&T::Fault))record.state=T::State::Unavailable;
            }
        }else if(axes&&axisNumber>=1U&&axisNumber<=8U){
            const auto slot=axisNumber-1U;const auto& axis=m_motion.GetAxisContext(static_cast<int>(slot));
            const auto& debug=axes[slot];
            const bool mapped=axis.axisIndex==static_cast<int>(slot)&&axis.isExist&&
                (axis.axisType==AxisType::ROTARY||axis.axisType==AxisType::ROTARY_CONTINUOUS);
            if(mapped&&observationFresh&&debug.IsServoOn){
                // ExportDebugInfo(...,true) supplies native degrees and deg/s.
                // Existing ActualRPM is MOTOR rpm and is intentionally unused.
                record.commandRpm=debug.CmdVel/6.0;record.actualRpm=debug.ActVel/6.0;
                record.angleDegrees=record.continuousMcsDegrees=debug.ActPos;
                record.flags|=T::RpmValid|T::PositionValid;
                if(debug.IsHomed)record.flags|=T::Homed;
                if(debug.IsFault||debug.IsLagAlarm||m_state==NCState::ALARM||
                    debug.State==static_cast<int>(MotionState::MotionState_ERROR)||
                    debug.State==static_cast<int>(MotionState::MotionState_ESTOP)){
                    record.flags|=T::Fault;record.state=T::State::Fault;
                }else if(axis.homeRuntime.active)record.state=T::State::Homing;
                else if(debug.State==static_cast<int>(MotionState::MotionState_STOPPING))record.state=T::State::Stopping;
                else if(debug.State==static_cast<int>(MotionState::MotionState_IDLE))record.state=T::State::Idle;
                else record.state=T::State::Positioning;
            }
        }
        T::Encode(out,i,record);
    }
}

void NCManager::RejectRotarySpindleSameThread(const char* reason,int line,int alarm) noexcept
{
    AlarmManager::GetInstance().Trigger(alarm?alarm:AlarmManager::G_Code_Invalid_parameter,line);
    CancelRotarySpindleSameThread(SpindleState::Alarm,reason);
    ChangeState(NCState::ALARM);
    PrintSpindleCapabilities(m_rotarySpindleParameters,m_motion,"REJECT",line);
    RtPrintf("[SPINDLE] build=EDM295 event=REJECT line=%d reason=%s\n",line,reason);
}

bool NCManager::ValidateRotarySpindleParsedScopeSameThread(const NCParsedBlock& parsed,int line) noexcept
{
    if(!m_rotarySpindle.active||SpindleProgramRow(parsed))return true;
    RejectRotarySpindleSameThread("SPINDLE_XYZ_PROGRAM_SCOPE",line);return false;
}

bool NCManager::IsRotarySpindleXYZProgramSameThread() const noexcept
{return m_rotarySpindle.active&&m_rotarySpindle.concurrentXYZ;}

bool NCManager::IsRotarySpindleXYZResumeReadySameThread() noexcept
{
    return IsRotarySpindleXYZProgramSameThread()&&m_state==NCState::HOLD&&m_rotarySpindle.resumeReadyPending&&
        m_rotarySpindle.broker.ResumeReady()&&IsRotarySpindleScopeCurrentSameThread()&&
        !AlarmManager::GetInstance().HasAlarm();
}

bool NCManager::IsRotarySpindleXYZFeedBlockSameThread(const NCBlock& block) const noexcept
{
    if(!IsRotarySpindleXYZProgramSameThread()||m_mode!=NCOperationMode::MEMORY||!m_macroStack.empty())return false;
    const auto* source=GetBaseProgramCache().TryGetLine(GetActiveDispatchPC());
    return source&&NCSpindleSource::ClassifyProgramRow(source->parsedBlock)==NCSpindleSource::ProgramRowKind::XYZFeed&&
        NCSpindleSource::ValidateProgramResolved(source->parsedBlock,block);
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
        m_motion.HasPendingSafetyOrRecoveryRequests()||AlarmManager::GetInstance().HasAlarm()){
        RejectRotarySpindleSameThread("PROGRAM_AUTHORITY",sourceLineNumber,AlarmManager::PATH_EXECUTION_NOT_READY);return false;}
    NCSpindleDispatch::Config config{};RotarySpindleFile::Error error{};
    if(!m_rotarySpindleParameterGeneration||!BuildRotarySpindleRuntimeSameThread(config,error)){
        CancelRotarySpindleSameThread(SpindleState::Alarm,"CONFIG_REJECT");
        RtPrintf("[SPINDLE] build=EDM295 event=CONFIG_REJECT line=%d spindle=%u key=%s code=%u reason=%s loadStatus=%u\n",
            sourceLineNumber,error.spindle,error.key,static_cast<unsigned>(error.code),
            RotarySpindleFile::ErrorName(error.code),static_cast<unsigned>(m_rotarySpindleParameterStatus));
        RejectRotarySpindleSameThread(RotarySpindleFile::ErrorName(error.code),sourceLineNumber);return false;}
    const auto admitWords=[&](const NCSpindleSource::Words& request,int line){
        const auto& limit=config[request.spindle];
        const char* reason=!limit.enabled?"SPINDLE_DISABLED":
            !request.stop&&request.rpm>limit.maximumRpm?"S_EXCEEDS_MAX_RPM":nullptr;
        if(!reason)return true;
        RejectRotarySpindleSameThread(reason,line);
        RtPrintf("[SPINDLE] build=EDM295 event=REQUEST_REJECT line=%d M=%d spindle=%u enabled=%u requestedRPM_milli=%lld effectiveRPM_milli=%lld reason=%s\n",
            line,request.mCode,request.spindle+1U,limit.enabled?1U:0U,SpindleMilli(request.rpm),
            SpindleMilli(limit.enabled?limit.maximumRpm:0.0),reason);
        return false;
    };
    if(!admitWords(words,sourceLineNumber))return false;
    if(m_rotarySpindle.active){
        if(!IsRotarySpindleScopeCurrentSameThread()||m_rotarySpindle.broker.CurrentState()!=SpindleState::Run||
            (!m_rotarySpindle.concurrentXYZ&&CoordSys.IsTranslationRunFrozen())){
            RejectRotarySpindleSameThread("SPINDLE_SCOPE_CHANGED",sourceLineNumber);return false;}
        return true;
    }
    // Bound the NC-owner preflight and reject every unsupported later row
    // before the first spindle command can reach RT. The only concurrent
    // positional vocabulary is explicit literal XYZ G01 with 0 < F <= 100.
    const auto& program=GetBaseProgramCache();
    if(program.Size()>256U){RejectRotarySpindleSameThread("SPINDLE_PROGRAM_LIMIT_256",sourceLineNumber);return false;}
    bool concurrentXYZ=false,executionStarted=false;
    for(std::size_t i=0;i<program.Size();++i){const auto* row=program.TryGetLine(static_cast<int>(i));
        if(!row||!SpindleProgramRow(row->parsedBlock)){
            RejectRotarySpindleSameThread("SPINDLE_XYZ_PROGRAM_SCOPE",row?row->sourceLineNumber:sourceLineNumber);return false;}
        const auto kind=NCSpindleSource::ClassifyProgramRow(row->parsedBlock);
        // Selector changes may drain or replace the native frame/Epoch. Keep
        // them in the program prefix, before either positional or spindle
        // execution, so the full restricted run retains one admitted scope.
        if(kind==NCSpindleSource::ProgramRowKind::Setup&&executionStarted){
            RejectRotarySpindleSameThread("SPINDLE_SETUP_REQUIRES_PREFIX",row->sourceLineNumber);return false;}
        concurrentXYZ=concurrentXYZ||kind==NCSpindleSource::ProgramRowKind::XYZFeed;
        executionStarted=executionStarted||kind==NCSpindleSource::ProgramRowKind::XYZFeed||
            kind==NCSpindleSource::ProgramRowKind::Spindle;
        if(NCSpindleSource::HasReservedM(row->parsedBlock)){
            NCSpindleSource::Words request{};
            if(NCSpindleSource::Decode(row->parsedBlock,request)!=NCSpindleSource::Status::Ok){
                RejectRotarySpindleSameThread("EXACT_STANDALONE_M_SOURCE_REQUIRED",row->sourceLineNumber);return false;}
            // Validate every future S/map before this run can claim an axis.
            // An over-cap S is rejected unchanged, never reduced to the cap.
            if(!admitWords(request,row->sourceLineNumber))return false;
        }
    }
    if(concurrentXYZ){
        // Match the native XYZ frame's existing type vocabulary before any
        // spindle claims an axis. Continuous-type auxiliary axes have no
        // native XYZ frame contract in this phase; pure spindle runs retain
        // their existing ROTARY_CONTINUOUS support.
        for(unsigned i=0U;i<8U;++i){const auto& axis=m_motion.GetAxisContext(static_cast<int>(i));
            if(i<3U&&(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||
                axis.axisType!=AxisType::LINEAR||m_axisNames[i]!="XYZ"[i])){
                RejectRotarySpindleSameThread("XYZ_FEED_AXIS_IDENTITY_UNSUPPORTED",sourceLineNumber);return false;
            }
            if(i>=3U&&axis.isExist&&axis.axisType!=AxisType::LINEAR&&axis.axisType!=AxisType::ROTARY){
                RejectRotarySpindleSameThread("XYZ_FEED_AXIS_TYPE_UNSUPPORTED",sourceLineNumber);return false;
            }
        }
        for(unsigned i=0U;i<NCRotarySpindle::Count;++i)if(config[i].enabled){
            const auto slot=config[i].axisSlot;
            if(slot<3U||slot>5U){
                RejectRotarySpindleSameThread("XYZ_SPINDLE_AXIS_REQUIRES_CUV",sourceLineNumber);return false;
            }
            if(m_axisNames[slot]!="CUV"[slot-3U]||
                m_motion.GetAxisContext(static_cast<int>(slot)).axisType!=AxisType::ROTARY){
                RejectRotarySpindleSameThread("XYZ_FEED_AXIS_IDENTITY_UNSUPPORTED",sourceLineNumber);return false;
            }
        }
    }
    if(!concurrentXYZ&&CoordSys.IsTranslationRunFrozen()){
        RejectRotarySpindleSameThread("PROGRAM_AUTHORITY",sourceLineNumber,AlarmManager::PATH_EXECUTION_NOT_READY);return false;
    }
    NCSpindleDispatch::Authority authority{};
    authority.scope.runToken=m_pathCoreLiveBookkeeping.currentRunToken;
    authority.scope.epoch=m_motion.GetCurrentExecutionEpoch();authority.scope.ownerGeneration=m_programMotionLease.generation;
    authority.cacheIdentity=static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(&GetBaseProgramCache()));
    authority.cacheGeneration=program.GetGeneration();authority.configGeneration=m_rotarySpindleParameterGeneration;
    authority.configReceipt=SpindleConfigReceipt(config);authority.memoryProgram=authority.autoOwner=true;
    if(m_rotarySpindle.broker.Begin(authority,config,m_startDiagnosticRequests,concurrentXYZ)!=NCSpindleDispatch::Status::Ok){
        RejectRotarySpindleSameThread("SPINDLE_PREVIOUS_SESSION_PENDING",sourceLineNumber);return false;}
    m_rotarySpindle.authority=authority;m_rotarySpindle.config=config;m_rotarySpindle.submitted={};
    m_rotarySpindle.active=true;m_rotarySpindle.rtBegun=false;m_rotarySpindle.concurrentXYZ=concurrentXYZ;
    m_rotarySpindle.waitActive=m_rotarySpindle.waitComplete=m_rotarySpindle.resumeReadyPending=false;
    m_rotarySpindle.firstFaultLogged=false;m_rotarySpindle.progressLogCount=0U;
    m_rotarySpindle.plcFeedbackTick=0ULL;m_rotarySpindle.plcLogCount=0U;
    m_rotarySpindle.diagnosticLastMs=SpindleDiagnosticMilliseconds();
    RtPrintf("[SPINDLE] build=EDM295 event=PROGRAM_SCOPE concurrentXYZ=%u allowed=G01_XYZ spindleAxes=CUV dischargeCommand=NONE\n",concurrentXYZ?1U:0U);
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
        if(!m_motion.BeginRotarySpindleSession(a,m_rotarySpindle.config,m_rotarySpindle.concurrentXYZ)){
            RejectRotarySpindleSameThread("SPINDLE_RT_BEGIN",line,AlarmManager::PATH_EXECUTION_NOT_READY);return;}
        m_rotarySpindle.rtBegun=true;
        unsigned enabled=0U;
        for(unsigned i=0U;i<NCRotarySpindle::Count;++i)if(m_rotarySpindle.config[i].enabled)enabled|=1U<<i;
        PublishRotarySpindlePLCActiveSameThread(m_rotarySpindle.plcActiveMask|enabled,"BEGIN_STOP_CONFIRM");
        // A concurrent XYZ program keeps its frozen/bound translation run.
        // Its C/U/V HOME invalidation is separately scoped to the spindle
        // claim; retiring the whole run would invalidate later G01 sources.
        if(!m_rotarySpindle.concurrentXYZ){
            RetireFixedTranslationSameThread();
            FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::EXPLICIT_FENCE);
        }
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
    RtPrintf("[SPINDLE] build=EDM295 event=COMMIT line=%d M=%d spindle=%u request=%llu rpm_milli=%lld\n",
        line,packet.mCode,words.spindle+1U,static_cast<unsigned long long>(packet.sequence),SpindleMilli(packet.signedRpm));
}

bool NCManager::PublishRotarySpindleRequestsSameThread() noexcept
{
    if(!m_rotarySpindle.rtBegun)return true;
    for(unsigned i=0;i<NCRotarySpindle::Count;++i){NCSpindleDispatch::Packet packet{};
        if(!m_rotarySpindle.broker.ReadRequest(i,packet)||m_rotarySpindle.submitted[i]==packet.sequence)continue;
        if(!m_motion.SubmitRotarySpindlePacket(packet))return false;
        m_rotarySpindle.submitted[i]=packet.sequence;
        PublishRotarySpindlePLCActiveSameThread(m_rotarySpindle.plcActiveMask|(1U<<i),"REQUEST_PENDING");}
    return true;
}

// NC-owned level status: ON means rotating or physical stop not yet confirmed.
// It is never a drive-enable, discharge permission, or a motion completion key.
void NCManager::PublishRotarySpindlePLCActiveSameThread(unsigned mask,const char* reason) noexcept
{
    mask&=7U;
    const auto previous=m_rotarySpindle.plcActiveMask;
    m_rotarySpindle.plcActiveMask=mask;
    if(g_PLC)g_PLC->PublishRotarySpindleOutputs(mask);
    if(previous==mask)return;
    // Bound all ordinary transitions, including OFF chatter. The single
    // terminal HANDBACK record below always includes the final OFF image.
    if(m_rotarySpindle.plcLogCount>=48U)return;
    ++m_rotarySpindle.plcLogCount;
    RtPrintf("[SPINDLE] build=EDM295 event=PLC_ACTIVE run=%llu S230=%u S231=%u S232=%u mask=%u previous=%u reason=%s\n",
        static_cast<unsigned long long>(m_rotarySpindle.authority.scope.runToken),
        (mask&1U)?1U:0U,(mask&2U)?1U:0U,(mask&4U)?1U:0U,mask,previous,reason?reason:"SYNC");
}

void NCManager::SyncRotarySpindlePLCFeedbackSameThread(const MotionRotarySpindleFeedback& feedback) noexcept
{
    if(!feedback.authority.Same(m_rotarySpindle.authority)||!feedback.tick||
        feedback.tick<=m_rotarySpindle.plcFeedbackTick)return;
    m_rotarySpindle.plcFeedbackTick=feedback.tick;
    unsigned mask=m_rotarySpindle.plcActiveMask;
    for(unsigned i=0U;i<NCRotarySpindle::Count;++i){
        if(!m_rotarySpindle.config[i].enabled)continue;
        const unsigned bit=1U<<i;
        NCSpindleDispatch::Packet pending{};
        const bool hasPending=m_rotarySpindle.broker.ReadRequest(i,pending);
        const auto& p=feedback.progress[i];const auto submitted=m_rotarySpindle.submitted[i];
        // A cached stopped image must not extinguish a newly queued M3/M4 or
        // START. A never-commanded enabled mapping can still prove standstill.
        const bool matching=hasPending?submitted==pending.sequence&&p.packetApplied&&p.appliedSequence==submitted:
            submitted?p.packetApplied&&p.appliedSequence==submitted:!p.packetApplied&&p.appliedSequence==0ULL;
        const bool stopped=matching&&!(hasPending&&pending.signedRpm!=0.0)&&
            (feedback.rotationActiveMask&bit)==0U&&p.observationTick==feedback.tick&&p.sourceValid&&p.speedReady&&
            p.exactPriorSend&&p.zeroSent&&p.zeroSamples>=NCRotarySpindle::RequiredSamples&&feedback.commandRpm[i]==0.0;
        if(stopped)mask&=~bit;
        else mask|=bit;
    }
    PublishRotarySpindlePLCActiveSameThread(mask,"PHYSICAL_FEEDBACK");
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
        RtPrintf("[SPINDLE] build=EDM295 event=CANCEL reason=%s state=%u\n",reason,static_cast<unsigned>(state));
    }else if(result==NCSpindleDispatch::Status::SequenceExhausted)m_motion.RequestEmergencyStopAllAxes();
}

bool NCManager::TryHoldRotarySpindleSameThread() noexcept
{
    if(!m_rotarySpindle.active)return false;
    // An End/Reset/Alarm stop cannot become a resumable HOLD or stall End.
    if(TerminalSpindleState(m_rotarySpindle.broker.CurrentState()))return true;
    if(m_state!=NCState::RUN&&m_state!=NCState::HOLD)return true;
    if(m_rotarySpindle.concurrentXYZ){
        CancelRotarySpindleSameThread(SpindleState::Hold,"XYZ_HOLD");
        // The ordinary PROGRAM feed-hold path must still request and prove
        // the XYZ stop. Do not publish HOLD early or consume that path here.
        return false;
    }
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
    if(m_rotarySpindle.concurrentXYZ&&!IsRotarySpindleXYZHoldReadySameThread()){
        (void)m_rotarySpindle.broker.ConsumeStart(m_rotarySpindle.authority,m_startDiagnosticRequests);
        return true;
    }
    const auto result=m_rotarySpindle.broker.TryResume(m_rotarySpindle.authority,m_startDiagnosticRequests);
    if(result==NCSpindleDispatch::Status::Ok&&!PublishRotarySpindleRequestsSameThread())
        RejectRotarySpindleSameThread("SPINDLE_RESUME_SUBMIT",0,AlarmManager::PATH_EXECUTION_NOT_READY);
    // Premature START is consumed by the broker; it never becomes a queued resume.
    return true;
}

void NCManager::ServiceRotarySpindleSameThread() noexcept
{
    // Republish the NC latch after PLC reconnect/reinitialization as well.
    PublishRotarySpindlePLCActiveSameThread(m_rotarySpindle.plcActiveMask,nullptr);
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
    // At most sixteen logical progress records per run, at one-second NC clock
    // cadence. Each copied record is emitted as bounded correlated parts.
    // Source/transport silence is observable without relying on a live RT tick.
    // These fields never satisfy a wait or change the existing RT deadlines.
    const auto logWait=[&](const MotionRotarySpindleFeedback* current,const char* unavailable){
        if(!m_rotarySpindle.broker.HasPending()||m_rotarySpindle.progressLogCount>=16U)return;
        const auto now=SpindleDiagnosticMilliseconds();if(!now)return;
        if(now<m_rotarySpindle.diagnosticLastMs){m_rotarySpindle.diagnosticLastMs=now;return;}
        if(now-m_rotarySpindle.diagnosticLastMs<1000ULL)return;
        m_rotarySpindle.diagnosticLastMs=now;
        if(!current){
            RtPrintf("[SPINDLE] build=EDM295 event=WAIT result=%s run=%llu state=%u record=%u expectedRequest=%llu\n",
                unavailable,static_cast<unsigned long long>(m_rotarySpindle.authority.scope.runToken),
                static_cast<unsigned>(m_rotarySpindle.broker.CurrentState()),++m_rotarySpindle.progressLogCount,
                static_cast<unsigned long long>(m_rotarySpindle.waitSequence));return;
        }
        for(unsigned i=0U;i<NCRotarySpindle::Count&&m_rotarySpindle.progressLogCount<16U;++i){
            NCSpindleDispatch::Packet pending{};if(!m_rotarySpindle.broker.ReadRequest(i,pending))continue;
            PrintSpindleProgress("WAIT",m_rotarySpindle.authority.scope.runToken,i,m_rotarySpindle.config[i].axisSlot,
                ++m_rotarySpindle.progressLogCount,current->progress[i],pending.sequence,current->receipt[i].publicationSequence);
        }
    };
    MotionRotarySpindleFeedback feedback{};
    if(!m_motion.ReadRotarySpindleFeedback(feedback)){logWait(nullptr,"NO_FEEDBACK");return;}
    if(!feedback.authority.Same(m_rotarySpindle.authority)){logWait(nullptr,"SCOPE_MISMATCH");return;}
    SyncRotarySpindlePLCFeedbackSameThread(feedback);
    if(feedback.fault){
        const bool reject=!TerminalSpindleState(state);
        if(reject)RejectRotarySpindleSameThread("SPINDLE_RT_FAULT",0,AlarmManager::PATH_EXECUTION_NOT_READY);
        // RT may latch the global alarm before NC reads this channel. Preserve
        // cancellation priority, but still report its first frozen cause once.
        if(!m_rotarySpindle.firstFaultLogged){
            m_rotarySpindle.firstFaultLogged=true;
            const auto& first=feedback.firstFault;const auto code=first.valid?first.code:feedback.faultCode;
            RtPrintf("[SPINDLE] build=EDM295 event=FIRST_FAULT run=%llu faultName=%s code=%u frozen=%u tick=%llu beginTick=%llu observedTick=%llu initialized=%u state=%u\n",
                static_cast<unsigned long long>(m_rotarySpindle.authority.scope.runToken),SpindleFaultName(code),code,first.valid?1U:0U,
                static_cast<unsigned long long>(first.valid?first.tick:feedback.tick),static_cast<unsigned long long>(first.beginTick),
                static_cast<unsigned long long>(feedback.tick),(first.valid?first.initialized:feedback.initialized)?1U:0U,
                static_cast<unsigned>(m_rotarySpindle.broker.CurrentState()));
            for(unsigned i=0U;i<NCRotarySpindle::Count;++i)if(m_rotarySpindle.config[i].enabled)
                PrintSpindleProgress("FAULT_AXIS",m_rotarySpindle.authority.scope.runToken,i,m_rotarySpindle.config[i].axisSlot,0U,
                    first.valid?first.progress[i]:feedback.progress[i],0ULL,0ULL);
        }
        if(reject)return;
    }
    for(unsigned i=0;i<NCRotarySpindle::Count;++i){
        const auto& receipt=feedback.receipt[i];
        if(m_rotarySpindle.broker.Poll(receipt)!=NCSpindleDispatch::Status::Ok)continue;
        if(m_rotarySpindle.waitActive&&receipt.packet.kind==NCSpindleDispatch::Kind::Command&&
            i==m_rotarySpindle.waitSpindle&&receipt.packet.sequence==m_rotarySpindle.waitSequence)
            m_rotarySpindle.waitComplete=true;
        RtPrintf("[SPINDLE] build=EDM295 event=ACK spindle=%u kind=%u request=%llu commandRPM_milli=%lld actualRPM_milli=%lld\n",
            i+1U,static_cast<unsigned>(receipt.packet.kind),static_cast<unsigned long long>(receipt.packet.sequence),
            SpindleMilli(receipt.commandRpm),SpindleMilli(receipt.actualRpm));
    }
    if(state==SpindleState::Resuming&&(m_rotarySpindle.broker.CurrentState()==SpindleState::Run||
        m_rotarySpindle.broker.ResumeReady()))
        m_rotarySpindle.resumeReadyPending=true;
    if(m_rotarySpindle.resumeReadyPending){
        if(!IsRotarySpindleScopeCurrentSameThread()||AlarmManager::GetInstance().HasAlarm()){
            CancelRotarySpindleSameThread(SpindleState::AuthorityLost,"RESUME_AUTHORITY");return;}
        if(m_rotarySpindle.concurrentXYZ){
            // Spindles have reached their saved targets, but NC remains HOLD.
            // The ordinary XYZ stop/alarm admission transaction supplies its
            // own commit. A BUSY transaction is retried with this same START.
            if(!IsRotarySpindleXYZResumeReadySameThread()||
                !CompleteRotarySpindleXYZResumeSameThread())return;
            if(m_state!=NCState::RUN||AlarmManager::GetInstance().HasAlarm()||!IsRotarySpindleScopeCurrentSameThread()||
                m_rotarySpindle.broker.CommitResume(m_rotarySpindle.authority)!=NCSpindleDispatch::Status::Ok){
                RejectRotarySpindleSameThread("XYZ_RESUME_COMMIT",0,AlarmManager::PATH_EXECUTION_NOT_READY);
                m_motion.RequestEmergencyStopAllAxes();return;
            }
            m_rotarySpindle.resumeReadyPending=false;
            if(m_rotarySpindle.waitActive)m_rotarySpindle.waitComplete=true;
            m_pauseAfterBlock=false;
            RtPrintf("[SPINDLE] build=EDM295 event=XYZ_RESUME_COMMITTED run=%llu start=%llu\n",
                static_cast<unsigned long long>(m_rotarySpindle.authority.scope.runToken),
                static_cast<unsigned long long>(m_startDiagnosticRequests));
        }else{
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
    }
    logWait(&feedback,nullptr);
    state=m_rotarySpindle.broker.CurrentState();
    if(TerminalSpindleState(state)&&!m_rotarySpindle.broker.HasPending()&&feedback.allStopped&&feedback.handbackReady&&
        m_motion.ReleaseRotarySpindleSession(m_rotarySpindle.authority)){
        if(m_rotarySpindle.broker.Retire()!=NCSpindleDispatch::Status::Ok)return;
        if(m_rotarySpindle.concurrentXYZ){
            // A prior M5 may have proved spindle handback before a later G01.
            // Its all-axis diagnostic image cannot replace that G01 endpoint.
            std::array<double,8U> position{};
            for(unsigned i=0U;i<8U;++i)position[i]=CoordSys.commandedMCS[i];
            for(unsigned i=0U;i<8U;++i)if(feedback.axisMask&(1U<<i))position[i]=feedback.handbackMcsUnit[i];
            CoordSys.SyncMachinePosition(position.data());
        }else CoordSys.SyncMachinePosition(feedback.handbackMcsUnit.data());
        m_rotarySpindle.active=m_rotarySpindle.rtBegun=m_rotarySpindle.waitActive=m_rotarySpindle.waitComplete=m_rotarySpindle.resumeReadyPending=false;
        PublishRotarySpindlePLCActiveSameThread(0U,"PHYSICAL_HANDBACK");
        RtPrintf("[SPINDLE] build=EDM295 event=HANDBACK state=%u homeRestored=0 S230=0 S231=0 S232=0\n",static_cast<unsigned>(state));
    }
}

bool NCManager::WaitForRotarySpindleCallback(NCManager* nc) noexcept
{
    if(!nc)return false;
    nc->ServiceRotarySpindleSameThread();
    if(!nc->m_rotarySpindle.active||!nc->m_rotarySpindle.waitActive||!nc->m_rotarySpindle.waitSequence||
        !nc->m_rotarySpindle.waitComplete||nc->m_state!=NCState::RUN||AlarmManager::GetInstance().HasAlarm()||
        !nc->IsRotarySpindleScopeCurrentSameThread())return false;
    // Command target-ready alone does not prove that the disjoint XYZ lane
    // has a fresh stopped baseline. Preserve the completed M wait until RT
    // supplies the same-owner, same-epoch physical settle acknowledgement.
    if(nc->m_rotarySpindle.concurrentXYZ&&
        !nc->m_motion.HasRotarySpindleXYZSettleAcknowledgement(
            nc->m_rotarySpindle.authority.scope.epoch,nc->m_programMotionLease))return false;
    nc->m_rotarySpindle.waitActive=false;return true;
}
