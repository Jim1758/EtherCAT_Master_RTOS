#include "NCManager.h"
#include "PositionAutoTuneConfig.h"
#include "PositionAutoTuneCapture.h"
#include "PositionAutoTuneAnalysis.h"
#include "PositionAutoTuneJob.h"
#include "NCGCodeSemantics.h"
#include "GlobalConfig.h"
#include "AlarmManager.h"
#include <windows.h>
#include <rtapi.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <limits>
#include <new>

namespace {
// TUNE03 FIX5: RtPrintf does not consume floating-point conversions. Format
// NC-only diagnostics with the CRT, then submit one bounded string. Keeping
// this out of line avoids adding its buffer to every NC caller's stack frame.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, format(printf, 1, 2)))
#endif
void TunePrintDiagnosticLine(const char* format, ...) noexcept {
    if(!format){
        RtPrintf("%s","[TUNE10] event=LOG_FORMAT_FAILURE\n");return;
    }
    char line[1536];
    va_list args;va_start(args,format);
    const int length=std::vsnprintf(line,sizeof(line),format,args);
    va_end(args);
    if(length<0){
        RtPrintf("%s","[TUNE10] event=LOG_FORMAT_FAILURE\n");return;
    }
    if(static_cast<std::size_t>(length)>=sizeof(line)){
        const char suffix[]=" [TRUNCATED]\n";
        std::memcpy(line+sizeof(line)-sizeof(suffix),suffix,sizeof(suffix));
    }
    RtPrintf("%s",line);
}
enum class TunePhase : unsigned { Idle, Admission, CaptureReady, Submit, Applied, Moving, Finish, Analysis, Cancel, Done,
    ApproachReady, ApproachApplied, ApproachMoving, MeasurementArm };
constexpr double TuneAdmissionWaitSeconds=0.25;
bool TuneClock(std::uint64_t& value) noexcept {
    LARGE_INTEGER now{};
    if(!RtQueryPerformanceCounter(&now)||now.QuadPart<=0) return false;
    value=static_cast<std::uint64_t>(now.QuadPart);return true;
}
// A recognizable new P1 supersedes the per-axis cruise comparison baseline,
// including requests rejected before capture or worker creation. This helper
// does not authorize the block and never performs IO.
int TuneRequestedP1Axis(const NCBlock& block) noexcept {
    if(!block.has('P')||block.val('P')!=1.0||!block.has('A'))return -1;
    const double axis=block.val('A');
    return std::isfinite(axis)&&axis>=0.0&&axis<static_cast<double>(PositionAutoTune::kAxisCount)&&
        std::floor(axis)==axis?static_cast<int>(axis):-1;
}
double TuneElapsed(std::uint64_t now,std::uint64_t then,std::uint64_t frequency) noexcept {
    return now>=then&&frequency?static_cast<double>(now-then)/static_cast<double>(frequency):-1.0;
}
PositionAutoTune::CaptureGains TuneGains(const PidConfig& source) noexcept {
    PositionAutoTune::CaptureGains gains{};
    gains.kp=source.Kp;gains.ki=source.Ki;gains.kd=source.Kd;gains.kvff=source.Kvff;return gains;
}
bool TuneSameGains(const PidConfig& actual,const PositionAutoTune::CaptureGains& frozen) noexcept {
    return actual.Kp==frozen.kp&&actual.Ki==frozen.ki&&actual.Kd==frozen.kd&&actual.Kvff==frozen.kvff;
}
void TuneAdmissionDiagnostic(const MotionStopSettleSnapshot& stop,std::uint64_t session) noexcept {
    // Diagnostic only. The exact formal START proof, not this separately read
    // advisory snapshot, owns admission. Print once on timeout, never per tick.
    TunePrintDiagnosticLine("[TUNE10] event=ADMISSION_BLOCK session=%llu stopPub=%llu sample=%llu blocker=%u blockAxis=%d axisState=%u groupActive=%u queues=%u/%u/%u axes=%u nonIdle=%u cmdMoving=%u actualMoving=%u outsideWindow=%u worstActualAxis=%d actualPps=%.9g actualDeadbandPps=%.9g motion=0 gainsChanged=0\n",
        static_cast<unsigned long long>(session),static_cast<unsigned long long>(stop.publicationGeneration),
        static_cast<unsigned long long>(stop.sampleSequence),static_cast<unsigned>(stop.primaryBlocker),
        stop.primaryAxisIndex,static_cast<unsigned>(stop.primaryAxisState),stop.groupActive?1U:0U,
        stop.commandQueueDepth,stop.commandIngressDepth,stop.commandReplayDepth,stop.existingAxisCount,
        stop.nonIdleAxisCount,stop.commandMovingAxisCount,stop.actualMovingAxisCount,
        stop.outsideInPositionWindowAxisCount,stop.worstActualVelocityAxisIndex,
        stop.maxAxisActualVelocityAbsPps,stop.actualVelocityDeadbandPps);
}
bool TuneValidGains(const PositionAutoTune::CaptureGains& g) noexcept {
    return std::isfinite(g.kp)&&g.kp>0.0&&std::isfinite(g.ki)&&g.ki>=0.0&&
        std::isfinite(g.kd)&&g.kd>=0.0&&std::isfinite(g.kvff)&&g.kvff>=0.0;
}
const char* TuneRangeAtOrigin(const PositionAutoTune::AxisConfig& p,double originPulse,double ppu,
    double& minimum,double& maximum) noexcept {
    minimum=maximum=0.0;
    if(!p.rangeEnabled)return nullptr;
    // currentCmdPos is already machine-referenced. Never apply the HOME
    // offset again or wrap rotary positions into 0..360 here.
    const double origin=originPulse/ppu;
    const double onePulse=1.0/ppu;
    const double settleMargin=p.settleWindowMm+onePulse;
    const double margin=(std::max)(p.followingLimitMm,(std::max)(p.maxOvershootMm,settleMargin));
    const double halfSpan=p.excursionMm+p.endPadding+margin;
    if(!std::isfinite(ppu)||ppu<=0.0||!std::isfinite(originPulse)||!std::isfinite(origin)||
        !std::isfinite(onePulse)||onePulse<=0.0||!std::isfinite(settleMargin)||
        !std::isfinite(halfSpan)||halfSpan<=0.0)return "RANGE_NONFINITE";
    minimum=origin-halfSpan;maximum=origin+halfSpan;
    if(!std::isfinite(minimum)||!std::isfinite(maximum)||minimum>=origin||maximum<=origin)
        return "RANGE_NONFINITE";
    return minimum<p.rangeMinMCS||maximum>p.rangeMaxMCS?"RANGE_ORIGIN_OUTSIDE":nullptr;
}
double TuneLegSeconds(double distance,double speed,double acceleration,double deceleration) noexcept {
    if(!std::isfinite(distance)||distance<0.0||!std::isfinite(speed)||speed<=0.0||
        !std::isfinite(acceleration)||acceleration<=0.0||!std::isfinite(deceleration)||deceleration<=0.0)
        return (std::numeric_limits<double>::infinity)();
    if(distance==0.0)return 0.0;
    const double ramps=acceleration+deceleration;
    return distance>=speed*ramps*0.5?distance/speed+ramps*0.5:
        std::sqrt(2.0*distance/speed)*std::sqrt(ramps);
}
bool TuneApproachPose(const PositionAutoTune::CaptureRequest& r,double command,double actual) noexcept {
    const double end=r.originPulse+r.excursionPulse;
    const double minimum=(std::min)(r.originPulse,r.approachStartPulse);
    const double maximum=(std::max)(end,r.approachStartPulse);
    return std::isfinite(command)&&std::isfinite(actual)&&
        std::isfinite(r.approachStartPulse)&&std::isfinite(minimum)&&std::isfinite(maximum)&&
        command>=minimum-r.settleWindowPulse-1.0&&command<=maximum+r.settleWindowPulse+1.0&&
        actual>=minimum-r.boundaryMarginPulse&&actual<=maximum+r.boundaryMarginPulse&&
        std::fabs(command-actual)<=r.followingLimitPulse;
}
enum class TuneBoundaryRead { Invalid, AwaitLatest, Ready };
TuneBoundaryRead TuneMeasurementBoundaryRead(const PositionAutoTune::CaptureMeasurementBoundary& boundary,
    const PositionAutoTune::CaptureSample& sample,std::size_t count,std::uint64_t firstObservedUs) noexcept {
    if(boundary.sampleOffset>=count||boundary.timestampUs<firstObservedUs)return TuneBoundaryRead::Invalid;
    return boundary.cycle>sample.cycle||boundary.timestampUs>sample.timestampUs?
        TuneBoundaryRead::AwaitLatest:TuneBoundaryRead::Ready;
}
struct TuneHash {
    std::uint64_t value=1469598103934665603ULL;
    void Number(std::uint64_t bits) noexcept {
        for(unsigned i=0;i<8U;++i){value^=(bits>>(8U*i))&255ULL;value*=1099511628211ULL;}
    }
    void Real(double number) noexcept {std::uint64_t bits=0;std::memcpy(&bits,&number,sizeof(bits));Number(bits);}
    void Text(const std::string& text) noexcept {Number(text.size());for(unsigned char c:text){value^=c;value*=1099511628211ULL;}}
};
// Deliberately excludes both gain banks and start position: P2 runs after
// explicit PID file adoption/restart. The baseline binds measured/candidate
// banks; the RT capture still rejects gain changes during an active test.
std::uint64_t TuneIdentity(const PositionAutoTune::Config& c,const AxisContext& a,int axis,bool pointToPoint=false,
    std::uint32_t sampleStride=1U) noexcept {
    // TUNE10 separates trajectory and guarded-output ceilings. Earlier P1
    // baselines do not describe this admitted correction-output envelope.
    TuneHash h;h.Number(7U);h.Number(pointToPoint);h.Text(c.machineId);h.Number(static_cast<unsigned>(c.machineProfile));
    h.Number(static_cast<unsigned>(axis));h.Number(static_cast<unsigned>(a.axisIndex));
    h.Number(static_cast<unsigned>(a.axisType));h.Number(static_cast<unsigned>(a.fbMode));
    h.Real(a.resolution_PPR);h.Real(a.finalLead);h.Real(a.mechanicalPitch);
    h.Real(a.rotaryModulo);
    h.Real(a.reduction_MotorSide);h.Real(a.reduction_LoadSide);h.Real(a.scaleToMotorRatio);
    h.Number(a.isReverse);h.Number(a.Axis_Reverse);h.Number(a.isExist);
    h.Real(a.maxVel_PPS);h.Real(a.smoothTime_ms);h.Real(a.Stop_dec_time);
    h.Real(a.inPositionWindow_Pulse);h.Real(a.pid.MaxLag);h.Number(a.pid.EnableLagCheck);
    h.Real(a.pid.MaxIntegral);h.Number(a.enablePitch);h.Number(a.enableBacklash);
    h.Number(a.travelLimit1Enable);h.Real(a.travelLimit1Negative_unit);h.Real(a.travelLimit1Positive_unit);
    h.Number(a.travelLimit2Enable);h.Real(a.travelLimit2Negative_unit);h.Real(a.travelLimit2Positive_unit);
    h.Number(a.travelLimit3Enable);h.Real(a.travelLimit3Negative_unit);h.Real(a.travelLimit3Positive_unit);
    h.Number(c.enabled);h.Real(c.maxDurationSeconds);h.Real(c.twoPointMaxDurationSeconds);h.Number(c.maxSamples);h.Real(CYCLE_TIME_SEC);
    h.Real(c.modelFitMinimum);h.Real(c.minimumVelocityMmS);h.Real(c.candidateResponseFactor);h.Real(c.maxKpIncreaseRatio);
    for(const auto& p:c.axis){
        h.Number(p.enabled);h.Real(p.excursionMm);h.Real(p.speedMmMin);h.Real(p.accelerationSeconds);
        h.Real(p.decelerationSeconds);h.Real(p.dwellSeconds);h.Number(p.cycles);h.Real(p.followingLimitMm);
        h.Real(p.settleWindowMm);h.Real(p.settleHoldSeconds);h.Real(p.settleTimeoutSeconds);
        h.Real(p.kpMin);h.Real(p.kpMax);h.Real(p.kvffMin);h.Real(p.kvffMax);
        h.Real(p.maxOvershootMm);h.Real(p.maxRmsErrorMm);
        h.Number(p.rangeEnabled);h.Real(p.rangeMinMCS);h.Real(p.rangeMaxMCS);h.Real(p.endPadding);
        h.Number(p.idleEnabled);h.Real(p.idleKpMin);h.Real(p.idleKpMax);h.Real(p.idleKiMin);h.Real(p.idleKiMax);
        h.Real(p.idleIntegralTimeSeconds);h.Real(p.idleObservationSeconds);
        h.Real(p.idleMaxRmsErrorMm);h.Real(p.idleMaxJitterMm);
    }
    // Keep option-free identities deterministic within this controller
    // revision. Any strategy/goal opt-in binds the entire new
    // context so P2 cannot compare a baseline made with another preference.
    bool strategyContext=false;
    for(const auto& p:c.axis)strategyContext=strategyContext||p.tuningStrategy!=0U||p.targetPositionMm!=0.0;
    if(strategyContext) {
        h.Text("TUNE07_STRATEGY_TARGET_V1");
        for(const auto& p:c.axis){h.Number(p.tuningStrategy);h.Real(p.targetPositionMm==0.0?0.0:p.targetPositionMm);}
    }
    bool cruiseContext=false;
    for(const auto& p:c.axis)cruiseContext=cruiseContext||p.cruiseLagTargetMm!=0.0;
    if(cruiseContext) {
        h.Text("TUNE09_CRUISE_LAG_TARGET_V1");
        for(const auto& p:c.axis)h.Real(p.cruiseLagTargetMm==0.0?0.0:p.cruiseLagTargetMm);
    }
    // Full-rate P1/P2 identities remain unchanged. Decimated diagnostics
    // cannot inherit a model candidate or verification from a full-rate run.
    if(sampleStride!=1U){h.Text("TUNE08_METRICS_ONLY_V1");h.Number(sampleStride);}
    return h.value?h.value:1ULL;
}
}

struct PositionAutoTuneNCState {
    PositionAutoTune::Config config{};
    PositionAutoTune::CaptureRequest capture{};
    PositionAutoTune::JobRequest job{};
    PositionAutoTune::CaptureLiveTelemetry liveTelemetry{};
    bool hasLiveTelemetry=false;
    std::shared_ptr<const PositionAutoTune::JobResult> result;
    std::vector<PositionAutoTune::AnalysisSegment> segments;
    MotionOwnerLease lease{};
    MotionAxisCommandSequence command=0ULL;
    TunePhase phase=TunePhase::Idle,livePublishedPhase=TunePhase::Idle;
    std::uint64_t lastLivePublishClock=0ULL;
    std::uint64_t serial=0ULL,session=0ULL,run=0ULL,cache=0ULL,identity=0ULL;
    std::uint64_t cruiseBaselineRevision=0ULL;
    std::uint64_t frequency=0ULL,startClock=0ULL,phaseClock=0ULL,lastClock=0ULL,lastFreshClock=0ULL;
    std::uint64_t lastCycle=0ULL,commandCompleteUs=0ULL,observationStartedUs=0ULL;
    std::uint64_t firstCaptureUs=0ULL,measurementStartCycle=0ULL;
    std::size_t measurementOffset=0U;
    double targetPulse=0.0,previousTargetPulse=0.0,initialCommandPulse=0.0;
    double sessionBudgetSeconds=0.0,predictedSeconds=0.0,approachSeconds=0.0;
    unsigned mode=0U,leg=0U;
    int axis=-1,sourcePC=-1,sourceLine=0;
    bool active=false,done=false,failed=false,cancelled=false,jobStarted=false,captureStarted=false;
    bool callbackConsumed=false,resetPending=false,resetReleased=false,resetBypass=false,resetAdmitted=false;
    char failure[96]{};
};

void NCManager::InitializePositionAutoTuneSameThread() noexcept {
    try {m_positionAutoTune=std::make_shared<PositionAutoTuneNCState>();}
    catch(...){m_positionAutoTune.reset();}
}

bool NCManager::ValidatePositionAutoTuneBlockSameThread(const NCBlock& block,int pc,int line) noexcept {
    const int requestedP1Axis=TuneRequestedP1Axis(block);
    if(requestedP1Axis>=0)PositionAutoTune::InvalidateCruiseBaseline(requestedP1Axis);
    const char* reason=nullptr;
    if(!m_positionAutoTune)reason="STORAGE";
    else if(m_positionAutoTune->active||m_positionAutoTune->resetPending||PositionAutoTune::JobBusy()||
        PositionAutoTune::Capture().Active()||PositionAutoTune::Capture().RuntimeNeedsStop())reason="PREVIOUS_SESSION_BUSY";
    else if(block.isEmpty||block.isGoto||block.isBlockSkip||!block.hasG||block.gCount!=1||
        block.gCode!=190||block.gCodes[0]!=190||block.mCount!=0||!block.has('P')||!block.has('A')||
        (block.val('P')!=1.0&&block.val('P')!=2.0)||!std::isfinite(block.val('A'))||
        block.val('A')<0.0||block.val('A')>=static_cast<double>(PositionAutoTune::kAxisCount)||std::floor(block.val('A'))!=block.val('A'))reason="BLOCK_SHAPE";
    else if(block.has('Q')&&block.val('Q')!=1.0)reason="TWO_POINT_REQUIRES_Q1";
    if(!reason)for(char c='A';c<='Z';++c)if(block.has(c)&&
        ((c!='A'&&c!='P'&&c!='N'&&c!='Q')||!std::isfinite(block.val(c)))){reason="UNEXPECTED_WORD";break;}
    const NCProgramCacheLine* authored=GetBaseProgramCache().TryGetLine(pc);
    if(!reason&&(m_mode!=NCOperationMode::MEMORY||!m_macroStack.empty()||m_isG66Active||
        !authored||authored->sourceLineNumber!=line))reason="MEMORY_LITERAL_ONLY";
    if(!reason){
        const auto& p=authored->parsedBlock;
        const char axisWord[]={static_cast<char>('0'+static_cast<int>(block.val('A'))),'\0'};
        if(p.error!=NCParseError::NONE||p.isEmpty||p.isBlockSkip||p.dependsOnMacroState||
            p.controlType!=NCParsedControlType::NONE||p.duplicateAddressMask||p.gCount!=1||p.mCount!=0||
            p.gExpressions[0]!="190"||p.expression('P')!=(block.val('P')==1.0?"1":"2")||p.expression('A')!=axisWord||
            (block.has('Q')&&p.expression('Q')!="1"))
            reason="LITERAL_ROW_REQUIRED";
    }
    if(reason){
        // A new invalid request cannot leave an older successful result visible.
        for(int i=MacroVariableRules::TuningResultFirst;i<=MacroVariableRules::TuningResultLast;++i)MacroSys.SetSysVar(i,0.0);
        MacroSys.SetSysVar(933,5.0);MacroSys.SetSysVar(905,100.0);MacroSys.SetSysVar(964,6.0);MacroSys.SetSysVar(901,6.0);
        TunePrintDiagnosticLine("[TUNE10] event=ADMISSION result=REJECT line=%d reason=%s motion=0 gainsChanged=0\n",line,reason);
        AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter,line);m_state=NCState::ALARM;return false;
    }
    m_positionAutoTune->sourcePC=pc;m_positionAutoTune->sourceLine=line;return true;
}

WaitConditionFunc NCManager::StartPositionAutoTuneSameThread(const NCBlock& block) {
    // Also protect direct callers that did not pass the literal-row validator.
    // Freeze the revision here, before any admission failure; an older capture
    // must not restore eligibility after a newer P1 request superseded it.
    const int requestedP1Axis=TuneRequestedP1Axis(block);
    if(requestedP1Axis>=0)PositionAutoTune::InvalidateCruiseBaseline(requestedP1Axis);
    if(!m_positionAutoTune)return nullptr;
    auto& s=*m_positionAutoTune;
    const int sourcePC=s.sourcePC,sourceLine=s.sourceLine;
    const char* reason=nullptr;
    try {
        if(s.active||s.resetPending||PositionAutoTune::JobBusy()||PositionAutoTune::Capture().Active()||
            PositionAutoTune::Capture().RuntimeNeedsStop())reason="BUSY";
        else if(s.serial==UINT64_MAX)reason="SESSION_EXHAUSTED";
        else {
            const std::uint64_t serial=s.serial+1ULL;s=PositionAutoTuneNCState{};s.serial=serial;s.session=serial;
            s.sourcePC=sourcePC;s.sourceLine=sourceLine;s.axis=static_cast<int>(block.val('A'));
            s.cruiseBaselineRevision=PositionAutoTune::CruiseBaselineRevision(s.axis);
            s.mode=static_cast<unsigned>(block.val('P'));s.capture.pointToPoint=block.has('Q');s.segments.reserve(128U);
            std::string configError;
            const auto loaded=PositionAutoTune::LoadConfig(GlobalConfig::GetInstance().ParameterDir+
                "PositionAutoTuneConfig.txt",s.config,configError);
            if(loaded!=PositionAutoTune::ConfigLoadStatus::Loaded||!s.config.enabled){
                TunePrintDiagnosticLine("[TUNE10] event=CONFIG result=REJECT detail=%s\n",configError.c_str());reason="DISABLED_OR_CONFIG";
            }
        }
        if(!reason&&(s.axis<0||s.axis>=static_cast<int>(PositionAutoTune::kAxisCount)||
            s.axis>=GlobalConfig::GetInstance().System_axisCount||!s.config.axis[s.axis].enabled))reason="AXIS_DISABLED";
        if(!reason&&(m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||Close_System_Com_flag||
            !m_macroStack.empty()||m_isG66Active||m_edm94.program||m_edm94.latched||m_edm94.armPending||
            m_gapDryRun.active||m_gapPath.active||m_gapWindow.active||m_gapRecovery.active||m_gapPending.active||
            m_edmZFixture.pending||m_edmZFixture.cleanupPending||m_edmZFixture.resetPending||
            (m_g76Orbit&&m_g76Orbit->active)||(m_edm171Air&&m_edm171Air->selected)||
            m_g76Modal.valid||m_pathHold.armed||m_pathHold.requested))reason="NON_POSITIONING_SCOPE";
        if(!reason&&(CoordSys.isInchMode||CoordSys.isG68Active||CoordSys.isWorkpieceRotationActive||
            CoordSys.IsScaleMirrorActive()||CoordSys.isPolarCoordinateActive||
            CoordSys.toolLengthMode!=49||CoordSys.toolRadiusMode!=40||CoordSys.IsManualFrameEnabled()))reason="FRAME_OR_COMPENSATION";
        s.lease=m_motion.GetMotionOwnerLease();s.capture.epoch=m_motion.GetCurrentExecutionEpoch();
        if(!reason&&(!s.lease.IsValid()||s.lease.owner!=MotionOwner::AUTO||
            !s.lease.Matches(m_programMotionLease)||!s.capture.epoch||AlarmManager::GetInstance().HasAlarm()||
            m_motion.HasPendingSafetyOrRecoveryRequests()))reason="AUTHORITY";
        // TUNE03 FIX2: raw 250 us encoder derivatives are advisory in an
        // in-position IDLE servo. Use the existing exact program-start proof
        // below, including all-axis readiness, drain, epoch and owner identity.
        if(!reason){
            auto& a=m_motion.GetAxisContext(s.axis);const auto& p=s.config.axis[s.axis];
            const double ppu=a.resolution_PPR/a.finalLead;
            if(!a.isExist||a.axisIndex!=s.axis||
                (a.axisType!=AxisType::LINEAR&&a.axisType!=AxisType::ROTARY)||
                a.fbMode!=FeedbackSource::MOTOR_ENCODER||a.targetMode!=9||!a.isServoOn||a.isFault||a.isLagAlarm||
                a.hardLimitPositive||a.hardLimitNegative||a.enablePitch||a.enableBacklash||
                !std::isfinite(ppu)||ppu<=0.0||!std::isfinite(a.currentCmdPos)||
                !std::isfinite(a.feedrateOverride)||a.feedrateOverride!=1.0)reason="AXIS_STATE_OR_SCALE";
            else if(a.axisType==AxisType::ROTARY&&
                (!std::isfinite(a.rotaryModulo)||a.rotaryModulo<=0.0))reason="ROTARY_SCALE";
            else if(s.config.machineProfile==PositionAutoTune::MachineProfile::Machine&&
                (!a.isHomed||(!(a.travelLimit1Enable&&CoordSys.IsProgrammableTravelLimitEnabled())&&
                    !a.travelLimit2Enable&&!a.travelLimit3Enable)))reason="MACHINE_HOME_OR_LIMITS";
            else {
                s.capture.session=s.session;s.capture.axis=s.axis;s.capture.axisType=static_cast<unsigned>(a.axisType);s.capture.owner=static_cast<unsigned>(s.lease.owner);
                s.capture.ownerGeneration=s.lease.generation;s.capture.pulsesPerMm=ppu;
                s.initialCommandPulse=a.currentCmdPos;
                s.capture.approachStartPulse=s.initialCommandPulse;
                s.capture.originPulse=s.capture.pointToPoint?p.rangeMinMCS*ppu:a.currentCmdPos;
                s.capture.excursionPulse=(s.capture.pointToPoint?p.rangeMaxMCS-p.rangeMinMCS:p.excursionMm)*ppu;
                s.capture.boundaryMarginPulse=(std::max)(p.followingLimitMm,p.maxOvershootMm)*ppu;
                s.capture.followingLimitPulse=p.followingLimitMm*ppu;
                // The normal servo loop already clamps feedback output at 1.2
                // times nominal maxVel_PPS. Keep the planned reference and
                // encoder/jump ceilings unchanged while allowing that existing
                // correction authority at the nominal maximum test speed.
                s.capture.maximumReferenceVelocityPps=(std::min)(2.0*p.speedMmMin*ppu/60.0,a.maxVel_PPS);
                s.capture.maximumVelocityPps=(std::min)(2.0*p.speedMmMin*ppu/60.0,1.2*a.maxVel_PPS);
                s.capture.cruiseVelocityPps=p.speedMmMin*ppu/60.0;
                s.sessionBudgetSeconds=s.capture.pointToPoint?s.config.twoPointMaxDurationSeconds:s.config.maxDurationSeconds;
                s.capture.maximumTicks=static_cast<std::uint64_t>(s.sessionBudgetSeconds/CYCLE_TIME_SEC);
                s.capture.settleWindowPulse=p.settleWindowMm*ppu;
                s.capture.settleVelocityPps=(std::max)(1.0/CYCLE_TIME_SEC,0.1*s.config.minimumVelocityMmS*ppu);
                s.capture.motionGains=TuneGains(a.Pid_G00);s.capture.idleGains=TuneGains(a.Pid_IDLE);
                s.capture.approachEnabled=s.capture.pointToPoint;
                s.capture.measurementSettleHoldUs=s.capture.pointToPoint?static_cast<std::uint64_t>(p.settleHoldSeconds*1e6):0ULL;
                const double envelopeMargin=s.capture.pointToPoint?
                    (std::max)(s.capture.boundaryMarginPulse,s.capture.settleWindowPulse+1.0):s.capture.boundaryMarginPulse;
                // Q1's taught values are command endpoints, including an
                // endpoint exactly at an existing software travel limit.
                // Servo tracking margins remain in the independent RT guard;
                // they must not silently reduce the requested travel stroke.
                const double minimum=s.capture.pointToPoint?p.rangeMinMCS:
                    (s.capture.originPulse-s.capture.excursionPulse-envelopeMargin)/ppu;
                const double maximum=s.capture.pointToPoint?p.rangeMaxMCS:
                    (s.capture.originPulse+s.capture.excursionPulse+envelopeMargin)/ppu;
                if(!TuneValidGains(s.capture.motionGains)||!TuneValidGains(s.capture.idleGains)||
                    (s.capture.motionGains.ki!=0.0||s.capture.motionGains.kd!=0.0))reason="GAINS_REQUIRE_P_AND_FF";
                else if(s.capture.pointToPoint&&(!p.rangeEnabled||!std::isfinite(s.capture.originPulse)||
                    !std::isfinite(s.capture.excursionPulse)||s.capture.excursionPulse<=0.0||
                    !std::isfinite(s.capture.originPulse+s.capture.excursionPulse)||
                    s.capture.originPulse+s.capture.excursionPulse<=s.capture.originPulse))reason="TWO_POINT_RANGE_REQUIRED";
                else if(s.capture.pointToPoint&&!TuneApproachPose(s.capture,a.currentCmdPos,a.currentActPos))
                    reason="APPROACH_START_OUTSIDE_ENVELOPE";
                else if(!std::isfinite(a.maxVel_PPS)||a.maxVel_PPS<=0.0||p.speedMmMin*ppu/60.0<=1.0||
                    p.speedMmMin*ppu/60.0>a.maxVel_PPS||p.accelerationSeconds<0.001||p.decelerationSeconds<0.001||
                    (p.speedMmMin*ppu/60.0)/p.accelerationSeconds<=10.0||
                    (p.speedMmMin*ppu/60.0)/p.decelerationSeconds<=10.0||
                    !CoordSys.IsTargetWithinConfiguredSoftwareTravelLimit(a,minimum)||
                    !CoordSys.IsTargetWithinConfiguredSoftwareTravelLimit(a,maximum)||
                    (s.capture.pointToPoint&&!CoordSys.IsTargetWithinConfiguredSoftwareTravelLimit(a,s.initialCommandPulse/ppu)))
                    reason="SPEED_OR_TRAVEL_BOUNDS";
                else {
                    if(s.capture.pointToPoint) {
                        auto stroke=p;stroke.excursionMm=s.capture.excursionPulse/ppu;
                        const double distance=std::fabs(s.initialCommandPulse-s.capture.originPulse)/ppu;
                        const bool moving=std::fabs(s.initialCommandPulse-s.capture.originPulse)>1.0;
                        const double smoothing=std::isfinite(a.smoothTime_ms)&&a.smoothTime_ms>=0.0?
                            a.smoothTime_ms/1000.0:(std::numeric_limits<double>::infinity)();
                        // Include ordinary trapezoid/triangle time, the existing
                        // output smoother for every move, the initial settle
                        // hold, the 20 ms drift-window warm-up at A and after
                        // EVERY leg, and bounded NC admission/dispatch reserve.
                        s.predictedSeconds=PositionAutoTune::PredictAxisDurationSeconds(stroke)+
                            (moving?TuneLegSeconds(distance,p.speedMmMin/60.0,p.accelerationSeconds,p.decelerationSeconds):0.0)+
                            smoothing*(4.0*p.cycles+(moving?1.0:0.0))+p.settleHoldSeconds+
                            0.02*(4.0*p.cycles+1.0)+0.25;
                        s.sessionBudgetSeconds=PositionAutoTune::EffectiveTwoPointDurationSeconds(s.config,s.predictedSeconds);
                        if(!std::isfinite(s.predictedSeconds)||s.predictedSeconds<=0.0||
                            !std::isfinite(s.sessionBudgetSeconds)||s.predictedSeconds>s.sessionBudgetSeconds)
                            reason="TWO_POINT_DURATION_EXCEEDS_ONE_HOUR";
                        else {
                            s.capture.maximumTicks=static_cast<std::uint64_t>(s.sessionBudgetSeconds/CYCLE_TIME_SEC);
                            constexpr std::uint64_t savedCapacity=PositionAutoTune::CaptureCapacity-4U;
                            s.capture.sampleStride=static_cast<std::uint32_t>(
                                (s.capture.maximumTicks+savedCapacity-1ULL)/savedCapacity);
                            if(!s.capture.maximumTicks||!s.capture.sampleStride||s.capture.sampleStride>64U)
                                reason="TWO_POINT_CAPTURE_BUDGET_INVALID";
                        }
                        TunePrintDiagnosticLine("[TUNE10] event=TWO_POINT_ADMISSION axis=%d result=%s startCmdMCS=%.12g startActualMCS=%.12g endpointA=%.12g endpointB=%.12g envelopeMinMCS=%.12g envelopeMaxMCS=%.12g speedNativePerMin=%.12g referenceLimitNativePerMin=%.12g outputLimitNativePerMin=%.12g predictedSeconds=%.12g budgetSeconds=%.12g sampleStride=%u legacyExcursionApplied=0 endPaddingApplied=0 reason=%s motion=0 gainsChanged=0\n",
                            s.axis,reason?"REJECT":"PASS",a.currentCmdPos/ppu,a.currentActPos/ppu,s.capture.originPulse/ppu,
                            (s.capture.originPulse+s.capture.excursionPulse)/ppu,minimum,maximum,p.speedMmMin,
                            s.capture.maximumReferenceVelocityPps/ppu*60.0,s.capture.maximumVelocityPps/ppu*60.0,
                            s.predictedSeconds,s.sessionBudgetSeconds,s.capture.sampleStride,reason?reason:"NONE");
                    } else {
                        double rangeMinimum=0.0,rangeMaximum=0.0;
                        if(PositionAutoTune::PredictAxisDurationSeconds(p)>s.config.maxDurationSeconds)
                            reason="NOMINAL_MOTION_EXCEEDS_DURATION";
                        else reason=TuneRangeAtOrigin(p,s.capture.originPulse,ppu,rangeMinimum,rangeMaximum);
                        if(p.rangeEnabled)TunePrintDiagnosticLine(
                        "[TUNE10] event=RANGE_ADMISSION axis=%d result=%s originMCS=%.12g rangeMinMCS=%.12g rangeMaxMCS=%.12g endPadding=%.12g envelopeMinMCS=%.12g envelopeMaxMCS=%.12g reason=%s motion=0 gainsChanged=0\n",
                        s.axis,reason?"REJECT":"PASS",s.capture.originPulse/ppu,p.rangeMinMCS,p.rangeMaxMCS,
                        p.endPadding,rangeMinimum,rangeMaximum,reason?reason:"NONE");
                    }
                    if(!reason){s.identity=TuneIdentity(s.config,a,s.axis,s.capture.pointToPoint,s.capture.sampleStride);s.previousTargetPulse=s.capture.originPulse;}
                }
            }
        }
        if(!reason){
            LARGE_INTEGER frequency{};
            if(!RtQueryPerformanceFrequency(&frequency)||frequency.QuadPart<=0||!TuneClock(s.startClock))reason="CLOCK";
            else{s.frequency=static_cast<std::uint64_t>(frequency.QuadPart);s.lastClock=s.phaseClock=s.lastFreshClock=s.startClock;}
        }
        if(!reason){
            s.run=m_pathCoreLiveBookkeeping.currentRunToken;s.cache=GetBaseProgramCache().GetGeneration();
            if(!s.run||!m_motion.IsMotionOwnerLeaseCurrent(s.lease)||
                m_motion.GetCurrentExecutionEpoch()!=s.capture.epoch)reason="CAPTURE_START";
        }
        if(!reason){
            s.phase=TunePhase::Admission;
            if(m_motion.HasExactProgramStartQuiescenceAcknowledgement(s.capture.epoch,s.lease)){
                if(!PositionAutoTune::Capture().Start(s.capture))reason="CAPTURE_START";
                else{s.captureStarted=true;s.phase=s.capture.pointToPoint?TunePhase::ApproachReady:TunePhase::CaptureReady;}
            }
            if(!reason){
                s.active=true;PublishPositionAutoTuneSameThread();
                TunePrintDiagnosticLine("[TUNE10] event=%s session=%llu axis=%d mode=%u machineProfile=%u configHash=%llu gainsChanged=0\n",
                    s.captureStarted?"BEGIN":"ADMISSION_WAIT",static_cast<unsigned long long>(s.session),s.axis,s.mode,
                    static_cast<unsigned>(s.config.machineProfile),static_cast<unsigned long long>(s.identity));
                return &NCManager::WaitPositionAutoTuneCallback;
            }
        }
    }catch(...){reason="ALLOCATION_OR_CONFIG_EXCEPTION";}
    s.failed=true;s.done=true;s.active=false;s.phase=TunePhase::Done;
    std::snprintf(s.failure,sizeof(s.failure),"%s",reason?reason:"ADMISSION");PublishPositionAutoTuneSameThread();
    TunePrintDiagnosticLine("[TUNE10] event=ADMISSION result=REJECT line=%d reason=%s motion=0 gainsChanged=0\n",sourceLine,s.failure);
    AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY,sourceLine);m_state=NCState::ALARM;
    return nullptr;
}

void NCManager::AbortPositionAutoTuneSameThread(const char* reason,bool raiseAlarm) noexcept {
    if(!m_positionAutoTune)return;
    auto& s=*m_positionAutoTune;
    if(!s.active||s.failed)return;
    auto& capture=PositionAutoTune::Capture();
    PositionAutoTune::CaptureFaultSnapshot fault{};
    const bool hasFault=s.captureStarted&&capture.TryReadFault(fault)&&fault.session==s.session;
    // A guard can request the RT stop before the NC scan observes its Fault.
    // Preserve that first fault if it becomes visible during the authority
    // check as well. This changes attribution only; every abort still stops.
    if(hasFault&&std::strcmp(reason,"AUTHORITY_OR_PROGRAM_CHANGED")==0)
        reason=PositionAutoTune::CaptureReasonName(fault.reason);
    const auto currentLease=m_motion.GetMotionOwnerLease();
    const auto currentEpoch=m_motion.GetCurrentExecutionEpoch();
    const auto currentCache=GetBaseProgramCache().GetGeneration();
    const auto currentRun=m_pathCoreLiveBookkeeping.currentRunToken;
    const auto currentPhase=s.phase;
    const auto currentCaptureState=capture.State();
    const bool currentSafety=m_motion.HasPendingSafetyOrRecoveryRequests();
    const bool currentAlarm=AlarmManager::GetInstance().HasAlarm();
    const NCState currentState=m_state.load(std::memory_order_acquire);
    const auto currentMode=m_mode;
    const auto currentResetPhase=m_resetContinuationPhase;
    const bool currentClose=Close_System_Com_flag;
    PositionAutoTune::CaptureSample timeoutSample{};
    const bool hasTimeoutSample=s.captureStarted&&
        (std::strcmp(reason,"SETTLE_TIMEOUT")==0||std::strcmp(reason,"APPROACH_SETTLE_TIMEOUT")==0)&&
        capture.TryReadLatest(timeoutSample)&&timeoutSample.session==s.session;
    // Freeze these observations before the stop, but defer all diagnostic IO
    // until the existing stop request has been issued. No RT-side printing.
    // Separately read values are diagnostic, not coherent stop proof.
    const auto printDiagnostic=[&]() noexcept {
    TunePrintDiagnosticLine("[TUNE10] event=ABORT_CONTEXT session=%llu axis=%d reason=%s phase=%u completedLegs=%u captureStarted=%u captureState=%u captureReason=%s owner=%u/%u expectedOwner=%u/%u ownerMatch=%u epoch=%u expectedEpoch=%u cache=%llu expectedCache=%llu run=%llu expectedRun=%llu safety=%u alarm=%u ncState=%u mode=%u resetPhase=%u close=%u observation=PRE_ABORT\n",
        static_cast<unsigned long long>(s.session),s.axis,reason,static_cast<unsigned>(currentPhase),s.leg,
        s.captureStarted?1U:0U,static_cast<unsigned>(currentCaptureState),hasFault?PositionAutoTune::CaptureReasonName(fault.reason):"NONE",
        static_cast<unsigned>(currentLease.owner),currentLease.generation,static_cast<unsigned>(s.lease.owner),s.lease.generation,
        currentLease.Matches(s.lease)?1U:0U,currentEpoch,s.capture.epoch,
        static_cast<unsigned long long>(currentCache),static_cast<unsigned long long>(s.cache),
        static_cast<unsigned long long>(currentRun),static_cast<unsigned long long>(s.run),
        currentSafety?1U:0U,currentAlarm?1U:0U,
        static_cast<unsigned>(currentState),static_cast<unsigned>(currentMode),static_cast<unsigned>(currentResetPhase),currentClose?1U:0U);
    if(hasTimeoutSample&&s.axis>=0&&s.axis<static_cast<int>(PositionAutoTune::kAxisCount)) {
        const auto& profile=s.config.axis[s.axis];
        const double ppu=s.capture.pulsesPerMm;
        const double endpointSeconds=s.commandCompleteUs&&timeoutSample.timestampUs>=s.commandCompleteUs?
            static_cast<double>(timeoutSample.timestampUs-s.commandCompleteUs)*1e-6:0.0;
        const double observationSeconds=s.observationStartedUs&&timeoutSample.timestampUs>=s.observationStartedUs&&
            timeoutSample.idleWindowDurationUs>=timeoutSample.timestampUs-s.observationStartedUs?
            static_cast<double>(timeoutSample.timestampUs-s.observationStartedUs)*1e-6:0.0;
        const double requiredObservation=(std::max)(profile.dwellSeconds,
            profile.idleEnabled?profile.idleObservationSeconds:0.0);
        // Diagnostic only: keep timeout, drift proof and position guards unchanged.
        // A small error at the abort instant is not proof of a complete IDLE tail.
        TunePrintDiagnosticLine("[TUNE10] event=SETTLE_DIAGNOSTIC session=%llu axis=%d leg=%u reason=%s cycle=%llu commandNative=%.9g actualNative=%.9g signedLagNative=%.9g windowNative=%.9g endpointSeconds=%.9g settleTimeoutSeconds=%.9g settleHoldSeconds=%.9g continuousSettleSeconds=%.9g continuousIdleWindowSeconds=%.9g observationSeconds=%.9g requiredObservationSeconds=%.9g state=%u bank=%u referencePps=%.9g pTermPps=%.9g iTermPps=%.9g integral=%.9g evidence=PARTIAL gainsChanged=0\n",
            static_cast<unsigned long long>(s.session),s.axis,s.leg,reason,
            static_cast<unsigned long long>(timeoutSample.cycle),timeoutSample.commandPulse/ppu,
            timeoutSample.actualPulse/ppu,(timeoutSample.commandPulse-timeoutSample.actualPulse)/ppu,
            profile.settleWindowMm,endpointSeconds,profile.settleTimeoutSeconds,profile.settleHoldSeconds,
            static_cast<double>(timeoutSample.settledDurationUs)*1e-6,
            static_cast<double>(timeoutSample.idleWindowDurationUs)*1e-6,observationSeconds,requiredObservation,
            timeoutSample.motionState,timeoutSample.gainBank,timeoutSample.referencePps,
            timeoutSample.pTerm,timeoutSample.iTerm,timeoutSample.integral);
    }
    if(hasFault){
        TunePrintDiagnosticLine("[TUNE10] event=CAPTURE_FAULT session=%llu reason=%s stage=%s sampleCount=%llu lastGoodCycle=%llu lastGoodUs=%llu hasCycle=%u cycle=%llu timeUs=%llu cycleOpen=%u sampleStaged=%u sourceValid=%u clockValid=%u safetyGood=%u layoutGood=%u owner=%u/%u epoch=%u hasObservedValue=%u observedValue=%.17g observedValid=%u observedAxis=%d\n",
            static_cast<unsigned long long>(fault.session),PositionAutoTune::CaptureReasonName(fault.reason),PositionAutoTune::CaptureFaultStageName(fault.stage),
            static_cast<unsigned long long>(fault.sampleCount),static_cast<unsigned long long>(fault.lastGoodCycle),static_cast<unsigned long long>(fault.lastGoodTimestampUs),
            fault.hasCycle?1U:0U,static_cast<unsigned long long>(fault.cycle.cycle),static_cast<unsigned long long>(fault.cycle.timestampUs),
            fault.cycleOpen?1U:0U,fault.sampleStaged?1U:0U,fault.cycle.sourceValid?1U:0U,fault.cycle.clockValid?1U:0U,
            fault.cycle.safetyGood?1U:0U,fault.cycle.layoutGood?1U:0U,fault.cycle.owner,fault.cycle.ownerGeneration,fault.cycle.epoch,
            fault.hasObservedValue?1U:0U,fault.observedValue,fault.observedValid?1U:0U,fault.observedAxis);
        if(fault.hasSample){
            const auto& f=fault.sample;
            TunePrintDiagnosticLine("[TUNE10] event=FAULT_SAMPLE session=%llu stage=%s commandPulse=%.17g actualPulse=%.17g errorPulse=%.17g referencePps=%.17g actualVelocityPps=%.17g proposedPps=%.17g outputPps=%.17g state=%u bank=%u valid=%u saturated=%u kp=%.17g ki=%.17g kd=%.17g kvff=%.17g integral=%.17g\n",
                static_cast<unsigned long long>(s.session),PositionAutoTune::CaptureFaultStageName(fault.stage),f.commandPulse,f.actualPulse,
                f.commandPulse-f.actualPulse,f.referencePps,f.actualVelocityPps,f.proposedLogicalPps,f.outputLogicalPps,
                f.motionState,f.gainBank,f.valid,f.saturated,f.kp,f.ki,f.kd,f.kvff,f.integral);
        }
        if(fault.velocity.present){
            const auto& v=fault.velocity;
            TunePrintDiagnosticLine("[TUNE10] event=FAULT_VELOCITY session=%llu reason=%s windowReady=%u intervals=%u elapsedUs=%llu estimationUs=%llu estimatedPps=%.17g actualLimitPps=%.17g hasPositionStep=%u positionStepPulse=%.17g rawStepPulse=%.17g stepLimitPulse=%.17g windowCycles=4 nominalWindowUs=1000 observation=REJECTED_BEFORE_OUTPUT\n",
                static_cast<unsigned long long>(s.session),PositionAutoTune::CaptureReasonName(fault.reason),
                v.windowReady?1U:0U,v.intervals,static_cast<unsigned long long>(v.elapsedUs),
                static_cast<unsigned long long>(v.estimationUs),v.estimatedPps,v.limitPps,
                v.hasPositionStep?1U:0U,v.positionStepPulse,v.rawStepPulse,v.stepLimitPulse);
        }
        TunePrintDiagnosticLine("[TUNE10] event=FAULT_LIMITS session=%llu pulsesPerUnit=%.17g originPulse=%.17g excursionPulse=%.17g boundaryMarginPulse=%.17g followingLimitPulse=%.17g settleWindowPulse=%.17g maximumVelocityPps=%.17g maximumReferenceVelocityPps=%.17g actualVelocityLimitPps=%.17g maximumTicks=%llu pointToPoint=%u approachActive=%u\n",
            static_cast<unsigned long long>(s.session),s.capture.pulsesPerMm,s.capture.originPulse,s.capture.excursionPulse,s.capture.boundaryMarginPulse,
            s.capture.followingLimitPulse,s.capture.settleWindowPulse,s.capture.maximumVelocityPps,
            PositionAutoTune::ReferenceVelocityLimitPps(s.capture),
            PositionAutoTune::ReferenceVelocityLimitPps(s.capture)*1.5+1.0/PositionAutoTune::CapturePeriodSeconds,static_cast<unsigned long long>(s.capture.maximumTicks),
            s.capture.pointToPoint?1U:0U,fault.approachActive?1U:0U);
    }
    };
    s.failed=true;s.cancelled=true;std::snprintf(s.failure,sizeof(s.failure),"%s",reason);
    if(!s.captureStarted){
        // Nothing was armed or submitted for this session. Never cancel or
        // report a previous session's terminal recorder while waiting here.
        s.done=true;s.active=false;s.phase=TunePhase::Done;
        if(raiseAlarm&&!AlarmManager::GetInstance().HasAlarm()){
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY,s.sourceLine);m_state=NCState::ALARM;
        }
        printDiagnostic();
        TunePrintDiagnosticLine("[TUNE10] event=ADMISSION result=ABORT session=%llu reason=%s motion=0 gainsChanged=0\n",
            static_cast<unsigned long long>(s.session),s.failure);
        PublishPositionAutoTuneSameThread();return;
    }
    PositionAutoTune::Capture().Cancel();
    if(s.jobStarted)PositionAutoTune::CancelJob(s.session);
    m_motion.RequestEmergencyStopAllAxes();s.phase=TunePhase::Cancel;
    printDiagnostic();
    TunePrintDiagnosticLine("[TUNE10] event=ABORT session=%llu axis=%d reason=%s returnToOrigin=0 resume=RESET_REQUIRED\n",
        static_cast<unsigned long long>(s.session),s.axis,s.failure);
    if(raiseAlarm&&!AlarmManager::GetInstance().HasAlarm()){
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_MOTION_NOT_ADMITTED,s.sourceLine);m_state=NCState::ALARM;
    }
    PublishPositionAutoTuneSameThread();
}

bool NCManager::AdmitPositionAutoTuneResetSameThread() noexcept {
    if(!m_positionAutoTune)return true;
    auto& s=*m_positionAutoTune;
    if(s.resetAdmitted)return true; // The existing Reset may need multiple task continuations.
    if(s.resetReleased){s.resetReleased=false;s.resetBypass=true;s.resetAdmitted=true;return true;}
    if(s.resetPending)return false;
    auto& capture=PositionAutoTune::Capture();
    if(s.active&&!s.captureStarted){
        AbortPositionAutoTuneSameThread("RESET",false);
        // The ordinary Reset owns the stop transaction; no TUNE RT ticket exists.
        return true;
    }
    if(!s.active&&!capture.Active()&&!capture.RuntimeNeedsStop())return true;
    AbortPositionAutoTuneSameThread("RESET",false);
    // Also retire an RT stop after the NC worker has already become inactive.
    // A terminal capture plus a drained ticket is not physical stop proof;
    // the ordinary Reset transaction below still owns that proof.
    if(capture.Active())capture.Cancel();
    s.resetPending=true;m_state=NCState::RESET_STATE;
    return false;
}

bool NCManager::ConsumePositionAutoTuneResetBypassSameThread() noexcept {
    if(!m_positionAutoTune)return false;
    const bool bypass=m_positionAutoTune->resetBypass;
    m_positionAutoTune->resetBypass=false;return bypass;
}

void NCManager::ServicePositionAutoTuneSameThread() noexcept {
    if(!m_positionAutoTune||(!m_positionAutoTune->active&&!m_positionAutoTune->resetPending))return;
    auto& s=*m_positionAutoTune;
    const auto publishPhaseChange=[this,&s]() noexcept {
        if(s.phase!=s.livePublishedPhase||!s.lastLivePublishClock||
            TuneElapsed(s.lastClock,s.lastLivePublishClock,s.frequency)>=0.05)
            PublishPositionAutoTuneSameThread();
    };
    struct LiveExitPublication {
        const decltype(publishPhaseChange)& publish;
        ~LiveExitPublication() noexcept {publish();}
    } liveExitPublication{publishPhaseChange};
    // Retire the diagnostic stop before the ordinary Reset captures its exact
    // safety provenance. Never publish a second late stop after that baseline.
    if(s.resetPending&&PositionAutoTune::Capture().Terminal()&&
        !PositionAutoTune::Capture().RuntimeNeedsStop()&&!m_motion.HasPendingSafetyOrRecoveryRequests()){
        s.resetPending=false;s.resetReleased=true;Reset();
    }
    if(!s.active)return;
    auto& capture=PositionAutoTune::Capture();std::uint64_t now=0ULL;
    if(!TuneClock(now)||now<s.lastClock){AbortPositionAutoTuneSameThread("CLOCK",true);now=s.lastClock;}
    s.lastClock=now;
    if(!s.failed){
        const bool reset=m_state==NCState::RESET_STATE||m_resetContinuationPhase!=ResetContinuationPhase::IDLE;
        if(reset)AbortPositionAutoTuneSameThread("RESET",false);
        else if(m_state!=NCState::RUN||m_mode!=NCOperationMode::MEMORY||Close_System_Com_flag)
            AbortPositionAutoTuneSameThread("HOLD_OR_MODE_OR_CLOSE",true);
        // The RT guard already requested its stop. Its original fault must
        // precede the resulting safety/owner/epoch changes in the NC report.
        else if(s.captureStarted&&capture.State()==PositionAutoTune::CaptureState::Fault)
            AbortPositionAutoTuneSameThread(PositionAutoTune::CaptureReasonName(capture.Reason()),true);
        else if(AlarmManager::GetInstance().HasAlarm()||m_motion.HasPendingSafetyOrRecoveryRequests()||
            !m_motion.IsMotionOwnerLeaseCurrent(s.lease)||m_motion.GetCurrentExecutionEpoch()!=s.capture.epoch||
            GetBaseProgramCache().GetGeneration()!=s.cache||m_pathCoreLiveBookkeeping.currentRunToken!=s.run)
            AbortPositionAutoTuneSameThread("AUTHORITY_OR_PROGRAM_CHANGED",true);
        else if(TuneIdentity(s.config,m_motion.GetAxisContext(s.axis),s.axis,s.capture.pointToPoint,s.capture.sampleStride)!=s.identity||
            m_motion.GetAxisContext(s.axis).feedrateOverride!=1.0)
            AbortPositionAutoTuneSameThread("CONFIGURATION_CHANGED",true);
        else if(s.phase!=TunePhase::Analysis&&TuneElapsed(now,s.startClock,s.frequency)>s.sessionBudgetSeconds)
            AbortPositionAutoTuneSameThread("SESSION_TIMEOUT",true);
    }
    if(!s.active)return;
    if(s.phase==TunePhase::Admission){
        if(TuneElapsed(now,s.phaseClock,s.frequency)>TuneAdmissionWaitSeconds){
            TuneAdmissionDiagnostic(m_motion.GetStopSettleSnapshot(),s.session);
            s.failed=true;s.done=true;s.active=false;s.phase=TunePhase::Done;
            std::snprintf(s.failure,sizeof(s.failure),"%s","AXES_NOT_READY_TIMEOUT");
            TunePrintDiagnosticLine("[TUNE10] event=ADMISSION result=REJECT session=%llu reason=%s motion=0 gainsChanged=0\n",
                static_cast<unsigned long long>(s.session),s.failure);
            PublishPositionAutoTuneSameThread();
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY,s.sourceLine);m_state=NCState::ALARM;
            return;
        }
        const auto& axis=m_motion.GetAxisContext(s.axis);
        if(axis.currentCmdPos!=s.initialCommandPulse||!TuneSameGains(axis.Pid_G00,s.capture.motionGains)||
            !TuneSameGains(axis.Pid_IDLE,s.capture.idleGains)){
            AbortPositionAutoTuneSameThread("ORIGIN_OR_GAINS_CHANGED",true);return;
        }
        if(s.capture.pointToPoint) {
            if(!TuneApproachPose(s.capture,axis.currentCmdPos,axis.currentActPos)) {
                AbortPositionAutoTuneSameThread("APPROACH_START_OUTSIDE_ENVELOPE",true);return;
            }
        } else {
            double rangeMinimum=0.0,rangeMaximum=0.0;
            if(const char* rangeReason=TuneRangeAtOrigin(s.config.axis[s.axis],axis.currentCmdPos,
                axis.resolution_PPR/axis.finalLead,rangeMinimum,rangeMaximum)){
                AbortPositionAutoTuneSameThread(rangeReason,true);return;
            }
        }
        if(!m_motion.HasExactProgramStartQuiescenceAcknowledgement(s.capture.epoch,s.lease))return;
        if(!capture.Start(s.capture)){AbortPositionAutoTuneSameThread("CAPTURE_START",true);return;}
        s.captureStarted=true;s.phase=s.capture.pointToPoint?TunePhase::ApproachReady:TunePhase::CaptureReady;
        s.startClock=s.phaseClock=s.lastFreshClock=now;
        TunePrintDiagnosticLine("[TUNE10] event=BEGIN session=%llu axis=%d mode=%u machineProfile=%u configHash=%llu gainsChanged=0\n",
            static_cast<unsigned long long>(s.session),s.axis,s.mode,static_cast<unsigned>(s.config.machineProfile),
            static_cast<unsigned long long>(s.identity));
        PublishPositionAutoTuneSameThread();return;
    }
    if(s.phase==TunePhase::Cancel){
        if(s.jobStarted){
            std::shared_ptr<const PositionAutoTune::JobResult> result;
            if(PositionAutoTune::PollJob(s.session,result)){s.result=result;s.done=true;s.active=false;s.phase=TunePhase::Done;PublishPositionAutoTuneSameThread();}
            return;
        }
        if(!capture.Terminal())return;
        const PositionAutoTune::CaptureSample* savedSamples=capture.Samples();
        std::size_t savedCount=capture.SampleCount();
        if(s.capture.pointToPoint) {
            PositionAutoTune::CaptureMeasurementBoundary boundary{};
            if(capture.TryReadMeasurementBoundary(s.session,boundary)&&boundary.sampleOffset<savedCount) {
                // TUNE09: failed full-rate Q1 captures must use the same formal
                // measurement boundary as completed and decimated captures.
                // Retaining approach rows otherwise contradicts the raw statistics.
                savedSamples+=boundary.sampleOffset;savedCount-=boundary.sampleOffset;
                s.measurementOffset=boundary.sampleOffset;s.measurementStartCycle=boundary.cycle;
                s.approachSeconds=static_cast<double>(boundary.approachDurationUs)*1e-6;
            }
        }
        // Preserve failed captures for diagnosis, with no candidate/baseline permission.
        try{s.job.config=s.config;s.job.capture=s.capture;s.job.mode=s.mode;s.job.session=s.session;
            s.job.configIdentityHash=s.identity;s.job.cruiseBaselineRevision=s.cruiseBaselineRevision;s.job.segments=s.segments;s.job.measurementComplete=false;s.job.failureReason=s.failure;
            (void)capture.TryReadStatistics(s.job.statistics);
            s.job.autoApproachUsed=s.capture.pointToPoint;s.job.approachStartNative=s.initialCommandPulse/s.capture.pulsesPerMm;
            s.job.approachTargetNative=s.capture.originPulse/s.capture.pulsesPerMm;
            s.job.measurementStartCycle=s.measurementStartCycle;s.job.approachSeconds=s.approachSeconds;
            s.job.outputDir=GlobalConfig::GetInstance().BaseDataDir+"PositionAutoTune\\";
            s.jobStarted=PositionAutoTune::StartJob(s.job,savedSamples,savedCount);
        }catch(...){s.jobStarted=false;}
        if(!s.jobStarted){s.done=true;s.active=false;s.phase=TunePhase::Done;PublishPositionAutoTuneSameThread();}
        return;
    }
    if(s.phase==TunePhase::Analysis){
        std::shared_ptr<const PositionAutoTune::JobResult> result;
        if(!PositionAutoTune::PollJob(s.session,result))return;
        s.result=result;s.done=true;s.active=false;s.phase=TunePhase::Done;
        if(!result){s.failed=true;std::snprintf(s.failure,sizeof(s.failure),"WORKER_FAILED");
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY,s.sourceLine);m_state=NCState::ALARM;}
        PublishPositionAutoTuneSameThread();return;
    }
    if(s.phase==TunePhase::Finish){
        if(capture.State()!=PositionAutoTune::CaptureState::Complete)return;
        if(s.capture.pointToPoint&&(!s.measurementStartCycle||s.measurementOffset>=capture.SampleCount())) {
            AbortPositionAutoTuneSameThread("MEASUREMENT_BOUNDARY_MISSING",true);return;
        }
        try{s.job.config=s.config;s.job.capture=s.capture;s.job.mode=s.mode;s.job.session=s.session;
            s.job.configIdentityHash=s.identity;s.job.cruiseBaselineRevision=s.cruiseBaselineRevision;s.job.segments=s.segments;s.job.measurementComplete=true;
            if(!capture.TryReadStatistics(s.job.statistics)&&s.capture.sampleStride>1U) {
                AbortPositionAutoTuneSameThread("FULL_RATE_STATISTICS_MISSING",true);return;
            }
            s.job.autoApproachUsed=s.capture.pointToPoint;s.job.approachStartNative=s.initialCommandPulse/s.capture.pulsesPerMm;
            s.job.approachTargetNative=s.capture.originPulse/s.capture.pulsesPerMm;
            s.job.measurementStartCycle=s.measurementStartCycle;s.job.approachSeconds=s.approachSeconds;
            s.job.outputDir=GlobalConfig::GetInstance().BaseDataDir+"PositionAutoTune\\";
            s.jobStarted=PositionAutoTune::StartJob(s.job,capture.Samples()+s.measurementOffset,capture.SampleCount()-s.measurementOffset);
        }catch(...){s.jobStarted=false;}
        if(!s.jobStarted){AbortPositionAutoTuneSameThread("WORKER_START",true);return;}
        s.phase=TunePhase::Analysis;PublishPositionAutoTuneSameThread();return;
    }
    PositionAutoTune::CaptureSample sample{};
    if(!capture.TryReadLatest(sample)||sample.session!=s.session||sample.cycle<=s.lastCycle){
        if(TuneElapsed(now,s.lastFreshClock,s.frequency)>0.25)AbortPositionAutoTuneSameThread("NO_FRESH_CAPTURE",true);
        return;
    }
    s.lastCycle=sample.cycle;s.lastFreshClock=now;
    if(!s.firstCaptureUs)s.firstCaptureUs=sample.timestampUs;
    const auto& profile=s.config.axis[s.axis];
    const double observationDwell=(std::max)(profile.dwellSeconds,profile.idleEnabled?profile.idleObservationSeconds:0.0);
    if(s.phase==TunePhase::ApproachReady) {
        if(capture.State()!=PositionAutoTune::CaptureState::Running)return;
        if(std::fabs(sample.commandPulse-s.capture.originPulse)<=1.0) {
            s.commandCompleteUs=sample.timestampUs;s.phase=TunePhase::ApproachMoving;s.phaseClock=now;
        } else {
            bool ingressBusy=false;s.targetPulse=s.capture.originPulse;
            if(!m_motion.SubmitAxisMoveToPosition(s.axis,s.targetPulse,profile.speedMmMin*s.capture.pulsesPerMm/60.0,
                profile.accelerationSeconds,profile.decelerationSeconds,false,MotionCommandSource::NC_MEMORY,
                s.lease,&s.command,&ingressBusy)) {
                if(!ingressBusy||TuneElapsed(now,s.phaseClock,s.frequency)>0.25)
                    AbortPositionAutoTuneSameThread("APPROACH_MOVE_NOT_ADMITTED",true);
                return;
            }
            s.commandCompleteUs=0ULL;s.phase=TunePhase::ApproachApplied;s.phaseClock=now;
            TunePrintDiagnosticLine("[TUNE10] event=APPROACH_BEGIN session=%llu axis=%d fromMCS=%.12g targetMCS=%.12g speedNativePerMin=%.12g gainsChanged=0\n",
                static_cast<unsigned long long>(s.session),s.axis,sample.commandPulse/s.capture.pulsesPerMm,
                s.capture.originPulse/s.capture.pulsesPerMm,profile.speedMmMin);
            return;
        }
    }
    if(s.phase==TunePhase::ApproachApplied) {
        MotionAxisCommandResult ack{};
        if(!m_motion.TryGetAxisCommandResult(s.command,ack)) {
            if(TuneElapsed(now,s.phaseClock,s.frequency)>1.0)AbortPositionAutoTuneSameThread("APPROACH_ACK_TIMEOUT",true);
            return;
        }
        if(ack.resultType!=MotionAxisCommandResultType::APPLIED||ack.axisIndex!=s.axis||
            ack.commandType!=MotionAxisCommandType::MOVE_TO_POSITION||ack.owner!=s.lease.owner||
            ack.ownerGeneration!=s.lease.generation) {
            AbortPositionAutoTuneSameThread("APPROACH_MOVE_REJECTED",true);return;
        }
        s.phase=TunePhase::ApproachMoving;
    }
    if(s.phase==TunePhase::ApproachMoving) {
        const bool endpoint=std::fabs(sample.commandPulse-s.capture.originPulse)<=1.0&&std::fabs(sample.referencePps)<=1.0;
        if(endpoint&&!s.commandCompleteUs)s.commandCompleteUs=sample.timestampUs;
        if(s.commandCompleteUs&&sample.timestampUs>=s.commandCompleteUs&&
            sample.timestampUs-s.commandCompleteUs>static_cast<std::uint64_t>(profile.settleTimeoutSeconds*1e6)) {
            AbortPositionAutoTuneSameThread("APPROACH_SETTLE_TIMEOUT",true);return;
        }
        if(!endpoint||sample.motionState!=static_cast<unsigned>(MotionState::MotionState_IDLE)||
            sample.settledDurationUs<s.capture.measurementSettleHoldUs)return;
        if(!capture.RequestMeasurement(s.session)){AbortPositionAutoTuneSameThread("MEASUREMENT_ARM_FAILED",true);return;}
        s.phase=TunePhase::MeasurementArm;s.phaseClock=now;return;
    }
    if(s.phase==TunePhase::MeasurementArm) {
        PositionAutoTune::CaptureMeasurementBoundary boundary{};
        if(!capture.TryReadMeasurementBoundary(s.session,boundary)) {
            if(TuneElapsed(now,s.phaseClock,s.frequency)>profile.settleTimeoutSeconds)
                AbortPositionAutoTuneSameThread("MEASUREMENT_ARM_TIMEOUT",true);
            return;
        }
        const auto boundaryRead=TuneMeasurementBoundaryRead(boundary,sample,capture.SampleCount(),s.firstCaptureUs);
        if(boundaryRead==TuneBoundaryRead::Invalid) {
            AbortPositionAutoTuneSameThread("MEASUREMENT_BOUNDARY_INVALID",true);return;
        }
        // Latest sample and the one-time boundary have separate acquire
        // publications. RT may publish the boundary after our sample read.
        if(boundaryRead==TuneBoundaryRead::AwaitLatest) {
            if(TuneElapsed(now,s.phaseClock,s.frequency)>profile.settleTimeoutSeconds)
                AbortPositionAutoTuneSameThread("MEASUREMENT_ARM_TIMEOUT",true);
            return;
        }
        s.measurementOffset=boundary.sampleOffset;s.measurementStartCycle=boundary.cycle;
        s.approachSeconds=static_cast<double>(boundary.approachDurationUs)*1e-6;
        s.previousTargetPulse=s.capture.originPulse;s.commandCompleteUs=0ULL;s.observationStartedUs=0ULL;
        s.phase=TunePhase::CaptureReady;s.phaseClock=now;
        TunePrintDiagnosticLine("[TUNE10] event=MEASUREMENT_BEGIN session=%llu axis=%d sampleOffset=%llu cycle=%llu approachSeconds=%.12g endpointA=%.12g endpointB=%.12g pointToPoint=1\n",
            static_cast<unsigned long long>(s.session),s.axis,static_cast<unsigned long long>(boundary.sampleOffset),
            static_cast<unsigned long long>(boundary.cycle),s.approachSeconds,s.capture.originPulse/s.capture.pulsesPerMm,
            (s.capture.originPulse+s.capture.excursionPulse)/s.capture.pulsesPerMm);
        return;
    }
    if(s.phase==TunePhase::CaptureReady){
        if(capture.State()!=PositionAutoTune::CaptureState::Running)return;
        if(TuneElapsed(now,s.phaseClock,s.frequency)>profile.settleTimeoutSeconds){
            AbortPositionAutoTuneSameThread("ORIGIN_NOT_SETTLED",true);return;
        }
        if(std::fabs(sample.commandPulse-s.capture.originPulse)>1.0||
            sample.settledDurationUs<static_cast<std::uint64_t>(profile.settleHoldSeconds*1e6))return;
        s.phase=TunePhase::Submit;s.phaseClock=now;
    }
    if(s.phase==TunePhase::Submit){
        if(s.leg>=4U*profile.cycles){capture.RequestFinish();s.phase=TunePhase::Finish;return;}
        const unsigned direction=s.leg%4U;
        s.targetPulse=s.capture.pointToPoint?s.capture.originPulse+((s.leg%2U)==0U?s.capture.excursionPulse:0.0):
            s.capture.originPulse+(direction==0U?s.capture.excursionPulse:direction==2U?-s.capture.excursionPulse:0.0);
        bool ingressBusy=false;
        capture.SetTelemetryLeg(s.session,static_cast<std::uint64_t>(s.leg)+1ULL,
            sample.cycle+1ULL,s.previousTargetPulse,s.targetPulse);
        // All four endpoints are continuous pulse positions relative to the captured
        // origin. false disables the consumer's shortest-path rewrite, including
        // rotary moves through 0/360 or an already accumulated multi-turn origin.
        if(!m_motion.SubmitAxisMoveToPosition(s.axis,s.targetPulse,profile.speedMmMin*s.capture.pulsesPerMm/60.0,
            profile.accelerationSeconds,profile.decelerationSeconds,false,MotionCommandSource::NC_MEMORY,
            s.lease,&s.command,&ingressBusy)){
            if(!ingressBusy||TuneElapsed(now,s.phaseClock,s.frequency)>0.25)AbortPositionAutoTuneSameThread("MOVE_NOT_ADMITTED",true);
            return;
        }
        PositionAutoTune::AnalysisSegment segment{};segment.firstCycle=sample.cycle+1ULL;
        segment.startPulse=s.previousTargetPulse;segment.targetPulse=s.targetPulse;s.segments.push_back(segment);
        s.commandCompleteUs=0ULL;s.observationStartedUs=0ULL;s.phase=TunePhase::Applied;s.phaseClock=now;return;
    }
    if(s.phase==TunePhase::Applied){
        MotionAxisCommandResult ack{};
        if(!m_motion.TryGetAxisCommandResult(s.command,ack)){
            if(TuneElapsed(now,s.phaseClock,s.frequency)>1.0)AbortPositionAutoTuneSameThread("MOVE_ACK_TIMEOUT",true);
            return;
        }
        if(ack.resultType!=MotionAxisCommandResultType::APPLIED||ack.axisIndex!=s.axis||
            ack.commandType!=MotionAxisCommandType::MOVE_TO_POSITION||ack.owner!=s.lease.owner||
            ack.ownerGeneration!=s.lease.generation){AbortPositionAutoTuneSameThread("MOVE_REJECTED",true);return;}
        s.phase=TunePhase::Moving;
    }
    if(s.phase==TunePhase::Moving){
        auto& segment=s.segments.back();
        const bool endpoint=sample.cycle>=segment.firstCycle&&std::fabs(sample.commandPulse-s.targetPulse)<=1.0&&
            std::fabs(sample.referencePps)<=1.0;
        if(endpoint&&!segment.commandCompleteCycle){segment.commandCompleteCycle=sample.cycle;s.commandCompleteUs=sample.timestampUs;}
        if(segment.commandCompleteCycle&&sample.timestampUs>=s.commandCompleteUs&&
            static_cast<double>(sample.timestampUs-s.commandCompleteUs)>(profile.settleTimeoutSeconds+observationDwell)*1e6){
            AbortPositionAutoTuneSameThread("SETTLE_TIMEOUT",true);return;
        }
        const bool idleEndpoint=endpoint&&sample.motionState==static_cast<unsigned>(MotionState::MotionState_IDLE)&&
            sample.gainBank==static_cast<unsigned>(PositionAutoTune::GainBank::Idle)&&
            sample.timestampUs>=s.commandCompleteUs;
        if(!idleEndpoint){s.observationStartedUs=0ULL;return;}
        // TUNE05: prove the short settle hold first, then observe the complete
        // IDLE window. Drift during an in-window observation remains measured
        // evidence; it must not restart the whole observation. The independent
        // RT counter still resets on every position/window/command/bank break.
        if(s.observationStartedUs&&(sample.timestampUs<s.observationStartedUs||
            sample.idleWindowDurationUs<sample.timestampUs-s.observationStartedUs))
            s.observationStartedUs=0ULL;
        if(!s.observationStartedUs){
            if(sample.settledDurationUs<static_cast<std::uint64_t>(profile.settleHoldSeconds*1e6))return;
            s.observationStartedUs=sample.timestampUs;
        }
        const std::uint64_t observationUs=sample.timestampUs-s.observationStartedUs;
        if(static_cast<double>(observationUs)<observationDwell*1e6)return;
        segment.lastCycle=sample.cycle;s.previousTargetPulse=s.targetPulse;++s.leg;
        TunePrintDiagnosticLine("[TUNE10] event=LEG_DONE session=%llu axis=%d leg=%u total=%u continuousSettleUs=%llu idleWindowUs=%llu observationUs=%llu\n",
            static_cast<unsigned long long>(s.session),s.axis,s.leg,4U*profile.cycles,
            static_cast<unsigned long long>(sample.settledDurationUs),
            static_cast<unsigned long long>(sample.idleWindowDurationUs),static_cast<unsigned long long>(observationUs));
        s.phase=TunePhase::Submit;s.phaseClock=now;
    }
}

void NCManager::PublishPositionAutoTuneSameThread() noexcept {
    if(!m_positionAutoTune)return;
    auto& s=*m_positionAutoTune;
    s.lastLivePublishClock=s.lastClock;s.livePublishedPhase=s.phase;
    for(int i=MacroVariableRules::TuningResultFirst;i<=MacroVariableRules::TuningResultLast;++i)MacroSys.SetSysVar(i,0.0);
    const auto* result=s.result.get();const auto* analysis=result?&result->analysis:nullptr;
    const bool candidate=!s.failed&&result&&!result->cancelled&&(s.mode==1U?result->analysis.candidateValid:result->baselineMatched);
    const PositionAutoTune::CaptureGains* suggested=result?
        (s.mode==1U?&result->analysis.suggested:&result->baselineMetrics.suggested):nullptr;
    unsigned status=s.failed?(s.cancelled?7U:6U):s.active?(s.phase==TunePhase::Analysis?2U:1U):
        result?(result->verified?5U:s.mode==1U&&candidate?3U:4U):0U;
    MacroSys.SetSysVar(902,static_cast<double>(s.session));MacroSys.SetSysVar(903,s.axis);MacroSys.SetSysVar(904,s.mode);
    MacroSys.SetSysVar(905,s.failed?100.0:analysis?static_cast<double>(analysis->reason):0.0);
    MacroSys.SetSysVar(906,candidate?1.0:0.0);
    MacroSys.SetSysVar(907,!s.failed&&result&&result->verified?1.0:0.0);
    MacroSys.SetSysVar(908,static_cast<unsigned>(s.config.machineProfile));
    const auto& measured=s.capture.motionGains;
    MacroSys.SetSysVar(913,measured.kp);MacroSys.SetSysVar(914,measured.ki);MacroSys.SetSysVar(915,measured.kd);MacroSys.SetSysVar(916,measured.kvff);
    if(analysis){
        if(candidate&&suggested){MacroSys.SetSysVar(909,suggested->kp);MacroSys.SetSysVar(910,suggested->ki);
            MacroSys.SetSysVar(911,suggested->kd);MacroSys.SetSysVar(912,suggested->kvff);}
        MacroSys.SetSysVar(917,analysis->peakErrorMm);MacroSys.SetSysVar(918,analysis->rmsMovingErrorMm);
        MacroSys.SetSysVar(919,analysis->rmsIdleErrorMm);MacroSys.SetSysVar(920,analysis->maxOvershootMm);
        MacroSys.SetSysVar(921,analysis->maxSettlingSeconds);MacroSys.SetSysVar(922,analysis->maxSettledJitterMm);
        MacroSys.SetSysVar(923,analysis->trainingFit);MacroSys.SetSysVar(924,analysis->validationFit);
        MacroSys.SetSysVar(925,analysis->plantGain);MacroSys.SetSysVar(926,analysis->tauSeconds);MacroSys.SetSysVar(927,analysis->delaySeconds);
        MacroSys.SetSysVar(928,static_cast<double>(analysis->sampleCount));MacroSys.SetSysVar(929,analysis->segmentCount);
        MacroSys.SetSysVar(930,result->baselineSaved?1.0:0.0);MacroSys.SetSysVar(931,result->rawSaved?1.0:0.0);MacroSys.SetSysVar(932,result->summarySaved?1.0:0.0);
    }
    // TUNE03 extension: native-unit IDLE metrics and independent P2 bank results.
    const bool idleCandidate=candidate&&result&&
        (s.mode==1U?result->analysis.idleCandidateValid:result->baselineMetrics.idleCandidateValid);
    const auto* idleSuggested=idleCandidate?
        (s.mode==1U?&result->analysis.suggestedIdle:&result->baselineMetrics.suggestedIdle):nullptr;
    MacroSys.SetSysVar(933,5.0);MacroSys.SetSysVar(934,idleCandidate?1.0:0.0);
    MacroSys.SetSysVar(935,!s.failed&&result&&!result->cancelled&&result->idleVerified?1.0:0.0);
    MacroSys.SetSysVar(936,analysis?static_cast<double>(analysis->idleReason):2.0);
    if(idleSuggested){MacroSys.SetSysVar(937,idleSuggested->kp);MacroSys.SetSysVar(938,idleSuggested->ki);
        MacroSys.SetSysVar(939,idleSuggested->kd);MacroSys.SetSysVar(940,idleSuggested->kvff);}
    const auto& measuredIdle=s.capture.idleGains;
    MacroSys.SetSysVar(941,measuredIdle.kp);MacroSys.SetSysVar(942,measuredIdle.ki);
    MacroSys.SetSysVar(943,measuredIdle.kd);MacroSys.SetSysVar(944,measuredIdle.kvff);
    if(analysis){MacroSys.SetSysVar(945,analysis->idleTailRmsMm);MacroSys.SetSysVar(946,analysis->idlePeakBiasMm);
        MacroSys.SetSysVar(947,analysis->idleJitterMm);MacroSys.SetSysVar(948,analysis->idleObservedSeconds);
        MacroSys.SetSysVar(950,!s.failed&&!result->cancelled&&analysis->idleQualityPassed?1.0:0.0);}
    MacroSys.SetSysVar(949,!s.failed&&result&&!result->cancelled&&result->motionVerified?1.0:0.0);
    // TUNE04: verification above is the absolute test envelope; adoption is
    // a separate paired comparison. Never publish stale comparison evidence
    // for cancellation, incomplete P2, an unapplied bank, or a different test.
    const bool compare=s.mode==2U&&!s.failed&&result&&!result->cancelled;
    const bool motionCompare=compare&&result->motionVerified&&
        result->motionComparison!=PositionAutoTune::PerformanceComparison::Unavailable;
    const bool idleCompare=compare&&result->idleVerified&&
        result->idleComparison!=PositionAutoTune::PerformanceComparison::Unavailable;
    if(motionCompare||idleCompare) {
        MacroSys.SetSysVar(954,result->comparisonPositionTolerance);
        MacroSys.SetSysVar(955,result->comparisonTimeToleranceSeconds);
    }
    if(motionCompare) {
        MacroSys.SetSysVar(951,static_cast<unsigned>(result->motionComparison));
        MacroSys.SetSysVar(956,result->baselineMetrics.peakErrorMm);
        MacroSys.SetSysVar(957,result->baselineMetrics.rmsMovingErrorMm);
        MacroSys.SetSysVar(958,result->baselineMetrics.maxOvershootMm);
        MacroSys.SetSysVar(959,result->baselineMetrics.maxSettlingSeconds);
    }
    if(idleCompare) {
        MacroSys.SetSysVar(952,static_cast<unsigned>(result->idleComparison));
        MacroSys.SetSysVar(960,result->baselineMetrics.idleTailRmsMm);
        MacroSys.SetSysVar(961,result->baselineMetrics.idlePeakBiasMm);
        MacroSys.SetSysVar(962,result->baselineMetrics.idleJitterMm);
    }
    if(compare&&result->verified)
        MacroSys.SetSysVar(953,static_cast<unsigned>(result->adoptionAdvice));
    // TUNE17: NC alone converts a coherent pulse-domain RT observation to
    // native-unit live metrics. Existing terminal result fields stay intact.
    PositionAutoTune::CaptureLiveTelemetry observed{};
    if(!s.captureStarted){s.liveTelemetry=PositionAutoTune::CaptureLiveTelemetry{};s.hasLiveTelemetry=false;}
    if(s.captureStarted&&PositionAutoTune::Capture().TryReadLiveTelemetry(observed)&&observed.session==s.session) {
        s.liveTelemetry=observed;s.hasLiveTelemetry=true;
    }
    const auto& live=s.liveTelemetry;
    std::uint64_t liveFlags=s.hasLiveTelemetry?live.flags:0ULL;
    const bool axisValid=s.axis>=0&&s.axis<static_cast<int>(PositionAutoTune::kAxisCount);
    const auto* liveProfile=axisValid?&s.config.axis[s.axis]:nullptr;
    const double ppu=s.capture.pulsesPerMm;
    const bool liveConfig=liveProfile&&s.capture.session==s.session&&s.session&&
        s.capture.axisType<=1U&&std::isfinite(ppu)&&ppu>0.0&&
        liveProfile->cycles>0U&&liveProfile->cycles<=32U&&
        s.capture.sampleStride>0U&&s.capture.sampleStride<=64U&&
        std::isfinite(liveProfile->kpMax)&&liveProfile->kpMax>0.0&&
        std::isfinite(s.capture.motionGains.kp)&&s.capture.motionGains.kp>0.0;
    if(liveConfig) {
        liveFlags|=16ULL;
        const auto& profile=*liveProfile;
        const unsigned totalLegs=4U*profile.cycles;
        const unsigned completed=(std::min)(s.leg,totalLegs);
        const unsigned current=(liveFlags&1ULL)?(std::min)(completed+1U,totalLegs):0U;
        MacroSys.SetSysVar(973,completed);MacroSys.SetSysVar(974,totalLegs);
        MacroSys.SetSysVar(975,completed/4U);MacroSys.SetSysVar(976,profile.cycles);
        MacroSys.SetSysVar(977,current);MacroSys.SetSysVar(978,current?(current+3U)/4U:0U);
        MacroSys.SetSysVar(979,profile.kpMax);MacroSys.SetSysVar(980,profile.followingLimitMm);
        MacroSys.SetSysVar(981,profile.maxRmsErrorMm);MacroSys.SetSysVar(982,profile.maxOvershootMm);
        MacroSys.SetSysVar(983,profile.cruiseLagTargetMm);MacroSys.SetSysVar(984,s.capture.axisType);
        MacroSys.SetSysVar(985,ppu);MacroSys.SetSysVar(986,s.capture.sampleStride);
        MacroSys.SetSysVar(987,(std::min)(profile.kpMax,s.capture.motionGains.kp*
            PositionAutoTune::EffectiveMaxKpIncreaseRatio(s.config,profile)));
        MacroSys.SetSysVar(990,profile.settleWindowMm);MacroSys.SetSysVar(991,profile.targetPositionMm);
        if(s.hasLiveTelemetry) {
            MacroSys.SetSysVar(965,static_cast<double>(live.sampleCount));
            MacroSys.SetSysVar(966,static_cast<double>(live.movingSampleCount));
            MacroSys.SetSysVar(967,static_cast<double>(live.cycle));
            if(live.sampleCount&&live.timestampUs>=live.firstTimestampUs)
                MacroSys.SetSysVar(968,static_cast<double>(live.timestampUs-live.firstTimestampUs)*1e-6);
            if(liveFlags&2ULL) {
                MacroSys.SetSysVar(969,live.currentFollowingErrorPulse/ppu);
                MacroSys.SetSysVar(970,live.peakFollowingErrorPulse/ppu);
            }
            if(liveFlags&4ULL)MacroSys.SetSysVar(971,std::sqrt(live.meanSquaredMovingFollowingErrorPulse)/ppu);
            if(liveFlags&8ULL)MacroSys.SetSysVar(972,live.maximumOvershootPulse/ppu);
            if(liveFlags&128ULL)MacroSys.SetSysVar(988,live.cruisePeakPulse/ppu);
            MacroSys.SetSysVar(989,static_cast<double>(live.observedEndpointCount));
        }
        PositionAutoTune::CaptureFaultSnapshot fault{};
        if(s.failed&&(liveFlags&32ULL)!=0ULL&&s.captureStarted&&
            PositionAutoTune::Capture().TryReadFault(fault)&&fault.session==s.session&&
            !fault.approachActive&&fault.stage==PositionAutoTune::CaptureFaultStage::Stage&&
            fault.hasCycle&&fault.hasSample&&fault.cycle.sourceValid&&fault.observedValid&&fault.sample.valid&&
            (fault.reason==PositionAutoTune::CaptureReason::Following||fault.reason==PositionAutoTune::CaptureReason::Boundary)) {
            const double faultLag=(fault.sample.commandPulse-fault.sample.actualPulse)/ppu;
            if(std::isfinite(faultLag)) {
                liveFlags|=512ULL;MacroSys.SetSysVar(992,faultLag);
                MacroSys.SetSysVar(993,static_cast<unsigned>(fault.reason));
                MacroSys.SetSysVar(994,static_cast<double>(fault.cycle.cycle));
            }
        }
    } else liveFlags=0ULL;
    const unsigned livePhase=!s.session?0U:s.failed||s.phase==TunePhase::Cancel?6U:
        s.done?5U:s.phase==TunePhase::Analysis?4U:s.phase==TunePhase::Admission?1U:
        s.active?((s.capture.pointToPoint&&(liveFlags&1ULL)==0ULL)?2U:3U):0U;
    MacroSys.SetSysVar(963,static_cast<double>(liveFlags));MacroSys.SetSysVar(964,livePhase);
    // Status is published last. HMI reads the existing NC-owner system mirror.
    MacroSys.SetSysVar(901,status);
    if(s.done)TunePrintDiagnosticLine("[TUNE10] event=RESULT session=%llu axis=%d mode=%u status=%u candidate=%u verified=%u baselineSaved=%u rawSaved=%u summarySaved=%u reason=%s motionComparison=%u idleComparison=%u adoptionAdvice=%u gainsChanged=0\n",
        static_cast<unsigned long long>(s.session),s.axis,s.mode,status,
        candidate?1U:0U,!s.failed&&result&&result->verified?1U:0U,
        result&&result->baselineSaved?1U:0U,result&&result->rawSaved?1U:0U,result&&result->summarySaved?1U:0U,
        s.failed?s.failure:result?result->detail.c_str():"NO_RESULT",
        motionCompare?static_cast<unsigned>(result->motionComparison):0U,
        idleCompare?static_cast<unsigned>(result->idleComparison):0U,
        compare&&result->verified?static_cast<unsigned>(result->adoptionAdvice):0U);
}

bool NCManager::CompletePositionAutoTuneSameThread() noexcept {
    if(!m_positionAutoTune)return false;
    auto& s=*m_positionAutoTune;
    if(!s.done||s.failed||!s.result)return false;
    s.callbackConsumed=true;return true;
}
bool NCManager::WaitPositionAutoTuneCallback(NCManager* nc) noexcept {
    return nc&&nc->CompletePositionAutoTuneSameThread();
}
