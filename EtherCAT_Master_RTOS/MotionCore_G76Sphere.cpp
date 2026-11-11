#include "MotionCore.h"
#include "AlarmManager.h"
#include "PLCManager.h"
#include "NCG76StartDiagnostic.h"
#include "NCG76SphereSimulationTestPolicy.h"
#include "EDMCoordinatedAxes384.h"
#include "EDMAxisMotionLimits385.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

extern PLCManager* g_PLC;
namespace {
void PrintSphereMotionMemoryStage(const char* stage,int pc) noexcept
{
    PrintG76SphereMemoryDiagnostic(stage,pc,sizeof(NCTranslationSnapshot),
        sizeof(NCG76SphereCoordinates::FrozenFrame),sizeof(NCG76SphereRuntimeGeometry::Geometry),
        sizeof(NCG76SphereCycle::Request),sizeof(MotionG76SphereFeedback),sizeof(MotionCore));
}
bool SphereIdentity(const MotionExecutionIdentity& a,const MotionExecutionIdentity& b) noexcept
{ return a.IsAssigned() && b.IsAssigned() && a.epoch==b.epoch && a.segmentId==b.segmentId &&
    a.sourceBlockId==b.sourceBlockId && a.source==b.source; }
bool SpherePositive(double v) noexcept {return std::isfinite(v)&&v>0.0;}
bool SpherePulseCeiling385(double rawPulse,double requestedNative,double ppu,double& out) noexcept
{
    out=0.0;if(!SpherePositive(rawPulse)||!SpherePositive(ppu)||!std::isfinite(requestedNative)||requestedNative<0.0)return false;
    if(requestedNative==0.0){out=rawPulse;return true;}
    // Resolve inheritance/clamping in native units first. A positive cap that
    // underflows during unit conversion must never become the zero sentinel.
    const double installedNative=rawPulse/ppu;
    if(!SpherePositive(installedNative))return false;
    const double selected=(std::min)(installedNative,requestedNative);
    const double converted=selected*ppu;
    if(!SpherePositive(converted))return false;
    out=(std::min)(rawPulse,converted);return true;
}

std::uint32_t SphereAxisMask(const NCG76SphereCycle::Request& r) noexcept
{return EDM384Axes::ExistingMask(r.geometry.frame.translation.axisIdentity);}
// The span is a native index bound, not the number of compact group slots.
unsigned SphereAxisCount(const NCG76SphereCycle::Request& r) noexcept
{const auto mask=SphereAxisMask(r);unsigned span=0U;for(unsigned i=0U;i<6U;++i)if(mask&(1U<<i))span=i+1U;return span;}
unsigned SpherePresentAxisCount(const NCG76SphereCycle::Request& r) noexcept
{const auto mask=SphereAxisMask(r);unsigned count=0U;for(unsigned i=0U;i<6U;++i)if(mask&(1U<<i))++count;return count;}
bool SphereAxisPresent(const NCG76SphereCycle::Request& r,unsigned axis) noexcept
{return axis<6U&&(SphereAxisMask(r)&(1U<<axis))!=0U;}
bool SphereRotary(const NCG76SphereCycle::Request& r,unsigned axis) noexcept
{return EDM384Axes::IsRotary(r.geometry.frame.translation.axisIdentity,axis);}
int SphereCommandSlot(const MotionCommand& command,unsigned axis) noexcept
{if(command.axisCount<0||command.axisCount>6)return -1;for(int slot=0;slot<command.axisCount;++slot)
    if(command.axisIndices[slot]==static_cast<int>(axis))return slot;return -1;}
std::uint32_t SphereCommandMask(const MotionCommand& command) noexcept
{if(command.axisCount<0||command.axisCount>6)return 0U;std::uint32_t mask=0U;
 for(int slot=0;slot<command.axisCount;++slot){const int axis=command.axisIndices[slot];
    if(axis<0||axis>=6||(mask&(1U<<axis)))return 0U;mask|=1U<<axis;}return mask;}
bool SphereNativeLineLimits(const NCG76SphereRuntimeGeometry::Geometry& geometry,
    const EDM384Axes::Budget& axes,unsigned retreat,EDM384Axes::Limits& output) noexcept
{
    NCG76SphereCycle::Point8 ratio8{};
    if(!NCG76SphereRuntimeGeometry::LineNativeRatio(geometry,ratio8))return false;
    EDM384Axes::Native6 ratio{};for(unsigned i=0U;i<6U;++i)ratio[i]=ratio8[i];
    if(!EDM384Axes::LimitsForRatio(axes,ratio,output))return false;
    if(retreat==1U){ratio.fill(0.0);ratio[2]=1.0;EDM384Axes::Limits z{};
        if(!EDM384Axes::LimitsForRatio(axes,ratio,z))return false;
        output.velocity=(std::min)(output.velocity,z.velocity);
        output.acceleration=(std::min)(output.acceleration,z.acceleration);
        output.deceleration=(std::min)(output.deceleration,z.deceleration);
        output.activeMask|=z.activeMask;
    }
    return true;
}
bool SphereMovingAuxiliary(const NCG76SphereCycle::Request& r) noexcept
{if(r.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line)return false;
 for(unsigned axis=3U;axis<6U;++axis)if(r.geometry.startMCS[axis]!=r.geometry.endMCS[axis])return true;return false;}
bool SphereManualGeometry(const NCG76SphereCycle::Request& r) noexcept {
 if(!r.physicalZManual)return !r.geometry.frame.physicalZManual;
 if(!r.geometry.frame.physicalZManual||!r.geometry.valid||
    (SphereAxisMask(r)&7U)!=7U||r.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line||
    r.retreatMode>4U||r.orbitInspectionRounds||r.geometry.travelMm<=0.0)return false;
 NCG76SphereCycle::Point8 ratio{};
 return NCG76SphereRuntimeGeometry::LineNativeRatio(r.geometry,ratio);
}
bool SpherePowerOff() noexcept
{ return g_PLC!=nullptr && g_PLC->ReadEDMOutputWord()==0U; }
constexpr std::uint32_t SphereRuntimeAlarm=AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY;
constexpr std::uint32_t SphereTimeLimitRuntimeFault=10U;
constexpr std::uint32_t SphereTimeLimitAlarm=AlarmManager::EDM_MACHINING_TIME_LIMIT;
bool SphereOrbitCompletionConfigValid(std::uint32_t rounds,std::uint32_t threshold,unsigned retryLimit=0U) noexcept
{
    return NCG76OrbitCompletion::ValidConfig(rounds,threshold,retryLimit);
}
bool SphereOrbitCompletionFeedbackMatches(const NCG76SphereCycle::Feedback& feedback,
    std::uint32_t rounds,std::uint32_t threshold,bool terminal=false,unsigned retryLimit=0U) noexcept
{
    if(!SphereOrbitCompletionConfigValid(rounds,threshold,retryLimit)||feedback.orbitRetryLimit!=retryLimit||
       feedback.orbitRequiredPasses!=rounds||feedback.orbitTargetPercentScaled!=threshold)return false;
    if(!rounds)return feedback.orbitCompletionState==0U&&!feedback.orbitCompletionActive&&
        !feedback.orbitPercentAvailable&&feedback.orbitPercentScaled==0U&&
        feedback.orbitQualifiedPasses==0U&&feedback.orbitAttemptCount==0ULL;
    if(feedback.orbitCompletionState==7U)
        return MotionG76SphereMachiningTime::RetryExhaustionStateQualified(feedback,rounds,threshold,retryLimit);
    if(!MotionG76SphereMachiningTime::RetryBudgetEvidenceQualified(feedback,rounds,threshold,retryLimit))return false;
    if(feedback.orbitCompletionState==6U)return feedback.timeLimitReached&&
        feedback.totalLimitUs&&feedback.totalElapsedUs==feedback.totalLimitUs&&
        feedback.totalTimerState==static_cast<std::uint8_t>(NCMachiningTime::TimerState::Completed)&&
        feedback.orbitQualifiedPasses<=rounds&&feedback.orbitAttemptCount>=feedback.orbitQualifiedPasses&&
        !feedback.orbitCompletionActive&&!feedback.orbitPercentAvailable&&!feedback.orbitPercentScaled&&
        (!terminal||feedback.timedCompletion);
    if(feedback.orbitCompletionState<1U||feedback.orbitCompletionState>5U||
       feedback.orbitPercentScaled>100000000U||
       (!feedback.orbitPercentAvailable&&feedback.orbitPercentScaled!=0U)||
       feedback.orbitQualifiedPasses>rounds||feedback.orbitAttemptCount<feedback.orbitQualifiedPasses)return false;
    if(feedback.orbitCompletionActive!=(feedback.orbitCompletionState==2U||feedback.orbitCompletionState==3U))return false;
    if(feedback.orbitCompletionState==1U&&(feedback.orbitPercentAvailable||feedback.orbitQualifiedPasses||feedback.orbitAttemptCount))return false;
    if(feedback.orbitCompletionState==5U&&(feedback.orbitPercentAvailable||feedback.orbitPercentScaled))return false;
    if(feedback.orbitCompletionState!=4U&&feedback.orbitQualifiedPasses==rounds)return false;
    if(feedback.orbitCompletionState==4U){
        if(feedback.orbitCompletionActive||!feedback.orbitPercentAvailable||
           feedback.orbitQualifiedPasses!=rounds||feedback.orbitPercentScaled<threshold)return false;
    }else if(terminal)return false;
    return true;
}
// NC readers derive a local criteria view only from a scoped RT receipt.
// No RT-owned mutable Request is read across threads, and geometry stays born.
bool SphereRecipeCompletionReceipt(const MotionG76SphereFeedback& report,
    std::uint32_t rounds,std::uint32_t birthThreshold,std::uint8_t birthRetry,
    std::uint32_t& threshold,std::uint8_t& retry) noexcept
{
    threshold=birthThreshold;retry=birthRetry;
    if(report.liveOrbitRecipeSequence){
        if(!report.liveRecipeSequence||report.liveOrbitRecipeSequence>report.liveRecipeSequence||
           report.cycle.orbitRequiredPasses!=rounds||
           !NCG76OrbitCompletion::ValidConfig(rounds,report.cycle.orbitTargetPercentScaled,
               report.cycle.orbitRetryLimit))return false;
        threshold=report.cycle.orbitTargetPercentScaled;retry=report.cycle.orbitRetryLimit;
    }
    return NCG76OrbitCompletion::ValidConfig(rounds,threshold,retry);
}
void SphereWithdrawOrbitCompletion(NCG76SphereCycle::Feedback& feedback) noexcept
{
    if(feedback.orbitRequiredPasses){
        feedback.orbitCompletionState=5U;feedback.orbitCompletionActive=false;
        feedback.orbitPercentAvailable=false;feedback.orbitPercentScaled=0U;feedback.orbitQualifiedPasses=0U;
    }
}
}

bool MotionCore::SelectManualGapZGains(AxisContext& axis) noexcept
{
    if(axis.axisIndex<0||axis.axisIndex>=6||!axis.isExist||axis.isVirtualAxis||!axis.isServoOn||
       axis.isFault||axis.isLagAlarm||axis.homeRuntime.active||axis.state!=MotionState::MotionState_INTERPOLATING||
       !m_g76SphereRunning||!SphereAxisPresent(m_g76SphereFrozen.request,static_cast<unsigned>(axis.axisIndex))||
       !m_g76SphereFrozen.request.physicalZManual||!m_g76SphereGainBanks.Configured()||
       m_g76SphereGainBanks.ConfiguredMask()!=SphereAxisMask(m_g76SphereFrozen.request)||
       !m_Group.isActive||!m_Group.currentCmd.g76SphereAir||
       !SphereIdentity(m_Group.currentCmd.execution,m_g76SphereFrozen.command.execution)||
       !m_Group.currentCmd.ownerLease.Matches(m_g76SphereFrozen.command.ownerLease)||
       m_g76SphereClaim.load(std::memory_order_acquire)!=m_g76SphereFrozen.request.session||
       m_g76SphereParentBound.load(std::memory_order_acquire)!=m_g76SphereFrozen.request.session||
       GetCurrentExecutionEpoch()!=m_g76SphereFrozen.command.execution.epoch||
       !IsMotionOwnerLeaseCurrent(m_g76SphereFrozen.command.ownerLease)||
       HasPendingSafetyOrRecoveryRequests()||HasPendingExecutionEpochChange()||m_safetyControlledStopInProgress||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==m_g76SphereFrozen.request.session||
       !m_g76SphereInput.sourceValid||!m_g76SphereInput.pdoValid||
       m_g76SphereInput.tick!=m_ncSettleRuntimeCycleTick||!m_ncSettleRuntimeCycleValid||!m_ncSettleRuntimeCycleContiguous||
       !MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
           m_g76SphereFrozen.request.session,m_g76SphereFrozen.command.execution)||
       !m_g76SpherePhysicalSource.CurrentAt(m_edmZMonotonicUs,m_edmZClockFrequency))return false;
    const auto bank=EDM382Gains::Resolve(m_g76SphereCycle.Current());
    const auto* gain=m_g76SphereGainBanks.Select(bank,static_cast<unsigned>(axis.axisIndex));
    if(!gain)return false;
    axis.pid.Kp=gain->Kp;axis.pid.Ki=gain->Ki;axis.pid.Kd=gain->Kd;axis.pid.Kvff=gain->Kvff;
    m_g76SphereGainBank[static_cast<unsigned>(axis.axisIndex)]=bank;return true;
}

bool MotionCore::ReadG76SphereFeedback(MotionG76SphereFeedback& out) const noexcept
{
    if(!m_g76SphereFeedback.Read(out).Ready())return false;
    out.endpointCommitted=out.completed&&m_g76SphereEndpointCommitted.load(std::memory_order_acquire)==out.session;
    if(m_g76SphereInspectionProducerBlocked.load(std::memory_order_acquire)){
        out.holdInspectionJogReady=false;out.holdRecoveryResumeReady=false;
    }
    if(m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==out.session){
        out.processShadow=NCG76SphereGapProcess::Feedback{};
        SphereWithdrawOrbitCompletion(out.cycle);
    }
    return true;
}

bool MotionCore::RevokeG76SphereProcessShadow(std::uint64_t session,
    const MotionExecutionIdentity& identity,const MotionOwnerLease& owner) noexcept
{
    // Withdrawal is data-only and is allowed after owner/epoch revocation too.
    // A stale parent can never withdraw a newer parent's observer.
    G76SphereFrozen frozen{};
    if(!session||m_g76SphereClaim.load(std::memory_order_acquire)!=session||
       !m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
       !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner))return false;
    auto previous=m_g76SphereProcessRevokedSession.load(std::memory_order_acquire);
    while(previous<session&&!m_g76SphereProcessRevokedSession.compare_exchange_weak(previous,session,
        std::memory_order_acq_rel,std::memory_order_acquire)){}
    return true;
}

bool MotionCore::UpdateG76SphereWorkTime(std::uint64_t session,
    const MotionExecutionIdentity& identity,const MotionOwnerLease& owner,
    std::uint64_t updateSequence,std::uint64_t baseRecipeGeneration,
    std::uint64_t originalWorkTimeUs,std::uint64_t newRecipeGeneration,std::uint64_t workTimeUs) noexcept
{
    G76SphereFrozen frozen{};G76SphereProcessFrozen process{};
    if(!session||!updateSequence||!baseRecipeGeneration||newRecipeGeneration<=baseRecipeGeneration||
       !EDM130::ValidWorkTime(originalWorkTimeUs)||!EDM130::ValidWorkTime(workTimeUs)||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==session||
       !m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
       !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
       !frozen.request.physicalZManual||
       (frozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        frozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
       GetCommandAuthorizationFailure(frozen.command)!=MotionRejectReason::NONE||
       !m_g76SphereProcessRequests.Read(process).Ready()||process.session!=session||
       !SphereIdentity(process.identity,identity)||!process.owner.Matches(owner)||
       !process.config.enabled||!process.config.physicalZManual||!NCG76SphereGapProcess::Valid(process.config)||
       process.config.recipeGeneration!=baseRecipeGeneration||process.config.workTimeUs!=originalWorkTimeUs||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session)return false;
    G76SphereWorkTimeUpdate update{};update.session=session;update.identity=identity;update.owner=owner;
    update.sequence=updateSequence;update.baseRecipeGeneration=baseRecipeGeneration;
    update.originalWorkTimeUs=originalWorkTimeUs;update.newRecipeGeneration=newRecipeGeneration;update.workTimeUs=workTimeUs;
    return m_g76SphereWorkTimeUpdates.Publish(update);
}

bool MotionCore::ApplyG76SphereWorkTimeUpdate() noexcept
{
    // Latest-value channel: exact scope and monotonic sequence prevent replay.
    // This function runs only after the ordinary source/owner/frozen-page guards.
    if(!m_g76SphereWorkTimeUpdates.Read(m_g76SphereIncomingWorkTimeUpdate).Ready())return true;
    const auto& update=m_g76SphereIncomingWorkTimeUpdate;
    const auto& process=m_g76SphereProcessFrozen;const auto& current=m_g76SphereProcess.Current();
    if(update.session!=m_g76SphereFrozen.request.session||update.sequence<=m_g76SphereWorkTimeSequence||
       !SphereIdentity(update.identity,m_g76SphereFrozen.command.execution)||
       !update.owner.Matches(m_g76SphereFrozen.command.ownerLease)||
       update.baseRecipeGeneration!=process.config.recipeGeneration||
       update.originalWorkTimeUs!=process.config.workTimeUs||
       update.newRecipeGeneration<=m_g76SphereWorkTimeRecipeGeneration||
       !EDM130::ValidWorkTime(update.workTimeUs)||!m_g76SphereFrozen.request.physicalZManual||
       (m_g76SphereFrozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        m_g76SphereFrozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
       process.session!=update.session||!process.config.enabled||!process.config.physicalZManual||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==update.session||
       current.session!=update.session||!current.enabled||current.terminal||
       current.reason!=NCG76SphereGapProcess::Reason::None)return true;
    if(!m_g76SphereProcess.UpdateWorkTime(update.workTimeUs))return false;
    m_g76SphereWorkTimeSequence=update.sequence;
    m_g76SphereWorkTimeRecipeGeneration=update.newRecipeGeneration;
    m_g76SphereRequestedWorkTimeUs=update.workTimeUs;return true;
}

bool MotionCore::UpdateG76SphereProcessTuning(std::uint64_t session,
    const MotionExecutionIdentity& identity,const MotionOwnerLease& owner,
    std::uint64_t baseTuningGeneration,std::uint64_t tuningGeneration,
    const NCG76SphereGapProcess::Config& config) noexcept
{
    G76SphereFrozen frozen{};G76SphereProcessFrozen process{};
    if(!session||!baseTuningGeneration||tuningGeneration<=baseTuningGeneration||
       config.tuningGeneration!=tuningGeneration||m_g76SphereClaim.load(std::memory_order_acquire)!=session||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==session||
       !m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
       !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
       !frozen.request.physicalZManual||
       (frozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        frozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
       GetCommandAuthorizationFailure(frozen.command)!=MotionRejectReason::NONE||
       !m_g76SphereProcessRequests.Read(process).Ready()||process.session!=session||
       !SphereIdentity(process.identity,identity)||!process.owner.Matches(owner)||
       process.config.tuningGeneration!=baseTuningGeneration||
       !NCG76SphereGapProcess::EquivalentForLiveTuning(process.config,config)||
       !NCG76SphereGapProcess::Valid(config))return false;
    NCG76SphereCycle::FlushConfig flush{};double ceiling=0.0;
    if((config.parameterFeeds&&(!NCG76SphereGapProcess::ResolveParameterMotionCeiling(config,ceiling)||
            ceiling>frozen.request.feedMmMin))||
       !NCG76SphereGapProcess::BuildFlushConfig(config,frozen.request.feedMmMin,flush)||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session)return false;
    G76SphereProcessTuningUpdate update{};update.session=session;update.identity=identity;update.owner=owner;
    update.baseTuningGeneration=baseTuningGeneration;update.tuningGeneration=tuningGeneration;update.config=config;
    return m_g76SphereProcessTuningUpdates.Publish(update);
}

bool MotionCore::ApplyG76SphereProcessTuningUpdate() noexcept
{
    // The source/clock, current owner and immutable birth pages were proved by
    // the ordinary callback gates. A tuning page carries no motion authority.
    const auto publication=m_g76SphereProcessTuningUpdates.Current();
    if(!publication.Ready()||publication.publication<=m_g76SphereLiveTuningPublication)return true;
    const auto receipt=m_g76SphereProcessTuningUpdates.Read(m_g76SphereIncomingProcessTuning);
    if(!receipt.Ready())return true;
    m_g76SphereLiveTuningPublication=receipt.publication;
    const auto& update=m_g76SphereIncomingProcessTuning;const auto& birth=m_g76SphereProcessFrozen;
    const auto& current=m_g76SphereProcess.Current();
    if(update.session!=m_g76SphereFrozen.request.session||
       update.tuningGeneration<=m_g76SphereLiveTuningGeneration||
       update.baseTuningGeneration!=birth.config.tuningGeneration||
       update.config.tuningGeneration!=update.tuningGeneration||
       !SphereIdentity(update.identity,m_g76SphereFrozen.command.execution)||
       !update.owner.Matches(m_g76SphereFrozen.command.ownerLease)||
       !m_g76SphereFrozen.request.physicalZManual||
       (m_g76SphereFrozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        m_g76SphereFrozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
       birth.session!=update.session||!birth.config.physicalZManual||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==update.session||
       current.session!=update.session||!current.enabled||current.terminal||
       current.reason!=NCG76SphereGapProcess::Reason::None)return true;
    double ceiling=0.0;
    if(!NCG76SphereGapProcess::EquivalentForLiveTuning(birth.config,update.config)||
       !NCG76SphereGapProcess::Valid(update.config)||
       (update.config.parameterFeeds&&
        (!NCG76SphereGapProcess::ResolveParameterMotionCeiling(update.config,ceiling)||
         ceiling>m_g76SphereFrozen.request.feedMmMin))||
       !NCG76SphereGapProcess::BuildFlushConfig(update.config,m_g76SphereFrozen.request.feedMmMin,
           m_g76SphereLiveFlushConfig))return false;
    // Retain the short detector's configuration revision and accumulated dwell.
    // The complete, new revision survives only in the continuation sidecar.
    auto curve=update.config.curve;curve.revision=birth.config.curve.revision;
    if(!m_g76SphereProcess.UpdateLiveCurve(curve)||
       !m_g76SphereCycle.UpdateFlushFeeds(m_g76SphereLiveFlushConfig))return false;
    m_g76SphereLiveTuningConfig=update.config;
    m_g76SphereLiveTuningGeneration=update.tuningGeneration;
    m_g76SphereCurrent.liveCurveTuningGeneration=update.tuningGeneration;
    if(!m_g76SphereCycle.HasPendingFlushFeeds())
        m_g76SphereCurrent.liveFlushTuningGeneration=update.tuningGeneration;
    return true;
}

bool MotionCore::UpdateG76SphereLiveRecipe(std::uint64_t session,
    const MotionExecutionIdentity& identity,const MotionOwnerLease& owner,std::uint64_t sequence,
    std::uint64_t baseRecipeGeneration,std::uint64_t baseTuningGeneration,
    const NCG76SphereGapProcess::Config& config,std::uint32_t orbitThresholdScaled,
    std::uint8_t orbitRetryLimit) noexcept
{
    G76SphereFrozen frozen{};G76SphereProcessFrozen birth{};G76SphereHoldFrozen hold{};
    if(!session||!sequence||!baseRecipeGeneration||!baseTuningGeneration||
       config.recipeGeneration<baseRecipeGeneration||config.tuningGeneration<baseTuningGeneration||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==session||
       !m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
       !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
       GetCommandAuthorizationFailure(frozen.command)!=MotionRejectReason::NONE||
       !m_g76SphereProcessRequests.Read(birth).Ready()||birth.session!=session||
       !SphereIdentity(birth.identity,identity)||!birth.owner.Matches(owner)||
       birth.config.recipeGeneration!=baseRecipeGeneration||birth.config.tuningGeneration!=baseTuningGeneration||
       !NCG76SphereGapProcess::EquivalentForLiveRecipe(birth.config,config)||
       !m_g76SphereHoldRequests.Read(hold).Ready()||hold.session!=session||
       !SphereIdentity(hold.identity,identity)||!hold.owner.Matches(owner)||
       !NCG76OrbitCompletion::ValidConfig(frozen.request.orbitInspectionRounds,orbitThresholdScaled,orbitRetryLimit))return false;
    NCG76SphereCycle::FlushConfig flush{};double ceiling=0.0;
    if((config.parameterFeeds&&(!NCG76SphereGapProcess::ResolveParameterMotionCeiling(config,ceiling)||
            ceiling>frozen.request.feedMmMin))||
       !NCG76SphereGapProcess::BuildFlushConfig(config,frozen.request.feedMmMin,flush)||
       !NCG76SphereCycle::ValidateLiveFlushConfig(frozen.request,flush,hold.finishRequiredCount)||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session)return false;
    G76SphereLiveRecipeUpdate update{};update.session=session;update.identity=identity;update.owner=owner;
    update.sequence=sequence;update.baseRecipeGeneration=baseRecipeGeneration;
    update.baseTuningGeneration=baseTuningGeneration;update.config=config;
    update.orbitThresholdScaled=orbitThresholdScaled;update.orbitRetryLimit=orbitRetryLimit;
    return m_g76SphereLiveRecipeUpdates.Publish(update);
}

bool MotionCore::ApplyG76SphereLiveRecipeUpdate() noexcept
{
    const auto publication=m_g76SphereLiveRecipeUpdates.Current();
    if(!publication.Ready()||publication.publication<=m_g76SphereLiveRecipePublication)return true;
    const auto receipt=m_g76SphereLiveRecipeUpdates.Read(m_g76SphereIncomingLiveRecipe);
    if(!receipt.Ready())return true;
    m_g76SphereLiveRecipePublication=receipt.publication;
    const auto& update=m_g76SphereIncomingLiveRecipe;const auto& birth=m_g76SphereProcessFrozen;
    const auto& current=m_g76SphereProcess.Current();
    if(update.session!=m_g76SphereFrozen.request.session||update.sequence<=m_g76SphereLiveRecipeSequence||
       !SphereIdentity(update.identity,m_g76SphereFrozen.command.execution)||
       !update.owner.Matches(m_g76SphereFrozen.command.ownerLease)||
       update.baseRecipeGeneration!=birth.config.recipeGeneration||
       update.baseTuningGeneration!=birth.config.tuningGeneration||
       update.config.recipeGeneration<m_g76SphereLiveRecipeConfig.recipeGeneration||
       update.config.tuningGeneration<m_g76SphereLiveRecipeConfig.tuningGeneration||
       birth.session!=update.session||m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==update.session||
       current.session!=update.session||!current.enabled||current.terminal||
       current.reason!=NCG76SphereGapProcess::Reason::None)return true;
    double ceiling=0.0;
    if(!NCG76SphereGapProcess::EquivalentForLiveRecipe(birth.config,update.config)||
       !NCG76OrbitCompletion::ValidConfig(m_g76SphereFrozen.request.orbitInspectionRounds,
            update.orbitThresholdScaled,update.orbitRetryLimit)||
       (update.config.parameterFeeds&&
        (!NCG76SphereGapProcess::ResolveParameterMotionCeiling(update.config,ceiling)||
         ceiling>m_g76SphereFrozen.request.feedMmMin))||
       !NCG76SphereGapProcess::BuildFlushConfig(update.config,m_g76SphereFrozen.request.feedMmMin,
            m_g76SphereLiveRecipeFlush)||
       !NCG76SphereCycle::ValidateLiveFlushValues(m_g76SphereFrozen.request,m_g76SphereLiveRecipeFlush,
            m_g76SphereHoldFrozen.finishRequiredCount))return false;
    // All setters validate before mutation; owner/source and the immutable
    // birth pages remain in force. Whole flush and criteria updates may defer.
    // A proposed E5 request can precede Cycle.flushActive by one callback.
    // Retain its old flush image too; changing it here could orphan that ACK.
    if(!current.pending&&!m_g76SphereCycle.UpdateLiveFlushConfig(m_g76SphereLiveRecipeFlush))return false;
    if(!m_g76SphereCycle.UpdateLiveOrbitCompletion(update.orbitThresholdScaled,update.orbitRetryLimit)||
       !m_g76SphereProcess.UpdateLiveRecipe(update.config))return false;
    m_g76SphereLiveRecipeFlushDeferred=current.pending;
    m_g76SphereLiveRecipeConfig=update.config;m_g76SphereLiveRecipeSequence=update.sequence;
    m_g76SphereLiveRecipeThreshold=update.orbitThresholdScaled;m_g76SphereLiveRecipeRetry=update.orbitRetryLimit;
    m_g76SphereCurrent.liveRecipeSequence=update.sequence;
    m_g76SphereCurrent.liveCurveTuningGeneration=update.config.tuningGeneration;
    RefreshG76SphereLiveRecipeReceipts();return true;
}

bool MotionCore::ApplyDeferredG76SphereRecipeFlush() noexcept
{
    if(!m_g76SphereLiveRecipeFlushDeferred||m_g76SphereProcess.Current().pending)return true;
    const auto& process=m_g76SphereProcess.Current();const auto& command=m_g76SphereCycle.Command();
    if(!process.enabled||process.terminal||!command.active||command.faulted||command.complete)return true;
    if(!m_g76SphereCycle.UpdateLiveFlushConfig(m_g76SphereLiveRecipeFlush))return false;
    m_g76SphereLiveRecipeFlushDeferred=false;return true;
}

void MotionCore::RefreshG76SphereLiveRecipeReceipts() noexcept
{
    if(!m_g76SphereLiveRecipeSequence)return;
    if(!m_g76SphereLiveRecipeFlushDeferred&&!m_g76SphereCycle.HasPendingLiveFlushConfig()&&
       !m_g76SphereProcess.HasPendingLiveRecipeFlush()){
        m_g76SphereCurrent.liveFlushRecipeSequence=m_g76SphereLiveRecipeSequence;
        m_g76SphereCurrent.liveFlushTuningGeneration=m_g76SphereLiveRecipeConfig.tuningGeneration;
    }
    if(!m_g76SphereCycle.HasPendingLiveOrbitCompletion()&&
       m_g76SphereCycle.LiveOrbitThresholdScaled()==m_g76SphereLiveRecipeThreshold&&
       m_g76SphereCycle.LiveOrbitRetryLimit()==m_g76SphereLiveRecipeRetry){
        m_g76SphereLiveCompletionRequest.orbitThresholdScaled=m_g76SphereLiveRecipeThreshold;
        m_g76SphereLiveCompletionRequest.orbitRetryLimit=m_g76SphereLiveRecipeRetry;
        m_g76SphereCurrent.liveOrbitRecipeSequence=m_g76SphereLiveRecipeSequence;
    }
}

bool MotionCore::TryG76SphereMove(const NCG76SphereCycle::Request& requested,
    const NCTranslationSnapshot& frame,MotionExecutionEpoch epoch,const MotionOwnerLease& owner,
    int sourcePC,MotionExecutionIdentity& identity,MotionG76SphereAdmission* admission,
    const NCG76SphereGapProcess::Config* processShadow,const NCHoldRecovery::Config* holdConfig,
    std::uint32_t declaredPreviousParents,std::uint32_t finishRequiredCount) noexcept
{
    identity=MotionExecutionIdentity{};
    if(admission)*admission=MotionG76SphereAdmission{};
    PrintSphereMotionMemoryStage("NC_MOTION_ADMISSION_BEGIN",sourcePC);
    const auto reject=[&](MotionG76SphereAdmissionGate gate,int axis=-1) noexcept {
        if(admission){admission->gate=gate;admission->axis=axis;}return false;
    };
    if(!requested.session||requested.session<=m_g76SphereProducerHighWater||sourcePC<0||
        !NCG76SphereCycle::ValidateRequest(requested)||!SphereManualGeometry(requested)||!SpherePowerOff())return reject(MotionG76SphereAdmissionGate::Request);
    if(!SameNCTranslationSnapshot(requested.geometry.frame.translation,frame)||!MatchesNCTranslation(frame)||
        !IsNCTranslationSnapshotValid(frame)||!IsNCTranslationAxisIdentityCurrent(frame))return reject(MotionG76SphereAdmissionGate::Frame);
    if(!m_CompEngine.IsSealed())return reject(MotionG76SphereAdmissionGate::Compensation);
    if(owner.owner!=MotionOwner::AUTO||
        !HasExactExecutionDrainAcknowledgement(epoch,owner)||m_pContexts==nullptr||m_pCoordMgr==nullptr||
        m_pContexts->size()<SphereAxisCount(requested)||IsRotarySpindleSessionClaimed()||GetQueueSize()!=0U||GetCommandIngressSize()!=0U||GetCommandReplaySize()!=0U)
        return reject(MotionG76SphereAdmissionGate::OwnerDrain);
    if(finishRequiredCount>99U||(holdConfig&&(!NCHoldRecovery::Valid(*holdConfig)||
            declaredPreviousParents>NCHoldPolicy::MaximumPreviousParents))||
       (!holdConfig&&declaredPreviousParents!=0U))return reject(MotionG76SphereAdmissionGate::Request);
    G76SphereHoldFrozen hold{};hold.session=requested.session;hold.enabled=holdConfig!=nullptr;
    hold.declaredPreviousParents=declaredPreviousParents;
    hold.finishRequiredCount=finishRequiredCount;
    hold.machiningTime=MotionG76SphereMachiningTime::Config(requested);
    hold.orbitInspectionRounds=requested.orbitInspectionRounds;
    hold.orbitThresholdScaled=requested.orbitThresholdScaled;
    hold.orbitRetryLimit=requested.orbitRetryLimit;
    if(holdConfig)hold.config=*holdConfig;
    G76SphereFrozen frozen{};frozen.request=requested;
    frozen.holdRecoveryEnabled=hold.enabled;
    frozen.holdDeclaredPreviousParents=static_cast<std::uint8_t>(declaredPreviousParents);
    frozen.finishRequiredCount=static_cast<std::uint8_t>(finishRequiredCount);
    auto& request=frozen.request;auto& command=frozen.command;
    // Capture the optional SIM-only process snapshot before checking its whole
    // flush travel envelope. An explicitly enabled, unsafe excursion rejects
    // this parent before publication rather than becoming an AIR fallback.
    G76SphereProcessFrozen process{};process.session=request.session;
    if(processShadow&&NCG76SphereGapProcess::Valid(*processShadow))process.config=*processShadow;
    else if(processShadow&&(processShadow->parameterFeeds||processShadow->flushMotionEnabled||processShadow->signedMotionEnabled||
        processShadow->independentFeedsEnabled||processShadow->finalApproachEnabled||
        processShadow->initialSlowEnabled||processShadow->initialSlowPercent!=0.0||
        processShadow->initialSlowDistanceMm!=0.0||processShadow->deepFlushEnabled||
        processShadow->deepFlushCycleInterval!=0U||processShadow->deepFlushHeightMultiplier!=0.0||
        processShadow->deepFlushHeightMm!=0.0||processShadow->orbitFeedsEnabled||
        processShadow->orbitScaleMmPerVoltMin!=0.0||processShadow->orbitOverridePercent!=0.0||processShadow->flushAxisLimitsEnabled||
        !EDM385::Empty(processShadow->flushProfile.axisModeLimits)||
        processShadow->e5WorkTimer.mode!=0U||!EDM386E5::Valid(processShadow->e5WorkTimer)))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(!MotionG76SphereParameterFeeds::Matches(request,process.config))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(request.physicalZManual&&(!process.config.physicalZManual||!SphereManualGeometry(request)||
       !process.config.flushAxisLimitsEnabled||process.config.flushAxisMask!=SphereAxisMask(request)||
       !m_g76SphereGainBanks.Configured()||m_g76SphereGainBanks.ConfiguredMask()!=SphereAxisMask(request)))
        return reject(MotionG76SphereAdmissionGate::Request);
    // Every mixed-unit line needs per-leg native limits, including a SIM
    // source whose physical motors may later traverse retained HOLD history.
    if(SphereMovingAuxiliary(request)&&(!process.config.flushAxisLimitsEnabled||
       process.config.flushAxisMask!=SphereAxisMask(request)))return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(!EDM385::Empty(process.config.flushProfile.axisModeLimits)&&
       (!process.config.enabled||!process.config.flushAxisLimitsEnabled||process.config.flushAxisMask!=SphereAxisMask(request)))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    frozen.signedMotionEnabled=process.config.signedMotionEnabled;
    frozen.orbitFeedsEnabled=process.config.orbitFeedsEnabled;
    frozen.flushFeatureFlags=static_cast<std::uint8_t>((process.config.independentFeedsEnabled?1U:0U)|
        (process.config.finalApproachEnabled?2U:0U)|(process.config.initialSlowEnabled?4U:0U)|
        (process.config.deepFlushEnabled?8U:0U));
    const bool commonProcess=request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere;
    if(request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
       (!process.config.enabled||!process.config.signedMotionEnabled||!process.config.orbitFeedsEnabled||
        request.parameterFeeds||(request.retreatMode!=0U&&request.retreatMode!=2U)))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(NCMachiningTime::Enabled(hold.machiningTime)&&
       (!process.config.enabled||!process.config.signedMotionEnabled||
        (!commonProcess&&!process.config.orbitFeedsEnabled)||
        (request.additionalUs&&request.geometry.travelMm<=0.0)))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    // The explicit common G01/G02/G03 lane is an E-aware SIM parent even in
    // B0. It may not silently become ordinary AIR when its sidecar is absent.
    if(commonProcess&&(!process.config.enabled||!frozen.signedMotionEnabled))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(request.orbitInspectionRounds&&(!process.config.enabled||!frozen.signedMotionEnabled||
       !frozen.orbitFeedsEnabled||finishRequiredCount!=0U))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    // EDM301: an explicit process mode must never silently become a B0 AIR
    // parent when its recipe or independent SIM sidecar is unavailable.
    // GAP history remains available when E6 disables periodic flushing.
    if(request.retreatMode!=0U&&(!process.config.enabled||!frozen.signedMotionEnabled))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(request.retreatMode!=0U){
        // NC-side admission only: establish distinct canonical stations before
        // any B2 traversal can begin. The RT cycle uses a bounded borrowed view
        // and never rebuilds/scans the parent's layers on an interpolation tick.
        NCG76SpherePath::Descriptor path{};
        if(NCG76SpherePath::Build(request.geometry,path)!=NCG76SpherePath::Status::Ok)
            return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    }
    NCG76SphereCycle::FlushConfig flush{};
    if(!NCG76SphereGapProcess::BuildFlushConfig(process.config,request.feedMmMin,flush))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    // A positive L belongs to this real analytic machining parent. It cannot
    // turn an AIR, zero-length or spherical request into a verified finish.
    if(finishRequiredCount!=0U&&(!commonProcess||
       request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour||request.geometry.travelMm<=0.0||
       !process.config.enabled||!frozen.signedMotionEnabled||!process.config.flushMotionEnabled||!flush.enabled))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    command.g76SphereAir=true;command.mode=InterpolationMode::LINEAR;
    command.commandPathMode=MotionCommandPathMode::EXACT_STOP;
    command.sourceLinePC=sourcePC;command.sourceTranslation=frame;
    command.sourceWCS=frame.wcsCode;command.sourceToolLengthMode=frame.toolLengthMode;
    command.sourceHCode=frame.toolHCode;command.sourceToolRadiusMode=frame.cutterMode;command.sourceDCode=frame.cutterD;
    command.sourceIsAbsoluteMode=frame.distanceMode==90;command.sourcePlaneMode=frame.rotationPlane;
    command.sourceG68Active=frame.rotationMode==68;command.sourceG68Angle=frame.rotationAngleDeg;
    command.sourceG168Active=frame.workMode==168;command.sourceWCode=frame.workWCode;
    command.sourceG51Active=frame.scalingMode==51;command.sourceScaleRatio=frame.scalingFactor;
    command.sourceMirrorMask=static_cast<std::uint8_t>(frame.mirrorMask);
    command.sourceG16Active=frame.polarMode==16;command.sourceG162Active=frame.axisIdentity.eccentricEnabled!=0U;
    request.travelMask=0U;
    request.settleSamples=(std::max)(200U,requested.settleSamples);
    request.minimumSettleSeconds=(std::max)(0.05,requested.minimumSettleSeconds);
    double rampSeconds=0.0,stopSeconds=0.0;
    const bool independentEdm=process.config.enabled&&process.config.signedMotionEnabled;
    if(independentEdm&&CYCLE_TIME_SEC!=EDM385::IndependentCycleSeconds)
        return reject(MotionG76SphereAdmissionGate::AxisConfiguration);
    EDM384Axes::Budget installedAxisBudget{};installedAxisBudget.mask=SphereAxisMask(request);
    if(independentEdm&&(!process.config.flushAxisLimitsEnabled||process.config.flushAxisMask!=installedAxisBudget.mask))
        return reject(MotionG76SphereAdmissionGate::AxisConfiguration);
    if(process.config.flushAxisLimitsEnabled&&process.config.flushAxisMask!=installedAxisBudget.mask)
        return reject(MotionG76SphereAdmissionGate::AxisConfiguration);
    for(unsigned i=0U;i<SphereAxisCount(request);++i){
        if(!SphereAxisPresent(request,i))continue;
        const auto& axis=(*m_pContexts)[i];const double ppu=axis.resolution_PPR/axis.finalLead;
        if(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||!SpherePositive(ppu)||
            !SpherePositive(axis.maxVel_PPS)||axis.maxVel_PPS>static_cast<double>((std::numeric_limits<std::int32_t>::max)())/1.2||
            !SpherePositive(axis.inPositionWindow_Pulse)||!SpherePositive(axis.resolution_PPR)||!SpherePositive(axis.finalLead)||
            (axis.axisType==AxisType::ROTARY&&!SpherePositive(axis.rotaryModulo))||
            static_cast<unsigned>(axis.axisType)!=frame.axisIdentity.axisType[i]||
            (i<3U&&axis.axisType!=AxisType::LINEAR)||
            (!independentEdm&&(!std::isfinite(axis.G00_acc_time)||axis.G00_acc_time<0.0||
            !std::isfinite(axis.G00_dec_time)||axis.G00_dec_time<0.0||
            !std::isfinite(axis.Stop_dec_time)||axis.Stop_dec_time<0.0)))return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
        auto& binding=frozen.axis[i];
        binding.resolution=axis.resolution_PPR;binding.lead=axis.finalLead;binding.maxVelocity=axis.maxVel_PPS;
        // These legacy times have no normal-EDM control or lineage meaning.
        // Canonical zeros keep unrelated G00 tuning out of retained history.
        binding.accSeconds=independentEdm?0.0:axis.G00_acc_time;
        binding.decSeconds=independentEdm?0.0:axis.G00_dec_time;
        binding.stopSeconds=independentEdm?0.0:axis.Stop_dec_time;
        binding.machineOffset=axis.machineCoordinateOffsetPulse;binding.scaleRatio=axis.scaleToMotorRatio;
        binding.maxDeviation=axis.maxDeviation;binding.followingWindow=axis.inPositionWindow_Pulse;
        binding.rotaryModulo=axis.rotaryModulo;
        binding.scalePointer=reinterpret_cast<std::uintptr_t>(axis.pScaleActualPos);
        binding.feedbackMode=static_cast<std::uint32_t>(axis.fbMode);
        binding.reverseFlags=static_cast<std::uint8_t>((axis.isReverse?1U:0U)|(axis.Axis_Reverse?2U:0U));
        binding.axisType=static_cast<std::uint8_t>(axis.axisType);
        binding.configurationFlags=static_cast<std::uint8_t>((axis.isHomed?1U:0U)|
            (axis.travelLimit1Enable?2U:0U)|(axis.travelLimit2Enable?4U:0U)|(axis.travelLimit3Enable?8U:0U)|
            (axis.enablePitch?16U:0U)|(axis.enableBacklash?32U:0U)|(axis.useShortestPath?64U:0U));
        binding.travelLimits={{axis.travelLimit1Negative_unit,axis.travelLimit1Positive_unit,
            axis.travelLimit2Negative_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Negative_unit,axis.travelLimit3Positive_unit}};
        binding.compensationParameters={{axis.backlashAmount_Pos_mm,axis.backlashAmount_Neg_mm,axis.backlashSpeed,
            axis.pitchStartPos_mm,axis.pitchStep_mm,axis.pitchSpeed_mm_s}};
        for(unsigned k=0U;k<6U;++k)if(!std::isfinite(binding.travelLimits[k])||!std::isfinite(binding.compensationParameters[k]))
            return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
        frozen.pulsePerNative[i]=ppu;frozen.axisMask|=1U<<i;
        {
            const double accTime=independentEdm?EDM385::IndependentCycleSeconds:(axis.G00_acc_time<0.001?0.2:axis.G00_acc_time);
            const double decTime=independentEdm?EDM385::IndependentCycleSeconds:(std::max)(axis.G00_dec_time<0.001?0.2:axis.G00_dec_time,
                axis.Stop_dec_time<0.001?0.2:axis.Stop_dec_time);
            if(!SpherePositive(accTime)||!SpherePositive(decTime))return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
            const double vmax=axis.maxVel_PPS/ppu;
            double acc=vmax/accTime,dec=vmax/decTime;
            if(independentEdm){
                EDM385::Native6 velocity{};velocity[i]=vmax;EDM384Axes::Budget normal{};
                if(!EDM385::BuildIndependentBudget(velocity,1U<<i,normal))
                    return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
                acc=normal.acceleration[i];dec=normal.deceleration[i];
            }
            if(!SpherePositive(vmax)||!SpherePositive(acc)||!SpherePositive(dec))
                return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
            installedAxisBudget.velocity[i]=vmax;
            installedAxisBudget.acceleration[i]=acc;
            installedAxisBudget.deceleration[i]=dec;
            // The motion owner independently reconstructs the NC-frozen
            // budget. Signed EDM derives it solely from MAX_Speed; its own
            // configured ramp/caps are resolved by the actual active leg.
            if(process.config.flushAxisLimitsEnabled&&
               (std::memcmp(&process.config.flushAxisVelocityMmS[i],&vmax,sizeof(double))!=0||
                std::memcmp(&process.config.flushAxisAccelerationMmS2[i],&acc,sizeof(double))!=0||
                std::memcmp(&process.config.flushAxisDecelerationMmS2[i],&dec,sizeof(double))!=0))
                return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
            const bool line=request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&request.geometry.travelMm>0.0;
            const bool moves=line?request.geometry.startMCS[i]!=request.geometry.endMCS[i]:i<3U;
            if(moves){
                rampSeconds=(std::max)(rampSeconds,accTime);stopSeconds=(std::max)(stopSeconds,decTime);
                if(!line&&i<3U){
                    request.velocityCapMmS=(std::min)(request.velocityCapMmS,vmax);
                    // Preserve the conservative legacy curvature reserve and
                    // HOLD/history budget; a_i=t_i*a + curvature_i*v^2.
                    request.accelerationMmS2=(std::min)(request.accelerationMmS2,0.5*acc);
                    request.decelerationMmS2=(std::min)(request.decelerationMmS2,0.5*dec);
                }
            }
            const double curvature=i<3U?request.geometry.maxCircleCurvatureAbs[i]:0.0;
            if(!std::isfinite(curvature)||curvature<0.0)return reject(MotionG76SphereAdmissionGate::RuntimeRequest,static_cast<int>(i));
            if(curvature>0.0)request.velocityCapMmS=(std::min)(request.velocityCapMmS,
                std::sqrt(0.5*(std::min)(acc,dec)/curvature));
            if(SphereRotary(request,i))request.rotaryToleranceDeg=(std::min)(request.rotaryToleranceDeg,axis.inPositionWindow_Pulse/ppu);
            else request.positionToleranceMm=(std::min)(request.positionToleranceMm,axis.inPositionWindow_Pulse/ppu);
        }
        const double startPulse=axis.logicalCmdPos.Load(),native=request.geometry.startMCS[i],expected=native*ppu;
        // Fixed C/U/V keep the accepted raw pulse, including its revolution.
        // A retained authored spelling additionally needs the exact producer
        // receipt and current shared tail; modulo similarity alone is no proof.
        const auto retainedNative=[&](const MotionCncPathTail& previous) noexcept {
            const std::uint32_t bit=1U<<i;
            return previous.valid&&previous.identity.IsAssigned()&&(previous.axisMask&bit)!=0U&&
                (previous.validAxisMask&bit)!=0U&&previous.identity.epoch==epoch&&
                previous.identity.source==MotionCommandSource::NC_MEMORY&&previous.ownerLease.Matches(owner)&&
                previous.translationGeneration==frame.generation&&m_g00ProducerQueueTailEpoch==epoch&&
                m_g00ProducerQueueTailOwnerLease.Matches(owner)&&(m_g00ProducerQueueTailValidMask&bit)!=0U&&
                previous.endPulse[i]==startPulse&&m_g00ProducerQueueTailPulse[i]==startPulse&&
                MotionG76SphereFixedAxes::SameBits(previous.endMCS[i],native);
        };
        const bool baseline=axis.axisType==AxisType::LINEAR?
            std::isfinite(expected)&&std::fabs(startPulse-expected)<=64.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,std::fabs(startPulse)):
            MotionG76SphereFixedAxes::MatchesNative(native,startPulse,axis.resolution_PPR,axis.finalLead,
                axis.axisType==AxisType::ROTARY,axis.rotaryModulo)||
            (axis.axisType==AxisType::ROTARY&&(retainedNative(m_zcFeedProducerTail)||retainedNative(m_rotaryFeedProducerTail)));
        if(!std::isfinite(startPulse)||!baseline)return reject(MotionG76SphereAdmissionGate::NativeBaseline,static_cast<int>(i));
        double envelopeMin=0.0,envelopeMax=0.0;
        if(!MotionG76SphereFlush::AxisEnvelope(request.geometry,flush,i,envelopeMin,envelopeMax,request.retreatMode))
            return reject(MotionG76SphereAdmissionGate::FlushEnvelope,static_cast<int>(i));
        if(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,envelopeMin)||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,envelopeMax))
            return reject(MotionG76SphereAdmissionGate::Travel,static_cast<int>(i));
        const bool enabled[3]={axis.travelLimit1Enable&&frame.storedStrokeMode==22,axis.travelLimit2Enable,axis.travelLimit3Enable};
        const double lows[3]={axis.travelLimit1Negative_unit,axis.travelLimit2Negative_unit,axis.travelLimit3Negative_unit};
        const double highs[3]={axis.travelLimit1Positive_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Positive_unit};
        double low=-(std::numeric_limits<double>::max)(),high=(std::numeric_limits<double>::max)();bool bounded=false;
        if(axis.isHomed)for(unsigned k=0U;k<3U;++k)if(enabled[k]){
            if(!std::isfinite(lows[k])||!std::isfinite(highs[k])||lows[k]>=highs[k])return reject(MotionG76SphereAdmissionGate::Travel,static_cast<int>(i));
            low=(std::max)(low,lows[k]);high=(std::min)(high,highs[k]);bounded=true;
        }
        if(bounded){request.travelMask|=1U<<i;request.travelMin[i]=low;request.travelMax[i]=high;}
        const int slot=command.axisCount++;command.axisIndices[slot]=static_cast<int>(i);
        command.mem_startPos[slot]=startPulse;command.targetPos[slot]=startPulse;command.mem_ratio[slot]=ppu;
    }
    if(request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&request.geometry.travelMm>0.0){
        EDM384Axes::Limits active{};
        if(!SphereNativeLineLimits(request.geometry,installedAxisBudget,request.retreatMode,active))
            return reject(MotionG76SphereAdmissionGate::RuntimeRequest);
        request.velocityCapMmS=(std::min)(request.velocityCapMmS,active.velocity);
        const double reserve=independentEdm?1.0:0.5;
        request.accelerationMmS2=(std::min)(request.accelerationMmS2,reserve*active.acceleration);
        request.decelerationMmS2=(std::min)(request.decelerationMmS2,reserve*active.deceleration);
    }
    // Keep the same symmetric nominal/return budget. Signed EDM uses only
    // the MAX_Speed-derived envelope, independently of legacy G00/STOP times.
    const double commonRamp=(std::min)(request.accelerationMmS2,request.decelerationMmS2);
    request.accelerationMmS2=commonRamp;request.decelerationMmS2=commonRamp;
    if(command.axisCount!=static_cast<int>(SpherePresentAxisCount(request))||!NCG76SphereCycle::ValidateRequest(request))return reject(MotionG76SphereAdmissionGate::RuntimeRequest);
    if((flush.enabled||flush.axisLimitsEnabled)&&!NCG76SphereCycle::ValidateFlushConfig(request,flush))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    command.accTime=rampSeconds;command.decTime=stopSeconds;command.targetVel=1.0;
    AssignExecutionIdentity(command,epoch,MotionCommandSource::NC_MEMORY,owner);
    LifecycleCommitReservationGuard reservation(*this,command.execution);
    if(!reservation.IsAcquired()||!SpherePowerOff()||!MatchesNCTranslation(frame)||
        !HasExactExecutionDrainAcknowledgementImpl(epoch,owner,true))return reject(MotionG76SphereAdmissionGate::Reservation);
    const auto previous=m_g76SphereClaim.load(std::memory_order_acquire);
    m_g76SphereEndpointCommitted.store(0ULL,std::memory_order_release);
    m_g76SphereParentBound.store(0ULL,std::memory_order_release);
    // A signed SIM parent requires this exact independent data snapshot.
    // Publication failure rejects admission; no process-mode AIR fallback.
    process.identity=command.execution;process.owner=owner;
    hold.identity=command.execution;hold.owner=owner;
    if(!m_g76SphereHoldRequests.Publish(hold))return reject(MotionG76SphereAdmissionGate::Publication);
    if(!m_g76SphereProcessRequests.Publish(process)&&frozen.signedMotionEnabled)
        return reject(MotionG76SphereAdmissionGate::Publication);
    if(!m_g76SphereRequests.Publish(frozen))return reject(MotionG76SphereAdmissionGate::Publication);
    m_g76SpherePhysicalClaim.store(request.physicalZManual?request.session:0ULL,std::memory_order_release);
    m_g76SphereClaim.store(request.session,std::memory_order_release);
    m_g76SphereBarrier.store(request.session,std::memory_order_release);
    if(!TryEnqueueMotionCommand(command)){
        m_g76SphereClaim.store(previous,std::memory_order_release);m_g76SphereBarrier.store(0ULL,std::memory_order_release);return reject(MotionG76SphereAdmissionGate::Enqueue);
    }
    m_g76SphereProducerHighWater=request.session;m_g76SphereProducerControl=G76SphereControl{};
    m_g76SphereInspectionProducer=G76SphereInspectionIntent{};
    m_g76SphereInspectionProducerBlocked.store(false,std::memory_order_release);
    identity=command.execution;return true;
}

bool MotionCore::IsG76SphereCommandAuthorized(const MotionCommand& command) const noexcept
{
    if(!IsMotionEDMCommissioningMarkerEncodingValid(command)||!command.g76SphereAir||
        command.commandPathMode!=MotionCommandPathMode::EXACT_STOP||command.mode!=InterpolationMode::LINEAR||
        command.ownerLease.owner!=MotionOwner::AUTO||command.execution.source!=MotionCommandSource::NC_MEMORY||!m_CompEngine.IsSealed()||
        !MatchesNCTranslation(command.sourceTranslation)||!IsNCTranslationAxisIdentityCurrent(command.sourceTranslation)||
        m_pContexts==nullptr||m_pContexts->size()<3U)return false;
    G76SphereFrozen frozen{};
    if(!m_g76SphereRequests.Read(frozen).Ready()||!frozen.request.session||
        m_g76SphereClaim.load(std::memory_order_acquire)!=frozen.request.session||
        std::memcmp(&command,&frozen.command,sizeof(command))!=0||frozen.axisMask!=SphereAxisMask(frozen.request)||
        SphereCommandMask(command)!=frozen.axisMask||command.axisCount!=static_cast<int>(SpherePresentAxisCount(frozen.request))||
        m_pContexts->size()<SphereAxisCount(frozen.request)||!SphereManualGeometry(frozen.request)||
        !NCG76SphereRetreat::ValidMode(frozen.request.geometry,frozen.request.retreatMode)||
        (frozen.flushFeatureFlags&~15U)!=0U||
        (frozen.flushFeatureFlags!=0U&&(frozen.flushFeatureFlags&1U)==0U)||
        (frozen.flushFeatureFlags!=0U&&!frozen.signedMotionEnabled)||
        (frozen.request.retreatMode!=0U&&!frozen.signedMotionEnabled)||
        frozen.holdDeclaredPreviousParents>NCHoldPolicy::MaximumPreviousParents||
        (!frozen.holdRecoveryEnabled&&frozen.holdDeclaredPreviousParents!=0U)||
        frozen.finishRequiredCount>99U)return false;
    G76SphereHoldFrozen hold{};
    if(!m_g76SphereHoldRequests.Read(hold).Ready()||hold.session!=frozen.request.session||
       !SphereIdentity(hold.identity,command.execution)||!hold.owner.Matches(command.ownerLease)||
       hold.enabled!=frozen.holdRecoveryEnabled||hold.declaredPreviousParents!=frozen.holdDeclaredPreviousParents||
       hold.finishRequiredCount!=frozen.finishRequiredCount||hold.finishRequiredCount>99U||
       hold.orbitInspectionRounds!=frozen.request.orbitInspectionRounds||
       hold.orbitThresholdScaled!=frozen.request.orbitThresholdScaled||
       hold.orbitRetryLimit!=frozen.request.orbitRetryLimit||
       !SphereOrbitCompletionConfigValid(hold.orbitInspectionRounds,hold.orbitThresholdScaled,hold.orbitRetryLimit)||
       (hold.enabled&&!NCHoldRecovery::Valid(hold.config))||
       !MotionG76SphereMachiningTime::Matches(hold.machiningTime,frozen.request))return false;
    const auto kind=frozen.request.geometry.kind;
    if(frozen.orbitFeedsEnabled&&(!frozen.signedMotionEnabled||
       (kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour)))return false;
    if(kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
       (!NCG76OrbitContour::ModeP(frozen.request.geometry)||
        !frozen.orbitFeedsEnabled||!frozen.signedMotionEnabled||frozen.request.parameterFeeds||
        (frozen.request.retreatMode!=0U&&frozen.request.retreatMode!=2U)))return false;
    if((kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&
        kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
        kind!=NCG76SphereRuntimeGeometry::Kind::Arc&&kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour)||
        (kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&!frozen.signedMotionEnabled))return false;
    const auto& request=frozen.request;
    if(request.orbitInspectionRounds&&(!frozen.signedMotionEnabled||!frozen.orbitFeedsEnabled||
       (kind!=NCG76SphereRuntimeGeometry::Kind::Sphere&&kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour)||
       hold.finishRequiredCount!=0U))return false;
    if(hold.finishRequiredCount!=0U&&(kind==NCG76SphereRuntimeGeometry::Kind::Sphere||
       kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour||
       !std::isfinite(request.geometry.travelMm)||request.geometry.travelMm<=0.0||
       !frozen.signedMotionEnabled))return false;
    if(request.parameterFeeds){
        if(kind==NCG76SphereRuntimeGeometry::Kind::Sphere||
           (kind==NCG76SphereRuntimeGeometry::Kind::Arc&&request.retreatMode!=2U)||
           !std::isfinite(request.nominalFeedCapMmMin)||request.nominalFeedCapMmMin<0.0||
           request.nominalFeedCapMmMin>100.0)return false;
    }else if(request.nominalFeedCapMmMin!=0.0)return false;
    if(m_g76SphereRunning&&m_g76SphereFrozen.request.session==request.session){
        const auto& active=m_g76SphereFrozen.request;
        if(active.physicalZManual!=request.physicalZManual||active.parameterFeeds!=request.parameterFeeds||active.geometry.kind!=kind||
           (kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
            NCG76OrbitContour::ModeP(active.geometry)!=NCG76OrbitContour::ModeP(request.geometry))||
           (kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&
            NCG76Mode::IsPolygon(NCG76OrbitContour::ModeP(request.geometry))&&
            active.geometry.orbit.source.polygonSides!=request.geometry.orbit.source.polygonSides)||
           frozen.orbitFeedsEnabled!=m_g76SphereFrozen.orbitFeedsEnabled||
           active.totalLimitUs!=request.totalLimitUs||active.additionalUs!=request.additionalUs||
           active.orbitInspectionRounds!=request.orbitInspectionRounds||
           active.orbitThresholdScaled!=request.orbitThresholdScaled||
           active.orbitRetryLimit!=request.orbitRetryLimit||
           !NCMachiningTime::SameConfig(hold.machiningTime,m_g76SphereHoldFrozen.machiningTime)||
           !MotionG76SphereFixedAxes::SameBits(active.nominalFeedCapMmMin,request.nominalFeedCapMmMin)||
           !MotionG76SphereFixedAxes::SameBits(active.feedMmMin,request.feedMmMin)||
           !MotionG76SphereFixedAxes::SameBits(active.geometry.feedMmMin,request.geometry.feedMmMin)||
           hold.enabled!=m_g76SphereHoldFrozen.enabled||
           hold.declaredPreviousParents!=m_g76SphereHoldFrozen.declaredPreviousParents||
           hold.finishRequiredCount!=m_g76SphereHoldFrozen.finishRequiredCount||
           hold.orbitRetryLimit!=m_g76SphereHoldFrozen.orbitRetryLimit||
           (hold.enabled&&!NCHoldRecovery::SameConfig(hold.config,m_g76SphereHoldFrozen.config)))return false;
    }
    for(unsigned i=0U;i<SphereAxisCount(frozen.request);++i){
        if(!SphereAxisPresent(frozen.request,i))continue;
        const auto& axis=(*m_pContexts)[i];const auto& b=frozen.axis[i];
        if(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||axis.resolution_PPR!=b.resolution||
            axis.finalLead!=b.lead||axis.maxVel_PPS!=b.maxVelocity||
            (!frozen.signedMotionEnabled&&(axis.G00_acc_time!=b.accSeconds||
             axis.G00_dec_time!=b.decSeconds||axis.Stop_dec_time!=b.stopSeconds))||axis.machineCoordinateOffsetPulse!=b.machineOffset||
            axis.scaleToMotorRatio!=b.scaleRatio||axis.maxDeviation!=b.maxDeviation||
            !MotionG76SphereFixedAxes::SameBits(axis.rotaryModulo,b.rotaryModulo)||
            axis.inPositionWindow_Pulse!=b.followingWindow||reinterpret_cast<std::uintptr_t>(axis.pScaleActualPos)!=b.scalePointer||
            static_cast<std::uint32_t>(axis.fbMode)!=b.feedbackMode||
            static_cast<std::uint8_t>((axis.isReverse?1U:0U)|(axis.Axis_Reverse?2U:0U))!=b.reverseFlags||
            static_cast<std::uint8_t>(axis.axisType)!=b.axisType||
            static_cast<std::uint8_t>((axis.isHomed?1U:0U)|(axis.travelLimit1Enable?2U:0U)|
                (axis.travelLimit2Enable?4U:0U)|(axis.travelLimit3Enable?8U:0U)|
                (axis.enablePitch?16U:0U)|(axis.enableBacklash?32U:0U)|(axis.useShortestPath?64U:0U))!=b.configurationFlags||
            b.travelLimits!=std::array<double,6U>{{axis.travelLimit1Negative_unit,axis.travelLimit1Positive_unit,
                axis.travelLimit2Negative_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Negative_unit,axis.travelLimit3Positive_unit}}||
            b.compensationParameters!=std::array<double,6U>{{axis.backlashAmount_Pos_mm,axis.backlashAmount_Neg_mm,axis.backlashSpeed,
                axis.pitchStartPos_mm,axis.pitchStep_mm,axis.pitchSpeed_mm_s}})return false;
    }
    return true;
}

bool MotionCore::RequestG76SphereHold(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t holdSequence,std::uint64_t startFloor) noexcept
{
    G76SphereFrozen frozen{};MotionG76SphereFeedback report{};
    if(!session||!holdSequence||!m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
        !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
        m_g76SphereClaim.load(std::memory_order_acquire)!=session||!IsMotionOwnerLeaseCurrent(owner)||
        GetCurrentExecutionEpoch()!=identity.epoch||HasPendingSafetyOrRecoveryRequests()||
        !SpherePowerOff()||AlarmManager::GetInstance().HasAlarm()||
        (ReadG76SphereFeedback(report)&&report.session==session&&
            (report.terminal||!SphereIdentity(report.identity,identity)||!report.owner.Matches(owner)))||
        (m_g76SphereProducerControl.session==session&&holdSequence<=m_g76SphereProducerControl.holdSequence))return false;
    G76SphereControl intent{};intent.session=session;intent.identity=identity;intent.owner=owner;
    intent.holdSequence=holdSequence;intent.startSequence=startFloor;intent.hold=true;
    if(!m_g76SphereControls.ProducerTryPush(intent))return false;
    m_g76SphereProducerControl=intent;return true;
}

bool MotionCore::RequestG76SphereResume(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t holdSequence,std::uint64_t startSequence) noexcept
{
    MotionG76SphereFeedback report{};
    if(!session||!holdSequence||!startSequence||!ReadG76SphereFeedback(report)||report.session!=session||
        !report.started||report.terminal||report.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart||
        report.holdSequence!=holdSequence||!SphereIdentity(report.identity,identity)||!report.owner.Matches(owner)||
        m_g76SphereClaim.load(std::memory_order_acquire)!=session||!IsMotionOwnerLeaseCurrent(owner)||
        GetCurrentExecutionEpoch()!=identity.epoch||HasPendingSafetyOrRecoveryRequests()||
        m_g76SphereProducerControl.session!=session||m_g76SphereProducerControl.holdSequence!=holdSequence||
        startSequence<=m_g76SphereProducerControl.startSequence||
        (report.cycle.holdRecoveryEnabled&&!report.holdRecoveryResumeReady)||
        (m_g76SphereInspectionProducer.session==session&&
         m_g76SphereInspectionProducer.holdSequence==holdSequence&&m_g76SphereInspectionProducer.direction!=0))return false;
    if(report.cycle.holdRecoveryEnabled&&report.cycle.inspectionTraceCount!=0U){
        EDMShortCircuitProtection::Snapshot collision{};
        if(m_g76SphereInspectionProducerBlocked.load(std::memory_order_acquire)||
           !ReadCollisionProtection(collision)||!collision.valid||
           (collision.shortActive&&!m_collisionManualBypass.load(std::memory_order_acquire)))return false;
    }
    G76SphereControl intent{};intent.session=session;intent.identity=identity;intent.owner=owner;
    intent.holdSequence=holdSequence;intent.startSequence=startSequence;
    if(!m_g76SphereControls.ProducerTryPush(intent))return false;
    m_g76SphereProducerControl=intent;return true;
}

bool MotionCore::RequestG76SphereInspectionJog(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t holdSequence,int axis,int direction,
    std::uint64_t inspectionSequence) noexcept
{
    // Repeated NC heartbeat commands refresh a scoped lease without filling
    // the intent queue. Repeating an old sequence never extends its lifetime.
    const bool stop=axis==-1&&direction==0;
    if(!inspectionSequence||(!stop&&(axis<0||axis>2||(direction!=1&&direction!=-1))))return false;
    MotionG76SphereFeedback report{};G76SphereHoldFrozen hold{};
    if(!session||!holdSequence||!ReadG76SphereFeedback(report)||report.session!=session||
       !report.started||report.terminal||!report.cycle.holdRecoveryEnabled||!report.cycle.holdActive||
       report.holdSequence!=holdSequence||report.cycle.holdSequence!=holdSequence||
       !SphereIdentity(report.identity,identity)||!report.owner.Matches(owner)||
       m_g76SphereClaim.load(std::memory_order_acquire)!=session||!IsMotionOwnerLeaseCurrent(owner)||
       GetCurrentExecutionEpoch()!=identity.epoch||HasPendingSafetyOrRecoveryRequests()||
       !SpherePowerOff()||AlarmManager::GetInstance().HasAlarm()||
       !m_g76SphereHoldRequests.Read(hold).Ready()||!hold.enabled||hold.session!=session||
       !SphereIdentity(hold.identity,identity)||!hold.owner.Matches(owner)||!NCHoldRecovery::Valid(hold.config))return false;
    if(!stop){
        EDMShortCircuitProtection::Snapshot collision{};
        if(!ReadCollisionProtection(collision)||!collision.valid||
           (collision.shortActive&&!m_collisionManualBypass.load(std::memory_order_acquire))){
            m_g76SphereInspectionProducerBlocked.store(true,std::memory_order_release);return false;
        }
        if(m_g76SphereInspectionProducerBlocked.load(std::memory_order_acquire))return false;
    }
    auto& previous=m_g76SphereInspectionProducer;
    const bool sameScope=previous.session==session&&previous.holdSequence==holdSequence&&
        SphereIdentity(previous.identity,identity)&&previous.owner.Matches(owner);
    if(sameScope&&inspectionSequence<=previous.sequence)
        return inspectionSequence==previous.sequence&&previous.axis==axis&&previous.direction==direction;
    G76SphereInspectionIntent intent{};intent.session=session;intent.identity=identity;intent.owner=owner;
    intent.holdSequence=holdSequence;intent.sequence=inspectionSequence;intent.axis=axis;intent.direction=direction;
    if(sameScope&&previous.axis==axis&&previous.direction==direction){
        if(!m_g76SphereInspectionLease.Publish(intent))return false;
        previous=intent;
        if(stop)m_g76SphereInspectionProducerBlocked.store(false,std::memory_order_release);
        return true;
    }
    if(!stop&&(!report.holdInspectionJogReady||!report.cycle.held||
       report.cycle.phase!=NCG76SphereCycle::Phase::HeldAtStart||
       (m_g76SphereProducerControl.session==session&&
            m_g76SphereProducerControl.startSequence>report.startSequence)))return false;
    if(!m_g76SphereInspectionLease.Publish(intent))return false;
    if(stop){
        // The data-only STOP page also contains an earlier queued start if the
        // bounded state-change queue is temporarily full.
        (void)m_g76SphereInspectionControls.ProducerTryPush(intent);
        previous=intent;m_g76SphereInspectionProducerBlocked.store(false,std::memory_order_release);return true;
    }
    if(!m_g76SphereInspectionControls.ProducerTryPush(intent))return false;
    previous=intent;return true;
}

bool MotionCore::BindG76SphereParent(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t dispatch,std::uint64_t commit) noexcept
{
    G76SphereFrozen frozen{};
    if(!session||!dispatch||!commit||!m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
        !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
        m_g76SphereClaim.load(std::memory_order_acquire)!=session||!IsMotionOwnerLeaseCurrent(owner)||
        GetCurrentExecutionEpoch()!=identity.epoch||HasPendingSafetyOrRecoveryRequests()||!SpherePowerOff())return false;
    G76SphereParentBinding binding{};
    if(m_g76SphereParentBound.load(std::memory_order_acquire)==session)
        return m_g76SphereParentBinding.Read(binding).Ready()&&binding.session==session&&
            binding.dispatch==dispatch&&binding.commit==commit&&SphereIdentity(binding.identity,identity)&&binding.owner.Matches(owner);
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired()||!IsMotionOwnerLeaseCurrent(owner)||GetCurrentExecutionEpoch()!=identity.epoch)return false;
    binding.session=session;binding.dispatch=dispatch;binding.commit=commit;binding.identity=identity;binding.owner=owner;
    if(!m_g76SphereParentBinding.Publish(binding))return false;
    m_g76SphereParentBound.store(session,std::memory_order_release);return true;
}

void MotionCore::SetG76SphereInputDiagnostic(MotionG76SphereInputStage stage,
    MotionG76SphereInputGate gate,int axis) noexcept
{
    auto& d=m_g76SphereInputObservation;const auto& input=m_g76SphereInput;
    const auto& frozen=stage==MotionG76SphereInputStage::Observe&&!m_g76SphereRunning?
        m_g76SpherePendingWorkspace:m_g76SphereFrozen;
    d.valid=true;d.stage=stage;d.gate=gate;d.axis=static_cast<std::int8_t>(axis);
    d.tick=m_ncSettleRuntimeCycleTick;d.inputTick=input.tick;d.physicalSequence=input.physicalSequence;
    d.nowUs=input.nowUs;d.clockFrequency=m_edmZClockFrequency;d.inputSession=m_g76SphereInputSession;
    d.expectedSession=frozen.request.session;d.inputIdentity=m_g76SphereInputIdentity;
    using namespace MotionG76SphereInputFlags;
    std::uint32_t flags=d.flags&RequestRead;
    if(m_g76SphereClaim.load(std::memory_order_acquire))flags|=Claim;
    if(m_g76SphereRunning)flags|=Running;
    if(m_ncSettleRuntimeCycleValid)flags|=PdoCycleValid;
    if(m_ncSettleRuntimeCycleContiguous)flags|=Contiguous;
    if(input.powerInhibited)flags|=PowerInhibited|InputPowerInhibited;
    if(m_edmZClockValid)flags|=ClockValid;
    if(m_edmZClockFrequency)flags|=ClockFrequency;
    if(m_edmZMonotonicUs)flags|=ClockNow;
    if(m_pContexts&&m_pContexts->size()>=3U)flags|=Contexts;
    if(input.sourceValid)flags|=SourceValid;
    if(input.pdoValid)flags|=InputPdoValid;
    if(input.cancel)flags|=Cancelled;
    if(input.tick==m_ncSettleRuntimeCycleTick)flags|=TickMatches;
    if(MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
        frozen.request.session,frozen.command.execution))flags|=ScopeMatches;
    if(input.physicalSequence==input.tick)flags|=PhysicalSequenceMatches;
    if(m_ncSettleRuntimeCycleTick>=m_g76SphereAdmissionTick)flags|=AdmissionTickValid;
    d.flags=flags;
}

void MotionCore::CaptureG76SphereInputFault() noexcept
{
    if(m_g76SphereCurrent.firstInputFault.valid)return;
    m_g76SphereCurrent.firstInputFault=m_g76SphereInputObservation;
    // EDM386_GAP_FAULT_TRACE_BEGIN
    // This executes once on the first existing input fault. No fresh read,
    // clock acquisition, conversion, formatting or control decision here.
    if(m_g76SphereFrozen.request.physicalZManual){
        auto& gap=m_g76SphereCurrent.firstGapInputFault;
        gap.captured=true;gap.readSucceeded=m_g76SphereAnalogRead;
        if(gap.readSucceeded)gap.sample=m_g76SphereAnalogInput;
        gap.profileConfigured=m_g76SpherePhysicalProfileConfigured;
        if(gap.profileConfigured)gap.profile=m_g76SpherePhysicalProfile;
        const auto& receipt=m_g76SpherePhysicalSource.DiagnosticReceipt();
        gap.adapterState=receipt.state;gap.adapterReason=receipt.reason;
        gap.adapterSequence=receipt.sample.sequence;gap.adapterCapturedUs=receipt.sample.capturedAtUs;
        const auto& frame=m_g76SpherePhysicalSource.CurrentFrame();
        gap.frameSequence=frame.sequence;gap.frameCapturedUs=frame.capturedAtUs;
        gap.sourceReady=m_g76SpherePhysicalSource.Ready();
    }
    // EDM386_GAP_FAULT_TRACE_END
    // Keep values, sequence and validity from the same pre-cancellation input.
    // Later source callbacks may reset their working sample after containment.
    m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
    m_g76SphereCurrent.commandMCS=m_g76SphereRunning?m_g76SphereCycle.Command().commandMCS:
        m_g76SphereFrozen.request.geometry.startMCS;
    m_g76SphereCurrent.inputSequence=m_g76SphereInput.physicalSequence;
    m_g76SphereCurrent.inputValidAxisMask=m_g76SphereInputObservation.validAxisMask;
}

bool MotionCore::G76LineTransportScopeCurrent() const noexcept
{
    const auto& frozen = m_g76SphereFrozen;
    const auto& request = frozen.request;
    const auto& command = frozen.command;
    return m_g76SphereRunning && request.session && request.physicalZManual &&
        request.geometry.kind == NCG76SphereRuntimeGeometry::Kind::Line && frozen.axisMask == 7U &&
        request.geometry.startMCS[0] == request.geometry.endMCS[0] &&
        request.geometry.startMCS[1] == request.geometry.endMCS[1] &&
        request.geometry.startMCS[2] != request.geometry.endMCS[2] &&
        request.retreatMode == 0U && !request.totalLimitUs && !request.additionalUs &&
        request.geometry.travelMm > 0.0 && m_Group.isActive && m_Group.currentCmd.g76SphereAir &&
        !m_Group.enableTransform && !m_Group.enableHistory && m_Group.jumpManager.state == JumpState::IDLE &&
        !m_pathHold.sourceSeen && m_Group.axisCount == 3 && m_Group.pathMode == PathMode::EXACT_STOP &&
        m_Group.feedrateOverride == 1.0 && SphereIdentity(m_Group.currentCmd.execution, command.execution) &&
        m_Group.currentCmd.ownerLease.Matches(command.ownerLease) && IsMotionOwnerLeaseCurrent(command.ownerLease) &&
        GetCurrentExecutionEpoch() == command.execution.epoch &&
        m_g76SphereClaim.load(std::memory_order_acquire) == request.session &&
        m_g76SpherePhysicalClaim.load(std::memory_order_acquire) == request.session &&
        m_g76SphereParentBound.load(std::memory_order_acquire) == request.session &&
        m_g76SphereProcessRevokedSession.load(std::memory_order_acquire) != request.session &&
        !m_g76SphereInspectionDeadlineUs && !m_g76SphereInspectionBlocked &&
        m_g76SphereControls.empty() && m_g76SphereInspectionControls.empty() &&
        !HasPendingSafetyOrRecoveryRequests() && !HasPendingExecutionEpochChange() &&
        !HasUnacknowledgedSafetyMotionRequest() && !m_safetyControlledStopInProgress &&
        !IsG76SphereResetOutputHeld() && !IsRotarySpindleSessionClaimed() &&
        !AlarmManager::GetInstance().HasAlarm() && SpherePowerOff();
}

bool MotionCore::BeginG76LineTransportGap(std::uint64_t previousTick, std::uint64_t previousUs) noexcept
{
    const auto& input = m_g76SphereInput;
    const auto& request = m_g76SphereFrozen.request;
    // This diagnostic retry policy reuses the existing settle timeout, with a
    // hard five-second ceiling for the entire pause (never renewed by retries).
    if (!SpherePositive(request.settleTimeoutSeconds)) return false;
    const auto budgetUs = static_cast<std::uint64_t>((std::min)(5.0, request.settleTimeoutSeconds) * 1000000.0);
    if (!budgetUs || !m_ncSettleRuntimeCycleContiguous || !m_edmZClockValid || !previousTick || !previousUs ||
        NCG76SphereCycle::Detail::ClassifyClockFault(previousTick, previousUs, input,
            request.minimumTickUs, request.maximumTickUs) || !G76LineTransportScopeCurrent() ||
        GetCommandAuthorizationFailure(m_Group.currentCmd) != MotionRejectReason::NONE ||
        previousTick != m_g76SphereCycle.Current().tick || previousTick != m_g76SphereProcess.Current().tick ||
        !m_g76SphereCycle.CanQuarantinePhysicalLine() || !m_g76SphereCycle.CanRemainTransportPaused(input.nowUs) ||
        !m_g76SphereProcess.CanQuarantinePhysicalLine() ||
        m_g76SphereCurrent.transportPauseCount == (std::numeric_limits<std::uint32_t>::max)() ||
        !m_g76SpherePhysicalSource.BeginTransportGap(input.nowUs, m_edmZClockFrequency) ||
        !m_g76SphereProcess.SuspendPhysicalLine()) return false;
    m_g76Dropout = G76LineTransportGap{};
    m_g76Dropout.active = true;
    m_g76Dropout.session = request.session;
    m_g76Dropout.firstTick = m_g76Dropout.lastTick = input.tick;
    m_g76Dropout.startedUs = m_g76Dropout.lastUs = input.nowUs;
    m_g76Dropout.budgetUs = budgetUs;
    if (!FreezeG76SphereMemberCommands()) return false;
    m_g76SphereZeroDemand = true;
    m_g76SphereProcessSentTick = m_g76SphereZeroSentTick = 0ULL;
    m_g76SpherePhysicalRange = false;
    m_g76SphereCurrent.transportPaused = true;
    m_g76SphereCurrent.transportPauseTick = input.tick;
    m_g76SphereCurrent.transportPauseBudgetUs = budgetUs;
    m_g76SphereCurrent.transportShortBlocked = m_g76SphereCurrent.transportBudgetExpired = false;
    ++m_g76SphereCurrent.transportPauseCount;
    m_g76SphereCurrent.transportLastValidSample = m_g76TransportLastValid.valid &&
        m_g76TransportLastValid.session == request.session && m_g76TransportLastValid.tick < input.tick;
    m_g76SphereCurrent.transportLastValidTick = m_g76TransportLastValid.tick;
    m_g76SphereCurrent.transportLastValidGapMv = m_g76TransportLastValid.gapMv;
    m_g76SphereCurrent.transportLastValidShort = m_g76TransportLastValid.shortActive;
    PublishG76SphereRuntime();
    return true;
}

// Additional clean empty-RX losses stay in the same frozen pause. They do
// not renew its absolute deadline or carry any previous zero/ADC proof forward.
bool MotionCore::RetryG76LineTransportGap() noexcept
{
    auto& gap = m_g76Dropout;
    const auto& input = m_g76SphereInput;
    const auto& request = m_g76SphereFrozen.request;
    if (!gap.active || gap.session != request.session || !G76LineTransportScopeCurrent() ||
        GetCommandAuthorizationFailure(m_Group.currentCmd) != MotionRejectReason::NONE ||
        m_ncSettleRuntimeCycleValid || !m_edmZClockValid || !input.nowUs || !gap.budgetUs ||
        gap.budgetUs > 5000000ULL || input.nowUs < gap.startedUs ||
        !m_g76SphereCycle.CanRemainTransportPaused(input.nowUs) ||
        NCG76SphereCycle::Detail::ClassifyClockFault(gap.lastTick, gap.lastUs, input,
            request.minimumTickUs, request.maximumTickUs) ||
        m_g76SphereCurrent.transportRetryCount == (std::numeric_limits<std::uint32_t>::max)()) return false;
    if (input.nowUs - gap.startedUs > gap.budgetUs) {
        m_g76SphereCurrent.transportBudgetExpired = true;
        return false;
    }
    if (!FreezeG76SphereMemberCommands()) return false;
    // Requalification may already have consumed the fresh GAP while one final
    // zero ACK is outstanding. Re-suspend that exact receipt, even if it chose
    // a negative demand: no nonzero demand has been released to the axes yet.
    if (gap.resumeTick && !m_g76SphereProcess.ResuspendRequalifiedPhysicalLine()) return false;
    if (!m_g76SpherePhysicalSource.RestartTransportGap(input.nowUs, m_edmZClockFrequency)) return false;
    gap.firstTick = gap.lastTick = input.tick; // most recent missing exchange
    gap.lastUs = input.nowUs;
    gap.sourceReady = false;
    gap.lastSourceSequence = gap.lastSourceUs = 0ULL;
    gap.zeroPreparedTick = gap.zeroSentTick = gap.wireZeroSequence = gap.resumeTick = 0ULL;
    gap.lastStandstillTick = gap.standstillSinceUs = 0ULL;
    gap.standstillSamples = 0U;
    // zeroFrame/zeroSequence are replay-prevention high-water marks and survive.
    m_g76SphereZeroDemand = true;
    m_g76SphereProcessSentTick = m_g76SphereZeroSentTick = 0ULL;
    m_g76SpherePhysicalRange = false;
    ++m_g76SphereCurrent.transportRetryCount;
    m_g76SphereCurrent.transportLastRetryTick = input.tick;
    m_g76SphereCurrent.transportLastValidSample = m_g76TransportLastValid.valid &&
        m_g76TransportLastValid.session == request.session && m_g76TransportLastValid.tick < input.tick;
    m_g76SphereCurrent.transportLastValidTick = m_g76TransportLastValid.tick;
    m_g76SphereCurrent.transportLastValidGapMv = m_g76TransportLastValid.gapMv;
    m_g76SphereCurrent.transportLastValidShort = m_g76TransportLastValid.shortActive;
    PublishG76SphereRuntime();
    return true;
}

bool MotionCore::QualifyG76LineTransportGap() noexcept
{
    auto& gap = m_g76Dropout;
    auto& input = m_g76SphereInput;
    const auto& request = m_g76SphereFrozen.request;
    if (!gap.active || gap.session != request.session || !G76LineTransportScopeCurrent() ||
        GetCommandAuthorizationFailure(m_Group.currentCmd) != MotionRejectReason::NONE ||
        !m_ncSettleRuntimeCycleValid || !m_edmZClockValid || !input.nowUs ||
        input.nowUs < gap.startedUs || !gap.budgetUs || gap.budgetUs > 5000000ULL ||
        !m_g76SphereCycle.CanRemainTransportPaused(input.nowUs) ||
        NCG76SphereCycle::Detail::ClassifyClockFault(gap.lastTick, gap.lastUs, input,
            request.minimumTickUs, request.maximumTickUs)) return false;
    if (input.nowUs - gap.startedUs > gap.budgetUs) {
        m_g76SphereCurrent.transportBudgetExpired = true;
        return false;
    }
    if (!FreezeG76SphereMemberCommands()) return false;
    gap.lastTick = input.tick;
    gap.lastUs = input.nowUs;
    const bool firstGood = input.tick - gap.firstTick == 1ULL;
    if (firstGood == m_ncSettleRuntimeCycleContiguous) return false;
    // A prepared, scoped zero must be the actual handoff this just-returned
    // exchange came from. An unexpected scrub or substituted image is terminal.
    if (gap.zeroPreparedTick && gap.zeroPreparedTick == input.tick - 1ULL &&
        gap.zeroSentTick != gap.zeroPreparedTick) return false;
    if (!gap.sourceReady) {
        const auto result = m_g76SpherePhysicalSource.ObserveAfterTransportGap(m_g76SphereAnalogInput,
            m_g76SphereAnalogRead, input.nowUs, m_edmZClockValid, m_edmZClockFrequency);
        using Result = EDM382Source::TransportGapResult;
        if (result == Result::Fault) return false;
        gap.sourceReady = result == Result::Ready;
    } else if (!m_g76SpherePhysicalSource.Observe(m_g76SphereAnalogInput, m_g76SphereAnalogRead,
            input.nowUs, m_edmZClockValid, m_edmZClockFrequency)) return false;
    if (!gap.sourceReady) return !gap.resumeTick;
    const auto& frame = m_g76SpherePhysicalSource.CurrentFrame();
    if (!m_g76SpherePhysicalSource.CurrentAt(input.nowUs, m_edmZClockFrequency) ||
        !frame.valid || frame.sequence <= gap.lastSourceSequence || frame.capturedAtUs <= gap.lastSourceUs) return false;
    gap.lastSourceSequence = frame.sequence;
    gap.lastSourceUs = frame.capturedAtUs;

    if (m_collisionCurrent.valid) m_g76SphereCurrent.transportShortBlocked = m_collisionCurrent.shortActive;
    const auto revision = m_collisionConfigurationRequested.load(std::memory_order_acquire);
    const bool safeLevel = revision && m_collisionRuntimeConfiguration == revision && m_collisionConfigurationValid &&
        m_collisionCurrent.valid && !m_collisionCurrent.shortActive;
    if (gap.resumeTick) {
        if (!safeLevel || !input.pdoValid || gap.zeroSentTick != gap.resumeTick ||
            gap.resumeTick + 1ULL != input.tick || !G76SphereDropoutStandstill()) return false;
        gap.active = false;
        m_g76SphereCurrent.transportPaused = false;
        m_g76SphereCurrent.transportShortBlocked = false;
        m_g76SphereCurrent.transportResumeTick = input.tick;
        return true;
    }
    // The first recovered exchange follows a bad one. Its ADC/servo status may
    // seed requalification, but it has no motion or E5 authority.
    if (firstGood || !safeLevel || !input.pdoValid || !G76SphereDropoutStandstill()) return true;
    NCG76SphereGapProcess::Input process{};
    process.nominal = m_g76SphereCycle.Current();
    process.gapFeedPhase = m_g76SphereCycle.ResolveGapFeedPhase();
    process.candidateMCS = m_g76SphereCycle.Command().commandMCS;
    process.session = request.session;
    process.tick = input.tick;
    process.nowUs = input.nowUs;
    process.sourceValid = input.sourceValid;
    process.powerInhibited = input.powerInhibited;
    process.physicalFrame = frame;
    input.physicalSettled = true;
    input.zeroFinalSend = true;
    input.zeroSendTick = gap.zeroSentTick;
    if (!m_g76SphereProcess.RequalifyPhysicalLine(process, gap.budgetUs) ||
        !m_g76SphereCycle.RequalifyPhysicalLine(input, gap.budgetUs)) return false;
    // Keep serializing the qualified zero on this baseline tick. The next
    // exchange must acknowledge it before Step can consume a new GAP demand.
    gap.resumeTick = input.tick;
    return true;
}

void MotionCore::ObserveG76SphereRuntimeInput(bool cleanReceiveDeadline, bool confirmedCombinedReturn) noexcept
{
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    // The existing NIC handoff for T arrives before input T+1. Seal that
    // pending row only now; a missing stage remains explicitly unobserved.
    m_g01ResponseTrace.Advance(m_ncSettleRuntimeCycleTick,m_edmZMonotonicUs,
        m_ncSettleRuntimeCycleValid,confirmedCombinedReturn);
    const auto responseSession=m_g01ResponseTrace.ActiveSession();
    const bool responseHeld=m_g76SphereRunning&&m_g76SphereCurrent.session==responseSession&&
        m_g76SphereCycle.Current().held;
    const bool responseEnded=responseSession&&(!m_g76SphereRunning||
        m_g76SphereCurrent.session!=responseSession||m_g76SphereCurrent.terminal);
    // An older retained capture must not enable printing during a new run.
    const bool responseDumpSafe=!m_g76SphereRunning||m_g76SphereCurrent.terminal||
        m_g76SphereCycle.Current().held;
    m_g01ResponseDumpSafe.store(responseDumpSafe,std::memory_order_release);
    if(responseHeld||responseEnded){
        m_g01ResponseTrace.Stop(responseHeld?EDMG01ResponseTrace::Held:EDMG01ResponseTrace::Terminal);
        m_g01ResponseTrace.AllowDump();
    }
    // EDM386_G01_RESPONSE_TRACE_END
    const auto previousTick = m_g76SphereInput.tick;
    const auto previousUs = m_g76SphereInput.nowUs;
    m_g76SphereInput=NCG76SphereCycle::Input{};
    m_g76SphereInputSession=0ULL;m_g76SphereInputIdentity=MotionExecutionIdentity{};
    m_g76SphereInputObservation=MotionG76SphereInputDiagnostic{};
    if(!m_g76SphereClaim.load(std::memory_order_acquire)){
        if(m_g76SphereRunning)
            SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Observe,MotionG76SphereInputGate::ClaimMissing);
        if(m_g76Dropout.active && m_g76SphereRunning) FailG76SphereRuntime(1U,true);
        return;
    }
    auto& input=m_g76SphereInput;
    input.tick=m_ncSettleRuntimeCycleTick;input.nowUs=m_edmZMonotonicUs;
    input.physicalSequence=input.tick;
    input.pdoValid=m_ncSettleRuntimeCycleValid&&m_ncSettleRuntimeCycleContiguous;
    input.powerInhibited=SpherePowerOff();
    const bool read=m_g76SphereRunning||m_g76SphereRequests.Read(m_g76SpherePendingWorkspace).Ready();
    const auto& frozen=m_g76SphereRunning?m_g76SphereFrozen:m_g76SpherePendingWorkspace;
    if(read)m_g76SphereInputObservation.flags|=MotionG76SphereInputFlags::RequestRead;
    if(read){m_g76SphereInputSession=frozen.request.session;m_g76SphereInputIdentity=frozen.command.execution;
        input.actualMCS=frozen.request.geometry.startMCS;}
    const auto expectedMask=SphereAxisMask(frozen.request);
    const bool mapping=(expectedMask&7U)==7U&&frozen.axisMask==expectedMask&&
        SphereCommandMask(frozen.command)==expectedMask&&
        frozen.command.axisCount==static_cast<int>(SpherePresentAxisCount(frozen.request));
    const bool inspectRecovery = m_g76Dropout.active && m_ncSettleRuntimeCycleValid;
    bool source=read&&mapping&&frozen.request.session&&(input.pdoValid||inspectRecovery)&&input.powerInhibited&&
        m_edmZClockValid&&m_edmZClockFrequency!=0ULL&&input.nowUs&&m_pContexts!=nullptr&&m_pContexts->size()>=SphereAxisCount(frozen.request);
    MotionG76SphereInputGate gate=MotionG76SphereInputGate::None;int failedAxis=-1;
    if(!read)gate=MotionG76SphereInputGate::RequestUnavailable;
    else if(!frozen.request.session)gate=MotionG76SphereInputGate::SessionMissing;
    else if(!m_ncSettleRuntimeCycleValid)gate=MotionG76SphereInputGate::PdoCycleInvalid;
    else if(!m_ncSettleRuntimeCycleContiguous&&!inspectRecovery)gate=MotionG76SphereInputGate::PdoNotContiguous;
    else if(!input.powerInhibited)gate=MotionG76SphereInputGate::PowerNotInhibited;
    else if(!m_edmZClockValid)gate=MotionG76SphereInputGate::ClockInvalid;
    else if(!m_edmZClockFrequency)gate=MotionG76SphereInputGate::ClockFrequencyMissing;
    else if(!input.nowUs)gate=MotionG76SphereInputGate::ClockTimeMissing;
    else if(!m_pContexts||m_pContexts->size()<SphereAxisCount(frozen.request))gate=MotionG76SphereInputGate::ContextsMissing;
    else if(!mapping)gate=MotionG76SphereInputGate::InputScopeMismatch;
    for(unsigned i=0U;source&&i<SphereAxisCount(frozen.request);++i){
        if(!SphereAxisPresent(frozen.request,i))continue;
        const auto& axis=(*m_pContexts)[i];MotionServoInputSnapshot physical{};
        failedAxis=static_cast<int>(i);
        if(!axis.isExist){source=false;gate=MotionG76SphereInputGate::AxisMissing;}
        else if(axis.isFirstCycle){source=false;gate=MotionG76SphereInputGate::AxisFirstCycle;}
        else if(axis.isFault){source=false;gate=MotionG76SphereInputGate::AxisFault;}
        else if(axis.isLagAlarm){source=false;gate=MotionG76SphereInputGate::AxisLag;}
        else if(!(inspectRecovery ? ReadG76DropoutCurrentServoInput(static_cast<int>(i),physical) :
            ReadMotionServoInputCompatibilitySnapshotBySlot(static_cast<int>(i),physical))){
            source=false;gate=MotionG76SphereInputGate::ServoSnapshotUnavailable;
        }else if((physical.StatusWord&0x006FU)!=0x0027U){source=false;gate=MotionG76SphereInputGate::ServoStatus;}
        else if(physical.ModesOfOperationDisplay!=9){source=false;gate=MotionG76SphereInputGate::ServoMode;}
        m_g76SphereInputObservation.statusWord=physical.StatusWord;
        m_g76SphereInputObservation.mode=physical.ModesOfOperationDisplay;
        m_g76SphereInputObservation.rawPosition=physical.ActualPosition;
        if(!source)break;
        double pulse=MotionG80RuntimeProof::DecodeMotorPulse(physical.ActualPosition,
            static_cast<std::uint32_t>(axis.lastRawActPos),axis.unwrappedActPos,
            axis.isReverse,axis.Axis_Reverse,axis.machineCoordinateOffsetPulse);
        if(axis.fbMode==FeedbackSource::LINEAR_SCALE){
            if(axis.pScaleActualPos==nullptr){source=false;gate=MotionG76SphereInputGate::ScalePointerMissing;break;}
            const double scale=static_cast<double>(*axis.pScaleActualPos)*axis.scaleToMotorRatio;
            if(!std::isfinite(scale)){source=false;gate=MotionG76SphereInputGate::ScaleNonfinite;break;}
            if(std::fabs(pulse+axis.machineCoordinateOffsetPulse-scale)>axis.maxDeviation){
                source=false;gate=MotionG76SphereInputGate::ScaleDeviation;break;
            }
            pulse=scale-axis.machineCoordinateOffsetPulse;
        }
        const double nominal=pbc::NominalFeedbackPulse(axis.enablePitch||axis.enableBacklash,
            axis.currentCompOffset_unit,pulse,axis.resolution_PPR,axis.finalLead,axis.mechanicalCompensationFrame);
        if(i<3U){input.actualMCS[i]=nominal/frozen.pulsePerNative[i];source=std::isfinite(input.actualMCS[i]);}
        else {const int slot=SphereCommandSlot(frozen.command,i);
            source=slot>=0&&MotionG76SphereFixedAxes::FeedbackNative(frozen.request.geometry.startMCS[i],
                frozen.command.mem_startPos[slot],nominal,frozen.pulsePerNative[i],input.actualMCS[i]);}
        if(!source){gate=std::isfinite(nominal)?MotionG76SphereInputGate::NativeNonfinite:MotionG76SphereInputGate::NominalNonfinite;break;}
        m_g76SphereInputObservation.validAxisMask|=1U<<i;
        failedAxis=-1;
    }
    // Diagnostic history only: cache a valid collision observation before a
    // missing exchange, then freeze that copy into feedback at Begin/Retry.
    if (m_g76SphereRunning && frozen.request.physicalZManual &&
        m_g76SphereCurrent.session == frozen.request.session) {
        if (m_g76TransportLastValid.session != frozen.request.session) {
            m_g76TransportLastValid = G76LineTransportLastValid{};
            m_g76TransportLastValid.session = frozen.request.session;
        }
        if (m_ncSettleRuntimeCycleValid && m_collisionCurrent.valid &&
            std::isfinite(m_collisionCurrent.voltageV) && m_collisionCurrent.voltageV >= 0.0 &&
            m_collisionCurrent.voltageV <= 300.0) {
            m_g76TransportLastValid.tick = input.tick;
            m_g76TransportLastValid.gapMv = static_cast<std::int32_t>(std::llround(m_collisionCurrent.voltageV * 1000.0));
            m_g76TransportLastValid.shortActive = m_collisionCurrent.shortActive;
            m_g76TransportLastValid.valid = true;
        }
    }
    const bool quarantined = m_g76Dropout.active;
    if(source&&!quarantined&&m_g76SphereRunning&&frozen.request.physicalZManual&&
       !m_g76SpherePhysicalSource.Observe(m_g76SphereAnalogInput,m_g76SphereAnalogRead,
           input.nowUs,m_edmZClockValid,m_edmZClockFrequency)){
        source=false;gate=MotionG76SphereInputGate::InputSourceInvalid;
    }
    input.sourceValid=source;
    SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Observe,gate,failedAxis);
    // A completed parent may retain its HOLD/START high-water marks while NC
    // publishes the next one. Only the exact sampled parent owns those marks.
    if(m_g76SphereCurrent.session==frozen.request.session&&
       SphereIdentity(m_g76SphereCurrent.identity,frozen.command.execution)&&
       m_g76SphereCurrent.owner.Matches(frozen.command.ownerLease)){
        input.holdSequence=m_g76SphereCurrent.holdSequence;input.startSequence=m_g76SphereCurrent.startSequence;
    }
    if (m_g76Dropout.active) {
        // Only the current call's proven all-NoData deadline may retry. TX
        // failures, malformed/foreign data and all other invalid causes stop.
        if (cleanReceiveDeadline && !m_ncSettleRuntimeCycleValid &&
            gate == MotionG76SphereInputGate::PdoCycleInvalid && RetryG76LineTransportGap()) return;
        if (!source || !confirmedCombinedReturn || !QualifyG76LineTransportGap()) {
            SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Observe,
                gate == MotionG76SphereInputGate::None ? MotionG76SphereInputGate::InputSourceInvalid : gate, failedAxis);
            FailG76SphereRuntime(1U,true);
            return;
        }
        if (m_g76Dropout.active) return;
    } else if (m_g76SphereRunning && cleanReceiveDeadline && !m_ncSettleRuntimeCycleValid &&
        gate == MotionG76SphereInputGate::PdoCycleInvalid && BeginG76LineTransportGap(previousTick, previousUs)) {
        return;
    }
    if(m_g76SphereRunning){
        const auto& output=m_g76SphereCycle.Command();
        input.zeroSendTick=m_g76SphereZeroSentTick;
        input.zeroFinalSend=m_g76SphereZeroDemand&&m_g76SphereZeroSentTick&&m_g76SphereZeroSentTick+1ULL==input.tick;
        bool settled=source&&input.zeroFinalSend&&output.stopped&&ArePhysicalGroupAxesReadyForTerminal();
        const bool same=m_g76SpherePhysicalRange&&m_g76SpherePhysicalTick+1ULL==input.tick&&
            m_g76SpherePhysicalReference==output.commandMCS;
        for(unsigned i=0U;settled&&i<SphereAxisCount(frozen.request);++i){
            if(!SphereAxisPresent(frozen.request,i))continue;
            const auto& axis=(*m_pContexts)[i];const double ppu=frozen.pulsePerNative[i];
            const double window=(std::min)(axis.inPositionWindow_Pulse/ppu,
                SphereRotary(frozen.request,i)?frozen.request.rotaryToleranceDeg:frozen.request.positionToleranceMm);
            const double low=same?(std::min)(m_g76SpherePhysicalMinimum[i],input.actualMCS[i]):input.actualMCS[i];
            const double high=same?(std::max)(m_g76SpherePhysicalMaximum[i],input.actualMCS[i]):input.actualMCS[i];
            const double range=(std::min)(0.5*axis.inPositionWindow_Pulse,
                (std::max)(4.0,0.25*axis.inPositionWindow_Pulse))/ppu;
            settled=SpherePositive(window)&&std::fabs(input.actualMCS[i]-output.commandMCS[i])<=window&&high-low<=range;
            m_g76SpherePhysicalMinimum[i]=low;m_g76SpherePhysicalMaximum[i]=high;
        }
        input.physicalSettled=settled;m_g76SpherePhysicalRange=settled;
        m_g76SpherePhysicalTick=input.tick;
        if(settled)m_g76SpherePhysicalReference=output.commandMCS;
        // Invalid PDO callbacks may skip interpolation. Contain here, before
        // a stale nonzero CSV image can survive the missing motion pass.
        if(!source)FailG76SphereRuntime(1U,true);
    }
}

bool MotionCore::LoadG76SphereCommand(const MotionCommand& front) noexcept
{
    if(m_Group.isActive||m_g76SphereRunning||!m_g76SphereRequests.Read(m_g76SphereFrozen).Ready())return false;
    if(!MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
        m_g76SphereFrozen.request.session,front.execution))return false;
    if(m_g76SphereAdmissionSession!=m_g76SphereFrozen.request.session){
        m_g76SphereAdmissionSession=m_g76SphereFrozen.request.session;m_g76SphereAdmissionTick=m_ncSettleRuntimeCycleTick;
    }
    const auto reject=[&](std::uint32_t code) noexcept {
        MotionCommand popped{};
        if(HasPendingExecutionEpochChange()||!m_Group.cmdQueue.ConsumerTryPop(popped))return false;
        m_g76SphereCurrent=MotionG76SphereFeedback{};m_g76SphereCurrent.session=m_g76SphereFrozen.request.session;
        m_g76SphereCurrent.identity=popped.execution;m_g76SphereCurrent.owner=popped.ownerLease;
        m_g76SphereCurrent.accepted=true;m_g76SphereCurrent.terminal=true;m_g76SphereCurrent.runtimeFault=code;
        if(code==1U)CaptureG76SphereInputFault();
        m_g76SphereCycle.Reset();RejectMotionCommand(popped,MotionRejectReason::NOT_READY,SphereRuntimeAlarm);
        PublishG76SphereRuntime();AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,popped.sourceLinePC);
        RequestEmergencyStopAllAxes();return false;
    };
    if(GetCommandAuthorizationFailure(front)!=MotionRejectReason::NONE)return reject(2U);
    if(!m_g76SphereInput.sourceValid||!m_g76SphereInput.pdoValid||!m_g76SphereInput.powerInhibited||
        !m_g76SphereInput.tick||!m_g76SphereInput.nowUs||m_g76SphereInput.cancel||
        m_g76SphereInput.tick!=m_ncSettleRuntimeCycleTick||
        m_g76SphereInput.physicalSequence!=m_g76SphereInput.tick||
        m_ncSettleRuntimeCycleTick<m_g76SphereAdmissionTick){
        auto gate=MotionG76SphereInputGate::InputSourceInvalid;
        if(!m_g76SphereInput.sourceValid){
            // Preserve the exact observer subgate instead of replacing it
            // with the loader's coarser source-invalid predicate.
            if(m_g76SphereInputObservation.valid&&m_g76SphereInputObservation.gate!=MotionG76SphereInputGate::None)
                return reject(1U);
        }else if(!m_g76SphereInput.pdoValid)gate=MotionG76SphereInputGate::InputPdoInvalid;
        else if(!m_g76SphereInput.powerInhibited)gate=MotionG76SphereInputGate::InputPowerNotInhibited;
        else if(!m_g76SphereInput.tick)gate=MotionG76SphereInputGate::AdmissionTickMissing;
        else if(!m_g76SphereInput.nowUs)gate=MotionG76SphereInputGate::AdmissionTimeMissing;
        else if(m_g76SphereInput.cancel)gate=MotionG76SphereInputGate::AdmissionCancelled;
        else if(m_g76SphereInput.tick!=m_ncSettleRuntimeCycleTick)gate=MotionG76SphereInputGate::InputTickMismatch;
        else if(m_g76SphereInput.physicalSequence!=m_g76SphereInput.tick)gate=MotionG76SphereInputGate::AdmissionPhysicalSequenceMismatch;
        else gate=MotionG76SphereInputGate::AdmissionTickBackwards;
        SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Admission,gate);return reject(1U);
    }
    const double waited=static_cast<double>(m_ncSettleRuntimeCycleTick-m_g76SphereAdmissionTick)*CYCLE_TIME_SEC;
    // EDM287 FIX1: the generic incoming-axis window can be wider than P50's
    // unchanged start tolerance. Do not pop/accept the parent while a valid
    // same-cycle physical sample is still converging to that frozen start.
    // Invalid sources and actual travel violations are never settle waits.
    const auto& request=m_g76SphereFrozen.request;
    bool atStart=true;
    for(unsigned i=0U;i<8U;++i){
        const double actual=m_g76SphereInput.actualMCS[i];
        const double error=std::fabs(actual-request.geometry.startMCS[i]);
        if(!std::isfinite(actual)||!std::isfinite(error)){
            SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Admission,
                MotionG76SphereInputGate::AdmissionActualNonfinite,static_cast<int>(i));return reject(1U);
        }
        if((request.travelMask&(1U<<i))&&(actual<request.travelMin[i]||actual>request.travelMax[i]))
            return reject(8U);
        if(error>(SphereRotary(request,i)?request.rotaryToleranceDeg:request.positionToleranceMm))atStart=false;
    }
    G76SphereParentBinding parent{};
    if(m_g76SphereParentBound.load(std::memory_order_acquire)!=m_g76SphereFrozen.request.session||
        !m_g76SphereParentBinding.Read(parent).Ready()||parent.session!=m_g76SphereFrozen.request.session||
        !parent.dispatch||!parent.commit||!SphereIdentity(parent.identity,front.execution)||!parent.owner.Matches(front.ownerLease))
        return waited>=5.0?reject(5U):false;
    for(int slot=0;slot<front.axisCount;++slot){
        const auto& axis=(*m_pContexts)[front.axisIndices[slot]];
        if(axis.logicalCmdPos.Load()!=front.mem_startPos[slot]||axis.homeRuntime.active||axis.isFault||axis.isLagAlarm)return reject(2U);
        if(!IsG80IncomingReady(axis)||!axis.IsMechanicalCompensationReadyForCompletion())
            return waited>=m_g76SphereFrozen.request.settleTimeoutSeconds?reject(4U):false;
    }
    if(!atStart)return waited>=request.settleTimeoutSeconds?reject(7U):false;
    LifecycleCommitReservationGuard loadReservation(*this,front.execution);
    if(!loadReservation.IsAcquired())return false;
    if(!SpherePowerOff()||GetCommandAuthorizationFailure(front)!=MotionRejectReason::NONE){
        loadReservation.Release();return reject(2U);
    }
    MotionCommand command{};
    if(HasPendingExecutionEpochChange()||!m_Group.cmdQueue.ConsumerTryPop(command))return false;
    if(!SphereIdentity(command.execution,front.execution)||GetCommandAuthorizationFailure(command)!=MotionRejectReason::NONE){
        loadReservation.Release();RejectMotionCommand(command,MotionRejectReason::NOT_READY,SphereRuntimeAlarm);return false;
    }
    // EDM307: the NC endpoint/drain release creates one small continuation
    // token. Read the incoming process page into persistent RT storage before
    // replacing the previous terminal observer or its immutable config.
    const bool processRead=m_g76SphereProcessRequests.Read(m_g76SphereIncomingProcessFrozen).Ready();
    const auto& incoming=m_g76SphereIncomingProcessFrozen;
    const bool holdRead=m_g76SphereHoldRequests.Read(m_g76SphereIncomingHoldFrozen).Ready();
    const auto& incomingHold=m_g76SphereIncomingHoldFrozen;
    const auto& previous=m_g76SphereCurrent;const auto& previousProcess=m_g76SphereProcess.Current();
    const auto& previousGeometry=m_g76SphereCycle.Geometry();
    const auto& nextRequest=m_g76SphereFrozen.request;
    // HOLD history has its own success/release proof. E5 may reset across P50
    // or a G181 declaration, whereas these are still executed machining parents.
    const auto holdRelease=m_g76SphereHoldContinuationReleased.load(std::memory_order_acquire);
    bool keepHoldHistory=holdRead&&incomingHold.enabled&&m_g76SphereHoldFrozen.enabled&&
        !IsResetSafetyOutputHoldEstablished()&&previous.session&&holdRelease==previous.session&&
        previous.session<nextRequest.session&&previous.completed&&previous.terminal&&!previous.runtimeFault&&!previous.cycle.timedCompletion&&
        previous.cycle.phase==NCG76SphereCycle::Phase::Complete&&previous.cycle.fault==NCG76SphereCycle::Fault::None&&
        !previous.cycle.holdActive&&!previous.cycle.held&&previous.identity.source==MotionCommandSource::NC_MEMORY&&
        previous.identity.epoch==command.execution.epoch&&previous.owner.Matches(command.ownerLease)&&
        m_g76SphereHoldReleaseFeedbackSequence.load(std::memory_order_acquire)==
            m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire)&&
        m_g76SphereCycle.Current().session==previous.session&&m_g76SphereCycle.Command().complete&&
        previousGeometry.valid&&previousGeometry.kind==previous.nominalKind&&
        SameNCTranslationSnapshot(m_g76SphereHoldHistoryFrame,nextRequest.geometry.frame.translation)&&
        SameNCTranslationSnapshot(previousGeometry.frame.translation,nextRequest.geometry.frame.translation)&&
        std::memcmp(previousGeometry.endMCS.data(),previous.endpointMCS.data(),sizeof(previous.endpointMCS))==0&&
        std::memcmp(previous.endpointMCS.data(),nextRequest.geometry.startMCS.data(),sizeof(previous.endpointMCS))==0;
    // The journal uses the same installed mapping/configuration as the current
    // parent; no old lead, feedback source, home or limit setting is restored.
    if(keepHoldHistory)for(unsigned i=0U;i<SphereAxisCount(nextRequest);++i){
        if(!SphereAxisPresent(nextRequest,i))continue;
        const auto& a=m_g76SphereHoldHistoryBindings[i];const auto& b=m_g76SphereFrozen.axis[i];
        if(a.resolution!=b.resolution||a.lead!=b.lead||a.maxVelocity!=b.maxVelocity||
           a.accSeconds!=b.accSeconds||a.decSeconds!=b.decSeconds||a.stopSeconds!=b.stopSeconds||
           a.machineOffset!=b.machineOffset||a.scaleRatio!=b.scaleRatio||a.maxDeviation!=b.maxDeviation||
           a.followingWindow!=b.followingWindow||!MotionG76SphereFixedAxes::SameBits(a.rotaryModulo,b.rotaryModulo)||
           a.scalePointer!=b.scalePointer||a.feedbackMode!=b.feedbackMode||a.reverseFlags!=b.reverseFlags||
           a.axisType!=b.axisType||a.configurationFlags!=b.configurationFlags||
           a.travelLimits!=b.travelLimits||a.compensationParameters!=b.compensationParameters){
            keepHoldHistory=false;break;
        }
    }
    if(!keepHoldHistory||m_g76SphereHoldHistoryCount>NCHoldPolicy::MaximumPreviousParents)
        m_g76SphereHoldHistoryCount=0U;
    else{
        if(m_g76SphereHoldHistoryCount==NCHoldPolicy::MaximumPreviousParents){
            for(unsigned i=1U;i<NCHoldPolicy::MaximumPreviousParents;++i)
                m_g76SphereHoldHistory[i-1U]=m_g76SphereHoldHistory[i];
            --m_g76SphereHoldHistoryCount;
        }
        m_g76SphereHoldHistory[m_g76SphereHoldHistoryCount++]=previousGeometry;
    }
    // Recheck every borrowed physical envelope against CURRENT travel guards.
    // An incompatible older path is a history boundary, never a widened limit.
    bool historyWithinTravel=true;
    for(unsigned row=0U;historyWithinTravel&&row<m_g76SphereHoldHistoryCount;++row){
        const auto& g=m_g76SphereHoldHistory[row];
        for(unsigned axis=0U;axis<8U;++axis){
            const double low=g.boundsMinMCS[axis],high=g.boundsMaxMCS[axis];
            if(!std::isfinite(low)||!std::isfinite(high)||low>high||
               ((nextRequest.travelMask&(1U<<axis))&&(low<nextRequest.travelMin[axis]||high>nextRequest.travelMax[axis]))||
               (SphereAxisPresent(nextRequest,axis)&&(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],low)||
                          !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],high)))){
                historyWithinTravel=false;break;
            }
        }
    }
    if(!historyWithinTravel)m_g76SphereHoldHistoryCount=0U;
    m_g76SphereHoldHistoryFrame=nextRequest.geometry.frame.translation;
    m_g76SphereHoldHistoryBindings=m_g76SphereFrozen.axis;
    auto staleHoldRelease=holdRelease;
    if(staleHoldRelease)(void)m_g76SphereHoldContinuationReleased.compare_exchange_strong(staleHoldRelease,0ULL,
        std::memory_order_acq_rel,std::memory_order_acquire);
    NCG76SphereGapProcess::Continuation continuation{};bool continueProcess=false;
    const auto nextKind=m_g76SphereFrozen.request.geometry.kind;
    const bool previousSingle=previous.nominalKind==NCG76SphereRuntimeGeometry::Kind::Line||
        previous.nominalKind==NCG76SphereRuntimeGeometry::Kind::Arc;
    const bool nextSingle=nextKind==NCG76SphereRuntimeGeometry::Kind::Line||nextKind==NCG76SphereRuntimeGeometry::Kind::Arc;
    // The birth page stays immutable. Only an RT-accepted and fully applied
    // E5 sidecar can align the preceding parent's continuation comparison.
    const bool tuningApplied=!m_g76SphereCycle.HasPendingFlushFeeds()&&
        previous.liveCurveTuningGeneration==m_g76SphereLiveTuningGeneration&&
        previous.liveFlushTuningGeneration==m_g76SphereLiveTuningGeneration;
    m_g76SphereContinuationProcessConfig=tuningApplied&&
        m_g76SphereLiveTuningGeneration>m_g76SphereProcessFrozen.config.tuningGeneration?
        m_g76SphereLiveTuningConfig:m_g76SphereProcessFrozen.config;
    const bool fullRecipe=m_g76SphereLiveRecipeSequence!=0ULL;
    const bool fullRecipeApplied=fullRecipe&&
        previous.liveRecipeSequence==m_g76SphereLiveRecipeSequence&&
        previous.liveFlushRecipeSequence==m_g76SphereLiveRecipeSequence&&
        previous.liveOrbitRecipeSequence==m_g76SphereLiveRecipeSequence&&
        !m_g76SphereLiveRecipeFlushDeferred&&!m_g76SphereCycle.HasPendingLiveFlushConfig()&&
        !m_g76SphereProcess.HasPendingLiveRecipeFlush()&&
        !m_g76SphereProcess.HasPendingWorkTimeUpdate()&&
        previousProcess.workTimeUs==m_g76SphereLiveRecipeConfig.workTimeUs;
    if(fullRecipeApplied)m_g76SphereContinuationProcessConfig=m_g76SphereLiveRecipeConfig;
    const bool workTimeApplied=fullRecipe?fullRecipeApplied:
        tuningApplied&&!m_g76SphereProcess.HasPendingWorkTimeUpdate()&&
        previousProcess.workTimeUs==m_g76SphereRequestedWorkTimeUs;
    if(!fullRecipe&&workTimeApplied&&m_g76SphereWorkTimeSequence){
        m_g76SphereContinuationProcessConfig.workTimeUs=m_g76SphereRequestedWorkTimeUs;
        m_g76SphereContinuationProcessConfig.recipeGeneration=m_g76SphereWorkTimeRecipeGeneration;
    }
    const bool continuationScope=workTimeApplied&&!IsResetSafetyOutputHoldEstablished()&&previousSingle&&nextSingle&&previous.session&&
        previous.session<m_g76SphereFrozen.request.session&&previous.completed&&previous.terminal&&!previous.cycle.timedCompletion&&
        !previous.runtimeFault&&previous.cycle.phase==NCG76SphereCycle::Phase::Complete&&
        previous.cycle.fault==NCG76SphereCycle::Fault::None&&!previous.cycle.holdActive&&
        !previous.cycle.held&&previous.identity.source==MotionCommandSource::NC_MEMORY&&
        previous.identity.epoch==command.execution.epoch&&previous.owner.Matches(command.ownerLease)&&
        previous.identity.sourceBlockId>=0&&previous.identity.sourceBlockId<(std::numeric_limits<int>::max)()&&
        previous.identity.sourceBlockId+1==command.sourceLinePC&&
        previous.translationGeneration==m_g76SphereFrozen.request.geometry.frame.translation.generation&&
        std::memcmp(previous.endpointMCS.data(),m_g76SphereFrozen.request.geometry.startMCS.data(),sizeof(previous.endpointMCS))==0&&
        previousProcess.enabled&&previousProcess.terminal&&previousProcess.reason==NCG76SphereGapProcess::Reason::Terminal&&
        previousProcess.session==previous.session&&m_g76SphereProcessFrozen.session==previous.session&&
        SphereIdentity(m_g76SphereProcessFrozen.identity,previous.identity)&&m_g76SphereProcessFrozen.owner.Matches(previous.owner)&&
        m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)!=previous.session&&
        processRead&&incoming.session==m_g76SphereFrozen.request.session&&SphereIdentity(incoming.identity,command.execution)&&
        incoming.owner.Matches(command.ownerLease)&&NCG76SphereGapProcess::Valid(incoming.config)&&
        NCG76SphereGapProcess::EquivalentForContinuation(m_g76SphereContinuationProcessConfig,incoming.config);
    if(continuationScope){
        continuation.intervalWorkUs=previousProcess.intervalWorkUs;
        continuation.simulatedWorkUs=previousProcess.simulatedWorkUs;
        continuation.completedFlushes=previousProcess.completedFlushes;
        continuation.abortedFlushes=previousProcess.abortedFlushes;
        continuation.e5Evidence=previousProcess.e5Evidence;
        continuation.e5StartUs=m_g76SphereProcess.E5StartUs();
        continuation.e5ObservedUs=m_g76SphereProcess.E5ObservedUs();
        if(NCG76SphereGapProcess::ValidContinuation(incoming.config,continuation)){
            auto released=previous.session;
            continueProcess=m_g76SphereContinuationReleased.compare_exchange_strong(released,0ULL,
                std::memory_order_acq_rel,std::memory_order_acquire);
        }
    }
    // A consumed/rejected predecessor page must never be reconsidered by a
    // later source after a mode/configuration or control-flow boundary.
    auto oldRelease=previous.session;
    if(oldRelease)(void)m_g76SphereContinuationReleased.compare_exchange_strong(oldRelease,0ULL,
        std::memory_order_acq_rel,std::memory_order_acquire);
    // EDM309: only the just-consumed NC endpoint/drain receipt can lend ONE
    // completed analytic predecessor to B2. Scope changes never resurrect it.
    bool previousParentReady=continueProcess&&nextRequest.retreatMode==2U&&
        m_g76SphereCycle.Current().session==previous.session&&m_g76SphereCycle.Command().complete&&
        previousGeometry.valid&&previousGeometry.travelMm>0.0&&nextRequest.geometry.travelMm>0.0&&
        previousGeometry.kind==previous.nominalKind&&
        SameNCTranslationSnapshot(previousGeometry.frame.translation,nextRequest.geometry.frame.translation)&&
        std::memcmp(previousGeometry.endMCS.data(),nextRequest.geometry.startMCS.data(),sizeof(previousGeometry.endMCS))==0;
    // Recheck the retained envelope against CURRENT installed travel bounds.
    // The new frozen request already contains their intersection; no old
    // geometry can widen those limits or reuse an old axis binding.
    if(previousParentReady)for(unsigned axis=0U;axis<8U;++axis){
        const double low=previousGeometry.boundsMinMCS[axis],high=previousGeometry.boundsMaxMCS[axis];
        if(!std::isfinite(low)||!std::isfinite(high)||low>high||
           ((nextRequest.travelMask&(1U<<axis))&&(low<nextRequest.travelMin[axis]||high>nextRequest.travelMax[axis]))||
           (SphereAxisPresent(nextRequest,axis)&&(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],low)||
                      !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],high)))){
            previousParentReady=false;break;
        }
    }
    const std::uint64_t previousParentSession=previousParentReady?previous.session:0ULL;
    const std::int32_t previousParentPC=previousParentReady?previous.identity.sourceBlockId:-1;
    if(previousParentReady)m_g76SpherePreviousGeometry=previousGeometry;
    m_g76SphereCurrent=MotionG76SphereFeedback{};m_g76SphereCurrent.session=m_g76SphereFrozen.request.session;
    m_g76TransportLastValid=G76LineTransportLastValid{};
    m_g76SphereCurrent.identity=command.execution;m_g76SphereCurrent.owner=command.ownerLease;m_g76SphereCurrent.accepted=true;
    m_g76SphereCurrent.initialIntervalWorkUs=continueProcess?continuation.intervalWorkUs:0ULL;
    m_g76SphereCurrent.completedFlushOffset=continueProcess?continuation.completedFlushes:0ULL;
    m_g76SphereCurrent.previousParentSession=previousParentSession;
    m_g76SphereCurrent.previousParentPC=previousParentPC;
    TrackMotionCommandAccepted(command);m_g76SphereCycle.Reset();
    m_g76SphereProcessFrozen=processRead?incoming:G76SphereProcessFrozen{};
    m_g76SphereWorkTimeSequence=0ULL;
    m_g76SphereWorkTimeRecipeGeneration=m_g76SphereProcessFrozen.config.recipeGeneration;
    m_g76SphereRequestedWorkTimeUs=m_g76SphereProcessFrozen.config.workTimeUs;
    m_g76SphereLiveTuningGeneration=m_g76SphereProcessFrozen.config.tuningGeneration;
    m_g76SphereLiveTuningPublication=0ULL;
    m_g76SphereLiveTuningConfig=m_g76SphereProcessFrozen.config;
    m_g76SphereLiveRecipeConfig=m_g76SphereProcessFrozen.config;
    m_g76SphereLiveRecipeSequence=m_g76SphereLiveRecipePublication=0ULL;
    m_g76SphereLiveRecipeFlushDeferred=false;
    m_g76SphereLiveCompletionRequest=m_g76SphereFrozen.request;
    m_g76SphereLiveRecipeThreshold=m_g76SphereFrozen.request.orbitThresholdScaled;
    m_g76SphereLiveRecipeRetry=m_g76SphereFrozen.request.orbitRetryLimit;
    m_g76SphereCurrent.liveCurveTuningGeneration=m_g76SphereLiveTuningGeneration;
    m_g76SphereCurrent.liveFlushTuningGeneration=m_g76SphereLiveTuningGeneration;
    m_g76SphereHoldFrozen=holdRead?incomingHold:G76SphereHoldFrozen{};
    m_g76SphereInspectionConsumedSequence=0ULL;
    m_g76SphereInspectionLeaseSeen=m_g76SphereInspectionDeadlineUs=m_g76SphereInspectionLeaseHold=0ULL;
    m_g76SphereInspectionLeaseAxis=-1;m_g76SphereInspectionLeaseDirection=0;
    m_g76SphereInspectionBlocked=false;
    const auto& process=m_g76SphereProcessFrozen;
    const bool processScope=processRead&&process.session==m_g76SphereFrozen.request.session&&
        SphereIdentity(process.identity,command.execution)&&process.owner.Matches(command.ownerLease)&&
        NCG76SphereGapProcess::Valid(process.config)&&
        m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)!=process.session;
    // EDM310 explicit SIM test build. Selecting a voltage waveform cannot
    // lend a predecessor or bypass the consumed NC endpoint release above.
    const auto simulationMode=NCG76SphereSimulationTestPolicy::PreviousParentProbeEnabled&&
        previousParentReady&&processScope&&process.config.parameterFeeds?
        NCG76SphereGapProcess::SimulationMode::PreviousParentProbe:NCG76SphereGapProcess::SimulationMode::Wave;
    m_g76SpherePhysicalSource.Clear();
    const bool manualPhysicalReady=!request.physicalZManual||(processScope&&process.config.physicalZManual&&
        m_g76SpherePhysicalProfileConfigured&&m_g76SphereGainBanks.Configured()&&
        m_g76SphereGainBanks.ConfiguredMask()==SphereAxisMask(request)&&m_edmZClockValid&&
        m_g76SpherePhysicalSource.Begin(m_g76SpherePhysicalProfile,m_g76SphereAnalogInput,
            m_g76SphereAnalogRead,request.session,m_edmZMonotonicUs,m_edmZClockFrequency));
    const bool processReady=manualPhysicalReady&&m_g76SphereProcess.Begin(processScope?process.config:NCG76SphereGapProcess::Config{},
        m_g76SphereFrozen.request.session,continueProcess?&continuation:nullptr,simulationMode,
        (NCG76OrbitContour::IsFixedRadiusContour(m_g76SphereFrozen.request.geometry)||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==16U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==17U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==21U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==22U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==23U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==24U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==25U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==26U||
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==27U)?&m_g76SphereFrozen.request.geometry:nullptr,
        request.physicalZManual?&m_g76SpherePhysicalSource.Config():nullptr);
    NCG76SphereCycle::FlushConfig flush{};
    if(!holdRead||m_g76SphereHoldFrozen.session!=m_g76SphereFrozen.request.session||
       !SphereIdentity(m_g76SphereHoldFrozen.identity,command.execution)||
       !m_g76SphereHoldFrozen.owner.Matches(command.ownerLease)||
       m_g76SphereHoldFrozen.enabled!=m_g76SphereFrozen.holdRecoveryEnabled||
       m_g76SphereHoldFrozen.declaredPreviousParents!=m_g76SphereFrozen.holdDeclaredPreviousParents||
       m_g76SphereHoldFrozen.finishRequiredCount!=m_g76SphereFrozen.finishRequiredCount||
       m_g76SphereHoldFrozen.finishRequiredCount>99U||
       m_g76SphereHoldFrozen.orbitInspectionRounds!=m_g76SphereFrozen.request.orbitInspectionRounds||
       m_g76SphereHoldFrozen.orbitThresholdScaled!=m_g76SphereFrozen.request.orbitThresholdScaled||
       m_g76SphereHoldFrozen.orbitRetryLimit!=m_g76SphereFrozen.request.orbitRetryLimit||
       !SphereOrbitCompletionConfigValid(m_g76SphereHoldFrozen.orbitInspectionRounds,m_g76SphereHoldFrozen.orbitThresholdScaled,
            m_g76SphereHoldFrozen.orbitRetryLimit)||
       (m_g76SphereFrozen.request.orbitInspectionRounds&&
        (!processScope||!processReady||!process.config.enabled||!process.config.signedMotionEnabled||
         !process.config.orbitFeedsEnabled||m_g76SphereHoldFrozen.finishRequiredCount!=0U))||
       !MotionG76SphereMachiningTime::Matches(m_g76SphereHoldFrozen.machiningTime,m_g76SphereFrozen.request)||
       (m_g76SphereHoldFrozen.enabled&&!NCHoldRecovery::Valid(m_g76SphereHoldFrozen.config))||
       process.config.orbitFeedsEnabled!=m_g76SphereFrozen.orbitFeedsEnabled||
       !MotionG76SphereParameterFeeds::Matches(m_g76SphereFrozen.request,process.config)||
       (m_g76SphereFrozen.signedMotionEnabled&&
        (!processScope||!processReady||!process.config.signedMotionEnabled))||
       (processScope&&m_g76SphereFrozen.flushFeatureFlags!=
        static_cast<std::uint8_t>((process.config.independentFeedsEnabled?1U:0U)|
            (process.config.finalApproachEnabled?2U:0U)|(process.config.initialSlowEnabled?4U:0U)|
            (process.config.deepFlushEnabled?8U:0U)))||
       (m_g76SphereFrozen.request.retreatMode!=0U&&
        (!m_g76SphereFrozen.signedMotionEnabled||!processScope||!processReady||!process.config.enabled))||
       (processScope&&processReady&&
        !NCG76SphereGapProcess::BuildFlushConfig(process.config,m_g76SphereFrozen.request.feedMmMin,flush))||
       (m_g76SphereHoldFrozen.finishRequiredCount!=0U&&
        (m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere||
         m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour||
         !std::isfinite(m_g76SphereFrozen.request.geometry.travelMm)||m_g76SphereFrozen.request.geometry.travelMm<=0.0||
         !processScope||!processReady||!process.config.enabled||!process.config.signedMotionEnabled||
         !process.config.flushMotionEnabled||!flush.enabled))){
        m_g76SphereProcess.Revoke();m_g76SphereCurrent.runtimeFault=9U;m_g76SphereCurrent.terminal=true;
        loadReservation.Release();FaultTrackedMotionCommand(SphereRuntimeAlarm,MotionRejectReason::NOT_READY);PublishG76SphereRuntime();
        AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,command.sourceLinePC);RequestEmergencyStopAllAxes();return false;
    }
    // EDM306 FIX1: this loader runs inside the EtherCAT callback. Keep stack
    // queries, CRT formatting and RtPrintf out of it; their variable cost can
    // disturb the next measured cycle interval. NC admission still records
    // memory observations. The clock and fresh-source guards remain strict.
    NCG76SphereCycle::GapConfig gap{};gap.enabled=m_g76SphereFrozen.signedMotionEnabled;
    // P50 keeps its existing internal-leg history. EDM309 adds one released
    // primitive predecessor for signed B2 GAP only; E5 flush remains local.
    gap.crossLegRetreatEnabled=gap.enabled&&
        (m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere||
         m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour||previousParentReady);
    if(previousParentReady)gap.previousPrimitive=&m_g76SpherePreviousGeometry;
    if(continueProcess)gap.completedFlushOffset=continuation.completedFlushes;
    if(!m_g76SphereCycle.Begin(m_g76SphereFrozen.request,m_g76SphereInput,(flush.enabled||flush.axisLimitsEnabled)?&flush:nullptr,&gap)||
       (m_g76SphereHoldFrozen.enabled&&!m_g76SphereCycle.ConfigureHoldRecoveryAdmitted(m_g76SphereHoldFrozen.config,
            m_g76SphereHoldFrozen.declaredPreviousParents,m_g76SphereHoldHistory.data(),m_g76SphereHoldHistoryCount))||
       !m_g76SphereCycle.ConfigureFinishVerification(m_g76SphereHoldFrozen.finishRequiredCount)){
        m_g76SphereProcess.Revoke();
        m_g76SphereCurrent.runtimeFault=3U;m_g76SphereCurrent.terminal=true;
        loadReservation.Release();FaultTrackedMotionCommand(SphereRuntimeAlarm,MotionRejectReason::NOT_READY);PublishG76SphereRuntime();
        AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,command.sourceLinePC);RequestEmergencyStopAllAxes();return false;
    }
    m_g76Scope=G76ScopeEvidence{};
    m_g76Scope.session=m_g76SphereFrozen.request.session;
    // Inspection has a stricter installed XYZ box independent of G22/G23.
    // Do not tighten the machining or nominal-history envelope under G23.
    m_g76SphereInspectionTravelMask=0U;
    if(m_g76SphereHoldFrozen.enabled){
        for(unsigned axis=0U;axis<3U;++axis){
            const auto& binding=m_g76SphereFrozen.axis[axis];
            bool bounded=false,valid=(binding.configurationFlags&1U)!=0U;
            double low=-(std::numeric_limits<double>::max)(),high=(std::numeric_limits<double>::max)();
            for(unsigned limit=0U;valid&&limit<3U;++limit){
                if((binding.configurationFlags&(2U<<limit))==0U)continue;
                const double lo=binding.travelLimits[2U*limit],hi=binding.travelLimits[2U*limit+1U];
                if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi){valid=false;break;}
                low=(std::max)(low,lo);high=(std::min)(high,hi);bounded=true;
            }
            if(valid&&bounded&&low<high){
                m_g76SphereInspectionMinimum[axis]=low;m_g76SphereInspectionMaximum[axis]=high;
                m_g76SphereInspectionTravelMask|=1U<<axis;
            }
        }
        if(m_g76SphereInspectionTravelMask!=7U||
           !m_g76SphereCycle.ConfigureInspectionTravel(m_g76SphereInspectionMinimum,
                m_g76SphereInspectionMaximum,m_g76SphereInspectionTravelMask))m_g76SphereInspectionTravelMask=0U;
    }
    m_g76Dropout=G76LineTransportGap{};
    m_g76SphereFinalGuard=G76SphereFinalGuard{};
    m_g76SphereFinalGuard.session=m_g76SphereFrozen.request.session;
    m_g76SphereProcessSentTick=m_g76SphereProcessSendSequence=m_g76SphereProcessSendFrame=0ULL;
    m_g76SphereZeroDemand=false;m_g76SphereZeroSentTick=0ULL;m_g76SpherePhysicalRange=false;m_g76SpherePhysicalTick=0ULL;
    m_Group.currentCmd=command;m_Group.isActive=true;m_Group.mode=InterpolationMode::LINEAR;
    m_Group.pathMode=PathMode::EXACT_STOP;m_Group.enableHistory=false;m_Group.enableTransform=false;
    m_Group.axisCount=command.axisCount;
    m_Group.currentExecutionPC=command.sourceLinePC;m_Group.currentExecutionWCS=command.sourceWCS;
    m_Group.currentExecutionToolMode=command.sourceToolLengthMode;m_Group.currentExecutionHCode=command.sourceHCode;
    m_Group.currentExecutionToolRadiusMode=command.sourceToolRadiusMode;m_Group.currentExecutionDCode=command.sourceDCode;
    m_Group.currentExecutionIsAbsoluteMode=command.sourceIsAbsoluteMode;
    m_Group.currentExecutionG68Active=command.sourceG68Active;m_Group.currentExecutionG68Angle=command.sourceG68Angle;
    m_Group.currentExecutionG168Active=command.sourceG168Active;m_Group.currentExecutionWCode=command.sourceWCode;
    m_Group.currentExecutionG51Active=command.sourceG51Active;m_Group.currentExecutionScaleRatio=command.sourceScaleRatio;
    m_Group.currentExecutionMirrorMask=command.sourceMirrorMask;m_Group.currentExecutionG16Active=command.sourceG16Active;
    m_Group.currentExecutionG162Active=command.sourceG162Active;m_Group.currentExecutionPlaneMode=command.sourcePlaneMode;
    for(int slot=0;slot<command.axisCount;++slot){
        const int i=command.axisIndices[slot];m_Group.axisIndices[slot]=i;auto& axis=(*m_pContexts)[i];
        axis.state=MotionState::MotionState_INTERPOLATING;axis.inPosition=false;
        axis.currentCmdVel=axis.logicalCmdVel=axis.targetVelocity=axis.targetEndVel=0.0;
        // Existing interpolation PID selection remains the sole gain owner.
        axis.pid.integralAcc=axis.pid.prevError=0.0;
    }
    auto& v=m_Group.virtualAxis;
    v.currentCmdPos=v.logicalCmdPos=v.planningPos=v.finalTargetPos=0.0;
    v.currentCmdVel=v.logicalCmdVel=v.targetVelocity=v.targetEndVel=0.0;
    v.state=MotionState::MotionState_INTERPOLATING;v.inPosition=false;
    m_g76SphereRunning=true;m_g76SphereCurrent.started=true;
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    m_g01ResponseDumpSafe.store(false,std::memory_order_release);
    const auto& responseGeometry=m_g76SphereFrozen.request.geometry;
    if(m_g76SphereFrozen.request.physicalZManual&&m_g76SphereFrozen.request.parameterFeeds&&
       responseGeometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
       m_g76SphereFrozen.axisMask==7U&&m_g76SphereFrozen.request.retreatMode==0U&&
       responseGeometry.startMCS[0]==responseGeometry.endMCS[0]&&
       responseGeometry.startMCS[1]==responseGeometry.endMCS[1]&&
       responseGeometry.startMCS[2]!=responseGeometry.endMCS[2])
        (void)m_g01ResponseTrace.Begin(m_g76SphereFrozen.request.session);
    // EDM386_G01_RESPONSE_TRACE_END
    CommitCommandPathModeConsumerAuthority(command,MotionCommandPathModeAuthorityDecision::APPLY_EXACT_STOP);
    TrackMotionCommandStarted(command);loadReservation.Release();PublishG76SphereRuntime();return true;
}

bool MotionCore::RequestG76SphereResetCancellation(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t resetHoldProvenance) noexcept
{
    G76SphereFrozen frozen{};
    if(!session||!identity.IsAssigned()||!m_g76SphereRequests.Read(frozen).Ready()||
       frozen.request.session!=session||!SphereIdentity(frozen.command.execution,identity)||
       !frozen.command.ownerLease.Matches(owner)||m_g76SphereClaim.load(std::memory_order_acquire)!=session||
       !IsMotionOwnerLeaseCurrent(owner)||GetCurrentExecutionEpoch()!=identity.epoch||
       !resetHoldProvenance||GetSafetyProvenanceGeneration()!=resetHoldProvenance) return false;
    G76SphereResetScope scope{};scope.session=session;scope.identity=identity;scope.owner=owner;
    scope.previousProvenance=scope.expectedProvenance=resetHoldProvenance;
    return m_g76SphereResetScope.Publish(scope);
}

void MotionCore::AdvanceG76SphereResetScope(std::uint64_t previousProvenance,std::uint64_t nextProvenance,
    std::uint64_t expectedSafetyOwnerState) noexcept
{
    // The same NC Reset producer announces only its next exact provenance
    // operation and prospective owner/ticket before the original CAS. The
    // fixed snapshot closes partial-token seams; it cannot authorize motion.
    G76SphereResetScope scope{};G76SphereFrozen frozen{};
    if(!previousProvenance||!nextProvenance||!m_g76SphereResetScope.Read(scope).Ready()||
       !m_g76SphereRequests.Read(frozen).Ready()||!scope.session||scope.session!=frozen.request.session||
       !SphereIdentity(scope.identity,frozen.command.execution)||!scope.owner.Matches(frozen.command.ownerLease)||
       scope.expectedProvenance!=previousProvenance) return;
    scope.previousProvenance=previousProvenance;scope.expectedProvenance=nextProvenance;
    if(expectedSafetyOwnerState&&expectedSafetyOwnerState!=scope.expectedSafetyOwnerState){
        scope.previousSafetyOwnerState=scope.expectedSafetyOwnerState;
        scope.expectedSafetyOwnerState=expectedSafetyOwnerState;
    }
    (void)m_g76SphereResetScope.Publish(scope);
}

bool MotionCore::IsG76SphereResetOutputHeld(bool applyingExactBatch) const noexcept
{
    G76SphereResetScope scope{};
    if(!m_g76SphereRunning||!m_g76SphereResetScope.Read(scope).Ready()||!scope.session||
       scope.session!=m_g76SphereFrozen.request.session||m_g76SphereCurrent.session!=scope.session||
       !SphereIdentity(scope.identity,m_g76SphereFrozen.command.execution)||
       !SphereIdentity(m_g76SphereCurrent.identity,scope.identity)||!scope.owner.Matches(m_g76SphereFrozen.command.ownerLease)||
       !IsResetSafetyOutputHoldEstablished()||m_safetyControlledStopInProgress||
       AlarmManager::GetInstance().HasAlarm()||
       m_emergencyStopRequestPublication.load(std::memory_order_acquire)!=0ULL||
       m_resetAllFaultsPending.load(std::memory_order_acquire)||m_stopGroupPending.load(std::memory_order_acquire)||
       m_axisFaultResetPendingMask.load(std::memory_order_acquire)!=0U||
       (m_p1MappingIntegrityAlarmRequestPublication.load(std::memory_order_acquire)&0x8000000000000000ULL)!=0ULL||
       !IsG76SphereCommandAuthorized(m_g76SphereFrozen.command)||m_Group.enableTransform||m_Group.enableHistory||
       m_Group.jumpManager.state!=JumpState::IDLE||m_pathHold.sourceSeen||m_Group.pathMode!=PathMode::EXACT_STOP||
       !std::isfinite(m_Group.feedrateOverride)||m_Group.feedrateOverride<0.0||m_Group.feedrateOverride>1.0||
       m_Group.axisCount!=m_g76SphereFrozen.command.axisCount||m_pContexts==nullptr) return false;
    for(int i=0;i<m_Group.axisCount;++i)
        if(m_Group.axisIndices[i]!=m_g76SphereFrozen.command.axisIndices[i])return false;
    // Epoch abort clears only currentCmd's identity/lease. Every other field
    // must still be the same frozen G76Sphere command, never a replacement group.
    MotionCommand current=m_Group.currentCmd;
    const bool exactActive=SphereIdentity(current.execution,scope.identity)&&current.ownerLease.Matches(scope.owner);
    const bool exactAborted=!m_Group.isActive&&!current.execution.IsAssigned()&&!current.ownerLease.IsValid()&&
        m_feedbackTrackedTerminal&&SphereIdentity(m_feedbackTrackedIdentity,scope.identity)&&
        m_feedbackTrackedOwnerLease.Matches(scope.owner);
    if(!exactActive&&!exactAborted)return false;
    current.execution=m_g76SphereFrozen.command.execution;current.ownerLease=m_g76SphereFrozen.command.ownerLease;
    if(std::memcmp(&current,&m_g76SphereFrozen.command,sizeof(current))!=0)return false;
    const auto provenance=GetSafetyProvenanceGeneration();
    if(provenance!=scope.previousProvenance&&provenance!=scope.expectedProvenance)return false;
    const auto ownerState=m_motionOwnerState.load(std::memory_order_acquire);
    const auto currentOwner=UnpackMotionOwnerState(ownerState);
    if(currentOwner.Matches(scope.owner))
        return exactActive&&GetCurrentExecutionEpoch()==scope.identity.epoch&&
            !UnpackMotionOwnerSafetyHandshake(ownerState)&&!UnpackMotionOwnerSafetyActionPending(ownerState);
    const auto exactOwnerTicket=[&](std::uint64_t expected) noexcept {
        return expected&&currentOwner.Matches(UnpackMotionOwnerState(expected))&&
            UnpackMotionOwnerSafetyRequestTicket(ownerState)==UnpackMotionOwnerSafetyRequestTicket(expected);
    };
    std::uint64_t expectedOwnerState=scope.expectedSafetyOwnerState;
    if(!exactOwnerTicket(expectedOwnerState)){
        if(m_executionDrainRevocationPublishersInProgress.load(std::memory_order_acquire)==0U||
           !exactOwnerTicket(scope.previousSafetyOwnerState))return false;
        expectedOwnerState=scope.previousSafetyOwnerState;
    }
    if(UnpackMotionOwnerSafetyActionPending(ownerState)&&
       !UnpackMotionOwnerSafetyActionPending(expectedOwnerState))return false;
    MotionExecutionEpoch exactEpoch=MOTION_EXECUTION_EPOCH_INVALID;
    const auto status=TryGetResetSafetyMotionOwnerEpoch(currentOwner,
        UnpackMotionOwnerSafetyRequestTicket(ownerState),provenance,exactEpoch);
    if(status==ResetSafetyAuthorityStatus::ACQUIRED)
        return exactEpoch==GetCurrentExecutionEpoch()&&exactEpoch!=scope.identity.epoch;
    if(status!=ResetSafetyAuthorityStatus::DEFERRED)return false;
    // A pending/handshake marker alone cannot explain an arbitrary GOTO or
    // program epoch. Only this immutable source epoch or its one claimed
    // SAFETY successor belongs to the anticipated Reset transition.
    const auto publication=m_executionEpochPublication.load(std::memory_order_acquire);
    const auto publishedEpoch=static_cast<MotionExecutionEpoch>(publication&0xFFFFFFFFULL);
    const auto publishedSource=static_cast<MotionCommandSource>((publication>>33U)&0xFFULL);
    const auto claim=m_safetyOwnerEpochClaim.load(std::memory_order_acquire);
    const auto successor=scope.identity.epoch==(std::numeric_limits<MotionExecutionEpoch>::max)()?1U:scope.identity.epoch+1U;
    const bool originalPublication=publishedEpoch==scope.identity.epoch&&publishedSource==scope.identity.source;
    const bool resetSuccessor=static_cast<MotionOwnerGeneration>(claim>>32U)==currentOwner.generation&&
        static_cast<MotionExecutionEpoch>(claim&0xFFFFFFFFULL)==scope.identity.epoch&&
        publishedEpoch==successor&&publishedSource==MotionCommandSource::SAFETY;
    if(!originalPublication&&!resetSuccessor)return false;
    // Child ticket acknowledgement intentionally follows the exact batch.
    // Accept its pre-CAS publishing seam or matching packet while output is
    // held; retirement additionally requires the packet to be RT-CLAIMED.
    const auto batch=m_resetSafetyBatchPending.load(std::memory_order_acquire);
    MotionExecutionEpoch appliedSafetyEpoch=MOTION_EXECUTION_EPOCH_INVALID;
    const bool exactBatchPacket=(batch&0x8000000000000000ULL)!=0ULL&&
        (!applyingExactBatch||(batch&0x4000000000000000ULL)!=0ULL)&&
        static_cast<MotionExecutionEpoch>(batch&0xFFFFFFFFULL)==publishedEpoch&&
        static_cast<std::uint32_t>((batch&0x00FFFFFE00000000ULL)>>33U)==UnpackMotionOwnerSafetyRequestTicket(ownerState)&&
        m_resetSafetyBatchProvenanceGeneration.load(std::memory_order_acquire)==provenance&&
        TryGetSafetyMotionOwnerEpoch(currentOwner,appliedSafetyEpoch)&&appliedSafetyEpoch==publishedEpoch;
    const bool childPublishing=UnpackMotionOwnerSafetyActionPending(ownerState)&&
        m_executionDrainRevocationPublishersInProgress.load(std::memory_order_acquire)!=0U;
    const bool exactChildBatchWait=UnpackMotionOwnerSafetyActionPending(expectedOwnerState)&&
        (exactBatchPacket||(!applyingExactBatch&&childPublishing));
    const bool parentAcknowledgementWait=!applyingExactBatch&&resetSuccessor&&
        !UnpackMotionOwnerSafetyActionPending(expectedOwnerState)&&
        !UnpackMotionOwnerSafetyActionPending(ownerState)&&!HasPendingExecutionEpochChange()&&
        TryGetSafetyMotionOwnerEpoch(currentOwner,appliedSafetyEpoch)&&appliedSafetyEpoch==publishedEpoch;
    return exactChildBatchWait||parentAcknowledgementWait||(!applyingExactBatch&&!UnpackMotionOwnerSafetyActionPending(ownerState)&&
        (UnpackMotionOwnerSafetyHandshake(ownerState)||HasPendingExecutionEpochChange()));
}

bool MotionCore::RetireG76SphereForResetSafetyBatch(MotionExecutionEpoch resetEpoch,
    const MotionOwnerLease& resetOwner,std::uint32_t resetTicket,std::uint64_t resetProvenance) noexcept
{
    if(!m_g76SphereRunning) return true;
    // Called only after the existing batch has proved its exact ticket,
    // epoch/provenance and aborted the group. Re-prove the immutable G76Sphere and
    // current Safety tuple; an arbitrary epoch mismatch cannot retire G76Sphere.
    const auto ownerState=m_motionOwnerState.load(std::memory_order_acquire);
    if(!IsG76SphereResetOutputHeld(true)||m_Group.isActive||m_safetyControlledStopInProgress||
       resetEpoch==MOTION_EXECUTION_EPOCH_INVALID||resetEpoch==m_g76SphereFrozen.command.execution.epoch||
       resetEpoch!=GetCurrentExecutionEpoch()||!resetOwner.IsValid()||resetOwner.owner!=MotionOwner::SAFETY||
       !resetOwner.Matches(UnpackMotionOwnerState(ownerState))||!IsMotionOwnerLeaseCurrent(resetOwner)||
       !resetTicket||UnpackMotionOwnerSafetyRequestTicket(ownerState)!=resetTicket||
       UnpackMotionOwnerSafetyHandshake(ownerState)||UnpackMotionOwnerSafetyActionPending(ownerState)||
       !resetProvenance||m_executionDrainRevocationGeneration.load(std::memory_order_acquire)!=resetProvenance||
       m_executionDrainRevocationPublishersInProgress.load(std::memory_order_acquire)!=0U||
       !m_feedbackTrackedTerminal||!SphereIdentity(m_feedbackTrackedIdentity,m_g76SphereFrozen.command.execution)||
       !m_feedbackTrackedOwnerLease.Matches(m_g76SphereFrozen.command.ownerLease)||
       m_g76SphereCurrent.runtimeFault||m_g76SphereCycle.Current().fault!=NCG76SphereCycle::Fault::None||
       m_pContexts==nullptr) return false;
    for(unsigned i=0U;i<6U;++i)if(m_g76SphereFrozen.axisMask&(1U<<i)){
        if(i>=m_pContexts->size()) return false;
        const auto& axis=(*m_pContexts)[i];
        if(axis.currentCmdVel!=0.0||axis.logicalCmdVel!=0.0||axis.targetVelocity!=0.0||axis.targetEndVel!=0.0)
            return false;
    }
    m_g76Dropout.active=false;m_g76SphereCurrent.transportPaused=false;
    m_g76SphereZeroDemand=true;m_g76SphereRunning=false;
    m_g76SphereProcess.Revoke();m_g76SphereCurrent.processShadow=m_g76SphereProcess.Current();
    m_g76SphereCurrent.cycle=m_g76SphereCycle.Current();
    m_g76SphereCurrent.cycle.phase=NCG76SphereCycle::Phase::Fault;m_g76SphereCurrent.cycle.fault=NCG76SphereCycle::Fault::Cancelled;
    m_g76SphereCurrent.cycle.finishActive=false;
    m_g76SphereCurrent.cycle.finishPhase=m_g76SphereCurrent.cycle.finishRequiredCount?5U:0U;
    SphereWithdrawOrbitCompletion(m_g76SphereCurrent.cycle);
    m_g76SphereCurrent.cycle.tick=m_ncSettleRuntimeCycleTick;m_g76SphereCurrent.cycle.held=false;
    m_g76SphereCurrent.cycle.flushActive=false;m_g76SphereCurrent.cycle.flushCheckpointSaved=false;
    m_g76SphereCurrent.commandMCS=m_g76SphereCycle.Command().commandMCS;
    m_g76SphereCurrent.endpointMCS=m_g76SphereCurrent.commandMCS;m_g76SphereCurrent.velocityNative.fill(0.0);
    m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
    m_g76SphereCurrent.terminal=true;m_g76SphereCurrent.completed=false;m_g76SphereCurrent.runtimeFault=0U;
    m_g76SphereCurrent.endpointCommitted=false;
    m_g76SphereCurrent.holdInspectionJogReady=m_g76SphereCurrent.holdRecoveryResumeReady=false;
    m_g76SphereCurrent.cycle.inspectionJogActive=false;
    m_g76SphereInspectionDeadlineUs=0ULL;m_g76SphereInspectionLeaseAxis=-1;
    m_g76SphereInspectionLeaseDirection=0;
    (void)m_g76SphereFeedback.Publish(m_g76SphereCurrent);
    return true;
}


void MotionCore::FailG76SphereRuntime(std::uint32_t fault,bool emergency) noexcept
{
    if(!m_g76SphereRunning)return;
    if(fault==2U&&IsG76SphereResetOutputHeld())return;
    // A member PID fault can retire the parent before later members run.
    // Keep a one-pass image fence so the final publisher scrubs every slot.
    if(!EDM385::Empty(m_g76SphereProcessFrozen.config.flushProfile.axisModeLimits))
        m_g76SphereFinalGuard.faultedTick=m_ncSettleRuntimeCycleTick;
    const bool cancelled=GetCurrentExecutionEpoch()!=m_g76SphereFrozen.command.execution.epoch||
        !IsMotionOwnerLeaseCurrent(m_g76SphereFrozen.command.ownerLease)||HasPendingSafetyOrRecoveryRequests();
    if(fault==1U)CaptureG76SphereInputFault();
    m_g76Dropout.active=false;m_g76SphereCurrent.transportPaused=false;
    m_g76SphereCurrent.runtimeFault=fault;m_g76SphereCurrent.terminal=true;m_g76SphereCurrent.completed=false;
    m_g76SphereInput.cancel=true;
    (void)m_g76SphereCycle.Step(m_g76SphereInput);
    m_g76SphereProcess.Revoke();m_g76SphereProcessSentTick=0ULL;
    if(cancelled)AbortTrackedMotionCommand();
    else{
        const auto alarm=fault==SphereTimeLimitRuntimeFault?SphereTimeLimitAlarm:SphereRuntimeAlarm;
        FaultTrackedMotionCommand(alarm,MotionRejectReason::NOT_READY);
        AlarmManager::GetInstance().Trigger(alarm,m_g76SphereFrozen.command.sourceLinePC);
    }
    m_g76SphereZeroDemand=true;m_g76SphereRunning=false;PublishG76SphereRuntime();
    if(emergency)EmergencyStopAllAxesImpl(false);
}

bool MotionCore::StepG76SphereRuntime() noexcept
{
    if(!m_g76SphereRunning)return false;
    if(m_g76Dropout.active)return true; // No interpolation or process step while the output is fenced.
    auto& input=m_g76SphereInput;
    if(!input.sourceValid||!input.pdoValid||!input.powerInhibited||input.tick!=m_ncSettleRuntimeCycleTick||
        !MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
            m_g76SphereFrozen.request.session,m_g76SphereFrozen.command.execution)){
        auto gate=MotionG76SphereInputGate::InputSourceInvalid;
        if(!input.sourceValid){
            if(m_g76SphereInputObservation.valid&&m_g76SphereInputObservation.gate!=MotionG76SphereInputGate::None){
                FailG76SphereRuntime(1U,true);return true;
            }
        }else if(!input.pdoValid)gate=MotionG76SphereInputGate::InputPdoInvalid;
        else if(!input.powerInhibited)gate=MotionG76SphereInputGate::InputPowerNotInhibited;
        else if(input.tick!=m_ncSettleRuntimeCycleTick)gate=MotionG76SphereInputGate::InputTickMismatch;
        else gate=MotionG76SphereInputGate::InputScopeMismatch;
        SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Step,gate);
        FailG76SphereRuntime(1U,true);return true;
    }
    if(IsG76SphereResetOutputHeld()){
        m_g76SphereProcess.Revoke();m_g76SphereProcessSentTick=0ULL;
        // This exact active reset scope owns the RT callback. Withdraw the
        // observation immediately without reading a large producer snapshot.
        m_g76SphereProcessRevokedSession.store(m_g76SphereFrozen.request.session,std::memory_order_release);
        return true;
    }
    if(GetCommandAuthorizationFailure(m_Group.currentCmd)!=MotionRejectReason::NONE||
        m_g76SphereParentBound.load(std::memory_order_acquire)!=m_g76SphereFrozen.request.session||
        m_Group.enableTransform||m_Group.enableHistory||m_Group.jumpManager.state!=JumpState::IDLE||
        m_pathHold.sourceSeen||m_Group.axisCount!=static_cast<int>(SpherePresentAxisCount(m_g76SphereFrozen.request))||
        SphereCommandMask(m_Group.currentCmd)!=m_g76SphereFrozen.axisMask||m_Group.pathMode!=PathMode::EXACT_STOP||
        !std::isfinite(m_Group.feedrateOverride)||m_Group.feedrateOverride!=1.0){
        FailG76SphereRuntime(2U,true);return true;
    }
    // Begin consumes its own fresh source. A loader reached on this same RT
    // callback must not call the pure engine twice with one physical sample.
    if(m_g76SphereCycle.Current().tick==input.tick)return true;
    LifecycleCommitReservationGuard stepReservation(*this,m_Group.currentCmd.execution);
    if(!stepReservation.IsAcquired()){FailG76SphereRuntime(2U,true);return true;}
    const auto guardedFail=[&](std::uint32_t fault,bool emergency) noexcept {
        stepReservation.Release();FailG76SphereRuntime(fault,emergency);
    };
    if(HasPendingSafetyOrRecoveryRequests()||GetCommandAuthorizationFailure(m_Group.currentCmd)!=MotionRejectReason::NONE){
        guardedFail(2U,true);return true;
    }
    bool haveHold=false;
    input.hold=false;
    for(unsigned count=0U;count<8U&&m_g76SphereControls.ConsumerTryPop(m_g76SphereIncomingControl);++count){
        const auto& intent=m_g76SphereIncomingControl;
        if(intent.session!=m_g76SphereFrozen.request.session)continue;
        if(!SphereIdentity(intent.identity,m_g76SphereFrozen.command.execution)||
            !intent.owner.Matches(m_g76SphereFrozen.command.ownerLease)||!intent.holdSequence){
            guardedFail(6U,true);return true;
        }
        if(intent.hold){
            if(intent.holdSequence<=m_g76SphereCurrent.holdSequence){guardedFail(6U,true);return true;}
            haveHold=true;input.hold=true;m_g76SphereCurrent.holdSequence=intent.holdSequence;
            // A new operator HOLD supersedes the old inspection lease. Its
            // detour remains in the Cycle, but an old button heartbeat cannot
            // restart it or expire later and block this newer held scope.
            m_g76SphereInspectionDeadlineUs=0ULL;m_g76SphereInspectionLeaseAxis=-1;
            m_g76SphereInspectionLeaseDirection=0;m_g76SphereInspectionLeaseHold=0ULL;
            m_g76SphereCurrent.startSequence=(std::max)(m_g76SphereCurrent.startSequence,intent.startSequence);
        }else if(!haveHold){
            if(intent.holdSequence!=m_g76SphereCurrent.holdSequence||
                intent.startSequence<=m_g76SphereCurrent.startSequence){guardedFail(6U,true);return true;}
            m_g76SphereCurrent.startSequence=intent.startSequence;
        }
    }
    // Inspection is a bounded manual detour under this exact suspended AUTO
    // parent. It never acquires or relaxes the generic JOG/MPG motion owner.
    const auto collisionRequested=m_collisionConfigurationRequested.load(std::memory_order_acquire);
    const bool inspectionCollisionSafe=collisionRequested!=0ULL&&
        m_collisionRuntimeConfiguration==collisionRequested&&m_collisionConfigurationValid&&m_collisionCurrent.valid&&
        (!m_collisionCurrent.shortActive||m_collisionManualBypass.load(std::memory_order_acquire));
    const bool leaseRead=m_g76SphereInspectionLease.Read(m_g76SphereInspectionLeaseIncoming).Ready();
    const auto& lease=m_g76SphereInspectionLeaseIncoming;
    const bool leaseScoped=leaseRead&&lease.session==m_g76SphereFrozen.request.session&&lease.sequence&&
        lease.holdSequence==m_g76SphereCurrent.holdSequence&&
        SphereIdentity(lease.identity,m_g76SphereFrozen.command.execution)&&
        lease.owner.Matches(m_g76SphereFrozen.command.ownerLease);
    for(unsigned count=0U;count<8U&&m_g76SphereInspectionControls.ConsumerTryPop(m_g76SphereInspectionIncoming);++count){
        const auto& intent=m_g76SphereInspectionIncoming;
        if(intent.session!=m_g76SphereFrozen.request.session)continue;
        if(!SphereIdentity(intent.identity,m_g76SphereFrozen.command.execution)||
           !intent.owner.Matches(m_g76SphereFrozen.command.ownerLease)||!intent.holdSequence||!intent.sequence){
            guardedFail(6U,true);return true;
        }
        if(intent.sequence<=m_g76SphereInspectionConsumedSequence)continue;
        m_g76SphereInspectionConsumedSequence=intent.sequence;
        if(intent.holdSequence<m_g76SphereCurrent.holdSequence)continue;
        if(intent.holdSequence!=m_g76SphereCurrent.holdSequence||!m_g76SphereHoldFrozen.enabled||
           !m_g76SphereCycle.Current().holdRecoveryEnabled){guardedFail(6U,true);return true;}
        const bool stop=intent.axis==-1&&intent.direction==0;
        if(!stop&&(intent.axis<0||intent.axis>2||(intent.direction!=1&&intent.direction!=-1))){
            guardedFail(6U,true);return true;
        }
        if(stop){
            (void)m_g76SphereCycle.EndInspectionJog();m_g76SphereInspectionDeadlineUs=0ULL;
            m_g76SphereInspectionLeaseAxis=-1;m_g76SphereInspectionLeaseDirection=0;
            m_g76SphereInspectionBlocked=false;continue;
        }
        // A later scoped STOP must contain an earlier queued button press.
        if(leaseScoped&&lease.sequence>=intent.sequence&&lease.axis==-1&&lease.direction==0)continue;
        bool atHeldCommand=!haveHold&&!m_g76SphereInspectionBlocked&&inspectionCollisionSafe&&
            input.physicalSettled&&input.zeroFinalSend&&m_g76SphereCycle.CanInspectionJog()&&
            m_g76SphereCycle.Command().stopped&&m_g76SphereCycle.Command().zeroDemand;
        for(unsigned i=0U;atHeldCommand&&i<SphereAxisCount(m_g76SphereFrozen.request);++i){
            if(!SphereAxisPresent(m_g76SphereFrozen.request,i))continue;
            const auto& axis=(*m_pContexts)[i];const double ppu=m_g76SphereFrozen.pulsePerNative[i];
            const double tolerance=(std::min)(axis.inPositionWindow_Pulse/ppu,SphereRotary(m_g76SphereFrozen.request,i)?
                m_g76SphereFrozen.request.rotaryToleranceDeg:m_g76SphereFrozen.request.positionToleranceMm);
            atHeldCommand=std::isfinite(input.actualMCS[i])&&
                std::fabs(input.actualMCS[i]-m_g76SphereCycle.Command().commandMCS[i])<=tolerance;
        }
        if(!inspectionCollisionSafe)m_g76SphereInspectionBlocked=true;
        // Qualification changes consume this JOG request; no automatic motion
        // is queued until a new released/repressed, exact-scoped command.
        if(atHeldCommand&&m_g76SphereCycle.BeginInspectionJog(static_cast<unsigned>(intent.axis),
            static_cast<double>(intent.direction)*m_g76SphereHoldFrozen.config.jogRecoveryFeedMmMin)){
            m_g76SphereInspectionLeaseAxis=intent.axis;m_g76SphereInspectionLeaseDirection=intent.direction;
            m_g76SphereInspectionLeaseHold=intent.holdSequence;
            m_g76SphereInspectionLeaseSeen=intent.sequence;
            m_g76SphereInspectionDeadlineUs=input.nowUs>(std::numeric_limits<std::uint64_t>::max)()-750000ULL?
                (std::numeric_limits<std::uint64_t>::max)():input.nowUs+750000ULL;
        }
    }
    if(leaseScoped&&lease.sequence>m_g76SphereInspectionLeaseSeen){
        m_g76SphereInspectionLeaseSeen=lease.sequence;
        if(lease.axis==-1&&lease.direction==0){
            (void)m_g76SphereCycle.EndInspectionJog();m_g76SphereInspectionDeadlineUs=0ULL;
            m_g76SphereInspectionLeaseAxis=-1;m_g76SphereInspectionLeaseDirection=0;
            m_g76SphereInspectionBlocked=false;
        }else if(m_g76SphereInspectionDeadlineUs!=0ULL&&!m_g76SphereInspectionBlocked&&
                 lease.holdSequence==m_g76SphereInspectionLeaseHold&&lease.axis==m_g76SphereInspectionLeaseAxis&&
                 lease.direction==m_g76SphereInspectionLeaseDirection&&m_g76SphereCycle.Current().inspectionJogActive){
            m_g76SphereInspectionDeadlineUs=input.nowUs>(std::numeric_limits<std::uint64_t>::max)()-750000ULL?
                (std::numeric_limits<std::uint64_t>::max)():input.nowUs+750000ULL;
        }
    }
    if(m_g76SphereInspectionDeadlineUs!=0ULL&&input.nowUs>=m_g76SphereInspectionDeadlineUs){
        (void)m_g76SphereCycle.EndInspectionJog();m_g76SphereInspectionDeadlineUs=0ULL;
        m_g76SphereInspectionLeaseAxis=-1;m_g76SphereInspectionLeaseDirection=0;
        m_g76SphereInspectionBlocked=true; // Late heartbeats cannot restart it.
    }
    if(!inspectionCollisionSafe&&(m_g76SphereCycle.Current().inspectionJogActive||
        m_g76SphereCycle.Current().holdRecoveryPhase==6U)){
        if(m_g76SphereCycle.Current().inspectionJogActive)(void)m_g76SphereCycle.EndInspectionJog();
        else input.hold=true; // Pause the return leg with the same held-parent sequence.
        m_g76SphereInspectionDeadlineUs=0ULL;m_g76SphereInspectionLeaseAxis=-1;
        m_g76SphereInspectionLeaseDirection=0;m_g76SphereInspectionBlocked=true;
    }
    if(m_g76SphereInspectionBlocked)input.hold=true; // Latched until an exact scoped STOP.
    input.holdSequence=m_g76SphereCurrent.holdSequence;input.startSequence=m_g76SphereCurrent.startSequence;
    const auto previousNominal=m_g76SphereCycle.Current();
    const auto previousCommand=m_g76SphereCycle.Command().commandMCS;
    const auto& processBefore=m_g76SphereProcess.Current();
    const auto& processFrozen=m_g76SphereProcessFrozen;
    if(!MotionG76SphereMachiningTime::Matches(m_g76SphereHoldFrozen.machiningTime,m_g76SphereFrozen.request)||
       previousNominal.totalLimitUs!=m_g76SphereFrozen.request.totalLimitUs||
       previousNominal.additionalUs!=m_g76SphereFrozen.request.additionalUs||
       (previousNominal.timeLimitReached&&!MotionG76SphereMachiningTime::TimeLimitReachedQualified(previousNominal,m_g76SphereLiveCompletionRequest))||
       previousNominal.finishRequiredCount!=m_g76SphereHoldFrozen.finishRequiredCount||
       m_g76SphereHoldFrozen.orbitInspectionRounds!=m_g76SphereFrozen.request.orbitInspectionRounds||
       m_g76SphereHoldFrozen.orbitThresholdScaled!=m_g76SphereFrozen.request.orbitThresholdScaled||
       m_g76SphereHoldFrozen.orbitRetryLimit!=m_g76SphereFrozen.request.orbitRetryLimit||
       !SphereOrbitCompletionFeedbackMatches(previousNominal,m_g76SphereLiveCompletionRequest.orbitInspectionRounds,
            m_g76SphereLiveCompletionRequest.orbitThresholdScaled,false,m_g76SphereLiveCompletionRequest.orbitRetryLimit)||
       (m_g76SphereHoldFrozen.finishRequiredCount!=0U&&
        (!processFrozen.config.enabled||!processFrozen.config.signedMotionEnabled||!processFrozen.config.flushMotionEnabled))){
        guardedFail(9U,true);return true;
    }
    if(processFrozen.config.orbitFeedsEnabled!=m_g76SphereFrozen.orbitFeedsEnabled||
       !MotionG76SphereParameterFeeds::Matches(m_g76SphereFrozen.request,processFrozen.config)){
        guardedFail(9U,true);return true;
    }
    if(m_g76SphereFrozen.request.parameterFeeds||m_g76SphereFrozen.orbitFeedsEnabled){
        // Parameter-mode data remains immutable for the whole parent. This
        // persistent fixed-size copy adds no stack-sized process temporary.
        const bool read=m_g76SphereProcessRequests.Read(m_g76SphereIncomingProcessFrozen).Ready();
        const auto& published=m_g76SphereIncomingProcessFrozen;
        if(!read||published.session!=m_g76SphereFrozen.request.session||
           !SphereIdentity(published.identity,m_g76SphereFrozen.command.execution)||
           !published.owner.Matches(m_g76SphereFrozen.command.ownerLease)||
           !NCG76SphereGapProcess::EquivalentForContinuation(published.config,processFrozen.config)){
            guardedFail(9U,true);return true;
        }
    }
    if(m_g76SphereFrozen.request.physicalZManual){
        if(!m_g76SpherePhysicalSource.Ready()||!m_g76SpherePhysicalSource.CurrentAt(input.nowUs,m_edmZClockFrequency))
            {guardedFail(9U,true);return true;}
    }
    if(!ApplyG76SphereLiveRecipeUpdate()||!ApplyDeferredG76SphereRecipeFlush()||
       (!m_g76SphereLiveRecipeSequence&&(!ApplyG76SphereWorkTimeUpdate()||!ApplyG76SphereProcessTuningUpdate()))){
        guardedFail(9U,true);return true;
    }
    const bool processRevoked=m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==m_g76SphereFrozen.request.session;
    const bool processCurrent=!processRevoked&&processBefore.enabled&&processBefore.valid&&
        processBefore.reason==NCG76SphereGapProcess::Reason::None&&
        processBefore.session==m_g76SphereFrozen.request.session&&processBefore.tick&&
        processBefore.tick==input.tick-1ULL&&
        processBefore.recipeGeneration==processFrozen.config.recipeGeneration&&
        processBefore.tuningGeneration==processFrozen.config.tuningGeneration;
    const bool signedMotion=m_g76SphereFrozen.signedMotionEnabled;
    const auto flushFeatures=static_cast<std::uint8_t>((processFrozen.config.independentFeedsEnabled?1U:0U)|
        (processFrozen.config.finalApproachEnabled?2U:0U)|(processFrozen.config.initialSlowEnabled?4U:0U)|
        (processFrozen.config.deepFlushEnabled?8U:0U));
    if(flushFeatures!=m_g76SphereFrozen.flushFeatureFlags||
       (flushFeatures!=0U&&!NCG76SphereGapProcess::Valid(processFrozen.config))){
        guardedFail(9U,true);return true;
    }
    if(m_g76SphereFrozen.request.retreatMode!=0U&&(!signedMotion||!processFrozen.config.enabled)){
        guardedFail(9U,true);return true;
    }
    const bool gapCurrent=processCurrent&&processFrozen.config.signedMotionEnabled&&
        processBefore.sourceSequence&&processBefore.sourceCapturedAtUs&&
        processBefore.sourceCapturedAtUs<=input.nowUs&&
        input.nowUs-processBefore.sourceCapturedAtUs<=NCG76SphereGapProcess::MaximumSourceAgeUs&&
        std::isfinite(processBefore.signedGapSpeedMmMin);
    // Only the first stationary callback may seed the SIM source. Once this
    // motion contract is admitted it can never fall back to fixed AIR feed.
    const bool processSeed=signedMotion&&!processRevoked&&processBefore.enabled&&
        !processBefore.valid&&processBefore.reason==NCG76SphereGapProcess::Reason::None&&
        processBefore.session==m_g76SphereFrozen.request.session&&
        processBefore.recipeGeneration==processFrozen.config.recipeGeneration&&
        processBefore.tuningGeneration==processFrozen.config.tuningGeneration&&processFrozen.config.signedMotionEnabled&&
        !processBefore.tick&&previousNominal.phase==NCG76SphereCycle::Phase::Settle&&
        previousNominal.nominalProgressMm==0.0;
    if(signedMotion&&!gapCurrent&&!processSeed){guardedFail(9U,true);return true;}
    input.gapValid=signedMotion&&gapCurrent;
    input.signedGapSpeedMmMin=input.gapValid?processBefore.signedGapSpeedMmMin:0.0;
    const auto consumedCurveTick=processBefore.tick;
    const auto consumedGainRegion=processBefore.gainRegion;
    const bool consumedGainValid=input.gapValid&&processBefore.gainRegionValid;
    const double consumedReferenceV=processBefore.referenceV;
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    // Current() is a reference into the observer. Preserve the consumed
    // scalars BEFORE its Step updates that same object to the next demand.
    if(m_g01ResponseTrace.ActiveSession()==m_g76SphereFrozen.request.session){
        if(auto* response=m_g01ResponseTrace.Pending(input.tick)){
            response->usedCurveTick=processBefore.tick;
            response->usedSequence=processBefore.sourceSequence;
            response->usedCapturedUs=processBefore.sourceCapturedAtUs;
            response->usedVoltageV=processBefore.voltageV;
            response->usedDemandMmMin=input.signedGapSpeedMmMin;
            if(input.gapValid)response->flags|=EDMG01ResponseTrace::UsedDemandValid;
        }
    }
    // EDM386_G01_RESPONSE_TRACE_END
    // The observer proposes a request only. The pure parent owns braking,
    // fresh-zero settle, the stopped checkpoint, excursion and path restore.
    input.flushRequestSequence=processCurrent&&processFrozen.config.enabled&&
        m_g76SphereProcess.LiveFlushEnabled()&&processBefore.pending?processBefore.requestSequence:0ULL;
    if(previousNominal.flushActive&&!processCurrent){guardedFail(9U,true);return true;}
    const auto output=m_g76SphereCycle.Step(input);
    RefreshG76SphereLiveRecipeReceipts();
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    // START may leave Held during this Step; close the dump gate immediately.
    if(!m_g76SphereCycle.Current().held&&!output.complete)
        m_g01ResponseDumpSafe.store(false,std::memory_order_release);
    // EDM386_G01_RESPONSE_TRACE_END
    if(output.emergencyStopRequired||output.faulted){
        const bool timeLimit=!output.emergencyStopRequired&&output.faulted&&output.stopped&&output.zeroDemand&&
            m_g76SphereCycle.Current().fault==NCG76SphereCycle::Fault::TimeLimit;
        if(timeLimit){
            // The deadline is not a successful endpoint. Retire the already
            // physically settled axes before withdrawing this runtime; otherwise
            // the next callback could enter the ordinary virtual-axis planner.
            if(!input.physicalSettled||!input.zeroFinalSend||!m_g76SphereZeroDemand||!RetireG80Axes()){
                guardedFail(3U,true);return true;
            }
            m_Group.isActive=false;m_g76SphereCurrent.timeLimitStopped=true;
        }
        guardedFail(timeLimit?SphereTimeLimitRuntimeFault:3U,!timeLimit);return true;
    }
    NCG76SphereGapProcess::Input processInput{};processInput.nominal=previousNominal;
    // Select the live leg AFTER Step: this evaluated demand is consumed on
    // the next tick. E5 accounting still uses the accepted previous cursor.
    processInput.gapFeedPhase=m_g76SphereCycle.ResolveGapFeedPhase();
    processInput.candidateMCS=previousCommand;processInput.session=m_g76SphereFrozen.request.session;
    processInput.tick=input.tick;processInput.nowUs=input.nowUs;
    processInput.stationaryNominal=m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
        m_g76SphereFrozen.request.geometry.travelMm==0.0;
    processInput.acceptedSendTick=m_g76SphereProcessSentTick;
    // H endpoint work still requires a current physical/zero-send receipt.
    processInput.additionalEndpointQualified=(m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line||
        m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc)&&
        previousNominal.additionalUs!=0ULL&&previousNominal.additionalElapsedUs<previousNominal.additionalUs&&
        previousNominal.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::Additional)&&
        previousNominal.additionalTimerState==static_cast<std::uint8_t>(NCMachiningTime::TimerState::Running)&&
        input.physicalSettled&&input.zeroFinalSend&&input.zeroSendTick==input.tick-1ULL;
    processInput.sourceValid=input.sourceValid&&input.pdoValid;
    processInput.powerInhibited=input.powerInhibited;
    if(m_g76SphereFrozen.request.physicalZManual)processInput.physicalFrame=m_g76SpherePhysicalSource.CurrentFrame();
    const auto& currentNominal=m_g76SphereCycle.Current();
    if(!m_g76SphereLiveRecipeSequence&&!m_g76SphereCycle.HasPendingFlushFeeds()&&
       m_g76SphereCurrent.liveCurveTuningGeneration==m_g76SphereLiveTuningGeneration)
        m_g76SphereCurrent.liveFlushTuningGeneration=m_g76SphereLiveTuningGeneration;
    processInput.suspended=input.hold||previousNominal.holdActive||currentNominal.holdActive||
        previousNominal.flushActive||currentNominal.flushActive||
        previousNominal.gapHistoryActive||currentNominal.gapHistoryActive||
        previousNominal.finishActive||currentNominal.finishActive||
        currentNominal.timeLimitReached||
        currentNominal.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::TimedOut);
    // Timer round reset is specifically HOLD and its return/recovery. GAP
    // history retreat may suspend nominal work but must not restart Mode 1.
    processInput.holdResetActive=input.hold||previousNominal.holdActive||currentNominal.holdActive||
        previousNominal.held||currentNominal.held;
    processInput.currentFlushActive=currentNominal.flushActive;
    processInput.currentFlushSequence=currentNominal.flushRequestSequence;
    processInput.timingStopActive=currentNominal.finishActive||currentNominal.timeLimitReached||
        currentNominal.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::TimedOut);
    processInput.terminal=output.complete;processInput.cancel=input.cancel;
    processInput.completedFlushSequence=currentNominal.flushCompletedSequence;
    processInput.abortedFlushSequence=currentNominal.flushAbandonedSequence;
    if(m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==processInput.session)
        m_g76SphereProcess.Revoke();
    else (void)m_g76SphereProcess.Step(processInput);
    // Only publish the band actually used by this motion callback. A fresh
    // next-demand evaluation during HOLD/flush is not an applied servo band.
    m_g76Scope.session=m_g76SphereFrozen.request.session;
    m_g76Scope.tick=input.tick;m_g76Scope.curveTick=0ULL;
    m_g76Scope.gainRegion=0;m_g76Scope.gainRegionValid=false;
    m_g76Scope.referenceV=m_g76SphereProcess.Current().referenceV;
    if(input.gapValid&&currentNominal.gapApplied&&!currentNominal.holdActive&&!currentNominal.flushActive){
        m_g76Scope.curveTick=consumedCurveTick;
        m_g76Scope.referenceV=consumedReferenceV;
        m_g76Scope.gainRegionValid=consumedGainValid;
        if(consumedGainValid)m_g76Scope.gainRegion=consumedGainRegion;
    }
    const auto& scopeProcess=m_g76SphereProcess.Current();
    if(!scopeProcess.pending||processInput.holdResetActive||m_g76SphereProcess.E5DueUs()==0ULL){
        m_g76Scope.requestSequence=0ULL;m_g76Scope.flushAcceptedUs=0ULL;m_g76Scope.flushMotionUs=0ULL;
    }else{
        if(m_g76Scope.requestSequence!=scopeProcess.requestSequence){
            m_g76Scope.requestSequence=scopeProcess.requestSequence;
            m_g76Scope.flushAcceptedUs=0ULL;m_g76Scope.flushMotionUs=0ULL;
        }
        if(currentNominal.flushActive&&
           currentNominal.flushOrigin==static_cast<std::uint8_t>(NCG76SphereCycle::FlushOrigin::Periodic)&&
           currentNominal.flushRequestSequence==scopeProcess.requestSequence){
            if(!m_g76Scope.flushAcceptedUs)m_g76Scope.flushAcceptedUs=input.nowUs;
            const auto p=currentNominal.phase;
            const bool excursion=p==NCG76SphereCycle::Phase::FlushLift||p==NCG76SphereCycle::Phase::FlushToCenter||
                p==NCG76SphereCycle::Phase::FlushPathRetreat||p==NCG76SphereCycle::Phase::FlushReturn||
                p==NCG76SphereCycle::Phase::FlushFromCenter||p==NCG76SphereCycle::Phase::FlushPathReturn;
            // Entering a leg with speed zero is not movement. This event is
            // the first nonzero excursion COMMAND, not encoder response.
            if(excursion&&!m_g76Scope.flushMotionUs&&output.commandMCS!=previousCommand)
                m_g76Scope.flushMotionUs=input.nowUs;
        }
    }
    if(!ApplyDeferredG76SphereRecipeFlush()){guardedFail(9U,true);return true;}
    RefreshG76SphereLiveRecipeReceipts();
    if((previousNominal.flushActive||currentNominal.flushActive||(signedMotion&&!output.complete))&&
       (!m_g76SphereProcess.Current().valid||
        m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==processInput.session)){
        guardedFail(9U,true);return true;
    }
    bool positionReady=output.stopped&&output.zeroDemand;
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    if(m_g01ResponseTrace.ActiveSession()==m_g76SphereFrozen.request.session){
        if(auto* response=m_g01ResponseTrace.Pending(input.tick)){
            const auto& next=m_g76SphereProcess.Current();
            response->stages|=EDMG01ResponseTrace::Motion;
            if(input.sourceValid)response->flags|=EDMG01ResponseTrace::SourceValid;
            if(next.valid)response->flags|=EDMG01ResponseTrace::NewCurveValid;
            if(m_collisionCurrent.valid)response->flags|=EDMG01ResponseTrace::CollisionValid;
            if(!processInput.suspended&&currentNominal.gapApplied&&
               currentNominal.phase==NCG76SphereCycle::Phase::InitialDepth)
                response->flags|=EDMG01ResponseTrace::Nominal;
            if(m_collisionCurrent.valid&&m_collisionCurrent.shortActive)response->flags|=EDMG01ResponseTrace::Short;
            if(processInput.suspended)response->flags|=EDMG01ResponseTrace::Suspended;
            if(currentNominal.gapApplied)response->flags|=EDMG01ResponseTrace::GapApplied;
            if(currentNominal.holdActive)response->flags|=EDMG01ResponseTrace::Hold;
            if(currentNominal.flushActive)response->flags|=EDMG01ResponseTrace::Flush;
            response->phase=static_cast<std::uint32_t>(currentNominal.phase);
            response->newCurveTick=next.tick;response->newSequence=next.sourceSequence;
            response->newCapturedUs=next.sourceCapturedAtUs;response->newVoltageV=next.voltageV;
            response->nextDemandMmMin=next.signedGapSpeedMmMin;
            response->pathMmMin=currentNominal.signedSpeedMmS*60.0;
            response->zReferenceMmMin=output.velocityNative[2]*60.0;
            response->ppu=m_g76SphereFrozen.pulsePerNative[2];
            response->commandPulse=output.commandMCS[2]*response->ppu;
            if(m_g76SphereAnalogRead){
                response->rawAd=static_cast<std::uint32_t>(m_g76SphereAnalogInput.rawCode);
                response->flags|=EDMG01ResponseTrace::RawAdValid;
            }
        }
    }
    // EDM386_G01_RESPONSE_TRACE_END
    for(int slot=0;slot<m_Group.axisCount;++slot){
        const unsigned i=static_cast<unsigned>(m_Group.axisIndices[slot]);
        if(i>=6U||i!=static_cast<unsigned>(m_g76SphereFrozen.command.axisIndices[slot])){
            guardedFail(2U,true);return true;
        }
        auto& axis=(*m_pContexts)[i];const double ppu=m_g76SphereFrozen.pulsePerNative[i];
        double position=output.commandMCS[i]*ppu;
        const double velocity=output.velocityNative[i]*ppu;
        const double acceleration=output.accelerationNative[i]*ppu;
        const bool fixedValid=i<3U||MotionG76SphereFixedAxes::CommandPulseRelative(output.commandMCS[i],
            m_g76SphereFrozen.request.geometry.startMCS[i],m_g76SphereFrozen.command.mem_startPos[slot],ppu,position,
            NCG76SphereRuntimeGeometry::NativeAxisErrorBound(m_g76SphereFrozen.request.geometry,i));
        if(!std::isfinite(position)||!std::isfinite(velocity)||!std::isfinite(acceleration)||
            std::fabs(velocity)>axis.maxVel_PPS*(1.0+1.e-12)||
            !fixedValid||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,output.commandMCS[i])){
            guardedFail(3U,true);return true;
        }
        {
            const auto& b=m_g76SphereFrozen.axis[i];
            const auto& limits=processFrozen.config.flushProfile.axisModeLimits;
            const bool flushing=m_g76SphereCycle.UsingFlushAxisLimits();
            double rawA=0.0,rawD=0.0;
            if(m_g76SphereFrozen.signedMotionEnabled){
                if(!processFrozen.config.flushAxisLimitsEnabled||!(processFrozen.config.flushAxisMask&(1U<<i))){
                    guardedFail(3U,true);return true;
                }
                rawA=processFrozen.config.flushAxisAccelerationMmS2[i]*ppu;
                rawD=processFrozen.config.flushAxisDecelerationMmS2[i]*ppu;
            }else{
                const double accTime=b.accSeconds<0.001?0.2:b.accSeconds;
                const double decTime=(std::max)(b.decSeconds<0.001?0.2:b.decSeconds,b.stopSeconds<0.001?0.2:b.stopSeconds);
                rawA=b.maxVelocity/accTime;rawD=b.maxVelocity/decTime;
            }
            double modeA=0.0,modeD=0.0;
            if(!SpherePulseCeiling385(rawA,flushing?limits.flushAcceleration[i]:limits.machiningAcceleration[i],ppu,modeA)||
               !SpherePulseCeiling385(rawD,flushing?limits.flushDeceleration[i]:limits.machiningDeceleration[i],ppu,modeD)){
                guardedFail(3U,true);return true;
            }
            const double selected=velocity*acceleration>0.0?modeA:
                velocity*acceleration<0.0?modeD:(std::min)(modeA,modeD);
            if(std::fabs(acceleration)>(std::min)((std::min)(rawA,rawD),selected)*(1.0+1.e-10)){
                guardedFail(3U,true);return true;
            }
        }
        const double window=(std::min)(axis.inPositionWindow_Pulse/ppu,SphereRotary(m_g76SphereFrozen.request,i)?
            m_g76SphereFrozen.request.rotaryToleranceDeg:m_g76SphereFrozen.request.positionToleranceMm);
        positionReady=positionReady&&std::fabs(input.actualMCS[i]-output.commandMCS[i])<=window;
        axis.logicalCmdPos=axis.currentCmdPos=position;axis.logicalCmdVel=axis.currentCmdVel=velocity;
        axis.planningPos=axis.finalTargetPos=position;axis.targetVelocity=velocity;axis.targetEndVel=0.0;
    }
    // Let the unchanged PID finish approaching a stationary endpoint first.
    // Only a physically in-window reference requests final zero PDO. The
    // engine then requires its phase-specific NEW adjacent zero-send/settle proofs.
    m_g76SphereZeroDemand=positionReady;
    if(!m_g76SphereZeroDemand){m_g76SphereZeroSentTick=0ULL;m_g76SpherePhysicalRange=false;}
    if(output.complete&&output.stopped){
        if(!input.physicalSettled||!input.zeroFinalSend||!m_g76SphereZeroDemand){guardedFail(4U,true);return true;}
        const auto& finish=m_g76SphereCycle.Current();
        if(!MotionG76SphereMachiningTime::TerminalQualified(finish,m_g76SphereLiveCompletionRequest)){
            guardedFail(3U,true);return true;
        }
        if(!SphereOrbitCompletionFeedbackMatches(finish,m_g76SphereLiveCompletionRequest.orbitInspectionRounds,
            m_g76SphereLiveCompletionRequest.orbitThresholdScaled,true,m_g76SphereLiveCompletionRequest.orbitRetryLimit)){
            guardedFail(3U,true);return true;
        }
        if(finish.timedCompletion&&output.commandMCS!=m_g76SphereCycle.TimedCompletionEndpoint()){
            guardedFail(3U,true);return true;
        }
        if(!finish.timedCompletion&&m_g76SphereHoldFrozen.finishRequiredCount!=0U&&
           (finish.finishActive||finish.finishPhase!=4U||
            finish.finishVerifiedCount!=m_g76SphereHoldFrozen.finishRequiredCount||
            finish.finishForcedCompletedCount<finish.finishVerifiedCount)){
            guardedFail(3U,true);return true;
        }
        if(!AlarmManager::GetInstance().HasAlarm()&&
            GetCommandAuthorizationFailure(m_Group.currentCmd)==MotionRejectReason::NONE){
            // This helper canonicalizes common stopped-axis state only; no
            // G80 controller or gain selection is called by the P50 lane.
            if(!RetireG80Axes()){guardedFail(3U,true);return true;}
            const auto before=m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire);
            CompleteTrackedMotionCommand(m_Group.currentCmd);
            if(m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire)==before){
                guardedFail(3U,true);return true;
            }
            m_Group.isActive=false;m_g76SphereRunning=false;
            m_g76SphereCurrent.terminal=m_g76SphereCurrent.completed=true;
        }
    }
    stepReservation.Release();PublishG76SphereRuntime();return true;
}

bool MotionCore::ReadMachiningScopeEdmFromOwner(std::uint64_t tick,MachiningScope379::Record& row) const noexcept
{
    using P=MachiningScope379::ProcessPhase;
    using C=NCG76SphereCycle::Phase;
    const auto& n=m_g76SphereCycle.Current();
    const auto& input=m_g76SphereInput;
    const auto parent=m_g76SphereFrozen.request.session;
    // Same callback and admitted parent only. No producer copy, lock, clock
    // read, inference from NC RUN, or telemetry authority enters control.
    if(!tick||!parent||n.session!=parent||n.tick!=tick||input.tick!=tick||!input.nowUs||
       !input.sourceValid||!input.pdoValid||m_g76Dropout.active||
       !MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
            parent,m_g76SphereFrozen.command.execution))return false;
    P phase=P::Waiting;
    switch(n.phase){
    case C::Idle:phase=P::Idle;break;
    case C::InitialDepth:case C::CircleEntry:case C::Orbit:case C::AdvanceCenter:
    case C::ContourNominal:case C::ContourFinish:phase=P::Machining;break;
    case C::InitialRetreat:case C::GapHistory:case C::TimedHistoryReturn:phase=P::ReturningToWork;break;
    case C::ReturnCenter:case C::TimedReturnCenter:phase=P::ToCenter;break;
    case C::Settle:case C::GapHistorySettle:phase=P::Waiting;break;
    case C::HoldBrake:case C::HoldSettle:case C::ResumeBrake:
    case C::RecoveryBrake:case C::RecoverySettle:phase=P::Stopping;break;
    case C::HoldToCenter:case C::HoldToStart:case C::RecoveryMove:phase=P::HoldRetreat;break;
    case C::HeldAtStart:phase=P::Held;break;
    case C::ResumeToCenter:case C::ResumeToAnchor:case C::InspectionJogReturn:phase=P::ReturningToWork;break;
    case C::InspectionJog:phase=P::RapidOrJog;break;
    case C::InspectionJogBrake:case C::FaultBrake:case C::FaultSettle:
    case C::TimeLimitBrake:case C::TimeLimitSettle:case C::FlushHoldBrake:phase=P::Stopping;break;
    case C::Complete:phase=P::Complete;break;
    case C::Fault:phase=P::Fault;break;
    case C::FlushBrake:phase=P::FlushBrake;break;
    case C::FlushLift:case C::FlushToCenter:case C::FlushPathRetreat:phase=P::FlushOutbound;break;
    case C::FlushReturn:case C::FlushFromCenter:case C::FlushPathReturn:
        phase=n.flushApproachActive?P::FlushFinalApproach:P::FlushReturning;break;
    case C::FlushSettle:{
        using A=NCG76SphereCycle::Cycle::SettleAction;
        const auto action=m_g76SphereCycle.settleAction_;
        if(action==A::CaptureFlush)phase=P::FlushAnchorDwell;
        else if(action==A::RestoreAfterFlush)phase=P::FlushReturnDwell;
        else if(action==A::NextFlushLeg){
            const auto route=m_g76SphereCycle.flushRouteIndex_;
            const auto mode=m_g76SphereFrozen.request.retreatMode;
            if(mode<=2U)phase=route==0U?P::FlushTopDwell:P::FlushReturnDwell;
            else phase=route==0U?P::CenterDwell:route==1U?P::FlushTopDwell:P::FlushReturnDwell;
        }else phase=P::Waiting;
        break;}
    }
    row.processPhase=static_cast<std::uint32_t>(phase);
    row.processFlags=MachiningScope379::ProcessValid|MachiningScope379::ProcessSourceRt;
    row.processAgeUs=0U;
    const auto& p=m_g76SphereProcess.Current();
    if(!p.enabled||!p.valid||p.reason!=NCG76SphereGapProcess::Reason::None||p.session!=parent||p.tick!=tick||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==parent)return true;
    row.parentSession=parent;row.edmNowUs=input.nowUs;row.edmFlags=MachiningScope379::EdmValid;
    if(std::isfinite(p.referenceV))row.gapReferenceVoltage=p.referenceV;
    if(m_g76Scope.session==parent&&m_g76Scope.tick==tick){
        if(std::isfinite(m_g76Scope.referenceV))row.gapReferenceVoltage=m_g76Scope.referenceV;
        if(m_g76Scope.gainRegionValid&&m_g76Scope.curveTick&&m_g76Scope.curveTick<=tick&&
           m_g76Scope.gainRegion>=-3&&m_g76Scope.gainRegion<=3){
            row.gainRegion=m_g76Scope.gainRegion;row.curveTick=m_g76Scope.curveTick;
            row.edmFlags|=MachiningScope379::GainRegionValid;
        }
        if(p.pending&&m_g76Scope.requestSequence==p.requestSequence){
            row.flushAcceptedUs=m_g76Scope.flushAcceptedUs;row.flushMotionUs=m_g76Scope.flushMotionUs;
            if(row.flushMotionUs)row.edmFlags|=MachiningScope379::FlushPhysicalMotion;
        }
    }
    row.e5ElapsedUs=(std::min)(p.intervalWorkUs,p.workTimeUs);row.e5TargetUs=p.workTimeUs;
    row.e5StartUs=m_g76SphereProcess.E5StartUs();row.e5DueUs=m_g76SphereProcess.E5DueUs();
    row.e5RequestSequence=p.requestSequence;
    const bool mode1=m_g76SphereProcessFrozen.config.e5WorkTimer.mode==1U;
    if(p.workTimeUs&&(!mode1||p.e5Timer.valid))row.edmFlags|=MachiningScope379::E5TimerValid;
    if(mode1)row.edmFlags|=MachiningScope379::E5Mode1;
    if(p.e5Eligible)row.edmFlags|=MachiningScope379::E5Eligible;
    if(p.eligible)row.edmFlags|=MachiningScope379::CoreForwardEligible;
    if(p.e5Timer.latched)row.edmFlags|=MachiningScope379::E5GateLatched;
    if(p.e5Timer.allow)row.edmFlags|=MachiningScope379::E5GateAllow;
    if(p.e5ShortBlocked)row.edmFlags|=MachiningScope379::E5ShortBlocked;
    if(p.pending)row.edmFlags|=MachiningScope379::E5Pending;
    if(p.suspended)row.edmFlags|=MachiningScope379::EdmSuspended;
    if(n.flushActive&&n.flushOrigin==static_cast<std::uint8_t>(NCG76SphereCycle::FlushOrigin::Periodic))
        row.edmFlags|=MachiningScope379::PeriodicFlushActive;
    if(m_g76SphereProcess.LiveFlushEnabled())row.edmFlags|=MachiningScope379::FlushEnabled;
    if(mode1&&p.e5Timer.valid&&p.e5Timer.filterReady&&std::isfinite(p.e5Timer.filteredVoltageV))
        row.filteredGapVoltage=p.e5Timer.filteredVoltageV;
    return true;
}

namespace {
bool SphereHasModeCaps385(const EDM385::AxisLimits& limits,std::uint32_t mask) noexcept
{
    for(unsigned i=0U;i<6U;++i)if(mask&(1U<<i))
        if(limits.machiningAcceleration[i]>0.0||limits.machiningDeceleration[i]>0.0||
           limits.flushAcceleration[i]>0.0||limits.flushDeceleration[i]>0.0)return true;
    return false;
}
}
bool MotionCore::G76DropoutOutputScopeCurrent() const noexcept
{
    return m_g76Dropout.active && m_g76Dropout.session == m_g76SphereFrozen.request.session &&
        G76LineTransportScopeCurrent();
}

bool MotionCore::FreezeG76SphereMemberCommands() noexcept
{
    // The frame already submitted before the miss is outside software's control.
    // Fence the next image immediately, even when the invalid cycle skips Motion.
    const bool zeroed=ZeroAllServoTargetVelocityForFrame();
    if(!zeroed||!G76DropoutOutputScopeCurrent()||!m_pContexts||m_pContexts->size()<3U||
       m_Group.axisCount!=3||m_g76SphereFrozen.axisMask!=7U)return false;
    for(unsigned i=0U;i<3U;++i){
        const auto& axis=(*m_pContexts)[i];
        if(m_Group.axisIndices[i]!=static_cast<int>(i)||axis.axisIndex!=static_cast<int>(i)||
           !axis.isExist||axis.isVirtualAxis||!axis.isServoOn||axis.isFault||axis.isLagAlarm||
           axis.homeRuntime.active||axis.state!=MotionState::MotionState_INTERPOLATING||
           !std::isfinite(axis.currentCmdPos)||!std::isfinite(axis.logicalCmdPos)||
           !std::isfinite(axis.currentCmdVel)||!std::isfinite(axis.logicalCmdVel)||
           !std::isfinite(axis.targetVelocity)||!std::isfinite(axis.targetEndVel)||
           !std::isfinite(axis.pid.integralAcc)||!std::isfinite(axis.pid.prevError))return false;
    }
    for(unsigned i=0U;i<3U;++i){
        auto& axis=(*m_pContexts)[i];
        axis.logicalCmdVel=axis.currentCmdVel=axis.targetVelocity=axis.targetEndVel=0.0;
        axis.pid.integralAcc=axis.pid.prevError=0.0;
    }
    return true;
}

bool MotionCore::G76SphereDropoutStandstill() noexcept
{
    auto& pause=m_g76Dropout;
    const auto& input=m_g76SphereInput;
    const auto& frozen=m_g76SphereFrozen;
    const auto tick=m_ncSettleRuntimeCycleTick;
    const auto resetDwell=[&]() noexcept {
        pause.lastStandstillTick=0ULL;pause.standstillSinceUs=0ULL;pause.standstillSamples=0U;
    };
    if(!G76DropoutOutputScopeCurrent()||!m_ncSettleRuntimeCycleValid||!m_ncSettleRuntimeCycleContiguous||
       !input.pdoValid||!input.sourceValid||input.tick!=tick||input.physicalSequence!=tick||!input.nowUs||
       !MotionG80RuntimeProof::SameInputScope(m_g76SphereInputSession,m_g76SphereInputIdentity,
           frozen.request.session,frozen.command.execution)||
       (m_g76SphereInputObservation.validAxisMask&7U)!=7U||!tick||!pause.zeroSentTick||
       pause.zeroSentTick!=tick-1ULL||!ArePhysicalGroupAxesReadyForTerminal()){
        resetDwell();return false;
    }
    bool withinRange=pause.lastStandstillTick!=0ULL&&pause.lastStandstillTick==tick-1ULL;
    const auto& command=m_g76SphereCycle.Command().commandMCS;
    for(unsigned i=0U;i<3U;++i){
        const auto& axis=(*m_pContexts)[i];
        const double ppu=frozen.pulsePerNative[i];
        if(!SpherePositive(ppu)){resetDwell();return false;}
        const double window=(std::min)(axis.inPositionWindow_Pulse/ppu,frozen.request.positionToleranceMm);
        const double actual=input.actualMCS[i];
        if(!SpherePositive(window)||!std::isfinite(command[i])||!std::isfinite(actual)||
           std::fabs(actual-command[i])>window||!std::isfinite(axis.currentActVel)||
           std::fabs(axis.currentActVel)>1.0/CYCLE_TIME_SEC){resetDwell();return false;}
        // The ordinary settle range bounds total peak-to-peak travel. Keeping
        // every sample within half that range of the first prevents slow drift.
        const double range=(std::min)(0.5*axis.inPositionWindow_Pulse,
            (std::max)(4.0,0.25*axis.inPositionWindow_Pulse))/ppu;
        if(std::fabs(actual-pause.lastActualMCS[i])>0.5*range)withinRange=false;
    }
    if(!withinRange){
        pause.lastActualMCS=input.actualMCS;pause.standstillSinceUs=input.nowUs;pause.standstillSamples=0U;
    }
    pause.lastStandstillTick=tick;
    if(input.nowUs<pause.standstillSinceUs||pause.standstillSamples==(std::numeric_limits<std::uint32_t>::max)()){
        resetDwell();return false;
    }
    ++pause.standstillSamples;
    const auto samples=(std::max)(200U,frozen.request.settleSamples);
    const double seconds=(std::max)(0.05,frozen.request.minimumSettleSeconds);
    return pause.standstillSamples>=samples&&static_cast<double>(input.nowUs-pause.standstillSinceUs)>=seconds*1000000.0;
}

void MotionCore::ObserveG76SphereFinalVelocity(unsigned axis,double hardwareVelocity) noexcept
{
    if(!m_g76SphereRunning||axis>=6U||!(m_g76SphereFrozen.axisMask&(1U<<axis))||
       !SphereHasModeCaps385(m_g76SphereProcessFrozen.config.flushProfile.axisModeLimits,m_g76SphereFrozen.axisMask))return;
    auto& guard=m_g76SphereFinalGuard;
    if(guard.session!=m_g76SphereFrozen.request.session)return;
    if(guard.proposedTick!=m_ncSettleRuntimeCycleTick){guard.proposedTick=m_ncSettleRuntimeCycleTick;guard.proposedMask=0U;}
    guard.proposed[axis]=hardwareVelocity;guard.proposedMask|=1U<<axis;
}
bool MotionCore::GuardG76SphereOutputImage(const std::array<std::int32_t,MAX_AXES>& velocity,
    std::uint32_t slotMask,std::uint64_t tick,std::uint64_t frameGeneration) noexcept
{
    if(!m_g76SphereRunning)return !tick||m_g76SphereFinalGuard.faultedTick!=tick;
    const auto& frozen=m_g76SphereFrozen;const auto& limits=m_g76SphereProcessFrozen.config.flushProfile.axisModeLimits;
    // A malformed all-NaN/negative row cannot masquerade as an inactive pair.
    if(!EDM385::Valid(limits)){
        if(m_safetyControlledStopInProgress||HasPendingSafetyOrRecoveryRequests()||IsG76SphereResetOutputHeld())return true;
        FailG76SphereRuntime(3U,true);return false;
    }
    if(!SphereHasModeCaps385(limits,frozen.axisMask))return true;
    // Existing safety owner and exact RESET hold control their own stop law.
    // A normal machining/flush ceiling never weakens or obstructs that law.
    if(m_safetyControlledStopInProgress||HasPendingSafetyOrRecoveryRequests()||IsG76SphereResetOutputHeld())return true;
    const auto fail=[&]() noexcept {FailG76SphereRuntime(3U,true);return false;};
    auto& guard=m_g76SphereFinalGuard;
    if(!m_g76SphereProcessFrozen.config.flushAxisLimitsEnabled||
       guard.session!=frozen.request.session||!tick||!frameGeneration||tick!=m_ncSettleRuntimeCycleTick||
       !m_ncSettleRuntimeCycleValid||!m_ncSettleRuntimeCycleContiguous||
       guard.proposedTick!=tick||(guard.proposedMask&frozen.axisMask)!=frozen.axisMask||
       (slotMask&frozen.axisMask)!=frozen.axisMask||!m_Group.isActive||!m_Group.currentCmd.g76SphereAir||
       !SphereIdentity(m_Group.currentCmd.execution,frozen.command.execution)||
       !m_Group.currentCmd.ownerLease.Matches(frozen.command.ownerLease)||
       GetCurrentExecutionEpoch()!=frozen.command.execution.epoch||!IsMotionOwnerLeaseCurrent(frozen.command.ownerLease)||
       m_g76SphereCycle.Current().tick!=tick)return fail();
    const bool flushing=m_g76SphereCycle.UsingFlushAxisLimits();
    for(unsigned axis=0U;axis<6U;++axis){
        if(!(frozen.axisMask&(1U<<axis)))continue;
        const double next=guard.proposed[axis];
        // No repeating one-PPS allowance: enforce continuous pre-serialization
        // delta, then prove the complete image is exactly its truncation. The
        // integer PDO differs by strictly less than one PPS at each sample.
        if(!std::isfinite(next)||next<(std::numeric_limits<std::int32_t>::min)()||
           next>(std::numeric_limits<std::int32_t>::max)()||
           velocity[axis]!=static_cast<std::int32_t>(next))return fail();
        const double requestedA=flushing?limits.flushAcceleration[axis]:limits.machiningAcceleration[axis];
        const double requestedD=flushing?limits.flushDeceleration[axis]:limits.machiningDeceleration[axis];
        if(requestedA==0.0&&requestedD==0.0)continue;
        if(!guard.sentTick){if(next!=0.0)return fail();continue;}
        if(guard.sentTick+1ULL!=tick)return fail();
        const auto& b=frozen.axis[axis];const double ppu=frozen.pulsePerNative[axis];
        const auto& process=m_g76SphereProcessFrozen.config;
        const double rawA=frozen.signedMotionEnabled?process.flushAxisAccelerationMmS2[axis]*ppu:
            b.maxVelocity/(b.accSeconds<0.001?0.2:b.accSeconds);
        const double rawD=frozen.signedMotionEnabled?process.flushAxisDecelerationMmS2[axis]*ppu:
            b.maxVelocity/(std::max)(b.decSeconds<0.001?0.2:b.decSeconds,b.stopSeconds<0.001?0.2:b.stopSeconds);
        double a=0.0,d=0.0;
        if(!SpherePulseCeiling385(rawA,requestedA,ppu,a)||!SpherePulseCeiling385(rawD,requestedD,ppu,d))return fail();
        const double prior=guard.sent[axis],delta=next-prior;
        const bool reverses=(prior<0.0&&next>0.0)||(prior>0.0&&next<0.0);
        const double selected=reverses?(std::min)(a,d):std::fabs(next)>std::fabs(prior)?a:d;
        const double bound=selected*CYCLE_TIME_SEC;
        const double scale=(std::max)(1.0,(std::max)(std::fabs(next),std::fabs(prior)));
        const double roundoff=16.0*(std::numeric_limits<double>::epsilon)()*scale;
        if(!SpherePositive(ppu)||!SpherePositive(a)||!SpherePositive(d)||!SpherePositive(bound)||
           !std::isfinite(delta)||std::fabs(delta)>bound*(1.0+1.e-10)+roundoff)return fail();
    }
    for(unsigned axis=0U;axis<6U;++axis)guard.serialized[axis]=velocity[axis];
    guard.validated=guard.proposed;guard.validatedTick=tick;guard.validatedFrame=frameGeneration;
    return true;
}

void MotionCore::ObserveG76SphereSend(const MotionServoHandoffSample& sample) noexcept
{
    // EDM386_G01_RESPONSE_TRACE_BEGIN
    // Observe zeros/failures too, before any authoritative send-credit gates.
    if(m_g01ResponseTrace.ActiveSession()==m_g76SphereFrozen.request.session){
        if(auto* response=m_g01ResponseTrace.Pending(sample.sourceTick)){
            response->stages|=EDMG01ResponseTrace::Send;
            response->txSequence=sample.sequence;response->txSourceTick=sample.sourceTick;
            response->txFrame=sample.frameGeneration;
            response->txProposedPps=sample.proposedVelocity[2];response->txCopiedPps=sample.copiedVelocity[2];
            response->sendFlags=(sample.attempted?EDMG01ResponseTrace::Attempted:0U)|
                (sample.apiAccepted?EDMG01ResponseTrace::ApiAccepted:0U)|
                (sample.finalChecked?EDMG01ResponseTrace::FinalChecked:0U)|
                (sample.scrubbed?EDMG01ResponseTrace::Scrubbed:0U)|
                (sample.idleScope?EDMG01ResponseTrace::Idle:0U);
            const auto& command=m_g76SphereFrozen.command;
            if(sample.epoch==command.execution.epoch&&sample.owner==static_cast<std::uint32_t>(command.ownerLease.owner)&&
               sample.ownerGeneration==command.ownerLease.generation&&
               (sample.slotMask&7U)==7U&&(sample.observedMask&7U)==7U&&(sample.sourceInputMask&7U)==7U)
                response->sendFlags|=EDMG01ResponseTrace::Scoped;
        }
    }
    // EDM386_G01_RESPONSE_TRACE_END
    if(!m_g76SphereRunning)return;
    const auto& command=m_g76SphereFrozen.command;
    if(m_g76Dropout.active){
        auto& pause=m_g76Dropout;
        const auto tick=m_ncSettleRuntimeCycleTick;
        const bool wireZero=pause.wireZeroSequence&&sample.sequence==pause.wireZeroSequence;
        pause.wireZeroSequence=0ULL;
        pause.zeroSentTick=0ULL;m_g76SphereProcessSentTick=0ULL;
        if(!wireZero)return;
        if(!G76DropoutOutputScopeCurrent()||!m_ncSettleRuntimeCycleValid||!m_ncSettleRuntimeCycleContiguous||
           !tick||pause.zeroPreparedTick!=tick||!sample.frameGeneration||sample.frameGeneration<=pause.zeroFrame||
           !sample.sequence||sample.sequence<=pause.zeroSequence||sample.idleScope||
           !sample.slotMask||sample.observedMask!=sample.slotMask||
           !MotionG80RuntimeProof::ExactZeroSend(sample,command.execution.epoch,command.ownerLease,
               m_g76SphereFrozen.axisMask,tick))return;
        for(unsigned i=0U;i<sample.copiedVelocity.size();++i){
            if((sample.slotMask&(1U<<i))&&(sample.proposedVelocity[i]||sample.copiedVelocity[i]))return;
        }
        pause.zeroSentTick=tick;pause.zeroFrame=sample.frameGeneration;pause.zeroSequence=sample.sequence;
        // Rebase the EDM385 acceleration history only on an actual NIC handoff.
        // The next observer must also match this frame's complete LRW response.
        auto& guard=m_g76SphereFinalGuard;
        guard.sent.fill(0.0);guard.sentTick=tick;guard.sentSequence=sample.sequence;
        return;
    }
    m_g76SphereProcessSentTick=0ULL;
    bool scoped=m_Group.isActive&&SphereIdentity(m_Group.currentCmd.execution,command.execution)&&
        m_Group.currentCmd.ownerLease.Matches(command.ownerLease)&&IsMotionOwnerLeaseCurrent(command.ownerLease)&&
        GetCurrentExecutionEpoch()==command.execution.epoch&&!HasPendingSafetyOrRecoveryRequests()&&
        m_g76SphereCycle.Current().tick==m_ncSettleRuntimeCycleTick;
    const auto& nominal=m_g76SphereCycle.Command();
    std::uint32_t nominalMovingMask=0U;
    for(unsigned axis=0U;scoped&&axis<6U;++axis){
        if(!(m_g76SphereFrozen.axisMask&(1U<<axis)))continue;
        if(nominal.velocityNative[axis]!=0.0)nominalMovingMask|=1U<<axis;
        // Non-line parents still own stationary auxiliary coordinates.
        if(axis>=3U&&m_g76SphereFrozen.request.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line)
            scoped=nominal.velocityNative[axis]==0.0&&nominal.accelerationNative[axis]==0.0&&
                MotionG76SphereFixedAxes::SameBits(nominal.commandMCS[axis],m_g76SphereFrozen.request.geometry.startMCS[axis]);
    }
    auto& finalGuard=m_g76SphereFinalGuard;
    if(scoped&&finalGuard.session==m_g76SphereFrozen.request.session&&
       finalGuard.validatedTick==sample.sourceTick&&finalGuard.validatedFrame==sample.frameGeneration&&
       !sample.idleScope&&sample.sequence>finalGuard.sentSequence&&
       (MotionG80RuntimeProof::ExactZeroSend(sample,command.execution.epoch,command.ownerLease,
            m_g76SphereFrozen.axisMask,m_ncSettleRuntimeCycleTick)||
        MotionG76SphereProcessSend::ExactNonzeroSend(sample,command.execution.epoch,command.ownerLease,
            m_g76SphereFrozen.axisMask,m_ncSettleRuntimeCycleTick,false,m_g76SphereFrozen.axisMask))){
        bool exact=true;
        for(unsigned axis=0U;axis<6U;++axis)if(m_g76SphereFrozen.axisMask&(1U<<axis))
            exact=exact&&sample.copiedVelocity[axis]==finalGuard.serialized[axis];
        if(exact){finalGuard.sent=finalGuard.validated;finalGuard.sentTick=sample.sourceTick;finalGuard.sentSequence=sample.sequence;}
    }
    if(scoped&&sample.sequence>m_g76SphereProcessSendSequence&&sample.frameGeneration>m_g76SphereProcessSendFrame&&
       MotionG76SphereProcessSend::ExactNonzeroSend(sample,command.execution.epoch,command.ownerLease,
           m_g76SphereFrozen.axisMask,m_ncSettleRuntimeCycleTick,m_g76SphereFrozen.request.physicalZManual,nominalMovingMask)){
        m_g76SphereProcessSentTick=sample.sourceTick;m_g76SphereProcessSendSequence=sample.sequence;
        m_g76SphereProcessSendFrame=sample.frameGeneration;
    }
    if(!m_g76SphereZeroDemand)return;
    const bool exact=MotionG80RuntimeProof::ExactZeroSend(sample,command.execution.epoch,command.ownerLease,
        m_g76SphereFrozen.axisMask,m_ncSettleRuntimeCycleTick);
    m_g76SphereZeroSentTick=exact?sample.sourceTick:0ULL;
}

bool MotionCore::G76SphereDemandsZero(const AxisContext& axis) const noexcept
{
    return m_g76SphereRunning&&(m_g76SphereZeroDemand||m_g76Dropout.active)&&axis.axisIndex>=0&&axis.axisIndex<6&&
        (m_g76SphereFrozen.axisMask&(1U<<static_cast<unsigned>(axis.axisIndex)))!=0U;
}

// EDM386_G01_RESPONSE_TRACE_BEGIN
void MotionCore::ObserveG01ResponseControl(unsigned axis,double servoCommandPulse,double servoReferencePps,
    double actualPulse,double posDiffPps,std::int32_t rawPosition,std::uint32_t status,std::int32_t mode,
    double pTerm,double iTerm,double kp,double ki,double kvff,double hardwareProposalPps,
    std::int32_t localCsvPps) noexcept
{
    if(axis!=2U||!m_g76SphereRunning||
       m_g01ResponseTrace.ActiveSession()!=m_g76SphereFrozen.request.session)return;
    auto* response=m_g01ResponseTrace.Pending(m_ncSettleRuntimeCycleTick);
    if(!response)return;
    response->stages|=EDMG01ResponseTrace::Control|EDMG01ResponseTrace::LocalImage;
    if((*m_pContexts)[2].isReverse!=(*m_pContexts)[2].Axis_Reverse)
        response->flags|=EDMG01ResponseTrace::HardwareReverse;
    response->servoCommandPulse=servoCommandPulse;response->servoReferencePps=servoReferencePps;
    response->actualPulse=actualPulse;response->posDiffPps=posDiffPps;
    response->rawActualPosition=rawPosition;response->sourceStatus=status;response->sourceMode=mode;
    response->pTerm=pTerm;response->iTerm=iTerm;response->kp=kp;response->ki=ki;response->kvff=kvff;
    response->hardwareProposalPps=hardwareProposalPps;response->localCsvPps=localCsvPps;
}
// EDM386_G01_RESPONSE_TRACE_END
void MotionCore::PublishG76SphereRuntime() noexcept
{
    m_g76SphereCurrent.cycle=m_g76SphereCycle.Current();
    m_g76SphereCurrent.timedCompletionEndpoint=m_g76SphereCycle.TimedCompletionEndpoint();
    if(m_g76SphereCurrent.terminal&&!m_g76SphereCurrent.completed)
        SphereWithdrawOrbitCompletion(m_g76SphereCurrent.cycle);
    m_g76SphereCurrent.holdRetreatAnchorMCS=m_g76SphereCycle.HoldRetreatAnchor();
    m_g76SphereCurrent.holdInspectionAxisMask=m_g76SphereInspectionTravelMask;
    m_g76SphereCurrent.holdProcessPointMCS=m_g76SphereCycle.HoldProcessPoint();
    const auto collisionRequested=m_collisionConfigurationRequested.load(std::memory_order_acquire);
    const bool collisionReady=collisionRequested!=0ULL&&m_collisionRuntimeConfiguration==collisionRequested&&
        m_collisionConfigurationValid&&m_collisionCurrent.valid&&
        (!m_collisionCurrent.shortActive||m_collisionManualBypass.load(std::memory_order_acquire));
    m_g76SphereCurrent.holdInspectionJogReady=m_g76SphereCycle.Current().holdRecoveryEnabled&&
        m_g76SphereCycle.CanInspectionJog()&&m_g76SphereInspectionTravelMask==7U&&
        collisionReady&&!m_g76SphereInspectionBlocked;
    m_g76SphereCurrent.holdRecoveryResumeReady=m_g76SphereCycle.Current().holdRecoveryEnabled&&
        m_g76SphereCycle.Current().held&&m_g76SphereCycle.Current().inspectionTraceValid&&
        m_g76SphereCycle.Current().phase==NCG76SphereCycle::Phase::HeldAtStart&&
        !m_g76SphereInspectionBlocked&&
        (m_g76SphereCycle.Current().inspectionTraceCount==0U||collisionReady);
    m_g76SphereCurrent.processShadow=m_g76SphereProcess.Current();
    if(m_g76SphereCurrent.processShadow.session!=m_g76SphereCurrent.session||
       m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==m_g76SphereCurrent.session)
        m_g76SphereCurrent.processShadow=NCG76SphereGapProcess::Feedback{};
    m_g76SphereCurrent.endpointMCS=m_g76SphereCycle.Command().commandMCS;
    if(!m_g76SphereCurrent.firstInputFault.valid){
        m_g76SphereCurrent.commandMCS=m_g76SphereCycle.Command().commandMCS;
        m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
        m_g76SphereCurrent.inputSequence=m_g76SphereInput.physicalSequence;
        m_g76SphereCurrent.inputValidAxisMask=m_g76SphereInputObservation.validAxisMask;
    }
    m_g76SphereCurrent.velocityNative=m_g76SphereCycle.Command().velocityNative;
    if (m_g76Dropout.active) m_g76SphereCurrent.velocityNative.fill(0.0);
    m_g76SphereCurrent.positionToleranceMm=m_g76SphereFrozen.request.positionToleranceMm;
    m_g76SphereCurrent.rotaryToleranceDeg=m_g76SphereFrozen.request.rotaryToleranceDeg;
    m_g76SphereCurrent.translationGeneration=m_g76SphereFrozen.request.geometry.frame.translation.generation;
    m_g76SphereCurrent.nominalKind=m_g76SphereFrozen.request.geometry.kind;
    if(!m_g76SphereFeedback.Publish(m_g76SphereCurrent)&&m_g76SphereRunning){
        m_g76SphereZeroDemand=true;EmergencyStopAllAxesImpl(false);
    }
}

bool MotionCore::CommitG76SphereCompletedEndpoint(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::array<double,8U>& endpointMCS) noexcept
{
    MotionG76SphereFeedback report{};
    if(!ReadG76SphereFeedback(report)||!report.completed||!report.terminal||report.session!=session||
        !SphereIdentity(report.identity,identity)||!report.owner.Matches(owner)||
        m_g76SphereClaim.load(std::memory_order_acquire)!=session||
        m_g76SphereBarrier.load(std::memory_order_acquire)!=session||
        !HasExactExecutionDrainAcknowledgement(identity.epoch,owner))return false;
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired()||!HasExactExecutionDrainAcknowledgementImpl(identity.epoch,owner,true))return false;
    G76SphereFrozen frozen{};
    if(!m_g76SphereRequests.Read(frozen).Ready()||frozen.request.session!=session||
       !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner))return false;
    std::uint32_t threshold=0U;std::uint8_t retry=0U;
    if(!SphereRecipeCompletionReceipt(report,frozen.request.orbitInspectionRounds,
        frozen.request.orbitThresholdScaled,frozen.request.orbitRetryLimit,threshold,retry))return false;
    frozen.request.orbitThresholdScaled=threshold;frozen.request.orbitRetryLimit=retry;
    if(!IsG76SphereCommandAuthorized(frozen.command)||report.runtimeFault||
        report.cycle.phase!=NCG76SphereCycle::Phase::Complete||report.cycle.fault!=NCG76SphereCycle::Fault::None||
        !MotionG76SphereMachiningTime::CompletedEndpointQualified(report,frozen.request)||
        !SphereOrbitCompletionFeedbackMatches(report.cycle,frozen.request.orbitInspectionRounds,
            frozen.request.orbitThresholdScaled,true,frozen.request.orbitRetryLimit)||
        !MotionG76SphereMachiningTime::TerminalQualified(report.cycle,frozen.request))return false;
    std::array<double,6U> endpointPulse{};
    for(unsigned i=0U;i<SphereAxisCount(frozen.request);++i){
        if(!SphereAxisPresent(frozen.request,i))continue;
        double pulse=report.endpointMCS[i]*frozen.pulsePerNative[i];
        const int slot=SphereCommandSlot(frozen.command,i);
        if(slot<0||(i>=3U&&!MotionG76SphereFixedAxes::CommandPulseRelative(report.endpointMCS[i],
            frozen.request.geometry.startMCS[i],frozen.command.mem_startPos[slot],frozen.pulsePerNative[i],pulse,
                NCG76SphereRuntimeGeometry::NativeAxisErrorBound(frozen.request.geometry,i))))return false;
        if(!std::isfinite(pulse)||(i<3U?(*m_pContexts)[i].logicalCmdPos.Load()!=pulse:
            !MotionG76SphereFixedAxes::SameBits((*m_pContexts)[i].logicalCmdPos.Load(),pulse)))return false;
        endpointPulse[i]=pulse;
    }
    for(unsigned i=0U;i<SphereAxisCount(frozen.request);++i){
        if(!SphereAxisPresent(frozen.request,i))continue;
        const double pulse=endpointPulse[i];
        m_g00ProducerQueueTailPulse[i]=pulse;(*m_pContexts)[i].lastQueuedPulse.Store(pulse);
        m_g00ProducerQueueTailValidMask|=1U<<i;
    }
    m_g00ProducerQueueTailEpoch=identity.epoch;m_g00ProducerQueueTailOwnerLease=owner;
    endpointMCS=report.endpointMCS;m_g76SphereEndpointCommitted.store(session,std::memory_order_release);return true;
}

bool MotionCore::IsG76SphereBarrierBlocking(const MotionCommand& command) const noexcept
{
    const auto session=m_g76SphereBarrier.load(std::memory_order_acquire);
    if(!session||(command.execution.source!=MotionCommandSource::NC_MEMORY&&command.execution.source!=MotionCommandSource::NC_MDI))return false;
    G76SphereFrozen frozen{};
    if(!m_g76SphereRequests.Read(frozen).Ready())return true;
    return frozen.request.session==session&&command.execution.epoch==frozen.command.execution.epoch&&
        command.ownerLease.Matches(frozen.command.ownerLease);
}

bool MotionCore::ReleaseG76SphereBarrier(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner) noexcept
{
    MotionG76SphereFeedback report{};
    if(!ReadG76SphereFeedback(report)||!report.completed||!report.terminal||!report.endpointCommitted||
        report.session!=session||!SphereIdentity(report.identity,identity)||!report.owner.Matches(owner)||
        !HasExactExecutionDrainAcknowledgement(identity.epoch,owner))return false;
    G76SphereHoldFrozen hold{};
    if(!m_g76SphereHoldRequests.Read(hold).Ready()||hold.session!=session||
       !SphereIdentity(hold.identity,identity)||!hold.owner.Matches(owner))return false;
    std::uint32_t threshold=0U;std::uint8_t retry=0U;
    if(!SphereRecipeCompletionReceipt(report,hold.orbitInspectionRounds,hold.orbitThresholdScaled,
        hold.orbitRetryLimit,threshold,retry)||
       !SphereOrbitCompletionFeedbackMatches(report.cycle,hold.orbitInspectionRounds,threshold,true,
            retry)||report.cycle.totalLimitUs!=hold.machiningTime.totalLimitUs||
       report.cycle.additionalUs!=hold.machiningTime.additionalUs)return false;
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired()||!HasExactExecutionDrainAcknowledgementImpl(identity.epoch,owner,true))return false;
    auto expected=session;
    if(!m_g76SphereBarrier.compare_exchange_strong(expected,0ULL,
        std::memory_order_acq_rel,std::memory_order_acquire))return false;
    if(report.cycle.holdRecoveryEnabled){
        m_g76SphereHoldReleaseFeedbackSequence.store(
            m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire),std::memory_order_relaxed);
        m_g76SphereHoldContinuationReleased.store(session,std::memory_order_release);
    }
    if(report.nominalKind==NCG76SphereRuntimeGeometry::Kind::Line||
        report.nominalKind==NCG76SphereRuntimeGeometry::Kind::Arc)
        m_g76SphereContinuationReleased.store(session,std::memory_order_release);
    return true;
}
