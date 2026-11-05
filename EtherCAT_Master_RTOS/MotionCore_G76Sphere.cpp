#include "MotionCore.h"
#include "AlarmManager.h"
#include "PLCManager.h"
#include "NCG76StartDiagnostic.h"
#include "NCG76SphereSimulationTestPolicy.h"
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
void SphereWithdrawOrbitCompletion(NCG76SphereCycle::Feedback& feedback) noexcept
{
    if(feedback.orbitRequiredPasses){
        feedback.orbitCompletionState=5U;feedback.orbitCompletionActive=false;
        feedback.orbitPercentAvailable=false;feedback.orbitPercentScaled=0U;feedback.orbitQualifiedPasses=0U;
    }
}
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
        !NCG76SphereCycle::ValidateRequest(requested)||!SpherePowerOff())return reject(MotionG76SphereAdmissionGate::Request);
    if(!SameNCTranslationSnapshot(requested.geometry.frame.translation,frame)||!MatchesNCTranslation(frame)||
        !IsNCTranslationSnapshotValid(frame)||!IsNCTranslationAxisIdentityCurrent(frame))return reject(MotionG76SphereAdmissionGate::Frame);
    if(!m_CompEngine.IsSealed())return reject(MotionG76SphereAdmissionGate::Compensation);
    if(owner.owner!=MotionOwner::AUTO||
        !HasExactExecutionDrainAcknowledgement(epoch,owner)||m_pContexts==nullptr||m_pCoordMgr==nullptr||
        m_pContexts->size()<6U||GetQueueSize()!=0U||GetCommandIngressSize()!=0U||GetCommandReplaySize()!=0U)
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
        processShadow->orbitScaleMmPerVoltMin!=0.0||processShadow->orbitOverridePercent!=0.0))
        return reject(MotionG76SphereAdmissionGate::FlushEnvelope);
    if(!MotionG76SphereParameterFeeds::Matches(request,process.config))
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
    const bool stationaryLine=request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&request.geometry.travelMm==0.0;
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
    if(request.retreatMode!=0U&&(!process.config.enabled||
        (!stationaryLine&&!process.config.flushMotionEnabled)||!frozen.signedMotionEnabled))
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
    for(unsigned i=0U;i<6U;++i){
        const auto& axis=(*m_pContexts)[i];const double ppu=axis.resolution_PPR/axis.finalLead;
        if(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||!SpherePositive(ppu)||
            !SpherePositive(axis.maxVel_PPS)||axis.maxVel_PPS>static_cast<double>((std::numeric_limits<std::int32_t>::max)())/1.2||
            !SpherePositive(axis.inPositionWindow_Pulse)||!SpherePositive(axis.resolution_PPR)||!SpherePositive(axis.finalLead)||
            (axis.axisType==AxisType::ROTARY&&!SpherePositive(axis.rotaryModulo))||
            (i<3U?axis.axisType!=AxisType::LINEAR:axis.axisType==AxisType::LINEAR)||
            !std::isfinite(axis.G00_acc_time)||axis.G00_acc_time<0.0||
            !std::isfinite(axis.G00_dec_time)||axis.G00_dec_time<0.0||
            !std::isfinite(axis.Stop_dec_time)||axis.Stop_dec_time<0.0)return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
        auto& binding=frozen.axis[i];
        binding.resolution=axis.resolution_PPR;binding.lead=axis.finalLead;binding.maxVelocity=axis.maxVel_PPS;
        binding.accSeconds=axis.G00_acc_time;binding.decSeconds=axis.G00_dec_time;binding.stopSeconds=axis.Stop_dec_time;
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
        if(i<3U){
            const double accTime=axis.G00_acc_time<0.001?0.2:axis.G00_acc_time;
            const double decTime=(std::max)(axis.G00_dec_time<0.001?0.2:axis.G00_dec_time,
                axis.Stop_dec_time<0.001?0.2:axis.Stop_dec_time);
            if(!SpherePositive(accTime)||!SpherePositive(decTime))return reject(MotionG76SphereAdmissionGate::AxisConfiguration,static_cast<int>(i));
            rampSeconds=(std::max)(rampSeconds,accTime);stopSeconds=(std::max)(stopSeconds,decTime);
            const double vmax=axis.maxVel_PPS/ppu,acc=vmax/accTime,dec=vmax/decTime;
            request.velocityCapMmS=(std::min)(request.velocityCapMmS,vmax);
            // Reserve half each installed axis acceleration for curvature.
            // |tangent_i|<=1, a_i=t_i*a + curvature_i*v^2.
            request.accelerationMmS2=(std::min)(request.accelerationMmS2,0.5*acc);
            request.decelerationMmS2=(std::min)(request.decelerationMmS2,0.5*dec);
            const double curvature=request.geometry.maxCircleCurvatureAbs[i];
            if(!std::isfinite(curvature)||curvature<0.0)return reject(MotionG76SphereAdmissionGate::RuntimeRequest,static_cast<int>(i));
            if(curvature>0.0)request.velocityCapMmS=(std::min)(request.velocityCapMmS,
                std::sqrt(0.5*(std::min)(acc,dec)/curvature));
            request.positionToleranceMm=(std::min)(request.positionToleranceMm,axis.inPositionWindow_Pulse/ppu);
        }else request.rotaryToleranceDeg=(std::min)(request.rotaryToleranceDeg,axis.inPositionWindow_Pulse/ppu);
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
        const bool baseline=i<3U?
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
    // All physical guards use the stricter installed acceleration/deceleration
    // limit. Apply that same symmetric budget to straight and HOLD return legs.
    const double commonRamp=(std::min)(request.accelerationMmS2,request.decelerationMmS2);
    request.accelerationMmS2=commonRamp;request.decelerationMmS2=commonRamp;
    if(command.axisCount!=6||!NCG76SphereCycle::ValidateRequest(request))return reject(MotionG76SphereAdmissionGate::RuntimeRequest);
    if(flush.enabled&&!NCG76SphereCycle::ValidateFlushConfig(request,flush))
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
        m_pContexts==nullptr||m_pContexts->size()<6U)return false;
    G76SphereFrozen frozen{};
    if(!m_g76SphereRequests.Read(frozen).Ready()||!frozen.request.session||
        m_g76SphereClaim.load(std::memory_order_acquire)!=frozen.request.session||
        std::memcmp(&command,&frozen.command,sizeof(command))!=0||frozen.axisMask!=63U||
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
        if(active.parameterFeeds!=request.parameterFeeds||active.geometry.kind!=kind||
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
    for(unsigned i=0U;i<6U;++i){
        const auto& axis=(*m_pContexts)[i];const auto& b=frozen.axis[i];
        if(!axis.isExist||axis.axisIndex!=static_cast<int>(i)||axis.resolution_PPR!=b.resolution||
            axis.finalLead!=b.lead||axis.maxVel_PPS!=b.maxVelocity||axis.G00_acc_time!=b.accSeconds||
            axis.G00_dec_time!=b.decSeconds||axis.Stop_dec_time!=b.stopSeconds||axis.machineCoordinateOffsetPulse!=b.machineOffset||
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
    if(m_pContexts&&m_pContexts->size()>=6U)flags|=Contexts;
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
    // Keep values, sequence and validity from the same pre-cancellation input.
    // Later source callbacks may reset their working sample after containment.
    m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
    m_g76SphereCurrent.commandMCS=m_g76SphereRunning?m_g76SphereCycle.Command().commandMCS:
        m_g76SphereFrozen.request.geometry.startMCS;
    m_g76SphereCurrent.inputSequence=m_g76SphereInput.physicalSequence;
    m_g76SphereCurrent.inputValidAxisMask=m_g76SphereInputObservation.validAxisMask;
}

void MotionCore::ObserveG76SphereRuntimeInput() noexcept
{
    m_g76SphereInput=NCG76SphereCycle::Input{};
    m_g76SphereInputSession=0ULL;m_g76SphereInputIdentity=MotionExecutionIdentity{};
    m_g76SphereInputObservation=MotionG76SphereInputDiagnostic{};
    if(!m_g76SphereClaim.load(std::memory_order_acquire)){
        if(m_g76SphereRunning)
            SetG76SphereInputDiagnostic(MotionG76SphereInputStage::Observe,MotionG76SphereInputGate::ClaimMissing);
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
    bool source=read&&frozen.request.session&&input.pdoValid&&input.powerInhibited&&
        m_edmZClockValid&&m_edmZClockFrequency!=0ULL&&input.nowUs&&m_pContexts!=nullptr&&m_pContexts->size()>=6U;
    MotionG76SphereInputGate gate=MotionG76SphereInputGate::None;int failedAxis=-1;
    if(!read)gate=MotionG76SphereInputGate::RequestUnavailable;
    else if(!frozen.request.session)gate=MotionG76SphereInputGate::SessionMissing;
    else if(!m_ncSettleRuntimeCycleValid)gate=MotionG76SphereInputGate::PdoCycleInvalid;
    else if(!m_ncSettleRuntimeCycleContiguous)gate=MotionG76SphereInputGate::PdoNotContiguous;
    else if(!input.powerInhibited)gate=MotionG76SphereInputGate::PowerNotInhibited;
    else if(!m_edmZClockValid)gate=MotionG76SphereInputGate::ClockInvalid;
    else if(!m_edmZClockFrequency)gate=MotionG76SphereInputGate::ClockFrequencyMissing;
    else if(!input.nowUs)gate=MotionG76SphereInputGate::ClockTimeMissing;
    else if(!m_pContexts||m_pContexts->size()<6U)gate=MotionG76SphereInputGate::ContextsMissing;
    for(unsigned i=0U;source&&i<6U;++i){
        const auto& axis=(*m_pContexts)[i];MotionServoInputSnapshot physical{};
        failedAxis=static_cast<int>(i);
        if(!axis.isExist){source=false;gate=MotionG76SphereInputGate::AxisMissing;}
        else if(axis.isFirstCycle){source=false;gate=MotionG76SphereInputGate::AxisFirstCycle;}
        else if(axis.isFault){source=false;gate=MotionG76SphereInputGate::AxisFault;}
        else if(axis.isLagAlarm){source=false;gate=MotionG76SphereInputGate::AxisLag;}
        else if(!ReadMotionServoInputCompatibilitySnapshotBySlot(static_cast<int>(i),physical)){
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
        else source=MotionG76SphereFixedAxes::FeedbackNative(frozen.request.geometry.startMCS[i],
            frozen.command.mem_startPos[i],nominal,frozen.pulsePerNative[i],input.actualMCS[i]);
        if(!source){gate=std::isfinite(nominal)?MotionG76SphereInputGate::NativeNonfinite:MotionG76SphereInputGate::NominalNonfinite;break;}
        m_g76SphereInputObservation.validAxisMask|=1U<<i;
        failedAxis=-1;
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
    if(m_g76SphereRunning){
        const auto& output=m_g76SphereCycle.Command();
        input.zeroSendTick=m_g76SphereZeroSentTick;
        input.zeroFinalSend=m_g76SphereZeroDemand&&m_g76SphereZeroSentTick&&m_g76SphereZeroSentTick+1ULL==input.tick;
        bool settled=source&&input.zeroFinalSend&&output.stopped&&ArePhysicalGroupAxesReadyForTerminal();
        const bool same=m_g76SpherePhysicalRange&&m_g76SpherePhysicalTick+1ULL==input.tick&&
            m_g76SpherePhysicalReference==output.commandMCS;
        for(unsigned i=0U;settled&&i<6U;++i){
            const auto& axis=(*m_pContexts)[i];const double ppu=frozen.pulsePerNative[i];
            const double window=(std::min)(axis.inPositionWindow_Pulse/ppu,
                i<3U?frozen.request.positionToleranceMm:frozen.request.rotaryToleranceDeg);
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
        if(error>(i<3U?request.positionToleranceMm:request.rotaryToleranceDeg))atStart=false;
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
    if(keepHoldHistory)for(unsigned i=0U;i<6U;++i){
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
               (axis<6U&&(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],low)||
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
    const bool continuationScope=!IsResetSafetyOutputHoldEstablished()&&previousSingle&&nextSingle&&previous.session&&
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
        NCG76SphereGapProcess::EquivalentForContinuation(m_g76SphereProcessFrozen.config,incoming.config);
    if(continuationScope){
        continuation.intervalWorkUs=previousProcess.intervalWorkUs;
        continuation.simulatedWorkUs=previousProcess.simulatedWorkUs;
        continuation.completedFlushes=previousProcess.completedFlushes;
        continuation.abortedFlushes=previousProcess.abortedFlushes;
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
           (axis<6U&&(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],low)||
                      !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit((*m_pContexts)[axis],high)))){
            previousParentReady=false;break;
        }
    }
    const std::uint64_t previousParentSession=previousParentReady?previous.session:0ULL;
    const std::int32_t previousParentPC=previousParentReady?previous.identity.sourceBlockId:-1;
    if(previousParentReady)m_g76SpherePreviousGeometry=previousGeometry;
    m_g76SphereCurrent=MotionG76SphereFeedback{};m_g76SphereCurrent.session=m_g76SphereFrozen.request.session;
    m_g76SphereCurrent.identity=command.execution;m_g76SphereCurrent.owner=command.ownerLease;m_g76SphereCurrent.accepted=true;
    m_g76SphereCurrent.initialIntervalWorkUs=continueProcess?continuation.intervalWorkUs:0ULL;
    m_g76SphereCurrent.completedFlushOffset=continueProcess?continuation.completedFlushes:0ULL;
    m_g76SphereCurrent.previousParentSession=previousParentSession;
    m_g76SphereCurrent.previousParentPC=previousParentPC;
    TrackMotionCommandAccepted(command);m_g76SphereCycle.Reset();
    m_g76SphereProcessFrozen=processRead?incoming:G76SphereProcessFrozen{};
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
    const bool processReady=m_g76SphereProcess.Begin(processScope?process.config:NCG76SphereGapProcess::Config{},
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
         NCG76OrbitContour::ModeP(m_g76SphereFrozen.request.geometry)==27U)?&m_g76SphereFrozen.request.geometry:nullptr);
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
        (!m_g76SphereFrozen.signedMotionEnabled||!processScope||!processReady||
         !process.config.enabled||(!process.config.flushMotionEnabled&&
             !(m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
               m_g76SphereFrozen.request.geometry.travelMm==0.0))))||
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
    if(!m_g76SphereCycle.Begin(m_g76SphereFrozen.request,m_g76SphereInput,flush.enabled?&flush:nullptr,&gap)||
       (m_g76SphereHoldFrozen.enabled&&!m_g76SphereCycle.ConfigureHoldRecoveryAdmitted(m_g76SphereHoldFrozen.config,
            m_g76SphereHoldFrozen.declaredPreviousParents,m_g76SphereHoldHistory.data(),m_g76SphereHoldHistoryCount))||
       !m_g76SphereCycle.ConfigureFinishVerification(m_g76SphereHoldFrozen.finishRequiredCount)){
        m_g76SphereProcess.Revoke();
        m_g76SphereCurrent.runtimeFault=3U;m_g76SphereCurrent.terminal=true;
        loadReservation.Release();FaultTrackedMotionCommand(SphereRuntimeAlarm,MotionRejectReason::NOT_READY);PublishG76SphereRuntime();
        AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,command.sourceLinePC);RequestEmergencyStopAllAxes();return false;
    }
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
    const bool cancelled=GetCurrentExecutionEpoch()!=m_g76SphereFrozen.command.execution.epoch||
        !IsMotionOwnerLeaseCurrent(m_g76SphereFrozen.command.ownerLease)||HasPendingSafetyOrRecoveryRequests();
    if(fault==1U)CaptureG76SphereInputFault();
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
        m_pathHold.sourceSeen||m_Group.axisCount!=6||m_Group.pathMode!=PathMode::EXACT_STOP||
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
        for(unsigned i=0U;atHeldCommand&&i<6U;++i){
            const auto& axis=(*m_pContexts)[i];const double ppu=m_g76SphereFrozen.pulsePerNative[i];
            const double tolerance=(std::min)(axis.inPositionWindow_Pulse/ppu,i<3U?
                m_g76SphereFrozen.request.positionToleranceMm:m_g76SphereFrozen.request.rotaryToleranceDeg);
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
       (previousNominal.timeLimitReached&&!MotionG76SphereMachiningTime::TimeLimitReachedQualified(previousNominal,m_g76SphereFrozen.request))||
       previousNominal.finishRequiredCount!=m_g76SphereHoldFrozen.finishRequiredCount||
       m_g76SphereHoldFrozen.orbitInspectionRounds!=m_g76SphereFrozen.request.orbitInspectionRounds||
       m_g76SphereHoldFrozen.orbitThresholdScaled!=m_g76SphereFrozen.request.orbitThresholdScaled||
       m_g76SphereHoldFrozen.orbitRetryLimit!=m_g76SphereFrozen.request.orbitRetryLimit||
       !SphereOrbitCompletionFeedbackMatches(previousNominal,m_g76SphereFrozen.request.orbitInspectionRounds,
            m_g76SphereFrozen.request.orbitThresholdScaled,false,m_g76SphereFrozen.request.orbitRetryLimit)||
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
    if(m_g76SphereFrozen.request.retreatMode!=0U&&(!signedMotion||
        !processFrozen.config.enabled||(!processFrozen.config.flushMotionEnabled&&
            !(m_g76SphereFrozen.request.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
              m_g76SphereFrozen.request.geometry.travelMm==0.0)))){
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
    // The observer proposes a request only. The pure parent owns braking,
    // fresh-zero settle, the stopped checkpoint, excursion and path restore.
    input.flushRequestSequence=processCurrent&&processFrozen.config.enabled&&
        processFrozen.config.flushMotionEnabled&&processBefore.pending?processBefore.requestSequence:0ULL;
    if(previousNominal.flushActive&&!processCurrent){guardedFail(9U,true);return true;}
    const auto output=m_g76SphereCycle.Step(input);
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
    const auto& currentNominal=m_g76SphereCycle.Current();
    processInput.suspended=input.hold||previousNominal.holdActive||currentNominal.holdActive||
        previousNominal.flushActive||currentNominal.flushActive||
        previousNominal.gapHistoryActive||currentNominal.gapHistoryActive||
        previousNominal.finishActive||currentNominal.finishActive||
        currentNominal.timeLimitReached||
        currentNominal.timePhase==static_cast<std::uint8_t>(NCMachiningTime::TimePhase::TimedOut);
    processInput.terminal=output.complete;processInput.cancel=input.cancel;
    processInput.completedFlushSequence=currentNominal.flushCompletedSequence;
    processInput.abortedFlushSequence=currentNominal.flushAbandonedSequence;
    if(m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==processInput.session)
        m_g76SphereProcess.Revoke();
    else (void)m_g76SphereProcess.Step(processInput);
    if((previousNominal.flushActive||currentNominal.flushActive||(signedMotion&&!output.complete))&&
       (!m_g76SphereProcess.Current().valid||
        m_g76SphereProcessRevokedSession.load(std::memory_order_acquire)==processInput.session)){
        guardedFail(9U,true);return true;
    }
    bool positionReady=output.stopped&&output.zeroDemand;
    for(int slot=0;slot<m_Group.axisCount;++slot){
        const unsigned i=static_cast<unsigned>(m_Group.axisIndices[slot]);
        if(i>=6U||i!=static_cast<unsigned>(m_g76SphereFrozen.command.axisIndices[slot])){
            guardedFail(2U,true);return true;
        }
        auto& axis=(*m_pContexts)[i];const double ppu=m_g76SphereFrozen.pulsePerNative[i];
        double position=output.commandMCS[i]*ppu;
        const double velocity=output.velocityNative[i]*ppu;
        const double acceleration=output.accelerationNative[i]*ppu;
        const bool fixedValid=i<3U||MotionG76SphereFixedAxes::CommandPulse(output.commandMCS[i],
            m_g76SphereFrozen.request.geometry.startMCS[i],m_g76SphereFrozen.command.mem_startPos[slot],position);
        if(!std::isfinite(position)||!std::isfinite(velocity)||!std::isfinite(acceleration)||
            std::fabs(velocity)>axis.maxVel_PPS*(1.0+1.e-12)||
            !fixedValid||(i>=3U&&(output.velocityNative[i]!=0.0||output.accelerationNative[i]!=0.0))||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,output.commandMCS[i])){
            guardedFail(3U,true);return true;
        }
        if(i<3U){
            const auto& b=m_g76SphereFrozen.axis[i];
            const double accTime=b.accSeconds<0.001?0.2:b.accSeconds;
            const double decTime=(std::max)(b.decSeconds<0.001?0.2:b.decSeconds,b.stopSeconds<0.001?0.2:b.stopSeconds);
            if(std::fabs(acceleration)>(std::min)(b.maxVelocity/accTime,b.maxVelocity/decTime)*(1.0+1.e-10)){
                guardedFail(3U,true);return true;
            }
        }
        const double window=(std::min)(axis.inPositionWindow_Pulse/ppu,i<3U?
            m_g76SphereFrozen.request.positionToleranceMm:m_g76SphereFrozen.request.rotaryToleranceDeg);
        positionReady=positionReady&&std::fabs(input.actualMCS[i]-output.commandMCS[i])<=window;
        axis.logicalCmdPos=axis.currentCmdPos=position;axis.logicalCmdVel=axis.currentCmdVel=velocity;
        axis.planningPos=axis.finalTargetPos=position;axis.targetVelocity=velocity;axis.targetEndVel=0.0;
    }
    // Let the unchanged PID finish approaching a stationary endpoint first.
    // Only a physically in-window reference requests final zero PDO. The
    // engine then requires 200 NEW adjacent zero-send/settle samples.
    m_g76SphereZeroDemand=positionReady;
    if(!m_g76SphereZeroDemand){m_g76SphereZeroSentTick=0ULL;m_g76SpherePhysicalRange=false;}
    if(output.complete&&output.stopped){
        if(!input.physicalSettled||!input.zeroFinalSend||!m_g76SphereZeroDemand){guardedFail(4U,true);return true;}
        const auto& finish=m_g76SphereCycle.Current();
        if(!MotionG76SphereMachiningTime::TerminalQualified(finish,m_g76SphereFrozen.request)){
            guardedFail(3U,true);return true;
        }
        if(!SphereOrbitCompletionFeedbackMatches(finish,m_g76SphereFrozen.request.orbitInspectionRounds,
            m_g76SphereFrozen.request.orbitThresholdScaled,true,m_g76SphereFrozen.request.orbitRetryLimit)){
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

void MotionCore::ObserveG76SphereSend(const MotionServoHandoffSample& sample) noexcept
{
    if(!m_g76SphereRunning)return;
    const auto& command=m_g76SphereFrozen.command;
    m_g76SphereProcessSentTick=0ULL;
    bool scoped=m_Group.isActive&&SphereIdentity(m_Group.currentCmd.execution,command.execution)&&
        m_Group.currentCmd.ownerLease.Matches(command.ownerLease)&&IsMotionOwnerLeaseCurrent(command.ownerLease)&&
        GetCurrentExecutionEpoch()==command.execution.epoch&&!HasPendingSafetyOrRecoveryRequests()&&
        m_g76SphereCycle.Current().tick==m_ncSettleRuntimeCycleTick;
    const auto& nominal=m_g76SphereCycle.Command();
    for(unsigned axis=3U;scoped&&axis<6U;++axis)
        scoped=nominal.velocityNative[axis]==0.0&&nominal.accelerationNative[axis]==0.0&&
            MotionG76SphereFixedAxes::SameBits(nominal.commandMCS[axis],m_g76SphereFrozen.request.geometry.startMCS[axis]);
    if(scoped&&sample.sequence>m_g76SphereProcessSendSequence&&sample.frameGeneration>m_g76SphereProcessSendFrame&&
       MotionG76SphereProcessSend::ExactNonzeroSend(sample,command.execution.epoch,command.ownerLease,
           m_g76SphereFrozen.axisMask,m_ncSettleRuntimeCycleTick)){
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
    return m_g76SphereRunning&&m_g76SphereZeroDemand&&axis.axisIndex>=0&&axis.axisIndex<6&&
        (m_g76SphereFrozen.axisMask&(1U<<static_cast<unsigned>(axis.axisIndex)))!=0U;
}

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
        !SphereIdentity(frozen.command.execution,identity)||!frozen.command.ownerLease.Matches(owner)||
        !IsG76SphereCommandAuthorized(frozen.command)||report.runtimeFault||
        report.cycle.phase!=NCG76SphereCycle::Phase::Complete||report.cycle.fault!=NCG76SphereCycle::Fault::None||
        !MotionG76SphereMachiningTime::CompletedEndpointQualified(report,frozen.request)||
        !SphereOrbitCompletionFeedbackMatches(report.cycle,frozen.request.orbitInspectionRounds,
            frozen.request.orbitThresholdScaled,true,frozen.request.orbitRetryLimit)||
        !MotionG76SphereMachiningTime::TerminalQualified(report.cycle,frozen.request))return false;
    std::array<double,6U> endpointPulse{};
    for(unsigned i=0U;i<6U;++i){
        double pulse=report.endpointMCS[i]*frozen.pulsePerNative[i];
        if(i>=3U&&!MotionG76SphereFixedAxes::CommandPulse(report.endpointMCS[i],
            frozen.request.geometry.startMCS[i],frozen.command.mem_startPos[i],pulse))return false;
        if(!std::isfinite(pulse)||(i<3U?(*m_pContexts)[i].logicalCmdPos.Load()!=pulse:
            !MotionG76SphereFixedAxes::SameBits((*m_pContexts)[i].logicalCmdPos.Load(),pulse)))return false;
        endpointPulse[i]=pulse;
    }
    for(unsigned i=0U;i<6U;++i){
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
       !SphereIdentity(hold.identity,identity)||!hold.owner.Matches(owner)||
       !SphereOrbitCompletionFeedbackMatches(report.cycle,hold.orbitInspectionRounds,hold.orbitThresholdScaled,true,
            hold.orbitRetryLimit)||report.cycle.totalLimitUs!=hold.machiningTime.totalLimitUs||
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
