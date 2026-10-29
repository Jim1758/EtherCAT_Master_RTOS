#include "MotionCore.h"
#include "AlarmManager.h"
#include "PLCManager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

extern PLCManager* g_PLC;
namespace {
bool SphereIdentity(const MotionExecutionIdentity& a,const MotionExecutionIdentity& b) noexcept
{ return a.IsAssigned() && b.IsAssigned() && a.epoch==b.epoch && a.segmentId==b.segmentId &&
    a.sourceBlockId==b.sourceBlockId && a.source==b.source; }
bool SpherePositive(double v) noexcept {return std::isfinite(v)&&v>0.0;}
bool SpherePowerOff() noexcept
{ return g_PLC!=nullptr && g_PLC->ReadEDMOutputWord()==0U; }
constexpr std::uint32_t SphereRuntimeAlarm=AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY;
}

bool MotionCore::ReadG76SphereFeedback(MotionG76SphereFeedback& out) const noexcept
{
    if(!m_g76SphereFeedback.Read(out).Ready())return false;
    out.endpointCommitted=out.completed&&m_g76SphereEndpointCommitted.load(std::memory_order_acquire)==out.session;
    return true;
}

bool MotionCore::TryG76SphereMove(const NCG76SphereCycle::Request& requested,
    const NCTranslationSnapshot& frame,MotionExecutionEpoch epoch,const MotionOwnerLease& owner,
    int sourcePC,MotionExecutionIdentity& identity,MotionG76SphereAdmission* admission) noexcept
{
    identity=MotionExecutionIdentity{};
    if(admission)*admission=MotionG76SphereAdmission{};
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
    G76SphereFrozen frozen{};frozen.request=requested;
    auto& request=frozen.request;auto& command=frozen.command;
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
        if(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,request.geometry.boundsMinMCS[i])||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,request.geometry.boundsMaxMCS[i]))
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
    command.accTime=rampSeconds;command.decTime=stopSeconds;command.targetVel=1.0;
    AssignExecutionIdentity(command,epoch,MotionCommandSource::NC_MEMORY,owner);
    LifecycleCommitReservationGuard reservation(*this,command.execution);
    if(!reservation.IsAcquired()||!SpherePowerOff()||!MatchesNCTranslation(frame)||
        !HasExactExecutionDrainAcknowledgementImpl(epoch,owner,true))return reject(MotionG76SphereAdmissionGate::Reservation);
    const auto previous=m_g76SphereClaim.load(std::memory_order_acquire);
    m_g76SphereEndpointCommitted.store(0ULL,std::memory_order_release);
    m_g76SphereParentBound.store(0ULL,std::memory_order_release);
    if(!m_g76SphereRequests.Publish(frozen))return reject(MotionG76SphereAdmissionGate::Publication);
    m_g76SphereClaim.store(request.session,std::memory_order_release);
    m_g76SphereBarrier.store(request.session,std::memory_order_release);
    if(!TryEnqueueMotionCommand(command)){
        m_g76SphereClaim.store(previous,std::memory_order_release);m_g76SphereBarrier.store(0ULL,std::memory_order_release);return reject(MotionG76SphereAdmissionGate::Enqueue);
    }
    m_g76SphereProducerHighWater=request.session;m_g76SphereProducerControl=G76SphereControl{};
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
        std::memcmp(&command,&frozen.command,sizeof(command))!=0||frozen.axisMask!=63U)return false;
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
        startSequence<=m_g76SphereProducerControl.startSequence)return false;
    G76SphereControl intent{};intent.session=session;intent.identity=identity;intent.owner=owner;
    intent.holdSequence=holdSequence;intent.startSequence=startSequence;
    if(!m_g76SphereControls.ProducerTryPush(intent))return false;
    m_g76SphereProducerControl=intent;return true;
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

void MotionCore::ObserveG76SphereRuntimeInput() noexcept
{
    m_g76SphereInput=NCG76SphereCycle::Input{};
    m_g76SphereInputSession=0ULL;m_g76SphereInputIdentity=MotionExecutionIdentity{};
    if(!m_g76SphereClaim.load(std::memory_order_acquire))return;
    auto& input=m_g76SphereInput;
    input.tick=m_ncSettleRuntimeCycleTick;input.nowUs=m_edmZMonotonicUs;
    input.physicalSequence=input.tick;
    input.pdoValid=m_ncSettleRuntimeCycleValid&&m_ncSettleRuntimeCycleContiguous;
    input.powerInhibited=SpherePowerOff();
    G76SphereFrozen pending{};
    const bool read=m_g76SphereRunning||m_g76SphereRequests.Read(pending).Ready();
    const auto& frozen=m_g76SphereRunning?m_g76SphereFrozen:pending;
    if(read){m_g76SphereInputSession=frozen.request.session;m_g76SphereInputIdentity=frozen.command.execution;
        input.actualMCS=frozen.request.geometry.startMCS;}
    bool source=read&&frozen.request.session&&input.pdoValid&&input.powerInhibited&&
        m_edmZClockValid&&m_edmZClockFrequency!=0ULL&&input.nowUs&&m_pContexts!=nullptr&&m_pContexts->size()>=6U;
    for(unsigned i=0U;source&&i<6U;++i){
        const auto& axis=(*m_pContexts)[i];MotionServoInputSnapshot physical{};
        source=axis.isExist&&!axis.isFirstCycle&&!axis.isFault&&!axis.isLagAlarm&&
            ReadMotionServoInputCompatibilitySnapshotBySlot(static_cast<int>(i),physical)&&
            (physical.StatusWord&0x006FU)==0x0027U&&physical.ModesOfOperationDisplay==9;
        if(!source)break;
        double pulse=MotionG80RuntimeProof::DecodeMotorPulse(physical.ActualPosition,
            static_cast<std::uint32_t>(axis.lastRawActPos),axis.unwrappedActPos,
            axis.isReverse,axis.Axis_Reverse,axis.machineCoordinateOffsetPulse);
        if(axis.fbMode==FeedbackSource::LINEAR_SCALE){
            if(axis.pScaleActualPos==nullptr){source=false;break;}
            const double scale=static_cast<double>(*axis.pScaleActualPos)*axis.scaleToMotorRatio;
            if(!std::isfinite(scale)||std::fabs(pulse+axis.machineCoordinateOffsetPulse-scale)>axis.maxDeviation){source=false;break;}
            pulse=scale-axis.machineCoordinateOffsetPulse;
        }
        const double nominal=pbc::NominalFeedbackPulse(axis.enablePitch||axis.enableBacklash,
            axis.currentCompOffset_unit,pulse,axis.resolution_PPR,axis.finalLead,axis.mechanicalCompensationFrame);
        if(i<3U){input.actualMCS[i]=nominal/frozen.pulsePerNative[i];source=std::isfinite(input.actualMCS[i]);}
        else source=MotionG76SphereFixedAxes::FeedbackNative(frozen.request.geometry.startMCS[i],
            frozen.command.mem_startPos[i],nominal,frozen.pulsePerNative[i],input.actualMCS[i]);
    }
    input.sourceValid=source;
    input.holdSequence=m_g76SphereCurrent.holdSequence;input.startSequence=m_g76SphereCurrent.startSequence;
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
        m_g76SphereCycle.Reset();RejectMotionCommand(popped,MotionRejectReason::NOT_READY,SphereRuntimeAlarm);
        PublishG76SphereRuntime();AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,popped.sourceLinePC);
        RequestEmergencyStopAllAxes();return false;
    };
    if(GetCommandAuthorizationFailure(front)!=MotionRejectReason::NONE)return reject(2U);
    if(!m_g76SphereInput.sourceValid||!m_g76SphereInput.pdoValid||!m_g76SphereInput.powerInhibited||
        !m_g76SphereInput.tick||!m_g76SphereInput.nowUs||m_g76SphereInput.cancel||
        m_g76SphereInput.tick!=m_ncSettleRuntimeCycleTick||
        m_g76SphereInput.physicalSequence!=m_g76SphereInput.tick||
        m_ncSettleRuntimeCycleTick<m_g76SphereAdmissionTick)return reject(1U);
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
        if(!std::isfinite(actual)||!std::isfinite(error))return reject(1U);
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
    m_g76SphereCurrent=MotionG76SphereFeedback{};m_g76SphereCurrent.session=m_g76SphereFrozen.request.session;
    m_g76SphereCurrent.identity=command.execution;m_g76SphereCurrent.owner=command.ownerLease;m_g76SphereCurrent.accepted=true;
    TrackMotionCommandAccepted(command);m_g76SphereCycle.Reset();
    if(!m_g76SphereCycle.Begin(m_g76SphereFrozen.request,m_g76SphereInput)){
        m_g76SphereCurrent.runtimeFault=3U;m_g76SphereCurrent.terminal=true;
        loadReservation.Release();FaultTrackedMotionCommand(SphereRuntimeAlarm,MotionRejectReason::NOT_READY);PublishG76SphereRuntime();
        AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,command.sourceLinePC);RequestEmergencyStopAllAxes();return false;
    }
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
    m_g76SphereCurrent.cycle=m_g76SphereCycle.Current();
    m_g76SphereCurrent.cycle.phase=NCG76SphereCycle::Phase::Fault;m_g76SphereCurrent.cycle.fault=NCG76SphereCycle::Fault::Cancelled;
    m_g76SphereCurrent.cycle.tick=m_ncSettleRuntimeCycleTick;m_g76SphereCurrent.cycle.held=false;
    m_g76SphereCurrent.commandMCS=m_g76SphereCycle.Command().commandMCS;
    m_g76SphereCurrent.endpointMCS=m_g76SphereCurrent.commandMCS;m_g76SphereCurrent.velocityNative.fill(0.0);
    m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
    m_g76SphereCurrent.terminal=true;m_g76SphereCurrent.completed=false;m_g76SphereCurrent.runtimeFault=0U;
    m_g76SphereCurrent.endpointCommitted=false;
    (void)m_g76SphereFeedback.Publish(m_g76SphereCurrent);
    return true;
}


void MotionCore::FailG76SphereRuntime(std::uint32_t fault,bool emergency) noexcept
{
    if(!m_g76SphereRunning)return;
    if(fault==2U&&IsG76SphereResetOutputHeld())return;
    const bool cancelled=GetCurrentExecutionEpoch()!=m_g76SphereFrozen.command.execution.epoch||
        !IsMotionOwnerLeaseCurrent(m_g76SphereFrozen.command.ownerLease)||HasPendingSafetyOrRecoveryRequests();
    m_g76SphereCurrent.runtimeFault=fault;m_g76SphereCurrent.terminal=true;m_g76SphereCurrent.completed=false;
    m_g76SphereInput.cancel=true;
    (void)m_g76SphereCycle.Step(m_g76SphereInput);
    if(cancelled)AbortTrackedMotionCommand();
    else{
        FaultTrackedMotionCommand(SphereRuntimeAlarm,MotionRejectReason::NOT_READY);
        AlarmManager::GetInstance().Trigger(SphereRuntimeAlarm,m_g76SphereFrozen.command.sourceLinePC);
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
        FailG76SphereRuntime(1U,true);return true;
    }
    if(IsG76SphereResetOutputHeld())return true;
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
            m_g76SphereCurrent.startSequence=(std::max)(m_g76SphereCurrent.startSequence,intent.startSequence);
        }else if(!haveHold){
            if(intent.holdSequence!=m_g76SphereCurrent.holdSequence||
                intent.startSequence<=m_g76SphereCurrent.startSequence){guardedFail(6U,true);return true;}
            m_g76SphereCurrent.startSequence=intent.startSequence;
        }
    }
    input.holdSequence=m_g76SphereCurrent.holdSequence;input.startSequence=m_g76SphereCurrent.startSequence;
    const auto output=m_g76SphereCycle.Step(input);
    if(output.emergencyStopRequired||output.faulted){guardedFail(3U,true);return true;}
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
    if(!m_g76SphereRunning||!m_g76SphereZeroDemand)return;
    const auto& command=m_g76SphereFrozen.command;
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
    m_g76SphereCurrent.endpointMCS=m_g76SphereCycle.Command().commandMCS;
    m_g76SphereCurrent.commandMCS=m_g76SphereCycle.Command().commandMCS;
    m_g76SphereCurrent.velocityNative=m_g76SphereCycle.Command().velocityNative;
    m_g76SphereCurrent.actualMCS=m_g76SphereInput.actualMCS;
    m_g76SphereCurrent.positionToleranceMm=m_g76SphereFrozen.request.positionToleranceMm;
    m_g76SphereCurrent.rotaryToleranceDeg=m_g76SphereFrozen.request.rotaryToleranceDeg;
    m_g76SphereCurrent.inputSequence=m_g76SphereInput.physicalSequence;
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
        report.endpointMCS!=frozen.request.geometry.endMCS)return false;
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
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired()||!HasExactExecutionDrainAcknowledgementImpl(identity.epoch,owner,true))return false;
    auto expected=session;return m_g76SphereBarrier.compare_exchange_strong(expected,0ULL,
        std::memory_order_acq_rel,std::memory_order_acquire);
}
