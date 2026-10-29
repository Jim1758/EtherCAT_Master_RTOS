#include "MotionCore.h"
#include "G80Coordinates.h"
#include "AlarmManager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
bool G80SameIdentity(const MotionExecutionIdentity& a,const MotionExecutionIdentity& b) noexcept
{ return a.epoch==b.epoch && a.segmentId==b.segmentId && a.sourceBlockId==b.sourceBlockId && a.source==b.source; }
G80::SimulationSettings G80SimulationSettings(const G80::Config& config) noexcept
{
    G80::SimulationSettings settings{};
    settings.scenario=config.simulationScenario;
    settings.edgeDistanceMm=config.simulationEdgeDistanceMm;settings.edgeDistanceDeg=config.simulationEdgeDistanceDeg;
    settings.spreadMm=config.simulationSpreadMm;settings.spreadDeg=config.simulationSpreadDeg;
    settings.sourceLossDistanceMm=config.simulationLossDistanceMm;settings.sourceLossDistanceDeg=config.simulationLossDistanceDeg;
    return settings;
}
std::uint32_t G80Alarm(G80::Fault fault) noexcept
{
    switch(fault) {
    case G80::Fault::SourceLost: case G80::Fault::PdoLost:
    case G80::Fault::NonMonotonicTick: case G80::Fault::SourceSequenceRegression:
        return AlarmManager::G80_SIGNAL_INVALID;
    case G80::Fault::InitialContact: case G80::Fault::ApproachContact:
        return AlarmManager::G80_CONTACT_AT_START;
    case G80::Fault::CycleTimeout: case G80::Fault::SearchDistanceExceeded: return AlarmManager::G80_EDGE_NOT_FOUND;
    case G80::Fault::ReleaseTimeout: return AlarmManager::G80_CONTACT_NOT_RELEASED;
    case G80::Fault::MeanErrorExceeded: return AlarmManager::G80_REPEAT_ERROR;
    case G80::Fault::SettleTimeout: return AlarmManager::G80_SETTLE_TIMEOUT;
    default: return AlarmManager::G80_RUNTIME_FAULT;
    }
}
}

bool MotionCore::ReadG80Feedback(MotionG80Feedback& out) const noexcept
{
    if(!m_g80Feedback.Read(out).Ready()) return false;
    out.endpointCommitted=out.completed && m_g80EndpointCommitted.load(std::memory_order_acquire)==out.session;
    return true;
}

bool MotionCore::TryG80Move(const G80::Request& requested,const NCTranslationSnapshot& frame,
    MotionExecutionEpoch epoch,const MotionOwnerLease& owner,int sourcePC,
    MotionExecutionIdentity& identity) noexcept
{
    identity=MotionExecutionIdentity{};
    std::string error;
    if(!requested.session || requested.session<=m_g80ProducerHighWater || requested.inputSource>1U ||
        !G80::ValidateConfig(requested.config,error) || !requested.config.enabled ||
        !IsNCTranslationSnapshotValid(frame) || frame.cutterMode!=40 || !MatchesNCTranslation(frame) ||
        (owner.owner!=MotionOwner::AUTO && owner.owner!=MotionOwner::MDI) ||
        (requested.hasApproach && owner.owner!=MotionOwner::AUTO) ||
        !HasExactExecutionDrainAcknowledgement(epoch,owner) || m_pContexts==nullptr ||
        m_pCoordMgr==nullptr || m_pContexts->size()<6U ||
        GetQueueSize()!=0U || GetCommandIngressSize()!=0U || GetCommandReplaySize()!=0U)
        return false;
    // One immutable sidecar at a time; the previous session can be replaced only
    // after the exact global drain above, never while RT can still drive it.
    G80Frozen frozen{}; frozen.request=requested;
    auto& command=frozen.command;
    command.g80EdgeFind=true; command.commandPathMode=MotionCommandPathMode::EXACT_STOP;
    command.sourceLinePC=sourcePC; command.sourceTranslation=frame;
    command.sourceWCS=frame.wcsCode; command.sourceToolLengthMode=frame.toolLengthMode;
    command.sourceHCode=frame.toolHCode; command.sourceToolRadiusMode=frame.cutterMode;
    command.sourceDCode=frame.cutterD; command.sourceIsAbsoluteMode=frame.distanceMode==90;
    command.sourceG68Active=frame.rotationMode==68; command.sourceG68Angle=frame.rotationAngleDeg;
    command.sourceG168Active=frame.workMode==168; command.sourceWCode=frame.workWCode;
    command.sourceG51Active=frame.scalingMode==51; command.sourceScaleRatio=frame.scalingFactor;
    command.sourceMirrorMask=static_cast<std::uint8_t>(frame.mirrorMask);
    command.sourceG16Active=frame.polarMode==16;
    command.sourceG162Active=frame.axisIdentity.eccentricEnabled!=0U;
    command.sourcePlaneMode=frame.rotationPlane;
    for(unsigned i=0U;i<6U;++i) {
        const auto& axis=(*m_pContexts)[i];
        const bool participates=requested.direction[i]!=0.0 ||
            (requested.hasApproach && requested.approachEndMCS[i]!=requested.startMCS[i]);
        if(!std::isfinite(requested.startMCS[i]) || !std::isfinite(requested.direction[i])) return false;
        if(!axis.isExist) {
            if(participates) return false;
            frozen.request.velocityCapsNative[i]=frozen.request.accelerationCapsNative[i]=
                frozen.request.decelerationCapsNative[i]=1.0;
            continue;
        }
        const double ppu=axis.resolution_PPR/axis.finalLead;
        if(axis.axisIndex!=static_cast<int>(i) || !std::isfinite(ppu) || ppu<=0.0 ||
            !std::isfinite(axis.maxVel_PPS) || axis.maxVel_PPS<=0.0 ||
            (i<3U ? axis.axisType!=AxisType::LINEAR : axis.axisType==AxisType::LINEAR)) return false;
        auto& binding=frozen.axis[i];
        binding.resolution=axis.resolution_PPR;binding.lead=axis.finalLead;
        binding.maxVelocity=axis.maxVel_PPS;
        binding.accSeconds=requested.config.accelerationSeconds;binding.decSeconds=requested.config.decelerationSeconds;
        binding.machineOffset=axis.machineCoordinateOffsetPulse;binding.scaleRatio=axis.scaleToMotorRatio;
        binding.maxDeviation=axis.maxDeviation;binding.scalePointer=reinterpret_cast<std::uintptr_t>(axis.pScaleActualPos);
        binding.feedbackMode=static_cast<std::uint32_t>(axis.fbMode);
        binding.reverseFlags=static_cast<std::uint8_t>((axis.isReverse?1U:0U)|(axis.Axis_Reverse?2U:0U));
        frozen.pulsePerNative[i]=ppu;
        if(!std::isfinite(requested.config.gains[i].followingLimitNative*ppu)) return false;
        const double vmax=axis.maxVel_PPS/ppu;
        frozen.request.velocityCapsNative[i]=vmax;
        // EDM279: G80 uses its frozen native ramp profile, independent of G00.
        if(!G80::BuildNativeRampCaps(requested.config,vmax,frozen.request.accelerationCapsNative[i],
            frozen.request.decelerationCapsNative[i])) return false;
        if(requested.hasApproach && participates &&
            (!std::isfinite(requested.rapidVelocityNative[i]) || requested.rapidVelocityNative[i]<=0.0 ||
             requested.rapidVelocityNative[i]>vmax)) return false;
        // Always carry the installed travel envelope in the frozen request.
        // Runtime still executes the ordinary current software/hardware guards.
        double lo=-(std::numeric_limits<double>::max)(),hi=(std::numeric_limits<double>::max)();
        bool bounded=false;
        const bool enabled[3]={axis.travelLimit1Enable && frame.storedStrokeMode==22,axis.travelLimit2Enable,axis.travelLimit3Enable};
        const double lows[3]={axis.travelLimit1Negative_unit,axis.travelLimit2Negative_unit,axis.travelLimit3Negative_unit};
        const double highs[3]={axis.travelLimit1Positive_unit,axis.travelLimit2Positive_unit,axis.travelLimit3Positive_unit};
        if(axis.isHomed) for(unsigned k=0U;k<3U;++k) if(enabled[k]) {
            if(!std::isfinite(lows[k]) || !std::isfinite(highs[k]) || lows[k]>=highs[k]) return false;
            lo=(std::max)(lo,lows[k]);hi=(std::min)(hi,highs[k]);bounded=true;
        }
        if(bounded) { frozen.request.travelMask|=1U<<i; frozen.request.travelMin[i]=lo;frozen.request.travelMax[i]=hi; }
        if(!participates) continue;
        if(!m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,requested.startMCS[i]) ||
            (requested.hasApproach && !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,requested.approachEndMCS[i]))) return false;
        frozen.axisMask|=1U<<i;
        const int slot=command.axisCount++;
        command.axisIndices[slot]=static_cast<int>(i);
        // Unknown final endpoint stays out of the producer queue tail until
        // the successful, settled RT receipt is explicitly consumed by NC.
        const double startPulse=axis.logicalCmdPos.Load();
        const double requestedPulse=requested.startMCS[i]*ppu;
        if(!std::isfinite(startPulse) || !std::isfinite(requestedPulse) ||
            std::fabs(startPulse-requestedPulse)>32.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,std::fabs(startPulse))) return false;
        frozen.request.startMCS[i]=startPulse/ppu;
        command.mem_startPos[slot]=startPulse;
        command.targetPos[slot]=command.mem_startPos[slot];
        command.mem_ratio[slot]=ppu;
    }
    if(command.axisCount==0 || !std::isfinite(requested.slowPercent) || requested.slowPercent<=0.0)
        return false;
    if(requested.inputSource==1U) {
        // Pure, request-specific simulation validation precedes any ownership,
        // queue or endpoint mutation. No physical ADC route is installed here.
        G80::VoltageSimulator validateSimulation{};
        if(!validateSimulation.Begin(frozen.request,G80SimulationSettings(requested.config))) return false;
    }
    command.accTime=requested.config.accelerationSeconds; command.decTime=requested.config.decelerationSeconds;
    command.targetVel=1.0;
    AssignExecutionIdentity(command,epoch,ResolveMotionCommandSourceForOwner(owner.owner),owner);
    // Reserve the exact lifecycle through publication and fixed-ring admission.
    LifecycleCommitReservationGuard reservation(*this,command.execution);
    if(!reservation.IsAcquired() || !MatchesNCTranslation(frame) ||
        !HasExactExecutionDrainAcknowledgementImpl(epoch,owner,true)) return false;
    const auto previous=m_g80Claim.load(std::memory_order_acquire);
    m_g80EndpointCommitted.store(0ULL,std::memory_order_release);
    if(!m_g80Requests.Publish(frozen)) return false;
    m_g80Claim.store(requested.session,std::memory_order_release);
    m_g80Barrier.store(requested.session,std::memory_order_release);
    if(!TryEnqueueMotionCommand(command)) {
        m_g80Claim.store(previous,std::memory_order_release);
        m_g80Barrier.store(0ULL,std::memory_order_release);
        return false;
    }
    m_g80ProducerHighWater=requested.session;
    identity=command.execution;
    return true;
}

bool MotionCore::IsG80CommandAuthorized(const MotionCommand& command) const noexcept
{
    if(!command.g80EdgeFind || command.commandPathMode!=MotionCommandPathMode::EXACT_STOP ||
        (command.ownerLease.owner!=MotionOwner::AUTO && command.ownerLease.owner!=MotionOwner::MDI) ||
        command.execution.source!=ResolveMotionCommandSourceForOwner(command.ownerLease.owner) ||
        !MatchesNCTranslation(command.sourceTranslation)) return false;
    G80Frozen frozen{};
    if(!m_g80Requests.Read(frozen).Ready() || !frozen.request.session ||
        m_g80Claim.load(std::memory_order_acquire)!=frozen.request.session ||
        std::memcmp(&command,&frozen.command,sizeof(command))!=0) return false;
    for(unsigned i=0U;i<6U;++i) if(frozen.axisMask&(1U<<i)) {
        const auto& axis=(*m_pContexts)[i];const auto& b=frozen.axis[i];
        if(axis.resolution_PPR!=b.resolution || axis.finalLead!=b.lead || axis.maxVel_PPS!=b.maxVelocity ||
            frozen.request.config.accelerationSeconds!=b.accSeconds || frozen.request.config.decelerationSeconds!=b.decSeconds ||
            axis.machineCoordinateOffsetPulse!=b.machineOffset || axis.scaleToMotorRatio!=b.scaleRatio ||
            axis.maxDeviation!=b.maxDeviation || reinterpret_cast<std::uintptr_t>(axis.pScaleActualPos)!=b.scalePointer ||
            static_cast<std::uint32_t>(axis.fbMode)!=b.feedbackMode ||
            static_cast<std::uint8_t>((axis.isReverse?1U:0U)|(axis.Axis_Reverse?2U:0U))!=b.reverseFlags) return false;
    }
    return true;
}

void MotionCore::ObserveG80RuntimeInput() noexcept
{
    m_g80Input=G80::Input{};m_g80InputSession=0ULL;m_g80InputIdentity=MotionExecutionIdentity{};
    if(!m_g80Claim.load(std::memory_order_acquire)) return;
    m_g80Input.tick=m_ncSettleRuntimeCycleTick;m_g80Input.nowUs=m_edmZMonotonicUs;
    m_g80Input.pdoValid=m_ncSettleRuntimeCycleValid && m_ncSettleRuntimeCycleContiguous;
    G80Frozen pending{};
    const bool pendingRead=m_g80Running || m_g80Requests.Read(pending).Ready();
    const auto& frozen=m_g80Running?m_g80Frozen:pending;
    const auto& request=frozen.request;
    const auto participantMask=frozen.axisMask;
    if(pendingRead) {m_g80InputSession=request.session;m_g80InputIdentity=frozen.command.execution;}
    // Q1 changes only this parent's GAP voltage. The clock, EtherCAT receipt,
    // axis identity, operation-enabled bits and actual pose remain physical.
    bool source=pendingRead && request.session!=0ULL && request.inputSource<=1U &&
        m_g80Input.pdoValid && m_edmZClockValid && m_edmZClockFrequency!=0ULL &&
        m_g80Input.nowUs!=0ULL && m_pContexts!=nullptr && m_pContexts->size()>=6U;
    for(unsigned i=0U;source && i<6U;++i) {
        const auto& axis=(*m_pContexts)[i];
        if(!axis.isExist) continue;
        MotionServoInputSnapshot input{};
        source=!axis.isFirstCycle && ReadMotionServoInputCompatibilitySnapshotBySlot(static_cast<int>(i),input) &&
            ((participantMask&(1U<<i))==0U || ((input.StatusWord&0x006FU)==0x0027U && input.ModesOfOperationDisplay==9));
        if(!source) break;
        double pulse=MotionG80RuntimeProof::DecodeMotorPulse(input.ActualPosition,
            static_cast<std::uint32_t>(axis.lastRawActPos),axis.unwrappedActPos,
            axis.isReverse,axis.Axis_Reverse,axis.machineCoordinateOffsetPulse);
        if(axis.fbMode==FeedbackSource::LINEAR_SCALE) {
            if(axis.pScaleActualPos==nullptr) {source=false;break;}
            const double scale=static_cast<double>(*axis.pScaleActualPos)*axis.scaleToMotorRatio;
            if(!std::isfinite(scale) || std::fabs(pulse+axis.machineCoordinateOffsetPulse-scale)>axis.maxDeviation)
                {source=false;break;}
            pulse=scale-axis.machineCoordinateOffsetPulse;
        }
        // Same current PDO pose and already-applied PBC frame in both modes.
        // Simulation never substitutes command position for actual feedback.
        const double nominal=pbc::NominalFeedbackPulse(axis.enablePitch||axis.enableBacklash,
            axis.currentCompOffset_unit,pulse,axis.resolution_PPR,axis.finalLead,axis.mechanicalCompensationFrame);
        m_g80Input.actualMCS[i]=nominal*axis.finalLead/axis.resolution_PPR;
        source=std::isfinite(m_g80Input.actualMCS[i]);
    }
    if(pendingRead && request.inputSource==1U) {
        if(!MotionG80RuntimeProof::SameInputScope(m_g80SimulationSession,m_g80SimulationIdentity,
            request.session,frozen.command.execution)) {
            m_g80VoltageSimulator.Reset();m_g80SimulationSession=request.session;
            m_g80SimulationIdentity=frozen.command.execution;
            m_g80SimulationReady=m_g80VoltageSimulator.Begin(request,G80SimulationSettings(request.config));
        }
        // The simulator sees physical validity before creating a synthetic
        // sequence. Invalid PDO/clock/pose cannot be hidden by a SIM voltage.
        m_g80Input.sourceValid=source && m_g80SimulationReady;
        G80::Feedback feedback{};
        if(m_g80Running) feedback=m_g80Cycle.Current();
        const auto simulated=m_g80VoltageSimulator.Sample(m_g80Input,feedback);
        m_g80Input.sourceSequence=simulated.sequence;m_g80Input.voltageV=simulated.voltageV;
        source=m_g80Input.sourceValid && simulated.valid && simulated.session==request.session;
    }
    else {
        const auto& receipt=m_edm129Physical.input;
        const auto& sample=receipt.sample;
        const bool fresh=m_edm129PhysicalStamp.Ready() && MotionG80RuntimeProof::FreshPhysicalInput(
            receipt,m_edmZMonotonicUs,m_edmZClockFrequency,m_g80SourceSequence,m_edmZClockValid);
        m_g80Input.sourceSequence=sample.sequence;
        m_g80Input.voltageV=static_cast<double>(receipt.conversion.voltageMv)/1000.0;
        source=source && fresh;
        if(m_g80Running) source=source && EDM63::SameRoute(m_g80SourceRoute,EDM63::ProducerRoute(sample)) &&
            m_g80ProfileRevision==receipt.expectedProfileRevision &&
            m_g80CalibrationRevision==receipt.expectedCalibrationRevision;
        if(fresh) m_g80SourceSequence=sample.sequence;
    }
    m_g80Input.sourceValid=source;
    // Invalid PDO callbacks skip UpdateInterpolation. Contain on their first
    // occurrence in both source modes, before stale CSV can survive the skip.
    if(m_g80Running && !source) FailG80Runtime(1U,true);
}

bool MotionCore::LoadG80Command(const MotionCommand& front) noexcept
{
    if(m_Group.isActive || m_g80Running || !m_g80Requests.Read(m_g80Frozen).Ready()) return false;
    // A producer may enqueue after this callback's source hook. That packet
    // waits for its own next physical input snapshot; another session's sample
    // is neither usable evidence nor proof that the new source has failed.
    if(!MotionG80RuntimeProof::SameInputScope(m_g80InputSession,m_g80InputIdentity,
        m_g80Frozen.request.session,front.execution)) return false;
    if(m_g80AdmissionSession!=m_g80Frozen.request.session) {
        m_g80AdmissionSession=m_g80Frozen.request.session;m_g80AdmissionTick=m_ncSettleRuntimeCycleTick;
    }
    const auto reject=[&](std::uint32_t alarm) noexcept {
        MotionCommand popped{};
        if(HasPendingExecutionEpochChange() || !m_Group.cmdQueue.ConsumerTryPop(popped)) return false;
        m_g80Current=MotionG80Feedback{};m_g80Current.session=m_g80Frozen.request.session;
        m_g80Current.identity=popped.execution;m_g80Current.owner=popped.ownerLease;
        m_g80Current.accepted=true;m_g80Current.terminal=true;m_g80Current.runtimeFault=alarm;
        m_g80Cycle.Reset();
        RejectMotionCommand(popped,MotionRejectReason::NOT_READY,alarm);
        PublishG80Runtime();
        AlarmManager::GetInstance().Trigger(alarm,popped.sourceLinePC);
        RequestEmergencyStopAllAxes();
        return false;
    };
    if(GetCommandAuthorizationFailure(front)!=MotionRejectReason::NONE)
        return reject(AlarmManager::G80_FRAME_CHANGED);
    if(!m_g80Input.sourceValid || !m_g80Input.pdoValid) return reject(AlarmManager::G80_SIGNAL_INVALID);
    const double waited=static_cast<double>(m_ncSettleRuntimeCycleTick-m_g80AdmissionTick)*G80::SampleSeconds;
    for(int slot=0;slot<front.axisCount;++slot) {
        const unsigned i=static_cast<unsigned>(front.axisIndices[slot]);
        const auto& axis=(*m_pContexts)[i];
        if(axis.logicalCmdPos.Load()!=front.mem_startPos[slot] || axis.homeRuntime.active || axis.isFault || axis.isLagAlarm)
            return reject(AlarmManager::G80_FRAME_CHANGED);
        if(!IsG80IncomingReady(axis) || !axis.IsMechanicalCompensationReadyForCompletion())
            return waited>=m_g80Frozen.request.config.settleTimeoutSeconds?reject(AlarmManager::G80_SETTLE_TIMEOUT):false;
    }
    MotionCommand command{};
    if(HasPendingExecutionEpochChange() || !m_Group.cmdQueue.ConsumerTryPop(command)) return false;
    if(!G80SameIdentity(command.execution,front.execution) ||
        GetCommandAuthorizationFailure(command)!=MotionRejectReason::NONE) {
        RejectMotionCommand(command,MotionRejectReason::NOT_READY,AlarmManager::G80_FRAME_CHANGED);
        return false;
    }
    m_g80Current=MotionG80Feedback{};
    m_g80Current.session=m_g80Frozen.request.session;m_g80Current.identity=command.execution;
    m_g80Current.owner=command.ownerLease;m_g80Current.accepted=true;
    TrackMotionCommandAccepted(command);
    m_g80Cycle.Reset();
    if(!m_g80Cycle.Begin(m_g80Frozen.request,m_g80Input)) {
        m_g80Current.runtimeFault=3U;m_g80Current.terminal=true;
        FaultTrackedMotionCommand(G80Alarm(m_g80Cycle.Current().fault),MotionRejectReason::NOT_READY);
        PublishG80Runtime();
        AlarmManager::GetInstance().Trigger(G80Alarm(m_g80Cycle.Current().fault),command.sourceLinePC);
        RequestEmergencyStopAllAxes();
        return false;
    }
    m_g80SourceRoute=EDM63::Route{};m_g80ProfileRevision=m_g80CalibrationRevision=0U;
    if(m_g80Frozen.request.inputSource==0U) {
        m_g80SourceRoute=EDM63::ProducerRoute(m_edm129Physical.input.sample);
        m_g80ProfileRevision=m_edm129Physical.input.expectedProfileRevision;
        m_g80CalibrationRevision=m_edm129Physical.input.expectedCalibrationRevision;
    }
    m_g80Derivative={};
    m_g80ZeroDemand=false;m_g80ZeroSentTick=0ULL;m_g80ZeroSettleTicks=0U;m_g80TerminalWaitTicks=0U;
    m_Group.currentCmd=command;m_Group.isActive=true;m_Group.mode=InterpolationMode::LINEAR;
    m_Group.pathMode=PathMode::EXACT_STOP;m_Group.enableHistory=false;m_Group.enableTransform=false;
    m_Group.axisCount=command.axisCount;
    m_Group.currentExecutionPC = command.sourceLinePC;
    m_Group.currentExecutionWCS = command.sourceWCS;
    m_Group.currentExecutionToolMode = command.sourceToolLengthMode;
    m_Group.currentExecutionHCode = command.sourceHCode;
    m_Group.currentExecutionToolRadiusMode = command.sourceToolRadiusMode;
    m_Group.currentExecutionDCode = command.sourceDCode;
    m_Group.currentExecutionIsAbsoluteMode = command.sourceIsAbsoluteMode;
    m_Group.currentExecutionG68Active = command.sourceG68Active;
    m_Group.currentExecutionG68Angle = command.sourceG68Angle;
    m_Group.currentExecutionG168Active = command.sourceG168Active;
    m_Group.currentExecutionWCode = command.sourceWCode;
    m_Group.currentExecutionG51Active = command.sourceG51Active;
    m_Group.currentExecutionScaleRatio = command.sourceScaleRatio;
    m_Group.currentExecutionMirrorMask = command.sourceMirrorMask;
    m_Group.currentExecutionG16Active = command.sourceG16Active;
    m_Group.currentExecutionG162Active = command.sourceG162Active;
    m_Group.currentExecutionPlaneMode = command.sourcePlaneMode;
    for(int slot=0;slot<command.axisCount;++slot) {
        const int i=command.axisIndices[slot];m_Group.axisIndices[slot]=i;
        auto& axis=(*m_pContexts)[i];
        m_g80SavedPid[static_cast<unsigned>(i)]=axis.pid;
        axis.state=MotionState::MotionState_INTERPOLATING;axis.inPosition=false;
        axis.currentCmdVel=axis.logicalCmdVel=axis.targetVelocity=axis.targetEndVel=0.0;
        axis.pid.integralAcc=axis.pid.prevError=0.0;
    }
    auto& v=m_Group.virtualAxis;
    v.currentCmdPos=v.logicalCmdPos=v.planningPos=v.finalTargetPos=0.0;v.currentCmdVel=v.logicalCmdVel=v.targetVelocity=v.targetEndVel=0.0;
    v.state=MotionState::MotionState_INTERPOLATING;v.inPosition=false;
    m_g80Running=true;m_g80Current.started=true;
    CommitCommandPathModeConsumerAuthority(command,MotionCommandPathModeAuthorityDecision::APPLY_EXACT_STOP);
    TrackMotionCommandStarted(command);
    PublishG80Runtime();
    return true;
}

void MotionCore::PublishG80Runtime() noexcept
{
    m_g80Current.cycle=m_g80Cycle.Current();m_g80Current.voltageV=m_g80Input.voltageV;
    m_g80Current.endpointMCS=m_g80Cycle.Command().commandMCS;
    m_g80Current.commandMCS=m_g80Cycle.Command().commandMCS;
    m_g80Current.velocityNative=m_g80Cycle.Command().velocityNative;
    m_g80Current.actualMCS=m_g80Input.actualMCS;
    m_g80Current.inputSequence=m_g80Input.sourceSequence;
    const auto& request=m_g80Frozen.request;
    m_g80Current.source=request.session==m_g80Current.session && request.inputSource<=1U?request.inputSource+1U:0U;
    m_g80Current.simScenario=m_g80Current.source==2U?request.config.simulationScenario:0U;
    m_g80Current.simContact=false;m_g80Current.simProjectedDistance=m_g80Current.simPlaneDistance=0.0;
    const auto& simulated=m_g80VoltageSimulator.Current();
    if(m_g80Current.source==2U && simulated.session==m_g80Current.session) {
        m_g80Current.simContact=simulated.contact;
        m_g80Current.simProjectedDistance=simulated.projectedDistance;
        m_g80Current.simPlaneDistance=simulated.planeDistance;
    }
    (void)m_g80Feedback.Publish(m_g80Current);
}

bool MotionCore::RequestG80ResetCancellation(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,std::uint64_t resetHoldProvenance) noexcept
{
    G80Frozen frozen{};
    if(!session||!identity.IsAssigned()||!m_g80Requests.Read(frozen).Ready()||
       frozen.request.session!=session||!G80SameIdentity(frozen.command.execution,identity)||
       !frozen.command.ownerLease.Matches(owner)||m_g80Claim.load(std::memory_order_acquire)!=session||
       !IsMotionOwnerLeaseCurrent(owner)||GetCurrentExecutionEpoch()!=identity.epoch||
       !resetHoldProvenance||GetSafetyProvenanceGeneration()!=resetHoldProvenance) return false;
    G80ResetScope scope{};scope.session=session;scope.identity=identity;scope.owner=owner;
    scope.previousProvenance=scope.expectedProvenance=resetHoldProvenance;
    return m_g80ResetScope.Publish(scope);
}

void MotionCore::AdvanceG80ResetScope(std::uint64_t previousProvenance,std::uint64_t nextProvenance,
    std::uint64_t expectedSafetyOwnerState) noexcept
{
    // The same NC Reset producer announces only its next exact provenance
    // operation and prospective owner/ticket before the original CAS. The
    // fixed snapshot closes partial-token seams; it cannot authorize motion.
    G80ResetScope scope{};G80Frozen frozen{};
    if(!previousProvenance||!nextProvenance||!m_g80ResetScope.Read(scope).Ready()||
       !m_g80Requests.Read(frozen).Ready()||!scope.session||scope.session!=frozen.request.session||
       !G80SameIdentity(scope.identity,frozen.command.execution)||!scope.owner.Matches(frozen.command.ownerLease)||
       scope.expectedProvenance!=previousProvenance) return;
    scope.previousProvenance=previousProvenance;scope.expectedProvenance=nextProvenance;
    if(expectedSafetyOwnerState&&expectedSafetyOwnerState!=scope.expectedSafetyOwnerState){
        scope.previousSafetyOwnerState=scope.expectedSafetyOwnerState;
        scope.expectedSafetyOwnerState=expectedSafetyOwnerState;
    }
    (void)m_g80ResetScope.Publish(scope);
}

bool MotionCore::IsG80ResetOutputHeld(bool applyingExactBatch) const noexcept
{
    G80ResetScope scope{};
    if(!m_g80Running||!m_g80ResetScope.Read(scope).Ready()||!scope.session||
       scope.session!=m_g80Frozen.request.session||m_g80Current.session!=scope.session||
       !G80SameIdentity(scope.identity,m_g80Frozen.command.execution)||
       !G80SameIdentity(m_g80Current.identity,scope.identity)||!scope.owner.Matches(m_g80Frozen.command.ownerLease)||
       !IsResetSafetyOutputHoldEstablished()||m_safetyControlledStopInProgress||
       AlarmManager::GetInstance().HasAlarm()||
       m_emergencyStopRequestPublication.load(std::memory_order_acquire)!=0ULL||
       m_resetAllFaultsPending.load(std::memory_order_acquire)||m_stopGroupPending.load(std::memory_order_acquire)||
       m_axisFaultResetPendingMask.load(std::memory_order_acquire)!=0U||
       (m_p1MappingIntegrityAlarmRequestPublication.load(std::memory_order_acquire)&0x8000000000000000ULL)!=0ULL||
       !IsG80CommandAuthorized(m_g80Frozen.command)||m_Group.enableTransform||m_Group.enableHistory||
       m_Group.jumpManager.state!=JumpState::IDLE||m_pathHold.sourceSeen||m_Group.pathMode!=PathMode::EXACT_STOP||
       !std::isfinite(m_Group.feedrateOverride)||m_Group.feedrateOverride<0.0||m_Group.feedrateOverride>1.0||
       m_Group.axisCount!=m_g80Frozen.command.axisCount||m_pContexts==nullptr) return false;
    for(int i=0;i<m_Group.axisCount;++i)
        if(m_Group.axisIndices[i]!=m_g80Frozen.command.axisIndices[i])return false;
    // Epoch abort clears only currentCmd's identity/lease. Every other field
    // must still be the same frozen G80 command, never a replacement group.
    MotionCommand current=m_Group.currentCmd;
    const bool exactActive=G80SameIdentity(current.execution,scope.identity)&&current.ownerLease.Matches(scope.owner);
    const bool exactAborted=!m_Group.isActive&&!current.execution.IsAssigned()&&!current.ownerLease.IsValid()&&
        m_feedbackTrackedTerminal&&G80SameIdentity(m_feedbackTrackedIdentity,scope.identity)&&
        m_feedbackTrackedOwnerLease.Matches(scope.owner);
    if(!exactActive&&!exactAborted)return false;
    current.execution=m_g80Frozen.command.execution;current.ownerLease=m_g80Frozen.command.ownerLease;
    if(std::memcmp(&current,&m_g80Frozen.command,sizeof(current))!=0)return false;
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

bool MotionCore::RetireG80ForResetSafetyBatch(MotionExecutionEpoch resetEpoch,
    const MotionOwnerLease& resetOwner,std::uint32_t resetTicket,std::uint64_t resetProvenance) noexcept
{
    if(!m_g80Running) return true;
    // Called only after the existing batch has proved its exact ticket,
    // epoch/provenance and aborted the group. Re-prove the immutable G80 and
    // current Safety tuple; an arbitrary epoch mismatch cannot retire G80.
    const auto ownerState=m_motionOwnerState.load(std::memory_order_acquire);
    if(!IsG80ResetOutputHeld(true)||m_Group.isActive||m_safetyControlledStopInProgress||
       resetEpoch==MOTION_EXECUTION_EPOCH_INVALID||resetEpoch==m_g80Frozen.command.execution.epoch||
       resetEpoch!=GetCurrentExecutionEpoch()||!resetOwner.IsValid()||resetOwner.owner!=MotionOwner::SAFETY||
       !resetOwner.Matches(UnpackMotionOwnerState(ownerState))||!IsMotionOwnerLeaseCurrent(resetOwner)||
       !resetTicket||UnpackMotionOwnerSafetyRequestTicket(ownerState)!=resetTicket||
       UnpackMotionOwnerSafetyHandshake(ownerState)||UnpackMotionOwnerSafetyActionPending(ownerState)||
       !resetProvenance||m_executionDrainRevocationGeneration.load(std::memory_order_acquire)!=resetProvenance||
       m_executionDrainRevocationPublishersInProgress.load(std::memory_order_acquire)!=0U||
       !m_feedbackTrackedTerminal||!G80SameIdentity(m_feedbackTrackedIdentity,m_g80Frozen.command.execution)||
       !m_feedbackTrackedOwnerLease.Matches(m_g80Frozen.command.ownerLease)||
       m_g80Current.runtimeFault||m_g80Cycle.Current().fault!=G80::Fault::None||
       m_pContexts==nullptr) return false;
    for(unsigned i=0U;i<6U;++i)if(m_g80Frozen.axisMask&(1U<<i)){
        if(i>=m_pContexts->size()) return false;
        const auto& axis=(*m_pContexts)[i];
        if(axis.currentCmdVel!=0.0||axis.logicalCmdVel!=0.0||axis.targetVelocity!=0.0||axis.targetEndVel!=0.0)
            return false;
    }
    RestoreG80PidLimits();
    m_g80ZeroDemand=true;m_g80Running=false;
    m_g80Current.cycle=m_g80Cycle.Current();
    m_g80Current.cycle.phase=G80::Phase::Fault;m_g80Current.cycle.fault=G80::Fault::Cancelled;
    m_g80Current.cycle.tick=m_ncSettleRuntimeCycleTick;m_g80Current.cycle.held=false;
    m_g80Current.commandMCS=m_g80Cycle.Command().commandMCS;
    m_g80Current.endpointMCS=m_g80Current.commandMCS;m_g80Current.velocityNative.fill(0.0);
    m_g80Current.actualMCS=m_g80Input.actualMCS;
    m_g80Current.terminal=true;m_g80Current.completed=false;m_g80Current.runtimeFault=0U;
    m_g80Current.endpointCommitted=false;
    (void)m_g80Feedback.Publish(m_g80Current);
    return true;
}

void MotionCore::FailG80Runtime(std::uint32_t fault,bool emergency) noexcept
{
    if(!m_g80Running) return;
    // A matched Reset with output held may span several NC authority scans.
    // Suspend its driver until the exact batch retires it; source/PDO and
    // all other runtime faults retain the original emergency containment.
    if(fault==2U&&IsG80ResetOutputHeld()) return;
    const bool cancelled=GetCurrentExecutionEpoch()!=m_g80Frozen.command.execution.epoch ||
        !IsMotionOwnerLeaseCurrent(m_g80Frozen.command.ownerLease) || HasPendingSafetyOrRecoveryRequests();
    m_g80Current.runtimeFault=fault;m_g80Current.terminal=true;m_g80Current.completed=false;
    m_g80Input.cancel=cancelled;
    if(fault==1U) m_g80Input.sourceValid=false;
    (void)m_g80Cycle.Step(m_g80Input);
    if(cancelled) AbortTrackedMotionCommand();
    else {
        const auto alarm=fault==4U?static_cast<std::uint32_t>(AlarmManager::G80_SETTLE_TIMEOUT):(fault==1U?static_cast<std::uint32_t>(AlarmManager::G80_SIGNAL_INVALID):
            (fault==2U?static_cast<std::uint32_t>(AlarmManager::G80_FRAME_CHANGED):G80Alarm(m_g80Cycle.Current().fault)));
        FaultTrackedMotionCommand(alarm,MotionRejectReason::NOT_READY);
        AlarmManager::GetInstance().Trigger(alarm,m_g80Frozen.command.sourceLinePC);
    }
    RestoreG80PidLimits();
    m_g80ZeroDemand=true;m_g80Running=false;
    PublishG80Runtime();
    if(emergency) EmergencyStopAllAxesImpl(false);
}

bool MotionCore::StepG80Runtime() noexcept
{
    if(!m_g80Running) return false;
    if(!m_g80Input.sourceValid || !m_g80Input.pdoValid ||
        !MotionG80RuntimeProof::SameInputScope(m_g80InputSession,m_g80InputIdentity,
            m_g80Frozen.request.session,m_g80Frozen.command.execution) ||
        m_g80Input.tick!=m_ncSettleRuntimeCycleTick) {FailG80Runtime(1U,true);return true;}
    if(IsG80ResetOutputHeld()) return true; // No command advance behind Reset's PDO hold.
    if(GetCommandAuthorizationFailure(m_Group.currentCmd)!=MotionRejectReason::NONE ||
        m_Group.enableTransform || m_Group.enableHistory || m_Group.jumpManager.state!=JumpState::IDLE ||
        m_pathHold.sourceSeen || !std::isfinite(m_Group.feedrateOverride) ||
        m_Group.feedrateOverride<0.0 || m_Group.feedrateOverride>1.0) {FailG80Runtime(2U,true);return true;}
    m_g80Input.hold=m_Group.feedrateOverride==0.0;
    const auto output=m_g80Cycle.Step(m_g80Input);
    if(output.emergencyStopRequired) {FailG80Runtime(3U,true);return true;}
    for(int slot=0;slot<m_Group.axisCount;++slot) {
        const unsigned i=static_cast<unsigned>(m_Group.axisIndices[slot]);auto& axis=(*m_pContexts)[i];
        const double position=output.commandMCS[i]*m_g80Frozen.pulsePerNative[i];
        const double velocity=output.velocityNative[i]*m_g80Frozen.pulsePerNative[i];
        if(!std::isfinite(position) || !std::isfinite(velocity) ||
            !m_pCoordMgr->IsTargetWithinSoftwareTravelLimit(axis,output.commandMCS[i])) {
            FailG80Runtime(3U,true);return true;
        }
        axis.logicalCmdPos=axis.currentCmdPos=position;axis.logicalCmdVel=axis.currentCmdVel=velocity;
        axis.planningPos=position;axis.finalTargetPos=position;
        axis.targetVelocity=velocity;axis.targetEndVel=0.0;
    }
    if(output.faulted && output.stopped) {FailG80Runtime(3U,true);return true;}
    if(output.complete && output.stopped && !m_g80Input.hold) {
        m_g80ZeroDemand=true;
        if(++m_g80TerminalWaitTicks>static_cast<std::uint32_t>(std::ceil(
            m_g80Frozen.request.config.settleTimeoutSeconds/G80::SampleSeconds))) {
            FailG80Runtime(4U,true);return true;
        }
        bool settled=m_g80ZeroSentTick!=0ULL && m_g80ZeroSentTick+1ULL==m_g80Input.tick &&
            ArePhysicalGroupAxesReadyForTerminal();
        for(int slot=0;settled && slot<m_Group.axisCount;++slot) {
            const unsigned i=static_cast<unsigned>(m_Group.axisIndices[slot]);
            const double window=i<3U?m_g80Frozen.request.config.settleWindowMm:m_g80Frozen.request.config.settleWindowDeg;
            settled=std::fabs(m_g80Input.actualMCS[i]-output.commandMCS[i])<=window;
        }
        if(settled) ++m_g80ZeroSettleTicks;else m_g80ZeroSettleTicks=0U;
        const auto required=static_cast<std::uint32_t>(std::ceil(m_g80Frozen.request.config.settleHoldSeconds/G80::SampleSeconds));
        if(m_g80ZeroSettleTicks>=(std::max)(1U,required)) {
            LifecycleCommitReservationGuard reservation(*this,m_Group.currentCmd.execution);
            if(reservation.IsAcquired() && !AlarmManager::GetInstance().HasAlarm() &&
                GetCommandAuthorizationFailure(m_Group.currentCmd)==MotionRejectReason::NONE) {
                if(!RetireG80Axes())
                    {reservation.Release();FailG80Runtime(3U,true);return true;}
                const auto feedbackBefore=m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire);
                CompleteTrackedMotionCommand(m_Group.currentCmd);
                if(m_lastPublishedMotionFeedbackSequence.load(std::memory_order_acquire)==feedbackBefore) {
                    reservation.Release();FailG80Runtime(3U,true);return true;
                }
                RestoreG80PidLimits();
                m_Group.isActive=false;m_g80Running=false;
                m_g80Current.terminal=m_g80Current.completed=true;
            }
        }
    }
    PublishG80Runtime();return true;
}

void MotionCore::ObserveG80Send(const MotionServoHandoffSample& sample) noexcept
{
    if(!m_g80Running || !m_g80ZeroDemand) return;
    const auto& command=m_g80Frozen.command;
    const bool exact=MotionG80RuntimeProof::ExactZeroSend(sample,command.execution.epoch,
        command.ownerLease,m_g80Frozen.axisMask,m_ncSettleRuntimeCycleTick);
    m_g80ZeroSentTick=exact?sample.sourceTick:0ULL;
}

bool MotionCore::SelectG80Gains(AxisContext& axis) noexcept
{
    if(!m_g80Running || axis.axisIndex<0 || axis.axisIndex>=6 ||
        !(m_g80Frozen.axisMask&(1U<<static_cast<unsigned>(axis.axisIndex))) ||
        axis.homeRuntime.active || m_safetyControlledStopInProgress ||
        axis.state!=MotionState::MotionState_INTERPOLATING ||
        GetCommandAuthorizationFailure(m_Group.currentCmd)!=MotionRejectReason::NONE) return false;
    const auto& gain=m_g80Frozen.request.config.gains[static_cast<unsigned>(axis.axisIndex)];
    axis.pid.Kp=gain.kp;axis.pid.Ki=gain.ki;axis.pid.Kd=gain.kd;axis.pid.Kvff=gain.kvff;
    axis.pid.MaxIntegral=gain.maxIntegral;
    const auto& saved=m_g80SavedPid[static_cast<unsigned>(axis.axisIndex)];
    const double edgeLimit=gain.followingLimitNative*m_g80Frozen.pulsePerNative[static_cast<unsigned>(axis.axisIndex)];
    axis.pid.MaxLag=saved.EnableLagCheck && saved.MaxLag>0.0?(std::min)(saved.MaxLag,edgeLimit):edgeLimit;
    axis.pid.EnableLagCheck=true;
    return true;
}

bool MotionCore::ApplyG80Derivative(AxisContext& axis,double error,double& velocity) noexcept
{
    if(!m_g80Running || axis.axisIndex<0 || axis.axisIndex>=6 ||
        (m_g80Frozen.axisMask&(1U<<static_cast<unsigned>(axis.axisIndex)))==0U) return true;
    if(axis.homeRuntime.active || m_safetyControlledStopInProgress ||
        axis.state!=MotionState::MotionState_INTERPOLATING ||
        GetCommandAuthorizationFailure(m_Group.currentCmd)!=MotionRejectReason::NONE) return false;
    const unsigned i=static_cast<unsigned>(axis.axisIndex);
    return G80::AddFilteredDerivative(m_g80Frozen.request.config.gains[i].kd,
        m_g80Frozen.request.config.derivativeFilterSeconds,error,G80::SampleSeconds,
        velocity,m_g80Derivative[i],velocity);
}

void MotionCore::RestoreG80PidLimits() noexcept
{
    m_g80Derivative={};
    if(m_pContexts==nullptr) return;
    for(unsigned i=0U;i<6U && i<m_pContexts->size();++i) if(m_g80Frozen.axisMask&(1U<<i)) {
        auto& axis=(*m_pContexts)[i]; const auto& saved=m_g80SavedPid[i];
        axis.pid.MaxIntegral=saved.MaxIntegral;axis.pid.MaxLag=saved.MaxLag;
        axis.pid.EnableLagCheck=saved.EnableLagCheck;axis.pid.integralAcc=0.0;axis.pid.prevError=0.0;
    }
}

bool MotionCore::G80DemandsZero(const AxisContext& axis) const noexcept
{
    return m_g80Running && m_g80ZeroDemand && axis.axisIndex>=0 && axis.axisIndex<6 &&
        (m_g80Frozen.axisMask&(1U<<static_cast<unsigned>(axis.axisIndex)))!=0U;
}

bool MotionCore::CommitG80CompletedEndpoint(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner,G80::Point6& endpointMCS) noexcept
{
    MotionG80Feedback report{};
    if(!ReadG80Feedback(report) || !report.completed || !report.terminal || report.session!=session ||
        !G80SameIdentity(report.identity,identity) || !report.owner.Matches(owner) ||
        m_g80Claim.load(std::memory_order_acquire)!=session ||
        m_g80Barrier.load(std::memory_order_acquire)!=session ||
        !HasExactExecutionDrainAcknowledgement(identity.epoch,owner)) return false;
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired() || !HasExactExecutionDrainAcknowledgementImpl(identity.epoch,owner,true)) return false;
    G80Frozen frozen{};
    if(!m_g80Requests.Read(frozen).Ready() || frozen.request.session!=session ||
        !G80SameIdentity(frozen.command.execution,identity) || !frozen.command.ownerLease.Matches(owner) ||
        !MatchesNCTranslation(frozen.command.sourceTranslation) || report.runtimeFault!=0U ||
        report.cycle.phase!=G80::Phase::Complete || report.cycle.fault!=G80::Fault::None) return false;
    for(unsigned i=0U;i<6U;++i) if(frozen.axisMask&(1U<<i)) {
        const double pulse=report.endpointMCS[i]*frozen.pulsePerNative[i];
        if(!std::isfinite(pulse) || (*m_pContexts)[i].logicalCmdPos.Load()!=pulse) return false;
    }
    for(unsigned i=0U;i<6U;++i) if(frozen.axisMask&(1U<<i)) {
        const double pulse=report.endpointMCS[i]*frozen.pulsePerNative[i];
        m_g00ProducerQueueTailPulse[i]=pulse;(*m_pContexts)[i].lastQueuedPulse.Store(pulse);
        m_g00ProducerQueueTailValidMask|=1U<<i;
    }
    m_g00ProducerQueueTailEpoch=identity.epoch;m_g00ProducerQueueTailOwnerLease=owner;
    endpointMCS=report.endpointMCS;
    m_g80EndpointCommitted.store(session,std::memory_order_release);
    return true;
}

MotionNCTranslationTransitionResult MotionCore::TryCommitG80OriginTranslation(
    const NCTranslationSnapshot& previous,const NCTranslationSnapshot& next,
    MotionExecutionEpoch epoch,const MotionOwnerLease& owner,std::uint64_t session) noexcept
{
    MotionG80Feedback report{}; G80Frozen frozen{};
    if(!G80::IsOriginSnapshotTransition(previous,next) || m_pCoordMgr==nullptr)
        return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;
    if(!ReadG80Feedback(report) || !m_g80Requests.Read(frozen).Ready())
        return MotionNCTranslationTransitionResult::DEFERRED;
    if(!report.completed || !report.terminal || !report.endpointCommitted || report.session!=session ||
        report.runtimeFault!=0U || report.cycle.phase!=G80::Phase::Complete || report.cycle.fault!=G80::Fault::None ||
        report.identity.epoch!=epoch || !report.owner.Matches(owner) || frozen.request.session!=session ||
        m_g80Claim.load(std::memory_order_acquire)!=session ||
        m_g80Barrier.load(std::memory_order_acquire)!=session ||
        !G80SameIdentity(frozen.command.execution,report.identity) || !frozen.command.ownerLease.Matches(owner) ||
        !SameNCTranslationSnapshot(frozen.command.sourceTranslation,previous) || !MatchesNCTranslation(previous))
        return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;
    if(!HasExactExecutionDrainAcknowledgement(epoch,owner)) return MotionNCTranslationTransitionResult::DEFERRED;
    LifecycleCommitReservationGuard reservation(*this,report.identity);
    if(!reservation.IsAcquired() || !HasExactExecutionDrainAcknowledgementImpl(epoch,owner,true))
        return MotionNCTranslationTransitionResult::DEFERRED;
    if(!MatchesNCTranslation(previous) || !m_pCoordMgr->CommitG80OriginTransition(previous,next))
        return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;
    // Both stores remain inside the same lifecycle reservation. NC owns the
    // coordinate table writer, and no active/queued geometry can observe half.
    m_translationPublication.Retire();
    if(PublishNCTranslation(next)) return MotionNCTranslationTransitionResult::ACCEPTED;
    reservation.Release();
    AlarmManager::GetInstance().Trigger(AlarmManager::G80_FRAME_CHANGED,frozen.command.sourceLinePC);
    RequestEmergencyStopAllAxes();
    return MotionNCTranslationTransitionResult::UNSUPPORTED_PREDECESSOR;
}

bool MotionCore::IsG80BarrierBlocking(const MotionCommand& command) const noexcept
{
    const auto session=m_g80Barrier.load(std::memory_order_acquire);
    if(!session || (command.execution.source!=MotionCommandSource::NC_MEMORY &&
        command.execution.source!=MotionCommandSource::NC_MDI)) return false;
    G80Frozen frozen{};
    if(!m_g80Requests.Read(frozen).Ready()) return true;
    return frozen.request.session==session && command.execution.epoch==frozen.command.execution.epoch &&
        command.ownerLease.Matches(frozen.command.ownerLease);
}

bool MotionCore::ReleaseG80Barrier(std::uint64_t session,const MotionExecutionIdentity& identity,
    const MotionOwnerLease& owner) noexcept
{
    MotionG80Feedback report{};
    if(!ReadG80Feedback(report) || !report.completed || !report.terminal || !report.endpointCommitted ||
        report.session!=session || !G80SameIdentity(report.identity,identity) || !report.owner.Matches(owner) ||
        !HasExactExecutionDrainAcknowledgement(identity.epoch,owner)) return false;
    LifecycleCommitReservationGuard reservation(*this,identity);
    if(!reservation.IsAcquired() || !HasExactExecutionDrainAcknowledgementImpl(identity.epoch,owner,true)) return false;
    auto expected=session;
    return m_g80Barrier.compare_exchange_strong(expected,0ULL,std::memory_order_acq_rel,std::memory_order_acquire);
}
