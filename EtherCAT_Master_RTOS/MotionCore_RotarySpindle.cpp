#include "MotionCore.h"
#include "AlarmManager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
bool SpindleSafetyPacket(const NCSpindleDispatch::Packet& p) noexcept
{
    return p.signedRpm==0.0&&(p.kind==NCSpindleDispatch::Kind::AlarmStop||
        p.kind==NCSpindleDispatch::Kind::ResetStop||p.kind==NCSpindleDispatch::Kind::AuthorityLostStop);
}
int SpindleCode(unsigned i,double rpm) noexcept
{
    const int codes[3][3]={{3,4,5},{13,14,15},{16,17,18}};
    return codes[i][rpm>0.0?0U:rpm<0.0?1U:2U];
}
bool SpindlePacketShape(const NCSpindleDispatch::Packet& p) noexcept
{
    if(!p.authority.Valid()||!p.sequence||p.spindle>=3U||p.axisSlot>=8U||!std::isfinite(p.signedRpm))return false;
    if(p.kind==NCSpindleDispatch::Kind::Command){
        const auto code=NCSpindleCodes::Decode(p.mCode);
        return code.valid&&code.spindle==p.spindle&&p.row.Valid()&&p.commitSequence&&
            !p.lifecycleSequence&&!p.startSequence&&p.mCode==SpindleCode(p.spindle,p.signedRpm);
    }
    if(!p.lifecycleSequence||p.commitSequence||p.mCode!=-1||p.row.dispatchId)return false;
    if(p.kind==NCSpindleDispatch::Kind::Resume)return p.startSequence!=0ULL;
    return p.signedRpm==0.0&&!p.startSequence&&
        (p.kind==NCSpindleDispatch::Kind::HoldStop||p.kind==NCSpindleDispatch::Kind::EndStop||SpindleSafetyPacket(p));
}
}

bool MotionCore::IsRotarySpindleSessionClaimed() const noexcept
{return m_rotarySpindleClaim.load(std::memory_order_acquire)!=0ULL;}

bool MotionCore::IsRotarySpindleAuthorityCurrent(const RotarySpindleFrozen& f) const noexcept
{
    const MotionOwnerLease owner{MotionOwner::AUTO,static_cast<MotionOwnerGeneration>(f.authority.scope.ownerGeneration)};
    return f.authority.Valid()&&f.authority.scope.epoch<=UINT32_MAX&&f.authority.scope.ownerGeneration<=UINT32_MAX&&
        GetCurrentExecutionEpoch()==f.authority.scope.epoch&&IsMotionOwnerLeaseCurrent(owner)&&
        !HasPendingExecutionEpochChange()&&!HasPendingSafetyOrRecoveryRequests()&&
        !HasUnacknowledgedSafetyMotionRequest()&&!AlarmManager::GetInstance().HasAlarm();
}

bool MotionCore::IsRotarySpindleConfigurationCurrent(const RotarySpindleFrozen& f) const noexcept
{
    if(!m_pContexts||!m_pDrives||m_pContexts->size()!=m_pDrives->size()||m_pContexts->size()>8U||
        m_pDrives->size()!=f.slots)return false;
    for(unsigned slot=0;slot<f.slots;++slot){
        const auto& drive=(*m_pDrives)[slot];
        if(!drive.pInput||!drive.pOutput||reinterpret_cast<std::uintptr_t>(drive.pInput)!=f.inputMapping[slot]||
            reinterpret_cast<std::uintptr_t>(drive.pOutput)!=f.outputMapping[slot])return false;
        for(unsigned other=0;other<slot;++other)if(f.inputMapping[slot]==f.inputMapping[other]||
            f.outputMapping[slot]==f.outputMapping[other])return false;
    }
    for(unsigned i=0U;i<3U;++i){
        const auto& c=f.config[i];if(!c.enabled)continue;
        if(c.axisSlot>=m_pContexts->size())return false;
        const auto& a=(*m_pContexts)[c.axisSlot];const auto& b=f.binding[i];
        const unsigned travel=(a.travelLimit1Enable?1U:0U)|(a.travelLimit2Enable?2U:0U)|(a.travelLimit3Enable?4U:0U);
        if(a.axisIndex!=static_cast<int>(c.axisSlot)||!a.isExist||a.isVirtualAxis||
            (a.axisType!=AxisType::ROTARY&&a.axisType!=AxisType::ROTARY_CONTINUOUS)||
            static_cast<std::uint32_t>(a.axisType)!=b.axisType||a.fbMode!=FeedbackSource::MOTOR_ENCODER||
            a.enablePitch||a.enableBacklash||a.currentCompOffset_unit!=0.0||
            a.resolution_PPR!=b.resolution||a.finalLead!=b.lead||a.maxVel_PPS!=b.maxVelocity||
            a.machineCoordinateOffsetPulse!=b.machineOffset||
            static_cast<unsigned>((a.isReverse?1U:0U)|(a.Axis_Reverse?2U:0U))!=b.reverseFlags||
            reinterpret_cast<std::uintptr_t>((*m_pDrives)[c.axisSlot].pOutput)!=b.output||
            !b.output||travel!=b.travelMask)return false;
        const double low[3]={a.travelLimit1Negative_unit,a.travelLimit2Negative_unit,a.travelLimit3Negative_unit};
        const double high[3]={a.travelLimit1Positive_unit,a.travelLimit2Positive_unit,a.travelLimit3Positive_unit};
        for(unsigned k=0;k<3;++k)if((travel&(1U<<k))&&(low[k]!=b.travelLow[k]||high[k]!=b.travelHigh[k]))return false;
    }
    return true;
}

bool MotionCore::BeginRotarySpindleSession(const NCSpindleDispatch::Authority& authority,
    const NCSpindleDispatch::Config& config) noexcept
{
    if(!authority.Valid()||authority.scope.runToken<=m_rotarySpindleProducerRun||IsRotarySpindleSessionClaimed()||
        authority.scope.epoch>UINT32_MAX||authority.scope.ownerGeneration>UINT32_MAX||
        !m_pContexts||!m_pDrives||m_pContexts->size()!=m_pDrives->size()||m_pContexts->size()>8U)return false;
    RotarySpindleFrozen f{};f.authority=authority;f.config=config;f.slots=static_cast<std::uint32_t>(m_pDrives->size());
    for(unsigned slot=0;slot<f.slots;++slot){
        f.inputMapping[slot]=reinterpret_cast<std::uintptr_t>((*m_pDrives)[slot].pInput);
        f.outputMapping[slot]=reinterpret_cast<std::uintptr_t>((*m_pDrives)[slot].pOutput);
    }
    if(!IsRotarySpindleAuthorityCurrent(f))return false;
    if(GetQueueSize()||GetCommandIngressSize()||GetCommandReplaySize()||m_axisCommandChannel.command_size())return false;
    for(unsigned i=0U;i<3U;++i){
        const auto& c=config[i];if(!NCRotarySpindle::ValidConfig(c,i))return false;if(!c.enabled)continue;
        if(c.axisSlot>=m_pContexts->size()||(f.axisMask&(1U<<c.axisSlot)))return false;
        const auto& a=(*m_pContexts)[c.axisSlot];auto& b=f.binding[i];
        if(!std::isfinite(a.finalLead)||a.finalLead<=0.0||!std::isfinite(a.resolution_PPR)||a.resolution_PPR<=0.0||
            !std::isfinite(a.maxVel_PPS)||a.maxVel_PPS<=0.0||!std::isfinite(a.machineCoordinateOffsetPulse)||
            a.isFirstCycle||!a.startupLagMonitorArmed||!a.isServoOn||a.targetMode!=9||a.isFault||a.isLagAlarm||a.resetRequest||
            a.homeRuntime.active||a.state!=MotionState::MotionState_IDLE||a.hardLimitPositive||a.hardLimitNegative)return false;
        const double ramp=c.maximumRpm/(std::min)(c.accelerationRpmPerSecond,
            (std::min)(c.decelerationRpmPerSecond,c.stopDecelerationRpmPerSecond));
        if(!std::isfinite(ramp)||ramp>7200.0||c.pulsesPerOutputRevolution!=360.0*(a.resolution_PPR/a.finalLead)||
            c.maximumRpm/60.0*c.pulsesPerOutputRevolution>a.maxVel_PPS)return false;
        b.resolution=a.resolution_PPR;b.lead=a.finalLead;b.maxVelocity=a.maxVel_PPS;
        b.machineOffset=a.machineCoordinateOffsetPulse;b.axisType=static_cast<std::uint32_t>(a.axisType);
        b.reverseFlags=static_cast<std::uint8_t>((a.isReverse?1U:0U)|(a.Axis_Reverse?2U:0U));
        b.output=reinterpret_cast<std::uintptr_t>((*m_pDrives)[c.axisSlot].pOutput);
        b.travelMask=(a.travelLimit1Enable?1U:0U)|(a.travelLimit2Enable?2U:0U)|(a.travelLimit3Enable?4U:0U);
        b.travelLow={{a.travelLimit1Negative_unit,a.travelLimit2Negative_unit,a.travelLimit3Negative_unit}};
        b.travelHigh={{a.travelLimit1Positive_unit,a.travelLimit2Positive_unit,a.travelLimit3Positive_unit}};
        b.lowPulse=-(std::numeric_limits<double>::max)();b.highPulse=(std::numeric_limits<double>::max)();
        for(unsigned k=0;k<3;++k)if(b.travelMask&(1U<<k)){
            const double lo=b.travelLow[k]*(a.resolution_PPR/a.finalLead),hi=b.travelHigh[k]*(a.resolution_PPR/a.finalLead);
            if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi)return false;
            b.lowPulse=(std::max)(b.lowPulse,lo);b.highPulse=(std::min)(b.highPulse,hi);
        }
        if(b.lowPulse>=b.highPulse)return false;
        f.axisMask|=1U<<c.axisSlot;
    }
    if(!f.axisMask||!IsRotarySpindleConfigurationCurrent(f))return false;
    for(const auto& a:*m_pContexts)if(a.isExist&&(a.homeRuntime.active||a.state!=MotionState::MotionState_IDLE))return false;
    // This is inert staging. NIC/lifecycle reservation contention must not
    // turn a healthy first M into an alarm. RT owns actual acceptance after
    // its fresh 200-sample zero barrier and repeats all authority/source gates.
    if(!IsRotarySpindleAuthorityCurrent(f)||!IsRotarySpindleConfigurationCurrent(f))return false;
    if(!m_rotarySpindleBegin.Publish(f))return false;
    m_rotarySpindleRelease.store(0ULL,std::memory_order_release);
    m_rotarySpindleClaim.store(authority.scope.runToken,std::memory_order_release);
    m_rotarySpindleProducerRun=authority.scope.runToken;
    return true;
}

bool MotionCore::SubmitRotarySpindlePacket(const NCSpindleDispatch::Packet& packet) noexcept
{
    if(!SpindlePacketShape(packet)||m_rotarySpindleClaim.load(std::memory_order_acquire)!=packet.authority.scope.runToken)return false;
    RotarySpindleFrozen frozen{};
    if(!m_rotarySpindleBegin.Read(frozen).Ready()||!frozen.authority.Same(packet.authority)||
        !frozen.config[packet.spindle].enabled||packet.axisSlot!=frozen.config[packet.spindle].axisSlot||
        std::fabs(packet.signedRpm)>frozen.config[packet.spindle].maximumRpm)return false;
    const auto i=packet.spindle;
    if(packet.sequence==m_rotarySpindleProducerSequence[i]){
        NCSpindleDispatch::Packet prior{};
        return m_rotarySpindleRequest[i].Read(prior).Ready()&&prior.Same(packet);
    }
    if(packet.sequence<m_rotarySpindleProducerSequence[i]||
        (!SpindleSafetyPacket(packet)&&!IsRotarySpindleAuthorityCurrent(frozen)))return false;
    if(!m_rotarySpindleRequest[i].Publish(packet))return false;
    m_rotarySpindleProducerSequence[i]=packet.sequence;return true;
}

bool MotionCore::ReadRotarySpindleFeedback(MotionRotarySpindleFeedback& out) const noexcept
{return m_rotarySpindleFeedback.Read(out).Ready();}

bool MotionCore::ReleaseRotarySpindleSession(const NCSpindleDispatch::Authority& authority) noexcept
{
    MotionRotarySpindleFeedback feedback{};
    if(!ReadRotarySpindleFeedback(feedback)||!feedback.authority.Same(authority)||!feedback.allStopped||!feedback.handbackReady)return false;
    if(feedback.active){m_rotarySpindleRelease.store(authority.scope.runToken,std::memory_order_release);return false;}
    if(IsRotarySpindleSessionClaimed()||!m_pContexts||m_pContexts->size()>8U)return false;
    for(unsigned i=0;i<m_pContexts->size();++i)if(feedback.axisMask&(1U<<i)){
        auto& axis=(*m_pContexts)[i];const double pulse=feedback.handbackPulse[i];
        if(!std::isfinite(pulse)||axis.logicalCmdPos.Load()!=pulse)return false;
    }
    for(unsigned i=0;i<m_pContexts->size();++i)if(feedback.axisMask&(1U<<i)){
        const double pulse=feedback.handbackPulse[i];m_g00ProducerQueueTailPulse[i]=pulse;
        (*m_pContexts)[i].lastQueuedPulse.Store(pulse);m_g00ProducerQueueTailValidMask|=1U<<i;
    }
    m_g00ProducerQueueTailEpoch=GetCurrentExecutionEpoch();m_g00ProducerQueueTailOwnerLease=GetMotionOwnerLease();return true;
}

void MotionCore::ContainRotarySpindleRuntime(MotionRotarySpindle::Fault fault,bool requestSafety) noexcept
{
    auto& r=m_rotarySpindle;if(!IsRotarySpindleSessionClaimed())return;
    const bool first=!r.fault;r.fault=true;
    if(!r.faultCode)r.faultCode=static_cast<std::uint32_t>(fault);
    r.hardwarePps.fill(0);r.commandRpm.fill(0.0);
    if(first){
        r.handback=false;r.zeroSamples.fill(0U);r.feedback.receipt={};
        if(r.initialized){NCRotarySpindle::Input cancel{};cancel.scope=r.frozen.authority.scope;
            cancel.lifecycle=NCRotarySpindle::Lifecycle::Alarm;(void)r.controller.Step(cancel);}
    }
    if(first&&requestSafety){RequestEmergencyStopAllAxes();AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY);}
}

void MotionCore::ObserveRotarySpindleRuntimeInput() noexcept
{
    const auto claim=m_rotarySpindleClaim.load(std::memory_order_acquire);auto& r=m_rotarySpindle;
    if(!claim)return;
    if(!r.loaded||r.frozen.authority.scope.runToken!=claim){
        RotarySpindleFrozen frozen{};if(!m_rotarySpindleBegin.Read(frozen).Ready()||frozen.authority.scope.runToken!=claim)return;
        // Preserve the numerical controller's run/sequence high-water marks.
        const auto controller=r.controller;const auto publication=r.publication;r=RotarySpindleRuntime{};
        r.controller=controller;r.publication=publication;r.frozen=frozen;r.loaded=true;r.beginTick=m_ncSettleRuntimeCycleTick;
    }
    const auto previousTick=r.tick;r.tick=m_ncSettleRuntimeCycleTick;
    const bool fresh=m_ncSettleRuntimeObserved&&m_ncSettleRuntimeCycleValid&&m_ncSettleRuntimeCycleContiguous&&r.tick&&
        (!previousTick||r.tick==previousTick+1ULL);
    const bool configuration=IsRotarySpindleConfigurationCurrent(r.frozen);
    const bool authority=IsRotarySpindleAuthorityCurrent(r.frozen);
    if(!fresh)ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Source,true);
    if(!configuration)ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Configuration,true);
    if(!authority)ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Authority,
        GetMotionOwnerLease().owner!=MotionOwner::SAFETY&&!HasPendingSafetyOrRecoveryRequests());
    bool allSource=fresh&&configuration;
    NCRotarySpindle::Input input{};input.scope=r.frozen.authority.scope;input.tick=r.tick;input.physicalSequence=r.tick;
    input.lifecycle=NCRotarySpindle::Lifecycle::Run;input.pdoValid=fresh;input.configurationCurrent=configuration;
    input.memoryProgram=input.programCommitted=true;
    for(unsigned i=0;i<3U;++i){
        const auto& c=r.frozen.config[i];if(!c.enabled)continue;
        bool valid=fresh&&configuration&&ReadMotionServoInputCompatibilitySnapshotBySlot(static_cast<int>(c.axisSlot),r.input[i]);
        if(valid){
            const auto& a=(*m_pContexts)[c.axisSlot];const auto& p=r.input[i];
            valid=!a.isFirstCycle&&(p.StatusWord&0x006FU)==0x0027U&&p.ModesOfOperationDisplay==9&&
                a.startupLagMonitorArmed&&a.isServoOn&&a.targetMode==9;
            const auto raw=static_cast<std::uint32_t>(p.ActualPosition);double rpm=0.0;
            const bool delta=r.rawSeen[i]&&r.rawTick[i]+1ULL==r.tick&&fresh&&
                MotionRotarySpindleProof::DecodeRpm(raw,r.previousRaw[i],c.pulsesPerOutputRevolution,a.isReverse,a.Axis_Reverse,rpm);
            bool unwrap=false;
            if(!r.rawTick[i]){
                r.rawUnwrapped[i]=a.unwrappedActPos+static_cast<double>(MotionRotarySpindleProof::RawDelta(raw,static_cast<std::uint32_t>(a.lastRawActPos)));
                unwrap=std::isfinite(r.rawUnwrapped[i]);
            }else if(r.tick>r.rawTick[i]){
                // The safety loop may skip UpdateMotion while our physical
                // source hook continues. Keep its own unwrapped encoder, and
                // reject gaps large enough to make a 32-bit turn ambiguous.
                const double maximumGapPulse=a.maxVel_PPS*NCRotarySpindle::CycleSeconds*static_cast<double>(r.tick-r.rawTick[i]);
                unwrap=std::isfinite(maximumGapPulse)&&maximumGapPulse<2147483648.0&&raw-r.previousRaw[i]!=0x80000000U;
                if(unwrap)r.rawUnwrapped[i]+=static_cast<double>(MotionRotarySpindleProof::RawDelta(raw,r.previousRaw[i]));
            }
            if(unwrap){r.previousRaw[i]=raw;r.rawTick[i]=r.tick;}
            r.rawSeen[i]=unwrap;r.actualRpm[i]=rpm;
            r.actualPulse[i]=r.rawUnwrapped[i]*(a.isReverse!=a.Axis_Reverse?-1.0:1.0)-a.machineCoordinateOffsetPulse;
            valid=valid&&delta&&unwrap&&std::isfinite(r.actualPulse[i]);
            if(a.isFault||a.isLagAlarm||a.resetRequest||a.homeRuntime.active||a.hardLimitPositive||a.hardLimitNegative||
                a.state!=MotionState::MotionState_IDLE)
                ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Source,
                    GetMotionOwnerLease().owner!=MotionOwner::SAFETY&&!HasPendingSafetyOrRecoveryRequests());
        }else r.rawSeen[i]=false;
        r.sourceValid[i]=valid;allSource=allSource&&valid;
        const auto& sent=r.sent[i];const auto currentOwner=GetMotionOwnerLease();
        const auto currentTicket=UnpackMotionOwnerSafetyRequestTicket(m_motionOwnerState.load(std::memory_order_acquire));
        const bool sameSendScope=sent.safetyZero?
            (currentOwner.owner==MotionOwner::SAFETY&&currentOwner.Matches(sent.owner)&&
                GetCurrentExecutionEpoch()==sent.epoch&&currentTicket==sent.safetyTicket):authority;
        const bool prior=valid&&sent.exact&&sent.tick+1ULL==r.tick&&sameSendScope;
        const bool zero=prior&&sent.commandRpm==0.0&&std::fabs(r.actualRpm[i])<=c.stoppedToleranceRpm;
        r.zeroSamples[i]=zero?(std::min)(r.zeroSamples[i]+1U,NCRotarySpindle::RequiredSamples):0U;
        input.axis[i].actualRpm=r.actualRpm[i];input.axis[i].sourceValid=valid;
        input.axis[i].positioningActive=m_Group.isActive||(*m_pContexts)[c.axisSlot].homeRuntime.active;
        input.axis[i].finalZeroSent=zero;input.axis[i].zeroSendTick=sent.tick;input.axis[i].zeroSendRequestSequence=sent.request;
        if(r.havePacket[i]){
            auto& receipt=r.feedback.receipt[i];
            if(r.window[i].Observe(r.tick,r.tick,r.actualRpm[i],sent.commandRpm,prior,sent.tick,sent.request,
                configuration,authority,c.stoppedToleranceRpm,c.readyToleranceRpm)&&r.window[i].Populate(receipt)){
                receipt.publicationSequence=++r.publication;receipt.accepted=true;receipt.authorityCurrent=authority;
                receipt.ready=r.packet[i].signedRpm!=0.0&&!r.fault;receipt.stopped=r.packet[i].signedRpm==0.0;
                r.qualifiedSequence[i]=r.packet[i].sequence;
                if(!authority&&SpindleSafetyPacket(r.packet[i])&&sent.safetyZero){
                    receipt.containment.causalLifecycleSequence=r.packet[i].lifecycleSequence;
                    receipt.containment.safetyEpoch=sent.epoch;receipt.containment.safetyOwnerGeneration=sent.owner.generation;
                    receipt.containment.safetyStopTicket=sent.safetyTicket;receipt.containment.finalSafetyZeroValidated=true;
                }
            }else receipt=NCSpindleDispatch::Receipt{};
        }
    }
    // A first encoder sample establishes a delta baseline only. It cannot
    // authorize motion, invalidate HOME or contribute a physical stop sample.
    if(!r.initialized&&allSource&&authority&&!r.fault){
        r.initialized=r.controller.Begin(input.scope,r.frozen.config,input);
    }else if(r.initialized&&!r.fault&&!allSource){
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Source,true);
    }
    if(!r.initialized&&!r.fault&&r.tick-r.beginTick>20000ULL)
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Timeout,true);
    if(m_Group.isActive||GetQueueSize()||GetCommandIngressSize()||GetCommandReplaySize())
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Positioning,true);
    if(!r.fault&&m_pContexts)for(const auto& axis:*m_pContexts)if(axis.isExist&&
        (axis.homeRuntime.active||axis.state!=MotionState::MotionState_IDLE))
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Positioning,true);
    std::array<NCRotarySpindle::Request,3> requests{};std::array<bool,3> present{};
    std::array<NCSpindleDispatch::Packet,3> incoming{};
    for(unsigned i=0;i<3U;++i){
        const auto& c=r.frozen.config[i];if(!c.enabled)continue;
        NCSpindleDispatch::Packet packet{};if(!m_rotarySpindleRequest[i].Read(packet).Ready())continue;
        if(!packet.authority.Same(r.frozen.authority))continue; // Retired run's mailbox image.
        if(packet.sequence!=r.deadlineSequence[i]){
            const double ramp=c.maximumRpm/(std::min)(c.accelerationRpmPerSecond,
                (std::min)(c.decelerationRpmPerSecond,c.stopDecelerationRpmPerSecond));
            const auto ticks=static_cast<std::uint64_t>(std::ceil((2.0*ramp+5.0)/NCRotarySpindle::CycleSeconds));
            r.deadlineSequence[i]=packet.sequence;r.deadlineTick[i]=r.tick+ticks;
        }
        if(!r.fault&&r.qualifiedSequence[i]!=packet.sequence&&r.tick>r.deadlineTick[i])
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Timeout,true);
        if(r.havePacket[i]&&packet.sequence==r.packet[i].sequence){
            if(!packet.Same(r.packet[i]))ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Mailbox,true);
            continue;
        }
        if(!SpindlePacketShape(packet)||packet.spindle!=i||packet.axisSlot!=c.axisSlot||
            (r.havePacket[i]&&packet.sequence<r.packet[i].sequence)||std::fabs(packet.signedRpm)>c.maximumRpm){
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Admission,true);continue;
        }
        if(!r.fault&&r.havePacket[i]&&packet.kind==NCSpindleDispatch::Kind::Command&&
            (r.packet[i].kind==NCSpindleDispatch::Kind::HoldStop||r.packet[i].kind==NCSpindleDispatch::Kind::EndStop||
                SpindleSafetyPacket(r.packet[i]))){
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Admission,true);continue;
        }
        if(r.fault){
            if(!SpindleSafetyPacket(packet))continue;
            r.packet[i]=packet;r.havePacket[i]=true;r.window[i].Reset(packet,r.tick);r.feedback.receipt[i]={};
            continue;
        }
        if(!r.initialized||!allSource)continue;
        if(packet.kind==NCSpindleDispatch::Kind::Resume){
            if(packet.startSequence<=r.startFloor)continue;
            bool stopped=true,completeBatch=true;
            for(unsigned other=0;other<3U;++other)if(r.frozen.config[other].enabled){
                stopped=stopped&&r.havePacket[other]&&r.packet[other].kind==NCSpindleDispatch::Kind::HoldStop&&
                    r.packet[other].lifecycleSequence==packet.lifecycleSequence&&
                    r.zeroSamples[other]>=NCRotarySpindle::RequiredSamples&&r.commandRpm[other]==0.0;
                NCSpindleDispatch::Packet peer{};
                completeBatch=completeBatch&&m_rotarySpindleRequest[other].Read(peer).Ready()&&
                    peer.authority.Same(packet.authority)&&peer.kind==NCSpindleDispatch::Kind::Resume&&
                    peer.startSequence==packet.startSequence&&peer.lifecycleSequence==packet.lifecycleSequence;
            }
            // A START before physical hold completion is consumed, not saved
            // for a delayed automatic restart. A split mailbox publication
            // merely waits for the complete same-event batch on the next tick.
            if(!stopped){r.startFloor=packet.startSequence;continue;}
            if(!completeBatch)continue;
        }
        const auto& old=r.controller.Current().axis[i];
        if(packet.signedRpm!=0.0&&old.phase==NCRotarySpindle::Phase::Stopped&&r.zeroSamples[i]<NCRotarySpindle::RequiredSamples)continue;
        requests[i].scope=input.scope;requests[i].sequence=packet.sequence;
        requests[i].mCode=SpindleCode(i,packet.signedRpm);requests[i].hasSpeed=packet.signedRpm!=0.0;
        requests[i].rpm=std::fabs(packet.signedRpm);present[i]=true;incoming[i]=packet;
    }
    bool hasResume=false,completeResume=true;std::uint64_t resumeStart=0ULL;
    for(unsigned i=0;i<3U;++i)if(present[i]&&incoming[i].kind==NCSpindleDispatch::Kind::Resume){hasResume=true;resumeStart=incoming[i].startSequence;}
    if(hasResume)for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled)
        completeResume=completeResume&&present[i]&&incoming[i].kind==NCSpindleDispatch::Kind::Resume&&incoming[i].startSequence==resumeStart;
    if(hasResume&&!completeResume)for(unsigned i=0;i<3U;++i)if(present[i]&&incoming[i].kind==NCSpindleDispatch::Kind::Resume)present[i]=false;
    if(r.initialized&&!r.fault&&r.controller.Current().tick!=r.tick){
        const auto& state=r.controller.StepBatch(input,requests,present);
        if(state.fault!=NCRotarySpindle::Code::None)ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Admission,true);
        for(unsigned i=0;i<3U;++i){
            const auto& c=r.frozen.config[i];if(!c.enabled)continue;const auto& out=state.axis[i];
            if(present[i]){
                if(out.requestSequence!=requests[i].sequence){ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Admission,true);continue;}
                r.packet[i]=incoming[i];r.havePacket[i]=true;r.window[i].Reset(incoming[i],r.tick);r.feedback.receipt[i]={};r.handback=false;
                if(incoming[i].kind==NCSpindleDispatch::Kind::Resume)r.startFloor=incoming[i].startSequence;
            }
            if(out.invalidateHome)(*m_pContexts)[c.axisSlot].isHomed=false;
            r.commandRpm[i]=out.commandRpm;
            const double hardware=out.pulsesPerSecond*((r.frozen.binding[i].reverseFlags==1U||r.frozen.binding[i].reverseFlags==2U)?-1.0:1.0);
            const double cap=c.maximumRpm/60.0*c.pulsesPerOutputRevolution;
            if(!std::isfinite(hardware)||std::fabs(hardware)>cap||std::fabs(hardware)>r.frozen.binding[i].maxVelocity||std::fabs(hardware)>INT32_MAX)
                ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Output,true);
            else r.hardwarePps[i]=static_cast<std::int32_t>(hardware);
        }
    }
    if(r.fault){r.commandRpm.fill(0.0);r.hardwarePps.fill(0);}
    bool allStopped=allSource;
    for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled)
        allStopped=allStopped&&r.commandRpm[i]==0.0&&r.zeroSamples[i]>=NCRotarySpindle::RequiredSamples;
    if(allStopped&&!r.handback)r.handback=HandbackRotarySpindleAxes(!authority);
    if(!allStopped)r.handback=false;
    r.feedback.allStopped=allStopped;r.feedback.handbackReady=r.handback;
    if(r.handback&&m_rotarySpindleRelease.load(std::memory_order_acquire)==claim&&
        (r.fault||r.controller.RetireStopped(r.frozen.authority.scope))){
        // A HOLD/NC scan may outlive the first stopped window. Rebind the
        // complete raw reference to this final fresh sample before releasing
        // the claim; never hand ordinary PID an older stopped position.
        if(HandbackRotarySpindleAxes(!authority)){
            r.feedback.active=false;m_rotarySpindleClaim.store(0ULL,std::memory_order_release);
        }else{
            r.feedback.active=true;r.handback=false;r.feedback.handbackReady=false;
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Handback,true);
        }
    }else r.feedback.active=true;
    PublishRotarySpindleRuntime();
}

bool MotionCore::HandbackRotarySpindleAxes(bool safety) noexcept
{
    auto& r=m_rotarySpindle;if(!m_pContexts||m_pContexts->size()>8U||!IsRotarySpindleConfigurationCurrent(r.frozen))return false;
    for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled){
        if(!r.sourceValid[i]||!std::isfinite(r.actualPulse[i])||r.zeroSamples[i]<NCRotarySpindle::RequiredSamples)return false;
    }
    for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled){
        auto& a=(*m_pContexts)[r.frozen.config[i].axisSlot];const double pulse=r.actualPulse[i];
        a.lastRawActPos=r.previousRaw[i];a.unwrappedActPos=r.rawUnwrapped[i];a.currentActPos=pulse;a.lastActPos=pulse;
        a.currentActVel=r.actualRpm[i]/60.0*r.frozen.config[i].pulsesPerOutputRevolution;
        a.currentCmdPos=pulse;a.logicalCmdPos.Store(pulse);a.planningPos=pulse;a.finalTargetPos=pulse;a.startCmdPos=pulse;
        a.currentCmdVel=a.logicalCmdVel=a.targetVelocity=a.targetEndVel=a.cruiseVel_PPS=a.programmedVel_PPS=0.0;
        a.bufferSum=0.0;a.bufferIndex=0;std::fill(a.velBuffer.begin(),a.velBuffer.end(),0.0);
        a.pid.integralAcc=a.pid.prevError=a.Pid_IDLE.integralAcc=a.Pid_IDLE.prevError=a.Pid_G00.integralAcc=a.Pid_G00.prevError=0.0;
        if(!safety&&!a.isFault&&!a.isLagAlarm&&a.state!=MotionState::MotionState_ESTOP&&a.state!=MotionState::MotionState_ERROR)
            a.state=MotionState::MotionState_IDLE;
        a.inPosition=!safety;
    }
    for(unsigned slot=0;slot<m_pContexts->size();++slot){
        const auto& a=(*m_pContexts)[slot];const double pulse=a.logicalCmdPos.Load();
        r.feedback.handbackPulse[slot]=pulse;
        r.feedback.handbackMcsUnit[slot]=a.isExist&&a.resolution_PPR>0.0?pulse*a.finalLead/a.resolution_PPR:0.0;
    }
    return true;
}

bool MotionCore::ApplyRotarySpindleAxis(AxisContext& axis,const MotionServoInputSnapshot& input,
    std::int32_t& hardwarePps) noexcept
{
    hardwarePps=0;const auto claim=m_rotarySpindleClaim.load(std::memory_order_acquire);auto& r=m_rotarySpindle;
    if(!claim)return false;
    // NC may publish a claim after this pass captured its physical inputs.
    // Until the next source callback adopts it, every axis is zero-only.
    // This is neither an accepted start nor a source failure.
    if(!r.loaded||r.frozen.authority.scope.runToken!=claim)return true;
    unsigned i=0;for(;i<3U;++i)if(r.frozen.config[i].enabled&&r.frozen.config[i].axisSlot==static_cast<unsigned>(axis.axisIndex))break;
    if(i==3U)return false;
    const bool firstDeltaPending=!r.initialized&&r.rawSeen[i]&&m_ncSettleRuntimeCycleValid&&m_ncSettleRuntimeCycleContiguous;
    if(r.tick!=m_ncSettleRuntimeCycleTick||(!r.sourceValid[i]&&!firstDeltaPending)||input.ActualPosition!=r.input[i].ActualPosition||
        input.StatusWord!=r.input[i].StatusWord||input.ModesOfOperationDisplay!=r.input[i].ModesOfOperationDisplay||
        !IsRotarySpindleConfigurationCurrent(r.frozen))ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Source,true);
    if(!IsRotarySpindleAuthorityCurrent(r.frozen))ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Authority,
        GetMotionOwnerLease().owner!=MotionOwner::SAFETY&&!HasPendingSafetyOrRecoveryRequests());
    if(axis.isFault||axis.isLagAlarm||axis.resetRequest||axis.homeRuntime.active||axis.hardLimitPositive||axis.hardLimitNegative||
        axis.state!=MotionState::MotionState_IDLE||
        (input.StatusWord&0x006FU)!=0x0027U||input.ModesOfOperationDisplay!=9)
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Source,true);
    const auto& b=r.frozen.binding[i];
    // Installed finite envelopes remain binding after HOME is invalidated.
    // Continuous rotation does not silently disable machine travel limits.
    if((r.commandRpm[i]>0.0&&axis.currentActPos>=b.highPulse)||
        (r.commandRpm[i]<0.0&&axis.currentActPos<=b.lowPulse))
        ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Configuration,true);
    if(!r.fault)hardwarePps=r.hardwarePps[i];
    axis.currentActVel=r.actualRpm[i]/60.0*r.frozen.config[i].pulsesPerOutputRevolution;
    if(!r.handback){
        // Spindle rotation has no positional target. Keep the raw, unwrapped
        // reference coherent; the ordinary position controller is bypassed.
        axis.currentCmdPos=axis.currentActPos;axis.logicalCmdPos.Store(axis.currentActPos);
        axis.planningPos=axis.finalTargetPos=axis.currentActPos;
    }
    axis.currentCmdVel=axis.logicalCmdVel=r.fault?0.0:r.commandRpm[i]/60.0*r.frozen.config[i].pulsesPerOutputRevolution;
    axis.targetVelocity=axis.currentCmdVel;axis.targetEndVel=0.0;
    axis.pid.integralAcc=axis.pid.prevError=0.0;
    if(!r.fault){axis.state=MotionState::MotionState_IDLE;axis.inPosition=r.feedback.allStopped;}
    return true;
}

void MotionCore::BeginRotarySpindleSend(ServoOutputFrameReservation& reservation) noexcept
{
    auto& r=m_rotarySpindle;const auto claim=m_rotarySpindleClaim.load(std::memory_order_acquire);
    if(!claim)return;
    if(!r.loaded||r.frozen.authority.scope.runToken!=claim){
        reservation.rotarySpindleRun=claim;reservation.rotarySpindleTick=m_ncSettleRuntimeCycleTick;return;
    }
    reservation.rotarySpindleRun=claim;reservation.rotarySpindleTick=r.tick;
    for(unsigned i=0;i<3U;++i){
        reservation.rotarySpindleRequest[i]=r.havePacket[i]?r.packet[i].sequence:0ULL;
        reservation.rotarySpindleRpm[i]=r.commandRpm[i];reservation.rotarySpindleHardwarePps[i]=r.hardwarePps[i];
    }
}

bool MotionCore::ValidateRotarySpindleSend(const ServoOutputFrameReservation& reservation) noexcept
{
    const auto claim=m_rotarySpindleClaim.load(std::memory_order_acquire);auto& r=m_rotarySpindle;
    if(!claim)return reservation.rotarySpindleRun==0ULL;
    // The caller scrubs/re-copies this pending source pass. Do not fault a
    // normal NC publication race or manufacture an RT acceptance receipt.
    if(!r.loaded||r.frozen.authority.scope.runToken!=claim)return false;
    bool valid=r.loaded&&reservation.rotarySpindleRun==claim&&r.frozen.authority.scope.runToken==claim&&
        reservation.rotarySpindleTick==r.tick&&r.tick==m_ncSettleRuntimeCycleTick&&reservation.handoff.sourceTick==r.tick&&
        IsRotarySpindleConfigurationCurrent(r.frozen);
    if(valid&&!r.fault)for(const auto& axis:*m_pContexts)if(axis.isExist&&
        (axis.homeRuntime.active||axis.state!=MotionState::MotionState_IDLE))valid=false;
    for(unsigned i=0;valid&&i<3U;++i)if(r.frozen.config[i].enabled){
        valid=reservation.rotarySpindleRequest[i]==(r.havePacket[i]?r.packet[i].sequence:0ULL)&&
            reservation.rotarySpindleHardwarePps[i]==r.hardwarePps[i]&&reservation.rotarySpindleRpm[i]==r.commandRpm[i];
        if(reservation.rotarySpindleHardwarePps[i]!=0)valid=valid&&!r.fault&&r.sourceValid[i]&&IsRotarySpindleAuthorityCurrent(r.frozen);
    }
    if(!valid)ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Output,true);
    return valid;
}

bool MotionCore::SealRotarySpindleSerializedImage(ServoOutputFrameReservation& reservation,
    const std::uint8_t* ioMapBase,std::size_t ioMapSize,const std::uint8_t* payload,std::size_t payloadSize) noexcept
{
    reservation.rotarySpindleSerializedExact=false;
    const auto claim=m_rotarySpindleClaim.load(std::memory_order_acquire);auto& r=m_rotarySpindle;
    if(claim&&(!r.loaded||r.frozen.authority.scope.runToken!=claim)){
        if(!ioMapBase||!payload||payloadSize!=ioMapSize||!m_pDrives||m_pDrives->size()>8U)return false;
        const auto base=reinterpret_cast<std::uintptr_t>(ioMapBase);
        for(const auto& drive:*m_pDrives){
            const auto address=reinterpret_cast<std::uintptr_t>(drive.pOutput);
            if(!drive.pOutput||address<base||address-base>ioMapSize)return false;
            const auto offset=static_cast<std::size_t>(address-base);
            if(offset>ioMapSize||sizeof(ServoOutput)>ioMapSize-offset)return false;
            std::int32_t wire=0;std::memcpy(&wire,payload+offset+offsetof(ServoOutput,TargetVelocity),sizeof(wire));
            if(wire!=0)return false;
        }
        return true; // Safe zero transport only; no spindle receipt is sealed.
    }
    if(!reservation.rotarySpindleRun)return !IsRotarySpindleSessionClaimed();
    if(!ValidateRotarySpindleSend(reservation)||!ioMapBase||!payload||payloadSize!=ioMapSize)return false;
    const auto base=reinterpret_cast<std::uintptr_t>(ioMapBase);
    for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled){
        const auto slot=r.frozen.config[i].axisSlot;const auto* output=(*m_pDrives)[slot].pOutput;
        const auto address=reinterpret_cast<std::uintptr_t>(output);
        if(!output||address<base||address-base>ioMapSize)return false;
        const auto offset=static_cast<std::size_t>(address-base);
        if(offset>ioMapSize||sizeof(ServoOutput)>ioMapSize-offset)return false;
        std::int32_t wire=0;std::memcpy(&wire,payload+offset+offsetof(ServoOutput,TargetVelocity),sizeof(wire));
        const double cap=r.frozen.config[i].maximumRpm/60.0*r.frozen.config[i].pulsesPerOutputRevolution;
        if(wire!=reservation.handoff.copiedVelocity[slot]||
            std::fabs(static_cast<double>(wire))>cap||std::fabs(static_cast<double>(wire))>r.frozen.binding[i].maxVelocity||
            (wire!=0&&(wire!=reservation.rotarySpindleHardwarePps[i]||!reservation.acquired||reservation.handoff.scrubbed||
                !reservation.handoff.finalChecked||reservation.handoff.proposedVelocity[slot]!=wire||
                reservation.handoff.epoch!=r.frozen.authority.scope.epoch||
                reservation.handoff.owner!=static_cast<std::uint32_t>(MotionOwner::AUTO)||
                reservation.handoff.ownerGeneration!=r.frozen.authority.scope.ownerGeneration||
                (reservation.handoff.sourceInputMask&(1U<<slot))==0U||
                !IsRotarySpindleAuthorityCurrent(r.frozen)||!r.sourceValid[i]))){
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Output,true);return false;
        }
    }
    reservation.rotarySpindleSerializedExact=true;return true;
}

void MotionCore::ObserveRotarySpindleSend(const ServoOutputFrameReservation& reservation) noexcept
{
    auto& r=m_rotarySpindle;
    if(!r.loaded||reservation.rotarySpindleRun!=r.frozen.authority.scope.runToken||
        reservation.rotarySpindleTick!=r.tick)return;
    const auto lease=GetMotionOwnerLease();const auto epoch=GetCurrentExecutionEpoch();
    for(unsigned i=0;i<3U;++i)if(r.frozen.config[i].enabled){
        auto& sent=r.sent[i];sent=RotarySpindleSent{};const auto slot=r.frozen.config[i].axisSlot;
        auto sample=reservation.handoff;
        // An alarm may skip the normal per-axis loop. Source evidence still
        // comes from this callback's independently captured real PDO input.
        if(r.sourceValid[i]&&sample.sourceTick==r.tick){sample.sourceInputMask|=1U<<slot;
            sample.sourceStatusWord[slot]=r.input[i].StatusWord;sample.sourceMode[slot]=r.input[i].ModesOfOperationDisplay;}
        const MotionOwnerLease original{MotionOwner::AUTO,static_cast<MotionOwnerGeneration>(r.frozen.authority.scope.ownerGeneration)};
        const bool normal=MotionRotarySpindleProof::ExactSend(sample,
            static_cast<MotionExecutionEpoch>(r.frozen.authority.scope.epoch),original,slot,r.tick,reservation.rotarySpindleHardwarePps[i]);
        const bool safety=r.fault&&MotionRotarySpindleProof::ExactSafetyZeroSend(sample,epoch,lease,slot,r.tick);
        sent.exact=reservation.rotarySpindleSerializedExact&&r.sourceValid[i]&&(normal||safety);
        sent.safetyZero=sent.exact&&safety;sent.tick=r.tick;sent.request=reservation.rotarySpindleRequest[i];
        sent.commandRpm=safety?0.0:reservation.rotarySpindleRpm[i];sent.epoch=epoch;sent.owner=lease;
        sent.safetyTicket=UnpackMotionOwnerSafetyRequestTicket(m_motionOwnerState.load(std::memory_order_acquire));
        if(sent.exact)r.outputMissTicks[i]=0U;
        else if(r.outputMissTicks[i]<32U)++r.outputMissTicks[i];
        // Existing NC commit/output reservations may legitimately scrub one
        // image to zero. It earns no readiness/stop sample, but is not a new
        // mechanical fault. A failed NIC call or sustained unavailable output
        // still enters the existing global safety path.
        const bool temporaryZero=reservation.rotarySpindleSerializedExact&&sample.sequence&&sample.frameGeneration&&
            sample.finalChecked&&sample.attempted&&sample.apiAccepted&&sample.sourceTick==r.tick&&
            sample.epoch==r.frozen.authority.scope.epoch&&sample.owner==static_cast<std::uint32_t>(MotionOwner::AUTO)&&
            sample.ownerGeneration==r.frozen.authority.scope.ownerGeneration&&sample.copiedVelocity[slot]==0&&
            r.sourceValid[i]&&IsRotarySpindleAuthorityCurrent(r.frozen);
        if(!sent.exact&&r.initialized&&!r.fault&&(!temporaryZero||r.outputMissTicks[i]>=32U))
            ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Output,true);
    }
}

void MotionCore::PublishRotarySpindleRuntime() noexcept
{
    auto& r=m_rotarySpindle;r.feedback.authority=r.frozen.authority;r.feedback.tick=r.tick;r.feedback.axisMask=r.frozen.axisMask;
    r.feedback.initialized=r.initialized;r.feedback.fault=r.fault;r.feedback.faultCode=r.faultCode;
    r.feedback.commandRpm=r.commandRpm;r.feedback.actualRpm=r.actualRpm;
    if(!m_rotarySpindleFeedback.Publish(r.feedback))ContainRotarySpindleRuntime(MotionRotarySpindle::Fault::Feedback,true);
}
