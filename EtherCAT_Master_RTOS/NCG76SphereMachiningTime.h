#pragma once
// Included after Cycle. Two clocks belong to this unfinished native parent;
// neither a countdown nor an additional pass can commit its endpoint.
namespace NCG76SphereCycle
{
inline void Cycle::RefreshMachiningTimeFeedback() noexcept
{
    using namespace NCMachiningTime;
    if(!request_.totalLimitUs&&!request_.additionalUs)return;
    feedback_.totalLimitUs=request_.totalLimitUs;
    feedback_.totalElapsedUs=machiningTime_.totalElapsedUs;
    feedback_.additionalUs=request_.additionalUs;
    feedback_.additionalElapsedUs=machiningTime_.additionalElapsedUs;
    const bool fault=feedback_.phase==Phase::Fault;
    const bool complete=feedback_.phase==Phase::Complete;
    const auto state=[](TimerState value) noexcept {return static_cast<std::uint8_t>(value);};
    const auto pause=[](PauseReason value) noexcept {return static_cast<std::uint8_t>(value);};
    if(request_.totalLimitUs){
        const auto reason=machiningTime_.totalExpired?PauseReason::None:
            fault?PauseReason::Fault:complete?PauseReason::None:machiningTime_.totalPause;
        feedback_.totalPauseReason=pause(reason);
        feedback_.totalTimerState=state(machiningTime_.totalExpired?TimerState::Expired:
            complete?TimerState::Completed:fault||reason!=PauseReason::None?TimerState::Paused:TimerState::Running);
    }
    if(request_.additionalUs){
        const auto reason=machiningTime_.additionalSatisfied?PauseReason::None:
            fault||machiningTime_.totalExpired?PauseReason::Fault:
            !machiningTime_.additionalStarted?PauseReason::Waiting:machiningTime_.additionalPause;
        feedback_.additionalPauseReason=pause(reason);
        feedback_.additionalTimerState=state(machiningTime_.additionalSatisfied?TimerState::Completed:
            fault||machiningTime_.totalExpired?TimerState::Paused:
            !machiningTime_.additionalStarted?TimerState::Waiting:
            reason!=PauseReason::None?TimerState::Paused:TimerState::Running);
    }
    const auto phase=machiningTime_.totalExpired?TimePhase::TimedOut:
        fault?TimePhase::Fault:complete?TimePhase::Completed:
        feedback_.finishActive?TimePhase::Verification:
        machiningTime_.additionalStarted?(machiningTime_.additionalSatisfied?TimePhase::Finishing:TimePhase::Additional):
        TimePhase::Machining;
    feedback_.timePhase=static_cast<std::uint8_t>(phase);
}

inline void Cycle::BeginTimeLimitStop(const Input& input) noexcept
{
    // The existing leg retains its scalar/circle representation, including a
    // flush or historical excursion. No point is replaced by a chord/end.
    if(leg_.speed!=0.0)(void)StartBrake(Phase::TimeLimitBrake);
    else StartSettle(Phase::TimeLimitSettle,SettleAction::TimeLimitFault,input);
}

inline bool Cycle::UpdateMachiningTime(const Input& input,std::uint64_t deltaUs,bool holdEdge) noexcept
{
    using namespace NCMachiningTime;
    if(!request_.totalLimitUs&&!request_.additionalUs)return true;
    if(machiningTime_.totalExpired){RefreshMachiningTimeFeedback();return true;}
    const bool held=holdEdge||input.hold||feedback_.holdActive||recovery_.active;
    const bool returning=feedback_.phase==Phase::ResumeToCenter||feedback_.phase==Phase::ResumeToAnchor||
        feedback_.holdRecoveryPhase==6U||feedback_.holdRecoveryPhase==7U||feedback_.holdRecoveryPhase==8U;
    machiningTime_.totalPause=held?(returning?PauseReason::Returning:PauseReason::Hold):PauseReason::None;
    if(request_.totalLimitUs&&!held){
        const auto remaining=request_.totalLimitUs-machiningTime_.totalElapsedUs;
        machiningTime_.totalElapsedUs+=deltaUs<remaining?deltaUs:remaining;
        if(machiningTime_.totalElapsedUs==request_.totalLimitUs){
            machiningTime_.totalExpired=true;machiningTime_.additionalPause=PauseReason::Fault;
            BeginTimeLimitStop(input);RefreshMachiningTimeFeedback();return !output_.faulted;
        }
    }
    if(request_.additionalUs&&machiningTime_.additionalStarted&&!machiningTime_.additionalSatisfied){
        PauseReason reason=PauseReason::None;
        const bool endpoint=leg_.position==leg_.length&&leg_.length>0.0&&leg_.speed==0.0;
        const bool endpointProof=endpoint&&input.physicalSettled&&input.zeroFinalSend&&
            input.zeroSendTick<input.tick&&input.tick-input.zeroSendTick==1ULL&&AtCommand(input.actualMCS);
        const bool work=Detail::IsFlushWorkPhase(feedback_.phase,feedback_.layerIndex)||
            (feedback_.phase==Phase::Settle&&Detail::IsFlushWorkPhase(feedback_.nominalPhase,feedback_.layerIndex));
        const bool moving=Detail::IsFlushWorkPhase(feedback_.phase,feedback_.layerIndex)&&leg_.speed>=0.0&&
            leg_.position<leg_.length&&AtCommand(input.actualMCS);
        if(held)reason=returning?PauseReason::Returning:PauseReason::Hold;
        else if(feedback_.flushActive)reason=PauseReason::Flush;
        else if(feedback_.gapHistoryActive||!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin)||
            input.signedGapSpeedMmMin<=0.0||leg_.speed<0.0)reason=PauseReason::Gap;
        else if(!work)reason=PauseReason::Returning;
        else if(!moving&&!endpointProof)reason=PauseReason::Qualification;
        machiningTime_.additionalPause=reason;
        if(reason==PauseReason::None){
            const auto remaining=request_.additionalUs-machiningTime_.additionalElapsedUs;
            machiningTime_.additionalElapsedUs+=deltaUs<remaining?deltaUs:remaining;
            machiningTime_.additionalSatisfied=machiningTime_.additionalElapsedUs==request_.additionalUs;
        }
    }
    RefreshMachiningTimeFeedback();return true;
}

inline bool Cycle::HandleAdditionalEndpoint(const Input& input) noexcept
{
    if(!request_.additionalUs)return false;
    if(!gap_.enabled||!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin))
    {Fail(Fault::SourceLost);return true;}
    if(!AtCommand(input.actualMCS)||leg_.speed!=0.0||leg_.position!=leg_.length||
       output_.commandMCS!=request_.geometry.endMCS)
    {Fail(Fault::Numeric);return true;}
    // A new physical endpoint proof precedes this hook. A negative GAP
    // returns to the unchanged scalar leg, even after the H clock is full.
    if(input.signedGapSpeedMmMin<=0.0){
        machiningTime_.additionalPause=NCMachiningTime::PauseReason::Gap;
        feedback_.phase=feedback_.nominalPhase;RefreshOutput();return true;
    }
    if(machiningTime_.additionalSatisfied)return false;
    machiningTime_.additionalStarted=true;
    machiningTime_.additionalPause=NCMachiningTime::PauseReason::Qualification;
    feedback_.phase=feedback_.nominalPhase;RefreshOutput();return true;
}

inline void Cycle::CompleteSphereWithAdditional(const Input& input) noexcept
{
    if(!request_.additionalUs||request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere||
       feedback_.completedLayers!=request_.geometry.circleLayerCount||leg_.speed!=0.0||
       leg_.position!=leg_.length||output_.commandMCS!=request_.geometry.endMCS||!AtCommand(input.actualMCS))
    {Fail(Fault::Numeric);return;}
    if(!gap_.enabled||!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin))
    {Fail(Fault::SourceLost);return;}
    if(input.signedGapSpeedMmMin<=0.0){
        // The zero-radius terminus cannot be credited as an extra circle.
        // Hold its qualified reference until a new positive physical dwell.
        machiningTime_.additionalPause=NCMachiningTime::PauseReason::Gap;
        StartSettle(Phase::Settle,SettleAction::TimeCheck,input);return;
    }
    if(machiningTime_.additionalStarted&&machiningTime_.additionalSatisfied){
        output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
        leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    // One H clock per parent. An additional pass reuses the admitted layer
    // geometry, centerline return and circles, without replaying initial depth
    // or any NC/G/M/E side effect. Even a short H completes this whole pass.
    machiningTime_.additionalStarted=true;machiningTime_.additionalSweeping=true;
    machiningTime_.additionalPause=NCMachiningTime::PauseReason::Returning;
    feedback_.layerIndex=0U;previousLegValid_=false;feedback_.gapHistoryActive=false;
    (void)StartNominal(Phase::InitialRetreat);
}
// The native grow/depth command is never replayed for H. Each extra pass
// traverses only the admitted final-center entry, full closed contour and return.
inline void Cycle::CompleteContourWithAdditional(const Input& input) noexcept
{
    if(!request_.additionalUs||request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||
       !NCG76OrbitContour::FinishEnabled(request_.geometry)||leg_.speed!=0.0||
       leg_.position!=leg_.length||output_.commandMCS!=request_.geometry.endMCS||!AtCommand(input.actualMCS)||
       (feedback_.nominalPhase==Phase::ContourNominal?feedback_.layerIndex!=24U:
            feedback_.nominalPhase!=Phase::ReturnCenter||feedback_.layerIndex!=0U))
    {Fail(Fault::Numeric);return;}
    if(!gap_.enabled||!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin))
    {Fail(Fault::SourceLost);return;}
    if(input.signedGapSpeedMmMin<=0.0){
        machiningTime_.additionalPause=NCMachiningTime::PauseReason::Gap;
        StartSettle(Phase::Settle,SettleAction::TimeCheck,input);return;
    }
    if(machiningTime_.additionalStarted&&machiningTime_.additionalSatisfied){
        feedback_.phase=Phase::Complete;leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    machiningTime_.additionalStarted=true;machiningTime_.additionalSweeping=true;
    machiningTime_.additionalPause=NCMachiningTime::PauseReason::Qualification;
    // Each completed H center-return is the same physical line as native
    // leg 24. Give it that earlier canonical identity only after bit-exact
    // endpoint, length and tangent proof. A history HOLD then has no wrapped
    // station and cannot turn a completed contour into a return chord.
    Point8 begin{},end{},center{};double length=0.0,prefix=0.0;Point3 direction{};
    bool canonical=NCG76OrbitContour::NominalLegAt(request_.geometry,24U,begin,end,center,length,prefix)&&
        std::memcmp(begin.data(),leg_.start.data(),sizeof(Point8))==0&&
        std::memcmp(end.data(),leg_.end.data(),sizeof(Point8))==0&&
        std::memcmp(&length,&leg_.length,sizeof(double))==0;
    if(canonical){
        for(unsigned a=0U;a<3U;++a)direction[a]=(end[a]-begin[a])/length;
        canonical=std::memcmp(direction.data(),leg_.direction.data(),sizeof(Point3))==0;
    }
    previousLegValid_=canonical;
    if(canonical){previousLeg_=leg_;previousPhase_=Phase::ContourNominal;previousLayer_=24U;}
    feedback_.layerIndex=0U;feedback_.gapHistoryActive=false;
    (void)StartNominal(Phase::CircleEntry);
}
}
