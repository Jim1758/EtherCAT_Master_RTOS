#pragma once
// Included after Cycle. Two clocks belong to this unfinished native parent;
// neither a countdown nor an additional pass can commit its endpoint.
namespace NCG76SphereCycle
{
inline void Cycle::RefreshOrbitCompletionFeedback() noexcept
{
    auto& f=feedback_;auto& o=orbitInspection_;
    f.orbitRequiredPasses=request_.orbitInspectionRounds;
    f.orbitTargetPercentScaled=LiveOrbitThresholdScaled();
    f.orbitRetryLimit=LiveOrbitRetryLimit();
    if(!request_.orbitInspectionRounds){
        f.orbitQualifiedPasses=0U;f.orbitAttemptCount=0ULL;f.orbitPercentScaled=0U;
        f.orbitCompletionState=0U;f.orbitPercentAvailable=f.orbitCompletionActive=false;
        f.orbitEvaluation=NCG76OrbitCompletion::AttemptEvidence{};f.orbitRetryLimit=0U;return;
    }
    if(o.state==NCG76OrbitCompletion::State::Disabled)o.state=NCG76OrbitCompletion::State::Armed;
    f.orbitQualifiedPasses=o.qualifiedPasses;f.orbitAttemptCount=o.attempts;
    f.orbitPercentScaled=o.percent;f.orbitCompletionState=static_cast<std::uint8_t>(o.state);
    f.orbitPercentAvailable=o.percentAvailable;f.orbitCompletionActive=o.active;
    f.orbitEvaluation=o.evaluation;
}

inline void Cycle::ObserveOrbitCompletion(const Input& input,bool holdEdge) noexcept
{
    auto& o=orbitInspection_;
    if(!o.active)return;
    const auto phase=feedback_.phase;
    const bool history=feedback_.gapHistoryActive&&
        (phase==Phase::GapHistory||(phase==Phase::GapHistorySettle&&settleAction_==SettleAction::RestoreGapHistory));
    const Phase nominal=history?previousPhase_:feedback_.nominalPhase;
    const unsigned layer=history?previousLayer_:feedback_.layerIndex;
    if(history&&(nominal==Phase::Orbit||nominal==Phase::ContourFinish)&&
       (!o.circleOpen||(nominal==Phase::Orbit&&layer!=feedback_.layerIndex))){
        // A closed circle's bitmap may already have been reused. Preserve the
        // admitted history motion, but never retain unscored old good credit.
        o.attemptInvalid=true;o.percent=0U;o.percentAvailable=true;o.lastStationValid=false;
        RefreshOrbitCompletionFeedback();return;
    }
    if(!o.circleOpen)return;
    const bool work=nominal==Phase::Orbit||nominal==Phase::ContourFinish;
    const bool retained=phase==nominal||phase==Phase::Settle||history;
    if(!work||!retained||holdEdge||input.hold||feedback_.holdActive||feedback_.flushActive||recovery_.active){
        o.lastStationValid=false;return;
    }
    double prefix=0.0;
    if(nominal==Phase::ContourFinish){
        Point8 a{},b{},center{};double length=0.0;
        if(!NCG76OrbitContour::FinishLegAt(request_.geometry,layer,a,b,center,length,prefix))
        {Fail(Fault::Numeric);return;}
    }
    const double station=prefix+leg_.position;
    const bool qualified=!history&&input.gapValid&&std::isfinite(input.signedGapSpeedMmMin)&&
        input.signedGapSpeedMmMin>0.0&&leg_.speed>=0.0&&AtCommand(input.actualMCS);
    if(o.lastStationValid){
        if(!o.coverage.Observe(o.lastStation,station,qualified)){Fail(Fault::Numeric);return;}
    }else if(!qualified&&!o.coverage.Observe(station,station,false)){Fail(Fault::Numeric);return;}
    o.lastStation=station;o.lastStationValid=true;
    o.percent=o.attemptInvalid?0U:NCG76OrbitCompletion::Percent(o.qualifiedLength+o.coverage.QualifiedLength(),o.expectedLength);
    o.percentAvailable=true;o.state=NCG76OrbitCompletion::State::Inspecting;
    RefreshOrbitCompletionFeedback();
}

inline void Cycle::FinishOrbitInspectionCircle() noexcept
{
    auto& o=orbitInspection_;
    if(!o.active||!o.circleOpen||!std::isfinite(o.coverage.QualifiedLength())){Fail(Fault::Numeric);return;}
    o.qualifiedLength+=o.coverage.QualifiedLength();++o.closedCircles;
    o.circleOpen=false;o.lastStationValid=false;
    o.percent=o.attemptInvalid?0U:NCG76OrbitCompletion::Percent(o.qualifiedLength,o.expectedLength);
    o.percentAvailable=true;RefreshOrbitCompletionFeedback();
}

inline bool Cycle::BeginOrbitInspectionSweep(const Input&) noexcept
{
    auto& o=orbitInspection_;
    const bool contour=request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour;
    if(!request_.orbitInspectionRounds||!gap_.enabled){Fail(Fault::InvalidRequest);return false;}
    if(contour){
        if(NCG76OrbitContour::IsStraightSideContour(request_.geometry))o.expectedLength=NCG76OrbitContour::FinishLength(request_.geometry);
        else{
            NCG76SphereRuntimeGeometry::Layer layer{};
            if(!NCG76SphereRuntimeGeometry::LayerAt(request_.geometry,0U,layer)){Fail(Fault::Numeric);return false;}
            o.expectedLength=layer.arcLengthMm;
        }
        o.expectedCircles=1U;
    }
    if(!std::isfinite(o.expectedLength)||o.expectedLength<=0.0||
       (!contour&&o.expectedCircles!=request_.geometry.circleLayerCount))
    {Fail(Fault::Numeric);return false;}
    o.active=true;o.attemptInvalid=false;o.percent=0U;o.percentAvailable=false;o.qualifiedLength=0.0;o.closedCircles=0U;o.lastStationValid=false;o.circleOpen=false;
    if(o.state!=NCG76OrbitCompletion::State::Retry)o.state=NCG76OrbitCompletion::State::Inspecting;
    if(contour&&NCG76OrbitContour::IsStraightSideContour(request_.geometry)){
        if(!o.coverage.Begin(o.expectedLength)){Fail(Fault::Numeric);return false;}o.circleOpen=true;
    }
    // Same canonical center-return identity as the H sweep. It is bound only
    // after exact endpoint/length/tangent equality, never by an approximate seam.
    if(contour){
        Point8 begin{},end{},center{};double length=0.0,prefix=0.0;Point3 direction{};
        bool canonical=NCG76OrbitContour::NominalLegAt(request_.geometry,Detail::NominalWorkEndIndex(request_.geometry),begin,end,center,length,prefix)&&
            std::memcmp(begin.data(),leg_.start.data(),sizeof(Point8))==0&&
            std::memcmp(end.data(),leg_.end.data(),sizeof(Point8))==0&&
            std::memcmp(&length,&leg_.length,sizeof(double))==0;
        if(canonical){
            for(unsigned a=0U;a<3U;++a)direction[a]=(end[a]-begin[a])/length;
            canonical=std::memcmp(direction.data(),leg_.direction.data(),sizeof(Point3))==0;
        }
        previousLegValid_=canonical;
        if(canonical){previousLeg_=leg_;previousPhase_=Phase::ContourNominal;previousLayer_=Detail::NominalWorkEndIndex(request_.geometry);}
    }else previousLegValid_=false;
    feedback_.layerIndex=0U;feedback_.gapHistoryActive=false;
    return StartNominal(contour?Phase::CircleEntry:Phase::InitialRetreat);
}

inline void Cycle::CompleteOrbitInspectionSweep(const Input& input) noexcept
{
    auto& o=orbitInspection_;
    if(!o.active||o.circleOpen||o.closedCircles!=o.expectedCircles||
       !std::isfinite(o.qualifiedLength)||o.qualifiedLength<0.0||
       o.attempts==(std::numeric_limits<std::uint64_t>::max)())
    {Fail(Fault::Numeric);return;}
    ++o.attempts;o.percent=o.attemptInvalid?0U:NCG76OrbitCompletion::Percent(o.qualifiedLength,o.expectedLength);o.percentAvailable=true;
    const bool accepted=o.percent>=LiveOrbitThresholdScaled();
    const auto qualifiedBefore=o.qualifiedPasses;
    o.qualifiedPasses=accepted?o.qualifiedPasses+1U:0U;
    // Preserve the completed-sweep decision when Begin resets live coverage,
    // and when D later clears its current, unfinished percentage.
    if(!accepted)++o.failedSweeps;
    o.evaluation.failedAttempts=o.failedSweeps;
    o.evaluation.lastPercentScaled=o.percent;o.evaluation.lastQualifiedBefore=qualifiedBefore;
    o.evaluation.peakQualified=(std::max)(o.evaluation.peakQualified,o.qualifiedPasses);
    o.evaluation.lastAvailable=true;o.evaluation.lastAccepted=accepted;
    // The complete old sweep is scored under its original criterion. A
    // pending E16/E29 edit then begins a new local evidence epoch instead of
    // retroactively reclassifying that score or inheriting its retry debt.
    if(pendingLiveOrbitCriteria_&&!machiningTime_.totalExpired&&!feedback_.timeLimitReached){
        o.active=false;
        if(!ApplyPendingLiveOrbitCriteria()){Fail(Fault::Numeric);return;}
        (void)BeginOrbitInspectionSweep(input);return;
    }
    if(o.qualifiedPasses==request_.orbitInspectionRounds){
        o.active=false;o.state=NCG76OrbitCompletion::State::Passed;
        output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
        leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    // The same complete-sweep physical center proof used by Passed already
    // precedes this hook. A rejected process may finish its NC motion parent
    // here without claiming L passed, a D goal, or a runtime fault.
    if(!accepted&&LiveOrbitRetryLimit()&&o.failedSweeps>LiveOrbitRetryLimit()){
        o.active=false;o.state=NCG76OrbitCompletion::State::RetryExhausted;
        output_.commandMCS=request_.geometry.endMCS;feedback_.phase=Phase::Complete;
        leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    o.state=accepted?NCG76OrbitCompletion::State::Inspecting:NCG76OrbitCompletion::State::Retry;
    (void)BeginOrbitInspectionSweep(input);
}

inline void Cycle::RefreshMachiningTimeFeedback() noexcept
{
    using namespace NCMachiningTime;
    if(!request_.totalLimitUs&&!request_.additionalUs){
        if(request_.orbitInspectionRounds)feedback_.timePhase=static_cast<std::uint8_t>(
            feedback_.phase==Phase::Fault?TimePhase::Fault:feedback_.phase==Phase::Complete?TimePhase::Completed:
            orbitInspection_.active?TimePhase::Verification:TimePhase::Machining);
        return;
    }
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
        feedback_.totalTimerState=state(machiningTime_.totalExpired?TimerState::Completed:
            complete?TimerState::Completed:fault||reason!=PauseReason::None?TimerState::Paused:TimerState::Running);
    }
    if(request_.additionalUs){
        const auto reason=machiningTime_.additionalSatisfied?PauseReason::None:
            fault||machiningTime_.totalExpired?PauseReason::Fault:
            !machiningTime_.additionalStarted?PauseReason::Waiting:machiningTime_.additionalPause;
        feedback_.additionalPauseReason=pause(reason);
        feedback_.additionalTimerState=state(machiningTime_.additionalSatisfied?TimerState::Completed:
            machiningTime_.totalExpired?TimerState::Paused:
            fault?TimerState::Paused:
            !machiningTime_.additionalStarted?TimerState::Waiting:
            reason!=PauseReason::None?TimerState::Paused:TimerState::Running);
    }
    const auto phase=fault?TimePhase::Fault:complete?TimePhase::Completed:
        machiningTime_.totalExpired?TimePhase::TimedFinishing:
        (feedback_.finishActive||orbitInspection_.active)?TimePhase::Verification:
        machiningTime_.additionalStarted?(machiningTime_.additionalSatisfied?TimePhase::Finishing:TimePhase::Additional):
        TimePhase::Machining;
    feedback_.timePhase=static_cast<std::uint8_t>(phase);
}

// EDM359: D is a normal completion condition. Keep the active analytic leg
// through braking; a returned flush/history checkpoint precedes G76's center.
inline void Cycle::BeginTimeLimitStop(const Input& input) noexcept
{
    if(feedback_.flushActive)return; // its already admitted return must drain
    if(leg_.speed!=0.0)(void)StartBrake(Phase::TimeLimitBrake);
    else StartSettle(Phase::TimeLimitSettle,SettleAction::TimedStop,input);
}
inline void Cycle::ContinueTimedStop(const Input& input) noexcept
{
    if(!machiningTime_.totalExpired||!request_.totalLimitUs||
       machiningTime_.totalElapsedUs!=request_.totalLimitUs||leg_.speed!=0.0||
       !AtCommand(input.actualMCS)||feedback_.flushActive||feedback_.holdActive)
    {Fail(Fault::Numeric);return;}
    if(machiningTime_.historyReturn){
        // A GAP excursion may be on a prior completed circle/primitive. Walk
        // its exact remaining analytic path to the current parent's start;
        // never use a chord or complete this parent on its predecessor.
        if(!feedback_.gapHistoryActive||leg_.end!=historyNominalLeg_.start||
           historyNominalLeg_.position!=0.0||leg_.position<0.0||leg_.position>leg_.length)
        {Fail(Fault::Numeric);return;}
        if(leg_.position==leg_.length){
            StartSettle(Phase::TimeLimitSettle,SettleAction::TimedHistoryRestored,input);return;
        }
        if(!StartProfile(leg_)){Fail(Fault::Numeric);return;}
        feedback_.phase=Phase::TimedHistoryReturn;RefreshOutput();return;
    }
    if(feedback_.gapHistoryActive){Fail(Fault::Numeric);return;}
    Point8 endpoint=output_.commandMCS;
    const bool contour=request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour;
    const bool g76=contour||request_.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere;
    if(contour&&feedback_.nominalPhase==Phase::ContourNominal){
        if(!(leg_.length>0.0)||!NCG76OrbitContour::NominalCenterAt(request_.geometry,
            feedback_.layerIndex,leg_.position/leg_.length,endpoint))
        {Fail(Fault::Numeric);return;}
    }else if(g76&&(feedback_.nominalPhase==Phase::CircleEntry||feedback_.nominalPhase==Phase::Orbit||
        feedback_.nominalPhase==Phase::ContourFinish||feedback_.nominalPhase==Phase::ReturnCenter))endpoint=leg_.center;
    if(!Detail::Finite(endpoint)||!InsideTravel(endpoint)) {Fail(Fault::TravelLimit);return;}
    for(unsigned axis=0U;axis<8U;++axis){
        if(endpoint[axis]<request_.geometry.boundsMinMCS[axis]||endpoint[axis]>request_.geometry.boundsMaxMCS[axis]||
           (axis>=3U&&request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&
            std::memcmp(&endpoint[axis],&request_.geometry.startMCS[axis],sizeof(double))!=0))
        {Fail(Fault::TravelLimit);return;}
    }
    machiningTime_.endpoint=endpoint;machiningTime_.endpointReady=true;
    if(endpoint==output_.commandMCS){
        // Numeric identity can still differ in an XYZ signed-zero bit. This
        // canonicalization has zero physical displacement and preserves the
        // already-validated fixed rotary bits for the published endpoint proof.
        output_.commandMCS=endpoint;CompleteTimed(input);return;
    }
    (void)StartLine(endpoint,Phase::TimedReturnCenter,request_.returnFeedMmMin);
}
inline void Cycle::CompleteTimed(const Input& input) noexcept
{
    if(!machiningTime_.totalExpired||!machiningTime_.endpointReady||!request_.totalLimitUs||
       machiningTime_.totalElapsedUs!=request_.totalLimitUs||leg_.speed!=0.0||
       output_.commandMCS!=machiningTime_.endpoint||!AtCommand(input.actualMCS)||
       !input.physicalSettled||!input.zeroFinalSend||input.zeroSendTick>=input.tick||
       input.tick-input.zeroSendTick!=1ULL||feedback_.holdActive||feedback_.flushActive||feedback_.gapHistoryActive)
    {Fail(Fault::Numeric);return;}
    feedback_.timedCompletion=true;feedback_.phase=Phase::Complete;
    leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();
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
            feedback_.timeLimitReached=true;
            // Preserve actual completed work; time completion never invents
            // additional H time, a passed L attempt or forced-finish credit.
            if(request_.orbitInspectionRounds){
                orbitInspection_.active=orbitInspection_.percentAvailable=false;
                orbitInspection_.percent=0U;orbitInspection_.circleOpen=orbitInspection_.lastStationValid=false;
                orbitInspection_.state=NCG76OrbitCompletion::State::StoppedByTime;
            }
            if(feedback_.finishRequiredCount){feedback_.finishActive=false;
                feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::StoppedByTime);}
            const bool history=feedback_.gapHistoryActive&&
                (feedback_.phase==Phase::GapHistory||
                 (feedback_.phase==Phase::GapHistorySettle&&settleAction_==SettleAction::RestoreGapHistory));
            machiningTime_.historyReturn=history;
            if(feedback_.gapHistoryActive&&!history)feedback_.gapHistoryActive=false;
            BeginTimeLimitStop(input);RefreshMachiningTimeFeedback();return !output_.faulted;
        }
    }
    if(request_.additionalUs&&machiningTime_.additionalStarted&&!machiningTime_.additionalSatisfied){
        PauseReason reason=PauseReason::None;
        const bool endpoint=leg_.position==leg_.length&&leg_.length>0.0&&leg_.speed==0.0;
        const bool endpointProof=endpoint&&input.physicalSettled&&input.zeroFinalSend&&
            input.zeroSendTick<input.tick&&input.tick-input.zeroSendTick==1ULL&&AtCommand(input.actualMCS);
        const bool work=Detail::IsFlushWorkPhase(feedback_.phase,feedback_.layerIndex,NCG76OrbitContour::FinishLegCount(request_.geometry),Detail::NominalWorkEndIndex(request_.geometry))||
            (feedback_.phase==Phase::Settle&&Detail::IsFlushWorkPhase(feedback_.nominalPhase,feedback_.layerIndex,NCG76OrbitContour::FinishLegCount(request_.geometry),Detail::NominalWorkEndIndex(request_.geometry)));
        const bool moving=Detail::IsFlushWorkPhase(feedback_.phase,feedback_.layerIndex,NCG76OrbitContour::FinishLegCount(request_.geometry),Detail::NominalWorkEndIndex(request_.geometry))&&leg_.speed>=0.0&&
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
    if((!request_.additionalUs&&!request_.orbitInspectionRounds)||request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere||
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
    if(orbitInspection_.active){CompleteOrbitInspectionSweep(input);return;}
    if(!request_.additionalUs||(machiningTime_.additionalStarted&&machiningTime_.additionalSatisfied)){
        if(request_.orbitInspectionRounds){(void)BeginOrbitInspectionSweep(input);return;}
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
    if((!request_.additionalUs&&!request_.orbitInspectionRounds)||request_.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||
       !NCG76OrbitContour::FinishEnabled(request_.geometry)||leg_.speed!=0.0||
       leg_.position!=leg_.length||output_.commandMCS!=request_.geometry.endMCS||!AtCommand(input.actualMCS)||
       (feedback_.nominalPhase==Phase::ContourNominal?feedback_.layerIndex!=Detail::NominalWorkEndIndex(request_.geometry):
            feedback_.nominalPhase!=Phase::ReturnCenter||feedback_.layerIndex!=0U))
    {Fail(Fault::Numeric);return;}
    if(!gap_.enabled||!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin))
    {Fail(Fault::SourceLost);return;}
    if(input.signedGapSpeedMmMin<=0.0){
        machiningTime_.additionalPause=NCMachiningTime::PauseReason::Gap;
        StartSettle(Phase::Settle,SettleAction::TimeCheck,input);return;
    }
    if(orbitInspection_.active){CompleteOrbitInspectionSweep(input);return;}
    if(!request_.additionalUs||(machiningTime_.additionalStarted&&machiningTime_.additionalSatisfied)){
        if(request_.orbitInspectionRounds){(void)BeginOrbitInspectionSweep(input);return;}
        feedback_.phase=Phase::Complete;leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    machiningTime_.additionalStarted=true;machiningTime_.additionalSweeping=true;
    machiningTime_.additionalPause=NCMachiningTime::PauseReason::Qualification;
    // Each completed H center-return is the same physical line as the
    // final native leg. Give it that earlier canonical identity only after bit-exact
    // endpoint, length and tangent proof. A history HOLD then has no wrapped
    // station and cannot turn a completed contour into a return chord.
    Point8 begin{},end{},center{};double length=0.0,prefix=0.0;Point3 direction{};
    bool canonical=NCG76OrbitContour::NominalLegAt(request_.geometry,Detail::NominalWorkEndIndex(request_.geometry),begin,end,center,length,prefix)&&
        std::memcmp(begin.data(),leg_.start.data(),sizeof(Point8))==0&&
        std::memcmp(end.data(),leg_.end.data(),sizeof(Point8))==0&&
        std::memcmp(&length,&leg_.length,sizeof(double))==0;
    if(canonical){
        for(unsigned a=0U;a<3U;++a)direction[a]=(end[a]-begin[a])/length;
        canonical=std::memcmp(direction.data(),leg_.direction.data(),sizeof(Point3))==0;
    }
    previousLegValid_=canonical;
    if(canonical){previousLeg_=leg_;previousPhase_=Phase::ContourNominal;previousLayer_=Detail::NominalWorkEndIndex(request_.geometry);}
    feedback_.layerIndex=0U;feedback_.gapHistoryActive=false;
    (void)StartNominal(Phase::CircleEntry);
}
}
