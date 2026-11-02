#pragma once
// Included after Cycle. The same unfinished native parent owns every check,
// forced wash and GAP rework; no old row, endpoint or E5 event is re-opened.
namespace NCG76SphereCycle
{
inline bool Cycle::ConfigureFinishVerification(std::uint32_t requiredCount) noexcept
{
    if(!initialized_||!output_.active)return false;
    if(requiredCount==0U)return feedback_.finishRequiredCount==0U;
    using K=NCG76SphereRuntimeGeometry::Kind;
    if(requiredCount>99U||feedback_.finishRequiredCount||feedback_.holdActive||
       feedback_.phase!=Phase::Settle||settleAction_!=SettleAction::BeginNominal||
       feedback_.tick!=settleTick_||lastTick_!=settleTick_||leg_.position!=0.0||leg_.speed!=0.0||
       (request_.geometry.kind!=K::Line&&request_.geometry.kind!=K::Arc)||
       !Detail::Positive(request_.geometry.travelMm)||!gap_.enabled||!flush_.enabled||
       !ValidateFlushConfig(request_,flush_))return false;
    feedback_.finishRequiredCount=requiredCount;
    feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Machining);
    return true;
}
inline void Cycle::BeginFinishFlush(const Input& input) noexcept
{
    if(!feedback_.finishActive||finish_.forcedFlush||finish_.returnedPending||feedback_.flushActive||
       feedback_.finishAttemptCount==(std::numeric_limits<std::uint64_t>::max)())
    {Fail(Fault::Numeric);return;}
    // E6 and the selected B route/pumps are shared, but E5's request/ACK,
    // successful count, interval and deep ordinal are never manufactured.
    finish_.forcedFlush=true;++feedback_.finishAttemptCount;
    feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::ForcedFlush);
    feedback_.flushOrigin=static_cast<std::uint8_t>(FlushOrigin::Finish);
    feedback_.flushSelectedHeightMm=flush_.heightMm;feedback_.flushDeepActive=false;
    feedback_.flushActive=true;feedback_.flushCheckpointSaved=false;
    StartSettle(Phase::FlushSettle,SettleAction::CaptureFlush,input);
}
inline void Cycle::CheckFinishEndpoint(const Input& input) noexcept
{
    if(!feedback_.finishRequiredCount||feedback_.nominalPhase!=Phase::InitialDepth||
       leg_.position!=leg_.length||!Detail::Positive(leg_.length)||leg_.speed!=0.0||
       output_.commandMCS!=request_.geometry.endMCS||feedback_.flushActive||finish_.forcedFlush)
    {Fail(Fault::Numeric);return;}
    feedback_.finishActive=true;feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Recheck);
    // This callback follows a NEW physical endpoint dwell. The GAP sample
    // arriving now, not the forward sample that began that dwell, decides
    // whether the same nominal leg may finish. A negative sample returns to
    // signed nominal stepping; no leg is rebuilt or teleported to its start.
    if(!input.gapValid||!std::isfinite(input.signedGapSpeedMmMin)){Fail(Fault::SourceLost);return;}
    if(input.signedGapSpeedMmMin<=0.0){feedback_.phase=feedback_.nominalPhase;RefreshOutput();return;}
    finish_.reworkLatched=false;
    if(finish_.returnedPending){
        if(feedback_.finishVerifiedCount>=feedback_.finishRequiredCount){Fail(Fault::Numeric);return;}
        finish_.returnedPending=false;++feedback_.finishVerifiedCount;
    }
    if(feedback_.finishVerifiedCount==feedback_.finishRequiredCount){
        feedback_.finishActive=false;feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Passed);
        feedback_.phase=Phase::Complete;leg_.speed=leg_.accelerationNow=0.0;RefreshOutput();return;
    }
    BeginFinishFlush(input);
}
inline bool Cycle::ObserveFinishProgress(double previousPosition) noexcept
{
    if(!feedback_.finishRequiredCount||!feedback_.finishActive||!(leg_.position<previousPosition))return true;
    // Only actually executed backward nominal progress invalidates prior
    // passes. A transient negative request while physically stopped does not
    // consume a wash or reset a checkpoint. HOLD/recovery/flush paths never
    // call this hook and retain their independent saved state.
    feedback_.finishVerifiedCount=0U;finish_.returnedPending=false;
    feedback_.finishPhase=static_cast<std::uint8_t>(FinishPhase::Machining);
    if(!finish_.reworkLatched){
        if(feedback_.finishReworkCount==(std::numeric_limits<std::uint64_t>::max)())
        {Fail(Fault::Numeric);return false;}
        ++feedback_.finishReworkCount;finish_.reworkLatched=true;
    }
    return true;
}
}
