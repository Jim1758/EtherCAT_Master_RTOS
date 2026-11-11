#pragma once
// Included after Cycle. A new E16/E29 criterion starts a fresh evidence epoch
// at a complete-sweep boundary; old passes never become new-criterion proof.
namespace NCG76SphereCycle
{
namespace LiveRecipeCompletionDetail
{
inline std::uint64_t Archive(std::uint64_t total,std::uint64_t added) noexcept
{
    const auto maximum=(std::numeric_limits<std::uint64_t>::max)();
    return added>maximum-total?maximum:total+added;
}
}

inline void Cycle::ApplyLiveOrbitCriteria(std::uint32_t threshold,std::uint8_t retry) noexcept
{
    auto& o=orbitInspection_;
    archivedOrbitAttempts_=LiveRecipeCompletionDetail::Archive(archivedOrbitAttempts_,o.attempts);
    archivedOrbitFailures_=LiveRecipeCompletionDetail::Archive(archivedOrbitFailures_,o.failedSweeps);
    // Preserve the already admitted sweep geometry. Reconstruct only the
    // evidence state in place, avoiding a coverage-sized RT stack temporary.
    const double expectedLength=o.expectedLength;const auto expectedCircles=o.expectedCircles;
    o.~OrbitInspectionState();::new (static_cast<void*>(&o)) OrbitInspectionState{};
    o.expectedLength=expectedLength;o.expectedCircles=expectedCircles;
    o.state=NCG76OrbitCompletion::State::Armed;
    liveOrbitThreshold_=threshold;liveOrbitRetry_=retry;liveOrbitCriteria_=true;
    pendingLiveOrbitCriteria_=false;pendingOrbitThreshold_=0U;pendingOrbitRetry_=0U;
    RefreshOrbitCompletionFeedback();
}

inline bool Cycle::ApplyPendingLiveOrbitCriteria() noexcept
{
    if(!pendingLiveOrbitCriteria_)return true;
    if(orbitInspection_.active||machiningTime_.totalExpired||feedback_.timeLimitReached)return false;
    ApplyLiveOrbitCriteria(pendingOrbitThreshold_,pendingOrbitRetry_);return true;
}

inline bool Cycle::UpdateLiveOrbitCompletion(std::uint32_t threshold,std::uint8_t retry) noexcept
{
    if(!initialized_||!output_.active||output_.faulted||output_.complete||
       !NCG76OrbitCompletion::ValidConfig(request_.orbitInspectionRounds,threshold,retry))return false;
    if(threshold==LiveOrbitThresholdScaled()&&retry==LiveOrbitRetryLimit()){
        // Latest-value cancellation is a true no-op for an unchanged target.
        pendingLiveOrbitCriteria_=false;pendingOrbitThreshold_=0U;pendingOrbitRetry_=0U;return true;
    }
    if(orbitInspection_.active||machiningTime_.totalExpired||feedback_.timeLimitReached){
        pendingOrbitThreshold_=threshold;pendingOrbitRetry_=retry;pendingLiveOrbitCriteria_=true;return true;
    }
    ApplyLiveOrbitCriteria(threshold,retry);return true;
}
}
