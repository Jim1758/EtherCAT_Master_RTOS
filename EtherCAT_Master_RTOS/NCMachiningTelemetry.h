#pragma once
#include <cstddef>
#include <cstdint>

// EDM366: retry exhaustion is a process-unqualified normal endpoint, distinct
// from passed inspection, D completion, or a runtime fault. This page neither changes
// EDM_CNC_SHM/HOLD nor grants motion, recipe, discharge or timer authority.
namespace NCMachiningTelemetry {
constexpr std::uint32_t Magic=0x47434D4EU; // NMCG, little endian.
constexpr std::uint16_t VersionMajor=1U,VersionMinor=19U;
constexpr std::uint32_t WireSize=256U;
constexpr unsigned SequenceOffset=12U;
constexpr std::uint32_t MaximumLayers=4096U;
constexpr std::uint64_t MaximumDurationUs=86400000000ULL;
constexpr const wchar_t* SharedMemoryName=L"EDM_NC_MACHINING_DIAG_SHM";
enum class TimerState:std::uint32_t {Disabled=0U,Waiting=1U,Running=2U,Paused=3U,Expired=4U,Completed=5U};
enum class Phase:std::uint32_t {
    Unavailable=0U,Idle=1U,Machining=2U,AdditionalMachining=3U,AdditionalFinishing=4U,
    FinishVerification=5U,Held=6U,Returning=7U,Complete=8U,TimeLimit=9U,Fault=10U,Unsupported=11U,TimedFinishing=12U
};
enum class PauseReason:std::uint32_t {
    None=0U,Held=1U,InspectionJog=2U,Returning=3U,Flush=4U,GapRetreat=5U,
    EndpointQualification=6U,NotStarted=7U,Terminated=8U
};
enum class EndReason:std::uint32_t {None=0U,Completed=1U,TimeLimit=2U,Cancelled=3U,Fault=4U,TimedCompleted=5U,ProcessUnqualified=6U};
enum Capability:std::uint32_t {TotalLimit=1U,AdditionalMachining=2U,HoldPause=4U,P50FullSweep=8U,OrbitCompletion=16U,OrbitRetryLimit=32U,OrbitRunStatistics=64U};
constexpr std::uint32_t KnownCapabilities=127U;
constexpr std::uint32_t ParentValidFlag=1U,RuntimeSupportedFlag=2U,ActiveFlag=4U,KnownStatusFlags=7U;
struct Timer {
    std::uint64_t configuredUs=0ULL,elapsedUs=0ULL,remainingUs=0ULL;
    TimerState state=TimerState::Disabled;
    PauseReason pauseReason=PauseReason::None;
};
// Percent units preserve the E-condition six decimal digits:100% =100000000.
constexpr std::uint32_t OrbitPercentScale=1000000U,MaximumOrbitPercent=100000000U;
constexpr std::uint32_t OrbitEnabledFlag=1U,OrbitActiveFlag=2U,OrbitRatioAvailableFlag=4U,OrbitSimulationOnlyFlag=8U;
enum class OrbitCompletionState:std::uint32_t {Disabled=0U,Armed=1U,Inspecting=2U,Retry=3U,Passed=4U,Fault=5U,StoppedByTime=6U,RetryExhausted=7U};
constexpr std::uint32_t OrbitLastAvailableFlag=1U,OrbitLastAcceptedFlag=2U;
struct Image {
    std::uint64_t parentSession=0ULL,sampleTick=0ULL;
    std::uint32_t currentSourceG=0U,currentSourceP=0U,currentSourceLine=0U;
    std::int32_t currentSequenceN=-1;
    Phase phase=Phase::Unavailable;
    std::uint32_t capabilities=0U,layerIndex=0U,layerCount=0U;
    bool parentValid=false,runtimeSupported=false,active=false;
    EndReason endReason=EndReason::None;
    Timer total{},additional{};
    std::uint32_t orbitCompletionState=0U,orbitCompletionFlags=0U,orbitPercentScaled=0U,orbitTargetPercentScaled=0U;
    std::uint32_t orbitQualifiedPasses=0U,orbitRequiredPasses=0U;
    std::uint64_t orbitAttemptCount=0ULL;
    std::uint32_t orbitRetryLimit=0U,orbitEvaluationFlags=0U;
    std::uint64_t orbitFailedAttemptCount=0ULL;
    std::uint32_t orbitLastPercentScaled=0U,orbitLastQualifiedBefore=0U,orbitPeakQualifiedPasses=0U;
    // Run-scoped observations survive a following G01 or parentless M30 page.
    // The NC producer binds the scope; these counters confer no authority.
    std::uint64_t orbitRunCompletedParents=0ULL,orbitRunInspectionPassed=0ULL,orbitRunInspectionDisabled=0ULL;
    std::uint64_t orbitRunTimedCompleted=0ULL,orbitRunRetryExhausted=0ULL;
};
#pragma pack(push,1)
struct Wire {
    std::uint32_t magic;
    std::uint16_t versionMajor,versionMinor;
    std::uint32_t size,sequence,heartbeat,publisherFlags;
    std::uint64_t publisherGeneration;
    std::uint64_t parentSession,sampleTick;
    std::uint32_t currentSourceG,currentSourceP;
    std::int32_t currentSequenceN;
    std::uint32_t currentSourceLine;
    std::uint32_t phase,capabilities,statusFlags,totalState,additionalState;
    std::uint32_t totalPauseReason,additionalPauseReason,endReason,layerIndex,layerCount;
    std::uint64_t totalConfiguredUs,totalElapsedUs,totalRemainingUs;
    std::uint64_t additionalConfiguredUs,additionalElapsedUs,additionalRemainingUs;
    std::uint32_t orbitCompletionState,orbitCompletionFlags,orbitPercentScaled,orbitTargetPercentScaled;
    std::uint32_t orbitQualifiedPasses,orbitRequiredPasses,orbitRetryLimit,orbitEvaluationFlags;
    std::uint64_t orbitAttemptCount;
    std::uint64_t orbitFailedAttemptCount;
    std::uint32_t orbitLastPercentScaled,orbitLastQualifiedBefore,orbitPeakQualifiedPasses,orbitEvaluationReserved;
    std::uint64_t orbitRunCompletedParents,orbitRunInspectionPassed,orbitRunInspectionDisabled;
    std::uint64_t orbitRunTimedCompleted,orbitRunRetryExhausted;
};
#pragma pack(pop)
static_assert(sizeof(Wire)==WireSize,"Machining-time diagnostic wire size");
static_assert(offsetof(Wire,sequence)==SequenceOffset&&offsetof(Wire,publisherGeneration)==24U,"Machining-time publication offsets");
static_assert(offsetof(Wire,parentSession)==32U&&offsetof(Wire,sampleTick)==40U,"Machining-time parent offsets");
static_assert(offsetof(Wire,currentSourceG)==48U&&offsetof(Wire,currentSequenceN)==56U&&offsetof(Wire,currentSourceLine)==60U,"Machining-time source offsets");
static_assert(offsetof(Wire,phase)==64U&&offsetof(Wire,totalState)==76U&&offsetof(Wire,additionalState)==80U,"Machining-time phase offsets");
static_assert(offsetof(Wire,totalPauseReason)==84U&&offsetof(Wire,additionalPauseReason)==88U&&offsetof(Wire,endReason)==92U,"Machining-time reason offsets");
static_assert(offsetof(Wire,totalConfiguredUs)==104U&&offsetof(Wire,totalRemainingUs)==120U&&offsetof(Wire,additionalConfiguredUs)==128U&&offsetof(Wire,additionalRemainingUs)==144U,"Machining-time clock offsets");
static_assert(offsetof(Wire,orbitCompletionState)==152U&&offsetof(Wire,orbitPercentScaled)==160U&&offsetof(Wire,orbitRequiredPasses)==172U,"Orbit completion publication offsets");
static_assert(offsetof(Wire,orbitRetryLimit)==176U&&offsetof(Wire,orbitEvaluationFlags)==180U&&offsetof(Wire,orbitAttemptCount)==184U,"Orbit completion retry and attempt offsets");
static_assert(offsetof(Wire,orbitFailedAttemptCount)==192U&&offsetof(Wire,orbitLastPercentScaled)==200U&&offsetof(Wire,orbitLastQualifiedBefore)==204U&&offsetof(Wire,orbitPeakQualifiedPasses)==208U&&offsetof(Wire,orbitEvaluationReserved)==212U,"Completed inspection evidence offsets");
static_assert(offsetof(Wire,orbitRunCompletedParents)==216U&&offsetof(Wire,orbitRunInspectionPassed)==224U&&offsetof(Wire,orbitRunInspectionDisabled)==232U&&offsetof(Wire,orbitRunTimedCompleted)==240U&&offsetof(Wire,orbitRunRetryExhausted)==248U,"Run-scoped inspection statistics offsets");
inline bool ValidTimer(const Timer& timer,bool additional) noexcept {
    if(static_cast<std::uint32_t>(timer.state)>5U||static_cast<std::uint32_t>(timer.pauseReason)>8U||
       timer.configuredUs>MaximumDurationUs||timer.elapsedUs>timer.configuredUs||
       timer.remainingUs!=timer.configuredUs-timer.elapsedUs)return false;
    if(timer.state==TimerState::Disabled)
        return timer.configuredUs==0ULL&&timer.elapsedUs==0ULL&&timer.remainingUs==0ULL&&timer.pauseReason==PauseReason::None;
    if(timer.configuredUs==0ULL)return false;
    switch(timer.state){
    case TimerState::Waiting:
        return timer.elapsedUs==0ULL&&timer.pauseReason==PauseReason::NotStarted;
    case TimerState::Running:
        return timer.remainingUs!=0ULL&&timer.pauseReason==PauseReason::None;
    case TimerState::Paused:
        return timer.remainingUs!=0ULL&&timer.pauseReason!=PauseReason::None&&timer.pauseReason!=PauseReason::NotStarted;
    case TimerState::Expired:
        return !additional&&timer.remainingUs==0ULL&&timer.pauseReason==PauseReason::None;
    case TimerState::Completed:
        // A successful parent may finish before exhausting its total budget.
        return (!additional||timer.remainingUs==0ULL)&&timer.pauseReason==PauseReason::None;
    default:return false;
    }
}
inline bool FixedRadiusSource(std::uint32_t g,std::uint32_t p) noexcept
{return g==76U&&p>=1U&&p<=7U;}
inline bool SupportedSource(std::uint32_t g,std::uint32_t p) noexcept {
    return FixedRadiusSource(g,p)||((g==1U||g==2U||g==3U)&&p==0U)||(g==76U&&(p==11U||p==12U||p==13U||p==14U||p==15U||p==16U||p==17U||p==21U||p==22U||p==23U||p==24U||p==25U||p==26U||p==27U||p==50U));
}
// Reaching D is a successful ending trigger. Final completion still requires
// the separately proven stopped endpoint; these predicates only validate data.
inline bool TotalGoalReached(const Image& image) noexcept {
    return image.total.configuredUs!=0ULL&&image.total.elapsedUs==image.total.configuredUs&&
        image.total.remainingUs==0ULL&&image.total.state==TimerState::Completed&&
        image.total.pauseReason==PauseReason::None;
}
inline bool ValidTimedAdditional(const Image& image) noexcept {
    if(!image.additional.configuredUs)return image.additional.state==TimerState::Disabled;
    return image.additional.state==TimerState::Completed||
        (image.additional.state==TimerState::Paused&&image.additional.pauseReason==PauseReason::Terminated&&
         image.additional.remainingUs!=0ULL);
}
// A detached completed-sweep observation never substitutes for the current
// in-progress coverage. Fault observations may explicitly lack old evidence.
inline bool EmptyOrbitEvaluation(const Image& image) noexcept {
    return image.orbitEvaluationFlags==0U&&image.orbitFailedAttemptCount==0ULL&&
        image.orbitLastPercentScaled==0U&&image.orbitLastQualifiedBefore==0U&&image.orbitPeakQualifiedPasses==0U;
}
inline bool ValidOrbitEvaluation(const Image& image) noexcept {
    const auto attempts=image.orbitAttemptCount,failed=image.orbitFailedAttemptCount;
    const auto flags=image.orbitEvaluationFlags;
    if((flags&~3U)!=0U||failed>attempts||image.orbitLastPercentScaled>MaximumOrbitPercent)return false;
    if(!attempts)return EmptyOrbitEvaluation(image);
    if((flags&OrbitLastAvailableFlag)==0U)
        return image.orbitCompletionState==5U&&EmptyOrbitEvaluation(image);
    const bool accepted=(flags&OrbitLastAcceptedFlag)!=0U;
    if(accepted!=(image.orbitLastPercentScaled>=image.orbitTargetPercentScaled)||
       image.orbitLastQualifiedBefore>=image.orbitRequiredPasses||
       image.orbitPeakQualifiedPasses>image.orbitRequiredPasses||
       image.orbitPeakQualifiedPasses<image.orbitLastQualifiedBefore||(!accepted&&!failed))return false;
    const auto successful=attempts-failed;
    if(successful<image.orbitPeakQualifiedPasses||
       successful<static_cast<std::uint64_t>(image.orbitLastQualifiedBefore)+(accepted?1ULL:0ULL))return false;
    if(image.orbitCompletionState!=5U){
        if(image.orbitQualifiedPasses!=(accepted?image.orbitLastQualifiedBefore+1U:0U)||
           image.orbitPeakQualifiedPasses<image.orbitQualifiedPasses)return false;
        if(!failed&&(!accepted||attempts!=image.orbitQualifiedPasses||
           image.orbitPeakQualifiedPasses!=image.orbitQualifiedPasses))return false;
    }
    if(image.orbitRetryLimit&&
       (failed>static_cast<std::uint64_t>(image.orbitRetryLimit)+1ULL||
        (failed>image.orbitRetryLimit&&image.orbitCompletionState!=7U&&image.orbitCompletionState!=5U)))return false;
    return true;
}
inline bool ValidOrbitCompletion(const Image& image) noexcept {
    const auto state=image.orbitCompletionState,flags=image.orbitCompletionFlags;
    if(state==0U)return flags==0U&&image.orbitPercentScaled==0U&&image.orbitTargetPercentScaled==0U&&
        image.orbitQualifiedPasses==0U&&image.orbitRequiredPasses==0U&&image.orbitAttemptCount==0ULL&&
        image.orbitRetryLimit==0U&&(image.capabilities&OrbitRetryLimit)==0U&&EmptyOrbitEvaluation(image);
    if(state>7U||(flags&~15U)!=0U||(flags&OrbitEnabledFlag)==0U||!image.parentValid||!image.runtimeSupported||
       image.currentSourceG!=76U||!SupportedSource(image.currentSourceG,image.currentSourceP)||
       (image.capabilities&OrbitCompletion)==0U||image.orbitPercentScaled>MaximumOrbitPercent||
       image.orbitTargetPercentScaled<10U*OrbitPercentScale||image.orbitTargetPercentScaled>MaximumOrbitPercent||
       image.orbitRequiredPasses==0U||image.orbitRequiredPasses>99U||image.orbitQualifiedPasses>image.orbitRequiredPasses||
       image.orbitAttemptCount<image.orbitQualifiedPasses||image.orbitRetryLimit>99U||
       ((image.capabilities&OrbitRetryLimit)!=0U)!=(image.orbitRetryLimit!=0U)||!ValidOrbitEvaluation(image))return false;
    const bool inspecting=state==2U||state==3U;
    if(((flags&OrbitActiveFlag)!=0U)!=inspecting)return false;
    if((flags&OrbitRatioAvailableFlag)==0U&&image.orbitPercentScaled!=0U)return false;
    if(state==1U&&(image.orbitQualifiedPasses||image.orbitAttemptCount||image.orbitPercentScaled||
        (flags&OrbitRatioAvailableFlag)!=0U))return false;
    if(inspecting&&image.orbitQualifiedPasses==image.orbitRequiredPasses)return false;
    if((state==5U||state==6U)&&((flags&OrbitRatioAvailableFlag)!=0U||image.orbitPercentScaled!=0U))return false;
    if(state==6U&&(!TotalGoalReached(image)||
       (image.phase!=Phase::TimedFinishing&&image.phase!=Phase::Held&&image.phase!=Phase::Returning&&
        !(image.phase==Phase::Complete&&image.endReason==EndReason::TimedCompleted))))return false;
    if(state==4U&&(image.orbitQualifiedPasses!=image.orbitRequiredPasses||(flags&OrbitRatioAvailableFlag)==0U||
        image.orbitPercentScaled<image.orbitTargetPercentScaled||image.orbitPercentScaled!=image.orbitLastPercentScaled||
        (image.orbitEvaluationFlags&OrbitLastAcceptedFlag)==0U))return false;
    if(state==7U&&(image.phase!=Phase::Complete||image.endReason!=EndReason::ProcessUnqualified||image.active||
       !image.orbitRetryLimit||image.orbitFailedAttemptCount!=static_cast<std::uint64_t>(image.orbitRetryLimit)+1ULL||
       image.orbitEvaluationFlags!=OrbitLastAvailableFlag||image.orbitQualifiedPasses!=0U||
       (flags&OrbitRatioAvailableFlag)==0U||image.orbitPercentScaled!=image.orbitLastPercentScaled||
       image.orbitLastPercentScaled>=image.orbitTargetPercentScaled||TotalGoalReached(image)))return false;
    if(image.phase==Phase::Complete&&state!=4U&&
       !(state==6U&&image.endReason==EndReason::TimedCompleted)&&
       !(state==7U&&image.endReason==EndReason::ProcessUnqualified))return false;
    return true;
}
inline bool ValidOrbitRunStatistics(const Image& image) noexcept {
    const auto total=image.orbitRunCompletedParents;
    if((image.capabilities&OrbitRunStatistics)==0U)
        return total==0ULL&&image.orbitRunInspectionPassed==0ULL&&image.orbitRunInspectionDisabled==0ULL&&
            image.orbitRunTimedCompleted==0ULL&&image.orbitRunRetryExhausted==0ULL;
    if(!total)return false;
    // Subtract rather than summing attacker-controlled uint64 values.
    auto remaining=total;
    const std::uint64_t categories[]={image.orbitRunInspectionPassed,image.orbitRunInspectionDisabled,
        image.orbitRunTimedCompleted,image.orbitRunRetryExhausted};
    for(auto count:categories){if(count>remaining)return false;remaining-=count;}
    return remaining==0ULL;
}
inline bool Valid(const Image& image) noexcept {
    if(static_cast<std::uint32_t>(image.phase)>12U||static_cast<std::uint32_t>(image.endReason)>6U||
       (image.capabilities&~KnownCapabilities)!=0U||image.currentSequenceN< -1||image.layerCount>MaximumLayers||
       !ValidTimer(image.total,false)||!ValidTimer(image.additional,true)||!ValidOrbitCompletion(image)||
       !ValidOrbitRunStatistics(image))return false;
    if(image.total.configuredUs!=0ULL&&!(image.capabilities&TotalLimit))return false;
    if(image.additional.configuredUs!=0ULL&&!(image.capabilities&AdditionalMachining))return false;
    if(!image.parentValid){
        if(image.parentSession||image.sampleTick||image.runtimeSupported||image.active||image.layerIndex||image.layerCount||
           image.endReason!=EndReason::None||image.total.state!=TimerState::Disabled||image.additional.state!=TimerState::Disabled)return false;
        if(image.phase==Phase::Unsupported)
            return image.currentSourceG!=0U&&image.currentSourceLine!=0U&&!SupportedSource(image.currentSourceG,image.currentSourceP);
        return (image.phase==Phase::Unavailable||image.phase==Phase::Idle)&&image.currentSourceG==0U&&
            image.currentSourceP==0U&&image.currentSourceLine==0U&&image.currentSequenceN==-1;
    }
    if(!image.parentSession||!image.sampleTick||!image.currentSourceLine||!image.runtimeSupported||
       !SupportedSource(image.currentSourceG,image.currentSourceP)||image.layerIndex>image.layerCount||
       image.phase==Phase::Unavailable||image.phase==Phase::Idle||image.phase==Phase::Unsupported)return false;
    // P01-P07 and the other contours have no P50 layer identity.
    if(image.currentSourceG==76U&&image.currentSourceP==50U){if(!image.layerCount)return false;}
    else if(image.layerIndex||image.layerCount)return false;
    if(image.phase==Phase::Complete){
        if(image.active)return false;
        if(image.endReason==EndReason::TimedCompleted){
            if(!TotalGoalReached(image)||!ValidTimedAdditional(image)||
               (image.orbitCompletionState!=0U&&image.orbitCompletionState!=6U))return false;
        }else if((image.endReason!=EndReason::Completed&&image.endReason!=EndReason::ProcessUnqualified)||
           (image.endReason==EndReason::ProcessUnqualified&&image.orbitCompletionState!=7U)||
           (image.total.configuredUs&&(image.total.state!=TimerState::Completed||
              image.total.elapsedUs==image.total.configuredUs))||
           (image.additional.configuredUs&&image.additional.state!=TimerState::Completed))return false;
    }else if(image.phase==Phase::TimedFinishing){
        if(!image.active||image.endReason!=EndReason::None||!TotalGoalReached(image)||
           !ValidTimedAdditional(image)||(image.orbitCompletionState!=0U&&image.orbitCompletionState!=6U))return false;
    }else if(image.phase==Phase::TimeLimit){
        if(image.endReason!=EndReason::TimeLimit||image.total.state!=TimerState::Expired)return false;
    }else if(image.phase==Phase::Fault){
        if(image.endReason!=EndReason::Cancelled&&image.endReason!=EndReason::Fault)return false;
    }else if(image.endReason!=EndReason::None)return false;
    if(TotalGoalReached(image)&&(image.phase==Phase::Held||image.phase==Phase::Returning)&&
       (!ValidTimedAdditional(image)||(image.orbitCompletionState!=0U&&image.orbitCompletionState!=6U)))return false;
    if(TotalGoalReached(image)&&image.phase!=Phase::TimedFinishing&&image.phase!=Phase::Complete&&
       image.phase!=Phase::Held&&image.phase!=Phase::Returning&&image.phase!=Phase::Fault)return false;
    if(image.total.state==TimerState::Completed&&!TotalGoalReached(image)&&image.phase!=Phase::Complete)return false;
    if(image.total.state==TimerState::Expired&&image.phase!=Phase::TimeLimit)return false;
    if(image.phase==Phase::AdditionalFinishing&&
       (!image.additional.configuredUs||image.additional.state!=TimerState::Completed))return false;
    if(image.phase==Phase::AdditionalMachining&&(!image.additional.configuredUs||
       (image.additional.state!=TimerState::Running&&image.additional.state!=TimerState::Paused)))return false;
    return true;
}
inline bool Encode(const Image& image,std::uint64_t generation,std::uint32_t heartbeat,Wire& wire) noexcept {
    wire=Wire{};
    if(!generation||!heartbeat||!Valid(image))return false;
    wire.magic=Magic;wire.versionMajor=VersionMajor;wire.versionMinor=VersionMinor;wire.size=WireSize;
    wire.heartbeat=heartbeat;wire.publisherFlags=1U;wire.publisherGeneration=generation;
    wire.parentSession=image.parentSession;wire.sampleTick=image.sampleTick;
    wire.currentSourceG=image.currentSourceG;wire.currentSourceP=image.currentSourceP;
    wire.currentSequenceN=image.currentSequenceN;wire.currentSourceLine=image.currentSourceLine;
    wire.phase=static_cast<std::uint32_t>(image.phase);wire.capabilities=image.capabilities;
    wire.statusFlags=(image.parentValid?ParentValidFlag:0U)|(image.runtimeSupported?RuntimeSupportedFlag:0U)|(image.active?ActiveFlag:0U);
    wire.totalState=static_cast<std::uint32_t>(image.total.state);wire.additionalState=static_cast<std::uint32_t>(image.additional.state);
    wire.totalPauseReason=static_cast<std::uint32_t>(image.total.pauseReason);wire.additionalPauseReason=static_cast<std::uint32_t>(image.additional.pauseReason);
    wire.endReason=static_cast<std::uint32_t>(image.endReason);wire.layerIndex=image.layerIndex;wire.layerCount=image.layerCount;
    wire.totalConfiguredUs=image.total.configuredUs;wire.totalElapsedUs=image.total.elapsedUs;wire.totalRemainingUs=image.total.remainingUs;
    wire.additionalConfiguredUs=image.additional.configuredUs;wire.additionalElapsedUs=image.additional.elapsedUs;wire.additionalRemainingUs=image.additional.remainingUs;
    wire.orbitCompletionState=image.orbitCompletionState;wire.orbitCompletionFlags=image.orbitCompletionFlags;
    wire.orbitPercentScaled=image.orbitPercentScaled;wire.orbitTargetPercentScaled=image.orbitTargetPercentScaled;
    wire.orbitQualifiedPasses=image.orbitQualifiedPasses;wire.orbitRequiredPasses=image.orbitRequiredPasses;
    wire.orbitAttemptCount=image.orbitAttemptCount;
    wire.orbitRetryLimit=image.orbitRetryLimit;wire.orbitEvaluationFlags=image.orbitEvaluationFlags;
    wire.orbitFailedAttemptCount=image.orbitFailedAttemptCount;wire.orbitLastPercentScaled=image.orbitLastPercentScaled;
    wire.orbitLastQualifiedBefore=image.orbitLastQualifiedBefore;wire.orbitPeakQualifiedPasses=image.orbitPeakQualifiedPasses;
    wire.orbitRunCompletedParents=image.orbitRunCompletedParents;wire.orbitRunInspectionPassed=image.orbitRunInspectionPassed;
    wire.orbitRunInspectionDisabled=image.orbitRunInspectionDisabled;wire.orbitRunTimedCompleted=image.orbitRunTimedCompleted;
    wire.orbitRunRetryExhausted=image.orbitRunRetryExhausted;
    return true;
}
// Storage lifecycle only initializes/closes the map. The NC 100 ms owner makes
// one bounded copy; publishing performs no file access, allocation or Motion.
bool Initialize() noexcept;
void Shutdown() noexcept;
void Publish(const Image&) noexcept;
std::uint64_t CurrentPublisherGeneration() noexcept;
}
