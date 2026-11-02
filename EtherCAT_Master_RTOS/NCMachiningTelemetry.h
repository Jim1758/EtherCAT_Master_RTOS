#pragma once
#include <cstddef>
#include <cstdint>

// EDM345: NC-owner observations only. This independent page neither changes
// EDM_CNC_SHM/HOLD nor grants motion, recipe, discharge or timer authority.
namespace NCMachiningTelemetry {
constexpr std::uint32_t Magic=0x47434D4EU; // NMCG, little endian.
constexpr std::uint16_t VersionMajor=1U,VersionMinor=5U;
constexpr std::uint32_t WireSize=256U;
constexpr unsigned SequenceOffset=12U;
constexpr std::uint32_t MaximumLayers=4096U;
constexpr std::uint64_t MaximumDurationUs=86400000000ULL;
constexpr const wchar_t* SharedMemoryName=L"EDM_NC_MACHINING_DIAG_SHM";
enum class TimerState:std::uint32_t {Disabled=0U,Waiting=1U,Running=2U,Paused=3U,Expired=4U,Completed=5U};
enum class Phase:std::uint32_t {
    Unavailable=0U,Idle=1U,Machining=2U,AdditionalMachining=3U,AdditionalFinishing=4U,
    FinishVerification=5U,Held=6U,Returning=7U,Complete=8U,TimeLimit=9U,Fault=10U,Unsupported=11U
};
enum class PauseReason:std::uint32_t {
    None=0U,Held=1U,InspectionJog=2U,Returning=3U,Flush=4U,GapRetreat=5U,
    EndpointQualification=6U,NotStarted=7U,Terminated=8U
};
enum class EndReason:std::uint32_t {None=0U,Completed=1U,TimeLimit=2U,Cancelled=3U,Fault=4U};
enum Capability:std::uint32_t {TotalLimit=1U,AdditionalMachining=2U,HoldPause=4U,P50FullSweep=8U};
constexpr std::uint32_t KnownCapabilities=15U;
constexpr std::uint32_t ParentValidFlag=1U,RuntimeSupportedFlag=2U,ActiveFlag=4U,KnownStatusFlags=7U;
struct Timer {
    std::uint64_t configuredUs=0ULL,elapsedUs=0ULL,remainingUs=0ULL;
    TimerState state=TimerState::Disabled;
    PauseReason pauseReason=PauseReason::None;
};
struct Image {
    std::uint64_t parentSession=0ULL,sampleTick=0ULL;
    std::uint32_t currentSourceG=0U,currentSourceP=0U,currentSourceLine=0U;
    std::int32_t currentSequenceN=-1;
    Phase phase=Phase::Unavailable;
    std::uint32_t capabilities=0U,layerIndex=0U,layerCount=0U;
    bool parentValid=false,runtimeSupported=false,active=false;
    EndReason endReason=EndReason::None;
    Timer total{},additional{};
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
    std::uint8_t reserved[104];
};
#pragma pack(pop)
static_assert(sizeof(Wire)==WireSize,"Machining-time diagnostic wire size");
static_assert(offsetof(Wire,sequence)==SequenceOffset&&offsetof(Wire,publisherGeneration)==24U,"Machining-time publication offsets");
static_assert(offsetof(Wire,parentSession)==32U&&offsetof(Wire,sampleTick)==40U,"Machining-time parent offsets");
static_assert(offsetof(Wire,currentSourceG)==48U&&offsetof(Wire,currentSequenceN)==56U&&offsetof(Wire,currentSourceLine)==60U,"Machining-time source offsets");
static_assert(offsetof(Wire,phase)==64U&&offsetof(Wire,totalState)==76U&&offsetof(Wire,additionalState)==80U,"Machining-time phase offsets");
static_assert(offsetof(Wire,totalPauseReason)==84U&&offsetof(Wire,additionalPauseReason)==88U&&offsetof(Wire,endReason)==92U,"Machining-time reason offsets");
static_assert(offsetof(Wire,totalConfiguredUs)==104U&&offsetof(Wire,totalRemainingUs)==120U&&offsetof(Wire,additionalConfiguredUs)==128U&&offsetof(Wire,additionalRemainingUs)==144U,"Machining-time clock offsets");
static_assert(offsetof(Wire,reserved)==152U,"Machining-time reserved tail");
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
inline bool SupportedSource(std::uint32_t g,std::uint32_t p) noexcept {
    return ((g==1U||g==2U||g==3U)&&p==0U)||(g==76U&&(p==11U||p==12U||p==13U||p==14U||p==15U||p==50U));
}
inline bool Valid(const Image& image) noexcept {
    if(static_cast<std::uint32_t>(image.phase)>11U||static_cast<std::uint32_t>(image.endReason)>4U||
       (image.capabilities&~KnownCapabilities)!=0U||image.currentSequenceN< -1||image.layerCount>MaximumLayers||
       !ValidTimer(image.total,false)||!ValidTimer(image.additional,true))return false;
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
    // P11/P12/P13/P14/P15 are single contours, with no P50 layer identity.
    if(image.currentSourceG==76U&&image.currentSourceP==50U){if(!image.layerCount)return false;}
    else if(image.layerIndex||image.layerCount)return false;
    if(image.phase==Phase::Complete){
        if(image.active||image.endReason!=EndReason::Completed||
           (image.total.configuredUs&&image.total.state!=TimerState::Completed)||
           (image.additional.configuredUs&&image.additional.state!=TimerState::Completed))return false;
    }else if(image.phase==Phase::TimeLimit){
        if(image.endReason!=EndReason::TimeLimit||image.total.state!=TimerState::Expired)return false;
    }else if(image.phase==Phase::Fault){
        if(image.endReason!=EndReason::Cancelled&&image.endReason!=EndReason::Fault)return false;
    }else if(image.endReason!=EndReason::None)return false;
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
    return true;
}
// Storage lifecycle only initializes/closes the map. The NC 100 ms owner makes
// one bounded copy; publishing performs no file access, allocation or Motion.
bool Initialize() noexcept;
void Shutdown() noexcept;
void Publish(const Image&) noexcept;
std::uint64_t CurrentPublisherGeneration() noexcept;
}
