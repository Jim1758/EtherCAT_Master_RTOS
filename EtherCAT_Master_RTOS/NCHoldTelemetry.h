#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// EDM331: independent, producer-owned observation contract. This page never
// grants motion authority and does not change EDM_CNC_SHM or spindle reserves.
namespace NCHoldTelemetry {
constexpr std::uint32_t Magic=0x444C484EU; // NHLD, little endian.
constexpr std::uint16_t VersionMajor=1U,VersionMinor=0U;
constexpr std::uint32_t WireSize=192U,MaximumPreviousSegments=15U;
constexpr unsigned SequenceOffset=12U;
constexpr const wchar_t* SharedMemoryName=L"EDM_NC_HOLD_DIAG_SHM";
enum class Phase:std::uint32_t {
    Unavailable=0U,Idle=1U,Braking=2U,Retreating=3U,Held=4U,ManualJog=5U,
    JogReturning=6U,ProcessReturning=7U,Resuming=8U,Unsupported=9U,Fault=10U
};
enum Capability:std::uint32_t {
    Declaration=1U,Config=2U,CurrentParentRecovery=4U,PreviousParentRecovery=8U,InspectionXYZ=16U
};
constexpr std::uint32_t KnownCapabilities=31U;
using Point3=std::array<double,3U>;
struct Image {
    std::uint32_t declaredSegments=0U,effectiveSegments=0U,availableSegments=0U;
    Phase phase=Phase::Unavailable;
    std::uint64_t holdSequence=0ULL;
    std::uint64_t parentSession=0ULL;
    std::uint32_t capabilities=0U,currentSourceG=0U,inspectionAxisMask=0U;
    bool runtimeSupported=false,canJog=false,canResume=false,manualTraceValid=false,declared=false;
    bool retreatAnchorValid=false,processPointValid=false;
    Point3 retreatAnchorXYZ{},processPointXYZ{};
};
#pragma pack(push,1)
struct Wire {
    std::uint32_t magic;
    std::uint16_t versionMajor,versionMinor;
    std::uint32_t size,sequence,heartbeat,publisherFlags;
    std::uint64_t publisherGeneration;
    std::uint32_t declaredSegments,effectiveSegments,availableSegments,phase;
    std::uint64_t holdSequence;
    std::uint32_t capabilities,currentSourceG,statusFlags,inspectionAxisMask;
    double retreatAnchorXYZ[3],processPointXYZ[3];
    std::uint64_t parentSession;
    std::uint8_t reserved[64];
};
#pragma pack(pop)
static_assert(sizeof(Wire)==WireSize,"HOLD diagnostic wire size");
static_assert(offsetof(Wire,sequence)==SequenceOffset,"HOLD seqlock offset");
static_assert(offsetof(Wire,declaredSegments)==32U&&offsetof(Wire,holdSequence)==48U,
    "HOLD declaration offsets");
static_assert(offsetof(Wire,retreatAnchorXYZ)==72U&&offsetof(Wire,processPointXYZ)==96U,
    "HOLD coordinate offsets");
static_assert(offsetof(Wire,parentSession)==120U,"HOLD parent session offset");
inline bool Valid(const Image& image) noexcept {
    if(image.declaredSegments>MaximumPreviousSegments||image.effectiveSegments>image.declaredSegments||
       image.availableSegments>MaximumPreviousSegments||image.effectiveSegments>image.availableSegments||
       (!image.declared&&image.declaredSegments!=0U)||(image.declared&&!(image.capabilities&Declaration))||
       static_cast<std::uint32_t>(image.phase)>10U||
       (image.capabilities&~KnownCapabilities)!=0U||
       (image.inspectionAxisMask!=0U&&image.inspectionAxisMask!=7U))return false;
    if(image.runtimeSupported&&image.currentSourceG!=1U&&image.currentSourceG!=2U&&
       image.currentSourceG!=3U&&image.currentSourceG!=76U)return false;
    if(image.canJog&&(!image.runtimeSupported||!(image.capabilities&InspectionXYZ)||
       image.inspectionAxisMask!=7U||(image.phase!=Phase::Held&&image.phase!=Phase::ManualJog)))return false;
    if(image.canResume&&(!image.runtimeSupported||!(image.capabilities&CurrentParentRecovery)||
       image.phase!=Phase::Held))return false;
    for(unsigned a=0U;a<3U;++a)
        if((image.retreatAnchorValid&&!std::isfinite(image.retreatAnchorXYZ[a]))||
           (image.processPointValid&&!std::isfinite(image.processPointXYZ[a])))return false;
    return true;
}
inline bool Encode(const Image& image,std::uint64_t generation,std::uint32_t heartbeat,Wire& wire) noexcept {
    wire=Wire{};
    if(!generation||!Valid(image))return false;
    wire.magic=Magic;wire.versionMajor=VersionMajor;wire.versionMinor=VersionMinor;wire.size=WireSize;
    wire.heartbeat=heartbeat;wire.publisherFlags=1U;wire.publisherGeneration=generation;
    wire.declaredSegments=image.declaredSegments;wire.effectiveSegments=image.effectiveSegments;
    wire.availableSegments=image.availableSegments;wire.phase=static_cast<std::uint32_t>(image.phase);
    wire.holdSequence=image.holdSequence;wire.capabilities=image.capabilities;wire.currentSourceG=image.currentSourceG;
    wire.parentSession=image.parentSession;
    wire.statusFlags=(image.runtimeSupported?1U:0U)|(image.canJog?2U:0U)|(image.canResume?4U:0U)|
        (image.manualTraceValid?8U:0U)|(image.declared?16U:0U)|(image.retreatAnchorValid?32U:0U)|
        (image.processPointValid?64U:0U);
    wire.inspectionAxisMask=image.inspectionAxisMask;
    for(unsigned a=0U;a<3U;++a){
        wire.retreatAnchorXYZ[a]=image.retreatAnchorValid?image.retreatAnchorXYZ[a]:0.;
        wire.processPointXYZ[a]=image.processPointValid?image.processPointXYZ[a]:0.;
    }
    return true;
}
// Initialize/Shutdown run only in the existing non-RT storage lifecycle. Publish
// is one bounded 192-byte copy on the NC 100 ms owner thread; never creates,
// retries, allocates, reads files, sleeps or acquires a motion lease.
bool Initialize() noexcept;
void Shutdown() noexcept;
void Publish(const Image&) noexcept;
std::uint64_t CurrentPublisherGeneration() noexcept;
}
