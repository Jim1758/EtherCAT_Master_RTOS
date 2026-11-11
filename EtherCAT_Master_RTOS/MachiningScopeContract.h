#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>
namespace EDMGapInput { struct Profile; }
class EtherCatMaster;

// EDM379 v2: telemetry has no motion, output, alarm or discharge authority.
// Windows opens DATA read-only. CONTROL is a separate, bounded observation
// lease; its fields are never read by the servo callback.
namespace MachiningScope379
{
constexpr std::uint32_t DataMagic=0x39504353U, ControlMagic=0x39434353U;
constexpr std::uint32_t Version=2U, Capacity=8192U;
constexpr std::uint64_t LeaseNs=2000000000ULL;
enum : std::uint32_t { Idle=0U, Active=1U, Stopped=2U, LeaseExpired=3U, Rejected=4U };
enum : std::uint32_t { PdoValid=1U, AxisValid=2U, GapValid=4U, TimestampValid=8U, Rotary=16U };
// EDM380 optional extension in the v1 reserved tail. Zero capability means
// unknown; NC RUN is never interpreted as an EDM feed or energized output.
constexpr std::uint64_t ProcessStateCapability=1ULL,EdmDetailsCapability=2ULL;
constexpr std::uint32_t EdmValid=1U,GainRegionValid=2U,E5TimerValid=4U,
    E5Eligible=8U,E5GateLatched=16U,E5GateAllow=32U,E5ShortBlocked=64U,E5Pending=128U,
    EdmSuspended=256U,PeriodicFlushActive=512U,E5Mode1=1024U,FlushPhysicalMotion=2048U,
    CoreForwardEligible=4096U,FlushEnabled=8192U,EdmFlagsMask=16383U;
constexpr std::uint32_t ProcessValid=1U,ProcessSourceMode=0x100U,
    ProcessSourceRt=0x200U,ProcessSourceNc=0x300U,ProcessMaximumAgeUs=100000U;
enum class ProcessPhase : std::uint32_t { Unknown=0U,Idle=1U,Machining=2U,
    FlushBrake=3U,FlushAnchorDwell=4U,FlushOutbound=5U,FlushTopDwell=6U,
    FlushReturning=7U,FlushFinalApproach=8U,FlushReturnDwell=9U,FlushResume=10U,
    ToCenter=11U,CenterDwell=12U,ToAnchor=13U,Held=14U,Stopping=15U,
    Complete=16U,Fault=17U,ReturningToWork=18U,RapidOrJog=19U,Waiting=20U,
    ProgramRun=21U,HoldRetreat=22U };
inline ProcessPhase NcProcessPhase(std::uint32_t value) noexcept
{
    switch(value) {case 0U:return ProcessPhase::Waiting;case 1U:case 2U:return ProcessPhase::Idle;
    case 3U:return ProcessPhase::ProgramRun;case 4U:return ProcessPhase::Complete;
    case 5U:return ProcessPhase::Held;case 6U:return ProcessPhase::Stopping;
    case 7U:return ProcessPhase::Fault;default:return ProcessPhase::Unknown;}
}
struct alignas(8) Header
{
    std::uint32_t magic,version,structBytes,headerBytes,recordBytes,capacity;
    std::uint64_t bootId,qpcFrequency;
    std::uint32_t sequence,state;
    std::uint64_t session,ownerToken,writeCount,lastPdoTick;
    std::uint32_t axisIndex,periodTicks;
    std::uint64_t requestId,reserved[20];
};
struct alignas(8) Record
{
    std::uint32_t sequence,flags;
    std::uint64_t session,sampleNumber,pdoTick,sampleQpc,gapSequence,gapQpc;
    std::uint32_t axisIndex,gapAdIndex;
    double commandPosition,actualPosition,lag,commandVelocity,actualVelocity,motorRpm;
    double gapVoltage,gapReferenceVoltage,gapBoardVoltage;
    std::int32_t gapRaw;
    std::uint32_t gapReason,gapSource,processPhase,processFlags,processAgeUs;
    // v2 tail: same-callback RT evidence. Event times share the core's
    // monotonic microsecond clock; they are not the earlier PDO receipt QPC.
    std::int32_t gainRegion;
    std::uint32_t edmFlags;
    std::uint64_t parentSession,curveTick,edmNowUs,e5ElapsedUs,e5TargetUs,e5StartUs,e5DueUs;
    std::uint64_t flushAcceptedUs,flushMotionUs,e5RequestSequence;
    double filteredGapVoltage;
};
struct alignas(8) Data { Header header; Record records[Capacity]; };
struct alignas(8) Control
{
    std::uint32_t magic,version,structBytes,reserved32;
    std::uint64_t bootId,ownerToken;
    std::uint32_t sequence,enabled,axisIndex,periodTicks;
    std::uint64_t heartbeat,requestId,echoBootId,echoOwnerToken,reserved[6];
};
static_assert(sizeof(Header)==256U && sizeof(Record)==256U && sizeof(Data)==2097408U && sizeof(Control)==128U,"Scope v2 ABI size");
static_assert(offsetof(Header,sequence)==40U && offsetof(Header,requestId)==88U && offsetof(Record,gapVoltage)==112U && offsetof(Control,ownerToken)==24U && offsetof(Control,echoOwnerToken)==72U,"Scope v2 unchanged prefix ABI offsets");
static_assert(std::is_trivially_copyable<Record>::value,"Fixed telemetry payload");
static_assert(offsetof(Header,reserved)==96U && offsetof(Record,processPhase)==148U &&
    offsetof(Record,processFlags)==152U && offsetof(Record,processAgeUs)==156U,"EDM380 optional process-state offsets");
static_assert(offsetof(Record,gainRegion)==160U&&offsetof(Record,edmFlags)==164U&&
    offsetof(Record,parentSession)==168U&&offsetof(Record,curveTick)==176U&&offsetof(Record,edmNowUs)==184U&&
    offsetof(Record,e5ElapsedUs)==192U&&offsetof(Record,e5TargetUs)==200U&&offsetof(Record,e5StartUs)==208U&&
    offsetof(Record,e5DueUs)==216U&&offsetof(Record,flushAcceptedUs)==224U&&offsetof(Record,flushMotionUs)==232U&&
    offsetof(Record,e5RequestSequence)==240U&&offsetof(Record,filteredGapVoltage)==248U,"Scope v2 RT evidence offsets");
inline bool ValidPeriod(std::uint32_t ticks) noexcept
{ return ticks==1U || ticks==2U || ticks==4U || ticks==8U || ticks==20U; }

// These operations live in existing DC_Runtime.cpp; no new project CPP item.
bool Initialize() noexcept;                 // P50, after normal SHM startup
void ShutdownAfterCallbacksStopped() noexcept;
bool ClientClaimed() noexcept;              // P50-only, avoids idle clock reads
void Service(std::uint32_t configuredAxes,std::uint64_t nowMasterNs) noexcept; // existing P50 10ms
bool Enabled() noexcept;                    // relaxed atomic + PDO-local stop edge
bool Begin(std::uint64_t tick,std::uint64_t nowMasterNs,std::uint32_t& axis) noexcept;
void Commit(Record& value) noexcept;         // PDO owner, bounded fixed copy
void ConfigureGapBeforeStart(const EDMGapInput::Profile& profile) noexcept;
void PublishNcState(std::uint32_t ncState,std::uint64_t nowMasterNs) noexcept; // NC/P50 owner only
void CaptureFromPdo(EtherCatMaster* master,std::uint64_t tick,std::int64_t receiptQpc,
    std::uint64_t nowMasterNs,bool pdoValid) noexcept;
}
