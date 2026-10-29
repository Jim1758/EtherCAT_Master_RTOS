#pragma once
#include <vector>
#include <memory>
#include <new>
#include "EtherCatTypes.h" // 必須包含這個，才能認識 ServoDrive
#include "HomeTypes.h"
#include <cmath>
#include <cstdint>
#include <atomic>
#include <array>
#include <cstddef>
#include <cstring>
#include <queue> // 引入佇列函式庫
#include <type_traits>
#include <deque>
#include "CoordinateManager.h"
#include "MechanicalCompensationCoordinates.h" // PBC-2 dual-coordinate contract
#include "CompensationEngine.h" // 引入剛寫好的標頭檔
#include "SHM_Types.h"
#include "MotionExecutionContract.h"
#include "MotionCore_G80.h"
#include "MotionCore_RotarySpindle.h"
#include "MotionRotarySpindleProof.h"
#include "MotionCore_G76Sphere.h"
#include "EDMShortCircuitProtection.h"
#include "G80VoltageSimulator.h"
#include "G80PID.h"

class NCEccentricCRuntimeValue; // BASE79D read-only pre-start binding
struct MotionEccentricCConsumerState;
struct MotionEccentricCContext;
struct NCEccentricCProfilePoint;
#include "MotionNCTranslationProof.h"
#include "MotionCommandPathModeTransport.h"
#include "MotionQueueTailTransaction.h"
#include "MotionRotaryTarget.h"
#include "MotionCommandedEndpointReceipt.h" // BQ producer-only data export
#include "NCG76OrbitPlan.h"
#include "NCG76MixedPlan.h"
#include "EDM173G76Gap.h"
#include "EDM173ServoInputDiagnostic.h"
#include "EDM174CommunicationTrace.h"
#include "EDM174G76LeafTiming.h"
#include "EDM206SyncHistory.h"
#include "MotionG76OrbitReceipt.h"
#include "NCRotaryFeedScope.h" // BASE68 pure rotary admission
#include "NCZCFeedLine.h" // BASE70 common-time Z/C geometry
#include "NCXYZCFeedLine.h"
#include "NCXYZCAbsoluteFeedTarget.h"
#include "NCXYZCFeedScope.h"
#include "NCXYZCUVFeedLine.h"
#include "NCXYZCUVAbsoluteFeedTarget.h"
#include "NCXYZCUVFeedScope.h"
#include "NCZCFeedScope.h" // BASE70 explicit G91 Z/C admission
#include "MotionFeedLineReceipt.h" // BX G01 producer-owned workspace

struct MotionEccentricCProducerWorkspace;
#include "MotionFeedArcReceipt.h" // BY G02/G03 producer-owned workspace
#include "MotionPathCoreRetainedReceipt.h" // BZ immutable traversal workspace
#include "MotionPathCoreHoldExcursion.h" // CB same-source held excursion
#include "MotionCommandRing.h"
#include "MotionFeedbackRing.h"
#include "RotaryPidDiagnostic.h" // ROTPID_DIAG1 observation-only sidecar
#include "MotionServoHandoffDiagnostic.h" // PBC-3A NIC handoff observation, not drive ACK
#include "MotionPbcXSendAudit.h" // PBC-3C zero-offset send transaction rehearsal
#include "MotionPbcReferenceContract.h"
#include "MotionPbcHomeStopProof.h"
#include "MotionPbcHomeCaptureProof.h"
#include "MotionAxisCommandMailbox.h"
#include "EDMZFixtureSession.h" // EDM28 isolated, finite single-Z diagnostic
#include "EDMGapRTRawSource.h" // EDM62 bounded raw capture and calibrated source
#include "EDMExecutionCancelFence.h"
#include "EDMExecutionFeedbackChannel.h"
#include "EDMExecutionFrameIdentity.h"
#include "EDMRTShadowChannel.h" // EDM48 bounded NC -> RT shadow transport
#include "EDMGapServoRTSession.h" // EDM55 continuous RT SIM servo, advisory values only
#include "EDMPhysicalServoRTSession.h" // EDM69 active PHYSICAL input, advisory values only
#include "EDMCNCRetainedRTSession.h" // EDM83 retained-path calculation; never an axis command
#include "EDMCoordinatedRTSession.h" // EDM84 six-axis composition; diagnostic values only
#include "EDMSixAxisPhysicalGuard.h"
#include "EDMSixAxisGapBoundary.h"
#include "EDMSixAxisGapSegment.h"
#include "EDMSixAxisCommonPath.h"
#include "EDMSixAxisB2Excursion.h"
#include "EDMSixAxisCurveRuntime.h"
#include "EDMSixAxisArcExecution.h"
#include "EDMSixAxisCurveB2Excursion.h"
#include "EDMSixAxisModeFlush.h"
#include "EDMSixAxisModeInterruption.h"
#include "EDMSixAxisGapTraversal.h"
#include "EDMSixAxisGapTraversalFixture.h"
#include "EDMSixAxisGapModeCoordinator.h"
#include "EDMSixAxisGapModeFixture.h"
#include "EDMSixAxisGapTuning.h"
#include "EDMSixAxisGapContinuous.h"
#include "EDM129PhysicalSource.h"
#include "EDM130WorkTimer.h"
#include "EDM139GapStimulus.h"
#include "EDM149VoltageMotion.h"
#include "EDM169ManualVoltage.h"
#include "EDM157GapPause.h"
#include "EDMSixAxisGapEndurance.h"
#include "EDMSixAxisGapPairedPath.h"
#include "EDMSixAxisGapFlushPath.h"
#include "EDMSixAxisSerializedGuard.h"
#include "AlarmManager.h"

// EDM126 supplementary evidence; only live authority grants physical output.
namespace EDM126
{
    // EDM133 FIX1 observations never participate in an authority decision.
    enum class FirstCauseSite : std::uint8_t { None, SourcePrecheck, PerAxisOrphan, GroupPrescan, GuardSource };
    enum FirstCauseGate : std::uint32_t {
        FirstCauseCurrent=1U, FirstCauseSource=2U, FirstCauseEpoch=4U,
        FirstCauseLease=8U, FirstCauseCommand=16U, FirstCauseMember=32U,
        FirstCauseMapping=64U, FirstCauseOrphan=128U
    };
    enum FirstCauseFlag : std::uint32_t {
        FirstCauseTimingCurrent=1U, FirstCauseProfileCurrent=2U, FirstCauseParticipationCurrent=4U,
        FirstCauseGuardFaulted=8U, FirstCauseGroupActive=16U, FirstCauseEpochMatches=32U,
        FirstCauseLeaseCurrent=64U, FirstCauseIdentityMatches=128U, FirstCauseGuardForceZero=256U,
        FirstCauseProposalDiffers=512U, FirstCausePrivateHeightCurrent=1024U,
        FirstCauseCommittedHeightValid=2048U, FirstCausePublicHeightValid=4096U,
        FirstCauseWorkValid=8192U
    };
    struct FirstCauseRecord
    {
        std::uint64_t tick=0ULL, nowUs=0ULL, executionEpoch=0ULL, ownerGeneration=0ULL;
        std::uint64_t expectedEpoch=0ULL, liveEpoch=0ULL, committedTick=0ULL, publishedTick=0ULL;
        std::uint64_t eventIndex=0ULL, privateTriggered=0ULL, privateCompleted=0ULL;
        std::uint64_t publicTriggered=0ULL, publicCompleted=0ULL;
        std::uint32_t evaluatedGates=0U, failedGates=0U, flags=0U;
        std::uint32_t orderedGroupAxes=0xffffffU, orderedCommandAxes=0xffffffU;
        std::uint16_t statusWord[6U]{};
        std::int8_t modes[6U]{};
        FirstCauseSite site=FirstCauseSite::None;
        std::int8_t axis=-1, groupAxisCount=0, commandAxisCount=0;
        std::uint8_t memberMask=0U, interpolatingMask=0U, inputMask=0U;
        std::uint8_t committedPhase=0U, proposedPhase=0U, publishedPhase=0U;
        std::uint8_t guardReason=0U, commandFailure=0U, owner=0U;
        std::uint8_t existMask=0U, configMatchMask=0U, mappingMatchMask=0U;
        std::uint8_t faultMask=0U, servoOnMask=0U, axisIndexMatchMask=0U;
    };
    static_assert(sizeof(FirstCauseRecord)<=176U && std::is_trivially_copyable<FirstCauseRecord>::value,
        "EDM133 FIX1 first-cause evidence stays bounded.");
    struct MotionFeedback
    {
        Feedback traversal{};
        Source authorizedSource{};
        std::uint64_t session=0ULL, armEpoch=0ULL, ownerGeneration=0ULL, liveEpoch=0ULL;
        std::uint64_t tick=0ULL, nowUs=0ULL, lastMappingTick=0ULL, lastMappingUs=0ULL;
        std::uint64_t sourceSequence=0ULL, sourceCapturedUs=0ULL;
        unsigned fixture=0U, authorizedRows=0U, completedRows=0U;
        bool enabled=false, active=false, sourceReady=false, rowActive=false, rowCompleted=false, faulted=false;
        bool physicalAd=false;
        double voltage=0.0, targetRatio=0.0;
        double controlVoltageV=0.0;
        std::uint64_t controlSequence=0ULL,controlCapturedUs=0ULL;
        EDM88::Reason sourceReason=EDM88::Reason::None;
        Reason reason=Reason::None;
        FirstCauseRecord firstSource{}, firstOrphan{};
    };
    static_assert(sizeof(MotionFeedback)<=1056U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM126 feedback must be fixed size.");
}

namespace EDM128
{
    struct MotionFeedback
    {
        Feedback coordinator{};
        EDM116::Policy rowPolicy{};
        EDM107::FlushProfile flushProfile{};
        std::uint64_t session=0ULL, armEpoch=0ULL, liveEpoch=0ULL, ownerGeneration=0ULL;
        std::uint64_t tick=0ULL, nowUs=0ULL, lastMappingTick=0ULL, lastMappingUs=0ULL;
        unsigned fixture=0U, authorizedRows=0U, completedRows=0U;
        bool enabled=false, active=false, faulted=false;
        bool physicalAd=false;
        bool workTimeFlush=false;
        EDM130::Recipe timerRecipe{};
    };
    static_assert(sizeof(MotionFeedback)<=1664U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM128 coordination evidence must stay bounded.");
}

namespace EDM129
{
    struct MotionFeedback
    {
        Binding binding{};
        SourceEvidence source{};
        EDM139::Evidence diagnostic{};
        std::uint64_t session=0ULL, armEpoch=0ULL, liveEpoch=0ULL, ownerGeneration=0ULL;
        std::uint64_t tick=0ULL, nowUs=0ULL;
        bool enabled=false, active=false, sourceReady=false, faulted=false;
    };
    static_assert(sizeof(MotionFeedback)<=1024U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM129 physical source evidence must stay bounded.");
}

// EDM109 records only geometry actually admitted and mapped by the existing
// six-axis guard. This value is evidence, never a cross-segment motion permit.
namespace EDM109
{
    struct Feedback
    {
        std::uint64_t session = 0ULL, armEpoch = 0ULL, epoch = 0ULL;
        std::uint64_t ownerGeneration = 0ULL, run = 0ULL, cache = 0ULL, dispatch = 0ULL;
        std::uint64_t segmentId = 0ULL, translationGeneration = 0ULL;
        std::uint64_t mappedSamples = 0ULL, lastMappingTick = 0ULL, lastMappingUs = 0ULL;
        std::array<double, 6U> startPulse{}, endPulse{}, pulsePerUnit{};
        std::array<std::uint64_t, 6U> configFingerprint{}, mappingFingerprint{};
        double pulseLength = 0.0;
        int sourcePC = -1;
        unsigned step = 0U, begunSegments = 0U, mappedSegments = 0U;
        bool enabled = false, lineValid = false, mappedOutput = false, completed = false;
    };
    static_assert(sizeof(Feedback) <= 512U && std::is_trivially_copyable<Feedback>::value,
        "EDM109 feedback must remain a small fixed RT publication.");
}

namespace EDM110
{
    struct MotionFeedback
    {
        Feedback excursion{};
        EDM109::Source trigger{};
        std::uint64_t session = 0ULL, armEpoch = 0ULL, ownerGeneration = 0ULL;
        std::uint64_t previousBoundaryTick = 0ULL, previousCommit = 0ULL, lastPhysicalTick = 0ULL;
        unsigned rangeResets = 0U, eventsStarted = 0U, eventsCompleted = 0U;
        bool enabled = false, tokenValidated = false, geometryCurrent = false, repeat = false;
    };
    static_assert(sizeof(MotionFeedback) <= 512U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM110 motion evidence must remain bounded.");
}

namespace EDM115
{
    struct MotionFeedback
    {
        Feedback excursion{};
        EDM109::Source trigger{};
        CompletionToken previous{};
        Point anchorPulse{}, returnedPulse{};
        double anchorQ = 0.0;
        std::uint64_t session = 0ULL, armEpoch = 0ULL, ownerGeneration = 0ULL;
        std::uint64_t previousBoundaryTick = 0ULL, previousCommit = 0ULL, lastPhysicalTick = 0ULL;
        unsigned rangeResets = 0U, eventsStarted = 0U, eventsCompleted = 0U;
        unsigned completedDwellsTotal = 0U, anchorReturnsTotal = 0U;
        EDM114::Kind currentKind = EDM114::Kind::None, previousKind = EDM114::Kind::None;
        bool enabled = false, tokenValidated = false, geometryCurrent = false;
    };
    static_assert(sizeof(MotionFeedback) <= 1024U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM115 motion evidence must remain bounded.");
}

namespace EDM116
{
    struct MotionFeedback
    {
        Feedback excursion{};
        EDM109::Source trigger{}, rowSource{};
        Policy policy{}, rowPolicy{};
        EDM115::CompletionToken previous{};
        Point anchorPulse{}, returnedPulse{}, retreatPulse{};
        double anchorQ=0.0, strokeNativeMm=0.0;
        std::uint64_t deadlineUs=0ULL, session=0ULL, armEpoch=0ULL, ownerGeneration=0ULL;
        std::uint64_t previousBoundaryTick=0ULL, previousCommit=0ULL, lastPhysicalTick=0ULL;
        std::array<unsigned,EDM116::ModeCount> modeStarted{}, modeCompleted{};
        std::uint64_t topDwellTicks=0ULL,topDwellUs=0ULL,returnDwellTicks=0ULL,returnDwellUs=0ULL;
        unsigned topSettledCycles=0U,returnSettledCycles=0U;
        unsigned rangeResets=0U, eventsStarted=0U, eventsCompleted=0U;
        unsigned completedDwellsTotal=0U, anchorReturnsTotal=0U, authorizedRows=0U, completedRows=0U;
        EDM114::Kind currentKind=EDM114::Kind::None, previousKind=EDM114::Kind::None;
        Fixture fixture=Fixture::Canonical116; // independently frozen by RT arm
        bool enabled=false, tokenValidated=false, geometryCurrent=false;
    };
    static_assert(sizeof(MotionFeedback)<=1024U && std::is_trivially_copyable<MotionFeedback>::value,
        "EDM116 fixed mode and source evidence");
}

class EtherCatMaster;
constexpr int MAX_AXES = 8;//最大軸數宣告
static_assert(MAX_AXES == MOTION_COMMANDED_ENDPOINT_AXIS_COUNT,
    "BQ native-MCS receipt must match Motion axis capacity.");
const double CYCLE_TIME_SEC = 0.00025;// EtherCAT 通訊週期 (250us)
constexpr std::uint32_t MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES = 8U;




//列舉定義--------------------------------------------------------------------
enum class FeedbackSource// 回授訊號來源
{
    MOTOR_ENCODER,  // 半閉迴路：馬達編碼器 (剛性好，適合高速)
    LINEAR_SCALE    // 全閉迴路：外部光學尺 (精度高，消除背隙)
};

enum class MotionState// 運動狀態機
{
    MotionState_IDLE,       // 閒置 (位置鎖定中)
    MotionState_MOVING,     // 移動中 (P2P 定位)
    MotionState_STOPPING,   // 減速停止中
    MotionState_ERROR,     // 警報狀態
    MotionState_VELOCITY,   // 速度模式移動中
    MotionState_INTERPOLATING,//多軸插補中
    MotionState_ESTOP,//緊急狀態

     // =====================================================
    // MPG Handwheel Position Following
    //
    // 放最後面避免改變前面既有 Enum 數值。
    // =====================================================
    MotionState_MPG,
};

enum class BufferMode
{
    BUFFERED, // 排隊模式 (依序執行佇列命令)
    ABORTING  // 覆寫模式 (清空佇列，打斷當前運動，立刻執行)
};




//資料結構-----------------------------------------------------------------------------------------------


struct AxisCommand // 軌跡規劃層算出的「瞬間理論值」，PID 負責追隨這兩個值
{
    double instantCmdPos; // 瞬間理論位置 (Pulse)
    double instantCmdVel; // 瞬間理論速度 (Pulse/Sec) - 用於前饋
};


struct PidConfig// PID 參數與保護設定
{
    // 增益參數
    double Kp = 0.0;           // 比例增益 (剛性)
    double Ki = 0.0;           // 積分增益 (消除靜差)
    double Kd = 0.0;           // 微分增益 (阻尼，通常 CSV 設 0)
    double Kvff = 1.0;              // 🌟 [新增] 速度前饋增益 (預設 1.0 = 100%)
    // 限制保護
    bool EnableLagCheck = true; //是否啟用跟隨誤差(Lag)跳機保護
    double MaxIntegral = 0.0;  // 積分上限 (抗飽和)
    double MaxLag = 0.0;       // 最大允許跟隨誤差 (Lag Limit) -> 超過跳機

    // 內部運算狀態
    double prevError = 0.0;    // 上一次的誤差
    double integralAcc = 0.0;  // 積分累積值
};
// ==========================================
// 🌟 1. 新增：軸型態列舉
// ==========================================
enum class AxisType {
    LINEAR = 0,           // 直線軸 (單位 mm)
    ROTARY = 1,       // 旋轉軸 (單位 Degree，0~360 循環，走最短路徑)
    ROTARY_CONTINUOUS = 2 // 連續旋轉軸 (例如主軸，一直累加不歸零)
};

// K.6.2: RT owns logical command-position updates while NC samples lifecycle
// baselines; NC normally owns the queued endpoint while lifecycle/reset may
// rebase it.  A copyable 64-bit atomic scalar keeps those roles data-race-free
// without changing either existing AxisContext field footprint.
class MotionAtomicDouble
{
public:
    MotionAtomicDouble() noexcept = default;
    MotionAtomicDouble(double value) noexcept
        : m_bits(Encode(value))
    {
    }

    MotionAtomicDouble(const MotionAtomicDouble& other) noexcept
        : m_bits(other.m_bits.load(std::memory_order_acquire))
    {
    }

    MotionAtomicDouble& operator=(
        const MotionAtomicDouble& other) noexcept
    {
        Store(other.Load());
        return *this;
    }

    MotionAtomicDouble& operator=(double value) noexcept
    {
        Store(value);
        return *this;
    }

    operator double() const noexcept
    {
        return Load();
    }

    double Load() const noexcept
    {
        return Decode(m_bits.load(std::memory_order_acquire));
    }

    void Store(double value) noexcept
    {
        m_bits.store(Encode(value), std::memory_order_release);
    }

    // One exact, bounded RMW.  Callers treat contention as an ownership
    // invariant failure; the fixed-period Runtime must never spin here.
    bool TryCompareExchange(double expected, double desired) noexcept
    {
        std::uint64_t expectedBits = Encode(expected);
        return m_bits.compare_exchange_strong(
            expectedBits,
            Encode(desired),
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    }

private:
    static std::uint64_t Encode(double value) noexcept
    {
        std::uint64_t bits = 0ULL;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }

    static double Decode(std::uint64_t bits) noexcept
    {
        double value = 0.0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::atomic<std::uint64_t> m_bits{ 0ULL };
};

static_assert(
    ATOMIC_LLONG_LOCK_FREE == 2,
    "K.6.2 queue-tail rebase requires lock-free 64-bit atomics.");
static_assert(
    sizeof(MotionAtomicDouble) == sizeof(double) &&
    alignof(MotionAtomicDouble) == alignof(double),
    "K.6.2 atomic queue-tail scalar must preserve the double footprint.");


struct AxisContext//軸參數與狀態
{
    //參數-------------------------------------------------------------------------------------------------------------
    bool isExist; // 🌟 [新增] 實體馬達是否存在 / 是否啟用
    int axisIndex = 0;//第幾軸

    bool isHomed = false;

    // G81 HOME 參數、Runtime 與 Machine Coordinate Offset。
    HomeConfig home;
    HomeRuntime homeRuntime;

    // Machine Position = Raw Logical Position - machineCoordinateOffsetPulse
    double machineCoordinateOffsetPulse = 0.0;

    //硬體物理參數-------------------------------------------------
    double resolution_PPR = 16777216.0;// 編碼器解析度
    double maxVel_PPS = 0.0;// 最高轉速 (Pulse/sec)


    double JOG_MAX_PPS = 0.0;
    double JOG_acc_time = 0.2;
    double JOG_dec_time = 0.2;
    double JOG_RAPID_PERCENT = 100.0;


    // =========================================================
// MPG / Manual Pulse Generator
//
// C21 = MPG Mode
//
// C210~217 = Axis Select
//
// C220 = x1
// C221 = x10
// C222 = x100
// C223 = x1000
//
// DR200 = Handwheel accumulated encoder count
//
// MPG_BASE_DISTANCE：
//     每 1 個 Handwheel Count 在 x1 時的移動距離
//
// Linear Axis：mm / count
// Rotary Axis：degree / count
//
// MPG_MAX_PPS：
//     MPG 追趕目標時允許的最高速度
//
// 加減速直接共用：
//     JOG_acc_time
//     JOG_dec_time
// =========================================================

    double MPG_BASE_DISTANCE = 0.001;

    double MPG_MAX_PPS = 0.0;

    // =========================================================
    // FINE CONTINUOUS JOG
    //
    // C23 = Fine JOG Mode
    //
    // 四段速度由各軸自行設定。
    // 單位：Pulse/sec
    //
    // 加減速直接共用：
    // JOG_acc_time
    // JOG_dec_time
    // =========================================================

    double FINE_JOG_0001_PPS = 0.0;
    double FINE_JOG_0010_PPS = 0.0;
    double FINE_JOG_0100_PPS = 0.0;
    double FINE_JOG_1000_PPS = 0.0;



    // =========================================================
// INCH JOG Parameters
//
// 距離單位：
// Linear = mm
// Rotary = degree
// =========================================================

    double INCH_0001_DISTANCE = 0.001;
    double INCH_0010_DISTANCE = 0.010;
    double INCH_0100_DISTANCE = 0.100;
    double INCH_1000_DISTANCE = 1.000;


    // INCH 移動速度
    double INCH_JOG_PPS = 0.0;

    double INCH_acc_time = 0.2;
    double INCH_dec_time = 0.2;

    double G00_PPS = 0.0;// G00 速度 (Pulse/sec)
    double G00_acc_time;  // 🌟 [新增] G00 的加速時間 (秒)
    double G00_dec_time;  // 🌟 [新增] G00 的減速時間 (秒)


    double G07_PPS = 0.0;// 
    double G07_acc_time;  // 
    double G07_dec_time;  // 

    double G161_PPS = 0.0;// 
    double G161_acc_time;  // 
    double G161_dec_time;  //

    double G28_PPS = 0.0;
    double G28_acc_time;
    double G28_dec_time;

    double G30_PPS = 0.0;
    double G30_acc_time;
    double G30_dec_time;

    double G32_PPS = 0.0;
    double G32_acc_time;
    double G32_dec_time;


    double G53_PPS = 0.0;
    double G53_acc_time;
    double G53_dec_time;


    double Stop_dec_time;  //滑行停止減速度 單位(秒)(幾秒內減速完成)

    //雙閉環/全閉環設定-------------------------------------------------
    FeedbackSource fbMode = FeedbackSource::MOTOR_ENCODER;//回授設定 
    double scaleToMotorRatio = 1.0;// 光學尺與馬達的解析度比例
    double maxDeviation = 0.0;// 雙閉環最大容許偏差

    //預設運動參數-------------------------------------------------
    double smoothTime_ms; // S曲線濾波時間 (例如 50ms) -> 決定機台有多「柔」


    //PID-------------------------------------------------
    PidConfig pid;//當下使用參數
    // 🌟 新增：兩組獨立的 PID 參數
    PidConfig Pid_IDLE;//閒置狀態PID
    PidConfig Pid_G00;      // 專屬：定位移動專用 (G00, G01)
    // EDM165: a separate gain bank, sharing the existing servo loop/history.
    PidConfig Pid_FLUSH;
    bool edmFlushGainSelected = false; // RT selector: previous-cycle bank only.




    //狀態與指令-------------------------------------------------------------------------------------------------------------

    //當次運動指令-------------------------------------------------
    double cruiseVel_PPS;// 本次移動的目標巡航速度
    double acc_PPS2 = 0.0;// 本次移動的加速度
    double dec_PPS2 = 0.0;// 本次移動的減速度


    double finalTargetPos = 0.0;// 本次移動的最終目標位置 (定位模式)


    double targetVelocity;// 目標速度 (速度模式)
    double VelocityMove_Acc;// 速度模式專用的切換加速度
    double targetEndVel = 0.0;// 終點速度 (連續軌跡過彎時，預留的不降速值)

    double feedrateOverride = 1.00; // 進給倍率控制 (預設 1.0 = 100%)



    //即時動態座標-------------------------------------------------
    double planningPos;// 虛擬大腦的理想位置 (未經 S-Curve 濾波的粗糙折線)
    double currentCmdPos = 0.0;// 濾波後的最終命令位置 (要餵給 PID 的理論位置)  是給底層 PID 追隨用的（經過加減速濾波的最終點）。
    double currentCmdVel = 0.0;// 濾波後的最終命令速度 (要餵給驅動器的前饋速度)
    double currentActPos = 0.0;// 實際馬達回授的真實位置

    // 🟢 [新增] 大腦專用的邏輯座標與速度 (G68 計算用)
    MotionAtomicDouble logicalCmdPos{ 0.0 };//是給上層 NC 大腦思考用的（它是還原了 G68 空間旋轉、補正後的純邏輯點）。
    double logicalCmdVel = 0.0;

    //機台狀態旗標-------------------------------------------------
    MotionState state = MotionState::MotionState_IDLE;// 當前狀態機 (預設閒置)
    bool inPosition = true;// 是否已到達終點 (剛開機視為已到位)
    bool isFault = false;// 是否發生硬體或軟體警報
    bool resetRequest = false;   // 🌟 [新增] 系統是否正在要求解除警報
    bool isServoOn = false;      //激磁狀態 只有這變成 true，NC 軌跡規劃器才允許下指令 
    int8_t targetMode = 9;       // 預設 CSV 模式 (9)，方便之後想改模式時設定

    //運算記憶體緩衝區-------------------------------------------------

    //S-Curve 移動平均濾波器--------
    std::vector<double> velBuffer;// 儲存歷史速度的環形陣列
    int bufferIndex = 0;// 目前陣列寫入的指標位置
    double bufferSum = 0.0;// 陣列內所有速度的總和 (加速計算用)


    //硬體綁定指標-------------------------------------------------
    int32_t* pScaleActualPos = nullptr; // 光學尺的實體記憶體位址 (未接時必須是 nullptr)


    //單軸專用的變數
    double startCmdPos = 0.0;  // 紀錄這段移動的「起點」
    double motionTime = 0.0;   // 紀錄這段移動「走了幾秒」
    double moveDir = 1.0;      // 移動方向 (1.0 或 -1.0)
    double programmedVel_PPS = 0.0; // 🟢 [新增] 紀錄下單時的原始目標速度

    // ==========================================
    // 🌟 2. 新增：軸型態與物理特性設定
    // ==========================================
    AxisType axisType = AxisType::LINEAR; // 預設為直線軸
    double rotaryModulo = 360.0;          // 旋轉一圈的單位 (預設 360度)
    bool useShortestPath = true;          // 旋轉軸是否走最短路徑 (0=否, 1=是)


    // ==========================================
    // 🌟 補償功能開關與基礎參數
    // ==========================================
    bool enableBacklash = false;     // 是否啟用背隙補償
    double backlashAmount_Pos_mm = 0; // 正向移動時的補償量 (例如 15um)
    double backlashAmount_Neg_mm = 0; // 負向移動時的補償量 (例如 5um)
    double backlashSpeed = 3.0;           // 背隙 補償漸變速度 設為 3.0 (代表每秒慢慢補進 3.0 mm)
    bool enablePitch = false;        // 是否啟用螺距補償
    double pitchStartPos_mm = 0.0;   // 螺距補償起點 (例如從機械座標 0.0 開始)
    double pitchStep_mm = 10.0;      // 每一格的間距 (例如每 10mm 補一格)
    double pitchSpeed_mm_s = 3.0;   // 節距 補償漸變速度 設為 3.0 (代表每秒慢慢補進 3.0 mm)
    // 🌟 [新增] 紀錄當前總共加上了多少補償 (mm/deg)
    double currentCompOffset_unit = 0.0;

    // PBC-2: RT-owned local command frame. NOT proof of PDO delivery. The raw
    // currentActPos remains untouched for servo feedback, HOME and limit checks.
    pbc::CoordinateFrame mechanicalCompensationFrame{};

    double GetNominalActualPositionPulse() const noexcept
    {
        return pbc::NominalFeedbackPulse(enablePitch || enableBacklash,
            currentCompOffset_unit, currentActPos, resolution_PPR, finalLead,
            mechanicalCompensationFrame);
    }
    double GetMechanicalFollowingErrorPulse() const noexcept
    {
        return pbc::FollowingErrorPulse(enablePitch || enableBacklash,
            currentCompOffset_unit, currentCmdPos, currentActPos,
            resolution_PPR, finalLead, mechanicalCompensationFrame);
    }
    bool IsMechanicalCompensationReadyForCompletion() const noexcept
    {
        return pbc::CoordinateCompletionReady(enablePitch || enableBacklash,
            currentCompOffset_unit, currentCmdPos, resolution_PPR, finalLead,
            mechanicalCompensationFrame);
    }

    // --- 1. 馬達/編碼器參數 ---

    bool isReverse = false;             // 方向反轉 (1=反轉, 0=正轉)
    // --- 🌟 新增的顯示參數 ---
    bool Axis_Reverse = false;  // 軸方向 反向

   // --- 2. 🌟 機構齒輪/皮帶比例 (Gear/Pulley Ratio) ---
    // 例如：馬達接 20 齒，負載接 60 齒
    double reduction_MotorSide = 1.0; // 驅動輪齒數/直徑
    double reduction_LoadSide = 1.0;  // 負載輪齒數/直徑

    // --- 3. 傳動元件參數 (Pitch/Circumference) ---
    // 螺桿：填導程 (Lead, 例如 10.0 mm)
    // 皮帶：填 (齒數 * 齒距) (例如 20齒 * 3mm = 60.0 mm)
    // 直驅：填 1.0 (直線) 或 360.0 (旋轉)
    double mechanicalPitch = 5.0;

    // --- 🌟 自動計算出的最終導程 (系統自己算) ---
    // 這一行不需要在設定檔讀取，在 InitAxis 時自動算出
    double finalLead = 10.0;


    // 在 AxisContext 結構中加入這兩個變數
    double inPositionWindow_mm = 0.005; // 預設 5um 到位視窗
    double inPositionWindow_Pulse = 0.0; // 底層實際判斷用的 Pulse

    // 🌟 [新增] 允許的最大跟隨誤差設定
    double maxLag_mm = 2.0;              // 人機設定值 (例如 2.0 mm，超過就 Alarm)

    bool isVirtualAxis = false;  // 🌟 [新增] 預設為一般實體軸

    // 🌟 [新增] 計算實際速度與紀錄 Lag 警報用
    double lastActPos = 0.0;    // 上一個 Cycle 的實體位置
    double currentActVel = 0.0; // 目前計算出的實際速度 (PPS)
    bool   isLagAlarm = false;  // 追隨誤差專屬警報旗標

    // 🌟 [新增] 解決 32-bit 溢位用的座標展開變數
    int32_t lastRawActPos = 0;    // 紀錄上一次的原始 32-bit 數值
    double  unwrappedActPos = 0.0;// 展開後、永遠不會溢位的絕對真實位置 (Pulse)
    bool    isFirstCycle = true;  // 開機第一圈對齊旗標

    // Stage NC-0.2J.6.4: startup feedback alignment is a one-shot boot
    // contract.  Once startupLagMonitorArmed becomes true it is never
    // automatically cleared by Servo-Off, Alarm Reset or a second Link().
    bool startupLagFeedbackReady = false;
    bool startupLagPositionAligned = false;
    bool startupLagMonitorArmed = false;
    bool startupLagPrematureMotionBlocked = false;
    std::uint32_t startupLagStableSampleCount = 0U;


    // 🌟 新增：用來給預讀引擎追蹤的「虛擬最後位置」
    MotionAtomicDouble lastQueuedPulse{ 0.0 };

    int servoOffCounter = 0;



    // 真實硬體極限 +OT / -OT。
    bool hardLimitPositive = false;
    bool hardLimitNegative = false;


    // 第 1 組軟體行程 G22 / G23 再決定目前是否啟用
    bool travelLimit1Enable = false;

    double travelLimit1Positive_unit = 0.0;
    double travelLimit1Negative_unit = 0.0;

    double travelLimit1Positive_Pulse = 0.0;
    double travelLimit1Negative_Pulse = 0.0;

    bool travelLimit1PositiveActive = false;
    bool travelLimit1NegativeActive = false;


    // 第 2 組軟體行程 系統參數決定是否開啟
    bool travelLimit2Enable = false;

    double travelLimit2Positive_unit = 0.0;
    double travelLimit2Negative_unit = 0.0;

    double travelLimit2Positive_Pulse = 0.0;
    double travelLimit2Negative_Pulse = 0.0;

    bool travelLimit2PositiveActive = false;
    bool travelLimit2NegativeActive = false;


    // 第 3 組軟體行程 系統參數決定是否開啟
    bool travelLimit3Enable = false;

    double travelLimit3Positive_unit = 0.0;
    double travelLimit3Negative_unit = 0.0;

    double travelLimit3Positive_Pulse = 0.0;
    double travelLimit3Negative_Pulse = 0.0;

    bool travelLimit3PositiveActive = false;
    bool travelLimit3NegativeActive = false;


    //總極限結果判斷此點是否碰到極限即可--------------------------------------
    bool positiveTravelBlocked = false;
    bool negativeTravelBlocked = false;
};

enum class InterpolationMode//插補群組的導航模式
{
    LINEAR,       // 直線插補
    CIRCULAR_CW,  // 圓弧插補 - 順時針
    CIRCULAR_CCW  // 圓弧插補 - 逆時針
};


enum class PathMode// 軌跡的連續模式 
{
    EXACT_STOP,// 精確停止 (每段終點速度降到 0)
    CONTINUOUS,// 連續軌跡 (速度融合提前預讀下一個速度)
    PATH_SERVO,//  路徑伺服模式：由外部即時給予路徑進給速度
    JUMP_TRACKING//跳躍排渣模式
};

inline bool IsMotionPositioningOperationValid(std::uint8_t operation) noexcept
{
    switch(operation){case 0U:case 1U:case 7U:case 28U:case 30U:case 32U:case 53U:case 161U:return true;default:return false;}
}

struct MotionCommand//運動指令包裹 (使用在塞進佇列)
{
    // Stage NC-0.1B：命令進入 Motion Queue 前正式取得執行識別。
    // Epoch 淘汰舊世代命令；SegmentId 唯一追蹤每一段路徑。
    MotionExecutionIdentity execution{};

    // Stage NC-0.1E：命令建立當下的 Motion 控制權快照。
    // Consumer 會在真正執行前再次比對 Owner + Generation。
    MotionOwnerLease ownerLease{};

    InterpolationMode mode = InterpolationMode::LINEAR;
    int axisCount = 0;
    int axisIndices[MAX_AXES] = { 0 };
    double targetPos[MAX_AXES] = { 0.0 };

    // 圓弧專用參數
    double centerPos[2] = { 0.0, 0.0 };

    // 螺線專用
    double startRadius = 0.0;
    double endRadius = 0.0;

    int dir = 0; // 方向 (1=CCW, -1=CW)
    // BY used the existing dir padding in the original 560-byte layout.
    bool pathCorePlanarCircle = false;
    bool pathCoreFullCircle = false;
    // BZ: canonical geometry uses mem_* only while this fresh traversal is active.
    bool pathCoreRetainedTraversal = false;
    bool pathCoreRetainedReverse = false;

    // 運動參數
    double targetVel = 0.0;
    double accTime = 0.0;
    double decTime = 0.0;

    // 時光機專用快照記憶體
    // True only for an RT replay transport copy whose lifecycle identity was
    // already terminal before B2 pushed it back in front of the path.
    bool replayTerminalAlreadyPublished = false;
    bool cncFeedLookahead = false; // DE uses the existing x64 padding byte at offset 201.
    bool cncCornerBlend = false; // DH compound LINE + ARC; consumes padding at offset 202.
    bool pathCoreFeedExactStop = false; // DT: nonbuffered native G01 provenance, padding byte 203.
    bool pathCoreRotaryFeedExactStop = false; // BASE68: degree feed; existing padding byte 204.
    bool pathCoreZCFeedExactStop = false; // BASE70: common-time Z/C; padding byte 205.
    bool pathCoreXYZCFeedExactStop = false; // BASE75: G90/G91 XYZ/C; padding byte 206.
    bool pathCoreXYZCUVFeedExactStop = false; // BASE73: G91 XYZ/C/U/V; padding byte 207.
    // With this marker only: mem_startPos[0]=native start pulse, mem_ratio[0]=PPD,
    // mem_startPos[1]/mem_ratio[1]=accepted MCS start/end degrees;
    // mem_totalDist=signed resolved degree sweep, mem_radius=F deg/min.
    // BASE69 G90 only: mem_startPos[2]/mem_ratio[2]=authored WCS/MCS target,
    // mem_startPos[3]=frozen modulo, mem_ratio[3]=shortest-path flag (0 or 1).
    // G91 keeps these four fields zero and preserves its signed authored delta.
    // These immutable fields are admission provenance, never replay geometry.
    // BASE70 Z/C only (the rotary marker above is false): compact axes [2,3],
    // targetPos[0,1]=end pulses; mem_startPos[0,1]=start pulses,
    // mem_startPos[2,3]=start MCS, mem_ratio[0,1]=PPU,
    // mem_ratio[2,3]=end MCS; mem_startPos[4]/mem_ratio[4]=authored dZ/dC.
    // mem_totalDist=scalar pulse length, mem_radius=F mm/min along Z,
    // mem_startAngle=nominal common seconds, mem_totalAngle=C deg/min.
    // These scalars are provenance only; LINEAR maps both axes by one lambda.
    // BASE75 XYZC-only provenance (no transform/history permission):
    // mem_startPos[0..3]=start pulses; [4..7]=start MCS XYZC;
    // mem_ratio[0..3]=PPU; [4..7]=G91 deltas or G90 RAW WCS targets.
    // G90 mem_centerX=C modulo, mem_centerY=shortest flag (exact +0 or 1).
    // G91 keeps mem_centerX/Y zero. Existing scalar length/F/time/C rate stay.
    // BASE73 six-axis-only provenance (never a transform/history snapshot):
    // mem_startPos[0..5]=start pulses; mem_ratio[0..5]=PPU; slots6/7 stay zero.
    // mem_transformOrigin=start MCS XYZ; matrix[0]=start MCS CUV,
    // G91 matrix[1]=authored XYZ deltas; matrix[2]=authored CUV deltas.
    // BASE74 G90 uses these six matrix values for RAW programmed WCS targets;
    // mem_startPos[6,7]=frozen C/U modulo; mem_ratio[6]=V modulo,
    // mem_ratio[7]=frozen shortest-path bits C/U/V (exact integer 0..7).
    // G91 retains zero spare slots. Resolve canonical ends without a second wrap.
    // mem_enableTransform is false. mem_radius=XYZ F, mem_startAngle=time,
    // mem_totalDist=pulse length; mem_totalAngle/mem_centerX/mem_centerY=CUV rates.
    double mem_startPos[MAX_AXES] = { 0.0 };
    double mem_ratio[MAX_AXES] = { 0.0 };
    double mem_radius = 0.0;
    double mem_startAngle = 0.0;
    double mem_centerX = 0.0;
    double mem_centerY = 0.0;
    double mem_totalDist = 0.0;
    double mem_totalAngle = 0.0;

    // 時光機專用：記憶當時的空間旋轉狀態
    bool mem_enableTransform = false;
    bool pathCoreEccentricCFeedExactStop = false; // BASE79B: existing padding byte 385; live admission closed.
    bool edm149ZOnlyCommissioning = false; // EDM149: padding byte 386, bound dry commissioning source only.
    bool edm150XYZOnlyCommissioning = false; // EDM150: padding byte 387, fixed C/U/V references.
    bool edm154SparseCommissioning = false; // EDM154: padding byte 388, bound sparse dry commissioning geometry.
    bool edm173G76Gap = false; // Explicit sealed common-GAP compound, existing padding389.
    std::uint8_t edm173ChildOrdinal = 0U, edm173ChildCount = 0U; // padding390/391; no output authority.
    double mem_transformOrigin[3] = { 0.0, 0.0, 0.0 };
    double mem_transformMatrix[3][3] = {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    // 這張單子是從哪一行 G-Code 產生。
    // 既有相容欄位；入列時同步鏡射到 execution.sourceBlockId。
    int sourceLinePC = 0;

    // 打包當下的座標與 Modal 狀態
    int sourceWCS = 54;
    int sourceToolLengthMode = 49;
    int sourceHCode = 0;
    int sourceToolRadiusMode = 40;
    int sourceDCode = 0;
    bool sourceIsAbsoluteMode = true;

    bool sourceG68Active = false;
    bool g80EdgeFind = false; // G80 parent marker in existing x64 padding.
    std::uint8_t positioningOperation=0U; // EDM280 immutable G-code provenance, existing padding.
    bool g76SphereAir=false; // EDM286 sealed P50 AIR parent; existing source-G68 padding.
    double sourceG68Angle = 0.0;

    bool sourceG168Active = false;
    int sourceWCode = 0;

    bool sourceG51Active = false;
    double sourceScaleRatio = 1.0;

    std::uint8_t sourceMirrorMask = 0U;
    bool sourceG16Active = false;
    bool sourceG162Active = true;

    // Stage NC-0.2K.6 / K.6.1: command-local G00 terminal policy.
    // K.6 transports this byte; K.6.1 makes the authorized 250 us Consumer
    // the only normal-G00 writer of the effective planner PathMode.
    MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED;
    int sourcePlaneMode = 17;

    // DK: immutable authored Fi scalar cap for a Q straight prefix only.
    // Zero retains the legacy DJ packet cap. Arc/seam dynamics use targetVel.
    double cncPrefixVelocityPPS = 0.0;

    // Immutable fixed-translation source; all prior transport offsets stay unchanged.
    NCTranslationSnapshot sourceTranslation{};
};

// Generic packet families must reject the P50 parent before a typed bool
// read; malformed marker bytes are rejected here as well as at the RT gate.
inline bool IsMotionG76SphereMarkerClear(const MotionCommand& command) noexcept
{ return reinterpret_cast<const unsigned char*>(&command)[offsetof(MotionCommand,g76SphereAir)]==0U; }

inline bool IsMotionFixedTranslationToolSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    // An empty legacy descriptor never authorizes an active H vector.
    if (IsNCTranslationSnapshotEmpty(command.sourceTranslation))
        return command.sourceToolLengthMode == 49;
    return IsNCTranslationToolModeAllowed(command.sourceToolLengthMode, command.sourceTranslation) &&
        command.sourceHCode == command.sourceTranslation.toolHCode &&
        (command.sourceToolLengthMode == 49 || !command.sourceG162Active);
}

// Fixed G68 / plane-preserving WORK rotation is baked into native geometry.
// G17 yaw retains its queued scope; G18 pitch / G19 roll remain EXACT_STOP in
// BASE-PLANE-7. Retained/reverse paths stay excluded.
inline bool IsMotionFixedRotationPathAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    if (command.commandPathMode == MotionCommandPathMode::EXACT_STOP)
        return !command.cncFeedLookahead && !command.cncCornerBlend;
    const bool plainXYLine = command.mode == InterpolationMode::LINEAR &&
        !command.cncCornerBlend && !command.pathCorePlanarCircle && !command.pathCoreFullCircle && command.dir == 0 &&
        command.startRadius == 0.0 && command.endRadius == 0.0 &&
        command.mem_radius == 0.0 && command.mem_totalAngle == 0.0;
    const bool planarCircle = !command.cncCornerBlend && command.pathCorePlanarCircle &&
        ((command.mode == InterpolationMode::CIRCULAR_CW && command.dir == -1) ||
            (command.mode == InterpolationMode::CIRCULAR_CCW && command.dir == 1));
    // This is a source-shape gate. The consumer separately proves the complete
    // canonical prefix/fillet geometry, equal XY pulse scale and speed bounds.
    const bool cornerBlend = command.cncCornerBlend && command.mode == InterpolationMode::LINEAR &&
        !command.pathCorePlanarCircle && !command.pathCoreFullCircle &&
        (command.dir == -1 || command.dir == 1) && std::isfinite(command.mem_radius) &&
        command.mem_radius > 0.0 && command.startRadius == command.mem_radius &&
        command.endRadius == command.mem_radius;
    return command.commandPathMode == MotionCommandPathMode::CONTINUOUS &&
        command.cncFeedLookahead && (plainXYLine || planarCircle || cornerBlend) &&
        !command.pathCoreFeedExactStop &&
        !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        command.execution.source == MotionCommandSource::NC_MEMORY &&
        command.ownerLease.owner == MotionOwner::AUTO && !command.sourceG162Active &&
        command.sourceIsAbsoluteMode && command.sourceTranslation.distanceMode == 90 &&
        command.sourcePlaneMode == 17 && command.axisCount == 2 &&
        command.axisIndices[0] == 0 && command.axisIndices[1] == 1;
}

inline bool IsMotionFixedTranslationWorkSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    // Physical G168/W tags require the complete frozen WORK row and MCS center.
    // Empty legacy descriptors cannot authorize active WORK, even with G49.
    if (IsNCTranslationSnapshotEmpty(command.sourceTranslation))
        return !command.sourceG168Active &&
            (command.execution.source != MotionCommandSource::NC_MEMORY || command.sourceWCode == 0);
    return IsNCTranslationWorkModeAllowed(command.sourceG168Active, command.sourceWCode,
        command.sourceTranslation) && (!command.sourceG168Active || !command.sourceG162Active) &&
        (!NCTranslationHasWorkPlaneRotation(command.sourceTranslation) ||
            IsMotionFixedRotationPathAllowed(command));
}

inline bool IsMotionFixedTranslationRotationSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    // A G68 tag never authorizes an unfrozen MEMORY transform. MDI keeps its
    // legacy lane; native fixed geometry carries the exact source descriptor.
    if (IsNCTranslationSnapshotEmpty(command.sourceTranslation))
        return !command.sourceG68Active;
    return IsNCTranslationRotationModeAllowed(command.sourceG68Active,
        command.sourceG68Angle, command.sourcePlaneMode, command.sourceTranslation) &&
        (!command.sourceG68Active || (!command.sourceG162Active &&
            IsMotionFixedRotationPathAllowed(command)));
}

inline bool IsMotionFixedTranslationScaleMirrorSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    // Native packet geometry already includes the complete fixed transform.
    // Tags prove provenance only; RT must never apply scale or mirror again.
    const NCTranslationSnapshot& s = command.sourceTranslation;
    if (IsNCTranslationSnapshotEmpty(s))
        return !command.sourceG51Active && command.sourceMirrorMask == 0U;
    return IsNCTranslationSnapshotValid(s) &&
        command.sourceG51Active == (s.scalingMode == 51) &&
        std::memcmp(&command.sourceScaleRatio, &s.scalingFactor, sizeof(double)) == 0 &&
        static_cast<std::uint32_t>(command.sourceMirrorMask) == s.mirrorMask &&
        (!NCTranslationHasScaleMirror(s) || !command.sourceG162Active);
}

inline bool IsMotionFixedTranslationPolarSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    // G16 has already become native Cartesian geometry at the NC boundary.
    // RT checks only immutable provenance; it never interprets radius or angle.
    const NCTranslationSnapshot& s = command.sourceTranslation;
    if (IsNCTranslationSnapshotEmpty(s)) return !command.sourceG16Active;
    if (!IsNCTranslationPolarModeAllowed(command.sourceG16Active, s)) return false;
    if (!command.sourceG16Active) return true;
    return command.execution.source == MotionCommandSource::NC_MEMORY &&
        command.ownerLease.owner == MotionOwner::AUTO &&
        command.sourceIsAbsoluteMode && s.distanceMode == 90 &&
        IsNCArcPlaneCode(command.sourcePlaneMode) &&
        command.sourcePlaneMode == s.rotationPlane && !command.sourceG162Active &&
        !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        ((command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
            !command.cncFeedLookahead && !command.cncCornerBlend) ||
         (command.sourcePlaneMode == 17 &&
            command.commandPathMode == MotionCommandPathMode::CONTINUOUS &&
            command.cncFeedLookahead));
}

inline bool IsMotionFixedTranslationCutterSourceAllowed(const MotionCommand& command) noexcept
{
    if (command.pathCoreEccentricCFeedExactStop) return false;
    const NCTranslationSnapshot& s = command.sourceTranslation;
    if (IsNCTranslationSnapshotEmpty(s))
        return command.sourceToolRadiusMode == 40 && command.sourceDCode == 0;
    if (!IsNCTranslationSnapshotValid(s) || command.sourceToolRadiusMode != s.cutterMode ||
        command.sourceDCode != s.cutterD) return false;
    if (s.cutterMode == 40) return true;
    // NC has already offset the physical tool-centre geometry. Lines use
    // ascending physical XYZ; prepared circles use canonical u/v. BASE-PLANE-10
    // admits explicit closed cutter circles in G18/G19; BASE-PLANE-11 also
    // carries the frozen G91 identity, never reinterpreting native endpoints. It
    // proves bit-exact physical seam, full sweep and frozen start geometry.
    // A frozen frame alone grants neither replay nor blending.
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    const bool line = command.mode == InterpolationMode::LINEAR &&
        command.pathCoreFeedExactStop && !command.pathCorePlanarCircle && !command.pathCoreFullCircle;
    const bool arc = command.pathCorePlanarCircle && !command.pathCoreFeedExactStop &&
        IsNCTranslationCutterArcNotationAllowed(s.rotationPlane, s.distanceMode,
            s.polarMode, command.pathCoreFullCircle) &&
        ((command.mode == InterpolationMode::CIRCULAR_CW && command.dir == -1) ||
            (command.mode == InterpolationMode::CIRCULAR_CCW && command.dir == 1));
    return command.execution.source == MotionCommandSource::NC_MEMORY &&
        command.ownerLease.owner == MotionOwner::AUTO &&
        (line || arc) && command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !command.cncFeedLookahead && !command.cncCornerBlend &&
        !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        IsNCTranslationCutterDistanceModeAllowed(s.rotationPlane, s.distanceMode) &&
        command.sourceIsAbsoluteMode == (s.distanceMode == 90) && command.sourcePlaneMode == s.rotationPlane &&
        command.sourceG16Active == (s.polarMode == 16) && !command.sourceG162Active &&
        // BASE-PLANE-15: G16 also grants NC-proved IJK seam circles here.
        // Rapid, replay and incremental polar remain closed. RT checks native
        // circle geometry/start/end bits, never authored angles or turn counts.
        IsNCTranslationCutterNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode) &&
        ((line && IsNCPlaneLinearPairMapping(s.rotationPlane, command.axisCount, command.axisIndices)) ||
            (arc && command.axisCount == 2 &&
                command.axisIndices[0] == static_cast<int>(plane.u) &&
                command.axisIndices[1] == static_cast<int>(plane.v)));
}

// Canonical circle coordinates are packet slots 0/1; physical axes depend
// on the frozen plane. Keep the RT endpoint/drain proof tied to both maps.
inline bool IsMotionArcPlaneGroupMapping(const MotionCommand& command,
    int groupAxisCount, const int* groupAxes) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (command.pathCoreEccentricCFeedExactStop) return false;
    NCArcPlaneAxes plane{};
    return groupAxes != nullptr && TryGetNCArcPlaneAxes(command.sourcePlaneMode, plane) &&
        command.axisCount == 2 && groupAxisCount == 2 &&
        command.axisIndices[0] == static_cast<int>(plane.u) &&
        command.axisIndices[1] == static_cast<int>(plane.v) &&
        groupAxes[0] == command.axisIndices[0] && groupAxes[1] == command.axisIndices[1];
}

// BASE-PLANE-2: non-XY source authorizes one neutral stopped native XYZ
// line or plane-mapped circle. Keep this existing helper name for callers.
// Retained replay, queued blending, helix and legacy callers stay G17;
// the cutter source gate independently checks literal-contour payload shape.
inline bool IsMotionBaseArcPlaneSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (command.pathCoreEccentricCFeedExactStop) return false;
    if (command.sourcePlaneMode == 17) return true;
    NCArcPlaneAxes plane{};
    return TryGetNCArcPlaneAxes(command.sourcePlaneMode, plane) &&
        IsNCTranslationSnapshotValid(command.sourceTranslation) &&
        IsNCTranslationBaseArcPlaneFrame(command.sourceTranslation) &&
        command.sourcePlaneMode == command.sourceTranslation.rotationPlane &&
        command.execution.source == MotionCommandSource::NC_MEMORY &&
        command.ownerLease.owner == MotionOwner::AUTO &&
        command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !command.cncFeedLookahead && !command.cncCornerBlend &&
        !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        ((command.pathCorePlanarCircle && !command.pathCoreFeedExactStop &&
            command.axisCount == 2 && command.axisIndices[0] == static_cast<int>(plane.u) &&
            command.axisIndices[1] == static_cast<int>(plane.v) &&
            ((command.mode == InterpolationMode::CIRCULAR_CW && command.dir == -1) ||
                (command.mode == InterpolationMode::CIRCULAR_CCW && command.dir == 1))) ||
         (command.mode == InterpolationMode::LINEAR && !command.pathCorePlanarCircle &&
            !command.pathCoreFullCircle && IsNCNativeXYZLinearMapping(command.axisCount, command.axisIndices) &&
            command.dir == 0 && command.startRadius == 0.0 && command.endRadius == 0.0 &&
            command.cncPrefixVelocityPPS == 0.0));
}

// BASE69: one G90/G91 positional rotary axis with a frozen neutral source.
// This marker never grants rapid, mixed-axis, lookahead, replay or EDM authority.
inline bool IsMotionRotaryFeedSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (command.pathCoreEccentricCFeedExactStop) return false;
    const NCTranslationSnapshot& source = command.sourceTranslation;
    if (!command.pathCoreRotaryFeedExactStop || command.pathCoreFeedExactStop || command.pathCoreZCFeedExactStop ||
        command.pathCoreXYZCFeedExactStop || command.pathCoreXYZCUVFeedExactStop ||
        command.mode != InterpolationMode::LINEAR || command.axisCount != 1 ||
        command.axisIndices[0] < 3 || command.axisIndices[0] >= MAX_AXES ||
        command.commandPathMode != MotionCommandPathMode::EXACT_STOP ||
        command.cncFeedLookahead || command.cncCornerBlend || command.pathCorePlanarCircle ||
        command.pathCoreFullCircle || command.pathCoreRetainedTraversal || command.pathCoreRetainedReverse ||
        command.replayTerminalAlreadyPublished || command.mem_enableTransform ||
        command.execution.source != MotionCommandSource::NC_MEMORY || command.ownerLease.owner != MotionOwner::AUTO ||
        !IsNCRotaryFeedNeutralFrame(source) ||
        (source.distanceMode != 90 && source.distanceMode != 91) ||
        command.sourceIsAbsoluteMode != (source.distanceMode == 90) || command.sourcePlaneMode != 17 ||
        command.sourceG162Active || command.sourceG168Active || command.sourceWCode != 0 ||
        command.sourceG68Active || command.sourceG68Angle != 0.0 ||
        command.sourceG51Active || command.sourceScaleRatio != 1.0 || command.sourceMirrorMask != 0U ||
        command.sourceG16Active || command.sourceToolLengthMode != 49 || command.sourceHCode != 0 ||
        command.sourceToolRadiusMode != 40 || command.sourceDCode != 0 ||
        !IsNCTranslationSourceAllowed(command.sourceWCS, source)) return false;
    const unsigned axis = static_cast<unsigned>(command.axisIndices[0]);
    return source.axisIdentity.exists[axis] == 1U && source.axisIdentity.axisType[axis] == 1U &&
        source.axisIdentity.nativeUnit[axis] == 2U;
}

// BASE71 admission is limited to one explicit G90/G91 Z/C line.
// Z remains linear millimetres and C positional rotary degrees. The scalar
// pulse path transports common progress, never a mixed physical feed metric.
inline bool IsMotionZCFeedSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (command.pathCoreEccentricCFeedExactStop) return false;
    const NCTranslationSnapshot& source = command.sourceTranslation;
    return command.pathCoreZCFeedExactStop && !command.pathCoreXYZCFeedExactStop &&
        !command.pathCoreXYZCUVFeedExactStop && !command.pathCoreFeedExactStop &&
        !command.pathCoreRotaryFeedExactStop && command.mode == InterpolationMode::LINEAR &&
        command.axisCount == 2 && command.axisIndices[0] == 2 && command.axisIndices[1] == 3 &&
        command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !command.cncFeedLookahead && !command.cncCornerBlend && !command.pathCorePlanarCircle &&
        !command.pathCoreFullCircle && !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        command.execution.source == MotionCommandSource::NC_MEMORY && command.ownerLease.owner == MotionOwner::AUTO &&
        IsNCZCFeedNeutralFrame(source) &&
        command.sourceIsAbsoluteMode == (source.distanceMode == 90) && command.sourcePlaneMode == 17 &&
        !command.sourceG162Active && !command.sourceG168Active && command.sourceWCode == 0 &&
        !command.sourceG68Active && command.sourceG68Angle == 0.0 &&
        !command.sourceG51Active && command.sourceScaleRatio == 1.0 && command.sourceMirrorMask == 0U &&
        !command.sourceG16Active && command.sourceToolLengthMode == 49 && command.sourceHCode == 0 &&
        command.sourceToolRadiusMode == 40 && command.sourceDCode == 0 &&
        IsNCTranslationSourceAllowed(command.sourceWCS, source);
}

// BASE75 exact G90/G91 XYZ/C mapping; physical units remain separate.
inline bool IsMotionXYZCFeedSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (command.pathCoreEccentricCFeedExactStop) return false;
    const NCTranslationSnapshot& source = command.sourceTranslation;
    return command.pathCoreXYZCFeedExactStop && !command.pathCoreXYZCUVFeedExactStop &&
        !command.pathCoreZCFeedExactStop && !command.pathCoreFeedExactStop &&
        !command.pathCoreRotaryFeedExactStop && command.mode == InterpolationMode::LINEAR &&
        command.axisCount == 4 && command.axisIndices[0] == 0 && command.axisIndices[1] == 1 &&
        command.axisIndices[2] == 2 && command.axisIndices[3] == 3 &&
        command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !command.cncFeedLookahead && !command.cncCornerBlend && !command.pathCorePlanarCircle &&
        !command.pathCoreFullCircle && !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        command.execution.source == MotionCommandSource::NC_MEMORY && command.ownerLease.owner == MotionOwner::AUTO &&
        IsNCXYZCFeedNeutralFrame(source) &&
        command.sourceIsAbsoluteMode == (source.distanceMode == 90) &&
        (source.distanceMode == 90 || source.distanceMode == 91) && command.sourcePlaneMode == 17 &&
        !command.sourceG162Active && !command.sourceG168Active && command.sourceWCode == 0 &&
        !command.sourceG68Active && command.sourceG68Angle == 0.0 &&
        !command.sourceG51Active && command.sourceScaleRatio == 1.0 && command.sourceMirrorMask == 0U &&
        !command.sourceG16Active && command.sourceToolLengthMode == 49 && command.sourceHCode == 0 &&
        command.sourceToolRadiusMode == 40 && command.sourceDCode == 0 &&
        IsNCTranslationSourceAllowed(command.sourceWCS, source);
}

inline double MotionXYZCStartMCS(const MotionCommand& command, unsigned slot) noexcept
{
    return command.mem_startPos[slot + 4U];
}
// G91 signed increment; G90 raw authored WCS coordinate.
inline double MotionXYZCProgrammedValue(const MotionCommand& command, unsigned slot) noexcept
{
    return command.mem_ratio[slot + 4U];
}
inline double MotionXYZCRequestedMCS(const MotionCommand& command, unsigned slot) noexcept
{
    return MotionXYZCProgrammedValue(command, slot) +
        NCTranslationAxisOffsetMM(command.sourceTranslation, slot);
}
inline bool TryResolveMotionXYZCAbsoluteTarget(const MotionCommand& command,
    NCXYZCAbsoluteFeedTarget& resolved) noexcept
{
    resolved = NCXYZCAbsoluteFeedTarget{};
    // Check represented +0/1 directly; no conversion of arbitrary floating data.
    if (!command.sourceIsAbsoluteMode || !IsMotionXYZCFeedSourceAllowed(command) ||
        (!NCRotaryFeedDetail::SameBits(command.mem_centerY, 0.0) &&
            !NCRotaryFeedDetail::SameBits(command.mem_centerY, 1.0))) return false;
    std::array<double, 4U> startMCS{}, startPulse{}, requestedMCS{}, pulsePerUnit{};
    for (unsigned axis = 0U; axis < 4U; ++axis)
    {
        startMCS[axis] = MotionXYZCStartMCS(command, axis);
        startPulse[axis] = command.mem_startPos[axis];
        requestedMCS[axis] = MotionXYZCRequestedMCS(command, axis);
        pulsePerUnit[axis] = command.mem_ratio[axis];
        if (!std::isfinite(MotionXYZCProgrammedValue(command, axis)) ||
            !std::isfinite(requestedMCS[axis])) return false;
    }
    return TryResolveNCXYZCAbsoluteFeedTarget(startMCS, startPulse, requestedMCS,
        pulsePerUnit, command.mem_centerY == 1.0, command.mem_centerX, resolved) && resolved.valid;
}


// Check raw bytes before any commissioning bool load. Invalid transport bytes
// cannot invoke undefined behavior through a typed bool read or borrow a scope.
inline bool IsMotionEDMCommissioningMarkerEncodingValid(const MotionCommand& command) noexcept
{
    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&command);
    unsigned selected = 0U;
    const std::size_t offsets[5U] = { offsetof(MotionCommand, edm149ZOnlyCommissioning),
        offsetof(MotionCommand, edm150XYZOnlyCommissioning), offsetof(MotionCommand, edm154SparseCommissioning),
        offsetof(MotionCommand, edm173G76Gap), offsetof(MotionCommand,g76SphereAir) };
    for (unsigned index = 0U; index < 5U; ++index)
    {
        const unsigned char value = bytes[offsets[index]];
        if (value > 1U) return false;
        selected += value;
    }
    if(selected>1U)return false;
    if(bytes[offsetof(MotionCommand,g76SphereAir)]!=0U)
    {
        // P50 is its own immutable parent, never a relabelled legacy packet.
        // Source/geometry/current authority are checked by its dedicated lane.
        const std::size_t incompatible[] = { offsetof(MotionCommand,g80EdgeFind),
            offsetof(MotionCommand,pathCoreEccentricCFeedExactStop),
            offsetof(MotionCommand,pathCoreFeedExactStop), offsetof(MotionCommand,pathCoreRotaryFeedExactStop),
            offsetof(MotionCommand,pathCoreZCFeedExactStop), offsetof(MotionCommand,pathCoreXYZCFeedExactStop),
            offsetof(MotionCommand,pathCoreXYZCUVFeedExactStop), offsetof(MotionCommand,pathCorePlanarCircle),
            offsetof(MotionCommand,pathCoreFullCircle), offsetof(MotionCommand,pathCoreRetainedTraversal),
            offsetof(MotionCommand,pathCoreRetainedReverse), offsetof(MotionCommand,replayTerminalAlreadyPublished),
            offsetof(MotionCommand,cncFeedLookahead), offsetof(MotionCommand,cncCornerBlend),
            offsetof(MotionCommand,mem_enableTransform) };
        for(std::size_t offset:incompatible)if(bytes[offset]!=0U)return false;
        if(command.positioningOperation!=0U)return false;
    }
    return bytes[offsetof(MotionCommand, edm173G76Gap)] ?
        (command.edm173ChildCount==25U || command.edm173ChildCount==26U) &&
        command.edm173ChildOrdinal<command.edm173ChildCount :
        command.edm173ChildCount==0U && command.edm173ChildOrdinal==0U;
}

// BASE74 exact G90/G91 XYZ/C/U/V mapping; physical units remain separate.
inline bool IsMotionXYZCUVFeedSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (!IsMotionEDMCommissioningMarkerEncodingValid(command) || command.pathCoreEccentricCFeedExactStop) return false;
    const NCTranslationSnapshot& source = command.sourceTranslation;
    return command.pathCoreXYZCUVFeedExactStop && !command.pathCoreXYZCFeedExactStop &&
        !command.pathCoreZCFeedExactStop && !command.pathCoreFeedExactStop &&
        !command.pathCoreRotaryFeedExactStop && command.mode == InterpolationMode::LINEAR &&
        command.axisCount == 6 && command.axisIndices[0] == 0 && command.axisIndices[1] == 1 &&
        command.axisIndices[2] == 2 && command.axisIndices[3] == 3 &&
        command.axisIndices[4] == 4 && command.axisIndices[5] == 5 &&
        command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
        !command.cncFeedLookahead && !command.cncCornerBlend && !command.pathCorePlanarCircle &&
        !command.pathCoreFullCircle && !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse &&
        !command.replayTerminalAlreadyPublished && !command.mem_enableTransform &&
        command.execution.source == MotionCommandSource::NC_MEMORY && command.ownerLease.owner == MotionOwner::AUTO &&
        IsNCXYZCUVFeedNeutralFrame(source) &&
        command.sourceIsAbsoluteMode == (source.distanceMode == 90) &&
        (source.distanceMode == 90 || source.distanceMode == 91) && command.sourcePlaneMode == 17 &&
        !command.sourceG162Active && !command.sourceG168Active && command.sourceWCode == 0 &&
        !command.sourceG68Active && command.sourceG68Angle == 0.0 &&
        !command.sourceG51Active && command.sourceScaleRatio == 1.0 && command.sourceMirrorMask == 0U &&
        !command.sourceG16Active && command.sourceToolLengthMode == 49 && command.sourceHCode == 0 &&
        command.sourceToolRadiusMode == 40 && command.sourceDCode == 0 &&
        IsNCTranslationSourceAllowed(command.sourceWCS, source);
}

inline double MotionXYZCUVStartMCS(const MotionCommand& command, unsigned slot) noexcept
{
    return slot < 3U ? command.mem_transformOrigin[slot] : command.mem_transformMatrix[0][slot - 3U];
}

// G91: signed native increment. G90: raw authored WCS coordinate.
inline double MotionXYZCUVProgrammedValue(const MotionCommand& command, unsigned slot) noexcept
{
    return command.mem_transformMatrix[slot < 3U ? 1U : 2U][slot % 3U];
}

inline double MotionXYZCUVRequestedMCS(const MotionCommand& command, unsigned slot) noexcept
{
    return MotionXYZCUVProgrammedValue(command, slot) +
        NCTranslationAxisOffsetMM(command.sourceTranslation, slot);
}

inline double MotionXYZCUVRotaryModulo(const MotionCommand& command, unsigned rotary) noexcept
{
    return rotary < 2U ? command.mem_startPos[6U + rotary] : command.mem_ratio[6U];
}

inline bool TryResolveMotionXYZCUVAbsoluteTarget(const MotionCommand& command,
    NCXYZCUVAbsoluteFeedTarget& resolved) noexcept
{
    resolved = NCXYZCUVAbsoluteFeedTarget{};
    const double maskValue = command.mem_ratio[7U];
    if (!command.sourceIsAbsoluteMode || command.sourceTranslation.distanceMode != 90 ||
        !IsMotionXYZCUVFeedSourceAllowed(command) ||
        !std::isfinite(maskValue) || maskValue < 0.0 || maskValue > 7.0 ||
        maskValue != std::floor(maskValue)) return false;
    // Validate before floating-to-integer conversion: malformed packets must
    // never invoke undefined behavior for NaN, infinity or out-of-range masks.
    const unsigned mask = static_cast<unsigned>(maskValue);
    if (!NCRotaryFeedDetail::SameBits(maskValue, static_cast<double>(mask))) return false;
    std::array<double, 6U> startMCS{}, startPulse{}, requestedMCS{}, pulsePerUnit{};
    std::array<double, 3U> modulo{};
    std::array<bool, 3U> shortest{};
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        startMCS[axis] = MotionXYZCUVStartMCS(command, axis);
        startPulse[axis] = command.mem_startPos[axis];
        requestedMCS[axis] = MotionXYZCUVRequestedMCS(command, axis);
        pulsePerUnit[axis] = command.mem_ratio[axis];
        if (!std::isfinite(MotionXYZCUVProgrammedValue(command, axis)) ||
            !std::isfinite(requestedMCS[axis])) return false;
    }
    for (unsigned rotary = 0U; rotary < 3U; ++rotary)
    {
        modulo[rotary] = MotionXYZCUVRotaryModulo(command, rotary);
        shortest[rotary] = (mask & (1U << rotary)) != 0U;
    }
    return TryResolveNCXYZCUVAbsoluteFeedTarget(startMCS, startPulse, requestedMCS,
        pulsePerUnit, shortest, modulo, resolved) && resolved.valid;
}

inline double MotionXYZCUVDeltaNative(const MotionCommand& command, unsigned slot) noexcept
{
    if (!command.sourceIsAbsoluteMode) return MotionXYZCUVProgrammedValue(command, slot);
    NCXYZCUVAbsoluteFeedTarget resolved{};
    return TryResolveMotionXYZCUVAbsoluteTarget(command, resolved) ?
        resolved.deltaNative[slot] : (std::numeric_limits<double>::quiet_NaN)();
}

inline double MotionXYZCUVEndMCS(const MotionCommand& command, unsigned slot) noexcept
{
    if (!command.sourceIsAbsoluteMode)
        return MotionXYZCUVStartMCS(command, slot) + MotionXYZCUVProgrammedValue(command, slot);
    NCXYZCUVAbsoluteFeedTarget resolved{};
    return TryResolveMotionXYZCUVAbsoluteTarget(command, resolved) ?
        resolved.endMCS[slot] : (std::numeric_limits<double>::quiet_NaN)();
}

// EDM171: an explicit finite native LINE/ARC compound may retain its
// frozen G90/G91, scale and mirror source. Marker 0/1 keep their old gates.
// EDM264 uses the disabled transform matrix as immutable source provenance
// only for complete P2 B0 AIR batches carrying an active fixed W frame.
// EDM273 adds the vector shapes without canonicalizing their authored P.
// Geometry, publication identity and the current frame remain separate gates.
inline bool IsMotionG76ExpansionAirWorkSource264(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    const auto& m=command.mem_transformMatrix;
    const double mode=m[0U][2U],ordinal=m[1U][1U];
    if(m[0U][0U]!=264.0 || m[0U][1U]!=1.0 ||
        !(mode==21.0 || mode==22.0 || mode==23.0 || mode==24.0 || mode==25.0 || mode==26.0 || mode==27.0) ||
        m[1U][0U]!=26.0 || !std::isfinite(ordinal) || ordinal<1.0 || ordinal>26.0 ||
        m[2U][0U]!=2640000.0+mode*100.0+ordinal ||
        std::floor(ordinal)!=ordinal || !command.pathCoreRetainedTraversal ||
        command.pathCoreRetainedReverse || command.pathCorePlanarCircle || command.pathCoreFullCircle ||
        command.edm173G76Gap || command.mem_enableTransform || command.mode!=InterpolationMode::LINEAR ||
        command.dir!=0 || command.axisCount!=3 || command.axisIndices[0U]!=0 ||
        command.axisIndices[1U]!=1 || command.axisIndices[2U]!=2 ||
        command.mem_transformOrigin[0U]!=0.0 || command.mem_transformOrigin[1U]!=1.0 ||
        command.mem_transformOrigin[2U]!=2.0 || !command.sourceG168Active ||
        command.sourceWCode<1 || command.sourceWCode>100) return false;
    const double zeroFields[]={m[1U][2U],m[2U][1U],m[2U][2U]};
    for(double value:zeroFields) if(value!=0.0 || std::signbit(value))return false;
    return true;
}

// EDM265: P0 uses a distinct source marker. These immutable AIR tags never
// grant GAP authority or broaden the older P2 marker's mode set.
inline bool IsMotionG76FixedRadiusAirWorkSource265(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    const auto& m=command.mem_transformMatrix;
    const double mode=m[0U][2U],ordinal=m[1U][1U];
    if(m[0U][0U]!=265.0 || m[0U][1U]!=1.0 ||
        !(mode==1.0 || mode==2.0 || mode==3.0 || mode==4.0 || mode==5.0 || mode==6.0 || mode==7.0) ||
        m[1U][0U]!=26.0 || !std::isfinite(ordinal) || ordinal<1.0 || ordinal>26.0 ||
        m[2U][0U]!=2650000.0+mode*100.0+ordinal ||
        std::floor(ordinal)!=ordinal || !command.pathCoreRetainedTraversal ||
        command.pathCoreRetainedReverse || command.pathCorePlanarCircle || command.pathCoreFullCircle ||
        command.edm173G76Gap || command.mem_enableTransform || command.mode!=InterpolationMode::LINEAR ||
        command.dir!=0 || command.axisCount!=3 || command.axisIndices[0U]!=0 ||
        command.axisIndices[1U]!=1 || command.axisIndices[2U]!=2 ||
        command.mem_transformOrigin[0U]!=0.0 || command.mem_transformOrigin[1U]!=1.0 ||
        command.mem_transformOrigin[2U]!=2.0 || !command.sourceG168Active ||
        command.sourceWCode<1 || command.sourceWCode>100) return false;
    const double zeroFields[]={m[1U][2U],m[2U][1U],m[2U][2U]};
    for(double value:zeroFields) if(value!=0.0 || std::signbit(value))return false;
    return true;
}

// EDM269: a separate immutable provenance tag for a complete ordinary B2
// compound in a fixed WORK XY frame. The matrix stays disabled: these are
// canonical native intervals, never a request to transform their geometry.
// EDM273 vector tags use version 2 and include the original P in the digest.
// Legacy version 1 excludes vectors and remains byte-for-byte compatible.
inline bool IsMotionG76MixedAirWorkSource269(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    const auto& m=command.mem_transformMatrix;
    const double kind=m[0U][2U],count=m[1U][0U],ordinal=m[1U][1U],phase=m[1U][2U];
    const double mode=m[2U][1U],nominal=m[2U][2U];
    const bool vectorMode=mode==4.0 || mode==5.0 || mode==7.0 ||
        mode==14.0 || mode==15.0 || mode==17.0 || mode==24.0 || mode==25.0 || mode==27.0;
    const bool vectorTag=m[0U][1U]==2.0;
    if(m[0U][0U]!=269.0 || (m[0U][1U]!=1.0 && !vectorTag) ||
        (vectorTag ? (kind!=3.0 || !vectorMode) : vectorMode) ||
        !(kind==1.0 || kind==2.0 || kind==3.0) ||
        !std::isfinite(count) || count<3.0 || count>128.0 || std::floor(count)!=count ||
        !std::isfinite(ordinal) || ordinal<1.0 || ordinal>count || std::floor(ordinal)!=ordinal ||
        !(phase==0.0 || phase==1.0 || phase==2.0) || std::signbit(phase) ||
        m[2U][0U]!=2690000000.0+kind*1000000.0+count*1000.0+ordinal+(vectorTag?mode*10000000.0:0.0) ||
        !command.pathCoreRetainedTraversal || command.mem_transformOrigin[2U]!=2.0 ||
        command.edm173G76Gap || command.mem_enableTransform || !command.sourceG168Active ||
        command.sourceWCode<1 || command.sourceWCode>100) return false;
    double trigger=1.0;
    if(kind==3.0)
    {
        const bool taper=mode==11.0 || mode==12.0 || mode==13.0 || mode==14.0 || mode==15.0 || mode==16.0 || mode==17.0;
        const bool entry=mode==1.0 || mode==2.0 || mode==3.0 || mode==4.0 || mode==5.0 || mode==6.0 || mode==7.0 ||
            mode==21.0 || mode==22.0 || mode==23.0 || mode==24.0 || mode==25.0 || mode==26.0 || mode==27.0;
        if(!(taper || entry) || nominal!=(taper?25.0:26.0)) return false;
        trigger=taper?12.0:13.0;
    }
    else if(mode!=0.0 || std::signbit(mode) || nominal!=1.0) return false;
    const double reverse=(count-nominal)*0.5;
    if(reverse<1.0 || std::floor(reverse)!=reverse) return false;
    const double expectedPhase=ordinal<=trigger || ordinal>trigger+2.0*reverse ? 0.0 :
        ordinal<=trigger+reverse ? 1.0 : 2.0;
    const double begin=command.mem_transformOrigin[0U],end=command.mem_transformOrigin[1U];
    if(phase!=expectedPhase || command.pathCoreRetainedReverse!=(phase==1.0) ||
        !std::isfinite(begin) || !std::isfinite(end) || begin<0.0 || begin>1.0 || end<0.0 || end>1.0 ||
        (phase==1.0 ? begin<=end : begin>=end)) return false;
    const bool arc=command.pathCorePlanarCircle;
    if(phase==0.0 && arc!=(kind==2.0)) return false;
    if(arc ? (command.dir!=1 && command.dir!=-1) ||
            command.mode!=(command.dir==1?InterpolationMode::CIRCULAR_CCW:InterpolationMode::CIRCULAR_CW) :
        command.dir!=0 || command.mode!=InterpolationMode::LINEAR || command.pathCoreFullCircle) return false;
    return true;
}

inline bool IsMotionG76MixedRetainedSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if (!IsMotionEDMCommissioningMarkerEncodingValid(command)) return false;
    // B0 and ordinary B2 fixed WORK compounds use separate immutable tags;
    // an untagged legacy mixed/reverse packet still keeps W disabled.
    const bool orbitAirWork=IsMotionG76ExpansionAirWorkSource264(command) ||
        IsMotionG76FixedRadiusAirWorkSource265(command);
    const bool mixedAirWork=IsMotionG76MixedAirWorkSource269(command);
    if(command.mem_transformMatrix[0U][0U]==269.0 && !mixedAirWork) return false;
    const bool allowFixedWorkTranslation=command.edm173G76Gap || orbitAirWork || mixedAirWork;
    const auto& frame=command.sourceTranslation;
    if (!command.pathCoreRetainedTraversal || command.mem_transformOrigin[2]!=2.0 ||
        !command.execution.IsAssigned() || command.execution.source!=MotionCommandSource::NC_MEMORY ||
        !command.ownerLease.IsValid() || command.ownerLease.owner!=MotionOwner::AUTO ||
        command.sourceLinePC<0 || command.execution.sourceBlockId!=static_cast<MotionSourceBlockId>(command.sourceLinePC) ||
        !NCG76::FrameSupported(frame,allowFixedWorkTranslation,allowFixedWorkTranslation) || command.sourcePlaneMode!=17 || command.sourceToolRadiusMode!=40 ||
        command.sourceG16Active || command.sourceG162Active ||
        (!allowFixedWorkTranslation && (command.sourceG168Active || command.sourceWCode!=0)) ||
        command.sourceIsAbsoluteMode!=(frame.distanceMode==90) ||
        command.commandPathMode!=MotionCommandPathMode::EXACT_STOP || command.cncFeedLookahead || command.cncCornerBlend ||
        command.pathCoreFeedExactStop || command.pathCoreRotaryFeedExactStop || command.pathCoreZCFeedExactStop ||
        command.pathCoreXYZCFeedExactStop || command.pathCoreXYZCUVFeedExactStop || command.pathCoreEccentricCFeedExactStop ||
        command.edm149ZOnlyCommissioning || command.edm150XYZOnlyCommissioning || command.edm154SparseCommissioning ||
        command.replayTerminalAlreadyPublished || command.mem_enableTransform) return false;
    if (command.edm173G76Gap || orbitAirWork || mixedAirWork)
    {
        // EDM187: this invocation already validated the complete immutable
        // frame above. Its G17/G40/G15, non-eccentric, exact-stop guards also
        // prove the shared helpers' path prerequisites. Compare every source
        // tag here without repeating that same full snapshot validation.
        // This is no cache or live authority: callers still check the current
        // epoch, owner, feedback and exact translation publication separately.
        if (command.sourceWCS!=frame.wcsCode ||
            command.sourceToolLengthMode!=frame.toolLengthMode || command.sourceHCode!=frame.toolHCode ||
            command.sourceToolRadiusMode!=frame.cutterMode || command.sourceDCode!=frame.cutterD ||
            (command.sourceG68Active?68:69)!=frame.rotationMode || command.sourcePlaneMode!=frame.rotationPlane ||
            std::memcmp(&command.sourceG68Angle,&frame.rotationAngleDeg,sizeof(double))!=0 ||
            (command.sourceG168Active?168:169)!=frame.workMode || command.sourceWCode!=frame.workWCode ||
            command.sourceG51Active!=(frame.scalingMode==51) ||
            std::memcmp(&command.sourceScaleRatio,&frame.scalingFactor,sizeof(double))!=0 ||
            static_cast<std::uint32_t>(command.sourceMirrorMask)!=frame.mirrorMask ||
            (command.sourceG16Active?16:15)!=frame.polarMode) return false;
    }
    else if (!IsNCTranslationSourceAllowed(command.sourceWCS,frame) ||
        !IsMotionFixedTranslationToolSourceAllowed(command) || !IsMotionFixedTranslationCutterSourceAllowed(command) ||
        !IsMotionFixedTranslationRotationSourceAllowed(command) || !IsMotionFixedTranslationWorkSourceAllowed(command) ||
        !IsMotionFixedTranslationScaleMirrorSourceAllowed(command) || !IsMotionFixedTranslationPolarSourceAllowed(command)) return false;
    if (command.axisCount<1 || command.axisCount>3) return false;
    int previous=-1;
    for (int slot=0;slot<command.axisCount;++slot) {
        const int axis=command.axisIndices[slot];
        if (axis<=previous || axis<0 || axis>2) return false;
        previous=axis;
    }
    if (command.pathCorePlanarCircle)
        return command.axisCount==2 && command.axisIndices[0]==0 && command.axisIndices[1]==1 &&
            ((command.mode==InterpolationMode::CIRCULAR_CW && command.dir==-1) ||
                (command.mode==InterpolationMode::CIRCULAR_CCW && command.dir==1));
    return command.mode==InterpolationMode::LINEAR && !command.pathCoreFullCircle && command.dir==0;
}

inline bool IsMotionFixedTranslationSourceAllowed(const MotionCommand& command) noexcept
{
    if(!IsMotionG76SphereMarkerClear(command))return false;
    if(!IsMotionEDMCommissioningMarkerEncodingValid(command))return false;
    if(command.edm173G76Gap)return command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse && !command.pathCorePlanarCircle && command.mode==InterpolationMode::LINEAR && command.mem_transformOrigin[0]==0.0 && command.mem_transformOrigin[1]==1.0 && IsMotionG76MixedRetainedSourceAllowed(command);
    if (command.pathCoreRetainedTraversal && command.mem_transformOrigin[2]==2.0)
        return IsMotionG76MixedRetainedSourceAllowed(command);
    if (command.pathCoreEccentricCFeedExactStop) return false;
    if (command.pathCoreXYZCUVFeedExactStop) return IsMotionXYZCUVFeedSourceAllowed(command);
    if (command.pathCoreXYZCFeedExactStop) return IsMotionXYZCFeedSourceAllowed(command);
    if (command.pathCoreZCFeedExactStop) return IsMotionZCFeedSourceAllowed(command);
    if (command.pathCoreRotaryFeedExactStop) return IsMotionRotaryFeedSourceAllowed(command);
    if (!IsMotionBaseArcPlaneSourceAllowed(command)) return false;
    if (IsNCTranslationSnapshotEmpty(command.sourceTranslation))
        return command.sourceWCS == 54 && IsMotionFixedTranslationToolSourceAllowed(command) &&
            IsMotionFixedTranslationWorkSourceAllowed(command) &&
            IsMotionFixedTranslationRotationSourceAllowed(command) &&
            IsMotionFixedTranslationScaleMirrorSourceAllowed(command) &&
            IsMotionFixedTranslationPolarSourceAllowed(command) &&
            IsMotionFixedTranslationCutterSourceAllowed(command);
    // G49 with no WORK leaves the G00 C-offset flag dormant. Active H or
    // WORK requires its frozen identity and G163; no dynamic transform authority.
    return IsNCTranslationSourceAllowed(command.sourceWCS, command.sourceTranslation) &&
        (command.sourceG162Active ? 1U : 0U) == command.sourceTranslation.axisIdentity.eccentricEnabled &&
        (command.sourceIsAbsoluteMode ? 90 : 91) == command.sourceTranslation.distanceMode &&
        (command.sourceTranslation.distanceMode != 91 ||
            (command.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
                !command.cncFeedLookahead && !command.cncCornerBlend &&
                !command.pathCoreRetainedTraversal && !command.pathCoreRetainedReverse)) &&
        command.sourcePlaneMode == command.sourceTranslation.rotationPlane &&
        IsMotionFixedTranslationToolSourceAllowed(command) && IsMotionFixedTranslationCutterSourceAllowed(command) &&
        IsMotionFixedTranslationRotationSourceAllowed(command) && IsMotionFixedTranslationWorkSourceAllowed(command) &&
        IsMotionFixedTranslationScaleMirrorSourceAllowed(command) &&
        IsMotionFixedTranslationPolarSourceAllowed(command) && !command.mem_enableTransform &&
        command.axisCount >= 1 && command.axisCount <= 3;
}

static_assert(
    std::is_trivially_copyable<MotionCommand>::value,
    "MotionCommand must remain trivially copyable for the fixed SPSC ring.");
static_assert(
    std::is_standard_layout<MotionCommand>::value,
    "MotionCommand layout must remain inspectable across the fixed transport.");

#if defined(_WIN64) || defined(__x86_64__) || defined(__aarch64__)
static_assert(
    sizeof(MotionCommand) == 1064U && alignof(MotionCommand) == 8U,
    "Fixed axis/coordinate source proof appends 496 bytes; rebuild every RTSS translation unit.");
static_assert(offsetof(MotionCommand, sourceTranslation) == 568U,
    "Fixed translation must preserve every pre-existing command offset.");
static_assert(offsetof(MotionCommand,positioningOperation)==offsetof(MotionCommand,g80EdgeFind)+1U,
    "EDM280 positioning provenance must use existing command padding.");
static_assert(offsetof(MotionCommand,g76SphereAir)==offsetof(MotionCommand,positioningOperation)+1U &&
    offsetof(MotionCommand,g76SphereAir)==516U,
    "EDM286 P50 marker must use existing command padding and preserve every source offset.");
static_assert(offsetof(MotionCommand,g80EdgeFind)==offsetof(MotionCommand,sourceG68Active)+1U,
    "G80 marker must occupy only existing source-G68 padding.");
static_assert(offsetof(MotionCommand, cncPrefixVelocityPPS) == 560U,
    "DK must preserve every pre-existing MotionCommand offset.");
static_assert(
    offsetof(MotionCommand, commandPathMode) == 555U &&
    offsetof(MotionCommand, sourcePlaneMode) == 556U,
    "K.6 commandPathMode must occupy the accepted x64 tail-padding byte.");
static_assert(offsetof(MotionCommand, pathCorePlanarCircle) == 172U &&
    offsetof(MotionCommand, pathCoreFullCircle) == 173U,
    "BY circular policy flags must consume existing dir padding only.");
static_assert(offsetof(MotionCommand, pathCoreRetainedTraversal) == 174U &&
    offsetof(MotionCommand, pathCoreRetainedReverse) == 175U,
    "BZ traversal flags must consume the remaining dir padding only.");
#endif

#if defined(_WIN64) || defined(__x86_64__) || defined(__aarch64__)
static_assert(offsetof(MotionCommand, cncFeedLookahead) == 201U, "DE must not grow command transport.");
static_assert(offsetof(MotionCommand, cncCornerBlend) == 202U, "DH must use existing command padding.");
static_assert(offsetof(MotionCommand, pathCoreFeedExactStop) == 203U &&
    offsetof(MotionCommand, mem_startPos) == 208U, "DT must preserve command size and all previous offsets.");
static_assert(offsetof(MotionCommand, pathCoreRotaryFeedExactStop) == 204U,
    "BASE68 must consume existing command padding only.");
static_assert(offsetof(MotionCommand, pathCoreZCFeedExactStop) == 205U,
    "BASE70 must consume existing command padding only.");
static_assert(offsetof(MotionCommand, pathCoreXYZCFeedExactStop) == 206U,
    "BASE72 must preserve the 1064-byte command transport.");
static_assert(offsetof(MotionCommand, pathCoreXYZCUVFeedExactStop) == 207U,
    "BASE73 must preserve the 1064-byte command transport.");
static_assert(offsetof(MotionCommand, pathCoreEccentricCFeedExactStop) == 385U,
    "BASE79B must preserve the 1064-byte command and every existing offset.");
static_assert(offsetof(MotionCommand, edm173G76Gap)==389U && offsetof(MotionCommand, edm173ChildOrdinal)==390U && offsetof(MotionCommand, edm173ChildCount)==391U,"EDM173 must only consume the three existing padding bytes.");
static_assert(offsetof(MotionCommand, edm154SparseCommissioning) == 388U,
    "EDM154 must consume padding and preserve the command transport ABI");
static_assert(offsetof(MotionCommand, edm150XYZOnlyCommissioning) == 387U,
    "EDM150 commissioning marker must retain the MotionCommand ABI");
static_assert(offsetof(MotionCommand, edm149ZOnlyCommissioning) == 386U,
    "EDM149 must preserve all command transport offsets.");
#endif

// Stage NC-0.2D：NC Producer 在單一 Program Block 派送期間，
// 固定容量收集該 Block 建立的 Segment Identity。它只存在 Producer
// 執行緒，不進入 250 us Runtime，也不改變 Command / Feedback ABI。
constexpr std::size_t MOTION_PROGRAM_BLOCK_CAPTURE_CAPACITY = 128U;

struct MotionProgramBlockSubmission
{
    MotionExecutionIdentity identity{};
    MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED;
    MotionQueueTailCommitReceipt queueTailReceipt{};
    std::uint64_t translationGeneration = 0ULL;
    bool producerAccepted = false;
    MotionRejectReason immediateRejectReason = MotionRejectReason::NONE;
};

struct MotionProgramBlockCapture
{
    std::array<MotionProgramBlockSubmission,
        MOTION_PROGRAM_BLOCK_CAPTURE_CAPACITY> submissions{};
    std::size_t count = 0U;
    bool overflow = false;
};

// Stage NC-0.1C：
// - 256 筆 SPSC Ingress：NC / MDI Producer -> 250 us Motion Consumer
// - 1024 筆 RT Replay：B2 倒退跨節後，保存稍後要正向重播的路段
// - NC 預讀高水位仍維持 100，Ring 保留額外空間給 Epoch 切換
//
// Transport capacity constants are declared in MotionCommandRing.h.
constexpr std::size_t MOTION_COMMAND_HISTORY_LIMIT = 1000U;

static_assert(
    MOTION_COMMAND_REPLAY_CAPACITY >= MOTION_COMMAND_HISTORY_LIMIT,
    "B2 replay capacity must cover the retained Motion history depth.");

using MotionCommandQueue = FixedCapacitySpscCommandChannel<
    MotionCommand,
    MOTION_COMMAND_INGRESS_CAPACITY,
    MOTION_COMMAND_REPLAY_CAPACITY>;

// 放電排渣模式
enum class JumpMode {
    B0_REVERSE,        // B0: 單節反向排渣
    B1_SPECIFIC_AXIS,  // B1: 強制沿特定單軸排渣 (例如 Z 軸)
    B2_PATH_REVERSE,   // B2: 原軌跡路徑退刀 (我們已完成的時光機模式)
    B3_CENTER,         // B3: 回中心再排渣 放電點開始執行跳躍腳本
    B3_CENTER_AUTO,    // B3: 回中心再排渣 回中心點在開始執行跳躍腳本
    B4_ORBITAL_DIAGONAL // B4: 搖動/行星加工斜角跳刀



};



// 1. 定義單段跳刀參數
struct JumpSegment {
    double distance;   // 該段要移動的距離
    double velocity;   // 最高速度
    double accTime;    // 🟢 加速時間 (秒)，例如 0.2
    double decTime;    // 🟢 減速時間 (秒)，例如 0.2
};

// 2. 定義跳刀的四個階段
enum class JumpState {
    IDLE,              // 沒事，正常放電中
    RETRACTING,        // 階段一：正在原路徑往後退 (可包含多段)
    DWELL,             // 階段二：退到頂點，停留排渣
    APPROACHING,        // 階段三：正在原路徑往前衝回放電點 (可包含多段)


    B3_TO_CENTER, B3_TO_APEX, B3_DWELL, B3_FROM_APEX, B3_TO_WORKPIECE, // 🌟 B3 專用的 5 個狀態

    B4_TO_UPPER_CENTER, // 1. 斜向退回上中心
    B4_TO_APEX,         // 2. 沿主軸垂直拔高
    B4_DWELL,           // 3. 頂點停留排渣
    B4_FROM_APEX,       // 4. 沿主軸垂直降落
    B4_TO_WORKPIECE,     // 5. 斜向摸回放電點



    PAUSED_HOLD,          // 暫停中 (停在空中)
    RESUME_ALIGN_PRIMARY, // 復歸對齊：優先軸
    RESUME_ALIGN_OTHERS,  // 復歸對齊：其他軸
    RESUME_ALIGN_ALL      // 復歸對齊：全同動
};

// 3. 跳刀管理器
struct PathJumpManager {
    JumpState state = JumpState::IDLE;
    JumpMode mode = JumpMode::B2_PATH_REVERSE; // 🟢 新增：記錄當前是哪個模式
    int b1_AxisIndex = 2; // 🟢 新增：B1專用，要跳躍的實體軸編號 (通常 2 代表 Z 軸)

    double b1_dir = 1.0;            // 👈 就是漏了這行！記錄 B1 退刀是正向(+1)還是負向(-1)

    // 🟢 新增：用來儲存 B0 模式的反向單位向量 (Unit Vector)
    double b0_Vector[3] = { 0.0, 0.0, 0.0 };

    // 🟢 [新增這行]：用來儲存跳刀瞬間的實體絕對位置 (完美降落的家)
    double frozenPos[8] = { 0.0 };

    double triggerPos = 0.0;
    double currentOffset = 0.0;
    double targetOffset = 0.0;
    std::vector<JumpSegment> retractSteps;
    std::vector<JumpSegment> approachSteps;
    int currentStepIdx = 0;
    double dwellTimer = 0.0;
    double dwellTimeTarget = 0.0;
    double jumpVel = 0.0;

    // 🟢 [新增這行]：用來標記「剛跳完刀，正在軟著陸中」
    bool isRecovering = false;



    // ==========================================
    // 🌟 B3 模式專屬幾何變數 (請在這裡加回來)
    // ==========================================
    double b3_centerPos[3] = { 0.0, 0.0, 0.0 };     // 記住使用者指定的「安全中心點」(X, Y, Z)
    double b3_retractVector[3] = { 0.0, 0.0, 1.0 }; // 記住使用者指定的「拔高 3D 向量」 (預設純Z軸向上)
    double b3_distToCenter = 0.0;                 // 算出來的「放電點到中心點」的真實幾何距離
    double b3_xyVector[3] = { 0.0, 0.0, 0.0 };      // 從放電點指向中心點的「逃脫方向向量」

    std::vector<JumpSegment> b3_toCenterSteps;
    std::vector<JumpSegment> b3_toApexSteps;
    std::vector<JumpSegment> b3_fromApexSteps;
    std::vector<JumpSegment> b3_toWorkpieceSteps;
    double b3_distToApex; // 頂點總距離


    // ===================================
    // 🌟 B4 專屬參數區
    // ===================================
    double b4_upperCenterPos[3]; // 使用者指定的「上中心點」
    double b4_retractVector[3];  // 垂直拔高的方向向量 (例如 Z軸 就是 0,0,1)

    double b4_distToUpper;       // 放電點 -> 上中心點 的斜線距離
    double b4_distToApex;        // 上中心點 -> 頂點 的垂直距離

    std::vector<JumpSegment> b4_toUpperSteps;
    std::vector<JumpSegment> b4_toApexSteps;
    std::vector<JumpSegment> b4_fromApexSteps;
    std::vector<JumpSegment> b4_toWorkpieceSteps;







    // 🌟 暫停與復歸專用變數
    bool isPauseMode = false;
    int resumeAlignMode = 0;
    // 🌟 新增這行：記錄第一階段要動哪些軸 (位元遮罩)
    int firstStageMask = 0;
    double alignVel = 0.0;

    // 🌟 獨立的對齊進度 (不干擾原本的 currentOffset)
    double alignOffset = 0.0;
    double alignDist = 0.0;

    // 🌟 座標快照
    double apexPos[8] = { 0 };        // 頂點快照
    double joggedStartPos[8] = { 0 }; // Jog後的起點快照



};


struct InterpolationGroup// 插補群組
{
    //運行狀態與模式------------------------------------------------------
    bool isActive = false;// 插補引擎運作標記 (true: 正在計算路徑位移, false: 停止)
    InterpolationMode mode = InterpolationMode::LINEAR;// 當前幾何模式 (預設直線)


    //參與軸與投影參數 (用於直線插補)------------------------------------------------------

    int  axisCount = 0;// 參與聯動的實體軸總數 (例如 2 軸或 3 軸)
    int  axisIndices[MAX_AXES] = { 0 };// 記錄參與軸的編號清單 (例如 {0, 1} 代表 X, Y 軸)
    double startPos[MAX_AXES] = { 0.0 };// 記錄各段路徑開始時，各實體軸的起點位置 (Snapshot)
    double ratio[MAX_AXES] = { 0.0 };// 方向向量/分量比例 (單位路徑位移時，各軸應分配的比例)



    //圓弧插補專用幾何參數------------------------------------------------------
    double centerX = 0.0;// 圓心 X 座標 (絕對座標或相對距離，依演算法定義)
    double centerY = 0.0;// 圓心 Y 座標
    double radius = 0.0;// 圓弧半徑
    double startAngle = 0.0;// 起始角度 (弧度 Radian)


    //螺旋插補專用記憶體
    double totalAngle = 0.0;   // 記錄總共要轉多少角度 (包含正負號)
    double totalDist3D = 0.0;  // 記錄真實的 3D 總路徑長度



    // ==========================================
    // 🟢 [新增] 空間座標旋轉矩陣 (G68 功能)
    // ==========================================
    bool enableTransform = false;         // 旋轉開關
    double transformOrigin[3] = { 0, 0, 0 }; // 旋轉中心點 (X, Y, Z)
    double transformMatrix[3][3] = {       // 3x3 旋轉矩陣 (預設為不旋轉的單位矩陣)
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    //運動規劃核心------------------------------------------------------
    AxisContext virtualAxis;// 這是「虛擬主軸」，負責跑總路徑長度 (s)，實體軸再依比例跟隨


   //速度與進給控制------------------------------------------------------
    double feedrateOverride = 0.01;// 插補整體的進給速度倍率控制 (0.0 ~ 1.0+)



    //任務緩衝管理------------------------------------------------------
    // Stage NC-0.1C：固定容量 SPSC Command Channel。
    // Producer 只寫 Ingress；250 us Runtime 是唯一 Consumer。
    // B2 push-front 由 Channel 內部的 RT-only Replay 區處理。
    MotionCommandQueue cmdQueue;
    int currentExecutionPC = 0; // 馬達當下的行號
    // 🌟 2. 新增：實體馬達當下正在跑的座標系
    int currentExecutionWCS = 54;

    // 🌟 2. 擴充實體狀態：實體馬達當下正在用的刀具狀態
    int currentExecutionToolMode = 49; // 預設 G49 (無補正)
    int currentExecutionHCode = 0;     // 預設 H0

    // 🌟 2. 新增實體狀態：馬達當下的刀徑狀態
    int currentExecutionToolRadiusMode = 40;
    int currentExecutionDCode = 0;

    // 🌟 沿用你的命名：馬達當下的 G90/G91 狀態
    bool currentExecutionIsAbsoluteMode = true; // 預設 G90(true)

    // 🌟 2. 實體狀態：馬達當下的 G68 狀態
    bool currentExecutionG68Active = false;
    double currentExecutionG68Angle = 0.0; // 🌟 2. 實體狀態：馬達當下的角度
    // 🌟 2. 實體狀態：馬達當下的 G168 狀態
    bool currentExecutionG168Active = false;
    int currentExecutionWCode = 0; // 🌟 2. 實體狀態：馬達當下的 W 碼
    // 🌟 2. 實體狀態：馬達當下的 G51 狀態
    bool currentExecutionG51Active = false;
    double currentExecutionScaleRatio = 1.0;

    // 🌟 2. 實體狀態：馬達當下的鏡像狀態
    uint8_t currentExecutionMirrorMask = 0;

    // 🌟 2. 實體狀態：馬達當下的極座標狀態
    bool currentExecutionG16Active = false;

    bool currentExecutionG162Active = true;
    int currentExecutionPlaneMode = 17; // 預設 G17

    //時光機專用擴充套件 ------------------------------------------------------

    bool enableHistory = false;             // 時光機模式開關
    std::deque<MotionCommand> historyQueue; // 歷史軌跡 (跑完的麵包屑)
    MotionCommand currentCmd;               // 當前指令 (備份用，退刀時才知道這條線長怎樣)

    //路徑連接模式------------------------------------------------------
    PathMode pathMode = PathMode::EXACT_STOP;// 決定兩段指令之間要「精確停止」還是「連續轉彎不降速」


    //路徑模式------------------------------------------------------
    double pathServoVel = 0.0;// 外部路徑伺服速度輸入 (單位: Pulse/sec)

    //跳刀管理器------------------------------------------------------

    PathJumpManager jumpManager;
};





// ============================================================================
// Stage 11E.4 - Servo OUTPUT Command Seam Field Identity
//
// Axis identity remains AxisContext.axisIndex.
//
// This enum describes which of the four Servo command fields was written
// through the active LEGACY command seam.
// ============================================================================

enum class MotionServoOutputCommandField : uint8_t
{
    ControlWord = 0,
    TargetVelocity = 1,
    TouchProbeFunction = 2,
    ModesOfOperation = 3
};


// ============================================================================
// Stage 11D.6 - Motion Servo INPUT Consumer Seam
//
// Active source in this stage remains LEGACY ENI_ServoDrive::pInput.
// Motion algorithms consume this POD snapshot instead of direct pInput reads.
// ============================================================================

struct MotionServoInputSnapshot
{
    uint16_t StatusWord = 0U;
    int32_t ActualPosition = 0;
    int8_t ModesOfOperationDisplay = 0;
    uint16_t TouchProbeStatus = 0U;
    int32_t TouchProbePosition = 0;
};



// =============================================================================
// Stage NC-0.2I.2 - Read-only Feed Hold Motion Stop Snapshot
//
// IsGroupStandstill() intentionally requires the active segment and all queued
// commands to be finished. Feed Hold pauses inside the current segment and
// preserves that segment plus future queue data. This snapshot therefore
// reports velocity-based controlled-stop conditions without requiring Group
// Done or an empty command queue.
// =============================================================================
struct MotionFeedHoldStopSnapshot
{
    double feedrateOverride = 1.0;
    double virtualCommandVelocityPps = 0.0;
    double maxAxisCommandVelocityPps = 0.0;
    double maxAxisActualVelocityPps = 0.0;

    std::uint32_t groupAxisCount = 0U;
    std::uint32_t commandMovingAxes = 0U;
    std::uint32_t actualMovingAxes = 0U;
    std::uint32_t faultedAxes = 0U;

    std::uint32_t commandQueueDepth = 0U;
    std::uint32_t commandIngressDepth = 0U;
    std::uint32_t commandReplayDepth = 0U;

    bool groupActive = false;
    bool groupDone = false;
    bool groupFaulted = false;
    bool groupEmergencyStopped = false;
    bool safetyOrRecoveryPending = false;
    bool overrideZero = false;
    bool virtualCommandStopped = true;
    bool axisCommandStopped = true;
    bool axisActualStopped = true;
    bool commandStopped = true;
    bool actualStopped = true;
    bool motionStopped = true;

    // Stage NC-0.2J.5: formal Feed Hold settle proof.  The legacy raw
    // encoder velocity fields above remain diagnostic-only; NC release gates
    // correlate this request identity with the 250 us continuous proof.
    std::uint64_t settleRequestSequence = 0ULL;
    std::uint64_t settleProofSequence = 0ULL;
    std::uint32_t settleScopeMask = 0U;
    std::uint32_t settleDwellCycles = 0U;
    std::uint32_t settleRequiredCycles = 0U;
    bool settleRequestAccepted = false;
    bool settleProofValid = false;
    bool ncSettled = false;
};

static_assert(
    std::is_trivially_copyable<MotionFeedHoldStopSnapshot>::value,
    "MotionFeedHoldStopSnapshot must remain trivially copyable.");


// =============================================================================
// Stage NC-0.2J.4 - Motion Stop / Settle Evidence Shadow
//
// This is diagnostic evidence only.  It mirrors the existing
// IsGroupStandstill() predicate without granting permission to release any
// NC gate.  The 250 us Motion owner publishes a fixed-size snapshot; readers
// never inspect AxisContext directly.
// =============================================================================
enum class MotionStopSettlePrimaryBlocker : std::uint8_t
{
    NONE = 0,
    GROUP_ACTIVE = 1,
    COMMAND_QUEUE = 2,
    AXIS_NOT_IDLE = 3,
    AXIS_COMMAND_VELOCITY = 4,
    AXIS_ACTUAL_VELOCITY = 5
};


struct MotionStopSettleCounters
{
    std::uint64_t sampleCount = 0ULL;
    std::uint64_t standstillSampleCount = 0ULL;
    std::uint64_t blockedSampleCount = 0ULL;

    std::uint64_t noneBlockerCount = 0ULL;
    std::uint64_t groupActiveBlockerCount = 0ULL;
    std::uint64_t commandQueueBlockerCount = 0ULL;
    std::uint64_t axisNotIdleBlockerCount = 0ULL;
    std::uint64_t axisCommandVelocityBlockerCount = 0ULL;
    std::uint64_t axisActualVelocityBlockerCount = 0ULL;

    std::uint64_t primaryBlockerTransitionCount = 0ULL;
    std::uint64_t standstillTransitionCount = 0ULL;
    std::uint64_t transitionIntoStandstillCount = 0ULL;
    std::uint64_t transitionOutOfStandstillCount = 0ULL;
};


struct MotionStopSettleSnapshot
{
    std::uint64_t publicationGeneration = 0ULL;
    std::uint64_t sampleSequence = 0ULL;

    MotionStopSettlePrimaryBlocker primaryBlocker =
        MotionStopSettlePrimaryBlocker::NONE;
    std::int32_t primaryAxisIndex = -1;
    MotionState primaryAxisState = MotionState::MotionState_IDLE;

    bool standstill = false;
    bool groupActive = false;

    std::uint32_t commandQueueDepth = 0U;
    std::uint32_t commandIngressDepth = 0U;
    std::uint32_t commandReplayDepth = 0U;
    std::uint32_t groupAxisCount = 0U;

    std::uint32_t existingAxisCount = 0U;
    std::uint32_t nonIdleAxisCount = 0U;
    std::uint32_t commandMovingAxisCount = 0U;
    std::uint32_t actualMovingAxisCount = 0U;
    std::uint32_t outsideInPositionWindowAxisCount = 0U;
    std::uint32_t groupOutsideInPositionWindowAxisCount = 0U;
    std::uint32_t pdoTargetVelocitySampledAxisCount = 0U;
    std::uint32_t pdoTargetVelocityNonzeroAxisCount = 0U;

    double commandVelocityDeadbandPps = 1.0;
    double actualVelocityDeadbandPps = 1.0 / CYCLE_TIME_SEC;

    std::int32_t worstCommandVelocityAxisIndex = -1;
    double maxAxisCommandVelocityAbsPps = 0.0;

    std::int32_t worstActualVelocityAxisIndex = -1;
    double maxAxisActualVelocityAbsPps = 0.0;

    std::int32_t worstFinalPdoTargetVelocityAxisIndex = -1;
    std::int32_t worstFinalPdoTargetVelocity = 0;
    std::uint32_t maxFinalPdoTargetVelocityAbs = 0U;

    // Worst following-error axis is selected by Error / Window ratio across
    // all enabled physical axes.  Values are absolute observer evidence and
    // are not part of IsGroupStandstill().
    std::int32_t worstFollowingErrorAxisIndex = -1;
    double worstFollowingErrorAbsMm = 0.0;
    double worstFollowingErrorWindowMm = 0.0;
    double worstFollowingErrorWindowRatio = 0.0;

    // Same evidence restricted to the current interpolation group.  This is
    // the useful explanation while GROUP_ACTIVE is waiting for G00/G01
    // endpoint pull-in.
    std::int32_t worstGroupFollowingErrorAxisIndex = -1;
    double worstGroupFollowingErrorAbsMm = 0.0;
    double worstGroupFollowingErrorWindowMm = 0.0;
    double worstGroupFollowingErrorWindowRatio = 0.0;

    std::int32_t maxStopDecTimeAxisIndex = -1;
    double maxConfiguredStopDecTimeSec = 0.0;
};


static_assert(
    std::is_trivially_copyable<MotionStopSettleCounters>::value,
    "MotionStopSettleCounters must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<MotionStopSettleSnapshot>::value,
    "MotionStopSettleSnapshot must remain trivially copyable.");


// =============================================================================
// Stage NC-0.2J.6.1/J.6.3.2 - Emergency-stop RT evidence shadow
//
// The existing EmergencyStopAllAxes() behavior is intentionally unchanged in
// this stage.  The 250 us Motion owner only publishes fixed-size evidence that
// tells the supervisory Alarm boundary whether an emergency request really
// reached the RT consumer, which execution Epoch it invalidated and which axes
// entered a non-running safety state.  No field below grants Reset/recovery
// permission and no SHM ABI is expanded.
// =============================================================================
struct MotionEmergencyStopCounters
{
    std::uint64_t requestAttempts = 0ULL;
    std::uint64_t requestsPublished = 0ULL;
    std::uint64_t requestsCoalesced = 0ULL;
    std::uint64_t rtApplications = 0ULL;
    std::uint64_t epochInvalidations = 0ULL;
};


struct MotionEmergencyStopEvidence
{
    std::uint64_t publicationGeneration = 0ULL;

    MotionExecutionEpoch currentExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch lastAppliedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;

    MotionOwner currentOwner = MotionOwner::NONE;
    MotionOwnerGeneration currentOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    MotionOwner lastAppliedOwner = MotionOwner::NONE;
    MotionOwnerGeneration lastAppliedOwnerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::uint32_t existingAxisMask = 0U;
    std::uint32_t estopAxisMask = 0U;
    std::uint32_t errorAxisMask = 0U;
    std::uint32_t faultAxisMask = 0U;
    std::uint32_t lagAlarmAxisMask = 0U;
    std::uint32_t commandZeroAxisMask = 0U;
    std::uint32_t targetSealedAxisMask = 0U;
    std::uint32_t pdoTargetVelocitySampledAxisMask = 0U;
    std::uint32_t pdoTargetVelocityZeroAxisMask = 0U;

    bool requestPending = false;
    bool requestInProgress = false;
    bool lastApplyHadExecutionToInvalidate = false;
    bool groupActive = false;
    bool groupEmergencyStopped = false;
    bool groupError = false;
    bool virtualCommandZero = false;
    bool virtualTargetSealed = false;
    bool allExistingAxesSafe = false;
    bool allExistingAxisCommandsZero = false;
    bool allExistingAxisTargetsSealed = false;
    bool allSampledPdoTargetVelocitiesZero = false;
    bool safetyLeaseMatchesLastApply = false;
    bool rtStopStateApplied = false;
};


// Stage NC-0.2J.6.3.2: one separately published atomic record retains the
// latest real E-stop Epoch transition without increasing the already-full
// 192-word stop/settle RT payload.  Both epochs are read from one 64-bit word.
struct MotionEmergencyStopEpochInvalidationEvidence
{
    MotionExecutionEpoch fromExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionEpoch toExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    std::uint64_t invalidationCount = 0ULL;
};


// =============================================================================
// Stage NC-0.2J.6.4 - Startup Feedback Alignment / Permanent Lag Arming
//
// This is a standalone atomic diagnostic record.  It deliberately stays out
// of the already-full J.5 192-word stop/settle publication.  The RT owner is
// the only writer; supervisory code receives masks and monotonic counters.
// =============================================================================
struct MotionStartupLagArmingEvidence
{
    std::uint32_t existingAxisMask = 0U;
    std::uint32_t feedbackReadyAxisMask = 0U;
    std::uint32_t positionAlignedAxisMask = 0U;
    std::uint32_t lagArmedAxisMask = 0U;
    std::uint32_t pendingAxisMask = 0U;
    std::uint32_t prematureMotionBlockedAxisMask = 0U;

    std::uint32_t minimumStableSampleCount = 0U;
    std::uint32_t stableSamplesRequired =
        MOTION_STARTUP_LAG_ARM_STABLE_SAMPLES;

    std::uint64_t alignmentEvents = 0ULL;
    std::uint64_t armingTransitions = 0ULL;
    std::uint64_t readinessResets = 0ULL;
    std::uint64_t prematureMotionBlocks = 0ULL;

    bool allExistingAxesArmed = false;
    bool blocked = false;
};


// =============================================================================
// Stage NC-0.2K.2.1 - P1 Mixed-Axis Handover Safety Diagnostics
//
// These values are monotonic, read-only diagnostics.  They deliberately stay
// outside the fixed 192-word J.5 publication: the RT owner updates atomics and
// the 1 s supervisor only observes them.  A healthy mixed-axis P1 run may
// increase mappingBoundaryStops and droppedAxisRetirements; retirement,
// orphan and invalid-producer failure counters must remain zero.
// =============================================================================
struct MotionP1HandoverSafetySnapshot
{
    std::uint64_t mappingBoundaryStops = 0ULL;
    std::uint64_t droppedAxisRetirements = 0ULL;
    std::uint64_t droppedAxisRetirementFailures = 0ULL;
    std::uint64_t orphanAxisContainments = 0ULL;
    std::uint64_t invalidProducerRejects = 0ULL;
    std::uint64_t mappingIntegrityAlarmRequests = 0ULL;
    bool mappingIntegrityAlarmPending = false;

    std::uint32_t lastPreviousAxisMask = 0U;
    std::uint32_t lastNextAxisMask = 0U;
    MotionExecutionEpoch lastMappingIntegrityAlarmExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    std::int32_t lastOrphanAxisIndex = -1;
};


// =============================================================================
// Stage NC-0.2K.2.2 - Lifecycle Atomic Commit Reservation
//
// The 250 us Runtime reserves the same packed word used to publish a new
// Execution Epoch before it commits a terminal, mapping handoff or history
// crossing.  A lifecycle publisher and an RT commit therefore have one total
// order: whichever CAS wins is observed first, and neither side can cross the
// other's mutation boundary.
// =============================================================================
struct MotionLifecycleCommitReservationSnapshot
{
    std::uint64_t attempts = 0ULL;
    std::uint64_t acquired = 0ULL;
    std::uint64_t blockedByLifecycle = 0ULL;
    std::uint64_t compareExchangeLost = 0ULL;
    std::uint64_t released = 0ULL;
    std::uint64_t releaseFailures = 0ULL;
    std::uint64_t publisherWaits = 0ULL;

    MotionExecutionEpoch currentExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    bool reservationActive = false;
    bool executionEpochPending = false;
};


static_assert(
    std::is_trivially_copyable<MotionEmergencyStopCounters>::value,
    "MotionEmergencyStopCounters must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<MotionEmergencyStopEvidence>::value,
    "MotionEmergencyStopEvidence must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    MotionEmergencyStopEpochInvalidationEvidence>::value,
    "Motion E-stop Epoch invalidation evidence must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionStartupLagArmingEvidence>::value,
    "Startup Lag arming evidence must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionP1HandoverSafetySnapshot>::value,
    "P1 handover safety diagnostics must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<
    MotionLifecycleCommitReservationSnapshot>::value,
    "Lifecycle commit reservation diagnostics must remain trivially copyable.");
static_assert(
    sizeof(MotionExecutionEpoch) == sizeof(std::uint32_t),
    "Packed E-stop Epoch invalidation requires 32-bit execution Epochs.");


// =============================================================================
// Stage NC-0.2J.5 - NC Settle Truth Model
//
// A gate receives permission only after 200 adjacent, valid 250 us Runtime
// samples (50 ms).  Encoder-derived currentActVel and final PDO velocity stay
// in the J.4 shadow above as advisory evidence and are deliberately absent
// from the formal predicate.
// =============================================================================
using MotionNCSettleRequestSequence = std::uint64_t;

constexpr MotionNCSettleRequestSequence
MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID = 0ULL;

constexpr std::uint32_t MOTION_NC_SETTLE_REQUIRED_CYCLES = 200U;

enum class MotionNCSettleProfile : std::uint8_t
{
    GROUP_COMPLETION = 0,
    FEED_HOLD_GROUP = 1,
    RESET_ALL = 2,
    COUNT = 3
};

enum class MotionNCSettleBlocker : std::uint8_t
{
    NONE = 0,
    RUNTIME_NOT_OBSERVED = 1,
    RUNTIME_INVALID = 2,
    RUNTIME_GAP = 3,
    REQUEST_MISSING = 4,
    EXECUTION_EPOCH_MISMATCH = 5,
    OWNER_LEASE_MISMATCH = 6,
    SCOPE_CHANGED = 7,
    GROUP_ACTIVE = 8,
    COMMAND_QUEUE = 9,
    FEED_OVERRIDE_NONZERO = 10,
    SAFETY_OR_RECOVERY_PENDING = 11,
    AXIS_FAULT_OR_ESTOP = 12,
    AXIS_SERVO_OFF = 13,
    AXIS_STATE = 14,
    COMMAND_VELOCITY = 15,
    COMMAND_POSITION_CHANGED = 16,
    IN_POSITION_WINDOW_INVALID = 17,
    FOLLOWING_ERROR = 18,
    ACTUAL_EXCURSION = 19,
    RESET_UNSUPPORTED = 20,
    COMPENSATION_ACTIVE = 21,
    PATH_RUNTIME_UNSUPPORTED = 22,
    REBASE_IN_PROGRESS = 23,
    REBASE_VERIFY_FAILED = 24,
    PUBLICATION_BUSY = 25,
    SCOPE_EMPTY = 26
};

struct MotionNCSettleCounters
{
    std::uint64_t sampleCount = 0ULL;
    std::uint64_t candidateStartCount = 0ULL;
    std::uint64_t candidateResetCount = 0ULL;
    std::uint64_t proofRiseCount = 0ULL;
    std::uint64_t proofRevokeCount = 0ULL;
    std::uint64_t invalidRuntimeCycleCount = 0ULL;
    std::uint64_t runtimeGapCount = 0ULL;
    std::uint64_t identityResetCount = 0ULL;
    std::uint64_t scopeResetCount = 0ULL;
    std::uint64_t requestRejectCount = 0ULL;
    std::uint64_t publicationReadFailureCount = 0ULL;
};

struct MotionNCSettleSnapshot
{
    std::uint64_t publicationGeneration = 0ULL;
    std::uint64_t sampleSequence = 0ULL;
    std::uint64_t runtimeCycleTick = 0ULL;
    MotionNCSettleRequestSequence requestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    std::uint64_t proofSequence = 0ULL;

    MotionNCSettleProfile profile =
        MotionNCSettleProfile::GROUP_COMPLETION;
    MotionNCSettleBlocker blocker =
        MotionNCSettleBlocker::RUNTIME_NOT_OBSERVED;
    std::int32_t blockerAxisIndex = -1;

    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;

    std::uint32_t scopeMask = 0U;
    std::uint32_t dwellCycles = 0U;
    std::uint32_t requiredCycles = MOTION_NC_SETTLE_REQUIRED_CYCLES;

    bool runtimeObserved = false;
    bool runtimeCycleValid = false;
    bool runtimeCycleContiguous = false;
    bool requestAccepted = false;
    bool groupActive = false;
    bool groupDrained = false;
    bool safetyOrRecoveryPending = false;
    bool anyAxisFaultOrEstop = false;
    bool overrideZero = false;
    bool virtualCommandStopped = false;
    bool allAxisCommandStopped = false;
    bool candidate = false;
    bool settled = false;
    bool rebasePreProofPassed = false;
    bool rebasePostProofPassed = false;

    std::uint32_t commandQueueDepth = 0U;
    std::uint32_t commandIngressDepth = 0U;
    std::uint32_t commandReplayDepth = 0U;

    double maxCommandVelocityAbsPps = 0.0;
    double feedrateOverride = 1.0;
    double virtualCommandVelocityAbsPps = 0.0;
    double worstFollowingErrorAbsPulse = 0.0;
    double worstFollowingWindowPulse = 0.0;
    double worstActualExcursionPulse = 0.0;
    double worstActualExcursionLimitPulse = 0.0;

    // Advisory only.  Neither value can block or revoke settled=true.
    double advisoryMaxActualVelocityAbsPps = 0.0;
    std::uint32_t advisoryMaxPdoTargetVelocityAbs = 0U;
    // Group 32-bit metadata to avoid padding in the fixed RT publication.
    std::int32_t worstCommandVelocityAxisIndex = -1;
    std::int32_t worstFollowingErrorAxisIndex = -1;
    std::int32_t worstActualExcursionAxisIndex = -1;
};

// Modal/physical tags captured by NC when Reset starts.  The RT rebase ACK
// returns the same POD, proving the acknowledgement belongs to that exact
// lifecycle transaction rather than to an older reset.
struct MotionNCResetExecutionState
{
    std::int32_t physicalExecutionPC = 0;
    std::int32_t physicalExecutionWCS = 54;
    std::int32_t physicalToolMode = 49;
    std::int32_t physicalHCode = 0;
    std::int32_t physicalToolRadiusMode = 40;
    std::int32_t physicalDCode = 0;
    std::int32_t physicalWCode = 0;
    std::int32_t physicalPlaneMode = 17;
    std::uint8_t physicalMirrorMask = 0U;
    bool physicalIsAbsoluteMode = true;
    bool physicalG68Active = false;
    bool physicalG168Active = false;
    bool physicalG51Active = false;
    bool physicalG16Active = false;
    bool physicalG162Active = true;
    double physicalG68Angle = 0.0;
    double physicalScaleRatio = 1.0;
};

enum class MotionNCResetRebasePhase : std::uint8_t
{
    IDLE = 0,
    WAIT_PREPROOF = 1,
    CLEARING_BUFFERS = 2,
    WAIT_POSTPROOF = 3,
    ACKNOWLEDGED = 4,
    BLOCKED = 5,
    SUPERSEDED = 6
};

struct MotionNCResetRebaseAck
{
    MotionNCSettleRequestSequence requestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    std::uint32_t requestedAxisMask = 0U;
    std::uint32_t appliedAxisMask = 0U;
    MotionNCResetRebasePhase phase = MotionNCResetRebasePhase::IDLE;
    MotionNCSettleBlocker failureBlocker = MotionNCSettleBlocker::NONE;
    bool requestAccepted = false;
    bool rebaseApplied = false;
    bool postVerifyPassed = false;
    bool acknowledged = false;
    bool acked = false;
    bool blocked = false;
    bool superseded = false;
    bool unsupportedFaultOrEstop = false;
    bool compensationBlocked = false;
    MotionNCResetExecutionState executionState{};
    double actualPulse[MAX_AXES] = { 0.0 };
    double actualMcsUnit[MAX_AXES] = { 0.0 };
    // PBC-3F: only physical X has an integrated retained-offset reference.
    // Keep raw measurements above intact and the existing RT payload budget.
    double xNominalMcsUnit = 0.0;
};

static_assert(
    std::is_trivially_copyable<MotionNCSettleCounters>::value,
    "MotionNCSettleCounters must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionNCSettleSnapshot>::value,
    "MotionNCSettleSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionNCResetExecutionState>::value,
    "MotionNCResetExecutionState must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<MotionNCResetRebaseAck>::value,
    "MotionNCResetRebaseAck must remain trivially copyable.");

// Kept outside MotionStopSettlePublicationPayload: that fixed RT bank already
// occupies its 192-word ABI ceiling. A separate bounded seqlock binds the
// supervisory Reset release to the exact post-proof Safety incident.
struct MotionNCResetSafetyReleaseAuthorization
{
    MotionNCSettleRequestSequence requestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    std::uint64_t drainRevocationGeneration = 0ULL;
    MotionExecutionEpoch executionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    std::uint32_t safetyRequestTicket = 0U;

    bool IsValid() const noexcept
    {
        return
            requestSequence !=
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            drainRevocationGeneration != 0ULL &&
            executionEpoch != MOTION_EXECUTION_EPOCH_INVALID &&
            ownerGeneration != MOTION_OWNER_GENERATION_INVALID &&
            safetyRequestTicket != 0U;
    }
};

static_assert(
    std::is_trivially_copyable<
    MotionNCResetSafetyReleaseAuthorization>::value,
    "Reset Safety release authorization must remain trivially copyable.");

// RESET smooth-stop ingress is deliberately separate from the immediate
// Alarm/E-stop ticket path. Its persistent phase closes the NC 10 ms / RT
// 250 us handoff without making the pre-ticket request an immediate PDO-zero
// condition.
enum class ResetControlledStopPhase : std::uint32_t
{
    IDLE = 0U,
    PENDING = 1U,
    APPLYING = 2U,
    ACTIVE = 3U,
    COMPLETED = 4U,
    SUPERSEDED = 5U
};


//核心類別宣告--------------------------------------------------------------------
// ==========================================
// 核心運動控制類別 (MotionCore Class)
// 職責：處理單軸運動、多軸插補、PID 閉迴路、以及 EDM 路徑伺服
// ==========================================
enum class MotionNCTranslationTransitionResult : std::uint8_t
{
    ACCEPTED,
    DEFERRED,
    UNSUPPORTED_PREDECESSOR
};

// FIX15: transient lifecycle sampling never substitutes for mapping validity.
enum class MotionQueueTailSyncResult { Ready, Retry, InvalidMapping };

// One NC producer owns this reusable staging storage. Commands stay private
// until the existing Motion ingress publishes the whole bounded compound.
struct MotionG76OrbitProducerWorkspace
{
    std::array<MotionCommand,128U> commands{};
    NCG76Mixed::NominalSource rebuiltNominal{};
    std::array<double,8U> startPulse{},stagedPulse{},stagedMCS{};
    std::vector<int> axes=std::vector<int>{0,1,2};
    std::vector<double> targetPulse=std::vector<double>(3U,0.0);
};

// Startup-owned separate NC staging and RT immutable copies. Only the SPSC
// publication crosses threads; neither side resets the shared ring.
struct MotionEDM173Seal
{
    EDM173::Plan plan{};
    std::array<MotionExecutionIdentity,EDM173::MaximumNominalCount> identities{};
    std::array<std::array<double,8U>,EDM173::MaximumNominalCount> endPulse{};
    std::array<double,8U> startPulse{},pulsePerUnit{};
    MotionExecutionEpoch epoch=MOTION_EXECUTION_EPOCH_INVALID;
};
struct MotionEDM173Storage
{
    // C++14 does not guarantee cache-line alignment for ordinary heap new.
    static void* operator new(std::size_t size)
    {
        void* allocation=::operator new(size+alignof(MotionEDM173Seal)+128U);
        const auto raw=reinterpret_cast<std::uintptr_t>(allocation)+sizeof(void*);
        const auto aligned=(raw+63U)&~static_cast<std::uintptr_t>(63U);
        void* output=reinterpret_cast<void*>(aligned);
        reinterpret_cast<void**>(output)[-1]=allocation;return output;
    }
    static void operator delete(void* value) noexcept
    { if(value)::operator delete(reinterpret_cast<void**>(value)[-1]); }

    FixedCapacitySpscRing<MotionEDM173Seal,2U> messages{};
    MotionEDM173Seal producer{}, current{};
    EDM173::ValidationScratch validation{}; // Startup-owned; only the RT seal consumer writes it.
    EDM128::B3Orbit b3Orbit{}; // B3/B4 center table: RT-owned, frozen at first child bind.
    std::atomic<std::uint64_t> publishedSession{0ULL};
    EDM53::Channel<EDM173::Feedback,4096U> feedback{};
    EDM173::Feedback currentFeedback{};
    // FIX7: RT-owned, observation-only source evidence. No control authority.
    EDM173InputDiag::Recorder inputDiagnostic{};
    EDM206::Recorder syncHistory{}; // Startup-owned bounded RT ring; immutable NC claim after containment.
    EDM53::Channel<EDM173InputDiag::Evidence,2048U> inputDiagnosticFeedback{};
    std::uint64_t inputDiagnosticPublishedSession=0ULL;
    std::uint32_t terminalMask=0U;
    bool sealed=false,handshaking=false;
    unsigned nextChild=0U;
    std::uint64_t handshakeUs=0ULL;
};

class MotionCore
{
public:
    MotionCore();
    ~MotionCore();
    bool TryG80Move(const G80::Request& request, const NCTranslationSnapshot& frame,
        MotionExecutionEpoch epoch, const MotionOwnerLease& owner, int sourcePC,
        MotionExecutionIdentity& identity) noexcept;
    bool ReadG80Feedback(MotionG80Feedback& out) const noexcept;
    bool BeginRotarySpindleSession(const NCSpindleDispatch::Authority& authority,
        const NCSpindleDispatch::Config& config) noexcept;
    bool SubmitRotarySpindlePacket(const NCSpindleDispatch::Packet& packet) noexcept;
    bool ReadRotarySpindleFeedback(MotionRotarySpindleFeedback& out) const noexcept;
    bool ReleaseRotarySpindleSession(const NCSpindleDispatch::Authority& authority) noexcept;
    bool IsRotarySpindleSessionClaimed() const noexcept;

    bool TryG76SphereMove(const NCG76SphereCycle::Request& request,const NCTranslationSnapshot& frame,
        MotionExecutionEpoch epoch,const MotionOwnerLease& owner,int sourcePC,
        MotionExecutionIdentity& identity,MotionG76SphereAdmission* admission=nullptr) noexcept;
    bool ReadG76SphereFeedback(MotionG76SphereFeedback& out) const noexcept;
    bool BindG76SphereParent(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::uint64_t dispatch,std::uint64_t commit) noexcept;
    bool RequestG76SphereHold(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::uint64_t holdSequence,std::uint64_t startFloor) noexcept;
    bool RequestG76SphereResume(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::uint64_t holdSequence,std::uint64_t startSequence) noexcept;
    bool RequestG76SphereResetCancellation(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::uint64_t resetHoldProvenance) noexcept;
    bool CommitG76SphereCompletedEndpoint(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::array<double,8U>& endpointMCS) noexcept;
    bool ReleaseG76SphereBarrier(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner) noexcept;

    bool ConfigureCollisionProtection(const EDMGapServo::Profile& profile,bool valid) noexcept;
    void SetManualShortBypass(bool bypass) noexcept;
    bool ReadCollisionProtection(EDMShortCircuitProtection::Snapshot& out) const noexcept;
    bool RequestG80ResetCancellation(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner,std::uint64_t resetHoldProvenance) noexcept;
    bool ReleaseG80Barrier(std::uint64_t session,const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner) noexcept;
    bool CommitG80CompletedEndpoint(std::uint64_t session, const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner, G80::Point6& endpointMCS) noexcept;
    MotionNCTranslationTransitionResult TryCommitG80OriginTranslation(
        const NCTranslationSnapshot& previous, const NCTranslationSnapshot& next,
        MotionExecutionEpoch epoch, const MotionOwnerLease& owner, std::uint64_t session) noexcept;

    // BASE79F: initialize once before any cyclic work; no cyclic allocation.
    bool InitializeEccentricCConsumer() noexcept;
    bool IsEccentricCConsumerReady() const noexcept;
    std::size_t EccentricCConsumerStorageBytes() const noexcept;




    //系統關聯與連結--------------------------------------------------------------------
    CoordinateManager* m_pCoordMgr = nullptr;
    CompensationEngine m_CompEngine; // 🌟 宣告補償引擎


    void Link(std::vector<ENI_ServoDrive>* pAxisList);// 連結實體驅動器列表 (EtherCAT 映射資料)
    void Link(std::vector<ENI_ServoDrive>* pDriveList, std::vector<AxisContext>* pContextList);// 連結實體驅動器與邏輯參數上下文 (Context)

    // Stage 11D.3:
    // Bind the owning EtherCatMaster only for SHADOW comparison.
    // Actual Motion input/output consumers remain ENI_ServoDrive.
    void BindStructuredServoReadShadowMaster(
        EtherCatMaster* pMaster);

    void LinkCoordinateManager(CoordinateManager* pCoord);
    AxisContext& GetAxisContext(int index)
    {
        static AxisContext dummy; // 防呆：避免指標為空時引發崩潰
        if (m_pContexts == nullptr || index < 0 || index >= m_pContexts->size()) {
            return dummy;
        }
        return (*m_pContexts)[index];
    }

    //軸狀態
    void UpdateAllMotion();//更新全部軸狀態 逐步激磁

    // Priority-64 EtherCAT send-point transaction. Begin acquires exact
    // packed Owner/Epoch reservations before the final LRW payload capture;
    // Finalize verifies them and End keeps both held through SendPacket. On a
    // race the live image is scrubbed and copied once more as all-zero.
    struct PbcHomeProbeCommand
    {
        MotionPbcHomeCaptureKey key{};
        std::uint64_t sequence = 0ULL, tick = 0ULL;
        std::uint16_t value = 0U;
        bool pending = false;
    };

    struct ServoOutputFrameReservation
    {
        AlarmManager::MotionAdmissionReservation alarmAdmission{};
        std::uint64_t baseOwnerState = 0ULL;
        std::uint64_t reservedOwnerState = 0ULL;
        std::uint64_t baseExecutionPublication = 0ULL;
        std::uint64_t reservedExecutionPublication = 0ULL;
        std::uint64_t safetyIntentState = 0ULL;
        bool acquired = false;
        bool recopyRequired = false;
        // Same RT thread: exact image identity and final API outcome only.
        MotionServoHandoffSample handoff{};
        MotionPbcXSendSample pbcX{};
        bool handoffPending = false;
        PbcHomeProbeCommand homeProbe{};
        bool homeProbeSerializedExact = false;
        // EDM28 seals the actual serialized CSV velocities, not a drive ACK.
        std::uint64_t edmZStopTicket = 0ULL;
        std::uint64_t edmZStopSession = 0ULL;
        bool edmZStopFenceCurrent = false;
        std::uint64_t edmZFeedbackPublication = 0ULL;
        bool edmZGuardHeld = false;
        bool edmZSerializedExact = false;
        EDM94Serialized::Receipt edm94{};
        std::uint64_t rotarySpindleRun=0ULL,rotarySpindleTick=0ULL;
        std::array<std::uint64_t,3> rotarySpindleRequest{};
        std::array<double,3> rotarySpindleRpm{};
        std::array<std::int32_t,3> rotarySpindleHardwarePps{};
        bool rotarySpindleSerializedExact=false;

    };

    bool BeginServoOutputFrameAtSendPoint(
        ServoOutputFrameReservation& reservation) noexcept;
    bool FinalizeServoOutputFrameAtSendPoint(
        ServoOutputFrameReservation& reservation) noexcept;
    void EndServoOutputFrameAfterSend(
        ServoOutputFrameReservation& reservation,
        bool sendAttempted, bool sendSucceeded) noexcept;

    bool SealPbcXHomeProbeSerializedImage(ServoOutputFrameReservation& reservation,
        const std::uint8_t* ioMapBase, std::size_t ioMapSize,
        const std::uint8_t* serializedPayload, std::size_t payloadSize) noexcept;
    bool IsPbcXPhysicalHomeCaptureSupported(int axisIndex) const noexcept;
    bool IsPbcXZeroOnlyHomeIdentity(int axisIndex) const noexcept;
    bool IsPbcXStagedHomeEntryIdentity(int axisIndex) const noexcept;
    bool IsPbcXConfiguredDisabledHomeIdentity(int axisIndex) const noexcept;
    bool GetPbcXPhysicalHomeCaptureSnapshot(int axisIndex,
        MotionPbcHomeCaptureSnapshot& out) const noexcept;

    // Stage 11D.6:
    // Input arrives through MotionServoInputSnapshot.
    // Output remains the original ENI_ServoDrive.
    void UpdateServoState(
        ENI_ServoDrive& servo,
        AxisContext& axis,
        const MotionServoInputSnapshot& input);

    void ExportDebugInfo(SHM_AxisDebugInfo* outDebugArray, bool outputInMM = false);
    // Sole NC producer -> 250 us RT owner. These APIs grant no EDM/discharge
    // permit. An acknowledgement denotes local RT planner application only.
    bool SubmitEDMZFixtureRequest(const EDM28::Request& request) noexcept;
    void RequestEDMZFixtureStop(std::uint64_t session) noexcept;
    bool ReadEDMZFixtureFeedback(EDM28::Feedback& feedback) const noexcept;
    // EDM48 receipts describe the RT shadow consumer only. These wrappers
    // neither grant physical motion/discharge authority nor touch a planner.
    bool StartEDMRTShadow(const EDM46::Config& config, const EDM46::Scope& scope,
        const EDM46::IntentSnapshot& initial, std::uint64_t nowMs) noexcept;
    bool SubmitEDMRTShadow(const EDM47::Packet& packet) noexcept;
    void NeutralizeEDMRTShadow(std::uint64_t session, std::uint64_t commandFloor) noexcept;
    void CancelEDMRTShadow(std::uint64_t session) noexcept;
    bool ReadEDMRTShadowFeedback(EDM48::Feedback& feedback) const noexcept;
    bool StartEDMGapServoRT(const EDM55RT::Scope& scope, EDM55RT::Profile profile,
        std::uint64_t nowUs) noexcept;
    void CancelEDMGapServoRT(std::uint64_t session) noexcept;
    void RetireEDMGapServoRT(std::uint64_t session) noexcept;
    bool TouchEDMGapServoRT(std::uint64_t session, std::uint64_t nowUs) noexcept;
    bool ReadEDMGapServoRT(EDM55RT::Feedback& feedback) const noexcept;
    // EDM83 executes private retained geometry once per observed RT callback.
    // These wrappers never acquire an axis/output permit or publish a setpoint.
    bool StartEDMCNCRetainedRT(const EDM83RT::Request& request) noexcept;
    bool TouchEDMCNCRetainedRT(std::uint64_t session, std::uint64_t nowUs) noexcept;
    void CancelEDMCNCRetainedRT(std::uint64_t session) noexcept;
    bool ReadEDMCNCRetainedRT(EDM83RT::Feedback& feedback) const noexcept;
    std::uint64_t ReadEDMCNCRetainedRTLatestSession() const noexcept;
    // EDM84 owns a private coordinated calculator, never physical axes.
    bool StartEDMCoordinatedRT(const EDM84RT::Request& request) noexcept;
    bool TouchEDMCoordinatedRT(std::uint64_t session, std::uint64_t nowUs) noexcept;
    void CancelEDMCoordinatedRT(std::uint64_t session) noexcept;
    bool ReadEDMCoordinatedRT(EDM84RT::Feedback& feedback) const noexcept;
    std::uint64_t ReadEDMCoordinatedRTLatestSession() const noexcept;
    // EDM94 brackets ordinary NC six-axis moves with a physical CSV guard.
    bool SubmitEDM94Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease) noexcept;
    bool SubmitEDM95Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease) noexcept;
    bool ReadEDM95Feedback(EDM95::Feedback& feedback) const noexcept;
    bool SubmitEDM96Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease) noexcept;
    bool SubmitEDM97Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM98Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM99Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM100Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM102Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM104Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool SubmitEDM105Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile = EDM107::FlushProfile{},
        bool commonPath = false, bool b2CrossSegment = false, bool b2Repeat = false,
        bool curveRuntime = false, bool physicalCurve = false, bool curveB2 = false, bool modeFlush = false,
        EDM116::Fixture modeFixture = EDM116::Fixture::Canonical116) noexcept;
    bool SubmitEDM114Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile) noexcept;
    bool AuthorizeEDM114Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double, 6U>& increment, const std::array<double, 2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM107::FlushProfile& flushProfile) noexcept;
    bool SubmitEDM115Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile) noexcept;
    bool AuthorizeEDM115Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double, 6U>& increment, const std::array<double, 2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM107::FlushProfile& flushProfile,
        const EDM115::CompletionToken& previous) noexcept;
    bool SubmitEDM116Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile,
        EDM116::Fixture fixture = EDM116::Fixture::Canonical116) noexcept;
    bool AuthorizeEDM116Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM107::FlushProfile& flushProfile,
        const EDM116::Policy& policy, const EDM115::CompletionToken& previous) noexcept;
    bool ReadEDM116Feedback(EDM116::MotionFeedback& feedback) const noexcept;
    bool ReadEDM118Feedback(EDM118::Feedback& feedback) const noexcept;
    bool ReadEDM115Feedback(EDM115::MotionFeedback& feedback) const noexcept;
    bool ReadEDM114Feedback(EDM114::Feedback& feedback) const noexcept;
    bool SubmitEDM126Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning, unsigned fixture) noexcept;
    bool AuthorizeEDM126Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM114::Source& previousSource,
        std::uint64_t completionCommit, std::uint64_t completionTick) noexcept;
    bool ReadEDM126Feedback(EDM126::MotionFeedback& feedback) const noexcept;
    bool SubmitEDM128Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile, unsigned fixture) noexcept;
    bool AuthorizeEDM128Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM114::Source& previousSource,
        std::uint64_t completionCommit, std::uint64_t completionTick, const EDM116::Policy& policy) noexcept;
    bool ReadEDM128Feedback(EDM128::MotionFeedback& feedback) const noexcept;
    bool RequestEDM157Hold(const EDM157::Scope& scope, std::uint64_t holdSequence,
        std::uint64_t startFloor) noexcept;
    bool RequestEDM157Resume(const EDM157::Scope& scope, std::uint64_t holdSequence,
        std::uint64_t startSequence) noexcept;
    bool ReadEDM157Feedback(EDM157::Feedback& feedback) const noexcept;
    bool SubmitEDM169ManualVoltage(const EDM169::Request& request) noexcept;
    bool ReadEDM169ManualVoltageFeedback(EDM169::Feedback& feedback) const noexcept;
    bool SubmitEDM129Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile, unsigned fixture) noexcept;
    bool AuthorizeEDM129Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM114::Source& previousSource,
        std::uint64_t completionCommit, std::uint64_t completionTick, const EDM116::Policy& policy) noexcept;
    bool ReadEDM129Feedback(EDM129::MotionFeedback& feedback) const noexcept;
    bool ReadEDM173Feedback(EDM173::Feedback& out) const noexcept;
    bool ReadEDM173InputDiagnostic(EDM173InputDiag::Evidence& out) const noexcept;
    bool BeginReadEDM206History(EDM206::Header& out) noexcept;
    bool ReadEDM206HistoryRow(std::uint64_t session, unsigned index, EDM206::Row& out) const noexcept;
    void EndReadEDM206History(std::uint64_t session) noexcept;
    // Observation only; call from the PDO/Motion owner, never another thread.
    EDM174Comm::MotionObservation GetEDM174CommunicationObservation() const noexcept;
    std::uint32_t GetEDM173ImplementationRevision() const noexcept;
    bool SubmitEDM173Arm(std::uint64_t session,MotionExecutionEpoch epoch,const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning,const EDM107::FlushProfile& flush,const EDM130::Recipe& recipe,
        bool rejectBeforeArm = false) noexcept;
    bool SubmitEDM130Arm(std::uint64_t session, MotionExecutionEpoch epoch,
        const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile, unsigned fixture, const EDM130::Recipe& recipe,
        std::uint64_t manualRun = 0ULL, std::uint64_t manualCache = 0ULL,
        bool rejectBeforeArm = false, unsigned ordinarySequenceProgram = 0U) noexcept;
    bool AuthorizeEDM130Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM114::Source& previousSource,
        std::uint64_t completionCommit, std::uint64_t completionTick, const EDM116::Policy& policy,
        const EDM130::Recipe& recipe) noexcept;
    bool ReadEDM130Feedback(EDM128::MotionFeedback& feedback) const noexcept;
    bool ReadEDM96Feedback(EDM96::Feedback& feedback) const noexcept;
    bool ReadEDM109Feedback(EDM109::Feedback& feedback) const noexcept;
    bool ReadEDM113Feedback(EDM113::Feedback& feedback) const noexcept;
    bool ReadEDM110Feedback(EDM110::MotionFeedback& feedback) const noexcept;
    void RequestEDM94Finish(std::uint64_t session) noexcept;
    void RequestEDM94Cancel(std::uint64_t session, bool externalStop = false) noexcept;
    void RequestEDM94Release(std::uint64_t session) noexcept;
    bool RequestEDM94CompletionRelease(std::uint64_t session,
        MotionExecutionEpoch armEpoch, const MotionOwnerLease& armLease,
        MotionExecutionEpoch completionEpoch, const MotionOwnerLease& idleLease,
        MotionOwnerGeneration& expectedNoneGeneration) noexcept;
    bool RequestEDM233ParentRelease(std::uint64_t session,
        MotionExecutionEpoch armEpoch, const MotionOwnerLease& armLease) noexcept;
    bool RequestEDM234ParentRelease(std::uint64_t session, MotionExecutionEpoch armEpoch,
        const MotionOwnerLease& armLease, unsigned kind, std::uint64_t run, std::uint64_t cache,
        int parentPC, std::uint64_t parentDispatch, const EDM130::Recipe& recipe, unsigned program = 234U) noexcept;
    bool ReadEDM94Feedback(EDM94::Feedback& feedback) const noexcept;
    bool AuthorizeEDM94Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease) noexcept;
    bool AuthorizeEDM95Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM95::Permit& permit) noexcept;
    bool AuthorizeEDM96Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease) noexcept;
    bool AuthorizeEDM97Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM98Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM99Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM100Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM102Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM104Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning) noexcept;
    bool AuthorizeEDM105Feed(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, const EDM97::Tuning& tuning,
        const EDM107::FlushProfile& flushProfile = EDM107::FlushProfile{},
        bool commonPath = false, bool b2CrossSegment = false,
        const EDM110::CompletionToken& previous = EDM110::CompletionToken{}, bool b2Repeat = false,
        bool curveRuntime = false) noexcept;
    bool CheckEDM94Endpoint(const std::array<double, 6U>& nativePulse,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease) const noexcept;
    // EDM69 routes real A/D into a scoped RT calculation only. It owns no
    // axis/planner/output permit; zero feedback is not physical stop proof.
    // Boot thread only, once before the PDO callback starts. This binding
    // is independent of structured servo input/output qualification.
    bool BindEDMPhysicalSourceBeforeStart(EtherCatMaster* master) noexcept;
    bool StartEDMPhysicalServoRT(const EDM69::Request& request) noexcept;
    bool TouchEDMPhysicalServoRT(std::uint64_t session, std::uint64_t nowUs) noexcept;
    void CancelEDMPhysicalServoRT(std::uint64_t session) noexcept;
    bool ReadEDMPhysicalServoRT(EDM69::Feedback& feedback) const noexcept;
    std::uint64_t ReadEDMPhysicalServoRTLatestSession() const noexcept;
    //單軸運動 API--------------------------------------------------------------------


    void InitAxis(AxisContext& axis, double resolution = 16777216.0);// 初始化軸參數 (如解析度、預設極限、PID)
    void InitSmoothBuffer(AxisContext& axis, double smoothTime_ms);// 初始化 S-Curve 平滑濾波緩衝區
    bool MoveToPosition(AxisContext& axis, double targetPos, double targetVel, double acc_time, double dec_time);// 下達 P2P 絕對位置移動指令 (Trapezoidal 梯形加減速)
    void VelocityMove(AxisContext& axis, double velocity, double acc_time = 0.0);// 下達速度模式指令 (用於放電或手動連續移動)
    void MPGMove(AxisContext& axis, double targetPos, double maxVel, double acc_time, double dec_time);
    void StopMove(AxisContext& axis, double dec_time = 0.0);// 正常減速停止單軸
    void EmergencyStop(AxisContext& axis);// 單軸急停 (瞬間鎖死，清空緩衝區)
    void ResetFault(AxisContext& axis);// 清除單軸故障狀態 (Reset Error)
    // 🌟 [新增] 全部軸警報解除與群組重置
    void ResetAllFaults();
    void SetAxisFeedrateOverride(int axisIndex, double overrideRatio);// 設定單軸的進給倍率 (0.0 ~ 1.0)
    void Stop(AxisContext& axis);// 簡易停止 API

    // 所有存在的實體軸立即急停
//
// 與 EmergencyStopGroup() 不同：
//
// EmergencyStopGroup()
//     只處理目前 interpolation group
//
// EmergencyStopAllAxes()
//     不管目前 Group 如何設定，
//     直接掃描所有 AxisContext，
//     只要 axis.isExist == true 就 EmergencyStop。
//
// 用於：
//     PLC C5 Emergency Stop
//     全機安全急停
// =========================================================
    void EmergencyStopAllAxes();


    //核心運算更新--------------------------------------------------------------------
    // 單軸運動狀態機更新 (必須在即時迴圈 1ms/250us 中呼叫)
    template <typename DriveType>
    void UpdateMotion(
        DriveType& servo,
        AxisContext& axis,
        const MotionServoInputSnapshot& input);


    //輔助工具--------------------------------------------------------------------
    static double RpmToPps(double rpm, double resolution); // RPM 轉 Pulse/Sec
    static double PpsToRpm(double pps, double resolution); // Pulse/Sec 轉 RPM

    // 🌟 [新增] 將使用者直覺的 mm/min 或 deg/min 轉換為 PPS
    static double UnitPerMinToPps(double unitPerMin, double resolution, double finalLead);

    // G81 HOME Feedback / Machine Coordinate Helpers
    double GetRawLogicalPositionPulse(const AxisContext& axis) const;
    bool GetDriveTouchProbeFunction(int axisIndex, uint16_t& functionValue) const;
    bool GetDriveTouchProbeData(int axisIndex, uint16_t& status, int32_t& capturedPosition) const;
    bool SetDriveTouchProbeFunction(int axisIndex, uint16_t value);
    double ConvertDriveCaptureToRawLogicalPulse(int axisIndex, int32_t capturedPosition, HomeReferenceSource source) const;
    bool ApplyMachineHome(
        AxisContext& axis,
        double capturedReferencePulse,
        double homeOffsetUnit,
        const MotionOwnerLease& ownerLease,
        bool* outDeferred = nullptr, std::uint64_t homeCaptureToken = 0ULL);

    //多軸插補功能區塊--------------------------------------------------------------------

    void LineMove(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        double targetVel,
        double acc_time,
        double dec_time,
        BufferMode mode = BufferMode::ABORTING,
        MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED);// 直線插補指令
    void ArcMove(const std::vector<int>& axes, const std::vector<double>& targetPos, const std::vector<double>& centerPos, int dir, double targetVel, double acc_time, double dec_time, BufferMode mode = BufferMode::ABORTING);// 圓弧插補指令
    void UpdateInterpolation();// 插補群組更新 (計算虛擬主軸並分配位移給實體軸)
    void InitVirtualAxisSmooth(int windowSize); // 初始化虛擬主軸的 S-Curve 平滑設定
    void LoadNextCommand(bool cncBoundaryCrossing = false);// 從指令佇列 (Queue) 載入下一段任務  
    void GetDirectionVector(const MotionCommand& cmd, double startX, double startY, double& vx, double& vy);// 取得當前路徑的方向向量
    void StopGroup();// 插補群組整體停止與急停
    void EmergencyStopGroup();//緊急停止

    // 插補群組進給倍率與路徑模式設定 (Exact Stop / Continuous)
    void SetGroupFeedrateOverride(double overrideRatio);// 設定當前插補群組進給倍率
    void SetGroupPathMode(PathMode mode);// 設定當前插補群組與路徑模式
    PathMode GetGroupPathMode() const;// 取得當前插補群組與路徑模式
    void UpdatePathServoVelocity(double velocity_pps);//更新外部速度  
    void EnableHistoryBuffer(bool enable);//時光機模式開關

    // ========================================================================
    // Stage NC-0.1B / NC-0.1C - Execution Epoch + Fixed SPSC Command Ring
    //
    // BeginNewExecutionEpoch() 只發布新的執行世代，不讓 NC 執行緒清除
    // Command Queue。舊世代淘汰由 250 us Motion Runtime 完成。
    //
    // NC-0.1C 已將跨執行緒 std::deque 替換為固定容量 SPSC Ingress。
    // B2 push-front 行為由 RT 專用固定 Replay 區承接。
    // ========================================================================
    MotionExecutionEpoch BeginNewExecutionEpoch(MotionCommandSource source) noexcept;
    MotionExecutionEpoch GetCurrentExecutionEpoch() const noexcept;

    // ====================================================================
    // Stage NC-0.1E - Motion Owner Lease Arbitration
    //
    // 所有權狀態使用單一 64-bit Atomic Packed State，避免 Owner 與
    // Generation 分開讀取時產生撕裂快照。
    // ====================================================================
    // True means the exact program lease was consumed by the handoff CAS.
    // NC must still honor any concurrent Alarm / Reset before publishing P_END.
    bool TryEnterProgramEndIdleHold(
        const MotionOwnerLease& programLease,
        MotionExecutionEpoch expectedEpoch) noexcept;

    // CJ FIX1: the 250 us/Priority-80 PDO owner is the sole producer.
    // Only the existing Priority-50 HMI 1000 ms task may consume this queue.
    enum class IdleHoldDiagnosticEventType : std::uint8_t
    {
        ACTIVE, REFERENCE, RELEASED, CANCELLED, FAILED,
        FOLLOWING_ERROR_SAMPLE, FOLLOWING_ERROR_CONTROL
    };
    enum class IdleHoldDiagnosticReason : std::uint8_t
    {
        NONE, GRANT_MISMATCH, AUTHORITY_CHANGED, GROUP_OR_MAPPING_CHANGED,
        SERVO_OR_MODE_LOST, AXIS_MAPPING, COMPENSATION_NONFINITE,
        UNSUPPORTED_SCOPE, AXIS_CONFIG_OR_COMMAND, VELOCITY_CONVERSION,
        REFERENCE_OR_CONFIG_CHANGED, FOLLOWING_ERROR, CORRECTION_NONFINITE,
        TRAVEL_LIMIT
    };
    struct IdleHoldDiagnosticEvent
    {
        std::uint64_t runtimeTick = 0ULL;
        std::uint64_t cmdPulseBits = 0ULL;
        std::uint64_t actPulseBits = 0ULL;
        std::uint64_t windowPulseBits = 0ULL;
        std::uint64_t sequence = 0ULL;
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerGeneration generation = 0U;
        std::uint32_t mask = 0U;
        // BASE57 event-local payload: ACTIVE = captured rotary mask;
        // REFERENCE = native unit (1 mm, 2 degree); RELEASED = next generation;
        // FOLLOWING_ERROR_CONTROL retains the BASE48_DIAG1 reverse flag.
        MotionOwnerGeneration nextGeneration = 0U;
        std::int32_t axisIndex = -1;
        MotionOwner owner = MotionOwner::NONE;
        MotionOwner nextOwner = MotionOwner::NONE;
        IdleHoldDiagnosticEventType eventType = IdleHoldDiagnosticEventType::ACTIVE;
        IdleHoldDiagnosticReason reason = IdleHoldDiagnosticReason::NONE;
    };
    static_assert(sizeof(IdleHoldDiagnosticEvent) == 64U &&
        std::is_trivially_copyable<IdleHoldDiagnosticEvent>::value,
        "CJ FIX1 diagnostic records must remain fixed 64-byte POD values.");
    // BASE57 native-unit policies. ROTPID_TUNE1: unloaded position-rotary
    // trial only: 0.5 deg/s hold correction cap; linear remains 0.1 mm/s.
    // Position reference, P-only law, owner fences and fault multiplier stay
    // unchanged. This is not a positioning-accuracy or hardware PASS claim.
    static constexpr std::uint32_t IDLE_HOLD_LINEAR_CAP_UMS = 100U;
    static constexpr std::uint32_t IDLE_HOLD_ROTARY_CAP_MDEGS = 500U;
    // Six-axis ACTIVE + 6 REFERENCE + 3 fault records + RELEASED = 11.
    // Keep the existing drain work bound; only fixed burst storage grows.
    static constexpr std::size_t IDLE_HOLD_DIAGNOSTIC_CAPACITY = 16U;
    static constexpr std::size_t IDLE_HOLD_DIAGNOSTIC_DRAIN_BUDGET = 8U;
    bool TryPopIdleHoldDiagnostic(IdleHoldDiagnosticEvent& event) noexcept;
    std::uint32_t GetIdleHoldDiagnosticDroppedCount() const noexcept;

    // ROTPID_DIAG1: immutable low-rate observations, never control authority.
    bool TryPopRotaryPidDiagnostic(RotaryPidDiagnosticEvent& event) noexcept
    { return m_rotaryPidDiagnosticMonitor.TryPop(event); }
    std::uint32_t GetRotaryPidDiagnosticDroppedCount() const noexcept
    { return m_rotaryPidDiagnosticMonitor.Dropped(); }
    bool TryPopServoHandoffDiagnostic(MotionServoHandoffEvent& event) noexcept
    { return m_servoHandoffMonitor.TryPop(event); }
    std::uint32_t GetServoHandoffDiagnosticDroppedCount() const noexcept
    { return m_servoHandoffMonitor.Dropped(); }

    bool TryPopPbcXSendDiagnostic(MotionPbcXSendEvent& event) noexcept
    { return m_pbcXSendAudit.TryPop(event); }
    std::uint32_t GetPbcXSendDiagnosticDroppedCount() const noexcept
    { return m_pbcXSendAudit.Dropped(); }

    bool TryPopPbcReferenceDiagnostic(MotionPbcReferenceEvent& event) noexcept
    { return m_pbcReferenceAudit.TryPop(event); }
    std::uint32_t GetPbcReferenceDiagnosticDroppedCount() const noexcept
    { return m_pbcReferenceAudit.Dropped(); }

    // DC: RT-only producer / existing HMI diagnostic-task consumer.
    enum class CncP1Event : std::uint8_t
    {
        LOAD_EMPTY, LOAD_READY, PROMOTED, KEEP_STOP, LEAVE
    };
    enum class CncP1Reason : std::uint8_t
    {
        NONE, SCOPE, AUTHORITY, MAPPING, GEOMETRY, DIRECTION,
        SPEED, CURRENT_DISTANCE, NEXT_DISTANCE, TERMINAL, OVERRIDE, QUEUE_EMPTY
    };
    struct CncP1Diagnostic
    {
        std::uint64_t runtimeTick = 0ULL;
        std::uint64_t sequence = 0ULL;
        MotionSegmentId segment = MOTION_SEGMENT_ID_INVALID;
        MotionSegmentId nextSegment = MOTION_SEGMENT_ID_INVALID;
        double commandVelocity = 0.0;
        double endVelocity = 0.0;
        double remainingPulse = 0.0;
        double nextLengthPulse = 0.0;
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerGeneration generation = 0U;
        std::int32_t sourcePC = -1;
        std::int32_t nextSourcePC = -1;
        std::uint32_t axisMask = 0U;
        std::uint32_t queueDepth = 0U;
        MotionOwner owner = MotionOwner::NONE;
        CncP1Event event = CncP1Event::LOAD_EMPTY;
        CncP1Reason reason = CncP1Reason::NONE;
        bool tickValid = false;
    };
    static_assert(sizeof(CncP1Diagnostic) <= 112U &&
        std::is_trivially_copyable<CncP1Diagnostic>::value,
        "DC diagnostic record must remain bounded POD.");
    static constexpr std::size_t CNC_P1_DIAGNOSTIC_CAPACITY = 32U;
    static constexpr std::size_t CNC_P1_DIAGNOSTIC_DRAIN_BUDGET = 16U;
    bool TryPopCncP1Diagnostic(CncP1Diagnostic& event) noexcept;
    std::uint32_t GetCncP1DiagnosticDroppedCount() const noexcept;

    // DF: per-command feed limits on the existing immutable opt-in and RT/HMI boundary.
    enum class CncFeedPlanEvent : std::uint8_t { LOAD, EXTEND, KEEP_PLAN, LEAVE, BLEND_ENTER, PREFIX_FAST, PREFIX_BRAKE, PREFIX_AUTHORED };
    enum class CncFeedPlanStop : std::uint8_t
    {
        QUEUE_END, HORIZON, SCOPE, AUTHORITY, MAPPING, DIRECTION, SPEED, SHORT_SEGMENT, LATE
    };
    struct CncFeedPlanDiagnostic
    {
        std::uint64_t runtimeTick = 0ULL, sequence = 0ULL;
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        MotionSegmentId lastSegment = 0ULL;
        double commandVelocity = 0.0, outputVelocity = 0.0;
        double endVelocity = 0.0, cruiseVelocity = 0.0, remainingPulse = 0.0;
        double horizonPulse = 0.0, reservePulse = 0.0;
        std::array<double, 4U> exitVelocity{};
        std::array<double, 4U> nominalVelocity{};
        std::array<double, 4U> limitedVelocity{};
        double radiusPulse = 0.0;
        // DJ/DK companion diagnostics: pulse distances and distinct prefix caps.
        double prefixLengthPulse = 0.0, prefixRemainingPulse = 0.0;
        double prefixLimitPPS = 0.0, prefixReservePulse = 0.0;
        double authoredPrefixPPS = 0.0;
        double entryCarry = 0.0;
        MotionSegmentId handoffFrom = MOTION_SEGMENT_ID_INVALID;
        std::uint32_t horizon = 0U, axisMask = 0U, circleMask = 0U, blendMask = 0U;
        CncFeedPlanEvent event = CncFeedPlanEvent::LOAD;
        CncFeedPlanStop stop = CncFeedPlanStop::QUEUE_END;
        bool tickValid = false;
    };
    static constexpr std::size_t CNC_FEED_PLAN_CAPACITY = 32U;
    static constexpr std::size_t CNC_FEED_PLAN_DRAIN_BUDGET = 16U;
    // DK adds one 8-byte authored cap; capacity and drain budget stay fixed.
    static_assert(sizeof(CncFeedPlanDiagnostic) <= 296U &&
        std::is_trivially_copyable<CncFeedPlanDiagnostic>::value, "DK bounded diagnostic value.");
    bool TryPopCncFeedPlanDiagnostic(CncFeedPlanDiagnostic& event) noexcept;
    std::uint32_t GetCncFeedPlanDiagnosticDroppedCount() const noexcept;

    bool TryAcquireMotionOwner(
        MotionOwner requestedOwner,
        MotionOwnerLease& outLease) noexcept;

    bool TryTransferMotionOwner(
        const MotionOwnerLease& currentLease,
        MotionOwner requestedOwner,
        MotionOwnerLease& outLease) noexcept;

    bool ReleaseMotionOwner(
        const MotionOwnerLease& lease) noexcept;

    enum class SafetyMotionOwnerReleaseStatus : std::uint8_t
    {
        DEFERRED = 0,
        RELEASED,
        SUPERSEDED
    };

    SafetyMotionOwnerReleaseStatus TryReleaseSafetyMotionOwner(
        const MotionOwnerLease& lease,
        const MotionNCResetSafetyReleaseAuthorization&
        authorization) noexcept;

    bool ReleaseSafetyMotionOwner(
        const MotionOwnerLease& lease,
        const MotionNCResetSafetyReleaseAuthorization&
        authorization) noexcept;

    MotionOwnerLease TakeSafetyMotionOwner() noexcept;
    MotionOwnerLease BeginNewSafetyMotionOwnerGeneration() noexcept;
    MotionOwnerLease ContinueNewSafetyMotionOwnerGeneration(
        MotionOwnerGeneration entryGeneration) noexcept;

    enum class ResetSafetyAuthorityStatus : std::uint8_t
    {
        DEFERRED = 0,
        ACQUIRED,
        SUPERSEDED
    };

    struct ResetSafetyAuthorityResult
    {
        ResetSafetyAuthorityStatus status =
            ResetSafetyAuthorityStatus::DEFERRED;
        MotionOwnerLease lease{};
        std::uint32_t requestTicket = 0U;
        std::uint64_t provenanceGeneration = 0ULL;
    };

    // Hold the process-data send seam closed for the complete multi-scan
    // Reset transaction.  This is deliberately independent of the drain-
    // acknowledgement publisher count: RT may continue consuming and
    // acknowledging pre-existing safety work while every outgoing target
    // velocity remains zero.
    std::uint64_t BeginResetSafetyOutputHold() noexcept;
    bool IsResetSafetyOutputHoldEstablished() const noexcept;
    void EndResetSafetyOutputHold() noexcept;
    std::uint64_t MarkResetSafetyOperatorEdge() noexcept;
    std::uint64_t GetSafetyProvenanceGeneration() const noexcept;

    // Capture the exact post-drain baseline belonging to the operator edge.
    // Every independent Safety producer increments provenance before it can
    // publish a ticket/mailbox, including a producer temporarily blocked by
    // a frame reservation.  Therefore a changed provenance is superseding,
    // while a stable in-flight baseline is merely deferred.
    ResetSafetyAuthorityResult TryCaptureResetSafetyAuthorityBaseline(
        std::uint64_t expectedProvenanceGeneration) const noexcept;

    // A Reset may retry through bounded reservations, but it may never join
    // an unrelated SAFETY incident which happened to acquire the next owner
    // generation.  Ticket + provenance form the persistent Reset identity.
    ResetSafetyAuthorityResult ContinueResetSafetyMotionOwnerGeneration(
        MotionOwnerGeneration entryGeneration,
        std::uint32_t baselineTicket,
        std::uint32_t requestTicket,
        std::uint64_t expectedProvenanceGeneration) noexcept;

    ResetSafetyAuthorityStatus TryGetResetSafetyMotionOwnerEpoch(
        const MotionOwnerLease& safetyLease,
        std::uint32_t requestTicket,
        std::uint64_t expectedProvenanceGeneration,
        MotionExecutionEpoch& executionEpoch) const noexcept;

    bool TryGetSafetyMotionOwnerEpoch(
        const MotionOwnerLease& safetyLease,
        MotionExecutionEpoch& executionEpoch) const noexcept;

    MotionOwnerLease GetMotionOwnerLease() const noexcept;

    bool IsMotionOwnerLeaseCurrent(
        const MotionOwnerLease& lease) const noexcept;

    MotionOwner GetMotionOwner() const noexcept
    {
        return GetMotionOwnerLease().owner;
    }

    // ====================================================================
    // Stage NC-0.1F - 10 ms Control -> 250 us RT Axis Command Mailbox
    //
    // These producer APIs never mutate AxisContext. They only publish a
    // fixed-size command carrying the current Owner + Generation lease.
    // ====================================================================
    bool SubmitAxisMoveToPosition(
        int axisIndex,
        double targetPosition,
        double targetVelocity,
        double accelerationTime,
        double decelerationTime,
        bool useShortestPath,
        MotionCommandSource source,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr, bool* outIngressBusy = nullptr) noexcept;

    bool SubmitAxisVelocityMove(
        int axisIndex,
        double targetVelocity,
        double accelerationTime,
        MotionCommandSource source,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr, bool* outIngressBusy = nullptr) noexcept;

    bool SubmitAxisMPGMove(
        int axisIndex,
        double targetPosition,
        double maximumVelocity,
        double accelerationTime,
        double decelerationTime,
        MotionCommandSource source,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

    bool SubmitAxisStopMove(
        int axisIndex,
        double decelerationTime,
        MotionCommandSource source,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr, bool* outIngressBusy = nullptr) noexcept;

    bool SubmitApplyMachineHome(
        int axisIndex,
        double capturedReferencePulse,
        double homeOffsetUnit,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence& outSequence, bool* outIngressBusy = nullptr,
        std::uint64_t homeCaptureToken = 0ULL) noexcept;

    bool SubmitDriveTouchProbeFunction(
        int axisIndex,
        std::uint16_t value,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr, bool* outIngressBusy = nullptr) noexcept;

    // Single consumer: NCPLC 10 ms task.
    void ProcessAxisCommandResults() noexcept;
    bool TryGetAxisCommandResult(
        MotionAxisCommandSequence sequence,
        MotionAxisCommandResult& outResult) const noexcept;

    // Safety / recovery producers publish atomic requests. The actual
    // AxisContext mutation is applied by UpdateInterpolation().
    void RequestEmergencyStopAllAxes() noexcept;
    void RequestAxisFaultReset(int axisIndex) noexcept;
    void RequestResetAllFaults() noexcept;
    void RequestStopGroup() noexcept;

    // RESET during an ordinary G00/G01 is deliberately staged without
    // taking the SAFETY owner in the NC/HMI thread.  The 250 us runtime
    // consumes this mailbox and creates the exact ticket/epoch together,
    // preventing a one-frame zero-PDO seam before controlled deceleration.
    bool RequiresPbcXRetainRecoveryReset() const noexcept;
    void RequestResetControlledStop() noexcept;
    ResetControlledStopPhase GetResetControlledStopPhase() const noexcept;
    bool ConsumeCompletedResetControlledStop() noexcept;
    bool RetireSupersededResetControlledStop() noexcept;

    // Stage NC-0.2J.2:
    // NC Reset already owns and publishes one execution Epoch before it
    // submits the deferred safety work. Correlate ResetAllFaults + StopGroup
    // as one RT-consumed batch so the leaf operations do not publish a
    // second Epoch for the same lifecycle interruption.
    void RequestResetSafetyBatch(
        MotionExecutionEpoch publishedEpoch,
        bool requestResetAllFaults) noexcept;

    // Strict Reset-only parent -> child publication.  The batch is visible
    // only when the exact SAFETY lease, Epoch, ticket and provenance still
    // belong to this Reset; an unrelated Safety producer is SUPERSEDED.
    ResetSafetyAuthorityResult RequestExactResetSafetyBatch(
        MotionExecutionEpoch publishedEpoch,
        const MotionOwnerLease& safetyLease,
        std::uint32_t parentRequestTicket,
        std::uint64_t expectedProvenanceGeneration,
        bool requestResetAllFaults) noexcept;

    // CQ_FIX1: read actual safety intent separately from a normal source epoch publication.
    bool HasPendingSafetyIntent() const noexcept;
    bool HasPendingSafetyOrRecoveryRequests() const noexcept;

    std::size_t GetAxisCommandMailboxDepth() const noexcept
    {
        return m_axisCommandChannel.command_size();
    }

    std::size_t GetAxisCommandResultDepth() const noexcept
    {
        return m_axisCommandChannel.result_size();
    }

    std::uint64_t GetAxisCommandQueueFullCount() const noexcept
    {
        return m_axisCommandQueueFullCount.load(std::memory_order_relaxed);
    }

    std::uint64_t GetAxisCommandResultOverflowCount() const noexcept
    {
        return m_axisCommandResultOverflowCount.load(std::memory_order_relaxed);
    }

    std::uint64_t GetStaleCommandDiscardCount() const noexcept
    {
        return m_staleCommandDiscardCount.load(std::memory_order_relaxed);
    }

    std::uint64_t GetMotionOwnerConflictRejectCount() const noexcept
    {
        return m_motionOwnerConflictRejectCount.load(
            std::memory_order_relaxed);
    }

    // --------------------------------------------------------------------
    // Stage NC-0.1D - Motion Feedback Consumer API
    //
    // 僅允許 NC 10 ms Task 作為單一 Consumer 呼叫 TryReadMotionFeedback。
    // HMI / API 不可另外 Pop Ring；正式跨執行緒 Snapshot 會在後續
    // SHM / Diagnostics 階段建立。
    // --------------------------------------------------------------------
    bool TryReadMotionFeedback(
        MotionFeedbackEvent& event) noexcept
    {
        return
            m_motionFeedbackChannel.NcTryConsumeFeedback(event);
    }

    std::size_t GetMotionFeedbackDepth() const noexcept
    {
        return
            m_motionFeedbackChannel.feedback_size();
    }

    std::size_t GetMotionFeedbackProducerNoticeDepth() const noexcept
    {
        return
            m_motionFeedbackChannel.producer_notice_size();
    }

    static constexpr std::size_t GetMotionFeedbackCapacity() noexcept
    {
        return
            MOTION_FEEDBACK_EVENT_CAPACITY;
    }

    std::uint64_t GetMotionFeedbackOverflowCount() const noexcept
    {
        return
            m_motionFeedbackOverflowCount.load(
                std::memory_order_relaxed);
    }

    std::uint64_t GetMotionFeedbackProducerNoticeOverflowCount() const noexcept
    {
        return
            m_motionFeedbackProducerNoticeOverflowCount.load(
                std::memory_order_relaxed);
    }

    MotionFeedbackSequence GetLastPublishedMotionFeedbackSequence() const noexcept
    {
        return
            m_lastPublishedMotionFeedbackSequence.load(
                std::memory_order_acquire);
    }

    MotionSegmentId GetLastDroppedMotionFeedbackSegmentId() const noexcept
    {
        return
            m_lastDroppedMotionFeedbackSegmentId.load(
                std::memory_order_relaxed);
    }

    MotionFeedbackType GetLastDroppedMotionFeedbackType() const noexcept
    {
        return
            m_lastDroppedMotionFeedbackType.load(
                std::memory_order_relaxed);
    }

    void SetPendingCommandSource(MotionCommandSource source) noexcept
    {
        m_pendingCommandSource.store(
            source,
            std::memory_order_release);
    }

    void BeginProgramBlockMotionCapture() noexcept;
    MotionProgramBlockCapture EndProgramBlockMotionCapture() noexcept;
    void EndProgramBlockMotionCapture(MotionProgramBlockCapture& output) noexcept;

    //設定空間座標旋轉 (參數：開關, 旋轉中心X,Y,Z, 繞Z軸旋轉角度, 繞Y軸旋轉角度, 繞X軸旋轉角度)
    void SetCoordinateTransform(bool enable, double ox, double oy, double oz, double yaw_deg, double pitch_deg, double roll_deg);



    //跳躍排渣區塊--------------------------------------------------------------------
    void TriggerPathJump(JumpMode mode, const std::vector<JumpSegment>& retract, const std::vector<JumpSegment>& approach, double dwellTime_ms, int b1_axis = 2);

    void TriggerCenterJump_B3(double targetCenterX, double targetCenterY, double targetCenterZ, double jumpVecX, double jumpVecY, double jumpVecZ, const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece, double dwellTime_ms);
    void TriggerOrbitalJump_B4(double upperCx, double upperCy, double upperCz, double vx, double vy, double vz, const std::vector<JumpSegment>& toUpper, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece, double dwellTime_ms);
    void FinalizeSafePath(const std::vector<JumpSegment>& retract, std::vector<JumpSegment>& approach);//腳本安全保護控制


    void TriggerPause_B0(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript);//觸發 B0 暫停 
    void TriggerPause_B1(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript, int axisIndex, double dir); //觸發 B1 暫停(指定單一軸與方向)
    void TriggerPause_B2(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript); // 觸發 B2 暫停 (沿原路徑倒退嚕)
    void TriggerPause_B3(double cx, double cy, double cz, double vx, double vy, double vz, const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece);
    void TriggerPause_B4(double upperCx, double upperCy, double upperCz, double vx, double vy, double vz, const std::vector<JumpSegment>& toUpper, const std::vector<JumpSegment>& toApex, const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece);




    void Process_B2_Approach_Planner(double dt);
    void Process_Forward_Crossing();
    void TriggerPauseResume(int alignMode, double alignVel, int firstStageMask);//觸發復歸 (只需給對齊模式和速度)

    // 🌟 讓 NC 系統查詢底層插補狀態
    bool IsGroupQueueFull() const noexcept
    {
        return
            m_Group.cmdQueue.producer_full() ||
            m_Group.cmdQueue.ingress_size() >=
            MOTION_COMMAND_PREREAD_HIGH_WATERMARK;
    }

    bool IsGroupDone() const noexcept
    {
        return
            !m_Group.isActive &&
            m_Group.cmdQueue.empty();
    }

    // 🌟 [新增]：檢查群組是否「完全靜止」(包含煞車滑行結束)
    bool IsGroupStandstill() const;

    // Stage NC-0.2I.2：Feed Hold 保留目前 Segment 與 Future Queue，
    // 因此使用速度型停止快照，不要求 IsGroupDone()。
    MotionFeedHoldStopSnapshot GetFeedHoldStopSnapshot() const noexcept;

    bool BindPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
        const MotionOwnerLease& lease, double lengthMM, double lengthPulse,
        double distanceMM, double feedMMMin,
        double sourceFeedMMMin, double sourceVelocityPPS,
        std::uint32_t cycleLimit = 1U,
        const MotionPathCoreHoldExcursionView* crossView = nullptr,
        bool requireReturnAuthorization = false) noexcept;
    bool RequestPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
        const MotionOwnerLease& lease, MotionNCSettleRequestSequence settleSequence) noexcept;
    bool CommitPathCoreHoldExcursion(const MotionExecutionIdentity& identity,
        const MotionOwnerLease& lease, MotionNCSettleRequestSequence settleSequence) noexcept;
    void CancelPathCoreHoldExcursion() noexcept;
    MotionPathCoreHoldExcursionSnapshot GetPathCoreHoldExcursionSnapshot() const noexcept;


    // Stage NC-0.2J.4：跨執行緒讀取只碰 Atomic Publication Bank，
    // 不直接讀取 250 us Motion owner 的 AxisContext / Group 狀態。
    MotionStopSettleSnapshot GetStopSettleSnapshot() const noexcept;
    MotionStopSettleCounters GetStopSettleCounters() const noexcept;
    void GetStopSettleEvidence(
        MotionStopSettleSnapshot& snapshot,
        MotionStopSettleCounters& counters) const noexcept;

    // Stage NC-0.2J.6.1: coherent read-only RT evidence.  This is diagnostic
    // shadow data only; it does not acknowledge an Alarm to the operator and
    // does not release the SAFETY owner.
    bool TryGetEmergencyStopEvidence(
        MotionEmergencyStopEvidence& evidence,
        MotionEmergencyStopCounters& counters) const noexcept;

    MotionEmergencyStopEpochInvalidationEvidence
        GetEmergencyStopEpochInvalidationEvidence() const noexcept;

    MotionStartupLagArmingEvidence
        GetStartupLagArmingEvidence() const noexcept;

    MotionP1HandoverSafetySnapshot
        GetP1HandoverSafetySnapshot() const noexcept;

    // Stage NC-0.2K.6: aggregate producer/consumer evidence only.  The command
    // field is not authoritative for planning until a later controlled cutover.
    MotionCommandPathModeTransportSnapshot
        GetCommandPathModeTransportSnapshot() const noexcept;

    // Stage NC-0.2K.6.2: coherent producer-side proof that the G00 queue-tail
    // endpoint was published only after the matching ingress command.
    MotionQueueTailTransactionSnapshot
        GetQueueTailTransactionSnapshot() const noexcept;

    MotionLifecycleCommitReservationSnapshot
        GetLifecycleCommitReservationSnapshot() const noexcept;

    bool AcknowledgeP1MappingIntegrityAlarmRequest(
        std::uint64_t requestSequence) noexcept;

    // Stage NC-0.2J.5 Runtime validity seam.  EtherCAT Runtime calls this
    // exactly once for every PDO cycle, including the first invalid cycle.
    void ObserveNCSettleRuntimeCycle(
        std::uint64_t runtimeCycleTick,
        bool pdoCycleValid) noexcept;

    MotionNCSettleRequestSequence RequestFeedHoldNCSettle(
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease) noexcept;

    MotionNCSettleRequestSequence RequestResetNCSettleAndRebase(
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease,
        std::uint32_t safetyRequestTicket,
        std::uint64_t safetyProvenanceGeneration,
        const MotionNCResetExecutionState& executionState,
        bool unsupportedFaultOrEstop) noexcept;

    bool TryGetNCSettleEvidence(
        MotionNCSettleProfile profile,
        MotionNCSettleSnapshot& snapshot,
        MotionNCSettleCounters& counters) const noexcept;

    bool IsGroupNCSettled() const noexcept;
    bool IsGroupNCDrained() const noexcept;
    bool HasExactExecutionDrainAcknowledgement(
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease) const noexcept;
    // Program start is intentionally stricter than a generic execution
    // drain: the next program must not consume its first motion command until
    // the same 250 us RT publication proves every physical axis satisfies the
    // exact incoming-axis readiness predicate used by LoadNextCommand().
    // Raw derivative velocity and IDLE PID PDO holding correction are advisory
    // only; treating either as a hard admission condition would permanently
    // block a stationary servo with noisy feedback.  This is a read-only
    // admission gate; it never clears, recovers, or weakens a safety condition.
    bool HasExactProgramStartQuiescenceAcknowledgement(
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease) const noexcept;
    MotionNCResetRebaseAck GetNCResetRebaseAck() const noexcept;
    bool TryGetNCResetSafetyReleaseAuthorization(
        MotionNCSettleRequestSequence requestSequence,
        MotionNCResetSafetyReleaseAuthorization&
        outAuthorization) const noexcept;


    // ========================================================
    // 🌟 [新增] 軸警報檢查 API
    // ========================================================

    // 檢查「全系統」所有啟用的實體軸，是否有任何一軸發生錯誤 (Fault)
    bool IsAnyAxisFaulted() const;

    // 檢查「當前插補群組」內正在參與同動的軸，是否有發生錯誤 (Fault)
    bool IsGroupFaulted() const;

    bool IsGroupEmergencyStopped() const;
    // 🌟 修正版：精準對應軸索引的目標座標抓取 API
    bool GetExecutingTargetMCS(double* outTarget_mm) const {
        // 如果機台靜止或插補器未啟用，回傳 false
        if (IsGroupStandstill() || !m_Group.isActive) {
            return false;
        }

        // 1. 先將 8 個軸的目標全部預設為「當前真實位置」
        // 這樣沒有參與移動的軸相減時，DTG 才會是完美的 0.0！
        for (int i = 0; i < 8; i++) {
            if (m_pContexts != nullptr && i < m_pContexts->size() && (*m_pContexts)[i].isExist) {
                const AxisContext& axis = (*m_pContexts)[i];
                double pulsePerUnit = axis.resolution_PPR / axis.finalLead;
                outTarget_mm[i] = axis.GetNominalActualPositionPulse() / pulsePerUnit;
            }
            else {
                outTarget_mm[i] = 0.0;
            }
        }

        // 2. 走訪當前這張單子 (currentCmd) 裡面「真正有參與移動」的軸
        for (int j = 0; j < m_Group.currentCmd.axisCount; j++) {
            // 取出第 j 個參與軸的物理編號 (例如: j=0 時, axisIdx可能是 1 (Y軸))
            int axisIdx = m_Group.currentCmd.axisIndices[j];

            // 安全邊界檢查：確保軸編號在合法範圍內
            if (m_pContexts != nullptr && axisIdx >= 0 && axisIdx < m_pContexts->size() && (*m_pContexts)[axisIdx].isExist) {
                const AxisContext& axis = (*m_pContexts)[axisIdx];

                // 單位換算：Pulse -> mm (或 deg)
                double pulsePerUnit = axis.resolution_PPR / axis.finalLead;

                // 🌟 關鍵修復點 🌟
                // 必須用 j 去拿 currentCmd.targetPos[j]，然後存到 outTarget_mm[axisIdx]！
                outTarget_mm[axisIdx] = m_Group.currentCmd.targetPos[j] / pulsePerUnit;
            }
        }

        return true;
    }

    size_t GetQueueSize() const noexcept
    {
        return
            m_Group.cmdQueue.size();
    }

    size_t GetCommandIngressSize() const noexcept
    {
        return
            m_Group.cmdQueue.ingress_size();
    }

    size_t GetCommandReplaySize() const noexcept
    {
        return
            m_Group.cmdQueue.replay_size();
    }

    static constexpr size_t GetCommandIngressCapacity() noexcept
    {
        return
            MOTION_COMMAND_INGRESS_CAPACITY;
    }

    std::uint64_t GetCommandQueueFullRejectCount() const noexcept
    {
        return
            m_commandQueueFullRejectCount.load(
                std::memory_order_relaxed);
    }

    std::uint64_t GetCommandReplayOverflowCount() const noexcept
    {
        return
            m_commandReplayOverflowCount.load(
                std::memory_order_relaxed);
    }

    MotionSegmentId GetLastRejectedSegmentId() const noexcept
    {
        return
            m_lastRejectedSegmentId.load(
                std::memory_order_relaxed);
    }
    // 🌟 新增：讓外部讀取「實體馬達正在執行的行號」
    int GetPhysicalExecutionPC() const {
        return m_Group.currentExecutionPC;
    }

    // 🌟 3. 新增：讓外部讀取實體馬達的 WCS
    int GetPhysicalExecutionWCS() const { return m_Group.currentExecutionWCS; }
    // 🌟 新增：貼標籤機，設定下一個進入佇列的指令是屬於哪一行的
    void SetNextCommandSourcePC(int pc) {
        m_pendingSourcePC = pc;
    }

    void ResetPhysicalPC() {
        m_Group.currentExecutionPC = 0;
        m_pendingSourcePC = 0;
    }



    // 🌟 3. 新增：讓外部 (HMI) 讀取實體馬達的刀具狀態
    int GetPhysicalExecutionToolMode() const { return m_Group.currentExecutionToolMode; }
    int GetPhysicalExecutionHCode() const { return m_Group.currentExecutionHCode; }

    // 🌟 3. 新增 Get 函式
    int GetPhysicalExecutionToolRadiusMode() const { return m_Group.currentExecutionToolRadiusMode; }
    int GetPhysicalExecutionDCode() const { return m_Group.currentExecutionDCode; }

    // 取得實體狀態
    bool GetPhysicalExecutionIsAbsoluteMode() const { return m_Group.currentExecutionIsAbsoluteMode; }

    // 🌟 3. 取得實體狀態
    bool GetPhysicalExecutionG68Active() const { return m_Group.currentExecutionG68Active; }
    double GetPhysicalExecutionG68Angle() const { return m_Group.currentExecutionG68Angle; } // 🌟 3. 新增 Get 函式
    // 🌟 3. 取得實體狀態
    bool GetPhysicalExecutionG168Active() const { return m_Group.currentExecutionG168Active; }
    int GetPhysicalExecutionWCode() const { return m_Group.currentExecutionWCode; } // 🌟 3. 新增 Get 函式
    // 🌟 3. 取得實體狀態
    bool GetPhysicalExecutionG51Active() const { return m_Group.currentExecutionG51Active; }
    double GetPhysicalExecutionScaleRatio() const { return m_Group.currentExecutionScaleRatio; }

    // 🌟 3. 取得實體狀態
    uint8_t GetPhysicalExecutionMirrorMask() const { return m_Group.currentExecutionMirrorMask; }

    // 🌟 3. 取得實體狀態
    bool GetPhysicalExecutionG16Active() const { return m_Group.currentExecutionG16Active; }

    bool GetPhysicalExecutionG162Active() const { return m_Group.currentExecutionG162Active; }
    int GetPhysicalExecutionPlaneMode() const { return m_Group.currentExecutionPlaneMode; }

    // 🌟 4. 終極標籤機：現在一次貼 6 張標籤！
    void SetNextCommandTranslation(const NCTranslationSnapshot& snapshot) noexcept
    { m_pendingTranslation = snapshot; }
    bool PublishNCTranslation(const NCTranslationSnapshot& snapshot) noexcept
    { return IsNCTranslationAxisIdentityCurrent(snapshot) && m_translationPublication.Publish(snapshot); }
    // NC writer only: replace the distance interpretation after exact RT drain.
    // Geometry, run identity and the accepted native endpoint remain unchanged.
    MotionNCTranslationTransitionResult TryTransitionNCTranslation(const NCTranslationSnapshot& previous,
        const NCTranslationSnapshot& next, MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease) noexcept;
    void RetireNCTranslation() noexcept { m_translationPublication.Retire(); }
    // Read-only bounded drain/predecessor proof for a native G53 handoff.
    MotionNCTranslationTransitionResult CheckG53NativeHandoff(const NCTranslationSnapshot& snapshot,
        MotionExecutionEpoch executionEpoch, const MotionOwnerLease& ownerLease) noexcept;
    bool MatchesNCTranslation(const NCTranslationSnapshot& snapshot) const noexcept
    { return IsNCTranslationAxisIdentityCurrent(snapshot) && m_translationPublication.Matches(snapshot); }
    std::uint64_t GetActiveTranslationGeneration() const noexcept
    { return m_translationPublication.Generation(); }

    void SetNextCommandState(int pc, int wcs, int tLenMode, int hCode, int tRadMode, int dCode, bool isAbsMode, bool isG68, double g68Angle, bool isG168, int wCode, bool isG51, double scaleRatio, uint8_t mirrorMask, bool isG16, bool isG162, int planeMode) {
        m_pendingSourcePC = pc;
        m_pendingSourceWCS = wcs;
        m_pendingToolMode = tLenMode;
        m_pendingHCode = hCode;

        m_pendingToolRadMode = tRadMode; // 新增
        m_pendingDCode = dCode;          // 新增

        m_pendingIsAbsoluteMode = isAbsMode;
        m_pendingG68Active = isG68;
        m_pendingG68Angle = g68Angle;

        m_pendingG168Active = isG168; // 🌟 放入標籤機暫存
        m_pendingWCode = wCode;


        m_pendingG51Active = isG51;
        m_pendingScaleRatio = scaleRatio;

        m_pendingMirrorMask = mirrorMask;
        m_pendingG16Active = isG16;

        m_pendingG162Active = isG162;
        m_pendingPlaneMode = planeMode;
    }

    // 🌟 5. 升級重置函式：防殘影
    void ResetPhysicalTags(int currentBrainWCS, int currentBrainToolMode, int currentBrainHCode, int curTRadMode, int curDCode, bool curIsAbsMode, bool curG68, double curG68Angle, bool isG168, int curWCode, bool curG51, double curScaleRatio, uint8_t curMirrorMask, bool curG16, bool curG162, int curPlaneMode) {
        m_Group.currentExecutionPC = 0;
        m_pendingSourcePC = 0;

        m_Group.currentExecutionWCS = currentBrainWCS;
        m_pendingSourceWCS = currentBrainWCS;

        // 刀具狀態同步
        m_Group.currentExecutionToolMode = currentBrainToolMode;
        m_pendingToolMode = currentBrainToolMode;

        m_Group.currentExecutionHCode = currentBrainHCode;
        m_pendingHCode = currentBrainHCode;

        // 刀徑狀態同步
        m_Group.currentExecutionToolRadiusMode = curTRadMode;
        m_pendingToolRadMode = curTRadMode;

        m_Group.currentExecutionDCode = curDCode;
        m_pendingDCode = curDCode;
        //G90G91 模式
        m_Group.currentExecutionIsAbsoluteMode = curIsAbsMode;
        m_pendingIsAbsoluteMode = curIsAbsMode;

        m_Group.currentExecutionG68Active = curG68;
        m_pendingG68Active = curG68;

        m_Group.currentExecutionG68Angle = curG68Angle;
        m_pendingG68Angle = curG68Angle;

        m_pendingG168Active = isG168; // 🌟 放入標籤機暫存

        m_Group.currentExecutionWCode = curWCode;
        m_pendingWCode = curWCode;

        m_Group.currentExecutionG51Active = curG51;
        m_pendingG51Active = curG51;

        m_Group.currentExecutionScaleRatio = curScaleRatio;
        m_pendingScaleRatio = curScaleRatio;

        m_pendingMirrorMask = curMirrorMask;

        m_Group.currentExecutionG16Active = curG16;
        m_pendingG16Active = curG16;

        m_Group.currentExecutionG162Active = curG162;
        m_pendingG162Active = curG162;

        m_Group.currentExecutionPlaneMode = curPlaneMode;
        m_pendingPlaneMode = curPlaneMode;
    }

    // Stage NC-0.2J.5: P50 producer-owned half of Reset tag rebasing.
    // This deliberately does not mutate m_Group; the matching physical tags
    // are applied later by the 250 us Reset rebase transaction.
    void SetPendingResetExecutionState(
        const MotionNCResetExecutionState& executionState) noexcept;


    // Compatibility helper only: all three arguments must use the same unit.
    double CalculateShortestTarget(double currentPos, double targetPos, double modulo);

    // 🌟 消滅幽靈座標專用 API：將大腦預讀起點，強制同步為馬達當下真實位置
    void SyncVirtualEndPosition();
    bool TryGetSynchronizedG00QueueTailMCS(
        double(&outputMCS)[MAX_AXES]) const noexcept;
    MotionQueueTailSyncResult TrySynchronizeProgramStartQueueTailMCS(
        MotionExecutionEpoch expectedEpoch,
        const MotionOwnerLease& expectedLease,
        double(&outputMCS)[MAX_AXES]) noexcept;

    //G碼參數專區------------------------------------------------------------
    double G00_overrideRatio = 1;//G00 專屬速度比例




private:
    std::unique_ptr<MotionG76OrbitProducerWorkspace> m_g76OrbitProducer;
    std::unique_ptr<MotionEDM173Storage> m_edm173;
    bool IsEDM173Profile() const noexcept;
    void ObserveEDM206HistoryFrame(bool complete) noexcept;
    void FreezeEDM206History() noexcept;
    bool CaptureEDM207Input(unsigned slot, EDM206::InputSource source,
        const MotionServoInputSnapshot& input, EDM206::InputCapture& out) const noexcept;
    bool IsEDM173CommandCurrent(bool bound) const noexcept;
    bool IsEDM173PendingHandoffMembershipCurrent() const noexcept;
    bool IsEDM173HandoffCurrent() const noexcept;
    bool IsEDM173GroupGeometrySupported() const noexcept;
    bool PreserveEDM173PendingHandoffZero(const AxisCommand& command) const noexcept;
    void ServiceEDM173Handoff(bool freshMotionFrame) noexcept;

    MotionQueueTailSyncResult SyncVirtualEndPositionResult() noexcept;
    MotionQueueTailSyncResult GetSynchronizedG00QueueTailMCSResult(
        double(&outputMCS)[MAX_AXES]) const noexcept;
    // EDM48 has its own NC producer / RT consumer channel. Its applied values
    // are shadow observations, never axis, planner or PDO output storage.
    EDM48::Channel m_edmRTShadow{};
    EDM55RT::Session m_edmGapServoRT{};
    EDM83RT::Session m_edmCNCRetainedRT{};
    EDM84RT::Session m_edmCoordinatedRT{};
    struct EDM94ArmRequest
    {
        std::uint64_t session = 0ULL;
        std::uint64_t manualRun = 0ULL, manualCache = 0ULL;
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerLease lease{};
        bool gapBoundary = false;
        bool gapSegment = false;
        bool proportional = false;
        bool noise = false;
        bool burst = false;
        bool continuous = false;
        bool endurance = false;
        bool paired = false;
        bool flush = false;
        bool commonPath = false; // EDM109 explicit opt-in; frozen at arm
        bool b2CrossSegment = false; // EDM110 bounded physical previous-row excursion
        bool b2Repeat = false; // EDM111 immutable second event at step 4
        bool curveRuntime = false; // EDM113 scratch-only RT curve observation
        bool physicalCurve = false; // EDM114 explicit physical LINEAR6 / ARC2 profile
        bool curveB2 = false; // EDM115 explicit frozen LINEAR6 / ARC2 excursion
        bool modeFlush = false; // EDM116 actual B0/B1/B2 flush selector
        bool historyServo = false; // EDM126 distinct signal-driven retained-curve profile
        bool historyModeFlush = false; // EDM128 frozen coordination opt-in
        bool physicalAd = false; // EDM129 independent real-source selection
        bool workTimeFlush = false; // EDM130 E-recipe committed machining-time opt-in
        bool rejectionProbe = false; // EDM216 explicit L216: reject before Engine::Arm, never an output permit.
        unsigned ordinarySequenceProgram = 0U; // EDM247 FIX1: explicit NC sequence scope, never inferred from recipe equality.
        EDM130::Recipe timerRecipe{}; // immutable NC -> RT admission and each handoff
        unsigned historyFixture = 0U;
        EDM116::Fixture modeFixture = EDM116::Fixture::Canonical116; // fixed at arm; never row-selected
        EDM97::Tuning tuning{0.0, 0.0, 0.0};
        EDM107::FlushProfile flushProfile{}; // copied NC -> RT; immutable for this session
    };
    // FIX4: NC is the sole authorization/message producer; the serialized
    // Motion/send owner alone consumes and mutates the transient zero bridge.
    struct EDM94FeedAuthorization
    {
        std::uint64_t session = 0ULL, run = 0ULL, cache = 0ULL, dispatch = 0ULL;
        int sourcePC = -1;
        unsigned step = 0U;
        std::array<double, 6U> increment{};
        double feedMMMin = 0.0;
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerLease lease{};
        int gCode = 1;
        std::array<double, 2U> arcCenter{};
        EDM95::Permit gapPermit{};
        EDM110::CompletionToken previous{};
        EDM115::CompletionToken previousCurve{};
        EDM116::Policy modePolicy{};
        std::uint64_t historyBoundaryTick=0ULL, historyCommit=0ULL;
        EDM126::Source historyPrevious{};
        EDM129::Binding physicalBinding{}; // Independently published ARM source binding
        bool gapSegment = false;
        bool proportional = false;
        bool noise = false;
        bool burst = false;
        bool continuous = false;
        bool endurance = false;
        bool paired = false;
        bool flush = false;
        bool commonPath = false; // EDM109 explicit opt-in; frozen at arm
        bool b2CrossSegment = false; // EDM110 bounded physical previous-row excursion
        bool b2Repeat = false; // EDM111 immutable second event at step 4
        bool curveRuntime = false; // EDM113 scratch-only RT curve observation
        bool physicalCurve = false; // EDM114 explicit physical LINEAR6 / ARC2 profile
        bool curveB2 = false; // EDM115 explicit frozen LINEAR6 / ARC2 excursion
        bool modeFlush = false; // EDM116 actual B0/B1/B2 flush selector
        bool historyServo = false; // EDM126 distinct signal-driven retained-curve profile
        bool historyModeFlush = false; // EDM128 frozen coordination opt-in
        bool physicalAd = false; // EDM129 independent real-source selection
        bool workTimeFlush = false; // EDM130 E-recipe committed machining-time opt-in
        EDM130::Recipe timerRecipe{}; // immutable NC -> RT admission and each handoff
        unsigned historyFixture = 0U;
        EDM97::Tuning tuning{0.0, 0.0, 0.0};
        EDM107::FlushProfile flushProfile{}; // copied NC -> RT; immutable for this session
    };
    struct EDM94HandoffMessage
    {
        EDM94FeedAuthorization authorization{};
        MotionExecutionEpoch nextEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        MotionExecutionIdentity command{};
        std::array<double, 6U> targetPulse{};
        std::array<double, 6U> gapSegmentStartPulse{};
        double targetVelocity = 0.0, acceleration = 0.0, deceleration = 0.0;
        std::array<double, 6U> arcPulsePerNative{};
        std::array<double, 2U> arcCenterPulse{};
        double arcRadius = 0.0, arcStartAngle = 0.0, arcSweep = 0.0, arcLength = 0.0;
        std::uint64_t ownerWord = 0ULL, executionWord = 0ULL;
        std::uint64_t safetyWord = 0ULL, alarmSafetyWord = 0ULL;
        std::uint32_t alarmRevision = 0U;
        bool committed = false;
    };
    EDM94FeedAuthorization m_edm94FeedAuthorization{}; // NC-thread only
    FixedCapacitySpscRing<EDM94HandoffMessage, 4U> m_edm94HandoffMessages{};
    EDM94HandoffMessage m_edm94Handoff{}; // RT owner only
    bool m_edm94HandoffPending = false;
    bool m_edm94HandoffCommitted = false;
    std::uint64_t m_edm94HandoffStartedUs = 0ULL;
    std::uint64_t m_edm94LastHandoffDispatch = 0ULL;
    unsigned m_edm94NextHandoffStep = 0U;
    MotionExecutionEpoch m_edm94ExpectedEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    EDM95::Boundary m_edm95{}; // RT owner only; does not create references.
    EDM53::Channel<EDM95::Feedback, 512U> m_edm95Feedback{};
    bool m_edm95Profile = false;
    void ServiceEDM95Boundary(bool complete) noexcept;
    void PublishEDM95Feedback() noexcept;
    EDM96::Segment m_edm96{}; // RT scalar only; ordinary mapping and CSV writer remain authoritative.
    EDM53::Channel<EDM96::Feedback, 512U> m_edm96Feedback{};
    bool m_edm96Profile = false;
    bool m_edm97Profile = false;
    bool m_edm98Profile = false;
    bool m_edm99Profile = false;
    bool m_edm100Profile = false;
    bool m_edm102Profile = false;
    bool m_edm104Profile = false;
    bool m_edm105Profile = false;
    EDM107::FlushProfile m_edm107ArmFlushProfile{}; // RT-owned frozen arm value; never read from NC state
    bool m_edm109Profile = false;
    EDM109::Mapping m_edm109Mapping{}; // retained through the existing formal stop
    MotionExecutionIdentity m_edm109Command{};
    EDM109::Feedback m_edm109Current{}; // RT writer only
    EDM53::Channel<EDM109::Feedback, 512U> m_edm109Feedback{};
    bool IsEDM109PublishedProfile(std::uint64_t session, bool commonPath) const noexcept;
    bool IsEDM109ProfileCurrent(bool commonPath) const noexcept;
    bool IsEDM109MappingCurrent() const noexcept;
    bool MapEDM109Line(AxisCommand& command) noexcept;
    void PublishEDM109Feedback() noexcept;
    static_assert(sizeof(EDM113::Observer) <= 4096U, "EDM113 fixed RT observation state budget");
    bool m_edm113Profile = false;
    EDM113::Observer m_edm113{}; // RT-owned scratch only; never controls mapped output
    EDM53::Channel<EDM113::Feedback, 512U> m_edm113Feedback{};
    bool IsEDM113PublishedProfile(std::uint64_t session, bool enabled) const noexcept;
    bool IsEDM113ProfileCurrent(bool enabled) const noexcept;
    bool CaptureEDM113Source(EDM109::Source& source) const noexcept;
    void BindEDM113Curve() noexcept;
    void ObserveEDM113Line(const AxisCommand& command) noexcept;
    void SuspendEDM113Curve() noexcept;
    void PublishEDM113Feedback() noexcept;
    bool m_edm110Profile = false, m_edm110Driving = false, m_edm111Repeat = false;
    EDM110::Excursion m_edm110{};
    EDM110::MotionFeedback m_edm110Current{};
    EDM53::Channel<EDM110::MotionFeedback, 512U> m_edm110Feedback{};
    EDM109::Line m_edm110PreviousLine{}, m_edm110OriginalLine{};
    double m_edm110LastAccelTime = 0.0, m_edm110LastDecelTime = 0.0;
    double m_edm110PreviousAccelTime = 0.0, m_edm110PreviousDecelTime = 0.0;
    double m_edm110OriginalMaxVelocity = 0.0, m_edm110OriginalAcceleration = 0.0, m_edm110OriginalDeceleration = 0.0;
    std::array<double, 6U> m_edm110StopTime{}, m_edm110Reference{}, m_edm110ActualMin{}, m_edm110ActualMax{};
    std::uint64_t m_edm110PhysicalTick = 0ULL;
    unsigned m_edm110PhysicalSpan = 0U;
    EDM109::Point m_edm110MappedReference{};
    std::uint64_t m_edm110MappedTick = 0ULL;
    unsigned m_edm110MappedSpan = 0U;
    bool m_edm110MappedZero = false, m_edm110SameReferencePriorTick = false;
    bool m_edm110PhysicalQualified = false, m_edm110RangeValid = false;
    bool IsEDM110PublishedProfile(std::uint64_t session, bool enabled, bool repeat = false) const noexcept;
    bool IsEDM110ProfileCurrent(bool enabled) const noexcept;
    bool ValidateEDM110Previous(const EDM94FeedAuthorization& authorization) noexcept;
    bool IsEDM110GeometryCurrent() const noexcept;
    bool IsEDM110VirtualCurrent() const noexcept;
    void ObserveEDM110PhysicalSettle(bool complete, const EDM94::RuntimeDiagnostic& priorSendProof) noexcept;
    bool BeginEDM110Excursion(AxisCommand& command) noexcept;
    bool StepEDM110Excursion(AxisCommand& command, bool stopping) noexcept;
    void ApplyEDM110Scalar(AxisCommand& command, bool stopping) noexcept;
    bool MapEDM110Excursion() noexcept;
    void PublishEDM110Feedback() noexcept;
    void ServiceEDM96Source() noexcept;
    void PublishEDM96Feedback() noexcept;
    bool BeginEDM96Segment() noexcept;
    bool StepEDM96Segment(AxisCommand& command) noexcept;
    bool AuthorizeEDM94FeedScoped(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step,
        const std::array<double, 6U>& increment, double feedMMMin,
        MotionExecutionEpoch epoch, const MotionOwnerLease& lease, bool endurance, bool paired = false, bool flush = false,
        const EDM107::FlushProfile& flushProfile = EDM107::FlushProfile{},
        bool commonPath = false, bool b2CrossSegment = false,
        const EDM110::CompletionToken& previous = EDM110::CompletionToken{}, bool b2Repeat = false,
        bool curveRuntime = false, bool physicalCurve = false, int gCode = 1,
        const std::array<double, 2U>& center = std::array<double, 2U>{}, bool curveB2 = false,
        const EDM115::CompletionToken& previousCurve = EDM115::CompletionToken{}, bool modeFlush = false,
        const EDM116::Policy& modePolicy = EDM116::Policy{}) noexcept;
    bool TakeEDM114ArcAuthorization(const std::array<double, 6U>& increment,
        const std::array<double, 6U>& startNative, const std::array<double, 2U>& center, int gCode, double feedMMMin,
        EDM94FeedAuthorization& result) noexcept;
    bool PublishEDM114ArcHandoff(const EDM94FeedAuthorization& authorization,
        const MotionCommand& command, const std::array<double, 6U>& startPulse,
        const std::array<double, 6U>& endPulse, const std::array<double, 6U>& pulsePerNative,
        double startAngle, double sweepRadians, double length,
        EDM94HandoffMessage& message) noexcept;
    bool AuthorizeEDM126FeedScoped(std::uint64_t session, std::uint64_t run, std::uint64_t cache,
        std::uint64_t dispatch, int sourcePC, unsigned step, int gCode,
        const std::array<double,6U>& increment, const std::array<double,2U>& center,
        double feedMMMin, MotionExecutionEpoch epoch, const MotionOwnerLease& lease,
        const EDM97::Tuning& tuning, const EDM114::Source& previousSource,
        std::uint64_t completionCommit, std::uint64_t completionTick,
        bool modeFlush, const EDM116::Policy& policy, bool physicalAd=false,
        bool workTimeFlush=false, const EDM130::Recipe& recipe=EDM130::Recipe{}) noexcept;
    bool IsEDM128FeedPolicyCurrent(const EDM94FeedAuthorization& authorization) const noexcept;
    bool IsEDM129FeedSourceCurrent(const EDM94FeedAuthorization& authorization) const noexcept;
    bool IsEDM129AwaitFreshZeroCurrent(std::uint64_t nowUs,
        std::uint32_t* failedGate = nullptr) const noexcept;
    bool IsEDM130FeedTimingCurrent(const EDM94FeedAuthorization& authorization) const noexcept;
    bool IsEDM130TimingCurrent() const noexcept;
    EDM139::Stimulus m_edm139Stimulus{};
    EDM149::Source m_edm149Voltage{}; // Separate simulated control; never overwrites physical AD.
    // One NC publisher supplies values only. RT owns admission, lease and
    // sample production; these channels never carry an axis/PDO command.
    EDM53::Channel<EDM169::Request,512U> m_edm169ManualRequests{};
    EDM53::Channel<EDM169::Feedback,512U> m_edm169ManualFeedback{};
    EDM169::Receiver m_edm169Manual{};
    void ClearEDM169ManualVoltage() noexcept;
    bool ServiceEDM169ManualVoltage() noexcept;
    bool m_edm130Profile=false;
    EDM130::Recipe m_edm130ArmRecipe{};
    bool m_edm129Profile=false;
    EDM129::PhysicalSource m_edm129Source{};
    EDM129::MotionFeedback m_edm129Current{};
    EDM53::Channel<EDM129::MotionFeedback,1024U> m_edm129Feedback{};
    EDM64::Snapshot m_edm129Physical{}; // Captured before the observer's QPC read.
    EDM53::Stamp m_edm129PhysicalStamp{};
    const EDM126::Feedback& EDM126CurrentImage() const noexcept;
    const EDM126::Feedback& EDM126CommittedImage() const noexcept;
    bool m_edm128Profile=false;
    EDM128::Coordinator m_edm128{};
    EDM128::TestVoltage m_edm128Voltage{};
    EDM128::MotionFeedback m_edm128Current{};
    EDM53::Channel<EDM128::MotionFeedback,1664U> m_edm128Feedback{};
    // EDM157: one NC producer, one serialized Motion/send consumer. Requests
    // carry values only; the existing committed traversal remains the cursor.
    struct EDM157Request
    {
        EDM157::Scope scope{};
        std::uint64_t holdSequence=0ULL, startSequence=0ULL, issuedTick=0ULL, issuedUs=0ULL;
        bool hold=false;
    };
    FixedCapacitySpscRing<EDM157Request,4U> m_edm157Requests{};
    EDM157Request m_edm157Incoming{},m_edm157HoldRequest{},m_edm157ResumeRequest{};
    EDM157::Transaction m_edm157Pause{};
    EDM53::Channel<EDM157::Feedback,1152U> m_edm157Feedback{};
    std::uint64_t m_edm157ZeroOutputTick=0ULL,m_edm157QualifySeedTick=0ULL;
    std::uint64_t m_edm157PhysicalDiagnosticTick=0ULL;
    std::uint32_t m_edm157PhysicalEvaluated=0U,m_edm157PhysicalFailures=0U,m_edm157PhysicalAxisMask=0U;
    EDM157::Scope CurrentEDM157Scope() const noexcept;
    bool IsEDM157ScopeCurrent(const EDM157::Scope& scope) const noexcept;
    bool IsEDM157PublishedScopeCurrent(const EDM157::Scope& scope, std::uint64_t& tick,
        std::uint64_t& nowUs) const noexcept;
    bool ServiceEDM157Requests() noexcept;
    void ObserveEDM157PauseProof(bool complete,const EDM94::RuntimeDiagnostic& priorSendProof) noexcept;
    void RejectEDM157Pause(EDM157::RejectSite site,EDM157::Phase priorPhase,
        std::uint32_t proofEvaluated=0U,std::uint32_t proofFailures=0U,std::uint64_t proofTick=0ULL) noexcept;
    void PublishEDM157Feedback() noexcept;
    // EDM126 bounded traversal state (no NC rewind or historical side-effect replay).
    bool m_edm126Profile=false, m_edm126Participating=false, m_edm126PhysicalSettled=false;
    EDM126::Traversal m_edm126{};
    EDM126::MotionFeedback m_edm126Current{};
    EDM126::FirstCauseRecord m_edm133FirstSource{}, m_edm133FirstOrphan{};
    EDM53::Channel<EDM126::MotionFeedback,1056U> m_edm126Feedback{};
    EDM96::Feedback m_edm126Gap{};
    EDM88::Qualifier m_edm126Source{};
    EDM88::SourceConfig m_edm126SourceConfig{};
    EDM88::Frame m_edm126Raw{};
    EDM89::Decision m_edm126Decision{};
    EDM97::Tuning m_edm126Tuning{};
    EDM126::TestVoltage m_edm126Voltage{};
    EDM112::Curve m_edm126Curve{};
    EDM126::Limits m_edm126Limits{};
    EDM126::Point m_edm126PhysicalMin{}, m_edm126PhysicalMax{}, m_edm126PhysicalReference{};
    bool m_edm126MappedZero=false, m_edm126SameReferencePriorTick=false;
    std::uint64_t m_edm126PhysicalTick=0ULL;
    bool m_edm126PhysicalRange=false;
    bool BeginEDM126Arm(const EDM94ArmRequest& arm) noexcept;
    bool IsEDM126ProfileCurrent() const noexcept;
    bool IsEDM126FeedCurrent(const EDM94FeedAuthorization& authorization) const noexcept;
    bool IsEDM126ParticipationCurrent() const noexcept;
    bool IsEDM126Current() const noexcept;
    // Set the local proof only directly after successful G76 IsEDM126Current.
    bool IsEDM126SourceCurrent(std::uint64_t nowUs,
        bool g76ProfileJustValidated=false) const noexcept;
    void ObserveEDM126Source(bool complete) noexcept;
    void ObserveEDM126PhysicalSettle(bool complete, const EDM94::RuntimeDiagnostic& priorSendProof) noexcept;
    bool BeginEDM126Segment() noexcept;
    bool StepEDM126Segment(AxisCommand& command) noexcept;
    bool MapEDM126Traversal(AxisCommand& command) noexcept;
    bool BeginEDM126Stop(MotionExecutionEpoch previousEpoch, const MotionOwnerLease& previousLease,
        std::uint32_t previousTicket) noexcept;
    bool StepEDM126Stop(AxisCommand& command) noexcept;
    void ApplyEDM126Scalar(AxisCommand& command, bool stopping) noexcept;
    void PublishEDM126Feedback() noexcept;
    void CaptureEDM133FirstCause(EDM126::FirstCauseSite site, int axis,
        std::uint32_t evaluatedGates, std::uint32_t failedGates) noexcept;
    bool m_edm114Profile = false;
    EDM114::Mapping m_edm114Mapping{};
    EDM114::Feedback m_edm114Current{};
    EDM53::Channel<EDM114::Feedback, 512U> m_edm114Feedback{};
    MotionExecutionIdentity m_edm114Command{};
    EDM114::StopScalar m_edm114Stop{};
    MotionOwnerLease m_edm114StopLease{};
    MotionExecutionEpoch m_edm114StopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    std::uint32_t m_edm114StopTicket = 0U;
    std::uint64_t m_edm114StopTick = 0ULL, m_edm114MappedTick = 0ULL;
    double m_edm114LastVelocity = 0.0, m_edm114LastPosition = 0.0;
    EDM114::Point m_edm114MappedPosition{}, m_edm114MappedVelocity{};
    std::array<std::uint64_t, 6U> m_edm114ConfigFingerprint{}, m_edm114MappingFingerprint{};
    bool IsEDM114PublishedProfile(std::uint64_t session, bool enabled) const noexcept;
    bool IsEDM114ProfileCurrent(bool enabled) const noexcept;
    bool IsEDM114ArcCommand(const MotionCommand& command) const noexcept;
    bool IsEDM114MappingCurrent() const noexcept;
    bool BindEDM114Mapping(double length) noexcept;
    bool MapEDM114Curve(AxisCommand& command) noexcept;
    bool BeginEDM114Stop(MotionExecutionEpoch previousEpoch,
        const MotionOwnerLease& previousLease, std::uint32_t previousTicket) noexcept;
    bool StepEDM114Stop(AxisCommand& command) noexcept;
    void PublishEDM114Feedback() noexcept;
    bool m_edm115Profile = false, m_edm115Driving = false, m_edm115Participating = false;
    bool m_edm115ResumeMap = false;
    EDM115::Excursion m_edm115{}, m_edm115Committed{};
    bool m_edm115UnmappedStep = false;
    EDM115::MotionFeedback m_edm115Current{};
    EDM53::Channel<EDM115::MotionFeedback, 1024U> m_edm115Feedback{};
    EDM114::Mapping m_edm115Previous{}, m_edm115Original{};
    EDM114::Feedback m_edm115PreviousProof{};
    double m_edm115LastAccelTime = 0.0, m_edm115LastDecelTime = 0.0;
    double m_edm115PreviousAccelTime = 0.0, m_edm115PreviousDecelTime = 0.0;
    double m_edm115OriginalMaxVelocity = 0.0, m_edm115OriginalAcceleration = 0.0, m_edm115OriginalDeceleration = 0.0;
    EDM115::Point m_edm115StopTime{}, m_edm115Reference{}, m_edm115ActualMin{}, m_edm115ActualMax{};
    EDM115::Point m_edm115MappedReference{}, m_edm115MappedVelocity{};
    double m_edm115MappedQ = 0.0, m_edm115MappedV = 0.0;
    std::uint64_t m_edm115PhysicalTick = 0ULL, m_edm115MappedTick = 0ULL;
    unsigned m_edm115PhysicalSpan = 0U, m_edm115MappedSpan = 0U;
    bool m_edm115MappedZero = false, m_edm115SameReferencePriorTick = false;
    bool m_edm115PhysicalQualified = false, m_edm115RangeValid = false;
    MotionOwnerLease m_edm115StopLease{};
    MotionExecutionEpoch m_edm115StopEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    std::uint32_t m_edm115StopTicket = 0U;
    bool IsEDM115PublishedProfile(std::uint64_t session, bool enabled) const noexcept;
    bool IsEDM115ProfileCurrent(bool enabled) const noexcept;
    bool ValidateEDM115Previous(const EDM94FeedAuthorization& authorization) noexcept;
    bool IsEDM115ParticipationCurrent() const noexcept;
    bool IsEDM115GeometryCurrent() const noexcept;
    bool IsEDM115VirtualCurrent() const noexcept;
    void ObserveEDM115PhysicalSettle(bool complete, const EDM94::RuntimeDiagnostic& priorSendProof) noexcept;
    bool BeginEDM115Excursion(AxisCommand& command) noexcept;
    bool StepEDM115Excursion(AxisCommand& command, bool stopping) noexcept;
    void ApplyEDM115Scalar(AxisCommand& command, bool stopping) noexcept;
    bool MapEDM115Excursion() noexcept;
    bool BeginEDM115Stop(MotionExecutionEpoch previousEpoch,
        const MotionOwnerLease& previousLease, std::uint32_t previousTicket) noexcept;
    void PublishEDM115Feedback() noexcept;
    bool m_edm116Profile=false;
    EDM116::Fixture m_edm116ArmFixture=EDM116::Fixture::Canonical116;
    EDM116::Excursion m_edm116{}, m_edm116Committed{};
    EDM116::MotionFeedback m_edm116Current{};
    EDM53::Channel<EDM116::MotionFeedback,1024U> m_edm116Feedback{};
    EDM116::Policy m_edm116RowPolicy{};
    std::uint64_t m_edm116PreviousCommit=0ULL;
    bool IsEDM116PublishedProfile(std::uint64_t session,bool enabled,
        EDM116::Fixture fixture=EDM116::Fixture::Canonical116) const noexcept;
    bool IsEDM116ProfileCurrent(bool enabled) const noexcept;
    bool ValidateEDM116Feed(const EDM94FeedAuthorization& authorization) noexcept;
    bool BeginEDM116Excursion(AxisCommand& command) noexcept;
    void PublishEDM116Feedback() noexcept;
    // Diagnostic-only stop evidence. It cannot permit output, continue a row,
    // release a session or replace the independent EDM94 physical zero proof.
    EDM118::Feedback m_edm118Current{};
    EDM53::Channel<EDM118::Feedback,1024U> m_edm118Feedback{};
    void ClearEDM118StopObservation() noexcept;
    void CaptureEDM118StopObservation() noexcept;
    void ObserveEDM118MappedStop() noexcept;
    void PublishEDM118Feedback() noexcept;
    bool FailEDM118StopObservation() noexcept;
    const EDM115::Feedback& ModeExcursionCurrent(bool committed=false) const noexcept;
    const EDM114::Mapping& ModeExcursionMapping(unsigned which=0U,bool committed=false) const noexcept;
    const EDM115::CompletionToken& ModeExcursionToken(bool committed=false) const noexcept;
    const EDM115::Point& ModeExcursionAnchor() const noexcept;
    double ModeExcursionAnchorQ() const noexcept;
    bool ModeExcursionMap(EDM115::Point& position,EDM115::Point& velocity) const noexcept;
    void ModeExcursionCommit() noexcept;
    void ModeExcursionRestore() noexcept;
    bool IsEDM107FlushProfileCurrent(const EDM107::FlushProfile& flushProfile) const noexcept;
    bool IsEDM102FeedProfileCurrent(const EDM94FeedAuthorization& authorization) const noexcept;
    bool TakeEDM94FeedAuthorization(const std::array<double, 6U>& increment,
        double feedMMMin, EDM94FeedAuthorization& result) noexcept;
    bool PublishEDM94Handoff(const EDM94FeedAuthorization& authorization,
        const MotionCommand& command, EDM94HandoffMessage& message) noexcept;
    void ObserveEDM94Handoff(bool freshMotionFrame) noexcept;
    bool IsEDM94HandoffAuthorityCurrent() const noexcept;
    bool IsEDM94HandoffCommandCurrent() const noexcept;
    EDM94::Engine m_edm94{};
    EDM94::Frame m_edm94Frame{};
    EDM94::TangentEnvelope m_edm192FrameEnvelope{};
    EDM94::CenterEnvelope m_edm204FrameEnvelope{};
    EDM94::Config m_edm94ObservedConfig{};
    EDM53::Channel<EDM94::Feedback, 4096U> m_edm94Feedback{};
    FixedCapacitySpscRing<EDM94ArmRequest, 2U> m_edm94ArmRequests{};
    std::atomic<std::uint64_t> m_edm94Claim{0ULL};
    std::atomic<std::uint64_t> m_edm94CancelSession{0ULL};
    std::atomic<std::uint64_t> m_edm94ExternalStopSession{0ULL};
    std::atomic<std::uint64_t> m_edm94FinishSession{0ULL};
    std::atomic<std::uint64_t> m_edm94ReleaseSession{0ULL};
    struct EDM94CompletionReleaseRequest
    {
        EDM94::Identity armIdentity{};
        MotionOwnerLease idleLease{};
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        std::uint64_t idleGrant = 0ULL;
        std::uint64_t safetyIntent = 0ULL, alarmIntent = 0ULL;
        std::uint32_t alarmRevision = 0U;
    };
    FixedCapacitySpscRing<EDM94CompletionReleaseRequest, 2U> m_edm94CompletionRequests{};
    EDM94CompletionReleaseRequest m_edm94CompletionRequest{}; // RT owner only
    bool m_edm94CompletionPending = false;
    bool m_edm94CompletionRevoked = false;
    bool m_edm94CompletionOwnerReleased = false;
    // Claim pins the session; this CAS orders NC cancel against normal release.
    // 0=open, 1=cancel won, 2=M30 release, 3=EDM233 parent retirement.
    std::atomic<unsigned> m_edm94CompletionDecision{0U};
    std::uint64_t m_edm94CompletionOwnerReleaseTick = 0ULL;
    void FinishEDM94CompletionRelease(ServoOutputFrameReservation& reservation,
        bool exactSent, bool handedOff) noexcept;
    struct EDM233ParentReleaseRequest
    {
        EDM94::Identity armIdentity{};
        MotionOwnerLease lease{};
        MotionExecutionEpoch liveEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        std::uint64_t safetyIntent = 0ULL, alarmIntent = 0ULL;
        std::uint32_t alarmRevision = 0U;
    };
    FixedCapacitySpscRing<EDM233ParentReleaseRequest, 2U> m_edm233ParentReleaseRequests{};
    EDM233ParentReleaseRequest m_edm233ParentReleaseRequest{}; // RT owner only
    EDM94::Identity m_edm233ParentReleaseSubmitted{}; // NC producer only
    bool m_edm233ParentReleasePending = false;
    void FinishEDM233ParentRelease(ServoOutputFrameReservation& reservation,
        bool exactSent, bool handedOff) noexcept;
    struct EDM234ParentReleaseRequest
    {
        EDM94::Identity armIdentity{};
        MotionOwnerLease lease{};
        MotionExecutionEpoch liveEpoch = MOTION_EXECUTION_EPOCH_INVALID;
        std::uint64_t run = 0ULL, cache = 0ULL, parentDispatch = 0ULL;
        int parentPC = -1;
        unsigned kind = 0U, program = 0U;
        EDM130::Recipe recipe{};
        std::uint64_t safetyIntent = 0ULL, alarmIntent = 0ULL;
        std::uint32_t alarmRevision = 0U;
    };
    FixedCapacitySpscRing<EDM234ParentReleaseRequest, 2U> m_edm234ParentReleaseRequests{};
    EDM234ParentReleaseRequest m_edm234ParentReleaseRequest{}; // RT owner only
    EDM234ParentReleaseRequest m_edm234ParentReleaseSubmitted{}; // NC producer only
    bool m_edm234ParentReleasePending = false;
    void FinishEDM234ParentRelease(ServoOutputFrameReservation& reservation,
        bool exactSent, bool handedOff) noexcept;
    std::uint64_t m_edm94LastSession = 0ULL;
    std::uint64_t m_edm94ArmUs = 0ULL;
    std::uint64_t m_edm94ClockCounter = 0ULL;
    MotionNCSettleRequestSequence m_edm94ResetFloor = 0ULL;
    bool m_edm94FormalStopRequested = false;
    std::array<EDM54::FrameIdentity<64U>, 6U> m_edm94ConfigIdentity{};
    EDM54::FrameIdentity<64U> m_edm94MapIdentity{};
    std::array<ServoOutput*, 6U> m_edm94FrozenOutput{};
    std::array<const ServoInput*, 6U> m_edm94FrozenInput{};
    EDM94Serialized::Mapping m_edm94FrozenSerializedMapping{};
    bool CaptureEDM192TangentEnvelope(const EDM94::Frame& frame,EDM94::TangentEnvelope& output) const noexcept;
    bool CaptureEDM204CenterEnvelope(const EDM94::Frame& frame,EDM94::CenterEnvelope& output) const noexcept;
    EDM94::Frame CaptureEDM94Frame(bool complete, bool configurationOnly = false) noexcept;
    void ServiceEDM94MotionPass(bool complete) noexcept;
    void ObserveEDM94Cancellation() noexcept;
    void EnsureEDM94FormalStop() noexcept;
    double GuardEDM94Velocity(int axisIndex, double value) noexcept;
    bool GuardEDM94OutputImage() noexcept;
    bool EDM94ConfigurationCurrent() noexcept;
    bool ValidateEDM94SendClock() noexcept;
    void TripEDM94(EDM94::Reason reason, EDM94::FaultSite site = EDM94::FaultSite::EngineFault) noexcept;
    void PublishEDM94Feedback() noexcept;
    bool BeginEDM94Send(ServoOutputFrameReservation& reservation) noexcept;
    bool FinalizeEDM94Send(ServoOutputFrameReservation& reservation) noexcept;
    bool SealEDM94SerializedImage(ServoOutputFrameReservation& reservation,
        const std::uint8_t* base, std::size_t size,
        const std::uint8_t* payload, std::size_t payloadSize) noexcept;
    void FinishEDM94Send(ServoOutputFrameReservation& reservation,
        bool attempted, bool succeeded) noexcept;
    // EDM90: exact identities for a read-only six-axis native preview. These
    // RT-owned observers never write AxisContext, planner or PDO state.
    std::array<EDM54::FrameIdentity<64U>, 6U> m_edmMultiAxisConfigIdentity{};
    EDM54::FrameIdentity<64U> m_edmMultiAxisMapIdentity{};
    void ObserveEDMMultiAxisMachineRuntime(bool complete) noexcept;
    EDM69::Session m_edmPhysicalServoRT{};
    EtherCatMaster* m_pEDMPhysicalSourceMaster = nullptr; // Immutable after boot binding.
    void ObserveEDMGapServoRTRuntimeCycle(const EDM48::Facts& facts) noexcept;
    void ObserveEDMRTShadowRuntimeCycle(std::uint64_t tick,
        bool pdoCycleValid, bool contiguous, bool servicePhysical,
        const EDM64::Snapshot& physical, const EDM53::Stamp& physicalStamp) noexcept;

    // EDM28 owns a separate bounded channel; the generic axis mailbox and all
    // installed axis/PID/COND parameters remain untouched.
    struct EDMZQueuedRequest
    {
        EDM28::Request request{};
        std::uint64_t stopTicket = 0ULL;
    };
    FixedCapacitySpscRing<EDMZQueuedRequest, 8U> m_edmZRequests{};
    EDM28::RTPolicy m_edmZPolicy{};
    EDM56::Controller m_edmGapPhysical{};
    EDM70::Controller m_edmPhysicalMotion{};
    EDM62::SimRawSource m_edmGapRTSource{};
    EDM28::Observation m_edmZObservation{};
    EDM52::CancelFence m_edmZCancelFence{};
    std::uint64_t m_edmZObservedStopTicket = 0ULL;
    std::uint64_t m_edmZHeartbeatDeadlineUs = 0ULL;
    std::uint64_t m_edmZSourceTick = 0ULL;
    std::uint64_t m_edmZReviewedCycleTick = 0ULL;
    std::uint64_t m_edmZLastActualTick = 0ULL;
    std::uint64_t m_edmZLastActualUs = 0ULL;
    double m_edmZLastActualPulse = 0.0;
    double m_edmZActualVelocityPps = 0.0;
    EDM54::FrameIdentity<64U> m_edmZConfigIdentity{};
    EDM54::FrameIdentity<64U> m_edmZMapIdentity{};
    std::uint64_t m_edmZClockFrequency = 0ULL;
    std::uint64_t m_edmZClockCounter = 0ULL;
    std::uint64_t m_edmZMonotonicUs = 0ULL;
    bool m_edmZClockValid = false;
    bool m_edmZInputSemantic = false;
    bool m_edmZFrameIntegrityStopLatched = false;
    EDM53::Channel<EDM28::Feedback> m_edmZFeedback{};
    void ObserveEDMZFixtureRuntimeClock(std::uint64_t tick, bool valid, bool contiguous) noexcept;
    EDM28::Observation BuildEDMZFixtureObservation(const AxisContext& axis,
        const MotionServoInputSnapshot* input, bool sourceFresh) noexcept;
    void ProcessEDMZFixtureAxis(AxisContext& axis, const MotionServoInputSnapshot& input) noexcept;
    bool GenerateEDMGapPhysicalCommand(AxisContext& axis, AxisCommand& command) noexcept;
    void FinishEDMZFixtureMotionPass() noexcept;
    void ApplyEDMZFixtureDecision(const EDM28::Decision& decision) noexcept;
    void PublishEDMZFixtureFeedback() noexcept;
    void StopEDMZFixtureFromRuntime() noexcept;
    void LatchEDMZFixtureFrameIntegrityFailure() noexcept;
    double GuardEDMZFixtureVelocity(int axisIndex, double value) noexcept;
    bool GuardEDMZFixtureOutputImage() noexcept;
    bool ValidateEDMZFixtureSendClock() noexcept;
    bool SealEDMZFixtureSerializedImage(ServoOutputFrameReservation& reservation,
        const std::uint8_t* ioMapBase, std::size_t ioMapSize,
        const std::uint8_t* payload, std::size_t payloadSize) noexcept;
    static_assert(sizeof(EDM28::Feedback) <= 2048U &&
        std::is_trivially_copyable<EDM28::Feedback>::value,
        "EDM28 coherent feedback must remain fixed, bounded value storage.");
    // ========================================================================
    // Stage NC-0.2J.4 - Atomic Stop / Settle Evidence Publication
    //
    // A complete POD payload is copied into one of two banks of 64-bit atomic
    // words.  The producer publishes the selected bank with one Generation.
    // A reader retries if Generation changes while copying.  Therefore the
    // reader never races a plain AxisContext read and no mutex or atomic large
    // structure is required.
    // ========================================================================
    struct PathCoreHoldRequest
    {
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        std::uint64_t generation = 0ULL;
        std::uint64_t settleSequence = 0ULL;
        std::uint64_t expectedTransitionSequence = 0ULL;
        std::uint64_t translationGeneration = 0ULL;
        std::uint32_t cycleLimit = 1U;
        double lengthMM = 0.0, lengthPulse = 0.0, distanceMM = 0.0, feedMMMin = 0.0;
        double sourceFeedMMMin = 0.0, sourceVelocityPPS = 0.0;
        bool start = false, crossSegment = false;
        bool returnStart = false, requireReturnAuthorization = false;
        MotionPathCoreHoldExcursionView crossView{};
    };
    FixedCapacitySpscRing<PathCoreHoldRequest, 8U> m_pathHoldRequests{};
    // NC producer only: successful enqueue ticket binds Commit and rejects
    // duplicate starts before the RT consumer publishes its next transition.
    PathCoreHoldRequest m_pathHoldStartTicket{};
    PathCoreHoldRequest m_pathHoldProducerRequest{}; // sole NC producer scratch
    PathCoreHoldRequest m_pathHoldConsumeRequest{}; // RT scratch, never a full view on the RT stack
    std::atomic<std::uint64_t> m_pathHoldGeneration{ 1ULL };
    std::atomic<std::uint64_t> m_pathHoldCommittedRequest{ 0ULL };
    struct PathCoreHoldRuntime
    {
        MotionPathCoreHoldExcursionSnapshot status{};
        std::uint64_t generation = 0ULL;
        std::uint64_t lastSettleTick = 0ULL;
        std::uint64_t lastAcceptedStartSequence = 0ULL;
        double lengthMM = 0.0, excursionVelocity = 0.0, goal = 0.0;
        double sourceVelocityPPS = 0.0;
        double savedPlanning = 0.0, savedFinalTarget = 0.0;
        double savedMaxVelocity = 0.0, savedCruise = 0.0, savedAcc = 0.0, savedDec = 0.0;
        double savedTargetVelocity = 0.0, savedTargetEndVelocity = 0.0;
        std::array<double, MAX_AXES> actualMinimum{}, actualMaximum{};
        std::array<double, MAX_AXES> heldCommand{};
        std::uint32_t settleCycles = 0U;
        bool sourceSeen = false, endpoint = false, endpointSettled = false, startPending = false, movementOwned = false;
        bool startCaptured = false;
        MotionPathCoreHoldExcursionView crossView{};
        double spanStartS = 0.0, originalExcursionVelocity = 0.0;
        std::array<int, MAX_AXES> originalAxisIndices{};
        std::uint32_t unionAxisMask = 0U;
        int originalAxisCount = 0;
        bool unionActive = false, crossGeometryActive = false;
        bool prelaunchRejected = false; // zero-motion distance rejection owns a stationary source fence
    } m_pathHold{};
    MotionCommand m_pathHoldAdmissionFront{}; // RT-only bounded queue-peek scratch.
    static constexpr std::size_t PATH_HOLD_WORD_COUNT =
        (sizeof(MotionPathCoreHoldExcursionSnapshot) + 7U) / 8U;
    struct PathCoreHoldAtomicBank
    {
        std::atomic<std::uint64_t> sequence{ 0ULL };
        std::array<std::atomic<std::uint64_t>, PATH_HOLD_WORD_COUNT> words{};
        PathCoreHoldAtomicBank() noexcept
        {
            for (auto& word : words) word.store(0ULL, std::memory_order_relaxed);
        }
    };
    std::array<PathCoreHoldAtomicBank, 2U> m_pathHoldBanks{};
    std::atomic<std::uint64_t> m_pathHoldPublication{ 0ULL };
    std::uint64_t m_pathHoldNextPublication = 0ULL;
    bool ProcessPathCoreHoldExcursion(AxisCommand& command) noexcept;
    bool IsPathCoreHoldExcursionDriving() const noexcept;
    bool IsPathCoreHoldEffectiveMappingValid() const noexcept;
    bool ValidatePathCoreHoldCrossSource() const noexcept;
    bool ExpandPathCoreHoldAxisUnion(bool& waitForSettle) noexcept;
    void RestorePathCoreHoldAxisUnion() noexcept;
    bool BeginPathCoreHoldSpan(std::uint32_t ordinal, double startS, double targetS) noexcept;
    bool MapPathCoreHoldExcursionGeometry(const AxisCommand& command) noexcept;
    bool IsPathCoreHoldStrictlyStopped() const noexcept;
    bool IsPathCoreHoldSourceCurrent() const noexcept;
    bool ClosePathCoreHoldEndpoint(AxisCommand& command) noexcept;
    bool IsPathCoreAdmissionWaitAxisHealthy(const AxisContext& axis) const noexcept;
    void ResetPathCoreAdmissionCorrectionDiagnostic() noexcept;
    MotionPathCoreAdmissionCorrectionDecision ResolvePathCoreAdmissionPositionCorrection(
        AxisContext& axis, const AxisCommand& command,
        const MotionServoInputSnapshot& input, double& velocityPPS) noexcept;
    std::uint64_t m_pathAdmissionCorrectionFrameGeneration = 0ULL; // RT only.
    std::uint32_t m_pathAdmissionCorrectionScopeMask = 0U; // Exact LoadNext admission members.
    void UpdatePathCoreHoldExcursionEvidence() noexcept;
    void PublishPathCoreHoldExcursionSnapshot() noexcept;
    void SetPathCoreHoldPhase(MotionPathCoreHoldExcursionPhase phase, std::uint32_t reason = 0U) noexcept;

    struct MotionStopSettlePublicationPayload
    {
        MotionStopSettleSnapshot snapshot{};
        MotionStopSettleCounters counters{};
        std::array<MotionNCSettleSnapshot,
            static_cast<std::size_t>(MotionNCSettleProfile::COUNT)>
            ncSettleSnapshots{};
        std::array<MotionNCSettleCounters,
            static_cast<std::size_t>(MotionNCSettleProfile::COUNT)>
            ncSettleCounters{};
        MotionNCResetRebaseAck resetRebaseAck{};
        MotionEmergencyStopEvidence emergencyStopEvidence{};
        MotionEmergencyStopCounters emergencyStopCounters{};
    };

    static_assert(
        std::is_trivially_copyable<MotionStopSettlePublicationPayload>::value,
        "Motion stop/settle publication payload must remain trivially copyable.");

    static constexpr std::size_t MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT =
        (sizeof(MotionStopSettlePublicationPayload) +
            sizeof(std::uint64_t) - 1U) /
        sizeof(std::uint64_t);

    static_assert(
        MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT <= 192U,
        "J.5 settle publication must remain a small fixed RT payload.");

    struct MotionStopSettleAtomicBank
    {
        // Even = stable bank, odd = the 250 us producer is rewriting it.
        // This per-bank sequence closes the two-generation bank-reuse window.
        std::atomic<std::uint64_t> writeSequence{ 0ULL };
        std::array<
            std::atomic<std::uint64_t>,
            MOTION_STOP_SETTLE_PUBLICATION_WORD_COUNT> words;

        MotionStopSettleAtomicBank() noexcept
        {
            for (std::size_t i = 0U; i < words.size(); ++i)
            {
                words[i].store(0ULL, std::memory_order_relaxed);
            }
        }
    };

    std::array<MotionStopSettleAtomicBank, 2U>
        m_stopSettlePublicationBanks{};
    std::atomic<std::uint64_t> m_stopSettlePublicationGeneration{ 0ULL };

    // The J.5 stop/settle bank above is deliberately capped at 192 words and
    // is already full.  Keep the Program-Start proof in its own compact
    // double-buffered publication, then bind it to the stop bank using the
    // same 250 us publication generation and sample sequence.  A reader only
    // accepts a matched pair, so it cannot combine a ready result from one RT
    // pass with drain evidence from another pass.
    struct MotionProgramStartReadinessSnapshot
    {
        std::uint64_t stopPublicationGeneration = 0ULL;
        std::uint64_t stopSampleSequence = 0ULL;
        std::uint32_t existingAxisCount = 0U;
        std::uint32_t readyAxisCount = 0U;
        std::uint32_t notReadyAxisCount = 0U;
        std::int32_t firstNotReadyAxisIndex = -1;
    };

    static_assert(
        std::is_trivially_copyable<MotionProgramStartReadinessSnapshot>::value,
        "Program-start readiness publication must remain trivially copyable.");

    static constexpr std::size_t
        MOTION_PROGRAM_START_READINESS_PUBLICATION_WORD_COUNT =
        (sizeof(MotionProgramStartReadinessSnapshot) +
            sizeof(std::uint64_t) - 1U) /
        sizeof(std::uint64_t);

    struct MotionProgramStartReadinessAtomicBank
    {
        std::atomic<std::uint64_t> writeSequence{ 0ULL };
        std::array<
            std::atomic<std::uint64_t>,
            MOTION_PROGRAM_START_READINESS_PUBLICATION_WORD_COUNT> words;

        MotionProgramStartReadinessAtomicBank() noexcept
        {
            for (std::size_t i = 0U; i < words.size(); ++i)
            {
                words[i].store(0ULL, std::memory_order_relaxed);
            }
        }
    };

    std::array<MotionProgramStartReadinessAtomicBank, 2U>
        m_programStartReadinessPublicationBanks{};
    std::atomic<std::uint64_t>
        m_programStartReadinessPublicationGeneration{ 0ULL };

    // The fields below are owned only by the 250 us Motion runtime.
    std::uint64_t m_stopSettleNextPublicationGeneration = 0ULL;
    MotionStopSettleCounters m_stopSettleProducerCounters{};
    MotionStopSettlePrimaryBlocker m_stopSettlePreviousPrimaryBlocker =
        MotionStopSettlePrimaryBlocker::NONE;
    bool m_stopSettlePreviousStandstill = false;
    bool m_stopSettleHasPreviousSample = false;

    struct MotionNCSettleRequest
    {
        MotionNCSettleRequestSequence requestSequence =
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
        MotionNCSettleProfile profile =
            MotionNCSettleProfile::GROUP_COMPLETION;
        MotionExecutionEpoch executionEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerLease ownerLease{};
        std::uint32_t safetyRequestTicket = 0U;
        std::uint64_t safetyProvenanceGeneration = 0ULL;
        MotionNCResetExecutionState resetExecutionState{};
        bool unsupportedFaultOrEstop = false;
    };

    static_assert(
        std::is_trivially_copyable<MotionNCSettleRequest>::value,
        "NC settle request ring payload must remain trivially copyable.");

    struct MotionNCSettleTracker
    {
        MotionNCSettleRequestSequence requestSequence =
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
        MotionExecutionEpoch executionEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        MotionOwnerLease ownerLease{};
        MotionExecutionIdentity executionIdentity{};
        std::uint32_t scopeMask = 0U;
        std::uint32_t dwellCycles = 0U;
        std::uint64_t proofSequence = 0ULL;
        bool requestAccepted = false;
        bool candidate = false;
        bool settled = false;
        std::array<double, MAX_AXES> commandAnchorPulse{};
        std::array<double, MAX_AXES> actualMinimumPulse{};
        std::array<double, MAX_AXES> actualMaximumPulse{};
    };

    static constexpr std::size_t MOTION_NC_SETTLE_PROFILE_COUNT =
        static_cast<std::size_t>(MotionNCSettleProfile::COUNT);
    static constexpr std::size_t MOTION_NC_SETTLE_REQUEST_CAPACITY = 8U;
    static constexpr std::size_t MOTION_NC_RESET_BUFFER_CLEAR_BUDGET = 128U;

    FixedCapacitySpscRing<
        MotionNCSettleRequest,
        MOTION_NC_SETTLE_REQUEST_CAPACITY> m_ncSettleRequestRing{};
    std::atomic<MotionNCSettleRequestSequence>
        m_nextNCSettleRequestSequence{ 1ULL };

    std::array<MotionNCSettleTracker,
        MOTION_NC_SETTLE_PROFILE_COUNT> m_ncSettleTrackers{};
    std::array<MotionNCSettleSnapshot,
        MOTION_NC_SETTLE_PROFILE_COUNT> m_ncSettlePublishedSnapshots{};
    std::array<MotionNCSettleCounters,
        MOTION_NC_SETTLE_PROFILE_COUNT> m_ncSettleProducerCounters{};

    MotionNCSettleRequest m_activeFeedHoldNCSettleRequest{};
    MotionNCSettleRequest m_activeResetNCSettleRequest{};
    MotionNCResetRebaseAck m_ncResetRebaseAckProducer{};
    std::atomic<std::uint64_t>
        m_ncResetReleaseAuthWriteSequence{ 0ULL };
    std::atomic<MotionNCSettleRequestSequence>
        m_ncResetReleaseAuthRequestSequence{
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID };
    std::atomic<std::uint64_t>
        m_ncResetReleaseAuthPackedIdentity{ 0ULL };
    std::atomic<std::uint32_t>
        m_ncResetReleaseAuthSafetyTicket{ 0U };
    std::atomic<std::uint64_t>
        m_ncResetReleaseAuthDrainGeneration{ 0ULL };
    std::atomic<std::uint8_t>
        m_ncResetReleaseAuthState{ 0U };
    MotionNCResetRebasePhase m_ncResetRebasePhase =
        MotionNCResetRebasePhase::IDLE;
    std::uint32_t m_ncLastGroupScopeMask = 0U;
    MotionExecutionEpoch m_ncLastGroupScopeExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionExecutionIdentity m_ncLastGroupScopeExecutionIdentity{};
    std::size_t m_ncResetBufferClearAxisSlot = 0U;
    std::size_t m_ncResetBufferClearElement = 0U;
    bool m_ncResetScalarRebaseApplied = false;

    std::uint64_t m_ncSettleRuntimeCycleTick = 0ULL;
    std::uint64_t m_ncSettlePreviousRuntimeCycleTick = 0ULL;
    std::uint64_t m_ncSettleLastEvaluatedRuntimeCycleTick = 0ULL;
    bool m_ncSettleRuntimeObserved = false;
    bool m_ncSettlePreviousRuntimeCycleValid = false;
    bool m_ncSettleRuntimeCycleValid = false;
    bool m_ncSettleRuntimeCycleContiguous = false;
    bool m_ncSettleMotionPassCompleted = false;
    bool m_ncSettleHasEvaluatedRuntimeCycle = false;
    std::uint64_t m_ncNextProofSequence = 0ULL;

    void PublishStopSettleEvidence() noexcept;
    bool TryReadStopSettlePublication(
        MotionStopSettlePublicationPayload& payload) const noexcept;
    bool TryReadProgramStartReadinessPublication(
        MotionProgramStartReadinessSnapshot& readiness) const noexcept;
    MotionNCSettleRequestSequence AllocateNCSettleRequestSequence() noexcept;
    bool SubmitNCSettleRequest(const MotionNCSettleRequest& request) noexcept;
    bool IsExactResetNCSettleAuthorityCurrent(
        const MotionNCSettleRequest& request) const noexcept;
    bool TryAcquireExactResetNCSettleReservation(
        const MotionNCSettleRequest& request,
        std::uint64_t& reservedOwnerState) noexcept;
    void ReleaseExactResetNCSettleReservation(
        std::uint64_t reservedOwnerState) noexcept;
    bool ProcessNCSettleRequestsAndResetRebase() noexcept;
    void UpdateNCSettleProducer(
        MotionStopSettlePublicationPayload& payload) noexcept;
    void ResetNCSettleCandidate(
        MotionNCSettleProfile profile,
        MotionNCSettleBlocker blocker,
        bool identityReset = false,
        bool scopeReset = false) noexcept;
    std::uint32_t BuildExistingNCAxisMask() const noexcept;
    std::uint32_t BuildCurrentNCGroupAxisMask() const noexcept;
    bool HasNCResetUnsupportedMotion() const noexcept;
    bool HasNCResetActiveCompensation() const noexcept;
    MotionNCSettleBlocker ValidateNCResetCommitSeam() const noexcept;
    void BlockNCResetCommit(MotionNCSettleBlocker blocker) noexcept;
    bool ApplyNCResetScalarRebase() noexcept;
    bool ClearNCResetBuffersWithBudget() noexcept;
    bool VerifyNCResetRebaseState() const noexcept;
    bool TryPublishNCResetSafetyReleaseAuthorization() noexcept;
    bool TryClaimNCResetSafetyReleaseAuthorization(
        MotionNCSettleRequestSequence requestSequence,
        MotionNCResetSafetyReleaseAuthorization&
        authorization) noexcept;
    bool TryInvalidateNCResetSafetyReleaseAuthorization() noexcept;
    bool ReleaseNCResetSafetyReleaseAuthorizationClaim() noexcept;
    bool ConsumeNCResetSafetyReleaseAuthorizationClaim() noexcept;

    // ========================================================================
    // Stage NC-0.1D - Execution Identity + Command / Feedback Transport State
    //
    // m_executionEpochPublication：
    //     RESET / GOTO / 新程式 / 新 Cycle Start 時遞增，並把目前 Epoch、
    //     exact-Epoch Abort Policy 與 RT Pending 位元放在同一個 atomic word。
    //
    // m_nextSegmentId：
    //     全系統單調遞增，避免不同 Epoch 之間重複使用 SegmentId。
    //
    //     這可避免 G00 ABORTING Epoch 尚未被 RT 消費時，後來的 Reset Epoch
    //     被 boolean pending 合併後誤套用舊 Abort Policy。
    // ========================================================================
    // Stage NC-0.1F - Axis command mailbox and RT-only mutation requests.
    // ========================================================================
    MotionAxisCommandChannel m_axisCommandChannel{};
    std::atomic<MotionAxisCommandSequence> m_nextAxisCommandSequence{ 1ULL };
    std::array<MotionAxisCommandResult, MOTION_AXIS_RESULT_CAPACITY>
        m_axisCommandResultLedger{};
    std::atomic<std::uint64_t> m_axisCommandQueueFullCount{ 0ULL };
    std::atomic<std::uint64_t> m_axisCommandResultOverflowCount{ 0ULL };

    // bits 0..31 = causal pre-takeover Epoch, bit 32 = exact-evidence
    // obligation, bit 62 = bounded publication reservation, bit 63 = pending.
    std::atomic<std::uint64_t> m_emergencyStopRequestPublication{ 0ULL };
    std::atomic<bool> m_resetAllFaultsPending{ false };
    std::atomic<bool> m_stopGroupPending{ false };

    // RESET-only pre-ticket ingress.  This single atomic is both mailbox and
    // persistent outcome: PENDING -> APPLYING -> ACTIVE -> COMPLETED, or
    // SUPERSEDED by a higher-priority Safety action.  Keeping it in one word
    // avoids a producer/RT race between a phase store and a separate pending
    // boolean.  NC never decides from a transient mailbox bit alone.
    std::atomic<std::uint32_t> m_resetControlledStopPhase{
        static_cast<std::uint32_t>(ResetControlledStopPhase::IDLE) };

    // NC-0.2J.6.1/J.6.3.2 producer/consumer accounting. Request counters may be
    // incremented by the 10 ms control side; the remaining fields are owned by
    // the 250 us Motion consumer and copied through the coherent publication.
    std::atomic<std::uint64_t> m_emergencyStopRequestAttemptCount{ 0ULL };
    std::atomic<std::uint64_t> m_emergencyStopRequestPublishedCount{ 0ULL };
    std::atomic<std::uint64_t> m_emergencyStopRequestCoalescedCount{ 0ULL };
    std::uint64_t m_emergencyStopRTApplicationCount = 0ULL;
    std::uint64_t m_emergencyStopEpochInvalidationCount = 0ULL;
    MotionExecutionEpoch m_emergencyStopLastAppliedExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease m_emergencyStopLastAppliedOwnerLease{};
    bool m_emergencyStopLastApplyHadExecutionToInvalidate = false;

    // Persistent identity of the most recent E-stop application that really
    // invalidated an execution Epoch.  A later no-op E-stop application keeps
    // this packed record intact so a late 10 ms observer can correlate it.
    std::atomic<std::uint64_t>
        m_emergencyStopLastEpochInvalidationPacked{ 0ULL };
    std::atomic<std::uint64_t>
        m_emergencyStopLastEpochInvalidationCount{ 0ULL };
    std::atomic<std::uint64_t>
        m_emergencyStopEpochInvalidationWriteSequence{ 0ULL };

    // NC-0.2J.6.4 is intentionally independent of the stop/settle payload.
    // Packed masks: existing [0..7], ready [8..15], aligned [16..23],
    // armed [24..31], premature-motion blocked [32..39].
    std::atomic<std::uint64_t> m_startupLagArmingMasks{ 0ULL };
    std::atomic<std::uint32_t>
        m_startupLagMinimumStableSampleCount{ 0U };
    std::atomic<std::uint64_t> m_startupLagAlignmentEvents{ 0ULL };
    std::atomic<std::uint64_t> m_startupLagArmingTransitions{ 0ULL };
    std::atomic<std::uint64_t> m_startupLagReadinessResets{ 0ULL };
    std::atomic<std::uint64_t> m_startupLagPrematureMotionBlocks{ 0ULL };

    // NC-0.2K.2.1: fixed-size diagnostics for exact-stop mapping boundaries,
    // expected dropped-axis retirement, and the independent RT orphan guard.
    std::atomic<std::uint64_t> m_p1MappingBoundaryStopCount{ 0ULL };
    std::atomic<std::uint64_t> m_p1DroppedAxisRetirementCount{ 0ULL };
    std::atomic<std::uint64_t> m_p1DroppedAxisRetirementFailureCount{ 0ULL };
    std::atomic<std::uint64_t> m_p1OrphanAxisContainmentCount{ 0ULL };
    std::atomic<std::uint64_t> m_p1InvalidProducerRejectCount{ 0ULL };
    // One coherent Motion -> NC publication. bits 0..31 carry the exact
    // pre-stop execution Epoch, bits 32..62 a nonzero 31-bit request
    // sequence, and bit 63 marks an unacknowledged request.  Motion never
    // writes AlarmManager from the producer or 250 us Runtime path.
    std::atomic<std::uint64_t>
        m_p1MappingIntegrityAlarmRequestPublication{ 0ULL };
    std::atomic<std::uint32_t> m_p1LastPreviousAxisMask{ 0U };
    std::atomic<std::uint32_t> m_p1LastNextAxisMask{ 0U };
    std::atomic<std::int32_t> m_p1LastOrphanAxisIndex{ -1 };

    // Producer counters below are owned only by the 250 us Motion runtime.
    std::uint64_t m_startupLagAlignmentEventProducer = 0ULL;
    std::uint64_t m_startupLagArmingTransitionProducer = 0ULL;
    std::uint64_t m_startupLagReadinessResetProducer = 0ULL;
    std::uint64_t m_startupLagPrematureMotionBlockProducer = 0ULL;

    void AlignStartupAxisCommandToActual(AxisContext& axis) noexcept;
    bool ObserveStartupLagMonitorArming(
        AxisContext& axis,
        bool feedbackReady) noexcept;
    void PublishStartupLagArmingEvidence() noexcept;

    // Packed NC Reset safety batch:
    // bits  0..31 = the Epoch already published by NCManager::Reset()
    // bit      32 = ResetAllFaults is part of this batch
    // bit      63 = request present
    //
    // One atomic value keeps the Epoch proof and its deferred actions
    // correlated across the NC producer and the 250 us Motion consumer.
    std::atomic<std::uint64_t> m_resetSafetyBatchPending{ 0ULL };
    std::atomic<std::uint64_t>
        m_resetSafetyBatchProvenanceGeneration{ 0ULL };

    std::atomic<std::uint32_t> m_axisFaultResetPendingMask{ 0U };
    std::atomic<bool> m_safetyRecoveryRequestInProgress{ false };

    MotionAxisCommandSequence AllocateAxisCommandSequence() noexcept;
    bool SubmitAxisCommand(
        MotionAxisCommand command,
        MotionAxisCommandSequence* outSequence, bool* outIngressBusy = nullptr) noexcept;
    void ApplyPendingSafetyAndRecoveryRequests() noexcept;
    void ApplyPendingResetControlledStopRequest() noexcept;
    bool HasResetControlledStopPriorityWinner() const noexcept;
    void SupersedeResetControlledStop() noexcept;
    void CompleteResetControlledStop() noexcept;
    void TriggerGroupMappingIntegrityEmergencyStop(
        int axisIndex,
        bool forceExecutionInvalidation = false) noexcept;
    bool TryPublishGroupMappingIntegrityAlarmRequest(
        MotionExecutionEpoch executionEpoch) noexcept;
    void EmergencyStopAllAxesImpl(
        bool forceExecutionInvalidation,
        MotionExecutionEpoch causalExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID) noexcept;
    void ResetAllFaultsImpl(bool publishExecutionEpoch);
    // An operator Reset holds every final PDO velocity at zero before the
    // exact RT batch is consumed.  It must therefore retire an active group
    // as an abort, not enter the normal controlled-deceleration path which
    // depends on non-zero PDO velocity commands reaching the drives.
    void AbortActiveExecutionForResetSafetyBatch() noexcept;
    void StopGroupImpl(bool publishExecutionEpoch);
    void DrainAxisCommandMailbox() noexcept;
    void PublishAxisCommandResult(
        const MotionAxisCommand& command,
        MotionAxisCommandResultType resultType,
        MotionRejectReason rejectReason) noexcept;

    // ========================================================================
    // Stage NC-0.1E：Owner + Generation 打包成單一 atomic state。
    // Layout: bits 0..3 Owner; bit 4 handshake; bit 5 per-axis output
    // commit; bit 6 Safety action; bit 7 whole-frame send reservation;
    // bit 8 Epoch commit reservation; bits 9..31 request ticket;
    // bits 32..63 Owner Generation.
    std::atomic<std::uint64_t> m_motionOwnerState{ 0ULL };

    // NC-0.2K.6.2: a successful SAFETY owner-generation takeover is not
    // complete until a successor SAFETY Epoch has been published. The high
    // word acknowledges the exact owner generation and the low word records
    // one causally-later SAFETY Epoch. The separate expected-Epoch claim makes
    // every same-generation helper target the same single publication.
    std::atomic<std::uint64_t>
        m_safetyOwnerEpochAcknowledgement{ 0ULL };
    std::atomic<std::uint64_t>
        m_safetyOwnerEpochClaim{ 0ULL };

    // NC-0.2K.6.2: bits 8..31 of m_motionOwnerState carry one global
    // cross-mailbox Safety request ticket. RT acknowledges only a ticket
    // whose request publication and mutation are both complete. A Safety
    // release CAS includes the same ticket, so a concurrent request
    // invalidates release without refreshing Owner Generation or Epoch.
    std::atomic<std::uint32_t>
        m_safetyRequestAcknowledgedTicket{ 0U };

    // A request can be consumed and have its pending bit cleared before the
    // next coherent RT settle publication. These generations prevent an old
    // exact-drain image from becoming valid again in that short interval.
    std::atomic<std::uint64_t>
        m_executionDrainRevocationGeneration{ 0ULL };
    // High 32 bits: monotonic Safety-intent sequence. Low 32 bits: active
    // revocation publishers. Begin updates both in one atomic RMW; this is
    // the whole-PDO frame linearization fence.
    std::atomic<std::uint64_t> m_frameSafetyIntentState{ 0ULL };
    std::atomic<std::uint64_t>
        m_executionDrainObservedRevocationGeneration{ 0ULL };
    std::atomic<std::uint32_t>
        m_executionDrainRevocationPublishersInProgress{ 0U };

    // bits 0..31 = current Epoch, bit 32 = exact-Epoch Abort Policy,
    // bits 33..40 = source, bit 62 = RT commit reservation and bit 63 =
    // RT apply pending. Initial Epoch is 1 with UNKNOWN source and no
    // pending work.
    std::atomic<std::uint64_t> m_executionEpochPublication{ 1ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationAttemptCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationAcquiredCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationBlockedCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationCASLostCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationReleasedCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationReleaseFailureCount{ 0ULL };
    std::atomic<std::uint64_t> m_lifecycleCommitReservationPublisherWaitCount{ 0ULL };
    std::atomic<MotionSegmentId> m_nextSegmentId{ 1ULL };
    std::atomic<std::uint64_t> m_staleCommandDiscardCount{ 0ULL };
    std::atomic<std::uint64_t> m_motionOwnerConflictRejectCount{ 0ULL };

    // 250 us Runtime 單一擁有者。UpdateInterpolation 每圈重設，
    // 所有 Stale 清理入口共用同一份額度，確保整個 Runtime Pass
    // 最多只淘汰固定筆數，而不是每個 Helper 各自再淘汰一批。
    std::size_t m_staleCommandDiscardBudgetRemaining =
        MOTION_COMMAND_STALE_DISCARD_LIMIT_PER_RUNTIME_PASS;
    std::atomic<bool> m_safetyControlledStopInProgress{ false };
    MotionOwnerLease m_safetyControlledStopOwnerLease{};
    MotionExecutionEpoch m_safetyControlledStopEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    std::uint32_t m_safetyControlledStopRequestTicket = 0U;

    std::atomic<std::uint64_t> m_commandQueueFullRejectCount{ 0ULL };
    std::atomic<std::uint64_t> m_commandReplayOverflowCount{ 0ULL };
    std::atomic<MotionSegmentId> m_lastRejectedSegmentId{ MOTION_SEGMENT_ID_INVALID };

    // Stage NC-0.2K.6 / K.6.1: Producer and Consumer update separate atomic
    // counters.  K.6.1 authority decisions remain bounded, allocation-free and
    // owned by the existing 250 us Consumer.
    std::atomic<std::uint64_t> m_pathModeProducerSequence{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerAccepted{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerRejected{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerExactStop{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerContinuous{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerUnspecified{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerInvalid{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeProducerFingerprint{
        MOTION_COMMAND_PATH_MODE_FINGERPRINT_SEED };

    std::atomic<std::uint64_t> m_pathModeConsumerSequence{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerIngressCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerReplayCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerExactStop{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerContinuous{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerUnspecified{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerInvalid{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeConsumerFingerprint{
        MOTION_COMMAND_PATH_MODE_FINGERPRINT_SEED };
    std::atomic<std::uint64_t> m_pathModeLegacyMatches{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeLegacyMismatches{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeDriverOverrideObservations{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityAttempts{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityApplied{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityExactStop{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityContinuous{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityLegacyFallbacks{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityReplayBypasses{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityDriverBlocks{ 0ULL };
    std::atomic<std::uint64_t> m_pathModeAuthorityInvalidRejects{ 0ULL };

    // K.6.2 producer-authoritative tail.  Normal G00 planning never samples
    // RT queue/group state; lifecycle rebase enters only through
    // SyncVirtualEndPosition before the next Program Run.
    std::array<double, MAX_AXES> m_g00ProducerQueueTailPulse{};
    std::uint32_t m_g00ProducerQueueTailValidMask = 0U;
    MotionExecutionEpoch m_g00ProducerQueueTailEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease m_g00ProducerQueueTailOwnerLease{};

    // Stage NC-0.2K.6.2: one NC Producer owns these counters. writeSequence is
    // odd while the diagnostic payload is being changed and even when stable.
    std::atomic<std::uint64_t> m_queueTailWriteSequence{ 0ULL };
    std::atomic<MotionQueueTailTransactionSequence>
        m_nextQueueTailTransactionSequence{ 1ULL };
    std::atomic<std::uint64_t> m_queueTailAttempts{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailCommandAccepted{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailCommandRejected{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailRejectPreserved{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailCommandedMCSCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailLastQueuedPulseCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailRapidOverrideCommitted{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailEndpointExact{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailCaptureBound{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailInvalidInputs{ 0ULL };
    std::atomic<std::uint64_t> m_queueTailMismatches{ 0ULL };
    std::atomic<MotionQueueTailTransactionSequence>
        m_lastQueueTailTransactionSequence{
            MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID };
    std::atomic<MotionExecutionEpoch> m_lastQueueTailExecutionEpoch{
        MOTION_EXECUTION_EPOCH_INVALID };
    std::atomic<MotionSegmentId> m_lastQueueTailSegmentId{
        MOTION_SEGMENT_ID_INVALID };
    std::atomic<std::uint32_t> m_lastQueueTailAxisMask{ 0U };
    std::atomic<std::uint64_t> m_lastQueueTailCommittedFingerprint{
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED };

    std::atomic<MotionCommandSource> m_pendingCommandSource{ MotionCommandSource::UNKNOWN };

    // Stage NC-0.2D：只由 Motion Command Producer 使用。
    MotionProgramBlockCapture m_programBlockMotionCapture{};
    bool m_programBlockMotionCaptureActive = false;

    void RecordProgramBlockMotionSubmission(
        const MotionCommand& command,
        bool producerAccepted,
        MotionRejectReason immediateRejectReason) noexcept;
    bool BindProgramBlockQueueTailReceipt(
        MotionQueueTailCommitReceipt& receipt) noexcept;
    MotionQueueTailTransactionSequence
        AllocateQueueTailTransactionSequence() noexcept;
    void PublishQueueTailTransactionReceipt(
        MotionQueueTailCommitReceipt& receipt,
        bool invalidInput) noexcept;
    void RejectPathCoreRetainedPublication(
        MotionCommand& command,
        MotionExecutionEpoch plannedEpoch,
        MotionExecutionEpoch publishedEpoch,
        MotionCommandSource source,
        const MotionOwnerLease& plannedOwner,
        MotionPathCoreRetainedReceipt& result) noexcept;
    void RejectPathCoreArcPublication(
        MotionCommand& command, MotionExecutionEpoch plannedEpoch,
        MotionExecutionEpoch publishedEpoch, MotionCommandSource source,
        const MotionOwnerLease& plannedOwner, MotionFeedArcReceipt& result) noexcept;

    bool TryLineMove(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        double targetVel,
        double acc_time,
        double dec_time,
        BufferMode mode,
        MotionCommandPathMode commandPathMode,
        MotionExecutionIdentity* producedIdentity,
        MotionOwnerLease* producedOwnerLease,
        MotionExecutionEpoch plannedTailEpoch,
        const MotionOwnerLease* plannedTailOwnerLease,
        bool cncFeedLookahead = false,
        const NCPathCoreRetainedGeometry* cncCorner = nullptr,
        double cncPrefixVelocityPPS = 0.0,
        bool pathCoreFeedExactStop = false,
        MotionCommand* preparedCommand = nullptr,
        bool pathCoreRotaryFeedExactStop = false,
        const NCRotaryFeedLineValue* rotaryGeometry = nullptr,
        bool pathCoreZCFeedExactStop = false,
        const NCZCFeedLineValue* zcGeometry = nullptr,
        bool pathCoreXYZCFeedExactStop = false,
        const NCXYZCFeedLineValue* xyzcGeometry = nullptr,
        bool pathCoreXYZCUVFeedExactStop = false,
        const NCXYZCUVFeedLineValue* xyzcuvGeometry = nullptr,
        const EDM94FeedAuthorization* edm94Authorization = nullptr,
        std::uint8_t positioningOperation = 0U) noexcept;
    bool TryG00MoveInternal(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode,
        MotionCommandPathMode commandPathMode,
        double rapidOverrideCandidate,
        double* commandedMCSTail,
        bool transactionalTail,
        MotionCommandedEndpointReceiptV1* commandedEndpointReceipt = nullptr,
        bool requirePlanarBaselineMatch = false,
        bool useG53Profile = false,
        int positioningProfile = 0);
    // RT-only natural completion for frozen planar or incremental NC lines.
    bool IsFixedPlanarLineEndpointScope() const noexcept;
    bool IsCncLineEndpointScope() const noexcept;
    bool IsCncArcEndpointScope() const noexcept;
    void InvalidateCncLineEndpointProof() noexcept;
    bool HasCompletedCncLineEndpointProof(const MotionCommand& source,
        const NCTranslationSnapshot& previous, MotionExecutionEpoch epoch) const noexcept;
    bool TryCompleteFixedPlanarLineEndpoint(AxisCommand& command) noexcept;
    // Producer-only native baseline proof for dependent endpoints; never reads feedback.
    // The default mask preserves the accepted sparse planar endpoint proof.
    bool IsPlanarEndpointBasisCurrent(const double* referenceMCS,
        const std::array<double, 8U>& startPulse,
        std::uint32_t validAxisMask, std::uint32_t requiredAxisMask = 3U) const noexcept;
    void ObserveCommandPathModeProducer(
        const MotionCommand& command,
        bool accepted) noexcept;
    void CommitCommandPathModeConsumerAuthority(
        const MotionCommand& command,
        MotionCommandPathModeAuthorityDecision decision) noexcept;
    void ObserveCommandPathModeAuthorityReject(
        MotionCommandPathModeAuthorityDecision decision) noexcept;
    bool RejectFrontCommandForPathModeAuthority(
        const MotionCommand& peekedCommand,
        MotionCommandPathModeAuthorityDecision decision) noexcept;
    void RejectNonGeometryProducerMotionCommand(
        MotionCommand command,
        MotionExecutionEpoch executionEpoch,
        MotionCommandSource commandSource,
        const MotionOwnerLease& ownerLease,
        MotionRejectReason rejectReason,
        MotionExecutionIdentity* producedIdentity = nullptr,
        MotionOwnerLease* producedOwnerLease = nullptr) noexcept;
    void RejectInvalidProducerMotionCommand(
        MotionCommand command,
        MotionExecutionEpoch executionEpoch,
        MotionCommandSource commandSource,
        const MotionOwnerLease& ownerLease,
        MotionExecutionIdentity* producedIdentity = nullptr,
        MotionOwnerLease* producedOwnerLease = nullptr) noexcept;

    // --------------------------------------------------------------------
    // Final Feedback Ring：Runtime Producer -> NC Consumer
    // Producer Notice Ring：NC Producer -> Runtime Consumer
    // --------------------------------------------------------------------
    MotionFeedbackChannel m_motionFeedbackChannel{};

    // 只有 250 us Runtime 會修改 next sequence 與 Tracking State。
    MotionFeedbackSequence m_nextMotionFeedbackSequence{ 1ULL };
    MotionExecutionIdentity m_feedbackTrackedIdentity{};
    MotionOwnerLease m_feedbackTrackedOwnerLease{};
    bool m_feedbackTrackedAccepted = false;
    bool m_feedbackTrackedStarted = false;
    bool m_feedbackTrackedTerminal = true;
    // Shared queued LINEAR / ARC candidate is RT-local. NC reads only the
    // atomic completion marker after exact drain, while a lifecycle reservation
    // keeps currentCmd immutable. Existing names preserve the lifecycle seams.
    MotionExecutionIdentity m_cncLineEndpointCandidate{};
    std::atomic<MotionSegmentId> m_cncLineEndpointCompletedSegment{ MOTION_SEGMENT_ID_INVALID };

    std::atomic<std::uint64_t> m_motionFeedbackOverflowCount{ 0ULL };
    std::atomic<std::uint64_t> m_motionFeedbackProducerNoticeOverflowCount{ 0ULL };
    std::atomic<MotionFeedbackSequence> m_lastPublishedMotionFeedbackSequence{ MOTION_FEEDBACK_SEQUENCE_INVALID };
    std::atomic<MotionSegmentId> m_lastDroppedMotionFeedbackSegmentId{ MOTION_SEGMENT_ID_INVALID };
    std::atomic<MotionFeedbackType> m_lastDroppedMotionFeedbackType{ MotionFeedbackType::NONE };


    struct CollisionProtectionConfiguration {
        EDMShortCircuitProtection::Config config{};
        std::uint64_t generation=0ULL;
        bool valid=false;
    };
    struct CollisionProtectionFeedback {
        EDMShortCircuitProtection::Snapshot protection{};
        std::uint64_t configurationGeneration=0ULL,tick=0ULL;
    };
    EDM53::Channel<CollisionProtectionConfiguration,128U> m_collisionConfiguration{};
    EDM53::Channel<CollisionProtectionFeedback,128U> m_collisionFeedback{};
    std::atomic<std::uint64_t> m_collisionConfigurationRequested{0ULL};
    std::atomic<std::uint64_t> m_collisionManualStopSequence{0ULL},m_collisionManualBlockedOwnerState{0ULL};
    std::atomic<bool> m_collisionManualBypass{false},m_collisionAcceptedShort{false};
    std::uint64_t m_collisionNextConfiguration=1ULL,m_collisionRuntimeConfiguration=0ULL;
    std::uint64_t m_collisionLastSequence=0ULL,m_collisionLastSource=0ULL,m_collisionLastRevision=0ULL;
    EDM63::Route m_collisionRoute{};
    EDM63::Receipt m_collisionLastReceipt{};
    std::uint64_t m_collisionSourceIdentity=0ULL,m_collisionObserverGeneration=0ULL;
    bool m_collisionHaveRoute=false,m_collisionHaveReceipt=false;
    EDMShortCircuitProtection::Detector m_collisionDetector{};
    EDMShortCircuitProtection::Snapshot m_collisionCurrent{};
    MotionOwnerLease m_collisionManualAxisOwner{},m_collisionManualStopOwner{};
    std::uint32_t m_collisionManualAxes=0U,m_collisionManualStoppedAxes=0U;
    bool m_collisionConfigurationValid=false;
    void ObserveCollisionProtectionRuntime() noexcept;
    void TripManualCollisionRuntime(const MotionOwnerLease& owner) noexcept;
    bool IsManualCollisionBlocked(const MotionOwnerLease& owner) const noexcept;
    bool RejectCollisionPositioningProducer(MotionCommand command,MotionExecutionEpoch epoch,
        MotionCommandSource source,const MotionOwnerLease& owner,
        MotionExecutionIdentity* producedIdentity=nullptr,MotionOwnerLease* producedOwner=nullptr) noexcept;
    bool ContainCollisionPositioningRuntime(const MotionCommand& command) noexcept;

    struct RotarySpindleBinding
    {
        double resolution=0.0,lead=0.0,maxVelocity=0.0,machineOffset=0.0;
        double lowPulse=0.0,highPulse=0.0;
        std::array<double,3> travelLow{},travelHigh{};
        std::uint32_t travelMask=0U;
        std::uint8_t reverseFlags=0U;
        std::uint32_t axisType=0U;
        std::uintptr_t output=0U;
    };
    struct RotarySpindleFrozen
    {
        NCSpindleDispatch::Authority authority{};
        NCSpindleDispatch::Config config{};
        std::array<RotarySpindleBinding,3> binding{};
        std::array<std::uintptr_t,8> inputMapping{},outputMapping{};
        std::uint32_t axisMask=0U,slots=0U;
    };
    struct RotarySpindleSent
    {
        std::uint64_t tick=0ULL,request=0ULL;
        double commandRpm=0.0;
        MotionExecutionEpoch epoch=0U;
        MotionOwnerLease owner{};
        std::uint32_t safetyTicket=0U;
        bool exact=false,safetyZero=false;
    };
    struct RotarySpindleRuntime
    {
        RotarySpindleFrozen frozen{};
        NCRotarySpindle::Controller controller{};
        MotionRotarySpindleFeedback feedback{};
        std::array<NCSpindleDispatch::Packet,3> packet{};
        std::array<MotionRotarySpindleProof::ReceiptWindow,3> window{};
        std::array<RotarySpindleSent,3> sent{};
        std::array<MotionServoInputSnapshot,3> input{};
        std::array<std::uint32_t,3> previousRaw{};
        std::array<std::uint64_t,3> rawTick{};
        std::array<double,3> rawUnwrapped{};
        std::array<double,3> actualRpm{},actualPulse{},commandRpm{};
        std::array<std::int32_t,3> hardwarePps{};
        std::array<unsigned,3> zeroSamples{},outputMissTicks{};
        std::array<std::uint64_t,3> deadlineSequence{},deadlineTick{},qualifiedSequence{};
        std::array<bool,3> rawSeen{},sourceValid{},havePacket{};
        std::uint64_t tick=0ULL,beginTick=0ULL,publication=0ULL,startFloor=0ULL;
        bool loaded=false,initialized=false,fault=false,handback=false;
        std::uint32_t faultCode=0U;
    };
    EDM53::Channel<RotarySpindleFrozen,2048U> m_rotarySpindleBegin{};
    std::array<EDM53::Channel<NCSpindleDispatch::Packet,512U>,3> m_rotarySpindleRequest{};
    EDM53::Channel<MotionRotarySpindleFeedback,4096U> m_rotarySpindleFeedback{};
    std::atomic<std::uint64_t> m_rotarySpindleClaim{0ULL},m_rotarySpindleRelease{0ULL};
    std::uint64_t m_rotarySpindleProducerRun=0ULL;
    std::array<std::uint64_t,3> m_rotarySpindleProducerSequence{};
    RotarySpindleRuntime m_rotarySpindle{};
    bool IsRotarySpindleConfigurationCurrent(const RotarySpindleFrozen& frozen) const noexcept;
    bool IsRotarySpindleAuthorityCurrent(const RotarySpindleFrozen& frozen) const noexcept;
    void ObserveRotarySpindleRuntimeInput() noexcept;
    bool ApplyRotarySpindleAxis(AxisContext& axis,const MotionServoInputSnapshot& input,
        std::int32_t& hardwarePps) noexcept;
    void ContainRotarySpindleRuntime(MotionRotarySpindle::Fault fault,bool requestSafety) noexcept;
    void BeginRotarySpindleSend(ServoOutputFrameReservation& reservation) noexcept;
    bool ValidateRotarySpindleSend(const ServoOutputFrameReservation& reservation) noexcept;
    bool SealRotarySpindleSerializedImage(ServoOutputFrameReservation& reservation,
        const std::uint8_t* ioMapBase,std::size_t ioMapSize,
        const std::uint8_t* payload,std::size_t payloadSize) noexcept;
    void ObserveRotarySpindleSend(const ServoOutputFrameReservation& reservation) noexcept;
    void PublishRotarySpindleRuntime() noexcept;
    bool HandbackRotarySpindleAxes(bool safety) noexcept;

    struct G80AxisBinding
    {
        double resolution=0.0,lead=0.0,maxVelocity=0.0,accSeconds=0.0,decSeconds=0.0;
        double machineOffset=0.0,scaleRatio=0.0,maxDeviation=0.0;
        std::uintptr_t scalePointer=0U;
        std::uint32_t feedbackMode=0U;
        std::uint8_t reverseFlags=0U;
    };
    struct G80Frozen
    {
        G80::Request request{};
        MotionCommand command{};
        G80::Point6 pulsePerNative{};
        std::array<G80AxisBinding,6U> axis{};
        std::uint32_t axisMask=0U;
    };
    EDM53::Channel<G80Frozen,4096U> m_g80Requests{}; // NC-only publisher
    EDM53::Channel<MotionG80Feedback,2048U> m_g80Feedback{}; // RT-only publisher
    std::atomic<std::uint64_t> m_g80Claim{0ULL};
    std::atomic<std::uint64_t> m_g80EndpointCommitted{0ULL};
    std::atomic<std::uint64_t> m_g80Barrier{0ULL};
    struct G80ResetScope {
        std::uint64_t session=0ULL,previousProvenance=0ULL,expectedProvenance=0ULL,expectedSafetyOwnerState=0ULL,previousSafetyOwnerState=0ULL;
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };
    EDM53::Channel<G80ResetScope,128U> m_g80ResetScope{}; // NC Reset-only intent, never motion authority.
    std::uint64_t m_g80ProducerHighWater=0ULL;
    G80Frozen m_g80Frozen{}; // RT-owned after exact ring dequeue
    G80::Cycle m_g80Cycle{};
    G80::Input m_g80Input{};
    std::uint64_t m_g80InputSession=0ULL;
    MotionExecutionIdentity m_g80InputIdentity{};
    G80::VoltageSimulator m_g80VoltageSimulator{};
    std::uint64_t m_g80SimulationSession=0ULL;
    MotionExecutionIdentity m_g80SimulationIdentity{};
    bool m_g80SimulationReady=false;
    MotionG80Feedback m_g80Current{};
    EDM63::Route m_g80SourceRoute{};
    std::uint32_t m_g80ProfileRevision=0U,m_g80CalibrationRevision=0U;
    std::uint64_t m_g80SourceSequence=0ULL,m_g80ZeroSentTick=0ULL;
    std::uint64_t m_g80AdmissionTick=0ULL,m_g80AdmissionSession=0ULL;
    std::uint32_t m_g80ZeroSettleTicks=0U,m_g80TerminalWaitTicks=0U;
    std::array<PidConfig,6U> m_g80SavedPid{};
    std::array<G80::DerivativeState,6U> m_g80Derivative{};
    bool m_g80Running=false,m_g80ZeroDemand=false;
    bool IsG80BarrierBlocking(const MotionCommand& command) const noexcept;
    bool IsG80IncomingReady(const AxisContext& axis) const noexcept;
    bool RetireG80Axes() noexcept;
    bool IsG80CommandAuthorized(const MotionCommand& command) const noexcept;
    bool LoadG80Command(const MotionCommand& command) noexcept;
    void ObserveG80RuntimeInput() noexcept;
    void ObserveG80Send(const MotionServoHandoffSample& sample) noexcept;
    bool StepG80Runtime() noexcept;
    bool SelectG80Gains(AxisContext& axis) noexcept;
    bool ApplyG80Derivative(AxisContext& axis,double error,double& velocity) noexcept;
    void RestoreG80PidLimits() noexcept;
    bool G80DemandsZero(const AxisContext& axis) const noexcept;
    void FailG80Runtime(std::uint32_t fault, bool emergency) noexcept;
    bool IsG80ResetOutputHeld(bool applyingExactBatch=false) const noexcept;
    void AdvanceG80ResetScope(std::uint64_t previousProvenance,std::uint64_t nextProvenance,
        std::uint64_t expectedSafetyOwnerState=0ULL) noexcept;
    bool RetireG80ForResetSafetyBatch(MotionExecutionEpoch resetEpoch,const MotionOwnerLease& resetOwner,
        std::uint32_t resetTicket,std::uint64_t resetProvenance) noexcept;
    void PublishG80Runtime() noexcept;
    struct G76SphereAxisBinding
    {
        double resolution=0.0,lead=0.0,maxVelocity=0.0,accSeconds=0.0,decSeconds=0.0;
        double machineOffset=0.0,scaleRatio=0.0,maxDeviation=0.0,followingWindow=0.0,stopSeconds=0.0;
        double rotaryModulo=0.0;
        std::uintptr_t scalePointer=0U;
        std::uint32_t feedbackMode=0U;
        std::uint8_t reverseFlags=0U,axisType=0U,configurationFlags=0U;
        std::array<double,6U> travelLimits{},compensationParameters{};
    };
    struct G76SphereFrozen
    {
        NCG76SphereCycle::Request request{};
        MotionCommand command{};
        std::array<double,8U> pulsePerNative{};
        std::array<G76SphereAxisBinding,6U> axis{};
        std::uint32_t axisMask=0U;
    };
    EDM53::Channel<G76SphereFrozen,4096U> m_g76SphereRequests{};
    EDM53::Channel<MotionG76SphereFeedback,2048U> m_g76SphereFeedback{};
    std::atomic<std::uint64_t> m_g76SphereClaim{0ULL},m_g76SphereEndpointCommitted{0ULL},m_g76SphereBarrier{0ULL};
    struct G76SphereControl
    {
        std::uint64_t session=0ULL,holdSequence=0ULL,startSequence=0ULL;
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
        bool hold=false;
    };
    FixedCapacitySpscRing<G76SphereControl,8U> m_g76SphereControls{};
    G76SphereControl m_g76SphereProducerControl{},m_g76SphereIncomingControl{};
    struct G76SphereResetScope
    {
        std::uint64_t session=0ULL,previousProvenance=0ULL,expectedProvenance=0ULL;
        std::uint64_t expectedSafetyOwnerState=0ULL,previousSafetyOwnerState=0ULL;
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };
    EDM53::Channel<G76SphereResetScope,128U> m_g76SphereResetScope{};
    struct G76SphereParentBinding
    {
        std::uint64_t session=0ULL,dispatch=0ULL,commit=0ULL;
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };
    EDM53::Channel<G76SphereParentBinding,128U> m_g76SphereParentBinding{};
    std::atomic<std::uint64_t> m_g76SphereParentBound{0ULL};
    std::uint64_t m_g76SphereProducerHighWater=0ULL;
    G76SphereFrozen m_g76SphereFrozen{};
    NCG76SphereCycle::Cycle m_g76SphereCycle{};
    NCG76SphereCycle::Input m_g76SphereInput{};
    MotionExecutionIdentity m_g76SphereInputIdentity{};
    std::uint64_t m_g76SphereInputSession=0ULL,m_g76SphereZeroSentTick=0ULL;
    std::uint64_t m_g76SphereAdmissionTick=0ULL,m_g76SphereAdmissionSession=0ULL;
    std::uint64_t m_g76SpherePhysicalTick=0ULL;
    std::array<double,8U> m_g76SpherePhysicalMinimum{},m_g76SpherePhysicalMaximum{},m_g76SpherePhysicalReference{};
    bool m_g76SpherePhysicalRange=false,m_g76SphereRunning=false,m_g76SphereZeroDemand=false;
    MotionG76SphereFeedback m_g76SphereCurrent{};
    bool IsG76SphereBarrierBlocking(const MotionCommand& command) const noexcept;
    bool IsG76SphereCommandAuthorized(const MotionCommand& command) const noexcept;
    bool LoadG76SphereCommand(const MotionCommand& command) noexcept;
    void ObserveG76SphereRuntimeInput() noexcept;
    void ObserveG76SphereSend(const MotionServoHandoffSample& sample) noexcept;
    bool StepG76SphereRuntime() noexcept;
    bool G76SphereDemandsZero(const AxisContext& axis) const noexcept;
    void FailG76SphereRuntime(std::uint32_t fault,bool emergency) noexcept;
    bool IsG76SphereResetOutputHeld(bool applyingExactBatch=false) const noexcept;
    void AdvanceG76SphereResetScope(std::uint64_t previousProvenance,std::uint64_t nextProvenance,
        std::uint64_t expectedSafetyOwnerState=0ULL) noexcept;
    bool RetireG76SphereForResetSafetyBatch(MotionExecutionEpoch resetEpoch,const MotionOwnerLease& resetOwner,
        std::uint32_t resetTicket,std::uint64_t resetProvenance) noexcept;
    void PublishG76SphereRuntime() noexcept;
    MotionSegmentId AllocateMotionSegmentId() noexcept;
    void AssignExecutionIdentity(
        MotionCommand& command,
        MotionExecutionEpoch exactEpoch,
        MotionCommandSource exactSource,
        const MotionOwnerLease& exactOwnerLease) noexcept;
    bool IsCommandFromCurrentEpoch(const MotionCommand& command) const noexcept;
    bool IsCommandOwnerLeaseCurrent(const MotionCommand& command) const noexcept;
    // BASE79F: private RT-owned curve storage is allocated outside the group.
    std::unique_ptr<MotionEccentricCConsumerState> m_eccentricCConsumer;
    void ClearEccentricCConsumer() noexcept;
    bool LoadEccentricCCommand(const MotionCommand& command) noexcept;
    bool IsEccentricCCommandLiveValid(const MotionCommand& command,
        const NCEccentricCRuntimeValue& runtime, bool requireStart) const noexcept;
    MotionEccentricCContext GetEccentricCContext() const noexcept;
    bool StepEccentricCProfile(AxisCommand& command, NCEccentricCProfilePoint& point) noexcept;
    bool BindEccentricCStop(const MotionEccentricCContext& context) noexcept;
    bool RetireEccentricCCommand() noexcept;

    // BASE79F: private RT sample reservation, never an NC admission token.
    struct EccentricCCommitReservation
    {
        AlarmManager::MotionAdmissionReservation alarmAdmission{};
        std::uint64_t baseOwnerState = 0ULL;
        std::uint64_t reservedOwnerState = 0ULL;
        std::uint64_t baseExecutionPublication = 0ULL;
        std::uint64_t reservedExecutionPublication = 0ULL;
        std::uint64_t safetyIntentState = 0ULL;
        std::uint64_t revocationGeneration = 0ULL;
        MotionExecutionIdentity sourceExecution{};
        MotionOwnerLease sourceOwner{};
        bool ownerAcquired = false;
        bool epochAcquired = false;
        bool acquired = false;
        bool releaseFailed = false;
    };
    bool TryAcquireEccentricCCommitReservation(const MotionEccentricCContext& context,
        EccentricCCommitReservation& reservation) noexcept;
    bool IsEccentricCCommitReservationCurrent(const MotionEccentricCContext& context,
        const EccentricCCommitReservation& reservation) const noexcept;
    bool ReleaseEccentricCCommitReservation(EccentricCCommitReservation& reservation) noexcept;

    // BASE79D: RT-owned preflight only; does not grant admission or write outputs.
    bool IsEccentricCStartBindingCurrent(const MotionCommand& command,
        const NCEccentricCRuntimeValue& runtime) const noexcept;
    MotionRejectReason GetCommandAuthorizationFailure(
        const MotionCommand& command, EDM174Leaf::Sample* edm174Trace = nullptr) const noexcept;

    static std::uint64_t PackMotionOwnerState(
        MotionOwner owner,
        MotionOwnerGeneration generation,
        std::uint32_t safetyRequestTicket = 0U,
        bool safetyHandshakeInProgress = false,
        bool safetyActionPending = false) noexcept;
    static MotionOwnerLease UnpackMotionOwnerState(
        std::uint64_t packed) noexcept;
    static std::uint32_t UnpackMotionOwnerSafetyRequestTicket(
        std::uint64_t packed) noexcept;
    static bool UnpackMotionOwnerSafetyHandshake(
        std::uint64_t packed) noexcept;
    static bool UnpackMotionOwnerSafetyActionPending(
        std::uint64_t packed) noexcept;
    static MotionOwnerGeneration NextMotionOwnerGeneration(
        MotionOwnerGeneration current) noexcept;

    bool EnsureSafetyMotionOwnerEpoch(
        const MotionOwnerLease& safetyLease,
        std::uint32_t safetyRequestTicket) noexcept;
    std::uint32_t PublishSafetyMotionRequestTicket(
        bool requiresRTApplication) noexcept;
    bool EnsureSafetyMotionActionTicket(
        std::uint32_t& safetyRequestTicket) noexcept;
    bool CompleteSafetyMotionActionTicket(
        std::uint32_t safetyRequestTicket) noexcept;
    bool TryPublishEmergencyStopMailbox(
        MotionExecutionEpoch causalExecutionEpoch,
        bool evidenceRequired = false) noexcept;
    bool PublishEmergencyStopMailboxContentionFallback(
        MotionExecutionEpoch causalExecutionEpoch,
        bool evidenceRequired) noexcept;
    bool TryClaimEmergencyStopMailbox(
        std::uint64_t& claimedRequest) noexcept;
    bool TryClaimEmergencyStopMailboxForDirectContainment(
        std::uint64_t& claimedRequest,
        bool& evidenceReservationHeld) noexcept;
    bool FinalizeDirectContainmentEmergencyStopMailbox(
        std::uint64_t claimedRequest,
        bool evidenceAlreadyPersistent) noexcept;
    bool TryClaimResetSafetyBatch(
        std::uint64_t& claimedBatch,
        std::uint64_t& provenanceGeneration) noexcept;
    bool FinalizeClaimedResetSafetyBatch(
        std::uint64_t claimedBatch) noexcept;
    MotionOwnerLease TryTakeSafetyMotionOwnerForTicket(
        std::uint32_t safetyRequestTicket) noexcept;
    bool HasUnacknowledgedSafetyMotionRequest() const noexcept;
    void TryAcknowledgeAppliedSafetyMotionRequests() noexcept;
    bool IsSafetyControlledStopAuthorized(
        int contextAxisSlot,
        const ServoOutputFrameReservation* sendReservation = nullptr) const noexcept;
    void BeginExecutionDrainAcknowledgementRevocation() noexcept;
    bool BeginResetSafetyProvenanceOperation(
        std::uint64_t expectedProvenanceGeneration,
        std::uint64_t& operationProvenanceGeneration) noexcept;
    void EndExecutionDrainAcknowledgementRevocation() noexcept;
    void RevokeExecutionDrainAcknowledgement() noexcept;

    MotionFeedbackSequence AllocateMotionFeedbackSequence() noexcept;
    bool TryQueueProducerFeedbackNotice(
        const MotionCommand& command,
        MotionFeedbackType type,
        MotionRejectReason rejectReason,
        std::uint32_t errorCode,
        double progress) noexcept;
    void DrainProducerFeedbackNotices() noexcept;
    bool PublishMotionFeedback(MotionFeedbackEvent event) noexcept;
    bool PublishMotionFeedbackForCommand(
        const MotionCommand& command,
        MotionFeedbackType type,
        MotionRejectReason rejectReason,
        std::uint32_t errorCode,
        double progress) noexcept;
    void TrackMotionCommandAccepted(const MotionCommand& command) noexcept;
    void TrackMotionCommandStarted(const MotionCommand& command) noexcept;
    void CompleteTrackedMotionCommand(
        const MotionCommand& command) noexcept;
    void AbortTrackedMotionCommand() noexcept;
    void FaultTrackedMotionCommand(
        std::uint32_t errorCode,
        MotionRejectReason reason) noexcept;
    bool IsTrackedMotionCommand(
        const MotionCommand& command) const noexcept;
    bool IsTerminalizedReplayOrTrackedCommand(
        const MotionCommand& command) const noexcept;
    void RejectMotionCommand(
        const MotionCommand& command,
        MotionRejectReason reason,
        std::uint32_t errorCode) noexcept;
    double GetTrackedMotionProgress() const noexcept;

    bool TryEnqueueMotionCommand(const MotionCommand& command) noexcept;
    bool TryEnqueueMotionCommandPair(
        const MotionCommand& first, const MotionCommand& second) noexcept;
    bool TryEnqueueMotionCommandBatch(const MotionCommand* commands, std::size_t count, bool traceG76Start = false) noexcept;
    bool IsG76MixedPreparedCommandGeometryValid(const MotionCommand& command) const noexcept;
    bool TryPeekNextMotionCommand(MotionCommand& command) const noexcept;
    bool TryPeekQueuedMotionCommandAt(
        std::size_t offset,
        MotionCommand& command) const noexcept;
    bool TryDequeueNextMotionCommand(MotionCommand& command) noexcept;
    bool TryRequeueMotionCommandFront(const MotionCommand& command) noexcept;

    bool HasExactExecutionDrainAcknowledgementImpl(
        MotionExecutionEpoch executionEpoch,
        const MotionOwnerLease& ownerLease,
        bool ownsCommitReservation) const noexcept;

    bool TryAcquireLifecycleCommitReservation(
        const MotionExecutionIdentity& execution,
        std::uint64_t& reservationToken) noexcept;
    void ReleaseLifecycleCommitReservation(
        std::uint64_t reservationToken) noexcept;
    class LifecycleCommitReservationGuard
    {
    public:
        LifecycleCommitReservationGuard(
            MotionCore& owner,
            const MotionExecutionIdentity& execution) noexcept;
        ~LifecycleCommitReservationGuard() noexcept;

        LifecycleCommitReservationGuard(
            const LifecycleCommitReservationGuard&) = delete;
        LifecycleCommitReservationGuard& operator=(
            const LifecycleCommitReservationGuard&) = delete;

        bool IsAcquired() const noexcept;
        void Release() noexcept;

    private:
        MotionCore* m_owner = nullptr;
        std::uint64_t m_reservationToken = 0ULL;
    };

    MotionExecutionEpoch PublishNewExecutionEpoch(
        MotionCommandSource source,
        bool abortActiveCommand) noexcept;
    MotionExecutionEpoch RequestAbortingExecutionEpoch(
        MotionCommandSource source) noexcept;
    bool TryPublishOwnerAuthorizedAbortingExecutionEpoch(
        MotionCommandSource source,
        MotionExecutionEpoch expectedEpoch,
        const MotionOwnerLease& expectedOwnerLease,
        MotionExecutionEpoch& publishedEpoch) noexcept;

    bool HasPendingExecutionEpochChange() const noexcept;
    void DiscardStaleQueuedCommands();
    void ApplyPendingExecutionEpochChange();

    bool IsNCTranslationAxisIdentityCurrent(const NCTranslationSnapshot& snapshot) const noexcept;
    // BASE68 producer-only accepted rotary source. Its owner/epoch and shared
    // pulse-tail tag must still match; RESET/restart/other motions revoke it.
    MotionCncPathTail m_rotaryFeedProducerTail{};
    // BASE70 private native-basis proof, never queued geometry authority.
    // axisMask records only independently proved native XYZ/C bases; shared
    // pulse tail and owner/epoch bind every retained bit. No queue permission.
    MotionCncPathTail m_zcFeedProducerTail{};
    NCTranslationSnapshot m_pendingTranslation{}; // NC producer only.
    MotionNCTranslationPublication m_translationPublication{};
    bool IsPendingCommandTranslationValid(MotionCommandSource source) const noexcept
    {
        if (source != MotionCommandSource::NC_MEMORY) return true;
        if (IsNCTranslationSnapshotEmpty(m_pendingTranslation))
            return m_pendingToolMode == 49 && m_pendingToolRadMode == 40 && m_pendingDCode == 0 && !m_pendingG168Active && m_pendingWCode == 0 &&
                !m_pendingG68Active && !m_pendingG51Active && m_pendingMirrorMask == 0U &&
                !m_pendingG16Active && GetActiveTranslationGeneration() == 0ULL;
        return IsNCTranslationSourceAllowed(m_pendingSourceWCS, m_pendingTranslation) &&
            (m_pendingG162Active ? 1U : 0U) == m_pendingTranslation.axisIdentity.eccentricEnabled &&
            (m_pendingIsAbsoluteMode ? 90 : 91) == m_pendingTranslation.distanceMode &&
            m_pendingPlaneMode == m_pendingTranslation.rotationPlane &&
            IsNCTranslationToolModeAllowed(m_pendingToolMode, m_pendingTranslation) &&
            m_pendingHCode == m_pendingTranslation.toolHCode &&
            (m_pendingToolMode == 49 || !m_pendingG162Active) &&
            m_pendingToolRadMode == m_pendingTranslation.cutterMode &&
            m_pendingDCode == m_pendingTranslation.cutterD &&
            (m_pendingTranslation.cutterMode == 40 ||
                (IsNCTranslationCutterNotationAllowed(m_pendingPlaneMode, m_pendingIsAbsoluteMode ? 90 : 91,
                    m_pendingG16Active ? 16 : 15) && !m_pendingG162Active)) &&
            IsNCTranslationRotationModeAllowed(m_pendingG68Active, m_pendingG68Angle,
                m_pendingPlaneMode, m_pendingTranslation) &&
            (!m_pendingG68Active || !m_pendingG162Active) &&
            IsNCTranslationWorkModeAllowed(m_pendingG168Active, m_pendingWCode, m_pendingTranslation) &&
            (!m_pendingG168Active || !m_pendingG162Active) &&
            m_pendingG51Active == (m_pendingTranslation.scalingMode == 51) &&
            std::memcmp(&m_pendingScaleRatio, &m_pendingTranslation.scalingFactor, sizeof(double)) == 0 &&
            static_cast<std::uint32_t>(m_pendingMirrorMask) == m_pendingTranslation.mirrorMask &&
            (!NCTranslationHasScaleMirror(m_pendingTranslation) || !m_pendingG162Active) &&
            IsNCTranslationPolarModeAllowed(m_pendingG16Active, m_pendingTranslation) &&
            (!m_pendingG16Active || (m_pendingIsAbsoluteMode && !m_pendingG162Active)) &&
            MatchesNCTranslation(m_pendingTranslation);
    }
    bool IsPendingFixedTranslationSourceAllowed() const noexcept
    {
        return IsNCTranslationSnapshotEmpty(m_pendingTranslation) ?
            (m_pendingSourceWCS == 54 && m_pendingToolMode == 49 && m_pendingToolRadMode == 40 && m_pendingDCode == 0 && !m_pendingG168Active && m_pendingWCode == 0 &&
                !m_pendingG68Active && !m_pendingG51Active && m_pendingMirrorMask == 0U &&
                !m_pendingG16Active && GetActiveTranslationGeneration() == 0ULL) :
            IsPendingCommandTranslationValid(MotionCommandSource::NC_MEMORY);
    }
    int m_pendingSourcePC = 0;
    int m_pendingSourceWCS = 54; // 預設 G54

    // 新增暫存變數
    int m_pendingToolMode = 49;
    int m_pendingHCode = 0;

    int m_pendingToolRadMode = 40;
    int m_pendingDCode = 0;

    bool m_pendingIsAbsoluteMode = true;

    bool m_pendingG68Active = false;
    double m_pendingG68Angle = 0.0; // 預設 0 度

    bool m_pendingG168Active = false;
    int m_pendingWCode = 0; // 預設 W0

    bool m_pendingG51Active = false;
    double m_pendingScaleRatio = 1.0;

    uint8_t m_pendingMirrorMask = 0;

    bool m_pendingG16Active = false;

    bool m_pendingG162Active = true;
    int m_pendingPlaneMode = 17;

    void Calc_Trajectory_Trapezoidal(AxisContext& axis, AxisCommand& outCmd); // 計算定位模式的梯形速度規劃 (S-Curve 前置)
    void Calc_Trajectory_Velocity(AxisContext& axis, AxisCommand& outCmd); // 計算速度模式的斜坡變速規劃
    void Calc_Trajectory_MPG(AxisContext& axis, AxisCommand& outCmd);

    // 執行 PID 運算、前饋控制以及安全 Lag 監控--------------------------------------------------------------------
    template <typename DriveType>

    void Run_Servo_Loop(DriveType& servo, AxisContext& axis, const AxisCommand& cmd,
        const MotionServoInputSnapshot& input, EDM174Leaf::Sample* edm174Trace = nullptr);
    // FIX6: the public three-argument template remains the untraced wrapper.
    template <typename DriveType>
    void UpdateMotionObserved(DriveType& servo, AxisContext& axis,
        const MotionServoInputSnapshot& input, EDM174Leaf::Sample* edm174Trace,
        EDM206::InputCapture* edm207Capture = nullptr);
    void DetermineActiveGainSet(AxisContext& axis, bool* physicalFixtureIntegralHold = nullptr,
        EDM174Leaf::Sample* edm174Trace = nullptr);
    bool IsEDM165FlushGainActive(const AxisContext& axis, EDM174Leaf::Sample* edm174Trace = nullptr) const noexcept;

    double PlanTrapezoidal(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt);
    double PlanTrapezoidal_B2(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt);

    int Sgn(double val); // 符號函數

    // =========================================================
    // Stage 11E.5 - centralized Servo OUTPUT command seam.
    //
    // Before E4 runtime qualification:
    //     legacy pOutput producer.
    //
    // After E4 runtime qualification:
    //     structured AxisIndex producer.
    //
    // Any structured fault boot-latches rollback to legacy.
    // =========================================================

    void WriteServoControlWordCommand(
        ServoOutput* output,
        int axisIndex,
        uint16_t value);

    void WriteServoTargetVelocityCommand(
        ServoOutput* output,
        int axisIndex,
        int32_t value, EDM174Leaf::Sample* edm174Trace = nullptr);

    enum class ServoOutputImageProofMode : std::uint8_t
    {
        INVALID = 0,
        ZERO_ONLY,
        NORMAL,
        CONTROLLED_STOP,
        IDLE_HOLD
    };

    struct ServoOutputImageProof
    {
        std::uint64_t ownerState = 0ULL;
        std::uint64_t executionPublication = 0ULL;
        std::uint64_t frameSafetyIntentState = 0ULL;
        std::uint64_t admissionCorrectionGeneration = 0ULL;
        std::uint64_t alarmSafetyIntentState = 0ULL;
        std::uint64_t generation = 0ULL;
        std::uint32_t alarmUpdateCount = 0U;
        std::uint32_t slotMask = 0U;
        ServoOutputImageProofMode mode =
            ServoOutputImageProofMode::INVALID;
        std::array<std::int32_t, MAX_AXES> targetVelocity{};
        std::uint64_t sourceRuntimeTick = 0ULL;
        std::uint32_t diagnosticAxisMask = 0U;
        std::uint32_t sourceInputMask = 0U;
        std::uint32_t reverseMask = 0U;
        std::array<std::uint16_t, MAX_AXES> sourceStatusWord{};
        std::array<std::int8_t, MAX_AXES> sourceMode{};
        MotionPbcXSource pbcXSource{};
        PbcHomeProbeCommand homeProbe{};
    };

    // These fields are single-writer/single-reader on the same Priority-64
    // callback: UpdateAllMotion builds the next image only after the current
    // frame round trip, and EtherCAT captures it at the following send point.
    ServoOutputImageProof m_servoOutputImageProof{};
    std::uint64_t m_servoOutputImageProofGeneration = 0ULL;
    MotionServoHandoffMonitor m_servoHandoffMonitor{};
    // PBC-3D: one atomic ingress state closes both command producer channels
    // during an actual coordinate commit. No RT spinning or heap work.
    MotionPbcIngressGate m_pbcReferenceIngress{};
    MotionPbcReferenceAudit m_pbcReferenceAudit{};
    std::uint64_t m_pbcXFeedbackTick = 0ULL;
    std::uint64_t m_pbcResetPreproofTick = 0ULL;
    MotionNCSettleRequestSequence m_pbcResetPreproofRequest = 0ULL;
    MotionExecutionEpoch m_pbcResetPreproofEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerGeneration m_pbcResetPreproofOwner = MOTION_OWNER_GENERATION_INVALID;
    std::atomic<bool> m_pbcXRetainRecoveryRequired{ false };
    std::uint64_t m_pbcXRetainLossSequence = 0ULL;
    std::uint64_t m_pbcXRetainCommittedLossSequence = 0ULL;
    std::uint64_t m_pbcXRetainRecoveryTicket = 0ULL;
    pbc::CoordinateFrame m_pbcXRetainFrame{};
    CompensationEngine::XRetainRecoveryKind m_pbcXRetainRecoveryKind =
        CompensationEngine::XRetainRecoveryKind::None;
    std::uint64_t m_pbcXRetainOriginReference = 0ULL;
    std::uint64_t m_pbcXRetainAppliedReference = 0ULL;
    MotionNCSettleRequestSequence m_pbcXRetainResetRequest = 0ULL;
    bool LatchPbcXRetainRecovery(AxisContext& axis,
        CompensationEngine::XRetainRecoveryKind kind) noexcept;
    bool HoldPbcXRetainRecoveryOutput(AxisContext& axis, ServoOutput* output) noexcept;
    bool IsPbcXRetainRecoveryPreproofEligible(const AxisContext& axis) const noexcept;
    void CompletePbcXRetainRecoveryAfterReset() noexcept;
    bool IsPbcXFeedbackCurrent(const AxisContext& axis) const noexcept;
    // PBC-3I: one RT writer; supervisory readers use a bounded atomic snapshot.
    MotionPbcHomeCaptureProof m_pbcXHomeCaptureProof{};
    PbcHomeProbeCommand m_pbcXHomeProbeCommand{};
    ServoOutputFrameReservation m_pbcXHomeProbeCanonicalSend{};
    bool m_pbcXHomeProbeSendPending = false;
    bool m_pbcXHomeCaptureSemanticSource = false;
    static constexpr std::size_t PBC_HOME_CAPTURE_WORDS =
        (sizeof(MotionPbcHomeCaptureSnapshot) + sizeof(std::uint64_t) - 1U) / sizeof(std::uint64_t);
    std::array<std::atomic<std::uint64_t>, PBC_HOME_CAPTURE_WORDS> m_pbcXHomeCaptureWords{};
    std::atomic<std::uint64_t> m_pbcXHomeCapturePublication{ 0ULL };
    std::atomic<bool> m_pbcXHomeCaptureSupported{ false };
    bool IsPbcXHomeCaptureConfiguration(const AxisContext& axis) const noexcept;
    bool BuildPbcXHomeCaptureKey(const AxisContext& axis, MotionPbcHomeCaptureKey& key,
        const ServoOutputFrameReservation* send = nullptr) const noexcept;
    void ObservePbcXHomeCapture(const AxisContext& axis,
        const MotionServoInputSnapshot& input, bool semanticSource) noexcept;
    void PublishPbcXHomeCaptureSnapshot() noexcept;
    void InvalidatePbcXHomeCapture() noexcept;
    // PBC-3J: only sealed physical-X exact zero calibration joins OFF identity.
    bool IsPbcCoordinateIdentity(const AxisContext& axis) const noexcept;
    bool IsPbcXHomeRawIdentity(const AxisContext& axis) const noexcept;
    bool IsPbcControlScope(const AxisContext& axis) const noexcept;
    void PublishPbcXZeroOnlyHomeIdentity() noexcept;
    std::atomic<bool> m_pbcXZeroOnlyHomeIdentity{ false };
    std::atomic<bool> m_pbcXStagedHomeEntryIdentity{ false };
    std::atomic<bool> m_pbcXConfiguredDisabledHomeIdentity{ false };
    MotionPbcHomeStopProof m_pbcXHomeStopProof{};
    bool m_pbcXHomeStopSemanticSource = false;
    bool BuildPbcXHomeStopKey(const AxisContext& axis,
        MotionPbcHomeStopKey& key) const noexcept;
    void ObservePbcXHomeStop(const AxisContext& axis,
        const MotionServoInputSnapshot& input, bool semanticSource) noexcept;
    bool IsPbcXHomeStopped(const AxisContext& axis) const noexcept;
    bool HasPbcXUnresolvedActiveOutput() const noexcept;
    bool IsMechanicalCompensationReadyForStop(const AxisContext& axis) const noexcept;
    bool ArePhysicalGroupAxesReadyForTerminal() const noexcept;
    std::array<double, MAX_AXES> m_pbcResetNominalReferencePulse{};
    std::uint64_t m_pbcResetReferenceGeneration = 0ULL;
    double m_pbcResetRetainedOffsetPulse = 0.0;
    bool m_pbcLastSnapAccepted = true;
    std::uint64_t m_pbcLastSnapTick = 0ULL;
    MotionPbcReferenceAction m_pbcLastSnapAction = MotionPbcReferenceAction::Reset;
    bool TryPbcNominalSnap(AxisContext& axis, double& nominal,
        MotionPbcReferenceAction action) noexcept;
    void PublishPbcReference(const AxisContext& axis,
        const CompensationEngine::XReferenceTransaction& tx,
        MotionPbcReferenceResult result, pbc::Error error) noexcept;
    struct PbcHomeReservation
    {
        AlarmManager::MotionAdmissionReservation alarm{};
        std::uint64_t owner = 0ULL, epoch = 0ULL, safety = 0ULL, drain = 0ULL;
        bool ingress = false, ownerHeld = false, epochHeld = false;
    };
    bool AcquirePbcHomeReservation(const MotionOwnerLease& lease,
        PbcHomeReservation& reservation) noexcept;
    bool IsPbcHomeReservationCurrent(const PbcHomeReservation& reservation) const noexcept;
    void ReleasePbcHomeReservation(PbcHomeReservation& reservation) noexcept;
    MotionPbcXSource m_pbcXSource{};
    pbc::SendContract m_pbcXSendContract{};
    MotionPbcXSendAudit m_pbcXSendAudit{};
    void CapturePbcXCommand(const AxisContext& axis, const AxisCommand& cmd) noexcept;
    static pbc::SendIdentity PbcXSendIdentity(const ServoOutputImageProof& proof) noexcept;
    bool SealPbcXSend(ServoOutputFrameReservation& reservation, bool authorityCurrent) noexcept;
    CompensationEngine::XCycleTransaction m_pbcXCycle{};
    std::uint64_t m_pbcXCycleSendSequence = 0ULL;
    std::uint64_t m_pbcXSendTicket = 0ULL, m_pbcXSendSequence = 0ULL;
    pbc::SendIdentity m_pbcXCanonicalSendIdentity{};
    bool PreparePbcXControl(const AxisContext& axis, const AxisCommand& command,
        pbc::ControlFrame& control) noexcept;
    void ReceiptPbcXControl(const AxisContext& axis, const AxisCommand& command,
        std::int32_t outputVelocity) noexcept;
    void DiscardPbcXUnsentCycle() noexcept;
    std::uint64_t m_servoHandoffSequence = 0ULL;
    std::uint32_t m_servoSourceInputMask = 0U;
    std::array<std::uint16_t, MAX_AXES> m_servoSourceStatusWord{};
    std::array<std::int8_t, MAX_AXES> m_servoSourceMode{};
    void CaptureServoHandoffImage(ServoOutputFrameReservation& reservation) const noexcept;

    // NC writes only the atomic grant; the 250 us owner owns all hold state.
    std::atomic<std::uint64_t> m_programEndIdleHoldGrant{ 0ULL };
    struct IdlePositionHoldState
    {
        MotionOwnerLease lease{};
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        std::uint32_t requiredMask = 0U;
        std::uint32_t capturedMask = 0U;
        std::uint32_t frameMask = 0U;
        std::uint32_t reverseMask = 0U;
        std::uint32_t rotaryMask = 0U; // Captured LINEAR / ROTARY identity.
        std::uint64_t runtimeTick = 0ULL;
        bool active = false;
        bool passRequested = false;
        bool cancelled = false;
        std::array<double, MAX_AXES> reference{};
        std::array<double, MAX_AXES> unitsPerPulse{};
        std::array<double, MAX_AXES> window{};
        std::array<double, MAX_AXES> kp{};
        std::array<double, MAX_AXES> maxVelocity{};
        std::array<double, MAX_AXES> machineOffset{};
    };
    static_assert(sizeof(IdlePositionHoldState) + sizeof(std::uint64_t) <= 512U,
        "CJ idle hold storage must remain bounded.");
    IdlePositionHoldState m_idlePositionHold{};
    void PrepareIdlePositionHoldPass() noexcept;
    void UpdateIdlePositionHoldAxis(ServoOutput* output, AxisContext& axis,
        const MotionServoInputSnapshot& input) noexcept;
    bool IsIdlePositionHoldImageCurrent(std::uint64_t ownerState,
        std::uint64_t executionPublication) const noexcept;
    void CancelIdlePositionHold(IdleHoldDiagnosticReason reason, bool fault,
        int axisIndex = -1) noexcept;

    struct IdleHoldDiagnostics
    {
        FixedCapacitySpscRing<IdleHoldDiagnosticEvent,
            IDLE_HOLD_DIAGNOSTIC_CAPACITY> events{};
        // RT-owned reusable capture workspace: no record array on RT stack.
        IdleHoldDiagnosticEvent producerEvent{};
        std::uint64_t nextSequence = 0ULL;
        std::atomic<std::uint32_t> dropped{ 0U };
    };
    // Transport + RT workspace + HMI's one record and last-drop scalar.
    // Separate from (and does not relax) the original 512-byte hold budget.
    static_assert(sizeof(IdleHoldDiagnostics) + sizeof(IdleHoldDiagnosticEvent) +
        sizeof(std::uint32_t) <= 1536U,
        "BASE57 total fixed diagnostic workspace must not exceed 1.5 KiB.");
    RotaryPidDiagnosticMonitor m_rotaryPidDiagnosticMonitor{};
    RotaryPidDiagnosticSample m_rotaryPidDiagnosticProducerSample{};
    void CaptureRotaryPidDiagnostics(bool imageAuthorized) noexcept;
    // TUNE15 RT-owned live PID state; no non-RT AxisContext reads.
    void ServicePositionPidRuntime(bool completedInputPass) noexcept;
    std::uint64_t m_positionPidGeneration=1ULL, m_positionPidLastCycle=0ULL, m_positionPidPublishedCycle=0ULL;
    std::array<std::uint32_t,6U> m_positionPidSettle{};
    std::array<double,6U> m_positionPidCommand{},m_positionPidActualMin{},m_positionPidActualMax{};


    IdleHoldDiagnostics m_idleHoldDiagnostics{};
    void QueueIdleHoldDiagnostic(IdleHoldDiagnosticEventType eventType,
        IdleHoldDiagnosticReason reason = IdleHoldDiagnosticReason::NONE,
        int axisIndex = -1,
        MotionOwnerLease nextLease = MotionOwnerLease{},
        const std::array<double, 3>* detail = nullptr,
        std::uint32_t detailFlags = 0U) noexcept;

    struct CncP1LateJunction
    {
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        MotionCommand successor{};
        bool loaded = false;
        bool pending = false;
        FixedCapacitySpscRing<CncP1Diagnostic, CNC_P1_DIAGNOSTIC_CAPACITY> events{};
        CncP1Diagnostic producerEvent{};
        std::uint64_t sequence = 0ULL;
        std::atomic<std::uint32_t> dropped{ 0U };
    };
    static_assert(sizeof(CncP1LateJunction) + sizeof(CncP1Diagnostic) +
        sizeof(std::uint32_t) <= 8192U,
        "DC total diagnostic/refresh workspace must not exceed 8 KiB.");
    CncP1LateJunction m_cncP1Late{};
    struct CncFeedLookahead
    {
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        MotionCommand scratch{};
        std::array<double, 4U> lengths{}, speeds{}, nominalSpeeds{}, accelerations{}, decelerations{};
        std::array<double, 5U> boundaries{};
        CncFeedPlanDiagnostic plan{}, event{};
        double entryCarry = 0.0;
        MotionSegmentId handoffFrom = MOTION_SEGMENT_ID_INVALID;
        FixedCapacitySpscRing<CncFeedPlanDiagnostic, CNC_FEED_PLAN_CAPACITY> events{};
        std::atomic<std::uint32_t> dropped{ 0U };
        std::uint64_t sequence = 0ULL;
        std::size_t observedDepth = (std::numeric_limits<std::size_t>::max)();
        std::size_t zeroInputCycles = 0U;
        bool loaded = false, blendEntered = false;
        // RT-local DJ prefix schedule. It cannot relax the arc or source seam.
        double prefixLength = 0.0, prefixLimit = 0.0, prefixReserve = 0.0;
        double prefixPreviousRaw = 0.0;
        bool prefixEnabled = false, prefixFastSeen = false, prefixBrakeSeen = false;
        bool prefixAuthoredSeen = false;
    };
    static_assert(sizeof(CncFeedLookahead) <= 12288U, "DG RT-owned storage budget.");
    CncFeedLookahead m_cncFeedLookahead{};
    static bool BuildCncFeedStopPlan(const std::array<double, 4U>& length,
        const std::array<double, 4U>& speed, const std::array<double, 4U>& acceleration,
        const std::array<double, 4U>& deceleration, std::size_t count, double entry,
        std::array<double, 5U>& boundary) noexcept;
    void RefreshCncFeedLookahead(bool loading = false) noexcept;
    void FinishCncFeedLookahead() noexcept;
    void QueueCncFeedPlanDiagnostic(CncFeedPlanEvent event) noexcept;


    bool IsCncP1LateJunctionScope() const noexcept;
    void ArmCncP1LateJunction(bool nextVisible, const MotionCommand& next) noexcept;
    void RefreshCncP1LateJunction() noexcept;
    void FinishCncP1Diagnostic() noexcept;
    void QueueCncP1Diagnostic(CncP1Event event, CncP1Reason reason,
        const MotionCommand* next = nullptr, double nextLength = 0.0) noexcept;

    void InvalidateServoOutputImageProof() noexcept;
    bool ZeroAllServoTargetVelocityForFrame() noexcept;
    bool PublishServoOutputImageProof(
        std::uint64_t ownerState,
        std::uint64_t executionPublication,
        std::uint64_t frameSafetyIntentState,
        std::uint64_t alarmSafetyIntentState,
        std::uint32_t alarmUpdateCount,
        ServoOutputImageProofMode mode) noexcept;

    void WriteServoTouchProbeFunctionCommand(
        ServoOutput* output,
        int axisIndex,
        uint16_t value);

    void WriteServoModesOfOperationCommand(
        ServoOutput* output,
        int axisIndex,
        int8_t value);


    // =========================================================
    // Stage 11D.6 - centralized active LEGACY input seam.
    // =========================================================

    bool ReadLegacyMotionServoInputSnapshot(
        const ENI_ServoDrive& servo,
        MotionServoInputSnapshot& snapshot) const;

    bool ReadLegacyMotionServoInputSnapshotBySlot(
        int motionSlot,
        MotionServoInputSnapshot& snapshot) const;


    // =========================================================
    // Stage 11D.8 - controlled compatibility input seam.
    //
    // motionSlot is converted to AxisContext.axisIndex.
    //
    // After D7 full qualification:
    //     published Motion semantic snapshot
    //
    // Before D7 qualification / on D8 read failure:
    //     legacy compatibility fallback
    // =========================================================

    bool ReadMotionServoInputCompatibilitySnapshotBySlot(
        int motionSlot,
        MotionServoInputSnapshot& snapshot) const;


    // 指向實體資料的指標列表--------------------------------------------------------------------
    std::vector<ENI_ServoDrive>* m_pAxes = nullptr;   // 舊有的驅動器關聯 (保留相容性)
    std::vector<ENI_ServoDrive>* m_pDrives = nullptr;  // 實體驅動器列表 (PDO 對接)
    std::vector<AxisContext>* m_pContexts = nullptr;   // 軸參數與狀態列表 (邏輯計算)


    // Stage 11D.3 shadow-only bridge back to the owning Master.
    // Never used to command a Servo in this stage.
    EtherCatMaster* m_pStructuredServoReadShadowMaster =
        nullptr;


    // 多軸插補管理器 (單一實體群組)--------------------------------------------------------------------
    InterpolationGroup m_Group;

    // FIX17: Runtime-only arithmetic state; never changes command/axis ABI.
    // Identity and observed anchors prevent compensation crossing a child or
    // an external scalar rebase. No value here grants motion authority.
    struct RetainedScalarSummation
    {
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
        double planCorrection=0.0, outputCorrection=0.0, filterCorrection=0.0;
        double expectedPlanning=0.0, expectedPosition=0.0, expectedSum=0.0, expectedTarget=0.0;
        std::size_t filterSize=0U, zeroInputCycles=0U;
        int filterIndex=0;
        bool active=false;
    };
    RetainedScalarSummation m_retainedScalarSummation{};



public:

    //G碼使用-------------------------------------------------------------- 
    void G00_Move(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode = BufferMode::ABORTING);// G00 快速定位 API
    void G00_Move(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode,
        MotionCommandPathMode commandPathMode);// K.6 explicit transport contract
    bool TryG00MoveTransactionalTail(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode,
        MotionCommandPathMode commandPathMode,
        double rapidOverrideCandidate,
        double(&commandedMCSTail)[MAX_AXES]);
    // BQ: optional data export; the compatibility overload above is unchanged.
    bool TryG00MoveTransactionalTail(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode,
        MotionCommandPathMode commandPathMode,
        double rapidOverrideCandidate,
        double(&commandedMCSTail)[MAX_AXES],
        MotionCommandedEndpointReceiptV1* commandedEndpointReceipt,
        bool requirePlanarBaselineMatch = false);
    // BX: G01 uses mm/min along the physical XYZ chord, exact-stop only.
    bool TryG01MoveTransactionalTail(
        const std::vector<int>& axes,
        const std::vector<double>& targetMCS,
        double feedMMMin,
        double(&commandedMCSTail)[MAX_AXES],
        MotionFeedLineWorkspace& workspace);
    // BASE79K: bounded C-only or Z+C G90/G91 with generated XY; F is raw NC F.
    bool TryG01EccentricCMoveTransactionalTail(double programmedC, double programmedFeed,
        double(&commandedMCSTail)[MAX_AXES], MotionEccentricCProducerWorkspace& workspace,
        MotionFeedLineReceipt& result, double programmedZ = 0.0, bool hasProgrammedZ = false);
    // BASE79L: sparse authored G91 XYZ+C; F measures the authored XYZ line.
    // XY-absent masks retain the existing G90/G91 C-only and Z+C contracts.
    bool TryG01EccentricXYZCMoveTransactionalTail(double programmedC, double programmedFeed,
        double(&commandedMCSTail)[MAX_AXES], MotionEccentricCProducerWorkspace& workspace,
        MotionFeedLineReceipt& result, const std::array<double, 3>& programmedXYZ,
        std::uint32_t linearMask);
    // BASE69: G90 absolute or G91 signed degrees; resolve the native path once.
    bool TryG01RotaryMoveTransactionalTail(int axisIndex, double programmedValue,
        double targetMCS, double feedDegMin, double(&commandedMCSTail)[MAX_AXES],
        MotionFeedLineWorkspace& workspace);
    // BASE71: explicit G90/G91 Z/C, F measures Z millimetres/minute; one common clock.
    bool TryG01ZCMoveTransactionalTail(double programmedZ, double programmedC,
        double targetZMCS, double targetCMCS, double feedMMMin,
        double(&commandedMCSTail)[MAX_AXES], MotionFeedLineWorkspace& workspace);
    // BASE72: G91 XYZ path mm/min plus C degrees under one common time.
    // BASE75 first array: raw WCS in G90, signed native increments in G91.
    bool TryG01XYZCMoveTransactionalTail(const std::array<double, 4U>& programmedValues,
        const std::array<double, 4U>& targetMCS, double feedMMMin,
        double(&commandedMCSTail)[MAX_AXES], MotionFeedLineWorkspace& workspace);
    // BASE74: G90 absolute targets or G91 XYZ path increments plus C/U/V degrees.
    // First array is raw programmed WCS in G90; second is requested MCS.
    bool TryG01XYZCUVMoveTransactionalTail(const std::array<double, 6U>& programmedDelta,
        const std::array<double, 6U>& targetMCS, double feedMMMin,
        double(&commandedMCSTail)[MAX_AXES], MotionFeedLineWorkspace& workspace,
        bool sparseCommissioning = false);
    // DD: a non-null predecessor selects BUFFERED/EXACT_STOP, same tuple/mapping.
    bool TryG01MoveTransactionalTail(
        const std::vector<int>& axes,
        const std::vector<double>& targetMCS,
        double feedMMMin,
        double(&commandedMCSTail)[MAX_AXES],
        MotionFeedLineWorkspace& workspace,
        const MotionFeedLineReceipt* predecessor);
    // DE only: the selected DD route explicitly opts into the bounded RT planner.
    bool TryG01MoveTransactionalTail(
        const std::vector<int>& axes, const std::vector<double>& targetMCS,
        double feedMMMin, double(&commandedMCSTail)[MAX_AXES],
        MotionFeedLineWorkspace& workspace, const MotionFeedLineReceipt* predecessor,
        bool cncFeedLookahead);
    // DG: mixed LINE/ARC producer chain; the anchor has no synthetic line geometry.
    bool TryG01MoveTransactionalCncTail(
        const std::vector<int>& axes, const std::vector<double>& targetMCS,
        double feedMMMin, double(&commandedMCSTail)[MAX_AXES],
        MotionFeedLineWorkspace& workspace, const MotionCncPathTail* predecessor,
        bool cncFeedLookahead,
        const std::array<double, 8U>* cornerNextMCS = nullptr, double cornerToleranceMM = 0.0,
        // DI: zero preserves the existing same-F caller; NC passes explicit next-row F.
        const MotionArcTravelGuard* cornerTravelGuard = nullptr, double cornerNextFeedMMMin = 0.0,
        // EG: 0 preserves explicit legacy axes; 1/2 retain the omitted planar endpoint.
        std::uint32_t endpointAxisMask = 0U,
        bool requirePlanarBaselineMatch = false,
        // BASE-PLANE-21: G17 polar cutter G40 lead-out must also prove
        // its stationary native Z basis. Active cutter enforces this itself.
        bool requireNativeXYZBaselineMatch = false);
    bool IsCncPathProducerTailCurrent(const MotionCncPathTail& tail,
        std::uint32_t mask, const double* commandedTail, MotionExecutionEpoch epoch,
        const MotionOwnerLease& owner, MotionCommandSource source) const noexcept;
    // BY: synchronous borrowed travel guard, caller-owned command scratch.
    bool TryG02G03MoveTransactionalTail(
        const std::array<double, 8U>& targetMCS,
        const std::array<double, 2U>& centerOffsetMM,
        int direction, bool fullCircle, double feedMMMin,
        const MotionArcTravelGuard& travelGuard,
        double(&commandedMCSTail)[MAX_AXES],
        MotionFeedArcWorkspace& workspace,
        MotionCommand& commandWorkspace);

    bool TryG02G03MoveTransactionalCncTail(
        const std::array<double, 8U>& targetMCS,
        const std::array<double, 2U>& centerOffsetMM,
        int direction, bool fullCircle, double feedMMMin,
        const MotionArcTravelGuard& travelGuard,
        double(&commandedMCSTail)[MAX_AXES], MotionFeedArcWorkspace& workspace,
        MotionCommand& commandWorkspace, const MotionCncPathTail* predecessor,
        bool cncFeedLookahead,
        // EF: endpoint-word presence, distinct from the physical XY arc mask.
        std::uint32_t endpointAxisMask = 3U,
        bool requirePlanarBaselineMatch = false,
        // NC contour geometry only: fully resolved partial canonical-plane cutter circle.
        bool preparedCutterArc = false);

    // BZ: evaluate the original canonical line/circle in either direction.
    bool TryPathCoreRetainedMoveTransactionalTail(
        const NCPathCoreRetainedGeometry& geometry, bool reverse, double feedMMMin,
        const MotionArcTravelGuard& travelGuard,
        double(&commandedMCSTail)[MAX_AXES],
        MotionPathCoreRetainedWorkspace& workspace,
        MotionCommand& commandWorkspace);

    // CA: traverse a finite sub-interval of the unchanged canonical source.
    bool TryPathCoreRetainedIntervalMoveTransactionalTail(
        const NCPathCoreRetainedGeometry& geometry, double startU, double endU, double feedMMMin,
        const MotionArcTravelGuard& travelGuard,
        double(&commandedMCSTail)[MAX_AXES],
        MotionPathCoreRetainedWorkspace& workspace,
        MotionCommand& commandWorkspace);

    // BASE41: profile 7/161 uses WCS-resolved native endpoints; 28/30/32 uses
    // direct reference MCS. Single exact-stop command; success-only tail commit.
    bool TryPositioningMoveTransactionalTail(const std::vector<int>& axes,
        const std::vector<double>& nativeTargets, int profileCode, double* commandedMCSTail);
    // BASE42: two native reference legs share one epoch and one atomic ingress
    // publication. Intermediate mask bits are native axis indices; other axes
    // retain their exact sampled pulse until the final reference leg.
    bool TryReferencePositionPairTransactionalTail(const std::vector<int>& axes,
        const std::vector<double>& intermediateNativeTargets,
        const std::vector<double>& finalNativeTargets, std::uint32_t intermediateMask,
        int profileCode, double* commandedMCSTail);
    bool TryG76GapMoveTransactionalTail(const EDM173::Plan& plan,double* commandedMCSTail,MotionG76OrbitReceipt& receipt,
        const NCG76Mixed::Point8* stationaryRotaryModulo=nullptr);
    bool TryG76MoveTransactionalTail(const NCG76::NativePlan& plan,
        double* commandedMCSTail, MotionG76OrbitReceipt& receipt);
    bool TryG76MixedMoveTransactionalTail(const NCG76Mixed::Plan& plan,
        double* commandedMCSTail, MotionG76OrbitReceipt& receipt);
    void G07_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G07 快速定位 API
    void G161_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G161 快速定位 API
    // Native mm/degrees, absolute exact-stop. Commits MCS/pulse tails only
    // after the same epoch/owner/enqueue proof used by transactional G00.
    bool TryG53MoveTransactionalTail(const std::vector<int>& axes,
        const std::vector<double>& nativeTargets, double* commandedMCSTail);
    void G53_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G53 機械定位 API
    void G28_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G28 參考點賦歸 API
    void G30_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G30 參考點賦歸 API
    void G32_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G32 參考點賦歸 API

};
