#pragma once
#include <vector>
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
#include "CompensationEngine.h" // 引入剛寫好的標頭檔
#include "SHM_Types.h"
#include "MotionExecutionContract.h"
#include "MotionCommandPathModeTransport.h"
#include "MotionQueueTailTransaction.h"
#include "MotionCommandRing.h"
#include "MotionFeedbackRing.h"
#include "MotionAxisCommandMailbox.h"
#include "AlarmManager.h"

class EtherCatMaster;
constexpr int MAX_AXES = 8;//最大軸數宣告
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

    // 運動參數
    double targetVel = 0.0;
    double accTime = 0.0;
    double decTime = 0.0;

    // 時光機專用快照記憶體
    // True only for an RT replay transport copy whose lifecycle identity was
    // already terminal before B2 pushed it back in front of the path.
    bool replayTerminalAlreadyPublished = false;
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
};

static_assert(
    std::is_trivially_copyable<MotionCommand>::value,
    "MotionCommand must remain trivially copyable for the fixed SPSC ring.");
static_assert(
    std::is_standard_layout<MotionCommand>::value,
    "MotionCommand layout must remain inspectable across the fixed transport.");

#if defined(_WIN64) || defined(__x86_64__) || defined(__aarch64__)
static_assert(
    sizeof(MotionCommand) == 560U && alignof(MotionCommand) == 8U,
    "K.6 must consume existing x64 padding without changing MotionCommand ABI size.");
static_assert(
    offsetof(MotionCommand, commandPathMode) == 555U &&
    offsetof(MotionCommand, sourcePlaneMode) == 556U,
    "K.6 commandPathMode must occupy the accepted x64 tail-padding byte.");
#endif

// Stage NC-0.2D：NC Producer 在單一 Program Block 派送期間，
// 固定容量收集該 Block 建立的 Segment Identity。它只存在 Producer
// 執行緒，不進入 250 us Runtime，也不改變 Command / Feedback ABI。
constexpr std::size_t MOTION_PROGRAM_BLOCK_CAPTURE_CAPACITY = 32U;

struct MotionProgramBlockSubmission
{
    MotionExecutionIdentity identity{};
    MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED;
    MotionQueueTailCommitReceipt queueTailReceipt{};
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
    std::int32_t worstCommandVelocityAxisIndex = -1;
    double worstFollowingErrorAbsPulse = 0.0;
    double worstFollowingWindowPulse = 0.0;
    std::int32_t worstFollowingErrorAxisIndex = -1;
    double worstActualExcursionPulse = 0.0;
    double worstActualExcursionLimitPulse = 0.0;
    std::int32_t worstActualExcursionAxisIndex = -1;

    // Advisory only.  Neither value can block or revoke settled=true.
    double advisoryMaxActualVelocityAbsPps = 0.0;
    std::uint32_t advisoryMaxPdoTargetVelocityAbs = 0U;
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
class MotionCore
{
public:
    MotionCore();




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
    };

    bool BeginServoOutputFrameAtSendPoint(
        ServoOutputFrameReservation& reservation) noexcept;
    bool FinalizeServoOutputFrameAtSendPoint(
        ServoOutputFrameReservation& reservation) noexcept;
    void EndServoOutputFrameAfterSend(
        ServoOutputFrameReservation& reservation) noexcept;

    // Stage 11D.6:
    // Input arrives through MotionServoInputSnapshot.
    // Output remains the original ENI_ServoDrive.
    void UpdateServoState(
        ENI_ServoDrive& servo,
        AxisContext& axis,
        const MotionServoInputSnapshot& input);

    void ExportDebugInfo(SHM_AxisDebugInfo* outDebugArray, bool outputInMM = false);
    //單軸運動 API--------------------------------------------------------------------


    void InitAxis(AxisContext& axis, double resolution = 16777216.0);// 初始化軸參數 (如解析度、預設極限、PID)
    void InitSmoothBuffer(AxisContext& axis, double smoothTime_ms);// 初始化 S-Curve 平滑濾波緩衝區
    void MoveToPosition(AxisContext& axis, double targetPos, double targetVel, double acc_time, double dec_time);// 下達 P2P 絕對位置移動指令 (Trapezoidal 梯形加減速)
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
        const MotionOwnerLease& ownerLease);

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
    void LoadNextCommand();// 從指令佇列 (Queue) 載入下一段任務  
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
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

    bool SubmitAxisVelocityMove(
        int axisIndex,
        double targetVelocity,
        double accelerationTime,
        MotionCommandSource source,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

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
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

    bool SubmitApplyMachineHome(
        int axisIndex,
        double capturedReferencePulse,
        double homeOffsetUnit,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence& outSequence) noexcept;

    bool SubmitDriveTouchProbeFunction(
        int axisIndex,
        std::uint16_t value,
        const MotionOwnerLease& ownerLease,
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

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
                outTarget_mm[i] = axis.currentActPos / pulsePerUnit;
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

    double CalculateShortestTarget(double currentPos, double targetPos, double modulo);

    // 🌟 消滅幽靈座標專用 API：將大腦預讀起點，強制同步為馬達當下真實位置
    void SyncVirtualEndPosition();
    bool TryGetSynchronizedG00QueueTailMCS(
        double(&outputMCS)[MAX_AXES]) const noexcept;

    //G碼參數專區------------------------------------------------------------
    double G00_overrideRatio = 1;//G00 專屬速度比例




private:
    // ========================================================================
    // Stage NC-0.2J.4 - Atomic Stop / Settle Evidence Publication
    //
    // A complete POD payload is copied into one of two banks of 64-bit atomic
    // words.  The producer publishes the selected bank with one Generation.
    // A reader retries if Generation changes while copying.  Therefore the
    // reader never races a plain AxisContext read and no mutex or atomic large
    // structure is required.
    // ========================================================================
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
    void ApplyNCResetScalarRebase() noexcept;
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
        MotionAxisCommandSequence* outSequence) noexcept;
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
        const MotionOwnerLease* plannedTailOwnerLease) noexcept;
    bool TryG00MoveInternal(
        const std::vector<int>& axes,
        const std::vector<double>& targetPos,
        BufferMode mode,
        MotionCommandPathMode commandPathMode,
        double rapidOverrideCandidate,
        double* commandedMCSTail,
        bool transactionalTail);
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

    std::atomic<std::uint64_t> m_motionFeedbackOverflowCount{ 0ULL };
    std::atomic<std::uint64_t> m_motionFeedbackProducerNoticeOverflowCount{ 0ULL };
    std::atomic<MotionFeedbackSequence> m_lastPublishedMotionFeedbackSequence{ MOTION_FEEDBACK_SEQUENCE_INVALID };
    std::atomic<MotionSegmentId> m_lastDroppedMotionFeedbackSegmentId{ MOTION_SEGMENT_ID_INVALID };
    std::atomic<MotionFeedbackType> m_lastDroppedMotionFeedbackType{ MotionFeedbackType::NONE };

    MotionSegmentId AllocateMotionSegmentId() noexcept;
    void AssignExecutionIdentity(
        MotionCommand& command,
        MotionExecutionEpoch exactEpoch,
        MotionCommandSource exactSource,
        const MotionOwnerLease& exactOwnerLease) noexcept;
    bool IsCommandFromCurrentEpoch(const MotionCommand& command) const noexcept;
    bool IsCommandOwnerLeaseCurrent(const MotionCommand& command) const noexcept;
    MotionRejectReason GetCommandAuthorizationFailure(
        const MotionCommand& command) const noexcept;

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
    bool HasPendingSafetyIntent() const noexcept;
    void TryAcknowledgeAppliedSafetyMotionRequests() noexcept;
    bool IsSafetyControlledStopAuthorized(
        int contextAxisSlot) const noexcept;
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
    bool TryPeekNextMotionCommand(MotionCommand& command) const noexcept;
    bool TryPeekQueuedMotionCommandAt(
        std::size_t offset,
        MotionCommand& command) const noexcept;
    bool TryDequeueNextMotionCommand(MotionCommand& command) noexcept;
    bool TryRequeueMotionCommandFront(const MotionCommand& command) noexcept;

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

    void Run_Servo_Loop(DriveType& servo, AxisContext& axis, const AxisCommand& cmd);
    void DetermineActiveGainSet(AxisContext& axis); // 👈 必須要有這行宣告

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
        int32_t value);

    enum class ServoOutputImageProofMode : std::uint8_t
    {
        INVALID = 0,
        ZERO_ONLY,
        NORMAL,
        CONTROLLED_STOP
    };

    struct ServoOutputImageProof
    {
        std::uint64_t ownerState = 0ULL;
        std::uint64_t executionPublication = 0ULL;
        std::uint64_t frameSafetyIntentState = 0ULL;
        std::uint64_t alarmSafetyIntentState = 0ULL;
        std::uint64_t generation = 0ULL;
        std::uint32_t alarmUpdateCount = 0U;
        std::uint32_t slotMask = 0U;
        ServoOutputImageProofMode mode =
            ServoOutputImageProofMode::INVALID;
        std::array<std::int32_t, MAX_AXES> targetVelocity{};
    };

    // These fields are single-writer/single-reader on the same Priority-64
    // callback: UpdateAllMotion builds the next image only after the current
    // frame round trip, and EtherCAT captures it at the following send point.
    ServoOutputImageProof m_servoOutputImageProof{};
    std::uint64_t m_servoOutputImageProofGeneration = 0ULL;

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
    void G07_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G07 快速定位 API
    void G161_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G161 快速定位 API
    void G53_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode = BufferMode::ABORTING);// G53 機械定位 API
    void G28_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G28 參考點賦歸 API
    void G30_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G30 參考點賦歸 API
    void G32_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode);// G32 參考點賦歸 API

};
