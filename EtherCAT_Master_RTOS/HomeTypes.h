#pragma once

#include <cstdint>

// ============================================================
// HOME Method
//
// 決定每一軸使用哪一種方式建立 Machine Home。
// ============================================================

enum class HomeMethod : int
{
    DOG_INDEX = 0,          // DOG -> Backoff -> INDEX
    LIMIT_INDEX = 1,        // Hard Limit -> Backoff -> INDEX
    DOG_ONLY = 2,           // DOG -> Backoff -> 直接建立 Home
    LIMIT_ONLY = 3,         // Hard Limit -> Backoff -> 直接建立 Home
    INDEX_ONLY = 4,         // 不找 DOG / Limit，直接尋找 INDEX
    ABSOLUTE_REFERENCE = 5,           // Absolute Encoder / Absolute Linear Scale
    CURRENT_POSITION = 6,   // 目前位置直接設為 Home
    MECHANICAL_STOP = 7     // 預留：Torque / Following Error 尋找機械端點
};


// ============================================================
// HOME Reference Source
//
// 決定 INDEX / Absolute Reference 從哪裡取得。
// ============================================================

enum class HomeReferenceSource : int
{
    NONE = 0,

    MOTOR_ENCODER_INDEX = 1,       // 馬達 Encoder INDEX
    LINEAR_SCALE_INDEX_DRIVE = 2,  // 光學尺 INDEX 由 Servo Drive 擷取
    EXTERNAL_IO_INDEX = 3,         // 外部 IO Board / PLC C Point

    ABSOLUTE_MOTOR_ENCODER = 4,    // 絕對式馬達 Encoder
    ABSOLUTE_LINEAR_SCALE = 5      // 絕對式光學尺
};


// ============================================================
// HOME Reference Capture Mode
//
// Reference Source：訊號從哪裡來。
// Capture Mode：位置如何被鎖存。
// ============================================================

enum class HomeReferenceCaptureMode : int
{
    DRIVE_HARDWARE_LATCH = 0,      // Drive Hardware Capture，例如 0x60B9 / 0x60BA
    EXTERNAL_HARDWARE_LATCH = 1,   // 外部 Counter / IO Board Hardware Capture
    SOFTWARE_SAMPLE = 2            // C Point Rising Edge 時讀取 currentActPos
};


// ============================================================
// Drive Touch Probe Arm Mode
//
// CONTROLLER_60B8：
//     HomingManager 負責寫入 0x60B8，執行 Disarm -> Clear -> Arm。
//
// DRIVE_AUTO_ARM：
//     驅動器參數已自行處理 Arm；上控不寫 0x60B8，
//     只用 0x60B9 / 0x60BA 判斷本輪新的 Capture。
// ============================================================

enum class HomeDriveProbeArmMode : int
{
    CONTROLLER_60B8 = 0,
    DRIVE_AUTO_ARM = 1
};


// ============================================================
// Drive Touch Probe Runtime Phase
// ============================================================

enum class HomeDriveProbePhase : int
{
    IDLE = 0,
    WRITE_DISARM,
    WAIT_CLEAR,
    WRITE_ARM,
    WAIT_ARMED,
    READY,
    CAPTURED,
    DISARM_AFTER_CAPTURE,

    // 不使用 ERROR，避免與 Windows ERROR 巨集衝突。
    PROBE_ERROR
};


// ============================================================
// G81 Multi-Axis Sequence
//
// P0 = SIMULTANEOUS
// P1 = BY_ORDER
// ============================================================

enum class HomeSequenceMode : int
{
    SIMULTANEOUS = 0,
    BY_ORDER = 1
};


// ============================================================
// HOME Run Control State
//
// HomeState：每一軸尋原點流程走到哪裡。
// HomeRunControlState：整個 G81 Request 是執行、減速暫停或已暫停。
// ============================================================

enum class HomeRunControlState : int
{
    IDLE = 0,
    RUNNING,
    HOLD_DECEL_STOP,
    PAUSED
};


// ============================================================
// HOME Backoff Mode
// ============================================================

enum class HomeBackoffMode : int
{
    FIXED_DISTANCE = 0,

    // 反向移動直到 DOG OFF，
    // 再額外移動 backoffExtraDistance_unit。
    UNTIL_DOG_OFF_PLUS_DISTANCE = 1
};


// ============================================================
// Per-Axis HOME State
//
// MotionState 繼續表示「軸現在怎麼動」：
//     VELOCITY / MOVING / STOPPING / IDLE
//
// HomeState 表示「尋原點流程進行到哪裡」。
// ============================================================

enum class HomeState : int
{
    IDLE = 0,

    PREPARE,

    SEARCH_SWITCH,

    // DOG / Expected Limit 找到後，使用 Controlled Stop。
    SWITCH_DECEL_STOP,

    BACK_OFF,

    // Backoff 完成後確認 DOG / Hard Limit 是否已解除。
    VALIDATE_RELEASE,

    // 多軸模式：所有參與軸 Backoff 完成後才一起尋找 INDEX。
    WAIT_GROUP_BACKOFF,

    // 清除舊狀態並 Arm 下一次 Reference Edge。
    ARM_REFERENCE,

    SEARCH_INDEX,

    // INDEX 已被鎖存，但軸仍在移動。
    INDEX_CAPTURED,

    // INDEX 找到後滑行減速，不能使用 Emergency Stop。
    INDEX_DECEL_STOP,

    // 多軸模式：等待所有參與軸都完成 INDEX 減速停止。
    WAIT_GROUP_INDEX_STOP,

    APPLY_HOME,

    MOVE_TO_ZERO,

    DONE,
    HOME_ERROR
};


// ============================================================
// HOME Error Reason
//
// 這裡是內部失敗原因。
// 後續由 HomingManager 對應 AlarmManager 的 HOME Alarm。
// ============================================================

enum class HomeErrorReason : int
{
    NONE = 0,

    INVALID_CONFIG,
    AXIS_NOT_EXIST,
    SERVO_NOT_READY,
    MOTION_BUSY,

    SEARCH_TIMEOUT,
    SEARCH_MAX_DISTANCE,
    DOG_NOT_FOUND,

    SWITCH_STOP_MAX_DISTANCE,

    BACKOFF_TIMEOUT,
    BACKOFF_MAX_DISTANCE,

    DOG_NOT_RELEASED,
    HARD_LIMIT_NOT_RELEASED,

    INDEX_TIMEOUT,
    INDEX_MAX_DISTANCE,
    INDEX_NOT_FOUND,

    REFERENCE_NOT_ARMED,
    REFERENCE_INVALID,

    OPPOSITE_HARD_LIMIT,
    BOTH_HARD_LIMITS,

    SERVO_FAULT,
    MOTION_FAULT,

    CANCELLED
};


// ============================================================
// HOME Dedicated Gain
//
// 第一版分成兩組：
//
// searchGain：
//     SEARCH_SWITCH
//     SWITCH_DECEL_STOP
//     BACK_OFF
//
// indexGain：
//     SEARCH_INDEX
//     INDEX_DECEL_STOP
//
// MOVE_TO_ZERO 後續直接使用 G00 Gain。
// ============================================================

struct HomeGainConfig
{
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double Kvff = 0.0;
};


// ============================================================
// Per-Axis HOME Configuration
//
// 這些是參數，不是本次執行狀態。
// ============================================================

struct HomeConfig
{
    // --------------------------------------------------------
    // Basic
    // --------------------------------------------------------

    // 未設定完成前預設不允許 G81 ALL HOME 自動選入此軸。
    bool enabled = false;

    HomeMethod method =
        HomeMethod::DOG_INDEX;

    HomeReferenceSource referenceSource =
        HomeReferenceSource::MOTOR_ENCODER_INDEX;

    HomeReferenceCaptureMode captureMode =
        HomeReferenceCaptureMode::DRIVE_HARDWARE_LATCH;

    // -1 = 往 Machine Negative Direction 尋找
    // +1 = 往 Machine Positive Direction 尋找
    int direction = -1;

    // G81 P1 時使用。
    // 數字小的先執行；相同 Order 的軸同時執行。
    int order = 0;


    // --------------------------------------------------------
    // HOME Input
    // --------------------------------------------------------

    // false：C Point OFF 視為有效。
    // true ：C Point ON 視為有效。
    bool dogActiveHigh = true;

    bool referenceActiveHigh = true;

    // EXTERNAL_IO_INDEX 使用。
    //
    // -1：
    //     使用 NCPLC::C::HOME_INDEX_BASE + axisIndex
    //
    // >= 0：
    //     使用指定的 PLC C Point。
    int externalReferenceCPoint = -1;


    // --------------------------------------------------------
    // Search DOG / Limit
    //
    // Speed 內部使用 Pulse/sec。
    // Distance 使用 Machine Unit：
    //     Linear = mm
    //     Rotary = degree
    // --------------------------------------------------------

    double searchSpeed_PPS = 0.0;

    double searchAccTime = 0.2;
    double searchDecTime = 0.2;

    // 0 = 不啟用此保護。
    double searchMaxDistance_unit = 0.0;

    // 0 = 不啟用此保護。
    double searchTimeoutSec = 0.0;


    // --------------------------------------------------------
    // DOG / Expected Limit Detected -> Controlled Stop
    // --------------------------------------------------------

    double switchStopDecTime = 0.2;

    // 從 Sensor Detected 到完全停止的最大允許滑行距離。
    // 0 = 不啟用此保護。
    double switchStopMaxDistance_unit = 0.0;


    // --------------------------------------------------------
    // Backoff
    // --------------------------------------------------------

    HomeBackoffMode backoffMode =
        HomeBackoffMode::UNTIL_DOG_OFF_PLUS_DISTANCE;

    // FIXED_DISTANCE 使用。
    double backoffDistance_unit = 0.0;

    // UNTIL_DOG_OFF_PLUS_DISTANCE 使用。
    double backoffExtraDistance_unit = 0.0;

    double backoffSpeed_PPS = 0.0;

    double backoffAccTime = 0.2;
    double backoffDecTime = 0.2;

    // 避免 DOG / Limit 永遠不解除而一直反向移動。
    // 0 = 不啟用此保護。
    double backoffMaxDistance_unit = 0.0;

    // 0 = 不啟用此保護。
    double backoffTimeoutSec = 0.0;


    // --------------------------------------------------------
    // Release Validation
    //
    // Backoff 完成後若訊號仍然存在：
    //
    // true  = 建立 HOME Alarm，不允許進 SEARCH_INDEX
    // false = 允許繼續
    //
    // 預設都必須 Alarm。
    // --------------------------------------------------------

    bool alarmIfDogNotReleasedBeforeIndex = true;

    bool alarmIfHardLimitNotReleasedBeforeIndex = true;


    // --------------------------------------------------------
    // Drive Touch Probe / Motor Encoder INDEX
    //
    // 只在：
    //     referenceSource = MOTOR_ENCODER_INDEX
    //     captureMode = DRIVE_HARDWARE_LATCH
    //
    // 或未來 LINEAR_SCALE_INDEX_DRIVE 使用相同 Drive Provider 時生效。
    // --------------------------------------------------------

    HomeDriveProbeArmMode driveProbeArmMode =
        HomeDriveProbeArmMode::CONTROLLER_60B8;

    // 0x60B8 Disarm / Arm 原始值。
    // 預設 0x0015 = Probe1 Enable + Motor Z Source + Positive Edge。
    // 若實際 Delta Firmware / Parameter 定義不同，可直接改參數檔，
    // 不必重新編譯 HomingManager。
    uint16_t driveProbeDisarmValue = 0x0000;
    uint16_t driveProbeArmValue = 0x0015;

    // 0x60B9 Status Mask。
    // armedMask：Probe 1 Enable Status。
    // capturedMask：Probe 1 Positive Edge Capture Complete。
    uint16_t driveProbeArmedMask = 0x0001;
    uint16_t driveProbeCapturedMask = 0x0002;

    // 某些 Multiple Capture 模式會用 Toggle Bit 辨識新事件。
    // 0 = 不使用 Toggle 判斷。
    uint16_t driveProbeCaptureToggleMask = 0x0000;

    // 選配：驗證 0x60B9 的 Capture Source Status。
    // sourceMask = 0 時不驗證。
    uint16_t driveProbeSourceMask = 0x0000;
    uint16_t driveProbeExpectedSourceValue = 0x0000;

    // Disarm 後等待舊 Capture Status 清除的最大時間。
    double driveProbeClearTimeoutSec = 2.0;

    // Arm 後等待 0x60B9 Armed Status 的最大時間。
    double driveProbeArmTimeoutSec = 2.0;

    // true：CONTROLLER_60B8 模式必須看到 Armed Mask 才開始移動。
    bool driveProbeRequireArmedStatus = true;

    // true：Capture 後立即把 0x60B8 寫回 Disarm Value。
    bool driveProbeDisarmAfterCapture = true;

    // true：必須辨識本輪的新 Capture；不可只因 60BA 非 0 就完成。
    bool driveProbeRequireNewCapture = true;

    // 安全預設 false。
    // 若驅動器 Auto Arm 模式的 Capture Complete Bit 為 Sticky，
    // 且沒有 Toggle Bit，可在確認硬體行為後允許用 60BA 變化判斷。
    bool driveProbeAllowPositionChangeDetection = false;


    // --------------------------------------------------------
    // INDEX Search Motion
    // --------------------------------------------------------

    double indexSearchSpeed_PPS = 0.0;

    double indexSearchAccTime = 0.2;

    // INDEX 找到後的 Controlled Stop 減速時間。
    // 不可使用 Emergency Stop，避免機台震動。
    double indexStopDecTime = 0.2;

    // 超過距離仍找不到 INDEX：
    //     HOME INDEX NOT FOUND Alarm
    //
    // 0 = 不啟用此保護，但正式參數建議一定設定。
    double indexMaxDistance_unit = 0.0;

    // 超過時間仍找不到 INDEX：
    //     HOME INDEX NOT FOUND Alarm
    //
    // 0 = 不啟用此保護。
    double indexTimeoutSec = 0.0;


    // --------------------------------------------------------
    // Final Machine Home
    //
    // INDEX Capture Position 與最後停止位置是不同的。
    // APPLY_HOME 必須使用 capturedReferencePulse。
    // --------------------------------------------------------

    double homeOffset_unit = 0.0;


    // --------------------------------------------------------
    // Optional Move To Machine Zero
    // --------------------------------------------------------

    bool moveToZero = false;

    double moveToZeroSpeed_PPS = 0.0;

    double moveToZeroAccTime = 0.2;
    double moveToZeroDecTime = 0.2;


    // --------------------------------------------------------
    // HOME Dedicated Gain
    // --------------------------------------------------------

    HomeGainConfig searchGain;
    HomeGainConfig indexGain;
};


// ============================================================
// Per-Axis HOME Runtime
//
// 這些是本次 G81 執行狀態，不寫入 Parameter File。
// ============================================================

struct HomeRuntime
{
    HomeState state =
        HomeState::IDLE;

    HomeErrorReason error =
        HomeErrorReason::NONE;

    bool selected = false;
    bool active = false;
    bool completed = false;

    bool dogDetected = false;
    bool dogReleased = false;

    bool hardLimitReleased = false;

    // SEARCH_INDEX 進入後先確認舊訊號已解除，
    // 再接受下一個 Rising Edge。
    bool referenceArmed = false;
    bool referenceDetected = false;

    bool waitingGroupBarrier = false;

    // 該 State 的運動命令是否已經下達。
    // Velocity State 用它避免 Hold -> Resume 時重設搜尋起點與 Timeout。
    // P2P State 用它判斷是否已派送 MoveToPosition。
    bool motionCommandIssued = false;

    // P2P State 原始終點是否已建立。
    // Hold 對 P2P 做 Controlled Stop 時，MotionCore 會改寫暫停點；
    // Resume 必須回到原始 stateTargetPulse，不能重新再加一次完整距離。
    bool stateTargetValid = false;

    double stateTargetPulse = 0.0;


    // --------------------------------------------------------
    // Input Edge Memory
    // --------------------------------------------------------

    bool previousDogInput = false;
    bool previousReferenceInput = false;

    uint16_t previousTouchProbeStatus = 0;

    // --------------------------------------------------------
    // Drive Touch Probe Runtime
    // --------------------------------------------------------

    HomeDriveProbePhase driveProbePhase =
        HomeDriveProbePhase::IDLE;

    double driveProbePhaseElapsedSec = 0.0;

    uint16_t driveProbeBaselineStatus = 0;
    int32_t driveProbeBaselinePosition = 0;

    uint16_t driveProbeLastFunction = 0;
    uint16_t driveProbeLastStatus = 0;
    int32_t driveProbeLastPosition = 0;

    bool driveProbeControlWritten = false;
    bool driveProbeClearConfirmed = false;
    bool driveProbeArmedConfirmed = false;
    bool driveProbeCaptureConfirmed = false;
    bool driveProbeDisarmedAfterCapture = false;


    // --------------------------------------------------------
    // Position Snapshot
    //
    // 主要採用 Pulse Domain，與 AxisContext.currentActPos 一致。
    // --------------------------------------------------------

    double searchStartPulse = 0.0;
    double stateStartPulse = 0.0;

    double switchDetectedPulse = 0.0;

    double backoffStartPulse = 0.0;

    double indexSearchStartPulse = 0.0;

    // Drive / External Hardware 回傳的原始 32-bit Capture Position。
    int32_t capturedReferenceRaw = 0;

    // 完成 unwrap / source conversion 後，統一保存成 Pulse。
    double capturedReferencePulse = 0.0;


    // --------------------------------------------------------
    // Runtime Timer
    // --------------------------------------------------------

    double stateElapsedSec = 0.0;
};


// ============================================================
// G81 Request
//
// axisMask:
//     Bit 0 = Axis 0
//     Bit 1 = Axis 1
//     ...
//     Bit 7 = Axis 7
// ============================================================

struct HomeRequest
{
    uint8_t axisMask = 0;

    HomeSequenceMode sequenceMode =
        HomeSequenceMode::SIMULTANEOUS;
};
