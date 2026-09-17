#pragma once
#include <array>
#include <stdint.h> // 🌟 為了支援 uint32_t
#include <cstdint>
#include <atomic> // 🌟 確保多執行緒安全
// 簡單的警報項目結構 (無 std::string，保證 RTOS 安全)
//
// Alarm producers are serialized, but HMI readers are independent.  Keep
// each field atomic so Clear/Trigger can never form a C++ data race with a
// reader which sampled the preceding published count.
struct AlarmItem {
    std::atomic<int> code{ 0 };
    std::atomic<int> lineNo{ 0 };
    std::atomic<int> axisIndex{ -1 }; // -1 表示無關
};

class AlarmManager {
public:
    // Fixed-cost Alarm-to-Motion admission reservation. Trigger/Clear never
    // wait: they advance sequence+active in one RMW, so a holder either owns
    // an exact quiet revision or detects the overlapping publication.
    struct MotionAdmissionReservation
    {
        std::uint64_t baseState = 0ULL;
        std::uint64_t reservedState = 0ULL;
        std::uint32_t expectedUpdateCount = 0U;
        bool requireNoAlarm = true;
        bool acquired = false;
    };

    enum class MotionAdmissionResult : std::uint8_t
    {
        ACQUIRED = 0,
        BUSY,
        SUPERSEDED
    };

    // 錯誤代碼列舉
   // 🌟 1. 定義分類的基準值 (Base Offsets)
    enum CategoryBase {
        SYS_BASE = 1000, // 系統與通訊層級
        NC_BASE = 2000, // 程式解析與語法層級
        AXIS_BASE = 3000, // 運動與伺服軸層級
        EDM_BASE = 4000  // 放電加工製程層級
    };

    // 🌟 2. 系統警報 (1000 ~ 1999)
    enum SysAlarm {
        NONE = 0,
        EMG_STOP = SYS_BASE + 1, // 1001: 緊急停止觸發
        SHM_LINK_ERROR = SYS_BASE + 2, // 1002: RTX64 共享記憶體通訊異常
        ETHERCAT_PDO_SAFETY_STOP = SYS_BASE + 3 // 1003: 連續 PDO 無效觸發安全停止
    };

    // 🌟 3. NC 程式警報 (2000 ~ 2999)
    enum NCAlarm {
        SYNTAX_ERROR = NC_BASE + 1,  // 2001: 語法錯誤
        MATH_ERROR = NC_BASE + 2,  // 2002: 數學或巨集計算錯誤
        GOTO_NOT_FOUND = NC_BASE + 3,   // 2003: 找不到跳躍的 N 行號
        MACRO_OVERFLOW = NC_BASE + 4,   // 2004:超過最大層數
        Macro_File_Not_Found = NC_BASE + 5,   // 2005:Macro找不到 檔案
        Unable_to_recognize_G_code = NC_BASE + 6,   // 2006:無法辨識G碼
        G_code_Count_Error = NC_BASE + 7,   // 2007:同一 Modal Group 衝突或單節含多個 Primary Action
        M_code_Count_Error = NC_BASE + 8,   // 2008:單行M碼超過1個
        G_Code_Invalid_parameter = NC_BASE + 9,   // 2009:不正確G碼參數
        axis_is_not_enabledr = NC_BASE + 10,   // 2010:下達不存在的軸指令
        axis_is_not_Homed = NC_BASE + 11,   // 2011:軸尚未尋原點
        PROGRAM_END_GATE_ERROR = NC_BASE + 12, // 2012:程式結束完整性閘門失敗
        MACRO_VARIABLE_INDEX_OUT_OF_RANGE = NC_BASE + 13, // 2013:Macro變數索引超出允許範圍
        PATH_EXECUTION_NOT_READY = NC_BASE + 14, // 2014:路徑執行狀態或設定未就緒 / Path execution not ready
        PATH_GEOMETRY_INVALID = NC_BASE + 15, // 2015:路徑座標或幾何無效 / Invalid path geometry
        PATH_REPLAY_HISTORY_UNAVAILABLE = NC_BASE + 16, // 2016:回退歷史或回放狀態不可用 / Replay history or state unavailable
        PATH_REPLAY_HISTORY_CAPACITY = NC_BASE + 17, // 2017:回退歷史超過固定容量 / Replay history capacity exceeded
        PATH_MOTION_NOT_ADMITTED = NC_BASE + 18, // 2018:路徑Motion命令未受理 / Path motion not admitted
        PATH_RETREAT_UNAVAILABLE = NC_BASE + 19, // 2019:要求的回退路徑不可用 / Requested retreat path unavailable
        GAP_INPUT_TEST_FAILED = NC_BASE + 20, // 2020: GAP input simulation self-test failed; see [GAP-CG].
        GAP_PATH_SIMULATION_FAILED = NC_BASE + 21, // 2021: GAP 路徑模擬訊號異常 / GAP path simulation signal invalid; see [GAP-CH].
        IDLE_POSITION_HOLD_FAILED = NC_BASE + 22, // 2022: 程序結束後保持位置無效 /Post-program position hold invalid; see [IDLE-CJ].
        PATH_INVALIDATED_BY_GOTO = NC_BASE + 23, // 2023: GOTO 後無法接續路徑運動 / Path motion invalidated by GOTO
    };

    // 🌟 4. 運動軸控警報 (3000 ~ 3999)
    enum AxisAlarm {
        OVER_TRAVEL = AXIS_BASE + 1,   // 3001: 軟體極限過行程
        HARD_LIMIT = AXIS_BASE + 2,   // 3002: 硬體極限開關觸發
        SERVO_ERROR = AXIS_BASE + 3,    // 3003: 伺服驅動器異常 (ALM)
        AXIS_LAG_ERROR = AXIS_BASE + 4,    // 3004: 追隨誤差過大 (Lag Error)
        AXIS_Fault = AXIS_BASE + 5,   // 3005: 驅動器硬體內部報警 (Fault)
        MANUAL_AXIS_PROTECT = AXIS_BASE + 6,   // 3006: 手動軸保護輸入觸發
        PROGRAMMED_OVER_TRAVEL = AXIS_BASE + 7, // 3007: NC 路徑預測會碰觸軟體極限

        // =====================================================
        // G81 HOME 尋原點警報
        //
        // 只要進入 AlarmManager：
        //
        //     NC -> ALARM
        //     Motion -> Emergency Stop
        //
        // 正常 DOG / INDEX 找到後的滑行停止不是 Alarm。
        // =====================================================

        HOME_INVALID_CONFIG = AXIS_BASE + 8,        // 3008: HOME 參數或方法設定錯誤
        HOME_SEARCH_NOT_FOUND = AXIS_BASE + 9,      // 3009: 超過距離或時間仍找不到 DOG / 預期 LIMIT
        HOME_SWITCH_STOP_DISTANCE = AXIS_BASE + 10, // 3010: DOG / LIMIT 觸發後滑行停止距離過大
        HOME_BACKOFF_FAILED = AXIS_BASE + 11,       // 3011: Backoff 超過最大距離或時間仍未完成
        HOME_DOG_NOT_RELEASED = AXIS_BASE + 12,     // 3012: Backoff 完成後 HOME DOG 仍未解除
        HOME_LIMIT_NOT_RELEASED = AXIS_BASE + 13,   // 3013: Backoff 完成後硬體極限仍未解除
        HOME_INDEX_NOT_FOUND = AXIS_BASE + 14,      // 3014: 超過距離或時間仍找不到 INDEX
        HOME_REFERENCE_INVALID = AXIS_BASE + 15,    // 3015: INDEX / Absolute Reference 資料無效
        HOME_OPPOSITE_LIMIT = AXIS_BASE + 16,       // 3016: HOME 過程觸發相反方向硬體極限
        HOME_BOTH_LIMITS = AXIS_BASE + 17,          // 3017: 正負硬體極限同時觸發
        HOME_MOTION_FAULT = AXIS_BASE + 18,         // 3018: HOME 過程發生 Servo / Motion Fault

        // Stage NC-0.1C：固定容量 Motion Command Transport
        MOTION_COMMAND_QUEUE_FULL = AXIS_BASE + 19, // 3019: NC -> Motion SPSC Ingress 已滿
        MOTION_REPLAY_QUEUE_FULL = AXIS_BASE + 20,  // 3020: B2 RT Replay 固定緩衝區已滿
        MOTION_GROUP_MAPPING_INTEGRITY = AXIS_BASE + 21 // 3021: 插補映射／孤兒軸完整性失敗

    };

    // 🌟 5. EDM 放電警報 (4000 ~ 4999)
    enum EDMAlarm {
        SHORT_CIRCUIT = EDM_BASE + 1,  // 4001: 極間嚴重短路
        FLUID_LOW = EDM_BASE + 2,  // 4002: 加工液位過低
        TEMP_HIGH = EDM_BASE + 3   // 4003: 加工液溫度過高
    };

    // 取得單例
    static AlarmManager& GetInstance();

    // 觸發與查詢函式
    void Trigger(int code, int lineNo = 0, int axisIndex = -1);
    bool HasAlarm() const;
    int GetAlarmCount() const;
    void Clear();

    // 🌟 新增：讓外部可以撈取特定索引的警報代碼
    int GetAlarmId(int index) const;
    int GetAlarmAxisIndex(int index) const; // 🌟 新增：撈取軸編號
    // 🌟 新增：取得警報更新計數器 (給 HMI_Bridge 判斷用的)
    uint32_t GetUpdateCount() const;
    std::uint64_t GetMotionSafetyIntentState() const noexcept;
    static std::uint64_t MotionAdmissionBaseState(
        std::uint64_t state) noexcept;
    MotionAdmissionResult TryBeginMotionAdmission(
        std::uint32_t expectedUpdateCount,
        MotionAdmissionReservation& reservation,
        bool requireNoAlarm = true) noexcept;
    MotionAdmissionResult TryBeginMotionAdmission(
        std::uint32_t expectedUpdateCount,
        std::uint64_t expectedIntentState,
        MotionAdmissionReservation& reservation,
        bool requireNoAlarm = true) noexcept;
    bool BeginMotionAdmission(
        std::uint32_t expectedUpdateCount,
        MotionAdmissionReservation& reservation,
        bool requireNoAlarm = true) noexcept;
    bool IsMotionAdmissionCurrent(
        const MotionAdmissionReservation& reservation) const noexcept;
    bool EndMotionAdmission(
        MotionAdmissionReservation& reservation) noexcept;
    bool ClearUnderMotionAdmission(
        MotionAdmissionReservation& reservation) noexcept;
    bool TriggerUnderMotionAdmission(
        MotionAdmissionReservation& reservation,
        int code,
        int lineNo = 0,
        int axisIndex = -1) noexcept;

private:
    static constexpr std::uint64_t MOTION_ADMISSION_ACTIVE_MASK =
        0x000000007FFFFFFFULL;
    static constexpr std::uint64_t MOTION_ADMISSION_RESERVED =
        0x0000000080000000ULL;
    static constexpr std::uint64_t MOTION_ALARM_INTENT_BEGIN_DELTA =
        0x0000000100000001ULL;
    static constexpr std::uint64_t MOTION_ALARM_INTENT_SEQUENCE_DELTA =
        0x0000000100000000ULL;

    static constexpr std::uint64_t DEFERRED_ALARM_CODE_MASK =
        0x000000000000FFFFULL;
    static constexpr unsigned DEFERRED_ALARM_LINE_SHIFT = 16U;
    static constexpr std::uint64_t DEFERRED_ALARM_LINE_MASK =
        0x0000FFFFFFFF0000ULL;
    static constexpr unsigned DEFERRED_ALARM_AXIS_SHIFT = 48U;
    static constexpr std::uint64_t DEFERRED_ALARM_AXIS_MASK =
        0x00FF000000000000ULL;
    static constexpr unsigned DEFERRED_ALARM_COUNT_SHIFT = 56U;
    static constexpr std::uint64_t DEFERRED_ALARM_COUNT_MASK =
        0x7F00000000000000ULL;
    static constexpr std::uint32_t DEFERRED_ALARM_COUNT_MAX = 0x7FU;
    static constexpr std::uint64_t DEFERRED_ALARM_OVERFLOW =
        0x8000000000000000ULL;
    static constexpr unsigned DEFERRED_ALARM_CAS_ATTEMPTS = 32U;

    static constexpr int MAX_ALARMS = 64;
    std::array<AlarmItem, MAX_ALARMS> m_alarms;

    // 🌟 改用 atomic，RTOS 寫、HMI 讀才不會崩潰
    std::atomic<int> m_alarmCount{ 0 };
    std::atomic<bool> m_hasAlarm{ false };
    std::atomic<uint32_t> m_updateCount{ 0 };
    std::atomic<std::uint64_t> m_motionSafetyIntentState{ 0ULL };
    // First-payload + count are one atomic publication, so multiple Alarm
    // producers can never tear code/line/axis or overwrite a claimed slot.
    std::atomic<std::uint64_t> m_deferredAlarmPublication{ 0ULL };
    // Only used when the bounded packed CAS budget is exhausted.  It keeps
    // the overflow-only diagnostic payload coherent; the packed overflow bit
    // remains the safety/liveness authority.
    std::atomic<std::uint64_t> m_deferredAlarmOverflowPayload{ 0ULL };
    std::atomic<bool> m_alarmWriterReserved{ false };

    void DeferAlarm(int code, int lineNo, int axisIndex) noexcept;
    void TryPromoteDeferredAlarms() noexcept;
    void TryPromoteDeferredAlarmsOnce() noexcept;
    bool HasDeferredAlarmPublication() const noexcept;
    void PublishAlarmUnderWriter(
        int code,
        int lineNo,
        int axisIndex,
        std::uint32_t updateDelta) noexcept;
    MotionAdmissionResult TryBeginMotionAdmissionImpl(
        std::uint32_t expectedUpdateCount,
        const std::uint64_t* expectedIntentState,
        MotionAdmissionReservation& reservation,
        bool requireNoAlarm) noexcept;

    // RTSS PDO callbacks must never enter a C++ function-local-static guard.
    // The instance is constructed by the module during startup, before the
    // timer callback can run; GetInstance() is then a plain reference return.
    static AlarmManager s_instance;

    AlarmManager();
    AlarmManager(const AlarmManager&) = delete; // 禁止複製
};
