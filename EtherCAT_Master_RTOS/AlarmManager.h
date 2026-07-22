#pragma once
#include <array>
#include <stdint.h> // 🌟 為了支援 uint32_t

// 簡單的警報項目結構 (無 std::string，保證 RTOS 安全)
struct AlarmItem {
    int code;
    int lineNo;
};

class AlarmManager {
public:
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
        SHM_LINK_ERROR = SYS_BASE + 2  // 1002: RTX64 共享記憶體通訊異常
    };

    // 🌟 3. NC 程式警報 (2000 ~ 2999)
    enum NCAlarm {
        SYNTAX_ERROR = NC_BASE + 1,  // 2001: 語法錯誤
        MATH_ERROR = NC_BASE + 2,  // 2002: 數學或巨集計算錯誤
        GOTO_NOT_FOUND = NC_BASE + 3   // 2003: 找不到跳躍的 N 行號
    };

    // 🌟 4. 運動軸控警報 (3000 ~ 3999)
    enum AxisAlarm {
        OVER_TRAVEL = AXIS_BASE + 1,   // 3001: 軟體極限過行程
        HARD_LIMIT = AXIS_BASE + 2,   // 3002: 硬體極限開關觸發
        SERVO_ERROR = AXIS_BASE + 3    // 3003: 伺服驅動器異常 (ALM)
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
    void Trigger(int code, int lineNo = 0);
    bool HasAlarm() const;
    int GetAlarmCount() const;
    void Clear();

    // 🌟 新增：讓外部可以撈取特定索引的警報代碼
    int GetAlarmId(int index) const;

    // 🌟 新增：取得警報更新計數器 (給 HMI_Bridge 判斷用的)
    uint32_t GetUpdateCount() const;

private:
    static constexpr int MAX_ALARMS = 64;
    std::array<AlarmItem, MAX_ALARMS> m_alarms;
    int m_alarmCount;
    bool m_hasAlarm;

    // 🌟 新增：紀錄狀態改變次數的變數
    uint32_t m_updateCount;

    AlarmManager(); // 私有建構子 (單例模式)
};