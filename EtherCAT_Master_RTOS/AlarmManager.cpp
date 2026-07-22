#include "AlarmManager.h"
// #include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 或 RtPrintf 請取消註解

AlarmManager::AlarmManager() {
    m_alarmCount = 0;
    m_hasAlarm = false;
    m_updateCount = 0; // 🌟 初始化計數器
    for (int i = 0; i < MAX_ALARMS; ++i) {
        m_alarms[i] = { 0, 0 };
    }
}

AlarmManager& AlarmManager::GetInstance() {
    static AlarmManager instance;
    return instance;
}

void AlarmManager::Trigger(int code, int lineNo) {
    // 將錯誤代碼存入陣列 (若超過 MAX_ALARMS 就不再紀錄，避免陣列越界)
    if (m_alarmCount < MAX_ALARMS) {
        m_alarms[m_alarmCount].code = code;
        m_alarms[m_alarmCount].lineNo = lineNo;
        m_alarmCount++;
    }

    m_hasAlarm = true; // 升起警報旗標
    m_updateCount++;   // 🌟 狀態改變，計數器 +1

    // DEBUG_PRINT("[ALARM] Code: %d at Line: %d\n", code, lineNo);
}

void AlarmManager::Clear() {
    if (m_hasAlarm) {
        m_hasAlarm = false;
        m_alarmCount = 0;
        m_updateCount++; // 🌟 警報消除也是一種狀態改變，計數器也要 +1 讓 HMI 知道
    }
}

bool AlarmManager::HasAlarm() const {
    return m_hasAlarm;
}

int AlarmManager::GetAlarmCount() const {
    return m_alarmCount;
}

// 🌟 新增實作：安全地回傳警報代碼
int AlarmManager::GetAlarmId(int index) const {
    if (index >= 0 && index < m_alarmCount) {
        return m_alarms[index].code;
    }
    return 0; // 越界保護
}

// 🌟 新增實作：回傳計數器
uint32_t AlarmManager::GetUpdateCount() const {
    return m_updateCount;
}