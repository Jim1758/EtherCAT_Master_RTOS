#include "AlarmManager.h"
// #include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 或 RtPrintf 請取消註解

/*

// 發現 G 碼寫錯，直接呼叫 (此時 axisIndex 自動 = -1)
AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);

// 或是帶上行號，但依然不需要軸號
AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR, 15);


// 假設現在迴圈跑到第 2 軸 (Z 軸)，發生了追隨誤差
int axisId = 2; // 0=X, 1=Y, 2=Z...
AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_LAG_ERROR, 0, axisId);

*/



AlarmManager::AlarmManager() {
    m_alarmCount.store(0);
    m_hasAlarm.store(false);
    m_updateCount.store(0);
    for (int i = 0; i < MAX_ALARMS; ++i) {
        m_alarms[i] = { 0, 0, -1 }; // 🌟 記得初始化 axisIndex 為 -1
    }
}

AlarmManager& AlarmManager::GetInstance() {
    static AlarmManager instance;
    return instance;
}

void AlarmManager::Trigger(int code, int lineNo, int axisIndex) {
    int count = m_alarmCount.load();
    if (count < MAX_ALARMS) {
        m_alarms[count].code = code;
        m_alarms[count].lineNo = lineNo;
        m_alarms[count].axisIndex = axisIndex;
        m_alarmCount.fetch_add(1);
    }
    m_hasAlarm.store(true);
    m_updateCount.fetch_add(1);
}

void AlarmManager::Clear() {
    if (m_hasAlarm.load()) {
        m_hasAlarm.store(false);
        m_alarmCount.store(0);
        // 🌟 警報消除時，把舊資料包含 axisIndex 也洗乾淨
        for (int i = 0; i < MAX_ALARMS; ++i) {
            m_alarms[i] = { 0, 0, -1 };
        }
        m_updateCount.fetch_add(1);
    }
}

bool AlarmManager::HasAlarm() const {
    return m_hasAlarm.load();
}

int AlarmManager::GetAlarmCount() const {
    return m_alarmCount.load();
}

int AlarmManager::GetAlarmId(int index) const {
    if (index >= 0 && index < m_alarmCount.load()) {
        return m_alarms[index].code;
    }
    return 0;
}

int AlarmManager::GetAlarmAxisIndex(int index) const {
    if (index >= 0 && index < m_alarmCount.load()) {
        return m_alarms[index].axisIndex;
    }
    return -1;
}

uint32_t AlarmManager::GetUpdateCount() const {
    return m_updateCount.load();
}
