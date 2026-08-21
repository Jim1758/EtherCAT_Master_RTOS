// ==========================================================
// 檔案：NCManager_PLCSync.cpp
// 功能：處理 NCManager 將大腦與軸狀態同步給 PLC 的 S 點
// ==========================================================

#include "NCManager.h"
#include "MotionCore.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include "AlarmManager.h"

extern PLCManager* g_PLC; // 引入 PLC 大腦

void NCManager::SyncNCStateToPLC()
{
    if (g_PLC == nullptr) return; // 防呆

    // =========================================================
    // 1. 機台綜合狀態 (S0 ~ S5)
    // =========================================================
    g_PLC->Set_S(0, (m_edmState == EDMState::READY || m_edmState == EDMState::START)); // 系統備妥
    g_PLC->Set_S(1, (m_edmState == EDMState::START)); // 執行中
    g_PLC->Set_S(2, (m_edmState == EDMState::HOLD));  // 暫停中
    g_PLC->Set_S(3, (m_state == NCState::P_END));     // 程式結束
    g_PLC->Set_S(4, (m_state == NCState::RESET_STATE)); // 系統重置中
    g_PLC->Set_S(5, AlarmManager::GetInstance().HasAlarm()); // 系統警報中

    // =========================================================
    // 2. 操作模式狀態 (S10 ~ S13)
    // =========================================================
    g_PLC->Set_S(10, (m_mode == NCOperationMode::MEMORY));
    g_PLC->Set_S(11, (m_mode == NCOperationMode::MDI));
    g_PLC->Set_S(12, (m_mode == NCOperationMode::MANUAL));
    g_PLC->Set_S(13, (m_mode == NCOperationMode::EDIT));

    // =========================================================
    // 3. NC 功能啟用狀態 (S20 ~ S26) - 這些您可以之後再接上 UI 變數
    // =========================================================
    g_PLC->Set_S(20, m_isSingleBlockEnabled);
    // g_PLC->Set_S(21, m_isDryRunEnabled);
    g_PLC->Set_S(22, m_isOptionalStopEnabled);
    g_PLC->Set_S(23, m_isBlockSkipEnabled);
    // g_PLC->Set_S(24, m_isMachineLockEnabled);
    // g_PLC->Set_S(25, m_isZAxisLockEnabled);
    // g_PLC->Set_S(26, false); // 預留

    // =========================================================
    // 4. M/S/T 碼執行狀態 (S30 ~ S34)
    // =========================================================
    // 注意：目前 M/S/T 碼由 GCodeHandlers 處理，如果有 M 碼卡住等待 (FIN)，
    // 我們可以透過檢查 m_waitCallback 是否有值來簡單判斷。
    // 如果未來您有專門的 M-Code Manager，可以直接讀取它的狀態。
    bool isWaitingForMCode = (m_waitCallback != nullptr);
    g_PLC->Set_S(30, isWaitingForMCode); // M 碼等待中

    // (M00, M01 這些狀態需要您在 GCodeHandlers 處理 M00 時設定一個旗標來讀取)
    // g_PLC->Set_S(33, m_isM00Active);
    // g_PLC->Set_S(34, m_isM01Active);

    // =========================================================
    // 5. G81 HOME 全域執行狀態
    //
    // S203 = HOME Request 還存在
    // S204 = Feed Hold 正在 Controlled Stop
    // S205 = 已完全停止，等待 Cycle Start Resume
    // =========================================================

    g_PLC->Set_S(
        NCPLC::S::HOME_ACTIVE,
        Homing.IsActive());

    g_PLC->Set_S(
        NCPLC::S::HOME_HOLD_DECEL,
        Homing.IsHoldDecelerating());

    g_PLC->Set_S(
        NCPLC::S::HOME_PAUSED,
        Homing.IsPaused());


    // =========================================================
    // 6. 各軸獨立狀態 (S100 ~ S157)
    // =========================================================
    for (int i = 0; i < 8; i++)
    {
        auto& axis = m_motion.GetAxisContext(i);

        // 只有該軸存在時才更新，否則填 0
        if (axis.isExist)
        {
            g_PLC->Set_S(100 + i, axis.isServoOn);      // S100~107: 軸啟用狀態
            g_PLC->Set_S(110 + i, axis.isHomed);        // S110~117: 尋原點完成 (需要您在 AxisContext 加這變數)
            g_PLC->Set_S(150 + i, axis.isLagAlarm);     // S150~157: 追隨誤差錯誤

            const HomeState homeState = axis.homeRuntime.state;
            const bool homeSearchSwitch = axis.homeRuntime.active &&
                (homeState == HomeState::SEARCH_SWITCH || homeState == HomeState::SWITCH_DECEL_STOP || homeState == HomeState::BACK_OFF || homeState == HomeState::VALIDATE_RELEASE || homeState == HomeState::WAIT_GROUP_BACKOFF);
            const bool homeSearchIndex = axis.homeRuntime.active &&
                (homeState == HomeState::ARM_REFERENCE || homeState == HomeState::SEARCH_INDEX || homeState == HomeState::INDEX_CAPTURED || homeState == HomeState::INDEX_DECEL_STOP || homeState == HomeState::WAIT_GROUP_INDEX_STOP);

            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_DOG_BASE, i), homeSearchSwitch);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_INDEX_BASE, i), homeSearchIndex);

            // =====================================================
            // Software Travel Limit Warning State
            //
            // S210~217 = +Software Limit
            // S220~227 = -Software Limit
            //
            // 只提供 PLC / HMI 顯示警訊。
            // 不進 AlarmManager。
            // =====================================================

            const bool softwarePositiveLimit =
                axis.travelLimit1PositiveActive ||
                axis.travelLimit2PositiveActive ||
                axis.travelLimit3PositiveActive;

            const bool softwareNegativeLimit =
                axis.travelLimit1NegativeActive ||
                axis.travelLimit2NegativeActive ||
                axis.travelLimit3NegativeActive;

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_POSITIVE_LIMIT_BASE,
                    i),
                softwarePositiveLimit);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_NEGATIVE_LIMIT_BASE,
                    i),
                softwareNegativeLimit);
        }
        else
        {
            g_PLC->Set_S(100 + i, false);
            g_PLC->Set_S(110 + i, false);
            g_PLC->Set_S(150 + i, false);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_DOG_BASE, i), false);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_INDEX_BASE, i), false);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_POSITIVE_LIMIT_BASE,
                    i),
                false);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_NEGATIVE_LIMIT_BASE,
                    i),
                false);
        }
    }
}