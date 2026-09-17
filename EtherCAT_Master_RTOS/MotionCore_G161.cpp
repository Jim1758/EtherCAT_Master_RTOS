#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
void MotionCore::G161_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0; // 紀錄跑最久的那軸需要幾秒

    // =========================================================
    // 🌟 核心修正：準備 Pulse 與 mm 雙軌計算 (消滅空間變形)
    // =========================================================
    double sum_sq_pulse = 0.0;  // 給底層大腦算虛擬脈衝用的
    double sum_sq_mm = 0.0;     // 給精準空間算法牽制用的

    std::vector<double> targetPos_Pulse(axes.size());

    // 🌟 1. 一視同仁地判斷預讀狀態
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    // =========================================================
    // 🌟 2. 計算雙軌距離與極限時間
    // =========================================================
    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        // 【新增底層防呆】如果有未啟用的軸被派單，直接拒絕執行！
        if (!axis.isExist)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
            return;
        }

        // 1-1. 取加減速最大值 (最安全的煞車距離)
        groupAccTime = std::max<double>(groupAccTime, axis.G161_acc_time);
        groupDecTime = std::max<double>(groupDecTime, axis.G161_dec_time);

        // 1-2. mm 轉 Pulse
        const bool rotaryShortestPath =
            axis.axisType == AxisType::ROTARY && axis.useShortestPath;
        double pulsePerUnit = 0.0;
        if (!TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
            rotaryShortestPath, pulsePerUnit))
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, m_pendingSourcePC, idx);
            return;
        }
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        // 1-3. 改用虛擬終點做為起點！
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        // 【保留旋轉軸最短路徑判斷】
        if (!TryResolveMotionTargetPulse(startPulse, targetPulse, pulsePerUnit,
            rotaryShortestPath, axis.rotaryModulo, targetPulse))
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, m_pendingSourcePC, idx);
            return;
        }

        targetPos_Pulse[i] = targetPulse;

        // =========================================================
        // 🌟 幾何距離雙計算 (精準防變形)
        // =========================================================
        double distancePulse = std::abs(targetPulse - startPulse);
        double distance_mm = distancePulse / pulsePerUnit;

        sum_sq_pulse += (distancePulse * distancePulse);
        sum_sq_mm += (distance_mm * distance_mm);

        // 極度重要：把這次的終點存起來，交接給下一行！
        // Publish all pulse tails only after every axis conversion succeeds.

        // =========================================================
        // 🌟 硬體極限防呆 (這顆馬達全速跑最少需要幾秒？)
        // =========================================================
        double currentAxisMaxPPS = axis.G161_PPS;

        if (currentAxisMaxPPS > 1.0)
        {
            double timeNeeded = distancePulse / currentAxisMaxPPS;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, timeNeeded);
        }
    }

    for (size_t i = 0; i < axes.size(); ++i)
    {
        (*m_pContexts)[axes[i]].lastQueuedPulse = targetPos_Pulse[i];
    }

    // 防呆保護
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 3. 算出最終群組速度 (PPS)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq_pulse);
    double totalDist_mm = std::sqrt(sum_sq_mm);

    // 🌟 修正複製貼上的變數名稱：改為 groupG161Vel_PPS
    double groupG161Vel_PPS = 0;

    // 💡 決策點：
    // 若 G161 是純粹的「極速移動」(像 G00)，只需這段：
    if (maxTimeNeeded > 0.0001) {
        groupG161Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // 💡 若 G161 是需要外部指定速度 (例如 5 m/min 同動)，請替換為：
    /*
    double targetFeedrate_mm_sec = 5000.0 / 60.0;
    if (totalDist_mm > 0.0001 && targetFeedrate_mm_sec > 0.0)
    {
        double exactTimeNeeded = totalDist_mm / targetFeedrate_mm_sec;
        double finalMotionTime = std::max<double>(exactTimeNeeded, maxTimeNeeded);
        groupG161Vel_PPS = totalDist_Pulse / finalMotionTime;
    }
    */

    // =========================================================
    // 🌟 4. 下達給 LineMove
    // =========================================================
    // 完美傳入 Pulse 陣列與計算好的 PPS 速度，將連續/準停模式交給標籤系統決定！
    LineMove(axes, targetPos_Pulse, groupG161Vel_PPS, groupAccTime, groupDecTime, mode);
}





