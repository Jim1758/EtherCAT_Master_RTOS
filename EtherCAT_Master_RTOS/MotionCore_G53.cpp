#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

void MotionCore::G53_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0; // 紀錄跑最久的那軸需要幾秒

    // =========================================================
    // 🌟 對齊完美架構：準備 Pulse 與 mm 雙軌計算
    // =========================================================
    double sum_sq_pulse = 0.0;  // 給底層大腦算虛擬脈衝用的
    double sum_sq_mm = 0.0;     // 給精準空間牽制算法用的

    std::vector<double> targetPos_Pulse(axes.size());

    // 🌟 1. 一視同仁地判斷預讀狀態 (雖然 G53 通常是獨立執行，但統一架構最安全)
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        if (!axis.isExist) {
            AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
            return;
        }

        groupAccTime = std::max<double>(groupAccTime, axis.G53_acc_time);
        groupDecTime = std::max<double>(groupDecTime, axis.G53_dec_time);

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

        // =========================================================
        // 🌟 2. 統一使用虛擬終點作為起點！
        // =========================================================
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        if (!TryResolveMotionTargetPulse(startPulse, targetPulse, pulsePerUnit,
            rotaryShortestPath, axis.rotaryModulo, targetPulse))
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, m_pendingSourcePC, idx);
            return;
        }

        targetPos_Pulse[i] = targetPulse;

        // 🌟 幾何距離雙計算 (精準防變形)
        double distancePulse = std::abs(targetPulse - startPulse);
        double distance_mm = distancePulse / pulsePerUnit;

        sum_sq_pulse += (distancePulse * distancePulse);
        sum_sq_mm += (distance_mm * distance_mm);

        // =========================================================
        // 🌟 3. 極度重要：把這次的 G53 終點存起來，給未來的指令當起點！
        // =========================================================
        // Publish all pulse tails only after every axis conversion succeeds.

        // 🌟 【神級修復】防止除以零的防呆寫法，讀取 G53 專屬極速
        double currentAxisMaxPPS = axis.G53_PPS;
        if (currentAxisMaxPPS > 1.0) {
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
    // 🌟 4. 算出最終群組速度 (PPS)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq_pulse);
    double totalDist_mm = std::sqrt(sum_sq_mm);
    double groupG53Vel_PPS = 0.0;

    // 💡 G53 通常以機台極限速度移動 (與 G00 類似)，以最慢軸牽制時間為主
    if (maxTimeNeeded > 0.0001) {
        groupG53Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // =========================================================
    // 🌟 5. 丟給 LineMove (動態決定準停與連續模式)
    // =========================================================
    // 配合你傳入的 mode，動態切換群組行為，不再死寫 EXACT_STOP
    if (mode == BufferMode::ABORTING) {
        SetGroupPathMode(PathMode::EXACT_STOP);
    }
    else {
        SetGroupPathMode(PathMode::CONTINUOUS);
    }

    // 🌟 【關鍵修正】把死寫的 BufferMode::ABORTING 換成上層傳進來的 mode
    LineMove(axes, targetPos_Pulse, groupG53Vel_PPS, groupAccTime, groupDecTime, mode);
}

