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
    double sum_sq = 0.0;        // 3D Pulse 距離平方和

    std::vector<double> targetPos_Pulse;
    targetPos_Pulse.resize(axes.size());

    // 🌟 1. 一視同仁地判斷預讀狀態 (雖然 G53 通常是 false，但統一架構最安全)
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

        double lead = axis.finalLead;
        if (lead < 1e-6) lead = 1.0;
        double pulsePerUnit = axis.resolution_PPR / lead;
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        // =========================================================
        // 🌟 2. 統一使用虛擬終點作為起點！
        // =========================================================
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse;
        double distancePulse = std::abs(targetPulse - startPulse);
        sum_sq += (distancePulse * distancePulse);

        // =========================================================
        // 🌟 3. 極度重要：把這次的 G53 終點存起來，給未來的指令當起點！
        // =========================================================
        axis.lastQueuedPulse = targetPulse;

        double currentAxisMaxPPS = axis.G53_PPS;
        // 修正你的筆誤：這裡應該是看 currentAxisMaxPPS 而不是 G00_PPS
        if (currentAxisMaxPPS > 1.0) {
            double timeNeeded = distancePulse / currentAxisMaxPPS;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, timeNeeded);
        }
    }

    // 防呆保護
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 2. 算出最終群組速度 (PPS)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq);
    double groupG53Vel_PPS = 1000.0; // 預設底速

    // 將總 Pulse 距離 / 瓶頸時間 = 完美的群組 PPS 速度
    if (maxTimeNeeded > 0.0001) {
        groupG53Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // =========================================================
    // 🌟 3. 丟給 LineMove
    // =========================================================
    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP);

    // 完美傳入 Pulse 陣列與計算好的 PPS 速度
    LineMove(axes, targetPos_Pulse, groupG53Vel_PPS, groupAccTime, groupDecTime, BufferMode::ABORTING);

   
}

