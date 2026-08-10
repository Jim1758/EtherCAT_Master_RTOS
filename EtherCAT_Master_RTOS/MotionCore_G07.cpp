#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
void MotionCore::G07_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0; // 紀錄跑最久的那軸需要幾秒
    double sum_sq = 0.0;        // 3D Pulse 距離平方和

    std::vector<double> targetPos_Pulse;
    targetPos_Pulse.resize(axes.size());

    // 🌟 1. 一視同仁地判斷預讀狀態
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    // =========================================================
      // 🌟 2. 將 mm 轉為 Pulse，並計算出「誰花的時間最長」
      // =========================================================
    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        // =========================================================
        // 🌟 【新增底層防呆】如果有未啟用的軸被派單，直接拒絕執行！
        // =========================================================
        if (!axis.isExist)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
            return; // 立即跳出，不發車！
        }

        // 1-1. 取加減速最大值 (最安全的煞車距離)
        groupAccTime = std::max<double>(groupAccTime, axis.G07_acc_time);
        groupDecTime = std::max<double>(groupDecTime, axis.G07_dec_time);

        // 1-2. mm 轉 Pulse
        double lead = axis.finalLead;
        if (lead < 1e-6) lead = 1.0;
        double pulsePerUnit = axis.resolution_PPR / lead;
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        // =========================================================
        // 🌟 1-3. 改用虛擬終點做為起點！
        // =========================================================
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        // 【保留旋轉軸最短路徑判斷】
        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse; // 存入給 LineMove 用的陣列
        double distancePulse = std::abs(targetPulse - startPulse);
        sum_sq += (distancePulse * distancePulse);

        // =========================================================
        // 🌟 1-4. 極度重要：把這次的終點存起來，交接給下一行！
        // =========================================================
        axis.lastQueuedPulse = targetPulse;

        // =========================================================
        // 解決速度問題的核心：算出這根軸如果全速跑，要花幾秒？
        // =========================================================
        double currentAxisMaxPPS = axis.G07_PPS;
        // =========================================================
 // 防呆：真正當分母的是乘完 Override 後的速度
 // =========================================================
        if (currentAxisMaxPPS > 1.0)
        {
            double timeNeeded =
                distancePulse /
                currentAxisMaxPPS;

            maxTimeNeeded =
                std::max<double>(
                    maxTimeNeeded,
                    timeNeeded);
        }
    }

    // 防呆保護
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 3. 算出最終群組速度 (PPS)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq);
    double groupG07Vel_PPS = 0; // 預設底速

    // 將總 Pulse 距離 / 瓶頸時間 = 完美的群組 PPS 速度
    if (maxTimeNeeded > 0.0001) {
        groupG07Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // =========================================================
    // 🌟 4. 刪除所有 SetGroupPathMode，直接丟給 LineMove
    // =========================================================

    // 完美傳入 Pulse 陣列與計算好的 PPS 速度，將連續/準停模式交給標籤系統決定！
    LineMove(axes, targetPos_Pulse, groupG07Vel_PPS, groupAccTime, groupDecTime, mode);
}





