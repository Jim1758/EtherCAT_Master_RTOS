#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
void MotionCore::G00_Move(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode)
{
    // Backward-compatible API: preserve the accepted G00 mapping for any
    // out-of-tree caller that has not yet supplied the independent K.6 field.
    const MotionCommandPathMode commandPathMode =
        mode == BufferMode::BUFFERED
        ? MotionCommandPathMode::CONTINUOUS
        : MotionCommandPathMode::EXACT_STOP;

    G00_Move(
        axes,
        targetPos_mm,
        mode,
        commandPathMode);
}

void MotionCore::G00_Move(
    const std::vector<int>& axes,
    const std::vector<double>& targetPos_mm,
    BufferMode mode,
    MotionCommandPathMode commandPathMode)
{
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0;

    double sum_sq_pulse = 0.0; // 虛擬主軸用的 Pulse 總長度
    double sum_sq_mm = 0.0;    // 🌟 新增：真實空間的 mm 總長度

    std::vector<double> targetPos_Pulse(axes.size());
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        if (!axis.isExist) {
            AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
            return;
        }

        groupAccTime = std::max<double>(groupAccTime, axis.G00_acc_time);
        groupDecTime = std::max<double>(groupDecTime, axis.G00_dec_time);

        double lead = axis.finalLead;
        if (lead < 1e-6) lead = 1.0;
        double pulsePerUnit = axis.resolution_PPR / lead;
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse;

        // 1. 累加 Pulse 距離的平方 (給底層引擎用)
        double distancePulse = std::abs(targetPulse - startPulse);
        sum_sq_pulse += (distancePulse * distancePulse);

        // 🌟 2. 累加 mm 距離的平方 (給精準速度計算用)
        double distance_mm = distancePulse / pulsePerUnit;
        sum_sq_mm += (distance_mm * distance_mm);

        axis.lastQueuedPulse = targetPulse;

        // 計算牽制時間 (單軸硬體極限防呆)
        double currentAxisMaxPPS = axis.G00_PPS * G00_overrideRatio;
        if (currentAxisMaxPPS > 1.0) {
            double timeNeeded = distancePulse / currentAxisMaxPPS;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, timeNeeded);
        }
    }

    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    double totalDist_Pulse = std::sqrt(sum_sq_pulse);
    double totalDist_mm = std::sqrt(sum_sq_mm); // 🌟 算出真正的 3D 空間移動距離

    double groupG00Vel_PPS = 0;

    // =========================================================
    // 🌟 [神級修復]：精準空間向量速度算法
    // =========================================================
    // 這裡我們暫時讀取你測試用的 5000 mm/min (5米速度)，你之後可以從 NC 解碼傳 G01 的 F 值進來
    double targetFeedrate_mm_min = 5000.0; // 假設要求空間走 5 米
    double targetFeedrate_mm_sec = targetFeedrate_mm_min / 60.0;

    if (totalDist_mm > 0.0001 && targetFeedrate_mm_sec > 0.0)
    {
        // 1. 算出這段 3D 直線，用 5 米速度跑，理論上要花幾秒？
        double exactTimeNeeded = totalDist_mm / targetFeedrate_mm_sec;

        // 2. 最慢軸牽制：如果用 5 米跑會逼死某一顆馬達，就強迫拉長總時間 (降速)
        double finalMotionTime = std::max<double>(exactTimeNeeded, maxTimeNeeded);

        // 3. 把最終的安全時間，灌回給你的 Pulse 虛擬主軸
        groupG00Vel_PPS = totalDist_Pulse / finalMotionTime;
    }

    // Stage NC-0.2K.6.1 authority cutover:
    // The NC Producer must not write m_Group.pathMode.  The explicit policy is
    // release-published with this MotionCommand and only the authorized 250 us
    // Consumer may apply it at the committed handoff.  BufferMode still owns
    // admission/epoch behavior and remains an independent dimension.

    LineMove(
        axes,
        targetPos_Pulse,
        groupG00Vel_PPS,
        groupAccTime,
        groupDecTime,
        mode,
        commandPathMode);


}





