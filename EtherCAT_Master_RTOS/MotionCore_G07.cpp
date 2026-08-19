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

    // =========================================================
    // 🌟 核心修正：同時準備 Pulse 與 mm 的平方和計算
    // =========================================================
    double sum_sq_pulse = 0.0;  // 給底層大腦算虛擬脈衝用的
    double sum_sq_mm = 0.0;     // 給精準速度牽制算法用的

    std::vector<double> targetPos_Pulse(axes.size());

    // 1. 一視同仁地判斷預讀狀態
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

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
        groupAccTime = std::max<double>(groupAccTime, axis.G07_acc_time);
        groupDecTime = std::max<double>(groupDecTime, axis.G07_dec_time);

        // 1-2. mm 轉 Pulse
        double lead = axis.finalLead;
        if (lead < 1e-6) lead = 1.0;
        double pulsePerUnit = axis.resolution_PPR / lead;
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        // 1-3. 改用虛擬終點做為起點！
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        // 【保留旋轉軸最短路徑判斷】
        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse;

        // =========================================================
        // 🌟 1-4. 分別計算這單一軸的 Pulse 距離與 mm 物理距離
        // =========================================================
        double distancePulse = std::abs(targetPulse - startPulse);
        double distance_mm = distancePulse / pulsePerUnit;

        sum_sq_pulse += (distancePulse * distancePulse);
        sum_sq_mm += (distance_mm * distance_mm);

        // 極度重要：把這次的終點存起來，交接給下一行！
        axis.lastQueuedPulse = targetPulse;

        // =========================================================
        // 🌟 1-5. 硬體極限防呆 (這顆馬達全速跑最少需要幾秒？)
        // =========================================================
        double currentAxisMaxPPS = axis.G07_PPS;
        // 💡 若未來 G07 有專屬的 override，可在這裡乘上倍率：
        // currentAxisMaxPPS = axis.G07_PPS * axis.G07_overrideRatio;

        if (currentAxisMaxPPS > 1.0)
        {
            double timeNeeded = distancePulse / currentAxisMaxPPS;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, timeNeeded);
        }
    }

    // 防呆保護
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 2. 算出最終群組速度 (完全比照 G00 算法)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq_pulse);
    double totalDist_mm = std::sqrt(sum_sq_mm); // 完美的 3D 空間真實距離
    double groupG07Vel_PPS = 0;

    // 💡 實務建議：這裡的 5000.0 你後續可以改成從外部參數傳入
    // 例如：double targetFeedrate_mm_min = axis.G07_Feedrate;
    double targetFeedrate_mm_min = 5000.0;
    double targetFeedrate_mm_sec = targetFeedrate_mm_min / 60.0;

    if (totalDist_mm > 0.0001 && targetFeedrate_mm_sec > 0.0)
    {
        // 步驟 A：算出這段 3D 直線，用指定速度跑，理論上要花幾秒？
        double exactTimeNeeded = totalDist_mm / targetFeedrate_mm_sec;

        // 步驟 B：最慢軸牽制！如果理論時間太短 (會逼死馬達)，就強制拉長總時間
        double finalMotionTime = std::max<double>(exactTimeNeeded, maxTimeNeeded);

        // 步驟 C：把最終的安全時間，灌回給虛擬主軸當作發送頻率
        groupG07Vel_PPS = totalDist_Pulse / finalMotionTime;
    }

    // =========================================================
    // 🌟 3. 將連續/準停模式交給標籤系統決定，下達給 LineMove
    // =========================================================
    LineMove(axes, targetPos_Pulse, groupG07Vel_PPS, groupAccTime, groupDecTime, mode);
}





