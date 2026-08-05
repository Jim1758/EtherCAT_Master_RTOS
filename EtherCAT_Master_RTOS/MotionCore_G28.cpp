#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能



// 🌟 預防性裝甲：強制 8-Byte 對齊，保護 std::vector 不受 EtherCAT PDO 污染
#pragma pack(push, 8)
struct MoveResult {
    bool success;
    std::vector<double> targetPulse;
    double vel;
    double acc;
    double dec;
};
#pragma pack(pop)

// ==========================================================

void MotionCore::G28_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.empty()) return;

    // =========================================================
    // 🌟 1. 一視同仁地判斷預讀狀態
    // =========================================================
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    std::vector<double> simulatedStartPulse(axes.size());
    for (size_t i = 0; i < axes.size(); ++i) {
        AxisContext& axis = (*m_pContexts)[axes[i]];

        // 🌟 2. 初始起點：改吃虛擬終點！
        simulatedStartPulse[i] = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;
    }

    auto processMove = [&](const std::vector<double>& target_mm) -> MoveResult {
        MoveResult result;
        result.success = false;
        result.vel = 0.0;
        result.acc = 0.2; // 安全值
        result.dec = 0.2; // 安全值

        if (target_mm.size() != axes.size()) return result;

        double groupAccTime = 0.0, groupDecTime = 0.0, maxTimeNeeded = 0.0, sum_sq = 0.0;
        std::vector<double> target_Pulse(axes.size());

        for (size_t i = 0; i < axes.size(); ++i) {
            int idx = axes[i];
            AxisContext& axis = (*m_pContexts)[idx];

            if (!axis.isExist) return result;

            double safe_acc = (axis.G28_acc_time > 0.01) ? axis.G28_acc_time : 0.2;
            double safe_dec = (axis.G28_dec_time > 0.01) ? axis.G28_dec_time : 0.2;
            groupAccTime = std::max<double>(groupAccTime, safe_acc);
            groupDecTime = std::max<double>(groupDecTime, safe_dec);

            double lead = (axis.finalLead < 1e-6) ? 1.0 : axis.finalLead;
            double targetPulse = target_mm[i] * (axis.resolution_PPR / lead);

            // 這裡吃到的會是迴圈外準備好的 simulatedStartPulse
            double startPulse = simulatedStartPulse[i];

            if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
                targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
            }

            target_Pulse[i] = targetPulse;
            double dist = std::abs(targetPulse - startPulse);
            sum_sq += (dist * dist);

            double safe_PPS = (axis.G28_PPS > 10.0) ? axis.G28_PPS : 0;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, dist / safe_PPS);

            // 🌟 這裡你原本寫的非常好！把算完的終點存起來，當作下一段(如果有)的起點
            simulatedStartPulse[i] = targetPulse;
        }

        double totalDist = std::sqrt(sum_sq);
        double vel = (maxTimeNeeded > 0.0001) ? (totalDist / maxTimeNeeded) : 0;

        result.success = true;
        result.targetPulse = target_Pulse;
        result.vel = vel;
        result.acc = groupAccTime;
        result.dec = groupDecTime;

        return result;
    };

    // ---------------------------------------------------------
    // 執行階段：採用安全的變數萃取法
    // ---------------------------------------------------------
    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP);

    // 1. 跑第一段 (中間點)
    if (intermediatePos_mm != nullptr)
    {
        MoveResult res = processMove(*intermediatePos_mm);
        if (res.success)
        {
            std::vector<double> res_pos = res.targetPulse;
            double res_vel = res.vel;
            double res_acc = res.acc;
            double res_dec = res.dec;

            // 🌟 將 mode (BufferMode) 當作標籤傳進去，取代原本的全域設定
            LineMove(axes, res_pos, res_vel, res_acc, res_dec, mode);
        }
    }

    // 2. 跑第二段 (參考點)
    MoveResult resRef = processMove(refPos_mm);
    if (resRef.success)
    {
        std::vector<double> resRef_pos = resRef.targetPulse;
        double resRef_vel = resRef.vel;
        double resRef_acc = resRef.acc;
        double resRef_dec = resRef.dec;

        // 🌟 同樣交給標籤系統
        LineMove(axes, resRef_pos, resRef_vel, resRef_acc, resRef_dec, mode);
    }

    // =========================================================
    // 🌟 3. 極度重要：交接棒！
    // 經過上面的 processMove，simulatedStartPulse 已經更新到「最最終的參考點位置」了。
    // 把它寫回 lastQueuedPulse，讓下一行預讀的 G 碼有正確的起點！
    // =========================================================
    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        (*m_pContexts)[idx].lastQueuedPulse = simulatedStartPulse[i];
    }


}