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

void MotionCore::G32_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.empty()) return;

    std::vector<double> simulatedStartPulse(axes.size());
    for (size_t i = 0; i < axes.size(); ++i) {
        simulatedStartPulse[i] = (*m_pContexts)[axes[i]].logicalCmdPos;
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

            double safe_acc = (axis.G32_acc_time > 0.01) ? axis.G32_acc_time : 0.2;
            double safe_dec = (axis.G32_dec_time > 0.01) ? axis.G32_dec_time : 0.2;
            groupAccTime = std::max<double>(groupAccTime, safe_acc);
            groupDecTime = std::max<double>(groupDecTime, safe_dec);

            double lead = (axis.finalLead < 1e-6) ? 1.0 : axis.finalLead;
            double targetPulse = target_mm[i] * (axis.resolution_PPR / lead);
            double startPulse = simulatedStartPulse[i];

            if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
                targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
            }

            target_Pulse[i] = targetPulse;
            double dist = std::abs(targetPulse - startPulse);
            sum_sq += (dist * dist);

            double safe_PPS = (axis.G32_PPS > 10.0) ? axis.G32_PPS : 50000.0;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, dist / safe_PPS);

            simulatedStartPulse[i] = targetPulse; // 更新起點給下一段
        }

        double totalDist = std::sqrt(sum_sq);
        double vel = (maxTimeNeeded > 0.0001) ? (totalDist / maxTimeNeeded) : 1000.0;

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
            // 🌟 你的神級改法：深拷貝與獨立傳遞，避開記憶體陷阱
            std::vector<double> res_pos = res.targetPulse;
            double res_vel = res.vel;
            double res_acc = res.acc;
            double res_dec = res.dec;
            LineMove(axes, res_pos, res_vel, res_acc, res_dec, mode);
        }
    }

    // 2. 跑第二段 (參考點)
    MoveResult resRef = processMove(refPos_mm);
    if (resRef.success)
    {
        // 🌟 同樣使用深拷貝保護
        std::vector<double> resRef_pos = resRef.targetPulse;
        double resRef_vel = resRef.vel;
        double resRef_acc = resRef.acc;
        double resRef_dec = resRef.dec;
        LineMove(axes, resRef_pos, resRef_vel, resRef_acc, resRef_dec, mode);
    }


}