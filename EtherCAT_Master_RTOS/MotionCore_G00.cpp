#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
#include "AlarmManager.h" 
#include <tuple> // 補上這一行
#include <vector>
#include <algorithm> // 確保也有這個，為了 std::max
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
void MotionCore::G00_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0;
    double sum_sq = 0.0;

    std::vector<double> targetPos_Pulse(axes.size());

    // 🌟 判斷大腦現在是不是在「連續預讀」狀態？
    // 如果倉庫裡有東西，或者馬達正在跑，大腦就必須使用「虛擬終點」！
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

        // =========================================================
        // 🌟 終極修復：決定正確的起點！
        // 如果正在預讀，起點 = 上一張訂單的終點；否則 = 馬達現在位置
        // =========================================================
        double startPulse = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;

        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse;

        // 使用正確的起點計算距離
        double distancePulse = std::abs(targetPulse - startPulse);
        sum_sq += (distancePulse * distancePulse);

        // 🌟 算完之後，把這次的終點存起來，給下一行預讀當作起點！
        axis.lastQueuedPulse = targetPulse;

        double currentAxisMaxPPS = axis.G00_PPS * G00_overrideRatio;
        if (axis.G00_PPS > 1.0) {
            double timeNeeded = distancePulse / currentAxisMaxPPS;
            maxTimeNeeded = std::max<double>(maxTimeNeeded, timeNeeded);
        }
    }

    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    double totalDist_Pulse = std::sqrt(sum_sq);
    double groupG00Vel_PPS = 0;

    if (maxTimeNeeded > 0.0001) {
        groupG00Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // =========================================================
    // 🌟 呼叫硬體 API (放回這裡就對了！)
    // =========================================================
    
    if (mode == BufferMode::ABORTING)
    {
        SetGroupPathMode(PathMode::EXACT_STOP);
    }
    else {
        SetGroupPathMode(PathMode::CONTINUOUS);
    }

    // 完美傳入 Pulse 陣列與計算好的 PPS 速度
    LineMove(axes, targetPos_Pulse, groupG00Vel_PPS, groupAccTime, groupDecTime, mode);

    //RtPrintf("G00>>> %d (LookAhead: %d)\n", mode, isLookAheadActive);
}





