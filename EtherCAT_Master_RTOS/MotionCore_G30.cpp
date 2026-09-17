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

void MotionCore::G30_Move(const std::vector<int>& axes, const std::vector<double>& refPos_mm, const std::vector<double>* intermediatePos_mm, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.empty()) return;

    // =========================================================
    // 🌟 1. 一視同仁地判斷預讀狀態
    // =========================================================
    bool isLookAheadActive = (!m_Group.cmdQueue.empty() || !IsGroupDone());

    std::vector<double> simulatedStartPulse(axes.size());
    for (size_t i = 0; i < axes.size(); ++i) {
        AxisContext& axis = (*m_pContexts)[axes[i]];

        // 初始起點：改吃虛擬終點！
        simulatedStartPulse[i] = isLookAheadActive ? axis.lastQueuedPulse : axis.logicalCmdPos;
    }

    auto processMove = [&](const std::vector<double>& target_mm) -> MoveResult {
        MoveResult result;
        result.success = false;
        result.vel = 0.0;
        result.acc = 0.2; // 安全值
        result.dec = 0.2; // 安全值

        if (target_mm.size() != axes.size()) return result;

        double groupAccTime = 0.0, groupDecTime = 0.0, maxTimeNeeded = 0.0;

        // 🌟 對齊 G00/G28 架構：準備 Pulse 與 mm 雙軌計算
        double sum_sq_pulse = 0.0;
        double sum_sq_mm = 0.0;

        std::vector<double> target_Pulse(axes.size());

        for (size_t i = 0; i < axes.size(); ++i) {
            int idx = axes[i];
            AxisContext& axis = (*m_pContexts)[idx];

            if (!axis.isExist) return result;

            double safe_acc = (axis.G30_acc_time > 0.01) ? axis.G30_acc_time : 0.2;
            double safe_dec = (axis.G30_dec_time > 0.01) ? axis.G30_dec_time : 0.2;
            groupAccTime = std::max<double>(groupAccTime, safe_acc);
            groupDecTime = std::max<double>(groupDecTime, safe_dec);

            const bool rotaryShortestPath =
                axis.axisType == AxisType::ROTARY && axis.useShortestPath;
            double pulsePerUnit = 0.0;
            if (!TryGetMotionPulsePerUnit(axis.resolution_PPR, axis.finalLead,
                rotaryShortestPath, pulsePerUnit))
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, m_pendingSourcePC, idx);
                return result;
            }

            double targetPulse = target_mm[i] * pulsePerUnit;
            double startPulse = simulatedStartPulse[i];

            if (!TryResolveMotionTargetPulse(startPulse, targetPulse, pulsePerUnit,
                rotaryShortestPath, axis.rotaryModulo, targetPulse))
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID, m_pendingSourcePC, idx);
                return result;
            }

            target_Pulse[i] = targetPulse;

            // 🌟 幾何距離雙計算
            double distPulse = std::abs(targetPulse - startPulse);
            double dist_mm = distPulse / pulsePerUnit;

            sum_sq_pulse += (distPulse * distPulse);
            sum_sq_mm += (dist_mm * dist_mm);

            // 🌟 【神級修復】防止除以零的防呆寫法 (讀取 G30 專屬極速)
            double safe_PPS = axis.G30_PPS;
            if (safe_PPS > 1.0) {
                maxTimeNeeded = std::max<double>(maxTimeNeeded, distPulse / safe_PPS);
            }

            // 把算完的終點存起來，當作下一段(如果有)的起點
            // Keep the simulated tail unchanged until every axis is valid.
        }

        double totalDist_Pulse = std::sqrt(sum_sq_pulse);

        // 用最慢軸的時間去牽制全部，計算群組虛擬速度
        double vel = (maxTimeNeeded > 0.0001) ? (totalDist_Pulse / maxTimeNeeded) : 0;

        simulatedStartPulse = target_Pulse;
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
    // Validate both converted legs before either can enter the motion queue.
    MoveResult intermediateResult{};
    if (intermediatePos_mm != nullptr)
    {
        intermediateResult = processMove(*intermediatePos_mm);
        if (!intermediateResult.success) return;
    }
    MoveResult referenceResult = processMove(refPos_mm);
    if (!referenceResult.success) return;

    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP);

    // 🌟 宣告一個游標模式，防止 ABORTING 誤刪中間點
    BufferMode currentMode = mode;

    // 1. 跑第一段 (中間點)
    if (intermediatePos_mm != nullptr)
    {
        const MoveResult& res = intermediateResult;
        if (res.success)
        {
            LineMove(axes, res.targetPulse, res.vel, res.acc, res.dec, currentMode);

            // 🌟 【神級修復】只要第一段出車了，後續強制轉為「排隊模式 (BUFFERED)」
            currentMode = BufferMode::BUFFERED;
        }
    }

    // 2. 跑第二段 (參考點)
    const MoveResult& resRef = referenceResult;
    if (resRef.success)
    {
        // 這裡的 currentMode 可能已經因為上面的 if 變成了 BUFFERED
        LineMove(axes, resRef.targetPulse, resRef.vel, resRef.acc, resRef.dec, currentMode);
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