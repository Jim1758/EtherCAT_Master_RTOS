#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min
void MotionCore::G00_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    double groupAccTime = 0.0;
    double groupDecTime = 0.0;
    double maxTimeNeeded = 0.0; // 紀錄跑最久的那軸需要幾秒
    double sum_sq = 0.0;        // 3D Pulse 距離平方和

    std::vector<double> targetPos_Pulse;
    targetPos_Pulse.resize(axes.size());

    // =========================================================
    // 🌟 1. 將 mm 轉為 Pulse，並計算出「誰花的時間最長」
    // =========================================================
    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        // 1-1. 取加減速最大值 (最安全的煞車距離)
        groupAccTime = std::max(groupAccTime, axis.G00_acc_time);
        groupDecTime = std::max(groupDecTime, axis.G00_dec_time);

        // 1-2. mm 轉 Pulse (你原本的正確邏輯)
        double lead = axis.finalLead;
        if (lead < 1e-6) lead = 1.0;
        double pulsePerUnit = axis.resolution_PPR / lead;
        double targetPulse = targetPos_mm[i] * pulsePerUnit;

        // 1-3. 計算這根軸要走多少 Pulse
        double startPulse = axis.logicalCmdPos;
        double distancePulse = 0.0;

        // 【保留旋轉軸最短路徑判斷】
        if (axis.axisType == AxisType::ROTARY && axis.useShortestPath) {
            targetPulse = CalculateShortestTarget(startPulse, targetPulse, axis.rotaryModulo);
        }

        targetPos_Pulse[i] = targetPulse; // 存入給 LineMove 用的陣列
        distancePulse = std::abs(targetPulse - startPulse);
        sum_sq += (distancePulse * distancePulse);

        // 1-4. 🌟 解決速度問題的核心：算出這根軸如果全速跑，要花幾秒？
        if (axis.G00_PPS > 1.0) {
            double timeNeeded = distancePulse / axis.G00_PPS;
            maxTimeNeeded = std::max(maxTimeNeeded, timeNeeded); // 抓出拖慢全隊的「瓶頸時間」
        }
    }

    // 防呆保護
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 2. 算出最終群組速度 (PPS)
    // =========================================================
    double totalDist_Pulse = std::sqrt(sum_sq);
    double groupG00Vel_PPS = 1000.0; // 預設底速

    // 將總 Pulse 距離 / 瓶頸時間 = 完美的群組 PPS 速度
    if (maxTimeNeeded > 0.0001) {
        groupG00Vel_PPS = totalDist_Pulse / maxTimeNeeded;
    }

    // =========================================================
    // 🌟 3. 丟給 LineMove
    // =========================================================
    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP);

    // 完美傳入 Pulse 陣列與計算好的 PPS 速度
    LineMove(axes, targetPos_Pulse, groupG00Vel_PPS, groupAccTime, groupDecTime, BufferMode::ABORTING);

    SetGroupPathMode(prevMode);
}
/*
void MotionCore::G00_Move(const std::vector<int>& axes, const std::vector<double>& targetPos_mm, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty() || axes.size() != targetPos_mm.size()) return;

    // =========================================================
    // 🌟 1. 計算該次移動的群組速度與加減速時間 (木桶效應)
    // =========================================================
    double groupG00Vel = 999999999.0;
    double groupAccTime = 0.0; // 初始設為最小
    double groupDecTime = 0.0; // 初始設為最小

    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        AxisContext& axis = (*m_pContexts)[idx];

        // 速度取「最小」：牽就跑得最慢的軸
        groupG00Vel = std::min(groupG00Vel, axis.G00_PPS);

        // 時間取「最大」：牽就加速最慢、需要最長煞車距離的軸
        groupAccTime = std::max(groupAccTime, axis.G00_acc_time);
        groupDecTime = std::max(groupDecTime, axis.G00_dec_time);
    }

    // 🛡️ 防呆保護：如果沒有設定參數，給予一個基本安全值 (0.2秒)
    if (groupAccTime < 0.001) groupAccTime = 0.2;
    if (groupDecTime < 0.001) groupDecTime = 0.2;

    // =========================================================
    // 🌟 2. [轉換引擎]：將 mm 轉為 Pulse
    // =========================================================
    std::vector<double> targetPos_Pulse;
    for (size_t i = 0; i < axes.size(); ++i) {
        int idx = axes[i];
        double lead = (*m_pContexts)[idx].finalLead;
        if (lead < 1e-6) lead = 1.0; // 防呆

        double pulsePerUnit = (*m_pContexts)[idx].resolution_PPR / lead;
        targetPos_Pulse.push_back(targetPos_mm[i] * pulsePerUnit);
    }

    // =========================================================
    // 🌟 3. 執行移動
    // =========================================================
    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP); // G00 一定是精準定位

    // 將算好的群組安全時間傳給 LineMove
    LineMove(axes, targetPos_Pulse, groupG00Vel, groupAccTime, groupDecTime, BufferMode::ABORTING); // ⚠️ 注意 mode 不要寫死 ABORTING，聽使用者的

    SetGroupPathMode(prevMode);
}*/