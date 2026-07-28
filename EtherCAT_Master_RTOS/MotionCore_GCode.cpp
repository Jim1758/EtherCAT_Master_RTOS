#include "MotionCore.h"
#include <algorithm> // 為了使用 std::min

// ⚠️ 注意 1：.cpp 的實作中，不能再寫預設值 "= BufferMode::ABORTING"
// ⚠️ 注意 2：參數名稱必須完整寫出 (axes, targetPos, mode)
void MotionCore::G00_Move(const std::vector<int>& axes, const std::vector<double>& targetPos, BufferMode mode)
{
    // 防呆檢查
    if (m_pContexts == nullptr || axes.empty()) return;

    // 1. 計算該次移動的目標速度
    // 為了安全，取所有參與軸中「最慢的 G00 速度」作為本次群組的最高速
    double groupG00Vel = 999999999.0;
    for (size_t i = 0; i < axes.size(); ++i)
    {
        int idx = axes[i];
        groupG00Vel = std::min(groupG00Vel, (*m_pContexts)[idx].G00_PPS);
    }

    // 2. 暫時切換為精確停止模式 (G00 到點通常要煞停)
    PathMode prevMode = GetGroupPathMode();
    SetGroupPathMode(PathMode::EXACT_STOP);

    // 3. 呼叫底層推入佇列 (給定專用的高加速度)
    // 這裡的 500000.0 是加速度/減速度的範例值，可依機台實際狀況調整
    double rapidAcc = 500000.0;
    LineMove(axes, targetPos, groupG00Vel, rapidAcc, rapidAcc, mode);

    // 4. 還原原本的路徑模式
    SetGroupPathMode(prevMode);
}