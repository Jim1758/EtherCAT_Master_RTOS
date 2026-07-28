#pragma once
#include <vector>
#include <cmath>

// 🌟 1. 刪除 #include "MotionCore.h" 或類似的引入
// 🌟 2. 改用「前置宣告」來打破循環引用
struct AxisContext;
struct AxisCommand;
// ==========================================
// 單軸的補償參數結構
// ==========================================
struct AxisCompensation {
    // --- 1. 背隙補償 (Backlash) ---
    bool enableBacklash = false;
    double backlashAmount_mm = 0.010; // 背隙大小 (例如 10um)
    double backlashSpeed_mm_s = 1.0;  // 補償注入速度 (例如 1 mm/sec，確保平滑)

    // 背隙動態狀態 (內部計算用)
    int lastDir = 1;                  // 1=正轉, -1=反轉
    double currentBacklash_mm = 0.0;  // 當前已注入的背隙量
    double targetBacklash_mm = 0.0;   // 目標背隙量

    // --- 2. 節距補償 (Pitch Error) ---
    bool enablePitch = false;
    double pitchStartPos_mm = 0.0;    // 表格起點的機械座標 (例如 0.0)
    double pitchStep_mm = 10.0;       // 表格每一格的間距 (例如每 10mm 補一格)
    std::vector<double> pitchErrors;  // 誤差表格 (單位 mm)
};

// ==========================================
// 補償引擎類別
// ==========================================
class CompensationEngine {
public:
    CompensationEngine();

    void InitAxisCompensation(int axisIndex, bool enBacklash, double backlash_mm, bool enPitch, double step_mm);
    void SetPitchTable(int axisIndex, const std::vector<double>& errors);

    // 因為這裡只是「傳遞參考 (&)」，編譯器不需要知道結構的完整大小，所以前置宣告就足夠了！
    void ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt_sec);

private:
    AxisCompensation m_CompData[8];
};