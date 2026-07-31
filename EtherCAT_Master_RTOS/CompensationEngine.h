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
    // 🌟 將單一參數改成雙向獨立參數
    double backlashAmount_Pos_mm = 0.0; // 正向補償量
    double backlashAmount_Neg_mm = 0.0; // 負向補償量
    double backlashSpeed_mm_s = 2.0;    // 漸變速度 (可視需求調整)
    // 背隙動態狀態 (內部計算用)
    int lastDir = 1;                  // 1=正轉, -1=反轉
    double currentBacklash_mm = 0.0;  // 當前已注入的背隙量
    double targetBacklash_mm = 0.0;   // 目標背隙量

    // --- 2. 節距補償 (Pitch Error) ---
    bool enablePitch = false;
    double pitchStartPos_mm = 0.0;    // 表格起點的機械座標 (例如 0.0)
    double pitchStep_mm = 10.0;       // 表格每一格的間距 (例如每 10mm 補一格)
    double currentPitch_mm = 0.0;  // 目前已注入的節距補償量
    double targetPitch_mm = 0.0;   // 目標節距補償量
    double pitchSpeed_mm_s = 2.0; // 節距補償的平滑速度
    std::vector<double> pitchErrors_Pos; // 正向誤差表
    std::vector<double> pitchErrors_Neg; // 負向誤差表
};

// ==========================================
// 補償引擎類別
// ==========================================
class CompensationEngine {
public:
    CompensationEngine();

    // 🌟 初始化介面更新：加入 backlashSpeed 與 pitchSpeed
    //void InitAxisCompensation(int axisIndex, bool enBacklash, double b_Pos, double b_Neg, double b_Speed, bool enPitch, double step_mm, double p_Speed);
    void InitAxisCompensation(int axisIndex, bool enBacklash, double b_Pos, double b_Neg, double b_Speed, bool enPitch, double startPos, double step_mm, double p_Speed);
    // 🌟 設定雙向節距表
    void SetPitchTables(int axisIndex, const std::vector<double>& posErrors, const std::vector<double>& negErrors);

    // 因為這裡只是「傳遞參考 (&)」，編譯器不需要知道結構的完整大小，所以前置宣告就足夠了！
    void ApplyCompensation(int axisIndex, AxisContext& axis, AxisCommand& cmd, double dt_sec);

    void SetPitchTablePos(int axisIndex, const std::vector<double>& errors);
    void SetPitchTableNeg(int axisIndex, const std::vector<double>& errors);

private:
    AxisCompensation m_CompData[8];
};