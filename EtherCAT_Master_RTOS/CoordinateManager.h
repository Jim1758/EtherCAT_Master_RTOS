#pragma once
#include "NC_Types.h"
#include <vector>

class CoordinateManager {
public:
    CoordinateManager();

    // 狀態紀錄
    bool isAbsoluteMode = true; // G90 = true (絕對), G91 = false (增量)
    int currentWCSIndex = 0;    // 0代表G54, 1代表G55... (擴充到59)

    // 🌟 紀錄「NC 理論上的最後機械位置」，用來算 G91 增量偏移
    double commandedMCS[8] = { 0.0 };

    // ==========================================
    // 參數表格 (依照你的規格規劃)
    // ==========================================
    double extOffset[8] = { 0.0 };                   // EXT 全域偏移
    std::vector<std::vector<double>> m_WCSTable;   // 60組 (G54~G59, G154~G959)
    std::vector<std::vector<double>> m_ToolOffset; // 100組 (刀具補償)
    std::vector<std::vector<double>> m_WorkOffset; // 100組 (工件補償)

    // 切換座標系函式
    bool SetWCS(int gCode);

    // 🌟 核心座標轉換引擎 (NC WCS -> 馬達 MCS)
    void Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, const double* currentMCS, double* outputMCS);
};