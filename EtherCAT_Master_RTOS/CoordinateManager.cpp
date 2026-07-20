#include "CoordinateManager.h"

CoordinateManager::CoordinateManager() {
    // 初始化表格大小 (8軸)
    m_WCSTable.resize(60, std::vector<double>(8, 0.0));
    m_ToolOffset.resize(100, std::vector<double>(8, 0.0));
    m_WorkOffset.resize(100, std::vector<double>(8, 0.0));

    // 🌟 為了待會的測試，我們在 G54 (Index 0) 預設塞入一個 X=50, Y=50 的偏移！
    // 這樣在測試時，你打 X10 Y10，馬達其實會跑到 X60 Y60。
    m_WCSTable[0][0] = 50.0; // X 軸
    m_WCSTable[0][1] = 50.0; // Y 軸
}

bool CoordinateManager::SetWCS(int gCode) {
    if (gCode >= 54 && gCode <= 59) {
        currentWCSIndex = gCode - 54;
        return true;
    }
    else if (gCode >= 154 && gCode <= 959) {
        // 數學推導：處理擴充座標系 (例如 G154 = index 6)
        int hundred = gCode / 100;
        int tail = gCode % 100;
        if (tail >= 54 && tail <= 59) {
            currentWCSIndex = (hundred - 1) * 6 + (tail - 54) + 6;
            return true;
        }
    }
    return false; // 不是座標切換的 G 碼
}

// 檔案：CoordinateManager.cpp

void CoordinateManager::Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, const double* currentMCS, double* outputMCS) {
    for (int i = 0; i < 8; i++) {
        // 如果該軸沒指令，就繼承目前的機械位置
        if (!hasAxis[i]) {
            outputMCS[i] = commandedMCS[i];
            continue;
        }

        double totalOffset = extOffset[i] + m_WCSTable[currentWCSIndex][i];

        if (isAbsoluteMode) {
            outputMCS[i] = targetWCS[i] + totalOffset;
        }
        else {
            // G91 增量模式：相對於「理論上的機械最後位置」繼續加值
            outputMCS[i] = commandedMCS[i] + targetWCS[i];
        }

        // 更新理論紀錄位置
        commandedMCS[i] = outputMCS[i];
    }
}