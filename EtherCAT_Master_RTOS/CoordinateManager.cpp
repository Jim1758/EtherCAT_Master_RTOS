#include "CoordinateManager.h"
#include "NCManager.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include "GlobalConfig.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
CoordinateManager::CoordinateManager()
{
    // 初始化表格大小 (8軸)
    m_WCSTable.resize(60, std::vector<double>(8, 0.0));
    m_ToolOffset.resize(100, std::vector<double>(8, 0.0));
    m_WorkOffset.resize(100, std::vector<double>(8, 0.0));

    // 2. 開機自動載入所有檔案
  
    LoadAllParameters();


  
}

bool CoordinateManager::SetWCS(int gCode, NCManager* nc)
{
    int calculatedIndex = -1;

    // 處理 G54~G59
    if (gCode >= 54 && gCode <= 59) {
        calculatedIndex = gCode - 54;
    }
    // 處理 G154~G959
    else if (gCode >= 154 && gCode <= 959) {
        int hundred = gCode / 100;
        int tail = gCode % 100;
        if (tail >= 54 && tail <= 59) {
            calculatedIndex = (hundred - 1) * 6 + (tail - 54) + 6;
        }
    }

    // 如果計算成功
    if (calculatedIndex != -1 && nc != nullptr) {
        currentWCSIndex = calculatedIndex;

        // 🌟 使用你原本的呼叫方式，直接設定 #14
        nc->MacroSys.SetVar('$', 14, (double)gCode);

        return true;
    }

    return false;
}

void CoordinateManager::SyncMachinePosition(const double* actualMCS) {
    // 將 EtherCAT 真實的機械座標同步到理論座標，確保 G91 接續移動的安全
    for (int i = 0; i < 8; i++) {
        commandedMCS[i] = actualMCS[i];
    }
}

void CoordinateManager::Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, double* outputMCS) {
    for (int i = 0; i < 8; i++) {
        // 如果該軸沒指令，就繼承目前的理論機械位置 (不移動)
        if (!hasAxis[i]) {
            outputMCS[i] = commandedMCS[i];
            continue;
        }

        // 計算該軸的總偏移量 (EXT + 目前選定的 G54~G59)
        double totalOffset = extOffset[i] + m_WCSTable[currentWCSIndex][i];

        if (isAbsoluteMode) {
            // G90 絕對模式：目標程式座標 + 總偏移量
            outputMCS[i] = targetWCS[i] + totalOffset;
        }
        else {
            // G91 增量模式：相對於「理論上的機械最後位置」繼續加值
            outputMCS[i] = commandedMCS[i] + targetWCS[i];
        }

        // 更新理論紀錄位置，供下一次計算使用
        commandedMCS[i] = outputMCS[i];
    }
}

// ==========================================
// 🌟 總管函式 (就是這裡遺失導致 LNK2019)
// ==========================================
void CoordinateManager::LoadAllParameters() {
    // 🌟 1. 檔名改為 WCS_STATUS.ini
    std::ifstream inStatus(GlobalConfig::GetInstance().NCDataDir + "WCS_STATUS.ini");
    if (inStatus.is_open()) {
        std::string line, key;
        while (std::getline(inStatus, line)) {
            std::stringstream ss(line);
            if (std::getline(ss, key, '=') && key == "LastWCSIndex") {
                ss >> currentWCSIndex;
                if (currentWCSIndex < 0 || currentWCSIndex >= 60) currentWCSIndex = 0;
            }
        }
        inStatus.close();
    }

    // 讀取 EXT_OFFSET
    std::ifstream inExt(GlobalConfig::GetInstance().NCDataDir + "EXT_OFFSET.txt");
    if (inExt.is_open()) {
        for (int i = 0; i < 8; i++) inExt >> extOffset[i];
        inExt.close();
    }

    // 讀取三大表格
    LoadTableFromFile("WCS_TABLE.txt", m_WCSTable, 60);
    LoadTableFromFile("TOOL_OFFSET.txt", m_ToolOffset, 100);
    LoadTableFromFile("WORK_OFFSET.txt", m_WorkOffset, 100);
}

void CoordinateManager::SaveAllParameters() {
    SaveWCSStatus();
    SaveExtOffset();
    SaveWCSTable();
    SaveToolOffset();
    SaveWorkOffset();
}

// ==========================================
// 🌟 單獨儲存函式 (RTX64 相容版)
// ==========================================
void CoordinateManager::SaveWCSStatus() { // 🌟 3. 函式改名
    std::string path = GlobalConfig::GetInstance().NCDataDir + "WCS_STATUS.ini"; // 🌟 檔名改為 WCS_STATUS.ini
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return;

    out << "LastWCSIndex=" << currentWCSIndex << "\n";
    out.close();
}

void CoordinateManager::SaveExtOffset() {
    std::string path = GlobalConfig::GetInstance().NCDataDir + "EXT_OFFSET.txt";
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return;

    out << std::fixed << std::setprecision(4);
    for (int i = 0; i < 8; i++) out << extOffset[i] << (i == 7 ? "" : "\t");
    out << "\n";
    out.close();
}

void CoordinateManager::SaveWCSTable() { SaveTableToFile("WCS_TABLE.txt", m_WCSTable); }
void CoordinateManager::SaveToolOffset() { SaveTableToFile("TOOL_OFFSET.txt", m_ToolOffset); }
void CoordinateManager::SaveWorkOffset() { SaveTableToFile("WORK_OFFSET.txt", m_WorkOffset); }

// ==========================================
// 🌟 共用底層：表格陣列讀寫引擎
// ==========================================
void CoordinateManager::SaveTableToFile(const std::string& filename, const std::vector<std::vector<double>>& table) {
    std::string path = GlobalConfig::GetInstance().NCDataDir + filename;
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
        // 若發生此問題，通常是因為 D 槽權限不足，或是 HMI 忘記建資料夾
        return;
    }

    out << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < table.size(); i++) {
        for (int j = 0; j < 8; j++) {
            out << table[i][j] << (j == 7 ? "" : "\t");
        }
        out << "\n";
    }
    out.close();
}

void CoordinateManager::LoadTableFromFile(const std::string& filename, std::vector<std::vector<double>>& table, int maxRows) {
    std::ifstream in(GlobalConfig::GetInstance().NCDataDir + filename);
    if (!in.is_open()) return; // 沒檔案就放空，維持 0.0

    std::string line;
    int row = 0;
    while (std::getline(in, line) && row < maxRows) {
        if (line.empty() || line[0] == ';') continue;
        std::stringstream ss(line);
        for (int i = 0; i < 8; i++) {
            ss >> table[row][i];
        }
        row++;
    }
    in.close();
}
int CoordinateManager::GetCurrentWCSGCode() const
{
    // 處理 G54 ~ G59 (Index 0 ~ 5)
    if (currentWCSIndex >= 0 && currentWCSIndex <= 5) {
        return currentWCSIndex + 54;
    }
    // 處理 G154 ~ G959 (Index 6 ~ 59)
    else if (currentWCSIndex >= 6 && currentWCSIndex <= 59) {
        int offset = currentWCSIndex - 6;
        int hundred = (offset / 6) + 1; // 算出百位數 (1 ~ 9)
        int tail = (offset % 6) + 54;   // 算出尾數 (54 ~ 59)
        return (hundred * 100) + tail;
    }

    return 54; // 預設防呆回傳 G54
}

// 🌟 更新機械座標 (未來由 EtherCAT 更新)
void CoordinateManager::UpdateActualMCS(const double* newMCS) {
    for (int i = 0; i < 8; i++) {
        actualMCS[i] = newMCS[i];
    }
}



// 🌟 1. 核心公式：算回最簡單的 絕對座標 = 機械座標 - EXT - 表格偏移
void CoordinateManager::GetActualWCS(double* outWCS) const {
    for (int i = 0; i < 8; i++) {
        double ext = extOffset[i];
        double wcsOffset = m_WCSTable[currentWCSIndex][i];

        outWCS[i] = actualMCS[i] - (ext + wcsOffset);
    }
}

// 🌟 2. 實作 ApplyG92 (直接覆寫當前表格！)
void CoordinateManager::ApplyG92(const bool* axisProgrammed, const double* targetPos) {
    for (int i = 0; i < 8; i++) {
        if (axisProgrammed[i]) {
            double ext = extOffset[i];

            // 公式：新 G54 表格偏移 = 機械座標 - EXT - 使用者要求的目標座標
            // 直接把算出來的結果塞進目前的 WCS 表格中 (例如 G54 就是 currentWCSIndex = 0)
            m_WCSTable[currentWCSIndex][i] = actualMCS[i] - ext - targetPos[i];

            DEBUG_PRINT("[Coordinate] G92 Overwrites WCS %d on Axis %d, New Offset: %f\n",
                currentWCSIndex, i, m_WCSTable[currentWCSIndex][i]);
        }
    }
}