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
    m_ToolRadius.resize(100, std::vector<double>(8, 0.0));
    m_WorkOffset.resize(100, std::vector<double>(8, 0.0));
    m_RefPoints.resize(100, std::vector<double>(8, 0.0));
   
    // 2. 開機自動載入所有檔案
  
    LoadAllParameters();

    // 🌟 開機預設為 G49 關閉刀長補正
    toolLengthMode = 49;
    currentHCode = 0;
  
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

    // 🌟 先複製一份 targetWCS，方便我們做旋轉加工
    double finalTargetWCS[8];
    for (int i = 0; i < 8; i++) finalTargetWCS[i] = targetWCS[i];

    // ==========================================================
    // 🌟 數學過濾器 0：G16 極座標 (Polar Coordinate)
    // 將操作員腦中的 (半徑, 角度) 轉換為標準 (X, Y) 直角座標
    // ==========================================================
    if (isPolarCoordinateActive && isAbsoluteMode)
    {
        int ax1 = 0, ax2 = 1; // 預設對應 G17 (XY 平面)
        if (activePlane == 18) { ax1 = 0; ax2 = 2; } // 若是 G18 (XZ 平面)
        if (activePlane == 19) { ax1 = 1; ax2 = 2; } // 若是 G19 (YZ 平面)

        // 取出操作員下達的「半徑」與「角度」
        double radius = finalTargetWCS[ax1];
        double angleRad = finalTargetWCS[ax2] * (3.14159265359 / 180.0);

        // 利用三角函數，將半徑與角度，換算回正常的 WCS 座標
        finalTargetWCS[ax1] = radius * std::cos(angleRad);
        finalTargetWCS[ax2] = radius * std::sin(angleRad);
    }


    // ==========================================================
      // 🌟 數學過濾器 1：G51 縮放 (Scaling)
      // 距離 = (目標 - 中心) * 倍率。新的位置 = 中心 + 距離
      // ==========================================================
    if (isScalingActive && isAbsoluteMode)
    {
        for (int i = 0; i < 8; i++) {
            if (hasAxis[i]) {
                finalTargetWCS[i] = scalingCenterWCS[i] + (finalTargetWCS[i] - scalingCenterWCS[i]) * scaleFactor;
            }
        }
    }

    // ==========================================================
    // 🌟 數學過濾器 2：G51.1 鏡像 (Mirror)
    // 距離 = (目標 - 中心)。新的位置 = 中心 - 距離 (反向)
    // ==========================================================
    if (isAbsoluteMode)
    {
        for (int i = 0; i < 8; i++) {
            if (isMirrorActive[i] && hasAxis[i]) {
                if (isAbsoluteMode) {
                    // G90：絕對座標對稱翻轉
                    finalTargetWCS[i] = mirrorCenterWCS[i] - (finalTargetWCS[i] - mirrorCenterWCS[i]);
                }
                else {
                    // G91：增量距離直接加負號！(走 +10 變成走 -10)
                    finalTargetWCS[i] = -finalTargetWCS[i];
                }
            }
        }
    }


    // ==========================================================
    // 🌟 數學處理：G68 2D 平面座標旋轉
    // ==========================================================
    if (isG68Active && isAbsoluteMode)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = std::sin(rad);
        double dx, dy;

        // 依照群組 2 的平面狀態，決定是哪兩軸要旋轉
        if (activePlane == 17) {
            // G17 (XY 平面)
            dx = targetWCS[0] - g68CenterWCS[0];
            dy = targetWCS[1] - g68CenterWCS[1];
            finalTargetWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            finalTargetWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            // G18 (XZ 平面)
            dx = targetWCS[0] - g68CenterWCS[0];
            double dz = targetWCS[2] - g68CenterWCS[2];
            finalTargetWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            finalTargetWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            // G19 (YZ 平面)
            dy = targetWCS[1] - g68CenterWCS[1];
            double dz = targetWCS[2] - g68CenterWCS[2];
            finalTargetWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            finalTargetWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }

    // ==========================================================
    // 🌟 【EDM 神級預判】：預先算出 C 軸 (第四軸) 的目標機械角度
    // 讓 X 和 Y 軸在計算時，可以吃到最新的 C 軸角度偏心量！
    // ==========================================================
    double targetCAngleMCS = commandedMCS[C_AXIS_INDEX]; // 預設為停留在原地

    if (hasAxis[C_AXIS_INDEX]) {
        double offsetC = extOffset[C_AXIS_INDEX]
            + m_WCSTable[currentWCSIndex][C_AXIS_INDEX]
            + GetActiveWorkOffset(C_AXIS_INDEX);

        if (isAbsoluteMode) {
            targetCAngleMCS = finalTargetWCS[C_AXIS_INDEX] + offsetC;
        }
        else {
            // G91 增量模式，C 軸目標為「當前位置 + 增量」
            targetCAngleMCS = commandedMCS[C_AXIS_INDEX] + targetWCS[C_AXIS_INDEX];
        }
    }

    // ==========================================================
    // 後續維持原本的偏移疊加運算 (注意要把 targetWCS 換成 finalTargetWCS)
    // ==========================================================
    for (int i = 0; i < 8; i++) {
        if (!hasAxis[i]) {
            outputMCS[i] = commandedMCS[i];
            continue;
        }

        // 🌟 這裡把預判好的 C 軸目標角度，傳給刀具補正引擎！
        double totalOffset = extOffset[i] + m_WCSTable[currentWCSIndex][i]
            + GetActiveToolOffset(i, targetCAngleMCS) // 👈 傳入
            + GetActiveWorkOffset(i);

        if (isAbsoluteMode) {
            outputMCS[i] = finalTargetWCS[i] + totalOffset; // 🌟 用旋轉後的 WCS 加偏移
        }
        else {
            outputMCS[i] = commandedMCS[i] + targetWCS[i]; // G91 增量通常不吃 G68 (視各廠牌規定)
        }
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
    LoadTableFromFile("TOOL_RADIUS.txt", m_ToolRadius, 100);
    LoadTableFromFile("WORK_OFFSET.txt", m_WorkOffset, 100);
    LoadTableFromFile("REFPOINTS.txt", m_RefPoints, 100);

}

void CoordinateManager::SaveAllParameters() {
    SaveWCSStatus();
    SaveExtOffset();
    SaveWCSTable();
    SaveToolOffset();
    SaveWorkOffset();
    SaveToolRadius();
    SaveRefPoints();
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
void CoordinateManager::SaveToolRadius() { SaveTableToFile("TOOL_RADIUS.txt", m_ToolRadius); }
void CoordinateManager::SaveRefPoints() { SaveTableToFile("TOOL_REFPOINTS.txt", m_RefPoints); }

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
    // 1. 複製一份真實的物理機械座標
    double tempMCS[8];
    for (int i = 0; i < 8; i++) tempMCS[i] = actualMCS[i];

    // ==========================================================
    // 🌟 2. 逆矩陣運算：把「歪掉的實體座標」轉回「方正的邏輯座標」
    // 在旋轉矩陣中，反矩陣 (Inverse) 剛好等於轉置矩陣 (Transpose)！
    // ==========================================================
    if (isWorkpieceRotationActive && currentWCode > 0 && currentWCode <= m_WorkOffset.size())
    {
        int idx = currentWCode - 1;

        // 取得角度轉成弧度
        double a = m_WorkOffset[idx][WO_ANGLE_XY_YAW] * (3.14159265359 / 180.0);
        double b = m_WorkOffset[idx][WO_ANGLE_XZ_PITCH] * (3.14159265359 / 180.0);
        double c = m_WorkOffset[idx][WO_ANGLE_YZ_ROLL] * (3.14159265359 / 180.0);

        double ca = std::cos(a), sa = std::sin(a);
        double cb = std::cos(b), sb = std::sin(b);
        double cc = std::cos(c), sc = std::sin(c);

        // 建立原本的正向旋轉矩陣 R
        double R[3][3];
        R[0][0] = ca * cb;
        R[0][1] = ca * sb * sc - sa * cc;
        R[0][2] = ca * sb * cc + sa * sc;
        R[1][0] = sa * cb;
        R[1][1] = sa * sb * sc + ca * cc;
        R[1][2] = sa * sb * cc - ca * sc;
        R[2][0] = -sb;
        R[2][1] = cb * sc;
        R[2][2] = cb * cc;

        // 計算距離旋轉圓心的位移 (Delta)
        double dx = actualMCS[0] - rotationCenterMCS[0];
        double dy = actualMCS[1] - rotationCenterMCS[1];
        double dz = actualMCS[2] - rotationCenterMCS[2];

        // 矩陣乘法，但注意這裡是 [col][row] 轉置相乘 (等於逆旋轉)！
        tempMCS[0] = rotationCenterMCS[0] + (R[0][0] * dx + R[1][0] * dy + R[2][0] * dz);
        tempMCS[1] = rotationCenterMCS[1] + (R[0][1] * dx + R[1][1] * dy + R[2][1] * dz);
        tempMCS[2] = rotationCenterMCS[2] + (R[0][2] * dx + R[1][2] * dy + R[2][2] * dz);
    }

    // 1. 扣除線性偏移量，得到「包含 G68 旋轉的 WCS」
    double tempWCS[8];
    for (int i = 0; i < 8; i++) {
        double ext = extOffset[i];
        double wcsOffset = m_WCSTable[currentWCSIndex][i];

        // 🌟 傳入 EtherCAT 讀回來的真實 C 軸物理角度
        double toolOffset = GetActiveToolOffset(i, actualMCS[C_AXIS_INDEX]);

        double workOffset = GetActiveWorkOffset(i);

        tempWCS[i] = tempMCS[i] - (ext + wcsOffset + toolOffset + workOffset);
    }

    // ==========================================================
    // 🌟 2. 顯示反轉：把 G68 的 2D 旋轉逆轉回來！
    // ==========================================================
    for (int i = 0; i < 8; i++) outWCS[i] = tempWCS[i];

    if (isG68Active)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = -std::sin(rad); // 🌟 逆矩陣，sin 變負的
        double dx, dy;

        if (activePlane == 17) {
            dx = tempWCS[0] - g68CenterWCS[0];
            dy = tempWCS[1] - g68CenterWCS[1];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            outWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            dx = tempWCS[0] - g68CenterWCS[0];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            dy = tempWCS[1] - g68CenterWCS[1];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }
}

// 🌟 1.5 核心公式：算回最簡單的 絕對座標 = 虛擬命令機械座標 - EXT - 表格偏移
void CoordinateManager::GetCommandedWCS(double* outWCS) const {
    // 1. 複製一份大腦的理論命令機械座標 (Commanded MCS)
    double tempMCS[8];
    for (int i = 0; i < 8; i++) tempMCS[i] = commandedMCS[i]; // 🌟 唯一差別：吃 commandedMCS

    // ==========================================================
    // 🌟 2. 逆矩陣運算：把「歪掉的實體座標」轉回「方正的邏輯座標」
    // 在旋轉矩陣中，反矩陣 (Inverse) 剛好等於轉置矩陣 (Transpose)！
    // ==========================================================
    if (isWorkpieceRotationActive && currentWCode > 0 && currentWCode <= m_WorkOffset.size())
    {
        int idx = currentWCode - 1;

        // 取得角度轉成弧度
        double a = m_WorkOffset[idx][WO_ANGLE_XY_YAW] * (3.14159265359 / 180.0);
        double b = m_WorkOffset[idx][WO_ANGLE_XZ_PITCH] * (3.14159265359 / 180.0);
        double c = m_WorkOffset[idx][WO_ANGLE_YZ_ROLL] * (3.14159265359 / 180.0);

        double ca = std::cos(a), sa = std::sin(a);
        double cb = std::cos(b), sb = std::sin(b);
        double cc = std::cos(c), sc = std::sin(c);

        // 建立原本的正向旋轉矩陣 R
        double R[3][3];
        R[0][0] = ca * cb;
        R[0][1] = ca * sb * sc - sa * cc;
        R[0][2] = ca * sb * cc + sa * sc;
        R[1][0] = sa * cb;
        R[1][1] = sa * sb * sc + ca * cc;
        R[1][2] = sa * sb * cc - ca * sc;
        R[2][0] = -sb;
        R[2][1] = cb * sc;
        R[2][2] = cb * cc;

        // 計算距離旋轉圓心的位移 (Delta)
        // ⚠️ 這裡的圓心還是用實體擷取的 rotationCenterMCS，這沒問題，因為 G168 是在靜止時啟動的
        double dx = tempMCS[0] - rotationCenterMCS[0];
        double dy = tempMCS[1] - rotationCenterMCS[1];
        double dz = tempMCS[2] - rotationCenterMCS[2];

        // 矩陣乘法，但注意這裡是 [col][row] 轉置相乘 (等於逆旋轉)！
        tempMCS[0] = rotationCenterMCS[0] + (R[0][0] * dx + R[1][0] * dy + R[2][0] * dz);
        tempMCS[1] = rotationCenterMCS[1] + (R[0][1] * dx + R[1][1] * dy + R[2][1] * dz);
        tempMCS[2] = rotationCenterMCS[2] + (R[0][2] * dx + R[1][2] * dy + R[2][2] * dz);
    }

    // 1. 扣除線性偏移量，得到「包含 G68 旋轉的 WCS」
    double tempWCS[8];
    for (int i = 0; i < 8; i++) {
        double ext = extOffset[i];
        double wcsOffset = m_WCSTable[currentWCSIndex][i];

        // 🌟 傳入大腦記錄的理論 C 軸角度
        double toolOffset = GetActiveToolOffset(i, commandedMCS[C_AXIS_INDEX]); // 🌟 唯一差別：吃 commandedMCS

        double workOffset = GetActiveWorkOffset(i);

        tempWCS[i] = tempMCS[i] - (ext + wcsOffset + toolOffset + workOffset);
    }

    // ==========================================================
    // 🌟 2. 顯示反轉：把 G68 的 2D 旋轉逆轉回來！
    // ==========================================================
    for (int i = 0; i < 8; i++) outWCS[i] = tempWCS[i];

    if (isG68Active)
    {
        double rad = g68Angle * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = -std::sin(rad); // 🌟 逆矩陣，sin 變負的
        double dx, dy;

        if (activePlane == 17) {
            dx = tempWCS[0] - g68CenterWCS[0];
            dy = tempWCS[1] - g68CenterWCS[1];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dy * s);
            outWCS[1] = g68CenterWCS[1] + (dx * s + dy * c);
        }
        else if (activePlane == 18) {
            dx = tempWCS[0] - g68CenterWCS[0];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[0] = g68CenterWCS[0] + (dx * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dx * s + dz * c);
        }
        else if (activePlane == 19) {
            dy = tempWCS[1] - g68CenterWCS[1];
            double dz = tempWCS[2] - g68CenterWCS[2];
            outWCS[1] = g68CenterWCS[1] + (dy * c - dz * s);
            outWCS[2] = g68CenterWCS[2] + (dy * s + dz * c);
        }
    }
}
// 🌟 2. 實作 ApplyG92 (直接覆寫當前表格！)
void CoordinateManager::ApplyG92(const bool* axisProgrammed, const double* targetPos) {

    // 1. 取得「當下」包含所有補正(刀長、旋轉等)的純粹命令工作座標
    double currentCmdWCS[8] = { 0.0 };

    // 🌟 呼叫剛剛寫好的神級函式！不吃實際位置！
    GetCommandedWCS(currentCmdWCS);

    for (int i = 0; i < 8; i++) {
        if (axisProgrammed[i]) {

            // 2. 算出差值 (你現在理應顯示的數字 - 你想變成的數字)
            double shift = currentCmdWCS[i] - targetPos[i];

            // 3. 將差值疊加到目前的 WCS 表格上
            m_WCSTable[currentWCSIndex][i] += shift;

            //DEBUG_PRINT("[Coordinate] G92 Shift on Axis %d: Shift %f (CmdWCS %f -> Target %f)\n",i, shift, currentCmdWCS[i], targetPos[i]);
        }
    }
}

void CoordinateManager::GetActualMCS(double* outMCS) const {
    for (int i = 0; i < 8; i++) {
        outMCS[i] = actualMCS[i];
    }
}

void  CoordinateManager::Set_G90G91(int value,NCManager* nc)//設定90絕對模式 91增量模式
{
    if (value == 90)
    {
        isAbsoluteMode = true;
        nc->MacroSys.SetVar('$', 3, 90);
       
    }
    if (value == 91)
    {
        isAbsoluteMode = false;
        nc->MacroSys.SetVar('$', 3, 91);
    }
}

// ==========================================
// 🌟 新增：刀長補正邏輯
// ==========================================
void CoordinateManager::SetToolLengthCompensation(int gCode, int hCode, NCManager* nc)
{
    // 處理 G49 或 H0 (取消補正)
    if (gCode == 49 || hCode == 0) {
        CancelToolLengthCompensation(nc);
        return;
    }

    if (gCode == 43 || gCode == 44) {
        // 🌟 防呆範圍修改：允許 hCode 介於 1 到 size 之間 (例如 1~100)
        if (hCode > 0 && hCode <= m_ToolOffset.size()) {
            toolLengthMode = gCode;
            currentHCode = hCode;

            if (nc) nc->MacroSys.SetVar('$', 8, (double)gCode);
        }
        else {
            // (選配) 可以呼叫你的警報系統：H 碼超出範圍！
        }
    }
}

void CoordinateManager::CancelToolLengthCompensation(NCManager* nc)
{
    toolLengthMode = 49; // 強制切換為 G49
    currentHCode = 0;
   

    // 更新系統巨集變數 (群組 8 的狀態改為 49)
    if (nc) nc->MacroSys.SetVar('$', 8, 49.0);
}

// 🌟 核心：算出「現在這個軸」要補償多少距離？
// 🌟 核心：算出「現在這個軸」要補償多少距離？(加入 EDM C 軸偏心旋轉)
double CoordinateManager::GetActiveToolOffset(int axisIndex, double cAngleMCS) const
{
    // 如果是 G49，或是 H0 (取消補正)，直接回傳 0.0
    if (toolLengthMode == 49 || currentHCode <= 0 || currentHCode > m_ToolOffset.size()) {
        return 0.0;
    }

    int arrayIndex = currentHCode - 1;

    // 取出原始表格中的補正數值
    double rawOffset = m_ToolOffset[arrayIndex][axisIndex];

    // =========================================================
     // 🌟 【EDM 偏心補償】：只有在「功能啟動 (isCAxisOffsetRotationEnabled == true)」
     // 且計算 X (0) 或 Y (1) 時，才套用 C 軸旋轉矩陣
     // =========================================================
    if (isCAxisOffsetRotationEnabled && (axisIndex == 0 || axisIndex == 1))
    {
        double offsetX = m_ToolOffset[arrayIndex][0];
        double offsetY = m_ToolOffset[arrayIndex][1];

        double rad = cAngleMCS * (3.14159265359 / 180.0);
        double c = std::cos(rad);
        double s = std::sin(rad);

        if (axisIndex == 0) {
            rawOffset = offsetX * c - offsetY * s; // 旋轉後的 X 偏心量
        }
        else if (axisIndex == 1) {
            rawOffset = offsetX * s + offsetY * c; // 旋轉後的 Y 偏心量
        }
    }

    // G43 為正向 (+)，G44 為負向 (-)
    return (toolLengthMode == 43) ? rawOffset : -rawOffset;
}
// ==========================================================
// 🌟 啟動工件旋轉 (G168)
// ==========================================================
void CoordinateManager::SetWorkpieceRotation(int wCode, const bool* hasAxis, const double* targetWCS, NCManager* nc)
{
    // 1. 防呆：確保 W 碼在陣列範圍內 (W1 對應 size 100 內)
    if (wCode < 1 || wCode > m_WorkOffset.size()) {
        RtPrintf(">>> [ALARM] G168 W%d is out of range!\n", wCode);
        return; // 範圍錯誤，不執行
    }

    currentWCode = wCode;
    int arrayIndex = wCode - 1; // 🌟 陣列從 0 開始，所以 W1 對應 [0]

    // 2. 決定旋轉圓心 (必須轉成絕對機械座標 MCS 讓底層使用)
    double convertedMCS[8] = { 0.0 };

    // 如果有下達 XYZ，先將指令的工作座標(WCS)轉成實體機械座標(MCS)
    Transform_WCS_to_MCS(targetWCS, hasAxis, convertedMCS);

    for (int i = 0; i < 3; i++)
    {
        if (hasAxis[i]) {
            // 有下 XYZ 指令 -> 使用轉換後的指令座標當圓心
            rotationCenterMCS[i] = convertedMCS[i];
        }
        else {
            // 沒下 XYZ 指令 -> 抓取機台「當下真實的機械座標」當圓心
            rotationCenterMCS[i] = actualMCS[i];
        }
    }

    // 3. 從 m_WorkOffset 表格抓出三個平面的旋轉角度
    double yaw_xy = m_WorkOffset[arrayIndex][WO_ANGLE_XY_YAW];
    double pitch_xz = m_WorkOffset[arrayIndex][WO_ANGLE_XZ_PITCH];
    double roll_yz = m_WorkOffset[arrayIndex][WO_ANGLE_YZ_ROLL];

    isWorkpieceRotationActive = true;

    // 4. 呼叫 MotionCore 底層的空間旋轉引擎！
    if (nc) {
        nc->GetMotion().SetCoordinateTransform
        (
            true,
            rotationCenterMCS[0], rotationCenterMCS[1], rotationCenterMCS[2],
            yaw_xy, pitch_xz, roll_yz
        );

        RtPrintf("[G168] Workpiece Rotation ON (W%d). Center:(%.3f, %.3f, %.3f)\n",
            wCode, rotationCenterMCS[0], rotationCenterMCS[1], rotationCenterMCS[2]);
    }
}

// ==========================================================
// 🌟 關閉工件旋轉 (G169)
// ==========================================================
void CoordinateManager::CancelWorkpieceRotation(NCManager* nc)
{
    isWorkpieceRotationActive = false;
    currentWCode = 0;

    if (nc) {
        // 傳入 false 關閉旋轉矩陣
        nc->GetMotion().SetCoordinateTransform(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        RtPrintf("[G169] Workpiece Rotation OFF.\n");
    }
}

// 🌟 核心：算出「現在這個軸」要因為 G168 額外平移多少距離？
double CoordinateManager::GetActiveWorkOffset(int axisIndex) const
{
    // 1. 如果 G168 沒開啟，或者是無效的 W 碼，就不平移
    if (!isWorkpieceRotationActive || currentWCode <= 0 || currentWCode > m_WorkOffset.size()) {
        return 0.0;
    }

    // =========================================================
    // 🌟 【神級修復防護罩】：隔離角度與位移
    // 陣列 0, 1, 2 分別是 X, Y, Z 的線性平移 (允許回傳)
    // 陣列 3, 4, 5 是 旋轉角度，絕對不能當作平移量回傳！
    // =========================================================
    if (axisIndex > 2) {
        return 0.0; // 只要大於 Z 軸 (也就是角度欄位)，全部強制回傳 0.0！
    }

    // 2. 只有 X, Y, Z (0, 1, 2) 才會走到這一步，回傳正確的平移量
    int arrayIndex = currentWCode - 1;
    return m_WorkOffset[arrayIndex][axisIndex];
}

void CoordinateManager::SetActivePlane(int gCode, NCManager* nc)
{
    if (gCode == 17 || gCode == 18 || gCode == 19) {
        activePlane = gCode;

        // 更新系統巨集變數 (群組 2)
        if (nc) {
            nc->MacroSys.SetVar('$', 2, (double)gCode);
        }
    }
}

// ==========================================================
// 🌟 啟動 G68 座標旋轉
// ==========================================================
void CoordinateManager::SetG68Rotation(const double* centerPos, const bool* hasAxis, double angle, NCManager* nc)
{
    isG68Active = true;
    g68Angle = angle;

    // 取得當前的 WCS (若 G68 沒給圓心參數，就以當前 WCS 為旋轉圓心)
    double currentWCS[8];
    GetActualWCS(currentWCS);

    for (int i = 0; i < 3; i++) {
        if (hasAxis[i]) {
            g68CenterWCS[i] = centerPos[i];
        }
        else {
            g68CenterWCS[i] = currentWCS[i];
        }
    }

    if (nc) nc->MacroSys.SetVar('$', 16, 68.0); // 更新群組 16 巨集
    RtPrintf("[G68] 2D Rotation ON. Plane:%d, Center:(%.3f, %.3f), Angle:%.3f\n",
        activePlane, g68CenterWCS[0], g68CenterWCS[1], g68Angle);
}

// ==========================================================
// 🌟 取消 G69 座標旋轉
// ==========================================================
void CoordinateManager::CancelG68Rotation(NCManager* nc)
{
    isG68Active = false;
    g68Angle = 0.0;

    if (nc) nc->MacroSys.SetVar('$', 16, 69.0);
    RtPrintf("[G69] 2D Rotation OFF.\n");
}

// ==========================================================
// 🌟 G51 啟動縮放 / G50 關閉縮放
// ==========================================================
void CoordinateManager::SetScaling(const double* centerPos, const bool* hasAxis, double factor, NCManager* nc) {
    isScalingActive = true;
    scaleFactor = factor;

    double currentWCS[8];
    GetActualWCS(currentWCS);

    for (int i = 0; i < 8; i++) {
        scalingCenterWCS[i] = hasAxis[i] ? centerPos[i] : currentWCS[i];
    }
    if (nc) nc->MacroSys.SetVar('$', 11, 51.0);
}

void CoordinateManager::CancelScaling(NCManager* nc) {
    isScalingActive = false;
    scaleFactor = 1.0;
    if (nc) nc->MacroSys.SetVar('$', 11, 50.0);
}

// ==========================================================
// 🌟 G151 啟動鏡像 / G150 關閉鏡像
// ==========================================================
void CoordinateManager::SetMirror(const double* mirrorPos, const bool* hasAxis, NCManager* nc) {
    double currentWCS[8];
    GetActualWCS(currentWCS);

    for (int i = 0; i < 8; i++) {
        if (hasAxis[i]) {
            isMirrorActive[i] = true;
            mirrorCenterWCS[i] = mirrorPos[i]; // 以指令指定的座標為鏡像對稱線
        }
    }
}

void CoordinateManager::CancelMirror(const bool* hasAxis, NCManager* nc) {
    // 🌟 1. 系統先假設你要「全部取消」 (cancelAll = true)
    bool cancelAll = true;

    // 🌟 2. 檢查你有沒有輸入特定的軸？
    for (int i = 0; i < 8; i++) {
        if (hasAxis[i]) {
            cancelAll = false; // 如果你有打 X 或 Y，就把「全部取消」關掉
        }
    }

    // 🌟 3. 執行關閉動作
    for (int i = 0; i < 8; i++) {
        // 如果是「全部取消」，或者「這個軸剛好有被點名」，就把它的鏡像關掉！
        if (cancelAll || hasAxis[i]) {
            isMirrorActive[i] = false;
        }
    }
}

// ==========================================================
// 🌟 G16 啟動極座標 / G15 關閉極座標
// ==========================================================
void CoordinateManager::SetPolarCoordinate(NCManager* nc) {
    isPolarCoordinateActive = true;
    if (nc) nc->MacroSys.SetVar('$', 17, 16.0); // 更新群組 17
    RtPrintf("[G16] Polar Coordinate System ON.\n");
}

void CoordinateManager::CancelPolarCoordinate(NCManager* nc) {
    isPolarCoordinateActive = false;
    if (nc) nc->MacroSys.SetVar('$', 17, 15.0);
    RtPrintf("[G15] Polar Coordinate System OFF.\n");
}


// 🌟 啟動 G41 / G42
void CoordinateManager::SetToolRadiusCompensation(int gCode, int dCode, NCManager* nc)
{
    if (gCode == 41 || gCode == 42) {
        if (dCode > 0 && dCode <= m_ToolOffset.size()) {
            toolRadiusMode = gCode;
            currentDCode = dCode;
            if (nc) nc->MacroSys.SetVar('$', 7, (double)gCode);
            RtPrintf("[G%d] Tool Radius Comp ON. D-Code: %d\n", gCode, dCode);
        }
        else {
            RtPrintf(">>> [ALARM] G%d D%d is out of range!\n", gCode, dCode);
        }
    }
}

// 🌟 取消 G40
void CoordinateManager::CancelToolRadiusCompensation(NCManager* nc)
{
    toolRadiusMode = 40;
     currentDCode = 0; // 通常 D 碼保留，只改狀態
    if (nc) nc->MacroSys.SetVar('$', 7, 40.0);
    RtPrintf("[G40] Tool Radius Comp OFF.\n");
}

// 🌟 取得當前刀具的「半徑」值
double CoordinateManager::GetActiveToolRadius() const
{
    if (toolRadiusMode == 40 || currentDCode <= 0 || currentDCode > m_ToolOffset.size()) {
        return 0.0;
    }

    int arrayIndex = currentDCode - 1;

    // ⚠️ 這裡要依照你們的刀具表定義！
    // 假設 X=0, Y=1, Z=2。 半徑可能存在 index 3 (第四個欄位)
    const int RADIUS_INDEX = 3;

    return  m_ToolRadius[arrayIndex][RADIUS_INDEX];
}
bool CoordinateManager::GetRefPoint(int pCode, double* outPos) const {
    int index = pCode - 1; // P1 對應 index 0
    if (index < 0 || index >= m_RefPoints.size()) return false;

    for (int i = 0; i < 8; i++) outPos[i] = m_RefPoints[index][i];
    return true;
}