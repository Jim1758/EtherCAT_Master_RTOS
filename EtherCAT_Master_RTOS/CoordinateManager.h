#pragma once
#include "NC_Types.h"
#include <vector>
#include <string>

class NCManager;

class CoordinateManager {
public:
    CoordinateManager();

    // ==========================================
    // 狀態紀錄
    // ==========================================
    bool isAbsoluteMode = true;
    int currentWCSIndex = 0;
    double commandedMCS[8] = { 0.0 };

    // ==========================================
    // 🌟 定義 C 軸的索引位置 (通常第四軸為 index 3)
    // ==========================================
    const int C_AXIS_INDEX = 3;
    // ==========================================
    // 🌟 C 軸電極偏心旋轉補償開關 (預設開啟)
    // ==========================================
    bool isCAxisOffsetRotationEnabled = true; // 預设 true (開啟)
    // ==========================================
    // 🌟 平面選擇狀態 (群組 2)
    // ==========================================
    int activePlane = 17; // 預設 G17 (XY 平面)

    // 設定平面的 API
    void SetActivePlane(int gCode, NCManager* nc);

    // ==========================================
    // 🌟 即時座標資料區
    // ==========================================
    // 儲存 8 軸的真實機械位置 (未來由 EtherCAT 的 RxPDO/TxPDO 更新)
    double actualMCS[8] = { 0.0 };    // 🌟 新增：真實機械位置

    // 🌟 新增：刀長補正狀態 (G43/G44/G49, 群組 8)
    int toolLengthMode = 49; // 預設 G49 關閉
    int currentHCode = 0;    // 預設 H 碼為 0


    // ==========================================
    // 🌟 定義 m_WorkOffset 陣列的 8 個欄位名稱
    // ==========================================
    enum WorkOffsetField {
        WO_OFFSET_X = 0,       // 平移 X
        WO_OFFSET_Y = 1,       // 平移 Y
        WO_OFFSET_Z = 2,       // 平移 Z
        WO_ANGLE_XY_YAW = 3,   // XY 平面旋轉角度 (繞 Z 軸旋轉 / Yaw)
        WO_ANGLE_XZ_PITCH = 4, // XZ 平面旋轉角度 (繞 Y 軸旋轉 / Pitch)
        WO_ANGLE_YZ_ROLL = 5,  // YZ 平面旋轉角度 (繞 X 軸旋轉 / Roll)
        WO_RESERVED_6 = 6,     // 保留備用
        WO_RESERVED_7 = 7      // 保留備用
    };

    // ==========================================
    // 🌟 G168 工件旋轉狀態紀錄
    // ==========================================
    bool isWorkpieceRotationActive = false;
    int currentWCode = 0;                          // 當前使用的 W 碼
    double rotationCenterMCS[3] = { 0.0, 0.0, 0.0 }; // 真實旋轉圓心 (機械座標)




    // ==========================================
    // 🌟 G68 座標旋轉狀態 (群組 16)
    // ==========================================
    bool isG68Active = false;
    double g68CenterWCS[3] = { 0.0, 0.0, 0.0 }; // 旋轉圓心 (WCS 邏輯座標)
    double g68Angle = 0.0;                      // 旋轉角度 (度)


    // 控制 API
    void SetG68Rotation(const double* centerPos, const bool* hasAxis, double angle, NCManager* nc);
    void CancelG68Rotation(NCManager* nc);

    // ==========================================
    // 🌟 G50 / G51 縮放比例 (群組 11)
    // ==========================================
    bool isScalingActive = false;
    double scalingCenterWCS[8] = { 0.0 }; // 縮放中心
    double scaleFactor = 1.0;             // 縮放倍率 (P)

    // ==========================================
    // 🌟 G150 / G151 鏡像 (通常歸類在群組 22 或獨立)
    // ==========================================
    bool isMirrorActive[8] = { false };     // 紀錄哪幾個軸開啟了鏡像
    double mirrorCenterWCS[8] = { 0.0 };    // 鏡像對稱中心

    bool IsMirrorActive(int axisIndex) const {
        if (axisIndex >= 0 && axisIndex < 8) return isMirrorActive[axisIndex];
        return false;
    }
    // ==========================================
    // 🌟 G15 / G16 極座標 (群組 17)
    // ==========================================
    bool isPolarCoordinateActive = false;

    void SetPolarCoordinate(NCManager* nc);
    void CancelPolarCoordinate(NCManager* nc);

    // ==========================================
    // 🌟 G40 / G41 / G42 刀徑補償 (群組 7)
    // ==========================================
    int toolRadiusMode = 40; // 預設 G40 (取消補償)
    int currentDCode = 0;    // 當前使用的 D 碼

    void SetToolRadiusCompensation(int gCode, int dCode, NCManager* nc);
    void CancelToolRadiusCompensation(NCManager* nc);
    double GetActiveToolRadius() const; // 取得當前刀具半徑

    // API
    void SetScaling(const double* centerPos, const bool* hasAxis, double factor, NCManager* nc);
    void CancelScaling(NCManager* nc);
    void SetMirror(const double* mirrorPos, const bool* hasAxis, NCManager* nc);
    void CancelMirror(const bool* hasAxis, NCManager* nc);

    // ==========================================
    // 🌟 新增：座標計算 API
    // ==========================================
    // 1. 把外部 (EtherCAT) 讀到的新座標更新進來
    void UpdateActualMCS(const double* newMCS);

    // 2. 獲取當前真實的工件座標 (WCS)
    void GetActualWCS(double* outWCS) const;



    // ==========================================
    // 參數表格 (支援 8 軸)
    // ==========================================
    double extOffset[8] = { 0.0 };
    std::vector<std::vector<double>> m_WCSTable;
    std::vector<std::vector<double>> m_ToolOffset;
    std::vector<std::vector<double>> m_ToolRadius;
    std::vector<std::vector<double>> m_WorkOffset;
    // 儲存 100 組參考點 (P1~P100)，每組 8 軸
    std::vector<std::vector<double>> m_RefPoints;


    // ==========================================
    // 核心操作函式
    // ==========================================
    bool SetWCS(int gCode, NCManager* nc);
    void SyncMachinePosition(const double* actualMCS);
    void Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, double* outputMCS);


    // 🌟 新增：刀長補正控制 API
    void SetToolLengthCompensation(int gCode, int hCode, NCManager* nc);
    void CancelToolLengthCompensation(NCManager* nc); // 供 Reset 時呼叫
    double GetActiveToolOffset(int axisIndex, double cAngleMCS = 0.0) const;
    // 🌟 新增：取得目前作用中的工件平移偏移量 (G168 W 碼)
    double GetActiveWorkOffset(int axisIndex) const;
    // ==========================================
    // 🌟 新增：多檔案管理系統
    // ==========================================
  
    void LoadAllParameters();
    void SaveAllParameters();

    // 針對單一表格的讀寫 (如果你 HMI 只改了刀具，可以單獨呼叫 SaveToolOffset)
    void SaveWCSStatus();   // 🌟 改名：讓函式名稱更明確
    void SaveExtOffset();
    void SaveToolOffset();
    void SaveToolRadius();
    void SaveRefPoints();
    void SaveWorkOffset();
 
    void SaveWCSTable();
    int GetCurrentWCSGCode() const;

  
    // 🌟 新增：G92 相關 API
    void ApplyG92(const bool* axisProgrammed, const double* targetPos);

    void GetActualMCS(double* outMCS) const; // 取出當前的真實機械座標
     // 🌟 新增：獲取當前純粹的數學命令座標 (Commanded WCS) - 供 G92 等內部數學計算用
    void GetCommandedWCS(double* outWCS) const;

    void Set_G90G91(int value, NCManager* nc);//設定90絕對模式 91增量模式
   
    // ==========================================
    // 🌟 旋轉控制 API
    // ==========================================
    void SetWorkpieceRotation(int wCode, const bool* hasAxis, const double* targetWCS, NCManager* nc);
    void CancelWorkpieceRotation(NCManager* nc);
   

    bool GetRefPoint(int pCode, double* outPos) const;
private:
    // 底層輔助函式：負責讀寫 8 軸二維陣列，並確保原子寫入防護
    void SaveTableToFile(const std::string& filename, const std::vector<std::vector<double>>& table);
    void LoadTableFromFile(const std::string& filename, std::vector<std::vector<double>>& table, int maxRows);
};