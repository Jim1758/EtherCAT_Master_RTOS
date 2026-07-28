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
    // 🌟 即時座標資料區
    // ==========================================
    // 儲存 8 軸的真實機械位置 (未來由 EtherCAT 的 RxPDO/TxPDO 更新)
    double actualMCS[8] = { 0.0 };    // 🌟 新增：真實機械位置

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
    std::vector<std::vector<double>> m_WorkOffset;

    // ==========================================
    // 核心操作函式
    // ==========================================
    bool SetWCS(int gCode, NCManager* nc);
    void SyncMachinePosition(const double* actualMCS);
    void Transform_WCS_to_MCS(const double* targetWCS, const bool* hasAxis, double* outputMCS);

    // ==========================================
    // 🌟 新增：多檔案管理系統
    // ==========================================
  
    void LoadAllParameters();
    void SaveAllParameters();

    // 針對單一表格的讀寫 (如果你 HMI 只改了刀具，可以單獨呼叫 SaveToolOffset)
    void SaveWCSStatus();   // 🌟 改名：讓函式名稱更明確
    void SaveExtOffset();
    void SaveToolOffset();
    void SaveWorkOffset();
    void SaveWCSTable();
    int GetCurrentWCSGCode() const;

  
    // 🌟 新增：G92 相關 API
    void ApplyG92(const bool* axisProgrammed, const double* targetPos);

    void GetActualMCS(double* outMCS) const; // 取出當前的真實機械座標
   
private:
    // 底層輔助函式：負責讀寫 8 軸二維陣列，並確保原子寫入防護
    void SaveTableToFile(const std::string& filename, const std::vector<std::vector<double>>& table);
    void LoadTableFromFile(const std::string& filename, std::vector<std::vector<double>>& table, int maxRows);
};