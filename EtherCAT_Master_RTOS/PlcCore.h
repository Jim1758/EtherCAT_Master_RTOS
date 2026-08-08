#pragma once
#include "EtherCatTypes.h"
#include <vector>
#include <string>

// 數位 IO 對映項目
struct DigitalMapItem {
    int listIdx;          // 在 m_pIo 陣列中的索引
    int userOrder;        // 🌟 新增：使用者自訂的邏輯順序 (例如 1, 2, 3...)
    int plcStartIndex;    // PLC 虛擬記憶體起點 (例如 I0 或 O0)
    int bitCount;         // 映射點數
    std::string comment;  // 註解說明
};

// 類比 IO 對映項目
struct AnalogMapItem {
    int listIdx;          // 在 m_pAd 陣列中的索引
    int userOrder;        // 🌟 新增：使用者自訂的順序
    int plcDrStartIndex;  // 對應到 PLC 的 DR 暫存器起點 (例如 DR100)
    int channelCount;     // 通道數量
    std::string comment;
};

class PlcCore {
public:
    PlcCore();

    int m_marqueeLed_debug = 0;
    // 🌟 1. 綁定數位與類比的硬體清單
    void SetIoLists(std::vector<ENI_GenericIO>* pIoList, std::vector<ENI_AnalogModule>* pAdList);

    // 🌟 2. 自動掃描硬體並建立 IO 對映表 (啟動時呼叫)
    void AutoMapIO();

    // 🌟 3. 核心橋接 API (在 1ms 中斷裡呼叫)
    void FetchInputs();           // [重要新增] 將硬體 Rx 拷貝到影子記憶體
    void SyncPhysicalToVirtual(); // 實體影子記憶體 -> PLC (I點 / DR暫存器)
    void SyncVirtualToPhysical(); // PLC (O點) -> 實體影子記憶體
    void FlushOutputs();          // 實體影子記憶體 -> 硬體 Tx

    void Update_Debug();

    // 底層操作 API
    bool Get_I(int listIdx, int bitIdx);
    void Set_O(int listIdx, int bitIdx, bool val);
    bool Get_O(int listIdx, int bitIdx);
    void Clear_Module_O(int listIdx);

private:
    std::vector<ENI_GenericIO>* m_pIo = nullptr;
    std::vector<ENI_AnalogModule>* m_pAd = nullptr;

    std::vector<DigitalMapItem> m_inputMaps;
    std::vector<DigitalMapItem> m_outputMaps;
    std::vector<AnalogMapItem>  m_analogInputMaps;

    int m_marqueeLed = 0;


};