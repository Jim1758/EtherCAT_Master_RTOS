#pragma once
#include "EtherCatTypes.h"
#include <vector>

// 🌟 新增：IO 對映表結構
struct IOMapping {
    int moduleIdx;     // 物理模組站號 (例如 0 代表第一站)
    int plcStartIndex; // PLC 陣列的起始點 (例如 10 代表從 I10 或 O10 開始)
    int bitCount;      // 該模組要對映幾個點 (例如 8 或 16)
};

class PlcCore {
public:
    PlcCore();

    void Link(std::vector<ENI_GenericIO>* pList);
    void SetIoList(std::vector<ENI_GenericIO>* pList) { m_pIo = pList; }

    // ===============================================
    // 🌟 1. 對映表設定 API (啟動時設定)
    // ===============================================
    void AddInputMapping(int moduleIdx, int plcStartIndex, int bitCount);
    void AddOutputMapping(int moduleIdx, int plcStartIndex, int bitCount);

    // ===============================================
    // 🌟 2. 核心橋接 API (在 1ms 中斷裡呼叫)
    // ===============================================
    void SyncPhysicalToVirtual(); // 將 EtherCAT 實體 Input 寫入 PLC I 點
    void SyncVirtualToPhysical(); // 將 PLC O 點寫入 EtherCAT 實體 Output

    void Update_Debug();

    // 底層控制介面
    bool Get_I(int moduleIdx, int bitIdx);
    void Set_O(int moduleIdx, int bitIdx, bool val);
    bool Get_O(int moduleIdx, int bitIdx);
    void Clear_Module_O(int moduleIdx);
    void FlushOutputs();

private:
    std::vector<ENI_GenericIO>* m_pIo = nullptr;

    // 🌟 新增：存放對映規則的陣列
    std::vector<IOMapping> m_inputMaps;
    std::vector<IOMapping> m_outputMaps;

    int m_loopCount = 0;
    int m_marqueeLed = 0;
};