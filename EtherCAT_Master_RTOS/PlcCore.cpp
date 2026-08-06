#include "PlcCore.h"
#include <cstring> // for memset
#include "PLCManager.h"



PlcCore::PlcCore()
{
    m_loopCount = 0;
    m_marqueeLed = 0;

  
}

void PlcCore::Link(std::vector<ENI_GenericIO>* pList)
{
    
    m_pIo = pList;
}

// ============================================================================
//  底層封裝區 (直接操作指標，不用在 Update 裡算位址)
// ============================================================================

bool PlcCore::Get_I(int moduleIdx, int bitIdx)
{
    // 1. 防呆
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return false;
    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];

    // [修改點] 檢查 inBuffer 是否有效 (而不是只檢查 pInputLoc)
    if (mod.inBuffer.empty()) return false;

    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;

    if (bytePos >= (int)mod.inBuffer.size()) return false;

    // [修改點] 從 inBuffer 讀取，而不是 pInputLoc
    // 這樣可以確保在同一個 PLC 週期內，讀到的值是恆定的
    return (mod.inBuffer[bytePos] & (1 << bitPos)) != 0;

}


void PlcCore::FlushOutputs()// 將邏輯狀態同步到硬體地圖
{
    if (!m_pIo) return;

    for (auto& mod : *m_pIo)
    {
        // 只有當 Output 存在且有 Buffer 時才複製
        if (mod.pOutputLoc != nullptr && !mod.outBuffer.empty())
        {
            // [關鍵] 一次性記憶體複製 (Memcpy)
            // 將 outBuffer (邏輯值) -> 覆蓋到 -> pOutputLoc (硬體發送區)
            memcpy(mod.pOutputLoc, mod.outBuffer.data(), mod.outBuffer.size());
        }
    }
}
void PlcCore::Set_O(int moduleIdx, int bitIdx, bool val)
{
    // 1. [防呆] 檢查 IO 清單是否存在及索引是否合法
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return;

    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];

    // 2. [防呆] 檢查是否為有效的輸出模組
    // 如果 pOutputLoc 是 nullptr，代表這是 Input 模組或未對映，不能寫入
    if (mod.pOutputLoc == nullptr) return;

    // 3. 計算 Byte 與 Bit 位置
    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;

    // 4. [防呆] 檢查是否超出該模組的 Buffer 大小
    if (bytePos >= (int)mod.outBuffer.size()) return;

    // ==========================================================
    // 5. [核心邏輯] 只修改影子記憶體 (Shadow Buffer)
    // ==========================================================
    // 我們只修改 outBuffer，這是軟體層面的狀態。
    // 此時此刻，EtherCAT 網卡還不知道這個改變，燈也不會亮。
    // 必須等到 1ms 週期到的 FlushOutputs() 執行後，才會生效。

    if (val) {
        mod.outBuffer[bytePos] |= (1 << bitPos);  // Set bit (置 1)
    }
    else {
        mod.outBuffer[bytePos] &= ~(1 << bitPos); // Clear bit (清 0)
    }
}

bool PlcCore::Get_O(int moduleIdx, int bitIdx)
{
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return false;
    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];

    // 這裡其實不需要檢查 pOutputLoc，只要有 outBuffer 就可以讀狀態
    // 但為了保持邏輯一致，檢查一下也無妨
    if (mod.outBuffer.empty()) return false;

    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;
    if (bytePos >= (int)mod.outBuffer.size()) return false;

    // ==========================================================
    // [關鍵修改] 從 "影子記憶體(outBuffer)" 讀取
    // 不要讀 pOutputLoc，因為它可能已經被 receive 函式清零了
    // ==========================================================
    return (mod.outBuffer[bytePos] & (1 << bitPos)) != 0;
}

void PlcCore::Clear_Module_O(int moduleIdx)
{
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return;
    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];

    if (mod.pOutputLoc != nullptr && mod.outBuffer.size() > 0) {

        // 1. [原有] 清除 IO Map (這是給網卡送出去用的)
        memset(mod.pOutputLoc, 0, mod.outBuffer.size());

        // 2. [新增] 清除影子記憶體 (這是給 Set_O 運算用的)
        // 必須把這裡也歸零，下一次 Set_O 才會是乾淨的開始！
        std::fill(mod.outBuffer.begin(), mod.outBuffer.end(), 0);
    }
}

// ============================================================================
//  邏輯控制區 (乾淨的跑馬燈邏輯)
// ============================================================================

void PlcCore::Update_Debug()
{
    if (!m_pIo) return;
    for (size_t i = 0; i < m_pIo->size(); ++i)
    {
        auto& mod = (*m_pIo)[i];

        // 針對 7062 輸出模組
        if (mod.pOutputLoc != nullptr && mod.productCode == 0x00007062)
        {
            // [修改點 1] 移除 Clear_Module_O，保留舊的亮燈狀態

            // [修改點 2] 點亮當前這一顆 (累積)
            Set_O((int)i, m_marqueeLed, true);
        }

        if (mod.pOutputLoc != nullptr && mod.productCode == 0x00000003)
        {
            // [修改點 1] 移除 Clear_Module_O，保留舊的亮燈狀態

            // [修改點 2] 點亮當前這一顆 (累積)
            Set_O((int)i, m_marqueeLed, true);
        }
    }

    // 移動索引
    m_marqueeLed++;

    // [修改點 3] 只有當計數超過 15 (跑完一圈) 時，才一次全部清空並歸零
    if (m_marqueeLed > 15)
    {

        // 跑完一圈了，把所有 7062 模組全關
        for (size_t k = 0; k < m_pIo->size(); ++k)
        {
            if ((*m_pIo)[k].productCode == 0x00007062)
            {
                Clear_Module_O((int)k);
            }
        }

        // 跑完一圈了，把所有 7062 模組全關
        for (size_t k = 0; k < m_pIo->size(); ++k)
        {
            if ((*m_pIo)[k].productCode == 0x00000003)
            {
                Clear_Module_O((int)k);
            }
        }

        m_marqueeLed = 0; // 歸零重來
    }
}


// ============================================================================
// 🌟 IO 對映表設定
// ============================================================================
void PlcCore::AddInputMapping(int moduleIdx, int plcStartIndex, int bitCount) {
    m_inputMaps.push_back({ moduleIdx, plcStartIndex, bitCount });
}

void PlcCore::AddOutputMapping(int moduleIdx, int plcStartIndex, int bitCount) {
    m_outputMaps.push_back({ moduleIdx, plcStartIndex, bitCount });
}

// ============================================================================
// 🌟 核心橋接邏輯：極速資料搬移
// ============================================================================

void PlcCore::SyncPhysicalToVirtual()
{
    if (!g_PLC || !m_pIo) return;

    // 掃描所有的 Input 對映規則
    for (const auto& map : m_inputMaps) {
        for (int i = 0; i < map.bitCount; i++) {
            // 讀取實體腳位
            bool physicalVal = Get_I(map.moduleIdx, i);
            // 寫入 PLC 虛擬 I 點
            g_PLC->SetBit_I(map.plcStartIndex + i, physicalVal);
        }
    }
}

void PlcCore::SyncVirtualToPhysical()
{
    if (!g_PLC || !m_pIo) return;

    // 掃描所有的 Output 對映規則
    for (const auto& map : m_outputMaps) {
        for (int i = 0; i < map.bitCount; i++) {
            // 讀取 PLC 虛擬 O 點
            bool plcVal = g_PLC->GetBit_O(map.plcStartIndex + i);
            // 寫入實體腳位 (影子記憶體)
            Set_O(map.moduleIdx, i, plcVal);
        }
    }
}