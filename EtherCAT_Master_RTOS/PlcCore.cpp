#include "PlcCore.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <vector>
#include <map>
#include <algorithm>
#include "PLCManager.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
extern PLCManager* g_PLC; // 引用全域 PLC 指標

PlcCore::PlcCore()
{
    m_marqueeLed = 0;
}
void PlcCore::SetIoLists(std::vector<ENI_GenericIO>* pIoList, std::vector<ENI_AnalogModule>* pAdList) {
    m_pIo = pIoList;
    m_pAd = pAdList;
}

// ============================================================================
// 🌟 讀取指定路徑設定檔、支援 0=不啟用 與自動排序的智慧型 AutoMapIO
// ============================================================================
void PlcCore::AutoMapIO()
{
    m_inputMaps.clear();
    m_outputMaps.clear();
    m_analogInputMaps.clear();

    // 指定的參數檔案完整路徑
    std::string configPath = GlobalConfig::GetInstance().ParameterDir +"IoMapConfig.txt";

    // 儲存從檔案解析出來的對照表 (Key: 實體掃描序 0, 1, 2..., Value: 邏輯排序編號)
    std::map<int, int> diUserOrder;
    std::map<int, int> doUserOrder;
    std::map<int, int> adUserOrder;

    // --------------------------------------------------------------------
    // 1. 從檔案讀取並解析設定
    // --------------------------------------------------------------------
    std::ifstream file(configPath);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // 去除註解 (//) 及其後面的字串
            size_t commentPos = line.find("//");
            if (commentPos != std::string::npos) {
                line = line.substr(0, commentPos);
            }

            // 尋找等號 '='
            size_t eqPos = line.find('=');
            if (eqPos != std::string::npos) {
                std::string key = line.substr(0, eqPos);
                std::string valStr = line.substr(eqPos + 1);

                // 清除前後空白與特殊控制字元
                key.erase(0, key.find_first_not_of(" \t\r\n"));
                key.erase(key.find_last_not_of(" \t\r\n") + 1);
                valStr.erase(0, valStr.find_first_not_of(" \t\r\n"));
                valStr.erase(valStr.find_last_not_of(" \t\r\n") + 1);

                if (valStr.empty()) continue;
                int val = std::stoi(valStr);

                // 解析 DI
                if (key.rfind("DI_Index_", 0) == 0) {
                    int scanIdx = std::stoi(key.substr(9));
                    diUserOrder[scanIdx] = val;
                }
                // 解析 DO
                else if (key.rfind("DO_Index_", 0) == 0) {
                    int scanIdx = std::stoi(key.substr(9));
                    doUserOrder[scanIdx] = val;
                }
                // 解析 AD
                else if (key.rfind("AD_Index_", 0) == 0) {
                    int scanIdx = std::stoi(key.substr(9));
                    adUserOrder[scanIdx] = val;
                }
            }
        }
        file.close();
        std::cout << "[PlcCore] 成功載入 IO 映射設定檔: " << configPath << std::endl;
    }
    else {
        std::cout << "[PlcCore Warning] 找不到設定檔: " << configPath << "，將採用預設掃描順序。" << std::endl;
    }

    // --------------------------------------------------------------------
    // 2. 自動收集並處理數位輸入 (DI)
    // --------------------------------------------------------------------
    std::vector<DigitalMapItem> rawDiItems;
    if (m_pIo) {
        int scanIndex = 0;
        for (size_t i = 0; i < m_pIo->size(); i++) {
            auto& mod = (*m_pIo)[i];
            int inBits = (int)mod.inBuffer.size() * 8; // 自動抓取點數

            if (inBits > 0) {
                // 若設定檔沒寫，預設依實體掃描順序 (1, 2, 3...)
                int assignedOrder = scanIndex + 1;
                if (diUserOrder.find(scanIndex) != diUserOrder.end()) {
                    assignedOrder = diUserOrder[scanIndex];
                }

                // 🌟 關鍵防呆：若 assignedOrder 大於 0 才啟用，等於 0 則略過
                if (assignedOrder > 0) {
                    rawDiItems.push_back({ (int)i, assignedOrder, 0, inBits, "DI (ScanIdx " + std::to_string(scanIndex) + ")" });
                }
                else {
                    std::cout << "[AutoMap] 提示：DI 實體掃描序 " << scanIndex << " (Index " << i << ") 設定為 0，已略過不啟用。" << std::endl;
                }
                scanIndex++;
            }
        }
    }

    // 依自訂編號由小到大排序
    std::sort(rawDiItems.begin(), rawDiItems.end(), [](const DigitalMapItem& a, const DigitalMapItem& b) {
        return a.userOrder < b.userOrder;
        });

    // 依序對應到 PLC I 點並進行重複點位防呆
    std::set<int> usedPlcInputBits;
    int current_I_Index = 0;

    for (auto& item : rawDiItems) {
        item.plcStartIndex = current_I_Index;

        for (int b = 0; b < item.bitCount; b++) {
            int checkBit = item.plcStartIndex + b;
            if (usedPlcInputBits.count(checkBit) > 0) {
                std::cerr << "[PLC FATAL ERROR] 嚴重錯誤：PLC 點位 I" << checkBit << " 發生重複衝突！啟動中止。" << std::endl;
                return;
            }
            usedPlcInputBits.insert(checkBit);
        }

        m_inputMaps.push_back(item);
        current_I_Index += item.bitCount;
        std::cout << "[AutoMap] DI 啟用排序 -> 邏輯編號[" << item.userOrder << "] (實體Index " << item.listIdx << ") -> 對應 PLC I" << item.plcStartIndex << std::endl;
    }

    // --------------------------------------------------------------------
    // 3. 自動收集並處理數位輸出 (DO)
    // --------------------------------------------------------------------
    std::vector<DigitalMapItem> rawDoItems;
    if (m_pIo) {
        int scanIndex = 0;
        for (size_t i = 0; i < m_pIo->size(); i++) {
            auto& mod = (*m_pIo)[i];
            int outBits = (int)mod.outBuffer.size() * 8; // 自動抓取點數

            if (outBits > 0) {
                int assignedOrder = scanIndex + 1;
                if (doUserOrder.find(scanIndex) != doUserOrder.end()) {
                    assignedOrder = doUserOrder[scanIndex];
                }

                if (assignedOrder > 0) {
                    rawDoItems.push_back({ (int)i, assignedOrder, 0, outBits, "DO (ScanIdx " + std::to_string(scanIndex) + ")" });
                }
                else {
                    std::cout << "[AutoMap] 提示：DO 實體掃描序 " << scanIndex << " (Index " << i << ") 設定為 0，已略過不啟用。" << std::endl;
                }
                scanIndex++;
            }
        }
    }

    std::sort(rawDoItems.begin(), rawDoItems.end(), [](const DigitalMapItem& a, const DigitalMapItem& b) {
        return a.userOrder < b.userOrder;
        });

    std::set<int> usedPlcOutputBits;
    int current_O_Index = 0;

    for (auto& item : rawDoItems) {
        item.plcStartIndex = current_O_Index;

        for (int b = 0; b < item.bitCount; b++) {
            int checkBit = item.plcStartIndex + b;
            if (usedPlcOutputBits.count(checkBit) > 0) {
                std::cerr << "[PLC FATAL ERROR] 嚴重錯誤：PLC 點位 O" << checkBit << " 發生重複衝突！啟動中止。" << std::endl;
                return;
            }
            usedPlcOutputBits.insert(checkBit);
        }

        m_outputMaps.push_back(item);
        current_O_Index += item.bitCount;
        std::cout << "[AutoMap] DO 啟用排序 -> 邏輯編號[" << item.userOrder << "] (實體Index " << item.listIdx << ") -> 對應 PLC O" << item.plcStartIndex << std::endl;
    }

    // --------------------------------------------------------------------
    // 4. 自動收集並處理類比 (AD)
    // --------------------------------------------------------------------
    std::vector<AnalogMapItem> rawAdItems;
    if (m_pAd) {
        int scanIndex = 0;
        for (size_t i = 0; i < m_pAd->size(); i++) {
            auto& ad = (*m_pAd)[i];
            int chCount = (int)ad.channelValues.size(); // 自動抓取通道數

            if (chCount > 0) {
                int assignedOrder = scanIndex + 1;
                if (adUserOrder.find(scanIndex) != adUserOrder.end()) {
                    assignedOrder = adUserOrder[scanIndex];
                }

                if (assignedOrder > 0) {
                    rawAdItems.push_back({ (int)i, assignedOrder, 0, chCount, "AD (ScanIdx " + std::to_string(scanIndex) + ")" });
                }
                else {
                    std::cout << "[AutoMap] 提示：AD 實體掃描序 " << scanIndex << " (Index " << i << ") 設定為 0，已略過不啟用。" << std::endl;
                }
                scanIndex++;
            }
        }
    }

    std::sort(rawAdItems.begin(), rawAdItems.end(), [](const AnalogMapItem& a, const AnalogMapItem& b) {
        return a.userOrder < b.userOrder;
        });

    std::set<int> usedPlcDrRegs;
    int current_DR_Index = 100;

    for (auto& item : rawAdItems) {
        item.plcDrStartIndex = current_DR_Index;

        for (int c = 0; c < item.channelCount; c++) {
            int checkDr = item.plcDrStartIndex + c;
            if (usedPlcDrRegs.count(checkDr) > 0) {
                std::cerr << "[PLC FATAL ERROR] 嚴重錯誤：PLC 暫存器 DR" << checkDr << " 發生重複衝突！啟動中止。" << std::endl;
                return;
            }
            usedPlcDrRegs.insert(checkDr);
        }

        m_analogInputMaps.push_back(item);
        current_DR_Index += item.channelCount;
        std::cout << "[AutoMap] AD 啟用排序 -> 邏輯編號[" << item.userOrder << "] (實體Index " << item.listIdx << ") -> 對應 PLC DR" << item.plcDrStartIndex << std::endl;
    }

    std::cout << "[PlcCore] 成功從指定路徑讀取設定並完成自動排序與對應！" << std::endl;
}

// ============================================================================
// 🌟 核心橋接邏輯 (資料搬移)
// ============================================================================

// [修復Bug] 將網卡收到的硬體記憶體 (pInputLoc) 拷貝到軟體的影子記憶體 (inBuffer)
void PlcCore::FetchInputs()
{
    if (m_pIo) {
        for (auto& mod : *m_pIo) {
            if (mod.pInputLoc != nullptr && !mod.inBuffer.empty()) {
                std::memcpy(mod.inBuffer.data(), mod.pInputLoc, mod.inBuffer.size());
            }
        }
    }
    if (m_pAd) {
        for (auto& ad : *m_pAd) {
            if (ad.pInputLoc != nullptr && !ad.channelValues.empty()) {
                // 假設 channelValues 是以 2 Bytes (16-bit) 為單位
                std::memcpy(ad.channelValues.data(), ad.pInputLoc, ad.channelValues.size() * 2);
            }
        }
    }
}





void PlcCore::SyncPhysicalToVirtual()
{
    if (!g_PLC) return;

    // 1. 同步數位輸入 (DI -> I 點)
    for (const auto& map : m_inputMaps) {
        for (int i = 0; i < map.bitCount; i++) {
            bool physicalVal = Get_I(map.listIdx, i);
            g_PLC->SetBit_I(map.plcStartIndex + i, physicalVal);
        }
    }

    // 2. 同步類比輸入 (AD -> DR 暫存器)
    if (m_pAd) {
        for (const auto& map : m_analogInputMaps) {
            if (map.listIdx < 0 || map.listIdx >= (int)m_pAd->size()) continue;
            auto& ad = (*m_pAd)[map.listIdx];

            for (int ch = 0; ch < map.channelCount; ch++) {
                // 直接從記憶體區塊中抽出 16-bit 數值，轉換為 32-bit 塞入 DR
                const uint8_t* pData = reinterpret_cast<const uint8_t*>(ad.channelValues.data());
                int16_t rawVal = *reinterpret_cast<const int16_t*>(&pData[ch * 2]);
                g_PLC->SetReg_DR(map.plcDrStartIndex + ch, static_cast<int32_t>(rawVal));
            }
        }
    }
}

// ============================================================================
// 🌟 橋接邏輯：虛擬 -> 實體 (PLC -> EtherCAT)
// ============================================================================
void PlcCore::SyncVirtualToPhysical()
{
    if (!g_PLC || !m_pIo) return;

    for (const auto& map : m_outputMaps) {
        for (int i = 0; i < map.bitCount; i++) {
            bool plcVal = g_PLC->GetBit_O(map.plcStartIndex + i);
            Set_O(map.listIdx, i, plcVal);
        }
    }
}

// ============================================================================
//  底層操作 API (維持原狀)
// ============================================================================

bool PlcCore::Get_I(int listIdx, int bitIdx) {
    if (!m_pIo || listIdx < 0 || listIdx >= (int)m_pIo->size()) return false;
    ENI_GenericIO& mod = (*m_pIo)[listIdx];
    if (mod.inBuffer.empty()) return false;
    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;
    if (bytePos >= (int)mod.inBuffer.size()) return false;
    return (mod.inBuffer[bytePos] & (1 << bitPos)) != 0;
}

void PlcCore::Set_O(int listIdx, int bitIdx, bool val) {
    if (!m_pIo || listIdx < 0 || listIdx >= (int)m_pIo->size()) return;
    ENI_GenericIO& mod = (*m_pIo)[listIdx];
    if (mod.pOutputLoc == nullptr || mod.outBuffer.empty()) return;
    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;
    if (bytePos >= (int)mod.outBuffer.size()) return;

    if (val) mod.outBuffer[bytePos] |= (1 << bitPos);
    else     mod.outBuffer[bytePos] &= ~(1 << bitPos);
}

bool PlcCore::Get_O(int moduleIdx, int bitIdx)
{
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return false;
    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];
    if (mod.outBuffer.empty()) return false;

    int bytePos = bitIdx / 8;
    int bitPos = bitIdx % 8;
    if (bytePos >= (int)mod.outBuffer.size()) return false;

    return (mod.outBuffer[bytePos] & (1 << bitPos)) != 0;
}

void PlcCore::Clear_Module_O(int moduleIdx)
{
    if (!m_pIo || moduleIdx < 0 || moduleIdx >= (int)m_pIo->size()) return;
    ENI_GenericIO& mod = (*m_pIo)[moduleIdx];

    if (mod.pOutputLoc != nullptr && mod.outBuffer.size() > 0) {
        memset(mod.pOutputLoc, 0, mod.outBuffer.size());
        std::fill(mod.outBuffer.begin(), mod.outBuffer.end(), 0);
    }
}

void PlcCore::FlushOutputs() {
    if (!m_pIo) return;
    for (auto& mod : *m_pIo) {
        if (mod.pOutputLoc != nullptr && !mod.outBuffer.empty()) {
            memcpy(mod.pOutputLoc, mod.outBuffer.data(), mod.outBuffer.size());
        }
    }
}

// ============================================================================
//  邏輯控制區 (完美驗證對映表的除錯跑馬燈)
// ============================================================================
void PlcCore::Update_Debug()
{
    if (!g_PLC) return;

    // 🌟 1. 減速計數器：每 50ms 進來一次，我們累積計數
    m_marqueeLed_debug++;
    if (m_marqueeLed_debug < 5) return; // 還沒到 250ms (5 * 50ms)，直接離開
    m_marqueeLed_debug = 0;             // 時間到，歸零計數器

    // 🌟 2. 自動計算總共有多少個 Output 點被 Mapping
    int totalOutputBits = 0;
    for (const auto& map : m_outputMaps) {
        totalOutputBits += map.bitCount;
    }

    // 如果完全沒有設定 Output，就直接跳出
    if (totalOutputBits == 0) return;

    // 🌟 3. 防呆：確保索引不會超出總數量
    if (m_marqueeLed >= totalOutputBits) {
        m_marqueeLed = 0;
    }

    // 🌟 4. 跑馬燈核心邏輯：
    // 先把所有對映到的 O 點全部設為 false (熄滅)
    for (int i = 0; i < totalOutputBits; i++) {
        g_PLC->SetBit_O(i, false);
    }

    // 只點亮目前算出來的那一顆
    g_PLC->SetBit_O(m_marqueeLed, true);

    // 🌟 5. 讓 LED 索引前進一格，準備迎接下一個 250ms
    m_marqueeLed++;

    // 如果超過總數就循環繞回 0
    if (m_marqueeLed >= totalOutputBits) {
        m_marqueeLed = 0;
    }
}