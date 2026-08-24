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
#include "EtherCatMaster.h"     // Stage 11C.7 semantic consumer API
#include "EtherCatMaster_DC_Internal.h" // Stage 11F.1 runtime health snapshots
#include "EtherCatMaster_DC_Topology.h" // Stage 11F.1 DC reference presence

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
    std::string configPath = GlobalConfig::GetInstance().ParameterDir + "IoMapConfig.txt";

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
    // Stage 11C.10:
    // Before qualification, keep legacy cache copies alive.
    // After retirement, inBuffer/channelValues become metadata-only.
    if (m_LegacyInputCacheRetired.load(
        std::memory_order_relaxed))
    {
        m_LegacyInputCacheSuppressedCycles.fetch_add(
            1ULL,
            std::memory_order_relaxed);

        return;
    }


    if (m_pIo)
    {
        for (auto& mod :
            *m_pIo)
        {
            if (mod.pInputLoc != nullptr &&
                !mod.inBuffer.empty())
            {
                std::memcpy(
                    mod.inBuffer.data(),
                    mod.pInputLoc,
                    mod.inBuffer.size());
            }
        }
    }


    if (m_pAd)
    {
        for (auto& ad :
            *m_pAd)
        {
            if (ad.pInputLoc != nullptr &&
                !ad.channelValues.empty())
            {
                std::memcpy(
                    ad.channelValues.data(),
                    ad.pInputLoc,
                    ad.channelValues.size() *
                    sizeof(int16_t));
            }
        }
    }
}





void PlcCore::SyncPhysicalToVirtual()
{
    if (!g_PLC)
    {
        return;
    }


    // ========================================================================
    // Stage 11D.2 - Structured ServoDrive Live Read SHADOW
    //
    // One sample per existing PLC input bridge cycle (~1ms).
    //
    // No logging / allocation / hardware access occurs in the sampler.
    // Motion still consumes ENI_ServoDrive directly.
    // ========================================================================

    if (m_pGenericReadMaster !=
        nullptr &&
        !m_pGenericReadMaster->
        IsServoInputReleaseComplete())
    {
        m_pGenericReadMaster->
            SampleStructuredServoDriveLiveReadShadow();
    }


    // ========================================================================
    // Stage 11C.8 - GENERIC INPUT LIVE ROUTE
    //
    // No allocation.
    // No logging.
    // No hardware register access.
    //
    // The semantic occurrence tables were prepared during startup.
    //
    // If a generic read unexpectedly fails, this cycle falls back to the
    // already-proven legacy input path so PLC state remains continuous.
    // ========================================================================

    bool genericCyclePass =
        m_GenericInputLiveRouteEnabled &&
        m_GenericInputLiveRoutePrepared &&
        m_pGenericReadMaster != nullptr &&
        m_GenericDiSemanticOccurrences.size() ==
        m_inputMaps.size() &&
        m_GenericAdBaseOccurrences.size() ==
        m_analogInputMaps.size();


    if (genericCyclePass)
    {
        // --------------------------------------------------------------------
        // 1. Digital Input -> PLC I
        // --------------------------------------------------------------------

        for (size_t mapIndex = 0;
            mapIndex <
            m_inputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_inputMaps[
                    mapIndex];


            if (map.bitCount <= 0 ||
                map.bitCount > 64)
            {
                genericCyclePass =
                    false;

                break;
            }


            uint64_t raw =
                0ULL;


            const bool readPass =
                m_pGenericReadMaster->
                ReadCompositeConsumerBits(
                    "DigitalInput",
                    "",
                    m_GenericDiSemanticOccurrences[
                        mapIndex],
                    raw);


            if (!readPass)
            {
                genericCyclePass =
                    false;

                break;
            }


            for (int bit = 0;
                bit <
                map.bitCount;
                ++bit)
            {
                const bool value =
                    (
                        (
                            raw >>
                            bit
                            ) &
                        0x01ULL
                        ) !=
                    0ULL;


                g_PLC->SetBit_I(
                    map.plcStartIndex +
                    bit,
                    value);
            }
        }


        // --------------------------------------------------------------------
        // 2. Analog Input -> PLC DR
        // --------------------------------------------------------------------

        if (genericCyclePass)
        {
            for (size_t mapIndex = 0;
                mapIndex <
                m_analogInputMaps.size();
                ++mapIndex)
            {
                const auto& map =
                    m_analogInputMaps[
                        mapIndex];


                if (map.channelCount <=
                    0)
                {
                    genericCyclePass =
                        false;

                    break;
                }


                const int baseOccurrence =
                    m_GenericAdBaseOccurrences[
                        mapIndex];


                for (int channel = 0;
                    channel <
                    map.channelCount;
                    ++channel)
                {
                    int16_t raw =
                        0;


                    const bool readPass =
                        m_pGenericReadMaster->
                        ReadCompositeConsumerInt16(
                            "AnalogInput",
                            "",
                            baseOccurrence +
                            channel,
                            raw);


                    if (!readPass)
                    {
                        genericCyclePass =
                            false;

                        break;
                    }


                    g_PLC->SetReg_DR(
                        map.plcDrStartIndex +
                        channel,
                        static_cast<int32_t>(
                            raw));
                }


                if (!genericCyclePass)
                {
                    break;
                }
            }
        }


        if (genericCyclePass)
        {
            m_GenericInputLiveCycles.fetch_add(
                1ULL,
                std::memory_order_relaxed);

            return;
        }


        m_GenericInputReadFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }


    // ========================================================================
    // Compatibility / safety fallback
    //
    // Stage 11C.10:
    // fallback no longer depends on retired inBuffer/channelValues.
    // It reads the proven pInputLoc Process Image pointers directly.
    // ========================================================================

    m_GenericInputFallbackCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);

    m_DirectInputFallbackCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    bool directFallbackPass =
        true;


    // Digital Input -> PLC I
    if (m_pIo == nullptr)
    {
        directFallbackPass =
            false;
    }
    else
    {
        for (const auto& map :
            m_inputMaps)
        {
            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()))
            {
                directFallbackPass =
                    false;

                continue;
            }


            const auto& io =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            if (io.pInputLoc == nullptr ||
                map.bitCount <= 0)
            {
                directFallbackPass =
                    false;

                continue;
            }


            const uint8_t* inputBytes =
                static_cast<const uint8_t*>(
                    io.pInputLoc);


            for (int bit = 0;
                bit < map.bitCount;
                ++bit)
            {
                const int bytePos =
                    bit / 8;

                const int bitPos =
                    bit % 8;

                const bool value =
                    (
                        inputBytes[
                            bytePos] &
                        static_cast<uint8_t>(
                            1U << bitPos)
                                ) !=
                    0U;


                            g_PLC->SetBit_I(
                                map.plcStartIndex +
                                bit,
                                value);
            }
        }
    }


    // Analog Input -> PLC DR
    if (m_pAd == nullptr)
    {
        directFallbackPass =
            false;
    }
    else
    {
        for (const auto& map :
            m_analogInputMaps)
        {
            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pAd->size()))
            {
                directFallbackPass =
                    false;

                continue;
            }


            const auto& ad =
                (*m_pAd)[
                    static_cast<size_t>(
                        map.listIdx)];


            if (ad.pInputLoc == nullptr ||
                map.channelCount <= 0)
            {
                directFallbackPass =
                    false;

                continue;
            }


            const uint8_t* inputBytes =
                static_cast<const uint8_t*>(
                    ad.pInputLoc);


            for (int channel = 0;
                channel < map.channelCount;
                ++channel)
            {
                int16_t raw =
                    0;


                std::memcpy(
                    &raw,
                    inputBytes +
                    channel *
                    sizeof(int16_t),
                    sizeof(raw));


                g_PLC->SetReg_DR(
                    map.plcDrStartIndex +
                    channel,
                    static_cast<int32_t>(
                        raw));
            }
        }
    }


    if (!directFallbackPass)
    {
        m_DirectInputFallbackFailures.fetch_add(
            1ULL,
            std::memory_order_relaxed);
    }
}


// ============================================================================
// 🌟 橋接邏輯：虛擬 -> 實體 (PLC -> EtherCAT)
// ============================================================================
void PlcCore::SyncVirtualToPhysical()
{
    if (!g_PLC ||
        !m_pIo)
    {
        return;
    }


    const bool genericControlledOutput =
        m_GenericOutputControlledCutoverEnabled.load(
            std::memory_order_acquire);


    // ========================================================================
    // Stage 11C.14 - ACTIVE Generic DigitalOutput command route
    //
    // IMPORTANT:
    //
    // This does NOT write pOutputLoc.
    //
    // It writes only the existing software outBuffer.  The already-proven
    // FlushOutputs() remains the only physical Process Image writer.
    //
    // Stage11C.14 intentionally supports only a whole-buffer, byte-aligned
    // DigitalOutput owner.  Future packed/composite output sub-ranges are
    // blocked by PrepareGenericOutputControlledCutover() until a dedicated
    // masked live writer stage is introduced.
    // ========================================================================

    if (genericControlledOutput)
    {
        bool routePass =
            m_GenericOutputControlledCutoverPrepared &&
            !m_GenericOutputControlledCutoverFaulted.load(
                std::memory_order_relaxed) &&
            m_GenericOutputControlledCutoverMapCount ==
            static_cast<int>(
                m_outputMaps.size()) &&
            m_GenericDoSemanticOccurrences.size() ==
            m_outputMaps.size();


        // --------------------------------------------------------------------
        // Validate the whole route BEFORE modifying any command buffer.
        // --------------------------------------------------------------------

        if (routePass)
        {
            for (size_t mapIndex = 0;
                mapIndex <
                m_outputMaps.size();
                ++mapIndex)
            {
                const auto& map =
                    m_outputMaps[
                        mapIndex];


                if (map.listIdx < 0 ||
                    map.listIdx >=
                    static_cast<int>(
                        m_pIo->size()) ||
                    map.bitCount <= 0 ||
                    map.bitCount > 64 ||
                    (map.bitCount % 8) != 0)
                {
                    routePass =
                        false;

                    break;
                }


                const auto& io =
                    (*m_pIo)[
                        static_cast<size_t>(
                            map.listIdx)];


                if (io.outBuffer.empty() ||
                    io.pOutputLoc ==
                    nullptr ||
                    io.outBuffer.size() *
                    8U !=
                    static_cast<size_t>(
                        map.bitCount))
                {
                    routePass =
                        false;

                    break;
                }
            }
        }


        if (routePass)
        {
            uint64_t mapWrites =
                0ULL;


            for (size_t mapIndex = 0;
                mapIndex <
                m_outputMaps.size();
                ++mapIndex)
            {
                const auto& map =
                    m_outputMaps[
                        mapIndex];


                auto& io =
                    (*m_pIo)[
                        static_cast<size_t>(
                            map.listIdx)];


                // Whole-buffer owner proven during Stage11C.14 prepare.
                std::fill(
                    io.outBuffer.begin(),
                    io.outBuffer.end(),
                    static_cast<uint8_t>(
                        0U));


                for (int bit = 0;
                    bit <
                    map.bitCount;
                    ++bit)
                {
                    const bool plcValue =
                        g_PLC->GetBit_O(
                            map.plcStartIndex +
                            bit);


                    if (!plcValue)
                    {
                        continue;
                    }


                    const int bytePos =
                        bit /
                        8;


                    const int bitPos =
                        bit %
                        8;


                    io.outBuffer[
                        static_cast<size_t>(
                            bytePos)] |=
                        static_cast<uint8_t>(
                            1U <<
                            bitPos);
                }


                mapWrites++;
            }


            m_GenericOutputControlledMapWrites.fetch_add(
                mapWrites,
                std::memory_order_relaxed);


            m_GenericOutputControlledLiveCycles.fetch_add(
                1ULL,
                std::memory_order_relaxed);


            return;
        }


        // --------------------------------------------------------------------
        // One-way fault latch for this boot.
        //
        // The failed Generic route is disabled before falling through to the
        // proven legacy Set_O() path below.
        // --------------------------------------------------------------------

        m_GenericOutputControlledRouteFaults.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        m_GenericOutputControlledFallbacks.fetch_add(
            1ULL,
            std::memory_order_relaxed);


        m_GenericOutputControlledCutoverFaulted.store(
            true,
            std::memory_order_release);


        m_GenericOutputControlledCutoverEnabled.store(
            false,
            std::memory_order_release);
    }


    // ========================================================================
    // ACTIVE LEGACY / FALLBACK path
    // ========================================================================

    for (const auto& map :
        m_outputMaps)
    {
        for (int bit = 0;
            bit <
            map.bitCount;
            ++bit)
        {
            const bool plcValue =
                g_PLC->GetBit_O(
                    map.plcStartIndex +
                    bit);


            Set_O(
                map.listIdx,
                bit,
                plcValue);
        }
    }


    // ========================================================================
    // Stage 11C.12 command equivalence SHADOW
    //
    // Once Stage11C.14 activates, this section is bypassed by the early return
    // above.  Therefore the already-qualified Stage11C.12 evidence freezes
    // rather than becoming a self-comparison of the new live route.
    // ========================================================================

    if (!m_GenericOutputCommandBridgeShadowEnabled ||
        !m_GenericOutputOwnershipShadowPrepared ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        return;
    }


    uint64_t cycleChecks =
        0ULL;

    uint64_t cycleMatches =
        0ULL;

    uint64_t cycleMismatches =
        0ULL;


    for (size_t mapIndex = 0;
        mapIndex <
        m_outputMaps.size();
        ++mapIndex)
    {
        const auto& map =
            m_outputMaps[
                mapIndex];


        cycleChecks++;


        if (map.listIdx < 0 ||
            map.listIdx >=
            static_cast<int>(
                m_pIo->size()) ||
            map.bitCount <= 0 ||
            map.bitCount > 64)
        {
            cycleMismatches++;
            continue;
        }


        const auto& io =
            (*m_pIo)[
                static_cast<size_t>(
                    map.listIdx)];


        if (io.outBuffer.empty() ||
            io.outBuffer.size() *
            8U <
            static_cast<size_t>(
                map.bitCount))
        {
            cycleMismatches++;
            continue;
        }


        uint64_t semanticCommand =
            0ULL;

        uint64_t legacyCommand =
            0ULL;


        for (int bit = 0;
            bit <
            map.bitCount;
            ++bit)
        {
            const bool plcValue =
                g_PLC->GetBit_O(
                    map.plcStartIndex +
                    bit);


            if (plcValue)
            {
                semanticCommand |=
                    1ULL <<
                    bit;
            }


            const int bytePos =
                bit /
                8;


            const int bitPos =
                bit %
                8;


            const bool legacyValue =
                (
                    io.outBuffer[
                        static_cast<size_t>(
                            bytePos)] &
                    static_cast<uint8_t>(
                        1U <<
                        bitPos)
                            ) !=
                0U;


                        if (legacyValue)
                        {
                            legacyCommand |=
                                1ULL <<
                                bit;
                        }
        }


        if (semanticCommand ==
            legacyCommand)
        {
            cycleMatches++;
        }
        else
        {
            cycleMismatches++;
        }
    }


    m_GenericOutputCommandShadowCycles.fetch_add(
        1ULL,
        std::memory_order_relaxed);


    m_GenericOutputCommandShadowMapChecks.fetch_add(
        cycleChecks,
        std::memory_order_relaxed);


    m_GenericOutputCommandShadowMatches.fetch_add(
        cycleMatches,
        std::memory_order_relaxed);


    m_GenericOutputCommandShadowMismatches.fetch_add(
        cycleMismatches,
        std::memory_order_relaxed);
}


// ============================================================================
//  底層操作 API (維持原狀)
// ============================================================================

bool PlcCore::Get_I(
    int listIdx,
    int bitIdx)
{
    if (!m_pIo ||
        listIdx < 0 ||
        listIdx >=
        static_cast<int>(
            m_pIo->size()) ||
        bitIdx < 0)
    {
        return false;
    }


    ENI_GenericIO& mod =
        (*m_pIo)[
            static_cast<size_t>(
                listIdx)];


    const int bytePos =
        bitIdx / 8;

    const int bitPos =
        bitIdx % 8;


    if (m_LegacyInputCacheRetired.load(
        std::memory_order_relaxed))
    {
        if (mod.pInputLoc == nullptr)
        {
            return false;
        }


        const uint8_t* inputBytes =
            static_cast<const uint8_t*>(
                mod.pInputLoc);


        return
            (
                inputBytes[
                    bytePos] &
                static_cast<uint8_t>(
                    1U << bitPos)
                        ) !=
            0U;
    }


    if (mod.inBuffer.empty() ||
        bytePos >=
        static_cast<int>(
            mod.inBuffer.size()))
    {
        return false;
    }


    return
        (
            mod.inBuffer[
                static_cast<size_t>(
                    bytePos)] &
            static_cast<uint8_t>(
                1U << bitPos)
                    ) !=
        0U;
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


// ============================================================================
// Stage 11C.7 - PLC Generic Read Consumer Map SHADOW
//
// IMPORTANT:
// - Does NOT modify FetchInputs()
// - Does NOT modify SyncPhysicalToVirtual()
// - Does NOT write PLC I / DR values
// - Does NOT touch output path
//
// It only validates that the current IoMapConfig-driven legacy PLC mapping can
// be represented by:
//
//     Kind + AxisRef + occurrence
//
// using the already-active Stage11C.6 consumer API.
// ============================================================================

void PlcCore::SetGenericReadMaster(
    EtherCatMaster* pMaster)
{
    m_pGenericReadMaster =
        pMaster;
}


bool PlcCore::AuditGenericReadConsumerMapShadow()
{
    int diMaps =
        0;

    int diBits =
        0;

    int diSemanticLookups =
        0;

    int diMatches =
        0;

    int adMaps =
        0;

    int adChannels =
        0;

    int adSemanticLookups =
        0;

    int adMatches =
        0;

    int outputMapsDeferred =
        static_cast<int>(
            m_outputMaps.size());

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-READ-MAP-SHADOW] BEGIN | "
        "Stage:11C.7 | "
        "Consumer:PLC | "
        "MapSource:IOMAP_CONFIG | "
        "ReadCandidate:GENERIC_CONSUMER_API | "
        "LivePLCRead:LEGACY | LivePLCWrite:LEGACY | "
        "Motion:LEGACY | ProductCodeBaseline:NO\n"
        "============================================================\n");


    if (m_pGenericReadMaster ==
        nullptr)
    {
        errors++;
    }


    // ========================================================================
    // DI mapping
    //
    // m_inputMaps is sorted by userOrder for PLC address assignment.
    //
    // The semantic occurrence must NOT be derived from this sorted order,
    // otherwise changing DI_Index_x would accidentally change hardware
    // identity.
    //
    // Therefore occurrence is calculated from the original physical m_pIo
    // scan order for input-capable modules.
    // ========================================================================

    if (m_pIo != nullptr &&
        m_pGenericReadMaster != nullptr)
    {
        for (const auto& map :
            m_inputMaps)
        {
            diMaps++;


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()))
            {
                errors++;
                continue;
            }


            int semanticOccurrence =
                0;


            bool targetFound =
                false;


            for (int ioIndex = 0;
                ioIndex <
                static_cast<int>(
                    m_pIo->size());
                ++ioIndex)
            {
                const auto& io =
                    (*m_pIo)[
                        static_cast<size_t>(
                            ioIndex)];


                if (io.inBuffer.empty())
                {
                    continue;
                }


                if (ioIndex ==
                    map.listIdx)
                {
                    targetFound =
                        true;

                    break;
                }


                semanticOccurrence++;
            }


            if (!targetFound ||
                map.bitCount <= 0 ||
                map.bitCount > 64)
            {
                errors++;
                continue;
            }


            const auto* descriptor =
                m_pGenericReadMaster->
                ResolveCompositeReadConsumer(
                    "DigitalInput",
                    "",
                    semanticOccurrence);


            uint64_t genericValue =
                0ULL;


            uint64_t legacyValue =
                0ULL;


            const bool genericPass =
                descriptor != nullptr &&
                descriptor->inputBitLength ==
                static_cast<uint32_t>(
                    map.bitCount) &&
                m_pGenericReadMaster->
                ReadCompositeConsumerBits(
                    "DigitalInput",
                    "",
                    semanticOccurrence,
                    genericValue);


            diSemanticLookups++;


            bool legacyPass =
                true;


            for (int bit = 0;
                bit <
                map.bitCount;
                ++bit)
            {
                const bool value =
                    Get_I(
                        map.listIdx,
                        bit);


                if (value)
                {
                    legacyValue |=
                        1ULL <<
                        bit;
                }


                diBits++;
            }


            if (genericPass &&
                legacyPass &&
                genericValue ==
                legacyValue)
            {
                diMatches++;
            }
            else
            {
                errors++;
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-READ-MAP-DI] "
                "LegacyList:%d | UserOrder:%d | "
                "Semantic:DigitalInput/-/%d | "
                "Resolved:S%d/%s | "
                "PLC:I%d..I%d | Bits:%d | "
                "Generic:0x%llX Legacy:0x%llX | Match:%s\n",

                map.listIdx,

                map.userOrder,

                semanticOccurrence,

                descriptor != nullptr
                ? descriptor->slaveIndex
                : -1,

                descriptor != nullptr &&
                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,

                map.plcStartIndex +
                map.bitCount -
                1,

                map.bitCount,

                (unsigned long long)
                genericValue,

                (unsigned long long)
                legacyValue,

                genericPass &&
                legacyPass &&
                genericValue ==
                legacyValue
                ? "YES"
                : "NO");
        }
    }
    else if (!m_inputMaps.empty())
    {
        errors++;
    }


    // ========================================================================
    // AD mapping
    //
    // Legacy PlcCore sees one ENI_AnalogModule with N channels.
    //
    // Current Composite project exposes each channel as one semantic
    // AnalogInput binding:
    //
    //     AnalogInput / "" / occurrence
    //
    // Base occurrence is derived from PHYSICAL AD module scan order, while
    // plcDrStartIndex still follows IoMapConfig userOrder.
    // ========================================================================

    if (m_pAd != nullptr &&
        m_pGenericReadMaster != nullptr)
    {
        for (const auto& map :
            m_analogInputMaps)
        {
            adMaps++;


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pAd->size()))
            {
                errors++;
                continue;
            }


            const auto& targetAd =
                (*m_pAd)[
                    static_cast<size_t>(
                        map.listIdx)];


            if (map.channelCount <= 0 ||
                map.channelCount >
                static_cast<int>(
                    targetAd.channelValues.size()))
            {
                errors++;
                continue;
            }


            int baseOccurrence =
                0;


            for (int adIndex = 0;
                adIndex <
                map.listIdx;
                ++adIndex)
            {
                baseOccurrence +=
                    static_cast<int>(
                        (*m_pAd)[
                            static_cast<size_t>(
                                adIndex)]
                        .channelValues
                                .size());
            }


            for (int channel = 0;
                channel <
                map.channelCount;
                ++channel)
            {
                const int semanticOccurrence =
                    baseOccurrence +
                    channel;


                const auto* descriptor =
                    m_pGenericReadMaster->
                    ResolveCompositeReadConsumer(
                        "AnalogInput",
                        "",
                        semanticOccurrence);


                int16_t genericValue =
                    0;


                const bool genericPass =
                    descriptor != nullptr &&
                    descriptor->inputBitLength ==
                    16U &&
                    std::strcmp(
                        descriptor->dataType,
                        "Int16") ==
                    0 &&
                    m_pGenericReadMaster->
                    ReadCompositeConsumerInt16(
                        "AnalogInput",
                        "",
                        semanticOccurrence,
                        genericValue);


                adSemanticLookups++;
                adChannels++;


                const int16_t legacyValue =
                    targetAd.channelValues[
                        static_cast<size_t>(
                            channel)];


                if (genericPass &&
                    genericValue ==
                    legacyValue)
                {
                    adMatches++;
                }
                else
                {
                    errors++;
                }


                DEBUG_PRINT(
                    "[PLC-GENERIC-READ-MAP-AI] "
                    "LegacyList:%d Ch:%d | UserOrder:%d | "
                    "Semantic:AnalogInput/-/%d | "
                    "Resolved:S%d/%s | "
                    "PLC:DR%d | "
                    "Generic:%d Legacy:%d | Match:%s\n",

                    map.listIdx,

                    channel,

                    map.userOrder,

                    semanticOccurrence,

                    descriptor != nullptr
                    ? descriptor->slaveIndex
                    : -1,

                    descriptor != nullptr &&
                    descriptor->id[0] != '\0'
                    ? descriptor->id
                    : "N/A",

                    map.plcDrStartIndex +
                    channel,

                    (int)
                    genericValue,

                    (int)
                    legacyValue,

                    genericPass &&
                    genericValue ==
                    legacyValue
                    ? "YES"
                    : "NO");
            }
        }
    }
    else if (!m_analogInputMaps.empty())
    {
        errors++;
    }


    // ========================================================================
    // DO remains deliberately outside this stage.
    // ========================================================================

    DEBUG_PRINT(
        "[PLC-GENERIC-READ-MAP-OUTPUT-DEFER] "
        "OutputMaps:%d | "
        "Action:KEEP_LEGACY | "
        "GenericLiveWrite:NO\n",

        outputMapsDeferred);


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[PLC-GENERIC-READ-MAP-SHADOW-RESULT] "
        "DIMaps:%d | DIBits:%d | DISemanticLookups:%d | DIMatches:%d | "
        "ADMaps:%d | ADChannels:%d | ADSemanticLookups:%d | ADMatches:%d | "
        "OutputMapsDeferred:%d | "
        "Errors:%d | Result:%s | "
        "MapSource:IOMAP_CONFIG | "
        "ReadCandidate:GENERIC_CONSUMER_API | "
        "LivePLCRead:LEGACY | LivePLCWrite:LEGACY | "
        "Motion:LEGACY | ProductCodeBaseline:NO | "
        "BridgeAction:SHADOW_ONLY | StartupAction:CONTINUE\n",

        diMaps,

        diBits,

        diSemanticLookups,

        diMatches,

        adMaps,

        adChannels,

        adSemanticLookups,

        adMatches,

        outputMapsDeferred,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-READ-MAP-SHADOW] END | "
        "Stage:11C.7 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11C.8 - PLC Generic INPUT Live Route
// ============================================================================

bool PlcCore::PrepareGenericInputLiveRoute()
{
    m_GenericDiSemanticOccurrences.clear();
    m_GenericAdBaseOccurrences.clear();

    m_GenericInputLiveRoutePrepared =
        false;

    m_GenericInputLiveRouteEnabled =
        false;


    int diPrepared =
        0;

    int adMapsPrepared =
        0;

    int adChannelsPrepared =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-INPUT-LIVE-PREPARE] BEGIN | "
        "Stage:11C.8 | "
        "MapSource:IOMAP_CONFIG | "
        "InputSource:GENERIC_CONSUMER_API | "
        "PLCOutput:LEGACY | Motion:LEGACY | LiveOutputWrite:NO\n"
        "============================================================\n");


    if (m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_pAd ==
        nullptr)
    {
        errors++;
    }


    // ------------------------------------------------------------------------
    // DI: resolve physical module scan order once.
    // ------------------------------------------------------------------------

    if (errors ==
        0)
    {
        for (const auto& map :
            m_inputMaps)
        {
            int semanticOccurrence =
                0;


            bool targetFound =
                false;


            for (int ioIndex = 0;
                ioIndex <
                static_cast<int>(
                    m_pIo->size());
                ++ioIndex)
            {
                const auto& io =
                    (*m_pIo)[
                        static_cast<size_t>(
                            ioIndex)];


                if (io.inBuffer.empty())
                {
                    continue;
                }


                if (ioIndex ==
                    map.listIdx)
                {
                    targetFound =
                        true;

                    break;
                }


                semanticOccurrence++;
            }


            const auto* descriptor =
                targetFound
                ? m_pGenericReadMaster->
                ResolveCompositeReadConsumer(
                    "DigitalInput",
                    "",
                    semanticOccurrence)
                : nullptr;


            if (!targetFound ||
                descriptor ==
                nullptr ||
                map.bitCount <=
                0 ||
                map.bitCount >
                64 ||
                descriptor->inputBitLength !=
                static_cast<uint32_t>(
                    map.bitCount))
            {
                errors++;

                break;
            }


            m_GenericDiSemanticOccurrences.push_back(
                semanticOccurrence);


            diPrepared++;


            DEBUG_PRINT(
                "[PLC-GENERIC-INPUT-LIVE-PREPARE-DI] "
                "LegacyList:%d | UserOrder:%d | "
                "Semantic:DigitalInput/-/%d | "
                "Resolved:S%d/%s | PLC:I%d..I%d | Result:PASS\n",

                map.listIdx,

                map.userOrder,

                semanticOccurrence,

                descriptor->slaveIndex,

                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,

                map.plcStartIndex +
                map.bitCount -
                1);
        }
    }


    // ------------------------------------------------------------------------
    // AD: resolve each module's base semantic occurrence once.
    // ------------------------------------------------------------------------

    if (errors ==
        0)
    {
        for (const auto& map :
            m_analogInputMaps)
        {
            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pAd->size()) ||
                map.channelCount <=
                0)
            {
                errors++;

                break;
            }


            int baseOccurrence =
                0;


            for (int adIndex = 0;
                adIndex <
                map.listIdx;
                ++adIndex)
            {
                baseOccurrence +=
                    static_cast<int>(
                        (*m_pAd)[
                            static_cast<size_t>(
                                adIndex)]
                        .channelValues
                                .size());
            }


            bool modulePass =
                true;


            for (int channel = 0;
                channel <
                map.channelCount;
                ++channel)
            {
                const int occurrence =
                    baseOccurrence +
                    channel;


                const auto* descriptor =
                    m_pGenericReadMaster->
                    ResolveCompositeReadConsumer(
                        "AnalogInput",
                        "",
                        occurrence);


                if (descriptor ==
                    nullptr ||
                    descriptor->inputBitLength !=
                    16U ||
                    std::strcmp(
                        descriptor->dataType,
                        "Int16") !=
                    0)
                {
                    modulePass =
                        false;

                    errors++;

                    break;
                }


                adChannelsPrepared++;


                DEBUG_PRINT(
                    "[PLC-GENERIC-INPUT-LIVE-PREPARE-AI] "
                    "LegacyList:%d Ch:%d | UserOrder:%d | "
                    "Semantic:AnalogInput/-/%d | "
                    "Resolved:S%d/%s | PLC:DR%d | Result:PASS\n",

                    map.listIdx,

                    channel,

                    map.userOrder,

                    occurrence,

                    descriptor->slaveIndex,

                    descriptor->id[0] != '\0'
                    ? descriptor->id
                    : "N/A",

                    map.plcDrStartIndex +
                    channel);
            }


            if (!modulePass)
            {
                break;
            }


            m_GenericAdBaseOccurrences.push_back(
                baseOccurrence);


            adMapsPrepared++;
        }
    }


    const bool pass =
        errors ==
        0 &&
        m_GenericDiSemanticOccurrences.size() ==
        m_inputMaps.size() &&
        m_GenericAdBaseOccurrences.size() ==
        m_analogInputMaps.size();


    if (pass)
    {
        m_GenericInputLiveRoutePrepared =
            true;
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-INPUT-LIVE-PREPARE-RESULT] "
        "DIMaps:%d | ADMaps:%d | ADChannels:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "LivePLCRead:NOT_YET_ACTIVE | "
        "PLCOutput:LEGACY | Motion:LEGACY | LiveOutputWrite:NO\n",

        diPrepared,

        adMapsPrepared,

        adChannelsPrepared,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_GenericInputLiveRoutePrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-INPUT-LIVE-PREPARE] END | "
        "Stage:11C.8 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool PlcCore::AuditGenericInputLiveRoute()
{
    int diChecks =
        0;

    int diMatches =
        0;

    int adChecks =
        0;

    int adMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-INPUT-LIVE-AUDIT] BEGIN | "
        "Stage:11C.8 | "
        "Prepared:%s | "
        "Compare:GENERIC_VS_LEGACY | "
        "WritePLC:NO | LiveOutputWrite:NO\n"
        "============================================================\n",

        m_GenericInputLiveRoutePrepared
        ? "YES"
        : "NO");


    if (!m_GenericInputLiveRoutePrepared ||
        m_pGenericReadMaster ==
        nullptr ||
        m_GenericDiSemanticOccurrences.size() !=
        m_inputMaps.size() ||
        m_GenericAdBaseOccurrences.size() !=
        m_analogInputMaps.size())
    {
        errors++;
    }


    // DI
    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_inputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_inputMaps[
                    mapIndex];


            uint64_t genericValue =
                0ULL;


            uint64_t legacyValue =
                0ULL;


            const bool genericPass =
                m_pGenericReadMaster->
                ReadCompositeConsumerBits(
                    "DigitalInput",
                    "",
                    m_GenericDiSemanticOccurrences[
                        mapIndex],
                    genericValue);


            for (int bit = 0;
                bit <
                map.bitCount;
                ++bit)
            {
                if (Get_I(
                    map.listIdx,
                    bit))
                {
                    legacyValue |=
                        1ULL <<
                        bit;
                }
            }


            diChecks++;


            if (genericPass &&
                genericValue ==
                legacyValue)
            {
                diMatches++;
            }
            else
            {
                errors++;
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-INPUT-LIVE-AUDIT-DI] "
                "Map:%u | Semantic:DigitalInput/-/%d | "
                "Generic:0x%llX Legacy:0x%llX | Match:%s\n",

                (unsigned int)
                mapIndex,

                m_GenericDiSemanticOccurrences[
                    mapIndex],

                (unsigned long long)
                        genericValue,

                        (unsigned long long)
                        legacyValue,

                        genericPass &&
                        genericValue ==
                        legacyValue
                        ? "YES"
                        : "NO");
        }
    }


    // AD
    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_analogInputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_analogInputMaps[
                    mapIndex];


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pAd->size()))
            {
                errors++;

                break;
            }


            const auto& ad =
                (*m_pAd)[
                    static_cast<size_t>(
                        map.listIdx)];


            for (int channel = 0;
                channel <
                map.channelCount;
                ++channel)
            {
                int16_t genericValue =
                    0;


                const bool genericPass =
                    m_pGenericReadMaster->
                    ReadCompositeConsumerInt16(
                        "AnalogInput",
                        "",
                        m_GenericAdBaseOccurrences[
                            mapIndex] +
                        channel,
                                genericValue);


                const bool legacyAvailable =
                    channel <
                    static_cast<int>(
                        ad.channelValues.size());


                const int16_t legacyValue =
                    legacyAvailable
                    ? ad.channelValues[
                        static_cast<size_t>(
                            channel)]
                    : 0;


                        adChecks++;


                        if (genericPass &&
                            legacyAvailable &&
                            genericValue ==
                            legacyValue)
                        {
                            adMatches++;
                        }
                        else
                        {
                            errors++;
                        }


                        DEBUG_PRINT(
                            "[PLC-GENERIC-INPUT-LIVE-AUDIT-AI] "
                            "Map:%u Ch:%d | Semantic:AnalogInput/-/%d | "
                            "Generic:%d Legacy:%d | Match:%s\n",

                            (unsigned int)
                            mapIndex,

                            channel,

                            m_GenericAdBaseOccurrences[
                                mapIndex] +
                            channel,

                                    (int)
                                    genericValue,

                                    (int)
                                    legacyValue,

                                    genericPass &&
                                    legacyAvailable &&
                                    genericValue ==
                                    legacyValue
                                    ? "YES"
                                    : "NO");
            }
        }
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[PLC-GENERIC-INPUT-LIVE-AUDIT-RESULT] "
        "DIChecks:%d | DIMatches:%d | "
        "ADChecks:%d | ADMatches:%d | "
        "Errors:%d | Result:%s | "
        "WritePLC:NO | "
        "ActivationAction:%s\n",

        diChecks,

        diMatches,

        adChecks,

        adMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        pass
        ? "ALLOW"
        : "BLOCK");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-INPUT-LIVE-AUDIT] END | "
        "Stage:11C.8 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool PlcCore::ActivateGenericInputLiveRoute()
{
    const bool pass =
        m_GenericInputLiveRoutePrepared &&
        m_pGenericReadMaster !=
        nullptr &&
        m_GenericDiSemanticOccurrences.size() ==
        m_inputMaps.size() &&
        m_GenericAdBaseOccurrences.size() ==
        m_analogInputMaps.size();


    m_GenericInputLiveRouteEnabled =
        pass;


    if (pass)
    {
        // Stage 11C.9 observation window begins exactly at activation.
        m_GenericInputLiveCycles.store(
            0ULL,
            std::memory_order_relaxed);

        m_GenericInputFallbackCycles.store(
            0ULL,
            std::memory_order_relaxed);

        m_GenericInputReadFailures.store(
            0ULL,
            std::memory_order_relaxed);
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-INPUT-LIVE-ACTIVATE] "
        "Prepared:%s | "
        "DIMaps:%u | ADMaps:%u | "
        "Result:%s | RouteEnabled:%s | "
        "LivePLCRead:%s | "
        "FetchInputs:COMPATIBILITY | "
        "PLCOutput:LEGACY | Motion:LEGACY | LiveOutputWrite:NO\n",

        m_GenericInputLiveRoutePrepared
        ? "YES"
        : "NO",

        (unsigned int)
        m_GenericDiSemanticOccurrences.size(),

        (unsigned int)
        m_GenericAdBaseOccurrences.size(),

        pass
        ? "PASS"
        : "FAIL",

        m_GenericInputLiveRouteEnabled
        ? "YES"
        : "NO",

        m_GenericInputLiveRouteEnabled
        ? "GENERIC_ACTIVE"
        : "LEGACY");


    return
        pass;
}


// ============================================================================
// Stage 11C.10 - Activate Legacy PLC DI/AD Input Cache Retirement
// ============================================================================

bool PlcCore::ActivateLegacyInputCacheRetirement()
{
    const bool genericRouteReady =
        m_GenericInputLiveRouteEnabled &&
        m_GenericInputLiveRoutePrepared &&
        m_pGenericReadMaster != nullptr;


    if (!genericRouteReady)
    {
        DEBUG_PRINT(
            "[PLC-LEGACY-INPUT-CACHE-RETIRE-ACTIVATE] "
            "GenericRouteReady:NO | Result:BLOCKED | "
            "LegacyCacheCopy:ACTIVE | "
            "Fallback:DIRECT_PROCESS_IMAGE_READY | "
            "PLCOutput:LEGACY | Motion:LEGACY | LiveOutputWrite:NO\n");

        return false;
    }


    bool expected =
        false;


    const bool transitioned =
        m_LegacyInputCacheRetired.compare_exchange_strong(
            expected,
            true,
            std::memory_order_relaxed,
            std::memory_order_relaxed);


    if (transitioned)
    {
        m_LegacyInputCacheSuppressedCycles.store(
            0ULL,
            std::memory_order_relaxed);

        m_DirectInputFallbackCycles.store(
            0ULL,
            std::memory_order_relaxed);

        m_DirectInputFallbackFailures.store(
            0ULL,
            std::memory_order_relaxed);
    }


    DEBUG_PRINT(
        "[PLC-LEGACY-INPUT-CACHE-RETIRE-ACTIVATE] "
        "GenericRouteReady:YES | Transition:%s | Result:PASS | "
        "LegacyCacheCopy:RETIRED | "
        "DIInBuffer:METADATA_ONLY | "
        "ADChannelValues:METADATA_ONLY | "
        "Fallback:DIRECT_PROCESS_IMAGE | "
        "PLCOutput:LEGACY | Motion:LEGACY | LiveOutputWrite:NO\n",

        transitioned
        ? "NOW"
        : "ALREADY_RETIRED");


    return true;
}


// ============================================================================
// ============================================================================
// Stage 11C.9 - PLC Generic Input Compatibility Retirement SHADOW
//
// This method is intentionally called only from the existing 1000ms
// supervisory loop in System_EDM_SINKER_MODE.cpp.
//
// The 1ms PLC bridge path only updates relaxed atomic counters.
//
// Qualification policy:
//     RouteEnabled == YES
//     LiveCycles   >= 5000
//     Fallback      == 0
//     ReadFailures  == 0
//
// 5000 cycles corresponds to approximately 5 seconds when the PLC bridge runs
// at its normal 1ms cadence.
// ============================================================================

void PlcCore::PrintGenericInputCompatibilityRetirementShadow()
{
    static constexpr uint64_t kMinimumQualifiedLiveCycles =
        5000ULL;


    const uint64_t liveCycles =
        m_GenericInputLiveCycles.load(
            std::memory_order_relaxed);

    const uint64_t fallbackCycles =
        m_GenericInputFallbackCycles.load(
            std::memory_order_relaxed);

    const uint64_t readFailures =
        m_GenericInputReadFailures.load(
            std::memory_order_relaxed);


    const bool routeEnabled =
        m_GenericInputLiveRouteEnabled &&
        m_GenericInputLiveRoutePrepared &&
        m_pGenericReadMaster != nullptr;


    const bool windowQualified =
        liveCycles >=
        kMinimumQualifiedLiveCycles;


    const bool cleanRuntime =
        fallbackCycles == 0ULL &&
        readFailures == 0ULL;


    const bool readyToRetireLegacyInput =
        routeEnabled &&
        windowQualified &&
        cleanRuntime;


    // Stage 11C.10:
    // Re-qualify on every boot before retiring legacy cache copies.
    if (readyToRetireLegacyInput &&
        !m_LegacyInputCacheRetired.load(
            std::memory_order_relaxed))
    {
        ActivateLegacyInputCacheRetirement();
    }


    const bool cacheRetired =
        m_LegacyInputCacheRetired.load(
            std::memory_order_relaxed);

    const uint64_t suppressedFetchCycles =
        m_LegacyInputCacheSuppressedCycles.load(
            std::memory_order_relaxed);

    const uint64_t directFallbackCycles =
        m_DirectInputFallbackCycles.load(
            std::memory_order_relaxed);

    const uint64_t directFallbackFailures =
        m_DirectInputFallbackFailures.load(
            std::memory_order_relaxed);


    DEBUG_PRINT(
        "[PLC-GENERIC-INPUT-RETIREMENT-SHADOW] "
        "RouteEnabled:%s | "
        "LiveCycles:%llu | "
        "FallbackCycles:%llu | "
        "ReadFailures:%llu | "
        "MinLiveCycles:%llu | "
        "Window:%s | RuntimeClean:%s | "
        "LegacyFetchInputs:%s | "
        "LegacyDIADBuffers:%s | "
        "PLCOutput:LEGACY | Motion:LEGACY | "
        "LiveOutputWrite:NO | "
        "ReadyToRetireLegacyInput:%s | "
        "CacheRetired:%s | Result:%s\n",

        routeEnabled
        ? "YES"
        : "NO",

        (unsigned long long)
        liveCycles,

        (unsigned long long)
        fallbackCycles,

        (unsigned long long)
        readFailures,

        (unsigned long long)
        kMinimumQualifiedLiveCycles,

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        cleanRuntime
        ? "YES"
        : "NO",

        cacheRetired
        ? "CACHE_COPY_RETIRED"
        : "COMPATIBILITY_ACTIVE",

        cacheRetired
        ? "METADATA_ONLY"
        : "STILL_UPDATED",

        readyToRetireLegacyInput
        ? "YES"
        : "NO",

        cacheRetired
        ? "YES"
        : "NO",

        readyToRetireLegacyInput &&
        cacheRetired
        ? "PASS"
        : "CHECK");


    DEBUG_PRINT(
        "[PLC-LEGACY-INPUT-CACHE-RETIRE-STATUS] "
        "Retired:%s | "
        "SuppressedFetchCycles:%llu | "
        "DirectFallbackCycles:%llu | "
        "DirectFallbackFailures:%llu | "
        "GenericLiveCycles:%llu | "
        "GenericFallbackCycles:%llu | "
        "GenericReadFailures:%llu | "
        "ActivePLCInput:GENERIC_CONSUMER_API | "
        "Fallback:DIRECT_PROCESS_IMAGE | "
        "PLCOutput:LEGACY | Motion:LEGACY | "
        "LiveOutputWrite:NO | Result:%s\n",

        cacheRetired
        ? "YES"
        : "NO",

        (unsigned long long)
        suppressedFetchCycles,

        (unsigned long long)
        directFallbackCycles,

        (unsigned long long)
        directFallbackFailures,

        (unsigned long long)
        liveCycles,

        (unsigned long long)
        fallbackCycles,

        (unsigned long long)
        readFailures,

        cacheRetired &&
        directFallbackFailures == 0ULL
        ? "PASS"
        : "CHECK");

    // Stage 11C.12 / 11C.13 diagnostics.
    PrintGenericOutputCommandBridgeShadow();


    // Stage 11C.14 controlled activation/status runs from the same
    // existing 1000ms supervisory path, never from the 1ms PLC loop.
    ProcessGenericOutputControlledCutover();


    // Stage 11C.15:
    // sustained ACTIVE Generic output compatibility retirement evidence.
    PrintGenericOutputCompatibilityRetirementShadow();


    // Stage 11C.16:
    // final aggregate PLC Generic I/O release checkpoint.
    PrintGenericPlcIoReleaseGate();


    // =========================================================
    // Servo INPUT diagnostics.
    //
    // Before Stage11D.10 release:
    //     retain the historical D2 -> D9 qualification chain.
    //
    // After release:
    //     retire that diagnostic wall and keep only the compact
    //     final Servo INPUT release status.
    // =========================================================

    if (m_pGenericReadMaster !=
        nullptr)
    {
        if (!m_pGenericReadMaster->
            IsServoInputReleaseComplete())
        {
            m_pGenericReadMaster->
                PrintStructuredServoDriveLiveReadShadow();
        }


        m_pGenericReadMaster->
            PrintServoInputReleaseGate();


        // Stage 11E.1:
        // independent Servo OUTPUT ownership shadow.
        m_pGenericReadMaster->
            PrintMotionServoOutputCommandOwnershipShadow();
    }
}


// ============================================================================
// Stage 11C.11 - Generic PLC Output Shadow Ownership Gate
//
// Active PLC output remains LEGACY.
//
// Generic output tests write ONLY to local scratch Process Image buffers.
// ============================================================================

bool PlcCore::PrepareGenericOutputOwnershipShadow()
{
    m_GenericDoSemanticOccurrences.clear();

    m_GenericOutputOwnershipShadowPrepared =
        false;


    int outputMaps =
        0;

    int preparedMaps =
        0;

    int ownedBits =
        0;

    int pointerMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-PREPARE] BEGIN | "
        "Stage:11C.11 | "
        "MapSource:IOMAP_CONFIG | "
        "SemanticKind:DigitalOutput | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | ScratchWrite:YES | "
        "Motion:LEGACY | ProductCodeBaseline:NO\n"
        "============================================================\n");


    if (m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr)
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (const auto& map :
            m_outputMaps)
        {
            outputMaps++;


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()) ||
                map.bitCount <= 0 ||
                map.bitCount > 64)
            {
                errors++;
                continue;
            }


            int semanticOccurrence =
                0;


            bool targetFound =
                false;


            // Physical output-module scan order defines semantic occurrence.
            for (int ioIndex = 0;
                ioIndex <
                static_cast<int>(
                    m_pIo->size());
                ++ioIndex)
            {
                const auto& io =
                    (*m_pIo)[
                        static_cast<size_t>(
                            ioIndex)];


                if (io.outBuffer.empty())
                {
                    continue;
                }


                if (ioIndex ==
                    map.listIdx)
                {
                    targetFound =
                        true;
                    break;
                }


                semanticOccurrence++;
            }


            const auto* descriptor =
                targetFound
                ? m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    semanticOccurrence)
                : nullptr;


            const auto& legacyIo =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            const uint8_t* legacyOutputBase =
                static_cast<const uint8_t*>(
                    legacyIo.pOutputLoc);


            const bool pointerMatch =
                descriptor != nullptr &&
                legacyOutputBase !=
                nullptr &&
                descriptor->pOutputByteBase ==
                legacyOutputBase;


            if (!targetFound ||
                descriptor ==
                nullptr ||
                descriptor->outputBitOffset <
                0 ||
                descriptor->outputBitLength !=
                static_cast<uint32_t>(
                    map.bitCount) ||
                descriptor->outputBitLength >
                64U ||
                descriptor->pOutputByteBase ==
                nullptr ||
                legacyIo.outBuffer.size() *
                8U !=
                static_cast<size_t>(
                    map.bitCount) ||
                !pointerMatch)
            {
                errors++;


                DEBUG_PRINT(
                    "[PLC-GENERIC-OUTPUT-OWNERSHIP-PREPARE-DO] "
                    "LegacyList:%d | UserOrder:%d | "
                    "Semantic:DigitalOutput/-/%d | "
                    "Resolved:%s | "
                    "PLC:O%d..O%d | Bits:%d | "
                    "Pointer:%s | Result:FAIL\n",

                    map.listIdx,
                    map.userOrder,
                    semanticOccurrence,

                    descriptor != nullptr
                    ? "YES"
                    : "NO",

                    map.plcStartIndex,
                    map.plcStartIndex +
                    map.bitCount -
                    1,
                    map.bitCount,

                    pointerMatch
                    ? "MATCH"
                    : "FAIL");


                continue;
            }


            m_GenericDoSemanticOccurrences.push_back(
                semanticOccurrence);


            preparedMaps++;
            ownedBits +=
                map.bitCount;
            pointerMatches++;


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-OWNERSHIP-PREPARE-DO] "
                "LegacyList:%d | UserOrder:%d | "
                "Semantic:DigitalOutput/-/%d | "
                "Resolved:S%d/%s | "
                "PLC:O%d..O%d | Bits:%d | "
                "OutBit:%d/%u | "
                "Pointer:MATCH | Result:PASS\n",

                map.listIdx,
                map.userOrder,
                semanticOccurrence,
                descriptor->slaveIndex,

                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,
                map.plcStartIndex +
                map.bitCount -
                1,
                map.bitCount,
                descriptor->outputBitOffset,

                (unsigned int)
                descriptor->outputBitLength);
        }
    }


    const bool pass =
        errors ==
        0 &&
        preparedMaps ==
        outputMaps &&
        m_GenericDoSemanticOccurrences.size() ==
        m_outputMaps.size();


    if (pass)
    {
        m_GenericOutputOwnershipShadowPrepared =
            true;
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-PREPARE-RESULT] "
        "OutputMaps:%d | PreparedMaps:%d | "
        "OwnedBits:%d | PointerMatches:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | ScratchWrite:YES | "
        "OwnershipAction:SHADOW_ONLY\n",

        outputMaps,
        preparedMaps,
        ownedBits,
        pointerMatches,
        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_GenericOutputOwnershipShadowPrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-PREPARE] END | "
        "Stage:11C.11 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool PlcCore::AuditGenericOutputOwnershipShadow()
{
    static constexpr uint32_t kScratchIoMapBytes =
        4096U;


    int outputMaps =
        0;

    int descriptorsClaimed =
        0;

    int ownershipBits =
        0;

    int plcOverlapErrors =
        0;

    int descriptorDuplicateErrors =
        0;

    int pointerChecks =
        0;

    int pointerMatches =
        0;

    int scratchCases =
        0;

    int scratchMatches =
        0;

    int livePreserveChecks =
        0;

    int livePreserveMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-SHADOW] BEGIN | "
        "Stage:11C.11 | "
        "SemanticKind:DigitalOutput | "
        "Ownership:EXCLUSIVE_SHADOW | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | ScratchWrite:YES | "
        "Motion:LEGACY | ProductCodeBaseline:NO\n"
        "============================================================\n");


    if (!m_GenericOutputOwnershipShadowPrepared ||
        m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        errors++;
    }


    std::set<int>
        ownedPlcBits;


    std::set<
        const EtherCatCompositeApplicationDescriptor*>
        claimedDescriptors;


    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_outputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_outputMaps[
                    mapIndex];


            outputMaps++;


            const int semanticOccurrence =
                m_GenericDoSemanticOccurrences[
                    mapIndex];


            const auto* descriptor =
                m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    semanticOccurrence);


            if (descriptor ==
                nullptr)
            {
                errors++;
                continue;
            }


            if (!claimedDescriptors.insert(
                descriptor)
                .second)
            {
                descriptorDuplicateErrors++;
                errors++;
            }
            else
            {
                descriptorsClaimed++;
            }


            for (int bit = 0;
                bit <
                map.bitCount;
                ++bit)
            {
                const int plcBit =
                    map.plcStartIndex +
                    bit;


                if (!ownedPlcBits.insert(
                    plcBit)
                    .second)
                {
                    plcOverlapErrors++;
                    errors++;
                }
                else
                {
                    ownershipBits++;
                }
            }


            const auto& legacyIo =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            const uint8_t* legacyOutputBase =
                static_cast<const uint8_t*>(
                    legacyIo.pOutputLoc);


            pointerChecks++;


            const bool pointerMatch =
                legacyOutputBase !=
                nullptr &&
                descriptor->pOutputByteBase ==
                legacyOutputBase;


            if (pointerMatch)
            {
                pointerMatches++;
            }
            else
            {
                errors++;
            }


            const uint32_t bitLength =
                descriptor->outputBitLength;


            if (descriptor->outputBitOffset <
                0 ||
                bitLength ==
                0U ||
                bitLength >
                64U)
            {
                errors++;
                continue;
            }


            const uint64_t mask =
                bitLength ==
                64U
                ? UINT64_MAX
                : (
                    (
                        1ULL <<
                        bitLength
                        ) -
                    1ULL
                    );


            const uint64_t patterns[3] =
            {
                0ULL,
                mask,
                0xA55AA55AA55AA55AULL &
                    mask
            };


            for (int caseIndex = 0;
                caseIndex <
                3;
                ++caseIndex)
            {
                uint8_t scratch[
                    kScratchIoMapBytes];


                std::fill(
                    scratch,
                    scratch +
                    kScratchIoMapBytes,
                    static_cast<uint8_t>(
                        0x5AU));


                uint64_t liveBefore =
                    0ULL;

                uint64_t liveAfter =
                    0ULL;


                const bool liveBeforePass =
                    m_pGenericReadMaster->
                    ReadCompositeOutputBitsShadow(
                        descriptor->slaveIndex,
                        descriptor->id,
                        liveBefore);


                const bool scratchWritePass =
                    m_pGenericReadMaster->
                    WriteCompositeOutputBitsToScratchShadow(
                        descriptor->slaveIndex,
                        descriptor->id,
                        patterns[
                            caseIndex],
                        scratch,
                                kScratchIoMapBytes);


                bool scratchIsolationPass =
                    scratchWritePass;


                const uint64_t ownedStart =
                    static_cast<uint64_t>(
                        descriptor->outputBitOffset);

                const uint64_t ownedEnd =
                    ownedStart +
                    static_cast<uint64_t>(
                        bitLength);


                if (ownedEnd >
                    static_cast<uint64_t>(
                        kScratchIoMapBytes) *
                    8ULL)
                {
                    scratchIsolationPass =
                        false;
                }


                if (scratchIsolationPass)
                {
                    for (uint32_t byteIndex = 0U;
                        byteIndex <
                        kScratchIoMapBytes &&
                        scratchIsolationPass;
                        ++byteIndex)
                    {
                        for (uint32_t bitIndex = 0U;
                            bitIndex <
                            8U;
                            ++bitIndex)
                        {
                            const uint64_t absoluteBit =
                                static_cast<uint64_t>(
                                    byteIndex) *
                                8ULL +
                                static_cast<uint64_t>(
                                    bitIndex);


                            const bool actualBit =
                                (
                                    scratch[
                                        byteIndex] &
                                    static_cast<uint8_t>(
                                        1U <<
                                        bitIndex)
                                            ) !=
                                0U;


                                        bool expectedBit =
                                            (
                                                static_cast<uint8_t>(
                                                    0x5AU) &
                                                static_cast<uint8_t>(
                                                    1U <<
                                                    bitIndex)
                                                ) !=
                                            0U;


                                        if (absoluteBit >=
                                            ownedStart &&
                                            absoluteBit <
                                            ownedEnd)
                                        {
                                            const uint32_t relativeBit =
                                                static_cast<uint32_t>(
                                                    absoluteBit -
                                                    ownedStart);


                                            expectedBit =
                                                (
                                                    (
                                                        patterns[
                                                            caseIndex] >>
                                                        relativeBit
                                                                ) &
                                                    0x01ULL
                                                                ) !=
                                                0ULL;
                                        }


                                        if (actualBit !=
                                            expectedBit)
                                        {
                                            scratchIsolationPass =
                                                false;
                                            break;
                                        }
                        }
                    }
                }


                const bool liveAfterPass =
                    m_pGenericReadMaster->
                    ReadCompositeOutputBitsShadow(
                        descriptor->slaveIndex,
                        descriptor->id,
                        liveAfter);


                const bool livePreserved =
                    liveBeforePass &&
                    liveAfterPass &&
                    liveBefore ==
                    liveAfter;


                scratchCases++;
                livePreserveChecks++;


                if (scratchIsolationPass)
                {
                    scratchMatches++;
                }
                else
                {
                    errors++;
                }


                if (livePreserved)
                {
                    livePreserveMatches++;
                }
                else
                {
                    errors++;
                }


                DEBUG_PRINT(
                    "[PLC-GENERIC-OUTPUT-OWNERSHIP-SCRATCH] "
                    "Map:%u | Semantic:DigitalOutput/-/%d | "
                    "Resolved:S%d/%s | "
                    "Case:%d Value:0x%llX | "
                    "ScratchMask:%s | "
                    "LiveBefore:0x%llX LiveAfter:0x%llX | "
                    "LivePreserved:%s | Result:%s\n",

                    (unsigned int)
                    mapIndex,
                    semanticOccurrence,
                    descriptor->slaveIndex,

                    descriptor->id[0] != '\0'
                    ? descriptor->id
                    : "N/A",

                    caseIndex,

                    (unsigned long long)
                    patterns[
                        caseIndex],

                    scratchIsolationPass
                            ? "PASS"
                            : "FAIL",

                            (unsigned long long)
                            liveBefore,

                            (unsigned long long)
                            liveAfter,

                            livePreserved
                            ? "YES"
                            : "NO",

                            scratchIsolationPass &&
                            livePreserved
                            ? "PASS"
                            : "FAIL");
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-OWNERSHIP-DO] "
                "Map:%u | LegacyList:%d | UserOrder:%d | "
                "Semantic:DigitalOutput/-/%d | "
                "Resolved:S%d/%s | "
                "PLC:O%d..O%d | "
                "OwnedBits:%d | Pointer:%s | "
                "Result:%s\n",

                (unsigned int)
                mapIndex,
                map.listIdx,
                map.userOrder,
                semanticOccurrence,
                descriptor->slaveIndex,

                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,
                map.plcStartIndex +
                map.bitCount -
                1,
                map.bitCount,

                pointerMatch
                ? "MATCH"
                : "FAIL",

                pointerMatch
                ? "PASS"
                : "FAIL");
        }
    }


    const bool pass =
        errors ==
        0;


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-SHADOW-RESULT] "
        "OutputMaps:%d | "
        "DescriptorsClaimed:%d | "
        "OwnershipBits:%d | "
        "PLCOverlapErrors:%d | "
        "DescriptorDuplicateErrors:%d | "
        "PointerChecks:%d PointerMatches:%d | "
        "ScratchCases:%d ScratchMatches:%d | "
        "LivePreserveChecks:%d LivePreserveMatches:%d | "
        "Errors:%d | Result:%s | "
        "SemanticKind:DigitalOutput | "
        "Ownership:EXCLUSIVE_SHADOW | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | ScratchWrite:YES | "
        "Motion:LEGACY | ProductCodeBaseline:NO | "
        "GateAction:%s | StartupAction:CONTINUE\n",

        outputMaps,
        descriptorsClaimed,
        ownershipBits,
        plcOverlapErrors,
        descriptorDuplicateErrors,
        pointerChecks,
        pointerMatches,
        scratchCases,
        scratchMatches,
        livePreserveChecks,
        livePreserveMatches,
        errors,

        pass
        ? "PASS"
        : "FAIL",

        pass
        ? "ALLOW_NEXT_OUTPUT_SHADOW_STAGE"
        : "BLOCK_OUTPUT_CUTOVER");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-OWNERSHIP-SHADOW] END | "
        "Stage:11C.11 | Result:%s | "
        "LiveWrite:NO | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


// ============================================================================
// Stage 11C.12 - Generic PLC Output Command Bridge SHADOW
// ============================================================================

bool PlcCore::PrepareGenericOutputCommandBridgeShadow()
{
    m_GenericOutputCommandBridgeShadowEnabled =
        false;

    m_GenericOutputCommandBridgeMapCount =
        0;


    m_GenericOutputCommandShadowCycles.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputCommandShadowMapChecks.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputCommandShadowMatches.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputCommandShadowMismatches.store(
        0ULL,
        std::memory_order_relaxed);


    int maps =
        0;

    int semanticRoutes =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-PREPARE] BEGIN | "
        "Stage:11C.12 | "
        "Source:PLC_O | CompareTo:LEGACY_OUTBUFFER | "
        "SemanticKind:DigitalOutput | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "BridgeAction:SHADOW_ONLY\n"
        "============================================================\n");


    if (!m_GenericOutputOwnershipShadowPrepared ||
        m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_outputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_outputMaps[
                    mapIndex];


            maps++;


            const int semanticOccurrence =
                m_GenericDoSemanticOccurrences[
                    mapIndex];


            const auto* descriptor =
                m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    semanticOccurrence);


            if (descriptor ==
                nullptr ||
                descriptor->outputBitLength !=
                static_cast<uint32_t>(
                    map.bitCount) ||
                descriptor->outputBitLength ==
                0U ||
                descriptor->outputBitLength >
                64U)
            {
                errors++;


                DEBUG_PRINT(
                    "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-PREPARE-MAP] "
                    "Map:%u | Semantic:DigitalOutput/-/%d | "
                    "Resolved:%s | PLC:O%d..O%d | Bits:%d | "
                    "Result:FAIL\n",

                    (unsigned int)
                    mapIndex,

                    semanticOccurrence,

                    descriptor != nullptr
                    ? "YES"
                    : "NO",

                    map.plcStartIndex,

                    map.plcStartIndex +
                    map.bitCount -
                    1,

                    map.bitCount);


                continue;
            }


            semanticRoutes++;


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-PREPARE-MAP] "
                "Map:%u | Semantic:DigitalOutput/-/%d | "
                "Resolved:S%d/%s | "
                "PLC:O%d..O%d | Bits:%d | "
                "Result:PASS\n",

                (unsigned int)
                mapIndex,

                semanticOccurrence,

                descriptor->slaveIndex,

                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,

                map.plcStartIndex +
                map.bitCount -
                1,

                map.bitCount);
        }
    }


    const bool pass =
        errors ==
        0 &&
        maps ==
        static_cast<int>(
            m_outputMaps.size()) &&
        semanticRoutes ==
        maps;


    if (pass)
    {
        m_GenericOutputCommandBridgeMapCount =
            maps;

        m_GenericOutputCommandBridgeShadowEnabled =
            true;
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-PREPARE-RESULT] "
        "Maps:%d | SemanticRoutes:%d | "
        "Errors:%d | Result:%s | Enabled:%s | "
        "Source:PLC_O | CompareTo:LEGACY_OUTBUFFER | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "BridgeAction:SHADOW_ONLY\n",

        maps,

        semanticRoutes,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_GenericOutputCommandBridgeShadowEnabled
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-PREPARE] END | "
        "Stage:11C.12 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void PlcCore::PrintGenericOutputCommandBridgeShadow() const
{
    static constexpr uint64_t kMinimumQualifiedCycles =
        5000ULL;


    const uint64_t cycles =
        m_GenericOutputCommandShadowCycles.load(
            std::memory_order_relaxed);


    const uint64_t mapChecks =
        m_GenericOutputCommandShadowMapChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_GenericOutputCommandShadowMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_GenericOutputCommandShadowMismatches.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        cycles *
        static_cast<uint64_t>(
            m_GenericOutputCommandBridgeMapCount);


    const bool windowQualified =
        cycles >=
        kMinimumQualifiedCycles;


    const bool countsConsistent =
        mapChecks ==
        expectedChecks &&
        matches +
        mismatches ==
        mapChecks;


    const bool commandClean =
        mismatches ==
        0ULL &&
        countsConsistent;


    const bool readyForNextStage =
        m_GenericOutputCommandBridgeShadowEnabled &&
        windowQualified &&
        commandClean;


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-COMMAND-BRIDGE-SHADOW] "
        "Enabled:%s | "
        "Maps:%d | "
        "Cycles:%llu | "
        "MapChecks:%llu ExpectedChecks:%llu | "
        "Matches:%llu | Mismatches:%llu | "
        "MinCycles:%llu | "
        "Window:%s | CountsConsistent:%s | CommandClean:%s | "
        "Source:PLC_O | CompareTo:LEGACY_OUTBUFFER | "
        "SemanticKind:DigitalOutput | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "BridgeAction:SHADOW_ONLY | "
        "ReadyForNextOutputStage:%s | Result:%s\n",

        m_GenericOutputCommandBridgeShadowEnabled
        ? "YES"
        : "NO",

        m_GenericOutputCommandBridgeMapCount,

        (unsigned long long)
        cycles,

        (unsigned long long)
        mapChecks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        (unsigned long long)
        kMinimumQualifiedCycles,

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        countsConsistent
        ? "YES"
        : "NO",

        commandClean
        ? "YES"
        : "NO",

        readyForNextStage
        ? "YES"
        : "NO",

        readyForNextStage
        ? "PASS"
        : "CHECK");

    // Stage 11C.13: same existing 1000ms supervisory path.
    PrintGenericOutputLiveWritePreflightShadow();
}


// ============================================================================
// Stage 11C.13 - Generic PLC Output Live-Write Preflight SHADOW
//
// SAFETY ARCHITECTURE
// -------------------
//
// Current and future-safe physical commit ownership:
//
//     PLC 1ms
//         -> shadow output command
//
//     PDO 250us
//         -> FlushOutputs()
//         -> pOutputLoc
//
// Stage11C.13 does NOT create a second pOutputLoc writer.
//
// Future Generic command cutover is allowed to replace only the command
// generation inside SyncVirtualToPhysical().  The physical commit point remains
// FlushOutputs() in the PDO cycle.
// ============================================================================

bool PlcCore::PrepareGenericOutputLiveWritePreflightShadow()
{
    m_GenericOutputLiveWritePreflightPrepared =
        false;

    m_GenericOutputLiveWritePreflightAuditPassed =
        false;

    m_GenericOutputLiveWritePreflightMapCount =
        0;

    m_GenericOutputLiveWritePreflightRollbackChecks =
        0;

    m_GenericOutputLiveWritePreflightRollbackMatches =
        0;

    m_GenericOutputLiveWritePreflightLivePreserveChecks =
        0;

    m_GenericOutputLiveWritePreflightLivePreserveMatches =
        0;


    int maps =
        0;

    int semanticRoutes =
        0;

    int pointerMatches =
        0;

    int shadowBufferMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-PREPARE] BEGIN | "
        "Stage:11C.13 | "
        "CommandStage:PLC_SYNC_VIRTUAL_TO_PHYSICAL | "
        "PhysicalCommit:PDO_FLUSH_OUTPUTS | "
        "PhysicalWriterCount:1 | "
        "PlannedGenericTarget:SHADOW_OUTBUFFER | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "Motion:LEGACY\n"
        "============================================================\n");


    if (!m_GenericOutputOwnershipShadowPrepared ||
        !m_GenericOutputCommandBridgeShadowEnabled ||
        m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_outputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_outputMaps[
                    mapIndex];


            maps++;


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()) ||
                map.bitCount <= 0 ||
                map.bitCount > 64)
            {
                errors++;
                continue;
            }


            const int occurrence =
                m_GenericDoSemanticOccurrences[
                    mapIndex];


            const auto* descriptor =
                m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    occurrence);


            const auto& legacyIo =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            const uint8_t* liveOutput =
                static_cast<const uint8_t*>(
                    legacyIo.pOutputLoc);


            const bool pointerMatch =
                descriptor != nullptr &&
                liveOutput !=
                nullptr &&
                descriptor->pOutputByteBase ==
                liveOutput;


            const bool shadowBufferMatch =
                descriptor != nullptr &&
                !legacyIo.outBuffer.empty() &&
                legacyIo.outBuffer.size() *
                8U ==
                static_cast<size_t>(
                    descriptor->outputBitLength) &&
                descriptor->outputBitLength ==
                static_cast<uint32_t>(
                    map.bitCount);


            if (descriptor != nullptr)
            {
                semanticRoutes++;
            }


            if (pointerMatch)
            {
                pointerMatches++;
            }


            if (shadowBufferMatch)
            {
                shadowBufferMatches++;
            }


            const bool mapPass =
                descriptor !=
                nullptr &&
                descriptor->outputBitOffset >=
                0 &&
                descriptor->outputBitLength >
                0U &&
                descriptor->outputBitLength <=
                64U &&
                pointerMatch &&
                shadowBufferMatch;


            if (!mapPass)
            {
                errors++;
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-MAP] "
                "Map:%u | "
                "Semantic:DigitalOutput/-/%d | "
                "Resolved:S%d/%s | "
                "PLC:O%d..O%d | "
                "Bits:%d | "
                "ShadowTarget:outBuffer | "
                "PhysicalTarget:pOutputLoc | "
                "Pointer:%s | Envelope:%s | "
                "PhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
                "Result:%s\n",

                (unsigned int)
                mapIndex,

                occurrence,

                descriptor != nullptr
                ? descriptor->slaveIndex
                : -1,

                descriptor != nullptr &&
                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,

                map.plcStartIndex +
                map.bitCount -
                1,

                map.bitCount,

                pointerMatch
                ? "MATCH"
                : "FAIL",

                shadowBufferMatch
                ? "MATCH"
                : "FAIL",

                mapPass
                ? "PASS"
                : "FAIL");
        }
    }


    const bool pass =
        errors ==
        0 &&
        maps ==
        static_cast<int>(
            m_outputMaps.size()) &&
        semanticRoutes ==
        maps &&
        pointerMatches ==
        maps &&
        shadowBufferMatches ==
        maps;


    if (pass)
    {
        m_GenericOutputLiveWritePreflightPrepared =
            true;

        m_GenericOutputLiveWritePreflightMapCount =
            maps;
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-PREPARE-RESULT] "
        "Maps:%d | "
        "SemanticRoutes:%d | "
        "PointerMatches:%d | "
        "ShadowBufferMatches:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "SinglePhysicalWriter:YES | "
        "PhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
        "CommandStage:PLC_SYNC_VIRTUAL_TO_PHYSICAL | "
        "PlannedGenericTarget:SHADOW_OUTBUFFER | "
        "ActivePLCOutput:LEGACY | GenericLiveWrite:NO\n",

        maps,

        semanticRoutes,

        pointerMatches,

        shadowBufferMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_GenericOutputLiveWritePreflightPrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-PREPARE] END | "
        "Stage:11C.13 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


bool PlcCore::AuditGenericOutputLiveWritePreflightShadow()
{
    static constexpr uint32_t kScratchIoMapBytes =
        4096U;


    int maps =
        0;

    int rollbackChecks =
        0;

    int rollbackMatches =
        0;

    int livePreserveChecks =
        0;

    int livePreserveMatches =
        0;

    int timingChecks =
        0;

    int timingMatches =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-AUDIT] BEGIN | "
        "Stage:11C.13 | "
        "Mode:SHADOW_ONLY | "
        "SinglePhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
        "CommandStage:PLC_SYNC_VIRTUAL_TO_PHYSICAL | "
        "Rollback:LEGACY_COMMAND_TO_SAME_OUTBUFFER | "
        "GenericLiveWrite:NO\n"
        "============================================================\n");


    if (!m_GenericOutputLiveWritePreflightPrepared ||
        m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_outputMaps.size();
            ++mapIndex)
        {
            maps++;


            const auto& map =
                m_outputMaps[
                    mapIndex];


            const int occurrence =
                m_GenericDoSemanticOccurrences[
                    mapIndex];


            const auto* descriptor =
                m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    occurrence);


            if (descriptor ==
                nullptr ||
                map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()) ||
                descriptor->outputBitLength ==
                0U ||
                descriptor->outputBitLength >
                64U)
            {
                errors++;
                continue;
            }


            const auto& legacyIo =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            uint64_t legacyCommand =
                0ULL;


            for (int bit = 0;
                bit <
                map.bitCount;
                ++bit)
            {
                const int bytePos =
                    bit /
                    8;


                const int bitPos =
                    bit %
                    8;


                const bool value =
                    (
                        legacyIo.outBuffer[
                            static_cast<size_t>(
                                bytePos)] &
                        static_cast<uint8_t>(
                            1U <<
                            bitPos)
                                ) !=
                    0U;


                            if (value)
                            {
                                legacyCommand |=
                                    1ULL <<
                                    bit;
                            }
            }


            const uint32_t bitLength =
                descriptor->outputBitLength;


            const uint64_t mask =
                bitLength ==
                64U
                ? UINT64_MAX
                : (
                    (
                        1ULL <<
                        bitLength
                        ) -
                    1ULL
                    );


            const uint64_t candidateCommand =
                (
                    ~legacyCommand
                    ) &
                mask;


            uint8_t scratch[
                kScratchIoMapBytes];


            std::fill(
                scratch,
                scratch +
                kScratchIoMapBytes,
                static_cast<uint8_t>(
                    0x5AU));


            uint64_t liveBefore =
                0ULL;


            uint64_t liveAfter =
                0ULL;


            const bool liveBeforePass =
                m_pGenericReadMaster->
                ReadCompositeOutputBitsShadow(
                    descriptor->slaveIndex,
                    descriptor->id,
                    liveBefore);


            const bool candidateWritePass =
                m_pGenericReadMaster->
                WriteCompositeOutputBitsToScratchShadow(
                    descriptor->slaveIndex,
                    descriptor->id,
                    candidateCommand,
                    scratch,
                    kScratchIoMapBytes);


            const bool rollbackWritePass =
                candidateWritePass &&
                m_pGenericReadMaster->
                WriteCompositeOutputBitsToScratchShadow(
                    descriptor->slaveIndex,
                    descriptor->id,
                    legacyCommand,
                    scratch,
                    kScratchIoMapBytes);


            bool rollbackPass =
                rollbackWritePass;


            if (rollbackPass)
            {
                const uint64_t startBit =
                    static_cast<uint64_t>(
                        descriptor->outputBitOffset);


                const uint64_t endBit =
                    startBit +
                    static_cast<uint64_t>(
                        bitLength);


                if (endBit >
                    static_cast<uint64_t>(
                        kScratchIoMapBytes) *
                    8ULL)
                {
                    rollbackPass =
                        false;
                }


                for (uint32_t byteIndex = 0U;
                    byteIndex <
                    kScratchIoMapBytes &&
                    rollbackPass;
                    ++byteIndex)
                {
                    for (uint32_t bitIndex = 0U;
                        bitIndex <
                        8U;
                        ++bitIndex)
                    {
                        const uint64_t absoluteBit =
                            static_cast<uint64_t>(
                                byteIndex) *
                            8ULL +
                            static_cast<uint64_t>(
                                bitIndex);


                        const bool actual =
                            (
                                scratch[
                                    byteIndex] &
                                static_cast<uint8_t>(
                                    1U <<
                                    bitIndex)
                                        ) !=
                            0U;


                                    bool expected =
                                        (
                                            static_cast<uint8_t>(
                                                0x5AU) &
                                            static_cast<uint8_t>(
                                                1U <<
                                                bitIndex)
                                            ) !=
                                        0U;


                                    if (absoluteBit >=
                                        startBit &&
                                        absoluteBit <
                                        endBit)
                                    {
                                        const uint32_t relativeBit =
                                            static_cast<uint32_t>(
                                                absoluteBit -
                                                startBit);


                                        expected =
                                            (
                                                (
                                                    legacyCommand >>
                                                    relativeBit
                                                    ) &
                                                0x01ULL
                                                ) !=
                                            0ULL;
                                    }


                                    if (actual !=
                                        expected)
                                    {
                                        rollbackPass =
                                            false;

                                        break;
                                    }
                    }
                }
            }


            const bool liveAfterPass =
                m_pGenericReadMaster->
                ReadCompositeOutputBitsShadow(
                    descriptor->slaveIndex,
                    descriptor->id,
                    liveAfter);


            const bool livePreserved =
                liveBeforePass &&
                liveAfterPass &&
                liveBefore ==
                liveAfter;


            // Timing plan is intentionally one command owner and one
            // physical commit owner:
            //
            // 1ms PLC:
            //     SyncVirtualToPhysical -> outBuffer
            //
            // 250us PDO:
            //     FlushOutputs -> pOutputLoc
            //
            // Stage11C.13 changes neither path.
            const bool timingPass =
                descriptor->pOutputByteBase ==
                static_cast<const uint8_t*>(
                    legacyIo.pOutputLoc) &&
                !legacyIo.outBuffer.empty();


            rollbackChecks++;
            livePreserveChecks++;
            timingChecks++;


            if (rollbackPass)
            {
                rollbackMatches++;
            }
            else
            {
                errors++;
            }


            if (livePreserved)
            {
                livePreserveMatches++;
            }
            else
            {
                errors++;
            }


            if (timingPass)
            {
                timingMatches++;
            }
            else
            {
                errors++;
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-MAP-AUDIT] "
                "Map:%u | "
                "Semantic:DigitalOutput/-/%d | "
                "Resolved:S%d/%s | "
                "LegacyCmd:0x%llX CandidateCmd:0x%llX | "
                "CandidateScratch:%s | "
                "Rollback:%s | "
                "LiveBefore:0x%llX LiveAfter:0x%llX | "
                "LivePreserved:%s | "
                "Timing:PLC_SHADOW_THEN_PDO_COMMIT_%s | "
                "Result:%s\n",

                (unsigned int)
                mapIndex,

                occurrence,

                descriptor->slaveIndex,

                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                (unsigned long long)
                legacyCommand,

                (unsigned long long)
                candidateCommand,

                candidateWritePass
                ? "PASS"
                : "FAIL",

                rollbackPass
                ? "PASS"
                : "FAIL",

                (unsigned long long)
                liveBefore,

                (unsigned long long)
                liveAfter,

                livePreserved
                ? "YES"
                : "NO",

                timingPass
                ? "PASS"
                : "FAIL",

                rollbackPass &&
                livePreserved &&
                timingPass
                ? "PASS"
                : "FAIL");
        }
    }


    const bool pass =
        errors ==
        0 &&
        maps ==
        m_GenericOutputLiveWritePreflightMapCount &&
        rollbackChecks ==
        maps &&
        rollbackMatches ==
        maps &&
        livePreserveChecks ==
        maps &&
        livePreserveMatches ==
        maps &&
        timingChecks ==
        maps &&
        timingMatches ==
        maps;


    m_GenericOutputLiveWritePreflightRollbackChecks =
        rollbackChecks;

    m_GenericOutputLiveWritePreflightRollbackMatches =
        rollbackMatches;

    m_GenericOutputLiveWritePreflightLivePreserveChecks =
        livePreserveChecks;

    m_GenericOutputLiveWritePreflightLivePreserveMatches =
        livePreserveMatches;

    m_GenericOutputLiveWritePreflightAuditPassed =
        pass;


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-AUDIT-RESULT] "
        "Maps:%d | "
        "RollbackChecks:%d RollbackMatches:%d | "
        "LivePreserveChecks:%d LivePreserveMatches:%d | "
        "TimingChecks:%d TimingMatches:%d | "
        "Errors:%d | Result:%s | "
        "SinglePhysicalWriter:YES | "
        "PhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
        "CommandStage:PLC_SYNC_VIRTUAL_TO_PHYSICAL | "
        "RollbackRoute:LEGACY_COMMAND_TO_SAME_OUTBUFFER | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "GateAction:%s | StartupAction:CONTINUE\n",

        maps,

        rollbackChecks,

        rollbackMatches,

        livePreserveChecks,

        livePreserveMatches,

        timingChecks,

        timingMatches,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        pass
        ? "WAIT_STAGE11C12_RUNTIME_QUALIFICATION"
        : "BLOCK_OUTPUT_CUTOVER");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-AUDIT] END | "
        "Stage:11C.13 | Result:%s | ShadowOnly:YES\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void PlcCore::PrintGenericOutputLiveWritePreflightShadow() const
{
    static constexpr uint64_t kMinimumQualifiedCommandCycles =
        5000ULL;


    const uint64_t cycles =
        m_GenericOutputCommandShadowCycles.load(
            std::memory_order_relaxed);


    const uint64_t mapChecks =
        m_GenericOutputCommandShadowMapChecks.load(
            std::memory_order_relaxed);


    const uint64_t matches =
        m_GenericOutputCommandShadowMatches.load(
            std::memory_order_relaxed);


    const uint64_t mismatches =
        m_GenericOutputCommandShadowMismatches.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        cycles *
        static_cast<uint64_t>(
            m_GenericOutputCommandBridgeMapCount);


    const bool runtimeQualified =
        m_GenericOutputCommandBridgeShadowEnabled &&
        cycles >=
        kMinimumQualifiedCommandCycles &&
        mapChecks ==
        expectedChecks &&
        matches ==
        mapChecks &&
        mismatches ==
        0ULL;


    const bool staticPreflightPass =
        m_GenericOutputLiveWritePreflightPrepared &&
        m_GenericOutputLiveWritePreflightAuditPassed &&
        m_GenericOutputLiveWritePreflightMapCount ==
        m_GenericOutputCommandBridgeMapCount &&
        m_GenericOutputLiveWritePreflightRollbackChecks ==
        m_GenericOutputLiveWritePreflightRollbackMatches &&
        m_GenericOutputLiveWritePreflightLivePreserveChecks ==
        m_GenericOutputLiveWritePreflightLivePreserveMatches;


    const bool readyForControlledCutover =
        runtimeQualified &&
        staticPreflightPass;


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-LIVEWRITE-PREFLIGHT-SHADOW] "
        "Prepared:%s | Audit:%s | "
        "Maps:%d | "
        "CommandCycles:%llu | "
        "MapChecks:%llu ExpectedChecks:%llu | "
        "Matches:%llu Mismatches:%llu | "
        "RuntimeWindow:%s | "
        "Rollback:%d/%d | "
        "LivePreserve:%d/%d | "
        "SinglePhysicalWriter:YES | "
        "PhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
        "CommandStage:PLC_SYNC_VIRTUAL_TO_PHYSICAL | "
        "PlannedGenericTarget:SHADOW_OUTBUFFER | "
        "RollbackRoute:LEGACY_COMMAND_TO_SAME_OUTBUFFER | "
        "ActivePLCOutput:LEGACY | "
        "GenericLiveWrite:NO | "
        "Motion:LEGACY | "
        "ReadyForControlledOutputCutover:%s | "
        "Result:%s\n",

        m_GenericOutputLiveWritePreflightPrepared
        ? "YES"
        : "NO",

        m_GenericOutputLiveWritePreflightAuditPassed
        ? "PASS"
        : "FAIL",

        m_GenericOutputLiveWritePreflightMapCount,

        (unsigned long long)
        cycles,

        (unsigned long long)
        mapChecks,

        (unsigned long long)
        expectedChecks,

        (unsigned long long)
        matches,

        (unsigned long long)
        mismatches,

        runtimeQualified
        ? "QUALIFIED"
        : "WARMUP",

        m_GenericOutputLiveWritePreflightRollbackMatches,

        m_GenericOutputLiveWritePreflightRollbackChecks,

        m_GenericOutputLiveWritePreflightLivePreserveMatches,

        m_GenericOutputLiveWritePreflightLivePreserveChecks,

        readyForControlledCutover
        ? "YES"
        : "NO",

        readyForControlledCutover
        ? "PASS"
        : "CHECK");
}


// ============================================================================
// Stage 11C.14 - Controlled Generic PLC DigitalOutput Live Command Cutover
//
// SAFETY MODEL
// ------------
//
// PLC 1ms command producer:
//     LEGACY Set_O()  ->  GENERIC semantic command encoder
//
// Physical Process Image writer:
//     UNCHANGED
//     FlushOutputs() remains the ONLY pOutputLoc writer.
//
// Activation:
//     only after Stage11C.13 is qualified on the CURRENT boot.
//
// Fault behavior:
//     one-way latch back to legacy Set_O() for the remainder of the boot.
// ============================================================================

bool PlcCore::PrepareGenericOutputControlledCutover()
{
    m_GenericOutputControlledCutoverPrepared =
        false;

    m_GenericOutputControlledCutoverMapCount =
        0;

    m_GenericOutputControlledCutoverEnabled.store(
        false,
        std::memory_order_relaxed);

    m_GenericOutputControlledCutoverFaulted.store(
        false,
        std::memory_order_relaxed);

    m_GenericOutputControlledLiveCycles.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputControlledMapWrites.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputControlledFallbacks.store(
        0ULL,
        std::memory_order_relaxed);

    m_GenericOutputControlledRouteFaults.store(
        0ULL,
        std::memory_order_relaxed);


    int maps =
        0;

    int semanticRoutes =
        0;

    int wholeBufferOwners =
        0;

    int errors =
        0;


    DEBUG_PRINT(
        "\n"
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-CUTOVER-PREPARE] BEGIN | "
        "Stage:11C.14 | "
        "Activation:RUNTIME_QUALIFIED | "
        "CommandTarget:OUTBUFFER | "
        "PhysicalWriter:PDO_FLUSH_OUTPUTS_ONLY | "
        "Fallback:LEGACY_SET_O | "
        "ServoMotion:LEGACY\n"
        "============================================================\n");


    if (!m_GenericOutputOwnershipShadowPrepared ||
        !m_GenericOutputCommandBridgeShadowEnabled ||
        !m_GenericOutputLiveWritePreflightPrepared ||
        !m_GenericOutputLiveWritePreflightAuditPassed ||
        m_pGenericReadMaster ==
        nullptr ||
        m_pIo ==
        nullptr ||
        m_GenericDoSemanticOccurrences.size() !=
        m_outputMaps.size())
    {
        errors++;
    }


    if (errors ==
        0)
    {
        for (size_t mapIndex = 0;
            mapIndex <
            m_outputMaps.size();
            ++mapIndex)
        {
            const auto& map =
                m_outputMaps[
                    mapIndex];


            maps++;


            if (map.listIdx < 0 ||
                map.listIdx >=
                static_cast<int>(
                    m_pIo->size()) ||
                map.bitCount <= 0 ||
                map.bitCount > 64 ||
                (map.bitCount % 8) != 0)
            {
                errors++;
                continue;
            }


            const int occurrence =
                m_GenericDoSemanticOccurrences[
                    mapIndex];


            const auto* descriptor =
                m_pGenericReadMaster->
                FindCompositeBindingByKindAxisShadow(
                    "DigitalOutput",
                    "",
                    occurrence);


            auto& io =
                (*m_pIo)[
                    static_cast<size_t>(
                        map.listIdx)];


            if (descriptor !=
                nullptr)
            {
                semanticRoutes++;
            }


            const uint8_t* physicalBase =
                static_cast<const uint8_t*>(
                    io.pOutputLoc);


            const bool wholeBufferOwner =
                descriptor !=
                nullptr &&
                descriptor->outputBitOffset >=
                0 &&
                descriptor->outputBitLength ==
                static_cast<uint32_t>(
                    map.bitCount) &&
                descriptor->outputBitLength ==
                io.outBuffer.size() *
                8U &&
                descriptor->outputByteAligned &&
                descriptor->outputBitShift ==
                0U &&
                descriptor->outputByteSpan ==
                io.outBuffer.size() &&
                descriptor->pOutputByteBase ==
                physicalBase &&
                physicalBase !=
                nullptr;


            if (wholeBufferOwner)
            {
                wholeBufferOwners++;
            }
            else
            {
                errors++;
            }


            DEBUG_PRINT(
                "[PLC-GENERIC-OUTPUT-CUTOVER-MAP] "
                "Map:%u | DO/-/%d | S%d/%s | "
                "PLC:O%d..O%d | Bits:%d | "
                "WholeBuffer:%s | ByteAligned:%s | "
                "Target:outBuffer | Result:%s\n",

                (unsigned int)
                mapIndex,

                occurrence,

                descriptor != nullptr
                ? descriptor->slaveIndex
                : -1,

                descriptor != nullptr &&
                descriptor->id[0] != '\0'
                ? descriptor->id
                : "N/A",

                map.plcStartIndex,

                map.plcStartIndex +
                map.bitCount -
                1,

                map.bitCount,

                wholeBufferOwner
                ? "YES"
                : "NO",

                descriptor != nullptr &&
                descriptor->outputByteAligned
                ? "YES"
                : "NO",

                wholeBufferOwner
                ? "PASS"
                : "FAIL");
        }
    }


    const bool pass =
        errors ==
        0 &&
        maps ==
        static_cast<int>(
            m_outputMaps.size()) &&
        semanticRoutes ==
        maps &&
        wholeBufferOwners ==
        maps;


    if (pass)
    {
        m_GenericOutputControlledCutoverMapCount =
            maps;

        m_GenericOutputControlledCutoverPrepared =
            true;
    }


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-CUTOVER-PREPARE-RESULT] "
        "Maps:%d | Routes:%d | WholeBuffer:%d | "
        "Errors:%d | Result:%s | Prepared:%s | "
        "Active:LEGACY_WAIT_GATE | "
        "Target:OUTBUFFER | PhysicalWriter:FLUSH_ONLY\n",

        maps,

        semanticRoutes,

        wholeBufferOwners,

        errors,

        pass
        ? "PASS"
        : "FAIL",

        m_GenericOutputControlledCutoverPrepared
        ? "YES"
        : "NO");


    DEBUG_PRINT(
        "============================================================\n"
        "[PLC-GENERIC-OUTPUT-CUTOVER-PREPARE] END | "
        "Stage:11C.14 | Result:%s\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}


void PlcCore::ProcessGenericOutputControlledCutover()
{
    static constexpr uint64_t kMinimumQualifiedCommandCycles =
        5000ULL;


    const uint64_t commandCycles =
        m_GenericOutputCommandShadowCycles.load(
            std::memory_order_relaxed);


    const uint64_t commandChecks =
        m_GenericOutputCommandShadowMapChecks.load(
            std::memory_order_relaxed);


    const uint64_t commandMatches =
        m_GenericOutputCommandShadowMatches.load(
            std::memory_order_relaxed);


    const uint64_t commandMismatches =
        m_GenericOutputCommandShadowMismatches.load(
            std::memory_order_relaxed);


    const uint64_t expectedChecks =
        commandCycles *
        static_cast<uint64_t>(
            m_GenericOutputCommandBridgeMapCount);


    const bool runtimeQualified =
        m_GenericOutputCommandBridgeShadowEnabled &&
        commandCycles >=
        kMinimumQualifiedCommandCycles &&
        commandChecks ==
        expectedChecks &&
        commandMatches ==
        commandChecks &&
        commandMismatches ==
        0ULL;


    const bool staticPreflightPass =
        m_GenericOutputLiveWritePreflightPrepared &&
        m_GenericOutputLiveWritePreflightAuditPassed &&
        m_GenericOutputLiveWritePreflightMapCount ==
        m_GenericOutputCommandBridgeMapCount &&
        m_GenericOutputLiveWritePreflightRollbackChecks ==
        m_GenericOutputLiveWritePreflightRollbackMatches &&
        m_GenericOutputLiveWritePreflightLivePreserveChecks ==
        m_GenericOutputLiveWritePreflightLivePreserveMatches;


    const bool faulted =
        m_GenericOutputControlledCutoverFaulted.load(
            std::memory_order_acquire);


    const bool activationGate =
        m_GenericOutputControlledCutoverPrepared &&
        runtimeQualified &&
        staticPreflightPass &&
        !faulted;


    bool enabled =
        m_GenericOutputControlledCutoverEnabled.load(
            std::memory_order_acquire);


    if (!enabled &&
        activationGate)
    {
        m_GenericOutputControlledLiveCycles.store(
            0ULL,
            std::memory_order_relaxed);

        m_GenericOutputControlledMapWrites.store(
            0ULL,
            std::memory_order_relaxed);

        m_GenericOutputControlledFallbacks.store(
            0ULL,
            std::memory_order_relaxed);

        m_GenericOutputControlledRouteFaults.store(
            0ULL,
            std::memory_order_relaxed);


        m_GenericOutputControlledCutoverEnabled.store(
            true,
            std::memory_order_release);


        enabled =
            true;


        DEBUG_PRINT(
            "[PLC-GENERIC-OUTPUT-CUTOVER-ACTIVATE] "
            "Gate:PASS | Transition:NOW | "
            "RouteEnabled:YES | "
            "ActivePLCOutput:GENERIC_TO_OUTBUFFER | "
            "PhysicalWriter:FLUSH_ONLY | "
            "Fallback:LEGACY_SET_O | "
            "ServoMotion:LEGACY | Result:PASS\n");
    }


    const uint64_t liveCycles =
        m_GenericOutputControlledLiveCycles.load(
            std::memory_order_relaxed);


    const uint64_t mapWrites =
        m_GenericOutputControlledMapWrites.load(
            std::memory_order_relaxed);


    const uint64_t fallbacks =
        m_GenericOutputControlledFallbacks.load(
            std::memory_order_relaxed);


    const uint64_t routeFaults =
        m_GenericOutputControlledRouteFaults.load(
            std::memory_order_relaxed);


    const bool currentFaulted =
        m_GenericOutputControlledCutoverFaulted.load(
            std::memory_order_acquire);


    enabled =
        m_GenericOutputControlledCutoverEnabled.load(
            std::memory_order_acquire);


    const bool liveHealthy =
        enabled &&
        !currentFaulted &&
        fallbacks ==
        0ULL &&
        routeFaults ==
        0ULL;


    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-CUTOVER] "
        "Prepared:%s | Gate:%s | Enabled:%s | Fault:%s | "
        "Live:%llu Writes:%llu Fallback:%llu RouteFault:%llu | "
        "Active:%s | PhysicalWriter:FLUSH_ONLY | Result:%s\n",

        m_GenericOutputControlledCutoverPrepared
        ? "YES"
        : "NO",

        activationGate
        ? "PASS"
        : "WAIT",

        enabled
        ? "YES"
        : "NO",

        currentFaulted
        ? "YES"
        : "NO",

        (unsigned long long)
        liveCycles,

        (unsigned long long)
        mapWrites,

        (unsigned long long)
        fallbacks,

        (unsigned long long)
        routeFaults,

        enabled
        ? "GENERIC_TO_OUTBUFFER"
        : "LEGACY_SET_O",

        liveHealthy
        ? "PASS"
        : (
            currentFaulted
            ? "FALLBACK"
            : "CHECK"
            ));
}


// ============================================================================
// Stage 11C.15 - Generic PLC DigitalOutput Compatibility Retirement SHADOW
//
// This stage changes NO output behavior.
//
// It observes the Stage11C.14 ACTIVE Generic command route after activation.
// The counters are already reset to zero exactly when Stage11C.14 transitions
// from LEGACY_SET_O to GENERIC_TO_OUTBUFFER.
//
// Qualification:
//     Enabled        == YES
//     LiveCycles     >= 5000
//     MapWrites      == LiveCycles * MapCount
//     Fallbacks      == 0
//     RouteFaults    == 0
//     Fault          == NO
//
// Legacy Set_O remains compiled as one-way emergency fallback only.
// ============================================================================

void PlcCore::PrintGenericOutputCompatibilityRetirementShadow() const
{
    static constexpr uint64_t kMinimumQualifiedLiveCycles =
        5000ULL;


    const bool prepared =
        m_GenericOutputControlledCutoverPrepared;


    const bool enabled =
        m_GenericOutputControlledCutoverEnabled.load(
            std::memory_order_acquire);


    const bool faulted =
        m_GenericOutputControlledCutoverFaulted.load(
            std::memory_order_acquire);


    const uint64_t liveCycles =
        m_GenericOutputControlledLiveCycles.load(
            std::memory_order_relaxed);


    const uint64_t mapWrites =
        m_GenericOutputControlledMapWrites.load(
            std::memory_order_relaxed);


    const uint64_t fallbacks =
        m_GenericOutputControlledFallbacks.load(
            std::memory_order_relaxed);


    const uint64_t routeFaults =
        m_GenericOutputControlledRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t expectedWrites =
        liveCycles *
        static_cast<uint64_t>(
            m_GenericOutputControlledCutoverMapCount);


    const bool windowQualified =
        liveCycles >=
        kMinimumQualifiedLiveCycles;


    const bool countsConsistent =
        m_GenericOutputControlledCutoverMapCount >
        0 &&
        mapWrites ==
        expectedWrites;


    const bool runtimeClean =
        !faulted &&
        fallbacks ==
        0ULL &&
        routeFaults ==
        0ULL;


    const bool readyToRetireLegacyNormalProducer =
        prepared &&
        enabled &&
        windowQualified &&
        countsConsistent &&
        runtimeClean;


    // Keep this line intentionally short so it remains readable even when
    // the older DC runtime diagnostic output is disabled.
    DEBUG_PRINT(
        "[PLC-GENERIC-OUTPUT-RETIREMENT-SHADOW] "
        "Prepared:%s Enabled:%s Fault:%s | "
        "Live:%llu/%llu | Writes:%llu Expected:%llu | "
        "Fallback:%llu RouteFault:%llu | "
        "Window:%s Counts:%s Clean:%s | "
        "LegacySetO:EMERGENCY_ONLY | PhysicalWriter:FLUSH_ONLY | "
        "Ready:%s Result:%s\n",

        prepared
        ? "YES"
        : "NO",

        enabled
        ? "YES"
        : "NO",

        faulted
        ? "YES"
        : "NO",

        (unsigned long long)
        liveCycles,

        (unsigned long long)
        kMinimumQualifiedLiveCycles,

        (unsigned long long)
        mapWrites,

        (unsigned long long)
        expectedWrites,

        (unsigned long long)
        fallbacks,

        (unsigned long long)
        routeFaults,

        windowQualified
        ? "QUALIFIED"
        : "WARMUP",

        countsConsistent
        ? "YES"
        : "NO",

        runtimeClean
        ? "YES"
        : "NO",

        readyToRetireLegacyNormalProducer
        ? "YES"
        : "NO",

        readyToRetireLegacyNormalProducer
        ? "PASS"
        : "CHECK");
}


// ============================================================================
// Stage 11C.16 - Generic PLC I/O Release Gate
//
// Diagnostic only.
//
// PASS means:
// - PLC DI / AD Generic input route is qualified and legacy input cache retired.
// - PLC DO Generic output route is qualified and active.
// - All output ownership / equivalence / preflight gates remain proven.
// - No input/output fallback or route fault has occurred.
//
// No Process Image write.
// No route activation.
// No hardware access.
// ============================================================================

void PlcCore::PrintGenericPlcIoReleaseGate() const
{
    static constexpr uint64_t kMinimumInputLiveCycles =
        5000ULL;

    static constexpr uint64_t kMinimumOutputShadowCycles =
        5000ULL;

    static constexpr uint64_t kMinimumOutputLiveCycles =
        5000ULL;


    // ------------------------------------------------------------------------
    // INPUT
    // ------------------------------------------------------------------------

    const uint64_t inputLiveCycles =
        m_GenericInputLiveCycles.load(
            std::memory_order_relaxed);

    const uint64_t inputFallbackCycles =
        m_GenericInputFallbackCycles.load(
            std::memory_order_relaxed);

    const uint64_t inputReadFailures =
        m_GenericInputReadFailures.load(
            std::memory_order_relaxed);

    const uint64_t directFallbackCycles =
        m_DirectInputFallbackCycles.load(
            std::memory_order_relaxed);

    const uint64_t directFallbackFailures =
        m_DirectInputFallbackFailures.load(
            std::memory_order_relaxed);


    const bool inputRouteReady =
        m_GenericInputLiveRoutePrepared &&
        m_GenericInputLiveRouteEnabled &&
        m_pGenericReadMaster !=
        nullptr;


    const bool inputCacheRetired =
        m_LegacyInputCacheRetired.load(
            std::memory_order_relaxed);


    const bool inputRelease =
        inputRouteReady &&
        inputCacheRetired &&
        inputLiveCycles >=
        kMinimumInputLiveCycles &&
        inputFallbackCycles ==
        0ULL &&
        inputReadFailures ==
        0ULL &&
        directFallbackCycles ==
        0ULL &&
        directFallbackFailures ==
        0ULL;


    // ------------------------------------------------------------------------
    // OUTPUT - qualified shadow / static evidence
    // ------------------------------------------------------------------------

    const uint64_t outputShadowCycles =
        m_GenericOutputCommandShadowCycles.load(
            std::memory_order_relaxed);

    const uint64_t outputShadowChecks =
        m_GenericOutputCommandShadowMapChecks.load(
            std::memory_order_relaxed);

    const uint64_t outputShadowMatches =
        m_GenericOutputCommandShadowMatches.load(
            std::memory_order_relaxed);

    const uint64_t outputShadowMismatches =
        m_GenericOutputCommandShadowMismatches.load(
            std::memory_order_relaxed);


    const uint64_t expectedOutputShadowChecks =
        outputShadowCycles *
        static_cast<uint64_t>(
            m_GenericOutputCommandBridgeMapCount);


    const bool outputOwnershipReady =
        m_GenericOutputOwnershipShadowPrepared;


    const bool outputCommandBridgeQualified =
        m_GenericOutputCommandBridgeShadowEnabled &&
        m_GenericOutputCommandBridgeMapCount >
        0 &&
        outputShadowCycles >=
        kMinimumOutputShadowCycles &&
        outputShadowChecks ==
        expectedOutputShadowChecks &&
        outputShadowMatches ==
        outputShadowChecks &&
        outputShadowMismatches ==
        0ULL;


    const bool outputPreflightReady =
        m_GenericOutputLiveWritePreflightPrepared &&
        m_GenericOutputLiveWritePreflightAuditPassed &&
        m_GenericOutputLiveWritePreflightMapCount ==
        m_GenericOutputCommandBridgeMapCount &&
        m_GenericOutputLiveWritePreflightRollbackChecks ==
        m_GenericOutputLiveWritePreflightRollbackMatches &&
        m_GenericOutputLiveWritePreflightLivePreserveChecks ==
        m_GenericOutputLiveWritePreflightLivePreserveMatches;


    // ------------------------------------------------------------------------
    // OUTPUT - ACTIVE live evidence
    // ------------------------------------------------------------------------

    const bool outputEnabled =
        m_GenericOutputControlledCutoverEnabled.load(
            std::memory_order_acquire);

    const bool outputFaulted =
        m_GenericOutputControlledCutoverFaulted.load(
            std::memory_order_acquire);

    const uint64_t outputLiveCycles =
        m_GenericOutputControlledLiveCycles.load(
            std::memory_order_relaxed);

    const uint64_t outputMapWrites =
        m_GenericOutputControlledMapWrites.load(
            std::memory_order_relaxed);

    const uint64_t outputFallbacks =
        m_GenericOutputControlledFallbacks.load(
            std::memory_order_relaxed);

    const uint64_t outputRouteFaults =
        m_GenericOutputControlledRouteFaults.load(
            std::memory_order_relaxed);


    const uint64_t expectedOutputMapWrites =
        outputLiveCycles *
        static_cast<uint64_t>(
            m_GenericOutputControlledCutoverMapCount);


    const bool outputRelease =
        outputOwnershipReady &&
        outputCommandBridgeQualified &&
        outputPreflightReady &&
        m_GenericOutputControlledCutoverPrepared &&
        outputEnabled &&
        !outputFaulted &&
        m_GenericOutputControlledCutoverMapCount >
        0 &&
        outputLiveCycles >=
        kMinimumOutputLiveCycles &&
        outputMapWrites ==
        expectedOutputMapWrites &&
        outputFallbacks ==
        0ULL &&
        outputRouteFaults ==
        0ULL;


    // ------------------------------------------------------------------------
    // FINAL
    // ------------------------------------------------------------------------

    const bool releasePass =
        inputRelease &&
        outputRelease;


    DEBUG_PRINT(
        "[PLC-GENERIC-IO-RELEASE] "
        "Input:%s Live:%llu Cache:%s IFallback:%llu IFail:%llu "
        "DirectFallback:%llu/%llu | "
        "Output:%s Shadow:%llu/%llu Preflight:%s "
        "Live:%llu/%llu Writes:%llu/%llu "
        "OFallback:%llu OFault:%llu Enabled:%s | "
        "PhysicalWriter:FLUSH_ONLY | "
        "LegacyInputCache:RETIRED | "
        "LegacySetO:EMERGENCY_ONLY | "
        "ServoMotion:%s | "
        "Release:%s Result:%s\n",

        inputRelease
        ? "PASS"
        : "WAIT",

        (unsigned long long)
        inputLiveCycles,

        inputCacheRetired
        ? "RETIRED"
        : "ACTIVE",

        (unsigned long long)
        inputFallbackCycles,

        (unsigned long long)
        inputReadFailures,

        (unsigned long long)
        directFallbackCycles,

        (unsigned long long)
        directFallbackFailures,

        outputRelease
        ? "PASS"
        : "WAIT",

        (unsigned long long)
        outputShadowCycles,

        (unsigned long long)
        kMinimumOutputShadowCycles,

        outputPreflightReady
        ? "PASS"
        : "WAIT",

        (unsigned long long)
        outputLiveCycles,

        (unsigned long long)
        kMinimumOutputLiveCycles,

        (unsigned long long)
        outputMapWrites,

        (unsigned long long)
        expectedOutputMapWrites,

        (unsigned long long)
        outputFallbacks,

        (unsigned long long)
        outputRouteFaults,

        outputEnabled
        ? "YES"
        : "NO",

        m_pGenericReadMaster !=
        nullptr &&
        m_pGenericReadMaster->
        IsServoGenericIoReleaseComplete()
        ? "GENERIC_SERVO_IO"
        : "QUALIFYING",

        releasePass
        ? "GENERIC_PLC_IO_COMPLETE"
        : "WAIT",

        releasePass
        ? "PASS"
        : "CHECK");
    // ========================================================================
    // Stage 11F.1 - Unified Generic I/O Release Candidate Gate
    //
    // Final machine-level aggregate acceptance only.
    //
    // This gate does NOT:
    // - change PLC or Servo command routing
    // - write Process Image data
    // - add another LRW/FRMW transaction
    // - change DC / RX / scheduler control
    //
    // It only reads already-published state from the existing 1000ms
    // supervisory diagnostic path and latches one Release Candidate result.
    // ========================================================================

    EtherCatMaster* const master =
        m_pGenericReadMaster;


    bool processImagePass =
        false;

    int runtimeSlaveCount =
        0;

    int processImageBindingCount =
        0;


    if (master != nullptr &&
        master->m_pEni != nullptr)
    {
        const auto& runtimeSlaves =
            master->m_pEni->GetSlaves();


        runtimeSlaveCount =
            static_cast<int>(
                runtimeSlaves.size());


        for (const auto& slave :
            runtimeSlaves)
        {
            if (slave.runtimeProcessImageBinding.present)
            {
                processImageBindingCount++;
            }
        }


        processImagePass =
            runtimeSlaveCount > 0 &&
            processImageBindingCount ==
            runtimeSlaveCount &&
            master->m_runtimeEquivalenceVerified &&
            master->m_IoMapSize > 0 &&
            master->m_IoMapSize <=
            static_cast<int>(
                sizeof(master->m_IoMap));
    }


    const bool servoReleasePass =
        master != nullptr &&
        master->IsServoGenericIoReleaseComplete();


    const EtherCatRuntimeSnapshot runtimeSnapshot =
        master != nullptr
        ? master->GetRuntimeSnapshot()
        : EtherCatRuntimeSnapshot{};


    const bool runtimePass =
        master != nullptr &&
        runtimeSnapshot.stage ==
        EtherCatRuntimeStage::Running &&
        runtimeSnapshot.result ==
        EtherCatRuntimeResult::Pass &&
        runtimeSnapshot.errorCode ==
        EcatRuntimeOk;


    // ------------------------------------------------------------------------
    // Read the same Priority-64 diagnostic snapshots used by the established
    // DC health summary.  Three attempts avoid treating a concurrent publish
    // (odd seqlock sequence) as a real machine failure.
    // ------------------------------------------------------------------------

    bool runtimeSnapshotValid =
        false;

    LONG realState =
        0;

    LONG phaseGood =
        0;

    LONG tripMask =
        0;

    LONG phasePState =
        0;

    LONG phasePGate =
        0;

    LONG phasePOffsetSat =
        0;

    LONGLONG actualErrNs =
        0;

    LONGLONG totalRecover =
        0;

    LONGLONG totalSkip =
        0;

    LONGLONG totalSoftLate =
        0;

    LONGLONG totalHardTimeout =
        0;

    LONG currentConsecutiveTimeout =
        0;

    LONG rxQpcFail =
        0;

    LONG lrwWkc =
        0;

    LONG dcWkc =
        0;

    LONG driftState =
        0;


    for (int attempt = 0;
        attempt < 3;
        ++attempt)
    {
        const LONG realSeqBefore =
            g_qpcRealFfV0Seq;

        const LONG phaseSeqBefore =
            g_qpcPhasePActV0Seq;

        const LONG recoverySeqBefore =
            g_pdoBootstrapDiagSequence;

        const LONG timeoutSeqBefore =
            g_ecatRxDiagSequence;

        const LONG wkcSeqBefore =
            g_pdoRtDiagSequence;

        const LONG driftSeqBefore =
            g_dcDriftCalibrationDiagSequence;


        if (realSeqBefore == 0 ||
            phaseSeqBefore == 0 ||
            recoverySeqBefore == 0 ||
            timeoutSeqBefore == 0 ||
            wkcSeqBefore == 0 ||
            (realSeqBefore & 1) != 0 ||
            (phaseSeqBefore & 1) != 0 ||
            (recoverySeqBefore & 1) != 0 ||
            (timeoutSeqBefore & 1) != 0 ||
            (wkcSeqBefore & 1) != 0 ||
            (driftSeqBefore & 1) != 0)
        {
            continue;
        }


        MemoryBarrier();


        realState =
            g_qpcRealFfV0State;

        phaseGood =
            g_qpcRealFfV0PhaseGood;

        tripMask =
            g_qpcRealFfV0TripMask;

        phasePState =
            g_qpcPhasePActV0State;

        phasePGate =
            g_qpcPhasePActV0GateGood;

        phasePOffsetSat =
            g_qpcPhasePActV0OffsetSat;

        actualErrNs =
            g_qpcPhasePActV0ActualErrNs;

        totalRecover =
            g_pdoRuntimeRecoveryTotalEvents;

        totalSkip =
            g_pdoRuntimeRecoveryTotalSkippedCycles;

        totalSoftLate =
            g_ecatRxDiagTotalSoftLateAccepted;

        totalHardTimeout =
            g_ecatRxDiagTotalHardTimeout;

        currentConsecutiveTimeout =
            g_ecatRxDiagCurrentConsecutiveTimeout;

        rxQpcFail =
            g_ecatRxDiagQpcFail;

        lrwWkc =
            g_pdoRtLrwWkc;

        dcWkc =
            g_pdoRtDcWkc;

        driftState =
            g_dcDriftCalibrationState;


        MemoryBarrier();


        const LONG realSeqAfter =
            g_qpcRealFfV0Seq;

        const LONG phaseSeqAfter =
            g_qpcPhasePActV0Seq;

        const LONG recoverySeqAfter =
            g_pdoBootstrapDiagSequence;

        const LONG timeoutSeqAfter =
            g_ecatRxDiagSequence;

        const LONG wkcSeqAfter =
            g_pdoRtDiagSequence;

        const LONG driftSeqAfter =
            g_dcDriftCalibrationDiagSequence;


        if (realSeqBefore != realSeqAfter ||
            phaseSeqBefore != phaseSeqAfter ||
            recoverySeqBefore != recoverySeqAfter ||
            timeoutSeqBefore != timeoutSeqAfter ||
            wkcSeqBefore != wkcSeqAfter ||
            driftSeqBefore != driftSeqAfter ||
            (realSeqAfter & 1) != 0 ||
            (phaseSeqAfter & 1) != 0 ||
            (recoverySeqAfter & 1) != 0 ||
            (timeoutSeqAfter & 1) != 0 ||
            (wkcSeqAfter & 1) != 0 ||
            (driftSeqAfter & 1) != 0)
        {
            continue;
        }


        runtimeSnapshotValid =
            true;

        break;
    }


    // ------------------------------------------------------------------------
    // RX health uses the same five-minute Recent_Timeout semantics as
    // [DC-HEALTH-SUMMARY].  TotalHardTimeout remains historical evidence;
    // an isolated event can recover only after a full quiet window.
    // ------------------------------------------------------------------------

    static constexpr LONG
        kRecentTimeoutResetSeconds =
        300;

    static bool
        rcTimeoutWindowInitialized =
        false;

    static LONGLONG
        rcTimeoutWindowLastTotal =
        0;

    static LONGLONG
        rcTimeoutWindowRecent =
        0;

    static LONG
        rcTimeoutWindowQuietSeconds =
        kRecentTimeoutResetSeconds;


    if (runtimeSnapshotValid)
    {
        if (!rcTimeoutWindowInitialized ||
            totalHardTimeout <
            rcTimeoutWindowLastTotal)
        {
            rcTimeoutWindowInitialized =
                true;

            rcTimeoutWindowLastTotal =
                totalHardTimeout;

            rcTimeoutWindowRecent =
                totalHardTimeout;

            rcTimeoutWindowQuietSeconds =
                totalHardTimeout > 0
                ? 0
                : kRecentTimeoutResetSeconds;
        }
        else
        {
            const LONGLONG newTimeout =
                totalHardTimeout -
                rcTimeoutWindowLastTotal;


            rcTimeoutWindowLastTotal =
                totalHardTimeout;


            if (newTimeout > 0)
            {
                rcTimeoutWindowRecent +=
                    newTimeout;

                rcTimeoutWindowQuietSeconds =
                    0;
            }
            else if (rcTimeoutWindowRecent > 0)
            {
                if (rcTimeoutWindowQuietSeconds <
                    kRecentTimeoutResetSeconds)
                {
                    rcTimeoutWindowQuietSeconds++;
                }


                if (rcTimeoutWindowQuietSeconds >=
                    kRecentTimeoutResetSeconds)
                {
                    rcTimeoutWindowRecent =
                        0;

                    rcTimeoutWindowQuietSeconds =
                        kRecentTimeoutResetSeconds;
                }
            }
        }
    }


    const int expectedLrwWkc =
        master != nullptr
        ? master->EXPECTED_WKC_PDO
        : 0;


    const bool lrwPass =
        runtimeSnapshotValid &&
        expectedLrwWkc > 0 &&
        lrwWkc == expectedLrwWkc;


    const int dcReferenceSlaveIndex =
        GetDcReferenceSlaveIndex();


    const bool dcWkcPass =
        runtimeSnapshotValid &&
        (
            dcReferenceSlaveIndex < 0 ||
            dcWkc > 0
            );


    const bool rxPass =
        runtimeSnapshotValid &&
        currentConsecutiveTimeout == 0 &&
        rxQpcFail == 0 &&
        totalRecover == 0 &&
        totalSkip == 0 &&
        totalSoftLate == 0 &&
        rcTimeoutWindowRecent == 0;


    const bool nearZero =
        actualErrNs >= -1000 &&
        actualErrNs <= 1000;


    const bool dcPass =
        runtimeSnapshotValid &&
        (driftState == 1 ||
            driftState == 2) &&
        realState == 2 &&
        phaseGood != 0 &&
        phasePState == 2 &&
        phasePGate != 0 &&
        phasePOffsetSat == 0 &&
        nearZero &&
        tripMask == 0;


    const bool transportPass =
        lrwPass &&
        dcWkcPass &&
        rxPass &&
        dcPass;


    const bool unifiedCurrentPass =
        releasePass &&
        servoReleasePass &&
        processImagePass &&
        runtimePass &&
        transportPass;


    // Diagnostic-only boot-local latch.  This records that the full RC gate
    // has been satisfied at least once.  Ready/Result still reflect CURRENT
    // health, so a later runtime problem is never hidden by the latch.
    static bool
        unifiedReleaseLatched =
        false;

    static uint64_t
        unifiedReleaseTransitions =
        0ULL;


    if (!unifiedReleaseLatched &&
        unifiedCurrentPass)
    {
        unifiedReleaseLatched =
            true;

        unifiedReleaseTransitions++;
    }


    const bool unifiedReady =
        unifiedReleaseLatched &&
        unifiedCurrentPass;


    DEBUG_PRINT(
        "[GENERIC-IO-RC-GATE] "
        "Stage:11F.1 | "
        "Released:%s Transition:%llu | "
        "PLC:%s Servo:%s | "
        "ProcessImage:%s Bindings:%d/%d Bytes:%d Cutover:%s | "
        "LRW:%s WKC:%ld/%d DCWKC:%ld | "
        "RX:%s TimeoutRecent:%lld Quiet:%ld/%lds Consecutive:%ld "
        "Recover:%lld Skip:%lld SoftLate:%lld QpcFail:%ld | "
        "DC:%s Drift:%ld RealFF:%ld PhaseGood:%s PhaseP:%ld Gate:%s "
        "Err:%+lldns OffsetSat:%s TripMask:0x%02lX | "
        "Runtime:%s Snapshot:%s | "
        "Release:UNIFIED_GENERIC_IO_RC_COMPLETE | "
        "Ready:%s Result:%s | Action:DIAGNOSTIC_ONLY\n",

        unifiedReleaseLatched
        ? "YES"
        : "NO",

        (unsigned long long)
        unifiedReleaseTransitions,

        releasePass
        ? "PASS"
        : "WAIT",

        servoReleasePass
        ? "PASS"
        : "WAIT",

        processImagePass
        ? "PASS"
        : "WAIT",

        processImageBindingCount,

        runtimeSlaveCount,

        master != nullptr
        ? master->m_IoMapSize
        : 0,

        master != nullptr &&
        master->m_runtimeEquivalenceVerified
        ? "OPEN"
        : "CLOSED",

        lrwPass
        ? "PASS"
        : "CHECK",

        (long)
        lrwWkc,

        expectedLrwWkc,

        (long)
        dcWkc,

        rxPass
        ? "PASS"
        : "CHECK",

        (long long)
        rcTimeoutWindowRecent,

        (long)
        rcTimeoutWindowQuietSeconds,

        (long)
        kRecentTimeoutResetSeconds,

        (long)
        currentConsecutiveTimeout,

        (long long)
        totalRecover,

        (long long)
        totalSkip,

        (long long)
        totalSoftLate,

        (long)
        rxQpcFail,

        dcPass
        ? "PASS"
        : "CHECK",

        (long)
        driftState,

        (long)
        realState,

        phaseGood
        ? "YES"
        : "NO",

        (long)
        phasePState,

        phasePGate
        ? "YES"
        : "NO",

        (long long)
        actualErrNs,

        phasePOffsetSat
        ? "YES"
        : "NO",

        (long)
        tripMask,

        runtimePass
        ? "PASS"
        : "CHECK",

        runtimeSnapshotValid
        ? "VALID"
        : "BUSY",

        unifiedReady
        ? "YES"
        : "NO",

        unifiedReady
        ? "PASS"
        : "CHECK");


}
