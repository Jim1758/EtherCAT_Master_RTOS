#include "NCManager.h"
#include "MacroEngine.h"
#include "MacroParser.h"
#include "GCodeParser.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include "AlarmManager.h"
#include "GMCodeHandlers.h" // 🌟 引入 G 碼處理器總表
#include <fstream>
#include <iostream>

NCManager::NCManager(MotionCore& motion) : m_motion(motion), MathParser(MacroSys), Parser(MathParser)
{
    // 初始化設定
    m_state = NCState::IDLE;
    m_mode = NCOperationMode::MEMORY; // 預設記憶體模式
}

bool NCManager::LoadProgram(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    //自動萃取檔名 (去掉資料夾路徑，只留 test.nc)
    size_t pos = filepath.find_last_of("/\\");
    m_mainProgramName = (pos != std::string::npos) ? filepath.substr(pos + 1) : filepath;

    // 🌟 載入新主程式時，清空所有的副程式與區域變數
    m_macroStack.clear();
    MacroSys.Reset();

    // UI 顯示歸零
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    m_programMemory.clear();
    m_jumpTable.clear(); // 清空舊的跳躍表
    m_programPC = 0;
    std::string line;

    int lineIndex = 0;
    while (std::getline(file, line)) {
        m_programMemory.push_back(line);

        // 🌟 快速掃描 N 碼，建立跳躍表
        std::string clean = MacroParser::CleanExpression(line);
        size_t nPos = clean.find('N');
        if (nPos != std::string::npos) {
            // 將 N 後面的數字轉為整數 (例如 N10 -> 10)
            int nVal = std::atoi(clean.c_str() + nPos + 1);
            m_jumpTable[nVal] = lineIndex;
        }
        lineIndex++;
    }
    file.close();

    DEBUG_PRINT("[NC] Program Loaded, Lines: %d\n", (int)m_programMemory.size());
    m_state = NCState::READY;
    return true;
}

void NCManager::ChangeMode(NCOperationMode newMode)
{
    // 只有在 IDLE 或 READY 狀態才能切換模式
    if (m_state == NCState::IDLE || m_state == NCState::READY) {
        m_mode = newMode;
    }
}

void NCManager::ChangeState(NCState newState) {
    m_state = newState;
}

void NCManager::CycleStart()
{
    if (m_state == NCState::READY || m_state == NCState::HOLD)
    {
        m_state = NCState::RUN;
        DEBUG_PRINT("[NC] Cycle Start!\n");
    }
}

void NCManager::FeedHold()
{
    if (m_state == NCState::RUN)
    {
        m_state = NCState::HOLD;
        // 呼叫 MotionCore 的群組減速停止
        // m_motion.StopGroup(..., 0.5); 
    }
}

void NCManager::Reset()
{
    AlarmManager::GetInstance().Clear();//清除Alarm訊息

    // 🌟 新增：重置巨集引擎，把堆疊清空退回主程式，並銷毀所有副程式的區域變數
    MacroSys.Reset();

    // 🌟 確保主程式的狀態也完全重置
    m_macroStack.clear();
    m_programPC = 0;
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    m_state = NCState::RESET_STATE;
    std::queue<NCBlock> empty;

    // 將指令佇列清空
    std::swap(m_blockQueue, empty);

    // m_motion.EmergencyStopGroup(); 



    // 2. 🌟 解決卡住的元凶：清空所有等待中的回呼！
    m_waitCallback = nullptr;

    // 3. 🌟 清空計時器
    GCodeHandlers::Reset_G04(this); // 將 G04 相關參數歸零





    m_state = NCState::READY;
}

// ==========================================
// 🌟 1. 實作呼叫副程式邏輯 (升級為 8 層架構)
// ==========================================
bool NCManager::CallMacro(const std::string& filename) {
    // 🌟 嘗試推入變數堆疊，如果失敗代表超過 8 層！
    if (MacroSys.PushCallStack() == false) {
        DEBUG_PRINT("[Alarm] Macro Call Depth Exceeded 8 Layers!\n");
        AlarmManager::GetInstance().Trigger(AlarmManager::MACRO_OVERFLOW);
        m_state = NCState::HOLD;
        return false;
    }

    std::string fullPath = "D:\\EtherCAT_Master_Data\\NC_Macro\\" + filename;
    std::ifstream file(fullPath);
    if (!file.is_open()) {
        DEBUG_PRINT("[Alarm] Macro File Not Found: %s\n", fullPath.c_str());
        AlarmManager::GetInstance().Trigger(AlarmManager::Macro_File_Not_Found);
        m_state = NCState::HOLD;
        return false;
    }

    // 🌟 建立這層副程式的專屬執行框架 (Frame)
    MacroFrame newFrame;
    newFrame.programName = filename;
    newFrame.currentPC = 0;

    // 計算 M99 返回時的行號 (如果是第1層副程式就取主程式 PC+1，如果是第2層就取第1層 PC+1)
    newFrame.returnPC = m_macroStack.empty() ? (m_programPC + 1) : (m_macroStack.back().currentPC + 1);

    std::string line;
    while (std::getline(file, line)) {
        newFrame.memory.push_back(line);
    }
    file.close();

    // 🌟 將這層副程式推入堆疊頂端
    m_macroStack.push_back(newFrame);

    m_programChanged = true; // 告訴系統剛切換了程式，不要把舊 PC + 1
    return true;
}

// ==========================================
// 🌟 2. 實作返回主程式邏輯
// ==========================================
void NCManager::ReturnMacro() 
{
    if (m_macroStack.empty()) return; // 防呆

    // 取出這一層原本預定要回傳的行號
    int retPC = m_macroStack.back().returnPC;

    // 🌟 彈出副程式堆疊，並同時銷毀這一層專屬的 #1~#100 區域變數
    m_macroStack.pop_back();
    MacroSys.PopCallStack();

    // 將行號還給上一層 (如果堆疊空了代表回到主程式)
    if (m_macroStack.empty()) {
        m_programPC = retPC;
    }
    else {
        m_macroStack.back().currentPC = retPC;
    }

    m_programChanged = true; // 告訴系統發生了跳轉
}

void NCManager::PushBlock(const NCBlock& block) {
    // 略過單節跳躍
    if (block.isBlockSkip /* && 系統開啟了單節跳躍開關 */) return;

    m_blockQueue.push(block);
}

// 🌟 (測試用) M 碼專用的檢查函式
static bool CheckMCodeDone(NCManager* nc) {
    int ticks = nc->GetSimulatedTicks() - 1;
    nc->SetSimulatedTicks(ticks);
    if (ticks > 0) {
        DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
        return false;
    }
    return true;
}

// 🌟 放在 RTOS 迴圈的核心任務
void NCManager::ProcessTask()
{
    NC_RunCount++; // NC執行迴圈數
    // 1. 檢查警報狀態
    if (AlarmManager::GetInstance().HasAlarm())
    {
        m_state = NCState::ALARM;
        return;
    }

    if (m_state == NCState::ALARM || m_state != NCState::RUN) return;

    if (m_mode == NCOperationMode::MEMORY)
    {
        // ==========================================
        // 🌟 智慧判斷：目前是在跑主程式，還是副程式？
        // ==========================================
        bool isMacro = !m_macroStack.empty();

        // 永遠將指標綁定在「最頂層」的 PC 與記憶體
        int& activePC = isMacro ? m_macroStack.back().currentPC : m_programPC;
        std::vector<std::string>& activeMemory = isMacro ? m_macroStack.back().memory : m_programMemory;

        // 同步 HMI 雙視窗需要的變數
        if (isMacro) {
            m_macroProgramName = m_macroStack.back().programName;
            m_macroProgramPC = activePC;
        }
        else {
            m_macroProgramName = "";
            m_macroProgramPC = -1;
        }

        // ==========================================
        // 🌟 階段 A：萬用等待條件檢查 
        // ==========================================
        if (m_waitCallback != nullptr) {
            if (m_waitCallback(this) == false) return; // 繼續等

            m_waitCallback = nullptr;

            // 🌟 安全防護：確保等待結束時沒有被觸發警報，才往下走
            if (m_state != NCState::ALARM && m_state != NCState::HOLD) {
                activePC++;
            }
        }

        // ==========================================
        // 🌟 階段 B & C：結束判斷與讀取執行
        // ==========================================
        if (m_waitCallback == nullptr)
        {
            // 階段 C：結束判斷
            if (activePC >= activeMemory.size())
            {
                if (isMacro) {
                    ReturnMacro(); // 防呆：副程式如果沒寫 M99，跑到底自動返回
                }
                else
                {
                    m_state = NCState::P_END;
                    DEBUG_PRINT("[NC] Program Finished (M30)\n");

                    // ==========================================
                    // 🌟 測試驗證：印出 #501, #502, #503 的最終結果
                    // ==========================================
                    double v501 = MacroSys.GetVar('#', 501);
                    double v502 = MacroSys.GetVar('#', 502);
                    double v503 = MacroSys.GetVar('#', 503);

                    DEBUG_PRINT("========================================\n");
                    DEBUG_PRINT(" --- Macro Stack Isolation Test ---\n");

                    // 🛠️ RTX64 安全寫法：拆解浮點數為整數與小數 (精準到小數後兩位)
                    int i501 = (int)v501;
                    int f501 = (int)((v501 - i501) * 100);
                    f501 = f501 < 0 ? -f501 : f501; // 防止負數印出 -X.-XX

                    int i502 = (int)v502;
                    int f502 = (int)((v502 - i502) * 100);
                    f502 = f502 < 0 ? -f502 : f502;

                    int i503 = (int)v503;
                    int f503 = (int)((v503 - i503) * 100);
                    f503 = f503 < 0 ? -f503 : f503;

                    DEBUG_PRINT("  #501 (Main Level)   = %d.%02d \n", i501, f501);
                    DEBUG_PRINT("  #502 (Macro Level 1)= %d.%02d \n", i502, f502);
                    DEBUG_PRINT("  #503 (Macro Level 2)= %d.%02d \n", i503, f503);
                    DEBUG_PRINT("========================================\n");

                    m_programPC = 0;
                }
                return;
            }

            // 階段 B：讀取指令
            std::string rawLine = activeMemory[activePC];
            NCBlock block = Parser.ParseLine(rawLine);

            if (!block.isEmpty)
            {
                if (block.isGoto)
                {
                    // ==========================================
                     // 🌟 巨集跳躍邏輯 (GOTO)
                     // ==========================================
                    int targetN = block.gotoTarget;
                    bool found = false;

                    // 尋找目前執行的程式 (主程式或副程式) 內的所有行
                    for (int i = 0; i < (int)activeMemory.size(); i++) {

                        // 🚀 效能優化：字串裡面有 'N' 才去解析它，節省 1ms 迴圈的 CPU 資源
                        if (activeMemory[i].find('N') != std::string::npos || activeMemory[i].find('n') != std::string::npos) {

                            NCBlock checkBlock = Parser.ParseLine(activeMemory[i]);

                            // 檢查解析出來的這行，是否有 N 碼，且數值等於我們要的 targetN
                            if (checkBlock.has('N') && (int)checkBlock.val('N') == targetN) {
                                activePC = i; // 🎯 關鍵：將程式指標直接跳轉到該行
                                found = true;
                                break;
                            }
                        }
                    }

                    // 防呆防護：如果整支程式都找不到這個 N 碼
                    if (!found) {
                        DEBUG_PRINT("[Alarm] GOTO target N%d not found!\n", targetN);
                        AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR); // 或新增 MACRO_ERROR
                        m_state = NCState::ALARM;
                        return; // 鎖死機台，結束執行
                    }
                }
                else {
                    m_programChanged = false;
                    ExecuteBlock(block);

                    // ==========================================
                    // 🌟 防護 1：真正的異常 (語法錯誤)
                    // 如果觸發了警報，立刻鎖死離開，絕對不加 PC，讓畫面停在錯誤行！
                    // ==========================================
                    if (AlarmManager::GetInstance().HasAlarm()) {
                        m_state = NCState::ALARM;
                        return;
                    }

                    // ==========================================
                    // 🌟 正常過關：執行成功 (包含 M00 也算成功執行完畢)
                    // 如果瞬間完成，且沒有跳轉，就把目前行號 + 1
                    // ==========================================
                    if (m_waitCallback == nullptr && !m_programChanged) {
                        activePC++;
                    }

                    // ==========================================
                    // 🌟 防護 2：正常的暫停 (M00)
                    // 雖然指標已經走到下一行了，但我們必須在這裡「凍結」迴圈，
                    // 讓機台停下來，等待操作員按下 Cycle Start。
                    // ==========================================
                    if (m_state == NCState::HOLD) {
                        return;
                    }
                }
            }
            else {
                activePC++; // 空行直接跳過
            }
        }
    }
}

void NCManager::ExecuteBlock(const NCBlock& block)
{
    // 預設不等待
    m_waitCallback = nullptr;



    // ==========================================
    // 🌟 安全性檢查：單節是否包含多個 G 碼
    // ==========================================
    if (block.gCount > 1) 
    {
        DEBUG_PRINT("[Alarm] Multiple G-Codes in a single block! Found: %d\n", block.gCount);
        AlarmManager::GetInstance().Trigger(AlarmManager::G_code_Count_Error);
        m_state = NCState::HOLD;
        return; // 直接中止
    }

    // ==========================================
    // 🌟 安全性檢查 2：單節是否包含多個 M 碼
    // ==========================================
    if (block.mCount > 1) {
        DEBUG_PRINT("[Alarm] Multiple M-Codes in a single block! Found: %d\n", block.mCount);

        // 建議未來可以在 AlarmManager 新增一個 M_CODE_CONFLICT 警報
        // 目前先借用 SYNTAX_ERROR
        AlarmManager::GetInstance().Trigger(AlarmManager::M_code_Count_Error);
        m_state = NCState::HOLD;
        return;
    }


    // ==========================================
    // 1. 瞬間完成的設定 (不需等待)
    // ==========================================
    if (block.has('E')) {
        // m_edmManager.ApplyE(block.val('E'));
    }
    if (block.has('B')) {
        // m_edmManager.ApplyB(block.val('B'));
    }

  

    // ==========================================
    // 2. G 碼轉接中心 (Routing Hub)
    // ==========================================
    if (block.hasG) {
        switch (block.gCode)
        {
        case 0:
        case 1:
            // 轉接給直線移動部門，並把他們回傳的「檢查函式」存起來
            m_waitCallback = GCodeHandlers::Handle_G00(block, this);
            break;

        case 4:
            // 轉接給延遲部門，並存下他們專屬的檢查函式
            m_waitCallback = GCodeHandlers::Handle_G04(block, this);
            break;

        case 90: case 91:
        case 65: // 
        case 54: case 55: case 56: case 57: case 58: case 59:
            // 狀態設定回傳的一定是 nullptr (不需等待)
            m_waitCallback = GCodeHandlers::Handle_GCode(block, this);
            break;

        default:
            // 🌟 關鍵修改：不支援的 G 碼，立刻觸發警報並鎖機！
            DEBUG_PRINT("[Alarm] Unsupported G-Code: G%02d\n", block.gCode);
            AlarmManager::GetInstance().Trigger(AlarmManager::Unable_to_recognize_G_code);
            m_state = NCState::HOLD;
            break;
        }
    }

    // ==========================================
    // 3. 需要等待的動作：M 碼
    // ==========================================
    if (block.mCount > 0)
    {
        int m = block.mCode[0];

        // 🌟 流程控制類 M 碼 (自己處理)

        if (m == 98)
        {
            int pVal = block.has('P') ? (int)block.val('P') : 0;
            std::string macroFile = "O" + std::to_string(pVal) + ".nc";
            CallMacro(macroFile);
        }
        else if (m == 99)
        {
            ReturnMacro();
        }
        else
        {
            // 🌟 IO / 狀態類 M 碼 (丟給 GCodeHandlers)
            // 這會處理 M00, M30, M03, M08 等等
            m_waitCallback = GCodeHandlers::Handle_MCode(block, this);
        }
    }
}