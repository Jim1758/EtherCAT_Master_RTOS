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

NCManager::NCManager(MotionCore& motion): m_motion(motion), MathParser(MacroSys), Parser(MathParser) 
{
    // 初始化設定
    m_state = NCState::IDLE;
    m_mode = NCOperationMode::MEMORY; // 預設記憶體模式
}

bool NCManager::LoadProgram(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

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
    m_state = NCState::RESET_STATE;
    std::queue<NCBlock> empty;

    // 將 m_commandQueue 改成 m_blockQueue
    std::swap(m_blockQueue, empty);

    // m_motion.EmergencyStopGroup(); 
    m_state = NCState::READY;
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
    NC_RunCount++;//NC執行迴圈數
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
        // 🌟 階段 A：萬用等待條件檢查 (完全消滅 if-else 地獄！)
        // ==========================================
        if (m_waitCallback != nullptr) {

            // 呼叫專屬的檢查函式。如果回傳 false，代表還沒完成。
            if (m_waitCallback(this) == false) {
                return; // 直接跳出迴圈，把時間還給 RTX64
            }

            // 如果回傳 true，代表等完了！清空 Callback，PC 準備往下走
            m_waitCallback = nullptr;
            m_programPC++;
        }

        // ==========================================
        // 🌟 階段 B：如果沒在等，就可以讀取下一行指令
        // ==========================================
        if (m_waitCallback == nullptr && m_programPC < m_programMemory.size())
        {
            std::string rawLine = m_programMemory[m_programPC];
            NCBlock block = Parser.ParseLine(rawLine);

            if (!block.isEmpty)
            {
                if (block.isGoto)
                {
                    DEBUG_PRINT("[NC_SIM] >>> EXECUTE GOTO N%d\n", block.gotoTarget);
                    if (m_jumpTable.find(block.gotoTarget) != m_jumpTable.end()) {
                        m_programPC = m_jumpTable[block.gotoTarget];
                    }
                    else {
                        DEBUG_PRINT("[Alarm] GOTO Target Not Found!\n");
                        AlarmManager::GetInstance().Trigger(AlarmManager::GOTO_NOT_FOUND);
                        m_state = NCState::HOLD;
                    }
                }
                else {
                    // 執行 G/M/E 碼 (呼叫總機小姐)
                    ExecuteBlock(block);

                    // 如果這行指令沒有設定 Callback，代表瞬間完成，直接去下一行
                    if (m_waitCallback == nullptr)
                    {
                        m_programPC++;
                    }
                }
            }
            else
            {
                m_programPC++; // 空行直接跳過
            }
        }
        // ==========================================
        // 🌟 階段 C：結束判斷
        // ==========================================
        else if (m_waitCallback == nullptr && m_programPC >= m_programMemory.size()) {
            m_state = NCState::P_END;
            DEBUG_PRINT("[NC] Program Finished (M30)\n");
            m_programPC = 0;
        }
    }
}

void NCManager::ExecuteBlock(const NCBlock& block)
{
    // 預設不等待
    m_waitCallback = nullptr;

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
        case 54: case 55: case 56: case 57: case 58: case 59:
            // 狀態設定回傳的一定是 nullptr (不需等待)
            m_waitCallback = GCodeHandlers::Handle_GCode(block, this);
            break;

        default:
            DEBUG_PRINT("[NC_SIM] Unsupported G-Code: G%02d\n", block.gCode);
            break;
        }
    }

    // ==========================================
     // 3. 需要等待的動作：M 碼
     // ==========================================
    if (block.mCount > 0) {
        // 🌟 轉接給 M 碼專屬部門！
        m_waitCallback = GCodeHandlers::Handle_MCode(block, this);
    }
}