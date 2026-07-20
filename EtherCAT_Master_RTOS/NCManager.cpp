#include "NCManager.h"
#include "MacroEngine.h"
#include "MacroParser.h"
#include "GCodeParser.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

#include <fstream>
#include <iostream>
NCManager::NCManager(MotionCore& motion)
    : m_motion(motion),
    MathParser(MacroSys), // 第一步：把 變數引擎 綁給 數學解譯器
    Parser(MathParser)    // 第二步：把 數學解譯器 綁給 翻譯官
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

void NCManager::ChangeMode(NCOperationMode newMode) {
    // 只有在 IDLE 或 READY 狀態才能切換模式
    if (m_state == NCState::IDLE || m_state == NCState::READY) {
        m_mode = newMode;
    }
}

void NCManager::ChangeState(NCState newState) {
    m_state = newState;
}

void NCManager::CycleStart() {
    if (m_state == NCState::READY || m_state == NCState::HOLD) {
        m_state = NCState::RUN;
        DEBUG_PRINT("[NC] Cycle Start!\n");
    }
}

void NCManager::FeedHold() {
    if (m_state == NCState::RUN) {
        m_state = NCState::HOLD;
        // 呼叫 MotionCore 的群組減速停止
        // m_motion.StopGroup(..., 0.5); 
    }
}

void NCManager::Reset() {
    m_state = NCState::RESET_STATE;
    std::queue<NCBlock> empty;

    // 🌟 將 m_commandQueue 改成 m_blockQueue
    std::swap(m_blockQueue, empty);

    // m_motion.EmergencyStopGroup(); 
    m_state = NCState::READY;
}

void NCManager::PushBlock(const NCBlock& block) {
    // 略過單節跳躍
    if (block.isBlockSkip /* && 系統開啟了單節跳躍開關 */) return;

    m_blockQueue.push(block);
}



// 🌟 這個函式放在你主程式的 while(1) 迴圈中
void NCManager::ProcessTask() 
{
    if (m_state != NCState::RUN) return;

    if (m_mode == NCOperationMode::MEMORY)
    {
        // 這裡一樣看底層 Queue 有沒有滿
        while (m_programPC < m_programMemory.size() && !m_motion.IsGroupQueueFull())
        {
            // 🌟 即時翻譯 (Runtime Parsing)：現在才把字串轉成指令！
            std::string rawLine = m_programMemory[m_programPC];
            NCBlock block = Parser.ParseLine(rawLine);

            if (!block.isEmpty)
            {
                // 🌟 處理 GOTO 跳躍邏輯
                if (block.isGoto) {
                    DEBUG_PRINT("[NC_SIM] >>> EXECUTE GOTO N%d\n", block.gotoTarget);

                    // 去跳躍表查查看有沒有這個 N 碼
                    if (m_jumpTable.find(block.gotoTarget) != m_jumpTable.end()) {
                        // 找到的話，把目前行號 (PC) 指過去
                        m_programPC = m_jumpTable[block.gotoTarget];
                        continue; // 直接跳到下一圈 while，不要往下跑 ExecuteBlock
                    }
                    else {
                        DEBUG_PRINT("[Alarm] GOTO Target N%d Not Found!\n", block.gotoTarget);
                        m_state = NCState::HOLD; // 發報警，機台暫停
                        return;
                    }
                }
                else {
                    // 不是跳躍指令，那就是正常的 G/M/放電指令
                    ExecuteBlock(block);
                }
            }
            // 往下走一行
            m_programPC++;
        }

        // 結束判斷
        if (m_programPC >= m_programMemory.size() && m_motion.IsGroupDone()) {
            m_state = NCState::P_END;
            DEBUG_PRINT("[NC] Program Finished (M30)\n");
            m_programPC = 0;
        }
    }
}


void NCManager::ExecuteBlock(const NCBlock& block)
{
    // ==========================================
    // 1. 處理狀態設定 (G90, G91, G54~G59)
    // ==========================================
    if (block.hasG) {
        switch (block.gCode) {
        case 90:
            CoordSys.isAbsoluteMode = true;
            DEBUG_PRINT("[NC_SIM] Set G90 Absolute Mode\n");
            break;
        case 91:
            CoordSys.isAbsoluteMode = false;
            DEBUG_PRINT("[NC_SIM] Set G91 Incremental Mode\n");
            break;
        default:
            // 丟給座標系統判定是不是 G54~G59 等座標切換碼
            if (CoordSys.SetWCS(block.gCode)) {
                DEBUG_PRINT("[NC_SIM] Switch WCS to G%d\n", block.gCode);
            }
            break;
        }
    }

    // ==========================================
    // 2. 處理直線移動 (G00 / G01)
    // ==========================================
    if (block.hasG && (block.gCode == 1 || block.gCode == 0))
    {
        double currentMCS[8] = { 0 };
        double targetMCS[8] = { 0 };

        bool hasXYZ[8] = { block.has('X'), block.has('Y'), block.has('Z'),
                           block.has('U'), block.has('V'), block.has('W'),
                           block.has('A'), block.has('C') };

        double target[8] = { block.val('X'), block.val('Y'), block.val('Z'),
                             block.val('U'), block.val('V'), block.val('W'),
                             block.val('A'), block.val('C') };

        // 🌟 呼叫剛寫好的神經中樞進行轉換！
        CoordSys.Transform_WCS_to_MCS(target, hasXYZ, currentMCS, targetMCS);

        DEBUG_PRINT("[NC_SIM] G%02d Move Ready!\n", block.gCode);
        if (block.has('X')) DEBUG_PRINT("  -> X Target: %d\n", (int)(targetMCS[0] + 0.5));
        if (block.has('Y')) DEBUG_PRINT("  -> Y Target: %d\n", (int)(targetMCS[1] + 0.5));
        if (block.has('F')) DEBUG_PRINT("  -> Feedrate (F): %d\n", (int)(block.val('F') + 0.5));
    }

    // ==========================================
    // 2. 處理 E 碼 / B 碼
    // ==========================================
    if (block.has('E')) {
        DEBUG_PRINT("[NC_SIM] Switch E Code: %d\n", (int)block.val('E'));
    }
    if (block.has('B')) {
        DEBUG_PRINT("[NC_SIM] Switch B Code: %d\n", (int)block.val('B'));
    }

    // ==========================================
    // 3. 處理 M 碼 (用迴圈把 1~3 個 M 碼處理完)
    // ==========================================
    for (int i = 0; i < block.mCount; i++) {
        int m = block.mCode[i];
        if (m == 30) {
            DEBUG_PRINT("[NC_SIM] M30 Program End\n");
        }
        else if (m == 8) {
            DEBUG_PRINT("[NC_SIM] M08 Pump ON\n");
        }
    }
}