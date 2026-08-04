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
#include <sstream>
NCManager::NCManager(MotionCore& motion) : m_motion(motion), MathParser(MacroSys), Parser(MathParser)
{
    // 初始化軸名稱為空白字元 (防呆)
    for (int i = 0; i < 8; i++) {
        m_axisNames[i] = ' ';
    }

    // 🌟 開機立刻載入軸定義檔！
    LoadAxisConfiguration();

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
    m_motion.ResetPhysicalPC(); // 🌟 載入新程式，實體行號歸零
   
    // 🌟 取得大腦目前的狀態，並同步給馬達標籤機
    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    bool curIsAbs = CoordSys.isAbsoluteMode;
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度
    bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
    int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;

    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; i++) {
        if (CoordSys.isMirrorActive[i]) {
            curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
        }
    }

    // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
    bool curG16 = CoordSys.isPolarCoordinateActive;

    // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane; // 17, 18 或是 19

    // 🌟 拿大腦最乾淨的狀態強制洗掉馬達的殘影
    m_motion.ResetPhysicalTags(CoordSys.GetCurrentWCSGCode(),CoordSys.toolLengthMode, CoordSys.currentHCode,CoordSys.toolRadiusMode, CoordSys.currentDCode, curIsAbs, curG68, curG68Angle, curG168, curWCode, curG51, curScale, curMirrorMask, curG16, curG162, curPlane);


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
    // 如果目前是 HOLD 狀態，代表我們要「解除暫停」
    if (m_state == NCState::HOLD)
    {
        m_state = NCState::RUN;

        // 🌟 恢復原本的進給倍率 (這裡寫死 1.0 代表 100%，如果有倍率旋鈕可以從 UI 讀取)
        m_motion.SetGroupFeedrateOverride(1.0);

        //DEBUG_PRINT("[NC] Resuming from Feed Hold!\n");
    }
    // 如果是正常 READY 狀態，代表我們要「全新啟動」
    else if (m_state == NCState::READY)
    {
        // 如果在 MANUAL 模式按下啟動，豎起自動執行旗標
        if (m_mode == NCOperationMode::MANUAL && !m_manualMemory.empty()) {
            m_manualAutoRunning = true;
        }

        m_state = NCState::RUN;
        //DEBUG_PRINT("[NC] Cycle Start!\n");
    }
}

void NCManager::FeedHold()
{
    // 只有在運行中 (RUN) 按下暫停才有效
    if (m_state == NCState::RUN)
    {
        m_state = NCState::HOLD; // 鎖住 NC，讓它停在現在的 G 碼，不要讀下一行

        // 🌟 神奇魔法：將倍率設為 0.0，底層的軌跡規劃器就會沿著原路徑平滑煞車！
        m_motion.SetGroupFeedrateOverride(0.0);

        //DEBUG_PRINT("[NC] Feed Hold Triggered!\n");
    }
}

void NCManager::Reset()
{

    //重置馬達區塊--------------------------------------------------
    m_motion.StopGroup();//滑行停止
    m_motion.ResetPhysicalPC(); // 🌟 按下 Reset，實體行號歸零
    
    

    // 🌟 2. 取得大腦洗乾淨後的 3 大狀態
    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    bool curIsAbs = CoordSys.isAbsoluteMode;
    // 🌟 讀取大腦的 G68 狀態 (假設你在 CoordSys 有這個變數)
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度
    bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
    int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;

    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; i++) {
        if (CoordSys.isMirrorActive[i]) {
            curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
        }
    }
    // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
    bool curG16 = CoordSys.isPolarCoordinateActive;

    // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane; // 17, 18 或是 19

    // 🌟 3. 強制同步給馬達！撕掉舊標籤，貼上乾淨狀態，徹底消滅殘影！
    m_motion.ResetPhysicalTags(currentBrainWCS, currentBrainToolMode, currentBrainHCode, currentBraintoolRadiusMode, currentBraintoolDCode, curIsAbs, curG68, curG68Angle, curG168, curWCode, curG51, curScale, curMirrorMask, curG16, curG162, curPlane);

    //m_motion.EmergencyStopGroup();//急停
    //m_motion.ResetAllFaults();//軸清除錯誤
    
   
    m_motion.SetGroupFeedrateOverride(1.0);//進給倍率回到100%

    //重置G碼區塊--------------------------------------------------
    Reset_Gode();       // 重置G碼相關
   
    //重置NC區塊--------------------------------------------------
    MacroSys.Reset();//重置Macro變數

    m_macroStack.clear();
    m_programPC = 0;
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    //清空 MDI 與 MANUAL 執行狀態
    m_mdiPC = 0;
    m_manualPC = 0;
    m_manualAutoRunning = false;


    std::queue<NCBlock> empty;
    std::swap(m_blockQueue, empty);
    m_waitCallback = nullptr;



    //重置Alarm--------------------------------------------------
    AlarmManager::GetInstance().Clear();


    m_state = NCState::RESET_STATE;
    m_state = NCState::READY;





}
void NCManager::Reset_Gode()       // 重置G碼相關
{
    GCodeHandlers::Reset_G04(this);
    CoordSys.Set_G90G91(90, this);//重置G90 絕對模式
    CoordSys.CancelToolLengthCompensation(this);//取消刀常補正
    CoordSys.CancelWorkpieceRotation(this);//工件補償取消
    CoordSys.SetActivePlane(17, this);//平面選擇
    CoordSys.isCAxisOffsetRotationEnabled = true;//C 軸電極偏心旋轉補償
    CoordSys.CancelScaling(this);//關閉縮放功能
    bool hasAxis[8] = { false };
    CoordSys.CancelMirror(hasAxis,this);//關閉鏡像功能
    CoordSys.CancelPolarCoordinate(this);//關閉極座標
    CoordSys.CancelToolRadiusCompensation(this);//關閉刀徑補償
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
   
    std::string fullPath = GlobalConfig::GetInstance().NCMacroProgramDir + filename;
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

    // 🌟 關鍵修改：將 m_programPC 換成 GetBasePC()
    newFrame.returnPC = m_macroStack.empty() ? (GetBasePC() + 1) : (m_macroStack.back().currentPC + 1);

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
    if (m_macroStack.empty()) return;

    int retPC = m_macroStack.back().returnPC;
    m_macroStack.pop_back();
    MacroSys.PopCallStack();

    // 🌟 關鍵修改：將 m_programPC 換成 GetBasePC()
    if (m_macroStack.empty()) {
        GetBasePC() = retPC;
    }
    else {
        m_macroStack.back().currentPC = retPC;
    }

    m_programChanged = true;
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
    NC_RunCount++;


  
    // =========================================================
    // 🌟 1. 【最高優先】每一圈都重新結算並更新機台總合狀態！
    // =========================================================
    m_edmState = GetMachineEDMState();


    // =========================================================
    // 🚨 2. 【絕對防禦攔截網】警報與急停鎖死區
    // =========================================================
    // 不論是軟體觸發的 Alarm，或是從 UI 傳下來的 Alarm 狀態
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM) 
    {
        m_state = NCState::ALARM; // 確保 NC 大腦確實進入警報狀態

        // 🌟 [關鍵新增]：只要在警報狀態，每一毫秒都強制下達急停！
        // (底層的 EmergencyStop 有防重複機制，所以這樣寫既安全又暴力)
        m_motion.EmergencyStopGroup();

        // ⚠️ 立刻退出迴圈，絕對不准往下執行任何軌跡運算或 G 碼解析！
        return; 
    }
    // =========================================================
    // 🌟 3. 正常任務分流 (只有在無警報時才會走到這裡)
    // =========================================================
    switch (m_mode)
    {
    case NCOperationMode::MEMORY:
        if (m_state == NCState::RUN) {
            ProcessExecutionEngine();
        }
        break;

    case NCOperationMode::MDI:
        // 🌟 關鍵修改：MDI 模式現在也支援手動操作了！
        if (m_state == NCState::RUN) {
            ProcessExecutionEngine(); // 如果按下 Cycle Start，執行 MDI 字串
        }
        else {
            ProcessManualMode();      // 閒置時，允許操作員直接使用手輪或 JOG
        }
        break;

    case NCOperationMode::MANUAL:
        if (m_state == NCState::RUN && m_manualAutoRunning) {
            ProcessExecutionEngine();
        }
        else {
            ProcessManualMode();
        }
        break;

    case NCOperationMode::EDIT:
        break;
    }

   
}


// ==========================================
// 🚀 終極統一執行引擎 (支援所有模式、GOTO、M98)
// ==========================================
void NCManager::ProcessExecutionEngine()
{
    bool isMacro = !m_macroStack.empty();

    // 🌟 動態綁定：根據目前的模式，抓出對應的底層資料
    int& basePC = GetBasePC();
    std::vector<std::string>& baseMemory = GetBaseMemory();

    // 判斷現在是在跑最上層的字串，還是在跑副程式

    int& activePC = isMacro ? m_macroStack.back().currentPC : basePC;
    std::vector<std::string>& activeMemory = isMacro ? m_macroStack.back().memory : baseMemory;

    // 同步 HMI 雙視窗需要的變數 (MDI/MANUAL 時也能顯示目前的副程式名稱)
    if (isMacro) {
        m_macroProgramName = m_macroStack.back().programName;
        m_macroProgramPC = activePC;
    }
    else {
        m_macroProgramName = "";
        m_macroProgramPC = -1;
    }

    // ==========================================================
    // --- 階段 A：萬用等待條件檢查 (物理卡點) ---
    // 如果這裡有值(例如遇到 G12 或是 G00)，大腦就會停止預讀，直到馬達走完
    // ==========================================================
    if (m_waitCallback != nullptr) {
        if (m_waitCallback(this) == false) return; // 繼續等馬達跑完

        m_waitCallback = nullptr;
        if (m_state != NCState::ALARM && m_state != NCState::HOLD) {
            activePC++; // 解除等待，準備讀下一行
        }
    }

    // ==========================================================
    // 🌟 預讀閘門：容量限制 (Look-Ahead Buffer Limit)
    // 如果 MotionCore 的倉庫已經塞了 50 條路徑，大腦這回合就先休息！
    // ==========================================================
    if (m_motion.GetQueueSize() >= 50) {
        return; // 下一個 Tick 再來看看倉庫有沒有空位
    }

    // --- 階段 B & C：讀取與結束判斷 ---
    if (m_waitCallback == nullptr)
    {
        // 結束判斷
        if (activePC >= activeMemory.size())
        {
            if (isMacro) 
            {
                ReturnMacro(); // 副程式結束返回
            }
            else 
            {
                // 最頂層程式結束了，依照模式決定去留
                if (m_mode == NCOperationMode::MANUAL) 
                {
                    m_state = NCState::READY;
                    m_manualAutoRunning = false;
                }
                // 🌟 關鍵修改：MDI 跑完後也直接退回 READY，無縫接軌手動 JOG！
                else if (m_mode == NCOperationMode::MDI) 
                {
                    m_state = NCState::READY;
                }
                else {
                    // 只有 MEMORY 主程式跑完才會進入 P_END (需按 Reset)
                    m_state = NCState::P_END;
                }

                basePC = 0; // 執行完畢指標歸零
                DEBUG_PRINT("[NC] Execution Finished.\n");


                //重置G碼區塊--------------------------------------------------
                Reset_Gode();// 重置G碼相關
            }
            return;
        }

        std::string rawLine = activeMemory[activePC];
        NCBlock block = Parser.ParseLine(rawLine);

        if (!block.isEmpty)
        {
            if (block.isGoto)
            {
                // 🌟 GOTO 跳躍邏輯 (完全相容所有模式)
                int targetN = block.gotoTarget;
                bool found = false;

                for (int i = 0; i < (int)activeMemory.size(); i++) {
                    if (activeMemory[i].find('N') != std::string::npos || activeMemory[i].find('n') != std::string::npos) {
                        NCBlock checkBlock = Parser.ParseLine(activeMemory[i]);
                        if (checkBlock.has('N') && (int)checkBlock.val('N') == targetN) {
                            activePC = i;
                            found = true;
                            break;
                        }
                    }
                }

                if (!found) {
                    DEBUG_PRINT("[Alarm] GOTO target N%d not found!\n", targetN);
                    AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
                    m_state = NCState::ALARM;
                    return;
                }
            }
            else {
                // ==========================================================
                  // 🌟 【神級預讀屏障 (Look-Ahead Barrier)】
                  // 這些 G 碼必須在「機台完全靜止、且底層倉庫為空」時才能下達！
                  // ==========================================================
                bool isBarrier = 
                    (
                        block.gCode == 0 || block.gCode == 12 ||
                        block.gCode == 4 ||
                        block.gCode == 7 ||
                        block.gCode == 28 || block.gCode == 30||
                        block.gCode == 32 ||
                        block.gCode == 53 || block.gCode == 161||
                        block.gCode == 65 ||
                        block.gCode == 92 ||   
                        ((block.gCode >= 54 && block.gCode <= 59))||
                        ((block.gCode >= 154 && block.gCode <= 159)) ||
                        ((block.gCode >= 254 && block.gCode <= 259)) ||
                        ((block.gCode >= 354 && block.gCode <= 359)) ||
                        ((block.gCode >= 454 && block.gCode <= 459)) ||
                        ((block.gCode >= 554 && block.gCode <= 559)) ||
                        ((block.gCode >= 654 && block.gCode <= 659)) ||
                        ((block.gCode >= 754 && block.gCode <= 759)) ||
                        ((block.gCode >= 854 && block.gCode <= 859)) ||
                        ((block.gCode >= 954 && block.gCode <= 959)) 
                    );
             
                if (block.gCode == 0 && block.has('P') == 1)//G00 P1模式為可預讀路徑
                {
                    isBarrier = false;
                }

                
                // 如果這行是 G00/G28/G30，且底層還在跑 (倉庫有東西，或馬達還沒到位)
                if (isBarrier && (m_motion.GetQueueSize() > 0 || !m_motion.IsGroupDone())) {
                    // 大腦立刻罷工！
                    // 不執行 ExecuteBlock，也不把 activePC++，
                    // 等下一毫秒再回來問：「馬達停了沒？」直到完全靜止才放行！
                    return;
                }

                // 只有在機台完全靜止時，G00 才會走到這裡被執行！
                m_programChanged = false;


              
            

                // 🌟 【貼標籤】：大腦瞬間讀取自己當下的狀態
                int currentBrainWCS = CoordSys.GetCurrentWCSGCode();  // 🌟 【貼標籤】：把大腦當下的「行號」和「座標系」印成標籤！
                int currentBrainToolMode = CoordSys.toolLengthMode; // 從 CoordSys 讀取
                int currentBrainHCode = CoordSys.currentHCode;      // 從 CoordSys 讀取
                int curTRad = CoordSys.toolRadiusMode; // 🌟 刀徑
                int curD = CoordSys.currentDCode;      // 🌟 D碼
                // 🌟 直接讀取你原本就寫好的 CoordSys.isAbsoluteMode
                bool curIsAbs = CoordSys.isAbsoluteMode;
                // 🌟 讀取大腦的 G68 狀態 (假設你在 CoordSys 有這個變數)
                bool curG68 = CoordSys.isG68Active;
                double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度

                bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
                int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
                // 🌟 讀取大腦的 G51 狀態 (假設你在 CoordSys 有這兩個變數)
                bool curG51 = CoordSys.isScalingActive;
                double curScale = CoordSys.scaleFactor;

                // 🌟 讀取大腦的鏡像狀態，並打包成一個 byte (Bitmask)
                uint8_t curMirrorMask = 0;
                for (int i = 0; i < 8; i++) {
                    if (CoordSys.isMirrorActive[i]) {
                        curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
                    }
                }

                // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
                bool curG16 = CoordSys.isPolarCoordinateActive;

                // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
                bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
                int curPlane = CoordSys.activePlane; // 17, 18 或是 19

                // 🌟 印出標籤並貼到標籤機上
                m_motion.SetNextCommandState(activePC, currentBrainWCS, currentBrainToolMode, currentBrainHCode, curTRad, curD, curIsAbs, curG68, curG68Angle, curG168, curWCode, curG51, curScale, curMirrorMask, curG16, curG162, curPlane);


                ExecuteBlock(block); // 執行 G 碼

                if (AlarmManager::GetInstance().HasAlarm()) {
                    m_state = NCState::ALARM;
                    if (m_mode == NCOperationMode::MANUAL) m_manualAutoRunning = false;
                    return;
                }

                if (m_waitCallback == nullptr && !m_programChanged) {
                    activePC++;
                }

                if (m_state == NCState::HOLD) {
                    return;
                }
            }
        }
        else {
            activePC++; // 空行跳過
        }
    }
}


// ==========================================
// 🌟 MANUAL 模式專屬邏輯 (自動指令優先，JOG 墊後)
// ==========================================
void NCManager::ProcessManualMode()
{
    // ==========================================
    // 🕹️ 正常處理：純硬體 JOG / MPG
    // ==========================================
    // SHM_Data* pShm = SHMManager::GetInstance().GetData();
    // if (pShm == nullptr) return;

    // (將你原本讀取 pShm 按鈕，呼叫 m_motion.Jog(...) 的邏輯寫在這裡)
}
// ==========================================
// 🌟 動態獲取當前模式的 PC 指標
// ==========================================
int& NCManager::GetBasePC()
{
    if (m_mode == NCOperationMode::MDI) return m_mdiPC;
    if (m_mode == NCOperationMode::MANUAL) return m_manualPC;
    return m_programPC; // 預設為 MEMORY 主程式
}

// ==========================================
// 🌟 動態獲取當前模式的記憶體緩衝區
// ==========================================
std::vector<std::string>& NCManager::GetBaseMemory()
{
    if (m_mode == NCOperationMode::MDI) return m_mdiMemory;
    if (m_mode == NCOperationMode::MANUAL) return m_manualMemory;
    return m_programMemory; // 預設為 MEMORY 主程式
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
            m_waitCallback = GCodeHandlers::Handle_G00(block, this);
            break;
        case 7:
            m_waitCallback = GCodeHandlers::Handle_G07(block, this);
            break;
        case 12:
            m_waitCallback = GCodeHandlers::Handle_G12(block, this);
            break;
        case 161:
            m_waitCallback = GCodeHandlers::Handle_G161(block, this);
            break;
        case 53:
            m_waitCallback = GCodeHandlers::Handle_G53(block, this);
            break;
        case 28:
            m_waitCallback = GCodeHandlers::Handle_G28(block, this);
            break;
      
        case 30:
            m_waitCallback = GCodeHandlers::Handle_G30(block, this);
            break;
        case 32:
            m_waitCallback = GCodeHandlers::Handle_G32(block, this);
            break;
        case 4:
          
            m_waitCallback = GCodeHandlers::Handle_G04(block, this);
            break;

        case 54: case 55: case 56: case 57: case 58: case 59:
        case 154: case 155: case 156: case 157: case 158: case 159:
        case 254: case 255: case 256: case 257: case 258: case 259:
        case 354: case 355: case 356: case 357: case 358: case 359:
        case 454: case 455: case 456: case 457: case 458: case 459:
        case 554: case 555: case 556: case 557: case 558: case 559:
        case 654: case 655: case 656: case 657: case 658: case 659:
        case 754: case 755: case 756: case 757: case 758: case 759:
        case 854: case 855: case 856: case 857: case 858: case 859:
        case 954: case 955: case 956: case 957: case 958: case 959:
            m_waitCallback = GCodeHandlers::Handle_GCode(block, this);
            break;
        case 10:
            m_waitCallback = GCodeHandlers::Handle_G10(block, this);
            break;
        case 160:
            m_waitCallback = GCodeHandlers::Handle_G160(block, this);
            break;
        case 68:
            m_waitCallback = GCodeHandlers::Handle_G68(block, this);
            break;
        case 69:
            m_waitCallback = GCodeHandlers::Handle_G69(block, this);
            break;
        case 90: case 91:case 92:
        case 43: case 44: case 49:
        case 17: case 18:case 19:
        case 65: // 
        case 162: case 163:
        
            // 狀態設定回傳的一定是 nullptr (不需等待)
            m_waitCallback = GCodeHandlers::Handle_GCode(block, this);
            break;
        case 168:
            m_waitCallback = GCodeHandlers::Handle_G168(block, this);
                break;
        case 169:
            m_waitCallback = GCodeHandlers::Handle_G169(block, this);
            break;
        case 40:
            m_waitCallback = GCodeHandlers::Handle_G40(block, this);
            break;
        case 41:
            m_waitCallback = GCodeHandlers::Handle_G41(block, this);
            break;
        case 42:
            m_waitCallback = GCodeHandlers::Handle_G42(block, this);
            break;
        case 50:
            m_waitCallback = GCodeHandlers::Handle_G50(block, this);
            break;
        case 51:
            m_waitCallback = GCodeHandlers::Handle_G51(block, this);
            break;
        case 150:
            m_waitCallback = GCodeHandlers::Handle_G150(block, this);
            break;
        case 151:
            m_waitCallback = GCodeHandlers::Handle_G151(block, this);
            break;
        case 15:
            m_waitCallback = GCodeHandlers::Handle_G15(block, this);
            break;
        case 16:
            m_waitCallback = GCodeHandlers::Handle_G16(block, this);
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

// =========================================================
// 🌟 2. 實作讀取 AXIS_CFG.ini
// =========================================================
void NCManager::LoadAxisConfiguration()
{
    
    std::string filepath = GlobalConfig::GetInstance().NCDataDir +"AXIS_CFG.ini";
    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        //printf("[Error] 無法開啟 AXIS_CFG.ini！將套用預設 X, Y, Z, A, B, C, U, V\n");
        // 如果找不到檔案，塞一組預設值給機台保命
        const char defaultAxes[8] = { 'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V' };
        for (int i = 0; i < 8; i++) m_axisNames[i] = defaultAxes[i];
        return;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.empty() || line[0] == ';') continue; // 略過空白與註解

        std::stringstream ss(line);
        std::string key, value;

        // 以 '=' 切割字串 (例如 "Axis0=X")
        if (std::getline(ss, key, '=') && std::getline(ss, value)) {
            // 解析 Axis0 ~ Axis7
            if (key.length() >= 5 && key.substr(0, 4) == "Axis") {
                int index = key[4] - '0'; // 把字元 '0' 轉成整數 0
                if (index >= 0 && index < 8) {
                    // 只取等號後面的第一個字元，如果寫 NONE 或空，就會抓不到英文字母
                    if (value.length() > 0 && value != "NONE") {
                        m_axisNames[index] = value[0];
                        //printf("[Config] 軸 %d 對應字元: %c\n", index, m_axisNames[index]);
                    }
                }
            }
        }
    }
    inFile.close();
}

// =========================================================
// 🌟 3. 提供給直譯器 (Parser) 搜尋用的 API
// =========================================================
int NCManager::GetAxisIndex(char gcodeLetter) const
{
    for (int i = 0; i < 8; i++) {
        if (m_axisNames[i] == gcodeLetter) {
            return i; // 找到對應的陣列 Index 了！
        }
    }
    return -1; // -1 代表這台機器沒有設定這個軸！
}


// ==========================================
// 🌟 載入 MDI 字串
// ==========================================
bool NCManager::LoadMDI(const std::string& mdiContent)
{
    m_mdiMemory.clear();
    m_mdiPC = 0;

    std::stringstream ss(mdiContent);
    std::string line;
    int lineCount = 0;

    while (std::getline(ss, line, '\n')) {
        if (!line.empty() && line.find_first_not_of("\r\t ") != std::string::npos) {
            m_mdiMemory.push_back(line);
            lineCount++;
        }
        // 使用我們在 .h 檔設定的常數來限制
        if (lineCount >= MAX_MDI_LINES) {
            DEBUG_PRINT("[NC Warning] MDI Input truncated to %zu lines.\n", MAX_MDI_LINES);
            break;
        }
    }
    return !m_mdiMemory.empty();
}

// ==========================================
// 🌟 載入 MANUAL 模式輕量自動指令
// ==========================================
bool NCManager::LoadManualAuto(const std::string& manualContent)
{
    // 使用我們在 .h 檔設定的常數來檢查 (1024KB)
    if (manualContent.length() > MAX_MANUAL_AUTO_BYTES) {
        DEBUG_PRINT("[Alarm] Manual Auto string exceeds %zu bytes limit!\n", MAX_MANUAL_AUTO_BYTES);
        return false;
    }

    m_manualMemory.clear();
    m_manualPC = 0;
    m_manualAutoRunning = false; // 載入後預設不啟動，等待 Cycle Start

    std::stringstream ss(manualContent);
    std::string line;

    while (std::getline(ss, line, '\n')) {
        if (!line.empty() && line.find_first_not_of("\r\t ") != std::string::npos) {
            m_manualMemory.push_back(line);
        }
    }
    return !m_manualMemory.empty();
}

// ==========================================
// 🌟 整合版：動態載入短程式碼 (自動判斷 MDI 還是 MANUAL)
// ==========================================
bool NCManager::LoadDynamicCode(const std::string& content)
{
    std::vector<std::string>* targetMemory = nullptr;
    int* targetPC = nullptr;

    // 1. 根據目前模式，動態綁定目標記憶體
    if (m_mode == NCOperationMode::MDI) {
        targetMemory = &m_mdiMemory;
        targetPC = &m_mdiPC;
    }
    else if (m_mode == NCOperationMode::MANUAL) {
        targetMemory = &m_manualMemory;
        targetPC = &m_manualPC;
        m_manualAutoRunning = false; // 載入時先關閉自動執行
    }
    else {
        DEBUG_PRINT("[NC Warning] Cannot load dynamic code in current OP mode!\n");
        return false; // 只有在 MDI 和 MANUAL 模式下才允許載入
    }

    // 2. 清空舊資料
    targetMemory->clear();
    *targetPC = 0;
    m_motion.ResetPhysicalPC(); // 🌟 載入 MDI，實體行號歸零

   

    // 🌟 取得大腦目前的狀態，並同步給馬達標籤機
    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    // 🌟 直接讀取你原本就寫好的 CoordSys.isAbsoluteMode
    bool curIsAbs = CoordSys.isAbsoluteMode;
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度
    bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
    int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;
    // 🌟 讀取大腦的鏡像狀態，並打包成一個 byte (Bitmask)
    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; i++) {
        if (CoordSys.isMirrorActive[i]) {
            curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
        }
    }
    // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
    bool curG16 = CoordSys.isPolarCoordinateActive;

    // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane; // 17, 18 或是 19

    m_motion.ResetPhysicalTags(currentBrainWCS, currentBrainToolMode, currentBrainHCode, currentBraintoolRadiusMode, currentBraintoolDCode, curIsAbs, curG68, curG68Angle, curG168, curWCode, curG51, curScale, curMirrorMask, curG16, curG162, curPlane);
    

    // 3. 解析並塞入記憶體
    std::stringstream ss(content);
    std::string line;

    while (std::getline(ss, line, '\n')) {
        if (!line.empty() && line.find_first_not_of("\r\t ") != std::string::npos) {
            targetMemory->push_back(line);
        }
    }

    DEBUG_PRINT("[NC] Dynamic Code Loaded, Lines: %d\n", (int)targetMemory->size());
    return !targetMemory->empty();
}

// ==========================================
// 🌟 獲取機台綜合狀態 (結算 NC 大腦與馬達硬體)
// ==========================================
EDMState NCManager::GetMachineEDMState()
{
    // ----------------------------------------------------
    // 🚨 1. [最高優先權] 警報與急停檢查 (回傳 ALARM)
    // ----------------------------------------------------
    // 檢查軟體警報 (AlarmManager 或是 NC 狀態為 ALARM)
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM) {
        return EDMState::ALARM;
    }

    // 檢查硬體馬達是否報警或處於急停狀態
    for (int i = 0; i < 8; ++i) {
        auto& axis = m_motion.GetAxisContext(i);
        // 只要有一軸急停、錯誤 (Fault) 或追隨誤差警報 (LagAlarm)
        if (axis.state == MotionState::MotionState_ESTOP ||
            axis.state == MotionState::MotionState_ERROR ||
            axis.isFault || axis.isLagAlarm)
        {

            if (axis.isFault)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_Fault, 0, axis.axisIndex);
            }
            if (axis.isLagAlarm)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_LAG_ERROR, 0, axis.axisIndex);
            }
       

            return EDMState::ALARM;
        }
    }

    // ----------------------------------------------------
      // 🔌 2. [次高優先權] 激磁 (Servo On) 檢查 (回傳 NOT_READY)
      // ----------------------------------------------------
    for (int i = 0; i < 8; ++i) {
        auto& axis = m_motion.GetAxisContext(i);

        // 🌟 只檢查「物理上確實存在（或被啟用）」的軸！
        // 這樣就算跳號 (例如有 0,1,2，跳過 3，有 4)，也不會卡死！
        if (axis.isExist) {
            if (!axis.isServoOn) {
                return EDMState::NOT_READY;
            }
        }
    }

    // ----------------------------------------------------
    // 🧠 3. [邏輯判斷] 根據 NC 大腦狀態推導機台狀態
    // ----------------------------------------------------
    switch (m_state)
    {
    case NCState::RUN:
        return EDMState::START;  // 🟢 正在跑程式

    case NCState::HOLD:
        return EDMState::HOLD;   // 🟡 操作員按下了暫停 (Feed Hold)

    case NCState::RESET_STATE:
    case NCState::P_END:
        return EDMState::STOP;   // ⚪ 程式結束或剛被 Reset 斬斷

    case NCState::IDLE:
    case NCState::READY:
        return EDMState::READY;  // 🔵 一切正常，等待 Cycle Start

    default:
        return EDMState::NOT_READY;
    }
}