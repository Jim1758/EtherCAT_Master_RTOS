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

    //DEBUG_PRINT("[NC] Program Loaded, Lines: %d\n", (int)m_programMemory.size());
    m_state = NCState::READY;
    return true;
}

void NCManager::ChangeMode(NCOperationMode newMode)
{
    // 只有在 IDLE 或 READY 狀態才能切換模式
    if (m_state == NCState::IDLE || m_state == NCState::READY|| m_state == NCState::P_END) {
        m_mode = newMode;
    }

    if (m_state == NCState::P_END)
    {
        Reset();
    }
}

void NCManager::ChangeState(NCState newState) {
    m_state = newState;
}

// ==========================================
// 🌟 升級版 CycleStart (支援 M30 P_END 乾淨重啟)
// ==========================================
void NCManager::CycleStart()
{
    // 如果目前是 HOLD 狀態，代表我們要「解除暫停」
    if (m_state == NCState::HOLD)
    {
        m_state = NCState::RUN;
        m_motion.SetGroupFeedrateOverride(1.0); // 恢復進給倍率
        m_pauseAfterBlock = false; // 🌟 核心防護：只要按下啟動，強制清除當前行的暫停要求，消滅雙重卡點！
    }
    // 如果是正常 READY 或 P_END (程式結束)，代表我們要「全新啟動」
    else if (m_state == NCState::READY || m_state == NCState::P_END)
    {
        // 🌟 從 P_END 重新啟動，強制行號為 0，洗乾淨狀態
        if (m_state == NCState::P_END) {
            GetBasePC() = 0;
            Reset_Gode();
            m_macroStack.clear();
        }

        if (m_mode == NCOperationMode::MANUAL && !m_manualMemory.empty()) {
            m_manualAutoRunning = true;
        }

        m_pauseAfterBlock = false; // 確保乾淨啟動
        m_motion.SyncVirtualEndPosition();
        UpdateSystemVariables();

        m_state = NCState::RUN; // 狀態轉為 RUN，正式出發！
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
    if (m_motion.IsAnyAxisFaulted())
    {
        m_motion.ResetAllFaults();//有錯誤才清除

    }
    m_motion.StopGroup();//滑行停止
    m_motion.ResetPhysicalPC(); // 🌟 按下 Reset，實體行號歸零
    
  
   
    // 🌟 [新增] 如果有放電跳刀/排渣，必須強制解鎖跳刀狀態機！
    // m_motion.ResetAllFaults(); // (如果您有寫清除跳刀狀態的 API，建議在這裡呼叫)

    // ==========================================================
    // 2. 【順序修正】：先洗乾淨大腦的 G 碼與 M 碼！
    // ==========================================================
    Reset_Gode();       // 🌟 必須先執行！將 G90, G49, G50, 平面等全部洗回預設值

    // 🌟 [強烈建議新增]：通知 PLC 關閉主軸與切削水 (相當於執行 M05, M09)
    // PLCManager::GetInstance().SetSpindleStop();
    // PLCManager::GetInstance().SetCoolantOff();

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

   
   
    // 🌟 清理完成後刷新變數
    UpdateSystemVariables();

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

    // 🌟 [新增]：清理我們為了單步與暫停所加的防暴衝旗標
    m_pauseAfterBlock = false;
    m_programChanged = false;
    m_waitCallback = nullptr;


    std::queue<NCBlock> empty;
    std::swap(m_blockQueue, empty);
    m_waitCallback = nullptr;



    //重置Alarm--------------------------------------------------
    AlarmManager::GetInstance().Clear();


    m_state = NCState::RESET_STATE;
   




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

    m_isG66Active = false; // 🌟 Reset 必須強制取消 G66
}

// ==========================================
// 🌟 1. 標準且安全的實作呼叫副程式邏輯
// ==========================================
bool NCManager::CallMacro(const std::string& filename) {

    // 🌟 檢查堆疊層數是否超過 8 層
    if (MacroSys.PushCallStack() == false) {
        //DEBUG_PRINT("[Alarm] Macro Call Depth Exceeded 8 Layers!\n");
        AlarmManager::GetInstance().Trigger(AlarmManager::MACRO_OVERFLOW);
        m_state = NCState::HOLD;
        return false;
    }

    // 🌟 【路徑自動補斜線】
    std::string macroDir = GlobalConfig::GetInstance().NCMacroProgramDir;
    if (!macroDir.empty() && macroDir.back() != '/' && macroDir.back() != '\\') {
        macroDir += "/";
    }

    std::string fullPath = macroDir + filename;
   // DEBUG_PRINT("[NC Macro] Attempting to open macro file: %s\n", fullPath.c_str());

    std::ifstream file(fullPath);
    if (!file.is_open()) {
        //DEBUG_PRINT("[Alarm] Macro File Not Found: %s\n", fullPath.c_str());
        AlarmManager::GetInstance().Trigger(AlarmManager::Macro_File_Not_Found);

        // 檔案找不到時，必須把變數堆疊 Pop 掉，避免記憶體錯亂！
        MacroSys.PopCallStack();
        m_state = NCState::HOLD;
        return false;
    }

    // 🌟 建立這層副程式的專屬執行框架 (Frame)
    MacroFrame newFrame;
    newFrame.programName = filename;
    newFrame.currentPC = 0;

    // 紀錄返回的主程式行號 (如果是從主程式呼叫，記住下一行；如果是從副程式呼叫，記住上一層的 PC + 1)
    newFrame.returnPC = m_macroStack.empty() ? (GetBasePC() + 1) : (m_macroStack.back().currentPC + 1);
    newFrame.repeatCount = 1; // 預設重複 1 次

    std::string line;
    while (std::getline(file, line)) {
        newFrame.memory.push_back(line);
    }
    file.close();

    //DEBUG_PRINT("[NC Macro] Successfully loaded macro: %s, Total Lines: %d, ReturnPC: %d\n",filename.c_str(), (int)newFrame.memory.size(), newFrame.returnPC);

    // 🌟 將這層副程式推入堆疊頂端
    m_macroStack.push_back(newFrame);

    m_programChanged = true; // 告訴大腦剛切換程式，不要把舊 PC + 1
    return true;
}
// ==========================================
// 🌟 2. 實作返回主程式邏輯
// ==========================================
void NCManager::ReturnMacro()
{
    if (m_macroStack.empty()) return;

    // =========================================================
    // 🌟 【L 重複次數核心】：如果 repeatCount 還大於 1，PC 歸零重跑！
    // =========================================================
    if (m_macroStack.back().repeatCount > 1) {
        m_macroStack.back().repeatCount--;  // 次數減 1
        m_macroStack.back().currentPC = 0;  // PC 歸零，回到副程式第一行
        m_programChanged = true;
        m_waitCallback = WaitAndClearQueueCallback; // 等待馬達清空後再跑下一輪
        return; // ⚠️ 不彈出堆疊，繼續留在副程式內重跑！
    }

    // --- 標準返回主程式邏輯 ---
    int retPC = m_macroStack.back().returnPC;
    m_macroStack.pop_back();
    MacroSys.PopCallStack();

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
        //DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
        return false;
    }
    return true;
}

// 🌟 放在 RTOS 迴圈的核心任務
void NCManager::ProcessTask()
{
    NC_RunCount++;

    if (UpdateSystemVariables_initialize_flag == 0)//第一次初始更新Macro變數
    {
        UpdateSystemVariables();
        UpdateSystemVariables_initialize_flag = 1;//
    }
  
    // =========================================================
    // 🌟 4. 【結尾動作】將最新的 NC 狀態刷給 PLC S 點！
    // =========================================================
    SyncNCStateToPLC();

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
    // 🌟 2.5 【新增：滑行煞車攔截網】等待 Reset 後的馬達完全靜止
    // =========================================================
    if (m_state == NCState::RESET_STATE)
    {
    
        // 檢查硬體馬達是否「完全靜止」？
        if (m_motion.IsGroupStandstill())
        {
           
            
            // 🛑 馬達完全靜止了！現在才是同步的完美時機！

            // 1. 同步大腦的數學座標 (把實體座標拉回大腦)
            CoordSys.SyncMachinePosition(CoordSys.actualMCS);

            // 2. 同步手腳的虛擬預讀起點 (徹底消滅幽靈座標！)
            m_motion.SyncVirtualEndPosition();

            // 3. 正式宣告機台準備就緒，可以接受下一個指令了！
            m_state = NCState::READY;
            // 🌟 清理完成後刷新變數
            UpdateSystemVariables();
            // DEBUG_PRINT("[NC] Reset Complete. Machine completely stopped.\n");
        }

        

        // ⚠️ 只要還在滑行，就立刻 return，不准執行下面的 G 碼解析與模式分流！
        return;
    }

    // =========================================================
// Machine Ready Interlock
// =========================================================
    if (m_edmState == EDMState::NOT_READY)
    {
        if (m_state == NCState::RUN)
        {
            FeedHold();
        }

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
// 🚀 終極統一執行引擎 (完美 M30 歸零卡住、M00 單擊解鎖)
// ==========================================
void NCManager::ProcessExecutionEngine()
{
    // 只有 RUN 狀態才能進來執行
    if (m_state != NCState::RUN) return;

    // 安全的 PC 控制器
    auto advancePC = [&]() {
        if (!m_macroStack.empty()) m_macroStack.back().currentPC++;
        else GetBasePC()++;
    };

    auto setPC = [&](int newPC) {
        if (!m_macroStack.empty()) m_macroStack.back().currentPC = newPC;
        else GetBasePC() = newPC;
    };

    bool isMacro = !m_macroStack.empty();
    if (isMacro) {
        m_macroProgramName = m_macroStack.back().programName;
        m_macroProgramPC = m_macroStack.back().currentPC;
    }
    else {
        m_macroProgramName = "";
        m_macroProgramPC = -1;
    }

    // ==========================================================
    // --- 階段 A：等待條件檢查與【神級任務接力】 ---
    // ==========================================================
    if (m_waitCallback != nullptr) {
        bool wasWaitingForStart = (m_waitCallback == WaitForCycleStartCallback);

        if (m_waitCallback(this) == false) return; // 繼續等馬達或按鈕

        m_waitCallback = nullptr; // 任務完成

        // 🌟 如果剛才是在「等按鈕」(Cycle Start)，現在按鈕解開了，
        // 代表操作員要開始跑這行了，直接 return 進入底下解析派發！
        if (wasWaitingForStart) {
            return;
        }

        // 🌟 動作跑完了 (例如 G00 移動到位或 M00 完成)
        // 1. 先安全推進 PC 到下一行 (讓 UI 畫面精準亮起下一行)
        if (m_state != NCState::ALARM && m_state != NCState::P_END && !m_programChanged) {
            advancePC();
        }

        // 2. 如果這行有暫停要求 (M00/M01 或 單步模式)
        if (m_pauseAfterBlock && m_state != NCState::ALARM && m_state != NCState::P_END) {
            m_pauseAfterBlock = false;
            m_state = NCState::HOLD;                    // 切換為暫停
            m_waitCallback = WaitForCycleStartCallback; // 掛上「等待 Start 按鈕」
            return; // 結束本回合，定格在下一行！
        }

        return; // 防暴衝：本回合結束，下一毫秒才處理下一行
    }

    // 預讀閘門：容量限制
    if (m_motion.GetQueueSize() >= 50)
    {
        return;
    }

    if (m_waitCallback == nullptr)
    {
        bool currentIsMacro = !m_macroStack.empty();
        int currentPC = currentIsMacro ? m_macroStack.back().currentPC : GetBasePC();
        const std::vector<std::string>& currentMemory = currentIsMacro ? m_macroStack.back().memory : GetBaseMemory();

        // ==========================================================
        // --- 結束判斷 (檔尾到達) ---
        // ==========================================================
        if (currentPC >= currentMemory.size())
        {
            if (m_motion.GetQueueSize() > 0 || !m_motion.IsGroupStandstill()) return;

            if (currentIsMacro) {
                ReturnMacro(); // 副程式結束，返回主程式
            }
            else {
                // 🌟 主程式結束：行號歸 0，設定 P_END，徹底卡住！
                m_macroStack.clear();
                GetBasePC() = 0;
                Reset_Gode();
                UpdateSystemVariables();
                m_state = NCState::P_END;
            }
            return;
        }

        m_programChanged = false;
        m_pauseAfterBlock = false;
        std::string rawLine = currentMemory[currentPC];

        NCBlock block = Parser.ParseLine(rawLine);

        // 選擇性跳躍 (Block Skip '/')
        if (block.isBlockSkip && m_isBlockSkipEnabled) {
            // 直接略過
        }
        else
        {
            if (block.isGoto)
            {
                int targetN = block.gotoTarget;
                bool found = false;
                for (int i = 0; i < (int)currentMemory.size(); i++) {
                    if (currentMemory[i].find('N') != std::string::npos || currentMemory[i].find('n') != std::string::npos) {
                        NCBlock checkBlock = Parser.ParseLine(currentMemory[i]);
                        if (checkBlock.has('N') && (int)checkBlock.val('N') == targetN) {
                            setPC(i);
                            found = true;
                            break;
                        }
                    }
                }
                if (!found) {
                    AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
                    m_state = NCState::ALARM;
                    return;
                }
                m_programChanged = true;
                if (m_isSingleBlockEnabled) m_pauseAfterBlock = true;
            }
            else
            {
                bool isBarrier = false;
                if (block.mCount > 0) {
                    int m = block.mCode[0];
                    if (m == 98 || m == 99 || m == 0 || m == 1 || m == 2 || m == 30) isBarrier = true;
                }

                isBarrier = isBarrier || (block.hasG && (
                    block.gCode == 0 || block.gCode == 12 || block.gCode == 4 ||
                    block.gCode == 7 || 
                    block.gCode == 20 || block.gCode == 21 ||
                    block.gCode == 22 || block.gCode == 23 ||
                    block.gCode == 28 || block.gCode == 30 ||
                    block.gCode == 32 || block.gCode == 53 || block.gCode == 161 ||
                    block.gCode == 65 || block.gCode == 66 || block.gCode == 67 || block.gCode == 92 ||
                    (block.gCode >= 54 && block.gCode <= 59) ||
                    (block.gCode >= 154 && block.gCode <= 159) ||
                    (block.gCode >= 254 && block.gCode <= 259) ||
                    (block.gCode >= 354 && block.gCode <= 359) ||
                    (block.gCode >= 454 && block.gCode <= 459) ||
                    (block.gCode >= 554 && block.gCode <= 559) ||
                    (block.gCode >= 654 && block.gCode <= 659) ||
                    (block.gCode >= 754 && block.gCode <= 759) ||
                    (block.gCode >= 854 && block.gCode <= 859) ||
                    (block.gCode >= 954 && block.gCode <= 959)
                    ));

                if (block.hasG && block.gCode == 0 && block.has('P') && block.val('P') == 1) {
                    isBarrier = false;
                }

                if (m_isSingleBlockEnabled && (block.hasG || block.mCount > 0 || block.has('X') || block.has('Y') || block.has('Z'))) {
                    isBarrier = true;
                }

                if (isBarrier && (m_motion.GetQueueSize() > 0 || !m_motion.IsGroupStandstill())) {
                    return;
                }

                bool wasMainProgram = m_macroStack.empty();

                // 貼標籤邏輯
                int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
                int currentBrainToolMode = CoordSys.toolLengthMode;
                int currentBrainHCode = CoordSys.currentHCode;
                int curTRad = CoordSys.toolRadiusMode;
                int curD = CoordSys.currentDCode;
                bool curIsAbs = CoordSys.isAbsoluteMode;
                bool curG68 = CoordSys.isG68Active;
                double curG68Angle = CoordSys.g68Angle;
                bool curG168 = CoordSys.isWorkpieceRotationActive;
                int curWCode = CoordSys.currentWCode;
                bool curG51 = CoordSys.isScalingActive;
                double curScale = CoordSys.scaleFactor;
                uint8_t curMirrorMask = 0;
                for (int i = 0; i < 8; i++) {
                    if (CoordSys.isMirrorActive[i]) curMirrorMask |= (1 << i);
                }
                bool curG16 = CoordSys.isPolarCoordinateActive;
                bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
                int curPlane = CoordSys.activePlane;

                m_motion.SetNextCommandState(currentPC, currentBrainWCS, currentBrainToolMode, currentBrainHCode, curTRad, curD, curIsAbs, curG68, curG68Angle, curG168, curWCode, curG51, curScale, curMirrorMask, curG16, curG162, curPlane);

                if (!block.isEmpty) {
                    ExecuteBlock(block);
                }

                if (AlarmManager::GetInstance().HasAlarm()) {
                    m_state = NCState::ALARM;
                    if (m_mode == NCOperationMode::MANUAL) m_manualAutoRunning = false;
                    return;
                }

                // ==========================================================
                 // 🌟 【國際標準】：G66 模態巨集自動攔截網 
                 // ==========================================================
                if (m_isG66Active && block.gCode != 66 && block.gCode != 67)
                {
                    // 🌟 一句話呼叫過濾器，取代原本又臭又長的判斷式！
                    bool isRealMotion = IsRealMotionBlock(block);

                    // 【防無限遞迴護城河】：必須確保目前在大腦的主程式層級
                    if (isRealMotion && m_macroStack.empty())
                    {
                        std::string macroFile = "O" + std::to_string(m_g66P) + ".nc";

                        if (CallMacro(macroFile))
                        {
                            // 設定 L 重複次數
                            m_macroStack.back().repeatCount = m_g66L;

                            // 傳遞區域變數
                            for (int i = 0; i < 26; i++) {
                                char c = 'A' + i;
                                if (c != 'P' && c != 'G' && c != 'L' && m_g66Block.has(c)) {
                                    MacroSys.SetVar('#', i + 1, m_g66Block.val(c));
                                }
                            }
                        }
                    }
                }

                // 🌟 M 碼暫停與結束旗標設定
                if (block.mCount > 0) {
                    int m = block.mCode[0];
                    if (m == 99 && wasMainProgram) {
                        GetBasePC() = 0;
                        m_programChanged = true;
                    }
                    // 🌟 【關鍵修正】：移除了 m == 0！
                    // M00 已經由底層 PLC 完美暫停了，大腦不需要再多管閒事掛上第二道鎖！
                    else if (m == 1 && m_isOptionalStopEnabled) {
                        m_pauseAfterBlock = true; // 只有 M01 是大腦自己攔截
                    }
                    else if (m == 2 || m == 30) {
                        // 🌟 M02/M30：主程式結束！直接行號歸 0，切換 P_END 並立刻 return 斬斷！
                        m_macroStack.clear();
                        GetBasePC() = 0;
                        Reset_Gode();
                        UpdateSystemVariables();

                        // 🌟 依據模式決定去留
                        if (m_mode == NCOperationMode::MANUAL) {
                            m_manualAutoRunning = false;
                            m_state = NCState::READY; // 手動巨集結束，回到 READY
                        }
                        else if (m_mode == NCOperationMode::MDI) {
                            m_state = NCState::READY; // MDI 結束，回到 READY
                        }
                        else {
                            m_state = NCState::P_END; // 主程式結束，才卡在 P_END
                        }
                        return; // 斬斷本回合！
                    }
                }
            }
        }

        // 單步模式，要求這行跑完後暫停
        if (m_isSingleBlockEnabled && m_state != NCState::P_END) {
            m_pauseAfterBlock = true;
        }

        // 🌟 派發後收網處理
        if (m_waitCallback == nullptr) {
            if (m_pauseAfterBlock || m_programChanged) {
                // 掛上等待馬達清空的 Callback，下一毫秒就會回到階段 A 放行並定格
                m_waitCallback = WaitAndClearQueueCallback;
            }
            else {
                advancePC(); // 沒事，直接推進下一行
            }
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
        //DEBUG_PRINT("[Alarm] Multiple G-Codes in a single block! Found: %d\n", block.gCount);
        AlarmManager::GetInstance().Trigger(AlarmManager::G_code_Count_Error);
        m_state = NCState::HOLD;
        return; // 直接中止
    }

    // ==========================================
    // 🌟 安全性檢查 2：單節是否包含多個 M 碼
    // ==========================================
    if (block.mCount > 1) {
        //DEBUG_PRINT("[Alarm] Multiple M-Codes in a single block! Found: %d\n", block.mCount);

        // 建議未來可以在 AlarmManager 新增一個 M_CODE_CONFLICT 警報
        // 目前先借用 SYNTAX_ERROR
        AlarmManager::GetInstance().Trigger(AlarmManager::M_code_Count_Error);
        m_state = NCState::HOLD;
        return;
    }


    // ==========================================
    // 1. 瞬間完成的設定 (不需等待)
    // ==========================================
    if (block.has('E')) 
    {
        // m_edmManager.ApplyE(block.val('E'));
    }
    if (block.has('B')) 
    {
        // m_edmManager.ApplyB(block.val('B'));
    }
    // 範例：在處理單節含 T 碼時呼叫
    if (block.has('T'))
    {
        int tVal = (int)block.val('T');
        CoordSys.SetToolNumber(tVal, this);
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
        case 20: case 21:
        case 22: case 23:
        case 43: case 44: case 49:
        case 17: case 18:case 19:
        case 65:  case 66: case 67:
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
            //DEBUG_PRINT("[Alarm] Unsupported G-Code: G%02d\n", block.gCode);
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

   

    // 🌟 在單節解單/發包完成後，立刻刷一次系統變數！
    UpdateSystemVariables();
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
            //DEBUG_PRINT("[NC Warning] MDI Input truncated to %zu lines.\n", MAX_MDI_LINES);
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
        //DEBUG_PRINT("[Alarm] Manual Auto string exceeds %zu bytes limit!\n", MAX_MANUAL_AUTO_BYTES);
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
        //DEBUG_PRINT("[NC Warning] Cannot load dynamic code in current OP mode!\n");
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

    //DEBUG_PRINT("[NC] Dynamic Code Loaded, Lines: %d\n", (int)targetMemory->size());
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
 // 🔌 2. External Machine Ready Interlock
 //
 // NCManager 不知道來源是 PLC C11。
 // 它只知道外部條件目前是否允許機台 Ready。
 //
 // false:
 //     Machine / Servo Power 條件尚未成立
 //
 // true:
 //     才繼續檢查各實體軸 Servo On
 // ----------------------------------------------------
    if (!m_externalReadyInterlock)
    {
        return EDMState::NOT_READY;
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



// 1. 等待馬達靜止
bool NCManager::WaitAndHoldCallback(NCManager* nc) {
    if (nc->m_motion.GetQueueSize() > 0 || !nc->m_motion.IsGroupStandstill()) return false;
    return true;
}

// 2. 🌟 專門等待操作員按下 Cycle Start 的卡點
bool NCManager::WaitForCycleStartCallback(NCManager* nc) {
    if (nc->m_state == NCState::RUN) {
        return true; // 操作員按下 Start 了！解除卡點！
    }
    return false; // 還沒按，繼續乖乖卡住
}

// 3. 只等待馬達靜止 (清空預讀)
bool NCManager::WaitAndClearQueueCallback(NCManager* nc) {
    if (nc->m_motion.GetQueueSize() > 0 || !nc->m_motion.IsGroupStandstill()) return false;
    return true;
}
// ==========================================================
// 🌟 G 碼屬性過濾器：判斷是否為真實的移動指令
// ==========================================================
bool NCManager::IsRealMotionBlock(const NCBlock& block)
{
    // 1. 如果根本沒有座標字元，絕對不可能是移動
    if (!block.has('X') && !block.has('Y') && !block.has('Z')) {
        return false;
    }

    // 2. 如果單節裡面有明確的 G 碼，我們來過濾「非移動」的特例
    if (block.hasG) {
        switch (block.gCode) {
        case 4:   // G04 暫留 (X 代表時間)
        case 10:  // G10 參數寫入 (X 代表寫入數值)
        case 50:
        case 51:  // G51 縮放 (X 代表縮放中心)
        case 52:  // G52 局部座標系設定 (X 代表偏移量)
        case 68:
        case 69:  // G68 座標旋轉 (X 代表旋轉中心)
        case 92:  // G92 座標設定 (X 代表指定座標)
        case 65:
        case 66:
        case 67:  // 巨集呼叫本身
            return false; // 🛑 這些是「帶有座標參數但不會移動」的 G 碼，攔截！
        }
    }

    // 3. 排除上面的例外後，只要帶有 XYZ，我們就視為真正的移動指令！
    // (例如 G00, G01, 或是單純只有 X10. 的模態移動)
    return true;
}