#pragma once
#include "NC_Types.h"
#include "MotionCore.h"
#include "HomingManager.h"
#include "CoordinateManager.h"
#include "MacroEngine.h"       // 必須要有
#include "MacroParser.h"       // 必須要有
#include "GCodeParser.h"       // 🌟 解決 Parser 找不到的關鍵！

#include <queue>
#include <vector>
#include <string>
#include <map>                 // 🌟 補上 map，因為跳躍表 m_jumpTable 會用到
#include <stack>               // 🌟 新增：為了支援副程式返回堆疊
class NCManager;

// 🌟 終極解法：定義一個「檢查條件」的函數指標。
// 回傳 true 代表條件滿足 (等待結束)，false 代表繼續等
using WaitConditionFunc = bool (*)(NCManager* nc);

class NCManager {
public:
    NCManager(MotionCore& motion);

    // 1. 系統狀態控制
    void ChangeMode(NCOperationMode newMode);
    void ChangeState(NCState newState);

    // =========================================================
// NC State Query
//
// 提供給外部控制層（例如 NCPLCManager）判斷
// NC 當前狀態。
// =========================================================
    NCState GetState() const
    {
        return m_state;
    }

    NCOperationMode GetMode() const
    {
        return m_mode;
    }

    // 🌟 新增功能：載入 NC 程式檔
    bool LoadProgram(const std::string& filepath);

    // 🌟 新增：呼叫與返回副程式的介面
    bool CallMacro(const std::string& filename);
    void ReturnMacro();

    // 2. 指令交握介面 (給 HMI 人機介面呼叫的)
    void CycleStart();  // 按下啟動鍵
    void FeedHold();    // 按下暫停鍵
    void Reset();       // 按下重置鍵
    // =========================================================
// External Machine Ready Interlock
//
// NCManager 不知道這個訊號來自 PLC C11、Safety PLC、
// EtherCAT 或其他來源。
//
// true  = 外部條件允許 NC 運轉
// false = 外部條件尚未 Ready
// =========================================================

    void SetExternalReadyInterlock(bool ready)
    {
        m_externalReadyInterlock = ready;
    }

    bool IsExternalReadyInterlock() const
    {
        return m_externalReadyInterlock;
    }

    void Reset_Gode();       // 重置G碼相關
    bool IsFeedHoldActive() const {
        return m_state == NCState::HOLD;
    }



    // 3. 接收解碼器傳來的單節指令 (MDI 或 自動模式)
    void PushBlock(const NCBlock& block);

    // 4. 核心執行緒 (放在 Main Loop 執行)
    void ProcessTask();

    // ======================================================
    // 🌟 5. 新增：資源存取介面 (讓外部的 Handler 檔案可以操作系統資源)
    // ======================================================
    MotionCore& GetMotion() { return m_motion; }

    // 開放 Ticks 讓外部檢查函式可以使用
    void SetG04TimeMs(double ms) { m_G04_TimeMs = ms; }
    double GetG04TimeMs() const { return m_G04_TimeMs; }


    int GetSimulatedTicks() const { return m_simulatedTicks; }
    void SetSimulatedTicks(int ticks) { m_simulatedTicks = ticks; }

    CoordinateManager& GetCoordSys() { return CoordSys; }

    // --- 子系統 ---
    CoordinateManager CoordSys;
    MacroEngine MacroSys;       // 變數引擎
    MacroParser MathParser;     // 🌟 2. 補上數學解譯器 (夾在中間)
    GCodeParser Parser;         // 字串翻譯官

    // ⚠️ 註解掉舊的 ExecState，因為我們已經全面採用頂端的 WaitState 來做狀態機了
    // enum class ExecState { RUNNING, WAITING_DWELL, WAITING_SYNC };
    // ExecState m_execState = ExecState::RUNNING;

    // ======================================================
    // 🌟 動態軸對應系統 (Dynamic Axis Mapping)
    // ======================================================
    char m_axisNames[8]; // 開機時從 ini 讀入：{'X','Y','Z','C','U','V','A','W'}

    // 從 D 槽讀取 AXIS_CFG.ini
    void LoadAxisConfiguration();

    // 🌟 核心 API：傳入英文字母 (如 'X')，回傳它是 0~7 的哪一軸。找不到回傳 -1。
    int GetAxisIndex(char gcodeLetter) const;


    // ======================================================
    // Dynamic Axis Mapping Query
    //
    // 回傳 AXIS_CFG.ini 對應的軸代號。
    // 例如 Axis3=C -> GetAxisName(3) == 'C'。
    // 無效或未設定則回傳 '?'。
    // ======================================================
    char GetAxisName(int axisIndex) const;


    const size_t MAX_MDI_LINES = 20;                        // MDI 模式最大行數
    const size_t MAX_MANUAL_AUTO_BYTES = 1024 * 1024;       // MANUAL 自動模式最大字串 (1024KB = 1MB)

    // 🌟 模式專用 API
    bool LoadMDI(const std::string& mdiContent);
    bool LoadManualAuto(const std::string& manualContent);

    // 🌟 任務分流函式
    void ProcessExecutionEngine();
    void ProcessManualMode(); // 只保留純手動 JOG 的部分

    // 🌟 新增：將 NC 狀態同步給 PLC
    void SyncNCStateToPLC();


    // 🌟 1. 新增：面板操作功能開關 (可由 HMI 或 PLC 同步設定)
    bool m_isSingleBlockEnabled = false;   // 單步執行
    bool m_isOptionalStopEnabled = false;  // 選擇性暫停 (M01)
    bool m_isBlockSkipEnabled = false;     // 選擇性跳躍 (/)


    // =========================================================
// Single Block
// =========================================================

    void SetSingleBlockEnabled(bool enabled)
    {
        m_isSingleBlockEnabled = enabled;
    }

    bool IsSingleBlockEnabled() const
    {
        return m_isSingleBlockEnabled;
    }


    // =========================================================
    // Optional Stop
    // =========================================================

    void SetOptionalStopEnabled(bool enabled)
    {
        m_isOptionalStopEnabled = enabled;
    }

    bool IsOptionalStopEnabled() const
    {
        return m_isOptionalStopEnabled;
    }


    // =========================================================
    // Block Skip
    // =========================================================

    void SetBlockSkipEnabled(bool enabled)
    {
        m_isBlockSkipEnabled = enabled;
    }

    bool IsBlockSkipEnabled() const
    {
        return m_isBlockSkipEnabled;
    }

    static bool WaitAndHoldCallback(NCManager* nc);
    static bool WaitAndClearQueueCallback(NCManager* nc);
    static bool WaitForCycleStartCallback(NCManager* nc); // 新增：專等 CycleStart 按鈕
private:
    // 🌟 新增：統一暫停旗標 (用來標記這行跑完後是否需要停下來)
    bool m_pauseAfterBlock = false;
    // 🌟 判斷這行單節是否為「真的會產生機台移動」的指令
    bool IsRealMotionBlock(const NCBlock& block);
public:
    uint32_t NC_RunCount;//NC執行迴圈數
    uint32_t API_RunCount;//API執行迴圈數
    MotionCore& m_motion;

    // =========================================================
    // G81 HOME Manager
    //
    // NCManager 擁有一個 HomingManager。
    //
    // HomingManager：
    //     負責 G81 HOME 流程 / 狀態機 / 多軸排程。
    //
    // MotionCore：
    //     仍負責真正的軸運動。
    //
    // 這裡直接使用 m_motion 建立，
    // 所以不需要另外 new / delete。
    // =========================================================

    HomingManager Homing{ m_motion };

    NCOperationMode m_mode = NCOperationMode::MANUAL;
    NCState m_state = NCState::NOT_READY;
    EDMState m_edmState = EDMState::NOT_READY;


    // =========================================================
// External Machine Ready Interlock
// =========================================================
    bool m_externalReadyInterlock = false;

    bool Close_System_Com_flag = 0;//關閉核心命令

    // 🌟 新增：主程式與副程式追蹤變數
    std::string m_mainProgramName = "";
    std::string m_macroProgramName = "";
    int m_macroProgramPC = -1; // -1 代表目前沒有在執行副程式


  // ==========================================
    // 🌟 新增：多層副程式 (Macro) 執行框架結構
    // ==========================================
    struct MacroFrame {
        std::string programName;            // 這層副程式的檔名 (例如 O1234.nc)
        std::vector<std::string> memory;    // 這層副程式的程式碼內容
        int currentPC;                      // 這層目前跑到第幾行
        int returnPC;                       // 執行完 M99 要回傳給上一層的行號

        // 🌟 新增：為了支援 G65/G66/M98 的 L 次數準備
        int repeatCount = 1;
    };

    // 🌟 這是解決錯誤的關鍵：用來儲存最多 8 層的副程式堆疊
    // (請把舊的 m_macroMemory 和 m_returnStack 刪掉，換成這個)
    std::vector<MacroFrame> m_macroStack;

    bool m_programChanged = false;          // 🌟 標記是否發生了程式跳轉 (M98/M99)

    // 🌟 修改：現在記憶體存的是「原始字串」，以支援執行時動態計算
    std::vector<std::string> m_programMemory;

    // 🌟 新增：跳躍表 (紀錄 N 碼對應的陣列索引)
    std::map<int, int> m_jumpTable;
    int m_programPC = 0;                  // Program Counter (目前跑到第幾行)

    // NC 指令緩衝區
    std::queue<NCBlock> m_blockQueue;

    // 內部執行功能
    void ExecuteBlock(const NCBlock& block);

    // 🌟 替換：捨棄 Enum，改用統一的檢查回呼函式
    WaitConditionFunc m_waitCallback = nullptr;

    int m_simulatedTicks = 0; // (測試用) 模擬馬達跑了多久
    double m_G04_TimeMs = 0.0;


    // 🌟 MDI 專屬變數
    std::vector<std::string> m_mdiMemory;
    int m_mdiPC = 0;

    // 🌟 MANUAL (輕量自動) 專屬變數
    std::vector<std::string> m_manualMemory;
    int m_manualPC = 0;
    bool m_manualAutoRunning = false; // 標記目前是否正在跑 MANUAL 的自動指令


    // 🌟 核心設計：動態獲取當前模式的「基準行號」與「基準記憶體」
    int& GetBasePC();
    std::vector<std::string>& GetBaseMemory();

    bool LoadDynamicCode(const std::string& content);

    // 🌟 新增：提供給 HMI 狀態廣播用的動態指標
    int GetActivePC() {
        return GetBasePC();
    }

    EDMState GetMachineEDMState();
    // 取得已經存好的狀態變數
    EDMState GetCurrentEDMState() const { return m_edmState; }


    // 🌟 集中更新所有 $ 變數 (供 G00, G01, G92, Reset 結束時即時呼叫)
    void UpdateSystemVariables();

    int UpdateSystemVariables_initialize_flag = 0;

    // 🌟 G66 模態巨集專用狀態
    bool m_isG66Active = false; // G66 開關
    int m_g66P = 0;             // 記住呼叫的副程式名稱 (P)
    int m_g66L = 1;             // 記住重複次數 (L)
    NCBlock m_g66Block;         // 記住 G66 當下夾帶的所有變數 (A, B, C...)




};