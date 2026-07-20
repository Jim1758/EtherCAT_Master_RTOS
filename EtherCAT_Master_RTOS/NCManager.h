#pragma once
#include "NC_Types.h"
#include "MotionCore.h"
#include "CoordinateManager.h"
#include "MacroEngine.h"       // 必須要有
#include "MacroParser.h"       // 必須要有
#include "GCodeParser.h"       // 🌟 解決 Parser 找不到的關鍵！

#include <queue>
#include <vector>
#include <string>
class NCManager {
public:
    NCManager(MotionCore& motion);

    // 1. 系統狀態控制
    void ChangeMode(NCOperationMode newMode);
    void ChangeState(NCState newState);

    // 🌟 新增功能：載入 NC 程式檔
    bool LoadProgram(const std::string& filepath);

    // 2. 指令交握介面 (給 HMI 人機介面呼叫的)
    void CycleStart();  // 按下啟動鍵
    void FeedHold();    // 按下暫停鍵
    void Reset();       // 按下重置鍵

    // 3. 接收解碼器傳來的單節指令 (MDI 或 自動模式)
    void PushBlock(const NCBlock& block);

    // 4. 核心執行緒 (放在 Main Loop 執行)
    void ProcessTask();

    // --- 子系統 ---
    CoordinateManager CoordSys;
    MacroEngine MacroSys;       // 變數引擎
    MacroParser MathParser;     // 🌟 2. 補上數學解譯器 (夾在中間)
    GCodeParser Parser;         // 字串翻譯官

private:
    MotionCore& m_motion;

    NCOperationMode m_mode = NCOperationMode::MANUAL;
    NCState m_state = NCState::NOT_READY;
    EDMState m_edmState = EDMState::NOT_READY;

    // 🌟 修改：現在記憶體存的是「原始字串」，以支援執行時動態計算
    std::vector<std::string> m_programMemory;

    // 🌟 新增：跳躍表 (紀錄 N 碼對應的陣列索引)
    std::map<int, int> m_jumpTable;
    int m_programPC = 0;                  // Program Counter (目前跑到第幾行)


    // NC 指令緩衝區
    std::queue<NCBlock> m_blockQueue;

    // 內部執行功能
    void ExecuteBlock(const NCBlock& block);
};