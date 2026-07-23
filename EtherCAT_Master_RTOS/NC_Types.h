#pragma once
#include <cstdint>

// 1. 操作模式
enum class NCOperationMode {
    MEMORY,     // 自動模式 (執行完整的 NC 程式)
    MDI,        // 手動數據輸入 (單行指令執行)
    MANUAL,     // 手動模式 (JOG, MPG 手輪)
    EDIT        // 編輯模式 (預留給人機介面改程式用)
};

// 2. NC 系統狀態
enum class NCState {
    NOT_READY,  // 系統未準備好 (例如尚未回原點或未激磁)
    READY,      // 準備就緒
    IDLE,       // 閒置中
    RUN,        // 執行中
    P_END,      // 程式結束 (M30)
    HOLD,       // 暫停狀態 (Feed Hold)
    RESET_STATE, // 系統重置中
    ALARM        //系統錯誤
};

// 3. EDM 設備狀態
enum class EDMState {
    NOT_READY,
    READY,
    START,      // 放電執行中
    HOLD,       // 放電暫停
    STOP        // 放電停止
};

// 4. 單行指令結構 (GCode 預先解碼後的樣子)
struct NCBlock {
    bool isEmpty = true;
    bool isBlockSkip = false;

    // 🌟 新增：跳躍指令屬性
    bool isGoto = false;
    int gotoTarget = -1;

    // 🌟 2. 擴充：支援多個 G 碼檢查
    int gCount = 0;
    int gCodes[10] = { 0 }; // 預留空間存這行出現的所有 G 碼

    bool hasG = false;
    int gCode = -1;

    int mCount = 0;
    int mCode[3] = { -1, -1, -1 };

    bool hasParam[26] = { false };
    double param[26] = { 0.0 };

    bool has(char letter) const {
        if (letter >= 'A' && letter <= 'Z') return hasParam[letter - 'A'];
        return false;
    }
    double val(char letter) const {
        if (letter >= 'A' && letter <= 'Z') return param[letter - 'A'];
        return 0.0;
    }
};