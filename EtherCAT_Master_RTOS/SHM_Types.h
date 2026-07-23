#pragma once
#include <stdint.h>
#pragma pack(push, 1)


// API狀態區塊-----------------------------------------------------------
struct SHM_API_Status
{
    uint32_t SHM_API_RunCount;//API執行迴圈數

    int32_t reserved[16];// 預留擴充空間
};

// NC狀態區塊-----------------------------------------------------------
struct SHM_NC_Status
{
    uint32_t SHM_NC_RunCount;//NC執行迴圈數
    int32_t SHM_NC_State;//NC狀態
    int32_t SHM_EDM_State;//EDM設備狀態


     // 🌟 1. 主程式追蹤
    char     mainProgName[64];  // 當前主程式名稱
    int32_t  mainCurrentLine;   // 主程式行號

    // 🌟 2. 副程式 (Macro) 追蹤
    char     macroProgName[64]; // 當前副程式名稱
    int32_t  macroCurrentLine;  // 副程式行號



    int32_t reserved[32];// 預留擴充空間
};


//警報狀態區塊 ------------------------------------------------
struct SHM_Alarm_Status
{
    uint32_t alarmUpdateCount;     // 警報更新計數器 (HMI 監控此數值，改變代表有新警報或警報消除)
    int32_t  activeAlarmCount;     // 目前發生中的警報數量
    int32_t  activeAlarms[64];     // 發生中的警報代碼陣列 (最多支援同時 64 個警報)

    int32_t  reserved[16];         // 預留擴充空間
};



//NC命令區塊-----------------------------------------------------------

struct SHM_NC_Command
{
    bool Close_System;//關閉核心系統命令

    bool cycleStart;//執行NC
    bool feedHold;//暫停NC
    bool reset;//重置NC

    // 程式載入控制
    bool loadProgramReq;//載入NC檔案命令
    char loadprogramName[256];// 載入NC檔案名稱

    
    int32_t reserved[16];// 預留擴充空間
};



//寫入Macro變數專用的指令區塊
struct SHM_Var_Write_Command {
    bool writeReq;            // 寫入請求旗標
    char prefix;              // 變數種類 ('#', '@', '$')
    int index;                // 變數編號
    double writeValue;        // 要寫入的值
};

//巨集變數狀態全廣播區塊 (大約 18.4 KB)
struct SHM_Macro_Status {
    int currentCallDepth;                   // 目前所在的副程式層數 (0~7)
    double localVars[8][101];               // 8層，每層 #1~#100 (index 0 不用)
    double globalVars[501];                 // @1~@500 (或 #501~#1000)
    double sysVars[1001];                   // $1~$1000
};


//總記憶體區塊-------------------------------------------------------
struct SHM_Data 
{
    SHM_API_Status API_Status;//API狀態區塊
    SHM_NC_Status  NC_Status;//NC狀態區塊
    SHM_Alarm_Status Alarm_Status;//警報狀態區塊
    SHM_NC_Command NC_Command;// 命令區塊
    SHM_Var_Write_Command varCmd;  // 新增：寫入變數指令
    SHM_Macro_Status macroStatus;  // 新增：變數全廣播區
};

#pragma pack(pop)