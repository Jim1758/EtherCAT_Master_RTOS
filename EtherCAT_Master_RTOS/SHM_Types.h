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
    int32_t SHM_CurrentLine;//目前NC執行到第幾行


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
    bool cycleStart;//執行NC
    bool feedHold;//暫停NC
    bool reset;//重置NC

    // 程式載入控制
    bool loadProgramReq;//載入NC檔案命令
    char loadprogramName[256];// 載入NC檔案名稱

    
    int32_t reserved[16];// 預留擴充空間
};




//總記憶體區塊-------------------------------------------------------
struct SHM_Data 
{
    SHM_API_Status API_Status;//API狀態區塊
    SHM_NC_Status  NC_Status;//NC狀態區塊
    SHM_Alarm_Status Alarm_Status;//警報狀態區塊
    SHM_NC_Command NC_Command;// 命令區塊

};

#pragma pack(pop)