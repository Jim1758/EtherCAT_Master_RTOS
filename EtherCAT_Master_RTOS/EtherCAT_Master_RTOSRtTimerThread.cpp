//////////////////////////////////////////////////////////////////
//
// EtherCAT_Master_RTOSRtTimerThread.cpp - cpp 檔案
//
// 此檔案由 Visual Studio 的 RTX64 應用程式範本生成。
//
// 建立時間: 2026/1/22 上午 11:13:21 
// 使用者: Jim
//
//////////////////////////////////////////////////////////////////

#include "EtherCAT_Master_RTOS.h"

// RTX64 週期性定時器處理函式 (Timer Handler)。
// 欲參考更多範例，請查看 RTX64 安裝目錄下的 Samples 資料夾。
//
void
RTFCNDCL
TimerHandler(PVOID context)
{
    // 待辦：
    // 在此處撰寫您的定時器處理程式碼。

    // 💡 技術提示：
    // 1. 此函式在高優先權的即時執行緒中運行，每 500us 觸發一次。
    // 2. 適合放置：EtherCAT PDO 資料交換、馬達位置插補運算、放電電壓狀態監控。
    // 3. 禁忌：嚴禁在此呼叫會導致「阻塞」(Blocking) 的 API (如 printf, 檔案讀寫)，
    //    這會導致時基抖動 (Jitter)，甚至造成系統崩潰。
}