// 檔案：EtherCatMaster_Run.cpp
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <stdio.h>
#include "SHMManager.h"
#include "HMI_Bridge.h"    // 🌟 1. 引入橋接器
#include "AlarmManager.h"
#include "PLCManager.h" // 🌟 1. 引入 PLC 管理器標頭檔
#include "NCPLCManager.h"

int EtherCatMaster::RunRealTimeCycle_EDM_SINKER_MODE()//主要程式迴圈執行 EDM模式
{
    GlobalConfig& globalConfig = GlobalConfig::GetInstance();

    Get_TotalSlave_WKC_Count();//取得從站WKC 分數

    if (StartDcPdoRuntime() != 0)//啟動PDO作業 DC同步
    {
        return -1;
    }


    if (StartPLCRuntime() != 0)//啟動PLC作業 
    {
        return -1;
    }


   

    bool isSuccess = PDO_SendCommandAndWait(EcatCmdType::CMD_SET_STATE, 0x0000, 0x0000, 0x00, 0x0008, 2, 1000);//廣播切換OP狀態 PDO傳送

    if (isSuccess == true)
    {
        DEBUG_PRINT("[System] System is now in OP Mode. (Success)\n");
    }
    else
    {
        DEBUG_PRINT("[Error] Failed to switch to OP Mode! (Timeout or Error)\n");
    }



    m_Motion.Link(&m_ServoList, &m_Axes);//綁定硬體指標


    // 檢查硬體數量與設定檔是否一致
    if (m_ServoList.size() != m_Axes.size())
    {
        DEBUG_PRINT("Error Servo Count !>>m_ServoList>>%d>>m_Axes>>%d\n", m_ServoList.size(), m_Axes.size());
    }


    //共享記憶體初始化-------------------------------------------
    SHMManager::GetInstance().Initialize("EDM_SINKER_MODE");
    SHM_Data* pShm = SHMManager::GetInstance().GetData();// 把指針交給 NCManager



    m_Motion.ResetAllFaults();//全軸 清除異常狀態

    NCPLCManager ncPLCManager(*m_NC, m_Motion, m_plcManager);// PLC <-> NC Interface Manager


    //主控迴圈-------------------------------------------------------------
    while (1)
    {

        if (m_NC->Close_System_Com_flag == true)//關閉核心命令
        {
            m_NC->CoordSys.SaveAllParameters();//儲存座標系統相關參數

            g_PLC->Close_PLC();//關閉PLC作業
            SHMManager::GetInstance().Shutdown();//關閉共享記憶體
            DEBUG_PRINT("Close System！\n");
            return 0;
        }

        RtSleep(10);
        HMI_Bridge::ProcessTask(m_NC);//高速API共享記憶體作業任務
        ncPLCManager.Process();
        m_NC->ProcessTask();//NC系統作業呼叫


        tickCount_RunRealTimeCycle += 40;
        timer_10ms += 10;
        timer_100ms += 10;
        timer_500ms += 10;   // 🌟 增加 500ms 的計時器累加
        timer_1000ms += 10;
        timer_5000ms += 10;
        timer_10000ms += 10;
        Debug_test_timer += 10; // 測試專用時間軸 


        //1s
        if (tickCount_RunRealTimeCycle % 4000 == 0)
        {
            //DEBUG_PRINT(">>> [1s] WKC:%d | Timeouts:%d | Err:%d |\n", wkc_PDO, timeout_count_PDO, wkc_error_count_PDO);
        }




        //10ms
        if (timer_10ms >= 10)
        {
            timer_10ms = 0; // 執行完立刻歸零
            timer_10ms_Count += 1;
            //DEBUG_PRINT("10ms\n");
        }

        //100ms
        if (timer_100ms >= 100)
        {
            timer_100ms = 0; // 執行完立刻歸零
            timer_100ms_Count += 1;
            HMI_Bridge::ProcessTask_100ms(m_NC); // 呼叫 100ms 任務
            //DEBUG_PRINT("100ms\n");
        }

        // 🌟 500ms 新增區塊
        if (timer_500ms >= 500)
        {
            timer_500ms = 0;
            timer_500ms_Count += 1;
            HMI_Bridge::ProcessTask_500ms(m_NC); // 呼叫 500ms 任務
        }

        //1000ms
        if (timer_1000ms >= 1000)
        {
            timer_1000ms = 0; // 執行完立刻歸零
            timer_1000ms_Count += 1;
            HMI_Bridge::ProcessTask_1000ms(m_NC); // 呼叫 1000ms 任務
            //DEBUG_PRINT("1000ms\n");


            PrintDcRuntimeDiagnostics();//DC診斷訊息
        }

        //5000ms
        if (timer_5000ms >= 5000)
        {
            timer_5000ms = 0; // 執行完立刻歸零
            timer_5000ms_Count += 1;
            
            //DEBUG_PRINT("5000ms\n");


            //PrintDcRuntimeDiagnostics();//DC診斷訊息
        }

        //10000ms
        if (timer_10000ms >= 10000)
        {
            timer_10000ms = 0; // 執行完立刻歸零
            timer_10000ms_Count += 1;

            //DEBUG_PRINT("5000ms\n");


           
        }

        if (Debug_test_timer >= 1000)
        {
            Debug_test_timer = 0; // 執行完立刻歸零
            Debug_test_timer_Count += 1;
            //DEBUG_PRINT("Debug_test_timer ms\n");
        }

    }



}
