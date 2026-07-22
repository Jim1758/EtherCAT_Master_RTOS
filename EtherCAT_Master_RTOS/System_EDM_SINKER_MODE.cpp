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


int EtherCatMaster::RunRealTimeCycle_EDM_SINKER_MODE()//主要程式迴圈執行 EDM模式
{
    Get_TotalSlave_WKC_Count();//取得從站WKC 分數

     //PDO 中斷宣告---------------------------------------------------------------
    HANDLE hTimer_PDO = NULL;
    LARGE_INTEGER liPeriod_PDO;
    liPeriod_PDO.QuadPart = 2500; // 250us
    hTimer_PDO = RtCreateTimer(NULL, 0, GlobalTimerHandler_PDO, this, 64, CLOCK_2);//PSECURITY_ATTRIBUTES,StackSize,pRoutine,Context,Priority (請填入一個優先權數值，0~127),Clock
    if (hTimer_PDO == NULL)
    {
        DEBUG_PRINT("GlobalTimerHandler_PDO Error>>%d\n", GetLastError());
        return -1;
    }
    else
    {
        if (RtSetTimerRelative(hTimer_PDO, &liPeriod_PDO, &liPeriod_PDO) == false)
        {
            DEBUG_PRINT("GlobalTimerHandler_PDO Error>>RtSetTimerRelative\n");
            return -1;
        }
    }

    //PLC 中斷宣告---------------------------------------------------------------
    HANDLE hTimer_PLC = NULL;// 用來存放計時器的 Handle
    LARGE_INTEGER liPeriod_PLC;
    liPeriod_PLC.QuadPart = 10000; // 1ms
    hTimer_PLC = RtCreateTimer(NULL, 0, GlobalTimerHandler_PLC, this, 63, CLOCK_2);//PSECURITY_ATTRIBUTES,StackSize,pRoutine,Context,Priority (請填入一個優先權數值，0~127),Clock

    if (hTimer_PLC == NULL)
    {
        DEBUG_PRINT("GlobalTimerHandler_PLC Error>>%d\n", GetLastError());
        return -1;
    }
    else
    {
        if (!RtSetTimerRelative(hTimer_PLC, &liPeriod_PLC, &liPeriod_PLC))
        {
            DEBUG_PRINT("GlobalTimerHandler_PLC Error>>RtSetTimerRelative\n");
        }
    }

    bool isSuccess = PDO_SendCommandAndWait(EcatCmdType::CMD_SET_STATE, 0x0000,0x0000,0x00,0x0008,2,1000);//廣播切換OP狀態 PDO傳送

    if (isSuccess==true)
    {
        DEBUG_PRINT("[System] System is now in OP Mode. (Success)\n");
    }
    else
    {
        DEBUG_PRINT("[Error] Failed to switch to OP Mode! (Timeout or Error)\n");
    }

   
    
    m_Motion.Link(&m_ServoList, &m_Axes);//綁定硬體指標

    // 2. 🌟 一鍵載入參數並初始化所有軸！
    std::string axisConfigPath = GlobalConfig::GetInstance().BaseDataDir + "Data\\Parameter\\AxisConfig.txt";
    if (!GlobalConfig::GetInstance().LoadAxisConfig(axisConfigPath, m_Axes, m_Motion)) 
    {
        DEBUG_PRINT("LoadAxisConfig Error！\n");
        return -1;
    }

    // 檢查硬體數量與設定檔是否一致
    if (m_ServoList.size() != m_Axes.size())
    {
        DEBUG_PRINT("Error Servo Count !>>m_ServoList>>%d>>m_Axes>>%d\n",m_ServoList.size(), m_Axes.size());
    }


    
    //共享記憶體初始化-------------------------------------------
    SHMManager::GetInstance().Initialize("EDM_SINKER_MODE");
    SHM_Data* pShm = SHMManager::GetInstance().GetData();// 把指針交給 NCManager


    //載入初始NC檔案---------------------------------------------------------------------
    std::string Initial_NcPath = GlobalConfig::GetInstance().NCProgramDir + "Null.nc";
    m_NC->LoadProgram(Initial_NcPath);
   

    AlarmManager::GetInstance().Trigger((int)AlarmManager::NCAlarm::SYNTAX_ERROR);

    //主控迴圈-------------------------------------------------------------
    while (1)
    {
      
       
        RtSleep(10);
        HMI_Bridge::ProcessTask(m_NC);//共享記憶體作業
        m_NC->ProcessTask();//NC系統作業呼叫


        tickCount_RunRealTimeCycle += 40;
        timer_10ms += 10;
        timer_100ms += 10;
        timer_1000ms += 10;
        Debug_test_timer += 10; // 測試專用時間軸 


        //1s
        if (tickCount_RunRealTimeCycle % 4000 == 0)
        {
           
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
            //DEBUG_PRINT("100ms\n");
        }

        //1000ms
        if (timer_1000ms >= 1000)
        {
            timer_1000ms = 0; // 執行完立刻歸零
            timer_1000ms_Count += 1;
            //DEBUG_PRINT("1000ms\n");
        }


        if (Debug_test_timer >= 1000)
        {
            Debug_test_timer = 0; // 執行完立刻歸零
            Debug_test_timer_Count += 1;
            //DEBUG_PRINT("Debug_test_timer ms\n");
        }

    }
  


}


