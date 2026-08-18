#include "EtherCatMaster.h"
#include "EtherCatMaster_DC_Internal.h"
#include "GlobalConfig.h"
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>
#include <stdio.h>



int EtherCatMaster::StartPLCRuntime()
{
    //PLC 中斷宣告---------------------------------------------------------------

    if (m_plcManager.LoadLogicProgram(GlobalConfig::GetInstance().PLC_Dir + "logic.bin") == false)
    {
        DEBUG_PRINT("LoadLogicProgram PLC Error!\n");
        return -1;
    }



    HANDLE hTimer_PLC = NULL;// 用來存放計時器的 Handle
    LARGE_INTEGER liPeriod_PLC;
    liPeriod_PLC.QuadPart = 10000; // 1ms
    hTimer_PLC = RtCreateTimer(NULL, 0, GlobalTimerHandler_PLC, this, 60, CLOCK_2);//PSECURITY_ATTRIBUTES,StackSize,pRoutine,Context,Priority (請填入一個優先權數值，0~127),Clock

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
    return 0;
}


