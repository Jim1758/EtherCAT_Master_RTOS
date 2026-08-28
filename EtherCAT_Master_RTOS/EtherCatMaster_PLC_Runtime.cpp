#include "EtherCatMaster.h"
#include "EtherCatMaster_DC_Internal.h"
#include "GlobalConfig.h"
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>
#include <stdio.h>





//PLC 中斷作業----------------------------------------------------------
void RTAPI GlobalTimerHandler_PLC(void* nContext)
{
    EtherCatMaster* pMaster =
        (EtherCatMaster*)nContext;

    if (pMaster == nullptr)
    {
        return;
    }


    // =========================================================
    // PLC Cycle
    //
    // 注意：
    //
    // PLC Handler 不直接碰 EtherCAT IO Map。
    //
    // EtherCAT Physical I/O：
    // 由 250us PDO Handler 統一管理。
    //
    // PLC Handler：
    // 只負責 Shadow Buffer <-> PLC Logic。
    // =========================================================


    // =========================================================
    // 1. Shadow Input -> PLC Virtual I/O
    // =========================================================

    pMaster->m_Plc.SyncPhysicalToVirtual();


    // =========================================================
    // 2. PLC Logic
    // =========================================================

    if (g_PLC != nullptr)
    {
        g_PLC->RunCycle(1);
    }


    // =========================================================
    // 3. PLC Virtual Output -> Shadow Output
    //
    // 注意：
    //
    // 這裡只更新 Shadow Output。
    //
    // 不要在 PLC Handler FlushOutputs。
    //
    // 下一次 PDO 250us Cycle 開始時，
    // PDO Handler 會統一 FlushOutputs。
    // =========================================================

    pMaster->m_Plc.SyncVirtualToPhysical();


    pMaster->tickCount_PLC++;

    //跑馬燈測試----------------------------------------


    if (pMaster->tickCount_PLC % 30 == 0)//50
    {
       // pMaster->m_Plc.Update_Debug();
    }
}

