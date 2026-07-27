#include "HMI_Bridge.h"
#include "SHMManager.h"
#include "GlobalConfig.h" 
#include "NCManager.h"
#include "AlarmManager.h"
#include <cstring> 

namespace HMI_Bridge
{
    // =========================================================================
    // 🚀 1. ProcessTask (每一圈執行) - 處理高優先級命令與即時座標
    // =========================================================================
    void ProcessTask(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // --- 狀態迴圈更新 ---
        pShm->API_Status.SHM_API_RunCount = nc->API_RunCount++;
        pShm->NC_Status.SHM_NC_RunCount = nc->NC_RunCount;

        pShm->NC_Status.SHM_m_mode = static_cast<int32_t>(nc->m_mode);

        pShm->NC_Status.SHM_NC_State = static_cast<int32_t>(nc->m_state);
        pShm->NC_Status.SHM_EDM_State = static_cast<int32_t>(nc->m_edmState);
        pShm->NC_Status.currentWCS_GCode = nc->CoordSys.GetCurrentWCSGCode();
        pShm->NC_Status.isAbsoluteMode = nc->CoordSys.isAbsoluteMode ? 1 : 0;

        // --- 即時座標廣播 ---
        double currentWCS[8] = { 0.0 };
        nc->CoordSys.GetActualWCS(currentWCS);
        for (int i = 0; i < 8; i++)
        {
            pShm->NC_Status.actualMCS[i] = nc->CoordSys.actualMCS[i];
            pShm->NC_Status.actualWCS[i] = currentWCS[i];
        }


        // --- HMI 控制指令交握 ---
        if (pShm->varCmd.writeReq) 
        {
            nc->MacroSys.SetVar(pShm->varCmd.prefix, pShm->varCmd.index, pShm->varCmd.writeValue);
            pShm->varCmd.writeReq = false;
        }
        if (pShm->NC_Command.cycleStart) { nc->CycleStart(); pShm->NC_Command.cycleStart = false; }
        if (pShm->NC_Command.feedHold) { nc->FeedHold(); pShm->NC_Command.feedHold = false; }
        if (pShm->NC_Command.reset) { nc->Reset(); pShm->NC_Command.reset = false; }
        if (pShm->NC_Command.Close_System) { nc->Close_System_Com_flag = true; pShm->NC_Command.Close_System = false; }

       
        if (pShm->Coord_Command.reqSwitchWCS)
        {
            nc->CoordSys.SetWCS(pShm->Coord_Command.targetWCS_GCode, nc);
            pShm->Coord_Command.reqSwitchWCS = false;
        }

      

    }

    // =========================================================================
    // 🐢 2. ProcessTask_100ms - 處理大資料表格與字串 (HMI 畫面刷新用)
    // =========================================================================
    void ProcessTask_100ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;


        // --- 警報檢查 (自帶條件判斷，極快) ---
        uint32_t currentUpdateCount = AlarmManager::GetInstance().GetUpdateCount();
        if (pShm->Alarm_Status.alarmUpdateCount != currentUpdateCount) {
            pShm->Alarm_Status.activeAlarmCount = AlarmManager::GetInstance().GetAlarmCount();
            std::memset(pShm->Alarm_Status.activeAlarms, 0, sizeof(pShm->Alarm_Status.activeAlarms));
            for (int i = 0; i < pShm->Alarm_Status.activeAlarmCount; i++) {
                pShm->Alarm_Status.activeAlarms[i] = AlarmManager::GetInstance().GetAlarmId(i);
            }
            pShm->Alarm_Status.alarmUpdateCount = currentUpdateCount;
        }



        // --- 字串複製 (程式名稱) ---
        std::strncpy(pShm->NC_Status.mainProgName, nc->m_mainProgramName.c_str(), 63);
        pShm->NC_Status.mainProgName[63] = '\0';
       
        pShm->NC_Status.mainCurrentLine = nc->GetActivePC();
        std::strncpy(pShm->NC_Status.macroProgName, nc->m_macroProgramName.c_str(), 63);
        pShm->NC_Status.macroProgName[63] = '\0';
        pShm->NC_Status.macroCurrentLine = nc->m_macroProgramPC;


       
        if (pShm->NC_Command.reqChangeMode)//處理 OP 模式切換請求 (來自 NC_Command)
        {
            nc->ChangeMode(static_cast<NCOperationMode>(pShm->NC_Command.targetMode));
            pShm->NC_Command.reqChangeMode = false;
        }

        if (pShm->NC_Command.loadProgramReq)
        {
            size_t safeLength = strnlen(pShm->NC_Command.loadprogramName, 256);
            std::string NC_loadprogramName(pShm->NC_Command.loadprogramName, safeLength);
            nc->LoadProgram(GlobalConfig::GetInstance().NCProgramDir + NC_loadprogramName + ".nc");
            pShm->NC_Command.loadProgramReq = false;
            std::memset(pShm->NC_Command.loadprogramName, 0, sizeof(pShm->NC_Command.loadprogramName));
        }


        
        if (pShm->String_Command.reqLoadCode)//處理動態程式碼載入請求 (來自 String_Command)
        {
            pShm->String_Command.codeContent[2047] = '\0'; // 安全結尾防溢位
            std::string codeStr(pShm->String_Command.codeContent);

            // 呼叫統一的 API，它會自己看現在是 MDI 還是 MANUAL
            nc->LoadDynamicCode(codeStr);

            // 處理完畢，降下旗標並清空緩衝區
            pShm->String_Command.reqLoadCode = false;
            std::memset(pShm->String_Command.codeContent, 0, sizeof(pShm->String_Command.codeContent));
        }

        if (pShm->Coord_Command.reqWriteOffset)
        {
            int row = pShm->Coord_Command.rowIndex;
            int axis = pShm->Coord_Command.axisIndex;
            double val = pShm->Coord_Command.writeValue;
            if (axis >= 0 && axis < 8)
            {
                switch (pShm->Coord_Command.offsetType)
                {
                case 0: nc->CoordSys.extOffset[axis] = val; break;
                case 1: if (row >= 0 && row < 60)  nc->CoordSys.m_WCSTable[row][axis] = val; break;
                case 2: if (row >= 0 && row < 100) nc->CoordSys.m_ToolOffset[row][axis] = val; break;
                case 3: if (row >= 0 && row < 100) nc->CoordSys.m_WorkOffset[row][axis] = val; break;
                case 10:
                    nc->CoordSys.m_WCSTable[nc->CoordSys.currentWCSIndex][axis] = nc->CoordSys.actualMCS[axis] - nc->CoordSys.extOffset[axis] - val;
                    break;
                }
            }
            pShm->Coord_Command.reqWriteOffset = false;
        }

        if (pShm->Coord_Command.reqSave)
        {
            switch (pShm->Coord_Command.saveType)
            {
            case 0: nc->CoordSys.SaveAllParameters(); break;
            case 1: nc->CoordSys.SaveWCSStatus(); break;
            case 2: nc->CoordSys.SaveExtOffset(); break;
            case 3: nc->CoordSys.SaveWCSTable(); break;
            case 4: nc->CoordSys.SaveToolOffset(); break;
            case 5: nc->CoordSys.SaveWorkOffset(); break;
            }
            pShm->Coord_Command.reqSave = false;
        }
    }

    // =========================================================================
    // 🚶 3. ProcessTask_500ms - 中慢速任務 (半秒 1次)
    // =========================================================================
    void ProcessTask_500ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

       

        // --- Macro 變數全廣播 (約 18KB) ---
        pShm->macroStatus.currentCallDepth = nc->MacroSys.GetCurrentDepth();
        
        if (pShm->varCmd.refresh_global)
        {
            memcpy(pShm->macroStatus.globalVars, nc->MacroSys.GetGlobalVarsArray(), sizeof(pShm->macroStatus.globalVars));
            pShm->varCmd.refresh_global = false;
        }
    
        if (pShm->varCmd.refresh_system)
        {
            memcpy(pShm->macroStatus.sysVars, nc->MacroSys.GetSysVarsArray(), sizeof(pShm->macroStatus.sysVars));
            pShm->varCmd.refresh_system = false;
        }
       
        if (pShm->varCmd.refresh_local)
        {
            for (int i = 0; i < 8; i++)
            {
                memcpy(pShm->macroStatus.localVars[i], nc->MacroSys.GetLocalVarsArray(i), sizeof(double) * 101);
            }
            pShm->varCmd.refresh_local= false;
        }
      

        // --- 座標與補償表格拷貝 (約 16KB) ---
        memcpy(pShm->Coord_Table.extOffset, nc->CoordSys.extOffset, sizeof(double) * 8);
        for (size_t row = 0; row < 60 && row < nc->CoordSys.m_WCSTable.size(); ++row) {
            memcpy(pShm->Coord_Table.wcsTable[row], nc->CoordSys.m_WCSTable[row].data(), sizeof(double) * 8);
        }
        for (size_t row = 0; row < 100 && row < nc->CoordSys.m_ToolOffset.size(); ++row) {
            memcpy(pShm->Coord_Table.toolOffset[row], nc->CoordSys.m_ToolOffset[row].data(), sizeof(double) * 8);
        }
        for (size_t row = 0; row < 100 && row < nc->CoordSys.m_WorkOffset.size(); ++row) {
            memcpy(pShm->Coord_Table.workOffset[row], nc->CoordSys.m_WorkOffset[row].data(), sizeof(double) * 8);
        }



    }

    // =========================================================================
    // 🐌 3. ProcessTask_1000ms - 慢速背景任務 (1秒 1次)
    // =========================================================================
    void ProcessTask_1000ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // 這裡目前可以留空，未來可以用來處理：
        // 1. 系統硬碟容量檢查 (HMI 顯示磁碟剩餘空間)
        // 2. 背景定期自動存檔 (例如每分鐘 Auto-save)
        // 3. 長時間的加工統計數據 (例如總通電時間、加工時數計算)
        // 4. 與外部 ERP / IoT 系統的低頻率通訊心跳包 (Heartbeat)
    }
}