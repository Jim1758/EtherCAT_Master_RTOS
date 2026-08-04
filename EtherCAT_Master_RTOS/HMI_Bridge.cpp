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


     

        // --- 即時座標廣播 ---
        double currentWCS[8] = { 0.0 };
        nc->CoordSys.GetActualWCS(currentWCS);

        for (int i = 0; i < 8; i++)
        {
            // 取得軸的 Context，方便後續讀取參數
            auto& axis = nc->m_motion.GetAxisContext(i);

            // 🌟 1. 取得這根軸當下的補償總量
            double currentComp = nc->m_motion.GetAxisContext(i).currentCompOffset_unit;

            // 🌟 2. 扣除補償量，還原成操作員眼中的「真實邏輯座標」
            double displayMCS = nc->CoordSys.actualMCS[i] - currentComp;
            double displayWCS = currentWCS[i] - currentComp;

          
          
            // ========================================================
            // 🌟 3. 旋轉軸的 0~360 度顯示處理
            // ========================================================
            AxisType type = nc->m_motion.GetAxisContext(i).axisType;

            // 如果是旋轉軸 (包含連續旋轉軸)
            if (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS)
            {
                // 對 360 取餘數 (折疊座標)
                displayMCS = std::fmod(displayMCS, 360.0);
                displayWCS = std::fmod(displayWCS, 360.0);

                // 防呆：如果是負角度 (例如 -10 度)，轉回正的 350 度
                if (displayMCS < 0.0) displayMCS += 360.0;
                if (displayWCS < 0.0) displayWCS += 360.0;
            }

            // 寫入 Shared Memory 廣播給人機
            pShm->NC_Status.actualMCS[i] = displayMCS;
            pShm->NC_Status.actualWCS[i] = displayWCS;
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
       // 1. 取得最新狀態
        uint32_t currentUpdateCount = AlarmManager::GetInstance().GetUpdateCount();

        // 2. 如果計數器有變，代表有新警報或警報剛被清除
        if (pShm->Alarm_Status.alarmUpdateCount != currentUpdateCount)
        {
            auto& am = AlarmManager::GetInstance();

            // 更新數量
            pShm->Alarm_Status.activeAlarmCount = am.GetAlarmCount();

            // 清空舊資料
            std::memset(pShm->Alarm_Status.activeAlarms, 0, sizeof(pShm->Alarm_Status.activeAlarms));

            // 🌟 將軸索引預設填滿 -1
            std::fill(std::begin(pShm->Alarm_Status.activeAlarmAxes),
                std::end(pShm->Alarm_Status.activeAlarmAxes), -1);

            // 🌟 把錯誤碼和軸號，一對一打包送進 SHM
            for (int i = 0; i < pShm->Alarm_Status.activeAlarmCount; i++) {
                pShm->Alarm_Status.activeAlarms[i] = am.GetAlarmId(i);
                pShm->Alarm_Status.activeAlarmAxes[i] = am.GetAlarmAxisIndex(i); // 抓取軸號
            }

            // 更新完成
            pShm->Alarm_Status.alarmUpdateCount = currentUpdateCount;
        }



        // --- 字串複製 (程式名稱) ---
        std::strncpy(pShm->NC_Status.mainProgName, nc->m_mainProgramName.c_str(), 63);
        pShm->NC_Status.mainProgName[63] = '\0';
       
        pShm->NC_Status.mainCurrentLine = nc->GetActivePC();
        std::strncpy(pShm->NC_Status.macroProgName, nc->m_macroProgramName.c_str(), 63);
        pShm->NC_Status.macroProgName[63] = '\0';


        // ==========================================================
            // 🌟 雙指標神同步 (Dual PC Synchronization) - 智慧混合版
            // ==========================================================

            // 1. 取得大腦的指標 (Interpreter PC)
        int interpreterMainPC = nc->GetBasePC();
        int interpreterMacroPC = nc->m_macroProgramPC;

        // 2. 取得實體馬達的指標 (Motion PC)
        int physicalPC = nc->GetMotion().GetPhysicalExecutionPC();

        // 3. 🌟 【神級判斷】：手腳追上大腦了嗎？
        // 如果 IsGroupDone() 為 true，代表底層倉庫全空，馬達完全靜止。
        // 這意味著目前的指令是「非運動指令」(如 M00, G04, 巨集變數)，機台正在執行大腦的狀態！
        bool isMachineIdle = nc->GetMotion().IsGroupDone();

        // 4. 判斷目前這張單子是屬於主程式還是副程式
        bool isMacroRunning = !nc->m_macroStack.empty();

        if (isMacroRunning) {
            // 在跑副程式
            // 🌟 如果機台靜止，游標顯示大腦卡住的地方(如 M00)；如果機台在動，顯示馬達正在跑的路徑
            pShm->NC_Status.macroCurrentLine = isMachineIdle ? interpreterMacroPC : physicalPC;
            pShm->NC_Status.mainCurrentLine = interpreterMainPC; // 主程式永遠顯示呼叫副程式的那一行
        }
        else {
            // 在跑主程式
            pShm->NC_Status.mainCurrentLine = isMachineIdle ? interpreterMainPC : physicalPC;
            pShm->NC_Status.macroCurrentLine = -1;
        }

        int interpreterWCS = nc->CoordSys.GetCurrentWCSGCode(); // 大腦的座標系
        int physicalWCS = nc->GetMotion().GetPhysicalExecutionWCS(); // 馬達的座標系
       
        pShm->NC_Status.currentWCS_GCode = isMachineIdle ? interpreterWCS : physicalWCS;//坐標系


        // 3. 🌟 刀具長度補正顯示 (Tool Length Comp)
        int interpreterToolMode = nc->CoordSys.toolLengthMode;
        int physicalToolMode = nc->GetMotion().GetPhysicalExecutionToolMode();

        int interpreterHCode = nc->CoordSys.currentHCode;
        int physicalHCode = nc->GetMotion().GetPhysicalExecutionHCode();

        // 如果機台靜止(大腦卡住)，顯示大腦狀態；如果機台在跑，顯示馬達標籤狀態！
        // 假設 pShm->NC_Status 有這兩個變數供 UI 綁定
        pShm->NC_Status.currentToolLengthMode = isMachineIdle ? interpreterToolMode : physicalToolMode;
        pShm->NC_Status.currentHCode = isMachineIdle ? interpreterHCode : physicalHCode;

        // 4. 🌟 刀徑補正顯示 (Tool Radius Comp)
        int interpreterTRadMode = nc->CoordSys.toolRadiusMode;
        int physicalTRadMode = nc->GetMotion().GetPhysicalExecutionToolRadiusMode();

        int interpreterDCode = nc->CoordSys.currentDCode;
        int physicalDCode = nc->GetMotion().GetPhysicalExecutionDCode();

        // 🌟 取得大腦與馬達的 isAbsoluteMode
        bool interpreterAbs = nc->CoordSys.isAbsoluteMode;
        bool physicalAbs = nc->GetMotion().GetPhysicalExecutionIsAbsoluteMode();

        // 根據機台是否靜止，決定 UI 要聽誰的，然後轉成 1 或 0 傳給 HMI
        bool finalAbs = isMachineIdle ? interpreterAbs : physicalAbs;
        pShm->NC_Status.isAbsoluteMode = finalAbs ? 1 : 0;


        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentToolRadiusMode = isMachineIdle ? interpreterTRadMode : physicalTRadMode;
        pShm->NC_Status.currentDCode = isMachineIdle ? interpreterDCode : physicalDCode;


        // 🌟 G68 旋轉狀態與角度顯示
        bool interpreterG68 = nc->CoordSys.isG68Active;
        bool physicalG68 = nc->GetMotion().GetPhysicalExecutionG68Active();

        double interpreterG68Angle = nc->CoordSys.g68Angle;
        double physicalG68Angle = nc->GetMotion().GetPhysicalExecutionG68Angle();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG68State = (isMachineIdle ? interpreterG68 : physicalG68) ? 1 : 0;
        pShm->NC_Status.currentG68Angle = isMachineIdle ? interpreterG68Angle : physicalG68Angle;



        // 🌟 G168 狀態與 W 碼顯示
        bool interpreterG168 = nc->CoordSys.isWorkpieceRotationActive;
        bool physicalG168 = nc->GetMotion().GetPhysicalExecutionG168Active();

        int interpreterWCode = nc->CoordSys.currentWCode;
        int physicalWCode = nc->GetMotion().GetPhysicalExecutionWCode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG168State = (isMachineIdle ? interpreterG168 : physicalG168) ? 1 : 0;
        pShm->NC_Status.currentWCode = isMachineIdle ? interpreterWCode : physicalWCode;


        // 🌟 7. G51 縮放狀態顯示
        bool interpreterG51 = nc->CoordSys.isScalingActive;
        bool physicalG51 = nc->GetMotion().GetPhysicalExecutionG51Active();

        double interpreterScale = nc->CoordSys.scaleFactor;
        double physicalScale = nc->GetMotion().GetPhysicalExecutionScaleRatio();

        // 🌟 8. G151 / G150 鏡像狀態顯示

        // 算出大腦目前的 Mask
        uint8_t interpreterMirrorMask = 0;
        for (int i = 0; i < 8; i++) {
            if (nc->CoordSys.isMirrorActive[i]) {
                interpreterMirrorMask |= (1 << i);
            }
        }

        // 拿取馬達目前的 Mask
        uint8_t physicalMirrorMask = nc->GetMotion().GetPhysicalExecutionMirrorMask();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentMirrorMask = isMachineIdle ? interpreterMirrorMask : physicalMirrorMask;

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG51State = (isMachineIdle ? interpreterG51 : physicalG51) ? 1 : 0;
        pShm->NC_Status.currentScaleRatio = isMachineIdle ? interpreterScale : physicalScale;

        // 🌟 9. G16 極座標狀態顯示
        bool interpreterG16 = nc->CoordSys.isPolarCoordinateActive;
        bool physicalG16 = nc->GetMotion().GetPhysicalExecutionG16Active();

        // 寫入 SHM 供 HMI 讀取燈號
        bool finalG16 = isMachineIdle ? interpreterG16 : physicalG16;
        pShm->NC_Status.currentG16State = finalG16 ? 1 : 0;


        // 🌟 G162 偏心補償與 G17/18/19 平面顯示
        bool interpreterG162 = nc->CoordSys.isCAxisOffsetRotationEnabled;
        bool physicalG162 = nc->GetMotion().GetPhysicalExecutionG162Active();

        int interpreterPlane = nc->CoordSys.activePlane;
        int physicalPlane = nc->GetMotion().GetPhysicalExecutionPlaneMode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG162State = (isMachineIdle ? interpreterG162 : physicalG162) ? 1 : 0;
        pShm->NC_Status.currentPlaneMode = isMachineIdle ? interpreterPlane : physicalPlane;



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


        
      
      
        //軸狀態-------------------------------------------------------
        // =======================================================
        // 🌟 [修改這裡] 軸狀態：直接請 MotionCore 把資料填入 pShm 
        // =======================================================
        // 取代你原本手寫的 for 迴圈與 m_pContexts
        nc->GetMotion().ExportDebugInfo(pShm->axisDebug,true);
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