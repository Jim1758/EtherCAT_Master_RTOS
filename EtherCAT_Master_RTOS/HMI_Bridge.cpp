#include "HMI_Bridge.h"
#include "SHMManager.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT
#include "NCManager.h"
#include "AlarmManager.h"
namespace HMI_Bridge 
{

    void ProcessTask(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr)
        {
            return; // 記憶體還沒準備好就退出
        }
        // API狀態區塊-----------------------------------------------------------
        pShm->API_Status.SHM_API_RunCount = nc->API_RunCount++;//API執行迴圈數

    
        // NC狀態區塊-----------------------------------------------------------
        pShm->NC_Status.SHM_NC_RunCount = nc->NC_RunCount;//NC執行迴圈數
        pShm->NC_Status.SHM_NC_State = static_cast<int32_t>(nc->m_state);//NC狀態
        pShm->NC_Status.SHM_EDM_State = static_cast<int32_t>(nc->m_edmState);//EDM設備狀態
        pShm->NC_Status.SHM_CurrentLine = nc->m_programPC;//目前NC執行到第幾行


       //警報區塊-----------------------------------------------------------
       // 抓取目前的更新次數
        uint32_t currentUpdateCount = AlarmManager::GetInstance().GetUpdateCount();

        // 只有當 Manager 裡的次數 跟 SHM 裡的次數「不一樣」時，我們才需要執行陣列複製 (優化效能)
        if (pShm->Alarm_Status.alarmUpdateCount != currentUpdateCount)
        {
            pShm->Alarm_Status.activeAlarmCount = AlarmManager::GetInstance().GetAlarmCount();

            //在填入新資料前，先把共享記憶體的警報陣列全部清零 (64個位子全填0)
            std::memset(pShm->Alarm_Status.activeAlarms, 0, sizeof(pShm->Alarm_Status.activeAlarms));
            for (int i = 0; i < pShm->Alarm_Status.activeAlarmCount; i++) 
            {
                pShm->Alarm_Status.activeAlarms[i] = AlarmManager::GetInstance().GetAlarmId(i);
            }

            // 最後才更新計數器，確保 C# 讀到新計數器時，陣列已經拷貝完畢
            pShm->Alarm_Status.alarmUpdateCount = currentUpdateCount;
        }
      
        // 命令區塊-----------------------------------------------------------
        if (pShm->NC_Command.cycleStart)//執行NC
        {    
            if (nc != nullptr)
            {
                nc->CycleStart();
            }
            pShm->NC_Command.cycleStart = false;//收到命令後清除
        }
        if (pShm->NC_Command.feedHold)//暫停NC
        {
            if (nc != nullptr)
            {
                nc->FeedHold();
            }
            pShm->NC_Command.feedHold = false;//收到命令後清除
        }
        if (pShm->NC_Command.reset)//重置NC
        {
            if (nc != nullptr)
            {
                nc->Reset();
            }
            pShm->NC_Command.reset = false;//收到命令後清除
        }
        if (pShm->NC_Command.loadProgramReq)//載入NC檔案命令
        {
            if (nc != nullptr)
            {
                size_t safeLength = strnlen(pShm->NC_Command.loadprogramName, 256);
                std::string NC_loadprogramName(pShm->NC_Command.loadprogramName, safeLength);
                nc->LoadProgram(GlobalConfig::GetInstance().NCProgramDir + NC_loadprogramName);
             
                if (nc->LoadProgram(GlobalConfig::GetInstance().NCProgramDir + NC_loadprogramName+".nc") == true)
                {
                    //載入成功
                }
                else
                {
                    //載入失敗
                }


            }
            pShm->NC_Command.loadProgramReq = false;//收到命令後清除
            std::memset(pShm->NC_Command.loadprogramName, 0, sizeof(pShm->NC_Command.loadprogramName));//清空
            
        }
    }

}