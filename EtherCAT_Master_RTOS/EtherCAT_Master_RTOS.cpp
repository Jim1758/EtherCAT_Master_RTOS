#include "EtherCAT_Master_RTOS.h"
#include <windows.h> 
#include <tchar.h>
#include <rtapi.h>
#include <string> // 確保引入 string
#include <fstream>   // 🌟 處理檔案讀取
#include <algorithm> // 🌟 處理字串轉換 (轉大寫防呆)
#include "NicDriver.h"
#include "EtherCatEni.h"
#include "EtherCatMaster.h"
#include "ConfigReader.h"
#include "GlobalConfig.h"

// RTX64 Boilerplate (標準 RTX64 樣板)
#ifndef NUM_REGISTERS
#define NUM_REGISTERS 10 
#endif
PCHAR vMemAddr[NUM_REGISTERS];
BOOLEAN RTAPI DeviceIST(PVOID pContext) { return TRUE; }

//const std::string BASE_DATA_DIR = "D:\\EtherCAT_Master_Data\\";//系統主要資料夾位置
CNicDriver MyNic;
EtherCatEni MyEni;
EtherCatMaster Master;

bool InitEtherCATMaster();//初始化 EtherCAT 主站


int _tmain(int argc, _TCHAR* argv[])//
{
  
    // ==============================================================
    // 🌟 1. 設定 CPU 核心親和性 (自動偵測安全版)
    // ==============================================================
    DWORD_PTR currentProcessMask = 0;
    DWORD_PTR availableSystemMask = 0; // 這個變數會裝載這台機台所有 RTSS 核心的遮罩
    HANDLE hCurrentProcess = GetCurrentProcess();

    // 步驟 A: 取得當前 RTSS 系統開放給您的可用核心遮罩
    if (!RtGetProcessAffinityMask(hCurrentProcess, &currentProcessMask, &availableSystemMask))
    {
        DEBUG_PRINT("[Error] RtGetProcessAffinityMask Failed! Error: %lu\n", GetLastError());
        return -1;
    }

    // 假設機台 RTSS 分配在 Core 2 與 Core 3，availableSystemMask 就會是 0x0C (二進位 1100)
    DEBUG_PRINT("[System] RTSS Available Core Mask is: 0x%llX\n", availableSystemMask);

    // 步驟 B: 從可用的核心中，抓出最低位的第一顆核心
    // (這是一招位元運算魔法: X & ~(X-1) 能精準提取最低位的 1)
    // 如果 availableSystemMask 是 0x0C (1100)，算出來的 targetMask 就會是 0x04 (0100)，也就是 Core 2
    DWORD_PTR targetMask = availableSystemMask & ~(availableSystemMask - 1);

    // 步驟 C: 將程式綁定到這顆核心
    if (!RtSetProcessAffinityMask(hCurrentProcess, targetMask))
    {
        DEBUG_PRINT("[Error] Failed to set CPU Affinity to Mask 0x%llX. Error: %lu\n", targetMask, GetLastError());
        return -1;
    }

    DEBUG_PRINT("[System] Successfully bound process to RTSS Core Mask: 0x%llX\n", targetMask);
    // ==============================================================


    std::string configPath = GlobalConfig::GetInstance().BaseDataDir + "SystemConfig.txt";
    GlobalConfig::GetInstance().LoadFromFile(configPath);//讀取系統檔案
  
    if (GlobalConfig::GetInstance().systemMode== SystemMode::UNKNOWN_MODE)
    {
        DEBUG_PRINT("Error System_Mode !\n");//錯誤系統模式
        return -1;
    }



    switch (GlobalConfig::GetInstance().systemMode)
    {
    case SystemMode::EDM_SINKER_MODE://EDM 雕磨模式
      
        
        if (InitEtherCATMaster() == false)//初始化 主站
        {
            DEBUG_PRINT("InitEtherCATMaster Error !\n");
            goto Exit;
        }
      
        //Master.RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式
        DEBUG_PRINT("EDM_SINKER_MODE Start !\n");
        if (Master.RunRealTimeCycle_EDM_SINKER_MODE() == -1)
        {
            DEBUG_PRINT("RunRealTimeCycle_EDM Error !\n");
            goto Exit;
        }
      
        //Master.RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式

        break;
    case SystemMode::EXAMPLE_MODE://範例模式 debug使用
        DEBUG_PRINT("EXAMPLE_MODE Start !\n");

        if (InitEtherCATMaster() == false)//初始化 主站
        {
            DEBUG_PRINT("InitEtherCATMaster Error !\n");
            goto Exit;
        }


        Master.RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式
        break;
    case SystemMode::UNKNOWN_MODE://未知模式
        DEBUG_PRINT("Error System_Mode>>UNKNOWN_MODE !\n");//錯誤系統模式
        goto Exit;

    }
   

 
Exit:
    DEBUG_PRINT("EtherCAT_Master Close !\n");
    MyNic.Close();   
    return 0;
}

bool InitEtherCATMaster()//初始化 EtherCAT 主站
{
    DEBUG_PRINT("InitEtherCATMaster !\n");


    if (MyNic.Open()==false) //初始化開啟網卡
    {
        DEBUG_PRINT("Failed to open NIC.\n");
        return  false;
    }


    std::string eniPath = GlobalConfig::GetInstance().BaseDataDir + "ENI\\ENI.xml";
  
    if (MyEni.LoadXml(eniPath.c_str())==false)//載入ENI
    {
        DEBUG_PRINT("Error>>LoadXml\n");
        return  false;
    }

  

    Master.AttachNic(&MyNic);// 綁定網卡驅動程式
    Master.AttachEni(&MyEni);// 綁定 ENI 設定檔解析器

    int SlavesCount = Master.ScanSlaves(); //ScanSlaves(掃描從站)
    if (SlavesCount == 0)
    {
        DEBUG_PRINT("Error>>ScanSlaves>>SlavesCount>>%d\n", SlavesCount);
        return  false;
    }
    else
    {
        DEBUG_PRINT("SlavesCount>>%d\n", SlavesCount);
    }


    if (Master.BuildIoMap() != 0)//自動建構 IO 地圖
    {
        DEBUG_PRINT("BuildIoMap Error !\n");
       
        return  false;
    }


   
    if (Master.Initialize_Slaves() != 0)//初始化所有從站 INIT>>PRE-OP>>SAFE-OP>>OP
    {
        DEBUG_PRINT("BInitialize_Slaves Error !\n");
    
        return  false;
    }

    return true;
}


