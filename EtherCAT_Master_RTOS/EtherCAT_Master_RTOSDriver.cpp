#include "EtherCAT_Master_RTOS.h"
#include <stdlib.h>  // 加入這行即可解決 exit 找不到的問題
#define VENDOR_ID_SIZE 2

// 全域變數
//

extern PCHAR vMemAddr[NUM_REGISTERS];     // 儲存由 RtMapMemory 回傳的虛擬記憶體位址指標
extern PUCHAR _baseAddress;               // 記憶體映射空間的起始基底位址


// DeviceSearch: 在機器的 PCI 總線上搜尋特定卡的第一個實例。
// 成功找到指定卡時回傳總線編號（Bus Number），
// 若未找到則回傳 DEVICE_NOT_FOUND (-1)。
//
int
DeviceSearch(int vendorID,                    // 輸入：裝置製造商 ID
    int deviceID,                    // 輸入：裝置產品 ID
    PCI_SLOT_NUMBER* pSlotNumber,   // 輸出：指向插槽編號的指標
    PPCI_COMMON_CONFIG PciData)      // 輸出：PCI 裝置詳細資訊
{
    ULONG bus = 0;             // 總線編號
    ULONG deviceNumber = 0;     // PCI 適配器的邏輯插槽編號
    ULONG functionNumber = 0;   // 指定適配器上的功能編號 (Function Number)
    ULONG bytesWritten = 0;     // RtGetBusDataByOffset 的回傳值
    BOOLEAN bFlag = TRUE;
    int CardIndex = 0;

    pSlotNumber->u.bits.Reserved = 0;

    // 當掃描完所有 PCI 總線後結束迴圈 (bFlag = FALSE)
    //
    for (bus = 0; bFlag; bus++)
    {
        for (deviceNumber = 0; deviceNumber < PCI_MAX_DEVICES && bFlag; deviceNumber++)
        {
            pSlotNumber->u.bits.DeviceNumber = deviceNumber;

            for (functionNumber = 0; functionNumber < PCI_MAX_FUNCTION; functionNumber++)
            {
                pSlotNumber->u.bits.FunctionNumber = functionNumber;

                // 取得 PCI 配置資訊
                bytesWritten = RtGetBusDataByOffset(PCIConfiguration,       // 欲取得的總線資料類型
                    bus,                    // 起始為零的總線編號
                    pSlotNumber->u.AsULONG, // 邏輯插槽編號
                    PciData,                // 存放配置資訊的緩衝區指標
                    0,                      // 緩衝區內的偏移量
                    PCI_COMMON_HDR_LENGTH); // 緩衝區長度

                if (bytesWritten == 0)
                {
                    // 已找完所有 PCI 總線：完成。
                    bFlag = FALSE;
                    break;
                }

                if (bytesWritten == VENDOR_ID_SIZE && PciData->VendorID == PCI_INVALID_VENDORID)
                {
                    // 多埠網卡 (NIC) 可能會跳過某些功能編號，繼續搜尋下一個功能。
                    continue;
                }

                //
                // 如果找到匹配的裝置則回傳，否則繼續搜尋直到找遍所有總線。
                // 您也可以使用 Base class 或 sub-class 來尋找特定裝置。
                //
                if ((PciData->VendorID == vendorID) && (PciData->DeviceID == deviceID))
                {
                    return bus;
                }
            } // 功能編號迴圈
        } // 裝置編號迴圈
    } // 總線迴圈

    return DEVICE_NOT_FOUND;
}

//
// DeviceInit: 初始化在機器 PCI 總線上找到的裝置。
// 成功回傳 TRUE，失敗則回傳 FALSE。
//
BOOLEAN
DeviceInit(int busNumber,                  // 輸入：中斷總線編號
    PCI_SLOT_NUMBER* pSlotNumber,  // 輸入：指向插槽編號的指標
    PPCI_COMMON_CONFIG PciData)     // 輸入：PCI 裝置詳細資訊
{

    LARGE_INTEGER memAddr;      // 基本埠號位址
    LARGE_INTEGER tranMemAddr;  // 由 RtMapMemory 回傳的轉換後基底位址
    ULONG AddressSpace = 0;     // 指示記憶體空間 (0 代表 Memory, 1 代表 I/O)
    ULONG bytesWritten = 0;
    int i = 0;
    int addressRange = 4 * 1024;  // 預設映射大小為 4KB

    //
    // 獲取每個位址暫存器 (Address Register) 的虛擬總線位址
    // 如果呼叫失敗，則設定為 NULL 並繼續
    //
    for (i = 0; i < NUM_REGISTERS; i++)
    {
        memAddr.QuadPart = PciData->u.type0.BaseAddresses[i];

        //
        // 將總線相關位址轉換為系統全局位址。
        //
        if (!RtTranslateBusAddress(PCIBus,               // 總線介面類型
            busNumber,            // 總線編號 (起始為零)
            memAddr,              // 總線相關位址
            &AddressSpace,        // 埠號或記憶體位址
            &tranMemAddr))        // 指向轉換後位址的指標
        {
            vMemAddr[i] = NULL;
            continue;
        }

        //
        // 將位址映射到軟體可以使用的虛擬位址
        //
        vMemAddr[i] = (PCHAR)RtMapMemory(tranMemAddr,    // 欲映射的實體位址範圍起始點
            addressRange,   // 以位元組為單位的位址範圍長度
            MmNonCached);   // 是否使用快取 (通常硬體暫存器不使用快取)
    }

    //
    // 設定命令參數，以便我們存取 PCI 裝置的控制暫存器。
    //
    PciData->Command = (PCI_ENABLE_IO_SPACE | PCI_ENABLE_MEMORY_SPACE |
        PCI_ENABLE_BUS_MASTER | PCI_ENABLE_WRITE_AND_INVALIDATE);


    bytesWritten = RtSetBusDataByOffset(PCIConfiguration,           // 欲設定的總線資料類型
        busNumber,                  // 總線編號
        pSlotNumber->u.AsULONG,     // 邏輯插槽編號
        PciData,                    // 包含配置資訊的緩衝區指標
        0,                          // 緩衝區內的偏移量
        PCI_COMMON_HDR_LENGTH);     // 緩衝區內的位元組數量

    if (bytesWritten == 0)
        return FALSE;

    Sleep(500); // 等待硬體穩定



    //
    // 待辦：
    // 執行此裝置所需的任何其他初始化操作。
    //

    return TRUE;
}

//
// DisableInterruptsOnChip: 停用晶片上的中斷功能，
// 以便您可以掛載到中斷向量。
//
void
DisableInterruptsOnChip()
{
    //
    // 待辦：
    // 在晶片上停用中斷
    //
}

//
// EnableInterruptsOnChip: 一旦成功掛載中斷向量，
// 即可啟用晶片上的中斷。
//
void
EnableInterruptsOnChip()
{
    //
    // 待辦：
    // 在晶片上啟用中斷
    //
}

//
// DeviceCleanup: 清理中斷向量與其他物件。
//
void
DeviceCleanup(HANDLE hHandle)  // 輸入：來自 RtAttachInterrupt 呼叫的中斷控制代碼
{
    BOOLEAN bResult = FALSE;
    int i = 0;

    if (hHandle != NULL)
    {
        // 釋放中斷資源
        bResult = RtReleaseInterrupt(hHandle);

        //
        // 等待 30 秒以驗證中斷資源已成功釋放。
        //
        Sleep(30000);

        //
        // 此處不需要再啟用中斷
        //

        if (!bResult)
        {
            MsgAndExit(_T("無法釋放此裝置的中斷資源。"));
        }
    }

    // 取消記憶體映射
    for (i = 0; i < NUM_REGISTERS; i++)
    {
        if (vMemAddr[i] != NULL)
        {
            RtUnmapMemory(vMemAddr[i]);
            vMemAddr[i] = NULL;
        }
    }

}


//
// MsgAndExit: 輸出錯誤訊息並以結束代碼 1 終止程式。
//
void
MsgAndExit(TCHAR* msg)  // 輸入：欲顯示的訊息。
{
#ifdef _UNICODE
    RtWprintf(L"錯誤: %s (錯誤碼 0x%X)\n", msg, RtGetLastError());
#else
    RtPrintf("錯誤: %s (錯誤碼 0x%X)\n", msg, RtGetLastError());
#endif

    // 注意：如果此處改為 ExitProcess()，則主模組中的 C++ 靜態解構子
    // 以及所有經由 atexit 註冊的回呼函式將不會被呼叫。
    exit(1);
}