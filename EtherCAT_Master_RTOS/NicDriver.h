#pragma once

/*
 * 檔案：NicDriver.h
 * 版本：EtherCAT DC Release Candidate RC1
 *
 * 功能：
 * 1. 封裝 RTX64 NAL 的 TX/RX Queue。
 * 2. 提供 EtherCAT Master 使用的 SendPacket()/ReceivePacket()。
 * 3. 管理一個由 RtNalAllocateFrame() 建立的 zero-copy TX Frame。
 * 4. 對外提供 TX ownership 與送出失敗的累積診斷計數。
 *
 * 執行緒規則：
 * - Open()/Close() 只能在初始化或停止流程呼叫。
 * - SendPacket()/ReceivePacket() 由 EtherCAT 通訊執行緒呼叫。
 * - 診斷執行緒只讀取下方 g_nicTx* 變數，不直接操作 Queue。
 *
 * 重要限制：
 * - NAL 擁有 TX Frame 時，不可修改 Frame 結構或資料緩衝區。
 * - SendPacket() 必須先經過 RtNalIsApplicationFrame() 確認 ownership。
 * - 本類別目前使用單一 TX Frame；TxFrameBusy 可用來判斷 Frame 是否
 *   尚未由 TX complete thread 歸還。
 */

#include <windows.h>
#include <rtapi.h>
#include <rtnapi.h>
#include <RtNalApi.h>
#include <tchar.h>
#include <string.h>

 // RTX64 NAL 設定的最大 Ethernet Frame 大小。
#define MAX_ETHER_FRAME_SIZE 1514

// RX 暫存空間額外保留 4 bytes，接收後仍會限制複製長度為 1514 bytes。
#define MAX_ETHER_RX_BUFFER_SIZE 1518

class CNicDriver
{
public:
    // 建立未開啟的 Driver 物件；不會在建構子內取得 Queue。
    CNicDriver();

    // 確保 TX Frame 與 TX/RX Queue 被釋放。
    ~CNicDriver();

    // 初始化 NAL、取得並設定 TX/RX Queue、設定 EtherCAT filter。
    bool Open();

    // 釋放 Frame 與 Queue；允許重複呼叫。
    void Close();

    // 將一個 Ethernet Frame 交給 RTX64 NAL 傳送。
    // 回傳 true 表示一個 Frame 已成功提交；false 表示未提交。
    bool SendPacket(unsigned char* pData, unsigned int length);

    // 嘗試取得一個 RX Frame。無資料或接收失敗時回傳 0。
    unsigned int ReceivePacket(unsigned char* pBuffer);

    // 複製目前 RTX64 NIC 的實體 MAC Address。
    void GetMacAddress(unsigned char* pMac)
    {
        memcpy(pMac, m_MacAddress, 6);
    }

private:
    // STANDARD_BUFFER_V2 的應用層 RX Packet。
    // NAL 透過 callbacks 取得 Data 位址並回填 Length。
    struct RxPacket
    {
        CNicDriver* Owner;
        ULONG Length;
        unsigned char Data[MAX_ETHER_RX_BUFFER_SIZE];
    };

    // RX Queue callbacks：提供接收緩衝區並解碼應用層 Packet。
    static PVOID RxGetPacket(PVOID pContext, LONG length);
    static VOID RxDecodePacket(
        PVOID pAppPacket,
        PVOID* ppContext,
        PULONG* ppData,
        PULONG pLength);

    // TX Queue 不使用傳統 callback copy path，但 ConfigureQueue 仍需要 callbacks。
    static PVOID StubGetPacket(PVOID pContext, LONG length);
    static VOID StubDecodePacket(
        PVOID pAppPacket,
        PVOID* ppContext,
        PULONG* ppData,
        PULONG pLength);

    // 尋找並獨占第一個可用的指定型別 Queue。
    bool AcquireTxQueueInternal();
    bool AcquireRxQueueInternal();

    // Queue handles 僅在 Open() 成功後有效。
    RTNAL_QUEUE_HANDLE m_hTxQueue;
    RTNAL_QUEUE_HANDLE m_hRxQueue;

    // zero-copy TX Frame；ownership 可能在 Application 與 NAL 之間切換。
    PRTNAL_FRAME m_pTxFrame;
    PRTNAL_FRAME m_pTxFrameArray[1];

    // Ethernet Header 使用的來源 MAC。
    unsigned char m_MacAddress[6];

    // STANDARD_BUFFER_V2 的固定 RX 緩衝區。
    RxPacket m_RxPacket;
};

// -----------------------------------------------------------------------------
// TX 累積診斷計數
//
// Calls        ：SendPacket() 呼叫總數。
// Success      ：成功提交一個 Frame 的總數。
// FrameBusy    ：送出前檢查發現 TX Frame 尚由 NAL 持有。
// NotOwner     ：RtNalTransmitEx() 回報 ERROR_NOT_OWNER。
// SubmitFail   ：輸入無效、NAL 呼叫失敗或提交數量不是 1。
// Submitted0   ：呼叫 RtNalTransmitEx() 後 submitted == 0。
// LastError    ：最後一次 TX 異常的 Win32/NAL error code；成功時不清除，
//                方便保留最後故障原因。
// -----------------------------------------------------------------------------
extern volatile LONGLONG g_nicTxCalls;
extern volatile LONGLONG g_nicTxSuccess;
extern volatile LONGLONG g_nicTxFrameBusy;
extern volatile LONGLONG g_nicTxNotOwner;
extern volatile LONGLONG g_nicTxSubmitFail;
extern volatile LONGLONG g_nicTxSubmitted0;
extern volatile LONG g_nicTxLastError;
