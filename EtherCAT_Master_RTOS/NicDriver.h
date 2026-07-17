#pragma once
#include <windows.h>
#include <rtapi.h>
#include <rtnapi.h>
#include <RtNalApi.h>
#include <tchar.h>

#define MAX_ETHER_FRAME_SIZE 1514

class CNicDriver
{
public:
    CNicDriver();
    ~CNicDriver();

    bool Open();
    void Close();

    bool SendPacket(unsigned char* pData, unsigned int length);
    unsigned int ReceivePacket(unsigned char* pBuffer);

    // ★ 新增：取得網卡真實 MAC 的函式
    void GetMacAddress(unsigned char* pMac) {
        memcpy(pMac, m_MacAddress, 6);
    }

    // Callbacks
    static BOOL RxCallback(PRTNAL_FRAME pFrame);
    static PVOID StubGetPacket(PVOID pContext, LONG length);
    static VOID  StubDecodePacket(PVOID pAppPacket, PVOID* ppContext, PULONG* ppData, PULONG pLength);

private:
    bool AcquireTxQueueInternal();
    bool AcquireRxQueueInternal();

private:
    RTNAL_QUEUE_HANDLE m_hTxQueue;
    RTNAL_QUEUE_HANDLE m_hRxQueue;
    PRTNAL_FRAME       m_pTxFrame;
    PRTNAL_FRAME       m_pTxFrameArray[1];

    // ★ 新增：儲存 MAC 位址
    unsigned char      m_MacAddress[6];

public:
    static unsigned char s_RxBuffer[MAX_ETHER_FRAME_SIZE];
    static volatile int  s_RxLen;
};