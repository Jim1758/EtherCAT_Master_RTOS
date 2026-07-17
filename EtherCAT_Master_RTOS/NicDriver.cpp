#include "NicDriver.h"
#include <stdio.h>
#include "GlobalConfig.h"
unsigned char CNicDriver::s_RxBuffer[MAX_ETHER_FRAME_SIZE];
volatile int  CNicDriver::s_RxLen = 0;

// 防呆 Buffer (防止驅動因為指標為 NULL 而報錯)
static unsigned char g_DummyBuffer[MAX_ETHER_FRAME_SIZE];

CNicDriver::CNicDriver() {
    m_hTxQueue = NULL; m_hRxQueue = NULL; m_pTxFrame = NULL;
    memset(m_pTxFrameArray, 0, sizeof(m_pTxFrameArray));
    memset(m_MacAddress, 0, 6);
}

CNicDriver::~CNicDriver() { Close(); }

// ---------------------------------------------------------
// 標準 Callback (資料接收核心)
// ---------------------------------------------------------
BOOL CNicDriver::RxCallback(PRTNAL_FRAME pFrame) {
    if (pFrame && pFrame->frameSize > 0) {
        int len = pFrame->frameSize;
        if (len > MAX_ETHER_FRAME_SIZE) len = MAX_ETHER_FRAME_SIZE;

        // 將資料複製到靜態緩衝區
        memcpy(s_RxBuffer, pFrame->frameBufferVirtualAddr, len);
        s_RxLen = len;
    }

    // 釋放 Frame，讓網卡可以重複利用這塊記憶體
    if (pFrame) RtNalFreeFrame(pFrame);
    return TRUE;
}

PVOID CNicDriver::StubGetPacket(PVOID pContext, LONG length) { return g_DummyBuffer; }
VOID CNicDriver::StubDecodePacket(PVOID pAppPacket, PVOID* ppContext, PULONG* ppData, PULONG pLength) {}

// ---------------------------------------------------------
// Open: 初始化並開啟網卡
// ---------------------------------------------------------
bool CNicDriver::Open()
{
    // 1. 初始化 NAL
    if (!RtNalInit(RTNAL_API_VERSION)) 
    {
      
        DEBUG_PRINT("CNicDriver Error: RtNalInit failed (0x%X)\n", GetLastError());//
        return false;
    }

    DEBUG_PRINT("Initializing Network Interface !\n");//
 


    // 2. 搜尋並獲取 TX/RX Queue
    if (!AcquireTxQueueInternal()) return false;
    if (!AcquireRxQueueInternal()) return false;

    RTNAL_QUEUE_CAPABILITIES caps;

    // 3. 配置 TX Queue
    memset(&caps, 0, sizeof(caps));
    caps.fpRtnDecodePacket = StubDecodePacket;
    caps.fpRtnGetPacket = StubGetPacket;

    if (!RtNalConfigureQueue(m_hTxQueue, (PVOID)this, &caps)) 
    {
        DEBUG_PRINT("CNicDriver Error: Configure TX failed (0x%X)\n", GetLastError());
        Close(); return false;
    }
    DEBUG_PRINT("CNicDriver: TX Configuration [OK]\n");

    // 4. 配置 RX Queue
    if (m_hRxQueue) {
        memset(&caps, 0, sizeof(caps));
        caps.fpRtnReceiveCallback = RxCallback; // 註冊 Callback
        caps.fpRtnGetPacket = StubGetPacket;
        caps.fpRtnDecodePacket = StubDecodePacket;

        if (!RtNalConfigureQueue(m_hRxQueue, (PVOID)this, &caps))
        {
            DEBUG_PRINT("CNicDriver Error: Configure RX failed (0x%X)\n", GetLastError());
            Close(); return false;
        }
        DEBUG_PRINT("CNicDriver: RX Configuration [OK]\n");

        // 5. 設定 EtherCAT 過濾器 (0x88A4)
        // 先清除所有舊規則
        for (int i = 0; i < 32; i++) RtNalClearReceiveFilterEntryEthertype(m_hRxQueue, i);

        // 加入 0x88A4 白名單
        if (!RtNalSetReceiveFilterEntryEthertype(m_hRxQueue, 0, 0x88A4, FALSE, TRUE)) {
            DEBUG_PRINT("CNicDriver Warning: Failed to set EtherCAT filter (Optional)\n");
        }
        else {
            DEBUG_PRINT("CNicDriver: EtherCAT Filter (0x88A4) Applied.\n");
        }
    }

    // 6. 分配 TX Frame 記憶體
    m_pTxFrame = RtNalAllocateFrame(MAX_ETHER_FRAME_SIZE);
    if (!m_pTxFrame) {
        DEBUG_PRINT("CNicDriver Error: Failed to allocate TX frame.\n");
        Close(); return false;
    }
    m_pTxFrameArray[0] = m_pTxFrame;

    DEBUG_PRINT("--- Network Interface Ready ---\n");
    return true;
}

void CNicDriver::Close()
{
    if (m_pTxFrame) { RtNalFreeFrame(m_pTxFrame); m_pTxFrame = NULL; }
    if (m_hTxQueue) { RtNalReleaseQueue(m_hTxQueue); m_hTxQueue = NULL; }
    if (m_hRxQueue) { RtNalReleaseQueue(m_hRxQueue); m_hRxQueue = NULL; }
}

bool CNicDriver::SendPacket(unsigned char* pData, unsigned int length) {
    if (!m_hTxQueue || !m_pTxFrame) return false;

    // 將資料複製到 DMA 緩衝區
    memcpy(m_pTxFrame->frameBufferVirtualAddr, pData, length);
    m_pTxFrame->frameSize = length;

    ULONG submitted = 0;
    return RtNalTransmitEx(m_hTxQueue, m_pTxFrameArray, 1, &submitted);
}

unsigned int CNicDriver::ReceivePacket(unsigned char* pBuffer) {
    if (!m_hRxQueue) return 0;
    s_RxLen = 0;

    // 主動觸發接收檢查 (標準模式)
    RtNalReceive(m_hRxQueue);

    // 如果 Callback 有被呼叫，s_RxLen 會大於 0
    if (s_RxLen > 0 && pBuffer != NULL) {
        memcpy(pBuffer, s_RxBuffer, s_RxLen);
        return s_RxLen;
    }
    return 0;
}

// ---------------------------------------------------------
// 搜尋並獲取 TX Queue (通用版)
// ---------------------------------------------------------
bool CNicDriver::AcquireTxQueueInternal()
{
    INT num = RtNalGetNumberOfQueues();
    RTNAL_QUEUE info; RTNAL_QUEUE_CRITERIA cri; RTNAL_QUEUE_EVENTS evt;

    // 遍歷所有 Queue，找到第一個可用的 TX Queue
    for (int i = 0; i < num; i++) {
        if (RtNalGetQueueInfoByIndex(i, &info)) {
            if (info.queueInfo.queueType == RTNAL_QUEUE_TYPE_TX) {

                memset(&cri, 0, sizeof(cri)); memset(&evt, 0, sizeof(evt));
                cri.method = RTNAL_DEVICE_NAME_QUEUE_EXACT;
                cri.deviceQueueNumber = info.queueInfo.deviceQueueNumber;
                cri.queueType = RTNAL_QUEUE_TYPE_TX;
                memcpy(cri.deviceName, info.deviceInfo.deviceName, RTNAL_DEVICE_NAME_LENGTH);

                // 標準 TX Flag
                cri.flags = RTNAL_USE_TX_COMPLETE_EVENT_FLAG;

                m_hTxQueue = RtNalAcquireQueue(&cri, &info, &evt);
                if (m_hTxQueue) {
                    DEBUG_PRINT("CNicDriver: [TX] Queue Acquired\n");
                    DEBUG_PRINT("    > Name: %s\n", info.deviceInfo.deviceName);

                    // 儲存並印出 MAC 位址
                    memcpy(m_MacAddress, info.deviceInfo.macAddress, 6);
                    DEBUG_PRINT("    > MAC : %02X-%02X-%02X-%02X-%02X-%02X\n",
                        m_MacAddress[0], m_MacAddress[1], m_MacAddress[2],
                        m_MacAddress[3], m_MacAddress[4], m_MacAddress[5]);

                    return true;
                }
            }
        }
    }

    DEBUG_PRINT("CNicDriver Error: No available RTX64 TX Queue found!\n");
    return false;
}

// ---------------------------------------------------------
// 搜尋並獲取 RX Queue (通用版)
// ---------------------------------------------------------
bool CNicDriver::AcquireRxQueueInternal()
{
    INT num = RtNalGetNumberOfQueues();
    RTNAL_QUEUE info; RTNAL_QUEUE_CRITERIA cri; RTNAL_QUEUE_EVENTS evt;

    // 遍歷所有 Queue，找到第一個可用的 RX Queue
    for (int i = 0; i < num; i++) {
        if (RtNalGetQueueInfoByIndex(i, &info)) {
            if (info.queueInfo.queueType == RTNAL_QUEUE_TYPE_RX) {

                memset(&cri, 0, sizeof(cri)); memset(&evt, 0, sizeof(evt));
                cri.method = RTNAL_DEVICE_NAME_QUEUE_EXACT;
                cri.deviceQueueNumber = info.queueInfo.deviceQueueNumber;
                cri.queueType = RTNAL_QUEUE_TYPE_RX;
                memcpy(cri.deviceName, info.deviceInfo.deviceName, RTNAL_DEVICE_NAME_LENGTH);

                // ★★★ 標準 RX Flag (啟用接收事件) ★★★
                cri.flags = RTNAL_USE_RX_EVENT_FLAG;

                m_hRxQueue = RtNalAcquireQueue(&cri, &info, &evt);
                if (m_hRxQueue) {
                    RtPrintf("CNicDriver: [RX] Queue Acquired\n");
                    RtPrintf("    > Name: %s\n", info.deviceInfo.deviceName);
                    return true;
                }
            }
        }
    }

    RtPrintf("CNicDriver Error: No available RTX64 RX Queue found!\n");
    return false;
}