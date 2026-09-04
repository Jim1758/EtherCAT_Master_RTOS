#include "NicDriver.h"
#include "GlobalConfig.h"

#include <rtssapi.h>
#include <string.h>

/*
 * 檔案：NicDriver.cpp
 * 版本：EtherCAT DC-RX.4A RX Incident Forensics（保留 DC-RX.2 Event Wait）
 *
 * 此檔案只負責 RTX64 NAL Queue、Frame ownership 與封包搬移。
 * 不處理 EtherCAT Datagram、DC 控制、Motion 或 NC 邏輯。
 *
 * 正式候選測試設定：
 * - RTX64 NAL Interrupt thread priority：70
 * - RTX64 NAL Transmit complete thread priority：70
 * - RX Mode：STANDARD_BUFFER_V2
 * - EtherType filter：0x88A4
 */

#if defined(_MSC_VER)
#pragma message("Compiling NicDriver.cpp - ETHERCAT_DC_RX4A_FORENSICS")
#endif

 // TX Queue callback 的保底緩衝區；zero-copy RtNalTransmitEx() 不使用它傳資料。
static unsigned char g_DummyBuffer[MAX_ETHER_FRAME_SIZE];

// 所有計數都只增加或在 Open() 成功時歸零，供低優先權診斷執行緒讀取。
volatile LONGLONG g_nicTxCalls = 0;
volatile LONGLONG g_nicTxSuccess = 0;
volatile LONGLONG g_nicTxFrameBusy = 0;
volatile LONGLONG g_nicTxNotOwner = 0;
volatile LONGLONG g_nicTxSubmitFail = 0;
volatile LONGLONG g_nicTxSubmitted0 = 0;
volatile LONG g_nicTxLastError = ERROR_SUCCESS;

CNicDriver::CNicDriver()
    : m_hTxQueue(NULL),
    m_hRxQueue(NULL),
    m_pTxFrame(NULL)
{
    memset(m_pTxFrameArray, 0, sizeof(m_pTxFrameArray));
    memset(m_MacAddress, 0, sizeof(m_MacAddress));
    memset(&m_RxQueueEvents, 0, sizeof(m_RxQueueEvents));
    memset(&m_RxPacket, 0, sizeof(m_RxPacket));
    m_RxPacket.Owner = this;
    m_RxCallCounter = 0ULL;
    m_LastRxCallDiagnostic = NicRxCallDiagnostic{};
}

CNicDriver::~CNicDriver()
{
    Close();
}

PVOID CNicDriver::RxGetPacket(PVOID pContext, LONG length)
{
    // NAL 要求一個可寫入的應用層 Packet；長度異常時拒絕接收。
    CNicDriver* driver = static_cast<CNicDriver*>(pContext);
    if (driver == NULL ||
        length <= 0 ||
        length > MAX_ETHER_RX_BUFFER_SIZE)
    {
        return NULL;
    }

    driver->m_RxPacket.Length = static_cast<ULONG>(length);
    return &driver->m_RxPacket;
}

VOID CNicDriver::RxDecodePacket(
    PVOID pAppPacket,
    PVOID* ppContext,
    PULONG* ppData,
    PULONG pLength)
{
    // 將 RxPacket 轉換成 NAL 需要的 context、資料位址與有效長度。
    if (pAppPacket == NULL ||
        ppContext == NULL ||
        ppData == NULL ||
        pLength == NULL)
    {
        return;
    }

    RxPacket* packet = static_cast<RxPacket*>(pAppPacket);
    *ppContext = packet->Owner;
    *ppData = reinterpret_cast<PULONG>(packet->Data);
    *pLength = packet->Length;
}

PVOID CNicDriver::StubGetPacket(PVOID pContext, LONG length)
{
    // TX Queue 的 ConfigureQueue 相容 callback；實際 TX 使用 NAL Frame。
    UNREFERENCED_PARAMETER(pContext);
    UNREFERENCED_PARAMETER(length);
    return g_DummyBuffer;
}

VOID CNicDriver::StubDecodePacket(
    PVOID pAppPacket,
    PVOID* ppContext,
    PULONG* ppData,
    PULONG pLength)
{
    UNREFERENCED_PARAMETER(pAppPacket);
    UNREFERENCED_PARAMETER(ppContext);
    UNREFERENCED_PARAMETER(ppData);
    UNREFERENCED_PARAMETER(pLength);
}

bool CNicDriver::Open()
{
    // RtNalInit 必須在任何 Queue/Frame API 之前成功。
    if (!RtNalInit(RTNAL_API_VERSION))
    {
        DEBUG_PRINT("CNicDriver Error: RtNalInit failed (0x%X)\n", GetLastError());
        return false;
    }

    DEBUG_PRINT("Initializing Network Interface !\n");

    // 先取得 TX，再取得 RX；RX 失敗時統一由 Close() 回收 TX。
    if (!AcquireTxQueueInternal())
        return false;

    if (!AcquireRxQueueInternal())
    {
        Close();
        return false;
    }

    // TX Queue 採 RtNalTransmitEx() zero-copy，callbacks 只供 Queue 設定相容。
    RTNAL_QUEUE_CAPABILITIES caps;
    memset(&caps, 0, sizeof(caps));
    caps.fpRtnDecodePacket = StubDecodePacket;
    caps.fpRtnGetPacket = StubGetPacket;

    if (!RtNalConfigureQueue(m_hTxQueue, static_cast<PVOID>(this), &caps))
    {
        DEBUG_PRINT("CNicDriver Error: Configure TX failed (0x%X)\n", GetLastError());
        Close();
        return false;
    }

    DEBUG_PRINT("CNicDriver: TX Configuration [OK]\n");
    DEBUG_PRINT("CNicDriver: TX Diagnostic [RELEASE_CANDIDATE_RC1]\n");

    // RX Queue 採 STANDARD_BUFFER_V2：NAL 將資料寫入 m_RxPacket.Data。
    memset(&caps, 0, sizeof(caps));
    caps.fpRtnGetPacket = RxGetPacket;
    caps.fpRtnDecodePacket = RxDecodePacket;

    if (!RtNalConfigureQueue(m_hRxQueue, static_cast<PVOID>(this), &caps))
    {
        DEBUG_PRINT("CNicDriver Error: Configure RX failed (0x%X)\n", GetLastError());
        Close();
        return false;
    }

    DEBUG_PRINT("CNicDriver: RX Configuration [OK]\n");
    DEBUG_PRINT("CNicDriver: RX Mode [STANDARD_BUFFER_V2]\n");
    DEBUG_PRINT(
        "CNicDriver: RX Wait [%s]\n",
        IsReceiveNotificationAvailable()
        ? "EVENT_HYBRID_DC_RX2"
        : "SLEEP_FALLBACK");

    // 清除舊 filter，再只接受 EtherCAT EtherType 0x88A4。
    for (int i = 0; i < 32; ++i)
        RtNalClearReceiveFilterEntryEthertype(m_hRxQueue, i);

    if (!RtNalSetReceiveFilterEntryEthertype(
        m_hRxQueue,
        0,
        0x88A4,
        FALSE,
        TRUE))
    {
        DEBUG_PRINT("CNicDriver Warning: Failed to set EtherCAT filter (Optional)\n");
    }
    else
    {
        DEBUG_PRINT("CNicDriver: EtherCAT Filter (0x88A4) Applied.\n");
    }

    // 配置一個由 NAL 管理 ownership 的 zero-copy TX Frame。
    m_pTxFrame = RtNalAllocateFrame(MAX_ETHER_FRAME_SIZE);
    if (m_pTxFrame == NULL)
    {
        DEBUG_PRINT("CNicDriver Error: Failed to allocate TX frame.\n");
        Close();
        return false;
    }

    m_pTxFrameArray[0] = m_pTxFrame;
    m_RxPacket.Owner = this;
    m_RxPacket.Length = 0;
    m_RxCallCounter = 0ULL;
    m_LastRxCallDiagnostic = NicRxCallDiagnostic{};

    // 每次 Open() 成功都開始一組新的 TX 診斷統計。
    InterlockedExchange64(&g_nicTxCalls, 0);
    InterlockedExchange64(&g_nicTxSuccess, 0);
    InterlockedExchange64(&g_nicTxFrameBusy, 0);
    InterlockedExchange64(&g_nicTxNotOwner, 0);
    InterlockedExchange64(&g_nicTxSubmitFail, 0);
    InterlockedExchange64(&g_nicTxSubmitted0, 0);
    InterlockedExchange(&g_nicTxLastError, ERROR_SUCCESS);

    DEBUG_PRINT("--- Network Interface Ready ---\n");
    return true;
}

void CNicDriver::Close()
{
    // RtNalFreeFrame() 會等待 NAL 歸還尚在傳送中的 Frame。
    if (m_pTxFrame != NULL)
    {
        RtNalFreeFrame(m_pTxFrame);
        m_pTxFrame = NULL;
        m_pTxFrameArray[0] = NULL;
    }

    if (m_hTxQueue != NULL)
    {
        RtNalReleaseQueue(m_hTxQueue);
        m_hTxQueue = NULL;
    }

    if (m_hRxQueue != NULL)
    {
        RtNalReleaseQueue(m_hRxQueue);
        m_hRxQueue = NULL;
    }

    memset(&m_RxQueueEvents, 0, sizeof(m_RxQueueEvents));
    m_RxPacket.Length = 0;
}

bool CNicDriver::SendPacket(unsigned char* pData, unsigned int length)
{
    // Calls 包含成功、輸入錯誤、ownership busy 與提交失敗。
    InterlockedIncrement64(&g_nicTxCalls);

    // 在接觸 NAL Frame 前先驗證 Handle、資料位址與長度。
    if (m_hTxQueue == NULL ||
        m_pTxFrame == NULL ||
        pData == NULL ||
        length == 0 ||
        length > MAX_ETHER_FRAME_SIZE)
    {
        InterlockedIncrement64(&g_nicTxSubmitFail);
        InterlockedExchange(&g_nicTxLastError, ERROR_INVALID_PARAMETER);
        return false;
    }

    // 只有 Application 擁有 Frame 時才能修改 frameSize 與資料緩衝區。
    // 若 NAL 尚未完成上一筆 TX，直接回傳 false，絕不覆寫使用中的 Frame。
    if (!RtNalIsApplicationFrame(m_pTxFrame))
    {
        const DWORD error = GetLastError();
        InterlockedIncrement64(&g_nicTxFrameBusy);
        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        return false;
    }

    // Ownership 已確認，現在才可安全填入 Ethernet Frame。
    memcpy(m_pTxFrame->frameBufferVirtualAddr, pData, length);
    m_pTxFrame->frameSize = length;

    ULONG submitted = 0;
    // RtNalTransmitEx() 成功後 ownership 交給 NAL，直到 TX complete 歸還。
    const BOOL result = RtNalTransmitEx(
        m_hTxQueue,
        m_pTxFrameArray,
        1,
        &submitted);

    // API 失敗時保留 submitted 與 GetLastError()，供長時間測試定位。
    if (result != TRUE)
    {
        const DWORD error = GetLastError();
        InterlockedIncrement64(&g_nicTxSubmitFail);

        if (submitted == 0)
            InterlockedIncrement64(&g_nicTxSubmitted0);

        if (error == ERROR_NOT_OWNER)
            InterlockedIncrement64(&g_nicTxNotOwner);

        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        return false;
    }

    // 本版本一次只提交一個 Frame，成功條件必須嚴格等於 1。
    if (submitted != 1)
    {
        const DWORD error = GetLastError();
        InterlockedIncrement64(&g_nicTxSubmitFail);

        if (submitted == 0)
            InterlockedIncrement64(&g_nicTxSubmitted0);

        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        return false;
    }

    InterlockedIncrement64(&g_nicTxSuccess);
    return true;
}

void CNicDriver::PublishReceiveCallDiagnostic(
    NicRxCallOutcome outcome,
    BOOL nalCallSucceeded,
    DWORD lastError,
    ULONG length)
{
    NicRxCallDiagnostic diagnostic{};
    diagnostic.CallSequence = ++m_RxCallCounter;
    diagnostic.Outcome = static_cast<uint32_t>(outcome);
    diagnostic.NalCallSucceeded = nalCallSucceeded == TRUE ? 1u : 0u;
    diagnostic.LastError = lastError;
    diagnostic.Length = static_cast<uint32_t>(length);

    // The same Priority-64 owner reads this immediately after ReceivePacket().
    m_LastRxCallDiagnostic = diagnostic;
}

bool CNicDriver::GetLastReceiveCallDiagnostic(
    NicRxCallDiagnostic* pDiagnostic) const
{
    if (pDiagnostic == NULL)
        return false;

    *pDiagnostic = m_LastRxCallDiagnostic;
    return pDiagnostic->CallSequence != 0ULL;
}

unsigned int CNicDriver::ReceivePacket(unsigned char* pBuffer)
{
    // Non-blocking receive. DC-RX.4A preserves the existing return contract
    // while retaining the raw NAL outcome for incident forensics.
    if (m_hRxQueue == NULL)
    {
        PublishReceiveCallDiagnostic(
            NicRxCallOutcome::QueueUnavailable,
            FALSE,
            ERROR_INVALID_HANDLE,
            0);
        return 0;
    }

    if (pBuffer == NULL)
    {
        PublishReceiveCallDiagnostic(
            NicRxCallOutcome::QueueUnavailable,
            FALSE,
            ERROR_INVALID_PARAMETER,
            0);
        return 0;
    }

    m_RxPacket.Length = 0;

    // RtNalReceive() distinguishes an empty queue from an API/driver fault
    // through GetLastError(). ERROR_NO_DATA is normal bounded polling.
    const BOOL nalResult = RtNalReceive(m_hRxQueue);
    if (nalResult != TRUE)
    {
        const DWORD error = GetLastError();
        PublishReceiveCallDiagnostic(
            error == ERROR_NO_DATA
            ? NicRxCallOutcome::NoData
            : NicRxCallOutcome::NalError,
            FALSE,
            error,
            0);
        return 0;
    }

    const ULONG reportedLength = m_RxPacket.Length;
    if (reportedLength == 0 ||
        reportedLength > MAX_ETHER_RX_BUFFER_SIZE)
    {
        PublishReceiveCallDiagnostic(
            NicRxCallOutcome::InvalidLength,
            TRUE,
            ERROR_INVALID_DATA,
            reportedLength);
        return 0;
    }

    ULONG copyLength = reportedLength;
    if (copyLength > MAX_ETHER_FRAME_SIZE)
        copyLength = MAX_ETHER_FRAME_SIZE;

    memcpy(pBuffer, m_RxPacket.Data, copyLength);
    PublishReceiveCallDiagnostic(
        NicRxCallOutcome::Frame,
        TRUE,
        ERROR_SUCCESS,
        reportedLength);

    return static_cast<unsigned int>(copyLength);
}

bool CNicDriver::IsReceiveNotificationAvailable() const
{
    return
        m_hRxQueue != NULL &&
        m_RxQueueEvents.hRxEvent != NULL;
}

NicRxWaitResult CNicDriver::WaitForReceiveNotification(
    ULONGLONG timeout100ns,
    DWORD* pLastError)
{
    if (pLastError != NULL)
        *pLastError = ERROR_SUCCESS;

    if (!IsReceiveNotificationAvailable())
    {
        if (pLastError != NULL)
            *pLastError = ERROR_NOT_READY;

        return NicRxWaitResult::Unavailable;
    }

    ULARGE_INTEGER waitInterval;
    waitInterval.QuadPart = timeout100ns;

    DWORD waitResult = WAIT_FAILED;

    // Stop event 放在 index 0；若 RX 與 Stop 同時 signaled，停止流程優先。
    if (m_RxQueueEvents.hStopEvent != NULL &&
        m_RxQueueEvents.hStopEvent != m_RxQueueEvents.hRxEvent)
    {
        HANDLE waitHandles[2] =
        {
            m_RxQueueEvents.hStopEvent,
            m_RxQueueEvents.hRxEvent
        };

        waitResult = RtWaitForMultipleObjectsEx(
            2,
            waitHandles,
            FALSE,
            &waitInterval);

        if (waitResult == WAIT_OBJECT_0)
        {
            if (pLastError != NULL)
                *pLastError = ERROR_OPERATION_ABORTED;

            return NicRxWaitResult::Stopped;
        }

        if (waitResult == WAIT_OBJECT_0 + 1U)
            return NicRxWaitResult::Signaled;
    }
    else
    {
        waitResult = RtWaitForSingleObjectEx(
            m_RxQueueEvents.hRxEvent,
            &waitInterval);

        if (waitResult == WAIT_OBJECT_0)
            return NicRxWaitResult::Signaled;
    }

    if (waitResult == WAIT_TIMEOUT)
        return NicRxWaitResult::Timeout;

    DWORD error =
        waitResult == WAIT_FAILED
        ? GetLastError()
        : ERROR_GEN_FAILURE;

    if (pLastError != NULL)
        *pLastError = error;

    return NicRxWaitResult::Failed;
}

bool CNicDriver::AcquireTxQueueInternal()
{
    // 掃描 NAL Queue，取得第一個可用 TX Queue。
    const INT queueCount = RtNalGetNumberOfQueues();
    RTNAL_QUEUE info;
    RTNAL_QUEUE_CRITERIA criteria;
    RTNAL_QUEUE_EVENTS events;

    for (int i = 0; i < queueCount; ++i)
    {
        if (!RtNalGetQueueInfoByIndex(i, &info))
            continue;

        if (info.queueInfo.queueType != RTNAL_QUEUE_TYPE_TX)
            continue;

        memset(&criteria, 0, sizeof(criteria));
        memset(&events, 0, sizeof(events));

        criteria.method = RTNAL_DEVICE_NAME_QUEUE_EXACT;
        criteria.deviceQueueNumber = info.queueInfo.deviceQueueNumber;
        criteria.queueType = RTNAL_QUEUE_TYPE_TX;
        // TX complete thread 負責在硬體送出完成後歸還 NAL Frame ownership。
        criteria.flags = RTNAL_USE_TX_COMPLETE_EVENT_FLAG;
        memcpy(
            criteria.deviceName,
            info.deviceInfo.deviceName,
            RTNAL_DEVICE_NAME_LENGTH);

        m_hTxQueue = RtNalAcquireQueue(&criteria, &info, &events);
        if (m_hTxQueue == NULL)
            continue;

        memcpy(m_MacAddress, info.deviceInfo.macAddress, 6);

        DEBUG_PRINT("CNicDriver: [TX] Queue Acquired\n");
        DEBUG_PRINT("    > Name: %s\n", info.deviceInfo.deviceName);
        DEBUG_PRINT(
            "    > MAC : %02X-%02X-%02X-%02X-%02X-%02X\n",
            m_MacAddress[0],
            m_MacAddress[1],
            m_MacAddress[2],
            m_MacAddress[3],
            m_MacAddress[4],
            m_MacAddress[5]);
        return true;
    }

    DEBUG_PRINT("CNicDriver Error: No available RTX64 TX Queue found!\n");
    return false;
}

bool CNicDriver::AcquireRxQueueInternal()
{
    // 掃描 NAL Queue，取得第一個可用 RX Queue。
    const INT queueCount = RtNalGetNumberOfQueues();
    RTNAL_QUEUE info;
    RTNAL_QUEUE_CRITERIA criteria;

    memset(&m_RxQueueEvents, 0, sizeof(m_RxQueueEvents));

    for (int i = 0; i < queueCount; ++i)
    {
        if (!RtNalGetQueueInfoByIndex(i, &info))
            continue;

        if (info.queueInfo.queueType != RTNAL_QUEUE_TYPE_RX)
            continue;

        memset(&criteria, 0, sizeof(criteria));
        memset(&m_RxQueueEvents, 0, sizeof(m_RxQueueEvents));

        criteria.method = RTNAL_DEVICE_NAME_QUEUE_EXACT;
        criteria.deviceQueueNumber = info.queueInfo.deviceQueueNumber;
        criteria.queueType = RTNAL_QUEUE_TYPE_RX;
        // RX notification 由 NAL IST 觸發；上層使用有界 event wait，失敗自動 fallback。
        criteria.flags = RTNAL_USE_RX_EVENT_FLAG;
        memcpy(
            criteria.deviceName,
            info.deviceInfo.deviceName,
            RTNAL_DEVICE_NAME_LENGTH);

        m_hRxQueue = RtNalAcquireQueue(
            &criteria,
            &info,
            &m_RxQueueEvents);

        if (m_hRxQueue == NULL)
            continue;

        DEBUG_PRINT("CNicDriver: [RX] Queue Acquired\n");
        DEBUG_PRINT("    > Name: %s\n", info.deviceInfo.deviceName);
        DEBUG_PRINT(
            "    > RX Event: %s | Stop Event: %s\n",
            m_RxQueueEvents.hRxEvent != NULL ? "READY" : "UNAVAILABLE",
            m_RxQueueEvents.hStopEvent != NULL ? "READY" : "UNAVAILABLE");
        return true;
    }

    memset(&m_RxQueueEvents, 0, sizeof(m_RxQueueEvents));
    DEBUG_PRINT("CNicDriver Error: No available RTX64 RX Queue found!\n");
    return false;
}
