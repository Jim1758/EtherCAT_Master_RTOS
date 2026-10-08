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
 * - NAL priorities are observed from the acquired TX interface at Open.
 * - NIC-TX-CONFIG is startup readback, not proof of live thread scheduling.
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

// PBC3J FIX3: failure-only cumulative diagnostics. Each field is atomic but
// fields are NOT a coherent transaction; safety uses the caller-local receipt.
struct NicTxFailureAdvisory
{
    volatile LONGLONG count;
    volatile LONGLONG lastCallSequence;
    volatile LONG error;
    volatile LONG errorValid;
    volatile LONG submitted;
};
static NicTxFailureAdvisory g_nicTxFailureAdvisory[4] = {};
static volatile LONGLONG g_nicTxFailurePublications = 0;
static volatile LONGLONG g_nicTxOwnershipNotOwner = 0;
static volatile LONGLONG g_nicTxOwnershipInvalidAddress = 0;
static volatile LONGLONG g_nicTxOwnershipNotReady = 0;
static volatile LONGLONG g_nicTxOwnershipOther = 0;
// EDM30 process-lifetime trace: Open starts a new call identity but never
// resets or overwrites diagnostics still owned by the low-priority consumer.
static volatile LONGLONG g_nicTxOpenGeneration = 0;
static NicTxFailureTrace g_nicTxFailureTrace{};

// Diagnostic reads preserve the exact caller thread error. Never inspect a
// NAL-owned frame: packet identity always comes from this caller-owned image.
template<typename T>
static void CaptureNicTxDatagram(T& value, const unsigned char* data,
    unsigned int length) noexcept
{
    if (data == nullptr || length < 26U || length > MAX_ETHER_FRAME_SIZE ||
        data[12] != 0x88U || data[13] != 0xA4U) return;
    value.datagramValid = 1U;
    value.command = data[16]; value.index = data[17];
    value.addressLow = static_cast<std::uint32_t>(data[18]) |
        (static_cast<std::uint32_t>(data[19]) << 8U);
    value.addressHigh = static_cast<std::uint32_t>(data[20]) |
        (static_cast<std::uint32_t>(data[21]) << 8U);
}

static void CaptureNicTxObservation(NicTxFailureEvent& event) noexcept
{
    const DWORD savedError = GetLastError();
    LARGE_INTEGER qpc{};
    const BOOL qpcResult = RtQueryPerformanceCounter(&qpc);
    if (qpcResult && qpc.QuadPart >= 0)
    {
        event.failureQpc = qpc.QuadPart;
        event.failureQpcValid = 1U;
    }
    const int priority = RtGetThreadPriority(GetCurrentThread());
    event.observedThreadPriority = priority;
    if (priority != THREAD_PRIORITY_ERROR_RETURN && priority >= 0 && priority <= 127)
    {
        event.observedThreadPriorityValid = 1U;
    }
    SetLastError(savedError);
}

// Startup-only bounded readback. A partial enumeration, duplicate MAC match,
// or error is explicitly invalid. No NAL setting is changed.
static NicTxInterfaceObservation ObserveNicTxInterface(
    const unsigned char* mac) noexcept
{
    NicTxInterfaceObservation result{};
    const DWORD savedError = GetLastError();
    for (DWORD index = 0U; index < 64U; ++index)
    {
        RTNAL_INTERFACE info{};
        if (!RtNalEnumInterfaceInfo(&info, index))
        {
            result.enumerationError = GetLastError();
            result.enumerationComplete =
                result.enumerationError == ERROR_NO_MORE_ITEMS ? 1U : 0U;
            break;
        }
        ++result.enumerated;
        if (memcmp(info.MacAddress, mac, 6U) != 0) continue;
        ++result.matches;
        result.version = info.Version;
        result.intPriority = info.IntPriority;
        result.intIdealProcessor = info.IntIdealProcessor;
        result.txCompletePriority = info.TxCompletePriority;
        result.txCompleteIdealProcessor = info.TxCompleteIdealProcessor;
        result.numTxBuffers = info.NumTxBuffers;
    }
    if (result.enumerationComplete == 0U)
        result.status = result.enumerated == 64U ? 3U : 2U;
    else if (result.matches == 0U) result.status = 4U;
    else if (result.matches != 1U) result.status = 5U;
    else if (result.intPriority > 127U || result.txCompletePriority > 127U)
        result.status = 6U;
    else result.status = 1U;
    result.valid = result.status == 1U ? 1U : 0U;
    SetLastError(savedError);
    return result;
}

static const char* NicTxCallerName(NicTxCaller caller) noexcept
{
    switch (caller)
    {
    case NicTxCaller::ControlDatagram: return "CONTROL_DATAGRAM";
    case NicTxCaller::RuntimeLRW: return "RUNTIME_LRW";
    case NicTxCaller::RuntimeLRW_FRMW: return "RUNTIME_LRW_FRMW";
    case NicTxCaller::RuntimeSDO: return "RUNTIME_SDO";
    case NicTxCaller::RuntimeEscDiag: return "RUNTIME_ESC_DIAG";
    case NicTxCaller::OtherLRW: return "OTHER_LRW";
    case NicTxCaller::OtherLRW_FRMW: return "OTHER_LRW_FRMW";
    default: return "UNSPECIFIED";
    }
}

static void PrintNicTxFailureTrace() noexcept
{
    for (std::size_t drained = 0U; drained < NicTxFailureTrace::Capacity; ++drained)
    {
        NicTxFailureEvent event{};
        if (!g_nicTxFailureTrace.TryPop(event)) break;
        const NicTxReceipt& call = event.receipt;
        const NicTxCallContext& c = event.context;
        const NicTxPreviousInvocation& previous = event.previous;
        const NicTxCallContext& pc = previous.context;
        const NicTxInterfaceObservation& config = event.interfaceObservation;
        // Each bounded row joins by publication/open/call. These are software
        // observations; neither NAL return nor ownership observation is drive ACK.
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=RECEIPT publication=%llu open=%llu call=%llu coherent=1 failureOnly=%lu kind=%s notDriveAck=1 caller=%s reason=%lu length=%lu nalInvoked=%lu apiResult=%ld submitted=%lu error=0x%08lX errorValid=%lu\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration),
            static_cast<unsigned long long>(call.callSequence),
            event.kind == NicTxEventKind::Failure ? 1UL : 0UL,
            event.kind == NicTxEventKind::Failure ? "FAILURE" : "OWNERSHIP_RETURNED",
            NicTxCallerName(c.caller), static_cast<unsigned long>(call.reason),
            static_cast<unsigned long>(call.length), static_cast<unsigned long>(call.nalInvoked),
            static_cast<long>(call.apiResult), static_cast<unsigned long>(call.submitted),
            static_cast<unsigned long>(call.error), static_cast<unsigned long>(call.errorValid));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=POOL publication=%llu open=%llu call=%llu frameSlot=%lu observedFrameSlot=%lu previousFrameSlot=%lu slots=%lu applicationOwned=%lu protocolFence=%lu producerOverlap=%lu slotState=%lu responseTrustTainted=%lu responseTrustValid=%lu readOnlyEscSlot=%lu cyclicFrame=%lu historyScope=FRAME responseIsNotOwnership=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration),
            static_cast<unsigned long long>(call.callSequence),
            static_cast<unsigned long>(call.frameSlot),
            static_cast<unsigned long>(event.observedFrameSlot),
            static_cast<unsigned long>(previous.receipt.frameSlot),
            static_cast<unsigned long>(event.poolSlotCount),
            static_cast<unsigned long>(event.poolOwnedCount),
            static_cast<unsigned long>(event.protocolFence),
            static_cast<unsigned long>(event.producerOverlap),
            static_cast<unsigned long>(event.slotState),
            static_cast<unsigned long>(event.responseTrustTainted),
            static_cast<unsigned long>(event.responseTrustValid),
            static_cast<unsigned long>(event.slotReadOnlyEsc),
            static_cast<unsigned long>(event.currentCyclicFrame));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=PACKET publication=%llu open=%llu call=%llu observationQpc=%lld qpcValid=%lu datagramValid=%lu cmd=%lu idx=%lu addressLow=0x%04lX addressHigh=0x%04lX notDriveAck=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration),
            static_cast<unsigned long long>(call.callSequence), static_cast<long long>(event.failureQpc),
            static_cast<unsigned long>(event.failureQpcValid), static_cast<unsigned long>(event.datagramValid),
            static_cast<unsigned long>(event.command), static_cast<unsigned long>(event.index),
            static_cast<unsigned long>(event.addressLow), static_cast<unsigned long>(event.addressHigh));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=AUTHORITY publication=%llu open=%llu call=%llu pdoTick=%llu pdoTickValid=%lu sourceTick=%llu sourceTickValid=%lu authorityHeld=%lu packedOwner=0x%016llX packedEpoch=0x%016llX edmGuardHeld=%lu edmFeedbackPublication=%llu notDriveAck=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration),
            static_cast<unsigned long long>(call.callSequence),
            static_cast<unsigned long long>(c.pdoTick), static_cast<unsigned long>(c.pdoTickValid),
            static_cast<unsigned long long>(c.sourceTick), static_cast<unsigned long>(c.sourceTickValid),
            static_cast<unsigned long>(c.authorityValid), static_cast<unsigned long long>(c.packedOwnerState),
            static_cast<unsigned long long>(c.packedExecutionPublication), static_cast<unsigned long>(c.edmGuardHeld),
            static_cast<unsigned long long>(c.edmFeedbackPublication));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=PREVIOUS publication=%llu open=%llu call=%llu historyCoherent=%lu historyGaps=%llu previousValid=%lu previousOpen=%llu previousCall=%llu caller=%s reason=%lu length=%lu nalInvoked=%lu apiResult=%ld submitted=%lu error=0x%08lX errorValid=%lu\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration),
            static_cast<unsigned long long>(call.callSequence), static_cast<unsigned long>(event.historyCoherent),
            static_cast<unsigned long long>(event.historyGaps), static_cast<unsigned long>(previous.valid),
            static_cast<unsigned long long>(previous.openGeneration),
            static_cast<unsigned long long>(previous.receipt.callSequence), NicTxCallerName(pc.caller),
            static_cast<unsigned long>(previous.receipt.reason), static_cast<unsigned long>(previous.receipt.length),
            static_cast<unsigned long>(previous.receipt.nalInvoked), static_cast<long>(previous.receipt.apiResult),
            static_cast<unsigned long>(previous.receipt.submitted), static_cast<unsigned long>(previous.receipt.error),
            static_cast<unsigned long>(previous.receipt.errorValid));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=PREV_PACKET publication=%llu open=%llu call=%llu datagramValid=%lu cmd=%lu idx=%lu addressLow=0x%04lX addressHigh=0x%04lX pdoTick=%llu pdoTickValid=%lu sourceTick=%llu sourceTickValid=%lu\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration), static_cast<unsigned long long>(call.callSequence),
            static_cast<unsigned long>(previous.datagramValid), static_cast<unsigned long>(previous.command),
            static_cast<unsigned long>(previous.index), static_cast<unsigned long>(previous.addressLow),
            static_cast<unsigned long>(previous.addressHigh), static_cast<unsigned long long>(pc.pdoTick),
            static_cast<unsigned long>(pc.pdoTickValid), static_cast<unsigned long long>(pc.sourceTick),
            static_cast<unsigned long>(pc.sourceTickValid));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=PREV_AUTHORITY publication=%llu open=%llu call=%llu authorityHeld=%lu packedOwner=0x%016llX packedEpoch=0x%016llX edmGuardHeld=%lu edmFeedbackPublication=%llu notDriveAck=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration), static_cast<unsigned long long>(call.callSequence),
            static_cast<unsigned long>(pc.authorityValid), static_cast<unsigned long long>(pc.packedOwnerState),
            static_cast<unsigned long long>(pc.packedExecutionPublication), static_cast<unsigned long>(pc.edmGuardHeld),
            static_cast<unsigned long long>(pc.edmFeedbackPublication));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=TIMING publication=%llu open=%llu call=%llu previousReturnQpc=%lld previousReturnQpcValid=%lu previousAgeQpc=%llu ageValid=%lu frequency=%llu frequencyValid=%lu boundary=NAL_RETURN_TO_OBSERVATION notCompletionLatency=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration), static_cast<unsigned long long>(call.callSequence),
            static_cast<long long>(previous.returnQpc), static_cast<unsigned long>(previous.returnQpcValid),
            static_cast<unsigned long long>(event.previousAgeQpc), static_cast<unsigned long>(event.previousAgeQpcValid),
            static_cast<unsigned long long>(event.qpcFrequency), static_cast<unsigned long>(event.qpcFrequencyValid));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=BUSY publication=%llu open=%llu call=%llu firstCall=%llu lastCall=%llu count=%llu firstQpc=%lld firstQpcValid=%lu lastQpc=%lld lastQpcValid=%lu observationOnly=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration), static_cast<unsigned long long>(call.callSequence),
            static_cast<unsigned long long>(event.busy.firstCall), static_cast<unsigned long long>(event.busy.lastCall),
            static_cast<unsigned long long>(event.busy.count), static_cast<long long>(event.busy.firstQpc),
            static_cast<unsigned long>(event.busy.firstQpcValid), static_cast<long long>(event.busy.lastQpc),
            static_cast<unsigned long>(event.busy.lastQpcValid));
        RtPrintf("[NIC-TX-EVENT] build=EDM45 part=SCHEDULING publication=%llu open=%llu call=%llu observedPriority=%ld observedPriorityValid=%lu configAtOpen=1 configValid=%lu configStatus=%lu intPriority=%lu intProcessor=%lu txPriority=%lu txProcessor=%lu txBuffers=%lu noPriorityChange=1\n",
            static_cast<unsigned long long>(event.publicationSequence),
            static_cast<unsigned long long>(event.openGeneration), static_cast<unsigned long long>(call.callSequence),
            static_cast<long>(event.observedThreadPriority), static_cast<unsigned long>(event.observedThreadPriorityValid),
            static_cast<unsigned long>(config.valid), static_cast<unsigned long>(config.status),
            static_cast<unsigned long>(config.intPriority), static_cast<unsigned long>(config.intIdealProcessor),
            static_cast<unsigned long>(config.txCompletePriority), static_cast<unsigned long>(config.txCompleteIdealProcessor),
            static_cast<unsigned long>(config.numTxBuffers));
    }
    static std::uint64_t lastPrintedDropped = 0ULL;
    const std::uint64_t dropped = g_nicTxFailureTrace.Dropped();
    if (dropped != lastPrintedDropped)
    {
        lastPrintedDropped = dropped;
        RtPrintf("[NIC-TX-TRACE-LOSS] build=EDM45 countersAdvisory=1 scope=PROCESS attempts=%llu dropped=%llu capacity=16 noRecoveryAuthority=1\n",
            static_cast<unsigned long long>(g_nicTxFailureTrace.Attempts()),
            static_cast<unsigned long long>(dropped));
    }
}

void PrintNicTransmitFailureDiagnostic() noexcept
{
    PrintNicTxFailureTrace();
    // Called by the existing low-priority diagnostic consumer. No queue/NAL
    // operations, retries or producer-side formatting are introduced.
    static LONGLONG lastPrintedPublications = 0;
    const LONGLONG publications =
        InterlockedCompareExchange64(&g_nicTxFailurePublications, 0, 0);
    if (publications == 0 || publications == lastPrintedPublications) return;
    lastPrintedPublications = publications;
    const auto read64 = [](volatile LONGLONG* value) noexcept -> LONGLONG
    { return InterlockedCompareExchange64(value, 0, 0); };
    RtPrintf("[NIC-TX-FAIL] build=EDM45 part=TOTAL cumulative=1 advisory=1 coherent=0 failureScope=PROCESS callScope=OPEN failures=%llu calls=%llu accepted=%llu preInvalid=%llu preOwnership=%llu nalFailed=%llu countMismatch=%llu\n",
        static_cast<unsigned long long>(publications),
        static_cast<unsigned long long>(read64(&g_nicTxCalls)),
        static_cast<unsigned long long>(read64(&g_nicTxSuccess)),
        static_cast<unsigned long long>(read64(&g_nicTxFailureAdvisory[0].count)),
        static_cast<unsigned long long>(read64(&g_nicTxFailureAdvisory[1].count)),
        static_cast<unsigned long long>(read64(&g_nicTxFailureAdvisory[2].count)),
        static_cast<unsigned long long>(read64(&g_nicTxFailureAdvisory[3].count)));
    RtPrintf("[NIC-TX-FAIL] build=EDM45 part=OWNERSHIP cumulative=1 advisory=1 coherent=0 failures=%llu ownershipNotOwner=%llu ownershipInvalidAddress=%llu ownershipNotReady=%llu ownershipOther=%llu legacyFrameBusy=%llu legacyNotOwner=%llu legacySubmitFail=%llu legacySubmitted0=%llu\n",
        static_cast<unsigned long long>(publications),
        static_cast<unsigned long long>(read64(&g_nicTxOwnershipNotOwner)),
        static_cast<unsigned long long>(read64(&g_nicTxOwnershipInvalidAddress)),
        static_cast<unsigned long long>(read64(&g_nicTxOwnershipNotReady)),
        static_cast<unsigned long long>(read64(&g_nicTxOwnershipOther)),
        static_cast<unsigned long long>(read64(&g_nicTxFrameBusy)),
        static_cast<unsigned long long>(read64(&g_nicTxNotOwner)),
        static_cast<unsigned long long>(read64(&g_nicTxSubmitFail)),
        static_cast<unsigned long long>(read64(&g_nicTxSubmitted0)));
    const char* const names[4] = {
        "PRE_INVALID", "PRE_OWNERSHIP", "NAL_FAILED", "COUNT_MISMATCH" };
    for (unsigned i = 0U; i < 4U; ++i)
    {
        NicTxFailureAdvisory& slot = g_nicTxFailureAdvisory[i];
        if (read64(&slot.count) == 0) continue;
        RtPrintf("[NIC-TX-REASON] build=PBC3J_FIX3 advisory=1 coherent=0 reason=%s count=%llu lastCall=%llu lastError=0x%08lX errorValid=%lu submitted=%lu\n",
            names[i], static_cast<unsigned long long>(read64(&slot.count)),
            static_cast<unsigned long long>(read64(&slot.lastCallSequence)),
            static_cast<unsigned long>(InterlockedCompareExchange(&slot.error, 0, 0)),
            static_cast<unsigned long>(InterlockedCompareExchange(&slot.errorValid, 0, 0)),
            static_cast<unsigned long>(InterlockedCompareExchange(&slot.submitted, 0, 0)));
    }
}

CNicDriver::CNicDriver()
    : m_hTxQueue(NULL),
    m_hRxQueue(NULL)
{
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

    // EDM45: allocate the bounded pool only while Open/Close are quiescent.
    // Each slot stays untouched while NAL owns it; there is no runtime growth.
    for (std::size_t i = 0U; i < NicTxFrameLifecycle::SlotCount; ++i)
    {
        m_TxFrames[i] = RtNalAllocateFrame(MAX_ETHER_FRAME_SIZE);
        if (m_TxFrames[i] == NULL)
        {
            DEBUG_PRINT("CNicDriver Error: Failed to allocate TX frame pool.\n");
            Close();
            return false;
        }
    }
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

    // FIX3 advisory failure counters intentionally span all Open calls in this process.
    m_TxOpenGeneration = static_cast<std::uint64_t>(
        InterlockedIncrement64(&g_nicTxOpenGeneration));

    const DWORD txDiagnosticSavedError = GetLastError();
    LARGE_INTEGER txQpcFrequency{};
    const BOOL txQpcFrequencyResult = RtQueryPerformanceFrequency(&txQpcFrequency);
    const bool txQpcFrequencyValid = txQpcFrequencyResult && txQpcFrequency.QuadPart > 0;
    m_TxLifecycle.Reset(m_TxOpenGeneration);
    for (std::size_t i = 0U; i < NicTxFrameLifecycle::SlotCount; ++i)
        m_TxOwnershipTrace[i].Reset(txQpcFrequencyValid ?
            static_cast<std::uint64_t>(txQpcFrequency.QuadPart) : 0ULL, txQpcFrequencyValid);
    m_TxInterfaceObservation = ObserveNicTxInterface(m_MacAddress);
    const NicTxInterfaceObservation& config = m_TxInterfaceObservation;
    DEBUG_PRINT("[NIC-TX-CONFIG] build=EDM45 open=%llu valid=%lu status=%lu matches=%lu enumerationComplete=%lu enumerationError=0x%08lX enumerated=%lu cap=64 version=%lu intPriority=%lu intProcessor=%lu txPriority=%lu txProcessor=%lu txBuffers=%lu observedOnly=1 poolSlots=4 protocolFence=1 responseIsNotOwnership=1\n",
        static_cast<unsigned long long>(m_TxOpenGeneration),
        static_cast<unsigned long>(config.valid), static_cast<unsigned long>(config.status),
        static_cast<unsigned long>(config.matches),
        static_cast<unsigned long>(config.enumerationComplete),
        static_cast<unsigned long>(config.enumerationError), static_cast<unsigned long>(config.enumerated),
        static_cast<unsigned long>(config.version), static_cast<unsigned long>(config.intPriority),
        static_cast<unsigned long>(config.intIdealProcessor), static_cast<unsigned long>(config.txCompletePriority),
        static_cast<unsigned long>(config.txCompleteIdealProcessor), static_cast<unsigned long>(config.numTxBuffers));

    SetLastError(txDiagnosticSavedError);
    DEBUG_PRINT("--- Network Interface Ready ---\n");
    return true;
}

void CNicDriver::Close()
{
    // Quiescent teardown only: NAL defers reclamation of its owned frames.
    // FreeFrame is not a documented synchronous completion barrier. Partial
    // Open failure follows the same cleanup; no callback lifetime is added.
    for (std::size_t i = 0U; i < NicTxFrameLifecycle::SlotCount; ++i)
    {
        if (m_TxFrames[i] != NULL)
        {
            RtNalFreeFrame(m_TxFrames[i]);
            m_TxFrames[i] = NULL;
        }
    }
    m_TxLifecycle.Invalidate();

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

bool CNicDriver::SendPacket(unsigned char* pData, unsigned int length,
    NicTxReceipt* receipt, const NicTxCallContext* context)
{
    NicTxReceipt call{};
    call.callSequence = static_cast<std::uint64_t>(
        InterlockedIncrement64(&g_nicTxCalls));
    call.length = static_cast<std::uint32_t>(length);
    call.openGeneration = m_TxOpenGeneration;
    NicTxProducerScope producerScope(m_TxProducerActive);
    std::size_t observedSlot = NicTxFrameLifecycle::NoSlot;
    std::uint32_t ownedCount = 0U;
    bool protocolFence = false;
    const bool isLrwFrame = NicTxFrameLifecycle::IsCyclicFrame(pData, length, false);
    const bool isLrwFrmwFrame = NicTxFrameLifecycle::IsCyclicFrame(pData, length, true);
    const bool currentIsRuntimeCyclic = context != nullptr &&
        ((context->caller == NicTxCaller::RuntimeLRW && isLrwFrame) ||
         (context->caller == NicTxCaller::RuntimeLRW_FRMW && isLrwFrmwFrame));
    const bool readOnlyEscProbe = context != nullptr &&
        context->caller == NicTxCaller::RuntimeEscDiag &&
        NicTxFrameLifecycle::IsReadOnlyEscProbe(pData, length);
    LARGE_INTEGER nalReturnQpc{};
    bool nalReturnQpcValid = false;

    const auto populateEvent = [&](NicTxFailureEvent& event) noexcept
    {
        event.receipt = call;
        if (context != nullptr) event.context = *context;
        event.openGeneration = call.openGeneration;
        event.interfaceObservation = m_TxInterfaceObservation;
        event.poolOwnedCount = ownedCount;
        event.protocolFence = protocolFence ? 1U : 0U;
        event.producerOverlap = producerScope.Held() ? 0U : 1U;
        event.currentCyclicFrame = currentIsRuntimeCyclic ? 1U : 0U;
        if (producerScope.Held())
        {
            event.responseTrustValid = 1U;
            event.responseTrustTainted = m_TxLifecycle.ResponseTrustTainted() ? 1U : 0U;
        }
        if (observedSlot < NicTxFrameLifecycle::SlotCount)
        {
            event.observedFrameSlot = static_cast<std::uint32_t>(observedSlot + 1U);
            event.slotState = static_cast<std::uint32_t>(m_TxLifecycle.SlotState(observedSlot));
            event.slotReadOnlyEsc = m_TxLifecycle.IsReadOnlyEscSlot(observedSlot) ? 1U : 0U;
        }
        CaptureNicTxObservation(event);
    };
    const auto finish = [&](NicTxReason reason, bool accepted) noexcept -> bool
    {
        call.reason = reason;
        if (call.nalInvoked != 0U && observedSlot < NicTxFrameLifecycle::SlotCount)
        {
            // Mailbox/control FPRD/FPWR share index identities with startup
            // traffic. Their RX cannot grant response-only pool advancement.
            const bool responseTrustRelevant = isLrwFrame || isLrwFrmwFrame ||
                (context != nullptr &&
                    (context->caller == NicTxCaller::RuntimeLRW ||
                     context->caller == NicTxCaller::RuntimeLRW_FRMW ||
                     context->caller == NicTxCaller::OtherLRW ||
                     context->caller == NicTxCaller::OtherLRW_FRMW));
            m_TxLifecycle.RecordInvocation(observedSlot, call,
                currentIsRuntimeCyclic, responseTrustRelevant, readOnlyEscProbe);
        }
        if (!accepted)
        {
            unsigned advisoryIndex = 3U;
            if (reason == NicTxReason::PreSubmitInvalidInput) advisoryIndex = 0U;
            else if (reason == NicTxReason::PreSubmitOwnershipRejected) advisoryIndex = 1U;
            else if (reason == NicTxReason::NalCallFailed) advisoryIndex = 2U;
            NicTxFailureAdvisory& advisory = g_nicTxFailureAdvisory[advisoryIndex];
            InterlockedExchange64(&advisory.lastCallSequence,
                static_cast<LONGLONG>(call.callSequence));
            InterlockedExchange(&advisory.error, static_cast<LONG>(call.error));
            InterlockedExchange(&advisory.errorValid, static_cast<LONG>(call.errorValid));
            InterlockedExchange(&advisory.submitted, static_cast<LONG>(call.submitted));
            InterlockedIncrement64(&advisory.count);
            InterlockedIncrement64(&g_nicTxFailurePublications);
            NicTxFailureEvent event{};
            populateEvent(event);
            if (reason != NicTxReason::PreSubmitInvalidInput)
                CaptureNicTxDatagram(event, pData, length);
            if (observedSlot < NicTxFrameLifecycle::SlotCount)
            {
                NicTxOwnershipTraceState& trace = m_TxOwnershipTrace[observedSlot];
                NicTxOwnershipTraceState::CallScope diagnosticScope(trace);
                trace.ObserveFailure(diagnosticScope, event,
                    call.errorValid != 0U && call.error == ERROR_NOT_OWNER);
                trace.ValidateEvent(diagnosticScope, event);
            }
            (void)g_nicTxFailureTrace.TryPublish(event);
        }
        if (call.nalInvoked != 0U && observedSlot < NicTxFrameLifecycle::SlotCount)
        {
            // Every NAL invocation is retained, including ambiguous failures.
            // Nothing here reclassifies such a receipt as definitely unsent.
            NicTxPreviousInvocation invocation{};
            invocation.receipt = call;
            if (context != nullptr) invocation.context = *context;
            invocation.openGeneration = call.openGeneration;
            invocation.returnQpc = nalReturnQpcValid ? nalReturnQpc.QuadPart : 0;
            invocation.returnQpcValid = nalReturnQpcValid ? 1U : 0U;
            CaptureNicTxDatagram(invocation, pData, length);
            NicTxOwnershipTraceState& trace = m_TxOwnershipTrace[observedSlot];
            NicTxOwnershipTraceState::CallScope diagnosticScope(trace);
            trace.RecordInvocation(diagnosticScope, invocation);
        }
        if (receipt != nullptr) *receipt = call;
        return accepted;
    };

    // A losing producer never reads pool state or calls NAL. This is a real
    // pre-submit rejection, unlike the older diagnostic-only CallScope.
    if (!producerScope.Held())
    {
        call.error = ERROR_BUSY;
        call.errorValid = 1U;
        InterlockedIncrement64(&g_nicTxFrameBusy);
        InterlockedExchange(&g_nicTxLastError, ERROR_BUSY);
        SetLastError(ERROR_BUSY);
        return finish(NicTxReason::PreSubmitOwnershipRejected, false);
    }
    if (m_hTxQueue == NULL || pData == NULL ||
        length == 0 || length > MAX_ETHER_FRAME_SIZE)
    {
        call.error = ERROR_INVALID_PARAMETER;
        call.errorValid = 1U;
        InterlockedIncrement64(&g_nicTxSubmitFail);
        InterlockedExchange(&g_nicTxLastError, ERROR_INVALID_PARAMETER);
        return finish(NicTxReason::PreSubmitInvalidInput, false);
    }

    bool owned[NicTxFrameLifecycle::SlotCount] = {};
    DWORD ownershipError[NicTxFrameLifecycle::SlotCount] = {};
    std::size_t availableSlot = NicTxFrameLifecycle::NoSlot;
    std::size_t ownershipFaultSlot = NicTxFrameLifecycle::NoSlot;
    // One fixed scan: no spin, retry, allocation, or assumed return latency.
    // Ownership-only reuse remains available after cyclic trust is tainted.
    for (std::size_t i = 0U; i < NicTxFrameLifecycle::SlotCount; ++i)
    {
        if (m_TxFrames[i] == NULL)
        {
            call.error = ERROR_INVALID_PARAMETER;
            call.errorValid = 1U;
            InterlockedIncrement64(&g_nicTxSubmitFail);
            InterlockedExchange(&g_nicTxLastError, ERROR_INVALID_PARAMETER);
                return finish(NicTxReason::PreSubmitInvalidInput, false);
        }
        owned[i] = RtNalIsApplicationFrame(m_TxFrames[i]) != FALSE;
        if (owned[i])
        {
            ++ownedCount;
            if (availableSlot == NicTxFrameLifecycle::NoSlot) availableSlot = i;
        }
        else
        {
            ownershipError[i] = GetLastError();
            if (ownershipError[i] != ERROR_NOT_OWNER &&
                ownershipFaultSlot == NicTxFrameLifecycle::NoSlot)
                ownershipFaultSlot = i;
        }
    }
    for (std::size_t i = 0U; i < NicTxFrameLifecycle::SlotCount; ++i)
    {
        if (!owned[i]) continue;
        observedSlot = i;
        NicTxOwnershipTraceState& trace = m_TxOwnershipTrace[i];
        NicTxOwnershipTraceState::CallScope diagnosticScope(trace);
        if (trace.HasBusyEpisode(diagnosticScope))
        {
            NicTxFailureEvent event{};
            populateEvent(event);
            CaptureNicTxDatagram(event, pData, length);
            if (trace.ObserveOwnershipReturn(diagnosticScope, event))
            {
                trace.ValidateEvent(diagnosticScope, event);
                (void)g_nicTxFailureTrace.TryPublish(event);
            }
        }
        m_TxLifecycle.ObserveApplicationOwnership(i);
    }
    observedSlot = m_TxLifecycle.BlockingSlot(currentIsRuntimeCyclic);
    protocolFence = observedSlot != NicTxFrameLifecycle::NoSlot;
    if (ownershipFaultSlot != NicTxFrameLifecycle::NoSlot ||
        protocolFence || availableSlot == NicTxFrameLifecycle::NoSlot)
    {
        // Even a free slot cannot pass an older unresolved/uncertain frame.
        // A pool-exhausted event identifies slot 1 as a concrete busy frame;
        // it does not claim that slot contains the globally previous call.
        if (ownershipFaultSlot != NicTxFrameLifecycle::NoSlot)
        {
            // INVALID_ADDRESS/NOT_READY/other errors are never hidden by a
            // spare frame, even when the bad slot has no recorded invocation.
            observedSlot = ownershipFaultSlot;
            protocolFence = false;
        }
        else if (!protocolFence) observedSlot = 0U;
        const DWORD error = ownershipError[observedSlot];
        call.error = static_cast<std::uint32_t>(error);
        call.errorValid = 1U;
        InterlockedIncrement64(&g_nicTxFrameBusy);
        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        if (error == ERROR_NOT_OWNER) InterlockedIncrement64(&g_nicTxOwnershipNotOwner);
        else if (error == ERROR_INVALID_ADDRESS) InterlockedIncrement64(&g_nicTxOwnershipInvalidAddress);
        else if (error == ERROR_NOT_READY) InterlockedIncrement64(&g_nicTxOwnershipNotReady);
        else InterlockedIncrement64(&g_nicTxOwnershipOther);
        SetLastError(error);
        return finish(NicTxReason::PreSubmitOwnershipRejected, false);
    }

    observedSlot = availableSlot;
    call.frameSlot = static_cast<std::uint32_t>(availableSlot + 1U);
    PRTNAL_FRAME frame = m_TxFrames[availableSlot];
    memcpy(frame->frameBufferVirtualAddr, pData, length);
    frame->frameSize = length;
    ULONG submitted = 0;
    call.nalInvoked = 1U;
    const BOOL result = RtNalTransmitEx(m_hTxQueue, &m_TxFrames[availableSlot], 1, &submitted);
    const DWORD nalThreadError = GetLastError();
    const BOOL nalReturnQpcResult = RtQueryPerformanceCounter(&nalReturnQpc);
    nalReturnQpcValid = nalReturnQpcResult && nalReturnQpc.QuadPart >= 0;
    SetLastError(nalThreadError);
    call.apiResult = static_cast<std::int32_t>(result);
    call.submitted = static_cast<std::uint32_t>(submitted);

    // NAL failure/count mismatch remains possibly submitted and quarantined
    // until actual ownership return. A response cannot clear uncertainty.
    if (result != TRUE)
    {
        const DWORD error = GetLastError();
        call.error = static_cast<std::uint32_t>(error);
        call.errorValid = submitted == 0 ? 1U : 0U;
        InterlockedIncrement64(&g_nicTxSubmitFail);
        if (submitted == 0) InterlockedIncrement64(&g_nicTxSubmitted0);
        if (error == ERROR_NOT_OWNER) InterlockedIncrement64(&g_nicTxNotOwner);
        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        return finish(NicTxReason::NalCallFailed, false);
    }
    if (submitted != 1)
    {
        const DWORD error = submitted == 0 ? GetLastError() : ERROR_SUCCESS;
        call.error = static_cast<std::uint32_t>(error);
        call.errorValid = submitted == 0 ? 1U : 0U;
        InterlockedIncrement64(&g_nicTxSubmitFail);
        if (submitted == 0) InterlockedIncrement64(&g_nicTxSubmitted0);
        InterlockedExchange(&g_nicTxLastError, static_cast<LONG>(error));
        return finish(NicTxReason::SubmittedCountMismatch, false);
    }
    InterlockedIncrement64(&g_nicTxSuccess);
    return finish(NicTxReason::Accepted, true);
}

bool CNicDriver::ObserveValidatedTxResponse(const NicTxReceipt& receipt) noexcept
{
    NicTxProducerScope producerScope(m_TxProducerActive);
    if (!producerScope.Held() || m_hTxQueue == NULL) return false;
    return m_TxLifecycle.ObserveValidatedResponse(receipt);
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

    // The same communications owner reads this immediately after ReceivePacket().
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
