#include "EDMProcessTuningService.h"
#include "EDMProcessTuningCodec.h"
#include "NCManager.h"
#include <windows.h>
#include <rtapi.h>
#include <cstring>
#include <cstdio>

namespace
{
    HANDLE g_mapping = NULL;
    EDMProcessTuningData* g_data = nullptr;
    std::uint64_t g_epoch = 0ULL;
    std::uint32_t g_heartbeat = 0U;
    EDMProcessTuningAck g_ack{};
    EDMProcessTuning::RequestHistory g_history{};
    LONG Exchange(std::uint32_t& word, std::uint32_t value) noexcept
    { return InterlockedExchange(reinterpret_cast<volatile LONG*>(&word), static_cast<LONG>(value)); }
    LONG CompareExchange(std::uint32_t& word, std::uint32_t value, std::uint32_t expected) noexcept
    { return InterlockedCompareExchange(reinterpret_cast<volatile LONG*>(&word), static_cast<LONG>(value), static_cast<LONG>(expected)); }
    std::uint32_t Read(std::uint32_t& word) noexcept
    { return static_cast<std::uint32_t>(CompareExchange(word, 0U, 0U)); }
    void Detail(const char* text) noexcept
    {
        std::memset(g_ack.Detail, 0, sizeof(g_ack.Detail));
        for (std::size_t i = 0; i + 1U < sizeof(g_ack.Detail) && text[i] != '\0'; ++i) g_ack.Detail[i] = text[i];
    }
    void Publish(NCManager& nc, bool complete) noexcept
    {
        EDMProcessTuningContext context{};
        EDMProcessTuningValues values{};
        nc.ReadEDMProcessTuningSameThread(context, values);
        const auto odd = (Read(g_data->Header.Sequence) + 1U) | 1U;
        Exchange(g_data->Header.Sequence, odd); MemoryBarrier();
        g_data->Header.Flags = EDM_PROCESS_TUNING_PUBLISHED;
        g_data->Header.Heartbeat = ++g_heartbeat;
        g_data->Context = context; g_data->Values = values; g_data->Ack = g_ack;
        MemoryBarrier(); Exchange(g_data->Header.Sequence, odd + 1U);
        if (complete) Exchange(g_data->Mailbox.State, static_cast<std::uint32_t>(EDMProcessTuningMailboxState::Complete));
    }
    const char* StatusName(std::uint32_t status) noexcept
    {
        switch (static_cast<EDMProcessTuningStatus>(status)) {
        case EDMProcessTuningStatus::None: return "NONE";
        case EDMProcessTuningStatus::Success: return "SUCCESS";
        case EDMProcessTuningStatus::Busy: return "BUSY";
        case EDMProcessTuningStatus::Stale: return "STALE";
        case EDMProcessTuningStatus::NotAllowed: return "NOT_ALLOWED";
        case EDMProcessTuningStatus::InvalidCommand: return "INVALID_COMMAND";
        case EDMProcessTuningStatus::DomainRejected: return "DOMAIN_REJECTED";
        case EDMProcessTuningStatus::Unavailable: return "UNAVAILABLE";
        default: return "UNKNOWN";
        }
    }
    void LogAck(const EDMProcessTuningRequest& request, bool replay) noexcept
    {
        // Publish has completed. ACK generation may be historical on replay;
        // currentGeneration/profileRevision describe the current NC profile.
        // One bounded record per processed request, never on heartbeat scans.
        // RTX64 RtPrintf receives only a string, with no floating conversions.
        char line[512]{};
        const int count = std::snprintf(line, sizeof(line),
            "[EDM21] event=TUNING_ACK session=%llu request=%llu epoch=%llu requestEpoch=%llu generation=%llu currentGeneration=%llu profileRevision=%u selection=%u status=%s statusCode=%u replay=%u runtimeOnly=1 physicalApplied=0 persisted=0\n",
            static_cast<unsigned long long>(g_ack.SessionId),
            static_cast<unsigned long long>(g_ack.RequestId),
            static_cast<unsigned long long>(g_ack.RuntimeEpoch),
            static_cast<unsigned long long>(request.RuntimeEpoch),
            static_cast<unsigned long long>(g_ack.Generation),
            static_cast<unsigned long long>(g_data->Context.Generation),
            static_cast<unsigned>(g_data->Context.ProfileRevision),
            static_cast<unsigned>(g_ack.SelectionMask), StatusName(g_ack.Status),
            static_cast<unsigned>(g_ack.Status), replay ? 1U : 0U);
        if (count >= 0 && static_cast<std::size_t>(count) < sizeof(line)) RtPrintf("%s", line);
        else RtPrintf("[EDM21] event=TUNING_ACK_FORMAT_ERROR runtimeOnly=1 physicalApplied=0 persisted=0\n");
    }
    void Ack(NCManager& nc, const EDMProcessTuningRequest& request,
        EDMProcessTuningStatus status, const char* detail, bool remember) noexcept
    {
        EDMProcessTuningContext current{}; EDMProcessTuningValues values{};
        nc.ReadEDMProcessTuningSameThread(current, values);
        g_ack = EDMProcessTuningAck{};
        g_ack.SessionId = request.SessionId; g_ack.RequestId = request.RequestId;
        g_ack.RuntimeEpoch = g_epoch; g_ack.Generation = current.Generation;
        g_ack.SelectionMask = request.SelectionMask; g_ack.Status = static_cast<std::uint32_t>(status);
        Detail(detail);
        if (remember) g_history.Remember(request, g_ack);
        Publish(nc, true);
        LogAck(request, false);
    }
}

bool InitializeEDMProcessTuningService() noexcept
{
    if (g_data != nullptr) return true;
    void* location = nullptr;
    g_mapping = RtCreateSharedMemory(PAGE_READWRITE, 0, EDM_PROCESS_TUNING_SIZE,
        L"OSCARMAX_EDM_PROCESS_TUNING", &location);
    if (g_mapping == NULL || location == nullptr) {
        if (g_mapping != NULL) RtCloseHandle(g_mapping);
        g_mapping = NULL;
        RtPrintf("[EDM20_FIX1] event=TUNING_SERVICE result=UNAVAILABLE\n");
        return false;
    }
    g_data = static_cast<EDMProcessTuningData*>(location);
    const std::uint64_t previous = g_data->Header.RuntimeEpoch;
    const auto odd = (Read(g_data->Header.Sequence) + 1U) | 1U;
    Exchange(g_data->Header.Sequence, odd); MemoryBarrier();
    LARGE_INTEGER counter{}; QueryPerformanceCounter(&counter);
    g_epoch = static_cast<std::uint64_t>(counter.QuadPart) ^ (static_cast<std::uint64_t>(GetCurrentProcessId()) << 32U);
    if (g_epoch == 0ULL || g_epoch == previous) g_epoch = previous + 1ULL;
    if (g_epoch == 0ULL) g_epoch = 1ULL;
    std::memset(g_data, 0, offsetof(EDMProcessTuningHeader, Sequence));
    std::memset(reinterpret_cast<unsigned char*>(g_data) + 16U, 0, EDM_PROCESS_TUNING_SIZE - 16U);
    g_data->Header.Magic = EDM_PROCESS_TUNING_MAGIC; g_data->Header.VersionMajor = 1U;
    g_data->Header.StructSize = EDM_PROCESS_TUNING_SIZE; g_data->Header.RuntimeEpoch = g_epoch;
    g_heartbeat = 0U; g_ack = EDMProcessTuningAck{}; g_history.Reset();
    MemoryBarrier(); Exchange(g_data->Header.Sequence, odd + 1U);
    RtPrintf("[EDM20_FIX1] event=TUNING_SERVICE result=READY runtimeOnly=1 physicalApplied=0\n");
    return true;
}

void ShutdownEDMProcessTuningService() noexcept
{
    // Existing SHM shutdown is called by this same NC owner; its loop exits
    // before any further ProcessTask invocation.
    if (g_data != nullptr) {
        const auto odd = (Read(g_data->Header.Sequence) + 1U) | 1U;
        Exchange(g_data->Header.Sequence, odd); MemoryBarrier();
        g_data->Header.Flags = EDM_PROCESS_TUNING_SHUTTING_DOWN;
        g_data->Context.Flags &= ~EDM_PROCESS_TUNING_CAN_APPLY;
        MemoryBarrier(); Exchange(g_data->Header.Sequence, odd + 1U);
    }
    g_data = nullptr;
    if (g_mapping != NULL) { RtCloseHandle(g_mapping); g_mapping = NULL; }
}

void ProcessEDMProcessTuningServiceSameThread(NCManager& nc) noexcept
{
    if (g_data == nullptr) return;
    const auto ready = static_cast<std::uint32_t>(EDMProcessTuningMailboxState::Ready);
    const auto processing = static_cast<std::uint32_t>(EDMProcessTuningMailboxState::Processing);
    if (static_cast<std::uint32_t>(CompareExchange(g_data->Mailbox.State, processing, ready)) != ready) {
        Publish(nc, false); return;
    }
    const std::uint32_t before = Read(g_data->Mailbox.RequestSequence);
    MemoryBarrier(); const EDMProcessTuningRequest request = g_data->Request; MemoryBarrier();
    const std::uint32_t after = Read(g_data->Mailbox.RequestSequence);
    if (before == 0U || (before & 1U) != 0U || before != after || !EDMProcessTuning::Canonical(request)) {
        Ack(nc, request, EDMProcessTuningStatus::InvalidCommand, "Malformed or torn request.", false); return;
    }
    if (request.RuntimeEpoch != g_epoch) {
        Ack(nc, request, EDMProcessTuningStatus::Stale, "Runtime restarted; read a new snapshot.", false); return;
    }
    const auto history = g_history.Inspect(request, g_ack);
    if (history == EDMProcessTuning::RequestHistory::Result::Duplicate) { Publish(nc, true); LogAck(request, true); return; }
    if (history == EDMProcessTuning::RequestHistory::Result::Reused) {
        Ack(nc, request, EDMProcessTuningStatus::Stale, "Request ID reused; read a new snapshot.", false); return;
    }
    const EDMProcessTuningStatus status = nc.ApplyEDMProcessTuningSameThread(request);
    Ack(nc, request, status, status == EDMProcessTuningStatus::Success ?
        "Applied to runtime shadow profile; disk files unchanged." : "Rejected; active profile unchanged.", true);
}
