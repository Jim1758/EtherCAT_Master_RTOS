#include "EDMConditionService.h"
#include "NCManager.h"
#include "EDMRecipeConfigIO.h"
#include "EDMConditionStartupIO.h"
#include "AlarmManager.h"
#include <windows.h>
#include <rtapi.h>
#include <atomic>
#include <cstring>
#include <cstdio>
#include <memory>

namespace
{
    // File I/O and catalog allocation/destruction are confined to priority 20.
    // Ownership is handed off with release/acquire, with one candidate slot.
    enum class WorkState : unsigned { Idle, Queued, Loading, Ready, Retire };
    HANDLE g_mapping = NULL, g_worker = NULL;
    EDMConditionData* g_data = nullptr;
    std::atomic<WorkState> g_work{WorkState::Idle};
    std::atomic<bool> g_stop{false}, g_workerAvailable{false}, g_workerExited{true};
    std::unique_ptr<EDMRecipe::Catalog> g_candidate;
    std::unique_ptr<const EDMRecipe::Catalog> g_retired;
    char g_directory[1024]{};
    EDMRecipeConfigIO::Diagnostic g_loadDiagnostic{};
    bool g_loadSucceeded = false;
    char g_loadDetail[88]{};
    EDMConditionRequest g_pending{};
    EDMConditionAck g_ack{};
    std::uint64_t g_epoch = 0ULL;
    std::uint32_t g_heartbeat = 0U;
    // All members are atomic: the bounded sequence read is race-free in C++ as
    // well as on x64. The NC owner only queues identity; the worker writes files.
    std::atomic<std::uint32_t> g_selectionSequence{0U}, g_selectionTable{0U};
    std::atomic<std::uint32_t> g_selectionSchema{0U}, g_selectionProfile{0U};
    std::atomic<std::uint32_t> g_selectionDefinition{0U}, g_selectionCatalog{0U};
    std::atomic<bool> g_selectionSaveFailed{false};
    std::uint32_t g_savedAttemptSequence = 0U, g_failedSelectionSequence = 0U, g_selectionRetryTicks = 0U; // worker-owned
    bool g_selectionFailureEpisode = false;
    EDMConditionStartupIO::Identity g_observedIdentity{}; // NC-owner only
    std::uint32_t g_observedTable = 0U;

    void QueueSelectionIdentity(const EDMConditionStartupIO::Identity& identity, std::uint32_t tableId, bool force) noexcept
    {
        if (!force && g_observedTable == tableId && EDMConditionStartupIO::SameIdentity(identity, g_observedIdentity)) return;
        g_observedTable = tableId; g_observedIdentity = identity;
        const std::uint32_t odd = (g_selectionSequence.load(std::memory_order_seq_cst) + 1U) | 1U;
        g_selectionSequence.store(odd, std::memory_order_seq_cst);
        g_selectionTable.store(tableId, std::memory_order_seq_cst);
        g_selectionSchema.store(identity.schema, std::memory_order_seq_cst);
        g_selectionProfile.store(identity.profile, std::memory_order_seq_cst);
        g_selectionDefinition.store(identity.definition, std::memory_order_seq_cst);
        g_selectionCatalog.store(identity.catalog, std::memory_order_seq_cst);
        g_selectionSequence.store(odd + 1U, std::memory_order_seq_cst);
    }
    void QueueLastSelection(NCManager& nc, const EDMRecipeControl::HmiContext& context, bool force = false) noexcept
    {
        if (context.tableId == 0U || context.eCode == 0U || !nc.IsEDMRecipeSelectionPersistenceAllowedSameThread()) return;
        EDMConditionStartupIO::Identity identity;
        identity.schema = context.schemaVersion; identity.profile = context.profileId;
        identity.definition = context.definitionRevision; identity.catalog = context.catalogRevision;
        QueueSelectionIdentity(identity, context.tableId, force);
    }
    void PersistLastSelection(bool finalAttempt = false) noexcept
    {
        const std::uint32_t before = g_selectionSequence.load(std::memory_order_seq_cst);
        if (before == 0U || before == g_savedAttemptSequence || (before & 1U) != 0U) return;
        if (!finalAttempt && before == g_failedSelectionSequence && g_selectionRetryTicks > 0U) { --g_selectionRetryTicks; return; }
        const std::uint32_t table = g_selectionTable.load(std::memory_order_seq_cst);
        EDMConditionStartupIO::Identity identity;
        identity.schema = g_selectionSchema.load(std::memory_order_seq_cst);
        identity.profile = g_selectionProfile.load(std::memory_order_seq_cst);
        identity.definition = g_selectionDefinition.load(std::memory_order_seq_cst);
        identity.catalog = g_selectionCatalog.load(std::memory_order_seq_cst);
        if (before != g_selectionSequence.load(std::memory_order_seq_cst)) return;
        if (!EDMConditionStartupIO::SaveLastSelection(g_directory, identity, table)) {
            g_failedSelectionSequence = before; g_selectionRetryTicks = 100U;
            if (!g_selectionFailureEpisode) {
                g_selectionFailureEpisode = true; g_selectionSaveFailed.store(true, std::memory_order_release);
                RtPrintf("[EDM19_FIX4] event=LAST_COND_SAVE result=FAILED table=%u priorValidSlotRetained=1 retry=BACKGROUND\n", static_cast<unsigned>(table));
            }
        } else {
            g_savedAttemptSequence = before; g_failedSelectionSequence = 0U; g_selectionRetryTicks = 0U; g_selectionFailureEpisode = false;
            RtPrintf("[EDM19_FIX4] event=LAST_COND_SAVE result=READY table=%u restartE=1\n", static_cast<unsigned>(table));
        }
    }

    LONG Exchange(std::uint32_t& location, std::uint32_t value) noexcept
    { return InterlockedExchange(reinterpret_cast<volatile LONG*>(&location), static_cast<LONG>(value)); }
    LONG CompareExchange(std::uint32_t& location, std::uint32_t value, std::uint32_t expected) noexcept
    { return InterlockedCompareExchange(reinterpret_cast<volatile LONG*>(&location), static_cast<LONG>(value), static_cast<LONG>(expected)); }
    std::uint32_t ReadAtomic(std::uint32_t& location) noexcept
    { return static_cast<std::uint32_t>(CompareExchange(location, 0U, 0U)); }
    void CopyText(char* target, std::size_t capacity, const char* source) noexcept
    {
        std::memset(target, 0, capacity);
        if (source == nullptr || capacity == 0U) return;
        std::size_t n = 0U;
        while (n + 1U < capacity && source[n] != '\0') { target[n] = source[n]; ++n; }
    }
    EDMConditionContext EncodeContext(const EDMRecipeControl::HmiContext& context) noexcept
    {
        EDMConditionContext result{};
        result.Generation = context.generation; result.ControlGeneration = context.controlGeneration;
        result.ProfileId = context.profileId; result.DefinitionRevision = context.definitionRevision;
        result.CatalogRevision = context.catalogRevision; result.TableRevision = context.tableRevision;
        result.SchemaVersion = context.schemaVersion; result.TableId = context.tableId; result.ECode = context.eCode;
        result.Flags = static_cast<std::uint16_t>((context.canEdit ? 1U : 0U) |
            (context.canReload && g_workerAvailable.load(std::memory_order_acquire) &&
                g_work.load(std::memory_order_acquire) == WorkState::Idle ? 2U : 0U));
        return result;
    }
    EDMRecipeControl::HmiContext DecodeContext(const EDMConditionContext& context) noexcept
    {
        EDMRecipeControl::HmiContext result{};
        result.generation = context.Generation; result.controlGeneration = context.ControlGeneration;
        result.profileId = context.ProfileId; result.definitionRevision = context.DefinitionRevision;
        result.catalogRevision = context.CatalogRevision; result.tableRevision = context.TableRevision;
        result.schemaVersion = context.SchemaVersion; result.tableId = context.TableId; result.eCode = context.ECode;
        result.canEdit = (context.Flags & 1U) != 0U; result.canReload = (context.Flags & 2U) != 0U;
        return result;
    }
    EDMRecipeControl::HmiCommand DecodeCommand(const EDMConditionRequest& request) noexcept
    {
        EDMRecipeControl::HmiCommand result{};
        result.operation = static_cast<EDMRecipeControl::HmiOperation>(request.Operation);
        result.tableId = request.TableId; result.eCode = request.ECode; result.fieldId = request.FieldId;
        result.stageId = request.StageId; result.step = request.Step; result.actualValue = request.ActualValue;
        return result;
    }
    bool Canonical(const EDMConditionRequest& request) noexcept
    {
        if (request.SessionId == 0ULL || request.RequestId == 0ULL || request.Operation < 1U ||
            request.Operation > 7U || request.Reserved16 != 0U || request.Reserved32 != 0U ||
            request.Reserved64 != 0ULL || (request.Expected.Flags & ~3U) != 0U) return false;
        for (const auto value : request.Expected.Reserved) if (value != 0U) return false;
        return EDMRecipeControl::IsHmiCommandCanonical(DecodeCommand(request));
    }
    EDMConditionAckStatus TranslateStatus(EDMRecipeControl::HmiStatus status) noexcept
    {
        switch (status)
        {
        case EDMRecipeControl::HmiStatus::Success: return EDMConditionAckStatus::Success;
        case EDMRecipeControl::HmiStatus::Busy: return EDMConditionAckStatus::Busy;
        case EDMRecipeControl::HmiStatus::Stale: return EDMConditionAckStatus::Stale;
        case EDMRecipeControl::HmiStatus::NotAllowed: return EDMConditionAckStatus::NotAllowed;
        case EDMRecipeControl::HmiStatus::InvalidCommand: return EDMConditionAckStatus::InvalidCommand;
        case EDMRecipeControl::HmiStatus::DomainRejected: return EDMConditionAckStatus::DomainRejected;
        }
        return EDMConditionAckStatus::InvalidCommand;
    }
    void Publish(NCManager& nc, bool complete) noexcept
    {
        EDMRecipeControl::HmiContext current{};
        nc.ReadEDMRecipeHmiContextSameThread(current);
        QueueLastSelection(nc, current);
        const EDMConditionContext encoded = EncodeContext(current);
        const auto writing = (ReadAtomic(g_data->Header.Sequence) + 1U) | 1U;
        Exchange(g_data->Header.Sequence, writing);
        MemoryBarrier();
        g_data->Header.Heartbeat = ++g_heartbeat;
        g_data->Header.Flags = EDM_CONDITION_PUBLISHED |
            (g_workerAvailable.load(std::memory_order_acquire) ? EDM_CONDITION_WORKER_AVAILABLE : 0U);
        g_data->Context = encoded;
        g_data->Ack = g_ack;
        MemoryBarrier();
        Exchange(g_data->Header.Sequence, writing + 1U);
        if (complete) Exchange(g_data->Mailbox.State, static_cast<std::uint32_t>(EDMConditionMailboxState::Complete));
    }
    void Acknowledge(NCManager& nc, const EDMConditionRequest& request,
        EDMConditionAckStatus status, const char* detail, std::uint32_t domainError = 0U,
        std::uint32_t errorLine = 0U) noexcept
    {
        EDMRecipeControl::HmiContext current{};
        nc.ReadEDMRecipeHmiContextSameThread(current);
        if (status == EDMConditionAckStatus::Success && (request.Operation == 1U || request.Operation == 7U)) QueueLastSelection(nc, current, true);
        g_ack = EDMConditionAck{};
        g_ack.SessionId = request.SessionId; g_ack.RequestId = request.RequestId; g_ack.RuntimeEpoch = g_epoch;
        g_ack.After = EncodeContext(current); g_ack.Status = static_cast<std::uint32_t>(status);
        g_ack.DomainError = domainError; g_ack.ErrorLine = errorLine;
        CopyText(g_ack.Detail, sizeof(g_ack.Detail), detail);
        Publish(nc, true);
    }
    DWORD WINAPI ReloadWorker(void*)
    {
        if (!RtSetThreadPriority(GetCurrentThread(), 20))
        { g_selectionSaveFailed.store(true, std::memory_order_release); g_workerExited.store(true, std::memory_order_release); return 1U; }
        g_workerAvailable.store(true, std::memory_order_release);
        while (!g_stop.load(std::memory_order_acquire))
        {
            const WorkState state = g_work.load(std::memory_order_acquire);
            if (state == WorkState::Queued)
            {
                g_work.store(WorkState::Loading, std::memory_order_release);
                g_candidate.reset(); g_retired.reset();
                g_loadDiagnostic = EDMRecipeConfigIO::Diagnostic{};
                g_loadSucceeded = EDMRecipeConfigIO::LoadDirectory(g_directory, g_candidate, g_loadDiagnostic);
                if (g_loadSucceeded) CopyText(g_loadDetail, sizeof(g_loadDetail), "Catalog validated; awaiting owner acceptance.");
                else
                {
                    std::snprintf(g_loadDetail, sizeof(g_loadDetail), "%s: %.40s / %.20s",
                        EDMRecipeConfigIO::ErrorName(g_loadDiagnostic.error), g_loadDiagnostic.file, g_loadDiagnostic.key);
                    g_loadDetail[sizeof(g_loadDetail) - 1U] = '\0';
                }
                g_work.store(WorkState::Ready, std::memory_order_release);
            }
            else if (state == WorkState::Retire)
            {
                // Also disposes a rejected candidate; never runs on NC/PDO.
                g_candidate.reset(); g_retired.reset();
                g_work.store(WorkState::Idle, std::memory_order_release);
            }
            PersistLastSelection();
            RtSleep(10U);
        }
        PersistLastSelection(true); // normal shutdown drains the latest queued table identity.
        g_candidate.reset(); g_retired.reset();
        g_workerAvailable.store(false, std::memory_order_release);
        g_workerExited.store(true, std::memory_order_release);
        return 0U;
    }
}

void RememberEDMConditionSelectionSameThread(const EDMRecipe::Catalog& catalog, const EDMRecipe::Summary& summary) noexcept
{
    if (g_data == nullptr || g_stop.load(std::memory_order_acquire) || !summary.tableSelected || summary.tableId == 0U) return;
    // Caller supplies the committed values directly; reading HMI context here
    // would revalidate Close/RESET and could erase the just-committed selection.
    QueueSelectionIdentity(EDMConditionStartupIO::GetIdentity(catalog), summary.tableId, true);
}

EDMConditionData* GetEDMConditionSharedMemoryData() noexcept { return g_data; }

bool InitializeEDMConditionService(const char* recipeDirectory) noexcept
{
    if (g_data != nullptr) return true;
    if (recipeDirectory == nullptr || std::strlen(recipeDirectory) >= sizeof(g_directory)) return false;
    CopyText(g_directory, sizeof(g_directory), recipeDirectory);
    void* location = nullptr;
    g_mapping = RtCreateSharedMemory(PAGE_READWRITE, 0, EDM_CONDITION_SIZE,
        L"OSCARMAX_EDM_CONDITION_CONTROL", &location);
    if (g_mapping == NULL || location == nullptr)
    { if (g_mapping != NULL) RtCloseHandle(g_mapping); g_mapping = NULL; return false; }
    g_data = static_cast<EDMConditionData*>(location);
    const std::uint64_t previousEpoch = g_data->Header.RuntimeEpoch;
    const auto writing = (ReadAtomic(g_data->Header.Sequence) + 1U) | 1U;
    Exchange(g_data->Header.Sequence, writing);
    MemoryBarrier();
    LARGE_INTEGER counter{};
    QueryPerformanceCounter(&counter);
    g_epoch = static_cast<std::uint64_t>(counter.QuadPart) ^ (static_cast<std::uint64_t>(GetCurrentProcessId()) << 32U);
    if (g_epoch == 0ULL || g_epoch == previousEpoch) g_epoch = previousEpoch + 1ULL;
    if (g_epoch == 0ULL) g_epoch = 1ULL;
    // Preserve the odd publication marker while clearing a retained mapping.
    std::memset(g_data, 0, offsetof(EDMConditionHeader, Sequence));
    std::memset(reinterpret_cast<unsigned char*>(g_data) + 16U, 0, EDM_CONDITION_SIZE - 16U);
    g_data->Header.Magic = EDM_CONDITION_MAGIC; g_data->Header.VersionMajor = 1U;
    g_data->Header.StructSize = EDM_CONDITION_SIZE; g_data->Header.RuntimeEpoch = g_epoch;
    g_ack = EDMConditionAck{}; g_pending = EDMConditionRequest{}; g_heartbeat = 0U;
    g_selectionSequence.store(0U); g_savedAttemptSequence = 0U;
    g_failedSelectionSequence = 0U; g_selectionRetryTicks = 0U; g_selectionFailureEpisode = false;
    g_observedTable = 0U; g_observedIdentity = EDMConditionStartupIO::Identity{};
    g_selectionSaveFailed.store(false);
    g_stop.store(false, std::memory_order_release); g_work.store(WorkState::Idle, std::memory_order_release);
    g_workerAvailable.store(false, std::memory_order_release); g_workerExited.store(false, std::memory_order_release);
    MemoryBarrier();
    Exchange(g_data->Header.Sequence, writing + 1U);
    g_worker = CreateThread(nullptr, 0U, ReloadWorker, nullptr, 0U, nullptr);
    if (g_worker == NULL) { g_workerExited.store(true, std::memory_order_release); g_selectionSaveFailed.store(true, std::memory_order_release); }
    return true; // Selection/edit commands remain available if the worker fails.
}

void ShutdownEDMConditionService() noexcept
{
    if (g_data != nullptr)
    {
        const auto writing = (ReadAtomic(g_data->Header.Sequence) + 1U) | 1U;
        Exchange(g_data->Header.Sequence, writing);
        MemoryBarrier();
        g_data->Header.Flags = EDM_CONDITION_SHUTTING_DOWN;
        g_data->Context.Flags = 0U;
        MemoryBarrier(); Exchange(g_data->Header.Sequence, writing + 1U);
    }
    g_stop.store(true, std::memory_order_release);
    // This is system shutdown, outside the periodic NC task. Do not destroy
    // catalog storage until the file worker has relinquished it.
    while (!g_workerExited.load(std::memory_order_acquire)) RtSleep(1U);
    if (g_worker != NULL) { CloseHandle(g_worker); g_worker = NULL; }
    g_data = nullptr;
    if (g_mapping != NULL) { RtCloseHandle(g_mapping); g_mapping = NULL; }
}

void ProcessEDMConditionServiceSameThread(NCManager& nc) noexcept
{
    if (g_data == nullptr || g_stop.load(std::memory_order_acquire)) return;
    if (g_selectionSaveFailed.exchange(false, std::memory_order_acq_rel))
        AlarmManager::GetInstance().Trigger(AlarmManager::EDM_CONDITION_SELECTION_SAVE_FAILED);
    if (g_work.load(std::memory_order_acquire) == WorkState::Ready)
    {
        if (!g_loadSucceeded)
        {
            Acknowledge(nc, g_pending, EDMConditionAckStatus::LoadFailure, g_loadDetail,
                static_cast<std::uint32_t>(g_loadDiagnostic.error), static_cast<std::uint32_t>(g_loadDiagnostic.line));
        }
        else
        {
            EDMRecipeControl::HmiResult result{};
            nc.ReloadValidatedEDMRecipeCatalogSameThread(DecodeContext(g_pending.Expected), g_candidate, g_retired, result);
            Acknowledge(nc, g_pending, TranslateStatus(result.status),
                result.status == EDMRecipeControl::HmiStatus::Success ? "Catalog reloaded; COND and E1 restored." : "Reload rejected; active catalog retained.",
                static_cast<std::uint32_t>(result.domainError));
        }
        g_work.store(WorkState::Retire, std::memory_order_release);
        return;
    }
    const auto ready = static_cast<std::uint32_t>(EDMConditionMailboxState::Ready);
    const auto processing = static_cast<std::uint32_t>(EDMConditionMailboxState::Processing);
    if (static_cast<std::uint32_t>(CompareExchange(g_data->Mailbox.State, processing, ready)) != ready)
    { Publish(nc, false); return; }
    const std::uint32_t before = ReadAtomic(g_data->Mailbox.RequestSequence);
    MemoryBarrier();
    const EDMConditionRequest request = g_data->Request;
    MemoryBarrier();
    const std::uint32_t after = ReadAtomic(g_data->Mailbox.RequestSequence);
    if (before == 0U || (before & 1U) != 0U || before != after || !Canonical(request))
    { Acknowledge(nc, request, EDMConditionAckStatus::InvalidCommand, "Malformed or torn request."); return; }
    if (request.RuntimeEpoch != g_epoch)
    { Acknowledge(nc, request, EDMConditionAckStatus::Stale, "Runtime restarted; read current context."); return; }
    if (request.Operation == 7U)
    {
        EDMRecipeControl::HmiContext current{};
        nc.ReadEDMRecipeHmiContextSameThread(current);
        if (!EDMRecipeControl::SameHmiKey(DecodeContext(request.Expected), current))
        { Acknowledge(nc, request, EDMConditionAckStatus::Stale, "Context changed; read current context."); return; }
        if (!current.canReload)
        { Acknowledge(nc, request, EDMConditionAckStatus::NotAllowed, "Reload requires idle and drained axes."); return; }
        if (!g_workerAvailable.load(std::memory_order_acquire))
        { Acknowledge(nc, request, EDMConditionAckStatus::Unavailable, "Reload worker unavailable."); return; }
        if (g_work.load(std::memory_order_acquire) != WorkState::Idle)
        { Acknowledge(nc, request, EDMConditionAckStatus::Busy, "Previous catalog is still being retired."); return; }
        g_pending = request;
        g_work.store(WorkState::Queued, std::memory_order_release);
        Publish(nc, false); // Processing remains owned until validation + owner swap.
        return;
    }
    EDMRecipeControl::HmiResult result{};
    nc.ApplyEDMRecipeHmiCommandSameThread(DecodeContext(request.Expected), DecodeCommand(request), result);
    Acknowledge(nc, request, TranslateStatus(result.status),
        result.status == EDMRecipeControl::HmiStatus::Success ? "Condition applied to shadow state." :
            (result.status == EDMRecipeControl::HmiStatus::DomainRejected ? EDMRecipe::ErrorName(result.domainError) : "Condition request rejected."),
        static_cast<std::uint32_t>(result.domainError));
}
